"""Independent Gazebo truth/contact observer for scoring only.

This observer is deliberately separate from the controller and camera
recorder. It subscribes to truth and collision topics, writes evidence files,
and never publishes flight, MAVLink, or estimator messages.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
from threading import Event, Lock
import time

TRUTH_TOPIC = '/world/dronehacks/dynamic_pose/info'
CONTACT_TOPICS = tuple(
    f'/model/course_quad/contact/{link}'
    for link in ('base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3')
)
MAX_WALL_SECONDS = 3600.0


def _xyz(vector):
    return (float(vector.x), float(vector.y), float(vector.z))


def _quaternion_xyzw(value):
    q = (float(value.x), float(value.y), float(value.z), float(value.w))
    norm = math.sqrt(sum(x*x for x in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid pose quaternion')
    return tuple(x / norm for x in q)


def rotate_xyzw(quaternion, vector):
    """Rotate a vector by an xyzw quaternion, returning ENU coordinates."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    return (
        (1 - 2*(y*y + z*z))*vx + 2*(x*y - z*w)*vy + 2*(x*z + y*w)*vz,
        2*(x*y + z*w)*vx + (1 - 2*(x*x + z*z))*vy + 2*(y*z - x*w)*vz,
        2*(x*z - y*w)*vx + 2*(y*z + x*w)*vy + (1 - 2*(x*x + y*y))*vz,
    )


def multiply_xyzw(first, second):
    """Quaternion composition: result rotates by second, then first."""
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    result = (aw*bx + ax*bw + ay*bz - az*by,
              aw*by - ax*bz + ay*bw + az*bx,
              aw*bz + ax*by - ay*bx + az*bw,
              aw*bw - ax*bx - ay*by - az*bz)
    norm = math.sqrt(sum(v*v for v in result))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid composed quaternion')
    return tuple(v / norm for v in result)


def base_link_pose(model_pose, link_pose):
    """Transform a link-local pose into world ENU using model pose."""
    model_q = _quaternion_xyzw(model_pose.orientation)
    link_q = _quaternion_xyzw(link_pose.orientation)
    offset = rotate_xyzw(model_q, _xyz(link_pose.position))
    model_p = _xyz(model_pose.position)
    position = tuple(p + d for p, d in zip(model_p, offset))
    orientation = multiply_xyzw(model_q, link_q)
    return position, orientation


def contact_topic_audit(node, topics=CONTACT_TOPICS):
    """Return transport discovery counts; discovery is not a sensor health test."""
    discovered = set(node.topic_list())
    result = {}
    for topic in topics:
        publishers = None
        if topic in discovered:
            try:
                info = node.topic_info(topic)
                # gz.transport Node.topic_info returns (publishers, subscribers).
                publishers = len(info[0]) if info is not None else None
            except Exception:
                publishers = None
        result[topic] = publishers
    return result


def contact_topic_interpretation(publisher_start, publisher_end, messages):
    """Describe observed coverage without treating advertisement as health."""
    if messages:
        return 'messages_received'
    if publisher_start or publisher_end:
        return 'publisher_advertised_no_messages_observed'
    return 'no_publisher_observed'


class ObserverLog:
    def __init__(self, output):
        self.path = Path(output)
        self.path.mkdir(parents=True, exist_ok=False)
        self.truth = (self.path / 'truth.jsonl').open('x', buffering=1)
        self.contacts = (self.path / 'contacts.jsonl').open('x', buffering=1)
        self.truth_samples = 0
        self.contact_messages = 0
        self.contact_pairs = 0
        self.contact_topic_messages = {topic: 0 for topic in CONTACT_TOPICS}
        self.contact_topic_pairs = {topic: 0 for topic in CONTACT_TOPICS}
        self.first_sim_s = None
        self.last_sim_s = None
        self.subscribed_truth = False
        self.contact_subscribed = []
        self.contact_unavailable = []
        self.contact_publishers_start = {}
        self.contact_publishers_end = {}

    def add_truth(self, stamp_ns, model_pose, link_pose):
        position, orientation = base_link_pose(model_pose, link_pose)
        stamp_s = int(stamp_ns) * 1e-9
        self.truth.write(json.dumps({
            'sim_time_s': stamp_s,
            'base_link_position_enu_m': position,
            'base_link_orientation_xyzw': orientation,
            'source': 'Gazebo dynamic_pose/info; evaluation only',
        }, separators=(',', ':')) + '\n')
        self.truth_samples += 1
        if self.first_sim_s is None:
            self.first_sim_s = stamp_s
        self.last_sim_s = stamp_s

    def add_contacts(self, topic, stamp_ns, pairs):
        if not pairs:
            return
        stamp_s = int(stamp_ns) * 1e-9
        self.contacts.write(json.dumps({
            'sim_time_s': stamp_s,
            'topic': topic,
            'pairs': pairs,
            'source': 'Gazebo contact sensor; raw observations only',
        }, separators=(',', ':')) + '\n')
        self.contact_messages += 1
        self.contact_pairs += len(pairs)

    def status(self, state, reason=None):
        data = {
            'state': state,
            'reason': reason,
            'truth_topic': TRUTH_TOPIC,
            'truth_subscribed': self.subscribed_truth,
            'truth_samples': self.truth_samples,
            'contacts_expected_topics': list(CONTACT_TOPICS),
            'contacts_subscribed_topics': self.contact_subscribed,
            'contacts_unavailable_topics': self.contact_unavailable,
            'contact_topic_publishers_at_start': self.contact_publishers_start,
            'contact_topic_publishers_at_end': self.contact_publishers_end,
            'contact_topic_observation': {
                topic: {
                    'publisher_count_start': self.contact_publishers_start.get(topic),
                    'publisher_count_end': self.contact_publishers_end.get(topic),
                    'transport_messages_received': self.contact_topic_messages[topic],
                    'messages_with_contact_pairs': self.contact_topic_pairs[topic],
                    'interpretation': contact_topic_interpretation(
                        self.contact_publishers_start.get(topic),
                        self.contact_publishers_end.get(topic),
                        self.contact_topic_messages[topic]),
                } for topic in CONTACT_TOPICS
            },
            'contacts_topics_with_messages': [topic for topic, count in
                                              self.contact_topic_messages.items() if count],
            'contacts_topics_without_messages': [topic for topic, count in
                                                 self.contact_topic_messages.items() if not count],
            'contact_messages_with_pairs': self.contact_messages,
            'raw_contact_pair_observations': self.contact_pairs,
            'first_sim_time_s': self.first_sim_s,
            'last_sim_time_s': self.last_sim_s,
            'truth_file': 'truth.jsonl',
            'contacts_file': 'contacts.jsonl',
            'computed_mission_score': False,
            'includes_latest_pose': False,
        }
        temp = self.path / 'status.json.tmp'
        temp.write_text(json.dumps(data, indent=2) + '\n')
        os.replace(temp, self.path / 'status.json')

    def close(self, state, reason=None):
        self.truth.close()
        self.contacts.close()
        self.status(state, reason)


def main():
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='new output directory; it must not already exist')
    parser.add_argument('--duration', type=float, default=900,
                        help='wall-clock observation duration, at most 3600 seconds')
    args = parser.parse_args()
    if not 0 < args.duration <= MAX_WALL_SECONDS:
        parser.error('duration must be between 0 and 3600 wall seconds')

    log = ObserverLog(args.output)
    node, lock, poses, contacts = Node(), Lock(), deque(maxlen=4), deque(maxlen=256)
    stop = Event()
    old_handlers = {}
    for sig in (signal.SIGTERM, signal.SIGINT):
        old_handlers[sig] = signal.signal(sig, lambda *_: stop.set())

    def receive_truth(message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
        with lock:
            poses.append((stamp, message))

    def receive_contact(topic, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nsec
        pairs = [(c.collision1.name, c.collision2.name) for c in message.contact]
        with lock:
            log.contact_topic_messages[topic] += 1
            if pairs:
                log.contact_topic_pairs[topic] += 1
                contacts.append((topic, stamp, pairs))

    subscribed = []
    state, reason = 'observing', None
    last_stamp_ns, started = -1, time.monotonic()
    try:
        log.subscribed_truth = bool(node.subscribe(Pose_V, TRUTH_TOPIC, receive_truth))
        if log.subscribed_truth:
            subscribed.append(TRUTH_TOPIC)
        else:
            reason = f'cannot subscribe to {TRUTH_TOPIC}'
        for topic in CONTACT_TOPICS:
            if node.subscribe(Contacts, topic,
                              lambda msg, topic=topic: receive_contact(topic, msg)):
                subscribed.append(topic)
                log.contact_subscribed.append(topic)
            else:
                log.contact_unavailable.append(topic)
        # Discovery is only evidence that Gazebo Transport currently advertises
        # a publisher. Contact sensors may be silent until contact occurs.
        log.contact_publishers_start = contact_topic_audit(node)
        deadline = started + args.duration
        last_status = started
        while not stop.is_set() and time.monotonic() < deadline:
            with lock:
                pose_batch, contact_batch = list(poses), list(contacts)
                poses.clear()
                contacts.clear()
            for stamp, message in pose_batch:
                if stamp <= last_stamp_ns:
                    continue
                by_name = {pose.name: pose for pose in message.pose}
                model, link = by_name.get('course_quad'), by_name.get('base_link')
                if model is None or link is None:
                    continue
                log.add_truth(stamp, model, link)
                last_stamp_ns = stamp
            for topic, stamp, pairs in contact_batch:
                log.add_contacts(topic, stamp, pairs)
            if time.monotonic() - last_status >= 1:
                log.status(state, reason)
                last_status = time.monotonic()
            time.sleep(.005)
        if stop.is_set():
            state = 'stopped'
        else:
            state = 'duration_complete'
        if log.truth_samples == 0:
            reason = reason or 'truth topic subscribed but no course_quad/base_link samples received'
    except KeyboardInterrupt:
        state = 'stopped'
    except BaseException as error:
        state, reason = 'error', f'{type(error).__name__}: {error}'
        raise
    finally:
        log.contact_publishers_end = contact_topic_audit(node)
        for topic in subscribed:
            node.unsubscribe(topic)
        log.close(state, reason)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    print(json.dumps({'output': str(args.output), 'truth_samples': log.truth_samples,
                      'raw_contact_pair_observations': log.contact_pairs,
                      'state': state, 'reason': reason,
                      'topics_subscribed': subscribed,
                      'computed_mission_score': False}))


if __name__ == '__main__':
    main()
