#!/usr/bin/env python3
"""READ-ONLY check that our nodes cannot command the flight controller (run on the Jetson, in the container).

It only queries the ROS graph (no publishers, no subscriptions, no service calls):
  * every publisher on /skybeta/* and /fmu/in/*, and the node that owns it;
  * every service client our nodes hold on /skybeta/* and /fmu/*;
  * that /dh/dry_run/cmd_vel is published by dh_mission (dry-run active).
Exit code 0 = PASS (no dh_* endpoint on the flight-control path), 1 = FAIL, 2 = our nodes are not running.

  python3 tools/hil/check_command_isolation.py [--wait 5]
"""
import argparse
import sys
import time

FORBIDDEN = ("/skybeta", "/fmu")


def forbidden(name: str) -> bool:
    return any(name == p or name.startswith(p + "/") for p in FORBIDDEN)


def result_code(ours, bad, dry):
    if bad:
        return 1
    if not any(name == "dh_mission" for name, _ in ours) or "dh_mission" not in dry:
        return 2
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=5.0, help="seconds to let graph discovery settle")
    args = ap.parse_args()
    import rclpy
    rclpy.init()
    node = rclpy.create_node("dh_isolation_check_readonly")
    t0 = time.time()
    while time.time() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.2)
    ours = [(n, ns) for n, ns in node.get_node_names_and_namespaces() if n.startswith("dh_") and n != node.get_name()]
    print("our nodes:", ours or "NONE")
    bad = []
    for name, ns in ours:
        for topic, types in node.get_publisher_names_and_types_by_node(name, ns):
            flag = "  <-- FORBIDDEN" if forbidden(topic) else ""
            print("  %s publishes %s %s%s" % (name, topic, types, flag))
            if flag:
                bad.append((name, "publisher", topic))
        for srv, types in node.get_client_names_and_types_by_node(name, ns):
            if forbidden(srv):
                print("  %s has a service client on %s  <-- FORBIDDEN in dry run" % (name, srv))
                bad.append((name, "client", srv))
    print("\nall publishers on the flight-control path (expected: only the organizers' nodes):")
    for topic, types in node.get_topic_names_and_types():
        if forbidden(topic) and (topic.startswith("/skybeta/setpoint") or topic.startswith("/fmu/in")):
            for info in node.get_publishers_info_by_topic(topic):
                print("  %-40s <- %s%s" % (topic, info.node_name, "  <-- OURS!" if info.node_name.startswith("dh_") else ""))
    dry = [i.node_name for i in node.get_publishers_info_by_topic("/dh/dry_run/cmd_vel")]
    print("\n/dh/dry_run/cmd_vel publishers:", dry or "none (mission not running or not in dry run)")
    node.destroy_node()
    rclpy.shutdown()
    result = result_code(ours, bad, dry)
    if result == 1:
        print("RESULT: FAIL - our nodes own flight-control endpoints: %s" % bad)
        sys.exit(1)
    if result == 2:
        print("RESULT: INCOMPLETE - require dh_mission and its dry-run command publisher; allow discovery to settle")
        sys.exit(2)
    print("RESULT: PASS - no dh_* publisher or client on /skybeta/* or /fmu/*")
    sys.exit(0)


if __name__ == "__main__":
    main()
