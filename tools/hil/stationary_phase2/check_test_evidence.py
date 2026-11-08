"""Fail closed on missing collection, real pytest execution or isolation evidence."""
import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as E

REQUIRED = {'test_commands_only_on_dry_run_topic', 'test_real_images_clearance_and_isolation'}
MODULES = ('test_ros_graph_dry_run.py', 'test_ros_perception_dry_run.py')


def check_collection(text):
    names = set()
    for line in text.splitlines():
        parts = line.strip().split('::')
        if len(parts) >= 3 and parts[0].endswith(MODULES):
            names.add(parts[-1].split('[')[0])
    if not REQUIRED <= names:
        raise RuntimeError('Zero/incomplete pytest collection: ' + str(sorted(names)))
    return sorted(names)


def check_tests(collection, junit, command):
    check_collection(collection)
    if not re.search(r'\bpython3(?:\.[0-9]+)?\s+-m\s+pytest\b', command) or '-m unittest' in command:
        raise RuntimeError('Actual Colcon command did not select pytest')
    root = E.fromstring(junit)
    cases = list(root.iter('testcase'))
    if not cases:
        raise RuntimeError('Missing/zero JUnit evidence')
    if any(c.find(tag) is not None for c in cases for tag in ('failure', 'error', 'skipped')):
        raise RuntimeError('Failed/errored/skipped integration evidence')
    names = {c.attrib.get('name') for c in cases}
    if not REQUIRED <= names:
        raise RuntimeError('Missing required JUnit cases: ' + str(sorted(names)))
    return len(cases)


def check_isolation(marker, text):
    if marker.get('executed') is not True or marker.get('returncode') != 0:
        raise RuntimeError('Isolation checker did not execute successfully')
    if not all(t in text for t in ('RESULT: PASS', 'dh_mission', '/dh/dry_run/cmd_vel')):
        raise RuntimeError('Missing positive isolation log')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--stage', choices=('collection', 'tests', 'complete'), required=True)
    a = p.parse_args()
    base = Path('/audit_ws')
    collected = (base / 'evidence/collection.log').read_text()
    check_collection(collected)
    if a.stage != 'collection':
        count = check_tests(collected, (base / 'build/dh_autonomy/pytest.xml').read_text(),
                            (base / 'log/latest_test/dh_autonomy/command.log').read_text())
        print('Required integration tests executed through actual Colcon pytest:', count)
    if a.stage == 'complete':
        check_isolation(json.loads((base / 'evidence/isolation-result.json').read_text()),
                        (base / 'evidence/isolation-final.log').read_text())
    print('EVIDENCE PASS:', a.stage)


if __name__ == '__main__':
    main()
