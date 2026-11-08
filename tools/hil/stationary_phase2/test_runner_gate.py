"""Local fail-closed evidence tests; no ROS, Docker or hardware execution."""
import unittest
import gzip
import hashlib
import io
import json
import tarfile
from finish_phase2 import verify_export, ROOT
from check_test_evidence import REQUIRED, check_collection, check_tests, check_isolation

COLLECTION = '\n'.join('test/test_ros_' + ('graph_dry_run.py::TestDryRunGraph::' if 'commands' in n else
                    'perception_dry_run.py::TestPerceptionContracts::') + n for n in sorted(REQUIRED))
JUNIT = '<testsuite>' + ''.join('<testcase name="' + n + '"/>' for n in sorted(REQUIRED)) + '</testsuite>'
COMMAND = '/usr/bin/python3 -m pytest --junit-xml=/audit_ws/build/dh_autonomy/pytest.xml'


class EvidenceTests(unittest.TestCase):
    def test_required_execution(self):
        self.assertEqual(check_tests(COLLECTION, JUNIT, COMMAND), 2)
        check_isolation({'executed': True, 'returncode': 0}, 'RESULT: PASS dh_mission /dh/dry_run/cmd_vel')

    def test_collection_zero_or_missing(self):
        for text in ('', COLLECTION.splitlines()[0], 'collected 0 tests', str(REQUIRED)):
            with self.assertRaises(RuntimeError): check_collection(text)

    def test_junit_missing_zero_or_required_missing(self):
        for xml in ('<testsuite/>', '<testsuite><testcase name="other"/></testsuite>'):
            with self.assertRaises(RuntimeError): check_tests(COLLECTION, xml, COMMAND)
        with self.assertRaises(Exception): check_tests(COLLECTION, '', COMMAND)

    def test_skipped_failed_or_error(self):
        for tag in ('skipped', 'failure', 'error'):
            xml = JUNIT.replace('/>', '><' + tag + '/></testcase>', 1)
            with self.assertRaises(RuntimeError): check_tests(COLLECTION, xml, COMMAND)

    def test_actual_runner_wrong(self):
        for command in ('/usr/bin/python3 -m unittest -v', '', 'pytest noted, but not executed'):
            with self.assertRaises(RuntimeError): check_tests(COLLECTION, JUNIT, command)

    def test_isolation_never_executed_or_failed(self):
        for marker in ({}, {'executed': False, 'returncode': 0}, {'executed': True, 'returncode': 1}):
            with self.assertRaises(RuntimeError): check_isolation(marker, 'RESULT: PASS dh_mission /dh/dry_run/cmd_vel')
        with self.assertRaises(RuntimeError): check_isolation({'executed': True, 'returncode': 0}, '')


class ExportTests(unittest.TestCase):
    def fixture(self, name='result.log'):
        cid, pin = 'a' * 64, 'b' * 40
        data = b'actual result\n'
        manifest = dict(canonical_root=ROOT, container_id=cid, source_commit=pin,
                        entries=[dict(path=name, uid=1000, type='file', size=len(data),
                                      sha256=hashlib.sha256(data).hexdigest())])
        mbytes = json.dumps(manifest).encode(); raw = io.BytesIO()
        with tarfile.open(fileobj=raw, mode='w') as t:
            for filename, content in ((name, data), ('export-manifest.json', mbytes)):
                info = tarfile.TarInfo(filename); info.size=len(content); info.uid=1000
                t.addfile(info, io.BytesIO(content))
        packed = gzip.compress(raw.getvalue())
        proof = dict(archive_sha256=hashlib.sha256(packed).hexdigest(), manifest_sha256=hashlib.sha256(mbytes).hexdigest())
        return packed, proof, cid, pin

    def test_complete_readable_export(self):
        self.assertEqual(len(verify_export(*self.fixture())['entries']), 1)

    def test_corrupted_export_rejected(self):
        data, proof, cid, pin = self.fixture()
        with self.assertRaises(RuntimeError): verify_export(data+b'changed', proof, cid, pin)

    def test_unsafe_path_rejected(self):
        with self.assertRaises(RuntimeError): verify_export(*self.fixture('../outside'))

    def test_wrong_source_or_container_rejected(self):
        data, proof, cid, pin = self.fixture()
        with self.assertRaises(RuntimeError): verify_export(data, proof, 'c'*64, pin)
        with self.assertRaises(RuntimeError): verify_export(data, proof, cid, 'c'*40)


if __name__ == '__main__':
    unittest.main(verbosity=2)
