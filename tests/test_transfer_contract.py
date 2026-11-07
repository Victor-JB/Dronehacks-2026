import copy
import hashlib
from pathlib import Path
import tempfile
import unittest

from learning.inference import PolicyError, validate_transfer_contract
from learning.transfer import TransferCourse


class TransferContractTests(unittest.TestCase):
    def test_changed_route_or_heading_cannot_reuse_a_checkpoint(self):
        root = Path(__file__).resolve().parents[1]
        course = TransferCourse()
        profile = course.profile_metadata()
        contract = dict(profile='x500-transfer', route_sha256=profile['route_sha256'],
                        heading_sha256=hashlib.sha256((root/'mission/heading.py').read_bytes()).hexdigest(),
                        transfer_profile=profile)
        source_paths = (course.route_path, root/'mission/heading.py', root/'models/flight_x500_base/model.sdf')
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory)/str(i) for i in range(3)]
            for source, target in zip(source_paths, paths):
                target.write_bytes(source.read_bytes())
            self.assertEqual(validate_transfer_contract(contract, *paths, speed_scale=1., lookahead=1.8), profile)
            for index in range(3):
                original = paths[index].read_bytes()
                paths[index].write_bytes(original+b'\n')
                with self.assertRaisesRegex(PolicyError, 'mismatch'):
                    validate_transfer_contract(contract, *paths, speed_scale=1., lookahead=1.8)
                paths[index].write_bytes(original)
            with self.assertRaisesRegex(PolicyError, 'control_profile_mismatch'):
                validate_transfer_contract(contract, *paths, speed_scale=.8, lookahead=1.8)
            wrong = copy.deepcopy(contract)
            wrong['transfer_profile']['estimated_waypoint_arrival_radii_m'][0] = .08
            with self.assertRaisesRegex(PolicyError, 'waypoint_tolerance_mismatch'):
                validate_transfer_contract(wrong, *paths, speed_scale=1., lookahead=1.8)
            with self.assertRaisesRegex(PolicyError, 'transfer_profile_required'):
                validate_transfer_contract(contract | {'profile': 'legacy'}, *paths, speed_scale=1., lookahead=1.8)


if __name__ == '__main__':
    unittest.main()
