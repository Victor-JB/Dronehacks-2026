"""Camera-only localization against analytic, textureless depth scenes."""
import unittest
from types import SimpleNamespace

try:
    import cv2
    import numpy as np
    from vision.depth_odometry import (DepthFrame, DepthGates, DepthSurvey, DownwardRange,
                                        InertialTilt, SurveySurface, TrackingFailure,
                                        register_depth, tilt_residual)
    from vision.rgbd_odometry import RGBDOdometry, calibration
    from vision.rgbd_scene import RGBD_BODY_FROM_OPTICAL, RGBD_TRANSLATION
except ImportError:
    cv2 = None


@unittest.skipIf(cv2 is None, 'OpenCV and NumPy required')
class DepthOdometryTests(unittest.TestCase):
    def setUp(self):
        self.width, self.height = 384, 256
        self.k = calibration(self.width, self.height)
        self.rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        self.rays = np.stack(((u-self.k[0, 2])/self.k[0, 0],
                              (v-self.k[1, 2])/self.k[1, 1], np.ones_like(u)), axis=-1)
        self.world_from_camera = np.eye(4)
        self.world_from_camera[:3, :3] = RGBD_BODY_FROM_OPTICAL
        self.world_from_camera[:3, 3] = np.array(RGBD_TRANSLATION) + (0., 0., 1.)
        # A back wall, offset side wall, and floor. The RGB image is blank.
        self.planes = ((0, 3.0), (1, -.83), (2, 0.))

    def render(self, pose, planes=None):
        rays = self.rays @ pose[:3, :3].T
        position = pose[:3, 3]
        depth = np.full(rays.shape[:2], np.inf)
        for axis, value in (self.planes if planes is None else planes):
            with np.errstate(divide='ignore', invalid='ignore'):
                z = (value-position[axis])/rays[:, :, axis]
            depth = np.minimum(depth, np.where(z > .04, z, np.inf))
        return depth.astype(np.float32)

    def moved(self):
        pose = self.world_from_camera.copy()
        pose[:3, :3] = cv2.Rodrigues(np.array([-.014, .023, .019]))[0] @ pose[:3, :3]
        pose[:3, 3] += [.047, -.031, .025]
        return pose

    def test_metric_motion_without_rgb_texture(self):
        estimator = RGBDOdometry((0, 0, 1), intrinsics=self.k)
        self.assertTrue(estimator.estimate(self.rgb, self.render(self.world_from_camera), 100_000)['valid'])
        pose = self.moved()
        record = estimator.estimate(self.rgb, self.render(pose), 150_000)
        self.assertTrue(record['valid'], record)
        self.assertEqual(record['source'], 'depth_odometry')
        rotation = pose[:3, :3] @ np.array(RGBD_BODY_FROM_OPTICAL).T
        np.testing.assert_allclose(record['position_enu'], pose[:3, 3]-rotation @ RGBD_TRANSLATION,
                                   atol=.004)
        np.testing.assert_allclose(estimator.world_from_optical, pose, atol=.004)
        self.assertTrue(all(0 < x < .1 for x in record['position_std_m']))

    def survey(self):
        points, normals = [], []
        a, b = np.meshgrid(np.arange(-4, 4, .035), np.arange(-3, 3, .035))
        for axis, value in self.planes[:2]:
            p = np.empty((a.size, 3))
            p[:, axis] = value
            p[:, [i for i in range(3) if i != axis]] = np.c_[a.ravel(), b.ravel()]
            points.extend(p)
            normals.extend(np.tile(np.eye(3)[axis], (len(p), 1)))
        return DepthSurvey(points, normals)

    def test_survey_corrects_an_offset_without_an_image_reference(self):
        survey = self.survey()
        pose = self.moved()
        frame = DepthFrame(self.render(pose), self.k, DepthGates())
        found, quality = register_depth(frame, survey, self.world_from_camera, survey=True)
        np.testing.assert_allclose(found, pose, atol=.005)
        self.assertGreater(quality['depth_overlap'], .9)

    def test_tof_recovers_the_missing_vertical_axis_only(self):
        survey = self.survey()
        pose = self.moved()
        frame = DepthFrame(self.render(pose, self.planes[:2]), self.k, DepthGates())
        direction = np.asarray(RGBD_BODY_FROM_OPTICAL).T @ np.array([0., 0., -1.])
        expected = -pose[2, 3]/(pose[:3, :3] @ direction)[2]
        tof = DownwardRange(expected, np.zeros(3), direction)
        with self.assertRaisesRegex(TrackingFailure, 'unobservable'):
            register_depth(frame, survey, self.world_from_camera, survey=True)
        found, quality = register_depth(frame, survey, self.world_from_camera,
                                        survey=True, downward_range=tof)
        np.testing.assert_allclose(found, pose, atol=.005)
        self.assertLess(abs(quality['tof_range_residual_m']), .002)

        one_wall = DepthFrame(self.render(pose, self.planes[:1]), self.k, DepthGates())
        with self.assertRaisesRegex(TrackingFailure, 'unobservable'):
            register_depth(one_wall, survey, self.world_from_camera, survey=True,
                           downward_range=tof)

    def test_tof_uses_nearest_mesh_surface_before_ground(self):
        triangle = np.array([[[-1., -1., .2], [1., -1., .2], [0., 1., .6]]])
        surface = SurveySurface(triangle, floor_z=0.)
        ray = np.array([0., 0., -1.])
        distance, normal = surface.intersect(np.array([0., 0., 1.]), ray)
        self.assertAlmostEqual(distance, .6)
        np.testing.assert_allclose(normal, (0., -.2, 1.)/np.linalg.norm((0., -.2, 1.)))
        distance, normal = surface.intersect(np.array([2., 0., 1.]), ray)
        self.assertAlmostEqual(distance, 1.)
        np.testing.assert_allclose(normal, (0., 0., 1.))

    def test_tilt_jacobian_matches_the_scaled_rotation_step(self):
        rotation = cv2.Rodrigues(np.array([.012, -.028, .08]))[0]
        up = np.array([.03, -.01, 1.])
        up /= np.linalg.norm(up)
        _, jac, _ = tilt_residual(rotation, up)
        for column, direction in enumerate(np.eye(3)):
            step = 1e-6
            plus = cv2.Rodrigues(step*direction/2)[0] @ rotation
            minus = cv2.Rodrigues(-step*direction/2)[0] @ rotation
            numeric = (tilt_residual(plus, up)[0]-tilt_residual(minus, up)[0])/(2*step)
            np.testing.assert_allclose(jac[:, 3+column], numeric, atol=1e-8)
        np.testing.assert_array_equal(jac[:, :3], 0.)
        np.testing.assert_array_equal(jac[:, 5], 0.)

    def test_tilt_resolves_pitch_null_but_cannot_invent_translation(self):
        a = np.linspace(-1., 1., 80)
        points = np.vstack([np.c_[np.full_like(a, 2.), a, 0*a],
                            np.c_[a, np.ones_like(a), a**2],
                            np.c_[0*a, a, 0*a]])
        normals = np.repeat(np.eye(3), len(a), axis=0)
        initial = np.eye(4)
        initial[:3, :3] = cv2.Rodrigues(np.array([0., .02, 0.]))[0]
        tilt = InertialTilt(1_000_000, np.array([0., 0., 1.]), .01, 0)
        wrong_tilt = InertialTilt(1_000_000, np.array([1., 0., 0.]), .01, 0)
        overconfident_tilt = InertialTilt(1_000_000, np.array([0., 0., 1.]), .001, 0)

        def pair(drop):
            xyz, directions = points[drop:], normals[drop:]
            frame = SimpleNamespace(sample_points=xyz, sample_normals=directions)
            reference = SimpleNamespace(correspond=lambda p, n, d: (np.arange(len(xyz)), xyz, directions))
            return frame, reference

        with self.assertRaisesRegex(TrackingFailure, 'unobservable'):
            register_depth(*pair(0), initial, survey=True)
        found, diagnostics = register_depth(*pair(0), initial, survey=True, inertial_tilt=tilt)
        np.testing.assert_allclose(found, np.eye(4), atol=.0002)
        self.assertEqual(diagnostics['px4_tilt_prior_status'], 'applied')
        self.assertEqual(diagnostics['px4_tilt_capture_usec'], 1_000_000)
        self.assertGreaterEqual(diagnostics['rotation_std'][1], .074)
        with self.assertRaisesRegex(TrackingFailure, 'unobservable'):
            register_depth(*pair(len(a)), initial, survey=True, inertial_tilt=tilt)
        with self.assertRaisesRegex(TrackingFailure, 'unobservable') as failure:
            register_depth(*pair(0), initial, survey=True, inertial_tilt=wrong_tilt)
        self.assertEqual(failure.exception.details['px4_tilt_prior_status'], 'disagreement')
        with self.assertRaisesRegex(TrackingFailure, 'unobservable'):
            register_depth(*pair(0), initial, survey=True, inertial_tilt=overconfident_tilt)

    def test_tilt_cannot_invent_yaw_during_disagreement(self):
        a = np.linspace(-1., 1., 80)
        points = np.vstack([np.c_[np.full_like(a, 2.), 0*a, a],
                            np.c_[0*a, np.ones_like(a), a],
                            np.c_[a, a**2, 0*a]])
        normals = np.repeat(np.eye(3), len(a), axis=0)
        frame = SimpleNamespace(sample_points=points, sample_normals=normals)
        reference = SimpleNamespace(correspond=lambda p, n, d: (np.arange(len(points)), points, normals))
        tilt = InertialTilt(1_000_000, np.array([np.sin(.03), 0., np.cos(.03)]), .01, 0)
        with self.assertRaisesRegex(TrackingFailure, 'unobservable'):
            register_depth(frame, reference, np.eye(4), survey=True, inertial_tilt=tilt)

    def test_stale_tilt_is_ignored_at_odometry_boundary(self):
        pose = self.world_from_camera
        depth = self.render(pose)
        estimator = RGBDOdometry((0, 0, 1), intrinsics=self.k, survey=self.survey())
        stale = InertialTilt(50_000, pose[:3, :3].T @ np.array([0., 0., 1.]), .01, 0)
        record = estimator.estimate(self.rgb, depth, 100_000, inertial_tilt=stale)
        self.assertTrue(record['valid'], record)
        self.assertNotIn('px4_tilt_prior_status', record)

    def test_unobservable_planes_are_rejected_and_loss_latches_after_startup(self):
        wall = self.render(self.world_from_camera, ((0, 3.),))
        estimator = RGBDOdometry((0, 0, 1), intrinsics=self.k)
        before_start = estimator.estimate(self.rgb, wall, 100_000)
        self.assertFalse(before_start['valid'])
        self.assertIn('unobservable', before_start['reason'])
        self.assertIsNone(before_start['position_enu'])
        usable = self.render(self.world_from_camera)
        self.assertTrue(estimator.estimate(self.rgb, usable, 150_000)['valid'])
        failed = estimator.estimate(self.rgb, wall, 200_000)
        self.assertFalse(failed['valid'])
        self.assertTrue(failed['reason'].startswith('tracking_lost:'))
        rejected_fit = failed['rejected_fits']['frame_depth']
        self.assertLess(rejected_fit['depth_observability'], 1e-6)
        self.assertEqual(len(rejected_fit['reference_from_optical_3x4']), 3)
        self.assertFalse(estimator.estimate(self.rgb, usable, 250_000)['valid'])

    def test_clipped_depth_is_not_a_surface(self):
        for fill in (np.nan, np.inf, .02, 8.):
            frame = DepthFrame(np.full(self.rgb.shape[:2], fill), self.k, DepthGates())
            self.assertEqual(len(frame.sample_points), 0)


if __name__ == '__main__':
    unittest.main()
