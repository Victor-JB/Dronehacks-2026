"""PX4 SITL parameters for the Sky Beta: the drone's own, then the few the sim needs.

Prints `export PX4_PARAM_<NAME>=<value>` lines for PX4's startup (run from sim/):
    eval "$(python3 skybeta/px4_params.py)"

Order, later wins:
  1. skybeta px4/params stage files 03a (height), 03b (vision), 07 (geometry), 08 (no flow), 09 (takeover)
  2. OUR drone's full dump, params/skybeta-02.params (QGroundControl: Vehicle Setup > Parameters >
     Tools > Save to file). Until it exists: drone 16's (same frame and fleet tuning, another unit).
     Only the groups that set how it estimates and flies: estimator, controllers, allocation,
     commander, navigator, failsafes
  3. SIM_OVERRIDES below: things that are physically different in SITL, each with its reason
"""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent / 'params'
STAGES = ['03a_height_reference', '03b_ekf2_vision', '07_airframe_geometry', '08_no_optical_flow',
          '09_autonomy_takeover']
OURS = 'skybeta-02'
FALLBACK = 'drone16-vio-2026-11-03'
GROUPS = ('EKF2_', 'MPC_', 'MC_', 'CA_ROTOR', 'COM_', 'NAV_', 'GF_', 'LNDMC_', 'MIS_',
          'IMU_GYRO_', 'IMU_DGYRO_', 'IMU_ACC', 'THR_MDL_FAC', 'MAN_', 'SYS_HAS_')
SKIP = ('EKF2_MULTI_', 'COM_RC_IN_MODE')    # one SITL IMU/mag; RC input is replaced below

SIM_OVERRIDES = {
    # The scripted pilot's sticks arrive as MAVLink MANUAL_CONTROL instead of an RC receiver.
    'COM_RC_IN_MODE': 1,
    # PX4 SITL runs on Gazebo's clock and so do the ROS nodes (use_sim_time): no clock to sync.
    'UXRCE_DDS_SYNCT': 0,
    # The sim's odometry is stamped when Gazebo samples the pose; the drone's 60 ms is cuVSLAM's
    # pipeline, which the sim does not have.
    'EKF2_EV_DELAY': 0,
    # Gazebo motors: output 0..1 -> 150..1600 rad/s (sim/models/skybeta, X2807 1500KV on 4S).
    **{f'SIM_GZ_EC_MIN{i}': 150 for i in range(1, 5)},
    **{f'SIM_GZ_EC_MAX{i}': 1600 for i in range(1, 5)},
    # Starting point for PX4's hover-thrust estimator (MPC_USE_HTE = 1 on the drone): the model hovers
    # at ~707 rad/s -> (707 - 150) / 1450 = 0.38.
    'MPC_THR_HOVER': 0.38,
}


def read(path):
    out = {}
    for line in path.read_text().splitlines():
        cols = line.split('\t')
        if line.startswith('#') or len(cols) < 4:
            continue
        out[cols[2]] = cols[3]
    return out


def params():
    merged = {}
    for stage in STAGES:
        merged.update(read(HERE / f'{stage}.params'))
    dump = HERE / f'{OURS}.params'
    if not dump.exists():
        print(f'# WARNING: no params/{OURS}.params: using {FALLBACK} (another Sky Beta)', file=sys.stderr)
        dump = HERE / f'{FALLBACK}.params'
    fleet = read(dump)
    merged.update({k: v for k, v in fleet.items() if k.startswith(GROUPS) and not k.startswith(SKIP)})
    merged.update({k: str(v) for k, v in SIM_OVERRIDES.items()})
    return merged


if __name__ == '__main__':
    p = params()
    if '--list' in sys.argv:
        for k in sorted(p):
            print(f'{k}={p[k]}')
    else:
        for k in sorted(p):
            print(f'export PX4_PARAM_{k}={p[k]}')
