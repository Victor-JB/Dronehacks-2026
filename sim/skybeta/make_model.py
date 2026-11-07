"""Build models/skybeta: the Sky Beta airframe for Gazebo + PX4 SITL.

Starts from the x500 PX4 already flies in this sim (models/flight_x500*, PX4 v1.16.2) so the PX4
interface stays the same (rotor order, motor plugins, IMU/baro/mag), and replaces what differs.
Run from sim/:  python3 skybeta/make_model.py   (writes models/skybeta/)

Sources, all in the CAD frame of sky-beta.step (mm, +Z up, front = -Y, +X = left), converted to
FLU metres with base_link at the CAD origin, BASE_Z above the floor:
  rotors   X2807 motor centres from the STEP; they match skybeta px4/params/07 CA_ROTOR*_PX/PY
  OAK-D S2 body centre (0.2, -89.1, 94.6), front face at Y = -103.2 -> right camera at the face
  VL53L1X  EKF2_RNG_POS_Y/Z from the fleet params (FRD), x = 0
  mass     1.25 kg total (the lab's Sky Beta figure); the centre of mass and inertia come from
           the STEP layout with ESTIMATED part masses -- see sim/skybeta/AIRFRAME.md
  thrust   X2807 1500KV, 4S, 7 in props: ESTIMATED, see AIRFRAME.md. Replace with measurements.
"""
from pathlib import Path
import copy
import math
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
MODELS = HERE.parent / 'models'

BASE_Z = 0.052                       # base_link (CAD origin, ~ Pixhawk height) above the floor
MASS = 1.25
ROTOR_MASS = 0.007
COG = (-0.0164, 0.0001, 0.0136)      # from base_link
INERTIA = dict(ixx=0.00716, iyy=0.00731, izz=0.01163, ixy=0.0, ixz=0.000122, iyz=0.0)
PROP_Z = 0.010                        # prop plane above base_link
PROP_RADIUS = 0.0889                  # 7 in
ROTORS = {                            # x500 order and spin: 0 FR ccw, 1 RL ccw, 2 FL cw, 3 RR cw
    0: (0.0831, -0.1597), 1: (-0.1475, 0.1351), 2: (0.0831, 0.1597), 3: (-0.1475, -0.1351)}
MAX_ROT_VELOCITY = 1600.0             # rad/s at full output
MOTOR_CONSTANT = 6.13e-06             # N/(rad/s)^2: ~15.7 N (1.6 kgf) per motor at full output
OAK = (0.1032, -0.0375, 0.0426)       # right camera (depth reference) from base_link
RANGEFINDER = (0.0, 0.0274, -0.0345)  # FLU from EKF2_RNG_POS_Y = -0.027432, EKF2_RNG_POS_Z = 0.03445


def text(parent, tag, value):
    e = ET.SubElement(parent, tag)
    e.text = str(value)
    return e


def box_visual(link, name, pose, size, rgba):
    v = ET.SubElement(link, 'visual', name=name)
    text(v, 'pose', ' '.join(f'{x:.4f}' for x in pose))
    text(ET.SubElement(ET.SubElement(v, 'geometry'), 'box'), 'size', ' '.join(f'{x:.4f}' for x in size))
    m = ET.SubElement(v, 'material')
    text(m, 'ambient', rgba)
    text(m, 'diffuse', rgba)


def box_collision(link, name, pose, size):
    c = ET.SubElement(link, 'collision', name=name)
    text(c, 'pose', ' '.join(f'{x:.4f}' for x in pose))
    text(ET.SubElement(ET.SubElement(c, 'geometry'), 'box'), 'size', ' '.join(f'{x:.4f}' for x in size))


def oak_link_xml(width=512, height=320, rate=30):
    """OAK-D S2 depth reference (right camera), looking forward along link +X. HFOV 80 deg: the OV9282
    eyes (skybeta sim: fx 762.69 at 1280 px). Depth below 0.35 m is no return, as on the OAK."""
    return ET.fromstring(f'''<link name="oak_link">
      <pose relative_to="base_link">{OAK[0]} {OAK[1]} {OAK[2]} 0 0 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
      <sensor name="oak" type="rgbd_camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>/sim/oak</topic>
        <camera>
          <horizontal_fov>1.39626</horizontal_fov>
          <image><width>{width}</width><height>{height}</height></image>
          <clip><near>0.35</near><far>10</far></clip>
        </camera>
      </sensor>
    </link>'''), ET.fromstring('''<joint name="oak_joint" type="fixed">
      <parent>base_link</parent><child>oak_link</child></joint>''')


def main():
    base = ET.parse(MODELS / 'flight_x500_base' / 'model.sdf')
    motors = ET.parse(MODELS / 'flight_x500' / 'model.sdf')
    model = base.getroot().find('model')
    model.set('name', 'skybeta')
    model.find('pose').text = f'0 0 {BASE_Z} 0 0 0'

    link = model.find("link[@name='base_link']")
    for child in list(link):
        if child.tag in ('visual', 'collision') or (child.tag == 'sensor' and child.get('type') == 'contact'):
            link.remove(child)
    inertial = link.find('inertial')
    for child in list(inertial):
        inertial.remove(child)
    text(inertial, 'pose', ' '.join(map(str, COG)) + ' 0 0 0')
    text(inertial, 'mass', round(MASS - 4 * ROTOR_MASS, 4))
    inertia = ET.SubElement(inertial, 'inertia')
    for k, v in INERTIA.items():
        text(inertia, k, v)

    z0 = -BASE_Z
    # what the camera and the walls see: battery, plates, Jetson, OAK, arms (simple boxes)
    box_visual(link, 'battery', (-0.0315, 0, 0.098 + z0, 0, 0, 0), (0.158, 0.046, 0.036), '0.15 0.15 0.15 1')
    box_visual(link, 'plates', (-0.014, 0, 0.042 + z0, 0, 0, 0), (0.206, 0.143, 0.004), '0.1 0.1 0.1 1')
    box_visual(link, 'jetson', (-0.0074, 0, 0.015 + z0, 0, 0, 0), (0.079, 0.100, 0.030), '0.2 0.5 0.2 1')
    box_visual(link, 'oak', (0.0891, 0, 0.0946 + z0, 0, 0, 0), (0.021, 0.097, 0.030), '0.3 0.3 0.3 1')
    box_collision(link, 'body', (-0.0037, 0, 0.058 + z0, 0, 0, 0), (0.2137, 0.143, 0.116))
    for i, (x, y) in ROTORS.items():
        length = math.hypot(x, y)
        pose = (x / 2, y / 2, 0.040 + z0, 0, 0, math.atan2(y, x))
        box_visual(link, f'arm_{i}', pose, (length, 0.025, 0.004), '0.1 0.1 0.1 1')
        box_collision(link, f'arm_{i}', pose, (length, 0.025, 0.020))

    for i, (x, y) in ROTORS.items():
        rotor = model.find(f"link[@name='rotor_{i}']")
        rotor.find('pose').text = f'{x} {y} {PROP_Z} 0 0 0'
        rotor.find('inertial/mass').text = str(ROTOR_MASS)
        for v in rotor.findall('visual'):
            rotor.remove(v)
        v = ET.SubElement(rotor, 'visual', name=f'rotor_{i}_visual')
        cyl = ET.SubElement(ET.SubElement(v, 'geometry'), 'cylinder')
        text(cyl, 'radius', PROP_RADIUS)
        text(cyl, 'length', 0.002)
        m = ET.SubElement(v, 'material')
        text(m, 'diffuse', '0.6 0.6 0.6 0.4')
        bell = ET.SubElement(rotor, 'visual', name=f'rotor_{i}_motor')
        text(bell, 'pose', '0 0 -0.009 0 0 0')
        cyl = ET.SubElement(ET.SubElement(bell, 'geometry'), 'cylinder')
        text(cyl, 'radius', 0.0168)
        text(cyl, 'length', 0.0175)
        size = rotor.find('collision/geometry/box/size')
        size.text = f'{2 * PROP_RADIUS} 0.0169 0.0008'

    for plugin in motors.getroot().find('model').findall('plugin'):
        plugin = copy.deepcopy(plugin)
        plugin.find('maxRotVelocity').text = str(MAX_ROT_VELOCITY)
        plugin.find('motorConstant').text = str(MOTOR_CONSTANT)
        model.append(plugin)

    # VL53L1X: PX4's gz bridge reads a 1-ray lidar at link lidar_sensor_link / sensor lidar as a
    # downward distance_sensor (PX4-gazebo-models x500_lidar_down; GZBridge.cpp, v1.16.2).
    model.append(ET.fromstring(f'''<link name="lidar_sensor_link">
      <pose relative_to="base_link">{RANGEFINDER[0]} {RANGEFINDER[1]} {RANGEFINDER[2]} 0 1.5708 0</pose>
      <inertial><mass>0.0024</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>lidar_sensor_link</gz_frame_id>
        <pose>0 0 0 3.14159 0 0</pose>
        <update_rate>5</update_rate>
        <ray>
          <scan>
            <horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>0.04</min><max>4.0</max><resolution>0.001</resolution></range>
        </ray>
        <always_on>1</always_on>
      </sensor>
    </link>'''))
    model.append(ET.fromstring('''<joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent><child>lidar_sensor_link</child></joint>'''))

    # The OAK-D S2 camera is added by skybeta/scene.py (oak_link_xml): its size follows the drone config.

    # Ground truth for vio_sim (the cuVSLAM stand-in) ONLY. Every topic is renamed away from
    # /model/<name>/odometry_with_covariance: PX4's gz bridge fuses that one as vision odometry.
    model.append(ET.fromstring('''<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/sim/truth/odometry</odom_topic>
      <odom_covariance_topic>/sim/truth/odometry_with_covariance</odom_covariance_topic>
      <tf_topic>/sim/truth/tf</tf_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>'''))

    out = MODELS / 'skybeta'
    out.mkdir(exist_ok=True)
    ET.indent(base, space='  ')
    base.write(out / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (out / 'model.config').write_text('''<?xml version="1.0"?>
<model>
  <name>skybeta</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Stanford DroneHacks Sky Beta for PX4 SITL. Generated by sim/skybeta/make_model.py.</description>
</model>
''')
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
