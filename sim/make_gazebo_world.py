"""Generate a Gazebo Harmonic world (SDF) that contains the official course mesh.

Usage:
  python make_gazebo_world.py /path/dronehacks-course-v3.obj  <PX4-Autopilot>/Tools/simulation/gz/worlds/dronehacks.sdf
then
  PX4_GZ_WORLD=dronehacks make px4_sitl gz_x500_depth
  MicroXRCEAgent udp4 -p 8888

Notes (PX4 v1.16 + Gazebo Harmonic):
  * The world must contain NO <plugin> elements: PX4 loads its sensor systems
    via GZ_SIM_SERVER_CONFIG_PATH, which Gazebo ignores when the world has
    plugins (you would silently lose IMU/baro/mag).
  * <world name="..."> must equal the file name (PX4 spawns into /world/<name>).
  * Gazebo Harmonic loads OBJ (and GLB) meshes. The OBJ is Z-up metres with the
    origin at the floor centre, which matches Gazebo's world frame.
  * The OBJ includes the net (thin planes on the boundary and the roof); they
    render as dark walls and also collide, like the real net.
  * There is no cuVSLAM in SITL: run `ros2 run dh_autonomy sitl_odom` to republish
    PX4's /fmu/out/vehicle_odometry as nav_msgs/Odometry (ENU) on /dh/sim_odom and
    start the mission with backend:=px4 odom_topic:=/dh/sim_odom.
"""
import os
import sys

TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>
    <model name="dronehacks_course">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="course">
        <visual name="visual"><geometry><mesh><uri>{uri}</uri></mesh></geometry></visual>
        <collision name="collision"><geometry><mesh><uri>{uri}</uri></mesh></geometry></collision>
      </link>
    </model>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>37.4241</latitude_deg>
      <longitude_deg>-122.1661</longitude_deg>
      <elevation>30</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    obj, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    name = os.path.splitext(os.path.basename(out))[0]
    with open(out, "w") as f:
        f.write(TEMPLATE.format(name=name, uri="file://" + obj.replace("\\", "/")))
    print("wrote", out, "(world name '%s')" % name)
    print("Spawn pose: the launch mark is at x=-2.515 y=0.914 (model frame); e.g.\n"
          "  PX4_GZ_MODEL_POSE='-2.515,0.914,0.1,0,0,1.5708' PX4_GZ_WORLD=%s make px4_sitl gz_x500_depth" % name)


if __name__ == "__main__":
    main()
