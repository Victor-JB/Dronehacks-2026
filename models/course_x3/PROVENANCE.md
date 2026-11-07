# X3 UAV

The base link, rotor joints, collision shapes, and visual meshes come from
[OpenRobotics X3 UAV, Fuel version 4](https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3%20UAV/4).
The Fuel listing credits OpenRobotics under
[Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/).
The model metadata credits Carlos Aguero and Cole Biesemeyer. The source
archive SHA-256 is
`c995a307936ce7df5a544e6c89b0396d054befa6ff5fbdbf262949b59cc12247`.
Only the four referenced mesh and texture files are included. They are unchanged.

This adaptation uses local mesh URIs, SDF 1.9 joint syntax, updated rotor
visual materials, and course-specific names and launch pose. It adds contact
sensors for the base and four rotors, 3D world odometry, and the four motor
and one flight-control system configurations from the
[Gazebo Sim 8 example at b791db8](https://github.com/gazebosim/gz-sim/blob/b791db8dd6fecf85f191a8796ff2dd817f951c07/examples/worlds/multicopter_velocity_control.sdf).
The upstream example configuration is Apache-2.0. A copy is included in
[LICENSE-APACHE-2.0.txt](LICENSE-APACHE-2.0.txt). This model adapts the topic
and entity names and sets the linear velocity gains to 5.4 on each axis.

The original X3 mass and dimensions are retained. The 0.47 m × 0.47 m ×
0.11 m base collision box and the four 0.10 m-radius rotor collision cylinders
fit within a 0.36 m sphere centred on the model origin. The farthest rotor
cylinder has an outer point approximately 0.356 m from the origin. The
window geometry check uses the 0.36 m radius as a padded spherical envelope.
