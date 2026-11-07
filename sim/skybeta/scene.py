"""The course world with the Sky Beta (models/skybeta) on the launch mark, for PX4 SITL.

    python3 skybeta/scene.py --output /tmp/skybeta-world.sdf      (run from sim/)

Reuses vision/scene.py for the course, the sensor systems and the launch mark, then swaps its x500
for the Sky Beta. Environment: SIM_REAL_TIME_FACTOR (default 1.0), SIM_OAK_WIDTH / SIM_OAK_HEIGHT
(default 512 x 320: the drone's team camera config, 1024 x 640 eyes with 2x on-device decimation).
"""
import argparse
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vision.scene import generate                                     # noqa: E402
from vision.course import LAUNCH_XY                                   # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_model import oak_link_xml                                   # noqa: E402

LAUNCH_YAW = math.pi / 2          # the surveyed launch heading of the course (vision/rgbd_scene.py)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=Path('worlds/course.sdf'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    generate(args.world, args.output, 0.0)

    tree = ET.parse(args.output)
    world = tree.getroot().find('world')
    rtf = float(os.environ.get('SIM_REAL_TIME_FACTOR', '1.0'))
    if not rtf > 0:
        raise ValueError('SIM_REAL_TIME_FACTOR must be > 0')
    world.find('physics/real_time_factor').text = str(rtf)

    model = world.find("model[@name='course_quad']")
    model.find('include/uri').text = 'model://skybeta'
    model.find('pose').text = f'{LAUNCH_XY[0]} {LAUNCH_XY[1]} 0 0 0 {LAUNCH_YAW}'
    for tag, name in (('link', 'down_camera_link'), ('joint', 'down_camera_joint')):
        element = model.find(f"{tag}[@name='{name}']")
        if element is not None:
            model.remove(element)

    # The OAK's size is set here, not in the model, so it can follow the drone's camera config.
    width = int(os.environ.get('SIM_OAK_WIDTH', '512'))
    height = int(os.environ.get('SIM_OAK_HEIGHT', '320'))
    model.extend(oak_link_xml(width, height))
    ET.indent(tree, space='  ')
    tree.write(args.output, encoding='utf-8', xml_declaration=True)
    print(f'{args.output}: Sky Beta at {LAUNCH_XY} yaw {math.degrees(LAUNCH_YAW):.0f} deg, '
          f'OAK {width}x{height}, real-time factor {rtf}')


if __name__ == '__main__':
    main()
