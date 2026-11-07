import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "ros2_ws", "src", "dh_core")))
COURSE_OBJ = os.environ.get("DH_COURSE_OBJ", "C:/dhdl/obj/dronehacks-course-v3.obj")
