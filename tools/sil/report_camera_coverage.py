"""Local geometry report using observed unit02 LEFT mono intrinsics.

These are NOT measured RGB intrinsics or a claim that the mount was changed.
The 0deg rotation is a forward-facing test mount; runtime accepts measured TF only.
"""
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/dh_core'))
from dh_core.online_geometry import coverage

K=[595.1951,0,487.3531,0,595.3043,305.8477,0,0,1]
if __name__=='__main__':
    print(json.dumps(dict(source='unit02 read-only audit: left mono 960x600',K=K,
        intended_tilt_degrees=0,physical_new_mount_verified=False,RGB_intrinsics_observed=False,
        coverage=[dict(camera_height_m=h,**coverage(K,960,600,0,h)) for h in (1.2,1.,.65,.3)],
        conclusion='No nadir view; frontal cone cannot certify the full surrounding vehicle volume. Not adequate alone for continuously observed descent.'),indent=2))
