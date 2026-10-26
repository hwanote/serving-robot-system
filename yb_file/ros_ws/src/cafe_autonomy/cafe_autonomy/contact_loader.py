"""Add the standard Contact world system before spawning robot sensors.

Uses the simulator service; the original cafe.world.sdf remains byte-identical.
"""
import subprocess,time
from pathlib import Path
from ament_index_python.packages import get_package_prefix

def main():
    library=Path(get_package_prefix('serving_robot_gazebo'))/'lib/libteam_contact_snapshot.so'
    if not library.is_file():raise FileNotFoundError(library)
    request='entity { name: "cafe" type: WORLD } plugins { filename: "'+str(library)+'" name: "team::ContactSnapshot" }'
    for attempt in range(10):
        result=subprocess.run(['gz','service','-s','/world/cafe/entity/system/add','--reqtype','gz.msgs.EntityPlugin_V','--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True)
        if result.returncode==0 and 'data: true' in result.stdout:
            print('CONTACT_SYSTEM_READY: original world file unchanged',flush=True);return
        time.sleep(.2)
    raise RuntimeError('Contact system did not load; refusing sensor spawn readiness')
