"""Low-frequency Gazebo world entity service calls. No robot cmd_vel access."""
import json
import re
import subprocess

NAME_RE = re.compile(r'^test_person_[0-9]{2}$')


def make_person_sdf(name, x, y, radius=0.22, height=1.55):
    if not NAME_RE.fullmatch(name):
        raise ValueError('invalid owned model name')
    # A real cylinder collision at robot LiDAR height (0.30m).
    # VelocityControl is a Gazebo Harmonic system, not the robot safety controller.
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x:.6f} {y:.6f} 0 0 0 0</pose>
    <link name="body">
      <kinematic>true</kinematic>
      <inertial>
        <mass>60</mass>
        <inertia><ixx>12.5</ixx><iyy>12.5</iyy><izz>3.0</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="person_collision">
        <pose>0 0 {height/2:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>
      </collision>
      <visual name="person_visual">
        <pose>0 0 {height/2:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>
        <material><ambient>0.1 0.5 0.9 1</ambient><diffuse>0.1 0.5 0.9 1</diffuse></material>
      </visual>
    </link>
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <topic>/model/{name}/cmd_vel</topic>
      <initial_linear>0 0 0</initial_linear>
      <initial_angular>0 0 0</initial_angular>
    </plugin>
  </model>
</sdf>'''


def _run_service(world, operation, request, req_type):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', world):
        raise ValueError('invalid world name')
    cmd = ['gz', 'service', '-s', f'/world/{world}/{operation}',
           '--reqtype', req_type, '--reptype', 'gz.msgs.Boolean',
           '--timeout', '3000', '--req', request]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'Gazebo {operation} service failed: {exc}') from exc
    output = (r.stdout + '\n' + r.stderr).strip()
    # Gazebo CLI may return exit status 0 even when service response is false.
    if r.returncode != 0 or not re.search(r'\bdata:\s*true\b', output, re.IGNORECASE):
        raise RuntimeError(f'Gazebo {operation} rejected or timed out: {output[:600]}')
    return output


def spawn(world, name, x, y, radius, height):
    if not NAME_RE.fullmatch(name):
        raise ValueError('invalid owned model name')
    sdf = make_person_sdf(name, x, y, radius, height)
    # UserCommands may queue the actual model creation until AFTER the service
    # replies. Passing a TemporaryDirectory sdf_filename deletes the file too
    # early, leaving a "successful" service reply but no Gazebo entity.
    # EntityFactory.sdf transfers the complete model XML with the request.
    request = f'sdf: {json.dumps(sdf)}, name: {json.dumps(name)}, allow_renaming: false'
    return _run_service(world, 'create', request, 'gz.msgs.EntityFactory')


def remove(world, name):
    if not NAME_RE.fullmatch(name):
        raise ValueError('invalid owned model name')
    return _run_service(world,'remove',f'name: "{name}", type: 2', 'gz.msgs.Entity')
