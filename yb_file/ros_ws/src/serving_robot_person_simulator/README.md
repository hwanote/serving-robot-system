# Isolated dynamic people simulator — ROS 2 Jazzy + Gazebo Harmonic

## Status

Version 0.1.0. **Package code and offline policy tests completed; not yet
live-validated in the user's current cafe.world.sdf.** No real positions are
invented. The `lanes: []` setting deliberately disables spawn until the map is
inspected and **up to 8 straight pedestrian lanes with checked clearance** are
entered. Requires the robot model's pose to appear in `/world/cafe/pose/info`.

Unlike the old `cafe_person_scenarios.py`, this package never uses raw `/odom`
for global location, never publishes robot `/cmd_vel`, and does not require
modifying the robot Nav2, Semantic Safety, or Collision Monitor configs.

## Files

- `config/policy.yaml`: world ID, approved lanes, 1..8 slots, probabilistic behavior.
- `serving_robot_person_simulator/policy.py`: pure random scenario logic.
- `serving_robot_person_simulator/gazebo_api.py`: on-demand Gazebo create/remove.
- `serving_robot_person_simulator/manager.py`: ROS manager, status, commands, log.
- `launch/people.launch.py`: people bridge, independent heartbeat watchdog, and manager; can be
  launched/stopped independently of existing Gazebo/robot/safety launches.
- `tests/test_policy.py`: no ROS/Gazebo needed; validates distributions and guards.

## Setup

Download the ZIP into Windows Downloads, then under WSL:

```bash
unzip -n /mnt/c/Users/user/Downloads/serving_robot_person_simulator.zip \
  -d ~/team_robot_lab/ros_ws/src/
cd ~/team_robot_lab/ros_ws
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s src/serving_robot_person_simulator/tests -v
colcon build --symlink-install --packages-select serving_robot_person_simulator
source install/local_setup.bash
```

Adjust Windows Downloads path if needed. Team original repo is NOT modified.

### Before spawning people

1. Verify `gz service -l` has `/world/cafe/create` and `/world/cafe/remove`.
2. Verify Gazebo advertises `/world/cafe/pose/info`.
3. Verify TFMessage bridge really reports `robot_01` and later `test_person_XX`
   as `child_frame_id` (no unsupported assumptions about model or link names).
4. Read current cafe.world.sdf and approve pedestrian straight lanes that do
   not cross furniture or clip the robot's starting footprint.
5. Add 1..8 lane entries to **a copy** of `config/policy.yaml` in the lab (example
   syntax is commented in the default YAML). No default spawn points exist.
6. For robot motion, keep the existing person's sensory stale-stop logic. This
   simulator **does not fake a PersonSafetyEvent** or weaken the robot's safety.

Launch in a SEPARATE terminal (Gazebo, robot, Safety already running):

```bash
export ROS_DOMAIN_ID=99
export GZ_PARTITION=team_robot_lab_gazebo_99
source /opt/ros/jazzy/setup.bash
source ~/team_robot_lab/ros_ws/install/local_setup.bash
ros2 launch serving_robot_person_simulator people.launch.py \
  config:=/home/user/team_robot_lab/person_policy_active.yaml
```

The manager starts **with no people and no motion**. If `robot_pose_valid:false`
in the status JSON, DO NOT spawn: check bridge/world frame labels first.

### Command interface

Topic: `/person_simulator/command` (`std_msgs/msg/String` JSON). Example:

```bash
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"status\"}"}'
# Configure and approve lanes before using spawn:
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"spawn\",\"count\":2}"}'
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"start\"}"}'
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"stop\",\"id\":1}"}'
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"resume\",\"id\":1}"}'
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"delete\",\"id\":1}"}'
ros2 topic pub --once /person_simulator/command std_msgs/msg/String \
  '{data: "{\"action\":\"cleanup\"}"}'
```

Other commands: `pause` (all), `unpause` (all), `shutdown` (manager exit +
delete own people), `recover` (explicit orphan cleanup *only* while no
currently-owned people). Missing id applies to all currently-owned people.

Monitor:

```bash
ros2 topic echo /person_simulator/status
```

If you choose `seed: null` the manager generates and logs one random seed per
run; choose an integer seed for reproducing draws. JSONL logs live in
`~/.ros/person_simulator/run_*.jsonl`.

### Probability semantics

- At the beginning of each lane traversal and after a stop, draw speed:
  slow 20% (0.08m/s), normal 60% (0.15m/s), fast 20% (0.22m/s).
- At first close approach to robot, draw yield 80% or non-yield 20%, cached for
  encounter; both obey unconditional proximity stop.
- At **random decision events spaced 8..16 simulated seconds apart** draw:
  continue 72%, gradual stop 22%, sudden stop 2%, long block 4%.
- `gradual_stop`: reduce speed for 1.0..2.2s, then wait 1.5..4.0s.
- `sudden_stop`: immediate zero speed and wait 0.7..2.0s (rare).
- `long_block`: pause 16..36s ONLY if current position is near the explicitly
  configured `blockage_corridor` centerline. This creates **a candidate** for
  path obstruction; the real Robot/Nav2 path must determine actual blockage.
- No per-frame probability draws. Never interpret event probabilities as a
  proportion of time spent stationary. No scripted artificial event fed into
  the real semantic person detector.

### Limitations and risk

- Kinematic cylinders are placeholders for pedestrian collision shape; their
  translation uses Gazebo VelocityControl in the world frame. **Verify with
  one person first** (GT motion, scan echo, physics interaction). Real walking
  animation is not implemented.
- A separately launched ROS watchdog publishes person-only zeros after 0.8s
  without a manager heartbeat. This handles a manager-only crash while the ROS
  bridge remains alive. **Simultaneous bridge/watchdog crashes, OS kill, or network
  failures are NOT guaranteed safe**, since Gazebo VelocityControl may retain
  the last nonzero command. On abnormal failure, pause Gazebo if necessary,
  restart people launch, then `recover` owned orphan models explicitly.
- Without confirmed model GT, all people motion is forced to zero. No LiDAR
  detection, robot autonomous navigation, or 2.5m team metric is claimed proven.
- Lane safety is a precondition; the module checks robot/person distances but
  **cannot infer furniture geometry** from pose messages alone.
- Stop and removal are separate: a stopped person remains a physical LiDAR
  obstacle; deleting removes the model from Gazebo.
- `gz service` may report true before the model appears in GT; status is the
  authoritative runtime observation. Verify absence after removal.
