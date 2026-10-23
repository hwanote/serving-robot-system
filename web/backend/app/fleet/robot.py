class Robot:
    """What the backend knows about one robot (filled by its adapter's callbacks)."""

    def __init__(self, name, adapter, home):
        self.name, self.adapter, self.home = name, adapter, home
        self.ms = None                # last mission_status (dict)
        self.health = None            # last health (dict)
        self.pose = None              # (x, y, yaw) in the map frame
        self.active = None            # task_id handed to this robot
        self.plan = []                # [(kitchen|table|home, [(edge, from, to)])] of that task
        self.idle_since = None        # monotonic time the robot started to idle (not at home)
        self.homing = None            # id of the go-home trip being started / driven
        self.homing_at = 0.0
        self.last_seen = None         # monotonic time of the last message from the robot
        self.offline = False          # no message for `offline_after_s`
        self.offline_since = None
        self.paused = False           # operator: take no new orders (a running order continues)
        self.assigned_at = 0.0        # monotonic time the active order was sent
