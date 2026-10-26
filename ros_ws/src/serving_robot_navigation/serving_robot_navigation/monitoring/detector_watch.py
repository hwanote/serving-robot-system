"""Is the person detector alive? (no ROS dependency)

The route executor uses it to tell the operator when the detector is silent.
"""


class DetectorWatch:
    """Tells the operator that the person detector is silent (for the route
    executor, which knows the mission). No ROS.

    Events returned by step(): 'SILENT' (once), 'SILENT_REPEAT' (every repeat_s
    while it stays silent), 'RECOVERED' (it speaks again). A detector that never
    spoke gets startup_grace_s (the model takes a while to load).
    """

    def __init__(self, timeout_s=1.0, startup_grace_s=30.0, repeat_s=30.0):
        self.timeout = timeout_s
        self.grace = startup_grace_s
        self.repeat = repeat_s
        self.t0 = None
        self.last = None
        self.silent = False
        self.alarm_t = None

    def heard(self, now):
        self.last = now

    def step(self, now):
        if self.t0 is None:
            self.t0 = now
        if self.last is None:
            silent = now - self.t0 > self.grace
        else:
            silent = now - self.last > self.timeout
        if silent:
            if not self.silent:
                self.silent, self.alarm_t = True, now
                return ['SILENT']
            if now - self.alarm_t >= self.repeat:
                self.alarm_t = now
                return ['SILENT_REPEAT']
        elif self.silent:
            self.silent = False
            return ['RECOVERED']
        return []


class CameraWatch:
    """Tells the operator which cameras are silent (person_detection lists them
    in silent_cameras). No ROS.

    update(now, silent) returns events [(event, camera)]: ('SILENT', cam) when a
    camera goes silent, ('SILENT_REPEAT', cam) every repeat_s while it stays
    silent, ('RECOVERED', cam) when it is back.
    """

    def __init__(self, repeat_s=30.0):
        self.repeat = repeat_s
        self.alarm_t = {}            # silent camera -> time of its last alarm

    def update(self, now, silent):
        silent = sorted(set(silent))
        out = []
        for cam in silent:
            if cam not in self.alarm_t:
                self.alarm_t[cam] = now
                out.append(('SILENT', cam))
            elif now - self.alarm_t[cam] >= self.repeat:
                self.alarm_t[cam] = now
                out.append(('SILENT_REPEAT', cam))
        for cam in sorted(set(self.alarm_t) - set(silent)):
            del self.alarm_t[cam]
            out.append(('RECOVERED', cam))
        return out
