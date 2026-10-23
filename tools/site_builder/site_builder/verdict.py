"""One tour step -> a verdict. Pure rules, no ROS.

Observation (what the drive client measured / saw), all optional except result:
  result       'SUCCEEDED' | 'FAILED' | 'CANCELED' | '' (nothing came back)
  code         the executor's result code ('NAV_FAILED', ...)
  alarms       list of alarm codes seen during the step
  latched      True when the contact sensor latched a collision (the body touched something)
  hardstop_s   seconds the collision monitor's HardStop zone was active during the step
  timed_out    the client gave up waiting
  duration_s, expected_s, stopped_max_s, max_off_rail_m
  person_slow_s, person_stop_s   seconds the person speed limit was SLOW / STOP during the step
  min_front_m, min_left_m, min_right_m   closest LiDAR return while driving (None = not measured)
"""
ORDER = ('COLLISION', 'HARDSTOP', 'BLOCKED', 'OFF_RAIL', 'NAV_FAILED', 'TIMEOUT', 'FAILED')
BLOCK_ALARMS = {'BLOCKED', 'NO_DETOUR', 'DETOUR_STARTED', 'DETOUR_LIMIT'}
MIN_STALL_S = 3.0           # the executor's own "not moving" limit
HARDSTOP_FAIL_S = 1.0       # HardStop active this long = the corridor stops the robot
PERSON_FLAG_S = 1.0         # the person limit (SLOW/STOP) held the robot this long: the pass is flagged
HARDSTOP_FLAG_S = 0.1       # shorter than that but seen: the pass is flagged


def judge(obs, par):
    """-> (verdict, flags, reasons). verdict PASS or one of ORDER; flags only for PASS-able runs."""
    alarms = set(obs.get('alarms') or [])
    why = {}
    if obs.get('latched'):
        why['COLLISION'] = 'the contact sensor latched a collision'
    if (obs.get('hardstop_s') or 0) >= HARDSTOP_FAIL_S:
        why['HARDSTOP'] = 'HardStop zone active for %.1f s' % obs['hardstop_s']
    if alarms & BLOCK_ALARMS:
        why['BLOCKED'] = 'blocked: %s' % ', '.join(sorted(alarms & BLOCK_ALARMS))
    if 'OFF_RAIL' in alarms:
        why['OFF_RAIL'] = 'the robot left the straight line (OFF_RAIL)'
    if obs.get('result') == 'FAILED':
        if obs.get('code') == 'NAV_FAILED':
            why['NAV_FAILED'] = 'Nav2 could not follow the path (NAV_FAILED)'
        else:
            why['FAILED'] = 'the executor failed the step: %s' % (obs.get('code') or 'no code')
    if obs.get('timed_out'):
        why['TIMEOUT'] = 'no result within %.0f s' % (obs.get('timeout_s') or 0)
    for v in ORDER:
        if v in why:
            return v, [], [why[x] for x in ORDER if x in why]
    if obs.get('result') != 'SUCCEEDED':
        return 'FAILED', [], ['no success result (got %r)' % obs.get('result')]
    flags, reasons = [], []
    exp, dur = obs.get('expected_s'), obs.get('duration_s')
    if exp and dur and dur > 1.5 * exp + 2.0:
        flags.append('SLOW')
        reasons.append('took %.0f s, expected %.0f s' % (dur, exp))
    if (obs.get('stopped_max_s') or 0) >= MIN_STALL_S:
        flags.append('STALLED')
        reasons.append('stood still for %.1f s' % obs['stopped_max_s'])
    if (obs.get('hardstop_s') or 0) >= HARDSTOP_FLAG_S:
        flags.append('HARDSTOP_BRIEF')
        reasons.append('HardStop zone touched for %.1f s while driving' % obs['hardstop_s'])
    pl = (obs.get('person_slow_s') or 0) + (obs.get('person_stop_s') or 0)
    if pl >= PERSON_FLAG_S:
        flags.append('PERSON_LIMIT')
        reasons.append('person speed limit active for %.0f s (slow %.0f s, stop %.0f s)'
                       % (pl, obs.get('person_slow_s') or 0, obs.get('person_stop_s') or 0))
    lat = [obs.get(k) for k in ('min_left_m', 'min_right_m') if obs.get(k) is not None]
    if lat and min(lat) < par.hs_half_m + par.tight_extra_m:
        flags.append('TIGHT')
        reasons.append('closest side return %.2f m (HardStop half width %.2f m)' % (min(lat), par.hs_half_m))
    return 'PASS', flags, reasons


def level(verdict, flags):
    if verdict != 'PASS':
        return 'FAIL'
    return 'WARN' if flags else 'OK'
