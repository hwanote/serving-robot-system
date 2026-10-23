"""Robot connections. An adapter talks to one robot and reports to a sink (the FleetManager):

    adapter.start(sink)      begin; call sink.on_status(name, dict), sink.on_health(name, dict),
                             sink.on_pose(name, x, y, yaw), sink.on_alarm(name, dict)
    adapter.send_cmd(dict)   a mission_cmd JSON (serve / go / cancel ...)

`make_adapter` picks the fake adapter (no ROS) or the rclpy adapter.
"""


def make_adapter(settings, name, domain, site):
    if settings.adapter == 'fake':
        from .fake import FakeAdapter
        return FakeAdapter(name, site, settings.fake_speed, settings.fake_wait,
                           settings.fake_fail)
    from .ros import RosAdapter
    return RosAdapter(name, domain, settings.use_sim_time)
