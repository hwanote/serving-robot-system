"""App factory: wires site, repository, fleet, adapters and routers."""
import asyncio
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI

from .config import Settings, default_data_dir
from .events import EventHub
from .fleet.manager import FleetManager
from .fleet.robot import Robot
from .mapping.adopt import MapAdopter
from .mapping.session import MappingConfig, MappingSession
from .repository.memory import MemoryRepository
from .repository.sql import SqlRepository
from .ros_adapter import make_adapter
from .routers import debug, maps, robots, tasks, ws
from .services.orders import OrderService
from .semantics import SemanticStore
from .promote import LiveConfig, live_map_image
from .site import Site


def build_fleet(settings, site, repo, hub):
    robots = []
    for name, domain in settings.robots:
        robots.append(Robot(name, make_adapter(settings, name, domain, site),
                            site.home_of(name)))
    return FleetManager(robots, site, repo, hub, settings)


def create_app(settings=None, repo=None):
    settings = settings or Settings()
    site = Site(settings.repo, settings.store_id)
    repo = repo or (MemoryRepository() if settings.db_url == 'memory'
                    else SqlRepository(settings.db_url))
    hub = EventHub()
    fleet = build_fleet(settings, site, repo, hub)

    data_dir = settings.data_dir or default_data_dir()
    domain = settings.robots[0][1] if settings.robots else None
    mapping = MappingSession(MappingConfig(
        store_id=settings.store_id,
        work_dir=settings.mapping_work_dir or os.path.join(data_dir, 'mapping'),
        runner=settings.mapping_runner or os.path.join(
            settings.repo, 'ros_ws', 'src', 'serving_robot_mapping', 'tools',
            'run_lidar_mapping_session.py'),
        ros_python=settings.mapping_ros_python, enabled=settings.mapping_enabled,
        require_ros_env=settings.mapping_require_ros_env,
        env={'ROS_DOMAIN_ID': str(domain)} if domain is not None else {}))
    fleet.mapping_active = lambda: mapping.phase == 'mapping'   # no lock: called under fleet.lock

    @asynccontextmanager
    async def lifespan(app):
        hub.loop = asyncio.get_running_loop()
        fleet.recover()
        for r in fleet.robots.values():
            r.adapter.start(fleet)

        async def tick():
            while True:
                await asyncio.sleep(0.5)
                if mapping.phase == 'mapping':      # notice the runner's end even if nobody polls
                    await asyncio.get_running_loop().run_in_executor(None, mapping.status)
                for ev in fleet.tick():
                    hub.push(ev)

        task = asyncio.create_task(tick())
        yield
        task.cancel()
        hub.loop = None
        for r in fleet.robots.values():
            r.adapter.stop()

    app = FastAPI(lifespan=lifespan, title='serving-robot backend')
    app.state.settings, app.state.site, app.state.repo = settings, site, repo
    app.state.hub, app.state.fleet = hub, fleet
    app.state.orders = OrderService(fleet, site)
    app.state.semantics = SemanticStore(site, settings.repo,
                                        settings.data_dir or default_data_dir(),
                                        settings.require_drive_test)
    app.state.live = LiveConfig(site, fleet, settings.repo, settings.data_dir or default_data_dir())
    fleet.live_revision = app.state.live.revision
    fleet.live_map_image = lambda: live_map_image(settings.repo, settings.store_id)
    app.state.mapping = mapping
    app.state.adopter = MapAdopter(site, fleet, app.state.semantics, mapping, data_dir,
                                   settings.mapping_shift_origin)
    for module in (maps, tasks, robots, debug, ws):
        app.include_router(module.router)
    return app
