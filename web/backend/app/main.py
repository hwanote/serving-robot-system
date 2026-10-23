"""App factory: wires site, repository, fleet, adapters and routers."""
import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI

from .config import Settings, default_data_dir
from .events import EventHub
from .fleet.manager import FleetManager
from .fleet.robot import Robot
from .repository.memory import MemoryRepository
from .repository.sql import SqlRepository
from .ros_adapter import make_adapter
from .routers import debug, maps, robots, tasks, ws
from .services.orders import OrderService
from .semantics import SemanticStore
from .promote import LiveConfig
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

    @asynccontextmanager
    async def lifespan(app):
        hub.loop = asyncio.get_running_loop()
        fleet.recover()
        for r in fleet.robots.values():
            r.adapter.start(fleet)

        async def tick():
            while True:
                await asyncio.sleep(0.5)
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
    for module in (maps, tasks, robots, debug, ws):
        app.include_router(module.router)
    return app
