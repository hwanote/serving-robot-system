from app.config import default_repo_root
from app.fleet import traffic
from app.site import Site


def test_route_graph_loads():
    site = Site(default_repo_root(), 'demo_store')
    assert 'table_01' in site.tables
    assert site.home_of('robot_01')
