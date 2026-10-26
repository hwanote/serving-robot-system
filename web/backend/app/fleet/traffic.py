"""Route planning and node occupancy (pure functions, no I/O).

An order is planned as kitchen -> table -> home over the route graph, the same legs the
robot drives. Two robots may not hold the same graph node at the same time (the aisles
are one lane wide), the kitchen node excepted: it is guarded by the kitchen lock.
"""


def plan_order(site, kitchen, robot, table):
    """[(stage, [(edge, from, to)])] with the stages kitchen, table, home."""
    here = site.nearest_node(*robot.pose[:2])
    plan = []
    for stage, pose in (('kitchen', kitchen), ('table', table), ('home', robot.home)):
        goal = site.node_of_pose(pose)
        p = site.path(here, goal)
        plan.append((stage, site.directed(p[0]) if p else []))
        here = goal
    return plan


def claimed_nodes(site, robot):
    """Nodes a robot holds: where it stands and what it still has to pass."""
    nodes, ms = set(), robot.ms or {}
    if ms.get('node'):
        nodes.add(ms['node'])
    if robot.pose is not None:
        nodes.add(site.nearest_node(*robot.pose[:2]))
    st = ms.get('state')
    if st == 'DRIVING':
        nodes.update(ms.get('route') or [])
    if robot.active:
        rest = robot.plan
        if ms.get('mission_id') == robot.active:
            if st == 'WAIT_LOAD':
                rest = robot.plan[1:]
            elif st == 'WAIT_HANDOVER':
                rest = robot.plan[2:]
            elif st == 'DRIVING' and ms.get('route'):
                rest = []
        for _stage, edges in rest:
            for _eid, a, b in edges:
                nodes.update((a, b))
    return nodes


def find_clash(site, kitchen_node, cand, plan, robots):
    """None, or a text naming the nodes another robot holds on the order's route."""
    edges = [e for _stage, es in plan for e in es]
    if not edges:
        return None
    start = edges[0][1]
    mine = {n for _eid, a, b in edges for n in (a, b)} - {start, kitchen_node}
    for other in robots:
        if other is cand:
            continue
        both = mine & claimed_nodes(site, other)
        if both:
            return '%s would pass %s held by %s' % (
                cand.name, ','.join(sorted(both)), other.name)
    return None
