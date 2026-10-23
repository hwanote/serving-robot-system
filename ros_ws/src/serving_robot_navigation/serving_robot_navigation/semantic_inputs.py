"""Read-only Part 1 input adapters. Never manufacture destination coordinates."""
from pathlib import Path
import math
import yaml

def read_mapping(path):
    data=yaml.safe_load(Path(path).read_text(encoding="utf8"))
    if not isinstance(data,dict):raise ValueError("Missing/empty Part 1 input: "+str(path))
    return data

def destinations(path):
    data=read_mapping(path);result={}
    if "destinations" in data:
        for name,pose in data["destinations"].items():
            if "approaches" in pose:raise ValueError("Select a named Part 1 waypoint, not an implicit approach: "+name)
            result[name]=tuple(float(pose[k]) for k in ("x","y","yaw"))
    elif "poses" in data and "nodes" in data:
        for name,pose in data["poses"].items():
            node=data["nodes"][pose["node"]]
            result[name]=(float(node["x"]),float(node["y"]),math.radians(float(pose["yaw_deg"])))
            for alias in pose.get("aliases",[]):result[alias]=result[name]
    elif isinstance(data.get("waypoints"),list):
        for wp in data["waypoints"]:
            pose=wp["pose"]
            value=tuple(float(pose[k]) for k in ("x","y","yaw"))
            result[wp["id"]]=value
            if wp.get("entity_id"):result[wp["entity_id"]]=value
    else:raise ValueError("Unsupported Part 1 waypoint schema")
    if not result or any(not all(math.isfinite(v) for v in pose) for pose in result.values()):
        raise ValueError("No valid finite Part 1 destinations")
    return result

def zones(path):
    data=read_mapping(path);out=[]
    for kind in ("no_go","slow","exclusive"):
        for name,entry in data.get(kind,{}).items():
            entry=entry if isinstance(entry,dict) else {"bounds":entry}
            out.append(dict(entry,id=name,type=kind))
    for entry in data.get("zones",[]):
        if entry.get("type") not in ("no_go","slow","exclusive"):continue
        out.append(entry)
    return out

def contains(zone,x,y):
    if "bounds" in zone:
        a,b,c,d=zone["bounds"];return a<=x<=b and c<=y<=d
    if "rect" in zone:
        q=zone["rect"];return q["x_min"]<=x<=q["x_max"] and q["y_min"]<=y<=q["y_max"]
    points=zone.get("polygon",[])
    if isinstance(points,dict):points=points.get("points",[])
    if not points:raise ValueError("Zone has no bounds/rect/polygon: "+str(zone.get("id")))
    pts=[(p["x"],p["y"]) if isinstance(p,dict) else tuple(p[:2]) for p in points]
    inside=False
    for (ax,ay),(bx,by) in zip(pts,pts[1:]+pts[:1]):
        if ((ay>y)!=(by>y)) and x<(bx-ax)*(y-ay)/(by-ay)+ax:inside=not inside
    return inside

def zone_ratio(zones,x,y,grants=()):
    ratio=1.
    for zone in zones:
        if not contains(zone,x,y):continue
        if zone["type"]=="no_go" or zone["type"]=="exclusive" and zone["id"] not in grants:return 0.
        if zone["type"]=="slow":
            value=float(zone.get("speed_ratio",0.4))
            if not 0.<value<=1.:raise ValueError("Invalid zone speed ratio")
            ratio=min(ratio,value)
    return ratio

def validate_inputs(map_yaml,waypoint_file,zones_file,entities_file):
    meta=read_mapping(map_yaml)
    if not all(k in meta for k in ("image","resolution","origin")):raise ValueError("Incomplete map metadata")
    image=Path(map_yaml).parent/meta["image"]
    if not image.is_file() or not image.stat().st_size:raise ValueError("Part 1 map image missing")
    if float(meta["resolution"])<=0:raise ValueError("Invalid map resolution")
    read_mapping(entities_file)
    return destinations(waypoint_file),zones(zones_file)
