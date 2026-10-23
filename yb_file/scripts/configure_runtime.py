"""Resolve portable TEAM configuration into ignored machine-local runtime files."""
from pathlib import Path
import yaml

def configure(root):
    source=root/'config/integration';output=root/'_local_only/config';output.mkdir(parents=True,exist_ok=True)
    data=yaml.safe_load((source/'system.yaml').read_text())
    def absolute(value):
        p=Path(value)
        return str(p if p.is_absolute() else root/p)
    for key in ('map_yaml','limits_file','waypoints_file','zones_file','database_path'):data[key]=absolute(data[key])
    data['perception']['weights']=absolute(data['perception']['weights'])
    data['nav2_files']={r:absolute(path) for r,path in data['nav2_files'].items()}
    out=output/'system.yaml';out.write_text(yaml.safe_dump(data,sort_keys=False));return out

if __name__=='__main__':print(configure(Path(__file__).resolve().parents[1]))
