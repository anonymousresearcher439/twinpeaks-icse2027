import sys
from string import Template
from pathlib import Path

import yaml

from drone_names import names


class MyDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, False)


SIMULATION_DIR = Path(__file__).parents[1] / "docker/simulations"

num = int(sys.argv[1])

template = Template((SIMULATION_DIR / "multi/drone.yaml.template").read_text())

for p in (SIMULATION_DIR / Path("multi/")).glob("*.yaml"):
    p.unlink(True)

depends_on = []
includes = []

for i in range(num):
    drone_name = names[i]
    local_mqtt_port = 1884 + i
    (SIMULATION_DIR / f"multi/{i}.yaml").write_text(
        template.substitute(
            n=i,
            sys_id=i + 1,
            local_mqtt_port=local_mqtt_port,
            drone_name=drone_name,
        )
    )
    depends_on.append(f"mavros_{i}")
    includes.append(f"./multi/{i}.yaml")

multi = yaml.safe_load((SIMULATION_DIR / "multi.yaml.template").read_text())

multi["services"]["px4"]["depends_on"] = depends_on
multi["services"]["px4"]["command"] = f"start_simulation_multi.sh {num}"
multi["include"] = includes

(SIMULATION_DIR / "multi.yaml").write_text(
    yaml.dump(multi, indent=2, Dumper=MyDumper, default_flow_style=False)
)
