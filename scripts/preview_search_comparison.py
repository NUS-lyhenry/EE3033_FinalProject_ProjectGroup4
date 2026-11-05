#!/usr/bin/env python3
"""Run the hardware-free preview with one explicit search policy for A/B tests.

The generated temporary ROS configuration differs only in search_strategy.
Camera pixels, detector, target estimator, map and navigation code are unchanged.
The ordinary preview's Docker/private-DDS guards remain in force.
"""
import argparse
import os
from pathlib import Path
import sys
import tempfile

import yaml

import run_algorithm_preview as preview


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--search-strategy',choices=('greedy','adaptive'),required=True)
    selected,remaining=parser.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires an explicitly isolated simulation DDS domain')
    if not Path('/.dockerenv').exists():
        raise SystemExit('Run only in the dedicated hardware-free Docker container')
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'src/ee3033_project/config/project.yaml').read_text())
    config['project_mission']['ros__parameters']['search_strategy']=selected.search_strategy
    directory=Path(tempfile.mkdtemp(prefix='preview-search-policy-',dir='/tmp'))
    path=directory/'project.yaml'
    path.write_text(yaml.safe_dump(config))

    class ComparisonConsole(preview.SimulationConsole):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            original=self.stage.popen
            def launch(command,**options):
                command=list(command)
                if command[:4]==['ros2','launch','ee3033_project','mission.launch.py']:
                    command.append('config:='+str(path))
                return original(command,**options)
            self.stage.popen=launch

    preview.SimulationConsole=ComparisonConsole
    sys.argv=[sys.argv[0],*remaining]
    preview.main()


if __name__=='__main__':main()
