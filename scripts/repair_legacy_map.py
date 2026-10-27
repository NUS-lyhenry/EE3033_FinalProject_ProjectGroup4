#!/usr/bin/env python3
"""Create an unreviewed corrected copy of a known old trinary map."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/ee3033_project'))
from ee3033_project.map_io import repair_legacy_map,validate_saved_map

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('map_yaml')
parser.add_argument('--output-root',help='Saved map directory; defaults to the source map directory parent')
args=parser.parse_args()
corrected=repair_legacy_map(args.map_yaml,args.output_root)
print(json.dumps(dict(source=str(Path(args.map_yaml).resolve()),corrected=corrected,
    validation=validate_saved_map(corrected or args.map_yaml)),indent=2))
