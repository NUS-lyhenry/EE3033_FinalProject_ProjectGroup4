#!/usr/bin/env python3
"""Select an actual COCO cup photograph for the rendered sensor, using real YOLO.

Input: official coco128 archive unpacked alongside official yolo26n.pt. This
prepares a texture; it does not replace inference in the running robot program.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from simulation_geometry import render_camera


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('assets', type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    model = YOLO(str(args.assets/'yolo26n.pt'))
    cup = next(k for k, v in model.names.items() if v == 'cup')
    candidates = []
    images = sorted((args.assets/'coco128/images/train2017').glob('*.jpg'))
    for path in images:
        result = model.predict(str(path), classes=[cup], conf=.7, imgsz=416, verbose=False)[0]
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            if min(x2-x1, y2-y1) < 35:
                continue
            texture = cv2.imread(str(path))[y1:y2, x1:x2]
            frame = render_camera([.95, .1, math.pi/2], texture, True)
            rendered = model.predict(frame, classes=[cup], conf=.6, imgsz=416, verbose=False)[0]
            score = max((float(b.conf.item()) for b in rendered.boxes), default=0.)
            candidates.append((score, path.name, texture))
    if not candidates:
        raise SystemExit('No suitable actual cup photograph; no detection results have been fabricated.')
    score, name, texture = max(candidates, key=lambda item:item[0])
    if score < .6:
        raise SystemExit(f'Rendered cup is not recognized at main confidence threshold: {score:.3f}')
    output = args.assets/'cup.png'
    cv2.imwrite(str(output), texture)
    cv2.imwrite(str(args.assets/'camera-check.jpg'), render_camera([.95, .1, math.pi/2], texture, True))
    metadata = dict(source='Ultralytics coco128 / COCO 2017', source_image=name,
        source_url='https://github.com/ultralytics/assets/releases/download/v0.0.0/coco128.zip',
        model_url='https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt',
        model_sha256=hashlib.sha256((args.assets/'yolo26n.pt').read_bytes()).hexdigest(),
        rendered_yolo_confidence=score, texture_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (args.assets/'provenance.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata), flush=True)


if __name__ == '__main__':
    main()
