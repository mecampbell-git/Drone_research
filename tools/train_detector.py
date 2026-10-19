#!/usr/bin/env python3
"""
Fine-tune a YOLO detector and install it for object_detector.py.

Trains with Ultralytics, writes the run (weights, plots, metrics) to ~/yolo_runs/<name>/,
and copies the best weights to ~/.cache/yolo/<name>.pt, where object_detector.py finds
them by bare name (model:=<name>.pt). Datasets are downloaded to Ultralytics' datasets_dir
(~/datasets on this machine; `yolo settings` shows it).

Usage:
  tools/train_detector.py                          # VisDrone, yolo26s, 1280 px, 100 epochs (~5-6 h)
  tools/train_detector.py --epochs 1 --fraction 0.05 --name smoke_test   # quick check
  tools/train_detector.py --resume                 # continue an interrupted run (same --name)
"""

import argparse
import os
import shutil

from ultralytics import YOLO

MODEL_DIR = os.path.expanduser('~/.cache/yolo')  # object_detector.py's model_dir
RUNS_DIR = os.path.expanduser('~/yolo_runs')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--data', default='VisDrone.yaml',
                        help='dataset yaml (built-in name or path)')
    parser.add_argument('--model', default='yolo26s.pt',
                        help=f'starting weights; bare names are looked up in {MODEL_DIR}')
    parser.add_argument('--name', default='visdrone_yolo26s', help='run and output name')
    parser.add_argument('--imgsz', type=int, default=1280,
                        help='training size; match object_detector.py imgsz')
    parser.add_argument('--epochs', type=int, default=100)
    # Not AutoBatch (-1): at 1280 px it profiles ~4.8 GB/image and picks 1 on the 16 GB
    # RTX 5070 Ti. Measured on VisDrone with yolo26s: 2 GB/image, 13.2 GB peak at batch 6.
    # Ultralytics accumulates gradients to an effective batch of 64 either way.
    parser.add_argument('--batch', type=int, default=5,
                        help='images per step (5 fits yolo26s at 1280 px in 16 GB)')
    parser.add_argument('--fraction', type=float, default=1.0,
                        help='fraction of the training set to use')
    parser.add_argument('--workers', type=int, default=8, help='dataloader processes')
    parser.add_argument('--resume', action='store_true',
                        help='continue ~/yolo_runs/<name> from its last.pt')
    args = parser.parse_args()

    run_dir = os.path.join(RUNS_DIR, args.name)
    if args.resume:
        results = YOLO(os.path.join(run_dir, 'weights', 'last.pt')).train(resume=True)
    else:
        model = args.model
        if '/' not in model:
            os.makedirs(MODEL_DIR, exist_ok=True)
            model = os.path.join(MODEL_DIR, model)
        results = YOLO(model).train(
            data=args.data, imgsz=args.imgsz, epochs=args.epochs, batch=args.batch,
            fraction=args.fraction, workers=args.workers,
            project=RUNS_DIR, name=args.name, exist_ok=True)

    best = os.path.join(results.save_dir, 'weights', 'best.pt')
    installed = os.path.join(MODEL_DIR, f'{args.name}.pt')
    shutil.copy(best, installed)
    print(f'Installed {best} as {installed}\n'
          f'Use it with: ros2 run px4_offboard object_detector.py '
          f'--ros-args -p model:={args.name}.pt')


if __name__ == '__main__':
    main()
