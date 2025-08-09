from ultralytics import YOLO
from ultralytics.utils.plotting import plot_results
from typing import Optional, Callable

plot_results(
    file="/Users/jasper/Desktop/blueberry sorter/BlueberryJam/runs/segment/train7/results.csv",
    dir="",
    segment=True,
    pose=False,
    classify=False,
    on_plot=None
)