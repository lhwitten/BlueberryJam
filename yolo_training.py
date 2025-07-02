from ultralytics import YOLO

model = YOLO("yolo11n.pt")

model.train(
data="/Users/jasper/Desktop/blueberry sorter/dataset1-yolov11/data.yaml", # Path to Roboflow-exported YAML
epochs=160,
patience=50,
imgsz=640,
batch=8, # Set your desired batch size here
device="mps" # 0 for GPU, -1 for CPU
)