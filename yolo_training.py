from ultralytics import YOLO

# Initialize a new YOLO model for segmentation
model = YOLO('yolov8n-seg.pt')  # Load the segmentation base model

model.train(
    data='/Users/jasper/Desktop/blueberry sorter/jully-11-blueberry-training-data/data.yaml',  # Path to your dataset YAML file
    epochs=150,
    patience=70,
    imgsz=640,
    batch=8,
    device="mps",
    task='segment'  # Specify segmentation task
)