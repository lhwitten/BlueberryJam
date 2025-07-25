from ultralytics import YOLO

# Initialize a new YOLO model for segmentation
model = YOLO('/Users/jasper/Desktop/blueberry sorter/BlueberryJam/runs/segment/train3/weights/best.pt')  # Load the segmentation base model

results = model.train(
    data='/Users/jasper/Desktop/blueberry sorter/Blueberry-Jam-Yolo.v7i.yolov11/data.yaml',  # Path to your dataset YAML file
    epochs=120,
    patience=90,
    imgsz=640,
    batch=8,
    device="mps",
    task='segment',  # Specify segmentation task
    # resume=True
)

print(results)