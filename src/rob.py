# Import the InferencePipeline object
from inference import InferencePipeline
# Import the built in render_boxes sink for visualizing results
from inference.core.interfaces.stream.sinks import render_boxes

# initialize a pipeline object
pipeline = InferencePipeline.init(
    model_id="blueberry-jam-yolo/4", # Roboflow model to use
    video_reference=0, # Path to video, device id (int, usually 0 for built in webcams), or RTSP stream url
    on_prediction=render_boxes, # Function to run after each prediction
)
pipeline.start()
pipeline.join()


# from inference import get_model
# import supervision as sv
# import cv2

# # define the image url to use for inference
# image_file = '/Users/jasper/Desktop/blueberry sorter/fri-Jully-11-underipe/capture_20250711_165832.jpg'
# image = cv2.imread(image_file)

# # load a pre-trained yolov8n model
# model = get_model(model_id="blueberry-jam-yolo/4")

# # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
# results = model.infer(image)[0]

# # load the results into the supervision Detections api
# detections = sv.Detections.from_inference(results)

# # create supervision annotators
# bounding_box_annotator = sv.BoxAnnotator()
# label_annotator = sv.LabelAnnotator()


# # annotate the image with our inference results
# annotated_image = bounding_box_annotator.annotate(
#     scene=image, detections=detections)
# annotated_image = label_annotator.annotate(
#     scene=annotated_image, detections=detections)

# # display the image
# sv.plot_image(annotated_image)