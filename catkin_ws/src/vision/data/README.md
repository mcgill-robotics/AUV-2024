All data must be put into the format expected by the YOLO v7 model.

That format is:
    - An "images" folder containing all training images
    - In the same folder as the images folder, a "labels" folder containing one .txt file for each image in /images
    - Each .txt file should have the same name as the image it annotates.

The format for annotation in the .txt files is:
    - each line describes one bounding box: "class, x_center, y_center, width, height"
    - The fields x, y, width, and height should be normalized to the dimensions of the image i.e. between 0 and 1

Pre-processing:

pip install -U albumentations

NOTE: All images and labels to be pre-processed should be in the YOLO format, and their location on the filesystem passed as a CLI argument to preprocess.py

Data in the argument folder is augmented and adapted to the target image size (modified versions of these images and their corresponding label files are generated)
All of the data is outputted into the /clean-data folder and split into the train, test, and val subfolders


Data augmentation is necessary since YOLO recommends 2-3k samples images per class.


YOLOv8:

pip install ultralytics

from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

# Use the model
results = model.train(data="coco128.yaml", epochs=3)  # train the model
results = model.val()  # evaluate model performance on the validation set
results = model("https://ultralytics.com/images/bus.jpg")  # predict on an image
success = model.export(format="onnx")  # export the model to ONNX format