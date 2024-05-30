from ultralytics import YOLO
 
# Load the model.
model = YOLO('yolov8n.pt')
 
# Training.
results = model.train(
   data='dataset_v8.yaml',
   imgsz=640,
   epochs=10,
   batch=8,
   name='yolov8n_v8_50e'
)