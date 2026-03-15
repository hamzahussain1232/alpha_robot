# ML Training Pipeline (Custom Object Detector)

This project now supports a full custom YOLO training flow for your real robot.

## 1. Start Camera

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch articubot_one camera.launch.py video_device:=/dev/video0
```

## 2. Capture Dataset Images Per Class

Run for each class separately (change `class_name`):

```bash
ros2 launch articubot_one capture_dataset.launch.py \
  class_name:=medicine_bottle \
  max_images:=400 \
  save_every_n_frames:=4
```

Suggested classes:
- `medicine_bottle`
- `medicine_box`
- `cup`
- `remote`
- `book`

Images are saved to:
- `assets/ml/dataset/images_all`

## 3. Label Images (YOLO format)

Use any labeling tool (Label Studio, CVAT, LabelImg, Roboflow export).

Required output:
- image files in `assets/ml/dataset/images_all`
- YOLO `.txt` labels in `assets/ml/dataset/labels_all`
- each label file name must match image stem (e.g., `img1.jpg` + `img1.txt`)

## 4. Build train/val/test splits + dataset.yaml

```bash
python3 ~/ros2_ws/src/articubot_one/scripts/prepare_yolo_dataset.py \
  --dataset-root ~/ros2_ws/src/articubot_one/assets/ml/dataset \
  --classes medicine_bottle medicine_box cup remote book
```

This creates:
- `assets/ml/dataset/images/{train,val,test}`
- `assets/ml/dataset/labels/{train,val,test}`
- `assets/ml/dataset/dataset.yaml`

## 5. Train YOLO

In your ML Python environment:

```bash
pip install ultralytics
yolo detect train \
  data=~/ros2_ws/src/articubot_one/assets/ml/dataset/dataset.yaml \
  model=yolo11n.pt \
  imgsz=640 \
  epochs=100 \
  batch=16 \
  device=cpu
```

Best model path typically:
- `runs/detect/train/weights/best.pt`

## 6. Plug trained model into ROS detector

Update:
- `config/detector_params_ml_real.yaml`
- `config/detector_params_ml_sim.yaml`

Set:
- `model_path` -> your `best.pt`
- `target_labels` -> your trained classes

## 7. Run ML vision pipeline

Real/laptop camera:

```bash
ros2 launch articubot_one ml_vision_pipeline.launch.py hardware_mode:=real launch_camera:=true
```

Simulation (camera already from Gazebo):

```bash
ros2 launch articubot_one ml_vision_pipeline.launch.py hardware_mode:=sim launch_camera:=false
```

## 8. Verify detections

```bash
ros2 topic echo /perception/detections
```

Then test intent flow:

```bash
ros2 topic echo /task/object_result
```
