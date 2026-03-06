# Retrain an Object Detection Model using YOLO 
Do following on the desktop in Robotics Lab.

## Copy Dataset to Docker
```bash
cd ~
docker cp ~/balbuc_dataset/ hailo8_ai_sw_suite_2025-10_container:/local/workspace/datasets/
```

## Launch Docker

```bash
cd ~/hailo8_ai_sw_suite_2025-10_docker
./hailo_ai_sw_suite_docker_run.sh --resume
```
> [!NOTE]
> Your command line prompt became `(hailo_virtualenv) hailo@<hostname>:/local/workspace$`

## (In Docker) Launch the Retraining 
```bash
cd /local/workspace/hailo_model_zoo/training/yolov8
yolo detect train data=/local/workspace/datasets/balbuc_dataset/data.yaml model=yolov8m.pt name=balbuc_yolov8m epochs=50 batch=16
```
> [!TIP]
> use `model=yolov8s.pt` if you need a lighter weighted model.


### Validate the new checkpoint 

```bash
ls /local/workspace/datasets/balbuc_dataset/valid/images  # use any jpg file for next step
yolo predict task=detect source=/local/workspace/datasets/buckets_dataset/valid/images/IMG_4511_jpg.rf.a1ce9a90595b28c828cbe7cd098bca9d.jpg model=/local/workspace/hailo_model_zoo/training/yolov8/runs/detect/bucket_yolov8s/weights/best.pt
```

### Export the model to ONNX

```bash
yolo export model=/local/workspace/hailo_model_zoo/training/yolov8/runs/detect/balbuc_yolov8m/weights/best.pt imgsz=640 format=onnx opset=11
```

## Copy the ONNX to a dedicated directory

```bash
cd /local/workspace/hailo_model_zoo/training/yolov8/bucket_models
cp ../runs/detect/bucket_yolov8s/weights/best.onnx ./bucket_detector.onnx
```


## Convert the model to Hailo 

Use the Hailo Model Zoo command (this can take up to 30 minutes):

```bash
hailomz compile yolov8m --ckpt=/local/workspace/hailo_model_zoo/training/yolov8/runs/detect/balbuc_yolov8m/weights/best.onnx --hw-arch hailo8 --calib-path /local/workspace/datasets/balbuc_dataset/test/images/ --classes 8 --performance
```
> [!NOTE]
> This will take quite a while.
> Make a coffee, stretch your legs.
> But after the conversion,you will get the `yolov8m.hef` in current directory.
This is the model file can be used on the Hailo AI HAT on top of Raspberry Pi 5.
