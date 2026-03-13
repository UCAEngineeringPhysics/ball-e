# Retrain YOLOv8 Model using Custom Dataset (balbuc)

## 1. Download Roboflow Dataset
1. "Project" -> "Versions" -> "Download Dataset" -> "Image and Annotation Format: YOLOv8" -> "Download zip to computer"
2. Double click downloaded zip, extract dataset to `project1.v<#>.yolov8` folder.
3. Move the extracted data folder to docker accessible folder, for example:
```sh
mv ~/Downloads/project1.v15.yolov8 ~/balbuc_datasets/bb_data-0312
```

## 2. Retrain Model
1. Build docker image
   ```sh
   cd hailo_model_zoo/training/yolov8
   docker build --build-arg timezone=`cat /etc/timezone` -t yolov8:v0 .
   ```
2. Start docker
   ```sh
   docker run --name "yolov8_retrain_docker" -it --gpus all --ipc=host -v  /home/ball-e/balbuc_datasets:/workspace/balbuc_datasets yolov8:v0
   ```
3. Start training
   ```sh
   yolo detect train data=/workspace/balbuc_datasets/bb_data-0312/data.yaml model=yolov8s.pt name=balbuc0312_yolov8s epochs=100 batch=16
   ```
 > [!TIP]
 > You can set `model=yolov8m.pt`
