# ball-e

## Usage
### 1 Activate venv
```console
cd ~/hailo-rpi5-examples/
source setup_env.sh
```

### 2 Start detection
```console
cd ~/ball-e/python_scripts
python test_arm_talker.py --hef-path ../models/yolov8s_11_4_25.hef --labels-json ../models/ball_bucket.json --input rpi
```

> [!IMPORTANT]
> Use `--hef-path` to specify model.
> Use `--labels-json` to specify labels.

## TODO
[] Get more data.
