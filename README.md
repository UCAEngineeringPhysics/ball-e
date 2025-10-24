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
python test_ball_detect_talker_revision.py --hef-path ../models/balle_detector.hef --labels-json ../models/ball_bucket.json --input rpi
```
> [!IMPORTANT]
> Use `--hef-path` to specify model.
> Use `--labels-json` to specify labels.

## TODO
[] Get more data.