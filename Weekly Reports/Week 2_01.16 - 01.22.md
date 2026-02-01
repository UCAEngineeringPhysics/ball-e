# ELIZABETH
## DONE
- Troubleshot communication between Pi and Realsense camera using basic frame and depth data test scripts and using simple test_detect_talker.py file from last semester.
- Tried to use previous pipeline from navigation scripts: 
	- Camera -> GStreamer Class -> Hailo -> callback() function
- After seeing issues with data speed, changed Realsense 480M cable for a 5000M cable.
- Was able to confirm GStreamer received frame data from Realsense camera.

> [!TIP]
> You'll need to specify input device when you run your python script, for example:
```console
# Make sure virtual env activated
python detection.py --input /dev/video0  # realsense rgb camera usually plays as /dev/video0
```
## TO DO 
- Continue working on getting object detection model running on AI Hat to recieve and process frame data from Realsense camera.
  
# MISIA
## DONE
- Made a 3D design for a mount for realsense d455 camera
- Started the design print

> [!WARNING]
> Sorry girl, I didn't see anything printed in the lab. -1

## TO DO
- Make any possibly needed changes to the camera mmount design
- Start designing 3D claw extension

# CALEB


# ERIC
## DONE
- Experimented with realsense and pyrealsense libraries

## TO DO 
- Continue work with realsense d455 camera
