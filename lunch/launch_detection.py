#!/bin/bash

# Set up environment manually (required for env -i and cron)
export HOME=/home/ball-e
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH="$HOME"

# Optional: Log to a file to see if script starts
echo "== Script started at $(date) ==" >> $HOME/detection.log
whoami >> $HOME/detection.log
which python3 >> $HOME/detection.log

# Source Hailo SDK env only if it exists
if [ -f "$HOME/hailo-rpi5-examples/setup_env.sh" ]; then
    source "$HOME/hailo-rpi5-examples/setup_env.sh"
else
    echo "⚠️ Hailo SDK env file not found" >> $HOME/detection.log
fi

# Optional: Activate venv if needed
# source $HOME/hailo-rpi5-examples/venv_hailo_rpi_examples/bin/activate

# Run detection script
#/usr/bin/python3 $HOME/python_scripts/detection.py --input rpi >> $HOME/detection.log 2>&1
/usr/bin/python3 -u /home/ball-e/python_scripts/detection.py --input rpi >> /home/ball-e/detection.log 2>&1
