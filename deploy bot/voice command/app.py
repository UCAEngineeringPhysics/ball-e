import json
import sys
import vosk
import numpy as np
import subprocess
from flask import Flask, render_template
from flask_sock import Sock

# Reduce print overhead
sys.stdout = sys.stderr

# --- Configuration ---
MODEL_PATH = "model"
SAMPLE_RATE = 16000

# --- Flask App Initialization ---
app = Flask(__name__)
sock = Sock(app)

# --- Vosk Model Loading ---
print("Loading Vosk model...")
try:
    model = vosk.Model(MODEL_PATH)
except Exception as e:
    print(f"Error loading model: {e}")
    print("Please make sure the 'model' folder is in the same directory and contains the Vosk model files.")
    exit(1)
print("Model loaded successfully.")

# --- Flask Routes ---
@app.route('/')
def index():
    """Serves the main HTML page."""
    return render_template('index.html')


@sock.route('/audio')
def audio_socket(ws):
    """Handles the WebSocket connection for audio streaming."""
    print("Client connected.")
    
    # Define your expected vocabulary/commands
    # Customize this list with the words you actually use
    vocabulary = '["start", "red", "blue", "green", "yellow", "orange", "purple", "black", "white", "brown", "pink", "gray", "turn on", "turn off", "lights", "light", "volume", "up", "down", "increase", "decrease", "set", "the", "to", "[unk]"]'
    
    recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE) # add ,vocabulary to use vocabulary settings
    recognizer.SetMaxAlternatives(0)  # Disable alternatives for speed
    recognizer.SetWords(False)  # Disable word timestamps
    recording = False

    try:
        while True:
            message = ws.receive()

            if isinstance(message, str):
                if message == 'start':
                    recording = True
                    recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE, vocabulary)
                    recognizer.SetMaxAlternatives(0)
                    recognizer.SetWords(False)
                    print("\n--- Recording Started ---")
                elif message == 'stop':
                    recording = False
                    print("\n--- Recording Stopped ---")
                    # Get any remaining text
                    final_result = json.loads(recognizer.FinalResult())
                    if final_result.get('text'):
                        final_text = final_result['text']
                        print(f"Final: {final_text}\n")
                        ws.send(json.dumps({'type': 'final', 'text': final_text}))

            elif isinstance(message, bytes) and recording:
                # Convert bytes to numpy array (int16 PCM format)
                audio_data = np.frombuffer(message, dtype=np.int16)
                
                # Convert to bytes for Vosk (it expects raw PCM bytes)
                audio_bytes = audio_data.tobytes()
                
                if recognizer.AcceptWaveform(audio_bytes):
                    result = json.loads(recognizer.Result())
                    if result.get('text'):
                        final_text = result['text']
                        print(f"\nFinal: {final_text}")

                        if "start" in final_text.lower():
                            print("\n[SYSTEM] Voice command recognized! Executing script...")
                            
                            command_sequence = (
                                "cd /home/ball-e/hailo-rpi5-examples/ && "
                                "source setup_env.sh && "
                                "cd /home/ball-e/ball-e/python_scripts/ && "
                                "python odom_course.py"
                            )
                            
                            subprocess.Popen(command_sequence, shell=True, executable='/bin/bash')
                        
                        ws.send(json.dumps({'type': 'final', 'text': final_text}))
                else:
                    partial_result = json.loads(recognizer.PartialResult())
                    if partial_result.get('partial'):
                        partial_text = partial_result['partial']
                        print(f"Partial: {partial_text}", end='\r')
                        ws.send(json.dumps({'type': 'partial', 'text': partial_text}))

    except Exception as e:
        print(f"An error occurred or client disconnected: {e}")
    finally:
        print("Client disconnected.")

# --- Main Execution ---
if __name__ == '__main__':
    print("Starting server on https://0.0.0.0:5000")
    print("Connect to this address from your phone's browser.")
    print("\nMake sure you have:")
    print("1. Generated SSL certificates (cert.pem and key.pem)")
    print("2. Downloaded and extracted a Vosk model to the 'model' folder")
    print("3. Installed required packages: flask, flask-sock, vosk, numpy")
    
    # Check if SSL certificates exist
    import os
    if not os.path.exists('cert.pem') or not os.path.exists('key.pem'):
        print("\n⚠️  Warning: SSL certificates not found!")
        print("Generate them with:")
        print("openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes")
        print("\nOr run without SSL (HTTP only - microphone won't work on most phones):")
        print("Remove the ssl_context parameter from app.run()")
    
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False,
            ssl_context=('cert.pem', 'key.pem'))

