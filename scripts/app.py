from flask import Flask, render_template, Response, jsonify, request
import threading
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import os
import importlib.util
import sys
import docx

# Disable GPU
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

app = Flask(__name__)

# Global variables
simulation_running = False  # Flag to track if a simulation is running
physics_client = None  # Store the PyBullet client ID
env_initialized = False  # Flag to track environment initialization
log_messages = []
# List of available lid files
LID_FILES = {
    "Lid_15": "Lid_15.py",
    "Lid_60": "Lid_60.py",
    "Lid_120": "Lid_120.py",
    "Lid_square_90": "Lid_square_90.py",
    "Lid_var1": "Lid_var1.py",
    "Test": "Test.py"
    
}

def setup_environment():
    global physics_client, env_initialized
    if p.isConnected():
        p.disconnect()  # Ensure fresh connection
    if not env_initialized:  # Only initialize once
        physics_client = p.connect(p.GUI)  # Connect to the physics server
        p.setTimeStep(1 / 500.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        env_initialized = True

def capture_frame():
    
    if not p.isConnected():
        return None  # Return None if not connected to the physics server

    # Adjust camera position and FOV for wider view
    camera_eye = [0.8, -1.0, 0.6]  # X, Y, Z (move camera back and up)
    camera_target = [0.5, 0, 0.1]  # Focus point
    
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_eye,
        cameraTargetPosition=camera_target,
        cameraUpVector=[0, 0, 1]
    )
    
    # Wider FOV (75 degrees) and correct aspect ratio
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=75, 
        aspect=640/480,  # 4:3 aspect ratio
        nearVal=0.1, 
        farVal=3.0  # Increased view distance
    )
    
    width, height = 1280 , 720
    _, _, rgbImg, _, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    return np.reshape(rgbImg, (height, width, 4))[:, :, :3]

def generate_frames():
    while True:
        if p.isConnected():  # Only capture frames if connected to the physics server
            frame = capture_frame()
            if frame is not None:
                #_, buffer = cv2.imencode('.jpg', frame)
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 100])  # High quality
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.01)  # Reduce sleep time to increase frame rate

@app.route('/')
def index():
    return render_template('index.html')

def read_help_file():
    """Reads content from Help.docx and returns as text."""
    try:
        doc = docx.Document("Help.docx")
        content = "\n".join([para.text for para in doc.paragraphs])
        return content
    except Exception as e:
        return f"Error reading Help file: {e}"
    
@app.route('/help')
def get_help():
    """Returns the help content."""
    help_content = read_help_file()
    return jsonify({"help_content": help_content})

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_simulation(file_name):
    global simulation_running
    simulation_running = True

    # Load the selected script dynamically
    spec = importlib.util.spec_from_file_location("module.name", file_name)
    module = importlib.util.module_from_spec(spec)
    sys.modules["module.name"] = module
    spec.loader.exec_module(module)

    # Run the simulation
    if hasattr(module, 'setup_environment'):
        env = module.setup_environment()  # Let Lid_15.py handle environment setup
        if hasattr(module, 'run_simulation'):
            module.run_simulation(env)
    
    simulation_running = False

@app.route('/start_simulation', methods=['POST'])
def start_simulation():
    global simulation_thread, simulation_running
    data = request.json  # Access JSON data from the request
    file_key = data.get("file")

    if file_key in LID_FILES and not simulation_running:
        simulation_running = True
        file_name = LID_FILES[file_key]
        simulation_thread = threading.Thread(target=run_simulation, args=(file_name,), daemon=True)
        simulation_thread.start()
        time.sleep(1)  # Give time for simulation to initialize
        return jsonify({"status": f"Simulation started for {file_key}"})
    return jsonify({"status": "Simulation already running or invalid file selection"})

@app.route('/stop_simulation', methods=['POST'])
def stop_simulation():
    """Stops the currently running simulation."""
    global simulation_running

    if simulation_running:
        if p.isConnected():
            p.disconnect()  # Stop PyBullet simulation
        simulation_running = False  # Reset the flag
        return jsonify({"status": "Simulation stopped successfully"})
    
    return jsonify({"status": "No simulation is currently running"})
# Global variables for camera settings
camera_angle = 323.0
camera_distance = 1.3
camera_z = 0.7

@app.route('/update_camera', methods=['POST'])
def update_camera():
    """Updates the camera position dynamically."""
    global camera_angle, camera_distance, camera_z
    data = request.json
    
    camera_angle = float(data.get("angle", camera_angle))
    camera_distance = float(data.get("distance", camera_distance))
    camera_z = float(data.get("height", camera_z))

    return jsonify({"status": f"Camera updated: Angle {camera_angle}°, Distance {camera_distance}, Height {camera_z}"})

def capture_frame():
    """Captures a frame from PyBullet with the updated camera settings."""
    if not p.isConnected():
        return None

    global camera_angle, camera_distance, camera_z

    # Calculate camera position dynamically
    angle_rad = np.radians(camera_angle)
    camera_eye = [
        camera_distance * np.cos(angle_rad),
        camera_distance * np.sin(angle_rad),
        camera_z
    ]
    camera_target = [0.5, 0, 0.1]

    view_matrix = p.computeViewMatrix(camera_eye, camera_target, [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(75, 640/480, 0.1, 3.0)

    _, _, rgbImg, _, _ = p.getCameraImage(640, 480, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    return np.reshape(rgbImg, (480, 640, 4))[:, :, :3]



if __name__ == "__main__":
    # Initialize PyBullet simulation before starting Flask
    # setup_environment()
    app.run(host='0.0.0.0', port=5000)