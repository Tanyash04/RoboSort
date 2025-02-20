import pybullet as p
import pybullet_data
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.metrics import MeanSquaredError
import time
from tensorflow.keras.preprocessing.image import img_to_array, array_to_img
from tensorflow.keras.preprocessing.image import smart_resize
import os
import cv2

# Disable GPU
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

def setup_environment():
    """
    Set up the PyBullet environment with a robot, cup, and lid.
    Returns:
        dict: A dictionary containing the object IDs of the plane, robot, cup, lid, and sliders.
    """
    # Start PyBullet simulation
    p.connect(p.GUI)
    p.setTimeStep(1 / 500.0)  # Increase simulation accuracy

    # Set up the environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For basic assets like the plane
    p.setGravity(0, 0, -9.8)

    # Create a large black plane for the background
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[10, 10, 0.01],  # Make it large enough to cover the view
        rgbaColor=[0, 0, 0, 1]  # Pure black color
    )
    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[10, 10, 0.01]
    )
    plane = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 0]
    )

    # Load the robot
    robot = p.loadURDF(
        "franka_panda/panda.urdf",
        basePosition=[-0.5, 0, 0],  # Robot positioned on the left
        useFixedBase=True
    )

    # Set default positions for the cup and lid
    default_cup_position = [0.421, 0.284, 0.0]
    default_lid_position = [0.411, 0.284, 0.0]

    # Load the cup .obj
    cup_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=r"C:\Users\ADMIN\Desktop\train_robosort\lids\Cupmesh(1).obj",
        meshScale=[4.0, 4.0, 4.0]
    )
    cup_collision = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=r"C:\Users\ADMIN\Desktop\train_robosort\lids\Cupmesh(1).obj",
        meshScale=[4.0, 4.0, 4.0]
    )
    cup = p.createMultiBody(
        baseMass=0,  # Static mass to prevent the cup from moving
        baseCollisionShapeIndex=cup_collision,
        baseVisualShapeIndex=cup_visual,
        basePosition=default_cup_position,
        baseOrientation=[0, 0, 0, 1]
    )

    # Load the lid .obj
    lid_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=r"C:\Users\ADMIN\Desktop\train_robosort\lids\Lidmesh(1).obj",
        meshScale=[4.0, 4.0, 4.0],
        rgbaColor=[1.0, 0.0, 0.0, 1.0]  # Red color for the lid
    )
    lid_collision = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=r"C:\Users\ADMIN\Desktop\train_robosort\lids\Lidmesh(1).obj",
        meshScale=[4.0, 4.0, 4.0],
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH
    )
    lid = p.createMultiBody(
        baseMass=5.0,  # Small mass for the lid
        baseCollisionShapeIndex=lid_collision,
        baseVisualShapeIndex=lid_visual,
        basePosition=default_lid_position,
    )

    # Increase lid friction to prevent accidental movement
    p.changeDynamics(lid, -1, lateralFriction=50, spinningFriction=30, rollingFriction=20)
    p.changeDynamics(lid, -1, contactProcessingThreshold=0.0001)  # Detects small contacts
    p.changeDynamics(lid, -1, restitution=0.0, collisionMargin=0.002)  # Prevents sinking

    # Enable collision detection between gripper and lid
    p.setCollisionFilterPair(robot, lid, -1, -1, enableCollision=1)

    # Keep lid stable before grasping
    lid_cup_constraint = p.createConstraint(lid, -1, cup, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

    return {
        "plane": plane,
        "robot": robot,
        "cup": cup,
        "lid": lid,
        "constraint": lid_cup_constraint
    }

def log_predictions(pred_x, pred_y, actual_x, actual_y, actual_z):
    with open("prediction_log.txt", "a") as f:
        f.write(f"{pred_x:.3f},{pred_y:.3f},{actual_x:.3f},{actual_y:.3f},{actual_z:.3f}\n")


def capture_image():
    """
    Capture an image from the simulation environment.
    Returns:
        np.array: The captured image.
    """
    view_matrix = p.computeViewMatrix([0.5, -0.5, 0.4], [0.5, 0, 0.1], [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(60, 1.0, 0.1, 1.5)
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=228, 
        height=228,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )
    rgb_img = np.reshape(rgbImg, (height, width, 4))[:, :, :3] / 255.0  # Normalize to [0, 1]
    return rgb_img

def load_cnn_model(model_path):
    """
    Load the pre-trained CNN model.
    Args:
        model_path (str): Path to the model file.
    Returns:
        keras.Model: The loaded model.
    """
    custom_objects = {
        'mse': MeanSquaredError,
    }
    return load_model(model_path, custom_objects=custom_objects)

def predict_grasp_point(model, image):
    # Preprocess the image
    img = cv2.resize(image, (128, 128))
    img = img / 255.0  # Normalize pixel values
    input_img = np.expand_dims(img, axis=0)
    
    # Predict
    pred_x, pred_y = model.predict(input_img)[0]
    
    return pred_x, pred_y

def open_gripper(robot, left_finger, right_finger, steps=50):
    """Opens the gripper to prepare for grasping."""
    for _ in range(steps):
        p.setJointMotorControl2(robot, left_finger, p.POSITION_CONTROL, targetPosition=0.04, force=100)
        p.setJointMotorControl2(robot, right_finger, p.POSITION_CONTROL, targetPosition=0.04, force=100)
        p.stepSimulation()
        time.sleep(1 / 50.0)

def close_gripper(robot, left_finger, right_finger, steps=50, force=3000):
    """Closes the gripper with stronger force for better grasp stability."""
    for _ in range(steps):
        p.setJointMotorControl2(robot, left_finger, p.POSITION_CONTROL, targetPosition=0.005, force=force)
        p.setJointMotorControl2(robot, right_finger, p.POSITION_CONTROL, targetPosition=0.005, force=force)
        p.stepSimulation()
        time.sleep(1 / 50.0)

def move_to_target(robot, target_pos, ee_index, steps=200):
    """Moves end-effector to target position while avoiding collisions."""
    for _ in range(steps):
        joint_positions = p.calculateInverseKinematics(robot, ee_index, target_pos)
        for i in range(len(joint_positions)):  # Ensure we iterate over all joints
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])
        p.stepSimulation()
        time.sleep(1 / 50.0)

def verify_grasp(robot, lid):
    """Checks if the gripper successfully grasped the lid."""
    contact_points = p.getContactPoints(bodyA=robot, bodyB=lid)
    print(f"Contact points: {len(contact_points)}")  # Print the number of contact points
    return bool(contact_points)

def mark_grasp_point(grasp_position, color=[1, 1, 0]):
    """
    Mark the grasp point with a debug point.
    Args:
        grasp_position (list): The [x, y, z] coordinates of the grasp point.
        color (list): The RGB color of the debug point.
    """
    p.addUserDebugPoints([grasp_position], [color], pointSize=10)

def convert_prediction_to_simulation_coords(pred_x, pred_y):
    # Define the range for the lid
    x_min, x_max = 0.2, 0.6  # Adjust based on your workspace
    y_min, y_max = -0.3, 0.3
    z = 0.3  # Fixed z-coordinate
    
    # Convert normalized predictions to simulation coordinates
    grasp_x = x_min + pred_x * (x_max - x_min)
    grasp_y = y_min + pred_y * (y_max - y_min)
    
    return grasp_x, grasp_y, z


if __name__ == "__main__":
    custom_objects = {
        'mse': MeanSquaredError,
    }

    model_path = r'C:\Users\ADMIN\Desktop\train_robosort\grasp_point_model_adj.h5'
    model = load_model(model_path, custom_objects=custom_objects)

    env = setup_environment()
    robot = env["robot"]
    lid = env["lid"]
    constraint = env["constraint"]
    ee_index = 11
    left_finger, right_finger = 9, 10

    lid_grasped = False

    while p.isConnected():
        p.stepSimulation()

        if lid_grasped:
            print("🔒 Holding the lid tightly.")
            close_gripper(robot, left_finger, right_finger, force=5000)
            time.sleep(1)
            continue

        # Capture image and predict grasp point
        image = capture_image()
        pred_x, pred_y = predict_grasp_point(model, image)
        grasp_x, grasp_y, grasp_z = convert_prediction_to_simulation_coords(pred_x, pred_y)
        grasp_position = [grasp_x, grasp_y, grasp_z]

        print(f"Predicted normalized grasp point: ({pred_x:.2f}, {pred_y:.2f})")
        print(f"Converted grasp position: ({grasp_x:.3f}, {grasp_y:.3f}, {grasp_z:.3f})")

        # Mark the grasp point with a yellow dot
        mark_grasp_point(grasp_position, color=[1, 1, 0])

        # Log predictions
        log_predictions(pred_x, pred_y, grasp_x, grasp_y, grasp_z)

        # Move to grasp position and attempt to grasp
        move_to_target(robot, [grasp_x, grasp_y, grasp_z + 0.1], ee_index)
        open_gripper(robot, left_finger, right_finger)
        move_to_target(robot, grasp_position, ee_index)
        close_gripper(robot, left_finger, right_finger)

        if verify_grasp(robot, lid):
            print("✅ Grasp successful!")
            lid_grasped = True
            close_gripper(robot, left_finger, right_finger, force=5000)
            p.removeConstraint(constraint)  # Remove the constraint holding the lid down
            # Lift the lid with a controlled motion
            lift_height = 0.15
            lift_steps = 50
            for i in range(lift_steps):
                new_pos = [grasp_x, grasp_y, grasp_z + (i / lift_steps) * lift_height]
                move_to_target(robot, new_pos, ee_index, steps=1)
                p.stepSimulation()
                time.sleep(1 / 50.0)
        else:
            print("Grasp failed, trying again...")

        time.sleep(1)
