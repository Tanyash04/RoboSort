import pybullet as p
import pybullet_data
import numpy as np
import time
import os



def setup_environment():
    """
    Set up the PyBullet environment with a robot, cup, and lid.
    Returns:
        dict: A dictionary containing the object IDs of the plane, robot, cup, lid, and constraint.
    """
    # Start PyBullet simulation
    if p.isConnected():
        p.disconnect()
    p.connect(p.GUI)
    p.setTimeStep(1 / 300.0)  # Increase simulation accuracy

    # Set up the environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For basic assets like the plane
    p.setGravity(0, 0, -9.8)

    # Load a ground plane
    plane = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane, -1, rgbaColor=[0.2, 0.1, 0.1, 1],specularColor=[2, 2, 2] )
    # Load the robot
    robot = p.loadURDF(
        "franka_panda/panda.urdf",
        basePosition=[-0.5, 0, 0],  # Robot positioned on the left
        useFixedBase=True
    )
     # Define the real camera position (same as PyBullet's virtual camera)
    camera_eye = [0.8, 1.0, 1.5]  # Camera Position (X, Y, Z)
    camera_target = [0.0, 0, 0.0]  # Where the camera looks

    # Load a 3D camera model (update with your model path)
    camera_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=r"C:\Users\tanya\Downloads\updated robosort\3dmodels\Camera.obj",  # Correct path
        meshScale=[0.1, 0.1, 0.1]  # Adjust scale to fit
    )

    camera_collision = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=r"C:\Users\tanya\Downloads\updated robosort\3dmodels\Camera.obj",
        meshScale=[0.1, 0.1, 0.1]
    )

    # Create the camera model in PyBullet
    camera_body = p.createMultiBody(
        baseMass=0,  # No physics for the camera
        baseCollisionShapeIndex=camera_collision,
        baseVisualShapeIndex=camera_visual,
        basePosition=camera_eye
    )


    # Set default positions for the cup and lid
    default_cup_position = [0.650, 0.184, 0.0]
    default_lid_position = [0.650, 0.184, 0.0]

    # Load the cup .obj
    cup_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=r"./3dmodels/Cup_15.obj",
        meshScale=[1.5, 1.5, 1.5]
    )
    cup_collision = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=r"./3dmodels/Cup_15.obj",
        meshScale=[1.5, 1.5, 1.5]
    )
    cup = p.createMultiBody(
        baseMass=0,  # Static mass to prevent the cup from moving
        baseCollisionShapeIndex=cup_collision,
        baseVisualShapeIndex=cup_visual,
        basePosition=default_cup_position
    )

    # Load the lid .obj
    lid_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=r"./3dmodels/Lid_60.obj",
        meshScale=[1.5, 1.5, 1.5],
        # rgbaColor=[1.0, 0.0, 0.0, 1.0]  # Red color for the lid
    )
    lid_collision = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=r"./3dmodels/Lid_60.obj",
        meshScale=[1.5, 1.5, 1.5],
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH
    )
    lid = p.createMultiBody(
        baseMass=5.0,  # Small mass for the lid
        baseCollisionShapeIndex=lid_collision,
        baseVisualShapeIndex=lid_visual,
        basePosition=default_lid_position
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

def move_to_target(robot, target_pos, ee_index, steps=200):
    """Moves the end-effector to the target grasp position."""
    for _ in range(steps):
        joint_positions = p.calculateInverseKinematics(robot, ee_index, target_pos)
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])
        p.stepSimulation()
        time.sleep(0.001)

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

def verify_grasp(robot, lid):
    """Checks if the gripper successfully grasped the lid."""
    contact_points = p.getContactPoints(bodyA=robot, bodyB=lid)
    print(f"Contact points: {len(contact_points)}")  # Print the number of contact points
    return bool(contact_points)

# if __name__ == "__main__":
def run_simulation(env):
    # env = setup_environment()
    robot = env["robot"]
    lid = env["lid"]
    constraint = env["constraint"]
    ee_index = 11
    left_finger, right_finger = 9, 10

    grasp_x, grasp_y, grasp_z = 0.284, 0.221, 0.322  

    grasp_position = [grasp_x, grasp_y, grasp_z]
    grasp_marker_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0, 0, 1, 1])
    p.createMultiBody(baseVisualShapeIndex=grasp_marker_visual, basePosition=[grasp_x, grasp_y, grasp_z ])

    # ✅ Move above grasp position
    move_to_target(robot, [grasp_x, grasp_y, grasp_z + 0.1], ee_index)  
    open_gripper(robot, left_finger, right_finger)

    # ✅ Move to grasp position
    move_to_target(robot, grasp_position, ee_index)  
    close_gripper(robot, left_finger, right_finger)

    if verify_grasp(robot, lid):
        print("✅ Grasp successful!")
        p.removeConstraint(constraint)  

        # ✅ Lift the lid
        lift_height = 0.20
        lift_steps = 50
        for i in range(lift_steps):
            new_pos = [grasp_x, grasp_y, grasp_z + (i / lift_steps) * lift_height]
            move_to_target(robot, new_pos, ee_index, steps=1)
            p.stepSimulation()
            time.sleep(1 / 50.0)

        print("✅ Lid lifted successfully!")

        # ✅ Move to drop location
        drop_x, drop_y, drop_z = 0.3, -0.3, 0.20  
        move_to_target(robot, [drop_x, drop_y, drop_z], ee_index, steps=100)  

        # ✅ Lower the lid gently onto the floor
        for i in range(50):  
            new_pos = [drop_x, drop_y, drop_z - (i / 50) * 0.13]  
            move_to_target(robot, new_pos, ee_index, steps=1)
            p.stepSimulation()
            time.sleep(1 / 50.0)

        # ✅ Open gripper to release the lid
        open_gripper(robot, left_finger, right_finger)
        print("✅ Lid placed successfully on the floor!")

        # ✅ NEW FIX: Stabilize the lid
        p.changeDynamics(lid, -1, mass=10, lateralFriction=2000, spinningFriction=1000, rollingFriction=1000)
        p.resetBaseVelocity(lid, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

        # ✅ WAIT to ensure the lid settles on the floor
        time.sleep(0.5)

        # ✅ Move back to initial position
        home_x, home_y, home_z = -0.5, 0, 0.5  
        move_to_target(robot, [home_x, home_y, home_z], ee_index, steps=150)
        print("✅ Robot returned to initial position!")

    else:
        print("❌ Grasp failed, retrying...")

    time.sleep(1)