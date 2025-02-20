import pybullet as p
import pybullet_data
import time
import os

# Disable GPU
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

# Configuration
LID_CONFIG = {
    "objects": {
        "cup": r"C:\Users\tanya\OneDrive\Desktop\MAI\Project Module\Robosort\3dmodels\Cup_15.obj",
        "lids": {
            "Lid_15": {
                "path": "/home/admin/RS-Project/3dmodels/Lid_15.obj",
                "grasp_position": [0.274, 0.095, 0.279]
            },
            "Lid_60": {
                "path": "/home/admin/RS-Project/3dmodels/Lid_60.obj",
                "grasp_position": [0.284, 0.221, 0.337]
            },
            "Lid_120": {
                "path": "/home/admin/RS-Project/3dmodels/Lid_120.obj",
                "grasp_position": [0.379, 0.126, 0.379]
            },
            "Lid_square_90": {
                "path": "/home/admin/RS-Project/3dmodels/Lid_square_90.obj",
                "grasp_position": [0.316, 0.158, 0.358]
            },
            "Lid_var_1": {
                "path": "/home/admin/RS-Project/3dmodels/Lid_var_1.obj",
                "grasp_position": [0.379, 0.147, 0.374]
            }
        }
    }
}

def setup_environment(lid_name):
    """Set up environment with selected lid"""
    lid_info = LID_CONFIG["objects"]["lids"][lid_name]
    
    if p.isConnected():
        p.disconnect()
        
    p.connect(p.GUI)
    p.setTimeStep(1/500.0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    
    # Load plane with dark color
    plane = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane, -1, rgbaColor=[0.1, 0.1, 0.2, 1])
    
    # Set camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=0.3,
        cameraYaw=50,
        cameraPitch=-30,
        cameraTargetPosition=[0.65, 0.184, 0.0]
    )
    
    # Load robot
    robot = p.loadURDF("franka_panda/panda.urdf", 
                      basePosition=[-0.5, 0, 0],
                      useFixedBase=True)
    
    # Load cup
    cup_visual = p.createVisualShape(p.GEOM_MESH, 
                                   fileName=LID_CONFIG["objects"]["cup"],
                                   meshScale=[1.5, 1.5, 1.5])
    cup_collision = p.createCollisionShape(p.GEOM_MESH,
                                         fileName=LID_CONFIG["objects"]["cup"],
                                         meshScale=[1.5, 1.5, 1.5])
    cup = p.createMultiBody(0, cup_collision, cup_visual, 
                          basePosition=[0.65, 0.184, 0.0])
    
    # Load selected lid
    lid_visual = p.createVisualShape(p.GEOM_MESH,
                                    fileName=lid_info["path"],
                                    meshScale=[1.5, 1.5, 1.5])
    lid_collision = p.createCollisionShape(p.GEOM_MESH,
                                          fileName=lid_info["path"],
                                          meshScale=[1.5, 1.5, 1.5],
                                          flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    lid = p.createMultiBody(5.0, lid_collision, lid_visual,
                          basePosition=[0.65, 0.184, 0.0])
    
    # Lid physics properties
    p.changeDynamics(lid, -1,
                    lateralFriction=50,
                    spinningFriction=30,
                    rollingFriction=20,
                    contactProcessingThreshold=0.0001,
                    restitution=0.0,
                    collisionMargin=0.002)
    
    # Create constraint
    constraint = p.createConstraint(lid, -1, cup, -1, 
                                   p.JOINT_FIXED, 
                                   [0, 0, 0], [0, 0, 0], [0, 0, 0])
    
    # Mark grasp position
    grasp_pos = lid_info["grasp_position"]
    marker_visual = p.createVisualShape(p.GEOM_SPHERE, 
                                        radius=0.01, 
                                        rgbaColor=[1, 0, 0, 1])
    marker = p.createMultiBody(0, baseVisualShapeIndex=marker_visual, 
                               basePosition=grasp_pos) 
    
    return {
        "robot": robot,
        "lid": lid,
        "cup": cup,
        "constraint": constraint,
        "grasp_position": lid_info["grasp_position"],
        "marker": marker
    }

def visualize_grasp_point(env):
    """Highlight grasp position"""
    grasp_pos = env["grasp_position"]
    p.addUserDebugText("Grasp Here", grasp_pos, textColorRGB=[1, 0, 0], textSize=1.2)

def move_to_target(robot, target_pos, ee_index, steps=200):
    """Move end-effector to target position"""
    for _ in range(steps):
        joint_positions = p.calculateInverseKinematics(robot, ee_index, target_pos)
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])
        p.stepSimulation()
        time.sleep(1 / 50.0)

def open_gripper(robot, left_finger, right_finger, steps=50):
    """Open gripper"""
    for _ in range(steps):
        p.setJointMotorControl2(robot, left_finger, p.POSITION_CONTROL, targetPosition=0.04, force=100)
        p.setJointMotorControl2(robot, right_finger, p.POSITION_CONTROL, targetPosition=0.04, force=100)
        p.stepSimulation()
        time.sleep(1 / 50.0)

def close_gripper(robot, left_finger, right_finger, steps=50, force=5000):
    """Close gripper with force maintenance"""
    target_pos = 0.005
    for _ in range(steps):
        p.setJointMotorControl2(robot, left_finger, p.POSITION_CONTROL, 
                               targetPosition=target_pos, force=force)
        p.setJointMotorControl2(robot, right_finger, p.POSITION_CONTROL, 
                               targetPosition=target_pos, force=force)
        p.stepSimulation()
        time.sleep(1 / 50.0)
    
    p.setJointMotorControl2(robot, left_finger, p.VELOCITY_CONTROL,
                           targetVelocity=0, force=force)
    p.setJointMotorControl2(robot, right_finger, p.VELOCITY_CONTROL,
                           targetVelocity=0, force=force)

def maintain_grip(robot, lid):
    """Maintain grip on lid"""
    if verify_grasp(robot, lid):
        p.setJointMotorControl2(robot, 9, p.VELOCITY_CONTROL, 
                               targetVelocity=0, force=3000)
        p.setJointMotorControl2(robot, 10, p.VELOCITY_CONTROL, 
                               targetVelocity=0, force=3000)
        return True
    return False

def execute_grasp_and_peel(env):
    """Full grasp and placement sequence with auto-reset"""
    grasp_pos = env["grasp_position"]
    success = False

    try:
        # ✅ Approach and grasp
        move_to_target(env["robot"], [grasp_pos[0], grasp_pos[1], grasp_pos[2] + 0.02], 11, steps=50)
        open_gripper(env["robot"], 9, 10)
        move_to_target(env["robot"], grasp_pos, 11)
        close_gripper(env["robot"], 9, 10, force=5000)

        if verify_grasp(env["robot"], env["lid"]):
            print("Grasp successful!")
            p.removeConstraint(env["constraint"])
            
            # ✅ Lift and move to drop position
            lift_pos = [grasp_pos[0], grasp_pos[1], grasp_pos[2] + 0.2]
            move_to_target(env["robot"], lift_pos, 11, steps=80)
            
            # ✅ Move to ground position
            drop_pos = [0.3, -0.3, 0.02]  # Adjusted drop position
            move_to_target(env["robot"], drop_pos, 11, steps=100)
            
            # ✅ Release lid
            open_gripper(env["robot"], 9, 10)
            move_to_target(env["robot"], [drop_pos[0], drop_pos[1], drop_pos[2] + 0.08], 11)
            
            success = True
            print("Lid placed successfully!")

    finally:
        if success:
            print("\nResetting simulation for next selection...\n")
            time.sleep(1)  # Short delay before reset
            p.resetSimulation()  # Reset the simulation
            time.sleep(0.5)  # Ensure clean reset
            main()  # Restart main function for new lid selection


def verify_grasp(robot, lid):
    """Check grasp contact points"""
    return len(p.getContactPoints(bodyA=robot, bodyB=lid)) > 0

def main():
    while True:
        print("\nAvailable lids:")
        lids = list(LID_CONFIG["objects"]["lids"].keys())
        for idx, lid_name in enumerate(lids, 1):
            print(f"{idx}. {lid_name}")

        choice = input("\nSelect lid (number/q): ").strip()
        if choice.lower() == 'q':
            break
            
        try:
            lid_idx = int(choice) - 1
            selected_lid = lids[lid_idx]
        except (ValueError, IndexError):
            print("Invalid selection")
            continue
            
        env = setup_environment(selected_lid)
        button_visualize = p.addUserDebugParameter("Show Grasp", 1, 0, 0)
        button_execute = p.addUserDebugParameter("Execute", 1, 0, 0)

        while p.isConnected():
            if p.readUserDebugParameter(button_visualize) == 1:
                visualize_grasp_point(env)
                
            if p.readUserDebugParameter(button_execute) == 1:
                if execute_grasp_and_peel(env):
                    break  # Exit to main menu after success
                    
            if env.get("lid"):
                maintain_grip(env["robot"], env["lid"])
                
            time.sleep(0.01)
            
        print("\n" + "="*40)
        print("Simulation reset! Choose another lid or press 'q' to quit")
        print("="*40)

if _name_ == "_main_":
    main()