import pybullet as p
import pybullet_data
import time

def setup_environment(lid_path, cup_path, grasp_position, cup_texture, lid_texture):
    """Set up environment with given lid path, cup path, and grasp position"""
    
    if p.isConnected():
        p.disconnect()
        
    p.connect(p.GUI)
    p.setTimeStep(1/1000.0)
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
    
    # ✅ Load cup with texture
    cup_visual = p.createVisualShape(p.GEOM_MESH, fileName=cup_path, meshScale=[1.5, 1.5, 1.5])
    cup_collision = p.createCollisionShape(p.GEOM_MESH, fileName=cup_path, meshScale=[1.5, 1.5, 1.5])
    cup = p.createMultiBody(0, cup_collision, cup_visual, basePosition=[0.65, 0.184, 0.0])


    # ✅ Load lid with texture
    lid_visual = p.createVisualShape(p.GEOM_MESH, fileName=lid_path, meshScale=[1.5, 1.5, 1.5])
    lid_collision = p.createCollisionShape(p.GEOM_MESH, fileName=lid_path, meshScale=[1.5, 1.5, 1.5])
    lid = p.createMultiBody(5.0, lid_collision, lid_visual, basePosition=[0.65, 0.184, 0.0])

  
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
    marker_visual = p.createVisualShape(p.GEOM_SPHERE, 
                                        radius=0.01, 
                                        rgbaColor=[1, 0, 0, 1])
    marker = p.createMultiBody(0, baseVisualShapeIndex=marker_visual, 
                               basePosition=grasp_position) 
    
    return {
        "robot": robot,
        "lid": lid,
        "cup": cup,
        "constraint": constraint,
        "grasp_position": grasp_position,
        "marker": marker
    }
def move_to_target(robot, target_pos, ee_index, steps=200):
    """Moves the end-effector to the target grasp position."""
    for _ in range(steps):
        joint_positions = p.calculateInverseKinematics(robot, ee_index, target_pos)
        for i in range(len(joint_positions)):
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])
        p.stepSimulation()
        time.sleep(1 / 50.0)

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
    """Check grasp contact"""
    return len(p.getContactPoints(bodyA=robot, bodyB=lid)) > 0

def maintain_grip(robot, lid):
    """Maintain grip on lid"""
    if verify_grasp(robot, lid):
        p.setJointMotorControl2(robot, 9, p.VELOCITY_CONTROL, 
                               targetVelocity=0, force=3000)
        p.setJointMotorControl2(robot, 10, p.VELOCITY_CONTROL, 
                               targetVelocity=0, force=3000)
        return True
    return False