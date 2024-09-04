import pybullet as p
import time
import pybullet_data

from robot_lqr import RobotLqr


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
# update loop on the real robot runs at 200Hz
p.setTimeStep(1./200.)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# define some debug parameters to control the robot
vel_param_id = p.addUserDebugParameter("velocity",-4.0, 4.0, 0.0)
yaw_param_id = p.addUserDebugParameter("yaw",-10.0, 10.0, 0.0)


planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0.01]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId: int = p.loadURDF(
    "robot-02.urdf",
    startPos,
    startOrientation,
    flags=p.URDF_USE_IMPLICIT_CYLINDER  # without this we get "chunky" wheels
)

robot = RobotLqr(robotId)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    input_vel = p.readUserDebugParameter(vel_param_id)
    input_yaw = p.readUserDebugParameter(yaw_param_id)
    robot.set_velocity_linear_set_point(input_vel)
    robot.set_yaw(input_yaw)
    robot.update_motor_speed()
    p.stepSimulation()
    time.sleep(1./200.)


cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()



