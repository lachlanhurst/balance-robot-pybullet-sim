import pybullet as p

# obtained from running `calculate_lqr_gains.py`
LQR_K = [-2.1402165848237837, -0.03501370844016172, 5.9748026764525894e-18, 2.236067977499789]
WHEEL_RADIUS = 0.034
MAX_MOTOR_VEL = 500.0 # rad/s


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


"""
Basic LQR controller implementation
"""
class RobotLqr:

    def __init__(self, id) -> None:
        self.id = id
        self.velocity_angular = 0.0
        self.velocity_linear_set_point = 0.0
        self.yaw = 0

        self.last_pitch = 0.0
        self.pitch_dot_filtered = 0.0
        self.velocity_angular_filtered = 0.0

    def set_velocity_linear_set_point(self, vel: float) -> None:
        """
        Sets the target velocity of the robot
        """
        self.velocity_linear_set_point = vel

    def set_yaw(self, yaw: float) -> None:
        """
        The yaw value is added to one wheel, and subtracted from the other
        to produce a yaw motion (turn)
        """
        self.yaw = yaw

    def get_pitch(self) -> float:
        _, cubeOrn = p.getBasePositionAndOrientation(self.id)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        return cubeEuler[0]

    def get_pitch_dot(self) -> float:
        _, angular = p.getBaseVelocity(self.id)
        return angular[0]

    def get_wheel_velocity(self) -> float:
        _, vel_m_0, _, _ =  p.getJointState(
            bodyUniqueId=self.id, 
            jointIndex=0
        )
        _, vel_m_1, _, _ = p.getJointState(
            bodyUniqueId=self.id, 
            jointIndex=1, 
        )
        # both wheels spin "forward", but one is spinning in a negative
        # direction as it's rotated 180deg from the other
        return (vel_m_0 * -1 + vel_m_1) / 2.0

    def calculate_lqr_velocity(self) -> float:
        pitch = -self.get_pitch()
        pitch_dot = self.get_pitch_dot()

        # apply a filter to pitch dot, and velocity
        # without these filters the controller seems to lack necessary dampening
        # would like to know why!
        self.pitch_dot_filtered = (self.pitch_dot_filtered * .975) + (pitch_dot * .025)
        self.velocity_angular_filtered = (self.velocity_angular_filtered * .975) + (self.get_wheel_velocity() * .025)

        velocity_linear_error = self.velocity_linear_set_point - self.velocity_angular_filtered * WHEEL_RADIUS

        lqr_v = LQR_K[0] * (0 - pitch) + LQR_K[1] * self.pitch_dot_filtered + LQR_K[2] * 0 + LQR_K[3] * velocity_linear_error
        return -lqr_v / WHEEL_RADIUS
    
    def update_motor_speed(self) -> None:
        vel = self.calculate_lqr_velocity()
        vel = clamp(vel, -MAX_MOTOR_VEL, MAX_MOTOR_VEL)

        p.setJointMotorControl2(
            bodyUniqueId=self.id, 
            jointIndex=0, 
            controlMode=p.VELOCITY_CONTROL, 
            targetVelocity=-vel + self.yaw
        )
        p.setJointMotorControl2(
            bodyUniqueId=self.id, 
            jointIndex=1, 
            controlMode=p.VELOCITY_CONTROL, 
            targetVelocity=vel + self.yaw
        )
