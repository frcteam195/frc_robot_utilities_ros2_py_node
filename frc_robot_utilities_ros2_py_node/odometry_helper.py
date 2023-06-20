from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.ckmath import *
from nav_msgs.msg import Odometry
from frc_robot_utilities_py_node.RobotStatusHelperPy import Alliance
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from frc_robot_utilities_py_node.frc_robot_utilities_py import *

class OdometryHelper(object):
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(OdometryHelper, cls).__new__(cls)
            cls.__odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
            cls.__odometry_subscriber.register_for_updates("odometry/filtered")
        return cls.instance

    def get_facing_own_alliance(cls) -> bool:
        # Determine the alliance station the robot is facing.
        if cls.__odometry_subscriber.get() is not None:
            odometry_message : Odometry = cls.__odometry_subscriber.get()
            rotation = Rotation(odometry_message.pose.pose.orientation)
            yaw = rotation.yaw
            yaw = normalize_to_2_pi(yaw)
            cls.heading = np.degrees(yaw)

        target_alliance = Alliance.RED if 90 < cls.heading < 270 else Alliance.BLUE
        return target_alliance == robot_status.get_alliance()