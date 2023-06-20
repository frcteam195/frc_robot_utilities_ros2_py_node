from enum import Enum
from threading import Lock
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy
from ck_ros_base_msgs_node.msg import Robot_Status

class Alliance(Enum):
    RED = 0
    BLUE = 1
    UNKNOWN = 2

class RobotMode(Enum):
    DISABLED = 0
    TELEOP = 1
    AUTONOMOUS = 2
    TEST = 3

class RobotStatusHelperPy:
    def __init__(self, buffered_msg_obj : BufferedROSMsgHandlerPy):
        self.__bufmsgobj = buffered_msg_obj
        self.__robot_state : RobotMode = RobotMode.DISABLED
        self.__alliance : Alliance = Alliance.UNKNOWN
        self.__match_time = 0
        self.__game_data = ""
        self.__selected_auto = 0
        self.__is_connected = False
        self.__mutex = Lock()

    def __update(self):
        if(self.__bufmsgobj.has_updated()):
            self.__mutex.acquire()
            try:
                r_stat : Robot_Status = self.__bufmsgobj.get()
                self.__robot_state = RobotMode(r_stat.robot_state)
                self.__alliance = Alliance(r_stat.alliance)
                self.__match_time = r_stat.match_time
                self.__game_data = r_stat.game_data
                self.__selected_auto = r_stat.selected_auto
                self.__is_connected = r_stat.is_connected
            finally:
                self.__mutex.release()

    def get_message(self) -> dict:
        self.__update()
        message = {
            "robot_data": str(self.__robot_state),
            "alliance": str(self.__alliance),
            "match_time": self.__match_time,
            "game_data": self.__game_data,
            "selected_auto": self.__selected_auto,
            "is_connected": self.__is_connected
        }
        return message

    def get_mode(self) -> RobotMode:
        self.__update()
        return self.__robot_state

    def get_alliance(self) -> Alliance:
        self.__update()
        return self.__alliance

    def get_match_time(self) -> float:
        self.__update()
        return self.__match_time

    def get_game_data(self) -> str:
        self.__update()
        return self.__game_data

    def get_selected_auto(self) -> int:
        self.__update()
        return self.__selected_auto

    def is_connected(self) -> bool:
        self.__update()
        return self.__is_connected
