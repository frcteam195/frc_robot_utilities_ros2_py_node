import rospy
from threading import Lock
from typing import TypeVar, Generic
from frc_robot_utilities_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy

ControlClass = TypeVar('ControlClass')
StatusClass = TypeVar('StatusClass')

class SubsystemController(Generic[ControlClass, StatusClass]):

    def __init__(self, control_topic : str, control_data_class, status_topic : str, status_data_class):
        self.__publisher = rospy.Publisher(name=control_topic, data_class=control_data_class, queue_size=50, tcp_nodelay=True)
        self.__status_handler = BufferedROSMsgHandlerPy(status_data_class)
        self.__status_handler.register_for_updates(status_topic)

    def get(self) -> StatusClass:
        return self.__status_handler.get()

    def publish(self, msg : ControlClass):
        if isinstance(msg, self.__orig_class__.__args__[0]):
            self.__publisher.publish(msg)
        else:
            rospy.logerr(f"Invalid type passed to publish: {str(msg.__class__)}. Type is supposed to be {self.__orig_class__.__args__[0]}")