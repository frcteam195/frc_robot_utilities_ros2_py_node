from typing import TypeVar, Generic
from ck_utilities_ros2_py_node.node_handle import NodeHandle
from frc_robot_utilities_ros2_py_node.BufferedROSMsgHandlerPy import BufferedROSMsgHandlerPy

ControlClass = TypeVar('ControlClass')
StatusClass = TypeVar('StatusClass')

class SubsystemController(Generic[ControlClass, StatusClass]):

    def __init__(self, control_topic : str, control_data_class, status_topic : str, status_data_class):
        self.__node_handle = NodeHandle().node_handle
        self.__publisher = self.__node_handle.create_publisher(topic=control_topic, msg_type=control_data_class, qos_profile=10)
        self.__status_handler = BufferedROSMsgHandlerPy(status_data_class)
        self.__status_handler.register_for_updates(status_topic)

    def get(self) -> StatusClass:
        return self.__status_handler.get()

    def publish(self, msg : ControlClass):
        if isinstance(msg, self.__orig_class__.__args__[0]):
            self.__publisher.publish(msg)
        else:
            self.__node_handle.get_logger().error(f"Invalid type passed to publish: {str(msg.__class__)}. Type is supposed to be {self.__orig_class__.__args__[0]}")