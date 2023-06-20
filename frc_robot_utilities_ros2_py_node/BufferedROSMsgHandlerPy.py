import rclpy
import rclpy.node
import rclpy.qos
from threading import Lock

class BufferedROSMsgHandlerPy:
    def __init__(self, node : rclpy.node.Node, data_class):
        self.__update_occurred = False
        self.__mutex = Lock()
        self.__msg_update_tmp = None
        self.__msg_buf = None
        self.__subscriber = None
        self.__node = node
        self.__data_class = data_class

    def __update_func(self, msg):
        self.__mutex.acquire()
        try:
            self.__msg_update_tmp = msg
            self.__update_occurred = True
        finally:
            self.__mutex.release()

    
    def register_for_updates(self, topic_name):
        self.__subscriber = self.__node.create_subscription(msg_type=self.__data_class, topic=topic_name, callback=self.__update_func)
    
    def get(self):
        if(self.__update_occurred):
            self.__mutex.acquire()
            try:
                self.__msg_buf = self.__msg_update_tmp
                self.__update_occurred = False
            finally:
                self.__mutex.release()
        return self.__msg_buf
    
    def has_updated(self)->bool:
        return self.__update_occurred
    
