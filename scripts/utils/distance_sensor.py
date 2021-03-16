import rospy
from sensor_msgs.msg import Range

class DistanceSensor:
    def __init__(self, topic_name):
        self.range = None
        self.max_range = None
        self.min_range = None
        self.topic_name = topic_name

    def initialise(self):
        rospy.Subscriber(self.topic_name, Range, self._callback)
        rospy.loginfo(f"Node iniciado de distância do sensor {rospy.get_time()}")
 
    def _callback(self, data):
        self.range = data.range
        self.max_range = data.max_range
        self.min_range = data.min_range

    def get_range(self):
        return self.range
    
    def get_limits(self):
        return (self.max_range,self.min_range)
