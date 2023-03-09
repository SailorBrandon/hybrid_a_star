import rospy
from nav_msgs.msg import OccupancyGrid

class TryMapNode:
    def __init__(self) -> None:
        map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
    
    def map_callback(self, msg):
        print(type(msg.data))
        print(len(msg.data))
        print(msg.data)

if __name__ == '__main__':
    rospy.init_node('try_map')
    node = TryMapNode()
    rospy.spin()