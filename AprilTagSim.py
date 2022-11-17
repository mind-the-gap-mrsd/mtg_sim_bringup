
import actionlib
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from robosar_messages.msg import *
from robosar_messages.srv import *
import sched, time
s = sched.scheduler(time.time, time.sleep)
class AprilTagSim:

      def __init__(self) -> None:
        self.mapData = OccupancyGrid()
        ns = rospy.get_name()
        self.april_tag_pub = rospy.Publisher(
            ns + "/apriltag_marker", Marker, queue_size=10
        )
        self.gui_publisher = rospy.Publisher(
            "/", PointArray, queue_size=10
        )
        self.agent_poses_sub = rospy.Subscriber(
            "/tf", PoseArray, self.agent_poses_callback
        )
        robot_poses=[]
        
        listener.waitForTransform('map', list(agent_active_status.keys())[0] + '/base_link', rospy.Time(), rospy.Duration(1.0))
        for name in agent_active_status:
            now = rospy.Time.now()
            listener.waitForTransform('map', name + '/base_link', now, rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('map', name + '/base_link', now)
            robot_poses.append([trans[0], trans[1]])
        rospy.loginfo("Frontier filter is ready.")
        rospy.spin()
# Generate random x,y poses in map
    def generate_random_poses(num_poses, map):
        poses = []
        for i in range(num_poses):
            x = random.uniform(origin.x, map_width)
            y = random.uniform(origin.y, map_height)
            poses.append([x, y])
        return poses


    # Subscribe to agent poses and check if they are in vicinity of apriltag poses
    def check_agent_poses():
        
        for agent_pose in agent_poses:
            for apriltag_pose in apriltag_poses:
                if (agent_pose[0] - apriltag_pose[0])**2 + (agent_pose[1] - apriltag_pose[1])**2 < threshold**2:
                    return True
        return False

    # If agent poses are in vicinity of apriltag poses, publish to gui
    def publish_agent_poses(agent_poses, apriltag_poses, threshold):
        if check_agent_poses(agent_poses, apriltag_poses, threshold):
            pub.publish(agent_poses)

#Create new node which subscribes to map, agent poses and publishes to gui
def main():
    global pub
    rospy.init_node('AprilTagSim', anonymous=True)
    pub = rospy.Publisher('agent_poses', PoseArray, queue_size=10)
    global listener = tf.TransformListener()
    
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.Subscriber("agent_poses", PoseArray, agent_poses_callback)
    
    #execute check agent poses once every second 
    rospy.Timer(rospy.Duration(1), check_agent_poses)
    rospy.spin()