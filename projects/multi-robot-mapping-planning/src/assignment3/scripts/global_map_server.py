import rospy
from discretise import process_points, generate_path
from std_msgs.msg import Bool
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose, PoseArray
import random
from assignment3.srv import GetPaths, GetPathsResponse

# Global variables
global_coords = []
graph = None
min_coords = None
count = 0

def init():
    global sub, s, pub
    #rospy.loginfo("1: Initializing node and setting up subscribers, publishers, and services")
    sub = rospy.Subscriber('/processing_ready', Bool, callback)
    #rospy.loginfo("2: Subscriber to /processing_ready set up")
    s = rospy.Service('paths_service', GetPaths, handle_get_paths)
    #rospy.loginfo("3: Service paths_service set up")
    pub = rospy.Publisher('/processing_done', Bool, queue_size=10)
    #rospy.loginfo("4: Publisher to /processing_done set up")

def handle_get_paths(req):
    global global_coords, graph, min_coords
    #rospy.loginfo("5: Handling get paths request")

    # Check if global_coords is empty
    if len(global_coords) == 0:
        #rospy.loginfo("6: Global coordinates are empty")
        response = GetPathsResponse()
        response.is_empty = True
        return response
    
    namespace = req.namespace
    #rospy.loginfo(f"7: Namespace received: {namespace}")
    
    rospy.wait_for_service('/gazebo/get_model_state')
    #rospy.loginfo("8: Waiting for /gazebo/get_model_state service")
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #rospy.loginfo("9: ServiceProxy for /gazebo/get_model_state created")
        model_state = get_model_state(namespace, 'world')
        #rospy.loginfo("10: Model state received")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        response = GetPathsResponse()
        response.is_empty = True
        return response
    
    start = (model_state.pose.position.x, model_state.pose.position.y)
    #rospy.loginfo(f"11: Start position: {start}")
    end_index = random.randint(0, len(global_coords) - 1)
    end = global_coords[end_index]
    #rospy.loginfo(f"12: End position: {end}")
    
    # Remove this goal from global_coords
    global_coords.pop(end_index)
    #rospy.loginfo("13: Goal removed from global coordinates")
    
    path = generate_path(start, end, graph, min_coords)
    #rospy.loginfo("14: Path generated")
    
    # Print the path
    rospy.loginfo(f"Path: Start: {start} Goal: {end}")
    for coord in path:
        rospy.loginfo(f"    {coord}")
    
    # Create a PoseArray
    pose_array = PoseArray()
    
    # Convert each (x, y) in the path to a Pose and append to pose_array.poses
    for coord in path:
        pose = Pose()
        pose.position.x = coord[0]
        pose.position.y = coord[1]
        pose.position.z = 0  # Assuming 2D path, z can be 0
        pose.orientation.w = 1  # Default orientation
        pose_array.poses.append(pose)
        #rospy.loginfo(f"15: Added pose: {pose.position.x}, {pose.position.y}")

    response = GetPathsResponse()
    response.paths = pose_array
    response.is_empty = False
    #rospy.loginfo("16: Paths response created")
    
    return response

def callback(msg):
    global count
    #rospy.loginfo("17: Callback triggered")
    if msg.data:
        count += 1
        #rospy.loginfo(f"18: Received true message {count} times")
    if count == 2:
        #rospy.loginfo("19: Count is 2, executing code")
        execute_code()

def execute_code():
    global global_coords, graph, min_coords
    #rospy.loginfo("20: Executing code")
    global_coords, graph, min_coords = process_points()
    global_coords = global_coords.tolist()
    #rospy.loginfo("21: Processed points and updated global coordinates, graph, and min_coords")
    pub.publish(Bool(True))
    #rospy.loginfo("22: Published processing done message")
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('path_finder', anonymous=True)
    #rospy.loginfo("23: Node initialized")
    init()
    rospy.spin()
