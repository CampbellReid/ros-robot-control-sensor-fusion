import rospy
from math import atan2, sqrt, cos, sin, pi
from geometry_msgs.msg import Twist
from assignment3.srv import GetPaths
from geometry_msgs.msg import Quaternion, PoseArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelStateRequest
from tf.transformations import quaternion_from_euler, quaternion_multiply

def controller(namespace):
    #rospy.loginfo("1: Initializing path controller")

    # Path controller for executing steps
    rospy.wait_for_service('/paths_service')
    #rospy.loginfo("2: /paths_service is available")
    
    controller_rate = rospy.Rate(0.5)
    limit = 0.25
    speed = 0.085
    
    # Try get path and iterate through steps
    try:
        i = 1
        #rospy.loginfo("3: Running...")
        ps_request = rospy.ServiceProxy('/paths_service', GetPaths)
        #rospy.loginfo("4: ServiceProxy for /paths_service created")
        
        request = ps_request(namespace)
        #rospy.loginfo("5: Path request made")

        if request.is_empty:
            #rospy.loginfo("6: No paths available")
            return False
        
        # Ask gazebo for the current position
        rospy.wait_for_service('/gazebo/get_model_state')
        #rospy.loginfo("6.1: /gazebo/get_model_state is available")
        
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #rospy.loginfo("6.2: ServiceProxy for /gazebo/get_model_state created")
        
        state_request = GetModelStateRequest()
        state_request.model_name = namespace
        
        state_response = get_state(state_request)
        #rospy.loginfo("6.3: Model state received")
        
        # Using state_response.pose.position.x and state_response.pose.position.y, log the current position
        #rospy.loginfo(f"6.4: Current position: ({state_response.pose.position.x}, {state_response.pose.position.y})")
        
        # Log the steps
        #rospy.loginfo("6.5: Steps:")
        for step in request.paths.poses:
            #rospy.loginfo(f"6.6: Step {i}: ({step.position.x}, {step.position.y})")
            i += 1
        
        for step in request.paths.poses:
            #rospy.loginfo(f"7: Processing step {i}")
            
            smt = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            ms_msg = ModelState()
            ms_msg.model_name = namespace
            ms_msg.pose.position = step.position
            smt(ms_msg)
            #rospy.loginfo("12: Model state set")
            
            # # Get robot details
            # try:
            #     gms_request = GetModelStateRequest()
            #     gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            #     #rospy.loginfo("8: ServiceProxy for /gazebo/get_model_state created")
                
            #     gms_request.relative_entity_name = "world"
            #     gms_request.model_name = namespace
            #     gms_response = gms(gms_request)
            #     #rospy.loginfo("9: ModelState request made")
            # except rospy.ServiceException as error:
            #     #rospy.loginfo(f"ERROR: Service call GMS --> {error}")
                
            # ms_msg = ModelState()
            # ms_msg.model_name = namespace
            
            # # Set current position based on previous position (avoid reverting to 0, 0)
            # ms_msg.pose.position = gms_response.pose.position
            # #rospy.loginfo("10: Current position set")

            # # Calculate movement direction
            # if int(step.position.x) > int(ms_msg.pose.position.x) and int(step.position.y) == int(ms_msg.pose.position.y):
            #     ms_msg.pose.orientation.z = 0
            #     ms_msg.pose.orientation.w = 1
            # elif int(step.position.x) < int(ms_msg.pose.position.x) and int(step.position.y) == int(ms_msg.pose.position.y):
            #     ms_msg.pose.orientation.z = 1
            #     ms_msg.pose.orientation.w = 0
            # elif int(step.position.y) > int(ms_msg.pose.position.y) and int(step.position.x) == int(ms_msg.pose.position.x):
            #     ms_msg.pose.orientation.z = 1
            #     ms_msg.pose.orientation.w = 1
            # elif int(step.position.y) < int(ms_msg.pose.position.y) and int(step.position.x) == int(ms_msg.pose.position.x):
            #     ms_msg.pose.orientation.z = -1
            #     ms_msg.pose.orientation.w = 1
            # #rospy.loginfo("11: Movement direction calculated")

            # try:
            #     smt = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            #     smt_response = smt(ms_msg)
            #     #rospy.loginfo("12: Model state set")
            #     rospy.sleep(1)
            # except rospy.ServiceException as error:
            #     #rospy.loginfo(f"ERROR: Service call SMT --> {error}")
            
            # # Publish constant speed
            # vel_publisher = rospy.Publisher(f'/{namespace}/cmd_vel', Twist, queue_size = 10)
            # vel_msg = Twist()
            # vel_msg.linear.x = speed
            # #rospy.loginfo("13: Velocity publisher set")

            # # Keep constant speed until target location reached
            # while max(gms_response.pose.position.x, step.position.x) - min(gms_response.pose.position.x, step.position.x) > limit or \
            #       max(gms_response.pose.position.y, step.position.y) - min(gms_response.pose.position.y, step.position.y) > limit and \
            #       not rospy.is_shutdown():
            #     # Update current location
            #     gms_response = gms(gms_request)
            #     vel_publisher.publish(vel_msg)
            #     #rospy.loginfo("14: Speed published, sleeping...")
            #     controller_rate.sleep()

            # # Target location reached, wait for additional steps
            # vel_msg.linear.x = 0
            # vel_publisher.publish(vel_msg)
            # #rospy.loginfo(f"15: Step {i} reached: ({step.position.x}, {step.position.y})")
                    
            # Display progress
            print("STEP ", i, ": ", "(", step.position.x, ",", step.position.y, ")")
            i += 1
            controller_rate.sleep()
    except rospy.ServiceException as error:
        pass
        #rospy.loginfo(f"ERROR: Service failed --> {error}")
    
    return True


if __name__ == '__main__':
     try:
         rospy.init_node('path_controller', anonymous = True)
         #rospy.loginfo("16: Node initialized")
         controller()
     except rospy.ROSInterruptException:
         #rospy.loginfo("17: ROS Interrupt Exception")
         pass
