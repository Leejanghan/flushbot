#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Brings in the service 
from std_srvs.srv import Empty

# Brings psd_voltage
from std_msgs.msg import Float32, Bool, String
import threading
import time
from geometry_msgs.msg import Twist

lidar_distance = None
lidar_threshold = 10.0 # consider about stop delay
cmd_vel_pub = 0.
drive_status = False 
flushbot_status = False 
current_position = ""

# 서비스는 동기적인 원격 프로시저 호출 : 한 노드가 다른 노드에서 실행되는 함수 호출을 가능하게 함. 
def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps') # 서버에서 서비스가 광고되기를 기다림
    try:
        clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty) # 지역 프록시를 설정 
        clear_costmaps()
    except rospy.ServiceException as e: # 서비스가 광고되기 전에 사용하려고 하면 다음과 같은 에러 발생 
    	rospy.logwarn("Service call failed: %s" & e)
    	
# 액션은 비동기적, 좌표 설정이 필요함 // 좌표값을 먼저 따와야함 
def movebase_client(x):
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    if x==1:
        goal.target_pose.pose.position.x = -0.75751
        goal.target_pose.pose.position.y = 7.0499
        goal.target_pose.pose.orientation.z = 0.999708
        goal.target_pose.pose.orientation.w = 0.024174
        
        global current_position
        current_position = 'point_3'
        flushbot_pub.publish(current_position)
        
    else: 
        goal.target_pose.pose.position.x = 0.
        goal.target_pose.pose.position.y = 0.
        goal.target_pose.pose.orientation.z = 0.0112
        goal.target_pose.pose.orientation.w = 0.999

   # Sends the goal to the action server	
    client.send_goal(goal)
   # Waits for the server to finish performing the action
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        global drive_status
        drive_status = False 
        return client.get_result()   

def callback(msg):
    global lidar_distance
    lidar_distance = msg.data
    
def lidar_sub():
    rospy.Subscriber("lidar_distance", Float32, callback)
    rospy.spin()
    
def adjust_position():
    global lidar_distance, cmd_vel_pub
    rate = rospy.Rate(10)
    lidar_cnt = 0 
    
    while lidar_distance is None and not rospy.is_shutdown():
    	rospy.loginfo("waiting for LiDAR data .. ")
    	rospy.sleep(3)
    	lidar_cnt += 1
    	if lidar_cnt >= 5 :
    	    break
    
    while not rospy.is_shutdown():
        if lidar_distance >= lidar_threshold:
            vel_msg=Twist()
            vel_msg.linear.x= -0.1
            cmd_vel_pub.publish(vel_msg)
            rospy.loginfo("LiDAR_distance: %f", lidar_distance)
        else:
            rospy.loginfo("LiDAR_distance: %f", lidar_distance)
            rospy.loginfo("LiDAR distance above threshold")
            vel_msg=Twist()
            vel_msg.linear.x=0.0
            cmd_vel_pub.publish(vel_msg)
            break

def backward_motion(duration):
    global cmd_vel_pub
    # Create Twist message for backwards motion
    vel_msg = Twist()
    vel_msg.linear.x = 0.1  # Move backwards (화장실 기준 뒤)
    cmd_vel_pub.publish(vel_msg)
    rospy.loginfo("Starting go back")   
    
    # Sleep for the given duration
    rospy.sleep(duration)

    # Stop the robot
    vel_msg.linear.x = 0
    cmd_vel_pub.publish(vel_msg)
    rospy.loginfo("Stopped moving")   

def stop_motion(duration):
    global cmd_vel_pub
    vel_msg = Twist()
    vel_msg.linear.x = 0.0 # Move stop
    cmd_vel_pub.publish(vel_msg)
    rospy.loginfo("stopped moving")
    rospy.sleep(duration)
 
def flushbot_status_callback(msg):
    global flushbot_status
    flushbot_status = msg.data
    rospy.loginfo("Received flushbot_status : %s", flushbot_status)
    
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        lidar_thread = threading.Thread(target=lidar_sub)
        lidar_thread.start()
       
       # Publisher for cmd_vel 
        cmd_vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        flushbot_pub=rospy.Publisher('/flushbot_sub', String, queue_size=10)
        flushbot_sub=rospy.Subscriber('/flushbot_pub', Bool, flushbot_status_callback)

       # Move to the first goal  
        if flushbot_status == True : 
            result = movebase_client(1)
        else:
            result = False 
	        
        # Navigation success 
        if drive_status == True :             
            rospy.loginfo("Arrived at first Goal")
            adjust_position()
            stop_motion(5)
            rospy.sleep(5) 
            backward_motion(10)
            clear_costmaps()
            rospy.sleep(5)
       	
            # Move to the second goal  
            result = movebase_client(2)
            rospy.loginfo("Return to origin")
            clear_costmaps()
            rospy.sleep(5)

        if result:
            rospy.loginfo("Goal execution done!"+str(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
