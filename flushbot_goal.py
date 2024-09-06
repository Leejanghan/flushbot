###  단독 구동이 가능함  ### 
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

class MoveRobot:
    def __init__(self):
    	# Initialize the ROS node 
        rospy.init_node('movebase_client_py')
        
        # set global parameter
        global current_position
        current_position = "" 
        
        # Initialize publishers and subscribers 
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.flushbot_pub = rospy.Publisher('/flushbot_sub', String, queue_size=10)
        self.flushbot_sub = rospy.Subscriber('/flushbot_pub', Float32, self.flushbot_status_callback)
        
        # Initialize the action client for move_base  
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # Initialize lidar subscriber in a seperate thread
        self.lidar_distance = None 
        self.lidar_threshold = 10.0 # Consider stop delay
        threading.Thread(target=self.lidar_sub).start()
        
        # Internal state variables 
        self.flushbot_status = 0.0
        self.drive_status = False 
        self.drive_status = False 
        

	# 서비스는 동기적인 원격 프로시저 호출 : 한 노드가 다른 노드에서 실행되는 함수 호출을 가능하게 함. 
    def clear_costmaps(self):
    	rospy.wait_for_service('/move_base/clear_costmaps') # 서버에서 서비스가 광고되기를 기다림
    	try:
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty) # 지역 프록시를 설정 
            clear_costmaps()
    	except rospy.ServiceException as e: # 서비스가 광고되기 전에 사용하려고 하면 다음과 같은 에러 발생 
    		rospy.logwarn("Service call failed: %s" & e)
    	
    # 액션은 비동기적, 좌표 설정이 필요함 // 좌표값을 먼저 따와야함 
    def movebase_client(self,x):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
    
        if x==1:
            goal.target_pose.pose.position.x = 3.656
            goal.target_pose.pose.position.y = 0.2732
            goal.target_pose.pose.orientation.z = 0.0231
            goal.target_pose.pose.orientation.w = 0.999
        else: 
            goal.target_pose.pose.position.x = 0.
            goal.target_pose.pose.position.y = 0.
            goal.target_pose.pose.orientation.z = 0.0112
            goal.target_pose.pose.orientation.w = 0.999

        # Sends the goal to the action server	
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action
        wait = self.client.wait_for_result()
  
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            self.drive_status = False 
        else:
            # Result of executing the action
            self.drive_status = True
            return self.client.get_result()   

    def lidar_callback(self,msg):
        self.lidar_distance = msg.data
    
    def lidar_sub(self):
        rospy.Subscriber("lidar_distance", Float32, self.lidar_callback)
        rospy.spin()
    
    def adjust_position(self):
        rate = rospy.Rate(10)
        lidar_cnt = 0 
    
        while self.lidar_distance is None and not rospy.is_shutdown():
            rospy.loginfo("waiting for LiDAR data .. ")
            rospy.sleep(3)
            lidar_cnt += 1
            if lidar_cnt >= 5 :
                break
    
        while not rospy.is_shutdown():
            if self.lidar_distance >= self.lidar_threshold:
                vel_msg=Twist()
                vel_msg.linear.x= -0.1
                self.cmd_vel_pub.publish(vel_msg)
                rospy.loginfo("LiDAR_distance: %f", self.lidar_distance)
            else:
                rospy.loginfo("LiDAR_distance: %f", self.lidar_distance)
                rospy.loginfo("LiDAR distance above threshold")
                vel_msg=Twist()
                vel_msg.linear.x=0.0
                self.cmd_vel_pub.publish(vel_msg)
                break

    def backward_motion(self, duration):
        # Create Twist message for backwards motion
        vel_msg = Twist()
        vel_msg.linear.x = 0.1  # Move backwards (화장실 기준 뒤)
        self.cmd_vel_pub.publish(vel_msg)
        rospy.loginfo("Starting go back")   
        
        # Sleep for the given duration
        rospy.sleep(duration)

        # Stop the robot
        vel_msg.linear.x = 0
        self.cmd_vel_pub.publish(vel_msg)
        rospy.loginfo("Stopped moving")   

    def stop_motion(self, duration):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0 # Move stop
        self.cmd_vel_pub.publish(vel_msg)
        rospy.loginfo("stopped moving")
        rospy.sleep(duration)
 
    def flushbot_status_callback(self,msg):
        self.flushbot_status = msg.data
        rospy.loginfo("Received flushbot_status : %s", self.flushbot_status)
    
    def Start_robot(self):
        while not rospy.is_shutdown():
            rospy.loginfo("waiting for flushbot to be active ... ")
            while not self.flushbot_status and not rospy.is_shutdown():
                rospy.sleep(3)
            
            # Execute actions based on flushbot_status
            if self.flushbot_status == 1.0:
                result = self.movebase_client(1)
                if result:
                    rospy.loginfo("Arrived at first goal")
                    self.adjust_position()
                    self.stop_motion(5)
                    rospy.sleep(5)
                    self.flushbot_status = 0.0
                    self.current_position = "point_3"
                    self.flushbot_pub.publish(self.current_position)
            
            elif self.flushbot_status == 2.0:
                self.backward_motion(10)
                self.clear_costmaps()
                rospy.sleep(5)
                result = self.movebase_client(2)
                if result:
                    rospy.loginfo("Return to origin")
                    self.clear_costmaps()
                    self.flushbot_status = 0.0
                    rospy.sleep(5)
        
            
if __name__ == '__main__':
    flushbot = MoveRobot()
    flushbot.Start_robot()
