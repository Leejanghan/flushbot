#include "CEncoder.h"
#include "flushbot.h"
#include "LIDAR_LITE.h"
/*******************************************************************************
* DC Motor instances
*******************************************************************************/
CEncoder* CEncoder::instances[2] = {NULL, NULL};  // Left motor, Right motor
CEncoder CMotor_L;
CEncoder CMotor_R;
/*******************************************************************************
* LiDAR instances
*******************************************************************************/
LIDAR_Lite myLidarLite;
/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{ 
  // Setting baudrate 
  Serial.begin(230400); 
   
  // setting LiDAR
  myLidarLite.begin(0, true); 
  myLidarLite.configure(0); // balanced performance mode 

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(230400);
  nh.subscribe(cmd_vel_sub); // teleop를 위한 구독 
  nh.advertise(imu_pub); // imu 퍼블리시
  nh.advertise(mag_pub); // mag 퍼블리시 
  nh.advertise(odom_pub); // odom 퍼블리시 
  nh.advertise(joint_states_pub); // joint_state 퍼블리시 
  nh.advertise(theta_pub); // checking theta
  nh.advertise(lidar_distance_pub);

  tf_broadcaster.init(nh); // broadcaster initialize

  // DC Motor pin setting 
  setPinMode();
  CMotor_L.begin(ENC_L_2, ENC_L_4, 1);   // int pin1, int pin2, motor_num
  CMotor_R.begin(ENC_R_7, ENC_R_8, 0);   // left : 1 , right : 0  

  // Setting for init
  sensors.init();

  // Setting gyro
  calibrationGyro(); // Do calibration 

  // Setting for odom 
  initOdom(); // init_encoder -> true 
  
  // Setting for joint_state
  initJointStates(); 
}
/*******************************************************************************
* Loop function
*******************************************************************************/
// need setting interval time 
void loop() 
{
  uint32_t t = millis();
  updateTime();

  // Odom_interval , motor interval --> 동일 주기 설정  
  // 주기 설정이 매우 중요함
  if (t - tTime[0] >= CONTROL_MOTOR_SPEED_FREQUENCY)
  { 
    prev_time = millis();
    updateGoalVelocity();
  
    // motor 구동 
    CMotor_L.CurrentVel(millis() - prev_time);
    CMotor_R.CurrentVel(millis() - prev_time);
    teleopMotor();
    check_data();
    tTime[0] = t; 
  }
  // IMU 
  if (t - tTime[1] >= IMU_PUBLISH_FREQUENCY)
  {
    publishImuMsg();
    publishMagMsg();
    tTime[1] = t;    
  }
  // ODOM
  if (t - tTime[2] >= DRIVE_INFORMATION_PUBLISH_FREQUENCY)
  {
    publishDriveInformation();
    tTime[2] = t;
  } 
  lidar_check();
  
  update_imu();
  nh.spinOnce();
  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}
/*******************************************************************************
* Motor for teleop
*******************************************************************************/
void teleopMotor()
{
  CMotor_L.CMDVELtoTarget(goal_velocity[LINEAR], goal_velocity[ANGULAR]);
  CMotor_R.CMDVELtoTarget(goal_velocity[LINEAR], goal_velocity[ANGULAR]);

  CMotor_L.EncoderPID();
  CMotor_R.EncoderPID();

  CMotor_L.PIDtoPWM();
  CMotor_R.PIDtoPWM();

  digitalWrite(motorDirL, CMotor_L.dir); 
  digitalWrite(motorDirR, CMotor_R.dir);
  analogWrite(motorPwmL, CMotor_L.pwm); // 모터 차이 
  analogWrite(motorPwmR, CMotor_R.pwm);
}
/* sequence : subscribe teleop --> PID contol --> PWM  
모터 차이가 어느 정도 존재 -->  양쪽 보정계수를 다르게 하는 방향 */
/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
}
/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{ 
  char log_msg[50];
  (void)(reset_msg);
  initOdom();
  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}
/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{ 
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}
/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}
/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
// Update function 
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);
    
  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);
}
/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_footprint";
  
  // 본래 calcodometry에 위치 
  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
  
  // 실물로봇 값 바탕으로 odom 실행 
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}
/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_position[LEFT];
  joint_states_pos[RIGHT] = last_position[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}
/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped & odom_tf)
{
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}
/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta, theta;
  static double last_theta = 0.0;
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = CMotor_L.pulse * TICK2RAD;
  wheel_r = CMotor_R.pulse * TICK2RAD;
  //wheel_l = CMotor_L.ang_vel * step_time;
  //wheel_r = CMotor_R.ang_vel * step_time; 

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  last_position[LEFT] += wheel_l;
  last_position[RIGHT] += wheel_r;

  // I think wheel_l is same as ang_vel
  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  
  // About theoretical value 
  // delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta; 
  last_theta = theta;

  // compute velocity  
  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = delta_s / step_time;
  odom_vel[1] = 0.0;
  odom_vel[2] = delta_theta / step_time;

  return true;
}
/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
// if isConnected is true, begin initialization
// but, we initialize at setup
/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
// wait a momoment 
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}
/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}
/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}
/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;
  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}
/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = "base_link";
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}
/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
// same role as cmd_vel cb & receive data from teleop
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}
/*******************************************************************************
* calibrationGyro
*******************************************************************************/
void calibrationGyro() 
{
  uint32_t pre_time;
  uint32_t t_time;
  const uint8_t led_ros_connect = 3;

  imu.SEN.gyro_cali_start();
  t_time   = millis();
  pre_time = millis();

  while(!imu.SEN.gyro_cali_get_done())
  {
    imu.update();
    if (millis()-pre_time > 5000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      setLedToggle(led_ros_connect);
    }
  }
}
/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}
/*******************************************************************************
* LiDAR check code 
*******************************************************************************/
void lidar_check()
{
  float lidar_distance = myLidarLite.distance();
  lidar_distance_filtered = FILTER_ALPHA * lidar_distance + (1 - FILTER_ALPHA) * lidar_distance_prev;
  if (lidar_distance_filtered >= 100.0) 
  {
    lidar_distance_filtered = 100.0; // 너무 큰 값일 때 업로딩이 길어지는 현상 발견 
  }
  else
  {
    lidar_distance_prev = lidar_distance_filtered;
  }

  // Prepare and publish message
  lidar_distance_msg.data = lidar_distance_filtered;
  lidar_distance_pub.publish(&lidar_distance_msg); 
}
/*******************************************************************************
* check any data  
*******************************************************************************/
void check_data()
{
  theta_msg.data = CMotor_L.realError;
  theta_pub.publish(&theta_msg);  
}
