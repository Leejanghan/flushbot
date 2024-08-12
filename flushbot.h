#ifndef FLUSHBOT_H_
#define FLUSHBOT_H_

/* include file */
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <turtlebot3_msgs/SensorState.h>
#include "Arduino.h"
#include "turtlebot3_sensor.h"

/* setting frequency */
// 주기를 다르게 할꺼면 다음과 같은 frequency를 이용 
#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz

/* setting wheel */
#define WHEEL_NUM                        2
#define LEFT                             1
#define RIGHT                            0

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI
#define TICK2RAD                         0.0024659283   
// 2548 ticks, So 0.1412872841 [deg] * pi / 180 = 0.0024659283

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishImuMsg(void);
void publishDriveInformation(void);
void updateOdometry(void);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped & odom_tf);
void updateGoalVelocity(void);
void initOdom(void);
void initJointStates(void);
bool calcOdometry(double diff_time);
void waitForSerialLink(bool isConnected);
// 여기서부턴 아래에 정의되어있음. 
void init_imu();
void setPinMode();
void update_imu();  // 처음 init_imu 부분에서 imu 한 번 업데이트 후 시작  
void teleopMotor();
void get_IMU();
void calc_theta();
void check_data();
// void lidar_check();

ros::Time rosNow(void);

/* Parameters */
#define ODOM_INTERVAL 25 
/* ms단위. controller 주기인 20hz(50ms)보단 빠르게, odom주기인 100hz(20ms)보단 느리게. 
이 값이 너무 커지면 로봇이 천천히 회전할때 delta_theta값이 0이 되어버림. */
#define PI 3.141592

/* pinMode */
#define LED_PIN 13
#define motorDirL 11
#define motorDirR 12
#define motorPwmL 9
#define motorPwmR 6

#define ENC_L_2 2
#define ENC_L_4 4
#define ENC_R_7 7
#define ENC_R_8 8

/* setting psd */
/* float lidar_distance;
float FILTER_ALPHA = 0.85;
float lidar_distance_filtered; 
float lidar_distance_prev;
*/
/* setting type */
int heading_dir;
float linx;  // cmd_vel linear.x
float angz;  // cmd_vel angular.z 
float heading_angle;
float prev_time;
float imu_angle; // 헷갈리지 않게 각도 추가 
float ori_x, ori_y, ori_z, ori_w;

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

// IMU of flushbot
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Magnetic field
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);

// Joint(Encoder_Motor) state of flushbot
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// sensor state of flushbot
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

//checking data
std_msgs::Float32 theta_msg;
ros::Publisher theta_pub("theta", &theta_msg);

// psd data
/* std_msgs::Float32 lidar_distance_msg;
ros::Publisher lidar_distance_pub("lidar_distance", &lidar_distance_msg); 
*/
cIMU imu;

/*******************************************************************************
* Declaration for sensor
*******************************************************************************/
static Turtlebot3Sensor sensors;
/*******************************************************************************
* SetPinMode
*******************************************************************************/
void setPinMode(){
  pinMode(motorDirL, OUTPUT);
  pinMode(motorDirR, OUTPUT);
  pinMode(motorPwmL, OUTPUT);
  pinMode(motorPwmR, OUTPUT);
} 
/*******************************************************************************
* update imu
*******************************************************************************/ 
void update_imu()
{
  sensors.updateIMU();
}
/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char odom_header_frame_id[30];
char odom_child_frame_id[30];
char imu_frame_id[30];
char mag_frame_id[30];
char joint_state_header_frame_id[30];
/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);
/*******************************************************************************
* Publisher
*******************************************************************************/
// Odometry of flushbot
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of flushbot
geometry_msgs::TransformStamped odom_tf;
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tf_broadcaster;
/*******************************************************************************
* SoftwareTimer of flushbot
*******************************************************************************/
static uint32_t tTime[10];
/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};
/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};
double  last_position[WHEEL_NUM]  = {0.0, 0.0};
double  v[WHEEL_NUM] = {0.0, 0.0};
double  w[WHEEL_NUM] = {0.0, 0.0};
// v = translational velocity [m/s], w = rotational velocity [rad/s]
/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

#endif  //FLUSHBOT_H_