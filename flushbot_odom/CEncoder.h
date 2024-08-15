#ifndef Flushbot_Encoder_H_
#define Flushbot_Encoder_H_ // multiple definition possible

// #include <stdint.h>
#include <Arduino.h>

/* Parameters */
#define MOTOR_RATIO 49. // maybe or not (294.) 
#define RPMtoRAD 0.10471975512 // fixed : 2pi/60 
#define PULSEtoRAD 6.283185307/(13*49*4)
#define ENCODER_INTERVAL 33 // setting encoder motor interval
#define WHEEL_MAX_ANG_VEL 15.4 // robot spec
#define WHEEL_RADIUS 0.0555  
#define WHEEL_SEPARATION 0.402 // distance from wheel to another wheel
#define MAX_LIN_VEL WHEEL_MAX_ANG_VEL * WHEEL_RADIUS  // maximum linear velocity (x-axis) 
#define MAX_ANG_VEL WHEEL_RADIUS * WHEEL_MAX_ANG_VEL / (WHEEL_SEPARATION/2)  // maximum angular velocity (z-axis) : yaw direction
#define LIN_SPEED_LIMIT 0.6  // set limitation
#define ANG_SPEED_LIMIT 1.0
#define MIN_LINEAR_VELOCITY -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY -MAX_ANGULAR_VELOCITY

/* PID GAIN */
#define P_GAIN_L 1.2
#define I_GAIN_L 0.4
#define D_GAIN_L 0.9
#define P_GAIN_R 1.2
#define I_GAIN_R 0.4
#define D_GAIN_R 0.9
#define PID_INTERVAL 3  // PID control interval
// need to change

class CEncoder
{
  
  private:
    uint8_t pinA, pinB;  // 2ch pinA, pinB (setting 8 bits)
    uint8_t pinA_state, pinB_state;
    
    double pidControl;
    bool left_motor_dir = true;
    bool left_motor_PID = true;
    double P_GAIN_, I_GAIN_, D_GAIN_;

    uint32_t prevMillis; // memorize previous time 
    int32_t prev_pulse_count; 
    uint8_t _previousState; 
    double prev_dControl = 0; // setting intial value
    double accError = 0;
    uint32_t prev_pid_millis = 0;

    static void DecodeISR0();  // left or right 
    static void DecodeISR1();  // left or right

    bool prev_pinSW0_state, prev_pinSW1_state;
    uint16_t target_position = 0;
    byte cnt_ = 0; // 8bit, role of counter 

    bool prev_pinB_state;
  public:
    bool check = false;
    byte initEncoder = 0;
    double rpm, ang_vel, dir_term, angVel_linear, angVel_merge, angVel_rotate, errorGap;
    volatile int32_t pulse_count = 0;  // real time encoder pulse
    volatile int32_t pulse_count2 = 0;  // real time encoder pulse
    int32_t pulse = 0;  
    uint8_t pwm = 0;
    bool dir = 0;
    int16_t input_B = 0;
    double current, target;
    static CEncoder *instances[2];  

    /* Motor(wheel) functions */   // !! important content !!
    /*
      0. begin : attach instance with initial settings
      0. Decode : use Interrupt to generate pulses
      0. PrintPulse : for checking pulses  

    ` 1. CurrentVel : convert 'pulse' to 'angular velocity(rad/s)'
      2. CMDVELtoTarget : convert 'cmd_vel' to 'target angular velocity of each motor'
      3. EncoderPID : PID using 'current angular velocity' and 'target angular velocity'
      4. PIDtoPWM : convert 'PID controled angular velocity' to 'target PWM' 
    */

    void begin(uint8_t PinA_, uint8_t PinB_, int motor_num);  // set pins, dir
    void Decode();  // decode pulse count 
    void PrintPulse();  // check pulse
    
    void CurrentVel(float time_interval); 
    double CMDVELtoTarget(double lin_x_, double ang_z_);
    void EncoderPID();
    uint8_t PIDtoPWM();
};


#endif 
