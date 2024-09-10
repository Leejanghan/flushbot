#include "CEncoder.h"
#include "Arduino.h"

/* 
  External Interrupt in openCR
  EXTI    Arduino
  0    ->    2
  1    ->    3
  2    ->    4    
  3    ->    7
  4    ->    8

  5    ->    42  // if you want to use this pin, you should not use int pin 0,2
  6    ->    45  // Do not work
  7    ->    72
  8    ->    75+
  
/* ISR Definition */
void CEncoder::DecodeISR0 ()
{
  if (CEncoder::instances[0] != NULL)
    CEncoder::instances[0]->Decode();
} 

void CEncoder::DecodeISR1 ()
{
  if (CEncoder::instances[1] != NULL)
    CEncoder::instances[1]->Decode();
}  // how to make a decode event

/* Motor */
void CEncoder::begin(uint8_t PinA_, uint8_t PinB_, int motor_num)
{  
  pinA = PinA_;
  pinB = PinB_;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  switch (pinA)
  {
  case 2: 
    attachInterrupt (0, DecodeISR0, CHANGE);
    attachInterrupt (2, DecodeISR0, CHANGE);
    instances [0] = this;
    break;

  case 4:
    attachInterrupt (0, DecodeISR0, CHANGE);
    attachInterrupt (2, DecodeISR0, CHANGE);
    instances [0] = this;
    break;
  
  case 7: 
    attachInterrupt (3, DecodeISR1, CHANGE);
    attachInterrupt (4, DecodeISR1, CHANGE);
    instances [1] = this;
    break;

  case 8: 
    attachInterrupt (3, DecodeISR1, CHANGE);
    attachInterrupt (4, DecodeISR1, CHANGE);
    instances [1] = this;
    break;
  }

  if (motor_num == 1) // left (want to change right)
  {
    left_motor_dir = true;
    left_motor_PID = true;
    P_GAIN_ = P_GAIN_L;
    I_GAIN_ = I_GAIN_L;
    D_GAIN_ = D_GAIN_L;
  }
  else if (motor_num == 0) // right (want to change left)
  {
    left_motor_dir = false;  // setting miss
    left_motor_PID = false;
    P_GAIN_ = P_GAIN_R;
    I_GAIN_ = I_GAIN_R;
    D_GAIN_ = D_GAIN_R;
  }
} // setting encoder motor

void CEncoder::Decode()
{  
  uint8_t pinA_state, pinB_state;

  pinA_state= digitalRead(pinA);
  pinB_state= digitalRead(pinB);
  uint8_t newState=(pinA_state<<1|pinB_state);
  switch((_previousState <<2)|newState){
    case 0b0001: // 0x01 PREV|CURR
    case 0b0111: // 0x07
    case 0b1110: // 0x0E
    case 0b1000: // 0x08
            {pulse_count--; break;}
    case 0b0010: // 0x02
    case 0b1011: // 0x0B
    case 0b1101: // 0x0D
    case 0b0100: // 0x04
            {pulse_count++; break;}
  default:
      break;
  }
  _previousState = newState;
} // how to get pulse 

void CEncoder::PrintPulse()
{
  Serial.print(pulse_count);
  Serial.print('\t');
  Serial.println(dir);
}

void CEncoder::CurrentVel(float time_interval)
{ 
  rpm = 60./((double)ENCODER_INTERVAL/1000.) * (double)pulse_count / (13. * 4.) / MOTOR_RATIO;  // calculate RPM or (13. * 4.)
  ang_vel = rpm * RPMtoRAD;  // calculate rad/s

  current = abs(ang_vel);  // current rad/s
  
  pulse = pulse_count;
  pulse_count = 0;  // reset pulse count
} 

// 시계 방향 : ang_z minus, 반시계 방향 : ang_z plus 
double CEncoder::CMDVELtoTarget(double lin_x_, double ang_z_) 
{
// lin_x_, ang_z_ --> m/s unit
  angVel_rotate = constrain((ang_z_*WHEEL_SEPARATION/2),-ANG_SPEED_LIMIT,ANG_SPEED_LIMIT);    
  dir_term=left_motor_dir==1?-1.:1.; // dir_term --> -1 (left motor 일때) 
  angVel_linear = constrain(lin_x_, -LIN_SPEED_LIMIT, LIN_SPEED_LIMIT);  
  angVel_merge = (angVel_rotate*dir_term + angVel_linear)/WHEEL_RADIUS;  // angVel_merge = angular velocity rotate term + angular velocity linear term
  // unit rad/s 
  dir=angVel_merge>0?1:0; 
  target = abs(angVel_merge);
} // teleop 값에 의한 각속도를 의미   

// PID 제어에 대한 이해
/* target : 목표로 하는 모터의 RPM , current : 현재 모터의 RPM,
   realError : 목표 RPM과 현재 RPM 사이의 차이, accError : realError의 누적 
   errorGap : 현재 Error와 이전 Error 사이의 차 */ 
   
void CEncoder::EncoderPID()
{
  if (millis() - prev_pid_millis >= PID_INTERVAL)
  {
    prev_pid_millis = millis();
    double prev_realError;
    double pControl, iControl, dControl;
      
    realError = target - current;             //for P
    accError += realError;                    //for I
    if (current <= 0.07) accError = 0; 
  
    errorGap = realError - prev_realError;  //for D  
    pControl = P_GAIN_ * realError;
    iControl = I_GAIN_ * (accError * PID_INTERVAL / 1000);
    dControl = D_GAIN_ * (errorGap / PID_INTERVAL);  // d control -- D_GAIN_ * (current - prev_current) / PID_INTERVAL

    iControl = constrain(iControl, -0.3, 0.3);

    dControl = prev_dControl*0.8 + dControl*0.2;
    prev_dControl = dControl;
    pidControl = current + pControl + iControl + dControl;

    prev_realError = realError; // update prev_realError
  }
}

uint8_t CEncoder::PIDtoPWM()
{
  int pidControl_to_pwm;
  
  pidControl_to_pwm = int((pidControl+0.42)*25./1.72);  // off pid 
  
  if (pidControl_to_pwm <= 5) pidControl_to_pwm = 0; // delete noise 
  pwm = constrain(pidControl_to_pwm, 0, 255);
}

// 이 파트에서 급발진에 대한 부분을 없앨 수도 있을 거 같음. 
