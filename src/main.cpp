#include "header.h"
#include "functions_proto.h"
#include "gyro_variables.h"
#include "motor_sensor_variables.h"

#define MOTORMAX 2000
#define MOTORMIN 1000
#define MPU_addr 0x68

#define M_PIN_LEFT_FRONT 10
#define M_PIN_LEFT_REAR 9
#define M_PIN_RIGHT_FRONT 11
#define M_PIN_RIGHT_REAR 3


void calibAccelGyro(){
  float sum_acX = 0, sum_acY = 0, sum_acZ = 0;
  float sum_gyX = 0, sum_gyY = 0, sum_gyZ = 0;

    
  for(int i=0; i<5; i++){
    getAngle();
    sum_acX += AcX; sum_acY += AcY; sum_acZ += AcZ;
    sum_gyX += GyX; sum_gyY += GyY; sum_gyZ += GyZ;
    delay(50);
  }
  base_acX = sum_acX / 5; base_acY = sum_acY / 5; base_acZ = sum_acZ / 5;
  base_gyX = sum_gyX / 5; base_gyY = sum_gyY / 5; base_gyZ = sum_gyZ / 5;
}//10번 돌리면서 초기 값을 구한다.

void calcGyroYPR(){
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;

  gyro_x = (GyX - base_gyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - base_gyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - base_gyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
}//자이로 센서 x y z 계산

void initYPR()
{
  for(int i=0; i<5; i++){
    getAngle();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();

    base_roll_target_angle += filtered_angle_y;
    base_pitch_target_angle += filtered_angle_x;
    base_yaw_target_angle += filtered_angle_z;

    delay(100);
    base_roll_target_angle /= 5;
    base_pitch_target_angle /= 5;
    base_yaw_target_angle /= 5;

    roll_target_angle = base_roll_target_angle;
    pitch_target_angle = base_pitch_target_angle;
    yaw_target_angle = base_yaw_target_angle;
  }
}
//roll pitch row 초기화

void calcAccelYPR(){
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / PI;

  accel_x = AcX - base_acX;
  accel_y = AcY - base_acY;
  accel_z = AcZ + (16384 - base_acZ);

  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_angle_z = 0; 
}

// void initMotorSpeed()
// {
//     m_right_rear.writeMicroseconds(MOTORMIN);
//     delay(1000);
//     m_right_front.writeMicroseconds(MOTORMIN);
//     delay(1000);
//     m_left_front.writeMicroseconds(MOTORMIN);
//     delay(1000);
//     m_left_rear.writeMicroseconds(MOTORMIN);  
//     delay(1000);
// }


void getAngle()
{
    Wire.beginTransmission(MPU_addr); 
    Wire.write(0x3B); //0x3B에 쓸것임
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_addr, 14, true); 
    AcX = Wire.read() << 8 | Wire.read(); 
    AcY = Wire.read() << 8 | Wire.read(); 
    AcZ = Wire.read() << 8 | Wire.read(); 
    Tmp = Wire.read() << 8 | Wire.read(); 
    GyX = Wire.read() << 8 | Wire.read(); 
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
}


long getHeight()
{        
    return sonar.ping_cm();
}

long getLeftDis()
{
    int distance=1;
    return distance;   
}

void goForward()
{
}

void yaw()
{
    
}

void hovering()
{
        
}

void throttleDown()
{
    
}


void rollLeft()
{
    
}

void rollRight()
{
    
}

long getFrontDisRight()
{
    long distance=1;
    return distance;
}

void calcFilteredYPR()
{
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  filtered_angle_x = ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z;
}

long getFrontDisLeft()
{
    long distance=1;
    return distance;
}// lidar sensor distance

bool isStuckFront()
{
    long left_distance=getFrontDisLeft();
    long right_distance=getFrontDisRight();
    if(left_distance< 30|| right_distance<30){
        return true;
    }
    else return false;
}

void gyroInit()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
}

void motorInit()
{
    delay(2500);
    m_left_rear.attach(M_PIN_LEFT_REAR);
    m_left_front.attach(M_PIN_LEFT_FRONT);
    m_right_rear.attach(M_PIN_RIGHT_REAR);
    m_right_front.attach(M_PIN_RIGHT_FRONT);
    // initMotorSpeed();
}


void dualPID(float target_angle,float angle_in,float rate_in,float stabilize_kp,float stabilize_ki,float rate_kp,float rate_ki,float &stabilize_iterm,float &rate_iterm,float &output){
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;

  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error * dt; 

  desired_rate = stabilize_pterm;

  rate_error = desired_rate - rate_in;
    
  rate_pterm = rate_kp * rate_error; 
  rate_iterm += rate_ki * rate_error * dt;

  output = rate_pterm + rate_iterm + stabilize_iterm; 
}


void calcYPRtoDualPID(){
  roll_angle_in = filtered_angle_y;
  roll_rate_in = gyro_y;

  dualPID(roll_target_angle,roll_angle_in,roll_rate_in,roll_stabilize_kp,roll_stabilize_ki,roll_rate_kp,roll_rate_ki,roll_stabilize_iterm,roll_rate_iterm,roll_output); 
    
  pitch_angle_in = filtered_angle_x;
  pitch_rate_in = gyro_x;

  dualPID(pitch_target_angle,pitch_angle_in,pitch_rate_in,pitch_stabilize_kp,pitch_stabilize_ki,pitch_rate_kp,pitch_rate_ki,pitch_stabilize_iterm,pitch_rate_iterm,pitch_output);

  yaw_angle_in = filtered_angle_z;
  yaw_rate_in = gyro_z;

  dualPID(yaw_target_angle,yaw_angle_in,yaw_rate_in,yaw_stabilize_kp,yaw_stabilize_ki,yaw_rate_kp,yaw_rate_ki,yaw_stabilize_iterm,yaw_rate_iterm,yaw_output);
}

void throttleUp(int speed)
{
    if(speed<=MOTORMIN){
        m_right_rear_speed=MOTORMIN;
        m_right_front_speed=MOTORMIN;
        m_left_rear_speed=MOTORMIN;
        m_left_front_speed=MOTORMIN;
        return;
    }
    else {
        m_left_front_speed=speed+yaw_output+roll_output+pitch_output+20;  
        m_right_front_speed=speed-yaw_output-roll_output+pitch_output+20; //  
        m_left_rear_speed=speed+yaw_output-roll_output-pitch_output+20;         
        m_right_rear_speed=speed-yaw_output+roll_output-pitch_output+20;  // 
    }
    if(m_right_rear_speed>=MOTORMAX){
        m_right_rear_speed=MOTORMAX;
    }
    else if(m_right_rear_speed<=MOTORMIN){
        m_right_rear_speed=MOTORMIN;
    }
    
    if(m_right_front_speed>=MOTORMAX){
        m_right_front_speed=MOTORMAX;
    }
    else if(m_right_front_speed<=MOTORMIN){
        m_right_front_speed=MOTORMIN;
    }
    
    if(m_left_rear_speed>=MOTORMAX){
        m_left_rear_speed=MOTORMAX;
    }
    else if(m_left_rear_speed<MOTORMIN){
        m_left_rear_speed=MOTORMIN;
    }
    
    if(m_left_front_speed>=MOTORMAX){
        m_left_front_speed=MOTORMAX;
    }
    else if(m_left_front_speed<=MOTORMIN){
        m_left_front_speed=MOTORMIN;
    }
    
    
    m_right_rear.writeMicroseconds(m_right_rear_speed);
    m_right_front.writeMicroseconds(m_right_front_speed);
    m_left_front.writeMicroseconds(m_left_front_speed);
    m_left_rear.writeMicroseconds(m_left_rear_speed);
}

void setup() {
    Serial.begin(9600);
    gyroInit();
    calibAccelGyro();
    initYPR();
    motorInit();  
}

void loop()
{
    int now_speed=1100;
    getAngle();//초기 각도 계산 
    calcAccelYPR(); 
    calcGyroYPR(); 
    calcFilteredYPR(); 
    calcYPRtoDualPID(); 
    
    if(getHeight()<40){
        
    }
    throttleUp(now_speed);
    Serial.print("x accel speed : ");Serial.println(AcX);
    Serial.print("y accel speed  : ");Serial.println(AcY);
    Serial.print("z accel speed : ");Serial.println(AcZ);
    Serial.print("x gyro speed : ");Serial.println(GyX);
    Serial.print("y gyro speed : ");Serial.println(GyY);
    Serial.print("z gyro speed : ");Serial.println(GyZ);
    Serial.print("left rear motor speed : ");
    Serial.println(m_left_rear_speed);
    
    Serial.print("right rear motor speed : ");
    Serial.println(m_right_rear_speed);
    Serial.print("left front motor speed : ");
    Serial.println(m_left_front_speed);
    
    Serial.print("right front motor speed : ");
    Serial.println(m_right_front_speed);
    Serial.println();
}