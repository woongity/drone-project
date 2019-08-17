#include "header.h"
#include "functions_proto.h"
#include "gyro_variables.h"
#include "motor_sensor_variables.h"

#define MOTORMAX 2000
#define MOTORMIN 1000
#define MPU_addr 0x68
#define ALPHA 0.96
#define BASE_SECOND 500

int m_pin_left_rear=9;
int m_pin_left_front=3;
int m_pin_right_rear=10;
int m_pin_right_front=11; 

int fixed_altitude_speed=1000;
bool is_window_found=false;


int count=0;
int height=0;
int s_left_side_distance=150;
int s_right_side_distance=150;
int s_front_left_distance=150;
int s_front_right_distance=150;

void calibAccelGyro(){
  float sum_acX = 0, sum_acY = 0, sum_acZ = 0;
  float sum_gyX = 0, sum_gyY = 0, sum_gyZ = 0;

  for(int i=0; i<10; i++){
    getAngle();
    sum_acX += AcX; sum_acY += AcY; sum_acZ += AcZ;
    sum_gyX += GyX; sum_gyY += GyY; sum_gyZ += GyZ;
    delay(500);
  }
  base_acX = sum_acX / 10; base_acY = sum_acY / 10; base_acZ = sum_acZ / 10;
  base_gyX = sum_gyX / 10; base_gyY = sum_gyY / 10; base_gyZ = sum_gyZ / 10;
}//5번 돌리면서 초기 값을 구한다.

void calcGyroYPR(){
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;

  gyro_x = (GyX - base_gyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - base_gyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - base_gyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
}//자이로 센서 x y z 계산

void initYPR()
{
  for(int i=0; i<10; i++){
    getAngle();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();

    base_roll_target_angle += filtered_angle_y;
    base_pitch_target_angle += filtered_angle_x;
    base_yaw_target_angle += filtered_angle_z;

    delay(500);
    base_roll_target_angle /= 10;
    base_pitch_target_angle /= 10;
    base_yaw_target_angle /= 10;

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
//roll pitch yaw의 가속도를 계산한다


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
}//자이로에서 raw값을 읽어온다

void getHeight()
{        
    
}
//높이를 측정
void getLeftDis()
{
        
}
//왼쪽 초음파 센서로 거리를 읽어온다
void goForward(int second)
{
    
}
//전진
void goLeft()
{
    

}
//왼쪽으로 간다
void goRight()
{
    
} //오른쪽으로 간다

void getFrontDisLeft()
{
    
}//앞 왼쪽에 달린 라이더 센서

void getFrontDisRight()
{
}//앞 오른쪽에 달린 라이더 센서

void calcFilteredYPR()
{
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;
  
  //변화량 적분
  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  //상보필터
  filtered_angle_x = ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z;
}

void getRightDis()
{
    
}

bool isStuckFront(bool direction)
{
    if(direction){
        getFrontDisRight();
        if(s_front_right_distance<30){
            return true;
        }else{
            return false;
        }
    }//left방향으로 가는중이라면
    else{
        getFrontDisLeft();
        if(s_front_left_distance<30){
            return true;
        }
        else {
            return false;
        }
    }//right 방향으로 가는 중이라면
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
    m_left_rear.attach(m_pin_left_rear);
    m_left_front.attach(m_pin_left_front);
    m_right_rear.attach(m_pin_right_rear);
    m_right_front.attach(m_pin_right_front);
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

void setSpeed(int speed)
{
    if(speed<=MOTORMIN){
        m_right_rear_speed=MOTORMIN;
        m_right_front_speed=MOTORMIN;
        m_left_rear_speed=MOTORMIN;
        m_left_front_speed=MOTORMIN;
    }
    else if(speed>=MOTORMAX){
        m_right_rear_speed=MOTORMAX;
        m_right_front_speed=MOTORMAX;
        m_left_rear_speed=MOTORMAX;
        m_left_front_speed=MOTORMAX;
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

void printValue()
{
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

void landing(int second,int nowSpeed)
{
    int temp=(nowSpeed-MOTORMIN)/second;
    for(int i=nowSpeed;i>=MOTORMIN;i-=temp){
        m_left_rear.writeMicroseconds(MOTORMIN);
        m_right_rear.writeMicroseconds(MOTORMIN);
        m_right_front.writeMicroseconds(MOTORMIN);
        m_left_front.writeMicroseconds(MOTORMIN);
        second-=1000;
        if(second==0) return;
    }    
}

void stable_throttle(int now_speed)
{
    getAngle(); 
    calcAccelYPR(); 
    calcGyroYPR(); 
    calcFilteredYPR(); 
    calcYPRtoDualPID(); 
    setSpeed(now_speed);
    printValue();
}


Timer right_side_distance_timer;
Timer left_side_distance_timer;
Timer bottom_distance_timer;
Timer front_left_distance_timer;
    
void setup() 
{
    Serial.begin(9600);
    gyroInit();
    calibAccelGyro();
    initYPR();
    motorInit();
        
}

void loop()
{   
    is_window_found=false;
    getHeight();
    while(height<100){
        stable_throttle(fixed_altitude_speed);
        fixed_altitude_speed+=10;
        getHeight();
    }
    if(count==3) {
        landing(fixed_altitude_speed,5000); 
        Serial.println("종료되었습니다");
        exit(0); 
    } 
    //벽을 세번 넘어가면 함수를 끝낸다.
    else{
        if(isStuckFront()){//벽 만남
            stable_throttle(fixed_altitude_speed);//호버링
            while(1){
                getLeftDis();
                if(s_left_side_distance<20){
                    break;                        
                }
                goLeft();
                if(!isStuckFront()){//창문이 막히지 않았다면, 창문 발견
                    is_window_found=true;
                    goForward(2000);//얘는 창문을 통과하는 함수
                    count++;
                    break;
                }//앞이 뚫려있다면 창문발견
            }
            while(s_right_side_distance>10){
                if(is_window_found){
                    break;
                }//이미 이전에 창문을 발견 했다면 그냥 패스한다.
                goRight();
                getRightDis();
                if(!isStuckFront()){
                    is_window_found=true;
                    break;
                }
            }
            if(is_window_found){
                goForward(BASE_SECOND);
            }
        }    
        else{   
            goForward(BASE_SECOND);            
        }//앞에 벽 안만남
    }
}