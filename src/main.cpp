/*
센서는 앞에 s가 붙음
모터는 앞에 m이 붙음
함수는 구분자가 대문자
변수는 구분자가 언더바


*/
// http://blog.naver.com/PostView.nhn?blogId=rlrkcka&logNo=221380249135&parentCategoryNo=&categoryNo=18&viewDate=&isShowPopularPosts=false&from=postView
// http://reefwingrobotics.blogspot.com/2018/04/arduino-self-levelling-drone-part-5.html
// https://sensibilityit.tistory.com/455?category=657462
// http://kr.bluesink.io/t/mpu-9250/36/5
// https://raduino.tistory.com/13
//     m_left_rear.attach(3);
//     m_left_front.attach(5);
//     m_right_rear.attach(6);
//     m_right_front.attach(9);

#include "header.h"
#include "functions.h"
#include "gyro_variables.h"
#include "motor_sensor_variables.h"

#define PING_INTRERVAL 33
#define MOTORMAX 2000
#define MOTORMIN 1000
#define MPU_addr 0x68
//모터                    
 
//mpu 기준으로 일단 구현

//시간관련 값//

void calcDT(){
  t_now = micros();
  dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;
}
//모터 캘리브레이션을 진행한다. 
void calibMotor()
{
    m_left_rear.writeMicroseconds(MOTORMAX);
    m_left_front.writeMicroseconds(MOTORMAX);
    m_right_rear.writeMicroseconds(MOTORMAX);
    m_right_front.writeMicroseconds(MOTORMAX);
    
    m_left_rear.writeMicroseconds(MOTORMIN);
    m_left_front.writeMicroseconds(MOTORMIN);
    m_right_rear.writeMicroseconds(MOTORMIN);
    m_right_front.writeMicroseconds(MOTORMIN);
}
// 현재 자이로 센서에서 받아들이는 초기값을 설정한다. -> 초기에 정지한 상태에서 현재값을 읽어들인다.
void calibAccelGyro(){
  float sum_acX = 0, sum_acY = 0, sum_acZ = 0;
  float sum_gyX = 0, sum_gyY = 0, sum_gyZ = 0;

  getAngle(); //가속도 자이로 센서 읽어들임

  //평균값 구하기
  for(int i=0; i<10; i++){
    getAngle();
    sum_acX += AcX; sum_acY += AcY; sum_acZ += AcZ;
    sum_gyX += GyX; sum_gyY += GyY; sum_gyZ += GyZ;
    delay(100);
  }
  base_acX = sum_acX / 10; base_acY = sum_acY / 10; base_acZ = sum_acZ / 10;
  base_gyX = sum_gyX / 10; base_gyY = sum_gyY / 10; base_gyZ = sum_gyZ / 10;
}

void calcGyroYPR(){
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;

  gyro_x = (GyX - base_gyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - base_gyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - base_gyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
}

void initDT(){
  t_prev = micros(); //초기 t_prev값은 근사값//
}

void initYPR(){
  //초기 호버링의 각도를 잡아주기 위해서 Roll, Pitch, Yaw 상보필터 구하는 과정을 10번 반복한다.
  for(int i=0; i<10; i++){
    getAngle();
    calcDT();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();

    base_roll_target_angle += filtered_angle_y;
    base_pitch_target_angle += filtered_angle_x;
    base_yaw_target_angle += filtered_angle_z;

    delay(100);
    //평균값을 구한다.//
  base_roll_target_angle /= 10;
  base_pitch_target_angle /= 10;
  base_yaw_target_angle /= 10;

  //초기 타겟 각도를 잡아준다.//
  roll_target_angle = base_roll_target_angle;
  pitch_target_angle = base_pitch_target_angle;
  yaw_target_angle = base_yaw_target_angle;
    }
}

void calcAccelYPR(){
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;

  accel_x = AcX - base_acX;
  accel_y = AcY - base_acY;
  accel_z = AcZ + (16384 - base_acZ);

  //accel_angle_y는 Roll각을 의미//
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;

  //accel_angle_x는 Pitch값을 의미//
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_angle_z = 0; //중력 가속도(g)의 방향과 정반대의 방향을 가리키므로 가속도 센서를 이용해서는 회전각을 계산할 수 없다.//
}

void initMotorSpeed(){
  analogWrite(3, MOTORMIN);
  delay(1000);
  analogWrite(5, MOTORMIN);
  delay(1000);
  analogWrite(6, MOTORMIN);
  delay(1000);
  analogWrite(9, MOTORMIN);
  delay(1000);
}



void getAngle()
{
    Wire.beginTransmission(MPU_addr); //0x68번지 값을 가지는 MPU-9250과 I2C 통신 시작
    Wire.write(0x3B); //0x3B번지에 저장
    Wire.endTransmission(false); //데이터 전송 후 재시작 메새지 전송(연결은 계속 지속)
    Wire.requestFrom(MPU_addr, 14, true); //0x68 번지에 0x3B 부터 48까지 총 14바이트 저장
    AcX = Wire.read() << 8 | Wire.read(); //x축 가속도
    AcY = Wire.read() << 8 | Wire.read(); //y축 가속도
    AcZ = Wire.read() << 8 | Wire.read(); //z축 가속도
    Tmp = Wire.read() << 8 | Wire.read(); 
    GyX = Wire.read() << 8 | Wire.read(); //x축 자이로
    GyY = Wire.read() << 8 | Wire.read();// y축 자이로
    GyZ = Wire.read() << 8 | Wire.read();//z축 자이로
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
    hovering(0.3);
    long distance=getFrontDisLeft();
}

void yaw()
{
    
}

//정지
void hovering()
{
        
}

//상승, 하강

void throttleUp(int speed)
{
    m_left_rear.writeMicroseconds(speed);
    m_right_rear.writeMicroseconds(speed);
    m_left_front.writeMicroseconds(speed);
    m_right_front.writeMicroseconds(speed);
    Serial.println(getHeight());
        
    hovering();
}

void throttleDown()
{
    
}

//좌우로 이동
void rollLeft()
{
    
}

void rollRight()
{
    
}

int pidControl()
{
            
}

long getFrontDisRight()
{
    long distance=1;
    return distance;
}
void calcFilteredYPR(){
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  //상보필터 값 구하기(가속도, 자이로 센서의 절충)//
  filtered_angle_x = ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z;
}

long getFrontDisLeft()
{
    long distance;
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
    m_left_rear.attach(3);
    m_left_front.attach(5);
    m_right_rear.attach(6);
    m_right_front.attach(9);
    calibMotor();
}


void setup() {
    Serial.begin(38400);
    gyroInit();
    gyroInit();
    motorInit();  
}

void loop()
{
    int count=0;
    if(count==3){
        return;
    }
    else{
        while(getHeight()<=40){ 
            throttleUp(1000);
            delay(500);
        }    
    }
}


