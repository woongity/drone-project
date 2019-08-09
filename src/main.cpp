#include "header.h"
#include "functions_proto.h"
#include "gyro_variables.h"
#include "motor_sensor_variables.h"

#define PING_INTRERVAL 33
#define MOTORMAX 2000
#define MOTORMIN 1000
#define MPU_addr 0x68

#define M_PIN_LEFT_FRONT 5
#define M_PIN_LEFT_REAR 3
#define M_PIN_RIGHT_FRONT 9
#define M_PIN_RIGHT_REAR 6


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
  //초기 호버링의 각도를 잡아주기 위해서 Roll, Pitch, Yaw 상보필터 구하는 과정을 10번 반복.
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

long getFrontDisRight()
{
    long distance=1;
    return distance;
    //TODO : 일단 테스트를 위해 값을 이런 식으로 구현. 
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

//자이로 센서 초기화
void gyroInit()
{
    //i2c연결
    Wire.begin();
    Wire.beginTransmission(MPU_addr); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
}

//모터 초기화, 캘리브레이션 까지 같이해줌
void motorInit()
{
    delay(2500);
    m_left_rear.attach(M_PIN_LEFT_REAR);
    m_left_front.attach(M_PIN_LEFT_FRONT);
    m_right_rear.attach(M_PIN_RIGHT_REAR);
    m_right_front.attach(M_PIN_RIGHT_FRONT);
    calibMotor();
}


void setup() {
    Serial.begin(38400);
    gyroInit();
    calibAccelGyro();
    initDT();
    initYPR();
    motorInit();  
}
void dualPID(float target_angle,float angle_in,float rate_in,float stabilize_kp,float stabilize_ki,float rate_kp,float rate_ki,float &stabilize_iterm,float &rate_iterm,float &output){
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;

  //이중루프PID알고리즘//
  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error * dt; //안정화 적분항//

  desired_rate = stabilize_pterm;

  rate_error = desired_rate - rate_in;

  rate_pterm = rate_kp * rate_error; //각속도 비례항//
  rate_iterm += rate_ki * rate_error * dt; //각속도 적분항//

  output = rate_pterm + rate_iterm + stabilize_iterm; //최종 출력 : 각속도 비례항 + 각속도 적분항 + 안정화 적분항//
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

void calcMotorSpeed(int speed)
{
    if(speed==MOTORMIN){
        m_right_rear_speed=MOTORMIN;
        m_right_front_speed=MOTORMIN;
        m_left_rear_speed=MOTORMIN;
        m_left_front_speed=MOTORMIN;
    }
    else {
        m_right_rear_speed=speed+yaw_output+roll_output+pitch_output+60;
        m_right_front_speed=speed-yaw_output-roll_output+pitch_output+60;
        m_left_front_speed=yaw_output-roll_output-pitch_output+60;
        m_left_rear_speed=speed-yaw_output+roll_output-pitch_output+60;
    }
    if(m_right_rear_speed>MOTORMAX){
        m_right_rear_speed=MOTORMAX;
    }
    else if(m_right_rear_speed<MOTORMIN){
        m_right_rear_speed=MOTORMIN;
    }
    
    if(m_right_front_speed>MOTORMAX){
        m_right_front_speed=MOTORMAX;
    }
    else if(m_right_front_speed<MOTORMIN){
        m_right_front_speed=MOTORMIN;
    }
    if(m_left_rear_speed>MOTORMAX){
        m_left_rear_speed=MOTORMAX;
    }
    else if(m_left_rear_speed<MOTORMIN){
        m_left_rear_speed=MOTORMIN;
    }
    if(m_left_front_speed>MOTORMAX){
        m_left_front_speed=MOTORMAX;
    }
    else if(m_left_front_speed<MOTORMIN){
        m_left_front_speed=MOTORMIN;
    }
    
    Serial.print("왼쪽 뒤 모터 속도 : ");
    Serial.println(m_left_rear_speed);
    
    Serial.print("오른쪽 뒤 모터 속도 : ");
    Serial.println(m_right_rear_speed);

    Serial.print("왼쪽 앞 모터 속도 : ");
    Serial.println(m_left_front_speed);
    
    Serial.print("오른쪽 앞 모터 속도 : ");
    Serial.println(m_right_front_speed);
    Serial.println();
    
    m_right_rear.writeMicroseconds(m_right_rear_speed);
    m_right_front.writeMicroseconds(m_right_front_speed);
    m_left_front.writeMicroseconds(m_left_front_speed);
    m_left_rear.writeMicroseconds(m_left_rear_speed);
}

void loop()
{
    long time = millis();
    int now_speed=1000;
    int target_speed=1100;//임의값
    getAngle();//현재 
    calcDT();
    calcAccelYPR(); //가속도 센서 Roll, Pitch, Yaw의 각도를 구하는 함수
    calcGyroYPR(); //자이로 센서 Roll, Pitch, Yaw의 각도를 구하는 함수
    calcFilteredYPR(); //상보필터를 적용해 Roll, Pitch, Yaw의 각도를 구하는 함수
    calcYPRtoDualPID(); //이중루프PID
    
    int count=0;
    if(count==3){
        return;
    }
    else{
        while(getHeight()<=40 || now_speed<1300){ 
            calcMotorSpeed(now_speed);
            delay(1000);
            now_speed+=1;
        }    
    }
}


