/*
센서는 앞에 s가 붙음
모터는 앞에 m이 붙음
함수는 구분자가 대문자
변수는 구분자가 언더바
*/
// http://blog.naver.com/PostView.nhn?blogId=rlrkcka&logNo=221380249135&parentCategoryNo=&categoryNo=18&viewDate=&isShowPopularPosts=false&from=postView
// http://reefwingrobotics.blogspot.com/2018/04/arduino-self-levelling-drone-part-5.html
// https://sensibilityit.tistory.com/455?category=657462
//     m_left_rear.attach(3);
//     m_left_front.attach(5);
//     m_right_rear.attach(6);
//     m_right_front.attach(9);

#include "header.h"
#include "functions.h"

#define DIST_S 200*58.2
#define PING_INTRERVAL 33
#define MOTORMAX 2000
#define MOTORMIN 1000
                     //서보의 핀번호
unsigned long int pingTimer;

float base_acX, base_acY, base_acZ;  //가속도 평균값 저장 변수
float base_gyX, base_gyY, base_gyZ;  //자이로 평균값 저장 변수
float Kp = 2.5;                //P게인 값
float Ki = 0;                  //I게인 값 
float Kd = 1;                  //D게인 값
    
//모터
Servo myServo;                                     //서보 객체 생성, 초기화
Servo m_left_front;
Servo m_left_rear;
Servo m_right_front;
Servo m_right_rear;

//센서들
const int s_side_sonar1=1;
const int s_side_sonar2=2;
const int s_bottom_lazer1=3;
const int s_bottom_lazer2=4; //드론 높이 제어 센서
const int s_front_lidar1=5;
const int s_front_lidar2=6;


const int MPU_addr=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//mpu 기준으로 일단 구현

double x; double y; double z;

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

void getAngle()
{
    Wire.beginTransmission(MPU_addr); //0x68번지 값을 가지는 MPU-6050과 I2C 통신 시작
    Wire.write(0x3B); //0x3B번지에 저장
    Wire.endTransmission(false); //데이터 전송 후 재시작 메새지 전송(연결은 계속 지속)
    Wire.requestFrom(MPU_addr, 14, true); //0x68 번지에 0x3B 부터 48까지 총 14바이트 저장
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
}
//gyro sensor get x,y,z
                 
long getHeight()
{        
    int data = analogRead(s_bottom_lazer2); 
	int volt = map(data, 0, 1023, 0, 5000);
	float distance = (21.61/(volt-0.1696))*1000; 
    return distance;
}

long getRightDis(int TRIG,int ECHO) //초음파 센서로
{
    long dist=1;
    digitalWrite(TRIG, LOW); 
    delayMicroseconds(2); 
    digitalWrite(TRIG, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    dist = pulseIn(ECHO, HIGH,DIST_S)/58.2;//멕시멈값을 잡는다
    return(dist);
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
void hovering(float time)
{
    
}

//상승, 하강

void throttleUp(int target_speed)
{
    int speed = map(target_speed,0,1024,MOTORMIN,MOTORMAX);
    m_left_rear.writeMicroseconds(speed);
    m_right_rear.writeMicroseconds(speed);
    m_left_front.writeMicroseconds(speed);
    m_right_front.writeMicroseconds(speed);
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
    m_left_rear.attach(3,MOTORMIN,MOTORMAX);
    m_left_front.attach(5,MOTORMIN,MOTORMAX);
    m_right_rear.attach(6,MOTORMIN,MOTORMAX);
    m_right_front.attach(9,MOTORMIN,MOTORMAX);
    //calibMotor();
}

void sonarSensorInit()
{
    pinMode(s_side_sonar1,OUTPUT);
    pinMode(s_side_sonar2,OUTPUT);
    pinMode(s_side_sonar2,INPUT);
    pinMode(s_side_sonar1,INPUT); 
}

void goThruWindow()
{
    
}

void setup() {
    Serial.begin(9600);
    //sonarSensorInit();
    //gyroInit();
    motorInit();  
}

void loop()
{
    long target_speed = 1200;
    int count=0;
    if(count==3){
        return;
    }
    throttleUp(target_speed);
    delay(100);
}


