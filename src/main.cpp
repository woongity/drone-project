#include "header.h"
#include "functions.h"

#define DIST_S 200*58.2
#define PING_INTRERVAL 33

/*
센서는 앞에 s가 붙음
모터는 앞에 m이 붙음
함수는 구분자가 대문자
변수는 구분자가 언더바
*/
// http://blog.naver.com/PostView.nhn?blogId=rlrkcka&logNo=221380249135&parentCategoryNo=&categoryNo=18&viewDate=&isShowPopularPosts=false&from=postView
// http://reefwingrobotics.blogspot.com/2018/04/arduino-self-levelling-drone-part-5.html
const int servoPin = 9;                          //서보의 핀번호
unsigned long int pingTimer;


float Kp = 2.5;                //P게인 값
float Ki = 0;                  //I게인 값 
float Kd = 1;                  //D게인 값
    

const int MPU_addr=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//mpu 기준으로 일단 구현




const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265; int maxVal=402;

double x; double y; double z;


void getAngle()
{
    Wire.beginTransmission(MPU_addr); 
    Wire.write(0x3B); 
    Wire.endTransmission(false);     
    Wire.requestFrom(MPU_addr,14,true); 
    AcX=Wire.read()<<8|Wire.read(); 
    AcY=Wire.read()<<8|Wire.read(); 
    AcZ=Wire.read()<<8|Wire.read(); 
    int xAng = map(AcX,minVal,maxVal,-90,90); 
    int yAng = map(AcY,minVal,maxVal,-90,90); 
    int zAng = map(AcZ,minVal,maxVal,-90,90);

    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); 
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); 
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);    
}
//gyro sensor get x,y,z


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

long getHeight()
{        
    int data = analogRead(s_bottom_lazer2); 
	int volt = map(data, 0, 1023, 0, 5000);
	float distance = (21.61/(volt-0.1696))*1000; 
    return distance;
}

long getRightDis(int TRIG,int ECHO) //초음파 센서로
{
    long dist;
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

void throttleUp()
{
    
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
    
}

void sonarSensorInit()
{
    pinMode(s_side_sonar1,OUTPUT);
    pinMode(s_side_sonar2,OUTPUT);
    pinMode(s_side_sonar2,INPUT);
    pinMode(s_side_sonar1,INPUT); 
}

void setup() {
    
    Serial.begin(9600);
    sonarSensorInit();
    gyroInit();
    motorInit();
   
//                                      초음파 센서

    //초음파 센서 설정
}
void goThruWindow()
{
    
}

void loop() 
{
    pingTimer=millis();
    while(getHeight()>=40){
        throttleUp();
    }
    
    int count=0;
    if(!isStuckFront(){ 
        goForward();
        count++;
    }//앞에 창문이 없다면
    else{
        while(getLeftDis()>10){
            rollLeft();    
        }        
    }
    if(count==3){
        return; //코드를 종료한다.
    }
}

