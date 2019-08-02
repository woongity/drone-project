#include <PID_v1.h>
#include <Servo.h>
#include <Arduino.h>
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
void setup() {
    Serial.begin(38400);                  //시리얼 통신 초기화
    
    pinMode(s_side_sonar1,OUTPUT);
    pinMode(s_side_sonar2,OUTPUT);
    pinMode(s_side_sonar2,INPUT);
    pinMode(s_side_sonar1,INPUT);
//                                      초음파 센서

    //초음파 센서 설정
        
}


void loop() 
{
    pingTimer=millis();
    bool check_front=isStuckFront();
    int count=0;
    if(check_front){ 
        goForward();
        count++;
    }
    else{
        while(getLeftDis()>10){
            rollLeft();    
        }        
    }
    if(count==3){
        return;
    }
}

//초음파 거리 측정

