#include <PID_v1.h>
#include <Servo.h>
#include <Arduino.h>
#include "variables.h"

#define DIST_S 200*58.2
#define PING_INTRERVAL 33

/*
센서는 앞에 s가 붙음
모터는 앞에 m이 붙음
함수는 구분자가 대문자
변수는 구분자가 언더바

*/
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

float getHeight()
{        
    int data = analogRead(s_bottom_lazer2); 
	int volt = map(data, 0, 1023, 0, 5000);
	float distance = (21.61/(volt-0.1696))*1000; 
    return distance;
}

long getSonarDis(int TRIG,int ECHO)
{
    long dist;
    digitalWrite(TRIG, LOW); 
    delayMicroseconds(2); 
    digitalWrite(TRIG, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    dist = pulseIn(ECHO, HIGH,DIST_S)/58.2;
    return(dist);
}

float getLeftDis()
{
    int distance=1;
     return distance;   
}

void forward(int speed,int time)
{
        
}
//정지
void hovering(int time)
{
    
}

//상승, 하강
void throttle()
{
    
}
//자우로 이동
void roll()
{
    
}

void setup() {
    Serial.begin(38400);                  //시리얼 통신 초기화
    
    pinMode(s_side_sonar1,OUTPUT);
    pinMode(s_side_sonar2,OUTPUT);
    pinMode(s_side_sonar2,INPUT);
    pinMode(s_side_sonar1,INPUT);
//                                      초음파 센서

    //초음파 센서 설정
    pingTimer=millis();
    
}


void loop() {
}

//초음파 거리 측정

