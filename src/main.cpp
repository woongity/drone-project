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
double Setpoint, Input, Output, ServoOutput;                                       
float duration, distance;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);   //PID객체 생성

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

int echoPin = 6;                 //초음파 핀번호 설정
int trigPin = 7;

float getHeight()
{        
    int data = analogRead(s_bottom_lazer2); 
	int volt = map(data, 0, 1023, 0, 5000);
	distance = (21.61/(volt-0.1696))*1000; 
    return distance;
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


float readPosition() {
    delay(40);                             //딜레이 설정 

  //초음파 센서 거리 측정 부분
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH); 
    distance = ((float)(340 * duration) / 10000) / 2;  

    if(distance > 30) {                   //공의 측정거리가 30cm 이상일 경우 최대 30으로 설정
        distance=30;
    }
    Serial.println(distance);             //시리얼 모니터로 공의 거리 출력
    return distance;                      //측정값 반환
}


void setup() {
    Serial.begin(38400);                  //시리얼 통신 초기화
    
    myServo.attach(servoPin);            //서보모터 핀번호 설정

    Input = readPosition();             //막대 위의 공의 위치를 측정값 함수 호출
  
    pinMode(s_side_sonar1,OUTPUT);
    pinMode(s_side_sonar2,OUTPUT);
    pinMode(s_side_sonar2,INPUT);
    pinMode(s_side_sonar1,INPUT);
//                                      초음파 센서

    //초음파 센서 설정
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);                                                       
    pingTimer=millis();
    

    myPID.SetMode(AUTOMATIC);               //PID모드를 AUTOMATIC으로 설정
    myPID.SetOutputLimits(-80,80);          //PID의 값을 최소 -80부터 최대 80까지 설정
}


void loop() {
    init();
    Setpoint = 15;                         //막대 중앙 위치(Set Point를 15cm로 설정)
    Input = readPosition();                //공의 위치 측정                            
    myPID.Compute();                       //PID계산 
    ServoOutput=100+Output;            //서보모터의 각도 설정(100도는 서보모터가 수평을 이루었을 때 각도) 
    myServo.write(ServoOutput);            //서보모터에게 값 전달
}

long trig_ultra(int TRIG,int ECHO)
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
