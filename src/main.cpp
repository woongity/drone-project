#include "header.h"
#include "functions_proto.h"
#include "gyro_variables.h"
#include "motor_sensor_variables.h"

#define MOTORMAX 2000
#define MOTORMIN 1000
#define MPU_addr 0x68

#define M_PIN_LEFT_FRONT 5
#define M_PIN_LEFT_REAR 3
#define M_PIN_RIGHT_FRONT 9
#define M_PIN_RIGHT_REAR 6


//?쒓컙愿??媛?/
void calcDT(){
  t_now = micros();
  dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;
}

void calibAccelGyro(){
  float sum_acX = 0, sum_acY = 0, sum_acZ = 0;
  float sum_gyX = 0, sum_gyY = 0, sum_gyZ = 0;

  getAngle(); //媛?띾룄 ?먯씠濡??쇱꽌 ?쎌뼱?ㅼ엫

  //?됯퇏媛?援ы븯湲?
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
  t_prev = micros(); //珥덇린 t_prev媛믪? 洹쇱궗媛?/
}

void initYPR()
{
  //珥덇린 ?몃쾭留곸쓽 媛곷룄瑜??≪븘二쇨린 ?꾪빐??Roll, Pitch, Yaw ?곷낫?꾪꽣 援ы븯??怨쇱젙??10踰?諛섎났.
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
    //?됯퇏媛믪쓣 援ы븳??//
    base_roll_target_angle /= 10;
    base_pitch_target_angle /= 10;
    base_yaw_target_angle /= 10;

    //珥덇린 ?寃?媛곷룄瑜??≪븘以??//
    roll_target_angle = base_roll_target_angle;
    pitch_target_angle = base_pitch_target_angle;
    yaw_target_angle = base_yaw_target_angle;
  }
}

void calcAccelYPR(){
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / PI;

  accel_x = AcX - base_acX;
  accel_y = AcY - base_acY;
  accel_z = AcZ + (16384 - base_acZ);

  //accel_angle_y??Roll媛곸쓣 ?섎?//
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;

  //accel_angle_x??Pitch媛믪쓣 ?섎?//
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_angle_z = 0; //以묐젰 媛?띾룄(g)??諛⑺뼢怨??뺣컲???諛⑺뼢??媛由ы궎誘濡?媛?띾룄 ?쇱꽌瑜??댁슜?댁꽌???뚯쟾媛곸쓣 怨꾩궛?????녿떎.//
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
    Wire.beginTransmission(MPU_addr); //0x68踰덉? 媛믪쓣 媛吏??MPU-9250怨?I2C ?듭떊 ?쒖옉
    Wire.write(0x3B); //0x3B踰덉??????
    Wire.endTransmission(false); //?곗씠???꾩넚 ???ъ떆??硫붿깉吏 ?꾩넚(?곌껐? 怨꾩냽 吏??
    Wire.requestFrom(MPU_addr, 14, true); //0x68 踰덉???0x3B 遺??48源뚯? 珥?14諛붿씠?????
    AcX = Wire.read() << 8 | Wire.read(); //x異?媛?띾룄
    AcY = Wire.read() << 8 | Wire.read(); //y異?媛?띾룄
    AcZ = Wire.read() << 8 | Wire.read(); //z異?媛?띾룄
    Tmp = Wire.read() << 8 | Wire.read(); 
    GyX = Wire.read() << 8 | Wire.read(); //x異??먯씠濡?
    GyY = Wire.read() << 8 | Wire.read();// y異??먯씠濡?
    GyZ = Wire.read() << 8 | Wire.read();//z異??먯씠濡?
    
    
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

//?뺤?
void hovering()
{
        
}
//?곸듅, ?섍컯

void throttleDown()
{
    
}

//醫뚯슦濡??대룞
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
    //TODO : ?쇰떒 ?뚯뒪?몃? ?꾪빐 媛믪쓣 ?대윴 ?앹쑝濡?援ы쁽. 
}

void calcFilteredYPR()
{
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  //?곷낫?꾪꽣 媛?援ы븯湲?媛?띾룄, ?먯씠濡??쇱꽌???덉땐)//
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

//?먯씠濡??쇱꽌 珥덇린??
void gyroInit()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
}

//紐⑦꽣 珥덇린?? 罹섎━釉뚮젅?댁뀡 源뚯? 媛숈씠?댁쨲
void motorInit()
{
    delay(2500);
    m_left_rear.attach(M_PIN_LEFT_REAR);
    m_left_front.attach(M_PIN_LEFT_FRONT);
    m_right_rear.attach(M_PIN_RIGHT_REAR);
    m_right_front.attach(M_PIN_RIGHT_FRONT);
    initMotorSpeed();
    // calibMotor();
}


void dualPID(float target_angle,float angle_in,float rate_in,float stabilize_kp,float stabilize_ki,float rate_kp,float rate_ki,float &stabilize_iterm,float &rate_iterm,float &output){
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;

  //?댁쨷猷⑦봽PID?뚭퀬由ъ쬁
  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;//?덉젙??
  stabilize_iterm += stabilize_ki * angle_error * dt; //?덉젙???곷텇??

  desired_rate = stabilize_pterm;//?먰븯??媛곷룄

  rate_error = desired_rate - rate_in;
    
  rate_pterm = rate_kp * rate_error; //媛곸냽??鍮꾨???
  rate_iterm += rate_ki * rate_error * dt; //媛곸냽???곷텇??

  output = rate_pterm + rate_iterm + stabilize_iterm; //理쒖쥌 異쒕젰 : 媛곸냽??鍮꾨???+ 媛곸냽???곷텇??+ ?덉젙???곷텇??/
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
    }
    else {
        m_left_front_speed=speed+yaw_output+roll_output+pitch_output+20;  
        m_right_front_speed=speed-yaw_output-roll_output+pitch_output+20; //  
        m_left_rear_speed=speed+yaw_output-roll_output-pitch_output+20;         
        m_right_rear_speed=speed-yaw_output+roll_output-pitch_output+100;  // 
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
    
    
    
    m_right_rear.writeMicroseconds(m_right_rear_speed);
    m_right_front.writeMicroseconds(m_right_front_speed);
    m_left_front.writeMicroseconds(m_left_front_speed);
    m_left_rear.writeMicroseconds(m_left_rear_speed);
}

void setup() {
    Serial.begin(9600);
    gyroInit();
    calibAccelGyro();
    initDT();
    initYPR();
    motorInit();  
}

void loop()
{
    long time = millis();
    int now_speed=1100;
    getAngle();//?꾩옱 
    calcDT();
    calcAccelYPR(); //媛?띾룄 ?쇱꽌 Roll, Pitch, Yaw??媛곷룄瑜?援ы븯???⑥닔
    calcGyroYPR(); //?먯씠濡??쇱꽌 Roll, Pitch, Yaw??媛곷룄瑜?援ы븯???⑥닔
    calcFilteredYPR(); //?곷낫?꾪꽣瑜??곸슜??Roll, Pitch, Yaw??媛곷룄瑜?援ы븯???⑥닔
    calcYPRtoDualPID(); //?댁쨷猷⑦봽PID
    
    int count=0;
    if(count==3){
        return;
    }
    else{
        throttleUp(now_speed);
        if(time%5000==0){
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
        return;
    }
}