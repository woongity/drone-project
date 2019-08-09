// 자이로 센서를 이용한 각도
float gyro_x, gyro_y, gyro_z;


float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float accel_angle_x, accel_angle_y, accel_angle_z;

float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 1;
float roll_stabilize_ki = 0;
float roll_rate_kp = 1;
float roll_rate_ki = 0;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 1;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 1;
float pitch_rate_ki = 0;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 1;
float yaw_rate_ki = 0;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;

float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;

float base_acX, base_acY, base_acZ;  //가속도 평균값 저장 변수
float base_gyX, base_gyY, base_gyZ;  //자이로 평균값 저장 변수


MPU9250 mpu;
 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//mpu 기준으로 일단 구현
NewPing sonar(9,8,200);
double x; double y; double z;

//시간관련 값//
float dt;
unsigned long t_now;
unsigned long t_prev;