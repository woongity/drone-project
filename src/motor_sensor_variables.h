Servo m_left_front;
Servo m_left_rear;
Servo m_right_front;
Servo m_right_rear;

// newping variation(trigger, echo, max_cm)

const int s_bottom_sonar_echo_pin=12;
const int s_bottom_sonar_trigger_pin=13;

const int s_left_sonar_echo_pin=4;
const int s_left_sonar_trigger_pin=5;

const int s_right_sonar_echo_pin=14;
const int s_right_sonar_trigger_pin=15;

const int s_front_right_lidar_rx=16;
const int s_front_right_lidar_tx=17;

const int s_front_left_lidar_rx=6;
const int s_front_left_lidar_tx=7;

NewPing s_bottom_sonar(s_bottom_sonar_trigger_pin,s_bottom_sonar_echo_pin, 150);
NewPing s_left_sonar(s_left_sonar_trigger_pin, s_bottom_sonar_echo_pin, 150);
NewPing s_right_sonar(s_right_sonar_trigger_pin, s_left_sonar_echo_pin, 150);
// //센서들

int m_right_rear_speed;
int m_right_front_speed;
int m_left_rear_speed;
int m_left_front_speed;