long getHeight();
long getRightDis(int TRIG,int ECHO);
long getLeftDis();
void forward(int speed,int time);
void yaw();
void hovering(float time);
void throttleUp();
void throttleDown();
void rollLeft();

void rollRight();

long getFrontDisRight();
long getFrontDisLeft();
bool isStuckFront();
void sonarSensorInit();