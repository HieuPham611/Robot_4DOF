#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

#define RAD2DEG 180 / PI
#define PITCH_ANGLE 0
#define HOME_ANGLE_Y 137
#define HOME_ANGLE_Z 111
#define MICRO_STEP 200 * 32

#define LINK1 122
#define LINK2 140
#define LINK3 140
#define LINK4 80
// công tắc hành trình
#define END_X 9
#define END_Y 10
#define END_Z 11
// khai báo chân các step
#define step_x 2
#define dir_x 5
#define step_y 3
#define dir_y 6
#define step_z 4
#define dir_z 7
#define enable 8
// STEP
float valuestep = 0;
#define maxspeed 5000
#define maxaccel 3000


Servo myservo;
int kep = 180;
int mo = 0;

AccelStepper Step_x(1, step_x, dir_x, enable);
AccelStepper Step_y(1, step_y, dir_y, enable);
AccelStepper Step_z(1, step_z, dir_z, enable);

void home_x() {
  int homeX = 0;
  Step_x.setMaxSpeed(maxspeed);
  Step_x.setAcceleration(maxaccel);
  Step_x.enableOutputs();

  while (digitalRead(END_X) != 0) {
    Step_x.moveTo(homeX);
    homeX++;
    Step_x.run();
  }
  Step_x.setCurrentPosition(0);
  homeX = 0;
}

void home_yz() {
  int homeY = 0;
  int homeZ = 0;
  Step_y.setMaxSpeed(maxspeed);
  Step_z.setMaxSpeed(maxspeed);
  Step_y.setAcceleration(maxaccel);
  Step_z.setAcceleration(maxaccel);
  while (digitalRead(END_Y) and digitalRead(END_Z)) {
    Step_y.moveTo(homeY);
    Step_z.moveTo(homeZ);
    homeY++;
    homeZ--;
    Step_y.run();
    Step_z.run();
  }

  while (digitalRead(END_Y)) {
    Step_y.moveTo(homeY);
    homeY++;
    Step_y.run();
  }
  Step_y.setCurrentPosition(0);  // giá trị măc định của y là 137 độ 11134

  while (digitalRead(END_Z)) {
    Step_z.moveTo(homeZ);
    homeZ--;
    Step_z.run();
  }
  Step_z.setCurrentPosition(0);

  homeY = 0;
  homeZ = 0;
}
long degtostep(float deg) {
  valuestep = deg / 360 * MICRO_STEP * 32.0 / 7.0;  // deg/(1.8/32)*32/9;
  return valuestep;
}
void run_to_potision(byte ser, float x, float y, float z) {
  x = degtostep(x);
  y = degtostep(y);
  z = degtostep(z);
  myservo.write(ser);
  long y_1 = ((HOME_ANGLE_Y / 360.0) * MICRO_STEP * 32.0 / 7.0);  // 137 & 111 là giá trị bước của robot ở vị trí HOME
  long z_1 = (-(HOME_ANGLE_Z / 360.0) * MICRO_STEP * 32.0 / 7.0); // 32/7 là tỉ số truyền của step và rô đai
  Step_x.moveTo(-x);
  // Serial.println(-(y_1 - y));
  // Serial.println(Step_y.currentPosition());
  // Serial.println(-(90.0 / 360.0) * 200 * 32 * 32 / 7);
  Step_y.moveTo(-(y_1 - y));
  // Step_z.moveTo(-(z_1 - z));
  Step_z.moveTo((y_1 - y) + (z_1 - z));
  // Serial.println((y_1 - y));
  // Serial.println((y_1 - y) + (z_1 - z));
  // Serial.println(Step_z.currentPosition());
  // Serial.println(-(90.0 / 360.0) * 200 * 32 * 32 / 7);
  while (Step_x.distanceToGo() != 0 or Step_y.distanceToGo() != 0 or Step_z.distanceToGo() != 0) {
    Step_x.run();
    Step_y.run();
    Step_z.run();
  }
}
void home() {
  myservo.write(kep);
  home_yz();
  home_x();
}

void invk(float x, float y, float z, float qt, double *theta) {
  float link1 = LINK1;
  float link2 = LINK2;
  float link3 = LINK3;
  float link4 = LINK4;
  double r = sqrt(x * x + y * y) - link4 * cos(qt);
  double s = link4 * sin(qt) + z - link1;
  theta[0] = atan2(y, x) * RAD2DEG;
  double D = (r * r + s * s - link2 * link2 - link3 * link3) / (2 * link2 * link3);
  double D1 = sqrt(abs(1 - D * D));
  double alpha = atan2(link3 * D1, link2 + link3 * D) * RAD2DEG;
  double phi = atan2(s, r) * RAD2DEG;
  theta[1] = phi + alpha;
  theta[2] = -acos(D) * RAD2DEG;
  theta[3] = qt - (theta[1] + theta[2]);
}
void FKN(double t1, double t2, double t3, double t4, float &x, float &y, float &z) {
    float a1 = LINK1;
    float a2 = LINK2;
    float a3 = LINK3;
    float a4 = LINK4;

    // Tính toán giá trị x, y, z trực tiếp từ các góc
    x = a1 * sin(t1) + a2 * cos(t1) * sin(t2) + a3 * cos(t1) * sin(t2 + t3) + a4 * cos(t1) * sin(t2 + t3 + t4);
    y = a1 * cos(t1) - a2 * sin(t1) * sin(t2) - a3 * sin(t1) * sin(t2 + t3) - a4 * sin(t1) * sin(t2 + t3 + t4);
    z = a2 * cos(t2) + a3 * cos(t2 + t3) + a4 * cos(t2 + t3 + t4);
}
void Path_plan(double t_realtime, double P0[], double Pf[], double tf, double &x, double &y, double &z) {
    // Chuyển đổi thời gian thực sang thời gian được sử dụng trong hàm (giây)
    double t = t_realtime / 1000.0; // Giả sử thời gian thực được đưa vào dưới định dạng mili-giây
    
    double v0[] = {0, 0, 0};
    double vf[] = {0, 0, 0};
    
    double a10 = P0[0];
    double a20 = P0[1];
    double a30 = P0[2];
 
    double a11 = v0[0];
    double a21 = v0[1];
    double a31 = v0[2];
   
    double a12 = 3/(tf*tf)*(Pf[0]-P0[0])-2/tf*v0[0]-1/tf*vf[0];
    double a22 = 3/(tf*tf)*(Pf[1]-P0[1])-2/tf*v0[1]-1/tf*vf[1];
    double a32 = 3/(tf*tf)*(Pf[2]-P0[2])-2/tf*v0[2]-1/tf*vf[2];
   
    double a13 = -2/(tf*tf*tf)*(Pf[0]-P0[0])+1/(tf*tf)*(vf[0]+v0[0]);
    double a23 = -2/(tf*tf*tf)*(Pf[1]-P0[1])+1/(tf*tf)*(vf[1]+v0[1]);
    double a33 = -2/(tf*tf*tf)*(Pf[2]-P0[2])+1/(tf*tf)*(vf[2]+v0[2]);
   
    x = a10 + a11*t + a12*t*t + a13*t*t*t;
    y = a20 + a21*t + a22*t*t + a23*t*t*t;
    z = a30 + a31*t + a32*t*t + a33*t*t*t;
}

void setup() {
  Serial.begin(9600);
  pinMode(END_X, INPUT_PULLUP);
  pinMode(END_Y, INPUT_PULLUP);
  pinMode(END_Z, INPUT_PULLUP);

  Step_x.setEnablePin(enable);
  Step_x.setPinsInverted(false, false, true);
  Step_x.enableOutputs();
  // Step_y.setEnablePin(enable);
  Step_y.setPinsInverted(false, false, true);
  Step_y.enableOutputs();
  // Step_z.setEnablePin(enable);
  Step_z.setPinsInverted(false, false, true);
  Step_z.enableOutputs();

  // home();
  myservo.attach(A1);
  myservo.write(kep);
  delay(1000);
  Serial.println("Starting");
}

void loop() {
  double theta[4];
  invk(300, 0, 80, PITCH_ANGLE, theta);
  Serial.print(theta[0]);
  Serial.print('\t');
  Serial.print(theta[1]);
  Serial.print('\t');
  Serial.print(theta[2]);
  Serial.println('\t');
  // run_to_potision(kep, theta[0], theta[1], theta[2]);
  delay(500);
  invk(200, 0, 60, PITCH_ANGLE, theta);
  Serial.print(theta[0]);
  Serial.print('\t');
  Serial.print(theta[1]);
  Serial.print('\t');
  Serial.print(theta[2]);
  Serial.println('\t');
  // run_to_potision(mo, degtostep(0), degtostep(29.7025), degtostep(-99.4795));
  // delay(1000);
  // for (int i = 0; i < 3; i++)
  // {
  //   run_to_potision(kep, degtostep(0), degtostep(29.7025), degtostep(-99.4795));
  //   delay(500);
  //   run_to_potision(kep, degtostep(0), degtostep(33.8342), degtostep(-122.3162));
  //   delay(500);
  //   run_to_potision(kep, degtostep(14.0362), degtostep(33.6937), degtostep(-119.7317));
  //   delay(500);
  //   run_to_potision(kep, degtostep(11.3099), degtostep(28.9650), degtostep(-96.9571));
  //   delay(500);
  //   run_to_potision(kep, degtostep(0), degtostep(29.7025), degtostep(-99.4795));
  //   delay(500);
  // }
  // run_to_potision(mo, degtostep(0), degtostep(29.7025), degtostep(-99.4795));
  // delay(1000);
  home();
  while (true)
    ;
}