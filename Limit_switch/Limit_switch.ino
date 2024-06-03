#include <Servo.h>
Servo myservo; 
int END_X = 9;int END_Y = 10;int END_Z = 11;
int pos = 180;
unsigned long previousMillis = 0;   // biến lưu trữ thời điểm trước đó

void delay_millis(long timedelay){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timedelay) {
    previousMillis = currentMillis;
  }
}

void setup() {
  Serial.begin(9600);
  myservo.attach(A1);
  pinMode(END_X, INPUT_PULLUP);
  pinMode(END_Y, INPUT_PULLUP);
  pinMode(END_Z, INPUT_PULLUP);
  myservo.write(0);
}

void loop() {
  Serial.print("x: ");
  Serial.print(digitalRead(END_X));
  Serial.print('\t');
  Serial.print("y: ");
  Serial.print(digitalRead(END_Y));
  Serial.print('\t');
  Serial.print("z: ");
  Serial.print(digitalRead(END_Z));
  Serial.println('\t');
  // for (int i = 0; i <=  180; i++)
  // {
  //   myservo.write(i);
  //   delay(10);
  // }
  // for (int i = 180; i >=  0; i--)
  // {
  //   myservo.write(i);
  //   delay(10);
  // }
  
}

