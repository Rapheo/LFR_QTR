//Rapheo
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

float Kp = 2.0; //set up the constants value
float Ki = 0.18;
float Kd = 0.38;
int P;
int I;
int D;

int lastError = 0;

const uint8_t maxspeeda = 70;
const uint8_t maxspeedb = 70;
const uint8_t basespeeda = 60;
const uint8_t basespeedb = 60;

int direction_Leftmotor1_F= 5;
int direction_Leftmotor1_B = 4;
int enableA = 3;

int direction_Rightmotor2_F = 7;
int direction_Rightmotor2_B=6;
int enableB = 9;

boolean Ok = false;

void setup() {
 Serial.begin(9600);
  qtr.setTypeAnalog();

  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);//LEDON PIN

  pinMode(direction_Rightmotor2_B, OUTPUT);
  pinMode(direction_Rightmotor2_F, OUTPUT);
  pinMode(enableB, OUTPUT);    
  pinMode(direction_Leftmotor1_B, OUTPUT);
  pinMode(direction_Leftmotor1_F, OUTPUT);
  pinMode(enableA, OUTPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  forward_brake(0, 0);

}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  while (Ok == false) { 
     calibration();
      Ok = true;
    }

     PID_control();
}

void forward_brake(int posa, int posb) {
  //set the appropriate values for direction_Rightmotor2_F and direction_Leftmotor1_B so that the robot goes straight
  digitalWrite(direction_Rightmotor2_F, LOW);
  digitalWrite(direction_Rightmotor2_B, HIGH);
  digitalWrite(direction_Leftmotor1_B, HIGH);
  digitalWrite(direction_Leftmotor1_F, LOW);
  analogWrite(enableB, posa);
  analogWrite(enableA, posb);
}

void forward() {
  //set the appropriate values for direction_Rightmotor2_F and direction_Leftmotor1_B so that the robot goes straight
  digitalWrite(direction_Rightmotor2_F, LOW);
  digitalWrite(direction_Rightmotor2_B, HIGH);
  digitalWrite(direction_Leftmotor1_B, HIGH);
  digitalWrite(direction_Leftmotor1_F, LOW);
  analogWrite(enableB, 100);
  analogWrite(enableA, 100);
}
void back() {
  //set the appropriate values for direction_Rightmotor2_F and direction_Leftmotor1_B so that the robot goes straight
  digitalWrite(direction_Rightmotor2_F, HIGH);
  digitalWrite(direction_Rightmotor2_B, LOW);
  digitalWrite(direction_Leftmotor1_B, LOW);
  digitalWrite(direction_Leftmotor1_F, HIGH);
  analogWrite(enableB, 100);
  analogWrite(enableA, 100);
}
void left() {
  //set the appropriate values for direction_Rightmotor2_F and direction_Leftmotor1_B so that the robot goes straight
  digitalWrite(direction_Rightmotor2_F, LOW);
  digitalWrite(direction_Rightmotor2_B, HIGH);
  digitalWrite(direction_Leftmotor1_B, HIGH);
  digitalWrite(direction_Leftmotor1_F, LOW);
  analogWrite(enableB, 0);
  analogWrite(enableA, 100);
}
void right() {
  //set the appropriate values for direction_Rightmotor2_F and direction_Leftmotor1_B so that the robot goes straight
  digitalWrite(direction_Rightmotor2_F, LOW);
  digitalWrite(direction_Rightmotor2_B, HIGH);
  digitalWrite(direction_Leftmotor1_B, HIGH);
  digitalWrite(direction_Leftmotor1_F, LOW);
  analogWrite(enableB, 100);
  analogWrite(enableA, 0);
}
void stops() {
  //set the appropriate values for direction_Rightmotor2_F and direction_Leftmotor1_B so that the robot goes straight
  digitalWrite(direction_Rightmotor2_F, HIGH);
  digitalWrite(direction_Rightmotor2_B, LOW);
  digitalWrite(direction_Leftmotor1_B, LOW);
  digitalWrite(direction_Leftmotor1_F, HIGH);
  analogWrite(enableB, 0);
  analogWrite(enableA, 0);
}


void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  // Serial.print(motorspeed);Serial.print(" ");Serial.print(motorspeeda);Serial.print(" ");Serial.println(motorspeedb);
  forward_brake(motorspeeda, motorspeedb);
  
}
