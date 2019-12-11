#include <QTRSensors.h>
//LIBRERIAS

#define e1 5  // Enable 1-2
#define i1 8  // der motor 1
#define i2 7   // der motor 1
#define e2 6  // Enable 3-4
#define i3 3   // izq motor 2
#define i4 4   // izq motor 2
#define swt 12

// kp = 0.02, kd = 0.015, vel 100 //
// kp = 0.03, kd = 0.015, vel 75 //
// kp = 0.015, kd = 0.03, vel 50 //
float kp =0.02;
float kd = 0.05;
float ki =0;//aumenta 30
int vel = 75;
int errorp = 0,olderrorp = 0, errord = 0,errortotal = 0,errori,errori1,errori0=0,errorder,errord1,errord0=0,l;
float ld,li,lfd,lfi;
int b=0;
// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 5000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  pinMode(6, OUTPUT);
  pinMode(swt, OUTPUT);
  digitalWrite(9,HIGH);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(13);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 150; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  while(digitalRead(swt)==LOW){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  } 
  Serial.println(position);
  l=position;

 PIDd();
 //SET_MOTOR(100,100);
  }
  digitalWrite(i1, LOW); 
    digitalWrite(i2, LOW);
    digitalWrite(i3, LOW); 
    digitalWrite(i4, LOW);
    analogWrite(e1, 0);
  analogWrite(e2, 0);
}

void SET_MOTOR(int MOTOR1,int MOTOR2){
  if(MOTOR1>255)MOTOR1=255;
  else if (MOTOR1<-255)MOTOR1=-255;
  if(MOTOR2>255)MOTOR2=255;
  else if (MOTOR2<-255)MOTOR2=-255;
  //* MOTOR 1**
  if(MOTOR1<0){
    MOTOR1=-MOTOR1;
    digitalWrite(i3, LOW);
    digitalWrite(i4, HIGH);
  }
  else{
    digitalWrite(i3, HIGH); 
    digitalWrite(i4, LOW);
  }
  //* MOTOR 2**
  if(MOTOR2<0){
    MOTOR2=-MOTOR2;
    digitalWrite(i2, LOW);
    digitalWrite(i1, HIGH);
  }
  else{
    digitalWrite(i2, HIGH); 
    digitalWrite(i1, LOW);
  }
  analogWrite(e1, MOTOR1);
  analogWrite(e2, MOTOR2);
}

void PIDd(){
  errorp=(l-3500);
  errord=errorp-olderrorp;
  errori=errori-olderrorp;
  errortotal=kp*errorp + kd*errord+ ki*errori;
  olderrorp=errorp;
  SET_MOTOR(vel-errortotal,vel+errortotal);
}
