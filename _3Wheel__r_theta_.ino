#include <I2Cdev.h>
#include <PID_v1.h>
#include <math.h>

double Setpoint, Input, Output, control;
double Kp = 13.0, Ki = 0.0, Kd = 0.5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

char q;
int xpwm = 120, h;
#define dir10 26
#define dir20 30
#define dir11 28

int valy, tempy;
int valp, tempp;
int valr, tempr;
int flag = 0;

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion m;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int yaw;
int mil = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
void setup()
{
  InitGyro();
  // Serial.begin(9600);
  //Serial2.begin(9600);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-120, 120);
  Setpoint = 0.00;
  pins();
  pwm();
}
void pwm()
{
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);

  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM40);
  TCCR4B = (1 << WGM42) | (1 << CS41);

  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21) ;
  TCCR2B = (1 << CS21);
}
void pins()
{
  pinMode(dir10, OUTPUT);
  pinMode(dir20, OUTPUT);
  pinMode(dir11, OUTPUT);
  //pinMode(dir22, OUTPUT);
}
void drive_motor(int mot, int dir, int pwm)
{
  //LOCOMOTION : 1,2,3
  //CLIMBING   : 4,5,6

  switch (mot)
  {
    //////////motor 1/////////
    case 1:
      switch (dir)
      {
        case 1:
          digitalWrite(dir10, LOW);
          break;

        case 2:
          digitalWrite(dir10, HIGH);
          break;
      }
      OCR4A = pwm;
      break;

    //////////motor 2/////////
    case 2:
      switch (dir)
      {
        case 1:
          digitalWrite(dir20, LOW);
          break;

        case 2:
          digitalWrite(dir20, HIGH);
          break;
      }
      OCR4C = pwm;
      break;

    //////////motor 3/////////
    case 3:
      switch (dir)
      {
        case 1:
          digitalWrite(dir11, LOW);
          break;

        case 2:
          digitalWrite(dir11, HIGH);
          break;
      }
      OCR4B = pwm;
      break;


  }
}
void gyro()
{
  //////////////////////////////Gyro initialisation starts
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&m, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &m);
    mpu.dmpGetYawPitchRoll(ypr, &m, &gravity);
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180/M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180/M_PI);
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180/M_PI);
#endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  if (flag < 3)
  {
    tempy = (ypr[0] * 180 / M_PI);
    tempp = (ypr[1] * 180 / M_PI);
    tempr = (ypr[2] * 180 / M_PI);
    flag++; Input = tempy;

    yaw = tempy;
    Serial.print(tempy); Serial.print("\t");
    //Serial.print(tempp);Serial.print("\t");
    //Serial.print(tempr);Serial.println("");
  }
  else
  {
    valy = (ypr[0] * 180 / M_PI);
    valy = valy - tempy;
    valp = (ypr[1] * 180 / M_PI);
    valp = valp - tempp;
    valr = (ypr[2] * 180 / M_PI);
    valr = valr - tempr;
    Input = valy;
    yaw = valy;
    Serial.print(valy); Serial.print("\t");
    //Serial.print(valp);Serial.print("\t");
    //Serial.print(valr);Serial.println("");
  }
  ////////////////////////////////////////////Gyro initialisation ends
}
void cases(char h)
{
  switch (h)
  {
    case 'w'://front
      Serial.println("Front Auto");
      if (Output < 0)
      {
        drive_motor(1, 1, 0);
        drive_motor(2, 1, (xpwm + Output));
        drive_motor(3, 2, (xpwm - Output));
      }
      else if (Output > 0)
      {
        drive_motor(1, 1, 0);
        drive_motor(2, 1, (xpwm + Output));
        drive_motor(3, 2, (xpwm - Output));
      }
      else
      {
        drive_motor(1, 1, 0);
        drive_motor(2, 1, xpwm);
        drive_motor(3, 2, xpwm);

      }
      break;

    case 'd'://right
      Serial.println("Right");
      /*drive_motor(1, 1, 163);
        drive_motor(2, 2, 80);
        drive_motor(3, 2, 85);
      */
      if (Output < 0)
      {
        drive_motor(1, 1, 80);
        drive_motor(2, 2, (xpwm + Output));
        drive_motor(3, 2, (xpwm - Output));
      }
      else if (Output > 0)
      {
        drive_motor(1, 1, 80);
        drive_motor(2, 2, (xpwm + Output));
        drive_motor(3, 2, (xpwm - Output));
      }
      else
      {
        drive_motor(1, 1, xpwm * 1.8);
        drive_motor(2, 1, xpwm);
        drive_motor(3, 2, xpwm);

      }
      break;

    case 'x'://Stop
      Serial.println("Stop");
      drive_motor(1, 1, 0);
      drive_motor(2, 1, 0);
      drive_motor(3, 1, 0);

      break;

    case 'a'://left
      Serial.println("Left");
      /*drive_motor(1, 2, 160);
        drive_motor(2, 1, 80);
        drive_motor(3, 1, 80);*/

      if (Output < 0)
      {
        drive_motor(1, 2, xpwm);
        drive_motor(2, 1, xpwm / 2 + Output);
        drive_motor(3, 1, xpwm / 2 - Output * 1.2);
      }
      else if (Output > 0)
      {
        drive_motor(1, 2, xpwm);
        drive_motor(2, 1, xpwm / 2 + Output);
        drive_motor(3, 1, xpwm / 2 - Output);
      }
      else
      {
        drive_motor(1, 2, xpwm);
        drive_motor(2, 1, xpwm / 2);
        drive_motor(3, 1, xpwm / 2);

      }
      break;
    case 'z'://clock
      drive_motor(1, 1, 30);
      drive_motor(2, 1, 30);
      drive_motor(3, 1, 30);
      Serial.println("Clock");
      break;

    case 's'://back
      drive_motor(1, 1, 0);
      drive_motor(2, 2, xpwm);
      drive_motor(3, 1, xpwm * 0.91);
      Serial.print("Back");
      break;
  }
}

int i = 0;
int sumx = 1;
int sumy = 2;
double hyp = 0, hyp2 = 0;
int theta;
void loop()
{
  gyro();
  hyp = (double)sqrt(sq(sumx) + sq(sumy));
  hyp = hyp * 1000;
  hyp2 = hyp * 1.25;
  Serial.print(hyp);Serial.print("\t");
  theta = round((atan(sumy / sumx)) * 180 / PI);
  theta = 90 - theta;
  Serial.print(theta);
  Serial.println("");

  if (i == 0)///////////////////////////Gotoxy starts
  {
    while (yaw < theta)
    {
      gyro();
      cases('z');
    }
    cases('x');
    Setpoint = theta;
    delay(500);
    mil = millis();
    while (millis() - mil < hyp2*1.13)
    {
      Serial.println("Front");
      gyro();
      myPID.Compute();
      cases('w');
    }
    cases('x');
    i++;
  }////////////////////////////////////Gotoxy ends
}

void InitGyro()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  delay(20000);
}
