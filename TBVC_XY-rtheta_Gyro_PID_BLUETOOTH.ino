/*
 * Author List : Kaiwalya Belsare, Sarang Badgujar , Avinash Kangare, Pratik Gaikwad
 * Filename : TBVC_XY-rtheta_Gyro_PID_BLUETOOTH
 * Functions : void go_forward(int fpwm)
 *             void go_forward_PID(int xpwm)
 *             void brake()
 *             void linear_distance_cm(unsigned int DistanceInCM)
 *             void doEncoder_L()
 *             void doEncoder_R()
 *             void InitEncoder()
 *             void left_sharp()
 *             void right_sharp()
 *             void pins()
 *             void angle_rotate(unsigned int Degrees)
 *             void rot_left_till(int ang)
 *             void count_zero()
 *             void go_forward_till(int x)
 *             void only_straight(int d)
 *             void rot_right_till(int ang)
 *             void gyro()
 *             void InitGyro()
 *             void gotoxy_r_theta(int x,int y)
 *             void gotoxy_XY(int x,int y)
 *             void setup()
 *             void loop()
 * Variables : 
               long unsigned int Count_L=0, Count_R=0;
               int i=0,z = 0,angle=0,d=0,L_pwm = 87,R_pwm = 80;
               float dist_L = 0,dist_R = 0,dist_av = 0;
               int arg_pwm = 100,arg_spwm = 20;
               char q,zum;
               int h,yaw=0,theta=0,ros;
               float vel_R,vel_L;
               double hyp = 0, pro =0 ,sq_side;
               int zom=0,u=0;
               int valy, tempy;
               int valp, tempp;
               int valr, tempr;
               int flag = 0;
 */


#include <I2Cdev.h>
#include <PID_v1.h>
double Setpoint, Input, Output, control;
double Kp = 11, Ki = 0.2, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Right Motor Encoder
#define Encoder_La 3 ////interrupt 1
#define Encoder_Lb 32
//Left Motor Encoder
#define Encoder_Ra 2 ////interrupt 0
#define Encoder_Rb 30
//Right Motor 
#define motorRdir 42
#define motorRpwm 44  //OC5C

//Left Motor
#define motorLdir 48
#define motorLpwm 46  //OC5A

#define PPR 1120
#define DIAMETER 11.0
#define circum PI*DIAMETER  
#define WIDTH 23.0
long unsigned int Count_L=0, Count_R=0;
int i=0,z = 0,angle=0,d=0,L_pwm = 87,R_pwm = 80;
float dist_L = 0,dist_R = 0,dist_av = 0;
int arg_pwm = 100,arg_spwm = 20;
char q,zum;
int h,yaw=0,theta=0,ros;
float vel_R,vel_L;
double hyp = 0, pro =0 ,sq_side;
int zom=0,u=0;
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


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

/*
 * Function Name:   setup
 * Input:           none
 * Output:          none
 * Logic:           initiates all functions and executes once.
 * Example call:    void setup();
 */

void setup()
{
  Serial2.begin(9600);
  Serial.begin(9600);
  InitGyro();
  
  InitEncoder();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-120, 120);
  Setpoint = 0.00;
  myPID.SetSampleTime(100);
  pins();
  attachInterrupt(1,doEncoder_L,RISING);
  attachInterrupt(0,doEncoder_R,RISING); 
}

/*
 * Function Name:   go_forward(int fpwm)
 * Input:           pwm in integer
 * Output:          none
 * Logic:           traverses in forward direction with no feedback.
 * Example call:    void go_forward(int fpwm);
 */


void go_forward(int fpwm)
{
    digitalWrite(motorRdir,LOW);
    digitalWrite(motorLdir,LOW);
    analogWrite(motorRpwm,fpwm);
    analogWrite(motorLpwm,fpwm*1.08);
    }
/*
 * Function Name:   go_forward_PID(int xpwm)
 * Input:           pwm in integer
 * Output:          none
 * Logic:           traverses in forward direction with gyro correction with PID
 * Example call:    void go_forward_PID(int xpwm);
 */


void go_forward_PID(int xpwm)
{
    myPID.Compute();
    if (Output < 0)
      {  
      
    digitalWrite(motorRdir,LOW);
    digitalWrite(motorLdir,LOW);
    analogWrite(motorRpwm,xpwm- Output);
    analogWrite(motorLpwm,xpwm*1.08 );
    
       }
      else if (Output > 0)
      {
      
    digitalWrite(motorRdir,LOW);
    digitalWrite(motorLdir,LOW);
    analogWrite(motorRpwm,xpwm );
    analogWrite(motorLpwm,(xpwm*1.08)+ Output);
      
      }
      else
      {
      
    digitalWrite(motorRdir,LOW);
    digitalWrite(motorLdir,LOW);
    analogWrite(motorRpwm,xpwm);
    analogWrite(motorLpwm,xpwm*1.08);
    
      }
}
/*
 * Function Name:   brake
 * Input:           none
 * Output:          none
 * Logic:           brakes the bot
 * Example call:    void brake();
 */

void brake()
{
    digitalWrite(motorRdir,HIGH);
    digitalWrite(motorLdir,HIGH);
    analogWrite(motorRpwm,0);
    analogWrite(motorLpwm,0);
    
  }
/*
 * Function Name:   linear_distance_cm
 * Input:           distance in cm in integer
 * Output:          none
 * Logic:           robot travels the specified distance taking feedback from encoders.
 * Example call:    linear_distance_cm(100);
 */
void linear_distance_cm(unsigned int DistanceInCM)
{
  float ReqdShaftCount = 0;
  unsigned long int ReqdShaftCountInt = 0;

  ReqdShaftCount = DistanceInCM*31.0; // division by resolution to get shaft count
  ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  Count_L = 0;
  Count_R = 0;
  
  while(1)
  {
    delay(1);
    if((Count_L+Count_R)*0.5 > ReqdShaftCountInt)
    {break;}
  }
  brake(); //Stop robot
}
/*
 * Function Name:   doEncoder_L
 * Input:           none
 * Output:          none
 * Logic:           increments a variable for left encoder
 * Example call:    attachInterrupt(0,doEncoder_L,FALLING);
 */
void doEncoder_L()
{
   Count_L++;
 /*if(digitalRead(Encoder_La) && !digitalRead(Encoder_Lb))
  {
    Count_L --;                                   //CCW for encoder shaft
  }
  if(digitalRead(Encoder_La) && digitalRead(Encoder_Lb))
  {
    Count_L ++;                                     //CW for encoder shaft
  }*/
}

/*
 * Function Name:   doEncoder_R
 * Input:           none
 * Output:          none
 * Logic:           increments a variable for left encoder
 * Example call:    attachInterrupt(0,doEncoder_R,FALLING);
 */
void doEncoder_R()
{
  Count_R++;
  /*if(digitalRead(Encoder_Ra) && !digitalRead(Encoder_Rb))
  {
    Count_R--;
                                       //CCW for encoder shaft
  }
  if(digitalRead(Encoder_Ra) && digitalRead(Encoder_Rb))
  {
     Count_R++;                                     //CW for encoder shaft
  }*/
}
/*
 * Function Name:   InitEncoder
 * Input:           none
 * Output:          none
 * Logic:           Set encoder pins as input and pull up interrupt pins
 * Example call:    void InitEncoder();
 */
void InitEncoder()
{
  pinMode(Encoder_La,INPUT);
  pinMode(Encoder_Ra,INPUT);
  pinMode(Encoder_Lb,INPUT);
  pinMode(Encoder_Rb,INPUT);
  
  digitalWrite(Encoder_La, HIGH);
  digitalWrite(Encoder_Ra, HIGH); 
}
/*
 * Function Name:   left_sharp
 * Input:           none
 * Output:          none
 * Logic:           turn left sharply
 * Example call:    void left_sharp();
 */

void left_sharp()
{
    digitalWrite(motorRdir,LOW);
    digitalWrite(motorLdir,HIGH);
    analogWrite(motorRpwm,arg_spwm);
    analogWrite(motorLpwm,arg_spwm);  
  }

 
/*
 * Function Name:   right_sharp
 * Input:           none
 * Output:          none
 * Logic:           turn right sharply
 * Example call:    void right_sharp();
 */

void right_sharp()
{
    digitalWrite(motorRdir,HIGH);
    digitalWrite(motorLdir,LOW);
    analogWrite(motorRpwm,arg_spwm);
    analogWrite(motorLpwm,arg_spwm);
  }
/*
 * Function Name:   pins
 * Input:           none
 * Output:          none
 * Logic:           sets pins as input or output 
 * Example call:    void pins();
 */
void pins()
{
 pinMode(motorRpwm,OUTPUT);     //motor1 pwm
 pinMode(motorRdir,OUTPUT);    //motor1 dir
 pinMode(motorLpwm,OUTPUT);     //motor2 pwm
 pinMode(motorLdir,OUTPUT);     //motor2 dir
}
/*
 * Function Name:   angle_rotate
 * Input:           degrees in integer
 * Output:          none
 * Logic:           robot rotates till input degrees taking feedback from encoders.
 * Example call:    angle_rotate(90);
 */
void angle_rotate(unsigned int Degrees)
{
  float ReqdShaftCount = 0;
  unsigned long int ReqdShaftCountInt = 0;

  ReqdShaftCount = (float) Degrees* 6.1; // division by resolution to get shaft count
  ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
  Count_L = 0;
  Count_R = 0;

  while (true)
  {
    delay(1);
    
    if((Count_L >= ReqdShaftCountInt) | (Count_R >= ReqdShaftCountInt))
    {break;}
  }
  brake(); //Stop robot
}
/*
 * Function Name:   rot_left_till(int ang)
 * Input:           degrees in integer
 * Output:          none
 * Logic:           robot rotates left until a certain angle.
 * Example call:    void rot_left_till(90);
 */

void rot_left_till(int ang)
{
  left_sharp();
  angle_rotate(ang);
  
  }
/*
 * Function Name:   count_zero()
 * Input:           degrees in integer
 * Output:          none
 * Logic:           sets counts of right and left encoder to zero
 * Example call:    void count_zero();
 */

void count_zero()
{
  Count_L = 0;
  Count_R = 0;
  dist_L = 0;
  dist_R = 0;
  dist_av = 0;
  
  
  }
/*
 * Function Name:   loop
 * Input:           none
 * Output:          none
 * Logic:           loops all functions written in while(1) fashion.
 * Example call:    void loop();
 */
void loop()
{
  dist_L = (((float)Count_L/PPR))*circum;
  dist_R = (((float)Count_R/PPR))*circum;
  dist_av = (0.5*(dist_L + dist_R));

  //Serial.print(dist_L);
  //Serial.print("\t");
  //Serial.println(dist_R);
  gyro();
  
    if(Serial2.available()>0)  
    {
      zum = Serial2.read();
      //Serial.print(zum);
      switch(zum)
      {
        case 'a':
        gotoxy_XY(100,100);
        //Serial.println("a");
        brake();
        break;
        
        
        case 'b':
        gotoxy_XY(150,100);
        //Serial.println("b");
        brake();
        break;

        

        default:
        brake();

        
        }
      
      
     
    
    }
   
 }
/*
 * Function Name:   go_forward_till(int x)
 * Input:           distance(cms) in integer
 * Output:          none
 * Logic:           goes forward only till specified distance with feedback from encoder
 * Example call:    void go_forward_till(int x);
 */
void go_forward_till(int x)
{
  go_forward(100);
  linear_distance_cm(x);
  }
/*
 * Function Name:   only_straight(int d)
 * Input:           distance(cms) in integer
 * Output:          none
 * Logic:           goes forward only till specified distance by taking gyro correction 
 * Example call:    void only_straight(int d);
 */

void only_straight(int d)
{
  int m=0,u=0;
   while( dist_av < d )
  {
  gyro();
  dist_L = (((float)Count_L/PPR))*circum;
  dist_R = (((float)Count_R/PPR))*circum;
  dist_av = (0.5*(dist_L + dist_R));
  if(m == 0){
  for(u=0; u <= arg_pwm ;u++)
   {
    go_forward(u);
    delay(20);
    }
  m++;
  }
  go_forward_PID(arg_pwm);
  
  //Serial.print(dist_av);
   //Serial.print('\t');
  //Serial.println(Output);
  }
}

/*
 * Function Name:   rot_right_till(int ang)
 * Input:           degrees in integer
 * Output:          none
 * Logic:           robot rotates right until a certain angle.
 * Example call:    void rot_right_till(int ang);
 */

void rot_right_till(int ang)
{
  right_sharp();
  angle_rotate(ang);
  
  }


/*
 * Function Name:   gyro
 * Input:           none
 * Output:          none
 * Logic:           computes readings of gyroscope
 * Example call:    void gyro();
 */

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
    flag++;
    Input = tempy;
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
 
  }


/*
 * Function Name:   InitGyro
 * Input:           none
 * Output:          none
 * Logic:           initiates gyroscope
 * Example call:    void InitGyro();
 */

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
    attachInterrupt(4, dmpDataReady, RISING);
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


/*
 * Function Name:   gotoxy_r_theta(int x,int y)
 * Input:           x,y (cms) in integers
 * Output:          none
 * Logic:           traverses to particular point using r-theta method
 * Example call:    void gotoxy_r_theta(int x,int y);
 */

void gotoxy_r_theta(int x,int y)
{
  int me=0,n1=0,n;
  gyro();
  dist_L = (((float)Count_L/PPR))*circum;
  dist_R = (((float)Count_R/PPR))*circum;
  dist_av = (0.5*(dist_L + dist_R));
  hyp = (double)sqrt(sq(x) + sq(y));
  theta = round((atan(y / x)) * 180 / PI);
  theta = 90 - theta;
  
  Serial.println(theta);
  if(me==0){
  
   rot_right_till(theta);
   brake();
   me++;}
   Setpoint = theta;
   count_zero();
   delay(2000); 
   while( dist_av < hyp )
  {
  gyro();
  dist_L = (((float)Count_L/PPR))*circum;
  dist_R = (((float)Count_R/PPR))*circum;
  dist_av = (0.5*(dist_L + dist_R));
  if(n1 == 0)
  {
    for(n=0; n < arg_pwm ;n++)
  {
    go_forward(arg_pwm);
  }
   n1++;
  }
  
  go_forward_PID(arg_pwm);
  Serial.print(dist_av);
  Serial.print('\t');
  Serial.println(hyp);
  }
   
} 

/*
 * Function Name:   gotoxy_XY(int x,int y)
 * Input:           x,y (cms) in integers
 * Output:          none
 * Logic:           traverses to particular point using XY method
 * Example call:    void gotoxy_XY(int x,int y);
 */

void gotoxy_XY(int x,int y)
{
  int m1,m2,n=0;
  
  while(dist_av < y)//1
  {
  gyro();
  dist_L = (((float)Count_L/PPR))*circum;
  dist_R = (((float)Count_R/PPR))*circum;
  dist_av = (0.5*(dist_L + dist_R));
  if(m1==0)
  {
   for(n=0; n < arg_pwm ;n++)
   {
    go_forward(u);
  
    }
  
   m1++;}
  
  go_forward_PID(arg_pwm);
  
 }
    brake();
    Serial.println(dist_av);
   delay(1000);
    rot_right_till(90);
    delay(1000);
    count_zero();
    Serial.println(dist_av);
    Setpoint = 90;
    while(dist_av < x)//2
  {
  gyro();
  dist_L = (((float)Count_L/PPR))*circum;
  dist_R = (((float)Count_R/PPR))*circum;
  dist_av = (0.5*(dist_L + dist_R));
  if(m2==0)
  {
   for(n=0; n < arg_pwm ;n++)
   {
    go_forward(n);
    
    }
  
   m2++;}
  
  go_forward_PID(arg_pwm);
  
 }
    brake();
  
  
  }

