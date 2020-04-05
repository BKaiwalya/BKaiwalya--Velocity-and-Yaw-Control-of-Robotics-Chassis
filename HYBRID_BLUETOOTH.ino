char q;
int xpwm = 120;
#define dir10 26
#define dir20 30
#define dir11 28
//#define dir22 32
void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  pins();
  pwm();
}
void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial2.available() > 0)
  {
    q = Serial2.read();
    switch (q)
    {
      case 'w'://front
        drive_motor(1, 1, 0);
        drive_motor(2, 1, xpwm);
        drive_motor(3, 2, xpwm);
        Serial.println("Front");
        break;

      case 'a'://left
        drive_motor(1, 2, 160);
        drive_motor(2, 1, 80);
        drive_motor(3, 1, 80);
        Serial.println("Left");
        break;

      case 's'://back
        drive_motor(1, 1, 0);
        drive_motor(2, 2, xpwm);
        drive_motor(3, 1, xpwm * 0.91);
        Serial.println("Back");
        break;

      case 'd'://right
        drive_motor(1, 1, 163);
        drive_motor(2, 2, 80);
        drive_motor(3, 2, 85);
        Serial.println("Right");
        break;

      case 'x'://stop
        drive_motor(1, 1, 0);
        drive_motor(2, 1, 0);
        drive_motor(3, 1, 0);
        Serial.println("Stop");
        break;

      case 'z'://clock
        drive_motor(1, 2, 90);
        drive_motor(2, 2, 90);
        drive_motor(3, 2, 90);
        Serial.println("Clock");
        break;

      case 'y'://anticlock
        drive_motor(1, 1, 90);
        drive_motor(2, 1, 90);
        drive_motor(3, 1, 90);
        Serial.println("Anti");
        break;

        default:
        drive_motor(1, 1, 0);
        drive_motor(2, 1, 0);
        drive_motor(3, 1, 0);
        break;


    }
  }


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

      //////////motor 4/////////
      /* case 3:
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
      */

      //////////motor 5/////////
      /* case 5:
         switch (dir)
         {
           case 1:
             digitalWrite(dir22, LOW);
             break;

           case 2:
             digitalWrite(dir22, HIGH);
             break;
         }
         OCR1A = pwm;
         break;

        //////////motor 6/////////
        case :
         switch (dir)
         {
           case 1:
             digitalWrite(dir33, LOW);
             break;

           case 2:
             digitalWrite(dir33, HIGH);
             break;
         }
         OCR1C = pwm;
         break;

      */
  }
}

