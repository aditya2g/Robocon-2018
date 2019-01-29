#include <PS2X_lib.h> // Bill Porter's PS2 Library
#include <Servo.h>
PS2X ps2x;  //The PS2 Controller Class
Servo myservo;

const int dir1 = 22;
const int dir2 = 10;
const int dir3 = 8;
const int dir4 = 4;

const int pwm1 = 12;
const int pwm2 = 11;
const int pwm3 = 9;
const int pwm4 = 7;

const int turn_delay=0;


int Gcnt = 0;// Servo Variables
int DT = 200;

int dir=0;
int rotVolt;
int TotVlot=24;
int fcnt = 0; // front Rotate variables
int bcnt = 0; // back Rotate variables
int enVal = 0;
int EncoderPin[2] = {2,3};
int encoded = 0;
int lastEncoded = 0;
int rotDir = 5;
int rotPWM = 6;
int volt = 4;
int motS;
int L1,R1;
int Servcnt=0;
int motorVal;

void setup() {
  ps2x.config_gamepad(32,30,34,36, false, false);
//  gamepad(clock, command, attention, data, pressure sensitivity enabled, rumble enabled) // ORANGE 30
//  put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(pwm1,OUTPUT);//pwm 1
  pinMode(dir1,OUTPUT);//dir 1
  pinMode(pwm2,OUTPUT);// pwm2
  pinMode(dir2,OUTPUT);// dir2
  pinMode(pwm3,OUTPUT); // pwm 3
  pinMode(dir3,OUTPUT); // dir 3
  pinMode(pwm4,OUTPUT); // pwm 4
  pinMode(dir4,OUTPUT); // dir 4

  pinMode(5,OUTPUT);//ground
  pinMode(4,OUTPUT);

  pinMode(42,OUTPUT);//ground
  pinMode(40,OUTPUT);
  
  pinMode(13,OUTPUT);//ground

  pinMode(rotDir,OUTPUT);
  pinMode(rotPWM,OUTPUT);

  pinMode(EncoderPin[0],INPUT);
  pinMode(EncoderPin[1],INPUT);
  Stop();

  attachInterrupt(digitalPinToInterrupt(EncoderPin[0]),updateEncoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderPin[1]),updateEncoder,CHANGE);
  myservo.attach(13);
  myservo.write(90);
  
}

void loop() {
   ps2x.read_gamepad();
   if(L1%2)
  {
    volt=15;
  }
  else
  {
    volt=3.2;
  }
  motorVal = map(volt,0,TotVlot,0,255);

  if(R1%2)
  {
    rotVolt = 9;
    //motS=125;
  }
  else
  {
    rotVolt = 5.17;
//    motS=110;
  }
  motS = map(rotVolt,0,TotVlot,0,255);
  if(ps2x.ButtonPressed(PSB_L1))  //L1 pressed
  {
    L1++;
  }
  if(ps2x.ButtonPressed(PSB_R1))  //R1
  {
   R1++;
  }
  

  if(ps2x.ButtonPressed(PSB_CIRCLE))  //Circle pressed
  {
    if(ps2x.ButtonPressed(PSB_TRIANGLE))
    {
       //
       if(Servcnt%2)
        {
         open();
         delay(1000); 
        }
        else
        {
          close();
          delay(1000);
        }
        Servcnt++;
//      Serial.println("Triangle");
    }
    else
    {
      //
      Stop();
//      Serial.println("Circle");
    }
    
  }
  else if(ps2x.ButtonPressed(PSB_SQUARE))  //Triangle pressed
  {
    //
    if(ps2x.ButtonPressed(PSB_CROSS))
    {
      if(fcnt%2)
        {
         rotFront(); 
        }
        else
        {
          rotateStop();
        }
        fcnt++;
//      Serial.println("Cross");
    }
    else
    {
      if(bcnt%2)
        {
         rotBack(); 
        }
        else
        {
          rotateStop();
        }
        bcnt++;
//      Serial.println("Square");
    }
  }
  else if(ps2x.ButtonPressed(PSB_PAD_UP))  //Triangle pressed
  {
    //
//    Serial.println("Up");
    forward();
    dir=0;
  }
  else if(ps2x.ButtonPressed(PSB_PAD_DOWN))  //Triangle pressed
  {
    //
    if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
    {
//       Serial.println("Right");
      if(!dir)
        right();
      else
        left();
    }
    else
    {
//      Serial.println("Down");
      back();
      dir=1;
    }
  }
  else if(ps2x.ButtonPressed(PSB_PAD_LEFT))  //Triangle pressed
  {
    //
//    Serial.println("Left");
    if(!dir)
      left();
    else
      right();
  }
}

void forward()
{
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);
  digitalWrite(dir3,HIGH);
  digitalWrite(dir4,HIGH);
  analogWrite(pwm1,motorVal);
  analogWrite(pwm2,motorVal);
  analogWrite(pwm3,motorVal);
  analogWrite(pwm4,motorVal);
  delay(turn_delay);
}


void back()
{
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);
  digitalWrite(dir3,LOW);
  digitalWrite(dir4,LOW);
  analogWrite(pwm1,motorVal);
  analogWrite(pwm2,motorVal);
  analogWrite(pwm3,motorVal);
  analogWrite(pwm4,motorVal); 
  delay(turn_delay); 
}


void left()
{
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,HIGH);
  digitalWrite(dir3,HIGH);
  digitalWrite(dir4,LOW);
  analogWrite(pwm1,motorVal);
  analogWrite(pwm2,motorVal);
  analogWrite(pwm3,motorVal);
  analogWrite(pwm4,motorVal);
delay(turn_delay);
}

void right()
{
  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);
  digitalWrite(dir3,LOW);
  digitalWrite(dir4,HIGH);
  analogWrite(pwm1,motorVal);
  analogWrite(pwm2,motorVal);
  analogWrite(pwm3,motorVal);
  analogWrite(pwm4,motorVal);
delay(turn_delay);
}

void Stop(){
    digitalWrite(dir1,LOW);
    digitalWrite(dir2,LOW);
    digitalWrite(dir3,LOW);
    digitalWrite(dir4,LOW);
    analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    analogWrite(pwm3,0);
    analogWrite(pwm4,0);  
}

void updateEncoder(){
  int MSB = digitalRead(EncoderPin[0]); //MSB = most significant bit
  int LSB = digitalRead(EncoderPin[1]); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enVal++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enVal--;
 
  lastEncoded = encoded; //store this value for next time
}


void open() {
  myservo.write(0);
  delay(DT);
  myservo.write(90);
}

void close() {
  myservo.write(180);
  delay(DT+25);
  myservo.write(90);
}

void rotFront()
{
  analogWrite(rotDir,motS);
  analogWrite(rotPWM,motS);
  digitalWrite(40,LOW);
  digitalWrite(42,HIGH);
}

void rotBack()
{
  analogWrite(rotDir,motS);
  analogWrite(rotPWM,motS);
  digitalWrite(40,HIGH);
  digitalWrite(42,LOW);
}

void rotateStop()
{
  analogWrite(rotDir,0);
  analogWrite(rotPWM,0);
}

