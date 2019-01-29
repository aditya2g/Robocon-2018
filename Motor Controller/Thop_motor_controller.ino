#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 10); // RX, TX
const int dir  = 8;
const int pwm  = 9;

const int volt = 12;
int value1 = map(volt-4,0,24,0,255);
int value2 = map(volt-2,0,24,0,255);
int value3 = map(volt,0,24,0,255);
void setup() {
  pinMode(dir,OUTPUT);
  pinMode(pwm,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  mySerial.begin(9600);
}

void loop() {
  if (mySerial.available())             // Reads data from bluetooth 
  {
    int y = mySerial.read();
    //Serial.println(y);
    if (y == 1)                          // If 1 ball transferrred
    {
      //Serial.println("success");
      Serial.println(2);
      // Sends to pi ball transferred
    }
    if (y == 2)                          //Ball is thrown
    {
       Stop();                         
       Serial.println(3);                 // Sends to pi ball is thrown
    }
  }
  if (Serial.available())               // Reads data from pi
  {
    char x = Serial.read();
   // Serial.println("4pass");
    if (x=='4')                         // Position Reached Start the motor
    { 
    //  Serial.println("4pass");
      if (!mySerial.available()) 
      {
       // Serial.println("Sending ble data");
        mySerial.write(1);              // Sends to bluetooth to start counting
        Rotate1();
      }
    }
    if (x=='5')                         // Position Reached Start the motor
    { 
    //  Serial.println("4pass");
      if (!mySerial.available())
      {
       // Serial.println("Sending ble data");
        mySerial.write(1);              // Sends to bluetooth to start counting
        Rotate2();
      }
    }
    if (x=='6')                         // Position Reached Start the motor
    { 
    //  Serial.println("4pass");
      if (!mySerial.available()) 
      {
       // Serial.println("Sending ble data");
        mySerial.write(1);              // Sends to bluetooth to start counting
        Rotate3();
      }
    }
    if (x=='8')
    {
      reset_init();
      Serial.println(9);
    }
  }
}
void Stop()
{
  digitalWrite (dir,HIGH);
  analogWrite (pwm,0);
}
void Rotate1()
{
  digitalWrite (dir,LOW);
  analogWrite (pwm,value1);
}

void Rotate2()
{
  digitalWrite (dir,LOW);
  analogWrite (pwm,value2);
}

void Rotate3()
{
  digitalWrite (dir,LOW);
  analogWrite (pwm,value3);
}

void reset_init()
{
  int count = 0;
  int flag = 0;
  int flag1 = 0;
  int val=0;
  
 while(!flag1)
 {  
    digitalWrite (dir,LOW);
    analogWrite (pwm,18);
    
    val = analogRead(A2);
   //Serial.println(val);
    if(!flag)
    {
      if (val < 25)
      {
        count++;
//        Serial.println(count);
        flag=1;
      }
  }
    else if(flag==1)
    {
      if (val >= 600)
      {
        flag=0;
      } 
    }
    if(count >= 2)
    {
      Stop();
      flag1=1;
    } 
  }
  
}


