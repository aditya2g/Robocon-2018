#include <SoftwareSerial.h>
SoftwareSerial bt (6,7);               //Rx and Tx pins 
const int ircount= A1;
const int irclose= A0;
const int dir1 = 8;
const int pwm1 = 9;
int count = 0, flag=0;
int val=0;
int flag1=0;
int value2 = 0;
const int k = 50;
int flag2 = 0;
void setup() {
bt.begin( 9600 );   
Serial.begin( 9600 );   
pinMode(dir1,OUTPUT);
pinMode(pwm1,OUTPUT); 
pinMode(13,OUTPUT);
pinMode(12,OUTPUT);
pinMode(4,OUTPUT);
pinMode(5,OUTPUT);
  
pinMode(ircount,INPUT);
pinMode(irclose,INPUT);
while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

}
 
void loop() {

  digitalWrite(12,HIGH);
  digitalWrite(13,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);

  Release(); //modified
  
  value2 = analogRead(irclose);
  
  if (!flag2)
  {
    
    if (value2 <=k)
    {
      delay (1500);
      hold();
     
      bt.write(1);            // Arduino sends that ball transferred.
      flag2 = 1;
    }
  }
  if ( bt.available() > 0 ) {
   // Serial.println("reading");
    int dat = bt.read();    // thop arduino sends 1 through bluetooth when to start throw mechanism
    //Serial.println(dat);
    if (dat == 1) 
    {
      int x = Throw();
      if (x == 1)
      {
        bt.write(2);  // When ball thrown arduino sends 2 to thop to stop motor.
        delay(1000); //modified 
        flag2 = 0;
      }
    }
  }  
}

int Throw()
{
  //Serial.println("throwing");
 count = 0;flag = 0;flag1 = 0;val=0;
 while(!flag1)
 {
    
    val = analogRead(ircount);
    if(!flag)
    {
      if (val <=40 )
      {
        count++;
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
    if(count >= 10)
    {
      Release();
      flag1=1;
    } 
  }
  return 1;
}

void hold()
{
  Serial.println("HOLD");
  digitalWrite(dir1,HIGH);
  digitalWrite(pwm1,LOW);
}
void Release()
{
  //Serial.println("Left");
  digitalWrite(dir1,HIGH);
  digitalWrite(pwm1,HIGH);
}
