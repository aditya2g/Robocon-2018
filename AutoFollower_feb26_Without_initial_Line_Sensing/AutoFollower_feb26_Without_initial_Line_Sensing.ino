// Rewire sensor 3
// Encoders on sensor 3 and sensor 2 should be wired atleast.
// Change Encoder Wiring
// Rev mapping

#include <PinChangeInt.h>

//Initialization
int sensor[4][5] = {0};     //initialization for LSS05 to zero
int initial_motor_speed = 45;        //can change the initial speed that you prefer
int maxSpeed = 55;                 //if the motor is too fast, decrease the max speed and vice versa
float Kp = -2.8;//3.5       //will be change using potentiometer
float Kd = 0;       //will be change using potentiometer
float P=0, D=0, PD_value=0;
float previous_error=0;
int flag = 0;
int initial=0;
int revEnc=30;
int n=500;
//Pin declaration for SHIELD-2AMOTOR  3

int i=0,j=0;

int motdir[4]={11,4,6,9};
int motpwm[4]={10,5,7,8};

int sensPin[4][5];
int EncoderPin[4][2];

volatile long long int lastEncoded[4];
volatile long long encoderValue[4];

void initialize(){
  for(int i=0;i<5;i++)
  {
      sensPin[0][i]=43+(2*i);//up
  }    
  
  for(int i=0;i<5;i++)
  {
      sensPin[1][i]=52-(2*i);//right
  }
  
  for(int i=0;i<5;i++)
  {
      sensPin[2][i]=36-(2*i);////down
  }
  
  for(int i=0;i<5;i++)
  {
      sensPin[3][i]=39-(2*i);//left
  }
  

  EncoderPin[0][0]=14;  EncoderPin[0][1]=15;
  EncoderPin[1][0]=2;  EncoderPin[1][1]=3;
  EncoderPin[2][0]=18;  EncoderPin[2][1]=19;
  EncoderPin[3][0]=20;  EncoderPin[3][1]=21;

  for(int i=0;i<4;i++)
  {
      lastEncoded[i]=0;
      encoderValue[i]=0;
  }
  
}
int read_sensor_values(int sensNum);
int calculate_pid(int i);
void motor_control(int orient,int PD);
void Rev_control(int orient,long long int enc,int matchEn);

int pos;
int flagdir=0;
int PD=-1;
int flagarr[4]={0,0,0,0};
int Rev[5]={0,3,4,1,2};// Rev[prevOrrientation] = New orientation 
int OrrMap[5]={0,0,3,2,1}; 
int SenOrrMap[4]={1,4,3,2};   //Sen->Orr

int val[4]={0};
int OnLineVal=-1;

void setup()
{
  initialize();
  Serial.begin(9600);       //Enable Serial Communications

  for(int i=0;i<4;i++)
  {
    pinMode(motdir[i],OUTPUT);
    pinMode(motpwm[i],OUTPUT);


///encoder
    pinMode(EncoderPin[i][0], INPUT); 
    pinMode(EncoderPin[i][1], INPUT);
 
    digitalWrite(EncoderPin[i][0], HIGH); //turn pullup resistor on
    digitalWrite(EncoderPin[i][1], HIGH); //turn pullup resistor on
  }
  
  for(int i=0;i<5;i++)
    {
      pinMode(sensPin[0][i],INPUT);
    }
    
    for(int i=0;i<5;i++)
    {
      pinMode(sensPin[1][i],INPUT);
    }
    
    for(int i=0;i<5;i++)
    {
      pinMode(sensPin[2][i],INPUT);
    }
    
    for(int i=0;i<5;i++)
    {
      pinMode(sensPin[3][i],INPUT);
    }

    attachPinChangeInterrupt(EncoderPin[0][0],updateEncoder0, CHANGE);
    attachPinChangeInterrupt(EncoderPin[0][1],updateEncoder0, CHANGE);
    

  //  attachPinChangeInterrupt(EncoderPin[1][0],updateEncoder1, CHANGE);
   // attachPinChangeInterrupt(EncoderPin[1][1],updateEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderPin[1][0]),updateEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderPin[1][1]),updateEncoder2, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(EncoderPin[2][0]),updateEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderPin[2][1]),updateEncoder2, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(EncoderPin[3][0]),updateEncoder3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderPin[3][1]),updateEncoder3, CHANGE);

}

int x;
void loop()
{
  if(Serial.available()){
    x=-1;
    x=Serial.read();
    if(x=='0')
      pos00();
    else if(x=='1')
      pos01();
    else if(x=='2')
      pos02();
    else if(x=='3')
      pos03();
    else if(x=='5')
      pos10();
    else if(x=='6')
      pos20();
    else if(x=='7')
      pos30();
    else
      Serial.println("Apply Pos");
  Serial.flush();
  }
}

void MotorStop(){
      analogWrite(motpwm[0],0);   //Left Motor Speed
      analogWrite(motpwm[1],0);
      analogWrite(motpwm[2],0);
      analogWrite(motpwm[3],0);
}

int read_sensor_values(int sensNum)
{
  sensor[sensNum][0] = digitalRead(sensPin[sensNum][0]);
  sensor[sensNum][1] = digitalRead(sensPin[sensNum][1]);
  sensor[sensNum][2] = digitalRead(sensPin[sensNum][2]);
  sensor[sensNum][3] = digitalRead(sensPin[sensNum][3]);
  sensor[sensNum][4] = digitalRead(sensPin[sensNum][4]);

  flag=0;
  int error=-10;
  if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==1) && (sensor[sensNum][2]==1) && (sensor[sensNum][3]==1) && (sensor[sensNum][4]==0))
  error = 0;
  else if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==0) && (sensor[sensNum][2]==1) && (sensor[sensNum][3]==1) && (sensor[sensNum][4]==0))
  error = -1;
  else if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==0) && (sensor[sensNum][2]==1) && (sensor[sensNum][3]==1) && (sensor[sensNum][4]==1))
  error = -2;
  else if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==0) && (sensor[sensNum][2]==0) && (sensor[sensNum][3]==1) && (sensor[sensNum][4]==1))
  error = -4;
  else if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==0) && (sensor[sensNum][2]==0) && (sensor[sensNum][3]==0) && (sensor[sensNum][4]==1))
  error = -3;
  else if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==1) && (sensor[sensNum][2]==1) && (sensor[sensNum][3]==0) && (sensor[sensNum][4]==0))
  error = 1;
  else if((sensor[sensNum][0]==1) && (sensor[sensNum][1]==1) && (sensor[sensNum][2]==1) && (sensor[sensNum][3]==0) && (sensor[sensNum][4]==0))
  error = 2;
  else if((sensor[sensNum][0]==1) && (sensor[sensNum][1]==1) && (sensor[sensNum][2]==0) && (sensor[sensNum][3]==0) && (sensor[sensNum][4]==0))
  error = 4;
  else if((sensor[sensNum][0]==1) && (sensor[sensNum][1]==0) && (sensor[sensNum][2]==0) && (sensor[sensNum][3]==0) && (sensor[sensNum][4]==0))
  error = 3;
  else if((sensor[sensNum][0]==0) && (sensor[sensNum][1]==0) && (sensor[sensNum][2]==0) && (sensor[sensNum][3]==0) && (sensor[sensNum][4]==0))
  flag=1;
  return error;
}

int calculate_pid(int i)
{  
    P = val[i];
    D = val[i] - previous_error;
    
    PD_value= (Kp*P) + (Kd*D);
    
    previous_error = val[i];
    
    return PD_value;
}

void motor_control(int orient,int PD)
{
  
  int left,right;
  if(orient == 1)
  {
    
  digitalWrite(motdir[0],HIGH);
  digitalWrite(motdir[1],HIGH);
  digitalWrite(motdir[2],LOW);
  digitalWrite(motdir[3],LOW);
    left= 3;
    right= 1;
  }
  if(orient == 2)
  {
    
  digitalWrite(motdir[0],HIGH);
  digitalWrite(motdir[1],HIGH);
  digitalWrite(motdir[2],LOW);
  digitalWrite(motdir[3],LOW);
    left= 2;
    right= 0;
  }
  if(orient == 3)
  {  
  digitalWrite(motdir[0],LOW);
  digitalWrite(motdir[1],LOW);
  digitalWrite(motdir[2],HIGH);
  digitalWrite(motdir[3],HIGH);
    left= 1;
    right= 3;
  }
  if(orient == 4)
  {
  digitalWrite(motdir[0],LOW);
  digitalWrite(motdir[1],LOW);
  digitalWrite(motdir[2],HIGH);
  digitalWrite(motdir[3],HIGH);
    left= 0;
    right= 2;
  }

  int x=OrrMap[orient];
    if(flagarr[x] == 0)
    {
      // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed + PD;
    int right_motor_speed = initial_motor_speed - PD;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,0,maxSpeed);
    constrain(right_motor_speed,0,maxSpeed);

    //right_motor_speed = right_motor_speed + 5; //this line is needed if your motor didn't have same speed
    
    //open the Serial Monitor to see the speed of each motor 
    
    analogWrite(motpwm[left],left_motor_speed);   //Left Motor Speed
    analogWrite(motpwm[right],right_motor_speed);  //Right Motor Speed
    }
    else
    {
       analogWrite(motpwm[left],0);   //Left Motor Speed
       analogWrite(motpwm[right],0);  //Right Motor Speed
      
      }
}

void Rev_control(int orient,long long int enc,int matchEn)
{
  
  int left,right;
  int newOrient=Rev[orient];
  while(encoderValue[matchEn]>enc+revEnc)
  {
  if(newOrient == 1)
  {
    
  digitalWrite(motdir[0],HIGH);
  digitalWrite(motdir[1],HIGH);
  digitalWrite(motdir[2],LOW);
  digitalWrite(motdir[3],LOW);
    left= 3;
    right= 1;
  }
  if(newOrient == 2)
  {
    
  digitalWrite(motdir[0],HIGH);
  digitalWrite(motdir[1],HIGH);
  digitalWrite(motdir[2],LOW);
  digitalWrite(motdir[3],LOW);
    left= 2;
    right= 0;
  }
  if(newOrient == 3)
  {  
  digitalWrite(motdir[0],LOW);
  digitalWrite(motdir[1],LOW);
  digitalWrite(motdir[2],HIGH);
  digitalWrite(motdir[3],HIGH);
    left= 1;
    right= 3;
  }
  if(newOrient == 4)
  {
  digitalWrite(motdir[0],LOW);
  digitalWrite(motdir[1],LOW);
  digitalWrite(motdir[2],HIGH);
  digitalWrite(motdir[3],HIGH);
    left= 0;
    right= 2;
  }
  
  int Speed=25;
  analogWrite(motpwm[left],Speed);   //Left Motor Speed
  analogWrite(motpwm[right],Speed);  //Right Motor Speed
  }
}

int ifJunction(int sensNum){
  if((sensor[sensNum][0]==1) && (sensor[sensNum][1]==1) && (sensor[sensNum][2]==1) && (sensor[sensNum][3]==1) && (sensor[sensNum][4]==1)){
    return 1;
  }
  else{
    return 0;
  }
}

void updateEncoder0(){
  int i=0;
  int MSB = digitalRead(EncoderPin[i][0]); //MSB = most significant bit
  int LSB = digitalRead(EncoderPin[i][1]); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[i] << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[i] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[i] --;
 
  lastEncoded[i] = encoded; //store this value for next time
}

void updateEncoder1(){
  int i=1;
  int MSB = digitalRead(EncoderPin[i][0]); //MSB = most significant bit
  int LSB = digitalRead(EncoderPin[i][1]); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[i] << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[i] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[i] --;
 
  lastEncoded[i] = encoded; //store this value for next time
}

void updateEncoder2(){
  int i=2;
  int MSB = digitalRead(EncoderPin[i][0]); //MSB = most significant bit
  int LSB = digitalRead(EncoderPin[i][1]); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[i] << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[i] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[i] --;
 
  lastEncoded[i] = encoded; //store this value for next time
}

void updateEncoder3(){
  int i=3;
  int MSB = digitalRead(EncoderPin[i][0]); //MSB = most significant bit
  int LSB = digitalRead(EncoderPin[i][1]); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[i] << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue[i] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue[i] --;
 
  lastEncoded[i] = encoded; //store this value for next time
}



int side(int SensNum)
{
  int CheckSens,f=0;
  if(SensNum == 0)
  {
      CheckSens=3;  
  }
  else if(SensNum == 1)
  {
      CheckSens=0;
  }
  else if(SensNum == 2)
  {
      CheckSens=1;
  }
  else if(SensNum == 3)
  {
      CheckSens=2;
  }
  else{
    Serial.println("fuck You Vashist");
  }
  if(ifJunction(SensNum))
  {
    OnLineVal=encoderValue[CheckSens];
    previous_error=0;
    MotorStop();
    delay(1000);
    f=1;
    Rev_control(SenOrrMap[SensNum],OnLineVal,CheckSens);// Rev_control(orientationFromWhichToReverse.EncoderValue, EncoderToMatch)
    MotorStop();
    delay(1000);
  }
  else
  {
      val[SensNum]=read_sensor_values(SensNum);
      flagarr[SensNum]=flag;
      PD=calculate_pid(SensNum);
      motor_control(SenOrrMap[SensNum],PD);
  }    
  return f;
}

int Move(int SensNum)
{
  int f=0;
  int CheckSens;
  if(SensNum == 0)
  {
      CheckSens=3;  
  }
  else if(SensNum == 1)
  {
      CheckSens=0;
  }
  else if(SensNum == 2)
  {
      CheckSens=1;
  }
  else if(SensNum == 3)
  {
      CheckSens=2;
  }
  else{
    Serial.println("fuck You Vashist");
  }
 
  if(flagarr[CheckSens]!=1)//if my sensor changes value from 0   MOVING FORWARD
  {
    OnLineVal=encoderValue[CheckSens];
    f=1;
    previous_error=0;
    MotorStop();
    delay(1000);
    
    Rev_control(SenOrrMap[SensNum],OnLineVal,CheckSens);// Rev_control(orientationFromWhichToReverse.EncoderValue, EncoderToMatch) SensOrr maps the sensor we are using just before this to orientation
    MotorStop();
    delay(1000);        
  }
  else
  {
    val[SensNum]=read_sensor_values(SensNum);
    flagarr[SensNum]=flag;
    PD=calculate_pid(SensNum);
    motor_control(SenOrrMap[SensNum],PD);
  }
  return f;
}

int Move_Int(int SensNum)
{
  int f=0;
  int CheckSens;
  if(SensNum == 0)
  {
      CheckSens=3;  
  }
  else if(SensNum == 1)
  {
      CheckSens=0;
  }
  else if(SensNum == 2)
  {
      CheckSens=1;
  }
  else if(SensNum == 3)
  {
      CheckSens=0;
  }
  else{
    Serial.println("fuck You Vashist");
  }
 
  if(flagarr[CheckSens]!=1)//if my sensor changes value from 0   MOVING FORWARD
  {
    OnLineVal=encoderValue[CheckSens];
    f=1;
    previous_error=0;
    MotorStop();
    delay(1000);
    
    Rev_control(SenOrrMap[SensNum],OnLineVal,CheckSens);// Rev_control(orientationFromWhichToReverse.EncoderValue, EncoderToMatch) SensOrr maps the sensor we are using just before this to orientation
    MotorStop();
    delay(1000);        
  }
  else
  {
    val[SensNum]=read_sensor_values(SensNum);
    flagarr[SensNum]=flag;
    PD=calculate_pid(SensNum);
    motor_control(SenOrrMap[SensNum],PD);
  }
  return f;
}

void TempMove(int N,int SensNum){
  int left,right,orient=SenOrrMap[SensNum];
  if(orient == 1)
  {
    
  digitalWrite(motdir[0],HIGH);
  digitalWrite(motdir[1],HIGH);
  digitalWrite(motdir[2],LOW);
  digitalWrite(motdir[3],LOW);
    left= 3;
    right= 1;
  }
  if(orient == 2)
  {
    
  digitalWrite(motdir[0],HIGH);
  digitalWrite(motdir[1],HIGH);
  digitalWrite(motdir[2],LOW);
  digitalWrite(motdir[3],LOW);
    left= 2;
    right= 0;
  }
  if(orient == 3)
  {  
  digitalWrite(motdir[0],LOW);
  digitalWrite(motdir[1],LOW);
  digitalWrite(motdir[2],HIGH);
  digitalWrite(motdir[3],HIGH);
    left= 1;
    right= 3;
  }
  if(orient == 4)
  {
  digitalWrite(motdir[0],LOW);
  digitalWrite(motdir[1],LOW);
  digitalWrite(motdir[2],HIGH);
  digitalWrite(motdir[3],HIGH);
    left= 0;
    right= 2;
  }
  
  int Speed=40;
  analogWrite(motpwm[left],Speed);   //Left Motor Speed
  analogWrite(motpwm[right],Speed);  //Right Motor Speed
  delay(N);
  MotorStop();
  
}

void pos00(){
    TempMove(n,3);
//  Serial.println("TempMove3");
    MotorStop();
   do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!Move_Int(3));
    previous_error=0;
    //Serial.println("Move0");
    MotorStop();
  do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!Move(0));
    previous_error=0;
    //Serial.println("Move0");
    MotorStop();
}

void pos01(){
  TempMove(n,3);
  //Serial.println("TempMove3");
  for(int i=0;i<2;i++){
    do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!side(3));
    previous_error=0;
    //Serial.println("side3");
  }
  MotorStop();
}
void pos10(){
  TempMove(n,1);
  //Serial.println("TempMove1");
  for(int i=0;i<2;i++){  
    do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!Move(1));
    previous_error=0;
    //Serial.println("Move1");
  }
  MotorStop();
}

void pos02(){
  TempMove(n,0);
  //Serial.println("TempMove0");
  do
  {
    OnLineVal=-1;
    for(int i=0;i<4;i++){
      val[i]=read_sensor_values(i);
      flagarr[i]=flag;
     }
  }while(!Move(0));
  previous_error=0;
  //Serial.println("Move0");
  TempMove(n,3);
  //Serial.println("TempMove3");
  for(int i=0;i<2;i++){
    do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!side(3));
    previous_error=0;
    //Serial.println("side3");
  }
  MotorStop();
}
void pos20(){
    TempMove(n,1);
    //Serial.println("TempMove1");
    for(int i=0;i<2;i++){  
      do
      {
        OnLineVal=-1;
        for(int i=0;i<4;i++){
          val[i]=read_sensor_values(i);
          flagarr[i]=flag;
         }
      }while(!Move(1));
      previous_error=0;
      //Serial.println("Move1");
    }
    MotorStop();
    TempMove(n,2);
    //Serial.println("TempMove2");
    do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!Move(2));
    previous_error=0;
    //Serial.println("Move2");
    MotorStop();
}

void pos03(){
  TempMove(n,0);
  //Serial.println("TempMove0");
  do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!Move(0));
    previous_error=0;
    //Serial.println("Move0");
    TempMove(n,3);
    //Serial.println("TempMove3");
    for(int x=0;x<5;x++){
      do
      {
        OnLineVal=-1;
        for(int i=0;i<4;i++){
          val[i]=read_sensor_values(i);
          flagarr[i]=flag;
         }
      }while(!side(3));
      previous_error=0;
      //Serial.println("Move3");
    }
    MotorStop();
}
void pos30(){
    TempMove(n,1);
    //Serial.println("TempMove1");
    for(int x=0;x<5;x++){
      do
      {
        OnLineVal=-1;
        for(int i=0;i<4;i++){
          val[i]=read_sensor_values(i);
          flagarr[i]=flag;
         }
      }while(!Move(1));
      previous_error=0;
      //Serial.println("Move1");
    }
    TempMove(n,2);
    //Serial.println("TempMove2");
    do
    {
      OnLineVal=-1;
      for(int i=0;i<4;i++){
        val[i]=read_sensor_values(i);
        flagarr[i]=flag;
       }
    }while(!Move(2));
    previous_error=0;
    //Serial.println("Move2");
    MotorStop();
}
