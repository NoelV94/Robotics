#include <Servo.h>

// ============== left =======================
Servo Lhu;  // create servo object to control a servo
Servo Lhd;  // create servo object to control a servo
Servo Lk;  // create servo object to control a servo
Servo Lau;  // create servo object to control a servo
Servo Lad;  // create servo object to control a servo

// ============== right ======================
Servo Rhu;  // create servo object to control a servo
Servo Rhd;  // create servo object to control a servo
Servo Rk;  // create servo object to control a servo
Servo Rau;  // create servo object to control a servo
Servo Rad;  // create servo object to control a servo

//===============left===============
int pos1 = 90;    // variable to store the servo position
int pos2 = 90;
int pos3 = 90;
int pos4 = 90;
int pos5 = 90;

//========= Right ==============
int pos6 = 90;
int pos7 = 90;
int pos8 = 90;
int pos9 = 90;
int pos10 = 90;

//============= Counts =============
const int pin1 = 23;
const int pin2 = 27;
const int pin3 = 29;
const int pin4 = 31;
int buttonState1;
int buttonState2;
int buttonState3;
int buttonState4;
int count = 0;

int angle(int pos,int dest, Servo &object)
{
    if (pos>dest)
    {
      for(count = pos; count >= dest ; count -=1)
      {
        pos = count;
        object.write(count);
        delay(150);
        
        }
      }
    if (pos<dest)
    {
      for(count = pos; count <= dest ; count +=1)
      {
        pos = count;
        object.write(count);
        delay(150);
        
        }
      }
     return pos;
  }


void setup() {
  Lhu.attach(2);  // attaches the servo on pin 9 to the servo object
  Lhd.attach(4);
  Lk.attach(5);
  Lau.attach(6);
  Lad.attach(7);
  Rhu.attach(8);  // attaches the servo on pin 9 to the servo object
  Rhd.attach(9);
  Rk.attach(10);
  Rau.attach(11);
  Rad.attach(12);

  // Buttons

  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  pinMode(pin3, INPUT_PULLUP);
  pinMode(pin4, INPUT_PULLUP);
}




void loop() 
{

    
    Lhu.write(pos1);
    Lhd.write(pos2);
    Lk.write(pos3);
    Lau.write(pos4);
    Lad.write(pos5);
    Rhu.write(pos6);
    Rhd.write(pos7);
    Rk.write(pos8);
    Rau.write(pos9);
    Rad.write(pos10);
    delay(1500);
    
    pos1 = 90;    
    pos2 = 160;
    pos3 = 40;
    pos4 = 80;
    pos5 = 90;
    
    //========= Right ==============
    pos6 = 90;
    pos7 = 20;
    pos8 = 175;
    pos9 = 110;
    pos10 = 90;
    Lhu.write(pos1);
    Lhd.write(pos2);
    Lk.write(pos3);
    Lau.write(pos4);
    Lad.write(pos5);
    Rhu.write(pos6);
    Rhd.write(pos7);
    Rk.write(pos8);
    Rau.write(pos9);
    Rad.write(pos10);
    delay(1500);

    
while(1)
{
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  pinMode(pin3, INPUT_PULLUP);
  pinMode(pin4, INPUT_PULLUP);
  buttonState1 = digitalRead(pin1);
  buttonState2 = digitalRead(pin2);
  buttonState3 = digitalRead(pin3);
  buttonState4 = digitalRead(pin4);

// Right
  if(buttonState1 == LOW && buttonState2 == HIGH && buttonState3 == LOW && buttonState4 == HIGH)
  {

    while(1)
    {
      
    delay(1000);


//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================


for(count = 0; count <= 10; count +=1)
    {
            
      pos5 = pos5 + 2;
      
      Lad.write(pos5);
      delay(150);

      }
      delay(1000);
 //=============================================================================================================================   

 //=============================================================================================================================   

    for(count = 0; count <=8 ;count+=1)    {
      pos7 = pos7 + 1;
      pos3 = pos3 + 2;
      pos4 = pos4 + 1;
      pos8 = pos8 + 2;
      pos9 = pos9 + 1;
      Rhd.write(pos7);
      Lk.write(pos3);
      Rk.write(pos8);
      Lau.write(pos4);
      Rau.write(pos9);
      delay(150);
    }

delay(1500);
 //===========================================================================================================================
 
    // Right balance shift reverse
    for(count = 0; count <= 5; count +=1)
      {
      pos5 = pos5 - 2;
      Lad.write(pos5);
      delay(150);
      }
      delay(2000);
//===========================================================================================================================
for(count = 0; count <=30 ;count+=1)    {
  pos2 = pos2 - 1;
  pos7 = pos7 + 1;
  Lhd.write(pos2);
    Rhd.write(pos7);
    delay(150);
}
////============================================================================================================================
    // Load shift to right
    for(count = 0; count <=10 ; count +=1)
    {
      pos8 = pos8 - 2;
      Rk.write(pos8);
      delay(150);
      }
//===========================================================================================================================
    // Preadjustment before left movement
    for(count = 0; count <= 10; count +=1)
    {
      pos6 = pos6-2;
      pos1  = pos1 -2;
      pos5 = pos5 + 1;
      pos10 = pos10 + 1;
      Rad.write(pos10);
      Lad.write(pos5);
      Rhu.write(pos6);
      Lhu.write(pos1);
      delay(150);
      }
    delay(1000);

    for(count = 0; count <= 10; count +=1)
    {
      pos2= pos2 + 1;
      pos7 = pos7 - 1;
      Rhd.write(pos7);
      Lhd.write(pos2);
      delay(150);
      }    

//===========================================================================================================================
// left move forward
    for(count = 0; count <= 10; count +=1)
    {
      pos7 = pos7 - 1;

      Lhd.write(pos2);
      delay(150);
      }
    delay(1000);
// 
    for(count = 0; count <= 10; count +=1)
    { 
      pos6 = pos6+2;
      pos1  = pos1 +2;
      Rhu.write(pos6);
      Lhu.write(pos1);
      delay(150);
    }
    delay(1000);
    for(count = 0; count <= 10; count +=1)
    {
            
      pos10 = pos10 -2;
      
      Rad.write(pos10);
      delay(150);

      }
      delay(1000);

    for(count = 0; count <= 20; count +=1)
    {
      pos2 = pos2 +1;
      Lhd.write(pos2);
      delay(150);
    }
    
//===========================================================================================================================
    

        
    pos6 = angle(pos6,90,Rhu);
    pos1 = angle(pos1,90,Lhu);
    pos8 = angle(pos8,175,Rk);
    pos3 = angle(pos3,40,Lk);
    pos4 = angle(pos4,80,Lau);
    pos9 = angle(pos9,110,Rau);
    pos7 = angle(pos7,20,Rhd);
    pos2 =  angle(pos2,160,Lhd);    
    pos10 = angle(pos10,90,Rad);
    pos5 = angle(pos5,90,Lad);

//=================================
//===================================
//===================================

    delay(1000);

    // Right Balance shift
 //=============================================================================================================================   
    for(count = 0; count <= 10; count +=1)
    {
            
      pos10 = pos10 -2;
      
      Rad.write(pos10);
      delay(150);

      }
      delay(1000);
 //=============================================================================================================================   

 //=============================================================================================================================   

    for(count = 0; count <=8 ;count+=1)    {
      pos2 = pos2 - 1;
      pos3 = pos3 - 2;
      pos4 = pos4 - 1;
      pos8 = pos8 - 2;
      pos9 = pos9 - 1;
      Lhd.write(pos2);
      Lk.write(pos3);
      Rk.write(pos8);
      Lau.write(pos4);
      Rau.write(pos9);
      delay(150);
    }

delay(1500);
 //===========================================================================================================================
 
    // Right balance shift reverse
    for(count = 0; count <= 10; count +=1)
      {
      pos10 = pos10 + 2;
      Rad.write(pos10);
      delay(150);
      }
      delay(2000);
//===========================================================================================================================
for(count = 0; count <=20 ;count+=1)    {
  pos2 = pos2 - 1;
  pos7 = pos7 + 1;
  Lhd.write(pos2);
    Rhd.write(pos7);
    delay(150);
}
////============================================================================================================================
    // Load shift to right
    for(count = 0; count <=10 ; count +=1)
    {
      pos3 = pos3 + 2;
      Lk.write(pos3);
      delay(150);
      }
//===========================================================================================================================
    // Preadjustment before left movement
    for(count = 0; count <= 10; count +=1)
    {
      pos6 = pos6-2;
      pos1  = pos1 -2;
      pos5 = pos5 +2;
      pos10 = pos10 + 2;
      Rad.write(pos10);
      Lad.write(pos5);
      Rhu.write(pos6);
      Lhu.write(pos1);
      delay(150);
      }
    delay(1000);

    for(count = 0; count <= 20; count +=1)
    {
      pos2= pos2 + 1;
      pos7 = pos7 - 1;
      Rhd.write(pos7);
      Lhd.write(pos2);
      delay(150);
      }    

//===========================================================================================================================
// left move forward
    for(count = 0; count <= 40; count +=1)
    {
      pos7 = pos7 + 1;
//      pos3 = pos3 + 1;
//      pos4 = pos4 + 1;
//      Lk.write(pos3);
//      Lau.write(pos4);
      Rhd.write(pos7);
      delay(150);
      }
    delay(1000);
    
//===========================================================================================================================
    // Back to initial position
    pos10 = angle(pos10,90,Rad);
    pos5 = angle(pos5,90,Lad);
    
////===========================================================================================================================
//    // Left balance shift reverse
//    for(count = 0; count <= 10; count +=1)
//      {
//      pos5 = pos5 + 1;
//      pos10 = pos10 - 1;
//      Rad.write(pos10);
//      Lad.write(pos5);
//      delay(150);
//      }
//      delay(2000);
////===========================================================================================================================
//    

        
    pos6 = angle(pos6,90,Rhu);
    pos1 = angle(pos1,90,Lhu);
    pos8 = angle(pos8,175,Rk);
    pos3 = angle(pos3,40,Lk);
    pos4 = angle(pos4,80,Lau);
    pos9 = angle(pos9,110,Rau);
    pos7 = angle(pos7,40,Rhd);
    pos2 =  angle(pos2,140,Lhd);    
    pos10 = angle(pos10,90,Rad);
    pos5 = angle(pos5,90,Lad);
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
  }
  }
}
}        
