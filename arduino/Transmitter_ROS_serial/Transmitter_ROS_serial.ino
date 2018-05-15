 /***************************************************************************
# SDU UAS Center TX firmware 
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# This library is based on the source example Generate_PPM_signal_V0.2 obtained
# from https://code.google.com/archive/p/generate-ppm-signal/downloads
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
This firmware is developed for the v03-2018 version of the SDU UAS Transmitter

Revision
2018-05-01 KJ First released test version
****************************************************************************/
/* parameters */

///#include <ros.h>
//#include <std_msgs/Empty.h>

//ros::NodeHandle nh;
#include <SoftwareSerial.h>

/****************************************************************************/
/* input defines */
#define PIN_LEFT_X A5
#define PIN_LEFT_Y A4
#define PIN_RIGHT_X A3
#define PIN_RIGHT_Y A2
#define PIN_3_POS_SW_LEFT A7
#define PIN_3_POS_SW_RIGHT A6
#define PIN_LEFT_BUTTON 3
#define PIN_RIGHT_BUTTON 4
#define PIN_2_POS_SW_LEFT 7
#define PIN_2_POS_SW_RIGHT 8
#define PIN_POT A1

#define PIN_TX 6
#define PIN_AUDIO 11
#define PIN_BATT_VOLT A0
#define PIN_BUZZER 5
#define PIN_LED_RED 9
#define PIN_LED_GREEN 10

/* ppm defines */
#define ppm_number 8  //set the number of ppm chanels
#define analog_number 8  //set the number of ppm chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define MAX_WORLD_COUNT 5
#define MIN_WORLD_COUNT 2

/****************************************************************************/
/* variables */
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
long ppm[ppm_number];
long analog[analog_number];
short count;
long maxi[4] = {1020, 1017, 1017, 1010};
long mini[4] = {35, 58, 49, 34};
int mid[4] = {519, 546, 520, 524};
boolean led_state;
boolean buzzer_state;
int thrVal = 0; 
int rollVal = 0; 
int pitchVal = 0; 
int yawVal = 0; 
/****************************************************************************/
void setup()
{  
  // setup digital output pins
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(PIN_LED_RED, OUTPUT); 
  pinMode(PIN_LED_GREEN, OUTPUT); 
  pinMode(PIN_BUZZER, OUTPUT); 

  // enable pull-up resistor on digital input pins 
  digitalWrite (PIN_LEFT_BUTTON, HIGH);
  digitalWrite (PIN_RIGHT_BUTTON, HIGH);
  digitalWrite (PIN_2_POS_SW_LEFT, HIGH);
  digitalWrite (PIN_2_POS_SW_RIGHT, HIGH);

  Serial.begin(115200);
  Serial.setTimeout(100);

  //initiallize default ppm values
  for(int i=0; i<ppm_number; i++)
  {
    ppm[i]= 00 ;
  }

  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_AUDIO, OUTPUT);
  digitalWrite(PIN_TX, !onState);  //set the PPM signal pin to the default state (off)
  digitalWrite(PIN_AUDIO, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}
/****************************************************************************/
ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state)
  {  //start pulse
    digitalWrite(PIN_TX, onState);
    digitalWrite(PIN_AUDIO, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else
  {  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(PIN_TX, !onState);
    digitalWrite(PIN_AUDIO, !onState);
    state = true;

    if(cur_chan_numb >= ppm_number)
    {
      digitalWrite(PIN_TX, !onState);
      digitalWrite(PIN_AUDIO, !onState);
       cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
/****************************************************************************/

void throttle(float value) 
{

 float temp = value;  

  //ppm[0] = temp*700/1023 + 1150 -10; // throttle 
 ppm[0]=temp;
  
}

/****************************************************************************/

void aileron(float value)
{

 float temp = value;  
 ppm[1]=temp;
    //ppm[1] =  temp*700/1023 + 1150; // roll (aileron)
 
  
}

/****************************************************************************/

void pitch(float value)
{

 float temp = value;  
 ppm[2]=temp;
 //ppm[2] = temp*700/1023 + 1150; // pitch (elevator)
 
  
}
/****************************************************************************/

void rudder(float value)
{

 float temp = value;
 ppm[3]=temp;  
  //ppm[3] = temp*700/1023 + 1150; // yaw (rudder)

 //Serial.print("r-temp = "); Serial.println(temp); 
}

/****************************************************************************/

void readInputs()
{
  //analog[0] = analogRead(PIN_LEFT_Y);
  //analog[1] = analogRead(PIN_LEFT_X);
  //analog[2] = analogRead(PIN_RIGHT_Y);
  //analog[3] = analogRead(PIN_RIGHT_X);
  analog[4] = analogRead(PIN_3_POS_SW_LEFT);
  analog[5] = analogRead(PIN_3_POS_SW_RIGHT);
  //analog[6] = analogRead(PIN_POT);
  //analog[7] = analogRead(PIN_BATT_VOLT);
  
}

/****************************************************************************/
void correctInputs()
{
 

  for(int i = 0; i<=3; i++){
    if  (analog[i] > maxi[i]){
      maxi[i] = analog[i];}
      
    if (analog[i] < mini[i]){
      mini[i] = analog[i];}
  
    if(analog[i] <= mid[i]){
      analog[i] = ((float)analog[i] - (float)mini[i]) / ((float)mid[i]-(float)mini[i]) * (float)512;}
    else if(analog[i] > mid[i]){
      analog[i] = ((float)analog[i] - (float)mid[i]) / ((float)maxi[i]-(float)mid[i]) * (float)511 + (float)512;}
}
}
/****************************************************************************/
void led() 
{
  if (count % 1000 == 0)
    digitalWrite(PIN_LED_GREEN, HIGH);
  else
    digitalWrite(PIN_LED_GREEN, LOW);
}

/****************************************************************************/
void switches()
{


  // handle left 3-way switch
  if (analog[4] < 300)
    ppm[4] = 1150;
  else if (analog[4] < 700)
    ppm[4] = 1500;
  else
    ppm[4] = 1850;
  
  // handle right 3-way switch
  if (analog[6] < 300)
    ppm[5] = 1150;
  else if (analog[5] < 700)
    ppm[5] = 1500;
  else
    ppm[5] =1850; 

  // unused for now 
  ppm[6] = default_servo_value;
  ppm[7] = default_servo_value;

  
}


/****************************************************************************/
void autoQuadArming(int throttle, int rudder)
{
  
  // handle special case of arming AutoQuad
  if (throttle < 450 && rudder > 1000)
  {
    Serial.println("ARMED!");
    ppm[0] = 1150;
    ppm[3] = 1850;  
  }

}

/****************************************************************************/
//char messageReading()
//{
//  char string[32];
//  char byteRead;
//
//int availableBytes = Serial.available();
//for(int i=0; i<availableBytes; i++)
//{
//   string[i] = Serial.read();
//}
//
/////return string; 
//  
//}

/****************************************************************************/
void serial() // formate TTTTRRRRPPPPYYYY 
{
  
  String input = ""; 
  input = Serial.readStringUntil("\n"); 

  thrVal += (input[0]-48)*1000; 
  thrVal += (input[1]-48)*100; 
  thrVal += (input[2]-48)*10; 
  thrVal += (input[3]-48); 
  rollVal += (input[4]-48)*1000; 
  rollVal += (input[5]-48)*100; 
  rollVal += (input[6]-48)*10; 
  rollVal += (input[7]-48); 
  pitchVal += (input[8]-48)*1000; 
  pitchVal += (input[9]-48)*100; 
  pitchVal += (input[10]-48)*10; 
  pitchVal += (input[11]-48); 
  yawVal += (input[8]-48)*1000; 
  yawVal += (input[9]-48)*100; 
  yawVal += (input[10]-48)*10; 
  yawVal += (input[11]-48); 


  ppm[0] = thrVal; 
  ppm[1] = rollVal; 
  ppm[2] = pitchVal; 
  ppm[3] = yawVal; 

  thrVal = 0; 
  rollVal = 0; 
  pitchVal = 0; 
  yawVal = 0; 

  Serial.print("throttle = "); Serial.println(ppm[0]);

 Serial.print("rudder = "); Serial.println(ppm[1]); 
 Serial.print("aileron = "); Serial.println(ppm[2] ); 
 Serial.print("pitch = "); Serial.println(ppm[3]); 

   
 
    

  
}


/****************************************************************************/
void messageReading()
{

int tempRudder, tempPitch, tempThrottle, tempAileron; 
int first = 0;
int realValueArray[12];

String input;
String tmpString;

input = Serial.readString();
int tmp[input.length()];


//Serial.print("input \n");

//Serial.print(input);



for(int i = 0; i<input.length();i++){

//Serial.print("\n");
//Serial.print(input[i]);
  if(i < 16 ){
    
    tmp[i] = input[i]-48;
    
    if(input[i] == -80){
      tmp[i]=input[i]+80;
    }
    if(input[i] == -77){
      tmp[i]=input[i]+80;
    }
    /*
    Serial.print(" = ASCII = ");
    Serial.print(tmp[0]);
    */
  }
}
/*
Serial.print("Next\n");
Serial.print(tmp[0]);
Serial.print("\n");
Serial.print(tmp[1]);
*/

/*for(int i =0; i<12; i++){
  
  realValueArray[i] = tmp[i]-48;        
    Serial.print("\n RV ");
    Serial.print(realValueArray[i]);  
  

  if(tmp[i] == -128 ){
    Serial.print("\n zero ");
    realValueArray[i] = tmp[i] + 80 + 48;    
  }
  if(tmp[i] == -125){
    Serial.print("\n three ");
    realValueArray[i] = tmp[i] + 80 + 48;    
  }
  
}
*/


/*
Serial.println(commaIndex);
Serial.println(secondCommaIndex);
Serial.println(thirdCommaIndex);

String firstValue = input.substring(0, commaIndex);
String secondValue = input.substring(commaIndex , secondCommaIndex);
String thirdValue = input.substring(secondCommaIndex , thirdCommaIndex); // To the end of the string
String fourthValue = input.substring(thirdCommaIndex); 

Serial.println(firstValue);
Serial.println(secondValue);
Serial.println(thirdValue);
Serial.println(fourthValue);

*/

int rudderVal ; 
int aileronVal ; 
int pitchVal ;
int throttleVal;
throttleVal += tmp[0]*1000;
throttleVal += tmp[1]*100;
throttleVal += tmp[2]*10;
throttleVal += tmp[3];
rudderVal += tmp[4]*1000;
rudderVal += tmp[5]*100;
rudderVal += tmp[6]*10;
rudderVal += tmp[7];
aileronVal += tmp[8]*1000;
aileronVal += tmp[9]*100;
aileronVal += tmp[10]*10;
aileronVal += tmp[11];
pitchVal += tmp[12]*1000;
pitchVal += tmp[13]*100;
pitchVal += tmp[14]*10;
pitchVal += tmp[15];




/*
Serial.print(throttleVal);
Serial.print("\n");
Serial.println(rudderVal);
Serial.print("\n");
Serial.println(aileronVal);
Serial.print("\n");
Serial.println(pitchVal);
*/




/*
// keep values from returning to zero . 
if( throttleVal >= 0  )
{
  //throttle(throttleVal); 
}
else 
{
  throttleVal = 0;
  
}


if( aileronVal >= 0  )
{
  //aileron(aileronVal); 
}
else 
{
  aileronVal = 0;
  
}


if( pitchVal >= 0  )
{
  //pitch(pitchVal); 
}
else 
{
  pitchVal = 0;
  
}



if( rudderVal >= 0  )
{
  //rudder(rudderVal); 
}
else 
{
  rudderVal = 0;
  
}

*/
autoQuadArming(throttleVal, rudderVal);
ppm[0]= throttleVal;
ppm[1]= aileronVal;
ppm[2]= pitchVal;
ppm[3]= rudderVal;


Serial.print("throttle = "); Serial.print(throttleVal);// Serial.print("  "); Serial.println(ppm[0]); 
Serial.print("yaw =      "); Serial.print(rudderVal); //Serial.print("  "); Serial.println(ppm[3]); 
Serial.print("pitch =    "); Serial.print(pitchVal);//Serial.print("  "); Serial.println(ppm[2]); 
Serial.print("roll =     "); Serial.print(aileronVal);//Serial.print("  "); Serial.println(ppm[1]); 

}

  

/****************************************************************************/
void echo()
{

  if (Serial.available()) {
    Serial.write(Serial.read());
    Serial.write("acknowledge");
  }
  if (Serial.available()) {
     Serial.write(Serial.read());
     Serial.write("acknowledge");
  
}
}

/****************************************************************************/
void debug() 
{
  
  // debug: output analog and digital input values to the serial port
  Serial.print (analog[0]);
  Serial.print (" ");
  Serial.print (analog[1]);
  Serial.print (" ");
  Serial.print (analog[2]);
  Serial.print (" ");
  Serial.print (analog[3]);
  Serial.print (" ");
  Serial.print (analog[4]);
  Serial.print (" ");
  Serial.print (analog[5]);
  Serial.print (" ");
  Serial.print (analog[6]);
  Serial.print (" ");
  Serial.print (analog[7]);
  Serial.print (" ");
  Serial.print (digitalRead(PIN_LEFT_BUTTON));
  Serial.print (" ");
  Serial.print (digitalRead(PIN_RIGHT_BUTTON));
  Serial.print (" ");
  Serial.print (digitalRead(PIN_2_POS_SW_LEFT));
  Serial.print (" ");
  Serial.println (digitalRead(PIN_2_POS_SW_RIGHT)); 
}


/****************************************************************************/
void loop()
{
  //put main code here
  static int val = 1;
  count ++;

  // update LED

  led();

  // read analog input
   
  readInputs(); 
  //correctInputs();

  // map to ppm output uncomment to use joystick 


  switches(); 
  //echo(); 
  //comment to use joystick 
  //messageReading();

  if(Serial.available() > 0)
  {
     //serial();
     messageReading();
     
  }
  /*
  Serial.print(analog[0]); Serial.print("  ");
  Serial.print(analog[3]); Serial.print("  ");
  Serial.print(analog[2]); Serial.print("  ");
  Serial.println(analog[1]); 
  throttle(analog[0]); 
  aileron(analog[3]); 
  pitch(analog[2]); 
  rudder(analog[1]);
  /**/

  
  

    
  //delay(10);
}
/****************************************************************************/

// 770:500:500:370

