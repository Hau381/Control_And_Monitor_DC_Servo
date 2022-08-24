#include <Arduino.h>
#include <HardwareSerial.h>
#include <HardwareTimer.h>
#include <ahau.h>
#include <string.h>
HardwareSerial Serial_Mon(PA10, PA9);
HardwareTimer timer3(TIM3);
HardwareTimer timer1(TIM1);
#define LED PC13
#define IN1 PB8
#define IN2 PB9
#define EA PA1
#define ENC PA3
String buff_serial = "";
float Kp =500;
float Ki =600;
float Kd = 0;
float tocdodat =0;
int chieu ;
Motor MOTORVIP(IN1, IN2, EA);
float tocdohientai=0;
int k;
int A =0 ;
float e =0;
double u=0 ;
int T,TD;
int tocdohientai1=0,tocdohientai2=0,k1=0,k2=0;
PIDk PIDrr (Kp,Ki,Kd,0.01);
Theta thetavip;
STR strvip;
void time_overflow()
{ 
   k=PIDrr.pidk(e);
   
  if(k>65535)
  {
    k=65535;
  }
  if(k<0)
  {
    k=0;
  }
  
}
void time4_overflow()
{
  String data = String(int(tocdohientai));//+','+String(int(tocdodat));
  Serial_Mon.println(data);
  if (A==0) MOTORVIP.Stop();
    else MOTORVIP.Run(chieu,k);
}
void dieu_khien()
    {
        buff_serial =Serial_Mon.readString();
        if(buff_serial == "!"){
           chieu = THUAN;
        if (A == 1) MOTORVIP.Stop();
        A=1;
        }

        else if(buff_serial == "@"){
         chieu = NGUOC;
         if (A == 1) MOTORVIP.Stop();
         A=1; 
        }

        else if(buff_serial == "#"){
           A =0 ;
           tocdohientai = 3000000 / T;
          tocdodat=0;
          PIDrr.reset();
          
        }

        else {
          int len = buff_serial.length();
          
          tocdodat =0;
         for(int i =0;i<len;i=i+1){
             tocdodat += (int(buff_serial[i])-48)*pow(10,len-i-1);
         }   
         e = tocdodat - tocdohientai;  
        }
       
    
        buff_serial = "";
      }

void setup()
{
    Serial_Mon.begin(9600);//115200
    pinMode(ENC, INPUT);
    pinMode(LED, OUTPUT);
    analogWriteResolution(16);
    analogWriteFrequency(1000);
    timer3.pause();
    timer3.setOverflow(100, HERTZ_FORMAT);
    timer3.attachInterrupt(time_overflow);
    timer3.resume();
    timer1.pause();
    timer1.setOverflow(1, HERTZ_FORMAT);
    timer1.attachInterrupt(time4_overflow);
    timer1.resume();
}
void loop()
{  
     if (A==0) MOTORVIP.Stop();
    else MOTORVIP.Run(chieu,k);
    TD=200000;
    int H = pulseIn(ENC, HIGH,TD);
    int L = pulseIn(ENC, LOW,TD);
    T = H+L;
    tocdohientai =0;
    tocdohientai = 3000000 / T;
    if(T=0) {tocdohientai =0;}
    e = tocdodat - tocdohientai;
   
    if (Serial_Mon.available()) dieu_khien();
}