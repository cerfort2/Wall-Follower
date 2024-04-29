#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/LPF.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/Bump.h"
#include "../inc/Motor.h"
#include "../inc/UART0.h"
#include "../inc/SSD1306.h"
#include "../inc/FFT.h"
#include "../inc/SysTickInts.h"
#include "../inc/UART0.h"
#include "../inc/Reflectance.h"
#include <time.h>


// Select one of the following three output possibilities
// define USENOKIA
#define USEOLED 1
//#define USEUART

#ifdef USENOKIA
// this batch configures for LCD
#include "../inc/Nokia5110.h"
#define Init Nokia5110_Init
#define Clear Nokia5110_Clear
#define SetCursor Nokia5110_SetCursor
#define OutString Nokia5110_OutString
#define OutChar Nokia5110_OutChar
#define OutUDec Nokia5110_OutUDec
#define OutSDec Nokia5110_OutSDec
#endif

#ifdef USEOLED
// this batch configures for OLED

void OLEDinit(void){SSD1306_Init(SSD1306_SWITCHCAPVCC);}
#define Init OLEDinit
#define Clear SSD1306_Clear
#define SetCursor SSD1306_SetCursor
#define OutChar SSD1306_OutChar
#define OutString SSD1306_OutString
#define OutUDec SSD1306_OutUDec
#define OutSDec SSD1306_OutSDec
#endif

#ifdef USEUART
// this batch configures for UART link to PC
#include "../inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}


void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec
#endif

char input;
int goboy = 0;
void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10

    if(input == 'g') {
        UART0_OutString("forward");
        goboy = 1;
    }else if(input == 's') {
        goboy = 0;
        Motor_Stop();
        UART0_OutString("stop");
       }
}

uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
bool pollDistanceSensor(void){
  if(OPT3101_CheckDistanceSensor()){
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}


// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}
// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left){
  return (1247*left)/2048 + 22;
}


#define N 1024
uint32_t Data[N];
#define M 1024
uint16_t Histogram[M];
uint32_t Sum;      // sum of data
uint32_t Sum2;     // sum of (data-average)^2
uint32_t Average;  // average of data = sum/N
uint32_t Variance; // =sum2/(N-1)
uint32_t Sigma;    // standard deviation = sqrt(Variance)


// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=1;  // integral controller gain
int32_t Kp=4;  // proportional controller gain //was 4
int32_t UR, UL;  // PWM duty 0 to 14,998

#define TOOCLOSE 200 //was 200
#define DESIRED 250 //was 250
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm
#define TOOFAR 400 // was 400

#define PWMNOMINAL 5000 // was 2500
#define SWING 2000 //was 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)
void Controller(void){ // runs at 100 Hz
  if(1){
    if((LeftDistance>DESIRED)&&(RightDistance>DESIRED)){
      SetPoint = (LeftDistance+RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }
 //   UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;
    Motor_Forward(UL,UR);

  }
}

void Controller_Right(void){ // runs at 100 Hz

    if(1){
    if((RightDistance>DESIRED)){
      SetPoint = (RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    /*if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }*/

    Error = SetPoint-RightDistance;
    //UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((RightDistance<250) && (CenterDistance <250)){
        UL = 0;
        UR = PWMNOMINAL;
    }
//UL,UR
    Motor_Forward(UL,UR);

  }
}


uint8_t reflectancein,bumpread,crashes;

void main(void){ // wallFollow wall following implementation
  int i = 0;
  uint32_t channel = 1;
  DisableInterrupts();
  Clock_Init48MHz();
  Bump_Init();
  UART0_Init();
  LaunchPad_Init(); // built-in switches and LEDs
  Motor_Stop(); // initialize and stop
  Mode = 0;
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  SysTick_Init(480000,2);
  Reflectance_Init();
  Init();
  Clear();
  OutString("OPT3101");
  SetCursor(0, 1);
  OutString("L=");
  SetCursor(0, 2);
  OutString("C=");
  SetCursor(0, 3);
  OutString("R=");
  SetCursor(0, 4);
  OutString("Wall follow");
  SetCursor(0, 5);
  OutString("SP=");
  SetCursor(0, 6);
  OutString("Er=");
  SetCursor(0, 7);
  OutString("U =");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);
  UR = UL = PWMNOMINAL; //initial power
  Motor_Init();

  EnableInterrupts();

  time_t startt,endt;
  while(1){
    WaitForInterrupt();

    input=UART0_InChar();
    if(input == 'g') {
        UART0_OutString("forward");
        goboy = 1;
    }else if(input == 's') {
        goboy = 0;
        Motor_Stop();
        UART0_OutString("stop");
       }

    if(goboy>0){

    if(Bump_Read()){ // collision
      Mode = 0;
      Motor_Stop();
      crashes++;

      //delay
      Clock_Delay1ms(500);
      Motor_Backward(5000,5000);
      Clock_Delay1ms(1000);

    }

    reflectancein=Reflectance_Read(1000);

    if(reflectancein==0){
        Clock_Delay1ms(500);
        Motor_Stop();
        break;}

    //publish to mqtt

    if(TxChannel <= 2){ // 0,1,2 means new data
      if(TxChannel==0){
        if(Amplitudes[0] > 1000){
          LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
        }else{
          LeftDistance = FilteredDistances[0] = 500;
        }
      }else if(TxChannel==1){
        if(Amplitudes[1] > 1000){
          CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
        }else{
          CenterDistance = FilteredDistances[1] = 500;
        }
      }else {
        if(Amplitudes[2] > 1000){
          RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
        }else{
          RightDistance = FilteredDistances[2] = 500;
        }
      }
      SetCursor(2, TxChannel+1);
      OutUDec(FilteredDistances[TxChannel]); OutChar(','); OutUDec(Amplitudes[TxChannel]);
      TxChannel = 3; // 3 means no data
      channel = (channel+1)%3;
      OPT3101_StartMeasurementChannel(channel);
      i = i + 1;
    }
    Controller_Right();
    if(i >= 100){
      i = 0;
      SetCursor(3, 5);
      OutUDec(SetPoint);
      SetCursor(3, 6);
      OutSDec(Error);
      SetCursor(3, 7);
      OutUDec(UL); OutChar(','); OutUDec(UR);
    }
    }
//    WaitForInterrupt();
  }
}

