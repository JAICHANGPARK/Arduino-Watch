/**
 *  Created by Dreamwalker
 *  Date : 2017. 02. 03
 *  
 *  Arduino Smart Watch Module
 *  Testing Module 
 *  ARDUINO : ARDUINO MICRO.
 *  DISPLAY : OLED SSD1306
 *  IMU Sensor : MPU- 6050  [I2C ADDR : 0x69 - AD0 HIGH(5V) ]  RTC 모듈과 주소가 중복되어 MPU6050 주소를 변경했다.
 *  RTC Module : DS1307 [I2C ADDR : 0x68]
 *  Capacitance Sensor ( = Touch Sensor ) : Digital Input Pin Number 4
 *  Vivration Motor : Digital Output Pin Number 5
 *  
 *  @Revision 
 *  
 *  2017.02.05
 *  
 *    1. MPU-6050 filter added. 
 *    2. GYRO DISPLAY FIXXXXED.
 *    3. Heart Rate Sensor Code added. 
 *    4. 상보 필터 적용. - 필터는 여려 경우를 해보면서 상황에 적합한 필터를 채택하길 바람.
 *  
 *  2017.02.06
 *  
 *    1. PID CODE ADDED.
 *    2. Fixed Display of DayofWeek :) 
 *    3. sleep mode added but it's unstable.
 *       pls comment sleepMode() function now.
 *       
 *    4. 사용 메모리 초과 .. 메모리 최적화 필요. 
 *      스케치는 프로그램 저장 공간 29,870 바이트(104%)를 사용. 최대 28,672 바이트.
        전역 변수는 동적 메모리 1,062바이트(41%)를 사용, 1,498바이트의 지역변수가 남음.  최대는 2,560 바이트. 
 *   
 *   2017.03.04 
 *   
 *   1. library 작성 
 */

#include "watch.h"

int pin = 13;
volatile int state = LOW;
short cnt = 0;
bool reading, previous = false;

Watch watch;

void setup() 
{
  
  pinMode(SW_INPUT,INPUT);
  pinMode(VIB_MOTOR, OUTPUT);
  pinMode(pin,OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  watch.begin();
  
  //DS3231 seconds, minutes, hours, day, date, month, year
  //watch.setDS3231time(00,51,18,2,06,02,17);

} 

void loop() 
{  
  watch.loopCycle(); 
  digitalWrite(pin, state);
  reading = digitalRead(SW_INPUT);
 
 if(reading == true && previous == false){
  
  cnt++;
  
  digitalWrite(VIB_MOTOR, HIGH);
  delay(150);
  digitalWrite(VIB_MOTOR, LOW);
  
  }else{
  cnt = cnt;
  }
 
 switch(cnt){
  case 1:
    watch.lcdTime();
  break;
  case 2:
    watch.lcdDate();
  break;
  case 3:
    //watch.drawAcc();
  break;
  case 4:
    //watch.drawGyro();
  break;
  case 5:
    //watch.drawFilterAngle();
  break;
  case 6:
    //watch.drawPIDYPR();  
  break;
  case 7:
    //watch.drawBMP280();
  break;
  case 8:
    cnt = 0;
    //sleepMode();
  break;
  default:
  break;
 }
 delay(150);
}  

