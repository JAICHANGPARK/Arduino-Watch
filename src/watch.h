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
#ifndef ARDUINOWATCH
#define ARDUINOWATCH
 
#include "Arduino.h"
#include "Wire.h"
#include "math.h"

#define SW_INPUT 7
#define VIB_MOTOR 5

#define DS3231_I2C_ADDRESS 0x68
#define MPU6050_ADDRESS 0x69
#define BMP280_ADDRESS  0x77
#define BMP280_CHIPID   0x58

enum{
  
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID              = 0xD0,
      BMP280_REGISTER_VERSION             = 0xD1,
      BMP280_REGISTER_SOFTRESET           = 0xE0,

      BMP280_REGISTER_CAL26               = 0xE1,  

      BMP280_REGISTER_CONTROL             = 0xF4,
      BMP280_REGISTER_CONFIG              = 0xF5,
      BMP280_REGISTER_PRESSUREDATA        = 0xF7,
      BMP280_REGISTER_TEMPDATA            = 0xFA,
};
    
typedef struct
{
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
      
} bmp280_calib_data;

 
class Watch
{
  public:
  
    void begin(void);
    void initDataTime(void);
    void calcDataTime(void);
    void mpu6050Setup(void);
    void mpu6050Read(void);
    void calibrationAccGyro(void);

    void calculateAccYPR();
    void calculateGyroYPR();
    void filterYPR();
    void calculatePID(const float KP, const float KI, const float KD);
    
    void bmp280_Write(byte reg, byte value);
    void readCoefficients(void);
    float readTemp(void);
    float readPressure(void); 
    float readAltitude(const float seaLevelhPa);

    void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year);
    void readDS3231time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year);
    void displayTime();
   
    void loopCycle();
    void lcdTime();
    void lcdDate();
    
  private:
  
    byte decToBcd(byte val);
    byte bcdToDec(byte val);
    void codePID(float& setpoint, float& input,float& prev_input,const float& kp, const float& ki, const float& kd, float& iterm, float& output);
   
    bmp280_calib_data _bmp280_calib_data;
    const float seaLevelhPa = 1013.25f;
    uint16_t bmp280_read16(byte reg);
    int16_t readS16(byte reg);
    int16_t readS16_LE(byte reg);
    uint16_t read16_LittleEndian(byte reg);
    uint32_t bmp280_read24(byte reg);

    void tempfuncion(); void tempfuncion2(); void tempfuncion3();  
    
    unsigned long _time_now, _time_previous;
    float _dataTime;
    int16_t _acx,_acy,_acz,_gyx,_gyy,_gyz,_tmp;
    float _meanAcx,_meanAcy,_meanAcz;
    float _meanGyx, _meanGyy, _meanGyz;
    
    float _tmpgyrx,_tmpgyy,_tmpgyz;
    float _accAngleX, _accAngleY, _accAngleZ;
    float _gyroAngleX, _gyroAngleY, _gyroAngleZ;
    float _filterAngleX = 0.0f, _filterAngleY = 0.0f, _filterAngleZ = 0.0f;
    float _pidRoll, _pidPitch, _pidYaw;
    
    const float _KP = 1.0f;
    const float _KI = 0.0f;
    const float _KD = 0.0f;
    
    float _roll_target_angle = 0.0f;
    float _roll_prev_angle = 0.0f;
    float _roll_iterm;
    float _roll_output;
    
    float _pitch_target_angle = 0.0f;
    float _pitch_prev_angle = 0.0f;
    float _pitch_iterm;
    float _pitch_output;
    
    float _yaw_target_angle = 0.0f;
    float _yaw_prev_angle = 0.0f;
    float _yaw_iterm;
    float _yaw_output;
    
    int32_t _t_fine;
   
    byte _second, _minute, _hour, _dayOfWeek, _dayOfMonth, _month, _year;
    String _day = "";

    char _buffhour[3], _buffmin[3], _buffsec[3];
    char _buffYear[3], _buffMon[3], _buffDay[3];
    char _buffDofDay[8];
    
    char _buffacx[6], _buffacy[6], _buffacz[6];
    char _buffgyx[6], _buffgyy[6], _buffgyz[6]; 
  
    char _buffFiltAngleX[4], _buffFiltAngleY[4], _buffFiltAngleZ[4];
    char _buffpidPitch[4], _buffpidRoll[4], _buffpidYaw[4];
  
    char _buffTemp[6], _buffPress[10], _buffAltit[5];  
     
    void drawTime(char* strhour, char* strmin, char* strsec);
    void drawDate(char* strDofW,char* strYear, char* strMon, char* strDay);
};
 
#endif
