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

#include "U8glib.h"
#include "Wire.h"
#include "watch.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); 

void Watch::begin(){
  
  u8g.setFont(u8g_font_unifont);   
  u8g.setColorIndex(1); 
  
  mpu6050Setup(); 
  calibrationAccGyro(); 
  initDataTime();
  bmp280_Write(BMP280_REGISTER_CONTROL, 0x3F);
  readCoefficients();
}
void Watch::initDataTime(){
  _time_previous = millis();
}

void Watch::calcDataTime(){
  //unsigned long time_now, time_previous;
  _time_now = millis();
  _dataTime = (_time_now - _time_previous) / 1000.0;
  _time_previous = _time_now;
}

void Watch::mpu6050Setup(){
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
}
void Watch::mpu6050Read(){
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS,14,true);
  
  _acx = Wire.read() << 8 | Wire.read(); // 가속도 x축 8비트씩 2번 읽어 16비트 만들기 
  _acy = Wire.read() << 8 | Wire.read();
  _acz = Wire.read() << 8 | Wire.read();
  _tmp = Wire.read() << 8 | Wire.read();
  _gyx = Wire.read() << 8 | Wire.read();
  _gyy = Wire.read() << 8 | Wire.read();
  _gyz = Wire.read() << 8 | Wire.read();
  
  Wire.endTransmission();
}

void Watch::calibrationAccGyro(){

  int i = 0;
  float sumAcx = 0, sumAcy = 0, sumAcz = 0;
  float sumGyx = 0, sumGyy = 0, sumGyz = 0;
  
  mpu6050Read();

  for(i=0;i<10;i++){

    mpu6050Read();
  
    sumAcx += _acx;
    sumAcy += _acy;
    sumAcz += _acz;

    sumGyx += _gyx;
    sumGyy += _gyy;
    sumGyz += _gyz;

    delay(100);
  }

  _meanAcx = (sumAcx / 10);
  _meanAcy = (sumAcy / 10);
  _meanAcz = (sumAcz / 10);
  
  _meanGyx = (sumGyx / 10);
  _meanGyy = (sumGyy / 10);
  _meanGyz = (sumGyz / 10);
  
}
void Watch::codePID(float& setpoint, float& input,float& prev_input,
const float& kp, const float& ki, const float& kd, float& iterm, float& output){
  
  float error;
  float dInput;
  float pterm,dterm;

  error = setpoint - input;
  dInput = input - prev_input;
  prev_input = input;

  pterm = kp * error;
  iterm += ki * error * _dataTime;
  dterm = -( kd * (dInput / _dataTime));

  output = pterm + iterm + dterm;

}
void Watch::calculateAccYPR(){

  float degreeAcx, degreeAcy, degreeAcz;
  float degreexz, degreeyz;
  const float RADIANS_TO_DEGREE = 180/3.14159;

  degreeAcx = _acx - _meanAcx;
  degreeAcy = _acy - _meanAcy;
  degreeAcz = _acz - (16384 - _meanAcz);

  degreeyz = sqrt(pow(degreeAcy,2) + pow(degreeAcz,2));
  _accAngleY = atan((-degreeAcx)/(degreeyz)) * RADIANS_TO_DEGREE;

  degreexz = sqrt(pow(degreeAcx,2) + pow(degreeAcz,2));
  _accAngleX = atan((degreeAcy)/(degreexz)) * RADIANS_TO_DEGREE;

  _accAngleZ = degreeAcz;
}
void Watch::calculateGyroYPR(){

  const float GYRO_TO_DEGREE_PER_SEC = 131; // mem setting 0 -> 32767/250['/s]

  _gyroAngleX += (((_gyx - _meanGyx)/(GYRO_TO_DEGREE_PER_SEC)) * _dataTime );
  _gyroAngleY += (((_gyy - _meanGyy)/(GYRO_TO_DEGREE_PER_SEC)) * _dataTime );
  _gyroAngleZ += (((_gyz - _meanGyz)/(GYRO_TO_DEGREE_PER_SEC)) * _dataTime );
  
}

void Watch::filterYPR(){
  
  //float filterAngleX = 0.0f, filterAngleY = 0.0f, filterAngleZ = 0.0f;
  const float COEFFICIENT_K = 0.96; // COEFF_K = TAU / TAU + DELTA_TIME;
  const float GYRO_TO_DEGREE_PER_SEC = 131;
  float tmpAngleX, tmpAngleY, tmpAngleZ;
  
  tmpAngleX = _filterAngleX + (((_gyx - _meanGyx)/(GYRO_TO_DEGREE_PER_SEC)) * _dataTime );
  tmpAngleY = _filterAngleY + (((_gyy - _meanGyy)/(GYRO_TO_DEGREE_PER_SEC)) * _dataTime );
  tmpAngleZ = _filterAngleZ + (((_gyz - _meanGyz)/(GYRO_TO_DEGREE_PER_SEC)) * _dataTime );

  _filterAngleX = (COEFFICIENT_K * tmpAngleX) + ((1.0 - COEFFICIENT_K) * _accAngleX);
  _filterAngleY = (COEFFICIENT_K * tmpAngleY) + ((1.0 - COEFFICIENT_K) * _accAngleY);
  _filterAngleZ = tmpAngleZ;
  
}

void Watch::calculatePID(const float KP, const float KI, const float KD){
  
  codePID(_pitch_target_angle,_filterAngleX,_pitch_prev_angle,KP,KI,KD,_pitch_iterm,_pitch_output);
  codePID(_roll_target_angle,_filterAngleY,_roll_prev_angle,KP,KI,KD,_roll_iterm,_roll_output);
  codePID(_yaw_target_angle,_filterAngleZ,_yaw_prev_angle,KP,KI,KD,_yaw_iterm,_yaw_output);
  
}


void Watch::bmp280_Write(byte reg, byte value)
{
    Wire.begin();
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}
uint16_t Watch::bmp280_read16(byte reg)
{
    uint16_t value;

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS,2);
    value = (Wire.read() << 8) | Wire.read();
    
    return value;
}

int16_t Watch::readS16(byte reg)
{
  return (int16_t)bmp280_read16(reg);

}

int16_t Watch::readS16_LE(byte reg)
{
  return (int16_t)read16_LittleEndian(reg);
}

uint16_t Watch::read16_LittleEndian(byte reg) {
  uint16_t temp = bmp280_read16(reg);
  return (temp >> 8) | (temp << 8);
}

uint32_t Watch::bmp280_read24(byte reg)
{
    uint32_t value;

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 3);
    
    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();

    return value;
}

void Watch::readCoefficients(void)
{
    _bmp280_calib_data.dig_T1 = read16_LittleEndian(BMP280_REGISTER_DIG_T1);
    _bmp280_calib_data.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib_data.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib_data.dig_P1 = read16_LittleEndian(BMP280_REGISTER_DIG_P1);
    _bmp280_calib_data.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib_data.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib_data.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib_data.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib_data.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib_data.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib_data.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib_data.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value

float Watch::readTemp(void)
{
  int32_t var1, var2;

  int32_t adc_T = bmp280_read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1  = ((((adc_T >> 3) - ((int32_t)_bmp280_calib_data.dig_T1 << 1 ))) * ((int32_t)_bmp280_calib_data.dig_T2)) >> 11;

  var2  = (((((adc_T >> 4) - ((int32_t)_bmp280_calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)_bmp280_calib_data.dig_T1))) >> 12) *
     ((int32_t)_bmp280_calib_data.dig_T3)) >> 14;

  _t_fine = var1 + var2;

  float T  = (_t_fine * 5 + 128) >> 8;
  return T/100;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa

float Watch::readPressure(void) {
  
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemp();
  
  int32_t adc_P = bmp280_read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib_data.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib_data.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib_data.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib_data.dig_P3) >> 8) + ((var1 * (int64_t)_bmp280_calib_data.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47)+var1))*((int64_t)_bmp280_calib_data.dig_P1) >> 33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib_data.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib_data.dig_P7) << 4);
  return (float)p/256;
}

float Watch::readAltitude(const float seaLevelhPa) {
  
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}

byte Watch::decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte Watch::bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void Watch::setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}

void Watch::readDS3231time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}
void Watch::displayTime()
{
  
  // retrieve data from DS3231
  readDS3231time(&_second, &_minute, &_hour, &_dayOfWeek, &_dayOfMonth, &_month, &_year);
    
switch(_dayOfWeek){
  
  case 1:
    _day = "Sunday";
    //Serial.println("Sunday");
    break;
  case 2:
    _day = "Monday";
    //Serial.println("Monday");
    break;
  case 3:
    _day = "Tuesday";
    //Serial.println("Tuesday");
    break;
  case 4:
    _day = "Wednesday";
    //Serial.println("Wednesday");
    break;
  case 5:
    _day = "Thursday";
   //Serial.println("Thursday");
    break;
  case 6:
    _day = "Friday";
    //Serial.println("Friday");
    break;
  case 7:
    _day = "Saturday";
    //Serial.println("Saturday");
    break;
  }
}

void Watch::tempfuncion(){
  
   String _strhour = String(""), _strmin = String(""), _strsec = String(""); 
   String _strYear = String(""), _strMon = String(""), _strDay = String(""), _srtDofW = String("");
   
  _strhour += (short)(_hour); _strmin += (short)(_minute); _strsec += (short)(_second);
  _strYear += (short)(_year); _strMon += (short)(_month); _strDay += (short)(_dayOfMonth);
  _srtDofW += (_day);
  
  _strhour.toCharArray(_buffhour, 3); _strmin.toCharArray(_buffmin, 3); _strsec.toCharArray(_buffsec, 3);
  _strYear.toCharArray(_buffYear, 3); _strMon.toCharArray(_buffMon, 3); _strDay.toCharArray(_buffDay, 3);
  _srtDofW.toCharArray(_buffDofDay, 8);
  
}

void Watch::tempfuncion2(){

  String _strAcx = String(""), _strAcy = String(""), _strAcz = String("");
  String _strGyx = String(""), _strGyy = String("") , _strGyz = String("");
  String _strFilAngleX = String(""), _strFilAngleY = String("") , _strFilAngleZ = String("");
  String _strpidPitch = String(""), _strpidRoll = String("") , _strpidYaw = String("");

  _strAcx += (short)(_accAngleX); _strAcy += (short)(_accAngleY); _strAcz += (short)(_accAngleZ);
  _strGyx += (short)(_gyroAngleX); _strGyy += (short)(_gyroAngleY); _strGyz += (short)(_gyroAngleZ);
  _strFilAngleX += (short)(_filterAngleX); _strFilAngleY += (short)(_filterAngleY); _strFilAngleZ += (short)(_filterAngleZ);
  _strpidPitch += (short)(_pitch_output); _strpidRoll += (short)(_roll_output); _strpidYaw += (short)(_yaw_output);
  
  _strAcx.toCharArray(_buffacx, 6); _strAcy.toCharArray(_buffacy, 6); _strAcz.toCharArray(_buffacz, 6);
  _strGyx.toCharArray(_buffgyx, 6); _strGyy.toCharArray(_buffgyy, 6); _strGyz.toCharArray(_buffgyz, 6);
  _strFilAngleX.toCharArray(_buffFiltAngleX, 4); _strFilAngleY.toCharArray(_buffFiltAngleY, 4); _strFilAngleZ.toCharArray(_buffFiltAngleZ, 4);
  _strpidPitch.toCharArray(_buffpidPitch, 4); _strpidRoll.toCharArray(_buffpidRoll, 4); _strpidYaw.toCharArray(_buffpidYaw, 4);
}

void Watch::tempfuncion3(){
  
  String _strTemp = String(""), _strPress = String("") , _strAltit = String("");
  
  _strTemp += (short)(readTemp()); 
  _strPress += (int)(readPressure()); 
  _strAltit += (short)(readAltitude(seaLevelhPa));

  _strTemp.toCharArray(_buffTemp, 6); _strPress.toCharArray(_buffPress, 10); _strAltit.toCharArray(_buffAltit,5);
  
}
void Watch::loopCycle(){
  
  displayTime(); 
  mpu6050Read();
  calcDataTime();
  calculateAccYPR();
  calculateGyroYPR();
  filterYPR();
  calculatePID(_KP,_KI,_KD);
  tempfuncion(); tempfuncion2(); tempfuncion3();
  
}
void Watch::lcdTime()
{
  drawTime(_buffhour,_buffmin,_buffsec);
}
void Watch::lcdDate()
{
  drawDate(_buffDofDay, _buffYear, _buffMon , _buffDay);
}
void Watch::drawTime(char* strhour, char* strmin, char* strsec) {
 
  // picture loop  
  u8g.firstPage();  
  do {
    // show text
    u8g.drawStr(0, 12, "TIME");
    
    u8g.drawStr(0, 30, "Hour :"); u8g.drawStr(60, 30, strhour);
    u8g.drawStr(0, 45, "Min :"); u8g.drawStr(60, 45, strmin);
    u8g.drawStr(0, 60, "Sec :"); u8g.drawStr(60, 60, strsec);
    
  } while( u8g.nextPage() );
}

void Watch::drawDate(char* strDofW,char* strYear, char* strMon, char* strDay) {
 
  // picture loop  
  u8g.firstPage();  
  do {
    // show text
    u8g.drawStr(0, 12, "Date"); u8g.drawStr(60, 12, strDofW);
    
    u8g.drawStr(0, 30, "Year :"); u8g.drawStr(60, 30, strYear);
    u8g.drawStr(0, 45, "Month :"); u8g.drawStr(60, 45, strMon);
    u8g.drawStr(0, 60, "day :"); u8g.drawStr(60, 60, strDay);
    
  } while( u8g.nextPage() );
}


