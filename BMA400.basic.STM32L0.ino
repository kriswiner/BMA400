/* 
   BMA400 accelerometer sensor test code
   Copyright2 2018 Tlera Corporation

   This sketch may be used without limitations with proper attribution

   This example code is in the public domain.
*/
#include <STM32L0.h>
#include <RTC.h>
#include "BMA400.h"

// Cricket pin assignments
#define myLed    10 // blue led 
#define myBat    A1 // LiPo battery ADC
#define myBat_en  2 // LiPo battery monitor enable

bool SerialDebug = true;

uint16_t Hour = 1, Minute = 1, Second = 1, Millisec, Year = 1, Month = 1, Day = 1;
uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
uint32_t subSeconds, milliseconds;
bool alarmFlag = false;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp;

//BMA400 definitions
#define BMA400_intPin1 A4   // interrupt1 pin definitions, wake-up from STANDBY pin
#define BMA400_intPin2  3   // interrupt2 pin definitions, data ready or sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      SR_15_5Hz, SRW_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode 
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t Ascale = AFS_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // accel bias offsets

// Logic flags to keep track of device states
bool BMA400_wake_flag = false;
bool BMA400_sleep_flag = false;
bool InMotion = false;

BMA400 BMA400(BMA400_intPin1, BMA400_intPin2); // instantiate BMA400 class


void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");
  
  /* configure IO pins */
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);  // start with blue led off, since active LOW

  pinMode(myBat_en, OUTPUT);
  pinMode(myBat, INPUT);    // set up ADC battery voltage monitor pin
  analogReadResolution(12); // use 12-bit ADC resolution

  pinMode(BMA400_intPin1, INPUT);  // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);

  /* initialize wire bus */
  Wire.begin(); // set master mode on default pins 14/15
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);

  BMA400.I2Cscan(); // should detect BMA400 at 0x18 
  
  /* Set the RTC time */
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the RTC date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  
  /* Check internal STML082 and battery power configuration */
  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  VBAT = 1.27f * 3.30f * ((float) analogRead(myBat)) / 4096.0f;
  STM32L0Temp = STM32L0.getTemperature();
  
  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
  if(VBUS ==  1)  Serial.println("USB Connected!"); 
  Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
  Serial.println(" ");
  

  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte c = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(1000); 

  if(c == 0x90) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("BMA400 is online..."); Serial.println(" ");
   
   aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);      
   BMA400.selfTestBMA400();                                             // perform sensor self test
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(1000);                                                         // give some time to read the screen
   BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
   BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application                     

  }
  else 
  {
  if(c != 0x90) Serial.println(" BMA400 not functioning!");
  }

   // set alarm to update the RTC periodically
  RTC.setAlarmTime(0, 0, 0);
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(BMA400_intPin1, myinthandler1, RISING);  // define wake-up interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // define data ready interrupt for INT2 pin output of BMA400 

  BMA400.getStatus(); // read status of interrupts to clear
    /* end of setup */
}

void loop()
{
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 
   digitalWrite(myLed, LOW);
  }

  if(BMA400_sleep_flag)
  {
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   detachInterrupt(BMA400_intPin2);       // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 
   digitalWrite(myLed, HIGH);
  }/* end of sleep/wake detect */
 
  /*RTC*/
  if (alarmFlag) { // update RTC output whenever the alarm triggers
    alarmFlag = false;
    
    if(SerialDebug && InMotion) {
      
     BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

    // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - offset[1];   
     az = (float)accelCount[2]*aRes - offset[2]; 
     
    Serial.println(" ");
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.println(" ");
    }

    VDDA = STM32L0.getVDDA();
    digitalWrite(myBat_en, HIGH);
    VBAT = 1.27f * VDDA * ((float) analogRead(myBat)) / 4096.0f;
    digitalWrite(myBat_en, LOW);
    STM32L0Temp = STM32L0.getTemperature();
    if(SerialDebug) {
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
      Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
      Serial.println(" ");
    }

    tempCount = BMA400.readBMA400TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

    // Read RTC
    Serial.println("RTC:");
    RTC.getDate(day, month, year);
    RTC.getTime(hours, minutes, seconds, subSeconds);

    milliseconds = ((subSeconds >> 17) * 1000 + 16384) / 32768;

    Serial.print("RTC Time = ");
    if (hours < 10)   {Serial.print("0");Serial.print(hours); } else Serial.print(hours);
    Serial.print(":");
    if (minutes < 10) {Serial.print("0"); Serial.print(minutes); } else Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10) {Serial.print("0"); Serial.print(seconds); } else Serial.print(seconds);
    Serial.print(".");
        if (milliseconds <= 9) {
            Serial.print("0");
        }
        if (milliseconds <= 99) {
            Serial.print("0");
        }
    Serial.print(milliseconds);
    Serial.println(" ");

    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);
        
    } // end of alarm section
    
 
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */
void myinthandler1()
{
  BMA400_wake_flag = true; 
  STM32L0.wakeup();
  Serial.println("** BMA400 is awake! **");
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
  Serial.println("** BMA400 is asleep! **");
}


void alarmMatch()
{
  alarmFlag = true;
  STM32L0.wakeup();
}
