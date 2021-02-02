#include <Arduino.h>
#include <TeensyThreads.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "ACS712.h"
#include <Filters.h>

#include <SPI.h> // not used here, but needed to prevent a RTClib compile error
#include <RTClib.h>
#include <SD.h>

const int chipSelect = 15;
String file_name;
String dir_name;
String header;
String year_name = "", month_name = "", day_name = "", hour_name = "", min_name = "", sec_name = "";
String startTime;
File dataFile;
String logString;
RTC_DS3231 RTC;

float Frequency = 3.0; //Hz 2.5
FilterOnePole current_motor1(LOWPASS, Frequency);
ACS712 sensor(ACS712_20A, 31);

LiquidCrystal_I2C lcd(0x27, 20, 4);
#define SW_GREEN 35
#define SW_RED 36

int PROX[4] = {21, 22, 20, 23}; //HEAD1 SLIDE1 HEAD2 SLIDE2 //detect -> 1
#define PROX_HEAD1 21
#define PROX_HEAD2 20
#define PROX_SLIDE1 22
#define PROX_SLIDE2 23

int SOLENOID[6] = {2, 3, 4, 5, 6, 7}; //0LOCK1 1LOCK2 2HEAD1 3HEAD2 4SLIDE1 5SLIDE2
#define SOLENOID_LOCK1 2
#define SOLENOID_LOCK2 3
#define SOLENOID_HEAD1 4
#define SOLENOID_HEAD2 5
#define SOLENOID_SLIDE1 6
#define SOLENOID_SLIDE2 7

int CURRENT[2] = {31, 32}; //C_MOTOR1 C_MOTOR2
#define C_MOTOR1 31
#define C_MOTOR2 32

#define MOTOR1_A 25
#define MOTOR1_B 24
#define MOTOR1_PWM 10

#define MOTOR2_A 27
#define MOTOR2_B 26
#define MOTOR2_PWM 9

#define SD_MOSI 11
#define SD_MISO 12
#define SD_CS 15

#define RTC_SCL 37
#define RTC_SDA 38

#define LCD_SDA 18
#define LCD_SCL 19

volatile bool flag_first = false;
volatile bool flag_ng1 = false;
volatile bool flag_ng2 = false;
volatile bool flag_last = false;
volatile bool flag_head1 = false;
volatile bool flag_head2 = false;
volatile bool flag_swGreen = false;
volatile bool flag_select = false;
volatile bool flag_sd = false;
volatile bool flag_save = false;

volatile int count_defect = 0;
volatile int count_ng1 = 0;
volatile int count_ng2 = 0;
volatile int count_go = 0;
long time_1;
long time_2;

int c;
int total;

void setupRTC()
{
  delay(500);
  Serial.print("setup rtc");
  Wire1.begin();
  RTC.begin();

  RTC.adjust(DateTime(__DATE__, __TIME__));
  if (!RTC.isrunning())
  {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  //DateTime now = RTC.now();
  //RTC.setAlarm1Simple(21, 58);
  //RTC.turnOnAlarm(1);
  //if (RTC.checkAlarmEnabled(1)) {
  //  Serial.println("Alarm Enabled");
  //}
}

void readRTC()
{
  DateTime now = RTC.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  //if (RTC.checkIfAlarm(1)) {
  //  Serial.println("Alarm Triggered");
  //}
}
void getDataLog()
{ 
  DateTime now = RTC.now();
  dir_name = "/" + String(now.day())+String(now.month()) + String(now.year());
  file_name = String(dir_name) + "/" + String(now.hour()) + String(now.minute()) + String(now.second()) + String(".CSV");
  SD.mkdir(dir_name.c_str());
  
  File dataFile = SD.open(file_name.c_str(), FILE_WRITE);
  Serial.println(file_name);
  if (dataFile)
  {
    //Serial.println("SAVE");
    logString = String("count_go")+","+String("count_ng1")+","+String("count_ng2")+","+String("total")+","+String("start time")+","+String("finish time")+","+"\r\n";
    dataFile.print(logString);
    logString="";
    logString = String(count_go)+","+String(count_ng1)+","+String(count_ng2)+","+String(total)+","+String(startTime)+","+String(now.year())+String("/")+String(now.month())+String("/")+String(now.day())+String("/")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second())+","+ "\r\n";
    dataFile.print(logString);
    dataFile.close();
  }
  else
  {
    Serial.println("error opening datalog.csv");
  }
  logString = "";
}

void motor(int m, int sp)
{
  if (m == 1)
  {
    if (sp > 0)
    {
      digitalWrite(MOTOR1_A, HIGH);
      digitalWrite(MOTOR1_B, LOW);
    }
    else if (sp < 0)
    {
      digitalWrite(MOTOR1_A, LOW);
      digitalWrite(MOTOR1_B, HIGH);
    }
    else
    {
      digitalWrite(MOTOR1_A, HIGH);
      digitalWrite(MOTOR1_B, HIGH);
    }
    analogWrite(MOTOR1_PWM, abs(sp));
  }
  else if (m == 2)
  {
    if (sp > 0)
    {
      digitalWrite(MOTOR2_A, HIGH);
      digitalWrite(MOTOR2_B, LOW);
    }
    else if (sp < 0)
    {
      digitalWrite(MOTOR2_A, LOW);
      digitalWrite(MOTOR2_B, HIGH);
    }
    else
    {
      digitalWrite(MOTOR2_A, HIGH);
      digitalWrite(MOTOR2_B, HIGH);
    }
    analogWrite(MOTOR2_PWM, abs(sp));
  }
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void thread_head1(int data)
{
  while (true)
  {
    if (flag_swGreen == true && flag_head1 == false && flag_last == false)
    {
      //Serial.println("HEAD1 ON");
      digitalWrite(SOLENOID_LOCK1, LOW);
      delay(100);
      //>>>>>>>>>>>>>>>>>>>
      Serial.println(digitalRead(PROX_SLIDE1));
      if (digitalRead(PROX_SLIDE1) == 1)
      {
        digitalWrite(SOLENOID_HEAD1, LOW);
        delay(1000);
        time_1 = millis();
        while (digitalRead(PROX_HEAD1) == LOW) //DOWN
        {
          //Serial.println("PROX");
          motor(1, 255);
          float C;
          if ((millis() - time_1) > 200)
          {
            float I = sensor.getCurrentDC();
            C = current_motor1.input(I);
          }

          if ((millis() - time_1) > 1500)
          {
            Serial.println("Defect Head1");
            flag_ng1 = true; //defect
            //Serial.println("overtime");
            break;
          }
          else if (C < -0.87)
          {
            Serial.println("Over current Head1");
            flag_ng1 = true; //defect
            C = 0;
            break;
          }
          else
          {
            flag_ng1 = false;
          }
        }

        motor(1, 0);
        while (digitalRead(PROX_HEAD1) == HIGH)
        {
          //Serial.println("PROX Head1");
          motor(1, -255);
        }
        motor(1, -255);
        delay(1500);
        motor(1, 0);

        digitalWrite(SOLENOID_HEAD1, HIGH);
        //delay(1000);
        digitalWrite(SOLENOID_LOCK1, HIGH);
        delay(500); //250
      }
      else
      {
        flag_last = true;
      }

      //>>>>>>>>>>>>>>>>>>>
      //digitalWrite(SOLENOID_HEAD1, LOW);
      //delay(1000);

      // time_1 = millis();
      // while (digitalRead(PROX_HEAD1) == LOW) //DOWN
      // {
      //   //Serial.println("PROX");
      //   motor(1, 255);
      //   if ((millis() - time_1) > 1500)
      //   {
      //     Serial.println("Defect Head1");
      //     flag_ng1 = true; //defect
      //     //Serial.println("overtime");

      //     break;
      //   }
      //   else
      //   {
      //     flag_ng1 = false;
      //   }
      // }

      // motor(1, 0);
      // while (digitalRead(PROX_HEAD1) == HIGH)
      // {
      //   //Serial.println("PROX Head1");
      //   motor(1, -255);
      // }
      // motor(1, -255);
      // delay(1500);
      // motor(1, 0);

      // digitalWrite(SOLENOID_HEAD1, HIGH);
      // //delay(1000);
      // digitalWrite(SOLENOID_LOCK1, HIGH);
      // delay(500);//250

      flag_head1 = true;
    }
    delay(50);
  }
}
void thread_head2(int data)
{
  while (true)
  {
    if (flag_swGreen == true && flag_head2 == false && flag_last == false)
    {
      if (flag_ng1 == true || flag_first == false)
      {
        Serial.println("Head2 Pass");
        if (flag_ng1 == true)
        {
          count_ng1++;
        }
        flag_ng2 = true;
        flag_head2 = true;
      }
      else
      {
        Serial.println("Head2 Not Pass");
        digitalWrite(SOLENOID_LOCK2, LOW);
        //delay(250);
        digitalWrite(SOLENOID_HEAD2, LOW);
        delay(1000);

        time_2 = millis();
        while ((millis() - time_2) < 1000) //DOWN
        {
          //Serial.println("PROX");
          motor(2, 125);
          if (digitalRead(PROX_HEAD2) == HIGH)
          {
            Serial.println("Defect Head2");
            flag_ng2 = true; //defect
            //Serial.println("overtime");
            count_ng2++;
            break;
          }
          else
          {
            flag_ng2 = false;
          }
        }

        motor(2, -125);
        delay(1000);
        motor(2, 0);

        digitalWrite(SOLENOID_HEAD2, HIGH);
        //delay(1000);
        digitalWrite(SOLENOID_LOCK2, HIGH);
        delay(500); //250

        // Serial.print("NG1 = ");
        // Serial.print(flag_ng1);
        // Serial.print("NG2 = ");
        // Serial.println(flag_ng2);

        flag_head2 = true;
      }
    }
    delay(50);
  }
}

void thread_slide(int data)
{
  while (true)
  {
    if (flag_swGreen == true)
    {
      if (flag_first == false)
      {
        if (flag_head1 == true && flag_head2 == true)
        {
          // Serial.print("NG1>> ");
          // Serial.print(flag_ng1);
          // Serial.print("  NG2>> ");
          // Serial.println(flag_ng2);

          // Serial.println("First ");
          // if(flag_ng1 == true)
          // {
          //   Serial.println("First Defect");
          //   digitalWrite(SOLENOID_SLIDE2,LOW);
          //   delay(150);
          //   count_defect ++;
          // }
          // else
          // {
          //   Serial.println("First Good");
          //   digitalWrite(SOLENOID_SLIDE2,HIGH);
          //   delay(150);
          // }
          digitalWrite(SOLENOID_SLIDE1, LOW);
          delay(400);
          flag_first = true;
          while (digitalRead(PROX_SLIDE1) == LOW)
            ;

          digitalWrite(SOLENOID_SLIDE1, HIGH);
          delay(400);
          flag_head1 = false;
          flag_head2 = false;
        }
      }
      else
      {
        if (flag_head1 == true && flag_head2 == true)
        {

          if (flag_ng2 == true)
          {
            //Serial.println("Defect");
            digitalWrite(SOLENOID_SLIDE2, LOW);
            delay(100);
            //count_defect++;
          }
          else
          {
            digitalWrite(SOLENOID_SLIDE2, HIGH);
            delay(100);
            count_go++;
          }

          // Serial.print("NG1>> ");
          // Serial.print(flag_ng1);
          // Serial.print("  NG2>> ");
          // Serial.println(flag_ng2);

          // if (flag_ng1 == true && flag_ng2 == false)
          // {
          //   Serial.println("111 Only NG1");
          //   digitalWrite(SOLENOID_SLIDE2, LOW);
          //   delay(150);
          //   count_defect++;
          // }
          // else if (flag_ng1 == false && flag_ng2 == true)
          // {
          //   Serial.println("222 NG1 NG2");
          //   digitalWrite(SOLENOID_SLIDE2, LOW);
          //   delay(150);
          //   count_defect++;
          // }
          // else if (flag_ng1 == false && flag_ng2 == false)
          // {
          //   Serial.println("333 Good");
          //   digitalWrite(SOLENOID_SLIDE2, HIGH);
          //   delay(150);
          // }

          digitalWrite(SOLENOID_SLIDE1, LOW);
          delay(400);
          while (true)
          {
            if (digitalRead(PROX_SLIDE1) == LOW && digitalRead(PROX_SLIDE2) == HIGH)
            {
              //flag_last = true;
               if(flag_last == true)
                {
                  getDataLog();
                }
              break;
            }
            else if (digitalRead(PROX_SLIDE1) == HIGH && digitalRead(PROX_SLIDE2) == HIGH)
            {
              break;
            }
            delay(50);
          }
          digitalWrite(SOLENOID_SLIDE1, HIGH);
          delay(400);
          flag_head1 = false;
          flag_head2 = false;
        }
      }
      delay(50);
    }
  }
}

void setup()
{
  setupRTC();
  Wire.setClock(100000);
  analogWriteFrequency(9, 10000);
  analogWriteFrequency(10, 10000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");

  for (int i = 0; i < 4; i++)
  {
    pinMode(PROX[i], INPUT);
  }
  pinMode(SW_GREEN, INPUT);
  pinMode(SW_RED, INPUT);
  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  for (int i = 0; i < 6; i++)
  {
    pinMode(SOLENOID[i], OUTPUT);
  }
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(SOLENOID[i], HIGH);
  }
  delay(500);

  lcd.begin();
  lcd.setCursor(0, 0);
  lcd.print("Go");
  lcd.setCursor(0, 1);
  lcd.print("NG1");
  lcd.setCursor(0, 2);
  lcd.print("NG2");
  lcd.setCursor(0, 3);
  lcd.print("Total ");

  threads.addThread(thread_head1, 1);
  threads.addThread(thread_head2, 1);
  threads.addThread(thread_slide, 1);

  //pinMode(SW_GREEN,INPUT);
  //pinMode(SW_RED,INPUT);
  delay(1000);
  Serial.println("Calibrating... Ensure that no current flows through the sensor at this moment");
  int zero = sensor.calibrate();
  Serial.println("Done!");
  //Serial.println("Zero point for this sensor = " + zero);
}
void loop()
{


  total = count_go + count_ng1 + count_ng2;
  lcd.setCursor(6, 0);
  lcd.print(count_go);
  lcd.setCursor(6, 1);
  lcd.print(count_ng1);
  lcd.setCursor(6, 2);
  lcd.print(count_ng2);
  lcd.setCursor(6, 3);
  lcd.print(total);

 
  if (!digitalRead(SW_GREEN))
  {
    while(!digitalRead(SW_GREEN));
    Serial.println("G ON");
    DateTime now = RTC.now();\
    startTime = String(now.year())+String("/")+String(now.month())+String("/")+String(now.day())+String("/")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second());
    flag_swGreen = true;
    flag_last = false;
    flag_first = false;
  }
  else if (!digitalRead(SW_RED))
  {
   while(!digitalRead(SW_RED));
   lcd.clear();
   delay(100);
   total=0;   
   count_go=0; 
   count_ng1=0;
   count_ng2=0;
  
   lcd.setCursor(0, 0);
   lcd.print("Go");
   lcd.setCursor(0, 1);
   lcd.print("NG1");
   lcd.setCursor(0, 2);
   lcd.print("NG2");
   lcd.setCursor(0, 3);
   lcd.print("Total ");

   flag_first = false;
   flag_ng1 = false;
   flag_ng2 = false;
   flag_last = false;
   flag_head1 = false;
   flag_head2 = false;
   flag_swGreen = false;
   flag_select = false;
   flag_sd = false;
   flag_save = false;
   for (int i = 0; i < 6; i++)
   {
     digitalWrite(SOLENOID[i], HIGH);
   }
   Serial.println("R ON");
   flag_swGreen = false;
   
    // getDataLog();
  }
  // if(flag_last == true&&flag_head1==true&&flag_head2==true)
  // {
  //   getDataLog();
  // }
  //readRTC();
  //Serial.println(current_motor1.output());
  //motor(1,250);
  // float I = sensor.getCurrentDC();
  // current_motor1.input(I);
  // Serial.println(current_motor1.output());
  // Send it to serial
  delay(100);

  //Serial.println(c++);
}

