#include <SoftwareSerial.h>

// Board:    Arduino Uno with Parallax Board of Education
// Title:    maestro_mod
// Date:     2013-03-28
// Author:   Kim Woodward
// Website:  rpiexperimenter.blogspot.com
// Details:  This is a sketch connecting Arduino and
//           the Mini Maestro Serial Servo Controller
//           to four servos, an LCD panel, an IR distance
//           sensor, and an ultrasonic distance sensor.

// Pololu Mini-Maestro 18 Serial Servo Controller
// Power 9v Seperate power supply from Arduino
// GND to GND on the Arduino
// SSC RX pin to Arduino TX pin Pin04
// SSC TX pin to Arduino RX pin Pin03
// Channel 0 - Ultrasonic Distance Sensor Servo
// Channel 2 - Camera Pan Servo
// Channel 4 - Camera Tilt Servo
// Channel 6 - IR Distance Sensor Servo

// GP2D12 IR Distance Sensor
// Power 5v from the Arduino
// GND to GND on the Arduino
// GP2D12 Sensor analog Rx pin to Arduino Analog pin 0

// Parallax 2x16 Serial LCD Panel
// Power 5v from the Arduino
// GND to GND on the Arduino
// LCD Rx pin to Arduino TX pin Pin06

// HC-SR04 Ultrasonic Distance Sensor
// Power 5v from the Arduino
// GND to GND on the Arduino
// HC-SR04 Trig to Arduino TX pin Pin13
// HC-SR04 Echo to Arduino RX pin Pin12

// LIBRARIES---------------
// Included libraries
// NewSoftSerial
#include <Arduino.h>
#include <SoftwareSerial.h>
// ^^^---------------------//

// DEFINE------------------
// Define constants
// Serial pins
const int tx_ToSSC=4; ///< Tx pin going to SSC
const int rx_ToSSC=3; ///< Rx pin coming from SSC
const int tx_ToLCD=6; ///< Tx pin going to LCD panel
const int rx_FrmLCD=5; ///< Rx pin coming from LCD panel(nc)
// Digital pins
const int digtx_ToUltraTrig=12; ///< Tx pin going to ultrasonic sensor trig
const int digrx_FrmUltraEcho=13; ///< Rx pin coming from ultrasonic sensor echo
// Channels on SSC
const int chan_ultraservo=0; ///< Channel 0 - Ultrasonic Distance Sensor Servo
const int chan_camerapanservo=2; ///< Channel 2 - Camera Pan Servo
const int chan_cameratiltservo=4; ///< Channel 4 - Camera Tilt Servo
const int chan_irdistservo=6; ///< Channel 6 - IR Distance Sensor Servo
// Analog pins
const int rx_irdistanalog=0; ///< IR Distance Sensor analog Rx
// ^^^---------------------//  

// INITUALIZE--------------
// initalizing serial connections
SoftwareSerial SSC_Serial = SoftwareSerial(rx_ToSSC, tx_ToSSC);
SoftwareSerial LCD_Serial = SoftwareSerial(rx_FrmLCD, tx_ToLCD);
// ^^^---------------------//  

// SETUP-------------------
// run once, when the sketch starts
void setup()
{
  // setup pins for serial interface to SSC
  pinMode(rx_ToSSC, INPUT);
  digitalWrite(tx_ToSSC, HIGH);
  pinMode(tx_ToSSC, OUTPUT);
  // set SSC serial interface speed
  SSC_Serial.begin(9600);
 
  // setup pins for serial interface to LCD
  pinMode(rx_FrmLCD, INPUT);
  digitalWrite(tx_ToLCD, HIGH);
  pinMode(tx_ToLCD, OUTPUT);
  // set LCD serial interface speed
  LCD_Serial.begin(9600);
  // initial clear and backlight for LCD
  LCD_Serial.write(12); // Clear             
  LCD_Serial.write(17); // Turn backlight on
  delay(5);             // Required delay for response

  // set SSC servos to initial state
  ssc_cmnd_pwm(chan_ultraservo,6000);
  ssc_cmnd_pwm(chan_camerapanservo,6000);
  ssc_cmnd_pwm(chan_cameratiltservo,6000);
  ssc_cmnd_pwm(chan_irdistservo,6000);
  
  // set Ultrasonic Distance Sensor pins
  pinMode(digtx_ToUltraTrig, OUTPUT);
  pinMode(digrx_FrmUltraEcho, INPUT);
  
  // setup our serial monitor
  Serial.begin(9600);
  
  // wait for settling to happen
  delay(1000);  
}

// Send a Set servo pwm command to the Maestro.
// Target is in units of quarter microseconds
// so the normal range is 4000 to 8000.
void ssc_cmnd_pwm(unsigned char servo, unsigned int target)
{
    SSC_Serial.write(0xAA); //start byte
    SSC_Serial.write(0x0C); //device id
    SSC_Serial.write(0x04); //command number
    SSC_Serial.write(servo); //servo number
    SSC_Serial.write(target & 0x7F);
    SSC_Serial.write((target >> 7) & 0x7F);
}

// display two strings on the LCD panel
void lcd_display_2_str(String myvalue1, String myvalue2) {
  delay(100);
  LCD_Serial.write(12); // Clear             
  LCD_Serial.print(myvalue1); // First line
  LCD_Serial.write(13); // Form Feed
  LCD_Serial.print(myvalue2); // Second line
}  

// display string and int value on one line, string on second line on the LCD panel
void lcd_display_2_str_int(String myvalue1, int myvalue, String myvalue2) {
  delay(100);
  LCD_Serial.write(12); // Clear             
  LCD_Serial.print(myvalue1); // First line
  LCD_Serial.print(myvalue, DEC);
  LCD_Serial.write(13); // Form Feed
  LCD_Serial.print(myvalue2); // Second line
}  

// display integer value on the LCD panel
void lcd_display_int_val(int myvalue) {
  delay(100);
  LCD_Serial.write(12);                 // Clear             
  LCD_Serial.print("val: ");
  LCD_Serial.print(myvalue, DEC);  // First line
  LCD_Serial.write(13);                 // Form feed
}  

/* 
 read_gp2d12_range
 Function that reads a value from GP2D12 infrared distance sensor and returns a value in centimeters.

 This sensor should be used with a refresh rate of 36ms or greater.

 BY: Javier Valencia 2008

 float read_gp2d12_range(byte pin)

 It can return -1 if something gone wrong.

 */

float read_gp2d12_range(byte pin) {
	int tmp;

	tmp = analogRead(pin);
	if (tmp < 3)
		return -1; // invalid value

	return (6787.0 /((float)tmp - 3.0)) - 4.0;
}

void test_irdistance()
{
  float myval;
  myval = read_gp2d12_range(rx_irdistanalog);
  Serial.print("ir val: ");
  Serial.println(myval);
//  LCD_Serial.write(12);                 // Clear             
//  LCD_Serial.print("ir val: ");
//  LCD_Serial.print(myval, DEC);         // First line
//  LCD_Serial.write(13);                 // Form feed
}

/*
  HC-SR04 Ping distance sensor
  VCC to Arduino 5v, GND to Arduino GND
  Echo to Arduino pin 13, Trig to Arduino pin 12
  more info at: http://goo.gl/kJ8G1
  */

float grab_ultrasonic()
{
  int duration;
  float distance;
  digitalWrite(digtx_ToUltraTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(digtx_ToUltraTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(digtx_ToUltraTrig, LOW);
  duration = pulseIn(digrx_FrmUltraEcho, HIGH);
  distance = (duration/2) / 29.1;
}

void test_ultrasonic()
{
  float mydist;
  mydist = grab_ultrasonic();
  if (mydist >= 200 || mydist <= 0){
    Serial.println("Ultra: Out of range");
//    LCD_Serial.write(12);                 // Clear             
//    LCD_Serial.print("Out of range");
//    LCD_Serial.write(13);                 // Form feed
  } else {
    Serial.print("Ultra: ");
    Serial.print(mydist);
    Serial.println(" cm");
//    LCD_Serial.write(12);                 // Clear             
//    LCD_Serial.print("Out of range");
//    LCD_Serial.print(mydist, DEC);
//    LCD_Serial.print(" cm");
//    LCD_Serial.write(13);                 // Form feed
  }
}
    
// ^^^---------------------//

// LOOP--------------------// The base for the program
void loop()
{
  // test for servo movement
//   delay(1000);
//   lcd_display_int_val(4000);
//   ssc_cmnd_pwm(chan_ultraservo, 4000);
//   delay(1000);
//   lcd_display_int_val(6000);
//   ssc_cmnd_pwm(chan_ultraservo, 6000);
//   delay(1000);
//   lcd_display_int_val(8000);
//   ssc_cmnd_pwm(chan_ultraservo, 8000);
//   delay(1000);
//   lcd_display_int_val(6000                                                                                                                                                                                                                                                                                                                                                                                                                                                                        );
//   ssc_cmnd_pwm(chan_ultraservo, 6000);
  // test for distance sensors
  test_ultrasonic();
  test_irdistance();
}
// ^^^---------------------//
