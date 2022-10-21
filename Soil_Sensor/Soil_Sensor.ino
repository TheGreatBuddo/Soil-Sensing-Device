//Mayfly LED Blink demo
#include <Arduino.h>
#include <Wire.h>

// Declare variables with a type and initialize with an inital value assignment
int motor = 6;
int sol_1 = 7;
int sol_2 = 8;
int sol_3 = 9;
int sol_4 = 10;
int sol_5 = 11;

//define conditions for millis delay
unsigned long CO2_previousMillis = 0;
const long CO2_interval = 1000;

unsigned long motor_previousMillis = 0;
const long motor_interval = 2000;



// The setup function runs once when you press reset or power the board
void setup()           // the void keyword is used in function declarations to indicate that no information will be returned outside the function.
{
  Serial.begin(9600);
  Wire.begin();                // join i2c bus with address #8
  Serial.println("EnviroDIY Mayfly");

  //set output pins
  pinMode(motor, OUTPUT);
  pinMode(sol_1, OUTPUT);
  pinMode(sol_2, OUTPUT);
  pinMode(sol_3, OUTPUT);
  pinMode(sol_4, OUTPUT);
  pinMode(sol_5, OUTPUT);

  //Turn solenoids 1-5,and motor off
  digitalWrite(motor, HIGH);
  digitalWrite(sol_1, HIGH);
  digitalWrite(sol_2, HIGH);
  digitalWrite(sol_3, HIGH);
  digitalWrite(sol_4, HIGH);
  digitalWrite(sol_5, HIGH);




  
}

// The loop function runs over and over again forever
void loop() {
  unsigned long currentMillis = millis();
  
  byte rcvData;


  if (currentMillis - CO2_previousMillis >= CO2_interval) 
  {
    CO2_previousMillis = currentMillis;


     //read PRHT-A1 temperature,pressure,humidity sensor
    Wire.requestFrom(0x65,1);
    Wire.requestFrom(0x68,1);
    if(Wire.available())
    {
//      rcvData = Wire.read(0x65);
//      Serial.print("address 65 reading = ");
//      Serial.print(rcvData);
//      Serial.print("\n");

      rcvData = Wire.read();
      Serial.print("address 68 reading = ");
      Serial.print(rcvData);
      Serial.print("\n");
    }


  }







//  //read CO2 sensors voltage across 240 ohm resistor
//  if (currentMillis - CO2_previousMillis >= CO2_interval) 
//  {
//    CO2_previousMillis = currentMillis;
//
//    //Read CO2 Sensor
//    int CO2_read = analogRead(A0);
//    float voltage = CO2_read * (5.0 / 1023.0);
//    Serial.print("CO2 = ");
//    Serial.print(voltage);
//    Serial.print("\n");
//    
//  }

    
//    //turn motor on for half of motor_interval and off for the other half
//    if(currentMillis - motor_previousMillis <= (motor_interval))
//    {
//      //turn motor OFF and solenoid 1
////      digitalWrite(motor,HIGH);
//      digitalWrite(sol_1,HIGH);
//    }
//    if(currentMillis - motor_previousMillis >= (motor_interval) && currentMillis - motor_previousMillis < (motor_interval * 2) )
//    {
//      //turn motor on and solenoid 1
//      digitalWrite(sol_1,LOW);
////      digitalWrite(motor,LOW);
//    }
//
//    if(currentMillis - motor_previousMillis >= (motor_interval*2)) {
//      motor_previousMillis = currentMillis;
//    }
    
}
