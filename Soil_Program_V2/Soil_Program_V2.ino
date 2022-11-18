#include <DS3231.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Defines Sea Level Pressure for BME280
#define SEALEVELPRESSURE_HPA (1013.25)

//Adds Definitions to the SD pin and File info for SD card
#define SD_SS_PIN 12
#define FILE_NAME "Datafile.txt"
#define LOGGERNAME "Mayfly Soil Vapor Sensor"  
#define DATA_HEADER "Date,Time,Depth Number,O2,CO2,Ambient Temperature C,Ambient Humidity %, Ambient Pressure hPa, Temperature C, Humidity%, Pressure hPa"

//Creates Clock Variable for RTC
DS3231 Clock;

//Sets RTC Parameters 
bool h24;
bool PM;
bool Century;
String sensor;
const int chipSelect = 10;
int second,minute,hour,month,DayMonth,year;

//Creates variables for different ADCs
   Adafruit_ADS1115 ads1; 
 //Adafruit_ADS1115 ads2; 

//Creates variables for the BME280s
  Adafruit_BME280 bme1;
  Adafruit_BME280 bme2;
  
//Setting variables for the ADC results 
 int16_t results1;
 int16_t results2;
 int16_t results3;
 float multiplier = 0.125F; 
 float O2_Volt;
 float CO2_Volt;

//Creates variables for BME280 results

float atemp;
float apressure;
float ahumidity;
float temp;
float pressure;
float humidity;

// Setting Valve pin
 int flushPin = 6;
 int pumpPin = 5;
 int pumpFS = 255;
 
 // Creates integer i and solenoid
int i;
int solenoid;

void setupLogFile() //Creates function to set up file of SD card
{
//Initialise the SD card and set creates file if not there
  if (!SD.begin(SD_SS_PIN))
  {
    Serial.println("Error: SD card failed to initialise or is missing.");
  }
 
 //Check if the file already exists
  bool oldFile = SD.exists(FILE_NAME);     

  //Open the file in write mode
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Add header information if the file did not already exist
  if (!oldFile)
  {
    logFile.println(LOGGERNAME);
    logFile.println(DATA_HEADER);
  }
  
  //Close the file to save it
  logFile.close();  
}

void logData(String rec) //Creates function to save data
{
  //Re-open the file
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Write the CSV data
  logFile.println(rec);

  //Close the file to save it
  logFile.close();
}
String createDataRecord()
{
  //Requests data from RTC
   year=Clock.getYear();
   hour=Clock.getHour(h24, PM);
   minute=Clock.getMinute();
   second=Clock.getSecond();
   month=Clock.getMonth(Century);
   DayMonth=Clock.getDate();
  
 //Create a String type data record in csv format
  String data = "";
  data += month;
  data += "/";
  data += DayMonth;
  data += "/";
  data += "20";
  data += year;      
  data += ",";      
  data += hour;
  data += ":";
  data += minute;
  data += ",";      
  data += solenoid;
  data += ",";      
  data += O2_Volt;
  data += ",";
  data += CO2_Volt;
  data += ",";
  data += atemp;
  data += ",";
  data += ahumidity;
  data += ",";
  data += apressure;
  data += ",";
  data += temp;
  data += ",";
  data += humidity;
  data += ",";
  data += pressure;
  return data;
}

void setup() {
 // put your setup code here, to run once:
  // Set Valve pin as outputs
    pinMode(flushPin, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
//Begins serial communication with 9600 Baud 
   Serial.begin(9600); 
 //Set the gain for each ADC 
   ads1.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   //ads2.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV  
 //Sets I2C location for ads1
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1);
  }
 //Sets I2C location for ads2 
  //if (!ads2.begin(0x49)) {
   // Serial.println("Failed to initialize ADS2.");
 //   while (1);
 // }
//Sets I2C Location for 0x76
if (!bme1.begin(0x76)) {
  Serial.println("Could not find a valid BME280 sensor, check wiring!");
  while (1);
}
//Sets I2C Location for 0x77
if (!bme2.begin(0x77)) {
  Serial.println("Could not find a valid BME280 sensor, check wiring!");
  while (1);
}
//Initialise log file 

  setupLogFile();

 //Echo the data header to the serial connection
  Serial.println(DATA_HEADER);

}

void loop() {
 

//___________Valve Control____________________________
 for (int i=7;i<=10;i++){

solenoid = i - 6;
             // Valve Opens

                        digitalWrite(i, HIGH);
                        delay(1000);

                        // Pump starts and runs for 20 s

                        digitalWrite(pumpPin, HIGH);
                        delay(2000);
                        
                        // Read results from the ADC
                        results1 = ads1.readADC_Differential_0_1();
                        results2 = ads1.readADC_Differential_2_3();
                        //results3 = ads2.readADC_Differential_0_1();

                        // Read results from the BME280
                        
                        atemp = bme1.readTemperature();
                        apressure = bme1.readPressure() / 100.0F;
                        ahumidity = bme1.readHumidity();
                        temp = bme2.readTemperature();
                        pressure = bme2.readPressure() / 100.0F;
                        humidity = bme2.readHumidity();
                        
                        // Pump turns off

                        digitalWrite(pumpPin, LOW);
                         delay(1000);

                        // Valve 1 closes

                        digitalWrite(i, LOW);
                        delay(1000);

                        // Valve 0 opens to flush out air

                        digitalWrite(flushPin, HIGH);
                        delay(1000);

                        // Pump runs at high flowrate for 5s

                        digitalWrite(pumpPin, HIGH);
                        delay(5000);

                        // Pump turns off

                         digitalWrite(pumpPin, LOW);
                         delay(1000);

                         // Valve 0 closes

                        digitalWrite(flushPin, LOW);
                        delay(1000);
                        
                        //Computes results from ADC
                                     
                        O2_Volt = results1 * multiplier / 1000;
                        CO2_Volt= results2 * multiplier / 1000;
                       
                        // Creates data string from function
                       
                        String dataRec = createDataRecord();
                       
                        //Save the data record to the log file
                        logData(dataRec);

                        //Echo the data to the serial connection
                        Serial.println(dataRec);
                        
                        
                        //Displays results in serial monitor
                        //Serial.print("Differential 1: "); Serial.print(results1); Serial.print("("); Serial.print(results1 * multiplier); Serial.println("mV)");
                        //Serial.print("Differential 2: "); Serial.print(results1); Serial.print("("); Serial.print(results2 * multiplier); Serial.println("mV)");
                        // Serial.print("Differential 3: "); Serial.print(results1); Serial.print("("); Serial.print(results3 * multiplier); Serial.println("mV)");
                        delay(1000);

 }
}
