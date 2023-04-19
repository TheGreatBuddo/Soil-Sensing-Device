//Include libraries for the Arduino and I2C
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

//Includes the Library for the Analog Digital Converters
#include <Adafruit_ADS1X15.h>

//Includes the libraries for the BME280 sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Includes the library for the SD card
#include <SD.h>

//Includes the Library for the Real Time Clock
#include <DS3231.h>

//Defines Sea Level Pressure for BME280
#define SEALEVELPRESSURE_HPA (1013.25)

//Adds Definitions to the SD pin and File info for SD card
#define SD_SS_PIN 12
#define FILE_NAME "Datafile.txt"
#define LOGGERNAME "Mayfly Soil Vapor Sensor"
#define DATA_HEADER "Date,Time,Depth Number,O2(%),CO2(PPM),CH4(V),Ambient Temperature (C),Ambient Humidity (%), Ambient Pressure (hPa), Temperature (C), Humidity (%), Pressure (hPa)"

//define MQ4 Methane Sensor I2C address
#define Addr 0x50

//Creates Clock Variable for RTC
DS3231 Clock;

//Sets RTC Parameters
bool h24;
bool PM;
bool Century;
String sensor;
const int chipSelect = 10;
int second, minute, hour, month, DayMonth, year;

//Creates variables for different ADCs
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

//Creates variables for the BME280s
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;

//Setting variables for the ADC results
int16_t results1;
int16_t results2;
int16_t results3;
int16_t results4;
float multiplier = 0.125F;
float O2_Volt;
float CO2_Volt;
float Battery_Lvl;
float Methane_Volt;
float O2_Percentage;
float CO2_PPM;
float Methane_PPM;
float accuracy_modifer;
float O2_Actual_Percentage;

//Creates variables for BME280 results

float atemp;
float apressure;
float ahumidity;
float temp;
float pressure;
float humidity;



int solenoid;
int k;
// Setting Valve pin
int flushPin = 11;

// Mayfly pins connected to control respective solenoids relays.
int solenoid_pins[] = { 23, 7, 9, 10 };

// Pump
int dir1PinA = 8;
int dir2PinA = 6;
int speedPinA = 4;  // Needs to be a PWM pin to be able to control motor speed

// Heating Element
int dir1PinB = 18;
int dir2PinB = 20;
int speedPinB = 5;  // Needs to be a PWM pin to be able to control motor speed

//Methane Calculations
float Rs;
float Ro = 23250;  // ohms
float Rl = 3170;
float Vdd = 5;
float Vload;
float ratio_corr;
float CH4_raw;
float CH4_log;
float CH4_ratio;





void setup() {

  Wire.begin();
  // Set OUTPUT Pins
  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(flushPin, OUTPUT);
  pinMode(solenoid_pins[0], OUTPUT);
  pinMode(solenoid_pins[1], OUTPUT);
  pinMode(solenoid_pins[2], OUTPUT);
  pinMode(solenoid_pins[3], OUTPUT);

  //HIGH implies off for relay board
  digitalWrite(flushPin, HIGH);
  digitalWrite(solenoid_pins[0], HIGH);
  digitalWrite(solenoid_pins[1], HIGH);
  digitalWrite(solenoid_pins[2], HIGH);
  digitalWrite(solenoid_pins[3], HIGH);

  //Begins serial communication with 9600 Baud
  Serial.begin(9600);

  //Set the gain for each ADC
  ads1.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //ads2.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads2.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //Sets I2C location for ads1

  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1)
      ;
  }


  //Sets I2C location for ads2 external ADC
  if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize ADS2.");
    while (1)
      ;
  }

  //Sets BME1 I2C Location to 0x76
  if (!bme1.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  //Sets BME1 I2C Location to 0x77
  if (!bme2.begin(0x77)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  //Initialise log file
  setupLogFile();

  //Echo the data header to the serial connection
  //Serial.println(DATA_HEADER);
  analogWrite(speedPinA, 1);
}

void loop() {


  //Heating Element ON
  analogWrite(speedPinB, 50);  //Sets speed variable via PWM
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);

  //Force Reset of for loop because unknown reason k++ increased infinitely
  k = 0;
  //___________Valve Control____________________________
  for (k = 0; k < 4; k++) {
    //Only use solenoid 1
     k = 0;
    int active_solpin = solenoid_pins[k];
    //add a list of solenoid pin numbers and go through them to activate solenoid!
    //Careful now! We cannot use pin eight because it triggers relay when restarting

    digitalWrite(active_solpin, LOW);
    delay(1000);

    //Pump ON
    analogWrite(speedPinA, 255);  //Sets speed variable via PWM
    digitalWrite(dir1PinA, LOW);
    digitalWrite(dir2PinA, HIGH);
    delay(5000);
    //Pump OFF
    analogWrite(speedPinA, 0);  //Sets speed variable via PWM
    digitalWrite(active_solpin, HIGH);

    // // Read results from the ADCs
    results1 = ads1.readADC_Differential_0_1();
    results2 = ads1.readADC_Differential_2_3();
    results3 = ads2.readADC_SingleEnded(0);
    results4 = ads2.readADC_SingleEnded(1);
    //Read results from the BME280
    atemp = bme1.readTemperature();
    apressure = bme1.readPressure() / 100.0F;
    ahumidity = bme1.readHumidity();
    temp = bme2.readTemperature();
    pressure = bme2.readPressure() / 100.0F;
    humidity = bme2.readHumidity();


    //Computes results from ADC
    O2_Volt = results1 * multiplier / 1000;
    CO2_Volt = results2 * multiplier / 1000;
    Battery_Lvl = results3 * multiplier / 1000;
    //Methane_Volt = results4 * multiplier / 1000;
    Methane_Volt = results4 * multiplier / 1000;
    //Converts Voltage to Percent O2
    O2_Percentage = (((0.0733 * abs(O2_Volt)) - 0.0813) * 100);

    //Error Reading if negative value
    if (O2_Percentage <= 0) {
      O2_Percentage = -999;
    }

    //Converts Volts CO2 to PPM
    CO2_PPM = (1671.4 * abs(CO2_Volt) - 1287.3);

    //Error Reading if negative value
    if (CO2_PPM <= 0) {
      CO2_PPM = -999;
    }

    //Converts Volts CH4 to PPM
    // Methane_PPM = 8.2318 * pow(2.71828, (2.7182 * Methane_Volt));

    //Error Reading if negative value
    if (Methane_PPM <= 0) {
      Methane_PPM = -999;
    }

    //Methane Calculations
    //varibles at disposal
    // Rs; Ro = 3163 ohms; Rl = 3170; Vdd = 5; Vload;
    Vload = Methane_Volt;
    Rs = Rl * ((Vdd / Vload) - 1);
    // ratio_corr = (Rs / Ro) * (0.024 + 0.0072 * humidity + 0.246 * temp);
    // CH4_raw = 1.828 + 0.0288 * ratio_corr;
    CH4_ratio = Rs / Ro;
    CH4_log = 465265 * exp(-12.04 * CH4_ratio);

    //Serial.print("Battery Voltage = ");
    Battery_Lvl = ((Battery_Lvl * (9862.939 + 99725.277)) / 9862.939);
    //Serial.println(Battery_Lvl);

    // Creates data string from function
    String dataRec = createDataRecord();

    //Creates Barrier for serial monitor
    Serial.println("-----------------------------------------------------------------------------");

    //Informs Solenoid used
    Serial.print("Solenoid ");
    Serial.print(k + 1);
    Serial.print("| ");

    // Serial.print("Methane = ");
    // Serial.print(Methane_PPM);
    // Serial.print("PPM, ");

    Serial.print("Water = ");
    Serial.print(analogRead(A0));
    Serial.print(" Moisture| ");

    Serial.print("CO2 = ");
    Serial.print(CO2_PPM);
    Serial.print("PPM, ");

    Serial.print("O2% = ");
    Serial.print(O2_Percentage);
    Serial.print("%| ");

    // Serial.print("Vload = ");
    // Serial.print(Vload);
    // Serial.print("V| ");

    // Serial.print("Rs = ");
    // Serial.print(Rs);
    // Serial.print("ohms| ");

    Serial.print("CH4 PPM = ");
    Serial.print(CH4_log);
    Serial.print("PPM| ");

    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.print("C| ");

    Serial.print("RH = ");
    Serial.print(humidity);
    Serial.print("%| ");


    Serial.print("\n");

    Serial.print("Ambient Temperature = ");
    Serial.print(atemp);
    Serial.print("C| ");

    Serial.print(" Ambient RH = ");
    Serial.print(ahumidity);
    Serial.print("%| ");

    Serial.print("\n");
    Serial.println("-----------------------------------------------------------------------------");
    //End Barrier for serial monitor


    //Save the data record to the log file
    logData(dataRec);

    delay(1000);
  }
}

String createDataRecord() {
  //Requests data from RTC
  year = Clock.getYear();
  hour = Clock.getHour(h24, PM);
  minute = Clock.getMinute();
  second = Clock.getSecond();
  month = Clock.getMonth(Century);
  DayMonth = Clock.getDate();

  //Create a String type data record in csv format seperated by commas
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
  data += O2_Percentage;
  data += ",";
  data += CO2_PPM;
  data += ",";
  data += Methane_Volt;
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

//Initialise the SD card and creates file if not there
void setupLogFile() {
  //Checks if SD Card is missing
  if (!SD.begin(SD_SS_PIN)) {
    Serial.println("Error: SD card failed to initialise or is missing.");
  }

  //Check if the file already exists
  bool oldFile = SD.exists(FILE_NAME);

  //Open the file in write mode
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Add header information if the file did not already exist
  if (!oldFile) {
    logFile.println(LOGGERNAME);
    logFile.println(DATA_HEADER);
  }

  //Close the file to save it
  logFile.close();
}

//Creates function to save data in a string
void logData(String rec) {
  //Re-open the file
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Write the CSV data
  logFile.println(rec);

  //Close the file to save it
  logFile.close();
}
