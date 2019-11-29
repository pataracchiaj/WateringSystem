//Mr. Pataracchia's branch
//Version 3.0.1
//Initial github commit: 29 Nov 19
#include <Blynk.h>
//Defining the libraries used for the project.
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHTesp.h>;
#include <C:\Users\CSE\Documents\Arduino\libraries\connection_info\connection_info.h>
const int TH_Sensor_Pin = 5;      // Define pin number to which the sensor is connected
DHTesp DHT;                       // Create a DHT object
BlynkTimer timer;
const byte MANUAL = 0;            //Define the two states the device can be in.
const byte AUTO = 1;
const byte pumpPin = 2;           //The pin our pump is attached to.
const byte WATER_Button = 12;     //The pin our manual water button is attached to.

const byte soilMoisturePin = A0;  //Constant for the soil moisture sensor pin (labeled 'ADC' on the esp8266)

//Rather than powering the sensor through the 3.3V or 5V pins,
//we'll use a digital pin to power the sensor. This will
//prevent corrosion of the sensor as it sits in the soil.

const byte moistureSensorPower = 14;  //Constant for Soil moisture Power pin

int moduleState = AUTO;               //Default our device to automatic mode, in case of forced restart.
bool debounce = false;                //Set the device so it isn't debouncing its button by default.
unsigned long currentTime;
unsigned long previousTime;
unsigned long timeSinceReported;      //Declare a variable to keep track of time between Blynk updates.
unsigned long timeSinceSMReport;      //Declare a variable to keep track of time between serial monitor updates.

//WiFi credentials:
//char ssid[] = ""; these lines are located in an external
//char pass[] = ""; library to prevent unauthorized access
//char auth[] = ""; to the wireless network.

int lastConnectionAttempt = millis();
const int connectionDelay = 5000;  // try to reconnect every 5 seconds

void setup()
{ //Set up the serial monitor, prepare the pins and initialize key variables.
  Serial.begin(9600);
  Serial.println();
  Serial.print("Connected");
  DHT.setup(TH_Sensor_Pin, DHTesp::DHT22);
  pinMode(pumpPin, OUTPUT);
  pinMode(WATER_Button, INPUT);
  Blynk.begin(auth, ssid, pass);
  previousTime = 0;
  timeSinceReported = millis();
  timeSinceSMReport = millis();
  pinMode(moistureSensorPower, OUTPUT);      //Configure moistureSensorPower pin for OUTPUT
  digitalWrite(pumpPin, LOW);                //Ensure water valve is closed on startup
  digitalWrite(moistureSensorPower, LOW);    //Disable the moisture sensor on startup
}

BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);
  Blynk.syncVirtual(V5);
  Blynk.syncVirtual(V6);
}

BLYNK_WRITE(V3)
{ //Report to the serial monitor when the device is digitally switched to MANUAL or AUTO.
  moduleState = param.asInt();
  if (moduleState == AUTO) {
    Serial.println("AUTO MODE");
  }
  else  {
    Serial.println("MANUAL MODE");

  }
}

BLYNK_WRITE(V4)
{ //Activate the pump when a digital button on the BLYNK app is pressed.
  if ((moduleState == 0) && (debounce == false)) {
    debounce = true;  //Ignore button presses until current watering session is complete.
    manualWater();
  }
}

void loop()
{
  checkWiFiconnection();
  timer.run();                                     //Initiate Blynk and the BlynkTimer.
  currentTime = millis();                          //Get the current time in milliseconds.

  if (currentTime - timeSinceReported >= 20000) {  //If twenty or more seconds have passed since the last report...
    reportSensorValues();                          //Report the current sensor values to the BLYNK app.
    timeSinceReported = currentTime;               //Record the current time to be used for next Blynk report.
  }

  if ((digitalRead(WATER_Button) == HIGH) && (debounce == false)) {//Physical button to activate watering.
    manualWater();
    debounce = true;                    //Ignore button presses until current watering session is complete.

  }

  if (moduleState == AUTO) { //WIP functionality for when AUTO mode is active.
    autoWater();
  }

  if (currentTime - previousTime >= 3000) { //Resets pump and button after 3 second MANUAL water.
    digitalWrite(pumpPin, LOW);
    debounce = false;                       // Re-enable watering buttons (both physical & virtual).
  }
}

void autoWater() { //WIP- Function to be used when device is in AUTO mode.
  //to do
}
void manualWater() { //Function that is called when device is in MANUAL mode that waters it for 3 seconds.
  Serial.println("Manually activating pump for 3 seconds.");
  previousTime = millis();
  digitalWrite(pumpPin, HIGH);
}

float getTemp() {//Read and return current temperature and send result to the serial monitor.
  float t = DHT.getTemperature();
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.print(" *C ");

  return t;
}
float getHum() {//Read and return current humidity and send result to the serial monitor.
  float h = DHT.getHumidity();
  Serial.print("    Humidity = ");
  Serial.print(h);
  Serial.println(" % ");
  return h;
}
float getSoilMoisture(){//Read and return current soil moisture and send result to the serial monitor.
  Serial.print("Soil Moisture = ");
  //Get soil moisture value from the function below and print it
  int val  = map(readSoil(), 51, 1023, 0, 100);
  Serial.print(readSoil());
  Serial.print(" : ");
  float soilMoistureValue  = float(val);
  Serial.println(soilMoistureValue);
  return soilMoistureValue;
}

float reportSensorValues()
{ //Write the current temperature, humidity, and moisture to the BLYNK app.
  Blynk.virtualWrite(V0, getTemp());
  Blynk.virtualWrite(V1, getHum());
  Blynk.virtualWrite(V2, getSoilMoisture());
}

int readSoil()
{
  digitalWrite(moistureSensorPower, HIGH);             //turn the moisture sensor "On"
  delay(10);                                           //wait 10 ms for sensor readings to stabilize.
  int soilMoistureValue = analogRead(soilMoisturePin); //Read the raw moisture value from the sensor
  digitalWrite(moistureSensorPower, LOW);              //turn the moisture sensor "Off"
  return soilMoistureValue;                            //send current moisture value
}

void checkWiFiconnection()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    // (optional) "offline" part of code

    // check delay:
    if (millis() - lastConnectionAttempt >= connectionDelay)
    {
      lastConnectionAttempt = millis();

      // attempt to connect to Wifi network:
      if (pass && strlen(pass))
      {
        WiFi.begin((char*)ssid, (char*)pass);
      }
      else
      {
        WiFi.begin((char*)ssid);
      }
    }
  }
  else
  {
    Blynk.run();
  }
}
