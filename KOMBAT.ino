/**
  Bismillahirahmanirrahim
  TODO:
  - Pressure & Altitude : Done
  - Temperature
  - GPS : Done
  - CO2 : Done
  - Yaw Pitch Roll : Not Yet
 */


/*----------  ADT Library   ----------*/
#include <I2Cdev.h>
#include <Wire.h>
#include "MS5611.h" //Pressure & Altitude
#include "SHT1x.h"
#include "TinyGPS.h"
/*----------  End of ADT Library  ----------*/

/*----------  PIN Declaration  ----------*/
#define CO2Sensor A14
#define airSpeedPIN  A15
#define dataPinSHT11  2
#define clockPinSHT11 52 //SCK
/*----------  End of Pin Declaration  ----------*/

/*----------  Class Declaration  ----------*/
MS5611 pressure;
SHT1x tempHumid(dataPinSHT11, clockPinSHT11);
TinyGPSPlus gps;
/*----------  End of Class Declaration  ----------*/

/*----------  Variable Declaration  ----------*/
bool  mulai, 
      closeSerial;

double referencePressure = 0.0, 
       relativeAltitude = 0.0, 
       realPressure = 0.0, 
       asOffsetV = 0.0 ,
       asVolts = 0.0,
       compVOut = 0.0,
       dynPress = 0.0,
       temperature = 0.0,
       airSpeed = 0.0;

float humidity = 0.0, 
      voltage = 0.0, 
      CO2 = 0.0;

int voltage_diference = 0, 
    sensorValue = 0, 
    sum = 0, 
    offset = 0;

char status;

/*----------  End of Variable Declaration  ----------*/


void setup() {
  // put your setup code here, to run once:

  delay(100); // Wait for sensors to get ready

  Serial.begin(115200);
  Serial2.begin(9600); //GPS Tx2 Rx2

  /*======================================
  =            Pressure Setup            =
  ======================================*/
   Serial.println("Initialize MS5611 Sensor");
   if (pressure.begin())
    Serial.println("MS5611 init success");
    else
    {// Something went wrong, this is usually a connection problem,
      // see the comments at the top of this sketch for the proper connections.

      Serial.println("MS5611 init fail (disconnected?)\n\n");
      while(1); // Pause forever.
    }
  referencePressure = getPressure(); // get pressure to calculate altitude
  /*=====  End of Pressure Setup  ======*/
  analogReference(DEFAULT); //set the voltase of Analog PIN
  asOffsetV = analogRead(airSpeedPIN) * .0047; //voltage offset from A15 (Airspeed)
  Serial.println("-- KASUARI Ready --");
  Serial.println(" type 1 to start...");
  Serial.println(" type 0 to stop...");
}

// get pressure
double getPressure() {
  double T,P;

  status = pressure.startTemperature(4);
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T, 1);
    if (status != 0)
    {
      status = pressure.startPressure(4);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P,T, 1);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void printPitchRollYaw(){
  Serial.print(0);
  Serial.print(','); 
  Serial.print(0); 
  Serial.print(','); 
  Serial.print(0); 
}

void printAltitude() {
  realPressure = getPressure();
  relativeAltitude = pressure.altitude(realPressure,referencePressure);
  Serial.print(relativeAltitude);
}

void printTempHumidity() {
  // temperature = tempHumid.readTemperatureC();
  // humidity = tempHumid.readHumidity();
  // Serial.print(temperature, DEC);

  status = pressure.startTemperature(4);
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(temperature, 1);
    if (status != 0)
      Serial.print(temperature);
  }
  Serial.print(",");
  Serial.print(0);
}

void printTekanan(){
  Serial.print(realPressure);
}

void printArahAngin(){
  Serial.print("0");
}

void printKecAngin() {
    asVolts = analogRead(airSpeedPIN) * .0047;
      compVOut = asVolts - asOffsetV;
    if(compVOut < .005)  {                    // Set noise to 0, min speed is ~8mph
      compVOut = 0.0;
    }  
    dynPress = compVOut * 1000.0;     // With autozero, dynamic pressure in kPa = Vout, convert kPa to P
    airSpeed = sqrt((2 * dynPress)/1.225);   // Converts pressure to m/s, 1.225 k/m3 is standard air density
    Serial.print(airSpeed);
}

void printLintangBujur() {
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6); //Latitude
  Serial.print(",");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6); //longitude
}

void printCO2() {
  sensorValue = analogRead(CO2Sensor); 
  voltage = sensorValue*(5000/1024.0); 
  
  if(voltage == 0)
  {
    Serial.print("Fault");
  }
  else if(voltage < 400)
  {
    Serial.print(0); //still preheating
  }
  else
  {
    voltage_diference=voltage-400;
    CO2=voltage_diference*50.0/16.0;
    //Print CO2 concentration
    Serial.print(CO2);
  }
}

//for GPS lat long
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
  }
  smartDelay(0);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0 ) {
    char command = (char) Serial.read();

    switch (command) {
      case '0' :
        closeSerial = true;
        break;

      case '1' : 
        mulai = true;
        closeSerial = false;
        break;

      default :
        break;
    }
  }

  // Start
  if (mulai) {
    //Start print the data
    // 0 1 2 3 4 5 6 7 8 9 10 11 12 13
    // Header Ketinggian  Suhu  Humid  Tekanan  Arah-Angin  Kec-Angin  Lintang   Bujur       CO2 
    // split by ,
    Serial.print("OK,");  // Header [0]
    printAltitude();      // ketinggian [1]
    Serial.print(",");
    printTempHumidity();  // temperature [2] tekanan [3]
    Serial.print(",");
    printTekanan();       // tekanan [4]
    Serial.print(",");
    printArahAngin();     // Arah Angin [5]
    Serial.print(",");
    printKecAngin();      // Kecepatan Angin [6]
    Serial.print(",");
    printLintangBujur();  // GPS Lintang [7] dan Bujur [8]
    Serial.print(",");
    printCO2();           // print CO2 [9]
    Serial.print(",");
    printPitchRollYaw();  // print the pitch [10] roll [11] yaw [12].
    Serial.println();
  }

  // Close Serial and stop all.
  if (closeSerial) {
    mulai = false;
    Serial.println("-- KASUARI Stopped --");
    Serial.end();
  }
  smartDelay(100); // must add for GPS data

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
