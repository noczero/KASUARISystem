/**
  Bismillahirahmanirrahim
  TODO:
  - Pressure & Altitude : Done
  - Temperature & Humidity : 
  - GPS : Done
  - CO2 : Done
  - Yaw Pitch Roll : Not Yet
  - Camera : done -multiple shoot
 */


/*----------  ADT Library   ----------*/
#include "I2C.h"
#include <Wire.h>
#include "MS5611.h" //Pressure & Altitude
#include "SHT1x.h"
#include "TinyGPS.h"
#include "dht.h"
#include "CMPS10.h"
/*----------  End of ADT Library  ----------*/

/*----------  PIN Declaration  ----------*/
#define CO2Sensor A9
#define airSpeedPIN  A8
#define dataPinSHT11  2
#define clockPinSHT11 52 //SCK
#define DHT11 6
//#define buz 6
// camera Tx Rx serial1
// GPS Tx Rx serial2
// 3Dr Rx Rx serial 
/*----------  End of Pin Declaration  ----------*/

/*----------  Class Declaration  ----------*/
MS5611 pressure;
//SHT1x tempHumid(dataPinSHT11, clockPinSHT11);
TinyGPSPlus gps;
dht DHT;
CMPS10 compass;
/*----------  End of Class Declaration  ----------*/

/*----------  Variable Declaration  ----------*/
bool  mulai, 
      closeSerial,
      ambilFoto = false;

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
  Serial1.begin(38400); //Camera
  Serial2.begin(9600); //GPS Tx2 Rx2

  // camera initializtion

  Serial.println("Camera Initialize...");
  delay(100);
  SendResetCmd();
  delay(500);   
  SetImageSizeCmd(0X11); //ukuran gambar 
  delay(200);
  //SendResetCmd();
  //delay(3000);
  Serial.println("Camera Init Success...");
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
  Serial.print(compass.pitch()); 
  Serial.print(','); 
  Serial.print(compass.roll());
  Serial.print(','); 
  Serial.print(compass.bearing()); 
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

  // from the barometer
  // status = pressure.startTemperature(4);
  // if (status != 0)
  // {
  //   delay(status);
  //   status = pressure.getTemperature(temperature, 1);
  //   if (status != 0)
  //     Serial.print(temperature);
  // }
  int chk = DHT.read11(DHT11);
  switch (chk)
  {
    case DHTLIB_OK:  
      Serial.print(DHT.temperature , 1);
      Serial.print(",");
      Serial.print(DHT.humidity , 1); 
    break;
    case DHTLIB_ERROR_CHECKSUM: 
      Serial.print(0.0);
      Serial.print(",");
      Serial.print(0.0); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
      Serial.print(0.0);
      Serial.print(",");
      Serial.print(0.0); 
    break;
    default: 
      Serial.print(0.0);
      Serial.print(",");
      Serial.print(0.0);  
    break;
  }
  
}

void printTekanan(){
  Serial.print(realPressure);
}

void printArahAngin(){
  Serial.print("18");
}

void printKecAngin() {
    asVolts = analogRead(airSpeedPIN) * .0047;
//    Serial.print(asVolts);
//    Serial.print(':');
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

/*==============================
=            Camera            =
==============================*/
byte ZERO = 0x00;
byte incomingbyte;
byte dataCamera[32];
long int j=0,k=0,count=0,i=0x0000;
uint8_t MH,ML;
boolean EndFlag=0;
 
void SendResetCmd()
{
    Serial1.write(0x56);
    Serial1.write(ZERO);
    Serial1.write(0x26);
    Serial1.write(ZERO);
}

/*************************************/
/* Set ImageSize :
/* <1> 0x22 : 160*120
/* <2> 0x11 : 320*240
/* <3> 0x00 : 640*480
/* <4> 0x1D : 800*600
/* <5> 0x1C : 1024*768
/* <6> 0x1B : 1280*960
/* <7> 0x21 : 1600*1200
/************************************/
void SetImageSizeCmd(byte Size)

{
    Serial1.write(0x56);
    Serial1.write(ZERO);  
    Serial1.write(0x54);
    Serial1.write(0x01);
    Serial1.write(Size);
}

/*************************************/
/* Set BaudRate :
/* <1>¡¡0xAEC8    :   9600
/* <2>¡¡0x2AE4    :   38400
/* <3>¡¡0x1C4C    :   57600
/* <4>¡¡0x0DA6  :   115200
/* <5>¡¡0xAE    :   128000
/* <6>¡¡0x56    :   256000
/*************************************/
void SetBaudRateCmd(byte baudrate)
{
    Serial1.write(0x56);
    Serial1.write(ZERO);
    Serial1.write(0x24);
    Serial1.write(0x03);
    Serial1.write(0x01);
    Serial1.write(0x0D);
    Serial1.write(baudrate);
}
 
void SendTakePhotoCmd()
{
    Serial1.write(0x56);
    Serial1.write(ZERO);
    Serial1.write(0x36);
    Serial1.write(0x01);
    Serial1.write(ZERO); 

    i = 0x0000; //reset so that another picture can taken
}
 
void SendReadDataCmd()
{
    MH=i/0x100;
    ML=i%0x100;
    Serial1.write(0x56);
    Serial1.write(ZERO);
    Serial1.write(0x32);
    Serial1.write(0x0c);
    Serial1.write(ZERO);
    Serial1.write(0x0a);
    Serial1.write(ZERO);
    Serial1.write(ZERO);
    Serial1.write(MH);
    Serial1.write(ML);
    Serial1.write(ZERO);
    Serial1.write(ZERO);
    Serial1.write(ZERO);
    Serial1.write(0x20);
    Serial1.write(ZERO);
    Serial1.write(0x0a);
    i+=0x20;
}
 
void StopTakePhotoCmd()
{
    Serial1.write(0x56);
    Serial1.write(ZERO);
    Serial1.write(0x36);
    Serial1.write(0x01);
    Serial1.write(0x03);
}


void mainPhoto(){
      SendTakePhotoCmd();
      delay(100);
      while(Serial1.available()>0)
      {
          incomingbyte=Serial1.read();
      }
   
      while(!EndFlag)
      {
          j=0;
          k=0;
          count=0;
          //Serial1.flush();
          SendReadDataCmd();
          delay(20);
          while(Serial1.available()>0)
          {
              incomingbyte=Serial1.read();
              k++;
              delay(1);
              if((k>5)&&(j<32)&&(!EndFlag))
              {
                  dataCamera[j]=incomingbyte;
                  if((dataCamera[j-1]==0xFF)&&(dataCamera[j]==0xD9))     //tell if the picture is finished
                  {
                      EndFlag=1;
                  }
                  j++;
                  count++;
              }
          }
   
          for(j=0;j<count;j++)
          {
              if(dataCamera[j]<0x10)  Serial.print("0");
              Serial.print(dataCamera[j],HEX);           // observe the image through serial port
              //Serial.print(" ");
          }
          //Serial.println();
      }

      delay(100);
      ambilFoto = false;
      EndFlag = 0;
 
}

void printFoto(int i) {
    //boolean udahSelesai = true;
    //int i = 0;

    //while(udahSelesai){
      if((dataCamera[i-1]==0xFF)&&(dataCamera[i]==0xD9) && (i > 1)) {
        ambilFoto=false;
      } else {
        if(dataCamera[i]<0x10) Serial.print("0");
        
        Serial.print(dataCamera[i], HEX);

        if((i % 32) == 0) Serial.println();
      }
    //  i++;
    //}
    
}

/*=====  End of Camera  ======*/

//check altitude
boolean checkAltitudeToCapture(double tinggi){
  boolean take;
  int ketinggian = (int) tinggi;
  
  if (ketinggian % 10 > 5)
    take = false;

  if((ketinggian > 2000) && (ketinggian < 3000)){
      if ((ketinggian % 10 == 0) && (take == false)){
        take = true;
        return true;
      }
      else if ((ketinggian % 10 == 1) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 10 == 2) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 10 == 3) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 10 == 4) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 10 == 5) && (take == false)) {
        take = true;
        return true; 
      }
//      else if ((ketinggian % 10 == 6) && (take == false)) {
//        take = true;
//        return true; 
//      }
//      else if ((ketinggian % 10 == 7) && (take == false)) {
//        take = true;
//        return true; 
//      }
//      else if ((ketinggian % 10 == 8) && (take == false)) {
//        take = true;
//        return true; 
//      }
//      else if ((ketinggian % 10 == 9) && (take == false)) {
//        take = true;
//        return true; 
//      } 
  }

  return false;
}

int inc = 0;
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

      case '2' :
        //mainPhoto();
        ambilFoto = true;
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
    Serial.print(",");
    
    if(ambilFoto || checkAltitudeToCapture(relativeAltitude))
      mainPhoto();
//  if(ambilFoto )
//      mainPhoto();
    //camera
    // if(ambilFoto){
    //   printFoto(inc);
    //   inc++;
    // } else {
    //   inc = 0;
    //   Serial.println();
    // }
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








