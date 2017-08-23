/**
  Bismillahirahmanirrahim
  TODO:
  - Pressure & Altitude : Done
  - Temperature & Humidity : Done 
  - GPS : Done
  - CO2 : Done
  - Yaw Pitch Roll : Done
  - Camera : done -multiple shoot
  - Servo : Done

  -- Config Serial --
  - 3Dr Tx Rx serial > Baudrate : 57600
  - camera Tx Rx serial1
  - GPS Tx Rx serial2
 */


/*----------  ADT Library   ----------*/
#include "I2C.h"
#include <Wire.h>
#include "MS5611.h" //Pressure & Altitude
#include "TinyGPS.h"
#include "CMPS10.h"
#include "Adafruit_SHT31.h" // Temperature & Humidity
#include "TimerOne.h"
#include "SimpleTimer.h"
#include <Servo.h>

/*----------  End of ADT Library  ----------*/

/*----------  PIN Declaration  ----------*/
#define CO2Sensor A8
#define airSpeedPIN  A9
#define dataPinSHT11  2
#define clockPinSHT11 52 //SCK

/*----------  End of Pin Declaration  ----------*/

/*----------  Class Declaration  ----------*/
MS5611 pressure;
TinyGPSPlus gps;
CMPS10 compass;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SimpleTimer timer;
SimpleTimer timerAltitude;
Servo antennaServo;
/*----------  End of Class Declaration  ----------*/

/*----------  Variable Declaration  ----------*/
bool  mulai, 
      closeSerial,
      ambilFoto = false, 
      receiveHome = false ,
      cmdTakeFoto = false;

double referencePressure = 0.0, 
       relativeAltitude = 0.0, 
       realPressure = 0.0, 
       R = 6372795 , delLat , delLong , q , w, e, a, c, distance ,
       T = 0.0 ,
       P = 0.0 ;

float latHome, longiHome,
      voltage = 0.0, 
      CO2 = 0.0;

int voltage_diference = 0, 
    sensorValue = 0, 
    sum = 0, 
    offset = 0 , elevation = 0;

float humidity = 0.0 , 
      temperature = 0.0;

char status;


/*----------  End of Variable Declaration  ----------*/


void setup() {
  // put your setup code here, to run once:
  delay(100); // Wait for sensors to get ready
  
  Serial.begin(57600); // system baudrate 
  Serial1.begin(38400); //Camera
  Serial2.begin(9600); //GPS Tx2 Rx2

  // camera initializtion
  Serial.println("Camera Initialize...");
  delay(100);
  SendResetCmd();
  delay(500);   
  SetImageSizeCmd(0X11); //ukuran gambar 
  delay(200);  

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
  
  /*----------  SHT31 Setup  ----------*/
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31, check wiring");
    while (1) delay(1);
  } else {
    Serial.println("SHT31 init success");
    readTemperature();
  }

  analogReference(DEFAULT); //set the voltase of Analog PIN
  
  Serial.println("-- KASUARI Ready --");
  Serial.println(" type 1 to start...");
  Serial.println(" type 0 to stop...");
  
  /*=====  For Turn Servo  ======*/
  antennaServo.attach(46); // the pin 46
  antennaServo.write(178); // start vertical angle 

  /*===============================================
  =            Timer for Multi Tasking            =
  ===============================================*/

  timer.setInterval(1000, readTemperature); // temperature SHT31
 //timerAltitude.setInterval(1000, demoAltitude); // demo ketinggian 
   
  /*=====  End of Timer for Multi Tasking  ======*/
  
}

// function to get pressure from barometer
double getPressure() {
  status = pressure.startTemperature(4);
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T, 1);
    if (status != 0) {
      status = pressure.startPressure(4);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T, 1);
        if (status != 0) {
          return(P);
        }
      }
    }
  }
}

// procedure to print pitch roll yaw for sikap wahana
void printPitchRollYaw(){
  Serial.print(compass.pitch()); 
  Serial.print(','); 
  Serial.print(compass.roll());
  Serial.print(','); 
  Serial.print(compass.bearing()); 
}

boolean turun = false;
double increment = 5;

// procedure to print altitude
void printAltitude() {
 realPressure = getPressure(); // get the pressure
 relativeAltitude = pressure.altitude(realPressure,referencePressure); // comment this if use demo altitude 
 Serial.print(relativeAltitude);
}

// procedure to print altitude but only demo
void demoAltitude() {
   if (!turun) {
     relativeAltitude = relativeAltitude + increment;
   } else {
     relativeAltitude = relativeAltitude - increment;
   }

   if (relativeAltitude >= 9000) {
     turun = true;
   } else if (relativeAltitude <= 0) {
     turun = false;
   }   
}

// procedure to print temperature and humidity value that call before on timer 
void printTempHumidity() {
  if (!isnan(temperature) || !isnan(humidity)) {  // check if 'is not a number'
     Serial.print(temperature);
     Serial.print(",");
     Serial.print(humidity);
  } else {
     Serial.print(0.0);
     Serial.print(",");
     Serial.print(0.0);
  }
}

// procedure to give the temperature and humidty values with sht31 method
void readTemperature(){
  temperature = sht31.readTemperature();
  humidity = sht31.readHumidity();
}

// procedure to print pressure
void printTekanan(){
  Serial.print(realPressure);
}

// procedure to print lintang dan bujur (latitude and longitude) use tiny gps
void printLintangBujur() {
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6); //Latitude
  Serial.print(",");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6); //longitude
}

// procedure to print CO2 value with Senserion
void printCO2() {
  sensorValue = analogRead(CO2Sensor); 
  voltage = sensorValue*(5000/1024.0); 

  if(voltage == 0) {
    Serial.print("Fault");
  }
  else if(voltage < 400) {
    Serial.print(0); //still preheating
  }
  else {
    voltage_diference=voltage-400;
    CO2=voltage_diference*50.0/16.0;
    //Print CO2 concentration
    Serial.print(CO2);
  }
}

// procedure to print gps value in precision
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

// smart delay with milis in GPS
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
          delay(10);
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
   
          // print data use paralel scheme
          Serial.print("OK,");  // Header [0]
          printAltitude();      // ketinggian [1]
          Serial.print(",");
          printTempHumidity();  // temperature [2] tekanan [3]
          Serial.print(",");
          printTekanan();       // tekanan [4]
          Serial.print(",");
          printLintangBujur();  // GPS Lintang [7] dan Bujur [8]
          Serial.print(",");
          printCO2();           // print CO2 [9]
          Serial.print(",");
          printPitchRollYaw();  // print the pitch [10] roll [11] yaw [12].
          Serial.print(",");

          for(j=0;j<count;j++)
          {
              if(dataCamera[j]<0x10)  Serial.print("0");
              Serial.print(dataCamera[j],HEX);           // observe the image through serial port
          }

          Serial.println();
          timer.run(); // to change the value of temp and humid using timer
          //timerAltitude.run();
      }

      delay(50);
      StopTakePhotoCmd(); // must add to keep images continous shooting
      ambilFoto = false;
      cmdTakeFoto = false;
      EndFlag = 0;
 
}

/*=====  End of Camera  ======*/

// check altitude to capture image every 200 meters in 500 - 3000 meters 
boolean checkAltitudeToCapture(double tinggi){
  boolean take;
  int ketinggian = (int) tinggi;
  
  if (ketinggian % 200 > 10)
    take = false;

  if((ketinggian > 500) && (ketinggian < 3000)){
      if ((ketinggian % 200 == 0) && (take == false)){
        take = true;
        return true;
      }
      else if ((ketinggian % 200 == 1) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 200 == 2) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 200 == 3) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 200 == 4) && (take == false)) {
        take = true;
        return true; 
      }
      else if ((ketinggian % 200 == 5) && (take == false)) {
        take = true;
        return true; 
      }
     else if ((ketinggian % 200 == 6) && (take == false)) {
       take = true;
       return true; 
     }
     else if ((ketinggian % 200 == 7) && (take == false)) {
       take = true;
       return true; 
     }
     else if ((ketinggian % 200 == 8) && (take == false)) {
       take = true;
       return true; 
     }
     else if ((ketinggian % 200 == 9) && (take == false)) {
       take = true;
       return true; 
     } 
  }

  return false;
}

// procedure to print the data, fit the rules
void printAll(){
    // Start print the data
    // 0 1 2 3 4 5 6 7 8 9 10 11 
    // Header Ketinggian  Suhu  Humid  Tekanan  Lintang Bujur CO2 pitch roll yaw IMG 
    // split by ,
    
    Serial.print("OK,");  // Header [0]
    printAltitude();      // ketinggian [1]
    Serial.print(",");
    printTempHumidity();  // temperature [2] tekanan [3]
    Serial.print(",");
    printTekanan();       // tekanan [4]
    Serial.print(",");
    printLintangBujur();  // GPS Lintang [5] dan Bujur [6]
    Serial.print(",");
    printCO2();           // print CO2 [7]
    Serial.print(",");
    printPitchRollYaw();  // print the pitch [8] roll [9] yaw [10].
    Serial.print(",");
    Serial.print("IMG"); // 
}

// procedure to set the home, then use to calculate elevation
void setHome() {
  Serial.read();
	String latti  = Serial.readStringUntil(',');
	String longi  = Serial.readStringUntil('\n');
      
  latHome = latti.toFloat();
  longiHome = longi.toFloat();

  receiveHome = true;
  //Serial.print("SetHOME : ");    Serial.print(latHome);
  //Serial.print("----");
  //Serial.println(longiHome);
}

// function to calculate elevasi for condition to move the servo
int calculateElevation(double startLat , double startLong , double endLat , double endLong, double alt) {
	startLat = radians(startLat);
	startLong = radians(startLong);
	endLat = radians(endLat);
	endLong = radians(endLong);

	delLat = endLat - startLat;
	delLong = endLong - startLong;

	q = sin(delLat/2) * sin(delLat/2);
	w = cos(startLat)*cos(endLat);
	e = sin(delLong/2)*sin(delLong/2);
	a = (q + w * e);
	c = 2*atan2(sqrt(a),sqrt(1-a));
	distance = c * R;

	return degrees(atan(alt/distance));
}


// the main brain
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
        setHome(); //parsingnya 1 -6.976132,107.630332
        break;

      case '2' :
        cmdTakeFoto = true;
        break;

      default :
        break;
    }
  }

  // check if command "1" is execute and all the things start
  if (mulai && receiveHome) {
    printAll(); // print all the data
    if(cmdTakeFoto || checkAltitudeToCapture(relativeAltitude)){
      //check ambil foto still running or not
      if (!ambilFoto){
        Serial.println(); 
        mainPhoto();
      }
    }
    Serial.println();
  }

  // check gps is valid and coordinate home is received
  if (receiveHome && gps.location.isValid()) {
  	elevation = calculateElevation(latHome,longiHome, gps.location.lat(), gps.location.lng(), relativeAltitude);
  	
  	if ( elevation > 70 && elevation < 90) {
  		antennaServo.write(78);
  	} else {
  		antennaServo.write(178);
  	}
  }

  // Close Serial and stop all.
  if (closeSerial) {
    mulai = false;
    Serial.println("-- KASUARI Stopped --");
    Serial.end();
  }
    
  smartDelay(50); // must add for GPS data

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  timer.run();
  //timerAltitude.run(); //DEMO ALTITUDE
}