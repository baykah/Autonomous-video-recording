#include <TinyGPS++.h>
#include <SPI.h>
#include <RFM69.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ssd1306syp.h>

TinyGPSPlus gps;
RFM69 radio;

//radio
#define NODEID      1
#define NETWORKID   100
#define FREQUENCY   RF69_868MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define KEY         "thisIsEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define LED         9
#define SERIAL_BAUD 115200
#define ACK_TIME    30  // # of ms to wait for an ack

//Ecran
#define SDA_PIN 20
#define SCL_PIN 21
Adafruit_ssd1306syp display(5,4);

//Servo
double pos_rotation = 0;
double vit_rotation = 10;
double pos_hauteur = 0;
double vit_hauteur = 10;
int pos_rotation_min = 50;
int pos_rotation_max = 140;
int pos_hauteur_min = 90;
int pos_hauteur_max = 110;

//GPS
double Distance, Altitude, Altitude_filt, CourseTo, CourseTo_corrected = 0, Altitude_correction = 0;
double flat_avion_filt, flon_avion_filt, pressure_avion_filt, fspeed_avion_filt;
double flat_base_filt, flon_base_filt, pressure_base_filt, fspeed_base_filt;
double k_alt = 1, k_base = 1, k_avion = 1, filter_GPS_base = 1;
double klat, klon;
int CourseTo_correction = 0;
#define CCPIN 12 // Defines pin 12 as course correction
int tracking = 0;
unsigned long time = 0;
unsigned long time_fix_base = 0, time_fix_avion = 0;;
unsigned long tracking_start = 0;

//VARIO
unsigned int calibrationData[7];
#define ACPIN 11 // Defines pin 11 as alt correction

//LED
#define RED 10 // Defines pin 3 as RED
#define BLUE 8 // Defines pin 5 as BLUE
#define GREEN 9 // Defines pin 6 as GREEN
int compt_led = 1;

//SD

//Caméra max = 60
int zoom_enable = 0, zoom = 0, zoom_max = 10, zoom_consigne = 0, zoom_max_change = 10, Distance_min = 50;

//Global
int Serial_on = 3;
int Ecran_on = 0;
unsigned long time_start, time_led = 0;
#define VIT_ROT_PIN 07 // Defines pin 07 as VIT_ROT_PIN
#define ZOOM_PIN 13 // Defines pin 13 as ZOOM_PIN
int V_max = 2, V_change = 0;

typedef struct
{
   double flat = 0;
   double flon = 0;
   double pressure = 0;
   double fspeed = 0;
   int GPS = 0;
   int Correction =0;
} data_radio;

  data_radio data_base;
  data_radio data_avion;


void setup()
{
//Serial 
 if (Serial_on != 0) Serial.begin(115200);
 Serial1.begin(38400);
 Serial2.begin(115200);
 Serial3.begin(115200);
 
//Pressure
Wire.begin();
setupSensor();

//Ecran
display.initialize();
display.clear();
display.setTextSize(2);
display.setCursor(0,0);
display.println("Startup");
display.update();
// RFM69
radio.initialize(FREQUENCY,NODEID,NETWORKID);
radio.setHighPower(); //uncomment only for RFM69HW!
radio.setPowerLevel(16);
radio.encrypt(KEY);
//LED 
pinMode (RED, OUTPUT);
pinMode (BLUE, OUTPUT);
pinMode (GREEN, OUTPUT);
pinMode (CCPIN, INPUT);
pinMode (ACPIN, INPUT);
pinMode (VIT_ROT_PIN, INPUT);
pinMode (ZOOM_PIN, INPUT);
/*
//servo
display.clear();
display.setCursor(0,0);
display.println("-90 deg");
display.update();
delay(1000);
//Serial2.print("<050255080255>");
update_servo(50,10,80,255);
delay(5000);
display.clear();
display.setCursor(0,0);
display.println("+90 deg");
display.update();
//Serial2.print("<140255110255>");
update_servo(140,10,110,255);
delay(5000);
display.clear();
display.setCursor(0,0);
display.println("0 deg");
display.update();
//Serial2.print("<095255087255>");
update_servo(95,10,87,255);
delay(5000);*/
update_servo(95,255,pos_hauteur_min,255);

}

void loop()
{

if (Serial_on != 0) Serial.println(millis() - time_start);
time_start = millis(); 
 
//GPS AVION
uint8_t len = sizeof(data_avion);
int a = 0;
while ( (millis() - time_start < 500) && ( a == 0 ) )
{
  if ( Serial3.available() > 0 )
    {
      if ( gps.encode(Serial3.read()) )
      {
         if ( gps.location.isValid() )  
         {
            time_fix_base = millis();
            data_base.GPS = 1;
            
            if ( filter_GPS_base == 1 )
            {
            data_base.flat = LP_lat_base(data_base.flat, gps.location.lat() );
            data_base.flon = LP_lon_base(data_base.flon, gps.location.lng() );
            }
            else
            {
            data_base.flat = gps.location.lat();
            data_base.flon = gps.location.lng();              
            }
            
            if (Serial_on == 3)
            {   
              
               Serial.print("Gba&");  
               Serial.print(gps.location.lat(),7);
               Serial.print("&");  
               Serial.print(gps.location.lng(),7);
               Serial.println("&");  
            }
         }
      }
   }
  
  
  a = radio.receiveDone();
}

if ( ( millis() - time_fix_base ) > 5000 )
{
  data_base.GPS = 0;
}
  
if (a)
  {
    data_avion = *(data_radio*)radio.DATA;
    if ( data_avion.GPS == 1) {
            if (Serial_on == 3)
            {   
               Serial.print("Gav&");  
               Serial.print(data_avion.flat,7);
               Serial.print("&");  
               Serial.print(data_avion.flon,7);
               Serial.println("&");  
            }
    }  
  }
  else
  {
    data_avion.GPS = 2;
  }

data_base.pressure = getPressure();
if (Serial_on == 2) Serial.println(data_base.pressure);
if (Serial_on == 2) Serial.println(data_avion.pressure); 
 
   
if (data_avion.GPS == 2)
{ 
analogWrite (GREEN, 255); // Sends the "green" signal to the green color terminal of the LED
analogWrite (BLUE, 255); // Sends the "blue" signal to the blue color terminal of the LED
analogWrite (RED, 0); // Sends the "red" signal to the red color terminal of the LED

display.clear();
display.setTextSize(1);
display.setCursor(0,0);
display.println("Pas de reception Avion");
display.update();
}
else if (data_avion.GPS == 0)
{
analogWrite (GREEN, 0); // Sends the "green" signal to the green color terminal of the LED
analogWrite (BLUE, 100); // Sends the "blue" signal to the blue color terminal of the LED
analogWrite (RED, 255); // Sends the "red" signal to the red color terminal of the LED  

display.clear();
display.setTextSize(1);
display.setCursor(0,0);
display.println("Attente GPS Avion");
display.update();
}
else if (data_base.GPS == 0)
{
analogWrite (GREEN, 255); // Sends the "green" signal to the green color terminal of the LED
analogWrite (BLUE, 255); // Sends the "blue" signal to the blue color terminal of the LED
analogWrite (RED, 0); // Sends the "red" signal to the red color terminal of the LED  

display.clear();
display.setTextSize(1);
display.setCursor(0,0);
display.println("Pas de GPS base");
display.update();

data_base.pressure = 0;
} 
else if (data_avion.GPS == 1 && data_base.GPS == 1)
{
  
if ( tracking == 0 ) {
analogWrite (GREEN, 0); // Sends the "green" signal to the green color terminal of the LED
analogWrite (BLUE, 255); // Sends the "blue" signal to the blue color terminal of the LED
analogWrite (RED, 255); // Sends the "red" signal to the red color terminal of the LED  
display.clear();
display.setTextSize(2);
display.setCursor(0,0);
display.println("Ready");
display.println(gps.hdop.value());
k_base = 1;
display.update();
} else {
k_base = 0.01;  
}

fspeed_avion_filt = data_avion.fspeed;

Distance = TinyGPSPlus::distanceBetween(data_base.flat, data_base.flon, data_avion.flat, data_avion.flon);
if ( Distance > 2000 ) Distance = 2000;
CourseTo = TinyGPSPlus::courseTo(data_base.flat,  data_base.flon, data_avion.flat, data_avion.flon);

  
  
  
if ( digitalRead(CCPIN) || data_avion.Correction == 1 ) {
  CourseTo_corrected = CourseTo; 
  if ( tracking == 0 ) {
  tracking = 1;
  display.clear();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Tracking");
  display.update();
  Serial1.print("<r>");   //send command start/stop  recording to mini pro IR
  delay(500);
  } else {
  tracking = 0;
  Serial1.print("<s>");   //send command start/stop recording to mini pro IR
  update_servo(95,255,pos_hauteur_min,255);
  zoom = 0;
  }
}

//pressure_base_filt = LP_pressure_base( pressure_base_filt , data_base.pressure );
//pressure_avion_filt = LP_pressure_avion( pressure_avion_filt , data_avion.pressure );
//Altitude = (pressure_base_filt - pressure_avion_filt) / 100 * 9.144;
Altitude = (data_base.pressure - data_avion.pressure) / 100 * 9.144;
Altitude_filt = LP_Altitude( Altitude_filt , Altitude );

if ( digitalRead(ACPIN) && tracking == 0 ) {
  Altitude_correction = Altitude;
  display.clear();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Altitude correction :");
  display.print(Altitude_correction);
  display.println(" m");
  display.update();
  delay(1000);
}

Altitude = Altitude - Altitude_correction;

if ( digitalRead(VIT_ROT_PIN) == LOW && tracking == 0 ) {
  V_change = 3;
  if ( V_change != V_max ) {
    V_max = V_change;
    display.clear();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Vitesse correction :");
    display.println(V_max);
    display.update();
    delay(1000);
    }
} else if ( digitalRead(VIT_ROT_PIN) == HIGH && tracking == 0 ) {
  V_change = 6;
  if ( V_change != V_max ) {
    V_max = V_change;
    display.clear();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Vitesse correction :");
    display.println(V_max);
    display.update();
    delay(1000);
    }
}

if ( digitalRead(ZOOM_PIN) == LOW && tracking == 0 ) {
  zoom_max_change = 5;
  if ( zoom_max_change != zoom_max ) {
    zoom_enable = 1;
    zoom_max = zoom_max_change;
    display.clear();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Zoom 1 :");
    display.println("Zoom max :");
    display.println(zoom_max);
    display.update();
    delay(1000);
    }
} else if ( digitalRead(ZOOM_PIN) == HIGH && tracking == 0 ) {
  zoom_max_change = 10;
  if ( zoom_max_change != zoom_max ) {
    zoom_max = zoom_max_change;
    display.clear();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Zoom max :");
    display.println(zoom_max);
    display.update();
    delay(1000);
    }
}

if ( tracking == 1 ) {

  if ( (millis() - time_led) > 500) {
    time_led = millis();
    if ( compt_led == 1 ) {
      analogWrite (GREEN, 255); // Sends the "green" signal to the green color terminal of the LED
      analogWrite (BLUE, 255); // Sends the "blue" signal to the blue color terminal of the LED
      analogWrite (RED, 255); // Sends the "red" signal to the red color terminal of the LED  
      compt_led = 0;
    } else {
      analogWrite (GREEN, 0); // Sends the "green" signal to the green color terminal of the LED
      analogWrite (BLUE, 255); // Sends the "blue" signal to the blue color terminal of the LED
      analogWrite (RED, 255); // Sends the "red" signal to the red color terminal of the LED   
      compt_led = 1; 
    }

  }


  
  if ( (CourseTo - CourseTo_corrected) < 0 ) {
  CourseTo = CourseTo - CourseTo_corrected + 360;
  } else {
  CourseTo = CourseTo - CourseTo_corrected;
  }
  
  //CAMERA_Pan
 if (CourseTo >= 270) {
 pos_rotation = CourseTo - 360;
 } else if (CourseTo >= 0 && CourseTo <= 90) {
 pos_rotation = CourseTo;
 } else if (CourseTo > 90 && CourseTo < 180) {
 pos_rotation = 90;
 } else if (CourseTo >= 180 && CourseTo < 270) {
 pos_rotation = -90;
 }
   
 //CAMERA_Tilt
 pos_hauteur = atan( ( Altitude / Distance ) ) * 57.295779;
 if ( (pos_hauteur <= 90) && (pos_hauteur >= 0) )
 {
 pos_hauteur = pos_hauteur;
 }
 else if (pos_hauteur > 90)
 {
 pos_hauteur = 90;
 }
 else if (pos_hauteur < 0)
 {
 pos_hauteur = 0;
 } 


pos_rotation = map(pos_rotation, -90, 90, pos_rotation_min, pos_rotation_max);
vit_rotation = fspeed_avion_filt / Distance * 50;
if (vit_rotation > 50) vit_rotation = 50;
vit_rotation = map(vit_rotation, 0, 50, 1, V_max); //5 30
pos_hauteur = map(pos_hauteur, 0, 90, pos_hauteur_min, pos_hauteur_max);
vit_hauteur = 10;

//update_servo(pos_rotation,255,pos_hauteur,255);
update_servo(pos_rotation,vit_rotation,pos_hauteur,vit_hauteur);

if ( zoom_enable == 1) {
  if ( Distance > Distance_min && fspeed_avion_filt < 100 ) {
    zoom_consigne = map(Distance, Distance_min , 1000, 0, zoom_max);
  } else {
    zoom_consigne = 0;
  }
  if (zoom_consigne > zoom) Serial1.print("<+>");   //send command zoom + to mini pro IR
  if (zoom_consigne < zoom) Serial1.print("<->");   //send command zoom + to mini pro IR
}

if (Serial_on == 1) {
Serial.println("");
Serial.println("************************");
Serial.print("pos_rotation "); Serial.println(pos_rotation);
Serial.print("vit_rotation "); Serial.println(vit_rotation);
Serial.print("Altitude "); Serial.println(Altitude);
Serial.print("pos_hauteur "); Serial.println(pos_hauteur);
Serial.print("fspeed_avion_filt "); Serial.println(fspeed_avion_filt);
Serial.print("Distance "); Serial.println(Distance);
Serial.print("zoom_consigne "); Serial.println(zoom_consigne);
Serial.println("************************");
Serial.println("");
}



if (Ecran_on == 1) {
display.print("Distance :");
display.println(Distance);
display.print("CourseTo :");
display.println(CourseTo);
display.print("Altitude :");
display.println(Altitude);
display.print("Speed :");
display.println(fspeed_avion_filt);
display.print("angle hori : ");
display.println(pos_rotation,2);
display.print("angle vert : ");
display.println(pos_hauteur,2);
display.print("Zoom : ");
display.print(zoom_consigne);
display.print(" Cam : ");
display.println(zoom);
display.update();
}


}
}


while ( (millis() - time_start < 100) )
  {
  }
  
}

double LP_Altitude(double x , double measurement)
{
x = ( x + k_alt * (measurement - x) ); 
return x;
}

double LP_lat_base(double x , double measurement)
{
x = ( x + k_base * (measurement - x) ); 
return x;
}

double LP_lon_base(double x , double measurement)
{
x = ( x + k_base * (measurement - x) ); 
return x;
}

/*double LP_pressure_avion(double x , double measurement)
{
x = ( x + k_avion * (measurement - x) ); 
return x;
}*/

void start_stop_rec(void)
{
  Serial1.print("<r>");     
  /*for (int i = 0; i < 3; i++) {
  irsend.sendSony(0xf343f, 20);
  float loop_start = millis();
  while ( (millis() - loop_start < loop_IR) )
  {
  }
  }*/
}

void zoom_plus(void)
{
  if (Serial_on != 0) Serial.println("+");
  zoom = zoom + 1;
  Serial1.print("<+>");   
  /*for (int i = 0; i < 3; i++) {
  irsend.sendSony(0x5364, 15);
  float loop_start = millis();
  while ( (millis() - loop_start < loop_IR) )
  {
  }
  }*/
}

void zoom_moins(void)
{
  if (Serial_on != 0) Serial.println("-");
  zoom = zoom - 1;
  Serial1.print("<->");  
  /*for (int i = 0; i < 3; i++) {
  irsend.sendSony(0x1364, 15);
  float loop_start = millis();
  while ( (millis() - loop_start < loop_IR) )
  {
  }
  }*/
}

void update_servo(float pos1, float vit1, float pos2, float vit2)
{
char test[15];
int int_pos1 = pos1 + 0.5;
int int_vit1 = vit1 + 0.5;  
int int_pos2 = pos2 + 0.5;
int int_vit2 = vit2 + 0.5;  

if (int_pos1 < 10)
sprintf(test, "<00%d", int_pos1);
else if (int_pos1 >= 10 && int_pos1 < 100)
sprintf(test, "<0%d", int_pos1);
else
sprintf(test, "<%d", int_pos1);

if (int_vit1 < 10)
sprintf(test, "%s00%d", test, int_vit1);
else if (int_vit1 >= 10 && int_vit1 < 100)
sprintf(test, "%s0%d", test, int_vit1);
else
sprintf(test, "%s%d", test, int_vit1);

if (int_pos2 < 10)
sprintf(test, "%s00%d", test, int_pos2);
else if (int_pos2 >= 10 && int_pos2 < 100)
sprintf(test, "%s0%d", test, int_pos2);
else
sprintf(test, "%s%d", test, int_pos2);

if (int_vit2 < 10)
sprintf(test, "%s00%d>", test, int_vit2);
else if (int_vit2 >= 10 && int_vit2 < 100)
sprintf(test, "%s0%d>", test, int_vit2);
else
sprintf(test, "%s%d>", test, int_vit2);

//if (Serial_on != 0) Serial.println(test);
Serial2.print(test);
}


long getPressure()
{
  long D1, D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;
 
  D1 = getData(0x48, 10);
  D2 = getData(0x50, 1);

  dT = D2 - ((long)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)) / (float)100;
  OFF = ((unsigned long)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
  
  //Serial.println(TEMP);
  //Serial.println(P);
  
  return P;
}


long getData(byte command, byte del)
{
  long result = 0;
  twiSendCommand(0x77, command);
  delay(del);
  twiSendCommand(0x77, 0x00);
  Wire.requestFrom(0x77, 3);
  if(Wire.available()!=3);// Serial.println("Error: raw data not available");
  for (int i = 0; i <= 2; i++)
  {
    result = (result<<8) | Wire.read(); 
  }
  return result;
}


void setupSensor()
{
  twiSendCommand(0x77, 0x1e);
  delay(100);
  
  for (byte i = 1; i <=6; i++)
  {
    unsigned int low, high;

    twiSendCommand(0x77, 0xa0 + i * 2);
    Wire.requestFrom(0x77, 2);
    if(Wire.available()!=2);// Serial.println("Error: calibration data not available");
    high = Wire.read();
    low = Wire.read();
    calibrationData[i] = high<<8 | low;
    //Serial.print("calibration data #");
    //Serial.print(i);
    //Serial.print(" = ");
    //Serial.println( calibrationData[i] ); 
  }
}


void twiSendCommand(byte address, byte command)
{
  Wire.beginTransmission(address);
  if (!Wire.write(command));// Serial.println("Error: write()");
  if (Wire.endTransmission()) 
  {
    //Serial.print("Error when sending command: ");
    //Serial.println(command, HEX);
  }
}
