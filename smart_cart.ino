// libraries
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
//                   IMPORTANT DEFINITIONS !!
// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 13° 24' E (Positive), which is ~13 Degrees, or (which we need) 0.23 radians
#define DECLINATION_ANGLE 0.191986f

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 0.0f

// How often the GPS should update in MS  Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000


// struct to store gps values

struct GeoLoc {
  float lat;
  float lon;
};
// structure for the bluetooth
struct AppLoc{
  GeoLoc apploc ;
  bool is_new ;
  
  
};
// struct to store Compass data 
struct MyCompass{
  float X = 0.f ;
  float Y = 0.f ;
  float Z = 0.f ;
  float H = 0.f ;
  
  
};
// MOTOR AND SENSOR  
// Sensor and Motor Part

//front
int motor1pin1 = 24;
int motor1pin2 = 25;
//back
int motor2pin1 = 26;
int motor2pin2 = 27;
//speed control front and back respectively
int ena=22;
int enb=23;
int speedf=100;






//sensor1//left
int trigPin = 29;
int echoPin = 28;
int led = 53;
//sensor2//right
int trigPin2 = 31;
int echoPin2 = 30;
//sensor3//front
int trigPin3 = 33;
int echoPin3 = 32;
//sensor4
int trigPin4 = 35;
int echoPin4 = 34;

//NULL STATE
void defaultdirection()
{
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  delay(1000);
}


//SHARP LEFT
void left()
{
  
 digitalWrite(motor1pin1, HIGH);
 digitalWrite(motor1pin2, LOW);
 delay(1000);

}
//SHARP RIGHT
void right()
{
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  delay(1000);
}

//FRONT
void front()
{
  analogWrite(enb, 130);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
}

//BACK
void back()
{
  analogWrite(enb, 130);
  digitalWrite(motor2pin1, HIGH);//high==1
  digitalWrite(motor2pin2, LOW);
  delay(1000);
}
//STOP
void brake()
{
   digitalWrite(motor2pin1, LOW);//high==1
  digitalWrite(motor2pin2, LOW);
  delay(1000);
}

void rightcontrolled1()
{
    analogWrite(ena, 100); //ENA pin
//    analogWrite(10, 100); //ENB pin  
   right();
   analogWrite(ena, 150); //ENA pin
//   analogWrite(10, 150); //ENB pin  
  right();
     analogWrite(ena, 200); //ENA pin
////  analogWrite(10, 200); //ENB pin  
  right();
   analogWrite(ena, 255); //ENA pin
////  analogWrite(10, 255); //ENB pin  
  right();
  defaultdirection();
}
void leftcontrolled1()
{
    analogWrite(ena, 100); //ENA pin
//    analogWrite(10, 100); //ENB pin  
   left();
   analogWrite(ena, 150); //ENA pin
//   analogWrite(10, 150); //ENB pin  
  left();
     analogWrite(ena, 200); //ENA pin
////  analogWrite(10, 200); //ENB pin  
left();
   analogWrite(ena, 255); //ENA pin
////  analogWrite(10, 255); //ENB pin  
  left();
  defaultdirection();
}

void ledCheckForDistance(int distance)
{
   if((distance<=10)) 
  {
    digitalWrite(led, HIGH);
}
   else if(distance>10)
 {
     digitalWrite(led, LOW);
   }
}

int getDistanceUs(int trigpin, int echopin){

  long duration, distance;
  digitalWrite(trigpin,HIGH);
  delayMicroseconds(1000);
  digitalWrite(trigpin, LOW);
  duration=pulseIn(echopin, HIGH);
   distance =(duration/2)/29.1; 

  Serial.print(distance);
 Serial.println("CM");

  
  delay(10);
  return distance ;
  }
// GPS
TinyGPS gps;
// initialise serial for bluetooth and gps
SoftwareSerial hc06(13, 12); // rx = 13 tx = 12
SoftwareSerial nss(10, 11);   // rx = 10 tx = 11

// compass calibration
float xMax = 13.36 ;
float yMax = 0.64;
float yMin = -28.91 ;
float xMin = -18.09 ;
/* BLUETOOTH CODE*/
int i = 0 ;
char full[24] = {0} ;
AppLoc getApploc()

{
  char recvChar;
 
char x_string[12] = {0} ; //25.45698

  char y_string[12] = {0} ;
  //char full[23] = {0} ;
  float x = 0.f ; 
  float y = 0.f ;
  //int i = 0 ;
  bool is_new = false ;
 bool received_full = false ;
  while(hc06.available()>0) {

      
      
      recvChar = hc06.read();
      //Serial.print(recvChar); // Print the character received to the Serial Monitor
      full[i] = recvChar ;

      i++ ;
      if(i == 23)
      {
        received_full = true ;
       // recvChar = hc06.read();
        i = 0 ;
      }
     /* if(recvChar == ',')
      {
        i = 0 ;
        is_x = false ;
        continue ;
        }
       if ( is_x)
       {
          x_string[i] = recvChar ;
          //Serial.print(recvChar);
        }
      else{
        y_string[i] = recvChar ;
        //Serial.print(recvChar);
       
        }
        //Serial.println(i);
      i++ ;
      //Serial.print("Read character: ");
      //Serial.println(recvChar); // Print the character received to the Serial Monitor*/
      
    
  }
   
  if(received_full)
  {
    Serial.println(full) ;
    is_new = true ;
    int k = 0 ;
    bool is_x = true ;
    for( int j = 0; j < 23; j++)
    {
       if(full[j] == ',')
      {
        k = 0 ;
        is_x = false ;
        continue ;
        }
       if ( is_x)
       {
          x_string[k] = full[j] ;
          full[j] = 0 ;
          //Serial.print(full[j]);
        }
      else{
        y_string[k] = full[j] ;
        full[j] = 0 ;
        //Serial.print(full[j]);
       
        }
        k++ ;
       
    }
  }
  
  if(  received_full) {
    //Serial.println(x_string) ;
    //Serial.println(y_string) ;
    x = atof(x_string);
    y = atof(y_string) ;
   for(int h= 0 ; h < 12; h++){
      x_string[h] = 0 ;
      y_string[h] = 0 ;
    }
    Serial.println(x,5);
    Serial.println(y,5);
     
  }
  AppLoc apploc ;
  apploc.apploc.lat = x ;
  apploc.apploc.lon = y ;
  apploc.is_new = is_new ;
  return apploc ;
}
// Master Enable
bool enabled = false;

// compass 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


// functions for the gps 
GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc cartLoc;
  cartLoc.lat = 0.0;
  cartLoc.lon = 0.0;
  
  return cartLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc cartLoc;
  cartLoc.lat = flat;
  cartLoc.lon = flon;

  Serial.print(cartLoc.lat, 7); Serial.print(", "); Serial.println(cartLoc.lon, 7);

  return cartLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}


void displayCompassDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km to meter
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2((event.magnetic.y - ((yMax + yMin) / 2.0)), (event.magnetic.x - ((xMax + xMin) / 2.0)));

  // Offset
  //heading +=  0.19;//DECLINATION_ANGLE;
  //heading -= COMPASS_OFFSET;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Map to -180 - 180
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}
// My geo heading 
float myGeoHeading(){
//  * Get a new sensor event */ 
  sensors_event_t event; 
  
  mag.getEvent(&event);
 // Serial.println("Hey, doesn,t get here");
 // Serial.print("after\n") ;
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.20f;// -0.628319f ;// 0.191986f;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees ;
}

// My geo heading2
float myGeoHeading2(){
//  * Get a new sensor event */ 
  sensors_event_t event; 
  
  mag.getEvent(&event);
 // Serial.println("Hey, doesn,t get here");
 // Serial.print("after\n") ;
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.20f;// -0.628319f ;// 0.191986f;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  if(headingDegrees > 180){
    headingDegrees -= 360 ;
  }
  
  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees ;
}

// calibrated
float myGeoHeadingCalibrated(){
  //* Get a new sensor event */ 
  sensors_event_t event; 
  
  mag.getEvent(&event);
 // Serial.println("Hey, doesn,t get here");
 // Serial.print("after\n") ;
 /*float xMax = 0.0 ;
 float yMax = 0.0 ; 
 float  xMin = 0.0 ;
 float yMin = 0.0;*/

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2((event.magnetic.y - ((yMax + yMin) / 2.0)), (event.magnetic.x - ((xMax + xMin) / 2.0)));
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.20f;// -0.628319f ;// 0.191986f;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  if(headingDegrees > 180){
    headingDegrees -= 360 ;
  }
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees ;
}
/*void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void setSpeed(int speed)
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  setSpeedMotorA(speed);

  // turn on motor B
  setSpeedMotorB(speed);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}*/

void drive(int distance, float turn) {
  int fullSpeed = 230;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original turn: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  /*setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);*/
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc cartLoc = checkGPS();
  //bluetoothSerial.listen();
  delay(1000); // added 
  Serial.println("Strat moving ..");
  if (cartLoc.lat != 0 && cartLoc.lon != 0 && enabled) {
    float d = 0;
    //Start move loop here
    do {
      nss.listen();
      cartLoc = checkGPS();
      //bluetoothSerial.listen();
      
      d = geoDistance(cartLoc, loc);
      float t =  geoBearing(cartLoc, loc) + geoHeading()  ; //geoHeading(); //  myGeoHeading()
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(cartLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(cartLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading()); //geoHeading() //  myGeoHeading()
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && enabled && timeout>0); 

    //stop();
  }
}

void setupCompass() {
   /* Initialise the compass */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displayCompassDetails();
}

void setup() {
  // put your setup code here, to run once:

   // Compass
  setupCompass();

   //Debugging via serial
  Serial.begin(4800);

  //GPS
  nss.begin(9600);

  //Bluetooth
  hc06.begin(9600);

  // Start  Ultrasonci  and Motor
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

//  for sensors
pinMode(led, OUTPUT);
//sensor1
   pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
//for senor2
   pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
//for sensor3
   pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
//  for sensor4
   pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);

  pinMode(ena, OUTPUT); //FRONT
  pinMode(enb, OUTPUT);//BACK
  Serial.println("go");
  delay(2000);
  
}


void loop() {
  // put your main code here, to run repeatedly:


/*AppLoc apploc_start = getApploc() ;
if( apploc_start.is_new){
  enabled = true ;
  driveTo(apploc_start.apploc, 120);
  
}*/
AppLoc ma ;


ma.apploc.lat = 62.888113 ;
ma.apploc.lon = 27.628687 ;
ma.is_new = true;
enabled = true ;
driveTo(ma.apploc, 10) ;
  

}
