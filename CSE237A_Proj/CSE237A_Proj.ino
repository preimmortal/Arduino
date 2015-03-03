

// 
//   FILE:  dht11_test1.pde
// PURPOSE: DHT11 library test sketch for Arduino
//

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
	return 1.8 * celsius + 32;
}

// fast integer version with rounding
//int Celcius2Fahrenheit(int celcius)
//{
//  return (celsius * 18 + 5)/10 + 32;
//}


//Celsius to Kelvin conversion
double Kelvin(double celsius)
{
	return celsius + 273.15;
}

// dewPoint function NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
//
double dewPoint(double celsius, double humidity)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
	double a = 17.271;
	double b = 237.7;
	double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
	double Td = (b * temp) / (a - temp);
	return Td;
}

#include <dht11.h>
#include <SoftwareSerial.h>

//Bluetooth Initalization Parameters
const int rxPin = 0; //SoftwaremySerial RX pin, connect to JY-MCY TX pin
const int txPin = 1; //SoftwaremySerial TX pin, connect to JY-MCU RX pin
                     // level shifting to 3.3 volts may be needed

SoftwareSerial mySerial(rxPin, txPin); // RX, TX
int state = 0;        // if state is 1, the LED will turn on and
                      // if state is 0, the LED will turn off
int flag = 0;         // a flag to prevent duplicate messages

dht11 DHT11;

#define DHT11PIN 8

int soundSensorPin=A4;
int soundReading=0;
int soundThreshold=20;

int HCSR501PIN=4;

int LEDPIN = 13;
  
void setup()
{
  
  //DHT11 (Temp/Humidity) Init
  /*
  mySerial.begin(115200);
  mySerial.println("DHT11 TEST PROGRAM ");
  mySerial.print("LIBRARY VERSION: ");
  mySerial.println(DHT11LIB_VERSION);
  mySerial.println();
  */
  //Turn on Bluetooth mySerial
  mySerial.begin(9600);
  
  //HC_SR501 - Motion Sensor Init
  pinMode(HCSR501PIN,INPUT);
  
  //Sound Sensor Init
  pinMode(soundSensorPin, INPUT);
  
  //LED Pin
  pinMode(LEDPIN, OUTPUT);
  //Turn LED Off
  digitalWrite(LEDPIN, LOW);
}

void DHT11_Reading()
{
  //DHT_11 Main Code
  mySerial.println("\n");

  int chk = DHT11.read(DHT11PIN);

  mySerial.print("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK: 
      mySerial.println("OK"); 
      break;
    case DHTLIB_ERROR_CHECKSUM: 
      mySerial.println("Checksum error"); 
      break;
    case DHTLIB_ERROR_TIMEOUT: 
      mySerial.println("Time out error"); 
      break;
    default: 
      mySerial.println("Unknown error"); 
      break;
  }

  mySerial.print("Humidity (%): ");
  mySerial.println((float)DHT11.humidity, 2);

  mySerial.print("Temperature (C): ");
  mySerial.println((float)DHT11.temperature, 2);

  mySerial.print("Temperature (F): ");
  mySerial.println(Fahrenheit(DHT11.temperature), 2);

  mySerial.print("Temperature (K): ");
  mySerial.println(Kelvin(DHT11.temperature), 2);

  mySerial.print("Dew Point (C): ");
  mySerial.println(dewPoint(DHT11.temperature, DHT11.humidity));

  mySerial.print("Dew PointFast (C): ");
  mySerial.println(dewPointFast(DHT11.temperature, DHT11.humidity));
  
  

}

void HC_SR501_Reading()
{
  //Motion Sensor
  //HC_SR501 Main Code
  //digitalWrite(13,digitalRead(2));
  if(digitalRead(HCSR501PIN)){
    mySerial.println("Motion: 1\n");
  }else{
    mySerial.println("Motion: 0\n"); 
  }
}

void Sound_Sensor_Reading()
{
  //Sound Sensor Main Code
  soundReading = analogRead(soundSensorPin);
  if(soundReading >1){
    mySerial.print("Sound Reading: ");
    mySerial.println(soundReading);
  }
  /*
  if(soundReading > 10){
    digitalWrite(LEDPIN, HIGH);
  }else{
    digitalWrite(LEDPIN, LOW); 
  }
  */
}

void Bluetooth_Module()
{
  //reads mySerial input and saves it in the state variable
  //mySerial.println("Test");
  /*
    if(mySerial.available() > 0){
      state = mySerial.read();
      flag=0; //clear the flag so we can print the state
    }
    // if the state is '0' the LED will turn off
    if (state == '0') {
        digitalWrite(LEDPIN, LOW);
        if(flag == 0){
          mySerial.println("LED: off");
          flag = 1;
        }
    }
    // if the state is '1' the led will turn on
    else if (state == '1') {
        digitalWrite(LEDPIN, HIGH);
        if(flag == 0){
          mySerial.println("LED: on");
          flag = 1;
        }
    }
  */

  typedef enum {SM_IDLE, READ_TEMP, READ_MOTION, READ_SOUND} STATES;
  //Implement State Machine
  static STATES STATE = SM_IDLE;
  
  //State Transition Logic
  if(mySerial.available() > 0){
    state = mySerial.read();
    switch(state){
      case '0': 
        mySerial.println("Switching to IDLE State\n");
        STATE = SM_IDLE;
        break;
      case '1':
        mySerial.println("Switching to TEMP State\n");
        STATE = READ_TEMP;
        break;
      case '2':
        mySerial.println("Switching to MOTION State\n");
        STATE = READ_MOTION;
        break;
      case '3':
        mySerial.println("Switching to SOUND State\n");
        STATE = READ_SOUND;
        break;
      default:
        STATE = SM_IDLE;
        break;
    }
    flag=0; //clear the flag so we can print the state
  }
  
  switch(STATE){
    case SM_IDLE:
      mySerial.println("IDLE State\n");
      mySerial.println("1 -> Temp Readings\n");
      mySerial.println("2 -> Motion Sensor Readings\n");
      mySerial.println("3 -> Sound Sensor Readings\n");
      //Delay
      delay(1000);
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
      break;
    case READ_TEMP:
      DHT11_Reading();
      //Delay
      delay(1000);
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
      break;
    case READ_MOTION:
      HC_SR501_Reading();
      //Delay
      delay(100);
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
      break;
    case READ_SOUND:
      Sound_Sensor_Reading();
      //Delay
      delay(10);
      break;
  }
  
  
  
}

void loop()
{
  //Temperature Readings
  //DHT11_Reading();
  
  //Motion Sensor Readings
  //HC_SR501_Reading();
  
  //Sound Sensor Readings
  //Sound_Sensor_Reading();
  
  //Bluetooth Module
  //This module will control when we read from sensors
  Bluetooth_Module();
  
  
  
  
}

//
// END OF FILE
//
