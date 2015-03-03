
#include <dht11.h>
#include <temp.h>
#include <Timer.h>
#include <SoftwareSerial.h>

//Bluetooth Initalization Parameters
const int rxPin = 2; //SoftwaremySerial RX pin, connect to JY-MCY TX pin
const int txPin = 7; //SoftwaremySerial TX pin, connect to JY-MCU RX pin
                     // level shifting to 3.3 volts may be needed

SoftwareSerial mySerial(rxPin, txPin); // RX, TX
int state = 0;        // if state is 1, the LED will turn on and
                      // if state is 0, the LED will turn off
int flag = 0;         // a flag to prevent duplicate messages

dht11 DHT11;

#define DHT11PIN 8

int soundSensorPin=A5;
int soundReading=0;
int soundThreshold=20;

int HCSR501PIN=4;

int LEDPIN = 13;

Timer t;

//Sensor Buffers
float temp_buffer[10]; //5 Minutes of samples
//These Buffes must store 1 bit at a time
byte motion_buffer[60]; //1 Minute of samples (motion)
byte sound_buffer[750]; //1 Minute of samples (sound)



void setup()
{
  //Turn on Bluetooth mySerial
  mySerial.begin(9600);
  Serial.begin(115200);
  //Initialize Timer Events
  //Temperature Sensor: 30s sample rate
  int tempReadEvent = t.every(1000*30, DHT11_Reading);
  mySerial.println("Started Temp Reading");
  mySerial.println("EventID: ");
  mySerial.println(tempReadEvent);
  
  /*
  //Motion Sensor: 100ms sample rate
  int motionReadEvent = t.every(100, HC_SR501_Reading);
  mySerial.println("Started Motion Reading");
  mySerial.println("EventID: ");
  mySerial.println(motionReadEvent);
  */
  
  //Sound Sensor: 10ms sample rate
  int soundReadEvent = t.every(10, Sound_Sensor_Reading);
  mySerial.println("Started Sound Reading");
  mySerial.println("EventID: ");
  mySerial.println(soundReadEvent);
  
  /*
  int clearScreenEvent = t.every(1000, Clear_Screen);
  mySerial.println("Started Clear Screen Event");
  mySerial.println("EventID: ");
  mySerial.println(clearScreenEvent);
  */
  
  
  
  //HC_SR501 - Motion Sensor Init
  pinMode(HCSR501PIN,INPUT);
  
  //Sound Sensor Init
  pinMode(soundSensorPin, INPUT);
  
  //LED Pin
  pinMode(LEDPIN, OUTPUT);
  //Turn LED Off
  digitalWrite(LEDPIN, LOW);
}

void Clear_Screen()
{
  //Clear Screen
  mySerial.write(27);       // ESC command
  mySerial.print("[2J");    // clear screen command
  mySerial.write(27);
  mySerial.print("[H");     // cursor to home command
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
    Serial.print("Sound Reading: ");
    Serial.println(soundReading);
  }
  
  if(soundReading > 10){
    digitalWrite(LEDPIN, HIGH);
  }else{
    digitalWrite(LEDPIN, LOW); 
  }
  
}

void Bluetooth_Module()
{
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
      //DHT11_Reading();
      //Delay
      delay(1000);
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
      break;
    case READ_MOTION:
      //HC_SR501_Reading();
      //Delay
      delay(100);
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
      break;
    case READ_SOUND:
      //Sound_Sensor_Reading();
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
  //Bluetooth_Module();
  
  /*
  int i;
  for(i=0; i < 10; i++){
    temp_buffer[i] = 1;
    mySerial.print("TempBuffer=");
    mySerial.println(i);
    mySerial.println(temp_buffer[i]);
  }
  for(i=0; i<600; i++){
    motion_buffer[i] = 1; 
    mySerial.print("motionbuffer=");
    mySerial.println(i);
    mySerial.println(motion_buffer[i]);
  }
  
  for(i=0; i<600; i++){
    motion_buffer[i] = 1; 
    mySerial.print("soundbuffer=");
    mySerial.println(i);
    mySerial.println(sound_buffer[i]);
  }
  */
  
  t.update();
  /*
  //Clear Screen
  mySerial.write(27);       // ESC command
  mySerial.print("[2J");    // clear screen command
  mySerial.write(27);
  mySerial.print("[H");     // cursor to home command
  */
}

//
// END OF FILE
//
