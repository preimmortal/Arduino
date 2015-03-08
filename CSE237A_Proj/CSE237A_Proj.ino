//Library Includes
#include <dht11.h>
#include <temp.h>
#include <Timer.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

//#define DEBUG
#define SERIAL

/***************************************/
/* Bluetooth Init */
/***************************************/

//Bluetooth Initalization Parameters
const int rxPin = 2; //SoftwaremySerial RX pin, connect to JY-MCY TX pin (Green)
const int txPin = 7; //SoftwaremySerial TX pin, connect to JY-MCU RX pin (Red)
                     // level shifting to 3.3 volts may be needed

SoftwareSerial mySerial(rxPin, txPin); // RX, TX


/***************************************/
/* DHT11 Init */
/***************************************/
dht11 DHT11;
#define DHT11PIN 8

/***************************************/
/* Sound Init */
/***************************************/
int soundSensorPin=A0;
int soundReading=0;
int SOUND_THRESH = 6;
/***************************************/
/* Motion Init */
/***************************************/
int HCSR501PIN=4;

/***************************************/
/* LED Init */
/***************************************/
int LEDPIN = 13;
int LEDSOUNDPIN = 12;


/***************************************/
/* Timer Init */
/***************************************/
Timer t;
int initIdle = 0;
int initTemp = 0;
/***************************************/
/* Buffers Init */
/***************************************/
//Buffer Sizes
#define TEMP_BUF_SIZE 10
#define MOTION_BUF_SIZE 60
#define SOUND_BUF_SIZE 750
//Initialize Buffers
float temp_buffer[TEMP_BUF_SIZE]; //5 Minutes of samples - Stored in deg C
float humidity_buffer[TEMP_BUF_SIZE]; // 5 Minutes of samples
//These Buffes must store 1 bit at a time
byte motion_buffer[MOTION_BUF_SIZE]; //1 Minute of samples (motion)
byte sound_buffer[SOUND_BUF_SIZE]; //1 Minute of samples (sound)

/***************************************/
/* Buffer Indices Init */
/***************************************/
static int DHT11_BUF_IDX = 0;
static int MOTION_BUF_IDX = 0;
static int SOUND_BUF_IDX = 0;

/***************************************/
/* Output Flags Init */
/***************************************/
byte SOUND_FLAG = 0;
byte MOTION_FLAG = 0;


/***************************************/
/* Setup Function */
/***************************************/
void setup()
{
  //Turn on Bluetooth mySerial
  mySerial.begin(9600);
  Serial.begin(115200);
  //Initialize Timer Events
  //Temperature Sensor: 30s sample rate
  int tempReadEvent = t.every(1000, DHT11_Reading);
  mySerial.println("Started Temp Reading");
  mySerial.println("EventID: ");
  mySerial.println(tempReadEvent);
  
  
  //Motion Sensor: 100ms sample rate
  int motionReadEvent = t.every(100, HC_SR501_Reading);
  mySerial.println("Started Motion Reading");
  mySerial.println("EventID: ");
  mySerial.println(motionReadEvent);
  
  
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
  pinMode(LEDSOUNDPIN, OUTPUT);
  //Turn LED Off
  //digitalWrite(LEDPIN, LOW);
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
  //static int DHT_READ_IDX = 0;
  //DHT_11 Main Code
  int chk = DHT11.read(DHT11PIN);
  
  temp_buffer[DHT11_BUF_IDX] = (float)DHT11.temperature;
  humidity_buffer[DHT11_BUF_IDX] = (float) DHT11.humidity;
  
  /****DEBUG CODE****/
  #ifdef DEBUG
  Serial.print("Wrote temp_buffer:\n");
  Serial.println(DHT11_BUF_IDX);
  Serial.println(temp_buffer[DHT11_BUF_IDX]);
  #endif
  /****DEBUG CODE****/
  
  DHT11_BUF_IDX++;
  if(DHT11_BUF_IDX>=TEMP_BUF_SIZE){
    DHT11_BUF_IDX = 0; 
  }
  
  
}

void HC_SR501_Reading()
{
  //Motion Sensor
  //HC_SR501 Main Code
  //digitalWrite(13,digitalRead(2));
  if(digitalRead(HCSR501PIN)){
    int BUF_IDX = MOTION_BUF_IDX / 8;
    int SUB_IDX = MOTION_BUF_IDX % 8;
    byte WriteVal = 1<<SUB_IDX;
    motion_buffer[BUF_IDX] |= WriteVal;
    
    /****DEBUG CODE****/
    #ifdef DEBUG
    //mySerial.println("Motion: 1");
    Serial.println("Motion: 1");
    Serial.print("Wrote motion_buffer:");
    Serial.println(BUF_IDX);
    Serial.println(SUB_IDX);
    Serial.println(motion_buffer[BUF_IDX]);
    Serial.println("\n");
    #endif
    /****DEBUG CODE****/
    if(MOTION_FLAG == 1){
      Serial.println("Motion: 1");
      mySerial.println("Motion: 1"); 
    }
  }else{
    int BUF_IDX = MOTION_BUF_IDX / 8;
    int SUB_IDX = MOTION_BUF_IDX % 8;
    byte WriteVal = 1<<SUB_IDX;
    WriteVal = ~WriteVal;
    motion_buffer[BUF_IDX] &= WriteVal;
    
    /****DEBUG CODE****/
    #ifdef DEBUG
    //mySerial.println("Motion: 0");
    Serial.println("Motion: 0");
    Serial.print("Wrote motion_buffer:");
    Serial.println(BUF_IDX);
    Serial.println(SUB_IDX);
    Serial.println(motion_buffer[BUF_IDX]);
    Serial.println("\n");
    #endif
    /****DEBUG CODE****/
    if(MOTION_FLAG == 1){
      Serial.println("Motion: 0");
      mySerial.println("Motion: 0"); 
    }
  }
  //Increment Index
  MOTION_BUF_IDX++;
  if(MOTION_BUF_IDX >= 8*MOTION_BUF_SIZE){
    MOTION_BUF_IDX=0; 
  }
}

void Sound_Sensor_Reading()
{
  //Sound Sensor Main Code
  
  soundReading = analogRead(soundSensorPin);
  
  if(soundReading >1 && SOUND_FLAG==1){
    mySerial.print("Sound Reading: ");
    mySerial.println(soundReading);
    Serial.print("Sound Reading: ");
    Serial.println(soundReading);
  }
  
  
  //Sample into Buffers
  if(soundReading > SOUND_THRESH){
    int BUF_IDX = SOUND_BUF_IDX / 8;
    int SUB_IDX = SOUND_BUF_IDX % 8;
    byte WriteVal = 1<<SUB_IDX;
    sound_buffer[BUF_IDX] |= WriteVal;
    
    //****DEBUG CODE****
    #ifdef DEBUG
    //mySerial.println("Sound: 1");
    //Serial.println("Sound: 1");
    Serial.print("Wrote sound_buffer:");
    Serial.println(BUF_IDX);
    Serial.println(SUB_IDX);
    Serial.println(sound_buffer[BUF_IDX]);
    //Serial.println("\n");
    #endif
    //****DEBUG CODE****
  }else{
    int BUF_IDX = SOUND_BUF_IDX / 8;
    int SUB_IDX = SOUND_BUF_IDX % 8;
    byte WriteVal = 1<<SUB_IDX;
    WriteVal = ~WriteVal;
    sound_buffer[BUF_IDX] &= WriteVal;
    
    //****DEBUG CODE****
    #ifdef DEBUG
    //mySerial.println("Sound: 0");
    //Serial.println("Sound: 0");
    Serial.print("Wrote sound_buffer:");
    Serial.println(BUF_IDX);
    Serial.println(SUB_IDX);
    Serial.println(sound_buffer[BUF_IDX]);
    //Serial.println("\n");
    #endif
    //****DEBUG CODE****
  }  
  SOUND_BUF_IDX++;
  if(SOUND_BUF_IDX >= 8*SOUND_BUF_SIZE){
    SOUND_BUF_IDX=0; 
  }
  
  //Try to make LED stay on longer
  //LED CIRCUIT
  static int sampleNum = 0;
  sampleNum++;
  if(soundReading > SOUND_THRESH){
    //Serial.println("HIGH Sound Reading:");
    //Serial.println(soundReading);
    digitalWrite(LEDSOUNDPIN, HIGH);
    sampleNum = 0;
  }else if(sampleNum >= 5){
    //Serial.println("LOW Sound Reading:");
    //Serial.println(soundReading);
    digitalWrite(LEDSOUNDPIN, LOW); 
  }
  
}

void DHT11_Value(){
  //Clear Screen
      Serial.write(27);       // ESC command
      Serial.print("[2J");    // clear screen command
      Serial.write(27);
      Serial.print("[H");     // cursor to home command
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
  
  int i;
  Serial.println("Last 10 T/H Readings");
  mySerial.println("Last 10 T/H Readings");
  
  for(i=0; i<TEMP_BUF_SIZE; i++){
    Serial.print(i);
    Serial.print(" -- "); 
    Serial.print("Temp: ");
    Serial.print(temp_buffer[i]); 
    Serial.print(" C - ");
    Serial.print(Fahrenheit(DHT11.temperature));
    Serial.print(" F -- ");
    Serial.print("Humidity: ");
    Serial.print(humidity_buffer[i]);
    Serial.println(" percent");
    
    mySerial.print(i);
    mySerial.print(" -- "); 
    mySerial.print("Temp: ");
    mySerial.print(temp_buffer[i]); 
    mySerial.print(" C - ");
    mySerial.print(Fahrenheit(DHT11.temperature));
    mySerial.print(" F -- ");
    mySerial.print("Humidity: ");
    mySerial.print(humidity_buffer[i]);
    mySerial.println(" percent");
  }
  
  Serial.print("Current Readings -- ");
  Serial.print("Temp: ");
  Serial.print((float)DHT11.temperature); 
  Serial.print(" C - ");
  Serial.print(Fahrenheit(DHT11.temperature));
  Serial.print(" F -- ");
  Serial.print("Humidity: ");
  Serial.print((float)DHT11.humidity);
  Serial.println(" percent");
  
  mySerial.print("Current Readings -- ");
  mySerial.print("Temp: ");
  mySerial.print((float)DHT11.temperature); 
  mySerial.print(" C - ");
  mySerial.print(Fahrenheit(DHT11.temperature));
  mySerial.print(" F -- ");
  mySerial.print("Humidity: ");
  mySerial.print((float)DHT11.humidity);
  mySerial.println(" percent");
}

void Print_Init(){
      //Clear Screen
      mySerial.write(27);       // ESC command
      mySerial.print("[2J");    // clear screen command
      mySerial.write(27);
      mySerial.print("[H");     // cursor to home command
      
      //Clear Screen
      Serial.write(27);       // ESC command
      Serial.print("[2J");    // clear screen command
      Serial.write(27);
      Serial.print("[H");     // cursor to home command
  
      //Print IDLE Message
      mySerial.println("IDLE State\n");
      mySerial.println("1 -> Temp Readings\n");
      mySerial.println("2 -> Motion Sensor Readings\n");
      mySerial.println("3 -> Sound Sensor Readings\n");
      
      Serial.println("IDLE State\n");
      Serial.println("1 -> Temp Readings\n");
      Serial.println("2 -> Motion Sensor Readings\n");
      Serial.println("3 -> Sound Sensor Readings\n");
      
      
}

void Bluetooth_Module()
{
  typedef enum {SM_IDLE, READ_TEMP, READ_MOTION, READ_SOUND} STATES;
  //Implement State Machine
  static STATES STATE = SM_IDLE;
  STATES NEXT_STATE = STATE;
  int event = 0;
  int event2 = 0;

  //State Transition Logic
  if(mySerial.available() > 0 || Serial.available() > 0){
    event = mySerial.read();
    event2 = Serial.read();
    switch(event){
      case '0': 
        mySerial.println("Switching to IDLE State\n");
        Serial.println("Switching to IDLE State\n");
        NEXT_STATE = SM_IDLE;
        break;
      case '1':
        mySerial.println("Switching to TEMP State\n");
        Serial.println("Switching to TEMP State\n");
        NEXT_STATE = READ_TEMP;
        break;
      case '2':
        mySerial.println("Switching to MOTION State\n");
        Serial.println("Switching to MOTION State\n");
        NEXT_STATE = READ_MOTION;
        break;
      case '3':
        mySerial.println("Switching to SOUND State\n");
        Serial.println("Switching to SOUND State\n");
        NEXT_STATE = READ_SOUND;
        break;
      default:
        NEXT_STATE = SM_IDLE;
        break;
    }
    
    switch(event2){
      case '0': 
        mySerial.println("Switching to IDLE State\n");
        Serial.println("Switching to IDLE State\n");
        NEXT_STATE = SM_IDLE;
        break;
      case '1':
        mySerial.println("Switching to TEMP State\n");
        Serial.println("Switching to TEMP State\n");
        NEXT_STATE = READ_TEMP;
        break;
      case '2':
        mySerial.println("Switching to MOTION State\n");
        Serial.println("Switching to MOTION State\n");
        NEXT_STATE = READ_MOTION;
        break;
      case '3':
        mySerial.println("Switching to SOUND State\n");
        Serial.println("Switching to SOUND State\n");
        NEXT_STATE = READ_SOUND;
        break;
      default:
        NEXT_STATE = SM_IDLE;
        break;
    }
  }
  
  
  
  switch(STATE){
    case SM_IDLE:
      if(initIdle == 0){
        initIdle = t.every(1000, Print_Init);
      }
      if(NEXT_STATE != SM_IDLE){
        t.stop(initIdle);
        initIdle = 0; 
      }
      break;
    case READ_TEMP:
      if(initTemp == 0){
        initTemp = t.every(1000, DHT11_Value); 
      }
      if(NEXT_STATE != READ_TEMP){
        t.stop(initTemp);
        initTemp = 0; 
      }
      break;
    case READ_MOTION:
      //HC_SR501_Reading();
      //Delay
      MOTION_FLAG = 1;
      if(NEXT_STATE != READ_MOTION){
         MOTION_FLAG = 0; 
      }
      break;
    case READ_SOUND:
      //Sound_Sensor_Reading();
      SOUND_FLAG = 1;
      if(NEXT_STATE != READ_SOUND){
         SOUND_FLAG = 0; 
      }
      break;
  }
  
  STATE = NEXT_STATE;
  
}

void loop()
{ 
  //Bluetooth Module
  //This module will control when we read from sensors
  Bluetooth_Module();
  
  //Update the Timer for interrupts
  t.update();
}

//
// END OF FILE
//
