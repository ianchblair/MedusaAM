 /* CBUS Medusa - Amuri version
  *  Based on CMRI ir sensor example
  *  This version based on Arduino Uno, as an example.
  *  (A mega would handle more inputs)
  *  Uses pins A0 to A5 as inputs from detectors (e.g Hall effect)
  *  Pin 13 used for SCK - can't toggle LED to show we're alive
  *  This example implements a sensor controller to detect block occupancy.
  *  It drives 6 sensors. Can be more in arduino Mega, but this is just an example.
  *  The module produces ON/OFF events.
  *  The events can be toggled by the first 2 node variables. 0 means ON when train enters, OFF when trains leaves.
  *  1 means the opposite. Each bit of the 2 bytes are to set toggle or not.
  *  Use FLIM mode to configure on/off events.
  *  It implements all automatic configuration, including learning events.
  *  It does not handle DCC messages, but you can do it on your user function.
  *  You can change the ports to fit to your arduino.
  *  This node uses 500 bytes of EPROM to store events and the other information.
  *  See MemoryManagement.h for memory configuration
  *  To clear the memory, press pushbutton1 while reseting the arduino
  *  Based on Arduino configuration used for CBUS sound player, but used in this case for inputs instead of outputs
*/


#include <Arduino.h>
#include <SPI.h> //required by the library
#include <MergCBUS.h>
#include <Message.h>
#include <EEPROM.h> //required by the library

//Module definitions
// For uno we have six analogue inputs. Mega has more.
#define NUMSENSORS 6
// TLIMIT and RLIMIT require clarification.
// Sensors have no effect when their ports are high.
// 
// TLIMIT is the time in ms that the sensor pin has to be low before a false event will be sent
#define TLIMIT 500
// RLIMIT is the time in ms that the sensor pin has had to be previously high before a false event will be sent
// i.e.a true event is sent when the sensor pin goes high, if a false event was sent (or on first pass(?) TBC). 
// a false event is only sent after a high period of at least RLIMIT, followed by a low period of at least TLIMIT
#define RLIMIT 25
struct SENSOR {
  int port;
  int state;
  unsigned long time;
  unsigned long resets;
};

struct SENSOR sensors[NUMSENSORS];
int sensorport[NUMSENSORS]={A0,A1 ,A2 ,A3, A4 ,A5};
int interruptflag = 0;

//first 2 are to indicate which servo is on. 2 bytes to indicate to toggle. 2 for start and end angle
#define VAR_PER_SENSOR 1  //variables per sensor. used as reserve. it is mostly used by consumers

//CBUS definitions
#define GREEN_LED 5                  //merg green led port
#define YELLOW_LED 4                 //merg yellow led port
#define PUSH_BUTTON 7                //std merg push button
#define PUSH_BUTTON1 6               //debug push button
#define NODE_VARS 4                  //2 for toggle events, 2 for spare
#define NODE_EVENTS NUMSENSORS*2     //max number of events in case of teaching short events
#define EVENTS_VARS VAR_PER_SENSOR   //number of variables per event
#define DEVICE_NUMBERS NUMSENSORS   //number of device numbers. Each sensor can be a device
#define CBUS_UNO_PORT 10             // Pin of CS line of CBUS I/F 10 for Ardiono Uno, 53 for Mega
#define CBUS_MEGA_PORT 53
#define CAN_INTR 0                   // D2 (Uno) for interrupt 0

//arduino uno has 2k RAM, micro has 2.5k,  mega has 4K.


//create the merg object
MergCBUS cbus=MergCBUS(NODE_VARS,NODE_EVENTS,EVENTS_VARS,DEVICE_NUMBERS);

//timer function to read the can messages
void readCanMessages(){
  //read the can messages and put them in a circular buffer
  cbus.cbusRead();
  //Serial.println("R");
  interruptflag++;
}

void setup(){

  pinMode(PUSH_BUTTON1,INPUT_PULLUP);//debug push button
  Serial.begin(115200);

  // configure and start CAN bus and CBUS message processing
// CBUS.setNumBuffers(4);
// CBUS.setOscFreq(8000000UL);   // MCP2515 CANBUS 8Mhz
// CBUS.setPins(CHIPSELECT,CBUSINTPIN);
// CBUS.begin();

  //Configuration data for the node
  cbus.getNodeId()->setNodeName("MEDUSADT",8);  //node name
  cbus.getNodeId()->setModuleId(57);            //module number
  cbus.getNodeId()->setManufacturerId(0xA5);    //merg code
  cbus.getNodeId()->setMinCodeVersion(1);       //Version 1
  cbus.getNodeId()->setMaxCodeVersion(0);
  cbus.getNodeId()->setProducerNode(true);
  cbus.getNodeId()->setConsumerNode(false);
  cbus.setStdNN(999); //standard node number

  if (digitalRead(PUSH_BUTTON1)==LOW){
    Serial.println("Setup new memory");
    cbus.setUpNewMemory();
    cbus.setSlimMode();
    cbus.saveNodeFlags();
  }
  cbus.setLeds(GREEN_LED,YELLOW_LED);//set the led ports
  cbus.setPushButton(PUSH_BUTTON);//set the push button ports
  cbus.setUserHandlerFunction(&myUserFunc);//function that implements the node logic
  cbus.initCanBus(CBUS_UNO_PORT,CAN_125KBPS,10,200);  //initiate the transport layer. pin=53, rate=125Kbps,10 tries,200 millis between each try
  //cbus.initCanBus(CBUS_UNO_PORT);  //initiate the transport layer. pin=CBUS_PORT, rate=125Kbps,10 tries,200 millis between each try
  
  //create the sensors object
  setupSensors();
  // Using pin can_intr
  attachInterrupt(CAN_INTR, readCanMessages, FALLING);
  //using timer
  // Timer1.initialize(10000);//microseconds
  // Timer1.attachInterrupt(readCanMessages);
  Serial.println("Setup finished");
}

void loop (){

  cbus.run();//do all logic

  if (cbus.getNodeState()==NORMAL){
    checkSensors();
  }

  //debug memory
  if (digitalRead(PUSH_BUTTON1)==LOW){
    cbus.dumpMemory();
  }
    //debug memory
  if (digitalRead(PUSH_BUTTON)==LOW){
     Serial.print("F");    
  }
  // Interrupt check
  if (interruptflag != 0) {
     Serial.print("R");
     interruptflag = 0;     
  }
  
}

//user defined function. contains the module logic.called every time run() is called.
void myUserFunc(Message *msg,MergCBUS *mcbus){

}

void checkSensors(){
  int state;
  int i;
  unsigned long actime;
  //int s=7;

  for (i=0;i<NUMSENSORS;i++){
    state=getSensorState(i);
    //Serial.println(state);
    actime=millis();
    if (state==LOW){
      if (sensors[i].state==HIGH){
        //if (i==s){
        /* 
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.println(" ON");
        */
        //}
        sendMessage(true,i);
        sensors[i].state=LOW;
      }
      sensors[i].state=LOW;
      sensors[i].time=actime;
      sensors[i].resets++;
    }
    else{
      if (actime-sensors[i].time>TLIMIT){
        if (sensors[i].resets<RLIMIT){
          //give extra time
          sensors[i].time=actime;
        }
       else {
          if (sensors[i].state==LOW){
          //    if (i==s){
          /*
              Serial.print("Sensor ");
              Serial.print(i);
              Serial.print(" OFF time: ");
              Serial.print(actime-sensors[i].time);
              Serial.print(" resets:");
              Serial.println(sensors[i].resets);
            */
              sendMessage(false,i) ;
            //  }
            sensors[i].state=HIGH;
            sensors[i].resets=0;
          }
       }
      }
    }
  }
}

//send the can message
void sendMessage(bool state,unsigned int sensor){
   unsigned int event;
   bool onEvent=true;
   event=sensor;
   if (togleSensor(sensor)){
     onEvent=false;
   }
   if (onEvent){
     cbus.sendOnEvent(true,event);
   }
   else{
     cbus.sendOffEvent(true,event);
   }
}
//check if we have to toggle the event
bool togleSensor(int sensor){
  byte first,second;
  bool resp=false;
  first=cbus.getNodeVar(0);
  second=cbus.getNodeVar(1);
  //check if the bit is set
  if (sensor>0 && sensor<9){
    if (bitRead(first,sensor)==1){
      resp=true;
    }
  }
  else if (sensor>8 && sensor<17){
     if (bitRead(second,sensor)==1){
      resp=true;
    }
  }
  return resp;
}
//configure the sensors
void setupSensors(){
  int i=0;
  for (i=0;i<NUMSENSORS;i++)  {
    sensors[i].state=HIGH;
    sensors[i].port=sensorport[i];
    pinMode(sensors[i].port,INPUT);
  }
}
//read the sensor state
byte getSensorState(int i){
  //return digitalRead(sensors[i].port);

  int j;
  float ntimes;
  ntimes=30;
  for (j=0;j<ntimes;j++){
    if (digitalRead(sensors[i].port)==0){
      return LOW;
    }
  }
  return HIGH;

}
