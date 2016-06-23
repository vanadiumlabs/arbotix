#include <ax12.h>
#include <BioloidController.h>
#include "poses.h"

BioloidController bioloid = BioloidController(1000000);

const int SERVOCOUNT = 18;
int id;
int pos;
boolean IDCheck;
boolean RunCheck;

void setup(){
   pinMode(0,OUTPUT);  
   
   //initialize variables 
   id = 1;
   pos = 0;
   IDCheck = 1;
   RunCheck = 0;
  //open serial port
   Serial.begin(9600);
   delay (500);   
    Serial.println("###########################");    
   Serial.println("Serial Communication Established.");    
  //Check Lipo Battery Voltage
  CheckVoltage();
  //Scan Servos, return position.
  
  MoveCenter();
  ScanServo();
  
  LeftLegTest();
  
  RightLegTest();
  
  MenuOptions();
 
  RunCheck = 1;
}

void loop(){
  // read the sensor:
  
    int inByte = Serial.read();

    switch (inByte) {

    case '1':    
      ScanServo();
      break;

    case '2':    
      MoveCenter();
      break;
      
     case '3':    
      RelaxServos();
      break;     

    case '4':    
      LeftLegTest();
      break;
      
    case '5':    
      RightLegTest();
      break;
      
    case '6':    
      CheckVoltage();
      break;
      
    case '7':
      LEDTest();
      break;
      

    } 
  
}


void ScanServo(){
  id = 1;  
  Serial.println("###########################");
  Serial.println("Starting Servo Scanning Test.");
  Serial.println("###########################");
      
  while (id <= SERVOCOUNT){
  pos =  ax12GetRegister(id, 36, 2);
  Serial.print("Servo ID: ");
  Serial.println(id);
  Serial.print("Servo Position: ");
  Serial.println(pos);
  
  if (pos <= 0){
  Serial.println("###########################");
  Serial.print("ERROR! Servo ID: ");
  Serial.print(id);
  Serial.println(" not found. Please check connection and verify correct ID is set.");
  Serial.println("###########################"); 
  IDCheck = 0;
  }
  
  id = (id++)%SERVOCOUNT;
  delay(1000);
  }
  if (IDCheck == 0){
    Serial.println("###########################");
    Serial.println("ERROR! Servo ID(s) are missing from Scan. Please check connection and verify correct ID is set.");
    Serial.println("###########################");  
  }
  else{
  Serial.println("All servo IDs present.");
  }
    if (RunCheck == 1){
    MenuOptions();
  }

}



void CheckVoltage(){  
   // wait, then check the voltage (LiPO safety)
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.println("###########################");   
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
  if (voltage < 10.0){
    Serial.println("Voltage levels below 10v, please charge battery.");
    while(1);
  }  
  if (voltage > 10.0){
  Serial.println("Voltage levels nominal.");
  }
    if (RunCheck == 1){
      MenuOptions();
  }
      Serial.println("###########################"); 
}

void MoveCenter(){
    delay(100);                    // recommended pause
    bioloid.loadPose(Center);   // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();            // read in current servo positions to the curPose buffer
    Serial.println("###########################");
    Serial.println("Moving servos to centered position");
    Serial.println("###########################");    
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);
    }
    if (RunCheck == 1){
      MenuOptions();
  }
}


void LeftLegTest(){
  Serial.println("###########################");
  Serial.println("Initializing Left Leg Tests");  
  Serial.println("###########################");
  delay(500);  
  id = 1;
  while(id <= (SERVOCOUNT-1)){
    
  Serial.print("Moving Servo ID: ");
  Serial.println(id);    
  SetPosition(id, 412);
  delay(1000);
  SetPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  SetPosition(id, 712);
  delay(1000);
  SetPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;  


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  SetPosition(id, 712);
  delay(1000);
  SetPosition(id, 512);
  delay(1000);
  //iterate to next servo ID
  id = id + 2;  
  }
    if (RunCheck == 1){
   MenuOptions();
  }
  
}




void RightLegTest(){

  Serial.println("###########################");
  Serial.println("Initializing Right Leg Tests"); 
  Serial.println("###########################");
  delay(500);  
  id = 2;
  while(id <= SERVOCOUNT){

  Serial.print("Moving Servo ID: ");
  Serial.println(id);  
  SetPosition(id, 612);
  delay(1000);
  SetPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  SetPosition(id, 312);
  delay(1000);
  SetPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;  


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  SetPosition(id, 312);
  delay(1000);
  SetPosition(id, 512);
  delay(1000);
  //iterate to next servo ID
  id = id + 2;  
  }
      if (RunCheck == 1){
      MenuOptions();
  }
  
}


void MenuOptions(){
  
    Serial.println("###########################"); 
    Serial.println("Please enter option 1-5 to run individual tests again.");     
    Serial.println("1) Servo Scanning Test");        
    Serial.println("2) Move Servos to Center");        
    Serial.println("3) Relax Servos");            
    Serial.println("4) Perform Left Leg Sign Test");        
    Serial.println("5) Perform Right Leg Sign Test");        
    Serial.println("6) Check System Voltage");    
    Serial.println("7) Perform LED Test");        
    Serial.println("###########################"); 
}

void RelaxServos(){
  id = 1;
  Serial.println("###########################");
  Serial.println("Relaxing Servos.");
  Serial.println("###########################");    
  while(id <= SERVOCOUNT){
    Relax(id);
    id = (id++)%SERVOCOUNT;
    delay(50);
  }
   if (RunCheck == 1){
      MenuOptions();
  }
}


void LEDTest(){
    id = 1;
  Serial.println("###########################");
  Serial.println("Running LED Test");
  Serial.println("###########################");    
  while(id <= SERVOCOUNT){
    ax12SetRegister(id, 25, 1);
    Serial.print("LED ON - Servo ID: ");
    Serial.println(id);
    delay(3000);
    ax12SetRegister(id, 25, 0);  
    Serial.print("LED OFF - Servo ID: ");
    Serial.println(id);    
    delay(3000);    
    id = id++;
  }
  
   if (RunCheck == 1){
      MenuOptions();
  }
}
   
    
