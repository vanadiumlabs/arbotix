#include <ax12Serial.h>
#include <BioloidSerial.h>
//====================================================================================================
// Kurts Test program to try out different ways to manipulate the AX12 servos on the PhantomX
// This is a test, only a test...  
//====================================================================================================
// Uncomment the next line if building for a Quad instead of a Hexapod.
//#define QUAD_MODE
#define TURRET


// Constants
/* Servo IDs */
#define     RF_COXA       2
#define     RF_FEMUR      4
#define     RF_TIBIA      6

#define     RM_COXA      14
#define     RM_FEMUR     16
#define     RM_TIBIA     18

#define     RR_COXA       8
#define     RR_FEMUR     10
#define     RR_TIBIA     12

#define     LF_COXA       1
#define     LF_FEMUR      3
#define     LF_TIBIA      5

#define     LM_COXA      13
#define     LM_FEMUR     15
#define     LM_TIBIA     17

#define     LR_COXA       7
#define     LR_FEMUR      9
#define     LR_TIBIA     11

#ifdef TURRET
#define     TURRET_ROT    20
#define     TURRET_TILT   21
#endif

#define PTEST_ID TURRET_TILT

static const byte pgm_axdIDs[] PROGMEM = {
  LF_COXA, LF_FEMUR, LF_TIBIA,    
#ifndef QUAD_MODE
  LM_COXA, LM_FEMUR, LM_TIBIA,    
#endif  
  LR_COXA, LR_FEMUR, LR_TIBIA,
  RF_COXA, RF_FEMUR, RF_TIBIA, 
#ifndef QUAD_MODE
  RM_COXA, RM_FEMUR, RM_TIBIA,    
#endif
  RR_COXA, RR_FEMUR, RR_TIBIA
#ifdef TURRET
  , TURRET_ROT, TURRET_TILT
#endif
};    

#define NUM_SERVOS (sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0]))
const char* IKPinsNames[] = {
  "LFC","LFF","LFT",
#ifndef QUAD_MODE
  "LMC","LMF","LMT",
#endif  
  "LRC","LRF","LRT",
  "RFC","RFF","RFT",
#ifndef QUAD_MODE
  "RMC","RMF","RMT",
#endif  
  "RRC","RRF","RRT",
#ifdef TURRET
  "T-ROT", "T-TILT"
#endif
};

// Global objects
/* IK Engine */
BioloidControllerEx bioloid = BioloidControllerEx(1000000);  // may use or not... may go direct to AX12

// other globals.
word           g_wVoltage;
char           g_aszCmdLine[80];
uint8_t        g_iszCmdLine;
boolean        g_fTrackServos = false;

// Values to use for servo position...
byte          g_bServoID;
word          g_wServoGoalPos;
word          g_wServoGoalSpeed;

//====================================================================================================
// Setup 
//====================================================================================================
void setup() {
  Serial.begin(9600);  // start off the serial port.  
  bioloid.poseSize = NUM_SERVOS;

  delay(1000);
  Serial.print("System Voltage in 10ths: ");
  Serial.println(g_wVoltage = ax12GetRegister(PTEST_ID, AX_PRESENT_VOLTAGE, 1), DEC);
  
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}


//====================================================================================================
// Loop
//====================================================================================================
void loop() {
  // Output a prompt
  word wNewVoltage = ax12GetRegister(PTEST_ID, AX_PRESENT_VOLTAGE, 1);
  if (wNewVoltage != g_wVoltage) {
    g_wVoltage = wNewVoltage;
    Serial.print("System Voltage in 10ths: ");
    Serial.println(g_wVoltage, DEC);
  }

  // lets toss any charcters that are in the input queue
  while(Serial.read() != -1) 
    ;

  Serial.println("0 - All Servos off");
  Serial.println("1 - All Servos center");
  Serial.println("2 - Set Servo position [<Servo>] <Position> [<Speed>]");
  Serial.println("3 - Set Servo Angle");
  Serial.println("4 - Get Servo Positions");
#if 0
  Serial.println("5 - Timed Move: <Servo> <From> <to> <speed> <cnt>");
  Serial.println("6 - Timed 2: <Servo> <speed>");
  Serial.println("7 - Timed 3: <Servo> <Dist> <Time>");
#endif
  Serial.println("8 - Set ID: <old> <new>");
  Serial.println("9 - Print Servo Values");
  Serial.println("t - Toggle track Servos");
  Serial.println("h - hold [<sn>]");
  Serial.println("f - free [<sn>]"); 
  Serial.print(":");
  Serial.flush();  // make sure the complete set of prompts has been output...  
  // Get a command
  if (GetCommandLine()) {
    Serial.println("");
    g_iszCmdLine = 1;  // skip over first byte...
    switch (g_aszCmdLine[0]) {
    case '0':
      AllServosOff();
      break;
    case '1':
      AllServosCenter();
      break;
    case '2':
      SetServoPosition();  
      break;
    case '3':
      break;
    case '4':
      GetServoPositions();
      break;
#if 0
    case '5':
      TimedMove();
      break;
    case '6':
      TimedMove2();
      break;
    case '7':
      TimedMove3();
      break;
#endif
    case '8':
      SetServoID();
      break;
    case '9':
      PrintServoValues();
      break;
    case 'f':
    case 'F':
      HoldOrFreeServos(0);
      break;
    case 'h':
    case 'H':
      HoldOrFreeServos(1);
      break;

    case 't':
    case 'T':
      g_fTrackServos = !g_fTrackServos;
      if (g_fTrackServos) {
        Serial.println("Tracking On");
        TrackServos(true);  // call to initialize all of the positions.
      }
      else
        Serial.println("Tracking Off");
      TrackPrintMinsMaxs();
      break;
    }
  }
}

// Helper function to read in a command line
uint8_t GetCommandLine(void) {
  int ch;
  uint8_t ich = 0;
  g_iszCmdLine = 0;

  for(;;) {
    // throw away any thing less than CR character...
    ch = Serial.read();
    if ((ch >= 10) && (ch <=15)) {
      g_aszCmdLine[ich] = 0;
      return ich;
    }    
    if (ch != -1) 
      g_aszCmdLine[ich++] = ch;

    if (g_fTrackServos)
      TrackServos(false);
  }
}

//
boolean FGetNextCmdNum(word *pw ) {
  // Skip all leading num number characters...
  while ((g_aszCmdLine[g_iszCmdLine] < '0') || (g_aszCmdLine[g_iszCmdLine] > '9')) {
    if (g_aszCmdLine[g_iszCmdLine] == 0)
      return false;  // end of the line...
    g_iszCmdLine++;  
  }
  *pw = 0;
  while ((g_aszCmdLine[g_iszCmdLine] >= '0') && (g_aszCmdLine[g_iszCmdLine] <= '9')) {
    *pw = *pw * 10 + (g_aszCmdLine[g_iszCmdLine] - '0');
    g_iszCmdLine++;
  }
  return true;
}

//=======================================================================================
void AllServosOff(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    ax12SetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_TORQUE_ENABLE, 0x0);
    ax12ReadPacket(6);  // git the response...
  }
}
//=======================================================================================
void AllServosCenter(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    // See if this turns the motor off and I can turn it back on...
    ax12SetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_TORQUE_ENABLE, 0x1);
    ax12ReadPacket(6);  // git the response...
    ax12SetRegister2(pgm_read_byte(&pgm_axdIDs[i]), AX_GOAL_POSITION_L, 0x1ff);
    ax12ReadPacket(6);  // git the response...
  }
}
//=======================================================================================
void HoldOrFreeServos(byte fHold) {
  word iServo;

  if (!FGetNextCmdNum(&iServo)) {
    // All servos...
    for (int i = 0; i < NUM_SERVOS; i++) {
      ax12SetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_TORQUE_ENABLE, fHold);
      ax12ReadPacket(6);  // git the response...
    }
  } 
  else {
    ax12SetRegister(iServo, AX_TORQUE_ENABLE, fHold);
    ax12ReadPacket(6);  // git the response...
  }
}

//=======================================================================================

//=======================================================================================
void SetServoPosition(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  Serial.println("Set Servo Position"); 
  if (FGetNextCmdNum(&w2)) {  // We have at least 2 parameters
    g_bServoID = w1;    // So first is which servo
    g_wServoGoalPos = w2;
    if (FGetNextCmdNum(&w2)) {  // We have at least 3 parameters
      g_wServoGoalSpeed = w2;  
      ax12SetRegister2(g_bServoID, AX_GOAL_SPEED_L, g_wServoGoalSpeed);
      ax12ReadPacket(6);  // git the response...
      Serial.print("Goal Speed: ");
      Serial.print(g_wServoGoalSpeed, DEC);
    }
  } 
  else 
    g_wServoGoalPos = w1;  // Only 1 paramter so assume it is the new position

  // Now lets try moving that servo there   
  ax12SetRegister2(g_bServoID, AX_GOAL_POSITION_L, g_wServoGoalPos);
  ax12ReadPacket(6);  // git the response...
  Serial.print(" ID: ");
  Serial.print(g_bServoID, DEC);
  Serial.print(" ");
  Serial.println(g_wServoGoalPos, DEC);
}  

//=======================================================================================
void SetServoID(void) {
  word w1;
  word w2;

  if (!FGetNextCmdNum(&w1))
    return;    // no parameters so bail.

  if (!FGetNextCmdNum(&w2))
    return;    // no parameters so bail.

  Serial.print("Set Servo ID From: ");
  Serial.print(w1, DEC);
  Serial.print(" To: ");
  Serial.println(w2, DEC);

  // Now lets try moving that servo there   
  ax12SetRegister(w1, AX_ID, w2);
  ax12ReadPacket(6);  // git the response...
}  


void WaitForMoveToComplete(word wID) {
  do {
    //    delay(1);
  } 
  while (ax12GetRegister(wID, AX_MOVING, 1));
}

#if 0
//=======================================================================================
void TimedMove(void) {
  word wID;
  word wFrom;
  word wTo;
  word wSpeed;
  word wCnt;
  unsigned long ulTimes[50];  
  byte iTimes;

  if (!FGetNextCmdNum(&wID))
    return;
  if (!FGetNextCmdNum(&wFrom))
    return;
  if (!FGetNextCmdNum(&wTo))
    return;
  if (!FGetNextCmdNum(&wSpeed))
    return;
  if (!FGetNextCmdNum(&wCnt))
    return;

  Serial.print("ID ");
  Serial.print(wID, DEC);

  Serial.print(" ");
  Serial.print(wFrom, DEC);
  Serial.print("-");
  Serial.print(wTo, DEC);
  Serial.print(" Speed: ");
  Serial.print(wSpeed, DEC);
  Serial.print(" Cnt: ");
  Serial.println(wCnt, DEC);

  // Print out some Compliance information...
  Serial.print("CW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CW Slope: ");
  Serial.println(ax12GetRegister(wID, AX_CW_COMPLIANCE_SLOPE, 1), DEC);
  Serial.print("CCW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CCW Slope: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_SLOPE, 1), DEC);

  // pretty dumb have to have all parameters to run
  ax12SetRegister2(wID, AX_GOAL_SPEED_L, wSpeed);  // First set the speed.
  ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
  WaitForMoveToComplete(wID);
  iTimes = 0;
  ulTimes[iTimes++] = millis();
  while (wCnt-- > 0) {
    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wTo);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    ulTimes[iTimes++] = millis();
    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    ulTimes[iTimes++] = millis();
  }
  unsigned long ulServoDelta4;
  unsigned long ulTimeDelta;
  if (wFrom >= wTo)
    ulServoDelta4 = (unsigned long)(wFrom-wTo)*1000L;
  else
    ulServoDelta4 = (unsigned long)(wTo-wFrom)*1000L;
  for (byte i=1; i < iTimes; i++) {
    Serial.print(ulTimeDelta = (ulTimes[i]-ulTimes[i-1]), DEC);
    Serial.print(" ");
    Serial.print(ulServoDelta4, DEC);
    Serial.print(" ");
    Serial.println(ulServoDelta4/ulTimeDelta, DEC);
  }
  Serial.print("Total Delta Time:");
  Serial.println(ulTimeDelta = ulTimes[iTimes-1]-ulTimes[0], DEC);
  Serial.print("Total Distance:");
  Serial.println(ulServoDelta4*(iTimes-1), DEC);
  Serial.print("Guess Time: ");
  // Units per ms = ((114*1024*360)/300))/(60*1000)
  Serial.println((((unsigned long)((ulServoDelta4/16)*(unsigned long)(iTimes-1))*3125L))/((unsigned long)(456L*(unsigned long)wSpeed)), DEC);
  // 

}


//=======================================================================================
void TimedMove2(void) {
  word wID;
  word wFrom;
  word wTo;
  word wSpeed;
  word wCnt;
  word wSlope;
  unsigned long ulTimes[10];
  unsigned long ulTotalDists4[10];
  unsigned long ulTotalTimes[10];
  byte iTotals;  
  byte iTimes;

  Serial.println("TimedMove 2");
  if (!FGetNextCmdNum(&wID))
    return;

  if (!FGetNextCmdNum(&wSpeed))
    return;

  Serial.print("ID ");
  Serial.print(wID, DEC);

  Serial.print(" Speed: ");
  Serial.println(wSpeed, DEC);
  // pretty dumb have to have all parameters to run
  ax12SetRegister2(wID, AX_GOAL_SPEED_L, wSpeed);  // First set the speed.

  // Print out some Compliance information...
  Serial.print("CW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CW Slope: ");
  wSlope = ax12GetRegister(wID, AX_CW_COMPLIANCE_SLOPE, 1);
  Serial.println(wSlope, DEC);
  Serial.print("CCW Margin: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_MARGIN, 1), DEC);
  Serial.print("CCW Slope: ");
  Serial.println(ax12GetRegister(wID, AX_CCW_COMPLIANCE_SLOPE, 1), DEC);
  Serial.print("Punch: ");
  Serial.println(ax12GetRegister(wID, AX_PUNCH_L, 2), DEC);
  Serial.print("Torque Limit: ");
  Serial.println(ax12GetRegister(wID, AX_TORQUE_LIMIT_L, 2), DEC);
  iTotals = 0;

  for (;wSlope <= 256; wSlope <<= 1) {
    wFrom = 512 - wSlope/2;
    wTo = wFrom + wSlope;
    wCnt = 2;    // twice each one should be enough...

    Serial.print(wFrom, DEC);
    Serial.print("-");
    Serial.print(wTo, DEC);
    Serial.flush();  // Make sure everything has been written out...

    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    delay(100);    // also give time to stabilize, so first move does not impact...
    iTimes = 0;
    ulTimes[iTimes++] = millis();
    while (wCnt-- > 0) {
      ax12SetRegister2(wID, AX_GOAL_POSITION_L, wTo);  // Move to the Start position.
      WaitForMoveToComplete(wID);
      ulTimes[iTimes++] = millis();
      ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
      WaitForMoveToComplete(wID);
      ulTimes[iTimes++] = millis();
    }
    unsigned long ulServoDelta4;
    unsigned long ulTimeDelta;

    ulServoDelta4 = (unsigned long)(wTo-wFrom)*1000L;
    for (byte i=1; i < iTimes; i++) {
      Serial.print(" ");
      Serial.print(ulTimeDelta = (ulTimes[i]-ulTimes[i-1]), DEC);
      Serial.print(":");
      Serial.print(ulServoDelta4/ulTimeDelta, DEC);
    }
    Serial.print(" Totals:");
    Serial.print(ulTotalTimes[iTotals] = ulTimes[iTimes-1]-ulTimes[0], DEC);
    ulTotalDists4[iTotals] = ulServoDelta4*(iTimes-1);
    Serial.print(":");
    Serial.println(ulTotalDists4[iTotals]/ulTotalTimes[iTotals], DEC);
    iTotals++;
  }

  // Now lets print out some total dists and some speeds...
  for (byte i=1; i < iTotals; i++) {
    Serial.print("DDist: ");
    Serial.print(ulTotalDists4[i-1], DEC);
    Serial.print("-");
    Serial.print(ulTotalDists4[i], DEC);
    Serial.print("=");
    Serial.print(ulTotalDists4[i]-ulTotalDists4[i-1], DEC);
    Serial.print(" DT: ");
    Serial.print(ulTotalTimes[i-1], DEC);
    Serial.print("-");
    Serial.print(ulTotalTimes[i], DEC);
    Serial.print("=");
    Serial.print(ulTotalTimes[i]-ulTotalTimes[i-1], DEC);
    Serial.print(" ? ");
    Serial.print((ulTotalDists4[i]-ulTotalDists4[i-1])/(ulTotalTimes[i]-ulTotalTimes[i-1]), DEC);
    Serial.print(" - ");  
    Serial.println(((ulTotalDists4[i]-ulTotalDists4[i-1])/(ulTotalTimes[i]-ulTotalTimes[i-1]))/wSpeed, DEC);
  }
}

//=======================================================================================
void TimedMove3(void) {
  word wID;
  word wFrom;
  word wTo;
  word wDT;
  word wDist;
  word wSpeed;
  word wCnt;
  word wSlope;
  unsigned long ulTimes[10];
  unsigned long ulStart;
  byte abT[10];    // only needed 4 for first test of this...
  uint8_t iCur;
  uint8_t i;
  word awCurPos[100];
  word awCurSpeed[100];

  byte iTotals;  
  byte iTimes;

  // <Servo> <Dist> <Time>");
  //speed value = (10000xRange(deg))/(6xTime(ms)).
  Serial.println("TimedMove 3");
  if (!FGetNextCmdNum(&wID))
    return;

  if (!FGetNextCmdNum(&wDist))
    return;

  if (!FGetNextCmdNum(&wDT))
    return;

  // now lets calculate a from/to and a guess on speed...
  Serial.print("ID ");
  Serial.print(wID, DEC);

  Serial.print("Dist ");
  wFrom = 512 - wDist/2;
  wTo = wFrom + wDist;
  Serial.print(wFrom, DEC);
  Serial.print("-");
  Serial.print(wTo, DEC);
  Serial.print("=");
  Serial.println(wDist, DEC);

  // From Zenta - speed value = (10000xRange(deg))/(6xTime(ms)).
  // 500 ms @ 180 deg range = 600
  // Guess a speed - The movement in degrees is: dist/1024 * 300 so
  // Speed = *wDist/1024)*10000 / (6 * wDT); 
  unsigned long ulT = (unsigned long)wDist*15625;
  wSpeed = max(ulT / ((unsigned long)wDT * 32L), 1);
  Serial.print("Desired Time: ");
  Serial.print(wDT, DEC);
  Serial.print(" ");
  Serial.print(ulT, DEC);
  Serial.print(" Speed: ");
  Serial.println(wSpeed, DEC);
  // pretty dumb have to have all parameters to run
  ax12SetRegister2(wID, AX_GOAL_SPEED_L, wSpeed);  // First set the speed.

  wCnt = 3;    

  Serial.flush();  // Make sure everything has been written out...

  ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
  WaitForMoveToComplete(wID);
  delay(100);    // also give time to stabilize, so first move does not impact...
  iTimes = 0;
  ulStart = ulTimes[iTimes++] = millis();
  iCur = 0;
  while (wCnt-- > 0) {
    do {
      ax12SetRegister2(wID, AX_GOAL_POSITION_L, wTo);  // Move to the Start position.
      while (millis()-ulStart < 25) 
        ;
      ulStart = millis();    
      GetMultax12Registers(wID, AX_PRESENT_POSITION_L, 4, abT);
      awCurPos[iCur] = abT[0] + (abT[1] << 8);    // Get the current position and speed.
      awCurSpeed[iCur] = abT[2] + (abT[3] << 8);
    } 
    while (awCurSpeed[iCur++]) ;    // See if checking the speed will work ok...
    ulTimes[iTimes++] = millis();

    // Pass 1 only do for one direction.
    for (i=0; i < iCur; i++) {
      Serial.print(awCurPos[i], DEC);
      Serial.print("=");
      Serial.print(awCurSpeed[i],DEC);
      if ((i % 10) == 9)
        Serial.println("");
      else
        Serial.print(" ");
    }  
    Serial.println("");
    Serial.flush();  // Make sure everything has been written out...



    ax12SetRegister2(wID, AX_GOAL_POSITION_L, wFrom);  // Move to the Start position.
    WaitForMoveToComplete(wID);
    ulTimes[iTimes++] = millis();
  }
  unsigned long ulServoDelta4;
  unsigned long ulTimeDelta;

  ulServoDelta4 = (unsigned long)(wTo-wFrom)*1000L;
  for (byte i=1; i < iTimes; i++) {
    Serial.print(" ");
    Serial.print(ulTimeDelta = (ulTimes[i]-ulTimes[i-1]), DEC);
  }
  Serial.println("");
}
#endif


//=======================================================================================
void GetServoPositions(void) {

  unsigned long ulBefore;
  unsigned long ulDelta;
  bioloid.readPose();
  int w;
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
    Serial.print(":");
    ulBefore = micros();
    w = ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_PRESENT_POSITION_L, 2 );
    ulDelta = micros() - ulBefore;
    Serial.print(w, DEC);
    Serial.print(" ");
    Serial.print(ulDelta, DEC);
    Serial.print(" ");
    Serial.println(ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_RETURN_DELAY_TIME, 1), DEC);

    if (w == 0xffff) {
      Serial.print("   Retry: ");
      w = ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_PRESENT_POSITION_L, 2 );
      Serial.println(w, DEC);
    }    
    delay (100);
  }
}
//=======================================================================================
int g_asPositionsPrev[NUM_SERVOS];
int g_asMins[NUM_SERVOS];
int g_asMaxs[NUM_SERVOS];

void TrackServos(boolean fInit) {

  bioloid.readPose();
  int w;
  bool fSomethingChanged = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    w = ax12GetRegister(pgm_read_byte(&pgm_axdIDs[i]), AX_PRESENT_POSITION_L, 2 );
    if (fInit) {
      g_asMins[i] = w;
      g_asMaxs[i] = w;
    }
    if (w != g_asPositionsPrev[i]) {
      if (!fInit) {
        // only print if we moved more than some delta...
        if (abs(w-g_asPositionsPrev[i]) > 3) {
          Serial.print(IKPinsNames[i]);
          Serial.print("(");
          Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
          Serial.print("):");
          Serial.print(w, DEC);
          Serial.print("(");
          Serial.print((((long)(w-512))*375L)/128L, DEC);
          Serial.print(") ");
          fSomethingChanged = true;
        }
      }
      g_asPositionsPrev[i] = w;
      if (g_asMins[i] > w)
        g_asMins[i] = w;

      if (g_asMaxs[i] < w)
        g_asMaxs[i] = w;
    }  
  }
  if (fSomethingChanged)
    Serial.println();
}

void TrackPrintMinsMaxs(void) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print((byte)pgm_read_byte(&pgm_axdIDs[i]), DEC);
    Serial.print(":");
    Serial.print(g_asMins[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMins[i]-512))*375L)/128L, DEC);
    Serial.print(") ");

    Serial.print(g_asMaxs[i], DEC);
    Serial.print("(");
    Serial.print((((long)(g_asMaxs[i]-512))*375L)/128L, DEC);
    Serial.println(")");
  }
}


//=======================================================================================
void PrintServoValues(void) {

  word wID;
  word w;
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  if (!FGetNextCmdNum(&wID))
    return;
  for (int i = 0; i < 50; i++) {
    Serial.print(i, DEC);
    Serial.print(":");
    digitalWrite(A2, HIGH);
    w = ax12GetRegister(wID, i, 1 );
    digitalWrite(A2, LOW);
    if (w == -1)
      digitalWrite(A3, !digitalRead(A3));
    Serial.print(w, HEX);
    Serial.print(" ");
    if ((i%10) == 9)
      Serial.println("");
    Serial.flush();  // try to avoid any interrupts while processing.
    delay(5);
  }    
}
//=======================================================================================


//=======================================================================================
boolean GetMultax12Registers(int id, int regstart, int length, uint8_t *pab){  
  uint8_t *pbT;  
  setTX(id);
  // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
  int checksum = ~((id + 6 + regstart + length)%256);
  ax12writeB(0xFF);
  ax12writeB(0xFF);
  ax12writeB(id);
  ax12writeB(4);    // length
  ax12writeB(AX_READ_DATA);
  ax12writeB(regstart);
  ax12writeB(length);
  ax12writeB(checksum);  
  setRX(id);    
  // Should verify size of data actually read...
  if(ax12ReadPacket(length + 6) > 0){
    pbT = &ax_rx_buffer[5];
    while (length--)
      *pab++ = *pbT++;    // copy the data
    return true;   
  }
  return false;
}













