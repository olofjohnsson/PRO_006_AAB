#include <SoftwareSerial.h>
#include <string.h>
/********************************************************
Autopilot project. PRO_006. control sw for hydraulic
pump motor in a Örnvik 490 with 40hk outboard engine. 
Heading, velocity and setpoint is received via bluetooth 
from a smartphone. Relay board is controlled from this sw.
Output from PID controlles On/Off time to relay board.
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT A0
#define ledP 16
#define ledSB 12
#define LOW_RUNTIME 250
#define MID_RUNTIME 1000
#define HIGH_RUNTIME 3000

SoftwareSerial btSerial(4, 5);//HC05 TX connected to RX PIN 4 (D2) and HC05 RX connected to TX PIN 5 (D1)

//Define Variables we'll be connecting to
double Setpoint, Input, Output, period = 4000, velocity = 3;
double sensorValue = 0, cog = 0, pumpRuntime = 0, i = 0, error =0;
bool TurnSB, TurnP;
const unsigned int MAX_INPUT = 50;
char string1[100];

//Specify the links and initial tuning parameters
double Kp = 5, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

int WindowSize = 10000;
unsigned long windowStartTime;
char separator = ';';
char *token;

void setup()
{
  Serial.begin(9600);
  btSerial.begin(9600);
  delay(1000);
  windowStartTime = millis();
  pinMode(ledP, OUTPUT);
  pinMode(ledSB, OUTPUT);
  //initialize the variables we're linked to
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  delay(2000);
  //delay(period / velocity);
  TurnP = false;
  TurnSB = false;
  pumpRuntime = 0;
  getData();
  while (btSerial.available() > 0)
    processIncomingByte(btSerial.read());
  calculateInput();
  myPID.Compute();
  serialInfo();
  setHydraulicPumpRuntime();
  if (Input > 3) { //Filter out small inputs
    if (TurnSB)
      turnSB();
    if (TurnP)
      turnP();
  }
}
///////////////////////////////////////////////////////////
void getData(){
  btSerial.write('0x01');
}
///////////////////////////////////////////////////////////
void calculateInput(){ 
  if (Setpoint>180){
    Setpoint = Setpoint-360;
  }
  if (cog>180){ //Normalize error
    cog = cog-360;
    error = abs(Setpoint-cog);
    Input = Setpoint+error;
  }
  else{
    error = abs(Setpoint-cog);
    Input = Setpoint+error;
  }
    
  if (cog>Setpoint)
    TurnP=true;
  else
    TurnSB=true;
    
  if (error>180){
    TurnP = !TurnP;
    TurnSB = !TurnSB;
    error = 360-error;
    Input = Setpoint+error;
  }
}
///////////////////////////////////////////////////////////
String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
///////////////////////////////////////////////////////////
void processIncomingByte (const byte inByte)
{
  Serial.println("processIncomingByte...");
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {

    case '\n':   // end of text
      Serial.println("end of text...");
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    case '\r':
      Serial.println("carriage return...");
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1)) {
        input_line [input_pos++] = inByte;
      }
      break;

  }  // end of switch

} // end of processIncomingByte


///////////////////////////////////////////////////////////
// here to process incoming serial data after a terminator received
void process_data (char *data)
{
  Serial.println("process data...");
  /* get the first token */
  String setpointValue = getValue(data, ';', 0);
  String headingValue = getValue(data, ';', 1);
  String velocityValue = getValue(data, ';', 2);

  Setpoint = setpointValue.toFloat();
  cog = headingValue.toFloat();
  velocity = velocityValue.toFloat();
  if (velocity<1)
    velocity = 1;
  
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  Serial.print("Heading: ");
  Serial.println(cog);
  Serial.print("Velocity: ");
  Serial.println(velocityValue);

}  // end of process_data
///////////////////////////////////////////////////////////
void serialInfo() {
  Serial.print("Setpoint is: ");
  Serial.print(Setpoint);
  Serial.print(" cog is: ");
  Serial.print(cog);
  Serial.print(" input is: ");
  Serial.print(Input);
  Serial.print(" Output is: ");
  Serial.print(Output);
  Serial.print(" velocity (knots) is: ");
  Serial.println(velocity);
}
///////////////////////////////////////////////////////////
void getSetpoint() {
  // if serial data available, process it
  Serial.println("getSetpoint...");
}
///////////////////////////////////////////////////////////
void setHydraulicPumpRuntime() {
  if ((Output > 10) && (Output < 30)) {
    pumpRuntime = LOW_RUNTIME/velocity;
  }
  if ((Output > 30) && (Output < 250)) {
    pumpRuntime = MID_RUNTIME;
  }
  if (Output > 250) {
    pumpRuntime = HIGH_RUNTIME;
  }
}
///////////////////////////////////////////////////////////
void turnSB() {
  Serial.println("turnSB...");
  digitalWrite(ledSB, HIGH);
  delay(pumpRuntime);
  digitalWrite(ledSB, LOW);
  returnToZeroAngle();
}
////////////////////////////////////////////////////////////
void turnP() {
  Serial.println("turnP");
  digitalWrite(ledP, HIGH);
  delay(pumpRuntime);
  digitalWrite(ledP, LOW);
  returnToZeroAngle();
}
////////////////////////////////////////////////////////////
void returnToZeroAngle() {
  Serial.println("returnToZeroAngle...");
  if (TurnP) {
    digitalWrite(ledSB, HIGH);
    delay(pumpRuntime); //replace with returning to zero angle from angle potentiometer
    digitalWrite(ledSB, LOW);
  }
  if (TurnSB) {
    digitalWrite(ledP, HIGH);
    delay(pumpRuntime); //replace with returning to zero angle from angle potentiometer
    digitalWrite(ledP, LOW);
  }
}
////////////////////////////////////////////////////////////
