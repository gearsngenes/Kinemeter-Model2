/*
  IR Breakbeam sensor demo!
*/

#define LEDPIN 13

int const sensors =5;
int const slots = 2 * sensors;
int pinArray [sensors] = { 3, 4, 5,6,7};
unsigned long brokenTime[sensors];
float AvgTime[sensors];
float instTime[sensors];
float accl[sensors];

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX | TX

boolean passArray[sensors];
int sensStateArray[sensors];
int lastStateArray[sensors];
unsigned long timeArray[slots];
float deltaArray[sensors];
//const float DIST = 5.0;
float disp[sensors]={0,5,5,7.5,5};
float speedArray[sensors];
float accArray[sensors];

unsigned long curTime;
unsigned long startTime;
//float speed1;


void BTSetup ()

{
  BTSerial.begin(9600);
  Serial.println("BT Demo");
  startTime = millis();
  curTime = startTime;
}


void setup() {
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);

  // initialize the sensor pin as an input:
  for (int j1 = 0; j1 < sensors; j1++)
  {
    pinMode(pinArray[j1], INPUT);
    digitalWrite(pinArray[j1], HIGH); // turn on the pullup
    passArray[j1] = false;
  }
  Serial.begin(115200);
  BTSetup();
  Serial.println("Set up Complete.");
  BTSerial.println("Set up Complete");
}


void processSensorPinat (int pinArrayindex)
{

  sensStateArray[pinArrayindex] = digitalRead(pinArray[pinArrayindex]);
  if  (sensStateArray[pinArrayindex] == LOW)

  {
    // turn LED on:
    digitalWrite(LEDPIN, HIGH);
  }
  else {
    // turn LED off:
    digitalWrite(LEDPIN, LOW);
  }

  if
  (sensStateArray[pinArrayindex] && !lastStateArray[pinArrayindex])

  {
    Serial.print("At sensor # ");
    timeArray[2 * pinArrayindex + 1] = millis();
 
    brokenTime[pinArrayindex] = timeArray[2 * pinArrayindex + 1] - timeArray[2 * pinArrayindex];
    instTime[pinArrayindex] = (timeArray[2 * pinArrayindex + 1] + timeArray[2 * pinArrayindex]) / 2000.0;
    //Serial.print("Mid Break:  "); Serial.println(instTime[pinArrayindex],4);
    Serial.print(pinArray[pinArrayindex]); Serial.print("  at T= ");
    Serial.println(instTime[pinArrayindex],4);
    if (pinArrayindex > 0)
    {

      deltaArray[pinArrayindex] = instTime[pinArrayindex]-instTime[pinArrayindex-1];
     // Serial.print("Time between Midpoints: "); Serial.println (deltaArray[pinArrayindex]);
      speedArray[pinArrayindex] = disp[pinArrayindex] / deltaArray[pinArrayindex] ;
      AvgTime[pinArrayindex]= (instTime[pinArrayindex]+instTime[pinArrayindex-1])/2;

      Serial.print("           Inst Velocity (avg.) betw. sensors  ");
      Serial.print(pinArray[pinArrayindex-1]);Serial.print(" and "); Serial.print(pinArray[pinArrayindex]);
      Serial.print(" is ");
       Serial.print(speedArray[pinArrayindex],4);Serial.print("cm/sec ");
      Serial.print(" at T=  "); Serial.print(AvgTime[pinArrayindex],4);Serial.println("sec");
    }

    passArray[pinArrayindex] = true;
  }
  if  (!sensStateArray[pinArrayindex] && lastStateArray[pinArrayindex])

  {

    //Serial.print("Sensor["); Serial.print(pinArray[pinArrayindex]); Serial.print("] broke @:");
    timeArray[2 * pinArrayindex] = millis();
   // Serial.print (timeArray[2 * pinArrayindex]);

  }

  lastStateArray[pinArrayindex] = sensStateArray[pinArrayindex];


}

boolean allDone() //all sensors have passed through
{
  boolean output = true;

  for (int j2 = 0; j2 < sensors; j2++)
  {
    if (passArray[j2] == false) {
      output = false; // if even one is false, the total result is false.
    }
  }
  return output;
}

void BTSerialPrint()
{
  for (int j = 0; j < sensors; j++)
  {
    BTSerial.print("Time at Sensor"); BTSerial.print('['); BTSerial.print(j); BTSerial.print(']'); BTSerial.print(": "); BTSerial.println(timeArray[2 * j]);

  }
}

void SerialPrint()

{
  float accSum = 0.0;

  Serial.println("----");

  for (int j = 0; j < slots; j++)
  {
    // Serial.print("timeArray"); Serial.print('['); Serial.print(j); Serial.print(']'); Serial.print(":"); Serial.println(timeArray[j]);
  }
  Serial.println("--"); Serial.println("--");

  for (int j = 1; j < sensors-1; j++)
  {


    accl[j] = (speedArray[j+1]-speedArray[j])/(AvgTime[j+1]-AvgTime[j]);
    accSum = accSum + accl[j];
    //Serial.print(speedArray[j],4); Serial.print("  and  "); Serial.println(speedArray[j+1],4);
   // Serial.print(AvgTime[j],4); Serial.print(" and "); Serial.println(AvgTime[j+1],4);
    Serial.print(" Acc.betw mid-pt of Sensors:"); Serial.print(pinArray[j-1]); Serial.print("&"); Serial.print(pinArray[j]);Serial.print (" and mid-pt-of "); Serial.print(pinArray[j]);Serial.print("&");Serial.print(pinArray[j+1]); Serial.print(" is:   "); Serial.print(accl[j],4); Serial.println (" cm/sec^2");
  }

  for (int j1 = 0; j1 < sensors; j1++)
  {
    passArray[j1] = false;
  }
  Serial.print("----"); Serial.print("\t"); Serial.println("----");
  Serial.print ("                      Average Acc ="); Serial.print(accSum / (sensors - 2),4);  Serial.println (" cm/sec^2");




}

void loop() {
  // read the state of the pushbutton value:



  for (int x = 0; x < sensors; x++)
  {
    processSensorPinat (x);
  }

  if  (allDone())

  { SerialPrint();
    BTSerialPrint();
    Serial.print("----"); Serial.print("\t"); Serial.println("----");
  }


}
