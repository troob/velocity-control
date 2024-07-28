/*
 * Test velocity calculation 
 * by running motors at arbitrary voltage,
 * outputting computed velocity from encoders,
 * and manually timing rotations to check computation.
 * Measure in rotations per second, 
 * b/c that's visually easiest to confirm
 */
 
//======Encoder======
const byte esPin = 3;

int pulsesPerRev = 20;

double minAngularRes,
  pulsesPerDeg,
  pubRotVel = 0.0, // [deg/s], PUBLISHER
  threshRotVel = 0.01, // [deg/s]
  prevRotPos; // [deg]

// values change in callback methods:
volatile long pulseCount,
  prevPulseCount;

volatile double latestRotPos; // [deg]

//======Motor Driver======
const byte mEnablePin = 6,
  mSigPin = 7;

//======Mobile Platform======
double wheelDiam = 6.35,
  wheelBase = 18.0;

//======Circle======
double piApprox = 3.14159,
  rads = 57.2958; // radians to deg conversion

//======Controller======
int desMtrPwrPrcnt,
  pubMtrCmd, // PUBLISHER
  outMin,
  outMax;

const int rollingPts = 2;

static unsigned long prevTime;

double prevPubRotVel[rollingPts] = {};

boolean forward = true;

void setup() 
{
  initNode("VelocityCalculationTest");

  initVars();

  setParams();

  initSubscribers();

  initPublishers();

  /* "spin()" */
  prevRotPos = latestRotPos;

  prevTime = millis();
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initVars()
{
  desMtrPwrPrcnt = 25; // CHANGE: update thru desMtrPwrPrcntCallback(msg)

  pubMtrCmd = 0;

  pubRotVel = 0.0;

  prevRotPos = 0.0;

  latestRotPos = 0.0;

  prevTime = millis();

  prevPulseCount = 0;
}

void setParams()
{
  outMin = 0;

  outMax = 255;

  computeMinAngularRes();

  threshRotVel = 0.01;

  for(int i=0; i < rollingPts; i++)
    prevPubRotVel[i] = 0.0;

  latestRotPos = 0.0; // confusing b/c already set in initVars()
}

void computeMinAngularRes()
{
  minAngularRes = 360.0 / pulsesPerRev;
  
  //pulsesPerDeg = pulsesPerRev / 360.0;
  
  //pulsesPerMeter = (int) ( (double) pulsesPerRev 
    // (piApprox * wheelDiam / 100.0 ) ); // [pulses/m] = (x [pulses/rev])/(\pi*d [m])

  Serial.print("Min. Ang. Res. (deg): ");
  Serial.println(minAngularRes);
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPin), encoderCallback, CHANGE);
}

void initPublishers()
{
  // motor command
  pinMode(mEnablePin, OUTPUT);
  pinMode(mSigPin, OUTPUT);

  // ADD: publish rotational (and later translational) velocity
}

void stopMoving()
{
  digitalWrite(mEnablePin, LOW);
}

/* "spinOnce()" */
void loop() 
{
  calcRotVel();

  calcMtrCmd();

  analogWrite(mEnablePin, pubMtrCmd); // pubMotor.publish(motor)
}

void calcRotVel()
{
  long dt; // [ms]
  
  double curRotVel;

  dt = millis() - prevTime; // [ms]

  double dtSec = dt / 1000.0;
  
  Serial.print("calcRotVel: dt = ");
  Serial.print(dtSec, 3);
  Serial.print(" s, latestRotPos = ");
  Serial.print(latestRotPos);
  Serial.print(" deg (");
  Serial.print(convertDegToRev(latestRotPos));
  Serial.print(" rev), prevRotPos = ");
  Serial.print(prevRotPos);
  Serial.println(" deg");

  if(dt > 0)
  {
    // we haven't received an updated wheel lately
    if(latestRotPos == prevRotPos)
    {
      curRotVel = 1000.0 * minAngularRes / dt; // if we got a pulse right now, this would be the rot. velocity
  
      // if the velocity is < threshold, consider our velocity 0
      if(abs(curRotVel) < threshRotVel)
      {
//        Serial.print("Below threshold: curRotVel=");
//        Serial.print(curRotVel);
//        Serial.println(" deg/s, pubRotVel = 0.0");
  
        calcRollingVel(0);
      }
      else
      {
//        Serial.print("Above threshold: curRotVel = ");
//        Serial.print(curRotVel);
//        Serial.println(" deg/s");
  
        if(abs(curRotVel) < pubRotVel) // we know we're slower than what we're currently publishing as a velocity
        {
          //Serial.println("curRotVel < pubRotVel");
          
          calcRollingVel(curRotVel);
        }
      }
    }
    else // we received a new pulse value
    {
      curRotVel = 1000.0 * ( latestRotPos - prevRotPos ) / dt;
  
//      Serial.print("Received New Pulse Value: curRotVel = ");
//      Serial.print(curRotVel);
//      Serial.println(" deg/s");
        
      calcRollingVel(curRotVel);
      
      Serial.print("*** Pulse Count Updated: pubRotVel = ");
      Serial.print(pubRotVel);
      Serial.println(" deg/s ***");
    
      prevRotPos = latestRotPos;
  
      prevTime = millis(); // [ms]
    }
  }
}

double convertDegToRev(double deg)
{
  return deg / 360.0;
}

void calcRollingVel(double val)
{
//  for(int i=0; i < rollingPts; i++)
//  {
//    Serial.print("prevPubRotVel[");
//    Serial.print(i);
//    Serial.print("]: ");
//    Serial.println(prevPubRotVel[i]);
//  }
  
  for(int i=0; i < rollingPts - 1; i++)
  {
    prevPubRotVel[i] = prevPubRotVel[i + 1];

//    Serial.print("Rolling Vel ");
//    Serial.print(i);
//    Serial.print(" (deg/s): ");
//    Serial.println(prevPubRotVel[i]);
  }

  prevPubRotVel[rollingPts - 1] = val;

//  Serial.print("Rolling Vel ");
//  Serial.print(rollingPts - 1);
//  Serial.print(" (deg/s): ");
//  Serial.println(val);

  pubRotVel = getMean(prevPubRotVel);
}

double getMean(double vals[])
{
  double sum;
  
  for(int i=0; i < rollingPts; i++)
    sum += vals[i];

  return sum / (double) rollingPts;
}

void calcMtrCmd()
{  
  pubMtrCmd = (int) ( 255.0 * ( desMtrPwrPrcnt / 100.0 ) ); // CHANGE: adjusted PID voltage

  if(pubMtrCmd <= 0)
  {
    digitalWrite(mSigPin, LOW);

    forward = false;
  }
  else
  {
    digitalWrite(mSigPin, HIGH);

    forward = true;
  }
    
  if(pubMtrCmd > outMax)
    pubMtrCmd = outMax;

  if(pubMtrCmd < outMin)
    pubMtrCmd = outMin;

  Serial.print("pubRotVel (deg/s): ");
  Serial.print(pubRotVel);
  Serial.print(" desMtrPwrPrcnt (%): ");
  Serial.print(desMtrPwrPrcnt);
  Serial.print(" pubMtrCmd (0-255): ");
  Serial.println(pubMtrCmd);
  Serial.println();
}

//======Interrupt Service Routines======
void encoderCallback()
{
  determinePulseCount(); // increment if forward, decrement if backward // OLD: pulseCount++;
    
  latestRotPos = (double) pulseCount * minAngularRes;
    
  prevPulseCount = pulseCount;
}

void determinePulseCount()
{
  if(forward)
    pulseCount++;
    
  else
    pulseCount--;

  Serial.print("Pulse Count: ");
  Serial.print(pulseCount);
  Serial.println();
}

// desMtrPwrPrcntCallback
//void serialEvent()
//{
//  while (Serial.available()) 
//  {
//    // get the new byte:
//    char inChar = (char) Serial.read();
//    
//    // add it to the inputString:
//    inputString += inChar;
//    
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n')
//      stringComplete = true;
//  }
//}
