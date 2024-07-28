/* Control Test
 *  Tune controller gains
 */

#include <avr/sleep.h>  

//======Advisor======
const int numEncoders = 2;

int eStop, // global flag to halt robot
  setPosActive,
  setVelActive;

volatile bool newPulses[] = { false, false },
  validPulses[numEncoders];

//======Interface======
volatile String inputString = "";

volatile boolean stringComplete = false;  // whether the string is complete

//======Encoders======
const byte esPins[] = 
{
  2, // encoder signal 1 pin
  3 // encoder signal 2 pin
};

const byte pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

volatile int angPos[numEncoders],
  prevAngPos[numEncoders],
  setPos[numEncoders];

volatile long pulseCounts[numEncoders];

//======Motor Driver======
const byte mSigPins[] = { 8, 9 },
  mEnablePins[] = { 6, 7 };

//======Mobile Platform======
byte wheelDiam = 64; // [mm]

//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
int sensorsTmrCtr,
  maxOutVal,
  pubVelRate,
  posDeadZone,
  minStaticDutyCycle,
  minStaticMtrCmd,
  minKineticDutyCycle,
  minKineticMtrCmd;

unsigned long prevSenseTime;

unsigned long curTimes[numEncoders];

long prevPulseTimes[numEncoders];

float minLinearRes;

volatile bool decelerating[] = { false, false };

volatile float kp[numEncoders], ki[numEncoders], kd[numEncoders];

volatile int topAngVel, // [deg/s]
  setFwdVel, // [mm/s]
  setYawVel; // [deg/s]

volatile int prevAngVels[numEncoders],
  angVels[numEncoders],
  setAngVels[numEncoders], // [deg/s]
  cmdAngVels[numEncoders], // [deg/s]
  cmdPrcntMtrVltges[numEncoders],
  prevCmdPMV[numEncoders],
  pubMtrCmds[numEncoders],
  signs[numEncoders],
  //setPrcntMtrVltge,
  pulses[numEncoders],
  prevPulses[numEncoders],
  prevPosErrs[numEncoders],
  posErrs[numEncoders],
  prevVelErrs[numEncoders],
  velErrs[numEncoders],
  bounceTimes[numEncoders];

volatile long instance;

volatile long samples[numEncoders],
  prevSamples[numEncoders];

//======Main======
void setup() 
{
  initSystem();

  initBehaviors();

  //initSensorsTimer(); // CHANGE to scheduler(); ADD: publish rotational (and later translational) angVel
}

int initSystem()
{
  initNode("VelControlTest5");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(115200);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");

  displayCommands();

  Serial.println("Inst\tTime1\tSamp1\tPuls1\tSetV1\tActV1\tVErr1\tCmdV1\tCmdPMV1\tMtrCmd1\tTime2\tSamp2\tPuls2\tSetV2\tActV2\tVErr2\tCmdV2\tCmdPMV2\tMtrCmd2");
}

void displayCommands()
{
  Serial.println("======User Commands======");
  Serial.println("s: stop moving (i.e. emergency stop robot)");
  // ADD for fwd displacement ctrl: Serial.println("fdx: move forward x mm");
  // ADD for turn displacement ctrl: Serial.println("tdx: turn x deg");
  Serial.println("fvx: move forward x mm/s");
  Serial.println("tvx: turn x deg/s");
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  /* Interrupt flags can be set before you attach the 
   * interrupt handler. To avoid this, manually clear 
   * the flag. 
   */
  EIFR = bit (INTF4); // clear flag for interrupt 4
  EIFR = bit (INTF5); // clear flag for interrupt 5
  
  attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);

  //Serial.println("Subscribers Inititialized");
}

void initPublishers()
{
  /* Start Motor Channel */
  for(int i=0; i < numEncoders; i++)
  {
    pinMode(mEnablePins[i], OUTPUT);
    pinMode(mSigPins[i], OUTPUT);
  }

  //Serial.println("Publishers Inititialized");
}

void initBehaviors()
{
  initVars();

  setParams();

  //Serial.println("Behaviors Inititialized");
}

void initVars()
{ 
  //===Controller===
  topAngVel = 360; // [deg/s]
  instance = 0;
  prevSenseTime = 0;
  
  for(int i=0; i < numEncoders; i++)
  {
    //===Encoder===
    pulseCounts[i] = 0;
    
    bounceTimes[i] = 50; // [ms]
    validPulses[i] = true;
  
    prevAngPos[i] = 0;
    angPos[i] = 0;
  
    prevAngVels[i] = 0;
    angVels[i] = 0;

    prevPulseTimes[i] = 0; // [ms]
    curTimes[i] = 0;

    //===Controller===
    setAngVels[i] = 0; // [deg/s]
    cmdAngVels[i] = 0; // [deg/s]
    prevVelErrs[i] = 0;
    velErrs[i] = 0;
  
    setPos[i] = 0;
    prevPosErrs[i] = 0;
    posErrs[i] = 0;
  
    // ADD for vel ctrl: setPrcntMtrVltge = 0; // %, duty cycle, proportional to percent of time motor turned on
    cmdPrcntMtrVltges[i] = 0;
    prevCmdPMV[i] = 0;
    pubMtrCmds[i] = 0;
    
    signs[i] = 1;
  
    prevSamples[i] = 0;
    samples[i] = 0;
    pulses[i] = 0;
    prevPulses[i] = 0;
  }

  //Serial.println("Variables Inititialized");
}

void setParams()
{
  float setKp[] = { 0.050, 0.050 },
    setKi[] = { 0.000, 0.000 },
    setKd[] = { 0.000, 0.000 };

  setM1PIDGains(setKp[0], setKi[0], setKd[0]); // m1
  setM2PIDGains(setKp[1], setKi[1], setKd[1]); // m2
  //Serial.println("PID Gains Set");
    
  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;

  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  eStop = 1;

  setPosActive = 0;
  setVelActive = 0;

  posDeadZone = 9; // [deg], CHANGE: tune based on resolution

  minStaticDutyCycle = 35; // [%] for M2 Decrementing by 1 from 35, 35% for M2 incrementing from 0 by 1; min duty cycle determined by OpenLoopControl.ino
  minStaticMtrCmd = (int) round( 255 * minStaticDutyCycle / 100.0 );
  minKineticDutyCycle = 14; // [%] for M2 Decrementing by 1 from 35, 35% for M2 incrementing from 0 by 1; min duty cycle determined by OpenLoopControl.ino
  minKineticMtrCmd = (int) round( 255 * minKineticDutyCycle / 100.0 );

  // diff drive:
  setFwdVel = 0;
  setYawVel = 0;

  //Serial.println("Parameters Set");
}

void setM1PIDGains(float pg, float ig, float dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

  kp[0] = pg;
  
  ki[0] = ig;

  kd[0] = dg;
}

void setM2PIDGains(float pg, float ig, float dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

  kp[1] = pg;
  
  ki[1] = ig;

  kd[1] = dg;
}

//void initSensorsTimer()
//{
//  noInterrupts();           // disable all interrupts
//  
//  TCCR1A = 0;
//  TCCR1B = 0;
//  sensorsTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
//  
//  TCNT1 = sensorsTmrCtr;   // preload timer
//  TCCR1B |= (1 << CS12);    // 256 prescaler 
//  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
//  
//  interrupts();             // enable all interrupts
//}

void loop() 
{
  int i;
  
  for(i=0; i < numEncoders; i++)
  { 
    if(decelerating[i] || pulses[i] == 0 || prevPulses[i] == 0)
      bounceTimes[i] = 50; // [ms]
    else
      bounceTimes[i] = 35; // [ms]
      
    curTimes[i] = millis();
    
    if(curTimes[i] - prevPulseTimes[i] > bounceTimes[i]) // if the noise is gone
    {
      noInterrupts();
      
      validPulses[i] = true;
      
      if(newPulses[i])
      {
        //Serial.println("===New Valid Pulse Detected===");
  
        prevPulseTimes[i] = curTimes[i]; // record the time of the first pulse in bounce group
        
        newPulses[i] = false;
      }

      interrupts();
    }
  }

  Serial.print(instance);
  Serial.print("\t");
  
  for(i=0; i < numEncoders; i++)
  {
    Serial.print(curTimes[i]);
    Serial.print("\t");
    Serial.print(samples[i]);
    Serial.print("\t");
    Serial.print(pulses[i]);
    Serial.print("\t");
    Serial.print(setAngVels[i]);
    Serial.print("\t");
    Serial.print(angVels[i]);
    Serial.print("\t");
    Serial.print(velErrs[i]);
    Serial.print("\t");
    Serial.print(cmdAngVels[i]);
    Serial.print("\t");
    Serial.print(cmdPrcntMtrVltges[i]);
    Serial.print("\t");
    Serial.print(pubMtrCmds[i]);
    Serial.print("\t");
  }
  Serial.println();

  if(millis() - prevSenseTime > 100)
  {
    userTask(); 
    
    sensorsTask();
  
    mtrCmd();
  
    instance++;

    prevSenseTime = millis();
  }
}

void userTask()
{
  if(stringComplete)
  {
    //Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "s")
    {
      //Serial.println("stop");

      eStop = 1;
    }
    else if(inputString.substring(0,2) == "fv") // given in mm/s
    { 
      setFwdVel = inputString.substring(2, inputString.length()).toInt(); // get string after 'f'
      
//      Serial.print("move forward ");
//      Serial.print(setFwdVel);
//      Serial.println(" mm/s\n");

      computeWheelVels();

      for(int i=0; i < numEncoders; i++)
        cmdPrcntMtrVltges[i] = map(setAngVels[i], -topAngVel, topAngVel, -100, 100); // convert to % mtr voltage
      
      setVelActive = 1;
      
      eStop = 0;
    }
    else if(inputString.substring(0,2) == "tv") // given in deg/s
    { 
      setYawVel = inputString.substring(2, inputString.length()).toInt(); // get string after 'f'
      
//      Serial.print("turn ");
//      Serial.print(setYawVel);
//      Serial.println(" deg/s\n");

      computeWheelVels();

      for(int i=0; i < numEncoders; i++)
        cmdPrcntMtrVltges[i] = map(setAngVels[i], -topAngVel, topAngVel, -100, 100); // convert to % mtr voltage
      
      setVelActive = 1;
      
      eStop = 0;
    }
    
    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

void computeWheelVels()
{
  int minRes = 180,
    lowerBound,
    upperBound;

  setAngVels[0] = (int) round( ( setFwdVel / minLinearRes + setYawVel / minAngularRes ) * minAngularRes ); // [deg/s]
  setAngVels[1] = (int) round( ( setFwdVel / minLinearRes - setYawVel / minAngularRes ) * minAngularRes ); // [deg/s]

  for(int i=0; i < numEncoders; i++)
  {
    if(setAngVels[i] != 0)
    {
      setAngVels[i] = clip(setAngVels[i], topAngVel, -topAngVel);
    
      if(abs(setAngVels[i]) < minRes) 
        setAngVels[i] = minRes;
  
      else
      {
        lowerBound = minRes * floor( setAngVels[i] / minRes );
        upperBound = lowerBound + minRes;
  
        setAngVels[i] = abs(lowerBound - setAngVels[i]) 
                      < abs(setAngVels[i] - upperBound)
                      ? lowerBound : upperBound;
      }
    }
  }
}

void sensorsTask()
{

//  if(setPosActive)
    //{
//    readPosition();
//    locateTarget();
    //}

  if(setVelActive)
  {
    for(int i=0; i < numEncoders; i++)
    {
      observeSample(i);

      cmdAngVels[i] = setAngVels[i];
      
      computeMtrCtrlSgnl(i); // assume set vel active b/c vel ctrl test
    }
  }
  
//  if(setPosActive)
//  {
//  if(abs(posErr) > posDeadZone)
//  {
//    Serial.print(abs(posErr));
//    Serial.println(" deg > posDeadZone");
//    Serial.print("===Rotate to ");
//    Serial.print(setPos);
//    Serial.println(" deg===");
//
//    if(decelerating && pulses != 0) // if mtr not fully at rest yet
//      cmdAngVel = 0; // ensure mtr does not move until 0 vel detected after decelerating
//    else if(decelerating && pulses == 0 && prevPulses != 0)
//      cmdAngVel = 0; // ensure mtr does not move until 0 vel detected after decelerating
//    else
//      cmdAngVel = setAngVel; // may only work for indep mode
//  }
//  else // found target
//  {
//    cmdAngVel = 0;
//    cmdPrcntMtrVltge = 0; // may not need to explicitly set to 0 if cmdAngVel auto-changes this quickly enough 
//    pubMtrCmd = 0; // may not need to explicitly set to 0 if cmdAngVel auto-changes this quickly enough 
//  
////    if(pulses == 0 && prevPulses == 0)
////      setPosActive = 0;
//  }
  //}
}
  
// get displacement of each wheel to get this instant's
// linear and angular displacements
/* Read and zero pulses.
 * Copy and accumulate counts from pulses
 * to the ang. vel. variable and
 * then reset pulses to zero.
 */
void observeSample(int mid)
{
  //Serial.println("======Observe Sample======");

  noInterrupts();
  
  samples[mid] = pulseCounts[mid];

  interrupts();

//  Serial.print("samples[");
//  Serial.print(mid);
//  Serial.print("] = pulseCounts[");
//  Serial.print(mid);
//  Serial.print("] = ");
//  Serial.print(samples[mid]);
//  Serial.println(" pulses");
  
  //  Serial.print("samples[0]: ");
  //  Serial.println(samples[0]);
  //angPos[i] = (int) round( minAngularRes * ( samples[i] % pulsesPerRev ) ); // [deg]
  
  prevPulses[mid] = pulses[mid];
  pulses[mid] = samples[mid] - prevSamples[mid]; // P pulses

  angVels[mid] = (int) round( minAngularRes * pulses[mid] * pubVelRate); // [deg/s]
  
  //angVel = minAngularRes * pulses * sign * pubVelRate; // [deg/s], converted from pulses
  //    Serial.print("angVels[");
  //    Serial.print(i);
  //    Serial.print("] = minAngularRes * pulses[");
  //    Serial.print(i);
  //    Serial.print("] * signs[");
  //    Serial.print(i);
  //    Serial.print("] * pubVelRate = ");
  //    Serial.print(minAngularRes);
  //    Serial.print(" deg/pulse * ");
  //    Serial.print(pulses[i]);
  //    Serial.print(" pulses * ");
  //    Serial.print(signs[i]);
  //    Serial.print(" * ");
  //    Serial.print(pubVelRate);
  //    Serial.print(" Hz = ");
  //    Serial.print(angVels[i]);
  //    Serial.println(" deg/s");
    
  prevSamples[mid] = samples[mid];
}

void computeMtrCtrlSgnl(int mid)
{
  long errs[numEncoders], 
    dAngVels[numEncoders], 
    P[numEncoders], 
    I[numEncoders], 
    D[numEncoders];

//  Serial.print("======Locate Target (");
//  Serial.print(setAngVel);
//  Serial.println(" deg/s)======");

  velErrs[mid] = cmdAngVels[mid] - angVels[mid];
  
  errs[mid] = (long) round( velErrs[mid] / degsPerRad * 256 ); // [rad/s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
  //  Serial.print("errs[");
  //  Serial.print(i);
  //  Serial.print("] = ( b * setAngVels[");
  //  Serial.print(i);
  //  Serial.print("] - angVels[");
  //  Serial.print(i);
  //  Serial.print("] ) / degsPerRad = ( ");
  //  Serial.print(b);
  //  Serial.print(" * ");
  //  Serial.print(setAngVels[i]);
  //  Serial.print(" deg/s - ");
  //  Serial.print(angVels[i]);
  //  Serial.print(" deg/s ) / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(b * setAngVels[i] - angVels[i]);
  //  Serial.print(" deg/s / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.println(" rad/s");
    
  P[mid] = (long) round( kp[mid] * errs[mid] * pubVelRate ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
  //  Serial.print("P[");
  //  Serial.print(i);
  //  Serial.print("] = kp[1][");
  //  Serial.print(i);
  //  Serial.print("] * errs[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print(kp[0][i]);
  //  Serial.print(" % * ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.print(" rad/s = ");
  //  Serial.print(P[i] / 256.0);
  //  Serial.println(" %/s");
  
  //    if(abs(errs[i]) < threshIntegral)
  //    {
  //      I[i] += (long) round( ki * errs[i] );
  //
  //      I[i] = clip(I[i], maxOutVal, -maxOutVal);
  //    }
  //    else
  //      I[i] = 0;
  //
  dAngVels[mid] = (long) round( ( angVels[mid] - prevAngVels[mid] ) / degsPerRad * 256 );
  //  Serial.print("dAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ( angVels[");
  //  Serial.print(i);
  //  Serial.print("] - prevAngVels[");
  //  Serial.print(i);
  //  Serial.print("] ) / degsPerRad = ( ");
  //  Serial.print(angVels[i]);
  //  Serial.print(" deg/s - ");
  //  Serial.print(prevAngVels[i]);
  //  Serial.print(" deg/s ) / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(dAngVels[i] / 256.0);
  //  Serial.println(" rad/s");
    
  D[mid] = (long) round( kd[mid] * dAngVels[mid] * pubVelRate ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
  //  Serial.print("D[");
  //  Serial.print(i);
  //  Serial.print("] = kd[1][");
  //  Serial.print(i);
  //  Serial.print("] * dAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print(kd[1][i]);
  //  Serial.print(" %.s * ");
  //  Serial.print(dAngVels[i]);
  //  Serial.print(" rad/s / 1 s = ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.println(" %/s");
  
  //  Serial.print("mtrOut[");
  //  Serial.print(i);
  //  Serial.print("] = prevMtrOut[");
  //  Serial.print(i);
  //  Serial.print("] + P[");
  //  Serial.print(i);
  //  Serial.print("] + D[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print((int) round( mtrOut[i] / 256.0 ));
  //  Serial.print(" %/s + ");
      
  cmdPrcntMtrVltges[mid] += (int) round( ( P[mid] + D[mid] ) / 256.0 ); // [%/s], + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
  //  Serial.print(P[i] / 256.0);
  //  Serial.print(" %/s + ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.print(" %/s = ");
  //  Serial.print((int) round( mtrOut[i] / 256.0 ));
  //  Serial.println(" %/s");
    
  prevAngVels[mid] = angVels[mid]; // maintain history of previous measured rotVel
  
  cmdPrcntMtrVltges[mid] = clip(cmdPrcntMtrVltges[mid], 100, -100); // [%/s]
  //  Serial.print("mtrOut[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print((int) round( mtrOut[i] / 256.0 ));
  //  Serial.println(" %/s");

//  if(prevCmdPMV[mid] > cmdPrcntMtrVltges[mid])
//    decelerating[mid] = true;
//  else
//    decelerating[mid] = false;
//
//  prevCmdPMV[mid] = cmdPrcntMtrVltges[mid];
}

void locateTarget()
{
  long errs[numEncoders], 
    dAngPos[numEncoders], 
    P[numEncoders], 
    I[numEncoders], 
    D[numEncoders];

//  Serial.print("======Locate Target (");
//  Serial.print(setPos);
//  Serial.println(" deg)======");

  computePosErrs();

  for(int i=0; i < numEncoders; i++)
  {
    errs[i] = (long) round( posErrs[i] / degsPerRad * 256 ); // [rad]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
  //  Serial.print("errs[");
  //  Serial.print(i);
  //  Serial.print("] = -posErrs[");
  //  Serial.print(i);
  //  Serial.print("] / degsPerRad = ");
  //  Serial.print(-posErrs[i]);
  //  Serial.print(" deg / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.println(" rad");
    
    P[i] = (long) round( kp[i] * errs[i] * pubVelRate ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
  //  Serial.print("P[");
  //  Serial.print(i);
  //  Serial.print("] = kp[0][");
  //  Serial.print(i);
  //  Serial.print("] * errs[");
  //  Serial.print(i);
  //  Serial.print("] * pubVelRate = ");
  //  Serial.print(kp[0][i]);
  //  Serial.print(" rad/s * ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.print(" rad * ");
  //  Serial.print(pubVelRate);
  //  Serial.print(" Hz = ");
  //  Serial.print(P[i] / 256.0);
  //  Serial.println(" rad/s");
  
  //    if(abs(errs[i]) < threshIntegral)
  //    {
  //      I[i] += (long) round( ki * errs[i] );
  //
  //      I[i] = clip(I[i], maxOutVal, -maxOutVal);
  //    }
  //    else
  //      I[i] = 0;
  //
    dAngPos[i] = (long) round( ( angPos[i] - prevAngPos[i] ) / degsPerRad * 256 );
  //  Serial.print("dAngPos[");
  //  Serial.print(i);
  //  Serial.print("] = ( angPos[");
  //  Serial.print(i);
  //  Serial.print("] - prevAngPos[");
  //  Serial.print(i);
  //  Serial.print("] ) / degsPerRad = ( ");
  //  Serial.print(angPos[i]);
  //  Serial.print(" deg - ");
  //  Serial.print(prevAngPos[i]);
  //  Serial.print(" deg ) / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(dAngPos[i] / 256.0);
  //  Serial.println(" rad");
    
    D[i] = (long) round( kd[i] * dAngPos[i] * pubVelRate ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
  //  Serial.print("D[");
  //  Serial.print(i);
  //  Serial.print("] = kd[0][");
  //  Serial.print(i);
  //  Serial.print("] * dAngPos[");
  //  Serial.print(i);
  //  Serial.print("] * pubVelRate = ");
  //  Serial.print(kd[0][i]);
  //  Serial.print(" rad * ");
  //  Serial.print(dAngPos[i] / 256.0);
  //  Serial.print(" rad * ");
  //  Serial.print(pubVelRate);
  //  Serial.print(" Hz = ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.println(" rad/s");
  
    // ADD: if vel control, output setAngVel to vel controller
    setAngVels[i] = (int) round( ( P[i] + D[i] ) * degsPerRad  / 256.0 ); // [deg/s], + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
  //  Serial.print("setAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ( P[");
  //  Serial.print(i);
  //  Serial.print("] + D[");
  //  Serial.print(i);
  //  Serial.print("] ) * degsPerRad / 256.0 = ( ");
  //  Serial.print(P[i] / 256.0);
  //  Serial.print(" rad/s + ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.print(" rad/s ) * ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(setAngVels[i]);
  //  Serial.println(" deg/s");
    
    prevAngPos[i] = angPos[i]; // maintain history of previous measured rotVel
  
    setAngVels[i] = clip(setAngVels[i], topAngVel, -topAngVel); // [deg/s]
  //  Serial.print("setAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print(setAngVels[i]);
  //  Serial.println(" deg/s");
  
    cmdPrcntMtrVltges[i] = map(setAngVels[i], -topAngVel, topAngVel, -100, 100);
  }
}

void computePosErrs()
{
  //Serial.println("===Compute Position Error===");

  int pe1, pe2, // pos errors
    b = 1; // set point weight
    
  float H = 1.0; // / (1 + timeConst); // sensor

  for(int i=0; i < numEncoders; i++)
  {
    pe1 = (int) round( b * H * setPos[i] - angPos[i] );
  //  Serial.print("pe1 = angPos[1] - setPos[1] = ");
  //  Serial.print(angPos[1]);
  //  Serial.print(" deg - ");
  //  Serial.print(setPos[1]);
  //  Serial.print(" deg = ");
  //  Serial.print(pe1);
  //  Serial.println(" deg");
  
    //Serial.print("pe2 = pe1 ");
    if(pe1 > 0)
    {
      pe2 = pe1 - 360; // [deg]
      //Serial.print("- ");
    }
    else
    {
      pe2 = pe1 + 360;
      //Serial.print("+ ");
    }
  
  //  Serial.print("360 deg = ");
  //  Serial.print(pe2);
  //  Serial.println(" deg");
  
    posErrs[i] = abs(pe1) > abs(pe2) ? pe2 : pe1;
  //  if(abs(pe1) > abs(pe2))
  //    posErr = pe2;
  //  else
  //    posErr = pe1;
  
  //  Serial.print("posErrs[1] = ");
  //  Serial.print(posErrs[1]);
  //  Serial.println(" deg");
  
    if(pulses[i] != 0 && prevPulses[i] != 0) // check that mtr is moving before checking if decelerating
    {
      // assume decelerating if passed target:
      if(prevPosErrs[i] <= 0 && posErrs[i] > 0) 
      {
        signs[i] = -1; // continue in same direc until at rest
        
        decelerating[i] = true;
      }
      else if(prevPosErrs[i] >= 0 && posErrs[i] < 0)
      {
        signs[i] = 1; // continue in same direc until at rest
        
        decelerating[i] = true;
      }
    }
  
    prevPosErrs[i] = posErrs[i];
  }
}

void mtrCmd()
{  
  if(eStop) emergencyStop();
  
  else 
  {
    // if only pos ctrl:
    //cmdPrcntMtrVltge = map(cmdAngVel, -topAngVel, topAngVel, -100, 100);
  
    modulatePulseWidths(cmdPrcntMtrVltges);
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{
  //Serial.print("Computed val: ");
  //Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  //Serial.print(", Clipped val: ");
  //Serial.println(a);

  return a;
}

void emergencyStop()
{
  for(int i=0; i < numEncoders; i++)
    digitalWrite(mEnablePins[i], LOW);
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  for(int i=0; i < numEncoders; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code
  
    setHBridgeDirectionBit(i, signedVals[i]);
    
    pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
  } 
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(!decelerating[mid])
  {
    // assume sign unchanged if signedVal=cmdPrcntMtrVltge=0
    if(signedVal < 0) // {motor direction of rotation} = backward
      signs[mid] = -1;
    else if(signedVal > 0)
      signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
    //else
      //Serial.println("Invalid command issued to PWM code!\n");
  }
//  Serial.print("M");
//  Serial.print(mid + 1);
//  Serial.print(" speedometer sign: ");
//  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  // definitely works for indep mode but may need to change for diff mode
  if(signedVal < 0) // {motor direction of rotation} = CW
  {
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH); // M1: CW = LOW; M2: CW = HIGH
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {CCW | resting}
  {
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  //else
    //Serial.println("Invalid command issued to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  if(magnitude == 0)
    return magnitude;
    
  else if(setPosActive && (pulses[mid] == 0 || prevPulses[mid] == 0))
    return map(magnitude, 0, 100, minStaticMtrCmd, 255);
    
  else if(pulses[mid] == 0 && prevPulses[mid] == 0)
    return map(magnitude, 0, 100, minStaticMtrCmd, 255);
    
  else
    return map(magnitude, 0, 100, minKineticMtrCmd, 255); // cruise outputs
}

//======Interrupt Service Routines======
void encoder1Callback()
{
  if(validPulses[0])
  {
//    if(decelerating[0] && pulses[0] == 0 && prevPulses[0] == 0) // wait for 2 readings of 0 vel before confirming at rest
//    {
//      signs[0] = -signs[0]; // change direction
//      
//      decelerating[0] = false;
//    }
    
    if(signs[0] == 1)
      pulseCounts[0]++;
    else
      pulseCounts[0]--;

    validPulses[0] = false;
  }

  newPulses[0] = true;
}

void encoder2Callback()
{
  if(validPulses[1])
  {
//    if(decelerating[1] && pulses[1] == 0 && prevPulses[1] == 0) // wait for 2 readings of 0 vel before confirming at rest
//    {
//      signs[1] = -signs[1]; // change direction
//      
//      decelerating[1] = false;
//    }
    
    if(signs[1] == 1)
      pulseCounts[1]++;
    else
      pulseCounts[1]--;

    validPulses[1] = false;
  }

  newPulses[1] = true;
}

/* Run Sensor Loop at x (maybe 10-20) Hz, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 */
//ISR(TIMER1_OVF_vect) // sensors loop!
//{
//  TCNT1 = sensorsTmrCtr; // set timer
//
//  userTask(); 
//  
//  sensorsTask();
//
//  mtrCmd();
//
//  instance++;
//}
