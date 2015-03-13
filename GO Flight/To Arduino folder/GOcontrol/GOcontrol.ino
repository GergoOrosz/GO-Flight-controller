// GO Flight attitude control algorithm 1.0       Helicopter flybarless system based on two Arduino micros

//Copyright (C) <2015>  <Gegely Orosz> <gergo_orosz@me.com>

//This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.

//You should have received a copy of the GNU General Public License
//along with this program.  If not, see <http://www.gnu.org/licenses/>.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

// That's right, I have tried this flybarless in an Align Trex 250 helicopter, which is relatively cheap and harmless in my view.
// It did happen, that the unit stopped responding and threw the helicopter onto the ground. The unit is in experimental stage
// therefore I CAN ABSOLUTELY NOT GARANTEE THAT IT'S THE SAFEST TO TAKE OFF WITH. I ACCEPT NO LIABILITY AT ALL FOR ANY DAMAGE OR INJURY caused by this unit.
// As a precaution I have not included the trottle signal in the script, its controlled directly by the receiver.
// Failiure can also be the result of the quality of the solderwork. Loads of cables, 
// likely short in the structure stops the Arduinos and the helicopter falls.
// It is also important not to touch the helicopter until the servos are moving (approx 25 sec from powering up) so the sensor can calibrate itself
// If you decide to try this unit in a bigger helicopter please have thorough testing and leave even greater distance than usual! Good luck

// I have used a few Arduino libraries, therefore I would like to say thank you for the following people for their work and
// contribution for my project:

//   Jeff Rowberg for the i2cdevlib library
//   Michael Schwager, Lex Talonis, Chris J. Klick for the PinChangeInt library
//   Brett Beauregard for the Arduino PID library
//   Nick Gammon for the I2C send anything library
//   DuaneB for the RCArduinoFastLib

// For more information please look at the attached manual


#include <PinChangeInt.h>
#include <Wire.h>
#include <I2C_Anything.h>
#include <PID_v1.h>
#include <RCArduinoFastLib.h>


// Assign your channel in pins
#define Aileron 8
#define Elevator 9
#define Pitch 10
#define Rudder 11

// Assign your channel out pins, note that actual pins to be used do differ, see wire diagramm
#define Aileron_Out 4
#define Elevator_Out 3
#define Pitch_Out 6
#define Rudder_Out 2

// Supply trim and endpoint values to trim the system ( not the servos individually), values applied to my current setup

#define center 1500 // defines the center, keep it to be 1500
#define Ailtrim 0
#define Ailendpoint 0
#define Eletrim 0
#define Eleendpoint 0
#define Pittrim 0
#define Pitendpoint 0
#define Rudtrim 0
#define Rudendpoint 0

// Use these values for trimming servos individually

#define Leftservotrim 0
#define Rightservotrim 0
#define Backservotrim 0


// Adjust these values to set the flip,roll and piro rates, this basically sets the agility of the helicopter
#define Fliprate 6.00 
#define Rollrate 6.00 
#define Pirorate 17.00 
#define PWMtime 20/1000000 

// These values determine the deadband of the controls

#define Aildeadband 15
#define Eledeadband 15
#define Ruddeadband 15

// Aileron gain section

#define Ailgain 0.9 // proportional gain
#define AilIgain 2.0 // integral gain, which determines the Ail position holding power of the system
#define Ailstopgain 0.02 // derivative gain, effective on stops

// Elevator gain section

#define Elegain 1.8 // proportional gain
#define Pitchupgain 2.9 // integral gain, which determines the Ele position holding power of the system a.k.a. pitch up compensation
#define Elestopgain 0.02 // derivative gain, effective on stops

// Rudder gain section

#define RudPgain 14 //proportional gain of the tail          Much higher gain values are required as the resolution of RUDPID is greater
#define RudHHgain 128 // Heading-Hold (Avcs) gain of the tail
#define Rudstopgain 5 // 1.25 Rudder stop gain

// Adaptive tuning parameters are applied in the below section. If the difference between the commanded rate and the measured rate is greater than
// AIL ELE RUDgainchange, the tuning values are multiplied by agrgain. THerefore for instance if it is set so, at greater errors different 
//tuning parameters are applied. =1000 indicates, that this function is turned off

#define agrgain 1.2

#define Ailgainchange 1000
#define Elegainchange 1000
#define Rudgainchange 10

// Control symmetry section If parameters are changed from one, it will affect rates for different directions. Note, this function is not active
// at the moment, it is needed to be commented out

#define Ailleft 1
#define Ailright 1
#define Elefor 1
#define Eleback 1
#define Rudleft 1
#define Rudright 1

// Precompensation section

#define Pitchprecomp 0.1 // Mixes a certain amount of Pitch to Tail
#define Cyclicprecomp 0.05 // Mixes a certain amount of Cyclic to Tail
#define Eleprecomp 0.05 // Elevator precompensation, Mixes a bit of Elevator to Pitch

//  Feedforvard amount determines how much cyclic command is applied directly to the swashplate
#define Feedforward 0.7

// Flybar parameter determines the amount of angular decay, not yet connected to PID.cpp its needed to be changed manually at line 
//ITerm+=(ki*error)-round(ITerm*0.01); in PID.cpp
#define flybarparam 0.01

// reverses the correction direction of controls. If does not work, change the cor sign in the AILin Elein Rudservo section

#define Ailcorreverse 1
#define Elecorreverse 1
#define Rudcorreverse -1

// Assign servo indexes
#define SERVO_AIL 0
#define SERVO_ELE 1
#define SERVO_PIT 2
#define SERVO_RUD 3
#define SERVO_FRAME_SPACE 5

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define AIL_FLAG 1
#define ELE_FLAG 2
#define PIT_FLAG 4
#define RUD_FLAG 8


// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unAileronInShared;
volatile uint16_t unElevatorInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRudderInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint16_t unAileronInStart;
uint16_t unElevatorInStart;
uint16_t unPitchInStart;
uint16_t unRudderInStart;

uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

const byte MY_ADDRESS = 42;
double Ailin;
double Elein;
double Pitin;
double Rudin;
double Leftservo;
double Rightservo;
double Backservo;
double RudOut;
double Ailcom = 0.00;
double Elecom = 0.00;
double Rudcom  =0.00;
double Ailcor=0;
double Elecor=0;
double Rudcor=0;
double Ailpure = 0;
double Elepure = 0;
double Pitpure = 0;
double Ailcormap;
double Elecormap;
double Rudcormap;
double LastAilcormap =0;
double LastElecormap=0;
double aggKpA=Ailgain*agrgain, aggKiA=AilIgain*agrgain, aggKdA=Ailstopgain;
double consKpA=Ailgain, consKiA=AilIgain, consKdA=Ailstopgain;
double aggKpE=Elegain*agrgain, aggKiE=Pitchupgain*agrgain, aggKdE=Elestopgain;
double consKpE=Elegain, consKiE=Pitchupgain, consKdE=Elestopgain;
double aggKpR=RudPgain/agrgain*0.5, aggKiR=RudHHgain*agrgain, aggKdR=Rudstopgain*0.1;
double consKpR=RudPgain, consKiR=RudHHgain, consKdR=Rudstopgain*0.1;
double Ailgap=0;
double Elegap=0;
double Rudgap=0;
double Wxbi = 0;
double Wybi = 0;
double Wzbi = 0;
double Rudprev;
long lastMillis = 0;
float dt = 0;


PID AilPID(&Wxbi, &Ailcor, &Ailcom, consKpA, consKiA, consKdA, DIRECT,flybarparam);
PID ElePID(&Wybi, &Elecor, &Elecom, consKpE, consKiE, consKdE, DIRECT,flybarparam);
PID RudPID(&Wzbi, &Rudcor, &Rudcom, consKpR, consKiR, consKdR, DIRECT,flybarparam);



void setup()
{

  
  Wire.begin (MY_ADDRESS);
  //Serial.begin (115200);
  Wire.onReceive (receiveEvent);
  // Limiting the outputs of the PID controllers
  AilPID.SetOutputLimits(-400, 400);
  ElePID.SetOutputLimits(-400, 400);
  RudPID.SetOutputLimits(-1600, 1600);

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers 
  CRCArduinoFastServos::attach(SERVO_AIL,Aileron_Out);
  CRCArduinoFastServos::attach(SERVO_ELE,Elevator_Out);
  CRCArduinoFastServos::attach(SERVO_PIT,Pitch_Out);
  CRCArduinoFastServos::attach(SERVO_RUD,Rudder_Out);
  
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,6*2000);

  CRCArduinoFastServos::begin();
  
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(Aileron, calcAileron,CHANGE);
  PCintPort::attachInterrupt(Elevator, calcElevator,CHANGE);
  PCintPort::attachInterrupt(Pitch, calcPitch,CHANGE);
  PCintPort::attachInterrupt(Rudder, calcRudder,CHANGE);

// Automatic means that the PID is on
  AilPID.SetMode(AUTOMATIC);
  ElePID.SetMode(AUTOMATIC);
  RudPID.SetMode(AUTOMATIC);
  

  // This delay is added so the sensor can calibrate itself, do not touch the helicopter while the servos are not moving
  delay(20000);
 
  
}

volatile boolean haveData = false;
volatile float Wx;
volatile float Wy;
volatile float Wz;
 

void loop()
{ 
  long currentMillis = millis();  
  digitalWrite(13,HIGH);
  // Receiving data from the sensor arduino, measured and desired rates are multiplied by 10 just for simply having greater gain values to work with.
  if (haveData)
    {
     /* Serial.print("Ang vel.\t");
      Serial.print(Wx);
      Serial.print("\t");
      Serial.print(Wy);
      Serial.print("\t");
      Serial.println(Wz);*/
      Wxbi = Wx*Ailcorreverse*10;
      Wybi =Wy*Elecorreverse*10;
      Wzbi = Wz*Rudcorreverse*10;
    haveData = false;  
    }  // end if haveData
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unAileronIn;
  static uint16_t unElevatorIn;
  static uint16_t unPitchIn;
  static uint16_t unRudderIn;
  static uint16_t Ailservo;
  static uint16_t Eleservo;
  static uint16_t Pitservo;
  static uint16_t Rudservo;

  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
   
    if(bUpdateFlags & AIL_FLAG)
    {
      unAileronIn = unAileronInShared;
    }
   
    if(bUpdateFlags & ELE_FLAG)
    {
      unElevatorIn = unElevatorInShared;
    }
   
    if(bUpdateFlags & PIT_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
    
     if(bUpdateFlags & RUD_FLAG)
    {
      unRudderIn = unRudderInShared;
    }
    
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
   
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  
  // deadband calculation
  
  if (unElevatorIn > 1500-Eledeadband && unElevatorIn < 1500+Eledeadband)
  {
    unElevatorIn = 1500;
  }
   if (unAileronIn > 1500-Aildeadband && unAileronIn < 1500+Aildeadband)
  {
    unAileronIn = 1500;
  }
   if (unRudderIn > 1500-Ruddeadband && unRudderIn < 1500+Ruddeadband)
  {
    unRudderIn = 1500;
  }
  
  // if signal is lost, set everything to mid position
  
  if (unAileronIn ==0){
    unAileronIn = 1500;
  }
  
  if (unElevatorIn ==0){
    unElevatorIn = 1500;
  }
  
  if (unPitchIn ==0){
    unPitchIn = 1500;
  }
  
  if (unRudderIn ==0){
    unRudderIn = 1500;
  }

  
  // Angular velocity command calculations
  Ailcom =(map(unAileronIn,1100,1900,-100,100))*Rollrate/10;
  Elecom =(map(unElevatorIn,1100,1900,-100,100))*Fliprate/10;
  Rudcom =(map(unRudderIn,1100,1900,-100,100))*Pirorate/10;
  

  // PID tuning parameter choice (by looking at the error)
  Ailgap= abs(Ailcom-Wxbi); //distance away from setpoint
  if(Ailgap<Ailgainchange)
  {  //we're close to setpoint, use conservative tuning parameters
    AilPID.SetTunings(consKpA, consKiA, consKdA);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     AilPID.SetTunings(aggKpA, aggKiA, aggKdA);
  }
  
    Elegap= abs(Elecom-Wybi); //distance away from setpoint
  if(Elegap<Elegainchange)
  {  //we're close to setpoint, use conservative tuning parameters
    ElePID.SetTunings(consKpE, consKiE, consKdE);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     ElePID.SetTunings(aggKpE, aggKiE, aggKdE);
  }
  
    Rudgap= abs(Rudcom-Wzbi); //distance away from setpoint
  if(Rudgap<Rudgainchange)
  {  //we're close to setpoint, use conservative tuning parameters
    RudPID.SetTunings(consKpR, consKiR, consKdR);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     RudPID.SetTunings(aggKpR, aggKiR, aggKdR);
  }
  
  // COmpute the PID values
  AilPID.Compute();
  ElePID.Compute();
  RudPID.Compute();
  
  
  
  // Limiting the correction to -400 400, or any other value
  Ailcormap = map(Ailcor,-400,400,-400,400);
  Elecormap = map(Elecor,-400,400,-400,400);
  Rudcormap = map(Rudcor,-1600,1600,1100+Rudtrim-Rudendpoint,1900+Rudtrim-Rudendpoint);
  
  
  // Calculating the direct AIL ELE and PIT which will be passed on to the control directly
  Ailin = map(unAileronIn+Ailtrim,1100+Ailtrim,1900+Ailtrim,400+Ailtrim+Ailendpoint,-400+Ailtrim-Ailendpoint);
  Elein = map(unElevatorIn+Eletrim,1100+Eletrim,1900+Eletrim,-400+Eletrim-Eleendpoint,400+Eletrim+Eleendpoint);
  Pitin = map(unPitchIn+Pittrim,1100 + Pittrim,1900+Pittrim,-400+Pittrim-Pitendpoint,400+Pittrim+Pitendpoint);
  // Calculating Pure command values for calculations and manipulation
  Rudin = map(unRudderIn,1100,1900,-400,400);
  Ailpure = map(unAileronIn,1100,1900,-400,400);
  Elepure = map(unElevatorIn,1100,1900,-400,400);
  Pitpure = map(unPitchIn,1100,1900,-400,400);
  
  // Determining the AIL ELE and RUD signals to get into the mix
  Ailin = Ailin*Feedforward - Ailcormap;
  Elein = Elein*Feedforward + Elecormap + round(Eleprecomp*abs(Pitpure));
  Rudservo =  Rudcormap + round(Cyclicprecomp*(abs(Ailpure)+abs(Elepure)))+round(Pitchprecomp*abs(Pitpure));//+Feedforward*Rudin;
  
  
  // Uncomment this section if symmetry adjustment is required
 /* if (Ailpure>0){
    Ailin = Ailin*Ailright;
  }
  if (Ailpure<0){
    Ailin = Ailin*Ailleft;
  }
  
   if (Elepure>0){
    Elein = Elein*Elefor;
  }
  if (Elepure<0){
    Elein = Elein*Eleback;
  }
  
   if (Rudin>0){
    Rudservo = Rudservo*Rudright;
  }
    if (Rudin<0){
    Rudservo = Rudservo*Rudleft;
  }
  
  */
 /* Serial.print("cor/t    ");
  Serial.print(Ailin);
  Serial.print("\t");
  Serial.print(Elein);
  Serial.print("\t");
  Serial.print(Rudservo);
  Serial.print("\t");*/
  /*Serial.print(AilPID.GetKi());
  Serial.print("\t");
  Serial.print(RudPID.GetKp());
  Serial.print("\t");
  Serial.println(RudPID.GetKi());*/
 
  
  
  Serial.print("Correction    ");
  Serial.print(unRudderIn);
  Serial.print("\t");
  Serial.print(unElevatorIn);
  Serial.print("\t");
  Serial.println(unPitchIn);
  
  // Servo travel limiter
  if (Ailin>400+Ailtrim+Ailendpoint){
    Ailin = 400+Ailtrim+Ailendpoint;
  }
  if (Ailin<-400+Ailtrim-Ailendpoint){
    Ailin = -400+Ailtrim-Ailendpoint;
  }
  if (Elein>400+Eletrim+Eleendpoint){
    Elein = 400+Eletrim+Eleendpoint;
  }
  if (Elein<-400+Eletrim-Eleendpoint){
    Elein = -400+Eletrim-Eleendpoint;
  }
  if (Rudin>400+Rudtrim+Rudendpoint){
    Rudin = 400+Rudtrim+Rudendpoint;
  }
  if (Rudin<-400+Rudtrim-Rudendpoint){
    Rudin = -400+Rudtrim+Rudendpoint;
  }
  
  //Rear servo = Pitch + (Aileron x SIN(0)) + (Elevator x COS(0))
  //Left servo = Pitch + (Aileron x SIN(120)) - (Elevator x COS(120))
  //Right servo = Pitch - (Aileron x SIN(-120)) - (Elevator x COS(-120))
  
  // 120 degrees standard CCPM mix according to the formula above, change sin and cos values to get other type of mixes eg 140degrees
  Leftservo = map(Pitin -round(Ailin*0.866) - round(Elein*0.5),-1200,1200,2000+Leftservotrim,1000+Leftservotrim);
  Rightservo =map(Pitin +round(Ailin*0.866) - round(Elein*0.5),-1200,1200,1000+Rightservotrim,2000+Rightservotrim);
  Backservo = map(Pitin + Elein+Backservotrim,-1200,1200,2000+Backservotrim,1000+Backservotrim);
  
  
  /*Serial.print("Correction");
  Serial.print(Wxbi);
  Serial.print("\t");
  Serial.print(Wybi);
  Serial.print("\t");
  Serial.print(Wzbi);
  Serial.print("\t");
  Serial.println(Rudservo);*/
  
  //Passing on signal to the servos
 
  if(bUpdateFlags & AIL_FLAG)
  {
    CRCArduinoFastServos::writeMicroseconds(SERVO_AIL,Leftservo);
  }

  if(bUpdateFlags & ELE_FLAG)
  {
    CRCArduinoFastServos::writeMicroseconds(SERVO_ELE,Backservo);
  }

  if(bUpdateFlags & PIT_FLAG)
  {
   CRCArduinoFastServos::writeMicroseconds(SERVO_PIT,Rightservo);
   }
   
    if(bUpdateFlags & RUD_FLAG)
  {
   CRCArduinoFastServos::writeMicroseconds(SERVO_RUD,Rudservo);
   }

  bUpdateFlags = 0;
  //delay(4);
  dt = (currentMillis-lastMillis);
  //Serial.print("\t");
  //Serial.println(dt);
  lastMillis = currentMillis;
}

// called by interrupt service routine when incoming data arrives
void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof Wx) + (sizeof Wy) + (sizeof Wz))
   {
   I2C_readAnything (Wx);   
   I2C_readAnything (Wy);   
   I2C_readAnything (Wz);
   haveData = true;     
   }  // end if have enough data
 }  // end of receiveEvent


// simple interrupt service routine
void calcAileron()
{
  if(PCintPort::pinState)
  {
    unAileronInStart = TCNT1;
  }
  else
  {
    unAileronInShared = (TCNT1 - unAileronInStart)>>1;
    bUpdateFlagsShared |= AIL_FLAG;
  }
}

void calcElevator()
{
  if(PCintPort::pinState)
  {
    unElevatorInStart = TCNT1;
  }
  else
  {
    unElevatorInShared = (TCNT1 - unElevatorInStart)>>1;

    bUpdateFlagsShared |= ELE_FLAG;
  }
}

void calcPitch()
{
  if(PCintPort::pinState)
  {
    unPitchInStart = TCNT1;
  }
  else
  {
    unPitchInShared = (TCNT1 - unPitchInStart)>>1;
    bUpdateFlagsShared |= PIT_FLAG;  }
   
}
void calcRudder()
{
  if(PCintPort::pinState)
  {
    unRudderInStart = TCNT1;
  }
  else
  {
    unRudderInShared = (TCNT1 - unRudderInStart)>>1;
    bUpdateFlagsShared |= RUD_FLAG;  }
}

