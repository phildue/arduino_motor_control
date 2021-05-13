#include <System.h>
#include <MotorLn298.h>
#include <MotorVelocityControl.h>
#include <SonarHcsr04.h>
#include "SystemArduino.h"
#ifdef ARDUINO_ARCH_MEGAAVR
#include "EveryTimerB.h"
#define Timer1 TimerB2 // use TimerB2 as a drop in replacement for Timer1
#else // assume architecture supported by TimerOne ....
#include "TimerOne.h"
#endif

using namespace robopi;

/// Pin Layout
constexpr int in1 = 2, in2 = 3, enA = 9;
constexpr int in3 = 4, in4 = 5, enB = 10;
constexpr int encRight = 6, encLeft = 7;

/// Control Parameters Init
constexpr double kpR = 0.02, kiR = 0.05, kdR = 0.00;
constexpr double kpL = 0.02, kiL = 0.05, kdL = 0.00;

constexpr double f = 1000;//[hz]
constexpr double dT = 1.0 / f; //[s]
constexpr double kpObs = 3.0, kiObs = 7.5;
double setPointLeft = 0;//[rad/s]
double setPointRight = 0;//[rad/s]

constexpr double S_TO_MS = 1000.0;
constexpr double S_TO_US = 1000.0 * 1000.0;
constexpr double V_MAX = 15.0;

volatile System* gpios;
volatile Encoder* encoderLeft;
volatile Encoder* encoderRight;
volatile MotorLn298* motorLeft;
volatile MotorLn298* motorRight;
volatile LuenbergerObserver* filterLeft;
volatile LuenbergerObserver* filterRight;
volatile MotorVelocityControl* controlLeft;
volatile MotorVelocityControl* controlRight;
volatile bool running = true;

unsigned long tick = 0U;
unsigned long lastCommand = 0U;
unsigned long commandTimeout = 10U;
constexpr unsigned int deltaTick = static_cast<unsigned long>(0.1 * S_TO_MS);
void isr_encRight()
{
  encoderRight->tick();
}
void isr_encLeft()
{
  encoderLeft->tick();
}

void isr_update() {
  if (running)
  {
    controlRight->update(dT);
    controlLeft->update(dT);
  }
}




void setup()
{
  Serial.begin(9600);

  gpios = new SystemArduino();
  motorRight = new MotorLn298(in1, in2, enA, gpios);
  encoderRight = new Encoder(encRight, gpios);

  //auto filterRight = std::make_shared<SlidingAverageFilter>(filterSize);
  filterRight = new LuenbergerObserver(kpObs, kiObs);

  controlRight = new MotorVelocityControl(motorRight, encoderRight, filterRight, kpR, kiR, kdR);

  motorLeft = new MotorLn298(in4, in3, enB, gpios);
  encoderLeft = new Encoder(encLeft, gpios);

  attachInterrupt(digitalPinToInterrupt(encRight), isr_encRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLeft), isr_encLeft, CHANGE);

  //auto filterLeft = std::make_shared<SlidingAverageFilter>(filterSize);
  filterLeft = new LuenbergerObserver(kpObs, kiObs);

  controlLeft = new MotorVelocityControl(motorLeft, encoderLeft, filterLeft, kpL, kiL, kdL);

  Timer1.initialize();
  Timer1.attachInterrupt(isr_update);
  Timer1.setPeriod(dT * S_TO_US);

  printHelp();
}

void readSerial();
void writeSerial();

void updateState()
{
   
   controlRight->set(setPointRight);
   controlLeft->set(setPointLeft);
   if (setPointRight > 0 || setPointLeft > 0)
   {
    running = true;
   }
   if (tick - lastCommand > commandTimeout * S_TO_MS)
   {
    lastCommand = tick;
    Serial.println("Timeout!");
    setPointLeft = 0.0;
    setPointRight = 0.0;
    running = false;
    controlLeft->stop();
    controlRight->stop();
    }

}


void loop()
{

  //setPoint = sin((float)tick/10000.0)*20.0;

  readSerial();

  updateState();
 
  writeSerial();


  delay(deltaTick);
  tick += deltaTick;

}
void printHelp()
{
  Serial.println("Welcome to Arduino Motor Control");

  Serial.println(String("Control Right: kpR = " ) + String(controlRight->P()) + String(" kiR = ") + String(controlRight->I()) + String(" kdR = ") + String(controlRight->D()));
  Serial.println(String("Control Left: kpL = " ) + String(controlLeft->P()) + String(" kiL = ") + String(controlLeft->I()) + String(" kdL = ") + String(controlLeft->D()));
  Serial.println(String("Filter: kpO = " ) + String(filterLeft->P()) + String(" kiO = ") + String(filterLeft->I()));
  Serial.println("Usage:");
  Serial.println("Configure: \"<param> <value>\" e.g with \"kp X\" or \"kpl X\"");
  Serial.println("Set point: \"<param> <value>\" e.g with \"vl X\" or \"vr X\"");

  Serial.println("Output Stream: vl* vl vr* vr");
}
void readSerial()
{
  if (Serial.available() > 0)
  {
    String msg = Serial.readStringUntil("\n");
    Serial.println("I recieved:");
    Serial.println(msg);
    lastCommand = tick;
    if (msg.startsWith("vl"))
    {
      setPointLeft = msg.substring(3).toFloat();
     
      Serial.println("Set point left:");
      Serial.println(setPointLeft);

    } else if (msg.startsWith("vr"))
    {
      setPointRight = msg.substring(3).toFloat();
      Serial.println("Set point right:");
      Serial.println(setPointRight);

    }
    else if (msg.startsWith("kpr"))
    {
      controlRight->P() = msg.substring(4).toFloat();
      Serial.println("Setting kpR:");
      Serial.println(controlRight->P() );

    }
    else if (msg.startsWith("kir"))
    {
      controlRight->I() = msg.substring(4).toFloat();
      Serial.println("Setting kiR:");
      Serial.println(controlRight->I());

    }
    else if (msg.startsWith("kdr"))
    {
      controlRight->D() = msg.substring(4).toFloat();
      Serial.println("Setting kdR:");
      Serial.println(controlRight->D());
    }
    else if (msg.startsWith("kpl"))
    {
      controlLeft->P() = msg.substring(4).toFloat();
      Serial.println("Setting kpL:");
      Serial.println(controlLeft->P());

    }
    else if (msg.startsWith("kil"))
    {
      controlLeft->I() = msg.substring(4).toFloat();
      Serial.println("Setting kiL:");
      Serial.println(controlLeft->I());

    }
    else if (msg.startsWith("kdl"))
    {
      controlLeft->D() = msg.substring(4).toFloat();
      Serial.println("Setting kdL:");
      Serial.println(controlLeft->D());

    }
    else if (msg.startsWith("kp"))
    {
      controlLeft->P() = msg.substring(3).toFloat();
      controlRight->P() = controlLeft->P();
      Serial.println("Setting kp:");
      Serial.println(controlLeft->P());

    }
    else if (msg.startsWith("ki"))
    {
      controlLeft->I() = msg.substring(3).toFloat();
      controlRight->I() = controlLeft->I();
      Serial.println("Setting ki:");
      Serial.println(controlLeft->I());

    }
    else if (msg.startsWith("kd"))
    {
      controlLeft->D() = msg.substring(3).toFloat();
      controlRight->D() = controlLeft->D();
      Serial.println("Setting kd:");
      Serial.println(controlLeft->D());

    }else if (msg.startsWith("kpo"))
    {
      filterLeft->P() = msg.substring(3).toFloat();
      filterRight->P() = filterLeft->P();
      Serial.println("Setting kpObs:");
      Serial.println(filterLeft->P());

    } else if (msg.startsWith("kio"))
    {
      filterLeft->I() = msg.substring(3).toFloat();
      filterRight->I() = filterLeft->I();
      Serial.println("Setting kiObs:");
      Serial.println(filterLeft->I());

    }else if (msg.startsWith("p"))
    {
      printHelp();
   }
    else {
      Serial.println("Unknown Command");
      setPointLeft = 0;
      setPointRight = 0;
      running = false;
      motorLeft->stop();
      motorRight->stop();
    }
  }
}

void plotStatus(const MotorVelocityControl* controller)
{
  //Serial.print(controller->wheelTicks());
  //Serial.print(" ");
  Serial.print(controller->velocitySet());
  Serial.print(" ");
  Serial.print((controller->velocity()));
  //Serial.print(" ");
  //Serial.print(controller->error());
  //Serial.print(" ");
  //Serial.print(controller->dutySet());
}

void writeSerial()
{

  plotStatus(controlLeft);
  Serial.print(" ");

  plotStatus(controlRight);
  Serial.print("\r\n");


}
