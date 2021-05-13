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

/// Control Parameters
constexpr double kp = 0.02, ki = 0.05, kd = 0.00;
volatile double kpR = kp, kiR = ki, kdR = kd;
volatile double kpL = kp, kiL = ki, kdL = kd;

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
volatile VelocityEstimator* filterLeft;
volatile VelocityEstimator* filterRight;
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

  controlRight = new MotorVelocityControl(motorRight, encoderRight, filterRight, kp, ki, kd);

  motorLeft = new MotorLn298(in4, in3, enB, gpios);
  encoderLeft = new Encoder(encLeft, gpios);

  attachInterrupt(digitalPinToInterrupt(encRight), isr_encRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLeft), isr_encLeft, CHANGE);

  //auto filterLeft = std::make_shared<SlidingAverageFilter>(filterSize);
  filterLeft = new LuenbergerObserver(kpObs, kiObs);

  controlLeft = new MotorVelocityControl(motorLeft, encoderLeft, filterLeft, kp, ki, kd);

  Timer1.initialize();
  Timer1.attachInterrupt(isr_update);
  Timer1.setPeriod(dT * S_TO_US);


}

void readSerial();
void writeSerial();

void updateState()
{
   if (setPointRight > V_MAX)
   {
    setPointRight = V_MAX;
   }
   if (setPointRight < -1.0 * V_MAX)
   {
    setPointRight = -1.0 * V_MAX;
   }
   if(setPointLeft > V_MAX)
   {
    setPointLeft = V_MAX;
   }
   if (setPointLeft < -1.0 * V_MAX)
   {
    setPointLeft = -1.0 * V_MAX;
   }
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
    motorLeft->stop();
    motorRight->stop();
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

void handleSerial()
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
      kpR = msg.substring(4).toFloat();
      Serial.println("Setting kpR:");
      Serial.println(kpR);

    }
    else if (msg.startsWith("kir"))
    {
      kiR = msg.substring(4).toFloat();
      Serial.println("Setting kiR:");
      Serial.println(kiR);

    }
    else if (msg.startsWith("kdr"))
    {
      kdR = msg.substring(4).toFloat();
      Serial.println("Setting kdR:");
      Serial.println(kdR);
    }
    else if (msg.startsWith("kpl"))
    {
      kpL = msg.substring(4).toFloat();
      Serial.println("Setting kpL:");
      Serial.println(kpL);

    }
    else if (msg.startsWith("kil"))
    {
      kiL = msg.substring(4).toFloat();
      Serial.println("Setting kiL:");
      Serial.println(kiL);

    }
    else if (msg.startsWith("kdl"))
    {
      kdL = msg.substring(4).toFloat();
      Serial.println("Setting kdL:");
      Serial.println(kdL);

    }
    else if (msg.startsWith("kp"))
    {
      kpR = msg.substring(4).toFloat();
      kpL = kpR;
      Serial.println("Setting kp:");
      Serial.println(kpR);

    }
    else if (msg.startsWith("ki"))
    {
      kiR = msg.substring(4).toFloat();
      kiL = kiR;
      Serial.println("Setting ki:");
      Serial.println(kiR);

    }
    else if (msg.startsWith("kd"))
    {
      kdR = msg.substring(4).toFloat();
      kdL = kdR;
      Serial.println("Setting kd:");
      Serial.println(kdR);

    } else if (msg.startsWith("p"))
    {
      Serial.println(String(" kpR = " ) + String(kpR) + String(" kiR = ") + String(kiR) + String(" kdR = ") + String(kdR));
      Serial.println(String(" kpL = " ) + String(kpL) + String(" kiL = ") + String(kiL) + String(" kdL = ") + String(kdL));
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

void plotStatus()
{

  plotStatus(controlLeft);
  Serial.print(" ");

  plotStatus(controlRight);
  Serial.print("\r\n");


}
