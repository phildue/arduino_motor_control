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
constexpr int GPIO_IN_1 = 2, GPIO_IN_2 = 3, GPIO_EN_A = 9;
constexpr int GPIO_IN_3 = 4, GPIO_IN_4 = 5, GPIO_EN_B = 10;
constexpr int GPIO_ENC_RIGHT = 6, GPIO_ENC_LEFT = 7;
constexpr int GPIO_LED = LED_BUILTIN;

/// Control Parameters Init
constexpr double kpR = 0.02, kiR = 0.05, kdR = 0.00;
constexpr double kpL = 0.02, kiL = 0.05, kdL = 0.00;
constexpr double kpObs = 3.0, kiObs = 7.5;
constexpr double V_MAX = 15.0;
constexpr double ERR_I_MAX = 10;

/// Timings
constexpr double S_TO_MS = 1000.0;
constexpr double S_TO_US = 1000.0 * 1000.0;
constexpr double FRQ_CONTROL = 100;//[hz]
constexpr double T_CONTROL = 1.0 / FRQ_CONTROL; //[s]
constexpr double FRQ_MAIN = 50;//[hz]
constexpr double T_MAIN = 1.0 / FRQ_MAIN;//[s]
constexpr double T_PID_STATE = 2.0;//[s]
constexpr double T_LED_TOGGLE = 0.125;//[s]

constexpr unsigned int T_MAIN_MS = static_cast<unsigned long>(T_MAIN * S_TO_MS); //[ms]
constexpr unsigned int T_PID_STATE_MS = static_cast<unsigned long>(T_PID_STATE * S_TO_MS); //[MS]
constexpr unsigned long COMMAND_TIMEOUT_MS = 10U * S_TO_MS; //[ms]


constexpr char* JOINT_NAMES[] = {"l", "r"};
constexpr char* FRAME_ID = "";

/// Modules
System* system_api;
Encoder* encoderLeft;
Encoder* encoderRight;
MotorLn298* motorLeft;
MotorLn298* motorRight;
LuenbergerObserver* filterLeft;
LuenbergerObserver* filterRight;
MotorVelocityControl* controlLeft;
MotorVelocityControl* controlRight;


/// Variables
volatile bool running = false;
unsigned int T_LED_TOGGLE_MS = static_cast<unsigned long>(T_LED_TOGGLE * S_TO_MS);
unsigned long t_lastMsg = 0U;
volatile float vel[] = {0, 0};
volatile float pos[] = {0, 0};
volatile float cmd_vel[] = {0,0};

void isr_tickRight()
{
  encoderRight->tick();
}
void isr_tickLeft()
{
  encoderLeft->tick();
}

void isr_controlLoop() {
  if (running)
  {
    controlLeft->set(cmd_vel[0]);
    controlRight->set(cmd_vel[1]);
    controlRight->update(T_CONTROL);
    controlLeft->update(T_CONTROL);
  }else{
    controlLeft->stop();
    controlRight->stop();
  }
}

void cb_set(const std_msgs::Float32MultiArray& msg) {
  t_lastMsg = millis();
  if(msg.data_length == 2)
  {
    cmd_vel[0] = msg.data[0];
    cmd_vel[1] = msg.data[1];
   }
  
}

void readSerial();
void sendState();
void sendConfig();
void printHelp();

void setup()
{

  /// System
  system_api = new SystemArduino();
  motorRight = new MotorLn298(GPIO_IN_1, GPIO_IN_2, GPIO_EN_A, system_api);
  encoderRight = new Encoder(GPIO_ENC_RIGHT, system_api);

  //auto filterRight = std::make_shared<SlidingAverageFilter>(filterSize);
  filterRight = new LuenbergerObserver(kpObs, kiObs);

  controlRight = new MotorVelocityControl(motorRight, encoderRight, filterRight, kpR, kiR, kdR,ERR_I_MAX,V_MAX);

  motorLeft = new MotorLn298(GPIO_IN_4, GPIO_IN_3, GPIO_EN_B, system_api);
  encoderLeft = new Encoder(GPIO_ENC_LEFT, system_api);

  //auto filterLeft = std::make_shared<SlidingAverageFilter>(filterSize);
  filterLeft = new LuenbergerObserver(kpObs, kiObs);

  controlLeft = new MotorVelocityControl(motorLeft, encoderLeft, filterLeft, kpL, kiL, kdL,ERR_I_MAX,V_MAX);

  /// Interrupts
  attachInterrupt(digitalPinToInterrupt(GPIO_ENC_RIGHT), isr_tickRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(GPIO_ENC_LEFT), isr_tickLeft, CHANGE);


  Timer1.initialize();
  Timer1.attachInterrupt(isr_controlLoop);
  Timer1.setPeriod(T_CONTROL * S_TO_US);

  pinMode(GPIO_LED, OUTPUT);

  Serial.begin(9600);
  printHelp();
 
}

void updateState();

void sendJointState();

void sendPidState();

void toggleLed();

void loop()
{

  readSerial();

  toggleLed();
  updateState();
  sendState();

  delay(T_MAIN_MS );

}


void updateState()
{

  if (cmd_vel[0] != 0 || cmd_vel[1] != 0)
  {
    running = true;
    T_LED_TOGGLE_MS = 100;
  }
  if (millis() - t_lastMsg > COMMAND_TIMEOUT_MS)
  {
    t_lastMsg = millis();
    T_LED_TOGGLE_MS = 500;
    running = false;
    cmd_vel[0] = 0;
    cmd_vel[1] = 0;

  }

}


void toggleLed()
{
  static unsigned long t_lastToggle = millis();
  if (millis() - t_lastToggle > T_LED_TOGGLE_MS)
  {
    t_lastToggle = millis();

    static int ledState = LOW;
    digitalWrite(GPIO_LED, ledState);
    ledState = !ledState;
  }
}

void printHelp()
{
  Serial.println("I Welcome to Arduino Motor Control");

  Serial.println(String("Control Right: kpR = " ) + String(controlRight->P()) + String(" kiR = ") + String(controlRight->I()) + String(" kdR = ") + String(controlRight->D()));
  Serial.println(String("Control Left: kpL = " ) + String(controlLeft->P()) + String(" kiL = ") + String(controlLeft->I()) + String(" kdL = ") + String(controlLeft->D()));
  Serial.println(String("Filter: kpO = " ) + String(filterLeft->P()) + String(" kiO = ") + String(filterLeft->I()));
  Serial.println("Usage:");
  Serial.println("Configure: \"<param> <value>\" e.g with \"kp X\" or \"kpl X\"");
  Serial.println("Set point: \"<param> <value>\" e.g with \"vl X\" or \"vr X\"");

  Serial.println("Output Stream:ID pl vl* vl el dl pr vr* vr er dr");
}

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
bool newData = false;
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void readSerial()
{
  recvWithEndMarker();
  
  if (newData)
  {
    String msg = String(receivedChars);
    newData = false;
    t_lastMsg = millis();
    if (msg.startsWith("v"))
    {
      int idxSep = msg.substring(2).indexOf(" ")+2;
      cmd_vel[0] = msg.substring(2,idxSep).toFloat();
      cmd_vel[1] = msg.substring(idxSep+1).toFloat();
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
      Serial.println("I recieved:");
      Serial.println(msg);

      Serial.println("Unknown Command");
      cmd_vel[0] = 0;
      cmd_vel[1] = 0;
      running = false;
      
    }
  }
}

void sendState(const MotorVelocityControl* controller)
{
  long int ticks = controller->wheelTicks();
  Serial.print(String(ticks) + " " + String(controller->velocitySet()) + " " + String(controller->velocity()) + " " + String(controller->error()) + " " + String(controller->dutySet()) + " ");
}

void sendState()
{

  Serial.print("S ");
  sendState(controlLeft);
  sendState(controlRight);
  Serial.print("\r\n");


}
