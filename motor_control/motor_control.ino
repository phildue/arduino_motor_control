#include "System.h"
#include "MotorLn298.h"
#include "MotorVelocityControl.h"
#include "SonarHcsr04.h"
#include "SystemArduino.h"
#ifdef ARDUINO_ARCH_MEGAAVR
#include "EveryTimerB.h"
#define Timer1 TimerB2  // use TimerB2 as a drop in replacement for Timer1
#else                   // assume architecture supported by TimerOne ....
#include "TimerOne.h"
#endif

using namespace robopi;

/// Pin Layout
constexpr int GPIO_IN_1 = 9, GPIO_IN_2 = 8, GPIO_EN_A = 11;
constexpr int GPIO_IN_3 = 7, GPIO_IN_4 = 6, GPIO_EN_B = 5;
constexpr int GPIO_ENC_RIGHT = 3, GPIO_ENC_LEFT = 2;
constexpr int GPIO_ENC_RIGHT_B = 12, GPIO_ENC_LEFT_B = 10;

/// Control Parameters Init
constexpr double kpR = 0.02, kiR = 0.05, kdR = 0.00;
constexpr double kpL = 0.02, kiL = 0.05, kdL = 0.00;
constexpr double kpObs = 3.0, kiObs = 7.5;
constexpr double V_MAX = 15.0;
constexpr double ERR_I_MAX = 10;
constexpr int filterSize = 1;

/// Timings
constexpr double S_TO_MS = 1000.0;
constexpr double S_TO_US = 1000.0 * 1000.0;
constexpr double FRQ_CONTROL = 100;              //[hz]
constexpr double T_CONTROL = 1.0 / FRQ_CONTROL;  //[s]
constexpr double FRQ_MAIN = 50;                  //[hz]
constexpr double T_MAIN = 1.0 / FRQ_MAIN;        //[s]
constexpr double T_PID_STATE = 2.0;              //[s]
constexpr double T_LED_TOGGLE = 0.125;           //[s]

constexpr unsigned int T_MAIN_MS = static_cast<unsigned long>(T_MAIN * S_TO_MS);            //[ms]
constexpr unsigned int T_PID_STATE_MS = static_cast<unsigned long>(T_PID_STATE * S_TO_MS);  //[MS]

constexpr unsigned int T_CONTROL_MS = static_cast<unsigned long>(T_CONTROL * S_TO_MS);  //[ms]
/// Dimensions
constexpr double WHEEL_TICKS_PER_TURN = 480.0;

constexpr char* JOINT_NAMES[] = { "l", "r" };
constexpr char* FRAME_ID = "";

/// State
constexpr int STOP = 0;
constexpr int VELOCITY_CONTROL = 1;
constexpr int FEED_FORWARD = 2;


/// Modules
System* system_api;
Encoder* encoderLeft;
Encoder* encoderRight;
MotorLn298* motorLeft;
MotorLn298* motorRight;
//LuenbergerObserver* filterLeft;
//LuenbergerObserver* filterRight;
SlidingAverageFilter* filterLeft;
SlidingAverageFilter* filterRight;

MotorVelocityControl* controlLeft;
MotorVelocityControl* controlRight;


/// Variables
volatile int state = STOP;
unsigned long COMMAND_TIMEOUT_MS = 1U * S_TO_MS;  //[ms]
unsigned int T_LED_TOGGLE_MS = static_cast<unsigned long>(T_LED_TOGGLE * S_TO_MS);
unsigned long t_lastMsg = 0U;
volatile float vel[] = { 0, 0 };
volatile float pos[] = { 0, 0 };
volatile float cmd_vel[] = { 0, 0 };
volatile float cmd_duty[] = { 0, 0 };

unsigned long t_main_next;

void tickRight() {
  if (digitalRead(GPIO_ENC_RIGHT_B) == LOW) {
    encoderRight->tickForward();
  } else {
    encoderRight->tickBackward();
  }
}
void tickLeft() {
  if (digitalRead(GPIO_ENC_LEFT_B) == HIGH) {
    encoderLeft->tickForward();
  } else {
    encoderLeft->tickBackward();
  }
}

void controlLoop() {
  if (state == VELOCITY_CONTROL) {
    controlLeft->set(cmd_vel[0]);
    controlRight->set(cmd_vel[1]);
    controlRight->update(T_CONTROL);
    controlLeft->update(T_CONTROL);
  } else if (state == FEED_FORWARD) {
    motorLeft->set(cmd_duty[0]);
    motorRight->set(cmd_duty[1]);

  } else {
    controlLeft->stop();
    controlRight->stop();
  }
}

void readSerial();
void sendState();
void sendConfig();
void printHelp();

void setup() {

  /// System
  system_api = new SystemArduino();
  motorRight = new MotorLn298(GPIO_IN_1, GPIO_IN_2, GPIO_EN_A, system_api);
  encoderRight = new Encoder(WHEEL_TICKS_PER_TURN);

  filterRight = new SlidingAverageFilter(filterSize);
  //filterRight = new LuenbergerObserver(kpObs, kiObs);

  controlRight = new MotorVelocityControl(motorRight, encoderRight, filterRight, kpR, kiR, kdR, ERR_I_MAX, V_MAX);

  motorLeft = new MotorLn298(GPIO_IN_3, GPIO_IN_4, GPIO_EN_B, system_api);
  encoderLeft = new Encoder(WHEEL_TICKS_PER_TURN);

  filterLeft = new SlidingAverageFilter(filterSize);
  //filterLeft = new LuenbergerObserver(kpObs, kiObs);

  controlLeft = new MotorVelocityControl(motorLeft, encoderLeft, filterLeft, kpL, kiL, kdL, ERR_I_MAX, V_MAX);

  /// Interrupts
  attachInterrupt(digitalPinToInterrupt(GPIO_ENC_RIGHT), tickRight, RISING);
  attachInterrupt(digitalPinToInterrupt(GPIO_ENC_LEFT), tickLeft, RISING);


  t_main_next = millis();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_ENC_LEFT_B, INPUT);
  pinMode(GPIO_ENC_RIGHT_B, INPUT);


  Serial.begin(9600);
}

void updateState();

void sendJointState();

void sendPidState();

void toggleLed();

void loop() {
  if (millis() > t_main_next) {
    readSerial();
    toggleLed();
    updateState();
    //sendState();

    t_main_next += T_MAIN_MS;
  }
  controlLoop();
  delay(T_CONTROL_MS);
}

void stop() {
  state = STOP;
  cmd_vel[0] = 0;
  cmd_vel[1] = 0;
  cmd_duty[0] = 0;
  cmd_duty[1] = 0;
}
void updateState() {

  if (cmd_duty[0] != 0 || cmd_duty[1] != 0) {
    state = FEED_FORWARD;
    cmd_vel[0] = 0;
    cmd_vel[1] = 0;

    T_LED_TOGGLE_MS = 100;
  }

  if (cmd_vel[0] != 0 || cmd_vel[1] != 0) {
    state = VELOCITY_CONTROL;
    T_LED_TOGGLE_MS = 100;
    cmd_duty[0] = 0;
    cmd_duty[1] = 0;
  }

  if (millis() - t_lastMsg > COMMAND_TIMEOUT_MS) {
    t_lastMsg = millis();
    T_LED_TOGGLE_MS = 500;
    state = STOP;
    stop();
  }
}


void toggleLed() {
  static unsigned long t_lastToggle = millis();
  if (millis() - t_lastToggle > T_LED_TOGGLE_MS) {
    t_lastToggle = millis();

    static int ledState = LOW;
    digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
  }
}

void printHelp() {
  String t = String(millis());
  Serial.println("info " + t + " Welcome to Arduino Motor Control");
  Serial.println("info " + t + " kpR = " + String(controlRight->P()) + String(" kiR = ") + String(controlRight->I()) + String(" kdR = ") + String(controlRight->D()));
  Serial.println("info " + t + " kpL = " + String(controlLeft->P()) + String(" kiL = ") + String(controlLeft->I()) + String(" kdL = ") + String(controlLeft->D()));
  //Serial.println("info " + t + " kpO = " + String(filterLeft->P()) + String(" kiO = ") + String(filterLeft->I()));
  Serial.println("info " + t + " Usage:");
  Serial.println("info " + t + " Configure: \"set cfg <param> <value>\" e.g with \"set cfg kp X\" or \"set cfg kpl X\"");
  Serial.println("info " + t + " Query: \"query <param> \" e.g with \"query p\" or \"query vap\"");
  Serial.println("info " + t + " Set point: \"set <param> <timestamp> <value left> <value right>\" e.g with \"set vel 0 10 10\" or \set dty 0 0.5 -0.5 \"");
}

const byte numChars = 64;
char receivedChars[numChars];  // an array to store the received data
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
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

int split(String s, String delimiter, String* fields) {
  String remainer = s;
  int n = 0;
  int idx = remainer.indexOf(delimiter);
  while (idx < remainer.length()) {
    fields[n++] = remainer.substring(0, idx);
    remainer = remainer.substring(idx + 1);
    idx = remainer.indexOf(delimiter);
  }
  fields[n++] = remainer;
  return n;
}

void readSerial() {
  recvWithEndMarker();

  if (newData) {
    String msg = String(receivedChars);
    newData = false;
    t_lastMsg = millis();
    String fields[10];
    int nFields = split(msg, " ", fields);

    if (fields[0].startsWith("set")) {
      if (nFields == 5) {
        if (fields[1].startsWith("vel")) {
          cmd_vel[0] = fields[3].toFloat();
          cmd_vel[1] = fields[4].toFloat();
        } else if (fields[1].startsWith("dty")) {
          cmd_duty[0] = fields[3].toFloat();
          cmd_duty[1] = fields[4].toFloat();
        } else if (fields[1].startsWith("rst")) {
          stop();
          encoderLeft->reset();
          encoderRight->reset();
        } else {
          Serial.println("info " + String(millis()) + " Available: vel, dty, rst");
        }
      } else if (nFields == 4) {
        if (fields[1].startsWith("cfg")) {
          if (fields[2].startsWith("kpr")) {
            controlRight->P() = fields[3].toFloat();
          } else if (fields[2].startsWith("kir")) {
            controlRight->I() = fields[3].toFloat();
          } else if (fields[2].startsWith("kdr")) {
            controlRight->D() = fields[3].toFloat();
          } else if (fields[2].startsWith("kpl")) {
            controlLeft->P() = fields[3].toFloat();
          } else if (fields[2].startsWith("kil")) {
            controlLeft->I() = fields[3].toFloat();
          } else if (fields[2].startsWith("kdl")) {
            controlLeft->D() = fields[3].toFloat();
          } else if (fields[2].startsWith("kp")) {
            controlLeft->P() = fields[3].toFloat();
            controlRight->P() = controlLeft->P();
          } else if (fields[2].startsWith("ki")) {
            controlLeft->I() = fields[3].toFloat();
            controlRight->I() = controlLeft->I();
          } else if (fields[2].startsWith("kd")) {
            controlLeft->D() = fields[3].toFloat();
            controlRight->D() = controlLeft->D();
         /* } else if (fields[2].startsWith("kpo")) {
            filterLeft->P() = fields[3].toFloat();
            filterRight->P() = filterLeft->P();
          } else if (fields[2].startsWith("kio")) {
            filterLeft->I() = fields[3].toFloat();
            filterRight->I() = filterLeft->I();*/
          } else if (fields[2].startsWith("t")) {
            COMMAND_TIMEOUT_MS = fields[3].toInt() * S_TO_MS;
          } else {
            Serial.println("info " + String(millis()) + " Available: kp=control p, ki=control i, kd=control d, kpo=filter p, kio=filter i, t=command timeout, kpr, kir, kdr, kpl, kil, kdl");
          }
        } else {
          Serial.println("info " + String(millis()) + " set message not complete");
        }
      }
    } else if (fields[0].startsWith("query")) {
      if (fields[1].startsWith("vap")) {
        sendState();
      } else if (fields[1].startsWith("pos")) {
        sendPosition();
      } else if (fields[1].startsWith("cfg")) {
        sendConfig();
      } else {
        Serial.println("info " + String(millis()) + " Available: s=state, p=position, c=config");
      }
    } else {
      Serial.println("inf 0 Unknown" + msg);
      stop();
      printHelp();
    }
  }
}

void sendState(const MotorVelocityControl* controller) {

  Serial.print(String(controller->position()) + " " + String(controller->velocity()) + " ");
}


void sendState() {
  Serial.print("state vap " + String(millis()) + " ");
  sendState(controlLeft);
  sendState(controlRight);
  Serial.print("\r\n");
}

void sendConfig(const MotorVelocityControl* controller) {

  Serial.print(String(controller->P()) + " " + String(controller->I()) + " " + String(controller->D()) + " ");
}
void sendConfig() {
  Serial.print("state cfg " + String(millis()) + " ");
  sendConfig(controlLeft);
  sendConfig(controlRight);
  Serial.print("\r\n");
}

void sendPosition(const MotorVelocityControl* controller, const Encoder* encoder) {
  Serial.print(String(controller->position()) + " " + String(encoder->ticks()) + " ");
}
void sendPosition() {
  Serial.print("state pos " + String(millis()) + " ");
  sendPosition(controlLeft, encoderLeft);
  sendPosition(controlRight, encoderRight);
  Serial.print("\r\n");
}
