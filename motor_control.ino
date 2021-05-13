#include "src/lib/src/System.h"
#include "src/lib/src/MotorLn298.h"
#include "src/lib/src/MotorVelocityControl.h"
#include "src/lib/src/SonarHcsr04.h"

#ifdef ARDUINO_ARCH_MEGAAVR
#include "EveryTimerB.h"
#define Timer1 TimerB2 // use TimerB2 as a drop in replacement for Timer1
#else // assume architecture supported by TimerOne ....
#include "TimerOne.h"
#endif

using namespace robopi;

/// Pin Layout
constexpr int in1 = 2,in2 = 3,enA = 9;
constexpr int in3 = 4,in4 = 5,enB = 10;
constexpr int encRight = 6,encLeft = 7;

/// Control Parameters
constexpr double kp = 0.02,ki = 0.05, kd = 0.00;
constexpr double f = 200;//[hz]
constexpr double dT = 1.0/f;//[s]
constexpr double kpObs = 2.5, kiObs = 7.5;
constexpr double setPoint = 10;//[rad/s]

constexpr double S_TO_MS = 1000.0;
constexpr double S_TO_US = 1000.0 * 1000.0;


class SystemArduino : public System
{
public:
    void sleep(int msec) override{ delay(msec);};

    void setMode(int pin,int sig) override{
        switch(sig)
        {
            case PI_OUTPUT:
                pinMode(pin,OUTPUT);
                break;
            case PI_INPUT:
                pinMode(pin,INPUT);
                break;
            default:
                Serial.println("Wrong pin setup.");
        }
    };

    void write(int pin,int sig) override{
        switch (sig)
        {
            case PI_ON:
                digitalWrite(pin,HIGH);
                break;
            case PI_OFF:
                digitalWrite(pin,LOW);
                break;
            default:
                Serial.println("Wrong pin write.");
        }
    };
    void pWM(int pin,int sig) override{
        
        analogWrite(pin,sig);
    };
    int read(int pin) override{
        switch (digitalRead(pin))
        {
            case HIGH:
                return PI_ON;
            case LOW:
                return PI_OFF;
            default:
                return PI_OFF;
        }
    };

    void setPWMfrequency(int pin,int freq) override{};
    void setPullUpDown(int pin,int upDown) override{
        switch (upDown)
        {
            case PI_PUD_UP:
                pinMode(pin,INPUT_PULLUP);
                break;
            default:
                Serial.println("Wrong pin setup.");
        }
    };
};

System* gpios;
Encoder* encoderLeft;
Encoder* encoderRight;
MotorLn298* motorLeft;
MotorLn298* motorRight;
VelocityEstimator* filterLeft;
VelocityEstimator* filterRight;
MotorVelocityControl* controlLeft;
MotorVelocityControl* controlRight;
unsigned long tick = 0U;
constexpr unsigned int deltaTick = static_cast<unsigned long>(0.1*S_TO_MS);
void isr_encRight()
{
    encoderRight->tick();
}
void isr_encLeft()
{
    encoderLeft->tick();
}

void isr_update(){
    controlRight->update(dT);
    controlLeft->update(dT);
}
void setup()
{
    Serial.begin(9600);

    gpios = new SystemArduino();
    motorRight = new MotorLn298(in1,in2,enA,gpios);
    encoderRight = new Encoder(encRight,gpios);

    //auto filterRight = std::make_shared<SlidingAverageFilter>(filterSize);
    filterRight = new LuenbergerObserver(kpObs,kiObs);

    controlRight = new MotorVelocityControl(motorRight,encoderRight,filterRight,kp,ki,kd);

    motorLeft = new MotorLn298(in4,in3,enB,gpios);
    encoderLeft = new Encoder(encLeft,gpios);

    attachInterrupt(digitalPinToInterrupt(encRight), isr_encRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLeft), isr_encLeft, CHANGE);

    //auto filterLeft = std::make_shared<SlidingAverageFilter>(filterSize);
    filterLeft = new LuenbergerObserver(kpObs,kiObs);

    controlLeft = new MotorVelocityControl(motorLeft,encoderLeft,filterLeft,kp,ki,kd);

    Timer1.initialize();
    Timer1.attachInterrupt(isr_update);
    Timer1.setPeriod(dT*S_TO_US); 
      

}



void plotStatus(const MotorVelocityControl* controller)
{
      //Serial.print(controller->wheelTicks());
      //Serial.print(" ");
      Serial.print(100);
      Serial.print(" ");
      Serial.print((controller->velocity()/controller->velocitySet())*100);
      Serial.print(" ");
      //Serial.print(controller->error());
      //Serial.print(" ");
      Serial.print(controller->dutySet());
}

void loop()
{
    controlRight->set(setPoint);
    controlLeft->set(setPoint);

   
     
      plotStatus(controlLeft);
            Serial.print(" ");

      plotStatus(controlRight); 
            Serial.print("\r\n");
   

    
    delay(deltaTick);
    tick += deltaTick;

}
