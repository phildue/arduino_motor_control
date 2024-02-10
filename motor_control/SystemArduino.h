#include "System.h"
class SystemArduino : public robopi::System
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
                pinMode(pin,INPUT_PULLUP);
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
