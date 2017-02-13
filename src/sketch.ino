#include <Servo.h>

namespace Motor {
    //Motor A
    const int PWMPin = 3; //Speed control 
    const int ForwardPin = 8; //Direction
    const int BackwardPin = 9; //Direction
    const int STBYPin = 10;
    const int ServoPin = 11;

    Servo servo;  

    bool current_reverse;

    void init() {
        pinMode(PWMPin, OUTPUT);
        pinMode(ForwardPin, OUTPUT);
        pinMode(BackwardPin, OUTPUT);
        pinMode(STBYPin, OUTPUT);

        pinMode(ServoPin, OUTPUT);
        servo.attach(ServoPin);

        current_reverse = 0;
    }

    void stop() {
        digitalWrite(STBYPin, LOW); 
    }

    void move(uint8_t heading, bool reverse, uint8_t power){
#define REGIST_ANGLE(c, pwm) \
    case c: \
            servo.write(pwm); \
    break

        // Change heading
        switch (heading)
        {
            REGIST_ANGLE(0, 75);
            REGIST_ANGLE(1, 80);
            REGIST_ANGLE(2, 85);
            REGIST_ANGLE(3, 90);
            REGIST_ANGLE(4, 95);
            REGIST_ANGLE(5, 100);
            REGIST_ANGLE(6, 105);

            default:
            servo.write(90);
            break;
        }

        // Change reverse
        if (reverse != current_reverse)
        {
            stop(); //stop
            delay(200); //hold for 250ms until move again
            current_reverse = reverse;

            digitalWrite(ForwardPin, !reverse);
            digitalWrite(BackwardPin, reverse);
        }

#define REGIST_POWER(p, pwm) \
    case p: \
        analogWrite(PWMPin, pwm); \
    break

        switch (power)
        {
            REGIST_POWER(0,   0);
            REGIST_POWER(1,  36);
            REGIST_POWER(2,  73);
            REGIST_POWER(3, 109);
            REGIST_POWER(4, 146);
            REGIST_POWER(5, 182);
            REGIST_POWER(6, 255);

            default:
                move(1, 255, reverse);
                break;
        }
    }
}

namespace Laser {
    const int Pin = 7;
    void init() {
       pinMode(Pin, OUTPUT); 
    }

    void fire(bool b) {
        if (b) tone(Pin, 40, 300);
    }
};

namespace Laser {
    const int PIN = 7;
    int state;

    void init() {
        pinMode(PIN, OUTPUT);
        digitalWrite(PIN, LOW);
        state = LOW;
    }

    void on() {
        if (!state) {
            digitalWrite(PIN, HIGH);
            state = HIGH;
        }
    }

    void off() {
        if (state) {
            digitalWrite(PIN, LOW);
            state = LOW;
        }
    }

    void toggle() {
        state = !state;
        digitalWrite(PIN, state);
    }
}

namespace LaserDetector {
    const int PIN = A0;

    void init() {
    }

    int value() {
        return analogRead(PIN);
    }
}

namespace Judger {
    const int BUFFER_SIZE = 20;
    const int THRESHOLD = 50;
    long values[BUFFER_SIZE];
    int ptr;

    void push(int val) {
        values[ptr++] = val;
        if (ptr == BUFFER_SIZE) ptr = 0;
    }

    long calc_std() {
        long s1, s2;
        s1 = s2 = 0;
        for (int i=0; i<BUFFER_SIZE; i++) {
            s1 += values[i];
            s2 += values[i] * values[i];
        }
        s1 = (s1 * s1) / BUFFER_SIZE;
        return (s2 - s1) / BUFFER_SIZE;
    }

    bool judge() {
        auto res = calc_std();
        return res > THRESHOLD;
    }
}

int servoAngle = 0;   // servo position in degrees

void setup()
{
    Serial.begin(115200);  
    Motor::init();
    Laser::init();
}

void loop()
{
    if (Serial.available()) {

        char c = Serial.read();

        uint8_t heading = (c & 0x70) >> 4;
        bool reverse = (c & 0x08) >> 3;
        uint8_t power = c & 0x07; 
        bool fire = !!(c & 0x80);

        Motor::move(heading, reverse, power);
        Laser::fire(reverse);
    } else {
        delay(50);
    }
}
