#include "NRL.h"

static HexaServo arm{{.signalPin = SERVO_1, .startAngle = 90.0f, .settleMs = 0}};

class Test_Prac1 : public NRLOpMode {
    int angle_ = 0;
    uint32_t lastAngleChangeMs_ = 0;
    bool sweepComplete_ = false;

public:
    void init() override {
        arm.begin();
        angle_ = static_cast<int>(arm.getPosition());
        lastAngleChangeMs_ = millis();
        sweepComplete_ = false;
    }

    void loop() override {
        if (sweepComplete_) return;

        telemetry.addData("arm", angle_);

        if (angle_ == 180) {
            sweepComplete_ = true;
        } else if (millis() - lastAngleChangeMs_ >= 500) {
            lastAngleChangeMs_ = millis();
            angle_++;
            arm.setPosition(angle_);
        }
    }

    void stop() override {
        arm.detach(); // let the servo relax
    }
};

REGISTER_OPMODE(Test_Prac1, "Tes1", TELEOP);