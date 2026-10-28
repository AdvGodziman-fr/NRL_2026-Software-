#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor({.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM});
static HexaDCMotor rightMotor({.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped=true});

class Prac2_MotorsDC : public NRLOpMode {
    int _speed = 30;
    uint32_t startTimeMs_ = 0;
    bool timeLimitReached_ = false;
    
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
    }

    void start() override {
        startTimeMs_ = millis();
        timeLimitReached_ = false;
    }

    void loop() override {
        if (timeLimitReached_) return;

        if (gamepad1.pressed(BTN_DPAD_RIGHT)) { 
            _speed++;
        }
        if (gamepad1.pressed(BTN_DPAD_LEFT)) { 
            --_speed;
        }

        leftMotor.setSpeed(_speed);
        rightMotor.setSpeed(_speed);
        telemetry.addData("speed", (float)_speed);
    }

    void stop() override {
        leftMotor.stop();
        rightMotor.stop();
    }
};

REGISTER_OPMODE(Prac2_MotorsDC, "Prac2", TELEOP);
