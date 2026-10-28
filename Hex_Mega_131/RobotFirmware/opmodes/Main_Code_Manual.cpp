#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM}};
static HexaServo arm{{.signalPin = SERVO_2, .startAngle = 30.0f, .settleMs = 0}};
static HexaServo claw{{.signalPin = SERVO_1, .startAngle = 90.0f, .settleMs = 0}};

class Main_Code_Manual : public NRLOpMode {
    int angleclaw_ = static_cast<int>(claw.getPosition());
    int anglebrdg_ = static_cast<int>(arm.getPosition());
    bool maxangleclaw_ = false;
    bool maxanglearm_ = false;

public:
    void init() override {
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
    }

    void loop() override {
        // Runs at 100 Hz until STOP — read gamepad1, drive motors here.
        
        if (gamepad1.pressed(BTN_DPAD_UP)) {
            anglebrdg_++;
            arm.setPosition(anglebrdg_);
        }

        if (gamepad1.pressed(BTN_DPAD_DOWN)) {
            --anglebrdg_;
            arm.setPosition(anglebrdg_);
        }

        if (gamepad1.pressed(BTN_DPAD_LEFT)) {
            --angleclaw_;
            arm.setPosition(angleclaw_);
        }

        if (gamepad1.pressed(BTN_DPAD_RIGHT)) {
            angleclaw_++;
            arm.setPosition(angleclaw_);
        }

        if (angleclaw_ == 145) {
            maxangleclaw_ = true;
        }

        if (anglebrdg_ == 89) {
            maxanglearm_ = true;
        }

        // Control each part in ONE place: loop() or an action, not
        // both. Actions run before loop() each tick, so loop() would
        // overwrite them. To share one part, step aside while a
        // sequence plays:  if (!isActionRunning()) { ... }
        if (maxangleclaw_ == true) {
            telemetry.addData("Claw Arm", " Maximum Angle Reached!");
        } else {
            telemetry.addData("Claw Arm: ", angleclaw_);
        }
        
        if (maxangleclaw_ == true) {
            telemetry.addData("Bridge Arm: ", " Maximum Angle Reached!");
            return;
        } else {
            telemetry.addData("Bridge Arm: ", anglebrdg_);
            return;
        }
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);
