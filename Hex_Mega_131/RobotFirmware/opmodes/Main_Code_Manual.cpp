#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM}};
static HexaServo arm{{.signalPin = SERVO_1, .startAngle = 30.0f, .settleMs = 0}};
static HexaServo claw{{.signalPin = SERVO_4, .startAngle = 90.0f, .settleMs = 0}};
static TankDrive drive(leftMotor, rightMotor);

class Main_Code_Manual : public NRLOpMode {
    int angleclaw_ = 90;
    int anglebrdg_ = 30;
    bool maxangleclaw_ = false;
    bool maxanglearm_ = false;
    bool maxspeed_ = false;
    bool minspeed_ = false;

public:
    void init() override {
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
    }

    void loop() override {
        // These are for Servo-Motor operations 
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
            claw.setPosition(angleclaw_);
        }

        if (gamepad1.pressed(BTN_DPAD_RIGHT)) {
            angleclaw_++;
            claw.setPosition(angleclaw_);
        }

        if (angleclaw_ == 165) {
            maxangleclaw_ = true;
        }

        if (anglebrdg_ == 89) {
            maxanglearm_ = true;
        }

        // These are DC Motor (Wheel) Operations
        drive.drive(gamepad1.leftY(), gamepad1.rightX());

        // Servo Motor Results
        float v = power.getServoCurrent();
        telemetry.addData("Servo Current: ", v);

        if (maxangleclaw_ == true) {
            telemetry.addData("Claw Arm: ", " Maximum Angle Reached!");
            claw.setPosition(165);
        } else {
            telemetry.addData("Claw Arm: ", angleclaw_);
        }
        
        if (maxangleclaw_ == true) {
            telemetry.addData("Bridge Arm: ", " Maximum Angle Reached!");
            arm.setPosition(89);
        } else {
            telemetry.addData("Bridge Arm: ", anglebrdg_);
        }
}

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);
