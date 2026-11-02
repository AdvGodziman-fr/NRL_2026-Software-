#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM}};
static HexaServo arm{{.signalPin = SERVO_1, .maxAngle = 90, .startAngle = 30.0f, .settleMs = 0 }};
static HexaServo claw{{.signalPin = SERVO_4, .maxAngle = 165, .startAngle = 90.0f, .settleMs = 0 }};
static TankDrive drive(leftMotor, rightMotor);
static HexaIMU imu { hexaImuConfig()};

class Main_Code_Manual : public NRLOpMode {
    int angleclaw_ = 90;
    int anglearm_ = 30;
    bool maxspeed_ = false;
    bool minspeed_ = false;

public:
    void init() override {
        imu.begin();
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
    }

    void loop() override {

        imu.tick(millis());
        telemetry.addData("IMU Reading: ", imu.getHeading());

        // These are DC Motor (Wheel) Operations
        drive.setScale(gamepad1.pressed(BTN_LB) ? 0.15f : 1.00f);
        drive.drive(gamepad1.leftY(), gamepad1.rightX());

        // Servo Motor Results
        float v = power.getServoCurrent();
        telemetry.addData("Servo Current: ", v);

        // These are for Servo-Motor operations
        if (gamepad1.pressed(BTN_DPAD_LEFT)) {
            --angleclaw_;
            claw.setPosition(angleclaw_);
        }
        if (gamepad1.pressed(BTN_DPAD_RIGHT) && angleclaw_ < claw.getMaxAngle()) {
            ++angleclaw_;
            claw.setPosition(angleclaw_);
        }

        if (angleclaw_ >= claw.getMaxAngle()) {
            telemetry.addData("Claw Arm: ", " Maximum Angle Reached!");
        } else {
            telemetry.addData("Claw Arm: ", angleclaw_);
        }

        if (gamepad1.pressed(BTN_DPAD_UP) && anglearm_ < arm.getMaxAngle()) {
            ++anglearm_;
            arm.setPosition(anglearm_);
        }
        if (gamepad1.pressed(BTN_DPAD_DOWN)) {
            --anglearm_;
            arm.setPosition(anglearm_);
        }

        if (anglearm_ >= arm.getMaxAngle()) {
            telemetry.addData("Bridge Arm: ", " Maximum Angle Reached!");
        } else {
            telemetry.addData("Bridge Arm: ", anglearm_);
        }
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);
