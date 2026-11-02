#include "NRL.h"
#include <cmath>

// ---------------- HARDWARE DECLARATION ----------------
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }};
static HexaServo arm {{ .signalPin = SERVO_1, .minAngle = 15.0f, .maxAngle = 90.0f, .startAngle = 30.0f, .settleMs = 0 }};
static HexaServo claw {{ .signalPin = SERVO_4, .minAngle = 35.0f, .maxAngle = 175.0f, .startAngle = 90.0f, .settleMs = 0 }};
static TankDrive drive(leftMotor, rightMotor);

// ---------------- SERVO SETTINGS ----------------
constexpr float DEADZONE = 0.08f;
constexpr float SERVO_SPEED = 60.0f;
constexpr float LOOP_TIME = 0.01f;

// ---------------- TURN SETTINGS ----------------
constexpr float MIN_TURN_SCALE = 0.35f;
constexpr float MAX_TURN_SCALE = 1.00f;

//Main Class
class Main_Code_Manual : public NRLOpMode {
    float armAngle;
    float clawAngle;

    // ---------------- SERVO CONTROL ----------------
    void updateServo(HexaServo &servo, float &currentAngle, float joystickInput) {
        // Ignore small joystick movements.
        if (std::fabs(joystickInput) < DEADZONE) {
            return;
        }
        // Calculate the new servo angle.
        currentAngle += joystickInput * SERVO_SPEED * LOOP_TIME;

        // Restrict the angle to the servo's permitted range.
        currentAngle = std::fmax(
            servo.getMinAngle(),
            std::fmin(servo.getMaxAngle(), currentAngle)
        );

        // Update servo position.
        servo.setPosition(currentAngle);
    }

    // ---------------- SPEED-DEPENDENT DRIVE ----------------
    void driveWithSpeedDependentTurning() {
        float forward = gamepad1.leftY();
        float turn = gamepad1.leftX();

        // Reduce turning sensitivity as forward speed increases.
        float turnScale = MIN_TURN_SCALE + (MAX_TURN_SCALE - MIN_TURN_SCALE) * std::fabs(forward);
        turn *= turnScale;

        // Slow mode while LB is held.
        drive.setScale(gamepad1.pressed(BTN_LB) ? 0.15f : 1.00f);

        // Apply drivetrain movement.
        drive.drive(forward, turn);
    }

public:
    void init() override {
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
        armAngle = arm.getPosition();
        clawAngle = claw.getPosition();
    }

    void loop() override {
        // -------- DRIVE CONTROL --------
        driveWithSpeedDependentTurning();

        // -------- SERVO CONTROL --------
        updateServo(arm, armAngle, gamepad1.rightY());
        updateServo(claw, clawAngle, gamepad1.rightX());

        // Telemetry Data
        telemetry.addData("Current Arm Angle: ", armAngle);
        telemetry.addData("Current Claw Angle: ", clawAngle);
        telemetry.addData("Joy - Left X", gamepad1.leftX());
        telemetry.addData("Joy - Left Y", gamepad1.leftY());
        telemetry.addData("Joy - Right X", gamepad1.rightX());
        telemetry.addData("Joy - Right Y", gamepad1.rightY());
    }

    void stop() override {
        arm.detach();
        claw.detach();
    }
};


REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);
