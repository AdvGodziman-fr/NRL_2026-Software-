
#include "NRL.h"
#include <cmath>

// ---------------- HARDWARE DECLARATION ----------------

static HexaDCMotor leftMotor {{
    .dirPin = MOTOR_L_DIR,
    .pwmPin = MOTOR_L_PWM
}};

static HexaDCMotor rightMotor {{
    .dirPin = MOTOR_R_DIR,
    .pwmPin = MOTOR_R_PWM,
    .flipped = true
}};

static HexaServo arm {{
    .signalPin = SERVO_1,
    .minAngle = 30.0f,
    .maxAngle = 43.5f,
    .startAngle = 30.0f,
    .settleMs = 0
}};

static HexaServo claw {{
    .signalPin = SERVO_4,
    .minAngle = 90.0f,
    .maxAngle = 175.0f,
    .startAngle = 90.0f,
    .settleMs = 0
}};

static TankDrive drive(leftMotor, rightMotor);


// ---------------- SERVO SETTINGS ----------------

constexpr float DEADZONE = 0.1f;
constexpr float SERVO_SPEED = 80.0f;
constexpr float LOOP_TIME = 0.01f;


// ---------------- TURN SETTINGS ----------------

constexpr float MIN_TURN_SCALE = 0.55f;
constexpr float MAX_TURN_SCALE = 1.00f;
constexpr float SLOW_MODE_SCALE = 0.45f;


// ---------------- MAIN CLASS ----------------

class Main_Code_Manual : public NRLOpMode {

    float armAngle;
    float clawAngle;


    // ---------------- SERVO CONTROL ----------------

    void updateServo(
        HexaServo &servo,
        float &currentAngle,
        float joystickInput
    ) {

        if (std::fabs(joystickInput) < DEADZONE) {
            return;
        }

        float newAngle = currentAngle +
            joystickInput * SERVO_SPEED * LOOP_TIME;

        // Clamp BEFORE sending the servo command.
        newAngle = std::fmax(
            servo.getMinAngle(),
            std::fmin(servo.getMaxAngle(), newAngle)
        );

        // Avoid unnecessary repeated commands.
        if (newAngle != currentAngle) {
            currentAngle = newAngle;
            servo.setPosition(currentAngle);
        }
    }


    // ---------------- SPEED-DEPENDENT DRIVE ----------------

    void driveWithSpeedDependentTurning() {

        float forward = -gamepad1.leftY();
        float turn = gamepad1.leftX();

        float forwardMagnitude = std::fabs(forward);

        float turnScale = MAX_TURN_SCALE -
            (MAX_TURN_SCALE - MIN_TURN_SCALE) *
            forwardMagnitude;

        turn *= turnScale;

        drive.setScale(
            gamepad1.pressed(BTN_LB)
                ? SLOW_MODE_SCALE
                : 1.00f
        );

        drive.drive(forward, turn);
    }


public:

    // ---------------- INITIALIZATION ----------------

    void init() override {

        arm.begin();
        claw.begin();

        leftMotor.begin();
        rightMotor.begin();

        armAngle = arm.getPosition();
        clawAngle = claw.getPosition();
    }


    // ---------------- MAIN LOOP ----------------

    void loop() override {

        driveWithSpeedDependentTurning();

        updateServo(
            arm,
            armAngle,
            gamepad1.rightY()
        );

        updateServo(
            claw,
            clawAngle,
            gamepad1.rightX()
        );

        telemetry.addData("Current Arm Angle", armAngle);
        telemetry.addData("Current Claw Angle", clawAngle);

        telemetry.addData("Left X", gamepad1.leftX());
        telemetry.addData("Left Y", gamepad1.leftY());

        telemetry.addData("Right X", gamepad1.rightX());
        telemetry.addData("Right Y", gamepad1.rightY());
    }

    // ---------------- STOP ----------------
    void stop() override {
        arm.detach();
        claw.detach();
    }
};


REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);
