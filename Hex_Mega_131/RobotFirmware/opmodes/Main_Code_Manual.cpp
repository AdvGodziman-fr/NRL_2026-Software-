#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM}};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }};
static HexaServo arm{{.signalPin = SERVO_1, .maxAngle = 90, .startAngle = 30.0f, .settleMs = 0 }};
static HexaServo claw{{.signalPin = SERVO_4, .maxAngle = 165, .startAngle = 90.0f, .settleMs = 0 }};
static TankDrive drive(leftMotor, rightMotor);
static HexaIMU imu { hexaImuConfig()};

static constexpr float SERVO_JOYSTICK_DEADZONE = 0.08f;
static constexpr float SERVO_CONTROL_RATE_DPS = 60.0f;
static constexpr float TELEOP_TICK_SECONDS = 0.01f;

class Main_Code_Manual : public NRLOpMode {
    int angleclaw_ = 90;
    int anglearm_ = 30;
    float ARM_MIN_ANGLE = arm.getMinAngle();
    float ARM_MAX_ANGLE = arm.getMaxAngle();
    float CLAW_MAX_ANGLE = claw.getMaxAngle();
    float CLAW_MIN_ANGLE = claw.getMinAngle();
    bool maxspeed_ = false;
    bool minspeed_ = false;

    void updateServoWithJoystick(float axis, float& angle, float minAngle, float maxAngle, HexaServo& servo) {
        const float input = fmaxf(-1.0f, fminf(axis, 1.0f));
        const float magnitude = fabsf(input);
        if (magnitude <= SERVO_JOYSTICK_DEADZONE) {
            return;
        }

        const float scaledInput = copysignf(
            (magnitude - SERVO_JOYSTICK_DEADZONE) / (1.0f - SERVO_JOYSTICK_DEADZONE),
            input);
        const float nextAngle = fmaxf(minAngle, fminf(
            angle + scaledInput * SERVO_CONTROL_RATE_DPS * TELEOP_TICK_SECONDS,
            maxAngle));
        if (nextAngle != angle) {
            angle = nextAngle;
            servo.setPosition(angle);
        }
    }

public:
    void init() override {
        imu.begin();
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
    }

    void loop() override {
        const float leftX = gamepad1.leftX();
        const float leftY = gamepad1.leftY();
        const float rightX = gamepad1.rightX();
        const float rightY = gamepad1.rightY();
        // These are DC Motor (Wheel) Operations
        drive.setScale(gamepad1.pressed(BTN_LB) ? 0.15f : 1.00f);
        drive.drive(gamepad1.leftY(), (gamepad1.leftX()));

        // Servo Motor Results
        float v = power.getServoCurrent();
        telemetry.addData("Servo Current: ", v);

        // These are for Servo-Motor operations
        updateServoWithJoystick(gamepad1.rightY(), anglearm_, ARM_MIN_ANGLE, ARM_MAX_ANGLE, arm);
        updateServoWithJoystick(gamepad1.rightX(), angleclaw_, CLAW_MIN_ANGLE, CLAW_MAX_ANGLE, claw);
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);
