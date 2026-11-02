#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM}};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM}};

static constexpr int ARM_MIN_ANGLE = 0;
static constexpr int ARM_MAX_ANGLE = 63;   // Change this to set the arm's maximum angle.
static constexpr int CLAW_MIN_ANGLE = 0;
static constexpr int CLAW_MAX_ANGLE = 174; // Change this to set the claw's maximum angle.
static constexpr int ARM_START_ANGLE = 30;
static constexpr int CLAW_START_ANGLE = 90;
static HexaServo arm{{
    .signalPin = SERVO_1,
    .minAngle = ARM_MIN_ANGLE,
    .maxAngle = ARM_MAX_ANGLE,
    .startAngle = ARM_START_ANGLE,
    .settleMs = 0
}};

static HexaServo claw{{
    .signalPin = SERVO_4,
    .minAngle = CLAW_MIN_ANGLE,
    .maxAngle = CLAW_MAX_ANGLE,
    .startAngle = CLAW_START_ANGLE,
    .settleMs = 0
}};

static TankDrive drive(leftMotor, rightMotor);

static constexpr float MOVING_TURN_THRESHOLD = 0.25f;
static constexpr float MOVING_TURN_INNER_TO_OUTER_RATIO = 0.4f;
static constexpr float SERVO_JOYSTICK_DEADZONE = 0.08f;
static constexpr float SERVO_CONTROL_RATE_DPS = 60.0f;
static constexpr float TELEOP_TICK_SECONDS = 0.01f;

// SPI wiring for the LSM6DSOX; keep these breakout pins free while in use.
static constexpr int IMU_CS_PIN   = DIGITAL_4;
static constexpr int IMU_SCK_PIN  = DIGITAL_1;
static constexpr int IMU_MISO_PIN = DIGITAL_2;
static constexpr int IMU_MOSI_PIN = DIGITAL_3;
static Adafruit_LSM6DSOX imu;

class Main_Test_Code : public NRLOpMode {
    float angleclaw_ = CLAW_START_ANGLE;
    float anglebrdg_ = ARM_START_ANGLE;
    int gear_;
    float G_x;
    float G_y;
    float G_z;
    bool maxspeed_ = false;
    bool minspeed_ = false;
    bool imuReady_ = false;

    void updateServoWithJoystick(float axis, float& angle, float minAngle,
                                 float maxAngle, HexaServo& servo) {
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

    void driveWithSpeedDependentTurn(float forward, float turn) {
        const float forwardMagnitude = fabsf(forward);
        const float turnMagnitude = fabsf(turn);

        if (forwardMagnitude < MOVING_TURN_THRESHOLD || turnMagnitude == 0.0f) {
            drive.drive(forward, turn);
            return;
        }

        // Keep the inside wheel at 3/4 of the outside wheel. Below full speed,
        // speed up the outside wheel; at the output limit, slow the inside wheel.
        const float outsideMagnitude = fminf(
            forwardMagnitude / MOVING_TURN_INNER_TO_OUTER_RATIO, 1.0f);
        const float insideMagnitude =
            outsideMagnitude * MOVING_TURN_INNER_TO_OUTER_RATIO;
        const float direction = forward < 0.0f ? -1.0f : 1.0f;
        float left = direction * insideMagnitude;
        float right = direction * insideMagnitude;

        if (turn > 0.0f) {
            left = direction * outsideMagnitude;
        } else {
            right = direction * outsideMagnitude;
        }

        drive.drive((left + right) * 0.5f, (left - right) * 0.5f);
    }

public:
    void init() override {
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
        drive.setRampRate(0.75f);
    }

    void loop() override {
        // Read each joystick axis once so controls and telemetry use the same sample.
        const float leftX = gamepad1.leftX();
        const float leftY = gamepad1.leftY();
        const float rightX = gamepad1.rightX();
        const float rightY = gamepad1.rightY();
        telemetry.addData("joy_left_x", leftX);
        telemetry.addData("joy_left_y", leftY);
        telemetry.addData("joy_right_x", rightX);
        telemetry.addData("joy_right_y", rightY);

        // Stick deflection changes servo angle at a proportional rate; neutral holds position.
        updateServoWithJoystick(rightY, anglebrdg_, ARM_MIN_ANGLE, ARM_MAX_ANGLE, arm);
        updateServoWithJoystick(rightX, angleclaw_, CLAW_MIN_ANGLE, CLAW_MAX_ANGLE, claw);

        // Hold D-pad right for an in-place pivot; left X sets the turn direction
        // and amount. Otherwise use speed-dependent steering while driving.
        if (gamepad1.pressed(BTN_DPAD_RIGHT)) {
            drive.drive(0.0f, leftX);
        } else {
            driveWithSpeedDependentTurn(leftY, leftX);
        }

        if (imuReady_) {
            sensors_event_t accel;
            sensors_event_t gyro;
            sensors_event_t temp;
            if (imu.getEvent(&accel, &gyro, &temp)) {
                telemetry.addData("accel_x", accel.acceleration.x);
                telemetry.addData("accel_y", accel.acceleration.y);
                telemetry.addData("accel_z", accel.acceleration.z);
                telemetry.addData("gyro_x", gyro.gyro.x);
                telemetry.addData("gyro_y", gyro.gyro.y);
                telemetry.addData("gyro_z", gyro.gyro.z);
                telemetry.addData("temp_c", temp.temperature);

                //calcuating G-Force in all axis:
                G_x = (accel.acceleration.x)/9.806;
                G_y = (accel.acceleration.y)/9.806;
                G_z = (accel.acceleration.z)/9.806;

                telemetry.addData("GF_x", G_x);
                telemetry.addData("GF_y", G_y);
                telemetry.addData("GF_z", G_z);
            } else {
                telemetry.addData("imu_status", "read failed");
            }

        }

        // Servo Motor Results
        float v = power.getServoCurrent();
        telemetry.addData("Servo Current: ", v);
        telemetry.addData("claw_angle", angleclaw_);
        telemetry.addData("claw_max", CLAW_MAX_ANGLE);
        telemetry.addData("arm_angle", anglebrdg_);
        telemetry.addData("arm_max", ARM_MAX_ANGLE);

    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Test_Code, "Test_Code", TELEOP);