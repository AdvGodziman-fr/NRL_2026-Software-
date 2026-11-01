#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }};
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
static constexpr float MOVING_TURN_FORWARD_LIMIT = 1.0f;

// SPI wiring for the LSM6DSOX; keep these breakout pins free while in use.
static constexpr int IMU_CS_PIN   = DIGITAL_4;
static constexpr int IMU_SCK_PIN  = DIGITAL_1;
static constexpr int IMU_MISO_PIN = DIGITAL_2;
static constexpr int IMU_MOSI_PIN = DIGITAL_3;
static Adafruit_LSM6DSOX imu;

class Main_Code_Manual : public NRLOpMode {
    int angleclaw_ = CLAW_START_ANGLE;
    int anglebrdg_ = ARM_START_ANGLE;
    int gear_;
    float G_x;
    float G_y;
    float G_z;
    bool maxspeed_ = false;
    bool minspeed_ = false;
    bool imuReady_ = false;

    void driveWithSpeedDependentTurn(float forward, float turn) {
        const float forwardMagnitude = fabsf(forward);
        const float turnMagnitude = fabsf(turn);

        if (forwardMagnitude < MOVING_TURN_THRESHOLD || turnMagnitude == 0.0f) {
            drive.drive(forward, turn);
            return;
        }

        // While translating, hold the inside wheel at the base speed and
        // accelerate only the outside wheel. Cap the base speed to leave
        // headroom for steering, including when the forward stick is full.
        const float baseMagnitude = fminf(forwardMagnitude, MOVING_TURN_FORWARD_LIMIT);
        const float outsideMagnitude = fminf(baseMagnitude + turnMagnitude, 1.0f);
        const float direction = forward < 0.0f ? -1.0f : 1.0f;
        float left = direction * baseMagnitude;
        float right = direction * baseMagnitude;

        if ((forward > 0.0f && turn > 0.0f) ||
            (forward < 0.0f && turn < 0.0f)) {
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
        SPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
        imuReady_ = imu.begin_SPI(IMU_CS_PIN, &SPI);
        telemetry.addData("imu_status", imuReady_ ? "connected" : "not found");
    }

    void loop() override {
        // These are for Servo-Motor operations 
        if (gamepad1.pressed(BTN_DPAD_UP) && anglebrdg_ < ARM_MAX_ANGLE) {
            anglebrdg_++;
            arm.setPosition(anglebrdg_);
        }

        if (gamepad1.pressed(BTN_DPAD_DOWN) && anglebrdg_ > ARM_MIN_ANGLE) {
            --anglebrdg_;
            arm.setPosition(anglebrdg_);
        }

        if (gamepad1.pressed(BTN_DPAD_LEFT) && angleclaw_ > CLAW_MIN_ANGLE) {
            --angleclaw_;
            claw.setPosition(angleclaw_);
        }

        if (gamepad1.pressed(BTN_DPAD_RIGHT) && angleclaw_ < CLAW_MAX_ANGLE) {
            angleclaw_++;
            claw.setPosition(angleclaw_);
        }
       
        // These are DC Motor (Wheel) Operations
        driveWithSpeedDependentTurn(gamepad1.leftY(), gamepad1.rightX());

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