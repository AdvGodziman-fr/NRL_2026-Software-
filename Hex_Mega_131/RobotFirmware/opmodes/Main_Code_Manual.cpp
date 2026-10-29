#include "NRL.h"

// Declare your hardware here (file-scope), e.g.:
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM}};
static HexaServo arm{{.signalPin = SERVO_1, .startAngle = 30.0f, .settleMs = 0}};
static HexaServo claw{{.signalPin = SERVO_4, .startAngle = 90.0f, .settleMs = 0}};
static TankDrive drive(leftMotor, rightMotor);

// SPI wiring for the LSM6DSOX; keep these breakout pins free while in use.
static constexpr int IMU_CS_PIN   = DIGITAL_4;
static constexpr int IMU_SCK_PIN  = DIGITAL_1;
static constexpr int IMU_MISO_PIN = DIGITAL_2;
static constexpr int IMU_MOSI_PIN = DIGITAL_3;
static Adafruit_LSM6DSOX imu;

class Main_Code_Manual : public NRLOpMode {
    int angleclaw_ = 90;
    int anglebrdg_ = 30;
    float G_x;
    float G_y;
    float G_z;
    bool maxangleclaw_ = false;
    bool maxanglearm_ = false;
    bool maxspeed_ = false;
    bool minspeed_ = false;
    bool imuReady_ = false;

public:
    void init() override {
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();

        SPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
        imuReady_ = imu.begin_SPI(IMU_CS_PIN, &SPI);
        telemetry.addData("imu_status", imuReady_ ? "connected" : "not found");
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
