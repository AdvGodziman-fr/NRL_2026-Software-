#include "NRL.h"

// SPI wiring for this example. These GPIOs are available on the digital
// breakout; leave them disconnected from any other peripherals while testing.
static constexpr int IMU_CS_PIN   = DIGITAL_4;
static constexpr int IMU_SCK_PIN  = DIGITAL_1;
static constexpr int IMU_MISO_PIN = DIGITAL_2;
static constexpr int IMU_MOSI_PIN = DIGITAL_3;

static Adafruit_LSM6DSOX imu;

class IMU_Data_Read : public NRLOpMode {
    bool imuReady_ = false;

public:
    void init() override {
        SPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
        imuReady_ = imu.begin_SPI(IMU_CS_PIN, &SPI);
        telemetry.addData("imu_status", imuReady_ ? "connected" : "not found");
    }

    void loop() override {
        if (!imuReady_) return;

        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        if (!imu.getEvent(&accel, &gyro, &temp)) {
            telemetry.addData("imu_status", "read failed");
            return;
        }

        telemetry.addData("accel_x", accel.acceleration.x);
        telemetry.addData("accel_y", accel.acceleration.y);
        telemetry.addData("accel_z", accel.acceleration.z);
        telemetry.addData("gyro_x", gyro.gyro.x);
        telemetry.addData("gyro_y", gyro.gyro.y);
        telemetry.addData("gyro_z", gyro.gyro.z);
        telemetry.addData("temp_c", temp.temperature);
    }
};

REGISTER_OPMODE(IMU_Data_Read, "IMU Data Read", TELEOP);
