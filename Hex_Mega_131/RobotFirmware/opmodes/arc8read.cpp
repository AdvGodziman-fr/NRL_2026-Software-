#include "NRL.h"

static constexpr int SENSOR_S2_PIN = DIGITAL_7;
static constexpr int SENSOR_S1_PIN = DIGITAL_6;
static constexpr int SENSOR_S0_PIN = DIGITAL_5;
static constexpr int SENSOR_SIG_PIN = DIGITAL_4;
static constexpr const char* SENSOR_KEYS[] = {
    "IR0", "IR1", "IR2", "IR3", "IR4", "IR5", "IR6", "IR7"
};

class Arc8Test : public NRLOpMode {
public:
    void init() override {
        pinMode(SENSOR_S2_PIN, OUTPUT);
        pinMode(SENSOR_S1_PIN, OUTPUT);
        pinMode(SENSOR_S0_PIN, OUTPUT);
        pinMode(SENSOR_SIG_PIN, INPUT);
    }

    void loop() override {
        for (uint8_t sensor = 0; sensor < 8; ++sensor) {
            digitalWrite(SENSOR_S0_PIN, (sensor & 0x01) ? HIGH : LOW);
            digitalWrite(SENSOR_S1_PIN, (sensor & 0x02) ? HIGH : LOW);
            digitalWrite(SENSOR_S2_PIN, (sensor & 0x04) ? HIGH : LOW);
            delayMicroseconds(500);

            const int state = digitalRead(SENSOR_SIG_PIN);
            telemetry.addData(SENSOR_KEYS[sensor], state == HIGH ? "HIGH" : "LOW");
        }
    }
    void stop() override {}
};

REGISTER_OPMODE(Arc8Test, "Arc8 Diagnostic", TELEOP);