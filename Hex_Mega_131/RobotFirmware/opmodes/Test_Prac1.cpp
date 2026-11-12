#include "NRL.h"

static HexaLineArray line{ hexaLineConfig() }; // 8-sensor bar configuration (ch0 to ch7)

class Test_Prac1 : public NRLOpMode {
    int angle_ = 0;
    uint32_t lastAngleChangeMs_ = 0;
    bool sweepComplete_ = false;

public:
    void init() override {
        Serial.begin(115200);
        line.begin(); // Initializes the sensor array and loads saved calibration data
    }

    void loop() override {
        // Print the raw or normalized readings (0–1000) for all 8 IR sensors
        Serial.print("Sensors [0-7]: ");
        for (uint8_t ch = 0; ch < 8; ch++) {
            uint16_t reading = line.read(ch);
            Serial.print(reading);
            Serial.print("\t");
        }
        Serial.println();

        delay(100); // Small delay to readable output rate
    }

    void stop() override {
        ;
    }
};

REGISTER_OPMODE(Test_Prac1, "Tes1", TELEOP);