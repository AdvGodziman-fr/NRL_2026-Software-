#include "NRL.h"

// Hardware Declarations
static HexaDCMotor leftMotor {{ 
    .dirPin = MOTOR_L_DIR, 
    .pwmPin = MOTOR_L_PWM, 
    .flipped = true, 
    .minDrive = 29 
}};

static HexaDCMotor rightMotor {{ 
    .dirPin = MOTOR_R_DIR, 
    .pwmPin = MOTOR_R_PWM, 
    .minDrive = 32 
}};

static TankDrive drive(leftMotor, rightMotor);
static HexaLineArray line{ hexaLineConfig() }; // 8-sensor bar configuration
static LineActions lineActions(drive, line);    // Line-following control engine

class Test : public NRLOpMode {
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        line.begin(); // Initializes sensor array and loads saved calibration data
    }

    void start() override {
        // Runs once when the match starts — queue your routine
        // with runAction(...) here.
    }

    void loop() override {
        // Read raw or normalized value from a specific sensor channel (ch0 to ch7)
        uint16_t ch3Reading = line.read(3);

        // Read calculated line metrics
        bool onLineState = line.onLine();      // Checks if any sensor detects the line
        int16_t pos = line.position();         // Center position (0–7000; 3500 is dead center)
        int16_t err = line.error();           // Error offset (-3500 to +3500)
        uint8_t sensorMask = line.mask();     // Bitmask of active channels (e.g., 00011000)
        bool greyDetected = line.onGrey();     // Checks if sensor array is over a grey boundary

        // Keep the robot centered on the line during basic testing
        lineActions.update();
    }

    void stop() override {
        // Runs once on STOP — stop your motors/servos here.
    }
};

REGISTER_OPMODE(Test, "Line Test", AUTO);
