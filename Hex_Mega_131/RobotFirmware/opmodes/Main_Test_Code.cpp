#include "NRL.h"
#include <cmath>

// ---------------- BAY SERVO SETTINGS (360-DEGREE POSITIONAL) ----------------
// The bay is driven by raw pulse width so its range is not limited by the 0-180
// angle mapping. 500-2500 us only gave ~180 deg of travel, so the span is widened.
// CALIBRATE: these two values are the two ends of the servo's travel.
//   - Hold a button and watch the telemetry "Bay Pulse (us)". Note the value where
//     the servo reaches a physical end, then set the limit slightly before it.
//   - Don't keep holding at an end if it buzzes or strains.
constexpr float BAY_MIN_PULSE_US = 400.0f;    // One end of travel
constexpr float BAY_MAX_PULSE_US = 2600.0f;   // Other end of travel

// While a D-pad button is held, the pulse width moves by this many microseconds
// per loop tick (100 Hz). 10 = 1000 us/s. Raise for faster, lower for finer.
constexpr float BAY_STEP_US = 10.0f;

// ---------------- HARDWARE ----------------
static HexaDCMotor leftMotor {{.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true}};

static HexaServo arm {{.signalPin = SERVO_1, .minAngle = 29.5f, .maxAngle = 88.5f, .startAngle = 30.0f, .settleMs = 30}};
static HexaServo claw {{.signalPin = SERVO_4, .minAngle = 90.0f, .maxAngle = 175.0f, .startAngle = 90.0f, .settleMs = 30}};

// Bay: settleMs is left at its default of 0 so the servo keeps holding its position.
static HexaServo bay {{.signalPin = SERVO_3, .minAngle = 0.0f, .maxAngle = 180.0f, .startAngle = 90.0f}};

static TankDrive drive(leftMotor, rightMotor);

// ---------------- SERVO SETTINGS ----------------
// Arm positions selected sequentially using RB.
constexpr float ARM_ANGLES[] = {60.0f, 50.0f, 40.5f};
constexpr int TOTAL_ANGLES = sizeof(ARM_ANGLES) / sizeof(ARM_ANGLES[0]);

// Claw alternates between these two target positions.
constexpr float CLAW_OPEN_ANGLE = 160.0f;
constexpr float CLAW_CLOSED_ANGLE = 125.0f;

// Claw moves by this many degrees per loop iteration.
constexpr float CLAW_STEP = 1.0f;

// ---------------- DRIVE SETTINGS ----------------
// Turning sensitivity decreases as forward/reverse speed increases. 
constexpr float MIN_TURN_SCALE = 0.55f;
constexpr float MAX_TURN_SCALE = 1.00f;

// Drive output is reduced to 25% while LB is held.
constexpr float SLOW_MODE_SCALE = 0.45f;

// ---------------- MAIN CLASS ----------------
class Main_Test_Code : public NRLOpMode {
    float armAngle;         
    float clawAngle;  
    int currentAngleIndex;  
    bool clawOpen;

    // Bay target pulse width in microseconds
    float bayPulseUs;

    // ---------------- ARM CONTROL ----------------
    void servoArm() {
        // justPressed() triggers once when RB transitions from released to pressed.
        if (gamepad1.justPressed(BTN_RB)) {
            // Advance to the next preset and return to the first after the last.
            currentAngleIndex = (currentAngleIndex + 1) % TOTAL_ANGLES;
            float newAngle = ARM_ANGLES[currentAngleIndex];

            // Clamp the target to the arm's configured limits before commanding it.
            newAngle = std::fmax(arm.getMinAngle(), std::fmin(arm.getMaxAngle(), newAngle));

            // Send a command only if the selected position is different.
            if (newAngle != armAngle) {
                armAngle = newAngle;
                arm.setPosition(armAngle);
            }
        }
        telemetry.addData("Current Arm Angle: ", armAngle);
    }

    // ---------------- CLAW CONTROL ----------------
    void servoClaw() {
        // Each DPAD_UP press switches the claw's target between open and closed. 
        if (gamepad1.justPressed(BTN_DPAD_UP)) {
            clawOpen = !clawOpen;
        }

        // Choose the target based on the current toggle state.
        float targetAngle = clawOpen ? CLAW_OPEN_ANGLE : CLAW_CLOSED_ANGLE;
        float newAngle = clawAngle;

        // Move gradually towards the target instead of jumping instantly.
        if (clawAngle < targetAngle) {
            newAngle = std::fmin(clawAngle + CLAW_STEP, targetAngle);
        } else if (clawAngle > targetAngle) {
            newAngle = std::fmax(clawAngle - CLAW_STEP, targetAngle);
        }

        // Prevent commands outside the claw's configured range.
        newAngle = std::fmax(claw.getMinAngle(),
            std::fmin(claw.getMaxAngle(), newAngle));

        // Update the servo only when its commanded angle changes.
        if (newAngle != clawAngle) {
            clawAngle = newAngle;
            claw.setPosition(clawAngle);
        }

        telemetry.addData("Current Claw Angle: ", clawAngle);
        telemetry.addData("Claw State: ", clawOpen ? 1 : 0);
    }

    // ---------------- BAY CONTROL (HOLD-TO-SWEEP) ----------------
    void servoBay() {
        bool leftHeld  = gamepad1.pressed(BTN_DPAD_LEFT);   // LEFT  = bay up
        bool rightHeld = gamepad1.pressed(BTN_DPAD_RIGHT);  // RIGHT = bay down

        // Sweep the pulse width while a button is held; release keeps the servo where it is.
        // Both held = no movement. If the directions are reversed, swap the +/- below.
        float newPulse = bayPulseUs;
        if (leftHeld && !rightHeld)      newPulse -= BAY_STEP_US;
        else if (rightHeld && !leftHeld) newPulse += BAY_STEP_US;

        newPulse = std::fmax(BAY_MIN_PULSE_US, std::fmin(BAY_MAX_PULSE_US, newPulse));

        // Send a command only when the target actually changes.
        if (newPulse != bayPulseUs) {
            bayPulseUs = newPulse;
            bay.writeMicroseconds((uint16_t)(bayPulseUs + 0.5f));
        }

        telemetry.addData("Bay Pulse (us): ", bayPulseUs);
    }

    // ---------------- DRIVE CONTROL ----------------
    void driveWithSpeedDependentTurning() {
        float forward = -gamepad1.leftY();
        float turn = -gamepad1.rightX();

        float forwardMagnitude = std::fabs(forward);

        // Interpolate turn sensitivity from 100% to 55% as drive input increases.
        float turnScale = MAX_TURN_SCALE - (MAX_TURN_SCALE - MIN_TURN_SCALE) * forwardMagnitude;

        // Apply the speed-dependent turning adjustment.
        turn *= turnScale;

        // LB enables slow mode only while the button is held.
        drive.setScale(gamepad1.pressed(BTN_LB) ? SLOW_MODE_SCALE : 1.00f);

        // Send the adjusted forward and turning commands to the drivetrain.
        drive.drive(forward, turn);
    }

public:
    // ---------------- INITIALIZATION ----------------
    void init() override {
        // Initialize all servos and drive motors.
        bay.begin();
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();

        // Initialize software angle tracking from the servo's reported positions.
        bayPulseUs = (BAY_MIN_PULSE_US + BAY_MAX_PULSE_US) / 2.0f;   // start mid-travel
        bay.writeMicroseconds((uint16_t)(bayPulseUs + 0.5f));
        armAngle = arm.getPosition();
        clawAngle = claw.getPosition();
        currentAngleIndex = TOTAL_ANGLES - 1;
        clawOpen = false;
    }

    // ---------------- MAIN LOOP ----------------
    void loop() override {
        driveWithSpeedDependentTurning();
        servoArm();
        servoClaw();
        servoBay();
    }

    // ---------------- STOP ----------------
    void stop() override {
        // Detach all servos when the OpMode stops.
        bay.detach();
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Test_Code, "Test_Code", TELEOP);