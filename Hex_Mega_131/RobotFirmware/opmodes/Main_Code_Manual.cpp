#include "NRL.h"
#include <cmath>

// ---------------- BAY SERVO SETTINGS (CONTINUOUS) ----------------
// For a continuous servo, setPosition() is a speed/direction command, not an angle
//   90 = stop, below 90 = one direction, above 90 = the other. Further from 90 = faster.
constexpr float BAY_STOP_POS = 90.0f;

// Bay controls (hold-to-move): hold DPAD_LEFT = bay up, hold DPAD_RIGHT = bay down.
// If the bay moves the wrong way, swap these two values.
constexpr float BAY_UP_POS = 0.0f;        // Full speed counterclockwise (try 45.0f for slower)
constexpr float BAY_DOWN_POS = 180.0f;    // Full speed clockwise (try 135.0f for slower)

// ---------------- HARDWARE ----------------
static HexaDCMotor leftMotor {{.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true}};

static HexaServo arm {{.signalPin = SERVO_1, .minAngle = 29.5f, .maxAngle = 88.5f, .startAngle = 30.0f, .settleMs = 30}};
static HexaServo claw {{.signalPin = SERVO_4, .minAngle = 90.0f, .maxAngle = 175.0f, .startAngle = 90.0f, .settleMs = 30}};

// Bay: settleMs is left at its default of 0 so the PWM signal is never auto-detached.
// (A detached continuous servo gets no signal, so it would stop turning.)
static HexaServo bay {{.signalPin = SERVO_3, .minAngle = 0.0f, .maxAngle = 180.0f, .startAngle = BAY_STOP_POS}};

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
class Main_Code_Manual : public NRLOpMode {
    float armAngle;         
    float clawAngle;  
    int currentAngleIndex;  
    bool clawOpen;

    // Continuous servo state: +1 = rotating up, -1 = rotating down, 0 = stopped
    int bayDir;

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

    // ---------------- BAY CONTROL (CONTINUOUS, HOLD-TO-MOVE) ----------------
    void servoBay() {
        bool upHeld   = gamepad1.pressed(BTN_DPAD_LEFT);   // LEFT  = bay up
        bool downHeld = gamepad1.pressed(BTN_DPAD_RIGHT);  // RIGHT = bay down

        // Direction follows exactly what is held. Both held (or neither) = stop.
        int newDir = 0;
        if (upHeld && !downHeld)      newDir = +1;
        else if (downHeld && !upHeld) newDir = -1;

        // Send a command only when the direction changes, so it never flip-flops.
        if (newDir != bayDir) {
            bayDir = newDir;
            if (bayDir > 0)      bay.setPosition(BAY_UP_POS);
            else if (bayDir < 0) bay.setPosition(BAY_DOWN_POS);
            else                 bay.setPosition(BAY_STOP_POS);
        }

        telemetry.addData("Bay Dir (+1 up, -1 down): ", bayDir);
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

        // Force continuous servo into stopped neutral state immediately on init.
        bay.setPosition(BAY_STOP_POS);
        bayDir = 0;

        // Initialize software angle tracking from the servo's reported positions.
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
        // Neutralize and detach all servos when the OpMode stops.
        bay.setPosition(BAY_STOP_POS);
        bay.detach();
        arm.detach();
        claw.detach();
    }
};

REGISTER_OPMODE(Main_Code_Manual, "Main_Code", TELEOP);