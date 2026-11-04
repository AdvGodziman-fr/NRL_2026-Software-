#include "NRL.h"
#include <cmath>

// ---------------- HARDWARE DECLARATION ----------------
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor {{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }};
static HexaServo arm {{ .signalPin = SERVO_1, .minAngle = 15.0f, .maxAngle = 90.0f, .startAngle = 30.0f, .settleMs = 0 }};
static HexaServo claw {{ .signalPin = SERVO_4, .minAngle = 35.0f, .maxAngle = 175.0f, .startAngle = 90.0f, .settleMs = 0 }};
static TankDrive drive(leftMotor, rightMotor);

// ---------------- SERVO SETTINGS ----------------
constexpr float DEADZONE = 0.08f;
constexpr float SERVO_SPEED = 60.0f;
constexpr float LOOP_TIME = 0.01f;
//Defining array for arm angles:
constexpr int angles[] = {88.5, 50, 29.5}; // The angles you want to toggle between
constexpr int TOTAL_ANGLES = sizeof(angles) / sizeof(angles[0]);
static int currentAngleIndex = 0;
//Static counter for claw state:
static bool counter1;

// ---------------- TURN SETTINGS ----------------
constexpr float MIN_TURN_SCALE = 0.35f;
constexpr float MAX_TURN_SCALE = 1.00f;

//Main Class
class Main_Test_Code : public NRLOpMode {
    float armAngle;
    float clawAngle;

    // ---------------- SERVO CONTROL ----------------
    //For Arm: (Toggleable angle states)
    void servoArm(HexaServo &servo, float &currentAngle){
       //Using pre-defined array to toggle trough set angles 
        if (gamepad1.pressed(BTN_RB)){
            currentAngleIndex = (currentAngleIndex + 1) % TOTAL_ANGLES;
            currentAngle = angles[currentAngleIndex];
        }
        servo.setPosition(currentAngle);
        //Adding telemetry data to inform driver
        telemetry.addData("Arm Angle Currrent:", currentAngle);
    }

     //For Claw: (Gradual angle open close states)
     void servoClaw(HexaServo &servo, float &currentAngle){
       //Using a pre-defiend claw speed to open and close:
        const float speed = 1.0f; // Adjust this to change how fast it opens/closes
    
        //Listening for the button to TOGGLE the target state
        if (gamepad1.pressed(BTN_DPAD_UP)) {
            counter1 = !counter1; // Flip the state once per press
        }

        //Move gradually frame-by-frame based on the target state
        if (counter1) {
            // Gradually open up to 145 degrees
            if (currentAngle < 145.0f) {
                currentAngle += speed;
            }
        } 
        else {
            // Gradually close down to 89 degrees
            if (currentAngle > 89.0f) {
                currentAngle -= speed;
            }
        }
        servo.setPosition(currentAngle);
        //Adding telemetry data to inform driver
        telemetry.addData("Claw Angle Currrent:", currentAngle);
    }
    
    // ---------------- SPEED-DEPENDENT DRIVE ----------------
    void driveWithSpeedDependentTurning() {
        float forward = gamepad1.leftY();
        float turn = gamepad1.rightX();

        // Reduce turning sensitivity as forward speed increases.
        float turnScale = MIN_TURN_SCALE + (MAX_TURN_SCALE - MIN_TURN_SCALE) * std::fabs(forward);
        turn *= turnScale;

        // Slow mode while LB is held.
        drive.setScale(gamepad1.pressed(BTN_LB) ? 0.15f : 1.00f);

        // Apply drivetrain movement.
        drive.drive(forward, turn);
    }

public:
    void init() override {
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();
        armAngle = arm.getPosition();
        clawAngle = claw.getPosition();
    }

    void loop() override {
        // -------- DRIVE CONTROL --------
        driveWithSpeedDependentTurning();

        // -------- SERVO CONTROL --------


        // Telemetry Data
        telemetry.addData("Current Arm Angle: ", armAngle);
        telemetry.addData("Current Claw Angle: ", clawAngle);
        telemetry.addData("Joy - Left X", gamepad1.leftX());
        telemetry.addData("Joy - Left Y", gamepad1.leftY());
        telemetry.addData("Joy - Right X", gamepad1.rightX());
        telemetry.addData("Joy - Right Y", gamepad1.rightY());
    }

    void stop() override {
        arm.detach();
        claw.detach();
    }
};


REGISTER_OPMODE(Main_Test_Code, "Test_Code", TELEOP);