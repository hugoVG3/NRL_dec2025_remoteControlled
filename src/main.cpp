#include <Arduino.h>
#include <ps5Controller.h>

// ===========================================
// --- 1. CONFIGURATION CONSTANTS
// ===========================================

// --- A. PS5 Bluetooth Setup ---
const char* ps5MAC = "A0:FA:9C:70:16:5D";

// --- B. Motor PWM & Speed Control ---
const int DRIVE_DEAD_ZONE = 10;      // Prevents drift when triggers are released (0-255)
const int STEERING_DEAD_ZONE = 10;   // Prevents steering drift when LSX is centered (0-128)
const int MAX_SPIN_SPEED = 200;      // Max speed for standard spin (0-255)
const int MAX_ALT_SPIN_SPEED = 100;  // Max speed for Triangle-activated precise spin (0-255)

// --- C. Analog Correction ---
// The current value (-128) is kept based on your previous finding that it corrects the drift.
const int LSX_BIAS_CORRECTION = -128; 

// --- D. Servo Control and Limits ---
const int SERVO1_PIN = 14;           // Gripper Servo Pin (Pin 14)
const int SERVO2_PIN = 13;           // Arm Servo Pin (Pin 13)

// Arm Limits (Right Stick Y-axis)
const int ARM_MIN_ANGLE = 50;         // Fully UP position (50)
const int ARM_MAX_ANGLE = 150;       // Fully DOWN position (150)
const int ARM_DEAD_ZONE = 7;        // Dead zone for RStickY (0-128 range)
const int ARM_CENTER = 128;          // Center value for RStickY
const float ARM_SPEED_FACTOR = 0.05; // Base factor for exponential speed (smaller = slower overall)

// Gripper Limits (L1/R1 buttons)
const int GRIPPER_MIN_CLOSED_ANGLE = 115; // Fully Closed position (no block) (115)
const int GRIPPER_MAX_OPEN_ANGLE = 0;  // Fully Open position (0)
const int GRIPPER_FORCE_LIMIT_ANGLE = 0; // Absolute minimum angle for force clamping (allows pressing)

// ==========================================================
// --- 2. HARDWARE PIN DEFINITIONS ---
// ==========================================================

// --- Motor Wiring: DIR1=32, PWM1=33, DIR2=25, PWM2=26 (MDD10A) ---
#define M1_PWM_PIN  33 // Left Motor PWM
#define M1_DIR_PIN  32 // Left Motor Direction
#define M2_PWM_PIN  26 // Right Motor PWM
#define M2_DIR_PIN  25 // Right Motor Direction

// --- PWM Channel Configuration ---
#define M1_PWM_CHANNEL 0
#define M2_PWM_CHANNEL 1
#define S1_PWM_CHANNEL 2   // Gripper Servo Channel
#define S2_PWM_CHANNEL 3   // Arm Servo Channel
#define PWM_FREQ       5000   
#define SERVO_FREQ     50     // Standard servo frequency
#define PWM_RESOLUTION 8     // Motor resolution (0-255)
#define SERVO_RESOLUTION 16  // Servo resolution

// --- Global Servo State ---
int armAngle = (ARM_MIN_ANGLE + ARM_MAX_ANGLE) / 2; // Start in the middle
int gripperAngle = GRIPPER_MAX_OPEN_ANGLE;

// --- Function Declarations ---
void setupMotors();
void setupServos();
void setMotorSpeed(uint8_t channel, int speed);
void stopAllMotors();
void controlMotors();
void controlServos();
void writeServo(uint8_t channel, int angle);


// ===========================================
// --- 3. HARDWARE SETUP AND UTILITIES ---
// ===========================================

void setupMotors() {
    Serial.println("Configuring motor pins...");
    
    // Set pins LOW first to prevent boot-up drift
    digitalWrite(M1_DIR_PIN, LOW);
    digitalWrite(M2_DIR_PIN, LOW);
    pinMode(M1_DIR_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);
    
    // Configure PWM Channels (0-255 range)
    ledcSetup(M1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(M1_PWM_PIN, M1_PWM_CHANNEL);
    ledcSetup(M2_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(M2_PWM_PIN, M2_PWM_CHANNEL);

    stopAllMotors();
}

/**
 * @brief Sets the angle for the servo (0-180 degrees).
 * @param channel The PWM channel (S1_PWM_CHANNEL or S2_PWM_CHANNEL).
 * @param angle The angle (0-180).
 */
void writeServo(uint8_t channel, int angle) {
    // Standard PWM for 16-bit/50Hz: maps 0-180 degrees to 3276 to 6553 duty cycles (1ms to 2ms pulse)
    int duty = map(angle, 0, 180, 3276, 6553);
    ledcWrite(channel, duty);
}


void setupServos() {
    Serial.println("Configuring servo pins...");

    // Configure PWM Channels
    ledcSetup(S1_PWM_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    ledcAttachPin(SERVO1_PIN, S1_PWM_CHANNEL);
    ledcSetup(S2_PWM_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    ledcAttachPin(SERVO2_PIN, S2_PWM_CHANNEL);

    // Set initial positions
    writeServo(S1_PWM_CHANNEL, gripperAngle);
    writeServo(S2_PWM_CHANNEL, armAngle);
}


/**
 * @brief Sets the speed and direction for one motor using MDD10A (DIR/PWM).
 * (Same as before, ensuring motor functionality is unchanged)
 */
void setMotorSpeed(uint8_t channel, int speed) {
    speed = constrain(speed, -255, 255);
    int absSpeed = abs(speed);
    
    // Determine which direction pin to use based on the channel
    int dir_pin = (channel == M1_PWM_CHANNEL) ? M1_DIR_PIN : M2_DIR_PIN;

    if (absSpeed == 0) {
        // Stop: PWM 0, Direction LOW
        ledcWrite(channel, 0);
        digitalWrite(dir_pin, LOW); 
    } else if (speed > 0) {
        // Forward: Direction HIGH, apply speed
        digitalWrite(dir_pin, HIGH);
        ledcWrite(channel, absSpeed);
    } else { // speed < 0
        // Reverse: Direction LOW, apply speed
        digitalWrite(dir_pin, LOW);
        ledcWrite(channel, absSpeed);
    }
}

void stopAllMotors() {
    setMotorSpeed(M1_PWM_CHANNEL, 0); 
    setMotorSpeed(M2_PWM_CHANNEL, 0);
}

// ===========================================
// --- 4. PS5 CONTROL LOGIC FUNCTIONS ---
// ===========================================

/**
 * @brief Controls the arm (RSY) with exponential movement and the gripper (L1/R1).
 */
void controlServos() {
    
    // --- ARM CONTROL (RSY - LINEAR RATE FIX) 
    int rsy = -ps5.RStickY(); // 0 (Up) to 255 (Down), center is 128
    // Calculate signed movement relative to center (-128 to 127)
    int movement = rsy; 

    if (abs(movement) > ARM_DEAD_ZONE) {
        
        // --- PROGRESSIVE LINEAR SPEED IMPLEMENTATION ---
        // 1. Minimum speed (1) is guaranteed right outside the dead zone (10).
        // 2. Speed progressively increases up to 6 degrees/step at maximum stick throw (128).
        // This is the robust replacement for the fractional step ratio.
        int stepSpeed = map(abs(movement), ARM_DEAD_ZONE, 128, 1, 6); 

        if (movement < 0) { 
            // Stick UP (negative movement): Decrease angle (moves towards 75)
            armAngle -= stepSpeed;
        } else { 
            // Stick DOWN (positive movement): Increase angle (moves towards 125)
            armAngle += stepSpeed;
        }
        
        // Constrain to the safe physical limits (75 and 125)
        armAngle = constrain(armAngle, ARM_MIN_ANGLE, ARM_MAX_ANGLE);
        
        // Write new angle
        writeServo(S2_PWM_CHANNEL, armAngle);
        Serial.printf("Arm Angle: %d deg (RSY: %d, Step: %d)\n", armAngle, rsy, stepSpeed);
    }


    


    // --- GRIPPER CONTROL (L1/R1) - LINEAR STEPPING FIX ---

    bool gripperOpen = ps5.L1(); // L1 for OPEN (Move towards 0)
    bool gripperClose = ps5.R1(); // R1 for CLOSE (Move towards 115)
    const int GRIPPER_STEP = 5; // Degrees to change per loop
    bool gripperMoved = false;

    if (gripperOpen && !gripperClose) {
        // L1: Open. Move angle DOWN towards the OPEN limit (35)
        gripperAngle -= GRIPPER_STEP;
        gripperMoved = true;
    } 
    else if (gripperClose && !gripperOpen) {
        // R1: Close. Move angle UP towards the CLOSED limit (115)
        gripperAngle += GRIPPER_STEP;
        gripperMoved = true;
    }

    if (gripperMoved) {
        // Constrain the angle to the safe physical limits (35 and 115)
        // constrain() will correctly handle that min (35) is lower than max (115).
        gripperAngle = constrain(gripperAngle, GRIPPER_MAX_OPEN_ANGLE, GRIPPER_MIN_CLOSED_ANGLE);
        
        // Write the new position
        writeServo(S1_PWM_CHANNEL, gripperAngle);
    }
    // If no button is pressed (or both are), the angle is held and writeServo is not called.
}

/**
 * Controls the robot's differential drive using L2/R2 for throttle and LSX for steering.
}
}
asdfgh
/**
 *  Controls the robot's differential drive using L2/R2 for throttle and LSX for steering.
 */
void controlMotors() {
    // --- 4.1. Get Raw Inputs & Correction ---
    
    int baseSpeed = ps5.R2Value() - ps5.L2Value(); 
    int lsx = ps5.LStickX(); 
    
    int effectiveCenter = 128 + LSX_BIAS_CORRECTION;
    effectiveCenter = constrain(effectiveCenter, 0, 255); 

    // Steering factor relative to the corrected center (-128 to 127)
    int steeringFactor = lsx - effectiveCenter; 
    int steerMagnitude = abs(steeringFactor);
    
    // --- 4.2. Apply Dead Zones ---

    int throttleSpeed = abs(baseSpeed) < DRIVE_DEAD_ZONE ? 0 : baseSpeed;

    if (steerMagnitude < STEERING_DEAD_ZONE) {
        steeringFactor = 0;
        steerMagnitude = 0;
    }

    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;
    
    // --- 4.3. Control Logic ---

    if (throttleSpeed != 0) {
        // MODE 1: Differential Steering (Moving + Turning - Aggressive Curve)
        
        float maxSteerRange = 128.0; 
        float steerRatio = (float)steerMagnitude / maxSteerRange;
        steerRatio = constrain(steerRatio, 0.0, 1.0); 

        int dominantSpeed = throttleSpeed;
        int submissiveSpeed = throttleSpeed * (1.0 - steerRatio); 
        submissiveSpeed = constrain(submissiveSpeed, -abs(dominantSpeed), abs(dominantSpeed));


        if (steeringFactor > 0) { // Turning Right
            leftMotorSpeed = dominantSpeed;
            rightMotorSpeed = submissiveSpeed;
        } else { // Turning Left
            leftMotorSpeed = submissiveSpeed;
            rightMotorSpeed = dominantSpeed;
        }

    } else if (steerMagnitude != 0) {
        // MODE 2: Spin Turning (Stopped + Turning on itself)
        
        int maxSpin;
        
        // Check if Triangle is pressed to switch to the alternate spin mode
        if (ps5.Triangle()) {
            // ALT Spin Mode: Slower and more precise for fine tuning
            maxSpin = MAX_ALT_SPIN_SPEED;
            Serial.println("--- ALT Spin Mode Active ---");
        } else {
            // Default Spin Mode: Fast spin
            maxSpin = MAX_SPIN_SPEED;
        }

        int spinSpeed = map(steerMagnitude, STEERING_DEAD_ZONE, 128, 0, maxSpin);
        spinSpeed = constrain(spinSpeed, 0, maxSpin);
        
        if (steeringFactor > 0) { // Spin Right (LSX Right)
            leftMotorSpeed = spinSpeed;
            rightMotorSpeed = -spinSpeed;
        } else { // Spin Left (LSX Left)
            leftMotorSpeed = -spinSpeed;
            rightMotorSpeed = spinSpeed;
        }

    } else {
        stopAllMotors();
        return; 
    }
    
    // --- 4.4. Final Command (Apply M1's Inverted Wiring Fix) ---
    
    // M1 (Left) is physically wired backward, so we MUST invert its speed 
    // to match the calculated direction.
    setMotorSpeed(M1_PWM_CHANNEL, -leftMotorSpeed); 
    setMotorSpeed(M2_PWM_CHANNEL, rightMotorSpeed);
}

// ===========================================
// --- 5. MAIN PROGRAM FLOW ---
// ===========================================

void setup() {
    Serial.begin(115200);
    Serial.println("--- PS5 Full Drive and Servo Control Starting ---");
    
    setupMotors();
    setupServos(); // Initialize Servos

    // Initialize Bluetooth. !!! REMEMBER TO CHANGE THE MAC ADDRESS !!!
    ps5.begin(ps5MAC); 
    
    Serial.println("Waiting for PS5 Controller connection...");
}

void loop() {
    if (ps5.isConnected()) {
        controlMotors();
        controlServos(); // Control Servos
    } else {
        // Safety: Stop all movement if the controller disconnects
        stopAllMotors();
    }
    
    delay(20); // 50 Hz loop rate
}