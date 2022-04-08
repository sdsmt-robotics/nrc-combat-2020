#include "driveFuncs.h"
#include "utility.h"

/**
 * Constructor for the class.
 */
DriveManager::DriveManager() : filter(100, 100, 0.08) {}


/**
 * Estimate the bot's angular velocity.
 * 
 * @param encoders - Array of encoders on the bot.
 * @param numEncoders - Number of encoders
 * @return The estimated bot velocity
 */
float DriveManager::estimateBotVelocity(float mtrVels[NUM_MTRS], bool encGood[NUM_MTRS]) {
    float sum = 0;
    unsigned numSummed = 0;
    for (int i = 0; i < NUM_MTRS; i++) {
        if (encGood[i]) {
            sum += mtrVels[i];
            numSummed++;
        }
    }

    // Invert wheel velocity if spinning in this mode (upright)
    if (spinnigCW) {
        sum *= -1;
    }

    return (numSummed == 0 ? 0 : sum * WHEEL_RADIUS / float(numSummed * BOT_RADIUS));
}

/**
 * Update orientation estimation.
 * 
 * @param imuVelocity - Angular velocity from IMU. In rad/s.
 * @param encVelocities - Array of velocity values from encoders. In rpm.
 * @return Angle estimate.
 */
float DriveManager::updateOrientation(float imuVelocity, float encVelocities[3]) {
    unsigned long lastUpdate = micros();
    static float lastAngle = curAngle;

    // Time delta for integrating
    unsigned curTime = micros();
    float deltaT = (curTime - lastUpdate) * 0.000001;
    lastUpdate = curTime;

    // Remove bad encoder values. Assume good if nonzero or if all are zero.
    bool encGood[3] = {0};
    bool allZero = true;
    for (int i = 0; i < 3; i++) {
        bool encZero = isZero(encVelocities[i]); 
        encGood[i] = !encZero;
        allZero = allZero && encZero;
    }
    // If all are zero, assume all are good
    if (allZero) {
        memset(encGood, 1, sizeof(bool)*3);
    }

    // Estimate velocity using encoder values
    float encVelocity = estimateBotVelocity(encVelocities, encGood) * RPM_TO_RAD_SEC;

    // Invert IMU velocity if needed (it is upside-down)
    if (spinnigCW) {
        imuVelocity *= -1;
    }

    // Fuse velocity values together in a semi-intelligent manner
    // TODO: will have to re-think this if adding accel gyros
    if (encVelocity < 500) {
        // Low speed mode
        velocityEst = 0.25 * encVelocity + 0.75 * imuVelocity;
    } else {
        // High speed mode
        velocityEst = encVelocity;
    }

    // Filter
    velocityEst = filter.updateEstimate(velocityEst) * driftComp;

    // Update angle estimate. 
    integrate(velocityEst, deltaT);

    // Heading estimation
    if (translateStarted) {
        // Check if we have crossed a boundary (sign change)
        if ((curAngle < 0) != (lastAngle < 0)) {
            // Start udpating heading if we have done at least two cycles with translation
            if (translateCycleCount >=2) {
                // Update heading and translation offset
                heading = normalizeAngle(maxSpeedAngle + (maxMtrSpeed > 0 ? HALF_PI : -HALF_PI));
                translateOffsetAngle += angleTo(commandDir, heading, !spinnigCW) * 0.5;
            } else {
                translateCycleCount++;
            }
            maxMtrSpeed = 0;
            maxSpeedAngle = 0;
        }

        // Track the max speed
        for (int i = 0; i < 3; i++) {
            if (encGood[i] && abs(encVelocities[i]) > abs(maxMtrSpeed)) {
                maxMtrSpeed = encVelocities[i];
                maxSpeedAngle = curAngle + MTR_OFFSETS[i];
            }
        }
    }

    return curAngle;
}

void DriveManager::setInverted(bool inverted) {
    spinnigCW = !inverted;
}

bool DriveManager::isInverted() {
    return !spinnigCW;
}

/**
   @brief Updates drive motor powers with control and angle inputs.
   @param power array of power values for motors.
   @param spin Spin speed modifier. -1.0 to 1.0
   @param x X direction movement modifier. -1.0 to 1.0
   @param y Y direction movement modifier. -1.0 to 1.0
   @param angle Current angle of robot in radians
*/
void DriveManager::calcDrivePower(int power[3], float spin, float x, float y) {
    // TODO: Set direction of spinning based on spinning dir var
    spinnigCW = spin > 0;

    // Handle translation offset updating
    if (!(abs(x) > 0.001 || abs(y) > 0.001)) { // Not translating
        translateStarted = false;
    } else if (!translateStarted) { // Translation starting
        translateCycleCount = 0;
        translateOffsetAngle = (spinnigCW ? 1 : -1) * HALF_PI * spin;  // Add an initial guess for drift compensation
        translateStarted = true;
    }

    // constrain inputs to prevent problems with math
    spin = constrain(spin, -1.0, 1.0);
    x = constrain(x, -1.0, 1.0);
    y = constrain(y, -1.0, 1.0);

    float spin_power = 0;
    float trans_power = TRANS_MAX_POWER * sqrtf((x * x) + (y * y));
    commandDir = atan2f(y, x) - HALF_PI; // Rotate because forward on controller is (x=0, y=1).
    
    float comp_angle = commandDir - curAngle + translateOffsetAngle;

    if (spin > 0) {
        // robot upright
        spin_power = SPIN_MIN_POWER + ((SPIN_MAX_POWER - SPIN_MIN_POWER) * spin);

        power[0] = (spin_power + (trans_power * sinf(MTR_OFFSETS[0] - comp_angle)));
        power[1] = (spin_power + (trans_power * sinf(MTR_OFFSETS[1] - comp_angle)));
        power[2] = (spin_power + (trans_power * sinf(MTR_OFFSETS[2] - comp_angle)));

    } else if (spin < 0) {
        // robot fliped over
        spin_power = -SPIN_MIN_POWER + ((SPIN_MAX_POWER - SPIN_MIN_POWER) * spin);

        power[0] = (spin_power + (trans_power * sinf(MTR_OFFSETS[0] + comp_angle)));
        power[1] = (spin_power + (trans_power * sinf(MTR_OFFSETS[1] + comp_angle)));
        power[2] = (spin_power + (trans_power * sinf(MTR_OFFSETS[2] + comp_angle)));
    }
}

/**
 * Integrate the velocity value over the given time period to update the angle estimate.
 * 
 * @param deltaT - Time that has elapsed since the last sample
 */
void DriveManager::integrate(float velocity, float deltaT) {
    static float lastVal = velocity;
    
    // Trapezoidal approximation
    curAngle = normalizeAngle(curAngle + (velocity + lastVal) / 2 * deltaT);

    lastVal = velocity;
}

/**
 * Get the current bot angle estimate.
 * 
 * @return the current angle estimate.
 */
float DriveManager::getCurAngle() {
    return curAngle;
}

/**
 * Get the current bot heading estimate. Only accurate if translating.
 * 
 * @return the current heading estimate.
 */
float DriveManager::getCurHeading() {
    return heading;
}

/**
 * Rotate "forward" direction by the given delta.
 * 
 * @param delta - Amount by which to rotate in rads.
 */
void DriveManager::rotate(float delta) {
    if (!isZero(delta)) {
        curAngle += delta;
        translateCycleCount = 0; // Reset the heading calculation cause it will be all kinds of screwed up
    }
}

/**
 * Adjust the drift compensator multiplier. The multiplier is 1.0 by default.
 * 
 * @param delta - Amount to chage by.
 */
void DriveManager::adjustDrift(float delta) {
    if (!isZero(delta)) {
        driftComp += delta;
    }
}