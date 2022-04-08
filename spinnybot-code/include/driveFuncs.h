/**
 * Functions used for driving the BB.
 */

#ifndef __BB_DRIVE__
#define __BB_DRIVE__

#include <SimpleKalmanFilter.h>
#include "constants.h"

class DriveManager {
   public:
    DriveManager();
    float updateOrientation(float imuVel, float mtrVel[3]);
    void calcDrivePower(int power[3], float spin, float x, float y);
    float getCurHeading();
    float getCurAngle();
    void rotate(float delta);
    void adjustDrift(float delta);
    void setInverted(bool inverted);
    bool isInverted();

   private:
    const int SPIN_MAX_POWER = 300;  // max motor power when enabled
    const int SPIN_MIN_POWER = 100;  // min motor power when enabled

    // max movement power. this is the magnitude of max power when moving
    const int TRANS_MAX_POWER = 50;

    float commandDir = 0.0; // Angle we were told to drive
    float velocityEst = 0.0;  // Current estimated angular velocity
    float curAngle = 0.0; // Current estimated bot angle
    float translateOffsetAngle = 0.0;  // offset angle for translation
    float driftComp = 1.0;
    
    // Heading tracking variables
    bool translateStarted = false; // Track whether we are doing a translation motion
    long translateCycleCount = 0;   // Count of the cycles since translation start
    float heading = 0.0; // Current heading ()
    float maxMtrSpeed = 0.0;  // Max motor speed encountered during a cycle
    float maxSpeedAngle = 0.0; // Angle of when the max speed occurred
    bool spinnigCW = true;

    SimpleKalmanFilter filter;

    void integrate(float velocity, float deltaT);
    float estimateBotVelocity(float mtrVels[NUM_MTRS], bool encGood[NUM_MTRS]);
};

#endif