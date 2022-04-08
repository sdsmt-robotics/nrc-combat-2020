/**
 * @brief Tests written for the spinning drive code. Run with "pio test -e test -f test_drivecode".
 */
#include <Arduino.h>
#include "driveFuncs.h"
#include "utility.h"
#include <unity.h>


void printDat(uint8_t dat[]) {
    Serial.print("{");
    for (int i = 0; i < 8; i++) {
        Serial.print(dat[i], HEX);
        Serial.print(", ");
    }
    Serial.println("}");
}

/**
 * @brief Test some basic power commands with the SPARK.
 */
void test_utilities(void) {
    //-----isZero func------------------
    TEST_ASSERT_TRUE(isZero(0));
    TEST_ASSERT_TRUE(isZero(0.00001));
    TEST_ASSERT_TRUE(isZero(-0.00001));
    TEST_ASSERT_FALSE(isZero(0.001));
    TEST_ASSERT_FALSE(isZero(-0.001));


    //-----angleTo func------------------
    // Equal angles
    TEST_ASSERT_EQUAL_FLOAT(0, angleTo(0, 0, true));
    TEST_ASSERT_EQUAL_FLOAT(0, angleTo(30*DEG_TO_RAD, 30*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(0, angleTo(-30*DEG_TO_RAD, -30*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(0, angleTo(0, 0, false));
    TEST_ASSERT_EQUAL_FLOAT(0, angleTo(30*DEG_TO_RAD, 30*DEG_TO_RAD, false));
    TEST_ASSERT_EQUAL_FLOAT(0, angleTo(-30*DEG_TO_RAD, -30*DEG_TO_RAD, false));

    // Same region angles
    TEST_ASSERT_EQUAL_FLOAT(20*DEG_TO_RAD, angleTo(30*DEG_TO_RAD, 50*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(340*DEG_TO_RAD, angleTo(50*DEG_TO_RAD, 30*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(-340*DEG_TO_RAD, angleTo(30*DEG_TO_RAD, 50*DEG_TO_RAD, false));
    TEST_ASSERT_EQUAL_FLOAT(-20*DEG_TO_RAD, angleTo(50*DEG_TO_RAD, 30*DEG_TO_RAD, false));

    TEST_ASSERT_EQUAL_FLOAT(340*DEG_TO_RAD, angleTo(-30*DEG_TO_RAD, -50*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(20*DEG_TO_RAD, angleTo(-50*DEG_TO_RAD, -30*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(-20*DEG_TO_RAD, angleTo(-30*DEG_TO_RAD, -50*DEG_TO_RAD, false));
    TEST_ASSERT_EQUAL_FLOAT(-340*DEG_TO_RAD, angleTo(-50*DEG_TO_RAD, -30*DEG_TO_RAD, false));

    // Angles crossing 180 deg
    TEST_ASSERT_EQUAL_FLOAT(20*DEG_TO_RAD, angleTo(170*DEG_TO_RAD, -170*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(340*DEG_TO_RAD, angleTo(-170*DEG_TO_RAD, 170*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(-340*DEG_TO_RAD, angleTo(170*DEG_TO_RAD, -170*DEG_TO_RAD, false));
    TEST_ASSERT_EQUAL_FLOAT(-20*DEG_TO_RAD, angleTo(-170*DEG_TO_RAD, 170*DEG_TO_RAD, false));

    // Angles crossing 0 deg
    TEST_ASSERT_EQUAL_FLOAT(340*DEG_TO_RAD, angleTo(10*DEG_TO_RAD, -10*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(20*DEG_TO_RAD, angleTo(-10*DEG_TO_RAD, 10*DEG_TO_RAD, true));
    TEST_ASSERT_EQUAL_FLOAT(-20*DEG_TO_RAD, angleTo(10*DEG_TO_RAD, -10*DEG_TO_RAD, false));
    TEST_ASSERT_EQUAL_FLOAT(-340*DEG_TO_RAD, angleTo(-10*DEG_TO_RAD, 10*DEG_TO_RAD, false));

    // Angles 
}

/**
 * @brief Test some basic power commands with the SPARK.
 */
void test_drive_funcs(void) {
    DriveManager driver;

    // Test some driving outputs

    // Test some angle tracking

    // Test angle tracking with missing encoder values


}

void setup() {
    delay(1000);

    DriveManager driver;

    UNITY_BEGIN();
    RUN_TEST(test_utilities);
    RUN_TEST(test_drive_funcs);
    UNITY_END();
}

void loop() {
    delay(500);
}