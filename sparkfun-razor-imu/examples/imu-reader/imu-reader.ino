// 1/7/2020
// Samuel Ryckman
//
// Example of reading values from the imu.
//
// Connect the imu to the 3.3v, gnd, and RX1 pins of an 
// Arduino and run this sketch.

#define REQUEST_DATA 5  //Request data (Enquiry control character)

void setup() {
  Serial.begin(9600);
  Serial1.begin(38400);
}

void loop() {
  static float val = 0.0;

  // Send the request for data
  Serial1.write(REQUEST_DATA);

  // Wait for a response
  while (Serial1.available() < sizeof(float)) {}

  //Read the data
  if (Serial1.available() >= sizeof(float)) {
    // Read the value from serial
    Serial1.readBytes((char*)(&val), sizeof(float));

    // say what you got:
    Serial.println(val);
  }

  delay(20);
}
