#include "/home/joseph/Desktop/Robot/NRC/nrc-combat-2020/Fin_NRC_BB_code/BB_IMU.cpp" //?

bb_imu IMU_test;

void setup() {

  Serial.begin(9600);

  while(!Serial) {}
  
  Serial.println("Start");

  
  if(!IMU_test.init())
    while(true)
    {
      Serial.println("IMU setup failed");
      delay(500);
    }

  Serial.println("IMU setup fin");
}

void loop() {
    
  while(true)
  {

    IMU_test.update();

    Serial.print("Rotaion: ");    
    Serial.println(IMU_test.Get_val());
    Serial.print("Orientation: ");
    if(IMU_test.Get_upright())
    {
      Serial.println("Up");
    } 
    else   
    {
      Serial.println("Down");
    }
    
    delay(1);
  }
  
}
