//pins
int pwm_pin = 11;
int dir_pin = 7;
int brake_pin = 8;
int fs_pin = 2;

//Speed Control
int rpm = 0;
long fs_time = 0;


void setup()
{
//Setup serial
Serial.begin(115200);

//Setup Pins
pinMode(pwm_pin, OUTPUT);
pinMode(dir_pin, OUTPUT);
pinMode(brake_pin, OUTPUT);
pinMode(fs_pin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(fs_pin), counter, RISING);
}

void loop()
{
digitalWrite(dir_pin, LOW);
power(100);
Serial.println(rpm);
delay(500);
}

void counter()
{
if(fs_time == 0){
fs_time = micros();
}
else{
rpm = (1/6.0)/((micros()-fs_time)/60000000.0);
fs_time = micros();
}
}

void brake()
{
digitalWrite(brake_pin, LOW);
}

void power(int mp)
{
digitalWrite(brake_pin, HIGH);
analogWrite(pwm_pin, 255-mp);
}
