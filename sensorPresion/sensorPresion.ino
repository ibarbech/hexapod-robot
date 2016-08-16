
  int sensorPin0 = 0;    // select the input pin for the potentiometer
  int sensorPin1 = 1;    // select the input pin for the potentiometer
  int sensorPin2 = 2;    // select the input pin for the potentiometer
  int sensorPin3 = 3;    // select the input pin for the potentiometer
  int sensorPin4 = 4;    // select the input pin for the potentiometer
  int sensorPin5 = 5;    // select the input pin for the potentiometer

  int sensorValue0 = 0;  // variable to store the value coming from the sensor
  int sensorValue1 = 0;  // variable to store the value coming from the sensor
  int sensorValue2 = 0;  // variable to store the value coming from the sensor
  int sensorValue3 = 0;  // variable to store the value coming from the sensor
  int sensorValue4 = 0;  // variable to store the value coming from the sensor
  int sensorValue5 = 0;  // variable to store the value coming from the sensor

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  sensorValue0 = analogRead(sensorPin0);
  sensorValue1 = analogRead(sensorPin1);
  sensorValue2 = analogRead(sensorPin2);
  sensorValue3 = analogRead(sensorPin3);
  sensorValue4 = analogRead(sensorPin4);
  sensorValue5 = analogRead(sensorPin5);
  
  String stringSensor0 = "p1 " + String(sensorValue0);
  String stringSensor1 = " p2 " + String(sensorValue1);
  String stringSensor2 = " p3 " + String(sensorValue2);
  String stringSensor3 = " p4 " + String(sensorValue3);
  String stringSensor4 = " p5 " + String(sensorValue4);
  String stringSensor5 = " p6 " + String(sensorValue5);

  Serial.println(stringSensor0 + stringSensor1 + stringSensor2 + stringSensor3 + stringSensor4 + stringSensor5);
  delay(100);
}
