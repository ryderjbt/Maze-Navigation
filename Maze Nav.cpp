#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>
#include <math.h>  //include math library for use with MPU6050
#include <MPU6050_tockn.h>  //include MPU6050 library
#define I2C_SLAVE_ADDR  0x04 //Define the I2C address of the Arduino Nano


// Define OLED display pins
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDR 0x3C // OLED display address
// Define the OLED display object
Adafruit_SSD1306 display(128, 64, &Wire, OLED_SDA, OLED_SCL);
MPU6050 mpu6050(Wire);

int leftMotor_speed, rightMotor_speed;  //variables to store the motor speeds and servo angle
int servoAngle = 10;
int x = 0;  //variable to store data for transmission

// Define the keypad pins and keys
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'3', '6', '9'},
  {'2', '5', '8'},
  {'1', '4', '7'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {2, 0, 4, 16};
byte colPins[COLS] = {18, 5, 17};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Define variables for storing the key sequence
char keySequence[50];
int sequenceLength = 0;

//Encoder
int leftEncoderCount;
int rightEncoderCount;
int distanceL;
double prevDistanceL = 0;  
int distanceR;
double prevDistanceR = 0;
//Set constants for pi, diameter of wheel (D) and number of encoders counts of revolution (N)
const float pi = 3.14;
const float D = 5.9;
const float N = 25;
int angle = 0;
float angle_change = 0;


void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize the OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Display initial message
  display.setCursor(0, 30);
  display.println("Enter command sequence");
  display.println("Press 5 to execute");
  display.display();  // Update the display with the new content  
  // Initialize I2C communication
  Wire.begin();
  mpu6050.begin();// This line initializes the MPU-6050 using the begin function from the MPU6050 library
  mpu6050.calcGyroOffsets(true);// This line uses the calcGyroOffsets function to calibrate the gyroscope in the MPU-6050.
 
}


void loop() {

  Wire.requestFrom(0x04, 2);  // Request 2 bytes of data from the Arduino Nano
  if (Wire.available() >= 2) {
    leftEncoderCount = Wire.read();
    rightEncoderCount = Wire.read();
  }
  //Serial.print("Left Encoder distance: ");
  //Serial.println(distanceL);
  //Serial.print("Right Encoder distance: ");
 // Serial.println(distanceR);


  distanceL = leftEncoderCount * ((D*pi)/N);
  distanceR = rightEncoderCount * ((D*pi)/N);

 


  // Read the keypad input
  char key = keypad.getKey();

//Debug
  if (key != NO_KEY) {
    // Display the key on the OLED display
    display.setCursor(0, 10);
    display.println("Key pressed: " + String(key));
    display.display();
  if (key) {
    Serial.println(key);
  }
}
    if (key == '2') {
      keySequence[sequenceLength] = 'F';
      sequenceLength++;
    }
    else if (key == '4') {
      keySequence[sequenceLength] = 'L';
      sequenceLength++;
    }
    else if (key == '6') {
      keySequence[sequenceLength] = 'R';
      sequenceLength++;
    }
    else if (key == '8') {
      keySequence[sequenceLength] = 'B';
      sequenceLength++;
    }
    else if (key == '9') {
      keySequence[sequenceLength] = 'S';
      sequenceLength++;
    }
    else if (key == '5') {
      // Execute the key sequence and display a message on the OLED display
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Executing command");
      display.display();
     
      for (int i = 0; i < sequenceLength; i++) {
        char command = keySequence[i];
        switch (command) {
          case 'F':
          // Read the keypad input
          char key = keypad.getKey();
          //Debug
          if (key != NO_KEY) {
            // Display the key on the OLED display
            display.setCursor(0, 10);
            display.println("Key pressed: " + String(key));
            display.display();
          if (key) {
            Serial.println(key);
            }
          }
          int inputDistance = (int(key) * 10)        
          goForward();
          measure_distance("forward", inputDistance);
          stop();
          break;
          case 'L':
          goLeft();
          turn("Left");
          delay(10);
          stop();
          break;
          case 'R':
          goRight();
          turn("Right");
          delay(10);
          stop();
          break;
          case 'B':
          // Read the keypad input
          char key = keypad.getKey();
          //Debug
          if (key != NO_KEY) {
            // Display the key on the OLED display
            display.setCursor(0, 10);
            display.println("Key pressed: " + String(key));
            display.display();
          if (key) {
            Serial.println(key);
            }
          }
          int inputDistance = (int(key) * 10)            
          goBackward();            
          measure_distance("backward", inputDistance);
          stop();
          break;
          case 'S':
          stop();
          break;
          }
      }
    }
    // Reset the key sequence
    sequenceLength = 0;
    display.setCursor(0, 20);
    display.println("Command executed");
           
    else if (key == '7') {
    // Clear the key sequence and display a message on the OLED display
      sequenceLength = 0;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("sequence cleared");
    }

      // Define motor control functions
      void goForward() {
        // Move the robot forward
        delay(1000);
        leftMotor_speed = 150;
        rightMotor_speed = 150;
        servoAngle = 90;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
        }

      void goLeft() {
        // Code to turn the robot left
        delay(1000);
        leftMotor_speed = 100;
        rightMotor_speed = 250;
        servoAngle = -20;
        Serial.println("Going Left!");
       Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
      }


      void goRight() {
        // Code to turn the robot right
        delay(1000);
        leftMotor_speed = 250;
        rightMotor_speed = 100;
        servoAngle = 130;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
        Serial.println("Going Right!");
    }

      void goBackward() {
        // Code to move the robot backward
        delay(1000);
        leftMotor_speed = -150;
        rightMotor_speed = -150;
        servoAngle = 88;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
        Serial.println("Going Backwards!");
      }

      void stop() {
        // Code to move the robot backward
        delay(1000);
        leftMotor_speed = 0;
        rightMotor_speed = 0;
        servoAngle = 88;
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);  //transmit data to arduino
        Serial.println("Stop!");
      }

  void measure_distance(char *direction, int distance)
  {
   
    long enc1_count, enc2_count;
    long enc1_count_new, enc2_count_new;
    int enc_change = 0;

    Wire.requestFrom(I2C_SLAVE_ADDR,2);
    if (Wire.available () >= 2)
    {
      enc1_count = Wire.read();
      enc2_count = Wire.read();
    }
   
    if (direction == "backward");
    {
      while (enc_change < distance)
      {
        Wire.requestFrom(I2C_SLAVE_ADDR,2);
        if (Wire.available () >= 2)
        {
          enc1_count_new = Wire.read();
          enc2_count_new = Wire.read();
          if (enc1_count <= enc1_count_new)
          {                            
            enc_change = (enc1_count_new - enc1_count);
            enc_change = enc_change * ((D*pi)/N);
          }
          else if (enc1_count > enc1_count_new)
          {                                  
            enc_change = ((enc1_count_new + 254) - enc1_count);
            enc_change = enc_change * ((D*pi)/N);
          }    
          if (enc_change > (distance + 10))
          {
            enc_change = 0;
            enc1_count = enc1_count_new;
          }
          Serial.print("\enc1 : ");
          Serial.println(enc1_count_new);
          Serial.print("\enc1 og : ");
          Serial.println(enc1_count);
          Serial.print("\enc1 change : ");
          Serial.println(enc_change);
        }
      }
    }
    if (direction == "forward");
    {
      while (enc_change < distance)
      {
        Wire.requestFrom(I2C_SLAVE_ADDR,2);
        if (Wire.available () >= 2)
        {
          enc1_count_new = Wire.read();
          enc2_count_new = Wire.read();
          if (enc1_count >= enc1_count_new)
          {                            
            enc_change = (enc1_count - enc1_count_new);
            enc_change = enc_change * ((D*pi)/N);
          }
          else if (enc1_count < enc1_count_new)
          {                                  
            enc_change = ((enc1_count + 254) - enc1_count_new);
            enc_change = enc_change * ((D*pi)/N);
          }    
          if (enc_change < (distance - 10)) // done
          {
            enc_change = 0;
            enc1_count = enc1_count_new;
          }
          Serial.print("\enc1 : ");
          Serial.println(enc1_count_new);
          Serial.print("\enc1 og : ");
          Serial.println(enc1_count);
          Serial.print("\enc1 change : ");
          Serial.println(enc_change);
        }
      }
    }
  }

  void turn(char *turn_type){
    int angle = 0;
    float angle_change = 0;

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    if (turn_type == "Left")
    {                                  
      goLeft();
    }
    else if (turn_type == "Right")
    {
      goRight();  
    }
    mpu6050.begin();
    angle = (mpu6050.getAngleZ());
    while (angle_change < 15)
    {
      mpu6050.update();
      Serial.print("\tangleZ : ");
      Serial.println(mpu6050.getAngleZ());
      if (turn_type == "Left")
      {
        angle_change = ((mpu6050.getAngleZ()) - angle);
      }
      else if (turn_type == "Right")
      {
        angle_change = (angle - (mpu6050.getAngleZ()));
      }
    }
  }

     
        void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle){
        Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
        Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
        Wire.write((byte)(leftMotor_speed & 0x000000FF));           //leftMotor_speed, containing the 8 LSB - bits 8 to 1
        Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of rightMotor_speed, containing bits 16 to 9
        Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1
        Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         // first byte of servoAngle, containing bits 16 to 9
        Wire.write((byte)(servoAngle & 0x000000FF)); // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
        Wire.endTransmission();   // stop transmitting
      }
