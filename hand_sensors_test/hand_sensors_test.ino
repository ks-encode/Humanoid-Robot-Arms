#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define DEBUG_MOTORS 0

class RoboticHand {
  public:
      RoboticHand(int chip_select[4], int hand_analog_out[4]);
      void set_binary_output(int number);
      void read_analog_data(int data[2]);
      void read_sensors(int pin,int analog_data[2]);
      void read_all_sensors(int analog_data[16][2]);

  private:
      int chip_select[4];  // Pins for the binary output
      int hand_analog_out[4];  // Pins for the analog input
      int motor_pwm_pins[5][3]; //TODO replace this all with a motor struct
      Adafruit_PWMServoDriver motor_boards[5][3];
};

RoboticHand::RoboticHand(int chip_select[4], int hand_analog_out[2]) {
    // Initialize pin modes for binary output and analog input pins
    for (int i = 0; i < 4; i++) {
        this->chip_select[i] = chip_select[i];
        pinMode(chip_select[i], OUTPUT);
    }
    for (int i = 0; i < 2; i++) {
        this->hand_analog_out[i] = hand_analog_out[i];
        pinMode(hand_analog_out[i], INPUT);
    }
}

void RoboticHand::set_binary_output(int number) {
    // Ensure the number is within the valid range (0-15)
    if (number < 0 || number > 15) {
      Serial.println("Error sensor select number out of range");
        return;  // Out of range
    }

    // Set each pin to the corresponding bit in the number
    for (int i = 0; i < 4; i++) {
        digitalWrite(chip_select[i], (number >> i) & 0x01);
    }
}

void RoboticHand::read_analog_data(int data[2]) {
    // Read analog values from the analog pins
    for (int i = 0; i < 2; i++) {
        data[i] = analogRead(hand_analog_out[i]);
    }
}

void RoboticHand::read_sensors(int pin,int analog_data[2]) {
    set_binary_output(pin);
    delay(1); // Small delay to allow pins to stabilize
    read_analog_data(analog_data);
}

void RoboticHand::read_all_sensors(int analog_data[16][2]) {
    // Loop through numbers 0 to 15, setting the binary output and reading analog data
    for (int i = 0; i < 16; i++) {
        set_binary_output(i);
        delay(1); // Small delay to allow pins to stabilize
        read_analog_data(analog_data[i]);
    }
}

int chip_select[4] = {30,31,32,33}; // Define your binary output pins
int hand_analog_out[2] = {40,41}; // Define your analog input pins

RoboticHand hand(chip_select, hand_analog_out);

void setup() {
  Serial.begin(9600);
  analogReadResolution(13);
  analogReadAveraging(20); //might have to reduce this averaging amount later to increase response time, result sin aorun 60% less noise at 20 //20 good
}

void loop() {
    int analog_data[16][2];
    int pin_data[2];
    // Read all sensors, iterating through binary outputs 0 to 15
    
    hand.read_all_sensors(analog_data);
    
    // Print the results to the Serial monitor
    for (int i = 0; i < 16; i++) {
        Serial.print("Binary Output ");
        Serial.print(i);
        Serial.print(": Analog Data: ");
        for (int j = 0; j < 2; j++) {
            Serial.print(analog_data[i][j]);
            Serial.print(", ");
        }
        Serial.println();
    }
    /*
    hand.read_sensors(2,pin_data);
    int i;
    for (i = 0; i < 2; i++) {
        Serial.print(pin_data[i]);
        Serial.print(", ");
    }
    Serial.println();
    */
    delay(1);
}