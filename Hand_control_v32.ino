//uses V5 of dt code
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h> //standard pid library
#include <vector>
#include <string>
#include "hiwonder_func.h"
#include <PWMServo.h> //copied working one to local direcotry to avoid confusion over multiple libraries with same name, except this didnt owrk
//#define DEBUG_MOTORS
//#define DEBUG_JOINTS
//#define DEBUG_BIG_SERVOS
#define DEBUG_TIME
//#define DEBUG_HIWONDER
#define SHOW_HIWONDER_TEMPS
#define DEBUG_RECIEVE_DATA
//#define DEBUG_FORCE_CONT
//#define DEBUG_FINGERS
#define PWMOSCILLATORF 27000000
#define PWMF 1600
#define RETURN_ANGLE_PRECISION 1
#define HISERIAL Serial8
#define HI_TXEN 11
#define HI_RXEN 12

#define MAIN_POWER_SENSE_PIN 20
#define MAIN_POWER_ON_THRESH_V 10 
//TODO CHANGE BIG SERVO TO BE WHERE IT SHOULD BE NOT ADD 1 EACH time

//Safety limits
#define HIWONDER_MAX_TEMP 60 //deg c only works between 50-100 (motor limits not code limits)

#define JOINT_BUFF_LEN 10
#define INFO_PRINT_INTERVAL_MS 100

#define BASE_TENDON_COMBINATION_MOVE_DEG 40
#define PIP_DONT_MOVE_TENDON_RANGE 5
#define PIP_TENDON_DEG_LINEAR_MOVE 20

#define SMOOTHING_APLHA 0.18
#define MAX_ANGLE_VAR 30
#define MAX_ANGLE_VAR_UP 500

String motor_test_values = "";


int SERIAL_CMD_MOV_ENABLED = 1;

// Adafruit_PWMServoDriver initialization
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x60, Wire);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x62, Wire);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x61, Wire);
unsigned long startTime = millis();
unsigned long last_print_time = 0;
unsigned long start_off_loop_time = 0;
int state = 0;
String  logging_msg = "";


void get_string_from_doubles(const double double_array[], size_t size, char* result, size_t result_size) {
    result[0] = '\0';  // Initialize result string with a null terminator to start the concatenation
    char temp[20];     // Temporary buffer for each double

    for (size_t i = 0; i < size; i++) {
        // Convert double to string with a fixed precision and store it in the temp buffer
        dtostrf(double_array[i], 1, RETURN_ANGLE_PRECISION, temp);

        // Check if there's enough space in the result buffer before appending
        if (strlen(result) + strlen(temp) + 2 < result_size) {
            strcat(result, temp);  // Append the converted string to the result string

            if (i < size - 1) {
                strcat(result, ",");  // Add a comma if not the last element
            }
        } else {
            break;  // If there's not enough space, stop appending to avoid overflow
        }
    }
}

class DCMotor {
public:
    DCMotor(Adafruit_PWMServoDriver* pwm_board, int motor_A_pin, int motor_B_pin);
    void set_motor_power(int power);
private:
    Adafruit_PWMServoDriver* pwm;
    int motor_A_pin;
    int motor_B_pin;
};

// Constructor implementation
DCMotor::DCMotor(Adafruit_PWMServoDriver* pwm_board, int motor_A_pin, int motor_B_pin)
    : pwm(pwm_board), motor_A_pin(motor_A_pin), motor_B_pin(motor_B_pin) {}

// Function to set motor power
void DCMotor::set_motor_power(int power) {
  if (pwm == nullptr) {
      Serial.println("Error: PWM object is null.");
      return;
  }
    // Set power limits
    if (power > 4095) {
        power = 4095;
        #ifdef DEBUG_MOTORS
        Serial.print("power: ");
        Serial.print(power);
        Serial.print(" (out of range)");
        #endif
    } else if (power < -4095) {
        power = -4095;
        #ifdef DEBUG_MOTORS
        Serial.print("power: ");
        Serial.print(power);
        Serial.print(" (out of range)");
        #endif
    }else{
      #ifdef DEBUG_MOTORS
      Serial.print("power: ");
      Serial.print(power);
      #endif
      //Serial.print("power: ");
      //Serial.println(power);
    }

    // Set PWM signals based on power value
    if (power >= 0) { //"forwards"
        pwm->setPWM(motor_A_pin, 0, abs(power));
        pwm->setPWM(motor_B_pin, 0, 0);
    } else { //"backwards"
        pwm->setPWM(motor_A_pin, 0, 0);
        pwm->setPWM(motor_B_pin, 0, abs(power));
    }
}

double get_interpolation(int raw_hall, double angles[], double readings[]) {
    //via linear interpolation
    double final_angle = 999;
    int i;
    for(i = 0; raw_hall > readings[i]; i++) {
      //Serial.println(readings[i]);
    } //find where it is in the readings
    double gradient = 0;
    if(i == 0) { //smaller reading than measured
        int higher_reading = readings[0];
        gradient = (angles[1] - angles[0]) / (readings[1] - readings[0]);
        final_angle = angles[0] + gradient * (raw_hall - higher_reading);
    } else if(readings[i] == 99999) { //end of the array therefore reading is greater than max measured
        int higher_reading = readings[i - 1];
        int lower_reading = readings[i - 2];
        gradient = (angles[i - 1] - angles[i - 2]) / (higher_reading - lower_reading);
        final_angle = angles[i - 1] + gradient * (raw_hall - higher_reading);
    } else { //standard interpolation
        int higher_reading = readings[i];
        int lower_reading = readings[i - 1];
        gradient = (angles[i] - angles[i - 1]) / (higher_reading - lower_reading);
        final_angle = angles[i - 1] + gradient * (raw_hall - lower_reading);
    }
    return final_angle;
}

double apply_exp_smooth(double old_val,double new_val,double alpha,int* count_off){
  /*if((new_val-old_val) > MAX_ANGLE_VAR){
    (*count_off)=(*count_off)+1;
    if(*count_off>MAX_ANGLE_VAR_UP){
      return new_val;
      *count_off=0;
    }else{
      return old_val;
    }
  }
  (*count_off)=0;*/
  return alpha * new_val + (1 - alpha) * old_val;
}

class Sensor {
public:
    Sensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2],int sense_enable);
    void set_raw(int raw_value);
    int get_raw() const;
    void update_sensor(); // Normal function for updating the sensor
    void set_binary_output(int address);
    int read_analog_data();
    void set_sense_enable(int on_or_off);
    virtual double get_angle(){return 0;}; // Make it a pure virtual function for polymorphism
    virtual double get_force(){return 0;};
protected:
    int pin;
    int address;
    int raw_value;
    int sense_enabled;
    int* chip_select_pins;
    int* analog_read_pins;
};

Sensor::Sensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2], int sense_enabled)
    : pin(pin), address(address), raw_value(0), sense_enabled(sense_enabled) {
    chip_select_pins = s_chip_select_pins;
    analog_read_pins = s_analog_read_pins;
}

void Sensor::set_sense_enable(int on_or_off){
  this->sense_enabled = on_or_off;
}

void Sensor::set_raw(int raw_value) {
    this->raw_value = raw_value;
}

int Sensor::get_raw() const {
    return raw_value;
}

void Sensor::update_sensor() {\
  if(sense_enabled){
    this->set_binary_output(this->address);
    delayMicroseconds(1);
    this->raw_value = this->read_analog_data();
  }
    //Serial.print("Sensor updated, new value: ");
    //Serial.print(this->raw_value);
}

void Sensor::set_binary_output(int address) {
    // Ensure the number is within the valid range (0-15)
    if (address < 0 || address > 15) {
        Serial.println("Error sensor select number out of range");
        return;  // Out of range
    }

    // Set each pin to the corresponding bit in the number
    for (int i = 0; i < 4; i++) {
        digitalWrite(this->chip_select_pins[i], (address >> i) & 0x01);
    }
}

int Sensor::read_analog_data() {
    return analogRead(analog_read_pins[this->pin]);
}



// FSRSensor class
class FSRSensor : public Sensor {
public:
    FSRSensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2],int sense_enabled, double a, double k, double c);
    double get_force();
    double get_angle() override; // Override the virtual function
private:
    double multiplier;
    double offset;
    double a;
    double k;
    double c;
};

FSRSensor::FSRSensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2],int sense_enabled, double a, double k, double c)
    : Sensor(pin, address, s_chip_select_pins, s_analog_read_pins,sense_enabled), multiplier(1.0), offset(0.0), a(a), k(k), c(c) {}

double FSRSensor::get_force() {
  double force = raw_value*a/(k-raw_value)+c;
  if(force<80){
    force = 0;
  }
  return force;
}

double FSRSensor::get_angle(){
    return 0.0; // FSR doesn't measure angles, return a default value or error
}

// PotSensor class
class PotSensor : public Sensor {
public:
    PotSensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2], double multiplier, double angle_offset,int sense_enabled);
    double get_angle()override; // Override the virtual function

private:
    double multiplier;
    double angle_offset;
};

PotSensor::PotSensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2], double multiplier, double angle_offset,int sense_enabled)
    : Sensor(pin, address, s_chip_select_pins, s_analog_read_pins, sense_enabled), multiplier(multiplier), angle_offset(angle_offset) {}

double PotSensor::get_angle(){
    //Serial.print("raw:");
    //Serial.print(this->raw_value);
    return (this->raw_value) * this->multiplier + this->angle_offset;
}

// HallSensor class
class HallSensor : public Sensor {
public:
    HallSensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2], double angles[], double readings[],int sense_enabled);
    double get_angle() override; // Override the virtual function

private:
    double* angles;
    double* readings;
};

HallSensor::HallSensor(int pin, int address, int s_chip_select_pins[4], int s_analog_read_pins[2], double angles[], double readings[],int sense_enabled)
    : Sensor(pin, address, s_chip_select_pins, s_analog_read_pins, sense_enabled){
    //Copy the 2 arrays to ensure that the data still exists if the original array goes out of scope, (which it will if it is a local vairable)
    this->angles = new double[20];
    this->readings = new double[20];
    for (int i = 0; i < 20; i++) {
        this->angles[i] = angles[i];
        this->readings[i] = readings[i];
    }
}

double HallSensor::get_angle(){
    //Serial.println("Reading hall effect");
    return get_interpolation(this->raw_value, this->angles, this->readings);
    
}

// PotSensor class
class DirectPotSensor{
public:
    DirectPotSensor(int pin, double multiplier, double angle_offset,int sense_enabled);
    double get_angle();
    void update_sensor();
    void set_sense_enable(int on_or_off);
    int sense_enabled;
private:
    double multiplier;
    double angle_offset;
    double raw_value;
    int pin;
};

DirectPotSensor::DirectPotSensor(int pin, double multiplier, double angle_offset, int sense_enabled) : sense_enabled(sense_enabled), multiplier(multiplier), angle_offset(angle_offset), pin(pin){
  raw_value = 0;
}

void DirectPotSensor::set_sense_enable(int on_or_off){
  this->sense_enabled = on_or_off;
}

double DirectPotSensor::get_angle(){
    //Serial.print("raw:");
    //Serial.print(this->raw_value);
    return ((this->raw_value) - this->angle_offset) * this->multiplier;
}

void DirectPotSensor::update_sensor(){
  this->raw_value = analogRead(this->pin);
}

class Joint {
public:
    Joint(int pin, double angle_offset, double min_angle, double max_angle,double angle_mult, DirectPotSensor* angle_sensor);
    double get_joint_angle();
    virtual void set_joint_angle(double angle, int use_time_over_speed, int time_2_move_millis, int speed);
    void update_motor_angle(double angle);
    double get_temp(){
      return this->temp;
    };
    
    virtual int loop();
    virtual void initialise();
    virtual void release();
    virtual int check_for_stall();
    virtual void log_info();
    void averageSpeedBuffAppend();
    void move_enable(int on_or_off);
    //////////////////////////////////////////////////////////////////////////
    int did_log = true;
protected:
    int pin;
    double angle_true = 0;
    double angle_offset;
    double min_angle;
    double max_angle;
    double angle_mult;
    double temp;
    double target_angle;
    double move_time;
    unsigned long int start_move_time;
    unsigned long int est_move_time;
    unsigned long int pos_buffer_update_time_ms;
    unsigned long int last_buffer_update_time_ms;
    unsigned long int stopped_time;
    unsigned long int ms_per_deg = 100000;
    unsigned long int last_command_sent_time;
    int mov_dir;
    int cur_commanded_angle;
    int start_angle;

    double lastXValues[JOINT_BUFF_LEN];
    int buffer_count = 0;
    int buffer_started = 0;
    int move_enabled = 0;
    PWMServo* servo_out;
    DirectPotSensor* angle_sensor;
    virtual void update_temp(){
      return;
    }
    virtual void update_joint_angle(){
      angle_sensor->update_sensor();
      if(angle_sensor->sense_enabled){ //if sensor working then get real angle value
        this->angle_true = angle_sensor->get_angle();
      }else{
        angle_true = cur_commanded_angle; //else assume the real angle value is the last commanded angle
      }
      //Serial.println(angle_sensor->get_angle());
    }
    void loop_slow_move();
};

// Constructor implementation
Joint::Joint(int pin, double angle_offset, double min_angle, double max_angle,double angle_mult,DirectPotSensor* angle_sensor) 
: pin(pin), angle_offset(angle_offset), min_angle(min_angle), max_angle(max_angle), angle_mult(angle_mult), angle_sensor(angle_sensor){
  temp=99999;
  angle_true = 0; //Defualt Angle
  pos_buffer_update_time_ms = 60; //times by JOINT_BUFF_LEN (100) to ms of speed calc
  stopped_time = millis();
  for(int i = 0; i<JOINT_BUFF_LEN;i++){
    lastXValues[i] = angle_true;
  }
  servo_out = new PWMServo(); // Allocate memory for servo_out
}

void Joint::log_info(){

}

void Joint::move_enable(int on_or_off){
  this->move_enabled = on_or_off;
  if(!on_or_off){
    this->release();
  }else{
    this->initialise();
  }
}

void Joint::averageSpeedBuffAppend(){ //append pos to buffer for calulating average speed
  buffer_count++;
  //shift all values in average array up one spot
  for(int i = 1; i<JOINT_BUFF_LEN;i++){
    lastXValues[i-1] = lastXValues[i];
  }
  //set the first value in the array to the input
  lastXValues[JOINT_BUFF_LEN-1] = angle_true;
  //if the array is full
  if(buffer_started==1){
    //get average speed
  }else{ //not enough values to take an average
    if(buffer_count>=JOINT_BUFF_LEN){
      buffer_started=1;
    }
  }
}

void Joint::initialise(){ //MUST BE RUN when enabled else target angles and movements will be all random
  update_joint_angle();
  cur_commanded_angle = (int)angle_true;
  target_angle = cur_commanded_angle;
  servo_out->attach(this->pin);
  Serial.print("Sending angle :");
  Serial.print(cur_commanded_angle);
  update_motor_angle(cur_commanded_angle);
}

void Joint::release(){
  //detach function is broken in library so manually set ouput to zero
  analogWrite(this->pin, 0);
}

double Joint::get_joint_angle() {
    return this->angle_true;
}

void Joint::loop_slow_move(){
  int should_have_moved = start_angle+(mov_dir*(int)((millis()-start_move_time)/ms_per_deg));
  /*logging_msg+="THis";
  logging_msg+=(millis()-start_move_time);
  logging_msg+="ms";
  logging_msg+=ms_per_deg;
  logging_msg+="That";
  logging_msg+=(int)((millis()-start_move_time)/ms_per_deg);
  logging_msg+="should have moved";
  logging_msg+=should_have_moved;
  logging_msg+="commanded angle";
  logging_msg+=cur_commanded_angle;*/

  if((cur_commanded_angle<target_angle && mov_dir == 1) || (cur_commanded_angle>target_angle && mov_dir == -1)){ //if havent reached the postion
    if(should_have_moved!=cur_commanded_angle){
      if((should_have_moved>=target_angle && mov_dir == 1) || (should_have_moved<=target_angle && mov_dir == -1)){ //if next movement would be too far
        should_have_moved = target_angle;
      }
      cur_commanded_angle=should_have_moved; //move to new position based on time that has passed
      update_motor_angle(cur_commanded_angle); 
    }
  }
  //last_command_sent_time = millis();
}

void Joint::set_joint_angle(double angle, int use_time_over_speed, int time_2_move_millis, int speed) {
  did_log = false;
  est_move_time = 4000;
  target_angle = constrain(angle, min_angle, max_angle); //limit to angle limits for the servo;
  #ifdef DEBUG_BIG_SERVOS
  //Serial.println("HERE: ");
  #endif
  start_move_time = millis();
  if(use_time_over_speed){
    ms_per_deg = time_2_move_millis/abs(angle_true-target_angle);
    if(ms_per_deg < 5){
      cur_commanded_angle = target_angle;
    }else{
      cur_commanded_angle = (int)angle_true;
      start_angle = (int)angle_true; 
    }
    #ifdef DEBUG_BIG_SERVOS
    logging_msg += ("MS per deg: ");
    logging_msg += (ms_per_deg);
    #endif
    
    mov_dir = (target_angle>angle_true)*2-1; //1 if increasing, -1 if decreasing angle

    update_motor_angle(cur_commanded_angle);
  }
}

void Joint::update_motor_angle(double angle){
  double constrained_angle = angle; //TODO change for JOINT
  constrained_angle = constrain(constrained_angle, min_angle, max_angle); //limit to angle limits for the servo
  double motor_angle = double((constrained_angle/angle_mult)+this->angle_offset);
  #ifdef DEBUG_BIG_SERVOS
  logging_msg += ("Cnst");
  logging_msg += String(constrained_angle);
  logging_msg += ("2ang: ");
  logging_msg += String(angle);
  logging_msg += ("motorVal: ");
  logging_msg += String(motor_angle);
  #endif
  if(move_enabled){
    servo_out->write(motor_angle);
  }
}

int Joint::check_for_stall(){
  //TODO CHANGE THIS!!!!!!!!!!
  return 1;
  if(abs(lastXValues[JOINT_BUFF_LEN-1]-lastXValues[0]) > 10){
    stopped_time = millis();
  }
  if(!move_enabled){
    stopped_time = millis();
  }
  //if less than 5 degrees difference between set and target angles
  if(abs(this->angle_true - this->target_angle)<4){
    stopped_time = millis();   
  }
  if(millis()-stopped_time > 800){ //if stopped for more than 1s and trying to move to position
      this->release();
      logging_msg += "LARGE JOINT STALLED, POWER DISCONECTED";
      return 0;
  }
  return 1;
}

int Joint::loop() {
  this->update_temp();
  this->update_joint_angle();
  if((millis() - last_buffer_update_time_ms) > pos_buffer_update_time_ms){
    this->averageSpeedBuffAppend();
  }
  loop_slow_move();
  log_info();
  #ifdef DEBUG_BIG_SERVOS
  if(millis()-start_move_time > 5000 && !did_log){
    did_log = true;
    motor_test_values+=" ";
    motor_test_values+=String(angle_sensor->get_angle());
  }
  logging_msg += ("Pot Angle: ");
  logging_msg += String(this->angle_true);
  #endif
  return this->check_for_stall(); //1 is no stall, 0 is stall and will shut off motor.
 
}

//Joint with a smart Hiwonder servo that has position and temperature sensing over half channel UART with rs485 module (at 3.3V)
class HiwonderJoint : public Joint {
  public:
    HiwonderJoint(double angle_offset, double min_angle, double max_angle, double angle_mult, int motor_id, HardwareSerial &SerialX);
    void set_joint_angle(double angle, int use_time_over_speed, int time_2_move_millis, int speed) override;
    void update_motor_angle(double angle, int time_ms);
    void initialise() override;
    int check_for_stall() override;
    void release() override;
    int loop() override;
    int motor_id;
    long int last_update_temp_time = 0;
    void log_info() override{
      #ifdef SHOW_HIWONDER_TEMPS
      logging_msg+=("Current temp:");
      logging_msg+=(temp);
      //logging_msg+=(" Temp limit:");
      //logging_msg+=(LobotSerialServoReadTempLimit(SerialX, motor_id));
      #endif
    }
    // Additional functionality specific to Thumb
  protected:
    void update_temp() override;
    void update_joint_angle() override;
    
    HardwareSerial &SerialX;
};

HiwonderJoint::HiwonderJoint(double angle_offset, double min_angle, double max_angle,double angle_mult,int motor_id,HardwareSerial &SerialX)
    : Joint(0,angle_offset, min_angle, max_angle,angle_mult,NULL), motor_id(motor_id), SerialX(SerialX){
  LobotSerialWriteTempLimit(SerialX, motor_id, HIWONDER_MAX_TEMP);
  last_update_temp_time = millis();
}

int HiwonderJoint::loop() {
  this->update_temp();
  this->update_joint_angle();
  log_info();
  return this->check_for_stall(); //1 is no stall, 0 is stall and will shut off motor.
}

void HiwonderJoint::update_temp(){
  if(millis() - last_update_temp_time > 1000){
    temp = LobotSerialServoReadTemp(SerialX, motor_id);
    last_update_temp_time = millis();
  }
  //LobotSerialWriteTempLimit(SerialX, motor_id, HIWONDER_MAX_TEMP);
}

void HiwonderJoint::release(){
  LobotSerialServoUnload(SerialX, this->motor_id);
}

int HiwonderJoint::check_for_stall(){
  return 1;
}

void HiwonderJoint::initialise() {
  LobotSerialServoLoad(SerialX, this->motor_id); //TODO uncomment?
    // Function implementation
}

void HiwonderJoint::set_joint_angle(double angle, int use_time_over_speed, int time_2_move_millis, int speed) {
  target_angle = angle;
  if(use_time_over_speed){
    update_motor_angle(target_angle, time_2_move_millis);
  }
}

void HiwonderJoint::update_joint_angle(){
  double read_angle = LobotSerialServoReadPosition(SerialX, motor_id);
  //Serial.println(read_angle);
  if(read_angle > -1){ //only update if reading didnt fail
    angle_true = (read_angle-angle_offset)*angle_mult;
  }
  //Serial.println("true:");
  //Serial.println(angle_true);
}

void HiwonderJoint::update_motor_angle(double angle, int time_ms){
  double constrained_angle = angle; //TODO change for JOINT
  constrained_angle = constrain(constrained_angle, min_angle, max_angle); //limit to angle limits for the servo
  int motor_angle = (int)((constrained_angle/angle_mult)+this->angle_offset);
  #ifdef DEBUG_HIWONDER
  Serial.print("Setting motor: ");
  Serial.print("To angle: ");
  Serial.print(angle);
  Serial.print("Contrained angle: ");
  Serial.print(constrained_angle);
  Serial.print("Actaul set: ");
  Serial.print(motor_angle);
  Serial.print("min: ");
  Serial.print(min_angle);
  Serial.print("max: ");
  Serial.print(max_angle);
  Serial.print("Enabled: ");
  Serial.print(move_enabled);
  #endif
  if(move_enabled){
    LobotSerialServoMove(SerialX, motor_id, motor_angle, time_ms);
  }
  #ifdef DEBUG_MOTORS
  Serial.print("Setting motor: ");
  Serial.print("To angle: ");
  Serial.print(angle);
  #endif
}

class Finger {
public:
    Finger(DCMotor* motors[], Sensor* sensors[], Sensor* fsr_sensors[]);
    double get_joint_angle(int joint);
    void set_joint_angle(int joint, double angle, double power = -1, int force = -1);
    void set_default_power(int joint, double power);
    void set_default_force(double force);
    void set_abs_power_limits(int joint, double min_power, double max_power);
    void get_forces(double forces[]);
    void update_forces();
    void update_motor_power(int motor,double power);
    void set_tendon_rev_power_max(double power);
    void compute_control();
    void move_enable(int joint, int on_or_off){
      if(!on_or_off){
        move_enabled[joint] = 1;
        this->update_motor_power(joint, 0); //turn off power to motor
        move_enabled[joint] = 0;
        Serial.println("Turnign off");
        motors[joint]->set_motor_power(0);
      }else{
        //initialise_PID();
      }
      this->move_enabled[joint] = on_or_off;
      //reset integral component (mainly just important if the movenet was blocked has been limited for the prevous movement)
      PID_controllers[joint]->SetOutputLimits(-1, 0); //max motor power is limits of PID
      PID_controllers[joint]->SetOutputLimits(0, 1); //max motor power is limits of PID
      PID_controllers[joint]->SetOutputLimits(-int(max_power_limits[joint] * 4095), int(max_power_limits[joint] * 4095)); //max motor power is limits of PID
    }
    virtual void loop();
    virtual void initialise_PID();
protected:
    Sensor** sensors;
    DCMotor** motors;
    Sensor** fsr_sensors;
    int move_enabled[3];
    double default_power[3];
    double default_force;
    double min_power_limits[3];
    double max_power_limits[3];
    double min_angle_limits[3];
    double max_angle_limits[3];
    double angles[3];
    double motor_powers[3];
    double motor_power_limits[3];
    double targets[3];
    double target_force;
    double cur_force;
    double adjusted_tendon_target;
    double PID_on[3];
    PID* PID_controllers[3];
    double tendon_rev_power_max; //tendon motor has a different maximum for the reverse power
    double base_dont_move_tendon_range = PIP_DONT_MOVE_TENDON_RANGE;
    double base_tendon_offset_backwards_mov = BASE_TENDON_COMBINATION_MOVE_DEG;
    PID* force_PID_controller = NULL;
    double force_tendon_multiplier;
    double tendon_power;
};

// Constructor implementation
Finger::Finger(DCMotor* motors[], Sensor* sensors[], Sensor* fsr_sensors[])
    : sensors(sensors), motors(motors), fsr_sensors(fsr_sensors) {
    for (int i = 0; i < 2; i++) {
        default_power[i] = 1;
        min_power_limits[i] = 0;
        motor_powers[i] = 7;
        max_power_limits[i] = 1;
        targets[i] = 45;
        adjusted_tendon_target = 45;
        motor_power_limits[i] = 0; //set the initail power limit to zero to avoid any movement until a command is sent
        move_enabled[i] = 1;
        this->update_motor_power(i, 0); //turn off power to motor
        move_enabled[i] = 0;
        PID_on[i] = false;
        move_enabled[i] = 1;
        this->update_motor_power(i, 0); //turn off power to motor
        move_enabled[i] = 0;
        motors[i]->set_motor_power(0);
    }
    min_angle_limits[0] = 0;
    max_angle_limits[0] = 80;
    min_angle_limits[1] = 5;
    max_angle_limits[1] = 80;
    cur_force = 0;
    default_force = 500.0;
    target_force = 500.0;
    tendon_rev_power_max = 0.3;
    force_tendon_multiplier = 1;
}

double Finger::get_joint_angle(int joint) {
  return sensors[joint]->get_angle();
}

void Finger::set_joint_angle(int joint, double angle, double power, int force) {
    angle = constrain(angle,min_angle_limits[joint],max_angle_limits[joint]);
    if(PID_on[joint] == false){
      PID_controllers[joint]->SetMode(AUTOMATIC); //move this out to start once commands have all been given
      PID_on[joint] = true;
      if(joint==1){
        if (force_PID_controller != nullptr) {
            force_PID_controller->SetMode(AUTOMATIC);
        } else {
            Serial.println("ERROR:  force_PID_controller is null");
            delay(10000);
        }
      }
    }
    if (power == -1) {
        power = default_power[joint];
    }
    if (force == -1) {
        force = default_force;
    }
    //reset integral component (mainly jsut important if the power has been limited for the prevous movement)
    PID_controllers[joint]->SetOutputLimits(-1, 0); //max motor power is limits of PID
    PID_controllers[joint]->SetOutputLimits(0, 1); //max motor power is limits of PID
    PID_controllers[joint]->SetOutputLimits(-int(max_power_limits[joint] * 4095), int(max_power_limits[joint] * 4095)); //max motor power is limits of PID
    // Constrain power to limits
    motor_power_limits[joint] = constrain(power, 0, max_power_limits[joint]);
    targets[joint] = angle;
    if(joint==1){
      this->adjusted_tendon_target = angle;
    }
    target_force = force;
    //TODO do update the power maximums

}

void Finger::set_default_power(int joint, double power) {
    default_power[joint] = power;
}

void Finger::set_tendon_rev_power_max(double power){
  tendon_rev_power_max = power;
}

void Finger::set_default_force(double force) {
    default_force = force;
}

void Finger::set_abs_power_limits(int joint, double min_power, double max_power) {
    min_power_limits[joint] = min_power;
    max_power_limits[joint] = max_power;
}

void Finger::get_forces(double forces[]) {
  for(int i = 0; i < 3; i++){
    forces[i] = fsr_sensors[i]->get_force();
  }
}

void Finger::update_forces(){
  for(int i = 0; i < 3; i++){
    fsr_sensors[i]->update_sensor();
  }
  //cur_force = max(fsr_sensors[0]->get_force(),max(fsr_sensors[1]->get_force(),fsr_sensors[2]->get_force())); //only care about the highest force value
  cur_force = fsr_sensors[0]->get_force()+fsr_sensors[1]->get_force()+fsr_sensors[2]->get_force(); //only care about the highest force value
}

void Finger::update_motor_power(int motor,double power){
  double constrained_power = 0;
  double requested_power = power/4096;
  #ifdef DEBUG_MOTORS
  Serial.print("req_power: ");
  Serial.print(motor_powers[motor]);
  #endif
  
  
  if(requested_power>0){
    if(requested_power>min_power_limits[motor]){ //only set the power if it is above a certain limit to avoid whinning
      constrained_power = requested_power;
    }
    if(motor==1){//tendon is special as has smaller reverse limit and no min_power when in reverse?
      constrained_power = requested_power;
      constrained_power = constrained_power*tendon_rev_power_max; //constrain(constrained_power,0,tendon_rev_power_max);
    }
  }else{
    if(requested_power<min_power_limits[motor]*-1){ //only set the power if it is above a certain limit to avoid whinning
      //Serial.print("Here!");
      constrained_power = requested_power;
    }
  }
  constrained_power = constrain(constrained_power, -motor_power_limits[motor], motor_power_limits[motor]); //limit to maximum power specified for this movement
  
  
  if(move_enabled[motor]){
    #ifdef DEBUG_MOTORS
    Serial.print("Setting motor: ");
    Serial.print(motor);
    Serial.print("To power: ");
    Serial.print(constrained_power);
    //Serial.print("plimit: ");
    //Serial.print(motor_power_limits[motor]);
    #endif
    motors[motor]->set_motor_power((int)(constrained_power*4095));
  }else{
    motors[motor]->set_motor_power(0);
  }
}

void Finger::initialise_PID() {
  PID_controllers[0] = new PID(&angles[0], &motor_powers[0], &targets[0], 120, 120, 0, 1); //base
  PID_controllers[1] = new PID(&angles[1], &motor_powers[1], &adjusted_tendon_target, 180, 50000, 0.05, 1); //tendon , 180, 10000, 0.05, 1); //t
  this->force_PID_controller = new PID(&cur_force, &force_tendon_multiplier, &target_force, 0.01, 0, 0, 0); //tendon force
  force_PID_controller->SetSampleTime(1);
  force_PID_controller->SetOutputLimits(-1, 1); 
  if (force_PID_controller == nullptr) {
    Serial.println("CONTROLLER IS NUll here too");
    delay(5000);
  } 
  for(int i = 0; i < 2; i++) {
      PID_controllers[i]->SetOutputLimits(-int(max_power_limits[i] * 4095), int(max_power_limits[i] * 4095)); //max motor power is limits of PID
      PID_controllers[i]->SetSampleTime(1);
      move_enabled[i] = 1;
      this->update_motor_power(i, 0); //turn off power to motor
      move_enabled[i] = 0;
      motors[i]->set_motor_power(0);
  }
  PID_controllers[0]->SetSampleTime(1);
  PID_controllers[1]->SetSampleTime(1); //change this???
}

void Finger::compute_control(){
  //if opening PIP joint and not currently close to the PIP target position
  if(angles[0] > targets[0]+20 /*&& !(angles[0] < targets[0]+base_dont_move_tendon_range && angles[1] < targets[1]+10 && angles[1] > targets[1]-10)*/){ //if opening PIP joint and not currently close to the PIP target position
    /*
    if(angles[0] > targets[0]+PIP_TENDON_DEG_LINEAR_MOVE){ //move a full amount of degrees to allwo the pip to open
      adjusted_tendon_target = targets[1]-base_tendon_offset_backwards_mov;
    }else{
      adjusted_tendon_target = targets[1]-base_tendon_offset_backwards_mov*(1-0.5*((angles[0]-targets[0])/PIP_TENDON_DEG_LINEAR_MOVE));
    }
    */
    adjusted_tendon_target = targets[1]-base_tendon_offset_backwards_mov;
  }else{
    adjusted_tendon_target = targets[1];
  }
  adjusted_tendon_target = constrain(adjusted_tendon_target,min_angle_limits[1],max_angle_limits[1]);
  #ifdef DEBUG_FINGERS
    Serial.print("adjsuted_target: ");
    Serial.print(adjusted_tendon_target);
    Serial.print("adjsuted_target: ");
    Serial.print(adjusted_tendon_target);
    Serial.print("Force PID Controller Mode: ");
    Serial.println(force_PID_controller->GetMode() == AUTOMATIC ? "AUTOMATIC" : "MANUAL");
  #endif
  
  force_PID_controller->Compute();

  /*
  if (force_PID_controller != nullptr) {
      force_PID_controller->Compute();
  } else {
      Serial.println("ERROR:  force_PID_controller is null when doing compute");
      delay(10000);
  }
  */
  for (int i = 0; i < 2; i++) {
    PID_controllers[i]->Compute(); // Compute the PID control
  }
  if(move_enabled[0]){
    this->update_motor_power(0,motor_powers[0]);
  }
  if(move_enabled[1]){
    if(force_tendon_multiplier<0 && motor_powers[1]>0){
      tendon_power = abs(force_tendon_multiplier)*motor_powers[1];
    }else{
      tendon_power = force_tendon_multiplier*motor_powers[1];
    }
    
    this->update_motor_power(1,tendon_power);
  }
  #ifdef DEBUG_FORCE_CONT
  Serial.print("target force:");
  Serial.print(target_force);
  Serial.print("cur force:");
  Serial.print(cur_force);
  Serial.print("orig tendon power");
  Serial.println(motor_powers[1]);
  Serial.print("tendon power:");
  Serial.print(tendon_power);
  Serial.print("mult");
  Serial.println(force_tendon_multiplier);
  #endif
}


void Finger::loop() {
  this->update_forces();
  //Serial.print("Finger looping");
  for (int i = 0; i < 2; i++) {
    sensors[i]->update_sensor();
    //Serial.print("after sensor update:");
    //Serial.println(millis()-start_off_loop_time);
    angles[i] = sensors[i]->get_angle();
    //Serial.print("after getting angle");
    //Serial.println(millis()-start_off_loop_time);
  }
  compute_control();
}

// Thumb class extending Finger
class Thumb : public Finger {
public:
    Thumb(DCMotor* motors[], Sensor* sensors[], Sensor* fsr_sensors[]);
    void initialise_PID() override;
    double get_joint_angle(int joint);
    void loop() override;
    // Additional functionality specific to Thumb
};

Thumb::Thumb(DCMotor* motors[], Sensor* sensors[],Sensor* fsr_sensors[])
    : Finger(motors, sensors, fsr_sensors) {
    for (int i = 0; i < 3; i++) {
        default_power[i] = 1;
        min_power_limits[i] = 0;
        motor_powers[i] = 7;
        max_power_limits[i] = 1;
        targets[i] = 45;
        motor_power_limits[i] = 0; //set the initail power limit to zero to avoid any movement until a command is sent
        move_enabled[i] = 1;
        this->update_motor_power(i, 0); //turn off power to motor
        move_enabled[i] = 0;
        PID_on[i] = false;
        min_angle_limits[0] = 13;
        max_angle_limits[0] = 101;
        min_angle_limits[1] = 5;
        max_angle_limits[1] = 80;
        min_angle_limits[2] = -8;
        max_angle_limits[2] = 56.5;
    }
    default_force = 1.0;
}

void Thumb::initialise_PID() {
  PID_controllers[0] = new PID(&angles[0], &motor_powers[0], &targets[0], 60, 60, 0, 1); //tilit
  PID_controllers[1] = new PID(&angles[1], &motor_powers[1], &adjusted_tendon_target, 180, 50000, 0.05, 1); //tendon
  PID_controllers[2] = new PID(&angles[0], &motor_powers[0], &targets[0], 120, 200, 0, 1); //rotate
  this->force_PID_controller = new PID(&cur_force, &force_tendon_multiplier, &target_force, 0.01, 0, 0, 0); //tendon force
  force_PID_controller->SetSampleTime(1);
  force_PID_controller->SetOutputLimits(-1, 1); 
  for(int i = 0; i < 3; i++) {
      PID_controllers[i]->SetOutputLimits(-int(max_power_limits[i] * 4095), int(max_power_limits[i] * 4095)); //max motor power is limits of PID
      PID_controllers[i]->SetMode(AUTOMATIC); //move this out to start once commands have all been given
  }
  PID_controllers[0]->SetSampleTime(1);
  PID_controllers[1]->SetSampleTime(1); //change this??? to be longer, used to be 50 and this worked, currently trying it shorter
  PID_controllers[2]->SetSampleTime(1);
}

double Thumb::get_joint_angle(int joint) {
    return sensors[joint]->get_angle();
}

void Thumb::loop() {
  this->update_forces();
  if (force_PID_controller != nullptr) {
      force_PID_controller->Compute();
  } else {
      Serial.println("ERROR:  force_PID_controller is null when doing compute");
      delay(10000);
  }
  //Serial.print("Looping the thumb");
  for (int i = 0; i < 3; i++) {
      sensors[i]->update_sensor();
      angles[i] = sensors[i]->get_angle();
      PID_controllers[i]->Compute(); // Compute the PID control
      this->update_motor_power(i,motor_powers[i]);
  }

}

class RoboticHand {
public:
    RoboticHand(int s_chip_select[4], int s_hand_analog_out[2]);
    void set_binary_output(int number);
    void read_analog_data(int data[2]);
    void read_sensors(int pin, int analog_data[2]);
    void read_all_sensors(int analog_data[16][2]);
    void rotate_motor(int power);
    void set_motor_power(int finger, int motor_number, int power);
    void testing_loop();
    void loop();
    void get_joint_angles(double angles[17]);
    void set_hand_move_time(double move_time);
    void set_arm_move_time(double move_time);
    void initialize(int type);
    void set_joint_angle(int joint,double angle, double power, int force);
    void set_all_joint_angles(double angles[]);
    void set_arm_joint_angles(double angles[]);
    void set_hand_joint_angles(double angles[]);
    void get_finger_forces(double forces[15]);
    void set_finger_only_forces(double force);
    void set_thumb_only_forces(double force);
    void set_finger_and_thumb_forces(double force);
    void release_all();
private:
    int* chip_select;  // Pins for the binary output
    int* hand_analog_out;  // Pins for the analog input
    int motor_pwm_pins[5][3];
    int move_time_ms_hand = 3000;
    int move_time_ms_arm = 3000;
    Finger* fingers[5];
    Joint* joints[6];
    Adafruit_PWMServoDriver motor_boards[5][3];

    // Add these member variables
    DCMotor* finger_motors[4][2];
    Sensor* finger_sensors[4][2];
    Sensor* finger_fsr_sensors[5][3];
    DCMotor* thumb_motors[3];
    Sensor* thumb_sensors[3];
    int smoothing_count[17];
    double last_angles[17];
};

RoboticHand::RoboticHand(int s_chip_select[4], int s_hand_analog_out[2]){
    // Initialize pin modes for binary output and analog input pins
    chip_select = s_chip_select;
    hand_analog_out = s_hand_analog_out;
    for (int i = 0; i < 2; i++) {
        pinMode(hand_analog_out[i], INPUT);
    }
}

void RoboticHand::initialize(int type){


  //************************************************************************RIGHT HAND INFO**************************************************************
  if(type == 0){ //RIGHT HAND
    double hall_angles[5][20] = {
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 89, 99999}, //index
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 88, 99999}, //middle
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 87, 99999}, //ring
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 86.5, 99999}, //pinky
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 89.5, 99999}
    };
    double hall_readings[5][20] = {
        {896, 914, 924, 940, 958, 974, 1005, 1036, 1079, 1125, 1186, 1304, 1410, 1586, 1827, 2132, 2580, 3307, 3580, 99999}, //index
        {883, 901, 915, 932, 950, 968, 1002, 1045, 1092, 1164, 1244, 1356, 1487, 1672, 2015, 2410, 3154, 3935, 4022, 99999}, //middle
        {852, 864, 872, 886, 901, 916, 937, 966, 997, 1037, 1098, 1152, 1247, 1382, 1562, 1802, 2095, 2494, 2766, 99999}, //ring
        {854, 874, 884, 902, 919, 941, 966, 1002, 1042, 1096, 1152, 1270, 1384, 1564, 1892, 2264, 2980, 4004, 4014, 99999}, //pinky
        {880, 909.0697674, 930, 955, 980, 1025, 1070, 1110, 1150, 1230, 1310, 1445, 1580, 1705, 1830, 2130, 2430, 3025, 3620, 99999} //thumb
    };
    finger_motors[0][0] = new DCMotor(&pwm1, 11,10);
    finger_motors[0][1] = new DCMotor(&pwm2, 8,9);
    finger_motors[1][0] = new DCMotor(&pwm1, 8, 9);
    finger_motors[1][1] = new DCMotor(&pwm2, 10, 11);
    finger_motors[2][0] = new DCMotor(&pwm1, 14, 15);
    finger_motors[2][1] = new DCMotor(&pwm2, 12, 13);
    finger_motors[3][0] = new DCMotor(&pwm1, 12, 13);
    finger_motors[3][1] = new DCMotor(&pwm2, 14, 15);

    thumb_motors[0] = new DCMotor(&pwm2, 4,5);
    thumb_motors[1] = new DCMotor(&pwm2, 6,7);
    thumb_motors[2] = new DCMotor(&pwm2, 3,2);

    finger_sensors[0][0] = new PotSensor(1, 1, chip_select, hand_analog_out, 0.09, -126,1);
    finger_sensors[0][1] = new HallSensor(1, 9, chip_select, hand_analog_out, hall_angles[0], hall_readings[0],1);
    finger_sensors[1][0] = new PotSensor(1, 2, chip_select, hand_analog_out, 0.09, -122,1);
    finger_sensors[1][1] = new HallSensor(1, 10, chip_select, hand_analog_out, hall_angles[1], hall_readings[1],1);
    finger_sensors[2][0] = new PotSensor(1, 3, chip_select, hand_analog_out, 0.09, -139,1);
    finger_sensors[2][1] = new HallSensor(1, 11, chip_select, hand_analog_out, hall_angles[2], hall_readings[2],1);
    finger_sensors[3][0] = new PotSensor(1, 4, chip_select, hand_analog_out, 0.09, -136,1);
    finger_sensors[3][1] = new HallSensor(1, 12, chip_select, hand_analog_out, hall_angles[3], hall_readings[3],1);

    finger_fsr_sensors[0][0] = new FSRSensor(2, 0, chip_select, hand_analog_out,1,1977.38744345351,4096,-345); //a k c
    finger_fsr_sensors[0][1] = new FSRSensor(2, 5, chip_select, hand_analog_out,1,324.226855894134,4096,-321.3929485468889);
    finger_fsr_sensors[0][2] = new FSRSensor(2, 10, chip_select, hand_analog_out,1,656.276423603454,4096,-279.8769762971593);
    finger_fsr_sensors[1][0] = new FSRSensor(2, 1, chip_select, hand_analog_out,0,517.542742690176,4096,0); //disabled
    finger_fsr_sensors[1][1] = new FSRSensor(2, 6, chip_select, hand_analog_out,1,517.542742690176,4096,-216);
    finger_fsr_sensors[1][2] = new FSRSensor(2, 11, chip_select, hand_analog_out,0,517.542742690176,4096,0); ///disabled
    finger_fsr_sensors[2][0] = new FSRSensor(2, 2, chip_select, hand_analog_out,1,1746.26509319768,4096,-395);
    finger_fsr_sensors[2][1] = new FSRSensor(2, 7, chip_select, hand_analog_out,1,274.022629517817,4096,-130.3488428785282);
    finger_fsr_sensors[2][2] = new FSRSensor(2, 15, chip_select, hand_analog_out,1,261.225939456928,4096,-145);
    finger_fsr_sensors[3][0] = new FSRSensor(2, 3, chip_select, hand_analog_out,1,1007.25516546861,4096,-220);
    finger_fsr_sensors[3][1] = new FSRSensor(2, 8, chip_select, hand_analog_out,1,220.712345008304,4096,-112.4);
    finger_fsr_sensors[3][2] = new FSRSensor(2, 13, chip_select, hand_analog_out,1,243.095433880297,4096,-52.9);
    finger_fsr_sensors[4][0] = new FSRSensor(2, 4, chip_select, hand_analog_out,1,1301.37094653136,4096,-156.5);
    finger_fsr_sensors[4][1] = new FSRSensor(2, 9, chip_select, hand_analog_out,1,436.232877432058,4096,-298.2);
    finger_fsr_sensors[4][2] = new FSRSensor(2, 14, chip_select, hand_analog_out,1,315.76453265087,4096,-123);

    thumb_sensors[0] = new PotSensor(1, 5, chip_select, hand_analog_out, -0.0874, 205,1); //tilt
    thumb_sensors[1] = new HallSensor(1, 13, chip_select, hand_analog_out, hall_angles[4], hall_readings[4],1); //tilt
    thumb_sensors[2] = new PotSensor(1, 15, chip_select, hand_analog_out, 0.0934, -190,1); //tilt
    for(int i = 0; i < 4; i++) {
      fingers[i] = new Finger(finger_motors[i], finger_sensors[i],finger_fsr_sensors[i]);
    }
    fingers[4] = new Thumb(thumb_motors, thumb_sensors, finger_fsr_sensors[4]);
    for(int i = 0; i < 5; i++) {
      fingers[i]->set_abs_power_limits(1, 0, 1);
      fingers[i]->initialise_PID();
    }
    fingers[0]->set_tendon_rev_power_max(0.8);
    fingers[1]->set_tendon_rev_power_max(0.8); 
    fingers[2]->set_tendon_rev_power_max(0.8); //harder to turn ring and middle finger pulleys for some reason
    fingers[3]->set_tendon_rev_power_max(1);
    fingers[4]->set_tendon_rev_power_max(0.8);

    joints[0] = new HiwonderJoint(552, -72, 61, 1/4.16666, 6, HISERIAL); //wrist
    joints[1] = new HiwonderJoint(937, -11, 180, -1/4.16666, 7, HISERIAL); //forearm
    joints[2] = new Joint(8,56, 2, 132,1.406,new DirectPotSensor(16, -0.0801, 2696.62,1)); //elbow //------
    joints[3] = new HiwonderJoint(546, -10, 90, -1/4.16666, 8, HISERIAL); //lower shoulder
    joints[4] = new Joint(6,135.4, 0, 90, -0.96,new DirectPotSensor(38, 0.056777, 1336,1)); //mid shoulder
    //joints[5] = new Joint(4,40.62, -40, 185, 1.35,new DirectPotSensor(25, -1, 0,1)); //upper shoulder
    joints[5] = new Joint(4,48.58, -40, 185, 1.35,new DirectPotSensor(25, -0.0761,2920,1)); //upper shoulder
    //joints[5] = new Joint(4,58.7, -40, 185, 1.55,new DirectPotSensor(25, -0.18018, 1299,1)); //upper shoulder

    
    joints[0]->move_enable(1); //wrist
    
    joints[1]->move_enable(1); //forearm
    joints[2]->move_enable(1); //elbow
    joints[3]->move_enable(1); //lower shoulder
    joints[4]->move_enable(1); //mid shoulder
    joints[5]->move_enable(1); //upper shoulder
    
    fingers[0]->move_enable(0,1);
    fingers[0]->move_enable(1,1);
    fingers[1]->move_enable(0,1);
    fingers[1]->move_enable(1,1);
    fingers[2]->move_enable(0,1);
    fingers[2]->move_enable(1,1);
    fingers[3]->move_enable(0,1);
    fingers[3]->move_enable(1,1);

    fingers[4]->move_enable(0,1);
    fingers[4]->move_enable(1,1);
    fingers[4]->move_enable(2,1);
  //****************************************************************END OF RIGHT HAND INFO**************************************************************

  //************************************************************************LEFT HAND INFO**************************************************************
  }else if(type==1){
    double hall_angles[5][20] = {
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 89, 99999}, //index
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 88, 99999}, //middle
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 87, 99999}, //ring
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 86.5, 99999}, //pinky
    {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 89.5, 99999} //thumb
    };
    double hall_readings[5][20] = {
        {896, 914, 924, 940, 958, 974, 1005, 1036, 1079, 1125, 1186, 1304, 1410, 1586, 1827, 2132, 2580, 3307, 3580, 99999}, //index
        {883, 901, 915, 932, 950, 968, 1002, 1045, 1092, 1164, 1244, 1356, 1487, 1672, 2015, 2410, 3154, 3935, 4022, 99999}, //middle
        {852, 864, 872, 886, 901, 916, 937, 966, 997, 1037, 1098, 1152, 1247, 1382, 1562, 1802, 2095, 2494, 2766, 99999}, //ring
        {854, 874, 884, 902, 919, 941, 966, 1002, 1042, 1096, 1152, 1270, 1384, 1564, 1892, 2264, 2980, 4004, 4014, 99999}, //pinky
        {956, 971, 982, 998, 1010, 1032, 1056, 1080, 1124, 1164, 1242, 1314, 1426, 1586, 1806, 2035, 2562, 3164, 3564, 99999} //thumb
    };
    finger_motors[0][0] = new DCMotor(&pwm3, 11,10);
    finger_motors[0][1] = new DCMotor(&pwm3, 8,9);
    finger_motors[1][0] = new DCMotor(&pwm3, 8, 9);
    finger_motors[1][1] = new DCMotor(&pwm3, 10, 11);
    finger_motors[2][0] = new DCMotor(&pwm3, 14, 15);
    finger_motors[2][1] = new DCMotor(&pwm3, 12, 13);
    finger_motors[3][0] = new DCMotor(&pwm3, 12, 13);
    finger_motors[3][1] = new DCMotor(&pwm3, 14, 15);

    thumb_motors[0] = new DCMotor(&pwm3, 4,5);
    thumb_motors[1] = new DCMotor(&pwm3, 6,7);
    thumb_motors[2] = new DCMotor(&pwm3, 3,2);

    finger_sensors[0][0] = new PotSensor(1, 1, chip_select, hand_analog_out, 0.09, -142,1);
    finger_sensors[0][1] = new HallSensor(1, 9, chip_select, hand_analog_out, hall_angles[0], hall_readings[0],1);
    finger_sensors[1][0] = new PotSensor(1, 2, chip_select, hand_analog_out, 0.09, -137,1);
    finger_sensors[1][1] = new HallSensor(1, 10, chip_select, hand_analog_out, hall_angles[1], hall_readings[1],1);
    finger_sensors[2][0] = new PotSensor(1, 3, chip_select, hand_analog_out, 0.09, -139,1);
    finger_sensors[2][1] = new HallSensor(1, 11, chip_select, hand_analog_out, hall_angles[2], hall_readings[2],1);
    finger_sensors[3][0] = new PotSensor(1, 4, chip_select, hand_analog_out, 0.09, -148,1);
    finger_sensors[3][1] = new HallSensor(1, 12, chip_select, hand_analog_out, hall_angles[3], hall_readings[3],1);

    finger_fsr_sensors[0][0] = new FSRSensor(2, 0, chip_select, hand_analog_out,1,1977.38744345351,4096,-688.872752527812); //a k c
    finger_fsr_sensors[0][1] = new FSRSensor(2, 5, chip_select, hand_analog_out,1,324.226855894134,4096,-21.3929485468889);
    finger_fsr_sensors[0][2] = new FSRSensor(2, 10, chip_select, hand_analog_out,1,656.276423603454,4096,-81.8769762971593);
    finger_fsr_sensors[1][0] = new FSRSensor(2, 1, chip_select, hand_analog_out,1,517.542742690176,4096,-70.8969943613639);
    finger_fsr_sensors[1][1] = new FSRSensor(2, 6, chip_select, hand_analog_out,1,517.542742690176,4096,-70.8969943613639);
    finger_fsr_sensors[1][2] = new FSRSensor(2, 11, chip_select, hand_analog_out,1,517.542742690176,4096,-70.8969943613639);
    finger_fsr_sensors[2][0] = new FSRSensor(2, 2, chip_select, hand_analog_out,1,1746.26509319768,4096,-662.52112031144);
    finger_fsr_sensors[2][1] = new FSRSensor(2, 7, chip_select, hand_analog_out,1,274.022629517817,4096,-17.3488428785282);
    finger_fsr_sensors[2][2] = new FSRSensor(2, 15, chip_select, hand_analog_out,1,261.225939456928,4096,-65.9222659810223);
    finger_fsr_sensors[3][0] = new FSRSensor(2, 3, chip_select, hand_analog_out,1,1007.25516546861,4096,-468.668005541961);
    finger_fsr_sensors[3][1] = new FSRSensor(2, 8, chip_select, hand_analog_out,1,220.712345008304,4096,-53.485238807424);
    finger_fsr_sensors[3][2] = new FSRSensor(2, 13, chip_select, hand_analog_out,1,243.095433880297,4096,-29.5503785844352);
    finger_fsr_sensors[4][0] = new FSRSensor(2, 4, chip_select, hand_analog_out,1,1301.37094653136,4096,-289.039927834054);
    finger_fsr_sensors[4][1] = new FSRSensor(2, 9, chip_select, hand_analog_out,1,436.232877432058,4096,-48.1871361481237);
    finger_fsr_sensors[4][2] = new FSRSensor(2, 14, chip_select, hand_analog_out,1,315.76453265087,4096,-50.375403346661);

    thumb_sensors[0] = new PotSensor(1, 5, chip_select, hand_analog_out, -0.0874, 205,1); //tilt
    thumb_sensors[1] = new HallSensor(1, 13, chip_select, hand_analog_out, hall_angles[4], hall_readings[4],1); //tilt
    thumb_sensors[2] = new PotSensor(1, 15, chip_select, hand_analog_out, 0.0934, -190,1); //tilt
    for(int i = 0; i < 4; i++) {
      fingers[i] = new Finger(finger_motors[i], finger_sensors[i],finger_fsr_sensors[i]);
    }
    fingers[4] = new Thumb(thumb_motors, thumb_sensors, finger_fsr_sensors[4]);
    for(int i = 0; i < 5; i++) {
      fingers[i]->set_abs_power_limits(1, 0, 1);
      fingers[i]->initialise_PID();
    }

    joints[0] = new HiwonderJoint(424, -67, 55, 1/4.16666, 3, HISERIAL); //wrist
    joints[1] = new HiwonderJoint(68, 0, 180, 1/4.16666, 5, HISERIAL); //forearm
    joints[2] = new Joint(7,16.31, 2, 126, 0.954,new DirectPotSensor(17, -0.0536, 3414.179,1)); //elbow
    joints[3] = new HiwonderJoint(294, -10, 90, 1/4.16666, 4, HISERIAL); //lower shoulder
    joints[4] = new Joint(5,160, 2, 87, -1.02,new DirectPotSensor(39, 0.0547,762.34,1)); //mid shoulder
    joints[5] = new Joint(3,145.5882, -40, 185, -1.36,new DirectPotSensor(24, 0.0771, 1133.5927,1)); //LEFT upper shoulder //pin, zero offest, min angle (deg), max angle (deg), multiplier (deg per reading), sensor(pin, multiplier, offset,enabled))
    //joints[5]->move_enable(1); (calibrated)
    joints[0]->move_enable(1);
    joints[1]->move_enable(1);
    joints[2]->move_enable(1); 
    joints[3]->move_enable(1); 
    joints[4]->move_enable(1); 
    joints[5]->move_enable(1); 
    fingers[0]->set_tendon_rev_power_max(0.6);
    fingers[1]->set_tendon_rev_power_max(0.6); 
    fingers[2]->set_tendon_rev_power_max(0.6); //harder to turn ring and middle finger pulleys for some reason
    fingers[3]->set_tendon_rev_power_max(0.6);
    fingers[4]->set_tendon_rev_power_max(0.6);
  }
  //*******************************************************************END OF LEFT HAND INFO**************************************************************
  for(int i = 0; i < 17; i++) {
    smoothing_count[i] =0;
    last_angles[i] = 0;
  }
}

void RoboticHand::testing_loop() {
    for(int i = 0; i < 5; i++) {
        fingers[i]->loop();
    }
}

void RoboticHand::loop() {
    //Serial.print("start of hand loop:");
   // Serial.println(millis()-start_off_loop_time);
    for(int i = 0; i < 5; i++) {
      //Serial.print("Finger: ");
     //Serial.print(i);
      fingers[i]->loop();
      //  Serial.print("after finger:");
       // Serial.print(i);
       // Serial.print(" :");
       // Serial.println(millis()-start_off_loop_time);
    }
    //Serial.print("afingers:");
    //Serial.println(millis()-start_off_loop_time);
    for(int i = 0; i < 6; i++) {
        joints[i]->loop();
    }
    //Serial.print("ajoints");
    //Serial.println(millis()-start_off_loop_time);
}

void RoboticHand::get_finger_forces(double forces[15]){
  for (int i = 0; i < 5; i++) {
    fingers[i]->get_forces(&forces[i*3]);
  }
}

void RoboticHand::set_finger_only_forces(double force){
  for (int i = 0; i < 4; i++) {
    fingers[i]->set_default_force(force);
  }
}

void RoboticHand::set_thumb_only_forces(double force){
  fingers[4]->set_default_force(force);
}

void RoboticHand::set_finger_and_thumb_forces(double force){
  for (int i = 0; i < 5; i++) {
    fingers[i]->set_default_force(force);
  }
}

void RoboticHand::get_joint_angles(double angles[17]) {
    // Initializes array with default values (7 in this case)
    for (int i = 0; i < 17; i++) {
        angles[i] = 7;
    }

    int angles_i = 0;

    // Loop through the first 4 fingers (index 0 to 3)
    for (int i = 0; i < 4; i++) {
        angles[angles_i++] = fingers[i]->get_joint_angle(0);  
        angles[angles_i++] = fingers[i]->get_joint_angle(1);  
    }

    // Handle the thumb
    angles[angles_i++] = fingers[4]->get_joint_angle(0);  
    angles[angles_i++] = fingers[4]->get_joint_angle(1);  
    angles[angles_i++] = fingers[4]->get_joint_angle(2);  // Assuming index 2 for the third joint of the thumb

    for (int i = 0; i < 6; i++) {
        angles[angles_i++] = joints[i]->get_joint_angle();  
    }

    //Smooth angles
    for(int i = 0; i< 17; i++){
      angles[i] = apply_exp_smooth(last_angles[i],angles[i],SMOOTHING_APLHA,&smoothing_count[i]);
      last_angles[i] = angles[i];
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

void RoboticHand::read_sensors(int pin, int analog_data[2]) {
    set_binary_output(pin);
    delayMicroseconds(1); // Small delay to allow pins to stabilize
    read_analog_data(analog_data);
}

void RoboticHand::read_all_sensors(int analog_data[16][2]) {
    // Loop through numbers 0 to 15, setting the binary output and reading analog data
    for (int i = 0; i < 16; i++) {
        set_binary_output(i);
        delayMicroseconds(1); // Small delay to allow pins to stabilize
        read_analog_data(analog_data[i]);
    }
}

void RoboticHand::set_joint_angle(int joint,double angle, double power, int force){
  if(joint<=7){ //finger joint
    int finger_i = joint/2;
    fingers[finger_i]->set_joint_angle(joint%2, angle, power, force);
  }else if(joint<=10){ //thumb joint (8,9,10)
    fingers[4]->set_joint_angle(joint-8, angle, power, force);
  }else{//do the rest of the joints
    joints[joint-11]->set_joint_angle(angle,1,move_time_ms_arm,1);
    //joints[joint-11]->update_motor_angle(angle);
  }
}

void RoboticHand::set_all_joint_angles(double angles[]){
  for (int joint = 0; joint<17; joint++){
    this->set_joint_angle(joint, angles[joint], -1,-1);
  }
}

void RoboticHand::set_arm_joint_angles(double angles[]){
  for (int joint = 11; joint<17; joint++){
    this->set_joint_angle(joint, angles[joint], -1,-1);
  }
}

void RoboticHand::set_hand_joint_angles(double angles[]){
  for (int joint = 0; joint<11; joint++){
    this->set_joint_angle(joint, angles[joint], -1,-1);
  }
}


void RoboticHand::set_hand_move_time(double move_time){
  move_time_ms_hand = (int)(move_time*1000);
}

void RoboticHand::set_arm_move_time(double move_time){
  move_time_ms_arm = (int)(move_time*1000);
}

void RoboticHand::release_all(){
  set_pwms_2_zero(&pwm1);
  set_pwms_2_zero(&pwm2);
  set_pwms_2_zero(&pwm3);
  for(int i = 0; i < 6; i++) {
    joints[i]->release();
  }
}
/*
Fingers
  0 pinky
  1 ring
  2 middle
  3 index
  4 thumb
Motor numbers
  0 proximal phalange motor
  1 tendon motor
  2 thumb rotation
Power
  4095/4096 = max closing
  -4095/-4096 = max opening
*/ 
/*
void RoboticHand::set_motor_power(int finger, int motor_number, int power) {
    //set power limits (TODO change to be limits for each motor if needed)
    if(power > 4095) {
        power = 4095;
        if(DEBUG_MOTORS) {
            Serial.print("power: ");
            Serial.print(power);
            Serial.print("(out of range)");
        }
    } else if(power < -4095) {
        power = -4095;
        if(DEBUG_MOTORS) {
            Serial.print("power: ");
            Serial.print(power);
            Serial.print("(out of range)");
        }
    }
    if(power >= 0) {
        pwm.setPWM(0, 0, abs(power));
        pwm.setPWM(1, 0, 0);
    } else {
        pwm.setPWM(0, 0, 0);
        pwm.setPWM(1, 0, abs(power));
    }
}

void RoboticHand::rotate_motor(int power) { //-4096 to 4096

  if(abs(power)>400){ //avoid small movements and noises
    if(power>=0){
      pwm.setPWM(0, 0, abs(power));
      pwm.setPWM(1, 0, 0);
    }else{
      pwm.setPWM(0, 0, 0);
      pwm.setPWM(1, 0, abs(power));
    }
  }else{
    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, 0);
  }
  
}
*/
// Example usage
int chip_select[4] = {30, 31, 32, 33}; // Define your binary output pins
int hand_analog_out[2] = {40, 41}; // Define your analog input pins
RoboticHand handR(chip_select, hand_analog_out);
RoboticHand handL(chip_select, hand_analog_out);

void set_pwms_2_zero(Adafruit_PWMServoDriver* pwm_board){
  for(int i = 0; i<16;i++){
    pwm_board->setPWM(i, 0, 0);
  }
}

unsigned long currentTime;
unsigned long elapsedTime;
void setup_powered_items(){
  //initialize serial coms for smart servos
  LobotSerialServoInit(HISERIAL, HI_TXEN, HI_RXEN);
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
  //delay(20);
  handR.initialize(0);
  handL.initialize(1);
  handL.set_arm_move_time(3);
  handR.set_arm_move_time(3);
  handL.set_hand_move_time(3);
  handR.set_hand_move_time(3);
  startTime =  millis();
  state = 0;
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  start_off_loop_time = millis();
  delay(5);
  set_pwms_2_zero(&pwm1);
  set_pwms_2_zero(&pwm2);
  set_pwms_2_zero(&pwm3);
}

void setup() {

  delay(5);
  pwm1.begin();
  pwm1.setOscillatorFrequency(PWMOSCILLATORF);
  pwm1.setPWMFreq(PWMF);  // This is the maximum PWM frequency
  set_pwms_2_zero(&pwm1);
  pwm2.begin();
  pwm2.setOscillatorFrequency(PWMOSCILLATORF);
  pwm2.setPWMFreq(PWMF);  // This is the maximum PWM frequency
  set_pwms_2_zero(&pwm2);
  pwm3.begin();
  pwm3.setOscillatorFrequency(PWMOSCILLATORF);
  pwm3.setPWMFreq(PWMF);  // This is the maximum PWM frequency
  set_pwms_2_zero(&pwm3);

  //delay(5000);
  setup_powered_items();

  Serial.begin(115200);
  //Serial.println("Rebuffer_");
  analogReadResolution(13);
  analogReadAveraging(20); //TODO CHANGE back to 20 once finished testing //might have to reduce this averaging amount later to increase response time, result sin aorun 60% less noise at 20 //20 good
  
  //---------------Initialise pwm I2C coms for motor control
  //delay(20000000);
}

const int MAX_SERIAL_READ_LENGTH = 256;
char Recieved_Serial_Chars[MAX_SERIAL_READ_LENGTH];

int new_data = false;
String read_stuff = 0;

//modified from FROM https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
//TODO NEED TO REWRITE TO EMPTY BUFFER CONTINOUSLY DUE TO 63 CHAR LIMIT?
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    //logging_msg+="here1";
    //logging_msg+=String(amt_read);
    while (Serial.available() > 0 && new_data == false) {
        rc = Serial.read();
        //logging_msg+="here2";
        //amt_read++;
        //logging_msg+="here";
        if (rc != endMarker) {
            Recieved_Serial_Chars[ndx] = rc;
            ndx++;
            if (ndx >= MAX_SERIAL_READ_LENGTH) {
                ndx = MAX_SERIAL_READ_LENGTH - 1;
            }
        }
        else {
            Recieved_Serial_Chars[ndx] = '\0'; // terminate the string
            ndx = 0;
            new_data = true;
        }
    }
}

void parseCSVtoArrays(String csv, double array1[17], double array2[17]) {
    int arrayIndex = 0;
    String tempStr = "";
    
    for (unsigned int i = 0; i < csv.length(); i++) {
        char c = csv.charAt(i);
        if (c == ',' || i == csv.length() - 1) {  // When a comma or end of string is encountered
            if (i == csv.length() - 1) {  // Append last character
                tempStr += c;
            }
            
            if (arrayIndex < 17) {
                array1[arrayIndex] = tempStr.toFloat();
            } else {
                array2[arrayIndex - 17] = tempStr.toFloat();
                //read_stuff += "|";
                //read_stuff += tempStr;
            }
            
            tempStr = "";  // Clear temp string for next value
            arrayIndex++;
        } else {
            tempStr += c;  // Keep building the number
        }
        
        // Stop when 34 values are processed
        if (arrayIndex == 34) {
            break;
        }
    }
}
int move_count = 0;

void execute_dt_command(){
  if(!SERIAL_CMD_MOV_ENABLED){ //dont do anything unless serial control is enabled
    return;
  }
  String command_string;
  String command_type; //one longer for null terminator
  String command_data;
  command_string = String(Recieved_Serial_Chars);
  command_type = command_string.substring(0, 20);
  command_data = command_string.substring(20);

  logging_msg+="Recieved command:";
  logging_msg+=(command_type);
  logging_msg+="command data:";
  logging_msg+=(command_data);
  if(command_type == "STOP________________"){
    handR.release_all();
    handL.release_all();
    //handL.stop();
  //if a move command
  }else if(command_type == "MOVE_TIME___________"){
    #ifdef DEBUG_RECIEVE_DATA
    //Serial.println("command_data:");
    //Serial.println(command_data);
    //Serial.println("atof command_data:");
    //Serial.println(command_data.toFloat());
    #endif
    double new_move_time = command_data.toFloat();
    handL.set_arm_move_time(new_move_time);
    handR.set_arm_move_time(new_move_time);
    handL.set_hand_move_time(new_move_time);
    handR.set_hand_move_time(new_move_time);
  }else if(command_string.substring(0,4) == "MOVE"){ //If a move command

    double handR_cmd_angles[17];
    double handL_cmd_angles[17];
    #ifdef DEBUG_RECIEVE_DATA
    logging_msg+=("Received move command, moving ");
    #endif
    move_count++;
    //read_stuff="DOING A MOVE"+String(move_count);
    parseCSVtoArrays(command_data, handR_cmd_angles, handL_cmd_angles);
    String joints_2_move = command_string.substring(11,19);

    if (joints_2_move == "ALL_____"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="All";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handR.set_all_joint_angles(handR_cmd_angles);
      handL.set_all_joint_angles(handL_cmd_angles);
    }else if (joints_2_move == "LEFTSIDE"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="Left Side";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handL.set_all_joint_angles(handL_cmd_angles);
    }else if (joints_2_move == "RGHTSIDE"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="Right Side";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handR.set_all_joint_angles(handR_cmd_angles);
    }else if (joints_2_move == "LEFT_ARM"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="Left Arm";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handL.set_arm_joint_angles(handL_cmd_angles);
    }else if (joints_2_move == "RGHT_ARM"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="Right Arm";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handR.set_arm_joint_angles(handR_cmd_angles);
    }else if (joints_2_move == "LEFTHAND"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="Left Hand";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handL.set_hand_joint_angles(handL_cmd_angles);
    }else if (joints_2_move == "RGHTHAND"){
      #ifdef DEBUG_RECIEVE_DATA
      logging_msg+="Right Hand";
      logging_msg+=" to ";
      logging_msg+=(command_data);
      #endif
      handR.set_hand_joint_angles(handR_cmd_angles);
    }else{
      Serial.println("INVALID MOVE COMMAND SENT");
      logging_msg+="INVALID MOVE COMMAND SENT";
    }
    
  }else if(command_type == "SPEED_ALLJOINTSMAX__"){
    logging_msg+="Going Super Fast";
    handL.set_arm_move_time(0.001);
    handR.set_arm_move_time(0.001);
    handL.set_hand_move_time(0.001);
    handR.set_hand_move_time(0.001);
  }else if(command_string.substring(0,9) == "GRIP_FORCE_"){ //If a grip force command
    double force;
    #ifdef DEBUG_RECIEVE_DATA
    logging_msg+=("Received force set command");
    #endif
    if(command_data == "__MAX__"){
      force = 10000;
    }else{
      force = command_data.toFloat();
    }
    String which_fingers = command_string.substring(9,19);
    if(which_fingers == "L_FINGER_"){
      handL.set_finger_only_forces(force);
    }else if(which_fingers == "L_THUMB__"){
      handL.set_thumb_only_forces(force);
    }else if(which_fingers == "L_HAND___"){
      handL.set_finger_and_thumb_forces(force);
    }else if(which_fingers == "R_FINGER_"){
      handR.set_finger_only_forces(force);
    }else if(which_fingers == "R_THUMB__"){
      handR.set_thumb_only_forces(force);
    }else if(which_fingers == "R_HAND___"){
      handR.set_finger_and_thumb_forces(force);
    }
  }
}

void print_robot_info() {
    double handR_angles[17];  // Pre-allocate arrays for the joint angles
    double handL_angles[17];
    double handR_forces[15];
    double handL_forces[15];
    char hand_joint_angles[300];  // Pre-allocate a buffer for the final result string (adjust size as needed)
    char handR_str[140];  // Pre-allocate buffers for each hand's joint angles string
    char handL_str[140];
    char handR_force_str[140];  // Pre-allocate buffers for each hand's joint angles string
    char handL_force_str[140];

    // Get joint angles for both hands
    handR.get_joint_angles(handR_angles);  // Populate handR_angles array
    handL.get_joint_angles(handL_angles);  // Populate handL_angles array

    //
    handR.get_finger_forces(handR_forces);  // Populate handR_angles array
    handL.get_finger_forces(handL_forces);  // Populate handL_angles array

    // Convert joint angles arrays to strings
    get_string_from_doubles(handR_angles, 17, handR_str, sizeof(handR_str));  // Convert handR angles to string
    get_string_from_doubles(handL_angles, 17, handL_str, sizeof(handL_str));  // Convert handL angles to string

    get_string_from_doubles(handR_forces, 15, handR_force_str, sizeof(handR_str));  // Convert handR angles to string 
    get_string_from_doubles(handL_forces, 15, handL_force_str, sizeof(handL_str));  // Convert handL angles to string

    // Concatenate the two strings with a comma separator
    snprintf(hand_joint_angles, sizeof(hand_joint_angles), "%s,%s", handR_str, handL_str);
    String print_string = String("MESSAGES:")+String(logging_msg)+String(">FORCES:")+String(handR_force_str)+","+String(handL_force_str)+String(">ANGLES:")+String(hand_joint_angles)+String(">END");
    // Print the final result
    Serial.println(print_string);
    //Serial.println("hi");
}

int loop_count = 0;
int power_high_count = 0;
int is_powered = 0;
double power_V;

int check_for_power(){
  power_V = analogRead(MAIN_POWER_SENSE_PIN)*(5.68/4096)*3.3;
  int last_power = is_powered;
  if(power_V>MAIN_POWER_ON_THRESH_V){
    if((power_high_count+1)>0){
      is_powered = true;
      //Serial.print("POWER ON:");
    }else{
      power_high_count++;
    }
  }else{
    if(power_high_count<=0){
      is_powered = false;
      //Serial.print("POWER OFF");
    }else{
      power_high_count--;
    }
  }
  if(last_power == false && is_powered == true){
    Serial.print("POWERED_ON!!!!!!!!!!!!!!!!!!!!!!");
    //power turned on (rising edge)
    //delay(10);
    setup_powered_items();
  }
  if(last_power == true && is_powered == false){
    Serial.print("POWERED_OFF!!!!!!!!!!!!!!!!!!!!!!");
    handR.release_all();
    handL.release_all();
    //delay(5000);
  }
      
  //Serial.println(power_V);
  //Serial.println(analogRead(MAIN_POWER_SENSE_PIN));
  return is_powered;
}

void loop() {
  check_for_power();
  //Serial.print("here 1");
  logging_msg=""; //use a string for all the messages this loop so that it can be appended to the data output and read by both vb and arduino
  
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  start_off_loop_time = millis();
  //Serial.print("here 2");
  //handR.testing_loop();
  //Serial.println("Right hand");
  handR.loop();
  check_for_power();
 // Serial.println("Left hand");
  handL.loop();
  check_for_power();
  //Serial.print("here 3");
  /*handL.set_joint_angle(14,-30,1,-1);
  delay(3000);
  handL.set_joint_angle(14,180,1,-1);
  delay(3000);*/
  if(elapsedTime>80000000 && state == 0){
    state = 1;
   // handR.set_joint_angle(16,0,1,-1);
    //handR.set_joint_angle(15,0,1,-1);
    /*handR.set_joint_angle(0,10,1,-1);
    handR.set_joint_angle(1,10,1,-1);
    handR.set_joint_angle(2,10,1,-1);
    handR.set_joint_angle(3,10,1,-1);
    handR.set_joint_angle(4,10,1,-1);
    handR.set_joint_angle(5,10,1,-1);
    handR.set_joint_angle(6,10,1,-1);
    handR.set_joint_angle(7,10,1,-1);
    
    handR.set_joint_angle(8,45,1,-1);
    handR.set_joint_angle(9,20,1,-1);
    handR.set_joint_angle(10,0,1,-1);*/
    
    /*
    handR.set_joint_angle(0,30,1,-1);
    handR.set_joint_angle(1,30,1,-1);
    handR.set_joint_angle(2,30,1,-1);
    handR.set_joint_angle(3,30,1,-1);
    handR.set_joint_angle(4,30,1,-1);
    handR.set_joint_angle(5,30,1,-1);
    handR.set_joint_angle(6,30,1,-1);
    handR.set_joint_angle(7,30,1,-1);*/
   // handL.set_joint_angle(11,20,1,-1);
   // handL.set_joint_angle(12,20,1,-1);
  //  handL.set_joint_angle(13,20,1,-1);
   // handL.set_joint_angle(14,80,1,-1);
    handL.set_joint_angle(15,20,1,-1);
//  handL.set_joint_angle(16,0,1,-1);
  }
  if(elapsedTime>20000 && state == 1){
    state = 2;
   // handL.set_arm_move_time(1);
   // handL.set_hand_move_time(1);
   // handL.set_joint_angle(11,-200,1,-1);
   // handL.set_joint_angle(12,-200,1,-1);
   // handL.set_joint_angle(13,-200,1,-1);
   // handL.set_joint_angle(14,100,1,-1);
    handL.set_joint_angle(15,-200,1,-1);
  //  handL.set_joint_angle(16,0,1,-1);

    /*
    handL.set_joint_angle(11,-40,1,-1);
    handL.set_joint_angle(12,180,1,-1);
    handL.set_joint_angle(13,30,1,-1);
    handL.set_joint_angle(14,80,1,-1);
    handL.set_joint_angle(15,30,1,-1);
    handL.set_joint_angle(16,150,1,-1);*/
   // handL.set_joint_angle(11,1000,1,-1);
   // handL.set_joint_angle(12,1000,1,-1);
    //handL.set_joint_angle(13,1000,1,-1);
  //  handL.set_joint_angle(14,1000,1,-1);
   // handL.set_joint_angle(15,1000,1,-1);
    //handL.set_joint_angle(16,1000,1,-1);
    /*
    handR.set_joint_angle(8,20,1,-1);
    handR.set_joint_angle(9,10,1,-1);
    handR.set_joint_angle(10,0,1,-1);*/
    /*
    handR.set_joint_angle(0,10,1,-1);
    handR.set_joint_angle(1,10,1,-1);
    handR.set_joint_angle(2,10,1,-1);
    handR.set_joint_angle(3,10,1,-1);
    handR.set_joint_angle(4,10,1,-1);
    handR.set_joint_angle(5,10,1,-1);
    handR.set_joint_angle(6,10,1,-1);
    handR.set_joint_angle(7,10,1,-1);
    */
    /*
    //handR.set_joint_angle(16,90,1,-1);
    handR.set_joint_angle(1,10,1,-1);
    handR.set_joint_angle(2,50,1,-1);
    handR.set_joint_angle(3,10,1,-1);
    handR.set_joint_angle(4,50,1,-1);
    handR.set_joint_angle(5,10,1,-1);
    handR.set_joint_angle(6,50,1,-1);
    handR.set_joint_angle(7,10,1,-1);*/
  }
  if(elapsedTime>12000000 && state == 2){
    state = 3;
    //handR.set_joint_angle(16,180,1,-1);
    
    handR.set_joint_angle(7,70,1,-1);
    handR.set_joint_angle(6,70,1,-1);
    handR.set_joint_angle(5,80,1,-1);
    handR.set_joint_angle(4,80,1,-1);
    handR.set_joint_angle(3,80,1,-1);
    handR.set_joint_angle(2,80,1,-1);
    handR.set_joint_angle(1,80,1,-1);
    handR.set_joint_angle(0,80,1,-1);
  }
  if(elapsedTime>28000 && state == 3){
    state = 4;
    /*
    handR.set_joint_angle(0,10,1,-1);
    handR.set_joint_angle(2,10,1,-1);
    handR.set_joint_angle(4,10,1,-1);
    handR.set_joint_angle(6,10,1,-1);*/
  }
  if(elapsedTime>34000000000 && state == 4){
    state = 5;
    handR.set_joint_angle(16,135,1,-1);
  }
  if(elapsedTime>400000000 && state == 5){
    state = 6;
    handR.set_joint_angle(16,110,1,-1);
  }
  if(elapsedTime>46000 && state == 6){
    state = 7;
    handR.set_joint_angle(16,40,1,-1);
  }
  if(elapsedTime>52000 && state == 7){
    state = 8;
    handR.set_joint_angle(16,120,1,-1);
  }
  if(elapsedTime>58000 && state == 8){
    state = 9;
    handR.set_joint_angle(16,50,1,-1);
  }
  if(elapsedTime>64000 && state == 9){
    state = 10;
    handR.set_joint_angle(16,130,1,-1);
  }
  if(elapsedTime>70000 && state == 10){
    state = 11;
    handR.set_joint_angle(16,60,1,-1);
  }
  if(elapsedTime>76000 && state == 11){
    state = 12;
    handR.set_joint_angle(16,140,1,-1);
  }
  if(elapsedTime>82000 && state == 12){
    state = 13;
    handR.set_joint_angle(16,70,1,-1);
  }
  if(elapsedTime>88000 && state == 13){
    state = 14;
    handR.set_joint_angle(16,150,1,-1);
  }
  if(elapsedTime>94000 && state == 14){
    state = 15;
    handR.set_joint_angle(16,80,1,-1);
  }
  if(elapsedTime>100000 && state == 15){
    state = 16;
    handR.set_joint_angle(16,160,1,-1);
  }
  if(elapsedTime>106000 && state == 16){
    state = 17;
    handR.set_joint_angle(16,85,1,-1);
  }
  if(elapsedTime>112000 && state == 17){
    state = 18;
    handR.set_joint_angle(16,170,1,-1);
  }
  if(elapsedTime>118000 && state == 18){
    state = 19;
    handR.set_joint_angle(16,95,1,-1);
  }
  #ifdef DEBUG_BIG_SERVOS
  //Serial.println(motor_test_values);
  #endif
    /*hand1.set_joint_angle(0,60,1,-1);
    hand1.set_joint_angle(2,60,1,-1);
    hand1.set_joint_angle(4,60,1,-1);
    hand1.set_joint_angle(6,60,1,-1);
    //hand1.set_joint_angle(1,80,1,-1);
    hand1.set_joint_angle(3,85,1,-1);
    //hand1.set_joint_angle(5,80,1,-1);
    //hand1.set_joint_angle(7,80,1,-1);
    //hand1.set_joint_angle(1,45,1,-1);
    //hand1.set_joint_angle(8,70,1,-1);
    //hand1.set_joint_angle(9,45,0.6,-1);*/
    
  //}
  /*if(elapsedTime>10000 && state == 1){
    state = 2;
    handR.set_joint_angle(16,-100,1,-1);
    handR.set_joint_angle(15,-100,1,-1);
    handR.set_joint_angle(14,-100,1,-1);
    handR.set_joint_angle(12,-100,1,-1);
    handR.set_joint_angle(11,-100,1,-1);
  }
  if(elapsedTime>20000 && state == 2){
    state = 3;
    handR.set_joint_angle(16,1000,1,-1);
    handR.set_joint_angle(15,1000,1,-1);
    handR.set_joint_angle(14,1000,1,-1);
    handR.set_joint_angle(12,1000,1,-1);
    handR.set_joint_angle(11,1000,1,-1);
  }*/
  /*
  if(elapsedTime>10000 && state == 1){
    state = 2;
    hand1.set_joint_angle(0,10,1,-1);
    hand1.set_joint_angle(2,10,1,-1);
    hand1.set_joint_angle(4,10,1,-1);
    hand1.set_joint_angle(6,10,1,-1);
    //hand1.set_joint_angle(1,40,1,-1); //INDEX FINGER DOESNT go back at all
    hand1.set_joint_angle(3,50,1,-1); //Middle finger
    //hand1.set_joint_angle(5,40,1,-1); //Ring finger is good, bit of friction when turning
    //hand1.set_joint_angle(7,40,1,-1); //pinky is good but a lot of friction, even on unwind
    //hand1.set_joint_angle(8,20,1,-1);
    //hand1.set_joint_angle(9,70,0.6,-1);
  }*/
  //Pull the finger
  /*
  if(elapsedTime>8000 && state == 0){
    state = 1;
    hand1.set_joint_angle(0,80,1,-1);
    hand1.set_joint_angle(1,80,1,-1);
    hand1.set_joint_angle(2,80,1,-1);
    hand1.set_joint_angle(3,80,1,-1);
    hand1.set_joint_angle(6,80,1,-1);
    hand1.set_joint_angle(7,80,1,-1);
  }
  if(elapsedTime>8700 && state == 1){
    state = 2;
    hand1.set_joint_angle(8,90,1,-1);
    hand1.set_joint_angle(9,80,1,-1);
  }
  */
  if(new_data){
    new_data=false;
    read_stuff+=("rec");
    read_stuff+=String(Recieved_Serial_Chars);
    execute_dt_command();
  }
  //Serial.println("End of loop");
  //Serial.println(millis());
  //logging_msg+=read_stuff;
  
  if(millis()-last_print_time > INFO_PRINT_INTERVAL_MS){ //dont print every loop
    //move based on recieved command
    recvWithEndMarker();
    last_print_time = millis();
    print_robot_info();
  }
  /*if(loop_count>100){
    Serial.println(millis());
    loop_count =0;
  }*/
  //loop_count++;
  //delay(500);
}
