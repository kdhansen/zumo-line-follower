// Line follower for Zumo 32U4
// 
// See video: https://youtu.be/pxXkpFztGrk
//
// First step: Press A to start calibrating the line sensor. Do this by draging
//             the robot across the black line. Check that all sensors return
//             a full reading when positioned over the line.
// Next step:  Press A to complete the calibration. The robot starts in PAUSE
//             mode. Press A to launch it. Use B and C to adjust desired forward
//             velocity. Press A to pause.
//
// Copyright 2021 (C) Karl D. Hansen (kdh@es.aau.dk)
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA button_a;
Zumo32U4ButtonB button_b;
Zumo32U4ButtonC button_c;
Zumo32U4LineSensors line_sensors;
const uint8_t NUM_SENSORS = 5;
unsigned int line_sensor_values[NUM_SENSORS] = {0, 0, 0, 0, 0};

Zumo32U4Motors motors;
bool pause = true;

/////////// Change these values to modify behaviour /////////
float desired_lin_speed = 100.0;
float rot_gain = 0.1; // Tip: This gain could be more agressive.
/////////////////////////////////////////////////////////////

// Loads 1-7 bar characters to show linereadings. The 0-bars is just a space,
// ' ', and the 8-bar is the character 0xFF (or 255), they do not need to be
// loaded.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  for (int i = 0; i < 7; ++i) {
    lcd.loadCustomCharacter(levels + i, i);
  }
}

// Prints a character representing signal strength as a stacked-bar, with
// 0-8 bars.
void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  static const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, '\xff'};
  lcd.print(barChars[height]);
}

// Prints the five line readings to the display. The readings are printed to
// the first five characters on the first line (this can be changed in the
// code).
void printReadingsToLCD(unsigned int* readings)
{
  lcd.gotoXY(0, 0); // Modify this, if you want to change the position.
  for (uint8_t i = 0; i < 5; i++)
  {
    uint8_t barHeight = map(readings[i], 0, 1000, 0, 8);
    printBar(barHeight);
  }
}

// Set the linear and rotational speeds. Min and max is [-400, 400]. If the
// limits are exceeded, the speed is constrained so that the robot prioritizes
// the rotation.
void setLinRotSpeeds(float lin_speed, float rot_speed) {
  float left_speed = lin_speed - rot_speed;
  float right_speed = lin_speed + rot_speed;
  // Now, constrain the speeds on both tracks, so that angular velocity is
  // prioritized.
  if (left_speed > 400) {
    right_speed = right_speed - (left_speed - 400);
    if (right_speed < -400) {right_speed = -400;}
    left_speed = 400;
  } else if (right_speed > 400) {
    left_speed = left_speed - (right_speed - 400);
    if (left_speed < -400) {left_speed = -400;}
    right_speed = 400;
  } else if (left_speed < -400) {
    right_speed = right_speed - (left_speed + 400);
    if (right_speed > 400) {right_speed = 400;}
    left_speed = -400;
  } else if (right_speed < -400) {
    left_speed = left_speed - (right_speed + 400);
    if (left_speed > 400) {left_speed = 400;}
    right_speed = -400;
  }

  motors.setSpeeds(left_speed, right_speed);
}

void setup() {
  // Initialize sensors
  line_sensors.initFiveSensors();
  // Load bars to show sensor values on display
  loadCustomCharacters();

  // Wait for button A to be pressed and released.
  lcd.clear();
  lcd.print(F("Press A"));
  lcd.gotoXY(0, 1);
  lcd.print(F("to calib"));
  button_a.waitForButton();
  
  lcd.clear();
  lcd.gotoXY(0,1);
  lcd.print(F("A->Cont."));
  while (!button_a.getSingleDebouncedPress()) {
    line_sensors.calibrate();
    line_sensors.readCalibrated(line_sensor_values);
    printReadingsToLCD(line_sensor_values);
  }
}

void loop() {
  int line_position = line_sensors.readLine(line_sensor_values);
  printReadingsToLCD(line_sensor_values);

  // Negating the line position, because the positive rotation is defined
  // counter-clockwise and the line position is clock-wise.
  float rot_speed = rot_gain * -(line_position - 2000);
  float lin_speed = desired_lin_speed;
  if (pause) {
    setLinRotSpeeds(0., 0.);
  } else {
    setLinRotSpeeds(lin_speed, rot_speed);
  }
  
  if (pause) {
    lcd.gotoXY(0,1);
    lcd.print(F(" Paused "));
  } else {
    lcd.gotoXY(0,1);
    lcd.print(F("        "));
    lcd.gotoXY(0,1);
    lcd.print((int)lin_speed);
    lcd.print(' ');
    lcd.print((int)rot_speed);
  }

  // If button A is pressed, pause.
  if (button_a.getSingleDebouncedPress())
  {
    pause = !pause;
  }

  // If button B is pressed, decrease speed.
  if (button_b.getSingleDebouncedPress())
  {
    desired_lin_speed *= 0.9;
  }

  // If button C is pressed, increase speed.
  if (button_c.getSingleDebouncedPress())
  {
    desired_lin_speed *= 1.1;
  }
}

