/*
  Koffiebrander.cpp - Library for controlling a coffee roaster.
  Created by Rein Velt, 2016.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Koffiebrander.h"
#include <max6675.h>
#include <Nextion.h>
#include <INextionColourable.h>
#include <NextionPage.h>
#include <NextionText.h>
#include <NextionWaveform.h>
#include <NextionProgressBar.h>
#include <NextionButton.h>
#include <Stepper.h>
//#include <PID_v1.h>


//TODO:
//- implement load/release
//- implement pid regulator
//- connect nexion display
//- fastgpio library to replace digitalWrite


//Define Variables we'll be connecting to
//double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//int _state;
//Serial2 nextionDisplay;
//SoftwareSerial nextionDisplay(15, 14);
Nextion nex(Serial3);
NextionPage pgStart(nex, 0, 0, "page0");
NextionText textTitle(nex, 0, 2, "t0");
NextionText textTime(nex, 0, 2, "t1");
NextionText textHeaterStatus(nex, 0, 3, "t2");
NextionText textCoolerStatus(nex, 0, 4, "t3");
NextionText textTemperatureTarget(nex, 0, 5, "t4");
NextionText textTemperatureHeater(nex, 0, 6, "t5");
NextionText textTemperatureBeans(nex, 0, 7, "t6");
NextionWaveform waveform(nex, 0, 8, "s0");
NextionProgressBar progressBar(nex, 0, 9, "j0");
NextionButton button(nex, 0, 10, "b0");
Stepper motor(200, PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4);
int temperaturePointArray[_POINTCOUNTMAX] = {
  25,  75,  150,  170,  190,  200,  210,  220,  225,  200  , 50, 0
};
int timePointArray[_POINTCOUNTMAX] =        {
  0,   60,  120, 180, 360, 400, 500, 600, 700, 1100  , 1200, 1300
};

bool buttonPressed = false;
int updateSequenceCounter = 0;
long temperatureSensorTimer = 0;
long cycleTimer = 0;
long lastCycle = 0;
long stateRepeatCounter = 0;
long lastUpdate = 0;
double pidSetPoint, pidInput, pidOutput;
double consKp = 1, consKi = 0.05, consKd = 0.25;
PID pidController(&pidInput, &pidOutput, &pidSetPoint, consKp, consKi, consKd, DIRECT);
MAX6675 thermocoupleHeater(PIN_THERMOCOUPLE_SCK, PIN_THERMOCOUPLE_CS_HEATER, PIN_THERMOCOUPLE_SO);
MAX6675 thermocoupleBeans(PIN_THERMOCOUPLE_SCK, PIN_THERMOCOUPLE_CS_BEANS, PIN_THERMOCOUPLE_SO);

Koffiebrander::Koffiebrander()
{
  _state = 0;
  _heaterOn = false;
  _coolerOn = false;
  _fanOn = false;
  _motorPosition = 0;
  _pointCount = 10;
  _timeCode = 0;
  _timeCodeOffset = 0;
  motor.setSpeed(60);
  _temperaturePointArray = temperaturePointArray;
  _timePointArray = timePointArray;
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_COOLER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_DRUM_OPEN, OUTPUT);
  pinMode(PIN_DRUM_RELEASE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

}
void Koffiebrander::_nextionCallback(NextionEventType type, INextionTouchable *widget)
{
  if (type == NEX_EVENT_PUSH)
  {
    buttonPressed = true;
  }
  else if (type == NEX_EVENT_POP)
  {
    buttonPressed = true;
  }
}


bool Koffiebrander::_buttonPressed()
{
  nex.poll();
  _setLed(true);
  button.setText("PRESS");
  bool pressed = buttonPressed;
  buttonPressed = false;

  pressed = true;
  return pressed;
}


void Koffiebrander::init()
{
  _setState(STATE_INIT);
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
  _setLed(false);
  Serial.begin(115200);
  Serial3.begin(115200);
  //nextionDisplay.begin(9600);
  nex.init();
  Serial.println("#\tKoffiebrander V1.0 by Rein Velt - Theo's Mechanische Aap - 2016");
  Serial.println("#Th\tTb\tTn");
  //button.attachCallback(&_nextionCallback);;
  button.setText("INIT");
  _updateDisplay();

}





//getters and setters - gettters and setters are a bit overkill and bloat-instruction but they
//give you an opportunity to hooks things...not required when using fastgpio library (todo)
void Koffiebrander::_setRelay(int relayPort, boolean relayOn)
{
  //ports are inverted;
  digitalWrite(relayPort, !relayOn);
}

int Koffiebrander::_setLed(bool ledstatus)
{
  digitalWrite(PIN_LED, ledstatus);
}

void Koffiebrander::_updateTemperature()
{
  if (millis() - temperatureSensorTimer > 1000)
  {
    _temperatureHeater = int(thermocoupleHeater.readCelsius());
    _temperatureBeans = int(thermocoupleBeans.readCelsius());
    temperatureSensorTimer = millis();
  }
}


int Koffiebrander::_getHeaterTemperature() {
  _updateTemperature();
  return _temperatureHeater;
  //return _getTemperature(PIN_TEMPERATURE_HEATER);
}

int Koffiebrander::_getBeansTemperature() {
  _updateTemperature();
  return _temperatureBeans;
  //return _getTemperature(PIN_TEMPERATURE_BEANS);
}

void Koffiebrander::_setHeater(boolean heaterOn) {
  _heaterOn = heaterOn;
  _setRelay(PIN_HEATER, _heaterOn);
}

boolean Koffiebrander::_getHeater() {
  return _heaterOn;
}

void Koffiebrander::_setCooler(boolean coolerOn) {
  _coolerOn = coolerOn;
  _setRelay(PIN_COOLER, _coolerOn);
}

boolean Koffiebrander::_getCooler()
{
  return _coolerOn;
}

void Koffiebrander::_setFan(boolean fanOn) {
  _fanOn = fanOn;
  _setRelay(PIN_FAN, _fanOn);
}

boolean Koffiebrander::_getFan() {
  return _fanOn;
}

void Koffiebrander::_setMotorPosition(int motorPosition) {
  _motorPosition = motorPosition;
}

int Koffiebrander::_getMotorPosition() {
  return _motorPosition;
}

void Koffiebrander::_setState(int state) {
  _state = state;
}

int Koffiebrander::_getState() {
  return _state;
}


void Koffiebrander::_setTemperaturePoint(int index, int temperature) {
}
int Koffiebrander::_getTemperaturePoint(int index) {
}
int Koffiebrander::_getPointCount() {
  return _pointCount;
}
void Koffiebrander::_setTimePoint(int index, int timecode) {
}
int Koffiebrander::_getTimePoint(int index) {
}

int Koffiebrander::_getTemperatureByTimecode(int secondsSinceRoastStarted)
{
  int temperatureBefore = 0;
  int temperatureAfter = 0;
  int timecodeBefore = 0;
  int timecodeAfter = 0;
  int newTemperature = 0;

  for (int i = 0; i < 9; i++)
  {
    if (_timePointArray[i - 1] == secondsSinceRoastStarted)
    {
      //we've found an exact point in our tabel
      newTemperature = _temperaturePointArray[i - 1];
      break;
    }
    else
    {
      //we are between two points.....interpolate the new target temperature
      if (_timePointArray[i] < secondsSinceRoastStarted && _timePointArray[i + 1] > secondsSinceRoastStarted)
      {
        temperatureBefore = _temperaturePointArray[i];
        temperatureAfter = _temperaturePointArray[i + 1];
        timecodeBefore = _timePointArray[i];
        timecodeAfter = _timePointArray[i + 1];
        newTemperature = map(secondsSinceRoastStarted, timecodeBefore, timecodeAfter, temperatureBefore, temperatureAfter);
        //newTemperature=_temperaturePointArray[i];
        break;
      }
    }
  }
  return newTemperature;
}



void Koffiebrander::_updateDisplay()
{
  updateSequenceCounter++;
  int timecode = (millis() - _timeCodeOffset) / 1000;
  int heaterTemperature = _getHeaterTemperature();
  int beansTemperature = _getBeansTemperature();
  int newTemperature = _getTemperatureByTimecode(timecode);
  char timeString[10];
  char temperatureBeansString[10];
  char temperatureHeaterString[10];
  char temperatureTargetString[10];
  char timeRemainString[10];
  int timeRemain = (timePointArray[_POINTCOUNTMAX - 1] - timecode) * 1000;
  int timePerStep = timeRemain / 320;
  int timeUpdate = lastUpdate + timePerStep;
  sprintf(timeString, "t=%ds",  -1 * (timePointArray[_POINTCOUNTMAX - 1] - timecode));
  sprintf(temperatureTargetString, "Tn=%d^C", newTemperature);
  sprintf(temperatureHeaterString, "Th=%d^C", heaterTemperature);
  sprintf(temperatureBeansString, "Tb=%d^C", beansTemperature);
  nex.sendCommand("ref_stop");
  progressBar.setValue(map(timecode, 0, timePointArray[_POINTCOUNTMAX - 1], 0, 100));
  waveform.addValue(0, newTemperature / 3);
  waveform.addValue(1, heaterTemperature / 3);
  waveform.addValue(2, beansTemperature / 3);
  textTemperatureBeans.setText(temperatureBeansString);
  textTemperatureHeater.setText(temperatureHeaterString);
  textTemperatureTarget.setText(temperatureTargetString);
  textTime.setText(timeString);
  nex.sendCommand("ref_star");
  lastUpdate = millis();

}

//state related functions
void Koffiebrander::_runStateError() {
  //all systems safe
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
}


void Koffiebrander::_runStateInit() {
  //all systems safe
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
  _setState(STATE_IDLE);
}

void Koffiebrander::_runStateIdle() {
  //do notrhing until keys are pressed or serial command is sent
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
  _setLed(true);
  if (_buttonPressed())
  {
    _setState(STATE_LOAD);
    buttonPressed = false;
  }
}

void Koffiebrander::_runStateLoad() {
  //load the beans
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
  _setLed(true);
  if (_buttonPressed())
  {
    _setState(STATE_START);
    buttonPressed = false;
  }
}

void Koffiebrander::_runStateStart() {
  //start burning process (pre-run)
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setLed(false);
  _setMotorPosition(0);
  _timeCodeOffset = millis();
  _setState(STATE_RUN);
  pidSetPoint = _getBeansTemperature();
  pidController.SetMode(AUTOMATIC);
}

void Koffiebrander::_runStateRun() {
  //burn
  _timeCode = (millis() - _timeCodeOffset);
  int heaterTemperature = _getHeaterTemperature();
  int beansTemperature = _getBeansTemperature();
  int newTemperature = _getTemperatureByTimecode(_timeCode / 1000);
  pidInput = beansTemperature;
  pidSetPoint = newTemperature;
  pidController.Compute();
  newTemperature = pidOutput;
  //newTemperature=pidOutput;
  motor.step(1);
  if (beansTemperature == newTemperature)
  {
    //temperature ok
    _setHeater(false);
    _setCooler(false);
    _setFan(true);
    _setMotorPosition(0);
    _setLed(false);

    nex.sendCommand("ref_stop");
    textHeaterStatus.setText("HEATER OFF");
    textHeaterStatus.setBackgroundColour(NEX_COL_BLACK);
    textCoolerStatus.setText("COOLER OFF");
    textCoolerStatus.setBackgroundColour(NEX_COL_BLACK);
    nex.sendCommand("ref_star");
  }
  else
  {
    if (beansTemperature < newTemperature)
    {
      //temperature must go up
      _setHeater(true);
      _setCooler(false);
      _setFan(true);
      _setMotorPosition(0);
      _setLed(false);
      nex.sendCommand("ref_stop");
      textHeaterStatus.setText("HEATER ON");
      textHeaterStatus.setBackgroundColour(NEX_COL_RED);
      textCoolerStatus.setText("COOLER OFF");
      textCoolerStatus.setBackgroundColour(NEX_COL_BLACK);
      nex.sendCommand("ref_star");
    }
    else
    {
      //temperature must go down
      _setHeater(false);
      _setCooler(true);
      _setFan(true);
      _setMotorPosition(0);
      _setLed(false);
      nex.sendCommand("ref_stop");
      textHeaterStatus.setText("HEATER OFF");
      textHeaterStatus.setBackgroundColour(NEX_COL_BLACK);
      textCoolerStatus.setText("COOLER ON");
      textCoolerStatus.setBackgroundColour(NEX_COL_BLUE);
      nex.sendCommand("ref_star");
    }
    _updateDisplay();
    Serial.print(_getHeaterTemperature());
    Serial.print("\t");
    Serial.print(_getBeansTemperature());
    Serial.print("\t");
    Serial.print(_getTemperatureByTimecode(_timeCode / 1000));
    Serial.println();
  }

  if (_timeCode / 1000 > timePointArray[_POINTCOUNTMAX - 1])
  {
    _setState(STATE_END);
  }
}

void Koffiebrander::_runStateEnd() {
  //end burning
  //post-run
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
  if (_buttonPressed())
  {
    _setState(STATE_RELEASE);
    buttonPressed = false;
  }
}

void Koffiebrander::_runStateRelease() {
  //release trhe beans
  _setHeater(false);
  _setCooler(false);
  _setFan(false);
  _setMotorPosition(0);
  _setLed(true);
  _setState(STATE_IDLE);
}


//state handler
int Koffiebrander::run()
{
  int state = _getState();
  cycleTimer = millis() - lastCycle;
  lastCycle = millis();
  nex.poll();
  stateRepeatCounter++;
  switch (state)
  {
    case STATE_INIT:
      _runStateInit();
      break;
    case STATE_IDLE:
      _runStateIdle();
      break;
    case STATE_LOAD:
      _runStateLoad();
      break;
    case STATE_START:
      _runStateStart();
      break;
    case STATE_RUN:
      _runStateRun();
      break;
    case STATE_END:
      _runStateEnd();
      break;
    case STATE_RELEASE:
      _runStateRelease();
      break;
    case STATE_ERROR:
      _runStateError();
      break;
    default: _runStateIdle(); break;
  }

  int newState = _getState();
  if (state != newState)
  {
    stateRepeatCounter = 0;;
    //state changed
    switch (newState)
    {
      case -1: button.setText("ERROR"); break;
      case 0: button.setText("INIT"); break;
      case 1: button.setText("READY"); break;
      case 2: button.setText("LOAD"); break;
      case 3: button.setText("START"); break;
      case 4: button.setText("RUN"); break;
      case 5: button.setText("END"); break;
      case 6: button.setText("RELEASE"); break;
    }
  }
}









