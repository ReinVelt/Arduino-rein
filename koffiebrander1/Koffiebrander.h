/*
  Koffiebrander.h - Library for controlling a coffee roaster.
  Created by Rein Velt, 2016.
  Released into the public domain.
*/
#ifndef Koffiebrander_h
#define Koffiebrander_h
#include "Arduino.h"
#include "Koffiebrander.h"
#include <Nextion.h>
#include <INextionColourable.h>
#include <NextionPage.h>
#include <NextionText.h>
#include <NextionWaveform.h>
#include <NextionProgressBar.h>
#include <NextionButton.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#define PIN_RX                       0
#define PIN_TX                       1
#define PIN_IRQ1                     2
#define PIN_IRQ2                     3
#define PIN_HEATER                   4
#define PIN_COOLER                   5
#define PIN_FAN                      6
#define PIN_MOTOR                    7
#define PIN_DRUM_OPEN                8
#define PIN_DRUM_RELEASE             9
#define PIN_LED                     13
#define PIN_MOTOR1                  22
#define PIN_MOTOR2                  23
#define PIN_MOTOR3                  24
#define PIN_MOTOR4                  25
#define PIN_TEMPERATURE_HEATER      A0
#define PIN_TEMPERATURE_BEANS       A1
#define PIN_I2C_DATA                A4
#define PIN_I2C_CLK                 A5
#define PIN_THERMOCOUPLE_CS_HEATER  53
#define PIN_THERMOCOUPLE_CS_BEANS   54
#define PIN_THERMOCOUPLE_CS_x1      55
#define PIN_THERMOCOUPLE_CS_x2      56
#define PIN_THERMOCOUPLE_CS_x3      57
#define PIN_THERMOCOUPLE_SCK        52
#define PIN_THERMOCOUPLE_SO         50

#define STATE_ERROR    -1
#define STATE_INIT      0
#define STATE_IDLE      1
#define STATE_LOAD      2
#define STATE_START     3
#define STATE_RUN       4
#define STATE_END       5
#define STATE_RELEASE   6

#define _POINTCOUNTMAX 12
//extern Koffiebrander koffiebrander;

class Koffiebrander
{


  public:

    Koffiebrander();
    void init();
    int run();
  private:
    int*     _temperaturePointArray;
    int*     _timePointArray;
    int      _temperatureHeater;
    int      _temperatureBeans;
    int     _state;
    int     _pointCount;
    unsigned long    _timeCode;
    unsigned long    _timeCodeOffset;
    boolean _heaterOn;
    boolean _coolerOn;
    boolean _fanOn;
    int     _motorPosition;
    void    _runStateError();
    void    _runStateInit();
    void    _runStateIdle();
    void    _runStateLoad();
    void    _runStateStart();
    void    _runStateRun();
    void    _runStateEnd();
    void    _runStateRelease();
    void    _updateTemperature();
    void    _setRelay(int relayPort, bool switchedOn);
    int     _getHeaterTemperature();
    int     _getBeansTemperature();
    void    _setHeater(boolean heaterOn);
    boolean _getHeater();
    void    _setCooler(boolean coolerOn);
    boolean _getCooler();
    void    _setFan(boolean fanOn);
    boolean _getFan();
    void    _setMotorPosition(int position);
    int     _getMotorPosition();
    int     _setLed(bool ledstatus);
    void    _setState(int state);
    int     _getState();
    void    _setTemperaturePoint(int index, int temperature);
    int     _getTemperaturePoint(int index);
    int     _getPointCount();
    void    _setTimePoint(int index, int timecode);
    int     _getTimePoint(int index);
    int     _getTemperatureByTimecode(int timecode);
    void    _updateDisplay();
    bool    _buttonPressed();
    static void    _nextionCallback(NextionEventType type, INextionTouchable *widget);


};


#endif
