#include "global.h"
#include "Parameter.h"
#include <EEPROM.h>
#include <Wire.h>
#include "pid.h"
#include "AHRS.h"
#include "RemoteXY_Control.h"
#include "WEP_APP.h"
#include "block.h"
#include "NeoPixel.h"
#include "VL53L0X.h"
#include "outputs.h"
#include "bme280.h"

ESPCOPTER esp;
VL53L0X sensor;

PID oto;
PID xOpt;
PID yOpt;

PID xMulti;
PID yMulti;

PID xMultiM;
PID yMultiM;

AHRS ahrs;

BLOCK blockly;

PID roll;
PID pitch;
PID yaw;

#include "autoOpt.h"
#include "vl5310x.h"
#include "optical.h"
#include "otoMission.h"
#include "multiRanger.h"
#include "PROCESSING.h"
#include "FlightControl.h"
#include "wblockly.h"
#include "setup.h"
#include "loop.h"
