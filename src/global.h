

String sfVersion = "1.5.2 (BETA)";
String spVersion = "1.5.2 (BETA)";

//---------------------------------------------------------------------------------------
// Define PWM pin

int pinPWM[4] = {14, 15, 12, 13};
int red = 0, green = 0, blue = 0;

int checkEepromOpt = 0;
int multiRangerDone = 0;

//---------------------------------------------------------------------------------------
// Define Led pin
#define blueLed 0
#define redLed 2
#define greenLed 16

int greenLedPin = 16;
int redLedPin = 2;
int blueLedPin = 0;

int targetOtoDf = 500;

int16_t Trim_Roll_Bs = 0;
int16_t Trim_Pitch_Bs = 0;
int16_t Trim_Yaw_Bs = 0;

int Trim_DF = 1;
int16_t Trim_Roll_DF = 0;
int16_t Trim_Pitch_DF = 0;
int16_t Trim_Yaw_DF = 0;

float roll_Extra = -0.00;
float pitch_Extra = -0.0;

int buzzerControl = 0;

#define magCalibraiton 1
#define gyro_Accel_Calibraiton 0

//#define AP_Mode
//#define STA_Mode

//---------------------------------------------------------------------------------------
// Define for remote method

//#define EduRXY

//#define REMOTE_XY_REMOTE

//#define BLYNK

//#define PROCESSING_REMOTE

#if defined(PROCESSING_REMOTE) == 1
#define PROCESSING_SINGLE
#endif

//#define PROCESSING_SINGLE
//#define PROCESSING_MULTIPLE

//#define PPM_REMOTE

//#define MQTT
#if defined(MQTT) == 1
#define BLOCKLY
#endif

#define BLOCKLY

//#define REMOTE_WEB_APP // Phone control

//#define FREECONTROL // Phone control

#if defined(REMOTE_XY_OWN) == 0
#define REMOTE_XY_REMOTE
#endif

#if defined(REMOTE_XY_OWN) == 1
#define STANDALONE
#endif

//---------------------------------------------------------------------------------------
// Define modules

//#define NeoPixel
#define bme280
#define NeoPixel
#define MULTI_RANGER // auto recognazation
//#define HandControl //
#define AntiCollision //

#define vl53l0x // auto recognazation

///#define LONG_RANGE
#define SHORT_RANGE
//#define HIGH_ACCURACY
//#define HIGH_SPEED

#define opticalFlow // auto recognazation
//#define AUTOFLY
//#define  otoMission

#define lps
//---------------------------------------------------------------------------------------

// Define trim and Set point values
int SetPoint[3] = {0}, _SetPoint[3] = {0}; // -1750 : 1750
int Trim_Roll = 0, Trim_Pitch = 0;         // -1750 : 1750
int Trim_Yaw = 0;                          // -8000 : 8000

float throttle = 0;
int SetPointOpt[2] = {0};
int SetPointOptBlock[2] = {0};
int SetPointOptRemote[2] = {0};
int SetPointOptMain[2] = {0};
// Define auto altitude (mm)
int targetOto = 500;
int targetOtoOld = 500;
//---------------------------------------------------------------------------------------
// Define fly modes

int flyMode_1 = 0, flyMode_2 = 0, flyMode_3 = 0;

//---------------------------------------------------------------------------------------
// Define controls

int firstStage = 0;
int armControl = 0;
int landingOff = 1;
int batteryCount = 0;
int vl5310xControl = 0;
int bme280Control = 0;
int multiRangerControl = 0;
int opticalFlowControl = 0;
int lpsControl = 0;
int throttleControl = 0;
int yawControl = 0;
int stopFlightControl = 1;

boolean otoHover = false;

boolean autoOpt = false;
boolean _autoOpt = false;
//---------------------------------------------------------------------------------------
// Define RX Value

float RX_throttle = 0;                       // 0 : 700
float RX_roll = 0, RX_pitch = 0, RX_yaw = 0; // -100 : + 100
float yaw_Control = 0;

float RX_roll_Multi = 0, RX_pitch_Multi = 0; // -100 : + 100
//---------------------------------------------------------------------------------------
// Define other variables

int controlMethod = 0;

float DistanceX1 = 1200;
float DistanceY1 = 1200;
float DistanceX0 = 1200;
float DistanceY0 = 1200;

float magOffsetMotorFnl[3] = {0};
float magOffsetMotorCount = 0;
float magOffsetMotor[3] = {0};
float sumMagOffsetMotor[3] = {0};
float magOffsetMotorCount2 = 0;
float magOffsetMotor2[3] = {0};
float sumMagOffsetMotor2[3] = {0};

unsigned long calMillis = 0; // will store last time LED was updated

// constants won't change:
const long calInterval = 1000;

float errorDegrees = 0;
int motorMax = 700;

#define weightOfDrone 0.9

float xOptical = 0;
float yOptical = 0;
float crcOpt = 0;
float crcOptQuality = 0;

String inString = ""; // string to hold input
int currentCommand = 0;
int currentColor = 0;
int currentPid = 0;
int currentValue = 0;

int boundRate = 1000;
double aggKi;
double aggKp;

boolean sendMpu = false;

int calAHRS = 0;
int calTest = 1;
int takeTimeCal = 0;
unsigned long calMillisPre = 0;

char getMode = 0;
int i;

float degree[4] = {0};

float altitudeBmp = 0;
float temperatureBmp = 0;

float rate[3] = {0};
float rateRadian[3] = {0};
float attitude[3] = {0};
float attitudeRadian[3] = {0};
int _Rate[3] = {0};
int16_t _Angle[3] = {0};
long ctr_value[3];

int16_t otoNewSet = 0;

float Alpha = 0.75, a = 0.7585; // Trim_Roll = -150, Trim_Pitch = 100,

int dt, time_prev = 0, time_now, dt2, time_prev2 = 0;

float yaw_rate = 0, motorFL = 0, motorFR = 0, motorRL = 0, motorRR = 0;
