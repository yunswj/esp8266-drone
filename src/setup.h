
// Include pf Arduino

#include "Arduino.h"

void scanShields()
{
  Wire.begin();
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");
  delay(1500);
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address);
      Serial.println("  !");
      if (address == 22 || address == 41)
      {
        vl5310xControl = 1;
        Serial.println("  Altidude hold shield was found" + vl5310xControl);
        Serial.println(vl5310xControl);
      }

      if (address == 112)
      {
        multiRangerControl = 1;
        Serial.println("  multi-ranger hold shield was found " + multiRangerControl);
        Serial.println(multiRangerControl);
      }

      if (address == 119)
      {
        bme280Control = 1;
        Serial.println("  BME280 shield was found " + bme280Control);
        Serial.println(bme280Control);
      }

      if (address == 8)
      {
        opticalFlowControl = 1;
        Serial.println("  Optical hold shield was found " + opticalFlowControl);
        Serial.println(opticalFlowControl);
      }

      if (address == 9)
      {
        lpsControl = 1;
        Serial.println("  dwml000 hold shield was found " + lpsControl);
        Serial.println(lpsControl);
      }

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void mainSetup()
{

  // Set Pin Mode
  Serial.begin(921600); // 921600

  while (!Serial)
  {
    delay(1);
  }

  // analogWriteFreq(20000);
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);

  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  digitalWrite(D4, LOW);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  ////// Initialize the PWM

  // Initial duty -> all off

  for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++)
  {
    pwm_duty_init[channel] = 0;
  }

  // Period

  uint32_t period = PWM_PERIOD;

  // Initialize

  pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);

  // Commit

  pwm_start();

  esp.redLed_Digital(false);
  esp.blueLed_Digital(false);
  esp.greenLed_Digital(false);

  EEPROM.begin(512);
  scanShields();

  ahrs.Initialize();

  if (Trim_DF == 1)
  {
    EEPROM.write(60, highByte(Trim_Roll_DF));
    EEPROM.write(61, lowByte(Trim_Roll_DF));

    EEPROM.write(62, highByte(Trim_Pitch_DF));
    EEPROM.write(63, lowByte(Trim_Pitch_DF));

    EEPROM.write(64, highByte(Trim_Yaw_DF));
    EEPROM.write(65, lowByte(Trim_Yaw_DF));

    EEPROM.commit();
  }

  Trim_Roll_Bs = word(EEPROM.read(60), EEPROM.read(61));
  Trim_Pitch_Bs = word(EEPROM.read(62), EEPROM.read(63));
  Trim_Yaw_Bs = word(EEPROM.read(64), EEPROM.read(65));

  yaw.SetItermRate(Trim_Yaw_Bs);

  roll.SetGain(15.2, 25.0, 0.18, 0.0, 1.4); //     roll.SetGain(15.2, 25.0, 0.18,0.0,1.4);
  roll.SetLimit(4500, 3000, 6000);          //  roll.SetLimit(4500, 3000, 6000);

  pitch.SetGain(15.2, 25.0, 0.18, 0.0, 1.4); //    pitch.SetGain(15.2, 25.0, 0.18,0.0,1.4);
  pitch.SetLimit(4500, 3000, 6000);          //    pitch.SetLimit(4500, 3000, 6000);

  yaw.SetGain(0.8, 0.3, 0.5, 0.9, 0.0); // (0.8,0.3,0.5,0.9,0.0);
  yaw.SetLimit(4500, 1000, 6000);

  oto.SetGain(0.0, 0.0, 1.8, 1.5, 750.0); // 1.8,3.5,1000.0);
  oto.SetLimit(1000, 1000, 6500);

  yOpt.SetGain(0.0, 0.0, 0.012, 0.05, 0.0); // 0.012(++) ,0.05 ,0.0(~));
  yOpt.SetLimit(6500, 1000, 10000);         // 4500, 1000, 10000

  xOpt.SetGain(0.0, 0.0, 0.012, 0.05, 0.0); // 0.02
  xOpt.SetLimit(6500, 1000, 10000);         // 3000, 900, 3500

  xMulti.SetGain(0.0, 0.0, 1.6, 0.4, 125.0); //  0.9 ,0.4 ,125); // 1.6 ,0.6 ,125.0);
  xMulti.SetLimit(4500, 1000, 6000);         // 3000, 900, 3500

  yMulti.SetGain(0.0, 0.0, 1.6, 0.4, 125.0);
  yMulti.SetLimit(4500, 1000, 6000); // 3000, 900, 3500

  xMultiM.SetGain(0.0, 0.0, 1.6, 0.4, 125.0); //  0.9 ,0.4 ,125); // 1.6 ,0.6 ,125.0);
  xMultiM.SetLimit(4500, 1000, 6000);         // 3000, 900, 3500

  yMultiM.SetGain(0.0, 0.0, 1.6, 0.4, 125.0);
  yMultiM.SetLimit(4500, 1000, 6000); // 3000, 900, 3500

#ifdef bme280
  if (bme280Control == 1)
  {
    bmeSetup();
  }
#endif

#ifdef vl53l0x
  if (vl5310xControl == 1)
  {
    InitVL53L0X();
  }
#endif

#ifdef opticalFlow
  if (opticalFlowControl)
  {
    opticalSensorSetup();
  }
#endif

#ifdef MULTI_RANGER
  if (multiRangerControl == 1)
  {
    multiRangerSetup();
  }
#endif

  esp.redLed_Digital(false);
  esp.blueLed_Digital(false);
  esp.greenLed_Digital(true);

  setupWiFi();
}
