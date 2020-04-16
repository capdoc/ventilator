#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 4 line display
//lcd refresh rate
#define REFRESH_RATE 200

#define DEBUG false

//BUTTONS/LEDS/SWITCHES
#define START_BTN_PIN      2
#define LIMIT_SWITCH_PIN   3
#define CONFIG_BTN_PIN     10
#define OK_BTN_PIN         12
#define START_LED_PIN      7
#define BUZZER             8

//STEPPER CONFIGS.

//enable pin
#define STEPPER_ENABLE     4
//direction pin
#define STEPPER_DIR        5
//step pin
#define STEPPER_STEP       6
//arm daow direction
#define STEPPER_DIR_DOWN   LOW
//arm up direction
#define STEPPER_DIR_UP     HIGH

// steps for upper most limit of arm
#define BAG_UPPER_LIMIT   3600


//POT CONFIG
#define VOLUME_POT              A0  // Amount to compress the AmbuBag 50ml -1L
#define BREATHS_PER_MIN_POT     A1  // Duty cycle 6-30 per minute
#define IE_RATION_POT           A2  // Ratio of Inspiratory to Expiratory time
// #define INSPIRATORY_TIME_POT    A3  // Tweak Inspiratory time, ie time to compress bag

//debounce for button
#define BTN_DEBOUNCE_DELAY 20
//debounce for pot
#define POT_DEBOUNCE_DELAY 2


#define VOLUME_MIN 150
#define VOLUME_MAX 850 //This should be entered in Calibration!!!!!****************************************
#define VOLUME_INCREMENTS 25
#define STEP_TO_VOLUME_INCREMENTS (VOLUME_MAX-VOLUME_MIN)/50

#define BREATHS_PER_MIN_MIN 6
#define BREATHS_PER_MIN_MAX 40

#define IE_RATIO_MIN 100
#define IE_RATIO_MAX 500


//calibration array for ml increments
uint16_t stepsToVolume[STEP_TO_VOLUME_INCREMENTS] = {0}; //850-150=700     700/50 = 14 50mL increments
//int calib_arr[ML_ARRAY_INC];
int calib_index = 0;

// FLAGS
//boolean switchState = false;
boolean okBtnFlag = false;
boolean confBtnFlag = false;
boolean startBtnFlag = false;
boolean lcd_dis = false;
boolean limitActived = false;
boolean err_flag = true;
boolean calib_done = false;

//*******************************EEPROM CONFIG EDIT************************************
// Always give your config an id, useful to debug and when config layout changes
#define CONFIG_VERSION 1

//EEPROM STRUCT
struct configStruct {
  //ADD EVERYTHING HERE YOU WANT USERS TO BE ABLE TO CONFIGURE
  //ie VOLUME_MAX, BAG_UPPER_LIMIT, etc. 
  uint16_t stepsUpperLimit;
  uint16_t stepsToVolume[STEP_TO_VOLUME_INCREMENTS];
  uint8_t version; // Placed last to verify you wrote/read correctly
} config ;

void saveConfig() 
{
  for (uint8_t t=0; t<sizeof(config); t++) {
    EEPROM.write(t, *((char*)&config + t));
  }
  Serial.println();
  Serial.println("Config saved!");
  Serial.println();
}

void loadConfig() 
{
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(sizeof(config)-1) == CONFIG_VERSION) {
    for (uint8_t t=0; t<sizeof(config); t++) {
      *((char*)&config + t) = EEPROM.read(t);
    }
  } else {
    //NO VALID CONFIG FOUND SAVING
    saveConfig();
  }
  Serial.println();
  Serial.print("Config loaded version: ");
  Serial.println(config.version);
  Serial.print("Config upper limit: ");
  Serial.println(config.stepsUpperLimit);
  Serial.println();
}



volatile boolean startEnabled = false;
volatile unsigned long lastStartPress = 0;

int steps = BAG_UPPER_LIMIT;              // Current Postion of Stepper Motor
int stepsUpperLimit = BAG_UPPER_LIMIT; // Upper Limit of Stepper Motor <= should be configured and stored in EEPROM -- 3600 full bag but weight of arm limits at 3500 about 20ml compression under weight of arm, some slack on string.
byte setupState = 0;        // State to store calibration and setup
boolean lockEnabled = false;



// OPERATING MODES
enum MODE {
  WAIT,
  RESET_ARM,
  STANDBY,
  POT_CONFIG,
  VOL_CONFIG,
  ML_CONFIG,
  READY,
  RUNNING,
  ERR
};

MODE mode;


/**********************************************************************
####################### ISR's ####################################
**********************************************************************/

// ISR TO ALERT WHEN LIMIT IS HIT
void limitTriggered_ISR()
{
  limitActived = true;
}

// ISR TO ENABLE/DISABLE MACHINE
// volatile boolean startEnabled = false;
// volatile unsigned long lastStartPress = 0;
// void startTriggered_ISR()
// {
//   unsigned long timeNow = millis();
//   if(timeNow - lastStartPress > 100) {
//     startEnabled = !startEnabled;
//     digitalWrite(START_LED_PIN, startEnabled);
//     lastStartPress = timeNow;
//     if (startEnabled) {
//       mode = RUNNING;
//     } else {
//       mode = RESET_ARM;
//     }
//   }
// }


/**********************************************************************
####################### FUNCTIONS ####################################
**********************************************************************/
//
// Function to initialise lcd screen
void lcdInit()
{
  lcd.init();
  lcd.backlight();
  lcd.home ();                      // Go to the home location
}



/**********************************************************************
* Clean read pin
**********************************************************************/
int cleanRead(byte pin)
{
  int val = analogRead(pin);
  delay(POT_DEBOUNCE_DELAY);
  if (val == analogRead(pin)){
    return val;
  }
}

/**********************************************************************
* Buzzer
**********************************************************************/
void buzzer(int ms)
{
  Serial.println("Buzzing..");
  tone(BUZZER, 1000); // Send 1KHz sound signal...
  delay(ms);        // ...for 1 sec
  noTone(BUZZER);     // Stop sound..
}



//Reads the Volume Pot, maps the reading to the above limits and increments by set amount
// uint16_t getVolume()
// {
//   int volumeIncrements = (VOLUME_MAX - VOLUME_MIN) / VOLUME_INCREMENTS;
//   byte volumeReading = map(cleanRead(VOLUME_POT), 0, 1023, 0, volumeIncrements);
//   return (volumeReading * VOLUME_INCREMENTS) + VOLUME_MIN;
// }



//Reads the BPM Pot, maps the reading to the above limits, contrains to within machines limits
// byte getBreathsPerMiute(byte restrictedMax)
// {
//   byte bpm = map(cleanRead(BREATHS_PER_MIN_POT), 0, 1023, BREATHS_PER_MIN_MIN, BREATHS_PER_MIN_MAX);
//   return constrain(bpm, BREATHS_PER_MIN_MIN, restrictedMax);
// }

//Increments by 25 ie 1:(IE_RATIO/100)  1:1, 1:1.25 etc

//Reads the I:E Pot, maps the reading to the above limits, contrains to within calculated limits
// uint16_t getIERatio(int restrictedMax)
// {
//   uint16_t ie =  map(cleanRead(IE_RATION_POT), 0, 1023, IE_RATIO_MIN, IE_RATIO_MAX);
//   return constrain(ie, IE_RATIO_MIN, restrictedMax);
// }




/**********************************************************************
* Slow step of stepper
**********************************************************************/
void slowStep(int delayTime)
{
  //Step the stepper 1 step
  digitalWrite(STEPPER_STEP, HIGH); // Output high
  delayMicroseconds(delayTime);    // Wait
  digitalWrite(STEPPER_STEP, LOW); // Output low
  delayMicroseconds(delayTime);   // Wait
}

/**********************************************************************
* Handle any Button Presses - with Debounce
**********************************************************************/

void handleBTN()
{
  // Debounce config button return false
  if (digitalRead(CONFIG_BTN_PIN) == LOW) { //<<<< High
    delay(BTN_DEBOUNCE_DELAY);
    if (digitalRead(CONFIG_BTN_PIN) == LOW) {
      confBtnFlag = true;
    }
  }

  // Debounce ok button return true
  if (digitalRead(OK_BTN_PIN) == LOW) {
    delay(BTN_DEBOUNCE_DELAY);
    if (digitalRead(OK_BTN_PIN) == LOW) {
      okBtnFlag = true;
    }
  }

  //not sure START should be on interrupt??

  // Debounce start button return true
  if (digitalRead(START_BTN_PIN) == HIGH) { // <<<< HIGH activated
    delay(BTN_DEBOUNCE_DELAY);
    if (digitalRead(START_BTN_PIN) == HIGH) {
      startBtnFlag = true;
    }
  }
}


/**********************************************************************
* Display lcd
**********************************************************************/
//Display Calibration date on LCD at above refresh rate
void displayPos (int value, boolean displaySteps = false, int pos = 0)
{
  static unsigned long lastRefreshTime = 0;
  unsigned long timeNow = millis();
  if (timeNow - lastRefreshTime > REFRESH_RATE) {
    lcd.setCursor( pos, 3 );
    lcd.setCursor( pos, 3 );
    lcd.print(value);
    lcd.print(" ");
    if(displaySteps) {
      lcd.setCursor(15,2);
      lcd.print("Steps");
      lcd.setCursor( 15, 3 );
      lcd.print(steps);
    }
    lastRefreshTime = timeNow;
  }
}

/**********************************************************************
* Clear lcd
**********************************************************************/
void clearLCD()
{
  lcd.setCursor (0, 0);
  lcd.print(F("                    "));
  lcd.print(F("                    "));
  lcd.print(F("                    "));
  lcd.print(F("                    "));
  lcd.setCursor (0, 0);
}

/**********************************************************************
* Zero arm position
**********************************************************************/
void zeroArm() {
  // Zero arm on power on
  clearLCD();
  lcd.print("Zeroing arm position");
  lcd.setCursor(0,2);
  lcd.print("**Keep hands clear**");
  // Move arm down to switch
  digitalWrite(STEPPER_ENABLE, HIGH);
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  while(!limitActived) {
    slowStep(200);
    //no need to decrement, possible unknown position
    //steps--;
  }
  limitActived = false;
  steps = 0;
  // Move arm to upper limit
  digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
  while(steps < stepsUpperLimit) {
    slowStep(200);
    steps++;
  }
  Serial.print("MAX STEPS: ");
  Serial.println(steps);
  digitalWrite(STEPPER_ENABLE, LOW);

  //small buzz
  buzzer(100);
}


/**********************************************************************
* Set arm to last known VALID calibration position
**********************************************************************/
void resetToLast(int lastGoodVolume)
{
  //first zero arm
  zeroArm();

  // Move arm down to switch
  digitalWrite(STEPPER_ENABLE, HIGH);
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  while(steps > lastGoodVolume) {
    slowStep(200);
    //no need to decrement, possible unknown position
    steps--;
  }
  //digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
  digitalWrite(STEPPER_ENABLE, LOW);

}
//FUNCTION *********************************************************************************************************************************
void findVolume(byte volumeIncrement)
{
  //ONLY MOVE ARM DOWN
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  lcd.setCursor (5, 2);
  lcd.print(F("    "));
  lcd.setCursor (5, 2);
  lcd.print(((volumeIncrement)*50)+VOLUME_MIN);
  boolean notSet = true;
  while(notSet){
    int VT = cleanRead(VOLUME_POT);
    if(VT < 256){
      if(!limitActived) {
          slowStep(500);
          steps--;
      }
      limitActived = false;
    }

    // if(OK button pressed){
    //   notSet = false;
    // }

    // if(CONFIG button pressed){
    //   resetToLast(volumeIncrement-1)
    // }
  }
  config.stepsToVolume[volumeIncrement] = steps;
}
// *********************************************************************************************************************************

void handleCalibrate()
{
  //Variables only used in Calibration
  //int amountToBreath = 2000;
  //int speedToStepAt = 2000;
  static unsigned long lungInflationTime = 0;
  static byte lastState = 0;

  while (setupState < 10) {
    handleBTN();              //Moves between state!
    int potReading = cleanRead(VOLUME_POT);
    switch (setupState) {
      case 0:
        // Does nothing
        Serial.println(F("CALIBRATE Routine"));
        setupState++;
      case 1:
        if (lastState != setupState) {
          Serial.println(F("Set Volume knob to 0"));

          clearLCD();
          lcd.print(F("***** CALIBRATE ****"));
          lcd.setCursor (0, 1);
          lcd.print(F("*** Turn VT to 0 ***"));
          lcd.setCursor (0, 2);
          lcd.print(F("Push OK to continue"));
        }
          lcd.setCursor (0, 3);
          lcd.print(F("VT: "));
        displayPos(map(potReading, 0, 1023, -5, 5));
        break;
      case 2:    // Moves ARM down, compressing bag until it hits limit switch
        if (lastState != setupState) {
          Serial.println(F("Compressing bag fully"));
          digitalWrite(STEPPER_ENABLE, HIGH);
          clearLCD();
          lcd.setCursor (0, 0);
          lcd.print(F("                    "));
          lcd.setCursor (0, 1);
          lcd.print(F("                    "));
          lcd.setCursor (0, 0);
          lcd.print(F("Setting Maximum Vol."));
        }

        digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
        while(!limitActived){
            slowStep(400);
            steps--;
        }
        limitActived = false;
        steps = 0;
        lcd.setCursor(0,3);
        lcd.print("Push OK to continue");
        break;
      case 3:    // Moves ARM up, inflating bag until it hits coded limit
        if (lastState != setupState) {
          Serial.println(F("Set Top, Allow bag to inflate fully"));
          clearLCD();
          lcd.setCursor (0, 0);
          lcd.print(F("                    "));
          lcd.setCursor (0, 0);
          lcd.print(F("Set Max Inf. of Bag"));
        }
        digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
        while(steps <= stepsUpperLimit) {
          slowStep(400);
          steps++;
        }
        lcd.setCursor(0,2);
        lcd.print("Steps: ");
        lcd.setCursor(8,2);
        lcd.print(steps);
        lcd.setCursor(0,3);
        lcd.print("Push OK to set");//user pot to increase and decrease steps to set max bag volume
        break;
      case 4:    // Oscillates Bag at user adjustable depth and speed to find different volumes
        clearLCD();
        lcd.print("Calibrate Bag Vol");

        // TO DO:
        // Dont move until pot turns until

//Calabrate Bag Volume,
//Find 100ml
//turn VT pot to 0 wich is all the way off.
//arm does not move.
//for us we can have steps for now showing.
//use one pot for 1024 on VT pot - give ml.
//if we blow the level on one stage we can refil the water and press conig
//sets arm to zero and asks to set VT pot back to zero and re-do that step.
//as you progres through you press ok to save.




        if (lastState != setupState) {
          Serial.println("Map Volume - using https://youtu.be/cy4kzOeLD5E");
          clearLCD();
          lcd.print(F("VT = Volume"));
          lcd.setCursor (0, 1);
          lcd.print(F("OK=Next, CONFIG=Back"));
          // lcd.print(F("BPM = Speed")); // Set speed low to find volume displaced
          lcd.setCursor (0, 2);
          lcd.print(F("Find              ml"));
        }
        for(byte i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++) {
          findVolume(i);
        }
        //SAVE stepsToVolume Array to EEPROM***************************************************************************************************************

        // digitalWrite(STEPPER_DIR, STEPPER_INVERT_DIR);
        // amountToBreath = map(cleanRead(VOLUME_POT), 0, 1023, 20, stepsUpperLimit);
        // speedToStepAt = map(cleanRead(BREATHS_PER_MIN_POT), 0, 1023, 220, stepsUpperLimit);
        // lungInflationTime = millis();
        // while(steps > amountToBreath){
        //     if(digitalRead(LIMIT_SWITCH_PIN)){
        //       slowStep(1000);
        //       steps--;
        //     }
        // }
        // displayPos(millis() - lungInflationTime, true);

        // delay(1000);

        // digitalWrite(STEPPER_DIR, !STEPPER_INVERT_DIR);
        // while(steps < stepsUpperLimit){
        //     slowStep(1000);
        //     steps++;
        // }

        // delay(1000);

        break;
      case 5:    // Move between Top and Bottom Limits
        //TODO:  IMPLMENT FOR ALL OTHER VOLUMES REQUIRED 200ml to Max @ 100ml increments
        break;
    }
    lastState = setupState;
  }
}


//POSSIBLE ROLLING AVERAGE THESE
int requiredVolume = 0;
int stepsForRequiredVolume = 0;
int requiredBPM = 0;
int requiredIERatio = 0;
int calculatedInspiratoryTime = 0;
int calculatedExpiratoryTime = 0;

int machineRestrictedBPM = BREATHS_PER_MIN_MAX;
int machineRestrictedIERation = IE_RATIO_MAX;


void handleSettings()
{
//   //Volume and machine specs limit BPM and IE ratio
//   requiredVolume = getVolume();
//   //BPM further limits IE Ratio

//   //  60000 / Max Time to Produce Volume * 2 (1:1 Ratio)
//   //  60000 / 800mS * 2 = 60000 / 1600 = 37.5BPM

//   // machineRestrictedBPM = 6000 / (mapToSteps(requiredVolume)*stepTime)*2

//   //DUMMYS TO SIMULATE MAX TIME TO REACH VOLUME & AMOUNT OF STEPS REQUIRED
//   uint16_t maxTimeToReachVolume = requiredVolume; //DUMMY DUMMY DUMMY DUMMY DUMMY DUMMY DUMMY DUMMY DUMMY DUMMY
//   stepsForRequiredVolume = requiredVolume;  //look up steps for required Volume!!!!

//   machineRestrictedBPM = 60000 / (maxTimeToReachVolume * 2);

//   requiredBPM = getBreathsPerMiute(machineRestrictedBPM);

//   //Calculation examples
//   //   mSPerBreath = 60000/requiredBPM;
//   //   1600 = 60000/37.5
//   // 1600 - Max Time to Produce Volume
//   // 1600 - 800 = 800 ExpiratoryTimeRemaining
//   // (ExpiratoryTimeRemaining / Max Time to Produce Volume)  * 100
//   // (800.0 / 800.0) * 100 = 100 (1:1 Ratio)

//   // 60000/20 = 3000
//   // 3000 - 800 = 2200 ExpiratoryTimeRemaining
//   // (2200.0 / 800.0) * 100 = 275 (1:2.75 Ratio)

//   // 60000/40 = 1500 mSPerBreath
//   // 1500 - 700 = 900 ExpiratoryTimeRemaining
//   // (900 / 700) * 100 = 128 (1:1.25 Ratio)


//   //mSPerBreath = Total Inspiration & Expiratory Time
//   uint16_t mSPerBreath = 60000/requiredBPM;
//   uint16_t expiratoryTimeRemaining = mSPerBreath-maxTimeToReachVolume;
//   machineRestrictedIERation = ((float)expiratoryTimeRemaining / (float)maxTimeToReachVolume) * 100.0;

//   requiredIERatio = getIERatio(machineRestrictedIERation);

//   float mSPerRatio = ((float)mSPerBreath/(100.0+(float)requiredIERatio));
//   calculatedInspiratoryTime = mSPerRatio * 100;
//   //calculatedExpiratoryTime = mSPerRatio * requiredIERatio;  //Incurs rounding issues - DON'T USE
//   calculatedExpiratoryTime = mSPerBreath - calculatedInspiratoryTime;

//   // Serial.println("");
//   // Serial.print(maxTimeToReachVolume);
//   // Serial.print(" : ");
//   // Serial.print(mSPerBreath);
//   // Serial.print(" : ");
//   // Serial.print(expiratoryTimeRemaining);
//   // Serial.print(" : ");
//   // Serial.print(machineRestrictedIERation);
//   // Serial.print(" : ");
//   // Serial.print(requiredIERatio);
//   // Serial.print(" : ");
//   // Serial.print(mSPerRatio);
//   // Serial.print(" : ");
//   // Serial.print(calculatedInspiratoryTime);
//   // Serial.print(" : ");
//   // Serial.println(calculatedExpiratoryTime);
//   // delay(1000);

//   //Calculation examples
//   // 60000/BPM = mSPerBreath
//   // mSPerBreath / (100+requiredIERatio) = calculatedRatioTime
//   // calculatedRatioTime * 100 = calculatedInspiratoryTime
//   // calculatedRatioTime * requiredIERatio = calculatedExpiratoryTime

//   // 30 BPM @ 1:1 Ratio
//   // 60000/30 BPM = 2000mS Per Breath
//   // 2000/(100+100) = 10 Ratio Time
//   // 10 * 100 = 1000 calculatedInspiratoryTime
//   // 10 * 100 = 1000 calculatedExpiratoryTime

//   // 30 BPM @ 1:1.5 Ratio
//   // 60000/30 BPM = 2000mS Per Breath
//   // 2000/(100+150) = 8 Ratio Time
//   // 8 * 100 = 800 calculatedInspiratoryTime
//   // 8 * 150 = 1200 calculatedExpiratoryTime
}

void handleScreen()
{
  static int lastVolume = 0;
  static int lastBPM = 0;
  static int lastIERation = 0;
  if(lastVolume != requiredVolume || lastBPM != requiredBPM || lastIERation != requiredIERatio){
    lcd.setCursor(1,2);
    lcd.print("VT");
    lcd.setCursor(0,3);
    lcd.print(F("                    "));
    lcd.setCursor(0,3);
    lcd.print(requiredVolume);
    lcd.print("ml");


    lcd.setCursor(9,2);
    lcd.print("BPM");
    lcd.setCursor(9,3);
    lcd.print(requiredBPM);

    lcd.setCursor(16,2);
    lcd.print("I/E");
    lcd.setCursor(14,3);
    lcd.print("1:");
    lcd.print((float)requiredIERatio/100.0);

    lcd.setCursor(0,0);
    lcd.print("In:Ex (ml)");
    lcd.setCursor(11,0);
    lcd.print("         ");
    lcd.setCursor(11,0);
    lcd.print(calculatedInspiratoryTime);
    lcd.print(":");
    lcd.print(calculatedExpiratoryTime);

    lastVolume = requiredVolume;
    lastBPM = requiredBPM;
    lastIERation = requiredIERatio;
  }
}

void breath()
{
  while(steps > stepsForRequiredVolume){
    if(!limitActived){
      slowStep(calculatedInspiratoryTime / stepsForRequiredVolume);
      steps--;
    } else {
      //ALARM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WE SHOULD NOT HIT LIMIT
    }
    limitActived = false;
    if(startEnabled == false) return; //EXIT IF START IS DISABLED
  }
  digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
  while(steps < stepsUpperLimit){
    slowStep(calculatedExpiratoryTime / stepsForRequiredVolume);
    steps++;
    if(startEnabled == false) return; //EXIT IF START IS DISABLED
  }
}

/**********************************************************************
* STANDBY SCREEN
**********************************************************************/
void standbyScreen() {
  clearLCD();
  lcd.print(F("* Ventilator v0.90 *"));
  lcd.setCursor(0,2);
  lcd.print(F("Press START to vent"));
  lcd.setCursor(0,3);
  lcd.print(F("Press CONFIG to cal"));

  if (DEBUG) {
    Serial.println(digitalRead(START_BTN_PIN));
    delay(50);
    Serial.println(digitalRead(CONFIG_BTN_PIN));
    delay(50);
  }
}

/**********************************************************************
* CONFIGURATION SCREENS
**********************************************************************/
//####################################################
/*
FIRST CONFIGURATION
*/
void potConfig() {
  Serial.println("First Config Screen");
  clearLCD();
  lcd.print(F("***** CALIBRATE ****"));
  lcd.setCursor (0, 1);
  lcd.print(F("*** Turn VT to 0 ***"));
  lcd.setCursor (0, 2);
  lcd.print(F("Push OK to continue"));
  lcd.setCursor (0, 3);
  lcd.print(F("VT: "));

  //wait for button press
  while (!okBtnFlag) {
    displayPos(map(cleanRead(VOLUME_POT), 0, 1023, -5, 5), false, 3);
    handleBTN();
  }
  okBtnFlag = false;
  Serial.println("Vol pot centered");
}



//####################################################
/*
VOLUME CALIBRATION CONFIGURATION
*/
void volConfig() {
  // Compress bag to get 0 point
  Serial.println(F("Compressing bag fully"));
  clearLCD();
  lcd.print(F("**Compressing bag**"));
  lcd.setCursor (0, 3);
  digitalWrite(STEPPER_ENABLE, HIGH);
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  while(!limitActived){
      slowStep(400);
      steps--;
  }
  limitActived = false;
  lcd.setCursor(0,3);
  lcd.print("Push OK to continue");
  while (!okBtnFlag) {
    handleBTN();
  }
  okBtnFlag = false;
  // Zero point found, bag fully compressed
  steps = 0;

  // Inflate Bag to set max, count steps
  // Write screen
  clearLCD();
  lcd.print(F("Set Maximum Volume "));
  lcd.setCursor(0,1);
  lcd.print(F("***Inflating Bag***"));
  lcd.setCursor(0,2);
  lcd.print(F("Press OK once bag is"));
  lcd.setCursor(0,3);
  lcd.print(F("fully inflated."));
  digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
  // Only inflate to upper limit as hardcoded
  Serial.println(okBtnFlag);
  while (steps < BAG_UPPER_LIMIT && !okBtnFlag) { //must be BAG_UPPER_LIMIT, not stepsUpperLimit
    slowStep(400);
    steps++;
    handleBTN();
  }
  // OK button pushed
  okBtnFlag = false;
  digitalWrite(STEPPER_ENABLE, LOW);

  //set the NEW UPPER LIMIT
  stepsUpperLimit = steps;

  //TODO - write upper limit to EEPROM?


  // Ok has been hit, check if retest is needed or proceed.
  // Write screen
  clearLCD();
  lcd.print(F("Bag inflated in"));
  lcd.setCursor(0,1);
  lcd.print(steps);
  lcd.setCursor(5,1);
  lcd.print(F(" steps."));
  lcd.setCursor(0,2);
  lcd.print(F("Press OK to continue"));
  lcd.setCursor(0,3);
  lcd.print(F("Press CONF to retry"));

  // wait for button press
  // Check if ok to proceed or retest
  while (!okBtnFlag && !confBtnFlag) {
    handleBTN();
  }
}



//######################################################
/*
ZERO POTENTIOMETER
*/
// calibrate volume to VT pot
void zeroPot() {
  Serial.println("Zeroing potentiometer..");
  // Write screen
  clearLCD();
  lcd.print(F("*** ML increments ***"));
  lcd.setCursor (0, 1);
  lcd.print(F("Turn VT fully"));
  lcd.setCursor (0, 2);
  lcd.print(F("anticlockwise."));
  lcd.setCursor (0, 3);
  lcd.print(F("Press OK to continue"));

  // wait for ok
  while (true) {
    //Serial.println("WAITING for ZERO");
    handleBTN();

    //if ok pressed
    if (okBtnFlag){
      Serial.println("WAITING for ZERO...");
      //get pot reading
      int potVal = cleanRead(VOLUME_POT);
      Serial.print("Pot val: ");
      Serial.println(potVal);

      //check if pot is actually set to zero
      if (potVal == 0){
        Serial.println("Pot at zero.");

        //reset ok flag
        okBtnFlag = false;

        //if zero reached, break out
        break;
      }
      //reset ok flag
      okBtnFlag = false;
    }
  }
}


//######################################################
/*
ML CALLIBRATE CONFIGURATION
*/
void mlConfig(){
  int newVol = 0;
  int currentVol = 0;

  //Enable stepper
  digitalWrite(STEPPER_ENABLE, HIGH);
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);

  // Move arm down slowly using VT pot until 50ml water displaced and press ok, or limit switch hit
  lcd_dis = true;
  while (true) {
    //display message once
    if (lcd_dis){
      clearLCD();
      lcd.print(F("*** ML increments ***"));
      lcd.setCursor (0, 1);
      lcd.print(F("Turn VT until 50 ml "));
      lcd.setCursor (0, 2);
      lcd.print(F("is reached."));
      lcd.setCursor (0, 3);
      lcd.print(F("Press OK when done"));
      lcd_dis = false;
    }

    lcd_dis = true;

    //get the pot reading and move arm if 'OK' not pressed
    while(!okBtnFlag && !confBtnFlag){
      //get the new reading
      newVol = cleanRead(VOLUME_POT);

      //compare the values
      if (newVol > currentVol + 5) {
        //Serial.println("Greater >>>>>");
        //Serial.print("New pot val: ");
        //Serial.println(newVol);

        //get pot difference (amount to move)
        int diff = (newVol - currentVol) * 3;

        //relative steps
        for (int i = 0; i < diff; i++){
          //Serial.print("STEP: ");
          //Serial.println(i);
          //step
          slowStep(500);
          steps--;
          //Serial.print("TOTAL STEPS: ");
          //Serial.println(steps);
        }

        //assign new value
        currentVol = newVol;

        //if reached the limit switch break out
        if (limitActived){
          //mode = ERR;
          //err_flag = true;
          //don't reset limit here
          break;
        }
      }

      // display next step message if not done
      if (lcd_dis){
        clearLCD();
        lcd.print(F("*** ML increments ***"));
        lcd.setCursor (0, 1);
        lcd.print(F("Set 50 ml increment?"));
        lcd.setCursor (0, 2);
        lcd.print(F("OK=Yes, CONF=No"));
        lcd_dis = false;
      }

      //handle ok pressed
      handleBTN();
    }


    //handle saving the value if 'OK' pressed
    if (okBtnFlag){
      //reset flag
      okBtnFlag = false;

      //ok pressed, save array
      Serial.println("SAVING!");

      //save the step marker to array
      stepsToVolume[calib_index] = (int)steps;
      //increment index for
      calib_index++;

      //view array
      for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
        //show the value for each
        Serial.print(stepsToVolume[i]);
        Serial.print(" ,");
      }

    //handle 'CONF' pressed
    }else if (confBtnFlag){
      // dont reset flag here!
      //confBtnFlag = false;
      //conf pressed, try again
      Serial.println("Returning....");
      //disable stepper
      digitalWrite(STEPPER_ENABLE, LOW);
      //break;
    }

    //raise flag if we are at the end of the array
    if(calib_index == STEP_TO_VOLUME_INCREMENTS){
      //calibration finished
      calib_done = true;
      //display next message
      lcd_dis = true;
      //enter ready mode
      mode = READY;
    }

    //return to set ML_CONFIG
    break;
  }

  //disable stepper
  digitalWrite(STEPPER_ENABLE, LOW);


}




/**********************************************************************
* SETUP
**********************************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(OK_BTN_PIN, INPUT_PULLUP);
  pinMode(CONFIG_BTN_PIN, INPUT_PULLUP);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_ENABLE, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(START_LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitTriggered_ISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(START_BTN_PIN), startTriggered_ISR, FALLING);

  //TODO - set initial states

  loadConfig();
  lcdInit();
  clearLCD();

  //start mode
  mode = WAIT;
}



/**********************************************************************
* MAIN LOOP
**********************************************************************/
void loop() {
  if(limitActived){
    //Serial.println("Ready...");
    limitActived = false;
  }

  /*
  SWITCH MODES
  */
  switch (mode) {

    /*
    WAITING MODE
    */
    case WAIT:
      clearLCD();
      lcd.print(F("*** WAITING ..... ***"));
      lcd.setCursor (0, 1);
      lcd.print(F("                     "));
      lcd.setCursor (0, 2);
      lcd.print(F("Press OK when ready."));

      Serial.println("Ready...");

      //wait for system to be ready and ok press
      while(true){
        handleBTN();
        if (okBtnFlag) {
          okBtnFlag = false;
          mode = RESET_ARM;
          break;
        }
      }
    break;


    /*
    RESET MODE
    */
    case RESET_ARM:
      //reset steps
      Serial.println("Reset Mode");
      zeroArm();

      //increment mode to next state
      mode = STANDBY;
      lcd_dis = true;


      break;

    /*
    STANDBY MODE
    */
    case STANDBY:
      //Serial.println("Standby Mode");
      if (lcd_dis) {
        lcd_dis = false;
        standbyScreen();
      }
      // Wait for user input
      handleBTN();

      //Config button pressed, go to config
      if (confBtnFlag) {
        confBtnFlag = false;

        //set config sequece start mode
        //should be 'POT_CONFIG'
        //skipping for now
        mode = POT_CONFIG;
      }

      //if OK pressed, go to start    <<<<<<<<START PRESSED
      if (startBtnFlag){
        startBtnFlag = false;

        //RUNNING mode, begin breathing
        mode = RUNNING;
      }

      break;


    //*************IGNORING for now
    case POT_CONFIG:
      /*
      CENTER POTENTIOMETER
      */
      Serial.println("Potentiometer Zeroing Mode");
      //first config screen
      potConfig();
      //Manually set to zero, assume correct
      mode = VOL_CONFIG;
      break;



    //***********IGNORING for now
    case VOL_CONFIG:
      /*
      CALIBRATE VOLUME OF BAG
      */
      Serial.println("Volume Config Mode");
      volConfig();
      // Recalibrate bag volume if config pressed
      if (okBtnFlag) {
        okBtnFlag = false;
        limitActived = false;
        mode = ML_CONFIG;
      }
      // Ok to proceed
      if (confBtnFlag) {
        confBtnFlag = false;
      }
      Serial.println("volume conf finished");
      break;



    case ML_CONFIG:
      /*
      CALLIBRATE ML
      */
      Serial.println("ML Config Mode.");

      //reset arm
      zeroArm();
      limitActived = false;

      Serial.println("Resetting volume array.....");
      //reset volume increment array
      for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
        stepsToVolume[i] = (int)0;
        //Serial.println(i);
      }

      //reset calibration index
      calib_index = 0;

      //while limit not reached
      while (!calib_done && !limitActived){

        //first zero pot by user
        zeroPot();

        // Ok to proceed
        okBtnFlag = false;
        Serial.println("starting ML config");

        //turn pot until 50 ml
        mlConfig();

        //break out if hit bottom and calibration incomplete
        //shouldn't happen but just in case
        if (limitActived && !calib_done){
          Serial.println("LIMIT!!!");
          limitActived = false;
          break;
        }

        //if conf button pressed, incorrect calibration increment
        if(confBtnFlag){
          //reset flag
          confBtnFlag = false;

          //return to last valid increment
          resetToLast(stepsToVolume[calib_index-1]);
        }

        //DONT KNOW WHY THIS IS HERE, but it needs to be...
        limitActived = false;
      }

      //Check calibration complete
      if (calib_done){
        //set config

        config.stepsUpperLimit = BAG_UPPER_LIMIT;
        for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
          config.stepsToVolume[i] = stepsToVolume[i];
        }

        //save to EEPROM
        saveConfig();
      }

      break;


    case READY:
      Serial.println("Ready Mode");

      if(lcd_dis){
        //zero arm position
        zeroArm();

        //print message
        clearLCD();
        lcd.print(F("*** CALIBRATED!! ***"));
        lcd.setCursor (0, 1);
        lcd.print(F("                     "));
        lcd.setCursor (0, 2);
        lcd.print(F("Press START to start."));

        //reset calibration flag = CAREFUL!!
        lcd_dis = false;
      }

      Serial.print("Array values: ");
      //display the increment array
      for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
        //show the value for each
        Serial.print(stepsToVolume[i]);
        Serial.print(" ,");
      }
      Serial.println();
      Serial.println();

      delay(1000);

      break;



    case RUNNING:

      Serial.println("Running Mode");
            
      //breath();

      // Wait for user input
      handleBTN();

      //Config button pressed, go to config
      if (confBtnFlag) {
        confBtnFlag = false;

        //TODO - confirm message??

        //change mode back to reset start
        mode = RESET_ARM;
      }

      if (startBtnFlag){
        startBtnFlag = false;

        //stop breathing???

      }


      break;



    case ERR:
      Serial.println("ERROR!!!!!");

      //zero arm and wait for reset
      if (err_flag == true){
        zeroArm();
        err_flag = false;
      }

      break;



    default:
      Serial.println("default");
  }

  //delay(100);
}
