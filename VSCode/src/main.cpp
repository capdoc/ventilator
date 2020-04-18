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

//STEPPER CONFIGS
#define STEPPER_ENABLE     4
#define STEPPER_DIR        5
#define STEPPER_STEP       6
#define STEPPER_DIR_DOWN   LOW
#define STEPPER_DIR_UP     HIGH

//POT CONFIG
#define VOLUME_POT              A0  // Amount to compress the AmbuBag 50ml -1L
#define BREATHS_PER_MIN_POT     A1  // Duty cycle 6-30 per minute
#define IE_RATIO_POT           A2  // Ratio of Inspiratory to Expiratory time

//debounce for button
#define BTN_DEBOUNCE_DELAY 20
//debounce for pot
#define POT_MAX_VALUE 255 // stop jittering


// steps for upper most limit of arm
#define BAG_UPPER_LIMIT 3600

//BREATH VARIABLES
#define VOLUME_MIN 150
#define VOLUME_MAX 850 //This should be entered in Calibration!!!!!****************************************
#define VOLUME_INCREMENTS 50
#define STEP_TO_VOLUME_INCREMENTS (VOLUME_MAX-VOLUME_MIN)/VOLUME_INCREMENTS

#define BREATHS_PER_MIN_MIN 6
#define BREATHS_PER_MIN_MAX 40

// in order to get float value between 1 and 5,
// 100 and 500 are used then reading divided by 100 = ratio
// e.g. 1:1.235, 1: 2.75, etc 
#define IE_RATIO_MIN 100
#define IE_RATIO_MAX 500 

#define INSPIRATORY_PAUSE 0
#define EXPPIRATORY_PAUSE 0

// Always give your config an id, useful to debug and when config layout changes
#define CONFIG_VERSION 1


//IO VARIABLES
//Postion of Stepper Motor
uint16_t steps = BAG_UPPER_LIMIT;

//previous pot readings
uint16_t volumeLast = 0;
uint16_t BPMLast = 0;
uint16_t IELast = 0;
//current pot readings
uint16_t volumeCurrent = 0;
uint16_t BPMCurrent = 0;
uint16_t IECurrent = 0;

//calculations
uint16_t calcVolume = 0;
uint16_t calcBPM = 0;
uint16_t calcIE = 0;
uint16_t calculatedInspiratoryTime = 0;
uint16_t calculatedExpiratoryTime = 0;


//calibration index 
uint8_t calibIndex = 0;

// FLAGS
boolean okBtnFlag = false;
boolean confBtnFlag = false;
boolean lcdDis = false;
boolean limitActived = false;
boolean errFlag = true;
boolean calibDone = false;

//Start button flag and timer
volatile boolean startEnabled = false;
volatile unsigned long lastStartPress = 0;


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


//EEPROM STRUCT
struct configStruct {
  //ADD EVERYTHING HERE YOU WANT USERS TO BE ABLE TO CONFIGURE
  //ie VOLUME_MAX, BAG_UPPER_LIMIT, etc. 
  uint16_t stepsUpperLimit;
  uint16_t stepsToVolume[STEP_TO_VOLUME_INCREMENTS];
  uint8_t version; // Placed last to verify you wrote/read correctly
} config = {
  //Set defaults
  BAG_UPPER_LIMIT,
  {0},
  CONFIG_VERSION
};


//**************************************************END OF DEFINITIONS*********************************8*********************



/**********************************************************************
####################### ISR's ####################################
**********************************************************************/

// ISR TO ALERT WHEN LIMIT IS HIT
void limitTriggered_ISR()
{
  limitActived = true;
}

// ISR TO ENABLE/DISABLE MACHINE
void startTriggered_ISR()
{
  unsigned long timeNow = millis();
  if(timeNow - lastStartPress > 250) {
    startEnabled = !startEnabled;
    digitalWrite(START_LED_PIN, startEnabled);
    digitalWrite(STEPPER_ENABLE, startEnabled);
    //starting
    if (startEnabled) {
      if (mode == READY || mode == STANDBY){
        mode = RUNNING;
        lcdDis = true;
      } else {
        return;
      }
    //stopping
    } else {
      mode = RESET_ARM;
    }
  }
  lastStartPress = timeNow;
}


/**********************************************************************
####################### FUNCTIONS ####################################
**********************************************************************/

/**********************************************************************
* Initialise LCD screen
**********************************************************************/
void lcdInit()
{
  lcd.init();
  lcd.backlight();
  lcd.home ();                      // Go to the home location
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
* Save configuration variables to EEPROM
**********************************************************************/
void saveConfig() 
{
  for (uint8_t t=0; t<sizeof(config); t++) {
    EEPROM.write(t, *((char*)&config + t));
  }
  Serial.println();
  Serial.println("Config saved!");
  Serial.println();
}

/**********************************************************************
* Load configuration variables from EEPROM
**********************************************************************/
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


/**********************************************************************
* Clean read pin
**********************************************************************/
int cleanRead(byte pin)
{
  return map(analogRead(pin), 0, 1023, 0, POT_MAX_VALUE);
}

/**********************************************************************
* Buzzer
**********************************************************************/
void buzzer(int ms)
{
  //Serial.println("Buzzing..");
  tone(BUZZER, 1000, ms); // Send 1KHz sound signal...
  //delay(ms);        // ...for ms duration
  //noTone(BUZZER);     // Stop sound..
}


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
}


// /**********************************************************************
// * Display lcd
// **********************************************************************/
// //Display Calibration date on LCD at above refresh rate
// void displayPos (int value, boolean displaySteps = false, int pos = 0)
// {
//   static unsigned long lastRefreshTime = 0;
//   unsigned long timeNow = millis();
//   if (timeNow - lastRefreshTime > REFRESH_RATE) {
//     lcd.setCursor( pos, 3 );
//     lcd.setCursor( pos, 3 );
//     lcd.print(value);
//     lcd.print(" ");
//     if(displaySteps) {
//       lcd.setCursor(15,2);
//       lcd.print("Steps");
//       lcd.setCursor( 15, 3 );
//       lcd.print(steps);
//     }
//     lastRefreshTime = timeNow;
//   }
// }


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
  while(steps < config.stepsUpperLimit) {
    slowStep(200);
    steps++;
  }
  Serial.print("MAX STEPS: ");
  Serial.println(config.stepsUpperLimit);
  digitalWrite(STEPPER_ENABLE, LOW);

  //small buzz
  buzzer(100);
}


/**********************************************************************
* Set arm to last known VALID calibration position
**********************************************************************/
void resetToLast(uint16_t lastGoodVolume)
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


/**********************************************************************
* Method to check which (if any) pot has been adjusted
**********************************************************************/
bool hasBeenAdjusted(byte pin){
  //return val
  boolean potHasBeenAdjusted = false;
  
  //switch pin value
  switch (pin)
  {
  case VOLUME_POT:
    potHasBeenAdjusted = (volumeCurrent != volumeLast);
    break;

  case BREATHS_PER_MIN_POT:
    potHasBeenAdjusted = (BPMCurrent != BPMLast);    
    break;
    
  case IE_RATIO_POT:
    potHasBeenAdjusted = (IECurrent != IELast);
    break;
  
  default:
    break;
  }

  //true if pot has been adjusted (Current != Last)
  return potHasBeenAdjusted;
}


/**********************************************************************
* Update VOLUME, BPM, IE, MILLISECONS screen
**********************************************************************/
void handleScreen()
{
  //if first display, update all characters
  if(lcdDis){
    //clear first
    clearLCD();

    lcd.setCursor(1,2);
    lcd.print("VT");
    lcd.setCursor(0,3);
    lcd.print(F("                    "));
    lcd.setCursor(0,3);
    lcd.print(volumeCurrent);
    lcd.print("ml");


    lcd.setCursor(9,2);
    lcd.print("BPM");
    lcd.setCursor(9,3);
    lcd.print(BPMCurrent);

    lcd.setCursor(16,2);
    lcd.print("I/E");
    lcd.setCursor(14,3);
    lcd.print("1:");
    lcd.print((float)IECurrent/100);

    lcd.setCursor(0,0);
    lcd.print("In:Ex (ml)");
    lcd.setCursor(11,0);
    lcd.print("         ");
    lcd.setCursor(11,0);
    lcd.print(calculatedInspiratoryTime);
    lcd.print(":");
    lcd.print(calculatedExpiratoryTime);

    lcdDis = false;
  //otherwise only update what has changed
  } else{
    //Volume pot
    if (hasBeenAdjusted(VOLUME_POT)){
      lcd.setCursor(0,3);
      lcd.print(volumeCurrent);
    } 
    //BPM pot
    if (hasBeenAdjusted(BREATHS_PER_MIN_POT)){
      lcd.setCursor(9,3);
      lcd.print(BPMCurrent);
    } 
    //IE Ratio pot
    if(hasBeenAdjusted(IE_RATIO_POT)){
      lcd.setCursor(14,3);
      lcd.print("1:");
      lcd.print((float)IECurrent/100);

      lcd.setCursor(11,0);
      lcd.print(calculatedInspiratoryTime);
      lcd.print(":");
      lcd.print(calculatedExpiratoryTime);
    }
  }
}




// *********************************************************************************************************************************






void handleSettings()
{
  if(hasBeenAdjusted(VOLUME_POT) || hasBeenAdjusted(BREATHS_PER_MIN_POT) || hasBeenAdjusted(IE_RATIO_POT)){
    // setting have changed


    
  }
  
}


/**********************************************************************
* BREATH
**********************************************************************/
void breath()
{
  

  //cycle timer
  //unsigned long time = millis();
  
  // Move arm down 
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  for (uint16_t i = (uint16_t)0; i < config.stepsUpperLimit - 50; i++)  { //volume level
    slowStep(130);
    steps--;
  }
  delay(10); //Inspiratory pause

  // Move arm to upper limit
  digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
  while(steps < config.stepsUpperLimit) {
    slowStep(130);
    steps++;
  }
  delay(10); //Expiratory pause

  //check we are not loosing steps (slipping)
  Serial.print("STEPS AT ORIGIN: ");
  Serial.println(steps);
  
  //print cycle time
  // Serial.print("Time: ");
  // Serial.println(millis() - time);
  


  // digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  // while(steps > stepsForRequiredVolume){
  //   if(startEnabled == false) return; //EXIT IF START IS DISABLED
  //   if(!limitActived){
  //     slowStep(calculatedInspiratoryTime / stepsForRequiredVolume);
  //     steps--;
  //   } else {
  //     //ALARM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WE SHOULD NOT HIT LIMIT
  //   }
  // }
  // digitalWrite(STEPPER_DIR, STEPPER_DIR_UP);
  // while(steps < config.stepsUpperLimit){
  //   if(startEnabled == false) return; //EXIT IF START IS DISABLED
  //   slowStep(calculatedExpiratoryTime / stepsForRequiredVolume);
  //   steps++;
  // }
}





/**********************************************************************
* CONFIGURATION ROUTINES
**********************************************************************/


//####################################################################
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
  config.stepsUpperLimit = steps;

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



//####################################################################
/*
ZERO POTENTIOMETER
*/
// calibrate volume to VT pot
void zeroVolumePot() {
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


//####################################################################
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
  lcdDis = true;
  while (true) {
    //display message once
    if (lcdDis){
      clearLCD();
      lcd.print(F("*** ML increments ***"));
      lcd.setCursor (0, 1);
      lcd.print(F("Turn VT until 50 ml "));
      lcd.setCursor (0, 2);
      lcd.print(F("is reached."));
      lcd.setCursor (0, 3);
      lcd.print(F("Press OK when done"));
      lcdDis = false;
    }

    lcdDis = true;

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
          //errFlag = true;
          //don't reset limit here
          break;
        }
      }

      // display next step message if not done
      if (lcdDis){
        clearLCD();
        lcd.print(F("*** ML increments ***"));
        lcd.setCursor (0, 1);
        lcd.print(F("Set 50 ml increment?"));
        lcd.setCursor (0, 2);
        lcd.print(F("OK=Yes, CONF=No"));
        lcdDis = false;
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
      config.stepsToVolume[calibIndex] = (int)steps;
      //increment index for
      calibIndex++;

      //view array
      for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
        //show the value for each
        Serial.print(config.stepsToVolume[i]);
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
    if(calibIndex == STEP_TO_VOLUME_INCREMENTS){
      //calibration finished
      calibDone = true;
    }

    //return to set ML_CONFIG
    break;
  }

  //disable stepper
  digitalWrite(STEPPER_ENABLE, LOW);
}


//####################################################################


/**********************************************************************
* SETUP
**********************************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //inputs
  pinMode(OK_BTN_PIN, INPUT_PULLUP);
  pinMode(CONFIG_BTN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  //ISRs
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitTriggered_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(START_BTN_PIN), startTriggered_ISR, FALLING);

  //outputs
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_ENABLE, OUTPUT);
  pinMode(START_LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  //Initial states
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_DIR, STEPPER_DIR_DOWN);
  digitalWrite(STEPPER_ENABLE, LOW);
  digitalWrite(START_LED_PIN, LOW);
  digitalWrite(BUZZER, LOW);
  
  //initialise
  loadConfig();
  lcdInit();
  clearLCD();

  //Waiting mode
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
      lcdDis = true;


      break;



    /*
    STANDBY MODE
    */
    case STANDBY:
      //Serial.println("Standby Mode");
      if (lcdDis) {
        lcdDis = false;
        clearLCD();
        lcd.print(F("* Ventilator v0.90 *"));
        lcd.setCursor(0,2);
        lcd.print(F("Press OK to begin"));
        lcd.setCursor(0,3);
        lcd.print(F("Press CONFIG to cal"));
      }

      // Wait for user input
      handleBTN();

      //Config button pressed, go to config
      if (confBtnFlag) {
        confBtnFlag = false;
        //set config sequence start mode
        mode = VOL_CONFIG;
      }

      if (okBtnFlag) {
        okBtnFlag = false;
        //set ventilation mode
        mode = READY;
        lcdDis = true;
      }

      break;



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




    /*
    CALLIBRATE ML
    */
    case ML_CONFIG:
      Serial.println("ML Config Mode.");
      //reset arm
      zeroArm();
      limitActived = false;
      Serial.println("Resetting volume array.....");
      //reset volume increment array
      for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
        config.stepsToVolume[i] = (int)0;
        //Serial.println(i);
      }
      //reset calibration index
      calibIndex = 0;
      //while limit not reached
      while (!calibDone && !limitActived){
        //first zero pot by user
        zeroVolumePot();
        // Ok to proceed
        okBtnFlag = false;
        Serial.println("starting ML config");
        //turn pot until 50 ml
        mlConfig();
        //break out if hit bottom and calibration incomplete
        //shouldn't happen but just in case
        if (limitActived && !calibDone){
          Serial.println("LIMIT!!!");
          limitActived = false;
          break;
        }
        //if conf button pressed, incorrect calibration increment
        if(confBtnFlag){
          //reset flag
          confBtnFlag = false;
          //return to last valid increment
          resetToLast(config.stepsToVolume[calibIndex-1]);
        }
        //??
        limitActived = false;
      }
      //Check calibration complete
      if (calibDone){
        //save to EEPROM
        saveConfig();
        //display next message
        lcdDis = true;
        //enter ready mode
        mode = READY;
      }

      break;





    /*
    READY TO START
    */
    case READY:
      Serial.println("Ready Mode");

      if(lcdDis){
        //zero arm position
        zeroArm();

        //print message
        //clearLCD();
        lcd.print(F("***** READY!! ******"));
        lcd.setCursor (0, 1);
        lcd.print(F("                    "));
        lcd.setCursor (0, 2);
        lcd.print(F("                    "));
        lcd.setCursor (0, 3);
        lcd.print(F("Press START to start"));
        

        //reset calibration flag = CAREFUL!!
        lcdDis = false;
      }

      Serial.print("Array values: ");
      //display the increment array
      for (int i = 0; i < STEP_TO_VOLUME_INCREMENTS; i++){
        //show the value for each
        Serial.print(config.stepsToVolume[i]);
        Serial.print(" ,");
      }
      Serial.println();
      Serial.println();
      delay(1000);

      break;

    /*
    STARTED
    */
    case RUNNING:

      Serial.println("Running...");

      //read the 3 pot values
      Serial.print("Potentiometer values: ");
      volumeCurrent = (map(cleanRead(VOLUME_POT), 0, POT_MAX_VALUE, 0, STEP_TO_VOLUME_INCREMENTS) * VOLUME_INCREMENTS) + VOLUME_MIN;
      BPMCurrent = map(cleanRead(BREATHS_PER_MIN_POT), 0, POT_MAX_VALUE, BREATHS_PER_MIN_MIN, BREATHS_PER_MIN_MAX);
      IECurrent = map(cleanRead(IE_RATIO_POT), 0, POT_MAX_VALUE, IE_RATIO_MIN, IE_RATIO_MAX);

      Serial.print(volumeCurrent);
      Serial.print(", ");
      Serial.print(BPMCurrent);
      Serial.print(", ");
      Serial.println(IECurrent);

      //set the global breath settings (Volume, BPM, and I/E)
      handleSettings();

      //update screen
      handleScreen();

      //check buttons
      handleBTN();

      delay(1000);
     
      // Take one breath
      breath();      

      break;
      




    /*
    ERROR MODE
    */
    case ERR:
      Serial.println("ERROR!!!!!");

      //zero arm and wait for reset
      if (errFlag == true){
        zeroArm();
        errFlag = false;
      }

      break;



    default:
      Serial.println("default");
  }


}
