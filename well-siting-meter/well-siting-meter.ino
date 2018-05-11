#include <LiquidCrystal.h>

// pin definitions
#define RS_PIN          2
#define VO_PIN          3
#define B1_PIN          4
#define B2_PIN          5
#define ISW_PIN         8
#define VSW_PIN         9
#define DB7_PIN         10
#define DB6_PIN         11
#define DB5_PIN         12
#define DB4_PIN         13
#define IOUT_PIN        A0
#define VOUT_PIN        A1
#define SHDN_PIN        A2
#define EN_PIN          A3

// other variables
#define ADC_MAX         1023.0f
#define I_GAIN          49.78f
#define V_GAIN          2.0f

// timers
#define WAIT_TIMEOUT    10
#define INIT_TIMEOUT    3000
#define MEAS_TIMEOUT    1000
#define STOP_TIMEOUT    2000

// screen setup
enum {
  SCREEN_INIT,
  SCREEN_START,
  SCREEN_WARNING,
  SCREEN_GROUND,
  SCREEN_FORWARD,
  SCREEN_REVERSE,
  SCREEN_STOP,
  SCREEN_DISPLAY,
  NUM_SCREENS
};
int screen;

// initialize lcd
LiquidCrystal lcd(RS_PIN, EN_PIN, DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

/*************************************************************************************************************
  setup
*************************************************************************************************************/
void setup() {
  // open serial port
  Serial.begin(9600);

  // setup lcd
  lcd.begin(16, 2);

  // screen
  screen = SCREEN_INIT;

  // setup lcd vo
  pinMode(VO_PIN, OUTPUT);
  analogWrite(VO_PIN, 64);

  // setup buttons
  pinMode(B1_PIN, INPUT_PULLUP);
  pinMode(B2_PIN, INPUT_PULLUP);

  // setup switches
  pinMode(ISW_PIN, OUTPUT);
  pinMode(VSW_PIN, OUTPUT);

  // setup boost shutdown
  pinMode(SHDN_PIN, OUTPUT);

  // setup analoginputs
  pinMode(IOUT_PIN, INPUT);
  pinMode(VOUT_PIN, INPUT);

  // reset pin states
  reset();
}

/*************************************************************************************************************
  loop
*************************************************************************************************************/
void loop() {
  // display switch
  switch(screen) {
    case(SCREEN_INIT):
      // update screen
      lcd.clear();
      lcd.print("well siting");
      lcd.setCursor(0, 1);
      lcd.print("meter (v2)");

      // wait, then transition to start
      wait(0, INIT_TIMEOUT);
      screen = SCREEN_START;
      break;
    case(SCREEN_START):
      // update screen
      lcd.clear();
      lcd.print("press button 1");
      lcd.setCursor(0, 1);
      lcd.print("to start...");

      // wait for button press
      if(wait(B1_PIN, -1)) {
        screen = SCREEN_WARNING;
      }
    case(SCREEN_WARNING):
      // update screen
      lcd.clear();
      lcd.print("starting in:");
      lcd.setCursor(0, 1);

      // count down
      for(int i = 3; i > 0; i--) {
        lcd.print(i);
        lcd.print("... ");

        // check for button interrupt
        if(wait(B2_PIN, 1000)) {
          reset();
          screen = SCREEN_START;
        }
      }

      // transition
      screen = SCREEN_GROUND;
      break;
    case(SCREEN_GROUND):
      lcd.clear();
      lcd.print("ground meas");
      lcd.setCursor(0, 1);
      lcd.print("|###");

      // check for button interrupt
      if(wait(B2_PIN, MEAS_TIMEOUT)) {
        reset();
        screen = SCREEN_INIT;
      }
      else {
        screen = SCREEN_FORWARD;
      }
      break;
    case(SCREEN_FORWARD):
      lcd.clear();
      lcd.print("forward meas");
      lcd.setCursor(0, 1);
      lcd.print("|#######");

      // check for button interrupt
      if(wait(B2_PIN, MEAS_TIMEOUT)) {
        reset();
        screen = SCREEN_INIT;
      }
      else {
        screen = SCREEN_REVERSE;
      }
      break;
    case(SCREEN_REVERSE):
      lcd.clear();
      lcd.print("reverse meas");
      lcd.setCursor(0, 1);
      lcd.print("|###########");

      // check for button interrupt
      if(wait(B2_PIN, MEAS_TIMEOUT)) {
        reset();
        screen = SCREEN_INIT;
      }
      else {
        screen = SCREEN_STOP;
      }
      break;
    case(SCREEN_STOP):
      lcd.clear();
      lcd.print("shutting down");
      lcd.setCursor(0, 1);
      lcd.print("|##############|");

      // wait and transition
      wait(0, STOP_TIMEOUT);
      screen = SCREEN_DISPLAY;
      break;
    case(SCREEN_DISPLAY):
      lcd.clear();
      lcd.print("finished!");
      lcd.setCursor(0, 1);
      lcd.print("press button 2");

      // wait for button press
      if(wait(B2_PIN, -1)) {
        screen = SCREEN_START;
      }
      break;
    default:
      screen = SCREEN_INIT;
      break;
  }
  
  /*
  // read button states
  Serial.print("b1 = ");
  Serial.println(digitalRead(B1_PIN));
  Serial.print("b2 = ");
  Serial.println(digitalRead(B2_PIN));

  // read voltages
  Serial.print("iout = ");
  float iout = convert(analogRead(IOUT_PIN), I_GAIN);
  Serial.println(iout);
  //Serial.println(analogRead(IOUT_PIN));
  Serial.print("vout = ");
  float vout = convert(analogRead(VOUT_PIN), V_GAIN);
  Serial.println(vout);
  //Serial.println(analogRead(VOUT_PIN));
  //Serial.print("shdn = ");
  //Serial.println(analogRead(SHDN_PIN));

  // switch pins
  //digitalWrite(ISW_PIN, !digitalRead(ISW_PIN));
  //digitalWrite(VSW_PIN, !digitalRead(VSW_PIN));
  //digitalWrite(SHDN_PIN, !digitalRead(SHDN_PIN));

  // print a line
  Serial.println("");

  // print to lcd
  lcd.setCursor(0, 0);
  lcd.print("vout:");
  lcd.setCursor(8, 0);
  lcd.print(analogRead(IOUT_PIN));
  lcd.setCursor(0, 1);
  lcd.print("iout:");
  lcd.setCursor(8, 1);
  lcd.print(iout);

  // delay for a bit
  delay(1000);
  */
}

/*************************************************************************************************************
  reset
*************************************************************************************************************/
void reset(void) {
  digitalWrite(ISW_PIN, LOW);
  digitalWrite(VSW_PIN, LOW);
  digitalWrite(SHDN_PIN, HIGH);
}

/*************************************************************************************************************
  wait
*************************************************************************************************************/
int wait(int button, int timeout) {
  // if we're waiting for a button press, check periodically
  if(button) {
    // set timer
    unsigned long timer = millis();

    // wait loop
    while((millis() - timer) < timeout) {
      // check for button down
      if(!digitalRead(button)) {
        return 1;
      }

      // delay 
      delay(WAIT_TIMEOUT);
    }

    return 0;
  }
  // otherwise just delay and return
  else {
    delay(timeout);
    return 0;
  }
}

/*************************************************************************************************************
  convert
*************************************************************************************************************/
float convert(int adc, float gain) {
  return ((float)adc - ADC_MAX / 2.0) * 5.0 / ADC_MAX / gain;
}

