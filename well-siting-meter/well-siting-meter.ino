#include <LiquidCrystal.h>

#define DEBUG

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
#define V_GAIN          2.0f
#define I_GAIN          49.78f
#define R_SENSE         0.05f
#define NUM_MEAS        10

// timers
#define WAIT_TIMEOUT    10
#define INIT_TIMEOUT    3000
#define MEAS_TIMEOUT    25
#define STOP_TIMEOUT    2000
#define BOOST_TIMEOUT   500
#define DIR_TIMEOUT     100

// button state
enum {
  DOWN  = LOW,
  UP    = HIGH
};

// boost state
enum {
  OFF   = LOW,
  ON    = HIGH
};

// dir state
enum {
  FWD   = LOW,
  REV   = HIGH
};

// system state
enum {
  INIT,
  START,
  WARNING,
  GROUND,
  FORWARD,
  REVERSE,
  STOP,
  RESULTS,
  NUM_STATES
};
int state;

// initialize lcd
LiquidCrystal lcd(RS_PIN, EN_PIN, DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

// results
float vgnd, vfwd, vrev, ifwd, irev, vout, iout;

/*************************************************************************************************************
  setup
*************************************************************************************************************/
void setup() {
#ifdef DEBUG
  // open serial port
  Serial.begin(9600);
  while(!Serial){};
  
  Serial.println("setup()");
#endif

  // setup lcd
  lcd.begin(16, 2);

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
  reset(INIT);
}

/*************************************************************************************************************
  loop
*************************************************************************************************************/
void loop() {
#ifdef DEBUG
  Serial.println("loop()");
#endif

#ifdef DEBUG
  Serial.println(String("state = ") + state);
#endif
  
  // display switch
  switch(state) {
    case(INIT):
      // update state
      lcd.clear();
      lcd.print("well siting");
      lcd.setCursor(0, 1);
      lcd.print("meter (v2)");

      // wait, then transition
      wait(B2_PIN, INIT_TIMEOUT);
      state = START;
      break;
    case(START):
      // update state
      lcd.clear();
      lcd.print("press button 1");
      lcd.setCursor(0, 1);
      lcd.print("to start...");

      // wait for button press
      if(wait(B1_PIN, -1) <= 0) {
        state = WARNING;
      }
    case(WARNING):
      // update state
      lcd.clear();
      lcd.print("starting in:");
      lcd.setCursor(0, 1);

      // count down
      for(int i = 3; i > 0; i--) {
        lcd.print(i);
        lcd.print("... ");

        // check for button interrupt
        if(wait(B2_PIN, 1000) <= 0) {
          reset(START);
          return;
        }
      }

      // transition
      state = GROUND;
      break;
    case(GROUND):
      lcd.clear();
      lcd.print("ground meas");
      lcd.setCursor(0, 1);
      lcd.print("|###           |");

      // measure gnd
      float vgfwd, vgrev, _;
      if(meas(OFF, FWD, vgfwd, _) <= 0) {
        reset(START);
        return;
      }
      if(meas(OFF, REV, vgrev, _) <= 0) {
        reset(START);
        return;
      }
      vgnd = (vgfwd + vgrev)/2.0f;

#ifdef DEBUG
    Serial.println(String("vgnd = ") + vgnd);
#endif
      
      // transition
      state = FORWARD;
      break;
    case(FORWARD):
      lcd.clear();
      lcd.print("forward meas");
      lcd.setCursor(0, 1);
      lcd.print("|#######       |");

      // measure fwd
      if(meas(ON, FWD, vfwd, ifwd) <= 0) {
        reset(START);
        return;
      }

#ifdef DEBUG
    Serial.println(String("vfwd = ") + vfwd + ", ifwd = " + ifwd);
#endif

      // transition
      state = REVERSE;
      break;
    case(REVERSE):
      lcd.clear();
      lcd.print("reverse meas");
      lcd.setCursor(0, 1);
      lcd.print("|###########   |");

      // measure rev
      if(meas(ON, REV, vrev, irev) <= 0) {
        reset(START);
        return;
      }

#ifdef DEBUG
    Serial.println(String("vrev = ") + vrev + ", irev = " + irev);
#endif

      // transition
      state = STOP;
      break;
    case(STOP):
      lcd.clear();
      lcd.print("shutting down");
      lcd.setCursor(0, 1);
      lcd.print("|##############|");

      // wait and transition
      wait(0, STOP_TIMEOUT);
      state = RESULTS;
      break;
    case(RESULTS):
      // calculate results
      vout = (vfwd - vrev)/2.0f;
      iout = (ifwd + irev)/2.0f;
      
#ifdef DEBUG
    Serial.println(String("vout = ") + vout + ", iout = " + iout);
#endif

      lcd.clear();
      lcd.print("vout = ");
      lcd.setCursor(8, 0);
      lcd.print(vout);
      lcd.setCursor(0, 1);
      lcd.print("iout = ");
      lcd.setCursor(8, 1);
      lcd.print(iout);

      // wait for button press
      if(wait(B2_PIN, -1) <= 0) {
        reset(START);
        return;
      }
      break;
    default:
      reset(START);
      return;
  }
}

/*************************************************************************************************************
  meas
*************************************************************************************************************/
int meas(int boost, int dir, float &vval, float &ival) {
#ifdef DEBUG
  Serial.println(String("meas(") + boost + ", " + dir + ", " + vout + ", " + iout + ")");
#endif
  static int n = NUM_MEAS*2;
  
  // check dir
  if(digitalRead(ISW_PIN) != dir) {
    digitalWrite(ISW_PIN, dir);

#ifdef DEBUG
    Serial.println(String("isw = ") + dir);
#endif

    // wait for switch
    if(wait(B2_PIN, DIR_TIMEOUT) <= 0) {
      return 0;
    }
  }
  
  if(boost == ON) {
    // enable boost
    digitalWrite(SHDN_PIN, LOW);

#ifdef DEBUG
    Serial.println("boost = ON");
#endif

    // wait for voltage to stabilize
    if(wait(B2_PIN, BOOST_TIMEOUT) <= 0) {
      return 0;
    }
  }

  // measurement vars
  float vfmeas[NUM_MEAS], vrmeas[NUM_MEAS];
  float imeas[n];

  // setup fwd
  if(digitalRead(VSW_PIN) != FWD) {
    digitalWrite(VSW_PIN, FWD);

#ifdef DEBUG
    Serial.println("vsw = FWD");
#endif

    // wait for switch
    if(wait(B2_PIN, DIR_TIMEOUT) <= 0) {
      return 0;
    }
  }

  // fwd meas
  for(int i = 0; i < NUM_MEAS; i++) {
    // read and convert
    analogRead(VOUT_PIN);
    if(wait(B2_PIN, MEAS_TIMEOUT) <= 0) {
      return 0;
    }
    vfmeas[i] = convert(VOUT_PIN, V_GAIN);


    // read and convert
    analogRead(IOUT_PIN);
    if(wait(B2_PIN, MEAS_TIMEOUT) <= 0) {
      return 0;
    }
    imeas[i] = convert(IOUT_PIN, I_GAIN)/R_SENSE;

#ifdef DEBUG
    Serial.println(String("") + i + ": vfmeas = " + vfmeas[i] + ", imeas = " + imeas[i]);
#endif
  }

  // setup rev
  digitalWrite(VSW_PIN, REV);

#ifdef DEBUG
    Serial.println("vsw = REV");
#endif

  // wait for switch
  if(wait(B2_PIN, DIR_TIMEOUT) <= 0) {
    return 0;
  }

  // rev meas
  for(int i = 0; i < NUM_MEAS; i++) {
    // read and convert
    analogRead(VOUT_PIN);
    if(wait(B2_PIN, MEAS_TIMEOUT) <= 0) {
      return 0;
    }
    vrmeas[i] = convert(VOUT_PIN, V_GAIN);


    // read and convert
    analogRead(IOUT_PIN);
    if(wait(B2_PIN, MEAS_TIMEOUT) <= 0) {
      return 0;
    }
    imeas[i + NUM_MEAS] = convert(IOUT_PIN, I_GAIN)/R_SENSE;

#ifdef DEBUG
    Serial.println(String("") + i + ": vrmeas = " + vrmeas[i] + ", imeas = " + imeas[i + NUM_MEAS]);
#endif
  }

  if(boost == ON) {
    // disable boost
    digitalWrite(SHDN_PIN, HIGH);

#ifdef DEBUG
    Serial.println("boost = OFF");
#endif
  }

  // average
  float vfsum = 0.0f;
  float vrsum = 0.0f;
  float isum = 0.0f;
  for(int i = 0; i < n; i++) {
    // sum vmeas
    if(i < NUM_MEAS) {
      vfsum += vfmeas[i];
    }
    else {
      vrsum += vrmeas[i - NUM_MEAS];
    }

    // sum imeas
    isum += imeas[i];
  }
  float vfwd = vfsum/NUM_MEAS;
  float vrev = vrsum/NUM_MEAS;
  ival = isum/n;

  // check for voltage dir
  if(vfwd >= vrev) {
    vval = vfwd;
  }
  else {
    vval = -vrev;
  }

#ifdef DEBUG
    Serial.println(String("vfwd = ") + vfwd + ", vrev = " + vrev);
    Serial.println(String("vval = ") + vval + ", ival = " + ival);
#endif

  return 1;
}

/*************************************************************************************************************
  reset
*************************************************************************************************************/
void reset(int next) {
#ifdef DEBUG
  Serial.println(String("reset(") + next + ")");
#endif

  digitalWrite(SHDN_PIN, HIGH);
  digitalWrite(ISW_PIN, FWD);
  digitalWrite(VSW_PIN, FWD);

  vgnd = 0;
  vfwd = 0;
  vrev = 0;
  ifwd = 0;
  irev = 0;
  vout = 0;
  iout = 0;
  
  state = next;
}

/*************************************************************************************************************
  wait
*************************************************************************************************************/
int wait(int button, int timeout) {
#ifdef DEBUG
  Serial.println(String("wait(") + button + ", " + timeout + ")");
#endif
  // if we're waiting for a button press, check periodically
  if(button > 0) {
    // set timer
    unsigned long timer = millis();

    // wait loop
    int prev = DOWN;
    while((millis() - timer) < timeout) {
      // check for button down
      int val = digitalRead(button);
      if((val == DOWN) && (prev == UP)) {
        return 0;
      }
      prev = val;

      // delay 
      delay(WAIT_TIMEOUT);
    }

    return 1;
  }
  // otherwise just delay and return
  else {
    delay(timeout);
    return 1;
  }
}

/*************************************************************************************************************
  convert
*************************************************************************************************************/
float convert(int pin, float gain) {
  int adc;
#ifdef DEBUG
  Serial.println(String("convert(") + pin + ", " + gain + ")");
#endif

  // read once to set multiplexor
  adc = analogRead(pin);
#ifdef DEBUG
    Serial.println(String("adc = ") + adc);
#endif

  // convert
  return ((float)adc - ADC_MAX/2.0f)*5.0f/ADC_MAX/gain;
}

