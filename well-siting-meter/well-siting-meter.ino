#include <LiquidCrystal.h>

// pin definitions
#define RS_PIN    2
#define VO_PIN    3
#define B1_PIN    4
#define B2_PIN    5
#define ISW_PIN   8
#define VSW_PIN   9
#define DB7_PIN   10
#define DB6_PIN   11
#define DB5_PIN   12
#define DB4_PIN   13
#define IOUT_PIN  A0
#define VOUT_PIN  A1
#define SHDN_PIN  A2
#define EN_PIN    A3

// other variables
#define ADC_MAX   1023.0f
#define I_GAIN    49.78f
#define V_GAIN    2.0f

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
  //lcd.print("hello, world!");

  // setup lcd vo
  pinMode(VO_PIN, OUTPUT);
  analogWrite(VO_PIN, 64);
  
  // setup buttons
  pinMode(B1_PIN, INPUT_PULLUP);
  pinMode(B2_PIN, INPUT_PULLUP);

  // setup switches
  pinMode(ISW_PIN, OUTPUT);
  digitalWrite(ISW_PIN, LOW); 
  pinMode(VSW_PIN, OUTPUT);
  digitalWrite(VSW_PIN, LOW); 

  // setup boost shutdown
  pinMode(SHDN_PIN, OUTPUT);
  digitalWrite(SHDN_PIN, LOW);

  // setup analoginputs
  pinMode(IOUT_PIN, INPUT);
  pinMode(VOUT_PIN, INPUT);
  //pinMode(SHDN_PIN, INPUT);
}

/*************************************************************************************************************
  loop
*************************************************************************************************************/
void loop() {
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
  delay(200);
}

/*************************************************************************************************************
  convert
*************************************************************************************************************/
float convert(int adc, float gain) {
  return ((float)adc - ADC_MAX/2.0)*5.0/ADC_MAX/gain;
}

