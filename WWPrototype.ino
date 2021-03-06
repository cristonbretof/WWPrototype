#include <stdint.h>

#include <CircularBuffer.h>
#include <LiquidCrystal.h>

#define LED_PIN      13       // GPIO pin (on-board LED)

#define V0           2.5      // Voltage refering to home position of platform
#define Vlim_Up      4        // Maximum allowed voltage to provide to amplifier

#define pinADC       A0       // Analog pin to read position input
#define ADC_RES      1023     // Maximum resolution of analog to digital converter
#define ADC_VREF     5        // ADC reference voltage

#define pinCURRENT   A1       // 

#define pinPWM       3        // PWM pin to control amplifier (basic PWM)

#define PRESCALER    256      // Prescaler digital value
#define BOARD_Freq   16000000 // Arduino Mega 2560 board frequency in Hz
#define MAX_FREQ     129      // Absolute maximum frequency
#define FREQUENCY    100      // Currently used frequency

#define Kp           1        // Proportional gain
#define Ki           2        // Integral gain
#define Kd           1        // Differential gain

#define pinRS        7       
#define pinE         8
#define pinD4        9
#define pinD5        10
#define pinD6        11
#define pinD7        12

#define NUM_TYPES       7
#define NUM_UNITS       2

#define NUM_BUTTONS     4

#define pinBUTTON_UNIT  16   
#define pinBUTTON_TYPE  15
#define pinBUTTON_MENU  14
#define pinBUTTON_TARE  17

#define BUF_LEN         20

#define NUM_ETALONS     11

#define RES_BOBINE      1.425

typedef enum {
  CONFIG_STATE = 0,
  SCALE_STATE = 1,
  BENCHMARK_STATE = 2,
  PROCESS_STATE = 3
} state_t;

typedef enum {
  MODE_NORMAL = 0,
  MODE_MOYENNAGE = 1,
  MODE_ETALONNAGE = 2
} mode_t;

/* Symbol arrays */
String typeArray[NUM_TYPES] = {"1¢","5¢","10¢","25¢","1$","2$","Ø"};
String unitArray[NUM_UNITS] = {"Oz","g"};

float massTypeGram[NUM_TYPES] = {2.35, 3.95, 1.75, 4.4, 6.9, 6.27, 6.92};
float massTypeOunce[NUM_TYPES] = {0.07054792, 0.1058219, 0.03527396, 0.141096, 0.211644, 0.2116438, 0.2440958};

/* Indices for type and unit ISRs */
uint8_t unit_index = 0;
uint8_t type_index = 0;

/* Initial slope for current to mass conversion */
float penteMasseCourant = 1; 
uint8_t tabEtalons[NUM_ETALONS] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

/* Main data configurations */
float massTare = 0;
float currMass = 0;
uint8_t currNumCoins = 0;

/* Pointer to current state */
void (*statePtr)(void);
static state_t currentState = CONFIG_STATE;
static mode_t selectedMode = MODE_NORMAL;

/* Array of all button pins */
const uint8_t inputPins[NUM_BUTTONS] = {pinBUTTON_MENU,pinBUTTON_TYPE,pinBUTTON_UNIT,pinBUTTON_TARE};

/* Error values declarations */
static float int_err = 0;
static float prev_err = 0;

/* Function declarations */
static void processState(void);
static void configState(void);
static void scaleState(void);
static float apply_PID(float Vin);
void setupTimer();

/* Up and down LCD character definitions */
byte up[8] = {
  B00100,
  B01110,
  B11111,
  B01110,
  B01110,
  B00000,
  B00000,
};
byte down[8] = {
  B00000,
  B00000,
  B01110,
  B01110,
  B11111,
  B01110,
  B00100,
};

CircularBuffer<float, BUF_LEN> scaleBuffer;
CircularBuffer<uint16_t, NUM_ETALONS> benchmarkBuffer;

/* Initialize LCD board */
LiquidCrystal lcd = LiquidCrystal(pinRS, pinE, pinD4, pinD5, pinD6, pinD7);

///////////////////////
// ARDUINO FONCTIONS //
///////////////////////

void setup() {
  Serial.begin(9600);

  /* Create both up and down arrows */
  lcd.createChar(0, up);
  lcd.createChar(1, down);

  /* Initialize all button pins to interrupt */
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_MENU), ISR_menuSelect, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_TYPE), ISR_buttonA, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_UNIT), ISR_buttonB, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_TARE), ISR_tare, FALLING);

  /* Start the LCD display */
  Serial.println("LCD init");
  lcd.begin(16, 2);

  statePtr = scaleState;

  setupTimer();
}

void loop()
{
  statePtr();
}

///////////////////////
//  PRINT FONCTIONS  //
///////////////////////
void printFirstLine(String unit)
{
  lcd.setCursor(0,0);
  lcd.print("MASSE: " currMass " " unit);
}

void printSecondLine(String type)
{
  lcd.setCursor(1,0);
  lcd.print(currNumCoins " x " type);
}

void printArrowUp()
{
  lcd.setCursor(0,15);
  lcd.write(byte(1));
}

void printArrowDown()
{
  lcd.setCursor(1,15);
  lcd.write(byte(0));
}

void eraseArrowUp()
{
  lcd.setCursor(0,15);
  lcd.write(" ");
}

void eraseArrowDown()
{
  lcd.setCursor(1,15);
  lcd.write(" ");
}

///////////////////////
// UTILITY FONCTIONS //
///////////////////////

/**
   @brief Fonction appliquant le PID sur les intrants

   @details Cette fonction prend comme paramètre la tension de sortie du
            capteur de position. Elle prend aussi comme argument la tension
            de référence qui décrit la position x0 de la lame.
*/
static float apply_PID(float Vin)
{
  float output;
  
  // Calculate positionning error
  float err = V0 - Vin;

  // Record previous integral error to allow reset in case of windup
  float temp_int = int_err;

  // Integrate error (sum it with previous errors)
  int_err += err;

  // Calculate differential error
  float diff_err = err - prev_err;
  prev_err = err;

  // Calculate portions of PID
  float proportional_part = Kp * err;
  float integral_part = (Ki * int_err * (1 / FREQUENCY));
  float differential_part = (Kd * diff_err / (1 / FREQUENCY));

  output = proportional_part + integral_part + differential_part;
  if (output >= Vlim_Up)
  {
    // Set value to absolute max
    integral_part = (Ki * temp_int * (1 / FREQUENCY));
    output = proportional_part + integral_part + differential_part;
  }
  // Convert value to 16 bit integer and send to PWM
  uint8_t dig_output = (uint8_t)output;
  analogWrite(pinPWM, dig_output);
  return output;
}

void setupTimer() {
  /* Disable interrupts */
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  /* Set compare register for desired frequency */
  OCR1A = (int)(BOARD_Freq/PRESCALER/FREQUENCY-1);
  Serial.println(OCR1A);

  TCCR1B |= (1 << WGM12);  // Turn on CTC mode
  TCCR1B |= (1 << CS12);   // Set prescaler to 8
  
  TIMSK1 |= (1 << OCIE1A); // Enable timer for compare interrupt

  /* Reenable interrupts */
  interrupts();
}

void incrementBenchmark(uint16_t val)
{
  benchmarkBuffer.push(val);
  if (benchmarkBuffer.isFull())
  {
    currentState = PROCESS_STATE;
    statePtr = processState;
  }
}

/////////////////////
// STATE FUNCTIONS //
/////////////////////

static void processState(void)
{
  lcd.clear();
  
  /* Calcul de la pente pour le calcul de la masse */
  
  /*float tabCourant[NUM_ETALONS];                                // Array contenant les valeur le l'ADC converties en courant
  for(int i = 0; i < NUM_ETALONS-1; i++)                          // Si jamais les profs veulent le courant
  {                                                              
    tabCourant[i] = ((benchmarkBuffer[i]*5)/1023)/RES_BOBINE;
  }*/

  float tabVoltage[NUM_ETALONS];
  for(int i = 0; i < NUM_ETALONS-1; i++) 
  {                                                              
    tabCourant[i] = (benchmarkBuffer[i]*5)/1023;
  }

  float tabPentes[NUM_ETALONS];                                 //Array pour stocker le coefficient pour chaque couple
  for(int i = 0; i < NUM_ETALONS-1; i++)
  {
    tabPentes[i] = tabEtalons[i]/tabVoltages[i];
  }

  float sommePentes = 0;
  for(int i = 0; i < NUM_ETALONS; i++)                         // Moyenne des coefficients
  {
    sommePentes = sommePentes + tabPentes[i];
  }

  penteMasseCourant = sommePentes/NUM_ETALONS;
}

static void configState(void)
{
  
}

static void benchmarkState(void)
{
  
}

static void scaleState(void)
{
  float Vin = 0;
  float output;
  if (!scaleBuffer.isEmpty())
  {
    Vin = scaleBuffer.pop();
    output = apply_PID(Vin);
    lcd.setCursor(0,0);
    lcd.print("PWM = "); lcd.print(output,4);
    lcd.setCursor(0,1);
    lcd.print("BUFFER LINE");
    Serial.println(scaleBuffer.size());
  }
}

/////////////////////
//    INTERRUPTS   //
/////////////////////

/**
   @brief Timer d'échantillonnage de la tension en entrée

   @details Cette routine prend en charge la captation de la valeur de l'ADC
            (position) au temps = Ts. Il recharge le timer pour qu'il y ait
            une autre interruption et il place la valeur de l'ADC
*/
ISR(TIMER1_COMPA_vect)
{
  unsigned short val = 0;
//  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  val = analogRead(pinADC);
  scaleBuffer.push((float)(val)*ADC_VREF/ADC_RES);
}

void ISR_menuSelect(void) // Ou select
{
  if(currentState == SCALE_STATE)
  {
    currentState = CONFIG_STATE;
    statePtr = configState;
  }
  else if(currentState == CONFIG_STATE)
  {
    if(selectedMode == MODE_ETALONNAGE)
    {
      currentState = BENCHMARK_STATE;
      statePtr = benchmarkState;
    }
    else
    {
      currentState = SCALE_STATE;
      statePtr = scaleState;
    }
  }
  else
  {
    currentState = CONFIG_STATE;
    statePtr = configState;
  }
}

void ISR_buttonA(void) // Ou down
{
  if(currentState == SCALE_STATE)
  {
    type_index++;
    printSecondLine(typeArray[type_index]);
    if (type_index == NUM_TYPES-1)
    {
      type_index = 0;
    }
  }
  else if(currentState == CONFIG_STATE)
  {
    if(selectedMode == MODE_NORMAL)
    {
      eraseArrowUp();
      printArrowDown();
    }
    else if (selectedMode == MODE_MOYENNAGE)
    {
      printArrowUp();
      printArrowDown();
    }
    else if (selectedMode == MODE_ETALONNAGE)
    {
      eraseArrowDown();
      printArrowUp();
    }
  }
}

void ISR_buttonB(void) // Ou up
{
  if(currentState == SCALE_STATE)
  {
    unit_index++;
    printFirstLine(unitArray[unit_index]);
    if (unit_index == NUM_UNITS-1)
    {
      unit_index = 0;
    }
  }
  else if(currentState == CONFIG_STATE)
  {
    if(selectedMode == MODE_NORMAL)
    {
      eraseArrowUp();
      printArrowDown();
    }
    else if (selectedMode == MODE_MOYENNAGE)
    {
      printArrowUp();
      printArrowDown();
    }
    else if (selectedMode == MODE_ETALONNAGE)
    {
      eraseArrowDown();
      printArrowUp();
    }
  }
}

void ISR_tare(void)
{
  if(currentSate == SCALE_STATE)
  {
    massTare = currMass;
  }
  else if (currentState == BENCHMARK_STATE)
  {
    uint16_t val = analogRead(pinCURRENT);
    incrementBenchmark(val);
  }
}
