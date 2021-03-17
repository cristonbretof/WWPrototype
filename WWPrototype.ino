#include <stdint.h>

#include <CircularBuffer.h>
#include <LiquidCrystal.h>

#include <Wire.h> //Library for DAC

#define LED_PIN      13       // GPIO pin (on-board LED)

#define V0           2.5      // Voltage refering to home position of platform
#define Vlim_Up      2        // Maximum allowed voltage to provide to amplifier


#define pinADC       A15       // Analog pin to read position input
#define ADC_RES      1023     // Maximum resolution of analog to digital converter
#define VREF         5        // ADC reference voltage

#define pinCURRENT   A14       // 
#define PWM_PIN      12        // PWM pin to control amplifier (basic PWM)

#define PRESCALER    256      // Prescaler digital value
#define BOARD_Freq   16000000 // Arduino Mega 2560 board frequency in Hz
#define MAX_FREQ     129      // Absolute maximum frequency
#define FREQUENCY    50      // Currently used frequency

#define MASS_ERROR   0.3
#define GRAMS_TO_OZ  0.035274

#define Kp           0.01        // Proportional gain
#define Ki           0.2        // Integral gain
#define Kd           0.001        // Differential gain

#define pinRS        24      
#define pinE         26
#define pinD4        28
#define pinD5        30
#define pinD6        32
#define pinD7        34

#define NUM_TYPES    7
#define NUM_UNITS    2

#define DAC_RES      4096     // Maximum resolution of digital to analog converter
#define DAC_VREF     5        // DAC reference voltage
#define OUTPUT_MAX   2        // Offre une protection sur la valeur appliquer au DAC.
#define OUTPUT_MIN   0        // Offre une protection sur la valeur appliquer au DAC.

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0. 
#define MCP4725_ADDR 0x60   
//For devices with A0 pulled HIGH, use 0x61


#define NUM_BUTTONS     4

#define pinBUTTON_A  18  
#define pinBUTTON_B  2
#define pinBUTTON_MENU  19
#define pinBUTTON_TARE  3

#define BUF_LEN         20
#define NUM_ETALONS     11
#define AVG_SAMPLES     3

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
String unitArray[NUM_UNITS] = {"oz","g"};

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
float prevMass = 0;
uint8_t currNumCoins = 0;

/* Unité et type actifs sur l'affichage */
String currentUnit;
String currentType;

/* État actuel de l'étalonnage */
uint16_t currentStep = 0;
uint8_t step_index = 0;

/* Pointer to current state */
void (*statePtr)(void);
static state_t currentState = CONFIG_STATE;
static mode_t selectedMode = MODE_NORMAL;

/* Array of all button pins */
const uint8_t inputPins[NUM_BUTTONS] = {pinBUTTON_MENU,pinBUTTON_B,pinBUTTON_A,pinBUTTON_TARE};

/* Error values declarations */
static float int_err = 0;
static float prev_err = 0;

/* Dernier Output */
float last_output = 0;

/* Function declarations */
static void processState(void);
static void configState(void);
static void scaleState(void);
static float apply_PID(float Vin);
void setupTimer();

/* Up and down LCD character definitions */
byte up[8] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B01110,
  B01110,
  B00000,
  B00000
};
byte down[8] = {
  B00000,
  B00000,
  B01110,
  B01110,
  B11111,
  B01110,
  B00100,
  B00000
};
byte full[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte stable[8] = {
  B00000,
  B00000,
  B01010,
  B01010,
  B00000,
  B10001,
  B01110,
  B00000
};


CircularBuffer<uint16_t, AVG_SAMPLES> avgBuffer;
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
  lcd.createChar(2, full);
  lcd.createChar(3, stable);

  /* Initialize all button pins to interrupt */
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_MENU), ISR_menuSelect, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_B), ISR_buttonB, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_A), ISR_buttonA, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_TARE), ISR_tare, FALLING);

  /* Start the LCD display */
  lcd.begin(16, 2);

  /*  */
  currentState = CONFIG_STATE;
  statePtr = configState;

  setupTimer();

  /* Start CAD */
  Wire.begin() ;
}

void loop()
{
  statePtr();
}


///////////////////////
//  PRINT FONCTIONS  //
///////////////////////
void printScaleFirstLine(void)
{
  lcd.setCursor(0,0);
  lcd.print("MASSE: ");
  lcd.print(currMass,2);
  lcd.print(" ");
  lcd.print(currentUnit);
}

void printScaleSecondLine(void)
{
  lcd.setCursor(0,1);
  lcd.print(currNumCoins);
  lcd.print(" x ");
  lcd.print(currentType);
}

void printMenuConfig(void)
{
  switch(selectedMode)
  {
    case MODE_NORMAL:
      lcd.clear();
      
      eraseArrowUp();
      printArrowDown();
      
      lcd.setCursor(0,0);
      lcd.print("MODE NORMAL");
      break;
    case MODE_MOYENNAGE:
      lcd.clear();
      
      printArrowUp();
      printArrowDown();
      
      lcd.setCursor(0,0);
      lcd.print("MODE MOYENNAGE");
      break;
    case MODE_ETALONNAGE:
      lcd.clear();
      
      printArrowUp();
      eraseArrowDown();
      
      lcd.setCursor(0,0);
      lcd.print("ETALONNAGE");
      break;
  }
}

void printStabilitySymbol(void)
{
  lcd.setCursor(15,1);
  lcd.write(byte(3));
}

void printBenchmarkSteps(void)
{
    lcd.clear();   
    lcd.setCursor(0,0);
    if (currentStep != 0)
    {
      lcd.print("Posez une masse");
      lcd.setCursor(0,1);
      lcd.print("de : ");
      lcd.print(currentStep);
      lcd.print("g");
    }
    else
    {
      lcd.print("Retirez le poids");
      lcd.setCursor(0,1);
      lcd.print("de la balance");
    }
}

void printArrowUp()
{
  lcd.setCursor(15,0);
  lcd.write(byte(0));
}

void printArrowDown()
{
  lcd.setCursor(15,1);
  lcd.write(byte(1));
}

void eraseArrowUp()
{
  lcd.setCursor(15,0);
  lcd.write(" ");
}

void eraseArrowDown()
{
  lcd.setCursor(15,1);
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
static float apply_PID(float Vin){
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
  float integral_part = (Ki * int_err * (float)(1 / (float)(FREQUENCY)));
  float differential_part = (Kd * diff_err / (float)(1 / (float)(FREQUENCY)));

  output = proportional_part + integral_part + differential_part;
  if (output >= Vlim_Up || output <= -Vlim_Up)
  {
	Serial.println("Windup");
	int_err = temp_int; //Remet l'ancienne int_erre
    // Set value to absolute max
    integral_part = (Ki * int_err * (float)(1 / (float)(FREQUENCY)));
    output = proportional_part + integral_part + differential_part;
  }
 
  output = last_output - output; //Corrige la dernière tension appliquée.
  
  //Le if, else if qui suit offre une protection pour de pas avoir des valeurs trop haute ou trop base. Important de la laisser car il arrive que la première erreur donne une erreur complètement erroné et que le PID diverge.  
  if (output < OUTPUT_MIN) // Vérifie que output respecte ça valeur min
  {
	output = OUTPUT_MIN;
  }
  else if (output > OUTPUT_MAX) // Vérifie que output respecte ça valeur max
  {
	output = OUTPUT_MAX;
  }  
  sendToDAC(output);
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
  currentStep = tabEtalons[step_index];
  benchmarkBuffer.push(val);
  if (benchmarkBuffer.isFull())
  {
    currentState = PROCESS_STATE;
    statePtr = processState;
  }
  step_index++;
}

void calculateAvgMass(void)
{
  uint32_t sum = 0;
  while (!avgBuffer.isEmpty())
  {
    sum += avgBuffer.pop();
  }
  
  currMass = (float)(penteMasseCourant*(sum/AVG_SAMPLES));
  if (currentUnit == "oz")
  {
    currMass *= GRAMS_TO_OZ;
  }
}

void calculateNumCoins(void)
{
  currNumCoins = floor(currMass/massTypeGram[type_index]);
}

void sendToDAC(float outputDAC){
	/* Fonction qui permet de prendre un float et de communiquer avec le DAC with I2C */
	
	last_output = outputDAC;
	
	int outputDACint = round(outputDAC * (DAC_RES/DAC_VREF)); // Converti l'argument transmit en int en fonction de la résolution et de la valeur de référence du DAC
	
	Wire.beginTransmission(MCP4725_ADDR);
	Wire.write(64);                     // cmd to update the DAC
	Wire.write(outputDACint >> 4);        // the 8 most significant bits...
	Wire.write((outputDACint & 15) << 4); // the 4 least significant bits...
	Wire.endTransmission();
}

/////////////////////
// STATE FUNCTIONS //
/////////////////////

static void configState(void)
{
  printMenuConfig();
}

static void processState(void)
{
  float sommePentes = 0;
  lcd.clear();
  
  /* Calcul de la pente pour le calcul de la masse */
  for(int i = 0; i < NUM_ETALONS; i++)                         // Moyenne des coefficients
  {
    sommePentes = sommePentes + tabEtalons[i]/((benchmarkBuffer.pop()*VREF)/ADC_RES);
    /* Petite animation durant l'état de processus */
    lcd.setCursor(i,i%2);
    lcd.print(byte(2));
  }
  penteMasseCourant = sommePentes/NUM_ETALONS;

  /* Affichage particulier suivant le calcul de la pente... */

  /* Retour à l'état de configuration */
  currentState = CONFIG_STATE;
  statePtr = configState;
}

static void benchmarkState(void)
{
  float Vin = 0;
  printBenchmarkSteps();
  if (!scaleBuffer.isEmpty())
  {
    Vin = scaleBuffer.pop();
    apply_PID(analogRead(pinADC)*(5.0/1024.0)); //Vin, converti en volt
  }
}

static void scaleState(void)
{
  float Vin = 0;
  
  if (!scaleBuffer.isEmpty())
  {
    Vin = scaleBuffer.pop();
    apply_PID(analogRead(pinADC)*(5.0/1024.0)); //Vin, converti en volt
    if (selectedMode == MODE_MOYENNAGE)
    {
      if (avgBuffer.isFull())
      {
        calculateAvgMass();
      }
      else
      {
        avgBuffer.push(analogRead(pinCURRENT)); 
      }
    }
  }
  if (abs(currMass - prevMass) < MASS_ERROR)
  {
    printStabilitySymbol();
  }
  printScaleFirstLine();
  printScaleSecondLine();
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

  val = analogRead(pinADC);
  scaleBuffer.push((float)(val)*VREF/ADC_RES);
}

void ISR_menuSelect(void) // Ou select
{
  Serial.println("menu");
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
  else if(currentState == BENCHMARK_STATE)
  {
    currentState = CONFIG_STATE;
    statePtr = configState;
  }
}

void ISR_buttonA(void) // Ou up
{
  if(currentState == SCALE_STATE)
  {
    unit_index++;
    currentUnit = unitArray[unit_index];
    if (unit_index == NUM_UNITS-1)
    {
      unit_index = 0;
    }
  }
  else if(currentState == CONFIG_STATE)
  {
    if (selectedMode == MODE_MOYENNAGE)
    {
      Serial.println("here");
      selectedMode = MODE_NORMAL;
    }
    else if (selectedMode == MODE_ETALONNAGE)
    {
      selectedMode = MODE_MOYENNAGE;
    }
  }
}

void ISR_buttonB(void) // Ou down

{
  if(currentState == SCALE_STATE)
  {
    type_index++;
    currentType = typeArray[type_index];
    if (type_index == NUM_TYPES-1)
    {
      type_index = 0;
    }
  }
  else if(currentState == CONFIG_STATE)
  {
    if(selectedMode == MODE_NORMAL)
    {
      selectedMode = MODE_MOYENNAGE;
    }
    else if (selectedMode == MODE_MOYENNAGE)
    {
      selectedMode = MODE_ETALONNAGE;
    }
  }
}

void ISR_tare(void)
{
  if(currentState == SCALE_STATE)
  {
    massTare = currMass;
  }
  else if (currentState == BENCHMARK_STATE)
  {
    uint16_t val = analogRead(pinCURRENT);
    incrementBenchmark(val);
  }
}
