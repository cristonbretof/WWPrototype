#include <stdint.h>

#include <CircularBuffer.h>
#include <LiquidCrystal.h>

#include <Wire.h>

///////////////////////////////
// DÉFINITIONS PRÉPROCESSEUR //
///////////////////////////////

#define V0            2.5                    // Tension centrale de référence pour la lame
#define Vlim_Up       2                      // Limite maximale permise pour la monté de tension (provenant de l'erreur)

#define pinADC        A15                    // Pin utilisé pour l'ADC servant à l'échantillonnage de tension
#define ADC_RES       1023                   // Résolution (en bit) de l'ADC utilisé
#define VREF          5                      // Tension de référence de l'ADC

#define pinMASSVOLTAGE    A14                    // Pin utilisé pour la lecture du courant
//#define PWM_PIN 12                         // PWM pin to control amplifier (basic PWM) // On peut enlever cette ligne là

#define PRESCALER     256                    // Prescaler digital value
#define BOARD_Freq    16000000               // Arduino Mega 2560 board frequency in Hz

#define MAX_FREQ      145                    // Absolute maximum frequency with minimal LCD use
#define FREQUENCY     145                    // Currently used frequency

#define ENABLE_TIMER  TIMSK1 | (1 << OCIE1A) // Macro pouvant changer le flag du timer d'échantillonnage à ON
#define DISABLE_TIMER TIMSK1 | (0 << OCIE1A) // Macro pouvant changer le flag du timer d'échantillonnage à OFF

#define MASS_ERROR    0.3                    // Erreur permise sur la masse lors de son calcul
#define GRAMS_TO_OZ   0.035274               // Pente de conversion de gramme à once

/*#define Ku            0.008     // Ultimate gain
#define tu            0.136     //Periode oscillations
#define Ki            Kp/(tu/2)        // Integral gain
#define Kd            Kp*(tu/3)         // Differential gain
#define Kp            Ku/3 */

/* Gains pour le PID */
#define Kp 0.008   // Gain proportionnel
#define Ki 0.06    // Gain integral
#define Kd 0.00003 // Gain différentiel

/* Pins utilisées pour le LCD */
#define pinRS 24
#define pinE  26
#define pinD4 28
#define pinD5 30
#define pinD6 32
#define pinD7 34

/* Nombre de types, d'unités et d'étalons dans les tableaux */
#define NUM_TYPES 7
#define NUM_UNITS 2
#define NUM_ETALONS 11

/* Configurations du DAC externe */
#define DAC_RES    4096 // Maximum resolution of digital to analog converter
#define DAC_VREF   5    // DAC reference voltage

/* Valeurs maximales et minimales de sortie */
#define OUTPUT_MAX 2    // Offre une protection sur la valeur appliquer au DAC
#define OUTPUT_MIN 0    // Offre une protection sur la valeur appliquer au DAC

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0.
#define MCP4725_ADDR 0x60
//For devices with A0 pulled HIGH, use 0x61

/* Nombre de boutons total sur le prototype */
#define NUM_BUTTONS 4

/* Définition des boutons utilisés pour le menu */
#define pinBUTTON_A    18
#define pinBUTTON_B    2
#define pinBUTTON_MENU 19
#define pinBUTTON_TARE 3

/* Paramètres d'échantillonnage (mode normal et moyennage) */
#define BUF_LEN 20    // Taille du buffer d'échantillons
#define AVG_SAMPLES 3 // Nombre d'écahntillons pour faire la moyenne

/* Indice de la pièce utilisée pour aider à calculer l'ordonnée dela courbe de
   conversion*/
#define INTERCEPT_CALCULATION_INDEX 5


///////////////////////////////
//   STRUCTURES DE DONNÉES   //
///////////////////////////////

/* Énumérateur pour chacun des états du système */
typedef enum
{
  CONFIG_STATE = 0,
  SCALE_STATE = 1,
  BENCHMARK_STATE = 2,
  PROCESS_STATE = 3
} state_t;

/* Énumérateur pour les modes d'opération offerts */
typedef enum
{
  MODE_NORMAL = 0,
  MODE_MOYENNAGE = 1,
  MODE_ETALONNAGE = 2
} mode_t;

/* Tableaux 1D représentant respectivement les types, les unités et les étalons supportés */
String typeArray[NUM_TYPES] = {"1c ", "5c ", "10c", "25c", "1$ ", "2$ ", "0  "};
String unitArray[NUM_UNITS] = {"oz   ", "g   "};
uint8_t tabEtalons[NUM_ETALONS] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
uint8_t tabCourantEtalons[NUM_ETALONS] = {0.335, 0.49, 0.645, 0.81, 0.95, 1.1, 1.25, 1.4, 1.545, 1.695, 1.86};

/* Masses des pièces de monnaie canadienne */
float massTypeGram[NUM_TYPES] = {2.35, 3.95, 1.75, 4.4, 6.9, 6.27, 6.92};

///////////////////////////////
//    VARIABLES "GLOBALES"   //
///////////////////////////////

/* Indices initiaux pour les unités et les types */
uint8_t unit_index = 0;
uint8_t type_index = 0;

/* Pente et ordonnée de la calibration masse/courant */
float penteMasseCourant = 1;
float ordMasseCourant = 0;

/* Variables référant à la masse */
float massTare = 0;
float currMass = 0;
float prevMass = 0;
uint8_t currNumCoins = 0;

/* Unité et type actifs sur l'affichage */
String currentUnit = unitArray[1];
String currentType = typeArray[0];

/* État actuel de l'étalonnage */
uint16_t currentStep = 0;
uint8_t step_index = 0;

/* Pointeur de fonction pour la machine à état */
void (*statePtr)(void);

/* Initialisation de l'état et du mode initiaux */
static state_t currentState = CONFIG_STATE;
static mode_t selectedMode = MODE_NORMAL;

/* Variables représentant les erreurs à travaer le temps (intégral et précédente) */
static float int_err = 0;
static float prev_err = 0;

/* Valeur de sortie précédente */
float last_output = 0;

/* Déclaration des fonctions pour la machine à état */
static void processState(void);
static void configState(void);
static void scaleState(void);
static float apply_PID(float Vin);

/* Déclaration des fonctions init et deinit du timer d'échantillonnage */
void setupTimer();
void deinitTimer();

/* Définitions des caractères spéciaux */
byte up[8] = {
    B00000,
    B00100,
    B01110,
    B11111,
    B01110,
    B01110,
    B00000,
    B00000};
byte down[8] = {
    B00000,
    B00000,
    B01110,
    B01110,
    B11111,
    B01110,
    B00100,
    B00000};
byte full[8] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111};
byte stable[8] = {
    B00000,
    B00000,
    B01010,
    B01010,
    B00000,
    B10001,
    B01110,
    B00000};

CircularBuffer<uint16_t, AVG_SAMPLES> avgBuffer;
CircularBuffer<float, BUF_LEN> scaleBuffer;
CircularBuffer<uint16_t, NUM_ETALONS> benchmarkBuffer;

/* Initialize LCD board */
LiquidCrystal lcd = LiquidCrystal(pinRS, pinE, pinD4, pinD5, pinD6, pinD7);

///////////////////////
// FONCTIONS ARDUINO //
///////////////////////

void setup()
{
  Serial.begin(9600);

  /* Création des caractères spéciaux */
  lcd.createChar(0, up);
  lcd.createChar(1, down);
  lcd.createChar(2, full);
  lcd.createChar(3, stable);

  /* Combinaison bouton/interruption  */
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_MENU), ISR_menuSelect, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_B), ISR_buttonB, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_A), ISR_buttonA, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBUTTON_TARE), ISR_tare, FALLING);

  /* Start the LCD display */
  lcd.begin(16, 2);

  /* Initier la phase de configuration */
  currentState = CONFIG_STATE;
  statePtr = configState;

  /* Amorcer le DAC */
  Wire.begin();
  sendToDAC(0.0); //Reset la valeur du DAC
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
  lcd.setCursor(0, 0);
  lcd.print("MASSE: ");
  lcd.print((float)(currMass - massTare), 2);
  lcd.print(currentUnit);
  Serial.println((float)(currMass - massTare));
}

void printScaleSecondLine(void)
{
  lcd.setCursor(0, 1);
  lcd.print(calculateNumCoins());
  lcd.print(" x ");
  lcd.print(currentType);
}

void printMenuConfig(void)
{
  switch (selectedMode)
  {
  case MODE_NORMAL:
    lcd.clear();

    eraseArrowUp();
    printArrowDown();

    lcd.setCursor(0, 0);
    lcd.print("MODE NORMAL");
    break;
  case MODE_MOYENNAGE:
    lcd.clear();

    printArrowUp();
    printArrowDown();

    lcd.setCursor(0, 0);
    lcd.print("MODE MOYENNAGE");
    break;
  case MODE_ETALONNAGE:
    lcd.clear();

    printArrowUp();
    eraseArrowDown();

    lcd.setCursor(0, 0);
    lcd.print("ETALONNAGE");
    break;
  }
}

void printStabilitySymbol(void)
{
  lcd.setCursor(15, 1);
  lcd.write(byte(3));
}

void eraseStabilitySymbol(void)
{
  lcd.setCursor(15, 1);
  lcd.write(" ");
}

void printBenchmarkSteps(void)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (currentStep > 0 && !benchmarkBuffer.isFull())
  {
    lcd.print("Posez une masse");
    lcd.setCursor(0, 1);
    lcd.print("de : ");
    lcd.print(currentStep);
    lcd.print("g");
  }
  else if (currentStep == 0)
  {
    lcd.print("Retirez le poids");
    lcd.setCursor(0, 1);
    lcd.print("de la balance");
  }
  else
  {
    lcd.print("Retirez le poids");
    lcd.setCursor(0, 1);
    lcd.print("de la balance");
  }
}

void printArrowUp()
{
  lcd.setCursor(15, 0);
  lcd.write(byte(0));
}

void printArrowDown()
{
  lcd.setCursor(15, 1);
  lcd.write(byte(1));
}

void eraseArrowUp()
{
  lcd.setCursor(15, 0);
  lcd.write(" ");
}

void eraseArrowDown()
{
  lcd.setCursor(15, 1);
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
  delay(10);
  float output;

  // Calculate positionning error
  float err = V0 - Vin;
  //Serial.println(err);

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

  if (abs(output) < (5 / ADC_RES))
  {
    Serial.println("haha");
    return 0.0;
  }
  //Le if, else if qui suit offre une protection pour de pas avoir des valeurs trop haute ou trop base. Important de la laisser car il arrive que la première erreur donne une erreur complètement erroné et que le PID diverge.
  if (output < OUTPUT_MIN) // Vérifie que output respecte ça valeur min
  {
    Serial.println("Output min");

    output = OUTPUT_MIN;
  }
  else if (output > OUTPUT_MAX) // Vérifie que output respecte ça valeur max
  {
    Serial.println("Output max");

    output = OUTPUT_MAX;
  }
  sendToDAC(output);
  return output;
}

void deinitTimer()
{
  /* Disable interrupts */
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  DISABLE_TIMER; // Enable timer for compare interrupt

  /* Reenable interrupts */
  interrupts();
}

void setupTimer()
{
  /* Disable interrupts */
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  /* Set compare register for desired frequency */
  OCR1A = (int)(BOARD_Freq / PRESCALER / FREQUENCY - 1);

  TCCR1B |= (1 << WGM12);  // Turn on CTC mode
  TCCR1B |= (1 << CS12);   // Set prescaler to 8
  TIMSK1 |= (1 << OCIE1A); // Enable timer for compare interrupt

  /* Reenable interrupts */
  interrupts();
}

void incrementBenchmark(uint16_t val)
{
  if (!benchmarkBuffer.isFull())
  {
    currentStep = tabEtalons[step_index];
    benchmarkBuffer.push(val);
    step_index++;
  }
}

void calculateAvgMass(void)
{
  uint32_t sum = 0;
  while (!avgBuffer.isEmpty())
  {
    sum += avgBuffer.pop();
  }

  currMass = (float)(penteMasseCourant * (sum / AVG_SAMPLES) + ordMasseCourant);
  if (currentUnit == "oz")
  {
    currMass *= GRAMS_TO_OZ;
  }
}

uint8_t calculateNumCoins(void)
{
  return (uint8_t)floor(currMass / massTypeGram[type_index]);
}

void sendToDAC(float outputDAC)
{
  /* Fonction qui permet de prendre un float et de communiquer avec le DAC with I2C */

  last_output = outputDAC;

  int outputDACint = round(outputDAC * (DAC_RES / DAC_VREF)); // Converti l'argument transmit en int en fonction de la résolution et de la valeur de référence du DAC

  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                       // cmd to update the DAC
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
  delay(100);
}

static void processState(void)
{
  uint16_t v_in = 0;
  float sommePentes = 0;
  float xCurrentForIntercept = 0;

  /* Désactivation des interruptions */
  noInterrupts();

  /* Remise zéro de l'indice d'étape */
  step_index = 0;

  lcd.clear();

  /* Calcul de la pente pour le calcul de la masse */
  for (int i = 0; i < NUM_ETALONS; i++) // Moyenne des coefficients
  {
    v_in = benchmarkBuffer.pop();
    sommePentes += (float)(tabEtalons[i] / ((v_in * VREF) / ADC_RES));
    tabCourantEtalons[i] = v_in;

    /* Petite animation durant l'état de processus */
    lcd.setCursor(i, i % 2);
    lcd.print(byte(2));
    delay(500);
  }
  penteMasseCourant = sommePentes / NUM_ETALONS;
  ordMasseCourant = tabCourantEtalons[0];

  currMass = (float)(penteMasseCourant * analogRead(pinMASSVOLTAGE) + ordMasseCourant);

  /* Réétablir la tare */
  massTare = currMass;

  /* Réactivation des interruptions */
  interrupts();

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
    /* Debug de la fréquence en se référant à la taille du buffer (nb d'élément dans le buffer à cet instant) */
    //Serial.println(scaleBuffer.size()); // On veut la valeur de 1, pas de 20, sur le moniteur

    Vin = scaleBuffer.pop();
    apply_PID(Vin); //Vin, converti en volt
  }
}

static void scaleState(void)
{
  float Vin = 0;

  if (!scaleBuffer.isEmpty())
  {
    /* Debug de la fréquence en se référant à la taille du buffer (nb d'élément dans le buffer à cet instant) */
    // Serial.println(scaleBuffer.size()); // On veut la valeur de 1, pas de 20, sur le moniteur

    Vin = scaleBuffer.pop();
    apply_PID(Vin); //Vin, converti en volt
    if (selectedMode == MODE_MOYENNAGE)
    {
      if (avgBuffer.isFull())
      {
        calculateAvgMass();
      }
      else
      {
        avgBuffer.push(analogRead(pinMASSVOLTAGE));
      }
    }
  }
  
  currMass = (float)(penteMasseCourant * analogRead(pinMASSVOLTAGE) + ordMasseCourant);
  if (abs(currMass - prevMass) < MASS_ERROR)
  {
    printStabilitySymbol();
  }
  else
  {
    eraseStabilitySymbol();
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
  uint16_t val = 0;

  val = analogRead(pinADC);
  scaleBuffer.push((float)((val)*VREF / ADC_RES));
}

void ISR_menuSelect(void) // Ou select
{
  if (currentState == SCALE_STATE)
  {
    deinitTimer();
    currentState = CONFIG_STATE;
    statePtr = configState;
  }
  else if (currentState == CONFIG_STATE)
  {
    setupTimer();
    if (selectedMode == MODE_ETALONNAGE)
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
  else if (currentState == BENCHMARK_STATE)
  {
    deinitTimer();
    currentState = CONFIG_STATE;
    statePtr = configState;
  }
}

void ISR_buttonA(void) // Ou up
{
  if (currentState == SCALE_STATE)
  {
    unit_index++;
    currentUnit = unitArray[unit_index];
    if (unit_index == NUM_UNITS - 1)
    {
      unit_index = 0;
    }
  }
  else if (currentState == CONFIG_STATE)
  {
    if (selectedMode == MODE_MOYENNAGE)
    {
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
  if (currentState == SCALE_STATE)
  {
    type_index++;
    currentType = typeArray[type_index];
    if (type_index == NUM_TYPES - 1)
    {
      type_index = 0;
    }
  }
  else if (currentState == CONFIG_STATE)
  {
    if (selectedMode == MODE_NORMAL)
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
  if (currentState == SCALE_STATE)
  {
    massTare = currMass;
  }
  else if (currentState == BENCHMARK_STATE)
  {
    if (benchmarkBuffer.isFull())
    {
      currentState = PROCESS_STATE;
      statePtr = processState;
    }
    else
    {
      uint16_t val = analogRead(pinMASSVOLTAGE);
      incrementBenchmark(val);
    }
  }
}
