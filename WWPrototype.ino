#include <stdint.h>

#include <CircularBuffer.h>
#include <LiquidCrystal.h>

#include <Wire.h> //Library for DAC

#define LED_PIN      13       // GPIO pin (on-board LED)

#define V0           2.5      // Voltage refering to home position of platform
#define Vlim_Up      2        // Maximum allowed voltage to provide to amplifier

#define ADC_PIN      A0       // Analog pin to read position input
#define ADC_RES      1023     // Maximum resolution of analog to digital converter
#define ADC_VREF     5        // ADC reference voltage

#define PWM_PIN      12        // PWM pin to control amplifier (basic PWM)

#define PRESCALER    256      // Prescaler digital value
#define BOARD_Freq   16000000 // Arduino Mega 2560 board frequency in Hz
#define MAX_FREQ     129      // Absolute maximum frequency
#define FREQUENCY    100

#define Kp           1        // Proportional gain
#define Ki           2        // Integral gain
#define Kd           1        // Differential gain

#define BUF_LEN 20

#define DAC_RES      4096     // Maximum resolution of digital to analog converter
#define DAC_VREF     5        // DAC reference voltage
#define OUTPUT_MAX   5        // Value max of variable output in apply_PID(float Vin)
#define OUTPUT_MIN   0        // Value max of variable output in apply_PID(float Vin)

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0. 
#define MCP4725_ADDR 0x60   
//For devices with A0 pulled HIGH, use 0x61

static float int_err = 0;
static float prev_err = 0;

int lookup = 0;// Pour test CAD varaible for navigating through the tables
float sintab2[100] = 
{
  0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4,
  0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8,
  0.85, 0.9, 0.95,
  1, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.35, 1.4,
  1.45, 1.5, 1.55, 1.6, 1.65, 1.7, 1.75, 1.8,
  1.85, 1.9, 1.95,
  2, 2.05, 2.1, 2.15, 2.2, 2.25, 2.3, 2.35, 2.4,
  2.45, 2.5, 2.55, 2.6, 2.65, 2.7, 2.75, 2.8,
  2.85, 2.9, 2.95,
  3, 3.05, 3.1, 3.15, 3.2, 3.25, 3.3, 3.35, 3.4,
  3.45, 3.5, 3.55, 3.6, 3.65, 3.7, 3.75, 3.8,
  3.85, 3.9, 3.95,
  4, 4.05, 4.1, 4.15, 4.2, 4.25, 4.3, 4.35, 4.4,
  4.45, 4.5, 4.55, 4.6, 4.65, 4.7, 4.75, 4.8,
  4.85, 4.9, 4.95
};// Pour test CAD

CircularBuffer<float, BUF_LEN> buffer;

/* Initialize LCD board */
LiquidCrystal lcd = LiquidCrystal(12, 11, 5, 4, 3, 2);

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

// ARDUINO CODE BEGIN
void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);

  /* Start the LCD display */
  Serial.println("LCD init");
  lcd.begin(16, 2);

  setupTimer();

  /* Start CAD */
  Wire.begin() ;
}

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
  float integral_part = (Ki * int_err * (float)(1 / (float)(FREQUENCY)));
  float differential_part = (Kd * diff_err / (float)(1 / (float)(FREQUENCY)));

  output = proportional_part + integral_part + differential_part;
  if (output >= Vlim_Up)
  {
    // Set value to absolute max
    integral_part = (Ki * temp_int * (float)(1 / (float)(FREQUENCY)));
    output = proportional_part + integral_part + differential_part;
  }
  // Convert value to 16 bit integer and send to PWM
  //uint8_t dig_output = (uint8_t)(output*256/5);
  //analogWrite(PWM_PIN, dig_output);
  
  /*
  Serial.println("err");
  Serial.println(err);
  Serial.println("int_err");
  Serial.println(int_err);
  Serial.println("diff_err");
  Serial.println(diff_err);
  Serial.println("Kd");
  Serial.println(Kd);
  Serial.println("FREQUENCY");
  Serial.println(FREQUENCY);
  Serial.println("1 / FREQUENCY");
  Serial.println(1 / FREQUENCY);
  Serial.println("proportional_part");
  Serial.println(proportional_part);
  Serial.println("integral_part");
  Serial.println(integral_part);
  Serial.println("differential_part");
  Serial.println(differential_part);
  Serial.println("output");
  Serial.println(output);
  */
  
  sendToDAC(output);
  
  return output;
}

void sendToDAC(float outputDAC)
{
	/* Fonction qui permet de prendre un float et de communiquer avec le DAC with I2C */
	
	if (outputDAC < OUTPUT_MIN) // Vérifie que output respecte ça valeur min
	{
		outputDAC = OUTPUT_MIN;
	}
	else if (outputDAC > OUTPUT_MAX) // Vérifie que output respecte ça valeur max
	{
		outputDAC = OUTPUT_MAX;
	}
	
	int outputDACint = round(outputDAC * (DAC_RES/DAC_VREF)); // Converti l'argument transmit en int en fonction de la résolution et de la valeur de référence du DAC
	
	Wire.beginTransmission(MCP4725_ADDR);
	Wire.write(64);                     // cmd to update the DAC
	Wire.write(outputDACint >> 4);        // the 8 most significant bits...
	Wire.write((outputDACint & 15) << 4); // the 4 least significant bits...
	Wire.endTransmission();
}

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
  val = analogRead(ADC_PIN);
  buffer.push((float)(val)*ADC_VREF/ADC_RES);
}

void loop() {
  float Vin = analogRead(A0)*(5.0/1024.0); //Vin, converti en volt
  float output;
  if (!buffer.isEmpty())
  {
    Vin = buffer.pop();
    output = apply_PID(Vin);
    lcd.setCursor(0,0);
    lcd.print("PWM = "); lcd.print(output,4);
    lcd.setCursor(0,1);
    lcd.print("BUFFER LINE");
    //Serial.println(buffer.size());
  }

/*  
  //Test DAC
  sendToDAC(sintab2[lookup]);
  if (lookup < 99)
  {
	  lookup = lookup + 1;
  }
  else
  {
	  lookup = 0;
  }
  //lookup = (lookup + 1) & 99;
*/}
