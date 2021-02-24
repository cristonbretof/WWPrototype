#include <stdint.h>

#include <CircularBuffer.h>
#include <LiquidCrystal.h>

#define LED_PIN      13       // GPIO pin (on-board LED)

#define V0           2.5      // Voltage refering to home position of platform
#define Vlim_Up      4        // Maximum allowed voltage to provide to amplifier

#define ADC_PIN      A0       // Analog pin to read position input
#define ADC_RES      1023     // Maximum resolution of analog to digital converter
#define ADC_VREF     5        // ADC reference voltage

#define PWM_PIN      3        // PWM pin to control amplifier (basic PWM)

#define PRESCALER    256      // Prescaler digital value
#define BOARD_Freq   16000000 // Arduino Mega 2560 board frequency in Hz
#define MAX_FREQ     129      // Absolute maximum frequency
#define FREQUENCY    100

#define Kp           1        // Proportional gain
#define Ki           2        // Integral gain
#define Kd           1        // Differential gain

#define BUF_LEN 20

static float int_err;
static float prev_err;

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
  analogWrite(PWM_PIN, dig_output);
  return output;
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
  float Vin = 0;
  float output;
  if (!buffer.isEmpty())
  {
    Vin = buffer.pop();
    output = apply_PID(Vin);
    lcd.setCursor(0,0);
    lcd.print("PWM = "); lcd.print(output,4);
    lcd.setCursor(0,1);
    lcd.print("BUFFER LINE");
    Serial.println(buffer.size());
  }
}
