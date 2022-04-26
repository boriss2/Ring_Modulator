/*
  AD9833 Waveform Module vwlowen.co.uk
  adopted by Boris Shteynberg bshteynb@gmail.com
*/
// This program requires the UTFT library.
#include <Arduino.h>
#include <SPI.h>
#include <Rotary.h> // Rotary encoder: https://github.com/brianlow/Rotary
#include <UTFT.h>
#include <Bounce2.h> //https://github.com/thomasfredericks/Bounce2/

// Declare which fonts we will be using
extern uint8_t Arial_round_16x24[];
extern uint8_t BigFont[];
//#define tft_height 320

UTFT tft(CTE32HR, 38, 39, 40, 41);

#define BLACK 0x000 // Define the display colors we'll be using
#define BLUE 0x001F
#define GREEN 0x07E0
#define YELLOW 0xFFE0
#define GREY 0x632C
#define APP_NAME " AD9833/AD9833  Ring Generator "
const int SINE = 0x2000;     // Define AD9833's waveform register value.
const int SQUARE = 0x2028;   // When we update the frequency, we need to
const int TRIANGLE = 0x2002; // define the waveform when we end writing.

int waveType = SINE;
#define BUTTON_PIN 9
/*The circuit:
  encoder pin A to Arduino pin 2
  encoder pin B to Arduino pin 3
  encoder ground pin to ground (GND)
*/
#define freqUpPin 2 // Define rotary encoder pins.
#define freqDownPin 3
#define FSYNC2 5
#define FSYNC1 4
#define SS 53 // SS                  // Standard SPI pins for the AD9833 waveform generator.
#define pulseHigh(pin)       \
  {                          \
    digitalWrite(pin, HIGH); \
    digitalWrite(pin, LOW);  \
  }

/*
  51 or ICSP-4  MOSI
  50 or ICSP-1  MISO
  52 or ICSP-3  SCK
  53
*/

const float refFreq = 25000000.0; // On-board crystal reference frequency on AD9833
double trimFreq = 124999500;
volatile unsigned long freq = 535000; // Set initial frequency.
unsigned long freqOld = 535010;
unsigned long incr = 10000;

volatile unsigned long tonefreq = 1000;
unsigned long tonefreqold = 100;
unsigned long minfreq = 535000;
unsigned long maxfreq = 1705000;
unsigned long stepfreq = 10000;
unsigned long mode = 0;
// 0 - stop  1 -change tonefreq 2- change carrier freq ,3 Run
unsigned long incrtone = 10;
unsigned long maxtonefreq = 15000;
unsigned long mintonefreq = 50;
unsigned long oldValue = HIGH;

Rotary rotary = Rotary(freqUpPin, freqDownPin); // Rotary encoder  connects to interrupt pins
Bounce debouncer = Bounce();                    // Button

void setup()
{
  Serial.begin(57600);
  Serial.println(APP_NAME);
  Serial.println("Start Up");
  pinMode(FSYNC1, OUTPUT);
  digitalWrite(FSYNC1, HIGH);
  pinMode(FSYNC2, OUTPUT);
  digitalWrite(FSYNC2, HIGH);

  pinMode(freqUpPin, INPUT_PULLUP);   // Set pins for rotary encoders as INPUTS and enable
  pinMode(freqDownPin, INPUT_PULLUP); // internal pullup resistors.
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // After setting up the button, setup the Bounce instance :
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5); // interval in ms

  rotary.begin(1);
  attachInterrupt(digitalPinToInterrupt(freqUpPin), rotaryISR, CHANGE); // Also LOW, RISING, FALLING
  attachInterrupt(digitalPinToInterrupt(freqDownPin), rotaryISR, CHANGE);
  SPI.begin();
  tft.InitLCD();
  tft.setFont(Arial_round_16x24);
  tft.clrScr();
  tft.setColor(YELLOW);
  tft.print(APP_NAME, CENTER, 20);
  AD9833reset(); // Reset AD9833 module after power-up.
  delay(50);
  // Set the frequency and Sine Wave output
  AD9833setFrequency(tonefreq, SINE, FSYNC1);
  AD9833setFrequency(freq, SINE, FSYNC2);
  DisplayCarrier(freq, tonefreq);
  Serial.println("Start Up completed");
}

void DisplayCarrier(unsigned long value, unsigned long tone)
{

  tft.setColor(YELLOW);
  int pos = 110;
  tft.print("Carrier Freq:" + String(value / 1000) + " KHz", CENTER, pos);
  pos += 30;
  tft.print("Test Tone Freq:" + String(tone) + " Hz", CENTER, pos);
  pos += 80;
  tft.print("                                            ", CENTER, pos);
  switch (mode)
  {
  case 1:
    tft.print("Mode: Change Tone freq", CENTER, pos);
    break;
  case 2:
    tft.print("Mode: Change carrier freq", CENTER, pos);
    break;
  case 3:
    tft.print("Mode: Running", CENTER, pos);
    break;
  default:
    tft.print("Mode: Stop", CENTER, pos);
  }
  pos += 40;
  tft.setColor(GREEN);
  tft.print("Version 1.10", CENTER, pos);
}

void loop()
{
  //  Serial.println("Loop Start");

  bool refresh = false;
  // Check if push button on  rotary encoder is pushed and set mode accordingly.
  debouncer.update();
  // Get the updated value :
  int value = debouncer.read();

  // if (digitalRead(BUTTON_PIN) == LOW) {
  if (value == LOW && oldValue == HIGH)
  {
    refresh = true;
    Serial.println("Button Pressed");
    mode += 1;
    if (mode > 3)
    {
      mode = 0;
    }
    //  Serial.print("Mode:");
    // Serial.println(mode);
  }

  oldValue = value;

  if (tonefreq != tonefreqold)
  {                                            // If frequency has changed, interrupt rotary encoder
    AD9833setFrequency(tonefreq, waveType, 4); // must have been turned so set flag to update display and update AD9833 .
    refresh = true;
    tonefreqold = tonefreq; // Remember new frequency to avoid unwanted updates
  }

  if (freqOld != freq)
  {
    AD9833setFrequency(tonefreq, waveType, FSYNC2);
    refresh = true;
    freqOld = freq;
  }

  if (refresh)
  {
    DisplayCarrier(freq, tonefreq);
  }
}
// Serial.println("Loop End");

// AD9833 documentation advises a 'Reset' on first applying power.
void AD9833reset()
{
  WriteRegister(0x100, FSYNC1); // Write '1' to AD9833 Control register bit D8.
  delay(10);
  WriteRegister(0x100, FSYNC2); // Write '1' to AD9833 Control register bit D8.
  delay(10);
}

// Set the frequency and waveform registers in the AD9833.
void AD9833setFrequency(long frequency, int Waveform, uint8_t Device)
{
  long FreqWord = (frequency * pow(2, 28)) / refFreq;
  int MSB = (int)((FreqWord & 0xFFFC000) >> 14); // Only lower 14 bits are used for data
  int LSB = (int)(FreqWord & 0x3FFF);
  // Set control bits 15 ande 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000;
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE2));
  WriteRegister(0x2100, Device);
  WriteRegister(LSB, Device);      // Write lower 16 bits to AD9833 registers
  WriteRegister(MSB, Device);      // Write upper 16 bits to AD9833 registers.
  WriteRegister(0xC000, Device);   // Phase register
  WriteRegister(Waveform, Device); // Exit & Reset to SINE, SQUARE or TRIANGLE
  SPI.endTransaction();
}

void WriteRegister(int dat, uint8_t Device)
{
  digitalWrite(Device, LOW);   // Set FSYNC low before writing to AD9833 registers
  delayMicroseconds(10);       // Give AD9833 time to get ready to receive data.
  SPI.transfer(highByte(dat)); // Each AD9833 register is 32 bits wide and each 16
  SPI.transfer(lowByte(dat));  // bits has to be transferred as 2 x 8-bit bytes.
  digitalWrite(Device, HIGH);  // Write done. Set FSYNC high
}

// Interrupt service routine for the 'frequency' rotary encoder.
void rotaryISR()
{
  if (mode == 2 || mode == 1)
  {
    unsigned char result = rotary.process();
    if (result == DIR_NONE)
    {
      // do nothing
    }
    else if (result == DIR_CW)
    { // Clockwise rotation so add increment to frequency
      if (mode == 2)
      {
        if ((freq + incr) < maxfreq)
        {
          freq += incr;
        }
      }
      else // mode 1
      {
        if ((tonefreq + incrtone) < maxtonefreq)
        {
          tonefreq += incrtone;
        }
      }
      // Serial.println("ClockWise");
    }
    else if (result == DIR_CCW)
    {
      if (mode == 2)
      { // 2- change carrier freq
        if ((freq - incr) > minfreq)
        {
          freq -= incr;
        }
      }
      else
      {
        if ((tonefreq - incrtone) > mintonefreq)
        {
          tonefreq -= incrtone;
        }
      }
      //   Serial.println("CounterClockWise");
    }
  }
}
