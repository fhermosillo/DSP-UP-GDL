/*
 * @brief ESP32 pmod-i2s2 example full-duplex (RX/TX)
 */

#include "I2SDriver.h"
#include "RingBuffer.h"
#include "WaveGenerator.h"

/* Namespace -------------------------------------*/
namespace int24
{
  const float INT24_MAX = 1.0F;		// Set the correct value
  const float INT24_SCALE = 1.0F;	// Set the correct value
  inline float toFloat(int32_t x) {
	// Integer to Float conversion (Use INT24_SCALE constant)
    return (float)(/* Write your conversion here */);
  }
  
  inline int32_t toInteger(float x) {
	// Saturate sample to the range [-1.0,1.0]
    
	// Float to Integer conversion (Use INT24_MAX constant)
    return (int32_t)(/* Write your conversion here */);
  }
}

/* Constants -------------------------------------*/
// I2S Pinout
const uint8_t BCLK_PIN = 27; /* Bit clock */
const uint8_t LRCK_PIN = 26; /* Left-right clock */
const uint8_t DOUT_PIN = 25; /* DAC */
const uint8_t DIN_PIN = 33;  /* ADC */
const uint8_t MCLK_PIN = 0;  /* GPIO0, GPIO1, or GPIO3 are valid */

// I2S Config
const uint32_t I2S_SAMPLING_FREQ = 48000;          /* Sample rate */
const uint32_t I2S_BUFFER_SIZE = 64;               /* 64 samples per block (buffer delay?) */
const i2s_audio_t I2S_CHANNELS = I2S_AUDIO_STEREO; /* Number of channels (Stereo) */
I2SDriver mI2S0(I2S_NUM_0, BCLK_PIN, LRCK_PIN, DOUT_PIN, DIN_PIN, MCLK_PIN);

/* Guitar effect parameters ----------------------------------*/
uint32_t MAX_DELAY = 2;	// Application-dependent
RingBuffer xbuf(MAX_DELAY);
WaveGenertor wgen((float)I2S_SAMPLING_FREQ);
uint32_t n = 0;	// Discrete-time variable (timestamp)

/* Arduino Setup -------------------------------------*/
void setup() {
  /* Serial communication setup */
  Serial.begin(115200);
  Serial.println("ESP32-GUITAR PROCESSOR");
  
  /* Ring buffer initialization */
  xbuf.setTo(0.0F);
  
  /* Waveform generator initialization */
  wgen.setFrequency(5.0F);
  wgen.setWaveform(WAVEFORM_COSINE);
  
  /* I2S driver setup */
  if (mI2S0.setup(I2S_SAMPLING_FREQ, I2S_BUFFER_SIZE, I2S_CHANNELS, I2S_BITS_PER_SAMPLE_32BIT) != ESP_OK) {
    Serial.println("I2S.Setup.Error: MCU will be halted!");
    while (true);
  }
}


/* Arduino Main Loop ------------------------------------*/
void loop() {
  /* BLOCK READING BEGIN */
  size_t N = mI2S0.readBlock();
  /* BLOCK READING END */
  
  
  
  /* DSP PROCESSING BEGIN */ 
  for (int k = 0; n < N / 2; k++) {
    // Read current sample (right-channel only)
    int32_t sample = mI2S0.readFromBlock<int32_t>(I2S_CHANNEL_RIGHT, k);

    // Integer to float
	float Gpre = 1.0F; // Preamplification gain
	// Convert 24-bit sample to float
    float x = int24::toFloat(sample) * Gpre;
    
    // Audio loop-back (Output is the input)
    float y = x;
    // Or write here your own algorithm
	// At least 3 audio effect algorithms
    
	// Float to integer, since our sound card
	// only transmit integer values
    sample = int24::toInteger(y);
	
	// Write samples to buffer before writing to our sound card
    mI2S0.writeToBlock<int32_t>(sample, I2S_CHANNEL_LEFT, n);
    mI2S0.writeToBlock<int32_t>(sample, I2S_CHANNEL_RIGHT, n);
	
	// Increment timestamp
    n++;
  }
  /* DSP PROCESSING END */
  
  
  
  /* BLOCK WRITING BEGIN */
  // Here the processed block is transmitted via I2S
  mI2S0.writeBlock();
  /* BLOCK WRITING END */
}