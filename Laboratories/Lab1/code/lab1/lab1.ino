/*
 This example generates a set of waveforms at a given frequency
 and sample rate. Then outputs the data using the I2S protocol
 via MAX98357 circuit.
 */

#include <math.h>
#include "I2SDriver.h"
#include "WaveGenerator.h"

// I2S Pinout
const uint8_t BCLK_PIN = 27;  /* Bit clock */
const uint8_t LRCK_PIN = 26;  /* Left-right clock */
const uint8_t DOUT_PIN = 25;  /* DAC */
// I2S Config
const uint32_t I2S_SAMPLING_FREQ  = 20000;            /* Sample rate */
const uint32_t I2S_BUFFER_SIZE    = 512;              /* 512 samples per block */
const uint8_t I2S_CHANNELS        = I2S_AUDIO_STEREO; /* Number of channels (Stereo) */
I2SDriver mI2S0(I2S_NUM_0, BCLK_PIN, LRCK_PIN, DOUT_PIN);




uint32_t n = 0;
float Fx = 100.0F;
float Fs = (float)I2S_SAMPLING_FREQ;
bool incFreq = true;
waveform_t waveform = WAVEFORM_NONE;
WaveGenertor wavegen(Fs);

void setup() {
  Serial.begin(115200);
  Serial.printf("Fcore = %d MHz\n", getCpuFrequencyMhz());
  Serial.println("Laboratorio #1");
  
  /* CPU Core Clock */
  if( !setCpuFrequencyMhz(240) )
  {
    Serial.println("CPU.SetFrequency.Error: MCU will be halted!");
    while (true);
  }

  /* GPIO Setup */
  pinMode(T0, INPUT);
  
  /* I2S Driver Setup */
  if (mI2S0.setup(I2S_SAMPLING_FREQ, I2S_BUFFER_SIZE, I2S_CHANNELS, I2S_BITS_PER_SAMPLE_16BIT) != ESP_OK) {
    Serial.println("I2S.Setup.Error: MCU will be halted!");
    while (true);
  }
  
  // Discrete-time signal generation
  wavegen.setFrequency(Fx);
  wavegen.setAmplitude(0.5);  // Jus for MAX98357, since output is L/2 + R/2, it avoids audio clipping
  
}


void loop() {
  // Waveform selection
  if (touchRead(T0) < 30)
  {
    if(waveform == WAVEFORM_NOISE)
      waveform = WAVEFORM_NONE;
    else
      waveform = (waveform_t)((int)(waveform) + 1);
    wavegen.setWaveform(waveform);
    Serial.printf("Waveform: %s\n",getWaveformName(waveform).c_str());
    delay(1000);
  }

  // Frequency increments
  if(touchRead(T3) < 30)
  {
    if(incFreq)
    {
      if(Fx < 5000.0F)
        Fx += 100.0F;
      else
      {
        incFreq = false;
        Fx -= 100.0F;
      }
     
    } else {
      if(Fx > 100.0F)
        Fx -= 100.0F;
      else
      {
        incFreq = true;
        Fx += 100.0F;
      }
    }
    Serial.printf("Frequency: %.1f\n",Fx);
    wavegen.setFrequency(Fx);
    wavegen.setWaveform(waveform);
    delay(1000);
  }
  
  // Write to I2S
  int16_t samples[2]; // Left & Right
  samples[0] = mI2S0.toInteger<int16_t>(wavegen.sample(n));
  samples[1] = samples[0];
  mI2S0.writeFrame(samples);  // Left, right channel

  // increment the counter for the next sample
  n++;
}