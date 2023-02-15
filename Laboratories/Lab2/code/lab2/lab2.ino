#include <BluetoothA2DPSink.h>
#include "I2SDriver.h"


// Bluetooth Receiver -------------------------------------------------------------
BluetoothA2DPSink BTDevice;

// I2S Driver ---------------------------------------------------------------------
// I2S Pinout
const uint8_t BCLK_PIN = 27;  /* Bit clock */
const uint8_t LRCK_PIN = 26;  /* Left-right clock */
const uint8_t DOUT_PIN = 25;  /* DAC */
// I2S Config (use always this configuration)
const uint32_t I2S_SAMPLING_FREQ  = 44100;  /* Sample rate */
const uint32_t I2S_BUFFER_SIZE    = 1024;   /* 1024 stereo samples per block */
const uint8_t I2S_CHANNELS        = 2; 		  /* Number of channels (Stereo) */
const i2s_bits_per_sample_t I2S_BPS = I2S_BITS_PER_SAMPLE_16BIT;  /* BPS can be changed */
I2SDriver mI2S0(I2S_NUM_0, BCLK_PIN, LRCK_PIN, DOUT_PIN);


/*
 * This function will be automatically called every time
 * a block of M samples from bluetooth arrive.
 */
void audio_process(const uint8_t *data, uint32_t len);



void setup() {
  /* Serial communication begin */
  Serial.begin(115200);
  
  /* Audio bluetooth device setup */
  BTDevice.set_stream_reader(audio_process, false); // Disable A2DP I2S Driver
  //a2dp_sink.set_auto_reconnect(false);
  BTDevice.start("ESP32-DSP-E1");

  /* I2S Driver Setup */
  // Instead, we will use our I2S driver
  if (mI2S0.setup(I2S_SAMPLING_FREQ, I2S_BUFFER_SIZE, I2S_CHANNELS, I2S_BPS) != ESP_OK) {
    Serial.println("I2S.Setup.Error: MCU will be halted!");
    while (true);
  }
}

void loop() {
}


void audio_process(const uint8_t *data, uint32_t len)
{
	// Casting to 16-bits items
	int16_t *rxbuf = (int16_t *)data; // Received buffer is always 16-bits per sample
	
	/* Sice the "data" variable is 8-bits, 
	 * but our sample is 16-bit we divided 
	 * by 2, e.g., if len = 32, there are 
	 * 32 bytes in "data", but there are 
	 * 32/2 = 16 samples of 16-bits
	 */
	uint32_t N = len/2;
  uint32_t m = 0;
	for(uint32_t n = 0; n < N; n+=2)
	{
		// Get current sample
    // If 16-bits per sample is used
    //int16_t samples[2] = {rxbuf[n],rxbuf[n+1]};
		int16_t sampleL = rxbuf[n];
    int16_t sampleR = rxbuf[n+1];
		// If 32-bits per sample is used
    //int32_t sampleL = rxbuf[n]*32768
    //int32_t sampleR = rxbuf[n+1]*32768

    
		/* DSP BEGIN */
		// Here you can write your own DSP algorithm
    
		/* DSP END */
		
    
		// Output processed sample
    // If 16-bits per sample is used
    mI2S0.writeToBlock<int16_t>(sampleL, m, I2S_CHANNEL_LEFT); // Write sample to a temporal buffer (not I2S)
    mI2S0.writeToBlock<int16_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)
		//mI2S0.writeFrame(samples);   // Write stereo sample via I2S directly
    // If 32-bits per sample is used
    //mI2S0.writeToBlock<int32_t>(sampleL, m, I2S_CHANNEL_LEFT); // Write sample to a temporal buffer (not I2S)
    //mI2S0.writeToBlock<int32_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)

    // Increment the pointer to next sample in our I2S DRIVER
    m++;
	}

  mI2S0.writeBlock();
}