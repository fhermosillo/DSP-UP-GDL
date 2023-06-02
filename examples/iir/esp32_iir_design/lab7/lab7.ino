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

/* DSP variables ----------------------------------*/
const uint32_t IIR_FILTER_TAPS = 5
const float iir_a_coefs[IIR_FILTER_TAPS] = {1.00000000000000000000	-3.86227311251867000408	5.63087456009529319090	-3.67242091442564078108	0.90416304086512311322}
const float iir_b_coefs[IIR_FILTER_TAPS] = {0.00120740519026005634	0.00000000000000000000	-0.00241481038052011269	0.00000000000000000000	0.00120740519026005634}

// Para filtros IIR, se requieren dos buffer de memoria:
// Uno para almacenar los valores pasados de la salida
float yprev[IIR_FILTER_TAPS-1];
// Y otro para almacenar los valores pasados de la entrada
float xprev[IIR_FILTER_TAPS-1];

/*
 * This function will be automatically called every time
 * a block of M samples from bluetooth arrive.
 */
void audio_process(const uint8_t *data, uint32_t len);



void setup() {
  /* Serial communication begin */
  Serial.begin(115200);

  // Initialize buffer values to zero
  for(int k = 0; k < IIR_FILTER_TAPS-1; k++) {
    yprev[k] = 0.0F;
    xprev[k] = 0.0F;
  }

  /* I2S Driver Setup */
  // Instead, we will use our I2S driver
  if (mI2S0.setup(I2S_SAMPLING_FREQ, I2S_BUFFER_SIZE, I2S_CHANNELS, I2S_BPS) != ESP_OK) {
    Serial.println("I2S.Setup.Error: MCU will be halted!");
    while (true);
  }
  
  /* Audio bluetooth device setup */
  BTDevice.set_stream_reader(audio_process, false); // Disable A2DP I2S Driver
  //a2dp_sink.set_auto_reconnect(false);
  BTDevice.start("ESP32-DSP-E1");

  
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
	for(uint32_t k = 0; k < N; k+=2)
	{
		// Get current sample
		// If 16-bits per sample is used
		int16_t sampleL = rxbuf[k];
		int16_t sampleR = rxbuf[k+1];
		
		
		/* DSP BEGIN */
    // Get x(n) normalized in the range of [0,1]
    float x = (mI2S0.toFloat<int16_t>(sampleL) + mI2S0.toFloat<int16_t>(sampleR))*0.5;

    // Implement IIR equation
    // y(n) = b[0]*x(n) + b[1]*x(n-1) + ... + b[M]*x(n-M) + a[1]*y(n-1) + ... + a[M]*y(n-M)
    //      = b[0]*x(n) + {b[1]*x(n-1) + a[1]*y(n-1)} + {b[2]*x(n-2) + a[2]*y(n-2)} + ... + {b[M]*x(n-M) + a[M]*y(n-M)}
    // a[0] = 1.0 therefore it doesn´t matter
    float y = iir_b_coefs[0]*x; // b[0]*x(n)
    for(int k = 0; k < IIR_FILTER_TAPS-1; k++) {
      y += iir_b_coefs[k+1]*xprev[k] + iir_a_coefs[k+1]*yprev[k]; // b[k+1]*x(n-k-1) + a[k+1]*y(n-k-1)
    }
    sampleL = mI2S0.toInteger<int16_t>(y);
    sampleR = sampleL;
		/* DSP END */
		
		
		// Output processed sample
		mI2S0.writeToBlock<int16_t>(sampleL, m, I2S_CHANNEL_LEFT); 	// Write sample to a temporal buffer (not I2S)
		mI2S0.writeToBlock<int16_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)
		
		// Increment the pointer to next sample in our I2S DRIVER
		m++;
	}
	
	mI2S0.writeBlock();
}