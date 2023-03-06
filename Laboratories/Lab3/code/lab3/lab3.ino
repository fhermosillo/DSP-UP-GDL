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
const size_t DSP_FIR_TAPS = 21;
const uint32_t DSP_IIR_TAPS = 3;

// FIR
float h1[DSP_FIR_TAPS] = {0.00539815856759449583,	0.00744752819464048268,	0.01300141135755400815,	0.02206724439955183581,	0.03412079558048336514,	0.04814633340660173788,	0.06275583442171613069,	0.07637102530405254219,	0.08744128951122645543,	0.09466399349295646914,	0.09717277152724483613,	0.09466399349295646914,	0.08744128951122645543,	0.07637102530405254219,	0.06275583442171613069,	0.04814633340660173788,	0.03412079558048336514,	0.02206724439955183581,	0.01300141135755400815,	0.00744752819464048268,	0.00539815856759449583};
float h2[DSP_FIR_TAPS] = {-0.00252552033472454958,	-0.00348431482022589856,	-0.00608269067172906384,	-0.01032412697117921115,	-0.01596336269052767154,	-0.02252518938416156122,	-0.02936022237399318544,	-0.03573006886959533002,	-0.04090927526301737366,	-0.04428840641471364775,	0.95697792974823248091,	-0.04428840641471364775,	-0.04090927526301737366,	-0.03573006886959533002,	-0.02936022237399318544,	-0.02252518938416156122,	-0.01596336269052767154,	-0.01032412697117921115,	-0.00608269067172906384,	-0.00348431482022589856,	-0.00252552033472454958};
float h3[DSP_FIR_TAPS] = {0.00024532611221641100,	0.01289279674287206745,	0.02982047267812083791,	0.03388462985502545960,	-0.00077376890732898677,	-0.07077665087391081522,	-0.12629169759639238824,	-0.10615272149127299994,	0.00060768129157881252,	0.12983770253771576009,	0.18767247716735585561,	0.12983770253771576009,	0.00060768129157881252,	-0.10615272149127299994,	-0.12629169759639238824,	-0.07077665087391081522,	-0.00077376890732898677,	0.03388462985502545960,	0.02982047267812083791,	0.01289279674287206745,	0.00024532611221641100};

// IIR
float acoef[DSP_IIR_TAPS] = {1.00000000000000000000,	-1.32573006828399653223,	0.86678843949963502169};
float bcoef[DSP_IIR_TAPS] = {0.93339421974981739982,	-1.32573006828399631019,	0.93339421974981739982};

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
	for(uint32_t k = 0; k < N; k+=2)
	{
		// Get current sample
		// If 16-bits per sample is used
		int16_t sampleL = rxbuf[k];
		int16_t sampleR = rxbuf[k+1];
		// If 32-bits per sample is used
		//int32_t sampleL = rxbuf[k]*32768;
		//int32_t sampleR = rxbuf[k+1]*32768;
		
		
		/* DSP BEGIN */
		// Implement discrete-time convolution for FIR systems here
		// y(n) = x(n)h(0) + x(n-1)*h(1) + ... + x(n-M)*h(M);
		
		// Or, implement IIR filter difference equation here
		// y(n) = b(0)*x(n) + b(1)*x(n - 1) + ... + b(M)*x(n - M) - a(1)*y(n - 1) - ... - a(M)*y(n - M)
		/* DSP END */
		
		
		// Output processed sample
		// If 16-bits per sample is used
		mI2S0.writeToBlock<int16_t>(sampleL, m, I2S_CHANNEL_LEFT); 	// Write sample to a temporal buffer (not I2S)
		mI2S0.writeToBlock<int16_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)
		// If 32-bits per sample is used
		//mI2S0.writeToBlock<int32_t>(sampleL, m, I2S_CHANNEL_LEFT); // Write sample to a temporal buffer (not I2S)
		//mI2S0.writeToBlock<int32_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)
		
		// Increment the pointer to next sample in our I2S DRIVER
		m++;
	}
	
	mI2S0.writeBlock();
}