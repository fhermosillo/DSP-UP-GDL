/*
 * Ejemplo de implementación de un filtro adaptativo en la ESP32 basado en el
 * diseño de "tutorial_2_filtro_adaptativo.m" del repositorio 
 *    
 * Este tipo de filtros asume que existe una señal de referencia, la cual es
 * la señal deseada d(n) a obtener al pasar la señal medida x(n) por nuestro
 * filtro adaptativo h(n) que se asume de tener N coeficientes. El modelo de
 * la señal aproximada es el siguiente:
 *                   dh(n) = x(n) * h(n)
 *
 * Se conoce al filtro como adaptativo justamente por ello, dado que la 
 * señal medida x(n) pudiera cambiar, el filtro necesita adaptarse a esos 
 * cambios y proveer la mejor aproximación de la señal de referencia d(n). 
 * Esto va a introducir un error por la aproximación no exacta, dado por
 *                   e(n) = d(n) - dh(n) = d(n) - x(n) * h(n)
 */
 
/* Private includes ------------------------------------------------------------*/
#include <BluetoothA2DPSink.h>
#include "I2SDriver.h"

/* Private defines --------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846F
#endif

/* Private macros --------------------------------------------------------------*/

/* Private typedefs --------------------------------------------------------------*/

/* Private prototype functions ---------------------------------------------------*/
/*
 * This function will be automatically called every time
 * a block of M samples from bluetooth arrive.
 */
void audio_process(const uint8_t *data, uint32_t len);

float inner_prod(const float *x1, const float *x2, uint32_t len) {
  float inner = 0.0F;
  for(int k = 0; k < len; k++)
    inner += x1[k]*x2[k];
  
  return inner;
}

/* Private constants --------------------------------------------------------------*/


/* Private variables -------------------------------------------------------------*/
// Bluetooth Receiver --------
BluetoothA2DPSink BTDevice;

// I2S Driver ---------------
// I2S Pinout
const uint8_t BCLK_PIN = 27;  /* Bit clock */
const uint8_t LRCK_PIN = 26;  /* Left-right clock */
const uint8_t DOUT_PIN = 25;  /* DAC */

// I2S Config (use always this configuration)
const uint32_t I2S_SAMPLING_FREQ  = 44100;                        /* Sample rate */
const uint32_t I2S_BUFFER_SIZE    = 1024;                         /* 1024 stereo samples per block */
const uint8_t I2S_CHANNELS        = 2; 		                        /* Number of channels (Stereo) */
const i2s_bits_per_sample_t I2S_BPS = I2S_BITS_PER_SAMPLE_16BIT;  /* BPS can be changed */
I2SDriver mI2S0(I2S_NUM_0, BCLK_PIN, LRCK_PIN, DOUT_PIN);


/**/
const uint32_t ADAPTIVE_FILTER_TAPS = 16;
float h[ADAPTIVE_FILTER_TAPS];      // [h(0), ..., h(ADAPTIVE_FILTER_ORDER-1)]
float xvec[ADAPTIVE_FILTER_TAPS];  // [x(n),x(n-1),...,x(n-ADAPTIVE_FILTER_ORDER+1)]
float step_size = 0.00001;
uint32_t n = 0; // Time index
/*
 * Setup function
 */
void setup() {
  /* Serial communication begin */
  Serial.begin(115200);

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
  uint32_t m = 0; // Used to increment 1 by 1 time index
	for(uint32_t k = 0; k < N; k+=2)
	{
		// Get current sample
		int16_t sampleL = rxbuf[k];
    int16_t sampleR = rxbuf[k+1];

    
		/* DSP BEGIN -----------------------------------*/
		// Cuantificar la entrada entera en flotante
    float dn = (mI2S0.toFloat<int16_t>(sampleL) + mI2S0.toFloat<int16_t>(sampleR))*0.5; // STEREO TO MONO

    // Leer x(n)
    float x = sin(2.0*M_PI*(60.0F/(float)I2S_SAMPLING_FREQ)*n); // @TODO: Reemplazar "sin" por ec. en diferencias
    
    // % Vector de los M valores pasados de x(n)
    for(int k = ADAPTIVE_FILTER_TAPS-1; k>0; k--) {
      xvec[k] = xvec[k-1];
    }
    xvec[0] = x;
    
    // Calcular el error e(n) = d(n) - x(n)*h(n)
    float err = dn - inner_prod(xvec, h, ADAPTIVE_FILTER_TAPS);
    
    // Update h^(k) = h^(k-1) + 2*step*e(n)*x
    for(int k = 0; k < ADAPTIVE_FILTER_TAPS; k++) {
      h[k] = h[k] + 2*step_size*err*xvec[k];
    }

    // Estimar la señal dh(n) = x(n) * h(n)
    float dhat = inner_prod(xvec, h, ADAPTIVE_FILTER_TAPS);
    
    // Codificar dhat como un entero de 16 bits
    sampleL = mI2S0.toInteger<int16_t>(err);
    sampleR = sampleL;
		/* DSP END --------------------------------------*/
		
    
		// Output processed sample
    mI2S0.writeToBlock<int16_t>(sampleL, m, I2S_CHANNEL_LEFT); // Write sample to a temporal buffer (not I2S)
    mI2S0.writeToBlock<int16_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)

    // Increment the pointer to next sample in our I2S DRIVER
    m++;
	}

  // I2S transmitira el bloque completo de muestras procesadas
  mI2S0.writeBlock();
}