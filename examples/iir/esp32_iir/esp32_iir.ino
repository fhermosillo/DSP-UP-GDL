/*
 * Ejemplo de implementación del filtro IIR en la ESP32. El filtro en cuestion es aquel 
 * que tiene respuesta al impulso h(n) = cos(wo n), es decir nos permitira generar una 
 * señal senoidal apartir de una ecuación en diferencias:
 * Para ello se lleva al dominio z la función h(n), obteniendo con ello que
 *            1 - cos(wo)*z^-1          Y(z)
 * H(z) = ------------------------- =  ------
 *        1 - 2*cos(wo)*z^-1 + z^-2     X(z)
 *
 * Recuperando la ecuación en diferencias original
 * Y(z)[1 - 2*cos(wo)*z^-1 + z^-2] = X(z)[1 - cos(wo)*z^-1]
 *
 * Si regresamos al tiempo esta ecuacion obtenemos que
 * y(n) - 2*cos(wo)*y(n - 1) + y(n - 2) = x(n) - cos(wo)*x(n - 1)
 *
 * Despejando para y(n)
 *  y(n) = x(n) - cos(wo)*x(n - 1) + 2*cos(wo)*y(n - 1) - y(n - 2)
 *
 * Esa es la ecuacion en diferencias a implementar en el microcontrolador
 * la cual pose un atraso de 2 para y(n), y un atrazo de 1 para x(n)
 *
 * Sin embargo aqui lo que nos interesa es calcular a h(n) y no a y(n)
 * De teoria se sabe que h(n) = y(n) cuando x(n) = delta(n) = d(n),
 *
 * h(n) = d(n) - cos(wo)*d(n - 1) + 2*cos(wo)*h(n - 1) - h(n - 2)
 */
 
/* Private includes ------------------------------------------------------------*/
#include <BluetoothA2DPSink.h>
#include "I2SDriver.h"

/* Private defines --------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Private macros --------------------------------------------------------------*/

/* Private typedefs --------------------------------------------------------------*/

/* Private prototype functions ---------------------------------------------------*/
/*
 * This function will be automatically called every time
 * a block of M samples from bluetooth arrive.
 */
void audio_process(const uint8_t *data, uint32_t len);


// Calcula delta(n - k)
float delta(uint32_t n, uint32_t k);

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

// Variable de tiempo discreto
uint32_t n = 0;

// De la ecuación en diferencias
// h(n) = 2*cos(wo)*h(n - 1) - h(n - 2)
// Se requiere un buffer de valores pasados de la salida de 2 elementos
// a fin de almacenar el atraso maximo de y que es 2 de y(n-2)
float yprev[2] = {0.0F, 0.0F}; // {y(n-1), y(n-2)}
// Para la entrada se tiene un atrazo maximo de 1, por lo que solo se
// necesita un solo valor a almacenr
float xprev = 0.0F; // x(n-1)

// Se puede modificar este parametro para cambiar la frecuencia de la
// señal de salida, en este caso, la frecuencia de tiempo continuo es
// (pi/50)/(2*pi) = 1/100 -> para recuperar la frecuencia hay que des-
// normalizar multiplicando por FS, es este caso FS=44100
// por lo que F = 44100/100 = 441Hz
float cos_wo = cos(M_PI/50.0F);  // Constante cos(pi/50)

/*
 * Setup function
 */
void setup() {
  /* Serial communication begin */
  Serial.begin(115200);
  delay(1000);

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


float delta(uint32_t n, uint32_t k) {
  return( n == k ? 1.0F : 0.0F);
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
    float x = mI2S0.toFloat<int16_t>(sampleL);

    // Calculo del la salida del filtro iir
    // Este filtro IIR va a calcular
    // Aqui se implementa la ecuación 
    // h(n) = delta(n) - cos(wo)*delta(n - 1) + 2*cos(wo)*h(n - 1) - h(n - 2)
    float y = delta(n,0) - cos_wo*delta(n,1) + 2*cos_wo*yprev[0] - yprev[1];

    // Actualizar los valores pasados de x(n) y de y(n)
    xprev = delta(n,0);
    yprev[1] = yprev[0];
    yprev[0] = y;
    n++;
    // Fin de calculo de salida del filtro iir -------------
    
    // Codificar a y en un entero de 16 bits
    sampleL = mI2S0.toInteger<int16_t>(y);
    sampleR = sampleL;
		/* DSP END --------------------------------------*/
		
    
		// Output processed sample
    mI2S0.writeToBlock<int16_t>(sampleL, m, I2S_CHANNEL_LEFT); // Write sample to a temporal buffer (not I2S)
    mI2S0.writeToBlock<int16_t>(sampleR, m, I2S_CHANNEL_RIGHT); // Write sample to a temporal buffer (not I2S)

    // Increment the pointer to next sample in our I2S DRIVER
    m++;
	}

  mI2S0.writeBlock();
}