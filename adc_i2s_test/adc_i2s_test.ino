/* Private Includes ----------------------------*/
#include "I2SADC.h"
#include "Cosgen.h"
#include <phyphoxBle.h>

    
/* Private Constants ---------------------------*/
const dac_channel_t DAC_CHANNEL = DAC_CHANNEL_2;    /* CHANNEL_1 = GPIO_*/
const adc1_channel_t ADC_CHANNEL = ADC1_CHANNEL_6;  /* CHANNEL_6 = GPIO_34 */
const uint32_t I2S_SAMPLE_RATE = 150000;
const uint32_t ADC_BUFFER_SIZE = 1024; // Ns * NChannels * (BITS_PER_SAMPLE/8) -> 8 to 1024
const uint8_t ADC_NUM_CHANNELS = 1;
const uint32_t ADC_NUM_SAMPLES = ADC_BUFFER_SIZE/(ADC_NUM_CHANNELS*2);



const int PWM_GPIO = 18;    /* GPIO_18 */
const int PWM_CHANNEL = 0;  /**/
const int PWM_FREQ = 1000;    /* PWM signal frequency */
const int PWM_RES = 8;      /* PWM resolution (up to 16-bits) */

/* Private Prototype Function ------------------*/
void vADCReaderTask(void *pvParams);

/* Private Global Variables --------------------*/
uint16_t adc_buffer[ADC_NUM_SAMPLES];
I2SADC mI2SADC(I2S_SAMPLE_RATE, ADC_BUFFER_SIZE);
CWDac mCosgen;

/* Arduino Main Code ---------------------------*/
void setup() {
  Serial.begin(115200);

  /* PhyPhox */
  PhyphoxBLE::start("DSP-MULTICHANNEL");
  
  /* PWM */
  uint8_t velocity = 128;
  uint32_t Fpwm = ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);  /*PWM signal defined*/
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, velocity);
  
  Serial.printf("F(pwm) = %lu Hz\n", Fpwm);
  
  /* Cosine wave generator (CW) */
  mCosgen.setPhase(CW_PHASE_90);
  mCosgen.setOffset(0);
  mCosgen.setScale(CW_SCALE_DIV2);  
  uint32_t Fdac = mCosgen.setFrequency(1000);
  mCosgen.start();
  Serial.printf("F(dac) = %lu Hz\n", Fdac);
  
  /* I2SADC  */
  adc_i2s_pattern_t patterns[4] = {
    {{.atten = ADC_ATTEN_DB_11, .bit_width = ADC_WIDTH_BIT_12, .channel=ADC1_CHANNEL_6}},
    {{.atten = ADC_ATTEN_DB_11, .bit_width = ADC_WIDTH_BIT_12, .channel=ADC1_CHANNEL_3}},
    {{.atten = ADC_ATTEN_DB_11, .bit_width = ADC_WIDTH_BIT_12, .channel=ADC1_CHANNEL_2}},
    {{.atten = ADC_ATTEN_DB_11, .bit_width = ADC_WIDTH_BIT_12, .channel=ADC1_CHANNEL_0}}
  }; // SEEMS TO BE SAMPLED DESCENDING ORDER, E.G., 0,6,3,2 WHILE WE EXPECT {2,3,6,0}?
  // 3,2,0,6 -> 0,6,3,2
  
  if ( mI2SADC.begin(patterns, ADC_NUM_CHANNELS) != ESP_OK) {
    Serial.println("I2SADC.Setup.Error: MCU will be halted!");
    while (true);
  }
  

  /* Add a ADC Reader Task to the schedule */
  xTaskCreate(
    vADCReaderTask, // Function that should be called
    "AdcReader",    // Name of the task (for debugging)
    8000,           // Stack size (bytes)
    &mI2SADC,       // Parameter to pass
    1,              // Task priority
    NULL            // Task handle
  );
}

void loop() {
}



/* Private Prototype Function ------------------*/
float adc_channel_f[1024];
void vADCReaderTask(void *pvParams)
{
  // I2SADC object
  I2SADC *mI2SADC = (I2SADC *)pvParams;
  
  // Infinite loop
  while(true)
  {
    // Wait until ADC buffer is filled (meanwhile other tasks can be executed)
    uint32_t N = mI2SADC->read(adc_buffer, 128);
    
    /* TODO */
    // Generate a cosine waveform of 50KHz, 100KHz, 150KHz, 200KHz, 250KHz
    // Sample Rate = 1MHz, 1.5MHz, 2MHz
    // Check samples and test if correct
    /* END TODO */
    
    // Do whatever with data
    Serial.print("x(n) = {");
    for(int n = 0; n < 20; n++)  //mI2SADC->length()
    {
      float x = (float)(adc_buffer[n] & 0x0FFF)/4095.0F;
      uint16_t adc_channel = (adc_buffer[n] >> 12) & 0x000F;
      Serial.printf("%.4f\t",x);
      //adc_channel_f[n] = (float)adc_channel;
      /*if(adc_channel == 6)
        Serial.printf("%.4f\n",x);*/
        
      //Serial.printf("ADC.CH<%d> = %.6f\n", adc_channel, x);
      //PhyphoxBLE::write(adc_channel_f[n]);
    }
    Serial.println("}\n");
    //PhyphoxBLE::write(adc_channel_f, N);
    delay(5000);
  }
}