/*
 * 	I2SDriver.h
 *
 *  Author: Fernando Hermosillo Reynoso
 * 	fernando.hermosillo@hotmail.com
 */

#ifndef __I2SADC_H
#define __I2SADC_H

#include <Arduino.h>
// include espressif hw files
#include <driver/i2s.h>
#include <driver/adc.h>
#include <hal/adc_types.h>


typedef union {
  struct {
    uint8_t atten : 2;       /*!< Pattern table ADC attenuation */
    uint8_t bit_width:  2;   /*!< Pattern table ADC capture width */
    uint8_t channel : 4;     /*!< Pattern table ADC channel number */  
  };
  uint8_t value;
} adc_i2s_pattern_t;


/*
 */
class I2SADC {
  public:
    I2SADC(uint32_t rate, uint32_t buffer_len);
    
    esp_err_t begin(adc_i2s_pattern_t *patterns, uint8_t num_patterns);
    void end(void);

    uint32_t length(void);
    uint32_t read(uint16_t *data, uint32_t len);
    uint8_t Channels(void);
    ~I2SADC();
    
  private:
    /* Hardware */
    i2s_config_t mI2sConfig;
    uint32_t mSampleRate;
    uint8_t mNumChannels;
    /* Buffer */
    uint32_t mBufferLen;
};

#endif /* __I2SDRIVER_H */