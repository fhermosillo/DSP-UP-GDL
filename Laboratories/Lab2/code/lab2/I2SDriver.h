/*
 * 	I2SDriver.h
 *
 *  Author: Fernando Hermosillo Reynoso
 * 	fernando.hermosillo@hotmail.com
 */

#ifndef __I2SDRIVER_H
#define __I2SDRIVER_H

#include <Arduino.h>
#include <cmath>

// include espressif hw files
#include <driver/i2s.h>

/* Master Clock GPIO */
// NONE
#define I2S_MCLK_DISABLE I2S_PIN_NO_CHANGE
// GPIO0
#define I2S_MCLK_CLKOUT1  0
// GPIO3
#define I2S_MCLK_CLKOUT2  3
// GPIO1
#define I2S_MCLK_CLKOUT3  1

/* I2S AUDIO CHANNELS */
#define I2S_AUDIO_MONO    1
#define I2S_AUDIO_STEREO  2

/* I2S CHANNEL SELECTION */
#define I2S_CHANNEL_LEFT  0
#define I2S_CHANNEL_RIGHT 1

class I2SDriver {
public:
  I2SDriver(i2s_port_t _i2s_port = I2S_NUM_0, uint8_t _bclk_pin = 27, uint8_t _lrck_pin = 26, int8_t _dout_pin = 25, int8_t _din_pin = I2S_PIN_NO_CHANGE, int8_t _mck_pin = I2S_PIN_NO_CHANGE);
  
  // I2S init/deinit methods
  esp_err_t setup(uint32_t _sample_rate = 8000, uint32_t _buffer_size = 8, uint8_t _channel_count = 2, i2s_bits_per_sample_t _bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, i2s_channel_fmt_t _channel_fmt = I2S_CHANNEL_FMT_RIGHT_LEFT);
  void close(void);

  // I2S start/stop methods  
  esp_err_t start(void);
  esp_err_t stop(void);

  // I2S W/R methods
  bool writeItem(void *src);                                // Write item to I2S directly
  bool writeFrame(void *frame);                             // Write a sample sample will be 
  bool readItem(void *dst);                                 //

  template<typename T>
  void writeToBlock(T src, uint32_t n, uint8_t channel) { // Write to buffer (not to I2S
    T *buf = (T *)wr_buffer;
    buf[mChannelCount * n + channel] = src;
  }

  template<typename T>
  T readFromBlock(uint32_t n, uint8_t channel) {          // Read from buffer (not from I2S)
    T *buf = (T *)rd_buffer;
    
    return buf[mChannelCount * n + channel];
  }
  
  size_t writeBlock(void);                                  // Write buffer to I2S
  size_t readBlock(void);                                   // Read buffer from I2S
  
  uint8_t ChannelCount(void);
  size_t BufferSize(void);
  
  // Data type conversions methods
  template<typename T>
  T toInteger(float sample) {
    // Saturation
    sample = min(sample, 1.0F-mInt2FloatScale);
    sample = max(sample, -1.0F);
    
    // Conversion
    sample *= mFloat2IntScale;
    
    return (T)(sample);
  }

  template<typename T>
  float toFloat(T sample) {
    return (float)(sample) * mInt2FloatScale;
  }

  // Get methods
  
  
  // Destructor method
  ~I2SDriver();
  


private:
  // I2S pinout
  i2s_pin_config_t i2s_pinout;

  // I2S ESP32 parameters
  i2s_port_t i2s_port;
  i2s_config_t i2s_config;
  uint8_t mChannelCount;
  i2s_mode_t i2s_mode;

  // I2S parameters
  bool need_tx;
  bool need_rx;
  uint32_t sample_rate;
  uint32_t buffer_size;
  int8_t *wr_buffer;
  int8_t *rd_buffer;

  float mInt2FloatScale;
  float mFloat2IntScale;
  uint8_t mBytesPerSample;
  
};

#endif /* __I2SDRIVER_H */