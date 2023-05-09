#include "Arduino.h"
#include <cmath>
#include <algorithm>
#include "I2SDriver.h"


I2SDriver::I2SDriver(i2s_port_t _i2s_port, uint8_t _bclk_pin, uint8_t _lrck_pin, int8_t _dout_pin, int8_t _din_pin, int8_t _mck_pin) {
  // Select I2S port
  i2s_port = _i2s_port;

  // I2S pinout
  i2s_pinout.bck_io_num = _bclk_pin;
  i2s_pinout.ws_io_num = _lrck_pin;  //  wclk
  i2s_pinout.data_out_num = _dout_pin;
  i2s_pinout.data_in_num = _din_pin;
  i2s_pinout.mck_io_num = _mck_pin;

  // I2S mode
  uint8_t mode = (uint8_t)I2S_MODE_MASTER;
  need_tx = false;
  need_rx = false;
  if (_din_pin > 0) {
    mode |= (uint8_t)I2S_MODE_RX;
    need_rx = true;
  }
  if (_dout_pin > 0) {
    mode |= (uint8_t)I2S_MODE_TX;
    need_tx = true;
  }

  i2s_mode = (i2s_mode_t)mode;

  mBytesPerSample = 0;
}


esp_err_t I2SDriver::setup(uint32_t _sample_rate, uint32_t _buffer_size, uint8_t _channel_count, i2s_bits_per_sample_t _bits_per_sample, i2s_channel_fmt_t _channel_fmt) {

  // Get scaling parameters
  switch (_bits_per_sample) {
    case I2S_BITS_PER_SAMPLE_8BIT:
    mFloat2IntScale = 128.0F;
    mInt2FloatScale = 1.0F/128.0F;
    mBytesPerSample = 1;
    break;

    case I2S_BITS_PER_SAMPLE_16BIT:
    mFloat2IntScale = 32768.0F;
    mInt2FloatScale = 1.0F/32768.0F;
    mBytesPerSample = 2;
    break;

    case I2S_BITS_PER_SAMPLE_24BIT:
    mFloat2IntScale = 8388608.0F;
    mInt2FloatScale = 1.0F/8388608.0F;
    mBytesPerSample = 4;
    break;

    case I2S_BITS_PER_SAMPLE_32BIT:
    mFloat2IntScale = 2147483648.0F;
    mInt2FloatScale = 1.0F/2147483648.0F;
    mBytesPerSample = 4;
    break;

    default:
    mFloat2IntScale = 1.0F;
    mInt2FloatScale = 1.0F;
    mBytesPerSample = 0;
    break;
  }
  
  // I2S configuration
  i2s_config.sample_rate = _sample_rate;
  i2s_config.bits_per_sample = _bits_per_sample;
  i2s_config.channel_format = _channel_fmt;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;  // interrupt priority
  i2s_config.dma_buf_count = 2;
  i2s_config.dma_buf_len = _buffer_size;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = true;
  i2s_config.fixed_mclk = I2S_PIN_NO_CHANGE;
  i2s_config.mode = i2s_mode;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  mChannelCount = _channel_count;

  // Initialize buffers
  // I2S buffers
  buffer_size = mChannelCount * _buffer_size;
  if(need_rx)
    rd_buffer = (int8_t *)calloc(buffer_size, mBytesPerSample);
  if(need_tx)
  wr_buffer = (int8_t *)calloc(buffer_size, mBytesPerSample);

  // Enable Master clock on GPIO0 (if needed)
  // Enable Master clock
  // IO_MUX_PIN_CTRL
  // [ RESERVED | CLK3 | CLK2 | CLK1 ]
  // [   31:12  | 11:8 |  7:4 | 3:0  ]
  // PIN_CTRL.CLK1 = 0x0 -> Setup CLK_OUT1 on GPIO0
  // PIN_CTRL.CLK1 = 0x0, PIN_CTRL.CLK2 = 0x0 -> Setup CLK_OUT2 on GPIO3
  // PIN_CTRL.CLK1 = 0x0, PIN_CTRL.CLK3 = 0x0 -> Setup CLK_OUT3 on GPIO1
  switch(i2s_pinout.mck_io_num)
  {
    case I2S_MCLK_CLKOUT1:
      REG_WRITE(PIN_CTRL, 0xFF0); 
      PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    break;

    case I2S_MCLK_CLKOUT2:
      REG_WRITE(PIN_CTRL, 0xF00); 
      PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
    break;

    case I2S_MCLK_CLKOUT3:
      REG_WRITE(PIN_CTRL, 0x0F0); 
      PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
    break;

    case I2S_MCLK_DISABLE:
      Serial.println("I2S.Setup.Mclk.Disable");
    break;

    default:
      Serial.println("I2S.Setup.Mclk.GPIO.Invalid");
    break;
  }

  // I2S install driver
  esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);

  // I2S set pinout
  err += i2s_set_pin(i2s_port, &i2s_pinout);

  // Set sample rate
  err += i2s_set_sample_rates(i2s_port, _sample_rate);

  // Clear I2S buffer
  err += i2s_zero_dma_buffer(i2s_port);

  // Start I2S
  err += start();

  return err;
}


void I2SDriver::close(void) {
  stop();
  i2s_driver_uninstall(i2s_port);
}


esp_err_t I2SDriver::start(void) {
  return i2s_start(i2s_port);
}


esp_err_t I2SDriver::stop(void) {
  return i2s_stop(i2s_port);
}


size_t I2SDriver::BufferSize(void)
{
  return buffer_size / mChannelCount;
}



bool I2SDriver::writeItem(void * src) {
  size_t bytesWritten = 0;
  esp_err_t err = i2s_write(i2s_port, (int8_t *)src, mBytesPerSample, &bytesWritten, 100);

  if ((err != ESP_OK) || (bytesWritten < mBytesPerSample)) {
    Serial.print("I2S.Write.Error: ");
    Serial.println(err);
    return false;
  }

  return true;
}

bool I2SDriver::writeFrame(void *frame)
{
  size_t bytesWritten = 0;
  esp_err_t err = i2s_write(i2s_port, (int8_t *)frame, mBytesPerSample * mChannelCount, &bytesWritten, 100);

  if ((err != ESP_OK) || (bytesWritten != mBytesPerSample * mChannelCount)) {
    Serial.print("I2S.Write.Error: ");
    Serial.println(bytesWritten);
    return false;
  }

  return true;
}


bool I2SDriver::readItem(void *dst) {
  size_t bytesRead = 0;

  esp_err_t err = i2s_read(i2s_port, (int8_t *)dst, mBytesPerSample, &bytesRead, 100);

  if ((err != ESP_OK) || (bytesRead < mBytesPerSample)) {
    Serial.print("I2S.Read.Error: ");
    Serial.println(err);
    return false;
  }

  return true;
}


size_t I2SDriver::writeBlock(void) {
  size_t bytesWritten = 0;
  esp_err_t err = i2s_write(i2s_port, (const char *)wr_buffer, buffer_size * mBytesPerSample, &bytesWritten, 500);

  if ((err != ESP_OK) || (bytesWritten < (buffer_size * mBytesPerSample))) {
    Serial.print("I2S.WriteBlock.Error: ");
    Serial.println(bytesWritten);
  } else
  {
      bytesWritten /= mBytesPerSample;
  }

  return bytesWritten;
}


size_t I2SDriver::readBlock(void) {
  size_t bytesRead = 0;

  esp_err_t err = i2s_read(i2s_port, (void *)rd_buffer, buffer_size * mBytesPerSample, &bytesRead, 500);

  if ((err != ESP_OK) || (bytesRead < (buffer_size * mBytesPerSample))) {
    Serial.print("I2S.ReadBlock.Error: ");
    Serial.println(bytesRead);
  } else
  {
      bytesRead /= mBytesPerSample;
  }
  
  return bytesRead;
}


uint8_t I2SDriver::ChannelCount(void) {
  return mChannelCount;
}


I2SDriver::~I2SDriver() {
  close();
}