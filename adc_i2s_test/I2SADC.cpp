#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2s.h"
#include "soc/sens_reg.h"
#include "I2SADC.h"
#include <driver/adc.h>
#include <hal/adc_types.h>
#include <driver/adc_common.h>
#include <esp_err.h>
#include <hal/i2s_types.h>
#include <driver/rtc_io.h>
#include <soc/soc.h>
#include <soc/adc_channel.h>
#include <soc/apb_ctrl_reg.h>
#include <soc/syscon_reg.h>
#include <driver/periph_ctrl.h>



#define SAR_ADC_CLK_DIV_DEFAULT       (2)
#define ADC_FSM_RSTB_WAIT_DEFAULT     (8)
#define ADC_FSM_START_WAIT_DEFAULT    (5)
#define ADC_FSM_STANDBY_WAIT_DEFAULT  (100)
#define ADC_FSM_TIME_KEEP             (-1)
#define ADC_MAX_MEAS_NUM_DEFAULT      (255)
#define ADC_MEAS_NUM_LIM_DEFAULT      (1)



/*
 * Idea from: https://github.com/epozzobon/esp32scope
 */
//static esp_err_t adc_i2s_scan_mode_init(const adc_i2s_pattern_t *adc_patterns, uint8_t num_patterns);
static esp_err_t adc_set_i2s_data_pattern(adc_unit_t adc_unit, uint8_t n, adc_i2s_pattern_t pattern);
static esp_err_t adc_set_i2s_data_len(adc_unit_t adc_unit, uint8_t len);

I2SADC::I2SADC(uint32_t rate, uint32_t buffer_len)
{
  // Setup sample rate
  mSampleRate = rate;

  // Set buffer len
  mBufferLen = buffer_len;

  // Setup I2S Configuration
  mI2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),  /* ADC will be sampled according to I2S */
    .sample_rate = mSampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,       /* ADC is 12-bits, nearest is 16-bits */
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       /* Only one channel will be employed // I2S_CHANNEL_FMT_ONLY_RIGHT */ 
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,  /* Standard I2S communication format // I2S_COMM_FORMAT_STAND_I2S */ 
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           /* Interrupt priority */
    .dma_buf_count = 10,                                 /* Four buffers of mBufferLen bytes */
    .dma_buf_len = mBufferLen,
    .use_apll = false,
    //.tx_desc_auto_clear = false,
    //.fixed_mclk = false
  };

  mNumChannels = 0;
}

esp_err_t I2SADC::begin(adc_i2s_pattern_t *patterns, uint8_t num_patterns)
{
  esp_err_t err = ESP_OK;

  // ADC I2S Config
  for(uint8_t n = 0; n < num_patterns; n++) {
    err += adc1_config_channel_atten((adc1_channel_t)patterns[n].channel, (adc_atten_t)patterns[n].atten);
  }  
  err += adc1_config_width((adc_bits_width_t)patterns[0].bit_width);
  
  // Install i2s driver
  err += i2s_driver_install(I2S_NUM_0, &mI2sConfig, 0, NULL);
  //err += i2s_set_clk(I2S_NUM_0, mSampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);  // ADC MONO
  
  // For internal ADC & DAC
  i2s_pin_config_t pin_config = {
      .bck_io_num = -1,
      .ws_io_num = GPIO_NUM_15,
      .data_out_num = -1,
      .data_in_num = -1                                               //Not used
  };
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Setup adc
  mNumChannels = num_patterns;
  err += i2s_set_adc_mode(ADC_UNIT_1, (adc1_channel_t)patterns[0].channel); //Channel doenst care, will be overridden
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV_M);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MEAS_NUM_LIMIT_V, 0, SYSCON_SARADC_MEAS_NUM_LIMIT_S);
  
  // ADC-I2S Scanning does not normal start (more badly) after the reset of ESP32. 
  // It is necessary to add a delay of 5 seconds before switching on the ADC for a 
  // reliable start, this is issue.
  vTaskDelay(5000/portTICK_RATE_MS);
    
  /* Enable ADC I2S */
  err += i2s_zero_dma_buffer(I2S_NUM_0);
  err += i2s_adc_enable(I2S_NUM_0);
  //Serial.printf("REG = 0x%08X\n", READ_PERI_REG(SYSCON_SARADC_CTRL_REG));
  
  // Setup ADC Channels on the patterns table
  if(num_patterns > 1)
  {
    // Invert data
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV_V, 1, SYSCON_SARADC_SAR1_INV_S);
    // For continuous adc sampling
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MEAS_NUM_LIMIT_V, 0, SYSCON_SARADC_MEAS_NUM_LIMIT_S);
    // Maximum number of measurements before restart ADC
    //uint32_t meas_value = (uint32_t)floorf(256.0F/(float)(mNumChannels*2))*(mNumChannels*2) - 1;
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MAX_MEAS_NUM_V, meas_value, SYSCON_SARADC_MAX_MEAS_NUM_S);
  
    // Set pattern table length: 0 - 15, where len = 0 is for 1 channel, 
    // len = 1 for 2 channels and so on
    err += adc_set_i2s_data_len(ADC_UNIT_1, num_patterns-1);
    
    // Add channels to patterns table
    for(uint8_t n = 0; n < num_patterns; n++){
      err += adc_set_i2s_data_pattern(ADC_UNIT_1, n, patterns[n]);
    }
  }
  
  // Characterize ADC
  esp_adc_cal_characteristics_t adc_char;
  esp_adc_cal_characterize(ADC_UNIT_1, (adc_atten_t)(patterns[0].atten), (adc_bits_width_t)(patterns[0].bit_width), 1100, &adc_char);
  
  return err;
}

void I2SADC::end(void) {
  // Disable I2S-ADC
  i2s_adc_disable(I2S_NUM_0);
  
  // Clear patterns table
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_P_CLEAR_V, 1, SYSCON_SARADC_SAR1_PATT_P_CLEAR_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_P_CLEAR_V, 0, SYSCON_SARADC_SAR1_PATT_P_CLEAR_S);
    
  // I2S driver close
  i2s_driver_uninstall(I2S_NUM_0);

  mNumChannels = 0;
}
    
uint32_t I2SADC::read(uint16_t *data, uint32_t len)
{
  if(len > mBufferLen)
  {
    return 0;
  }
  
  // I2S Read
  size_t bytesToRead = 0;
  i2s_read(I2S_NUM_0, (int8_t *)data, sizeof(uint16_t)*len, &bytesToRead, portMAX_DELAY);
  int32_t nRxSamples = bytesToRead / sizeof(uint16_t);
  
  return nRxSamples;
}

uint32_t I2SADC::length(void)
{
  return mBufferLen;
}

uint8_t I2SADC::Channels(void) {
  return mNumChannels;
}
I2SADC::~I2SADC()
{
  end();
}









static esp_err_t adc_set_i2s_data_pattern(adc_unit_t adc_unit, uint8_t n, adc_i2s_pattern_t pattern) {
  const uint8_t patt_tab_offset[4] = {24,16,8,0};
  //Configure pattern table, each 8 bit defines one channel
  //[7:4]-channel [3:2]-bit width [1:0]- attenuation
  //BIT WIDTH: 3: 12BIT  2: 11BIT  1: 10BIT  0: 9BIT
  //ATTEN: 3: ATTEN = 11dB 2: 6dB 1: 2.5dB 0: 0dB
  uint32_t patt_tab_reg = 0;
  if(adc_unit == ADC_UNIT_1)
  {
    patt_tab_reg=SYSCON_SARADC_SAR1_PATT_TAB1_REG + 4*(n/4);
  } else if (adc_unit == ADC_UNIT_2){
    patt_tab_reg = SYSCON_SARADC_SAR2_PATT_TAB1_REG + 4*(n/4);
  } else
  {
    return (esp_err_t)(-1);
  }
  
  uint32_t patt_tab_pos = patt_tab_offset[n % 4];
  uint32_t patt_tab_reg_val = (READ_PERI_REG(patt_tab_reg) & ~(0x000000FF << patt_tab_pos));
  patt_tab_reg_val |= ((uint32_t)(pattern.value) << patt_tab_pos);
  WRITE_PERI_REG(patt_tab_reg, patt_tab_reg_val);
  
  return ESP_OK;
}

static esp_err_t adc_set_i2s_data_len(adc_unit_t adc_unit, uint8_t len) {
  if(adc_unit == ADC_UNIT_1)
  {
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_P_CLEAR_V, 1, SYSCON_SARADC_SAR1_PATT_P_CLEAR_S);
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_P_CLEAR_V, 0, SYSCON_SARADC_SAR1_PATT_P_CLEAR_S);
    SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN_V, len, SYSCON_SARADC_SAR1_PATT_LEN_S);
  } else if(adc_unit == ADC_UNIT_2) {
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_PATT_P_CLEAR_V, 1, SYSCON_SARADC_SAR2_PATT_P_CLEAR_S);
    //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_PATT_P_CLEAR_V, 0, SYSCON_SARADC_SAR2_PATT_P_CLEAR_S);
    SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_PATT_LEN_V, len, SYSCON_SARADC_SAR2_PATT_LEN_S);
  } else
    return (esp_err_t)(-1);
  
  return ESP_OK;
}





/*static esp_err_t adc_i2s_scan_mode_init(const adc_i2s_pattern_t *adc_patterns, uint8_t num_patterns)
{
  esp_err_t err = ESP_OK;
  uint8_t table_len = num_patterns-1;
  
  //
  //adc_power_always_on();
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_V, SENS_FORCE_XPD_SAR_PU, SENS_FORCE_XPD_SAR_S);

  // Set data len for patter table
  //err = adc_set_i2s_data_len(adc_unit, table_len);
  //SYSCON.saradc_ctrl.sar1_patt_len = patt_len - 1;
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN_V, table_len, SYSCON_SARADC_SAR1_PATT_LEN_S);

  // Set pattern table
  for(int n = 0; n < num_patterns; n++)
  {
    //adc_gpio_init(adc_unit, channel[n]);
    adc1_config_channel_atten((adc1_channel_t)adc_patterns[n].channel, ADC_ATTEN_DB_11);
    
    //adc_set_i2s_data_pattern(adc_unit, n, channel[n], ADC_WIDTH_BIT_12, ADC_ATTEN_DB_11);
    uint32_t patt_tab_reg = SYSCON_SARADC_SAR1_PATT_TAB1_REG + 4*(n/4);
    uint32_t patt_tab_pos = patt_tab_offset[n % 4];
    uint32_t patt_tab_reg_val = (READ_PERI_REG(patt_tab_reg) & ~(0x000000FF << patt_tab_pos));
    patt_tab_reg_val |= ((uint32_t)(adc_patterns[n].value) << patt_tab_pos);
    WRITE_PERI_REG(patt_tab_reg, patt_tab_reg_val);
  }

  // Not found in library
  //adc_set_controller(ADC_UNIT_1, ADC_CTRL_DIG);
  // For ADC_UNIT_1 & ADC_CTRL_DIG
  //SENS.sar_read_ctrl.sar1_dig_force = true;
  //SENS.sar_meas_start1.meas1_start_force = true;
  //SENS.sar_meas_start1.sar1_en_pad_force = true;
  //SENS.sar_touch_ctrl1.xpd_hall_force = true;
  //SENS.sar_touch_ctrl1.hall_phase_force = true;
  //
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DIG_FORCE_V, 1, SENS_SAR1_DIG_FORCE_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE_V, 1, SENS_MEAS1_START_FORCE_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE_V, 1, SENS_SAR1_EN_PAD_FORCE_S);
  SET_PERI_REG_BITS(SENS_SAR_TOUCH_CTRL1_REG, SENS_XPD_HALL_FORCE_V, 1, SENS_XPD_HALL_FORCE_S);
  SET_PERI_REG_BITS(SENS_SAR_TOUCH_CTRL1_REG, SENS_HALL_PHASE_FORCE_V, 1, SENS_HALL_PHASE_FORCE_S);

  //
  adc_set_i2s_data_source(ADC_I2S_DATA_SRC_ADC);
  
  // ADC clock devided from APB clk, 80 / 2 = 40Mhz,
  //SYSCON.saradc_ctrl.sar_clk_div = clk_div;
  adc_set_clk_div(SAR_ADC_CLK_DIV_DEFAULT);
  
  // Set internal FSM wait time. (not found in library)
  //adc_set_fsm_time(rst_wait,start_wait,standby_wait,sample_cycle)
  //adc_set_fsm_time(ADC_FSM_RSTB_WAIT_DEFAULT, ADC_FSM_START_WAIT_DEFAULT, ADC_FSM_STANDBY_WAIT_DEFAULT, ADC_FSM_TIME_KEEP);
  // if(rst_wait>=0) SYSCON.saradc_fsm.rstb_wait = rst_wait;
  // if(start_wait>=0) SYSCON.saradc_fsm.start_wait = start_wait;
  // if(standby_wait>=0) SYSCON.saradc_fsm.standby_wait = standby_wait;
  // if(sample_cycle>=0) SYSCON.saradc_fsm.sample_cycle = sample_cycle;
  SET_PERI_REG_BITS(SYSCON_SARADC_FSM_REG, SYSCON_SARADC_RSTB_WAIT_V, ADC_FSM_RSTB_WAIT_DEFAULT, SYSCON_SARADC_RSTB_WAIT_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_FSM_REG, SYSCON_SARADC_START_WAIT_V, ADC_FSM_START_WAIT_DEFAULT, SYSCON_SARADC_START_WAIT_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_FSM_REG, SYSCON_SARADC_STANDBY_WAIT_V, ADC_FSM_STANDBY_WAIT_DEFAULT, SYSCON_SARADC_STANDBY_WAIT_S);
  
  //
  //adc_set_work_mode(adc_unit);
  // for (ADC_UNIT_1)
  //  saradc mode sel : 0--single saradc;  1--double saradc;  2--alternative saradc
  //  SYSCON.saradc_ctrl.work_mode = 0;
  //  ENABLE ADC  0: ADC1  1: ADC2, only work for single SAR mode
  //  SYSCON.saradc_ctrl.sar_sel = 0;
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_WORK_MODE_V, 0, SYSCON_SARADC_WORK_MODE_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR_SEL_V, 0, SYSCON_SARADC_SAR_SEL_S);

  //
  //adc_set_data_format(ADC_ENCODE_12BIT);
  //data format:
  //0: ADC_ENCODE_12BIT  [15:12]-channel [11:0]-12 bits ADC data
  //1: ADC_ENCODE_11BIT  [15]-1 [14:11]-channel [10:0]-11 bits ADC data, the resolution should not be larger than 11 bits in this case.
  //SYSCON.saradc_ctrl.data_sar_sel = mode;
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_DATA_SAR_SEL_V, ADC_ENCODE_12BIT, SYSCON_SARADC_DATA_SAR_SEL_S);

  //
  //adc_set_measure_limit(ADC_MAX_MEAS_NUM_DEFAULT, ADC_MEAS_NUM_LIM_DEFAULT);
  // Set max measure number
  //SYSCON.saradc_ctrl2.max_meas_num = meas_num;
  // Enable max measure number limit
  //SYSCON.saradc_ctrl2.meas_num_limit = lim_en;
  //SYSCON_SARADC_CTRL2_REG
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MAX_MEAS_NUM_V, ADC_MAX_MEAS_NUM_DEFAULT, SYSCON_SARADC_MAX_MEAS_NUM_S);
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_MEAS_NUM_LIMIT_V, 0, SYSCON_SARADC_MEAS_NUM_LIMIT_S);  // For continuous adc sampling

  //Invert The Level, Invert SAR ADC1 data (found in library)
  adc_set_data_inv(ADC_UNIT_1, false);
  
  return err;
}*/