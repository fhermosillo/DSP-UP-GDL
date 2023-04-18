#include "soc/sens_reg.h"
#include "Cosgen.h"

typedef enum {
  CW_CLK_DIV_1 = 0,
  CW_CLK_DIV_2,
  CW_CLK_DIV_3,
  CW_CLK_DIV_4,
  CW_CLK_DIV_5,
  CW_CLK_DIV_6,
  CW_CLK_DIV_7,
  CW_CLK_DIV_8
} cw_clock_div_t;


/* Private Prototype Function */
static void cw_set_channel(dac_channel_t channel, bool en);
static void cw_set_clock(cw_clock_div_t clk_8m_div, uint16_t step);
static void cw_set_scale(dac_channel_t channel, cw_scale_t scale);
static void cw_set_offset(dac_channel_t channel, uint8_t offset);
static void cw_set_phase(dac_channel_t channel, cw_phase_t phase);

/* Class Reference */
CWDac::CWDac() {
  mChannel = DAC_CHANNEL_2;
  stop();
  mChannel = DAC_CHANNEL_1;
  stop();

  mFrequency = 0;
  mEnable = false;
}

CWDac::~CWDac() {
  stop();
}

bool CWDac::stop(void) {
  cw_set_channel(mChannel, false);
  dac_output_disable(mChannel);

  bool ret = mEnable;
  mEnable = false;

  return ret;
}

void CWDac::start(void) {
  cw_set_channel(mChannel, true);
  dac_output_enable(mChannel);
  mEnable = true;
}

void CWDac::setChannel(dac_channel_t channel) {
  mChannel = channel;
}

uint32_t CWDac::setFrequency(uint32_t freq) {

  // F = dig_clk_rtc_freq*SENS_SAR_SW_FSTEP/65536
  uint32_t step = 65537;
  uint8_t clk_div = 0;
  while (step > 65535 && clk_div < 8) {
    step = (uint32_t)((float)(freq * (clk_div + 1)) * (65536.0F / 8000000.0F));
    clk_div++;
  }

  if (clk_div < 8 && step < 65536) {
    mFrequency = (float)(8000000.0F / (65536.0F * clk_div)) * step;
    cw_set_clock((cw_clock_div_t)(clk_div - 1), (uint16_t)step);
    return mFrequency;
  }

  return 0;
}

void CWDac::setScale(cw_scale_t scale) {
  cw_set_scale(mChannel, scale);
}

void CWDac::setPhase(cw_phase_t phase) {
  cw_set_phase(mChannel, phase);
}

void CWDac::setOffset(uint8_t offset) {
  cw_set_offset(mChannel, offset);
}

void CWDac::setConfig(dac_channel_t channel, uint32_t freq, cw_scale_t scale, cw_phase_t phase, uint8_t offset) {
  mChannel = channel;
  bool en = stop();

  setFrequency(freq);
  setScale(scale);
  setPhase(phase);
  setOffset(offset);

  if (en) start();
}

/* ------------------------------------------------ */
/* 			Private Reference Function 				*/
/* ------------------------------------------------ */
void cw_set_channel(dac_channel_t dac_channel, bool en) {
  // Enable tone generator common to both channels
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN_V, en, SENS_SW_TONE_EN_S);

  switch (dac_channel) {
    case DAC_CHANNEL_1:
      // Enable / connect tone tone generator on / to this channel
      SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_V, en, SENS_DAC_CW_EN1_S);
      break;

    case DAC_CHANNEL_2:
      SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_V, en, SENS_DAC_CW_EN2_S);
      break;

    default:
      break;
  }
}

/*
 * Set frequency of internal CW generator common to both DAC channels
 *
 * clk_8m_div: range 0b00 - 0b111
 *		clock_out = CLOCK_8MHZ / (clk_8m_div + 1). (
 * frequency: range 0x0001 - 0xFFFF
 *
 */
void cw_set_clock(cw_clock_div_t clk_8m_div, uint16_t step) {
  SET_PERI_REG_BITS(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL_V, (uint8_t)clk_8m_div, RTC_CNTL_CK8M_DIV_SEL_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP_V, step, SENS_SW_FSTEP_S);
}


/*
 * Scale output of a DAC channel using two bit pattern:
 *
 * - 00: no scale
 * - 01: scale to 1/2
 * - 10: scale to 1/4
 * - 11: scale to 1/8
 *
 */
void cw_set_scale(dac_channel_t channel, cw_scale_t scale) {
  switch (channel) {
    case DAC_CHANNEL_1:
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1_V, (uint8_t)scale, SENS_DAC_SCALE1_S);
    break;

    case DAC_CHANNEL_2:
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2_V, (uint8_t)scale, SENS_DAC_SCALE2_S);
    break;

    default:
    break;
  }
}

/*
 * Offset output of a DAC channel
 *
 * Range 0x00 - 0xFF
 *
 */
void cw_set_offset(dac_channel_t channel, uint8_t offset) {
  switch (channel) {
    case DAC_CHANNEL_1:
      SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC1_V, offset, SENS_DAC_DC1_S);
      break;
    case DAC_CHANNEL_2:
      SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC2_V, offset, SENS_DAC_DC2_S);
      break;
    default:
    break;
  }
}

void cw_set_phase(dac_channel_t channel, cw_phase_t phase) {
  switch (channel) {
    case DAC_CHANNEL_1:
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1_V, (uint8_t)phase, SENS_DAC_INV1_S);
    break;
    case DAC_CHANNEL_2:
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2_V, (uint8_t)phase, SENS_DAC_INV2_S);
    break;
      
    default:
    break;
  }
}