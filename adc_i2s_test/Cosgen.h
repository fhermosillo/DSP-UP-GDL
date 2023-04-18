#ifndef __DAC_COSINE_H
#define __DAC_COSINE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <soc/rtc_io_reg.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/sens_reg.h>
#include <soc/rtc.h>

#include <driver/dac.h>

typedef enum {
	CW_PHASE_0=0,
	CW_PHASE_90,
	CW_PHASE_180,
	CW_PHASE_270
} cw_phase_t;

typedef enum {
	CW_SCALE_DIV1=0,
	CW_SCALE_DIV2,
	CW_SCALE_DIV4,
	CW_SCALE_DIV8
} cw_scale_t;

class CWDac {
    public:
      CWDac();
          
      void start(void);
      bool stop(void);
      
      void setChannel(dac_channel_t channel);
      uint32_t setFrequency(uint32_t freq);
      void setScale(cw_scale_t scale);
      void setPhase(cw_phase_t phase);
      void setOffset(uint8_t offset);
      void setConfig(dac_channel_t channel, uint32_t freq, cw_scale_t scale, cw_phase_t phase, uint8_t offset);
		
		  ~CWDac();
    private:
		dac_channel_t mChannel;
		uint32_t mFrequency;
		bool mEnable;
};

#endif  /* __DAC_COSINE_H */
