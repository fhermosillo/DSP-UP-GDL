/*
 * 	@filename: WaveGenerator.h
 *
 *  @author: Fernando Hermosillo Reynoso
 * 	@email: fernando.hermosillo@hotmail.com
 *  @date: 15/02/2023
 */

#ifndef __WAVE_GENERATOR_H
#define __WAVE_GENERATOR_H

/* Exported includes --------------------------*/
#include <Arduino.h>
#include <cmath>

/* Exported defines ---------------------------*/

/* Exported macros ----------------------------*/

/* Exported constants -------------------------*/

/* Exported typedefs --------------------------*/
typedef enum{
  WAVEFORM_NONE = 0,
  WAVEFORM_SINE,
  WAVEFORM_COSINE,
  WAVEFORM_SAWTOOTH,
  WAVEFORM_TRIANGULAR,
  WAVEFORM_SQUARE,
  WAVEFORM_NOISE
} waveform_t;

/* Exported variables -------------------------*/

/* Exported functions -------------------------*/
String getWaveformName(waveform_t waveform);

/* Exported classes ---------------------------*/
class WaveGenertor {
  public:
    WaveGenertor(float Fs);
    
    void setWaveform(waveform_t wave);
    void setFrequency(float F);
    void setAmplitude(float A);
    
    float sample(uint32_t n);
    
    ~WaveGenertor();
    
  private:
    float mFs;
    float mF;
    uint32_t mN;
    uint32_t mNhalf;
    float mA;
    waveform_t mWaveform;
    float buf[1024];
};

#endif /* __WAVE_GENERATOR_H */