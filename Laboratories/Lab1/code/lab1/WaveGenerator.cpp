/* Private includes ------------------------------------*/
#include "WaveGenerator.h"

/* Private defines -------------------------------------*/

/* Private macros --------------------------------------*/

/* Private constants -----------------------------------*/

/* Private typedefs ------------------------------------*/

/* Private variables -----------------------------------*/

/* Exported variables ----------------------------------*/

/* Private functions -----------------------------------*/

/* Exported reference functions ------------------------*/
String getWaveformName(waveform_t waveform)
{
  switch (waveform) {
    case WAVEFORM_SINE:
      return String("SINE");
    break;

    case WAVEFORM_COSINE:
      return String("COSINE");
    break;

    case WAVEFORM_SAWTOOTH:
      return String("SAWTOOTH");
    break;

    case WAVEFORM_TRIANGULAR:
      return String("TRIANGULAR");
    break;

    case WAVEFORM_SQUARE:
      return String("SQUARE");
    break;

    case WAVEFORM_NOISE:
      return String("NOISE");
    break;

    default:
      return String("NONE");
    break;
  }
}


/* Private reference functions ------------------------*/


/* Exported class functions ------------------------*/
WaveGenertor::WaveGenertor(float Fs)
{
  mFs = Fs;
  mF = 1000.0F;
  mN = ceil(mFs/mF);
  mNhalf = floor((float)mN/2.0F);
  mA = 1.0F;
  mWaveform = WAVEFORM_NONE;
  
}

void WaveGenertor::setWaveform(waveform_t wave)
{
  int i;
  mWaveform = wave;
  switch(wave)
  {
    case WAVEFORM_SINE:
      for(i = 0; i < mN; i++)
      {
        buf[i] = sinf(2.0F * 3.1416F * mF/mFs * i);
      }
    break;

    case WAVEFORM_COSINE:
      for(i = 0; i < mN; i++)
      {
        buf[i] = cosf(2.0F * 3.1416F * mF/mFs * i);
      }
    break;

    case WAVEFORM_SAWTOOTH:
      for(i = 0; i < mN; i++)
      {
        buf[i] = ((2.0*mF)*i/mFs - 1.0F);
      }
    break;

    case WAVEFORM_TRIANGULAR:
      for(i = 0; i < mN; i++)
      {
        buf[i] = (i < mNhalf ? (4.0*mF)*i/mFs - 1.0 : -(4.0*mF)*i/mFs + 3.0);
      }
    break;

    case WAVEFORM_SQUARE:
      for(i = 0; i < mN; i++)
      {
        buf[i] = (i < mNhalf ? 1.0F : -1.0F);
      }
    break;

    case WAVEFORM_NOISE:
    default:
      for(i = 0; i < mN; i++)
      {
        buf[i] = 0.0F;
      }
    break;
  }
}

void WaveGenertor::setFrequency(float F)
{
  mF = F;
  mN = ceil(mFs/mF);
  mNhalf = floor((float)mN/2.0F);
}

void WaveGenertor::setAmplitude(float A)
{
  mA = abs(A);
}

float WaveGenertor::sample(uint32_t n) {
  if(mWaveform == WAVEFORM_NOISE)
    return mA*((float)random(32768)/32768.0F - 0.5F) * 2.0;
  else
    return mA*buf[n % mN];
}

WaveGenertor::~WaveGenertor() {
  
}