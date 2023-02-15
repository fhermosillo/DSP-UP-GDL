#include <cmath>
#include "DSP32.h"

namespace dsp32 {
const unsigned int ciSine16LUT[] = {
  0, 1144, 2287, 3430, 4571, 5712, 6850, 7987, 9121, 10252, 11380,
  12505, 13625, 14742, 15854, 16962, 18064, 19161, 20251, 21336, 22414, 23486,
  24550, 25607, 26655, 27696, 28729, 29752, 30767, 31772, 32768,
  33753, 34728, 35693, 36647, 37589, 38521, 39440, 40347, 41243, 42125, 42995,
  43851, 44695, 45524, 46340, 47142, 47929, 48702, 49460, 50203, 50930, 51642,
  52339, 53019, 53683, 54331, 54962, 55577, 56174, 56755,
  57318, 57864, 58392, 58902, 59395, 59869, 60325, 60763, 61182, 61583, 61965,
  62327, 62671, 62996, 63302, 63588, 63855, 64103, 64331, 64539, 64728, 64897,
  65047, 65176, 65286, 65375, 65445, 65495, 65525, 65535
};


// Sine Look Up Table (LUT) function
float sin_lut(float x) {
  bool isPosEdge = true;  // positive
  if (x < 0) {
    x = -x;
    isPosEdge = false;
  }

  // Get index
  long ulX = x;
  unsigned int r = (x - ulX) * 256;
  if (ulX >= 360) ulX %= 360;
  if (ulX >= 180) {
    ulX -= 180;
    isPosEdge = !isPosEdge;
  }
  if (ulX >= 90) {
    ulX = 180 - ulX;
    if (r != 0) {
      r = 256 - r;
      ulX--;
    }
  }
  
  // Read value from LUT
  unsigned int v = ciSine16LUT[ulX];

  // interpolate if needed
  if (r > 0) v = v + ((ciSine16LUT[ulX + 1] - v) * 0.125F * r) * 0.03125F;  // v = v + ((isinTable16[ulX + 1] - v)/8.0F * r) /32.0F;

  return (isPosEdge ? v * 0.0000152590219F : v * -0.0000152590219); // = /65535.0
}




double fastATan(double x)
{
  //return M_PI_4*x - x*(std::fabs(x) - 1)*(0.2447 + 0.0663*std::fabs(x));
  return x*((M_PI_4 + 0.2447) - x*((0.2447 - 0.0663) + 0.0663*x));
}









bool Processor::begin(uint32_t Fs) {
  nodes.clear();
  mSamplingRate = Fs;
  return true;
}

float Processor::process(float sample) {
  float y = sample;

  // process all nodes
  for (std::list<ProcessorNode *>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
    ProcessorNode *node = *it;
    y = node->process(y);
  }

  return y;
}

void Processor::addNode(ProcessorNode *node) {
  nodes.push_back(node);
}
}