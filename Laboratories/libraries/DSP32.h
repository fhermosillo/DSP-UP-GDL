/*
 * ESP32 Digital Signal Processing library
 *
 * Copyright Fernando Hermosillo Reynoso 2023
 *
 * This software is distributed under the GNU Public Licence v2 (GPLv2)
 *
 * Please read the LICENCE file
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
 
#ifndef __DSP32LIB_H
#define __DSP32LIB_H

#include <iostream>
#include <list>
#include <cmath>

namespace dsp32
{
  /*
  */
  const float DSP_PI = 3.14159265358979323846F;
  const float DSP_TWOPI = 2.0F * DSP_PI;

  inline float rad2deg(float r)
  {
    return r*180.0F/DSP_PI; 
  }
  
  float msine(float x);

  
  /*inline float min(float x1, float x2)
  {
    return x2 < x1 ? x2 : x1;
  }

  inline float max(float x1, float x2)
  {
    return x2 > x1 ? x2 : x1;
  }*/
  
	/*
	 * @class: ProcessorNode
	 * @brief: Base class for every DSP algorithm
	 *
	*/
	class ProcessorNode {
	public:
	  ProcessorNode() {}

	  virtual bool begin(uint32_t Fs) {
      mSamplingRate = Fs;
		return true;
	  }
	  virtual float process(float sample) {
		return 0.0F;
	  }

	  ~ProcessorNode() {}

	protected:
		uint32_t mSamplingRate;
	};


	/*
	 * @class: DSP
	 * @brief: Main class of algorithms
	 *
	*/
	class Processor : public ProcessorNode {
	public:
	  Processor();

	  bool begin(uint32_t Fs);
	  float process(float sample);
	  void addNode(ProcessorNode *node);

	  ~Processor();

	private:
	  std::list<ProcessorNode *> nodes;
	};
}
#endif /* __DSP32LIB_H */