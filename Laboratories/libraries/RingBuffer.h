/*
 * Ring Buffer library
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

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <Arduino.h>
#include <stdlib.h>

class RingBuffer {
public:
  RingBuffer(size_t sz);
  void setTo(float val);
  void push(float din);
  float pop(void);
  void reset(void);
  inline size_t length(void) {
    return mSize;
  }
  void print(void);
  float begin(void);
  float end(void);

  float &operator[](size_t idx);

  ~RingBuffer();

private:
  size_t mSize;

  float *mBuffer;
  float *mWrPtr;
  float *mRdPtr;

  size_t mWrIdx;
  size_t mRdIdx;
};

#endif /* __RING_BUFFER_H */