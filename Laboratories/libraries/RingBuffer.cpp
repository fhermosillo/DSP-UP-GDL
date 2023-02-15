#include "HardwareSerial.h"
#include "RingBuffer.h"

RingBuffer::RingBuffer(size_t sz) {
  mSize = sz;
  mBuffer = (float *)malloc(sizeof(float) * sz);
  mWrIdx = 0;
  mRdIdx = 0;
  mWrPtr = mBuffer;
  mRdPtr = mBuffer;
}

void RingBuffer::setTo(float val) {
  for (int i = 0; i < mSize; i++) {
    mBuffer[i] = val;
  }
}

void RingBuffer::reset(void) {
  mRdIdx = mWrIdx;
  mRdPtr = mWrPtr;
}

void RingBuffer::push(float din) {
  *mWrPtr = din;
  reset();

  // Update write pointer
  if (++mWrIdx >= mSize)
    mWrIdx = 0;
  mWrPtr = mBuffer + mWrIdx;
}

float RingBuffer::pop(void) {
  // Read data
  float dout = *mRdPtr;

  // Update pointer
  if (!mRdIdx) {
    mRdIdx = mSize - 1;
    mRdPtr = mBuffer + mRdIdx;
  } else {
    mRdIdx--;
    mRdPtr--;
  }

  return dout;
}

float RingBuffer::begin() {
  return mBuffer[mWrIdx];
}

float RingBuffer::end() {
  size_t m = (!mWrIdx ? mSize-1 : mWrIdx-1);
  return mBuffer[m];
}

void RingBuffer::print(void) {
  float val = 0.0;
  int l = mSize;
  Serial.print("CBuffer = [");
  while (l--) {
    Serial.print(pop());
    Serial.print(",");
  }
  Serial.print("]\n");
}


float &RingBuffer::operator[](uint32_t idx) {
  uint32_t m = mWrIdx  + idx;

  if (m >= mSize)
    m -= mSize;

  return mBuffer[m];  
}

RingBuffer::~RingBuffer() {
  if (mBuffer) {
    free(mBuffer);
  }
}