#ifndef FILTER_H
#define FILTER_H

#include "Arduino.h"

class Filter {
public:
  Filter(unsigned size);
  ~Filter();

  float filter(float newVal);
  float getFilteredVal();
  void reset(float fillVal = 0);

private:
  unsigned bufferSize;   // size of the input buffer
  float *buffer;         // buffer of input values
  unsigned curIndex = 0; // index in the input values buffer
  float filteredVal = 0; // filtered result
  float total = 0;       // running total of the values
};

#endif