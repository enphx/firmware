#ifndef CONVFILTER_H
#define CONVFILTER_H


#include <cstdint>

// N is kernel size.
template <typename T, uint32_t N>
class ConvFilter {
public:
  ConvFilter(T kernel[N]);
  T eval(T input);
  void clearData();

private:
  uint32_t index = 0;
  T data[N];
  T kern[N];
  
};


template<typename T, uint32_t N>
ConvFilter<T, N>::ConvFilter(T kernel[N]) {
  for (int i = 0; i < N; i++) {
    kern[i] = kernel[i];
  }
  clearData();
}


template<typename T, uint32_t N>
T ConvFilter<T, N>::eval(T input) {
  // Write new data ring buffer style.
  data[index] = input;
  index = (index + 1) % N;
  

  // Convolve kernel with current data frame.
  T result = 0;
  for (int i = 0; i < N; i++) {
    result += kern[i] * data[(i + index) % N];
  }

  return result;
}

template<typename T, uint32_t N>
void ConvFilter<T, N>::clearData() {
  for (int i = 0; i < N; i++) {
    data[i] = 0;
  }
  index = 0;
}


#endif
