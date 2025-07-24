#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H


#include <cstdint>
#include <cmath>

template <uint32_t N>
class CubicSpline {
public:
  CubicSpline(float m_knots[N], float m_coeffs[4][N - 1]);
  float eval(float x);
  int knots_bin_search(float x);
  
private:
  float knots[N];
  float coeffs[4][N - 1];
  
};

template <uint32_t N>
CubicSpline<N>::CubicSpline(float m_knots[N], float m_coeffs[4][N - 1]) {
  for (int i = 0; i < N; i++) {
    knots[i] = m_knots[i];
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < N - 1; j++) {
      coeffs[i][j] = m_coeffs[i][j];
    }
  }
}

template <uint32_t N>
int CubicSpline<N>::knots_bin_search(float x) {

  if (x > knots[N - 1]) {
    // Return N if out of bounds upwards.
    return N - 1;
  } else if (x < knots[0]) {
    // Return -1 if out of bounds downwards.
    return -1;
  }
  
  int low = 0;
  int high = N - 1;
  int m;
  while (low <= high) {
    m = (low + (high - low)/2);


    if (high - low == 1 ) {
      return low;
    }
    
    if (x > knots[m] ) {
      low = m;
    } else if (x < knots[m]) {
      high = m;
    } else {
      return m;
    }
  }

  return -2;
  
}

template <uint32_t N>
float CubicSpline<N>::eval(float x) {
  int section = knots_bin_search(x);
  if (section == -1) {
    section = 0;
    x = knots[0];
  } else if (section >= N - 1) {
    section = N - 2;
    x =  knots[N - 1];
  }
  x = x - knots[section];
  return 
    coeffs[0][section] * pow(x, 3) +
    coeffs[1][section] * pow(x, 2) +
    coeffs[2][section] * x +
    coeffs[3][section];
}

#endif
