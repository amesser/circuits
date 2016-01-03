
#include <iostream>
#include <cstdint>

#include "ecpp/Math/Complex.hpp"
#include "ecpp/Math/Fixedpoint.hpp"

#include "util/datatypes.hpp"
#include "algorithm/dft.hpp"
#include "algorithm/cordic.hpp"
#include "algorithm/math.hpp"

using namespace ecpp;
using namespace std;

using Platform::Algorithm::Cordic;
using Platform::Algorithm::CordicHypIndex;
using Platform::Algorithm::CMath;


int main()
{
  FixedPoint<int16_t,0x4000> a(0.9), b, c(0.9);

  cout << "Testwert      : " << a << endl;
  cout << "Testwert x 0  : " << (a * b) << endl;
  cout << "Testwert x 0.9: " << (a * c) << endl;

  for(int i = 0; i < 16; ++i)
    cout << "BitReverse " << i << ": " << reverseBits<4>(i) << endl; 
 
  typedef FixedPoint<int16_t,0x4000> float_type;

  typedef Complex<float_type> fft_type[128];

  fft_type work_cmath, work_cordic, work_table;

  for(int i = 0; i < 128; ++i)
  {
    work_cmath[i] = 
      (      sin(2 * M_PI * i / 8)  +
      0.5 * sin(2 * M_PI * i / 16) +
      0.3 * sin(2 * M_PI * i / 32) +
      0.1 * sin(2 * M_PI * i / 64) +
      0.1) / 2;

    work_table[i] = work_cordic[i] = work_cmath[i];

    cout << "INP " << i << ": " << work_cmath[i] << endl; 
  }

  double test;


  for(test = 0.0; test < 1.0; test +=0.01)
  {
      cout << test << ": cmath sqrt:  " << CMath::sqrt(test)
                   << "  cordic_sqrt: " << Cordic<>::sqrt(test)
                   << endl;
  }

  for(test = 0.0; test < Cordic<>::m_pi / 2; test +=0.11)
  {
      cout << test << ": cmath_sin:  " << CMath::sin(test)
                   << "  cordic_sin: " << Cordic<>::sin(test)
                   << endl;
  }

  for(test = 0.0; test < Cordic<>::m_pi / 2; test +=0.11)
  {
      cout << test << ": cmath_cos:  " << CMath::cos(test)
                   << "  cordic_cos: " << Cordic<>::cos(test)
                   << endl;
  }

  typedef Radix2DFT<float_type, 7> DFT_Type;

  /*  ldfs */
  auto dft_factors_cmath = DFT_Type::w<CMath>();
  auto dft_factors_cordic = DFT_Type::w<Cordic<> >();
  auto dft_factors_table = dft_factors_cordic.asArray();


  DFT_Type::decimation_in_f(work_cmath,  dft_factors_cmath);
  DFT_Type::decimation_in_f(work_cordic, dft_factors_cordic);
  DFT_Type::decimation_in_f(work_table, dft_factors_table);

  for(int i = 0; i < 128; ++i)
  {
    int index = reverseBits<7>(i);

    cout << "DFT " << i
         << ": CMath:  " << work_cmath[index]
         <<  " Cordic: " << work_cordic[index]
         <<  " Table:  " << work_table[index]
         <<  " ABS:    " << (2*Cordic<>::sqrt(((work_table[index] * work_table[index].conjugate()).real()))) << endl;
  }
  //for(int i = 0; i < 256; ++i)
  //  output[reverseBits<8>(i)] = input[i];
  return 0;
}
