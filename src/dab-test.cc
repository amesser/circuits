/*
 * dab-test.c
 *
 *  Created on: 13.01.2015
 *      Author: andi
 */
#include <iostream>
#include <iomanip>

#include "util/datatypes.hpp"
#include "algorithm/dft.hpp"
#include "algorithm/cordic.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>


using namespace ::std;
using namespace Platform::Algorithm::DFT;
using namespace Platform::Algorithm;

using namespace Platform::Util::Datatypes;


typedef FixedPoint<int16_t, (0x1 << 14)> DFT_BaseType;

//typedef double DFT_BaseType;
typedef Radix2DFT<DFT_BaseType, 11>              DFT_Type;

typedef DFT_Type::t_Type DFT_Buffer[DFT_Type::m_bins];

const DFT_Type::FactorArrayType          s_DftTwiddleFactors  = DFT_Type::w<Cordic<> >().asArray();
const DFT_Type::DescrambleArrayTypeFull  s_DftDescramble      = DFT_Type::descramble().asArrayFull();


int main(int argc, char *argv[])
{
  DFT_Buffer BufferI, BufferO;
  uint8_t    abIBuffer[DFT_Type::m_bins][2];

  int fdIn, fdOut;
  int limit = 0;

  fdIn = open(argv[1], O_RDONLY);
  fdOut = open(argv[2], O_WRONLY | O_TRUNC | O_CREAT);

  if (!fdIn or !fdOut)
    exit(-1);

  /*
  for(unsigned int i = 0; i < DFT_Type::m_bins; ++i)
  {
    cout << setw(4) << setfill('0') << hex  << i << " " \
         << setw(4) << setfill('0') << s_DftDescramble[i] << endl;
  }
  */

  const ssize_t ReadLen = sizeof(abIBuffer);
  while(ReadLen == read(fdIn, abIBuffer, ReadLen) /* && ((limit++) < 1024) */)
  {
    unsigned int i;

    for(i = 0; i < DFT_Type::m_bins; ++i)
    {
      int16_t iReal = (int16_t)abIBuffer[i][0] - 128;
      int16_t iImag = (int16_t)abIBuffer[i][1] - 128;


      const int scale = 16;

      /*
      if (iReal >= scale || iReal < -scale || iImag >= scale || iImag < -scale)
        cout << dec << "iReal: " << iReal << " iImag: " << iImag << endl;
      */

      if (iReal >= scale)
        iReal = scale - 1;
      else if (iReal < -scale)
        iReal = -scale;

      if (iImag >= scale)
        iImag = scale - 1;
      else if (iImag < -scale)
        iImag = -scale;

      const DFT_BaseType real(iReal, scale);
      const DFT_BaseType imag(iImag, scale);

      /* const DFT_BaseType real = iReal / 16.;
         const DFT_BaseType imag = iImag / 16.; */


      BufferI[i] = {real, imag};
      /* cout << dec << iReal << "::::" << real << endl; */
    }

    DFT_Type::decimation_in_f(BufferI, s_DftTwiddleFactors);

    for(unsigned int i = 0; i < DFT_Type::m_bins; ++i)
    {
      unsigned int reversed = s_DftDescramble[i];

      BufferO[reversed] = BufferI[i];
    }

    if((limit++) < 32)
    {
      for(i = 0; i < DFT_Type::m_bins/2; ++i)
      {
        char abOBuffer[256];
        snprintf(abOBuffer, sizeof(abOBuffer), "%5u %6e %6e\n", i, (float)BufferO[i + DFT_Type::m_bins/2].real(), (float)BufferO[i+ DFT_Type::m_bins/2].imag());

        write(fdOut, abOBuffer, strlen(abOBuffer));
      }

      for(;i < DFT_Type::m_bins; ++i)
      {
        char abOBuffer[256];
        snprintf(abOBuffer, sizeof(abOBuffer), "%5u %6e %6e\n", i, (float)BufferO[i - DFT_Type::m_bins/2].real(), (float)BufferO[i - DFT_Type::m_bins/2].imag());

        write(fdOut, abOBuffer, strlen(abOBuffer));
      }

      write(fdOut, "\n", 1);
    }

  }

  close(fdIn);
  close(fdOut);

  return 0;
}
