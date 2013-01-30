/*
 *  Copyright 2013 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of SDCard Data Logger firmware.
 *
 *  This software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *  */
#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <stdint.h>

namespace Interface {
  enum {
    BLOCK_TYPE_INVALID = 0,
    BLOCK_TYPE_CONFIG  = 1,
    BLOCK_TYPE_DATA    = 2,
  };

  enum {
    CHANNEL_AD0 = 0,
    CHANNEL_AD1,
    CHANNEL_AD2,
    CHANNEL_AD3,
    CHANNEL_AD0_AD1,
    CHANNEL_AD1_AD0,
    CHANNEL_AD2_AD3,
    CHANNEL_AD3_AD2,
    CHANNEL_DI,
    CHANNEL_MAX
  };


  class BlockHead {
  private:
    uint8_t  _Type;
    uint8_t  _Len;

  public:
    uint8_t getType() const {return _Type;}
    uint8_t getLen()  const {return _Len;}

    template<typename STOR> void write(STOR &ctx) const {ctx.write(*this);}
    template<typename STOR> int8_t read (STOR &ctx)     {return ctx.read (*this);}
  } __attribute__((packed));


  class BlockEnd {
  private:
    uint8_t _Checksum;

  public:
    template<typename STOR> void write(STOR &ctx) const {ctx.write(*this);}
    template<typename STOR> int8_t read (STOR &ctx)     {return ctx.read (*this);}
  } __attribute__((packed));

  class BlockConfigA {
  private:
    uint16_t _MeasureInterval;
  public:
    uint16_t getMeasureInterval() const {return _MeasureInterval;}

    template<typename STOR> void write(STOR &ctx) const {ctx.write(*this);}
    template<typename STOR> int8_t read (STOR &ctx)     {return ctx.read (*this);}
  } __attribute__((packed));

  class BlockConfigB {
  private:
    uint8_t  _Channel;
  public:
    uint8_t getChannel() const { return _Channel;}

    template<typename STOR> void write(STOR &ctx) const {ctx.write(*this);}
    template<typename STOR> int8_t read (STOR &ctx)     {return ctx.read (*this);}
  } __attribute__((packed));

  class BlockDataA {
  private:
    uint32_t _Timestamp;
  public:
    template<typename STOR> void write(STOR &ctx) const {ctx.write(*this);}
    template<typename STOR> int8_t read (STOR &ctx)     {return ctx.read (*this);}
  } __attribute__((packed));

  class BlockDataB {
  private:
    uint16_t _Data;
  public:
    template<typename STOR> void write(STOR &ctx) const {ctx.write(*this);}
    template<typename STOR> int8_t read (STOR &ctx)     {return ctx.read (*this);}
  } __attribute__((packed));

};

#endif /* INTERFACE_H_ */
