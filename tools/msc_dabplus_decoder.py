#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import struct
from fic_decoder import crc_any, crc_ccitt
from reedsolo import RSCodec

if __name__ == "__main__":
    blub = open("/tmp/subchannel.aac", "wb")
    
    sync       = False
    superframe = ""
    cnt        = 0
    rs = RSCodec(10)
    
    with open("/tmp/subchannel.dat", "rb") as f:
        content = f.read()
        
    while content:
        dab_frame = content[0:360]
        content = content[360:]
        
        fire_code, = struct.unpack_from(">H", dab_frame)
        
        test = crc_any(dab_frame[2:11], 0x1782F, 16)
        
        if not sync and fire_code == test:
            sync = True
            
        if sync:
            superframe = superframe + dab_frame
            cnt = cnt + 1
            
            if (cnt % 5) == 0:
                dummy = ""
                
                # discard rs correction, we have subchannelindex 15
                rs_corrected = []
                
                for i in xrange(15):
                    row = bytearray(superframe[i::15])
                    try:
                        rs_corrected.append(str(rs.decode(row)))
                    except:
                        print("bububg")
                     
                if len(rs_corrected) == 15:
                    interleaved = "".join(rs_corrected)
                
                    superframe = "".join(interleaved[i::110] for i in xrange(110))
                    
                    
                    fire_code, prm = struct.unpack_from(">HB", superframe)
    
                    if fire_code != crc_any(superframe[2:11], 0x1782F, 16):
                        raise "wtf"
                    
                    
                    rfa      = 0 != (prm & 0x80)
                    dac_rate = 0 != (prm & 0x40)
                    sbr_flag = 0 != (prm & 0x20)
                    
                    num_aus = {
                     (False, True)   : 2,
                     (True, True)    : 3,
                     (False, False)  : 4,
                     (True, False)   : 6,
                     }[(dac_rate, sbr_flag)]
                     
                    print num_aus
                    
                    offsets = [3 + (num_aus * 12 - 8) / 8]
                    
                    x = 3
                    while len(offsets) < num_aus:
                        a,b,c = struct.unpack_from(">3B", superframe[x:])
                        x += 3
                        
                        offsets.append(((a << 4) + (b >> 4)) & 0xFFF)
                        offsets.append(((b << 8) + c)      & 0xFFF)
                        
                    offsets = offsets[0:num_aus] + [len(superframe)]
                    
                    for i in range(len(offsets)-1):
                        au = superframe[offsets[i]:offsets[i+1]]
                        
                        crc, = struct.unpack(">H",au[-2:])
                        au  = au[:-2]
                        
                        print hex(crc_ccitt(au, crc=0xFFFF)), hex(crc ^ 0xFFFF)
                        
                        blub.write(au)
                    
                    print (offsets)
                
                superframe = ""
        
                
        
        