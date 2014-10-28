#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

crc_table = (\
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
    0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
    0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
    0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
    0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
    0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
    0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
    0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
    0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
    0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
    0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
    0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
    0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
    0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
)

def crc_ccitt(data, crc = 0):
    global crc_table

    for x in data:
        if isinstance(x, str):
            x = ord(x)
        
        i = (x ^ (crc >> 8)) & 0xff
        crc = (crc_table[i] ^ (crc << 8)) & 0xFFFF

    return crc

def crc_any(data, poly, bits, crc = 0):
    mask_crc = 0x1 << (bits - 1)
    
    for x in data:
        if isinstance(x, str):
            x = ord(x)
        
        mask = 0x80
        
        while(mask):
            input = 1 if (mask & x) else 0
            out   = 1 if (crc & mask_crc) else 0
            
            crc = crc << 1
            
            if input != out:
                crc = crc ^ poly

            mask = mask >> 1
            
    return crc & ((0x1 << bits) - 1)

from struct import unpack, unpack_from

class create_object_if_not_exists(object):
    def __init__(self, cache, create_func):
        self.cache       = cache
        self.create_func = create_func
        
    def __getitem__(self, key):
        try:
            item = self.cache[key]
        except KeyError:
            item = self.create_func(key)
            
        return item
    
class dab_service_org(object):
    def __init__(self, tmid_and_xid):
        self.tmid, self.xid = tmid_and_xid
        
    @property
    def id(self):
        return self.tmid, self.xid
            
class dab_service(object):
    def __init__(self, sref):
        self.sref = sref
        self.org  = []
        self._org_by_id_cache = None
        
    def createOrg(self, tmid_and_xid):
        org = dab_service_org(tmid_and_xid)
        self.org.append(org)
        
        # force rebuild of cache
        self._org_by_id_cache = None
        return org

    @property
    def organization_by_id(self):
        if not self._org_by_id_cache:
            org_by_id = dict( (x.id,x) for x in self.org)
            self._org_by_id_cache = org_by_id
        else:
            org_by_id = self._org_by_id_cache
            
        return create_object_if_not_exists(org_by_id, self.createOrg)
    
class dab_subchannel(object):
    fec_scheme = None
    
    def __init__(self, id):
        self.id = id

class dab_database(object):
        
    def __init__(self):
        self.subchannels = []
        self.services    = []
        
        self._subchannels_by_id_cache = None
        self._services_by_sref_cache = None

    def createSubchannel(self, subchid):
        subchannel = dab_subchannel(subchid)
        self.subchannels.append(subchannel)
        
        # force rebuild of cache
        self._subchannels_by_id_cache = None
        return subchannel

    def createService(self, sref):
        service = dab_service(sref)
        self.services.append(service)
        
        # force rebuild of cache
        self._services_by_sref_cache = None
        return service

    @property
    def subchannel_by_id(self):
        if not self._subchannels_by_id_cache:
            subchannels_by_id = dict( (x.id,x) for x in self.subchannels)
            self._subchannels_by_id_cache = subchannels_by_id
        else:
            subchannels_by_id = self._subchannels_by_id_cache
            
        return subchannels_by_id
    
    @property
    def create_subchannel_by_id(self):
        return create_object_if_not_exists(self.subchannel_by_id, self.createSubchannel)

    @property
    def service_by_sref(self):
        if not self._services_by_sref_cache:
            services_by_sref = dict( (x.sref,x) for x in self.services)
            self._services_by_sref_cache = services_by_sref
        else:
            services_by_sref = self._services_by_sref_cache
            
        return create_object_if_not_exists(services_by_sref, self.createService)
    
    @property
    def audio_channel_to_decode(self):
        if getattr(self,"audio_subchannel_id", None) is not None:
            return self.subchannel_by_id.get(self.audio_subchannel_id, None)
        else:
            return None
            

class fic_decoder(object):
    
    def __init__(self,db):
        super(fic_decoder,self).__init__()
        self.db = db

    def decode_fig0(self, fig_data):
        extension   = ord(fig_data[0])
        type0_field = fig_data[1:]
        
        cn_flag = (0 != (extension & 0x80))
        oe_flag = (0 != (extension & 0x40))
        pd_flag = (0 != (extension & 0x20))
        
        extension = extension & 0x1F

        decode_fig0 = getattr(self,"decode_fig0_extension" + str(extension), None)
        
        if(decode_fig0):
            decode_fig0(cn_flag, oe_flag, pd_flag, type0_field)
        else:
            print("No decoder for fig 0 extension {0}".format(extension))
    
    def decode_fig0_extension1(self,cn_flag, oe_flag, pd_flag, type0_field):
        while type0_field:
            x,y = unpack_from(">HB", type0_field)
            
            subchannelid  = x >> 10
            
            subchannel = self.db.create_subchannel_by_id[subchannelid]
            subchannel.startaddress  = x & 0x3FF
            
            if y & 0x80:
                x, = unpack_from(">H",type0_field[2:])
                x &= 0x7FFF
                
                subchannel.size             = x & 0x3FF
                subchannel.option           = x >> 13
                subchannel.protection_level = (x & 0x1800) >> 10
                
                type0_field = type0_field[4:]
            else:
                subchannel.tableindex  = y & 0x3F
                subchannel.tableswitch = (0 != (y & 0x40))
                 
                type0_field = type0_field[3:]

    def decode_fig0_extension2(self,cn_flag, oe_flag, pd_flag, type0_field):
        while type0_field:
            if pd_flag:
                sid, x = unpack_from(">LB", type0_field)
                
                sref              =  sid & 0x000FFFFF
                countryid         = (sid >> 20) & 0x0F
                extendedcountryid = (sid >> 24)
                
                type0_field = type0_field[5:]
            else:
                sid, x = unpack_from(">HB", type0_field)
                
                sref              =  sid & 0x0FFF
                countryid         = (sid >> 12)
                extendedcountryid = None
                
                type0_field = type0_field[3:]
                
            service = self.db.service_by_sref[sref]
            
            service.countryid         = countryid
            service.extendedcountryid = extendedcountryid
            service.local_flag = 0 != (x & 0x80)
            service.caid       = (x >> 4) & 0x7
            
            numberofcomponents = x & 0x0F
            
            orgfields = unpack_from(">" + ("H" * numberofcomponents), type0_field);
            type0_field = type0_field[2*numberofcomponents:]
            
            for x in orgfields:
                tmid = x >> 14
                
                if tmid < 3:
                    ty  = (x >> 8) & 0x3F
                    id = (x >> 2) & 0x3F
                elif tmid == 3:
                    id = (x >> 2) & 0x0FFF
                    ty = None
                    
                org = service.organization_by_id[(tmid, id)]
                org.ps_flag = 0 != (x & 0x02)
                org.ca_flag = 0 != (x & 0x01)
                
                if ty is not None:
                    org.ty = ty

    def decode_fig0_extension14(self,cn_flag, oe_flag, pd_flag, type0_field):
        for x in type0_field:
            x = ord(x)
            
            subchannelid  = x >> 2
                        
            subchannel = self.db.create_subchannel_by_id[subchannelid]
            subchannel.fec_scheme = x & 0x03
            


    def decode_fig1(self, fig_data):
        extension   = ord(fig_data[0])
        type1_field = fig_data[1:]
        
        charset = extension >> 4
        oe_flag = (0 != (extension & 0x08))
        
        extension = extension & 0x07
        
        character_field  = fig_data[-18:-2]
        character_flag   = unpack(">H", fig_data[-2:])
        identifier_field = fig_data[:-18]

        decode_fig1 = getattr(self,"decode_fig1_extension" + str(extension), None)
        
        if(decode_fig1):
            decode_fig1(charset, oe_flag, identifier_field, character_field, character_flag)
        else:
            print("No decoder for fig 1 extension {0}".format(extension))

    def decode(self, fic):
        for i in xrange(0,len(fic),32):
            fib = fic[i:i+32]
            
            fib_data, fib_crc = unpack(">30sH", fib)

            crc = crc_ccitt(fib_data, 0xffff) ^ 0xFFFF
        
            if fib_crc == crc:
                while fib_data:
                    fig_header = ord(fib_data[0])
                    
                    if fig_header == 0xFF:
                        # end token
                        break
                    else:
                        fig_type   = fig_header >> 5
                        fig_length = fig_header & 0x1F
                        fig_data = fib_data[1:1+ fig_length]

                        fib_data = fib_data[1+fig_length:]
                        
                        decode_fig = getattr(self,"decode_fig" + str(fig_type), None)
                        
                        if(decode_fig):
                            decode_fig(fig_data)
                        else:
                            print("No decoder for fig type {0}".format(fig_type))
                
            
        
if __name__ == "__main__":
    with open("../test/fic.dat", "rb") as f:
        fic_data = f.read()
    
    db = dab_database()
    
    decoder = fic_decoder(db)
    
    while len(fic_data) > 96:
        fic = fic_data[0:96]
        fic_data = fic_data[96:]
        
        decoder.decode(fic)
        
    for x in db.subchannels:
        print("subchannel ", x.id, x.startaddress, x.size, x.protection_level, x.option, x.fec_scheme)
    
    for x in db.services:
        print ("service ",x.sref)
        
        for y in x.org:
            print("org ",y.tmid, y.xid)
    
    