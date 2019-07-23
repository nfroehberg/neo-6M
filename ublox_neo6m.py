#/usr/bin/python3
#-*- coding: utf-8 -*-

# Using the Ublox Neo-6M GPS module
# Reading GPS data from NMEA sequences
# changing configuration of the chip with UBX commands
# Nico Fröhberg 2019
# script for reading GPS data based on: https://github.com/linuxnico/neo-6M

import binascii
import serial
import sys

try:
    from ublox_neo6m_ubx import Neo6M_UBX
except:
    print("ublox_neo6m_ubx driver not found. You can still get location data from the chip but not change its configuration")

try:
    from geopy.geocoders import Nominatim
    geo=True
except:
    geo=False

# using ordered dictionaries for easier reading of printout in order (no benefit for referencing in code),
# in Python 3.6 and later, normal dictionaries are ordered
if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
    from collections import OrderedDict

NMEA_key = {
    'GPDTM': ['LLL', 'LSD','lat','NS','Lon','EW','alt','RRR','cs'],
    'GPGBS': ['hhmmss.ss','errlat','errlon','erralt','svid','prob','bias','stddev','cs'],
    'GPGGA': ['hhmmss.ss','Latitude','N','Longitude','E','FS','NoSV','HDOP','msl','uMsl',
            'Altref','uSep','DiffAge','DiffStation','cs'],
    'GPGST': ['hhmmss.ss','range_rms','std_major','std_minor','hdg','std_lat','std_long','std_alt','cs'],
    'GPGRS': ['hhmmss.ss','mode'],
    'GPRMC': ['hhmmss.ss','Status','Latitude','N','Longitude','E','Spd','Cog','date','mv','mvE','mode','cs'],
    'GPTHS': ['headt','mi','cs'],
    'GPTXT': ['xx','yy','zz','string','cs'],
    'GPVTG': ['cogt','T','cogm','M','sog','N','kph','K','mode','cs'],
    'GPZDA': ['hhmmss.ss','day','month','year','ltzh','ltzn','cs']
    }

# implement NMEA polling with GPQ
class GpsNeo6():
    """
    Class for the Neo 6M GPS chip
    """

    
    def __init__(self, port, b_rate=9600, diff=2, geoloc=geo):
        """
        initialize variables:
        port: serial port
        b_rate: baud rate
        diff: time difference of local time to UTC in hours
        """
        
        self.port=serial.Serial(port,b_rate)
        self.diff=diff
        self.data={} # all NMEA data put out by the chip will be stored here
        self.velocity=""
        self.latitude=""
        self.NS=""
        self.longitude=""
        self.EW=""
        self.time=""
        self.altitude=""
        self.precision=""
        self.satellite=""
        self.altref=""
        self.geoloc=Nominatim()
        self.fix=0
        self.ubx=Neo6M_UBX(port,b_rate)
        self.geo=geoloc
        
    def __del__(self):
        """
        close serial port on destruction of object
        """
        self.port.close()
        
    def __repr__(self):
        """
        display data
        """
        
        if self.fix:
            rep="Time: "+str(self.time)+"\nlatitude: "+str(self.latitude)+" "+self.NS\
                +"\nLongitude: "+str(self.longitude)+" "+self.EW\
                +"\nVelocity: "+str(self.velocity)+" km/h" \
                +"\nAltitude: "+str(self.altitude)+" metre(s)"\
                +"\nGeoid separation: "+str(self.altref)+" metre(s)"\
                +"\nHorizontal precision: "+str(self.precision)+" meter(s)" \
                +"\nNumber of connected satellites: "+str(self.satellite)
            if self.geo:
                rep+="\nLocation : "+self.geolocation()
            return rep
        else:
            rep='GPS not located, connected sattellites: ' + str(self.satellite)
            return rep
    
    
    def read(self):
        """
        read gps data, parse and store in data variables
        """
        
        self.data = self.readSerial()
        if not "GPGGA" in self.data.keys():
            print("Please activate GPGGA NMEA message output")
            return
        
        gpgga = self.data["GPGGA"]
        t = str(int(gpgga['hhmmss.ss'][0:2])+self.diff)+":"+gpgga['hhmmss.ss'][2:4]+":"\
            +str(float(gpgga['hhmmss.ss'][4:8])) # convert time to local time zone       
        self.time = t
        self.fix = int(gpgga['FS'])
        if self.fix != 0:
            self.latitude = self.degToDec(gpgga['Latitude']) # latitude decimal
            self.NS=gpgga['N'] # North/South Indicator
            self.longitude = self.degToDec(gpgga['Longitude']) # longitude decimal
            self.EW = gpgga['E'] # East/West Indicator
            self.satellite = int(gpgga['NoSV']) # number of connected satellites
            self.altitude = float(gpgga['msl']) # altitude
            self.precision = float(gpgga['HDOP']) # horitontal precision
            self.altref = float(gpgga['Altref']) # geoid separation
            
            if "GPVTG" in self.data.keys():
                self.velocity = float(self.data["GPVTG"]['kph']) # get velocity
            else:
                self.velocity = ""
                print('For velocity output, please activate GPVTG NMEA message output')
        else:
            print('No Satellite Fix')
    

    def readSerial(self):
        first_header = ''
        raw = {}
        while True:
            # split line to list
            line = self.port.readline()
            line = line.decode().strip().split(',')
            
            #split checksum from last element
            if '*' in line[-1]:
                cksm = line[-1][line[-1].index('*'):]
                line[-1] = line[-1][: line[-1].index('*')]
                line.append(cksm)
                
            if not ('$' in line[0]):
                continue
            header = line[0].strip('$')
            if not header in ['GPGSV', 'GPTXT'] and first_header == '':
                first_header = header
                raw[header] = line[1:]
                continue
            elif header == 'GPGSV' and first_header == '':
                continue
            
            if header == first_header:
                break
            
            if not header in ['GPGSV', 'GPTXT']:
                raw[header] = line[1:]
            else:
                if header in raw.keys():
                    raw[header].append(line[1:])
                else:
                    raw[header] = []
                    raw[header].append(line[1:])
        return self.parseNMEA(raw)

    def parseNMEA(self, raw):
        """
        select appropriate method to decode NMEA message
        """
        
        for message in raw:
            if not message in ['GPGSV', 'GPGLL', 'GPGRS', 'GPGSA', 'GPTXT', 'PUBX']:
                raw[message] = self.decodeNMEA(message, raw[message])
                
            elif message == 'GPTXT':
                for i in range(len(raw[message])):
                    raw[message][i] = self.decodeNMEA(message, raw[message][i])
                    
            elif message == 'GPGSV':
                raw[message] = self.decodeGPGSV(raw[message])
                    
            elif message == 'GPGLL':
                raw[message] = self.decodeGPGLL(raw[message])
                    
            elif message == 'GPGRS':
                raw[message] = self.decodeGPGRS(raw[message])
                    
            elif message == 'GPGSA':
                raw[message] = self.decodeGPGSA(raw[message])
                    
            elif message == 'PUBX':
                raw[message] = self.decodePUBX(raw[message])
                
        return raw

            
    def decodeNMEA(self, header, message):
        """
        decode standard NMEA message of fixed length
        """

        if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
            message_dic = OrderedDict()
        else:
            message_dic = {}
            
        for i in range(len(message)):
            message_dic[NMEA_key[header][i]] = message[i]
        return message_dic


    def decodeGPGSV(self, message):
        """
        decode GPGSV NMEA messages
        """
        
        message_out = []
        for i in range(len(message)):
            
            if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
                message_dic = OrderedDict()
            else:
                message_dic = {}

            # GSV messages are read as list, parsing one by one            
            message_i = message[i]

            # first three parameters are fixed
            beginning_msg = ['NoMsg','MsgNo','NoSv']
            for j in range(len(beginning_msg)):
                message_dic[beginning_msg[j]] = message_i[j]

            # repeated block for individual satellites (max 4 reported in one message)
            message_dic['satellites'] = []
            repeat = message_i[3:-1]
            
            for j in range(int(len(repeat)/4)):
                if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
                    satellite = OrderedDict()
                else:
                    satellite = {}
                    
                repeat_sat = repeat[0+j*4:4+j*4]
                satellite_ref = ['sv','elv','az','cno']
                #decoding each satellites parameters
                for k in range(len(satellite_ref)):
                    satellite[satellite_ref[k]] = repeat_sat[k]
                message_dic['satellites'].append(satellite)
                
            message_dic['cs'] = message_i[-1]
            message_out.append(message_dic)

        return message_out


    def decodeGPGLL(self, message):
        """
        decode GPGLL NMEA messages
        """
        
        if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
            message_dic = OrderedDict()
        else:
            message_dic = {}
            
        # fixed block
        beginning_msg = ['Latitude','N','Longitude','E','hhmmss.ss','valid']
        for i in range(len(beginning_msg)):
            message_dic[beginning_msg[i]] = message[i]

        # optional parameter
        if len(message) == 8:
            message_dic['Mode'] = message[6]
            message_dic['cs'] = message[7]
        else:
            message_dic['cs'] = message[6]

        return message_dic


    def decodeGPGRS(self, message):
        """
        decode GPGRS NMEA messages
        """
        
        if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
            message_dic = OrderedDict()
        else:
            message_dic = {}

        message_dic['hhmmss.ss'] = message[0]
        message_dic['mode'] = message[1]
        message_dic['residual'] = message[2:14]
        message_dic['cs'] = message[14]

        return message_dic


    def decodeGPGSA(self, message):
        """
        decode GPGSA NMEA messages
        """
        
        if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
            message_dic = OrderedDict()
        else:
            message_dic = {}

        message_dic['Smode'] = message[0]
        message_dic['FS'] = message[1]
        message_dic['sv'] = message[2:14]
        message_dic['PDOP'] = message[14]
        message_dic['HDOP'] = message[15]
        message_dic['VDOP'] = message[16]
        message_dic['cs'] = message[17]

        return message_dic


    def decodePUBX(self, message):
        """
        decode PUBX NMEA messages (ublox custom)
        """
        pass
    
    
    def degToDec(self,deg):
        """
        transform degrees to decimal
        """
        
        degrees=int(deg[0:deg.find(".")-2])
        minutes=float(deg[deg.find(".")-2:])/100
        decimal_degrees = degrees+(minutes*(100/60))
        return decimal_degrees
        
    
    def geolocation(self):
        """
        get geolocation from coordinates
        """
        
        if geo:
            try:                
                location = self.geoloc.reverse(str(self.latitude)+", "+str(self.longitude))
                return str(location)
            except:
                return "Address for coordinates not found"
        else:
            return "Geopy is not installed, cannot get address for coordinates."

    def sleep(self, seconds=0):
        """
        deactivate the chip for specified time (set to zero for indefinite time)
        baseline current in deactivated state is ~15 mA @3.3V
        """
        
        duration = int(seconds*1000)
        command = self.ubx.encode_ubx('RXM', 'PMREQ', 8, {'duration':duration, 'flags':self.ubx.ubx_flags('00000000000000000000000000000010')})
        self.ubx.send_ubx(command)
    
    
if __name__=="__main__":
    # define the serial port, baud rate and time difference of local time zone to UTC
    # if geoloc = True and geopy.geocoders is installed on the system,
    # address information is looked up for the specified coordinates
    gps=GpsNeo6(port="/dev/ttyS0",b_rate=9600,diff=2, geoloc=True)
    # read gps data
    gps.read()
    # print info
    print(gps)       
