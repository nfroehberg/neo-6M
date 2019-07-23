#/usr/bin/python3
#-*- coding: utf-8 -*-

# Using the Ublox Neo-6M GPS module
# Reading GPS data from NMEA sequences
# changing configuration of the chip with UBX commands
# Nico Fröhberg 2019
# script for reading GPS data based on: https://github.com/linuxnico/neo-6M

import binascii
import serial

try:
    from ublox_neo6m_ubx import Neo6M_UBX
except:
    print("ublox_neo6m_ubx driver not found. You can still get location data from the chip but not change its configuration")

try:
    from geopy.geocoders import Nominatim
    geo=True
except:
    geo=False


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
        self.tabCode=["GPVTG","GPGGA","GPGSA","GPGSV","GPGLL","GPRMC"]
        self.velocity=""
        self.latitude=""
        self.longitude=""
        self.latitudeDeg=""
        self.longitudeDeg=""
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
            rep="time: "+str(self.time)+"\nlatitude: "+str(self.latitude) \
                +"\nlongitude: "+str(self.longitude)+"\nvelocity: "+str(self.velocity)+" km/h" \
                +"\naltitude: "+str(self.altitude)+" metre(s)"+"\ngeoid separation: "+str(self.altref)+" metre(s)"+"\nhorizontal precision: "+str(self.precision)+" meter(s)" \
                +"\nNumber of connected satellites: "+str(self.satellite)
            if self.geo:
                rep+="\nlocation : "+self.geolocation()
            return rep
        else:
            rep='GPS not located, connected sattellites: ' + str(self.satellite)
            return rep
    
    
    
    def readSerial(self):
        """
        read data from serial port
        """
        
        l='->'
        line=""
        tab={}
        gp=[]
        # find starting point
        while True:
            line=self.port.readline().decode().strip()
            if 'GPRMC' in line:
                break
        # read all data
        while True:
            # split line to list
            line=self.port.readline()
            line = line.decode().strip().split(',')
            line[0]=line[0].strip('$')
            #print(line)
            # multiple GPSV lines joined in list other types separately
            for i in self.tabCode:
                if i=="GPGSV":
                    gp.append(line[1:])
                elif line[0] == i:
                    tab[i]=line[1:]
                else:
                    pass
            
            if line[0] == 'GPRMC':
                tab["GPGSV"]=gp
                break
        return tab
    
    def degToDec(self,deg):
        """
        transform degrees to decimal
        """
        
        degrees=int(deg[0:deg.find(".")-2])
        minutes=float(deg[deg.find(".")-2:])/100
        decimal_degrees = degrees+(minutes*(100/60))
        return decimal_degrees
    
    
    def read(self):
        """
        transform raw gps data
        """
        
        raw=self.readSerial()
        data=raw["GPGGA"]
        t=str(int(data[0][0:2])+self.diff)+":"+data[0][2:4]+":"+data[0][4:6] # convert time to local time zone       
        self.time=t
        self.fix = int(data[5])
        if self.fix != 0:
            self.latitude=self.degToDec(data[1]) # latitude decimal
            self.longitude=self.degToDec(data[3]) # longitude decimal
            self.satellite=int(data[6]) # number of connected satellites
            self.altitude=float(data[8]) # altitude
            self.precision=float(data[7]) # horitontal precision
            self.altref=float(data[10]) # geoid separation
            self.velocity=float(raw["GPVTG"][6]) # get velocity
        else:
            print('No Satellite Fix')
        
    
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
