# neo-6M

Python 3 class for neo-6M GPS

# required:

geopy library if you want to geolocalize your gps point

https://github.com/geopy/geopy

# use:

        from ublox_neo6m import GpsNeo6

        # define the serial port, baud rate and time difference of local time zone to UTC
        # if geoloc = True and geopy.geocoders is installed on the system,
        # address information is looked up for the specified coordinates
        
        gps=GpsNeo6(port="/dev/ttyS0",b_rate=9600,diff=2, geoloc=True)
        
        # read gps data
        gps.read()
        
        # print info
        print(gps)  
  


# results

        Time: 18:57:56.0
        latitude: 54.3078435 N
        Longitude: 9.534970833333333 E
        Velocity: 0.108 km/h
        Altitude: -2.0 metre(s)
        Geoid separation: 44.6 metre(s)
        Horizontal precision: 0.95 meter(s)
        Number of connected satellites: 10


