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

                time: 10:45:04
                latitude: 54.30782966666666
                longitude: 9.534877
                velocity: 0.031 km/h
                altitude: 28.1 metre(s)
                geoid separation: 44.6 metre(s)
                horizontal precision: 0.95 meter(s)
                Number of connected satellites: 9

