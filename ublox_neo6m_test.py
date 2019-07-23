from ublox_neo6m import GpsNeo6

neo6 = GpsNeo6(port="/dev/ttyS0", geoloc=False)
neo6.sleep(10.234566)


