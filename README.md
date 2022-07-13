# MinUDP

Extracts data from multi-agent mavlink streams and forwards it via any user written protocol on UDP.

Works alongside the ardupilot_sitl_docker by default.

Requires Python 3.

To run with simulated drones, use:

**python .\minUDP.py -s**

To run with a connection to real drones, use:

**python .\minUDP.py -r**

Note this is a work in progress!
