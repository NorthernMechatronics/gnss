# NM180100EVB Interface with the u-blox ZED-F9P High Precision GNSS Module

This application demonstrates how to retrieve GNSS coordinates from the ZED-F9P over
UART1 on the NM180100EVB and it is based on the NMAPP application template.

# Hardware Requirement and Setup

* A NM180100EVB
* A u-blox C099-F9P application board or equivalent with the UART exposed

Pin connections between the NM180100EVB and the C099-F9P

| NM180100EVB | C099-F9P |
| - | - |
| J705 Pin 1, GPIO 25 (UART RX) | J9 Pin 2, TXD_ZED (UART TX) |  
| J701 Pin 7, GND | J2 Pin 7, GND |

# Software Requirement

This example utilizes the NMSDK.  Please follow the SDK build instructions.

# Description

This application runs on the NM180100EVB and is used in conjunction with the 
u-blox C099-F9P application board containing the F9 high precisioin GNSS module.

This example implements a buffered serial port on UART1 that communicates with
the F9P at a baudrate of 921600.  The example looks for the NMEA standard message
"GLL" containing the latitude, longitude, time of position fix and fix status.
The message is then displayed over the serial command console on UART0.

# Build Instructions

There are two build configurations: one for debug and another for release.  The
configuration to be build is defined by the variable DEBUG.  When DEBUG is defined,
the debug configuration is selected and if it is left undefined, then the release
configuration is selected.  The output target will be located in either the debug or
the release directory.

## Debug Configuration
* make DEBUG=1
* make DEBUG=1 clean

## Release Configuration
* make
* make clean
