# ModernGCS - Ground Control Station

A modern Ground Control Station for ArduPilot vehicles built with Python and PyQt6.

## Features

- **Multiple Connection Types**: Serial, TCP, UDP connections to vehicles
- **Real-time Flight Data**: Live telemetry display with status monitoring
- **Interactive Mapping**: ESRI-based mapping with waypoint management
- **Flight Instruments**: Attitude indicator, compass, and flight data displays
- **Vehicle Control**: Arm/disarm, takeoff, land, RTL, and mode changes
- **Mission Planning**: Create, edit, and upload waypoint missions

## Requirements

- Python 3.8 or higher
- PyQt6
- pymavlink
- Internet connection for map tiles

## Installation

1. Clone this repository:
```bash
git clone <repository_url>
cd modern-gcs
```

2. Install required packages:
```bash
pip install -r requirements.txt
```

## Usage

1. Run the application:
```bash
python main.py
```

2. Connect to your vehicle:
   - Click "File" > "Connect..." in the menu
   - Choose connection type (Serial, TCP, or UDP)
   - Configure connection parameters
   - Click "Connect"

3. Use the interface:
   - **Left Panel**: Vehicle control (arm/disarm, takeoff, etc.) and status
   - **Center Panel**: Interactive map with waypoint management
   - **Right Panel**: Flight instruments (attitude, compass)

## Connection Types

### Serial Connection
- Connect via USB or telemetry radio
- Select the correct COM port
- Set appropriate baud rate (usually 57600 or 115200)

### TCP Connection
- Connect to vehicle over network
- Enter IP address and port
- Useful for simulators (SITL)

### UDP Connection
- Connect via UDP protocol
- Enter target IP and port
- Common for wireless connections

## Map Usage

- **Add Waypoint**: Double-click on the map
- **Move Waypoint**: Drag waypoint markers
- **Remove Waypoint**: Right-click waypoint and select remove
- **Upload Mission**: Click "Upload Mission" button
- **Download Mission**: Click "Download Mission" button

## Vehicle Control

### Basic Operations
- **ARM**: Enable motors (vehicle must be ready)
- **DISARM**: Disable motors
- **TAKEOFF**: Take off to specified altitude
- **LAND**: Land at current position
- **RTL**: Return to launch position

### Flight Modes
Supported ArduCopter modes:
- STABILIZE: Manual flight with attitude stabilization
- ALT_HOLD: Manual flight with altitude hold
- LOITER: Hold position and altitude
- AUTO: Follow uploaded mission
- GUIDED: Accept external commands
- RTL: Return to launch
- LAND: Automatic landing

## Status Information

The status panel shows:
- Armed/Disarmed state
- Current flight mode
- Battery voltage, current, and percentage
- GPS fix status and satellite count
- Altitude and ground speed

## Flight Instruments

- **Attitude Indicator**: Shows roll and pitch angles
- **Compass**: Shows heading direction
- Real-time updates from vehicle telemetry

## Safety Notes

⚠️ **IMPORTANT SAFETY INFORMATION** ⚠️

- Always maintain visual line of sight with your vehicle
- Ensure sufficient battery levels before flight
- Check GPS fix before takeoff
- Have a manual override ready (RC transmitter)
- Follow local regulations and airspace restrictions
- Test in a safe, open area away from people and property

## Troubleshooting

### Connection Issues
- Check cable connections
- Verify correct COM port and baud rate
- Ensure vehicle is powered and telemetry is enabled
- Check firewall settings for TCP/UDP connections

### Map Issues
- Ensure internet connection for map tiles
- Clear browser cache if maps don't load
- Check proxy settings if behind corporate firewall

### Vehicle Control Issues
- Ensure vehicle is properly calibrated
- Check that all pre-arm checks pass
- Verify GPS lock before attempting to arm
- Check battery levels and voltage

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review ArduPilot documentation
3. Check connection and vehicle setup

## License

This project is open source and available under the MIT License.

## Acknowledgments

- ArduPilot project for the excellent autopilot software
- pymavlink for MAVLink communication
- ESRI for mapping services
- Qt/PyQt for the user interface framework
