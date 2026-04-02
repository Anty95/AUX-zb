[![Support me on Ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/Fabiancrg)



| Supported Targets | ESP32-C6 | ESP32-H2 |
| ----------------- |  -------- | -------- |

# H-Link Zigbee Bridge for Hitachi HVAC

[![License: GPL v3](https://img.shields.io/badge/Software-GPLv3-blue.svg)](./LICENSE)
[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/Hardware-CC%20BY--NC--SA%204.0-green.svg)](./LICENSE-hardware)


Zigbee bridge for controlling Hitachi HVAC units via H-Link protocol using ESP32-C6.

## Overview

This project creates a Zigbee router device that bridges H-Link capable Hitachi HVAC units to Zigbee networks (Zigbee2MQTT, Home Assistant ZHA, etc.). It's based on the ESP-Zigbee SDK and implements the H-Link serial protocol for communication with the AC unit.

## Hardware Requirements

- **ESP32-C6** microcontroller (e.g., XIAO ESP32-C6, ESP32-C6-DevKitC)
- **H-Link compatible Hitachi HVAC** unit
- **BSS138 MOSFET level shifters** (for 3.3V to 5V logic level conversion)
- Connection to H-Link port on the AC unit (typically via SPX-WFG adapter port or direct H-Link connection)

### PCB Design

This project can use the same PCB design as the ACW02-ZB project:
- **PCB Files**: [ACW02-ZB v1.1 PCB](https://github.com/Fabiancrg/acw02_zb/tree/master/PCB/acw02_zb-v1.1)
- **Features**: 
  - Seeedstudio XIAO ESP32-C6 compatible
  - MPS MP1584 buck converter (12V to 5V/3.3V)
  - BSS138 level shifter on GPIO1 and GPIO2 to conenct to H-Link port 1 and 2
  - Compact 53.5mm x 33mm form factor

**Note**: While the PCB includes footprint for TXB0102, using BSS138 MOSFET level shifters is recommended for more reliable UART communication.

## Pin Configuration

Default UART pins (configurable in `hlink_driver.h`):
- **TX**: GPIO 1 (via BSS138 MOSFET level shifter to 5V H-Link)
- **RX**: GPIO 2 (via BSS138 MOSFET level shifter from 5V H-Link)
- **GND**: Common ground with AC unit

**Note**: The H-Link port operates at 5V logic levels, so bidirectional level shifters (BSS138) are required to interface with the ESP32-C6's 3.3V logic.

## Zigbee Endpoints

### Endpoint 1: Thermostat (Main HVAC Control)
- **Cluster**: hvacThermostat (0x0201), hvacFanCtrl (0x0202)
- **Features**:
  - System mode (Off, Auto, Cool, Heat, Dry, Fan Only)
  - Target temperature (16-32°C)
  - Current temperature
  - Fan mode (Auto, Low, Medium, High, Quiet)
  - Running state (Idle, Heating, Cooling, Fan Only, Drying)

### Endpoint 2: Swing Mode
- **Cluster**: On/Off (0x0006)
- **Function**: Control air flow swing (Off, Vertical, Horizontal, Both)

### Endpoint 3: Remote Control Lock
- **Cluster**: On/Off (0x0006)
- **Function**: Lock/unlock the physical remote control

### Endpoint 4: Beeper
- **Cluster**: On/Off (0x0006)
- **Function**: Trigger beep sound on AC unit

### Endpoint 5: Leave Home (Away Preset)
- **Cluster**: On/Off (0x0006)
- **Function**: Enable/disable away preset (sets heating to 10°C)

### Endpoint 6: Air Filter Warning
- **Cluster**: On/Off (0x0006)
- **Function**: Binary sensor indicating filter needs cleaning (read-only)

## H-Link Protocol

The H-Link protocol is a UART-based serial protocol used by Hitachi HVAC systems:
- **Baud Rate**: 9600
- **Data Bits**: 8
- **Parity**: Odd
- **Stop Bits**: 1

### Protocol Format

**Read Request (MT):**
```
MT P=XXXX C=YYYY\r
```

**Write Request (ST):**
```
ST P=XXXX,DATA C=YYYY\r
```

**Response:**
```
OK P=DATA C=YYYY\r
```

Where:
- `XXXX` = Feature address (16-bit hex)
- `DATA` = Data bytes (hex string)
- `YYYY` = Checksum (0xFFFF - address_high - address_low - data_bytes)

### Supported Features

| Address | Feature | Access |
|---------|---------|--------|
| 0x0000 | Power State | R/W |
| 0x0001 | Mode | R/W |
| 0x0002 | Fan Mode | R/W |
| 0x0003 | Target Temperature | R/W |
| 0x0006 | Remote Control Lock | R/W |
| 0x0007 | Clean Filter Reset | W |
| 0x0014 | Swing Mode | R/W |
| 0x0100 | Current Indoor Temp | R |
| 0x0102 | Current Outdoor Temp | R |
| 0x0300 | Leave Home Write | W |
| 0x0301 | Activity Status | R |
| 0x0302 | Air Filter Warning | R |
| 0x0304 | Leave Home Read | R |
| 0x0800 | Beeper | W |
| 0x0900 | Model Name | R |

## Building and Flashing

### Prerequisites

1. Install ESP-IDF v5.1 or later
2. Set up ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

### Build

```bash
cd H-Link_zb
idf.py set-target esp32c6
idf.py build
```

### Flash

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port.

## Factory Reset

Hold the **BOOT button** for **5 seconds** to perform a factory reset and clear Zigbee network settings.

## Zigbee2MQTT Integration

The device will be automatically recognized as a thermostat once you add support in Zigbee2MQTT. See the separate converter file `hlink-zb.ts` for Zigbee2MQTT integration.

### Pairing

1. Power on the device
2. It will automatically enter pairing mode (LED indicator)
3. In Zigbee2MQTT, click "Permit Join"
4. Device should appear as "Custom devices (DiY) hlink-zb"

## Configuration

### UART Pins

Edit `main/hlink_driver.h`:
```c
#define HLINK_UART_TX_PIN        1    // GPIO1 (TX) - via BSS138 level shifter
#define HLINK_UART_RX_PIN        2    // GPIO2 (RX) - via BSS138 level shifter
```

**Important**: BSS138 MOSFET-based level shifters are used instead of TXB0102DCUR because the TXB0102's auto-direction sensing can interfere with UART communication.

### Status Update Interval

Edit `main/hlink_driver.h`:
```c
#define HLINK_STATUS_UPDATE_INTERVAL_MS 5000  // Poll every 5 seconds
```

## Troubleshooting

### Device Not Responding

1. Check UART connections (TX/RX/GND)
2. Verify H-Link port is accessible on AC unit
3. Check baud rate and parity settings
4. Monitor serial output: `idf.py monitor`

### Zigbee Connection Issues

1. Check Zigbee coordinator is in pairing mode
2. Perform factory reset (hold BOOT 5 seconds)
3. Check power supply (stable 3.3V/5V)
4. Review Zigbee logs in coordinator

## References

- [ESP-Zigbee SDK Documentation](https://docs.espressif.com/projects/esp-zigbee-sdk/)
- [ESPHome H-Link AC Component](https://github.com/lumixen/esphome-hlink-ac)
- [Hitachi H-Link Protocol](https://github.com/lumixen/esphome-hlink-ac/blob/main/README.md#h-link-protocol)
- [ACW02-ZB PCB Design](https://github.com/Fabiancrg/acw02_zb/tree/master/PCB/acw02_zb-v1.1)

## License

See LICENSE file for details.

## Credits

Based on:
- ESP-Zigbee SDK examples by Espressif
- ESPHome H-Link AC component by lumixen
- ACW02-ZB project for PCB design and hardware reference

---

# ⚠️ Disclaimer

This project is provided **as-is**, without any warranty or guarantee of fitness for a particular purpose. **Use at your own risk.** The authors and contributors are not responsible for any damage, malfunction, injury, or loss resulting from the use, modification, or installation of this hardware or software.

- This is an open-source, community-driven project intended for **educational and experimental use**.
- The hardware and firmware are **not certified** for commercial or safety-critical applications.
- **Always follow proper electrical safety procedures** when working with HVAC systems and mains power.
- Modifying or installing this project **may void your equipment warranty**.
- Ensure compliance with **local laws and regulations** regarding wireless devices and HVAC modifications.
- **Improper installation or configuration** may result in equipment damage or safety hazards.
- Always **disconnect power** before working on HVAC systems.
- The H-Link protocol implementation is based on **reverse engineering** and may not be complete or accurate for all Hitachi HVAC models.

**By using this project, you acknowledge and accept these terms and assume all risks.**
