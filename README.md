# SalmonSpotter

## Project Overview
**Primary Goal:**  
The SalmonSpotter project aims to build a durable device capable of withstanding the harsh weather conditions of an Alaskan lake. It collects data to identify prime areas for salmon spawning, ensuring battery efficiency, wireless data transmission, and cost-effectiveness.

**Problem Solved:**  
The device gathers data to help scientists understand why salmon choose specific spawning areas.

## Features
- **Sensors:** TDS, IMU, pH, water and air temperature sensors.
- **Data Transmission:** Uses Zolertia RE-Mote to send data wirelessly (802.15.4) to a land-based node.
- **Modularity:** Sensors are connected via a custom daughter-board on an I2C bus for easy replacement.

## Installation Instructions
### Prerequisites
- Set up a cross-compile toolchain for compiling the application for both transmitting and receiving nodes.

*Step-by-step guide to be added later.*

## Usage
*Detailed usage instructions to be added later.*

**Commands:** No specific commands required to use the system.

## Architecture
### Hardware
#### Sensors:
1. **SHT31 Temperature & Humidity Sensor (-40℃~125℃) - I2C**  
   - **Price:** $8.49
2. **Soil Digital Temperature & Humidity Sensor Probe**  
   - **Description:** Waterproof, Stainless Steel Case Housing, High Accuracy, Low Consumption, I2C Output  
   - **Price:** $15.46
3. **LSM6DSO**  
   - **Price:** $12.95
4. **Atlas Scientific Gravity Analog pH Kit**  
   - **Price:** $67.99
5. **CQRobot Ocean: TDS (Total Dissolved Solids)**  
   - **Price:** $11.90

**Total Cost:** $116.79

#### Zolertia RE-Mote Specifications:
- **MCU:** CC2538 (ARM Cortex-M3 with on-board 2.4GHz radio)
- **Radio:**
  - Two radio interfaces (IEEE 802.15.4): 2.4GHz and 863-950MHz
  - RP-SMA connector for external antenna (with a RF switch to select either 2.4GHz/Sub-GHz radio)
- **USB-to-Serial:** CP2104
- **Peripherals:** RTCC, built-in battery charger for LiPo batteries, External WDT (optional), Micro-SD
- **Others:** RGB LED, power management block (150nA when the mote is shutdown)

### Software
- **Operating System:** Contiki-NG (open source with extensive documentation)
- **Schematic Diagrams:** Included in the KiCad project folder in the repository.

## Contributors
- Alexander Regueiro
- Brian Neris
- Daniel Izquierdo

## Contact
- **Alexander Regueiro:** [alexinquiries1020@proton.me](mailto:alexinquiries1020@proton.me)

## License
- GNU License

## Demo
*Videos and pictures to be added later.*
