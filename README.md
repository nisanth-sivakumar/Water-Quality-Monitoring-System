# Water Quality Monitoring System

A distributed embedded system for real-time water quality monitoring using two communicating STM32 microcontroller boards. The **Sensor Board** acquires pH and turbidity measurements via I2C and transmits data to the **Display Board** over UART for real-time readout.

---

## System Architecture

```
┌─────────────────────────────┐         UART          ┌─────────────────────────────┐
│         Sensor Board        │  ──────────────────►  │        Display Board        │
│                             │                        │                             │
│  pH Sensor     ──► I2C ──►  │                        │  ──► Parse & Display Data   │
│  Turbidity Sensor ► I2C ──► │  STM32 ◄──────────►   │      STM32                  │
└─────────────────────────────┘                        └─────────────────────────────┘
```

The system is split across two independent STM32 boards to separate concerns: one node handles sensor interfacing and data acquisition, the other handles data presentation. Communication between boards is handled over a UART serial link with a simple framed packet protocol, achieving a transmission error rate under 1%.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | STM32 (x2) |
| pH Sensor | I2C interface |
| Turbidity Sensor | I2C interface |
| Communication | UART (inter-board) |
| IDE | STM32CubeIDE |

---

## Firmware Structure

```
Water-Quality-Monitoring-System/
├── Sensor_Board/          # Firmware for the sensor acquisition node
│   ├── Core/
│   │   ├── Src/           # Application source (sensor reads, I2C, UART TX)
│   │   └── Inc/           # Headers
│   └── Drivers/           # STM32 HAL drivers
│
└── Display_Board/         # Firmware for the display/output node
    └── Display_Board-main/
        ├── Core/
        │   ├── Src/       # UART RX, data parsing, display logic
        │   └── Inc/
        └── Drivers/
```

---

## Key Implementation Details

**Sensor Board**
- Initializes I2C peripheral to interface with pH and turbidity sensors
- Samples sensor data at regular intervals and applies basic filtering
- Frames measurement data into a packet and transmits over UART to the Display Board

**Display Board**
- Listens on UART for incoming sensor packets
- Parses and validates received frames
- Renders current pH and turbidity readings in real time

**Inter-board Communication**
- Protocol: UART serial link between STM32 boards
- Transmission error rate: <1% across test runs
- Measurement accuracy: 95% across pH and turbidity validation tests

---

## Building & Flashing

### Requirements
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- STM32 programmer / ST-LINK debugger

### Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/nisanth-sivakumar/Water-Quality-Monitoring-System.git
   ```

2. Open STM32CubeIDE and import the project for each board:
   - `File → Import → Existing Projects into Workspace`
   - Select `Sensor_Board/` for the sensor node
   - Select `Display_Board/Display_Board-main/` for the display node

3. Build each project:
   - `Project → Build Project` (or `Ctrl+B`)

4. Flash to the respective STM32 board via ST-LINK:
   - `Run → Debug` or `Run → Run`

> Flash the Sensor Board and Display Board independently. Ensure both boards share a common ground and are connected UART TX/RX pins are crossed (Sensor TX → Display RX).

---

## Results

| Metric | Result |
|---|---|
| pH & turbidity measurement accuracy | 95% across test conditions |
| UART inter-board transmission error rate | < 1% |
| Sensor interface | I2C (pH + turbidity) |
| Data flow | Real-time, continuous sampling |

---

## Tools & Technologies

- **Language:** C
- **Platform:** STM32 microcontrollers
- **IDE:** STM32CubeIDE
- **Protocols:** I2C (sensor interface), UART (inter-board communication)
- **HAL:** STM32 Hardware Abstraction Layer
- **Version Control:** Git
