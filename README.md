# VR Robot Arm Control

Control a 3-motor robot arm using an Oculus Quest 2 VR headset. Move your hand in VR and the robot arm follows!

## Hardware

- **Oculus Quest 2** - VR headset with controller
- **Teensy 4.1** - Microcontroller
- **ODrive Motor Controllers** - 3x ODrive controllers (CAN bus, Node IDs 1, 2, 3)
- **3 Motors** - With 50:1 gear ratio
  - Base rotation (Q/E keyboard)
  - Shoulder up/down (A/D keyboard)
  - Elbow up/down (W/S keyboard)

## Software Requirements

- Node.js
- Arduino IDE with Teensyduino
- OpenSSL (for generating certificates)

## Quick Start

### 1. Generate SSL Certificates (first time only)

```bash
openssl req -newkey rsa:2048 -new -nodes -x509 -days 3650 -keyout key.pem -out cert.pem
```

### 2. Install Dependencies

```bash
npm install
```

### 3. Upload Arduino Code

1. Open `arduino/vr_robot_arm_control.ino` in Arduino IDE
2. Select Teensy 4.1 board
3. Upload to Teensy

### 4. Start the Servers

Open **two PowerShell windows** in the project folder:

**Window 1 - Relay Server:**
```powershell
node relay-server.js
```

**Window 2 - Web Server:**
```powershell
npm run serve-ssl
```

### 5. Connect from Quest

1. Put on Quest 2 headset
2. Open browser and go to: `https://<YOUR_PC_IP>:8080`
   - Accept the self-signed certificate warning
3. Click "Enter VR"

### 6. Enable the Robot

In the relay server PowerShell window:
1. Type `x` and press Enter to enable motors

## Controls

### VR Controller (with trigger held)
- **Left/Right** - Rotate base
- **Up/Down** - Move shoulder
- **Forward/Back** - Move elbow

The arm only moves while holding the trigger. Release to stop. Position accumulates between grabs.

### Keyboard Commands (in relay server terminal)
- `x` - Enable/disable motors (MUST DO FIRST)
- `Space` or `Enter` or `s` - **EMERGENCY STOP**
- `0` - Return to zero position
- `p` - Print status

### Direct Keyboard Control (when VR mode off)
- `Q/E` - Base rotation
- `A/D` - Shoulder up/down
- `W/S` - Elbow up/down

## Safety Features

- Motors disabled on startup - must type `x` to enable
- Emergency stop via spacebar, Enter, or `s`
- Motor position limits: ±25 turns
- Smooth acceleration (no sudden movements)
- 60-second timeout auto-disable

## Configuration

Edit `arduino/vr_robot_arm_control.ino`:

```cpp
// Sensitivity: motor turns per meter of VR hand movement
const float VR_SENSITIVITY = 10.0f;

// Motor position limits (turns)
const float MOTOR_MIN = -25.0f;
const float MOTOR_MAX = 25.0f;

// Smoothing (0.0-1.0, higher = faster response)
const float SMOOTHING = 0.15f;
```

## Troubleshooting

### Quest won't connect
- Make sure Quest and PC are on the same WiFi network
- Check that both servers are running
- Accept the SSL certificate warning in Quest browser

### Motors not responding
- Type `x` in relay terminal to enable
- Check Teensy is connected (COM port in relay-server.js)
- Verify ODrive controllers are powered and detected

### Wrong motor directions
- Edit the signs in `processVrPosition()` in the Arduino code

## File Structure

```
├── arduino/
│   └── vr_robot_arm_control.ino  # Teensy firmware
├── relay-server.js               # WebSocket to Serial bridge
├── xrMechanicalControllerInput.js # VR controller handling
├── index.html                    # Web interface
├── cert.pem / key.pem           # SSL certificates
└── package.json                  # Node dependencies
```

## Network Ports

- **8080** - HTTPS web server
- **8081** - Secure WebSocket relay server
- **COM4** - Teensy serial (250000 baud)