# VR Robot Arm Control

Control a 3-motor robot arm using an Oculus Quest 2 VR headset or keyboard - locally or over the internet!

## Hardware

- **Oculus Quest 2** - VR headset with controller
- **Teensy 4.1** - Microcontroller
- **ODrive Motor Controllers** - 3x ODrive controllers (CAN bus, Node IDs 1, 2, 3)
- **3 Motors** - With 50:1 gear ratio
  - Motor 2: Base rotation
  - Motor 3: Shoulder
  - Motor 1: Elbow

## Software Requirements

- Node.js
- Arduino IDE with Teensyduino
- OpenSSL (for generating certificates)
- ngrok (for internet access)

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

### 4. Start the Server

```powershell
node server-combined.js
```

### 5. Connect

**Local (same network):**
- Quest or PC browser: `https://<YOUR_PC_IP>:8080`

**Internet (anywhere in the world):**
```powershell
ngrok http https://localhost:8080
```
Then use the ngrok URL (e.g., `https://abc123.ngrok.io`)

## Controls

### VR Controller (with trigger held)
- **Left/Right** - Rotate base
- **Up/Down** - Move shoulder
- **Forward/Back** - Move elbow

### Web Keyboard Control
- **Q/E** - Base rotation
- **W/S** - Shoulder up/down
- **A/D** - Elbow up/down
- **SPACE** - Emergency stop

### Server Terminal Commands
- `x` - Toggle motors on/off
- `Space` or `s` - Emergency stop
- `0` - Return to zero position
- `p` - Print status

## Features

- **Auto-enable** - Motors activate on first command (no need to type `x`)
- **Auto-reconnect** - Serial reconnects automatically if disconnected
- **Dual control** - VR and keyboard work simultaneously
- **Internet control** - Anyone with the ngrok URL can control the robot

## Safety Features

- Emergency stop via spacebar
- Motor position limits: ±25 turns
- Smooth acceleration (no sudden movements)
- 60-second timeout auto-disable

## Configuration

Edit `arduino/vr_robot_arm_control.ino`:

```cpp
const float VR_SENSITIVITY = 10.0f;  // Motor turns per meter of VR movement
const float MOTOR_MIN = -25.0f;      // Position limits (turns)
const float MOTOR_MAX = 25.0f;
const float SMOOTHING = 0.15f;       // 0.0-1.0, higher = faster response
const float STEP_SIZE = 0.5f;        // Keyboard step size (turns)
```

## File Structure

```
├── arduino/
│   └── vr_robot_arm_control.ino  # Teensy firmware
├── server-combined.js            # Combined web + WebSocket server
├── relay-server.js               # Legacy separate relay server
├── xrMechanicalControllerInput.js # VR/keyboard controller handling
├── index.html                    # Web interface
├── cert.pem / key.pem           # SSL certificates
└── package.json                  # Node dependencies
```

## Troubleshooting

### Quest won't connect
- Make sure Quest and PC are on the same WiFi network
- Accept the SSL certificate warning in Quest browser

### Keyboard not working
- Make sure you're focused on the webpage (click on it)
- Check browser console for connection errors

### Serial keeps disconnecting
- Likely voltage spikes from motors - add a capacitor (100-470µF) to Teensy power
- Server will auto-reconnect within 2 seconds

### Wrong motor directions
- Edit the signs in `processVrPosition()` in the Arduino code