/**
 * VR Robot Arm Relay Server
 * 
 * This runs on your PC and:
 * 1. Receives position data from Quest 2 via WebSocket
 * 2. Forwards it to Teensy via Serial port
 * 
 * Usage:
 *   node relay-server.js
 * 
 * Then open the Quest browser to https://YOUR_PC_IP:8080
 */

const WebSocket = require('ws');
const https = require('https');
const fs = require('fs');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

// ============ CONFIGURATION ============
const WEBSOCKET_PORT = 8081;
const SERIAL_PORT = 'COM4';  // Teensy 4.1
const SERIAL_BAUD = 250000;

// SSL certificates (same as http-server uses)
const SSL_KEY = 'key.pem';
const SSL_CERT = 'cert.pem';
// =======================================

let serialPort = null;
let parser = null;
let wsServer = null;
let connectedClients = new Set();

// Track state
let isSerialConnected = false;
let vrEnabled = false;
let lastPosition = { x: 0, y: 0, z: 0 };

// ============ SERIAL CONNECTION ============
function connectSerial() {
  console.log(`\n🔌 Connecting to Teensy on ${SERIAL_PORT}...`);
  
  try {
    serialPort = new SerialPort({
      path: SERIAL_PORT,
      baudRate: SERIAL_BAUD,
    });

    parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

    serialPort.on('open', () => {
      isSerialConnected = true;
      console.log(`✓ Serial connected to ${SERIAL_PORT} at ${SERIAL_BAUD} baud`);
      broadcast({ type: 'serial_status', connected: true });
    });

    parser.on('data', (data) => {
      console.log(`← Teensy: ${data}`);
      broadcast({ type: 'teensy_message', message: data });
    });

    serialPort.on('error', (err) => {
      console.error(`✗ Serial error: ${err.message}`);
      isSerialConnected = false;
      broadcast({ type: 'serial_status', connected: false, error: err.message });
    });

    serialPort.on('close', () => {
      console.log('Serial port closed');
      isSerialConnected = false;
      broadcast({ type: 'serial_status', connected: false });
    });

  } catch (err) {
    console.error(`✗ Failed to open serial port: ${err.message}`);
    console.log('\nAvailable ports:');
    listPorts();
  }
}

async function listPorts() {
  const ports = await SerialPort.list();
  ports.forEach(port => {
    console.log(`  ${port.path} - ${port.manufacturer || 'Unknown'}`);
  });
}

function sendToTeensy(message) {
  if (serialPort && isSerialConnected) {
    serialPort.write(message + '\n', (err) => {
      if (err) {
        console.error('Serial write error:', err.message);
      }
    });
  }
}

// ============ WEBSOCKET SERVER (with SSL) ============
function startWebSocketServer() {
  // Create HTTPS server for secure WebSocket
  const server = https.createServer({
    key: fs.readFileSync(SSL_KEY),
    cert: fs.readFileSync(SSL_CERT)
  });
  
  wsServer = new WebSocket.Server({ server });
  
  server.listen(WEBSOCKET_PORT, () => {
    console.log(`\n🌐 Secure WebSocket server running on wss://0.0.0.0:${WEBSOCKET_PORT}`);
  });

  wsServer.on('connection', (ws, req) => {
    const clientIP = req.socket.remoteAddress;
    console.log(`\n📱 Quest connected from ${clientIP}`);
    connectedClients.add(ws);
    
    // Send current status
    ws.send(JSON.stringify({
      type: 'status',
      serial_connected: isSerialConnected,
      vr_enabled: vrEnabled
    }));

    ws.on('message', (data) => {
      try {
        const msg = JSON.parse(data);
        handleMessage(msg, ws);
      } catch (e) {
        console.error('Invalid message:', data.toString());
      }
    });

    ws.on('close', () => {
      console.log(`📱 Quest disconnected`);
      connectedClients.delete(ws);
    });

    ws.on('error', (err) => {
      console.error('WebSocket error:', err.message);
      connectedClients.delete(ws);
    });
  });
}

function broadcast(message) {
  const data = JSON.stringify(message);
  connectedClients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(data);
    }
  });
}

function handleMessage(msg, ws) {
  switch (msg.type) {
    case 'position':
      // Position update from Quest - always send if serial connected
      lastPosition = { x: msg.x, y: msg.y, z: msg.z };
      if (isSerialConnected) {
        sendToTeensy(`P:${msg.x.toFixed(4)},${msg.y.toFixed(4)},${msg.z.toFixed(4)}`);
        // Log every 20th position to avoid spam
        if (Math.random() < 0.05) {
          console.log(`→ Position: ${msg.x.toFixed(3)}, ${msg.y.toFixed(3)}, ${msg.z.toFixed(3)}`);
        }
      }
      break;

    case 'enable':
      vrEnabled = msg.enabled;
      console.log(`\n${vrEnabled ? '▶️  VR CONTROL ENABLED' : '⏸️  VR CONTROL DISABLED'}`);
      if (isSerialConnected) {
        sendToTeensy(vrEnabled ? 'E:1' : 'E:0');
      }
      broadcast({ type: 'vr_status', enabled: vrEnabled });
      break;

    case 'stop':
      console.log('\n🛑 EMERGENCY STOP');
      vrEnabled = false;
      sendToTeensy('STOP');
      broadcast({ type: 'vr_status', enabled: false });
      break;

    case 'home':
      console.log('\n🏠 HOME');
      sendToTeensy('HOME');
      break;

    case 'ping':
      ws.send(JSON.stringify({ type: 'pong' }));
      break;

    default:
      console.log('Unknown message type:', msg.type);
  }
}

// ============ MAIN ============
console.log('╔════════════════════════════════════════╗');
console.log('║     VR ROBOT ARM RELAY SERVER          ║');
console.log('╚════════════════════════════════════════╝');

// List available ports first
console.log('\n📋 Available serial ports:');
listPorts().then(() => {
  // Start servers
  connectSerial();
  startWebSocketServer();
  
  console.log('\n════════════════════════════════════════');
  console.log('INSTRUCTIONS:');
  console.log(`1. Make sure Teensy is on ${SERIAL_PORT} (edit SERIAL_PORT if different)`);
  console.log('2. On Teensy Serial Monitor, press X to enable motors');
  console.log('3. Open Quest browser to https://192.168.x.x:8080');
  console.log('4. Click "Enable VR Control" in the Quest browser');
  console.log('5. Move your right hand to control the robot!');
  console.log('════════════════════════════════════════\n');
});

// Handle keyboard input from terminal
const readline = require('readline');
const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

rl.on('line', (input) => {
  const cmd = input.trim().toLowerCase();
  if (cmd === 'x') {
    sendToTeensy('x');
    console.log('→ Sent: x (toggle safety)');
  } else if (cmd === 'v') {
    sendToTeensy('v');
    console.log('→ Sent: v (toggle VR mode)');
  } else if (cmd === 'stop' || cmd === '' || cmd === 's') {
    // Empty enter, 's', or 'stop' = EMERGENCY STOP
    sendToTeensy(' ');  // Space triggers emergency stop on Teensy
    console.log('🛑 EMERGENCY STOP!');
  } else if (cmd === 'home' || cmd === '0') {
    sendToTeensy('0');
    console.log('→ Sent: 0 (home)');
  } else if (cmd === 'p') {
    sendToTeensy('p');
    console.log('→ Sent: p (print status)');
  } else if (cmd) {
    sendToTeensy(cmd);
    console.log(`→ Sent: ${cmd}`);
  }
});

console.log('\nKEYBOARD COMMANDS: x=safety, v=vr mode, stop, home, p=status\n');

// Handle graceful shutdown
process.on('SIGINT', () => {
  console.log('\n\nShutting down...');
  if (serialPort) {
    sendToTeensy('STOP');
    serialPort.close();
  }
  if (wsServer) {
    wsServer.close();
  }
  rl.close();
  process.exit(0);
});
