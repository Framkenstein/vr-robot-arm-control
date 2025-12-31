/**
 * VR Robot Arm Combined Server (Web + WebSocket)
 * 
 * Single server for both web files and WebSocket relay.
 * Perfect for ngrok tunneling (only need one tunnel).
 * 
 * Usage:
 *   node server-combined.js
 */

const WebSocket = require('ws');
const https = require('https');
const fs = require('fs');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

// ============ CONFIGURATION ============
const PORT = 8080;  // Single port for everything
const SERIAL_PORT = 'COM4';  // Teensy 4.1
const SERIAL_BAUD = 250000;

// SSL certificates
const SSL_KEY = 'key.pem';
const SSL_CERT = 'cert.pem';
// =======================================

let serialPort = null;
let parser = null;
let connectedClients = new Set();

// Track state
let isSerialConnected = false;
let vrEnabled = false;
let lastPosition = { x: 0, y: 0, z: 0 };

// MIME types for serving files
const mimeTypes = {
  '.html': 'text/html',
  '.js': 'application/javascript',
  '.css': 'text/css',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.gif': 'image/gif',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon'
};

// ============ SERIAL CONNECTION WITH AUTO-RECONNECT ============
let reconnectTimer = null;
const RECONNECT_DELAY = 2000;  // 2 seconds

function connectSerial() {
  console.log(`\n🔌 Connecting to Teensy on ${SERIAL_PORT}...`);
  
  // Clear any existing reconnect timer
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  
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
      scheduleReconnect();
    });

    serialPort.on('close', () => {
      console.log('⚠️ Serial port closed - will auto-reconnect...');
      isSerialConnected = false;
      broadcast({ type: 'serial_status', connected: false });
      scheduleReconnect();
    });

  } catch (err) {
    console.error(`✗ Failed to open serial port: ${err.message}`);
    scheduleReconnect();
  }
}

function scheduleReconnect() {
  if (reconnectTimer) return;  // Already scheduled
  
  console.log(`🔄 Reconnecting in ${RECONNECT_DELAY/1000} seconds...`);
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    connectSerial();
  }, RECONNECT_DELAY);
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

// ============ COMBINED HTTPS + WEBSOCKET SERVER ============
function startServer() {
  const server = https.createServer({
    key: fs.readFileSync(SSL_KEY),
    cert: fs.readFileSync(SSL_CERT)
  }, (req, res) => {
    // Serve static files
    let filePath = req.url === '/' ? '/index.html' : req.url;
    filePath = path.join(__dirname, filePath);
    
    const ext = path.extname(filePath);
    const contentType = mimeTypes[ext] || 'application/octet-stream';
    
    fs.readFile(filePath, (err, content) => {
      if (err) {
        if (err.code === 'ENOENT') {
          res.writeHead(404);
          res.end('File not found');
        } else {
          res.writeHead(500);
          res.end('Server error');
        }
      } else {
        res.writeHead(200, { 'Content-Type': contentType });
        res.end(content);
      }
    });
  });

  // WebSocket server on same port
  const wss = new WebSocket.Server({ server });

  wss.on('connection', (ws, req) => {
    const clientIP = req.socket.remoteAddress;
    console.log(`\n📱 Client connected from ${clientIP}`);
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
      console.log(`📱 Client disconnected`);
      connectedClients.delete(ws);
    });

    ws.on('error', (err) => {
      console.error('WebSocket error:', err.message);
      connectedClients.delete(ws);
    });
  });

  server.listen(PORT, () => {
    console.log(`\n🌐 Server running on https://0.0.0.0:${PORT}`);
    console.log(`   WebSocket available at wss://0.0.0.0:${PORT}`);
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
      lastPosition = { x: msg.x, y: msg.y, z: msg.z };
      if (isSerialConnected) {
        sendToTeensy(`P:${msg.x.toFixed(4)},${msg.y.toFixed(4)},${msg.z.toFixed(4)}`);
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

    case 'calibrate':
      console.log('\n🔧 CALIBRATE');
      sendToTeensy('c');
      break;

    case 'keyboard':
      // Keyboard control: K:base,shoulder,elbow (-1, 0, or 1 for each)
      console.log(`⌨️ Keyboard received: base=${msg.base}, shoulder=${msg.shoulder}, elbow=${msg.elbow}`);
      if (isSerialConnected) {
        sendToTeensy(`K:${msg.base},${msg.shoulder},${msg.elbow}`);
      } else {
        console.log('  ⚠️ Serial not connected - command not sent');
      }
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
console.log('║   VR ROBOT ARM SERVER (Combined)       ║');
console.log('╚════════════════════════════════════════╝');

console.log('\n📋 Available serial ports:');
listPorts().then(() => {
  connectSerial();
  startServer();
  
  console.log('\n════════════════════════════════════════');
  console.log('FOR LOCAL USE:');
  console.log(`  https://YOUR_PC_IP:${PORT}`);
  console.log('\nFOR INTERNET USE (run in another terminal):');
  console.log('  ngrok http https://localhost:8080');
  console.log('════════════════════════════════════════\n');
});

// Handle keyboard input
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
    sendToTeensy(' ');
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

process.on('SIGINT', () => {
  console.log('\n\nShutting down...');
  if (serialPort) {
    sendToTeensy('STOP');
    serialPort.close();
  }
  rl.close();
  process.exit(0);
});
