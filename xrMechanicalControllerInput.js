"use strict";

import * as THREE from 'three';

// Working variables, prevents "new" allocations
const __rot = new THREE.Quaternion();
const __wristOffset = new THREE.Vector3();
const __euler = new THREE.Euler();
const __wristQuat = new THREE.Quaternion();

/**
 * Manages the standard WebXR mechanical "grip" controller.
 */

// ---[ Robot Arm WebSocket Control ]---
// Connects to relay server on PC which forwards to Teensy via Serial

let ws = null;
let isConnected = false;
let vrEnabled = false;
let lastSendTime = 0;
const SEND_INTERVAL_MS = 50;  // Send at 20Hz max

// Track which hand controls the robot (default: right)
let controlHand = 'right';

// Get the relay server URL (same host and port as web server for combined server)
function getRelayUrl() {
  const host = window.location.hostname;
  const port = window.location.port;
  // If there's a port (local dev), include it. If no port (ngrok), omit it.
  if (port) {
    return `wss://${host}:${port}`;
  } else {
    return `wss://${host}`;
  }
}

/**
 * Connect to the relay server running on PC
 */
export async function connectTeensySerial() {
  const url = getRelayUrl();
  console.log(`Connecting to relay server at ${url}...`);
  
  try {
    ws = new WebSocket(url);
    
    ws.onopen = () => {
      isConnected = true;
      console.log('✓ Connected to relay server');
      updateStatus('Connected to PC relay');
      
      // Update UI
      const btn = document.getElementById('serialButton');
      if (btn) {
        btn.textContent = '✓ Connected';
        btn.classList.add('connected');
      }
    };
    
    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        handleRelayMessage(msg);
      } catch (e) {
        console.log('Relay:', event.data);
      }
    };
    
    ws.onclose = () => {
      isConnected = false;
      vrEnabled = false;
      console.log('Disconnected from relay server');
      updateStatus('Disconnected');
      
      const btn = document.getElementById('serialButton');
      if (btn) {
        btn.textContent = '🔌 Connect';
        btn.classList.remove('connected');
      }
      
      // Try to reconnect after 3 seconds
      setTimeout(() => {
        if (!isConnected) {
          console.log('Attempting to reconnect...');
          connectTeensySerial();
        }
      }, 3000);
    };
    
    ws.onerror = (err) => {
      console.error('WebSocket error:', err);
      updateStatus('Connection error');
    };
    
  } catch (error) {
    console.error('Failed to connect:', error);
    isConnected = false;
  }
}

function handleRelayMessage(msg) {
  switch (msg.type) {
    case 'status':
      console.log(`Relay status - Serial: ${msg.serial_connected}, VR: ${msg.vr_enabled}`);
      if (msg.serial_connected) {
        updateStatus('Teensy connected');
      } else {
        updateStatus('Teensy not connected');
      }
      break;
    case 'serial_status':
      updateStatus(msg.connected ? 'Teensy connected' : 'Teensy disconnected');
      break;
    case 'teensy_message':
      console.log('Teensy:', msg.message);
      break;
    case 'vr_status':
      vrEnabled = msg.enabled;
      break;
  }
}

function updateStatus(text) {
  const status = document.getElementById('status');
  if (status) status.textContent = `Status: ${text}`;
}

function sendToRelay(message) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(message));
  }
}

/**
 * Send position command to robot arm
 */
export function sendPosition(x, y, z) {
  if (!isConnected) return;
  
  const now = Date.now();
  if (now - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = now;
  
  // Auto-enable VR if not enabled yet
  if (!vrEnabled) {
    vrEnabled = true;
    sendToRelay({ type: 'enable', enabled: true });
    console.log('Auto-enabled VR control');
  }
  
  sendToRelay({ type: 'position', x, y, z });
  
  // Log occasionally for debug
  if (Math.random() < 0.02) {
    console.log(`Sending position: ${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}`);
  }
}

/**
 * Send command to enable/disable VR control
 */
export function sendEnable(enable) {
  if (!isConnected) {
    console.warn('Not connected to relay server');
    return;
  }
  vrEnabled = enable;
  sendToRelay({ type: 'enable', enabled: enable });
  console.log(enable ? 'VR Control ENABLED' : 'VR Control DISABLED');
}

/**
 * Send emergency stop
 */
export function sendStop() {
  vrEnabled = false;
  sendToRelay({ type: 'stop' });
  console.log('EMERGENCY STOP sent');
}

/**
 * Send home command
 */
export function sendHome() {
  sendToRelay({ type: 'home' });
  console.log('HOME command sent');
}

/**
 * Send calibrate command (toggle calibration mode)
 */
export function sendCalibrate() {
  sendToRelay({ type: 'calibrate' });
  console.log('CALIBRATE command sent');
}

/**
 * Send keyboard movement command
 * @param {number} base - Base rotation (-1, 0, 1)
 * @param {number} shoulder - Shoulder movement (-1, 0, 1)
 * @param {number} elbow - Elbow movement (-1, 0, 1)
 */
export function sendKeyboardMove(base, shoulder, elbow) {
  console.log(`sendKeyboardMove called: base=${base}, shoulder=${shoulder}, elbow=${elbow}, connected=${isConnected}`);
  if (!isConnected) {
    console.warn('Not connected - keyboard command not sent');
    return;
  }
  sendToRelay({ type: 'keyboard', base, shoulder, elbow });
}

/**
 * Check if connected and VR enabled
 */
export function isVrControlActive() {
  return isConnected && vrEnabled;
}

/**
 * Set which hand controls the robot
 */
export function setControlHand(hand) {
  controlHand = hand;
  console.log(`Control hand set to: ${hand}`);
}

export function getControlHand() {
  return controlHand;
}

// Auto-connect when module loads
setTimeout(() => {
  console.log('Auto-connecting to relay server...');
  connectTeensySerial();
}, 1000);
// ---[ END Robot Arm WebSocket Control ]---

export class XrMechanicalControllerInput {
    constructor(context, grip, gamePad, handSide) {
        this.context = context;
        this._grip = grip;
        this._gamePad = gamePad;
        this._handSide = handSide;
        this._wristAxis = new THREE.AxesHelper(0.1);

        this.select = false;
        this.squeeze = false;
        
        // For relative movement - capture position when trigger pressed
        this._triggerWasPressed = false;
        this._grabStartPos = new THREE.Vector3();
        // Accumulated robot arm position (persists between grabs)
        this._armPosition = { x: 0, y: 0, z: 0 };
        this._armStartPos = { x: 0, y: 0, z: 0 };  // Snapshot when grab started
        
        this.touchPad = new THREE.Vector2();
        this.touchPadButton = false;
        this.thumbStick = new THREE.Vector2();
        this.thumbStickButton = false;
        this.buttonA = false;
        this.buttonB = false;
        this.hasHand = false;

        this._localPosition = new THREE.Vector3();
        this._localRotation = new THREE.Quaternion();
        this._worldPosition = new THREE.Vector3();
        this._worldRotation = new THREE.Quaternion();
        this.pointerActive = true;
        this._pointerWOrigin = new THREE.Vector3();
        this._pointerWDirection = new THREE.Vector3();
        this._lastUpdate = -1;
    }

    /*
     * Position of the head tracker relative to the parent object.
     */
    get wristLPos() {
        this.refresh();
        return this._localPosition;
    }

    /*
     * Rotation of the head tracker relative to the parent object.
     */
    get wristLQuat() {
        this.refresh();
        return this._localRotation;
    }

    /*
     * position of head in world coordinates
     */
    get wristWPos() {
        this.refresh();
        return this._worldPosition;
    }

    /*
     * rotation of head in world orientation
     */
    get wristWQuat() {
        this.refresh();
        return this._worldRotation;
    }

    /*
     *  The position of the pointer that matches the controller
     */
    get pointerWOrigin() {
        this.refresh();
        return this._pointerWOrigin;
    }

    /*
     *  The direction of the pointer that matches the controller
     */
    get pointerWDirection() {
        this.refresh();
        return this._pointerWDirection;
    }   

    /*
     * Apply haptic feedback to the controller (vibrate)
     */
    vibrate(intensity, timeMs) {
        if (this._gamePad.hapticActuators && this._gamePad.hapticActuators.length >= 1) {
            this._gamePad.hapticActuators[0].pulse(intensity || 1, timeMs || 100);
        }
    }

    /**
     * Called when the controller is connected
     */
    onConnect() {       
        this.context.scene.add(this._wristAxis); 
    }

    /**
     * Called on each animation frame
     */
    onAnimate() {  
        this._wristAxis.position.copy(this.wristWPos);
        this._wristAxis.quaternion.copy(this.wristWQuat);
    }

    /**
     * Called when the controller is disconnected
     */
    onDisconnect() {
        this._wristAxis?.removeFromParent();
    }

    /*
     * In order to keep the reference points like the wrist location and the 
     * pointer location abstracted from input (hand or controller) map
     * the wrist position and rotation to a [[[]]] and position the pointer
     * at the point in the controller that makes sense for the controller. 
     */
    refresh() {
        if (this._lastUpdate == this.context.frame) {
            return; // already updated for this frame
        }
        this._lastUpdate = this.context.frame;

        // Position, and determine local (to the parent) position
        this._grip.getWorldPosition(this._worldPosition);
        this._grip.getWorldQuaternion(this._worldRotation);

        // Offset the world position to find the wrist location
        const offset = (this._handSide == 'left') 
            ? { x: -0.02, y: 0.0, z: 0.09 }  // Left Wrist offset
            : { x:  0.02, y: 0.0, z: 0.09 }; // Right Wrist offset
        __wristOffset.set(offset.x, offset.y, offset.z);
        __wristOffset.applyQuaternion(this._worldRotation);
        this._worldPosition.add(__wristOffset);

        // Send position to robot arm if this is the control hand AND trigger is held
        // Uses RELATIVE movement - movement accumulates between grabs
        if (this._handSide === controlHand && isConnected) {
            if (this.select) {
                if (!this._triggerWasPressed) {
                    // Just pressed trigger - capture start positions
                    this._grabStartPos.copy(this._worldPosition);
                    this._armStartPos.x = this._armPosition.x;
                    this._armStartPos.y = this._armPosition.y;
                    this._armStartPos.z = this._armPosition.z;
                    this._triggerWasPressed = true;
                    console.log('Grabbed - arm at:', this._armPosition.x.toFixed(3), this._armPosition.y.toFixed(3), this._armPosition.z.toFixed(3));
                }
                
                // Calculate delta from grab start position
                const deltaX = this._worldPosition.x - this._grabStartPos.x;
                const deltaY = this._worldPosition.y - this._grabStartPos.y;
                const deltaZ = this._worldPosition.z - this._grabStartPos.z;
                
                // Add delta to arm start position (accumulate)
                this._armPosition.x = this._armStartPos.x + deltaX;
                this._armPosition.y = this._armStartPos.y + deltaY;
                this._armPosition.z = this._armStartPos.z + deltaZ;
                
                // Send accumulated position
                // Swap X and Z: VR left/right (X) should control base rotation
                sendPosition(this._armPosition.z, this._armPosition.y, this._armPosition.x);
            } else {
                if (this._triggerWasPressed) {
                    // Just released trigger - position is saved in _armPosition
                    this._triggerWasPressed = false;
                    console.log('Released - arm stays at:', this._armPosition.x.toFixed(3), this._armPosition.y.toFixed(3), this._armPosition.z.toFixed(3));
                }
            }
        }

        // Convert world position and rotation to relative to the parent object
        this._localPosition.copy(this._worldPosition);
        this._localPosition.sub(this._grip.parent.position);
        __rot.copy(this._grip.parent.quaternion).invert();
        this._localPosition.applyQuaternion(__rot);

        // Rotate the hand so that the fingers are forward, thumb up position
        if (this._handSide == 'left') {
            __wristQuat.setFromEuler(__euler.set(0.0, Math.PI / 8.0 * 1.5, Math.PI / 2.0, "ZYX"));
        } else {
            __wristQuat.setFromEuler(__euler.set(0.0, -Math.PI / 8.0 * 1.5, -Math.PI / 2.0, "ZYX"));
        }
        this._worldRotation.multiply(__wristQuat);

        // Rotation, and determine local (to the parent) rotation
        this._localRotation.copy(this._grip.parent.quaternion);
        this._localRotation.invert();
        this._localRotation.multiply(this._worldRotation);

        // Pointer
        this._grip.getWorldPosition(this._pointerWOrigin.setScalar(0));
        this._pointerWDirection.set(0, -1, -1).normalize(); // Forward
        this._grip.getWorldQuaternion(__rot);
        this._pointerWDirection.applyQuaternion(__rot);

        // Update gamepad
        // https://www.w3.org/TR/webxr-gamepads-module-1/
        if (this._gamePad) {
            let axis = this._gamePad.axes;
            if (axis && axis.length > 3) {
                // Mixed Reality
                this.touchPad.set(axis[0], axis[1]);
                // Mixed Reality and Quest 2
                this.thumbStick.set(axis[2], axis[3]);
            }
            let buttons = this._gamePad.buttons;
            if (buttons) {
                // Mixed Reality and Quest 2
                this.touchPadButton = (buttons.length > 2) ? buttons[2].pressed : false;
                this.thumbStickButton = (buttons.length > 3) ? buttons[3].pressed : false;
                // Quest 2
                this.buttonA = (buttons.length > 4) ? buttons[4].pressed : false;
                this.buttonB = (buttons.length > 5) ? buttons[5].pressed : false;
            }
        }
    }
}
