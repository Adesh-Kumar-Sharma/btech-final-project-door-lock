/*
  Bluetooth Door Lock (Arduino Uno)
  Frame format:
    [0x7E][CMD][ARG][CHK][0x7F]
    CHK = CMD ^ ARG

  Commands (CMD byte):
    0x01 -> LOCK
    0x02 -> UNLOCK
    0x03 -> STATUS_REQUEST  (device replies with a small STATUS frame on Serial/Bluetooth)
    0x04 -> PING (used by app for health check)
  ARG:
    - used for optional auth token.
    - If AUTH_REQUIRED true, ARG must match AUTH_TOKEN to accept command.
  Debounce:
    - By default, a command is accepted only if the same validated frame is received twice within DEBOUNCE_WINDOW_MS to reduce false activations.
    - A single frame that contains ARG == AUTH_OVERRIDE (special value) can be accepted immediately.
  Auto-lock:
    - If STATE_PIN is connected to HC-05 STATE and goes LOW for more than AUTOLOCK_ON_DISCONNECT_MS, the system will engage lock automatically.
*/

#include <Arduino.h>
#include <SoftwareSerial.h>

/* ========== CONFIG ========== */
// Bluetooth (HC-05) connected via SoftwareSerial:
const uint8_t BT_RX_PIN = 10; // Arduino pin to which HC-05 TX is wired
const uint8_t BT_TX_PIN = 11; // Arduino pin to which HC-05 RX is wired
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX

// HC-05 STATE pin
const uint8_t STATE_PIN = 2; // HC-05 STATE -> digital input

// Actuator / Output pins
const uint8_t RELAY_PIN = 8;     // drives relay for lock actuator
const uint8_t BUZZER_PIN = 9;    // buzzer for feedback
const uint8_t LED_PIN = 13;      // status LED

// Behavior / auth
const bool AUTH_REQUIRED = true;        // if true, commands must include AUTH_TOKEN in ARG
const uint8_t AUTH_TOKEN = 0x42;        // app/client token (1-byte for simplicity)
const uint8_t AUTH_OVERRIDE = 0xFF;     // if ARG==AUTH_OVERRIDE, treat as explicit override

// Command debounce & noise filtering
const unsigned long DEBOUNCE_WINDOW_MS = 400; // time window within which we expect duplicate frame
const unsigned long MIN_INTER_FRAME_MS  = 30; // ignore frames too close (likely noise)

// Auto-lock on disconnect
const unsigned long AUTOLOCK_ON_DISCONNECT_MS = 5000UL; // ms after disconnect to auto-lock

// Actuator safe control
const unsigned long ACTUATOR_ENGAGE_MS = 800UL;  // how long to drive actuator to lock/unlock (ms)
const unsigned long SAFETY_LOCK_DELAY_MS = 500UL; // minimal delay between toggles

/* ========== END CONFIG ========== */

/* Frame parsing constants */
const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE   = 0x7F;
const uint8_t FRAME_LEN  = 5; // start + cmd + arg + chk + end

/* Commands */
const uint8_t CMD_LOCK   = 0x01;
const uint8_t CMD_UNLOCK = 0x02;
const uint8_t CMD_STATUS = 0x03;
const uint8_t CMD_PING   = 0x04;

/* Runtime state */
bool lockedState = true; // assume locked at boot (set safety)
unsigned long lastAcceptedActionMillis = 0;
unsigned long lastReceivedFrameMillis = 0;

// For duplicate-frame debounce:
uint8_t lastValidCmd = 0;
uint8_t lastValidArg = 0;
unsigned long lastValidFrameTime = 0;
bool awaitingDuplicate = false;

// For auto-lock via STATE pin:
bool lastStatePinConnected = false;
unsigned long lastStateChangeMillis = 0;
unsigned long lastDisconnectMillis = 0;

/* Buffers for incoming stream from BT or Serial */
uint8_t streamBuffer[16];
uint8_t streamBufferLen = 0;

void safeActuatorLock() {
  // Engage actuator to lock (relay HIGH for ACTUATOR_ENGAGE_MS)
  digitalWrite(RELAY_PIN, HIGH);
  tone(BUZZER_PIN, 1000, 80); // beep
  delay(ACTUATOR_ENGAGE_MS);
  digitalWrite(RELAY_PIN, LOW); // stop driving
  lockedState = true;
  lastAcceptedActionMillis = millis();
  digitalWrite(LED_PIN, HIGH); // LED on when locked
}

void safeActuatorUnlock() {
  // Engage actuator to unlock - mirror of lock
  digitalWrite(RELAY_PIN, HIGH);
  tone(BUZZER_PIN, 2000, 80); // different beep
  delay(ACTUATOR_ENGAGE_MS);
  digitalWrite(RELAY_PIN, LOW);
  lockedState = false;
  lastAcceptedActionMillis = millis();
  digitalWrite(LED_PIN, LOW); // LED off when unlocked
}

void sendStatusResponse(Stream &outStream) {
  // Send small framed STATUS response: [START][CMD_STATUS][locked?][CHK][END]
  uint8_t cmd = CMD_STATUS;
  uint8_t arg = lockedState ? 1 : 0;
  uint8_t chk = cmd ^ arg;
  uint8_t frame[5] = { START_BYTE, cmd, arg, chk, END_BYTE };
  for (int i=0;i<5;i++) outStream.write(frame[i]);
  outStream.flush();
}

bool validateFrame(const uint8_t *frame) {
  if (frame[0] != START_BYTE) return false;
  if (frame[4] != END_BYTE) return false;
  uint8_t cmd = frame[1];
  uint8_t arg = frame[2];
  uint8_t chk = frame[3];
  if ((cmd ^ arg) != chk) return false;
  // frame valid
  return true;
}

void handleValidatedFrame(const uint8_t *frame, Stream &replyStream) {
  uint8_t cmd = frame[1];
  uint8_t arg = frame[2];
  // Authentication check
  if (AUTH_REQUIRED) {
    if (!(arg == AUTH_TOKEN || arg == AUTH_OVERRIDE)) {
      // bad token -> ignore but optionally reply with NACK
      Serial.println("AUTH failed - ignoring frame");
      return;
    }
  }

  unsigned long now = millis();
  // Debounce logic: require duplicate identical valid frame within DEBOUNCE_WINDOW_MS
  if (!awaitingDuplicate) {
    // store for duplicate checking
    lastValidCmd = cmd;
    lastValidArg = arg;
    lastValidFrameTime = now;
    awaitingDuplicate = true;
    // do not execute yet; wait for duplicate
    Serial.println("Valid frame received - waiting for duplicate to confirm (debounce)");
    return;
  } else {
    // there was a previous frame awaiting duplicate
    if (cmd == lastValidCmd && arg == lastValidArg && (now - lastValidFrameTime) <= DEBOUNCE_WINDOW_MS) {
      // confirmed command -> execute
      Serial.print("Duplicate confirmed; executing cmd: ");
      Serial.println(cmd, HEX);

      // Safety: don't toggle actuator too frequently
      if ((now - lastAcceptedActionMillis) < SAFETY_LOCK_DELAY_MS) {
        Serial.println("Action skipped: safety delay");
        awaitingDuplicate = false;
        return;
      }

      if (cmd == CMD_LOCK) {
        safeActuatorLock();
      } else if (cmd == CMD_UNLOCK) {
        safeActuatorUnlock();
      } else if (cmd == CMD_STATUS) {
        // return status
        sendStatusResponse(replyStream);
      } else if (cmd == CMD_PING) {
        // reply with a PING ACK (reuse STATUS frame with arg=0xAA)
        uint8_t ackFrame[5] = { START_BYTE, CMD_PING, 0xAA, (uint8_t)(CMD_PING ^ 0xAA), END_BYTE };
        for (int i=0;i<5;i++) replyStream.write(ackFrame[i]);
        replyStream.flush();
      } else {
        Serial.println("Unknown command");
      }

      // reset duplicate waiting
      awaitingDuplicate = false;
      lastValidCmd = 0;
      lastValidArg = 0;
      lastValidFrameTime = 0;
    } else {
      // new/different frame - replace waiting frame with this new one
      lastValidCmd = cmd;
      lastValidArg = arg;
      lastValidFrameTime = now;
      // still waiting for duplicate
      Serial.println("Different frame received - replaced waiting frame");
    }
  }
}

void feedStream(uint8_t b, Stream &replyStream) {
  // Append byte to rolling buffer and try to detect a 5-byte frame ending at last position
  if (streamBufferLen >= sizeof(streamBuffer)) streamBufferLen = 0; // reset if overflow (simple recovery)
  streamBuffer[streamBufferLen++] = b;

  // Keep scanning for possible frame by checking last 5 bytes
  if (streamBufferLen >= FRAME_LEN) {
    uint8_t idx = streamBufferLen - FRAME_LEN;
    uint8_t candidate[5];
    for (int i=0;i<5;i++) candidate[i] = streamBuffer[idx + i];

    if (validateFrame(candidate)) {
      unsigned long now = millis();
      // basic rate limiting - ignore frames that are too close (noise)
      if ((now - lastReceivedFrameMillis) < MIN_INTER_FRAME_MS) {
        // skip
        Serial.println("Frame dropped: inter-frame too close");
        lastReceivedFrameMillis = now;
        return;
      }
      lastReceivedFrameMillis = now;
      handleValidatedFrame(candidate, replyStream);
    }
  }
}

void readFromBluetooth() {
  while (btSerial.available()) {
    int incoming = btSerial.read();
    if (incoming < 0) break;
    feedStream((uint8_t)incoming, btSerial);
  }
}

void readFromUSBSerial() {
  // Accept same framed commands over USB Serial for debugging/testing
  while (Serial.available()) {
    int incoming = Serial.read();
    if (incoming < 0) break;
    feedStream((uint8_t)incoming, Serial);
  }
}

void setupPins() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  if (STATE_PIN != 255) {
    pinMode(STATE_PIN, INPUT);
    lastStatePinConnected = digitalRead(STATE_PIN);
    lastStateChangeMillis = millis();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait */ }
  Serial.println("Booting Bluetooth Door Lock firmware...");

  btSerial.begin(9600); // HC-05 default baud
  setupPins();

  // Ensure initial lock state
  digitalWrite(LED_PIN, HIGH); // show locked
  lockedState = true;
  Serial.println("Setup complete. System assumed LOCKED at boot.");
}

void loop() {
  // Read incoming from both BT module and USB serial
  readFromBluetooth();
  readFromUSBSerial();

  // Handle waiting for duplicate frame timeout: if waiting too long, reset
  if (awaitingDuplicate) {
    if ((millis() - lastValidFrameTime) > (DEBOUNCE_WINDOW_MS + 200)) {
      // timed out waiting for duplicate - discard
      awaitingDuplicate = false;
      Serial.println("Debounce timeout - discarded waiting frame");
    }
  }

  // Handle HC-05 STATE pin for auto-lock on disconnect
  if (STATE_PIN != 255) {
    bool cur = digitalRead(STATE_PIN);
    if (cur != lastStatePinConnected) {
      lastStatePinConnected = cur;
      lastStateChangeMillis = millis();
      if (!cur) {
        // disconnected now
        lastDisconnectMillis = millis();
        Serial.println("HC-05 STATE -> disconnected");
      } else {
        Serial.println("HC-05 STATE -> connected");
      }
    } else {
      // unchanged - if disconnected and enough time elapsed, auto-lock
      if (!cur) {
        if ((millis() - lastDisconnectMillis) >= AUTOLOCK_ON_DISCONNECT_MS) {
          // trigger auto-lock if not already locked
          if (!lockedState && ((millis() - lastAcceptedActionMillis) > SAFETY_LOCK_DELAY_MS)) {
            Serial.println("Auto-locking due to disconnect");
            safeActuatorLock();
            // Optionally broadcast status over BT
            sendStatusResponse(btSerial);
          }
          // prevent repeating (set lastDisconnectMillis far in future)
          lastDisconnectMillis = millis() + 1000000UL;
        }
      }
    }
  }

  // small idle delay
  delay(5);
}
