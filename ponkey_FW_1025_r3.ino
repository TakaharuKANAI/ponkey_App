#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ========================================
// 定数定義
// ========================================

#define LED_PIN 21
#define LED_COUNT 32
#define DEBUG 1

const int SDA_PIN = 17;
const int SCL_PIN = 18;
const uint8_t MCP_ADDR = 0x20;

#define SERVICE_UUID "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

// 拡張コントロールチェンジ定義
const uint8_t CC_STATE_REQUEST = 0x50;
const uint8_t CC_STATE_SYNC_START = 0x51;
const uint8_t CC_HEARTBEAT = 0x52;
const uint8_t CC_RECORDING_MODE = 0x53;
const uint8_t CC_BAR_SYNC = 0x54;        // バー頭同期(旧STEP_SYNC)
const uint8_t CC_PATTERN_DATA = 0x55;

// タイミング定数
const unsigned long SOLENOID_PULSE_MS = 50;
const unsigned long PHYSICAL_KEY_DETECTION_WINDOW = 200;
const unsigned long NOTE_GROUPING_WINDOW = 500;
const unsigned long MULTI_PRESS_TIMEOUT = 100;
const unsigned long KEY_SCAN_INTERVAL = 5;
const unsigned long UPDATE_INTERVAL = 10;
const unsigned long HEARTBEAT_INTERVAL = 1000;
const unsigned long STATE_SYNC_DELAY = 500;
const unsigned long LONG_PRESS_DURATION = 500;  // 長押し判定時間

const uint8_t RECORDING_PLAYBACK_NOTE_THRESHOLD = 3;
const uint8_t RECORDING_PLAYBACK_TIME_THRESHOLD = 100;
const uint8_t MAX_SIMULTANEOUS_SOLENOIDS = 8;

// ピン定義
const uint8_t colPins[4] = {4, 5, 6, 15};
const uint8_t rowPins[4] = {3, 2, 1, 0};
const uint8_t directPins[7] = {14, 13, 12, 11, 10, 9, 8};
const uint8_t directPinToMode[7] = {1, 2, 3, 4, 5, 6, 7};
const int ESP_KEY_PIN = 35;

// マッピング
const uint8_t MATRIX_TO_SOLENOID[16] = {
  14, 13, 12, 11, 10, 9,  3,  8,
  16, 15, 7,  6,  5,  4,  2,  1
};

const uint8_t MATRIX_TO_LED[16][2] = {
  {24, 31}, {25, 30}, {26, 29}, {27, 28},
  {16, 23}, {17, 22}, {18, 21}, {19, 20},
  {8,  15}, {9,  14}, {10, 13}, {11, 12},
  {0,  7},  {1,  6},  {2,  5},  {3,  4}
};

const uint8_t CENTER_KEYS[4] = {5, 6, 9, 10};
const uint8_t CORNER_KEYS[4] = {0, 3, 12, 15};  // 修正: 四隅の正しい位置

// アドバタイズ表示用のソレノイド番号
const uint8_t ADVERTISE_INNER[4] = {9, 3, 15, 7};     // 内側4つ (マトリクス5,6,9,10)
const uint8_t ADVERTISE_CORNERS[4] = {14, 11, 1, 5};  // 四隅4つ (マトリクス0,3,12,15)

const uint32_t MODE_COLORS[8] = {
  0xFF0000, 0x00FF00, 0x0000FF, 0xFF00FF,
  0xFFFF00, 0x00FFFF, 0x8000FF, 0xFF8000
};

const uint32_t ADVERTISE_COLOR = 0xFF4000;  // 赤みの強いオレンジ色
const uint8_t ADVERTISE_BRIGHTNESS = 100;   // アドバタイズ時の明るさ

const char* PART_NAMES[8] = {
  "KICK", "SNARE", "HAT", "PERC", "PAD", "LEAD", "PLUCK", "BELL"
};

const uint8_t BRIGHTNESS_OFF = 0;
const uint8_t BRIGHTNESS_TOGGLE_ON = 26;
const uint8_t BRIGHTNESS_INDICATOR = 51;
const uint8_t BRIGHTNESS_INDICATOR_SYNTH = 26;
const uint8_t BRIGHTNESS_ACTIVE = 255;
const uint8_t FADE_IN_PERCENT = 5;
const uint8_t FADE_OUT_PERCENT = 20;

// ========================================
// グローバルオブジェクト
// ========================================

Adafruit_MCP23X17 mcp;
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
unsigned long connectionStartTime = 0;
bool needsStateSync = false;

// 状態同期用キュー
struct SyncMessage {
  uint8_t control;
  uint8_t value;
  unsigned long sendTime;
};
SyncMessage syncQueue[128];
uint8_t syncQueueHead = 0;
uint8_t syncQueueTail = 0;
bool isSyncing = false;

// ========================================
// 状態管理
// ========================================

struct PonkeyState {
  uint8_t currentMode;
  bool rhythmPatterns[4][16];
  bool matrixKeys[16];
  bool directKeys[8];
  uint8_t ledBrightness[32];
  unsigned long solenoidOffTime[16];
  bool isPlaying;
  uint8_t currentStep;
  uint16_t bpm;
  unsigned long lastStepTime;
  unsigned long indicatorStartTime[16];
  bool indicatorActive[16];
  unsigned long multiPressTimer;
  bool multiPressDetected;
  uint8_t multiPressType;
  bool physicalKeyPressed[16];
  unsigned long physicalKeyPressTime[16];
  unsigned long lastNoteTime;
  uint8_t notesInWindow;
  uint8_t activeSolenoidCount;
  bool appRecording;
  unsigned long lastHeartbeatSent;
  unsigned long lastHeartbeatReceived;
  unsigned long directKeyPressTime[8];  // ダイレクトキー長押し判定用
  bool directKeyLongPressHandled[8];    // 長押し処理済みフラグ

  PonkeyState() : 
    currentMode(0), isPlaying(false), currentStep(0), bpm(100),
    lastStepTime(0), multiPressTimer(0), multiPressDetected(false),
    multiPressType(0), lastNoteTime(0), notesInWindow(0),
    activeSolenoidCount(0), appRecording(false), 
    lastHeartbeatSent(0), lastHeartbeatReceived(0)
  {
    memset(rhythmPatterns, false, sizeof(rhythmPatterns));
    memset(matrixKeys, false, sizeof(matrixKeys));
    memset(directKeys, false, sizeof(directKeys));
    memset(ledBrightness, 0, sizeof(ledBrightness));
    memset(solenoidOffTime, 0, sizeof(solenoidOffTime));
    memset(indicatorStartTime, 0, sizeof(indicatorStartTime));
    memset(indicatorActive, false, sizeof(indicatorActive));
    memset(physicalKeyPressed, false, sizeof(physicalKeyPressed));
    memset(physicalKeyPressTime, 0, sizeof(physicalKeyPressTime));
    memset(directKeyPressTime, 0, sizeof(directKeyPressTime));
    memset(directKeyLongPressHandled, false, sizeof(directKeyLongPressHandled));
  }
} state;

// ========================================
// 関数プロトタイプ
// ========================================

void queueSyncMessage(uint8_t control, uint8_t value);
void processSyncQueue();
void startStateSync();
void sendHeartbeat();
void sendMIDIOverBLE(uint8_t status, uint8_t data1, uint8_t data2);
bool activateSolenoid(uint8_t note);
void deactivateSolenoid(uint8_t note);
void setModeColor(uint8_t mode);
void clearAllLEDs();

// ========================================
// BLEコールバック
// ========================================

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    needsStateSync = true;
    connectionStartTime = millis();
    state.lastHeartbeatReceived = millis();
    Serial.println("BLE Connected - sync scheduled");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    needsStateSync = false;
    isSyncing = false;
    connectionStartTime = 0;
    syncQueueHead = 0;
    syncQueueTail = 0;
    Serial.println("BLE Disconnected");
    delay(100);
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() < 3) return;

    uint8_t status = value[2];
    uint8_t command = status & 0xF0;

#if DEBUG
    Serial.printf("MIDI Rx: ");
    for (size_t i = 0; i < value.length(); i++) {
      Serial.printf("%02X ", (uint8_t)value[i]);
    }
    Serial.println();
#endif

    if (command == 0x90 && value.length() >= 5) {
      uint8_t note = value[3];
      uint8_t velocity = value[4];
      
      if (note >= 16) return;
      unsigned long now = millis();

      if (state.currentMode < 4) {
        if (state.rhythmPatterns[state.currentMode][note]) {
          activateSolenoid(note);
        }
      } else {
        if (now - state.lastNoteTime < NOTE_GROUPING_WINDOW) {
          state.notesInWindow++;
        } else {
          state.notesInWindow = 1;
        }
        state.lastNoteTime = now;

        bool isPhysicalPress = state.physicalKeyPressed[note] && 
                              (now - state.physicalKeyPressTime[note] < PHYSICAL_KEY_DETECTION_WINDOW);
        bool isRecordingPlayback = (state.notesInWindow >= RECORDING_PLAYBACK_NOTE_THRESHOLD) ||
                                  (state.notesInWindow >= 2 && 
                                   (now - state.lastNoteTime) < RECORDING_PLAYBACK_TIME_THRESHOLD);

        if (isPhysicalPress || isRecordingPlayback || state.appRecording) {
          uint8_t led1 = MATRIX_TO_LED[note][0];
          uint8_t led2 = MATRIX_TO_LED[note][1];
          activateSolenoid(note);
          state.ledBrightness[led1] = BRIGHTNESS_ACTIVE;
          state.ledBrightness[led2] = BRIGHTNESS_ACTIVE;
        }
      }
    } 
    else if (command == 0x80 && value.length() >= 4) {
      uint8_t note = value[3];
      if (note >= 16) return;

      if (state.currentMode >= 4) {
        uint8_t led1 = MATRIX_TO_LED[note][0];
        uint8_t led2 = MATRIX_TO_LED[note][1];
        state.ledBrightness[led1] = 0;
        state.ledBrightness[led2] = 0;
      }
    }
    else if (command == 0xB0 && value.length() >= 5) {
      uint8_t control = value[3];
      uint8_t val = value[4];

      if (control >= 16 && control < 32) {
        uint8_t keyIndex = control - 16;
        if (keyIndex < 16 && state.currentMode < 4) {
          state.rhythmPatterns[state.currentMode][keyIndex] = (val > 0);
        }
      } 
      else if (control == 0x40) {
        bool wasPlaying = state.isPlaying;
        state.isPlaying = val > 0;
        if (state.isPlaying && !wasPlaying) {
          state.currentStep = 0;
          state.lastStepTime = millis();
          memset(state.indicatorActive, false, sizeof(state.indicatorActive));
          Serial.println("Playback started");
        } else if (!state.isPlaying && wasPlaying) {
          Serial.println("Playback stopped");
        }
      } 
      else if (control == 0x41) {
        state.bpm = constrain(60 + val, 60, 200);
        Serial.printf("BPM: %d\n", state.bpm);
      } 
      else if (control == 0x42) {
        memset(state.rhythmPatterns, false, sizeof(state.rhythmPatterns));
        clearAllLEDs();
        Serial.println("All patterns reset");
      } 
      else if (control < 8) {
        state.currentMode = control;
        clearAllLEDs();
        setModeColor(state.currentMode);
        Serial.printf("Mode: %s\n", PART_NAMES[control]);
      }
      else if (control == CC_STATE_REQUEST) {
        Serial.println("State request received");
        startStateSync();
      }
      else if (control == CC_HEARTBEAT) {
        state.lastHeartbeatReceived = millis();
      }
      else if (control == CC_RECORDING_MODE) {
        state.appRecording = (val > 0);
        Serial.printf("App recording: %s\n", state.appRecording ? "ON" : "OFF");
      }
    }
  }
};

// ========================================
// 同期キュー管理
// ========================================

void queueSyncMessage(uint8_t control, uint8_t value) {
  uint8_t nextTail = (syncQueueTail + 1) % 128;
  if (nextTail == syncQueueHead) {
    Serial.println("Sync queue full!");
    return;
  }
  
  syncQueue[syncQueueTail].control = control;
  syncQueue[syncQueueTail].value = value;
  syncQueue[syncQueueTail].sendTime = millis() + (syncQueueTail * 30);
  syncQueueTail = nextTail;
}

void processSyncQueue() {
  if (syncQueueHead == syncQueueTail) {
    if (isSyncing) {
      isSyncing = false;
      Serial.println("State sync complete");
    }
    return;
  }
  
  unsigned long now = millis();
  if (now >= syncQueue[syncQueueHead].sendTime) {
    sendMIDIOverBLE(0xB0, 
                    syncQueue[syncQueueHead].control,
                    syncQueue[syncQueueHead].value);
    syncQueueHead = (syncQueueHead + 1) % 128;
  }
}

void startStateSync() {
  syncQueueHead = 0;
  syncQueueTail = 0;
  isSyncing = true;
  
  queueSyncMessage(CC_STATE_SYNC_START, 127);
  queueSyncMessage(state.currentMode, 127);
  queueSyncMessage(0x41, state.bpm - 60);
  queueSyncMessage(0x40, state.isPlaying ? 127 : 0);
  
  // 全パターンを送信
  for (int mode = 0; mode < 4; mode++) {
    for (int step = 0; step < 16; step++) {
      if (state.rhythmPatterns[mode][step]) {
        uint8_t packedData = (mode << 4) | step;
        queueSyncMessage(CC_PATTERN_DATA, packedData);
      }
    }
  }
  
  Serial.println("State sync started");
}

void sendHeartbeat() {
  unsigned long now = millis();
  
  if (now - state.lastHeartbeatSent >= HEARTBEAT_INTERVAL) {
    state.lastHeartbeatSent = now;
    sendMIDIOverBLE(0xB0, CC_HEARTBEAT, 127);
  }
}

// ========================================
// ソレノイド管理
// ========================================

bool activateSolenoid(uint8_t note) {
  if (note >= 16) return false;
  if (state.activeSolenoidCount >= MAX_SIMULTANEOUS_SOLENOIDS) {
#if DEBUG
    Serial.println("Max solenoids reached");
#endif
    return false;
  }
  
  uint8_t solPin = MATRIX_TO_SOLENOID[note];
  digitalWrite(solPin, HIGH);
  state.solenoidOffTime[note] = millis() + SOLENOID_PULSE_MS;
  state.activeSolenoidCount++;
  return true;
}

void deactivateSolenoid(uint8_t note) {
  if (note >= 16) return;
  uint8_t solPin = MATRIX_TO_SOLENOID[note];
  digitalWrite(solPin, LOW);
  state.solenoidOffTime[note] = 0;
  if (state.activeSolenoidCount > 0) {
    state.activeSolenoidCount--;
  }
}

// ========================================
// LED制御
// ========================================

void setModeColor(uint8_t mode) {
  if (mode >= 8) return;
  // モード切替時は色を変更せず、現在の状態を維持
  // LED更新はupdateLEDs()で行われる
}

void clearAllLEDs() {
  memset(state.ledBrightness, 0, sizeof(state.ledBrightness));
  memset(state.indicatorActive, false, sizeof(state.indicatorActive));
}

uint8_t calculateFadeBrightness(unsigned long elapsed, unsigned long duration,
                                uint8_t baseBrightness, uint8_t targetBrightness) {
  if (duration == 0) return targetBrightness;

  unsigned long fadeInTime = duration * FADE_IN_PERCENT / 100;
  unsigned long fadeOutStart = duration * (100 - FADE_OUT_PERCENT) / 100;

  if (elapsed < fadeInTime) {
    return baseBrightness + (uint32_t)(targetBrightness - baseBrightness) * elapsed / fadeInTime;
  } else if (elapsed < fadeOutStart) {
    return targetBrightness;
  } else if (elapsed < duration) {
    uint32_t fadeProgress = elapsed - fadeOutStart;
    uint32_t fadeTime = duration - fadeOutStart;
    return targetBrightness - (uint32_t)(targetBrightness - baseBrightness) * fadeProgress / fadeTime;
  }
  return baseBrightness;
}

void updateLEDs() {
  static uint32_t lastColors[LED_COUNT] = {0};
  unsigned long now = millis();
  bool needsUpdate = false;

  // BLE未接続時のアドバタイズ表示
  if (!deviceConnected) {
    // 1秒周期で内側と四隅を交互に点灯
    bool showInner = ((now / 1000) % 2) == 0;
    
    // まず全てのLEDをOFFに
    for (int i = 0; i < LED_COUNT; i++) {
      leds.setPixelColor(i, 0);
    }
    
    for (int i = 0; i < LED_COUNT; i++) {
      // このLEDが内側4つまたは四隅4つに該当するかチェック
      for (int k = 0; k < 16; k++) {
        if (MATRIX_TO_LED[k][0] == i || MATRIX_TO_LED[k][1] == i) {
          uint8_t solenoidNum = MATRIX_TO_SOLENOID[k];
          bool shouldLight = false;
          
          // 内側4つのチェック
          if (showInner) {
            for (int j = 0; j < 4; j++) {
              if (solenoidNum == ADVERTISE_INNER[j]) {
                shouldLight = true;
                break;
              }
            }
          } 
          // 四隅4つのチェック
          else {
            for (int j = 0; j < 4; j++) {
              if (solenoidNum == ADVERTISE_CORNERS[j]) {
                shouldLight = true;
                break;
              }
            }
          }
          
          if (shouldLight) {
            uint8_t r = ((ADVERTISE_COLOR >> 16) & 0xFF) * ADVERTISE_BRIGHTNESS / 255;
            uint8_t g = ((ADVERTISE_COLOR >> 8) & 0xFF) * ADVERTISE_BRIGHTNESS / 255;
            uint8_t b = (ADVERTISE_COLOR & 0xFF) * ADVERTISE_BRIGHTNESS / 255;
            leds.setPixelColor(i, leds.Color(r, g, b));
          }
          break;
        }
      }
    }
    
    leds.show();
    return;
  }

  // 以下、接続時の通常処理
  if (state.isPlaying) {
    unsigned long stepDuration = 60000UL / state.bpm / 4;
    if (now - state.lastStepTime >= stepDuration) {
      state.lastStepTime = now;
      memset(state.indicatorActive, false, sizeof(state.indicatorActive));
      state.indicatorActive[state.currentStep] = true;
      state.indicatorStartTime[state.currentStep] = now;
      
      // ドラムモード(0-3)の場合、パターンに従ってソレノイドを駆動
      if (state.currentMode < 4) {
        if (state.rhythmPatterns[state.currentMode][state.currentStep]) {
          activateSolenoid(state.currentStep);
        }
      }
      
      if (deviceConnected && !isSyncing) {
        sendMIDIOverBLE(0xB0, CC_BAR_SYNC, state.currentStep);
      }
      
      state.currentStep = (state.currentStep + 1) % 16;
    }
  }

  for (int i = 0; i < LED_COUNT; i++) {
    uint32_t color = MODE_COLORS[state.currentMode];
    uint8_t brightness = 0;

    int keyIndex = -1;
    for (int k = 0; k < 16; k++) {
      if (MATRIX_TO_LED[k][0] == i || MATRIX_TO_LED[k][1] == i) {
        keyIndex = k;
        break;
      }
    }

    if (keyIndex >= 0) {
      if (state.currentMode < 4) {
        if (state.rhythmPatterns[state.currentMode][keyIndex]) {
          brightness = BRIGHTNESS_TOGGLE_ON;
        }
        
        if (state.isPlaying && state.indicatorActive[keyIndex]) {
          unsigned long stepDuration = 60000UL / state.bpm / 4;
          unsigned long elapsed = now - state.indicatorStartTime[keyIndex];
          uint8_t targetBrightness = state.rhythmPatterns[state.currentMode][keyIndex] ? 
                                   BRIGHTNESS_ACTIVE : BRIGHTNESS_INDICATOR;
          brightness = calculateFadeBrightness(elapsed, stepDuration, brightness, targetBrightness);
        }
      } else {
        if (state.isPlaying && state.indicatorActive[keyIndex]) {
          unsigned long stepDuration = 60000UL / state.bpm / 4;
          unsigned long elapsed = now - state.indicatorStartTime[keyIndex];
          brightness = calculateFadeBrightness(elapsed, stepDuration, 0, BRIGHTNESS_INDICATOR_SYNTH);
        }
        
        if (state.ledBrightness[i] > brightness) {
          brightness = state.ledBrightness[i];
        }
      }
    }

    uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
    uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
    uint8_t b = (color & 0xFF) * brightness / 255;

    uint32_t newColor = leds.Color(r, g, b);
    if (lastColors[i] != newColor) {
      leds.setPixelColor(i, newColor);
      lastColors[i] = newColor;
      needsUpdate = true;
    }
  }

  if (needsUpdate) {
    leds.show();
  }
}

// ========================================
// キースキャン
// ========================================

void checkMultiPress() {
  bool centerPressed = true;
  bool cornerPressed = true;

  for (int i = 0; i < 4; i++) {
    if (!state.matrixKeys[CENTER_KEYS[i]]) centerPressed = false;
    if (!state.directKeys[CORNER_KEYS[i]]) cornerPressed = false;
  }

  unsigned long now = millis();

  if ((centerPressed || cornerPressed) && !state.multiPressDetected) {
    state.multiPressDetected = true;
    state.multiPressTimer = now;

    if (centerPressed) {
      state.multiPressType = 1;
      if (state.currentMode < 4) {
        memset(state.rhythmPatterns[state.currentMode], false, 16);
      }
    } else if (cornerPressed) {
      state.multiPressType = 2;
      memset(state.rhythmPatterns, false, sizeof(state.rhythmPatterns));
      clearAllLEDs();
      
      // アプリ側にもリセット通知を送信
      if (deviceConnected && !isSyncing) {
        sendMIDIOverBLE(0xB0, 0x42, 127);
      }
      
      Serial.println("Full reset triggered (patterns cleared, app notified)");
    }
  }

  if (state.multiPressDetected) {
    bool shouldRelease = false;
    if (state.multiPressType == 1 && !centerPressed) shouldRelease = true;
    else if (state.multiPressType == 2 && !cornerPressed) shouldRelease = true;
    else if (now - state.multiPressTimer > MULTI_PRESS_TIMEOUT) shouldRelease = true;

    if (shouldRelease) {
      state.multiPressDetected = false;
      state.multiPressType = 0;
    }
  }
}

void scanKeys() {
  static bool lastMatrixState[16] = {false};
  static bool lastDirectState[8] = {false};
  static unsigned long lastScanTime = 0;
  unsigned long now = millis();

  if (now - lastScanTime < KEY_SCAN_INTERVAL) return;
  lastScanTime = now;

  bool currentMatrixState[16] = {false};

  for (int row = 0; row < 4; row++) {
    for (int r = 0; r < 4; r++) {
      mcp.digitalWrite(rowPins[r], HIGH);
    }
    mcp.digitalWrite(rowPins[row], LOW);
    delayMicroseconds(50);

    for (int col = 0; col < 4; col++) {
      int keyIndex = row * 4 + col;
      currentMatrixState[keyIndex] = !mcp.digitalRead(colPins[col]);
    }
  }

  for (int r = 0; r < 4; r++) {
    mcp.digitalWrite(rowPins[r], HIGH);
  }

  for (int i = 0; i < 16; i++) {
    if (currentMatrixState[i] != lastMatrixState[i]) {
      lastMatrixState[i] = currentMatrixState[i];
      state.matrixKeys[i] = currentMatrixState[i];

      if (currentMatrixState[i] && !state.multiPressDetected) {
        state.physicalKeyPressed[i] = true;
        state.physicalKeyPressTime[i] = now;

        if (state.currentMode < 4) {
          state.rhythmPatterns[state.currentMode][i] = !state.rhythmPatterns[state.currentMode][i];
          if (deviceConnected && !isSyncing) {
            sendMIDIOverBLE(0xB0, 16 + i, state.rhythmPatterns[state.currentMode][i] ? 127 : 0);
          }
        } else {
          if (deviceConnected && !isSyncing) {
            sendMIDIOverBLE(0x90, i, 127);
          }
          uint8_t led1 = MATRIX_TO_LED[i][0];
          uint8_t led2 = MATRIX_TO_LED[i][1];
          state.ledBrightness[led1] = BRIGHTNESS_ACTIVE;
          state.ledBrightness[led2] = BRIGHTNESS_ACTIVE;
        }
      } else if (!currentMatrixState[i]) {
        state.physicalKeyPressed[i] = false;
        if (state.currentMode >= 4) {
          if (deviceConnected && !isSyncing) {
            sendMIDIOverBLE(0x80, i, 0);
          }
          uint8_t led1 = MATRIX_TO_LED[i][0];
          uint8_t led2 = MATRIX_TO_LED[i][1];
          state.ledBrightness[led1] = 0;
          state.ledBrightness[led2] = 0;
        }
      }
    }
  }

  bool currentDirectState[8];
  for (int i = 0; i < 7; i++) {
    currentDirectState[i] = !mcp.digitalRead(directPins[i]);
  }
  currentDirectState[7] = !digitalRead(ESP_KEY_PIN);

  for (int i = 0; i < 8; i++) {
    if (currentDirectState[i] != lastDirectState[i]) {
      lastDirectState[i] = currentDirectState[i];
      state.directKeys[i] = currentDirectState[i];

      if (currentDirectState[i] && !state.multiPressDetected) {
        // キーが押された: 長押しタイマー開始 & 即座にモード切替
        state.directKeyPressTime[i] = now;
        state.directKeyLongPressHandled[i] = false;
        
        // シンセモード(4-7)の場合のみ、押した瞬間にモード切替を実行
        uint8_t targetMode = (i < 7) ? directPinToMode[i] : 0;
        if (targetMode >= 4) {
          // 即座にモード切替
          if (deviceConnected && !isSyncing) {
            sendMIDIOverBLE(0xB0, targetMode, 127);
          }
          state.currentMode = targetMode;
          clearAllLEDs();
          setModeColor(state.currentMode);
        } else {
          // ドラムモード(0-3)も即座に切替
          if (deviceConnected && !isSyncing) {
            sendMIDIOverBLE(0xB0, targetMode, 127);
          }
          state.currentMode = targetMode;
          clearAllLEDs();
          setModeColor(state.currentMode);
        }
        
      } else if (!currentDirectState[i]) {
        // キーが離された
        // 長押しされていた場合は録音モード終了を通知
        if (state.directKeyLongPressHandled[i]) {
          uint8_t targetMode = (i < 7) ? directPinToMode[i] : 0;
          if (targetMode >= 4 && deviceConnected && !isSyncing) {
            // CC_RECORDING_MODE (0x53)で録音終了を通知
            sendMIDIOverBLE(0xB0, 0x53, 0);  // value=0で録音終了
            Serial.printf("Long press released: Stop recording mode %d\n", targetMode);
          }
        }
        state.directKeyPressTime[i] = 0;
        state.directKeyLongPressHandled[i] = false;
      }
    } 
    // 長押し判定
    else if (currentDirectState[i] && !state.directKeyLongPressHandled[i]) {
      if (now - state.directKeyPressTime[i] >= LONG_PRESS_DURATION) {
        state.directKeyLongPressHandled[i] = true;
        
        uint8_t targetMode = (i < 7) ? directPinToMode[i] : 0;
        
        // シンセモード(4-7)の場合のみ録音モード開始
        if (targetMode >= 4) {
          if (deviceConnected && !isSyncing) {
            // CC_RECORDING_MODE (0x53)で録音開始を通知
            sendMIDIOverBLE(0xB0, 0x53, targetMode);  // valueにモード番号を送信
          }
          Serial.printf("Long press detected: Start recording mode %d\n", targetMode);
        }
      }
    }
  }

  checkMultiPress();
}

// ========================================
// MIDI送信
// ========================================

void sendMIDIOverBLE(uint8_t status, uint8_t data1, uint8_t data2) {
  if (!deviceConnected || !pCharacteristic) return;

  uint8_t packet[5];
  packet[0] = 0x80;
  packet[1] = (millis() & 0x7F) | 0x80;
  packet[2] = status;
  packet[3] = data1;

  int length = 4;
  if ((status & 0xF0) == 0x90 || (status & 0xF0) == 0xB0) {
    packet[4] = data2;
    length = 5;
  }

  pCharacteristic->setValue(packet, length);
  pCharacteristic->notify();
}

void updateSolenoids() {
  unsigned long now = millis();
  for (int i = 0; i < 16; i++) {
    if (state.solenoidOffTime[i] > 0 && now >= state.solenoidOffTime[i]) {
      deactivateSolenoid(i);
    }
  }
}

// ========================================
// Setup & Loop
// ========================================

void setup() {
  Serial.begin(115200);
  Serial.println("Ponkey starting...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mcp.begin_I2C(MCP_ADDR, &Wire)) {
    Serial.println("MCP23017 not found!");
    while (1) delay(10);
  }

  for (int i = 0; i < 4; i++) {
    mcp.pinMode(colPins[i], INPUT_PULLUP);
  }
  for (int i = 0; i < 4; i++) {
    mcp.pinMode(rowPins[i], OUTPUT);
    mcp.digitalWrite(rowPins[i], HIGH);
  }
  for (int i = 0; i < 7; i++) {
    mcp.pinMode(directPins[i], INPUT_PULLUP);
  }
  pinMode(ESP_KEY_PIN, INPUT_PULLUP);

  for (int i = 0; i < 16; i++) {
    pinMode(MATRIX_TO_SOLENOID[i], OUTPUT);
    digitalWrite(MATRIX_TO_SOLENOID[i], LOW);
  }

  leds.begin();
  leds.setBrightness(255);
  leds.clear();
  leds.show();
  setModeColor(state.currentMode);

  BLEDevice::init("Ponkey");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE_NR
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  Serial.println("=== Ponkey v2.5 - Long Press Looper ===");
  Serial.println("Ponkey ready!");
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  // 接続後500ms待機してから状態同期開始
  if (needsStateSync && (now - connectionStartTime >= STATE_SYNC_DELAY)) {
    needsStateSync = false;
    startStateSync();
  }
  
  processSyncQueue();
  scanKeys();
  
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;
    updateLEDs();
    updateSolenoids();
  }
  
  if (deviceConnected) {
    sendHeartbeat();
  }
}