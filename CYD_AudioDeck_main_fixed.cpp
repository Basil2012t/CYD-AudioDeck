#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutput.h>
#include <BluetoothA2DPSource.h>
#include <esp_gap_bt_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// CYD AudioDeck - ESP32-2432S028 (2.8-inch CYD)
// Read-only MP3 library from microSD -> Bluetooth A2DP headphones.
// The SD card is mounted for reading only: this firmware does not format,
// create, delete, rename, or write files to the card.

// Common CYD pin map
static constexpr int PIN_SD_CS      = 5;
static constexpr int PIN_SD_SCK     = 18;
static constexpr int PIN_SD_MISO    = 19;
static constexpr int PIN_SD_MOSI    = 23;
static constexpr int PIN_TFT_BL     = 21;
static constexpr int PIN_TOUCH_CS   = 33;
static constexpr int PIN_TOUCH_IRQ  = 36;
static constexpr int PIN_TOUCH_CLK  = 25;
static constexpr int PIN_TOUCH_MISO = 39;
static constexpr int PIN_TOUCH_MOSI = 32;

// Adjust only if touch coordinates are offset on a particular panel.
static constexpr int TOUCH_MIN_X = 300;
static constexpr int TOUCH_MAX_X = 3900;
static constexpr int TOUCH_MIN_Y = 300;
static constexpr int TOUCH_MAX_Y = 3900;

static constexpr int SCREEN_W = 240;
static constexpr int SCREEN_H = 320;
static constexpr int MAX_TRACKS = 150;
static constexpr int MAX_BT_DEVICES = 10;
static constexpr int BT_NAME_LEN = 36;
static constexpr size_t BT_RING_FRAMES = 4096;

// Dark navy / cyan theme
static constexpr uint16_t C_BG      = 0x0841;
static constexpr uint16_t C_PANEL   = 0x10A2;
static constexpr uint16_t C_PANEL2  = 0x18E3;
static constexpr uint16_t C_TEXT    = 0xFFFF;
static constexpr uint16_t C_MUTED   = 0x9CF3;
static constexpr uint16_t C_ACCENT  = 0x46FF;
static constexpr uint16_t C_PURPLE  = 0xA5FF;
static constexpr uint16_t C_GREEN   = 0x5F6C;
static constexpr uint16_t C_RED     = 0xF986;

TFT_eSPI tft;
SPIClass touchSPI(HSPI);
XPT2046_Touchscreen touch(PIN_TOUCH_CS, PIN_TOUCH_IRQ);

static BluetoothA2DPSource a2dp;

struct Track {
  char path[120];
  char title[72];
  uint32_t bytes;
};
static Track tracks[MAX_TRACKS];
static int trackCount = 0;
static int currentTrack = -1;
static int pendingTrack = -1;
static int listPage = 0;
static bool isPaused = false;
static bool isPlaying = false;
static volatile bool btConnected = false;
static bool userSelectedBt = false;
static bool screenSleeping = false;
static unsigned long lastTouchMs = 0;
static unsigned long lastUiMs = 0;
static unsigned long playStartedMs = 0;
static unsigned long pausedAtMs = 0;
static unsigned long pausedAccumMs = 0;

enum Screen { SCREEN_LIBRARY, SCREEN_PLAYER, SCREEN_BLUETOOTH };
static Screen screen = SCREEN_LIBRARY;

struct BtDevice {
  char name[BT_NAME_LEN];
  uint8_t addr[ESP_BD_ADDR_LEN];
  int rssi;
};
static BtDevice btDevices[MAX_BT_DEVICES];
static volatile int btDeviceCount = 0;
static portMUX_TYPE btMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t chosenBtAddr[ESP_BD_ADDR_LEN] = {0};

static int16_t btRing[BT_RING_FRAMES * 2];
static volatile size_t ringHead = 0;
static volatile size_t ringTail = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

static AudioFileSourceSD *audioFile = nullptr;
static AudioGeneratorMP3 *mp3 = nullptr;

static void clearBtRing() {
  portENTER_CRITICAL(&ringMux);
  ringHead = 0;
  ringTail = 0;
  portEXIT_CRITICAL(&ringMux);
}

class AudioDeckOutput : public AudioOutput {
public:
  float gain = 0.78f;

  AudioDeckOutput() {
    hertz = 44100;
    channels = 2;
    gainF2P6 = 50;
  }

  bool SetRate(int hz) override { hertz = hz; return true; }
  bool SetChannels(int ch) override { channels = ch; return true; }

  bool SetGain(float value) override {
    gain = constrain(value, 0.0f, 1.0f);
    gainF2P6 = (uint8_t)(gain * 64.0f);
    return true;
  }

  bool begin() override { return true; }

  bool ConsumeSample(int16_t sample[2]) override {
    if (!btConnected || isPaused) return true;

    int32_t left = (int32_t)((float)sample[0] * gain);
    int32_t right = (int32_t)((float)sample[1] * gain);
    left = constrain(left, -32768, 32767);
    right = constrain(right, -32768, 32767);

    portENTER_CRITICAL(&ringMux);
    size_t next = (ringHead + 1) % BT_RING_FRAMES;
    if (next == ringTail) ringTail = (ringTail + 1) % BT_RING_FRAMES;
    btRing[ringHead * 2] = (int16_t)left;
    btRing[ringHead * 2 + 1] = (int16_t)right;
    ringHead = next;
    portEXIT_CRITICAL(&ringMux);
    return true;
  }

  bool stop() override { return true; }
  void flush() override { clearBtRing(); }
};
static AudioDeckOutput audioOut;

static int32_t btStreamCallback(Frame *frames, int32_t count) {
  if (!frames) return 0;
  portENTER_CRITICAL(&ringMux);
  for (int32_t i = 0; i < count; ++i) {
    if (isPaused || ringTail == ringHead) {
      frames[i].channel1 = 0;
      frames[i].channel2 = 0;
    } else {
      frames[i].channel1 = btRing[ringTail * 2];
      frames[i].channel2 = btRing[ringTail * 2 + 1];
      ringTail = (ringTail + 1) % BT_RING_FRAMES;
    }
  }
  portEXIT_CRITICAL(&ringMux);
  return count;
}

static bool sameAddress(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}

// Discovery callback: collect nearby devices for the Bluetooth picker.
// Returning false keeps scanning; the selected device is connected directly
// when the user taps it in the list.
static bool btScanCallback(const char *name, esp_bd_addr_t address, int rssi) {
  portENTER_CRITICAL(&btMux);
  int found = -1;
  for (int i = 0; i < btDeviceCount; ++i) {
    if (sameAddress(btDevices[i].addr, address)) { found = i; break; }
  }
  if (found < 0 && btDeviceCount < MAX_BT_DEVICES) found = btDeviceCount++;
  if (found >= 0) {
    BtDevice &d = btDevices[found];
    memset(&d, 0, sizeof(d));
    const char *label = (name && name[0]) ? name : "Unknown audio device";
    strncpy(d.name, label, sizeof(d.name) - 1);
    memcpy(d.addr, address, ESP_BD_ADDR_LEN);
    d.rssi = rssi;
  }
  portEXIT_CRITICAL(&btMux);
  return false;
}

static int btCountSnapshot() {
  portENTER_CRITICAL(&btMux);
  int count = btDeviceCount;
  portEXIT_CRITICAL(&btMux);
  return count;
}

static bool readTouch(int &x, int &y) {
  if (!touch.touched()) return false;
  TS_Point p = touch.getPoint();
  x = map(p.x, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_W - 1);
  y = map(p.y, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_H - 1);
  x = constrain(x, 0, SCREEN_W - 1);
  y = constrain(y, 0, SCREEN_H - 1);
  return true;
}

static void drawHeader(const char *title, bool showControls = true) {
  tft.fillRect(0, 0, SCREEN_W, 42, C_PANEL);
  tft.setTextColor(C_TEXT, C_PANEL);
  tft.setTextSize(2);
  tft.setCursor(10, 13);
  tft.print(title);

  if (showControls) {
    uint16_t btColor = btConnected ? C_GREEN : C_PANEL2;
    tft.fillRoundRect(151, 7, 38, 28, 7, btColor);
    tft.fillRoundRect(195, 7, 36, 28, 7, C_PANEL2);
    tft.setTextColor(C_TEXT, btColor);
    tft.setTextSize(1);
    tft.setCursor(161, 18);
    tft.print("BT");
    tft.setTextColor(C_ACCENT, C_PANEL2);
    tft.setCursor(204, 18);
    tft.print("Zz");
  } else {
    tft.fillRoundRect(194, 7, 38, 28, 7, C_PANEL2);
    tft.setTextColor(C_ACCENT, C_PANEL2);
    tft.setTextSize(1);
    tft.setCursor(201, 18);
    tft.print("BACK");
  }
}

static void drawLibrary() {
  tft.fillScreen(C_BG);
  drawHeader("My Music");
  tft.setTextColor(C_MUTED, C_BG);
  tft.setTextSize(1);
  tft.setCursor(12, 51);
  tft.printf("%d MP3 files  |  %s", trackCount, btConnected ? "Bluetooth ready" : "Connect headphones");

  if (trackCount == 0) {
    tft.fillRoundRect(12, 100, 216, 104, 12, C_PANEL);
    tft.setTextColor(C_TEXT, C_PANEL);
    tft.setTextSize(2);
    tft.setCursor(42, 128);
    tft.print("No MP3 files");
    tft.setTextColor(C_MUTED, C_PANEL);
    tft.setTextSize(1);
    tft.setCursor(26, 160);
    tft.print("Keep songs in the SD card");
  } else {
    const int first = listPage * 8;
    const int last = min(trackCount, first + 8);
    for (int i = first; i < last; ++i) {
      int y = 70 + (i - first) * 27;
      uint16_t bg = (i == currentTrack) ? C_PANEL2 : C_PANEL;
      tft.fillRoundRect(8, y, 224, 23, 6, bg);
      tft.setTextColor((i == currentTrack) ? C_ACCENT : C_TEXT, bg);
      tft.setTextSize(1);
      tft.setCursor(16, y + 8);
      String title = tracks[i].title;
      if (title.length() > 31) title = title.substring(0, 28) + "...";
      tft.print(title);
    }
  }

  tft.fillRoundRect(8, 292, 66, 22, 6, C_PANEL2);
  tft.fillRoundRect(166, 292, 66, 22, 6, C_PANEL2);
  tft.fillRoundRect(76, 292, 88, 22, 6, C_PANEL);
  tft.setTextColor(C_TEXT, C_PANEL2);
  tft.setTextSize(1);
  tft.setCursor(25, 299); tft.print("PREV");
  tft.setCursor(183, 299); tft.print("NEXT");
  tft.setTextColor(C_ACCENT, C_PANEL);
  tft.setCursor(91, 299); tft.print("NOW PLAYING");
  tft.setTextColor(C_MUTED, C_BG);
  tft.setCursor(98, 278);
  tft.printf("%d / %d", trackCount ? listPage + 1 : 0, max(1, (trackCount + 7) / 8));
}

static void formatTime(uint32_t seconds, char *out, size_t size) {
  snprintf(out, size, "%02lu:%02lu", (unsigned long)(seconds / 60), (unsigned long)(seconds % 60));
}

static uint32_t estimatedDuration() {
  if (currentTrack < 0 || currentTrack >= trackCount) return 0;
  return (uint32_t)((tracks[currentTrack].bytes * 8ULL) / 128000ULL);
}

static uint32_t elapsedSeconds() {
  if (!isPlaying && !isPaused) return 0;
  unsigned long elapsed = (isPaused ? pausedAtMs : millis()) - playStartedMs - pausedAccumMs;
  return elapsed / 1000;
}

static void drawPlayer() {
  tft.fillScreen(C_BG);
  drawHeader("Now Playing");
  if (currentTrack < 0 || currentTrack >= trackCount) {
    tft.setTextColor(C_MUTED, C_BG);
    tft.setTextSize(2);
    tft.setCursor(42, 140);
    tft.print("Choose a song");
    return;
  }

  tft.fillRoundRect(24, 62, 192, 112, 14, C_PANEL);
  tft.drawRoundRect(24, 62, 192, 112, 14, C_PANEL2);
  tft.fillCircle(120, 118, 34, C_BG);
  tft.drawCircle(120, 118, 34, C_ACCENT);
  tft.drawCircle(120, 118, 8, C_PANEL2);
  for (int i = 0; i < 7; ++i) {
    int h = 10 + ((i * 13 + 7) % 28);
    tft.fillRoundRect(58 + i * 20, 178 - h, 8, h, 3, (i % 2) ? C_PURPLE : C_ACCENT);
  }

  tft.setTextColor(C_TEXT, C_BG);
  tft.setTextSize(2);
  String title = tracks[currentTrack].title;
  if (title.length() > 20) title = title.substring(0, 20) + "...";
  int titleW = title.length() * 12;
  tft.setCursor(max(8, (SCREEN_W - titleW) / 2), 205);
  tft.print(title);

  uint32_t duration = estimatedDuration();
  uint32_t elapsed = elapsedSeconds();
  if (duration > 0 && elapsed > duration) elapsed = duration;
  char left[12], right[12];
  formatTime(elapsed, left, sizeof(left));
  formatTime(duration, right, sizeof(right));
  tft.fillRoundRect(16, 240, 208, 6, 3, C_PANEL2);
  if (duration > 0) {
    int width = (int)((208ULL * elapsed) / duration);
    tft.fillRoundRect(16, 240, min(208, width), 6, 3, C_ACCENT);
  }
  tft.setTextColor(C_MUTED, C_BG);
  tft.setTextSize(1);
  tft.setCursor(16, 252); tft.print(left);
  tft.setCursor(196, 252); tft.print(right);

  tft.fillRoundRect(18, 270, 56, 40, 9, C_PANEL);
  tft.fillRoundRect(92, 264, 56, 46, 10, C_PANEL2);
  tft.fillRoundRect(166, 270, 56, 40, 9, C_PANEL);
  tft.setTextColor(C_TEXT, C_PANEL);
  tft.setTextSize(2);
  tft.setCursor(36, 282); tft.print("<<");
  tft.setTextColor(C_ACCENT, C_PANEL2);
  tft.setCursor(108, 280); tft.print(isPaused ? ">" : "||");
  tft.setTextColor(C_TEXT, C_PANEL);
  tft.setCursor(184, 282); tft.print(">>");
}

static void drawBluetoothScreen() {
  tft.fillScreen(C_BG);
  drawHeader("Bluetooth", false);
  tft.setTextColor(C_MUTED, C_BG);
  tft.setTextSize(1);
  tft.setCursor(12, 51);
  tft.print(btConnected ? "Headphones connected" : "Turn on headphones and make them discoverable");

  int count = btCountSnapshot();
  if (count == 0) {
    tft.fillRoundRect(12, 91, 216, 100, 12, C_PANEL);
    tft.setTextColor(C_TEXT, C_PANEL);
    tft.setTextSize(2);
    tft.setCursor(44, 119);
    tft.print("Searching...");
    tft.setTextColor(C_MUTED, C_PANEL);
    tft.setTextSize(1);
    tft.setCursor(29, 151);
    tft.print("Keep your headphones nearby");
  } else {
    int visible = min(count, 7);
    for (int i = 0; i < visible; ++i) {
      BtDevice copy;
      portENTER_CRITICAL(&btMux);
      copy = btDevices[i];
      portEXIT_CRITICAL(&btMux);
      int y = 76 + i * 27;
      tft.fillRoundRect(8, y, 224, 23, 6, C_PANEL);
      tft.setTextColor(C_TEXT, C_PANEL);
      tft.setTextSize(1);
      tft.setCursor(16, y + 8);
      String name = copy.name;
      if (name.length() > 27) name = name.substring(0, 24) + "...";
      tft.print(name);
      tft.setTextColor(C_ACCENT, C_PANEL);
      tft.setCursor(202, y + 8);
      tft.print(copy.rssi);
    }
  }

  tft.fillRoundRect(8, 292, 224, 22, 6, C_PANEL2);
  tft.setTextColor(C_TEXT, C_PANEL2);
  tft.setTextSize(1);
  tft.setCursor(90, 299);
  tft.print("BACK TO MUSIC");
}

static void addTrack(const char *path, const char *name, uint32_t bytes) {
  if (trackCount >= MAX_TRACKS) return;
  String lower = String(name);
  lower.toLowerCase();
  if (!lower.endsWith(".mp3")) return;

  Track &t = tracks[trackCount];
  strncpy(t.path, path, sizeof(t.path) - 1);
  t.path[sizeof(t.path) - 1] = '\0';
  String title = String(name);
  int slash = title.lastIndexOf('/');
  if (slash >= 0) title = title.substring(slash + 1);
  int dot = title.lastIndexOf('.');
  if (dot > 0) title = title.substring(0, dot);
  title.toCharArray(t.title, sizeof(t.title));
  t.bytes = bytes;
  ++trackCount;
}

// Recursively read directories. This routine only opens and reads entries.
static void scanDirectory(File dir, const String &prefix, int depth) {
  if (!dir || depth > 4) return;
  while (trackCount < MAX_TRACKS) {
    File file = dir.openNextFile();
    if (!file) break;
    String entry = String(file.name());
    String fullPath;
    if (entry.startsWith("/")) fullPath = entry;
    else if (prefix.length() == 0) fullPath = "/" + entry;
    else fullPath = prefix + "/" + entry;

    if (file.isDirectory()) {
      file.close();
      File child = SD.open(fullPath.c_str());
      if (child) {
        scanDirectory(child, fullPath, depth + 1);
        child.close();
      }
    } else {
      String titleCheck = entry;
      titleCheck.toLowerCase();
      if (titleCheck.endsWith(".mp3")) addTrack(fullPath.c_str(), entry.c_str(), file.size());
      file.close();
    }
  }
}

static void scanMusic() {
  trackCount = 0;
  File root = SD.open("/");
  if (!root) return;
  scanDirectory(root, "", 0);
  root.close();
  for (int i = 0; i < trackCount - 1; ++i) {
    for (int j = i + 1; j < trackCount; ++j) {
      if (strcasecmp(tracks[i].title, tracks[j].title) > 0) {
        Track tmp = tracks[i]; tracks[i] = tracks[j]; tracks[j] = tmp;
      }
    }
  }
}

static void stopPlayback() {
  if (mp3) {
    if (mp3->isRunning()) mp3->stop();
    delete mp3;
    mp3 = nullptr;
  }
  if (audioFile) {
    delete audioFile;
    audioFile = nullptr;
  }
  audioOut.stop();
  clearBtRing();
  isPlaying = false;
  isPaused = false;
  pausedAccumMs = 0;
}

static void openBluetoothScreen() {
  screen = SCREEN_BLUETOOTH;
  drawBluetoothScreen();
}

static bool playTrack(int index) {
  if (index < 0 || index >= trackCount) return false;
  if (!btConnected) {
    pendingTrack = index;
    openBluetoothScreen();
    return false;
  }

  stopPlayback();
  currentTrack = index;
  audioFile = new AudioFileSourceSD(tracks[index].path);
  if (!audioFile || !audioFile->isOpen()) {
    if (audioFile) { delete audioFile; audioFile = nullptr; }
    screen = SCREEN_LIBRARY;
    drawLibrary();
    return false;
  }
  mp3 = new AudioGeneratorMP3();
  if (!mp3 || !mp3->begin(audioFile, &audioOut)) {
    stopPlayback();
    screen = SCREEN_LIBRARY;
    drawLibrary();
    return false;
  }
  isPlaying = true;
  playStartedMs = millis();
  pausedAccumMs = 0;
  screen = SCREEN_PLAYER;
  drawPlayer();
  return true;
}

static void togglePause() {
  if (!mp3 || !isPlaying) {
    if (currentTrack >= 0) playTrack(currentTrack);
    else if (trackCount > 0) playTrack(0);
    return;
  }
  if (!isPaused) {
    isPaused = true;
    pausedAtMs = millis();
    clearBtRing();
  } else {
    pausedAccumMs += millis() - pausedAtMs;
    isPaused = false;
  }
  drawPlayer();
}

static void nextTrack() {
  if (trackCount == 0) return;
  int next = (currentTrack + 1 + trackCount) % trackCount;
  playTrack(next);
}

static void previousTrack() {
  if (trackCount == 0) return;
  int prev = (currentTrack - 1 + trackCount) % trackCount;
  playTrack(prev);
}

static void chooseBluetoothDevice(int index) {
  if (index < 0 || index >= btCountSnapshot()) return;
  portENTER_CRITICAL(&btMux);
  memcpy(chosenBtAddr, btDevices[index].addr, ESP_BD_ADDR_LEN);
  portEXIT_CRITICAL(&btMux);

  userSelectedBt = true;
  tft.fillScreen(C_BG);
  drawHeader("Bluetooth", false);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.setTextSize(2);
  tft.setCursor(42, 143);
  tft.print("Connecting...");
  esp_bd_addr_t target;
  memcpy(target, chosenBtAddr, ESP_BD_ADDR_LEN);
  bool result = a2dp.connect_to(target);
  if (!result) {
    tft.setTextColor(C_RED, C_BG);
    tft.setTextSize(1);
    tft.setCursor(24, 176);
    tft.print("Connection request failed");
  }
}

// Screen-off sleep: Bluetooth audio keeps playing; only the TFT backlight is
// switched off. A tap wakes the display. Full CPU sleep would interrupt A2DP.
static void enterDisplaySleep() {
  if (screenSleeping) return;
  screenSleeping = true;
  tft.fillScreen(C_BG);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.setTextSize(2);
  tft.setCursor(83, 126);
  tft.print("SLEEP");
  tft.setTextColor(C_MUTED, C_BG);
  tft.setTextSize(1);
  tft.setCursor(48, 158);
  tft.print(isPlaying && !isPaused ? "Music keeps playing" : "Tap screen to wake");
  delay(500);
  digitalWrite(PIN_TFT_BL, LOW);
}

static void wakeDisplay() {
  digitalWrite(PIN_TFT_BL, HIGH);
  delay(60);
  screenSleeping = false;
  if (screen == SCREEN_LIBRARY) drawLibrary();
  else if (screen == SCREEN_PLAYER) drawPlayer();
  else drawBluetoothScreen();
  lastUiMs = millis();
}

static void handleTouch() {
  int x, y;
  if (!readTouch(x, y)) return;
  unsigned long now = millis();
  if (now - lastTouchMs < 220) return;
  lastTouchMs = now;
  while (touch.touched()) delay(1);

  if (screenSleeping) {
    wakeDisplay();
    return;
  }

  if (screen == SCREEN_BLUETOOTH) {
    if (y >= 282 || (y < 42 && x >= 190)) {
      screen = SCREEN_LIBRARY;
      drawLibrary();
      return;
    }
    if (y >= 76 && y < 76 + 7 * 27) {
      int index = (y - 76) / 27;
      chooseBluetoothDevice(index);
    }
    return;
  }

  if (screen == SCREEN_LIBRARY) {
    if (y < 42 && x >= 195) { enterDisplaySleep(); return; }
    if (y < 42 && x >= 150) { openBluetoothScreen(); return; }
    if (y >= 292) {
      if (x < 75) listPage = max(0, listPage - 1);
      else if (x >= 166) listPage = min(max(0, (trackCount - 1) / 8), listPage + 1);
      else if (currentTrack >= 0) { screen = SCREEN_PLAYER; drawPlayer(); return; }
      drawLibrary();
      return;
    }
    if (y >= 70 && y < 286) {
      int index = listPage * 8 + (y - 70) / 27;
      if (index >= 0 && index < trackCount) playTrack(index);
    }
    return;
  }

  if (screen == SCREEN_PLAYER) {
    if (y < 42 && x >= 195) { enterDisplaySleep(); return; }
    if (y < 42 && x >= 150) { openBluetoothScreen(); return; }
    if (y < 42 && x < 80) { screen = SCREEN_LIBRARY; drawLibrary(); return; }
    if (y >= 264) {
      if (x < 82) previousTrack();
      else if (x < 158) togglePause();
      else nextTrack();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);

  // Mount card only. No formatting or write operations are used anywhere.
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, 20000000)) {
    drawHeader("AudioDeck");
    tft.setTextColor(C_RED, C_BG);
    tft.setTextSize(2);
    tft.setCursor(31, 118);
    tft.print("SD not readable");
    tft.setTextColor(C_MUTED, C_BG);
    tft.setTextSize(1);
    tft.setCursor(18, 151);
    tft.print("Card was not changed or formatted");
    while (true) {
      handleTouch();
      delay(20);
    }
  }

  touchSPI.begin(PIN_TOUCH_CLK, PIN_TOUCH_MISO, PIN_TOUCH_MOSI, PIN_TOUCH_CS);
  touch.begin(touchSPI);
  touch.setRotation(0);

  scanMusic();

  a2dp.set_auto_reconnect(true, 5); // reconnect to the last headphones on next boot
  a2dp.set_volume(127);
  a2dp.set_data_callback_in_frames(btStreamCallback);
  a2dp.set_ssid_callback(btScanCallback);
  a2dp.start();

  screen = SCREEN_LIBRARY; // Show the music library immediately after boot.
  drawLibrary();
  lastUiMs = millis();
}

void loop() {
  btConnected = a2dp.is_connected();

  if (screen == SCREEN_BLUETOOTH && userSelectedBt && btConnected) {
    userSelectedBt = false;
    screen = SCREEN_LIBRARY;
    drawLibrary();
    if (pendingTrack >= 0) {
      int toPlay = pendingTrack;
      pendingTrack = -1;
      playTrack(toPlay);
    }
  }

  if (isPlaying && !isPaused && mp3 && mp3->isRunning()) {
    int loops = 0;
    while (loops < 48 && mp3 && mp3->isRunning()) {
      size_t queued;
      portENTER_CRITICAL(&ringMux);
      queued = (ringHead >= ringTail) ? ringHead - ringTail : BT_RING_FRAMES - ringTail + ringHead;
      portEXIT_CRITICAL(&ringMux);
      if (queued > (BT_RING_FRAMES * 3 / 4)) break;
      if (!mp3->loop()) break;
      ++loops;
    }

    if (mp3 && !mp3->isRunning()) nextTrack();
  }

  if (!screenSleeping && screen == SCREEN_PLAYER && millis() - lastUiMs > 500) {
    drawPlayer();
    lastUiMs = millis();
  }

  if (!screenSleeping && screen == SCREEN_BLUETOOTH && millis() - lastUiMs > 1000) {
    drawBluetoothScreen();
    lastUiMs = millis();
  }

  handleTouch();
  delay(2);
}
