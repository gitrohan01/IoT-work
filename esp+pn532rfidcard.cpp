// PN532 Unified Test Sketch - I2C mode (ESP32)
// Features:
// - Detect PN532
// - Detect card UID and type (MIFARE Classic vs NTAG/NTAG2xx)
// - Read block/page
// - Write block/page (safe defaults)
// - Erase (write zeros)
// NOTE: Do NOT write block 0 (contains manufacturer UID) or sector trailers.

// Libraries
#include <Wire.h>
#include <Adafruit_PN532.h>

// I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Create PN532 instance (I2C)
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("==== PN532 Unified Test (ESP32) ====");
  Serial.println("Wiring: VCC=3.3V, GND=GND, SDA=21, SCL=22");
  Wire.begin(SDA_PIN, SCL_PIN);

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("ERROR: PN532 not found. Check wiring and I2C mode.");
    while (1) delay(1000);
  }
  Serial.print("Found PN532 chip: 0x"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver: "); Serial.print((versiondata >> 16) & 0xFF); Serial.print("."); Serial.println((versiondata >> 8) & 0xFF);
  nfc.SAMConfig();
  Serial.println("PN532 Ready. Place a card near the module.");
  Serial.println();
  printMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') return;
    handleMenu(c);
  }
}

// --- Menu UI ---
void printMenu() {
  Serial.println("[R] Read UID + detect card type");
  Serial.println("[B] Read data block / page");
  Serial.println("[W] Write (safe) block/page");
  Serial.println("[E] Erase block/page (write zeros)");
  Serial.println("[H] Help (this menu)");
  Serial.println();
  Serial.print("Enter: ");
}

void handleMenu(char c) {
  c = toupper(c);
  Serial.println();
  switch (c) {
    case 'R': doReadUID(); break;
    case 'B': doReadBlock(); break;
    case 'W': doWriteBlock(); break;
    case 'E': doEraseBlock(); break;
    case 'H': printMenu(); break;
    default: Serial.println("Unknown command"); printMenu();
  }
}

// --- Helpers ---
bool waitForCard(uint8_t *uid, uint8_t *uidLen, uint16_t timeoutMs=5000) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, uidLen)) {
      return true;
    }
    delay(50);
  }
  return false;
}

void printUID(uint8_t *uid, uint8_t uidLen) {
  Serial.print("UID: ");
  for (uint8_t i = 0; i < uidLen; i++) {
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// Try to detect simple card type using SAK/ATQA & probe
String detectTagType(uint8_t *uid, uint8_t uidLen) {
  // Try Mifare Classic authentication to check if classic:
  bool classic = false;
  // test authenticate block 4 with default key (not writing anything)
  uint8_t keyA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  if (nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 4, 0, keyA)) {
    classic = true;
  }
  if (classic) return "MIFARE Classic (likely)";
  // else try reading NTAG page 4 (ntag2xx)
  uint8_t data4[4];
  if (nfc.ntag2xx_ReadPage(4, data4)) return "NTAG/Ultralight (likely)";
  return "Unknown/Other";
}

// --- R: Read UID + detect type ---
void doReadUID() {
  Serial.println("Place card near PN532 (reading UID)...");
  uint8_t uid[7]; uint8_t uidLen;
  if (!waitForCard(uid, &uidLen, 8000)) {
    Serial.println("No card detected (timeout).");
    printMenu();
    return;
  }
  printUID(uid, uidLen);
  String t = detectTagType(uid, uidLen);
  Serial.print("Detected type: "); Serial.println(t);
  Serial.println();
  printMenu();
}

// --- B: Read block/page ---
void doReadBlock() {
  Serial.println("Read: Place card...");
  uint8_t uid[7]; uint8_t uidLen;
  if (!waitForCard(uid, &uidLen, 8000)) { Serial.println("No card."); printMenu(); return; }
  printUID(uid, uidLen);
  String t = detectTagType(uid, uidLen);
  Serial.print("Type: "); Serial.println(t);

  if (t.startsWith("MIFARE Classic")) {
    Serial.println("Reading MIFARE Classic block 4 (16 bytes)...");
    uint8_t data[16];
    uint8_t keyA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 4, 0, keyA)) {
      Serial.println("Auth failed for block 4!");
    } else {
      if (nfc.mifareclassic_ReadDataBlock(4, data)) {
        Serial.print("Block 4 hex: ");
        for (int i=0;i<16;i++){ if (data[i] < 0x10) Serial.print("0"); Serial.print(data[i], HEX); Serial.print(" "); }
        Serial.println();
        Serial.print("Block 4 ASCII: ");
        for (int i=0;i<16;i++){ if (data[i] >= 32 && data[i] <= 126) Serial.write(data[i]); else Serial.print("."); }
        Serial.println();
      } else Serial.println("Read failed.");
    }
  } else if (t.startsWith("NTAG")) {
    // NTAG pages are 4 bytes each. We'll read pages 4..7 (16 bytes)
    Serial.println("Reading NTAG pages 4..7 (16 bytes)...");
    uint8_t buf[16];
    bool ok = true;
    for (uint8_t p = 4; p <= 7; p++) {
      uint8_t pageData[4];
      if (!nfc.ntag2xx_ReadPage(p, pageData)) { ok = false; break; }
      memcpy(&buf[(p-4)*4], pageData, 4);
    }
    if (ok) {
      Serial.print("Pages 4..7 hex: ");
      for (int i=0;i<16;i++){ if (buf[i] < 0x10) Serial.print("0"); Serial.print(buf[i], HEX); Serial.print(" "); }
      Serial.println();
      Serial.print("ASCII: ");
      for (int i=0;i<16;i++){ if (buf[i] >= 32 && buf[i] <= 126) Serial.write(buf[i]); else Serial.print("."); }
      Serial.println();
    } else Serial.println("NTAG read failed.");
  } else {
    Serial.println("Unknown tag type: try reading UID only or test with a different card.");
  }
  printMenu();
}

// --- W: Write block/page (safe default) ---
void doWriteBlock() {
  Serial.println("Write: Enter data to write (max 16 chars). Type in Serial Monitor:");
  String s = "";
  while (s.length() == 0) {
    while (!Serial.available()) delay(10);
    s = Serial.readStringUntil('\n');
    s.trim();
  }
  if (s.length() == 0) { Serial.println("No input."); printMenu(); return; }
  if (s.length() > 16) s = s.substring(0,16);
  Serial.print("Data to write: '"); Serial.print(s); Serial.println("'");

  Serial.println("Place card to write...");
  uint8_t uid[7]; uint8_t uidLen;
  if (!waitForCard(uid, &uidLen, 8000)) { Serial.println("No card found."); printMenu(); return; }
  printUID(uid, uidLen);
  String t = detectTagType(uid, uidLen);
  Serial.print("Type: "); Serial.println(t);

  if (t.startsWith("MIFARE Classic")) {
    // write to block 4 only (safe)
    uint8_t keyA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 4, 0, keyA)) {
      Serial.println("Auth failed for block 4. Cannot write.");
    } else {
      uint8_t data[16]; memset(data,0,16);
      s.getBytes(data,17);
      if (nfc.mifareclassic_WriteDataBlock(4, data)) {
        Serial.println("Write OK to block 4.");
      } else Serial.println("Write failed.");
    }
  } else if (t.startsWith("NTAG")) {
    // NTAG write: pages are 4 bytes. We'll write pages 4..7 using 16-bytes payload.
    uint8_t buf[16]; memset(buf,0,16);
    s.getBytes(buf,17);
    bool ok = true;
    for (uint8_t p=4; p<=7; p++) {
      if (!nfc.ntag2xx_WritePage(p, &buf[(p-4)*4])) { ok = false; break; }
    }
    if (ok) Serial.println("NTAG pages 4..7 written OK.");
    else Serial.println("NTAG write failed.");
  } else {
    Serial.println("Unknown tag type; write aborted.");
  }
  printMenu();
}

// --- E: Erase (write zeros) ---
void doEraseBlock() {
  Serial.println("Erase: (This will overwrite the chosen block/page with zeros). Continue? (y/n)");
  while (!Serial.available()) delay(5);
  char c = Serial.read();
  if (toupper(c) != 'Y') { Serial.println("Abort."); printMenu(); return; }
  Serial.println("Place card to erase...");
  uint8_t uid[7]; uint8_t uidLen;
  if (!waitForCard(uid, &uidLen, 8000)) { Serial.println("No card."); printMenu(); return; }
  printUID(uid, uidLen);
  String t = detectTagType(uid, uidLen);
  Serial.print("Type: "); Serial.println(t);

  if (t.startsWith("MIFARE Classic")) {
    uint8_t keyA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 4, 0, keyA)) {
      Serial.println("Auth failed for block 4. Cannot erase.");
    } else {
      uint8_t zeros[16]; memset(zeros, 0, 16);
      if (nfc.mifareclassic_WriteDataBlock(4, zeros)) Serial.println("Block 4 erased.");
      else Serial.println("Erase write failed.");
    }
  } else if (t.startsWith("NTAG")) {
    uint8_t zeros[4]; memset(zeros,0,4);
    bool ok = true;
    for (uint8_t p=4; p<=7; p++) {
      if (!nfc.ntag2xx_WritePage(p, zeros)) { ok = false; break; }
    }
    if (ok) Serial.println("NTAG pages 4..7 erased.");
    else Serial.println("Erase failed.");
  } else Serial.println("Unknown type; abort.");
  printMenu();
}
