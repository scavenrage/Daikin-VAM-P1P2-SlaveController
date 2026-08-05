/*
 * VERSIONE: v19 - Slave P1P2 con link seriale verso ESP32-H2 (niente output umano)
 * P1P2_ATmega_v19.ino
 *
 * Continua a fare esattamente quello che faceva il v18 sul bus P1/P2 (si
 * registra come controller SLAVE, legge lo stato reale dal pacchetto 4000 40,
 * comanda la VMC cambiando la propria risposta 40F032). La logica del bus
 * P1P2 non è cambiata rispetto al v18 (pin RST/DOUT/DIN identici: 7/8/9).
 *
 * CAMBIA il resto: la scheda finale va dietro al pannello nel muro, nessuno
 * la tocca più via seriale a mano. Quindi:
 *  - Avvio dello slave AUTOMATICO all'accensione (equivalente a X1+R1 del v18,
 *    non servono più comandi manuali).
 *  - La UART hardware (pin PD0/PD1) è dedicata al link con l'ESP32-H2: niente
 *    più log leggibili da umano, solo il protocollo compatto sotto.
 *
 * ============================ PROTOCOLLO VERSO ESP32-H2 ============================
 * ESP32 -> ATmega (comandi, riga terminata da \n):
 *   ON | OFF | VL | VH | MA | MS | MB | FU1 | FU0 | GET
 *   (GET = richiede l'invio immediato dello stato corrente, usato dall'ESP32
 *    per riallinearsi dopo un riavvio/riconnessione)
 *
 * ATmega -> ESP32:
 *   S,<onoff>,<mode>,<speed>,<freshup>,<registered>\n
 *     onoff:      0=spento 1=acceso
 *     mode:       0=Auto 1=Scambio 2=Bypass
 *     speed:      1=Lento 5=Veloce (valori grezzi del bus, come nel payload 40F032)
 *     freshup:    0/1
 *     registered: 0/1 (siamo agganciati come slave sul canale 32 del bus)
 *   Inviato in automatico ad OGNI cambiamento (comandato da ESP32, letto dal
 *   pannello fisico, o cambio di registrazione), oppure subito in risposta a GET.
 *
 *   E,<code>\n
 *     1 = errore CRC su un pacchetto del bus (rate-limited, max 1 ogni 2s)
 *     2 = canale perso: il master ci ha richiesto poll 38/39 mentre eravamo
 *         registrati sul 32 (declassamento inatteso)
 *     3 = silenzio dal master: nessun poll verso F0 da oltre 5s
 *
 * Collegamenti (invariati dal v18): DOUT -> pin 8, DIN -> pin 9, RST -> pin 7
 */

#include <P1P2MQTT.h>

#define RST_PIN 7
#define SERIAL_BAUD 115200
#define BUS_BAUD 9600
#define CRC_GEN 0xD9
#define CRC_CS_FEED 0x00
#define SLAVE_DELAY 25   // ms: lo slave reale rispondeva a ~24-26ms
#define BUF_SIZE 32

#define SILENCE_TIMEOUT_MS 5000UL
#define CRC_ERROR_COOLDOWN_MS 2000UL

P1P2MQTT P1P2;

#define CMD_BUFFER_SIZE 8
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

bool registered = false;   // true dopo la prima risposta di presenza (40F0FF)

// --- Stato interno della VMC (ciò che comandiamo / rispecchiamo) ---
uint8_t stOnOff = 0;   // 0=off 1=on
uint8_t stMode  = 0;   // 00=Auto 01=Scambio 02=Bypass
uint8_t stSpeed = 1;   // 01=Lento 05=Veloce
bool    stFreshUp = false;
// true = rispecchia lo stato reale letto dal bus; false = c'è un comando in
// transizione, in attesa che il bus confermi il nuovo stato voluto.
bool    mirrorReal = true;

// Ultimo valore del contatore filtro visto nel poll 00F033 (da rimandare indietro)
uint8_t filterHi = 0x0F, filterLo = 0x0D;

// Snapshot dell'ultimo stato inviato all'ESP32, per inviare S, solo on-change
uint8_t lastSentOnOff, lastSentMode, lastSentSpeed;
bool lastSentFreshUp, lastSentRegistered;
bool stateEverSent = false;

unsigned long lastPollMs = 0;
bool silenceReported = false;
unsigned long lastCrcErrMs = 0;

uint8_t crcDaikin(uint8_t* data, uint8_t len) {
  uint8_t crc = CRC_CS_FEED;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      uint8_t mix = (crc ^ b) & 0x01;
      crc >>= 1;
      if (mix) crc ^= CRC_GEN;
      b >>= 1;
    }
  }
  return crc;
}

void restoreArduinoTimer0() {
  TCCR0A = (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS01) | (1 << CS00);
  TIMSK0 = (1 << TOIE0);
}

uint8_t modeByte() {
  return stMode | (stFreshUp ? 0x08 : 0x00);
}

void sendStateLine() {
  Serial.print(F("S,"));
  Serial.print(stOnOff); Serial.print(',');
  Serial.print(stMode); Serial.print(',');
  Serial.print(stSpeed); Serial.print(',');
  Serial.print(stFreshUp ? 1 : 0); Serial.print(',');
  Serial.println(registered ? 1 : 0);

  lastSentOnOff = stOnOff;
  lastSentMode = stMode;
  lastSentSpeed = stSpeed;
  lastSentFreshUp = stFreshUp;
  lastSentRegistered = registered;
  stateEverSent = true;
}

void sendError(uint8_t code) {
  Serial.print(F("E,"));
  Serial.println(code);
}

// Invia S, solo se qualcosa è cambiato rispetto all'ultimo invio
void checkAndSendState() {
  if (!stateEverSent ||
      stOnOff != lastSentOnOff || stMode != lastSentMode ||
      stSpeed != lastSentSpeed || stFreshUp != lastSentFreshUp ||
      registered != lastSentRegistered) {
    sendStateLine();
  }
}

void setup() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);   // transceiver in reset durante l'init del bus
  P1P2.begin(BUS_BAUD);
  restoreArduinoTimer0();
  digitalWrite(RST_PIN, LOW);    // abilita il transceiver (equivalente a "X1" del v18)
  Serial.begin(SERIAL_BAUD);
  lastPollMs = millis();
}

// Costruisce e invia la risposta 40 F0 <type> con payload dato
void sendResponse(uint8_t type, uint8_t* payload, uint8_t payloadLen) {
  uint8_t buf[BUF_SIZE];
  buf[0] = 0x40; buf[1] = 0xF0; buf[2] = type;
  for (uint8_t i = 0; i < payloadLen; i++) buf[3 + i] = payload[i];
  if (!P1P2.writeready()) return;
  P1P2.writepacket(buf, 3 + payloadLen, SLAVE_DELAY, CRC_GEN, CRC_CS_FEED);
}

void handlePoll(uint8_t type, uint8_t* reqPayload, uint8_t reqLen) {
  if ((type == 0x38 || type == 0x39) && registered) {
    // Il master ci ha richiesto sul canale single-controller: siamo stati
    // declassati dal 32. Non tentiamo di rispondere (vicolo cieco noto),
    // segnaliamo solo l'errore.
    registered = false;
    sendError(2);
    return;
  }
  if (type == 0x30) {
    // Primo poll 30 (registrazione): risposta 40F0FF (presenza).
    // Poll 30 successivi: SILENZIO ASSOLUTO (come il pannello vero), altrimenti
    // il master ci declassa al canale 38.
    if (!registered) {
      uint8_t buf[4] = {0x40, 0xF0, 0xFF, 0x00};
      if (P1P2.writeready()) {
        P1P2.writepacket(buf, 3, SLAVE_DELAY, CRC_GEN, CRC_CS_FEED);
        registered = true;
      }
    }
  } else if (type == 0x32) {
    uint8_t pl[6] = { stOnOff, 0x03, modeByte(), stSpeed, 0x00, 0x00 };
    sendResponse(0x32, pl, 6);
  } else if (type == 0x33) {
    if (reqLen >= 5) { filterHi = reqPayload[3]; filterLo = reqPayload[4]; }
    uint8_t pl[5] = { 0x00, 0x00, 0x00, filterHi, filterLo };
    sendResponse(0x33, pl, 5);
  } else if (type == 0x34) {
    sendResponse(0x34, NULL, 0);
  }
}

void processCommand(char* cmd) {
  int len = strlen(cmd);
  if (len > 0 && (cmd[len - 1] == '\r')) cmd[len - 1] = '\0';

  if (strcmp(cmd, "GET") == 0) { sendStateLine(); return; }
  if (strcmp(cmd, "ON") == 0)  { stOnOff = 1; mirrorReal = false; return; }
  if (strcmp(cmd, "OFF") == 0) { stOnOff = 0; mirrorReal = false; return; }
  if (strcmp(cmd, "VL") == 0)  { stSpeed = 0x01; mirrorReal = false; return; }
  if (strcmp(cmd, "VH") == 0)  { stSpeed = 0x05; mirrorReal = false; return; }
  if (strcmp(cmd, "MA") == 0)  { stMode = 0x00; mirrorReal = false; return; }
  if (strcmp(cmd, "MS") == 0)  { stMode = 0x01; mirrorReal = false; return; }
  if (strcmp(cmd, "MB") == 0)  { stMode = 0x02; mirrorReal = false; return; }
  if (strcmp(cmd, "FU1") == 0) { stFreshUp = true;  mirrorReal = false; return; }
  if (strcmp(cmd, "FU0") == 0) { stFreshUp = false; mirrorReal = false; return; }
  // comando non riconosciuto: ignorato silenziosamente
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdIndex > 0) processCommand(cmdBuffer);
      cmdIndex = 0;
    } else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }

  while (P1P2.packetavailable()) {
    uint8_t readBuf[BUF_SIZE];
    errorbuf_t errorBuf[BUF_SIZE];
    uint16_t delta;
    int nread = P1P2.readpacket(readBuf, delta, errorBuf, BUF_SIZE, CRC_GEN, CRC_CS_FEED);
    if (nread > BUF_SIZE) nread = BUF_SIZE;
    bool hasError = false;
    for (int i = 0; i < nread; i++) if (errorBuf[i]) hasError = true;

    if (nread < 3) continue;
    if (hasError) {
      unsigned long now = millis();
      if (now - lastCrcErrMs >= CRC_ERROR_COOLDOWN_MS) {
        sendError(1);
        lastCrcErrMs = now;
      }
      continue;
    }

    uint8_t dir = readBuf[0], addr = readBuf[1], type = readBuf[2];

    // --- AUTO-LETTURA STATO REALE dal pacchetto 4000 40 (emesso dall'unità) ---
    if (dir == 0x40 && addr == 0x00 && type == 0x40 && nread >= 14) {
      uint8_t realOn   = readBuf[3];
      uint8_t realMode = readBuf[12] & 0x07;
      uint8_t realFU   = (readBuf[12] & 0x08) ? 1 : 0;
      uint8_t realSpd  = readBuf[13];
      if (mirrorReal) {
        stOnOff = realOn; stMode = realMode; stFreshUp = realFU; stSpeed = realSpd;
      } else if (realOn == stOnOff && realMode == stMode &&
                 realSpd == stSpeed && realFU == (stFreshUp ? 1 : 0)) {
        mirrorReal = true;   // il bus ha confermato il comando
      }
    }

    // Poll del master verso F0 (dir=00 addr=F0)
    if (dir == 0x00 && addr == 0xF0) {
      lastPollMs = millis();
      silenceReported = false;
      handlePoll(type, &readBuf[3], (nread >= 4) ? (nread - 4) : 0);
    }
  }

  if (millis() - lastPollMs > SILENCE_TIMEOUT_MS && !silenceReported) {
    sendError(3);
    silenceReported = true;
  }

  checkAndSendState();
}
