/*
 * VERSIONE: v18 (rev.18) - "FIX: silenzio sui poll 30 dopo la registrazione (come il pannello vero)"
 * P1P2_SlaveController.ino
 *
 * ============================== SCOPO ==============================
 * L'Arduino prende il posto FISICO del pannello slave (in prestito) sul bus.
 * Configurazione finale: pannello storico = MASTER, Arduino = SLAVE.
 * Replica esattamente il comportamento del pannello slave catturato nei log,
 * e permette di COMANDARE la VMC (on/off, modalita', velocita', fresh-up)
 * da seriale/Home Assistant.
 *
 * ================= PROTOCOLLO DECODIFICATO (dai log reali) =================
 * Il master interroga lo slot F0. Lo slave DEVE rispondere:
 *   00F030 -> 40F0FF (presenza) alla registrazione, poi 40F030 vuoto
 *   00F032 -> 40F032 payload "01 03 MM VV 00 00"  <-- STATO e COMANDO
 *   00F033 -> 40F033 payload "00 00 00 FH FL"     <-- eco contatore filtro
 *   00F034 -> 40F034 vuoto (solo header+CRC)
 *
 * Codifica stato nel 40F032 (payload):
 *   byte0 = ON/OFF: 00=spento, 01=acceso
 *   byte1 = 03 (fisso, "valido")
 *   byte2 = MODALITA': 00=Auto 01=Scambio 02=Bypass ; bit 0x08 = Fresh-up
 *   byte3 = VELOCITA': 01=Lento 05=Veloce
 *   byte4,5 = 00 00
 *
 * Per COMANDARE: si cambia lo stato interno (con i comandi seriali sotto) e
 * lo slave, alla successiva richiesta 00F032, risponde col nuovo stato. Il
 * master lo recepisce e comanda la VMC. (Confermato sui log: ogni cambio di
 * 40F032 e' seguito dal cambiamento del pacchetto 4000 40.)
 *
 * ============================== COMANDI SERIALI ==============================
 *   R1 / R0   registrazione slave ON/OFF (R1 = inizia a rispondere ai poll)
 *   ON / OFF  accende / spegne la VMC
 *   VL / VH   velocita' Lenta / Veloce
 *   MA / MS / MB   Modalita' Auto / Scambio / Bypass
 *   FU1 / FU0 Fresh-up ON / OFF
 *   ST        stampa lo stato interno corrente
 *   V1 / V0   log grezzo ON/OFF
 *   X1 / X0   TX hardware ON/OFF
 *
 * SEQUENZA USO: X1 -> R1. Poi comandi (ON, VH, MS...) a piacere.
 *   L'Arduino risponde ai poll del master col nuovo stato -> la VMC obbedisce.
 *
 * ============================== SICUREZZA ==============================
 * - Configurazione: SOLO pannello-master + questo Arduino sul bus.
 *   NON deve esserci un altro slave (il pannello in prestito va RIMOSSO).
 * - Se compare UA sul pannello master, dai X0 e verifica le risposte.
 *
 * ============================== LOGICA v17 ==============================
 * L'Arduino LEGGE lo stato reale della VMC DAL BUS, in automatico, dal
 * pacchetto 4000 40 (emesso dall'unita', non da noi). Mappa verificata:
 *   indice 3  = ON/OFF   (00=spento 01=acceso)
 *   indice 12 = MODALITA'(00=Auto 01=Scambio 02=Bypass ; bit 08=FreshUp)
 *   indice 13 = VELOCITA'(01=Lento 05=Veloce)
 *
 * Finche' l'utente non da' un comando, l'Arduino rispecchia lo stato reale
 * (cosi' e' sempre allineato e non forza transizioni). Quando arriva un
 * comando (ON/OFF/...), aggiorna lo stato interno: la sua risposta 40F032
 * cambia -> il master vede la TRANSIZIONE -> comanda la VMC. Dopo che il
 * bus conferma il nuovo stato, l'Arduino tornera' a rispecchiare il reale.
 *
 * SEQUENZA TEST: X1 -> R1 -> (attendi registrazione) -> ON (o OFF).
 *   Nessuna dichiarazione manuale: lo stato lo conosce gia' dal bus.
 *
 * Collegamenti: DOUT -> pin 8, DIN -> pin 9, RST -> pin 7
 */

#include <P1P2MQTT.h>

#define RST_PIN 7
#define SERIAL_BAUD 115200
#define BUS_BAUD 9600
#define CRC_GEN 0xD9
#define CRC_CS_FEED 0x00
#define SLAVE_DELAY 25   // ms: lo slave reale rispondeva a ~24-26ms

P1P2MQTT P1P2;

#define CMD_BUFFER_SIZE 100
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;
#define BUF_SIZE 32

bool writeEnabled = false;
bool verboseMode = false;
bool slaveActive = false;
bool registered = false;   // true dopo la prima risposta di presenza

// --- Stato interno della VMC (cio' che comandiamo) ---
// Lo stato reale viene letto in automatico dal bus (pacchetto 4000 40).
// Questi default valgono solo per i primissimi ms prima della prima lettura.
uint8_t stOnOff = 0;   // 0=off 1=on
uint8_t stMode  = 0;   // 00=Auto 01=Scambio 02=Bypass (bit 0x08 = FreshUp)
uint8_t stSpeed = 1;   // 01=Lento 05=Veloce
bool    stFreshUp = false;
// Quando true, l'Arduino rispecchia lo stato reale letto dal bus.
// Quando l'utente comanda, diventa false finche' il bus non conferma il
// nuovo stato (evita che il rispecchiamento sovrascriva il comando).
bool    mirrorReal = true;
bool    realKnown = false;

// Ultimo valore del contatore filtro visto nel poll 00F033 (da rimandare indietro)
uint8_t filterHi = 0x0F, filterLo = 0x0D;

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

void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void restoreArduinoTimer0() {
  TCCR0A = (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS01) | (1 << CS00);
  TIMSK0 = (1 << TOIE0);
}

// Restituisce il byte modalita' combinato (modo + eventuale bit fresh-up)
uint8_t modeByte() {
  return stMode | (stFreshUp ? 0x08 : 0x00);
}

void printState() {
  Serial.print(F("* STATO: "));
  Serial.print(stOnOff ? F("ACCESO") : F("SPENTO"));
  Serial.print(F(" | Modo "));
  if (stMode == 0) Serial.print(F("Auto"));
  else if (stMode == 1) Serial.print(F("Scambio"));
  else if (stMode == 2) Serial.print(F("Bypass"));
  Serial.print(F(" | Vel "));
  Serial.print(stSpeed == 5 ? F("Veloce") : F("Lento"));
  Serial.print(F(" | FreshUp "));
  Serial.print(stFreshUp ? F("ON") : F("OFF"));
  Serial.print(F("  [40F032 payload: "));
  printHexByte(stOnOff); Serial.print(' ');
  Serial.print(F("03 "));
  printHexByte(modeByte()); Serial.print(' ');
  printHexByte(stSpeed); Serial.print(F(" 00 00]"));
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  writeEnabled = false;
  P1P2.begin(BUS_BAUD);
  restoreArduinoTimer0();
  Serial.println(F("* P1P2 SLAVE CONTROLLER v14"));
  Serial.println(F("* L'Arduino fa da controller SLAVE. Master = pannello storico."));
  Serial.println(F("* Sequenza: X1 -> R1. Poi comandi: ON/OFF VL/VH MA/MS/MB FU1/FU0"));
  Serial.println(F("* ST=stato  V1/V0=log  X0=stop"));
}

// Costruisce e invia la risposta 40 F0 <type> con payload dato
void sendResponse(uint8_t type, uint8_t* payload, uint8_t payloadLen) {
  uint8_t buf[BUF_SIZE];
  buf[0] = 0x40; buf[1] = 0xF0; buf[2] = type;
  for (uint8_t i = 0; i < payloadLen; i++) buf[3 + i] = payload[i];
  // La libreria calcola e appende il CRC (ultimi due parametri)
  if (!P1P2.writeready()) return;
  P1P2.writepacket(buf, 3 + payloadLen, SLAVE_DELAY, CRC_GEN, CRC_CS_FEED);
  if (verboseMode) {
    Serial.print(F("40F0")); printHexByte(type); Serial.print(F("-> "));
    for (uint8_t i = 0; i < payloadLen; i++) printHexByte(payload[i]);
    Serial.println();
  }
}

void handlePoll(uint8_t type, uint8_t* reqPayload, uint8_t reqLen) {
  if (type == 0x30) {
    // Verificato riga-per-riga sui log del pannello vero:
    //  - PRIMO poll 30 (registrazione): risposta 40F0FF (presenza)
    //  - poll 30 SUCCESSIVI: NESSUNA risposta (silenzio assoluto)
    // La v17 rispondeva 40F030 vuoto ai 30 successivi: comportamento
    // estraneo al pannello, e il master declassava l'Arduino al canale 38
    // invece di dargli il 32. FIX v18: silenzio dopo la registrazione.
    if (!registered) {
      uint8_t buf[4] = {0x40, 0xF0, 0xFF, 0x00};
      if (P1P2.writeready()) {
        P1P2.writepacket(buf, 3, SLAVE_DELAY, CRC_GEN, CRC_CS_FEED);
        registered = true;
        Serial.println(F("* Registrato come slave (risposta di presenza inviata)"));
      }
    }
    // else: SILENZIO (nessuna risposta, come il pannello vero)
  } else if (type == 0x32) {
    // STATO / COMANDO: rispondi con lo stato interno corrente
    uint8_t pl[6] = { stOnOff, 0x03, modeByte(), stSpeed, 0x00, 0x00 };
    sendResponse(0x32, pl, 6);
  } else if (type == 0x33) {
    // Filtro: rimanda il contatore che il master ci ha inviato nel poll
    // Il poll 00F033 ha payload tipo "00 00 00 0F 0D" -> lo rispecchiamo
    if (reqLen >= 5) { filterHi = reqPayload[3]; filterLo = reqPayload[4]; }
    uint8_t pl[5] = { 0x00, 0x00, 0x00, filterHi, filterLo };
    sendResponse(0x33, pl, 5);
  } else if (type == 0x34) {
    // Risposta vuota (come 40F034 nel log)
    sendResponse(0x34, NULL, 0);
  }
}

void processCommand(char* cmd) {
  int len = strlen(cmd);
  if (len > 0 && (cmd[len - 1] == '\r')) cmd[len - 1] = '\0';

  if (strcmp(cmd, "X1") == 0) { digitalWrite(RST_PIN, LOW); writeEnabled = true; Serial.println(F("* TX ON")); return; }
  if (strcmp(cmd, "X0") == 0) { digitalWrite(RST_PIN, HIGH); writeEnabled = false; slaveActive = false; registered = false; Serial.println(F("* TX OFF, slave fermato")); return; }
  if (strcmp(cmd, "R1") == 0) {
    if (!writeEnabled) { Serial.println(F("* Serve prima X1")); return; }
    slaveActive = true; registered = false;
    Serial.println(F("* SLAVE ATTIVO: rispondo ai poll del master. Attendo registrazione..."));
    return;
  }
  if (strcmp(cmd, "R0") == 0) { slaveActive = false; Serial.println(F("* Slave disattivato")); return; }

  if (strcmp(cmd, "ON") == 0)  { stOnOff = 1; mirrorReal = false; Serial.println(F("* COMANDO ON: transizione verso ACCESO")); printState(); return; }
  if (strcmp(cmd, "OFF") == 0) { stOnOff = 0; mirrorReal = false; Serial.println(F("* COMANDO OFF: transizione verso SPENTO")); printState(); return; }
  if (strcmp(cmd, "VL") == 0)  { stSpeed = 0x01; mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "VH") == 0)  { stSpeed = 0x05; mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "MA") == 0)  { stMode = 0x00; mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "MS") == 0)  { stMode = 0x01; mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "MB") == 0)  { stMode = 0x02; mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "FU1") == 0) { stFreshUp = true;  mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "FU0") == 0) { stFreshUp = false; mirrorReal = false; printState(); return; }
  if (strcmp(cmd, "ST") == 0)  { printState(); return; }
  if (strcmp(cmd, "V1") == 0)  { verboseMode = true;  Serial.println(F("* Verbose ON")); return; }
  if (strcmp(cmd, "V0") == 0)  { verboseMode = false; Serial.println(F("* Verbose OFF")); return; }

  Serial.print(F("* Comando non riconosciuto: ")); Serial.println(cmd);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdIndex > 0) processCommand(cmdBuffer);
      cmdIndex = 0;
    } else if (cmdIndex < CMD_BUFFER_SIZE - 1) cmdBuffer[cmdIndex++] = c;
  }

  while (P1P2.packetavailable()) {
    uint8_t readBuf[BUF_SIZE];
    errorbuf_t errorBuf[BUF_SIZE];
    uint16_t delta;
    int nread = P1P2.readpacket(readBuf, delta, errorBuf, BUF_SIZE, CRC_GEN, CRC_CS_FEED);
    if (nread > BUF_SIZE) nread = BUF_SIZE;
    bool hasError = false;
    for (int i = 0; i < nread; i++) if (errorBuf[i]) hasError = true;

    if (verboseMode) {
      Serial.print(hasError ? F("E ") : F("R "));
      for (int i = 0; i < nread; i++) printHexByte(readBuf[i]);
      Serial.println();
    }

    if (hasError || nread < 3) continue;

    uint8_t dir = readBuf[0], addr = readBuf[1], type = readBuf[2];

    // --- AUTO-LETTURA STATO REALE dal pacchetto 4000 40 (emesso dall'unita') ---
    // indice 3=on/off, 12=modalita', 13=velocita'. E' lo stato VERO della VMC.
    if (dir == 0x40 && addr == 0x00 && type == 0x40 && nread >= 14) {
      uint8_t realOn   = readBuf[3];
      uint8_t realMode = readBuf[12] & 0x07;
      uint8_t realFU   = (readBuf[12] & 0x08) ? 1 : 0;
      uint8_t realSpd  = readBuf[13];
      realKnown = true;
      if (mirrorReal) {
        // Allinea l'Arduino allo stato reale (nessun comando in corso)
        stOnOff = realOn; stMode = realMode; stFreshUp = realFU; stSpeed = realSpd;
      } else {
        // Comando in corso: se il bus ha raggiunto lo stato che vogliamo,
        // il comando e' andato a segno -> torna a rispecchiare.
        if (realOn == stOnOff && realMode == stMode &&
            realSpd == stSpeed && realFU == (stFreshUp?1:0)) {
          mirrorReal = true;
          Serial.println(F("* Comando confermato dal bus, stato allineato."));
        }
      }
    }

    // Rispondiamo ai poll del master verso F0 (dir=00 addr=F0)
    if (slaveActive && dir == 0x00 && addr == 0xF0) {
      handlePoll(type, &readBuf[3], (nread >= 4) ? (nread - 4) : 0);
    }
  }
}
