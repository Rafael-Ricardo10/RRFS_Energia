/*
 * ════════════════════════════════════════════════════════════
 * BMS PROXY v3 - captura o protocolo app LanmaoBMS <-> BMS
 * Rodar em um SEGUNDO ESP32 (nao no sistema de energia).
 *
 * v3:
 *   - MTU 185 (como o iPhone): a BMS manda a resposta COMPLETA
 *     (antes vinham so 20 bytes de cada resposta)
 *   - Monta os quadros, confere o CRC e mostra as palavras de 16 bits
 *   - "Sonda propria": ao iniciar, o proprio ESP32 le os 4 blocos de
 *     registradores, sem precisar do app
 *
 * Placa: ESP32 Dev Module | Particao: "Huge APP (3MB No OTA)"
 * Serial Monitor: 115200
 * ════════════════════════════════════════════════════════════
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>
#include <esp_gap_ble_api.h>
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3)
#include <BLE2902.h>
#endif

// ── CONFIG ───────────────────────────────────────────────
#define BMS_MAC       "A2:60:20:20:03:33"   // MAC da sua BMS
#define MTU_DESEJADO  185
#define MAX_CHARS     10
#define EVT_MAX       200
#define MAX_TENTATIVAS_SCAN 25

// ── ESTRUTURAS ───────────────────────────────────────────
struct Mapa {
    BLERemoteCharacteristic* remoto;
    BLECharacteristic*       local;
    bool                     respRemota;
};
Mapa    mapa[MAX_CHARS];
uint8_t nMapa = 0;

struct Evt {
    uint8_t  tipo;          // 1 = app->BMS | 2 = BMS->app
    uint8_t  idx;
    uint8_t  len;
    uint32_t t;
    uint8_t  dados[EVT_MAX];
};
QueueHandle_t fila;

BLEClient*           cliente  = nullptr;
BLEServer*           servidor = nullptr;
BLEAdvertisedDevice* alvo     = nullptr;
String               uuidAdv     = "";
String               primeiroSvc = "";
uint8_t              advRaw[32];
size_t               advLen = 0;
volatile bool        appConectado = false;
volatile bool        bmsOnline    = false;
volatile bool        proxyPronto  = false;

// ── UTIL ─────────────────────────────────────────────────
static String uuidStr(BLEUUID u) { return String(u.toString().c_str()); }

static void dump(const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; i++) Serial.printf("%02X", d[i]);
    Serial.print("  |");
    for (size_t i = 0; i < n; i++) Serial.print((d[i] >= 32 && d[i] < 127) ? (char)d[i] : '.');
    Serial.println("|");
}

static uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// Decodifica pedido do app: A5 A5 00 03 RRRR QQQQ CRC
static void decodificarPedido(const uint8_t* d, size_t n) {
    if (n == 10 && d[0] == 0xA5 && d[1] == 0xA5 && d[3] == 0x03) {
        unsigned reg = (d[4] << 8) | d[5];
        unsigned qtd = (d[6] << 8) | d[7];
        Serial.printf("   -> LER registradores %u (0x%04X) ate %u, qtd=%u\n",
                      reg, reg, reg + qtd - 1, qtd);
    }
}

// Junta os pedacos de uma resposta e confere o CRC
static uint8_t acc[400];
static size_t  accLen = 0;

static void analisarFrame(const uint8_t* d, size_t n) {
    if (accLen == 0 && !(n >= 2 && d[0] == 0xA5 && d[1] == 0xA5)) return;
    if (accLen + n > sizeof(acc)) { accLen = 0; return; }
    memcpy(acc + accLen, d, n);
    accLen += n;
    if (accLen < 5) return;

    if (acc[3] != 0x03) {
        Serial.printf("   FRAME func=%02X (nao e leitura) - bytes brutos acima\n", acc[3]);
        accLen = 0;
        return;
    }
    size_t total = 5 + (size_t)acc[4] + 2;
    if (accLen < total) return;                     // ainda faltam pedacos

    uint16_t c = crc16(acc + 2, total - 4);
    bool ok = (acc[total - 2] == (c & 0xFF)) && (acc[total - 1] == (c >> 8));
    Serial.printf("   FRAME completo: %u bytes, dados=%u bytes, CRC %s\n",
                  (unsigned)total, acc[4], ok ? "OK" : "ERRO");
    Serial.print("   palavras 16b (decimal): ");
    for (size_t i = 5; i + 1 < 5 + (size_t)acc[4]; i += 2)
        Serial.printf("%u ", (acc[i] << 8) | acc[i + 1]);
    Serial.println();
    accLen = 0;
}

// ── CALLBACKS ────────────────────────────────────────────
class CbScan : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) override {
        String mac = String(device.getAddress().toString().c_str());
        mac.toUpperCase();
        if (mac != String(BMS_MAC)) return;

        size_t n = device.getPayloadLength();
        if (advLen == 0 && device.haveServiceUUID() && n > 0 && n <= 31) {
            memcpy(advRaw, device.getPayload(), n);
            advLen = n;
        }
        if (device.haveServiceUUID() && uuidAdv.length() == 0)
            uuidAdv = uuidStr(device.getServiceUUID());
        if (!alvo) alvo = new BLEAdvertisedDevice(device);
    }
};

class CbCliente : public BLEClientCallbacks {
    void onConnect(BLEClient*) override {}
    void onDisconnect(BLEClient*) override { bmsOnline = false; }
};

class CbServidor : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        if (!proxyPronto) return;
        appConectado = true;
        Serial.println("[SRV] conexao no proxy (app)");
    }
    void onDisconnect(BLEServer*) override {
        if (!proxyPronto) return;
        appConectado = false;
        Serial.println("[SRV] desconexao (app ou BMS) - anunciando de novo");
        BLEDevice::startAdvertising();
    }
};

class CbChar : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        for (uint8_t i = 0; i < nMapa; i++) {
            if (mapa[i].local != c) continue;
            Evt e;
            e.tipo = 1; e.idx = i; e.t = millis();
            size_t n = c->getLength();
            if (n > EVT_MAX) n = EVT_MAX;
            memcpy(e.dados, c->getData(), n);
            e.len = (uint8_t)n;
            xQueueSend(fila, &e, 0);
            return;
        }
    }
};

void cbNotifyRemoto(BLERemoteCharacteristic* rc, uint8_t* d, size_t n, bool isNotify) {
    for (uint8_t i = 0; i < nMapa; i++) {
        if (mapa[i].remoto != rc) continue;
        Evt e;
        e.tipo = 2; e.idx = i; e.t = millis();
        if (n > EVT_MAX) n = EVT_MAX;
        memcpy(e.dados, d, n);
        e.len = (uint8_t)n;
        xQueueSend(fila, &e, 0);
        return;
    }
}

static CbScan     cbScan;
static CbCliente  cbCliente;
static CbServidor cbServidor;
static CbChar     cbChar;

// ── PROCESSA A FILA (imprime e repassa) ──────────────────
static void processarFila() {
    Evt e;
    while (xQueueReceive(fila, &e, 0) == pdTRUE) {
        Mapa& m = mapa[e.idx];
        String u = uuidStr(m.remoto->getUUID());
        u = u.substring(4, 8);        // mostra so FFFB / FFFC / 0002 / 0003

        if (e.tipo == 1) {
            Serial.printf("[%lu] APP->BMS  %s (%uB): ", (unsigned long)e.t, u.c_str(), e.len);
            dump(e.dados, e.len);
            decodificarPedido(e.dados, e.len);
            if (bmsOnline) m.remoto->writeValue(e.dados, e.len, m.respRemota);
        } else {
            Serial.printf("[%lu] BMS->APP  %s (%uB): ", (unsigned long)e.t, u.c_str(), e.len);
            dump(e.dados, e.len);
            analisarFrame(e.dados, e.len);
            if (appConectado) {
                m.local->setValue(e.dados, e.len);
                m.local->notify();
            }
        }
    }
}

// ── SONDA PROPRIA: le os 4 blocos que o app le ───────────
static void sondarBMS() {
    BLERemoteCharacteristic* tx = nullptr;
    for (uint8_t i = 0; i < nMapa; i++)
        if (uuidStr(mapa[i].remoto->getUUID()).indexOf("fffb") >= 0) tx = mapa[i].remoto;
    if (!tx) { Serial.println("Sonda: FFFB nao achada"); return; }

    static const uint8_t reqs[4][10] = {
        {0xA5,0xA5,0x00,0x03,0x00,0x00,0x00,0x48,0x44,0x2D},   // regs 0..71
        {0xA5,0xA5,0x00,0x03,0x00,0x3C,0x00,0x16,0x05,0xD9},   // regs 60..81
        {0xA5,0xA5,0x00,0x03,0x00,0x5A,0x00,0x17,0x24,0x06},   // regs 90..112
        {0xA5,0xA5,0x00,0x03,0x00,0x6F,0x00,0x0B,0x35,0xC1},   // regs 111..121
    };
    Serial.println("--- SONDA PROPRIA (sem app) ---");
    for (int i = 0; i < 4; i++) {
        Serial.printf("[sonda] pedido %d\n", i + 1);
        tx->writeValue((uint8_t*)reqs[i], 10, false);
        unsigned long t0 = millis();
        while (millis() - t0 < 1200) { processarFila(); delay(5); }
    }
    Serial.println("--- fim da sonda ---");
}

// ── SETUP ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println();
    Serial.println("=== BMS PROXY v3 ===");
    Serial.println("Deixe o app LanmaoBMS FECHADO e o outro ESP32 DESLIGADO.");

    fila = xQueueCreate(64, sizeof(Evt));

    BLEDevice::init("BMS-PROXY");
    BLEDevice::setMTU(MTU_DESEJADO);

    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&cbScan, true);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    for (int t = 0; t < MAX_TENTATIVAS_SCAN && !alvo; t++) {
        Serial.printf("Procurando BMS %s (tentativa %d/%d)...\n", BMS_MAC, t + 1, MAX_TENTATIVAS_SCAN);
        scan->start(8, false);
        scan->clearResults();
    }
    if (!alvo) {
        Serial.println("ERRO: BMS nao apareceu. Reiniciando em 5s.");
        delay(5000);
        ESP.restart();
    }
    Serial.printf("BMS encontrada. Anuncio clonado: %s (%u bytes)\n",
                  advLen ? "SIM" : "NAO", (unsigned)advLen);

    cliente = BLEDevice::createClient();
    cliente->setClientCallbacks(&cbCliente);
    Serial.println("Conectando na BMS real...");
    if (!cliente->connect(alvo)) {
        Serial.println("ERRO: falha ao conectar na BMS. Reiniciando em 5s.");
        delay(5000);
        ESP.restart();
    }
    bmsOnline = true;
    Serial.printf("Conectado na BMS real. MTU negociado: %u\n", (unsigned)cliente->getMTU());

    servidor = BLEDevice::createServer();
    servidor->setCallbacks(&cbServidor);

    auto* servicos = cliente->getServices();
    if (!servicos) {
        Serial.println("ERRO: sem servicos. Reiniciando.");
        delay(3000);
        ESP.restart();
    }

    for (auto& kv : *servicos) {
        BLERemoteService* rs = kv.second;
        String su = uuidStr(rs->getUUID());
        if (su.indexOf("1800") >= 0 || su.indexOf("1801") >= 0) continue;

        Serial.printf("SERVICO %s\n", su.c_str());
        if (primeiroSvc.length() == 0) primeiroSvc = su;

        BLEService* ls = servidor->createService(rs->getUUID());

        auto* chars = rs->getCharacteristics();
        if (!chars) continue;

        for (auto& kc : *chars) {
            BLERemoteCharacteristic* rc = kc.second;
            if (nMapa >= MAX_CHARS) break;

            uint32_t pr = 0;
            if (rc->canRead())            pr |= BLECharacteristic::PROPERTY_READ;
            if (rc->canWrite())           pr |= BLECharacteristic::PROPERTY_WRITE;
            if (rc->canWriteNoResponse()) pr |= BLECharacteristic::PROPERTY_WRITE_NR;
            if (rc->canNotify())          pr |= BLECharacteristic::PROPERTY_NOTIFY;
            if (rc->canIndicate())        pr |= BLECharacteristic::PROPERTY_INDICATE;
            if (pr == 0) continue;

            Serial.printf("  CHAR %s  R%d W%d WN%d N%d I%d\n",
                          uuidStr(rc->getUUID()).c_str(),
                          (int)rc->canRead(), (int)rc->canWrite(),
                          (int)rc->canWriteNoResponse(),
                          (int)rc->canNotify(), (int)rc->canIndicate());

            BLECharacteristic* lc = ls->createCharacteristic(rc->getUUID(), pr);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3)
            if (rc->canNotify() || rc->canIndicate()) lc->addDescriptor(new BLE2902());
#endif
            if (rc->canWrite() || rc->canWriteNoResponse()) lc->setCallbacks(&cbChar);

            if (rc->canRead()) {
                auto v = rc->readValue();
                lc->setValue((uint8_t*)v.c_str(), v.length());
            }

            mapa[nMapa].remoto     = rc;
            mapa[nMapa].local      = lc;
            mapa[nMapa].respRemota = (rc->canWrite() && !rc->canWriteNoResponse());
            nMapa++;
        }
        ls->start();
    }

    for (uint8_t i = 0; i < nMapa; i++) {
        BLERemoteCharacteristic* rc = mapa[i].remoto;
        if (!(rc->canNotify() || rc->canIndicate())) continue;
        rc->registerForNotify(cbNotifyRemoto);
        BLERemoteDescriptor* d = rc->getDescriptor(BLEUUID((uint16_t)0x2902));
        if (d) {
            uint8_t v[2] = { (uint8_t)(rc->canNotify() ? 0x01 : 0x02), 0x00 };
            d->writeValue(v, 2, true);
        }
    }

    // Sonda propria ANTES de abrir para o app
    sondarBMS();

    // Anuncio clonado
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    if (advLen >= 3) {
        BLEAdvertisementData ad;
        ad.addData((char*)advRaw, (uint32_t)advLen);
        adv->setAdvertisementData(ad);
        adv->setScanResponse(false);
    } else {
        String uAdv = (uuidAdv.length() ? uuidAdv : primeiroSvc);
        if (uAdv.length()) adv->addServiceUUID(BLEUUID(uAdv));
        adv->setScanResponse(true);
    }
    BLEDevice::startAdvertising();
    proxyPronto = true;

    Serial.println();
    Serial.println("=======================================================");
    Serial.println("PROXY PRONTO. Abra AGORA o app LanmaoBMS no iPhone.");
    Serial.println("Deixe uns 30 s na tela principal e tire PRINTS de cada tela.");
    Serial.println("Depois copie TUDO deste Serial Monitor e me envie.");
    Serial.println("=======================================================");
}

// ── LOOP ─────────────────────────────────────────────────
void loop() {
    processarFila();

    if (!bmsOnline) {
        Serial.println("BMS real desconectou. Reiniciando em 2s...");
        delay(2000);
        ESP.restart();
    }

    static unsigned long tHb = 0;
    if (millis() - tHb > 30000UL) {
        tHb = millis();
        Serial.printf("[hb] app=%d bms=%d\n", (int)appConectado, (int)bmsOnline);
    }
    delay(2);
}
