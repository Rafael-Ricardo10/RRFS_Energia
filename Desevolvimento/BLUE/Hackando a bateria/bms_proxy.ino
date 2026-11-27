/*
 * ════════════════════════════════════════════════════════════
 * BMS PROXY - captura o protocolo app LanmaoBMS <-> BMS
 * Rodar em um SEGUNDO ESP32 (nao no sistema de energia).
 *
 * O que faz:
 *   1) Procura a BMS real pelo MAC e conecta nela (como cliente)
 *   2) Copia todos os servicos/caracteristicas dela
 *   3) Anuncia o mesmo nome (como se fosse a BMS)
 *   4) O app no iPhone conecta neste ESP32; tudo que o app
 *      escreve e tudo que a BMS responde e repassado e
 *      MOSTRADO NO SERIAL MONITOR (115200).
 *
 * Placa: ESP32 Dev Module | Particao: "Huge APP (3MB No OTA)"
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
#define NOME_FORCADO  ""                    // so preencha se o nome nao for detectado
#define MAX_CHARS     10
#define EVT_MAX       96

// ── ESTRUTURAS ───────────────────────────────────────────
struct Mapa {
    BLERemoteCharacteristic* remoto;   // lado BMS real
    BLECharacteristic*       local;    // lado app (este ESP32 como "BMS")
    bool                     respRemota;
};
Mapa    mapa[MAX_CHARS];
uint8_t nMapa = 0;

struct Evt {
    uint8_t  tipo;          // 1 = app->BMS (escrita) | 2 = BMS->app (notificacao)
    uint8_t  idx;
    uint8_t  len;
    uint32_t t;
    uint8_t  dados[EVT_MAX];
};
QueueHandle_t fila;

BLEClient*           cliente = nullptr;
BLEServer*           servidor = nullptr;
BLEAdvertisedDevice* alvo = nullptr;
String               nomeBMS = "";
String               uuidAdv = "";
String               primeiroSvc = "";
volatile bool        appConectado = false;
volatile bool        bmsOnline = false;

// ── UTIL ─────────────────────────────────────────────────
static String uuidStr(BLEUUID u) { return String(u.toString().c_str()); }

static void dump(const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; i++) Serial.printf("%02X", d[i]);
    Serial.print("  |");
    for (size_t i = 0; i < n; i++) Serial.print((d[i] >= 32 && d[i] < 127) ? (char)d[i] : '.');
    Serial.println("|");
}

// ── CALLBACKS ────────────────────────────────────────────
class CbScan : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) override {
        String mac = String(device.getAddress().toString().c_str());
        mac.toUpperCase();
        if (mac != String(BMS_MAC)) return;

        static uint8_t nprint = 0;
        if (nprint < 6) {
            nprint++;
            Serial.printf("ADV[%d] rssi=%d payload(%u): ", nprint, device.getRSSI(),
                          (unsigned)device.getPayloadLength());
            for (size_t i = 0; i < device.getPayloadLength(); i++)
                Serial.printf("%02X", device.getPayload()[i]);
            Serial.println();
        }
        if (device.haveName()) {
            String n = String(device.getName().c_str());
            if (n.length()) nomeBMS = n;
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
        appConectado = true;
        Serial.println("[APP] conectou no proxy");
    }
    void onDisconnect(BLEServer*) override {
        appConectado = false;
        Serial.println("[APP] desconectou - anunciando de novo");
        BLEDevice::startAdvertising();
    }
};

// App escreveu numa caracteristica -> enfileira para enviar a BMS
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

// BMS enviou notificacao -> enfileira para repassar ao app
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

// ── SETUP ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println();
    Serial.println("=== BMS PROXY ===");
    Serial.println("Deixe o app LanmaoBMS FECHADO e o outro ESP32 DESLIGADO.");

    fila = xQueueCreate(32, sizeof(Evt));

    BLEDevice::init("BMS-PROXY");

    // 1) Procurar a BMS real
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&cbScan, true);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    for (int t = 0; t < 4 && !alvo; t++) {
        Serial.printf("Procurando BMS %s (tentativa %d)...\n", BMS_MAC, t + 1);
        scan->start(8, false);
        scan->clearResults();
    }
    if (!alvo) {
        Serial.println("ERRO: BMS nao encontrada. Reiniciando em 5s.");
        delay(5000);
        ESP.restart();
    }
    Serial.printf("BMS encontrada. Nome anunciado: '%s'  UUID anunciado: '%s'\n",
                  nomeBMS.c_str(), uuidAdv.c_str());

    // 2) Conectar na BMS real
    cliente = BLEDevice::createClient();
    cliente->setClientCallbacks(&cbCliente);
    Serial.println("Conectando na BMS real...");
    if (!cliente->connect(alvo)) {
        Serial.println("ERRO: falha ao conectar na BMS. Reiniciando em 5s.");
        delay(5000);
        ESP.restart();
    }
    bmsOnline = true;
    Serial.println("Conectado na BMS real.");

    // 3) Copiar servicos/caracteristicas
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
        if (su.indexOf("1800") >= 0 || su.indexOf("1801") >= 0) continue;   // servicos padrao

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

            // Valor inicial (leitura unica) para o app poder ler
            if (rc->canRead()) {
                auto v = rc->readValue();
                lc->setValue((uint8_t*)v.c_str(), v.length());
                Serial.print("    valor lido: ");
                dump((const uint8_t*)v.c_str(), v.length());
            }

            mapa[nMapa].remoto     = rc;
            mapa[nMapa].local      = lc;
            mapa[nMapa].respRemota = (rc->canWrite() && !rc->canWriteNoResponse());
            nMapa++;
        }
        ls->start();
    }

    // Assina as notificacoes da BMS real
    for (uint8_t i = 0; i < nMapa; i++) {
        BLERemoteCharacteristic* rc = mapa[i].remoto;
        if (!(rc->canNotify() || rc->canIndicate())) continue;
        rc->registerForNotify(cbNotifyRemoto);
        BLERemoteDescriptor* d = rc->getDescriptor(BLEUUID((uint16_t)0x2902));
        if (d) {
            uint8_t v[2] = { (uint8_t)(rc->canNotify() ? 0x01 : 0x02), 0x00 };
            d->writeValue(v, 2, true);
        }
        Serial.printf("Assinado: %s\n", uuidStr(rc->getUUID()).c_str());
    }

    // 4) Anunciar como a BMS
    String nome = String(NOME_FORCADO);
    if (nome.length() == 0) nome = nomeBMS;
    if (nome.length() == 0) nome = "BMS";
    esp_ble_gap_set_device_name(nome.c_str());

    String uAdv = (uuidAdv.length() ? uuidAdv : primeiroSvc);
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    if (uAdv.length()) adv->addServiceUUID(BLEUUID(uAdv));
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println();
    Serial.println("=======================================================");
    Serial.printf ("PROXY PRONTO. Anunciando como: '%s'\n", nome.c_str());
    Serial.println("Abra o app LanmaoBMS no iPhone e conecte nesse nome.");
    Serial.println("Navegue por TODAS as telas (dados, versao, celulas...).");
    Serial.println("Depois copie TUDO deste Serial Monitor e me envie.");
    Serial.println("=======================================================");
}

// ── LOOP ─────────────────────────────────────────────────
void loop() {
    Evt e;
    while (xQueueReceive(fila, &e, 0) == pdTRUE) {
        Mapa& m = mapa[e.idx];
        String u = uuidStr(m.remoto->getUUID());

        if (e.tipo == 1) {
            Serial.printf("[%lu] APP->BMS  %s (%uB): ", (unsigned long)e.t, u.c_str(), e.len);
            dump(e.dados, e.len);
            if (bmsOnline) m.remoto->writeValue(e.dados, e.len, m.respRemota);
        } else {
            Serial.printf("[%lu] BMS->APP  %s (%uB): ", (unsigned long)e.t, u.c_str(), e.len);
            dump(e.dados, e.len);
            if (appConectado) {
                m.local->setValue(e.dados, e.len);
                m.local->notify();
            }
        }
    }

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
