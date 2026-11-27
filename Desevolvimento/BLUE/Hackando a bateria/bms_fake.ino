/*
 * ════════════════════════════════════════════════════════════
 * BMS FAKE - finge ser a bateria para MAPEAR os registradores
 * Rodar em um SEGUNDO ESP32 (nao no sistema de energia).
 *
 * Protocolo (descoberto na captura):
 *   Pedido  (app -> FFFB): A5 A5 | 00 03 | REG(2) | QTD(2) | CRC(2)
 *   Resposta (FFFC):       A5 A5 | 00 03 | NBYTES | dados... | CRC(2)
 *   (Modbus RTU com prefixo A5A5, CRC16 Modbus, lo/hi)
 *
 * Perfis de registradores:
 *   padrao : cada registrador vale o proprio numero (reg 24 = 24)
 *   real   : so os valores que vimos na captura da bateria
 *   misto  : (inicial) padrao + valores conhecidos DIFERENTES dos reais
 *
 * Comandos pelo Serial Monitor (115200, "Nova linha"):
 *   padrao | real | misto | dump
 *   r <reg> <valor>    define 1 registrador (16 bits, aceita negativo)
 *   l <reg> <valor>    define 32 bits em reg (alto) e reg+1 (baixo)
 *   ver <reg>          mostra o valor
 *
 * Placa: ESP32 Dev Module | Particao: "Huge APP (3MB No OTA)"
 * ════════════════════════════════════════════════════════════
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3)
#include <BLE2902.h>
#endif

// ── CONFIG ───────────────────────────────────────────────
#define MTU_LOCAL           247
#define N_REGS              256
#define PERFIL_INICIAL      2        // 0=padrao 1=real 2=misto
#define FORCAR_PACOTES_20   0        // 1 = sempre envia em pacotes de 20 bytes

// Anuncio clonado da BMS real, com o MAC alterado (ultimo byte 0x33 -> 0x44)
// para o app mostrar este dispositivo diferente da bateria verdadeira.
static const uint8_t ADV_RAW[30] = {
    0x02,0x01,0x06,
    0x03,0x16,0x0A,0x18,
    0x03,0x03,0xFA,0xFF,
    0x07,0xFF, 0x44,0x03,0x20,0x20,0x60,0xA2,
    0x0A,0x09, 0x95,0x03,0x00,0x00,0x00,0x00,0x06,0x03,0xCB
};

// ── ESTADO ───────────────────────────────────────────────
uint16_t regs[N_REGS];

BLEServer*         servidor = nullptr;
BLECharacteristic* cFFFB = nullptr;
BLECharacteristic* cFFFC = nullptr;
BLECharacteristic* c2    = nullptr;   // 11110002
BLECharacteristic* c3    = nullptr;   // 11110003
volatile bool      appConectado = false;

struct Req { uint8_t src; uint8_t len; uint8_t d[40]; };
QueueHandle_t fila;

// ── UTIL ─────────────────────────────────────────────────
static uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static void hex(const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; i++) Serial.printf("%02X", d[i]);
    Serial.println();
}

// ── PERFIS ───────────────────────────────────────────────
static void perfilPadrao() {
    for (int i = 0; i < N_REGS; i++) regs[i] = (uint16_t)i;
    // palavras ALTAS de valores de 32 bits (vimos 0000 nelas): ficam 0
    regs[0] = regs[2] = regs[6] = 0;
    regs[60] = regs[62] = regs[64] = 0;
    regs[113] = 0;
}

static void aplicarConhecidos(bool distintos) {
    // reg 0-1: int32 (corrente?)   reg 2-3: int32 (tensao?)
    // reg 4: percentual?  reg 5: percentual?
    if (!distintos) {
        regs[0] = 0xFFFF; regs[1] = 0xFE93;      // -365
        regs[2] = 0;      regs[3] = 1323;        // 13.23 V ?
        regs[4] = 76;     regs[5] = 100;
        regs[60] = 0; regs[61] = 14000;
        regs[62] = 0; regs[63] = 14600;
        regs[64] = 0; regs[65] = 13320;
        regs[66] = 10;
        static const uint16_t cel[7] = {3230,3330,3230,3280,3380,3280,2780};
        for (int i = 0; i < 7; i++) regs[90 + i] = cel[i];
        static const uint16_t par[7] = {3400,30,0,14000,2000,3150,5};
        for (int i = 0; i < 7; i++) regs[111 + i] = par[i];
    } else {
        // mesmos lugares, valores DIFERENTES (para nao confundir com a bateria real)
        regs[0] = 0xFFFF; regs[1] = (uint16_t)(int16_t)-123;   // -123
        regs[2] = 0;      regs[3] = 1234;                      // 12.34 V ?
        regs[4] = 55;     regs[5] = 99;
        regs[60] = 0; regs[61] = 15000;
        regs[62] = 0; regs[63] = 15500;
        regs[64] = 0; regs[65] = 12300;
        regs[66] = 11;
        static const uint16_t cel[7] = {3101,3102,3103,3104,3105,3106,3107};
        for (int i = 0; i < 7; i++) regs[90 + i] = cel[i];
        static const uint16_t par[7] = {3399,29,0,15001,2001,3151,6};
        for (int i = 0; i < 7; i++) regs[111 + i] = par[i];
    }
}

static void carregarPerfil(int p) {
    if (p == 0)      { perfilPadrao(); }
    else if (p == 1) { memset(regs, 0, sizeof(regs)); aplicarConhecidos(false); }
    else             { perfilPadrao(); aplicarConhecidos(true); }
}

static void dumpRegs() {
    Serial.println("--- registradores (0..127) ---");
    for (int i = 0; i < 128; i += 8) {
        Serial.printf("%3d: ", i);
        for (int k = 0; k < 8; k++) Serial.printf("%6u ", regs[i + k]);
        Serial.println();
    }
}

// ── ENVIO DA RESPOSTA ────────────────────────────────────
static void enviarFrame(const uint8_t* f, size_t n, unsigned reg, unsigned qtd) {
    if (!appConectado) return;
    uint16_t mtu = servidor->getPeerMTU(servidor->getConnId());
    size_t maxPay = (mtu > 23) ? (size_t)(mtu - 3) : 20;
    if (FORCAR_PACOTES_20) maxPay = 20;
    size_t pacotes = (n + maxPay - 1) / maxPay;

    Serial.printf("REQ reg=%u qtd=%u -> resposta %uB em %u pacote(s) (MTU %u)\n",
                  reg, qtd, (unsigned)n, (unsigned)pacotes, (unsigned)mtu);

    for (size_t off = 0; off < n; off += maxPay) {
        size_t c = (n - off < maxPay) ? (n - off) : maxPay;
        cFFFC->setValue((uint8_t*)(f + off), c);
        cFFFC->notify();
        if (pacotes > 1) delay(20);
    }
}

static void tratarModbus(const uint8_t* d, size_t n) {
    if (n < 10 || d[0] != 0xA5 || d[1] != 0xA5) {
        Serial.print("REQ desconhecida: "); hex(d, n);
        return;
    }
    uint8_t addr = d[2], func = d[3];
    if (func != 0x03) {
        Serial.printf("REQ func=%02X (nao tratada): ", func); hex(d, n);
        return;
    }
    uint16_t c = crc16(d + 2, 6);
    if (d[8] != (c & 0xFF) || d[9] != (c >> 8)) {
        Serial.print("REQ com CRC invalido, ignorada: "); hex(d, n);
        return;
    }
    unsigned reg = (d[4] << 8) | d[5];
    unsigned qtd = (d[6] << 8) | d[7];
    if (qtd == 0 || qtd > 120) {
        Serial.printf("REQ qtd invalida (%u)\n", qtd);
        return;
    }

    uint8_t f[256];
    size_t  k = 0;
    f[k++] = 0xA5; f[k++] = 0xA5; f[k++] = addr; f[k++] = 0x03;
    f[k++] = (uint8_t)(qtd * 2);
    for (unsigned i = 0; i < qtd; i++) {
        uint16_t v = (reg + i < N_REGS) ? regs[reg + i] : 0;
        f[k++] = (uint8_t)(v >> 8);
        f[k++] = (uint8_t)(v & 0xFF);
    }
    uint16_t cr = crc16(f + 2, k - 2);
    f[k++] = (uint8_t)(cr & 0xFF);
    f[k++] = (uint8_t)(cr >> 8);

    enviarFrame(f, k, reg, qtd);
}

// ── CALLBACKS BLE ────────────────────────────────────────
class CbServidor : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        appConectado = true;
        Serial.println("[APP] conectou");
    }
    void onDisconnect(BLEServer*) override {
        appConectado = false;
        Serial.println("[APP] desconectou - anunciando de novo");
        BLEDevice::startAdvertising();
    }
};

class CbChar : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        Req r;
        r.src = (c == cFFFB) ? 1 : (c == c2) ? 2 : 3;
        size_t n = c->getLength();
        if (n > sizeof(r.d)) n = sizeof(r.d);
        memcpy(r.d, c->getData(), n);
        r.len = (uint8_t)n;
        xQueueSend(fila, &r, 0);
    }
};

static CbServidor cbServidor;
static CbChar     cbChar;

// ── COMANDOS PELO SERIAL ─────────────────────────────────
static void comando(String s) {
    s.trim();
    if (s.length() == 0) return;

    unsigned r; long v;
    if (s == "padrao")      { carregarPerfil(0); Serial.println("perfil PADRAO carregado"); }
    else if (s == "real")   { carregarPerfil(1); Serial.println("perfil REAL carregado"); }
    else if (s == "misto")  { carregarPerfil(2); Serial.println("perfil MISTO carregado"); }
    else if (s == "dump")   { dumpRegs(); }
    else if (sscanf(s.c_str(), "r %u %ld", &r, &v) == 2 && r < N_REGS) {
        regs[r] = (uint16_t)(v & 0xFFFF);
        Serial.printf("reg %u = %u\n", r, regs[r]);
    }
    else if (sscanf(s.c_str(), "l %u %ld", &r, &v) == 2 && r + 1 < N_REGS) {
        regs[r]     = (uint16_t)((v >> 16) & 0xFFFF);
        regs[r + 1] = (uint16_t)(v & 0xFFFF);
        Serial.printf("reg %u,%u = %ld (32 bits)\n", r, r + 1, v);
    }
    else if (sscanf(s.c_str(), "ver %u", &r) == 1 && r < N_REGS) {
        Serial.printf("reg %u = %u\n", r, regs[r]);
    }
    else Serial.println("comando invalido. Use: padrao | real | misto | dump | r <reg> <val> | l <reg> <val> | ver <reg>");
}

static void lerSerial() {
    static String linha;
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (linha.length()) { comando(linha); linha = ""; }
        } else if (linha.length() < 60) {
            linha += ch;
        }
    }
}

// ── SETUP ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println();
    Serial.println("=== BMS FAKE ===");

    fila = xQueueCreate(16, sizeof(Req));
    carregarPerfil(PERFIL_INICIAL);

    BLEDevice::init("BMS-FAKE");
    BLEDevice::setMTU(MTU_LOCAL);

    servidor = BLEDevice::createServer();
    servidor->setCallbacks(&cbServidor);

    // Servico FFFA (o que o app realmente usa)
    BLEService* sA = servidor->createService(BLEUUID("0000fffa-0000-1000-8000-00805f9b34fb"));
    cFFFB = sA->createCharacteristic(BLEUUID("0000fffb-0000-1000-8000-00805f9b34fb"),
                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
    cFFFC = sA->createCharacteristic(BLEUUID("0000fffc-0000-1000-8000-00805f9b34fb"),
                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3)
    cFFFC->addDescriptor(new BLE2902());
#endif
    cFFFB->setValue((uint8_t*)"BLE BMS", 7);
    cFFFC->setValue((uint8_t*)"BLE BMS", 7);
    cFFFB->setCallbacks(&cbChar);
    sA->start();

    // Servico 11110001 (existe na BMS real; so registramos o que o app escrever)
    BLEService* sB = servidor->createService(BLEUUID("11110001-1111-1111-1111-111111111111"));
    c2 = sB->createCharacteristic(BLEUUID("11110002-1111-1111-1111-111111111111"),
                BLECharacteristic::PROPERTY_WRITE_NR);
    c3 = sB->createCharacteristic(BLEUUID("11110003-1111-1111-1111-111111111111"),
                BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3)
    c3->addDescriptor(new BLE2902());
#endif
    c2->setCallbacks(&cbChar);
    c3->setCallbacks(&cbChar);
    sB->start();

    // Anuncio clonado
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    BLEAdvertisementData ad;
    ad.addData((char*)ADV_RAW, sizeof(ADV_RAW));
    adv->setAdvertisementData(ad);
    adv->setScanResponse(false);
    BLEDevice::startAdvertising();

    Serial.println("Anunciando como BMS falsa (MAC fake termina em ...03:44).");
    Serial.println("Abra o app, escolha na lista e conecte.");
    Serial.println("Comandos: padrao | real | misto | dump | r <reg> <val> | l <reg> <val> | ver <reg>");
    Serial.printf("Perfil inicial: %s\n",
                  PERFIL_INICIAL == 0 ? "padrao" : PERFIL_INICIAL == 1 ? "real" : "misto");
}

// ── LOOP ─────────────────────────────────────────────────
void loop() {
    lerSerial();

    Req r;
    while (xQueueReceive(fila, &r, 0) == pdTRUE) {
        if (r.src == 1) {
            tratarModbus(r.d, r.len);
        } else {
            Serial.printf("APP escreveu em 1111000%d (%uB): ", r.src, r.len);
            hex(r.d, r.len);
        }
    }
    delay(2);
}
