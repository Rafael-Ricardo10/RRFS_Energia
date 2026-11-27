/*
 * ════════════════════════════════════════════════════════════
 * BMS Teste v4 - testa ser a bateria para MAPEAR os registradores
 * Rodar em um SEGUNDO ESP32 (nao no sistema de energia).
 *
 * MAPA CONFIRMADO ate agora (bloco 0, regs 0..71):
 *   regs 0-1  : corrente  (int32, 0.01 A, negativo = descarga)   [confirmado]
 *   regs 2-3  : tensao    (int32, 0.01 V)                        [confirmado]
 *   reg  4    : SOC       (%)                                    [confirmado]
 *   reg  12   : ciclos                                           [provavel]
 *   regs 10-11: capacidade (int32, 0.01 Ah)                      [provavel]
 *
 * v4:
 *   - Envio automatico DESLIGADO (causava a oscilacao 55% / 200%)
 *   - Perfil misto ja com os campos mapeados
 *   - Comandos nomeados:  tensao 13.5 | corrente -2.5 | soc 80 |
 *                         ciclos 99   | cap 140.0
 *
 * Protocolo:
 *   Pedido  (app -> FFFB): A5 A5 | 00 03 | REG(2) | QTD(2) | CRC(2)
 *   Resposta (FFFC):       A5 A5 | 00 03 | NBYTES | dados... | CRC(2)
 *
 * Comandos pelo Serial Monitor (115200, "Nova linha"):
 *   padrao | real | misto | dump
 *   tensao <V> | corrente <A> | soc <%> | ciclos <n> | cap <Ah>
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
#if __has_include(<BLE2902.h>)
#include <BLE2902.h>
#define TEM_2902 1
#endif

// ── CONFIG ───────────────────────────────────────────────
#define MTU_LOCAL           185
#define N_REGS              256
#define PERFIL_INICIAL      2        // 0=padrao 1=real 2=misto
#define FORCAR_PACOTES_20   0        // 1 = sempre envia em pacotes de 20 bytes
#define MAC_BYTE_FINAL      0x33     // 0x33 = igual a BMS real | 0x44 = diferente
#define PUSH_AUTOMATICO     0        // 0 = so responde aos pedidos (recomendado)
#define PUSH_MS             1500

// Anuncio clonado da BMS real
static const uint8_t ADV_RAW[30] = {
    0x02,0x01,0x06,
    0x03,0x16,0x0A,0x18,
    0x03,0x03,0xFA,0xFF,
    0x07,0xFF, MAC_BYTE_FINAL,0x03,0x20,0x20,0x60,0xA2,
    0x0A,0x09, 0x95,0x03,0x00,0x00,0x00,0x00,0x06,0x03,0xCB
};

// ── ESTADO ───────────────────────────────────────────────
uint16_t regs[N_REGS];

BLEServer*         servidor = nullptr;
BLECharacteristic* cFFFB = nullptr;
BLECharacteristic* cFFFC = nullptr;
BLECharacteristic* c2    = nullptr;   // 11110002
BLECharacteristic* c3    = nullptr;   // 11110003
#ifdef TEM_2902
BLE2902* cccdC = nullptr;
BLE2902* cccd3 = nullptr;
#endif
volatile bool          appConectado = false;
volatile unsigned long tConexao = 0;

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

// Grava um inteiro de 32 bits em dois registradores (alto, baixo)
static void setL32(unsigned r, int32_t v) {
    uint32_t u = (uint32_t)v;
    regs[r]     = (uint16_t)(u >> 16);
    regs[r + 1] = (uint16_t)(u & 0xFFFF);
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
    if (!distintos) {
        // Valores vistos na bateria real (captura)
        setL32(0, -365);          // corrente
        setL32(2, 1323);          // tensao 13.23 V
        regs[4] = 76;             // SOC
        regs[5] = 100;
        regs[60] = 0; regs[61] = 14000;
        regs[62] = 0; regs[63] = 14600;
        regs[64] = 0; regs[65] = 13320;
        regs[66] = 10;
        static const uint16_t cel[7] = {3230,3330,3230,3280,3380,3280,2780};
        for (int i = 0; i < 7; i++) regs[90 + i] = cel[i];
        static const uint16_t par[7] = {3400,30,0,14000,2000,3150,5};
        for (int i = 0; i < 7; i++) regs[111 + i] = par[i];
    } else {
        // Valores DIFERENTES dos reais, para reconhecer a bateria falsa
        setL32(0, -123);          // -1.23 A
        setL32(2, 1234);          // 12.34 V
        regs[4]  = 55;            // SOC 55 %
        regs[5]  = 99;
        setL32(10, 14000);        // capacidade 140.00 Ah (a confirmar)
        regs[12] = 37;            // ciclos 37 (a confirmar)
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
static void enviarFrame(const uint8_t* f, size_t n, unsigned reg, unsigned qtd, const char* rotulo) {
    if (!appConectado) return;
    uint16_t mtu = servidor->getPeerMTU(servidor->getConnId());
    size_t maxPay = (mtu > 23) ? (size_t)(mtu - 3) : 20;
    if (FORCAR_PACOTES_20) maxPay = 20;
    size_t pacotes = (n + maxPay - 1) / maxPay;

    Serial.printf("%s reg=%u qtd=%u -> %uB em %u pacote(s) (MTU %u)\n",
                  rotulo, reg, qtd, (unsigned)n, (unsigned)pacotes, (unsigned)mtu);

    for (size_t off = 0; off < n; off += maxPay) {
        size_t c = (n - off < maxPay) ? (n - off) : maxPay;
        cFFFC->setValue((uint8_t*)(f + off), c);
        cFFFC->notify();
        if (pacotes > 1) delay(20);
    }
}

static void responder(uint8_t addr, unsigned reg, unsigned qtd, const char* rotulo) {
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
    enviarFrame(f, k, reg, qtd, rotulo);
}

static void tratarModbus(const uint8_t* d, size_t n) {
    if (n < 10 || d[0] != 0xA5 || d[1] != 0xA5) {
        Serial.print("REQ desconhecida: "); hex(d, n);
        return;
    }
    uint8_t func = d[3];
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
    responder(d[2], reg, qtd, "REQ ");
}

// Envio automatico (desligado por padrao: confunde o app)
static void pushPeriodico() {
    if (!PUSH_AUTOMATICO || !appConectado) return;
    if (millis() - tConexao < 3000) return;

    static unsigned long t = 0;
    static uint8_t i = 0;
    if (millis() - t < PUSH_MS) return;
    t = millis();

    static const uint16_t blocos[4][2] = { {0,72}, {60,22}, {90,23}, {111,11} };
    responder(0x00, blocos[i][0], blocos[i][1], "PUSH");
    i = (i + 1) & 3;
}

// ── CALLBACKS BLE ────────────────────────────────────────
class CbServidor : public BLEServerCallbacks {
    void onConnect(BLEServer*, esp_ble_gatts_cb_param_t* p) override {
        appConectado = true;
        tConexao = millis();
        Serial.printf("[APP] conectou (conn_id=%u)\n", (unsigned)p->connect.conn_id);
    }
    void onDisconnect(BLEServer*, esp_ble_gatts_cb_param_t* p) override {
        appConectado = false;
        Serial.printf("[APP] desconectou apos %lu ms, motivo=0x%02X - anunciando de novo\n",
                      millis() - tConexao, (unsigned)p->disconnect.reason);
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
    void onRead(BLECharacteristic* c) override {
        Serial.printf("[APP] leu %s\n", (c == cFFFB) ? "FFFB" : (c == cFFFC) ? "FFFC" : "outra");
    }
};

static CbServidor cbServidor;
static CbChar     cbChar;

// ── COMANDOS PELO SERIAL ─────────────────────────────────
static void comando(String s) {
    s.trim();
    if (s.length() == 0) return;

    unsigned r; long v; float f;
    if (s == "padrao")      { carregarPerfil(0); Serial.println("perfil PADRAO carregado"); }
    else if (s == "real")   { carregarPerfil(1); Serial.println("perfil REAL carregado"); }
    else if (s == "misto")  { carregarPerfil(2); Serial.println("perfil MISTO carregado"); }
    else if (s == "dump")   { dumpRegs(); }
    else if (sscanf(s.c_str(), "tensao %f", &f) == 1) {
        setL32(2, (int32_t)lroundf(f * 100.0f));
        Serial.printf("tensao = %.2f V (regs 2-3 = %u,%u)\n", f, regs[2], regs[3]);
    }
    else if (sscanf(s.c_str(), "corrente %f", &f) == 1) {
        setL32(0, (int32_t)lroundf(f * 100.0f));
        Serial.printf("corrente = %.2f A (regs 0-1 = %u,%u)\n", f, regs[0], regs[1]);
    }
    else if (sscanf(s.c_str(), "soc %f", &f) == 1) {
        regs[4] = (uint16_t)lroundf(f);
        Serial.printf("soc = %u %% (reg 4)\n", regs[4]);
    }
    else if (sscanf(s.c_str(), "ciclos %ld", &v) == 1) {
        regs[12] = (uint16_t)(v & 0xFFFF);
        Serial.printf("ciclos = %u (reg 12)\n", regs[12]);
    }
    else if (sscanf(s.c_str(), "cap %f", &f) == 1) {
        setL32(10, (int32_t)lroundf(f * 100.0f));
        Serial.printf("capacidade = %.2f Ah (regs 10-11 = %u,%u)\n", f, regs[10], regs[11]);
    }
    else if (sscanf(s.c_str(), "r %u %ld", &r, &v) == 2 && r < N_REGS) {
        regs[r] = (uint16_t)(v & 0xFFFF);
        Serial.printf("reg %u = %u\n", r, regs[r]);
    }
    else if (sscanf(s.c_str(), "l %u %ld", &r, &v) == 2 && r + 1 < N_REGS) {
        setL32(r, (int32_t)v);
        Serial.printf("reg %u,%u = %ld (32 bits)\n", r, r + 1, v);
    }
    else if (sscanf(s.c_str(), "ver %u", &r) == 1 && r < N_REGS) {
        Serial.printf("reg %u = %u\n", r, regs[r]);
    }
    else Serial.println("comando invalido. Use: padrao | real | misto | dump | tensao | corrente | soc | ciclos | cap | r | l | ver");
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
    Serial.println("=== BMS Teste v4 ===");

    fila = xQueueCreate(16, sizeof(Req));
    carregarPerfil(PERFIL_INICIAL);

    BLEDevice::init("BMS-Teste");
    BLEDevice::setMTU(MTU_LOCAL);

    servidor = BLEDevice::createServer();
    servidor->setCallbacks(&cbServidor);

    // Servico FFFA (o que o app realmente usa) - UUIDs de 16 bits
    BLEService* sA = servidor->createService(BLEUUID((uint16_t)0xFFFA));
    cFFFB = sA->createCharacteristic(BLEUUID((uint16_t)0xFFFB),
                                     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
    cFFFC = sA->createCharacteristic(BLEUUID((uint16_t)0xFFFC),
                                     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    #ifdef TEM_2902
    cccdC = new BLE2902();
    cFFFC->addDescriptor(cccdC);
    #else
    Serial.println("AVISO: BLE2902.h nao encontrado neste core!");
    #endif
    cFFFB->setValue((uint8_t*)"BLE BMS", 7);
    cFFFC->setValue((uint8_t*)"BLE BMS", 7);
    cFFFB->setCallbacks(&cbChar);
    cFFFC->setCallbacks(&cbChar);
    sA->start();

    // Servico 11110001 (existe na BMS real) - UUIDs de 128 bits
    BLEService* sB = servidor->createService(BLEUUID("11110001-1111-1111-1111-111111111111"));
    c2 = sB->createCharacteristic(BLEUUID("11110002-1111-1111-1111-111111111111"),
                                  BLECharacteristic::PROPERTY_WRITE_NR);
    c3 = sB->createCharacteristic(BLEUUID("11110003-1111-1111-1111-111111111111"),
                                  BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    #ifdef TEM_2902
    cccd3 = new BLE2902();
    c3->addDescriptor(cccd3);
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

    Serial.printf("Anunciando como BMS falsa (MAC no anuncio: A2:60:20:20:03:%02X).\n", MAC_BYTE_FINAL);
    Serial.println("Confira no app:  12.34 V / SOC 55% / 140.00 Ah / 37 ciclos = FALSA");
    Serial.println("Comandos: tensao <V> | corrente <A> | soc <%> | ciclos <n> | cap <Ah> | dump | padrao | real | misto");
    Serial.printf("Perfil inicial: %s | envio automatico: %s\n",
                  PERFIL_INICIAL == 0 ? "padrao" : PERFIL_INICIAL == 1 ? "real" : "misto",
                  PUSH_AUTOMATICO ? "SIM" : "NAO");
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

    #ifdef TEM_2902
    // Detecta o app ativando/desativando notify
    static bool subC = false, sub3 = false;
    if (!appConectado) {
        if (cccdC && cccdC->getNotifications()) cccdC->setNotifications(false);
        if (cccd3 && cccd3->getNotifications()) cccd3->setNotifications(false);
    }
    bool nC = cccdC && cccdC->getNotifications();
    bool n3 = cccd3 && cccd3->getNotifications();
    if (nC != subC) { subC = nC; Serial.printf("[APP] notify FFFC %s\n", nC ? "ATIVADO" : "DESATIVADO"); }
    if (n3 != sub3) { sub3 = n3; Serial.printf("[APP] notify 11110003 %s\n", n3 ? "ATIVADO" : "DESATIVADO"); }
    #endif

    pushPeriodico();

    static unsigned long tHb = 0;
    if (millis() - tHb > 15000UL) {
        tHb = millis();
        Serial.printf("[hb] app=%d\n", (int)appConectado);
    }
    delay(2);
}
