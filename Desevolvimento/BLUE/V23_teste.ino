/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v23 CORRIGIDO - INVERSOR ALTA + BMS BLE + BANCO
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 * v23 CORRIGIDO:
 *   - REMOVIDOS cálculos desnecessários de SOC% e ciclos
 *   - BMS fornece SOC%, ciclos e capacidade DIRETAMENTE
 *   - Registradores corrigidos conforme mapa BMS
 *   - Simplificação do parser Modbus
 *   - Histórico mantém Ah acumulado apenas
 * ════════════════════════════════════════════════════════════
 */

#include <HTTPClient.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <Update.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_heap_caps.h>
#include <esp_bt.h>

// ═══════════════════════════════════════════════════════════
// [HW] PINOS
// ═══════════════════════════════════════════════════════════
#define LED_AZUL               2
#define BOTAO_BOOT             0
#define PIN_SENSOR_PAINEL     34
#define PIN_AC_SINAL_ALTA     33
#define PIN_RELE_INV_ALTA     27
#define PIN_RELE_LAMP_EXT     12

// ═══════════════════════════════════════════════════════════
// [CFG] CONSTANTES GERAIS v23
// ═══════════════════════════════════════════════════════════
#define DNS_PORT               53
#define TEMPO_RESET_MS      10000
#define MAX_LOGS              100
#define LOG_MAX_LEN            64
#define LOG_NVS_NS        "rrfs_log"

// ADC
#define ADC_VREF             3300
#define ADC_BITS             4095

// Sensor corrente painel
#define PAINEL_OFFSET_MV     2500
#define PAINEL_SENS_MV_A     35.9f
#define FILTRO_RUIDO_A        0.15f
#define BUFFER_CORRENTE        10

// ═══════════════════════════════════════════════════════════
// [BLE] CONSTANTES BMS JBD/LanmaoBMS
// ═══════════════════════════════════════════════════════════
#define BMS_SERVICE_UUID  "0000ff00-0000-1000-8000-00805f9b34fb"
#define BMS_RX_UUID       "0000ff01-0000-1000-8000-00805f9b34fb"
#define BMS_TX_UUID       "0000ff02-0000-1000-8000-00805f9b34fb"

// Comando JBD: solicitar dados básicos (registro 0x03)
static const uint8_t BMS_CMD_BASICO[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};

#define BLE_RECONECT_MS    15000UL
#define BLE_TIMEOUT_MS      8000UL
#define BLE_LEITURA_MS      1000UL
#define BLE_SCAN_DURACAO_S     5
#define BMS_FILTRO_RUIDO_A  0.1f
#define BMS_DADO_VALIDO_MS 10000UL

// ═══════════════════════════════════════════════════════════
// [AC] CONSTANTES SINAL AC
// ═══════════════════════════════════════════════════════════
#define AC_JANELA_MS            40
#define AC_BUFFER_SUAV           6
#define AC_CONFIRMA_MS         400UL
#define AC_HISTERESE_LIGA_MV    50
#define AC_HISTERESE_DESLIGA_MV 40
#define AC_LIMIAR_MV_DEFAULT   400

#define DIAG_AGUARDA_AC_MS   8000UL
#define DIAG_DESLIGA_MS      5000UL
#define DIAG_MAX_TENTATIVAS     3

#define LAMP_EXT_HORA_LIGA      19
#define LAMP_EXT_MIN_LIGA        0
#define LAMP_EXT_HORA_DESLIGA    5
#define LAMP_EXT_MIN_DESLIGA    30
#define LAMP_EXT_CHECK_MS    60000UL

#define INTERVALO_LEITURA_MS   500UL
#define INTERVALO_CONTROLE_MS 1000UL
#define INTERVALO_NTP_MS    3600000UL
#define INTERVALO_PREVISAO_MS 3600000UL

#define WIFI_RECON_INTERVALO  30000UL
#define WIFI_RECON_TIMEOUT    10000UL
#define WIFI_CHECK_INTERVALO  15000UL
#define WIFI_MAX_FALHAS            5

#define HIST_DIAS              30
#define HIST_HORAS             24
#define HIST_KEY_VER           27
#define HIST_SNAP_PONTOS       60

#define PREV_DIAS               7
#define PREV_TOTAL_HORAS      168

#define NTP_SERVER1       "pool.ntp.org"
#define NTP_SERVER2       "time.google.com"

#define BLE_LISTA_MAX      20
#define BLE_SCAN_LISTA_S    8

const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ═══════════════════════════════════════════════════════════
// [STR] STRUCTS v23 SIMPLIFICADO
// ═══════════════════════════════════════════════════════════

// ── Configuração banco de baterias ────────────────────────
struct ConfigBanco {
    uint8_t nS;          // baterias em série
    uint8_t nP;          // baterias em paralelo
    bool    configurado;
};

// ── Dados da BMS (SIMPLIFICADO v23) ──────────────────────
// Agora a BMS fornece:
// - Corrente (regs 0-1): 32 bits, 0.01 A
// - Tensão (regs 2-3): 32 bits, 0.01 V
// - SOC (reg 4): % direto
// - Ciclos (reg 12): direto
// - Capacidade (regs 10-11): 32 bits, 0.01 Ah
struct DadosBMS {
    float         corrente;      // A por string
    float         tensao;        // V da BMS
    float         temperatura;   // C (25.0 por enquanto)
    float         soc;           // % direto da BMS
    uint16_t      ciclos;
    bool          valido;
    unsigned long tUltimaDado;
    float         capacidadeAh;  // regs 10-11 ← NOVA
};

// ── Dados calculados do banco ────────────────────────────
struct DadosBanco {
    float tensaoTotal;    // V = tensao_bms × nS
    float correnteTotal;  // A = corrente_bms × nP
    float potencia;       // W = tensaoTotal × correnteTotal
    float socPct;         // % direto da BMS
    uint16_t ciclos;      // direto da BMS
    float capacidadeTotal; // Ah do banco = capacidade_bms × nP (NOVO)
};

// ── Estado BLE ────────────────────────────────────────────
enum EstadoBLE {
    BLE_DESCONECTADO = 0,
    BLE_ESCANEANDO,
    BLE_CONECTANDO,
    BLE_CONECTADO,
    BLE_FALHA
};

struct BleItem {
    char mac[18];
    char nome[24];
    int  rssi;
    bool jbd;
};

struct ConfigBMS {
    char   macAddress[18];
    bool   configurada;
};

struct EstadoBLECtrl {
    EstadoBLE     estado;
    unsigned long tUltimoTentativa;
    unsigned long tUltimaLeitura;
    unsigned long tConectando;
    bool          notificacaoAtiva;
    uint8_t       falhasConsec;
    uint8_t       bufferRX[64];
    uint8_t       bufferLen;
};

// ── Sensor painel ─────────────────────────────────────────
struct CalibSensor {
    int   offsetMV;
    float sensMvA;
    bool  calibrado;
};

struct BufferCorr {
    float   amostras[BUFFER_CORRENTE];
    uint8_t indice;
    uint8_t total;
};

// ── Sensores consolidados ─────────────────────────────────
struct DadosSensores {
    float correntePainel;    // A (ADC)
    float correnteBateria;   // A banco total
    float correnteCasa;      // A calculada
    float potenciaPainel;    // W
    float potenciaBateria;   // W (banco)
    float potenciaCasa;      // W
    float tensaoBanco;       // V (BMS × nS)
    float temperaturaBMS;    // °C
    float socBMS;            // % (direto BMS)
    uint16_t ciclosBMS;      // (direto BMS)
    float capacidadeBanco;   // Ah do banco NOVO
    int   sinalPainel_mV;
};

// ── Relés e inversores ────────────────────────────────────
struct EstadoReles {
    bool lampExt;
    bool invAlta;
};

struct ConfigInversores {
    bool invertReleInvAlta;
    bool invertReleLampExt;
};

// ── Sinal AC ──────────────────────────────────────────────
struct SinalAC {
    float         mediaMV;
    bool          presente;
    bool          aguardandoLigar;
    bool          aguardandoDesligar;
    unsigned long tInicioConfirma;
};

// ── Máquina inversor alta ─────────────────────────────────
enum EstadoAlta {
    INV_A_DESLIGADO = 0,
    INV_A_LIGADO,
    INV_A_DIAG_AGUARDA,
    INV_A_DIAG_OFF,
};

struct MaquinaAlta {
    EstadoAlta    estado;
    unsigned long tInicioEstado;
    uint8_t       tentativas;
    uint16_t      totalLigadas;
    uint16_t      totalFalhas;
};

// ── Histórico ─────────────────────────────────────────────
struct RegistroHora {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
};

struct RegistroDia {
    uint16_t     ano;
    uint8_t      mes, dia;
    bool         valido;
    RegistroHora horas[HIST_HORAS];
    float totalPainelAh, totalCasaAh;
    float totalBatCargAh, totalBatDescAh;
};

struct ContadoresGlobais {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
};

struct AcumHora {
    float         painelAh, casaAh;
    float         batCargAh, batDescAh;
    uint8_t       horaAtual, diaAtual, mesAtual;
    uint16_t      anoAtual;
    unsigned long ultimoAcum;
};

// ── Tempo / NTP ───────────────────────────────────────────
struct ConfigTempo {
    int  fusoHoras;
    int  fusoMinutos;
    bool ntpAtivo;
    bool sincronizado;
    int  manualAnno, manualMes, manualDia;
    int  manualHora, manualMin;
};

// ── Previsão solar (AJUSTADA) ────────────────────────────
struct ConfigPrevisao {
    bool  ativa;
    float latitude, longitude;
    float painelKW;           // Potência painel em kW
    float fatorCalib;
    bool  configurada;
    // Capacidade da bateria AGORA vem da BMS
};

struct DadosPrevisao {
    bool          valido;
    float         radiacao[PREV_TOTAL_HORAS];
    float         ahPrevisto[PREV_TOTAL_HORAS];
    float         ahDia[PREV_DIAS];
    unsigned long tAtualizacao;
    int           horaBase, diaBase, mesBase, anoBase;
};

// ── Limiar sinal AC ───────────────────────────────────────
struct ConfigLimiaresSinal {
    int limiarAcMV;
};

// ── Limiar lâmpada ────────────────────────────────────────
struct ConfigLimiares {
    float pctLampExt;    // SOC% mínimo para ligar lâmpada
};

// ── Estimativa tempo ──────────────────────────────────────
struct EstimativaTempo {
    bool  valido;
    int   horas, minutos;
    bool  carregando;
    float taxaA;
};

// ═══════════════════════════════════════════════════════════
// [VAR] VARIÁVEIS GLOBAIS v23
// ═══════════════════════════════════════════════════════════
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP = false;
bool logado = false;

unsigned long ultimoBlink        = 0;
bool          ledState           = false;
bool          blinkAtivo         = true;
unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

char     logs[MAX_LOGS][LOG_MAX_LEN];
uint16_t totalLogs      = 0;
uint8_t  logIndex       = 0;
bool     logsCarregados = false;
char     tempBuf[256];



// Banco de baterias
ConfigBanco  banco   = {1, 1, false};
DadosBMS dadosBMS = {0.0f, 0.0f, 25.0f, 0.0f, 0, false, 0, 0.0f};
DadosBanco   dadosBanco = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f};

// BLE BMS
ConfigBMS     cfgBMS   = {"", false};
EstadoBLECtrl ctrlBLE  = {BLE_DESCONECTADO, 0, 0, 0, false, 0, {0}, 0};

// BLE objetos
BLEClient*                bleClient      = nullptr;
BLERemoteService*         bleService     = nullptr;
BLERemoteCharacteristic*  bleCharRX      = nullptr;
BLERemoteCharacteristic*  bleCharTX2     = nullptr;
BLERemoteCharacteristic*  bleCharTX      = nullptr;
BLERemoteCharacteristic*  bleCharRX2     = nullptr;
BLEScan*                  bleScan        = nullptr;
bool                      bleEncontrou   = false;
BLEAdvertisedDevice*      bleDispositivo = nullptr;

BleItem       bleLista[BLE_LISTA_MAX];
volatile uint8_t bleListaN        = 0;
volatile bool    bleScanSolicitado = false;
volatile bool    bleScanAtivo      = false;
volatile bool    bleModoLista      = false;
bool             bleIniciado       = false;
bool             bleTxResposta     = false;

volatile uint32_t bleRxTotal    = 0;
volatile bool     bleRxPend     = false;
uint8_t           bleRxDump[20];
volatile uint8_t  bleRxDumpLen  = 0;
uint8_t           bleRxLogRest  = 0;
bool              bleTxAmbos    = false;
unsigned long     bleTesteAte   = 0;

// Sensores
DadosSensores sensores   = {0};
CalibSensor   calibPainel;
BufferCorr    bufPainel   = {{0}, 0, 0};
BufferCorr    bufBateria  = {{0}, 0, 0};

// Relés
EstadoReles    estados = {false, false};
ConfigInversores cfgInv = {false, false};

// Sinal AC
SinalAC acAlta = {0.0f, false, false, false, 0};

// Máquina inversor alta
MaquinaAlta maqAlta = {INV_A_DESLIGADO, 0, 0, 0, 0};

// Limiar AC
ConfigLimiaresSinal limiarSinal = {AC_LIMIAR_MV_DEFAULT};

// Limiar lâmpada
ConfigLimiares limiares = {20.0f};

// Timers
unsigned long ultimaLeitura   = 0;
unsigned long ultimoControle  = 0;
unsigned long ultimoContagem  = 0;
unsigned long ultimoLampCheck = 0;

// Tempo
ConfigTempo   cfgTempo   = {-3, 0, false, false, 2024, 1, 1, 0, 0};
unsigned long tUltimoNTP = 0;
bool          ntpSincronizado = false;

// Previsão
ConfigPrevisao cfgPrevisao    = {false, 0, 0, 1.0f, 1.0f, false};
DadosPrevisao  previsao       = {false};
unsigned long  tUltimaPrevisao = 0;

// WiFi
String        gWifiSSID, gWifiPASS, gWifiIP, gWifiGW, gWifiMASK, gWifiDNS;
unsigned long tUltimoCheckWifi = 0;
unsigned long tUltimoReconWifi = 0;
uint8_t       wifiFalhasConsec = 0;
bool          wifiReconectando = false;
unsigned long tInicioRecon     = 0;
unsigned long tUltimaReconWifi2 = 0;

// Histórico
RegistroDia       histDiaAtual;
uint16_t          histIndiceAtual = 0, histTotalDias = 0;
ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora          acum       = {0, 0, 0, 0, 255, 0, 0, 0, 0};

float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx   = 0;
bool    histSnapFull  = false;
unsigned long ultimoHistSnap = 0;

uint8_t  diaCache = 0, mesCache = 0;
uint16_t anoCache = 0;
unsigned long ultimaAtualizacaoDia = 0;

// Modo teste
bool          modoTeste       = false;
unsigned long tInicioTeste    = 0;
unsigned long TIMEOUT_TESTE_MS = 30000UL;

// Proteção boot
unsigned long tInicioSistema      = 0;
bool          sistemaEstabilizado = false;

// ═══════════════════════════════════════════════════════════
// [PROTO] PROTÓTIPOS
// ═══════════════════════════════════════════════════════════
EstimativaTempo calcularTempoRestante();

void adicionarLog(const char* msg);
void logRAM();
void carregarLogsPersistentes();
void salvarLogNVS(uint8_t pos, const char* msg);

void setupLED(); void atualizarLED();
void ledBlink(); void ledFixo(bool v);
void setupBotao(); void verificarReset();

void setupReles();
void ligarInvAlta();    void desligarInvAlta();
void ligarLampExt();    void desligarLampExt();

int   lerADC(uint8_t pin);
float adicionarAmostraCorr(BufferCorr& buf, float nova);
float lerCorrentePainel();
void  calcularDadosBanco();
void  lerSensores();

// BLE BMS
void   salvarConfigBMS();    void carregarConfigBMS();
void   salvarConfigBanco();  void carregarConfigBanco();
bool   parsearDadosBMS(const uint8_t* data, size_t len);
void   bleNotifyCallback(BLERemoteCharacteristic* pChar,
                         uint8_t* pData, size_t length, bool isNotify);
void   conectarBMS();
void   desconectarBMS();
void   gerenciarBLE();
bool   bleSolicitarDados();
String statusBLEStr();

void processarSinalAC();
bool acAltaPresente();

void sincronizarNTP();
bool obterTempoAtual(struct tm& t);
String formatarDataHora();
String formatarHora();
void salvarConfigTempo(); void carregarConfigTempo();

void salvarConfigPrevisao(); void carregarConfigPrevisao();
bool buscarPrevisaoOpenMeteo();
void calcularAhPrevisto(); void calibrarPrevisao();
float getAhPrevHoraAtual(); float getAhPrevRestanteDia();
int  getIndiceHoraAtual(); void gerenciarPrevisao();

void calcularTotaisDia(RegistroDia& dia);
void salvarContadores(); void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void carregarHistorico();
void acumularEnergia(); void atualizarDiaAtual();

void maquinaInvAlta();
void controlarLampExt();
void controlarEnergia();

bool existeCredenciais(); bool existeWiFiConfig();
void salvarCredenciais(const String& u, const String& p);
bool verificarLogin(const String& u, const String& p);
void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap);
void carregarWifi(String& ssid, String& pass, String& ip,
                  String& gw,   String& mask,  String& dns, bool& ap);
void salvarConfigInversores(); void carregarConfigInversores();
void salvarLimiaresSinal();    void carregarLimiaresSinal();
void salvarLimiares();         void carregarLimiares();
void carregarCalibSensores();

void iniciarAP();
bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns);
void gerenciarWiFi();
void configurarRotas();

String gerarPagina(const String& titulo, const String& conteudo, bool comNav = true);
String barraProgresso(float pct, const String& label = "");
String nomeEstadoAlta(EstadoAlta e);

void handleRoot();      void handleDados();
void handleBateria();   void handleSensores();
void handleBMS();       void handleConfig();
void handleHistorico(); void handlePrevisao();
void handleLogs();      void handleLogsSalvos();
void handleSetupLogin();void handleSetupWifi();
void handleLogin();     void handleLogout();
void handleNotFound();  void handleReiniciar();
void handleOTA();       void handleOTAUpload();
void handleControle();

void executarScanLista();
void bleIniciar();
void handleBMSScan();
void handleBMSLista();

// ═══════════════════════════════════════════════════════════
// [UTIL] UTILITÁRIOS
// ═══════════════════════════════════════════════════════════
EstimativaTempo calcularTempoRestante() {
    EstimativaTempo est = {false, 0, 0, false, 0};

    if (!dadosBMS.valido) {
        Serial.println("[TEMPO] BMS inválido");
        return est;
    }
    if (dadosBMS.capacidadeAh < 1.0f) {
        Serial.println("[TEMPO] Capacidade inválida");
        return est;
    }

    float taxa = dadosBMS.corrente;  // A (positivo = carga, negativo = descarga)
    if (fabsf(taxa) < 0.3f) {
        Serial.println("[TEMPO] Taxa baixa demais");
        return est;
    }

    est.taxaA      = sensores.correnteBateria;  // corrente total do banco
    est.valido     = true;
    est.carregando = (taxa > 0);

    float soc = dadosBMS.soc;
    float cap = dadosBMS.capacidadeAh;

    // Ah ainda disponíveis
    float ah = est.carregando
    ? cap * (100.0f - soc) / 100.0f  // para completar 100%
    : cap * soc / 100.0f;             // até esvaziar

    if (ah <= 0.0f) {
        Serial.printf("[TEMPO] Ah inválido: %.2f\n", ah);
        return est;
    }

    float h = ah / fabsf(taxa);
    if (h > 999.0f) h = 999.0f;

    est.horas   = (int)h;
    est.minutos = (int)((h - est.horas) * 60);

    Serial.printf("[TEMPO] %s: %d:%02d (%.2fAh / %.2fA)\n",
                  est.carregando ? "CARGA" : "DESCARGA",
                  est.horas, est.minutos, ah, fabsf(taxa));

    return est;
}

// ═══════════════════════════════════════════════════════════
// [LOG] LOGS PERSISTENTES
// ═══════════════════════════════════════════════════════════
void carregarLogsPersistentes() {
    if (logsCarregados) return;
    prefs.begin(LOG_NVS_NS, true);
    uint16_t total = prefs.getUShort("total", 0);
    uint8_t  idx   = prefs.getUChar ("idx",   0);
    totalLogs = total;
    logIndex  = (idx < MAX_LOGS) ? idx : 0;
    uint16_t ini = (total > MAX_LOGS) ? (total - MAX_LOGS) : 0;
    for (uint16_t i = ini; i < total; i++) {
        uint8_t pos = i % MAX_LOGS;
        char chave[8];
        snprintf(chave, sizeof(chave), "l%03d", pos);
        String s = prefs.getString(chave, "");
        if (s.length() > 0) {
            strncpy(logs[pos], s.c_str(), LOG_MAX_LEN - 1);
            logs[pos][LOG_MAX_LEN - 1] = '\0';
        }
    }
    prefs.end();
    logsCarregados = true;
}

void salvarLogNVS(uint8_t pos, const char* msg) {
    char chave[8];
    snprintf(chave, sizeof(chave), "l%03d", pos);
    prefs.begin(LOG_NVS_NS, false);
    prefs.putString(chave, msg);
    prefs.putUShort("total", totalLogs);
    prefs.putUChar ("idx",   logIndex);
    prefs.end();
}

void adicionarLog(const char* msg) {
    if (!logsCarregados) carregarLogsPersistentes();

    static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
    char    entrada[LOG_MAX_LEN];
    uint8_t pos;
    bool    salvar;

    portENTER_CRITICAL(&logMux);
    totalLogs++;
    snprintf(entrada, sizeof(entrada), "[%05d] %s", totalLogs, msg);
    pos = logIndex;
    strncpy(logs[pos], entrada, LOG_MAX_LEN - 1);
    logs[pos][LOG_MAX_LEN - 1] = '\0';
    logIndex = (logIndex + 1) % MAX_LOGS;
    salvar   = (totalLogs % 5 == 0);
    portEXIT_CRITICAL(&logMux);

    if (salvar) salvarLogNVS(pos, logs[pos]);
}

void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf), "RAM %.0fK max%.0fK r%d",
             esp_get_free_heap_size() / 1024.0f,
             heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024.0f,
             (int)esp_reset_reason());
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [LED]
// ═══════════════════════════════════════════════════════════
void setupLED()  { pinMode(LED_AZUL, OUTPUT); digitalWrite(LED_AZUL, LOW); }
void atualizarLED() {
    if (!blinkAtivo) return;
    if (millis() - ultimoBlink >= 500) {
        ultimoBlink = millis();
        ledState    = !ledState;
        digitalWrite(LED_AZUL, ledState);
    }
}
void ledBlink()      { blinkAtivo = true; ultimoBlink = millis(); }
void ledFixo(bool v) { blinkAtivo = false; digitalWrite(LED_AZUL, v); }

// ═══════════════════════════════════════════════════════════
// [BTN]
// ═══════════════════════════════════════════════════════════
void setupBotao() { pinMode(BOTAO_BOOT, INPUT_PULLUP); }

void verificarReset() {
    bool p = (digitalRead(BOTAO_BOOT) == LOW);
    if (p && !botaoSegurando) { botaoSegurando = true; botaoPressionadoEm = millis(); }
    if (!p) botaoSegurando = false;
    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET TOTAL botao");
        for (int i = 0; i < 15; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(80);
            digitalWrite(LED_AZUL, LOW);  delay(80);
        }
        nvs_flash_erase();
        nvs_flash_init();
        ESP.restart();
    }
}

// ═══════════════════════════════════════════════════════════
// [RELE]
// ═══════════════════════════════════════════════════════════
void setupReles() {
    pinMode(PIN_RELE_INV_ALTA,  OUTPUT);
    pinMode(PIN_RELE_LAMP_EXT,  OUTPUT);
    digitalWrite(PIN_RELE_INV_ALTA,  cfgInv.invertReleInvAlta  ? HIGH : LOW);
    digitalWrite(PIN_RELE_LAMP_EXT,  cfgInv.invertReleLampExt  ? HIGH : LOW);
    estados = {false, false};
    adicionarLog("Reles OFF v23");
}

void ligarInvAlta()    { digitalWrite(PIN_RELE_INV_ALTA, cfgInv.invertReleInvAlta ? LOW  : HIGH); estados.invAlta = true;  adicionarLog("InvAlta ON"); }
void desligarInvAlta() { digitalWrite(PIN_RELE_INV_ALTA, cfgInv.invertReleInvAlta ? HIGH : LOW);  estados.invAlta = false; adicionarLog("InvAlta OFF"); }
void ligarLampExt()    { digitalWrite(PIN_RELE_LAMP_EXT, cfgInv.invertReleLampExt ? LOW  : HIGH); estados.lampExt = true; }
void desligarLampExt() { digitalWrite(PIN_RELE_LAMP_EXT, cfgInv.invertReleLampExt ? HIGH : LOW);  estados.lampExt = false; }

// ═══════════════════════════════════════════════════════════
// [ADC] Sensor painel
// ═══════════════════════════════════════════════════════════
int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < 30; i++) soma += analogRead(pin);
    return (int)(soma / 30);
}

float adicionarAmostraCorr(BufferCorr& buf, float nova) {
    buf.amostras[buf.indice] = nova;
    buf.indice = (buf.indice + 1) % BUFFER_CORRENTE;
    if (buf.total < BUFFER_CORRENTE) buf.total++;
    float soma = 0;
    for (uint8_t i = 0; i < buf.total; i++) soma += buf.amostras[i];
    return soma / buf.total;
}

float lerCorrentePainel() {
    int raw = lerADC(PIN_SENSOR_PAINEL);
    int mv  = (int)((long)raw * ADC_VREF / ADC_BITS);
    sensores.sinalPainel_mV = mv;
    float corrBruta = -(float)(mv - calibPainel.offsetMV) / calibPainel.sensMvA;
    if (corrBruta < 0.15f) corrBruta = 0.0f;
    float media = adicionarAmostraCorr(bufPainel, corrBruta);
    return (media < 0.15f) ? 0.0f : media;
}

// ═══════════════════════════════════════════════════════════
// [BANCO] Cálculos do banco v23 CORRIGIDO
// ═══════════════════════════════════════════════════════════
void calcularDadosBanco() {
    float nS = (float)banco.nS;
    float nP = (float)banco.nP;

    // Tensão do banco = tensão BMS × nS
    dadosBanco.tensaoTotal   = dadosBMS.tensao * nS;

    // Corrente do banco = corrente BMS × nP
    dadosBanco.correnteTotal = dadosBMS.corrente * nP;

    // Potência do banco
    dadosBanco.potencia      = dadosBanco.tensaoTotal * dadosBanco.correnteTotal;

    // SOC e ciclos direto da BMS
    dadosBanco.socPct  = dadosBMS.soc;
    dadosBanco.ciclos  = dadosBMS.ciclos;

    // NOVO: Capacidade do banco = capacidade BMS × nP
    dadosBanco.capacidadeTotal = dadosBMS.capacidadeAh * nP;
}

// ═══════════════════════════════════════════════════════════
// [SENS] Leitura sensores v23
// ═══════════════════════════════════════════════════════════
void lerSensores() {
    esp_task_wdt_reset();

    sensores.correntePainel = lerCorrentePainel();

    bool bmsOK = dadosBMS.valido &&
                 (millis() - dadosBMS.tUltimaDado < BMS_DADO_VALIDO_MS);

    if (bmsOK) {
        calcularDadosBanco();

        float corrFilt = adicionarAmostraCorr(bufBateria, dadosBanco.correnteTotal);
        if (fabsf(corrFilt) < BMS_FILTRO_RUIDO_A) corrFilt = 0.0f;

        sensores.correnteBateria = corrFilt;
        sensores.tensaoBanco     = dadosBanco.tensaoTotal;
        sensores.temperaturaBMS  = dadosBMS.temperatura;
        sensores.socBMS          = dadosBMS.soc;
        sensores.ciclosBMS       = dadosBMS.ciclos;
        sensores.capacidadeBanco = dadosBanco.capacidadeTotal;
    } else if (millis() - dadosBMS.tUltimaDado > 30000UL) {
        sensores.correnteBateria = 0.0f;
    }

    float casa = sensores.correntePainel - sensores.correnteBateria;
    if (casa < 0.0f) casa = 0.0f;
    sensores.correnteCasa = casa;

    float vBanco = (sensores.tensaoBanco > 1.0f) ? sensores.tensaoBanco : 12.0f;
    sensores.potenciaPainel  = sensores.correntePainel  * vBanco;
    sensores.potenciaBateria = sensores.correnteBateria * vBanco;
    sensores.potenciaCasa    = sensores.correnteCasa    * vBanco;

    esp_task_wdt_reset();

    if (millis() - ultimoHistSnap >= 1000) {
        ultimoHistSnap = millis();
        histCorrPainel[histSnapIdx] = sensores.correntePainel;
        histCorrCasa  [histSnapIdx] = sensores.correnteCasa;
        histCorrBat   [histSnapIdx] = sensores.correnteBateria;
        histSnapIdx = (histSnapIdx + 1) % HIST_SNAP_PONTOS;
        if (histSnapIdx == 0) histSnapFull = true;
    }
}

// ═══════════════════════════════════════════════════════════
// [BLE] BMS JBD/LanmaoBMS v23 CORRIGIDO
// ═══════════════════════════════════════════════════════════

void salvarConfigBMS() {
    prefs.begin("rrfs_bms", false);
    prefs.putString("mac", cfgBMS.macAddress);
    prefs.putBool  ("cfg", cfgBMS.configurada);
    prefs.putUChar ("ver", 25);
    prefs.end();
    adicionarLog("BMS config salva");
}

void carregarConfigBMS() {
    prefs.begin("rrfs_bms", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver == 25) {
        String mac = prefs.getString("mac", "");
        strncpy(cfgBMS.macAddress, mac.c_str(), sizeof(cfgBMS.macAddress)-1);
        cfgBMS.macAddress[sizeof(cfgBMS.macAddress)-1] = '\0';
        cfgBMS.configurada = prefs.getBool("cfg", false);
    } else {
        cfgBMS.configurada = false;
        cfgBMS.macAddress[0] = '\0';
    }
    prefs.end();
    if (cfgBMS.configurada)
        adicionarLog(("BMS MAC: " + String(cfgBMS.macAddress)).c_str());
}

void salvarConfigBanco() {
    prefs.begin("rrfs_banco", false);
    prefs.putUChar("nS",  banco.nS);
    prefs.putUChar("nP",  banco.nP);
    prefs.putBool ("cfg", banco.configurado);
    prefs.putUChar("ver", 25);
    prefs.end();
    snprintf(tempBuf, sizeof(tempBuf), "Banco: %dS%dP", banco.nS, banco.nP);
    adicionarLog(tempBuf);
}

void carregarConfigBanco() {
    prefs.begin("rrfs_banco", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver == 25) {
        banco.nS          = prefs.getUChar("nS",  1);
        banco.nP          = prefs.getUChar("nP",  1);
        banco.configurado = prefs.getBool ("cfg", false);
    } else {
        banco = {1, 1, false};
    }
    if (banco.nS < 1 || banco.nS > 8) banco.nS = 1;
    if (banco.nP < 1 || banco.nP > 8) banco.nP = 1;
    prefs.end();
}

// ── FUNÇÃO CRC16 (se não tiver) ──────────────────────────
static uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}


// ── CRC16 Modbus ──────────────────────────────────────────
static uint16_t crc16Modbus(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// ── Monta pedido: A5 A5 00 03 REG QTD CRC ─────────────────
static void montarPedido(uint8_t* out, uint16_t reg, uint16_t qtd) {
    out[0] = 0xA5; out[1] = 0xA5; out[2] = 0x00; out[3] = 0x03;
    out[4] = (uint8_t)(reg >> 8); out[5] = (uint8_t)(reg & 0xFF);
    out[6] = (uint8_t)(qtd >> 8); out[7] = (uint8_t)(qtd & 0xFF);
    uint16_t c = crc16Modbus(out + 2, 6);
    out[8] = (uint8_t)(c & 0xFF);
    out[9] = (uint8_t)(c >> 8);
}

// ── Parser da resposta ────────────────────────────────────
// Resposta A5 A5 00 03 NBYTES DADOS... CRC(2)
// Janela A (12 bytes de dados): regs 0..5 (corrente, tensao, SOC)
// Janela B (6 bytes de dados):  regs 10..12 (capacidade, ciclos)
bool parsearDadosBMS(const uint8_t* d, size_t len) {
    if (len < 9) return false;
    if (d[0] != 0xA5 || d[1] != 0xA5 || d[3] != 0x03) return false;

    size_t bc    = d[4];  // número de bytes de dados
    size_t total = 5 + bc + 2;  // cabeçalho + dados + CRC
    if (len < total) return false;

    // CRC: lo/hi (Modbus: CRC em little-endian)
    uint16_t c = crc16Modbus(d + 2, total - 4);
    bool crcOk = (d[total - 2] == (c & 0xFF)) && (d[total - 1] == (c >> 8));
    static bool avisouCrc = false;
    if (!crcOk && !avisouCrc) {
        avisouCrc = true;
        adicionarLog("BMS: CRC difere (aceito)");
    }

    const uint8_t* r = d + 5;  // ponteiro aos dados
    auto u16 = [&](int i) -> uint16_t {
        return (uint16_t)(((uint16_t)r[2 * i] << 8) | r[2 * i + 1]);
    };

    // ── JANELA A: regs 0..5 (12 bytes) ──────────────────
    if (bc == 12) {
        // reg 0-1: corrente (int32, 0.01 A)
        int32_t iRaw = (int32_t)(((uint32_t)u16(0) << 16) | u16(1));
        float corr = iRaw * 0.01f;

        // reg 2-3: tensao (int32, 0.01 V)
        int32_t vRaw = (int32_t)(((uint32_t)u16(2) << 16) | u16(3));
        float tens = vRaw * 0.01f;

        // reg 4: SOC (uint16, %)
        uint16_t soc = u16(4);

        // Validações
        if (tens < 0.0f || tens > 200.0f) {
            Serial.printf("[BMS] Tensão inválida: %.2f V\n", tens);
            return false;
        }
        if (corr < -500.0f || corr > 500.0f) {
            Serial.printf("[BMS] Corrente inválida: %.2f A\n", corr);
            return false;
        }
        if (soc > 100) {
            Serial.printf("[BMS] SOC inválido: %d%%\n", soc);
            return false;
        }

        // Salva dados
        dadosBMS.corrente    = corr;
        dadosBMS.tensao      = tens;
        dadosBMS.soc         = (float)soc;
        dadosBMS.valido      = true;
        dadosBMS.tUltimaDado = millis();

        Serial.printf("[BMS-JANELA A] I=%.2fA V=%.2fV SOC=%d%%\n", corr, tens, soc);
        return true;
    }

    // ── JANELA B: regs 10..12 (6 bytes) ──────────────────
    if (bc == 6) {
        // reg 10-11: capacidade (uint32, 0.01 Ah)
        uint32_t capRaw = ((uint32_t)u16(0) << 16) | u16(1);
        float cap = capRaw * 0.01f;

        // reg 12: ciclos (uint16)
        uint16_t ciclos = u16(2);

        // Validações
        if (cap > 100000.0f) {
            Serial.printf("[BMS] Capacidade inválida: %.0f Ah\n", cap);
            return false;
        }

        // Salva dados
        dadosBMS.capacidadeAh = cap;
        dadosBMS.ciclos       = ciclos;

        Serial.printf("[BMS-JANELA B] Cap=%.0fAh Ciclos=%d\n", cap, ciclos);
        return true;
    }

    Serial.printf("[BMS] Tamanho de dados inesperado: %u bytes\n", (unsigned)bc);
    return false;
}

void bleNotifyCallback(BLERemoteCharacteristic* pChar,
                       uint8_t* pData, size_t length, bool isNotify) {
    if (length == 0) return;

    bleRxTotal++;

    // Guarda os primeiros bytes brutos para o log
    if (bleRxLogRest > 0 && !bleRxPend) {
        uint8_t n = (length > sizeof(bleRxDump)) ? sizeof(bleRxDump) : (uint8_t)length;
        memcpy(bleRxDump, pData, n);
        bleRxDumpLen = n;
        bleRxPend    = true;

        // Log direto
        char logBuf[80];
        int p = snprintf(logBuf, sizeof(logBuf), "RX%u:", (unsigned)length);
        for (uint8_t i = 0; i < n && p < (int)sizeof(logBuf) - 3; i++)
            p += snprintf(logBuf + p, sizeof(logBuf) - p, "%02X", pData[i]);
        Serial.println(logBuf);
    }

    if (length > 64) {
        Serial.printf("[BLE] Pacote muito grande: %u bytes\n", (unsigned)length);
        return;
    }

    // Frame novo deve começar com A5 A5
    if (ctrlBLE.bufferLen == 0) {
        if (!(length >= 2 && pData[0] == 0xA5 && pData[1] == 0xA5)) {
            Serial.printf("[BLE] Descartado pacote sem A5A5: %02X %02X\n", pData[0], pData[1]);
            return;
        }
    }

    // Adiciona ao buffer
    if (ctrlBLE.bufferLen + length > sizeof(ctrlBLE.bufferRX)) {
        Serial.println("[BLE] Buffer overflow, reset");
        ctrlBLE.bufferLen = 0;
        return;
    }
    memcpy(ctrlBLE.bufferRX + ctrlBLE.bufferLen, pData, length);
    ctrlBLE.bufferLen += length;

    // Procura frame completo
    if (ctrlBLE.bufferLen < 5) {
        Serial.printf("[BLE] Aguardando mais bytes... (%u/5)\n", (unsigned)ctrlBLE.bufferLen);
        return;
    }

    // Calcula tamanho esperado
    size_t esperado = 5 + (size_t)ctrlBLE.bufferRX[4] + 2;
    if (esperado > sizeof(ctrlBLE.bufferRX)) {
        Serial.printf("[BLE] Frame inválido (esperado %u bytes)\n", (unsigned)esperado);
        ctrlBLE.bufferLen = 0;
        return;
    }

    if (ctrlBLE.bufferLen < esperado) {
        Serial.printf("[BLE] Aguardando mais bytes... (%u/%u)\n",
                     (unsigned)ctrlBLE.bufferLen, (unsigned)esperado);
        return;
    }

    // Frame completo - tenta parsear
    Serial.printf("[BLE] Frame completo recebido (%u bytes)\n", (unsigned)ctrlBLE.bufferLen);
    bool ok = parsearDadosBMS(ctrlBLE.bufferRX, ctrlBLE.bufferLen);
    ctrlBLE.bufferLen = 0;

    if (ok) {
        ctrlBLE.falhasConsec = 0;
        static unsigned long tLogBMS = 0;
        if (millis() - tLogBMS > 60000UL) {
            tLogBMS = millis();
            char b[80];
            snprintf(b, sizeof(b), "BMS OK: %.2fA | %.2fV | SOC%.0f%% | %dc | %.0fAh",
                     dadosBMS.corrente, dadosBMS.tensao, dadosBMS.soc,
                     (int)dadosBMS.ciclos, dadosBMS.capacidadeAh);
            adicionarLog(b);
        }
    } else {
        Serial.println("[BLE] ✗ Parse falhou");
    }
}

bool bleSolicitarDados() {
    if (ctrlBLE.estado != BLE_CONECTADO) return false;
    if (!bleCharTX || !bleClient || !bleClient->isConnected()) return false;

    static uint8_t janela = 0;
    uint8_t pedido[10];

    if (janela == 0) {
        montarPedido(pedido, 0, 6);   // regs 0..5 (I, V, SOC)
        Serial.println("[BLE-TX] Solicitando JANELA A (regs 0..5)");
    } else {
        montarPedido(pedido, 10, 3);  // regs 10..12 (Cap, Ciclos)
        Serial.println("[BLE-TX] Solicitando JANELA B (regs 10..12)");
    }
    janela ^= 1;

    try {
        bleCharTX->writeValue(pedido, sizeof(pedido), false);  // FFFB: sem resposta
        char logBuf[40];
        int p = snprintf(logBuf, sizeof(logBuf), "TX: ");
        for (int i = 0; i < 10; i++)
            p += snprintf(logBuf + p, sizeof(logBuf) - p, "%02X", pedido[i]);
        Serial.println(logBuf);
        return true;
    } catch (...) {
        Serial.println("[BLE-TX] Erro ao enviar comando");
        adicionarLog("BMS: erro write");
        return false;
    }
}

String statusBLEStr() {
    switch (ctrlBLE.estado) {
        case BLE_DESCONECTADO: return F("Desconectado");
        case BLE_ESCANEANDO:   return F("Escaneando");
        case BLE_CONECTANDO:   return F("Conectando");
        case BLE_CONECTADO:    return F("Conectado");
        case BLE_FALHA:        return F("Falha");
        default:               return F("?");
    }
}

class BLECallbackScan : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) override {
        String mac = String(device.getAddress().toString().c_str());
        mac.toUpperCase();

        if (bleModoLista) {
            for (uint8_t i = 0; i < bleListaN; i++) {
                if (strcmp(bleLista[i].mac, mac.c_str()) == 0) {
                    bleLista[i].rssi = device.getRSSI();
                    return;
                }
            }
            if (bleListaN >= BLE_LISTA_MAX) return;

            String nm = device.haveName() ? String(device.getName().c_str()) : String("");
            String nl = nm; nl.toLowerCase();
            bool jbd = device.isAdvertisingService(BLEUUID(BMS_SERVICE_UUID)) ||
                       nl.indexOf("bms") >= 0 || nl.indexOf("jbd") >= 0 ||
                       nl.indexOf("xiaoxiang") >= 0 || nl.indexOf("lanmao") >= 0;
            if (nm.length() == 0 && !jbd) return;

            BleItem& it = bleLista[bleListaN];
            strncpy(it.mac, mac.c_str(), sizeof(it.mac) - 1);
            it.mac[sizeof(it.mac) - 1] = '\0';
            if (nm.length() == 0) nm = "(sem nome)";
            strncpy(it.nome, nm.c_str(), sizeof(it.nome) - 1);
            it.nome[sizeof(it.nome) - 1] = '\0';
            it.rssi = device.getRSSI();
            it.jbd  = jbd;
            bleListaN++;
            return;
        }

        if (!cfgBMS.configurada) return;
        if (mac == String(cfgBMS.macAddress)) {
            bleEncontrou   = true;
            bleDispositivo = new BLEAdvertisedDevice(device);
            bleScan->stop();
        }
    }
};

class BLEClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* c) override {}
    void onDisconnect(BLEClient* c) override {
        ctrlBLE.estado           = BLE_DESCONECTADO;
        ctrlBLE.notificacaoAtiva = false;
        ctrlBLE.bufferLen        = 0;
        adicionarLog("BMS: BLE desconectado");
    }
};

void bleIniciar() {
    if (bleIniciado) return;
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    BLEDevice::init("RRFS");
    bleScan = BLEDevice::getScan();
    static BLECallbackScan cbScan;
    bleScan->setAdvertisedDeviceCallbacks(&cbScan, false);
    bleScan->setActiveScan(true);
    bleScan->setInterval(100);
    bleScan->setWindow(99);
    bleIniciado = true;
    logRAM();
}

static void wdtTimeout(uint32_t ms) {
    esp_task_wdt_config_t c = {
        .timeout_ms     = ms,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_reconfigure(&c);
}

void executarScanLista() {
    bleIniciar();
    bleListaN    = 0;
    bleModoLista = true;
    bleScanAtivo = true;
    adicionarLog("BLE: buscando dispositivos");

    for (int i = 0; i < BLE_SCAN_LISTA_S; i++) {
        esp_task_wdt_reset();
        bleScan->start(1, false);
        bleScan->clearResults();
        server.handleClient();
    }
    bleModoLista = false;
    bleScanAtivo = false;
    snprintf(tempBuf, sizeof(tempBuf), "BLE: %d dispositivos", (int)bleListaN);
    adicionarLog(tempBuf);
}

static bool bleDescobrirCaracteristicas() {
    bleService = nullptr;
    bleCharRX = nullptr;
    bleCharRX2 = nullptr;
    bleCharTX = nullptr;
    bleCharTX2 = nullptr;

    Serial.println("[BLE] Buscando serviço FFFA...");

    // Tentativa 1: UUID exato
    BLERemoteService* s = bleClient->getService(
        BLEUUID("0000fffa-0000-1000-8000-00805f9b34fb"));

    // Tentativa 2: procura genérica
    if (!s) {
        Serial.println("[BLE] FFFA exato não encontrado, buscando serviços...");
        auto* servicos = bleClient->getServices();
        if (servicos) {
            for (auto& kv : *servicos) {
                String u = String(kv.second->getUUID().toString().c_str());
                u.toLowerCase();
                if (u.indexOf("fffa") >= 0) {
                    s = kv.second;
                    Serial.printf("[BLE] Encontrado serviço: %s\n", u.c_str());
                    break;
                }
            }
        }
    }

    if (!s) {
        adicionarLog("BMS: serviço FFFA não achado");
        Serial.println("[BLE] ✗ Serviço FFFA não encontrado");
        return false;
    }

    // Busca características
    BLERemoteCharacteristic* tx = s->getCharacteristic(
        BLEUUID("0000fffb-0000-1000-8000-00805f9b34fb"));
    BLERemoteCharacteristic* rx = s->getCharacteristic(
        BLEUUID("0000fffc-0000-1000-8000-00805f9b34fb"));

    if (!tx) {
        adicionarLog("BMS: FFFB (TX) não achada");
        Serial.println("[BLE] ✗ Característica FFFB não encontrada");
        return false;
    }
    if (!rx) {
        adicionarLog("BMS: FFFC (RX) não achada");
        Serial.println("[BLE] ✗ Característica FFFC não encontrada");
        return false;
    }
    if (!rx->canNotify()) {
        adicionarLog("BMS: FFFC sem notify");
        Serial.println("[BLE] ✗ FFFC não suporta notificações");
        return false;
    }

    bleService = s;
    bleCharTX = tx;
    bleCharRX = rx;

    Serial.println("[BLE] ✓ Serviço FFFA + características OK");
    return true;
}

static void bleRegistrar(BLERemoteCharacteristic* ch) {
    if (!ch) {
        Serial.println("[BLE] Característica nula para registrar");
        return;
    }

    Serial.printf("[BLE] Registrando notificações em %s\n",
                  String(ch->getUUID().toString().c_str()).c_str());

    bool ind = !ch->canNotify() && ch->canIndicate();
    ch->registerForNotify(bleNotifyCallback, !ind);

    BLERemoteDescriptor* d = ch->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (!d) {
        adicionarLog("BMS: CCCD 0x2902 não achado");
        Serial.println("[BLE] ✗ Descritor CCCD não encontrado");
        return;
    }

    uint8_t v[2] = {(uint8_t)(ind ? 0x02 : 0x01), 0x00};
    d->writeValue(v, 2, true);

    adicionarLog("BMS: CCCD ativado");
    Serial.println("[BLE] ✓ CCCD ativado");
}

void conectarBMS() {
    if (!cfgBMS.configurada || cfgBMS.macAddress[0] == '\0') {
        Serial.println("[BLE] BMS não configurada");
        return;
    }
    if (ctrlBLE.estado == BLE_CONECTADO ||
        ctrlBLE.estado == BLE_CONECTANDO) {
        Serial.println("[BLE] Já conectando ou conectado");
    return;
        }

        bleIniciar();
        adicionarLog("BMS: iniciando scan BLE");
        Serial.printf("[BLE] Procurando por: %s\n", cfgBMS.macAddress);

        ctrlBLE.estado    = BLE_ESCANEANDO;
        ctrlBLE.bufferLen = 0;
        bleEncontrou      = false;
        bleModoLista      = false;

        if (bleDispositivo) {
            delete bleDispositivo;
            bleDispositivo = nullptr;
        }

        for (int i = 0; i < BLE_SCAN_DURACAO_S && !bleEncontrou; i++) {
            esp_task_wdt_reset();
            Serial.printf("[BLE] Escaneando... (%d/%d)\n", i+1, BLE_SCAN_DURACAO_S);
            bleScan->start(1, false);
            bleScan->clearResults();
            server.handleClient();
        }

        if (!bleEncontrou || !bleDispositivo) {
            ctrlBLE.estado = BLE_FALHA;
            ctrlBLE.falhasConsec++;
            adicionarLog("BMS: não encontrada no scan");
            Serial.println("[BLE] ✗ BMS não encontrada");
            return;
        }

        adicionarLog("BMS: encontrada, conectando");
        Serial.println("[BLE] ✓ BMS encontrada! Conectando...");

        ctrlBLE.estado      = BLE_CONECTANDO;
        ctrlBLE.tConectando = millis();

        if (!bleClient) {
            bleClient = BLEDevice::createClient();
            static BLEClientCallback cbCliente;
            bleClient->setClientCallbacks(&cbCliente);
        } else if (bleClient->isConnected()) {
            bleClient->disconnect();
        }

        wdtTimeout(60000);
        esp_task_wdt_reset();
        bool conectou = bleClient->connect(bleDispositivo);
        esp_task_wdt_reset();
        wdtTimeout(30000);

        if (!conectou) {
            ctrlBLE.estado = BLE_FALHA;
            ctrlBLE.falhasConsec++;
            adicionarLog("BMS: falha conexão BLE");
            Serial.println("[BLE] ✗ Falha ao conectar");
            return;
        }

        Serial.println("[BLE] ✓ Conectado! Descobrindo características...");

        wdtTimeout(60000);
        bool achou = bleDescobrirCaracteristicas();
        wdtTimeout(30000);
        esp_task_wdt_reset();

        if (!achou || !bleCharRX || !bleCharTX) {
            bleClient->disconnect();
            ctrlBLE.estado = BLE_FALHA;
            ctrlBLE.falhasConsec++;
            adicionarLog("BMS: serviço/características não achados");
            Serial.println("[BLE] ✗ Características não encontradas");
            return;
        }

        bleRxTotal   = 0;
        bleRxPend    = false;
        bleRxLogRest = 3;  // mostra os 3 primeiros RX brutos

        bleRegistrar(bleCharRX);
        ctrlBLE.notificacaoAtiva = true;

        bleTxResposta = false;
        bleTxAmbos    = false;
        bleCharTX2    = nullptr;
        bleCharRX2    = nullptr;

        Serial.printf("[BLE] TX: W=%d WNR=%d\n",
                      (int)bleCharTX->canWrite(),
                      (int)bleCharTX->canWriteNoResponse());

        for (int i = 0; i < 3; i++) {
            delay(100);
            esp_task_wdt_reset();
        }

        ctrlBLE.estado         = BLE_CONECTADO;
        ctrlBLE.falhasConsec   = 0;
        ctrlBLE.tUltimaLeitura = 0;
        ctrlBLE.tConectando    = millis();

        adicionarLog("BMS: BLE conectado OK");
        Serial.println("[BLE] ✓ Sistema BLE pronto!");
        logRAM();
}

void desconectarBMS() {
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    ctrlBLE.estado           = BLE_DESCONECTADO;
    ctrlBLE.notificacaoAtiva = false;
    ctrlBLE.bufferLen        = 0;
}

void gerenciarBLE() {
    // Imprime nos logs os bytes brutos recebidos
    if (bleRxPend) {
        char b[56];
        int p = snprintf(b, sizeof(b), "RX%u:", (unsigned)bleRxDumpLen);
        for (uint8_t i = 0; i < bleRxDumpLen && p < (int)sizeof(b) - 3; i++)
            p += snprintf(b + p, sizeof(b) - p, "%02X", bleRxDump[i]);
        adicionarLog(b);
        if (bleRxLogRest > 0) bleRxLogRest--;
        bleRxPend = false;
    }

    if (bleScanSolicitado) {
        bleScanSolicitado = false;
        executarScanLista();
        return;
    }

    if (!cfgBMS.configurada) return;

    unsigned long agora = millis();
    bool emTeste = (bleTesteAte != 0 && (long)(bleTesteAte - agora) > 0);

    if (ctrlBLE.estado == BLE_CONECTADO) {
        if (emTeste) return;

        // Verifica se recebeu dados
        if (agora - ctrlBLE.tConectando > 20000UL &&
            (dadosBMS.tUltimaDado == 0 ||
             agora - dadosBMS.tUltimaDado > 20000UL)) {
            char b[60];
            snprintf(b, sizeof(b), "BMS: sem dados (rx=%u)", (unsigned)bleRxTotal);
            adicionarLog(b);
            Serial.printf("[BLE] ✗ %s\n", b);
            desconectarBMS();
            ctrlBLE.falhasConsec++;
            return;
        }

        // Solicita dados periodicamente
        if (agora - ctrlBLE.tUltimaLeitura >= BLE_LEITURA_MS) {
            ctrlBLE.tUltimaLeitura = agora;
            esp_task_wdt_reset();
            server.handleClient();
            if (!bleSolicitarDados()) {
                ctrlBLE.estado = BLE_DESCONECTADO;
                ctrlBLE.falhasConsec++;
                adicionarLog("BMS: erro solicitação");
                Serial.println("[BLE] ✗ Erro ao solicitar dados");
            }
            esp_task_wdt_reset();
        }
        return;
    }

    if (ctrlBLE.estado == BLE_FALHA ||
        ctrlBLE.estado == BLE_DESCONECTADO) {
        unsigned long intervalo = BLE_RECONECT_MS;
        if (ctrlBLE.falhasConsec >  5) intervalo =  60000UL;
        if (ctrlBLE.falhasConsec > 10) intervalo = 300000UL;

        if (agora - ctrlBLE.tUltimoTentativa >= intervalo) {
            ctrlBLE.tUltimoTentativa = agora;
            server.handleClient();
            esp_task_wdt_reset();
            conectarBMS();
            esp_task_wdt_reset();
            server.handleClient();
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [AC] SINAL AC
// ═══════════════════════════════════════════════════════════
void processarSinalAC() {
    static float   bufAC[AC_BUFFER_SUAV] = {0};
    static uint8_t bufIdx = 0, bufTot = 0;

    unsigned long agora = millis();

    server.handleClient();
    if (modoAP || WiFi.getMode() == WIFI_AP_STA)
        dnsServer.processNextRequest();
    esp_task_wdt_reset();

    long   somaADC = 0;
    int    contADC = 0;
    unsigned long tJanela = millis();
    while (millis() - tJanela < AC_JANELA_MS) {
        somaADC += analogRead(PIN_AC_SINAL_ALTA);
        contADC++;
        delayMicroseconds(150);
        if (contADC % 20 == 0) esp_task_wdt_reset();
    }
    if (contADC == 0) return;

    float mediaMV = ((float)somaADC / contADC * ADC_VREF) / ADC_BITS;

    server.handleClient();
    esp_task_wdt_reset();

    bufAC[bufIdx] = mediaMV;
    bufIdx = (bufIdx + 1) % AC_BUFFER_SUAV;
    if (bufTot < AC_BUFFER_SUAV) bufTot++;

    float soma = 0;
    for (uint8_t i = 0; i < bufTot; i++) soma += bufAC[i];
    acAlta.mediaMV = soma / bufTot;

    int limiarLiga    = limiarSinal.limiarAcMV + AC_HISTERESE_LIGA_MV;
    int limiarDesliga = limiarSinal.limiarAcMV - AC_HISTERESE_DESLIGA_MV;
    if (limiarDesliga < 10) limiarDesliga = 10;

    bool acimaDeLiga   = (acAlta.mediaMV >= limiarLiga);
    bool abaixoDesliga = (acAlta.mediaMV <  limiarDesliga);

    if (!acAlta.presente) {
        if (acimaDeLiga) {
            if (!acAlta.aguardandoLigar) {
                acAlta.aguardandoLigar    = true;
                acAlta.tInicioConfirma    = agora;
                acAlta.aguardandoDesligar = false;
            } else if (agora - acAlta.tInicioConfirma >= AC_CONFIRMA_MS) {
                acAlta.presente        = true;
                acAlta.aguardandoLigar = false;
                memset(bufAC, 0, sizeof(bufAC));
                bufIdx = 0; bufTot = 0;
                adicionarLog("AC Alta: PRESENTE");
            }
        } else {
            acAlta.aguardandoLigar = false;
        }
    } else {
        if (abaixoDesliga) {
            if (!acAlta.aguardandoDesligar) {
                acAlta.aguardandoDesligar = true;
                acAlta.tInicioConfirma    = agora;
                acAlta.aguardandoLigar    = false;
            } else if (agora - acAlta.tInicioConfirma >= AC_CONFIRMA_MS) {
                acAlta.presente           = false;
                acAlta.aguardandoDesligar = false;
                memset(bufAC, 0, sizeof(bufAC));
                bufIdx = 0; bufTot = 0;
                adicionarLog("AC Alta: QUEDA");
            }
        } else {
            acAlta.aguardandoDesligar = false;
        }
    }
}

bool acAltaPresente() { return acAlta.presente; }

// ═══════════════════════════════════════════════════════════
// [NTP]
// ═══════════════════════════════════════════════════════════
void salvarConfigTempo() {
    prefs.begin("rrfs_time", false);
    prefs.putInt ("fuso_h",    cfgTempo.fusoHoras);
    prefs.putInt ("fuso_m",    cfgTempo.fusoMinutos);
    prefs.putBool("ntp_ativo", cfgTempo.ntpAtivo);
    prefs.putInt ("man_ano",   cfgTempo.manualAnno);
    prefs.putInt ("man_mes",   cfgTempo.manualMes);
    prefs.putInt ("man_dia",   cfgTempo.manualDia);
    prefs.putInt ("man_hora",  cfgTempo.manualHora);
    prefs.putInt ("man_min",   cfgTempo.manualMin);
    prefs.putUChar("ver", 25);
    prefs.end();
}

void carregarConfigTempo() {
    prefs.begin("rrfs_time", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver != 25) {
        prefs.end();
        cfgTempo = {-3, 0, false, false, 2024, 1, 1, 0, 0};
        return;
    }
    cfgTempo.fusoHoras    = prefs.getInt ("fuso_h",   -3);
    cfgTempo.fusoMinutos  = prefs.getInt ("fuso_m",    0);
    cfgTempo.ntpAtivo     = prefs.getBool("ntp_ativo", false);
    cfgTempo.manualAnno   = prefs.getInt ("man_ano",   2024);
    cfgTempo.manualMes    = prefs.getInt ("man_mes",   1);
    cfgTempo.manualDia    = prefs.getInt ("man_dia",   1);
    cfgTempo.manualHora   = prefs.getInt ("man_hora",  0);
    cfgTempo.manualMin    = prefs.getInt ("man_min",   0);
    cfgTempo.sincronizado = false;
    prefs.end();
}

void sincronizarNTP() {
    if (!cfgTempo.ntpAtivo) return;
    if (modoAP || WiFi.status() != WL_CONNECTED) {
        adicionarLog("NTP: sem WiFi"); return;
    }
    if (WiFi.RSSI() < -85) { adicionarLog("NTP: sinal fraco"); return; }

    adicionarLog("NTP: iniciando");
    long off = (long)cfgTempo.fusoHoras * 3600L + (long)cfgTempo.fusoMinutos * 60L;
    configTime(off, 0, NTP_SERVER1, NTP_SERVER2);

    unsigned long tInicio = millis();
    struct tm ti;
    bool ok = false;
    while (millis() - tInicio < 1500UL) {
        esp_task_wdt_reset();
        server.handleClient();
        if (modoAP || WiFi.getMode() == WIFI_AP_STA)
            dnsServer.processNextRequest();
        if (getLocalTime(&ti, 50)) { ok = true; break; }
        delay(50);
    }
    if (ok) {
        ntpSincronizado = true; cfgTempo.sincronizado = true;
        tUltimoNTP = millis(); adicionarLog("NTP: OK");
    } else {
        ntpSincronizado = false; adicionarLog("NTP: timeout");
    }
}

bool obterTempoAtual(struct tm& t) {
    if (ntpSincronizado && cfgTempo.ntpAtivo) return getLocalTime(&t, 100);
    memset(&t, 0, sizeof(struct tm));
    t.tm_year = cfgTempo.manualAnno - 1900;
    t.tm_mon  = cfgTempo.manualMes  - 1;
    t.tm_mday = cfgTempo.manualDia;
    t.tm_hour = cfgTempo.manualHora;
    t.tm_min  = cfgTempo.manualMin;
    return true;
}

String formatarDataHora() {
    struct tm t;
    if (!obterTempoAtual(t)) return F("--/--/---- --:--");
    char buf[24];
    snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d",
             t.tm_mday, t.tm_mon+1, t.tm_year+1900, t.tm_hour, t.tm_min);
    return String(buf);
}

String formatarHora() {
    struct tm t;
    if (!obterTempoAtual(t)) return F("--:--");
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    return String(buf);
}

// ═══════════════════════════════════════════════════════════
// [PREV] Previsão Solar v23 CORRIGIDO
// ═══════════════════════════════════════════════════════════
void salvarConfigPrevisao() {
    prefs.begin("rrfs_prev", false);
    prefs.putBool ("ativa",       cfgPrevisao.ativa);
    prefs.putFloat("lat",         cfgPrevisao.latitude);
    prefs.putFloat("lon",         cfgPrevisao.longitude);
    prefs.putFloat("painel_kw",   cfgPrevisao.painelKW);
    prefs.putFloat("fator_calib", cfgPrevisao.fatorCalib);
    prefs.putBool ("configurada", cfgPrevisao.configurada);
    prefs.end();
}

void carregarConfigPrevisao() {
    prefs.begin("rrfs_prev", true);
    cfgPrevisao.configurada  = prefs.getBool ("configurada", false);
    cfgPrevisao.ativa        = prefs.getBool ("ativa",       false);
    cfgPrevisao.latitude     = prefs.getFloat("lat",         0.0f);
    cfgPrevisao.longitude    = prefs.getFloat("lon",         0.0f);
    cfgPrevisao.painelKW     = prefs.getFloat("painel_kw",   0.0f);
    cfgPrevisao.fatorCalib   = prefs.getFloat("fator_calib", 1.0f);
    prefs.end();
}

int getIndiceHoraAtual() {
    if (!previsao.valido) return -1;
    struct tm t; if (!obterTempoAtual(t)) return -1;
    auto toDias = [](int a, int m, int d) -> long {
        if (m < 3) { m += 12; a--; }
        return (long)365*a + a/4 - a/100 + a/400 + (153*m-457)/5 + d;
    };
    long diasBase  = toDias(previsao.anoBase, previsao.mesBase, previsao.diaBase);
    long diasAtual = toDias(t.tm_year+1900, t.tm_mon+1, t.tm_mday);
    long diff = (diasAtual - diasBase)*24L + (t.tm_hour - previsao.horaBase);
    if (diff < 0 || diff >= PREV_TOTAL_HORAS) return -1;
    return (int)diff;
}

float getAhPrevHoraAtual() {
    int idx = getIndiceHoraAtual();
    if (idx < 0 || idx >= PREV_TOTAL_HORAS) return 0.0f;
    return previsao.ahPrevisto[idx];
}

float getAhPrevRestanteDia() {
    int idx = getIndiceHoraAtual(); if (idx < 0) return 0.0f;
    struct tm t; if (!obterTempoAtual(t)) return 0.0f;
    int hFalt = 24 - t.tm_hour; float total = 0.0f;
    for (int i = 0; i < hFalt && (idx+i) < PREV_TOTAL_HORAS; i++)
        total += previsao.ahPrevisto[idx+i];
    return total;
}

void calcularAhPrevisto() {
    // Usa CAPACIDADE REAL da BMS, não configuração do usuário
    float capacidadeBanco = dadosBMS.capacidadeAh * (float)banco.nP;
    if (capacidadeBanco <= 0.0f) capacidadeBanco = 100.0f; // fallback

    float vBanco = (dadosBMS.valido && dadosBMS.tensao > 1.0f)
        ? dadosBMS.tensao * banco.nS
        : 12.0f * banco.nS;
    if (vBanco < 1.0f) vBanco = 12.0f;

    for (int d = 0; d < PREV_DIAS; d++) previsao.ahDia[d] = 0.0f;
    for (int h = 0; h < PREV_TOTAL_HORAS; h++) {
        float rad = previsao.radiacao[h]; if (rad < 0) rad = 0;
        float corrA = (rad / 1000.0f) * (cfgPrevisao.painelKW * 1000.0f) / vBanco;
        // Limita pela capacidade real do banco, não por configuração
        float corrMax = capacidadeBanco / 10.0f; // estimativa conservadora
        if (corrA > corrMax) corrA = corrMax;
        corrA *= cfgPrevisao.fatorCalib;
        previsao.ahPrevisto[h] = corrA;
        int dIdx = h / 24; if (dIdx < PREV_DIAS) previsao.ahDia[dIdx] += corrA;
    }
}

void calibrarPrevisao() {
    if (!previsao.valido || !cfgPrevisao.configurada) return;
    int idx = getIndiceHoraAtual(); if (idx < 0) return;
    float ahReal = acum.painelAh, ahPrev = previsao.ahPrevisto[idx];
    if (ahPrev < 0.1f || ahReal < 0.1f) return;
    float nf = ahReal / ahPrev;
    cfgPrevisao.fatorCalib = cfgPrevisao.fatorCalib * 0.90f + nf * 0.10f;
    if (cfgPrevisao.fatorCalib < 0.3f) cfgPrevisao.fatorCalib = 0.3f;
    if (cfgPrevisao.fatorCalib > 2.0f) cfgPrevisao.fatorCalib = 2.0f;
    calcularAhPrevisto();
    prefs.begin("rrfs_prev", false);
    prefs.putFloat("fator_calib", cfgPrevisao.fatorCalib);
    prefs.end();
}

bool buscarPrevisaoOpenMeteo() {
    if (WiFi.status() != WL_CONNECTED || !cfgPrevisao.configurada) return false;
    if (WiFi.RSSI() < -85) { adicionarLog("Prev: sinal fraco"); return false; }

    char url[256];
    snprintf(url, sizeof(url),
        "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
        "&hourly=shortwave_radiation&forecast_days=7&timezone=auto",
        cfgPrevisao.latitude, cfgPrevisao.longitude);

    HTTPClient http;
    http.begin(url);
    http.setTimeout(5000);
    http.setConnectTimeout(3000);

    esp_task_wdt_reset(); server.handleClient();
    int code = http.GET();
    esp_task_wdt_reset(); server.handleClient();

    if (code != 200) {
        http.end();
        snprintf(tempBuf, sizeof(tempBuf), "Prev: HTTP %d", code);
        adicionarLog(tempBuf); return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    String body = ""; body.reserve(8192);
    unsigned long tL = millis();
    while (http.connected() && millis() - tL < 5000UL) {
        if (stream->available()) {
            body += (char)stream->read();
            if (body.length() > 16000) break;
        }
        if (body.length() % 200 == 0) {
            esp_task_wdt_reset(); server.handleClient();
        }
    }
    http.end();

    int posArr = body.indexOf("shortwave_radiation\":[");
    if (posArr < 0) return false;
    posArr += 22;
    int count = 0, pos = posArr;
    while (count < PREV_TOTAL_HORAS && pos < (int)body.length()) {
        while (pos < (int)body.length() &&
               body[pos] != '-' && body[pos] != '.' &&
               (body[pos]<'0' || body[pos]>'9')) {
            if (body[pos] == ']') goto fimParser;
            pos++;
        }
        if (pos >= (int)body.length()) break;
        float val = 0.0f; bool neg = false;
        if (body[pos] == '-') { neg = true; pos++; }
        while (pos < (int)body.length() && body[pos]>='0' && body[pos]<='9')
            val = val*10.0f + (body[pos++]-'0');
        if (pos < (int)body.length() && body[pos]=='.') {
            pos++; float dec=0.1f;
            while (pos < (int)body.length() && body[pos]>='0' && body[pos]<='9')
                val += (body[pos++]-'0')*dec, dec*=0.1f;
        }
        if (neg) val = -val;
        previsao.radiacao[count++] = val;
    }
    fimParser:
    if (count < 24) return false;
    for (int i = count; i < PREV_TOTAL_HORAS; i++) previsao.radiacao[i] = 0.0f;

    struct tm t;
    if (obterTempoAtual(t)) {
        previsao.horaBase = t.tm_hour; previsao.diaBase  = t.tm_mday;
        previsao.mesBase  = t.tm_mon+1; previsao.anoBase = t.tm_year+1900;
    }
    previsao.tAtualizacao = millis();
    previsao.valido = true;
    calcularAhPrevisto();
    snprintf(tempBuf, sizeof(tempBuf), "Prev OK %d h", count);
    adicionarLog(tempBuf);
    return true;
}

void gerenciarPrevisao() {
    if (!cfgPrevisao.ativa || !cfgPrevisao.configurada) return;
    if (WiFi.status() != WL_CONNECTED || modoAP) return;
    if (millis() - tUltimaReconWifi2 < 30000UL) return;

    unsigned long agora = millis();
    if (!previsao.valido || agora - tUltimaPrevisao >= INTERVALO_PREVISAO_MS) {
        tUltimaPrevisao = agora;
        esp_task_wdt_reset(); server.handleClient();
        buscarPrevisaoOpenMeteo();
        esp_task_wdt_reset(); server.handleClient();
    }
    static unsigned long tUltimaCalib = 0;
    if (agora - tUltimaCalib >= 3600000UL) {
        tUltimaCalib = agora; calibrarPrevisao();
    }
}

// ═══════════════════════════════════════════════════════════
// [HIST] Histórico
// ═══════════════════════════════════════════════════════════
void calcularTotaisDia(RegistroDia& dia) {
    dia.totalPainelAh = dia.totalCasaAh =
    dia.totalBatCargAh = dia.totalBatDescAh = 0;
    for (uint8_t h = 0; h < HIST_HORAS; h++) {
        dia.totalPainelAh  += dia.horas[h].painelAh;
        dia.totalCasaAh    += dia.horas[h].casaAh;
        dia.totalBatCargAh += dia.horas[h].batCargAh;
        dia.totalBatDescAh += dia.horas[h].batDescAh;
    }
}

void salvarContadores() {
    prefs.begin("rrfs_cnt", false);
    prefs.putFloat("p_ah",  contadores.painelAh);
    prefs.putFloat("c_ah",  contadores.casaAh);
    prefs.putFloat("bc_ah", contadores.batCargAh);
    prefs.putFloat("bd_ah", contadores.batDescAh);
    prefs.end();
}

void carregarContadores() {
    prefs.begin("rrfs_cnt", true);
    contadores.painelAh  = prefs.getFloat("p_ah",  0.0f);
    contadores.casaAh    = prefs.getFloat("c_ah",  0.0f);
    contadores.batCargAh = prefs.getFloat("bc_ah", 0.0f);
    contadores.batDescAh = prefs.getFloat("bd_ah", 0.0f);
    prefs.end();
}

void salvarDiaFlash(uint16_t idx, const RegistroDia& dia) {
    char ns[12]; snprintf(ns, sizeof(ns), "rh%03d", idx%HIST_DIAS);
    prefs.begin(ns, false); prefs.putBytes("d", &dia, sizeof(RegistroDia)); prefs.end();
}

bool carregarDiaFlash(uint16_t idx, RegistroDia& dia) {
    char ns[12]; snprintf(ns, sizeof(ns), "rh%03d", idx%HIST_DIAS);
    prefs.begin(ns, true);
    size_t len = prefs.getBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();
    return (len == sizeof(RegistroDia) && dia.valido);
}

void salvarIndiceHistorico() {
    prefs.begin("rrfs_hidx", false);
    prefs.putUChar ("ver",   HIST_KEY_VER);
    prefs.putUShort("idx",   histIndiceAtual);
    prefs.putUShort("total", histTotalDias);
    prefs.end();
}

void carregarHistorico() {
    prefs.begin("rrfs_hidx", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver != HIST_KEY_VER) {
        prefs.end();
        histIndiceAtual = 0; histTotalDias = 0;
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
        adicionarLog("Hist reset v23"); return;
    }
    histIndiceAtual = prefs.getUShort("idx",   0);
    histTotalDias   = prefs.getUShort("total", 0);
    prefs.end();
    if (histTotalDias > 0 && !carregarDiaFlash(histIndiceAtual, histDiaAtual))
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
}

void atualizarDiaAtual() {
    unsigned long agora = millis();
    if (agora - ultimaAtualizacaoDia < 60000UL && diaCache != 0) return;
    ultimaAtualizacaoDia = agora;
    struct tm t;
    if (obterTempoAtual(t)) {
        diaCache = t.tm_mday; mesCache = t.tm_mon+1; anoCache = t.tm_year+1900;
    } else if (!diaCache) {
        diaCache = 1; mesCache = 1; anoCache = 2024;
    }
}

void acumularEnergia() {
    atualizarDiaAtual();
    unsigned long agora = millis();
    struct tm t;
    uint8_t  hA=0, dA=diaCache, mA=mesCache;
    uint16_t aA=anoCache;
    if (obterTempoAtual(t)) {
        hA=t.tm_hour; dA=t.tm_mday; mA=t.tm_mon+1; aA=t.tm_year+1900;
    } else { hA=(agora/3600000UL)%24; }

    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum=agora; acum.horaAtual=hA;
        acum.diaAtual=dA; acum.mesAtual=mA; acum.anoAtual=aA;
        bool diaEx=false;
        if (histTotalDias>0) {
            RegistroDia dT;
            if (carregarDiaFlash(histIndiceAtual,dT) &&
                dT.dia==dA && dT.mes==mA && dT.ano==aA)
                { histDiaAtual=dT; diaEx=true; }
        }
        if (!diaEx) {
            memset(&histDiaAtual,0,sizeof(RegistroDia));
            histDiaAtual.ano=aA; histDiaAtual.mes=mA;
            histDiaAtual.dia=dA; histDiaAtual.valido=true;
            if (histTotalDias==0) { histIndiceAtual=0; histTotalDias=1; }
            else {
                RegistroDia dT;
                if (carregarDiaFlash(histIndiceAtual,dT) &&
                    (dT.dia!=dA||dT.mes!=mA||dT.ano!=aA)) {
                    histIndiceAtual=(histIndiceAtual+1)%HIST_DIAS;
                    if (histTotalDias<HIST_DIAS) histTotalDias++;
                }
            }
            salvarDiaFlash(histIndiceAtual,histDiaAtual);
            salvarIndiceHistorico();
        }
        return;
    }

    float dH=(float)(agora-acum.ultimoAcum)/3600000.0f;
    acum.ultimoAcum=agora;
    if (dH>0.1f) dH=0.1f;

    float ip=sensores.correntePainel;
    float ic=sensores.correnteCasa;
    float ib=sensores.correnteBateria;

    if (ip>0.15f)  acum.painelAh  += ip*dH;
    if (ic>0.3f)   acum.casaAh    += ic*dH;
    if (ib>0.3f)   acum.batCargAh += ib*dH;
    if (ib<-0.3f)  acum.batDescAh += fabsf(ib)*dH;

    bool mudouHora=(hA!=acum.horaAtual);
    bool mudouDia =(dA!=acum.diaAtual||mA!=acum.mesAtual||aA!=acum.anoAtual);

    if (mudouHora||mudouDia) {
        uint8_t hi=acum.horaAtual%HIST_HORAS;
        histDiaAtual.horas[hi].painelAh  += acum.painelAh;
        histDiaAtual.horas[hi].casaAh    += acum.casaAh;
        histDiaAtual.horas[hi].batCargAh += acum.batCargAh;
        histDiaAtual.horas[hi].batDescAh += acum.batDescAh;
        calcularTotaisDia(histDiaAtual);
        contadores.painelAh  += acum.painelAh;
        contadores.casaAh    += acum.casaAh;
        contadores.batCargAh += acum.batCargAh;
        contadores.batDescAh += acum.batDescAh;
        if (mudouDia) {
            salvarDiaFlash(histIndiceAtual,histDiaAtual);
            histIndiceAtual=(histIndiceAtual+1)%HIST_DIAS;
            if (histTotalDias<HIST_DIAS) histTotalDias++;
            memset(&histDiaAtual,0,sizeof(RegistroDia));
            histDiaAtual.ano=aA; histDiaAtual.mes=mA;
            histDiaAtual.dia=dA; histDiaAtual.valido=true;
        }
        salvarDiaFlash(histIndiceAtual,histDiaAtual);
        salvarIndiceHistorico(); salvarContadores();
        acum.painelAh=acum.casaAh=acum.batCargAh=acum.batDescAh=0;
        acum.horaAtual=hA;
        if (mudouDia) { acum.diaAtual=dA; acum.mesAtual=mA; acum.anoAtual=aA; }
    }

    static unsigned long tParcial=0;
    if (agora-tParcial>=300000UL) {
        tParcial=agora;
        salvarDiaFlash(histIndiceAtual,histDiaAtual);
        salvarContadores();
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] MÁQUINA INVERSOR ALTA v23
// ═══════════════════════════════════════════════════════════
void maquinaInvAlta() {
    if (modoTeste) return;
    unsigned long agora = millis();

    if (!sistemaEstabilizado) {
        if (agora - tInicioSistema < 120000UL) {
            if (!estados.invAlta) {
                ligarInvAlta();
                maqAlta.estado        = INV_A_LIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: ON (boot)");
            }
            return;
        } else {
            sistemaEstabilizado = true;
            adicionarLog("InvA: sistema estabilizado");
        }
    }

    if (wifiReconectando || (agora - tUltimaReconWifi2 < 30000UL)) return;

    bool bmsConectada = dadosBMS.valido &&
                        (millis() - dadosBMS.tUltimaDado < 30000UL);
    bool batOK = !bmsConectada || (dadosBMS.soc > 0.5f);
    if (!cfgBMS.configurada) batOK = true;

    switch (maqAlta.estado) {

        case INV_A_DESLIGADO:
            if (batOK) {
                ligarInvAlta();
                maqAlta.estado        = INV_A_DIAG_AGUARDA;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: ligou, aguardando AC");
            }
            break;

        case INV_A_LIGADO:
            if (!batOK) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: DESLIGADO bat/SOC=0%");
            } else if (!acAltaPresente()) {
                maqAlta.estado        = INV_A_DIAG_AGUARDA;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: AC perdido, diagnostico");
            }
            break;

        case INV_A_DIAG_AGUARDA:
            if (!batOK) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: SOC=0% (diag)");
            } else if (acAltaPresente()) {
                maqAlta.estado        = INV_A_LIGADO;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas    = 0;
                maqAlta.totalLigadas++;
                adicionarLog("InvA: AC OK, LIGADO");
            } else if (agora - maqAlta.tInicioEstado >= DIAG_AGUARDA_AC_MS) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DIAG_OFF;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas++;
                snprintf(tempBuf, sizeof(tempBuf),
                    "InvA: sem AC, tentativa %d/%d",
                    maqAlta.tentativas, DIAG_MAX_TENTATIVAS);
                adicionarLog(tempBuf);
            }
            break;

        case INV_A_DIAG_OFF:
            if (!batOK) {
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: SOC=0% (off diag)");
            } else if (agora - maqAlta.tInicioEstado >= DIAG_DESLIGA_MS) {
                if (maqAlta.tentativas >= DIAG_MAX_TENTATIVAS) {
                    maqAlta.tentativas = 0;
                    maqAlta.totalFalhas++;
                    adicionarLog("InvA: falha definitiva, retry 60s");
                    maqAlta.tInicioEstado = agora + 55000UL;
                } else {
                    ligarInvAlta();
                    maqAlta.estado        = INV_A_DIAG_AGUARDA;
                    maqAlta.tInicioEstado = agora;
                    adicionarLog("InvA: religando diagnostico");
                }
            }
            break;
    }
}

// ═══════════════════════════════════════════════════════════
// [LAMP]
// ═══════════════════════════════════════════════════════════
void controlarLampExt() {
    if (modoTeste) return;
    unsigned long agora = millis();
    if (agora - ultimoLampCheck < LAMP_EXT_CHECK_MS) return;
    ultimoLampCheck = agora;

    bool gerando = (sensores.correntePainel > 0.5f);
    bool batOK = !dadosBMS.valido || (dadosBMS.soc > limiares.pctLampExt);

    struct tm t;
    bool tempoOK = obterTempoAtual(t);

    if (gerando && estados.lampExt) {
        desligarLampExt(); adicionarLog("LampExt OFF: solar"); return;
    }
    if (estados.lampExt && !batOK) {
        desligarLampExt(); adicionarLog("LampExt OFF: SOC baixo"); return;
    }
    if (estados.lampExt && tempoOK) {
        if (t.tm_hour == LAMP_EXT_HORA_DESLIGA && t.tm_min >= LAMP_EXT_MIN_DESLIGA) {
            desligarLampExt(); adicionarLog("LampExt OFF: 5:30"); return;
        }
        if (t.tm_hour > LAMP_EXT_HORA_DESLIGA && t.tm_hour < 7) {
            desligarLampExt(); adicionarLog("LampExt OFF: apos 5:30"); return;
        }
    }
    if (!gerando && !estados.lampExt && batOK && estados.invAlta && tempoOK) {
        bool noturno = false;
        if (t.tm_hour >= LAMP_EXT_HORA_LIGA) noturno = true;
        else if (t.tm_hour < LAMP_EXT_HORA_DESLIGA) noturno = true;
        else if (t.tm_hour == LAMP_EXT_HORA_DESLIGA &&
                 t.tm_min  <  LAMP_EXT_MIN_DESLIGA)  noturno = true;
        if (noturno) {
            ligarLampExt();
            char msgBuf[48];
            snprintf(msgBuf, sizeof(msgBuf),
                "LampExt ON: %02d:%02d", t.tm_hour, t.tm_min);
            adicionarLog(msgBuf);
        }
    }
}

void controlarEnergia() {
    if (modoTeste) return;
    maquinaInvAlta();
    controlarLampExt();
}

// ═══════════════════════════════════════════════════════════
// [PREF] Preferências
// ═══════════════════════════════════════════════════════════
bool existeCredenciais() {
    prefs.begin("rrfs",true); bool ok=prefs.isKey("user"); prefs.end(); return ok;
}
bool existeWiFiConfig() {
    prefs.begin("rrfs",true); bool ok=prefs.isKey("wifi_ok"); prefs.end(); return ok;
}
void salvarCredenciais(const String& u, const String& p) {
    prefs.begin("rrfs",false); prefs.putString("user",u); prefs.putString("pass",p); prefs.end();
}
bool verificarLogin(const String& u, const String& p) {
    prefs.begin("rrfs",true);
    String su=prefs.getString("user",""), sp=prefs.getString("pass","");
    prefs.end();
    return (su==u && sp==p && su.length()>0);
}
void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap) {
    prefs.begin("rrfs",false);
    prefs.putBool  ("usar_ap",   ap);
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.putString("wifi_ip",   ip);
    prefs.putString("wifi_gw",   gw);
    prefs.putString("wifi_mask", mask);
    prefs.putString("wifi_dns",  dns);
    prefs.putBool  ("wifi_ok",   true);
    prefs.end();
}
void carregarWifi(String& ssid, String& pass, String& ip,
                  String& gw,   String& mask,  String& dns, bool& ap) {
    prefs.begin("rrfs",true);
    ap   = prefs.getBool  ("usar_ap",   true);
    ssid = prefs.getString("wifi_ssid", "");
    pass = prefs.getString("wifi_pass", "");
    ip   = prefs.getString("wifi_ip",   "");
    gw   = prefs.getString("wifi_gw",   "");
    mask = prefs.getString("wifi_mask", "255.255.255.0");
    dns  = prefs.getString("wifi_dns",  "8.8.8.8");
    prefs.end();
}
void salvarConfigInversores() {
    prefs.begin("rrfs_inv",false);
    prefs.putBool ("inv_inva", cfgInv.invertReleInvAlta);
    prefs.putBool ("inv_lamp", cfgInv.invertReleLampExt);
    prefs.putUChar("ver", 25);
    prefs.end();
}
void carregarConfigInversores() {
    prefs.begin("rrfs_inv",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if (ver==25) {
        cfgInv.invertReleInvAlta = prefs.getBool("inv_inva",false);
        cfgInv.invertReleLampExt = prefs.getBool("inv_lamp", false);
    }
    prefs.end();
    adicionarLog("Config inv v23");
}
void salvarLimiaresSinal() {
    prefs.begin("rrfs_lsig",false);
    prefs.putInt("lim_ac", limiarSinal.limiarAcMV);
    prefs.end();
}
void carregarLimiaresSinal() {
    prefs.begin("rrfs_lsig",true);
    limiarSinal.limiarAcMV = prefs.getInt("lim_ac", AC_LIMIAR_MV_DEFAULT);
    prefs.end();
}
void salvarLimiares() {
    prefs.begin("rrfs_lim",false);
    prefs.putFloat("lim_lamp", limiares.pctLampExt);
    prefs.putUChar("ver", 25);
    prefs.end();
}
void carregarLimiares() {
    prefs.begin("rrfs_lim",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if (ver==25) limiares.pctLampExt = prefs.getFloat("lim_lamp", 20.0f);
    else         limiares.pctLampExt = 20.0f;
    prefs.end();
}
void carregarCalibSensores() {
    calibPainel = {PAINEL_OFFSET_MV, PAINEL_SENS_MV_A, true};
    prefs.begin("rrfs_cal",true);
    if (prefs.isKey("pn_off"))  calibPainel.offsetMV = prefs.getInt  ("pn_off",  calibPainel.offsetMV);
    if (prefs.isKey("pn_sens")) calibPainel.sensMvA  = prefs.getFloat("pn_sens", calibPainel.sensMvA);
    prefs.end();
}

// ═══════════════════════════════════════════════════════════
// [WIFI]
// ═══════════════════════════════════════════════════════════
void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP RRFS 10.0.0.1");
    logRAM(); ledBlink();
}

bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns) {
    IPAddress lIP, lGW, lMask, lDNS;
    if (!lIP.fromString(ip))     return false;
    if (!lGW.fromString(gw))     return false;
    if (!lMask.fromString(mask)) return false;
    if (!lDNS.fromString(dns))   lDNS.fromString("8.8.8.8");

    gWifiSSID=ssid; gWifiPASS=pass;
    gWifiIP=ip; gWifiGW=gw; gWifiMASK=mask; gWifiDNS=dns;

    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.config(lIP, lGW, lMask, lDNS);
    WiFi.begin(ssid.c_str(), pass.c_str());

    adicionarLog("WiFi conectando"); ledBlink();
    unsigned long t = millis();
    while (WiFi.status()!=WL_CONNECTED && millis()-t<15000) {
        atualizarLED(); esp_task_wdt_reset(); delay(100);
    }
    if (WiFi.status()==WL_CONNECTED) {
        adicionarLog("WiFi OK");
        server.stop(); delay(50); server.begin();
        modoAP=false; wifiFalhasConsec=0; ledFixo(true);
        return true;
    }
    adicionarLog("WiFi FALHOU"); return false;
}

void gerenciarWiFi() {
    unsigned long agora = millis();
    if (agora - tUltimoCheckWifi < WIFI_CHECK_INTERVALO) return;
    tUltimoCheckWifi = agora;

    if (modoAP && WiFi.getMode()==WIFI_AP) {
        if (WiFi.softAPIP()==IPAddress(0,0,0,0)) {
            adicionarLog("AP caiu: reiniciando"); iniciarAP();
        }
        return;
    }
    if (WiFi.status()==WL_CONNECTED) {
        if (wifiReconectando) {
            adicionarLog("WiFi: reconectado OK");
            if (WiFi.getMode()==WIFI_AP_STA) {
                WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA);
                dnsServer.stop(); modoAP=false;
                adicionarLog("AP temporario encerrado");
            }
            server.stop(); delay(50); server.begin(); ledFixo(true);
        }
        wifiFalhasConsec=0; wifiReconectando=false; tInicioRecon=0;
        tUltimaReconWifi2=millis();
        return;
    }
    if (agora-tUltimoReconWifi<WIFI_RECON_INTERVALO) return;
    tUltimoReconWifi=agora; wifiFalhasConsec++;
    snprintf(tempBuf,sizeof(tempBuf),"WiFi: falha %d/%d",wifiFalhasConsec,WIFI_MAX_FALHAS);
    adicionarLog(tempBuf);

    if (!wifiReconectando) {
        wifiReconectando=true; tInicioRecon=agora;
        if (!gWifiIP.isEmpty()) {
            IPAddress lIP,lGW,lMask,lDNS;
            lIP.fromString(gWifiIP); lGW.fromString(gWifiGW);
            lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
            WiFi.config(lIP,lGW,lMask,lDNS);
        }
        WiFi.reconnect(); adicionarLog("WiFi: tentando reconectar"); return;
    }
    if (agora-tInicioRecon>=WIFI_RECON_TIMEOUT) {
        wifiReconectando=false; tInicioRecon=0;
        if (wifiFalhasConsec>=WIFI_MAX_FALHAS) {
            adicionarLog("WiFi: AP emergencial");
            wifiFalhasConsec=0;
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAPConfig(AP_IP,AP_GW,AP_MASK);
            WiFi.softAP(AP_SSID);
            dnsServer.stop(); dnsServer.start(DNS_PORT,"*",AP_IP);
            modoAP=true; ledBlink();
            adicionarLog("AP emergencial: 10.0.0.1");
            if (!gWifiSSID.isEmpty())
                WiFi.begin(gWifiSSID.c_str(),gWifiPASS.c_str());
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [HELPERS WEB]
// ═══════════════════════════════════════════════════════════
String nomeEstadoAlta(EstadoAlta e) {
    switch(e) {
        case INV_A_DESLIGADO:    return F("DESLIGADO");
        case INV_A_LIGADO:       return F("LIGADO");
        case INV_A_DIAG_AGUARDA: return F("AGUARD AC");
        case INV_A_DIAG_OFF:     return F("DIAG OFF");
        default:                 return F("?");
    }
}

// ═══════════════════════════════════════════════════════════
// [SETUP]
// ═══════════════════════════════════════════════════════════
void setup() {
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms    = 30000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    logsCarregados = false;
    carregarLogsPersistentes();

    esp_err_t errNVS = nvs_flash_init();
    if (errNVS == ESP_ERR_NVS_NO_FREE_PAGES ||
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND)
        adicionarLog("NVS aviso");

    switch (esp_reset_reason()) {
        case ESP_RST_BROWNOUT: adicionarLog("RESET BROWNOUT"); break;
        case ESP_RST_PANIC:    adicionarLog("RESET PANIC");    break;
        case ESP_RST_TASK_WDT: adicionarLog("RESET WDT");     break;
        case ESP_RST_SW:       adicionarLog("Reset SW");       break;
        case ESP_RST_POWERON:  adicionarLog("Power ON");       break;
        default:
            snprintf(tempBuf,sizeof(tempBuf),"Reset %d",(int)esp_reset_reason());
            adicionarLog(tempBuf); break;
    }

    esp_task_wdt_reset();
    setupLED();
    setupBotao();
    carregarConfigInversores();
    setupReles();

    pinMode(PIN_SENSOR_PAINEL, INPUT);
    pinMode(PIN_AC_SINAL_ALTA, INPUT);

    ledBlink();
    adicionarLog("RRFS v23 CORRIGIDO - InvAlta + BMS BLE + Banco");
    logRAM();
    esp_task_wdt_reset();

    carregarCalibSensores();
    carregarLimiaresSinal();
    carregarLimiares();
    carregarHistorico();
    carregarContadores();
    carregarConfigTempo();
    carregarConfigPrevisao();
    carregarConfigBMS();
    carregarConfigBanco();
    esp_task_wdt_reset();

    ultimoContagem = millis();
    tInicioSistema      = millis();
    sistemaEstabilizado = false;
    tUltimaReconWifi2   = 0;

    bool usarAP = true;
    if (existeWiFiConfig()) {
        String ssid,pass,ip,gw,mask,dns;
        carregarWifi(ssid,pass,ip,gw,mask,dns,usarAP);
        esp_task_wdt_reset();
        if (!usarAP && ssid.length()>0)
            if (iniciarWiFiNB(ssid,pass,ip,gw,mask,dns)) usarAP=false;
    }
    esp_task_wdt_reset();
    if (usarAP) iniciarAP();

    if (!modoAP && cfgTempo.ntpAtivo) {
        esp_task_wdt_reset();
        sincronizarNTP();
    }

    if (cfgBMS.configurada) {
        adicionarLog("BMS: aguardando conexao BLE no loop");
    }

    esp_task_wdt_reset();
    configurarRotas();
    adicionarLog("Sistema pronto v23 CORRIGIDO");
    logRAM();
    esp_task_wdt_reset();
}

// ═══════════════════════════════════════════════════════════
// [LOOP]
// ═══════════════════════════════════════════════════════════
void loop() {
    esp_task_wdt_reset();
    verificarReset();
    atualizarLED();

    if (modoAP || WiFi.getMode()==WIFI_AP_STA)
        dnsServer.processNextRequest();
    server.handleClient();
    esp_task_wdt_reset();

    unsigned long agora = millis();

    // ── Leitura sensores (a cada 500ms) ─────────────
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;

        esp_task_wdt_reset();
        lerSensores();
        server.handleClient();
        esp_task_wdt_reset();

        processarSinalAC();
        server.handleClient();
        esp_task_wdt_reset();

        controlarEnergia();
        acumularEnergia();
        server.handleClient();
        esp_task_wdt_reset();
    }

    // ── Controle extra (a cada 1s) ──────────────────
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;
        controlarLampExt();
        esp_task_wdt_reset();
        server.handleClient();
    }

    // ── BLE BMS ─────────────────────────────────────
    gerenciarBLE();
    esp_task_wdt_reset();
    server.handleClient();

    // ── NTP (a cada hora) ───────────────────────────
    if (cfgTempo.ntpAtivo && !modoAP &&
        WiFi.status()==WL_CONNECTED &&
        agora - tUltimoNTP >= INTERVALO_NTP_MS) {
        sincronizarNTP();
        esp_task_wdt_reset();
    }

    // ── Previsão solar ──────────────────────────────
    if (!modoAP && WiFi.status()==WL_CONNECTED) {
        gerenciarPrevisao();
        esp_task_wdt_reset();
    }

    // ── WiFi ────────────────────────────────────────
    gerenciarWiFi();
    esp_task_wdt_reset();

    // ── Log RAM a cada 10 min ───────────────────────
    {
        static unsigned long tRAM = 0;
        if (agora - tRAM >= 600000UL) { tRAM = agora; logRAM(); }
    }

    esp_task_wdt_reset();
}

// ═══════════════════════════════════════════════════════════
// [WEB] CSS
// ═══════════════════════════════════════════════════════════
static const char CSS[] PROGMEM = R"RAW(
body{background:#0d0d0d;color:#e0e0e0;font-family:Arial,sans-serif;margin:0;padding:10px}
.card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:15px;max-width:520px;margin:0 auto}
h2{color:#4fc3f7;text-align:center;margin:0 0 15px;font-size:18px}
h3{color:#81d4fa;font-size:14px;margin:12px 0 8px;border-bottom:1px solid #333;padding-bottom:6px}
label{display:block;color:#90a4ae;font-size:12px;margin-top:10px;margin-bottom:3px}
input,select{width:100%;padding:9px;background:#111;border:1px solid #444;border-radius:4px;color:#e0e0e0;font-size:13px;box-sizing:border-box}
input:focus,select:focus{outline:none;border-color:#0277bd}
button{width:100%;padding:11px;margin-top:12px;background:#0277bd;color:#fff;border:none;border-radius:4px;font-size:14px;font-weight:bold;cursor:pointer}
button:hover{background:#039be5}
.btn-r{background:#c62828}.btn-r:hover{background:#ef5350}
.btn-g{background:#2e7d32}.btn-g:hover{background:#43a047}
.btn-o{background:#ef6c00}.btn-o:hover{background:#f57c00}
.btn-s{background:#546e7a}.btn-s:hover{background:#607d8b}
.btn-b{background:#4a148c}.btn-b:hover{background:#6a1b9a}
.btn-sm{font-size:12px;padding:8px}
.ok{background:#1b5e20;color:#a5d6a7;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.er{background:#b71c1c;color:#ef9a9a;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.wn{background:#e65100;color:#ffe0b2;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.nf{background:#0d47a1;color:#90caf9;padding:10px;border-radius:4px;margin:8px 0;font-size:12px;line-height:1.5}
.al{background:#4a0000;border:2px solid #c62828;color:#ef9a9a;padding:10px;border-radius:6px;margin:6px 0;font-size:12px}
.nav{display:flex;gap:4px;margin-top:12px;padding-top:10px;border-top:1px solid #333;flex-wrap:wrap}
.nav a{flex:1;min-width:36px;text-align:center;padding:9px 4px;background:#1e1e1e;color:#90a4ae;text-decoration:none;border-radius:4px;font-size:11px;border:1px solid #333}
.nav a.active{background:#0277bd;color:#fff;border-color:#0277bd}
.nav a:hover{background:#263238;color:#e0e0e0}
.sc{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:6px 0}
.sc .lbl{color:#546e7a;font-size:11px;margin-bottom:4px}
.sc .val{color:#e0e0e0;font-size:20px;font-weight:bold}
.sc .sub{color:#78909c;font-size:11px;margin-top:2px}
.bb{background:#111;border:1px solid #333;border-radius:4px;height:18px;margin:6px 0;overflow:hidden}
.bv{height:100%;border-radius:4px;transition:width .5s;text-align:center;line-height:18px;font-size:11px;font-weight:bold;color:#fff}
.lb{color:#546e7a;font-size:10px;text-transform:uppercase;margin-top:15px;margin-bottom:4px}
.lbx{background:#000;border:1px solid #222;border-radius:4px;padding:8px;height:160px;overflow-y:auto;font-family:monospace;font-size:10px;color:#76ff03}
.ll{padding:2px 0;border-bottom:1px solid #0a0a0a}
.cg{border-radius:8px;border:2px solid #222;background:#111;padding:9px 11px;margin-bottom:5px}
.cr{display:flex;justify-content:space-between;align-items:center}
.cn{font-size:12px;color:#90a4ae}
.cs{font-size:13px;font-weight:bold}
.ci{font-size:10px;color:#546e7a;margin-top:5px;padding-top:5px;border-top:1px solid #1e1e1e}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.grid4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px}
.pd{background:#0a1a0a;border:1px solid #1b5e20;border-radius:6px;padding:10px;margin:4px 0}
.pd .dt{font-size:11px;color:#43a047;font-weight:bold;margin-bottom:4px}
.pd .dv{font-size:18px;font-weight:bold;color:#a5d6a7}
.pd .ds{font-size:10px;color:#546e7a;margin-top:2px}
.sig-box{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:5px 0}
.sig-lbl{font-size:11px;color:#90a4ae;font-weight:bold;margin-bottom:4px}
.sig-val{font-size:18px;font-weight:bold;margin:3px 0}
.sig-sub{font-size:10px;color:#546e7a}
.bms-card{background:#0a0a1a;border:2px solid #4a148c;border-radius:10px;padding:12px;margin:6px 0}
.bms-title{font-size:11px;color:#ce93d8;font-weight:bold;margin-bottom:8px}
.bms-val{font-size:20px;font-weight:bold;color:#e1bee7}
.bms-sub{font-size:10px;color:#7b1fa2;margin-top:3px}
.banco-card{background:#0a1a2a;border:2px solid #01579b;border-radius:10px;padding:12px;margin:6px 0}
.banco-title{font-size:11px;color:#81d4fa;font-weight:bold;margin-bottom:8px}
.banco-badge{display:inline-block;background:#01579b;color:#e1f5fe;font-size:16px;font-weight:bold;padding:6px 14px;border-radius:6px;letter-spacing:2px}
.pulse{animation:pulse 1s infinite}
@keyframes pulse{0%{opacity:1}50%{opacity:.5}100%{opacity:1}}
)RAW";

// ═══════════════════════════════════════════════════════════
// [WEB] HELPERS
// ═══════════════════════════════════════════════════════════
String barraProgresso(float pct, const String& label) {
    if (pct < 0) pct = 0; if (pct > 100) pct = 100;
    String cor = pct >= 50 ? "#2e7d32" : pct >= 25 ? "#ef6c00" : "#c62828";
    char buf[200];
    snprintf(buf, sizeof(buf),
        "<div class='bb'><div class='bv' style='width:%d%%;background:%s'>%d%%</div></div>",
        (int)pct, cor.c_str(), (int)pct);
    String h = String(buf);
    if (label.length())
        h += "<div style='font-size:11px;color:#546e7a;text-align:center'>" + label + "</div>";
    return h;
}

String gerarPagina(const String& titulo, const String& conteudo, bool comNav) {
    String html;
    html.reserve(conteudo.length() + 3000);
    html  = F("<!DOCTYPE html><html><head>"
              "<meta charset='utf-8'>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>"); html += CSS; html += F("</style></head><body><div class='card'>");
    html += F("<h2>&#9889; RRFS v23</h2>");
    html += conteudo;
    if (comNav && logado) {
        String pag = server.uri();
        html += F("<div class='nav'>");
        html += "<a href='/'          " + (pag=="/"         ?String("class='active'"):String("")) + ">&#128202;</a>";
        html += "<a href='/bateria'   " + (pag=="/bateria"  ?String("class='active'"):String("")) + ">&#128267;</a>";
        html += "<a href='/controle'  " + (pag=="/controle" ?String("class='active'"):String("")) + ">&#9881;</a>";
        html += "<a href='/sensores'  " + (pag=="/sensores" ?String("class='active'"):String("")) + ">&#128270;</a>";
        html += "<a href='/bms'       " + (pag=="/bms"      ?String("class='active'"):String("")) + ">&#128268;</a>";
        html += "<a href='/previsao'  " + (pag=="/previsao" ?String("class='active'"):String("")) + ">&#127774;</a>";
        html += "<a href='/historico' " + (pag=="/historico"?String("class='active'"):String("")) + ">&#128200;</a>";
        html += "<a href='/config'    " + (pag=="/config"   ?String("class='active'"):String("")) + ">&#128736;</a>";
        html += F("</div>");
    }
    html += F("<div class='lb'>&#128221; Logs</div>"
              "<div class='lbx' id='lb'><div style='color:#666'>...</div></div>"
              "</div>"
              "<script>"
              "function aL(){var x=new XMLHttpRequest();"
              "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
              "try{var d=document.getElementById('lb');"
              "var a=JSON.parse(x.responseText);var h='';"
              "for(var i=0;i<a.length;i++)h+='<div class=\"ll\">'+a[i]+'</div>';"
              "d.innerHTML=h||'<div style=\"color:#666\">Sem logs</div>';"
              "d.scrollTop=d.scrollHeight;}catch(e){}}};"
              "x.open('GET','/logs',true);x.send();}"
              "setInterval(aL,4000);aL();"
              "</script></body></html>");
    return html;
}

static const char* const NAV_URL[8] = {
    "/", "/bateria", "/controle", "/sensores",
    "/bms", "/previsao", "/historico", "/config"
};
static const char* const NAV_ICO[8] = {
    "&#128202;", "&#128267;", "&#9881;", "&#128270;",
    "&#128268;", "&#127774;", "&#128200;", "&#128736;"
};

void paginaInicio(const char* titulo) {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html;charset=utf-8", "");
    server.sendContent(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'><title>"));
    server.sendContent(String(titulo));
    server.sendContent(F("</title><style>"));
    server.sendContent_P(CSS);
    server.sendContent(F("</style></head><body><div class='card'>"
                         "<h2>&#9889; RRFS v23</h2>"));
}

void paginaFim(bool comNav = true) {
    if (comNav && logado) {
        String pag = server.uri();
        String n = F("<div class='nav'>");
        for (uint8_t i = 0; i < 8; i++) {
            n += "<a href='"; n += NAV_URL[i]; n += "'";
            if (pag == NAV_URL[i]) n += " class='active'";
            n += ">"; n += NAV_ICO[i]; n += "</a>";
        }
        n += F("</div>");
        server.sendContent(n);
    }
    server.sendContent(F("<div class='lb'>&#128221; Logs</div>"
              "<div class='lbx' id='lb'><div style='color:#666'>...</div></div>"
              "</div>"
              "<script>"
              "function aL(){var x=new XMLHttpRequest();"
              "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
              "try{var d=document.getElementById('lb');"
              "var a=JSON.parse(x.responseText);var h='';"
              "for(var i=0;i<a.length;i++)h+='<div class=\"ll\">'+a[i]+'</div>';"
              "d.innerHTML=h||'<div style=\"color:#666\">Sem logs</div>';"
              "d.scrollTop=d.scrollHeight;}catch(e){}}};"
              "x.open('GET','/logs',true);x.send();}"
              "setInterval(aL,4000);aL();"
              "</script></body></html>"));
    server.sendContent("");
}

// ═══════════════════════════════════════════════════════════
// [API] /dados — JSON completo v23 CORRIGIDO
// ═══════════════════════════════════════════════════════════
void handleDados() {
    if (!logado) { server.send(403); return; }

    unsigned long tempoA   = (millis() - maqAlta.tInicioEstado) / 1000UL;
    EstimativaTempo tempo   = calcularTempoRestante();
    float ahH = 0, ahR = 0;
    if (cfgPrevisao.ativa && previsao.valido) {
        ahH = getAhPrevHoraAtual();
        ahR = getAhPrevRestanteDia();
    }
    unsigned long tBMSSeg = dadosBMS.tUltimaDado > 0
        ? (millis() - dadosBMS.tUltimaDado) / 1000UL : 9999UL;

    char buf[1400];
    snprintf(buf, sizeof(buf),
        "{"
        "\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
        "\"wp\":%.0f,\"wc\":%.0f,\"wb\":%.0f,"
        "\"v_banco\":%.2f,\"soc\":%.1f,\"ciclos\":%d,"
        "\"cap_banco\":%.2f,"
        "\"nS\":%d,\"nP\":%d,"
        "\"inva\":%s,\"le\":%s,"
        "\"est_alta\":%d,\"nome_alta\":\"%s\","
        "\"tempo_a\":%lu,\"lig_a\":%d,\"fal_a\":%d,"
        "\"ac_alta_mv\":%.0f,\"ac_alta_ok\":%s,"
        "\"lim_lamp\":%.0f,"
        "\"tempo_val\":%s,\"tempo_h\":%d,\"tempo_m\":%d,\"tempo_carg\":%s,"
        "\"mv_painel\":%d,"
        "\"prev_ok\":%s,\"ah_prev_h\":%.2f,\"ah_prev_r\":%.2f,"
        "\"ntp\":%s,\"modo_teste\":%s,"
        "\"ram\":%.1f,\"uptime\":%lu,"
        "\"data_hora\":\"%s\","
        "\"bms_corr\":%.2f,\"bms_tensao\":%.2f,"
        "\"bms_soc\":%.1f,\"bms_ciclos\":%d,\"bms_cap\":%.2f,"
        "\"bms_valido\":%s,\"bms_t_seg\":%lu,"
        "\"ble_estado\":%d,\"ble_str\":\"%s\","
        "\"bms_cfg\":%s,\"bms_mac\":\"%s\","
        "\"ble_falhas\":%d"
        "}",
        sensores.correntePainel,
        sensores.correnteCasa,
        sensores.correnteBateria,
        sensores.potenciaPainel,
        sensores.potenciaCasa,
        fabsf(sensores.potenciaBateria),
        sensores.tensaoBanco,
        sensores.socBMS,
        (int)sensores.ciclosBMS,
        sensores.capacidadeBanco,
        (int)banco.nS, (int)banco.nP,
        estados.invAlta  ? "true" : "false",
        estados.lampExt  ? "true" : "false",
        (int)maqAlta.estado,
        nomeEstadoAlta(maqAlta.estado).c_str(),
        tempoA,
        (int)maqAlta.totalLigadas,
        (int)maqAlta.totalFalhas,
        acAlta.mediaMV,
        acAlta.presente  ? "true" : "false",
        limiares.pctLampExt,
        tempo.valido     ? "true" : "false",
        tempo.horas, tempo.minutos,
        tempo.carregando ? "true" : "false",
        sensores.sinalPainel_mV,
        previsao.valido  ? "true" : "false",
        ahH, ahR,
        ntpSincronizado  ? "true" : "false",
        modoTeste        ? "true" : "false",
        esp_get_free_heap_size() / 1024.0f,
        millis() / 1000UL,
        formatarDataHora().c_str(),
        dadosBMS.corrente,
        dadosBMS.tensao,
        dadosBMS.soc,
        (int)dadosBMS.ciclos,
        dadosBMS.capacidadeAh,
        dadosBMS.valido  ? "true" : "false",
        tBMSSeg,
        (int)ctrlBLE.estado,
        statusBLEStr().c_str(),
        cfgBMS.configurada ? "true" : "false",
        cfgBMS.macAddress,
        (int)ctrlBLE.falhasConsec
    );
    server.send(200, "application/json", buf);
}

// ═══════════════════════════════════════════════════════════
// [AUTH]
// ═══════════════════════════════════════════════════════════
void handleSetupLogin() {
    if (existeCredenciais() && existeWiFiConfig())
        { server.sendHeader("Location","/login");      server.send(302); return; }
    if (existeCredenciais() && !existeWiFiConfig())
        { server.sendHeader("Location","/setup-wifi"); server.send(302); return; }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String u=server.arg("user"), p=server.arg("pass"), cf=server.arg("conf");
        if (u.length()<3)      msg = "<div class='er'>Min 3 chars usuario</div>";
        else if (p.length()<4) msg = "<div class='er'>Min 4 chars senha</div>";
        else if (p!=cf)        msg = "<div class='er'>Senhas nao conferem</div>";
        else { salvarCredenciais(u,p); server.sendHeader("Location","/setup-wifi"); server.send(302); return; }
    }
    String c = F("<div class='nf'>&#8505; Primeiro acesso.</div><h3>&#128274; Criar Conta</h3>");
    c += msg;
    c += F("<form method='POST' action='/setup-login'>"
           "<label>Usuario:</label><input type='text' name='user' maxlength='32' required autocomplete='off'>"
           "<label>Senha:</label><input type='password' name='pass' maxlength='32' required>"
           "<label>Confirmar:</label><input type='password' name='conf' maxlength='32' required>"
           "<button type='submit'>Criar &#8594;</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Setup",c,false));
}

void handleSetupWifi() {
    if (!existeCredenciais())
        { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (existeWiFiConfig() && !logado)
        { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String modo = server.arg("modo");
        if (modo == "ap") {
            salvarWifi("","","","","","",true);
            server.send(200,"text/html;charset=utf-8",
                gerarPagina("Salvo",F("<div class='ok'>&#10003; Reiniciando...</div>"
                "<div class='nf'>SSID: <b>RRFS Energia</b> | IP: <b>10.0.0.1</b></div>"),false));
            delay(2000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"), wp=server.arg("wpass"),
               ip=server.arg("ip"),   gw=server.arg("gw"),
               mask=server.arg("mask"), dns=server.arg("dns");
        if (ssid.isEmpty())     msg = "<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7) msg = "<div class='er'>Informe o IP</div>";
        else if (gw.length()<7) msg = "<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wp,ip,gw,mask,dns,false);
            server.send(200,"text/html;charset=utf-8",
                gerarPagina("Salvo","<div class='ok'>&#10003; Acesse: http://"+ip+
                "</div><div class='nf'>Reiniciando...</div>",false));
            delay(2000); ESP.restart(); return;
        }
    }
    static int16_t nRedes=-1; static bool scanIniciado=false;
    if (!scanIniciado) {
        if (WiFi.getMode()==WIFI_AP) { WiFi.mode(WIFI_AP_STA); delay(100); }
        if (WiFi.status()!=WL_CONNECTED) { WiFi.setAutoReconnect(false); WiFi.disconnect(); delay(100); }
        esp_task_wdt_reset();
        WiFi.scanDelete(); WiFi.scanNetworks(true,false,false,300);
        scanIniciado=true; nRedes=-1; adicionarLog("Scan WiFi iniciado");
    }
    nRedes = WiFi.scanComplete();
    String c = F("<div class='nf'>&#8505; IP fixo obrigatorio.</div><h3>&#128246; Rede</h3>");
    c += msg;
    c += F("<form method='POST' action='/setup-wifi'>"
           "<label>Modo:</label>"
           "<select name='modo' id='modo' onchange='toggleModo()'>"
           "<option value='wifi'>Conectar ao WiFi</option>"
           "<option value='ap'>Ponto de Acesso</option></select>"
           "<div id='dw'><label>Rede:</label><select name='ssid'>");
    bool escaneando=false;
    if (nRedes==WIFI_SCAN_RUNNING) { c+=F("<option>Escaneando...</option>"); escaneando=true; }
    else if (nRedes==WIFI_SCAN_FAILED||nRedes==0) {
        c+=F("<option value=''>Nenhuma rede</option>");
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
    } else if (nRedes>0) {
        for (int i=0;i<nRedes;i++) {
            String q=WiFi.RSSI(i)>-60?"Bom":WiFi.RSSI(i)>-75?"Regular":"Fraco";
            char b2[32]; snprintf(b2,sizeof(b2)," [%s %ddBm]",q.c_str(),WiFi.RSSI(i));
            c+="<option value='"+WiFi.SSID(i)+"'>"+WiFi.SSID(i)+String(b2)+"</option>";
        }
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
    }
    c+=F("</select>"
         "<button type='button' class='btn-o btn-sm' onclick='location.reload()' style='margin-top:8px'>&#128259; Atualizar</button>"
         "<label>Senha WiFi:</label><input type='password' name='wpass' placeholder='Vazio se aberta'>"
         "<label>IP Fixo:</label><input type='text' name='ip' required placeholder='192.168.1.100'>"
         "<label>Gateway:</label><input type='text' name='gw' required placeholder='192.168.1.1'>"
         "<label>Mascara:</label><input type='text' name='mask' required value='255.255.255.0'>"
         "<label>DNS:</label><input type='text' name='dns' required value='8.8.8.8'></div>"
         "<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");
    if (logado) c+=F("<button onclick='location.href=\"/\"' class='btn-s' style='margin-top:8px'>&#8592; Voltar</button>");
    c+=F("<script>function toggleModo(){"
         "document.getElementById('dw').style.display="
         "document.getElementById('modo').value=='wifi'?'block':'none';}</script>");
    if (escaneando) c+=F("<script>setTimeout(function(){location.reload();},3000);</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Rede",c,false));
}

void handleLogin() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (logado) { server.sendHeader("Location","/"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        if (verificarLogin(server.arg("user"),server.arg("pass"))) {
            logado=true; adicionarLog("Login OK"); logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        }
        adicionarLog("Login FALHOU");
        msg=F("<div class='er'>&#10007; Usuario/senha incorretos</div>");
    }
    String c=F("<div class='nf'>&#128274; Acesso seguro</div><h3>Entrar</h3>");
    c+=msg;
    c+=F("<form method='POST' action='/login'>"
         "<label>Usuario:</label><input type='text' name='user' required autocomplete='off'>"
         "<label>Senha:</label><input type='password' name='pass' required>"
         "<button type='submit'>&#128275; Entrar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Login",c,false));
}

void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado=false; }
    server.sendHeader("Location","/login"); server.send(302);
}

// ═══════════════════════════════════════════════════════════
// [DASH] PÁGINA PRINCIPAL v23 CORRIGIDO
// ═══════════════════════════════════════════════════════════
void handleRoot() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (!existeWiFiConfig())  { server.sendHeader("Location","/setup-wifi");  server.send(302); return; }
    if (!logado)              { server.sendHeader("Location","/login");        server.send(302); return; }

    paginaInicio("RRFS - Painel");

    if (!cfgBMS.configurada)
        server.sendContent(F("<div class='wn' style='font-size:11px'>&#128268; BMS nao configurada. "
             "<a href='/bms' style='color:#ffe0b2'>Configurar BMS</a></div>"));

    server.sendContent(F("<div style='display:flex;justify-content:space-between;align-items:center;"
         "font-size:11px;color:#546e7a;margin-bottom:6px'>"
         "<span id='dh_txt'>&#128336; --/--/---- --:--</span>"
         "<span id='upd_ind' style='color:#43a047;font-size:10px'>&#9679; ao vivo</span>"
         "</div>"));

    // ── Banco de Baterias ────────────────────────────────────
    server.sendContent(F("<h3>&#128267; Banco de Baterias</h3>"
         "<div class='banco-card'>"
         "<div class='banco-title'>&#9889; Configuracao do Banco</div>"
         "<div class='cr' style='margin-bottom:8px'>"
         "<div id='banco_badge' class='banco-badge'>--</div>"
         "<div style='text-align:right'>"
         "<div style='font-size:10px;color:#546e7a'>Total baterias</div>"
         "<div id='banco_total' style='font-size:16px;font-weight:bold;color:#4fc3f7'>--</div>"
         "</div></div>"
         "<div class='grid4'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>SOC</div>"
         "<div id='bms_soc' class='bms-val' style='font-size:18px'>--</div>"
         "<div class='bms-sub'>%</div>"
         "</div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Tensao</div>"
         "<div id='banco_v' style='font-size:18px;font-weight:bold;color:#4fc3f7'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>V</div>"
         "</div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Corrente</div>"
         "<div id='banco_i' style='font-size:18px;font-weight:bold'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>A</div>"
         "</div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Ciclos</div>"
         "<div id='bms_ciclos' style='font-size:18px;font-weight:bold;color:#90a4ae'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>BMS</div>"
         "</div>"
         "</div>"
         "<div class='bb' style='margin-top:8px'>"
         "<div id='soc_bv' class='bv' style='width:0%'>0%</div></div>"
         "<div class='grid2' style='margin-top:6px'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Capacidade</div>"
         "<div id='banco_cap' style='font-size:14px;font-weight:bold'>--</div>"
         "</div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>&#9889; Potencia</div>"
         "<div id='banco_w' style='font-size:14px;font-weight:bold;color:#fdd835'>--</div>"
         "</div>"
         "</div>"
         "<div id='banco_dir' style='text-align:center;margin-top:6px;font-size:12px'>--</div>"
         "<div id='ble_status' style='text-align:center;font-size:10px;color:#546e7a;margin-top:4px'>BLE: --</div>"
         "</div>"));

    // ── Inversor Alta + Sinal AC ─────────────────────────────
    server.sendContent(F("<h3>&#9889; Inversor Alta</h3>"
         "<div id='card_a' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#9889; Inversor Alta Potencia</span>"
         "<span id='st_a' class='cs'>--</span>"
         "</div>"
         "<div id='sub_a' class='ci'>--</div>"
         "</div>"
         "<h3>&#128246; Sinal AC</h3>"
         "<div class='sig-box'>"
         "<div class='cr'>"
         "<span class='sig-lbl'>GPIO33 &mdash; Inversor Alta</span>"
         "<span id='ac_a_txt' class='sig-val' style='font-size:14px'>--</span>"
         "</div>"
         "<div id='ac_a_sub' class='sig-sub'>--</div>"
         "<div class='bb'><div id='ac_a_bv' class='bv' style='width:0%'></div></div>"
         "</div>"));

    // ── Previsão solar ───────────────────────────────────────
    if (cfgPrevisao.ativa && cfgPrevisao.configurada) {
        server.sendContent(F("<div id='prev_box' class='pd' style='display:none'>"
             "<div class='dt'>&#127774; Previsao Solar</div>"
             "<div class='grid2'>"
             "<div style='text-align:center'>"
             "<div style='font-size:10px;color:#546e7a'>Esta hora</div>"
             "<div id='prev_h' class='dv'>--</div></div>"
             "<div style='text-align:center'>"
             "<div style='font-size:10px;color:#546e7a'>Restante hoje</div>"
             "<div id='prev_r' class='dv'>--</div></div>"
             "</div>"
             "<div class='ds'><a href='/previsao' style='color:#43a047'>Ver completo</a></div>"
             "</div>"));
    }

    // ── SVG Fluxo ────────────────────────────────────────────
    server.sendContent(F("<h3 style='text-align:center;margin:8px 0 6px'>&#9889; Fluxo de Energia</h3>"
         "<svg viewBox='0 0 340 220' style='width:100%;height:auto;display:block'>"
         "<path id='ln_pb' d='M110,50 C160,20 200,20 240,50' fill='none' stroke='#333' stroke-width='3'/>"
         "<path id='ln_pc' d='M90,85 C80,125 90,145 130,168' fill='none' stroke='#333' stroke-width='3'/>"
         "<path id='ln_bc' d='M255,85 C260,125 250,145 210,168' fill='none' stroke='#333' stroke-width='3'/>"
         "<g><rect x='20' y='10' width='100' height='80' rx='12' fill='#181818' id='rect_p' stroke='#455a64' stroke-width='2'/>"
         "<text x='70' y='36' text-anchor='middle' font-size='20'>&#9728;</text>"
         "<text x='70' y='50' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>"
         "<text id='sv_ip' x='70' y='65' text-anchor='middle' font-size='13' fill='#455a64' font-weight='bold'>0.0A</text>"
         "<text id='sv_wp' x='70' y='80' text-anchor='middle' font-size='11' fill='#455a64'>0W</text></g>"
         "<g><rect x='220' y='10' width='100' height='80' rx='12' fill='#181818' id='rect_b' stroke='#455a64' stroke-width='2'/>"
         "<text id='sv_ico_b' x='270' y='34' text-anchor='middle' font-size='18'>&#128267;</text>"
         "<text x='270' y='48' text-anchor='middle' font-size='9' fill='#78909c'>BANCO</text>"
         "<text id='sv_soc' x='270' y='63' text-anchor='middle' font-size='13' fill='#455a64' font-weight='bold'>--%</text>"
         "<text id='sv_ib' x='270' y='78' text-anchor='middle' font-size='11' fill='#455a64'>--</text></g>"
         "<g><rect id='rect_c' x='105' y='163' width='130' height='55' rx='10' fill='#181818' stroke='#3a3a3a' stroke-width='2'/>"
         "<text x='130' y='195' font-size='18'>&#127968;</text>"
         "<text id='sv_ico_inv' x='145' y='175' text-anchor='start' font-size='9' fill='#78909c'>CASA</text>"
         "<text id='sv_ic' x='228' y='190' text-anchor='end' font-size='13' fill='#3a3a3a' font-weight='bold'>--</text>"
         "<text id='sv_wc' x='228' y='205' text-anchor='end' font-size='11' fill='#3a3a3a'>--</text></g>"
         "</svg>"));

    // ── Grid correntes + info ────────────────────────────────
    server.sendContent(F("<div class='grid3' style='gap:6px;margin:4px 0'>"
         "<div class='sc' style='padding:8px;text-align:center'>"
         "<div class='lbl'>&#9728; Painel</div>"
         "<div id='cc_p' style='font-size:14px;font-weight:bold'>--</div>"
         "<div id='cw_p' style='font-size:11px;color:#78909c'>--</div></div>"
         "<div class='sc' style='padding:8px;text-align:center'>"
         "<div class='lbl'>&#127968; Casa</div>"
         "<div id='cc_c' style='font-size:14px;font-weight:bold;color:#4fc3f7'>--</div>"
         "<div id='cw_c' style='font-size:11px;color:#78909c'>--</div></div>"
         "<div class='sc' style='padding:8px;text-align:center'>"
         "<div class='lbl'>&#128267; Banco</div>"
         "<div id='cc_b' style='font-size:14px;font-weight:bold'>--</div>"
         "<div id='cw_b' style='font-size:11px;color:#78909c'>--</div></div>"
         "</div>"
         "<div id='sys_info' class='nf' style='font-size:10px'>Carregando...</div>"));

    // ── JAVASCRIPT (parte 1) ─────────────────────────────────
    server.sendContent(F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function sBc(id,c){var e=document.getElementById(id);if(e)e.style.borderColor=c;}"
    "function sAttr(id,a,v){var e=document.getElementById(id);if(e)e.setAttribute(a,v);}"
    "function atualizarDash(d){"
    "sT('dh_txt','&#128336; '+d.data_hora+(d.ntp?' <span style=\"color:#43a047\">&#9679;NTP</span>':''));"
    "sT('banco_badge',d.nS+'S'+d.nP+'P');"
    "sT('banco_total',(d.nS*d.nP)+' bat');"
    "var cSOC=d.soc>=50?'#43a047':d.soc>=25?'#ef6c00':'#c62828';"
    "sT('bms_soc','<span style=\"color:'+cSOC+'\">'+d.soc.toFixed(0)+'</span>');"
    "var socBv=document.getElementById('soc_bv');"
    "if(socBv){socBv.style.width=Math.min(100,d.soc)+'%';socBv.style.background=cSOC;"
    "  socBv.textContent=d.soc.toFixed(0)+'%';}"
    "sT('banco_v',d.v_banco.toFixed(1));"
    "var cIB=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
    "sT('banco_i','<span style=\"color:'+cIB+'\">'+Math.abs(d.ib).toFixed(2)+'</span>');"
    "sT('bms_ciclos',d.ciclos);"
    "sT('banco_cap',d.cap_banco.toFixed(2)+' Ah');"
    "sT('banco_w',Math.round(Math.abs(d.wb))+'W');"
    "var dirStr=d.ib>0.2?'&#9650; Carregando '+d.ib.toFixed(2)+'A':"
    "           d.ib<-0.2?'&#9660; Descarregando '+Math.abs(d.ib).toFixed(2)+'A':'&#8652; Parado';"
    "var dirCol=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
    "sT('banco_dir','<span style=\"color:'+dirCol+'\">'+dirStr+'</span>');"
    "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#43a047':'#ef5350';"
    "sT('ble_status','BLE: <span style=\"color:'+cBLE+'\">'+d.ble_str+'</span>"
    " | '+d.bms_t_seg+'s atras');"
    "var cA=d.inva?'#fdd835':'#546e7a';"
    "sBc('card_a',cA);"
    "sT('st_a','<span style=\"color:'+cA+'\">'+(d.inva?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "sT('sub_a',d.nome_alta+' | '+d.tempo_a+'s | F:'+d.fal_a);"
    "var cAcA=d.ac_alta_ok?'#43a047':'#546e7a';"
    "sT('ac_a_txt','<span style=\"color:'+cAcA+'\">'+(d.ac_alta_ok?'&#9679; OK':'&#9675; SEM AC')+'</span>');"
    "sT('ac_a_sub',d.ac_alta_mv.toFixed(0)+' mV');"
    "var bAcA=document.getElementById('ac_a_bv');"
    "if(bAcA){bAcA.style.width=Math.min((d.ac_alta_mv/3300)*100,100)+'%';bAcA.style.background=cAcA;}"
    "var pBox=document.getElementById('prev_box');"
    "if(pBox){"
    "  if(d.prev_ok&&(d.ah_prev_h>0||d.ah_prev_r>0)){"
    "    pBox.style.display='block';"
    "    sT('prev_h',d.ah_prev_h.toFixed(1)+' Ah');"
    "    sT('prev_r',d.ah_prev_r.toFixed(1)+' Ah');"
    "  } else pBox.style.display='none';"
    "}"));

    // ── JAVASCRIPT (parte 2) ─────────────────────────────────
    server.sendContent(F(
    "var painelGer=(d.ip>0.5);"
    "var batCarg=(d.ib>0.2);"
    "var batDesc=(d.ib<-0.2);"
    "var cP=painelGer?'#fdd835':'#455a64';"
    "var cBat=d.soc>=50?'#43a047':d.soc>=25?'#ef6c00':'#c62828';"
    "var cCas=d.inva?'#fdd835':'#3a3a3a';"
    "var lPB=(painelGer&&batCarg)?'#43a047':'#333';"
    "var lPC=(painelGer&&d.ic>0.3)?'#fdd835':'#333';"
    "var lBC=(batDesc&&d.ic>0.3)?'#ef6c00':'#333';"
    "sAttr('rect_p','stroke',cP);"
    "sAttr('rect_b','stroke',cBat);"
    "sAttr('rect_c','stroke',cCas);"
    "sAttr('ln_pb','stroke',lPB);"
    "sAttr('ln_pc','stroke',lPC);"
    "sAttr('ln_bc','stroke',lBC);"
    "var svIp=document.getElementById('sv_ip');"
    "if(svIp){svIp.textContent=painelGer?d.ip.toFixed(1)+'A':'0.0A';svIp.style.fill=cP;}"
    "var svWp=document.getElementById('sv_wp');"
    "if(svWp){svWp.textContent=painelGer?Math.round(d.wp)+'W':'0W';svWp.style.fill=cP;}"
    "sT('sv_ico_b',batCarg?'&#9650;&#128267;':batDesc?'&#9660;&#128267;':'&#128267;');"
    "var svSoc=document.getElementById('sv_soc');"
    "if(svSoc){svSoc.textContent=d.soc.toFixed(0)+'%';svSoc.style.fill=cBat;}"
    "var svIb=document.getElementById('sv_ib');"
    "if(svIb){svIb.textContent=(batCarg?'+':batDesc?'-':'~')+Math.abs(d.ib).toFixed(1)+'A';svIb.style.fill=cBat;}"
    "sT('sv_ico_inv',d.inva?'CASA &#9889;':'CASA &#9711;');"
    "var svIc=document.getElementById('sv_ic');"
    "if(svIc){svIc.textContent=d.ic.toFixed(1)+'A';svIc.style.fill=cCas;}"
    "var svWc=document.getElementById('sv_wc');"
    "if(svWc){svWc.textContent=Math.round(d.wc)+'W';svWc.style.fill=cCas;}"
    "var cP2=painelGer?'#fdd835':'#455a64';"
    "var cB2=batCarg?'#43a047':batDesc?'#ef6c00':'#fdd835';"
    "sT('cc_p','<span style=\"color:'+cP2+'\">'+(painelGer?d.ip.toFixed(1)+'A':'0.0A')+'</span>');"
    "sT('cw_p',painelGer?Math.round(d.wp)+'W':'0W');"
    "sT('cc_c',d.ic.toFixed(1)+'A');"
    "sT('cw_c',Math.round(d.wc)+'W');"
    "var siBI=(d.ib>0.2?'+':d.ib<-0.2?'-':'~')+Math.abs(d.ib).toFixed(1)+'A';"
    "sT('cc_b','<span style=\"color:'+cB2+'\">'+siBI+'</span>');"
    "sT('cw_b',Math.round(Math.abs(d.wb))+'W');"
    "var si='&#128246; '+d.ram.toFixed(1)+'KB RAM | Up '+d.uptime+'s"
    " | Cap '+d.cap_banco.toFixed(1)+'Ah';"
    "if(d.modo_teste)si+=' | <span style=\"color:#ef6c00\">MODO TESTE</span>';"
    "sT('sys_info',si);"
    "var ind=document.getElementById('upd_ind');"
    "if(ind){ind.style.opacity='1';setTimeout(function(){if(ind)ind.style.opacity='0.3';},300);}"
    "}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarDash(JSON.parse(x.responseText));}catch(e){console.log(e);}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>"));

    paginaFim(true);
}

// ── Scan: inicia ──────────────────────────────────────────
void handleBMSScan() {
    if (!logado) { server.send(403); return; }
    if (!bleScanAtivo) bleScanSolicitado = true;
    server.send(200, "application/json", "{\"ok\":true}");
}

// ── Scan: resultado (JSON) ────────────────────────────────
void handleBMSLista() {
    if (!logado) { server.send(403); return; }
    String j = "{\"scan\":";
    j += (bleScanAtivo || bleScanSolicitado) ? "true" : "false";
    j += ",\"itens\":[";
    uint8_t n = bleListaN;
    for (uint8_t i = 0; i < n; i++) {
        if (i) j += ",";
        j += "{\"mac\":\""; j += bleLista[i].mac; j += "\",\"nome\":\"";
        for (const char* p = bleLista[i].nome; *p; p++) {
            char ch = *p;
            if (ch == '"' || ch == '\\' || (unsigned char)ch < 32 ||
                (unsigned char)ch > 126) ch = ' ';
            j += ch;
        }
        j += "\",\"rssi\":"; j += String(bleLista[i].rssi);
        j += ",\"jbd\":";    j += bleLista[i].jbd ? "true" : "false";
        j += "}";
    }
    j += "]}";
    server.send(200, "application/json", j);
}

// ═══════════════════════════════════════════════════════════
// [BMS-WEB] /bms — Config BMS + Banco v23
// ═══════════════════════════════════════════════════════════
void handleBMS() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "salvar_mac") {
            String mac = server.arg("mac");
            mac.trim(); mac.toUpperCase();
            bool valido = (mac.length() == 17);
            if (valido) {
                for (int i = 0; i < 17; i++) {
                    char ch = mac[i];
                    if (i%3==2) { if (ch!=':') { valido=false; break; } }
                    else { if (!isxdigit(ch)) { valido=false; break; } }
                }
            }
            if (!valido) {
                msg = "<div class='er'>&#10007; MAC invalido. Formato: AA:BB:CC:DD:EE:FF</div>";
            } else {
                strncpy(cfgBMS.macAddress, mac.c_str(), sizeof(cfgBMS.macAddress)-1);
                cfgBMS.macAddress[sizeof(cfgBMS.macAddress)-1] = '\0';
                cfgBMS.configurada = true;
                salvarConfigBMS();
                desconectarBMS();
                ctrlBLE.tUltimoTentativa = 0;
                ctrlBLE.falhasConsec     = 0;
                dadosBMS.valido          = false;
                msg = "<div class='ok'>&#10003; BMS salva (" + mac + "). Conectando...</div>";
            }
        } else if (acao == "salvar_banco") {
            int nS = server.arg("nS").toInt();
            int nP = server.arg("nP").toInt();
            if (nS<1||nS>8) msg="<div class='er'>Serie: 1 a 8</div>";
            else if (nP<1||nP>8) msg="<div class='er'>Paralelo: 1 a 8</div>";
            else {
                banco.nS = (uint8_t)nS;
                banco.nP = (uint8_t)nP;
                banco.configurado = true;
                salvarConfigBanco();
                msg = "<div class='ok'>&#10003; Banco salvo: " + String(nS) + "S" +
                      String(nP) + "P (" + String(nS*nP) + " baterias)</div>";
            }
        } else if (acao == "reconectar") {
            desconectarBMS();
            ctrlBLE.tUltimoTentativa = 0;
            ctrlBLE.falhasConsec     = 0;
            msg = "<div class='ok'>&#10003; Reconectando...</div>";
            adicionarLog("BMS: reconexao manual");
        } else if (acao == "desconectar") {
            desconectarBMS();
            msg = "<div class='ok'>&#10003; Desconectado.</div>";
            adicionarLog("BMS: desconexao manual");
        } else if (acao == "limpar") {
            desconectarBMS();
            cfgBMS.configurada   = false;
            cfgBMS.macAddress[0] = '\0';
            dadosBMS.valido      = false;
            salvarConfigBMS();
            msg = "<div class='ok'>&#10003; BMS removida.</div>";
            adicionarLog("BMS: config removida");
        }
    }

    String c = msg;

    c+=F("<div class='nf'>&#128268; <b>BMS Bluetooth JBD/LanmaoBMS</b><br>"
         "1) Feche o app LanmaoBMS no celular (a BMS aceita so 1 conexao).<br>"
         "2) Toque em <b>Buscar BMS</b> e escolha a sua na lista.</div>");

    // ── BMS atual ────────────────────────────────────────────
    c+=F("<h3>&#9881; BMS selecionada</h3>");
    if (cfgBMS.configurada) {
        char macBuf[96];
        snprintf(macBuf, sizeof(macBuf),
            "<div class='ok' style='font-family:monospace;letter-spacing:2px'>"
            "&#10003; <b>%s</b></div>", cfgBMS.macAddress);
        c += String(macBuf);
    } else {
        c+=F("<div class='wn'>&#9888; Nenhuma BMS selecionada</div>");
    }

    // ── Buscar BMS ───────────────────────────────────────────
    c+=F("<button type='button' class='btn-b' id='btnScan' onclick='iniciarScan()'>"
         "&#128269; Buscar BMS</button>"
         "<div id='scan_st' style='text-align:center;font-size:11px;color:#78909c;margin:6px 0'></div>"
         "<div id='scan_lista'></div>"
         "<details style='margin-top:8px'><summary style='font-size:12px;color:#78909c'>"
         "Informar MAC manualmente</summary>"
         "<form method='POST' action='/bms'>"
         "<input type='hidden' name='acao' value='salvar_mac'>"
         "<label>MAC (AA:BB:CC:DD:EE:FF):</label>"
         "<input type='text' name='mac' maxlength='17' placeholder='AA:BB:CC:DD:EE:FF' "
         "style='font-family:monospace;text-transform:uppercase' required>"
         "<button type='submit' class='btn-b btn-sm'>&#128190; Salvar MAC</button></form>"
         "</details>");

    c+=F(R"JS(<script>
var _poll=null;
function sel(mac){
 var f=document.createElement('form');f.method='POST';f.action='/bms';
 var a=document.createElement('input');a.type='hidden';a.name='acao';a.value='salvar_mac';f.appendChild(a);
 var m=document.createElement('input');m.type='hidden';m.name='mac';m.value=mac;f.appendChild(m);
 document.body.appendChild(f);f.submit();
}
function iniciarScan(){
 document.getElementById('btnScan').disabled=true;
 document.getElementById('scan_st').textContent='Buscando... (ate 10s)';
 document.getElementById('scan_lista').innerHTML='';
 var x=new XMLHttpRequest();x.open('GET','/bms-scan',true);x.send();
 if(_poll)clearInterval(_poll);
 _poll=setInterval(lerLista,1500);
}
function lerLista(){
 var x=new XMLHttpRequest();
 x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){
  try{var d=JSON.parse(x.responseText);rl(d);
   if(!d.scan){clearInterval(_poll);_poll=null;
    document.getElementById('btnScan').disabled=false;
    document.getElementById('scan_st').textContent=d.itens.length?
     'Concluido: '+d.itens.length+' dispositivo(s). \u2605 = provavel BMS':
     'Nenhum dispositivo. Feche o app da BMS e tente de novo.';}
  }catch(e){}}};
 x.open('GET','/bms-lista',true);x.send();
}
function rl(d){
 var el=document.getElementById('scan_lista');el.innerHTML='';
 var it=d.itens.slice();
 it.sort(function(a,b){return (b.jbd-a.jbd)||(b.rssi-a.rssi);});
 for(var i=0;i<it.length;i++){(function(o){
  var row=document.createElement('div');row.className='cg';
  row.style.borderColor=o.jbd?'#7b1fa2':'#222';
  var l=document.createElement('div');l.className='cr';
  var t=document.createElement('span');t.className='cn';
  t.textContent=(o.jbd?'\u2605 ':'')+o.nome+' | '+o.mac+' | '+o.rssi+'dBm';
  var b=document.createElement('button');b.textContent='Usar';
  b.className='btn-g btn-sm';b.style.width='auto';b.style.marginTop='0';
  b.onclick=function(){sel(o.mac);};
  l.appendChild(t);l.appendChild(b);row.appendChild(l);el.appendChild(row);
 })(it[i]);}
}
</script>)JS");

    // ── Status BLE ao vivo ───────────────────────────────────
    c+=F("<h3>&#128202; Status BLE</h3>"
         "<div class='bms-card'>"
         "<div class='grid2'>"
         "<div><div style='font-size:10px;color:#7b1fa2'>Estado BLE</div>"
         "<div id='b_estado' class='bms-val' style='font-size:16px'>--</div></div>"
         "<div><div style='font-size:10px;color:#7b1fa2'>Falhas</div>"
         "<div id='b_falhas' class='bms-val' style='font-size:16px'>--</div></div>"
         "</div>"
         "<div class='grid3' style='margin-top:8px'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Corrente BMS</div>"
         "<div id='b_corr' class='bms-val'>--</div><div id='b_dir' class='bms-sub'>--</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Tensao BMS</div>"
         "<div id='b_tensao' class='bms-val'>--</div><div class='bms-sub'>V/bat</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Capacidade</div>"
         "<div id='b_cap' class='bms-val'>--</div><div class='bms-sub'>Ah</div></div>"
         "</div>"
         "<div class='grid3' style='margin-top:6px'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>SOC</div>"
         "<div id='b_soc' class='bms-val'>--</div><div class='bms-sub'>%</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Ciclos</div>"
         "<div id='b_ciclos' class='bms-val'>--</div><div class='bms-sub'>BMS</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Ultima</div>"
         "<div id='b_ultima' class='bms-val' style='font-size:14px'>--</div>"
         "<div class='bms-sub'>seg atras</div></div>"
         "</div></div>");

    // ── Configuração Banco ───────────────────────────────────
    c+=F("<h3>&#128267; Configuracao do Banco</h3><div class='banco-card'>");
    char bancoBuf[160];
    snprintf(bancoBuf, sizeof(bancoBuf),
        "<div style='text-align:center;margin-bottom:10px'>"
        "<span class='banco-badge'>%dS%dP</span>"
        " <span style='font-size:12px;color:#4fc3f7;margin-left:8px'>"
        "%d baterias no total</span></div>",
        banco.nS, banco.nP, banco.nS * banco.nP);
    c += String(bancoBuf);
    c+=F("<div class='nf' style='font-size:11px'>"
         "<b>nS</b> = baterias em serie (soma tensao)<br>"
         "<b>nP</b> = baterias em paralelo (soma corrente)</div>"
         "<div id='banco_preview' class='sc' style='margin:8px 0;text-align:center'>"
         "<div class='lbl'>Preview com dados atuais da BMS</div>"
         "<div id='bp_v' style='font-size:14px;color:#4fc3f7'>--</div>"
         "<div id='bp_i' style='font-size:14px;color:#43a047'>--</div>"
         "<div id='bp_w' style='font-size:14px;color:#fdd835'>--</div>"
         "<div id='bp_cap' style='font-size:14px;color:#ce93d8'>--</div></div></div>");

    c+=F("<form method='POST' action='/bms'>"
         "<input type='hidden' name='acao' value='salvar_banco'>"
         "<div class='grid2' style='gap:10px'>"
         "<div><label>Serie (nS)</label><select name='nS' id='sel_nS' onchange='atualizarPreview()'>");
    for (int i=1;i<=8;i++) {
        char opt[56];
        snprintf(opt, sizeof(opt), "<option value='%d'%s>%d serie%s</option>",
                 i, i==banco.nS?" selected":"", i, i>1?"s":"");
        c += String(opt);
    }
    c+=F("</select></div><div><label>Paralelo (nP)</label>"
         "<select name='nP' id='sel_nP' onchange='atualizarPreview()'>");
    for (int i=1;i<=8;i++) {
        char opt[56];
        snprintf(opt, sizeof(opt), "<option value='%d'%s>%d paralelo%s</option>",
                 i, i==banco.nP?" selected":"", i, i>1?"s":"");
        c += String(opt);
    }
    c+=F("</select></div></div>"
         "<button type='submit' class='btn-g'>&#128190; Salvar Banco</button></form>");

    // ── Ações BLE ────────────────────────────────────────────
    c+=F("<h3>&#9881; Acoes BLE</h3>"
         "<form method='POST' action='/bms'><input type='hidden' name='acao' value='reconectar'>"
         "<button type='submit' class='btn-g btn-sm'>&#8635; Reconectar</button></form>"
         "<form method='POST' action='/bms' style='margin-top:6px'>"
         "<input type='hidden' name='acao' value='desconectar'>"
         "<button type='submit' class='btn-o btn-sm'>&#9209; Desconectar</button></form>"
         "<form method='POST' action='/bms' style='margin-top:6px' "
         "onsubmit='return confirm(\"Remover BMS?\")'>"
         "<input type='hidden' name='acao' value='limpar'>"
         "<button type='submit' class='btn-r btn-sm'>&#128465; Remover BMS</button></form>");

    // ── JS status ao vivo ────────────────────────────────────
    c+=F("<script>"
    "var _bmsV=0,_bmsI=0,_bmsCap=0;"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function atualizarPreview(){"
    "var nS=parseInt(document.getElementById('sel_nS').value)||1;"
    "var nP=parseInt(document.getElementById('sel_nP').value)||1;"
    "var vB=(_bmsV*nS).toFixed(2);var iB=(_bmsI*nP).toFixed(2);"
    "var cB=(_bmsCap*nP).toFixed(2);"
    "var wB=(parseFloat(vB)*parseFloat(iB)).toFixed(0);"
    "sT('bp_v',nS+'S: V_bms x'+nS+' = '+vB+'V');"
    "sT('bp_i',nP+'P: I_bms x'+nP+' = '+iB+'A');"
    "sT('bp_w','Potencia banco: '+wB+'W');"
    "sT('bp_cap','Capacidade: '+cB+'Ah');}"
    "function atualizarBMS(d){"
    "_bmsV=d.bms_tensao;_bmsI=d.bms_corr;_bmsCap=d.bms_cap;"
    "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#a5d6a7':'#ef9a9a';"
    "sT('b_estado','<span style=\"color:'+cBLE+'\">'+d.ble_str+'</span>');"
    "sT('b_falhas','<span style=\"color:'+(d.ble_falhas>3?'#ef5350':'#a5d6a7')+'\">'+d.ble_falhas+'</span>');"
    "var cI=d.bms_corr>0.2?'#43a047':d.bms_corr<-0.2?'#ef6c00':'#fdd835';"
    "sT('b_corr','<span style=\"color:'+cI+'\">'+Math.abs(d.bms_corr).toFixed(2)+'A</span>');"
    "var dS=d.bms_corr>0.2?'&#9650; Carg':d.bms_corr<-0.2?'&#9660; Desc':'&#8652; Parado';"
    "sT('b_dir','<span style=\"color:'+cI+'\">'+dS+'</span>');"
    "sT('b_tensao',d.bms_tensao.toFixed(2));"
    "sT('b_cap',d.bms_cap.toFixed(2));"
    "var cS=d.bms_soc>=50?'#43a047':d.bms_soc>=25?'#ef6c00':'#c62828';"
    "sT('b_soc','<span style=\"color:'+cS+'\">'+d.bms_soc.toFixed(0)+'</span>');"
    "sT('b_ciclos',d.bms_ciclos);sT('b_ultima',d.bms_t_seg);atualizarPreview();}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarBMS(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,3000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - BMS",c));
}

// ═══════════════════════════════════════════════════════════
// [CONTROLE-WEB]
// ═══════════════════════════════════════════════════════════
void handleControle() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "lim_lamp") {
            float ll = server.arg("lim_lamp").toFloat();
            if (ll<0||ll>95) msg="<div class='er'>SOC Lampada: 0-95%</div>";
            else {
                limiares.pctLampExt = ll;
                salvarLimiares();
                adicionarLog("Limiar lamp salvo");
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao == "reles") {
            cfgInv.invertReleInvAlta  = (server.arg("inv_inva")=="1");
            cfgInv.invertReleLampExt  = (server.arg("inv_lamp")=="1");
            salvarConfigInversores();
            if (estados.invAlta) ligarInvAlta();  else desligarInvAlta();
            if (estados.lampExt) ligarLampExt();  else desligarLampExt();
            adicionarLog("Inversao reles salva");
            msg="<div class='ok'>&#10003; Inversao salva!</div>";
        } else if (acao == "lim_ac") {
            int lim = server.arg("lim_ac").toInt();
            if (lim<50||lim>2000) msg="<div class='er'>Limiar 50-2000mV</div>";
            else {
                limiarSinal.limiarAcMV=lim;
                salvarLimiaresSinal();
                adicionarLog("Limiar AC salvo");
                msg="<div class='ok'>&#10003; Limiar AC salvo!</div>";
            }
        } else if (acao=="entrar_teste") {
            modoTeste=true; tInicioTeste=millis(); TIMEOUT_TESTE_MS=60000UL;
            adicionarLog("Modo teste ATIVO");
            msg="<div class='ok'>&#10003; Modo teste ativo 60s!</div>";
        } else if (acao=="sair_teste") {
            modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
            desligarInvAlta(); desligarLampExt();
            maqAlta.estado=INV_A_DESLIGADO; maqAlta.tInicioEstado=millis();
            adicionarLog("Modo teste encerrado");
            msg="<div class='ok'>&#10003; Teste encerrado!</div>";
        } else if (acao=="teste_a_on")    { if (!modoTeste) msg="<div class='er'>Ative o teste!</div>"; else { ligarInvAlta();   adicionarLog("TESTE: InvAlta ON");  msg="<div class='ok'>InvAlta ON</div>"; } }
        else if (acao=="teste_a_off")     { if (!modoTeste) msg="<div class='er'>Ative o teste!</div>"; else { desligarInvAlta();adicionarLog("TESTE: InvAlta OFF"); msg="<div class='ok'>InvAlta OFF</div>"; } }
        else if (acao=="teste_lamp_on")   { if (!modoTeste) msg="<div class='er'>Ative o teste!</div>"; else { ligarLampExt();   adicionarLog("TESTE: Lamp ON");     msg="<div class='ok'>Lamp ON</div>"; } }
        else if (acao=="teste_lamp_off")  { if (!modoTeste) msg="<div class='er'>Ative o teste!</div>"; else { desligarLampExt();adicionarLog("TESTE: Lamp OFF");    msg="<div class='ok'>Lamp OFF</div>"; } }
    }

    if (modoTeste && millis()-tInicioTeste>=TIMEOUT_TESTE_MS) {
        modoTeste=false; adicionarLog("Modo teste: timeout");
    }

    String c = msg;
    c+=F("<div class='nf' style='font-size:11px'>&#9889; <b>Controle v23</b></div>");

    // ── Status ao vivo ───────────────────────────────────────
    c+=F("<h3>&#128202; Status ao Vivo</h3>"
         "<div id='ctrl_card_a' class='cg'><div class='cr'>"
         "<span class='cn'>&#9889; Inv. Alta</span>"
         "<span id='ctrl_st_a' class='cs'>--</span></div>"
         "<div id='ctrl_sub_a' class='ci'>--</div></div>"
         "<div id='ctrl_card_lamp' class='cg'><div class='cr'>"
         "<span class='cn'>&#128161; Lampada Ext</span>"
         "<span id='ctrl_lamp' class='cs'>--</span></div></div>"
         "<div class='sc' style='margin-top:6px'>"
         "<div class='lbl'>&#128267; SOC Banco (BMS)</div>"
         "<div class='bb'><div id='ctrl_soc_bv' class='bv' style='width:0%'>0%</div></div>"
         "<div id='ctrl_soc_info' style='font-size:11px;color:#546e7a;text-align:center'>"
         "Aguardando BMS...</div></div>");

    // ── Limiar Lâmpada ───────────────────────────────────────
    c+=F("<h3>&#9881; Limiar Lampada</h3>"
         "<div class='nf' style='font-size:11px'>"
         "SOC% minimo (da BMS) para ligar a lampada externa.</div>"
         "<form method='POST' action='/controle'>"
         "<input type='hidden' name='acao' value='lim_lamp'>"
         "<label>SOC% Minimo para Lampada:</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",limiares.pctLampExt);
    c+="<input type='number' name='lim_lamp' min='0' max='95' step='1' required value='"+String(tempBuf)+"'>";
    c+=F("<div id='ctrl_lim_info' class='nf' style='font-size:11px;margin-top:4px'>"
         "SOC atual: <b id='ctrl_soc_live'>--</b>% | Limiar lamp: <b id='ctrl_lim_l_live'>--</b>%"
         "</div>"
         "<button type='submit' class='btn-g'>&#128190; Salvar Limiar</button></form>");

    // ── Limiar AC ────────────────────────────────────────────
    c+=F("<h3>&#128246; Limiar Sinal AC</h3>"
         "<form method='POST' action='/controle'>"
         "<input type='hidden' name='acao' value='lim_ac'>"
         "<label>Limiar AC (mV):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",limiarSinal.limiarAcMV);
    c+="<input type='number' name='lim_ac' min='50' max='2000' step='10' required value='"+String(tempBuf)+"'>";
    {
        char hBuf[80];
        snprintf(hBuf,sizeof(hBuf),"Liga >%dmV | Desliga <%dmV",
            limiarSinal.limiarAcMV+AC_HISTERESE_LIGA_MV,
            limiarSinal.limiarAcMV-AC_HISTERESE_DESLIGA_MV);
        c+="<div class='nf' style='font-size:11px'>"+String(hBuf)+"</div>";
    }
    c+=F("<button type='submit' class='btn-g'>&#128190; Salvar Limiar AC</button></form>");

    // ── Inversão relés ────────────────────────────────────────
    c+=F("<h3>&#128260; Inversao de Reles</h3>"
         "<form method='POST' action='/controle'>"
         "<input type='hidden' name='acao' value='reles'>"
         "<label>Rele Inv. Alta (GPIO27):</label>"
         "<select name='inv_inva'>");
    c+=cfgInv.invertReleInvAlta
        ?F("<option value='0'>Normal</option><option value='1' selected>Invertido</option>")
        :F("<option value='0' selected>Normal</option><option value='1'>Invertido</option>");
    c+=F("</select><label>Rele Lamp. Ext (GPIO12):</label><select name='inv_lamp'>");
    c+=cfgInv.invertReleLampExt
        ?F("<option value='0'>Normal</option><option value='1' selected>Invertido</option>")
        :F("<option value='0' selected>Normal</option><option value='1'>Invertido</option>");
    c+=F("</select><button type='submit' class='btn-o'>&#128190; Salvar Inversao</button></form>");

    // ── Modo Teste ───────────────────────────────────────────
    c+=F("<h3>&#128268; Teste de Reles</h3>");
    if (modoTeste) {
        unsigned long tr=(millis()-tInicioTeste>=TIMEOUT_TESTE_MS)?0
            :(TIMEOUT_TESTE_MS-(millis()-tInicioTeste))/1000UL;
        char tBuf[60];
        snprintf(tBuf,sizeof(tBuf),"&#9888; MODO TESTE — Timeout em %lus",tr);
        c+="<div class='al'>"+String(tBuf)+"</div>";
        c+=F("<form method='POST' action='/controle'>"
             "<input type='hidden' name='acao' value='sair_teste'>"
             "<button type='submit' class='btn-r'>&#9209; Sair do Teste</button></form>");

        auto btnTeste=[&](const char* ico,const char* nome,
                          const char* acOn,const char* acOff,bool on){
            String cor=on?"#43a047":"#546e7a";
            c+="<div class='cg' style='border-color:"+cor+";margin-top:8px'>"
               "<div class='cr'>"
               "<span class='cn'>"+String(ico)+" "+String(nome)+"</span>"
               "<span class='cs' style='color:"+cor+"'>"+(on?"ON":"OFF")+"</span></div>"
               "<div class='grid2' style='gap:6px;margin-top:8px'>"
               "<form method='POST' action='/controle' style='margin:0'>"
               "<input type='hidden' name='acao' value='"+String(acOn)+"'>"
               "<button type='submit' class='btn-g btn-sm'>&#9650; LIGAR</button></form>"
               "<form method='POST' action='/controle' style='margin:0'>"
               "<input type='hidden' name='acao' value='"+String(acOff)+"'>"
               "<button type='submit' class='btn-r btn-sm'>&#9660; DESLIGAR</button></form>"
               "</div></div>";
        };
        btnTeste("&#9889;","Inv. Alta (GPIO27)","teste_a_on","teste_a_off",estados.invAlta);
        btnTeste("&#128161;","Lampada (GPIO12)","teste_lamp_on","teste_lamp_off",estados.lampExt);
    } else {
        c+=F("<div class='nf' style='font-size:11px'>"
             "&#9888; Monitor automatico suspenso durante o teste.</div>"
             "<form method='POST' action='/controle'>"
             "<input type='hidden' name='acao' value='entrar_teste'>"
             "<button type='submit' class='btn-o'>&#9654; Ativar Modo Teste (60s)</button></form>");
    }

    // ── JAVASCRIPT ───────────────────────────────────────────
    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function sBc(id,c){var e=document.getElementById(id);if(e)e.style.borderColor=c;}"
    "function atualizarCtrl(d){"
    "var cA=d.inva?'#fdd835':'#546e7a';"
    "sBc('ctrl_card_a',cA);"
    "sT('ctrl_st_a','<span style=\"color:'+cA+'\">'+(d.inva?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "sT('ctrl_sub_a',d.nome_alta+' | '+d.tempo_a+'s | F:'+d.fal_a);"
    "var cL=d.le?'#fdd835':'#546e7a';"
    "sBc('ctrl_card_lamp',cL);"
    "sT('ctrl_lamp','<span style=\"color:'+cL+'\">'+(d.le?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "var soc=d.soc;"
    "var cor=soc>=50?'#2e7d32':soc>=25?'#ef6c00':'#c62828';"
    "var bv=document.getElementById('ctrl_soc_bv');"
    "if(bv){bv.style.width=Math.min(100,soc)+'%';bv.style.background=cor;bv.textContent=Math.round(soc)+'%';}"
    "sT('ctrl_soc_info',d.bms_valido?'SOC: '+soc.toFixed(0)+'% | Ciclos: '+d.ciclos:'BMS desconectada');"
    "sT('ctrl_soc_live',soc.toFixed(0));"
    "sT('ctrl_lim_l_live',d.lim_lamp.toFixed(0));}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarCtrl(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Controle",c));
}

// ═══════════════════════════════════════════════════════════
// [SENS-WEB]
// ═══════════════════════════════════════════════════════════
void handleSensores() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        String acao=server.arg("acao");
        if (acao=="calib_painel") {
            float iReal=server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>50) msg="<div class='er'>0-50A</div>";
            else {
                int novoOff=(int)((float)sensores.sinalPainel_mV+iReal*calibPainel.sensMvA);
                calibPainel.offsetMV=novoOff;
                prefs.begin("rrfs_cal",false); prefs.putInt("pn_off",novoOff); prefs.end();
                adicionarLog("Calib painel OK");
                msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao=="reset_calib") {
            prefs.begin("rrfs_cal",false);
            prefs.remove("pn_off"); prefs.remove("pn_sens");
            prefs.end(); carregarCalibSensores();
            adicionarLog("Calib reset");
            msg="<div class='ok'>&#10003; Resetado</div>";
        }
    }
    String c=msg;
    c+=F("<div class='nf' style='font-size:11px'>"
         "Corrente do banco via BMS BLE × nP | "
         "Casa = Painel - Banco</div>");

    c+=F("<h3>&#128270; Leituras ao Vivo</h3>"
         "<div class='sc'><div class='lbl'>&#9728; Painel Solar (GPIO34)</div>"
         "<div id='sv_p_corr' class='val'>--</div>"
         "<div id='sv_p_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div id='sv_p_sub' class='sub'>--</div></div>"
         "<div class='bms-card'>"
         "<div class='bms-title'>&#128267; Banco Baterias (BMS BLE)</div>"
         "<div class='grid3'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>I banco</div>"
         "<div id='sv_b_corr' class='bms-val'>--</div>"
         "<div id='sv_b_dir' class='bms-sub'>--</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>V banco</div>"
         "<div id='sv_b_v' class='bms-val'>--</div>"
         "<div class='bms-sub'>V</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>Capacidade</div>"
         "<div id='sv_b_cap' class='bms-val'>--</div>"
         "<div class='bms-sub'>Ah</div></div>"
         "</div>"
         "<div class='grid3' style='margin-top:6px'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>SOC</div>"
         "<div id='sv_b_soc' class='bms-val'>--</div>"
         "<div class='bms-sub'>%</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>Ciclos</div>"
         "<div id='sv_b_ciclos' class='bms-val'>--</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>BLE</div>"
         "<div id='sv_b_ble' class='bms-val' style='font-size:12px'>--</div></div>"
         "</div></div>"
         "<div class='sc'><div class='lbl'>&#127968; Casa (calculada)</div>"
         "<div id='sv_c_corr' class='val'>--</div>"
         "<div id='sv_c_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div class='sub'>= Painel - Banco</div></div>");

    c+=F("<h3>&#9728; Calibrar Painel</h3>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='calib_painel'>"
         "<label>Corrente real no painel (A):</label>"
         "<input type='number' name='corrente_real' step='0.01' min='0' max='50' required>"
         "<button type='submit' class='btn-g'>&#128190; Calibrar</button></form>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='reset_calib'>"
         "<button type='submit' class='btn-r btn-sm' style='margin-top:8px'>"
         "&#8635; Reset Calibracao</button></form>");

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function atualizarSens(d){"
    "var cP=d.ip>0.5?'#fdd835':'#455a64';"
    "sT('sv_p_corr','<span style=\"color:'+cP+'\">'+d.ip.toFixed(2)+'A</span>');"
    "sT('sv_p_pot',Math.round(d.wp)+'W');");
    char sB[120];
    snprintf(sB,sizeof(sB),
        "sT('sv_p_sub',d.mv_painel+' mV | Off:%d | S:%.1f');",
        calibPainel.offsetMV, calibPainel.sensMvA);
    c+=sB;
    c+=F("var cI=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
         "sT('sv_b_corr','<span style=\"color:'+cI+'\">'+Math.abs(d.ib).toFixed(2)+'A</span>');"
         "var dS=d.ib>0.2?'&#9650; Carg':d.ib<-0.2?'&#9660; Desc':'&#8652; Parado';"
         "sT('sv_b_dir','<span style=\"color:'+cI+'\">'+dS+'</span>');"
         "sT('sv_b_v',d.v_banco.toFixed(2));"
         "sT('sv_b_cap',d.cap_banco.toFixed(2));"
         "var cS=d.soc>=50?'#43a047':d.soc>=25?'#ef6c00':'#c62828';"
         "sT('sv_b_soc','<span style=\"color:'+cS+'\">'+d.soc.toFixed(0)+'</span>');"
         "sT('sv_b_ciclos',d.ciclos);"
         "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#a5d6a7':'#ef9a9a';"
         "sT('sv_b_ble','<span style=\"color:'+cBLE+'\">'+d.ble_str+'</span>');"
         "sT('sv_c_corr',d.ic.toFixed(2)+'A');"
         "sT('sv_c_pot',Math.round(d.wc)+'W');}"
         "function aD(){var x=new XMLHttpRequest();"
         "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
         "try{atualizarSens(JSON.parse(x.responseText));}catch(e){}}};"
         "x.open('GET','/dados',true);x.send();}"
         "setInterval(aD,2000);aD();"
         "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Sensores",c));
}

// ═══════════════════════════════════════════════════════════
// [BAT-WEB] — Mostra dados BMS + Banco v23
// ═══════════════════════════════════════════════════════════
void handleBateria() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String c = "";

    c+=F("<div class='nf' style='font-size:11px'>"
         "&#128268; Todos os dados vem da BMS via BLE.<br>"
         "Configure o banco em <a href='/bms' style='color:#90caf9'>BMS Bluetooth</a>.</div>");

    c+=F("<h3>&#128267; Banco ao Vivo</h3>"
         "<div class='banco-card'>"
         "<div class='banco-title' id='bat_badge'>Banco --</div>"
         "<div style='text-align:center;margin:8px 0'>"
         "<div style='font-size:10px;color:#7b1fa2'>SOC (BMS)</div>"
         "<div id='bat_soc' style='font-size:48px;font-weight:bold'>--%</div>"
         "<div class='bb' style='margin-top:6px'>"
         "<div id='bat_soc_bv' class='bv' style='width:0%'>0%</div></div>"
         "</div>"
         "<div class='grid4' style='margin-top:8px'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>V banco</div>"
         "<div id='bat_v' style='font-size:16px;font-weight:bold;color:#4fc3f7'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>V</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>I banco</div>"
         "<div id='bat_i' style='font-size:16px;font-weight:bold'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>A</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Potencia</div>"
         "<div id='bat_w' style='font-size:16px;font-weight:bold;color:#fdd835'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>W</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Ciclos</div>"
         "<div id='bat_ciclos' style='font-size:16px;font-weight:bold;color:#90a4ae'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>BMS</div></div>"
         "</div>"
         "<div class='grid2' style='margin-top:8px'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>Capacidade</div>"
         "<div id='bat_cap' style='font-size:14px;font-weight:bold;color:#ce93d8'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>Ah</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#546e7a'>BLE Status</div>"
         "<div id='bat_ble' style='font-size:14px;font-weight:bold'>--</div></div>"
         "</div>"
         "<div id='bat_dir' style='text-align:center;margin-top:8px;font-size:13px;font-weight:bold'>--</div>"
         "<div id='bat_tempo2' style='text-align:center;margin-top:4px;font-size:11px'></div>"
         "</div>");

    // BMS unitária
    c+=F("<h3>&#128268; BMS Unitaria (1 string)</h3>"
         "<div class='bms-card'>"
         "<div class='grid3'>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>Corrente</div>"
         "<div id='bat_bms_i' class='bms-val'>--</div>"
         "<div class='bms-sub'>A/string</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>Tensao</div>"
         "<div id='bat_bms_v' class='bms-val'>--</div>"
         "<div class='bms-sub'>V/bat</div></div>"
         "<div style='text-align:center'>"
         "<div style='font-size:9px;color:#7b1fa2'>Capacidade</div>"
         "<div id='bat_bms_cap' class='bms-val'>--</div>"
         "<div class='bms-sub'>Ah/bat</div></div>"
         "</div></div>");

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function atualizarBat(d){"
    "sT('bat_badge','Banco '+d.nS+'S'+d.nP+'P — '+(d.nS*d.nP)+' baterias');"
    "var soc=d.soc;"
    "var cS=soc>=50?'#43a047':soc>=25?'#ef6c00':'#c62828';"
    "sT('bat_soc','<span style=\"color:'+cS+'\">'+soc.toFixed(0)+'%</span>');"
    "var bv=document.getElementById('bat_soc_bv');"
    "if(bv){bv.style.width=Math.min(100,soc)+'%';bv.style.background=cS;"
    "  bv.textContent=soc.toFixed(0)+'%';}"
    "sT('bat_v',d.v_banco.toFixed(2));"
    "var cI=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
    "sT('bat_i','<span style=\"color:'+cI+'\">'+Math.abs(d.ib).toFixed(2)+'</span>');"
    "sT('bat_w',Math.round(Math.abs(d.wb)));"
    "sT('bat_ciclos',d.ciclos);"
    "sT('bat_cap',d.cap_banco.toFixed(2));"
    "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#a5d6a7':'#ef9a9a';"
    "sT('bat_ble','<span style=\"color:'+cBLE+'\">'+d.ble_str+'</span>');"
    "var dS=d.ib>0.2?'&#9650; Carregando':d.ib<-0.2?'&#9660; Descarregando':'&#8652; Parado';"
    "sT('bat_dir','<span style=\"color:'+cI+'\">'+dS+'</span>');"
    "if(d.tempo_val){"
    "  var cT2=d.tempo_carg?'#43a047':'#ef6c00';"
    "  sT('bat_tempo2','<span style=\"color:'+cT2+'\">'+"
    "    (d.tempo_carg?'&#9650; Completa em: ':'&#9660; Esgota em: ')+"
    "    d.tempo_h+'h'+String(d.tempo_m).padStart(2,'0')+'m</span>');"
    "} else sT('bat_tempo2','');"
    "sT('bat_bms_i',d.bms_corr.toFixed(2));"
    "sT('bat_bms_v',d.bms_tensao.toFixed(2));"
    "sT('bat_bms_cap',d.bms_cap.toFixed(2));}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarBat(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Bateria",c));
}

// ═══════════════════════════════════════════════════════════
// [PREV-WEB]
// ═══════════════════════════════════════════════════════════
void handlePrevisao() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        String acao=server.arg("acao");
        if (acao=="config_prev") {
            float lat=server.arg("lat").toFloat(),lon=server.arg("lon").toFloat();
            float pkw=server.arg("painel_kw").toFloat();
            bool ativ=(server.arg("ativa")=="1");
            if (pkw<=0||pkw>100)     msg="<div class='er'>Potencia 0.1-100kW</div>";
            else if (lat<-90||lat>90) msg="<div class='er'>Latitude invalida</div>";
            else if (lon<-180||lon>180) msg="<div class='er'>Longitude invalida</div>";
            else {
                cfgPrevisao.latitude=lat; cfgPrevisao.longitude=lon;
                cfgPrevisao.painelKW=pkw;
                cfgPrevisao.ativa=ativ; cfgPrevisao.configurada=true;
                salvarConfigPrevisao();
                if (ativ&&WiFi.status()==WL_CONNECTED){ previsao.valido=false; tUltimaPrevisao=0; }
                adicionarLog("Prev config salva"); msg="<div class='ok'>&#10003; Configurado!</div>";
            }
        } else if (acao=="buscar_agora") {
            previsao.valido=false; tUltimaPrevisao=0; esp_task_wdt_reset();
            msg=buscarPrevisaoOpenMeteo()
                ?"<div class='ok'>&#10003; Atualizado!</div>"
                :"<div class='er'>&#10007; Falha. Verifique WiFi.</div>";
        } else if (acao=="reset_calib_prev") {
            cfgPrevisao.fatorCalib=1.0f; salvarConfigPrevisao();
            if (previsao.valido) calcularAhPrevisto();
            adicionarLog("Calib prev reset"); msg="<div class='ok'>&#10003; Reset OK</div>";
        }
    }
    String c=msg;
    c+=F("<h3>&#9881; Configurar</h3>"
         "<form method='POST' action='/previsao'>"
         "<input type='hidden' name='acao' value='config_prev'>"
         "<label>Latitude:</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.6f",cfgPrevisao.latitude);
    c+="<input type='number' name='lat' step='0.000001' min='-90' max='90' required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.6f",cfgPrevisao.longitude);
    c+=F("<label>Longitude:</label>");
    c+="<input type='number' name='lon' step='0.000001' min='-180' max='180' required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.2f",cfgPrevisao.painelKW>0?cfgPrevisao.painelKW:1.0f);
    c+=F("<label>Potencia Painel (kW):</label>");
    c+="<input type='number' name='painel_kw' step='0.01' min='0.1' max='100' required value='"+String(tempBuf)+"'>";
    c+=F("<label>Ativar:</label><select name='ativa'>");
    c+=cfgPrevisao.ativa
        ?F("<option value='1' selected>Sim</option><option value='0'>Nao</option>")
        :F("<option value='1'>Sim</option><option value='0' selected>Nao</option>");
    c+=F("</select>");
    char bFator[48]; snprintf(bFator,sizeof(bFator),"Fator calibracao: %.3f | Capacidade BMS: ",cfgPrevisao.fatorCalib);
    c+="<div class='nf' style='font-size:11px'>"+String(bFator);
    if (dadosBMS.capacidadeAh > 0) {
        char capStr[32];
        snprintf(capStr, sizeof(capStr), "%.2f Ah (x%d paralelo = %.2f Ah banco)",
                 dadosBMS.capacidadeAh, banco.nP, dadosBMS.capacidadeAh * banco.nP);
        c += String(capStr);
    } else {
        c += "Aguardando BMS...";
    }
    c+="</div>";
    c+=F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form><h3>&#127774; Status</h3>");
    if (!cfgPrevisao.configurada) c+=F("<div class='wn'>Nao configurada</div>");
    else if (!cfgPrevisao.ativa)  c+=F("<div class='wn'>Desativada</div>");
    else if (!previsao.valido) {
        c+=F("<div class='wn'>Aguardando dados...</div>"
             "<form method='POST' action='/previsao'><input type='hidden' name='acao' value='buscar_agora'>"
             "<button type='submit' class='btn-o btn-sm'>&#8635; Buscar Agora</button></form>");
    } else {
        c+=F("<form method='POST' action='/previsao'><input type='hidden' name='acao' value='buscar_agora'>"
             "<button type='submit' class='btn-o btn-sm'>&#8635; Atualizar</button></form>"
             "<form method='POST' action='/previsao' style='margin-top:4px'>"
             "<input type='hidden' name='acao' value='reset_calib_prev'>"
             "<button type='submit' class='btn-s btn-sm'>&#8635; Reset Calib</button></form>"
             "<h3>&#9200; Previsao</h3>"
             "<div class='grid2'>"
             "<div class='pd'><div class='dt'>Esta hora</div><div id='pv_h' class='dv'>--</div></div>"
             "<div class='pd'><div class='dt'>Restante hoje</div><div id='pv_r' class='dv'>--</div>"
             "<div class='ds'>Ate meia-noite</div></div></div>"
             "<h3>&#128197; 7 Dias</h3>");
        struct tm tA; bool tT=obterTempoAtual(tA);
        float mx=0;
        for (int x=0;x<PREV_DIAS;x++) if(previsao.ahDia[x]>mx) mx=previsao.ahDia[x];
        for (int d=0;d<PREV_DIAS;d++) {
            String nd;
            if(d==0)nd="Hoje"; else if(d==1)nd="Amanha";
            else if(tT){struct tm td=tA;td.tm_mday+=d;mktime(&td);char db[12];snprintf(db,sizeof(db),"%02d/%02d",td.tm_mday,td.tm_mon+1);nd=String(db);}
            else nd="Dia+"+String(d);
            char vB[20]; snprintf(vB,sizeof(vB),"%.1f Ah",previsao.ahDia[d]);
            float pt=mx>0?(previsao.ahDia[d]/mx)*100.0f:0;
            c+="<div style='margin:4px 0'>"
               "<div style='display:flex;justify-content:space-between;font-size:11px;margin-bottom:2px'>"
               "<span style='color:#90a4ae'>"+nd+"</span>"
               "<span style='color:#fdd835;font-weight:bold'>"+String(vB)+"</span></div>"
               "<div class='bb'><div class='bv' style='width:"+String((int)pt)+
               "%;background:"+(d==0?"#0277bd":"#1b5e20")+"'></div></div></div>";
        }
        c+=F("<script>"
             "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
             "function aD(){var x=new XMLHttpRequest();"
             "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
             "try{var d=JSON.parse(x.responseText);"
             "sT('pv_h',d.ah_prev_h.toFixed(2)+' Ah');"
             "sT('pv_r',d.ah_prev_r.toFixed(2)+' Ah');"
             "}catch(e){}}};"
             "x.open('GET','/dados',true);x.send();}"
             "setInterval(aD,10000);aD();</script>");
    }
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Previsao",c));
}

// ═══════════════════════════════════════════════════════════
// [HIST-WEB]
// ═══════════════════════════════════════════════════════════
void handleHistorico() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String c="";
    c+=F("<h3>&#128200; Totais Acumulados</h3><div class='grid2'>");
    auto cT=[&](const char* ico,const char* nm,float ah,const char* cor){
        char buf[20]; snprintf(buf,sizeof(buf),"%.1fAh",ah);
        c+="<div class='sc' style='text-align:center'><div class='lbl'>"+
           String(ico)+" "+String(nm)+"</div>"
           "<div class='val' style='color:"+String(cor)+"'>"+String(buf)+"</div></div>";
    };
    cT("&#9728;","Painel",  contadores.painelAh,  "#fdd835");
    cT("&#127968;","Casa",  contadores.casaAh,    "#4fc3f7");
    cT("&#9650;","Bat+",    contadores.batCargAh,  "#43a047");
    cT("&#9660;","Bat-",    contadores.batDescAh,  "#ef6c00");
    c+=F("</div><h3>&#128200; Ultimos 7 Dias</h3>");
    int maxD=histTotalDias<7?histTotalDias:7;
    if (maxD==0) { c+=F("<div class='nf'>Sem historico</div>"); }
    else {
        c+=F("<div style='overflow-x:auto'><table style='width:100%;border-collapse:collapse;font-size:11px'>"
             "<tr style='color:#546e7a'>"
             "<th style='text-align:left;padding:4px'>Data</th>"
             "<th style='padding:4px'>Painel</th>"
             "<th style='padding:4px'>Casa</th></tr>");
        for (int i=0;i<maxD;i++) {
            RegistroDia dT;
            bool ok=(i==0)?(dT=histDiaAtual,true)
                :carregarDiaFlash(((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,dT);
            if (!ok||!dT.valido) continue;
            char bD[16]; snprintf(bD,sizeof(bD),"%02d/%02d/%04d",dT.dia,dT.mes,dT.ano);
            char buf[10];
            c+="<tr style='border-top:1px solid #222'><td style='padding:4px'>"+String(bD)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalPainelAh);
            c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalCasaAh);
            c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td></tr>";
        }
        c+=F("</table></div>");
    }
    c+=F("<h3>&#128200; Tempo Real (60s)</h3>");
    int total=histSnapFull?HIST_SNAP_PONTOS:histSnapIdx;
    if (total<2) { c+=F("<div class='nf'>Aguardando...</div>"); }
    else {
        float maxC=0.5f;
        for (int i=0;i<total;i++) {
            int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
            if(histCorrPainel[idx]>maxC) maxC=histCorrPainel[idx];
            if(histCorrCasa[idx]>maxC)   maxC=histCorrCasa[idx];
            if(fabsf(histCorrBat[idx])>maxC) maxC=fabsf(histCorrBat[idx]);
        }
        c+="<div style='height:80px;background:#111;border:1px solid #333;border-radius:4px'>"
           "<svg viewBox='0 0 "+String(total)+" 80' preserveAspectRatio='none' style='width:100%;height:100%'>";
        auto py=[&](float v)->float{ return 78.0f-(v/maxC)*76.0f; };
        String pP="M",pC="M",pB="M";
        for (int i=0;i<total;i++) {
            int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
            char pt[20];
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrPainel[idx]));
            if(i>0)pP+="L"; pP+=String(pt);
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrCasa[idx]));
            if(i>0)pC+="L"; pC+=String(pt);
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(fabsf(histCorrBat[idx])));
            if(i>0)pB+="L"; pB+=String(pt);
        }
        c+="<path d='"+pP+"' fill='none' stroke='#fdd835' stroke-width='2'/>"
           "<path d='"+pC+"' fill='none' stroke='#4fc3f7' stroke-width='2'/>"
           "<path d='"+pB+"' fill='none' stroke='#ce93d8' stroke-width='1' stroke-dasharray='3,2'/>"
           "</svg></div>";
        c+=F("<div style='font-size:10px;color:#546e7a;margin-top:4px'>"
             "<span style='color:#fdd835'>&#9644;</span> Painel "
             "<span style='color:#4fc3f7;margin-left:8px'>&#9644;</span> Casa "
             "<span style='color:#ce93d8;margin-left:8px'>&#9644;</span> Banco</div>");
    }
    c+=F("<script>setInterval(function(){location.reload();},30000);</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Historico",c));
}

// ═══════════════════════════════════════════════════════════
// [CFG-WEB]
// ═══════════════════════════════════════════════════════════
void handleConfig() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        String acao=server.arg("acao");
        if (acao=="senha") {
            prefs.begin("rrfs",true); String usr=prefs.getString("user",""); prefs.end();
            String pA=server.arg("pass_atual"),pN=server.arg("pass_nova"),pC=server.arg("pass_conf");
            if (!verificarLogin(usr,pA))  msg="<div class='er'>Senha atual incorreta</div>";
            else if (pN.length()<4)       msg="<div class='er'>Min 4 chars</div>";
            else if (pN!=pC)              msg="<div class='er'>Senhas nao conferem</div>";
            else { salvarCredenciais(usr,pN); adicionarLog("Senha alterada"); msg="<div class='ok'>&#10003; Alterada!</div>"; }
        } else if (acao=="tempo") {
            int fh=server.arg("fuso_h").toInt(),fm=server.arg("fuso_m").toInt();
            bool ntp=(server.arg("ntp_ativo")=="1");
            if (fh<-12||fh>14) msg="<div class='er'>Fuso -12 a +14</div>";
            else {
                cfgTempo.fusoHoras=fh; cfgTempo.fusoMinutos=fm; cfgTempo.ntpAtivo=ntp;
                if (!ntp) {
                    cfgTempo.manualAnno=server.arg("man_ano").toInt();
                    cfgTempo.manualMes =server.arg("man_mes").toInt();
                    cfgTempo.manualDia =server.arg("man_dia").toInt();
                    cfgTempo.manualHora=server.arg("man_hora").toInt();
                    cfgTempo.manualMin =server.arg("man_min").toInt();
                }
                salvarConfigTempo(); adicionarLog("Tempo salvo");
                if (ntp&&WiFi.status()==WL_CONNECTED){ ntpSincronizado=false; sincronizarNTP(); }
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao=="reset_historico") {
            if (server.arg("confirmar")!="SIM") msg="<div class='er'>Confirmacao invalida</div>";
            else {
                contadores={0,0,0,0}; salvarContadores();
                memset(&acum,0,sizeof(AcumHora)); acum.horaAtual=255; acum.ultimoAcum=0;
                RegistroDia dV; memset(&dV,0,sizeof(RegistroDia));
                for (uint16_t i=0;i<HIST_DIAS;i++) salvarDiaFlash(i,dV);
                histIndiceAtual=0; histTotalDias=0;
                memset(&histDiaAtual,0,sizeof(RegistroDia)); salvarIndiceHistorico();
                memset(histCorrPainel,0,sizeof(histCorrPainel));
                memset(histCorrCasa,  0,sizeof(histCorrCasa));
                memset(histCorrBat,   0,sizeof(histCorrBat));
                histSnapIdx=0; histSnapFull=false;
                adicionarLog("HIST apagado"); msg="<div class='ok'>&#10003; Zerado!</div>";
            }
        }
    }
    String c=msg;
    c+=F("<h3>&#128202; Sistema</h3>"
         "<div class='sc'>"
         "<div id='cfg_sys' class='val' style='font-size:12px'>Carregando...</div>"
         "<div id='cfg_hora' class='sub'></div>"
         "</div>");

    c+=F("<h3>&#128336; Data e Hora</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='tempo'>"
         "<label>Fuso (h):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.fusoHoras);
    c+="<input type='number' name='fuso_h' min='-12' max='14' step='1' required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.fusoMinutos);
    c+=F("<label>Minutos fuso:</label>");
    c+="<input type='number' name='fuso_m' min='0' max='59' step='1' value='"+String(tempBuf)+"'>";
    c+=F("<label>NTP:</label><select name='ntp_ativo' id='ntpSel' onchange='toggleNTP()'>");
    c+=cfgTempo.ntpAtivo
        ?F("<option value='1' selected>Sim</option><option value='0'>Nao-Manual</option>")
        :F("<option value='1'>Sim</option><option value='0' selected>Nao-Manual</option>");
    c+=F("</select><div id='manualTime' style='display:");
    c+=cfgTempo.ntpAtivo?"none":"block"; c+=F("'>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualAnno);
    c+=F("<label>Ano:</label>"); c+="<input type='number' name='man_ano' min='2024' max='2099' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualMes);
    c+=F("<label>Mes:</label>"); c+="<input type='number' name='man_mes' min='1' max='12' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualDia);
    c+=F("<label>Dia:</label>"); c+="<input type='number' name='man_dia' min='1' max='31' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualHora);
    c+=F("<label>Hora:</label>"); c+="<input type='number' name='man_hora' min='0' max='23' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualMin);
    c+=F("<label>Minuto:</label>"); c+="<input type='number' name='man_min' min='0' max='59' value='"+String(tempBuf)+"'>";
    c+=F("</div><button type='submit' class='btn-g'>&#128190; Salvar Hora</button></form>"
         "<script>function toggleNTP(){"
         "document.getElementById('manualTime').style.display="
         "document.getElementById('ntpSel').value=='1'?'none':'block';}</script>"
         "<h3>&#128274; Senha</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='senha'>"
         "<label>Atual:</label><input type='password' name='pass_atual' required>"
         "<label>Nova:</label><input type='password' name='pass_nova' required>"
         "<label>Confirmar:</label><input type='password' name='pass_conf' required>"
         "<button type='submit' class='btn-o'>Alterar</button></form>"
         "<h3>&#128465; Limpar Historico</h3>"
         "<form method='POST' action='/config' onsubmit='return confirm(\"Apagar?\")'>"
         "<input type='hidden' name='acao' value='reset_historico'>"
         "<input type='hidden' name='confirmar' value='SIM'>"
         "<button type='submit' class='btn-r'>Apagar</button></form>"
         "<h3>&#9881; Acoes</h3>"
         "<form method='GET' action='/logs-salvos'>"
         "<button type='submit' class='btn-s btn-sm'>&#128221; Logs</button></form>"
         "<form method='GET' action='/bms'>"
         "<button type='submit' class='btn-b btn-sm' style='margin-top:6px'>&#128268; BMS Bluetooth</button></form>"
         "<form method='GET' action='/setup-wifi'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>&#128246; WiFi</button></form>"
         "<form method='GET' action='/ota'>"
         "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>&#128640; OTA</button></form>"
         "<form method='GET' action='/logout'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>&#128275; Logout</button></form>"
         "<form method='POST' action='/reiniciar'>"
         "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>&#8635; Reiniciar</button></form>");
    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{var d=JSON.parse(x.responseText);"
    "sT('cfg_sys','v23 | RAM '+d.ram.toFixed(1)+'KB | Up '+d.uptime+'s | Banco '+d.nS+'S'+d.nP+'P | Cap '+d.cap_banco.toFixed(1)+'Ah"
    "+(d.modo_teste?' | <span style=\"color:#ef6c00\">MODO TESTE</span>':''));"
    "sT('cfg_hora','&#128336; '+d.data_hora+(d.ntp?' <span style=\"color:#43a047\">NTP OK</span>':''));"
    "}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,5000);aD();"
    "</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Config",c));
}

// ═══════════════════════════════════════════════════════════
// [LOGS]
// ═══════════════════════════════════════════════════════════
void handleLogs() {
    if (!logsCarregados) carregarLogsPersistentes();
    String json="["; bool primeiro=true;
    uint16_t inicio=totalLogs>50?totalLogs-50:0;
    for (uint16_t num=inicio;num<totalLogs;num++) {
        uint8_t idx=num%MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        if (!primeiro) json+=",";
        primeiro=false; json+="\"";
        for (int ch=0;logs[idx][ch]&&ch<LOG_MAX_LEN;ch++) {
            char cc=logs[idx][ch];
            if(cc=='"')      json+="\\\"";
            else if(cc=='\\')json+="\\\\";
            else if(cc=='\n')json+=" ";
            else if(cc=='\r')continue;
            else json+=cc;
        }
        json+="\"";
    }
    json+="]";
    server.send(200,"application/json",json);
}

void handleLogsSalvos() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    if (!logsCarregados) carregarLogsPersistentes();
    String c=F("<h3>&#128221; Logs Salvos</h3>");
    c+="<div class='nf'>Total: <b>"+String(totalLogs)+"</b></div>";
    c+=F("<div class='lbx' style='height:400px'>");
    uint16_t inicio=totalLogs>MAX_LOGS?totalLogs-MAX_LOGS:0;
    for (uint16_t num=inicio;num<totalLogs;num++) {
        uint8_t idx=num%MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        c+="<div class='ll'>"+String(logs[idx])+"</div>";
    }
    c+=F("</div><form method='GET' action='/config'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:8px'>&#8592; Voltar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Logs",c));
}

// ═══════════════════════════════════════════════════════════
// [UTIL]
// ═══════════════════════════════════════════════════════════
void handleReiniciar() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    adicionarLog("Reinicio usuario");
    server.send(200,"text/html;charset=utf-8",
        gerarPagina("Reiniciando",F("<div class='ok'>&#10003; Reiniciando em 3s...</div>"),false));
    delay(3000); ESP.restart();
}

void handleNotFound() {
    String uri=server.uri();
    if (uri.indexOf("generate_204")>=0||uri.indexOf("gen_204")>=0||
        uri.indexOf("hotspot")>=0||uri.indexOf("ncsi.txt")>=0||
        uri.indexOf("connecttest")>=0) {
        if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302,"text/plain",""); }
        else server.send(204,"text/plain","");
        return;
    }
    if (uri=="/favicon.ico") { server.send(204); return; }
    if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else server.send(404,"text/plain","404");
}

// ═══════════════════════════════════════════════════════════
// [OTA]
// ═══════════════════════════════════════════════════════════
void handleOTA() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String c=F("<h3>&#128640; Atualizar Firmware</h3>"
               "<div class='nf'>Selecione o arquivo .bin para ESP32</div>"
               "<form method='POST' action='/ota-upload' enctype='multipart/form-data' id='fOTA'>"
               "<input type='file' id='fArq' name='firmware' accept='.bin' required "
               "style='background:#111;border:1px solid #444;border-radius:4px;padding:8px;"
               "color:#e0e0e0;width:100%;box-sizing:border-box'>"
               "<div id='prog' style='display:none;margin-top:8px'>"
               "<div class='bb'><div class='bv' id='pBar' style='width:0%;background:#0277bd'>0%</div></div>"
               "<div style='text-align:center;font-size:12px;color:#78909c;margin-top:4px' id='pTxt'>Enviando...</div></div>"
               "<button type='submit' class='btn-o' style='margin-top:10px' id='btnOTA'>&#128640; Enviar</button></form>"
               "<form method='GET' action='/config'>"
               "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>&#8592; Voltar</button></form>"
               "<script>"
               "document.getElementById('fOTA').onsubmit=function(e){"
               "e.preventDefault();"
               "var f=document.getElementById('fArq').files[0];"
               "if(!f||!f.name.endsWith('.bin')){alert('Selecione .bin');return;}"
               "var fd=new FormData();fd.append('firmware',f);"
               "var x=new XMLHttpRequest();"
               "document.getElementById('prog').style.display='block';"
               "document.getElementById('btnOTA').disabled=true;"
               "x.upload.onprogress=function(e){if(e.lengthComputable){"
               "var p=Math.round(e.loaded/e.total*100);"
               "document.getElementById('pBar').style.width=p+'%';"
               "document.getElementById('pBar').textContent=p+'%';"
               "if(p>=100)document.getElementById('pTxt').textContent='Gravando...';}};"
               "x.onreadystatechange=function(){if(x.readyState==4){"
               "if(x.status==200||x.status==0){"
               "document.getElementById('pBar').style.background='#2e7d32';"
               "document.getElementById('pBar').style.width='100%';"
               "document.getElementById('pBar').textContent='100%';"
               "document.getElementById('pTxt').textContent='&#10003; Gravado! Reiniciando...';"
               "document.getElementById('pTxt').style.color='#a5d6a7';"
               "setTimeout(function(){location.href='/';},6000);"
               "}else{"
               "document.getElementById('pTxt').textContent='Erro '+x.status;"
               "document.getElementById('pBar').style.background='#c62828';"
               "document.getElementById('btnOTA').disabled=false;}}};"
               "x.open('POST','/ota-upload',true);x.send(fd);};"
               "</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - OTA",c));
}

void handleOTAUpload() {
    if (!logado) { server.send(403); return; }
    HTTPUpload& up=server.upload();
    if (up.status==UPLOAD_FILE_START) {
        adicionarLog("OTA inicio");
        esp_task_wdt_reset();
        desligarInvAlta();
        desconectarBMS();
        modoTeste=true; tInicioTeste=millis(); TIMEOUT_TESTE_MS=600000UL;
        esp_task_wdt_reset();
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            adicionarLog("OTA ERRO begin");
            modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
        }
    } else if (up.status==UPLOAD_FILE_WRITE) {
        esp_task_wdt_reset();
        if (Update.write(up.buf,up.currentSize)!=up.currentSize)
            adicionarLog("OTA ERRO write");
    } else if (up.status==UPLOAD_FILE_END) {
        esp_task_wdt_reset();
        if (Update.end(true)) {
            adicionarLog("OTA OK");
            server.send(200,"text/plain","OK");
            delay(1500); ESP.restart();
        } else {
            adicionarLog("OTA ERRO end");
            modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
            server.send(500,"text/plain","ERRO");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [ROTAS]
// ═══════════════════════════════════════════════════════════
void configurarRotas() {
    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/dados",       HTTP_GET,  handleDados);
    server.on("/bateria",     HTTP_ANY,  handleBateria);
    server.on("/controle",    HTTP_ANY,  handleControle);
    server.on("/sensores",    HTTP_ANY,  handleSensores);
    server.on("/bms",         HTTP_ANY,  handleBMS);
    server.on("/config",      HTTP_ANY,  handleConfig);
    server.on("/historico",   HTTP_ANY,  handleHistorico);
    server.on("/previsao",    HTTP_ANY,  handlePrevisao);
    server.on("/logs",        HTTP_GET,  handleLogs);
    server.on("/logs-salvos", HTTP_GET,  handleLogsSalvos);
    server.on("/login",       HTTP_ANY,  handleLogin);
    server.on("/logout",      HTTP_GET,  handleLogout);
    server.on("/setup-login", HTTP_ANY,  handleSetupLogin);
    server.on("/setup-wifi",  HTTP_ANY,  handleSetupWifi);
    server.on("/reiniciar",   HTTP_POST, handleReiniciar);
    server.on("/ota",         HTTP_GET,  handleOTA);
    server.on("/ota-upload",  HTTP_POST, [](){}, handleOTAUpload);
    server.on("/bms-scan",    HTTP_GET,  handleBMSScan);
    server.on("/bms-lista",   HTTP_GET,  handleBMSLista);
    server.onNotFound(handleNotFound);
    server.begin();
    adicionarLog("Rotas OK v23 CORRIGIDO");
}
