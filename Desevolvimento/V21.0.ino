/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v21 - DUAL INVERSOR + PREVISÃO SOLAR
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 * MUDANÇAS v21:
 * - ADICIONADO: Monitoramento de status dos inversores
 * - ADICIONADO: PIN_STATUS_INV_BAIXA (GPIO33) sinal >2V = ON
 * - ADICIONADO: PIN_STATUS_INV_ALTA  (GPIO36) sinal >2V = ON
 * - ADICIONADO: Média móvel no sinal de status (anti-flutuação)
 * - ADICIONADO: Confirmação 5s para detectar falha real
 * - ADICIONADO: Auto-religue: desliga 5s → religa → aguarda 5s
 * - ADICIONADO: Inv.Alta só monitora status se sinal casa ativo
 * - ADICIONADO: Contadores de falhas e religamentos por inversor
 * - ADICIONADO: Status de religue na dashboard
 * - MANTIDO:    Toda lógica v20
 * ════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════
// [INC] INCLUDES
// ═══════════════════════════════════════════════════════════
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <Update.h>
#include <HTTPClient.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════
// [HW] PINOS
// ═══════════════════════════════════════════════════════════
#define LED_AZUL                2
#define BOTAO_BOOT              0
// Sensores
#define PIN_SENSOR_PAINEL      34   // sensor corrente painel  // Azul claro
#define PIN_SENSOR_CASA        35   // sensor corrente casa    // Azul escuro
// Sinal
#define PIN_SINAL_CASA         32   // sinal alta potência casa  // Marom claro
#define PIN_STATUS_INV_BAIXA   33   // status inversor baixa (>2V = ON) // Marom escuro
#define PIN_STATUS_INV_ALTA    36   // status inversor alta  (>2V = ON)  // Verde Claro
// Reles
#define PIN_RELE_INV_ALTA      27   // relé inversor alta  // Verde escuro
#define PIN_RELE_INV_BAIXA     25   // relé inversor baixa // Laranja claro
#define PIN_RELE_LAMP_EXT      26   // relé lâmpada externa // Laranja claro


// ═══════════════════════════════════════════════════════════
// [CFG] CONSTANTES
// ═══════════════════════════════════════════════════════════
#define DNS_PORT              53
#define TEMPO_RESET_MS     10000
#define MAX_LOGS             100
#define LOG_MAX_LEN           64
#define LOG_NVS_NS       "rrfs_log"

// ADC
#define ADC_SAMPLES          150
#define ADC_VREF            3300
#define ADC_BITS            4095

// Sensores corrente
#define PAINEL_OFFSET_MV    2500
#define PAINEL_SENS_MV_A      40
#define CASA_OFFSET_MV       600
#define CASA_SENS_MV_A        20
#define FILTRO_RUIDO_A       0.3f
#define BUFFER_CORRENTE        10

// Sinal casa (alta potência)
#define SINAL_CASA_LIMIAR_MV  2000
#define SINAL_CASA_MM_SIZE      10
#define SINAL_CONFIRMA_MS     5000UL

// Status inversores
#define STATUS_INV_LIMIAR_MV  2000   // >2V = inversor confirmado ON
#define STATUS_INV_MM_SIZE      10   // amostras média móvel
#define STATUS_FALHA_MS       5000UL // tempo p/ confirmar falha
#define STATUS_DESLIGA_MS     5000UL // tempo desligado antes de religar
#define STATUS_RELIGAR_MS     5000UL // tempo aguardando após religar
#define STATUS_MAX_TENTATIVAS    3   // máx tentativas de religue

// Intervalos
#define INTERVALO_LEITURA_MS    1000
#define INTERVALO_CONTROLE_MS   5000
#define INTERVALO_NTP_MS     3600000UL
#define INTERVALO_PREVISAO_MS 3600000UL
#define LAMP_EXT_DELAY_MS    300000UL

// WiFi
#define WIFI_RECON_INTERVALO  30000UL
#define WIFI_RECON_TIMEOUT    10000UL
#define WIFI_CHECK_INTERVALO  15000UL
#define WIFI_MAX_FALHAS            5

// Histórico
#define HIST_DIAS             30
#define HIST_HORAS            24
#define HIST_KEY_VER          10   // v21
#define HIST_SNAP_PONTOS      60

// Previsão
#define PREV_DIAS              7
#define PREV_TOTAL_HORAS     168

// NTP
#define NTP_SERVER1      "pool.ntp.org"
#define NTP_SERVER2      "time.google.com"

// WiFi AP
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ═══════════════════════════════════════════════════════════
// [STR] STRUCTS
// ═══════════════════════════════════════════════════════════
struct ConfigBateria {
    float tensaoNominal;
    float capacidadeAh;
    float descargaSegura;
    float ahDisponivel;
    float ahAtual;
    float percentualAtual;
    bool  calibrada;
};

struct ConfigLimiares {
    float pctInvBaixa;
    float pctInvAlta;
    float pctLampExt;
};

struct CalibSensor {
    int   offsetMV;
    float sensMvA;
    bool  calibrado;
};

struct BufferMM {
    float   amostras[BUFFER_CORRENTE];
    uint8_t indice;
    uint8_t total;
};

struct DadosSensores {
    float correntePainel;
    float correnteCasa;
    float correnteBateria;
    float potenciaPainel;
    float potenciaCasa;
    float potenciaBateria;
    int   sinalPainel_mV;
    int   sinalCasa_mV;
};

struct EstadoReles {
    bool lampExt;
    bool invAlta;
    bool invBaixa;
};

struct EstimativaTempo {
    bool  valido;
    int   horas, minutos;
    bool  carregando;
    float taxaA;
};

struct EstadoLampExt {
    bool          ligada;
    unsigned long tUltimaMudanca;
    bool          estadoAnteriorPainel;
};

// ── Sinal Casa Alta Potência ─────────────────────────────────
struct SinalCasa {
    float         amostras[SINAL_CASA_MM_SIZE];
    uint8_t       indice;
    uint8_t       total;
    float         mediaMV;
    bool          ativo;
    bool          aguardandoLigar;
    bool          aguardandoDesligar;
    unsigned long tInicioConfirma;
    bool          semEnergia;
};

// ── Status Inversor (monitoramento + religue automático) ─────
// Máquina de estados:
//   NORMAL      → inversor ligado e status OK
//   FALHA_DET   → sinal sumiu, aguardando 5s p/ confirmar
//   DESLIGANDO  → falha confirmada, desligou, aguardando 5s
//   RELIGANDO   → religou, aguardando 5s p/ ver status
//   FALHOU      → não voltou após tentativas máximas
enum EstadoMonitor {
    MON_NORMAL = 0,
    MON_FALHA_DET,
    MON_DESLIGANDO,
    MON_RELIGANDO,
    MON_FALHOU
};

struct MonitorInversor {
    // Média móvel do sinal ADC (mV)
    float         amostras[STATUS_INV_MM_SIZE];
    uint8_t       indice;
    uint8_t       total;
    float         mediaMV;          // tensão média atual

    // Estado da máquina
    EstadoMonitor estado;
    unsigned long tInicioEstado;    // quando entrou no estado atual

    // Contadores
    uint8_t       tentativas;       // tentativas de religue
    uint16_t      totalFalhas;      // falhas históricas
    uint16_t      totalReligues;    // religues bem-sucedidos

    // Flags para dashboard
    bool          statusOK;         // sinal confirmado presente
    bool          emReligue;        // está tentando religar
    bool          falhaDefinitiva;  // esgotou tentativas
};

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

// ── NTP / Tempo ──────────────────────────────────────────────
struct ConfigTempo {
    int  fusoHoras;
    int  fusoMinutos;
    bool ntpAtivo;
    bool sincronizado;
    int  manualAnno;
    int  manualMes;
    int  manualDia;
    int  manualHora;
    int  manualMin;
};

// ── Previsão Solar ───────────────────────────────────────────
struct ConfigPrevisao {
    bool  ativa;
    float latitude;
    float longitude;
    float painelKW;
    float controladorA;
    float fatorCalib;
    bool  configurada;
};

struct DadosPrevisao {
    bool          valido;
    float         radiacao[PREV_TOTAL_HORAS];
    float         ahPrevisto[PREV_TOTAL_HORAS];
    float         ahDia[PREV_DIAS];
    unsigned long tAtualizacao;
    int           horaBase, diaBase, mesBase, anoBase;
    float         ahRealHoraAtual;
    float         ahPrevHoraAtual;
    int           horaAtualIdx;
};

// ═══════════════════════════════════════════════════════════
// [VAR] VARIÁVEIS GLOBAIS
// ═══════════════════════════════════════════════════════════
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP  = false;
bool logado  = false;

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

ConfigBateria  bateria  = {12.8f, 100.0f, 80.0f, 80.0f, 0.0f, 0.0f, false};
ConfigLimiares limiares = {20.0f, 40.0f, 30.0f};

DadosSensores  sensores    = {0};
CalibSensor    calibPainel, calibCasa;
BufferMM       bufPainel   = {0}, bufCasa = {0};

EstadoReles   estados = {false, false, false};
EstadoLampExt lampExt = {false, 0, true};

// Sinal casa alta potência
SinalCasa sinalCasa = {{0}, 0, 0, 0.0f, false, false, false, 0, false};

// Monitores de status dos inversores
MonitorInversor monInvBaixa = {
    {0}, 0, 0, 0.0f,
    MON_NORMAL, 0,
    0, 0, 0,
    false, false, false
};
MonitorInversor monInvAlta = {
    {0}, 0, 0, 0.0f,
    MON_NORMAL, 0,
    0, 0, 0,
    false, false, false
};

unsigned long ultimaLeitura  = 0;
unsigned long ultimoControle = 0;
unsigned long ultimoContagem = 0;

// NTP
ConfigTempo   cfgTempo    = {-3, 0, false, false, 2024, 1, 1, 0, 0};
unsigned long tUltimoNTP  = 0;
bool          ntpSincronizado = false;

// Previsão
ConfigPrevisao cfgPrevisao    = {false, 0.0f, 0.0f, 1.0f, 30.0f, 1.0f, false};
DadosPrevisao  previsao       = {false};
unsigned long  tUltimaPrevisao = 0;

// WiFi
String        gWifiSSID, gWifiPASS, gWifiIP, gWifiGW, gWifiMASK, gWifiDNS;
unsigned long tUltimoCheckWifi = 0;
unsigned long tUltimoReconWifi = 0;
uint8_t       wifiFalhasConsec = 0;
bool          wifiReconectando = false;
unsigned long tInicioRecon     = 0;

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

uint8_t  diaCache = 0;
uint8_t  mesCache = 0;
uint16_t anoCache = 0;
unsigned long ultimaAtualizacaoDia = 0;

// ═══════════════════════════════════════════════════════════
// [PROTO] PROTÓTIPOS
// ═══════════════════════════════════════════════════════════
EstimativaTempo calcularTempoRestante();
String tempoAteCorte(float pctCorte, float taxaA);
inline float kW(float amperes);

void adicionarLog(const char* msg);
void logRAM();
void carregarLogsPersistentes();
void salvarLogNVS(uint8_t pos, const char* msg);

void setupLED();
void atualizarLED();
void ledBlink();
void ledFixo(bool v);
void setupBotao();
void verificarReset();

void setupReles();
void ligarLampExt();    void desligarLampExt();
void ligarInvAlta();    void desligarInvAlta();
void ligarInvBaixa();   void desligarInvBaixa();

int   lerADC(uint8_t pin);
int   adcParaMv(int v);
float adicionarAmostra(BufferMM& buf, float nova);
float lerCorrentePainel();
float lerCorrenteCasa();
void  calcularPotencias();
void  lerSensores();

void lerSinalCasa();
void processarSinalCasa();
bool sinalCasaAtivo();

void lerStatusInversor(MonitorInversor& mon, uint8_t pin);
void processarMonitorInversor(MonitorInversor& mon,
                               bool deveEstarLigado,
                               const char* nomeInv,
                               void (*fnLigar)(),
                               void (*fnDesligar)());

void calcularBateria();
void verificarCalibracaoAutomatica();
void atualizarAhBateria();

void   sincronizarNTP();
bool   obterTempoAtual(struct tm& t);
String formatarDataHora();
String formatarHora();
void   salvarConfigTempo();
void   carregarConfigTempo();

void   salvarConfigPrevisao();
void   carregarConfigPrevisao();
bool   buscarPrevisaoOpenMeteo();
void   calcularAhPrevisto();
void   calibrarPrevisao();
float  getAhPrevHoraAtual();
float  getAhPrevRestanteDia();
int    getIndiceHoraAtual();
void   gerenciarPrevisao();

void calcularTotaisDia(RegistroDia& dia);
void salvarContadores();
void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void salvarHistorico();
void carregarHistorico();
void acumularEnergia();
void atualizarDiaAtual();

void controlarInvBaixa();
void controlarInvAlta();
void controlarLampExt();
void controlarEnergia();
void monitorarInversores();

bool existeCredenciais();
bool existeWiFiConfig();
void salvarCredenciais(const String& u, const String& p);
bool verificarLogin(const String& u, const String& p);
void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap);
void carregarWifi(String& ssid, String& pass, String& ip, String& gw,
                  String& mask, String& dns, bool& ap);
void salvarConfigBateria();
void carregarConfigBateria();
void salvarLimiares();
void carregarLimiares();
void carregarCalibSensores();

void iniciarAP();
bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns);
void gerenciarWiFi();

void   configurarRotas();
String gerarPagina(const String& titulo,
                   const String& conteudo, bool comNav = true);
String barraProgresso(float pct, const String& label = "");

void handleRoot();
void handleDados();
void handleBateria();
void handleRedes();
void handleSensores();
void handleConfig();
void handleHistorico();
void handlePrevisao();
void handleLogs();
void handleLogsSalvos();
void handleSetupLogin();
void handleSetupWifi();
void handleLogin();
void handleLogout();
void handleNotFound();
void handleReiniciar();
void handleOTA();
void handleOTAUpload();

// ═══════════════════════════════════════════════════════════
// [UTIL] UTILITÁRIOS
// ═══════════════════════════════════════════════════════════
inline float kW(float amperes) {
    return (amperes * bateria.tensaoNominal) / 1000.0f;
}

EstimativaTempo calcularTempoRestante() {
    EstimativaTempo est = {false, 0, 0, false, 0};
    if (!bateria.calibrada || bateria.ahDisponivel <= 0) return est;
    float taxa = sensores.correnteBateria;
    if (fabsf(taxa) < 0.3f) return est;
    est.taxaA      = taxa;
    est.valido     = true;
    est.carregando = (taxa > 0);
    float h;
    if (est.carregando) {
        float falt = bateria.ahDisponivel - bateria.ahAtual;
        if (falt <= 0) return est;
        h = falt / taxa;
    } else {
        if (bateria.ahAtual <= 0) return est;
        h = bateria.ahAtual / fabsf(taxa);
    }
    if (h > 999) h = 999;
    est.horas   = (int)h;
    est.minutos = (int)((h - est.horas) * 60);
    return est;
}

String tempoAteCorte(float pctCorte, float taxaA) {
    if (!bateria.calibrada || taxaA >= 0 ||
        bateria.ahDisponivel <= 0) return F("--");
    float ahCorte = bateria.ahDisponivel * (pctCorte / 100.0f);
    float ate     = bateria.ahAtual - ahCorte;
    if (ate <= 0) return F("Agora");
    float h = ate / fabsf(taxaA);
    if (h > 999) return F(">999h");
    int hh = (int)h, mm = (int)((h - hh) * 60);
    char buf[24];
    if (hh == 0) snprintf(buf, sizeof(buf), "%dm", mm);
    else         snprintf(buf, sizeof(buf), "%dh%02dm", hh, mm);
    return String(buf);
}

// ═══════════════════════════════════════════════════════════
// [LOG] SISTEMA DE LOGS
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
    totalLogs++;
    char entrada[LOG_MAX_LEN];
    snprintf(entrada, sizeof(entrada), "[%05d] %s", totalLogs, msg);
    strncpy(logs[logIndex], entrada, LOG_MAX_LEN - 1);
    logs[logIndex][LOG_MAX_LEN - 1] = '\0';
    salvarLogNVS(logIndex, logs[logIndex]);
    logIndex = (logIndex + 1) % MAX_LOGS;
}

void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf),
             "RAM %.1fKB | reset %d",
             esp_get_free_heap_size() / 1024.0f,
             (int)esp_reset_reason());
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [LED] INDICADOR
// ═══════════════════════════════════════════════════════════
void setupLED() {
    pinMode(LED_AZUL, OUTPUT);
    digitalWrite(LED_AZUL, LOW);
}

void atualizarLED() {
    if (!blinkAtivo) return;
    if (millis() - ultimoBlink >= 500) {
        ultimoBlink = millis();
        ledState    = !ledState;
        digitalWrite(LED_AZUL, ledState);
    }
}

void ledBlink() { blinkAtivo = true;  ultimoBlink = millis(); }
void ledFixo(bool v) { blinkAtivo = false; digitalWrite(LED_AZUL, v); }

// ═══════════════════════════════════════════════════════════
// [BTN] BOTÃO RESET
// ═══════════════════════════════════════════════════════════
void setupBotao() { pinMode(BOTAO_BOOT, INPUT_PULLUP); }

void verificarReset() {
    bool p = (digitalRead(BOTAO_BOOT) == LOW);
    if (p && !botaoSegurando) {
        botaoSegurando     = true;
        botaoPressionadoEm = millis();
    }
    if (!p) botaoSegurando = false;
    if (botaoSegurando &&
        millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
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
// [RELE] CONTROLE DE RELÉS
// ═══════════════════════════════════════════════════════════
void setupReles() {
    pinMode(PIN_RELE_INV_ALTA,  OUTPUT);
    digitalWrite(PIN_RELE_INV_ALTA,  LOW);

    pinMode(PIN_RELE_INV_BAIXA, OUTPUT);
    digitalWrite(PIN_RELE_INV_BAIXA, LOW);

    pinMode(PIN_RELE_LAMP_EXT,  OUTPUT);
    digitalWrite(PIN_RELE_LAMP_EXT,  LOW);

    estados = {false, false, false};
    adicionarLog("Reles: todos OFF");
}

void ligarInvAlta()    { digitalWrite(PIN_RELE_INV_ALTA,  HIGH); estados.invAlta  = true;  }
void desligarInvAlta() { digitalWrite(PIN_RELE_INV_ALTA,  LOW);  estados.invAlta  = false; }
void ligarInvBaixa()   { digitalWrite(PIN_RELE_INV_BAIXA, HIGH); estados.invBaixa = true;  }
void desligarInvBaixa(){ digitalWrite(PIN_RELE_INV_BAIXA, LOW);  estados.invBaixa = false; }
void ligarLampExt()    { digitalWrite(PIN_RELE_LAMP_EXT,  HIGH); estados.lampExt  = true;  }
void desligarLampExt() { digitalWrite(PIN_RELE_LAMP_EXT,  LOW);  estados.lampExt  = false; }

// ═══════════════════════════════════════════════════════════
// [ADC] LEITURA E MÉDIA MÓVEL
// ═══════════════════════════════════════════════════════════
int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        soma += analogRead(pin);
        if (i % 50 == 49) yield();
    }
    return (int)(soma / ADC_SAMPLES);
}

int adcParaMv(int v) {
    return (int)((long)v * ADC_VREF / ADC_BITS);
}

float adicionarAmostra(BufferMM& buf, float nova) {
    buf.amostras[buf.indice] = nova;
    buf.indice = (buf.indice + 1) % BUFFER_CORRENTE;
    if (buf.total < BUFFER_CORRENTE) buf.total++;
    float soma = 0;
    for (uint8_t i = 0; i < buf.total; i++) soma += buf.amostras[i];
    return soma / buf.total;
}

float lerCorrentePainel() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_PAINEL));
    sensores.sinalPainel_mV = mv;
    float corrBruta = -(float)(mv - calibPainel.offsetMV) / calibPainel.sensMvA;
    if (corrBruta < 0.15f) corrBruta = 0.0f;
    float corrMedia = adicionarAmostra(bufPainel, corrBruta);
    return (corrMedia < 0.15f) ? 0.0f : corrMedia;
}

float lerCorrenteCasa() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_CASA));
    sensores.sinalCasa_mV = mv;
    if (mv <= calibCasa.offsetMV) {
        adicionarAmostra(bufCasa, 0.0f);
        return 0.0f;
    }
    float corr = (float)(mv - calibCasa.offsetMV) / calibCasa.sensMvA;
    if (corr < FILTRO_RUIDO_A) corr = 0.0f;
    float media = adicionarAmostra(bufCasa, corr);
    return (media < FILTRO_RUIDO_A) ? 0.0f : media;
}

void calcularPotencias() {
    float vn = bateria.tensaoNominal;
    sensores.potenciaPainel  = sensores.correntePainel  * vn;
    sensores.potenciaCasa    = sensores.correnteCasa    * vn;
    sensores.potenciaBateria = sensores.correnteBateria * vn;
}

void lerSensores() {
    esp_task_wdt_reset();
    sensores.correntePainel  = lerCorrentePainel();
    esp_task_wdt_reset();
    sensores.correnteCasa    = lerCorrenteCasa();
    esp_task_wdt_reset();
    sensores.correnteBateria = sensores.correntePainel - sensores.correnteCasa;
    calcularPotencias();
    esp_task_wdt_reset();

    atualizarAhBateria();
    verificarCalibracaoAutomatica();

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
// [SINAL] SINAL CASA ALTA POTÊNCIA
// ═══════════════════════════════════════════════════════════
void lerSinalCasa() {
    int raw  = analogRead(PIN_SINAL_CASA);
    float mv = (float)raw * ADC_VREF / ADC_BITS;

    sinalCasa.amostras[sinalCasa.indice] = mv;
    sinalCasa.indice = (sinalCasa.indice + 1) % SINAL_CASA_MM_SIZE;
    if (sinalCasa.total < SINAL_CASA_MM_SIZE) sinalCasa.total++;

    float soma = 0;
    for (uint8_t i = 0; i < sinalCasa.total; i++)
        soma += sinalCasa.amostras[i];
    sinalCasa.mediaMV = soma / sinalCasa.total;
}

void processarSinalCasa() {
    lerSinalCasa();
    unsigned long agora    = millis();
    bool          detecAlt = (sinalCasa.mediaMV >= SINAL_CASA_LIMIAR_MV);

    if (!sinalCasa.ativo) {
        if (detecAlt) {
            if (!sinalCasa.aguardandoLigar) {
                sinalCasa.aguardandoLigar    = true;
                sinalCasa.tInicioConfirma    = agora;
                sinalCasa.aguardandoDesligar = false;
            } else if (agora - sinalCasa.tInicioConfirma >= SINAL_CONFIRMA_MS) {
                sinalCasa.ativo           = true;
                sinalCasa.aguardandoLigar = false;
                adicionarLog("Sinal casa: ATIVO confirmado");
            }
        } else {
            sinalCasa.aguardandoLigar = false;
        }
    } else {
        if (!detecAlt) {
            if (!sinalCasa.aguardandoDesligar) {
                sinalCasa.aguardandoDesligar = true;
                sinalCasa.tInicioConfirma    = agora;
                sinalCasa.aguardandoLigar    = false;
            } else if (agora - sinalCasa.tInicioConfirma >= SINAL_CONFIRMA_MS) {
                sinalCasa.ativo              = false;
                sinalCasa.aguardandoDesligar = false;
                sinalCasa.semEnergia         = false;
                adicionarLog("Sinal casa: INATIVO confirmado");
            }
        } else {
            sinalCasa.aguardandoDesligar = false;
        }
    }
}

bool sinalCasaAtivo() { return sinalCasa.ativo; }

// ═══════════════════════════════════════════════════════════
// [MON] MONITORAMENTO DE STATUS DOS INVERSORES
// ─────────────────────────────────────────────────────────
// Lê o pino de status do inversor com média móvel.
// Só monitora se deveEstarLigado == true.
// Máquina de estados:
//   MON_NORMAL    → status OK, inversor respondendo
//   MON_FALHA_DET → sinal sumiu, aguardando 5s p/ confirmar
//   MON_DESLIGANDO→ falha confirmada: desliga, aguarda 5s
//   MON_RELIGANDO → religou, aguarda 5s p/ ver se voltou
//   MON_FALHOU    → tentativas esgotadas, não religar
// ═══════════════════════════════════════════════════════════
void lerStatusInversor(MonitorInversor& mon, uint8_t pin) {
    int raw  = analogRead(pin);
    float mv = (float)raw * ADC_VREF / ADC_BITS;

    mon.amostras[mon.indice] = mv;
    mon.indice = (mon.indice + 1) % STATUS_INV_MM_SIZE;
    if (mon.total < STATUS_INV_MM_SIZE) mon.total++;

    float soma = 0;
    for (uint8_t i = 0; i < mon.total; i++) soma += mon.amostras[i];
    mon.mediaMV = soma / mon.total;
}

void processarMonitorInversor(MonitorInversor& mon,
                               bool            deveEstarLigado,
                               const char*     nomeInv,
                               void (*fnLigar)(),
                               void (*fnDesligar)()) {
    unsigned long agora     = millis();
    bool          sinalOK   = (mon.mediaMV >= STATUS_INV_LIMIAR_MV);

    // Se não deve estar ligado — reseta monitor e sai
    if (!deveEstarLigado) {
        mon.estado        = MON_NORMAL;
        mon.tentativas    = 0;
        mon.statusOK      = false;
        mon.emReligue     = false;
        mon.falhaDefinitiva = false;
        mon.tInicioEstado = agora;
        return;
    }

    // ── Máquina de estados ────────────────────────────────
    switch (mon.estado) {

        // ── NORMAL: inversor ligado, monitorando sinal ──────
        case MON_NORMAL:
            if (sinalOK) {
                mon.statusOK      = true;
                mon.emReligue     = false;
                mon.falhaDefinitiva = false;
                mon.tentativas    = 0;  // reseta tentativas ao estabilizar
            } else {
                // Sinal sumiu → inicia contagem de confirmação
                mon.estado        = MON_FALHA_DET;
                mon.tInicioEstado = agora;
                mon.statusOK      = false;
            }
            break;

        // ── FALHA_DET: aguardando 5s para confirmar falha ───
        case MON_FALHA_DET:
            if (sinalOK) {
                // Sinal voltou antes dos 5s — era flutuação, volta ao normal
                mon.estado    = MON_NORMAL;
                mon.statusOK  = true;
                mon.emReligue = false;
            } else if (agora - mon.tInicioEstado >= STATUS_FALHA_MS) {
                // Falha confirmada após 5s
                mon.totalFalhas++;
                snprintf(tempBuf, sizeof(tempBuf),
                         "%s falha confirmada #%d",
                         nomeInv, mon.totalFalhas);
                adicionarLog(tempBuf);

                if (mon.tentativas >= STATUS_MAX_TENTATIVAS) {
                    // Esgotou tentativas
                    mon.estado          = MON_FALHOU;
                    mon.falhaDefinitiva = true;
                    mon.emReligue       = false;
                    snprintf(tempBuf, sizeof(tempBuf),
                             "%s falha definitiva", nomeInv);
                    adicionarLog(tempBuf);
                    fnDesligar();
                } else {
                    // Desliga e aguarda 5s antes de religar
                    mon.estado        = MON_DESLIGANDO;
                    mon.tInicioEstado = agora;
                    mon.emReligue     = true;
                    fnDesligar();
                    snprintf(tempBuf, sizeof(tempBuf),
                             "%s desligando p/ religue", nomeInv);
                    adicionarLog(tempBuf);
                }
            }
            break;

        // ── DESLIGANDO: inversor desligado, aguarda 5s ──────
        case MON_DESLIGANDO:
            if (agora - mon.tInicioEstado >= STATUS_DESLIGA_MS) {
                // Passados 5s desligado → religa
                mon.tentativas++;
                mon.estado        = MON_RELIGANDO;
                mon.tInicioEstado = agora;
                fnLigar();
                snprintf(tempBuf, sizeof(tempBuf),
                         "%s religando tent %d/%d",
                         nomeInv, mon.tentativas,
                         STATUS_MAX_TENTATIVAS);
                adicionarLog(tempBuf);
            }
            break;

        // ── RELIGANDO: aguarda 5s para ver se status voltou ─
        case MON_RELIGANDO:
            if (sinalOK) {
                // Status voltou — religue bem-sucedido!
                mon.totalReligues++;
                mon.estado    = MON_NORMAL;
                mon.statusOK  = true;
                mon.emReligue = false;
                snprintf(tempBuf, sizeof(tempBuf),
                         "%s religado OK #%d",
                         nomeInv, mon.totalReligues);
                adicionarLog(tempBuf);
            } else if (agora - mon.tInicioEstado >= STATUS_RELIGAR_MS) {
                // Não voltou em 5s — verifica tentativas
                if (mon.tentativas >= STATUS_MAX_TENTATIVAS) {
                    mon.estado          = MON_FALHOU;
                    mon.falhaDefinitiva = true;
                    mon.emReligue       = false;
                    fnDesligar();
                    snprintf(tempBuf, sizeof(tempBuf),
                             "%s falha definitiva apos %d tentativas",
                             nomeInv, mon.tentativas);
                    adicionarLog(tempBuf);
                } else {
                    // Tenta novamente: desliga e aguarda
                    mon.estado        = MON_DESLIGANDO;
                    mon.tInicioEstado = agora;
                    fnDesligar();
                }
            }
            break;

        // ── FALHOU: não fazer nada — aguarda intervenção ────
        case MON_FALHOU:
            mon.statusOK      = false;
            mon.emReligue     = false;
            mon.falhaDefinitiva = true;
            break;
    }
}

// ═══════════════════════════════════════════════════════════
// [BAT] BATERIA — COULOMB COUNTING
// ═══════════════════════════════════════════════════════════
void calcularBateria() {
    bateria.ahDisponivel =
        bateria.capacidadeAh * (bateria.descargaSegura / 100.0f);
}

void verificarCalibracaoAutomatica() {
    if (sensores.correntePainel  >  0.5f &&
        fabsf(sensores.correnteBateria) <= 0.5f &&
        sensores.correnteCasa    >  0.1f &&
        !bateria.calibrada) {
        bateria.ahAtual         = bateria.ahDisponivel;
        bateria.percentualAtual = 100.0f;
        bateria.calibrada       = true;
        prefs.begin("rrfs", false);
        prefs.putFloat("bat_ah_at", bateria.ahAtual);
        prefs.putBool ("bat_calib", true);
        prefs.end();
        adicionarLog("Bat auto-calib 100%");
    }
}

void atualizarAhBateria() {
    if (!bateria.calibrada) return;
    unsigned long agora  = millis();
    float         deltaH = (float)(agora - ultimoContagem) / 3600000.0f;
    ultimoContagem = agora;
    bateria.ahAtual += sensores.correnteBateria * deltaH;
    if (bateria.ahAtual < 0) bateria.ahAtual = 0;
    if (bateria.ahAtual > bateria.ahDisponivel)
        bateria.ahAtual = bateria.ahDisponivel;
    if (bateria.ahDisponivel > 0)
        bateria.percentualAtual =
            (bateria.ahAtual / bateria.ahDisponivel) * 100.0f;
}

// ═══════════════════════════════════════════════════════════
// [NTP] SINCRONIZAÇÃO DE TEMPO
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
    prefs.end();
}

void carregarConfigTempo() {
    prefs.begin("rrfs_time", true);
    cfgTempo.fusoHoras   = prefs.getInt ("fuso_h",   -3);
    cfgTempo.fusoMinutos = prefs.getInt ("fuso_m",    0);
    cfgTempo.ntpAtivo    = prefs.getBool("ntp_ativo", false);
    cfgTempo.manualAnno  = prefs.getInt ("man_ano",  2024);
    cfgTempo.manualMes   = prefs.getInt ("man_mes",     1);
    cfgTempo.manualDia   = prefs.getInt ("man_dia",     1);
    cfgTempo.manualHora  = prefs.getInt ("man_hora",    0);
    cfgTempo.manualMin   = prefs.getInt ("man_min",     0);
    prefs.end();
}

void sincronizarNTP() {
    if (!cfgTempo.ntpAtivo) return;
    if (WiFi.status() != WL_CONNECTED) return;
    long offsetSeg = (long)cfgTempo.fusoHoras   * 3600L
                   + (long)cfgTempo.fusoMinutos  *   60L;
    configTime(offsetSeg, 0, NTP_SERVER1, NTP_SERVER2);
    unsigned long t = millis();
    struct tm ti;
    while (millis() - t < 5000) {
        if (getLocalTime(&ti)) {
            ntpSincronizado       = true;
            cfgTempo.sincronizado = true;
            adicionarLog("NTP sincronizado");
            tUltimoNTP = millis();
            return;
        }
        delay(200);
        esp_task_wdt_reset();
    }
    adicionarLog("NTP timeout");
}

bool obterTempoAtual(struct tm& t) {
    if (ntpSincronizado && cfgTempo.ntpAtivo)
        return getLocalTime(&t, 100);
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
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
             t.tm_hour, t.tm_min);
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
// [PREV] PREVISÃO SOLAR — OpenMeteo
// ═══════════════════════════════════════════════════════════
void salvarConfigPrevisao() {
    prefs.begin("rrfs_prev", false);
    prefs.putBool ("ativa",       cfgPrevisao.ativa);
    prefs.putFloat("lat",         cfgPrevisao.latitude);
    prefs.putFloat("lon",         cfgPrevisao.longitude);
    prefs.putFloat("painel_kw",   cfgPrevisao.painelKW);
    prefs.putFloat("ctrl_a",      cfgPrevisao.controladorA);
    prefs.putFloat("fator_calib", cfgPrevisao.fatorCalib);
    prefs.putBool ("configurada", cfgPrevisao.configurada);
    prefs.end();
}

void carregarConfigPrevisao() {
    prefs.begin("rrfs_prev", true);
    cfgPrevisao.ativa        = prefs.getBool ("ativa",      false);
    cfgPrevisao.latitude     = prefs.getFloat("lat",         0.0f);
    cfgPrevisao.longitude    = prefs.getFloat("lon",         0.0f);
    cfgPrevisao.painelKW     = prefs.getFloat("painel_kw",  1.0f);
    cfgPrevisao.controladorA = prefs.getFloat("ctrl_a",    30.0f);
    cfgPrevisao.fatorCalib   = prefs.getFloat("fator_calib",1.0f);
    cfgPrevisao.configurada  = prefs.getBool ("configurada",false);
    prefs.end();
}

int getIndiceHoraAtual() {
    if (!previsao.valido) return -1;
    struct tm t;
    if (!obterTempoAtual(t)) return -1;
    int diffH = (t.tm_year + 1900 - previsao.anoBase) * 8760
              + (t.tm_mon  + 1    - previsao.mesBase)  *  730
              + (t.tm_mday        - previsao.diaBase)   *   24
              + (t.tm_hour        - previsao.horaBase);
    if (diffH < 0 || diffH >= PREV_TOTAL_HORAS) return -1;
    return diffH;
}

float getAhPrevHoraAtual() {
    int idx = getIndiceHoraAtual();
    if (idx < 0) return 0.0f;
    return previsao.ahPrevisto[idx];
}

float getAhPrevRestanteDia() {
    int idx = getIndiceHoraAtual();
    if (idx < 0) return 0.0f;
    struct tm t;
    if (!obterTempoAtual(t)) return 0.0f;
    int   restantes = 23 - t.tm_hour;
    float total     = 0.0f;
    for (int i = 0; i <= restantes && (idx + i) < PREV_TOTAL_HORAS; i++)
        total += previsao.ahPrevisto[idx + i];
    return total;
}

void calcularAhPrevisto() {
    float vn     = bateria.tensaoNominal > 0.1f ? bateria.tensaoNominal : 12.8f;
    float pKW    = cfgPrevisao.painelKW;
    float maxA   = cfgPrevisao.controladorA;
    float fatorC = cfgPrevisao.fatorCalib;
    for (int d = 0; d < PREV_DIAS; d++) previsao.ahDia[d] = 0.0f;
    for (int h = 0; h < PREV_TOTAL_HORAS; h++) {
        float rad  = previsao.radiacao[h];
        if (rad < 0) rad = 0;
        float corrA = (rad / 1000.0f) * (pKW * 1000.0f) / vn;
        if (corrA > maxA) corrA = maxA;
        corrA *= fatorC;
        previsao.ahPrevisto[h] = corrA;
        int diaIdx = h / 24;
        if (diaIdx < PREV_DIAS) previsao.ahDia[diaIdx] += corrA;
    }
}

void calibrarPrevisao() {
    if (!previsao.valido || !cfgPrevisao.configurada) return;
    int idx = getIndiceHoraAtual();
    if (idx < 0) return;
    float ahReal = acum.painelAh;
    float ahPrev = previsao.ahPrevisto[idx];
    if (ahPrev < 0.1f || ahReal < 0.1f) return;
    float novoFator = ahReal / ahPrev;
    cfgPrevisao.fatorCalib = cfgPrevisao.fatorCalib * 0.90f
                           + novoFator              * 0.10f;
    if (cfgPrevisao.fatorCalib < 0.3f) cfgPrevisao.fatorCalib = 0.3f;
    if (cfgPrevisao.fatorCalib > 2.0f) cfgPrevisao.fatorCalib = 2.0f;
    calcularAhPrevisto();
    prefs.begin("rrfs_prev", false);
    prefs.putFloat("fator_calib", cfgPrevisao.fatorCalib);
    prefs.end();
}

bool buscarPrevisaoOpenMeteo() {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (!cfgPrevisao.configurada)       return false;
    char url[256];
    snprintf(url, sizeof(url),
        "http://api.open-meteo.com/v1/forecast"
        "?latitude=%.4f&longitude=%.4f"
        "&hourly=shortwave_radiation"
        "&forecast_days=7&timezone=auto",
        cfgPrevisao.latitude, cfgPrevisao.longitude);
    HTTPClient http;
    http.begin(url);
    http.setTimeout(10000);
    int code = http.GET();
    if (code != 200) {
        http.end();
        snprintf(tempBuf, sizeof(tempBuf), "Prev HTTP %d", code);
        adicionarLog(tempBuf);
        return false;
    }
    WiFiClient* stream = http.getStreamPtr();
    String body = "";
    body.reserve(8192);
    unsigned long tL = millis();
    while (http.connected() && millis() - tL < 8000) {
        if (stream->available()) {
            char c = stream->read();
            body += c;
            if (body.length() > 16000) break;
        }
        esp_task_wdt_reset();
    }
    http.end();
    int posArr = body.indexOf("shortwave_radiation\":[");
    if (posArr < 0) { adicionarLog("Prev: array nao encontrado"); return false; }
    posArr += 22;
    int count = 0, pos = posArr;
    while (count < PREV_TOTAL_HORAS && pos < (int)body.length()) {
        while (pos < (int)body.length() &&
               body[pos] != '-' && body[pos] != '.' &&
               (body[pos] < '0' || body[pos] > '9')) {
            if (body[pos] == ']') goto fimParser;
            pos++;
        }
        if (pos >= (int)body.length()) break;
        float val = 0.0f; bool neg = false;
        if (body[pos] == '-') { neg = true; pos++; }
        while (pos < (int)body.length() &&
               body[pos] >= '0' && body[pos] <= '9')
            val = val * 10.0f + (body[pos++] - '0');
        if (pos < (int)body.length() && body[pos] == '.') {
            pos++; float dec = 0.1f;
            while (pos < (int)body.length() &&
                   body[pos] >= '0' && body[pos] <= '9')
                val += (body[pos++] - '0') * dec, dec *= 0.1f;
        }
        if (neg) val = -val;
        previsao.radiacao[count++] = val;
    }
    fimParser:
    if (count < 24) { adicionarLog("Prev: dados insuf"); return false; }
    for (int i = count; i < PREV_TOTAL_HORAS; i++) previsao.radiacao[i] = 0.0f;
    struct tm t;
    if (obterTempoAtual(t)) {
        previsao.horaBase = t.tm_hour;
        previsao.diaBase  = t.tm_mday;
        previsao.mesBase  = t.tm_mon + 1;
        previsao.anoBase  = t.tm_year + 1900;
    }
    previsao.tAtualizacao = millis();
    previsao.valido       = true;
    calcularAhPrevisto();
    snprintf(tempBuf, sizeof(tempBuf), "Prev OK %d h fator=%.2f",
             count, cfgPrevisao.fatorCalib);
    adicionarLog(tempBuf);
    return true;
}

void gerenciarPrevisao() {
    if (!cfgPrevisao.ativa || !cfgPrevisao.configurada) return;
    unsigned long agora = millis();
    if (!previsao.valido ||
        agora - tUltimaPrevisao >= INTERVALO_PREVISAO_MS) {
        tUltimaPrevisao = agora;
        esp_task_wdt_reset();
        buscarPrevisaoOpenMeteo();
        esp_task_wdt_reset();
    }
    static unsigned long tUltimaCalib = 0;
    if (agora - tUltimaCalib >= 3600000UL) {
        tUltimaCalib = agora;
        calibrarPrevisao();
    }
}

// ═══════════════════════════════════════════════════════════
// [HIST] HISTÓRICO
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
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);
    prefs.begin(ns, false);
    prefs.putBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();
}

bool carregarDiaFlash(uint16_t idx, RegistroDia& dia) {
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);
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

void salvarHistorico() {
    salvarDiaFlash(histIndiceAtual, histDiaAtual);
    salvarIndiceHistorico();
}

void carregarHistorico() {
    prefs.begin("rrfs_hidx", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver != HIST_KEY_VER) {
        prefs.end();
        histIndiceAtual = 0; histTotalDias = 0;
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
        adicionarLog("Hist reset v21");
        return;
    }
    histIndiceAtual = prefs.getUShort("idx",   0);
    histTotalDias   = prefs.getUShort("total", 0);
    prefs.end();
    if (histTotalDias > 0 &&
        !carregarDiaFlash(histIndiceAtual, histDiaAtual))
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
}

void atualizarDiaAtual() {
    unsigned long agora = millis();
    if (agora - ultimaAtualizacaoDia < 60000UL && diaCache != 0) return;
    ultimaAtualizacaoDia = agora;
    struct tm t;
    if (obterTempoAtual(t)) {
        diaCache = t.tm_mday;
        mesCache = t.tm_mon + 1;
        anoCache = t.tm_year + 1900;
    } else {
        if (diaCache == 0) { diaCache = 1; mesCache = 1; anoCache = 2024; }
        else {
            diaCache++;
            if (diaCache > 28) {
                diaCache = 1; mesCache++;
                if (mesCache > 12) { mesCache = 1; anoCache++; }
            }
        }
    }
}

void acumularEnergia() {
    atualizarDiaAtual();
    unsigned long agora = millis();
    struct tm t;
    uint8_t  hA = 0; uint8_t  dA = diaCache;
    uint8_t  mA = mesCache; uint16_t aA = anoCache;
    if (obterTempoAtual(t)) {
        hA = t.tm_hour; dA = t.tm_mday;
        mA = t.tm_mon + 1; aA = t.tm_year + 1900;
    } else { hA = (agora / 3600000UL) % 24; }

    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum = agora; acum.horaAtual = hA;
        acum.diaAtual   = dA;   acum.mesAtual  = mA; acum.anoAtual = aA;
        bool diaEx = false;
        if (histTotalDias > 0) {
            RegistroDia dT;
            if (carregarDiaFlash(histIndiceAtual, dT) &&
                dT.dia == dA && dT.mes == mA && dT.ano == aA)
                { histDiaAtual = dT; diaEx = true; }
        }
        if (!diaEx) {
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano = aA; histDiaAtual.mes = mA;
            histDiaAtual.dia = dA; histDiaAtual.valido = true;
            if (histTotalDias == 0) { histIndiceAtual = 0; histTotalDias = 1; }
            else {
                RegistroDia dT;
                if (carregarDiaFlash(histIndiceAtual, dT) &&
                    (dT.dia != dA || dT.mes != mA || dT.ano != aA)) {
                    histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
                    if (histTotalDias < HIST_DIAS) histTotalDias++;
                }
            }
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
            salvarIndiceHistorico();
        }
        return;
    }

    float dH = (float)(agora - acum.ultimoAcum) / 3600000.0f;
    acum.ultimoAcum = agora;
    if (dH > 0.1f) dH = 0.1f;
    float ip = sensores.correntePainel;
    float ic = sensores.correnteCasa;
    float ib = sensores.correnteBateria;
    if (ip >  0.15f) acum.painelAh  += ip        * dH;
    if (ic >  0.3f)  acum.casaAh    += ic        * dH;
    if (ib >  0.3f)  acum.batCargAh += ib        * dH;
    if (ib < -0.3f)  acum.batDescAh += fabsf(ib) * dH;

    bool mudouHora = (hA != acum.horaAtual);
    bool mudouDia  = (dA != acum.diaAtual ||
                      mA != acum.mesAtual || aA != acum.anoAtual);
    if (mudouHora || mudouDia) {
        uint8_t hi = acum.horaAtual % HIST_HORAS;
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
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
            histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
            if (histTotalDias < HIST_DIAS) histTotalDias++;
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano = aA; histDiaAtual.mes = mA;
            histDiaAtual.dia = dA; histDiaAtual.valido = true;
        }
        salvarDiaFlash(histIndiceAtual, histDiaAtual);
        salvarIndiceHistorico();
        salvarContadores();
        acum.painelAh = acum.casaAh = acum.batCargAh = acum.batDescAh = 0;
        acum.horaAtual = hA;
        if (mudouDia) { acum.diaAtual=dA; acum.mesAtual=mA; acum.anoAtual=aA; }
    }
    static unsigned long tParcial = 0;
    if (agora - tParcial >= 300000UL) {
        tParcial = agora;
        salvarDiaFlash(histIndiceAtual, histDiaAtual);
        salvarContadores();
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] INVERSOR BAIXA POTÊNCIA
// ═══════════════════════════════════════════════════════════
void controlarInvBaixa() {
    if (!bateria.calibrada) return;

    // Se falha definitiva — não religar automaticamente
    if (monInvBaixa.falhaDefinitiva) return;

    float pct = bateria.percentualAtual;
    if (!estados.invBaixa && pct >= limiares.pctInvBaixa) {
        ligarInvBaixa();
        // Reseta monitor ao ligar
        monInvBaixa.estado        = MON_NORMAL;
        monInvBaixa.tentativas    = 0;
        monInvBaixa.statusOK      = false;
        monInvBaixa.emReligue     = false;
        monInvBaixa.tInicioEstado = millis();
        adicionarLog("InvBaixa ON");
    } else if (estados.invBaixa && pct < limiares.pctInvBaixa) {
        desligarInvBaixa();
        monInvBaixa.estado    = MON_NORMAL;
        monInvBaixa.statusOK  = false;
        adicionarLog("InvBaixa OFF bat baixa");
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] INVERSOR ALTA POTÊNCIA
// ═══════════════════════════════════════════════════════════
void controlarInvAlta() {
    if (!bateria.calibrada) return;

    // Se falha definitiva — não religar automaticamente
    if (monInvAlta.falhaDefinitiva) return;

    float pct      = bateria.percentualAtual;
    bool  sinAtivo = sinalCasaAtivo();
    bool  batOK    = (pct >= limiares.pctInvAlta);

    if (!estados.invAlta) {
        if (sinAtivo && batOK) {
            ligarInvAlta();
            // Reseta monitor ao ligar
            monInvAlta.estado        = MON_NORMAL;
            monInvAlta.tentativas    = 0;
            monInvAlta.statusOK      = false;
            monInvAlta.emReligue     = false;
            monInvAlta.tInicioEstado = millis();
            sinalCasa.semEnergia     = false;
            adicionarLog("InvAlta ON sinal+bat OK");
        } else if (sinAtivo && !batOK) {
            if (!sinalCasa.semEnergia) {
                sinalCasa.semEnergia = true;
                adicionarLog("InvAlta: sinal OK bat insuf");
            }
        } else {
            sinalCasa.semEnergia = false;
        }
    } else {
        if (!sinAtivo) {
            desligarInvAlta();
            monInvAlta.estado    = MON_NORMAL;
            monInvAlta.statusOK  = false;
            sinalCasa.semEnergia = false;
            adicionarLog("InvAlta OFF sinal cortado");
        } else if (!batOK) {
            desligarInvAlta();
            monInvAlta.estado    = MON_NORMAL;
            monInvAlta.statusOK  = false;
            sinalCasa.semEnergia = true;
            adicionarLog("InvAlta OFF bat baixa");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] LÂMPADA EXTERNA
// ═══════════════════════════════════════════════════════════
void controlarLampExt() {
    unsigned long agora  = millis();
    bool gerando = (sensores.correntePainel > 0.5f);

    if (gerando != lampExt.estadoAnteriorPainel) {
        lampExt.estadoAnteriorPainel = gerando;
        lampExt.tUltimaMudanca       = agora;
    }
    unsigned long decorrido = agora - lampExt.tUltimaMudanca;

    if (!gerando && !estados.lampExt) {
        if (decorrido >= LAMP_EXT_DELAY_MS && estados.invBaixa) {
            if (!bateria.calibrada ||
                bateria.percentualAtual > limiares.pctLampExt) {
                ligarLampExt();
                adicionarLog("LampExt ON noite");
            }
        }
    } else if (gerando && estados.lampExt) {
        if (decorrido >= LAMP_EXT_DELAY_MS) {
            desligarLampExt();
            adicionarLog("LampExt OFF dia");
        }
    }
    if (estados.lampExt && bateria.calibrada &&
        bateria.percentualAtual <= limiares.pctLampExt) {
        desligarLampExt();
        adicionarLog("LampExt OFF bat");
    }
}

// ═══════════════════════════════════════════════════════════
// [MON] MONITORAR INVERSORES — chamado no loop
// ═══════════════════════════════════════════════════════════
void monitorarInversores() {
    // Lê sinais de status com média móvel
    lerStatusInversor(monInvBaixa, PIN_STATUS_INV_BAIXA);
    lerStatusInversor(monInvAlta,  PIN_STATUS_INV_ALTA);

    // Processa monitor inv. baixa (deve estar ligado se relé ativo)
    processarMonitorInversor(
        monInvBaixa,
        estados.invBaixa,
        "InvBaixa",
        ligarInvBaixa,
        desligarInvBaixa
    );

    // Processa monitor inv. alta
    // Só monitora se relé ativo E sinal casa confirmado
    // (se sinal casa inativo, inv. alta deve estar desligado)
    processarMonitorInversor(
        monInvAlta,
        estados.invAlta && sinalCasaAtivo(),
        "InvAlta",
        ligarInvAlta,
        desligarInvAlta
    );
}

void controlarEnergia() {
    controlarInvBaixa();
    controlarInvAlta();
    controlarLampExt();
}

// ═══════════════════════════════════════════════════════════
// [PREF] PREFERENCES
// ═══════════════════════════════════════════════════════════
bool existeCredenciais() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("user");
    prefs.end();
    return ok;
}

bool existeWiFiConfig() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("wifi_ok");
    prefs.end();
    return ok;
}

void salvarCredenciais(const String& u, const String& p) {
    prefs.begin("rrfs", false);
    prefs.putString("user", u);
    prefs.putString("pass", p);
    prefs.end();
}

bool verificarLogin(const String& u, const String& p) {
    prefs.begin("rrfs", true);
    String su = prefs.getString("user", "");
    String sp = prefs.getString("pass", "");
    prefs.end();
    return (su == u && sp == p && su.length() > 0);
}

void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap) {
    prefs.begin("rrfs", false);
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

void carregarWifi(String& ssid, String& pass, String& ip, String& gw,
                  String& mask, String& dns, bool& ap) {
    prefs.begin("rrfs", true);
    ap   = prefs.getBool  ("usar_ap",   true);
    ssid = prefs.getString("wifi_ssid", "");
    pass = prefs.getString("wifi_pass", "");
    ip   = prefs.getString("wifi_ip",   "");
    gw   = prefs.getString("wifi_gw",   "");
    mask = prefs.getString("wifi_mask", "255.255.255.0");
    dns  = prefs.getString("wifi_dns",  "8.8.8.8");
    prefs.end();
}

void salvarConfigBateria() {
    prefs.begin("rrfs", false);
    prefs.putFloat("bat_vnom",  bateria.tensaoNominal);
    prefs.putFloat("bat_ah",    bateria.capacidadeAh);
    prefs.putFloat("bat_desc",  bateria.descargaSegura);
    prefs.putFloat("bat_ah_at", bateria.ahAtual);
    prefs.putBool ("bat_calib", bateria.calibrada);
    prefs.end();
}

void carregarConfigBateria() {
    prefs.begin("rrfs", true);
    bateria.tensaoNominal  = prefs.getFloat("bat_vnom",  12.8f);
    bateria.capacidadeAh   = prefs.getFloat("bat_ah",   100.0f);
    bateria.descargaSegura = prefs.getFloat("bat_desc",  80.0f);
    bateria.ahAtual        = prefs.getFloat("bat_ah_at",  0.0f);
    bateria.calibrada      = prefs.getBool ("bat_calib", false);
    prefs.end();
    calcularBateria();
    if (bateria.ahDisponivel > 0)
        bateria.percentualAtual =
            (bateria.ahAtual / bateria.ahDisponivel) * 100.0f;
}

void salvarLimiares() {
    prefs.begin("rrfs", false);
    prefs.putFloat("lim_invb", limiares.pctInvBaixa);
    prefs.putFloat("lim_inva", limiares.pctInvAlta);
    prefs.putFloat("lim_lamp", limiares.pctLampExt);
    prefs.end();
}

void carregarLimiares() {
    prefs.begin("rrfs", true);
    limiares.pctInvBaixa = prefs.getFloat("lim_invb", 20.0f);
    limiares.pctInvAlta  = prefs.getFloat("lim_inva", 40.0f);
    limiares.pctLampExt  = prefs.getFloat("lim_lamp", 30.0f);
    prefs.end();
}

void carregarCalibSensores() {
    calibPainel.offsetMV  = PAINEL_OFFSET_MV;
    calibPainel.sensMvA   = 35.9f;
    calibPainel.calibrado = true;
    calibCasa.offsetMV    = 541;
    calibCasa.sensMvA     = 43.3f;
    calibCasa.calibrado   = true;
    prefs.begin("rrfs_cal", true);
    if (prefs.isKey("pn_off"))
        calibPainel.offsetMV = prefs.getInt  ("pn_off",   calibPainel.offsetMV);
    if (prefs.isKey("pn_sens"))
        calibPainel.sensMvA  = prefs.getFloat("pn_sens",  calibPainel.sensMvA);
    if (prefs.isKey("casa_off"))
        calibCasa.offsetMV   = prefs.getInt  ("casa_off", calibCasa.offsetMV);
    if (prefs.isKey("casa_sens"))
        calibCasa.sensMvA    = prefs.getFloat("casa_sens",calibCasa.sensMvA);
    prefs.end();
}

// ═══════════════════════════════════════════════════════════
// [WIFI] REDE
// ═══════════════════════════════════════════════════════════
void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(50);
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
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(50);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.config(lIP, lGW, lMask, lDNS);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    adicionarLog("WiFi conectando"); ledBlink();
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        atualizarLED(); esp_task_wdt_reset(); delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
        adicionarLog("WiFi OK");
        modoAP = false; wifiFalhasConsec = 0; ledFixo(true); return true;
    }
    adicionarLog("WiFi FALHOU"); return false;
}

void gerenciarWiFi() {
    unsigned long agora = millis();
    if (agora - tUltimoCheckWifi < WIFI_CHECK_INTERVALO) return;
    tUltimoCheckWifi = agora;
    if (gWifiSSID.isEmpty()) return;
    if (WiFi.status() == WL_CONNECTED) {
        wifiFalhasConsec = 0; wifiReconectando = false; tInicioRecon = 0; return;
    }
    if (agora - tUltimoReconWifi < WIFI_RECON_INTERVALO) return;
    wifiFalhasConsec++;
    if (!wifiReconectando) {
        wifiReconectando = true; tInicioRecon = agora; tUltimoReconWifi = agora;
        if (!gWifiIP.isEmpty()) {
            IPAddress lIP, lGW, lMask, lDNS;
            lIP.fromString(gWifiIP); lGW.fromString(gWifiGW);
            lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
            WiFi.config(lIP, lGW, lMask, lDNS);
        }
        WiFi.reconnect(); return;
    }
    if (WiFi.status() == WL_CONNECTED) {
        adicionarLog("WiFi reconectado");
        wifiFalhasConsec = 0; wifiReconectando = false;
        tInicioRecon = 0; ledFixo(true); return;
    }
    if (agora - tInicioRecon >= WIFI_RECON_TIMEOUT) {
        wifiReconectando = false; tInicioRecon = 0; tUltimoReconWifi = agora;
        if (wifiFalhasConsec >= WIFI_MAX_FALHAS) {
            adicionarLog("WiFi falhas AP temp");
            wifiFalhasConsec = 0;
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
            WiFi.softAP(AP_SSID);
            dnsServer.start(DNS_PORT, "*", AP_IP);
            modoAP = true; ledBlink();
            WiFi.begin(gWifiSSID.c_str(), gWifiPASS.c_str());
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [SETUP]
// ═══════════════════════════════════════════════════════════
void setup() {
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms     = 30000,
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
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    esp_reset_reason_t motivo = esp_reset_reason();
    switch (motivo) {
        case ESP_RST_BROWNOUT: adicionarLog("RESET BROWNOUT"); break;
        case ESP_RST_PANIC:    adicionarLog("RESET PANIC");    break;
        case ESP_RST_TASK_WDT: adicionarLog("RESET TASK WDT");break;
        case ESP_RST_INT_WDT:  adicionarLog("RESET INT WDT"); break;
        case ESP_RST_WDT:      adicionarLog("RESET WDT");     break;
        case ESP_RST_SW:       adicionarLog("Reset software"); break;
        case ESP_RST_POWERON:  adicionarLog("Power ON");      break;
        default:
            snprintf(tempBuf, sizeof(tempBuf), "Reset %d", (int)motivo);
            adicionarLog(tempBuf); break;
    }

    esp_task_wdt_reset();
    setupLED();
    setupBotao();
    setupReles();

    // Pinos de entrada analógica
    pinMode(PIN_SINAL_CASA,       INPUT);
    pinMode(PIN_STATUS_INV_BAIXA, INPUT);
    pinMode(PIN_STATUS_INV_ALTA,  INPUT);

    ledBlink();
    adicionarLog("RRFS v21");
    logRAM();

    esp_task_wdt_reset();
    carregarConfigBateria();
    carregarLimiares();
    carregarCalibSensores();
    carregarHistorico();
    carregarContadores();
    carregarConfigTempo();
    carregarConfigPrevisao();

    esp_task_wdt_reset();
    ultimoContagem = millis();

    bool usarAP = true;
    if (existeWiFiConfig()) {
        String ssid, pass, ip, gw, mask, dns;
        carregarWifi(ssid, pass, ip, gw, mask, dns, usarAP);
        esp_task_wdt_reset();
        if (!usarAP && ssid.length() > 0)
            if (iniciarWiFiNB(ssid, pass, ip, gw, mask, dns))
                usarAP = false;
    }

    esp_task_wdt_reset();
    if (usarAP) iniciarAP();

    if (!modoAP && cfgTempo.ntpAtivo) {
        esp_task_wdt_reset();
        sincronizarNTP();
    }

    esp_task_wdt_reset();
    configurarRotas();
    adicionarLog("Sistema pronto v21");
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

    if (modoAP || WiFi.getMode() == WIFI_AP_STA)
        dnsServer.processNextRequest();

    server.handleClient();
    esp_task_wdt_reset();

    unsigned long agora = millis();

    // Leitura sensores + sinais a cada 1s
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;
        esp_task_wdt_reset();
        lerSensores();
        processarSinalCasa();
        monitorarInversores();   // lê status + processa religue automático
        acumularEnergia();
        esp_task_wdt_reset();
    }

    // Controle energia a cada 5s
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;
        controlarEnergia();
        esp_task_wdt_reset();
    }

    // NTP a cada 1 hora
    if (cfgTempo.ntpAtivo &&
        WiFi.status() == WL_CONNECTED &&
        agora - tUltimoNTP >= INTERVALO_NTP_MS) {
        sincronizarNTP();
        esp_task_wdt_reset();
    }

    // Previsão solar
    gerenciarPrevisao();
    esp_task_wdt_reset();

    gerenciarWiFi();
    esp_task_wdt_reset();

    // Salva bateria a cada 5min
    {
        static unsigned long tSalva = 0;
        if (bateria.calibrada && agora - tSalva >= 300000UL) {
            tSalva = agora;
            esp_task_wdt_reset();
            prefs.begin("rrfs", false);
            prefs.putFloat("bat_ah_at", bateria.ahAtual);
            prefs.end();
            esp_task_wdt_reset();
        }
    }

    // Log RAM a cada 10min
    {
        static unsigned long tRAM = 0;
        if (agora - tRAM >= 600000UL) {
            tRAM = agora;
            logRAM();
        }
    }

    esp_task_wdt_reset();
    delay(1);
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
.btn-sm{font-size:12px;padding:8px}
.ok{background:#1b5e20;color:#a5d6a7;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.er{background:#b71c1c;color:#ef9a9a;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.wn{background:#e65100;color:#ffe0b2;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.nf{background:#0d47a1;color:#90caf9;padding:10px;border-radius:4px;margin:8px 0;font-size:12px;line-height:1.5}
.al{background:#4a0000;border:2px solid #c62828;color:#ef9a9a;padding:10px;border-radius:6px;margin:6px 0;font-size:12px}
.nav{display:flex;gap:4px;margin-top:12px;padding-top:10px;border-top:1px solid #333}
.nav a{flex:1;text-align:center;padding:9px 4px;background:#1e1e1e;color:#90a4ae;text-decoration:none;border-radius:4px;font-size:11px;border:1px solid #333}
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
.ph{background:#111;border:1px solid #333;border-radius:6px;padding:8px;margin:3px 0;display:flex;justify-content:space-between;align-items:center}
.ph .ht{font-size:11px;color:#546e7a}
.ph .hv{font-size:13px;font-weight:bold;color:#fdd835}
.ph .hr{font-size:10px;color:#78909c}
.pd{background:#0a1a0a;border:1px solid #1b5e20;border-radius:6px;padding:10px;margin:4px 0}
.pd .dt{font-size:11px;color:#43a047;font-weight:bold;margin-bottom:4px}
.pd .dv{font-size:18px;font-weight:bold;color:#a5d6a7}
.pd .ds{font-size:10px;color:#546e7a;margin-top:2px}
.sa{background:#1a1200;border:2px solid #f9a825;border-radius:8px;padding:10px;margin:6px 0}
.sa .st{font-size:11px;color:#f9a825;font-weight:bold;margin-bottom:4px}
.sa .sv{font-size:20px;font-weight:bold;color:#fdd835}
.sa .si{font-size:10px;color:#78909c;margin-top:4px}
.mi{border-radius:8px;border:2px solid #333;background:#0d0d0d;padding:10px;margin:5px 0}
.mi .mt{font-size:11px;color:#90a4ae;font-weight:bold;margin-bottom:6px}
.mi .ms{font-size:13px;font-weight:bold}
.mi .md{font-size:10px;color:#546e7a;margin-top:4px;line-height:1.6}
.mi .mb{display:flex;gap:4px;margin-top:6px}
)RAW";

// ═══════════════════════════════════════════════════════════
// [WEB] HELPERS
// ═══════════════════════════════════════════════════════════
String barraProgresso(float pct, const String& label) {
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    String cor = pct >= 50 ? "#2e7d32" : pct >= 25 ? "#ef6c00" : "#c62828";
    String h   = "<div class='bb'><div class='bv' style='width:";
    h += String((int)pct) + "%;background:" + cor + "'>";
    h += String((int)pct) + "%</div></div>";
    if (label.length())
        h += "<div style='font-size:11px;color:#546e7a;text-align:center'>"
             + label + "</div>";
    return h;
}

// Helper: card de monitor de inversor
String cardMonitor(const char* nome, const char* ico,
                   MonitorInversor& mon, bool releAtivo) {
    String cor, txt, sub;

    if (!releAtivo) {
        cor = "#546e7a"; txt = "&#9675; Desligado";
        sub = "Rele desativado — sem monitoramento";
    } else if (mon.falhaDefinitiva) {
        cor = "#c62828"; txt = "&#9888; FALHA DEFINITIVA";
        sub = "Esgotadas " + String(STATUS_MAX_TENTATIVAS)
              + " tentativas. Verifique o inversor!";
    } else if (mon.emReligue) {
        cor = "#f9a825";
        switch (mon.estado) {
            case MON_FALHA_DET:
                txt = "&#9660; Falha detectada";
                sub = "Aguardando confirmacao 5s..."; break;
            case MON_DESLIGANDO:
                txt = "&#9660; Desligando";
                sub = "Aguardando 5s antes de religar..."; break;
            case MON_RELIGANDO:
                txt = "&#9650; Religando";
                sub = "Tentativa " + String(mon.tentativas)
                      + "/" + String(STATUS_MAX_TENTATIVAS)
                      + " — aguardando status 5s..."; break;
            default:
                txt = "&#9650; Religue"; sub = "Processando..."; break;
        }
    } else if (mon.statusOK) {
        cor = "#43a047"; txt = "&#9679; OK";
        sub = "Status confirmado — inversor respondendo";
    } else {
        cor = "#ef6c00"; txt = "&#9675; Aguardando";
        sub = "Aguardando sinal de status do inversor...";
    }

    char mvB[20];
    snprintf(mvB, sizeof(mvB), "%.0f mV", mon.mediaMV);

    String c = "<div class='mi' style='border-color:";
    c += cor + "'>";
    c += "<div class='mt'>" + String(ico) + " " + String(nome) + "</div>";
    c += "<div class='cr'>";
    c += "<span class='ms' style='color:" + cor + "'>" + txt + "</span>";
    c += "<span style='font-size:11px;color:#78909c'>" + String(mvB) + "</span>";
    c += "</div>";
    c += "<div class='md'>" + sub + "<br>";
    c += "Falhas: <b>" + String(mon.totalFalhas) + "</b> | ";
    c += "Religues OK: <b>" + String(mon.totalReligues) + "</b> | ";
    c += "Limiar: " + String(STATUS_INV_LIMIAR_MV) + "mV";
    c += "</div></div>";
    return c;
}

String gerarPagina(const String& titulo, const String& conteudo, bool comNav) {
    String html;
    html.reserve(8000);
    html = F("<!DOCTYPE html><html><head>"
             "<meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>");
    html += CSS;
    html += F("</style></head><body><div class='card'>");
    html += F("<h2>&#9889; RRFS v21</h2>");
    html += conteudo;

    if (comNav && logado) {
        String pag = server.uri();
        html += F("<div class='nav'>");
        html += "<a href='/' "          + String(pag=="/"          ?"class='active'":"") + ">&#128202;</a>";
        html += "<a href='/bateria' "   + String(pag=="/bateria"   ?"class='active'":"") + ">&#128267;</a>";
        html += "<a href='/redes' "     + String(pag=="/redes"     ?"class='active'":"") + ">&#128268;</a>";
        html += "<a href='/sensores' "  + String(pag=="/sensores"  ?"class='active'":"") + ">&#128270;</a>";
        html += "<a href='/previsao' "  + String(pag=="/previsao"  ?"class='active'":"") + ">&#127774;</a>";
        html += "<a href='/historico' " + String(pag=="/historico" ?"class='active'":"") + ">&#128200;</a>";
        html += "<a href='/config' "    + String(pag=="/config"    ?"class='active'":"") + ">&#9881;</a>";
        html += F("</div>");
    }

    html += F("<div class='lb'>&#128221; Logs</div>"
              "<div class='lbx' id='lb'><div style='color:#666'>...</div></div>"
              "</div>"
              "<script>"
              "function aL(){"
              "var x=new XMLHttpRequest();"
              "x.onreadystatechange=function(){"
              "if(x.readyState==4&&x.status==200){"
              "try{"
              "var d=document.getElementById('lb');"
              "var a=JSON.parse(x.responseText);"
              "var h='';"
              "for(var i=0;i<a.length;i++)h+='<div class=\"ll\">'+a[i]+'</div>';"
              "d.innerHTML=h||'<div style=\"color:#666\">Sem logs</div>';"
              "d.scrollTop=d.scrollHeight;"
              "}catch(e){}}}"
              ";x.open('GET','/logs',true);x.send();}"
              "setInterval(aL,4000);aL();"
              "</script></body></html>");
    return html;
}

// ═══════════════════════════════════════════════════════════
// [AUTH] LOGIN / SETUP
// ═══════════════════════════════════════════════════════════
void handleSetupLogin() {
    if (existeCredenciais() && existeWiFiConfig()) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String u = server.arg("user"),
               p = server.arg("pass"),
               cf = server.arg("conf");
        if (u.length() < 3)       msg = "<div class='er'>&#10007; Min 3 chars usuario</div>";
        else if (p.length() < 4)  msg = "<div class='er'>&#10007; Min 4 chars senha</div>";
        else if (p != cf)         msg = "<div class='er'>&#10007; Senhas nao conferem</div>";
        else {
            salvarCredenciais(u, p);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location", "/setup-wifi");
            server.send(302); return;
        }
    }
    String c = F("<div class='nf'>&#8505; Primeiro acesso.</div>");
    c += "<h3>&#128274; Criar Conta</h3>" + msg;
    c += F("<form method='POST' action='/setup-login'>"
           "<label>Usuario (min 3):</label>"
           "<input type='text' name='user' maxlength='32' required autocomplete='off'>"
           "<label>Senha (min 4):</label>"
           "<input type='password' name='pass' maxlength='32' required>"
           "<label>Confirmar:</label>"
           "<input type='password' name='conf' maxlength='32' required>"
           "<button type='submit'>Criar &#8594;</button></form>");
    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Setup", c, false));
}

void handleSetupWifi() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login"); server.send(302); return;
    }
    if (existeWiFiConfig() && !logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String modo = server.arg("modo");
        if (modo == "ap") {
            salvarWifi("", "", "", "", "", "", true);
            String ct = F("<div class='ok'>&#10003; Salvo! Reiniciando...</div>"
                          "<div class='nf'>SSID: <b>RRFS Energia</b> | IP: <b>10.0.0.1</b></div>");
            server.send(200, "text/html;charset=utf-8",
                        gerarPagina("Salvo", ct, false));
            delay(2000); ESP.restart(); return;
        }
        String ssid = server.arg("ssid"), wp   = server.arg("wpass"),
               ip   = server.arg("ip"),   gw   = server.arg("gw"),
               mask = server.arg("mask"), dns  = server.arg("dns");
        if (ssid.isEmpty())       msg = "<div class='er'>Selecione uma rede</div>";
        else if (ip.length() < 7) msg = "<div class='er'>Informe o IP</div>";
        else if (gw.length() < 7) msg = "<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid, wp, ip, gw, mask, dns, false);
            String ct = "<div class='ok'>&#10003; Salvo! Acesse: http://" + ip + "</div>";
            ct += F("<div class='nf'>Reiniciando...</div>");
            server.send(200, "text/html;charset=utf-8",
                        gerarPagina("Salvo", ct, false));
            delay(2000); ESP.restart(); return;
        }
    }
    static int16_t nRedes      = -1;
    static bool    scanIniciado = false;
    if (!scanIniciado) {
        WiFi.scanDelete();
        WiFi.scanNetworks(true, false, false, 300);
        scanIniciado = true; nRedes = -1;
    }
    if (nRedes == -1) nRedes = WiFi.scanComplete();

    String c = F("<div class='nf'>&#8505; IP fixo obrigatorio.</div>");
    c += "<h3>&#128246; Configurar Rede</h3>" + msg;
    c += F("<form method='POST' action='/setup-wifi'>"
           "<label>Modo:</label>"
           "<select name='modo' id='modo' onchange='toggleModo()'>"
           "<option value='wifi'>Conectar ao WiFi</option>"
           "<option value='ap'>Manter Ponto de Acesso</option>"
           "</select><div id='dw'>"
           "<label>Rede WiFi:</label><select name='ssid'>");
    if (nRedes == WIFI_SCAN_RUNNING) {
        c += F("<option>Escaneando...</option>");
    } else if (nRedes <= 0) {
        c += F("<option value=''>Nenhuma rede</option>");
        scanIniciado = false;
    } else {
        for (int i = 0; i < nRedes; i++) {
            String q = WiFi.RSSI(i) > -60 ? "Bom" :
                       WiFi.RSSI(i) > -75 ? "Regular" : "Fraco";
            char buf[32];
            snprintf(buf, sizeof(buf), " [%s %ddBm]", q.c_str(), WiFi.RSSI(i));
            c += "<option value='" + WiFi.SSID(i) + "'>"
                 + WiFi.SSID(i) + String(buf) + "</option>";
        }
        WiFi.scanDelete(); scanIniciado = false; nRedes = -1;
    }
    c += F("</select>"
           "<button type='button' class='btn-o btn-sm' "
           "onclick='location.reload()' style='margin-top:8px'>"
           "&#128259; Atualizar</button>"
           "<label>Senha WiFi:</label>"
           "<input type='password' name='wpass' placeholder='Vazio se aberta'>"
           "<label>IP Fixo:</label>"
           "<input type='text' name='ip' required placeholder='192.168.1.100'>"
           "<label>Gateway:</label>"
           "<input type='text' name='gw' required placeholder='192.168.1.1'>"
           "<label>Mascara:</label>"
           "<input type='text' name='mask' required value='255.255.255.0'>"
           "<label>DNS:</label>"
           "<input type='text' name='dns' required value='8.8.8.8'>"
           "</div>"
           "<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");
    if (logado)
        c += F("<button onclick='location.href=\"/\"' class='btn-s' "
               "style='margin-top:8px'>&#8592; Voltar</button>");
    c += F("<script>function toggleModo(){"
           "document.getElementById('dw').style.display="
           "document.getElementById('modo').value=='wifi'?'block':'none';}</script>");
    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Rede", c, false));
}

void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login"); server.send(302); return;
    }
    if (logado) {
        server.sendHeader("Location", "/"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        if (verificarLogin(server.arg("user"), server.arg("pass"))) {
            logado = true;
            adicionarLog("Login OK"); logRAM();
            server.sendHeader("Location", "/"); server.send(302); return;
        }
        adicionarLog("Login FALHOU");
        msg = F("<div class='er'>&#10007; Usuario/senha incorretos</div>");
    }
    String c = F("<div class='nf'>&#128274; Acesso seguro</div><h3>Entrar</h3>");
    c += msg;
    c += F("<form method='POST' action='/login'>"
           "<label>Usuario:</label>"
           "<input type='text' name='user' required autocomplete='off'>"
           "<label>Senha:</label>"
           "<input type='password' name='pass' required>"
           "<button type='submit'>&#128275; Entrar</button></form>");
    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Login", c, false));
}

void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado = false; }
    server.sendHeader("Location", "/login"); server.send(302);
}

// ═══════════════════════════════════════════════════════════
// [DASH] DASHBOARD PRINCIPAL
// ═══════════════════════════════════════════════════════════
void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login"); server.send(302); return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi"); server.send(302); return;
    }
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }

    bool painelGer = (sensores.correntePainel  >  0.5f);
    bool batCarg   = (sensores.correnteBateria >  0.3f);
    bool batDesc   = (sensores.correnteBateria < -0.3f);

    char sIP[12], sIC[12], sIB[12];
    char sWP[12], sWC[12], sWB[12];
    snprintf(sIP, sizeof(sIP), "%.1fA", sensores.correntePainel);
    snprintf(sIC, sizeof(sIC), "%.1fA", sensores.correnteCasa);
    snprintf(sIB, sizeof(sIB), "%.1fA", fabsf(sensores.correnteBateria));
    snprintf(sWP, sizeof(sWP), "%.0fW", sensores.potenciaPainel);
    snprintf(sWC, sizeof(sWC), "%.0fW", sensores.potenciaCasa);
    snprintf(sWB, sizeof(sWB), "%.0fW", fabsf(sensores.potenciaBateria));

    char pctBuf[8];
    snprintf(pctBuf, sizeof(pctBuf), "%.0f%%",
             bateria.calibrada ? bateria.percentualAtual : 0.0f);

    EstimativaTempo tempo = calcularTempoRestante();

    String cPainel = painelGer ? "#fdd835" : "#455a64";
    String cBat    = !bateria.calibrada         ? "#455a64" :
                      bateria.percentualAtual > 50 ? "#43a047" :
                      bateria.percentualAtual > 25 ? "#ef6c00" : "#c62828";

    String c = "";

    // Avisos críticos de falha definitiva
    if (monInvBaixa.falhaDefinitiva)
        c += F("<div class='al'>&#9888; FALHA DEFINITIVA: Inv. Baixa nao responde! "
               "Verifique o equipamento.</div>");
    if (monInvAlta.falhaDefinitiva)
        c += F("<div class='al'>&#9888; FALHA DEFINITIVA: Inv. Alta nao responde! "
               "Verifique o equipamento.</div>");

    // Aviso bateria não calibrada
    if (!bateria.calibrada)
        c += F("<div class='wn' style='font-size:11px;margin-bottom:6px'>"
               "&#9888; Bateria nao calibrada. "
               "<a href='/bateria' style='color:#ffe0b2'>Calibrar</a></div>");

    // Data/hora
    c += "<div style='text-align:center;font-size:11px;color:#546e7a;margin-bottom:6px'>";
    c += "&#128336; " + formatarDataHora();
    if (ntpSincronizado) c += F(" <span style='color:#43a047'>&#9679;NTP</span>");
    c += F("</div>");

    // ── Monitor Inversores ───────────────────────────────────
    c += F("<h3>&#128202; Monitor Inversores</h3>");
    c += cardMonitor("Inv. Baixa Potencia", "&#128268;",
                     monInvBaixa, estados.invBaixa);
    c += cardMonitor("Inv. Alta Potencia",  "&#9889;",
                     monInvAlta,  estados.invAlta && sinalCasaAtivo());

    // ── Sinal Casa ───────────────────────────────────────────
    c += F("<h3>&#128246; Sinal Alta Potencia</h3>");
    {
        String corS, txtS, subS;
        if (sinalCasa.ativo) {
            corS = "#43a047"; txtS = "&#9679; Ativo";
            subS = "Sinal confirmado — equipamento ligado";
        } else if (sinalCasa.aguardandoLigar) {
            corS = "#f9a825"; txtS = "&#9650; Aguardando...";
            unsigned long r = SINAL_CONFIRMA_MS
                - (millis() - sinalCasa.tInicioConfirma);
            char bR[20]; snprintf(bR, sizeof(bR), "Confirmando em %lus", r/1000UL);
            subS = String(bR);
        } else if (sinalCasa.aguardandoDesligar) {
            corS = "#ef6c00"; txtS = "&#9660; Encerrando...";
            unsigned long r = SINAL_CONFIRMA_MS
                - (millis() - sinalCasa.tInicioConfirma);
            char bR[20]; snprintf(bR, sizeof(bR), "Desligando em %lus", r/1000UL);
            subS = String(bR);
        } else {
            corS = "#546e7a"; txtS = "&#9675; Inativo";
            subS = "Nenhum equipamento de alta potencia";
        }
        char mvB[20];
        snprintf(mvB, sizeof(mvB), "%.0f mV", sinalCasa.mediaMV);
        c += "<div class='sa'>";
        c += "<div class='cr'>";
        c += "<span class='st'>&#128246; Casa</span>";
        c += "<span class='sv' style='color:" + corS + "'>" + txtS + "</span>";
        c += "</div>";
        c += "<div class='si'>" + subS + " | " + String(mvB) + "</div></div>";

        if (sinalCasa.semEnergia)
            c += F("<div class='wn' style='font-size:11px'>"
                   "&#9888; Sinal ativo mas bateria insuficiente!</div>");
    }

    // ── Status Inversores ────────────────────────────────────
    c += F("<h3>&#9889; Inversores</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
    {
        String cor = estados.invBaixa ? "#43a047" : "#546e7a";
        String txt = estados.invBaixa ? "&#9679; Ligado" : "&#9675; Aguardando";
        // Cor de borda muda se em religue
        if (monInvBaixa.emReligue) cor = "#f9a825";
        if (monInvBaixa.falhaDefinitiva) cor = "#c62828";
        c += "<div class='cg' style='border-color:" + cor + "'>";
        c += "<div class='cr'>";
        c += F("<span class='cn'>&#128268; Inv. Baixa</span>");
        c += "<span class='cs' style='color:" + cor + "'>" + txt + "</span>";
        c += "</div><div class='ci'>Limiar >=";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvBaixa);
        c += String(tempBuf) + "% | Religue auto: ";
        c += monInvBaixa.falhaDefinitiva ? "FALHOU" :
             monInvBaixa.emReligue       ? "EM CURSO" : "OK";
        c += "</div></div>";
    }
    {
        String cor = estados.invAlta ? "#43a047" :
                     sinalCasa.semEnergia ? "#ef6c00" : "#546e7a";
        String txt = estados.invAlta    ? "&#9679; Ligado" :
                     sinalCasa.semEnergia ? "&#9888; Bat insuf" : "&#9675; Aguardando";
        if (monInvAlta.emReligue) cor = "#f9a825";
        if (monInvAlta.falhaDefinitiva) cor = "#c62828";
        c += "<div class='cg' style='border-color:" + cor + "'>";
        c += "<div class='cr'>";
        c += F("<span class='cn'>&#9889; Inv. Alta</span>");
        c += "<span class='cs' style='color:" + cor + "'>" + txt + "</span>";
        c += "</div><div class='ci'>";
        c += "Sinal: ";
        c += sinalCasa.ativo ? "<span style='color:#43a047'>OK</span>"
                             : "<span style='color:#546e7a'>N</span>";
        c += " | Bat >=";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvAlta);
        c += String(tempBuf) + "% | Religue: ";
        c += monInvAlta.falhaDefinitiva ? "FALHOU" :
             monInvAlta.emReligue       ? "EM CURSO" : "OK";
        c += "</div></div>";
    }
    c += F("</div>");

    // ── Previsão Solar ───────────────────────────────────────
    if (cfgPrevisao.ativa && cfgPrevisao.configurada && previsao.valido) {
        float ahAtualH  = getAhPrevHoraAtual();
        float ahRestDia = getAhPrevRestanteDia();
        c += F("<div class='pd'>");
        c += F("<div class='dt'>&#127774; Previsao Solar</div>");
        c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
        char bP[24];
        snprintf(bP, sizeof(bP), "%.1f Ah", ahAtualH);
        c += F("<div style='text-align:center'>"
               "<div style='font-size:10px;color:#546e7a'>Esta hora</div>");
        c += "<div class='dv'>" + String(bP) + "</div></div>";
        snprintf(bP, sizeof(bP), "%.1f Ah", ahRestDia);
        c += F("<div style='text-align:center'>"
               "<div style='font-size:10px;color:#546e7a'>Restante hoje</div>");
        c += "<div class='dv'>" + String(bP) + "</div></div>";
        c += F("</div>");
        c += "<div class='ds'><a href='/previsao' style='color:#43a047'>"
             "Ver previsao completa</a></div>";
        c += F("</div>");
    }

    // ── SVG Fluxo ────────────────────────────────────────────
    c += F("<h3 style='text-align:center;margin:8px 0 6px'>&#9889; Fluxo</h3>");
    {
        bool fPB = painelGer && batCarg;
        bool fPC = painelGer && (sensores.correnteCasa > 0.3f);
        bool fBC = batDesc   && (sensores.correnteCasa > 0.3f);
        String lPB = fPB ? "#43a047" : "#333";
        String lPC = fPC ? "#fdd835" : "#333";
        String lBC = fBC ? "#ef6c00" : "#333";

        c += "<svg viewBox='0 0 340 210' "
             "style='width:100%;height:auto;display:block'>";
        c += "<path d='M110,50 C160,20 200,20 240,50' fill='none' stroke='"
             + lPB + "' stroke-width='3'/>";
        c += "<path d='M90,85 C80,120 90,140 130,160' fill='none' stroke='"
             + lPC + "' stroke-width='3'/>";
        c += "<path d='M255,85 C260,120 250,140 210,160' fill='none' stroke='"
             + lBC + "' stroke-width='3'/>";

        // Painel
        c += "<g><rect x='20' y='10' width='100' height='80' rx='12' "
             "fill='#181818' stroke='" + cPainel + "' stroke-width='2'/>";
        c += "<text x='70' y='36' text-anchor='middle' font-size='20'>&#9728;</text>";
        c += "<text x='70' y='50' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>";
        c += "<text x='70' y='65' text-anchor='middle' font-size='13' fill='"
             + cPainel + "' font-weight='bold'>";
        c += painelGer ? String(sIP) : "0.0A";
        c += "</text><text x='70' y='80' text-anchor='middle' font-size='11' fill='"
             + cPainel + "'>";
        c += painelGer ? String(sWP) : "0W";
        c += "</text></g>";

        // Bateria
        String icoBat = batCarg ? "&#9650;&#128267;" :
                        batDesc ? "&#9660;&#128267;" : "&#128267;";
        c += "<g><rect x='220' y='10' width='100' height='80' rx='12' "
             "fill='#181818' stroke='" + cBat + "' stroke-width='2'/>";
        c += "<text x='270' y='34' text-anchor='middle' font-size='18'>"
             + icoBat + "</text>";
        c += "<text x='270' y='48' text-anchor='middle' font-size='9' "
             "fill='#78909c'>BATERIA</text>";
        c += "<text x='270' y='63' text-anchor='middle' font-size='13' fill='"
             + cBat + "' font-weight='bold'>";
        c += bateria.calibrada ? String(pctBuf) : "N/Cal";
        c += "</text><text x='270' y='78' text-anchor='middle' font-size='11' fill='"
             + cBat + "'>";
        {
            String si = batCarg ? "+" : batDesc ? "-" : "~";
            c += si + String(sIB) + "/" + String(sWB);
        }
        c += "</text></g>";

        // Casa
        String corCasa = monInvAlta.statusOK && estados.invAlta  ? "#fdd835" :
                         monInvBaixa.statusOK && estados.invBaixa ? "#4fc3f7" :
                         "#3a3a3a";
        // Borda vermelha se falha definitiva
        if (monInvBaixa.falhaDefinitiva || monInvAlta.falhaDefinitiva)
            corCasa = "#c62828";
        c += "<g><rect x='105' y='158' width='130' height='52' rx='10' "
             "fill='#181818' stroke='" + corCasa + "' stroke-width='2'/>";
        c += "<text x='125' y='190' font-size='18'>&#127968;</text>";
        c += "<text x='228' y='175' text-anchor='end' font-size='9' fill='#78909c'>CASA</text>";
        c += "<text x='228' y='190' text-anchor='end' font-size='13' fill='"
             + corCasa + "' font-weight='bold'>" + String(sIC) + "</text>";
        c += "<text x='228' y='203' text-anchor='end' font-size='11' fill='"
             + corCasa + "'>" + String(sWC) + "</text></g>";
        c += F("</svg>");
    }

    // ── Barra Bateria ────────────────────────────────────────
    c += F("<div style='margin:8px 0'>");
    c += F("<div style='display:flex;justify-content:space-between;"
           "align-items:center;margin-bottom:4px'>");
    c += F("<span style='font-size:11px;color:#78909c'>&#128267; Bateria</span>");
    if (tempo.valido) {
        char bT[50];
        snprintf(bT, sizeof(bT), "%s %dh%02dm",
                 tempo.carregando ? "&#9650;" : "&#9660;",
                 tempo.horas, tempo.minutos);
        String corT = tempo.carregando ? "#43a047" : "#ef6c00";
        c += "<span style='font-size:11px;font-weight:bold;color:"
             + corT + "'>" + String(bT) + "</span>";
    }
    c += F("</div>");
    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual);
        char ahBuf[40];
        snprintf(ahBuf, sizeof(ahBuf), "%.1fAh de %.1fAh",
                 bateria.ahAtual, bateria.ahDisponivel);
        c += "<div style='font-size:11px;color:#546e7a;text-align:center;"
             "margin-top:2px'>" + String(ahBuf) + "</div>";
    } else {
        c += F("<div class='wn' style='font-size:11px;text-align:center'>"
               "Nao calibrada</div>");
    }
    c += F("</div>");

    // ── Grid correntes ───────────────────────────────────────
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
           "gap:6px;margin:4px 0'>");
    c += F("<div class='sc' style='padding:8px;text-align:center'>"
           "<div class='lbl'>&#9728; Painel</div>");
    c += "<div style='font-size:15px;font-weight:bold;color:"
         + cPainel + "' id='cp'>";
    c += painelGer ? String(sIP) : "0.0A";
    c += "</div><div style='font-size:11px;color:#78909c' id='wp'>";
    c += painelGer ? String(sWP) : "0W";
    c += F("</div></div>");

    c += F("<div class='sc' style='padding:8px;text-align:center'>"
           "<div class='lbl'>&#127968; Casa</div>"
           "<div style='font-size:15px;font-weight:bold;color:#4fc3f7' id='cc'>");
    c += String(sIC);
    c += F("</div><div style='font-size:11px;color:#78909c' id='wc'>");
    c += String(sWC);
    c += F("</div></div>");

    {
        String cf = batCarg ? "#43a047" : batDesc ? "#ef6c00" : "#fdd835";
        String si = batCarg ? "+" : batDesc ? "-" : "~";
        c += F("<div class='sc' style='padding:8px;text-align:center'>"
               "<div class='lbl'>&#128267; Bat</div>");
        c += "<div style='font-size:15px;font-weight:bold;color:"
             + cf + "' id='cb'>";
        c += si + String(sIB);
        c += F("</div><div style='font-size:11px;color:#78909c' id='wb'>");
        c += String(sWB);
        c += F("</div></div>");
    }
    c += F("</div>");

    // ── Info rede ────────────────────────────────────────────
    String ip = modoAP ? WiFi.softAPIP().toString()
                       : WiFi.localIP().toString();
    c += "<div class='nf' style='font-size:10px'>";
    c += "&#128246; " + (modoAP ? String("AP") : String("WiFi")) + " | " + ip;
    if (!modoAP && WiFi.status() == WL_CONNECTED)
        c += " | " + String(WiFi.RSSI()) + "dBm";
    snprintf(tempBuf, sizeof(tempBuf), " | RAM %.1fKB | Up %lus",
             esp_get_free_heap_size() / 1024.0f, millis() / 1000UL);
    c += String(tempBuf) + "</div>";

    // ── AJAX ─────────────────────────────────────────────────
    c += F("<script>var vn=");
    c += String(bateria.tensaoNominal, 1);
    c += F(";"
           "function aD(){"
           "var x=new XMLHttpRequest();"
           "x.onreadystatechange=function(){"
           "if(x.readyState==4&&x.status==200){"
           "try{"
           "var d=JSON.parse(x.responseText);"
           "var ip=parseFloat(d.ip||0);"
           "var ic=parseFloat(d.ic||0);"
           "var ib=parseFloat(d.ib||0);"
           "var wp=Math.round(ip*vn);"
           "var wc=Math.round(ic*vn);"
           "var wb=Math.round(Math.abs(ib)*vn);"
           "var e=document.getElementById('cp');"
           "if(e)e.textContent=ip>0.5?ip.toFixed(1)+'A':'0.0A';"
           "e=document.getElementById('wp');"
           "if(e)e.textContent=ip>0.5?wp+'W':'0W';"
           "e=document.getElementById('cc');"
           "if(e)e.textContent=ic.toFixed(1)+'A';"
           "e=document.getElementById('wc');"
           "if(e)e.textContent=wc+'W';"
           "e=document.getElementById('cb');"
           "if(e){var s=ib>0.3?'+':ib<-0.3?'-':'~';"
           "e.textContent=s+Math.abs(ib).toFixed(1)+'A';}"
           "e=document.getElementById('wb');"
           "if(e)e.textContent=wb+'W';"
           "}catch(ex){}}}"
           ";x.open('GET','/dados',true);x.send();}"
           "setInterval(aD,2000);aD();</script>");

    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Painel", c));
}

// ═══════════════════════════════════════════════════════════
// [DADOS] JSON API
// ═══════════════════════════════════════════════════════════
void handleDados() {
    if (!logado) { server.send(403); return; }
    char buf[512];
    snprintf(buf, sizeof(buf),
             "{\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
             "\"wp\":%.0f,\"wc\":%.0f,\"wb\":%.0f,"
             "\"pct\":%.1f,\"ah_at\":%.2f,\"ah_disp\":%.2f,"
             "\"vnom\":%.1f,\"calib\":%s,"
             "\"invb\":%s,\"inva\":%s,\"le\":%s,"
             "\"ntp\":%s,\"prev_ok\":%s,"
             "\"sinal_mv\":%.0f,\"sinal_ativo\":%s,"
             "\"sinal_sem_bat\":%s,"
             "\"st_invb_mv\":%.0f,\"st_invb_ok\":%s,"
             "\"st_invb_rel\":%s,\"st_invb_fal\":%s,"
             "\"st_inva_mv\":%.0f,\"st_inva_ok\":%s,"
             "\"st_inva_rel\":%s,\"st_inva_fal\":%s}",
             sensores.correntePainel,   sensores.correnteCasa,
             sensores.correnteBateria,
             sensores.potenciaPainel,   sensores.potenciaCasa,
             fabsf(sensores.potenciaBateria),
             bateria.percentualAtual,   bateria.ahAtual,
             bateria.ahDisponivel,      bateria.tensaoNominal,
             bateria.calibrada         ? "true" : "false",
             estados.invBaixa          ? "true" : "false",
             estados.invAlta           ? "true" : "false",
             estados.lampExt           ? "true" : "false",
             ntpSincronizado           ? "true" : "false",
             previsao.valido           ? "true" : "false",
             sinalCasa.mediaMV,
             sinalCasa.ativo           ? "true" : "false",
             sinalCasa.semEnergia      ? "true" : "false",
             monInvBaixa.mediaMV,
             monInvBaixa.statusOK      ? "true" : "false",
             monInvBaixa.emReligue     ? "true" : "false",
             monInvBaixa.falhaDefinitiva ? "true" : "false",
             monInvAlta.mediaMV,
             monInvAlta.statusOK       ? "true" : "false",
             monInvAlta.emReligue      ? "true" : "false",
             monInvAlta.falhaDefinitiva  ? "true" : "false");
    server.send(200, "application/json", buf);
}

// ═══════════════════════════════════════════════════════════
// [REDES-WEB] CONTROLE E LIMIARES
// ═══════════════════════════════════════════════════════════
void handleRedes() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "limiares") {
            float lb = server.arg("limInvBaixa").toFloat();
            float la = server.arg("limInvAlta").toFloat();
            float ll = server.arg("limLamp").toFloat();
            if (lb < 0 || lb > 95) msg = "<div class='er'>&#10007; Inv.Baixa 0-95%</div>";
            else if (la < 0 || la > 95) msg = "<div class='er'>&#10007; Inv.Alta 0-95%</div>";
            else if (ll < 0 || ll > 95) msg = "<div class='er'>&#10007; Lamp 0-95%</div>";
            else {
                limiares.pctInvBaixa = lb;
                limiares.pctInvAlta  = la;
                limiares.pctLampExt  = ll;
                salvarLimiares();
                adicionarLog("Limiares salvos");
                msg = "<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao == "rele_baixa") {
            bool ligar = (server.arg("valor") == "1");
            if (ligar) {
                ligarInvBaixa();
                // Reseta monitor ao ligar manualmente
                monInvBaixa.estado        = MON_NORMAL;
                monInvBaixa.tentativas    = 0;
                monInvBaixa.statusOK      = false;
                monInvBaixa.emReligue     = false;
                monInvBaixa.falhaDefinitiva = false;
                monInvBaixa.tInicioEstado = millis();
            } else { desligarInvBaixa(); }
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual InvBaixa %s", ligar ? "ON" : "OFF");
            adicionarLog(tempBuf);
            msg = "<div class='ok'>&#10003; OK</div>";
        } else if (acao == "rele_alta") {
            bool ligar = (server.arg("valor") == "1");
            if (ligar) {
                ligarInvAlta();
                monInvAlta.estado        = MON_NORMAL;
                monInvAlta.tentativas    = 0;
                monInvAlta.statusOK      = false;
                monInvAlta.emReligue     = false;
                monInvAlta.falhaDefinitiva = false;
                monInvAlta.tInicioEstado = millis();
            } else { desligarInvAlta(); }
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual InvAlta %s", ligar ? "ON" : "OFF");
            adicionarLog(tempBuf);
            msg = "<div class='ok'>&#10003; OK</div>";
        } else if (acao == "rele_lamp") {
            bool ligar = (server.arg("valor") == "1");
            ligar ? ligarLampExt() : desligarLampExt();
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual LampExt %s", ligar ? "ON" : "OFF");
            adicionarLog(tempBuf);
            msg = "<div class='ok'>&#10003; OK</div>";
        } else if (acao == "reset_mon_baixa") {
            // Reseta falha definitiva do inv. baixa
            monInvBaixa.falhaDefinitiva = false;
            monInvBaixa.tentativas      = 0;
            monInvBaixa.estado          = MON_NORMAL;
            monInvBaixa.emReligue       = false;
            adicionarLog("Mon InvBaixa resetado");
            msg = "<div class='ok'>&#10003; Monitor InvBaixa resetado</div>";
        } else if (acao == "reset_mon_alta") {
            monInvAlta.falhaDefinitiva  = false;
            monInvAlta.tentativas       = 0;
            monInvAlta.estado           = MON_NORMAL;
            monInvAlta.emReligue        = false;
            adicionarLog("Mon InvAlta resetado");
            msg = "<div class='ok'>&#10003; Monitor InvAlta resetado</div>";
        }
    }

    String c = msg;

    // Monitor inversores detalhado
    c += F("<h3>&#128202; Monitor de Status</h3>");
    c += cardMonitor("Inv. Baixa Potencia", "&#128268;",
                     monInvBaixa, estados.invBaixa);
    c += cardMonitor("Inv. Alta Potencia",  "&#9889;",
                     monInvAlta,  estados.invAlta && sinalCasaAtivo());

    // Botões reset monitor
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
    c += F("<form method='POST' action='/redes'>"
           "<input type='hidden' name='acao' value='reset_mon_baixa'>"
           "<button type='submit' class='btn-o btn-sm'>"
           "&#8635; Reset Mon. Baixa</button></form>");
    c += F("<form method='POST' action='/redes'>"
           "<input type='hidden' name='acao' value='reset_mon_alta'>"
           "<button type='submit' class='btn-o btn-sm'>"
           "&#8635; Reset Mon. Alta</button></form>");
    c += F("</div>");

    // Sinal casa
    c += F("<h3>&#128246; Sinal Alta Potencia</h3>");
    {
        String corS = sinalCasa.ativo       ? "#43a047" :
                      sinalCasa.aguardandoLigar ? "#f9a825" : "#546e7a";
        String txtS = sinalCasa.ativo       ? "&#9679; Ativo" :
                      sinalCasa.aguardandoLigar ? "&#9650; Aguardando" :
                      sinalCasa.aguardandoDesligar ? "&#9660; Encerrando" :
                      "&#9675; Inativo";
        char mvB[20];
        snprintf(mvB, sizeof(mvB), "%.0f mV", sinalCasa.mediaMV);
        c += "<div class='cg' style='border-color:" + corS + "'>";
        c += "<div class='cr'>";
        c += F("<span class='cn'>&#128246; Sinal Casa</span>");
        c += "<span class='cs' style='color:" + corS + "'>" + txtS + "</span></div>";
        c += "<div class='ci'>Tensao: " + String(mvB)
             + " | Limiar: " + String(SINAL_CASA_LIMIAR_MV) + "mV"
             + " | Confirm: 5s</div></div>";
        if (sinalCasa.semEnergia)
            c += F("<div class='wn' style='font-size:11px'>"
                   "&#9888; Sinal ativo mas bateria insuficiente!</div>");
    }

    c += F("<h3>&#9889; Status Atual</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px'>");
    auto cardSt = [&](const char* ico, const char* nm,
                       bool on, float lim, bool corte) {
        String cor = on ? "#43a047" : "#546e7a";
        c += "<div class='sc' style='text-align:center;padding:8px'>";
        c += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
        c += "<div style='font-size:13px;font-weight:bold;color:" + cor + "'>";
        c += on ? "Ligado" : "Aguard";
        c += "</div><div style='font-size:10px;color:#546e7a'>";
        char lb[22];
        snprintf(lb, sizeof(lb), corte ? "Corte:%.0f%%" : "Lim:%.0f%%", lim);
        c += String(lb) + "</div></div>";
    };
    cardSt("&#128268;", "Inv.Baixa", estados.invBaixa, limiares.pctInvBaixa, false);
    cardSt("&#9889;",   "Inv.Alta",  estados.invAlta,  limiares.pctInvAlta,  false);
    cardSt("&#128161;", "Lamp.Ext",  estados.lampExt,  limiares.pctLampExt,  true);
    c += F("</div>");

    if (bateria.calibrada) {
        char bB[32];
        snprintf(bB, sizeof(bB), "Bateria: %.0f%% (%.1fAh)",
                 bateria.percentualAtual, bateria.ahAtual);
        c += "<div style='margin:8px 0'>"
             + barraProgresso(bateria.percentualAtual, String(bB)) + "</div>";
    }

    c += F("<h3>&#128268; Controle Manual</h3>");
    c += F("<div class='nf' style='font-size:11px'>"
           "Controle manual reseta a falha definitiva do monitor.</div>");

    auto ctrlM = [&](const char* ico, const char* nm,
                      const char* acStr, bool on) {
        String cor = on ? "#43a047" : "#78909c";
        c += "<div class='cg'><div class='cr'>";
        c += "<span class='cn'>" + String(ico) + " " + String(nm) + "</span>";
        c += "<span class='cs' style='color:" + cor + "'>";
        c += on ? "&#9679; ON" : "&#9675; OFF";
        c += "</span></div>";
        c += F("<div style='display:flex;gap:6px;margin-top:8px'>");
        c += "<form method='POST' action='/redes' style='flex:1'>"
             "<input type='hidden' name='acao' value='" + String(acStr) + "'>"
             "<input type='hidden' name='valor' value='1'>"
             "<button type='submit' class='btn-g btn-sm'>ON</button></form>";
        c += "<form method='POST' action='/redes' style='flex:1'>"
             "<input type='hidden' name='acao' value='" + String(acStr) + "'>"
             "<input type='hidden' name='valor' value='0'>"
             "<button type='submit' class='btn-r btn-sm'>OFF</button></form>";
        c += F("</div></div>");
    };

    ctrlM("&#128268;", "Inv. Baixa", "rele_baixa", estados.invBaixa);
    ctrlM("&#9889;",   "Inv. Alta",  "rele_alta",  estados.invAlta);
    ctrlM("&#128161;", "Lamp. Ext",  "rele_lamp",  estados.lampExt);

    c += F("<h3>&#9881; Limiares</h3>");
    c += F("<div class='nf'>"
           "<b>Inv. Baixa:</b> Liga se bat &gt;= X%<br>"
           "<b>Inv. Alta:</b> Sinal confirmado E bat &gt;= X%<br>"
           "<b>Lamp. Ext:</b> Desliga se bat &lt;= X%</div>");
    c += F("<form method='POST' action='/redes'>"
           "<input type='hidden' name='acao' value='limiares'>"
           "<label>% Minimo Inv. Baixa:</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvBaixa);
    c += "<input type='number' name='limInvBaixa' min='0' max='95' step='1' "
         "required value='" + String(tempBuf) + "'>";
    c += F("<label>% Minimo Inv. Alta:</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvAlta);
    c += "<input type='number' name='limInvAlta' min='0' max='95' step='1' "
         "required value='" + String(tempBuf) + "'>";
    c += F("<label>% Corte Lamp. Ext:</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctLampExt);
    c += "<input type='number' name='limLamp' min='0' max='95' step='1' "
         "required value='" + String(tempBuf) + "'>";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");

    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Controle", c));
}

// ═══════════════════════════════════════════════════════════
// [SENS-WEB] SENSORES
// ═══════════════════════════════════════════════════════════
void handleSensores() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "calib_painel") {
            float iReal = server.arg("corrente_real").toFloat();
            if (iReal < 0 || iReal > 50)
                msg = "<div class='er'>&#10007; 0-50A</div>";
            else {
                int novoOff = (int)((float)sensores.sinalPainel_mV
                                    + iReal * calibPainel.sensMvA);
                calibPainel.offsetMV = novoOff;
                prefs.begin("rrfs_cal", false);
                prefs.putInt("pn_off", novoOff); prefs.end();
                adicionarLog("Calib painel OK");
                msg = "<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao == "calib_casa") {
            float iReal = server.arg("corrente_real").toFloat();
            if (iReal < 0 || iReal > 100)
                msg = "<div class='er'>&#10007; 0-100A</div>";
            else {
                int novoOff = (int)((float)sensores.sinalCasa_mV
                                    - iReal * calibCasa.sensMvA);
                calibCasa.offsetMV = novoOff;
                prefs.begin("rrfs_cal", false);
                prefs.putInt("casa_off", novoOff); prefs.end();
                adicionarLog("Calib casa OK");
                msg = "<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao == "reset_calib") {
            prefs.begin("rrfs_cal", false);
            prefs.remove("pn_off");   prefs.remove("pn_sens");
            prefs.remove("casa_off"); prefs.remove("casa_sens");
            prefs.end();
            carregarCalibSensores();
            adicionarLog("Calib reset");
            msg = "<div class='ok'>&#10003; Resetado</div>";
        }
    }

    String c = msg;
    c += F("<h3>&#128270; Leituras Corrente</h3>");

    auto cardSensor = [&](const char* ico, const char* nm,
                           float corr, float pot, int mv,
                           int off, float sens) {
        c += "<div class='sc'><div class='lbl'>"
             + String(ico) + " " + String(nm) + "</div>";
        char buf[24];
        snprintf(buf, sizeof(buf), "%.2fA", corr);
        c += "<div class='val'>" + String(buf) + "</div>";
        snprintf(buf, sizeof(buf), "%.0fW", pot);
        c += "<div style='font-size:13px;color:#78909c'>" + String(buf) + "</div>";
        snprintf(buf, sizeof(buf), "ADC %dmV | Off %d | Sens %.1f",
                 mv, off, sens);
        c += "<div class='sub'>" + String(buf) + "</div></div>";
    };

    cardSensor("&#9728;",   "Painel",
               sensores.correntePainel,  sensores.potenciaPainel,
               sensores.sinalPainel_mV,  calibPainel.offsetMV,
               calibPainel.sensMvA);
    cardSensor("&#127968;", "Casa",
               sensores.correnteCasa,    sensores.potenciaCasa,
               sensores.sinalCasa_mV,    calibCasa.offsetMV,
               calibCasa.sensMvA);
    {
        String siBat = sensores.correnteBateria >  0.3f ? "+" :
                       sensores.correnteBateria < -0.3f ? "-" : "~";
        c += F("<div class='sc'><div class='lbl'>&#128267; Bateria (calculada)</div>");
        char buf[24];
        snprintf(buf, sizeof(buf), "%s%.2fA",
                 siBat.c_str(), fabsf(sensores.correnteBateria));
        c += "<div class='val'>" + String(buf) + "</div>";
        snprintf(buf, sizeof(buf), "%.0fW", fabsf(sensores.potenciaBateria));
        c += "<div style='font-size:13px;color:#78909c'>" + String(buf) + "</div>";
        c += F("<div class='sub'>Painel - Casa</div></div>");
    }

    // Sinais digitais (sinal casa + status inversores)
    c += F("<h3>&#128246; Sinais de Controle</h3>");

    auto cardSinal = [&](const char* ico, const char* nm,
                          float mediaMV, bool ativo, uint8_t pin) {
        String cor = ativo ? "#43a047" : "#546e7a";
        char mvB[20];
        snprintf(mvB, sizeof(mvB), "%.0f mV", mediaMV);
        c += "<div class='sc'><div class='lbl'>"
             + String(ico) + " " + String(nm) + " (GPIO"
             + String(pin) + ")</div>";
        c += "<div class='val' style='color:" + cor + "'>";
        c += ativo ? "ATIVO" : "INATIVO";
        c += "</div><div class='sub'>Media movel: " + String(mvB)
             + " | Limiar: 2000mV | MM: 10 amostras | Confirm: 5s</div></div>";
    };

    cardSinal("&#128246;", "Sinal Casa Alta",
              sinalCasa.mediaMV, sinalCasa.ativo, PIN_SINAL_CASA);
    cardSinal("&#128268;", "Status Inv. Baixa",
              monInvBaixa.mediaMV, monInvBaixa.statusOK,
              PIN_STATUS_INV_BAIXA);
    cardSinal("&#9889;",   "Status Inv. Alta",
              monInvAlta.mediaMV, monInvAlta.statusOK,
              PIN_STATUS_INV_ALTA);

    c += F("<h3>&#9728; Calibrar Painel</h3>"
           "<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_painel'>"
           "<label>Corrente real (A):</label>"
           "<input type='number' name='corrente_real' step='0.01' "
           "min='0' max='50' required placeholder='Ex: 8.5'>"
           "<button type='submit' class='btn-g'>Calibrar</button></form>"
           "<h3>&#127968; Calibrar Casa</h3>"
           "<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_casa'>"
           "<label>Corrente real (A):</label>"
           "<input type='number' name='corrente_real' step='0.01' "
           "min='0' max='100' required placeholder='Ex: 12.3'>"
           "<button type='submit' class='btn-g'>Calibrar</button></form>"
           "<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='reset_calib'>"
           "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
           "&#8635; Reset Fabrica</button></form>");

    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Sensores", c));
}

// ═══════════════════════════════════════════════════════════
// [BATERIA / PREVISAO / HISTORICO / CONFIG — inalterados v21]
// Reutiliza exatamente os mesmos handlers da v20
// ═══════════════════════════════════════════════════════════
void handleBateria() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config") {
            float vn = server.arg("vnom").toFloat();
            float ah = server.arg("ah").toFloat();
            float dc = server.arg("desc").toFloat();
            if (vn < 6 || vn > 60)        msg = "<div class='er'>Tensao 6-60V</div>";
            else if (ah < 1 || ah > 5000) msg = "<div class='er'>Cap 1-5000Ah</div>";
            else if (dc < 10 || dc > 100) msg = "<div class='er'>Desc 10-100%</div>";
            else {
                bateria.tensaoNominal  = vn;
                bateria.capacidadeAh   = ah;
                bateria.descargaSegura = dc;
                calcularBateria(); salvarConfigBateria();
                adicionarLog("Bat config salva");
                msg = "<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao == "calibrar") {
            float pBMS = server.arg("pct_atual").toFloat();
            if (pBMS < 0 || pBMS > 100)
                msg = "<div class='er'>0-100%</div>";
            else {
                float ahFis = bateria.capacidadeAh * (pBMS / 100.0f);
                float ahRes = bateria.capacidadeAh *
                              (1.0f - bateria.descargaSegura / 100.0f);
                float ahUt  = ahFis - ahRes;
                if (ahUt < 0) ahUt = 0;
                if (ahUt > bateria.ahDisponivel) ahUt = bateria.ahDisponivel;
                bateria.ahAtual         = ahUt;
                bateria.percentualAtual = (bateria.ahDisponivel > 0)
                    ? (ahUt / bateria.ahDisponivel) * 100.0f : 0.0f;
                bateria.calibrada = true;
                ultimoContagem    = millis();
                salvarConfigBateria();
                adicionarLog("Bat calibrada");
                msg = "<div class='ok'>&#10003; Calibrada!</div>";
            }
        }
    }
    String c = msg;
    c += F("<h3>&#128267; Status</h3>");
    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual);
        char buf[80];
        snprintf(buf, sizeof(buf), "%.1fAh / %.1fAh (%.0f%%)",
                 bateria.ahAtual, bateria.ahDisponivel, bateria.percentualAtual);
        c += "<div class='nf' style='text-align:center'>" + String(buf) + "</div>";
        c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px'>");
        char b2[24];
        snprintf(b2, sizeof(b2), "%.1f Ah", bateria.ahAtual);
        c += F("<div class='sc' style='text-align:center'><div class='lbl'>Ah Atual</div>");
        c += "<div class='val' style='font-size:16px;color:#43a047'>" + String(b2) + "</div></div>";
        snprintf(b2, sizeof(b2), "%.1f Ah", bateria.ahDisponivel);
        c += F("<div class='sc' style='text-align:center'><div class='lbl'>Ah Disponivel</div>");
        c += "<div class='val' style='font-size:16px;color:#4fc3f7'>" + String(b2) + "</div></div>";
        snprintf(b2, sizeof(b2), "%.0f W", fabsf(sensores.potenciaBateria));
        c += F("<div class='sc' style='text-align:center'><div class='lbl'>Potencia</div>");
        String corP = sensores.correnteBateria > 0.3f ? "#43a047" :
                      sensores.correnteBateria < -0.3f ? "#ef6c00" : "#fdd835";
        c += "<div class='val' style='font-size:16px;color:" + corP + "'>" + String(b2) + "</div></div>";
        snprintf(b2, sizeof(b2), "%.1f V", bateria.tensaoNominal);
        c += F("<div class='sc' style='text-align:center'><div class='lbl'>Tensao Nominal</div>");
        c += "<div class='val' style='font-size:16px;color:#90a4ae'>" + String(b2) + "</div></div>";
        c += F("</div>");
        EstimativaTempo tempo = calcularTempoRestante();
        if (tempo.valido) {
            String corT = tempo.carregando ? "#43a047" : "#ef6c00";
            char bT[40];
            snprintf(bT, sizeof(bT), "%dh %02dm", tempo.horas, tempo.minutos);
            c += "<div class='nf' style='text-align:center'>";
            c += tempo.carregando ? "Completa em: " : "Esgota em: ";
            c += "<b style='color:" + corT + "'>" + String(bT) + "</b></div>";
        }
    } else {
        c += F("<div class='wn'>Aguardando calibracao</div>");
    }
    c += F("<h3>&#9881; Configurar</h3>"
           "<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='config'>"
           "<label>Tensao Nominal (V):</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.1f", bateria.tensaoNominal);
    c += "<input type='number' name='vnom' step='0.1' min='6' max='60' "
         "required value='" + String(tempBuf) + "'>";
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", bateria.capacidadeAh);
    c += F("<label>Capacidade (Ah):</label>");
    c += "<input type='number' name='ah' step='1' min='1' max='5000' "
         "required value='" + String(tempBuf) + "'>";
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", bateria.descargaSegura);
    c += F("<label>Descarga Segura (%):</label>");
    c += "<input type='number' name='desc' step='1' min='10' max='100' "
         "required value='" + String(tempBuf) + "'>";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>"
           "<h3>&#127919; Calibrar</h3>"
           "<div class='nf'>Informe o % atual do BMS/Display</div>"
           "<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='calibrar'>"
           "<label>% no Display/BMS:</label>"
           "<input type='number' name='pct_atual' step='1' "
           "min='0' max='100' required placeholder='Ex: 75'>"
           "<button type='submit' class='btn-o'>&#127919; Calibrar</button></form>");
    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Bateria", c));
}

void handlePrevisao() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config_prev") {
            float lat = server.arg("lat").toFloat();
            float lon = server.arg("lon").toFloat();
            float pkw = server.arg("painel_kw").toFloat();
            float ctA = server.arg("ctrl_a").toFloat();
            bool  ativ = (server.arg("ativa") == "1");
            if (pkw<=0||pkw>100) msg="<div class='er'>Potencia 0.1-100kW</div>";
            else if(ctA<=0||ctA>500) msg="<div class='er'>Corrente 1-500A</div>";
            else if(lat<-90||lat>90) msg="<div class='er'>Latitude invalida</div>";
            else if(lon<-180||lon>180) msg="<div class='er'>Longitude invalida</div>";
            else {
                cfgPrevisao.latitude=lat; cfgPrevisao.longitude=lon;
                cfgPrevisao.painelKW=pkw; cfgPrevisao.controladorA=ctA;
                cfgPrevisao.ativa=ativ; cfgPrevisao.configurada=true;
                cfgPrevisao.fatorCalib=1.0f;
                salvarConfigPrevisao(); adicionarLog("Prev config salva");
                if(ativ&&WiFi.status()==WL_CONNECTED){previsao.valido=false;tUltimaPrevisao=0;}
                msg="<div class='ok'>&#10003; Configurado!</div>";
            }
        } else if(acao=="buscar_agora"){
            previsao.valido=false; tUltimaPrevisao=0;
            msg=buscarPrevisaoOpenMeteo()
                ?"<div class='ok'>&#10003; Atualizado!</div>"
                :"<div class='er'>&#10007; Falha. Verifique WiFi.</div>";
        } else if(acao=="reset_calib_prev"){
            cfgPrevisao.fatorCalib=1.0f; salvarConfigPrevisao();
            if(previsao.valido) calcularAhPrevisto();
            adicionarLog("Calib prev reset");
            msg="<div class='ok'>&#10003; Reset OK</div>";
        }
    }
    String c = msg;
    c += F("<h3>&#127758; Localizacao</h3>"
           "<div class='nf'>1. Abrir Google Maps<br>"
           "2. Navegar ate sua localizacao<br>"
           "3. Clique direito — copie lat,lon</div>"
           "<a href='https://maps.google.com' target='_blank'>"
           "<button type='button' class='btn-s' style='margin-bottom:8px'>"
           "&#127759; Abrir Google Maps</button></a>"
           "<h3>&#9881; Configurar</h3>"
           "<form method='POST' action='/previsao'>"
           "<input type='hidden' name='acao' value='config_prev'>"
           "<label>Latitude:</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.6f",cfgPrevisao.latitude);
    c+="<input type='number' name='lat' step='0.000001' min='-90' max='90' "
       "required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.6f",cfgPrevisao.longitude);
    c+=F("<label>Longitude:</label>");
    c+="<input type='number' name='lon' step='0.000001' min='-180' max='180' "
       "required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.2f",cfgPrevisao.painelKW);
    c+=F("<label>Potencia Painel (kW):</label>");
    c+="<input type='number' name='painel_kw' step='0.01' min='0.1' max='100' "
       "required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",cfgPrevisao.controladorA);
    c+=F("<label>Corrente Max Controlador (A):</label>");
    c+="<input type='number' name='ctrl_a' step='1' min='1' max='500' "
       "required value='"+String(tempBuf)+"'>";
    c+=F("<label>Ativar:</label><select name='ativa'>");
    c+=cfgPrevisao.ativa
       ?F("<option value='1' selected>Sim</option><option value='0'>Nao</option>")
       :F("<option value='1'>Sim</option><option value='0' selected>Nao</option>");
    c+=F("</select><button type='submit' class='btn-g'>&#128190; Salvar</button></form>"
         "<h3>&#127774; Status</h3>");
    if(!cfgPrevisao.configurada){
        c+=F("<div class='wn'>Nao configurada</div>");
    } else if(!cfgPrevisao.ativa){
        c+=F("<div class='wn'>Desativada</div>");
    } else if(!previsao.valido){
        c+=F("<div class='wn'>Aguardando dados...</div>"
             "<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='buscar_agora'>"
             "<button type='submit' class='btn-o btn-sm'>&#8635; Buscar</button></form>");
    } else {
        char bF[32]; snprintf(bF,sizeof(bF),"Fator: %.3f",cfgPrevisao.fatorCalib);
        c+=F("<div class='sc'><div class='lbl'>Calibracao</div>");
        c+="<div class='val' style='font-size:14px'>"+String(bF)+"</div>";
        c+=F("<div class='sub'>Ajuste automatico pelo sensor</div></div>"
             "<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='buscar_agora'>"
             "<button type='submit' class='btn-o btn-sm'>&#8635; Atualizar</button></form>"
             "<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='reset_calib_prev'>"
             "<button type='submit' class='btn-s btn-sm' style='margin-top:4px'>"
             "&#8635; Reset Calib</button></form>");
        int idx=getIndiceHoraAtual();
        float ahH=getAhPrevHoraAtual(), ahR=getAhPrevRestanteDia();
        c+=F("<h3>&#9200; Previsao Atual</h3>"
             "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
        char bV[24];
        snprintf(bV,sizeof(bV),"%.2f Ah",ahH);
        c+=F("<div class='pd'><div class='dt'>Esta hora</div>");
        c+="<div class='dv'>"+String(bV)+"</div>";
        if(idx>=0){snprintf(bV,sizeof(bV),"%.0f W/m2",previsao.radiacao[idx]);
        c+="<div class='ds'>"+String(bV)+"</div>";}
        c+=F("</div>");
        snprintf(bV,sizeof(bV),"%.2f Ah",ahR);
        c+=F("<div class='pd'><div class='dt'>Restante hoje</div>");
        c+="<div class='dv'>"+String(bV)+"</div>";
        c+=F("<div class='ds'>Ate meia-noite</div></div></div>");
        c+=F("<h3>&#128197; Previsao 7 Dias</h3>");
        struct tm tA; bool tT=obterTempoAtual(tA);
        for(int d=0;d<PREV_DIAS;d++){
            String nd;
            if(d==0) nd="Hoje";
            else if(d==1) nd="Amanha";
            else if(tT){struct tm td=tA;td.tm_mday+=d;mktime(&td);
            char db[12];snprintf(db,sizeof(db),"%02d/%02d",td.tm_mday,td.tm_mon+1);nd=String(db);}
            else nd="Dia+"+String(d);
            char vB[20]; snprintf(vB,sizeof(vB),"%.1f Ah",previsao.ahDia[d]);
            float mx=0; for(int x=0;x<PREV_DIAS;x++) if(previsao.ahDia[x]>mx) mx=previsao.ahDia[x];
            float pt=mx>0?(previsao.ahDia[d]/mx)*100.0f:0;
            c+="<div style='margin:4px 0'>"
               "<div style='display:flex;justify-content:space-between;font-size:11px;margin-bottom:2px'>"
               "<span style='color:#90a4ae'>"+nd+"</span>"
               "<span style='color:#fdd835;font-weight:bold'>"+String(vB)+"</span></div>"
               "<div class='bb'><div class='bv' style='width:"+String((int)pt)+"%;"
               "background:"+(d==0?"#0277bd":"#1b5e20")+"'></div></div></div>";
        }
    }
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Previsao",c));
}

void handleHistorico() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String c = "";
    c += F("<h3>&#128200; Totais Acumulados</h3>"
           "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
    auto cT=[&](const char* ico,const char* nm,float ah,const char* cor){
        char buf[20]; snprintf(buf,sizeof(buf),"%.1fAh",ah);
        c+="<div class='sc' style='text-align:center'><div class='lbl'>"
           +String(ico)+" "+String(nm)+"</div>"
           "<div class='val' style='color:"+String(cor)+"'>"+String(buf)+"</div></div>";
    };
    cT("&#9728;","Painel",contadores.painelAh,"#fdd835");
    cT("&#127968;","Casa",contadores.casaAh,"#4fc3f7");
    cT("&#9650;","Bat+",contadores.batCargAh,"#43a047");
    cT("&#9660;","Bat-",contadores.batDescAh,"#ef6c00");
    c+=F("</div><h3>&#128200; Ultimos 7 Dias</h3>");
    int maxD=histTotalDias<7?histTotalDias:7;
    if(maxD==0){ c+=F("<div class='nf'>Sem historico</div>"); }
    else {
        c+=F("<div style='overflow-x:auto'><table style='width:100%;"
             "border-collapse:collapse;font-size:11px'>"
             "<tr style='color:#546e7a'>"
             "<th style='text-align:left;padding:4px'>Data</th>"
             "<th style='padding:4px'>Painel</th>"
             "<th style='padding:4px'>Casa</th></tr>");
        for(int i=0;i<maxD;i++){
            RegistroDia dT;
            bool ok=(i==0)?(dT=histDiaAtual,true)
                :carregarDiaFlash(((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,dT);
            if(!ok||!dT.valido) continue;
            char bD[16]; snprintf(bD,sizeof(bD),"%02d/%02d/%04d",dT.dia,dT.mes,dT.ano);
            char buf[10];
            c+="<tr style='border-top:1px solid #222'>"
               "<td style='padding:4px'>"+String(bD)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalPainelAh);
            c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalCasaAh);
            c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td></tr>";
        }
        c+=F("</table></div>");
    }
    c+=F("<h3>&#128200; Tempo Real (60s)</h3>");
    int total=histSnapFull?HIST_SNAP_PONTOS:histSnapIdx;
    if(total<2){ c+=F("<div class='nf'>Aguardando...</div>"); }
    else {
        float maxC=0.5f;
        for(int i=0;i<total;i++){
            int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
            if(histCorrPainel[idx]>maxC) maxC=histCorrPainel[idx];
            if(histCorrCasa[idx]>maxC)   maxC=histCorrCasa[idx];
            if(fabsf(histCorrBat[idx])>maxC) maxC=fabsf(histCorrBat[idx]);
        }
        c+="<div style='height:80px;background:#111;border:1px solid #333;border-radius:4px'>"
           "<svg viewBox='0 0 "+String(total)+" 80' preserveAspectRatio='none' "
           "style='width:100%;height:100%'>";
        auto py=[&](float v)->float{ return 78.0f-(v/maxC)*76.0f; };
        String pP="M",pC="M";
        for(int i=0;i<total;i++){
            int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
            char pt[20];
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrPainel[idx]));
            if(i>0) pP+="L"; pP+=String(pt);
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrCasa[idx]));
            if(i>0) pC+="L"; pC+=String(pt);
        }
        c+="<path d='"+pP+"' fill='none' stroke='#fdd835' stroke-width='2'/>"
           "<path d='"+pC+"' fill='none' stroke='#4fc3f7' stroke-width='2'/>";
        c+=F("</svg></div>"
             "<div style='font-size:10px;color:#546e7a;margin-top:4px'>"
             "<span style='color:#fdd835'>&#9644;</span> Painel "
             "<span style='color:#4fc3f7;margin-left:8px'>&#9644;</span> Casa</div>");
    }
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Historico",c));
}

void handleConfig() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "senha") {
            prefs.begin("rrfs",true);
            String usr=prefs.getString("user",""); prefs.end();
            String pA=server.arg("pass_atual"),pN=server.arg("pass_nova"),
                   pC=server.arg("pass_conf");
            if(!verificarLogin(usr,pA)) msg="<div class='er'>Senha atual incorreta</div>";
            else if(pN.length()<4) msg="<div class='er'>Min 4 chars</div>";
            else if(pN!=pC) msg="<div class='er'>Senhas nao conferem</div>";
            else { salvarCredenciais(usr,pN); adicionarLog("Senha alterada");
                   msg="<div class='ok'>&#10003; Alterada!</div>"; }
        } else if(acao=="tempo"){
            int fh=server.arg("fuso_h").toInt(), fm=server.arg("fuso_m").toInt();
            bool ntp=(server.arg("ntp_ativo")=="1");
            if(fh<-12||fh>14) msg="<div class='er'>Fuso -12 a +14</div>";
            else {
                cfgTempo.fusoHoras=fh; cfgTempo.fusoMinutos=fm; cfgTempo.ntpAtivo=ntp;
                if(!ntp){
                    cfgTempo.manualAnno=server.arg("man_ano").toInt();
                    cfgTempo.manualMes=server.arg("man_mes").toInt();
                    cfgTempo.manualDia=server.arg("man_dia").toInt();
                    cfgTempo.manualHora=server.arg("man_hora").toInt();
                    cfgTempo.manualMin=server.arg("man_min").toInt();
                }
                salvarConfigTempo(); adicionarLog("Tempo salvo");
                if(ntp&&WiFi.status()==WL_CONNECTED){ntpSincronizado=false;sincronizarNTP();}
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if(acao=="reset_historico"){
            if(server.arg("confirmar")!="SIM"){
                msg="<div class='er'>Confirmacao invalida</div>";
            } else {
                contadores={0,0,0,0}; salvarContadores();
                memset(&acum,0,sizeof(AcumHora));
                acum.horaAtual=255; acum.ultimoAcum=0;
                RegistroDia dV; memset(&dV,0,sizeof(RegistroDia));
                for(uint16_t i=0;i<HIST_DIAS;i++) salvarDiaFlash(i,dV);
                histIndiceAtual=0; histTotalDias=0;
                memset(&histDiaAtual,0,sizeof(RegistroDia));
                salvarIndiceHistorico();
                memset(histCorrPainel,0,sizeof(histCorrPainel));
                memset(histCorrCasa,0,sizeof(histCorrCasa));
                memset(histCorrBat,0,sizeof(histCorrBat));
                histSnapIdx=0; histSnapFull=false;
                adicionarLog("HIST apagado");
                msg="<div class='ok'>&#10003; Zerado!</div>";
            }
        }
    }
    String c=msg;
    c+=F("<h3>&#128202; Sistema v21</h3><div class='sc'>");
    char bS[100];
    snprintf(bS,sizeof(bS),"v21 | RAM %.1fKB | Up %lus",
             esp_get_free_heap_size()/1024.0f,millis()/1000UL);
    c+="<div class='val' style='font-size:12px'>"+String(bS)+"</div>";
    c+="<div class='sub'>"+(modoAP?String("AP 10.0.0.1"):String("WiFi ")+WiFi.localIP().toString())+"</div>";
    c+="<div class='sub'>&#128336; "+formatarDataHora();
    if(ntpSincronizado) c+=F(" <span style='color:#43a047'>NTP OK</span>");
    c+=F("</div></div>");
    c+=F("<div class='nf'><b>Logica v21:</b><br>"
         "Inv.Baixa: bat &gt;= limiar<br>"
         "Inv.Alta: sinal casa(5s) E bat &gt;= limiar<br>"
         "Monitor: falha 5s → desliga 5s → religa → aguarda 5s<br>"
         "Max tentativas: 3 | MM status: 10 amostras</div>");
    c+=F("<h3>&#128336; Data e Hora</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='tempo'>"
         "<label>Fuso (h, ex: -3 Brasilia):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.fusoHoras);
    c+="<input type='number' name='fuso_h' min='-12' max='14' step='1' "
       "required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.fusoMinutos);
    c+=F("<label>Minutos fuso:</label>");
    c+="<input type='number' name='fuso_m' min='0' max='59' step='1' value='"
       +String(tempBuf)+"'>";
    c+=F("<label>NTP:</label><select name='ntp_ativo' id='ntpSel' onchange='toggleNTP()'>");
    c+=cfgTempo.ntpAtivo
       ?F("<option value='1' selected>Sim</option><option value='0'>Nao-Manual</option>")
       :F("<option value='1'>Sim</option><option value='0' selected>Nao-Manual</option>");
    c+=F("</select>");
    c+="<div id='manualTime' style='display:";
    c+=cfgTempo.ntpAtivo?"none":"block";
    c+=F("'>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualAnno);
    c+=F("<label>Ano:</label>");
    c+="<input type='number' name='man_ano' min='2024' max='2099' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualMes);
    c+=F("<label>Mes:</label>");
    c+="<input type='number' name='man_mes' min='1' max='12' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualDia);
    c+=F("<label>Dia:</label>");
    c+="<input type='number' name='man_dia' min='1' max='31' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualHora);
    c+=F("<label>Hora:</label>");
    c+="<input type='number' name='man_hora' min='0' max='23' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualMin);
    c+=F("<label>Minuto:</label>");
    c+="<input type='number' name='man_min' min='0' max='59' value='"+String(tempBuf)+"'>";
    c+=F("</div><button type='submit' class='btn-g'>&#128190; Salvar Hora</button></form>"
         "<script>function toggleNTP(){"
         "document.getElementById('manualTime').style.display="
         "document.getElementById('ntpSel').value=='1'?'none':'block';}"
         "</script>"
         "<h3>&#128274; Senha</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='senha'>"
         "<label>Atual:</label><input type='password' name='pass_atual' required>"
         "<label>Nova:</label><input type='password' name='pass_nova' required>"
         "<label>Confirmar:</label><input type='password' name='pass_conf' required>"
         "<button type='submit' class='btn-o'>Alterar</button></form>"
         "<h3>&#128465; Limpar Historico</h3>"
         "<form method='POST' action='/config' "
         "onsubmit='return confirm(\"Apagar permanentemente?\")'>"
         "<input type='hidden' name='acao' value='reset_historico'>"
         "<input type='hidden' name='confirmar' value='SIM'>"
         "<button type='submit' class='btn-r'>Apagar</button></form>"
         "<h3>&#9881; Acoes</h3>"
         "<form method='GET' action='/logs-salvos'>"
         "<button type='submit' class='btn-s btn-sm'>&#128221; Logs</button></form>"
         "<form method='GET' action='/setup-wifi'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
         "&#128246; WiFi</button></form>"
         "<form method='GET' action='/ota'>"
         "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>"
         "&#128640; OTA</button></form>"
         "<form method='GET' action='/logout'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
         "&#128275; Logout</button></form>"
         "<form method='POST' action='/reiniciar'>"
         "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
         "&#8635; Reiniciar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Config",c));
}

// ═══════════════════════════════════════════════════════════
// [AUX-WEB] AUXILIARES
// ═══════════════════════════════════════════════════════════
void handleLogs() {
    if (!logsCarregados) carregarLogsPersistentes();
    String json = "[";
    bool primeiro = true;
    uint16_t inicio = totalLogs > 50 ? totalLogs - 50 : 0;
    for (uint16_t num = inicio; num < totalLogs; num++) {
        uint8_t idx = num % MAX_LOGS;
        if (logs[idx][0] == '\0') continue;
        if (!primeiro) json += ",";
        primeiro = false;
        json += "\"";
        for (int ch = 0; logs[idx][ch] && ch < LOG_MAX_LEN; ch++) {
            char cc = logs[idx][ch];
            if (cc == '"')       json += "\\\"";
            else if (cc == '\\') json += "\\\\";
            else if (cc == '\n') json += " ";
            else if (cc == '\r') continue;
            else                 json += cc;
        }
        json += "\"";
    }
    json += "]";
    server.send(200, "application/json", json);
}

void handleLogsSalvos() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    if (!logsCarregados) carregarLogsPersistentes();
    String c = F("<h3>&#128221; Logs Salvos</h3>");
    c += "<div class='nf'>Total: <b>" + String(totalLogs) + "</b></div>";
    c += F("<div class='lbx' style='height:400px'>");
    uint16_t inicio = totalLogs > MAX_LOGS ? totalLogs - MAX_LOGS : 0;
    for (uint16_t num = inicio; num < totalLogs; num++) {
        uint8_t idx = num % MAX_LOGS;
        if (logs[idx][0] == '\0') continue;
        c += "<div class='ll'>" + String(logs[idx]) + "</div>";
    }
    c += F("</div>"
           "<form method='GET' action='/config'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:8px'>"
           "&#8592; Voltar</button></form>");
    server.send(200, "text/html;charset=utf-8",
                gerarPagina("RRFS - Logs", c));
}

void handleReiniciar() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    adicionarLog("Reinicio usuario");
    server.send(200, "text/html;charset=utf-8",
                gerarPagina("Reiniciando",
                F("<div class='ok'>&#10003; Reiniciando em 3s...</div>"), false));
    delay(3000); ESP.restart();
}

void handleNotFound() {
    String uri = server.uri();
    if (uri.indexOf("generate_204") >= 0 || uri.indexOf("gen_204")     >= 0 ||
        uri.indexOf("hotspot")      >= 0 || uri.indexOf("ncsi.txt")    >= 0 ||
        uri.indexOf("connecttest")  >= 0) {
        if (modoAP) {
            server.sendHeader("Location", "http://10.0.0.1/");
            server.send(302, "text/plain", "");
        } else { server.send(204, "text/plain", ""); }
        return;
    }
    if (uri == "/favicon.ico") { server.send(204); return; }
    if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else server.send(404, "text/plain", "404");
}

void handleOTA() {
    if (!logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    String c = F("<h3>&#128640; Atualizar Firmware</h3>"
                 "<div class='nf'>Selecione o arquivo .bin</div>"
                 "<form method='POST' action='/ota-upload' "
                 "enctype='multipart/form-data' id='fOTA'>"
                 "<input type='file' id='fArq' name='firmware' accept='.bin' required "
                 "style='background:#111;border:1px solid #444;border-radius:4px;"
                 "padding:8px;color:#e0e0e0;width:100%;box-sizing:border-box'>"
                 "<div id='prog' style='display:none;margin-top:8px'>"
                 "<div class='bb'><div class='bv' id='pBar' "
                 "style='width:0%;background:#0277bd'>0%</div></div>"
                 "<div style='text-align:center;font-size:11px;color:#78909c' "
                 "id='pTxt'>Enviando...</div></div>"
                 "<button type='submit' class='btn-o' style='margin-top:10px' "
                 "id='btnOTA'>&#128640; Enviar</button></form>"
                 "<form method='GET' action='/config'>"
                 "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
                 "&#8592; Voltar</button></form>"
                 "<script>"
                 "document.getElementById('fOTA').onsubmit=function(e){"
                 "e.preventDefault();"
                 "var f=document.getElementById('fArq').files[0];"
                 "if(!f||!f.name.endsWith('.bin')){alert('Selecione .bin');return;}"
                 "var fd=new FormData();fd.append('firmware',f);"
                 "var x=new XMLHttpRequest();"
                 "document.getElementById('prog').style.display='block';"
                 "document.getElementById('btnOTA').disabled=true;"
                 "x.upload.onprogress=function(e){"
                 "if(e.lengthComputable){"
                 "var p=Math.round(e.loaded/e.total*100);"
                 "document.getElementById('pBar').style.width=p+'%';"
                 "document.getElementById('pBar').textContent=p+'%';}};"
                 "x.onreadystatechange=function(){"
                 "if(x.readyState==4){"
                 "if(x.status==200){"
                 "document.getElementById('pBar').style.background='#2e7d32';"
                 "document.getElementById('pTxt').textContent='OK! 5s...';"
                 "setTimeout(function(){location.href='/';},5000);"
                 "}else{"
                 "document.getElementById('pTxt').textContent='ERRO '+x.status;"
                 "document.getElementById('pBar').style.background='#c62828';"
                 "document.getElementById('btnOTA').disabled=false;}}};"
                 "x.open('POST','/ota-upload',true);x.send(fd);};"
                 "</script>");
    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - OTA", c));
}

void handleOTAUpload() {
    if (!logado) { server.send(403); return; }
    HTTPUpload& up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
        adicionarLog("OTA inicio");
        esp_task_wdt_reset();
        desligarInvAlta(); desligarInvBaixa();
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
            adicionarLog("OTA ERRO begin");
    } else if (up.status == UPLOAD_FILE_WRITE) {
        esp_task_wdt_reset();
        if (Update.write(up.buf, up.currentSize) != up.currentSize)
            adicionarLog("OTA ERRO write");
    } else if (up.status == UPLOAD_FILE_END) {
        esp_task_wdt_reset();
        if (Update.end(true)) {
            adicionarLog("OTA OK");
            server.send(200, "text/plain", "OK");
            delay(500); ESP.restart();
        } else {
            adicionarLog("OTA ERRO end");
            server.send(500, "text/plain", "ERRO");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [ROUTES] ROTAS
// ═══════════════════════════════════════════════════════════
void configurarRotas() {
    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/dados",       HTTP_GET,  handleDados);
    server.on("/bateria",     HTTP_ANY,  handleBateria);
    server.on("/redes",       HTTP_ANY,  handleRedes);
    server.on("/sensores",    HTTP_ANY,  handleSensores);
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
    server.on("/ota-upload",  HTTP_POST,
        []() {},
        handleOTAUpload);
    server.onNotFound(handleNotFound);
    server.begin();
    adicionarLog("Rotas OK v21");
}
