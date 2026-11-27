/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v18- DUAL INVERSOR
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 * MUDANÇAS v18:
 * - REMOVIDO:   PIN_STATUS_INV_ALTA  (monitoramento status inv. alta)
 * - REMOVIDO:   PIN_STATUS_INV_BAIXA (monitoramento status inv. baixa)
 * - REMOVIDO:   PIN_SINAL_LIGA_ALTA  (sinal da casa)
 * - REMOVIDO:   Lógica de religamento (sem monitoramento de status)
 * - REMOVIDO:   Estados complexos inversores (sem/sinal, religando)
 * - SIMPLIFICADO: Lógica inversores - liga/desliga por % bateria
 * - ADICIONADO: Potência estimada (W) em painel, casa e bateria
 * - ADICIONADO: Ah disponível na página bateria
 * - ADICIONADO: Amperagem e potência da bateria no fluxo
 * - MANTIDO:    Calibração manual persistente
 * - MANTIDO:    Lâmpada externa
 * - MANTIDO:    Média móvel correntes
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

// ═══════════════════════════════════════════════════════════
// [HW] PINOS
// ═══════════════════════════════════════════════════════════
#define LED_AZUL              2   // LED interno ESP32
#define BOTAO_BOOT            0   // Botão BOOT - reset 10s

// Sensores de corrente
#define PIN_SENSOR_PAINEL    34   // ACS758 50A - painel solar
#define PIN_SENSOR_CASA      35   // ACS758 100A - consumo casa

// Inversor ALTA potência (geladeira, forno, panela)
// LÓGICA INVERTIDA: LOW=ligado, HIGH=desligado
#define PIN_RELE_INV_ALTA    27

// Inversor BAIXA potência (tomadas, TV, notebook, lâmpadas)
#define PIN_RELE_INV_BAIXA   25

// Lâmpada externa
#define PIN_RELE_LAMP_EXT    14

// ═══════════════════════════════════════════════════════════
// [CFG] CONSTANTES
// ═══════════════════════════════════════════════════════════
#define DNS_PORT             53
#define TEMPO_RESET_MS    10000
#define MAX_LOGS            100
#define LOG_MAX_LEN          64
#define LOG_NVS_NS      "rrfs_log"

// ADC
#define ADC_SAMPLES         150
#define ADC_VREF           3300
#define ADC_BITS           4095

// Sensores corrente - valores padrão
#define PAINEL_OFFSET_MV   2500
#define PAINEL_SENS_MV_A     40
#define CASA_OFFSET_MV      600
#define CASA_SENS_MV_A       20
#define FILTRO_RUIDO_A      0.3f
#define BUFFER_CORRENTE      10

// Intervalos principais
#define INTERVALO_LEITURA_MS   1000
#define INTERVALO_CONTROLE_MS  5000

// ── Lâmpada externa ─────────────────────────────────────────
#define LAMP_EXT_DELAY_MS  300000UL  // 5min confirmação

// ── WiFi ────────────────────────────────────────────────────
#define WIFI_RECON_INTERVALO  30000UL
#define WIFI_RECON_TIMEOUT    10000UL
#define WIFI_CHECK_INTERVALO  15000UL
#define WIFI_MAX_FALHAS            5

// ── Histórico ───────────────────────────────────────────────
#define HIST_DIAS            30
#define HIST_HORAS           24
#define HIST_KEY_VER          7   // incrementado v18
#define HIST_SNAP_PONTOS     60

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
    float pctInvBaixa;    // % mínimo para inv. baixa (padrão 20%)
    float pctInvAlta;     // % mínimo para inv. alta  (padrão 40%)
    float pctLampExt;     // % corte lâmpada externa  (padrão 30%)
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
    float potenciaPainel;   // W estimado
    float potenciaCasa;     // W estimado
    float potenciaBateria;  // W estimado
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
    bool  ligada;
    unsigned long tUltimaMudanca;
    bool  estadoAnteriorPainel;
};

struct RegistroHora {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
};

struct RegistroDia {
    uint16_t    ano;
    uint8_t     mes, dia;
    bool        valido;
    RegistroHora horas[HIST_HORAS];
    float totalPainelAh, totalCasaAh;
    float totalBatCargAh, totalBatDescAh;
};

struct ContadoresGlobais {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
};

struct AcumHora {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
    uint8_t  horaAtual, diaAtual, mesAtual;
    uint16_t anoAtual;
    unsigned long ultimoAcum;
};

// ═══════════════════════════════════════════════════════════
// [VAR] VARIÁVEIS GLOBAIS
// ═══════════════════════════════════════════════════════════
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP = false;
bool logado = false;

unsigned long ultimoBlink = 0;
bool ledState = false, blinkAtivo = true;
unsigned long botaoPressionadoEm = 0;
bool botaoSegurando = false;

char     logs[MAX_LOGS][LOG_MAX_LEN];
uint16_t totalLogs    = 0;
uint8_t  logIndex     = 0;
bool     logsCarregados = false;
char     tempBuf[200];

ConfigBateria  bateria   = {12.8f, 100.0f, 80.0f, 80.0f, 0.0f, 0.0f, false};
ConfigLimiares limiares  = {20.0f, 40.0f, 30.0f};

DadosSensores  sensores  = {0};
CalibSensor    calibPainel, calibCasa;
BufferMM       bufPainel = {0}, bufCasa = {0};

EstadoReles  estados = {false, false, false};
EstadoLampExt lampExt = {false, 0, true};

unsigned long ultimaLeitura   = 0;
unsigned long ultimoControle  = 0;
unsigned long ultimoContagem  = 0;

String        gWifiSSID, gWifiPASS, gWifiIP, gWifiGW, gWifiMASK, gWifiDNS;
unsigned long tUltimoCheckWifi = 0;
unsigned long tUltimoReconWifi = 0;
uint8_t       wifiFalhasConsec = 0;
bool          wifiReconectando = false;
unsigned long tInicioRecon     = 0;

RegistroDia       histDiaAtual;
uint16_t          histIndiceAtual = 0, histTotalDias = 0;
ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora          acum = {0, 0, 0, 0, 255, 0, 0, 0, 0};

float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx  = 0;
bool    histSnapFull = false;
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

void setupLED(); void atualizarLED();
void ledBlink(); void ledFixo(bool v);
void setupBotao(); void verificarReset();

void setupReles();
void setReleInvAlta(bool ligar);
void setReleInvBaixa(bool ligar);
void setReleLampExt(bool ligar);
void ligarLampExt();    void desligarLampExt();
void ligarInvAlta();    void desligarInvAlta();
void ligarInvBaixa();   void desligarInvBaixa();

int   lerADC(uint8_t pin);
int   adcParaMv(int v);
float adicionarAmostra(BufferMM& buf, float nova);
float lerCorrentePainel();
float lerCorrenteCasa();
void  lerSensores();
void  calcularPotencias();

void calcularBateria();
void verificarCalibracaoAutomatica();
void atualizarAhBateria();

void calcularTotaisDia(RegistroDia& dia);
void salvarContadores(); void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void salvarHistorico(); void carregarHistorico();
void acumularEnergia();
void atualizarDiaAtual();

void controlarInvBaixa();
void controlarInvAlta();
void controlarLampExt();
void controlarEnergia();

bool existeCredenciais(); bool existeWiFiConfig();
void salvarCredenciais(const String& u, const String& p);
bool verificarLogin(const String& u, const String& p);
void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap);
void carregarWifi(String& ssid, String& pass, String& ip, String& gw,
                  String& mask, String& dns, bool& ap);
void salvarConfigBateria(); void carregarConfigBateria();
void salvarLimiares();      void carregarLimiares();
void carregarCalibSensores();

void iniciarAP();
bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns);
void gerenciarWiFi();

void configurarRotas();
String gerarPagina(const String& titulo, const String& conteudo, bool comNav = true);
String barraProgresso(float pct, const String& label = "");

void handleRoot();       void handleDados();
void handleBateria();    void handleRedes();
void handleSensores();   void handleConfig();
void handleHistorico();
void handleLogs();       void handleLogsSalvos();
void handleSetupLogin(); void handleSetupWifi();
void handleLogin();      void handleLogout();
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
    est.taxaA    = taxa;
    est.valido   = true;
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
        ledState = !ledState;
        digitalWrite(LED_AZUL, ledState);
    }
}

void ledBlink() { blinkAtivo = true; ultimoBlink = millis(); }
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
// Inv. Alta: lógica INVERTIDA (LOW=ligado, HIGH=desligado)
// Inv. Baixa e Lamp: lógica NORMAL (HIGH=ligado, LOW=desligado)
// ═══════════════════════════════════════════════════════════
void setupReles() {
    // Inv. Alta: HIGH no setup = desligado (lógica invertida)
    pinMode(PIN_RELE_INV_ALTA,   OUTPUT);
    digitalWrite(PIN_RELE_INV_ALTA, HIGH);

    // Inv. Baixa: LOW no setup = desligado (lógica normal)
    pinMode(PIN_RELE_INV_BAIXA,  OUTPUT);
    digitalWrite(PIN_RELE_INV_BAIXA, LOW);

    // Lâmpada: LOW no setup = desligada (lógica normal)
    pinMode(PIN_RELE_LAMP_EXT,   OUTPUT);
    digitalWrite(PIN_RELE_LAMP_EXT, LOW);

    estados = {false, false, false};
    adicionarLog("Reles: todos OFF");
}

// Inv. Alta - lógica invertida
void ligarInvAlta()   { digitalWrite(PIN_RELE_INV_ALTA, LOW);  estados.invAlta  = true;  }
void desligarInvAlta(){ digitalWrite(PIN_RELE_INV_ALTA, HIGH); estados.invAlta  = false; }

// Inv. Baixa - lógica normal
void ligarInvBaixa()   { digitalWrite(PIN_RELE_INV_BAIXA, HIGH); estados.invBaixa = true;  }
void desligarInvBaixa(){ digitalWrite(PIN_RELE_INV_BAIXA, LOW);  estados.invBaixa = false; }

// Lâmpada - lógica normal
void ligarLampExt()   { digitalWrite(PIN_RELE_LAMP_EXT, HIGH); estados.lampExt  = true;  }
void desligarLampExt(){ digitalWrite(PIN_RELE_LAMP_EXT, LOW);  estados.lampExt  = false; }

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

// Calcula potências estimadas (W = A x V_nominal)
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

    // Snapshot histórico tempo real
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
    unsigned long agora = millis();
    float deltaH = (float)(agora - ultimoContagem) / 3600000.0f;
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
        histIndiceAtual = 0;
        histTotalDias   = 0;
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
        adicionarLog("Hist reset v18");
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
    if (diaCache == 0) {
        diaCache = 1; mesCache = 1; anoCache = 2024;
    } else {
        diaCache++;
        if (diaCache > 28) {
            diaCache = 1; mesCache++;
            if (mesCache > 12) { mesCache = 1; anoCache++; }
        }
    }
}

void acumularEnergia() {
    atualizarDiaAtual();
    unsigned long agora = millis();
    uint8_t  hA = (agora / 3600000UL) % 24;
    uint8_t  dA = diaCache;
    uint8_t  mA = mesCache;
    uint16_t aA = anoCache;

    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum = agora;
        acum.horaAtual  = hA;
        acum.diaAtual   = dA;
        acum.mesAtual   = mA;
        acum.anoAtual   = aA;
        bool diaEx = false;
        if (histTotalDias > 0) {
            RegistroDia dT;
            if (carregarDiaFlash(histIndiceAtual, dT) &&
                dT.dia == dA && dT.mes == mA && dT.ano == aA) {
                histDiaAtual = dT;
                diaEx = true;
            }
        }
        if (!diaEx) {
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano = aA; histDiaAtual.mes = mA;
            histDiaAtual.dia = dA; histDiaAtual.valido = true;
            if (histTotalDias == 0) {
                histIndiceAtual = 0; histTotalDias = 1;
            } else {
                RegistroDia dT;
                if (carregarDiaFlash(histIndiceAtual, dT) &&
                    (dT.dia != dA || dT.mes != mA || dT.ano != aA)) {
                    histIndiceAtual = (histIndiceAtual+1) % HIST_DIAS;
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
            histIndiceAtual = (histIndiceAtual+1) % HIST_DIAS;
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
        if (mudouDia) {
            acum.diaAtual = dA;
            acum.mesAtual = mA;
            acum.anoAtual = aA;
        }
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
// Liga se bat% >= pctInvBaixa, desliga se bat% < pctInvBaixa
// ═══════════════════════════════════════════════════════════
void controlarInvBaixa() {
    if (!bateria.calibrada) return;
    float pct = bateria.percentualAtual;

    if (!estados.invBaixa && pct >= limiares.pctInvBaixa) {
        ligarInvBaixa();
        adicionarLog("InvBaixa ON");
    } else if (estados.invBaixa && pct < limiares.pctInvBaixa) {
        desligarInvBaixa();
        adicionarLog("InvBaixa OFF bat baixa");
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] INVERSOR ALTA POTÊNCIA
// Liga se bat% >= pctInvAlta, desliga se bat% < pctInvAlta
// ═══════════════════════════════════════════════════════════
void controlarInvAlta() {
    if (!bateria.calibrada) return;
    float pct = bateria.percentualAtual;

    if (!estados.invAlta && pct >= limiares.pctInvAlta) {
        ligarInvAlta();
        adicionarLog("InvAlta ON");
    } else if (estados.invAlta && pct < limiares.pctInvAlta) {
        desligarInvAlta();
        adicionarLog("InvAlta OFF bat baixa");
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] LÂMPADA EXTERNA
// Liga após 5min sem geração solar e inv.baixa ligado
// Desliga ao retomar geração ou bat <= pctLampExt
// ═══════════════════════════════════════════════════════════
void controlarLampExt() {
    unsigned long agora = millis();
    bool gerando = (sensores.correntePainel > 0.5f);

    if (gerando != lampExt.estadoAnteriorPainel) {
        lampExt.estadoAnteriorPainel = gerando;
        lampExt.tUltimaMudanca       = agora;
    }

    unsigned long decorrido = agora - lampExt.tUltimaMudanca;

    if (!gerando && !estados.lampExt) {
        if (decorrido >= LAMP_EXT_DELAY_MS) {
            if (estados.invBaixa) {
                if (!bateria.calibrada ||
                    bateria.percentualAtual > limiares.pctLampExt) {
                    ligarLampExt();
                    adicionarLog("LampExt ON noite");
                }
            }
        }
    } else if (gerando && estados.lampExt) {
        if (decorrido >= LAMP_EXT_DELAY_MS) {
            desligarLampExt();
            adicionarLog("LampExt OFF dia");
        }
    }

    // Corte por bateria baixa
    if (estados.lampExt && bateria.calibrada) {
        if (bateria.percentualAtual <= limiares.pctLampExt) {
            desligarLampExt();
            adicionarLog("LampExt OFF bat");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] CONTROLE GERAL
// ═══════════════════════════════════════════════════════════
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
    String su = prefs.getString("user","");
    String sp = prefs.getString("pass","");
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
    prefs.putFloat("lim_invb",  limiares.pctInvBaixa);
    prefs.putFloat("lim_inva",  limiares.pctInvAlta);
    prefs.putFloat("lim_lamp",  limiares.pctLampExt);
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
        calibPainel.offsetMV = prefs.getInt  ("pn_off",  calibPainel.offsetMV);
    if (prefs.isKey("pn_sens"))
        calibPainel.sensMvA  = prefs.getFloat("pn_sens", calibPainel.sensMvA);
    if (prefs.isKey("casa_off"))
        calibCasa.offsetMV   = prefs.getInt  ("casa_off",calibCasa.offsetMV);
    if (prefs.isKey("casa_sens"))
        calibCasa.sensMvA    = prefs.getFloat("casa_sens",calibCasa.sensMvA);
    prefs.end();
}

// ═══════════════════════════════════════════════════════════
// [WIFI] REDE
// ═══════════════════════════════════════════════════════════
void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP RRFS 10.0.0.1");
    logRAM();
    ledBlink();
}

bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns) {
    IPAddress lIP, lGW, lMask, lDNS;
    if (!lIP.fromString(ip))     return false;
    if (!lGW.fromString(gw))     return false;
    if (!lMask.fromString(mask)) return false;
    if (!lDNS.fromString(dns))   lDNS.fromString("8.8.8.8");

    gWifiSSID = ssid; gWifiPASS = pass;
    gWifiIP   = ip;   gWifiGW   = gw;
    gWifiMASK = mask; gWifiDNS  = dns;

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.config(lIP, lGW, lMask, lDNS);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    adicionarLog("WiFi conectando");
    ledBlink();
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        atualizarLED();
        esp_task_wdt_reset();
        delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
        adicionarLog("WiFi OK");
        modoAP = false;
        wifiFalhasConsec = 0;
        ledFixo(true);
        return true;
    }
    adicionarLog("WiFi FALHOU");
    return false;
}

void gerenciarWiFi() {
    unsigned long agora = millis();
    if (agora - tUltimoCheckWifi < WIFI_CHECK_INTERVALO) return;
    tUltimoCheckWifi = agora;
    if (gWifiSSID.isEmpty()) return;

    if (WiFi.status() == WL_CONNECTED) {
        wifiFalhasConsec = 0;
        wifiReconectando = false;
        tInicioRecon     = 0;
        return;
    }
    if (agora - tUltimoReconWifi < WIFI_RECON_INTERVALO) return;

    wifiFalhasConsec++;
    if (!wifiReconectando) {
        wifiReconectando = true;
        tInicioRecon     = agora;
        tUltimoReconWifi = agora;
        if (!gWifiIP.isEmpty()) {
            IPAddress lIP, lGW, lMask, lDNS;
            lIP.fromString(gWifiIP);
            lGW.fromString(gWifiGW);
            lMask.fromString(gWifiMASK);
            lDNS.fromString(gWifiDNS);
            WiFi.config(lIP, lGW, lMask, lDNS);
        }
        WiFi.reconnect();
        return;
    }
    if (WiFi.status() == WL_CONNECTED) {
        adicionarLog("WiFi reconectado");
        wifiFalhasConsec = 0;
        wifiReconectando = false;
        tInicioRecon     = 0;
        ledFixo(true);
        return;
    }
    if (agora - tInicioRecon >= WIFI_RECON_TIMEOUT) {
        wifiReconectando = false;
        tInicioRecon     = 0;
        tUltimoReconWifi = agora;
        if (wifiFalhasConsec >= WIFI_MAX_FALHAS) {
            adicionarLog("WiFi falhas AP temp");
            wifiFalhasConsec = 0;
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
            WiFi.softAP(AP_SSID);
            dnsServer.start(DNS_PORT, "*", AP_IP);
            modoAP = true;
            ledBlink();
            WiFi.begin(gWifiSSID.c_str(), gWifiPASS.c_str());
        }
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
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_reset_reason_t motivo = esp_reset_reason();
    switch (motivo) {
        case ESP_RST_BROWNOUT:  adicionarLog("RESET BROWNOUT"); break;
        case ESP_RST_PANIC:     adicionarLog("RESET PANIC");    break;
        case ESP_RST_TASK_WDT:  adicionarLog("RESET TASK WDT");break;
        case ESP_RST_INT_WDT:   adicionarLog("RESET INT WDT"); break;
        case ESP_RST_WDT:       adicionarLog("RESET WDT");     break;
        case ESP_RST_SW:        adicionarLog("Reset software"); break;
        case ESP_RST_POWERON:   adicionarLog("Power ON");      break;
        default:
            snprintf(tempBuf, sizeof(tempBuf), "Reset %d", (int)motivo);
            adicionarLog(tempBuf);
            break;
    }

    esp_task_wdt_reset();
    setupLED();
    setupBotao();
    setupReles();
    ledBlink();
    adicionarLog("RRFS v18");
    logRAM();

    esp_task_wdt_reset();
    carregarConfigBateria();
    carregarLimiares();
    carregarCalibSensores();
    carregarHistorico();
    carregarContadores();

    esp_task_wdt_reset();
    ultimoContagem = millis();

    bool usarAP = true;
    if (existeWiFiConfig()) {
        String ssid, pass, ip, gw, mask, dns;
        carregarWifi(ssid, pass, ip, gw, mask, dns, usarAP);
        esp_task_wdt_reset();
        if (!usarAP && ssid.length() > 0) {
            if (iniciarWiFiNB(ssid, pass, ip, gw, mask, dns))
                usarAP = false;
        }
    }

    esp_task_wdt_reset();
    if (usarAP) iniciarAP();

    esp_task_wdt_reset();
    configurarRotas();
    adicionarLog("Sistema pronto v18");
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

    // Leitura sensores 1s
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;
        esp_task_wdt_reset();
        lerSensores();
        acumularEnergia();
        esp_task_wdt_reset();
    }

    // Controle energia a cada 5s
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;
        controlarEnergia();
        esp_task_wdt_reset();
    }

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
)RAW";

// ═══════════════════════════════════════════════════════════
// [WEB] HELPERS
// ═══════════════════════════════════════════════════════════
String barraProgresso(float pct, const String& label) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    String cor = pct>=50?"#2e7d32":pct>=25?"#ef6c00":"#c62828";
    String h = "<div class='bb'><div class='bv' style='width:";
    h += String((int)pct) + "%;background:" + cor + "'>";
    h += String((int)pct) + "%</div></div>";
    if (label.length())
        h += "<div style='font-size:11px;color:#546e7a;text-align:center'>"
             + label + "</div>";
    return h;
}

String gerarPagina(const String& titulo, const String& conteudo, bool comNav) {
    String html;
    html.reserve(7000);
    html = F("<!DOCTYPE html><html><head>"
             "<meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>");
    html += CSS;
    html += F("</style></head><body><div class='card'>");
    html += F("<h2>&#9889; RRFS v18</h2>");
    html += conteudo;

    if (comNav && logado) {
        String pag = server.uri();
        html += F("<div class='nav'>");
        html += "<a href='/' "         + String(pag=="/"          ?"class='active'":"") + ">&#128202;</a>";
        html += "<a href='/bateria' "   + String(pag=="/bateria"   ?"class='active'":"") + ">&#128267;</a>";
        html += "<a href='/redes' "     + String(pag=="/redes"     ?"class='active'":"") + ">&#128268;</a>";
        html += "<a href='/sensores' "  + String(pag=="/sensores"  ?"class='active'":"") + ">&#128270;</a>";
        html += "<a href='/config' "    + String(pag=="/config"    ?"class='active'":"") + ">&#9881;</a>";
        html += "<a href='/historico' " + String(pag=="/historico" ?"class='active'":"") + ">&#128200;</a>";
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
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location","/setup-wifi"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String u=server.arg("user"), p=server.arg("pass"), c=server.arg("conf");
        if (u.length()<3)      msg="<div class='er'>&#10007; Min 3 chars usuario</div>";
        else if (p.length()<4) msg="<div class='er'>&#10007; Min 4 chars senha</div>";
        else if (p!=c)         msg="<div class='er'>&#10007; Senhas nao conferem</div>";
        else {
            salvarCredenciais(u,p);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location","/setup-wifi");
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
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Setup",c,false));
}

void handleSetupWifi() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login"); server.send(302); return;
    }
    if (existeWiFiConfig() && !logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String modo = server.arg("modo");
        if (modo == "ap") {
            salvarWifi("","","","","","",true);
            String ct = F("<div class='ok'>&#10003; Salvo! Reiniciando...</div>"
                          "<div class='nf'>SSID: <b>RRFS Energia</b> | IP: <b>10.0.0.1</b></div>");
            server.send(200,"text/html;charset=utf-8",gerarPagina("Salvo",ct,false));
            delay(2000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"), wp=server.arg("wpass"),
               ip=server.arg("ip"),     gw=server.arg("gw"),
               mask=server.arg("mask"), dns=server.arg("dns");
        if (ssid.isEmpty())   msg="<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7) msg="<div class='er'>Informe o IP</div>";
        else if (gw.length()<7) msg="<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wp,ip,gw,mask,dns,false);
            String ct = "<div class='ok'>&#10003; Salvo! Acesse: http://"+ip+"</div>";
            ct += F("<div class='nf'>Reiniciando...</div>");
            server.send(200,"text/html;charset=utf-8",gerarPagina("Salvo",ct,false));
            delay(2000); ESP.restart(); return;
        }
    }

    static int16_t nRedes = -1;
    static bool scanIniciado = false;
    if (!scanIniciado) {
        WiFi.scanDelete();
        WiFi.scanNetworks(true,false,false,300);
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
        for (int i=0;i<nRedes;i++) {
            String q = WiFi.RSSI(i)>-60?"Bom":WiFi.RSSI(i)>-75?"Regular":"Fraco";
            char buf[32];
            snprintf(buf,sizeof(buf)," [%s %ddBm]",q.c_str(),WiFi.RSSI(i));
            c += "<option value='"+WiFi.SSID(i)+"'>"+WiFi.SSID(i)+String(buf)+"</option>";
        }
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
    }
    c += F("</select>"
           "<button type='button' class='btn-o btn-sm' onclick='location.reload()'"
           " style='margin-top:8px'>&#128259; Atualizar</button>"
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
        c += F("<button onclick='location.href=\"/\"' class='btn-s' style='margin-top:8px'>"
               "&#8592; Voltar</button>");
    c += F("<script>function toggleModo(){"
           "document.getElementById('dw').style.display="
           "document.getElementById('modo').value=='wifi'?'block':'none';}</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Rede",c,false));
}

void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login"); server.send(302); return;
    }
    if (logado) {
        server.sendHeader("Location","/"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        if (verificarLogin(server.arg("user"),server.arg("pass"))) {
            logado = true;
            adicionarLog("Login OK"); logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        }
        adicionarLog("Login FALHOU");
        msg = F("<div class='er'>&#10007; Usuario/senha incorretos</div>");
    }
    String c = F("<div class='nf'>&#128274; Acesso seguro</div><h3>Entrar</h3>");
    c += msg;
    c += F("<form method='POST' action='/login'>"
           "<label>Usuario:</label><input type='text' name='user' required autocomplete='off'>"
           "<label>Senha:</label><input type='password' name='pass' required>"
           "<button type='submit'>&#128275; Entrar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Login",c,false));
}

void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado = false; }
    server.sendHeader("Location","/login"); server.send(302);
}

// ═══════════════════════════════════════════════════════════
// [DASH] DASHBOARD
// ═══════════════════════════════════════════════════════════
void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login"); server.send(302); return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location","/setup-wifi"); server.send(302); return;
    }
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }

    bool painelGer = (sensores.correntePainel  >  0.5f);
    bool batCarg   = (sensores.correnteBateria >  0.3f);
    bool batDesc   = (sensores.correnteBateria < -0.3f);

    // Buffers de texto reutilizados
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
    String cBat    = !bateria.calibrada ? "#455a64" :
                      bateria.percentualAtual>50  ? "#43a047" :
                      bateria.percentualAtual>25  ? "#ef6c00" : "#c62828";

    String c = "";

    if (!bateria.calibrada)
        c += F("<div class='wn' style='font-size:11px;margin-bottom:6px'>"
               "&#9888; Bateria nao calibrada. "
               "<a href='/bateria' style='color:#ffe0b2'>Calibrar</a></div>");

    // Status inversores
    c += F("<h3>&#9889; Inversores</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");

    // Card Inv. Baixa
    {
        String cor = estados.invBaixa ? "#43a047" : "#546e7a";
        String txt = estados.invBaixa ? "&#9679; Ligado" : "&#9675; Desligado";
        c += "<div class='cg' style='border-color:" + cor + "'>";
        c += "<div class='cr'>";
        c += F("<span class='cn'>&#128268; Inv. Baixa</span>");
        c += "<span class='cs' style='color:" + cor + "'>" + txt + "</span>";
        c += "</div>";
        c += "<div class='ci'>Limiar: >=";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvBaixa);
        c += String(tempBuf) + "%</div></div>";
    }

    // Card Inv. Alta
    {
        String cor = estados.invAlta ? "#43a047" : "#546e7a";
        String txt = estados.invAlta ? "&#9679; Ligado" : "&#9675; Desligado";
        c += "<div class='cg' style='border-color:" + cor + "'>";
        c += "<div class='cr'>";
        c += F("<span class='cn'>&#9889; Inv. Alta</span>");
        c += "<span class='cs' style='color:" + cor + "'>" + txt + "</span>";
        c += "</div>";
        c += "<div class='ci'>Limiar: >=";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvAlta);
        c += String(tempBuf) + "%</div></div>";
    }
    c += F("</div>");

    // SVG Fluxo de energia
    c += F("<h3 style='text-align:center;margin:8px 0 6px'>&#9889; Fluxo</h3>");
    {
        bool fPB = painelGer && batCarg;
        bool fPC = painelGer && (sensores.correnteCasa > 0.3f);
        bool fBC = batDesc   && (sensores.correnteCasa > 0.3f);
        String lPB = fPB?"#43a047":"#333";
        String lPC = fPC?"#fdd835":"#333";
        String lBC = fBC?"#ef6c00":"#333";

        c += "<svg viewBox='0 0 340 210' style='width:100%;height:auto;display:block'>";

        // Linhas de fluxo
        c += "<path d='M110,50 C160,20 200,20 240,50' fill='none' stroke='"+lPB+"' stroke-width='3'/>";
        c += "<path d='M90,85 C80,120 90,140 130,160' fill='none' stroke='"+lPC+"' stroke-width='3'/>";
        c += "<path d='M255,85 C260,120 250,140 210,160' fill='none' stroke='"+lBC+"' stroke-width='3'/>";

        // Painel solar
        c += "<g><rect x='20' y='10' width='100' height='80' rx='12' fill='#181818' stroke='"+cPainel+"' stroke-width='2'/>";
        c += "<text x='70' y='36' text-anchor='middle' font-size='20'>&#9728;</text>";
        c += "<text x='70' y='50' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>";
        c += "<text x='70' y='65' text-anchor='middle' font-size='13' fill='"+cPainel+"' font-weight='bold'>";
        c += painelGer ? String(sIP) : "0.0A";
        c += "</text>";
        c += "<text x='70' y='80' text-anchor='middle' font-size='11' fill='"+cPainel+"'>";
        c += painelGer ? String(sWP) : "0W";
        c += "</text></g>";

        // Bateria
        String icoBat = batCarg?"&#9650;&#128267;":batDesc?"&#9660;&#128267;":"&#128267;";
        c += "<g><rect x='220' y='10' width='100' height='80' rx='12' fill='#181818' stroke='"+cBat+"' stroke-width='2'/>";
        c += "<text x='270' y='34' text-anchor='middle' font-size='18'>"+icoBat+"</text>";
        c += "<text x='270' y='48' text-anchor='middle' font-size='9' fill='#78909c'>BATERIA</text>";
        c += "<text x='270' y='63' text-anchor='middle' font-size='13' fill='"+cBat+"' font-weight='bold'>";
        c += bateria.calibrada ? String(pctBuf) : "N/Cal";
        c += "</text>";
        // Amperagem bateria
        {
            String siBat = batCarg ? "+" : batDesc ? "-" : "~";
            c += "<text x='270' y='78' text-anchor='middle' font-size='11' fill='"+cBat+"'>";
            c += siBat + String(sIB) + " / " + String(sWB);
            c += "</text>";
        }
        c += "</g>";

        // Casa
        bool invBOK = estados.invBaixa;
        String corCasa = invBOK ? "#4fc3f7" : "#3a3a3a";
        c += "<g><rect x='105' y='158' width='130' height='52' rx='10' fill='#181818' stroke='"+corCasa+"' stroke-width='2'/>";
        c += "<text x='125' y='190' font-size='18'>&#127968;</text>";
        c += "<text x='228' y='175' text-anchor='end' font-size='9' fill='#78909c'>CASA</text>";
        c += "<text x='228' y='190' text-anchor='end' font-size='13' fill='"+corCasa+"' font-weight='bold'>";
        c += String(sIC);
        c += "</text>";
        c += "<text x='228' y='203' text-anchor='end' font-size='11' fill='"+corCasa+"'>";
        c += String(sWC);
        c += "</text></g>";

        c += F("</svg>");
    }

    // Barra bateria
    c += F("<div style='margin:8px 0'>");
    c += F("<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:4px'>");
    c += F("<span style='font-size:11px;color:#78909c'>&#128267; Bateria</span>");
    if (tempo.valido) {
        char bT[50];
        snprintf(bT, sizeof(bT), "%s %dh%02dm",
                 tempo.carregando?"&#9650;":"&#9660;",
                 tempo.horas, tempo.minutos);
        String corT = tempo.carregando?"#43a047":"#ef6c00";
        c += "<span style='font-size:11px;font-weight:bold;color:"+corT+"'>"+String(bT)+"</span>";
    }
    c += F("</div>");
    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual);
        // Ah atual / disponível
        char ahBuf[40];
        snprintf(ahBuf, sizeof(ahBuf), "%.1fAh de %.1fAh",
                 bateria.ahAtual, bateria.ahDisponivel);
        c += "<div style='font-size:11px;color:#546e7a;text-align:center;margin-top:2px'>";
        c += String(ahBuf) + "</div>";
    } else {
        c += F("<div class='wn' style='font-size:11px;text-align:center'>Nao calibrada</div>");
    }
    c += F("</div>");

    // Grid correntes + potências
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin:4px 0'>");

    // Painel
    c += F("<div class='sc' style='padding:8px;text-align:center'>");
    c += F("<div class='lbl'>&#9728; Painel</div>");
    c += "<div style='font-size:15px;font-weight:bold;color:"+cPainel+"' id='cp'>";
    c += painelGer ? String(sIP) : "0.0A";
    c += "</div>";
    c += "<div style='font-size:11px;color:#78909c' id='wp'>";
    c += painelGer ? String(sWP) : "0W";
    c += F("</div></div>");

    // Casa
    c += F("<div class='sc' style='padding:8px;text-align:center'>");
    c += F("<div class='lbl'>&#127968; Casa</div>");
    c += F("<div style='font-size:15px;font-weight:bold;color:#4fc3f7' id='cc'>");
    c += String(sIC);
    c += F("</div>");
    c += F("<div style='font-size:11px;color:#78909c' id='wc'>");
    c += String(sWC);
    c += F("</div></div>");

    // Bateria
    {
        String cf = batCarg?"#43a047":batDesc?"#ef6c00":"#fdd835";
        String si = batCarg?"+":batDesc?"-":"~";
        c += F("<div class='sc' style='padding:8px;text-align:center'>");
        c += F("<div class='lbl'>&#128267; Bat</div>");
        c += "<div style='font-size:15px;font-weight:bold;color:"+cf+"' id='cb'>";
        c += si + String(sIB);
        c += F("</div>");
        c += "<div style='font-size:11px;color:#78909c' id='wb'>";
        c += String(sWB);
        c += F("</div></div>");
    }
    c += F("</div>");

    // Info rede
    String ip = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    c += "<div class='nf' style='font-size:10px'>";
    c += "&#128246; " + (modoAP ? String("AP") : String("WiFi")) + " | " + ip;
    if (!modoAP && WiFi.status()==WL_CONNECTED)
        c += " | " + String(WiFi.RSSI()) + "dBm";
    snprintf(tempBuf, sizeof(tempBuf), " | RAM %.1fKB | Up %lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000UL);
    c += String(tempBuf) + "</div>";

    // AJAX auto-refresh
    c += F("<script>"
    "var vn=" );
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
    "var eCP=document.getElementById('cp');"
    "if(eCP)eCP.textContent=ip>0.5?ip.toFixed(1)+'A':'0.0A';"
    "var eWP=document.getElementById('wp');"
    "if(eWP)eWP.textContent=ip>0.5?wp+'W':'0W';"
    "var eCC=document.getElementById('cc');"
    "if(eCC)eCC.textContent=ic.toFixed(1)+'A';"
    "var eWC=document.getElementById('wc');"
    "if(eWC)eWC.textContent=wc+'W';"
    "var eCB=document.getElementById('cb');"
    "if(eCB){var s=ib>0.3?'+':ib<-0.3?'-':'~';"
    "eCB.textContent=s+Math.abs(ib).toFixed(1)+'A';}"
    "var eWB=document.getElementById('wb');"
    "if(eWB)eWB.textContent=wb+'W';"
    "}catch(e){}}}"
    ";x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();</script>");

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Painel", c));
}

// ═══════════════════════════════════════════════════════════
// [DADOS] JSON API
// ═══════════════════════════════════════════════════════════
void handleDados() {
    if (!logado) { server.send(403); return; }
    char buf[300];
    snprintf(buf, sizeof(buf),
             "{\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
             "\"wp\":%.0f,\"wc\":%.0f,\"wb\":%.0f,"
             "\"pct\":%.1f,\"ah_at\":%.2f,\"ah_disp\":%.2f,"
             "\"vnom\":%.1f,\"calib\":%s,"
             "\"invb\":%s,\"inva\":%s,\"le\":%s}",
             sensores.correntePainel, sensores.correnteCasa,
             sensores.correnteBateria,
             sensores.potenciaPainel, sensores.potenciaCasa,
             fabsf(sensores.potenciaBateria),
             bateria.percentualAtual, bateria.ahAtual, bateria.ahDisponivel,
             bateria.tensaoNominal,
             bateria.calibrada?"true":"false",
             estados.invBaixa?"true":"false",
             estados.invAlta?"true":"false",
             estados.lampExt?"true":"false");
    server.send(200, "application/json", buf);
}

// ═══════════════════════════════════════════════════════════
// [BAT-WEB] BATERIA
// ═══════════════════════════════════════════════════════════
void handleBateria() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config") {
            float vn=server.arg("vnom").toFloat();
            float ah=server.arg("ah").toFloat();
            float dc=server.arg("desc").toFloat();
            if (vn<6||vn>60)       msg="<div class='er'>&#10007; Tensao 6-60V</div>";
            else if (ah<1||ah>5000) msg="<div class='er'>&#10007; Cap. 1-5000Ah</div>";
            else if (dc<10||dc>100) msg="<div class='er'>&#10007; Desc. 10-100%</div>";
            else {
                bateria.tensaoNominal=vn;
                bateria.capacidadeAh=ah;
                bateria.descargaSegura=dc;
                calcularBateria();
                salvarConfigBateria();
                adicionarLog("Bat config salva");
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao == "calibrar") {
            float pBMS=server.arg("pct_atual").toFloat();
            if (pBMS<0||pBMS>100) msg="<div class='er'>&#10007; 0-100%</div>";
            else {
                float ahFis = bateria.capacidadeAh*(pBMS/100.0f);
                float ahRes = bateria.capacidadeAh*(1.0f-bateria.descargaSegura/100.0f);
                float ahUt  = ahFis-ahRes;
                if (ahUt<0) ahUt=0;
                if (ahUt>bateria.ahDisponivel) ahUt=bateria.ahDisponivel;
                bateria.ahAtual = ahUt;
                bateria.percentualAtual = (bateria.ahDisponivel>0)
                    ?(ahUt/bateria.ahDisponivel)*100.0f:0.0f;
                bateria.calibrada=true;
                ultimoContagem=millis();
                salvarConfigBateria();
                adicionarLog("Bat calibrada");
                msg="<div class='ok'>&#10003; Calibrada!</div>";
            }
        }
    }

    String c = msg;

    // Status com Ah disponível
    c += F("<h3>&#128267; Status</h3>");
    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual);
        char buf[80];
        snprintf(buf, sizeof(buf), "%.1fAh atual / %.1fAh disponivel (%.0f%%)",
                 bateria.ahAtual, bateria.ahDisponivel, bateria.percentualAtual);
        c += "<div class='nf' style='text-align:center'>"+String(buf)+"</div>";

        // Detalhes adicionais
        c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:8px'>");

        char b2[24];
        snprintf(b2, sizeof(b2), "%.1f Ah", bateria.ahAtual);
        c += F("<div class='sc' style='text-align:center'>");
        c += F("<div class='lbl'>Ah Atual</div>");
        c += "<div class='val' style='font-size:16px;color:#43a047'>"+String(b2)+"</div></div>";

        snprintf(b2, sizeof(b2), "%.1f Ah", bateria.ahDisponivel);
        c += F("<div class='sc' style='text-align:center'>");
        c += F("<div class='lbl'>Ah Disponivel</div>");
        c += "<div class='val' style='font-size:16px;color:#4fc3f7'>"+String(b2)+"</div></div>";

        snprintf(b2, sizeof(b2), "%.0f W", fabsf(sensores.potenciaBateria));
        c += F("<div class='sc' style='text-align:center'>");
        c += F("<div class='lbl'>Potencia Atual</div>");
        {
            String corP = sensores.correnteBateria>0.3f?"#43a047":
                          sensores.correnteBateria<-0.3f?"#ef6c00":"#fdd835";
            c += "<div class='val' style='font-size:16px;color:"+corP+"'>"+String(b2)+"</div>";
        }
        c += F("</div>");

        snprintf(b2, sizeof(b2), "%.1f V", bateria.tensaoNominal);
        c += F("<div class='sc' style='text-align:center'>");
        c += F("<div class='lbl'>Tensao Nominal</div>");
        c += "<div class='val' style='font-size:16px;color:#90a4ae'>"+String(b2)+"</div></div>";

        c += F("</div>");

        // Estimativa tempo
        EstimativaTempo tempo = calcularTempoRestante();
        if (tempo.valido) {
            String corT = tempo.carregando?"#43a047":"#ef6c00";
            String dirT = tempo.carregando?"Completa em":"Esgota em";
            char bT[40];
            snprintf(bT, sizeof(bT), "%dh %02dm", tempo.horas, tempo.minutos);
            c += "<div class='nf' style='text-align:center'>";
            c += dirT + ": <b style='color:"+corT+"'>"+String(bT)+"</b></div>";
        }
    } else {
        c += F("<div class='wn'>Aguardando calibracao</div>");
    }

    c += F("<h3>&#9881; Configurar</h3>"
           "<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='config'>"
           "<label>Tensao Nominal (V):</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.1f", bateria.tensaoNominal);
    c += "<input type='number' name='vnom' step='0.1' min='6' max='60' required value='"+String(tempBuf)+"'>";
    c += F("<label>Capacidade (Ah):</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", bateria.capacidadeAh);
    c += "<input type='number' name='ah' step='1' min='1' max='5000' required value='"+String(tempBuf)+"'>";
    c += F("<label>Descarga Segura (%):</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", bateria.descargaSegura);
    c += "<input type='number' name='desc' step='1' min='10' max='100' required value='"+String(tempBuf)+"'>";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");

    c += F("<h3>&#127919; Calibrar</h3>"
           "<div class='nf'>Informe o % do BMS/Display</div>"
           "<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='calibrar'>"
           "<label>% no Display/BMS:</label>"
           "<input type='number' name='pct_atual' step='1' min='0' max='100' required placeholder='Ex: 75'>"
           "<button type='submit' class='btn-o'>&#127919; Calibrar</button></form>");

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Bateria", c));
}

// ═══════════════════════════════════════════════════════════
// [REDES-WEB] CONTROLE E LIMIARES
// ═══════════════════════════════════════════════════════════
void handleRedes() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "limiares") {
            float lb = server.arg("limInvBaixa").toFloat();
            float la = server.arg("limInvAlta").toFloat();
            float ll = server.arg("limLamp").toFloat();
            if (lb<0||lb>95)  msg="<div class='er'>&#10007; Inv.Baixa 0-95%</div>";
            else if (la<0||la>95) msg="<div class='er'>&#10007; Inv.Alta 0-95%</div>";
            else if (ll<0||ll>95) msg="<div class='er'>&#10007; Lamp 0-95%</div>";
            else {
                limiares.pctInvBaixa = lb;
                limiares.pctInvAlta  = la;
                limiares.pctLampExt  = ll;
                salvarLimiares();
                adicionarLog("Limiares salvos");
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao == "rele_baixa") {
            bool ligar = (server.arg("valor")=="1");
            ligar ? ligarInvBaixa() : desligarInvBaixa();
            snprintf(tempBuf, sizeof(tempBuf), "Manual InvBaixa %s", ligar?"ON":"OFF");
            adicionarLog(tempBuf);
            msg="<div class='ok'>&#10003; OK</div>";
        } else if (acao == "rele_alta") {
            bool ligar = (server.arg("valor")=="1");
            ligar ? ligarInvAlta() : desligarInvAlta();
            snprintf(tempBuf, sizeof(tempBuf), "Manual InvAlta %s", ligar?"ON":"OFF");
            adicionarLog(tempBuf);
            msg="<div class='ok'>&#10003; OK</div>";
        } else if (acao == "rele_lamp") {
            bool ligar = (server.arg("valor")=="1");
            ligar ? ligarLampExt() : desligarLampExt();
            snprintf(tempBuf, sizeof(tempBuf), "Manual LampExt %s", ligar?"ON":"OFF");
            adicionarLog(tempBuf);
            msg="<div class='ok'>&#10003; OK</div>";
        }
    }

    String c = msg;

    // Status atual
    c += F("<h3>&#9889; Status Atual</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px'>");

    auto cardStatus = [&](const char* ico, const char* nm, bool on,
                           float limiar, bool isCorte) {
        String cor = on ? "#43a047" : "#546e7a";
        String txt = on ? "Ligado" : "Desligado";
        c += "<div class='sc' style='text-align:center;padding:8px'>";
        c += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
        c += "<div style='font-size:14px;font-weight:bold;color:" + cor + "'>" + txt + "</div>";
        char lB[20];
        snprintf(lB, sizeof(lB), isCorte?"Corte:<=%.0f%%":"Limiar:>=%.0f%%", limiar);
        c += "<div style='font-size:10px;color:#546e7a'>" + String(lB) + "</div>";
        c += "</div>";
    };

    cardStatus("&#128268;","Inv.Baixa", estados.invBaixa, limiares.pctInvBaixa, false);
    cardStatus("&#9889;",  "Inv.Alta",  estados.invAlta,  limiares.pctInvAlta,  false);
    cardStatus("&#128161;","Lamp.Ext",  estados.lampExt,  limiares.pctLampExt,  true);
    c += F("</div>");

    // Barra bateria resumida
    if (bateria.calibrada) {
        c += F("<div style='margin:8px 0'>");
        char bB[30];
        snprintf(bB, sizeof(bB), "Bateria: %.0f%% (%.1fAh)",
                 bateria.percentualAtual, bateria.ahAtual);
        c += barraProgresso(bateria.percentualAtual, String(bB));
        c += F("</div>");
    }

    // Controle manual
    c += F("<h3>&#128268; Controle Manual</h3>");

    auto ctrlManual = [&](const char* ico, const char* nm,
                           const char* acaoStr, bool on) {
        c += "<div class='cg'>";
        c += "<div class='cr'><span class='cn'>" + String(ico) + " " + String(nm) + "</span>";
        String cor = on ? "#43a047" : "#78909c";
        String txt = on ? "&#9679; ON" : "&#9675; OFF";
        c += "<span class='cs' style='color:" + cor + "'>" + txt + "</span></div>";
        c += F("<div style='display:flex;gap:6px;margin-top:8px'>");
        c += "<form method='POST' action='/redes' style='flex:1'>"
             "<input type='hidden' name='acao' value='" + String(acaoStr) + "'>"
             "<input type='hidden' name='valor' value='1'>"
             "<button type='submit' class='btn-g btn-sm'>ON</button></form>";
        c += "<form method='POST' action='/redes' style='flex:1'>"
             "<input type='hidden' name='acao' value='" + String(acaoStr) + "'>"
             "<input type='hidden' name='valor' value='0'>"
             "<button type='submit' class='btn-r btn-sm'>OFF</button></form>";
        c += F("</div></div>");
    };

    ctrlManual("&#128268;","Inv. Baixa Potencia","rele_baixa", estados.invBaixa);
    ctrlManual("&#9889;",  "Inv. Alta Potencia", "rele_alta",  estados.invAlta);
    ctrlManual("&#128161;","Lampada Externa",     "rele_lamp",  estados.lampExt);

    // Limiares
    c += F("<h3>&#9881; Limiares de Bateria</h3>");
    c += F("<div class='nf'>"
           "<b>Inv. Baixa:</b> Mantem ligado se bat &gt;= X% (padrao 20%)<br>"
           "<b>Inv. Alta:</b> Mantem ligado se bat &gt;= X% (padrao 40%)<br>"
           "<b>Lamp. Ext:</b> Desliga se bat &lt;= X% (padrao 30%)</div>");
    c += F("<form method='POST' action='/redes'>"
           "<input type='hidden' name='acao' value='limiares'>"
           "<label>% Minimo Inv. Baixa:</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvBaixa);
    c += "<input type='number' name='limInvBaixa' min='0' max='95' step='1' required value='"+String(tempBuf)+"'>";
    c += F("<label>% Minimo Inv. Alta:</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctInvAlta);
    c += "<input type='number' name='limInvAlta' min='0' max='95' step='1' required value='"+String(tempBuf)+"'>";
    c += F("<label>% Corte Lamp. Ext:</label>");
    snprintf(tempBuf, sizeof(tempBuf), "%.0f", limiares.pctLampExt);
    c += "<input type='number' name='limLamp' min='0' max='95' step='1' required value='"+String(tempBuf)+"'>";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Controle", c));
}

// ═══════════════════════════════════════════════════════════
// [SENS-WEB] SENSORES
// ═══════════════════════════════════════════════════════════
void handleSensores() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "calib_painel") {
            float iReal=server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>50) msg="<div class='er'>&#10007; 0-50A</div>";
            else {
                int novoOff=(int)((float)sensores.sinalPainel_mV+iReal*calibPainel.sensMvA);
                calibPainel.offsetMV=novoOff;
                prefs.begin("rrfs_cal",false);
                prefs.putInt("pn_off",novoOff);
                prefs.end();
                adicionarLog("Calib painel OK");
                msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao == "calib_casa") {
            float iReal=server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>100) msg="<div class='er'>&#10007; 0-100A</div>";
            else {
                int novoOff=(int)((float)sensores.sinalCasa_mV-iReal*calibCasa.sensMvA);
                calibCasa.offsetMV=novoOff;
                prefs.begin("rrfs_cal",false);
                prefs.putInt("casa_off",novoOff);
                prefs.end();
                adicionarLog("Calib casa OK");
                msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao == "reset_calib") {
            prefs.begin("rrfs_cal",false);
            prefs.remove("pn_off"); prefs.remove("pn_sens");
            prefs.remove("casa_off"); prefs.remove("casa_sens");
            prefs.end();
            carregarCalibSensores();
            adicionarLog("Calib reset");
            msg="<div class='ok'>&#10003; Resetado</div>";
        }
    }

    String c = msg;
    c += F("<h3>&#128270; Leituras Atuais</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "Calibracao MANUAL. Valores persistem apos reboot.</div>");

    auto cardSensor = [&](const char* ico, const char* nm,
                           float corr, float pot, int mv,
                           int off, float sens) {
        c += "<div class='sc'>";
        c += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
        char buf[24];
        snprintf(buf, sizeof(buf), "%.2fA", corr);
        c += "<div class='val'>" + String(buf) + "</div>";
        snprintf(buf, sizeof(buf), "%.0fW", pot);
        c += "<div style='font-size:13px;color:#78909c'>" + String(buf) + "</div>";
        snprintf(buf, sizeof(buf), "ADC %dmV | Off %d | Sens %.1f", mv, off, sens);
        c += "<div class='sub'>" + String(buf) + "</div></div>";
    };

    cardSensor("&#9728;","Painel",
               sensores.correntePainel, sensores.potenciaPainel,
               sensores.sinalPainel_mV, calibPainel.offsetMV, calibPainel.sensMvA);
    cardSensor("&#127968;","Casa",
               sensores.correnteCasa, sensores.potenciaCasa,
               sensores.sinalCasa_mV, calibCasa.offsetMV, calibCasa.sensMvA);

    // Bateria
    {
        String siBat = sensores.correnteBateria>0.3f?"+":
                       sensores.correnteBateria<-0.3f?"-":"~";
        c += F("<div class='sc'>");
        c += F("<div class='lbl'>&#128267; Bateria (calculada)</div>");
        char buf[24];
        snprintf(buf, sizeof(buf), "%s%.2fA", siBat.c_str(),
                 fabsf(sensores.correnteBateria));
        c += "<div class='val'>" + String(buf) + "</div>";
        snprintf(buf, sizeof(buf), "%.0fW", fabsf(sensores.potenciaBateria));
        c += "<div style='font-size:13px;color:#78909c'>" + String(buf) + "</div>";
        c += "<div class='sub'>Painel - Casa</div></div>";
    }

    c += F("<h3>&#9728; Calibrar Painel</h3>"
           "<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_painel'>"
           "<label>Corrente real (A):</label>"
           "<input type='number' name='corrente_real' step='0.01' min='0' max='50' required placeholder='Ex: 8.5'>"
           "<button type='submit' class='btn-g'>&#9728; Calibrar</button></form>");

    c += F("<h3>&#127968; Calibrar Casa</h3>"
           "<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_casa'>"
           "<label>Corrente real (A):</label>"
           "<input type='number' name='corrente_real' step='0.01' min='0' max='100' required placeholder='Ex: 12.3'>"
           "<button type='submit' class='btn-g'>&#127968; Calibrar</button></form>");

    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='reset_calib'>"
           "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
           "&#8635; Reset Fabrica</button></form>");

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Sensores", c));
}

// ═══════════════════════════════════════════════════════════
// [HIST-WEB] HISTÓRICO
// ═══════════════════════════════════════════════════════════
void handleHistorico() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    String c = "";
    c += F("<h3>&#128200; Totais Acumulados</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
    auto cardT=[&](const char* ico, const char* nm, float ah, const char* cor){
        c += "<div class='sc' style='text-align:center'>";
        c += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
        char buf[20];
        snprintf(buf, sizeof(buf), "%.1fAh", ah);
        c += "<div class='val' style='color:" + String(cor) + "'>" + String(buf) + "</div></div>";
    };
    cardT("&#9728;","Painel",   contadores.painelAh, "#fdd835");
    cardT("&#127968;","Casa",   contadores.casaAh,   "#4fc3f7");
    cardT("&#9650;","Bat+",     contadores.batCargAh,"#43a047");
    cardT("&#9660;","Bat-",     contadores.batDescAh,"#ef6c00");
    c += F("</div>");

    c += F("<h3>&#128200; Ultimos 7 Dias</h3>");
    int maxD = histTotalDias<7 ? histTotalDias : 7;
    if (maxD==0) {
        c += F("<div class='nf'>Sem historico</div>");
    } else {
        c += F("<div style='overflow-x:auto'>"
               "<table style='width:100%;border-collapse:collapse;font-size:11px'>"
               "<tr style='color:#546e7a'>"
               "<th style='text-align:left;padding:4px'>Data</th>"
               "<th style='padding:4px'>Painel</th>"
               "<th style='padding:4px'>Casa</th></tr>");
        for (int i=0; i<maxD; i++) {
            RegistroDia dT;
            bool ok = (i==0) ? (dT=histDiaAtual, true)
                              : carregarDiaFlash(((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS, dT);
            if (!ok || !dT.valido) continue;
            char bD[16];
            snprintf(bD, sizeof(bD), "%02d/%02d/%04d", dT.dia, dT.mes, dT.ano);
            c += "<tr style='border-top:1px solid #222'>";
            c += "<td style='padding:4px'>" + String(bD) + "</td>";
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", dT.totalPainelAh);
            c += "<td style='padding:4px;text-align:center'>" + String(buf) + "</td>";
            snprintf(buf, sizeof(buf), "%.1f", dT.totalCasaAh);
            c += "<td style='padding:4px;text-align:center'>" + String(buf) + "</td>";
            c += F("</tr>");
        }
        c += F("</table></div>");
    }

    // Gráfico tempo real
    c += F("<h3>&#128200; Tempo Real (60s)</h3>");
    {
        int total = histSnapFull ? HIST_SNAP_PONTOS : histSnapIdx;
        if (total < 2) {
            c += F("<div class='nf'>Aguardando...</div>");
        } else {
            float maxC = 0.5f;
            for (int i=0; i<total; i++) {
                int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
                if (histCorrPainel[idx]>maxC) maxC=histCorrPainel[idx];
                if (histCorrCasa[idx]>maxC)   maxC=histCorrCasa[idx];
                if (fabsf(histCorrBat[idx])>maxC) maxC=fabsf(histCorrBat[idx]);
            }
            c += F("<div style='position:relative;height:80px;background:#111;"
                   "border:1px solid #333;border-radius:4px'>");
            c += "<svg viewBox='0 0 " + String(total) + " 80' "
                 "preserveAspectRatio='none' style='width:100%;height:100%'>";
            auto py = [&](float v)->float { return 78.0f-(v/maxC)*76.0f; };
            String pP="M", pC="M";
            for (int i=0; i<total; i++) {
                int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
                char pt[20];
                snprintf(pt, sizeof(pt), "%d,%.1f", i, py(histCorrPainel[idx]));
                if (i>0) pP+="L"; pP+=String(pt);
                snprintf(pt, sizeof(pt), "%d,%.1f", i, py(histCorrCasa[idx]));
                if (i>0) pC+="L"; pC+=String(pt);
            }
            c += "<path d='"+pP+"' fill='none' stroke='#fdd835' stroke-width='2'/>";
            c += "<path d='"+pC+"' fill='none' stroke='#4fc3f7' stroke-width='2'/>";
            c += F("</svg></div>");
            c += F("<div style='font-size:10px;color:#546e7a;margin-top:4px'>"
                   "<span style='color:#fdd835'>&#9644;</span> Painel "
                   "<span style='color:#4fc3f7;margin-left:8px'>&#9644;</span> Casa</div>");
        }
    }

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Historico", c));
}

// ═══════════════════════════════════════════════════════════
// [CFG-WEB] CONFIGURAÇÃO GERAL
// ═══════════════════════════════════════════════════════════
void handleConfig() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "senha") {
            prefs.begin("rrfs",true);
            String usr=prefs.getString("user","");
            prefs.end();
            String pA=server.arg("pass_atual");
            String pN=server.arg("pass_nova");
            String pC=server.arg("pass_conf");
            if (!verificarLogin(usr,pA)) msg="<div class='er'>&#10007; Senha atual incorreta</div>";
            else if (pN.length()<4)      msg="<div class='er'>&#10007; Min 4 chars</div>";
            else if (pN!=pC)             msg="<div class='er'>&#10007; Senhas nao conferem</div>";
            else {
                salvarCredenciais(usr,pN);
                adicionarLog("Senha alterada");
                msg="<div class='ok'>&#10003; Senha alterada!</div>";
            }
        } else if (acao == "reset_historico") {
            if (server.arg("confirmar")!="SIM") {
                msg="<div class='er'>&#10007; Confirmacao invalida</div>";
            } else {
                contadores={0,0,0,0};
                salvarContadores();
                memset(&acum,0,sizeof(AcumHora));
                acum.horaAtual=255;
                acum.ultimoAcum=0;
                RegistroDia dV;
                memset(&dV,0,sizeof(RegistroDia));
                for (uint16_t i=0;i<HIST_DIAS;i++) salvarDiaFlash(i,dV);
                histIndiceAtual=0; histTotalDias=0;
                memset(&histDiaAtual,0,sizeof(RegistroDia));
                salvarIndiceHistorico();
                memset(histCorrPainel,0,sizeof(histCorrPainel));
                memset(histCorrCasa,  0,sizeof(histCorrCasa));
                memset(histCorrBat,   0,sizeof(histCorrBat));
                histSnapIdx=0; histSnapFull=false;
                adicionarLog("HIST apagado");
                msg="<div class='ok'>&#10003; Historico zerado!</div>";
            }
        }
    }

    String c = msg;
    c += F("<h3>&#128202; Sistema v18</h3><div class='sc'>");
    char bSys[80];
    snprintf(bSys, sizeof(bSys), "v18| RAM %.1fKB | Up %lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000UL);
    c += "<div class='val' style='font-size:12px'>"+String(bSys)+"</div>";
    String wf = modoAP ? "AP 10.0.0.1" : "WiFi "+WiFi.localIP().toString();
    c += "<div class='sub'>"+wf+"</div></div>";

    c += F("<div class='nf'><b>Logica v18:</b><br>"
           "Inv.Baixa: mantem ligado se bat >= limiar (padrao 20%)<br>"
           "Inv.Alta: mantem ligado se bat >= limiar (padrao 40%)<br>"
           "Lamp.Ext: 5min sem geracao ON | desliga bat &lt;= limiar (padrao 30%)<br>"
           "Potencia estimada = Corrente x Tensao Nominal</div>");

    c += F("<h3>&#128274; Alterar Senha</h3>"
           "<form method='POST' action='/config'>"
           "<input type='hidden' name='acao' value='senha'>"
           "<label>Senha Atual:</label>"
           "<input type='password' name='pass_atual' required>"
           "<label>Nova Senha:</label>"
           "<input type='password' name='pass_nova' required>"
           "<label>Confirmar:</label>"
           "<input type='password' name='pass_conf' required>"
           "<button type='submit' class='btn-o'>Alterar</button></form>");

    c += F("<h3>&#128465; Limpar Historico</h3>"
           "<form method='POST' action='/config' "
           "onsubmit='return confirm(\"Apagar permanentemente?\")'>"
           "<input type='hidden' name='acao' value='reset_historico'>"
           "<input type='hidden' name='confirmar' value='SIM'>"
           "<button type='submit' class='btn-r'>Apagar</button></form>");

    c += F("<h3>&#9881; Acoes</h3>");
    c += F("<form method='GET' action='/logs-salvos'>"
           "<button type='submit' class='btn-s btn-sm'>&#128221; Logs Salvos</button></form>");
    c += F("<form method='GET' action='/setup-wifi'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
           "&#128246; WiFi</button></form>");
    c += F("<form method='GET' action='/ota'>"
           "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>"
           "&#128640; OTA Firmware</button></form>");
    c += F("<form method='GET' action='/logout'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
           "&#128275; Logout</button></form>");
    c += F("<form method='POST' action='/reiniciar'>"
           "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
           "&#8635; Reiniciar ESP32</button></form>");

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Config", c));
}

// ═══════════════════════════════════════════════════════════
// [AUX-WEB] HANDLERS AUXILIARES
// ═══════════════════════════════════════════════════════════
void handleLogs() {
    if (!logsCarregados) carregarLogsPersistentes();
    String json = "[";
    bool primeiro = true;
    uint16_t inicio = totalLogs>50 ? totalLogs-50 : 0;
    for (uint16_t num=inicio; num<totalLogs; num++) {
        uint8_t idx = num % MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        if (!primeiro) json+=",";
        primeiro=false;
        json+="\"";
        for (int ch=0; logs[idx][ch] && ch<LOG_MAX_LEN; ch++) {
            char cc=logs[idx][ch];
            if (cc=='"')       json+="\\\"";
            else if (cc=='\\') json+="\\\\";
            else if (cc=='\n') json+=" ";
            else if (cc=='\r') continue;
            else               json+=cc;
        }
        json+="\"";
    }
    json+="]";
    server.send(200, "application/json", json);
}

void handleLogsSalvos() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    if (!logsCarregados) carregarLogsPersistentes();
    String c = F("<h3>&#128221; Logs Salvos</h3>");
    c += "<div class='nf'>Total: <b>"+String(totalLogs)+"</b></div>";
    c += F("<div class='lbx' style='height:400px'>");
    uint16_t inicio = totalLogs>MAX_LOGS ? totalLogs-MAX_LOGS : 0;
    for (uint16_t num=inicio; num<totalLogs; num++) {
        uint8_t idx = num % MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        c += "<div class='ll'>"+String(logs[idx])+"</div>";
    }
    c += F("</div>");
    c += F("<form method='GET' action='/config'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:8px'>"
           "&#8592; Voltar</button></form>");
    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - Logs", c));
}

void handleReiniciar() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    adicionarLog("Reinicio usuario");
    String c = F("<div class='ok'>&#10003; Reiniciando em 3s...</div>");
    server.send(200, "text/html;charset=utf-8", gerarPagina("Reiniciando", c, false));
    delay(3000);
    ESP.restart();
}

void handleNotFound() {
    String uri = server.uri();
    if (uri.indexOf("generate_204")>=0 || uri.indexOf("gen_204")>=0 ||
        uri.indexOf("hotspot-detect")>=0 || uri.indexOf("ncsi.txt")>=0 ||
        uri.indexOf("connecttest")>=0) {
        if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302,"text/plain",""); }
        else         server.send(204,"text/plain","");
        return;
    }
    if (uri=="/favicon.ico") { server.send(204); return; }
    if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else         server.send(404,"text/plain","404");
}

// ═══════════════════════════════════════════════════════════
// [OTA] ATUALIZAÇÃO FIRMWARE
// ═══════════════════════════════════════════════════════════
void handleOTA() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    String c = F("<h3>&#128640; Atualizar Firmware</h3>");
    c += F("<div class='nf'>"
           "&#8505; Selecione o arquivo .bin gerado pelo Arduino IDE.<br>"
           "O sistema reiniciara automaticamente apos a atualizacao.</div>");

    c += F("<form method='POST' action='/ota-upload' "
           "enctype='multipart/form-data' id='fOTA'>"
           "<label>Arquivo firmware (.bin):</label>"
           "<input type='file' id='fArq' name='firmware' accept='.bin' required "
           "style='background:#111;border:1px solid #444;border-radius:4px;"
           "padding:8px;color:#e0e0e0;width:100%;box-sizing:border-box'>"
           "<div id='prog' style='display:none;margin-top:8px'>"
           "<div class='bb'><div class='bv' id='pBar' style='width:0%;background:#0277bd'>"
           "0%</div></div>"
           "<div style='text-align:center;font-size:11px;color:#78909c' id='pTxt'>"
           "Enviando...</div></div>"
           "<button type='submit' class='btn-o' style='margin-top:10px' id='btnOTA'>"
           "&#128640; Enviar Firmware</button></form>"
           "<form method='GET' action='/config'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
           "&#8592; Voltar</button></form>");

    c += F("<script>"
           "document.getElementById('fOTA').onsubmit=function(e){"
           "e.preventDefault();"
           "var f=document.getElementById('fArq').files[0];"
           "if(!f){alert('Selecione um arquivo .bin');return;}"
           "if(!f.name.endsWith('.bin')){alert('Arquivo deve ser .bin');return;}"
           "var fd=new FormData();"
           "fd.append('firmware',f);"
           "var x=new XMLHttpRequest();"
           "document.getElementById('prog').style.display='block';"
           "document.getElementById('btnOTA').disabled=true;"
           "document.getElementById('btnOTA').textContent='Enviando...';"
           "x.upload.onprogress=function(e){"
           "if(e.lengthComputable){"
           "var p=Math.round(e.loaded/e.total*100);"
           "document.getElementById('pBar').style.width=p+'%';"
           "document.getElementById('pBar').textContent=p+'%';"
           "document.getElementById('pTxt').textContent="
           "'Enviando: '+Math.round(e.loaded/1024)+'KB / '+"
           "Math.round(e.total/1024)+'KB';}};"
           "x.onreadystatechange=function(){"
           "if(x.readyState==4){"
           "if(x.status==200){"
           "document.getElementById('pBar').style.width='100%';"
           "document.getElementById('pBar').style.background='#2e7d32';"
           "document.getElementById('pBar').textContent='100%';"
           "document.getElementById('pTxt').textContent="
           "'Concluido! Reiniciando em 5s...';"
           "setTimeout(function(){location.href='/';},5000);"
           "}else{"
           "document.getElementById('pTxt').textContent='ERRO: '+x.status;"
           "document.getElementById('pBar').style.background='#c62828';"
           "document.getElementById('btnOTA').disabled=false;"
           "document.getElementById('btnOTA').textContent='&#128640; Tentar Novamente';"
           "}}};"
           "x.open('POST','/ota-upload',true);"
           "x.send(fd);};"
           "</script>");

    server.send(200, "text/html;charset=utf-8", gerarPagina("RRFS - OTA", c));
}

void handleOTAUpload() {
    if (!logado) { server.send(403); return; }

    HTTPUpload& up = server.upload();

    if (up.status == UPLOAD_FILE_START) {
        adicionarLog("OTA inicio");
        esp_task_wdt_reset();
        // Desliga inversores durante OTA por segurança
        desligarInvAlta();
        desligarInvBaixa();
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            adicionarLog("OTA ERRO begin");
        }
    } else if (up.status == UPLOAD_FILE_WRITE) {
        esp_task_wdt_reset();
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
            adicionarLog("OTA ERRO write");
        }
    } else if (up.status == UPLOAD_FILE_END) {
        esp_task_wdt_reset();
        if (Update.end(true)) {
            adicionarLog("OTA OK - reiniciando");
            server.send(200, "text/plain", "OK");
            delay(500);
            ESP.restart();
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
    server.on("/logs",        HTTP_GET,  handleLogs);
    server.on("/logs-salvos", HTTP_GET,  handleLogsSalvos);
    server.on("/login",       HTTP_ANY,  handleLogin);
    server.on("/logout",      HTTP_GET,  handleLogout);
    server.on("/setup-login", HTTP_ANY,  handleSetupLogin);
    server.on("/setup-wifi",  HTTP_ANY,  handleSetupWifi);
    server.on("/reiniciar",   HTTP_POST, handleReiniciar);
    server.on("/ota",         HTTP_GET,  handleOTA);
    server.on("/ota-upload",  HTTP_POST,
        []() { /* resposta enviada dentro do handleOTAUpload */ },
        handleOTAUpload);
    server.onNotFound(handleNotFound);
    server.begin();
    adicionarLog("Rotas OK v18");
}
