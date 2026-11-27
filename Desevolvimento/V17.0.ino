/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v17 - CORRIGIDO
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 * MUDANÇAS v17:
 * - REMOVIDO: Sistema de previsão meteorológica
 * - REMOVIDO: NTP e sincronização de horário
 * - REMOVIDO: Calibração automática de sensores
 * - REMOVIDO: Lógica de consumo próprio ESP32
 * - SIMPLIFICADO: Logs apenas com números [00001]
 * - SIMPLIFICADO: Lamp.Ext liga 5min após parar geração
 * - OTIMIZADO: WiFi scan não-bloqueante
 * - CORRIGIDO: Compatibilidade ESP32 v3.x
 * - CORRIGIDO: Panic/reboot no setup
 * - MANTIDO: Calibração manual persistente
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
// [HW] PINOS - CONEXÕES FÍSICAS DO HARDWARE
// ═══════════════════════════════════════════════════════════
// LED_AZUL (GPIO2): LED interno do ESP32 para indicação visual
//   - Pisca: WiFi conectando / Modo AP
//   - Fixo: WiFi conectado
#define LED_AZUL              2

// BOTAO_BOOT (GPIO0): Botão BOOT do ESP32
//   - Pressionar 10s: Reset total (apaga todas configurações)
#define BOTAO_BOOT            0

// PIN_SENSOR_PAINEL (GPIO34): Entrada analógica do sensor ACS758 50A
//   - Sensor bidirecional de corrente do painel solar
//   - Conectar saída VOUT do ACS758 aqui
//   - Tensão: 0-3.3V (VCC do ACS758 = 5V, usar divisor 5V→3.3V)
//   - Calibração: manual via interface web
#define PIN_SENSOR_PAINEL    34

// PIN_SENSOR_CASA (GPIO35): Entrada analógica do sensor ACS758 100A
//   - Sensor unidirecional de corrente da casa
//   - Conectar saída VOUT do ACS758 aqui
//   - Tensão: 0-3.3V (VCC do ACS758 = 5V, usar divisor 5V→3.3V)
//   - Calibração: manual via alicate amperímetro
#define PIN_SENSOR_CASA      35

// PIN_STATUS_INV_ALTA (GPIO36): Entrada analógica de status do inversor
//   - Lê tensão de saída do inversor (via divisor resistivo)
//   - Tensão alta (>1.5V/1500mV) = inversor ligado
//   - Tensão baixa (<1.5V) = inversor desligado ou com falha
//   - Usado para proteção contra curto-circuito
#define PIN_STATUS_INV_ALTA  36

// PIN_RELE_INV_ALTA (GPIO27): Saída digital para relé do inversor
//   - HIGH (3.3V) = relé ligado → inversor energizado
//   - LOW (0V) = relé desligado → inversor desligado
//   - Usar módulo relé com opto-isolamento
//   - Capacidade mínima: 10A @ 12-48V DC
#define PIN_RELE_INV_ALTA    27

// PIN_RELE_LAMP_EXT (GPIO14): Saída digital para relé da lâmpada externa
//   - HIGH (3.3V) = relé ligado → lâmpada acesa
//   - LOW (0V) = relé desligado → lâmpada apagada
//   - Usar módulo relé com opto-isolamento
//   - Lógica: liga 5min após parar geração, desliga 5min após voltar
#define PIN_RELE_LAMP_EXT    14

// ═══════════════════════════════════════════════════════════
// [CFG] CONSTANTES - PARÂMETROS DO SISTEMA
// ═══════════════════════════════════════════════════════════

// DNS_PORT: Porta do servidor DNS (modo AP captive portal)
#define DNS_PORT             53

// TEMPO_RESET_MS: Tempo de pressão do botão BOOT para reset total (10s)
#define TEMPO_RESET_MS    10000

// MAX_LOGS: Quantidade máxima de logs salvos na memória flash (100)
#define MAX_LOGS            100

// LOG_MAX_LEN: Tamanho máximo de cada log em caracteres (64)
#define LOG_MAX_LEN          64

// LOG_NVS_NS: Namespace da NVS para armazenamento de logs
#define LOG_NVS_NS      "rrfs_log"

// ───────────────────────────────────────────────────────────
// ADC - Configuração do conversor analógico-digital
// ───────────────────────────────────────────────────────────
// ADC_SAMPLES: Número de amostras para média (150 = ~1.5s @ 100us/amostra)
#define ADC_SAMPLES         150

// ADC_VREF: Tensão de referência do ADC em mV (3300mV = 3.3V)
#define ADC_VREF           3300

// ADC_BITS: Resolução do ADC (4095 = 12 bits)
#define ADC_BITS           4095

// ───────────────────────────────────────────────────────────
// Sensores de Corrente - Valores de fábrica (ajustar após calibração)
// ───────────────────────────────────────────────────────────
// PAINEL_OFFSET_MV: Tensão de zero do sensor ACS758 painel (2500mV @ 0A)
#define PAINEL_OFFSET_MV   2500

// PAINEL_SENS_MV_A: Sensibilidade do ACS758 painel em mV/A (40mV/A típico)
#define PAINEL_SENS_MV_A     40

// CASA_OFFSET_MV: Tensão de zero do sensor ACS758 casa (600mV @ 0A após divisor)
#define CASA_OFFSET_MV      600

// CASA_SENS_MV_A: Sensibilidade do ACS758 casa em mV/A (20mV/A após divisor)
#define CASA_SENS_MV_A       20

// FILTRO_RUIDO_A: Corrente mínima para considerar (ignora ruído < 0.3A)
#define FILTRO_RUIDO_A      0.3f

// BUFFER_CORRENTE: Tamanho do buffer de média móvel (10 amostras)
#define BUFFER_CORRENTE      10

// ───────────────────────────────────────────────────────────
// Inversor - Configuração de monitoramento
// ───────────────────────────────────────────────────────────
// INV_THRESHOLD_MV: Tensão mínima para considerar inversor ligado (1500mV)
#define INV_THRESHOLD_MV   1500

// INV_BUFFER_MS: Tempo de buffer para evitar falsos positivos (2s)
#define INV_BUFFER_MS      2000

// ───────────────────────────────────────────────────────────
// Intervalos de Execução
// ───────────────────────────────────────────────────────────
// INTERVALO_LEITURA_MS: Intervalo entre leituras de sensores (1s)
#define INTERVALO_LEITURA_MS   1000

// INTERVALO_CONTROLE_MS: Intervalo de decisões de controle (5s)
#define INTERVALO_CONTROLE_MS  5000

// ───────────────────────────────────────────────────────────
// Proteção Contra Curto-Circuito
// ───────────────────────────────────────────────────────────
// MAX_TENTATIVAS_RELIGAR: Tentativas antes de travar por falha (3x)
#define MAX_TENTATIVAS_RELIGAR  3

// T_ESPERA_STATUS_INV: Tempo para aguardar status após ligar (2.5s)
#define T_ESPERA_STATUS_INV  2500

// T_STATUS_AUSENTE: Tempo sem status para considerar falha (1s)
#define T_STATUS_AUSENTE     1000

// T_PAUSA_RELIGAR: Pausa antes de tentar religar (2s)
#define T_PAUSA_RELIGAR      2000

// T_AGUARDA_STATUS: Tempo máximo aguardando status (3s)
#define T_AGUARDA_STATUS     3000

// ───────────────────────────────────────────────────────────
// Histórico de Energia
// ───────────────────────────────────────────────────────────
// HIST_DIAS: Dias de histórico armazenados (30 dias)
#define HIST_DIAS            30

// HIST_HORAS: Horas por dia no histórico (24h)
#define HIST_HORAS           24

// HIST_KEY_VER: Versão da estrutura de dados (incrementar se mudar)
#define HIST_KEY_VER          5

// HIST_SNAP_PONTOS: Pontos do gráfico em tempo real (60 segundos)
#define HIST_SNAP_PONTOS     60

// ───────────────────────────────────────────────────────────
// Lâmpada Externa
// ───────────────────────────────────────────────────────────
// LAMP_EXT_DELAY_MS: Tempo de confirmação antes de ligar/desligar (5min)
//   - Liga quando painel < 0.5A por 5 minutos (pôr do sol)
//   - Desliga quando painel > 1.0A por 5 minutos (nascer do sol)
#define LAMP_EXT_DELAY_MS  300000UL

// ───────────────────────────────────────────────────────────
// WiFi
// ───────────────────────────────────────────────────────────
// WIFI_RECON_INTERVALO: Intervalo entre tentativas de reconexão (30s)
#define WIFI_RECON_INTERVALO  30000UL

// WIFI_RECON_TIMEOUT: Timeout de cada tentativa (10s)
#define WIFI_RECON_TIMEOUT    10000UL

// WIFI_CHECK_INTERVALO: Intervalo de verificação de conexão (15s)
#define WIFI_CHECK_INTERVALO  15000UL

// WIFI_MAX_FALHAS: Falhas antes de ativar AP temporário (5x)
#define WIFI_MAX_FALHAS            5

// WiFi AP - Configuração do Access Point
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ═══════════════════════════════════════════════════════════
// [ENUM] ESTADOS
// ═══════════════════════════════════════════════════════════
enum EstadoCurto {
    CURTO_NORMAL = 0, CURTO_RELIGANDO,
    CURTO_TESTANDO_ALTA,
    CURTO_CONFIRMADO_INV, CURTO_AGUARDANDO_RESET
};
enum SubEstadoCurto {
    SUB_IDLE = 0, SUB_INVERSOR_DESLIGADO,
    SUB_AGUARDANDO_STATUS, SUB_INVERSOR_OK
};

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

struct ConfigRedes {
    float corteLampExt;
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
    int   sinalPainel_mV;
    int   sinalCasa_mV;
    int   sinalInversor_mV;
    bool  inversoreAtivo;
};

struct EstadoReles {
    bool lampExt;
    bool inversor;
};

struct TimersBuffer {
    unsigned long inversorUltimoHigh;
    bool          inversorEstadoReal;
};

struct EstimativaTempo {
    bool  valido;
    int   horas, minutos;
    bool  carregando;
    float taxaA;
};

struct GerenciadorCurto {
    EstadoCurto    estado;
    SubEstadoCurto subEstado;
    unsigned long  tInicio;
    bool           inversorCurto;
    bool           monitorando;
    uint8_t        tentativasReligar;
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

struct EstadoLampExt {
    bool  ligada;
    unsigned long tUltimaMudanca;
    bool  estadoAnteriorPainel;
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

char    logs[MAX_LOGS][LOG_MAX_LEN];
uint16_t totalLogs = 0;
uint8_t  logIndex  = 0;
bool     logsCarregados = false;
char     tempBuf[200];

ConfigBateria bateria   = {12.8f, 100.0f, 80.0f, 80.0f, 0.0f, 0.0f, false};
ConfigRedes   redes     = {50.0f};

DadosSensores sensores  = {0};
CalibSensor   calibPainel, calibCasa;
BufferMM      bufPainel = {0}, bufCasa = {0};

EstadoReles  estados = {false, false};
TimersBuffer timers  = {0, false};

GerenciadorCurto curto = {CURTO_NORMAL, SUB_IDLE, 0, false, false, 0};

EstadoLampExt lampExt = {false, 0, true};

unsigned long ultimaLeitura   = 0;
unsigned long ultimoControle  = 0;
unsigned long ultimoContagem  = 0;

String        gWifiSSID, gWifiPASS, gWifiIP, gWifiGW, gWifiMASK, gWifiDNS;
unsigned long tUltimoCheckWifi  = 0;
unsigned long tUltimoReconWifi  = 0;
uint8_t       wifiFalhasConsec  = 0;
bool          wifiReconectando  = false;
unsigned long tInicioRecon      = 0;

RegistroDia       histDiaAtual;
uint16_t          histIndiceAtual = 0, histTotalDias = 0;
ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora          acum = {0, 0, 0, 0, 255, 0, 0, 0, 0};

float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx = 0;
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

void adicionarLog(const char* msg);
void logRAM();
void setupLED(); void atualizarLED();
void ledBlink(); void ledFixo(bool v);
void setupBotao(); void verificarReset();
void setupReles();
void setRele(uint8_t pin, bool ligar);
void ligarLampExt();    void desligarLampExt();
void ligarInversor();   void desligarInversor();

int   lerADC(uint8_t pin);
int   adcParaMv(int v);
float adicionarAmostra(BufferMM& buf, float nova);
float lerCorrentePainel();
float lerCorrenteCasa();
bool  lerStatusInversor();
void  lerSensores();

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

void controlarEnergia();
void controlarLampExt();

void   monitorarInversor();
void   resetCurtoInversor();
String htmlStatusCurto();
void   _desligarInversorComBuffer();
void   _ligarInversorComBuffer();

bool existeCredenciais(); bool existeWiFiConfig();
void salvarCredenciais(const String& u, const String& p);
bool verificarLogin(const String& u, const String& p);
void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap);
void carregarWifi(String& ssid, String& pass, String& ip, String& gw,
                  String& mask, String& dns, bool& ap);
void salvarConfigBateria(); void carregarConfigBateria();
void salvarConfigRedes();   void carregarConfigRedes();
void carregarCalibSensores();

void iniciarAP();
bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns);
void gerenciarWiFi();

void configurarRotas();
String gerarPagina(const String& titulo, const String& conteudo,
                   bool comNav = true);
String barraProgresso(float pct, const String& label = "");

void handleRoot();      void handleDados();
void handleBateria();   void handleRedes();
void handleSensores();  void handleConfig();
void handleHistorico();
void handleLogs();      void handleLogsSalvos();
void handleSetupLogin();void handleSetupWifi();
void handleLogin();     void handleLogout();
void handleNotFound();
void handleResetCurto();
void handleReiniciar();
void handleOTAUpload(); void handleOTAResult();

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
    est.taxaA = taxa; est.valido = true; est.carregando = (taxa > 0);
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
    float ate = bateria.ahAtual - ahCorte;
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
    uint8_t  idx   = prefs.getUChar("idx", 0);

    totalLogs = total;
    logIndex  = idx;

    if (logIndex >= MAX_LOGS) logIndex = 0;

    uint16_t inicioLoad = (total > MAX_LOGS) ? (total - MAX_LOGS) : 0;
    for (uint16_t i = inicioLoad; i < total; i++) {
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
    prefs.putUChar("idx", logIndex);
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

void ledBlink() {
    blinkAtivo = true;
    ultimoBlink = millis();
}

void ledFixo(bool v) {
    blinkAtivo = false;
    digitalWrite(LED_AZUL, v);
}

// ═══════════════════════════════════════════════════════════
// [BTN] BOTÃO RESET
// ═══════════════════════════════════════════════════════════
void setupBotao() {
    pinMode(BOTAO_BOOT, INPUT_PULLUP);
}

void verificarReset() {
    bool p = (digitalRead(BOTAO_BOOT) == LOW);
    if (p && !botaoSegurando) {
        botaoSegurando = true;
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
    uint8_t pinos[] = {PIN_RELE_INV_ALTA, PIN_RELE_LAMP_EXT};
    for (uint8_t p : pinos) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }
    pinMode(PIN_STATUS_INV_ALTA, INPUT);
    estados = {false, false};
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
    adicionarLog("Reles: Inv+LampExt OFF");
}

void setRele(uint8_t pin, bool on) {
    digitalWrite(pin, on ? HIGH : LOW);
}

void ligarLampExt() {
    setRele(PIN_RELE_LAMP_EXT, true);
    estados.lampExt = true;
}

void desligarLampExt() {
    setRele(PIN_RELE_LAMP_EXT, false);
    estados.lampExt = false;
}

void ligarInversor() {
    setRele(PIN_RELE_INV_ALTA, true);
    estados.inversor = true;
}

void desligarInversor() {
    setRele(PIN_RELE_INV_ALTA, false);
    estados.inversor = false;
}

// ═══════════════════════════════════════════════════════════
// [ADC] LEITURA DE SENSORES - SEM CALIBRAÇÃO AUTOMÁTICA
// Calibração é SEMPRE manual via interface web
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

    // Usa valores salvos da calibração manual (NVS)
    float corrBruta = -(float)(mv - calibPainel.offsetMV) / calibPainel.sensMvA;

    if (corrBruta < 0.0f) corrBruta = 0.0f;
    if (corrBruta < 0.15f) corrBruta = 0.0f;

    float corrMedia = adicionarAmostra(bufPainel, corrBruta);
    if (corrMedia < 0.15f) corrMedia = 0.0f;

    return corrMedia;
}

float lerCorrenteCasa() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_CASA));
    sensores.sinalCasa_mV = mv;

    if (mv <= calibCasa.offsetMV) {
        adicionarAmostra(bufCasa, 0.0f);
        return 0.0f;
    }

    // Usa valores salvos da calibração manual (NVS)
    float corr = (float)(mv - calibCasa.offsetMV) / calibCasa.sensMvA;
    if (corr < 0) corr = 0;
    if (corr < FILTRO_RUIDO_A) corr = 0;

    float media = adicionarAmostra(bufCasa, corr);
    return (media < FILTRO_RUIDO_A) ? 0.0f : media;
}

bool lerStatusInversor() {
    int mv = adcParaMv(lerADC(PIN_STATUS_INV_ALTA));
    sensores.sinalInversor_mV = mv;
    if (!estados.inversor) {
        timers.inversorEstadoReal = false;
        timers.inversorUltimoHigh = 0;
        return false;
    }
    bool high = (mv >= INV_THRESHOLD_MV);
    if (high) {
        timers.inversorUltimoHigh = millis();
        timers.inversorEstadoReal = true;
    } else if (timers.inversorEstadoReal &&
        millis() - timers.inversorUltimoHigh >= INV_BUFFER_MS) {
        timers.inversorEstadoReal = false;
        }
        return timers.inversorEstadoReal;
}

void lerSensores() {
    esp_task_wdt_reset();
    sensores.correntePainel  = lerCorrentePainel();
    esp_task_wdt_reset();
    sensores.correnteCasa    = lerCorrenteCasa();
    esp_task_wdt_reset();
    sensores.correnteBateria = sensores.correntePainel - sensores.correnteCasa;
    sensores.inversoreAtivo  = lerStatusInversor();
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
// [HIST] HISTÓRICO (Ah)
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
        histTotalDias = 0;
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
        adicionarLog("Hist reset v17");
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

    uint32_t diasUptime = agora / 86400000UL;

    if (diaCache == 0) {
        diaCache = 1;
        mesCache = 1;
        anoCache = 2024;
    } else {
        diaCache++;
        if (diaCache > 28) {
            diaCache = 1;
            mesCache++;
            if (mesCache > 12) {
                mesCache = 1;
                anoCache++;
            }
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
        acum.diaAtual = dA;
        acum.mesAtual   = mA;
        acum.anoAtual = aA;

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
            histDiaAtual.ano = aA;
            histDiaAtual.mes = mA;
            histDiaAtual.dia = dA;
            histDiaAtual.valido = true;
            if (histTotalDias == 0) {
                histIndiceAtual = 0;
                histTotalDias = 1;
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
    if (ip >  0.15f) acum.painelAh   += ip        * dH;
    if (ic >  0.3f)  acum.casaAh     += ic        * dH;
    if (ib >  0.3f)  acum.batCargAh  += ib        * dH;
    if (ib < -0.3f)  acum.batDescAh  += fabsf(ib) * dH;

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
            histDiaAtual.ano = aA;
            histDiaAtual.mes = mA;
            histDiaAtual.dia = dA;
            histDiaAtual.valido = true;
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
// [CTRL] LÂMPADA EXTERNA
// ═══════════════════════════════════════════════════════════
void controlarLampExt() {
    unsigned long agora = millis();
    bool gerando = (sensores.correntePainel > 0.5f);

    if (gerando != lampExt.estadoAnteriorPainel) {
        lampExt.estadoAnteriorPainel = gerando;
        lampExt.tUltimaMudanca = agora;
    }

    unsigned long tempoDecorrido = agora - lampExt.tUltimaMudanca;

    if (!gerando && !estados.lampExt) {
        if (tempoDecorrido >= LAMP_EXT_DELAY_MS) {
            if (!bateria.calibrada ||
                bateria.percentualAtual > redes.corteLampExt) {
                if (estados.inversor) {
                    ligarLampExt();
                    adicionarLog("LampExt ON noite");
                }
                }
        }
    } else if (gerando && estados.lampExt) {
        if (tempoDecorrido >= LAMP_EXT_DELAY_MS) {
            desligarLampExt();
            adicionarLog("LampExt OFF dia");
        }
    }

    if (estados.lampExt && bateria.calibrada) {
        if (bateria.percentualAtual <= redes.corteLampExt) {
            desligarLampExt();
            adicionarLog("LampExt OFF bat");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] CONTROLE DE ENERGIA
// ═══════════════════════════════════════════════════════════
void controlarEnergia() {
    if (!bateria.calibrada) return;
    float pct = bateria.percentualAtual;

    if (pct <= 0.0f && estados.inversor &&
        curto.estado == CURTO_NORMAL) {
        desligarInversor();
    if (estados.lampExt) desligarLampExt();
    adicionarLog("CRITICO bat 0% Inv OFF");
        return;
        }

        if (pct > 5.0f && !estados.inversor &&
            curto.estado == CURTO_NORMAL) {
            ligarInversor();
        adicionarLog("Inv ON bat >5%");
            }
}

// ═══════════════════════════════════════════════════════════
// [CURTO] PROTEÇÃO CONTRA CURTO
// ═══════════════════════════════════════════════════════════
void _desligarInversorComBuffer() {
    setRele(PIN_RELE_INV_ALTA, false);
    estados.inversor = false;
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
}

void _ligarInversorComBuffer() {
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
    setRele(PIN_RELE_INV_ALTA, true);
    estados.inversor = true;
}

void monitorarInversor() {
    unsigned long agora = millis();
    bool emProc = (curto.estado == CURTO_RELIGANDO ||
    curto.estado == CURTO_TESTANDO_ALTA);

    if (!estados.inversor && !emProc) {
        if (curto.estado == CURTO_NORMAL) {
            curto.monitorando       = false;
            curto.tInicio           = 0;
            curto.tentativasReligar = 0;
        }
        return;
    }

    if (curto.estado == CURTO_NORMAL) {
        bool st = lerStatusInversor();
        if (st) {
            curto.tInicio           = 0;
            curto.monitorando       = true;
            curto.tentativasReligar = 0;
            return;
        }
        if (curto.monitorando) {
            if (!curto.tInicio) {
                curto.tInicio = agora;
                adicionarLog("Inv status perdido");
                return;
            }
            if (agora - curto.tInicio < T_STATUS_AUSENTE) return;
            adicionarLog("Inv protecao ativa");
            _desligarInversorComBuffer();
            curto.estado    = CURTO_RELIGANDO;
            curto.subEstado = SUB_INVERSOR_DESLIGADO;
            curto.tInicio   = agora;
        }
        return;
    }

    if (curto.estado == CURTO_RELIGANDO) {
        switch (curto.subEstado) {
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_PAUSA_RELIGAR) return;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Religando %d/%d",
                         curto.tentativasReligar+1, MAX_TENTATIVAS_RELIGAR);
                adicionarLog(tempBuf);
                _ligarInversorComBuffer();
                curto.tInicio   = agora;
                curto.subEstado = SUB_AGUARDANDO_STATUS;
                break;

            case SUB_AGUARDANDO_STATUS: {
                bool st = lerStatusInversor();
                if (st && timers.inversorEstadoReal) {
                    curto.estado            = CURTO_NORMAL;
                    curto.subEstado         = SUB_IDLE;
                    curto.tInicio           = 0;
                    curto.monitorando       = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Inv OK pos religar");
                    return;
                }
                if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    curto.tentativasReligar++;
                    _desligarInversorComBuffer();
                    curto.tInicio = agora;
                    if (curto.tentativasReligar < MAX_TENTATIVAS_RELIGAR) {
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    } else {
                        curto.estado    = CURTO_AGUARDANDO_RESET;
                        curto.subEstado = SUB_IDLE;
                        curto.inversorCurto = true;
                        adicionarLog("FALHA INV reset necessario");
                    }
                }
                break;
            }
            default:
                curto.estado = CURTO_NORMAL;
                curto.subEstado = SUB_IDLE;
                break;
        }
        return;
    }

    if (curto.estado == CURTO_CONFIRMADO_INV ||
        curto.estado == CURTO_AGUARDANDO_RESET) {
        if (estados.inversor) _desligarInversorComBuffer();
        if (estados.lampExt)  desligarLampExt();
        }
}

void resetCurtoInversor() {
    if (curto.estado != CURTO_CONFIRMADO_INV &&
        curto.estado != CURTO_AGUARDANDO_RESET) return;
    adicionarLog("Reset curto inv");
    _desligarInversorComBuffer();

    delay(T_PAUSA_RELIGAR);

    _ligarInversorComBuffer();
    unsigned long t = millis();
    bool ok = false;
    while (millis() - t < T_ESPERA_STATUS_INV) {
        if (lerStatusInversor()) {
            ok = true;
            break;
        }
        delay(100);
    }
    if (ok) {
        curto.estado        = CURTO_NORMAL;
        curto.inversorCurto = false;
        curto.subEstado     = SUB_IDLE;
        curto.tInicio       = 0;
        curto.monitorando   = true;
        curto.tentativasReligar = 0;
        adicionarLog("Inv RESOLVIDO");
    } else {
        _desligarInversorComBuffer();
        curto.estado = CURTO_AGUARDANDO_RESET;
        adicionarLog("Inv nao respondeu");
    }
}

String htmlStatusCurto() {
    if (curto.estado == CURTO_NORMAL && !curto.inversorCurto)
        return F("<div class='ok' style='font-size:12px'>"
        "&#10003; Sem falhas</div>");
    if (curto.estado == CURTO_RELIGANDO)
        return F("<div class='wn'>&#9874; Tentando religar...</div>");
    String h = "";
    if (curto.estado == CURTO_AGUARDANDO_RESET ||
        curto.estado == CURTO_CONFIRMADO_INV) {
        h += F("<div class='er'><b>&#9888; FALHA NO INVERSOR!</b><br>"
        "Verifique fisicamente o inversor.</div>");
    h += F("<form method='POST' action='/reset-curto'>"
    "<button type='submit' class='btn-g' style='margin-top:8px'>"
    "&#9889; Tentar Religar</button></form>");
        }
        return h;
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
    return (su==u && sp==p && su.length()>0);
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

                                  void salvarConfigRedes() {
                                      prefs.begin("rrfs", false);
                                      prefs.putFloat("r_lmpext", redes.corteLampExt);
                                      prefs.end();
                                  }

                                  void carregarConfigRedes() {
                                      prefs.begin("rrfs", true);
                                      redes.corteLampExt = prefs.getFloat("r_lmpext", 50.0f);
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
                                  // [WIFI] REDE — NÃO BLOQUEANTE
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
                                                             wifiReconectando  = false;
                                                             tInicioRecon      = 0;
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
                                                             wifiReconectando  = false;
                                                             tInicioRecon      = 0;
                                                             ledFixo(true);
                                                             return;
                                                         }

                                                         if (agora - tInicioRecon >= WIFI_RECON_TIMEOUT) {
                                                             wifiReconectando  = false;
                                                             tInicioRecon      = 0;
                                                             tUltimoReconWifi  = agora;

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
                                                     // [SETUP] INICIALIZAÇÃO
                                                     // ═══════════════════════════════════════════════════════════
                                                     void setup() {
                                                         // Watchdog 30s - CORRIGIDO para ESP32 v3.x
                                                         esp_task_wdt_config_t wdt_config = {
                                                             .timeout_ms = 30000,
                                                             .idle_core_mask = 0,
                                                             .trigger_panic = true
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
                                                         errNVS = nvs_flash_init();
                                                             }

                                                             esp_reset_reason_t motivo = esp_reset_reason();
                                                             switch (motivo) {
                                                                 case ESP_RST_BROWNOUT:
                                                                     adicionarLog("RESET BROWNOUT");
                                                                     break;
                                                                 case ESP_RST_PANIC:
                                                                     adicionarLog("RESET PANIC");
                                                                     break;
                                                                 case ESP_RST_TASK_WDT:
                                                                     adicionarLog("RESET TASK WDT");
                                                                     break;
                                                                 case ESP_RST_INT_WDT:
                                                                     adicionarLog("RESET INT WDT");
                                                                     break;
                                                                 case ESP_RST_WDT:
                                                                     adicionarLog("RESET WDT");
                                                                     break;
                                                                 case ESP_RST_SW:
                                                                     adicionarLog("Reset software");
                                                                     break;
                                                                 case ESP_RST_POWERON:
                                                                     adicionarLog("Power ON");
                                                                     break;
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
                                                             adicionarLog("RRFS v17");
                                                             logRAM();

                                                             esp_task_wdt_reset();

                                                             carregarConfigBateria();
                                                             carregarConfigRedes();
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
                                                                     if (iniciarWiFiNB(ssid, pass, ip, gw, mask, dns)) {
                                                                         usarAP = false;
                                                                     }
                                                                 }
                                                             }

                                                             esp_task_wdt_reset();

                                                             if (usarAP) iniciarAP();

                                                             esp_task_wdt_reset();

                                                         configurarRotas();

                                                         adicionarLog("Sistema pronto");
                                                         logRAM();

                                                         esp_task_wdt_reset();
                                                     }

                                                     // ═══════════════════════════════════════════════════════════
                                                     // [LOOP] CICLO PRINCIPAL
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

                                                         if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
                                                             ultimaLeitura = agora;
                                                             esp_task_wdt_reset();
                                                             lerSensores();
                                                             acumularEnergia();
                                                             esp_task_wdt_reset();
                                                         }

                                                         {
                                                             bool emProc = (curto.estado == CURTO_RELIGANDO ||
                                                             curto.estado == CURTO_TESTANDO_ALTA);
                                                             if (estados.inversor || emProc) {
                                                                 monitorarInversor();
                                                             } else if (curto.estado == CURTO_NORMAL && curto.monitorando) {
                                                                 curto.monitorando       = false;
                                                                 curto.tInicio           = 0;
                                                                 curto.tentativasReligar = 0;
                                                             }
                                                         }

                                                         if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
                                                             ultimoControle = agora;
                                                             esp_task_wdt_reset();
                                                             if (curto.estado == CURTO_NORMAL ||
                                                                 curto.estado == CURTO_CONFIRMADO_INV) {
                                                                 controlarEnergia();
                                                             controlarLampExt();
                                                                 }
                                                                 esp_task_wdt_reset();
                                                         }

                                                         gerenciarWiFi();
                                                         esp_task_wdt_reset();

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
.sc.ok{border-color:#2e7d32;background:#0a1f0a}
.sc.wn{border-color:#ef6c00;background:#1f130a}
.sc.er{border-color:#c62828;background:#1f0a0a}
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
    String cor = pct>=50 ? "#2e7d32" : pct>=25 ? "#ef6c00" : "#c62828";
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
    html.reserve(6000);
    html = F("<!DOCTYPE html><html><head>"
             "<meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>");
    html += CSS;
    html += F("</style></head><body><div class='card'>");
    html += F("<h2>&#9889; RRFS v17</h2>");
    html += conteudo;

    if (comNav && logado) {
        String pag = server.uri();
        html += F("<div class='nav'>");
        html += "<a href='/' " + String(pag=="/" ? "class='active'":"") + ">&#128202;</a>";
        html += "<a href='/bateria' " + String(pag=="/bateria" ? "class='active'":"") + ">&#128267;</a>";
        html += "<a href='/redes' " + String(pag=="/redes" ? "class='active'":"") + ">&#128161;</a>";
        html += "<a href='/sensores' " + String(pag=="/sensores" ? "class='active'":"") + ">&#128270;</a>";
        html += "<a href='/config' " + String(pag=="/config" ? "class='active'":"") + ">&#9881;</a>";
        html += "<a href='/historico' " + String(pag=="/historico" ? "class='active'":"") + ">&#128200;</a>";
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
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location","/setup-wifi");
        server.send(302);
        return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String u=server.arg("user"), p=server.arg("pass"),
               c=server.arg("conf");
        if (u.length()<3)
            msg="<div class='er'>&#10007; Min 3 chars usuario</div>";
        else if (p.length()<4)
            msg="<div class='er'>&#10007; Min 4 chars senha</div>";
        else if (p!=c)
            msg="<div class='er'>&#10007; Senhas nao conferem</div>";
        else {
            salvarCredenciais(u,p);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location","/setup-wifi");
            server.send(302);
            return;
        }
    }
    String c = F("<div class='nf'>&#8505; Primeiro acesso. Crie suas credenciais.</div>");
    c += "<h3>&#128274; Criar Conta</h3>" + msg;
    c += F("<form method='POST' action='/setup-login'>"
           "<label>Usuario (min 3):</label>"
           "<input type='text' name='user' maxlength='32' required "
           "autocomplete='off' placeholder='Usuario'>"
           "<label>Senha (min 4):</label>"
           "<input type='password' name='pass' maxlength='32' required "
           "placeholder='Senha'>"
           "<label>Confirmar Senha:</label>"
           "<input type='password' name='conf' maxlength='32' required "
           "placeholder='Confirmar'>"
           "<button type='submit'>Criar &#8594;</button></form>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Setup", c, false));
}

void handleSetupWifi() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login");
        server.send(302);
        return;
    }
    if (existeWiFiConfig() && !logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }

    String msg = "";
    if (server.method() == HTTP_POST) {
        String modo = server.arg("modo");
        if (modo == "ap") {
            salvarWifi("","","","","","",true);
            String ct = F("<div class='ok'>&#10003; Salvo! Reiniciando...</div>"
                          "<div class='nf'>SSID: <b>RRFS Energia</b> | "
                          "IP: <b>10.0.0.1</b></div>");
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("Salvo",ct,false));
            delay(2000);
            ESP.restart();
            return;
        }
        String ssid=server.arg("ssid"), wp=server.arg("wpass"),
               ip=server.arg("ip"),     gw=server.arg("gw"),
               mask=server.arg("mask"), dns=server.arg("dns");
        if (ssid.isEmpty())
            msg="<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7)
            msg="<div class='er'>Informe o IP</div>";
        else if (gw.length()<7)
            msg="<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wp,ip,gw,mask,dns,false);
            String ct = "<div class='ok'>&#10003; Salvo! Acesse: http://"+ip+"</div>";
            ct += F("<div class='nf'>Reiniciando em 2s...</div>");
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("Salvo",ct,false));
            delay(2000);
            ESP.restart();
            return;
        }
    }

    static int16_t nRedes = -1;
    static bool scanIniciado = false;

    if (!scanIniciado) {
        WiFi.scanDelete();
        WiFi.scanNetworks(true, false, false, 300);
        scanIniciado = true;
        nRedes = -1;
    }

    if (nRedes == -1) {
        nRedes = WiFi.scanComplete();
    }

    String c = F("<div class='nf'>&#8505; Configure a rede. IP fixo obrigatorio.</div>");
    c += "<h3>&#128246; Configurar Rede</h3>" + msg;
    c += F("<form method='POST' action='/setup-wifi'>"
           "<label>Modo:</label>"
           "<select name='modo' id='modo' onchange='toggleModo()'>"
           "<option value='wifi'>Conectar ao WiFi</option>"
           "<option value='ap'>Manter Ponto de Acesso</option>"
           "</select><div id='dw'>"
           "<label>Rede WiFi:</label><select name='ssid'>");

    if (nRedes == WIFI_SCAN_RUNNING) {
        c += F("<option value=''>Escaneando...</option>");
    } else if (nRedes == 0 || nRedes == WIFI_SCAN_FAILED) {
        c += F("<option value=''>Nenhuma rede</option>");
        scanIniciado = false;
    } else {
        for (int i = 0; i < nRedes; i++) {
            String q = WiFi.RSSI(i)>-60?"Bom":
                       WiFi.RSSI(i)>-75?"Regular":"Fraco";
            c += "<option value='" + WiFi.SSID(i) + "'>"
               + WiFi.SSID(i);
            char buf[32];
            snprintf(buf, sizeof(buf), " [%s %ddBm]", q.c_str(), WiFi.RSSI(i));
            c += String(buf) + "</option>";
        }
        WiFi.scanDelete();
        scanIniciado = false;
        nRedes = -1;
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
           "<button type='submit' class='btn-g'>&#128190; Salvar</button>"
           "</form>");
    if (logado)
        c += F("<button onclick='location.href=\"/\"' class='btn-s' "
               "style='margin-top:8px'>&#8592; Voltar</button>");
    c += F("<script>function toggleModo(){"
           "document.getElementById('dw').style.display="
           "document.getElementById('modo').value=='wifi'?'block':'none';"
           "}</script>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Rede", c, false));
}

void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login");
        server.send(302);
        return;
    }
    if (logado) {
        server.sendHeader("Location","/");
        server.send(302);
        return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String u=server.arg("user"), p=server.arg("pass");
        if (verificarLogin(u,p)) {
            logado = true;
            adicionarLog("Login OK");
            logRAM();
            server.sendHeader("Location","/");
            server.send(302);
            return;
        }
        adicionarLog("Login FALHOU");
        msg = F("<div class='er'>&#10007; Usuario/senha incorretos</div>");
    }
    String c = F("<div class='nf'>&#128274; Acesso seguro</div>"
                 "<h3>Entrar</h3>");
    c += msg;
    c += F("<form method='POST' action='/login'>"
           "<label>Usuario:</label>"
           "<input type='text' name='user' required autocomplete='off'>"
           "<label>Senha:</label>"
           "<input type='password' name='pass' required>"
           "<button type='submit'>&#128275; Entrar</button></form>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Login", c, false));
}

void handleLogout() {
    if (logado) {
        adicionarLog("Logout");
        logado = false;
    }
    server.sendHeader("Location","/login");
    server.send(302);
}

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
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }

    if (!logsCarregados) carregarLogsPersistentes();

    String c = F("<h3>&#128221; Logs Salvos (Ultimos 100)</h3>");
    c += "<div class='nf'>Total de logs: <b>" + String(totalLogs) + "</b></div>";
    c += F("<div class='lbx' style='height:400px'>");

    uint16_t inicio = totalLogs > MAX_LOGS ? totalLogs - MAX_LOGS : 0;
    for (uint16_t num = inicio; num < totalLogs; num++) {
        uint8_t idx = num % MAX_LOGS;
        if (logs[idx][0] == '\0') continue;
        c += "<div class='ll'>";
        c += logs[idx];
        c += "</div>";
    }

    c += F("</div>");
    c += F("<form method='GET' action='/config'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:8px'>"
           "&#8592; Voltar</button></form>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Logs", c));
}

void handleNotFound() {
    String uri = server.uri();
    if (uri.indexOf("generate_204")>=0 || uri.indexOf("gen_204")>=0 ||
        uri.indexOf("hotspot-detect")>=0 || uri.indexOf("ncsi.txt")>=0 ||
        uri.indexOf("connecttest")>=0) {
        if (modoAP) {
            server.sendHeader("Location","http://10.0.0.1/");
            server.send(302,"text/plain","");
        } else {
            server.send(204,"text/plain","");
        }
        return;
    }
    if (uri == "/favicon.ico") {
        server.send(204);
        return;
    }
    if (modoAP) {
        server.sendHeader("Location","http://10.0.0.1/");
        server.send(302);
    } else {
        server.send(404,"text/plain","404");
    }
}

// ═══════════════════════════════════════════════════════════
// [DASH] DASHBOARD PRINCIPAL
// ═══════════════════════════════════════════════════════════
void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login");
        server.send(302);
        return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location","/setup-wifi");
        server.send(302);
        return;
    }
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }

    bool painelGer = (sensores.correntePainel  >  0.5f);
    bool batCarg   = (sensores.correnteBateria >  0.3f);
    bool batDesc   = (sensores.correnteBateria < -0.3f);
    bool casaCons  = (sensores.correnteCasa    >  0.3f);
    bool invAtivo  = sensores.inversoreAtivo;
    bool falhaInv  = (curto.estado == CURTO_CONFIRMADO_INV ||
                      curto.estado == CURTO_AGUARDANDO_RESET);

    char sIP[10], sIC[10], sIB[10];
    char sWP[10], sWC[10], sWB[10];
    float vn = bateria.tensaoNominal;
    snprintf(sIP,sizeof(sIP),"%.1fA",sensores.correntePainel);
    snprintf(sIC,sizeof(sIC),"%.1fA",sensores.correnteCasa);
    snprintf(sIB,sizeof(sIB),"%.1fA",fabsf(sensores.correnteBateria));
    snprintf(sWP,sizeof(sWP),"%.0fW",sensores.correntePainel*vn);
    snprintf(sWC,sizeof(sWC),"%.0fW",sensores.correnteCasa*vn);
    snprintf(sWB,sizeof(sWB),"%.0fW",fabsf(sensores.correnteBateria)*vn);

    char pctBuf[8];
    snprintf(pctBuf,sizeof(pctBuf),"%.0f%%",
             bateria.calibrada ? bateria.percentualAtual : 0.0f);

    EstimativaTempo tempo = calcularTempoRestante();
    String tLE = "--";
    if (batDesc && bateria.calibrada)
        tLE = tempoAteCorte(redes.corteLampExt, sensores.correnteBateria);

    String cPainel = painelGer ? "#fdd835" : "#455a64";
    String cBat    = !bateria.calibrada ? "#455a64" :
                      bateria.percentualAtual>50  ? "#43a047" :
                      bateria.percentualAtual>25  ? "#ef6c00" : "#c62828";
    String cInv    = invAtivo ? "#4fc3f7"  : "#455a64";
    String cLE     = estados.lampExt ? "#ffd54f" : "#455a64";

    String c = "";

    if (!bateria.calibrada)
        c += F("<div class='wn' style='font-size:11px;margin-bottom:6px'>"
               "&#9888; Bateria nao calibrada. "
               "<a href='/bateria' style='color:#ffe0b2'>Calibrar</a></div>");

    c += F("<h3 style='text-align:center;margin:4px 0 10px'>"
           "&#9889; Fluxo de Energia</h3>");
    {
        bool fPB = painelGer && batCarg;
        bool fPC = painelGer && casaCons;
        bool fBC = batDesc   && casaCons;
        String lPB = fPB ? "#43a047" : "#333";
        String lPC = fPC ? "#fdd835" : "#333";
        String lBC = fBC ? "#ef6c00" : "#333";

        c += "<svg viewBox='0 0 340 230' "
             "style='width:100%;height:auto;display:block'>";

        c += "<path d='M110,55 C160,20 200,20 240,55' fill='none' stroke='" +
             lPB + "' stroke-width='3'/>";
        c += "<path d='M90,90 C80,130 90,150 130,175' fill='none' stroke='" +
             lPC + "' stroke-width='3'/>";
        c += "<path d='M255,90 C260,130 250,150 210,175' fill='none' stroke='" +
             lBC + "' stroke-width='3'/>";

        c += "<g><rect x='20' y='15' width='100' height='80' rx='14' "
             "fill='#181818' stroke='" + cPainel + "' stroke-width='2'/>";
        c += "<text x='70' y='42' text-anchor='middle' font-size='24'>&#9728;</text>";
        c += "<text x='70' y='60' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>";
        c += "<text x='70' y='76' text-anchor='middle' font-size='16' fill='" +
             cPainel + "' font-weight='bold'>" +
             (painelGer ? String(sIP) : "0.0A") + "</text></g>";

        c += "<g><rect x='220' y='15' width='100' height='80' rx='14' "
             "fill='#181818' stroke='" + cBat + "' stroke-width='2'/>";
        String icoBat = batCarg ? "&#9650;&#128267;" :
                        batDesc ? "&#9660;&#128267;" : "&#128267;";
        c += "<text x='270' y='42' text-anchor='middle' font-size='22'>" +
             icoBat + "</text>";
        c += "<text x='270' y='60' text-anchor='middle' font-size='9' fill='#78909c'>BATERIA</text>";
        c += "<text x='270' y='76' text-anchor='middle' font-size='16' fill='" +
             cBat + "' font-weight='bold'>" +
             (bateria.calibrada ? String(pctBuf) : "N/Cal") + "</text></g>";

        // CORRIGIDO: usar String() para concatenação
        String corCasa = casaCons ? String("#4fc3f7") : String("#3a3a3a");
        c += "<g><rect x='105' y='175' width='130' height='55' rx='14' "
             "fill='#181818' stroke='" + corCasa + "' stroke-width='2'/>";
        c += "<text x='128' y='210' font-size='22'>&#127968;</text>";
        c += "<text x='225' y='196' text-anchor='end' font-size='9' fill='#78909c'>CASA+INV</text>";
        String corTexto = casaCons ? String("#4fc3f7") : String("#455a64");
        c += "<text x='225' y='212' text-anchor='end' font-size='14' fill='" +
             corTexto + "' font-weight='bold'>" +
             (casaCons ? String(sIC) : "0.0A") + "</text></g>";

        c += "</svg>";
    }

    c += F("<div style='margin:10px 0'>");
    c += F("<div style='display:flex;justify-content:space-between;"
           "align-items:center;margin-bottom:4px'>");
    c += F("<span style='font-size:11px;color:#78909c'>&#128267; Bateria</span>");

    if (tempo.valido) {
        char bT[50];
        if (tempo.carregando) {
            snprintf(bT,sizeof(bT),"&#9650; %dh%02dm",
                     tempo.horas,tempo.minutos);
        } else {
            snprintf(bT,sizeof(bT),"&#9660; %dh%02dm",
                     tempo.horas,tempo.minutos);
        }
        String corT = tempo.carregando ? "#43a047" : "#ef6c00";
        c += "<span style='font-size:11px;font-weight:bold;color:" + corT + "'>";
        c += String(bT) + "</span>";
    }
    c += F("</div>");

    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual);
    } else {
        c += F("<div class='wn' style='font-size:11px;text-align:center'>"
               "Nao calibrada</div>");
    }
    c += F("</div>");

    c += F("<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
           "gap:6px;margin:4px 0'>");

    c += "<div class='sc' style='padding:8px;text-align:center'>";
    c += F("<div class='lbl'>&#9728;</div>");
    c += "<div style='font-size:18px;font-weight:bold;color:" +
         cPainel + "' id='cp'>";
    c += painelGer ? String(sIP) : "0.0A";
    c += F("</div></div>");

    c += F("<div class='sc' style='padding:8px;text-align:center'>");
    c += F("<div class='lbl'>&#127968;</div>");
    c += "<div style='font-size:18px;font-weight:bold;color:#4fc3f7' id='cc'>";
    c += String(sIC);
    c += F("</div></div>");

    {
        String cf = batCarg?"#43a047":batDesc?"#ef6c00":"#fdd835";
        String si = batCarg?"+":batDesc?"-":"~";
        c += "<div class='sc' style='padding:8px;text-align:center'>";
        c += F("<div class='lbl'>&#128267;</div>");
        c += "<div style='font-size:18px;font-weight:bold;color:" + cf + "' id='cb'>";
        c += si + String(sIB);
        c += F("</div></div>");
    }
    c += F("</div>");

    String ip = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    c += "<div class='nf' style='font-size:10px'>";
    c += "&#128246; " + String(modoAP?"AP":"WiFi") + " | " + ip;
    if (!modoAP && WiFi.status()==WL_CONNECTED)
        c += " | " + String(WiFi.RSSI()) + "dBm";
    snprintf(tempBuf,sizeof(tempBuf)," | RAM %.1fKB | Up %lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000UL);
    c += String(tempBuf) + F("</div>");

    c += F("<script>"
    "function aD(){"
    "var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){"
    "if(x.readyState==4&&x.status==200){"
    "try{"
    "var d=JSON.parse(x.responseText);"
    "var ip=parseFloat(d.ip||0);"
    "var ic=parseFloat(d.ic||0);"
    "var ib=parseFloat(d.ib||0);"
    "var vn=parseFloat(d.vnom||12.8);"
    "var eCP=document.getElementById('cp');"
    "if(eCP)eCP.textContent=ip>0.5?ip.toFixed(1)+'A':'0.0A';"
    "var eCC=document.getElementById('cc');"
    "if(eCC)eCC.textContent=ic.toFixed(1)+'A';"
    "var eCB=document.getElementById('cb');"
    "if(eCB){var s=ib>0.3?'+':ib<-0.3?'-':'~';"
    "eCB.textContent=s+Math.abs(ib).toFixed(1)+'A';}"
    "}catch(e){}}}"
    ";x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Painel", c));
}

void handleDados() {
    if (!logado) {
        server.send(403);
        return;
    }
    char buf[200];
    snprintf(buf,sizeof(buf),
             "{\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
             "\"pct\":%.1f,\"ah_at\":%.2f,\"ah_disp\":%.2f,"
             "\"vnom\":%.1f,\"calib\":%s,"
             "\"inv\":%s,\"le\":%s}",
             sensores.correntePainel, sensores.correnteCasa,
             sensores.correnteBateria,
             bateria.percentualAtual, bateria.ahAtual, bateria.ahDisponivel,
             bateria.tensaoNominal,
             bateria.calibrada?"true":"false",
             sensores.inversoreAtivo?"true":"false",
             estados.lampExt?"true":"false");
    server.send(200,"application/json", buf);
}

// ═══════════════════════════════════════════════════════════
// [BAT-WEB] PÁGINA DE BATERIA
// ═══════════════════════════════════════════════════════════
void handleBateria() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config") {
            float vn=server.arg("vnom").toFloat();
            float ah=server.arg("ah").toFloat();
            float dc=server.arg("desc").toFloat();
            if (vn<6||vn>60)
                msg="<div class='er'>&#10007; Tensao 6-60V</div>";
            else if (ah<1||ah>5000)
                msg="<div class='er'>&#10007; Capacidade 1-5000Ah</div>";
            else if (dc<10||dc>100)
                msg="<div class='er'>&#10007; Descarga 10-100%</div>";
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
            float pBMS = server.arg("pct_atual").toFloat();
            if (pBMS<0||pBMS>100)
                msg="<div class='er'>&#10007; Percentual 0-100</div>";
            else {
                float ahFis = bateria.capacidadeAh*(pBMS/100.0f);
                float ahRes = bateria.capacidadeAh*(1.0f-bateria.descargaSegura/100.0f);
                float ahUt  = ahFis - ahRes;
                if (ahUt<0) ahUt=0;
                if (ahUt>bateria.ahDisponivel) ahUt=bateria.ahDisponivel;
                bateria.ahAtual = ahUt;
                bateria.percentualAtual = (bateria.ahDisponivel>0)
                    ? (ahUt/bateria.ahDisponivel)*100.0f : 0.0f;
                bateria.calibrada = true;
                ultimoContagem = millis();
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
        snprintf(buf,sizeof(buf),"%.1fAh / %.1fAh (%.0f%%)",
                 bateria.ahAtual,bateria.ahDisponivel,bateria.percentualAtual);
        c += "<div class='nf' style='text-align:center'>" + String(buf) + F("</div>");
    } else {
        c += F("<div class='wn'>Aguardando calibracao</div>");
    }

    c += F("<h3>&#9881; Configurar</h3>"
           "<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='config'>"
           "<label>Tensao Nominal (V):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"value='%.1f'",bateria.tensaoNominal);
    c += "<input type='number' name='vnom' step='0.1' min='6' max='60' required "
         + String(tempBuf) + ">";
    c += F("<label>Capacidade (Ah):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"value='%.0f'",bateria.capacidadeAh);
    c += "<input type='number' name='ah' step='1' min='1' max='5000' required "
         + String(tempBuf) + ">";
    c += F("<label>Descarga Segura (%):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"value='%.0f'",bateria.descargaSegura);
    c += "<input type='number' name='desc' step='1' min='10' max='100' required "
         + String(tempBuf) + ">";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");

    c += F("<h3>&#127919; Calibrar</h3>");
    c += F("<div class='nf'>Informe o % do BMS/Display</div>");
    c += F("<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='calibrar'>"
           "<label>% no Display/BMS:</label>"
           "<input type='number' name='pct_atual' step='1' min='0' max='100' "
           "required placeholder='Ex: 75'>"
           "<button type='submit' class='btn-o'>"
           "&#127919; Calibrar</button></form>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Bateria",c));
}

// ═══════════════════════════════════════════════════════════
// [REDES-WEB] PÁGINA
// ═══════════════════════════════════════════════════════════
void handleRedes() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            float cle = server.arg("corteLampExt").toFloat();
            if (cle<0||cle>90)
                msg="<div class='er'>&#10007; Lamp.Ext 0-90%</div>";
            else {
                redes.corteLampExt = cle;
                salvarConfigRedes();
                adicionarLog("Redes config salva");
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao == "rele") {
            String qual  = server.arg("rele");
            bool   ligar = (server.arg("valor") == "1");

            if (qual == "inv") {
                if (curto.estado==CURTO_CONFIRMADO_INV ||
                    curto.estado==CURTO_AGUARDANDO_RESET) {
                    msg="<div class='er'>&#9888; Bloqueado: falha inv</div>";
                } else {
                    if (ligar) {
                        ligarInversor();
                        adicionarLog("Manual Inv ON");
                    }
                    else {
                        desligarInversor();
                        adicionarLog("Manual Inv OFF");
                    }
                    msg="<div class='ok'>&#10003; OK</div>";
                }
            } else if (qual == "le") {
                if (ligar) {
                    ligarLampExt();
                    adicionarLog("Manual LE ON");
                }
                else {
                    desligarLampExt();
                    adicionarLog("Manual LE OFF");
                }
                msg="<div class='ok'>&#10003; OK</div>";
            }
        }
    }

    bool falhaInv = (curto.estado==CURTO_CONFIRMADO_INV ||
                     curto.estado==CURTO_AGUARDANDO_RESET);

    String c = msg;

    c += F("<h3>&#9888; Protecao</h3>");
    c += htmlStatusCurto();

    c += F("<h3>&#128268; Controle Manual</h3>");

    c += "<div class='cg'>";
    c += F("<div class='cr'><span class='cn'>&#9889; Inversor</span>");
    c += "<span class='cs' style='color:" +
         String(estados.inversor?"#43a047":"#78909c") + "'>";
    c += estados.inversor ? "&#9679; ON" : "&#9675; OFF";
    c += F("</span></div>");
    if (!falhaInv) {
        c += F("<div style='display:flex;gap:6px;margin-top:8px'>");
        c += F("<form method='POST' action='/redes' style='flex:1'>"
               "<input type='hidden' name='acao' value='rele'>"
               "<input type='hidden' name='rele' value='inv'>"
               "<input type='hidden' name='valor' value='1'>"
               "<button type='submit' class='btn-g btn-sm'>ON</button></form>");
        c += F("<form method='POST' action='/redes' style='flex:1'>"
               "<input type='hidden' name='acao' value='rele'>"
               "<input type='hidden' name='rele' value='inv'>"
               "<input type='hidden' name='valor' value='0'>"
               "<button type='submit' class='btn-r btn-sm'>OFF</button></form>");
        c += F("</div>");
    }
    c += F("</div>");

    c += "<div class='cg'>";
    c += F("<div class='cr'><span class='cn'>&#128161; Lamp. Ext</span>");
    c += "<span class='cs' style='color:" +
         String(estados.lampExt?"#ffd54f":"#78909c") + "'>";
    c += estados.lampExt ? "&#9679; ON" : "&#9675; OFF";
    c += F("</span></div>");
    c += F("<div style='display:flex;gap:6px;margin-top:8px'>");
    c += F("<form method='POST' action='/redes' style='flex:1'>"
           "<input type='hidden' name='acao' value='rele'>"
           "<input type='hidden' name='rele' value='le'>"
           "<input type='hidden' name='valor' value='1'>"
           "<button type='submit' class='btn-g btn-sm'>ON</button></form>");
    c += F("<form method='POST' action='/redes' style='flex:1'>"
           "<input type='hidden' name='acao' value='rele'>"
           "<input type='hidden' name='rele' value='le'>"
           "<input type='hidden' name='valor' value='0'>"
           "<button type='submit' class='btn-r btn-sm'>OFF</button></form>");
    c += F("</div></div>");

    c += F("<h3>&#9889; Config Lamp. Ext</h3>");
    c += F("<div class='nf'>Liga quando painel para de gerar (5min sem geracao).<br>"
           "Desliga quando volta geracao (5min gerando).</div>");
    c += F("<form method='POST' action='/redes'>"
           "<input type='hidden' name='acao' value='config'>"
           "<label>Corte por Bateria Baixa (%):</label>");
    char v[16];
    snprintf(v,sizeof(v),"value='%.0f'",redes.corteLampExt);
    c += "<input type='number' name='corteLampExt' min='0' max='90' "
         "step='1' required " + String(v) + ">";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Controle",c));
}

// ═══════════════════════════════════════════════════════════
// [SENS-WEB] PÁGINA DE SENSORES
// ═══════════════════════════════════════════════════════════
void handleSensores() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "calib_painel") {
            float iReal = server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>50) {
                msg="<div class='er'>&#10007; Corrente 0-50A</div>";
            } else {
                int mvAt = sensores.sinalPainel_mV;
                int novoOff = (int)((float)mvAt + iReal*calibPainel.sensMvA);
                calibPainel.offsetMV = novoOff;
                prefs.begin("rrfs_cal",false);
                prefs.putInt("pn_off", novoOff);
                prefs.end();
                adicionarLog("Calib painel OK");
                msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao == "calib_casa") {
            float iReal = server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>100) {
                msg="<div class='er'>&#10007; Corrente 0-100A</div>";
            } else {
                int mvAt = sensores.sinalCasa_mV;
                int novoOff = (int)((float)mvAt - iReal*calibCasa.sensMvA);
                calibCasa.offsetMV  = novoOff;
                calibCasa.calibrado = true;
                prefs.begin("rrfs_cal",false);
                prefs.putInt("casa_off", novoOff);
                prefs.end();
                adicionarLog("Calib casa OK");
                msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao == "reset_calib") {
            prefs.begin("rrfs_cal",false);
            prefs.remove("pn_off");
            prefs.remove("pn_sens");
            prefs.remove("casa_off");
            prefs.remove("casa_sens");
            prefs.end();
            carregarCalibSensores();
            adicionarLog("Calib reset");
            msg="<div class='ok'>&#10003; Resetado</div>";
        }
    }

    String c = msg;
    c += F("<h3>&#128270; Leituras</h3>");
    c += F("<div class='nf' style='font-size:10px'><b>IMPORTANTE:</b> "
           "Calibracao e MANUAL. Apos calibrar, valores sao salvos na memoria flash "
           "e persistem apos reboot. Nao ha calibracao automatica.</div>");

    c += "<div class='sc'>";
    c += F("<div class='lbl'>&#9728; Painel</div>");
    char buf[20];
    snprintf(buf,sizeof(buf),"%.2fA",sensores.correntePainel);
    c += "<div class='val'>" + String(buf) + F("</div>");
    snprintf(buf,sizeof(buf),"ADC %dmV | Off %d | Sens %.1f",
             sensores.sinalPainel_mV, calibPainel.offsetMV, calibPainel.sensMvA);
    c += "<div class='sub'>" + String(buf) + F("</div></div>");

    c += "<div class='sc'>";
    c += F("<div class='lbl'>&#127968; Casa</div>");
    snprintf(buf,sizeof(buf),"%.2fA",sensores.correnteCasa);
    c += "<div class='val'>" + String(buf) + F("</div>");
    snprintf(buf,sizeof(buf),"ADC %dmV | Off %d | Sens %.1f",
             sensores.sinalCasa_mV, calibCasa.offsetMV, calibCasa.sensMvA);
    c += "<div class='sub'>" + String(buf) + F("</div></div>");

    c += "<div class='sc'>";
    c += F("<div class='lbl'>&#128267; Bateria</div>");
    snprintf(buf,sizeof(buf),"%.2fA",sensores.correnteBateria);
    c += "<div class='val'>" + String(buf) + F("</div>");
    c += F("<div class='sub'>Calculado: Painel - Casa</div></div>");

    c += F("<h3>&#9728; Calibrar Painel</h3>");
    c += F("<div class='nf'>Use o display do controlador de carga como referencia.</div>");
    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_painel'>"
           "<label>Corrente real (A):</label>"
           "<input type='number' name='corrente_real' step='0.01' "
           "min='0' max='50' required placeholder='Ex: 8.5'>"
           "<button type='submit' class='btn-g'>&#9728; Calibrar</button></form>");

    c += F("<h3>&#127968; Calibrar Casa</h3>");
    c += F("<div class='nf'>Use um alicate amperimetro como referencia.</div>");
    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_casa'>"
           "<label>Corrente real (A):</label>"
           "<input type='number' name='corrente_real' step='0.01' "
           "min='0' max='100' required placeholder='Ex: 12.3'>"
           "<button type='submit' class='btn-g'>&#127968; Calibrar</button></form>");

    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='reset_calib'>"
           "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
           "&#8635; Reset Fabrica</button></form>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Sensores",c));
}

// ═══════════════════════════════════════════════════════════
// [HIST-WEB] HISTÓRICO
// ═══════════════════════════════════════════════════════════
void handleHistorico() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }

    String c = "";
    c += F("<h3>&#128200; Totais</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");

    auto cardT = [&](const char* ico, const char* nm, float ah, const char* cor) {
        c += "<div class='sc' style='text-align:center'>";
        c += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
        char buf[20];
        snprintf(buf,sizeof(buf),"%.1fAh",ah);
        c += "<div class='val' style='color:" + String(cor) + "'>" + String(buf) + "</div></div>";
    };
    cardT("&#9728;","Painel",   contadores.painelAh, "#fdd835");
    cardT("&#127968;","Casa",   contadores.casaAh,   "#4fc3f7");
    cardT("&#9650;","Bat+",     contadores.batCargAh,"#43a047");
    cardT("&#9660;","Bat-",     contadores.batDescAh,"#ef6c00");
    c += F("</div>");

    c += F("<h3>&#128200; Ultimos 7 Dias</h3>");
    int maxD = histTotalDias < 7 ? histTotalDias : 7;
    if (maxD == 0) {
        c += F("<div class='nf'>Sem historico</div>");
    } else {
        c += F("<div style='overflow-x:auto'>"
               "<table style='width:100%;border-collapse:collapse;font-size:11px'>"
               "<tr style='color:#546e7a'>"
               "<th style='text-align:left;padding:4px'>Data</th>"
               "<th style='padding:4px'>Painel</th>"
               "<th style='padding:4px'>Casa</th></tr>");
        for (int i = 0; i < maxD; i++) {
            RegistroDia dT;
            bool ok = (i==0)?(dT=histDiaAtual,true)
                            :carregarDiaFlash(
                                ((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,dT);
            if (!ok||!dT.valido) continue;
            char bD[16];
            snprintf(bD,sizeof(bD),"%02d/%02d/%04d",dT.dia,dT.mes,dT.ano);
            c += "<tr style='border-top:1px solid #222'>";
            c += "<td style='padding:4px'>" + String(bD) + "</td>";
            char buf[10];
            snprintf(buf,sizeof(buf),"%.1f",dT.totalPainelAh);
            c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalCasaAh);
            c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td>";
            c += F("</tr>");
        }
        c += F("</table></div>");
    }

    c += F("<h3>&#128200; Tempo Real (60s)</h3>");
    {
        int total = histSnapFull ? HIST_SNAP_PONTOS : histSnapIdx;
        if (total < 2) {
            c += F("<div class='nf'>Aguardando...</div>");
        } else {
            float maxC = 0.5f;
            for (int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                if (histCorrPainel[idx]>maxC) maxC=histCorrPainel[idx];
                if (histCorrCasa[idx]>maxC)   maxC=histCorrCasa[idx];
                if (fabsf(histCorrBat[idx])>maxC) maxC=fabsf(histCorrBat[idx]);
            }
            c += F("<div style='position:relative;height:80px;background:#111;"
                   "border:1px solid #333;border-radius:4px'>");
            c += "<svg viewBox='0 0 " + String(total) + " 80' "
                 "preserveAspectRatio='none' style='width:100%;height:100%'>";

            auto py=[&](float v)->float{
                return 78.0f-(v/maxC)*76.0f;
            };

            String pP="M";
            for(int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                char pt[20];
                snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrPainel[idx]));
                if(i>0)pP+="L";
                pP+=String(pt);
            }
            c+="<path d='"+pP+"' fill='none' stroke='#fdd835' stroke-width='2'/>";

            String pC="M";
            for(int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                char pt[20];
                snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrCasa[idx]));
                if(i>0)pC+="L";
                pC+=String(pt);
            }
            c+="<path d='"+pC+"' fill='none' stroke='#4fc3f7' stroke-width='2'/>";

            c+=F("</svg></div>");
        }
    }

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Historico",c));
}

// ═══════════════════════════════════════════════════════════
// [CFG-WEB] CONFIGURAÇÃO GERAL
// ═══════════════════════════════════════════════════════════
void handleConfig() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
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
            if (!verificarLogin(usr,pA))
                msg="<div class='er'>&#10007; Senha atual incorreta</div>";
            else if (pN.length()<4)
                msg="<div class='er'>&#10007; Min 4 chars</div>";
            else if (pN!=pC)
                msg="<div class='er'>&#10007; Senhas nao conferem</div>";
            else {
                salvarCredenciais(usr,pN);
                adicionarLog("Senha alterada");
                msg="<div class='ok'>&#10003; Senha alterada!</div>";
            }
        } else if (acao == "reset_historico") {
            if (server.arg("confirmar") != "SIM") {
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
                histIndiceAtual=0;
                histTotalDias=0;
                memset(&histDiaAtual,0,sizeof(RegistroDia));
                salvarIndiceHistorico();
                memset(histCorrPainel,0,sizeof(histCorrPainel));
                memset(histCorrCasa,  0,sizeof(histCorrCasa));
                memset(histCorrBat,   0,sizeof(histCorrBat));
                histSnapIdx=0;
                histSnapFull=false;
                adicionarLog("HIST apagado");
                msg="<div class='ok'>&#10003; Historico zerado!</div>";
            }
        }
    }

    String c = msg;
    c += F("<h3>&#128202; Sistema</h3>");
    c += F("<div class='sc'>");
    char bSys[80];
    snprintf(bSys,sizeof(bSys),"v17 | RAM %.1fKB | Up %lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000UL);
    c += "<div class='val' style='font-size:12px'>" + String(bSys) + F("</div>");
    String wf = modoAP ? "AP 10.0.0.1" :
                "WiFi "+WiFi.localIP().toString();
    c += "<div class='sub'>" + wf + F("</div></div>");

    c += F("<div class='nf'><b>Logica v17:</b><br>"
           "Inversor: 0% OFF | >5% ON<br>"
           "Lamp.Ext: 5min sem geracao ON | 5min gerando OFF<br>"
           "Sensores: calibracao MANUAL persistente</div>");

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

    c += F("<h3>&#128465; Limpar Historico</h3>");
    c += F("<form method='POST' action='/config' "
           "onsubmit='return confirm(\"Apagar permanentemente?\")'>"
           "<input type='hidden' name='acao' value='reset_historico'>"
           "<input type='hidden' name='confirmar' value='SIM'>"
           "<button type='submit' class='btn-r'>Apagar</button></form>");

    c += F("<h3>&#9881; Acoes</h3>");
    c += F("<form method='GET' action='/logs-salvos'>"
           "<button type='submit' class='btn-s btn-sm'>"
           "&#128221; Ver Logs Salvos</button></form>");
    c += F("<form method='GET' action='/setup-wifi'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
           "&#128246; WiFi</button></form>");
    c += F("<form method='GET' action='/ota'>"
           "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>"
           "&#128640; OTA</button></form>");
    c += F("<form method='GET' action='/logout'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
           "&#128275; Logout</button></form>");
    c += F("<form method='POST' action='/reiniciar'>"
           "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
           "&#8635; Reiniciar</button></form>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Config",c));
}

// ═══════════════════════════════════════════════════════════
// [AUX-WEB] HANDLERS AUXILIARES
// ═══════════════════════════════════════════════════════════
void handleResetCurto() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    resetCurtoInversor();
    server.sendHeader("Location","/redes");
    server.send(302);
}

void handleReiniciar() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    adicionarLog("Reinicio usuario");
    String c = F("<div class='ok'>&#10003; Reiniciando...</div>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("Reiniciando",c,false));
    delay(2000);
    ESP.restart();
}

void handleOTAResult() {
    if (!logado) {
        server.sendHeader("Location","/login");
        server.send(302);
        return;
    }
    String c = F("<h3>&#128640; OTA</h3>"
                 "<div class='nf'>Selecione o .bin</div>"
                 "<form method='POST' action='/ota-upload' "
                 "enctype='multipart/form-data'>"
                 "<label>Firmware (.bin):</label>"
                 "<input type='file' name='firmware' accept='.bin' required "
                 "style='background:#111;border:1px solid #444;border-radius:4px;"
                 "padding:8px;color:#e0e0e0;width:100%'>"
                 "<button type='submit' class='btn-o' style='margin-top:10px'>"
                 "&#128640; Enviar</button></form>"
                 "<form method='GET' action='/config'>"
                 "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
                 "&#8592; Voltar</button></form>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - OTA",c));
}

void handleOTAUpload() {
    HTTPUpload& up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
        adicionarLog("OTA inicio");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
            adicionarLog("OTA ERRO begin");
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize)
            adicionarLog("OTA ERRO write");
    } else if (up.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            adicionarLog("OTA OK");
            String c = F("<div class='ok'>&#10003; Atualizado!</div>");
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("OTA OK",c,false));
            delay(2000);
            ESP.restart();
        } else {
            adicionarLog("OTA ERRO end");
            String c = F("<div class='er'>&#10007; Erro OTA</div>");
            server.send(500,"text/html;charset=utf-8",
                        gerarPagina("OTA Erro",c,false));
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
    server.on("/reset-curto", HTTP_POST, handleResetCurto);
    server.on("/reiniciar",   HTTP_POST, handleReiniciar);
    server.on("/ota",         HTTP_GET,  handleOTAResult);
    server.on("/ota-upload",  HTTP_POST,
              [](){handleOTAResult();}, handleOTAUpload);
    server.onNotFound(handleNotFound);
    server.begin();
}
