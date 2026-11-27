/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v16
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 * MUDANÇAS v16:
 * - REMOVIDO: Sistema de IA completamente
 * - REMOVIDO: Estimativa de tensão
 * - MANTIDO: Controle manual por % de bateria
 * - MANTIDO: Calibração de sensores (1 ponto, persistente)
 * - OTIMIZADO: Cálculo kW inline, sem campos extras na struct
 * - OTIMIZADO: Sem bloqueantes desnecessários
 * - OTIMIZADO: Strings F() para economizar RAM
 * ════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════
// [INC] INCLUDES
// ═══════════════════════════════════════════════════════════
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <time.h>
#include <Update.h>
#include <HTTPClient.h>
#include <nvs_flash.h>

// ═══════════════════════════════════════════════════════════
// [HW] PINOS
// ═══════════════════════════════════════════════════════════
#define LED_AZUL              2
#define BOTAO_BOOT            0
#define PIN_SENSOR_PAINEL    34
#define PIN_SENSOR_CASA      35
//#define PIN_BKP_BATERIA      32
#define PIN_STATUS_INV_ALTA  36
#define PIN_RELE_INV_ALTA    27
#define PIN_RELE_REDE_BAIXA  26
#define PIN_RELE_REDE_ALTA   25
#define PIN_RELE_LAMP_EXT    14
#define PIN_RELE_LAMP_INT    12

// ═══════════════════════════════════════════════════════════
// [CFG] CONSTANTES
// ═══════════════════════════════════════════════════════════
#define MAX_LOGS             20
#define LOG_MAX_LEN         100
#define DNS_PORT             53
#define TEMPO_RESET_MS    10000

// ADC
#define ADC_SAMPLES         150
#define ADC_VREF           3300
#define ADC_BITS           4095

// Sensores
#define PAINEL_OFFSET_MV   2500
#define PAINEL_SENS_MV_A     40
#define CASA_OFFSET_MV      600
#define CASA_SENS_MV_A       20
#define FILTRO_RUIDO_A      0.3f
#define BUFFER_CORRENTE      10

// Inversor
#define INV_THRESHOLD_MV   1500
#define INV_BUFFER_MS      2000
#define RELE_BUFFER_MS    60000

// Intervalos
#define INTERVALO_LEITURA_MS   1000
#define INTERVALO_CONTROLE_MS  5000

// Proteção curto
#define MAX_TENTATIVAS_RELIGAR  3
#define T_ESPERA_STATUS_INV  2500
#define T_STATUS_AUSENTE     1000
#define T_PAUSA_RELIGAR      2000
#define T_AGUARDA_QUEDA      1500
#define T_AGUARDA_STATUS     3000

// Histórico
#define HIST_DIAS            30
#define HIST_HORAS           24
#define HIST_KEY_VER          3
#define HIST_SNAP_PONTOS     60

// Previsão
#define PREV_HORAS           48
#define PREV_URL_MAX        256
#define PREV_INTERVALO   3600000UL

// Restauração
#define RESTORE_T_BAIXA      2000
#define RESTORE_T_INV_BOOT   5000
#define RESTORE_T_INV_STATUS 3000
#define RESTORE_T_ALTA        200

// Antecipação
#define T_ANTECIP_ESTABILIZA  60000UL
#define T_ANTECIP_REAVAL     300000UL
#define T_ANTECIP_COOLDOWN   900000UL
#define MARGEM_ANTECIP_MIN       30

// WiFi AP
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ═══════════════════════════════════════════════════════════
// [ENUM] ESTADOS
// ═══════════════════════════════════════════════════════════
enum EstadoCurto {
    CURTO_NORMAL = 0, CURTO_RELIGANDO,
    CURTO_TESTANDO_ALTA, CURTO_TESTANDO_BAIXA,
    CURTO_CONFIRMADO_ALTA, CURTO_CONFIRMADO_BAIXA,
    CURTO_AGUARDANDO_RESET
};
enum SubEstadoCurto {
    SUB_IDLE = 0, SUB_INVERSOR_DESLIGADO,
    SUB_AGUARDANDO_QUEDA, SUB_AGUARDANDO_STATUS,
    SUB_INVERSOR_OK_ALTA_OFF, SUB_INVERSOR_OK_LIGAR_ALTA
};
enum EstadoRestauro {
    RESTORE_OCIOSO = 0, RESTORE_LIGANDO_BAIXA,
    RESTORE_AGUARDANDO_BAIXA, RESTORE_LIGANDO_INVERSOR,
    RESTORE_AGUARDANDO_INVERSOR_BOOT, RESTORE_CHECANDO_STATUS_INV,
    RESTORE_LIGANDO_ALTA, RESTORE_LIGANDO_LAMP_INT,
    RESTORE_CONCLUIDO
};
enum EstadoAntecip {
    ANTECIP_IDLE = 0, ANTECIP_TESTANDO, ANTECIP_MANTIDO
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
    float corteBaixa;
    float corteAlta;
    float corteLampExt;
    float corteLampInt;
};

struct ConfigLocal {
    double  latitude;
    double  longitude;
    float   ampControlador;
    float   eficiencia;
    bool    configurado;
    float   fatorCalib;
    uint8_t amostrasCalib;
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

// kW calculado inline: (corrente * tensaoNominal / 1000)
struct DadosSensores {
    float correntePainel;
    float correnteCasa;
    float correnteSistema;
    float correnteBateria;
    int   sinalPainel_mV;
    int   sinalCasa_mV;
    int   sinalInversor_mV;
    bool  inversoreAtivo;
};

struct EstadoReles {
    bool redeBaixa;
    bool redesAlta;
    bool lampExt;
    bool lampInt;
    bool inversor;
};

struct TimersBuffer {
    unsigned long inicioBaixa, inicioAlta;
    unsigned long inicioLampExt, inicioLampInt;
    bool pendenteBaixa, pendenteAlta;
    bool pendenteLampExt, pendenteLampInt;
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
    bool           redeAltaCurto, redeBaixaCurto;
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

struct PrevisaoHora {
    uint8_t  hora, dia, mes;
    uint16_t ano;
    float    radiacao, radiacaoDifusa;
    float    coberturaNuvens, temperatura;
    float    correnteEstimada;
    bool     valido;
};

struct ResultadoPrev {
    PrevisaoHora  horas[PREV_HORAS];
    uint8_t       totalHoras;
    float         correnteHojeRestante;
    float         correnteAmanha;
    unsigned long ultimaAtualizacao;
    bool          valido;
    char          erro[64];
};

struct DecisaoSistema {
    bool  desligarRedeAltaNoite;
    bool  desligarLampExtNoite;
    bool  desligarLampIntDia;
    float horaLigarRedeAlta;
    bool  alertaEnergiaInsuficiente;
    char  motivo[128];
};

struct RestauroEnergia {
    EstadoRestauro estado;
    unsigned long  tInicio;
};

struct GerenciadorAntecip {
    EstadoAntecip estado;
    unsigned long tInicio, tUltimaAval, tCooldown;
};

// ═══════════════════════════════════════════════════════════
// [VAR] VARIÁVEIS GLOBAIS
// ═══════════════════════════════════════════════════════════
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP = false;
bool logado = false;

// LED / Botão
unsigned long ultimoBlink = 0;
bool ledState = false, blinkAtivo = true;
unsigned long botaoPressionadoEm = 0;
bool botaoSegurando = false;

// Logs
char    logs[MAX_LOGS][LOG_MAX_LEN];
uint8_t totalLogs = 0, logIndex = 0;
char    tempBuf[200];

// Configurações
ConfigBateria bateria   = {12.8f, 100.0f, 80.0f, 80.0f, 0.0f, 0.0f, false};
ConfigRedes   redes     = {10.0f, 30.0f, 50.0f, 0.0f};
ConfigLocal   configLocal = {0, 0, 30.0f, 0.80f, false, 1.0f, 0};

// Sensores
DadosSensores sensores  = {0};
CalibSensor   calibPainel, calibCasa;
bool          autoCalibPainelAtivo = true;
BufferMM      bufPainel = {0}, bufCasa = {0}, bufSistema = {0};

// Relés
EstadoReles  estados = {false, false, false, false, false};
TimersBuffer timers  = {0};

// Curto
GerenciadorCurto curto = {CURTO_NORMAL, SUB_IDLE, 0, false, false, false, 0};

// Intervalos
unsigned long ultimaLeitura = 0, ultimoControle = 0, ultimoContagem = 0;

// Restauração / Antecipação
RestauroEnergia   restauro     = {RESTORE_OCIOSO, 0};
GerenciadorAntecip antecipAlta = {ANTECIP_IDLE, 0, 0, 0};

// Histórico
RegistroDia       histDiaAtual;
uint16_t          histIndiceAtual = 0, histTotalDias = 0;
ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora          acum = {0, 0, 0, 0, 255, 0, 0, 0, 0};

// Snapshot corrente (gráfico 60s)
float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx = 0;
bool    histSnapFull = false;
unsigned long ultimoHistSnap = 0;

// Previsão / Decisão
ResultadoPrev  previsao = {0};
DecisaoSistema decisao  = {false, false, false, 0, false, ""};
unsigned long  ultimaPrev = 0;
bool           buscandoPrev = false;
bool           ehDia = false;

// WiFi
String gWifiSSID, gWifiPASS, gWifiIP, gWifiGW, gWifiMASK, gWifiDNS;

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
void ligarRedeBaixa();   void desligarRedeBaixa();
void ligarRedeAlta();    void desligarRedeAlta();
void ligarLampExt();     void desligarLampExt();
void ligarLampInt();     void desligarLampInt();
void ligarInversor();    void desligarInversor();

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

void  salvarConfigLocal(); void carregarConfigLocal();
void  calibrarPrevisao();
float calcularCorrenteEstimadaHora(float rad, float radDif,
                                    float nuv, float temp);
void  buscarPrevisaoMeteo();
void  tomarDecisoes();
void  verificarReligamentoAuto();
float consumoMedioParaHora(uint8_t horaAlvo, bool* temDados = nullptr);
float consumoMedioParaHoraCache(uint8_t horaAlvo, bool* temDados = nullptr);
float referenciaConsumoAntecip();
float horasAteGeracaoSuperar(float refA);
void  avaliarAntecipacaoRedeAlta();

void controlarEnergia();
void processarRestauroEnergia();

void   monitorarInversor();
void   resetCurtoAlta(); void resetCurtoBaixa();
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
bool iniciarWiFi(const String& ssid, const String& pass,
                 const String& ip,   const String& gw,
                 const String& mask, const String& dns);
void tentarReconectarWiFi();
void sincronizarNTP();

void configurarRotas();
String gerarPagina(const String& titulo, const String& conteudo,
                   bool comNav = true);
String barraProgresso(float pct, const String& label = "");
String statusCard(const String& ico, const String& label,
                  const String& valor, const String& sub,
                  const String& tipo = "");

void handleRoot();      void handleDados();
void handleBateria();   void handleRedes();
void handleSensores();  void handleConfig();
void handleHistorico(); void handlePrevisao();
void handleLogs();
void handleSetupLogin();void handleSetupWifi();
void handleLogin();     void handleLogout();
void handleNotFound();
void handleResetCurto();void handleNTPSync();
void handleReiniciar();
void handleOTAUpload(); void handleOTAResult();

// ═══════════════════════════════════════════════════════════
// [UTIL] UTILITÁRIOS
// ═══════════════════════════════════════════════════════════

// kW inline — evita campos extras na struct
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
    est.horas = (int)h;
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
    String r = String(buf);
    struct tm ti;
    if (getLocalTime(&ti)) {
        time_t agora = mktime(&ti);
        time_t alvo  = agora + (time_t)(h * 3600.0f);
        struct tm tA; localtime_r(&alvo, &tA);
        char bh[8];
        snprintf(bh, sizeof(bh), "%02d:%02d", tA.tm_hour, tA.tm_min);
        r += " (~" + String(bh) + ")";
    }
    return r;
}

// ═══════════════════════════════════════════════════════════
// [LOG] SISTEMA DE LOGS
// ═══════════════════════════════════════════════════════════
void adicionarLog(const char* msg) {
    struct tm ti;
    if (getLocalTime(&ti))
        snprintf(logs[logIndex], LOG_MAX_LEN, "[%02d:%02d:%02d] %s",
                 ti.tm_hour, ti.tm_min, ti.tm_sec, msg);
    else {
        unsigned long s = millis() / 1000;
        snprintf(logs[logIndex], LOG_MAX_LEN, "[%02lu:%02lu:%02lu] %s",
                 s/3600, (s%3600)/60, s%60, msg);
    }
    logIndex = (logIndex + 1) % MAX_LOGS;
    if (totalLogs < MAX_LOGS) totalLogs++;
}

void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf),
             "RAM: %.1fKB", esp_get_free_heap_size() / 1024.0f);
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [LED] INDICADOR
// ═══════════════════════════════════════════════════════════
void setupLED() { pinMode(LED_AZUL, OUTPUT); digitalWrite(LED_AZUL, LOW); }
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
        botaoSegurando = true; botaoPressionadoEm = millis();
    }
    if (!p) botaoSegurando = false;
    if (botaoSegurando &&
        millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET TOTAL por botao");
        for (int i = 0; i < 15; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(80);
            digitalWrite(LED_AZUL, LOW);  delay(80);
        }
        nvs_flash_erase(); nvs_flash_init();
        ESP.restart();
    }
}

// ═══════════════════════════════════════════════════════════
// [RELE] CONTROLE DE RELÉS
// ═══════════════════════════════════════════════════════════
void setupReles() {
    uint8_t pinos[] = {PIN_RELE_INV_ALTA, PIN_RELE_REDE_BAIXA,
                       PIN_RELE_REDE_ALTA, PIN_RELE_LAMP_EXT,
                       PIN_RELE_LAMP_INT};
    for (uint8_t p : pinos) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    pinMode(PIN_STATUS_INV_ALTA, INPUT);
    pinMode(PIN_BKP_BATERIA,     INPUT);
    estados = {false, false, false, false, false};
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
    adicionarLog("Reles: todos OFF");
}

void setRele(uint8_t pin, bool on) { digitalWrite(pin, on ? HIGH : LOW); }

void ligarRedeBaixa()   { setRele(PIN_RELE_REDE_BAIXA, true);  estados.redeBaixa = true;  }
void desligarRedeBaixa(){ setRele(PIN_RELE_REDE_BAIXA, false); estados.redeBaixa = false; }
void ligarRedeAlta()    { setRele(PIN_RELE_REDE_ALTA,  true);  estados.redesAlta = true;  }
void desligarRedeAlta() { setRele(PIN_RELE_REDE_ALTA,  false); estados.redesAlta = false; }
void ligarLampExt()     { setRele(PIN_RELE_LAMP_EXT,   true);  estados.lampExt   = true;  }
void desligarLampExt()  { setRele(PIN_RELE_LAMP_EXT,   false); estados.lampExt   = false; }
void ligarLampInt()     { setRele(PIN_RELE_LAMP_INT,   true);  estados.lampInt   = true;  }
void desligarLampInt()  { setRele(PIN_RELE_LAMP_INT,   false); estados.lampInt   = false; }
void ligarInversor()    { setRele(PIN_RELE_INV_ALTA,   true);  estados.inversor  = true;  }
void desligarInversor() { setRele(PIN_RELE_INV_ALTA,   false); estados.inversor  = false; }

// ═══════════════════════════════════════════════════════════
// [ADC] LEITURA DE SENSORES
// ═══════════════════════════════════════════════════════════
int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        soma += analogRead(pin);
        delayMicroseconds(100);
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

// ── Painel: auto-calibração de offset ─────────────────────
float lerCorrentePainel() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_PAINEL));
    sensores.sinalPainel_mV = mv;

    static int    offsetAprendido = PAINEL_OFFSET_MV;
    static bool   offsetCalibrado = false;
    static long   somaOff = 0;
    static int    amostrasOff = 0;
    static unsigned long tUltimoZero = 0;

    // Auto-calibração noturna (apenas se ativo)
    if (autoCalibPainelAtivo) {
        struct tm ti;
        if (getLocalTime(&ti)) {
            int hora = ti.tm_hour;
            bool noite = (hora >= 22 || hora <= 4);
            if (noite) {
                if (!tUltimoZero) tUltimoZero = millis();
                somaOff += mv; amostrasOff++;
                if (millis() - tUltimoZero >= 30000 && amostrasOff >= 20) {
                    int novo = (int)(somaOff / amostrasOff);
                    if (abs(novo - PAINEL_OFFSET_MV) <= 120) {
                        offsetAprendido = novo;
                        offsetCalibrado = true;
                        if (abs(novo - calibPainel.offsetMV) > 3) {
                            calibPainel.offsetMV = novo;
                            snprintf(tempBuf, sizeof(tempBuf),
                                     "Painel offset noturno: %dmV", novo);
                            adicionarLog(tempBuf);
                        }
                    }
                    somaOff = 0; amostrasOff = 0; tUltimoZero = 0;
                }
            } else {
                tUltimoZero = 0; somaOff = 0; amostrasOff = 0;
            }
        }
    }

    int offsetUsado = (autoCalibPainelAtivo && offsetCalibrado)
                    ? offsetAprendido : calibPainel.offsetMV;

    // Cabos invertidos: sinal desce = painel gerando
    float corrBruta = -(float)(mv - offsetUsado) / calibPainel.sensMvA;

    // Consumo sistema (sinal negativo = ESP32/controlador)
    float sistBruta = 0.0f;
    if (corrBruta < 0.0f) {
        float mag = fabsf(corrBruta);
        if (mag >= 0.30f) {
            sistBruta = mag - 0.15f;
            if (sistBruta < 0) sistBruta = 0;
        }
        corrBruta = 0.0f;
    }
    if (corrBruta < 0.15f) corrBruta = 0.0f;

    float corrMedia = adicionarAmostra(bufPainel, corrBruta);
    if (corrMedia < 0.15f) corrMedia = 0.0f;

    float sistMedia = adicionarAmostra(bufSistema, sistBruta);
    if (sistMedia < 0.09f) sistMedia = 0.0f;
    sensores.correnteSistema = (corrMedia < 0.15f) ? sistMedia : 0.0f;

    return corrMedia;
}

float lerCorrenteCasa() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_CASA));
    sensores.sinalCasa_mV = mv;
    if (mv <= calibCasa.offsetMV) {
        adicionarAmostra(bufCasa, 0.0f); return 0.0f;
    }
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
    sensores.correntePainel  = lerCorrentePainel();
    sensores.correnteCasa    = lerCorrenteCasa();
    sensores.correnteBateria = sensores.correntePainel
                             - sensores.correnteCasa
                             - sensores.correnteSistema;
    sensores.inversoreAtivo  = lerStatusInversor();
    atualizarAhBateria();
    verificarCalibracaoAutomatica();

    // Snapshot 60s para gráfico
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
    // BMS cortou carga = bateria 100%
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
        adicionarLog("Bat calibrada: 100% detectado");
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
    snprintf(tempBuf, sizeof(tempBuf),
             "Cnt: P=%.1f C=%.1f BC=%.1f BD=%.1fAh",
             contadores.painelAh, contadores.casaAh,
             contadores.batCargAh, contadores.batDescAh);
    adicionarLog(tempBuf);
}

void salvarDiaFlash(uint16_t idx, const RegistroDia& dia) {
    char ns[12]; snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);
    prefs.begin(ns, false);
    prefs.putBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();
}

bool carregarDiaFlash(uint16_t idx, RegistroDia& dia) {
    char ns[12]; snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);
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
        adicionarLog("Hist: resetado (v15)");
        return;
    }
    histIndiceAtual = prefs.getUShort("idx",   0);
    histTotalDias   = prefs.getUShort("total", 0);
    prefs.end();
    if (histTotalDias > 0 &&
        !carregarDiaFlash(histIndiceAtual, histDiaAtual))
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
    snprintf(tempBuf, sizeof(tempBuf),
             "Hist: %d/%d dias", histTotalDias, HIST_DIAS);
    adicionarLog(tempBuf);
}

void acumularEnergia() {
    struct tm ti;
    if (!getLocalTime(&ti)) return;
    unsigned long agora = millis();
    uint8_t  hA = ti.tm_hour, dA = ti.tm_mday, mA = ti.tm_mon+1;
    uint16_t aA = ti.tm_year + 1900;

    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum = agora;
        acum.horaAtual  = hA; acum.diaAtual = dA;
        acum.mesAtual   = mA; acum.anoAtual = aA;
        bool diaEx = false;
        if (histTotalDias > 0) {
            RegistroDia dT;
            if (carregarDiaFlash(histIndiceAtual, dT) &&
                dT.dia == dA && dT.mes == mA && dT.ano == aA) {
                histDiaAtual = dT; diaEx = true;
                adicionarLog("Hist: dia recuperado");
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
            histDiaAtual.ano = aA; histDiaAtual.mes = mA;
            histDiaAtual.dia = dA; histDiaAtual.valido = true;
            snprintf(tempBuf, sizeof(tempBuf),
                     "Hist: novo dia %02d/%02d/%04d", dA, mA, aA);
            adicionarLog(tempBuf);
        }
        salvarDiaFlash(histIndiceAtual, histDiaAtual);
        salvarIndiceHistorico();
        salvarContadores();

        acum.painelAh = acum.casaAh = acum.batCargAh = acum.batDescAh = 0;
        acum.horaAtual = hA;
        if (mudouDia) {
            acum.diaAtual = dA; acum.mesAtual = mA; acum.anoAtual = aA;
        }
    }

    // Salva parcial a cada 5min
    static unsigned long tParcial = 0;
    if (agora - tParcial >= 300000UL) {
        tParcial = agora;
        uint8_t hi = hA % HIST_HORAS;
        RegistroDia dT = histDiaAtual;
        dT.horas[hi].painelAh  += acum.painelAh;
        dT.horas[hi].casaAh    += acum.casaAh;
        dT.horas[hi].batCargAh += acum.batCargAh;
        dT.horas[hi].batDescAh += acum.batDescAh;
        calcularTotaisDia(dT);
        salvarDiaFlash(histIndiceAtual, dT);
        salvarContadores();
    }
}

// ═══════════════════════════════════════════════════════════
// [PREV] PREVISÃO METEOROLÓGICA
// ═══════════════════════════════════════════════════════════
void salvarConfigLocal() {
    prefs.begin("rrfs_loc", false);
    prefs.putDouble("lat",    configLocal.latitude);
    prefs.putDouble("lon",    configLocal.longitude);
    prefs.putFloat ("amp",    configLocal.ampControlador);
    prefs.putFloat ("ef",     configLocal.eficiencia);
    prefs.putFloat ("fcalib", configLocal.fatorCalib);
    prefs.putUChar ("ncalib", configLocal.amostrasCalib);
    prefs.putBool  ("conf",   configLocal.configurado);
    prefs.end();
    adicionarLog("Config local salva");
}

void carregarConfigLocal() {
    prefs.begin("rrfs_loc", true);
    configLocal.latitude       = prefs.getDouble("lat",    0.0);
    configLocal.longitude      = prefs.getDouble("lon",    0.0);
    configLocal.ampControlador = prefs.getFloat ("amp",   30.0f);
    configLocal.eficiencia     = prefs.getFloat ("ef",    0.80f);
    configLocal.fatorCalib     = prefs.getFloat ("fcalib", 1.0f);
    configLocal.amostrasCalib  = prefs.getUChar ("ncalib",    0);
    configLocal.configurado    = prefs.getBool  ("conf",  false);
    prefs.end();
    if (configLocal.configurado) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Local: lat=%.6f lon=%.6f amp=%.0fA",
                 configLocal.latitude, configLocal.longitude,
                 configLocal.ampControlador);
        adicionarLog(tempBuf);
    }
}

void calibrarPrevisao() {
    if (!configLocal.configurado || !previsao.valido) return;
    if (sensores.correntePainel < 1.0f) return;
    struct tm ti;
    if (!getLocalTime(&ti)) return;
    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (ph.hora != (uint8_t)ti.tm_hour ||
            ph.dia  != (uint8_t)ti.tm_mday ||
            ph.mes  != (uint8_t)(ti.tm_mon+1)) continue;
        if (ph.correnteEstimada < 0.5f) return;
        float fN = sensores.correntePainel / ph.correnteEstimada;
        if (fN < 0.1f) fN = 0.1f;
        if (fN > 3.0f) fN = 3.0f;
        if (configLocal.amostrasCalib < 50) configLocal.amostrasCalib++;
        float p = 1.0f / (float)configLocal.amostrasCalib;
        configLocal.fatorCalib = configLocal.fatorCalib*(1.0f-p) + fN*p;
        if (configLocal.amostrasCalib % 10 == 0) salvarConfigLocal();
        break;
    }
}

float calcularCorrenteEstimadaHora(float rad, float radDif,
                                    float nuv, float temp) {
    if (!configLocal.configurado ||
        configLocal.ampControlador <= 0) return 0;
    float radT = rad + radDif * 0.5f;
    if (radT < 0) radT = 0;
    float fRad = radT / 1000.0f;
    float fTemp = 1.0f;
    if (temp > 25.0f) {
        fTemp = 1.0f - (temp - 25.0f) * 0.004f;
        if (fTemp < 0.6f) fTemp = 0.6f;
    }
    float c = configLocal.ampControlador * fRad *
              configLocal.eficiencia * fTemp * configLocal.fatorCalib;
    return (c < 0) ? 0 : c;
}

void buscarPrevisaoMeteo() {
    if (!configLocal.configurado ||
        WiFi.status() != WL_CONNECTED || buscandoPrev) return;
    buscandoPrev = true;
    adicionarLog("Prev: buscando...");

    char url[PREV_URL_MAX];
    snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/forecast"
             "?latitude=%.6f&longitude=%.6f"
             "&hourly=direct_normal_irradiance"
             ",diffuse_radiation,cloud_cover,temperature_2m"
             "&timezone=America%%2FSao_Paulo"
             "&forecast_days=2&timeformat=unixtime",
             (float)configLocal.latitude,
             (float)configLocal.longitude);

    WiFiClient client;
    HTTPClient http;
    http.setReuse(false);
    http.setTimeout(15000);
    http.setConnectTimeout(10000);

    if (!http.begin(client, url)) {
        adicionarLog("Prev ERRO: begin()");
        buscandoPrev = false; return;
    }
    http.addHeader("User-Agent", "ESP32-RRFS/15.1");
    http.addHeader("Connection", "close");

    int code = http.GET();
    snprintf(tempBuf, sizeof(tempBuf), "Prev HTTP: %d", code);
    adicionarLog(tempBuf);

    if (code != HTTP_CODE_OK) {
        snprintf(previsao.erro, sizeof(previsao.erro), "HTTP %d", code);
        http.end(); buscandoPrev = false; return;
    }
    if (esp_get_free_heap_size() < 8000) {
        adicionarLog("Prev ERRO: RAM");
        http.end(); buscandoPrev = false; return;
    }

    String payload = http.getString();
    http.end();

    if (payload.length() < 200 ||
        payload.indexOf("\"error\"") >= 0) {
        snprintf(previsao.erro, sizeof(previsao.erro), "Parse falhou");
        buscandoPrev = false; return;
    }

    // Parser inline (sem alocações extras quando possível)
    auto extrairFloat = [&](const char* chave,
                            float* arr, int maxLen) -> int {
        String busca = "\""; busca += chave; busca += "\":[";
        int pos = payload.indexOf(busca);
        if (pos < 0) return 0;
        pos += busca.length();
        if (payload.charAt(pos) == 'n') return 0;
        int count = 0;
        char nb[20]; int ni = 0;
        while (count < maxLen) {
            char ch = payload.charAt(pos);
            if (ch == ']') {
                if (ni > 0) { nb[ni]='\0'; arr[count++]=atof(nb); }
                break;
            } else if (ch == ',') {
                if (ni > 0) { nb[ni]='\0'; arr[count++]=atof(nb); ni=0; }
            } else if (ch == 'n') {
                arr[count++] = 0.0f; pos += 3; ni = 0;
            } else if ((ch>='0'&&ch<='9')||ch=='.'||ch=='-') {
                if (ni < 18) nb[ni++] = ch;
            }
            pos++;
            if (pos >= (int)payload.length()) break;
        }
        return count;
    };

    auto extrairLong = [&](const char* chave,
                           long* arr, int maxLen) -> int {
        String busca = "\""; busca += chave; busca += "\":[";
        int pos = payload.indexOf(busca);
        if (pos < 0) return 0;
        pos += busca.length();
        if (payload.charAt(pos) == 'n') return 0;
        int count = 0;
        char nb[16]; int ni = 0;
        while (count < maxLen) {
            char ch = payload.charAt(pos);
            if (ch == ']') {
                if (ni > 0) { nb[ni]='\0'; arr[count++]=atol(nb); }
                break;
            } else if (ch == ',') {
                if (ni > 0) { nb[ni]='\0'; arr[count++]=atol(nb); ni=0; }
            } else if (ch>='0'&&ch<='9') {
                if (ni < 14) nb[ni++] = ch;
            }
            pos++;
            if (pos >= (int)payload.length()) break;
        }
        return count;
    };

    float* radDir = (float*)malloc(PREV_HORAS * sizeof(float));
    float* radDif = (float*)malloc(PREV_HORAS * sizeof(float));
    float* nuv    = (float*)malloc(PREV_HORAS * sizeof(float));
    float* tmp    = (float*)malloc(PREV_HORAS * sizeof(float));
    long*  stamps = (long*) malloc(PREV_HORAS * sizeof(long));

    if (!radDir||!radDif||!nuv||!tmp||!stamps) {
        adicionarLog("Prev ERRO: malloc");
        if (radDir) free(radDir); if (radDif) free(radDif);
        if (nuv) free(nuv); if (tmp) free(tmp); if (stamps) free(stamps);
        buscandoPrev = false; return;
    }

    memset(radDir,0,PREV_HORAS*sizeof(float));
    memset(radDif,0,PREV_HORAS*sizeof(float));
    memset(nuv,   0,PREV_HORAS*sizeof(float));
    memset(tmp,   0,PREV_HORAS*sizeof(float));
    memset(stamps,0,PREV_HORAS*sizeof(long));

    int nTs  = extrairLong ("time",                     stamps, PREV_HORAS);
    int nRad = extrairFloat("direct_normal_irradiance", radDir, PREV_HORAS);
    int nDif = extrairFloat("diffuse_radiation",        radDif, PREV_HORAS);
    int nNuv = extrairFloat("cloud_cover",              nuv,    PREV_HORAS);
    int nTmp = extrairFloat("temperature_2m",           tmp,    PREV_HORAS);

    payload = ""; payload.clear();

    if (nTs < 1 || nRad < 1) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "Parse: ts=%d rad=%d", nTs, nRad);
        free(radDir); free(radDif); free(nuv); free(tmp); free(stamps);
        buscandoPrev = false; return;
    }

    struct tm ti;
    if (!getLocalTime(&ti)) {
        adicionarLog("Prev ERRO: NTP");
        free(radDir); free(radDif); free(nuv); free(tmp); free(stamps);
        buscandoPrev = false; return;
    }

    int total = nTs;
    if (nRad < total) total = nRad;
    if (total > PREV_HORAS) total = PREV_HORAS;

    previsao.totalHoras = 0;
    previsao.correnteHojeRestante = previsao.correnteAmanha = 0;

    struct tm tiA = ti;
    tiA.tm_mday += 1; tiA.tm_hour = 0;
    tiA.tm_min = 0; tiA.tm_sec = 0;
    mktime(&tiA);

    int cHR = 0, cA = 0;
    int hojeAno = ti.tm_year+1900, hojeMes = ti.tm_mon+1;
    int hojeDia = ti.tm_mday, hojeH = ti.tm_hour;

    for (int i = 0; i < total; i++) {
        time_t ts = (time_t)stamps[i];
        struct tm tl; localtime_r(&ts, &tl);
        PrevisaoHora& ph = previsao.horas[i];
        ph.hora = tl.tm_hour; ph.dia = tl.tm_mday;
        ph.mes  = tl.tm_mon+1; ph.ano = tl.tm_year+1900;
        ph.radiacao       = i<nRad ? radDir[i] : 0;
        ph.radiacaoDifusa = i<nDif ? radDif[i] : 0;
        ph.coberturaNuvens= i<nNuv ? nuv[i]   : 0;
        ph.temperatura    = i<nTmp ? tmp[i]   : 25.0f;
        ph.valido = true;
        if (ph.radiacao < 0)        ph.radiacao = 0;
        if (ph.radiacaoDifusa < 0)  ph.radiacaoDifusa = 0;
        if (ph.coberturaNuvens < 0) ph.coberturaNuvens = 0;
        if (ph.coberturaNuvens >100)ph.coberturaNuvens = 100;
        ph.correnteEstimada = calcularCorrenteEstimadaHora(
            ph.radiacao, ph.radiacaoDifusa,
            ph.coberturaNuvens, ph.temperatura);
        if (ph.dia==hojeDia && ph.mes==hojeMes &&
            ph.ano==hojeAno && ph.hora>=(uint8_t)hojeH) {
            previsao.correnteHojeRestante += ph.correnteEstimada;
            cHR++;
        }
        if (ph.dia==(uint8_t)tiA.tm_mday &&
            ph.mes==(uint8_t)(tiA.tm_mon+1)) {
            previsao.correnteAmanha += ph.correnteEstimada; cA++;
        }
        previsao.totalHoras++;
    }
    if (cHR > 0) previsao.correnteHojeRestante /= cHR;
    if (cA  > 0) previsao.correnteAmanha       /= cA;

    free(radDir); free(radDif); free(nuv); free(tmp); free(stamps);

    previsao.valido = true;
    previsao.ultimaAtualizacao = millis();
    ultimaPrev = millis();
    memset(previsao.erro, 0, sizeof(previsao.erro));

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev OK: %dh hoje=%.1fA amanha=%.1fA",
             previsao.totalHoras,
             previsao.correnteHojeRestante,
             previsao.correnteAmanha);
    adicionarLog(tempBuf);
    logRAM();
    tomarDecisoes();
    buscandoPrev = false;
}

// ═══════════════════════════════════════════════════════════
// [PREV] DECISÕES E HISTÓRICO DE CONSUMO
// ═══════════════════════════════════════════════════════════
float consumoMedioParaHora(uint8_t horaAlvo, bool* temDados) {
    if (horaAlvo > 23) horaAlvo = 23;
    if (temDados) *temDados = false;
    float soma = 0; int dias = 0;
    int maxD = (histTotalDias < 14) ? histTotalDias : 14;
    for (int i = 0; i < maxD; i++) {
        RegistroDia dT;
        bool ok = (i==0) ? (dT=histDiaAtual, true)
                         : carregarDiaFlash(
                             ((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,
                             dT);
        if (ok && dT.valido && dT.horas[horaAlvo].casaAh > 0) {
            soma += dT.horas[horaAlvo].casaAh; dias++;
        }
    }
    if (dias > 0) { if (temDados) *temDados = true; return soma/dias; }
    // Fallback: média diária
    float geral = 0; int dG = 0;
    int maxDG = (histTotalDias < 7) ? histTotalDias : 7;
    for (int i = 0; i < maxDG; i++) {
        RegistroDia dT;
        bool ok = (i==0) ? (dT=histDiaAtual, true)
                         : carregarDiaFlash(
                             ((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,
                             dT);
        if (ok && dT.valido && dT.totalCasaAh > 0) {
            geral += dT.totalCasaAh / 24.0f; dG++;
        }
    }
    return dG > 0 ? (geral/dG) : 5.0f;
}

float consumoMedioParaHoraCache(uint8_t horaAlvo, bool* temDados) {
    if (horaAlvo > 23) horaAlvo = 23;
    static float  cacheV[24] = {0};
    static bool   cacheT[24] = {false};
    static bool   cacheOk[24]= {false};
    static uint8_t epochH = 255;
    uint8_t hNow = 255;
    struct tm ti;
    if (getLocalTime(&ti)) hNow = ti.tm_hour;
    if (hNow != epochH) {
        for (int i=0;i<24;i++) cacheOk[i]=false;
        epochH = hNow;
    }
    if (!cacheOk[horaAlvo]) {
        bool td = false;
        cacheV[horaAlvo] = consumoMedioParaHora(horaAlvo, &td);
        cacheT[horaAlvo] = td;
        cacheOk[horaAlvo] = true;
    }
    if (temDados) *temDados = cacheT[horaAlvo];
    return cacheV[horaAlvo];
}

float horasAteGeracaoSuperar(float refA) {
    if (!previsao.valido) return -1;
    time_t now = time(nullptr);
    if (now < 1000000) return -1;
    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (!ph.valido) continue;
        struct tm tC = {0};
        tC.tm_year = ph.ano-1900; tC.tm_mon  = ph.mes-1;
        tC.tm_mday = ph.dia;      tC.tm_hour = ph.hora;
        tC.tm_isdst = -1;
        time_t tsH = mktime(&tC);
        if (tsH <= now) continue;
        if (ph.correnteEstimada > refA)
            return (float)(tsH - now) / 3600.0f;
    }
    return -1;
}

void tomarDecisoes() {
    if (!previsao.valido || !bateria.calibrada) return;
    struct tm ti;
    if (!getLocalTime(&ti)) return;

    int  hA    = ti.tm_hour;
    bool noite = (hA >= 18 || hA < 6);

    // Consumo médio geral
    float consAh = 0; int dV = 0;
    int maxD = (histTotalDias<7)?histTotalDias:7;
    for (int i = 0; i < maxD; i++) {
        RegistroDia dT;
        bool ok = (i==0)?(dT=histDiaAtual,true)
                        :carregarDiaFlash(
                            ((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,dT);
        if (ok && dT.valido && dT.totalCasaAh > 0) {
            consAh += dT.totalCasaAh/24.0f; dV++;
        }
    }
    if (dV > 0) consAh /= dV; else consAh = 5.0f;

    float ahAt  = bateria.ahAtual;
    float ahMin = bateria.ahDisponivel * (redes.corteBaixa / 100.0f);

    decisao.desligarRedeAltaNoite = false;
    decisao.desligarLampExtNoite  = false;
    decisao.desligarLampIntDia    = false;
    decisao.alertaEnergiaInsuficiente = false;

    if (noite) {
        float horaGer = 6.0f;
        for (int i = 0; i < previsao.totalHoras; i++) {
            PrevisaoHora& ph = previsao.horas[i];
            if (ph.correnteEstimada > 1.0f &&
                (ph.dia==ti.tm_mday || ph.dia==ti.tm_mday+1)) {
                horaGer = ph.hora; break;
            }
        }
        float horasAte = (hA>=18) ? (24.0f-hA+horaGer) : (horaGer-hA);
        float ahNec    = consAh * horasAte;
        float ahDisp   = ahAt - ahMin;
        float saldo    = ahDisp - ahNec;
        float consAlta = consAh * 0.40f;

        decisao.horaLigarRedeAlta = horaGer + 1.0f;

        if (saldo < 0 || saldo < consAlta * 3.0f) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Noite: saldo=%.1fAh disp=%.1fAh nec=%.1fAh",
                     saldo, ahDisp, ahNec);
        }
        if (sensores.correntePainel <= 0.5f) {
            float lampAh = consAh * 0.05f;
            if (ahDisp < lampAh * horasAte)
                decisao.desligarLampExtNoite = true;
        }
        if (saldo < 0 && previsao.correnteAmanha < consAh * 0.5f)
            decisao.alertaEnergiaInsuficiente = true;

        if (decisao.desligarRedeAltaNoite &&
            estados.redesAlta &&
            curto.estado == CURTO_NORMAL) {
            desligarRedeAlta();
            antecipAlta.estado    = ANTECIP_IDLE;
            antecipAlta.tCooldown = millis() + T_ANTECIP_COOLDOWN;
            adicionarLog("NOITE: Rede Alta OFF");
            adicionarLog(decisao.motivo);
        }
        if (decisao.desligarLampExtNoite && estados.lampExt) {
            desligarLampExt(); adicionarLog("NOITE: Lamp.Ext OFF");
        }
    } else {
        float consRest = 0;
        for (int h = hA; h < 24; h++)
            consRest += consumoMedioParaHoraCache((uint8_t)h);
        float gerRest = 0;
        for (uint8_t i = 0; i < previsao.totalHoras; i++) {
            PrevisaoHora& ph = previsao.horas[i];
            if (!ph.valido) continue;
            if (ph.dia==(uint8_t)ti.tm_mday &&
                ph.mes==(uint8_t)(ti.tm_mon+1) &&
                ph.hora>=(uint8_t)hA)
                gerRest += ph.correnteEstimada;
        }
        float resAh  = bateria.ahDisponivel*(redes.corteBaixa/100.0f);
        float saldoD = (ahAt + gerRest) - (consRest + resAh);
        if (saldoD < 0 && estados.redesAlta &&
            curto.estado == CURTO_NORMAL &&
            antecipAlta.estado == ANTECIP_IDLE) {
            desligarRedeAlta();
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Dia: deficit=%.1fAh", saldoD);
            adicionarLog("DIA: Rede Alta OFF - deficit");
        } else if (saldoD < 0 &&
            bateria.percentualAtual <= redes.corteLampInt + 10.0f) {
            decisao.desligarLampIntDia = true;
            if (estados.lampInt) {
                desligarLampInt();
                adicionarLog("DIA: Lamp.Int OFF - deficit");
            }
        }
    }
}

void verificarReligamentoAuto() {
    if (!previsao.valido ||
        !decisao.desligarRedeAltaNoite ||
        curto.estado != CURTO_NORMAL) return;
    struct tm ti;
    if (!getLocalTime(&ti)) return;

    bool pode = (sensores.correntePainel > 0.5f &&
                 bateria.percentualAtual > redes.corteAlta+5.0f &&
                 sensores.inversoreAtivo) ||
                ((float)ti.tm_hour >= decisao.horaLigarRedeAlta &&
                 bateria.percentualAtual > redes.corteAlta+2.0f &&
                 sensores.inversoreAtivo);

    if (pode && !estados.redesAlta) {
        ligarRedeAlta();
        decisao.desligarRedeAltaNoite = false;
        adicionarLog("AUTO: Rede Alta ON - manha");
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] ANTECIPAÇÃO REDE ALTA
// ═══════════════════════════════════════════════════════════
void avaliarAntecipacaoRedeAlta() {
    if (curto.estado != CURTO_NORMAL) {
        antecipAlta.estado = ANTECIP_IDLE; return;
    }
    if (!bateria.calibrada) return;
    unsigned long agora = millis();
    float pct = bateria.percentualAtual;
    if (pct > redes.corteAlta + 2.0f) {
        antecipAlta.estado = ANTECIP_IDLE; return;
    }

    switch (antecipAlta.estado) {
        case ANTECIP_IDLE: {
            if (antecipAlta.tCooldown && agora < antecipAlta.tCooldown) return;
            antecipAlta.tCooldown = 0;
            if (estados.redesAlta ||
                !estados.inversor || !sensores.inversoreAtivo) return;
            if (!previsao.valido) return;
            if (pct <= redes.corteBaixa+5.0f) return;
            if (sensores.correntePainel < 0.3f) return;

            struct tm ti;
            if (!getLocalTime(&ti)) return;
            float genP = -1;
            for (uint8_t i = 0; i < previsao.totalHoras; i++) {
                PrevisaoHora& ph = previsao.horas[i];
                if (!ph.valido) continue;
                if (ph.dia==(uint8_t)ti.tm_mday &&
                    ph.mes==(uint8_t)(ti.tm_mon+1) &&
                    ph.hora>(uint8_t)ti.tm_hour) {
                    genP = ph.correnteEstimada; break;
                }
            }
            if (genP < 0 || genP < sensores.correntePainel*0.8f) return;
            ligarRedeAlta();
            antecipAlta.estado  = ANTECIP_TESTANDO;
            antecipAlta.tInicio = agora;
            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIP: teste bat=%.0f%% genP=%.1fA",
                     pct, genP);
            adicionarLog(tempBuf);
            break;
        }
        case ANTECIP_TESTANDO: {
            if (!estados.redesAlta) { antecipAlta.estado=ANTECIP_IDLE; return; }
            if (agora-antecipAlta.tInicio < T_ANTECIP_ESTABILIZA) return;
            float ibat = sensores.correnteBateria;
            if (ibat >= -0.3f) {
                antecipAlta.estado = ANTECIP_MANTIDO;
                antecipAlta.tUltimaAval = agora;
                adicionarLog("ANTECIP: mantida - geracao cobre");
                return;
            }
            float ahCorte = bateria.ahAtual -
                            (bateria.ahDisponivel*(redes.corteBaixa/100.0f));
            float hCorte  = (ahCorte<=0)?0.0f:(ahCorte/fabsf(ibat));
            float hMel    = horasAteGeracaoSuperar(sensores.correnteCasa*0.8f);
            float mrgH    = MARGEM_ANTECIP_MIN / 60.0f;
            bool ok = (hMel>=0) && (hCorte>hMel+mrgH);
            if (ok) {
                antecipAlta.estado = ANTECIP_MANTIDO;
                antecipAlta.tUltimaAval = agora;
                adicionarLog("ANTECIP: mantida - melhora a tempo");
            } else {
                desligarRedeAlta();
                antecipAlta.estado = ANTECIP_IDLE;
                antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
                adicionarLog("ANTECIP: OFF - nao favoravel");
            }
            break;
        }
        case ANTECIP_MANTIDO: {
            if (!estados.redesAlta) { antecipAlta.estado=ANTECIP_IDLE; return; }
            if (agora-antecipAlta.tUltimaAval < T_ANTECIP_REAVAL) return;
            antecipAlta.tUltimaAval = agora;
            float ibat = sensores.correnteBateria;
            if (ibat >= -0.3f) return;
            float ahC = bateria.ahAtual -
                        (bateria.ahDisponivel*(redes.corteBaixa/100.0f));
            float hC  = (ahC<=0)?0.0f:(ahC/fabsf(ibat));
            float hM  = horasAteGeracaoSuperar(sensores.correnteCasa*0.8f);
            float mH  = MARGEM_ANTECIP_MIN / 60.0f;
            if (!((hM>=0)&&(hC>hM+mH))) {
                desligarRedeAlta();
                antecipAlta.estado = ANTECIP_IDLE;
                antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
                adicionarLog("ANTECIP: OFF - reavaliacao");
            }
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] RESTAURAÇÃO NÃO-BLOQUEANTE
// ═══════════════════════════════════════════════════════════
void processarRestauroEnergia() {
    if (restauro.estado == RESTORE_OCIOSO) return;
    unsigned long agora = millis();
    switch (restauro.estado) {
        case RESTORE_LIGANDO_BAIXA:
            if (!estados.redeBaixa) {
                ligarRedeBaixa(); adicionarLog("Restauro: Rede Baixa ON");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_AGUARDANDO_BAIXA;
            break;
        case RESTORE_AGUARDANDO_BAIXA:
            if (agora-restauro.tInicio >= RESTORE_T_BAIXA)
                restauro.estado = RESTORE_LIGANDO_INVERSOR;
            break;
        case RESTORE_LIGANDO_INVERSOR:
            if (!estados.inversor) {
                ligarInversor(); adicionarLog("Restauro: Inversor ON");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_AGUARDANDO_INVERSOR_BOOT;
            break;
        case RESTORE_AGUARDANDO_INVERSOR_BOOT:
            if (agora-restauro.tInicio >= RESTORE_T_INV_BOOT) {
                restauro.tInicio = agora;
                restauro.estado  = RESTORE_CHECANDO_STATUS_INV;
            }
            break;
        case RESTORE_CHECANDO_STATUS_INV:
            if (lerStatusInversor() ||
                agora-restauro.tInicio >= RESTORE_T_INV_STATUS)
                restauro.estado = RESTORE_LIGANDO_ALTA;
            break;
        case RESTORE_LIGANDO_ALTA:
            if (!estados.redesAlta &&
                bateria.percentualAtual > redes.corteAlta+2.0f &&
                sensores.inversoreAtivo) {
                ligarRedeAlta(); adicionarLog("Restauro: Rede Alta ON");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_LIGANDO_LAMP_INT;
            break;
        case RESTORE_LIGANDO_LAMP_INT:
            if (agora-restauro.tInicio >= RESTORE_T_ALTA) {
                if (!estados.lampInt &&
                    bateria.percentualAtual > redes.corteLampInt+2.0f &&
                    sensores.inversoreAtivo) {
                    ligarLampInt(); adicionarLog("Restauro: Lamp.Int ON");
                }
                restauro.estado = RESTORE_CONCLUIDO;
            }
            break;
        default:
            restauro.estado = RESTORE_OCIOSO; break;
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] CONTROLE DE ENERGIA (MODO MANUAL)
// ═══════════════════════════════════════════════════════════
void controlarEnergia() {
    if (!bateria.calibrada) return;
    float pct  = bateria.percentualAtual;
    unsigned long agora = millis();

    // ── Lâmpada Externa: controle solar (dia/noite) ────────
    {
        float horasLuz = 12.0f, nascerSol = 6.0f, porSol = 18.0f;
        struct tm ti;
        if (getLocalTime(&ti) && configLocal.configurado) {
            int mes = ti.tm_mon+1;
            float latR = (float)(configLocal.latitude * M_PI / 180.0);
            float dec  = -23.45f*cosf(2.0f*M_PI*(float)(mes+9.5f)/12.0f);
            float dR   = dec * M_PI / 180.0f;
            float cH   = -tanf(latR)*tanf(dR);
            if (cH>=-1.0f&&cH<=1.0f)
                horasLuz=(2.0f/15.0f)*(acosf(cH)*180.0f/M_PI);
            nascerSol = 12.0f - horasLuz/2.0f;
            porSol    = 12.0f + horasLuz/2.0f;
            float hF  = (float)ti.tm_hour + (float)ti.tm_min/60.0f;
            ehDia = (hF>=(nascerSol+0.5f) && hF<(porSol+0.5f));
        } else {
            if (!ehDia && sensores.correntePainel > 1.0f)      ehDia = true;
            else if (ehDia && sensores.correntePainel < 0.3f)  ehDia = false;
        }

        static unsigned long tMudaLE = 0;
        static bool ehDiaConf = false;
        static bool primeira = true;
        if (primeira) { primeira = false; ehDiaConf = ehDia; }

        if (ehDia != ehDiaConf) {
            if (!tMudaLE) tMudaLE = agora;
            else if (agora - tMudaLE >= 60000UL) {
                ehDiaConf = ehDia; tMudaLE = 0;
                if (ehDiaConf) {
                    if (estados.lampExt) {
                        desligarLampExt();
                        adicionarLog("LampExt OFF: amanheceu");
                    }
                } else {
                    if (!estados.lampExt &&
                        pct > redes.corteLampExt+2.0f &&
                        estados.inversor && sensores.inversoreAtivo) {
                        ligarLampExt(); adicionarLog("LampExt ON: anoiteceu");
                    }
                }
            }
        } else {
            tMudaLE = 0;
            if (ehDiaConf && estados.lampExt) {
                desligarLampExt(); adicionarLog("LampExt OFF: correcao dia");
            } else if (!ehDiaConf && !estados.lampExt &&
                        pct > redes.corteLampExt+2.0f &&
                        estados.inversor && sensores.inversoreAtivo) {
                ligarLampExt(); adicionarLog("LampExt ON: correcao noite");
            }
        }
    }

    // ── Lâmpada Interna ────────────────────────────────────
    if (pct <= redes.corteLampInt && estados.lampInt) {
        if (!timers.pendenteLampInt) {
            timers.pendenteLampInt = true;
            timers.inicioLampInt   = agora;
        } else if (agora-timers.inicioLampInt >= RELE_BUFFER_MS) {
            desligarLampInt();
            timers.pendenteLampInt = false;
            adicionarLog("LampInt OFF: bat baixa");
        }
    } else {
        timers.pendenteLampInt = false;
        if (pct > redes.corteLampInt+2.0f && !estados.lampInt &&
            estados.inversor && sensores.inversoreAtivo) {
            ligarLampInt(); adicionarLog("LampInt ON: recuperada");
        }
    }

    // ── Lâmpada Externa: corte por energia ────────────────
    if (pct <= redes.corteLampExt && estados.lampExt) {
        if (!timers.pendenteLampExt) {
            timers.pendenteLampExt = true;
            timers.inicioLampExt   = agora;
        } else if (agora-timers.inicioLampExt >= RELE_BUFFER_MS) {
            desligarLampExt();
            timers.pendenteLampExt = false;
            adicionarLog("LampExt OFF: bat baixa");
        }
    } else {
        timers.pendenteLampExt = false;
    }

    // ── Rede Alta ──────────────────────────────────────────
    bool antAti = (antecipAlta.estado != ANTECIP_IDLE);
    if (pct <= redes.corteAlta && estados.redesAlta && !antAti) {
        if (!timers.pendenteAlta) {
            timers.pendenteAlta = true;
            timers.inicioAlta   = agora;
        } else if (agora-timers.inicioAlta >= RELE_BUFFER_MS) {
            desligarRedeAlta();
            timers.pendenteAlta = false;
            adicionarLog("Rede Alta OFF: bat baixa");
        }
    } else {
        timers.pendenteAlta = false;
        if (pct > redes.corteAlta+2.0f && !estados.redesAlta &&
            estados.inversor && sensores.inversoreAtivo) {
            ligarRedeAlta(); adicionarLog("Rede Alta ON: recuperada");
        }
    }

    // ── Rede Baixa + Inversor ──────────────────────────────
    if (pct <= redes.corteBaixa &&
        (estados.redeBaixa || estados.inversor)) {
        restauro.estado       = RESTORE_OCIOSO;
        antecipAlta.estado    = ANTECIP_IDLE;
        antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
        if (!timers.pendenteBaixa) {
            timers.pendenteBaixa = true;
            timers.inicioBaixa   = agora;
        } else if (agora-timers.inicioBaixa >= RELE_BUFFER_MS) {
            if (estados.redesAlta)  { desligarRedeAlta();  delay(200); }
            if (estados.lampExt)    { desligarLampExt();   delay(100); }
            if (estados.lampInt)    { desligarLampInt();   delay(100); }
            if (estados.inversor)   { desligarInversor();  delay(500); }
            if (estados.redeBaixa)  { desligarRedeBaixa(); }
            timers.pendenteBaixa = false;
            adicionarLog("CRITICO: Tudo OFF");
        }
    } else {
        timers.pendenteBaixa = false;
        if (pct > redes.corteBaixa+2.0f &&
            (!estados.redeBaixa || !estados.inversor) &&
            restauro.estado == RESTORE_OCIOSO) {
            restauro.estado = RESTORE_LIGANDO_BAIXA;
            adicionarLog("Restauro iniciado");
        }
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
    bool emProc = (curto.estado==CURTO_RELIGANDO ||
                   curto.estado==CURTO_TESTANDO_ALTA ||
                   curto.estado==CURTO_TESTANDO_BAIXA);

    if (!estados.inversor && !emProc) {
        if (curto.estado == CURTO_NORMAL) {
            curto.monitorando = false;
            curto.tInicio = 0;
            curto.tentativasReligar = 0;
        }
        return;
    }

    if (curto.estado == CURTO_NORMAL) {
        bool st = lerStatusInversor();
        if (st) {
            curto.tInicio = 0;
            curto.monitorando = true;
            curto.tentativasReligar = 0;
            return;
        }
        if (curto.monitorando) {
            if (!curto.tInicio) {
                curto.tInicio = agora;
                adicionarLog("Inv: status perdido, aguardando...");
                return;
            }
            if (agora - curto.tInicio < T_STATUS_AUSENTE) return;
            adicionarLog("Inv: protecao ativa!");
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
                if (agora-curto.tInicio < T_PAUSA_RELIGAR) return;
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
                    curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                    curto.tInicio = 0; curto.monitorando = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Inv OK apos religar");
                    return;
                }
                if (agora-curto.tInicio >= T_AGUARDA_STATUS) {
                    curto.tentativasReligar++;
                    _desligarInversorComBuffer();
                    curto.tInicio = agora;
                    if (curto.tentativasReligar < MAX_TENTATIVAS_RELIGAR) {
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    } else {
                        curto.estado = CURTO_TESTANDO_ALTA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        adicionarLog("=== DIAGNOSTICO CURTO ===");
                    }
                }
                break;
            }
            default:
                curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                break;
        }
        return;
    }

    if (curto.estado == CURTO_TESTANDO_ALTA) {
        switch (curto.subEstado) {
            case SUB_INVERSOR_DESLIGADO:
                if (agora-curto.tInicio < T_AGUARDA_QUEDA) return;
                desligarRedeAlta(); delay(300);
                _ligarInversorComBuffer();
                curto.tInicio = agora; curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("TesteAlta: sem rede alta");
                break;
            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("TesteAlta: OK sem alta");
                } else if (agora-curto.tInicio >= T_AGUARDA_STATUS) {
                    _desligarInversorComBuffer();
                    curto.estado = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio = agora;
                    adicionarLog("TesteAlta: sem resp, testa baixa");
                }
                break;
            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora-curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        _desligarInversorComBuffer();
                        curto.estado = CURTO_TESTANDO_BAIXA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        curto.tInicio = agora;
                        adicionarLog("TesteAlta: instavel");
                    }
                    return;
                }
                ligarRedeAlta();
                curto.tInicio = agora;
                curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                adicionarLog("TesteAlta: ligando alta...");
                break;
            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora-curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor() && agora-curto.tInicio > 500) {
                        adicionarLog("=== CURTO REDE ALTA ===");
                        desligarRedeAlta(); delay(200);
                        _desligarInversorComBuffer(); delay(500);
                        _ligarInversorComBuffer(); delay(2000);
                        curto.estado = CURTO_CONFIRMADO_ALTA;
                        curto.redeAltaCurto = true;
                        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                        adicionarLog("Rede Alta BLOQUEADA");
                    }
                    return;
                }
                if (lerStatusInversor()) {
                    curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                    curto.tInicio = 0; curto.monitorando = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("TesteAlta: OK");
                } else {
                    _desligarInversorComBuffer(); desligarRedeAlta();
                    curto.estado = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio = agora;
                }
                break;
            default: break;
        }
        return;
    }

    if (curto.estado == CURTO_TESTANDO_BAIXA) {
        switch (curto.subEstado) {
            case SUB_INVERSOR_DESLIGADO:
                if (agora-curto.tInicio < T_AGUARDA_QUEDA) return;
                desligarRedeAlta(); desligarRedeBaixa(); delay(300);
                _ligarInversorComBuffer();
                curto.tInicio = agora; curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("TesteBaixa: sem cargas");
                break;
            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("TesteBaixa: OK sem cargas");
                } else if (agora-curto.tInicio >= T_AGUARDA_STATUS) {
                    _desligarInversorComBuffer();
                    curto.estado = CURTO_AGUARDANDO_RESET;
                    curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                    adicionarLog("=== FALHA INVERSOR ===");
                }
                break;
            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora-curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        _desligarInversorComBuffer();
                        curto.estado = CURTO_AGUARDANDO_RESET;
                        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                        adicionarLog("=== INV INSTAVEL SEM CARGA ===");
                    }
                    return;
                }
                ligarRedeBaixa();
                curto.tInicio = agora;
                curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                adicionarLog("TesteBaixa: ligando baixa...");
                break;
            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora-curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor() && agora-curto.tInicio > 500) {
                        adicionarLog("=== CURTO REDE BAIXA ===");
                        desligarRedeBaixa(); delay(200);
                        _desligarInversorComBuffer(); delay(500);
                        curto.estado = CURTO_CONFIRMADO_BAIXA;
                        curto.redeBaixaCurto = true;
                        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                        adicionarLog("Rede Baixa BLOQUEADA");
                    }
                    return;
                }
                if (lerStatusInversor()) {
                    ligarRedeAlta(); delay(300);
                    ligarLampExt(); delay(200); ligarLampInt();
                    curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                    curto.tInicio = 0; curto.monitorando = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("TesteBaixa: OK - tudo religado");
                } else {
                    _desligarInversorComBuffer(); desligarRedeBaixa();
                    curto.estado = CURTO_AGUARDANDO_RESET;
                    curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                }
                break;
            default: break;
        }
        return;
    }

    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        if (estados.redesAlta) desligarRedeAlta();
        return;
    }
    if (curto.estado == CURTO_CONFIRMADO_BAIXA ||
        curto.estado == CURTO_AGUARDANDO_RESET) {
        if (estados.inversor)  _desligarInversorComBuffer();
        if (estados.redeBaixa) desligarRedeBaixa();
        if (estados.redesAlta) desligarRedeAlta();
    }
}

void resetCurtoAlta() {
    if (curto.estado != CURTO_CONFIRMADO_ALTA) return;
    adicionarLog("Reset curto alta");
    desligarRedeAlta(); delay(500); ligarRedeAlta(); delay(300);
    if (lerStatusInversor()) {
        curto.estado = CURTO_NORMAL; curto.redeAltaCurto = false;
        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
        curto.monitorando = true;
        adicionarLog("Curto alta RESOLVIDO");
    } else {
        desligarRedeAlta();
        adicionarLog("AVISO: curto alta NAO resolvido!");
    }
}

void resetCurtoBaixa() {
    if (curto.estado != CURTO_CONFIRMADO_BAIXA &&
        curto.estado != CURTO_AGUARDANDO_RESET) return;
    adicionarLog("Reset curto baixa");
    desligarRedeBaixa(); desligarInversor(); delay(500);
    ligarInversor();
    unsigned long t = millis(); bool ok = false;
    while (millis()-t < T_ESPERA_STATUS_INV) {
        if (lerStatusInversor()) { ok = true; break; }
        delay(100);
    }
    if (!ok) { desligarInversor(); adicionarLog("ERRO: Inv nao ligou!"); return; }
    ligarRedeBaixa(); delay(500);
    if (lerStatusInversor()) {
        curto.estado = CURTO_NORMAL; curto.redeBaixaCurto = false;
        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
        curto.monitorando = true;
        adicionarLog("Curto baixa RESOLVIDO");
    } else {
        desligarRedeBaixa(); desligarInversor();
        curto.estado = CURTO_CONFIRMADO_BAIXA;
        adicionarLog("AVISO: curto baixa NAO resolvido!");
    }
}

String htmlStatusCurto() {
    if (curto.estado == CURTO_NORMAL &&
        !curto.redeAltaCurto && !curto.redeBaixaCurto)
        return F("<div class='ok' style='font-size:12px'>"
                 "&#10003; Sem curto</div>");
    if (curto.estado == CURTO_TESTANDO_ALTA ||
        curto.estado == CURTO_TESTANDO_BAIXA)
        return F("<div class='wn'>&#9874; Diagnosticando... Aguarde.</div>");
    String h = "";
    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        h += F("<div class='er'><b>&#9888; CURTO REDE ALTA!</b><br>"
               "1. Resolva o curto.<br>2. Pressione o botao.</div>");
        h += F("<form method='POST' action='/reset-curto'>"
               "<input type='hidden' name='rede' value='alta'>"
               "<button type='submit' class='btn-g' style='margin-top:8px'>"
               "&#10003; Testar e Religar</button></form>");
    } else if (curto.estado == CURTO_CONFIRMADO_BAIXA) {
        h += F("<div class='er'><b>&#9888; CURTO REDE BAIXA!</b><br>"
               "1. Resolva o curto.<br>2. Pressione o botao.</div>");
        h += F("<form method='POST' action='/reset-curto'>"
               "<input type='hidden' name='rede' value='baixa'>"
               "<button type='submit' class='btn-g' style='margin-top:8px'>"
               "&#10003; Testar e Religar</button></form>");
    } else if (curto.estado == CURTO_AGUARDANDO_RESET) {
        h += F("<div class='er'><b>&#9888; FALHA NO INVERSOR!</b></div>");
        h += F("<form method='POST' action='/reset-curto'>"
               "<input type='hidden' name='rede' value='inversor'>"
               "<button type='submit' class='btn-o' style='margin-top:8px'>"
               "&#9889; Tentar Religar</button></form>");
    }
    return h;
}

// ═══════════════════════════════════════════════════════════
// [PREF] PREFERENCES
// ═══════════════════════════════════════════════════════════
bool existeCredenciais() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("user"); prefs.end(); return ok;
}
bool existeWiFiConfig() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("wifi_ok"); prefs.end(); return ok;
}
void salvarCredenciais(const String& u, const String& p) {
    prefs.begin("rrfs", false);
    prefs.putString("user", u); prefs.putString("pass", p);
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
    prefs.putString("wifi_ssid", ssid); prefs.putString("wifi_pass", pass);
    prefs.putString("wifi_ip",   ip);   prefs.putString("wifi_gw",   gw);
    prefs.putString("wifi_mask", mask); prefs.putString("wifi_dns",  dns);
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
    prefs.putFloat("r_baixa",  redes.corteBaixa);
    prefs.putFloat("r_alta",   redes.corteAlta);
    prefs.putFloat("r_lmpext", redes.corteLampExt);
    prefs.putFloat("r_lmpint", redes.corteLampInt);
    prefs.end();
}
void carregarConfigRedes() {
    prefs.begin("rrfs", true);
    redes.corteBaixa   = prefs.getFloat("r_baixa",  10.0f);
    redes.corteAlta    = prefs.getFloat("r_alta",   30.0f);
    redes.corteLampExt = prefs.getFloat("r_lmpext", 50.0f);
    redes.corteLampInt = prefs.getFloat("r_lmpint",  0.0f);
    prefs.end();
}

// Calibração de sensores — persistente após reboot
void carregarCalibSensores() {
    // Valores de fábrica
    calibPainel.offsetMV  = PAINEL_OFFSET_MV;
    calibPainel.sensMvA   = 35.9f;
    calibPainel.calibrado = true;

    calibCasa.offsetMV  = 541;
    calibCasa.sensMvA   = 43.3f;
    calibCasa.calibrado = true;

    // Sobrescreve com valores salvos pelo usuário
    prefs.begin("rrfs_cal", true);
    if (prefs.isKey("pn_off"))
        calibPainel.offsetMV = prefs.getInt  ("pn_off",  calibPainel.offsetMV);
    if (prefs.isKey("pn_sens"))
        calibPainel.sensMvA  = prefs.getFloat("pn_sens", calibPainel.sensMvA);
    if (prefs.isKey("casa_off"))
        calibCasa.offsetMV   = prefs.getInt  ("casa_off",calibCasa.offsetMV);
    if (prefs.isKey("casa_sens"))
        calibCasa.sensMvA    = prefs.getFloat("casa_sens",calibCasa.sensMvA);
    autoCalibPainelAtivo = prefs.getBool("auto_cal", true);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "Painel: off=%dmV sens=%.1fmV/A autoCal=%s",
             calibPainel.offsetMV, calibPainel.sensMvA,
             autoCalibPainelAtivo ? "ON" : "OFF");
    adicionarLog(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf),
             "Casa: off=%dmV sens=%.1fmV/A",
             calibCasa.offsetMV, calibCasa.sensMvA);
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [WIFI] REDE
// ═══════════════════════════════════════════════════════════
void sincronizarNTP() {
    if (WiFi.status() != WL_CONNECTED) {
        adicionarLog("NTP: sem WiFi"); return;
    }
    configTime(-3*3600, 0, "pool.ntp.org",
               "time.google.com", "time.cloudflare.com");
    adicionarLog("NTP: sincronizando...");
    struct tm ti; unsigned long t = millis(); bool ok = false;
    while (millis()-t < 5000) {
        if (getLocalTime(&ti)) { ok = true; break; }
        delay(200);
    }
    if (ok) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "NTP OK: %02d:%02d %02d/%02d/%04d",
                 ti.tm_hour, ti.tm_min,
                 ti.tm_mday, ti.tm_mon+1, ti.tm_year+1900);
        adicionarLog(tempBuf);
    } else adicionarLog("NTP: falhou");
}

void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP: RRFS Energia | 10.0.0.1");
    logRAM(); ledBlink();
}

bool iniciarWiFi(const String& ssid, const String& pass,
                  const String& ip,   const String& gw,
                  const String& mask, const String& dns) {
    IPAddress lIP, lGW, lMask, lDNS;
    if (!lIP.fromString(ip))     return false;
    if (!lGW.fromString(gw))     return false;
    if (!lMask.fromString(mask)) return false;
    if (!lDNS.fromString(dns))   return false;

    gWifiSSID = ssid; gWifiPASS = pass;
    gWifiIP   = ip;   gWifiGW   = gw;
    gWifiMASK = mask; gWifiDNS  = dns;

    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(300);
    WiFi.mode(WIFI_STA); WiFi.setSleep(false);
    WiFi.config(lIP, lGW, lMask, lDNS);
    WiFi.setAutoReconnect(true); WiFi.persistent(false);

    snprintf(tempBuf, sizeof(tempBuf),
             "Conectando: %s | %s", ssid.c_str(), ip.c_str());
    adicionarLog(tempBuf); ledBlink();
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long t = millis(); int tent = 0;
    while (WiFi.status()!=WL_CONNECTED && millis()-t<20000) {
        atualizarLED(); delay(500); tent++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi OK! %s %ddBm",
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
        adicionarLog(tempBuf);
        logRAM(); modoAP = false; ledFixo(true);
        sincronizarNTP(); return true;
    }
    snprintf(tempBuf, sizeof(tempBuf),
             "WiFi FALHOU: %d", WiFi.status());
    adicionarLog(tempBuf);
    return false;
}

void tentarReconectarWiFi() {
    static bool   emRecon = false;
    static uint8_t tent   = 0;
    if (emRecon) return;
    emRecon = true; tent++;
    snprintf(tempBuf, sizeof(tempBuf),
             "WiFi perdido - tent %d/5", tent);
    adicionarLog(tempBuf);

    if (!gWifiIP.isEmpty()) {
        IPAddress lIP, lGW, lMask, lDNS;
        lIP.fromString(gWifiIP);   lGW.fromString(gWifiGW);
        lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
        WiFi.config(lIP, lGW, lMask, lDNS);
    }
    WiFi.reconnect();
    unsigned long t = millis();
    while (WiFi.status()!=WL_CONNECTED && millis()-t<15000) {
        atualizarLED(); server.handleClient();
        if (modoAP) dnsServer.processNextRequest();
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi reconectado! %ddBm", WiFi.RSSI());
        adicionarLog(tempBuf);
        tent = 0; ledFixo(true);
        sincronizarNTP(); emRecon = false; return;
    }
    if (tent >= 5) {
        adicionarLog("5 falhas - AP temporario");
        tent = 0;
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
        WiFi.softAP(AP_SSID);
        dnsServer.start(DNS_PORT, "*", AP_IP);
        modoAP = true; ledBlink();
        server.close(); delay(200); configurarRotas();
    }
    emRecon = false;
}

// ═══════════════════════════════════════════════════════════
// [SETUP] INICIALIZAÇÃO
// ═══════════════════════════════════════════════════════════
void setup() {
    esp_err_t errNVS = nvs_flash_init();
    if (errNVS == ESP_ERR_NVS_NO_FREE_PAGES ||
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); errNVS = nvs_flash_init();
    }

    esp_reset_reason_t motivo = esp_reset_reason();
    if      (motivo == ESP_RST_BROWNOUT)
        adicionarLog("!!! RESET: BROWNOUT !!!");
    else if (motivo == ESP_RST_PANIC)
        adicionarLog("!!! RESET: PANIC !!!");
    else if (motivo == ESP_RST_TASK_WDT)
        adicionarLog("!!! RESET: WATCHDOG !!!");

    setupLED(); setupBotao(); setupReles(); ledBlink();
    adicionarLog("RRFS Energia v16 - sem IA");
    if (errNVS != ESP_OK) adicionarLog("NVS reparada");
    logRAM();

    carregarCalibSensores();
    carregarConfigBateria();
    carregarConfigRedes();
    carregarContadores();
    carregarHistorico();
    carregarConfigLocal();
    calcularBateria();

    snprintf(tempBuf, sizeof(tempBuf),
             "Bat: %.1fV %.0fAh %.0f%% desc | Calib:%s",
             bateria.tensaoNominal, bateria.capacidadeAh,
             bateria.descargaSegura,
             bateria.calibrada ? "SIM" : "NAO");
    adicionarLog(tempBuf);

    ultimoContagem = millis();

    // Sequência de boot
    ligarRedeBaixa(); adicionarLog("Rede Baixa ON");
    {
        unsigned long t = millis();
        while (millis()-t < 8000) { atualizarLED(); delay(10); }
    }
    ligarInversor(); adicionarLog("Inversor ON");
    {
        unsigned long t = millis();
        while (millis()-t < 5000) { atualizarLED(); delay(10); }
    }
    {
        bool ok = false; unsigned long t = millis();
        while (millis()-t < 3000) {
            if (lerStatusInversor()) { ok = true; break; }
            delay(200);
        }
        adicionarLog(ok ? "Inv STATUS: OK" : "Inv STATUS: sem sinal");
    }
    ligarRedeAlta();  adicionarLog("Rede Alta ON");  delay(500);
    ligarLampExt();   adicionarLog("Lamp.Ext ON");   delay(200);
    ligarLampInt();   adicionarLog("Lamp.Int ON");

    curto = {CURTO_NORMAL, SUB_IDLE, 0, false, false, false, 0};
    logRAM();

    // WiFi
    bool temCred = existeCredenciais();
    bool temWifi = existeWiFiConfig();
    if (!temCred || !temWifi) {
        adicionarLog("Setup inicial - AP");
        iniciarAP();
    } else {
        String ssid, pass, ip, gw, mask, dns; bool ap;
        carregarWifi(ssid, pass, ip, gw, mask, dns, ap);
        if (ap) { adicionarLog("Modo AP"); iniciarAP(); }
        else {
            adicionarLog("Conectando WiFi...");
            if (!iniciarWiFi(ssid, pass, ip, gw, mask, dns)) {
                adicionarLog("Fallback -> AP"); iniciarAP();
            }
        }
    }

    configurarRotas();
    adicionarLog("Servidor web ATIVO");
    lerSensores();

    snprintf(tempBuf, sizeof(tempBuf),
             "1a leitura: P=%.1fA C=%.1fA B=%.1fA",
             sensores.correntePainel,
             sensores.correnteCasa,
             sensores.correnteBateria);
    adicionarLog(tempBuf);

    ultimoHistSnap = millis();
    adicionarLog("Sistema v16 pronto!");
    logRAM();
}

// ═══════════════════════════════════════════════════════════
// [LOOP] CICLO PRINCIPAL
// ═══════════════════════════════════════════════════════════
void loop() {
    verificarReset();
    atualizarLED();
    if (modoAP) dnsServer.processNextRequest();
    server.handleClient();

    unsigned long agora = millis();

    // Leitura 1s
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;
        lerSensores();
        acumularEnergia();
    }

    // Monitoramento de curto (todo ciclo)
    {
        bool emProc = (curto.estado==CURTO_RELIGANDO ||
                       curto.estado==CURTO_TESTANDO_ALTA ||
                       curto.estado==CURTO_TESTANDO_BAIXA);
        if (estados.inversor || emProc) {
            monitorarInversor();
        } else if (curto.estado==CURTO_NORMAL && curto.monitorando) {
            curto.monitorando = false;
            curto.tInicio = 0;
            curto.tentativasReligar = 0;
        }
    }

    // Controle energia 5s
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;
        bool emProc = (curto.estado==CURTO_RELIGANDO ||
                       curto.estado==CURTO_TESTANDO_ALTA ||
                       curto.estado==CURTO_TESTANDO_BAIXA);
        if (!emProc && (curto.estado==CURTO_NORMAL ||
                        curto.estado==CURTO_CONFIRMADO_ALTA))
            controlarEnergia();
    }

    // Restauração (todo ciclo)
    processarRestauroEnergia();

    // WiFi - verifica 15s
    {
        static unsigned long tWifi = 0, tRecon = 0;
        if (!gWifiSSID.isEmpty() && agora-tWifi >= 15000) {
            tWifi = agora;
            if (WiFi.status() != WL_CONNECTED) {
                if (agora-tRecon >= 30000) {
                    tRecon = agora; tentarReconectarWiFi();
                }
            } else {
                // NTP diário
                static unsigned long tNTP = 0;
                if (agora-tNTP >= 86400000UL) {
                    tNTP = agora; sincronizarNTP();
                }
                // Log RSSI 5min
                static unsigned long tRSSI = 0;
                if (agora-tRSSI >= 300000) {
                    tRSSI = agora;
                    snprintf(tempBuf, sizeof(tempBuf),
                             "WiFi: %s %ddBm",
                             WiFi.SSID().c_str(), WiFi.RSSI());
                    adicionarLog(tempBuf);
                }
            }
        }
    }

    // Salva Ah bateria 5min
    {
        static unsigned long tSalva = 0;
        if (bateria.calibrada && agora-tSalva >= 300000) {
            tSalva = agora;
            prefs.begin("rrfs", false);
            prefs.putFloat("bat_ah_at", bateria.ahAtual);
            prefs.end();
        }
    }

    // Previsão 1h
    if (configLocal.configurado &&
        WiFi.status()==WL_CONNECTED &&
        !buscandoPrev &&
        (ultimaPrev==0 || agora-ultimaPrev>=PREV_INTERVALO))
        buscarPrevisaoMeteo();

    // Decisões + calibração 5min
    {
        static unsigned long tCalib = 0;
        if (agora-tCalib >= 300000) {
            tCalib = agora;
            calibrarPrevisao();
            verificarReligamentoAuto();
            tomarDecisoes();
        }
    }

    // Antecipação rede alta 10s
    {
        static unsigned long tAnt = 0;
        if (agora-tAnt >= 10000) {
            tAnt = agora;
            avaliarAntecipacaoRedeAlta();
        }
    }

    // RAM 10min
    {
        static unsigned long tRAM = 0;
        if (agora-tRAM >= 600000) { tRAM = agora; logRAM(); }
    }

    delay(1);
}

// ═══════════════════════════════════════════════════════════
// [WEB] CSS E HELPERS HTML
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
@keyframes fluir{to{stroke-dashoffset:-24}}
@keyframes pulso{0%,100%{opacity:1}50%{opacity:.55}}
.fl{stroke-dasharray:6 6;animation:fluir 1s linear infinite}
.fp{opacity:0.12;stroke-dasharray:2 4}
.pulse{animation:pulso 1.6s ease-in-out infinite}
)RAW";

String gerarPagina(const String& titulo, const String& conteudo,
                   bool comNav) {
    String html;
    html.reserve(8000);
    html = F("<!DOCTYPE html><html><head>"
             "<meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>"); html += CSS; html += F("</style>");
    html += F("</head><body><div class='card'>");
    html += F("<h2>&#9889; RRFS Energia v16</h2>");
    html += conteudo;

    if (comNav && logado) {
        String pag = server.uri();
        html += F("<div class='nav'>");
        html += "<a href='/'         " + String(pag=="/"?         "class='active'":"") + ">&#128202; Painel</a>";
        html += "<a href='/bateria'  " + String(pag=="/bateria"?  "class='active'":"") + ">&#128267; Bateria</a>";
        html += "<a href='/redes'    " + String(pag=="/redes"?    "class='active'":"") + ">&#128268; Redes</a>";
        html += "<a href='/sensores' " + String(pag=="/sensores"? "class='active'":"") + ">&#128270; Sens.</a>";
        html += "<a href='/config'   " + String(pag=="/config"?   "class='active'":"") + ">&#9881; Config</a>";
        html += "<a href='/historico'" + String(pag=="/historico"?"class='active'":"") + ">&#128200; Hist.</a>";
        html += "<a href='/previsao' " + String(pag=="/previsao"? "class='active'":"") + ">&#127774; Prev.</a>";
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

// ── Helper: card de carga ──────────────────────────────────
static String cardCarga(const char* ico, const char* nome,
                         const String& cor, const String& bg,
                         bool ligado, const String& status,
                         const String& info,
                         const String& estim = "") {
    String h = "<div class='cg' style='border-color:" + cor +
               ";background:" + bg + "'>";
    h += "<div class='cr'>";
    h += "<span class='cn'>" + String(ico) + " " + String(nome) + "</span>";
    h += "<span class='cs' style='color:" + cor + "'>";
    h += (ligado ? "&#9679; " : "&#9675; ") + status;
    h += "</span></div>";
    if (info.length() || estim.length()) {
        h += "<div class='ci'>";
        if (info.length())  h += "<div>" + info + "</div>";
        if (estim.length()) h += "<div>" + estim + "</div>";
        h += "</div>";
    }
    h += "</div>";
    return h;
}

// ── Helper: botão liga/desliga ─────────────────────────────
static String btnRele(const char* nome, const char* rel,
                       bool estado, bool bloqueado) {
    String h = "<div class='cg' style='border-color:";
    h += bloqueado ? "#ff6f00" : (estado ? "#43a047" : "#455a64");
    h += ";background:" + String(estado ? "#0a1f0a" : "#0d0d0d") + "'>";
    h += "<div class='cr'><span class='cn'>" + String(nome) + "</span>";
    h += "<span class='cs' style='color:" +
         String(bloqueado?"#ff9800":estado?"#43a047":"#78909c") + "'>";
    h += bloqueado ? "&#9888; BLOQ" : (estado ? "&#9679; ON" : "&#9675; OFF");
    h += "</span></div>";
    if (!bloqueado) {
        h += "<div style='display:flex;gap:6px;margin-top:8px'>";
        h += "<form method='POST' action='/redes' style='flex:1'>"
             "<input type='hidden' name='acao' value='rele'>"
             "<input type='hidden' name='rele' value='" + String(rel) + "'>"
             "<input type='hidden' name='valor' value='1'>"
             "<button type='submit' class='btn-g btn-sm'"
             + String(!estado?"":" disabled") + ">&#9679; Ligar</button></form>";
        h += "<form method='POST' action='/redes' style='flex:1'>"
             "<input type='hidden' name='acao' value='rele'>"
             "<input type='hidden' name='rele' value='" + String(rel) + "'>"
             "<input type='hidden' name='valor' value='0'>"
             "<button type='submit' class='btn-r btn-sm'"
             + String(estado?"":" disabled") + ">&#9675; Desligar</button></form>";
        h += "</div>";
    }
    h += "</div>";
    return h;
}

// ── Estimativa de tempo até corte (HTML) ──────────────────
static String htmlEstim(const String& t) {
    if (t == "--" || t == "") return "";
    String cor = (t == "Agora") ? "#ef5350" : "#ffb74d";
    return "<span style='color:" + cor + "'>&#8987; Desliga em: <b>"
           + t + "</b></span>";
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
        String u=server.arg("user"), p=server.arg("pass"),
               c=server.arg("conf");
        if (u.length()<3)
            msg = "<div class='er'>&#10007; Min 3 chars no usuario</div>";
        else if (p.length()<4)
            msg = "<div class='er'>&#10007; Min 4 chars na senha</div>";
        else if (p!=c)
            msg = "<div class='er'>&#10007; Senhas nao conferem</div>";
        else {
            salvarCredenciais(u,p);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location","/setup-wifi"); server.send(302); return;
        }
    }
    String c = F("<div class='nf'>&#8505; Primeiro acesso. Crie suas credenciais.</div>");
    c += "<h3>&#128274; Criar Conta</h3>" + msg;
    c += F("<form method='POST' action='/setup-login'>"
           "<label>Usuario (min 3 chars):</label>"
           "<input type='text' name='user' maxlength='32' required "
           "autocomplete='off' placeholder='Usuario'>"
           "<label>Senha (min 4 chars):</label>"
           "<input type='password' name='pass' maxlength='32' required "
           "placeholder='Senha'>"
           "<label>Confirmar Senha:</label>"
           "<input type='password' name='conf' maxlength='32' required "
           "placeholder='Confirmar'>"
           "<button type='submit'>Criar &#8594;</button></form>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Primeiro Acesso", c, false));
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
                          "<div class='nf'>SSID: <b>RRFS Energia</b> | "
                          "IP: <b>10.0.0.1</b></div>");
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("Salvo",ct,false));
            delay(4000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"), wp=server.arg("wpass"),
               ip=server.arg("ip"),   gw=server.arg("gw"),
               mask=server.arg("mask"),dns=server.arg("dns");
        if (ssid.isEmpty())  msg="<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7) msg="<div class='er'>Informe o IP</div>";
        else if (gw.length()<7) msg="<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wp,ip,gw,mask,dns,false);
            String ct = "<div class='ok'>&#10003; Salvo! Acesse: http://"+ip+"</div>";
            ct += F("<div class='nf'>Reiniciando...</div>");
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("Salvo",ct,false));
            delay(4000); ESP.restart(); return;
        }
    }
    adicionarLog("Escaneando WiFi...");
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false,false,false,600);

    String c = F("<div class='nf'>&#8505; Configure a rede. IP fixo obrigatorio.</div>");
    c += "<h3>&#128246; Configurar Rede</h3>" + msg;
    c += F("<form method='POST' action='/setup-wifi'>"
           "<label>Modo:</label>"
           "<select name='modo' id='modo' onchange='toggleModo()'>"
           "<option value='wifi'>Conectar ao WiFi</option>"
           "<option value='ap'>Manter Ponto de Acesso</option>"
           "</select><div id='dw'>"
           "<label>Rede WiFi:</label><select name='ssid'>");
    if (n == 0) c += F("<option value=''>Nenhuma rede</option>");
    else {
        for (int i = 0; i < n; i++) {
            String q = WiFi.RSSI(i)>-60?"Bom":
                       WiFi.RSSI(i)>-75?"Regular":"Fraco";
            c += "<option value='" + WiFi.SSID(i) + "'>"
               + WiFi.SSID(i);
            snprintf(tempBuf,sizeof(tempBuf)," [%s %ddBm]",
                     q.c_str(), WiFi.RSSI(i));
            c += String(tempBuf) + "</option>";
        }
    }
    WiFi.scanDelete();
    c += F("</select>"
           "<button type='button' class='btn-o btn-sm' "
           "onclick='location.reload()' style='margin-top:8px'>"
           "&#128259; Atualizar</button>"
           "<label>Senha WiFi:</label>"
           "<input type='password' name='wpass' placeholder='Vazio se aberta'>"
           "<label>IP Fixo do ESP32:</label>"
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
        server.sendHeader("Location","/setup-login"); server.send(302); return;
    }
    if (logado) {
        server.sendHeader("Location","/"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String u=server.arg("user"), p=server.arg("pass");
        if (verificarLogin(u,p)) {
            logado = true;
            snprintf(tempBuf,sizeof(tempBuf),"Login: %s",u.c_str());
            adicionarLog(tempBuf); logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        }
        snprintf(tempBuf,sizeof(tempBuf),"Login FALHOU: %s",u.c_str());
        adicionarLog(tempBuf);
        msg = F("<div class='er'>&#10007; Usuario ou senha incorretos</div>");
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
    if (logado) { adicionarLog("Logout"); logado = false; }
    server.sendHeader("Location","/login"); server.send(302);
}

void handleLogs() {
    String json = "[";
    for (int i = 0; i < totalLogs; i++) {
        int idx = ((int)logIndex-1-i+MAX_LOGS) % MAX_LOGS;
        if (i > 0) json += ",";
        json += "\"";
        for (int ch = 0; logs[idx][ch]; ch++) {
            char cc = logs[idx][ch];
            if (cc=='"')       json += "\\\"";
            else if (cc=='\\') json += "\\\\";
            else               json += cc;
        }
        json += "\"";
    }
    json += "]";
    server.send(200,"application/json", json);
}

void handleNotFound() {
    String uri = server.uri();
    if (uri.indexOf("generate_204")>=0 || uri.indexOf("gen_204")>=0 ||
        uri.indexOf("hotspot-detect")>=0 || uri.indexOf("ncsi.txt")>=0 ||
        uri.indexOf("connecttest")>=0) {
        if (modoAP) {
            server.sendHeader("Location","http://10.0.0.1/");
            server.send(302,"text/plain","");
        } else server.send(204,"text/plain","");
        return;
    }
    if (uri == "/favicon.ico") { server.send(204); return; }
    if (modoAP) {
        server.sendHeader("Location","http://10.0.0.1/");
        server.send(302);
    } else server.send(404,"text/plain","404");
}

// ═══════════════════════════════════════════════════════════
// [DASH] DASHBOARD PRINCIPAL
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
    bool casaCons  = (sensores.correnteCasa    >  0.3f);
    bool sistCons  = (sensores.correnteSistema >  0.15f) && !painelGer;
    bool invAtivo  = sensores.inversoreAtivo;
    bool curtoAlt  = (curto.estado==CURTO_CONFIRMADO_ALTA || curto.redeAltaCurto);
    bool curtoBx   = (curto.estado==CURTO_CONFIRMADO_BAIXA|| curto.redeBaixaCurto);
    bool diagAtivo = (curto.estado==CURTO_TESTANDO_ALTA   ||
                      curto.estado==CURTO_TESTANDO_BAIXA  ||
                      curto.estado==CURTO_RELIGANDO);

    // Buffers rápidos
    char sIP[10],sIC[10],sIB[10],sIS[10];
    char sWP[10],sWC[10],sWB[10];
    float vn = bateria.tensaoNominal;
    snprintf(sIP,sizeof(sIP),"%.1fA",sensores.correntePainel);
    snprintf(sIC,sizeof(sIC),"%.1fA",sensores.correnteCasa);
    snprintf(sIB,sizeof(sIB),"%.1fA",fabsf(sensores.correnteBateria));
    snprintf(sIS,sizeof(sIS),"%.2fA",sensores.correnteSistema);
    snprintf(sWP,sizeof(sWP),"%.0fW",sensores.correntePainel*vn);
    snprintf(sWC,sizeof(sWC),"%.0fW",sensores.correnteCasa*vn);
    snprintf(sWB,sizeof(sWB),"%.0fW",fabsf(sensores.correnteBateria)*vn);

    char pctBuf[8];
    snprintf(pctBuf,sizeof(pctBuf),"%.0f%%",
             bateria.calibrada ? bateria.percentualAtual : 0.0f);

    // Estimativas
    EstimativaTempo tempo = calcularTempoRestante();
    String tLI="--",tLE="--",tRA="--",tRB="--";
    bool estimOk = (batDesc && bateria.calibrada);
    if (estimOk) {
        float tx = sensores.correnteBateria;
        tLI = tempoAteCorte(redes.corteLampInt, tx);
        tLE = tempoAteCorte(redes.corteLampExt, tx);
        tRA = tempoAteCorte(redes.corteAlta,    tx);
        tRB = tempoAteCorte(redes.corteBaixa,   tx);
    }

    // Cores
    String cPainel = painelGer    ? "#fdd835" : "#455a64";
    String cBat    = !bateria.calibrada         ? "#455a64" :
                      bateria.percentualAtual>50 ? "#43a047" :
                      bateria.percentualAtual>25 ? "#ef6c00" : "#c62828";
    String cInv    = invAtivo     ? "#4fc3f7" : "#455a64";
    String cAlt    = curtoAlt     ? "#ff6f00" :
                      estados.redesAlta ? "#ef5350" : "#455a64";
    String cBaixa  = curtoBx      ? "#ff6f00" :
                      estados.redeBaixa ? "#42a5f5" : "#455a64";
    String cLE     = estados.lampExt ? "#ffd54f" : "#455a64";
    String cLI     = estados.lampInt ? "#ffd54f" : "#455a64";

    String c = "";

    // Aviso bateria
    if (!bateria.calibrada)
        c += F("<div class='wn' style='font-size:11px;margin-bottom:6px'>"
               "&#9888; Bateria nao calibrada. "
               "<a href='/bateria' style='color:#ffe0b2'>Calibrar</a></div>");

    // ── SVG Fluxo de Energia ───────────────────────────────
    c += F("<h3 style='text-align:center;margin:4px 0 10px'>&#9889; Fluxo de Energia</h3>");
    {
        bool fPB = painelGer && batCarg;
        bool fPC = painelGer && casaCons;
        bool fBC = batDesc   && casaCons;
        String lPB = fPB ? "#43a047":"#333";
        String lPC = fPC ? "#fdd835":"#333";
        String lBC = fBC ? "#ef6c00":"#333";
        String lBS = sistCons ? "#ff9800":"#333";
        String cPBg = painelGer ? "#3a2f00":"#181818";
        String cPBd = painelGer ? "#fdd835":"#3a3a3a";
        String cBBg = !bateria.calibrada ? "#181818" :
                       bateria.percentualAtual>50?"#0a2210":
                       bateria.percentualAtual>25?"#2a1600":"#2a0808";
        String cCBg = casaCons ? "#062231":"#181818";
        String cCBd = casaCons ? "#4fc3f7":"#3a3a3a";
        int vbH = sistCons ? 275 : 230;
        c += "<svg viewBox='0 0 340 " + String(vbH) +
             "' xmlns='http://www.w3.org/2000/svg' "
             "style='width:100%;height:auto;display:block'>";
        // Linhas
        c += "<path d='M110,55 C160,20 200,20 240,55' fill='none' stroke='" +
             lPB + "' stroke-width='3' stroke-linecap='round' class='" +
             String(fPB?"fl":"fp") + "'/>";
        c += "<path d='M90,90 C80,130 90,150 130,175' fill='none' stroke='" +
             lPC + "' stroke-width='3' stroke-linecap='round' class='" +
             String(fPC?"fl":"fp") + "'/>";
        c += "<path d='M255,90 C260,130 250,150 210,175' fill='none' stroke='" +
             lBC + "' stroke-width='3' stroke-linecap='round' class='" +
             String(fBC?"fl":"fp") + "'/>";
        if (sistCons)
            c += "<path d='M270,95 C270,220 270,220 270,240' fill='none' stroke='" +
                 lBS + "' stroke-width='3' stroke-linecap='round' class='fl'/>";
        // Labels linhas
        if (fPB) {
            c += "<rect x='143' y='24' width='54' height='16' rx='8' fill='#0d0d0d'/>";
            c += "<text x='170' y='36' text-anchor='middle' font-size='11' fill='" +
                 lPB + "' font-weight='bold'>" + String(sWP) + "</text>";
        }
        if (fPC) {
            c += "<rect x='60' y='125' width='46' height='15' rx='7' fill='#0d0d0d'/>";
            c += "<text x='83' y='136' text-anchor='middle' font-size='10' fill='" +
                 lPC + "' font-weight='bold'>" + String(sWC) + "</text>";
        }
        if (fBC) {
            c += "<rect x='232' y='125' width='46' height='15' rx='7' fill='#0d0d0d'/>";
            c += "<text x='255' y='136' text-anchor='middle' font-size='10' fill='" +
                 lBC + "' font-weight='bold'>" + String(sWB) + "</text>";
        }
        // Painel
        c += "<g" + String(painelGer?" class='pulse'":"") + ">";
        c += "<rect x='20' y='15' width='100' height='80' rx='14' fill='" +
             cPBg + "' stroke='" + cPBd + "' stroke-width='2'/>";
        c += "<text x='70' y='42' text-anchor='middle' font-size='24'>&#9728;</text>";
        c += "<text x='70' y='60' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>";
        c += "<text x='70' y='76' text-anchor='middle' font-size='16' fill='" +
             cPBd + "' font-weight='bold'>";
        c += painelGer ? String(sIP) : "0.0A";
        c += "</text>";
        c += "<text x='70' y='90' text-anchor='middle' font-size='9' fill='#546e7a'>";
        c += painelGer ? String(sWP) : "sem geracao";
        c += "</text></g>";
        // Bateria
        c += "<g>";
        c += "<rect x='220' y='15' width='100' height='80' rx='14' fill='" +
             cBBg + "' stroke='" + cBat + "' stroke-width='2'/>";
        String icoBat = batCarg ? "&#9650;&#128267;" : batDesc ? "&#9660;&#128267;" : "&#128267;";
        c += "<text x='270' y='42' text-anchor='middle' font-size='22'>" + icoBat + "</text>";
        c += "<text x='270' y='60' text-anchor='middle' font-size='9' fill='#78909c'>BATERIA</text>";
        c += "<text x='270' y='76' text-anchor='middle' font-size='16' fill='" +
             cBat + "' font-weight='bold'>";
        c += bateria.calibrada ? String(pctBuf) : "N/Cal";
        c += "</text>";
        char sBS[14];
        snprintf(sBS,sizeof(sBS),"%c%.1fA",
                 batCarg?'+':batDesc?'-':'~',
                 fabsf(sensores.correnteBateria));
        c += "<text x='270' y='90' text-anchor='middle' font-size='9' fill='#ce93d8'>";
        c += String(sBS) + "</text></g>";
        // Casa
        c += "<g" + String(casaCons?" class='pulse'":"") + ">";
        c += "<rect x='105' y='175' width='130' height='55' rx='14' fill='" +
             cCBg + "' stroke='" + cCBd + "' stroke-width='2'/>";
        c += "<text x='128' y='210' font-size='22'>&#127968;</text>";
        c += "<text x='225' y='196' text-anchor='end' font-size='9' fill='#78909c'>CASA</text>";
        c += "<text x='225' y='212' text-anchor='end' font-size='14' fill='" +
             (casaCons?cCBd:String("#455a64")) + "' font-weight='bold'>";
        c += casaCons ? String(sIC) : "0.0A";
        c += "</text>";
        c += "<text x='225' y='224' text-anchor='end' font-size='9' fill='#546e7a'>";
        c += casaCons ? String(sWC) : "";
        c += "</text></g>";
        // Sistema
        if (sistCons) {
            c += "<text x='239' y='207' text-anchor='middle' font-size='20' "
                 "font-weight='bold' fill='#ff9800'>+</text>";
            c += "<g class='pulse'>";
            c += "<rect x='245' y='240' width='90' height='68' rx='12' "
                 "fill='#241505' stroke='#ff9800' stroke-width='2'/>";
            c += "<text x='290' y='262' text-anchor='middle' font-size='18'>&#128268;</text>";
            c += "<text x='290' y='276' text-anchor='middle' font-size='8' "
                 "fill='#78909c'>ESP32+CTRL</text>";
            c += "<text x='290' y='291' text-anchor='middle' font-size='12' "
                 "fill='#ff9800' font-weight='bold'>" + String(sIS) + "</text></g>";
        }
        c += "</svg>";
    }

    if (sistCons) {
        c += "<div class='nf' style='font-size:10px;text-align:center'>"
             "&#128161; Sem geracao: consumo ESP32/ctrl (<b>" +
             String(sIS) + "</b>) somado a casa.</div>";
    }

    // ── Cards de cargas ────────────────────────────────────
    String obsInv = diagAtivo ?
        "<span style='color:#90caf9'>&#128270; Diagnosticando...</span>" :
        (!sensores.inversoreAtivo && estados.inversor) ?
        "<span style='color:#78909c'>Aguardando sinal...</span>" : "";

    String obsAlt = curto.redeAltaCurto ?
        "&#9888; Curto - <a href='/redes' style='color:#ffcc80'>resolver</a>" : "";

    // Horário solar lamp externa
    String obsLE = "";
    if (configLocal.configurado) {
        struct tm tiLE;
        if (getLocalTime(&tiLE)) {
            float latR = (float)(configLocal.latitude*M_PI/180.0);
            float dec  = -23.45f*cosf(2.0f*M_PI*(float)(tiLE.tm_mon+10)/12.0f);
            float dR   = dec*M_PI/180.0f;
            float cH   = -tanf(latR)*tanf(dR);
            float hLuz = 12.0f;
            if (cH>=-1&&cH<=1)
                hLuz=(2.0f/15.0f)*(acosf(cH)*180.0f/M_PI);
            float nS=12.0f-hLuz/2.0f, pS=12.0f+hLuz/2.0f;
            snprintf(tempBuf,sizeof(tempBuf),
                     "&#9728; Nasc: %.0fh%02.0fm | Por: %.0fh%02.0fm",
                     floorf(nS),(nS-floorf(nS))*60,
                     floorf(pS),(pS-floorf(pS))*60);
            obsLE = String(tempBuf);
        }
    } else {
        obsLE = "&#9888; Configure local em Previsao";
    }

    c += cardCarga("&#9889;","Inversor",
                   cInv, invAtivo?"#081824":"#0d0d0d",
                   invAtivo,
                   invAtivo?"ATIVO":(estados.inversor?"AGUARD":"OFF"),
                   obsInv);
    c += cardCarga("&#128268;","Rede Alta",
                   curtoAlt?"#ff6f00":cAlt,
                   curtoAlt?"#1a0f00":estados.redesAlta?"#180808":"#0d0d0d",
                   estados.redesAlta && !curtoAlt,
                   curtoAlt?"CURTO!":estados.redesAlta?"ON":"OFF",
                   obsAlt, estados.redesAlta?htmlEstim(tRA):"");
    c += cardCarga("&#128268;","Rede Baixa",
                   curtoBx?"#ff6f00":cBaixa,
                   curtoBx?"#1a0f00":estados.redeBaixa?"#080d18":"#0d0d0d",
                   estados.redeBaixa && !curtoBx,
                   curtoBx?"CURTO!":estados.redeBaixa?"ON":"OFF",
                   curto.redeBaixaCurto?"&#9888; Curto":"",
                   estados.redeBaixa?htmlEstim(tRB):"");
    c += cardCarga("&#128161;","Lamp. Externa",
                   cLE, estados.lampExt?"#181500":"#0d0d0d",
                   estados.lampExt, estados.lampExt?"ON":"OFF",
                   obsLE, estados.lampExt?htmlEstim(tLE):"");
    c += cardCarga("&#128161;","Lamp. Interna",
                   cLI, estados.lampInt?"#181500":"#0d0d0d",
                   estados.lampInt, estados.lampInt?"ON":"OFF",
                   "", estados.lampInt?htmlEstim(tLI):"");

    // Alerta curto
    if (curtoAlt || curtoBx || curto.estado==CURTO_AGUARDANDO_RESET) {
        c += F("<div style='background:#7f0000;border:2px solid #ef5350;"
               "border-radius:8px;padding:10px;text-align:center;margin-top:4px'>");
        c += F("<div style='font-size:13px;font-weight:bold;color:#ef9a9a'>&#9888; ");
        if (curtoAlt) c += "CURTO REDE ALTA";
        if (curtoBx)  c += "CURTO REDE BAIXA";
        if (curto.estado==CURTO_AGUARDANDO_RESET) c += "FALHA INVERSOR";
        c += F("</div><a href='/redes' style='color:#ffcc80;font-size:12px'>"
               "Resolver &rarr;</a></div>");
    }

    // ── Barra de Bateria ───────────────────────────────────
    c += F("<div style='margin:10px 0'>");
    c += F("<div style='display:flex;justify-content:space-between;"
           "align-items:center;margin-bottom:4px'>");
    c += F("<span style='font-size:11px;color:#78909c'>&#128267; Bateria</span>");

    if (tempo.valido) {
        char bT[50];
        if (tempo.carregando) {
            if (tempo.horas==0&&tempo.minutos==0)
                snprintf(bT,sizeof(bT),"Carregada!");
            else
                snprintf(bT,sizeof(bT),"Completa em %dh%02dm",
                         tempo.horas,tempo.minutos);
        } else {
            if (tempo.horas==0&&tempo.minutos<5)
                snprintf(bT,sizeof(bT),"ENERGIA CRITICA!");
            else
                snprintf(bT,sizeof(bT),"Restam %dh%02dm",
                         tempo.horas,tempo.minutos);
        }
        String corT = tempo.carregando ? "#43a047" :
                      (tempo.horas==0&&tempo.minutos<30?"#ef5350":"#ef6c00");
        c += "<span style='font-size:11px;font-weight:bold;color:" + corT + "'>";
        c += tempo.carregando ? "&#9650; " : "&#9660; ";
        c += String(bT) + "</span>";
    }
    c += F("</div>");

    if (bateria.calibrada) {
        float p = bateria.percentualAtual;
        String corB = p>=50?"#2e7d32":p>=25?"#ef6c00":"#c62828";
        c += "<div class='bb'><div class='bv' id='bat-barra' style='width:" +
             String((int)p) + "%;background:" + corB + "'>" +
             String((int)p) + "%</div></div>";
        c += F("<div style='display:flex;justify-content:space-between;"
               "margin-top:3px;font-size:10px;color:#546e7a'>");
        char bAh[40];
        snprintf(bAh,sizeof(bAh),"%.1f / %.1f Ah",
                 bateria.ahAtual, bateria.ahDisponivel);
        c += "<span id='bat-ah'>" + String(bAh) + "</span>";
        char bKW[40];
        snprintf(bKW,sizeof(bKW),"%.0fW @ %.1fV %s",
                 fabsf(sensores.correnteBateria)*vn,
                 bateria.tensaoNominal,
                 batCarg?"entrada":batDesc?"saida":"equil.");
        c += "<span id='bat-kw'>" + String(bKW) + "</span></div>";
    } else {
        c += F("<div class='wn' style='font-size:11px;text-align:center'>"
               "Nao calibrada - <a href='/bateria' style='color:#ffe0b2'>"
               "calibrar</a></div>");
    }
    c += F("</div>");

    // ── Cards corrente ─────────────────────────────────────
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
           "gap:6px;margin:4px 0'>");
    // Painel
    c += "<div class='sc" + String(painelGer?" ok":"") + "' "
         "style='padding:8px;text-align:center'>";
    c += F("<div class='lbl'>&#9728; Painel</div>");
    c += "<div style='font-size:18px;font-weight:bold;color:" +
         cPainel + "' id='cp'>";
    c += painelGer ? String(sIP) : "0.0A";
    c += "</div><div class='sub' id='wp'>";
    c += painelGer ? String(sWP) : "sem ger.";
    c += F("</div></div>");
    // Casa
    c += F("<div class='sc' style='padding:8px;text-align:center'>");
    c += F("<div class='lbl'>&#127968; Casa</div>");
    c += "<div style='font-size:18px;font-weight:bold;color:#4fc3f7' id='cc'>";
    c += String(sIC);
    c += "</div><div class='sub' id='wc'>" + String(sWC) + F("</div></div>");
    // Bateria
    {
        String cf = batCarg?"#43a047":batDesc?"#ef6c00":"#fdd835";
        String si = batCarg?"+":batDesc?"-":"~";
        c += "<div class='sc' style='padding:8px;text-align:center'>";
        c += F("<div class='lbl'>&#128267; Bat.</div>");
        c += "<div style='font-size:18px;font-weight:bold;color:" + cf + "' id='cb'>";
        c += si + String(sIB);
        c += "</div><div class='sub' id='wb'>" + String(sWB) + F("</div></div>");
    }
    c += F("</div>");

    // Info sistema
    String ip = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    c += "<div class='nf' style='font-size:10px'>";
    c += "&#128246; " + String(modoAP?"AP":"WiFi") + " | " + ip;
    if (!modoAP && WiFi.status()==WL_CONNECTED)
        c += " | " + String(WiFi.RSSI()) + "dBm";
    snprintf(tempBuf,sizeof(tempBuf)," | RAM:%.1fKB | Up:%lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000UL);
    c += String(tempBuf) + F("</div>");

    // AJAX 1s
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
    "var wp=Math.abs(ip*vn),wc=Math.abs(ic*vn),wb=Math.abs(ib*vn);"
    "var eCP=document.getElementById('cp');"
    "if(eCP)eCP.textContent=ip>0.5?ip.toFixed(1)+'A':'0.0A';"
    "var eCC=document.getElementById('cc');"
    "if(eCC)eCC.textContent=ic.toFixed(1)+'A';"
    "var eCB=document.getElementById('cb');"
    "if(eCB){var s=ib>0.3?'+':ib<-0.3?'-':'~';"
    "eCB.textContent=s+Math.abs(ib).toFixed(1)+'A';}"
    "var eWP=document.getElementById('wp');"
    "if(eWP)eWP.textContent=ip>0.5?wp.toFixed(0)+'W':'sem ger.';"
    "var eWC=document.getElementById('wc');"
    "if(eWC)eWC.textContent=wc.toFixed(0)+'W';"
    "var eWB=document.getElementById('wb');"
    "if(eWB)eWB.textContent=wb.toFixed(0)+'W';"
    "var pct=parseFloat(d.pct||0);"
    "var eBar=document.getElementById('bat-barra');"
    "if(eBar&&d.calib){"
    "eBar.style.width=pct+'%';"
    "eBar.textContent=Math.round(pct)+'%';"
    "var cor=pct>=50?'#2e7d32':pct>=25?'#ef6c00':'#c62828';"
    "eBar.style.background=cor;}"
    "var eAh=document.getElementById('bat-ah');"
    "if(eAh&&d.calib)eAh.textContent="
    "parseFloat(d.ah_at||0).toFixed(1)+' / '"
    "+parseFloat(d.ah_disp||0).toFixed(1)+' Ah';"
    "var eKW=document.getElementById('bat-kw');"
    "if(eKW){var st=ib>0.3?'entrada':ib<-0.3?'saida':'equil.';"
    "eKW.textContent=wb.toFixed(0)+'W @ '+vn.toFixed(1)+'V '+st;}"
    "}catch(e){}}}"
    ";x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,1000);aD();"
    "setInterval(function(){location.reload();},30000);"
    "</script>");

    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS - Painel", c));
}

// ── JSON AJAX ──────────────────────────────────────────────
void handleDados() {
    if (!logado) { server.send(403); return; }
    char buf[300];
    snprintf(buf,sizeof(buf),
             "{\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,\"is\":%.2f,"
             "\"pct\":%.1f,\"ah_at\":%.2f,\"ah_disp\":%.2f,"
             "\"vnom\":%.1f,\"calib\":%s,"
             "\"inv\":%s,\"rb\":%s,\"ra\":%s,\"le\":%s,\"li\":%s}",
             sensores.correntePainel, sensores.correnteCasa,
             sensores.correnteBateria, sensores.correnteSistema,
             bateria.percentualAtual, bateria.ahAtual, bateria.ahDisponivel,
             bateria.tensaoNominal,
             bateria.calibrada?"true":"false",
             sensores.inversoreAtivo?"true":"false",
             estados.redeBaixa?"true":"false",
             estados.redesAlta?"true":"false",
             estados.lampExt?"true":"false",
             estados.lampInt?"true":"false");
    server.send(200,"application/json", buf);
}

// ═══════════════════════════════════════════════════════════
// [BAT-WEB] PÁGINA DE BATERIA
// ═══════════════════════════════════════════════════════════
void handleBateria() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config") {
            float vn=server.arg("vnom").toFloat();
            float ah=server.arg("ah").toFloat();
            float dc=server.arg("desc").toFloat();
            if (vn<6||vn>60)
                msg="<div class='er'>&#10007; Tensao invalida (6-60V)</div>";
            else if (ah<1||ah>5000)
                msg="<div class='er'>&#10007; Capacidade invalida</div>";
            else if (dc<10||dc>100)
                msg="<div class='er'>&#10007; Descarga invalida (10-100%)</div>";
            else {
                bateria.tensaoNominal=vn; bateria.capacidadeAh=ah;
                bateria.descargaSegura=dc; calcularBateria();
                salvarConfigBateria();
                snprintf(tempBuf,sizeof(tempBuf),
                         "Bat: %.1fV %.0fAh %.0f%%",vn,ah,dc);
                adicionarLog(tempBuf);
                msg="<div class='ok'>&#10003; Config salva!</div>";
            }
        } else if (acao == "calibrar") {
            float pBMS = server.arg("pct_atual").toFloat();
            if (pBMS<0||pBMS>100)
                msg="<div class='er'>&#10007; Percentual invalido</div>";
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
                snprintf(tempBuf,sizeof(tempBuf),
                         "Bat cal: BMS=%.0f%% util=%.1fAh (%.0f%%)",
                         pBMS, ahUt, bateria.percentualAtual);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Calibrada! BMS=" +
                      String((int)pBMS) + "% → Sistema=" +
                      String((int)bateria.percentualAtual) + "%</div>";
            }
        }
    }
    calcularBateria();
    String cls = bateria.percentualAtual<25?"er":
                 bateria.percentualAtual<50?"wn":"ok";
    String c = msg;
    c += F("<h3>&#128267; Status</h3>");
    if (bateria.calibrada) {
        c += "<div class='sc " + cls + "'>";
        c += F("<div class='lbl'>Carga Atual</div>");
        char bAh[60];
        snprintf(bAh,sizeof(bAh),"%.1fAh de %.1fAh (%.0f%%)",
                 bateria.ahAtual,bateria.ahDisponivel,bateria.percentualAtual);
        c += "<div class='val'>" + String(bAh) + "</div>";
        char bKWh[60];
        snprintf(bKWh,sizeof(bKWh),"%.2fkWh de %.2fkWh (@ %.1fV nom)",
                 (bateria.ahAtual*bateria.tensaoNominal)/1000.0f,
                 (bateria.ahDisponivel*bateria.tensaoNominal)/1000.0f,
                 bateria.tensaoNominal);
        c += "<div class='sub'>" + String(bKWh) + "</div></div>";
        c += barraProgresso(bateria.percentualAtual);
    } else {
        c += F("<div class='sc wn'><div class='lbl'>Status</div>"
               "<div class='val' style='font-size:16px'>"
               "Aguardando calibracao</div></div>");
    }
    char bInf[120];
    snprintf(bInf,sizeof(bInf),
             "%.1fV nom | %.0fAh | %.0f%% desc | "
             "<b>%.1fAh disp</b> | <b>%.2fkWh</b>",
             bateria.tensaoNominal,bateria.capacidadeAh,
             bateria.descargaSegura,bateria.ahDisponivel,
             (bateria.ahDisponivel*bateria.tensaoNominal)/1000.0f);
    c += "<div class='nf'>" + String(bInf) + "</div>";
    c += F("<h3>&#9881; Configurar Banco (LiFePO4)</h3>"
           "<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='config'>"
           "<label>Tensao Nominal (V) - Ex: 12.8 / 25.6 / 51.2:</label>");
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
    c += "<div class='nf'>Informe o % exato do display/BMS (base: " +
         String(bateria.capacidadeAh,0) + "Ah). "
         "O sistema desconta " + String(100.0f-bateria.descargaSegura,0) +
         "% de reserva automaticamente.</div>";
    c += F("<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='calibrar'>"
           "<label>% no Display/BMS:</label>"
           "<input type='number' name='pct_atual' step='1' min='0' max='100' required "
           "placeholder='Ex: 75'>"
           "<button type='submit' class='btn-o'>&#127919; Calibrar</button></form>");
    c += F("<div class='nf' style='font-size:10px'>"
           "&#9889; <b>Coulomb Counting:</b> integra corrente real a cada segundo. "
           "Mais preciso que estimativa por tensao.</div>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Bateria",c));
}

// ═══════════════════════════════════════════════════════════
// [REDES-WEB] PÁGINA DE REDES
// ═══════════════════════════════════════════════════════════
void handleRedes() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config") {
            float cb=server.arg("corteBaixa").toFloat();
            float ca=server.arg("corteAlta").toFloat();
            float cle=server.arg("corteLampExt").toFloat();
            float cli=server.arg("corteLampInt").toFloat();
            if (cb<0||cb>50)
                msg="<div class='er'>&#10007; Corte Baixa: 0-50%</div>";
            else if (ca<cb||ca>80)
                msg="<div class='er'>&#10007; Corte Alta: >Baixa e <=80%</div>";
            else if (cle<0||cle>90)
                msg="<div class='er'>&#10007; Lamp.Ext: 0-90%</div>";
            else if (cli<0||cli>90)
                msg="<div class='er'>&#10007; Lamp.Int: 0-90%</div>";
            else {
                redes.corteBaixa=cb; redes.corteAlta=ca;
                redes.corteLampExt=cle; redes.corteLampInt=cli;
                salvarConfigRedes();
                snprintf(tempBuf,sizeof(tempBuf),
                         "Redes: Bx=%.0f%% At=%.0f%% LE=%.0f%% LI=%.0f%%",
                         cb,ca,cle,cli);
                adicionarLog(tempBuf);
                msg="<div class='ok'>&#10003; Niveis salvos!</div>";
            }
        } else if (acao == "rele") {
            String qual=server.arg("rele");
            bool ligar=(server.arg("valor")=="1");
            if (qual=="inv") {
                if (curto.estado==CURTO_CONFIRMADO_BAIXA||
                    curto.estado==CURTO_AGUARDANDO_RESET)
                    msg="<div class='er'>&#9888; Bloqueado por curto</div>";
                else {
                    if (ligar){ligarInversor();adicionarLog("Manual: Inv ON");}
                    else{desligarInversor();adicionarLog("Manual: Inv OFF");}
                    msg="<div class='ok'>&#10003; Inversor "+
                        String(ligar?"ligado":"desligado")+"</div>";
                }
            } else if (qual=="ra") {
                if (curto.estado==CURTO_CONFIRMADO_ALTA)
                    msg="<div class='er'>&#9888; Curto rede alta</div>";
                else {
                    if (ligar){ligarRedeAlta();adicionarLog("Manual: RA ON");}
                    else{desligarRedeAlta();adicionarLog("Manual: RA OFF");}
                    msg="<div class='ok'>&#10003; Rede Alta "+
                        String(ligar?"ligada":"desligada")+"</div>";
                }
            } else if (qual=="rb") {
                if (curto.estado==CURTO_CONFIRMADO_BAIXA)
                    msg="<div class='er'>&#9888; Curto rede baixa</div>";
                else {
                    if (ligar){ligarRedeBaixa();adicionarLog("Manual: RB ON");}
                    else{desligarRedeBaixa();adicionarLog("Manual: RB OFF");}
                    msg="<div class='ok'>&#10003; Rede Baixa "+
                        String(ligar?"ligada":"desligada")+"</div>";
                }
            } else if (qual=="le") {
                if (ligar){ligarLampExt();adicionarLog("Manual: LE ON");}
                else{desligarLampExt();adicionarLog("Manual: LE OFF");}
                msg="<div class='ok'>&#10003; Lamp.Ext "+
                    String(ligar?"ligada":"desligada")+"</div>";
            } else if (qual=="li") {
                if (ligar){ligarLampInt();adicionarLog("Manual: LI ON");}
                else{desligarLampInt();adicionarLog("Manual: LI OFF");}
                msg="<div class='ok'>&#10003; Lamp.Int "+
                    String(ligar?"ligada":"desligada")+"</div>";
            }
        }
    }

    bool curtoAlt=(curto.estado==CURTO_CONFIRMADO_ALTA ||curto.redeAltaCurto);
    bool curtoBx =(curto.estado==CURTO_CONFIRMADO_BAIXA||curto.redeBaixaCurto);
    bool curtoInv=(curto.estado==CURTO_CONFIRMADO_BAIXA||
                   curto.estado==CURTO_AGUARDANDO_RESET);

    String c = msg;
    c += F("<h3>&#9888; Protecao de Curto</h3>");
    c += htmlStatusCurto();

    c += F("<h3>&#128268; Controle Manual</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "&#128161; Controle automatico continua ativo. "
           "Acoes manuais podem ser sobrescritas.</div>");
    c += btnRele("&#9889; Inversor",       "inv", estados.inversor,  curtoInv);
    c += btnRele("&#128268; Rede Alta",    "ra",  estados.redesAlta, curtoAlt);
    c += btnRele("&#128268; Rede Baixa",   "rb",  estados.redeBaixa, curtoBx);
    c += btnRele("&#128161; Lamp. Externa","le",  estados.lampExt,   false);
    c += btnRele("&#128161; Lamp. Interna","li",  estados.lampInt,   false);

    c += F("<h3>&#9889; Niveis de Corte (%)</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "Hierarquia: LampInt &rarr; LampExt &rarr; "
           "Rede Alta &rarr; Rede Baixa+Inversor</div>");
    c += F("<form method='POST' action='/redes'>"
           "<input type='hidden' name='acao' value='config'>");

    auto campo = [&](const char* lbl, const char* nm,
                     float val, const char* dica) {
        c += "<label>" + String(lbl) + ":</label>";
        c += "<div class='nf' style='font-size:10px;margin:3px 0'>" +
             String(dica) + "</div>";
        char v[16]; snprintf(v,sizeof(v),"value='%.0f'",val);
        c += "<input type='number' name='" + String(nm) +
             "' min='0' max='90' step='1' required " + String(v) + ">";
    };
    campo("&#128161; Lamp. Interna (%)","corteLampInt",redes.corteLampInt,
          "0% = nunca corta automaticamente");
    campo("&#128161; Lamp. Externa (%)","corteLampExt",redes.corteLampExt,
          "Corte por energia baixa (solar e separado)");
    campo("&#128268; Rede Alta (%)","corteAlta",redes.corteAlta,
          "Desliga equipamentos pesados");
    campo("&#128268; Rede Baixa+Inversor (%)","corteBaixa",redes.corteBaixa,
          "Corte total. Religar automatico ao recuperar");
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");

    // Mini-barras
    c += F("<h3>&#128202; Status Atual</h3>");
    float pct = bateria.calibrada ? bateria.percentualAtual : 0.0f;
    auto miniCorte = [&](const char* nm, float corte, bool ativo) {
        c += "<div style='margin:6px 0'>";
        c += "<div style='display:flex;justify-content:space-between;"
             "font-size:11px;margin-bottom:2px'>";
        c += "<span style='color:#90a4ae'>" + String(nm) + "</span>";
        c += "<span style='color:" +
             String(ativo?"#43a047":"#ef5350") + "'>";
        c += String(ativo?"&#9679; ON":"&#9675; OFF") +
             " | " + String((int)corte) + "%</span></div>";
        String cb = pct>=50?"#43a047":pct>=25?"#ef6c00":"#c62828";
        c += "<div style='position:relative;background:#111;"
             "border:1px solid #333;border-radius:4px;height:14px'>";
        c += "<div style='width:" + String((int)pct) +
             "%;height:100%;background:" + cb + ";border-radius:3px'></div>";
        c += "<div style='position:absolute;top:0;left:" +
             String((int)corte) + "%;width:2px;height:100%;"
             "background:#ef5350'></div></div></div>";
    };
    miniCorte("Lamp. Interna", redes.corteLampInt, estados.lampInt);
    miniCorte("Lamp. Externa", redes.corteLampExt, estados.lampExt);
    miniCorte("Rede Alta",     redes.corteAlta,    estados.redesAlta);
    miniCorte("Rede Baixa",    redes.corteBaixa,   estados.redeBaixa);
    if (bateria.calibrada)
        c += "<div style='font-size:11px;color:#90a4ae;text-align:right'>"
             "Bat: <b style='color:#4fc3f7'>" + String((int)pct) + "%</b></div>";

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Redes",c));
}

// ═══════════════════════════════════════════════════════════
// [SENS-WEB] PÁGINA DE SENSORES
// ═══════════════════════════════════════════════════════════
void handleSensores() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // ── Calibração 1 ponto Painel ──────────────────────
        // Usuário vê corrente no controlador de carga → digita → salva
        if (acao == "calib_painel") {
            float iReal = server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>50) {
                msg="<div class='er'>&#10007; Corrente invalida (0-50A)</div>";
            } else {
                int mvAt = sensores.sinalPainel_mV;
                // Formula: corrBruta = -(mv - offset)/sens
                // offset = mv + iReal*sens  (cabos invertidos)
                int novoOff = (int)((float)mvAt + iReal*calibPainel.sensMvA);
                calibPainel.offsetMV = novoOff;
                prefs.begin("rrfs_cal",false);
                prefs.putInt("pn_off", novoOff);
                prefs.end();
                snprintf(tempBuf,sizeof(tempBuf),
                         "Calib Painel: off=%dmV (mv=%d I=%.2fA)",
                         novoOff, mvAt, iReal);
                adicionarLog(tempBuf);
                msg="<div class='ok'>&#10003; Painel calibrado! "
                    "Offset=" + String(novoOff) + "mV | "
                    "Persiste apos reboot.</div>";
            }
        }
        // ── Calibração 1 ponto Casa ────────────────────────
        // Usuário vê corrente no alicate → digita → salva
        else if (acao == "calib_casa") {
            float iReal = server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>100) {
                msg="<div class='er'>&#10007; Corrente invalida (0-100A)</div>";
            } else {
                int mvAt = sensores.sinalCasa_mV;
                // Formula: corrBruta = (mv - offset)/sens
                // offset = mv - iReal*sens
                int novoOff = (int)((float)mvAt - iReal*calibCasa.sensMvA);
                calibCasa.offsetMV  = novoOff;
                calibCasa.calibrado = true;
                prefs.begin("rrfs_cal",false);
                prefs.putInt("casa_off", novoOff);
                prefs.end();
                snprintf(tempBuf,sizeof(tempBuf),
                         "Calib Casa: off=%dmV (mv=%d I=%.2fA)",
                         novoOff, mvAt, iReal);
                adicionarLog(tempBuf);
                msg="<div class='ok'>&#10003; Casa calibrada! "
                    "Offset=" + String(novoOff) + "mV | "
                    "Persiste apos reboot.</div>";
            }
        }
        // ── Calibração 2 pontos Casa (avançada) ───────────
        else if (acao == "calib_casa_2pt") {
            float i1=server.arg("i1").toFloat(), mv1=server.arg("mv1").toFloat();
            float i2=server.arg("i2").toFloat(), mv2=server.arg("mv2").toFloat();
            if (i1<0||i2<0||i1>=i2)
                msg="<div class='er'>&#10007; I1 deve ser menor que I2</div>";
            else if (mv1<=0||mv2<=mv1)
                msg="<div class='er'>&#10007; mV invalidos (mV1 < mV2)</div>";
            else {
                float ns=(mv2-mv1)/(i2-i1);
                float no=(float)((int)(mv1-ns*i1));
                if (ns<5||ns>100)
                    msg="<div class='er'>&#10007; Sensibilidade fora (5-100mV/A)</div>";
                else {
                    calibCasa.sensMvA  = ns;
                    calibCasa.offsetMV = (int)no;
                    calibCasa.calibrado= true;
                    prefs.begin("rrfs_cal",false);
                    prefs.putFloat("casa_sens",ns);
                    prefs.putInt  ("casa_off",(int)no);
                    prefs.end();
                    snprintf(tempBuf,sizeof(tempBuf),
                             "Calib Casa 2pt: off=%dmV sens=%.2fmV/A",
                             (int)no,ns);
                    adicionarLog(tempBuf);
                    msg="<div class='ok'>&#10003; Casa 2pt: Sens="+
                        String(ns,2)+"mV/A Off="+String((int)no)+"mV | "
                        "Persiste apos reboot.</div>";
                }
            }
        }
        else if (acao == "toggle_autocal") {
            autoCalibPainelAtivo = !autoCalibPainelAtivo;
            prefs.begin("rrfs_cal",false);
            prefs.putBool("auto_cal",autoCalibPainelAtivo);
            prefs.end();
            snprintf(tempBuf,sizeof(tempBuf),
                     "AutoCal Painel: %s",
                     autoCalibPainelAtivo?"ON":"OFF");
            adicionarLog(tempBuf);
            msg="<div class='ok'>&#10003; "+String(tempBuf)+"</div>";
        }
        else if (acao == "reset_calib") {
            prefs.begin("rrfs_cal",false);
            prefs.remove("pn_off"); prefs.remove("pn_sens");
            prefs.remove("casa_off"); prefs.remove("casa_sens");
            prefs.end();
            carregarCalibSensores();
            adicionarLog("Calibracao resetada (fabrica)");
            msg="<div class='ok'>&#10003; Resetado para fabrica</div>";
        }
    }

    prefs.begin("rrfs_cal",true);
    bool temP = prefs.isKey("pn_off");
    bool temC = prefs.isKey("casa_off");
    prefs.end();

    String c = msg;
    c += F("<h3>&#128270; Leituras em Tempo Real</h3>");

    // Card genérico sensor
    auto cardS = [&](const char* titulo, float cor,
                     int mv, int offMV, float sens,
                     const char* desc) {
        String cl = cor>0.3f?"ok":"";
        c += "<div class='sc " + cl + "' style='margin-bottom:8px'>";
        c += "<div class='lbl'>" + String(titulo) + "</div>";
        c += "<div style='display:flex;justify-content:space-between;align-items:center'>";
        String corC = cor>0.3f?"#43a047":"#546e7a";
        char buf[12]; snprintf(buf,sizeof(buf),"%.2fA",cor);
        c += "<div class='val' style='color:"+corC+";font-size:22px'>"+String(buf)+"</div>";
        c += "<div style='text-align:right'>";
        snprintf(buf,sizeof(buf),"%dmV",mv);
        c += "<div class='sub'>ADC: "+String(buf)+"</div>";
        snprintf(buf,sizeof(buf),"Off:%d Sens:%.1f",offMV,sens);
        c += "<div class='sub'>"+String(buf)+"mV/A</div></div></div>";
        c += "<div class='sub' style='margin-top:4px'>"+String(desc)+"</div></div>";
    };

    cardS("&#9728; Painel (ACS758 50A Bidirecional)",
          sensores.correntePainel,
          sensores.sinalPainel_mV, calibPainel.offsetMV, calibPainel.sensMvA,
          "Cabos invertidos | Positivo = gerando");
    cardS("&#127968; Casa (ACS758LCB-100B Unidirecional)",
          sensores.correnteCasa,
          sensores.sinalCasa_mV, calibCasa.offsetMV, calibCasa.sensMvA,
          "Unidirecional | Positivo = consumindo");

    // Sistema
    {
        String corS = sensores.correnteSistema>0.15f?"#ff9800":"#546e7a";
        c += F("<div class='sc' style='margin-bottom:8px'>");
        c += F("<div class='lbl'>&#128268; Sistema (ESP32+Controlador)</div>");
        c += "<div class='val' style='color:"+corS+";font-size:22px'>";
        char buf[12]; snprintf(buf,sizeof(buf),"%.3fA",sensores.correnteSistema);
        c += String(buf);
        c += F("</div><div class='sub'>Calculado do sensor painel "
               "(apenas sem geracao)</div></div>");
    }
    // Bateria calc
    {
        float ib = sensores.correnteBateria;
        String cf = ib>0.3f?"#43a047":ib<-0.3f?"#ef6c00":"#fdd835";
        String si = ib>0.3f?"+":ib<-0.3f?"-":"~";
        String cl = ib>0.3f?"ok":ib<-0.3f?"wn":"";
        c += "<div class='sc " + cl + "' style='margin-bottom:8px'>";
        c += F("<div class='lbl'>&#128267; Bateria (calculado)</div>");
        c += "<div class='val' style='color:"+cf+"'>";
        char buf[12]; snprintf(buf,sizeof(buf),"%.2fA",fabsf(ib));
        c += si + String(buf);
        c += "</div><div class='sub'>";
        c += ib>0.3f?"Carregando":ib<-0.3f?"Descarregando":"Equilibrio";
        char buf2[60];
        snprintf(buf2,sizeof(buf2)," | P%.2f-C%.2f-S%.3f",
                 sensores.correntePainel,sensores.correnteCasa,
                 sensores.correnteSistema);
        c += String(buf2) + F("</div></div>");
    }

    // kW informativos
    c += "<div class='nf' style='font-size:10px'>";
    c += "<b>kW informativos</b> (I × " +
         String(bateria.tensaoNominal,1) + "V nom):<br>";
    char bKW[100];
    snprintf(bKW,sizeof(bKW),
             "Painel:<b>%.0fW</b> Casa:<b>%.0fW</b> "
             "Bat:<b>%.0fW</b> Sist:<b>%.0fW</b>",
             sensores.correntePainel*bateria.tensaoNominal,
             sensores.correnteCasa*bateria.tensaoNominal,
             fabsf(sensores.correnteBateria)*bateria.tensaoNominal,
             sensores.correnteSistema*bateria.tensaoNominal);
    c += String(bKW) + F("</div>");

    // Inversor
    c += F("<h3>&#9889; Status Inversor</h3>");
    {
        bool ia = sensores.inversoreAtivo;
        c += "<div class='sc " + String(ia?"ok":"er") + "'>";
        c += "<div class='cr'><span class='lbl'>Sinal STATUS_INV</span>";
        c += "<span class='cs' style='color:" + String(ia?"#43a047":"#ef5350") + "'>";
        c += ia ? "&#9679; ATIVO" : "&#9675; INATIVO";
        c += F("</span></div>");
        char buf[60];
        snprintf(buf,sizeof(buf),"ADC:%dmV Limiar:%dmV Rele:%s",
                 sensores.sinalInversor_mV,INV_THRESHOLD_MV,
                 estados.inversor?"ON":"OFF");
        c += "<div class='sub'>" + String(buf) + F("</div></div>");
    }

    // ══════════════════════════════════════════════════════
    // CALIBRAÇÃO PAINEL — 1 PONTO (RECOMENDADA)
    // Usuário vê corrente no controlador de carga
    // ══════════════════════════════════════════════════════
    c += F("<h3>&#9728; Calibrar Painel (1 Ponto)</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "&#128161; <b>Como calibrar:</b><br>"
           "1. Veja a corrente no <b>display do controlador de carga</b><br>"
           "2. Digite exatamente esse valor abaixo<br>"
           "3. Salve. O sistema calcula o offset e salva na memoria.<br>"
           "4. O offset persiste mesmo apos reboot/atualizacao.</div>");
    char bAt[16];
    snprintf(bAt,sizeof(bAt),"%.2f",sensores.correntePainel);
    c += "<div class='nf' style='font-size:10px'>Mostrando agora: <b>" +
         String(bAt) + "A</b> | ADC: " +
         String(sensores.sinalPainel_mV) + "mV</div>";
    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_painel'>"
           "<label>Corrente real (A) - valor do controlador:</label>"
           "<input type='number' name='corrente_real' step='0.01' "
           "min='0' max='50' required placeholder='Ex: 8.50'>"
           "<button type='submit' class='btn-g'>"
           "&#9728; Calibrar e Salvar Painel</button></form>");
    if (temP)
        c += F("<div class='ok' style='font-size:10px;margin-top:-6px'>"
               "&#10003; Painel: calibracao personalizada ativa</div>");

    // ══════════════════════════════════════════════════════
    // CALIBRAÇÃO CASA — 1 PONTO (RECOMENDADA)
    // Usuário vê corrente no alicate amperímetro
    // ══════════════════════════════════════════════════════
    c += F("<h3>&#127968; Calibrar Casa (1 Ponto)</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "&#128161; <b>Como calibrar:</b><br>"
           "1. Use um <b>alicate amperimetro</b> no fio de entrada da casa<br>"
           "2. Digite exatamente o valor que o alicate mostra<br>"
           "3. Salve. Persiste apos reboot.</div>");
    snprintf(bAt,sizeof(bAt),"%.2f",sensores.correnteCasa);
    c += "<div class='nf' style='font-size:10px'>Mostrando agora: <b>" +
         String(bAt) + "A</b> | ADC: " +
         String(sensores.sinalCasa_mV) + "mV</div>";
    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_casa'>"
           "<label>Corrente real (A) - valor do alicate:</label>"
           "<input type='number' name='corrente_real' step='0.01' "
           "min='0' max='100' required placeholder='Ex: 12.30'>"
           "<button type='submit' class='btn-g'>"
           "&#127968; Calibrar e Salvar Casa</button></form>");
    if (temC)
        c += F("<div class='ok' style='font-size:10px;margin-top:-6px'>"
               "&#10003; Casa: calibracao personalizada ativa</div>");

    // Autoaprendizado
    c += F("<h3>&#129302; Autoaprendizado Painel</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "Aprende o zero do sensor a cada noite. "
           "Pode sobrescrever calibracao manual. "
           "Desative se notar deriva apos calibrar.</div>");
    c += "<div class='sc' style='text-align:center'>";
    c += "<div class='lbl'>Status</div>";
    c += "<div class='val' style='color:" +
         String(autoCalibPainelAtivo?"#43a047":"#ef6c00") + "'>";
    c += autoCalibPainelAtivo ? "ATIVADO" : "DESATIVADO";
    c += F("</div></div>");
    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='toggle_autocal'>"
           "<button type='submit' class='");
    c += autoCalibPainelAtivo ? "btn-o" : "btn-g";
    c += F("'>");
    c += autoCalibPainelAtivo ?
         "&#128683; Desativar Autoaprendizado" :
         "&#9679; Reativar Autoaprendizado";
    c += F("</button></form>");

    // Calibração 2 pontos casa
    c += F("<h3>&#128295; Casa - 2 Pontos (Avancada)</h3>");
    c += "<div class='nf'>Sens atual: " +
         String(calibCasa.sensMvA,1) + "mV/A | Off: " +
         String(calibCasa.offsetMV) + "mV</div>";
    c += F("<form method='POST' action='/sensores'>"
           "<input type='hidden' name='acao' value='calib_casa_2pt'>"
           "<div style='display:grid;grid-template-columns:1fr 1fr;gap:8px'>"
           "<div>"
           "<label>I1 (A):</label>"
           "<input type='number' name='i1' step='0.1' min='0' max='100' "
           "required placeholder='Ex: 2.9'>"
           "<label>mV1:</label>"
           "<input type='number' name='mv1' step='1' min='0' max='3300' "
           "required placeholder='Ex: 666'>"
           "</div><div>"
           "<label>I2 (A):</label>"
           "<input type='number' name='i2' step='0.1' min='0' max='100' "
           "required placeholder='Ex: 34.0'>"
           "<label>mV2:</label>"
           "<input type='number' name='mv2' step='1' min='0' max='3300' "
           "required placeholder='Ex: 2011'>"
           "</div></div>"
           "<button type='submit' class='btn-o'>"
           "&#127919; Calibrar 2 Pontos</button></form>");

    if (temP || temC) {
        c += F("<form method='POST' action='/sensores'>"
               "<input type='hidden' name='acao' value='reset_calib'>"
               "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
               "&#8635; Resetar para Fabrica</button></form>");
    }
    c += F("<h3>&#128200; Info ADC</h3>"
           "<div class='nf' style='font-size:10px'>"
           "Resolucao: 12 bits | Vref: 3300mV | "
           "Amostras: 150 | Buffer MM: 10</div>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Sensores",c));
}

// ═══════════════════════════════════════════════════════════
// [HIST-WEB] HISTÓRICO
// ═══════════════════════════════════════════════════════════
void handleHistorico() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String c = "";
    c += F("<h3>&#128200; Totais desde o Inicio</h3>");
    c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>");
    auto cardT = [&](const char* ico, const char* nm, float ah, const char* cor) {
        c += "<div class='sc' style='text-align:center'>";
        c += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
        char buf[20]; snprintf(buf,sizeof(buf),"%.1fAh",ah);
        c += "<div class='val' style='color:" + String(cor) + "'>" +
             String(buf) + "</div>";
        char buf2[24];
        snprintf(buf2,sizeof(buf2),"~%.2fkWh",
                 ah*bateria.tensaoNominal/1000.0f);
        c += "<div class='sub'>" + String(buf2) + "</div></div>";
    };
    cardT("&#9728;","Painel",   contadores.painelAh, "#fdd835");
    cardT("&#127968;","Casa",   contadores.casaAh,   "#4fc3f7");
    cardT("&#9650;","Bat+",     contadores.batCargAh,"#43a047");
    cardT("&#9660;","Bat-",     contadores.batDescAh,"#ef6c00");
    c += F("</div>");
    c += "<div class='nf' style='font-size:10px'>kWh @ " +
         String(bateria.tensaoNominal,1) + F("V nominal (informativo)</div>");

    // Últimos 7 dias
    c += F("<h3>&#128197; Ultimos 7 Dias</h3>");
    int maxD = histTotalDias < 7 ? histTotalDias : 7;
    if (maxD == 0) {
        c += F("<div class='nf'>Sem historico ainda.</div>");
    } else {
        c += F("<div style='overflow-x:auto'>"
               "<table style='width:100%;border-collapse:collapse;font-size:11px'>"
               "<tr style='color:#546e7a'>"
               "<th style='text-align:left;padding:4px'>Data</th>"
               "<th style='padding:4px'>&#9728;Painel</th>"
               "<th style='padding:4px'>&#127968;Casa</th>"
               "<th style='padding:4px'>&#9650;Bat+</th>"
               "<th style='padding:4px'>&#9660;Bat-</th></tr>");
        for (int i = 0; i < maxD; i++) {
            RegistroDia dT;
            bool ok = (i==0)?(dT=histDiaAtual,true)
                            :carregarDiaFlash(
                                ((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,dT);
            if (!ok||!dT.valido) continue;
            float pAh=dT.totalPainelAh, cAh=dT.totalCasaAh;
            float bCAh=dT.totalBatCargAh, bDAh=dT.totalBatDescAh;
            if (i==0){ pAh+=acum.painelAh; cAh+=acum.casaAh;
                       bCAh+=acum.batCargAh; bDAh+=acum.batDescAh; }
            char bD[16];
            snprintf(bD,sizeof(bD),"%02d/%02d/%04d",dT.dia,dT.mes,dT.ano);
            c += "<tr style='border-top:1px solid #222;background:" +
                 String(i==0?"#0a1f0a":"") + "'>";
            c += "<td style='padding:4px;color:" +
                 String(i==0?"#43a047":"#90a4ae") + "'>";
            if (i==0) c += "<b>";
            c += String(bD); if (i==0) c += " (hoje)</b>";
            c += F("</td>");
            char buf[10];
            snprintf(buf,sizeof(buf),"%.1f",pAh);
            c+="<td style='padding:4px;text-align:center;color:#fdd835'>"+String(buf)+"Ah</td>";
            snprintf(buf,sizeof(buf),"%.1f",cAh);
            c+="<td style='padding:4px;text-align:center;color:#4fc3f7'>"+String(buf)+"Ah</td>";
            snprintf(buf,sizeof(buf),"%.1f",bCAh);
            c+="<td style='padding:4px;text-align:center;color:#43a047'>"+String(buf)+"Ah</td>";
            snprintf(buf,sizeof(buf),"%.1f",bDAh);
            c+="<td style='padding:4px;text-align:center;color:#ef6c00'>"+String(buf)+"Ah</td>";
            c += F("</tr>");
        }
        c += F("</table></div>");
    }

    // Gráfico por hora hoje
    c += F("<h3>&#128200; Hoje - Por Hora</h3>");
    {
        float maxAh = 0.1f;
        struct tm ti; int hA=-1;
        if (getLocalTime(&ti)) hA=ti.tm_hour;
        for (int h=0;h<24;h++){
            float v=histDiaAtual.horas[h].painelAh+histDiaAtual.horas[h].casaAh;
            if (v>maxAh) maxAh=v;
        }
        c += F("<div style='overflow-x:auto'>");
        c += F("<div style='display:flex;align-items:flex-end;gap:2px;"
               "height:80px;padding:4px 0'>");
        for (int h=0;h<24;h++){
            float pAh=histDiaAtual.horas[h].painelAh;
            float cAh=histDiaAtual.horas[h].casaAh;
            if (h==hA){pAh+=acum.painelAh;cAh+=acum.casaAh;}
            int aP=(int)((pAh/maxAh)*70);
            int aC=(int)((cAh/maxAh)*70);
            if (aP>70)aP=70; if(aC>70)aC=70;
            bool eh=(h==hA);
            c += F("<div style='flex:1;display:flex;flex-direction:column;"
                   "align-items:center;min-width:8px'>");
            c += "<div style='width:100%;height:" + String(aP) +
                 "px;background:" + String(eh?"#fdd835":"#a89420") +
                 ";border-radius:2px 2px 0 0'></div>";
            c += "<div style='width:100%;height:" + String(aC) +
                 "px;background:" + String(eh?"#4fc3f7":"#1e5f7a") +
                 "'></div>";
            c += "<div style='font-size:7px;color:#546e7a'>";
            if (h%4==0||eh) c+=String(h)+"h"; else c+=" ";
            c += F("</div></div>");
        }
        c += F("</div>");
        c += F("<div style='display:flex;gap:12px;font-size:10px;color:#78909c'>"
               "<span><span style='color:#fdd835'>&#9632;</span> Painel(Ah)</span>"
               "<span><span style='color:#4fc3f7'>&#9632;</span> Casa(Ah)</span>"
               "</div></div>");
    }

    // Gráfico RT 60s
    c += F("<h3>&#128200; Corrente Tempo Real (60s)</h3>");
    {
        int total = histSnapFull ? HIST_SNAP_PONTOS : histSnapIdx;
        if (total < 2) {
            c += F("<div class='nf'>Aguardando dados...</div>");
        } else {
            float maxC = 0.5f;
            for (int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                if (histCorrPainel[idx]>maxC) maxC=histCorrPainel[idx];
                if (histCorrCasa[idx]>maxC)   maxC=histCorrCasa[idx];
                if (fabsf(histCorrBat[idx])>maxC) maxC=fabsf(histCorrBat[idx]);
            }
            c += F("<div style='position:relative;height:80px;background:#111;"
                   "border:1px solid #333;border-radius:4px;overflow:hidden'>");
            c += F("<div style='position:absolute;top:50%;left:0;right:0;"
                   "height:1px;background:#333'></div>");
            c += "<svg viewBox='0 0 " + String(total) + " 80' "
                 "preserveAspectRatio='none' "
                 "style='width:100%;height:100%;position:absolute'>";
            auto py=[&](float v,bool bi)->float{
                return bi ? 40.0f-(v/maxC)*38.0f : 78.0f-(v/maxC)*76.0f;
            };
            // Painel
            String pP="M"; for(int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                char pt[20]; snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrPainel[idx],false));
                if(i>0)pP+="L"; pP+=String(pt);
            }
            c+="<path d='"+pP+"' fill='none' stroke='#fdd835' stroke-width='1.5'/>";
            // Casa
            String pC="M"; for(int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                char pt[20]; snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrCasa[idx],false));
                if(i>0)pC+="L"; pC+=String(pt);
            }
            c+="<path d='"+pC+"' fill='none' stroke='#4fc3f7' stroke-width='1.5'/>";
            // Bat
            String pB="M"; for(int i=0;i<total;i++){
                int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
                char pt[20]; snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrBat[idx],true));
                if(i>0)pB+="L"; pB+=String(pt);
            }
            c+="<path d='"+pB+"' fill='none' stroke='#ce93d8' stroke-width='1' stroke-dasharray='3 2'/>";
            c+=F("</svg>");
            char bMax[12]; snprintf(bMax,sizeof(bMax),"%.1fA",maxC);
            c+="<div style='position:absolute;top:2px;right:4px;font-size:9px;color:#546e7a'>"+
               String(bMax)+"</div>";
            c+=F("<div style='position:absolute;bottom:2px;right:4px;"
                 "font-size:9px;color:#546e7a'>0A</div></div>");
            c+=F("<div style='display:flex;gap:10px;font-size:10px;color:#78909c;margin-top:4px'>"
                 "<span><span style='color:#fdd835'>&#9660;</span>Painel</span>"
                 "<span><span style='color:#4fc3f7'>&#9660;</span>Casa</span>"
                 "<span><span style='color:#ce93d8'>---</span>Bat</span></div>");
        }
    }
    char bI[60];
    snprintf(bI,sizeof(bI),"Hist: %d/%d dias",histTotalDias,HIST_DIAS);
    c += "<div class='nf' style='font-size:10px'>" + String(bI) + F("</div>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Historico",c));
}

// ═══════════════════════════════════════════════════════════
// [PREV-WEB] PREVISÃO
// ═══════════════════════════════════════════════════════════
void handlePrevisao() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config") {
            double lat=server.arg("lat").toDouble();
            double lon=server.arg("lon").toDouble();
            float  amp=server.arg("amp").toFloat();
            float  efP=server.arg("ef").toFloat();
            if (lat<-90||lat>90)
                msg="<div class='er'>&#10007; Latitude invalida</div>";
            else if (lon<-180||lon>180)
                msg="<div class='er'>&#10007; Longitude invalida</div>";
            else if (amp<1||amp>500)
                msg="<div class='er'>&#10007; Amperagem invalida</div>";
            else if (efP<30||efP>100)
                msg="<div class='er'>&#10007; Eficiencia invalida</div>";
            else {
                configLocal.latitude=lat; configLocal.longitude=lon;
                configLocal.ampControlador=amp; configLocal.eficiencia=efP/100.0f;
                configLocal.configurado=true;
                configLocal.fatorCalib=1.0f; configLocal.amostrasCalib=0;
                salvarConfigLocal(); ultimaPrev=0;
                msg="<div class='ok'>&#10003; Salvo! Buscando previsao...</div>";
            }
        } else if (acao=="buscar") {
            ultimaPrev=0; adicionarLog("Prev: busca manual");
            msg=F("<div class='ok'>&#10003; Solicitado! Aguarde ~30s.</div>");
        } else if (acao=="reset_calib") {
            configLocal.fatorCalib=1.0f; configLocal.amostrasCalib=0;
            salvarConfigLocal(); adicionarLog("Prev: calib resetada");
            msg=F("<div class='ok'>&#10003; Calib resetada</div>");
        }
    }

    String c = msg;
    c += F("<h3>&#127774; Status</h3>");
    if (!configLocal.configurado) {
        c += F("<div class='wn'>&#9888; Localizacao nao configurada.</div>");
    } else if (!previsao.valido) {
        c += F("<div class='wn'>&#9203; Aguardando dados...</div>");
        if (strlen(previsao.erro)>0)
            c += "<div class='er'>&#10007; " + String(previsao.erro) + F("</div>");
    } else {
        struct tm ti; bool temT=getLocalTime(&ti);
        struct tm tiA={0};
        if (temT){ tiA=ti; tiA.tm_mday+=1; tiA.tm_hour=0; mktime(&tiA); }

        float totHR=0, totA=0; int cHR=0, cA=0;
        for (uint8_t i=0;i<previsao.totalHoras;i++){
            PrevisaoHora& ph=previsao.horas[i];
            if (!ph.valido) continue;
            if (temT&&ph.dia==(uint8_t)ti.tm_mday&&
                ph.mes==(uint8_t)(ti.tm_mon+1)&&
                ph.hora>=(uint8_t)ti.tm_hour){
                totHR+=ph.correnteEstimada; cHR++;
            }
            if (temT&&ph.dia==(uint8_t)tiA.tm_mday&&
                ph.mes==(uint8_t)(tiA.tm_mon+1)){
                totA+=ph.correnteEstimada; cA++;
            }
        }
        float mHR=cHR>0?totHR/cHR:0;
        float mA =cA >0?totA/cA :0;

        c += "<div class='sc ok'>";
        char bPrev[80];
        snprintf(bPrev,sizeof(bPrev),
                 "%d horas | Calib:%.3f (%d amostras)",
                 previsao.totalHoras,configLocal.fatorCalib,
                 configLocal.amostrasCalib);
        c += "<div class='val' style='font-size:12px'>" + String(bPrev) + F("</div>");
        unsigned long sAt=(millis()-previsao.ultimaAtualizacao)/1000;
        c += "<div class='sub'>Ha " + String(sAt) + F("s</div></div>");

        // Cards hoje/amanha
        c += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:8px 0'>");
        auto cardPrev=[&](const char* tit, float tot, float med,
                          int nh, const char* cor){
            c += "<div class='sc' style='border-color:" + String(cor) + "'>";
            c += "<div class='lbl'>" + String(tit) + F("</div>");
            char buf[20]; snprintf(buf,sizeof(buf),"%.1fAh",tot);
            c += "<div style='font-size:22px;font-weight:bold;color:" +
                 String(cor) + ";text-align:center'>" + String(buf) + F("</div>");
            char buf2[60];
            snprintf(buf2,sizeof(buf2),"Med:%.1fA | %dh | ~%.2fkWh",
                     med,nh,tot*bateria.tensaoNominal/1000.0f);
            c += "<div style='font-size:10px;color:#78909c;text-align:center'>" +
                 String(buf2) + F("</div></div>");
        };
        cardPrev("&#9728; Hoje Restante",totHR,mHR,cHR,"#fdd835");
        cardPrev("&#127774; Amanha",     totA, mA, cA, "#4fc3f7");
        c += F("</div>");

        if (decisao.desligarRedeAltaNoite||decisao.desligarLampExtNoite||
            decisao.alertaEnergiaInsuficiente){
            c += F("<div class='wn' style='font-size:11px'>"
                   "<b>&#9888; Decisoes ativas:</b><br>");
            if (decisao.desligarRedeAltaNoite)
                c += F("&#128268; Rede Alta OFF (noite)<br>");
            if (decisao.desligarLampExtNoite)
                c += F("&#128161; Lamp.Ext OFF (critico)<br>");
            if (decisao.alertaEnergiaInsuficiente)
                c += F("&#128267; Energia pode ser insuficiente!<br>");
            if (strlen(decisao.motivo)>0)
                c += "<span style='color:#78909c'>"+String(decisao.motivo)+"</span>";
            c += F("</div>");
        }

        // Tabela 24h
        c += F("<h3>&#128200; Proximas 24 Horas</h3>");
        float maxC=0.1f;
        for (uint8_t i=0;i<previsao.totalHoras;i++)
            if (previsao.horas[i].correnteEstimada>maxC)
                maxC=previsao.horas[i].correnteEstimada;

        c += F("<div style='overflow-x:auto'>"
               "<table style='width:100%;border-collapse:collapse;font-size:11px'>"
               "<tr style='color:#546e7a'>"
               "<th style='text-align:left;padding:4px'>Hora</th>"
               "<th>Rad</th><th>Nuv</th><th>Tmp</th>"
               "<th>A est.</th><th>Ah est.</th></tr>");
        int exib=0;
        for (uint8_t i=0;i<previsao.totalHoras&&exib<24;i++){
            PrevisaoHora& ph=previsao.horas[i];
            if (!ph.valido) continue;
            if (temT&&ph.dia==(uint8_t)ti.tm_mday&&
                ph.mes==(uint8_t)(ti.tm_mon+1)&&
                ph.hora<(uint8_t)ti.tm_hour) continue;
            exib++;
            bool eHA=(temT&&ph.hora==(uint8_t)ti.tm_hour&&ph.dia==(uint8_t)ti.tm_mday);
            bool eAmn=(temT&&ph.dia==(uint8_t)tiA.tm_mday&&ph.mes==(uint8_t)(tiA.tm_mon+1));
            c += "<tr style='border-top:1px solid #222;background:" +
                 String(eHA?"#0a1f0a":eAmn?"#0a0a1f":"") + "'>";
            char bH[10]; snprintf(bH,sizeof(bH),"%02d:00",ph.hora);
            c += "<td style='padding:4px;color:" +
                 String(eHA?"#43a047":eAmn?"#90caf9":"#90a4ae") + "'>";
            if (eHA) c += "<b>";
            c += String(bH);
            if (eHA) c += " &#9664;</b>";
            if (eAmn&&ph.hora==0) c+=F("<span style='font-size:9px;color:#546e7a'>(amanha)</span>");
            c += F("</td>");
            char buf[10];
            snprintf(buf,sizeof(buf),"%.0f",ph.radiacao);
            String cRad=ph.radiacao>500?"#fdd835":ph.radiacao>200?"#ef6c00":"#78909c";
            c+="<td style='padding:4px;text-align:center;color:"+cRad+"'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.0f%%",ph.coberturaNuvens);
            c+="<td style='padding:4px;text-align:center;color:#78909c'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.0f&deg;",ph.temperatura);
            c+="<td style='padding:4px;text-align:center;color:#78909c'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.2fA",ph.correnteEstimada);
            String cC=ph.correnteEstimada>15?"#43a047":ph.correnteEstimada>8?"#fdd835":
                      ph.correnteEstimada>2?"#ef6c00":ph.correnteEstimada>0.1f?"#78909c":"#333";
            c+="<td style='padding:4px;text-align:center;font-weight:bold;color:"+cC+"'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.2fAh",ph.correnteEstimada);
            c+="<td style='padding:4px;text-align:center;color:"+cC+"'>";
            c+=ph.correnteEstimada>0.1f?String(buf):"<span style='color:#333'>-</span>";
            c+=F("</td></tr>");
        }
        c += F("</table></div>");
        if (configLocal.amostrasCalib>0){
            c+=F("<form method='POST' action='/previsao'>"
                 "<input type='hidden' name='acao' value='reset_calib'>"
                 "<button type='submit' class='btn-s btn-sm'>"
                 "&#8635; Resetar Calib Previsao</button></form>");
        }
    }

    // Config local
    c += F("<h3>&#127759; Configurar Local</h3>");
    c += F("<div class='nf' style='font-size:10px'>"
           "Dica: Google Maps → clique direito → copie coordenadas</div>");
    c += F("<form method='POST' action='/previsao'>"
           "<input type='hidden' name='acao' value='config'>"
           "<label>Latitude:</label>");
    char bLat[32],bLon[32];
    snprintf(bLat,sizeof(bLat),"%.8f",configLocal.configurado?configLocal.latitude:-15.78);
    snprintf(bLon,sizeof(bLon),"%.8f",configLocal.configurado?configLocal.longitude:-47.93);
    c += "<input type='number' name='lat' step='any' min='-90' max='90' required value='"
         + String(bLat) + "'>";
    c += F("<label>Longitude:</label>");
    c += "<input type='number' name='lon' step='any' min='-180' max='180' required value='"
         + String(bLon) + "'>";
    c += F("<label>Amperagem do Controlador (A):</label>");
    char bAmp[10];
    snprintf(bAmp,sizeof(bAmp),"%.0f",configLocal.configurado?configLocal.ampControlador:30.0f);
    c += "<input type='number' name='amp' step='1' min='1' max='500' required value='"
         + String(bAmp) + "'>";
    c += F("<label>Eficiencia do Sistema (%):</label>");
    char bEf[8];
    snprintf(bEf,sizeof(bEf),"%.0f",configLocal.configurado?configLocal.eficiencia*100.0f:80.0f);
    c += "<input type='number' name='ef' step='1' min='30' max='100' required value='"
         + String(bEf) + "'>";
    c += F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");
    if (configLocal.configurado){
        c+=F("<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='buscar'>"
             "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>"
             "&#8635; Buscar Agora</button></form>");
    }
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Previsao",c));
}

// ═══════════════════════════════════════════════════════════
// [CFG-WEB] CONFIGURAÇÃO GERAL
// ═══════════════════════════════════════════════════════════
void handleConfig() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "senha") {
            prefs.begin("rrfs",true);
            String usr=prefs.getString("user",""); prefs.end();
            String pA=server.arg("pass_atual");
            String pN=server.arg("pass_nova");
            String pC=server.arg("pass_conf");
            if (!verificarLogin(usr,pA))
                msg="<div class='er'>&#10007; Senha atual incorreta</div>";
            else if (pN.length()<4)
                msg="<div class='er'>&#10007; Nova senha min 4 chars</div>";
            else if (pN!=pC)
                msg="<div class='er'>&#10007; Senhas nao conferem</div>";
            else {
                salvarCredenciais(usr,pN);
                adicionarLog("Senha alterada");
                msg="<div class='ok'>&#10003; Senha alterada!</div>";
            }
        } else if (acao == "reset_historico") {
            if (server.arg("confirmar") != "SIM") {
                msg="<div class='er'>&#10007; Confirmacao invalida.</div>";
            } else {
                contadores={0,0,0,0};
                salvarContadores();
                memset(&acum,0,sizeof(AcumHora));
                acum.horaAtual=255; acum.ultimoAcum=0;
                RegistroDia dV; memset(&dV,0,sizeof(RegistroDia));
                for (uint16_t i=0;i<HIST_DIAS;i++) salvarDiaFlash(i,dV);
                histIndiceAtual=0; histTotalDias=0;
                memset(&histDiaAtual,0,sizeof(RegistroDia));
                salvarIndiceHistorico();
                memset(histCorrPainel,0,sizeof(histCorrPainel));
                memset(histCorrCasa,  0,sizeof(histCorrCasa));
                memset(histCorrBat,   0,sizeof(histCorrBat));
                histSnapIdx=0; histSnapFull=false;
                adicionarLog("HISTORICO apagado");
                msg="<div class='ok'>&#10003; Historico zerado!</div>";
            }
        }
    }

    String c = msg;
    c += F("<h3>&#128202; Sistema</h3>");
    c += F("<div class='sc'>");
    char bSys[80];
    snprintf(bSys,sizeof(bSys),"v16 | RAM:%.1fKB | Up:%lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000UL);
    c += "<div class='val' style='font-size:12px'>" + String(bSys) + F("</div>");
    String wf = modoAP ? "AP:10.0.0.1" :
                "STA:"+WiFi.localIP().toString()+" "+String(WiFi.RSSI())+"dBm";
    c += "<div class='sub'>" + wf + F("</div></div>");

    struct tm ti;
    if (getLocalTime(&ti)){
        char bH[40];
        snprintf(bH,sizeof(bH),"%02d/%02d/%04d %02d:%02d:%02d (UTC-3)",
                 ti.tm_mday,ti.tm_mon+1,ti.tm_year+1900,
                 ti.tm_hour,ti.tm_min,ti.tm_sec);
        c += "<div class='nf' style='text-align:center'>&#128336; " +
             String(bH) + F("</div>");
    } else {
        c += F("<div class='wn'>&#9888; NTP nao sincronizado</div>");
    }

    c += F("<h3>&#128274; Alterar Senha</h3>"
           "<form method='POST' action='/config'>"
           "<input type='hidden' name='acao' value='senha'>"
           "<label>Senha Atual:</label>"
           "<input type='password' name='pass_atual' required>"
           "<label>Nova Senha:</label>"
           "<input type='password' name='pass_nova' required>"
           "<label>Confirmar:</label>"
           "<input type='password' name='pass_conf' required>"
           "<button type='submit' class='btn-o'>&#128274; Alterar</button></form>");

    c += F("<h3>&#128465; Limpar Historico</h3>");
    c += F("<div class='er' style='font-size:11px'>"
           "&#9888; <b>ACAO PERMANENTE!</b> Apaga todo historico e contadores. "
           "Nao pode ser desfeito.</div>");
    c += F("<form method='POST' action='/config' "
           "onsubmit='return confirm(\"Apagar TUDO permanentemente?\")'>"
           "<input type='hidden' name='acao' value='reset_historico'>"
           "<input type='hidden' name='confirmar' value='SIM'>"
           "<button type='submit' class='btn-r'>"
           "&#128465; Apagar Historico</button></form>");

    c += F("<h3>&#9881; Acoes</h3>");
    c += F("<form method='POST' action='/ntp-sync'>"
           "<button type='submit' class='btn-s btn-sm'>&#128336; Sincronizar NTP</button></form>");
    c += F("<form method='GET' action='/setup-wifi'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
           "&#128246; Reconfigurar WiFi</button></form>");
    c += F("<form method='GET' action='/ota'>"
           "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>"
           "&#128640; Atualizar Firmware</button></form>");
    c += F("<form method='POST' action='/reiniciar'>"
           "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
           "&#8635; Reiniciar ESP32</button></form>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Config",c));
}

// ═══════════════════════════════════════════════════════════
// [AUX-WEB] HANDLERS AUXILIARES
// ═══════════════════════════════════════════════════════════
void handleResetCurto() {
    if (!logado){ server.sendHeader("Location","/login"); server.send(302); return; }
    String r=server.arg("rede");
    if (r=="alta") resetCurtoAlta();
    else if (r=="baixa"||r=="inversor") resetCurtoBaixa();
    server.sendHeader("Location","/redes"); server.send(302);
}

void handleNTPSync() {
    if (!logado){ server.sendHeader("Location","/login"); server.send(302); return; }
    sincronizarNTP();
    server.sendHeader("Location","/config"); server.send(302);
}

void handleReiniciar() {
    if (!logado){ server.sendHeader("Location","/login"); server.send(302); return; }
    adicionarLog("Reinicio solicitado");
    String c = F("<div class='ok'>&#10003; Reiniciando em 3s...</div>"
                 "<div class='nf'>Aguarde e reconecte.</div>");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("Reiniciando",c,false));
    delay(3000); ESP.restart();
}

void handleOTAResult() {
    if (!logado){ server.sendHeader("Location","/login"); server.send(302); return; }
    String c = F("<h3>&#128640; Atualizacao OTA</h3>"
                 "<div class='nf'>Selecione o arquivo .bin gerado pelo Arduino IDE.<br>"
                 "O ESP32 reinicia automaticamente apos o upload.</div>"
                 "<div class='wn' style='font-size:11px'>"
                 "&#9888; Nao feche a pagina durante o upload!</div>"
                 "<form method='POST' action='/ota-upload' "
                 "enctype='multipart/form-data'>"
                 "<label>Firmware (.bin):</label>"
                 "<input type='file' name='firmware' accept='.bin' required "
                 "style='background:#111;border:1px solid #444;border-radius:4px;"
                 "padding:8px;color:#e0e0e0;width:100%;box-sizing:border-box;margin-top:4px'>"
                 "<button type='submit' class='btn-o' style='margin-top:10px'>"
                 "&#128640; Enviar e Atualizar</button></form>"
                 "<form method='GET' action='/config'>"
                 "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
                 "&#8592; Voltar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - OTA",c));
}

void handleOTAUpload() {
    HTTPUpload& up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
        adicionarLog("OTA: inicio");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
            adicionarLog("OTA ERRO: begin");
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize)
            adicionarLog("OTA ERRO: write");
    } else if (up.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            snprintf(tempBuf,sizeof(tempBuf),"OTA OK: %u bytes",up.totalSize);
            adicionarLog(tempBuf);
            String c = F("<div class='ok'>&#10003; Firmware atualizado! Reiniciando...</div>");
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("OTA OK",c,false));
            delay(3000); ESP.restart();
        } else {
            adicionarLog("OTA ERRO: end");
            String c = F("<div class='er'>&#10007; Erro OTA. Tente novamente.</div>"
                         "<form method='GET' action='/ota'>"
                         "<button type='submit' class='btn-s'>Voltar</button></form>");
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
    server.on("/previsao",    HTTP_ANY,  handlePrevisao);
    server.on("/logs",        HTTP_GET,  handleLogs);
    server.on("/login",       HTTP_ANY,  handleLogin);
    server.on("/logout",      HTTP_GET,  handleLogout);
    server.on("/setup-login", HTTP_ANY,  handleSetupLogin);
    server.on("/setup-wifi",  HTTP_ANY,  handleSetupWifi);
    server.on("/reset-curto", HTTP_POST, handleResetCurto);
    server.on("/ntp-sync",    HTTP_POST, handleNTPSync);
    server.on("/reiniciar",   HTTP_POST, handleReiniciar);
    server.on("/ota",         HTTP_GET,  handleOTAResult);
    server.on("/ota-upload",  HTTP_POST,
              [](){handleOTAResult();}, handleOTAUpload);
    server.onNotFound(handleNotFound);
    server.begin();
}
