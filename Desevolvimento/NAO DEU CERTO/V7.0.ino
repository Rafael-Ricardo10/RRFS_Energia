/*
 * RRFS Energia V7.0
 * Sistema de Gerenciamento de Energia Solar
 * Sensores: ACS758 50A (Painel) + ACS758 100A (Casa)
 * Bateria: LiFePO4 com estimativa por coulomb counting
 * Inversor: único inversor de alta potência
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <time.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>

// ============================================================
// PINAGEM - SENSORES
// ============================================================
#define LED_AZUL          2
#define BOTAO_BOOT        0
#define PIN_SENSOR_PAINEL 34   // ACS758 50A  - ADC1_CH6
#define PIN_SENSOR_CASA   35   // ACS758 100A - ADC1_CH7
#define PIN_BKP_BATERIA   32   // Sinal relé controlador de carga (3.3V)

// ============================================================
// PINAGEM - INVERSOR ÚNICO (Alta Potência)
// ============================================================
#define PIN_RELE_INV_ALTA    27   // Relé Inversor Alta (LOW=ligado, HIGH=desligado)
#define PIN_STATUS_INV_ALTA  36   // Sinal LED status Inversor Alta (VP)

// ============================================================
// PINAGEM - REDE ELÉTRICA
// ============================================================
#define PIN_RELE_REDE_BAIXA  26   // Relé Rede Baixa
                                   // HIGH = rede DESLIGADA
                                   // LOW  = rede LIGADA
#define PIN_RELE_REDE_ALTA   25   // Relé Rede Alta (equipamentos pesados)
                                   // HIGH = rede DESLIGADA
                                   // LOW  = rede LIGADA
#define PIN_RELE_LAMP_EXTERNA 14  // Relé Lâmpada Externa
                                   // HIGH = lâmpada DESLIGADA
                                   // LOW  = lâmpada LIGADA
#define PIN_RELE_LAMP_INTERNA 12  // Relé Lâmpada Interna
                                   // HIGH = lâmpada DESLIGADA
                                   // LOW  = lâmpada LIGADA

// ============================================================
// CONSTANTES DO SISTEMA
// ============================================================
#define MAX_LOGS            20
#define LOG_MAX_LEN         110
#define DNS_PORT            53
#define TEMPO_RESET_MS      10000
#define ADC_RESOLUCAO       4095.0f
#define ADC_VREF            3.3f
#define DIVISOR_TENSAO      (3.3f / 5.0f)
#define ZERO_CORRENTE_RAW   (ADC_RESOLUCAO * (2.5f / 5.0f) * DIVISOR_TENSAO / ADC_VREF)
#define SENSIBILIDADE_50A   (0.040f * DIVISOR_TENSAO)
#define SENSIBILIDADE_100A  (0.020f * DIVISOR_TENSAO)
#define AMOSTRAS_ADC        64
#define INTERVALO_LEITURA   1000UL
#define INTERVALO_HISTORICO 60000UL
#define INTERVALO_WIFI_CHK  30000UL
#define INTERVALO_RAM_LOG   600000UL
#define LOG_POR_RAM         10
#define MAX_HISTORICO       1440
#define VERSAO              "V7.0"

// ============================================================
// CONSTANTES - INVERSOR
// ============================================================
#define TEMPO_RESTART_INV_MS      3000UL  // aguarda antes de religar
#define TEMPO_DEBOUNCE_STATUS      200UL  // debounce sinal normal
#define TEMPO_TRANSITORIO_REDE    1500UL  // transitório normal ao ligar rede
#define TEMPO_AGUARDA_SINAL       2000UL  // aguarda sinal após ligar inversor
#define TEMPO_ESTABILIDADE_OK     3000UL  // tempo sinal estável = OK
#define TEMPO_DIAGNOSTICO_MAX    10000UL  // timeout diagnóstico

// ============================================================
// CONSTANTES - REDE ELÉTRICA
// ============================================================
#define TEMPO_DECISAO_REDE_MS   60000UL
#define HISTERESE_REDE_PERC     2.0f

// ============================================================
// CONSTANTES - PREVISÃO SOLAR
// ============================================================
#define INTERVALO_PREVISAO      3600000UL
#define EFICIENCIA_PADRAO       0.75f
#define FATOR_CAL_MIN           0.20f
#define FATOR_CAL_MAX           1.30f
#define PESO_HISTORICO          0.70f
#define PESO_NOVO               0.30f
#define MIN_RADIACAO_CALIBRAR   50.0f
#define MIN_CORRENTE_CALIBRAR   0.5f
#define HORAS_PREVISAO          48
#define HORA_AMANHECER          6
#define HORA_ANTECIPAR_CORTE    21
#define DELTA_FATOR_SALVAR      0.02f
#define HTTP_TIMEOUT_MS         15000
#define BUFFER_HTTP             6144

const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ============================================================
// ESTRUTURAS
// ============================================================
struct DadosSensores {
    float correntePainel;
    float correnteCasa;
    float correnteBateria;
    float potenciaPainel;
    float potenciaCasa;
    float potenciaBateria;
    float tensaoBateriaEst;
    bool  bateriaCarregando;
    bool  alertaPainelInvertido;
    bool  alertaCasaInvertida;
    bool  sinalBKP;
};

struct ConfigBateria {
    float tensaoNominal;
    float amperagem;
    float percDescargaSegura;
    float tensaoMinima;
    float capacidadeTotal;
    float energiaDisponivel;
    float tensaoMaxima;
};

struct EstadoBateria {
    float ahDisponiveis;
    float ahMaximos;
    float ciclos;
    float ciclosParciais;
    bool  carregada100;
    unsigned long tsCarregada;
};

struct RegistroHistorico {
    uint32_t timestamp;
    int16_t  whPainel;
    int16_t  whCasa;
    int16_t  whCargaBat;
    int16_t  whDescargaBat;
};

struct Contadores {
    double kwhPainelTotal;
    double kwhCasaTotal;
    double kwhCargaTotal;
    double kwhDescargaTotal;
};

// ============================================================
// ESTRUTURA - INVERSOR ÚNICO
// ============================================================
struct EstadoInversor {
    bool   invLigado;
    bool   statusInv;
    bool   emProtecao;
    bool   testando;
    bool   redeAltaBloqCurto;
    bool   redeBaixaBloqCurto;
    bool   aguardandoConfirmacaoRA;
    bool   aguardandoConfirmacaoRB;
    unsigned long tsProtecao;
    unsigned long tsStatusInv;
    unsigned long tsUltimoSinal;
    unsigned long tsLigouRede;
    uint32_t totalProtecoes;
    uint32_t totalReiniciamentos;
    uint32_t totalCurtosRA;
    uint32_t totalCurtosRB;
    uint8_t  faseTesteDiagnostico;
    // 0 = normal
    // 1 = aguardando apos religar inversor
    // 2 = testando sem rede alta
    // 3 = testando sem rede baixa
    // 4 = aguardando confirmacao usuario RA
    // 5 = aguardando confirmacao usuario RB
};

// ============================================================
// ESTRUTURA - CONFIGURAÇÃO REDE ELÉTRICA
// ============================================================
struct ConfigRede {
    // Percentuais de corte baseados em energiaDisponivel
    float percRedeBaixa;      // % mínimo rede baixa   (padrão 10%)
    float percRedeAlta;       // % mínimo rede alta    (padrão 30%)
    float percLampExt;        // % mínimo lamp. ext    (padrão 25%)
    float percLampInt;        // % mínimo lamp. int    (padrão  0%)
    bool  configurado;

    // Controle lâmpada externa por horário
    uint8_t modoLampExt;      // 0=apenas energia 1=horário 2=painel
    uint8_t ligarHora;
    uint8_t ligarMin;
    uint8_t desligarHora;
    uint8_t desligarMin;
};

struct EstadoRede {
    // Estados lógicos das cargas
    bool redeBaixaAtiva;
    bool redeAltaAtiva;
    bool lampExternaAtiva;
    bool lampInternaAtiva;

    // Timers de decisão (1 minuto)
    unsigned long tsAbaixoRedeBaixa;
    unsigned long tsAbaixoRedeAlta;
    unsigned long tsAbaixoLampExt;
    unsigned long tsAbaixoLampInt;

    // Flags de aguardo
    bool aguardandoRedeBaixa;
    bool aguardandoRedeAlta;
    bool aguardandoLampExt;
    bool aguardandoLampInt;

    // Controle horário lâmpada externa
    bool lampExtPorHorario;
    bool lampExtPorPainel;
    bool lampExtBloqEnergia;

    // Contadores de eventos
    uint32_t totalCortesRedeBaixa;
    uint32_t totalCortesRedeAlta;
    uint32_t totalCortesLampExt;
    uint32_t totalCortesLampInt;
    uint32_t totalLigadasHorario;
    uint32_t totalLigadasPainel;
};

// ============================================================
// ESTRUTURAS - PREVISÃO SOLAR
// ============================================================
struct ConfigPrevisao {
    double  latitude;
    double  longitude;
    float   kwpInstalado;
    bool    configurado;
};

struct EstadoPrevisao {
    float   radiacaoHora[48];
    float   nuvensPct[48];
    int8_t  chuvaProb[48];
    float   geracaoEstHora[48];
    float   fatorHorario[24];
    uint8_t amostrasHorario[24];
    float   totalRestanteHoje;
    float   totalAmanha;
    float   consumoMedioDia;
    float   taxaAcerto;
    bool    bloqRedeAlta;
    bool    bloqLampExt;
    char    motivoRedeAlta[90];
    char    motivoLampExt[90];
    bool    redeAltaFoiDesligada;
    bool    lampExtFoiDesligada;
    unsigned long tsUltimaAtualizacao;
    unsigned long tsUltimoSalvamento;
    float   fatorAntesSalvar;
    bool    dadosValidos;
    int8_t  horaOffsetAPI;
    uint8_t errosConsecutivos;
    float   geracaoRealAcumHora;
    float   geracaoEstAcumHora;
    uint8_t minutosNaHora;
};

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP  = false;
bool logado  = false;

// LED
unsigned long ultimoBlink = 0;
bool          ledState    = false;
bool          blinkAtivo  = true;

// Botão
unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

// Logs
char    logs[MAX_LOGS][LOG_MAX_LEN];
uint8_t totalLogs  = 0;
uint8_t logIndex   = 0;
uint8_t contLogs   = 0;
char    tempBuf[180];

// Dados do sistema
DadosSensores  sensores   = {0};
ConfigBateria  cfgBat     = {0};
EstadoBateria  estadoBat  = {0};
Contadores     contadores = {0};

// Histórico em RAM (circular)
RegistroHistorico historico[MAX_HISTORICO];
uint16_t histIdx   = 0;
uint16_t histTotal = 0;

// Acumuladores para histórico
float acumWhPainel    = 0;
float acumWhCasa      = 0;
float acumWhCargaBat  = 0;
float acumWhDescBat   = 0;

// Timers
unsigned long tLeitura   = 0;
unsigned long tHistorico = 0;
unsigned long tWifi      = 0;
unsigned long tRAMLog    = 0;

// NTP
bool ntpSincronizado = false;

// ADC offset calibrado
float adcOffsetPainel = 0;
float adcOffsetCasa   = 0;

// ============================================================
// VARIÁVEIS GLOBAIS - INVERSOR ÚNICO
// ============================================================
EstadoInversor estadoInv = {0};

// ============================================================
// VARIÁVEIS GLOBAIS - REDE ELÉTRICA
// ============================================================
ConfigRede cfgRede   = {0};
EstadoRede estadoRede = {0};

// ============================================================
// VARIÁVEIS GLOBAIS - PREVISÃO SOLAR
// ============================================================
ConfigPrevisao cfgPrev   = {0};
EstadoPrevisao estadoPrev = {0};
unsigned long  tPrevisao  = 0;

// ============================================================
// PROTÓTIPOS
// ============================================================
void notificarCurtoRA();
void notificarCurtoRB();
void confirmarConsertoRedeAlta();
void confirmarConsertoRedeBaixa();
void setRedeBaixa(bool ligada);
void setRedeAlta(bool ligada);

// ============================================================
// LOGS
// ============================================================
void adicionarLog(const char* msg) {
    time_t now = time(nullptr);
    struct tm* ti = localtime(&now);
    if (ntpSincronizado && ti->tm_year > 70) {
        snprintf(logs[logIndex], LOG_MAX_LEN,
                 "[%02d:%02d:%02d] %s",
                 ti->tm_hour, ti->tm_min, ti->tm_sec, msg);
    } else {
        unsigned long s = millis() / 1000;
        snprintf(logs[logIndex], LOG_MAX_LEN,
                 "[%02lu:%02lu:%02lu] %s",
                 s / 3600, (s % 3600) / 60, s % 60, msg);
    }
    logIndex = (logIndex + 1) % MAX_LOGS;
    if (totalLogs < MAX_LOGS) totalLogs++;
    contLogs++;
}

void logRAMSeNecessario() {
    if (contLogs >= LOG_POR_RAM) {
        contLogs = 0;
        uint32_t ram = (uint32_t)esp_get_free_heap_size();
        snprintf(tempBuf, sizeof(tempBuf), "RAM livre: %u KB", ram / 1024);
        adicionarLog(tempBuf);
    }
}

// ============================================================
// LED
// ============================================================
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

void ledBlink()      { blinkAtivo = true;  ultimoBlink = millis(); }
void ledFixo(bool v) { blinkAtivo = false; digitalWrite(LED_AZUL, v); }

// ============================================================
// BOTÃO BOOT
// ============================================================
void setupBotao() { pinMode(BOTAO_BOOT, INPUT_PULLUP); }

void verificarReset() {
    bool pressionado = (digitalRead(BOTAO_BOOT) == LOW);
    if (pressionado && !botaoSegurando) {
        botaoSegurando     = true;
        botaoPressionadoEm = millis();
    }
    if (!pressionado) botaoSegurando = false;

    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET por botao BOOT");
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(100);
            digitalWrite(LED_AZUL, LOW);  delay(100);
        }
        prefs.begin("rrfs", false); prefs.clear(); prefs.end();
        delay(500);
        ESP.restart();
    }
}

// ============================================================
// NTP / HORA
// ============================================================
void iniciarNTP() {
    prefs.begin("rrfs", true);
    String tz = prefs.getString("timezone", "BRT3");
    prefs.end();
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    setenv("TZ", tz.c_str(), 1);
    tzset();

    unsigned long t = millis();
    while (!time(nullptr) > 100000 && millis() - t < 5000) delay(100);

    time_t now = time(nullptr);
    if (now > 100000) {
        ntpSincronizado = true;
        struct tm* ti = localtime(&now);
        snprintf(tempBuf, sizeof(tempBuf),
                 "NTP OK: %02d/%02d/%04d %02d:%02d",
                 ti->tm_mday, ti->tm_mon+1, ti->tm_year+1900,
                 ti->tm_hour, ti->tm_min);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("NTP: sem sincronismo");
    }
}

String getDataHora() {
    time_t now = time(nullptr);
    struct tm* ti = localtime(&now);
    char buf[30];
    if (ntpSincronizado && ti->tm_year > 70) {
        snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d:%02d",
                 ti->tm_mday, ti->tm_mon+1, ti->tm_year+1900,
                 ti->tm_hour, ti->tm_min, ti->tm_sec);
    } else {
        unsigned long s = millis() / 1000;
        snprintf(buf, sizeof(buf), "UP %02lu:%02lu:%02lu",
                 s/3600, (s%3600)/60, s%60);
    }
    return String(buf);
}

// ============================================================
// PREFERENCES
// ============================================================
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

bool existeBateriaConfig() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("bat_ok");
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
                const String& mask, const String& dns,
                const String& tz,   bool ap) {
    prefs.begin("rrfs", false);
    prefs.putBool  ("usar_ap",   ap);
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.putString("wifi_ip",   ip);
    prefs.putString("wifi_gw",   gw);
    prefs.putString("wifi_mask", mask);
    prefs.putString("wifi_dns",  dns);
    prefs.putString("timezone",  tz);
    prefs.putBool  ("wifi_ok",   true);
    prefs.end();
}

void carregarWifi(String& ssid, String& pass, String& ip, String& gw,
                  String& mask, String& dns, String& tz, bool& ap) {
    prefs.begin("rrfs", true);
    ap   = prefs.getBool  ("usar_ap",   true);
    ssid = prefs.getString("wifi_ssid", "");
    pass = prefs.getString("wifi_pass", "");
    ip   = prefs.getString("wifi_ip",   "");
    gw   = prefs.getString("wifi_gw",   "");
    mask = prefs.getString("wifi_mask", "255.255.255.0");
    dns  = prefs.getString("wifi_dns",  "8.8.8.8");
    tz   = prefs.getString("timezone",  "BRT3");
    prefs.end();
}

void salvarConfigBateria() {
    prefs.begin("rrfs", false);
    prefs.putFloat("bat_vnom",  cfgBat.tensaoNominal);
    prefs.putFloat("bat_ah",    cfgBat.amperagem);
    prefs.putFloat("bat_pdesc", cfgBat.percDescargaSegura);
    prefs.putFloat("bat_vmin",  cfgBat.tensaoMinima);
    prefs.putBool ("bat_ok",    true);
    prefs.end();
}

void carregarConfigBateria() {
    prefs.begin("rrfs", true);
    cfgBat.tensaoNominal      = prefs.getFloat("bat_vnom",  12.8f);
    cfgBat.amperagem          = prefs.getFloat("bat_ah",    100.0f);
    cfgBat.percDescargaSegura = prefs.getFloat("bat_pdesc", 0.80f);
    cfgBat.tensaoMinima       = prefs.getFloat("bat_vmin",  12.0f);
    prefs.end();
    cfgBat.capacidadeTotal   = cfgBat.tensaoNominal * cfgBat.amperagem;
    cfgBat.energiaDisponivel = cfgBat.capacidadeTotal * cfgBat.percDescargaSegura;
    if      (cfgBat.tensaoNominal <= 13.0f) cfgBat.tensaoMaxima = 14.0f;
    else if (cfgBat.tensaoNominal <= 26.0f) cfgBat.tensaoMaxima = 28.0f;
    else                                     cfgBat.tensaoMaxima = cfgBat.tensaoNominal * 1.094f;
}

void salvarEstadoBateria() {
    prefs.begin("rrfsb", false);
    prefs.putFloat("ah_disp",  estadoBat.ahDisponiveis);
    prefs.putFloat("ah_max",   estadoBat.ahMaximos);
    prefs.putFloat("ciclos",   estadoBat.ciclos);
    prefs.putFloat("ciclos_p", estadoBat.ciclosParciais);
    prefs.putBool ("cheia",    estadoBat.carregada100);
    prefs.end();
}

void carregarEstadoBateria() {
    prefs.begin("rrfsb", true);
    estadoBat.ahDisponiveis  = prefs.getFloat("ah_disp",  -1.0f);
    estadoBat.ahMaximos      = prefs.getFloat("ah_max",    0.0f);
    estadoBat.ciclos         = prefs.getFloat("ciclos",    0.0f);
    estadoBat.ciclosParciais = prefs.getFloat("ciclos_p",  0.0f);
    estadoBat.carregada100   = prefs.getBool ("cheia",     false);
    prefs.end();
    if (estadoBat.ahDisponiveis < 0) {
        estadoBat.ahDisponiveis = cfgBat.amperagem * cfgBat.percDescargaSegura;
        estadoBat.carregada100  = true;
    }
    estadoBat.ahMaximos = cfgBat.amperagem * cfgBat.percDescargaSegura;
}

void salvarContadores() {
    prefs.begin("rrfsc", false);
    prefs.putDouble("kwh_pai", contadores.kwhPainelTotal);
    prefs.putDouble("kwh_cas", contadores.kwhCasaTotal);
    prefs.putDouble("kwh_car", contadores.kwhCargaTotal);
    prefs.putDouble("kwh_des", contadores.kwhDescargaTotal);
    prefs.end();
}

void carregarContadores() {
    prefs.begin("rrfsc", true);
    contadores.kwhPainelTotal   = prefs.getDouble("kwh_pai", 0.0);
    contadores.kwhCasaTotal     = prefs.getDouble("kwh_cas", 0.0);
    contadores.kwhCargaTotal    = prefs.getDouble("kwh_car", 0.0);
    contadores.kwhDescargaTotal = prefs.getDouble("kwh_des", 0.0);
    prefs.end();
}

// ============================================================
// PREFERENCES - INVERSOR ÚNICO
// ============================================================
void salvarEstadoInversor() {
    prefs.begin("rrfsi", false);
    prefs.putUInt ("total_prot", estadoInv.totalProtecoes);
    prefs.putUInt ("total_rein", estadoInv.totalReiniciamentos);
    prefs.putUInt ("total_cra",  estadoInv.totalCurtosRA);
    prefs.putUInt ("total_crb",  estadoInv.totalCurtosRB);
    prefs.putBool ("bloq_ra",    estadoInv.redeAltaBloqCurto);
    prefs.putBool ("bloq_rb",    estadoInv.redeBaixaBloqCurto);
    prefs.putBool ("conf_ra",    estadoInv.aguardandoConfirmacaoRA);
    prefs.putBool ("conf_rb",    estadoInv.aguardandoConfirmacaoRB);
    prefs.putUChar("fase_diag",  estadoInv.faseTesteDiagnostico);
    prefs.end();
}

void carregarEstadoInversor() {
    prefs.begin("rrfsi", true);
    estadoInv.totalProtecoes          = prefs.getUInt ("total_prot", 0);
    estadoInv.totalReiniciamentos     = prefs.getUInt ("total_rein", 0);
    estadoInv.totalCurtosRA           = prefs.getUInt ("total_cra",  0);
    estadoInv.totalCurtosRB           = prefs.getUInt ("total_crb",  0);
    estadoInv.redeAltaBloqCurto       = prefs.getBool ("bloq_ra",    false);
    estadoInv.redeBaixaBloqCurto      = prefs.getBool ("bloq_rb",    false);
    estadoInv.aguardandoConfirmacaoRA = prefs.getBool ("conf_ra",    false);
    estadoInv.aguardandoConfirmacaoRB = prefs.getBool ("conf_rb",    false);
    estadoInv.faseTesteDiagnostico    = prefs.getUChar("fase_diag",  0);
    prefs.end();

    // Se reiniciou durante diagnóstico ativo → volta ao normal
    if (estadoInv.faseTesteDiagnostico > 0 &&
        estadoInv.faseTesteDiagnostico < 4) {
        adicionarLog("INV: Reinicio durante diagnostico - resetando fase");
    estadoInv.faseTesteDiagnostico = 0;
    estadoInv.emProtecao           = false;
        }

        // Mantém bloqueios de curto após reinício
        if (estadoInv.redeAltaBloqCurto) {
            adicionarLog("INV: Rede Alta ainda bloqueada (curto anterior)");
            adicionarLog("Acesse /rede para liberar apos consertar!");
            if (estadoInv.aguardandoConfirmacaoRA)
                estadoInv.faseTesteDiagnostico = 4;
        }
        if (estadoInv.redeBaixaBloqCurto) {
            adicionarLog("INV: Rede Baixa ainda bloqueada (curto anterior)");
            adicionarLog("Acesse /rede para liberar apos consertar!");
            if (estadoInv.aguardandoConfirmacaoRB)
                estadoInv.faseTesteDiagnostico = 5;
        }
}

// ============================================================
// PREFERENCES - REDE ELÉTRICA
// ============================================================
void salvarConfigRede() {
    prefs.begin("rrfsr", false);
    prefs.putFloat("r_redbaixa", cfgRede.percRedeBaixa);
    prefs.putFloat("r_redealta", cfgRede.percRedeAlta);
    prefs.putFloat("r_lampext",  cfgRede.percLampExt);
    prefs.putFloat("r_lampint",  cfgRede.percLampInt);
    prefs.putUChar("r_modolext", cfgRede.modoLampExt);
    prefs.putUChar("r_lighora",  cfgRede.ligarHora);
    prefs.putUChar("r_ligmin",   cfgRede.ligarMin);
    prefs.putUChar("r_dslhora",  cfgRede.desligarHora);
    prefs.putUChar("r_dslmin",   cfgRede.desligarMin);
    prefs.putBool ("r_ok",       true);
    prefs.end();
}

void carregarConfigRede() {
    prefs.begin("rrfsr", true);
    cfgRede.percRedeBaixa = prefs.getFloat("r_redbaixa", 10.0f);
    cfgRede.percRedeAlta  = prefs.getFloat("r_redealta", 30.0f);
    cfgRede.percLampExt   = prefs.getFloat("r_lampext",  25.0f);
    cfgRede.percLampInt   = prefs.getFloat("r_lampint",   0.0f);
    cfgRede.modoLampExt   = prefs.getUChar("r_modolext",    0);
    cfgRede.ligarHora     = prefs.getUChar("r_lighora",    18);
    cfgRede.ligarMin      = prefs.getUChar("r_ligmin",      0);
    cfgRede.desligarHora  = prefs.getUChar("r_dslhora",     6);
    cfgRede.desligarMin   = prefs.getUChar("r_dslmin",      0);
    cfgRede.configurado   = prefs.getBool ("r_ok",      false);
    prefs.end();
}

bool existeConfigRede() {
    prefs.begin("rrfsr", true);
    bool ok = prefs.isKey("r_ok");
    prefs.end();
    return ok;
}

void salvarEstadoRede() {
    prefs.begin("rrfsrs", false);
    prefs.putUInt("r_credb",  estadoRede.totalCortesRedeBaixa);
    prefs.putUInt("r_creda",  estadoRede.totalCortesRedeAlta);
    prefs.putUInt("r_clext",  estadoRede.totalCortesLampExt);
    prefs.putUInt("r_clint",  estadoRede.totalCortesLampInt);
    prefs.putUInt("r_clighr", estadoRede.totalLigadasHorario);
    prefs.putUInt("r_cpain",  estadoRede.totalLigadasPainel);
    prefs.end();
}

void carregarEstadoRede() {
    prefs.begin("rrfsrs", true);
    estadoRede.totalCortesRedeBaixa = prefs.getUInt("r_credb",  0);
    estadoRede.totalCortesRedeAlta  = prefs.getUInt("r_creda",  0);
    estadoRede.totalCortesLampExt   = prefs.getUInt("r_clext",  0);
    estadoRede.totalCortesLampInt   = prefs.getUInt("r_clint",  0);
    estadoRede.totalLigadasHorario  = prefs.getUInt("r_clighr", 0);
    estadoRede.totalLigadasPainel   = prefs.getUInt("r_cpain",  0);
    prefs.end();
}

// ============================================================
// PREFERENCES - PREVISÃO SOLAR
// ============================================================
void salvarConfigPrevisao() {
    prefs.begin("rrfsp", false);
    prefs.putString("lat",  String(cfgPrev.latitude,  15));
    prefs.putString("lon",  String(cfgPrev.longitude, 15));
    prefs.putFloat ("kwp",  cfgPrev.kwpInstalado);
    prefs.putBool  ("p_ok", true);
    prefs.end();
}

void carregarConfigPrevisao() {
    prefs.begin("rrfsp", true);
    cfgPrev.latitude     = prefs.getString("lat", "0").toDouble();
    cfgPrev.longitude    = prefs.getString("lon", "0").toDouble();
    cfgPrev.kwpInstalado = prefs.getFloat ("kwp",  0.0f);
    cfgPrev.configurado  = prefs.getBool  ("p_ok", false);
    prefs.end();
}

bool existeConfigPrevisao() {
    prefs.begin("rrfsp", true);
    bool ok = prefs.isKey("p_ok");
    prefs.end();
    return ok;
}

void salvarCalibracaoHoraria(bool forcado) {
    unsigned long agora = millis();
    float fatorAtual = estadoPrev.fatorHorario[0];

    bool mudouSignificativo =
        fabsf(fatorAtual - estadoPrev.fatorAntesSalvar) > DELTA_FATOR_SALVAR;
    bool tempoExpirou =
        (agora - estadoPrev.tsUltimoSalvamento) >= INTERVALO_PREVISAO;

    if (!forcado && !mudouSignificativo && !tempoExpirou) return;

    prefs.begin("rrfspc", false);
    for (int h = 0; h < 24; h++) {
        char chF[8], chA[8];
        snprintf(chF, sizeof(chF), "f%d", h);
        snprintf(chA, sizeof(chA), "a%d", h);
        prefs.putFloat(chF, estadoPrev.fatorHorario[h]);
        prefs.putUChar(chA, estadoPrev.amostrasHorario[h]);
    }
    prefs.putFloat("acerto",  estadoPrev.taxaAcerto);
    prefs.putFloat("consmed", estadoPrev.consumoMedioDia);
    prefs.end();

    estadoPrev.fatorAntesSalvar   = fatorAtual;
    estadoPrev.tsUltimoSalvamento = agora;
}

void carregarCalibracaoHoraria() {
    prefs.begin("rrfspc", true);
    for (int h = 0; h < 24; h++) {
        char chF[8], chA[8];
        snprintf(chF, sizeof(chF), "f%d", h);
        snprintf(chA, sizeof(chA), "a%d", h);
        estadoPrev.fatorHorario[h]    = prefs.getFloat(chF, 1.0f);
        estadoPrev.amostrasHorario[h] = prefs.getUChar(chA, 0);
    }
    estadoPrev.taxaAcerto      = prefs.getFloat("acerto",  0.0f);
    estadoPrev.consumoMedioDia = prefs.getFloat("consmed", 0.0f);
    prefs.end();
    estadoPrev.fatorAntesSalvar = estadoPrev.fatorHorario[0];
    adicionarLog("Calibracao solar carregada da flash");
}

// ============================================================
// CONSUMO MÉDIO DIÁRIO
// ============================================================
void calcularConsumoMedio() {
    if (histTotal < 60) return;

    float    whDia[7]  = {0};
    uint16_t ctDia[7]  = {0};

    for (int i = 0; i < histTotal && i < MAX_HISTORICO; i++) {
        int idx = ((int)histIdx - 1 - i + MAX_HISTORICO) % MAX_HISTORICO;
        uint32_t ts = historico[idx].timestamp;
        if (ts == 0) continue;
        time_t agora = time(nullptr);
        int diasAtras = (int)((agora - ts) / 86400);
        if (diasAtras < 0 || diasAtras >= 7) continue;
        whDia[diasAtras]  += historico[idx].whCasa / 10.0f;
        ctDia[diasAtras]++;
    }

    float somaWh = 0, somaPeso = 0;
    for (int d = 0; d < 7; d++) {
        if (ctDia[d] < 10) continue;
        float pesoD = (float)(7 - d);
        somaWh   += whDia[d] * pesoD;
        somaPeso += pesoD;
    }

    if (somaPeso > 0) {
        estadoPrev.consumoMedioDia = somaWh / somaPeso;
        snprintf(tempBuf, sizeof(tempBuf),
                 "Consumo medio dia: %.0f Wh", estadoPrev.consumoMedioDia);
        adicionarLog(tempBuf);
    }
}

// ============================================================
// PARSE MANUAL DO JSON OPEN-METEO
// ============================================================
int parseArrayFloat(const String& json,
                    const char* campo,
                    float* dest,
                    int maxValores) {
    String busca = "\"";
    busca += campo;
    busca += "\":[";
    int pos = json.indexOf(busca);
    if (pos < 0) return 0;
    pos += busca.length();

    int count = 0;
    while (count < maxValores) {
        int fim    = json.indexOf(',', pos);
        int fimArr = json.indexOf(']', pos);
        if (fimArr >= 0 && (fim < 0 || fimArr < fim)) fim = fimArr;
        if (fim < 0) break;
        String token = json.substring(pos, fim);
        token.trim();
        dest[count++] = token.toFloat();
        if (fim == fimArr) break;
        pos = fim + 1;
    }
    return count;
}

// ============================================================
// BUSCA DADOS OPEN-METEO
// ============================================================
bool buscarDadosOpenMeteo() {
    if (modoAP || WiFi.status() != WL_CONNECTED) return false;
    if (!cfgPrev.configurado) return false;

    char url[320];
    snprintf(url, sizeof(url),
        "http://api.open-meteo.com/v1/forecast?"
        "latitude=%.15f&longitude=%.15f"
        "&hourly=shortwave_radiation,cloudcover,"
        "precipitation_probability"
        "&forecast_days=2&timezone=auto",
        cfgPrev.latitude, cfgPrev.longitude);

    adicionarLog("Prev: buscando API...");

    WiFiClient  client;
    HTTPClient  http;
    http.begin(client, url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.addHeader("Accept", "application/json");

    int httpCode = http.GET();
    if (httpCode != 200) {
        snprintf(tempBuf, sizeof(tempBuf), "Prev: erro HTTP %d", httpCode);
        adicionarLog(tempBuf);
        http.end();
        estadoPrev.errosConsecutivos++;
        return false;
    }

    String payload = http.getString();
    http.end();

    if (payload.length() < 100) {
        adicionarLog("Prev: resposta invalida");
        estadoPrev.errosConsecutivos++;
        return false;
    }

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    estadoPrev.horaOffsetAPI = ti->tm_hour;

    float radTemp[48]    = {0};
    float nuvensTemp[48] = {0};
    float chuvaTemp[48]  = {0};

    int nRad = parseArrayFloat(payload, "shortwave_radiation", radTemp,    48);
              parseArrayFloat(payload, "cloudcover",          nuvensTemp, 48);
              parseArrayFloat(payload, "precipitation_probability",
                              chuvaTemp, 48);

    if (nRad < 24) {
        adicionarLog("Prev: parse falhou");
        estadoPrev.errosConsecutivos++;
        return false;
    }

    for (int i = 0; i < 48; i++) {
        estadoPrev.radiacaoHora[i] = radTemp[i];
        estadoPrev.nuvensPct[i]    = nuvensTemp[i];
        estadoPrev.chuvaProb[i]    = (int8_t)chuvaTemp[i];
    }

    estadoPrev.dadosValidos        = true;
    estadoPrev.errosConsecutivos   = 0;
    estadoPrev.tsUltimaAtualizacao = millis();

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev: API OK! Rad=%.0fW/m2 Nuv=%.0f%%",
             estadoPrev.radiacaoHora[estadoPrev.horaOffsetAPI],
             estadoPrev.nuvensPct[estadoPrev.horaOffsetAPI]);
    adicionarLog(tempBuf);
    return true;
}

// ============================================================
// CALCULA GERAÇÃO ESTIMADA POR HORA
// ============================================================
void calcularGeracaoEstimada() {
    if (!estadoPrev.dadosValidos)  return;
    if (cfgPrev.kwpInstalado <= 0) return;

    int   horaAtual  = estadoPrev.horaOffsetAPI;
    float totalRest  = 0;
    float totalAmanha = 0;

    for (int i = 0; i < 48; i++) {
        float rad  = estadoPrev.radiacaoHora[i];
        int   hDia = (horaAtual + i) % 24;

        float bruto     = rad * cfgPrev.kwpInstalado * EFICIENCIA_PADRAO;
        float calibrado = bruto * estadoPrev.fatorHorario[hDia];
        float maxPoss   = cfgPrev.kwpInstalado * 1000.0f * 1.1f;
        calibrado = constrain(calibrado, 0.0f, maxPoss);

        estadoPrev.geracaoEstHora[i] = calibrado;

        if (i < (24 - horaAtual))
            totalRest += calibrado;
        else if (i < (24 - horaAtual + 24))
            totalAmanha += calibrado;
    }

    estadoPrev.totalRestanteHoje = totalRest;
    estadoPrev.totalAmanha       = totalAmanha;
}

// ============================================================
// CALIBRAÇÃO DO FATOR HORÁRIO
// Reutiliza: sensores.correntePainel, sensores.tensaoBateriaEst
// ============================================================
void calibrarFatorHorario() {
    if (!estadoPrev.dadosValidos)  return;
    if (!ntpSincronizado)          return;
    if (cfgPrev.kwpInstalado <= 0) return;

    // Reutiliza valores já calculados em lerSensores()
    float corrPainel = sensores.correntePainel;
    float tensao     = sensores.tensaoBateriaEst;

    if (corrPainel < MIN_CORRENTE_CALIBRAR) {
        estadoPrev.geracaoRealAcumHora = 0;
        estadoPrev.geracaoEstAcumHora  = 0;
        estadoPrev.minutosNaHora       = 0;
        return;
    }

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int horaAtual = ti->tm_hour;
    int idxAPI    = horaAtual - estadoPrev.horaOffsetAPI;
    if (idxAPI < 0)  idxAPI += 24;
    if (idxAPI >= 48) return;

    float radAtual = estadoPrev.radiacaoHora[idxAPI];
    if (radAtual < MIN_RADIACAO_CALIBRAR) return;

    float whRealMinuto = (corrPainel * tensao) / 60.0f;
    float whEstMinuto  = estadoPrev.geracaoEstHora[idxAPI] / 60.0f;

    estadoPrev.geracaoRealAcumHora += whRealMinuto;
    estadoPrev.geracaoEstAcumHora  += whEstMinuto;
    estadoPrev.minutosNaHora++;

    if (estadoPrev.minutosNaHora < 10)        return;
    if (estadoPrev.geracaoEstAcumHora < 1.0f) return;

    float fatorMedido = estadoPrev.geracaoRealAcumHora /
                        estadoPrev.geracaoEstAcumHora;
    fatorMedido = constrain(fatorMedido, FATOR_CAL_MIN, FATOR_CAL_MAX);

    uint8_t nAmostras = estadoPrev.amostrasHorario[horaAtual];
    float   pesoHist  = (nAmostras > 10) ? 0.85f : PESO_HISTORICO;
    float   pesoNovo  = 1.0f - pesoHist;

    float fatorNovo = (estadoPrev.fatorHorario[horaAtual] * pesoHist)
                    + (fatorMedido * pesoNovo);
    fatorNovo = constrain(fatorNovo, FATOR_CAL_MIN, FATOR_CAL_MAX);
    estadoPrev.fatorHorario[horaAtual] = fatorNovo;

    if (nAmostras < 255) estadoPrev.amostrasHorario[horaAtual]++;

    float erroPerc  = fabsf((fatorMedido - 1.0f) * 100.0f);
    float acertoEst = constrain(100.0f - erroPerc, 0.0f, 100.0f);
    estadoPrev.taxaAcerto = (estadoPrev.taxaAcerto < 1.0f)
                          ? acertoEst
                          : estadoPrev.taxaAcerto * 0.9f + acertoEst * 0.1f;

    calcularGeracaoEstimada();
    salvarCalibracaoHoraria(false);

    estadoPrev.geracaoRealAcumHora = 0;
    estadoPrev.geracaoEstAcumHora  = 0;
    estadoPrev.minutosNaHora       = 0;
}

// ============================================================
// DECISÕES AUTOMÁTICAS COM BASE NA PREVISÃO
// Reutiliza: estadoBat.ahDisponiveis, sensores.tensaoBateriaEst
//            cfgBat.energiaDisponivel, cfgRede.percRedeAlta
// ============================================================
void tomarDecisoesPrev() {
    if (!estadoPrev.dadosValidos) return;
    if (!existeConfigRede())       return;
    if (!existeBateriaConfig())    return;
    if (!ntpSincronizado)          return;

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int hora = ti->tm_hour;

    // Reutiliza SoC já calculado
    float soc = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    float consMedia     = (estadoPrev.consumoMedioDia < 100.0f)
                        ? 3000.0f : estadoPrev.consumoMedioDia;
    float geracaoAmanha = estadoPrev.totalAmanha;

    // Reutiliza whDisp (mesma equação de controlarRede)
    float whDisp    = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float whMinRAOn = cfgBat.energiaDisponivel *
                      ((cfgRede.percRedeAlta + HISTERESE_REDE_PERC) / 100.0f);

    // ── DECISÃO 1: LÂMPADA EXTERNA ──
    bool noite = (hora >= 22 || hora < HORA_AMANHECER);
    if (noite && !estadoPrev.bloqLampExt) {
        if (geracaoAmanha < consMedia * 0.60f && soc < 35.0f) {
            estadoPrev.bloqLampExt         = true;
            estadoPrev.lampExtFoiDesligada = true;
            snprintf(estadoPrev.motivoLampExt, sizeof(estadoPrev.motivoLampExt),
                     "Prev=%.0fWh<%.0fWh Bat=%.0f%%",
                     geracaoAmanha, consMedia * 0.60f, soc);
            adicionarLog("PREV: Lamp.Ext bloqueada por previsao ruim");
        }
    }
    if (hora >= HORA_AMANHECER && hora < 10 && estadoPrev.lampExtFoiDesligada) {
        estadoPrev.bloqLampExt         = false;
        estadoPrev.lampExtFoiDesligada = false;
        estadoPrev.motivoLampExt[0]    = 0;
        adicionarLog("PREV: Lamp.Ext desbloqueada (amanheceu)");
    }

    // ── DECISÃO 2: REDE ALTA APÓS MEIA-NOITE ──
    bool aposMeiaNoite = (hora >= 0 && hora < HORA_AMANHECER);
    if (aposMeiaNoite && !estadoPrev.bloqRedeAlta) {
        if (geracaoAmanha < consMedia * 0.50f || soc < 30.0f) {
            estadoPrev.bloqRedeAlta         = true;
            estadoPrev.redeAltaFoiDesligada = true;
            snprintf(estadoPrev.motivoRedeAlta, sizeof(estadoPrev.motivoRedeAlta),
                     "Noite: Prev=%.0fWh Bat=%.0f%%",
                     geracaoAmanha, soc);
            adicionarLog("PREV: Rede Alta bloqueada apos meia-noite");
        }
    }

    // ── DECISÃO 3: REDE ALTA ANTECIPADA (CRÍTICO) ──
    if (hora >= HORA_ANTECIPAR_CORTE && !estadoPrev.bloqRedeAlta) {
        if (geracaoAmanha < consMedia * 0.35f && soc < 25.0f) {
            estadoPrev.bloqRedeAlta         = true;
            estadoPrev.redeAltaFoiDesligada = true;
            snprintf(estadoPrev.motivoRedeAlta, sizeof(estadoPrev.motivoRedeAlta),
                     "CRITICO: Prev=%.0fWh Bat=%.0f%%",
                     geracaoAmanha, soc);
            adicionarLog("PREV: Rede Alta bloqueada antecipado (critico)");
        }
    }

    // ── DECISÃO 4: RELIGAR REDE ALTA AO AMANHECER ──
    if (hora >= HORA_AMANHECER && hora < 12 &&
        estadoPrev.redeAltaFoiDesligada && whDisp > whMinRAOn) {
        estadoPrev.bloqRedeAlta         = false;
        estadoPrev.redeAltaFoiDesligada = false;
        estadoPrev.motivoRedeAlta[0]    = 0;
        snprintf(tempBuf, sizeof(tempBuf),
                 "PREV: Rede Alta desbloqueada (%.0fWh>%.0fWh)",
                 whDisp, whMinRAOn);
        adicionarLog(tempBuf);
    }
}

// ============================================================
// FUNÇÃO: setInversor
// Relé LOW  = Inversor LIGADO
// Relé HIGH = Inversor DESLIGADO
// ============================================================
void setInversor(bool ligado) {
    estadoInv.invLigado = ligado;
    digitalWrite(PIN_RELE_INV_ALTA, ligado ? LOW : HIGH);
}

bool lerStatusInversor() {
    return (digitalRead(PIN_STATUS_INV_ALTA) == HIGH);
}

// ============================================================
// FUNÇÃO: notificarCurtoRA
// ============================================================
void notificarCurtoRA() {
    estadoInv.redeAltaBloqCurto       = true;
    estadoInv.aguardandoConfirmacaoRA  = true;
    estadoInv.totalCurtosRA++;

    snprintf(tempBuf, sizeof(tempBuf),
             "CURTO/FALHA Rede Alta! Total: %lu",
             (unsigned long)estadoInv.totalCurtosRA);
    adicionarLog(tempBuf);
    adicionarLog(">>> REDE ALTA BLOQUEADA ate confirmacao manual <<<");
    adicionarLog("Acesse /rede para liberar apos consertar!");

    salvarEstadoInversor();
}

// ============================================================
// FUNÇÃO: notificarCurtoRB
// Avisa usuário sobre problema na rede baixa
// ============================================================
void notificarCurtoRB() {
    estadoInv.redeBaixaBloqCurto       = true;
    estadoInv.aguardandoConfirmacaoRB  = true;
    estadoInv.totalCurtosRB++;

    snprintf(tempBuf, sizeof(tempBuf),
             "CURTO/FALHA Rede Baixa! Total: %lu",
             (unsigned long)estadoInv.totalCurtosRB);
    adicionarLog(tempBuf);
    adicionarLog(">>> REDE BAIXA BLOQUEADA ate confirmacao manual <<<");
    adicionarLog("Acesse /rede para liberar apos consertar!");

    salvarEstadoInversor();
}

void controlarInversor() {
    unsigned long agora = millis();

    // ── Se inversor desligado: ignora status ──
    if (!estadoInv.invLigado) {
        estadoInv.statusInv     = false;
        estadoInv.tsUltimoSinal = 0;
        return;
    }

    // ── Lê sinal atual ──
    bool sinalAtual = lerStatusInversor();

    // ── Atualiza timestamp do último sinal ──
    if (sinalAtual) {
        estadoInv.tsUltimoSinal = agora;
        estadoInv.statusInv     = true;
    } else {
        estadoInv.statusInv = false;
    }

    // ══════════════════════════════════════════════════════
    // MÁQUINA DE ESTADOS DO DIAGNÓSTICO
    // ══════════════════════════════════════════════════════
    switch (estadoInv.faseTesteDiagnostico) {

        // ──────────────────────────────────────────────────
        // FASE 0: OPERAÇÃO NORMAL
        // ──────────────────────────────────────────────────
        case 0: {
            bool emTransitorio = (estadoInv.tsLigouRede > 0 &&
            (agora - estadoInv.tsLigouRede) < TEMPO_TRANSITORIO_REDE);

            if (!sinalAtual &&
                !emTransitorio &&
                estadoInv.tsUltimoSinal > 0 &&
                (agora - estadoInv.tsUltimoSinal) >= TEMPO_AGUARDA_SINAL) {

                adicionarLog("INV: Sinal perdido! Reiniciando inversor...");
            estadoInv.totalProtecoes++;

            setInversor(false);
            delay(3000);
            setInversor(true);

            estadoInv.tsProtecao           = agora;
            estadoInv.tsUltimoSinal        = agora;
            estadoInv.faseTesteDiagnostico = 1;
            estadoInv.emProtecao           = true;

            snprintf(tempBuf, sizeof(tempBuf),
                     "INV: Reiniciado (total: %lu)",
                     (unsigned long)estadoInv.totalProtecoes);
            adicionarLog(tempBuf);
            salvarEstadoInversor();
                }
                break;
        }

        // ──────────────────────────────────────────────────
        // FASE 1: AGUARDA SINAL APÓS RELIGAR INVERSOR
        // ──────────────────────────────────────────────────
        case 1: {
            if (sinalAtual) {
                unsigned long tempoEstavel = agora - estadoInv.tsUltimoSinal;

                if (tempoEstavel >= TEMPO_ESTABILIDADE_OK) {
                    adicionarLog("INV: Sinal estavel - OK!");
                    estadoInv.emProtecao           = false;
                    estadoInv.faseTesteDiagnostico = 0;
                    estadoInv.totalReiniciamentos++;
                    salvarEstadoInversor();
                }
            } else {
                bool emTransitorio = (agora - estadoInv.tsProtecao)
                < TEMPO_TRANSITORIO_REDE;
                if (!emTransitorio) {
                    adicionarLog("INV: Sinal instavel! Iniciando diagnostico...");
                    adicionarLog("INV: Desligando Rede Alta para teste...");

                    setRedeAlta(false);
                    estadoRede.redeAltaAtiva = false;

                    setInversor(false);
                    delay(2000);
                    setInversor(true);

                    estadoInv.tsProtecao           = agora;
                    estadoInv.tsUltimoSinal        = agora;
                    estadoInv.faseTesteDiagnostico = 2;

                    adicionarLog("INV: Aguardando sinal SEM rede alta...");
                }
            }
            break;
        }

        // ──────────────────────────────────────────────────
        // FASE 2: TESTE SEM REDE ALTA
        // ──────────────────────────────────────────────────
        case 2: {
            bool emTransitorio = (agora - estadoInv.tsProtecao)
            < TEMPO_TRANSITORIO_REDE;
            if (emTransitorio) break;

            if (sinalAtual) {
                unsigned long tempoEstavel = agora - estadoInv.tsUltimoSinal;

                if (tempoEstavel >= TEMPO_ESTABILIDADE_OK) {
                    adicionarLog("DIAGNOSTICO: Problema na REDE ALTA!");
                    adicionarLog("Possivel curto ou equipamento com defeito.");
                    notificarCurtoRA();
                    estadoInv.emProtecao           = false;
                    estadoInv.faseTesteDiagnostico = 4;
                }
            } else {
                if ((agora - estadoInv.tsProtecao) >= TEMPO_DIAGNOSTICO_MAX) {
                    adicionarLog("INV: Instavel sem RA! Testando sem Rede Baixa...");
                    adicionarLog("INV: Desligando Rede Baixa para teste...");

                    setRedeBaixa(false);
                    estadoRede.redeBaixaAtiva = false;

                    setInversor(false);
                    delay(2000);
                    setInversor(true);

                    estadoInv.tsProtecao           = agora;
                    estadoInv.tsUltimoSinal        = agora;
                    estadoInv.faseTesteDiagnostico = 3;

                    adicionarLog("INV: Aguardando sinal SEM rede baixa...");
                }
            }
            break;
        }

        // ──────────────────────────────────────────────────
        // FASE 3: TESTE SEM REDE BAIXA
        // ──────────────────────────────────────────────────
        case 3: {
            bool emTransitorio = (agora - estadoInv.tsProtecao)
            < TEMPO_TRANSITORIO_REDE;
            if (emTransitorio) break;

            if (sinalAtual) {
                unsigned long tempoEstavel = agora - estadoInv.tsUltimoSinal;

                if (tempoEstavel >= TEMPO_ESTABILIDADE_OK) {
                    adicionarLog("DIAGNOSTICO: Problema na REDE BAIXA!");
                    adicionarLog("Possivel curto ou equipamento com defeito.");
                    notificarCurtoRB();
                    estadoInv.emProtecao           = false;
                    estadoInv.faseTesteDiagnostico = 5;
                }
            } else {
                if ((agora - estadoInv.tsProtecao) >= TEMPO_DIAGNOSTICO_MAX) {
                    adicionarLog("DIAGNOSTICO: Problema no INVERSOR!");
                    adicionarLog("Verifique o inversor fisicamente.");
                    adicionarLog("Desligando inversor por seguranca.");

                    setInversor(false);
                    estadoInv.emProtecao           = true;
                    estadoInv.faseTesteDiagnostico = 0;
                    salvarEstadoInversor();
                }
            }
            break;
        }

        // ──────────────────────────────────────────────────
        // FASE 4: AGUARDA CONFIRMAÇÃO USUÁRIO - REDE ALTA
        // ──────────────────────────────────────────────────
        case 4: {
            if (estadoRede.redeAltaAtiva) {
                setRedeAlta(false);
                estadoRede.redeAltaAtiva = false;
                adicionarLog("INV: Rede Alta mantida OFF (aguarda confirmacao)");
            }
            break;
        }

        // ──────────────────────────────────────────────────
        // FASE 5: AGUARDA CONFIRMAÇÃO USUÁRIO - REDE BAIXA
        // ──────────────────────────────────────────────────
        case 5: {
            if (estadoRede.redeBaixaAtiva) {
                setRedeBaixa(false);
                estadoRede.redeBaixaAtiva = false;
                adicionarLog("INV: Rede Baixa mantida OFF (aguarda confirmacao)");
            }
            break;
        }

        // ──────────────────────────────────────────────────
        // FASE 6: TESTE FINAL REDE ALTA (após confirmação)
        // ──────────────────────────────────────────────────
        case 6: {
            bool emTransitorio = (agora - estadoInv.tsProtecao)
            < TEMPO_TRANSITORIO_REDE;
            if (emTransitorio) break;

            if (sinalAtual) {
                unsigned long tempoEstavel = agora - estadoInv.tsUltimoSinal;

                if (tempoEstavel >= TEMPO_ESTABILIDADE_OK) {
                    adicionarLog("TESTE OK: Rede Alta funcionando normal!");
                    estadoInv.redeAltaBloqCurto       = false;
                    estadoInv.aguardandoConfirmacaoRA  = false;
                    estadoInv.faseTesteDiagnostico     = 0;
                    estadoInv.emProtecao               = false;
                    adicionarLog("Rede Alta LIBERADA - sistema normal.");
                    salvarEstadoInversor();
                }
            } else {
                if ((agora - estadoInv.tsProtecao) >= TEMPO_DIAGNOSTICO_MAX) {
                    adicionarLog("TESTE FALHOU: Rede Alta ainda com problema!");
                    adicionarLog("Verifique novamente e confirme no /rede");

                    setRedeAlta(false);
                    estadoRede.redeAltaAtiva          = false;
                    estadoInv.redeAltaBloqCurto       = true;
                    estadoInv.aguardandoConfirmacaoRA  = true;
                    estadoInv.faseTesteDiagnostico     = 4;
                    salvarEstadoInversor();
                }
            }
            break;
        }

        // ──────────────────────────────────────────────────
        // FASE 7: TESTE FINAL REDE BAIXA (após confirmação)
        // ──────────────────────────────────────────────────
        case 7: {
            bool emTransitorio = (agora - estadoInv.tsProtecao)
            < TEMPO_TRANSITORIO_REDE;
            if (emTransitorio) break;

            if (sinalAtual) {
                unsigned long tempoEstavel = agora - estadoInv.tsUltimoSinal;

                if (tempoEstavel >= TEMPO_ESTABILIDADE_OK) {
                    adicionarLog("TESTE OK: Rede Baixa funcionando normal!");
                    estadoInv.redeBaixaBloqCurto       = false;
                    estadoInv.aguardandoConfirmacaoRB  = false;
                    estadoInv.faseTesteDiagnostico     = 0;
                    estadoInv.emProtecao               = false;

                    setRedeBaixa(true);
                    estadoRede.redeBaixaAtiva = true;
                    delay(500);

                    if (!estadoInv.redeAltaBloqCurto &&
                        !estadoPrev.bloqRedeAlta) {
                        setRedeAlta(true);
                    estadoRede.redeAltaAtiva = true;
                        }
                        adicionarLog("Rede Baixa LIBERADA - sistema normal.");
                        salvarEstadoInversor();
                }
            } else {
                if ((agora - estadoInv.tsProtecao) >= TEMPO_DIAGNOSTICO_MAX) {
                    adicionarLog("TESTE FALHOU: Rede Baixa ainda com problema!");
                    adicionarLog("Verifique novamente e confirme no /rede");

                    setRedeBaixa(false);
                    estadoRede.redeBaixaAtiva         = false;
                    estadoInv.redeBaixaBloqCurto      = true;
                    estadoInv.aguardandoConfirmacaoRB  = true;
                    estadoInv.faseTesteDiagnostico     = 5;
                    salvarEstadoInversor();
                }
            }
            break;
        }

    } // fim switch
}

// ============================================================
// FUNÇÃO: confirmarConsertoRedeAlta()
// Chamada pela página /rede quando usuário confirma conserto
// ============================================================
void confirmarConsertoRedeAlta() {
    if (!estadoInv.aguardandoConfirmacaoRA) return;

    adicionarLog("Usuario confirmou conserto Rede Alta!");
    adicionarLog("Iniciando teste da Rede Alta...");

    setRedeAlta(true);
    estadoRede.redeAltaAtiva = true;

    estadoInv.tsProtecao           = millis();
    estadoInv.tsUltimoSinal        = millis();
    estadoInv.faseTesteDiagnostico = 6;
}

// ============================================================
// FUNÇÃO: confirmarConsertoRedeBaixa()
// Chamada pela página /rede quando usuário confirma conserto
// ============================================================
void confirmarConsertoRedeBaixa() {
    if (!estadoInv.aguardandoConfirmacaoRB) return;

    adicionarLog("Usuario confirmou conserto Rede Baixa!");
    adicionarLog("Iniciando teste da Rede Baixa...");

    setInversor(true);
    delay(2000);
    setRedeBaixa(true);
    estadoRede.redeBaixaAtiva = true;

    estadoInv.tsProtecao           = millis();
    estadoInv.tsUltimoSinal        = millis();
    estadoInv.faseTesteDiagnostico = 7;
}

// ============================================================
// SETUP REDE ELÉTRICA
// ============================================================

// ── Relé Rede Baixa ──
void setRedeBaixa(bool ligada) {
    estadoRede.redeBaixaAtiva = ligada;
    digitalWrite(PIN_RELE_REDE_BAIXA, ligada ? LOW : HIGH);
    if (ligada) {
        estadoInv.tsLigouRede = millis();
        adicionarLog("Rede Baixa: LIGADA");
    } else {
        adicionarLog("Rede Baixa: DESLIGADA");
    }
}

// ── Relé Rede Alta ──
void setRedeAlta(bool ligada) {
    estadoRede.redeAltaAtiva = ligada;
    digitalWrite(PIN_RELE_REDE_ALTA, ligada ? LOW : HIGH);
    if (ligada) {
        estadoInv.tsLigouRede = millis();
        adicionarLog("Rede Alta: LIGADA");
    } else {
        adicionarLog("Rede Alta: DESLIGADA");
    }
}

// ── Relé Lâmpada Externa ──
void setLampadaExterna(bool ligada) {
    estadoRede.lampExternaAtiva = ligada;
    estadoRede.lampExtBloqEnergia = !ligada;
    digitalWrite(PIN_RELE_LAMP_EXTERNA, ligada ? LOW : HIGH);
}

// ── Relé Lâmpada Interna ──
void setLampadaInterna(bool ligada) {
    estadoRede.lampInternaAtiva = ligada;
    digitalWrite(PIN_RELE_LAMP_INTERNA, ligada ? LOW : HIGH);
}

void setupRede() {
    pinMode(PIN_RELE_REDE_BAIXA,   OUTPUT);
    pinMode(PIN_RELE_REDE_ALTA,    OUTPUT);
    pinMode(PIN_RELE_LAMP_EXTERNA, OUTPUT);
    pinMode(PIN_RELE_LAMP_INTERNA, OUTPUT);

    // Estado inicial seguro: tudo LIGADO
    setRedeBaixa(true);
    setRedeAlta(true);
    setLampadaExterna(true);
    setLampadaInterna(true);

    estadoRede.redeBaixaAtiva   = true;
    estadoRede.redeAltaAtiva    = true;
    estadoRede.lampExternaAtiva = true;
    estadoRede.lampInternaAtiva = true;

    // Zera timers e flags
    estadoRede.tsAbaixoRedeBaixa  = 0;
    estadoRede.tsAbaixoRedeAlta   = 0;
    estadoRede.tsAbaixoLampExt    = 0;
    estadoRede.tsAbaixoLampInt    = 0;
    estadoRede.aguardandoRedeBaixa = false;
    estadoRede.aguardandoRedeAlta  = false;
    estadoRede.aguardandoLampExt   = false;
    estadoRede.aguardandoLampInt   = false;
    estadoRede.lampExtBloqEnergia  = false;

    adicionarLog("Rede eletrica: inicializada");
}

void setupInversor() {
    pinMode(PIN_RELE_INV_ALTA,   OUTPUT);
    pinMode(PIN_STATUS_INV_ALTA, INPUT);

    // Começa DESLIGADO (HIGH = desligado na nova lógica)
    digitalWrite(PIN_RELE_INV_ALTA, HIGH);
    estadoInv.invLigado            = false;
    estadoInv.emProtecao           = false;
    estadoInv.testando             = false;
    estadoInv.tsUltimoSinal        = 0;
    estadoInv.tsLigouRede          = 0;
    estadoInv.faseTesteDiagnostico = 0;

    adicionarLog("Inversor: inicializado (desligado)");

    // Liga após 2 segundos
    delay(2000);
    setInversor(true);
    adicionarLog("Inversor: LIGADO");
}

// ============================================================
// CONTROLE LÂMPADA EXTERNA (horário/painel)
// Reutiliza: sensores.correntePainel (calculado em lerSensores)
// ============================================================
void controlarLampadaExterna() {
    if (estadoRede.lampExtBloqEnergia) return;

    // Modo 0: apenas energia controla
    if (cfgRede.modoLampExt == 0) {
        if (!estadoRede.lampExternaAtiva)
            setLampadaExterna(true);
        estadoRede.lampExtPorHorario = false;
        estadoRede.lampExtPorPainel  = false;
        return;
    }

    // ── Modo 1: Horário Manual ──
    if (cfgRede.modoLampExt == 1) {
        if (!ntpSincronizado) return;

        time_t now = time(nullptr);
        struct tm* ti = localtime(&now);
        int minAgora = ti->tm_hour * 60 + ti->tm_min;
        int minLigar = cfgRede.ligarHora  * 60 + cfgRede.ligarMin;
        int minDesl  = cfgRede.desligarHora * 60 + cfgRede.desligarMin;

        bool deveEstarLigada = (minLigar < minDesl)
            ? (minAgora >= minLigar || minAgora < minDesl)
            : (minAgora >= minLigar && minAgora < minDesl);

        if (deveEstarLigada != estadoRede.lampExternaAtiva) {
            setLampadaExterna(deveEstarLigada);
            estadoRede.lampExtPorHorario = deveEstarLigada;
            if (deveEstarLigada) {
                estadoRede.totalLigadasHorario++;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Lamp.Ext LIGADA por horario (%02d:%02d)",
                         ti->tm_hour, ti->tm_min);
                adicionarLog(tempBuf);
                salvarEstadoRede();
            } else {
                snprintf(tempBuf, sizeof(tempBuf),
                         "Lamp.Ext DESLIGADA por horario (%02d:%02d)",
                         ti->tm_hour, ti->tm_min);
                adicionarLog(tempBuf);
            }
        }
        estadoRede.lampExtPorPainel = false;
        return;
    }

    // ── Modo 2: Automático por Painel Solar ──
    if (cfgRede.modoLampExt == 2) {
        // Reutiliza correntePainel já calculado
        bool painelAtivo = (sensores.correntePainel > 0.5f);

        if (painelAtivo && estadoRede.lampExternaAtiva) {
            setLampadaExterna(false);
            estadoRede.lampExtPorPainel = true;
            estadoRede.totalLigadasPainel++;
            adicionarLog("Lamp.Ext DESLIGADA: painel gerando (dia)");
            salvarEstadoRede();
        } else if (!painelAtivo && !estadoRede.lampExternaAtiva
                   && estadoRede.lampExtPorPainel) {
            setLampadaExterna(true);
            estadoRede.lampExtPorPainel = true;
            adicionarLog("Lamp.Ext LIGADA: painel parou (noite)");
        }
        estadoRede.lampExtPorHorario = false;
    }
}

// ============================================================
// CONTROLE DA REDE ELÉTRICA
// Reutiliza: estadoBat.ahDisponiveis, sensores.tensaoBateriaEst
//            cfgBat.energiaDisponivel (pré-calculado)
// Prioridade de corte: LampInt → LampExt → RedeAlta → RedeBaixa
// ============================================================
void controlarRede() {
    if (!existeConfigRede())    return;
    if (!existeBateriaConfig()) return;

    unsigned long agora = millis();

    // ── Calcula UMA VEZ - reutilizado por todos os controles ──
    float soc = 0.0f;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    // whDisp reutiliza tensaoBateriaEst já calculada
    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float eD     = cfgBat.energiaDisponivel;

    // Limites de corte
    float whMinRB   = eD * (cfgRede.percRedeBaixa / 100.0f);
    float whMinRA   = eD * (cfgRede.percRedeAlta  / 100.0f);
    float whMinLE   = eD * (cfgRede.percLampExt   / 100.0f);
    float whMinLI   = eD * (cfgRede.percLampInt   / 100.0f);

    // Limites de religamento (+ histerese)
    float whMinRBOn = eD * ((cfgRede.percRedeBaixa + HISTERESE_REDE_PERC) / 100.0f);
    float whMinRAOn = eD * ((cfgRede.percRedeAlta  + HISTERESE_REDE_PERC) / 100.0f);
    float whMinLEOn = eD * ((cfgRede.percLampExt   + HISTERESE_REDE_PERC) / 100.0f);
    float whMinLIOn = eD * ((cfgRede.percLampInt   + HISTERESE_REDE_PERC) / 100.0f);

    // ════════════════════════════════════════════════════════
    // CONTROLE 4: LÂMPADA INTERNA
    // ════════════════════════════════════════════════════════
    if (estadoRede.lampInternaAtiva) {
        if (whDisp <= whMinLI) {
            if (!estadoRede.aguardandoLampInt) {
                estadoRede.aguardandoLampInt = true;
                estadoRede.tsAbaixoLampInt   = agora;
                adicionarLog("Rede: aguardando 1min lamp.interna...");
            } else if ((agora - estadoRede.tsAbaixoLampInt) >= TEMPO_DECISAO_REDE_MS) {
                setLampadaInterna(false);
                estadoRede.aguardandoLampInt = false;
                estadoRede.totalCortesLampInt++;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: Lamp.Int DESLIGADA (%.0f<%.0fWh)", whDisp, whMinLI);
                adicionarLog(tempBuf);
                salvarEstadoRede();
            }
        } else {
            if (estadoRede.aguardandoLampInt) {
                estadoRede.aguardandoLampInt = false;
                adicionarLog("Rede: contagem lamp.int cancelada");
            }
        }
    } else {
        if (whDisp > whMinLIOn)
            setLampadaInterna(true);
    }

    // ════════════════════════════════════════════════════════
    // CONTROLE 3: LÂMPADA EXTERNA
    // PRIORIDADE: Previsão > Energia > Horário/Painel
    // ════════════════════════════════════════════════════════
    if (estadoPrev.bloqLampExt) {
        estadoRede.lampExtBloqEnergia = true;
        if (estadoRede.lampExternaAtiva) {
            setLampadaExterna(false);
            estadoRede.aguardandoLampExt = false;
            snprintf(tempBuf, sizeof(tempBuf),
                     "PREV: Lamp.Ext DESLIGADA - %s",
                     estadoPrev.motivoLampExt);
            adicionarLog(tempBuf);
        }
    } else if (whDisp <= whMinLE) {
        estadoRede.lampExtBloqEnergia = true;
        if (estadoRede.lampExternaAtiva) {
            if (!estadoRede.aguardandoLampExt) {
                estadoRede.aguardandoLampExt = true;
                estadoRede.tsAbaixoLampExt   = agora;
                adicionarLog("Rede: aguardando 1min lamp.externa...");
            } else if ((agora - estadoRede.tsAbaixoLampExt) >= TEMPO_DECISAO_REDE_MS) {
                setLampadaExterna(false);
                estadoRede.aguardandoLampExt = false;
                estadoRede.totalCortesLampExt++;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: Lamp.Ext DESLIGADA energia (%.0f<%.0fWh)",
                         whDisp, whMinLE);
                adicionarLog(tempBuf);
                salvarEstadoRede();
            }
        }
    } else {
        if (estadoRede.aguardandoLampExt) {
            estadoRede.aguardandoLampExt = false;
            adicionarLog("Rede: contagem lamp.ext cancelada");
        }
        if (estadoRede.lampExtBloqEnergia &&
            whDisp > whMinLEOn && !estadoPrev.bloqLampExt) {
            estadoRede.lampExtBloqEnergia = false;
            adicionarLog("Rede: Lamp.Ext desbloqueada (energia OK)");
        }
        if (!estadoRede.lampExtBloqEnergia)
            controlarLampadaExterna();
    }

    // ════════════════════════════════════════════════════════
    // CONTROLE 2: REDE ALTA
    // PRIORIDADE: Previsão > Energia
    // ════════════════════════════════════════════════════════
    if (estadoPrev.bloqRedeAlta) {
        if (estadoRede.redeAltaAtiva) {
            setRedeAlta(false);
            estadoRede.aguardandoRedeAlta = false;
            snprintf(tempBuf, sizeof(tempBuf),
                     "PREV: Rede Alta DESLIGADA - %s",
                     estadoPrev.motivoRedeAlta);
            adicionarLog(tempBuf);
            salvarEstadoRede();
        }
    } else if (!estadoPrev.bloqRedeAlta &&
               !estadoRede.redeAltaAtiva &&
               !estadoPrev.redeAltaFoiDesligada) {
        if (whDisp > whMinRAOn) {
            setRedeAlta(true);
            snprintf(tempBuf, sizeof(tempBuf),
                     "PREV: Rede Alta REATIVADA (%.0f>%.0fWh)",
                     whDisp, whMinRAOn);
            adicionarLog(tempBuf);
        }
    } else {
        // Controle normal por energia
        if (estadoRede.redeAltaAtiva) {
            if (whDisp <= whMinRA) {
                if (!estadoRede.aguardandoRedeAlta) {
                    estadoRede.aguardandoRedeAlta = true;
                    estadoRede.tsAbaixoRedeAlta   = agora;
                    adicionarLog("Rede: aguardando 1min rede alta...");
                } else if ((agora - estadoRede.tsAbaixoRedeAlta) >= TEMPO_DECISAO_REDE_MS) {
                    setRedeAlta(false);
                    estadoRede.aguardandoRedeAlta = false;
                    estadoRede.totalCortesRedeAlta++;
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Rede: Rede Alta DESLIGADA (%.0f<%.0fWh)",
                             whDisp, whMinRA);
                    adicionarLog(tempBuf);
                    salvarEstadoRede();
                }
            } else {
                if (estadoRede.aguardandoRedeAlta) {
                    estadoRede.aguardandoRedeAlta = false;
                    adicionarLog("Rede: contagem rede alta cancelada");
                }
            }
        } else {
            if (whDisp > whMinRAOn) {
                setRedeAlta(true);
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: Rede Alta REATIVADA (%.0f>%.0fWh)",
                         whDisp, whMinRAOn);
                adicionarLog(tempBuf);
            }
        }
    }

    // ════════════════════════════════════════════════════════
    // CONTROLE 1: REDE BAIXA + INVERSOR
    // Corte mais crítico - desliga o inversor também
    // ════════════════════════════════════════════════════════
    if (estadoRede.redeBaixaAtiva) {
        if (whDisp <= whMinRB) {
            if (!estadoRede.aguardandoRedeBaixa) {
                estadoRede.aguardandoRedeBaixa = true;
                estadoRede.tsAbaixoRedeBaixa   = agora;
                adicionarLog("Rede: aguardando 1min rede baixa...");
            } else if ((agora - estadoRede.tsAbaixoRedeBaixa) >= TEMPO_DECISAO_REDE_MS) {
                // Desliga tudo: rede alta, rede baixa e inversor
                setRedeAlta(false);
                setRedeBaixa(false);
                setInversor(false);
                estadoRede.redeAltaAtiva      = false;
                estadoRede.aguardandoRedeBaixa = false;
                estadoRede.totalCortesRedeBaixa++;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: CORTE TOTAL - Inversor OFF (%.0f<%.0fWh)",
                         whDisp, whMinRB);
                adicionarLog(tempBuf);
                salvarEstadoRede();
            }
        } else {
            if (estadoRede.aguardandoRedeBaixa) {
                estadoRede.aguardandoRedeBaixa = false;
                adicionarLog("Rede: contagem rede baixa cancelada");
            }
        }
    } else {
        // Tenta religar quando energia recuperou
        if (whDisp > whMinRBOn) {
            setInversor(true);
            delay(2000); // aguarda inversor inicializar
            setRedeBaixa(true);
            estadoRede.redeBaixaAtiva = true;
            if (whDisp > whMinRAOn && !estadoPrev.bloqRedeAlta) {
                setRedeAlta(true);
                estadoRede.redeAltaAtiva = true;
            }
            snprintf(tempBuf, sizeof(tempBuf),
                     "Rede: RELIGADO - Inversor ON (%.0f>%.0fWh)",
                     whDisp, whMinRBOn);
            adicionarLog(tempBuf);
        }
    }
}

// ============================================================
// SENSOR ADC
// ============================================================
float lerADCMedia(uint8_t pin) {
    uint32_t soma = 0;
    for (int i = 0; i < AMOSTRAS_ADC; i++) {
        soma += analogRead(pin);
        delayMicroseconds(100);
    }
    return (float)soma / AMOSTRAS_ADC;
}

void calibrarZeroSensores() {
    adicionarLog("Calibrando zero sensores...");
    adcOffsetPainel = lerADCMedia(PIN_SENSOR_PAINEL);
    adcOffsetCasa   = lerADCMedia(PIN_SENSOR_CASA);
    snprintf(tempBuf, sizeof(tempBuf),
             "Zero Painel: %.1f | Zero Casa: %.1f",
             adcOffsetPainel, adcOffsetCasa);
    adicionarLog(tempBuf);
}

// ============================================================
// ESTIMATIVA DE TENSÃO DA BATERIA (LiFePO4 - OCV por SoC)
// ============================================================
float estimarTensaoBateria() {
    if (cfgBat.amperagem <= 0 || estadoBat.ahMaximos <= 0)
        return cfgBat.tensaoNominal;

    float soc = constrain(estadoBat.ahDisponiveis / estadoBat.ahMaximos,
                          0.0f, 1.0f);
    float fatorEscala = cfgBat.tensaoNominal / 12.8f;

    const float ocvTabela[] = {
        10.00f, 12.00f, 12.50f, 12.80f, 12.90f,
        13.00f, 13.10f, 13.20f, 13.30f, 13.50f, 14.00f
    };

    float socPct = soc * 10.0f;
    int   idx    = (int)socPct;
    if (idx >= 10) return ocvTabela[10] * fatorEscala;
    if (idx <  0)  return ocvTabela[0]  * fatorEscala;

    float frac = socPct - idx;
    float ocv  = ocvTabela[idx] + frac * (ocvTabela[idx+1] - ocvTabela[idx]);
    return ocv * fatorEscala;
}

// ============================================================
// LEITURA DOS SENSORES
// ============================================================
void lerSensores() {
    // ── Painel Solar (ACS758 50A) ──
    float rawPainel       = lerADCMedia(PIN_SENSOR_PAINEL);
    float voltsDeltaP     = (rawPainel - adcOffsetPainel) * (ADC_VREF / ADC_RESOLUCAO);
    float corrPainel      = voltsDeltaP / SENSIBILIDADE_50A;

    // ── Casa (ACS758 100A) ──
    float rawCasa         = lerADCMedia(PIN_SENSOR_CASA);
    float voltsDeltaC     = (rawCasa - adcOffsetCasa) * (ADC_VREF / ADC_RESOLUCAO);
    float corrCasa        = voltsDeltaC / SENSIBILIDADE_100A;

    // ── Filtro de ruído ──
    if (fabsf(corrPainel) < 0.3f) corrPainel = 0.0f;
    if (fabsf(corrCasa)   < 0.3f) corrCasa   = 0.0f;

    // ── Detecção de inversão painel ──
    bool painelInvertido = false;
    if (corrPainel > 1.0f) {
        painelInvertido = true;
        corrPainel = -corrPainel;
    }
    if (corrPainel < -0.3f && corrPainel > -1.0f) corrPainel = 0.0f;
    corrPainel = fabsf(corrPainel);

    // ── Detecção de inversão casa ──
    bool casaInvertida = false;
    if (corrCasa < -0.5f) {
        casaInvertida = true;
        corrCasa = fabsf(corrCasa);
    }

    // ── Sinal BKP ──
    bool sinalBKP = (digitalRead(PIN_BKP_BATERIA) == HIGH);

    // ── Tensão e potências ──
    float tensaoEst = estimarTensaoBateria();
    float corrBat   = corrPainel - corrCasa;

    // ── Atualiza estrutura global ──
    sensores.correntePainel        = corrPainel;
    sensores.correnteCasa          = corrCasa;
    sensores.correnteBateria       = corrBat;
    sensores.potenciaPainel        = corrPainel  * tensaoEst;
    sensores.potenciaCasa          = corrCasa    * tensaoEst;
    sensores.potenciaBateria       = corrBat     * tensaoEst;
    sensores.tensaoBateriaEst      = tensaoEst;
    sensores.bateriaCarregando     = (corrBat >= 0);
    sensores.alertaPainelInvertido = painelInvertido;
    sensores.alertaCasaInvertida   = casaInvertida;
    sensores.sinalBKP              = sinalBKP;
}

// ============================================================
// ATUALIZAR ESTADO DA BATERIA (Coulomb Counting)
// ============================================================
void atualizarEstadoBateria(float deltaSegundos) {
    float corrBat = sensores.correnteBateria;
    float deltaAh = corrBat * deltaSegundos / 3600.0f;

    // Detecta bateria 100% carregada
    bool painelAtivo  = sensores.correntePainel > 0.5f;
    bool batZero      = fabsf(corrBat) < 0.3f;
    bool casaOuPainel = sensores.correnteCasa > 0.1f ||
                        sensores.correntePainel > 0.1f;

    if (painelAtivo && batZero && casaOuPainel && !estadoBat.carregada100) {
        estadoBat.carregada100  = true;
        estadoBat.ahDisponiveis = estadoBat.ahMaximos;
        estadoBat.tsCarregada   = millis();
        adicionarLog("Bateria 100% carregada detectada!");
        salvarEstadoBateria();
        salvarContadores();
    }

    // BKP: sinal do controlador -> bateria crítica
    if (sensores.sinalBKP && estadoBat.ahDisponiveis > 0) {
        float ahNaBKP = cfgBat.amperagem *
                        (1.0f - cfgBat.percDescargaSegura) * 0.5f;
        if (estadoBat.ahDisponiveis > ahNaBKP) {
            estadoBat.ahDisponiveis = ahNaBKP;
            adicionarLog("BKP: bateria nivel critico!");
            salvarEstadoBateria();
        }
    }

    if (estadoBat.carregada100 && deltaAh < 0)
        estadoBat.carregada100 = false;

    estadoBat.ahDisponiveis = constrain(
        estadoBat.ahDisponiveis + deltaAh, 0.0f, estadoBat.ahMaximos);

    // Contagem de ciclos
    if (deltaAh < 0) {
        float deltaAhAbs = fabsf(deltaAh);
        estadoBat.ciclosParciais += deltaAhAbs / estadoBat.ahMaximos;
        if (estadoBat.ciclosParciais >= 1.0f) {
            estadoBat.ciclos++;
            estadoBat.ciclosParciais -= 1.0f;
            snprintf(tempBuf, sizeof(tempBuf),
                     "Ciclo completo! Total: %.0f", estadoBat.ciclos);
            adicionarLog(tempBuf);
        }
    }

    // Acumuladores para histórico
    float whPainel = sensores.potenciaPainel * deltaSegundos / 3600.0f;
    float whCasa   = sensores.potenciaCasa   * deltaSegundos / 3600.0f;
    acumWhPainel += whPainel;
    acumWhCasa   += whCasa;

    if (corrBat > 0) {
        float wh = sensores.potenciaBateria * deltaSegundos / 3600.0f;
        acumWhCargaBat            += wh;
        contadores.kwhCargaTotal  += wh / 1000.0;
    } else if (corrBat < 0) {
        float wh = fabsf(sensores.potenciaBateria) * deltaSegundos / 3600.0f;
        acumWhDescBat               += wh;
        contadores.kwhDescargaTotal += wh / 1000.0;
    }

    contadores.kwhPainelTotal += whPainel / 1000.0;
    contadores.kwhCasaTotal   += whCasa   / 1000.0;
}

// ============================================================
// SALVAR HISTÓRICO
// ============================================================
void salvarHistorico() {
    time_t now = time(nullptr);
    historico[histIdx].timestamp     = (uint32_t)now;
    historico[histIdx].whPainel      = (int16_t)(acumWhPainel  * 10.0f);
    historico[histIdx].whCasa        = (int16_t)(acumWhCasa    * 10.0f);
    historico[histIdx].whCargaBat    = (int16_t)(acumWhCargaBat * 10.0f);
    historico[histIdx].whDescargaBat = (int16_t)(acumWhDescBat  * 10.0f);

    histIdx = (histIdx + 1) % MAX_HISTORICO;
    if (histTotal < MAX_HISTORICO) histTotal++;

    acumWhPainel   = 0;
    acumWhCasa     = 0;
    acumWhCargaBat = 0;
    acumWhDescBat  = 0;

    static uint8_t cntSave = 0;
    if (++cntSave >= 5) {
        cntSave = 0;
        salvarContadores();
        salvarEstadoBateria();
    }
}

// ============================================================
// PREFERENCES - WiFi e Rede
// ============================================================
void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP ativo: RRFS Energia");
    adicionarLog("IP: 10.0.0.1");
    logRAMSeNecessario();
    ledBlink();
}

bool iniciarWiFi(const String& ssid, const String& pass,
                  const String& ip,   const String& gw,
                  const String& mask, const String& dns) {
    IPAddress lIP, lGW, lMask, lDNS;
    if (!lIP.fromString(ip))    { adicionarLog("ERRO: IP invalido");      return false; }
    if (!lGW.fromString(gw))    { adicionarLog("ERRO: Gateway invalido"); return false; }
    if (!lMask.fromString(mask)){ adicionarLog("ERRO: Mascara invalida"); return false; }
    if (!lDNS.fromString(dns))  { adicionarLog("ERRO: DNS invalido");     return false; }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    if (!WiFi.config(lIP, lGW, lMask, lDNS)) {
        adicionarLog("ERRO: Falha config IP estatico");
        return false;
    }

    snprintf(tempBuf, sizeof(tempBuf), "Conectando: %s", ssid.c_str());
    adicionarLog(tempBuf);
    ledBlink();
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long t = millis();
    int tent = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
        atualizarLED(); delay(500); tent++;
        if (tent % 4 == 0) {
            snprintf(tempBuf, sizeof(tempBuf), "Tentando... %ds", tent/2);
            adicionarLog(tempBuf);
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi OK! IP: %s", WiFi.localIP().toString().c_str());
        adicionarLog(tempBuf);
        snprintf(tempBuf, sizeof(tempBuf), "RSSI: %d dBm", WiFi.RSSI());
        adicionarLog(tempBuf);
        modoAP = false;
        ledFixo(true);
        return true;
    }

    switch (WiFi.status()) {
        case WL_NO_SSID_AVAIL:  adicionarLog("ERRO: SSID nao encontrado"); break;
        case WL_CONNECT_FAILED: adicionarLog("ERRO: Senha incorreta");     break;
        default:
            snprintf(tempBuf, sizeof(tempBuf), "ERRO WiFi: %d", WiFi.status());
            adicionarLog(tempBuf);
    }
    return false;
}

// ============================================================
// SEGURANÇA
// ============================================================
bool checarAuth() {
    if (!logado) {
        adicionarLog("Acesso negado: sem autenticacao");
        server.sendHeader("Location", "/login");
        server.send(302);
        return false;
    }
    return true;
}

// ============================================================
// HTML BASE
// ============================================================
String gerarPagina(const String& titulo,
                   const String& conteudo,
                   const String& scriptExtra = "") {

    String html = F("<!DOCTYPE html><html><head>"
                    "<meta charset='utf-8'>"
                    "<meta name='viewport' "
                    "content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>"
              "body{background:#0d0d0d;color:#e0e0e0;"
              "font-family:Arial,sans-serif;margin:0;padding:8px;}"
              ".card{background:#1a1a1a;border:1px solid #333;"
              "border-radius:8px;padding:16px;max-width:560px;"
              "margin:0 auto;}"
              "h2{color:#4fc3f7;text-align:center;"
              "margin:0 0 16px 0;font-size:18px;}"
              "h3{color:#81d4fa;font-size:14px;"
              "margin:12px 0 8px 0;border-bottom:1px solid #333;"
              "padding-bottom:6px;}"
              "label{display:block;color:#90a4ae;font-size:12px;"
              "margin-top:10px;margin-bottom:3px;}"
              "input,select,textarea{width:100%;padding:9px;"
              "background:#111;border:1px solid #444;"
              "border-radius:4px;color:#e0e0e0;font-size:13px;"
              "box-sizing:border-box;}"
              "input:focus,select:focus,textarea:focus{"
              "outline:none;border-color:#0277bd;}"
              "button{width:100%;padding:11px;margin-top:12px;"
              "background:#0277bd;color:#fff;border:none;"
              "border-radius:4px;font-size:14px;font-weight:bold;"
              "cursor:pointer;}"
              "button:hover{background:#039be5;}"
              ".btn-red{background:#c62828;}"
              ".btn-red:hover{background:#ef5350;}"
              ".btn-green{background:#2e7d32;}"
              ".btn-green:hover{background:#43a047;}"
              ".btn-orange{background:#ef6c00;}"
              ".btn-orange:hover{background:#f57c00;}"
              ".btn-gray{background:#546e7a;}"
              ".btn-gray:hover{background:#607d8b;}"
              ".msg-ok{background:#1b5e20;color:#a5d6a7;"
              "padding:10px;border-radius:4px;margin:8px 0;"
              "text-align:center;font-size:13px;}"
              ".msg-err{background:#b71c1c;color:#ef9a9a;"
              "padding:10px;border-radius:4px;margin:8px 0;"
              "text-align:center;font-size:13px;}"
              ".msg-warn{background:#e65100;color:#fff9c4;"
              "padding:10px;border-radius:4px;margin:8px 0;"
              "font-size:13px;}"
              ".info{background:#0d47a1;color:#90caf9;"
              "padding:10px;border-radius:4px;margin:8px 0;"
              "font-size:12px;line-height:1.7;}"
              ".stat-grid{display:grid;"
              "grid-template-columns:1fr 1fr;"
              "gap:8px;margin:8px 0;}"
              ".stat-box{background:#111;border:1px solid #333;"
              "border-radius:6px;padding:10px;text-align:center;}"
              ".stat-label{color:#546e7a;font-size:10px;"
              "text-transform:uppercase;margin-bottom:4px;}"
              ".stat-val{color:#4fc3f7;font-size:18px;"
              "font-weight:bold;}"
              ".stat-unit{color:#78909c;font-size:10px;"
              "margin-top:2px;}"
              ".stat-ok{color:#66bb6a;}"
              ".stat-warn{color:#ffa726;}"
              ".stat-err{color:#ef5350;}"
              ".nav{display:flex;gap:3px;flex-wrap:wrap;"
              "margin-bottom:12px;}"
              ".nav a{flex:1;min-width:50px;text-align:center;"
              "padding:6px 2px;background:#1e3a5f;color:#90caf9;"
              "border-radius:4px;text-decoration:none;"
              "font-size:9px;font-weight:bold;"
              "border:1px solid #1565c0;}"
              ".nav a:hover{background:#1565c0;}"
              ".nav a.ativo{background:#0277bd;color:#fff;}"
              ".log-titulo{color:#546e7a;font-size:10px;"
              "text-transform:uppercase;margin-top:16px;"
              "margin-bottom:4px;}"
              ".log-box{background:#000;border:1px solid #222;"
              "border-radius:4px;padding:8px;height:160px;"
              "overflow-y:auto;font-family:monospace;"
              "font-size:10px;color:#76ff03;}"
              ".log-linha{padding:2px 0;"
              "border-bottom:1px solid #0a0a0a;}"
              ".flex{display:flex;gap:8px;margin-top:12px;}"
              ".flex button{margin-top:0;}"
              ".hora{color:#546e7a;font-size:11px;"
              "text-align:right;margin-bottom:8px;}"
              ".ram-info{color:#37474f;font-size:10px;"
              "text-align:right;margin-top:4px;}"
              ".contador{font-family:monospace;color:#ffeb3b;"
              "font-size:14px;letter-spacing:2px;}"
              "</style></head><body><div class='card'>");

    html += "<h2>&#9889; RRFS Energia " VERSAO "</h2>";

    if (logado) {
        html += "<div class='nav'>";
        html += "<a href='/' "
                + String(titulo.indexOf("Painel") >= 0 ? "class='ativo'" : "")
                + ">&#127968; Painel</a>";
        html += "<a href='/bateria' "
                + String(titulo.indexOf("Bateria") >= 0 ? "class='ativo'" : "")
                + ">&#128267; Bat</a>";
        html += "<a href='/rede' "
                + String(titulo.indexOf("Rede") >= 0 ? "class='ativo'" : "")
                + ">&#128161; Rede</a>";
        html += "<a href='/previsao' "
                + String(titulo.indexOf("Previs") >= 0 ? "class='ativo'" : "")
                + ">&#9728; Prev</a>";
        html += "<a href='/historico' "
                + String(titulo.indexOf("Hist") >= 0 ? "class='ativo'" : "")
                + ">&#128202; Hist</a>";
        html += "<a href='/config' "
                + String(titulo.indexOf("Config") >= 0 ? "class='ativo'" : "")
                + ">&#9881; Cfg</a>";
        html += "</div>";
    }

    html += "<div class='hora' id='dh'>--/--/---- --:--:--</div>";
    html += conteudo;
    html += "<div class='log-titulo'>&#128221; LOGS DO SISTEMA</div>";
    html += "<div class='log-box' id='logbox'>"
            "<div style='color:#555'>Aguardando...</div></div>";
    html += "<div class='ram-info' id='raminfo'></div>";
    html += "</div>";

    html += F("<script>"
              "function pad(n){return n<10?'0'+n:n;}"
              "function atualizarRelogio(){"
              "var d=new Date();"
              "document.getElementById('dh').textContent="
              "pad(d.getDate())+'/'+pad(d.getMonth()+1)+'/'+"
              "d.getFullYear()+' '+"
              "pad(d.getHours())+':'+pad(d.getMinutes())+':'+"
              "pad(d.getSeconds());}"
              "setInterval(atualizarRelogio,1000);"
              "atualizarRelogio();"
              "function atualizarLogs(){"
              "var x=new XMLHttpRequest();"
              "x.onreadystatechange=function(){"
              "if(x.readyState==4&&x.status==200){"
              "try{"
              "var d=document.getElementById('logbox');"
              "var a=JSON.parse(x.responseText);"
              "var h='';"
              "for(var i=0;i<a.length;i++){"
              "h+='<div class=\"log-linha\">'+a[i]+'</div>';}"
              "d.innerHTML=h||'<div style=\"color:#555\">Sem logs</div>';"
              "}catch(e){}}};"
              "x.open('GET','/logs',true);x.send();}"
              "setInterval(atualizarLogs,4000);"
              "atualizarLogs();");

    if (scriptExtra.length() > 0) html += scriptExtra;
    html += F("</script></body></html>");
    return html;
}

// ============================================================
// API: /api/dados - JSON em tempo real
// ============================================================
void handleApiDados() {
    if (!checarAuth()) return;

    // ── Tempo estimado (reutiliza valores já calculados) ──
    float  tempoHoras = 0;
    String tempoStr   = "--";
    String tempoLabel = "";

    if (sensores.bateriaCarregando && estadoBat.ahMaximos > 0) {
        float ahFaltam = estadoBat.ahMaximos - estadoBat.ahDisponiveis;
        if (sensores.correnteBateria > 0.1f) {
            tempoHoras = ahFaltam / sensores.correnteBateria;
            int h = (int)tempoHoras;
            int m = (int)((tempoHoras - h) * 60);
            snprintf(tempBuf, sizeof(tempBuf), "%dh %02dm", h, m);
            tempoStr   = String(tempBuf);
            tempoLabel = "p/ carga completa";
        }
    } else if (!sensores.bateriaCarregando && estadoBat.ahDisponiveis > 0) {
        if (sensores.correnteCasa > 0.1f) {
            float corrRef = fabsf(sensores.correnteBateria) > 0.1f
                          ? fabsf(sensores.correnteBateria)
                          : sensores.correnteCasa;
            tempoHoras = estadoBat.ahDisponiveis / corrRef;
            int h = (int)tempoHoras;
            int m = (int)((tempoHoras - h) * 60);
            snprintf(tempBuf, sizeof(tempBuf), "%dh %02dm", h, m);
            tempoStr   = String(tempBuf);
            tempoLabel = "energia restante";
        }
    }

    // ── SoC e energia disponível ──
    float soc = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    uint32_t ram = (uint32_t)esp_get_free_heap_size() / 1024;
    String ip    = modoAP ? WiFi.softAPIP().toString()
                          : WiFi.localIP().toString();

    String j = "{";

    // Painel
    snprintf(tempBuf, sizeof(tempBuf),
             "\"iP\":%.2f,\"wP\":%.1f,",
             sensores.correntePainel, sensores.potenciaPainel);
    j += tempBuf;

    // Casa
    snprintf(tempBuf, sizeof(tempBuf),
             "\"iC\":%.2f,\"wC\":%.1f,",
             sensores.correnteCasa, sensores.potenciaCasa);
    j += tempBuf;

    // Bateria
    snprintf(tempBuf, sizeof(tempBuf),
             "\"iB\":%.2f,\"wB\":%.1f,\"vB\":%.2f,"
             "\"soc\":%.1f,\"whD\":%.0f,",
             sensores.correnteBateria, sensores.potenciaBateria,
             sensores.tensaoBateriaEst, soc, whDisp);
    j += tempBuf;

    snprintf(tempBuf, sizeof(tempBuf),
             "\"cic\":%.1f,\"car\":%d,",
             estadoBat.ciclos + estadoBat.ciclosParciais,
             sensores.bateriaCarregando ? 1 : 0);
    j += tempBuf;

    // Alertas sensores
    snprintf(tempBuf, sizeof(tempBuf),
             "\"aPi\":%d,\"aCi\":%d,\"bkp\":%d,",
             sensores.alertaPainelInvertido ? 1 : 0,
             sensores.alertaCasaInvertida   ? 1 : 0,
             sensores.sinalBKP              ? 1 : 0);
    j += tempBuf;

    // Inversor
    snprintf(tempBuf, sizeof(tempBuf),
             "\"invLig\":%d,\"invSt\":%d,\"invProt\":%d,",
             estadoInv.invLigado   ? 1 : 0,
             estadoInv.statusInv   ? 1 : 0,
             estadoInv.emProtecao  ? 1 : 0);
    j += tempBuf;

    // Rede
    snprintf(tempBuf, sizeof(tempBuf),
             "\"stRB\":%d,\"stRA\":%d,\"stLE\":%d,\"stLI\":%d,",
             estadoRede.redeBaixaAtiva   ? 1 : 0,
             estadoRede.redeAltaAtiva    ? 1 : 0,
             estadoRede.lampExternaAtiva ? 1 : 0,
             estadoRede.lampInternaAtiva ? 1 : 0);
    j += tempBuf;

    // Tempo estimado
    j += "\"tStr\":\"" + tempoStr + "\",";
    j += "\"tLbl\":\"" + tempoLabel + "\",";

    // Contadores kWh
    snprintf(tempBuf, sizeof(tempBuf),
             "\"kPai\":%.4f,\"kCas\":%.4f,"
             "\"kCar\":%.4f,\"kDes\":%.4f,",
             contadores.kwhPainelTotal, contadores.kwhCasaTotal,
             contadores.kwhCargaTotal,  contadores.kwhDescargaTotal);
    j += tempBuf;

    // Sistema
    snprintf(tempBuf, sizeof(tempBuf),
             "\"ram\":%u,\"ip\":\"%s\","
             "\"modo\":\"%s\",\"up\":%lu,",
             ram, ip.c_str(),
             modoAP ? "AP" : "WiFi",
             millis() / 1000);
    j += tempBuf;

    // Previsão solar
    snprintf(tempBuf, sizeof(tempBuf),
             "\"prevOk\":%d,\"prevBloqRA\":%d,\"prevBloqLE\":%d,",
             estadoPrev.dadosValidos ? 1 : 0,
             estadoPrev.bloqRedeAlta ? 1 : 0,
             estadoPrev.bloqLampExt  ? 1 : 0);
    j += tempBuf;

    // Motivos (escapa aspas)
    j += "\"prevMRA\":\"";
    for (int c = 0; estadoPrev.motivoRedeAlta[c]; c++) {
        char ch = estadoPrev.motivoRedeAlta[c];
        if (ch == '"') j += "\\\""; else j += ch;
    }
    j += "\",\"prevMLE\":\"";
    for (int c = 0; estadoPrev.motivoLampExt[c]; c++) {
        char ch = estadoPrev.motivoLampExt[c];
        if (ch == '"') j += "\\\""; else j += ch;
    }
    j += "\",";

    snprintf(tempBuf, sizeof(tempBuf),
             "\"prevAmanha\":%.0f,\"prevRest\":%.0f,\"prevAcerto\":%.1f",
             estadoPrev.totalAmanha,
             estadoPrev.totalRestanteHoje,
             estadoPrev.taxaAcerto);
    j += tempBuf;
    j += "}";

    server.send(200, "application/json", j);
}

// ============================================================
// API: /api/rede - JSON da rede elétrica
// ============================================================
void handleApiRede() {
    if (!checarAuth()) return;

    // ── Reutiliza cálculos já feitos ──
    float soc = 0.0f;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float eD     = cfgBat.energiaDisponivel;

    float whMinRB = eD * (cfgRede.percRedeBaixa / 100.0f);
    float whMinRA = eD * (cfgRede.percRedeAlta  / 100.0f);
    float whMinLE = eD * (cfgRede.percLampExt   / 100.0f);
    float whMinLI = eD * (cfgRede.percLampInt   / 100.0f);

    // ── Tempos restantes para corte ──
    unsigned long agora = millis();
    int tRestRB = 0, tRestRA = 0, tRestLE = 0, tRestLI = 0;

    if (estadoRede.aguardandoRedeBaixa)
        tRestRB = max(0L, (long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoRedeBaixa))) / 1000;
    if (estadoRede.aguardandoRedeAlta)
        tRestRA = max(0L, (long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoRedeAlta))) / 1000;
    if (estadoRede.aguardandoLampExt)
        tRestLE = max(0L, (long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoLampExt))) / 1000;
    if (estadoRede.aguardandoLampInt)
        tRestLI = max(0L, (long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoLampInt))) / 1000;

    // ── Hora atual formatada ──
    char horaAtualStr[6] = "--:--";
    if (ntpSincronizado) {
        time_t now = time(nullptr);
        struct tm* ti = localtime(&now);
        snprintf(horaAtualStr, sizeof(horaAtualStr),
                 "%02d:%02d", ti->tm_hour, ti->tm_min);
    }

    // ══════════════════════════════════════════════════════
    // MONTA JSON
    // ══════════════════════════════════════════════════════
    String j = "{";

    // ── Percentuais configurados ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"pRB\":%.1f,\"pRA\":%.1f,\"pLE\":%.1f,\"pLI\":%.1f,",
             cfgRede.percRedeBaixa, cfgRede.percRedeAlta,
             cfgRede.percLampExt,   cfgRede.percLampInt);
    j += tempBuf;

    // ── Wh mínimos calculados ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"wRB\":%.0f,\"wRA\":%.0f,\"wLE\":%.0f,\"wLI\":%.0f,",
             whMinRB, whMinRA, whMinLE, whMinLI);
    j += tempBuf;

    // ── Estados das cargas ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"stRB\":%d,\"stRA\":%d,\"stLE\":%d,\"stLI\":%d,",
             estadoRede.redeBaixaAtiva   ? 1 : 0,
             estadoRede.redeAltaAtiva    ? 1 : 0,
             estadoRede.lampExternaAtiva ? 1 : 0,
             estadoRede.lampInternaAtiva ? 1 : 0);
    j += tempBuf;

    // ── Flags de aguardo (contagem 1 min) ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"agRB\":%d,\"agRA\":%d,\"agLE\":%d,\"agLI\":%d,",
             estadoRede.aguardandoRedeBaixa ? 1 : 0,
             estadoRede.aguardandoRedeAlta  ? 1 : 0,
             estadoRede.aguardandoLampExt   ? 1 : 0,
             estadoRede.aguardandoLampInt   ? 1 : 0);
    j += tempBuf;

    // ── Tempos restantes ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"tRB\":%d,\"tRA\":%d,\"tLE\":%d,\"tLI\":%d,",
             tRestRB, tRestRA, tRestLE, tRestLI);
    j += tempBuf;

    // ── Modo e estado lâmpada externa ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"mLE\":%d,\"leHor\":%d,\"lePain\":%d,\"leBloq\":%d,",
             cfgRede.modoLampExt,
             estadoRede.lampExtPorHorario  ? 1 : 0,
             estadoRede.lampExtPorPainel   ? 1 : 0,
             estadoRede.lampExtBloqEnergia ? 1 : 0);
    j += tempBuf;

    // ── Horários configurados ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"ligH\":%d,\"ligM\":%d,\"dslH\":%d,\"dslM\":%d,",
             cfgRede.ligarHora, cfgRede.ligarMin,
             cfgRede.desligarHora, cfgRede.desligarMin);
    j += tempBuf;

    // ── Estado inversor ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"painelAtivo\":%d,\"invLig\":%d,\"invSt\":%d,\"invProt\":%d,",
             (sensores.correntePainel > 0.5f) ? 1 : 0,
             estadoInv.invLigado   ? 1 : 0,
             estadoInv.statusInv   ? 1 : 0,
             estadoInv.emProtecao  ? 1 : 0);
    j += tempBuf;

    // ── Contadores de eventos da rede ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"cRB\":%lu,\"cRA\":%lu,\"cLE\":%lu,\"cLI\":%lu,"
             "\"cLigH\":%lu,\"cPain\":%lu,",
             (unsigned long)estadoRede.totalCortesRedeBaixa,
             (unsigned long)estadoRede.totalCortesRedeAlta,
             (unsigned long)estadoRede.totalCortesLampExt,
             (unsigned long)estadoRede.totalCortesLampInt,
             (unsigned long)estadoRede.totalLigadasHorario,
             (unsigned long)estadoRede.totalLigadasPainel);
    j += tempBuf;

    // ── Energia e SoC ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"soc\":%.1f,\"whDisp\":%.0f,\"whTotal\":%.0f,"
             "\"horaAtual\":\"%s\",\"ntp\":%d,\"config\":%d,",
             soc, whDisp, eD,
             horaAtualStr,
             ntpSincronizado     ? 1 : 0,
             cfgRede.configurado ? 1 : 0);
    j += tempBuf;

    // ── NOVOS V7.0: Diagnóstico do inversor ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"bloqRA\":%d,\"bloqRB\":%d,"
             "\"confRA\":%d,\"confRB\":%d,"
             "\"faseDiag\":%d,"
             "\"curtosRA\":%lu,\"curtosRB\":%lu,"
             "\"totalProt\":%lu,\"totalRein\":%lu",
             estadoInv.redeAltaBloqCurto       ? 1 : 0,
             estadoInv.redeBaixaBloqCurto      ? 1 : 0,
             estadoInv.aguardandoConfirmacaoRA  ? 1 : 0,
             estadoInv.aguardandoConfirmacaoRB  ? 1 : 0,
             (int)estadoInv.faseTesteDiagnostico,
             (unsigned long)estadoInv.totalCurtosRA,
             (unsigned long)estadoInv.totalCurtosRB,
             (unsigned long)estadoInv.totalProtecoes,
             (unsigned long)estadoInv.totalReiniciamentos);
    j += tempBuf;

    j += "}";

    server.send(200, "application/json", j);
}

// ============================================================
// API: /api/previsao - JSON dados de previsão
// ============================================================
void handleApiPrevisao() {
    if (!checarAuth()) return;

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int horaAtual = ti->tm_hour;
    int idxAtual  = horaAtual - estadoPrev.horaOffsetAPI;
    if (idxAtual < 0) idxAtual = 0;

    String j = "{";
    snprintf(tempBuf, sizeof(tempBuf),
             "\"ok\":%d,\"kwp\":%.2f,\"acerto\":%.1f,"
             "\"fatorGlobal\":%.3f,\"consMedia\":%.0f,",
             estadoPrev.dadosValidos ? 1 : 0,
             cfgPrev.kwpInstalado,
             estadoPrev.taxaAcerto,
             estadoPrev.fatorHorario[horaAtual],
             estadoPrev.consumoMedioDia);
    j += tempBuf;

    snprintf(tempBuf, sizeof(tempBuf),
             "\"radAgora\":%.0f,\"nuvAgora\":%.0f,"
             "\"chuvaAgora\":%d,\"gerEstAgora\":%.0f,",
             estadoPrev.radiacaoHora[idxAtual],
             estadoPrev.nuvensPct[idxAtual],
             (int)estadoPrev.chuvaProb[idxAtual],
             estadoPrev.geracaoEstHora[idxAtual]);
    j += tempBuf;

    snprintf(tempBuf, sizeof(tempBuf),
             "\"gerRealAgora\":%.0f,\"restHoje\":%.0f,\"totalAmanha\":%.0f,",
             sensores.potenciaPainel,
             estadoPrev.totalRestanteHoje,
             estadoPrev.totalAmanha);
    j += tempBuf;

    snprintf(tempBuf, sizeof(tempBuf),
             "\"bloqRA\":%d,\"bloqLE\":%d,"
             "\"mRA\":\"%s\",\"mLE\":\"%s\",",
             estadoPrev.bloqRedeAlta ? 1 : 0,
             estadoPrev.bloqLampExt  ? 1 : 0,
             estadoPrev.motivoRedeAlta,
             estadoPrev.motivoLampExt);
    j += tempBuf;

    // Próximas 12 horas
    j += "\"horas\":[";
    for (int i = 0; i < 12; i++) {
        int idx = idxAtual + i;
        if (idx >= 48) break;
        if (i > 0) j += ",";
        snprintf(tempBuf, sizeof(tempBuf),
                 "{\"h\":%d,\"r\":%.0f,\"n\":%.0f,\"g\":%.0f,\"c\":%d}",
                 (horaAtual + i) % 24,
                 estadoPrev.radiacaoHora[idx],
                 estadoPrev.nuvensPct[idx],
                 estadoPrev.geracaoEstHora[idx],
                 (int)estadoPrev.chuvaProb[idx]);
        j += tempBuf;
    }
    j += "],";

    // Amanhã
    int inicioAmanha = 24 - horaAtual;
    j += "\"amanha\":[";
    for (int i = 0; i < 24; i++) {
        int idx = inicioAmanha + i;
        if (idx >= 48) break;
        if (i > 0) j += ",";
        snprintf(tempBuf, sizeof(tempBuf),
                 "{\"h\":%d,\"r\":%.0f,\"n\":%.0f,\"g\":%.0f,\"c\":%d}",
                 i,
                 estadoPrev.radiacaoHora[idx],
                 estadoPrev.nuvensPct[idx],
                 estadoPrev.geracaoEstHora[idx],
                 (int)estadoPrev.chuvaProb[idx]);
        j += tempBuf;
    }
    j += "],";

    // Fatores de calibração
    j += "\"fatores\":[";
    for (int h = 0; h < 24; h++) {
        if (h > 0) j += ",";
        snprintf(tempBuf, sizeof(tempBuf), "%.3f",
                 estadoPrev.fatorHorario[h]);
        j += tempBuf;
    }
    j += "],";

    snprintf(tempBuf, sizeof(tempBuf),
             "\"tsAtualizacao\":%lu,\"erros\":%d,"
             "\"horaAtual\":%d,\"config\":%d",
             estadoPrev.tsUltimaAtualizacao / 1000,
             (int)estadoPrev.errosConsecutivos,
             horaAtual,
             cfgPrev.configurado ? 1 : 0);
    j += tempBuf;
    j += "}";

    server.send(200, "application/json", j);
}

// ============================================================
// Atualização OTA via Web
// ============================================================

// Resposta final após o upload (sucesso ou falha) - handler POST
void handleUpdateResultado() {
    if (!logado) {
        server.send(403, "text/plain", "Nao autenticado");
        return;
    }

    bool ok = !Update.hasError();

    // ── Monta resposta ──
    String resposta = ok ? "OK" : Update.errorString();

    server.sendHeader("Connection", "close");
    server.send(ok ? 200 : 500, "text/plain", resposta);

    if (ok) {
        adicionarLog("OTA: Reiniciando em 2s...");
        delay(2000);
        ESP.restart();
    } else {
        snprintf(tempBuf, sizeof(tempBuf),
                 "OTA: Falha - %s", Update.errorString());
        adicionarLog(tempBuf);
    }
}

// Processa os chunks do arquivo enquanto ele chega - handler de upload
void handleUpdateUpload() {
    if (!logado) {
        server.send(403, "text/plain", "Nao autenticado");
        return;
    }

    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        blinkAtivo = false;
        digitalWrite(LED_AZUL, HIGH);

        snprintf(tempBuf, sizeof(tempBuf),
                 "OTA: Recebendo %s (%.0f KB livres na flash)",
                 upload.filename.c_str(),
                 (float)ESP.getFreeSketchSpace() / 1024.0f);
        adicionarLog(tempBuf);

        // ── Verifica espaço disponível antes de iniciar ──
        uint32_t espacoLivre = ESP.getFreeSketchSpace();
        if (espacoLivre < 100000) {
            adicionarLog("OTA: ERRO - Espaco insuficiente na flash!");
            adicionarLog("Mude Partition Scheme no Arduino IDE:");
            adicionarLog("Minimal SPIFFS (1.9MB APP with OTA)");
            return;
        }

        // ── Inicia update com tamanho do arquivo ──
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            String err = "OTA: Falha ao iniciar - ";
            err += Update.errorString();
            adicionarLog(err.c_str());
            Update.printError(Serial);
            return;
        }

        adicionarLog("OTA: Upload iniciado com sucesso!");

    } else if (upload.status == UPLOAD_FILE_WRITE) {
        // ── Escreve chunk ──
        if (Update.hasError()) return; // para se já teve erro

        size_t written = Update.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
            String err = "OTA: Erro ao escrever chunk - ";
            err += Update.errorString();
            adicionarLog(err.c_str());
            Update.printError(Serial);
        }

    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.hasError()) {
            String err = "OTA: Falha - ";
            err += Update.errorString();
            adicionarLog(err.c_str());
            Update.printError(Serial);
            ledBlink();
            return;
        }

        if (Update.end(true)) {
            snprintf(tempBuf, sizeof(tempBuf),
                     "OTA: Sucesso! %u bytes gravados",
                     (unsigned)upload.totalSize);
            adicionarLog(tempBuf);
        } else {
            String err = "OTA: Erro ao finalizar - ";
            err += Update.errorString();
            adicionarLog(err.c_str());
            Update.printError(Serial);
        }
        ledBlink();

    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        adicionarLog("OTA: Upload abortado");
        ledBlink();
    }
}

// ============================================================
// PÁGINA PRINCIPAL: /
// ============================================================
void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    adicionarLog("Painel principal acessado");
    logRAMSeNecessario();

    bool temBat = existeBateriaConfig();
    String conteudo = "";

    conteudo += "<div id='alertas'></div>";

    // ── Painel Solar ──
    conteudo += "<h3>&#9728; Painel Solar</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Corrente</div>"
                "<div class='stat-val stat-ok' id='iP'>--</div>"
                "<div class='stat-unit'>A</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Pot&ecirc;ncia</div>"
                "<div class='stat-val stat-ok' id='wP'>--</div>"
                "<div class='stat-unit'>W</div></div>";
    conteudo += "</div>";

    // ── Consumo da Casa ──
    conteudo += "<h3>&#127968; Consumo da Casa</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Corrente</div>"
                "<div class='stat-val stat-warn' id='iC'>--</div>"
                "<div class='stat-unit'>A</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Pot&ecirc;ncia</div>"
                "<div class='stat-val stat-warn' id='wC'>--</div>"
                "<div class='stat-unit'>W</div></div>";
    conteudo += "</div>";

    // ── Bateria ──
    conteudo += "<h3>&#128267; Bateria</h3>";
    if (!temBat) {
        conteudo += "<div class='msg-warn'>&#9888; Bateria n&atilde;o configurada. "
                    "<a href='/bateria' style='color:#fff'>Configurar agora</a></div>";
    }
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Corrente</div>"
                "<div class='stat-val' id='iB'>--</div>"
                "<div class='stat-unit'>A</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Pot&ecirc;ncia</div>"
                "<div class='stat-val' id='wB'>--</div>"
                "<div class='stat-unit'>W</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Tens&atilde;o Est.</div>"
                "<div class='stat-val' id='vB'>--</div>"
                "<div class='stat-unit'>V</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Estado</div>"
                "<div class='stat-val' id='stB'>--</div>"
                "<div class='stat-unit' id='stBu'></div></div>";
    conteudo += "</div>";

    // Barra SoC
    conteudo += "<div style='margin:8px 0;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-bottom:3px;'>"
                "<span>Carga da Bateria</span>"
                "<span id='socPct'>--%</span></div>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:4px;height:14px;'>"
                "<div id='socBar' style='height:100%;border-radius:4px;"
                "width:0%;background:#66bb6a;transition:width 1s;'></div>"
                "</div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-top:3px;'>"
                "<span>Cap. Total: <b id='capTot'>--</b> Wh</span>"
                "<span>Dispon&iacute;vel: <b id='whDisp'>--</b> Wh</span>"
                "</div></div>";

    // Tempo estimado + ciclos
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label' id='tLbl'>Tempo</div>"
                "<div class='stat-val' id='tStr' style='font-size:14px;'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Ciclos</div>"
                "<div class='stat-val' id='ciclos'>--</div></div>";
    conteudo += "</div>";

    // ── Status Inversor + Rede ──
    conteudo += "<h3>&#9889; Inversor e Rede</h3>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:10px;font-size:11px;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#9889; Inversor</span>"
                "<span id='stInv' style='font-weight:bold;'>--</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#128267; Rede Baixa</span>"
                "<span id='stRB' style='font-weight:bold;'>--</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#9889; Rede Alta</span>"
                "<span id='stRA' style='font-weight:bold;'>--</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#128161; Lamp. Externa</span>"
                "<span id='stLE' style='font-weight:bold;'>--</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;'>"
                "<span style='color:#78909c;'>&#128161; Lamp. Interna</span>"
                "<span id='stLI' style='font-weight:bold;'>--</span></div>";
    conteudo += "</div>";

    // ── Resumo Previsão ──
    conteudo += "<div id='resumoPrev' style='display:none;'>";
    conteudo += "<h3>&#9728; Previs&atilde;o Solar</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Restante Hoje</div>"
                "<div class='stat-val stat-ok' id='prevRest'>--</div>"
                "<div class='stat-unit'>Wh</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Amanh&atilde;</div>"
                "<div class='stat-val' id='prevAmanha'>--</div>"
                "<div class='stat-unit'>Wh</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Acerto</div>"
                "<div class='stat-val' id='prevAcerto'>--</div>"
                "<div class='stat-unit'>%</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Detalhes</div>"
                "<div class='stat-val' style='font-size:11px;'>"
                "<a href='/previsao' style='color:#4fc3f7;'>&#9728; Ver</a>"
                "</div></div>";
    conteudo += "</div></div>";

    // ── Contadores ──
    conteudo += "<h3>&#128200; Contadores Totais</h3>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:10px;font-size:11px;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#9728; Painel Gerado</span>"
                "<span class='contador' id='kPai'>------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#127968; Casa Consumido</span>"
                "<span class='contador' id='kCas'>------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#128267;&#8679; Bat. Carregado</span>"
                "<span class='contador' id='kCar'>------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;'>"
                "<span style='color:#78909c;'>&#128267;&#8681; Bat. Descarregado</span>"
                "<span class='contador' id='kDes'>------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "</div>";

    // ── Sistema ──
    conteudo += "<h3>&#128297; Sistema</h3>";
    conteudo += "<div class='info' style='font-size:11px;'>"
                "&#128190; RAM: <b id='ramVal'>--</b> KB &nbsp;|&nbsp; "
                "&#127760; IP: <b id='ipVal'>--</b> &nbsp;|&nbsp; "
                "&#9201; Up: <b id='upVal'>--</b></div>";

    // ── Botões ──
    conteudo += "<div class='flex'>";
    conteudo += "<button onclick='location.href=\"/bateria\"' "
                "class='btn-green' style='font-size:12px;'>"
                "&#128267; Bateria</button>";
    conteudo += "<button onclick='location.href=\"/rede\"' "
                "style='background:#ef6c00;font-size:12px;'>"
                "&#128161; Rede</button>";
    conteudo += "<button onclick='location.href=\"/previsao\"' "
                "style='background:#1565c0;font-size:12px;'>"
                "&#9728; Prev</button>";
    conteudo += "<button onclick='location.href=\"/historico\"' "
                "style='background:#1e3a5f;font-size:12px;'>"
                "&#128202; Hist</button>";
    conteudo += "<button onclick='location.href=\"/logout\"' "
                "class='btn-red' style='font-size:12px;'>"
                "&#128275; Sair</button>";
    conteudo += "</div>";

    // ── Script ──
    String script = "";
    script += "var capTotal=" + String(cfgBat.capacidadeTotal, 1) + ";";

    script += F("function fmt(v,d){return isNaN(v)?'--':v.toFixed(d);}"
                "function fmtCont(v){"
                "var s=v.toFixed(4);"
                "var p=s.indexOf('.');"
                "var i=s.substring(0,p);"
                "var d=s.substring(p);"
                "while(i.length<6)i='0'+i;"
                "return i+d;}"
                "function corSoc(s){"
                "if(s>=60)return'#66bb6a';"
                "if(s>=30)return'#ffa726';"
                "return'#ef5350';}"
                "function stTexto(v,on,off){"
                "return v?on:off;}"

                "function atualizarDados(){"
                "var x=new XMLHttpRequest();"
                "x.onreadystatechange=function(){"
                "if(x.readyState==4&&x.status==200){"
                "try{var d=JSON.parse(x.responseText);"

                // Painel
                "document.getElementById('iP').textContent=fmt(d.iP,2);"
                "document.getElementById('wP').textContent=fmt(d.wP,1);"

                // Casa
                "document.getElementById('iC').textContent=fmt(d.iC,2);"
                "document.getElementById('wC').textContent=fmt(d.wC,1);"

                // Bateria
                "var iBel=document.getElementById('iB');"
                "iBel.textContent=fmt(Math.abs(d.iB),2);"
                "iBel.className='stat-val '+(d.iB>=0?'stat-ok':'stat-err');"
                "document.getElementById('wB').textContent=fmt(Math.abs(d.wB),1);"
                "document.getElementById('vB').textContent=fmt(d.vB,2);"
                "var stEl=document.getElementById('stB');"
                "stEl.textContent=d.car?'Carregando':'Descarregando';"
                "stEl.className='stat-val '+(d.car?'stat-ok':'stat-err');"

                // SoC
                "document.getElementById('socPct').textContent=fmt(d.soc,1)+'%';"
                "var bar=document.getElementById('socBar');"
                "bar.style.width=d.soc+'%';"
                "bar.style.background=corSoc(d.soc);"
                "document.getElementById('capTot').textContent=fmt(capTotal,0);"
                "document.getElementById('whDisp').textContent=fmt(d.whD,0);"

                // Tempo e ciclos
                "document.getElementById('tStr').textContent=d.tStr||'--';"
                "document.getElementById('tLbl').textContent=d.tLbl||'Tempo';"
                "document.getElementById('ciclos').textContent=fmt(d.cic,2);"

                // Inversor e Rede
                "var invEl=document.getElementById('stInv');"
                "invEl.textContent=d.invProt?'⚠ PROTECAO':(d.invSt?'✓ OK':'✗ SEM SINAL');"
                "invEl.style.color=d.invProt?'#ef5350':(d.invSt?'#66bb6a':'#ffa726');"
                "var rbEl=document.getElementById('stRB');"
                "rbEl.textContent=d.stRB?'✓ LIGADA':'✗ DESLIGADA';"
                "rbEl.style.color=d.stRB?'#66bb6a':'#ef5350';"
                "var raEl=document.getElementById('stRA');"
                "raEl.textContent=d.stRA?'✓ LIGADA':'✗ DESLIGADA';"
                "raEl.style.color=d.stRA?'#66bb6a':'#546e7a';"
                "var leEl=document.getElementById('stLE');"
                "leEl.textContent=d.stLE?'✓ LIGADA':'✗ DESLIGADA';"
                "leEl.style.color=d.stLE?'#66bb6a':'#546e7a';"
                "var liEl=document.getElementById('stLI');"
                "liEl.textContent=d.stLI?'✓ LIGADA':'✗ DESLIGADA';"
                "liEl.style.color=d.stLI?'#66bb6a':'#546e7a';"

                // Contadores
                "document.getElementById('kPai').textContent=fmtCont(d.kPai);"
                "document.getElementById('kCas').textContent=fmtCont(d.kCas);"
                "document.getElementById('kCar').textContent=fmtCont(d.kCar);"
                "document.getElementById('kDes').textContent=fmtCont(d.kDes);"

                // Sistema
                "document.getElementById('ramVal').textContent=d.ram;"
                "document.getElementById('ipVal').textContent=d.ip;"
                "var up=d.up;"
                "var uh=Math.floor(up/3600);"
                "var um=Math.floor((up%3600)/60);"
                "var us=up%60;"
                "document.getElementById('upVal').textContent="
                "uh+'h'+pad(um)+'m'+pad(us)+'s';"

                // Previsão
                "var rPrev=document.getElementById('resumoPrev');"
                "if(d.prevOk&&rPrev){"
                "rPrev.style.display='block';"
                "document.getElementById('prevRest').textContent=d.prevRest.toFixed(0);"
                "document.getElementById('prevAmanha').textContent=d.prevAmanha.toFixed(0);"
                "var acEl=document.getElementById('prevAcerto');"
                "acEl.textContent=d.prevAcerto.toFixed(1);"
                "acEl.className='stat-val '+"
                "(d.prevAcerto>=70?'stat-ok':(d.prevAcerto>=40?'stat-warn':'stat-err'));}"

                // Alertas
                "var al='';"
                "if(d.aPi)al+='<div class=\"msg-err\">&#9888; Sensor Painel INVERTIDO!</div>';"
                "if(d.aCi)al+='<div class=\"msg-err\">&#9888; Sensor Casa INVERTIDO!</div>';"
                "if(d.bkp)al+='<div class=\"msg-warn\">&#128308; BKP: Bateria critica!</div>';"
                "if(d.invProt)al+='<div class=\"msg-warn\">&#9889; Inversor em PROTECAO!</div>';"
                "if(d.soc<20&&!d.car)al+='<div class=\"msg-warn\">"
                "&#9889; Bateria abaixo de 20%!</div>';"
                "if(d.prevBloqRA)al+='<div class=\"msg-warn\">"
                "&#9728; Rede Alta desativada: '+d.prevMRA+'</div>';"
                "if(d.prevBloqLE)al+='<div class=\"msg-warn\">"
                "&#9728; Lamp.Ext desativada: '+d.prevMLE+'</div>';"
                "if(d.prevOk&&!d.prevBloqRA&&!d.prevBloqLE&&d.prevAmanha>0){"
                "al+='<div class=\"msg-ok\" style=\"font-size:10px;\">"
                "&#9728; Prev.amanha: '+d.prevAmanha.toFixed(0)+"
                "'Wh | Acerto: '+d.prevAcerto.toFixed(1)+'%</div>';}"
                "document.getElementById('alertas').innerHTML=al;"

                "}catch(e){}"
                "}};"
                "x.open('GET','/api/dados',true);x.send();}"
                "setInterval(atualizarDados,2000);"
                "atualizarDados();");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Painel", conteudo, script));
}

// ============================================================
// PÁGINA: /bateria
// ============================================================
void handleBateria() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "salvar") {
            float vnom  = server.arg("vnom").toFloat();
            float ah    = server.arg("ah").toFloat();
            float pdesc = server.arg("pdesc").toFloat() / 100.0f;
            float vmin  = server.arg("vmin").toFloat();

            if      (vnom < 10.0f || vnom > 60.0f)
                msgHtml = "<div class='msg-err'>&#10007; Tens&atilde;o nominal inv&aacute;lida (10-60V)</div>";
            else if (ah < 10.0f || ah > 2000.0f)
                msgHtml = "<div class='msg-err'>&#10007; Amperagem inv&aacute;lida (10-2000Ah)</div>";
            else if (pdesc < 0.1f || pdesc > 1.0f)
                msgHtml = "<div class='msg-err'>&#10007; Descarga segura inv&aacute;lida (10-100%)</div>";
            else if (vmin < 5.0f || vmin >= vnom)
                msgHtml = "<div class='msg-err'>&#10007; Tens&atilde;o m&iacute;nima inv&aacute;lida</div>";
            else {
                cfgBat.tensaoNominal      = vnom;
                cfgBat.amperagem          = ah;
                cfgBat.percDescargaSegura = pdesc;
                cfgBat.tensaoMinima       = vmin;
                cfgBat.capacidadeTotal    = vnom * ah;
                cfgBat.energiaDisponivel  = cfgBat.capacidadeTotal * pdesc;
                if      (vnom <= 13.0f) cfgBat.tensaoMaxima = 14.0f;
                else if (vnom <= 26.0f) cfgBat.tensaoMaxima = 28.0f;
                else                    cfgBat.tensaoMaxima = vnom * 1.094f;

                salvarConfigBateria();
                estadoBat.ahMaximos     = ah * pdesc;
                estadoBat.ahDisponiveis = estadoBat.ahMaximos;
                estadoBat.ciclos        = 0;
                estadoBat.ciclosParciais= 0;
                estadoBat.carregada100  = true;
                salvarEstadoBateria();

                snprintf(tempBuf, sizeof(tempBuf),
                         "Bateria config: %.1fV %.0fAh %.0f%%",
                         vnom, ah, pdesc * 100.0f);
                adicionarLog(tempBuf);
                msgHtml = "<div class='msg-ok'>&#10003; Bateria configurada!</div>";
            }
        } else if (acao == "resetciclos") {
            estadoBat.ciclos         = 0;
            estadoBat.ciclosParciais = 0;
            salvarEstadoBateria();
            adicionarLog("Ciclos resetados");
            msgHtml = "<div class='msg-ok'>&#10003; Ciclos resetados!</div>";
        } else if (acao == "resetah") {
            estadoBat.ahDisponiveis = estadoBat.ahMaximos;
            estadoBat.carregada100  = true;
            salvarEstadoBateria();
            adicionarLog("Estado bateria resetado para 100%");
            msgHtml = "<div class='msg-ok'>&#10003; Bateria marcada como 100%!</div>";
        }
    }

    float soc    = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);
    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;

    String conteudo = msgHtml;

    if (existeBateriaConfig()) {
        conteudo += "<h3>&#128202; Status Atual</h3>";
        conteudo += "<div class='stat-grid'>";
        conteudo += "<div class='stat-box'><div class='stat-label'>Tens&atilde;o Est.</div>"
                    "<div class='stat-val stat-ok'>"
                    + String(sensores.tensaoBateriaEst, 2)
                    + "</div><div class='stat-unit'>V</div></div>";
        conteudo += "<div class='stat-box'><div class='stat-label'>Carga (SoC)</div>"
                    "<div class='stat-val'>" + String(soc, 1)
                    + "</div><div class='stat-unit'>%</div></div>";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", cfgBat.capacidadeTotal);
        conteudo += "<div class='stat-box'><div class='stat-label'>Cap. Total</div>"
                    "<div class='stat-val'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>Wh</div></div>";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", whDisp);
        conteudo += "<div class='stat-box'><div class='stat-label'>Dispon&iacute;vel</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>Wh</div></div>";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f", cfgBat.energiaDisponivel);
        conteudo += "<div class='stat-box'><div class='stat-label'>Seg. Total</div>"
                    "<div class='stat-val'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>Wh</div></div>";
        snprintf(tempBuf, sizeof(tempBuf), "%.2f",
                 estadoBat.ciclos + estadoBat.ciclosParciais);
        conteudo += "<div class='stat-box'><div class='stat-label'>Ciclos</div>"
                    "<div class='stat-val stat-warn'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>ciclos</div></div>";
        conteudo += "</div>";

        String corBar = (soc >= 60) ? "#66bb6a" : (soc >= 30) ? "#ffa726" : "#ef5350";
        conteudo += "<div style='background:#111;border:1px solid #333;"
                    "border-radius:4px;height:18px;margin:8px 0;'>"
                    "<div style='height:100%;border-radius:4px;width:"
                    + String(soc, 0) + "%;background:" + corBar + ";'></div></div>";

        conteudo += "<div class='info' style='font-size:11px;'>";
        conteudo += "&#9889; " + String(cfgBat.tensaoNominal, 1)
                  + "V &nbsp;|&nbsp; " + String(cfgBat.amperagem, 0)
                  + "Ah &nbsp;|&nbsp; Desc: "
                  + String(cfgBat.percDescargaSegura * 100.0f, 0)
                  + "% &nbsp;|&nbsp; Vmin: "
                  + String(cfgBat.tensaoMinima, 1) + "V</div>";

        conteudo += "<div class='flex'>";
        conteudo += "<form method='POST' action='/bateria' style='flex:1'>"
                    "<input type='hidden' name='acao' value='resetah'>"
                    "<button type='submit' class='btn-green' style='font-size:11px;'>"
                    "&#128267; Marcar 100%</button></form>";
        conteudo += "<form method='POST' action='/bateria' style='flex:1'>"
                    "<input type='hidden' name='acao' value='resetciclos'>"
                    "<button type='submit' class='btn-orange' style='font-size:11px;'>"
                    "&#8635; Zerar Ciclos</button></form>";
        conteudo += "</div>";
    }

    conteudo += "<h3>&#9881; Configurar Bateria LiFePO4</h3>";
    conteudo += "<form method='POST' action='/bateria'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";

    conteudo += "<label>Tens&atilde;o Nominal (V):</label>";
    conteudo += "<select name='vnom'>";
    float opts[] = {12.8f, 25.6f, 38.4f, 51.2f};
    const char* optNomes[] = {"12V 4S","24V 8S","36V 12S","48V 16S"};
    for (int i = 0; i < 4; i++) {
        conteudo += "<option value='" + String(opts[i], 1) + "'";
        if (fabsf(cfgBat.tensaoNominal - opts[i]) < 0.1f) conteudo += " selected";
        conteudo += ">" + String(opts[i], 1) + "V (" + optNomes[i] + ")</option>";
    }
    conteudo += "</select>";

    conteudo += "<label>Capacidade (Ah):</label>";
    conteudo += "<input type='number' name='ah' min='10' max='2000' step='1' required "
                "value='" + String((int)cfgBat.amperagem) + "' placeholder='Ex: 100'>";

    conteudo += "<label>Descarga Segura (%) - recomendado 80%:</label>";
    conteudo += "<input type='number' name='pdesc' min='10' max='100' step='1' required "
                "value='" + String((int)(cfgBat.percDescargaSegura * 100.0f))
                + "' placeholder='Ex: 80'>";

    conteudo += "<label>Tens&atilde;o M&iacute;nima BKP (V):</label>";
    conteudo += "<input type='number' name='vmin' min='5' max='55' step='0.1' required "
                "value='" + String(cfgBat.tensaoMinima, 1) + "' placeholder='Ex: 12.0'>";

    conteudo += "<div class='info' style='font-size:11px;margin-top:8px;'>"
                "&#9432; LiFePO4: 12.8V &rarr; 14.0V &nbsp;|&nbsp; "
                "25.6V &rarr; 28.0V<br>"
                "Descarga segura recomendada: 80%</div>";

    conteudo += "<button type='submit' class='btn-green'>"
                "&#128190; Salvar Configura&ccedil;&atilde;o</button>";
    conteudo += "</form>";
    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Bateria", conteudo));
}

// ============================================================
// PÁGINA: /rede
// ============================================================
void handleRede() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "salvar") {
            float pRB = server.arg("perc_redbaixa").toFloat();
            float pRA = server.arg("perc_redealta").toFloat();
            float pLE = server.arg("perc_lampext").toFloat();
            float pLI = server.arg("perc_lampint").toFloat();
            uint8_t mLE = (uint8_t)server.arg("modo_lampext").toInt();
            uint8_t lH  = (uint8_t)server.arg("ligar_hora").toInt();
            uint8_t lM  = (uint8_t)server.arg("ligar_min").toInt();
            uint8_t dH  = (uint8_t)server.arg("desl_hora").toInt();
            uint8_t dM  = (uint8_t)server.arg("desl_min").toInt();

            bool ok = true;
            if      (pRB < 0 || pRB > 50)
                { msgHtml="<div class='msg-err'>&#10007; Rede Baixa: 0-50%</div>"; ok=false; }
            else if (pRA < 0 || pRA > 80)
                { msgHtml="<div class='msg-err'>&#10007; Rede Alta: 0-80%</div>"; ok=false; }
            else if (pLE < 0 || pLE > 80)
                { msgHtml="<div class='msg-err'>&#10007; Lamp.Ext: 0-80%</div>"; ok=false; }
            else if (pLI < 0 || pLI > 50)
                { msgHtml="<div class='msg-err'>&#10007; Lamp.Int: 0-50%</div>"; ok=false; }
            else if (pRA < pLE || pRA < pLI)
                { msgHtml="<div class='msg-err'>&#10007; Rede Alta deve ser "
                  "&gt; Lamp.Ext e Lamp.Int</div>"; ok=false; }
            else if (mLE == 1 && !ntpSincronizado)
                { msgHtml="<div class='msg-err'>&#10007; Modo Hor&aacute;rio "
                  "requer NTP sincronizado</div>"; ok=false; }

            if (ok) {
                cfgRede.percRedeBaixa = pRB;
                cfgRede.percRedeAlta  = pRA;
                cfgRede.percLampExt   = pLE;
                cfgRede.percLampInt   = pLI;
                cfgRede.modoLampExt   = mLE;
                cfgRede.ligarHora     = lH;
                cfgRede.ligarMin      = lM;
                cfgRede.desligarHora  = dH;
                cfgRede.desligarMin   = dM;
                cfgRede.configurado   = true;
                salvarConfigRede();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: RB=%.0f%% RA=%.0f%% LE=%.0f%% LI=%.0f%%",
                         pRB, pRA, pLE, pLI);
                adicionarLog(tempBuf);
                msgHtml = "<div class='msg-ok'>&#10003; Rede configurada!</div>";
            }

        } else if (acao == "reset_stats") {
            estadoRede.totalCortesRedeBaixa = 0;
            estadoRede.totalCortesRedeAlta  = 0;
            estadoRede.totalCortesLampExt   = 0;
            estadoRede.totalCortesLampInt   = 0;
            estadoRede.totalLigadasHorario  = 0;
            estadoRede.totalLigadasPainel   = 0;
            salvarEstadoRede();
            adicionarLog("Stats rede zeradas");
            msgHtml = "<div class='msg-ok'>&#10003; Zerado!</div>";

        } else if (acao == "ligar_tudo") {
            if (estadoInv.redeAltaBloqCurto || estadoInv.redeBaixaBloqCurto) {
                msgHtml = "<div class='msg-err'>&#9888; Bloqueio ativo! "
                          "Confirme o conserto antes de ligar tudo.</div>";
            } else {
                setInversor(true);
                delay(2000);
                setRedeBaixa(true);
                setRedeAlta(true);
                setLampadaExterna(true);
                setLampadaInterna(true);
                estadoRede.redeBaixaAtiva      = true;
                estadoRede.redeAltaAtiva       = true;
                estadoRede.lampExternaAtiva    = true;
                estadoRede.lampInternaAtiva    = true;
                estadoRede.lampExtBloqEnergia  = false;
                estadoRede.aguardandoRedeBaixa = false;
                estadoRede.aguardandoRedeAlta  = false;
                estadoRede.aguardandoLampExt   = false;
                estadoRede.aguardandoLampInt   = false;
                estadoInv.faseTesteDiagnostico = 0;
                adicionarLog("Manual: tudo LIGADO");
                msgHtml = "<div class='msg-ok'>&#10003; Tudo ligado!</div>";
            }

        } else if (acao == "toggle_lampext") {
            bool novoEstado = !estadoRede.lampExternaAtiva;
            setLampadaExterna(novoEstado);
            estadoRede.lampExtBloqEnergia = false;
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual: lamp.ext %s", novoEstado ? "LIGADA" : "DESLIGADA");
            adicionarLog(tempBuf);
            msgHtml = novoEstado
                ? "<div class='msg-ok'>&#10003; Lamp.Ext LIGADA!</div>"
                : "<div class='msg-ok'>&#10003; Lamp.Ext DESLIGADA!</div>";

        } else if (acao == "toggle_lampint") {
            bool novoEstado = !estadoRede.lampInternaAtiva;
            setLampadaInterna(novoEstado);
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual: lamp.int %s", novoEstado ? "LIGADA" : "DESLIGADA");
            adicionarLog(tempBuf);
            msgHtml = novoEstado
                ? "<div class='msg-ok'>&#10003; Lamp.Int LIGADA!</div>"
                : "<div class='msg-ok'>&#10003; Lamp.Int DESLIGADA!</div>";

        } else if (acao == "toggle_redealta") {
            if (estadoInv.redeAltaBloqCurto) {
                msgHtml = "<div class='msg-err'>&#9888; Rede Alta bloqueada! "
                          "Confirme o conserto primeiro.</div>";
            } else {
                bool novoEstado = !estadoRede.redeAltaAtiva;
                setRedeAlta(novoEstado);
                snprintf(tempBuf, sizeof(tempBuf),
                         "Manual: rede alta %s", novoEstado ? "LIGADA" : "DESLIGADA");
                adicionarLog(tempBuf);
                msgHtml = novoEstado
                    ? "<div class='msg-ok'>&#10003; Rede Alta LIGADA!</div>"
                    : "<div class='msg-ok'>&#10003; Rede Alta DESLIGADA!</div>";
            }

        } else if (acao == "confirmar_ra") {
            if (estadoInv.aguardandoConfirmacaoRA) {
                confirmarConsertoRedeAlta();
                msgHtml = "<div class='msg-ok'>&#9889; Testando Rede Alta... "
                          "Aguarde o resultado nos logs!</div>";
            } else {
                msgHtml = "<div class='msg-err'>&#10007; Nenhum bloqueio ativo "
                          "na Rede Alta.</div>";
            }

        } else if (acao == "confirmar_rb") {
            if (estadoInv.aguardandoConfirmacaoRB) {
                confirmarConsertoRedeBaixa();
                msgHtml = "<div class='msg-ok'>&#9889; Testando Rede Baixa... "
                          "Aguarde o resultado nos logs!</div>";
            } else {
                msgHtml = "<div class='msg-err'>&#10007; Nenhum bloqueio ativo "
                          "na Rede Baixa.</div>";
            }
        }
    }

    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float soc    = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);
    float eD = cfgBat.energiaDisponivel;

    String conteudo = msgHtml;

    if (estadoInv.aguardandoConfirmacaoRA || estadoInv.aguardandoConfirmacaoRB) {
        conteudo += "<div style='background:#7b1fa2;border:2px solid #ce93d8;"
                    "border-radius:8px;padding:12px;margin-bottom:12px;'>";
        conteudo += "<div style='color:#ce93d8;font-weight:bold;font-size:13px;"
                    "margin-bottom:8px;'>&#9889; DIAGN&Oacute;STICO - A&ccedil;&atilde;o Necess&aacute;ria</div>";

        if (estadoInv.aguardandoConfirmacaoRA) {
            conteudo += "<div style='background:#4a148c;border-radius:6px;"
                        "padding:10px;margin-bottom:8px;'>";
            conteudo += "<div style='color:#ef9a9a;font-weight:bold;'>"
                        "&#9888; PROBLEMA DETECTADO: REDE ALTA</div>";
            conteudo += "<div style='color:#ce93d8;font-size:11px;margin:6px 0;'>"
                        "O inversor detectou curto ou equipamento com defeito "
                        "na Rede Alta.<br>"
                        "A Rede Alta foi DESLIGADA automaticamente por seguran&ccedil;a.<br>"
                        "<b>Verifique e conserte antes de continuar!</b></div>";
            snprintf(tempBuf, sizeof(tempBuf), "%lu",
                     (unsigned long)estadoInv.totalCurtosRA);
            conteudo += "<div style='color:#546e7a;font-size:10px;'>"
                        "Total de ocorr&ecirc;ncias: " + String(tempBuf) + "</div>";
            conteudo += "<form method='POST' action='/rede' style='margin-top:8px;'>"
                        "<input type='hidden' name='acao' value='confirmar_ra'>"
                        "<button type='submit' style='background:#2e7d32;"
                        "color:#fff;border:none;border-radius:4px;"
                        "padding:10px 16px;font-size:12px;font-weight:bold;"
                        "cursor:pointer;width:100%;'>"
                        "&#10003; Consertei a Rede Alta - Iniciar Teste!</button></form>";
            conteudo += "</div>";
        }

        if (estadoInv.aguardandoConfirmacaoRB) {
            conteudo += "<div style='background:#4a148c;border-radius:6px;"
                        "padding:10px;margin-bottom:8px;'>";
            conteudo += "<div style='color:#ef9a9a;font-weight:bold;'>"
                        "&#9888; PROBLEMA DETECTADO: REDE BAIXA</div>";
            conteudo += "<div style='color:#ce93d8;font-size:11px;margin:6px 0;'>"
                        "O inversor detectou curto ou equipamento com defeito "
                        "na Rede Baixa.<br>"
                        "A Rede Baixa foi DESLIGADA automaticamente por seguran&ccedil;a.<br>"
                        "<b>Verifique e conserte antes de continuar!</b></div>";
            snprintf(tempBuf, sizeof(tempBuf), "%lu",
                     (unsigned long)estadoInv.totalCurtosRB);
            conteudo += "<div style='color:#546e7a;font-size:10px;'>"
                        "Total de ocorr&ecirc;ncias: " + String(tempBuf) + "</div>";
            conteudo += "<form method='POST' action='/rede' style='margin-top:8px;'>"
                        "<input type='hidden' name='acao' value='confirmar_rb'>"
                        "<button type='submit' style='background:#2e7d32;"
                        "color:#fff;border:none;border-radius:4px;"
                        "padding:10px 16px;font-size:12px;font-weight:bold;"
                        "cursor:pointer;width:100%;'>"
                        "&#10003; Consertei a Rede Baixa - Iniciar Teste!</button></form>";
            conteudo += "</div>";
        }
        conteudo += "</div>";
    }

    conteudo += "<h3>&#9889; Status da Rede El&eacute;trica</h3>";
    conteudo += "<div style='margin:8px 0;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-bottom:3px;'>"
                "<span>Energia Dispon&iacute;vel</span>"
                "<span id='whDispLabel'>-- Wh (--%)</span></div>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:4px;height:20px;position:relative;'>";

    if (eD > 0) {
        struct { float p; const char* cor; } marcas[] = {
            { cfgRede.percLampInt,  "#4fc3f7" },
            { cfgRede.percLampExt,  "#ff9800" },
            { cfgRede.percRedeAlta, "#ffc107" },
            { cfgRede.percRedeBaixa,"#ef5350" },
        };
        for (int i = 0; i < 4; i++) {
            if (marcas[i].p > 0) {
                conteudo += "<div style='position:absolute;top:0;height:100%;"
                            "width:2px;background:" + String(marcas[i].cor)
                          + ";left:" + String(marcas[i].p, 0)
                          + "%;opacity:0.9;'></div>";
            }
        }
    }
    conteudo += "<div id='barraEnergia' style='height:100%;border-radius:4px;"
                "width:0%;background:#66bb6a;transition:width 1s;'></div>";
    conteudo += "</div></div>";

    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>&#9889; Inversor</div>"
                "<div class='stat-val' id='stInvBox'>--</div>"
                "<div class='stat-unit' id='stInvUnit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>&#128267; Rede Baixa</div>"
                "<div class='stat-val' id='stRBbox'>--</div>"
                "<div class='stat-unit' id='stRBunit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>&#9889; Rede Alta</div>"
                "<div class='stat-val' id='stRAbox'>--</div>"
                "<div class='stat-unit' id='stRAunit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>&#128161; Lamp.Ext</div>"
                "<div class='stat-val' id='stLEbox'>--</div>"
                "<div class='stat-unit' id='stLEunit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>&#128161; Lamp.Int</div>"
                "<div class='stat-val' id='stLIbox'>--</div>"
                "<div class='stat-unit' id='stLIunit'>--</div></div>";

    conteudo += "<div class='stat-box'><div class='stat-label'>&#128269; Diagn&oacute;stico</div>"
                "<div class='stat-val' id='stDiagBox' style='font-size:11px;'>--</div>"
                "<div class='stat-unit' id='stDiagUnit'>--</div></div>";
    conteudo += "</div>";

    conteudo += "<div id='infoLampExt' style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:8px;font-size:11px;margin:6px 0;'>"
                "Carregando...</div>";

    conteudo += "<div id='alertasRede'></div>";
    conteudo += "<div id='alertasDiag'></div>";

    conteudo += "<h3>&#128200; N&iacute;veis de Controle</h3>";
    conteudo += "<div style='overflow-x:auto;'>";
    conteudo += "<table style='width:100%;border-collapse:collapse;font-size:10px;'>";
    conteudo += "<thead><tr style='background:#1e3a5f;color:#90caf9;'>"
                "<th style='padding:5px;text-align:left;'>Carga</th>"
                "<th style='padding:5px;text-align:center;'>Corte</th>"
                "<th style='padding:5px;text-align:center;'>Wh M&iacute;n</th>"
                "<th style='padding:5px;text-align:center;'>Cortes</th>"
                "</tr></thead><tbody>";

    struct {
        const char* n; const char* ic;
        float p; float w; uint32_t c; const char* cor;
    } niveis[] = {
        {"Rede Baixa","&#128267;", cfgRede.percRedeBaixa,
         eD*(cfgRede.percRedeBaixa/100.0f),
         estadoRede.totalCortesRedeBaixa,"#ef5350"},
        {"Rede Alta","&#9889;",    cfgRede.percRedeAlta,
         eD*(cfgRede.percRedeAlta/100.0f),
         estadoRede.totalCortesRedeAlta,"#ffc107"},
        {"Lamp.Ext","&#128161;",   cfgRede.percLampExt,
         eD*(cfgRede.percLampExt/100.0f),
         estadoRede.totalCortesLampExt, "#ff9800"},
        {"Lamp.Int","&#128161;",   cfgRede.percLampInt,
         eD*(cfgRede.percLampInt/100.0f),
         estadoRede.totalCortesLampInt, "#4fc3f7"},
    };

    for (int i = 0; i < 4; i++) {
        const char* bg = (i%2==0) ? "#0d0d0d" : "#111";
        conteudo += "<tr style='background:" + String(bg) + ";'>";
        conteudo += "<td style='padding:5px;color:#90caf9;'>"
                  + String(niveis[i].ic) + " "
                  + String(niveis[i].n) + "</td>";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f%%", niveis[i].p);
        conteudo += "<td style='padding:5px;text-align:center;color:"
                  + String(niveis[i].cor) + ";font-weight:bold;'>"
                  + String(tempBuf) + "</td>";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f Wh", niveis[i].w);
        conteudo += "<td style='padding:5px;text-align:center;"
                    "color:#78909c;'>" + String(tempBuf) + "</td>";
        snprintf(tempBuf, sizeof(tempBuf), "%lu", (unsigned long)niveis[i].c);
        conteudo += "<td style='padding:5px;text-align:center;"
                    "color:#546e7a;'>" + String(tempBuf) + "</td>";
        conteudo += "</tr>";
    }
    conteudo += "</tbody></table></div>";

    conteudo += "<div class='info' style='font-size:11px;margin-top:6px;'>";
    conteudo += "&#128267; Base: <b>" + String(eD, 0) + " Wh</b>"
                " &nbsp;|&nbsp; Histerese: <b>+2%</b>"
                " &nbsp;|&nbsp; Confirma&ccedil;&atilde;o: <b>1 min</b><br>";
    snprintf(tempBuf, sizeof(tempBuf), "%lu / %lu",
             (unsigned long)estadoInv.totalCurtosRA,
             (unsigned long)estadoInv.totalCurtosRB);
    conteudo += "&#9889; Curtos RA/RB: <b>" + String(tempBuf) + "</b>"
                " &nbsp;|&nbsp; Fase diag: <b id='faseAtual'>";
    conteudo += String(estadoInv.faseTesteDiagnostico) + "</b></div>";

    conteudo += "<h3>&#9881; Controle Manual</h3>";
    conteudo += "<div class='flex'>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='ligar_tudo'>"
                "<button type='submit' class='btn-green' style='font-size:10px;'>"
                "&#9889; Ligar Tudo</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='toggle_redealta'>"
                "<button type='submit' class='btn-orange' style='font-size:10px;"
                + String(estadoInv.redeAltaBloqCurto ?
                  "opacity:0.5;cursor:not-allowed;" : "") + "'>"
                "&#9889; Tog.RA</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='toggle_lampext'>"
                "<button type='submit' class='btn-orange' style='font-size:10px;'>"
                "&#128161; Tog.LE</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='toggle_lampint'>"
                "<button type='submit' class='btn-orange' style='font-size:10px;'>"
                "&#128161; Tog.LI</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='reset_stats'>"
                "<button type='submit' class='btn-gray' style='font-size:10px;'>"
                "&#8635; Zerar</button></form>";
    conteudo += "</div>";

    conteudo += "<h3>&#9881; Configurar N&iacute;veis de Corte</h3>";
    conteudo += "<div class='msg-warn' style='font-size:10px;'>"
                "&#9432; Rede Baixa = desliga tudo (inversor OFF). "
                "Base: <b>" + String(eD, 0) + " Wh</b></div>";

    conteudo += "<form method='POST' action='/rede'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";

    struct {
        const char* lbl; const char* nm;
        float val; const char* cor; const char* desc;
    } campos[] = {
        {"&#128267; Rede Baixa - Desligar TUDO + Inversor (%)",
         "perc_redbaixa", cfgRede.percRedeBaixa, "#ef5350",
         "Corte total: desliga inversor + rede baixa + alta. (padrao: 10%)"},
        {"&#9889; Rede Alta - Desligar equipamentos pesados (%)",
         "perc_redealta", cfgRede.percRedeAlta, "#ffc107",
         "Tomadas de alta potencia sem energia. (padrao: 30%)"},
        {"&#128161; Lamp.Externa - Corte por energia (%)",
         "perc_lampext",  cfgRede.percLampExt,  "#ff9800",
         "Energia minima para lamp. externa. (padrao: 25%)"},
        {"&#128161; Lamp.Interna - Corte por energia (%)",
         "perc_lampint",  cfgRede.percLampInt,  "#4fc3f7",
         "0% = so protecao da bateria. (padrao: 0%)"},
    };

    for (int i = 0; i < 4; i++) {
        conteudo += "<label>" + String(campos[i].lbl) + ":</label>";
        conteudo += "<div style='display:flex;gap:8px;align-items:center;'>";
        conteudo += "<input type='number' name='" + String(campos[i].nm)
                  + "' min='0' max='80' step='1' required value='"
                  + String((int)campos[i].val)
                  + "' style='flex:1;' oninput='calcWh(this,"
                  + String(eD, 0) + ")'>";
        conteudo += "<span style='color:" + String(campos[i].cor)
                  + ";font-size:11px;min-width:70px;' id='wh_"
                  + String(campos[i].nm) + "'>";
        snprintf(tempBuf, sizeof(tempBuf), "%.0f Wh",
                 eD * (campos[i].val / 100.0f));
        conteudo += String(tempBuf) + "</span></div>";
        conteudo += "<div style='color:#546e7a;font-size:9px;margin-top:1px;'>"
                  + String(campos[i].desc) + "</div>";
    }

    conteudo += "<h3>&#128161; Controle Lamp. Externa</h3>";
    conteudo += "<div class='info' style='font-size:10px;'>"
                "&#9432; O corte por energia tem PRIORIDADE sobre "
                "qualquer modo de controle.</div>";

    conteudo += "<label>Modo de Controle:</label>";
    conteudo += "<select name='modo_lampext' id='modoLampSel' "
                "onchange='toggleModoLamp()'>";
    conteudo += "<option value='0'"
                + String(cfgRede.modoLampExt==0?" selected":"")
                + ">&#9889; Apenas energia (sem hor&aacute;rio)</option>";
    conteudo += "<option value='1'"
                + String(cfgRede.modoLampExt==1?" selected":"")
                + ">&#128336; Hor&aacute;rio Manual</option>";
    conteudo += "<option value='2'"
                + String(cfgRede.modoLampExt==2?" selected":"")
                + ">&#127774; Autom&aacute;tico por Painel Solar</option>";
    conteudo += "</select>";

    conteudo += "<div id='divHorario' style='display:"
              + String(cfgRede.modoLampExt==1?"block":"none") + ";'>";
    conteudo += "<div style='display:grid;grid-template-columns:1fr 1fr;"
                "gap:8px;margin-top:8px;'>";
    conteudo += "<div><label style='color:#66bb6a;'>&#9650; Ligar &agrave;s:</label>"
                "<div style='display:flex;gap:4px;'>"
                "<input type='number' name='ligar_hora' min='0' max='23' "
                "style='width:50%;' value='" + String(cfgRede.ligarHora) + "'>"
                "<input type='number' name='ligar_min'  min='0' max='59' "
                "style='width:50%;' value='" + String(cfgRede.ligarMin)  + "'>"
                "</div></div>";
    conteudo += "<div><label style='color:#ef5350;'>&#9660; Desligar &agrave;s:</label>"
                "<div style='display:flex;gap:4px;'>"
                "<input type='number' name='desl_hora' min='0' max='23' "
                "style='width:50%;' value='" + String(cfgRede.desligarHora) + "'>"
                "<input type='number' name='desl_min'  min='0' max='59' "
                "style='width:50%;' value='" + String(cfgRede.desligarMin)  + "'>"
                "</div></div>";
    conteudo += "</div>";

    if (!ntpSincronizado)
        conteudo += "<div class='msg-warn' style='font-size:10px;margin-top:6px;'>"
                    "&#9888; NTP n&atilde;o sincronizado!</div>";
    else {
        char horaStr[10];
        time_t now = time(nullptr);
        struct tm* ti = localtime(&now);
        snprintf(horaStr, sizeof(horaStr), "%02d:%02d",
                 ti->tm_hour, ti->tm_min);
        conteudo += "<div class='msg-ok' style='font-size:10px;margin-top:6px;'>"
                    "&#10003; NTP OK - Hora: <b>" + String(horaStr)
                  + "</b></div>";
    }
    conteudo += "</div>";

    conteudo += "<div id='divPainel' style='display:"
              + String(cfgRede.modoLampExt==2?"block":"none") + ";'>";
    conteudo += "<div class='info' style='font-size:10px;'>"
                "&#127774; Painel &gt; 0.5A &rarr; L&acirc;mpada APAGA (dia)<br>"
                "Painel = 0A &rarr; L&acirc;mpada LIGA (noite)<br>"
                "Painel atual: <b id='painelStatusConf'>--</b></div>";
    conteudo += "</div>";

    conteudo += "<button type='submit' class='btn-green' style='margin-top:12px;'>"
                "&#128190; Salvar Configura&ccedil;&atilde;o</button>";
    conteudo += "</form>";
    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";

    String script = "";
    script += "var energiaDisp=" + String(eD, 1) + ";";

    script += F("var faseNomes=["
                "'Normal',"
                "'Aguard.Sinal',"
                "'Teste s/RA',"
                "'Teste s/RB',"
                "'Bloq.RA-Conf',"
                "'Bloq.RB-Conf',"
                "'Teste Final RA',"
                "'Teste Final RB'"
                "];");

    script += F("function calcWh(el,eD){"
                "var p=parseFloat(el.value)||0;"
                "var sp=document.getElementById('wh_'+el.name);"
                "if(sp)sp.textContent=(eD*(p/100)).toFixed(0)+' Wh';}"

                "function toggleModoLamp(){"
                "var m=parseInt(document.getElementById('modoLampSel').value);"
                "document.getElementById('divHorario').style.display="
                "(m==1?'block':'none');"
                "document.getElementById('divPainel').style.display="
                "(m==2?'block':'none');}"

                "function fmtT(s){"
                "var m=Math.floor(s/60);var sc=s%60;"
                "return m+'m '+pad(sc)+'s';}"
                "function pad(n){return n<10?'0'+n:n;}"

                "function atualizarRede(){"
                "var x=new XMLHttpRequest();"
                "x.onreadystatechange=function(){"
                "if(x.readyState==4&&x.status==200){"
                "try{var d=JSON.parse(x.responseText);"

                "var p=(d.whDisp/d.whTotal)*100;"
                "p=Math.max(0,Math.min(100,p));"
                "var bEl=document.getElementById('barraEnergia');"
                "bEl.style.width=p+'%';"
                "bEl.style.background=p>=50?'#66bb6a':(p>=25?'#ffa726':'#ef5350');"
                "document.getElementById('whDispLabel').textContent="
                "d.whDisp.toFixed(0)+' Wh ('+d.soc.toFixed(1)+'%)';"

                "function stCard(bId,uId,ativo,aguard,tRest,nOn,nOff,bloq){"
                "var b=document.getElementById(bId);"
                "var u=document.getElementById(uId);"
                "if(!b||!u)return;"
                "if(bloq){"
                "b.textContent='⚠ BLOQUEADO';"
                "b.className='stat-val stat-err';"
                "u.textContent='Curto detectado';"
                "u.style.color='#ce93d8';"
                "return;}"
                "b.textContent=ativo?'✓ LIGADO':'✗ DESLIGADO';"
                "b.className='stat-val '+(ativo?'stat-ok':'stat-err');"
                "if(aguard&&tRest>0){"
                "u.textContent='⏱ '+fmtT(tRest);"
                "u.style.color='#ffa726';}else{"
                "u.textContent=ativo?nOn:nOff;"
                "u.style.color='';}}"

                "var iEl=document.getElementById('stInvBox');"
                "var iUEl=document.getElementById('stInvUnit');"
                "if(d.invProt){"
                "iEl.textContent='⚠ PROTECAO';"
                "iEl.className='stat-val stat-err';"
                "iUEl.textContent='Reiniciando...';}"
                "else if(!d.invSt&&d.invLig){"
                "iEl.textContent='⚠ SEM SINAL';"
                "iEl.className='stat-val stat-warn';"
                "iUEl.textContent='Verificando...';}"
                "else{"
                "iEl.textContent=d.invSt?'✓ OK':'✗ DESLIGADO';"
                "iEl.className='stat-val '+(d.invSt?'stat-ok':'stat-warn');"
                "iUEl.textContent=d.invLig?'Ligado':'Desligado';}"

                "stCard('stRBbox','stRBunit',d.stRB,d.agRB,d.tRB,"
                "'Rede baixa ativa','Rede baixa OFF',d.bloqRB);"
                "stCard('stRAbox','stRAunit',d.stRA,d.agRA,d.tRA,"
                "'Tomadas ativas','Tomadas OFF',d.bloqRA);"
                "stCard('stLEbox','stLEunit',d.stLE,d.agLE,d.tLE,"
                "'Lamp.ext ON','Lamp.ext OFF',false);"
                "stCard('stLIbox','stLIunit',d.stLI,d.agLI,d.tLI,"
                "'Lamp.int ON','Lamp.int OFF',false);"

                "var dEl=document.getElementById('stDiagBox');"
                "var dUEl=document.getElementById('stDiagUnit');"
                "var fase=d.faseDiag||0;"
                "if(dEl){"
                "dEl.textContent=faseNomes[fase]||'Fase '+fase;"
                "dEl.className='stat-val '+(fase==0?'stat-ok':"
                "(fase>=4?'stat-err':'stat-warn'));}"
                "if(dUEl)dUEl.textContent='Fase '+fase+'/7';"

                "var fEl=document.getElementById('faseAtual');"
                "if(fEl)fEl.textContent=fase;"

                "var modos=['Energia','Horario','Painel'];"
                "var il='&#128161; Modo: <b>'+modos[d.mLE]+'</b> &nbsp;|&nbsp; ';"
                "if(d.mLE==1){"
                "il+='Liga: <b>'+pad(d.ligH)+':'+pad(d.ligM)+'</b>"
                " &nbsp;|&nbsp; ';"
                "il+='Desliga: <b>'+pad(d.dslH)+':'+pad(d.dslM)+'</b>"
                " &nbsp;|&nbsp; ';"
                "il+='Hora: <b>'+d.horaAtual+'</b><br>';}"
                "if(d.mLE==2){"
                "il+='&#127774; Painel: <b>"
                "'+(d.painelAtivo?'GERANDO (dia)':'PARADO (noite)')+'</b><br>';}"
                "if(d.leBloq)il+='<span style=\"color:#ef5350;\">"
                "&#9888; Bloqueada por energia</span>';"
                "document.getElementById('infoLampExt').innerHTML=il;"

                "var pSEl=document.getElementById('painelStatusConf');"
                "if(pSEl){pSEl.textContent=d.painelAtivo?"
                "'GERANDO > 0.5A':'PARADO 0A';"
                "pSEl.style.color=d.painelAtivo?'#66bb6a':'#ffa726';}"

                "var al='';"
                "if(d.agRB)al+='<div class=\"msg-err\">&#9201; Desligando"
                " TUDO em '+fmtT(d.tRB)+'</div>';"
                "if(d.agRA)al+='<div class=\"msg-warn\">&#9201; Desligando"
                " Rede Alta em '+fmtT(d.tRA)+'</div>';"
                "if(d.agLE)al+='<div class=\"msg-warn\">&#9201; Desligando"
                " Lamp.Ext em '+fmtT(d.tLE)+'</div>';"
                "if(d.agLI)al+='<div class=\"msg-warn\">&#9201; Desligando"
                " Lamp.Int em '+fmtT(d.tLI)+'</div>';"
                "document.getElementById('alertasRede').innerHTML=al;"

                "var ad='';"
                "if(fase==1)ad+='<div class=\"msg-warn\">&#128269; "
                "Aguardando sinal do inversor...</div>';"
                "if(fase==2)ad+='<div class=\"msg-warn\">&#128269; "
                "Testando SEM Rede Alta...</div>';"
                "if(fase==3)ad+='<div class=\"msg-warn\">&#128269; "
                "Testando SEM Rede Baixa...</div>';"
                "if(fase==6)ad+='<div class=\"msg-ok\">&#9889; "
                "Testando Rede Alta - aguarde resultado...</div>';"
                "if(fase==7)ad+='<div class=\"msg-ok\">&#9889; "
                "Testando Rede Baixa - aguarde resultado...</div>';"
                "if(d.bloqRA&&fase==4)ad+='<div class=\"msg-err\">"
                "&#9888; REDE ALTA BLOQUEADA - Confirme o conserto acima!</div>';"
                "if(d.bloqRB&&fase==5)ad+='<div class=\"msg-err\">"
                "&#9888; REDE BAIXA BLOQUEADA - Confirme o conserto acima!</div>';"
                "document.getElementById('alertasDiag').innerHTML=ad;"

                "}catch(e){}"
                "}};"
                "x.open('GET','/api/rede',true);x.send();}"
                "setInterval(atualizarRede,2000);"
                "atualizarRede();");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Rede El&eacute;trica",
                            conteudo, script));
}

// ============================================================
// PÁGINA: /historico
// ============================================================
void handleHistorico() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String filtroData = server.arg("data");
    bool   temFiltro  = filtroData.length() == 10;

    adicionarLog("Historico acessado");

    String conteudo = "";
    conteudo += "<h3>&#128202; Hist&oacute;rico de Energia</h3>";
    conteudo += "<form method='GET' action='/historico' "
                "style='display:flex;gap:6px;margin-bottom:10px;'>";
    conteudo += "<input type='date' name='data' value='" + filtroData + "' "
                "style='flex:1;padding:8px;background:#111;border:1px solid #444;"
                "border-radius:4px;color:#e0e0e0;font-size:12px;'>";
    conteudo += "<button type='submit' "
                "style='width:auto;padding:8px 12px;margin:0;font-size:12px;'>"
                "&#128269; Filtrar</button>";
    if (temFiltro)
        conteudo += "<a href='/historico' style='padding:8px 10px;"
                    "background:#546e7a;color:#fff;border-radius:4px;"
                    "font-size:12px;text-decoration:none;"
                    "display:flex;align-items:center;'>&#10005;</a>";
    conteudo += "</form>";

    if (histTotal == 0) {
        conteudo += "<div class='msg-warn'>&#9888; Nenhum registro ainda. "
                    "Aguarde 60s...</div>";
    } else {
        double   totalPainelWh = 0, totalCasaWh = 0;
        double   totalCargaWh  = 0, totalDescWh = 0;
        int      registrosFilt = 0;
        int      maxExibir     = temFiltro ? MAX_HISTORICO : 60;

        conteudo += "<div style='overflow-x:auto;'>";
        conteudo += "<table style='width:100%;border-collapse:collapse;font-size:10px;'>";
        conteudo += "<thead><tr style='background:#1e3a5f;color:#90caf9;'>"
                    "<th style='padding:6px 4px;text-align:left;'>Data/Hora</th>"
                    "<th style='padding:6px 4px;text-align:right;'>&#9728; Painel</th>"
                    "<th style='padding:6px 4px;text-align:right;'>&#127968; Casa</th>"
                    "<th style='padding:6px 4px;text-align:right;'>&#8679; Carga</th>"
                    "<th style='padding:6px 4px;text-align:right;'>&#8681; Desc.</th>"
                    "</tr></thead><tbody>";

        for (int i = 0; i < histTotal && registrosFilt < maxExibir; i++) {
            int      idx = ((int)histIdx - 1 - i + MAX_HISTORICO) % MAX_HISTORICO;
            uint32_t ts  = historico[idx].timestamp;

            if (temFiltro && ts > 0) {
                struct tm* ti = localtime((time_t*)&ts);
                char dataBuf[12];
                snprintf(dataBuf, sizeof(dataBuf), "%04d-%02d-%02d",
                         ti->tm_year+1900, ti->tm_mon+1, ti->tm_mday);
                if (filtroData != String(dataBuf)) continue;
            }

            float wPai = historico[idx].whPainel      / 10.0f;
            float wCas = historico[idx].whCasa        / 10.0f;
            float wCar = historico[idx].whCargaBat    / 10.0f;
            float wDes = historico[idx].whDescargaBat / 10.0f;

            totalPainelWh += wPai;
            totalCasaWh   += wCas;
            totalCargaWh  += wCar;
            totalDescWh   += wDes;
            registrosFilt++;

            char dhBuf[20] = "--/-- --:--";
            if (ts > 0) {
                struct tm* ti = localtime((time_t*)&ts);
                snprintf(dhBuf, sizeof(dhBuf), "%02d/%02d %02d:%02d",
                         ti->tm_mday, ti->tm_mon+1,
                         ti->tm_hour, ti->tm_min);
            }

            const char* bg = (registrosFilt % 2 == 0) ? "#0d0d0d" : "#111";
            conteudo += "<tr style='background:" + String(bg) + ";'>";
            conteudo += "<td style='padding:5px 4px;color:#78909c;'>"
                      + String(dhBuf) + "</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wPai);
            conteudo += "<td style='padding:5px 4px;text-align:right;"
                        "color:#66bb6a;'>" + String(tempBuf) + " Wh</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wCas);
            conteudo += "<td style='padding:5px 4px;text-align:right;"
                        "color:#ffa726;'>" + String(tempBuf) + " Wh</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wCar);
            conteudo += "<td style='padding:5px 4px;text-align:right;"
                        "color:#4fc3f7;'>" + String(tempBuf) + " Wh</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wDes);
            conteudo += "<td style='padding:5px 4px;text-align:right;"
                        "color:#ef5350;'>" + String(tempBuf) + " Wh</td>";
            conteudo += "</tr>";
        }
        conteudo += "</tbody></table></div>";

        if (registrosFilt > 0) {
            conteudo += "<div style='background:#1e3a5f;border-radius:4px;"
                        "padding:10px;margin-top:8px;font-size:11px;'>"
                        "<b style='color:#90caf9;'>&#8721; Totais</b><br>";
            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalPainelWh/1000.0);
            conteudo += "<span style='color:#66bb6a;'>&#9728; "
                      + String(tempBuf) + " kWh</span> &nbsp;";
            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalCasaWh/1000.0);
            conteudo += "<span style='color:#ffa726;'>&#127968; "
                      + String(tempBuf) + " kWh</span><br>";
            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalCargaWh/1000.0);
            conteudo += "<span style='color:#4fc3f7;'>&#8679; "
                      + String(tempBuf) + " kWh</span> &nbsp;";
            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalDescWh/1000.0);
            conteudo += "<span style='color:#ef5350;'>&#8681; "
                      + String(tempBuf) + " kWh</span>";
            snprintf(tempBuf, sizeof(tempBuf), "%d", registrosFilt);
            conteudo += "<br><span style='color:#546e7a;'>Registros: "
                      + String(tempBuf) + " (60s/reg)</span></div>";
        }

        snprintf(tempBuf, sizeof(tempBuf),
                 "Hist: %d/%d registros (%.0f%% cheio)",
                 histTotal, MAX_HISTORICO,
                 (histTotal / (float)MAX_HISTORICO) * 100.0f);
        conteudo += "<div class='info' style='font-size:10px;margin-top:8px;'>"
                    "&#128190; " + String(tempBuf) + "</div>";

        if (!temFiltro && histTotal > 60)
            conteudo += "<div class='msg-warn' style='font-size:11px;'>"
                        "&#9432; Use filtro de data para ver todo o hist&oacute;rico.</div>";
    }

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:10px;font-size:12px;'>&#8592; Voltar</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Hist&oacute;rico", conteudo));
}

// ============================================================
// PÁGINA: /previsao
// ============================================================
void handlePrevisao() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "salvar") {
            double lat = server.arg("lat").toDouble();
            double lon = server.arg("lon").toDouble();
            float  kwp = server.arg("kwp").toFloat();

            if (lat == 0.0 || lon == 0.0)
                msgHtml = "<div class='msg-err'>&#10007; Coordenadas inv&aacute;lidas</div>";
            else if (kwp < 0.1f || kwp > 1000.0f)
                msgHtml = "<div class='msg-err'>&#10007; kWp inv&aacute;lido</div>";
            else {
                cfgPrev.latitude     = lat;
                cfgPrev.longitude    = lon;
                cfgPrev.kwpInstalado = kwp;
                cfgPrev.configurado  = true;
                salvarConfigPrevisao();
                tPrevisao = 0; // força busca imediata
                snprintf(tempBuf, sizeof(tempBuf),
                         "Prev: lat=%.10f lon=%.10f kwp=%.2f",
                         lat, lon, kwp);
                adicionarLog(tempBuf);
                msgHtml = "<div class='msg-ok'>&#10003; Configurado! "
                          "Buscando previs&atilde;o...</div>";
            }
        } else if (acao == "reset_cal") {
            for (int h = 0; h < 24; h++) {
                estadoPrev.fatorHorario[h]    = 1.0f;
                estadoPrev.amostrasHorario[h] = 0;
            }
            estadoPrev.taxaAcerto = 0;
            salvarCalibracaoHoraria(true);
            adicionarLog("Calibracao solar resetada");
            msgHtml = "<div class='msg-ok'>&#10003; Calibra&ccedil;&atilde;o resetada!</div>";
        }
    }

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int horaAtual = ti->tm_hour;

    String conteudo = msgHtml;

    // ── CONFIGURAÇÃO ──
    conteudo += "<h3>&#128205; Localiza&ccedil;&atilde;o do Painel</h3>";
    conteudo += "<div class='info' style='font-size:11px;'>"
                "&#9432; Abra o Google Maps, clique no local do painel e "
                "copie as coordenadas com alta precis&atilde;o.</div>";
    conteudo += "<a href='https://maps.google.com' target='_blank' "
                "style='display:block;text-align:center;padding:10px;"
                "background:#1565c0;color:#fff;border-radius:4px;"
                "text-decoration:none;font-weight:bold;margin:8px 0;'>"
                "&#127759; Abrir Google Maps</a>";

    conteudo += "<form method='POST' action='/previsao'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";
    conteudo += "<label>Latitude:</label>";
    conteudo += "<input type='text' name='lat' required "
                "placeholder='Ex: -22.917549777384778' "
                "value='" + (cfgPrev.configurado
                    ? String(cfgPrev.latitude, 15) : "")
                + "' style='font-family:monospace;'>";
    conteudo += "<label>Longitude:</label>";
    conteudo += "<input type='text' name='lon' required "
                "placeholder='Ex: -47.017843958072520' "
                "value='" + (cfgPrev.configurado
                    ? String(cfgPrev.longitude, 15) : "")
                + "' style='font-family:monospace;'>";
    conteudo += "<label>Pot&ecirc;ncia instalada (kWp):</label>";
    conteudo += "<input type='number' name='kwp' min='0.1' max='1000' "
                "step='0.01' required placeholder='Ex: 3.30' "
                "value='" + String(cfgPrev.kwpInstalado, 2) + "'>";
    conteudo += "<button type='submit' class='btn-green'>"
                "&#128190; Salvar e Buscar</button>";
    conteudo += "</form>";

    // ── STATUS ──
    if (estadoPrev.dadosValidos) {
        int idxAtual = max(0, horaAtual - estadoPrev.horaOffsetAPI);

        conteudo += "<h3>&#9728; Status Atual</h3>";
        conteudo += "<div class='stat-grid'>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f",
                 estadoPrev.radiacaoHora[idxAtual]);
        conteudo += "<div class='stat-box'><div class='stat-label'>"
                    "Radia&ccedil;&atilde;o</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>W/m&sup2;</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f",
                 estadoPrev.nuvensPct[idxAtual]);
        conteudo += "<div class='stat-box'><div class='stat-label'>Nuvens</div>"
                    "<div class='stat-val stat-warn'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>%</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f",
                 estadoPrev.geracaoEstHora[idxAtual]);
        conteudo += "<div class='stat-box'><div class='stat-label'>Ger. Est.</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>Wh/h</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", sensores.potenciaPainel);
        conteudo += "<div class='stat-box'><div class='stat-label'>Ger. Real</div>"
                    "<div class='stat-val stat-ok' id='gerReal'>"
                    + String(tempBuf)
                    + "</div><div class='stat-unit'>W</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.1f", estadoPrev.taxaAcerto);
        conteudo += "<div class='stat-box'><div class='stat-label'>Acerto</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>%</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.3f",
                 estadoPrev.fatorHorario[horaAtual]);
        conteudo += "<div class='stat-box'><div class='stat-label'>Fator Cal.</div>"
                    "<div class='stat-val'>" + String(tempBuf)
                    + "</div><div class='stat-unit'>x</div></div>";
        conteudo += "</div>";

        conteudo += "<div class='info' style='font-size:11px;'>";
        snprintf(tempBuf, sizeof(tempBuf),
                 "&#9728; Restante hoje: <b>%.0f Wh</b>"
                 " &nbsp;|&nbsp; &#128197; Amanh&atilde;: <b>%.0f Wh</b>"
                 " &nbsp;|&nbsp; &#127968; Consumo: <b>%.0f Wh/dia</b>",
                 estadoPrev.totalRestanteHoje,
                 estadoPrev.totalAmanha,
                 estadoPrev.consumoMedioDia);
        conteudo += String(tempBuf) + "</div>";

        // Decisões automáticas
        conteudo += "<h3>&#129302; Decis&otilde;es Autom&aacute;ticas</h3>";
        conteudo += "<div style='background:#111;border:1px solid #333;"
                    "border-radius:6px;padding:10px;font-size:11px;'>";
        conteudo += "<div style='padding:6px 0;border-bottom:1px solid #1a1a1a;'>"
                    "<span style='color:#78909c;'>&#9889; Rede Alta: </span>";
        if (estadoPrev.bloqRedeAlta)
            conteudo += "<span style='color:#ef5350;font-weight:bold;'>"
                        "&#10007; DESATIVADA</span><br>"
                        "<span style='color:#546e7a;font-size:10px;'>"
                        + String(estadoPrev.motivoRedeAlta) + "</span>";
        else
            conteudo += "<span style='color:#66bb6a;'>&#10003; Normal</span>";
        conteudo += "</div>";
        conteudo += "<div style='padding:6px 0;'>"
                    "<span style='color:#78909c;'>&#128161; Lamp.Ext: </span>";
        if (estadoPrev.bloqLampExt)
            conteudo += "<span style='color:#ef5350;font-weight:bold;'>"
                        "&#10007; DESATIVADA</span><br>"
                        "<span style='color:#546e7a;font-size:10px;'>"
                        + String(estadoPrev.motivoLampExt) + "</span>";
        else
            conteudo += "<span style='color:#66bb6a;'>&#10003; Normal</span>";
        conteudo += "</div></div>";

        // Gráficos
        conteudo += "<h3>&#128202; Pr&oacute;ximas Horas</h3>";
        conteudo += "<div id='graficoHoras' style='background:#000;"
                    "border:1px solid #333;border-radius:8px;padding:10px;'>"
                    "<div style='color:#546e7a;font-size:11px;"
                    "text-align:center;'>Carregando...</div></div>";

        conteudo += "<h3>&#128197; Amanh&atilde;</h3>";
        conteudo += "<div id='graficoAmanha' style='background:#000;"
                    "border:1px solid #333;border-radius:8px;padding:10px;'>"
                    "<div style='color:#546e7a;font-size:11px;"
                    "text-align:center;'>Carregando...</div></div>";

        // Calibração horária
        conteudo += "<h3>&#9881; Calibra&ccedil;&atilde;o por Hora</h3>";
        conteudo += "<div style='overflow-x:auto;'>";
        conteudo += "<table style='width:100%;border-collapse:collapse;font-size:9px;'>";
        conteudo += "<thead><tr style='background:#1e3a5f;color:#90caf9;'>"
                    "<th style='padding:4px;'>Hora</th>"
                    "<th style='padding:4px;'>Fator</th>"
                    "<th style='padding:4px;'>Amostras</th>"
                    "<th style='padding:4px;'>Status</th>"
                    "</tr></thead><tbody>";

        for (int h = 0; h < 24; h++) {
            float   f  = estadoPrev.fatorHorario[h];
            uint8_t a  = estadoPrev.amostrasHorario[h];
            const char* bg  = (h == horaAtual) ? "#1e3a1e" :
                              (h%2==0)          ? "#0d0d0d" : "#111";
            const char* corF = (f > 1.1f) ? "#ef5350" :
                               (f < 0.9f) ? "#ffa726" : "#66bb6a";
            const char* st   = (a == 0)  ? "Sem dados"  :
                               (a < 5)   ? "Aprendendo" :
                               (a < 20)  ? "Bom"        : "Calibrado";

            conteudo += "<tr style='background:" + String(bg) + ";'>";
            snprintf(tempBuf, sizeof(tempBuf), "%02dh", h);
            conteudo += "<td style='padding:4px;color:#78909c;'>"
                      + String(tempBuf) + "</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%.3f", f);
            conteudo += "<td style='padding:4px;text-align:center;color:"
                      + String(corF) + ";font-weight:bold;'>"
                      + String(tempBuf) + "</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%d", (int)a);
            conteudo += "<td style='padding:4px;text-align:center;"
                        "color:#546e7a;'>" + String(tempBuf) + "</td>";
            conteudo += "<td style='padding:4px;color:#546e7a;'>"
                      + String(st) + "</td>";
            conteudo += "</tr>";
        }
        conteudo += "</tbody></table></div>";

        conteudo += "<form method='POST' action='/previsao' "
                    "style='margin-top:8px;'>"
                    "<input type='hidden' name='acao' value='reset_cal'>"
                    "<button type='submit' class='btn-orange' "
                    "style='font-size:11px;'>"
                    "&#8635; Resetar Calibra&ccedil;&atilde;o</button></form>";

    } else {
        conteudo += "<div class='msg-warn'>";
        if (!cfgPrev.configurado)
            conteudo += "&#9888; Configure a localiza&ccedil;&atilde;o acima.";
        else if (modoAP)
            conteudo += "&#9888; Sem internet (modo AP).";
        else
            conteudo += "&#9888; Aguardando dados...";
        conteudo += "</div>";
    }

    if (estadoPrev.tsUltimaAtualizacao > 0) {
        unsigned long decorrido =
            (millis() - estadoPrev.tsUltimaAtualizacao) / 60000;
        snprintf(tempBuf, sizeof(tempBuf),
                 "&#128337; &Uacute;ltima atualiza&ccedil;&atilde;o: "
                 "h&aacute; %lu min", decorrido);
        conteudo += "<div class='info' style='font-size:10px;'>"
                  + String(tempBuf) + "</div>";
    }

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";

    // ── Script gráficos ──
    String script = F("function pad(n){return n<10?'0'+n:n;}"
                      "function iconeNuvem(n,c){"
                      "if(c>30)return'&#9928;';"
                      "if(n<20)return'&#9728;';"
                      "if(n<50)return'&#9925;';"
                      "return'&#9729;';}"

                      "function gerarGrafico(cId,dados,maxG){"
                      "var c=document.getElementById(cId);"
                      "if(!c)return;"
                      "var h='';"
                      "for(var i=0;i<dados.length;i++){"
                      "var d=dados[i];"
                      "var pct=maxG>0?(d.g/maxG*100):0;"
                      "pct=Math.min(pct,100);"
                      "var cor=d.n<30?'#66bb6a':(d.n<60?'#ffa726':'#546e7a');"
                      "h+='<div style=\"display:flex;align-items:center;"
                      "margin-bottom:4px;\">';"
                      "h+='<span style=\"width:28px;color:#546e7a;"
                      "font-size:9px;text-align:right;margin-right:4px;\">"
                      "'+pad(d.h)+'h</span>';"
                      "h+='<span style=\"font-size:12px;margin-right:4px;\">"
                      "'+iconeNuvem(d.n,d.c)+'</span>';"
                      "h+='<div style=\"flex:1;background:#1a1a1a;"
                      "border-radius:2px;height:16px;\">';"
                      "h+='<div style=\"height:100%;background:'+cor+"
                      "';border-radius:2px;width:'+pct.toFixed(0)+"
                      "'%;\"></div></div>';"
                      "h+='<span style=\"width:60px;text-align:right;"
                      "color:#4fc3f7;font-size:9px;margin-left:4px;\">"
                      "'+d.g.toFixed(0)+' Wh</span>';"
                      "h+='<span style=\"width:35px;text-align:right;"
                      "color:#546e7a;font-size:9px;\">"
                      "'+d.n.toFixed(0)+'%</span>';"
                      "h+='</div>';}"
                      "c.innerHTML=h||'<div style=\"color:#555\">"
                      "Sem dados</div>';}"

                      "function atualizarPrevisao(){"
                      "var x=new XMLHttpRequest();"
                      "x.onreadystatechange=function(){"
                      "if(x.readyState==4&&x.status==200){"
                      "try{var d=JSON.parse(x.responseText);"
                      "var gr=document.getElementById('gerReal');"
                      "if(gr)gr.textContent=d.gerRealAgora.toFixed(0);"
                      "var mH=0,mA=0;"
                      "for(var i=0;i<d.horas.length;i++)"
                      "if(d.horas[i].g>mH)mH=d.horas[i].g;"
                      "for(var i=0;i<d.amanha.length;i++)"
                      "if(d.amanha[i].g>mA)mA=d.amanha[i].g;"
                      "gerarGrafico('graficoHoras',d.horas,mH);"
                      "gerarGrafico('graficoAmanha',d.amanha,mA);"
                      "}catch(e){}"
                      "}};"
                      "x.open('GET','/api/previsao',true);x.send();}"
                      "setInterval(atualizarPrevisao,10000);"
                      "atualizarPrevisao();");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Previs&atilde;o Solar",
                            conteudo, script));
}

// ============================================================
// PÁGINA: /config
// ============================================================
void handleConfig() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String msgHtml  = "";
    String abaAtiva = server.arg("aba");
    if (abaAtiva.length() == 0) abaAtiva = "wifi";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "salvar_wifi") {
            String modo = server.arg("modo");
            bool usarAP = (modo == "ap");
            String tz   = server.arg("timezone");
            if (tz.length() == 0) tz = "BRT3";

            if (usarAP) {
                salvarWifi("", "", "", "", "", "", tz, true);
                adicionarLog("Config WiFi: modo AP");
                msgHtml = "<div class='msg-ok'>&#10003; Reiniciando...</div>";
                server.send(200, "text/html; charset=utf-8",
                            gerarPagina("Config", msgHtml));
                delay(3000); ESP.restart(); return;
            }

            String ssid = server.arg("ssid");
            String wpass= server.arg("wpass");
            String ip   = server.arg("ip");
            String gw   = server.arg("gw");
            String mask = server.arg("mask");
            String dns  = server.arg("dns");

            if (ssid.length() == 0 || ip.length() < 7 || gw.length() < 7) {
                msgHtml = "<div class='msg-err'>&#10007; Preencha todos os campos</div>";
                abaAtiva = "wifi";
            } else {
                salvarWifi(ssid, wpass, ip, gw, mask, dns, tz, false);
                snprintf(tempBuf, sizeof(tempBuf), "WiFi: %s", ssid.c_str());
                adicionarLog(tempBuf);
                msgHtml = "<div class='msg-ok'>&#10003; Reiniciando...</div>";
                server.send(200, "text/html; charset=utf-8",
                            gerarPagina("Config", msgHtml));
                delay(3000); ESP.restart(); return;
            }

        } else if (acao == "alterar_senha") {
            String userAtual = server.arg("user_atual");
            String passAtual = server.arg("pass_atual");
            String passNova  = server.arg("pass_nova");
            String passConf  = server.arg("pass_conf");
            abaAtiva = "senha";

            if (!verificarLogin(userAtual, passAtual))
                msgHtml = "<div class='msg-err'>&#10007; Credenciais incorretas</div>";
            else if (passNova.length() < 4)
                msgHtml = "<div class='msg-err'>&#10007; M&iacute;nimo 4 caracteres</div>";
            else if (passNova != passConf)
                msgHtml = "<div class='msg-err'>&#10007; Senhas n&atilde;o conferem</div>";
            else {
                salvarCredenciais(userAtual, passNova);
                adicionarLog("Senha alterada");
                msgHtml = "<div class='msg-ok'>&#10003; Senha alterada!</div>";
            }

        } else if (acao == "exportar") {
            String json = "{\"versao\":\"" VERSAO "\",";
            json += "\"bateria\":{";
            json += "\"vnom\":"  + String(cfgBat.tensaoNominal, 2)       + ",";
            json += "\"ah\":"    + String(cfgBat.amperagem, 2)            + ",";
            json += "\"pdesc\":" + String(cfgBat.percDescargaSegura, 3)   + ",";
            json += "\"vmin\":"  + String(cfgBat.tensaoMinima, 2)         + "},";
            json += "\"estado\":{";
            json += "\"ahDisp\":" + String(estadoBat.ahDisponiveis, 3)    + ",";
            json += "\"ciclos\":" + String(estadoBat.ciclos, 2)            + ",";
            json += "\"ciclosP\":" + String(estadoBat.ciclosParciais, 4)   + "},";
            json += "\"contadores\":{";
            json += "\"kwhPai\":"  + String(contadores.kwhPainelTotal,  6) + ",";
            json += "\"kwhCas\":"  + String(contadores.kwhCasaTotal,    6) + ",";
            json += "\"kwhCar\":"  + String(contadores.kwhCargaTotal,   6) + ",";
            json += "\"kwhDes\":"  + String(contadores.kwhDescargaTotal,6) + "}}";
            server.sendHeader("Content-Disposition",
                              "attachment; filename=rrfs_backup.json");
            server.send(200, "application/json", json);
            adicionarLog("Exportacao realizada");
            return;

        } else if (acao == "importar") {
            String json   = server.arg("json_dados");
            String campos = server.arg("importar_campos");
            abaAtiva = "dados";
            bool ok = false;

            if (json.length() > 10) {
                if (campos == "tudo" || campos == "bateria") {
                    int p;
                    p = json.indexOf("\"vnom\":"); if (p>=0) cfgBat.tensaoNominal = json.substring(p+7).toFloat();
                    p = json.indexOf("\"ah\":");   if (p>=0) cfgBat.amperagem = json.substring(p+5).toFloat();
                    p = json.indexOf("\"pdesc\":"); if (p>=0) cfgBat.percDescargaSegura = json.substring(p+8).toFloat();
                    p = json.indexOf("\"vmin\":"); if (p>=0) cfgBat.tensaoMinima = json.substring(p+7).toFloat();
                    cfgBat.capacidadeTotal   = cfgBat.tensaoNominal * cfgBat.amperagem;
                    cfgBat.energiaDisponivel = cfgBat.capacidadeTotal * cfgBat.percDescargaSegura;
                    if      (cfgBat.tensaoNominal <= 13.0f) cfgBat.tensaoMaxima = 14.0f;
                    else if (cfgBat.tensaoNominal <= 26.0f) cfgBat.tensaoMaxima = 28.0f;
                    else cfgBat.tensaoMaxima = cfgBat.tensaoNominal * 1.094f;
                    salvarConfigBateria(); ok = true;
                }
                if (campos == "tudo" || campos == "estado") {
                    int p;
                    p = json.indexOf("\"ahDisp\":"); if (p>=0) estadoBat.ahDisponiveis = json.substring(p+9).toFloat();
                    p = json.indexOf("\"ciclos\":"); if (p>=0) estadoBat.ciclos = json.substring(p+9).toFloat();
                    p = json.indexOf("\"ciclosP\":"); if (p>=0) estadoBat.ciclosParciais = json.substring(p+10).toFloat();
                    estadoBat.ahMaximos = cfgBat.amperagem * cfgBat.percDescargaSegura;
                    salvarEstadoBateria(); ok = true;
                }
                if (campos == "tudo" || campos == "contadores") {
                    int p;
                    p = json.indexOf("\"kwhPai\":"); if (p>=0) contadores.kwhPainelTotal   = json.substring(p+9).toDouble();
                    p = json.indexOf("\"kwhCas\":"); if (p>=0) contadores.kwhCasaTotal     = json.substring(p+9).toDouble();
                    p = json.indexOf("\"kwhCar\":"); if (p>=0) contadores.kwhCargaTotal    = json.substring(p+9).toDouble();
                    p = json.indexOf("\"kwhDes\":"); if (p>=0) contadores.kwhDescargaTotal = json.substring(p+9).toDouble();
                    salvarContadores(); ok = true;
                }
            }
            msgHtml = ok
                ? "<div class='msg-ok'>&#10003; Importado!</div>"
                : "<div class='msg-err'>&#10007; JSON inv&aacute;lido</div>";
        }
    }

    // Escaneia redes se aba WiFi
    String opcoesRede = "";
    if (abaAtiva == "wifi") {
        WiFi.scanDelete();
        int n = WiFi.scanNetworks(false, false, false, 600);
        for (int i = 0; i < n; i++) {
            String s = WiFi.SSID(i);
            int r    = WiFi.RSSI(i);
            opcoesRede += "<option value='" + s + "'>" + s;
            if      (r > -50) opcoesRede += " [Excelente]";
            else if (r > -60) opcoesRede += " [Bom]";
            else if (r > -70) opcoesRede += " [Regular]";
            else              opcoesRede += " [Fraco]";
            snprintf(tempBuf, sizeof(tempBuf), " (%ddBm)", r);
            opcoesRede += String(tempBuf) + "</option>";
        }
        WiFi.scanDelete();
    }

    String cSsid, cPass, cIp, cGw, cMask, cDns, cTz;
    bool   cAp;
    carregarWifi(cSsid, cPass, cIp, cGw, cMask, cDns, cTz, cAp);

    String conteudo = msgHtml;

    // ── Abas ──
    conteudo += "<div style='display:flex;gap:4px;margin-bottom:12px;flex-wrap:wrap;'>";
    String abas[]     = {"wifi","senha","dados"};
    String abasNome[] = {"&#127760; WiFi","&#128274; Senha","&#128190; Dados"};
    for (int i = 0; i < 3; i++) {
        String est = (abaAtiva == abas[i])
            ? "background:#0277bd;color:#fff;"
            : "background:#1a1a1a;color:#90a4ae;border:1px solid #333;";
        conteudo += "<a href='/config?aba=" + abas[i]
                  + "' style='flex:1;text-align:center;"
                    "padding:8px 4px;border-radius:4px;"
                    "text-decoration:none;font-size:12px;"
                    "font-weight:bold;" + est + "'>"
                  + abasNome[i] + "</a>";
    }
    conteudo += "</div>";

    // ── ABA WiFi ──
    if (abaAtiva == "wifi") {
        conteudo += "<h3>&#127760; Configura&ccedil;&atilde;o de Rede</h3>";
        conteudo += "<form method='POST' action='/config?aba=wifi'>";
        conteudo += "<input type='hidden' name='acao' value='salvar_wifi'>";
        conteudo += "<label>Modo:</label>";
        conteudo += "<select name='modo' id='modoSel' onchange='toggleModo()'>";
        conteudo += "<option value='wifi'" + String(!cAp?" selected":"")
                  + ">Conectar ao WiFi</option>";
        conteudo += "<option value='ap'" + String(cAp?" selected":"")
                  + ">Ponto de Acesso (AP)</option>";
        conteudo += "</select>";
        conteudo += "<div id='divwifi'>";
        conteudo += "<label>Rede WiFi:</label>";
        conteudo += "<select name='ssid'>";
        conteudo += (opcoesRede.length() > 0)
            ? opcoesRede
            : "<option>Nenhuma rede encontrada</option>";
        conteudo += "</select>";
        conteudo += "<button type='button' class='btn-orange' "
                    "style='font-size:11px;margin-top:6px;' "
                    "onclick='location.href=\"/config?aba=wifi\"'>"
                    "&#128259; Atualizar Redes</button>";
        conteudo += "<label>Senha WiFi:</label>";
        conteudo += "<input type='password' name='wpass' maxlength='64' "
                    "placeholder='Senha do WiFi'>";
        conteudo += "<label>IP Fixo:</label>";
        conteudo += "<input type='text' name='ip' required value='"
                  + cIp + "' placeholder='Ex: 192.168.1.100'>";
        conteudo += "<label>Gateway:</label>";
        conteudo += "<input type='text' name='gw' required value='"
                  + cGw + "' placeholder='Ex: 192.168.1.1'>";
        conteudo += "<label>M&aacute;scara:</label>";
        conteudo += "<input type='text' name='mask' required value='" + cMask + "'>";
        conteudo += "<label>DNS:</label>";
        conteudo += "<input type='text' name='dns' required value='" + cDns + "'>";
        conteudo += "</div>";
        conteudo += "<label>Fuso Hor&aacute;rio:</label>";
        conteudo += "<select name='timezone'>";
        struct { const char* v; const char* n; } fusos[] = {
            {"BRT3","Brasil - Bras&iacute;lia (UTC-3)"},
            {"BRT4","Brasil - Manaus (UTC-4)"},
            {"BRT5","Brasil - Acre (UTC-5)"},
            {"UTC0","UTC (GMT+0)"},
            {"EST5EDT,M3.2.0,M11.1.0","EUA - Leste"},
            {"CET-1CEST,M3.5.0,M10.5.0/3","Europa Central"},
        };
        for (int i = 0; i < 6; i++) {
            conteudo += "<option value='" + String(fusos[i].v) + "'";
            if (cTz == fusos[i].v) conteudo += " selected";
            conteudo += ">" + String(fusos[i].n) + "</option>";
        }
        conteudo += "</select>";
        conteudo += "<button type='submit' class='btn-green'>"
                    "&#128190; Salvar e Reiniciar</button>";
        conteudo += "</form>";
        conteudo += "<script>function toggleModo(){"
                    "var m=document.getElementById('modoSel').value;"
                    "document.getElementById('divwifi').style.display="
                    "(m=='wifi'?'block':'none');}"
                    "toggleModo();</script>";
    }

    // ── ABA Senha ──
    else if (abaAtiva == "senha") {
        conteudo += "<h3>&#128274; Alterar Senha</h3>";
        conteudo += "<form method='POST' action='/config?aba=senha'>";
        conteudo += "<input type='hidden' name='acao' value='alterar_senha'>";
        conteudo += "<label>Usu&aacute;rio:</label>";
        conteudo += "<input type='text' name='user_atual' required "
                    "autocomplete='off' placeholder='Usu&aacute;rio'>";
        conteudo += "<label>Senha Atual:</label>";
        conteudo += "<input type='password' name='pass_atual' required "
                    "placeholder='Senha atual'>";
        conteudo += "<label>Nova Senha (m&iacute;n. 4 chars):</label>";
        conteudo += "<input type='password' name='pass_nova' required "
                    "placeholder='Nova senha'>";
        conteudo += "<label>Confirmar:</label>";
        conteudo += "<input type='password' name='pass_conf' required "
                    "placeholder='Confirme a nova senha'>";
        conteudo += "<button type='submit' class='btn-green'>"
                    "&#128274; Alterar Senha</button>";
        conteudo += "</form>";
    }

    // ── ABA Dados ──
    else if (abaAtiva == "dados") {
        conteudo += "<h3>&#128190; Exportar Dados</h3>";
        conteudo += "<form method='POST' action='/config?aba=dados'>";
        conteudo += "<input type='hidden' name='acao' value='exportar'>";
        conteudo += "<button type='submit' class='btn-green'>"
                    "&#8659; Exportar Backup JSON</button>";
        conteudo += "</form>";

        conteudo += "<h3>&#128228; Importar Dados</h3>";
        conteudo += "<form method='POST' action='/config?aba=dados'>";
        conteudo += "<input type='hidden' name='acao' value='importar'>";
        conteudo += "<label>Dados a importar:</label>";
        conteudo += "<select name='importar_campos'>";
        conteudo += "<option value='tudo'>Tudo</option>";
        conteudo += "<option value='bateria'>Config. bateria</option>";
        conteudo += "<option value='estado'>Estado (Ah + ciclos)</option>";
        conteudo += "<option value='contadores'>Contadores kWh</option>";
        conteudo += "</select>";
        conteudo += "<label>JSON do backup:</label>";
        conteudo += "<textarea name='json_dados' rows='5' "
                    "style='width:100%;background:#111;border:1px solid #444;"
                    "border-radius:4px;color:#e0e0e0;font-size:11px;"
                    "padding:8px;box-sizing:border-box;font-family:monospace;' "
                    "placeholder='{\"versao\":\"V7.0\",...}'></textarea>";
        conteudo += "<button type='submit' class='btn-orange'>"
                    "&#8657; Importar</button>";
        conteudo += "</form>";

        // ── NOVO: Botão OTA ──
        conteudo += "<h3>&#128268; Atualiza&ccedil;&atilde;o de Firmware</h3>";
        conteudo += "<div class='info' style='font-size:11px;'>"
                    "&#9432; Atualize o firmware do ESP32 sem precisar "
                    "de cabo USB.<br>"
                    "No Arduino IDE: <b>Sketch &rarr; Export Compiled Binary</b>"
                    "<br>Depois clique no bot&atilde;o abaixo e selecione "
                    "o arquivo <b>.bin</b>.</div>";
        conteudo += "<button onclick='location.href=\"/update\"' "
                    "style='width:100%;padding:14px;margin-top:10px;"
                    "background:#6a1b9a;color:#fff;border:none;"
                    "border-radius:4px;font-size:14px;font-weight:bold;"
                    "cursor:pointer;letter-spacing:1px;'>"
                    "&#128268; Atualizar Firmware (OTA)</button>";
        conteudo += "<div class='msg-warn' style='font-size:10px;margin-top:6px;'>"
                    "&#9888; N&atilde;o desligue o ESP32 durante a "
                    "atualiza&ccedil;&atilde;o!</div>";

        // ── Sistema ──
        conteudo += "<h3>&#128297; Sistema</h3>";
        conteudo += "<div class='info' style='font-size:11px;'>";
        snprintf(tempBuf, sizeof(tempBuf), "%u",
                 (unsigned)esp_get_free_heap_size()/1024);
        conteudo += "&#128190; RAM: <b>" + String(tempBuf) + " KB</b><br>";
        snprintf(tempBuf, sizeof(tempBuf), "%d/%d", histTotal, MAX_HISTORICO);
        conteudo += "&#128202; Hist: <b>" + String(tempBuf) + "</b><br>";
        conteudo += "&#9881; Vers&atilde;o: <b>" VERSAO "</b><br>";
        conteudo += "&#127760; Modo: <b>"
                  + String(modoAP ? "AP" : "WiFi") + "</b>";
        if (!modoAP)
            conteudo += "<br>&#128246; SSID: <b>" + WiFi.SSID()
                      + "</b><br>&#127760; IP: <b>"
                      + WiFi.localIP().toString() + "</b>";
        conteudo += "</div>";
        conteudo += "<div class='msg-warn' style='font-size:11px;'>"
                    "&#9888; Bot&atilde;o BOOT 10s = apaga tudo!</div>";
    }

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:10px;font-size:12px;'>&#8592; Voltar</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Config", conteudo));
}

// ============================================================
// PÁGINAS: /setup-login, /setup-wifi, /login, /logout
// ============================================================
void handleSetupLogin() {
    if (existeCredenciais() && existeWiFiConfig()) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi"); server.send(302); return;
    }

    String msgHtml = "";
    if (server.method() == HTTP_POST) {
        String user = server.arg("user");
        String pass = server.arg("pass");
        String conf = server.arg("conf");

        if      (user.length() < 3)
            msgHtml = "<div class='msg-err'>&#10007; M&iacute;nimo 3 caracteres</div>";
        else if (pass.length() < 4)
            msgHtml = "<div class='msg-err'>&#10007; Senha: m&iacute;nimo 4 chars</div>";
        else if (pass != conf)
            msgHtml = "<div class='msg-err'>&#10007; Senhas n&atilde;o conferem</div>";
        else {
            salvarCredenciais(user, pass);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location", "/setup-wifi");
            server.send(302); return;
        }
    }

    String conteudo = "<div class='info'>&#9432; Primeira configura&ccedil;&atilde;o.</div>";
    conteudo += "<h3>&#128274; Criar Conta</h3>" + msgHtml;
    conteudo += "<form method='POST' action='/setup-login'>";
    conteudo += "<label>Usu&aacute;rio (m&iacute;n. 3 chars):</label>";
    conteudo += "<input type='text' name='user' maxlength='32' required autocomplete='off'>";
    conteudo += "<label>Senha (m&iacute;n. 4 chars):</label>";
    conteudo += "<input type='password' name='pass' maxlength='32' required>";
    conteudo += "<label>Confirmar Senha:</label>";
    conteudo += "<input type='password' name='conf' maxlength='32' required>";
    conteudo += "<button type='submit'>Criar &#8594;</button>";
    conteudo += "</form>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Primeira Config", conteudo));
}

void handleSetupWifi() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login"); server.send(302); return;
    }
    if (existeWiFiConfig() && !logado) {
        server.sendHeader("Location", "/login"); server.send(302); return;
    }

    String msgHtml = "";
    if (server.method() == HTTP_POST) {
        String modo  = server.arg("modo");
        bool usarAP  = (modo == "ap");
        String tz    = server.arg("timezone");
        if (tz.length() == 0) tz = "BRT3";

        if (usarAP) {
            salvarWifi("", "", "", "", "", "", tz, true);
            String cont = "<div class='msg-ok'>&#10003; Reiniciando em 5s...</div>"
                          "<div class='info'>SSID: <b>RRFS Energia</b>"
                          "<br>IP: <b>10.0.0.1</b></div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - Configurado", cont));
            delay(5000); ESP.restart(); return;
        }

        String ssid  = server.arg("ssid");
        String wpass = server.arg("wpass");
        String ip    = server.arg("ip");
        String gw    = server.arg("gw");
        String mask  = server.arg("mask");
        String dns   = server.arg("dns");

        if (ssid.length() == 0 || ip.length() < 7) {
            msgHtml = "<div class='msg-err'>&#10007; Preencha todos os campos</div>";
        } else {
            salvarWifi(ssid, wpass, ip, gw, mask, dns, tz, false);
            String cont = "<div class='msg-ok'>&#10003; Reiniciando em 5s...</div>"
                          "<div class='info'>Rede: <b>" + ssid + "</b>"
                          "<br>IP: <b>" + ip + "</b>"
                          "<br>Acesse: <b>http://" + ip + "</b></div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - Configurado", cont));
            delay(5000); ESP.restart(); return;
        }
    }

    adicionarLog("Escaneando WiFi...");
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, false, false, 600);

    String conteudo = "<div class='info'>&#9432; Configure a rede WiFi.</div>";
    conteudo += "<h3>&#128246; Configurar Rede</h3>" + msgHtml;
    conteudo += "<form method='POST' action='/setup-wifi'>";
    conteudo += "<label>Modo:</label>";
    conteudo += "<select name='modo' id='modoSel' onchange='toggleModo()'>";
    conteudo += "<option value='wifi'>Conectar ao WiFi</option>";
    conteudo += "<option value='ap'>Ponto de Acesso (AP)</option>";
    conteudo += "</select>";
    conteudo += "<div id='divwifi'>";
    conteudo += "<label>Rede WiFi:</label><select name='ssid'>";
    if (n == 0) {
        conteudo += "<option>Nenhuma rede</option>";
    } else {
        for (int i = 0; i < n; i++) {
            String s = WiFi.SSID(i);
            int r = WiFi.RSSI(i);
            conteudo += "<option value='" + s + "'>" + s;
            if      (r > -50) conteudo += " [Excelente]";
            else if (r > -60) conteudo += " [Bom]";
            else if (r > -70) conteudo += " [Regular]";
            else              conteudo += " [Fraco]";
            snprintf(tempBuf, sizeof(tempBuf), " (%ddBm)", r);
            conteudo += String(tempBuf) + "</option>";
        }
    }
    WiFi.scanDelete();
    conteudo += "</select>";
    conteudo += "<button type='button' class='btn-orange' "
                "style='font-size:11px;margin-top:6px;' "
                "onclick='location.reload()'>&#128259; Atualizar</button>";
    conteudo += "<label>Senha:</label>";
    conteudo += "<input type='password' name='wpass' maxlength='64' "
                "placeholder='Senha (vazio se aberta)'>";
    conteudo += "<label>IP Fixo:</label>";
    conteudo += "<input type='text' name='ip' required "
                "placeholder='Ex: 192.168.1.100'>";
    conteudo += "<label>Gateway:</label>";
    conteudo += "<input type='text' name='gw' required "
                "placeholder='Ex: 192.168.1.1'>";
    conteudo += "<label>M&aacute;scara:</label>";
    conteudo += "<input type='text' name='mask' required value='255.255.255.0'>";
    conteudo += "<label>DNS:</label>";
    conteudo += "<input type='text' name='dns' required value='8.8.8.8'>";
    conteudo += "</div>";
    conteudo += "<label>Fuso Hor&aacute;rio:</label>";
    conteudo += "<select name='timezone'>";
    conteudo += "<option value='BRT3' selected>Brasil - Bras&iacute;lia (UTC-3)</option>";
    conteudo += "<option value='BRT4'>Brasil - Manaus (UTC-4)</option>";
    conteudo += "<option value='BRT5'>Brasil - Acre (UTC-5)</option>";
    conteudo += "<option value='UTC0'>UTC</option>";
    conteudo += "</select>";
    conteudo += "<button type='submit' class='btn-green'>"
                "&#128190; Salvar e Reiniciar</button>";
    conteudo += "</form>";
    if (logado)
        conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                    "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";
    conteudo += "<script>function toggleModo(){"
                "var m=document.getElementById('modoSel').value;"
                "document.getElementById('divwifi').style.display="
                "(m=='wifi'?'block':'none');}"
                "</script>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Configurar Rede", conteudo));
}

// ============================================================
// PÁGINA: /update - Atualização OTA via Web
// ============================================================
void handleUpdatePage() {
    if (!checarAuth()) return;

    adicionarLog("Pagina de atualizacao OTA acessada");

    String conteudo = "<h3>&#128268; Atualiza&ccedil;&atilde;o OTA (Web)</h3>";
    conteudo += "<div class='info' style='font-size:11px;'>"
                "&#9432; No Arduino IDE: Sketch &gt; Export Compiled Binary.<br>"
                "Selecione o arquivo .bin gerado e clique em Enviar.<br>"
                "O sistema reiniciar&aacute; sozinho ao terminar.</div>";

    conteudo += "<form id='formUpdate'>";
    conteudo += "<label>Arquivo .bin do firmware:</label>";
    conteudo += "<input type='file' name='firmware' accept='.bin' required id='fileInput'>";
    conteudo += "<button type='submit' class='btn-green' id='btnUpdate'>"
                "&#8679; Enviar e Atualizar</button>";
    conteudo += "</form>";

    conteudo += "<div id='progressWrap' style='display:none;margin-top:12px;'>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:4px;height:20px;'>";
    conteudo += "<div id='progressBar' style='height:100%;border-radius:4px;"
                "width:0%;background:#0277bd;transition:width 0.2s;'></div>";
    conteudo += "</div>";
    conteudo += "<div id='progressTxt' style='text-align:center;font-size:11px;"
                "color:#78909c;margin-top:4px;'>0%</div>";
    conteudo += "</div>";

    conteudo += "<div id='resultUpdate'></div>";

    conteudo += "<div class='msg-warn' style='font-size:11px;margin-top:12px;'>"
                "&#9888; N&Atilde;O desligue o ESP32 durante a atualiza&ccedil;&atilde;o! "
                "Mantenha a p&aacute;gina aberta at&eacute; o fim.</div>";

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";

    String script = F(
        "document.getElementById('formUpdate').addEventListener('submit',function(e){"
        "e.preventDefault();"
        "var fileInput=document.getElementById('fileInput');"
        "if(!fileInput.files.length)return;"
        "var file=fileInput.files[0];"
        "var fd=new FormData();"
        "fd.append('firmware',file);"
        "var xhr=new XMLHttpRequest();"
        "document.getElementById('progressWrap').style.display='block';"
        "document.getElementById('btnUpdate').disabled=true;"
        "document.getElementById('resultUpdate').innerHTML='';"
        "xhr.upload.onprogress=function(evt){"
        "if(evt.lengthComputable){"
        "var pct=Math.round((evt.loaded/evt.total)*100);"
        "document.getElementById('progressBar').style.width=pct+'%';"
        "document.getElementById('progressTxt').textContent=pct+'%';"
        "}};"
        "xhr.onreadystatechange=function(){"
        "if(xhr.readyState==4){"
        "document.getElementById('btnUpdate').disabled=false;"
        "if(xhr.status==200){"
        "document.getElementById('progressBar').style.width='100%';"
        "document.getElementById('progressTxt').textContent='100%';"
        "document.getElementById('resultUpdate').innerHTML="
        "'<div class=\"msg-ok\">&#10003; Atualizado! Reiniciando...</div>';"
        "setTimeout(function(){window.location.href='/';},8000);"
        "}else{"
        "document.getElementById('resultUpdate').innerHTML="
        "'<div class=\"msg-err\">&#10007; Falha: '+xhr.responseText+'</div>';"
        "}}};"
        "xhr.open('POST','/update',true);"
        "xhr.send(fd);"
        "});"
    );

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Atualiza&ccedil;&atilde;o OTA",
                            conteudo, script));
}

void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login"); server.send(302); return;
    }
    if (logado) {
        server.sendHeader("Location", "/"); server.send(302); return;
    }

    String msgHtml = "";
    if (server.method() == HTTP_POST) {
        String user = server.arg("user");
        String pass = server.arg("pass");
        if (verificarLogin(user, pass)) {
            logado = true;
            snprintf(tempBuf, sizeof(tempBuf), "Login OK: %s", user.c_str());
            adicionarLog(tempBuf);
            server.sendHeader("Location", "/"); server.send(302); return;
        } else {
            snprintf(tempBuf, sizeof(tempBuf), "Login FALHOU: %s", user.c_str());
            adicionarLog(tempBuf);
            delay(1000);
            msgHtml = "<div class='msg-err'>&#10007; Credenciais incorretas</div>";
        }
    }

    String conteudo = "<div class='info'>&#128274; Acesso seguro RRFS Energia</div>";
    conteudo += "<h3>Entrar</h3>" + msgHtml;
    conteudo += "<form method='POST' action='/login'>";
    conteudo += "<label>Usu&aacute;rio:</label>";
    conteudo += "<input type='text' name='user' required autocomplete='off'>";
    conteudo += "<label>Senha:</label>";
    conteudo += "<input type='password' name='pass' required>";
    conteudo += "<button type='submit'>&#128275; Entrar</button>";
    conteudo += "</form>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Login", conteudo));
}

void handleLogout() {
    if (logado) {
        adicionarLog("Logout realizado");
        logado = false;
    }
    server.sendHeader("Location", "/login");
    server.send(302);
}

// ============================================================
// ENDPOINT: /logs
// ============================================================
void handleLogs() {
    String json = "[";
    bool primeiro = true;
    for (int i = 0; i < totalLogs; i++) {
        int idx = ((int)logIndex - 1 - i + MAX_LOGS) % MAX_LOGS;
        if (logs[idx][0] == 0) continue;
        if (!primeiro) json += ",";
        primeiro = false;
        json += "\"";
        for (int c = 0; logs[idx][c]; c++) {
            char ch = logs[idx][c];
            if      (ch == '"')  json += "\\\"";
            else if (ch == '\\') json += "\\\\";
            else                 json += ch;
        }
        json += "\"";
    }
    json += "]";
    server.send(200, "application/json", json);
}

// ============================================================
// NOT FOUND
// ============================================================
void handleNotFound() {
    if (modoAP) {
        server.sendHeader("Location", "http://10.0.0.1/");
        server.send(302);
    } else {
        server.send(404, "text/plain", "404 - Pagina nao encontrada");
    }
}

// ============================================================
// CONFIGURAR ROTAS
// ============================================================
void configurarRotas() {
    server.on("/",            handleRoot);
    server.on("/login",       handleLogin);
    server.on("/logout",      handleLogout);
    server.on("/setup-login", handleSetupLogin);
    server.on("/setup-wifi",  handleSetupWifi);
    server.on("/bateria",     handleBateria);
    server.on("/rede",        handleRede);
    server.on("/historico",   handleHistorico);
    server.on("/config",      handleConfig);
    server.on("/previsao",    handlePrevisao);
    server.on("/logs",        handleLogs);
    server.on("/api/dados",   handleApiDados);
    server.on("/api/rede",    handleApiRede);
    server.on("/api/previsao",handleApiPrevisao);

    // ── NOVO: OTA via Web ──
    server.on("/update", HTTP_GET, handleUpdatePage);
    server.on("/update", HTTP_POST, handleUpdateResultado, handleUpdateUpload);

    server.onNotFound(handleNotFound);
    server.begin();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    setupLED();
    setupBotao();
    ledBlink();

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    adicionarLog("RRFS Energia " VERSAO " iniciando...");

    // ── Bateria ──
    if (existeBateriaConfig()) {
        carregarConfigBateria();
        carregarEstadoBateria();
        snprintf(tempBuf, sizeof(tempBuf),
                 "Bat: %.1fV %.0fAh cheia:%s",
                 cfgBat.tensaoNominal, cfgBat.amperagem,
                 estadoBat.carregada100 ? "sim" : "nao");
        adicionarLog(tempBuf);
    } else {
        cfgBat.tensaoNominal      = 12.8f;
        cfgBat.amperagem          = 100.0f;
        cfgBat.percDescargaSegura = 0.80f;
        cfgBat.tensaoMinima       = 12.0f;
        cfgBat.capacidadeTotal    = 1280.0f;
        cfgBat.energiaDisponivel  = 1024.0f;
        cfgBat.tensaoMaxima       = 14.0f;
        estadoBat.ahMaximos       = 80.0f;
        estadoBat.ahDisponiveis   = 80.0f;
        estadoBat.carregada100    = true;
        adicionarLog("Bateria: valores padrao");
    }

    // ── Contadores ──
    carregarContadores();
    snprintf(tempBuf, sizeof(tempBuf),
             "kWh: P=%.2f C=%.2f",
             contadores.kwhPainelTotal,
             contadores.kwhCasaTotal);
    adicionarLog(tempBuf);

    // ── Inversor ──
    carregarEstadoInversor();
    snprintf(tempBuf, sizeof(tempBuf),
             "Inv: prot=%lu rein=%lu",
             (unsigned long)estadoInv.totalProtecoes,
             (unsigned long)estadoInv.totalReiniciamentos);
    adicionarLog(tempBuf);

    // ── Rede Elétrica ──
    carregarConfigRede();
    carregarEstadoRede();
    if (existeConfigRede()) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Rede: RB=%.0f%% RA=%.0f%% LE=%.0f%% LI=%.0f%%",
                 cfgRede.percRedeBaixa, cfgRede.percRedeAlta,
                 cfgRede.percLampExt,   cfgRede.percLampInt);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Rede: valores padrao");
    }

    // ── Previsão Solar ──
    carregarConfigPrevisao();
    carregarCalibracaoHoraria();
    estadoPrev.bloqRedeAlta         = false;
    estadoPrev.bloqLampExt          = false;
    estadoPrev.redeAltaFoiDesligada = false;
    estadoPrev.lampExtFoiDesligada  = false;
    estadoPrev.dadosValidos         = false;
    estadoPrev.errosConsecutivos    = 0;
    estadoPrev.geracaoRealAcumHora  = 0;
    estadoPrev.geracaoEstAcumHora   = 0;
    estadoPrev.minutosNaHora        = 0;
    tPrevisao = 0;

    if (cfgPrev.configurado) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Prev: %.8f,%.8f kwp=%.2f",
                 (float)cfgPrev.latitude,
                 (float)cfgPrev.longitude,
                 cfgPrev.kwpInstalado);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Previsao: aguardando config");
    }

    // ── Hardware ──
    setupInversor();
    setupRede();

    // ── Sensores ──
    delay(500);
    calibrarZeroSensores();
    pinMode(PIN_BKP_BATERIA, INPUT);

    // ── WiFi ──
    bool temCred = existeCredenciais();
    bool temWifi = existeWiFiConfig();

    if (!temCred || !temWifi) {
        adicionarLog("Primeira vez - AP setup");
        iniciarAP();
    } else {
        String ssid, pass, ip, gw, mask, dns, tz;
        bool ap;
        carregarWifi(ssid, pass, ip, gw, mask, dns, tz, ap);

        if (ap) {
            adicionarLog("Modo AP pelo usuario");
            iniciarAP();
        } else {
            adicionarLog("Conectando WiFi...");
            if (!iniciarWiFi(ssid, pass, ip, gw, mask, dns)) {
                adicionarLog("Falha WiFi - AP fallback");
                iniciarAP();
            } else {
                iniciarNTP();
            }
        }
    }

    configurarRotas();
    adicionarLog("Servidor web ATIVO");
    logRAMSeNecessario();

    tLeitura   = millis();
    tHistorico = millis();
    tWifi      = millis();
    tRAMLog    = millis();

    adicionarLog("RRFS V7.0 pronto!");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    unsigned long agora = millis();

    // ── Tarefas de sistema ──
    verificarReset();
    atualizarLED();
    if (modoAP) dnsServer.processNextRequest();
    server.handleClient();

    // ── Ciclo principal: sensores + controles (1 segundo) ──
    if (agora - tLeitura >= INTERVALO_LEITURA) {
        float delta = (agora - tLeitura) / 1000.0f;
        tLeitura = agora;

        // 1. Lê sensores → correntePainel, correnteCasa,
        //                   tensaoBateriaEst, potenciaPainel
        //    (reutilizados por TODOS os sistemas abaixo)
        lerSensores();

        // 2. Atualiza bateria → ahDisponiveis, ahMaximos
        //    (reutilizados por controlarRede e previsão)
        if (existeBateriaConfig())
            atualizarEstadoBateria(delta);

        // 3. Calibração previsão
        //    → reutiliza: correntePainel, tensaoBateriaEst (de 1)
        //    → reutiliza: dados API já carregados
        //    → ZERO leituras ADC extras
        if (estadoPrev.dadosValidos && cfgPrev.configurado && !modoAP)
            calibrarFatorHorario();

        // 4. Controla inversor único
        //    → verifica status + proteção
        controlarInversor();

        // 5. Controla rede elétrica
        //    → reutiliza: ahDisponiveis (de 2)
        //    → reutiliza: tensaoBateriaEst (de 1)
        //    → reutiliza: energiaDisponivel (pré-calculado)
        //    → reutiliza: bloqRedeAlta/bloqLampExt (da previsão)
        if (existeBateriaConfig())
            controlarRede();
    }

    // ── Previsão Solar - busca API (1 hora) ──
    bool podeAtualizar = (!modoAP &&
                          WiFi.status() == WL_CONNECTED &&
                          ntpSincronizado &&
                          cfgPrev.configurado);

    bool deveAtualizar = podeAtualizar &&
                         ((tPrevisao == 0) ||
                          (agora - tPrevisao >= INTERVALO_PREVISAO));

    if (deveAtualizar) {
        tPrevisao = agora;
        adicionarLog("Prev: atualizando...");
        if (buscarDadosOpenMeteo()) {
            calcularGeracaoEstimada();
            calcularConsumoMedio();
            tomarDecisoesPrev();
            snprintf(tempBuf, sizeof(tempBuf),
                     "Prev OK: Hoje=%.0f Amanha=%.0fWh Acerto=%.1f%%",
                     estadoPrev.totalRestanteHoje,
                     estadoPrev.totalAmanha,
                     estadoPrev.taxaAcerto);
            adicionarLog(tempBuf);
        } else if (estadoPrev.errosConsecutivos > 0) {
            // Retry em 5 min se falhou
            tPrevisao = agora - INTERVALO_PREVISAO + 300000UL;
        }
        logRAMSeNecessario();
    }

    // ── Reavalia decisões de previsão (5 minutos) ──
    static unsigned long tDecisao = 0;
    if (estadoPrev.dadosValidos && ntpSincronizado &&
        (agora - tDecisao >= 300000UL)) {
        tDecisao = agora;
        tomarDecisoesPrev();
    }

    // ── Histórico (60 segundos) ──
    if (agora - tHistorico >= INTERVALO_HISTORICO) {
        tHistorico = agora;
        salvarHistorico();
        logRAMSeNecessario();
    }

    // ── WiFi check (30 segundos) ──
    if (!modoAP && agora - tWifi >= INTERVALO_WIFI_CHK) {
        tWifi = agora;
        if (WiFi.status() != WL_CONNECTED) {
            adicionarLog("WiFi perdido - reconectando...");
            WiFi.reconnect();

            unsigned long t = millis();
            while (WiFi.status() != WL_CONNECTED &&
                   millis() - t < 15000) {
                atualizarLED();
                controlarInversor();
                if (existeBateriaConfig()) controlarRede();
                delay(200);
            }

            if (WiFi.status() == WL_CONNECTED) {
                adicionarLog("WiFi reconectado!");
                ledFixo(true);
                iniciarNTP();
                tPrevisao = 0; // força re-busca
            } else {
                adicionarLog("Reconexao falhou - AP fallback");
                server.close();
                iniciarAP();
                configurarRotas();
            }
            logRAMSeNecessario();
        }
    }

    // ── Log RAM + salva estados (10 minutos) ──
    if (agora - tRAMLog >= INTERVALO_RAM_LOG) {
        tRAMLog = agora;
        uint32_t ram = (uint32_t)esp_get_free_heap_size() / 1024;
        snprintf(tempBuf, sizeof(tempBuf), "RAM livre: %u KB", ram);
        adicionarLog(tempBuf);
        contLogs = 0;
        salvarEstadoInversor();
        if (existeConfigRede())    salvarEstadoRede();
        if (cfgPrev.configurado)   salvarCalibracaoHoraria(false);
    }

    delay(1);
}

/*
 = *===============================================================================
 Documentação Completa - Equações do Sistema RRFS Energia V7.0
 ================================================================================

 SUMÁRIO DE MUDANÇAS V7.0 → V7.0:
 - Alterado: lógica do relé do inversor (LOW=LIGADO, HIGH=DESLIGADO)
 - Adicionado: máquina de estados de diagnóstico (8 fases)
 - Adicionado: detecção automática de curto em Rede Alta e Rede Baixa
 - Adicionado: bloqueio de carga com confirmação manual pelo usuário
 - Adicionado: teste automático após confirmação de conserto
 - Adicionado: timestamp de transitório natural ao ligar rede
 - Adicionado: campos de diagnóstico na API /api/rede
 - Atualizado: setRedeBaixa() e setRedeAlta() com log e timestamp
 - Atualizado: salvarEstadoInversor() e carregarEstadoInversor()
 - Atualizado: handleRede() com alertas e botões de confirmação
 - Atualizado: handleApiRede() com campos de diagnóstico

 COMO USAR ESTE DOCUMENTO:
 Cada seção indica na primeira linha o nome exato da função
 no código fonte para localização rápida.
 Exemplo:
 → Função: void controlarInversor()
 Use Ctrl+F no seu editor e busque exatamente esse texto.

 ================================================================================
 PARTE 1 - SENSORES E CONVERSÃO ADC
 ================================================================================

 1. CONVERSÃO ADC → TENSÃO
 → Função: float lerADCMedia(uint8_t pin)
 ─────────────────────────
 Formula:
 VOLTS_POR_BIT = VREF / RESOLUCAO
 VOLTS_POR_BIT = 3.3 / 4095 = 0.000806 V/bit

 Para que serve:
 Converte o valor digital do ADC (0 a 4095)
 em tensão real no pino do ESP32

 ──────────────────────────────────────────────────────────────────────────────

 2. DIVISOR DE TENSÃO (5V → 3.3V)
 → Constante: #define DIVISOR_TENSAO (3.3f / 5.0f)
 ──────────────────────────────────
 Formula:
 FATOR_DIVISOR = 3.3 / 5.0 = 0.66

 Para que serve:
 O sensor ACS758 opera em 5V
 O ESP32 suporta no máximo 3.3V
 O divisor resistivo reduz o sinal do sensor
 para não danificar o ESP32

 ──────────────────────────────────────────────────────────────────────────────

 3. SENSIBILIDADE REAL DO SENSOR (após divisor)
 → Constante: #define SENSIBILIDADE_50A / #define SENSIBILIDADE_100A
 ───────────────────────────────────────────────
 Formula:
 SENS_50A  = 0.040 V/A × 0.66 = 0.0264 V/A
 SENS_100A = 0.020 V/A × 0.66 = 0.0132 V/A

 Para que serve:
 Define quantos Volts o sinal muda
 para cada Ampere de corrente medida
 já considerando o divisor de tensão

 ──────────────────────────────────────────────────────────────────────────────

 4. MÉDIA DAS AMOSTRAS ADC
 → Função: float lerADCMedia(uint8_t pin)
 ──────────────────────────
 Formula:
 RAW_MEDIO = (amostra1 + ... + amostra64) / 64

 Para que serve:
 Reduz ruído eletrônico do ADC
 64 amostras aumentam a precisão da leitura
 Elimina picos e variações momentâneas

 ──────────────────────────────────────────────────────────────────────────────

 5. DELTA ADC (variação em relação ao zero)
 → Função: void lerSensores()
 ───────────────────────────────────────────
 Formula:
 DELTA_RAW = RAW_MEDIO - OFFSET_ZERO

 onde OFFSET_ZERO = leitura sem corrente
 (calibrado na inicialização)

 Para que serve:
 Encontra a variação real do sinal
 O sensor com 0A lê ~2047 (2.5V após divisor)
 O delta mostra quanto variou acima ou abaixo do zero
 positivo = corrente numa direção
 negativo = corrente na direção oposta

 ──────────────────────────────────────────────────────────────────────────────

 6. CONVERSÃO DELTA → CORRENTE REAL
 → Função: void lerSensores()
 ────────────────────────────────────
 Formula:
 VOLTS_DELTA  = DELTA_RAW × VOLTS_POR_BIT
 CORRENTE (A) = VOLTS_DELTA / SENSIBILIDADE

 Painel (50A):
 CORRENTE_PAINEL = VOLTS_DELTA / 0.0264

 Casa (100A):
 CORRENTE_CASA = VOLTS_DELTA / 0.0132

 Para que serve:
 Converte a variação de tensão do ADC
 em corrente elétrica real em Amperes

 ──────────────────────────────────────────────────────────────────────────────

 7. FILTRO DE RUÍDO
 → Função: void lerSensores()
 ───────────────────
 Formula:
 SE |CORRENTE| < 0.3A → CORRENTE = 0

 Para que serve:
 Elimina leituras falsas causadas por
 ruído eletrônico quando não há corrente real
 Evita acúmulo de erro no coulomb counting

 ──────────────────────────────────────────────────────────────────────────────

 8. DETECÇÃO DE INVERSÃO - PAINEL
 → Função: void lerSensores()
 ──────────────────────────────────
 Formula:
 SE CORRENTE_PAINEL > +1.0A → SENSOR INVERTIDO
 Ação: CORRENTE_PAINEL = |CORRENTE_PAINEL| + alerta

 SE CORRENTE_PAINEL entre -0.3A e -1.0A → normal
 Ação: CORRENTE_PAINEL = 0 (sem alerta)

 Para que serve:
 O painel sempre gera corrente numa direção
 Se o sinal sobe (positivo) significa fios invertidos
 Corrente reversa menor que 1A é o consumo
 normal do próprio controlador de carga à noite

 ──────────────────────────────────────────────────────────────────────────────

 9. DETECÇÃO DE INVERSÃO - CASA
 → Função: void lerSensores()
 ────────────────────────────────
 Formula:
 SE CORRENTE_CASA < -0.5A → SENSOR INVERTIDO
 Ação: CORRENTE_CASA = |CORRENTE_CASA| + alerta

 Para que serve:
 A casa sempre consome corrente numa direção
 Se o sinal for negativo os fios estão invertidos
 Protege contra cálculos errados de energia

 ──────────────────────────────────────────────────────────────────────────────

 10. CORRENTE DA BATERIA
 → Função: void lerSensores()
 ────────────────────────────
 Formula:
 CORRENTE_BATERIA = CORRENTE_PAINEL - CORRENTE_CASA

 Resultado positivo (+) = bateria CARREGANDO
 Resultado negativo (-) = bateria DESCARREGANDO

 Exemplos:
 Painel=20A  Casa=10A  → Bateria= +10A (carregando)
 Painel=30A  Casa=60A  → Bateria= -30A (descarregando)
 Painel= 0A  Casa=50A  → Bateria= -50A (descarregando)
 Painel=30A  Casa= 0A  → Bateria= +30A (carregando)

 Para que serve:
 Calcula a corrente da bateria sem sensor físico
 usando a lei de conservação de energia:
 tudo que o painel gera e a casa não consome
 vai para a bateria e vice-versa

 ================================================================================
 PARTE 2 - BATERIA E COULOMB COUNTING
 ================================================================================

 11. CAPACIDADE TOTAL DA BATERIA
 → Função: void carregarConfigBateria()
 ────────────────────────────────
 Formula:
 CAPACIDADE_TOTAL (Wh) = TENSAO_NOMINAL × AMPERAGEM

 Exemplo:
 12.8V × 100Ah = 1280 Wh = 1.28 kWh

 Para que serve:
 Calcula a energia máxima que a bateria
 pode armazenar quando completamente carregada

 ──────────────────────────────────────────────────────────────────────────────

 12. ENERGIA DISPONÍVEL SEGURA
 → Função: void carregarConfigBateria()
 ──────────────────────────────
 Formula:
 ENERGIA_DISPONIVEL (Wh) = CAPACIDADE_TOTAL
 × PERC_DESCARGA_SEGURA

 AH_MAXIMOS = AMPERAGEM × PERC_DESCARGA_SEGURA

 Exemplo:
 1280 Wh × 0.80 = 1024 Wh
 100Ah   × 0.80 =   80Ah utilizáveis com segurança

 Para que serve:
 Define o limite seguro de uso da bateria
 LiFePO4 pode descarregar até 80% sem danos
 Usar além deste limite reduz a vida útil

 ──────────────────────────────────────────────────────────────────────────────

 13. TENSÃO MÁXIMA DA BATERIA LiFePO4
 → Função: void carregarConfigBateria()
 ──────────────────────────────────────
 Formula:
 SE TENSAO_NOMINAL <= 13.0V → TENSAO_MAXIMA = 14.0V (4S)
 SE TENSAO_NOMINAL <= 26.0V → TENSAO_MAXIMA = 28.0V (8S)
 SE TENSAO_NOMINAL >  26.0V → TENSAO_MAXIMA = NOMINAL × 1.094

 Para que serve:
 Define a tensão de carga completa da bateria LiFePO4
 Cada célula carrega até 3.65V
 4S = 14.6V máximo (usa 14.0V como seguro)
 8S = 29.2V máximo (usa 28.0V como seguro)

 ──────────────────────────────────────────────────────────────────────────────

 14. COULOMB COUNTING - DELTA DE CARGA
 → Função: void atualizarEstadoBateria(float deltaSegundos)
 ───────────────────────────────────────
 Formula:
 DELTA_AH = CORRENTE_BATERIA
 × DELTA_TEMPO_SEGUNDOS / 3600

 Para que serve:
 Calcula quantos Ah foram adicionados ou removidos
 da bateria no intervalo entre duas leituras
 Dividir por 3600 converte segundos para horas

 Exemplo:
 Descarregando 10A por 30 segundos:
 DELTA_AH = -10 × 30 / 3600 = -0.0833 Ah

 ──────────────────────────────────────────────────────────────────────────────

 15. ATUALIZAÇÃO DOS AH DISPONÍVEIS
 → Função: void atualizarEstadoBateria(float deltaSegundos)
 ────────────────────────────────────
 Formula:
 AH_DISPONIVEIS = AH_DISPONIVEIS + DELTA_AH

 Limites de segurança:
 AH_DISPONIVEIS = constrain(AH_DISPONIVEIS, 0, AH_MAXIMOS)

 Para que serve:
 Rastreia a carga atual da bateria em tempo real
 Nunca permite valor negativo ou acima do máximo
 É o coração do sistema de gerenciamento de energia

 ──────────────────────────────────────────────────────────────────────────────

 16. DETECÇÃO DE BATERIA 100% CARREGADA
 → Função: void atualizarEstadoBateria(float deltaSegundos)
 ────────────────────────────────────────
 Formula (todas as condições juntas):
 SE CORRENTE_PAINEL > 0.5A
 E  |CORRENTE_BATERIA| < 0.3A
 E  (CORRENTE_CASA > 0.1A OU CORRENTE_PAINEL > 0.1A)
 ENTÃO → BATERIA COMPLETAMENTE CARREGADA

 Ação:
 AH_DISPONIVEIS = AH_MAXIMOS  (recalibra para 100%)
 CARREGADA_100  = verdadeiro

 Para que serve:
 Detecta quando o BMS cortou a carga da bateria
 O painel gera mas corrente vai toda para a casa
 Bateria não recebe nem entrega = cheia!
 Este evento RECALIBRA o coulomb counting
 zerando qualquer erro acumulado ao longo do tempo

 ──────────────────────────────────────────────────────────────────────────────

 17. SoC - ESTADO DE CARGA (percentual)
 → Função: void controlarRede() / void handleApiDados()
 ────────────────────────────────────────
 Formula:
 SoC (%) = (AH_DISPONIVEIS / AH_MAXIMOS) × 100
 Limites: entre 0% e 100%

 Para que serve:
 Mostra ao usuário a porcentagem de carga
 disponível na bateria de forma intuitiva

 ──────────────────────────────────────────────────────────────────────────────

 18. ESTIMATIVA DE TENSÃO - CURVA OCV LiFePO4
 → Função: float estimarTensaoBateria()
 ─────────────────────────────────────────────
 Formula:
 FATOR_ESCALA = TENSAO_NOMINAL / 12.8

 Tabela OCV × SoC para LiFePO4 (base 12.8V):
 ┌──────┬───────┬───────┬───────┬───────┬───────┐
 │ SoC  │  0%   │  10%  │  20%  │  30%  │  40%  │
 │ OCV  │ 10.00 │ 12.00 │ 12.50 │ 12.80 │ 12.90 │
 ├──────┼───────┼───────┼───────┼───────┼───────┤
 │ SoC  │  50%  │  60%  │  70%  │  80%  │  90%  │ 100%  │
 │ OCV  │ 13.00 │ 13.10 │ 13.20 │ 13.30 │ 13.50 │ 14.00 │
 └──────┴───────┴───────┴───────┴───────┴───────┘

 Interpolação linear entre pontos:
 INDEX     = inteiro(SoC × 10)
 FRACAO    = (SoC × 10) - INDEX
 OCV_BASE  = tabela[INDEX]
 + FRACAO × (tabela[INDEX+1] - tabela[INDEX])
 OCV_FINAL = OCV_BASE × FATOR_ESCALA

 Para que serve:
 Estima a tensão atual da bateria baseada no SoC
 sem necessidade de sensor de tensão físico
 Precisão: ±0.05V entre 20% e 80% de SoC

 ──────────────────────────────────────────────────────────────────────────────

 19. POTÊNCIAS
 → Função: void lerSensores()
 ──────────────
 Formula:
 POTENCIA_PAINEL  (W) = CORRENTE_PAINEL  × TENSAO_ESTIMADA
 POTENCIA_CASA    (W) = CORRENTE_CASA    × TENSAO_ESTIMADA
 POTENCIA_BATERIA (W) = CORRENTE_BATERIA × TENSAO_ESTIMADA

 Positivo = energia entrando na bateria (carregando)
 Negativo = energia saindo da bateria (descarregando)

 ──────────────────────────────────────────────────────────────────────────────

 20. ENERGIA ACUMULADA (Wh)
 → Função: void atualizarEstadoBateria(float deltaSegundos)
 ───────────────────────────────────────
 Formula:
 WH = POTENCIA × DELTA_TEMPO_SEGUNDOS / 3600

 Para que serve:
 Calcula a energia em Watt-hora produzida
 ou consumida em cada intervalo de leitura
 Base para contadores e histórico

 ──────────────────────────────────────────────────────────────────────────────

 21. CONTADORES TOTAIS (kWh)
 → Função: void atualizarEstadoBateria(float deltaSegundos)
 ────────────────────────────────────────────────────────────
 Formula:
 KWH_PAINEL_TOTAL   += WH_PAINEL   / 1000
 KWH_CASA_TOTAL     += WH_CASA     / 1000
 KWH_CARGA_TOTAL    += WH_BATERIA  / 1000  (só positivo)
 KWH_DESCARGA_TOTAL += |WH_BATERIA|/ 1000  (só negativo)

 Para que serve:
 Acumula toda a energia gerada e consumida
 desde o início do sistema
 Salvo na flash a cada 5 minutos

 ──────────────────────────────────────────────────────────────────────────────

 22. TEMPO ESTIMADO
 → Função: void handleApiDados()
 ───────────────────
 Formula A - Bateria DESCARREGANDO:
 TEMPO_HORAS = AH_DISPONIVEIS / |CORRENTE_BATERIA|

 Formula B - Bateria CARREGANDO:
 AH_FALTAM   = AH_MAXIMOS - AH_DISPONIVEIS
 TEMPO_HORAS = AH_FALTAM  / CORRENTE_BATERIA

 Conversão:
 HORAS   = inteiro(TEMPO_HORAS)
 MINUTOS = inteiro((TEMPO_HORAS - HORAS) × 60)

 ──────────────────────────────────────────────────────────────────────────────

 23. CICLOS DA BATERIA
 → Função: void atualizarEstadoBateria(float deltaSegundos)
 ──────────────────────
 Formula:
 CICLOS_PARCIAIS += |DELTA_AH| / AH_MAXIMOS

 SE CICLOS_PARCIAIS >= 1.0:
 CICLOS++
 CICLOS_PARCIAIS -= 1.0

 Para que serve:
 Conta quantos ciclos completos de descarga
 a bateria já realizou
 LiFePO4 suporta tipicamente 2000-6000 ciclos

 ──────────────────────────────────────────────────────────────────────────────

 24. ACUMULADORES DO HISTÓRICO
 → Função: void salvarHistorico()
 ──────────────────────────────────
 Formula:
 A cada segundo acumula:
 ACUM_WH_PAINEL   += WH_PAINEL
 ACUM_WH_CASA     += WH_CASA
 ACUM_WH_CARGA    += WH_BATERIA  (se positivo)
 ACUM_WH_DESCARGA += |WH_BATERIA|(se negativo)

 A cada 60 segundos salva no histórico:
 REGISTRO.whPainel      = int16(ACUM_WH_PAINEL   × 10)
 REGISTRO.whCasa        = int16(ACUM_WH_CASA      × 10)
 REGISTRO.whCargaBat    = int16(ACUM_WH_CARGA     × 10)
 REGISTRO.whDescargaBat = int16(ACUM_WH_DESCARGA  × 10)

 Multiplica por 10 para salvar 1 casa decimal
 em inteiro de 16 bits (economiza memória)

 ================================================================================
 PARTE 3 - INVERSOR ÚNICO (ATUALIZADO V7.0)
 ================================================================================

 25. LÓGICA DO INVERSOR ÚNICO
 → Função: void setInversor(bool ligado)
 ─────────────────────────────
 Arquitetura V7.0:
 Um único inversor de alta potência alimenta TUDO.

 LÓGICA DO RELÉ ATUALIZADA V7.0:
 ┌─────────────────────────────────────────────┐
 │ Relé LOW  (desenergizado) = Inversor LIGADO │
 │ Relé HIGH (energizado)    = Inversor DESLIG │
 └─────────────────────────────────────────────┘

 Por que invertido:
 Fail-safe: se ESP32 perder energia ou reiniciar
 → relé desenergiza (LOW)
 → Inversor PERMANECE LIGADO
 → Casa não perde energia por falha do ESP32
 O corte do inversor só ocorre quando
 o ESP32 comanda ativamente (HIGH)

 ──────────────────────────────────────────────────────────────────────────────

 26. LÓGICA DOS RELÉS V7.0
 → Funções: setInversor() / setRedeBaixa() /
 setRedeAlta() / setLampadaExterna() /
 setLampadaInterna()
 ───────────────────────────────────────────────────
 RELÉ INVERSOR ALTA (GPIO 27) - V7.0 INVERTIDO:
 LOW  = inversor LIGADO   ← NOVO V7.0
 HIGH = inversor DESLIGADO← NOVO V7.0

 RELÉ REDE BAIXA (GPIO 26) - FAIL SAFE:
 LOW  = rede baixa LIGADA
 HIGH = rede baixa DESLIGADA

 RELÉ REDE ALTA (GPIO 25) - FAIL SAFE:
 LOW  = rede alta LIGADA
 HIGH = rede alta DESLIGADA

 RELÉ LÂMPADA EXTERNA (GPIO 14) - FAIL SAFE:
 LOW  = lâmpada LIGADA
 HIGH = lâmpada DESLIGADA

 RELÉ LÂMPADA INTERNA (GPIO 12) - FAIL SAFE:
 LOW  = lâmpada LIGADA
 HIGH = lâmpada DESLIGADA

 ──────────────────────────────────────────────────────────────────────────────

 27. DETECÇÃO DE SINAL DO INVERSOR
 → Função: bool lerStatusInversor()
 ───────────────────────────────────
 Formula:
 SINAL_OK = (digitalRead(PIN_STATUS_INV_ALTA) == HIGH)

 Regras:
 Inversor DESLIGADO → ESP32 ignora o sinal (retorna false)
 Inversor LIGADO    → ESP32 monitora o sinal

 Para que serve:
 O LED de status do inversor indica funcionamento
 HIGH = inversor funcionando normalmente
 LOW  = inversor em proteção ou desligado

 ──────────────────────────────────────────────────────────────────────────────

 28. TRANSITÓRIO NATURAL AO LIGAR REDE
 → Função: void setRedeBaixa(bool) / void setRedeAlta(bool)
 ───────────────────────────────────────────────────────────
 Formula:
 tsLigouRede = millis()  (registra quando ligou a rede)

 emTransitorio = (tsLigouRede > 0 &&
 (agora - tsLigouRede) < TEMPO_TRANSITORIO_REDE)

 Onde TEMPO_TRANSITORIO_REDE = 1500ms

 Para que serve:
 É NORMAL o sinal do inversor parar brevemente
 quando uma carga (rede alta ou baixa) é conectada
 O pico de corrente momentâneo faz o inversor
 piscar o sinal por até 1.5 segundos
 O sistema IGNORA esta queda durante o transitório
 O que NÃO pode: sinal parar e não voltar após 1.5s

 ──────────────────────────────────────────────────────────────────────────────

 29. MÁQUINA DE ESTADOS DO DIAGNÓSTICO (V7.0)
 → Função: void controlarInversor()
 ──────────────────────────────────────────────
 8 Fases de diagnóstico:

 ┌────┬──────────────────┬──────────────────────────────┐
 │Fase│ Nome             │ O que acontece               │
 ├────┼──────────────────┼──────────────────────────────┤
 │  0 │ Normal           │ Monitoramento padrão         │
 │  1 │ Aguard.Sinal     │ Aguarda sinal após reiniciar │
 │  2 │ Teste s/RA       │ Testa sem Rede Alta          │
 │  3 │ Teste s/RB       │ Testa sem Rede Baixa         │
 │  4 │ Bloq.RA-Conf     │ Aguarda confirmação usuário  │
 │  5 │ Bloq.RB-Conf     │ Aguarda confirmação usuário  │
 │  6 │ Teste Final RA   │ Testa RA após conserto       │
 │  7 │ Teste Final RB   │ Testa RB após conserto       │
 └────┴──────────────────┴──────────────────────────────┘

 ──────────────────────────────────────────────────────────────────────────────

 30. FASE 0 - OPERAÇÃO NORMAL
 → Função: void controlarInversor() / case 0
 ───────────────────────────────────────────
 Formula:
 PROTECAO = (INVERSOR_LIGADO   == true)
 E  (SINAL_ATUAL       == LOW)
 E  (EM_TRANSITORIO    == false)
 E  (TEMPO_SEM_SINAL   >= TEMPO_AGUARDA_SINAL)

 Onde TEMPO_AGUARDA_SINAL = 2000ms

 Ação quando detecta proteção:
 1. Desliga inversor (HIGH no relé)
 2. Aguarda 3 segundos
 3. Liga inversor novamente (LOW no relé)
 4. Vai para FASE 1

 Para que serve:
 Monitora continuamente o sinal do inversor
 Só age após 2 segundos sem sinal
 Evita reação a quedas momentâneas normais

 ──────────────────────────────────────────────────────────────────────────────

 31. FASE 1 - AGUARDA SINAL APÓS REINICIAR
 → Função: void controlarInversor() / case 1
 ───────────────────────────────────────────
 Formula:
 SE sinal estável por >= TEMPO_ESTABILIDADE_OK (3s):
 → FASE 0 (recuperado!)
 → totalReiniciamentos++

 SE sinal instável (volta e cai) após transitório:
 → Desliga Rede Alta
 → Reinicia inversor
 → FASE 2 (diagnóstico)

 Para que serve:
 Verifica se o reinício simples resolveu o problema
 Se resolveu: volta ao normal
 Se não resolveu: inicia diagnóstico por eliminação

 ──────────────────────────────────────────────────────────────────────────────

 32. FASE 2 - TESTE SEM REDE ALTA
 → Função: void controlarInversor() / case 2
 ───────────────────────────────────────────
 Sequência:
 1. Rede Alta DESLIGADA
 2. Inversor reiniciado
 3. Aguarda sinal estável por 3 segundos

 SE estável SEM rede alta:
 → Problema estava na Rede Alta!
 → notificarCurtoRA()
 → FASE 4 (aguarda confirmação usuário)

 SE instável após TEMPO_DIAGNOSTICO_MAX (10s):
 → Problema não é na Rede Alta
 → Desliga Rede Baixa também
 → FASE 3

 Para que serve:
 Isolamento por eliminação:
 Retira a Rede Alta e testa se o inversor estabiliza
 Se estabilizar = carga da Rede Alta estava com problema

 ──────────────────────────────────────────────────────────────────────────────

 33. FASE 3 - TESTE SEM REDE BAIXA
 → Função: void controlarInversor() / case 3
 ───────────────────────────────────────────
 Sequência:
 1. Rede Alta JÁ desligada (da fase 2)
 2. Rede Baixa DESLIGADA
 3. Inversor reiniciado
 4. Aguarda sinal estável por 3 segundos

 SE estável SEM ambas as redes:
 → Problema estava na Rede Baixa!
 → notificarCurtoRB()
 → FASE 5 (aguarda confirmação usuário)

 SE instável após 10 segundos:
 → Problema no próprio INVERSOR
 → Desliga inversor por segurança
 → Avisa usuário para verificar fisicamente
 → Volta FASE 0 (sem religar automaticamente)

 ──────────────────────────────────────────────────────────────────────────────

 34. FASE 4 - AGUARDA CONFIRMAÇÃO REDE ALTA
 → Função: void controlarInversor() / case 4
 ───────────────────────────────────────────
 Estado:
 redeAltaBloqCurto       = true
 aguardandoConfirmacaoRA  = true
 Rede Alta mantida DESLIGADA

 Segurança:
 SE alguém tentar ligar Rede Alta manualmente
 → Sistema desliga novamente automaticamente
 → Exibe mensagem de bloqueio ativo

 Saída desta fase:
 Somente via confirmarConsertoRedeAlta()
 chamado pelo usuário na página /rede

 Persistência:
 Estado salvo na Flash
 Se ESP32 reiniciar → mantém bloqueio ativo

 ──────────────────────────────────────────────────────────────────────────────

 35. FASE 5 - AGUARDA CONFIRMAÇÃO REDE BAIXA
 → Função: void controlarInversor() / case 5
 ───────────────────────────────────────────
 Igual à FASE 4 porém para Rede Baixa:
 redeBaixaBloqCurto       = true
 aguardandoConfirmacaoRB  = true
 Rede Baixa mantida DESLIGADA

 ──────────────────────────────────────────────────────────────────────────────

 36. FASE 6 - TESTE FINAL REDE ALTA
 → Função: void controlarInversor() / case 6
 ───────────────────────────────────────────
 Acionado por: confirmarConsertoRedeAlta()

 Sequência:
 1. Usuário confirmou conserto na página /rede
 2. Sistema liga Rede Alta para teste
 3. Aguarda sinal estável por 3 segundos

 SE sinal estável:
 → redeAltaBloqCurto = false
 → aguardandoConfirmacaoRA = false
 → FASE 0 (sistema normal!)
 → Log: "Rede Alta LIBERADA"

 SE sinal instável após 10s:
 → Desliga Rede Alta novamente
 → Volta FASE 4 (ainda com problema)
 → Log: "TESTE FALHOU: Rede Alta ainda com problema!"

 ──────────────────────────────────────────────────────────────────────────────

 37. FASE 7 - TESTE FINAL REDE BAIXA
 → Função: void controlarInversor() / case 7
 ───────────────────────────────────────────
 Acionado por: confirmarConsertoRedeBaixa()

 Sequência:
 1. Usuário confirmou conserto na página /rede
 2. Liga inversor → aguarda 2s → liga Rede Baixa
 3. Aguarda sinal estável por 3 segundos

 SE sinal estável:
 → redeBaixaBloqCurto = false
 → FASE 0 (sistema normal!)
 → Liga Rede Alta se não houver outro bloqueio
 → Log: "Rede Baixa LIBERADA"

 SE sinal instável após 10s:
 → Volta FASE 5 (ainda com problema)

 ──────────────────────────────────────────────────────────────────────────────

 38. NOTIFICAÇÃO DE CURTO
 → Funções: void notificarCurtoRA() / void notificarCurtoRB()
 ────────────────────────────────────────────────────────────
 Ação:
 1. Seta flag de bloqueio (redeAltaBloqCurto/redeBaixaBloqCurto)
 2. Seta flag de confirmação pendente
 3. Incrementa contador de ocorrências
 4. Adiciona logs de alerta
 5. Salva estado na Flash imediatamente

 Persistência após reinício:
 carregarEstadoInversor() restaura:
 → Bloqueios ativos
 → Fase de diagnóstico (4 ou 5)
 → Mensagens de alerta exibidas novamente

 ──────────────────────────────────────────────────────────────────────────────

 39. CONFIRMAÇÃO DE CONSERTO
 → Funções: void confirmarConsertoRedeAlta()
 void confirmarConsertoRedeBaixa()
 ─────────────────────────────────────────────
 Chamadas por: handleRede() quando acao=="confirmar_ra"
 quando acao=="confirmar_rb"

 Fluxo confirmarConsertoRedeAlta():
 1. Verifica se aguardandoConfirmacaoRA == true
 2. Liga Rede Alta para teste
 3. Registra tsProtecao e tsUltimoSinal
 4. Vai para FASE 6

 Fluxo confirmarConsertoRedeBaixa():
 1. Verifica se aguardandoConfirmacaoRB == true
 2. Liga inversor → delay 2s → liga Rede Baixa
 3. Registra timestamps
 4. Vai para FASE 7

 ──────────────────────────────────────────────────────────────────────────────

 40. FLUXO COMPLETO DE DIAGNÓSTICO
 → Função: void controlarInversor()
 ─────────────────────────────────────
 Diagrama de fluxo:

 ┌──────────────────────────────────────────────────────────┐
 │ FASE 0: Normal                                          │
 │ Sinal para por 2s?                                      │
 └──────────────┬───────────────────────────────────────────┘
 │ SIM
 ┌──────────────▼───────────────────────────────────────────┐
 │ Reinicia inversor → FASE 1                              │
 │ Sinal voltou e ficou estável 3s?                        │
 └──────┬───────────────────┬──────────────────────────────┘
 │ SIM               │ NÃO (instável)
 │                   │
 ┌──────▼──────┐    ┌───────▼──────────────────────────────┐
 │ FASE 0 OK!  │    │ Desliga Rede Alta → FASE 2           │
 └─────────────┘    │ Sinal estável SEM Rede Alta?         │
 └───────┬──────────────────┬────────────┘
 │ SIM              │ NÃO (10s)
 │                  │
 ┌───────▼──────┐   ┌───────▼────────────┐
 │ CURTO RA!    │   │ Desliga Rede Baixa  │
 │ notifica     │   │ → FASE 3            │
 │ → FASE 4     │   │ Estável SEM ambas?  │
 │ (aguarda)    │   └───┬─────────────┬───┘
 └───────┬──────┘       │ SIM         │ NÃO
 │              │             │
 ┌───────▼──────┐ ┌─────▼──────┐ ┌───▼────────┐
 │ Usuário      │ │ CURTO RB!  │ │ PROBLEMA   │
 │ confirma     │ │ notifica   │ │ INVERSOR   │
 │ → FASE 6     │ │ → FASE 5   │ │ Desliga    │
 │ Teste OK?    │ │ (aguarda)  │ │ → FASE 0   │
 └───┬──────┬───┘ └─────┬──────┘ └────────────┘
 │OK    │FAIL        │
 │      │     ┌──────▼───────┐
 ┌───▼──┐ ┌─▼──┐  │ Usuário      │
 │LIVRE!│ │F.4 │  │ confirma     │
 └──────┘ └────┘  │ → FASE 7     │
 │ Teste OK?    │
 └───┬──────┬───┘
 │OK    │FAIL
 ┌───▼──┐ ┌─▼──┐
 │LIVRE!│ │F.5 │
 └──────┘ └────┘

 ================================================================================
 PARTE 4 - REDE ELÉTRICA
 ================================================================================

 41. ENERGIA DISPONÍVEL (base dos controles)
 → Função: void controlarRede()
 ─────────────────────────────────────────────
 Formula:
 WH_DISPONIVEL = AH_DISPONIVEIS × TENSAO_BATERIA_EST

 Para que serve:
 Valor base usado por TODOS os controles da rede
 Calculado UMA VEZ por segundo em controlarRede()
 Reutilizado pelos 4 controles de carga

 ──────────────────────────────────────────────────────────────────────────────

 42. LIMITES DE CORTE DA REDE ELÉTRICA
 → Função: void controlarRede()
 ─────────────────────────────────────────────────────
 Formula:
 WH_MIN_REDE_BAIXA = ENERGIA_DISPONIVEL × (PERC_REDE_BAIXA / 100)
 WH_MIN_REDE_ALTA  = ENERGIA_DISPONIVEL × (PERC_REDE_ALTA  / 100)
 WH_MIN_LAMP_EXT   = ENERGIA_DISPONIVEL × (PERC_LAMP_EXT   / 100)
 WH_MIN_LAMP_INT   = ENERGIA_DISPONIVEL × (PERC_LAMP_INT   / 100)

 Exemplo (12.8V 100Ah 80% descarga, padrões):
 ENERGIA_DISPONIVEL  = 12.8 × 100 × 0.80 = 1024 Wh
 WH_MIN_REDE_BAIXA   = 1024 × 0.10 = 102.4 Wh (10%)
 WH_MIN_REDE_ALTA    = 1024 × 0.30 = 307.2 Wh (30%)
 WH_MIN_LAMP_EXT     = 1024 × 0.25 = 256.0 Wh (25%)
 WH_MIN_LAMP_INT     = 1024 × 0.00 =   0.0 Wh  (0%)

 ──────────────────────────────────────────────────────────────────────────────

 43. HISTERESE DE RELIGAMENTO
 → Função: void controlarRede()
 ───────────────────────────────────────────────
 Formula:
 WH_RELIGAR_X = ENERGIA_DISPONIVEL
 × ((PERC_X + HISTERESE_REDE_PERC) / 100)

 Onde HISTERESE_REDE_PERC = 2%

 Exemplo:
 Corte   rede alta:  < 307.2 Wh (30%)
 Religar rede alta:  > 327.7 Wh (32%)

 ──────────────────────────────────────────────────────────────────────────────

 44. CONFIRMAÇÃO TEMPORAL DE 1 MINUTO
 → Função: void controlarRede()
 ───────────────────────────────────────────────────────
 Formula:
 CONFIRMAR_CORTE = (WH_DISPONIVEL  <= WH_MIN_X)
 E  (TEMPO_ABAIXO   >= 60000ms)

 CANCELAR = (WH_DISPONIVEL > WH_MIN_X)
 em qualquer momento antes dos 60s

 ──────────────────────────────────────────────────────────────────────────────

 45. SEQUÊNCIA DE CORTES POR PRIORIDADE
 → Função: void controlarRede()
 ───────────────────────────────────────────────
 Ordem de desligamento (menos → mais crítico):
 ┌─────────────────────────────────────────────────────────┐
 │ 1. Lâmpada Interna   → corte em  0%  (padrão)         │
 │ 2. Lâmpada Externa   → corte em 25%  (padrão)         │
 │ 3. Rede Alta         → corte em 30%  (padrão)         │
 │ 4. Rede Baixa+Inversor→corte em 10%  (padrão)         │
 └─────────────────────────────────────────────────────────┘

 Importante V7.0:
 Corte da Rede Baixa = desliga INVERSOR também (HIGH no relé)
 Religamento:
 Liga inversor (LOW) → delay 2s → liga Rede Baixa

 ──────────────────────────────────────────────────────────────────────────────

 46. PROTEÇÃO DA BATERIA POR CAMADAS
 → Funções: controlarRede() / atualizarEstadoBateria()
 ─────────────────────────────────────────────
 Camada 1 - Percentual do usuário (rede elétrica)
 Camada 2 - Percentual de descarga segura
 Camada 3 - BKP do controlador de carga

 Exemplo com 100Ah 80% descarga segura:
 Capacidade física total:   100Ah = 1280 Wh
 Energia disponível (80%):   80Ah = 1024 Wh
 Energia de proteção (20%):  20Ah =  256 Wh (nunca usada)

 Quando rede desliga a 10% de 1024 Wh = 102 Wh:
 Bateria ainda tem: 102 + 256 = 358 Wh físicos
 = ainda tem 27.9% da carga física real

 ──────────────────────────────────────────────────────────────────────────────

 47. CONTROLE DA LÂMPADA EXTERNA (3 modos)
 → Função: void controlarLampadaExterna()
 ───────────────────────────────────────────
 Prioridade: Previsão > Energia > Modo configurado

 Modo 0: Apenas energia controla
 Modo 1: Horário Manual (cruza meia-noite suportado)
 Modo 2: Automático por Painel (reutiliza correntePainel)

 ──────────────────────────────────────────────────────────────────────────────

 48. REUTILIZAÇÃO DE CÁLCULOS
 → Funções: loop() / controlarRede() / calibrarFatorHorario()
 ───────────────────────────────────────────────────────────────────────────────
 Sequência no loop (1x por segundo):

 lerSensores()           → tensaoBateriaEst, correntePainel
 atualizarEstadoBateria()→ ahDisponiveis (reutiliza tensaoBateriaEst)
 calibrarFatorHorario()  → reutiliza correntePainel + tensaoBateriaEst
 controlarInversor()     → 1 digitalRead, ZERO cálculos matemáticos
 controlarRede()         → reutiliza ahDisponiveis + tensaoBateriaEst

 Total: apenas 5 operações simples por segundo para controle de rede

 ================================================================================
 PARTE 5 - PREVISÃO SOLAR
 ================================================================================

 49. API METEOROLÓGICA - OPEN-METEO
 → Função: bool buscarDadosOpenMeteo()
 ────────────────────────────────────
 API: Open-Meteo (gratuita, sem chave)
 Dados: shortwave_radiation, cloudcover, precipitation_probability
 Atualização: 1x por hora
 Retry em falha: 5 minutos

 ──────────────────────────────────────────────────────────────────────────────

 50. PRECISÃO DAS COORDENADAS
 → Função: void salvarConfigPrevisao()
 ──────────────────────────────────────
 Tipo: double (64 bits) → precisão de 0.01mm
 float (32 bits) → erro de ~11 metros (insuficiente)

 ──────────────────────────────────────────────────────────────────────────────

 51. GERAÇÃO ESTIMADA BRUTA
 → Função: void calcularGeracaoEstimada()
 ───────────────────────────────────────────
 Formula:
 GER_BRUTA = RADIACAO × KWP × EFICIENCIA_PADRAO (0.75)

 Eficiência 0.75 inclui perdas de:
 inversor (8%), cabos (3%), temperatura (7%),
 sujeira (5%), orientação (5%)

 ──────────────────────────────────────────────────────────────────────────────

 52. FATOR DE CALIBRAÇÃO HORÁRIO
 → Função: void calibrarFatorHorario()
 ──────────────────────────────────────────────────────────
 Formula:
 FATOR_MEDIDO = GER_REAL_ACUMULADA / GER_EST_ACUMULADA
 Limites: constrain(0.20, 1.30)

 Reutiliza: correntePainel + tensaoBateriaEst (ZERO ADC extras)

 ──────────────────────────────────────────────────────────────────────────────

 53. MÉDIA MÓVEL PONDERADA DO FATOR
 → Função: void calibrarFatorHorario()
 ────────────────────────────────────────
 Formula:
 SE amostras > 10: PESO_HIST=0.85 PESO_NOVO=0.15
 SENÃO:            PESO_HIST=0.70 PESO_NOVO=0.30

 FATOR_NOVO = (FATOR_ANTIGO × PESO_HIST)
 + (FATOR_MEDIDO × PESO_NOVO)

 ──────────────────────────────────────────────────────────────────────────────

 54. DECISÕES AUTOMÁTICAS COM PREVISÃO
 → Função: void tomarDecisoesPrev()
 ───────────────────────────────────────────────────────────────────────────
 Decisão 1 - Lâmpada Externa noturna:
 HORA>=22 ou HORA<6
 E geracaoAmanha < consumoMedio × 0.60
 E SoC < 35%
 → bloqLampExt = true

 Decisão 2 - Rede Alta após meia-noite:
 HORA>=0 e HORA<6
 E (geracaoAmanha < consumoMedio × 0.50 OU SoC < 30%)
 → bloqRedeAlta = true

 Decisão 3 - Rede Alta antecipada crítico:
 HORA>=21
 E geracaoAmanha < consumoMedio × 0.35
 E SoC < 25%
 → bloqRedeAlta = true

 Decisão 4 - Religar ao amanhecer:
 HORA>=6 e HORA<12
 E redeAltaFoiDesligada == true
 E whDisponivel > whMinRedeAltaOn
 → bloqRedeAlta = false

 ──────────────────────────────────────────────────────────────────────────────

 55. CONSUMO MÉDIO DIÁRIO
 → Função: void calcularConsumoMedio()
 ─────────────────────────────────────
 Formula (média ponderada 7 dias):
 PESO_DIA[D] = 7 - D  (hoje=7, ontem=6, ...)
 CONSUMO_MEDIO = Σ(WH_DIA × PESO) / Σ(PESO)

 ──────────────────────────────────────────────────────────────────────────────

 56. PERSISTÊNCIA DA CALIBRAÇÃO
 → Função: void salvarCalibracaoHoraria(bool forcado)
 ─────────────────────────────────────────────────────
 Namespace "rrfspc": f0..f23, a0..a23, acerto, consmed

 Política de escrita:
 |fatorAtual - fatorAntesSalvar| > 0.02
 OU millis() - tsUltimoSalvamento >= 3600000
 OU forcado == true

 Vida estimada da flash: 11+ anos

 ──────────────────────────────────────────────────────────────────────────────

 57. REAVALIAÇÃO DAS DECISÕES
 → Função: loop() - bloco "static unsigned long tDecisao"
 ─────────────────────────────────────────────────────────
 Intervalo: 5 minutos
 Reutiliza: soc, whDisp, geracaoAmanha (já calculados)
 Custo: apenas comparações simples, sem HTTP

 ================================================================================
 RESUMO VISUAL - FLUXO COMPLETO DO SISTEMA V7.0
 ================================================================================

 ┌─────────────────────────────────────────────────────────┐
 │              SENSORES (1x/segundo)                      │
 │  lerSensores()                                          │
 │  Painel→corrente  Casa→corrente  BKP→digital            │
 └──────────────────┬──────────────────────────────────────┘
 │ reutiliza
 ┌──────────────────▼──────────────────────────────────────┐
 │  atualizarEstadoBateria()                               │
 │  ahDisponiveis  tensaoBateriaEst  SoC%  whDisponivel    │
 └──────────────────┬──────────────────────────────────────┘
 │ reutiliza
 ┌──────────┴──────────────────┐
 │                             │
 ┌───────▼──────┐             ┌────────▼─────────┐
 │controlarInv()│             │  controlarRede() │
 │              │             │                  │
 │ FASE 0..7    │             │ 4 cargas         │
 │ diagnóstico  │             │ 1 min confirm.   │
 │ curto RA/RB  │             │ histerese 2%     │
 │ LOW=LIGADO   │             │ previsão solar   │
 └───────┬──────┘             └────────┬─────────┘
 │                             │
 ┌───────▼─────────────────────────────▼──────────┐
 │                   RELÉS V7.0                   │
 │  InvAlta(LOW=ON)  RedeBaixa   RedeAlta         │
 │  LampExterna      LampInterna                  │
 │  (5 relés - todos FAIL SAFE)                   │
 └────────────────────────────────────────────────┘

 ┌─────────────────────────────────────────────────────────┐
 │         PREVISÃO SOLAR (1x/hora)                        │
 │  buscarDadosOpenMeteo()                                 │
 │  calcularGeracaoEstimada()                              │
 │  calcularConsumoMedio()                                 │
 │  tomarDecisoesPrev() → flags bloqRedeAlta/bloqLampExt  │
 └─────────────────────┬───────────────────────────────────┘
 │ reavalia a cada 5 min
 ┌─────────────────────▼───────────────────────────────────┐
 │         CALIBRAÇÃO (1x/segundo)                         │
 │  calibrarFatorHorario()                                 │
 │  → fatorHorario[24] → flash se mudou >2%               │
 └─────────────────────────────────────────────────────────┘

 ================================================================================
 PINAGEM COMPLETA DO SISTEMA V7.0
 ================================================================================

 ADC SENSORES:
 GPIO 34 → ACS758 50A   (corrente painel solar)
 GPIO 35 → ACS758 100A  (corrente casa)
 GPIO 32 → Sinal BKP controlador de carga
 HIGH = bateria com energia (normal)
 LOW  = bateria 100% descarregada (crítico)

 RELÉ INVERSOR ÚNICO V7.0 (LÓGICA INVERTIDA):
 GPIO 27 → Relé Inversor Alta
 LOW  = Inversor LIGADO   ← NOVO V7.0
 HIGH = Inversor DESLIGADO← NOVO V7.0

 STATUS INVERSOR:
 GPIO 36 → LED Status Inversor (HIGH=OK LOW=problema)

 RELÉS REDE ELÉTRICA (FAIL SAFE: LOW=LIGADO):
 GPIO 26 → Relé Rede Baixa    (LOW=LIGADA  HIGH=DESLIGADA)
 GPIO 25 → Relé Rede Alta     (LOW=LIGADA  HIGH=DESLIGADA)
 GPIO 14 → Relé Lamp. Externa (LOW=LIGADA  HIGH=DESLIGADA)
 GPIO 12 → Relé Lamp. Interna (LOW=LIGADA  HIGH=DESLIGADA)

 LED SISTEMA:
 GPIO  2 → LED Azul (blink=AP  fixo=WiFi)
 GPIO  0 → Botão BOOT (10s = reset total)

 ================================================================================
 PÁGINAS WEB DO SISTEMA V7.0
 ================================================================================

 /             → Painel principal (dados em tempo real)
 /bateria      → Config e status da bateria LiFePO4
 /rede         → Config, controle e diagnóstico da rede
 NOVO V7.0: alertas de curto + botões confirmação
 /previsao     → Previsão solar + calibração horária
 /historico    → Histórico 24h com filtro por data
 /config       → WiFi, senha, backup/restore
 /login        → Autenticação
 /logout       → Encerrar sessão
 /setup-login  → Primeira configuração (credenciais)
 /setup-wifi   → Primeira configuração (rede)

 APIs JSON:
 /api/dados      → sensores + bateria + rede + previsão (2s)
 /api/rede       → estado rede + diagnóstico inversor (2s)
 NOVO V7.0: bloqRA, bloqRB, confRA, confRB,
faseDiag, curtosRA, curtosRB,
totalProt, totalRein
/api/previsao   → previsão completa + arrays 48h (10s)
/logs           → logs do sistema (4s)

================================================================================
NAMESPACES DAS PREFERENCES (flash) V7.0
================================================================================

"rrfs"   → WiFi, credenciais, timezone, config bateria
"rrfsb"  → Estado da bateria (Ah, ciclos)
"rrfsc"  → Contadores kWh totais
"rrfsi"  → Estado inversor - ATUALIZADO V7.0:
total_prot, total_rein (existentes)
total_cra, total_crb   (NOVOS: curtos RA/RB)
bloq_ra, bloq_rb       (NOVOS: bloqueios ativos)
conf_ra, conf_rb       (NOVOS: confirmações pendentes)
fase_diag              (NOVO:  fase atual 0-7)
"rrfsr"  → Config rede elétrica (% de corte)
"rrfsrs" → Estado rede (total de cortes por carga)
"rrfsp"  → Config previsão (lat, lon, kWp)
"rrfspc" → Calibração horária (fatores[24], acerto)

================================================================================
CONSTANTES DO SISTEMA V7.0
================================================================================

ADC E SENSORES:
ADC_RESOLUCAO             = 4095
ADC_VREF                  = 3.3V
DIVISOR_TENSAO            = 0.66  (3.3/5.0)
SENSIBILIDADE_50A         = 0.0264 V/A
SENSIBILIDADE_100A        = 0.0132 V/A
AMOSTRAS_ADC              = 64

SISTEMA:
INTERVALO_LEITURA         = 1000ms
INTERVALO_HISTORICO       = 60000ms
INTERVALO_WIFI_CHK        = 30000ms
INTERVALO_RAM_LOG         = 600000ms
MAX_HISTORICO             = 1440 registros (24h)
TEMPO_RESET_MS            = 10000ms

INVERSOR ÚNICO V7.0:
TEMPO_RESTART_INV_MS      = 3000ms  (desligado antes de religar)
TEMPO_DEBOUNCE_STATUS     = 200ms   (herdado, não usado na V7.0)
TEMPO_TRANSITORIO_REDE    = 1500ms  (NOVO: pico normal ao ligar rede)
TEMPO_AGUARDA_SINAL       = 2000ms  (NOVO: aguarda antes de agir)
TEMPO_ESTABILIDADE_OK     = 3000ms  (NOVO: tempo para confirmar OK)
TEMPO_DIAGNOSTICO_MAX     = 10000ms (NOVO: timeout de cada fase)

REDE ELÉTRICA:
TEMPO_DECISAO_REDE_MS     = 60000ms
HISTERESE_REDE_PERC       = 2.0%

PREVISÃO SOLAR:
INTERVALO_PREVISAO        = 3600000ms (1 hora)
EFICIENCIA_PADRAO         = 0.75
FATOR_CAL_MIN             = 0.20
FATOR_CAL_MAX             = 1.30
PESO_HISTORICO            = 0.70
PESO_NOVO                 = 0.30
MIN_RADIACAO_CALIBRAR     = 50.0 W/m²
MIN_CORRENTE_CALIBRAR     = 0.5A
HORAS_PREVISAO            = 48
HORA_AMANHECER            = 6
HORA_ANTECIPAR_CORTE      = 21
DELTA_FATOR_SALVAR        = 0.02
HTTP_TIMEOUT_MS           = 15000ms

================================================================================
ESTRUTURA EstadoInversor - ATUALIZADA V7.0
================================================================================

bool   invLigado                → estado lógico do inversor
bool   statusInv                → true = sinal OK recebido
bool   emProtecao               → true = em processo de recuperação
bool   testando                 → reservado para uso futuro
bool   redeAltaBloqCurto        → NOVO: RA bloqueada por curto
bool   redeBaixaBloqCurto       → NOVO: RB bloqueada por curto
bool   aguardandoConfirmacaoRA  → NOVO: aguarda usuário confirmar
bool   aguardandoConfirmacaoRB  → NOVO: aguarda usuário confirmar
unsigned long tsProtecao        → quando entrou em proteção
unsigned long tsStatusInv       → último status válido (legado)
unsigned long tsUltimoSinal     → NOVO: último sinal HIGH recebido
unsigned long tsLigouRede       → NOVO: quando ligou rede (transitório)
uint32_t totalProtecoes         → total de proteções detectadas
uint32_t totalReiniciamentos    → total de reinícios bem-sucedidos
uint32_t totalCurtosRA          → NOVO: total de curtos na Rede Alta
uint32_t totalCurtosRB          → NOVO: total de curtos na Rede Baixa
uint8_t  faseTesteDiagnostico   → NOVO: fase atual (0-7)

================================================================================
*/
