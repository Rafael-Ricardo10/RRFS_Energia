/*
 * RRFS Energia V4.0
 * Sistema de Gerenciamento de Energia Solar
 * Sensores: ACS758 50A (Painel) + ACS758 100A (Casa)
 * Bateria: LiFePO4 com estimativa por coulomb counting
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <time.h>
#include <Arduino.h>
#include <HTTPClient.h>

// ============================================================
// PINAGEM Sensores
// ============================================================
#define LED_AZUL          2
#define BOTAO_BOOT        0
#define PIN_SENSOR_PAINEL 34   // ACS758 50A  - ADC1_CH6
#define PIN_SENSOR_CASA   35   // ACS758 100A - ADC1_CH7
#define PIN_BKP_BATERIA   32   // Sinal relé controlador de carga (3.3V)

// ============================================================
// PINAGEM - INVERSORES
// ============================================================

#define PIN_RELE_INV_BAIXA      26   // Relé Inversor Baixa (HIGH=desligado, LOW=ligado)
#define PIN_RELE_INV_ALTA       27   // Relé Inversor Alta (HIGH=ligado, LOW=desligado)
#define PIN_STATUS_INV_BAIXA    33   // Sinal LED status Inversor Baixa
#define PIN_STATUS_INV_ALTA     36   // Sinal LED status Inversor Alta (VP)

// ============================================================
// PINAGEM - REDE ELÉTRICA
// ============================================================
#define PIN_RELE_LAMP_EXTERNA   14   // Relé Lâmpada Externa
// HIGH = lâmpada DESLIGADA
// LOW  = lâmpada LIGADA
#define PIN_RELE_LAMP_INTERNA   12   // Relé Lâmpada Interna
// HIGH = lâmpada DESLIGADA
// LOW  = lâmpada LIGADA
#define PIN_RELE_REDE_ALTA      25   // Relé Rede Alta
// (HIGH=Inversor Alta, LOW=Inversor Baixa)

// ============================================================
// CONSTANTES DO SISTEMA
// ============================================================
#define MAX_LOGS            20
#define LOG_MAX_LEN         110
#define DNS_PORT            53
#define TEMPO_RESET_MS      10000
#define ADC_RESOLUCAO       4095.0f
#define ADC_VREF            3.3f
#define DIVISOR_TENSAO      (3.3f / 5.0f)   // Divisor resistivo 5V -> 3.3V
#define ZERO_CORRENTE_RAW   (ADC_RESOLUCAO * (2.5f / 5.0f) * DIVISOR_TENSAO / ADC_VREF)
// 2.5V no sensor -> após divisor 1.65V -> em RAW ~2047
#define SENSIBILIDADE_50A   (0.040f * DIVISOR_TENSAO)  // 40mV/A -> ajustado
#define SENSIBILIDADE_100A  (0.020f * DIVISOR_TENSAO)  // 20mV/A -> ajustado
#define AMOSTRAS_ADC        64
#define INTERVALO_LEITURA   1000UL    // ms
#define INTERVALO_HISTORICO 60000UL   // salva histórico a cada 60s
#define INTERVALO_WIFI_CHK  30000UL   // verifica WiFi a cada 30s
#define INTERVALO_RAM_LOG   600000UL  // log RAM a cada 10 min
#define LOG_POR_RAM         10        // mostra RAM a cada 10 logs
#define MAX_HISTORICO       1440      // 1440 registros = 24h em minutos
#define VERSAO              "V4.0"

// ============================================================
// CONSTANTES INVERSORES
// ============================================================
#define HISTERESE_PERC          5.0f    // % de histerese para evitar chaveamento rápido
#define TEMPO_ESPERA_ALTA_MS    60000UL // 1 minuto monitorando antes de transferir de volta
#define TEMPO_RESTART_INV_MS    3000UL  // 3 segundos esperando status antes de reiniciar
#define TEMPO_DEBOUNCE_STATUS   200UL   // debounce do sinal de status (ms)
#define TEMPO_MIN_CHAVEAMENTO   5000UL  // tempo mínimo entre chaveamentos (ms)

// ============================================================
// CONSTANTES REDE ELÉTRICA
// ============================================================
#define TEMPO_DECISAO_REDE_MS   60000UL  // 1 minuto antes de agir
#define HISTERESE_REDE_PERC     2.0f     // 2% histerese para evitar oscilação

// ============================================================
// CONSTANTES - PREVISÃO SOLAR
// ============================================================
#define INTERVALO_PREVISAO      3600000UL  // 1 hora em ms
#define EFICIENCIA_PADRAO       0.75f      // rendimento painel+inversor
#define FATOR_CAL_MIN           0.20f      // fator mínimo calibração
#define FATOR_CAL_MAX           1.30f      // fator máximo calibração
#define PESO_HISTORICO          0.70f      // peso do fator histórico
#define PESO_NOVO               0.30f      // peso da nova medição
#define MIN_RADIACAO_CALIBRAR   50.0f      // W/m² mínimo para calibrar
#define MIN_CORRENTE_CALIBRAR   0.5f       // A mínimo painel para calibrar
#define HORAS_PREVISAO          48         // horas retornadas pela API
#define SOC_RELIGAR_REDE_ALTA   50.0f      // % SoC para religar rede alta
#define HORA_MEIA_NOITE         0          // hora de verificar desligar
#define HORA_AMANHECER          6          // hora de verificar religar
#define HORA_ANTECIPAR_CORTE    21         // hora para corte antecipado
#define DELTA_FATOR_SALVAR      0.02f      // diferença mínima para salvar
#define HTTP_TIMEOUT_MS         15000      // timeout requisição HTTP
#define BUFFER_HTTP             6144       // buffer resposta JSON (6KB)

const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ============================================================
// ESTRUTURAS
// ============================================================
struct DadosSensores {
    float correntePainel;    // A
    float correnteCasa;      // A
    float correnteBateria;   // A (calculada)
    float potenciaPainel;    // W
    float potenciaCasa;      // W
    float potenciaBateria;   // W
    float tensaoBateriaEst;  // V (estimada)
    bool  bateriaCarregando;
    bool  alertaPainelInvertido;
    bool  alertaCasaInvertida;
    bool  sinalBKP;
};

struct ConfigBateria {
    float tensaoNominal;     // V  ex: 12.8
    float amperagem;         // Ah ex: 100
    float percDescargaSegura;// 0.0-1.0 ex: 0.80
    float tensaoMinima;      // V configurada no controlador
    float capacidadeTotal;   // Wh = tensaoNominal * amperagem
    float energiaDisponivel; // Wh = capacidadeTotal * percDescargaSegura
    float tensaoMaxima;      // V ex: 14.0 para 12.8V nominal
};

struct EstadoBateria {
    float ahDisponiveis;     // Ah restantes (coulomb counting)
    float ahMaximos;         // Ah máximos (= amperagem * percDescargaSegura)
    float ciclos;            // ciclos completos
    float ciclosParciais;    // fração do ciclo atual (0.0-1.0)
    bool  carregada100;      // flag bateria cheia detectada
    unsigned long tsCarregada; // timestamp quando ficou cheia
};

struct RegistroHistorico {
    uint32_t timestamp;      // unix time compactado
    int16_t  whPainel;       // Wh gerados (x10 para 1 decimal)
    int16_t  whCasa;         // Wh consumidos (x10)
    int16_t  whCargaBat;     // Wh carregados bateria (x10)
    int16_t  whDescargaBat;  // Wh descarregados bateria (x10)
};

struct Contadores {
    double kwhPainelTotal;    // kWh totais gerados
    double kwhCasaTotal;      // kWh totais consumidos
    double kwhCargaTotal;     // kWh totais carregados bateria
    double kwhDescargaTotal;  // kWh totais descarregados bateria
};

// ============================================================
// ESTRUTURAS - INVERSORES
// ============================================================
struct ConfigInversor {
    float potenciaBaixa;      // W - potência do inversor de baixa
    float potenciaAlta;       // W - potência do inversor de alta
    float percChaveamento;    // % - percentual para chavear (padrão 70%)
    bool  configurado;        // flag de configuração salva

    // Calculados automaticamente
    float correnteBaixa;      // A = potenciaBaixa / tensaoNominal
    float correnteAlta;       // A = potenciaAlta  / tensaoNominal
    float limiteChaveamento;  // A = correnteBaixa * percChaveamento / 100
    float limiteRetorno;      // A = limiteChaveamento - histerese
};

struct EstadoInversor {
    // Estados dos relés
    bool releRedeAlta;        // true = usando inversor alta, false = usando inversor baixa
    bool releInvBaixa;        // true = inversor baixa DESLIGADO, false = LIGADO
    bool releInvAlta;         // true = inversor alta LIGADO,     false = DESLIGADO

    // Status dos inversores (via LED de status)
    bool statusInvBaixa;      // true = OK funcionando
    bool statusInvAlta;       // true = OK funcionando

    // Flags de controle
    bool invBaixaLigado;      // estado lógico do inversor baixa
    bool invAltaLigado;       // estado lógico do inversor alta
    bool modoAlta;            // true = sistema usando inversor alta
    bool emProtecao;          // true = algum inversor em proteção

    // Timers de controle
    unsigned long tsLigouAlta;     // quando ligou o inversor alta
    unsigned long tsProtecao;      // quando entrou em proteção
    unsigned long tsUltimoChav;    // último chaveamento
    unsigned long tsStatusBaixa;   // último status válido baixa
    unsigned long tsStatusAlta;    // último status válido alta

    // Contadores
    uint32_t totalChaveamentos;    // total de chaveamentos realizados
    uint32_t totalProtecoes;       // total de proteções detectadas
    uint32_t totalReiniciamentos;  // total de reiniciamentos de inversor
};

// ============================================================
// ESTRUTURA - CONFIGURAÇÃO REDE ELÉTRICA
// ============================================================
struct ConfigRede {
    // Percentuais de corte (baseados em energiaDisponivel)
    float percInvBaixa;       // % mínimo inv. baixa  (padrão 10%)
    float percRedeAlta;       // % mínimo rede alta   (padrão 30%)
    float percLampExt;        // % mínimo lamp. ext   (padrão 25%)
    float percLampInt;        // % mínimo lamp. int   (padrão  0%)
    bool  configurado;

    // Controle lâmpada externa por horário
    uint8_t modoLampExt;      // 0=sempre auto-bateria
    // 1=horário manual
    // 2=automático por painel
    uint8_t ligarHora;        // hora de ligar  (0-23)
    uint8_t ligarMin;         // minuto de ligar
    uint8_t desligarHora;     // hora de desligar (0-23)
    uint8_t desligarMin;      // minuto de desligar
};

struct EstadoRede {
    // Estados lógicos das cargas
    bool invBaixaAtivo;
    bool redeAltaAtiva;
    bool lampExternaAtiva;
    bool lampInternaAtiva;

    // Estados físicos dos relés
    bool releLampExterna;
    bool releLampInterna;

    // Timers de decisão (1 minuto)
    unsigned long tsAbaixoInvBaixa;
    unsigned long tsAbaixoRedeAlta;
    unsigned long tsAbaixoLampExt;
    unsigned long tsAbaixoLampInt;

    // Flags de aguardo
    bool aguardandoInvBaixa;
    bool aguardandoRedeAlta;
    bool aguardandoLampExt;
    bool aguardandoLampInt;

    // Controle horário lâmpada externa
    bool lampExtPorHorario;   // true = horário está mandando ligar
    bool lampExtPorPainel;    // true = painel detectado = apaga lamp
    bool lampExtBloqEnergia;  // true = bloqueada por energia baixa

    // Contadores de eventos
    uint32_t totalCortesInvBaixa;
    uint32_t totalCortesRedeAlta;
    uint32_t totalCortesLampExt;
    uint32_t totalCortesLampInt;
    uint32_t totalLigadasHorario;  // quantas vezes ligou por horário
    uint32_t totalLigadasPainel;   // quantas vezes desligou por painel
};

// ============================================================
// ESTRUTURAS - PREVISÃO SOLAR
// ============================================================
struct ConfigPrevisao {
    double  latitude;          // double para precisão total
    double  longitude;         // ex: -22.917549777384778
    float   kwpInstalado;      // kWp total instalado
    bool    configurado;       // flag configuração salva
};

struct EstadoPrevisao {
    // ── Arrays de previsão (48h = hoje + amanhã) ──
    float   radiacaoHora[48];      // W/m² por hora
    float   nuvensPct[48];         // % cobertura nuvens
    int8_t  chuvaProb[48];         // % probabilidade chuva
    float   geracaoEstHora[48];    // Wh estimado calibrado por hora

    // ── Calibração por hora do dia (0-23) ──
    float   fatorHorario[24];      // fator aprendido por hora
    uint8_t amostrasHorario[24];   // nº amostras por hora (peso)

    // ── Totais calculados ──
    float   totalRestanteHoje;     // Wh geração restante hoje
    float   totalAmanha;           // Wh total estimado amanhã
    float   consumoMedioDia;       // Wh consumo médio 7 dias
    float   taxaAcerto;            // % precisão média global

    // ── Estado das decisões automáticas ──
    bool    bloqRedeAlta;          // true = rede alta bloqueada por previsão
    bool    bloqLampExt;           // true = lamp externa bloqueada por previsão
    char    motivoRedeAlta[90];    // texto motivo para exibição
    char    motivoLampExt[90];     // texto motivo para exibição
    bool    redeAltaFoiDesligada;  // flag para religar ao amanhecer
    bool    lampExtFoiDesligada;   // flag para religar ao amanhecer

    // ── Controle de tempo ──
    unsigned long tsUltimaAtualizacao;  // millis() última busca API
    unsigned long tsUltimoSalvamento;   // millis() último save flash
    float   fatorAntesSalvar;           // fator no último save (detecta mudança)
    bool    dadosValidos;               // true = tem dados da API
    int8_t  horaOffsetAPI;              // índice [0] da API = qual hora do dia
    uint8_t errosConsecutivos;          // contador falhas HTTP

    // ── Calibração da hora atual ──
    float   geracaoRealAcumHora;   // Wh reais acumulados na hora atual
    float   geracaoEstAcumHora;    // Wh estimados acumulados na hora atual
    uint8_t minutosNaHora;         // minutos acumulados para calibração
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
uint8_t contLogs   = 0;   // contador para RAM log
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

// Acumuladores para histórico (resetam a cada intervalo)
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
// VARIÁVEIS GLOBAIS - INVERSORES
// ============================================================
ConfigInversor cfgInv   = {0};
EstadoInversor estadoInv = {0};

// Máquina de estados do controle
enum EstadoMaquina {
    ESTADO_BAIXA_NORMAL,      // operando normal com inversor baixa
    ESTADO_TRANSFERINDO_ALTA, // transferindo para inversor alta
    ESTADO_ALTA_NORMAL,       // operando normal com inversor alta
    ESTADO_TRANSFERINDO_BAIXA,// transferindo de volta para inversor baixa
    ESTADO_PROTECAO_BAIXA,    // inversor baixa em proteção
    ESTADO_REINICIANDO_INV,   // reiniciando inversor após proteção
    ESTADO_ERRO               // erro crítico
};

EstadoMaquina maquinaEstado = ESTADO_BAIXA_NORMAL;
char nomeEstado[30] = "Baixa Normal";

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
unsigned long  tPrevisao  = 0;    // timer busca API

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
        adicionarLog("NTP: sem sincronismo (sem internet)");
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
    prefs.putFloat("bat_vnom",   cfgBat.tensaoNominal);
    prefs.putFloat("bat_ah",     cfgBat.amperagem);
    prefs.putFloat("bat_pdesc",  cfgBat.percDescargaSegura);
    prefs.putFloat("bat_vmin",   cfgBat.tensaoMinima);
    prefs.putBool ("bat_ok",     true);
    prefs.end();
}

void carregarConfigBateria() {
    prefs.begin("rrfs", true);
    cfgBat.tensaoNominal      = prefs.getFloat("bat_vnom",  12.8f);
    cfgBat.amperagem          = prefs.getFloat("bat_ah",    100.0f);
    cfgBat.percDescargaSegura = prefs.getFloat("bat_pdesc", 0.80f);
    cfgBat.tensaoMinima       = prefs.getFloat("bat_vmin",  12.0f);
    prefs.end();
    // Recalcula derivados
    cfgBat.capacidadeTotal  = cfgBat.tensaoNominal * cfgBat.amperagem;
    cfgBat.energiaDisponivel= cfgBat.capacidadeTotal * cfgBat.percDescargaSegura;
    // Tensão máxima LiFePO4
    if (cfgBat.tensaoNominal <= 13.0f)       cfgBat.tensaoMaxima = 14.0f;
    else if (cfgBat.tensaoNominal <= 26.0f)  cfgBat.tensaoMaxima = 28.0f;
    else                                      cfgBat.tensaoMaxima = cfgBat.tensaoNominal * 1.094f;
}

void salvarEstadoBateria() {
    prefs.begin("rrfsb", false);
    prefs.putFloat("ah_disp",   estadoBat.ahDisponiveis);
    prefs.putFloat("ah_max",    estadoBat.ahMaximos);
    prefs.putFloat("ciclos",    estadoBat.ciclos);
    prefs.putFloat("ciclos_p",  estadoBat.ciclosParciais);
    prefs.putBool ("cheia",     estadoBat.carregada100);
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
    // Se nunca inicializado, assume bateria cheia
    if (estadoBat.ahDisponiveis < 0) {
        estadoBat.ahDisponiveis = cfgBat.amperagem * cfgBat.percDescargaSegura;
        estadoBat.carregada100  = true;
    }
    estadoBat.ahMaximos = cfgBat.amperagem * cfgBat.percDescargaSegura;
}

void salvarContadores() {
    prefs.begin("rrfsc", false);
    prefs.putDouble("kwh_pai",  contadores.kwhPainelTotal);
    prefs.putDouble("kwh_cas",  contadores.kwhCasaTotal);
    prefs.putDouble("kwh_car",  contadores.kwhCargaTotal);
    prefs.putDouble("kwh_des",  contadores.kwhDescargaTotal);
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
// PREFERENCES - INVERSORES
// ============================================================
void salvarConfigInversor() {
    prefs.begin("rrfsi", false);
    prefs.putFloat("inv_pbaixa",  cfgInv.potenciaBaixa);
    prefs.putFloat("inv_palta",   cfgInv.potenciaAlta);
    prefs.putFloat("inv_pchav",   cfgInv.percChaveamento);
    prefs.putBool ("inv_ok",      true);
    prefs.end();
}

void carregarConfigInversor() {
    prefs.begin("rrfsi", true);
    cfgInv.potenciaBaixa   = prefs.getFloat("inv_pbaixa", 500.0f);
    cfgInv.potenciaAlta    = prefs.getFloat("inv_palta",  3000.0f);
    cfgInv.percChaveamento = prefs.getFloat("inv_pchav",  70.0f);
    cfgInv.configurado     = prefs.getBool ("inv_ok",     false);
    prefs.end();
    recalcularInversor();
}

bool existeConfigInversor() {
    prefs.begin("rrfsi", true);
    bool ok = prefs.isKey("inv_ok");
    prefs.end();
    return ok;
}

void salvarEstadoInversor() {
    prefs.begin("rrfsis", false);
    prefs.putUInt ("total_chav",  estadoInv.totalChaveamentos);
    prefs.putUInt ("total_prot",  estadoInv.totalProtecoes);
    prefs.putUInt ("total_rein",  estadoInv.totalReiniciamentos);
    prefs.end();
}

void carregarEstadoInversor() {
    prefs.begin("rrfsis", true);
    estadoInv.totalChaveamentos  = prefs.getUInt("total_chav", 0);
    estadoInv.totalProtecoes     = prefs.getUInt("total_prot", 0);
    estadoInv.totalReiniciamentos= prefs.getUInt("total_rein", 0);
    prefs.end();
}

void recalcularInversor() {
    if (cfgBat.tensaoNominal > 0) {
        cfgInv.correnteBaixa     = cfgInv.potenciaBaixa / cfgBat.tensaoNominal;
        cfgInv.correnteAlta      = cfgInv.potenciaAlta  / cfgBat.tensaoNominal;
        cfgInv.limiteChaveamento = cfgInv.correnteBaixa * cfgInv.percChaveamento / 100.0f;
        cfgInv.limiteRetorno     = cfgInv.limiteChaveamento *
        (1.0f - HISTERESE_PERC / 100.0f);
    }
}

// ============================================================
// PREFERENCES - REDE ELÉTRICA
// ============================================================
void salvarConfigRede() {
    prefs.begin("rrfsr", false);
    prefs.putFloat ("r_invbaixa",  cfgRede.percInvBaixa);
    prefs.putFloat ("r_redealta",  cfgRede.percRedeAlta);
    prefs.putFloat ("r_lampext",   cfgRede.percLampExt);
    prefs.putFloat ("r_lampint",   cfgRede.percLampInt);
    prefs.putUChar ("r_modolext",  cfgRede.modoLampExt);
    prefs.putUChar ("r_lighora",   cfgRede.ligarHora);
    prefs.putUChar ("r_ligmin",    cfgRede.ligarMin);
    prefs.putUChar ("r_dslhora",   cfgRede.desligarHora);
    prefs.putUChar ("r_dslmin",    cfgRede.desligarMin);
    prefs.putBool  ("r_ok",        true);
    prefs.end();
}

void carregarConfigRede() {
    prefs.begin("rrfsr", true);
    cfgRede.percInvBaixa  = prefs.getFloat ("r_invbaixa", 10.0f);
    cfgRede.percRedeAlta  = prefs.getFloat ("r_redealta", 30.0f);
    cfgRede.percLampExt   = prefs.getFloat ("r_lampext",  25.0f);
    cfgRede.percLampInt   = prefs.getFloat ("r_lampint",   0.0f);
    cfgRede.modoLampExt   = prefs.getUChar ("r_modolext",    0);
    cfgRede.ligarHora     = prefs.getUChar ("r_lighora",    18);
    cfgRede.ligarMin      = prefs.getUChar ("r_ligmin",      0);
    cfgRede.desligarHora  = prefs.getUChar ("r_dslhora",     6);
    cfgRede.desligarMin   = prefs.getUChar ("r_dslmin",      0);
    cfgRede.configurado   = prefs.getBool  ("r_ok",      false);
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
    prefs.putUInt("r_cinvb",  estadoRede.totalCortesInvBaixa);
    prefs.putUInt("r_creda",  estadoRede.totalCortesRedeAlta);
    prefs.putUInt("r_clext",  estadoRede.totalCortesLampExt);
    prefs.putUInt("r_clint",  estadoRede.totalCortesLampInt);
    prefs.putUInt("r_clighr", estadoRede.totalLigadasHorario);
    prefs.putUInt("r_cpain",  estadoRede.totalLigadasPainel);
    prefs.end();
}

void carregarEstadoRede() {
    prefs.begin("rrfsrs", true);
    estadoRede.totalCortesInvBaixa  = prefs.getUInt("r_cinvb",  0);
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
    // Double salvo como String para preservar precisão total
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

// Salva calibração horária (24 fatores + 24 amostras)
// Só salva se fator mudou mais de 2% OU passaram 60 min
void salvarCalibracaoHoraria(bool forcado) {
    unsigned long agora = millis();
    float fatorAtual = estadoPrev.fatorHorario[0]; // proxy

    bool mudouSignificativo =
    fabsf(fatorAtual - estadoPrev.fatorAntesSalvar) > DELTA_FATOR_SALVAR;
    bool tempoExpirou =
    (agora - estadoPrev.tsUltimoSalvamento) >= INTERVALO_PREVISAO;

    if (!forcado && !mudouSignificativo && !tempoExpirou) return;

    prefs.begin("rrfspc", false);
    // Salva os 24 fatores e 24 contadores de amostras
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
        // Padrão 1.0 = sem calibração ainda
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
// CONSUMO MÉDIO DIÁRIO (reutiliza histórico existente)
// Média ponderada dos últimos 7 dias
// ============================================================
void calcularConsumoMedio() {
    if (histTotal < 60) return; // precisa de pelo menos 1 hora

    // Acumula Wh por dia (registros de 60s = 1440 por dia)
    float whDia[7]   = {0};
    uint16_t ctDia[7] = {0};

    for (int i = 0; i < histTotal && i < MAX_HISTORICO; i++) {
        int idx = ((int)histIdx - 1 - i + MAX_HISTORICO) % MAX_HISTORICO;
        uint32_t ts = historico[idx].timestamp;
        if (ts == 0) continue;

        // Qual dia atrás é este registro?
        time_t agora = time(nullptr);
        int diasAtras = (int)((agora - ts) / 86400);
        if (diasAtras < 0 || diasAtras >= 7) continue;

        whDia[diasAtras]  += historico[idx].whCasa / 10.0f;
        ctDia[diasAtras]++;
    }

    // Média ponderada: dia mais recente (peso 7) → mais antigo (peso 1)
    float somaWh   = 0;
    float somaPeso = 0;
    for (int d = 0; d < 7; d++) {
        if (ctDia[d] < 10) continue; // dia incompleto, ignora
        float pesoD = (float)(7 - d); // dia 0=hoje(7), dia 6=antigo(1)
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
// Extrai array de floats de um campo JSON
// Reutiliza tempBuf para economizar stack
// ============================================================
int parseArrayFloat(const String& json,
                    const char* campo,
                    float* dest,
                    int maxValores) {
    // Acha o campo: "shortwave_radiation":[...]
    String busca = "\"";
    busca += campo;
    busca += "\":[";
    int pos = json.indexOf(busca);
    if (pos < 0) return 0;
    pos += busca.length();

    int count = 0;
    while (count < maxValores) {
        // Acha próximo número
        int fim = json.indexOf(',', pos);
        int fimArr = json.indexOf(']', pos);
        if (fimArr >= 0 && (fim < 0 || fimArr < fim)) fim = fimArr;
        if (fim < 0) break;

        String token = json.substring(pos, fim);
        token.trim();
        dest[count++] = token.toFloat();

        if (fim == fimArr) break; // fim do array
        pos = fim + 1;
    }
    return count;
}

// ============================================================
// BUSCA DADOS OPEN-METEO (chamada HTTP)
// Retorna true se sucesso
// ============================================================
bool buscarDadosOpenMeteo() {
    if (modoAP || WiFi.status() != WL_CONNECTED) return false;
    if (!cfgPrev.configurado) return false;

    // Monta URL com coordenadas double (precisão total)
    char url[320];
    snprintf(url, sizeof(url),
        "http://api.open-meteo.com/v1/forecast?"
        "latitude=%.15f&longitude=%.15f"
        "&hourly=shortwave_radiation,cloudcover,"
        "precipitation_probability"
        "&forecast_days=2"
        "&timezone=auto",
        cfgPrev.latitude, cfgPrev.longitude);

    snprintf(tempBuf, sizeof(tempBuf), "Prev: buscando API...");
    adicionarLog(tempBuf);

    WiFiClient  client;
    HTTPClient  http;
    http.begin(client, url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.addHeader("Accept", "application/json");

    int httpCode = http.GET();
    if (httpCode != 200) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Prev: erro HTTP %d", httpCode);
        adicionarLog(tempBuf);
        http.end();
        estadoPrev.errosConsecutivos++;
        return false;
    }

    // Buffer alocado na heap para não explodir o stack
    String payload = http.getString();
    http.end();

    if (payload.length() < 100) {
        adicionarLog("Prev: resposta invalida");
        estadoPrev.errosConsecutivos++;
        return false;
    }

    // ── Descobre o offset de hora atual na resposta ──
    // A API retorna horas de 00:00 do dia atual
    // Precisamos saber qual índice = hora atual
    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    // Índice 0 = 00:00 hoje, índice ti->tm_hour = hora atual
    estadoPrev.horaOffsetAPI = ti->tm_hour;

    // ── Parse dos 3 arrays (48 valores cada) ──
    float radTemp[48]   = {0};
    float nuvensTemp[48]= {0};
    float chuvaTemp[48] = {0};

    int nRad   = parseArrayFloat(payload, "shortwave_radiation",
                                  radTemp,   48);
    int nNuv   = parseArrayFloat(payload, "cloudcover",
                                  nuvensTemp, 48);
    parseArrayFloat(payload, "precipitation_probability",
                    chuvaTemp, 48);

    if (nRad < 24) {
        adicionarLog("Prev: parse falhou");
        estadoPrev.errosConsecutivos++;
        return false;
    }

    // ── Copia para estrutura global ──
    for (int i = 0; i < 48; i++) {
        estadoPrev.radiacaoHora[i]  = radTemp[i];
        estadoPrev.nuvensPct[i]     = nuvensTemp[i];
        estadoPrev.chuvaProb[i]     = (int8_t)chuvaTemp[i];
    }

    estadoPrev.dadosValidos          = true;
    estadoPrev.errosConsecutivos     = 0;
    estadoPrev.tsUltimaAtualizacao   = millis();

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev: API OK! Rad agora=%.0f W/m2 Nuv=%.0f%%",
             estadoPrev.radiacaoHora[estadoPrev.horaOffsetAPI],
             estadoPrev.nuvensPct[estadoPrev.horaOffsetAPI]);
    adicionarLog(tempBuf);
    return true;
}

// ============================================================
// CALCULA GERAÇÃO ESTIMADA POR HORA (com calibração)
// Reutiliza: cfgPrev.kwpInstalado, estadoPrev.fatorHorario[]
//            estadoPrev.radiacaoHora[]
// ============================================================
void calcularGeracaoEstimada() {
    if (!estadoPrev.dadosValidos) return;
    if (cfgPrev.kwpInstalado <= 0) return;

    int horaAtual = estadoPrev.horaOffsetAPI;
    float totalRest = 0;
    float totalAmanha = 0;

    for (int i = 0; i < 48; i++) {
        float rad    = estadoPrev.radiacaoHora[i];
        int   hDia   = (horaAtual + i) % 24; // hora do dia (0-23)

        // Geração bruta: kWp × radiação × eficiência
        // rad em W/m², kwp em kW → resultado em Wh por hora
        float bruto = rad * cfgPrev.kwpInstalado * EFICIENCIA_PADRAO;

        // Aplica fator de calibração aprendido para esta hora do dia
        float fator = estadoPrev.fatorHorario[hDia];
        float calibrado = bruto * fator;

        // Limita: não pode gerar mais que capacidade instalada × 1.1
        float maxPossivel = cfgPrev.kwpInstalado * 1000.0f * 1.1f;
        calibrado = constrain(calibrado, 0.0f, maxPossivel);

        estadoPrev.geracaoEstHora[i] = calibrado;

        // Acumula totais
        if (i >= 0 && i < (24 - horaAtual)) {
            // Ainda é hoje (horas restantes)
            totalRest += calibrado;
        } else if (i >= (24 - horaAtual) &&
                   i <  (24 - horaAtual + 24)) {
            // Amanhã (próximas 24h após hoje)
            totalAmanha += calibrado;
        }
    }

    // Reutiliza variáveis globais (zero recálculo externo)
    estadoPrev.totalRestanteHoje = totalRest;
    estadoPrev.totalAmanha       = totalAmanha;
}

// ============================================================
// CALIBRAÇÃO DO FATOR HORÁRIO
// Chamado dentro do loop a cada segundo
// Reutiliza: sensores.correntePainel (já calculado em lerSensores)
//            sensores.tensaoBateriaEst (já calculado)
//            estadoPrev.radiacaoHora[] (já carregado da API)
// ============================================================
void calibrarFatorHorario() {
    if (!estadoPrev.dadosValidos)  return;
    if (!ntpSincronizado)          return;
    if (cfgPrev.kwpInstalado <= 0) return;

    // Reutiliza valores já calculados em lerSensores()
    float corrPainel = sensores.correntePainel;
    float tensao     = sensores.tensaoBateriaEst;

    // Só calibra se painel está gerando significativamente
    if (corrPainel < MIN_CORRENTE_CALIBRAR) {
        // Painel parado: reseta acumuladores da hora
        estadoPrev.geracaoRealAcumHora = 0;
        estadoPrev.geracaoEstAcumHora  = 0;
        estadoPrev.minutosNaHora       = 0;
        return;
    }

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int horaAtual = ti->tm_hour;
    int idxAPI    = horaAtual - estadoPrev.horaOffsetAPI;
    if (idxAPI < 0) idxAPI += 24;
    if (idxAPI >= 48) return;

    float radAtual = estadoPrev.radiacaoHora[idxAPI];
    if (radAtual < MIN_RADIACAO_CALIBRAR) return;

    // Acumula geração real e estimada desta hora (em Wh por minuto)
    float whRealMinuto = (corrPainel * tensao) / 60.0f;
    float whEstMinuto  = estadoPrev.geracaoEstHora[idxAPI] / 60.0f;

    estadoPrev.geracaoRealAcumHora += whRealMinuto;
    estadoPrev.geracaoEstAcumHora  += whEstMinuto;
    estadoPrev.minutosNaHora++;

    // Calibra apenas quando acumulou pelo menos 10 minutos de dados
    // para ter amostra estatisticamente válida
    if (estadoPrev.minutosNaHora < 10) return;
    if (estadoPrev.geracaoEstAcumHora < 1.0f) return;

    // Calcula fator desta hora
    float fatorMedido = estadoPrev.geracaoRealAcumHora /
                        estadoPrev.geracaoEstAcumHora;
    fatorMedido = constrain(fatorMedido, FATOR_CAL_MIN, FATOR_CAL_MAX);

    // Média móvel ponderada pelo número de amostras acumuladas
    // Quanto mais amostras, mais confiança no histórico
    uint8_t nAmostras = estadoPrev.amostrasHorario[horaAtual];
    float pesoHist = (nAmostras > 10) ? 0.85f : PESO_HISTORICO;
    float pesoNovo = 1.0f - pesoHist;

    float fatorAntigo = estadoPrev.fatorHorario[horaAtual];
    float fatorNovo   = (fatorAntigo * pesoHist) + (fatorMedido * pesoNovo);
    fatorNovo = constrain(fatorNovo, FATOR_CAL_MIN, FATOR_CAL_MAX);

    estadoPrev.fatorHorario[horaAtual] = fatorNovo;

    // Incrementa amostras (limita em 255)
    if (nAmostras < 255)
        estadoPrev.amostrasHorario[horaAtual]++;

    // Calcula taxa de acerto desta calibração
    float erroPerc = fabsf((fatorMedido - 1.0f) * 100.0f);
    float acertoEsta = constrain(100.0f - erroPerc, 0.0f, 100.0f);

    // Atualiza taxa de acerto global (média móvel)
    if (estadoPrev.taxaAcerto < 1.0f)
        estadoPrev.taxaAcerto = acertoEsta;
    else
        estadoPrev.taxaAcerto = estadoPrev.taxaAcerto * 0.9f
                               + acertoEsta * 0.1f;

    // Recalcula estimativas com novo fator
    calcularGeracaoEstimada();

    // Salva se mudou significativamente (economiza flash)
    salvarCalibracaoHoraria(false);

    // Reseta acumuladores para próximo período de 10 min
    estadoPrev.geracaoRealAcumHora = 0;
    estadoPrev.geracaoEstAcumHora  = 0;
    estadoPrev.minutosNaHora       = 0;
}

// ============================================================
// TOMAR DECISÕES AUTOMÁTICAS COM BASE NA PREVISÃO
// Reutiliza: estadoPrev.totalAmanha (já calculado)
//            estadoBat.ahDisponiveis (já calculado)
//            sensores.tensaoBateriaEst (já calculado)
//            cfgBat.energiaDisponivel (pré-calculado)
//            cfgRede.percRedeAlta (configurado pelo usuário)
// ============================================================
void tomarDecisoesPrev() {
    if (!estadoPrev.dadosValidos)  return;
    if (!existeConfigRede())        return;
    if (!existeBateriaConfig())     return;
    if (!ntpSincronizado)           return;

    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int hora = ti->tm_hour;

    // Reutiliza SoC já calculado pela lógica existente
    float soc = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    // Reutiliza consumoMedioDia (calculado 1x por dia)
    float consMedia = estadoPrev.consumoMedioDia;
    if (consMedia < 100.0f) consMedia = 3000.0f; // fallback seguro

    float geracaoAmanha = estadoPrev.totalAmanha;

    // Reutiliza whDisp e whMinRA (mesma equação de controlarRede)
    float whDisp  = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float whMinRA = cfgBat.energiaDisponivel *
    (cfgRede.percRedeAlta / 100.0f);
    float whMinRAOn = cfgBat.energiaDisponivel *
    ((cfgRede.percRedeAlta + HISTERESE_REDE_PERC) / 100.0f);

    // ────────────────────────────────────────────────────────
    // DECISÃO 1: LÂMPADA EXTERNA NOTURNA
    // Condição: noite + previsão ruim + bateria baixa
    // ────────────────────────────────────────────────────────
    bool noite = (hora >= 22 || hora < HORA_AMANHECER);

    if (noite && !estadoPrev.bloqLampExt) {
        bool prevRuim = (geracaoAmanha < consMedia * 0.60f);
        bool batBaixa = (soc < 35.0f);

        if (prevRuim && batBaixa) {
            estadoPrev.bloqLampExt         = true;
            estadoPrev.lampExtFoiDesligada = true;
            snprintf(estadoPrev.motivoLampExt,
                     sizeof(estadoPrev.motivoLampExt),
                     "Prev.amanha=%.0fWh<%.0fWh Bat=%.0f%%",
                     geracaoAmanha, consMedia * 0.60f, soc);
            adicionarLog("PREV: Lamp.Ext bloqueada por previsao ruim");
            adicionarLog(estadoPrev.motivoLampExt);
        }
    }

    // Religa lâmpada ao amanhecer (remove bloqueio de previsão)
    if (hora >= HORA_AMANHECER && hora < 10 &&
        estadoPrev.lampExtFoiDesligada) {
        estadoPrev.bloqLampExt         = false;
    estadoPrev.lampExtFoiDesligada = false;
    estadoPrev.motivoLampExt[0]    = 0;
    adicionarLog("PREV: Lamp.Ext desbloqueada (amanheceu)");
        }

        // ────────────────────────────────────────────────────────
        // DECISÃO 2: REDE ALTA APÓS MEIA-NOITE
        // Geladeira usa menos à noite → economia
        // ────────────────────────────────────────────────────────
        bool aposMeiaNoite = (hora >= HORA_MEIA_NOITE && hora < HORA_AMANHECER);

        if (aposMeiaNoite && !estadoPrev.bloqRedeAlta) {
            bool prevMuitoRuim = (geracaoAmanha < consMedia * 0.50f);
            bool batMediaBaixa = (soc < 30.0f);

            if (prevMuitoRuim || batMediaBaixa) {
                estadoPrev.bloqRedeAlta         = true;
                estadoPrev.redeAltaFoiDesligada = true;
                snprintf(estadoPrev.motivoRedeAlta,
                         sizeof(estadoPrev.motivoRedeAlta),
                         "Noite: Prev=%.0fWh Bat=%.0f%% Cons=%.0fWh",
                         geracaoAmanha, soc, consMedia);
                adicionarLog("PREV: Rede Alta bloqueada apos meia-noite");
                adicionarLog(estadoPrev.motivoRedeAlta);
            }
        }

        // ────────────────────────────────────────────────────────
        // DECISÃO 3: REDE ALTA ANTES DA MEIA-NOITE (situação crítica)
        // ────────────────────────────────────────────────────────
        bool tardinha = (hora >= HORA_ANTECIPAR_CORTE && hora < 24);

        if (tardinha && !estadoPrev.bloqRedeAlta) {
            bool prevCritico = (geracaoAmanha < consMedia * 0.35f);
            bool batCritica  = (soc < 25.0f);

            if (prevCritico && batCritica) {
                estadoPrev.bloqRedeAlta         = true;
                estadoPrev.redeAltaFoiDesligada = true;
                snprintf(estadoPrev.motivoRedeAlta,
                         sizeof(estadoPrev.motivoRedeAlta),
                         "CRITICO: Prev=%.0fWh<%.0fWh Bat=%.0f%%",
                         geracaoAmanha, consMedia * 0.35f, soc);
                adicionarLog("PREV: Rede Alta bloqueada antecipado (critico)");
                adicionarLog(estadoPrev.motivoRedeAlta);
            }
        }

        // ────────────────────────────────────────────────────────
        // DECISÃO 4: RELIGAR REDE ALTA AO AMANHECER
        // Reutiliza percRedeAlta (já configurado em cfgRede)
        // Reutiliza whDisp e whMinRA calculados acima
        // ────────────────────────────────────────────────────────
        bool amanheceu = (hora >= HORA_AMANHECER && hora < 12);

        if (amanheceu && estadoPrev.redeAltaFoiDesligada &&
            whDisp > whMinRAOn) {
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

    // Status geral
    snprintf(tempBuf, sizeof(tempBuf),
             "\"ok\":%d,\"kwp\":%.2f,\"acerto\":%.1f,"
             "\"fatorGlobal\":%.3f,\"consMedia\":%.0f,",
             estadoPrev.dadosValidos ? 1 : 0,
             cfgPrev.kwpInstalado,
             estadoPrev.taxaAcerto,
             estadoPrev.fatorHorario[horaAtual],
             estadoPrev.consumoMedioDia);
    j += tempBuf;

    // Hora atual
    snprintf(tempBuf, sizeof(tempBuf),
             "\"radAgora\":%.0f,\"nuvAgora\":%.0f,"
             "\"chuvaAgora\":%d,\"gerEstAgora\":%.0f,",
             estadoPrev.radiacaoHora[idxAtual],
             estadoPrev.nuvensPct[idxAtual],
             (int)estadoPrev.chuvaProb[idxAtual],
             estadoPrev.geracaoEstHora[idxAtual]);
    j += tempBuf;

    // Geração real atual (reutiliza sensores já calculados)
    float gerRealAgora = sensores.potenciaPainel; // W instantâneo
    snprintf(tempBuf, sizeof(tempBuf),
             "\"gerRealAgora\":%.0f,\"restHoje\":%.0f,"
             "\"totalAmanha\":%.0f,",
             gerRealAgora,
             estadoPrev.totalRestanteHoje,
             estadoPrev.totalAmanha);
    j += tempBuf;

    // Decisões
    snprintf(tempBuf, sizeof(tempBuf),
             "\"bloqRA\":%d,\"bloqLE\":%d,"
             "\"mRA\":\"%s\",\"mLE\":\"%s\",",
             estadoPrev.bloqRedeAlta ? 1 : 0,
             estadoPrev.bloqLampExt  ? 1 : 0,
             estadoPrev.motivoRedeAlta,
             estadoPrev.motivoLampExt);
    j += tempBuf;

    // Arrays próximas 12 horas (compacto)
    j += "\"horas\":[";
    for (int i = 0; i < 12; i++) {
        int idx = idxAtual + i;
        if (idx >= 48) break;
        int hDia = (horaAtual + i) % 24;
        if (i > 0) j += ",";
        snprintf(tempBuf, sizeof(tempBuf),
                 "{\"h\":%d,\"r\":%.0f,\"n\":%.0f,"
                 "\"g\":%.0f,\"c\":%d}",
                 hDia,
                 estadoPrev.radiacaoHora[idx],
                 estadoPrev.nuvensPct[idx],
                 estadoPrev.geracaoEstHora[idx],
                 (int)estadoPrev.chuvaProb[idx]);
        j += tempBuf;
    }
    j += "],";

    // Arrays amanhã (24h após hoje)
    int inicioAmanha = 24 - horaAtual;
    j += "\"amanha\":[";
    for (int i = 0; i < 24; i++) {
        int idx = inicioAmanha + i;
        if (idx >= 48) break;
        if (i > 0) j += ",";
        snprintf(tempBuf, sizeof(tempBuf),
                 "{\"h\":%d,\"r\":%.0f,\"n\":%.0f,"
                 "\"g\":%.0f,\"c\":%d}",
                 i,
                 estadoPrev.radiacaoHora[idx],
                 estadoPrev.nuvensPct[idx],
                 estadoPrev.geracaoEstHora[idx],
                 (int)estadoPrev.chuvaProb[idx]);
        j += tempBuf;
    }
    j += "],";

    // Fatores de calibração por hora
    j += "\"fatores\":[";
    for (int h = 0; h < 24; h++) {
        if (h > 0) j += ",";
        snprintf(tempBuf, sizeof(tempBuf),
                 "%.3f", estadoPrev.fatorHorario[h]);
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
// SETUP REDE ELÉTRICA
// ============================================================
void setupRede() {
    pinMode(PIN_RELE_LAMP_EXTERNA, OUTPUT);
    pinMode(PIN_RELE_LAMP_INTERNA, OUTPUT);

    // Estado inicial seguro: tudo LIGADO (relés LOW = lâmpadas ON)
    setLampadaExterna(true);
    setLampadaInterna(true);

    // Estado inicial das flags
    estadoRede.invBaixaAtivo   = true;
    estadoRede.redeAltaAtiva   = true;
    estadoRede.lampExternaAtiva= true;
    estadoRede.lampInternaAtiva= true;

    // Zera timers
    estadoRede.tsAbaixoInvBaixa = 0;
    estadoRede.tsAbaixoRedeAlta = 0;
    estadoRede.tsAbaixoLampExt  = 0;
    estadoRede.tsAbaixoLampInt  = 0;

    // Zera flags de aguardo
    estadoRede.aguardandoInvBaixa = false;
    estadoRede.aguardandoRedeAlta = false;
    estadoRede.aguardandoLampExt  = false;
    estadoRede.aguardandoLampInt  = false;

    adicionarLog("Rede eletrica: inicializada");
}

// ── Relé Lâmpada Externa ──
// ligada=true  → relé LOW  → lâmpada LIGADA
// ligada=false → relé HIGH → lâmpada DESLIGADA
void setLampadaExterna(bool ligada) {
    estadoRede.lampExternaAtiva = ligada;
    estadoRede.releLampExterna  = !ligada;
    digitalWrite(PIN_RELE_LAMP_EXTERNA, ligada ? LOW : HIGH);
}

// ── Relé Lâmpada Interna ──
void setLampadaInterna(bool ligada) {
    estadoRede.lampInternaAtiva = ligada;
    estadoRede.releLampInterna  = !ligada;
    digitalWrite(PIN_RELE_LAMP_INTERNA, ligada ? LOW : HIGH);
}

// ============================================================
// CONTROLE DA LÂMPADA EXTERNA POR HORÁRIO E PAINEL
// Chamado dentro de controlarRede()
// Reutiliza: sensores.correntePainel (já calculado)
//            ntpSincronizado (flag global)
// ============================================================
void controlarLampadaExterna() {
    // Se energia está bloqueando → não interfere
    // A lógica de energia em controlarRede() tem prioridade
    if (estadoRede.lampExtBloqEnergia) return;

    // Modo 0: apenas controle por energia (sem horário)
    if (cfgRede.modoLampExt == 0) {
        // Lâmpada ligada por padrão, só energia controla
        if (!estadoRede.lampExtBloqEnergia &&
            !estadoRede.lampExternaAtiva) {
            setLampadaExterna(true);
            }
            estadoRede.lampExtPorHorario = false;
        estadoRede.lampExtPorPainel  = false;
        return;
    }

    // ── Modo 1: Horário Manual ──
    if (cfgRede.modoLampExt == 1) {
        if (!ntpSincronizado) return; // sem NTP não pode controlar

        time_t    now = time(nullptr);
        struct tm* ti = localtime(&now);
        int horaAtual = ti->tm_hour;
        int minAtual  = ti->tm_min;

        // Converte horários para minutos do dia
        int minAgora  = horaAtual * 60 + minAtual;
        int minLigar  = cfgRede.ligarHora  * 60 + cfgRede.ligarMin;
        int minDesl   = cfgRede.desligarHora * 60 + cfgRede.desligarMin;

        bool deveEstarLigada = false;

        if (minLigar < minDesl) {
            // Caso normal: liga às 18h desliga às 06h (cruza meia-noite)
            // Liga: das 18:00 até 23:59 OU das 00:00 até 06:00
            deveEstarLigada = (minAgora >= minLigar || minAgora < minDesl);
        } else {
            // Caso invertido: liga às 06h desliga às 18h (período diurno)
            deveEstarLigada = (minAgora >= minLigar && minAgora < minDesl);
        }

        // Aplica estado
        if (deveEstarLigada != estadoRede.lampExternaAtiva) {
            if (deveEstarLigada) {
                setLampadaExterna(true);
                estadoRede.lampExtPorHorario = true;
                estadoRede.totalLigadasHorario++;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Lamp.Ext LIGADA por horario (%02d:%02d)",
                         horaAtual, minAtual);
                adicionarLog(tempBuf);
                salvarEstadoRede();
            } else {
                setLampadaExterna(false);
                estadoRede.lampExtPorHorario = false;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Lamp.Ext DESLIGADA por horario (%02d:%02d)",
                         horaAtual, minAtual);
                adicionarLog(tempBuf);
            }
        }
        estadoRede.lampExtPorPainel = false;
        return;
    }

    // ── Modo 2: Automático por Painel Solar ──
    if (cfgRede.modoLampExt == 2) {
        // Reutiliza correntePainel já calculado por lerSensores()
        // Painel gerando > 0.5A → é dia → apaga lâmpada externa
        // Painel parado  = 0.0A → é noite → liga lâmpada externa
        bool painelAtivo = (sensores.correntePainel > 0.5f);

        if (painelAtivo && estadoRede.lampExternaAtiva) {
            // Painel ligou → amanheceu → apaga lâmpada
            setLampadaExterna(false);
            estadoRede.lampExtPorPainel = true;
            estadoRede.totalLigadasPainel++;
            adicionarLog("Lamp.Ext DESLIGADA: painel gerando (dia)");
            salvarEstadoRede();

        } else if (!painelAtivo && !estadoRede.lampExternaAtiva
            && estadoRede.lampExtPorPainel) {
            // Painel parou → anoiteceu → liga lâmpada
            // Só religa se foi o painel que desligou
            setLampadaExterna(true);
        estadoRede.lampExtPorPainel = true;
        adicionarLog("Lamp.Ext LIGADA: painel parou (noite)");
            }

            estadoRede.lampExtPorHorario = false;
            return;
    }
}

// ============================================================
// CONTROLE DA REDE ELÉTRICA
// Integração com previsão solar via flags de bloqueio
// Reutiliza: estadoBat.ahDisponiveis (calculado em atualizarEstadoBateria)
//            sensores.tensaoBateriaEst (calculado em lerSensores)
//            cfgBat.energiaDisponivel (pré-calculado)
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

    // whDisp reutiliza tensaoBateriaEst já calculada em lerSensores()
    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;

    // Limites reutilizam energiaDisponivel pré-calculado
    float eD        = cfgBat.energiaDisponivel;
    float whMinIB   = eD * (cfgRede.percInvBaixa / 100.0f);
    float whMinRA   = eD * (cfgRede.percRedeAlta  / 100.0f);
    float whMinLE   = eD * (cfgRede.percLampExt   / 100.0f);
    float whMinLI   = eD * (cfgRede.percLampInt   / 100.0f);

    // Limites de religamento (+ histerese)
    float whMinIBOn = eD * ((cfgRede.percInvBaixa + HISTERESE_REDE_PERC) / 100.0f);
    float whMinRAOn = eD * ((cfgRede.percRedeAlta  + HISTERESE_REDE_PERC) / 100.0f);
    float whMinLEOn = eD * ((cfgRede.percLampExt   + HISTERESE_REDE_PERC) / 100.0f);
    float whMinLIOn = eD * ((cfgRede.percLampInt   + HISTERESE_REDE_PERC) / 100.0f);

    // ════════════════════════════════════════════════════════
    // CONTROLE 4: LÂMPADA INTERNA (0% padrão)
    // ════════════════════════════════════════════════════════
    if (estadoRede.lampInternaAtiva) {
        if (whDisp <= whMinLI) {
            if (!estadoRede.aguardandoLampInt) {
                estadoRede.aguardandoLampInt = true;
                estadoRede.tsAbaixoLampInt   = agora;
                adicionarLog("Rede: aguardando 1min lamp.interna...");
            } else if ((agora - estadoRede.tsAbaixoLampInt)
                >= TEMPO_DECISAO_REDE_MS) {
                setLampadaInterna(false);
            estadoRede.aguardandoLampInt = false;
            estadoRede.totalCortesLampInt++;
            snprintf(tempBuf, sizeof(tempBuf),
                     "Rede: Lamp.Int DESLIGADA (%.0f<%.0fWh)",
                     whDisp, whMinLI);
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
        if (whDisp > whMinLIOn) {
            setLampadaInterna(true);
            adicionarLog("Rede: Lamp.Int RELIGADA");
        }
    }

    // ════════════════════════════════════════════════════════
    // CONTROLE 3: LÂMPADA EXTERNA (25% padrão)
    // PRIORIDADE: Previsão > Energia > Horário/Painel
    // ════════════════════════════════════════════════════════

    // ── Bloqueio por previsão solar (maior prioridade) ──
    if (estadoPrev.bloqLampExt) {
        // Previsão mandou desligar
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
        // ── Bloqueio por energia baixa ──
        estadoRede.lampExtBloqEnergia = true;
        if (estadoRede.lampExternaAtiva) {
            if (!estadoRede.aguardandoLampExt) {
                estadoRede.aguardandoLampExt = true;
                estadoRede.tsAbaixoLampExt   = agora;
                adicionarLog("Rede: aguardando 1min lamp.externa...");
            } else if ((agora - estadoRede.tsAbaixoLampExt)
                >= TEMPO_DECISAO_REDE_MS) {
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
        // ── Energia OK e sem bloqueio de previsão ──
        if (estadoRede.aguardandoLampExt) {
            estadoRede.aguardandoLampExt = false;
            adicionarLog("Rede: contagem lamp.ext cancelada");
        }
        // Remove bloqueio de energia se recuperou
        if (estadoRede.lampExtBloqEnergia && whDisp > whMinLEOn
            && !estadoPrev.bloqLampExt) {
            estadoRede.lampExtBloqEnergia = false;
        adicionarLog("Rede: Lamp.Ext desbloqueada (energia OK)");
            }
            // Controle por horário ou painel (só se energia OK e sem bloqueio)
            if (!estadoRede.lampExtBloqEnergia) {
                controlarLampadaExterna();
            }
    }

    // ════════════════════════════════════════════════════════
    // CONTROLE 2: REDE ALTA (30% padrão)
    // PRIORIDADE: Previsão > Energia
    // ════════════════════════════════════════════════════════

    // ── Bloqueio por previsão solar ──
    if (estadoPrev.bloqRedeAlta) {
        // Previsão mandou desligar rede alta
        if (estadoRede.redeAltaAtiva) {
            estadoRede.redeAltaAtiva      = false;
            estadoRede.aguardandoRedeAlta = false;
            // Desliga: relé HIGH = sem energia nas tomadas
            //          inversor alta LOW = desligado
            setRedeAlta(true);
            setInversorAlta(false);
            maquinaEstado      = ESTADO_BAIXA_NORMAL;
            estadoInv.modoAlta = false;
            snprintf(tempBuf, sizeof(tempBuf),
                     "PREV: Rede Alta DESLIGADA - %s",
                     estadoPrev.motivoRedeAlta);
            adicionarLog(tempBuf);
            salvarEstadoRede();
        }
    } else if (!estadoPrev.bloqRedeAlta && !estadoRede.redeAltaAtiva
        && estadoPrev.redeAltaFoiDesligada == false) {
        // ── Previsão removeu bloqueio → volta ao controle normal ──
        if (whDisp > whMinRAOn) {
            estadoRede.redeAltaAtiva = true;
            setRedeAlta(false); // LOW = inversor baixa controla
            snprintf(tempBuf, sizeof(tempBuf),
                     "PREV: Rede Alta REATIVADA (%.0f>%.0fWh)",
                     whDisp, whMinRAOn);
            adicionarLog(tempBuf);
        }
        } else {
            // ── Controle normal por energia ──
            if (estadoRede.redeAltaAtiva) {
                if (whDisp <= whMinRA) {
                    if (!estadoRede.aguardandoRedeAlta) {
                        estadoRede.aguardandoRedeAlta = true;
                        estadoRede.tsAbaixoRedeAlta   = agora;
                        adicionarLog("Rede: aguardando 1min rede alta...");
                    } else if ((agora - estadoRede.tsAbaixoRedeAlta)
                        >= TEMPO_DECISAO_REDE_MS) {
                        estadoRede.redeAltaAtiva      = false;
                    setRedeAlta(true);
                    setInversorAlta(false);
                    estadoRede.aguardandoRedeAlta = false;
                    estadoRede.totalCortesRedeAlta++;
                    maquinaEstado      = ESTADO_BAIXA_NORMAL;
                    estadoInv.modoAlta = false;
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
                    estadoRede.redeAltaAtiva = true;
                    setRedeAlta(false);
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Rede: Rede Alta REATIVADA (%.0f>%.0fWh)",
                             whDisp, whMinRAOn);
                    adicionarLog(tempBuf);
                }
            }
        }

        // ════════════════════════════════════════════════════════
        // CONTROLE 1: INVERSOR BAIXA (10% padrão)
        // Não tem bloqueio de previsão (corte mais crítico)
        // ════════════════════════════════════════════════════════
        if (estadoRede.invBaixaAtivo) {
            if (whDisp <= whMinIB) {
                if (!estadoRede.aguardandoInvBaixa) {
                    estadoRede.aguardandoInvBaixa = true;
                    estadoRede.tsAbaixoInvBaixa   = agora;
                    adicionarLog("Rede: aguardando 1min inv.baixa...");
                } else if ((agora - estadoRede.tsAbaixoInvBaixa)
                    >= TEMPO_DECISAO_REDE_MS) {
                    estadoRede.invBaixaAtivo      = false;
                setInversorBaixa(false);
                setRedeAlta(false);
                setInversorAlta(false);
                estadoRede.redeAltaAtiva      = false;
                estadoRede.aguardandoInvBaixa = false;
                estadoRede.totalCortesInvBaixa++;
                maquinaEstado = ESTADO_BAIXA_NORMAL;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: Inv.Baixa DESLIGADO (%.0f<%.0fWh)",
                         whDisp, whMinIB);
                adicionarLog(tempBuf);
                salvarEstadoRede();
                    }
            } else {
                if (estadoRede.aguardandoInvBaixa) {
                    estadoRede.aguardandoInvBaixa = false;
                    adicionarLog("Rede: contagem inv.baixa cancelada");
                }
            }
        } else {
            if (whDisp > whMinIBOn) {
                estadoRede.invBaixaAtivo = true;
                setInversorBaixa(true);
                if (whDisp > whMinRAOn && !estadoPrev.bloqRedeAlta) {
                    estadoRede.redeAltaAtiva = true;
                    setRedeAlta(false);
                }
                snprintf(tempBuf, sizeof(tempBuf),
                         "Rede: Inv.Baixa RELIGADO (%.0f>%.0fWh)",
                         whDisp, whMinIBOn);
                adicionarLog(tempBuf);
            }
        }
}

// ============================================================
// API: /api/rede - JSON da rede elétrica
// ============================================================
void handleApiRede() {
    if (!checarAuth()) return;

    // Reutiliza cálculos já feitos
    float soc = 0.0f;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float eD     = cfgBat.energiaDisponivel;

    float whMinIB = eD * (cfgRede.percInvBaixa / 100.0f);
    float whMinRA = eD * (cfgRede.percRedeAlta  / 100.0f);
    float whMinLE = eD * (cfgRede.percLampExt   / 100.0f);
    float whMinLI = eD * (cfgRede.percLampInt   / 100.0f);

    unsigned long agora = millis();
    int tRestIB = 0, tRestRA = 0, tRestLE = 0, tRestLI = 0;
    if (estadoRede.aguardandoInvBaixa)
        tRestIB = max(0L,(long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoInvBaixa)))/1000;
    if (estadoRede.aguardandoRedeAlta)
        tRestRA = max(0L,(long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoRedeAlta)))/1000;
    if (estadoRede.aguardandoLampExt)
        tRestLE = max(0L,(long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoLampExt)))/1000;
    if (estadoRede.aguardandoLampInt)
        tRestLI = max(0L,(long)(TEMPO_DECISAO_REDE_MS -
        (agora - estadoRede.tsAbaixoLampInt)))/1000;

    // Hora atual para exibição
    char horaAtualStr[6] = "--:--";
    if (ntpSincronizado) {
        time_t    now = time(nullptr);
        struct tm* ti = localtime(&now);
        snprintf(horaAtualStr, sizeof(horaAtualStr),
                 "%02d:%02d", ti->tm_hour, ti->tm_min);
    }

    String j = "{";
    snprintf(tempBuf, sizeof(tempBuf),
             "\"pIB\":%.1f,\"pRA\":%.1f,\"pLE\":%.1f,\"pLI\":%.1f,",
             cfgRede.percInvBaixa, cfgRede.percRedeAlta,
             cfgRede.percLampExt,  cfgRede.percLampInt);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"wIB\":%.0f,\"wRA\":%.0f,\"wLE\":%.0f,\"wLI\":%.0f,",
             whMinIB, whMinRA, whMinLE, whMinLI);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"stIB\":%d,\"stRA\":%d,\"stLE\":%d,\"stLI\":%d,",
             estadoRede.invBaixaAtivo    ? 1 : 0,
             estadoRede.redeAltaAtiva    ? 1 : 0,
             estadoRede.lampExternaAtiva ? 1 : 0,
             estadoRede.lampInternaAtiva ? 1 : 0);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"agIB\":%d,\"agRA\":%d,\"agLE\":%d,\"agLI\":%d,",
             estadoRede.aguardandoInvBaixa ? 1 : 0,
             estadoRede.aguardandoRedeAlta ? 1 : 0,
             estadoRede.aguardandoLampExt  ? 1 : 0,
             estadoRede.aguardandoLampInt  ? 1 : 0);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"tIB\":%d,\"tRA\":%d,\"tLE\":%d,\"tLI\":%d,",
             tRestIB, tRestRA, tRestLE, tRestLI);
    j += tempBuf;
    // Lâmpada externa - modo e flags
    snprintf(tempBuf, sizeof(tempBuf),
             "\"mLE\":%d,\"leHor\":%d,\"lePain\":%d,\"leBloq\":%d,",
             cfgRede.modoLampExt,
             estadoRede.lampExtPorHorario ? 1 : 0,
             estadoRede.lampExtPorPainel  ? 1 : 0,
             estadoRede.lampExtBloqEnergia? 1 : 0);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"ligH\":%d,\"ligM\":%d,\"dslH\":%d,\"dslM\":%d,",
             cfgRede.ligarHora,   cfgRede.ligarMin,
             cfgRede.desligarHora,cfgRede.desligarMin);
    j += tempBuf;
    // Painel
    snprintf(tempBuf, sizeof(tempBuf),
             "\"painelAtivo\":%d,",
             (sensores.correntePainel > 0.5f) ? 1 : 0);
    j += tempBuf;
    // Contadores
    snprintf(tempBuf, sizeof(tempBuf),
             "\"cIB\":%lu,\"cRA\":%lu,\"cLE\":%lu,\"cLI\":%lu,"
             "\"cLigH\":%lu,\"cPain\":%lu,",
             (unsigned long)estadoRede.totalCortesInvBaixa,
             (unsigned long)estadoRede.totalCortesRedeAlta,
             (unsigned long)estadoRede.totalCortesLampExt,
             (unsigned long)estadoRede.totalCortesLampInt,
             (unsigned long)estadoRede.totalLigadasHorario,
             (unsigned long)estadoRede.totalLigadasPainel);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"soc\":%.1f,\"whDisp\":%.0f,\"whTotal\":%.0f,"
             "\"horaAtual\":\"%s\",\"ntp\":%d,\"config\":%d",
             soc, whDisp, eD, horaAtualStr,
             ntpSincronizado ? 1 : 0,
             cfgRede.configurado ? 1 : 0);
    j += tempBuf;
    j += "}";

    server.send(200, "application/json", j);
}

// ============================================================
// CONTROLE DOS RELÉS
// ============================================================
void setupInversores() {
    // Configura pinos dos relés
    pinMode(PIN_RELE_REDE_ALTA,   OUTPUT);
    pinMode(PIN_RELE_INV_BAIXA,   OUTPUT);
    pinMode(PIN_RELE_INV_ALTA,    OUTPUT);

    // Configura pinos de status (entrada com pulldown)
    pinMode(PIN_STATUS_INV_BAIXA, INPUT);
    pinMode(PIN_STATUS_INV_ALTA,  INPUT);

    // Estado inicial seguro:
    // Inversor baixa LIGADO  (relé LOW = ligado)
    // Inversor alta  DESLIGADO (relé LOW = desligado)
    // Rede alta      DESLIGADO (relé LOW = usando inversor baixa)
    setRedeAlta(false);
    setInversorAlta(false);
    setInversorBaixa(true); // true = ligado

    estadoInv.invBaixaLigado = true;
    estadoInv.invAltaLigado  = false;
    estadoInv.modoAlta       = false;
    maquinaEstado = ESTADO_BAIXA_NORMAL;
    strcpy(nomeEstado, "Baixa Normal");

    adicionarLog("Inversores: inicializados");
    snprintf(tempBuf, sizeof(tempBuf),
             "Inv Baixa: %.0fW (%.1fA) | Alta: %.0fW (%.1fA)",
             cfgInv.potenciaBaixa, cfgInv.correnteBaixa,
             cfgInv.potenciaAlta,  cfgInv.correnteAlta);
    adicionarLog(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf),
             "Limite chaveamento: %.1fA (%.0f%% de %.1fA)",
             cfgInv.limiteChaveamento,
             cfgInv.percChaveamento,
             cfgInv.correnteBaixa);
    adicionarLog(tempBuf);
}

// Liga/desliga a rede de alta (comuta entre inversores na rede alta)
void setRedeAlta(bool ligar) {
    estadoInv.releRedeAlta = ligar;
    digitalWrite(PIN_RELE_REDE_ALTA, ligar ? HIGH : LOW);
}

// Liga/desliga o inversor de baixa potência
// ligado=true  → relé LOW  → inversor LIGADO
// ligado=false → relé HIGH → inversor DESLIGADO
void setInversorBaixa(bool ligado) {
    estadoInv.invBaixaLigado = ligado;
    estadoInv.releInvBaixa   = !ligado; // lógica invertida
    digitalWrite(PIN_RELE_INV_BAIXA, ligado ? LOW : HIGH);
}

// Liga/desliga o inversor de alta potência
// ligado=true  → relé HIGH → inversor LIGADO
// ligado=false → relé LOW  → inversor DESLIGADO
void setInversorAlta(bool ligado) {
    estadoInv.invAltaLigado = ligado;
    estadoInv.releInvAlta   = ligado;
    digitalWrite(PIN_RELE_INV_ALTA, ligado ? HIGH : LOW);
}

// ============================================================
// LEITURA DE STATUS DOS INVERSORES
// ============================================================
bool lerStatusInvBaixa() {
    // LED aceso = funcionando OK = HIGH no pino
    return (digitalRead(PIN_STATUS_INV_BAIXA) == HIGH);
}

bool lerStatusInvAlta() {
    return (digitalRead(PIN_STATUS_INV_ALTA) == HIGH);
}

void atualizarStatusInversores() {
    unsigned long agora = millis();

    // Só lê status se o inversor estiver logicamente ligado
    if (estadoInv.invBaixaLigado) {
        bool st = lerStatusInvBaixa();
        if (st) estadoInv.tsStatusBaixa = agora;
        estadoInv.statusInvBaixa = st;
    } else {
        // Inversor desligado - status falso por definição
        estadoInv.statusInvBaixa = false;
    }

    if (estadoInv.invAltaLigado) {
        bool st = lerStatusInvAlta();
        if (st) estadoInv.tsStatusAlta = agora;
        estadoInv.statusInvAlta = st;
    } else {
        estadoInv.statusInvAlta = false;
    }
}

// ============================================================
// MÁQUINA DE ESTADOS - CONTROLE DOS INVERSORES
// ============================================================
void controlarInversores() {
    if (!existeConfigInversor()) return;

    unsigned long agora = millis();
    atualizarStatusInversores();

    float correnteCasa = sensores.correnteCasa;
    bool  tempoMinChav = (agora - estadoInv.tsUltimoChav) >= TEMPO_MIN_CHAVEAMENTO;

    switch (maquinaEstado) {

        // ─────────────────────────────────────────────────────
        case ESTADO_BAIXA_NORMAL:
            // Operando normalmente com inversor de baixa potência
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "Baixa Normal");

            // Verifica proteção do inversor baixa
            if (estadoInv.invBaixaLigado &&
                !estadoInv.statusInvBaixa &&
                (agora - estadoInv.tsStatusBaixa) > TEMPO_DEBOUNCE_STATUS) {
                // Inversor baixa entrou em proteção!
                estadoInv.totalProtecoes++;
            estadoInv.tsProtecao = agora;
            estadoInv.emProtecao = true;
            adicionarLog("PROTECAO: Inversor Baixa caiu! Transferindo para Alta...");
            maquinaEstado = ESTADO_PROTECAO_BAIXA;
            salvarEstadoInversor();
            break;
                }

                // Verifica se corrente ultrapassou o limite de chaveamento
                if (tempoMinChav &&
                    correnteCasa > cfgInv.limiteChaveamento) {
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Carga alta: %.1fA > %.1fA (%.0f%%) - Ligando inversor Alta",
                             correnteCasa,
                             cfgInv.limiteChaveamento,
                             cfgInv.percChaveamento);
                    adicionarLog(tempBuf);
                maquinaEstado = ESTADO_TRANSFERINDO_ALTA;
                estadoInv.tsUltimoChav = agora;
                    }
                    break;
        }

        // ─────────────────────────────────────────────────────
        case ESTADO_TRANSFERINDO_ALTA:
            // Processo de ligar inversor alta e comutar rede alta
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "Transferindo->Alta");

            // 1. Liga inversor de alta potência
            setInversorAlta(true);
            delay(500); // aguarda inversor alta inicializar

            // 2. Aguarda status OK do inversor alta (até 3s)
            unsigned long tEspera = millis();
            bool invAltaOK = false;
            while (millis() - tEspera < TEMPO_RESTART_INV_MS) {
                if (lerStatusInvAlta()) {
                    invAltaOK = true;
                    break;
                }
                delay(50);
                server.handleClient(); // mantém web server respondendo
            }

            if (!invAltaOK) {
                // Inversor alta não respondeu - aborta transferência
                adicionarLog("ERRO: Inversor Alta nao respondeu! Cancelando transferencia");
                setInversorAlta(false);
                estadoInv.totalReiniciamentos++;
                maquinaEstado = ESTADO_BAIXA_NORMAL;
                estadoInv.tsUltimoChav = millis();
                salvarEstadoInversor();
                break;
            }

            // 3. Comuta rede alta para inversor alta
            setRedeAlta(true);
            delay(200);

            estadoInv.modoAlta    = true;
            estadoInv.tsLigouAlta = millis();
            estadoInv.totalChaveamentos++;

            snprintf(tempBuf, sizeof(tempBuf),
                     "Inversor Alta LIGADO! Chaveamentos: %lu",
                     (unsigned long)estadoInv.totalChaveamentos);
            adicionarLog(tempBuf);

            maquinaEstado = ESTADO_ALTA_NORMAL;
            salvarEstadoInversor();
            break;
        }

        // ─────────────────────────────────────────────────────
        case ESTADO_ALTA_NORMAL:
            // Operando com inversor alta, monitorando para retorno
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "Alta Normal");

            // Verifica status do inversor alta
            if (!estadoInv.statusInvAlta &&
                (agora - estadoInv.tsStatusAlta) > TEMPO_DEBOUNCE_STATUS) {
                adicionarLog("PROTECAO: Inversor Alta caiu!");
            estadoInv.totalProtecoes++;
            estadoInv.emProtecao = true;
            maquinaEstado = ESTADO_ERRO;
            salvarEstadoInversor();
            break;
                }

                // Aguarda 1 minuto antes de verificar retorno
                bool esperouMinuto = (agora - estadoInv.tsLigouAlta) >= TEMPO_ESPERA_ALTA_MS;

                if (esperouMinuto && correnteCasa < cfgInv.limiteRetorno) {
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Carga baixou: %.1fA < %.1fA - Retornando para Baixa",
                             correnteCasa, cfgInv.limiteRetorno);
                    adicionarLog(tempBuf);
                    maquinaEstado = ESTADO_TRANSFERINDO_BAIXA;
                    estadoInv.tsUltimoChav = agora;
                }
                break;
        }

        // ─────────────────────────────────────────────────────
        case ESTADO_TRANSFERINDO_BAIXA:
            // Processo de retornar para inversor de baixa potência
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "Transferindo->Baixa");

            // 1. Descomuta rede alta (volta para inversor baixa)
            setRedeAlta(false);
            delay(200);

            // 2. Desliga inversor alta
            setInversorAlta(false);
            delay(200);

            estadoInv.modoAlta  = false;
            estadoInv.emProtecao = false;
            estadoInv.totalChaveamentos++;
            salvarEstadoInversor();

            snprintf(tempBuf, sizeof(tempBuf),
                     "Retornou para Inversor Baixa. Total chav: %lu",
                     (unsigned long)estadoInv.totalChaveamentos);
            adicionarLog(tempBuf);

            maquinaEstado = ESTADO_BAIXA_NORMAL;
            break;
        }

        // ─────────────────────────────────────────────────────
        case ESTADO_PROTECAO_BAIXA:
            // Inversor baixa entrou em proteção - transfere para alta
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "Protecao Baixa");

            // Liga imediatamente o inversor alta para não perder energia
            setInversorAlta(true);
            delay(300);

            // Comuta rede alta para inversor alta
            setRedeAlta(true);

            estadoInv.modoAlta    = true;
            estadoInv.tsLigouAlta = millis();
            estadoInv.totalChaveamentos++;

            adicionarLog("Rede Alta comutada para Inversor Alta (emergencia)");

            // Agora tenta reiniciar o inversor baixa
            maquinaEstado = ESTADO_REINICIANDO_INV;
            estadoInv.tsProtecao = millis();
            break;
        }

        // ─────────────────────────────────────────────────────
        case ESTADO_REINICIANDO_INV:
            // Tenta reiniciar o inversor que entrou em proteção
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "Reiniciando Inversor");

            // Espera 3 segundos para ver se o inversor volta sozinho
            if ((agora - estadoInv.tsProtecao) < TEMPO_RESTART_INV_MS) break;

            // Verifica se o inversor baixa voltou sozinho
            if (lerStatusInvBaixa()) {
                adicionarLog("Inversor Baixa voltou sozinho apos protecao");
                estadoInv.statusInvBaixa = true;
                estadoInv.emProtecao     = false;
                // Fica no modo alta por enquanto, monitorando
                maquinaEstado = ESTADO_ALTA_NORMAL;
                break;
            }

            // Tenta reiniciar: desliga e liga o inversor baixa
            adicionarLog("Reiniciando Inversor Baixa (desliga/liga)...");
            setInversorBaixa(false); // desliga
            delay(2000);
            setInversorBaixa(true);  // liga
            estadoInv.totalReiniciamentos++;
            estadoInv.tsProtecao = millis();

            // Aguarda status por mais 3 segundos
            unsigned long tWait = millis();
            bool voltou = false;
            while (millis() - tWait < TEMPO_RESTART_INV_MS) {
                if (lerStatusInvBaixa()) {
                    voltou = true;
                    break;
                }
                delay(100);
                server.handleClient();
            }

            if (voltou) {
                adicionarLog("Inversor Baixa reiniciado com sucesso!");
                estadoInv.emProtecao = false;
                maquinaEstado = ESTADO_ALTA_NORMAL;
            } else {
                adicionarLog("ERRO: Inversor Baixa nao voltou apos reinicio!");
                // Continua no modo alta, verificando periodicamente
                estadoInv.tsProtecao = millis();
            }
            salvarEstadoInversor();
            break;
        }

        // ─────────────────────────────────────────────────────
        case ESTADO_ERRO:
            // Erro crítico - ambos inversores com problema
            // ─────────────────────────────────────────────────────
        {
            strcpy(nomeEstado, "ERRO CRITICO");
            // Tenta reiniciar tudo após 10 segundos
            if ((agora - estadoInv.tsProtecao) > 10000) {
                adicionarLog("Tentando recuperar de ERRO CRITICO...");
                setRedeAlta(false);
                setInversorAlta(false);
                delay(1000);
                setInversorBaixa(true);
                delay(2000);
                if (lerStatusInvBaixa()) {
                    adicionarLog("Recuperacao OK - voltando modo Baixa");
                    estadoInv.emProtecao = false;
                    estadoInv.modoAlta   = false;
                    maquinaEstado = ESTADO_BAIXA_NORMAL;
                } else {
                    estadoInv.tsProtecao = millis();
                    adicionarLog("Recuperacao FALHOU - tentando novamente em 10s");
                }
                salvarEstadoInversor();
            }
            break;
        }
    }
}

// ============================================================
// API: /api/inversores - JSON dos inversores
// ============================================================
void handleApiInversores() {
    if (!checarAuth()) return;

    float correnteCasa = sensores.correnteCasa;
    float percUso = 0;
    if (cfgInv.correnteBaixa > 0)
        percUso = (correnteCasa / cfgInv.correnteBaixa) * 100.0f;
    percUso = constrain(percUso, 0.0f, 150.0f);

    float percLimite = cfgInv.percChaveamento;
    float ampLimite  = cfgInv.limiteChaveamento;
    float ampRetorno = cfgInv.limiteRetorno;

    // Tempo no modo atual
    unsigned long tempoModo = 0;
    if (estadoInv.modoAlta && estadoInv.tsLigouAlta > 0)
        tempoModo = (millis() - estadoInv.tsLigouAlta) / 1000;

    // Tempo para poder retornar
    unsigned long tempoParaRetorno = 0;
    if (estadoInv.modoAlta && estadoInv.tsLigouAlta > 0) {
        unsigned long decorrido = millis() - estadoInv.tsLigouAlta;
        if (decorrido < TEMPO_ESPERA_ALTA_MS)
            tempoParaRetorno = (TEMPO_ESPERA_ALTA_MS - decorrido) / 1000;
    }

    String j = "{";
    // Config
    snprintf(tempBuf, sizeof(tempBuf),
             "\"pBaixa\":%.0f,\"pAlta\":%.0f,\"percChav\":%.0f,",
             cfgInv.potenciaBaixa, cfgInv.potenciaAlta, cfgInv.percChaveamento);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"iBaixa\":%.1f,\"iAlta\":%.1f,\"iLimite\":%.1f,\"iRetorno\":%.1f,",
             cfgInv.correnteBaixa, cfgInv.correnteAlta,
             ampLimite, ampRetorno);
    j += tempBuf;
    // Estado
    snprintf(tempBuf, sizeof(tempBuf),
             "\"modoAlta\":%d,\"emProtecao\":%d,",
             estadoInv.modoAlta ? 1 : 0,
             estadoInv.emProtecao ? 1 : 0);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"stBaixa\":%d,\"stAlta\":%d,",
             estadoInv.statusInvBaixa ? 1 : 0,
             estadoInv.statusInvAlta  ? 1 : 0);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"rRedeAlta\":%d,\"rInvBaixa\":%d,\"rInvAlta\":%d,",
             estadoInv.releRedeAlta  ? 1 : 0,
             estadoInv.releInvBaixa  ? 1 : 0,
             estadoInv.releInvAlta   ? 1 : 0);
    j += tempBuf;
    // Corrente e uso
    snprintf(tempBuf, sizeof(tempBuf),
             "\"iCasa\":%.2f,\"percUso\":%.1f,",
             correnteCasa, percUso);
    j += tempBuf;
    // Estatísticas
    snprintf(tempBuf, sizeof(tempBuf),
             "\"totalChav\":%lu,\"totalProt\":%lu,\"totalRein\":%lu,",
             (unsigned long)estadoInv.totalChaveamentos,
             (unsigned long)estadoInv.totalProtecoes,
             (unsigned long)estadoInv.totalReiniciamentos);
    j += tempBuf;
    // Máquina de estados
    j += "\"estado\":\"";
    j += String(nomeEstado) + "\",";
    // Tempo
    snprintf(tempBuf, sizeof(tempBuf),
             "\"tempoModo\":%lu,\"tempoRetorno\":%lu,\"config\":%d",
             tempoModo, tempoParaRetorno,
             cfgInv.configurado ? 1 : 0);
    j += tempBuf;
    j += "}";

    server.send(200, "application/json", j);
}

// ============================================================
// SENSOR ADC - LEITURA COM MÉDIA
// ============================================================
float lerADCMedia(uint8_t pin) {
    uint32_t soma = 0;
    for (int i = 0; i < AMOSTRAS_ADC; i++) {
        soma += analogRead(pin);
        delayMicroseconds(100);
    }
    return (float)soma / AMOSTRAS_ADC;
}

// ============================================================
// CALIBRAÇÃO DO ZERO (executar sem corrente)
// ============================================================
void calibrarZeroSensores() {
    adicionarLog("Calibrando zero sensores...");
    // Lê offset real do ADC (com corrente zero idealmente)
    // Valor teórico: 2048 (metade do ADC)
    // Na prática pode variar por ruído do ADC e divisor
    adcOffsetPainel = lerADCMedia(PIN_SENSOR_PAINEL);
    adcOffsetCasa   = lerADCMedia(PIN_SENSOR_CASA);
    snprintf(tempBuf, sizeof(tempBuf),
             "Zero Painel: %.1f | Zero Casa: %.1f",
             adcOffsetPainel, adcOffsetCasa);
    adicionarLog(tempBuf);
}

// ============================================================
// ESTIMATIVA DE TENSÃO DA BATERIA (LiFePO4)
// Método: Mapeamento OCV por SoC
// OCV = tensão em circuito aberto estimada pelo SoC
// ============================================================
float estimarTensaoBateria() {
    // ahMaximos está em EstadoBateria, não em ConfigBateria
    if (cfgBat.amperagem <= 0 || estadoBat.ahMaximos <= 0) return cfgBat.tensaoNominal;

    float soc = estadoBat.ahDisponiveis / estadoBat.ahMaximos;
    soc = constrain(soc, 0.0f, 1.0f);

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
    float ocv  = ocvTabela[idx] + frac * (ocvTabela[idx + 1] - ocvTabela[idx]);
    return ocv * fatorEscala;
}

// ============================================================
// LEITURA DOS SENSORES
// ============================================================
void lerSensores() {
    // --- Painel Solar (ACS758 50A) ---
    float rawPainel = lerADCMedia(PIN_SENSOR_PAINEL);
    float deltaPainel = rawPainel - adcOffsetPainel;
    // Converte delta ADC -> corrente
    // Cada unidade ADC = ADC_VREF/ADC_RESOLUCAO Volts
    float voltsDeltaPainel = deltaPainel * (ADC_VREF / ADC_RESOLUCAO);
    // Sensibilidade 40mV/A -> mas o sinal já passou pelo divisor 3.3/5
    float corrPainel = voltsDeltaPainel / SENSIBILIDADE_50A;

    // --- Casa (ACS758 100A) ---
    float rawCasa = lerADCMedia(PIN_SENSOR_CASA);
    float deltaCasa = rawCasa - adcOffsetCasa;
    float voltsDeltaCasa = deltaCasa * (ADC_VREF / ADC_RESOLUCAO);
    float corrCasa = voltsDeltaCasa / SENSIBILIDADE_100A;

    // --- Filtro de ruído (< 0.3A considera zero) ---
    if (fabsf(corrPainel) < 0.3f) corrPainel = 0.0f;
    if (fabsf(corrCasa)   < 0.3f) corrCasa   = 0.0f;

    // --- Detecção de inversão ---
    // Painel: corrente esperada vai DIMINUIR o sinal (tensão cai de 2.5V)
    // Logo corrente positiva real = delta negativo no ADC
    // Então corrPainel positivo significa sinal subindo = INVERTIDO
    bool painelInvertido = false;
    if (corrPainel > 1.0f) {
        // Mais de 1A na direção errada = fio invertido
        painelInvertido = true;
        corrPainel = -corrPainel; // Corrige sinal para cálculo
    }
    // Menos de 1A invertido = controlador de carga consumindo (normal à noite)
    // Não avisa, usa valor absoluto pequeno como zero efetivo
    if (corrPainel < -0.3f && corrPainel > -1.0f) corrPainel = 0.0f;
    // Garante painel sempre positivo ou zero (nunca negativo real)
    corrPainel = fabsf(corrPainel);

    // Casa: corrente deve ser sempre positiva (consumo)
    bool casaInvertida = false;
    if (corrCasa < -0.5f) {
        casaInvertida = true;
        corrCasa = fabsf(corrCasa);
    }

    // --- Sinal BKP do controlador ---
    // Relé fecha -> 3.3V no pino -> bateria crítica
    bool sinalBKP = (digitalRead(PIN_BKP_BATERIA) == HIGH);

    // --- Tensão estimada da bateria ---
    float tensaoEst = estimarTensaoBateria();

    // --- Potências ---
    float potPainel  = corrPainel  * tensaoEst;
    float potCasa    = corrCasa    * tensaoEst;

    // --- Corrente da bateria (coulomb counting) ---
    // Positivo = carregando, Negativo = descarregando
    float corrBat = corrPainel - corrCasa;
    float potBat  = corrBat * tensaoEst;

    // Atualiza estrutura global
    sensores.correntePainel         = corrPainel;
    sensores.correnteCasa           = corrCasa;
    sensores.correnteBateria        = corrBat;
    sensores.potenciaPainel         = potPainel;
    sensores.potenciaCasa           = potCasa;
    sensores.potenciaBateria        = potBat;
    sensores.tensaoBateriaEst       = tensaoEst;
    sensores.bateriaCarregando      = (corrBat >= 0);
    sensores.alertaPainelInvertido  = painelInvertido;
    sensores.alertaCasaInvertida    = casaInvertida;
    sensores.sinalBKP               = sinalBKP;
}

// ============================================================
// ATUALIZAR ESTADO DA BATERIA (Coulomb Counting)
// ============================================================
void atualizarEstadoBateria(float deltaSegundos) {
    float corrBat = sensores.correnteBateria;

    // Delta Ah = corrente * tempo / 3600
    float deltaAh = corrBat * deltaSegundos / 3600.0f;

    // Detecta bateria 100% carregada:
    // Painel > 0, corrente bateria ≈ 0 (BMS cortou), casa consumindo do painel
    bool painelAtivo   = sensores.correntePainel > 0.5f;
    bool batZero       = fabsf(corrBat) < 0.3f;
    bool casaOuPainel  = sensores.correnteCasa > 0.1f || sensores.correntePainel > 0.1f;

    if (painelAtivo && batZero && casaOuPainel && !estadoBat.carregada100) {
        // Bateria cheia! BMS cortou a carga
        estadoBat.carregada100   = true;
        estadoBat.ahDisponiveis  = estadoBat.ahMaximos;
        estadoBat.tsCarregada    = millis();
        adicionarLog("Bateria 100% carregada detectada!");
        salvarEstadoBateria();
        salvarContadores();
    }

    // BKP: sinal do controlador -> bateria crítica
    if (sensores.sinalBKP && estadoBat.ahDisponiveis > 0) {
        float ahNaBKP = cfgBat.amperagem * (1.0f - cfgBat.percDescargaSegura) * 0.5f;
        if (estadoBat.ahDisponiveis > ahNaBKP) {
            estadoBat.ahDisponiveis = ahNaBKP;
            adicionarLog("BKP: bateria nivel critico!");
            salvarEstadoBateria();
        }
    }

    // Atualiza Ah
    if (estadoBat.carregada100 && deltaAh < 0) {
        // Começou a descarregar após estar cheia
        estadoBat.carregada100 = false;
    }

    estadoBat.ahDisponiveis += deltaAh;
    estadoBat.ahDisponiveis  = constrain(estadoBat.ahDisponiveis, 0.0f, estadoBat.ahMaximos);

    // Ciclos: conta quando descarrega (corrente negativa)
    if (deltaAh < 0) {
        float deltaAhAbs = fabsf(deltaAh);
        estadoBat.ciclosParciais += deltaAhAbs / estadoBat.ahMaximos;
        if (estadoBat.ciclosParciais >= 1.0f) {
            estadoBat.ciclos += 1.0f;
            estadoBat.ciclosParciais -= 1.0f;
            snprintf(tempBuf, sizeof(tempBuf), "Ciclo completo! Total: %.0f", estadoBat.ciclos);
            adicionarLog(tempBuf);
        }
    }

    // Acumuladores para histórico e contadores
    float whPainel = sensores.potenciaPainel * deltaSegundos / 3600.0f;
    float whCasa   = sensores.potenciaCasa   * deltaSegundos / 3600.0f;

    acumWhPainel += whPainel;
    acumWhCasa   += whCasa;

    if (corrBat > 0) {
        float whCarga = sensores.potenciaBateria * deltaSegundos / 3600.0f;
        acumWhCargaBat += whCarga;
        contadores.kwhCargaTotal += whCarga / 1000.0;
    } else if (corrBat < 0) {
        float whDesc = fabsf(sensores.potenciaBateria) * deltaSegundos / 3600.0f;
        acumWhDescBat += whDesc;
        contadores.kwhDescargaTotal += whDesc / 1000.0;
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
    historico[histIdx].whPainel      = (int16_t)(acumWhPainel   * 10.0f);
    historico[histIdx].whCasa        = (int16_t)(acumWhCasa      * 10.0f);
    historico[histIdx].whCargaBat    = (int16_t)(acumWhCargaBat  * 10.0f);
    historico[histIdx].whDescargaBat = (int16_t)(acumWhDescBat   * 10.0f);

    histIdx = (histIdx + 1) % MAX_HISTORICO;
    if (histTotal < MAX_HISTORICO) histTotal++;

    // Zera acumuladores
    acumWhPainel    = 0;
    acumWhCasa      = 0;
    acumWhCargaBat  = 0;
    acumWhDescBat   = 0;

    // Salva contadores e estado a cada histórico
    static uint8_t cntSave = 0;
    if (++cntSave >= 5) { // Salva a cada 5 minutos
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
    if (!lIP.fromString(ip))   { adicionarLog("ERRO: IP invalido");      return false; }
    if (!lGW.fromString(gw))   { adicionarLog("ERRO: Gateway invalido"); return false; }
    if (!lMask.fromString(mask)){ adicionarLog("ERRO: Mascara invalida"); return false; }
    if (!lDNS.fromString(dns)) { adicionarLog("ERRO: DNS invalido");     return false; }

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
        snprintf(tempBuf, sizeof(tempBuf), "WiFi OK! IP: %s", WiFi.localIP().toString().c_str());
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
bool verificarAutenticacao() {
    if (!existeCredenciais()) return true;
    return logado;
}

// Redirect se não autenticado
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

    // ── Navegação (só mostra se logado) ──
    if (logado) {
        html += "<div class='nav'>";

        html += "<a href='/' "
                + String(titulo.indexOf("Painel") >= 0
                         ? "class='ativo'" : "")
                + ">&#127968; Painel</a>";

        html += "<a href='/bateria' "
                + String(titulo.indexOf("Bateria") >= 0
                         ? "class='ativo'" : "")
                + ">&#128267; Bat</a>";

        html += "<a href='/inversores' "
                + String(titulo.indexOf("Invers") >= 0
                         ? "class='ativo'" : "")
                + ">&#9889; Inv</a>";

        html += "<a href='/rede' "
                + String(titulo.indexOf("Rede") >= 0
                         ? "class='ativo'" : "")
                + ">&#128161; Rede</a>";

        // ── Link Previsão Solar (NOVO) ──
        html += "<a href='/previsao' "
                + String(titulo.indexOf("Previs") >= 0
                         ? "class='ativo'" : "")
                + ">&#9728; Prev</a>";

        html += "<a href='/historico' "
                + String(titulo.indexOf("Hist") >= 0
                         ? "class='ativo'" : "")
                + ">&#128202; Hist</a>";

        html += "<a href='/config' "
                + String(titulo.indexOf("Config") >= 0
                         ? "class='ativo'" : "")
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

    // Script base
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
              "d.innerHTML=h||"
              "'<div style=\"color:#555\">Sem logs</div>';"
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

    // Tempo estimado
    float tempoHoras = 0;
    String tempoStr  = "--";
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

    // SoC percentual
    float soc = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    // Energia disponível em Wh
    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;

    // RAM
    uint32_t ram = (uint32_t)esp_get_free_heap_size() / 1024;

    String ip = modoAP ? WiFi.softAPIP().toString()
    : WiFi.localIP().toString();

    // Monta JSON compacto
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
             "\"iB\":%.2f,\"wB\":%.1f,\"vB\":%.2f,",
             sensores.correnteBateria, sensores.potenciaBateria,
             sensores.tensaoBateriaEst);
    j += tempBuf;
    snprintf(tempBuf, sizeof(tempBuf),
             "\"soc\":%.1f,\"whD\":%.0f,",
             soc, whDisp);
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
    // ── Flags de previsão solar ──
    snprintf(tempBuf, sizeof(tempBuf),
             "\"prevOk\":%d,"
             "\"prevBloqRA\":%d,"
             "\"prevBloqLE\":%d,",
             estadoPrev.dadosValidos  ? 1 : 0,
             estadoPrev.bloqRedeAlta  ? 1 : 0,
             estadoPrev.bloqLampExt   ? 1 : 0);
    j += tempBuf;
    // Motivos (escapa aspas para JSON seguro)
    j += "\"prevMRA\":\"";
    for (int c = 0; estadoPrev.motivoRedeAlta[c]; c++) {
        char ch = estadoPrev.motivoRedeAlta[c];
        if (ch == '"') j += "\\\"";
        else           j += ch;
    }
    j += "\",\"prevMLE\":\"";
    for (int c = 0; estadoPrev.motivoLampExt[c]; c++) {
        char ch = estadoPrev.motivoLampExt[c];
        if (ch == '"') j += "\\\"";
        else           j += ch;
    }
    j += "\",";
    snprintf(tempBuf, sizeof(tempBuf),
             "\"prevAmanha\":%.0f,"
             "\"prevRest\":%.0f,"
             "\"prevAcerto\":%.1f",
             estadoPrev.totalAmanha,
             estadoPrev.totalRestanteHoje,
             estadoPrev.taxaAcerto);
    j += tempBuf;
    j += "}";

    server.send(200, "application/json", j);
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

    // Alertas críticos + notificações de previsão
    conteudo += "<div id='alertas'></div>";

    // Status Painel Solar
    conteudo += "<h3>&#9728; Painel Solar</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Corrente</div>"
                "<div class='stat-val stat-ok' id='iP'>--</div>"
                "<div class='stat-unit'>A</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Pot&ecirc;ncia</div>"
                "<div class='stat-val stat-ok' id='wP'>--</div>"
                "<div class='stat-unit'>W</div></div>";
    conteudo += "</div>";

    // Status Casa
    conteudo += "<h3>&#127968; Consumo da Casa</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Corrente</div>"
                "<div class='stat-val stat-warn' id='iC'>--</div>"
                "<div class='stat-unit'>A</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Pot&ecirc;ncia</div>"
                "<div class='stat-val stat-warn' id='wC'>--</div>"
                "<div class='stat-unit'>W</div></div>";
    conteudo += "</div>";

    // Status Bateria
    conteudo += "<h3>&#128267; Bateria</h3>";
    if (!temBat) {
        conteudo += "<div class='msg-warn'>&#9888; Bateria n&atilde;o "
                    "configurada. <a href='/bateria' style='color:#fff'>"
                    "Configurar agora</a></div>";
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
                "color:#546e7a;font-size:10px;margin-bottom:3px;'>";
    conteudo += "<span>Carga da Bateria</span>"
                "<span id='socPct'>--%</span></div>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:4px;height:14px;'>";
    conteudo += "<div id='socBar' style='height:100%;border-radius:4px;"
                "width:0%;background:#66bb6a;transition:width 1s;'></div>"
                "</div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-top:3px;'>";
    conteudo += "<span>Cap. Total: <b id='capTot'>--</b> Wh</span>";
    conteudo += "<span>Dispon&iacute;vel: <b id='whDisp'>--</b> Wh</span>";
    conteudo += "</div></div>";

    // Tempo estimado + ciclos
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label' id='tLbl'>Tempo</div>"
                "<div class='stat-val' id='tStr' style='font-size:14px;'>"
                "--</div></div>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>Ciclos Bateria</div>"
                "<div class='stat-val' id='ciclos'>--</div></div>";
    conteudo += "</div>";

    // ── Resumo Previsão Solar (visível se dados disponíveis) ──
    conteudo += "<div id='resumoPrev' style='display:none;'>";
    conteudo += "<h3>&#9728; Previs&atilde;o Solar</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>Restante Hoje</div>"
                "<div class='stat-val stat-ok' id='prevRest'>--</div>"
                "<div class='stat-unit'>Wh</div></div>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>Amanh&atilde;</div>"
                "<div class='stat-val' id='prevAmanha'>--</div>"
                "<div class='stat-unit'>Wh</div></div>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>Acerto Modelo</div>"
                "<div class='stat-val' id='prevAcerto'>--</div>"
                "<div class='stat-unit'>%</div></div>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>Detalhes</div>"
                "<div class='stat-val' style='font-size:11px;'>"
                "<a href='/previsao' style='color:#4fc3f7;'>"
                "&#9728; Ver</a></div></div>";
    conteudo += "</div></div>";

    // Contadores
    conteudo += "<h3>&#128200; Contadores Totais</h3>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:10px;font-size:11px;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#9728; Painel Gerado</span>"
                "<span class='contador' id='kPai'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#127968; Casa Consumido</span>"
                "<span class='contador' id='kCas'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#128267;&#8679; Bat. Carregado</span>"
                "<span class='contador' id='kCar'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "padding:4px 0;'>"
                "<span style='color:#78909c;'>&#128267;&#8681; Bat. Descarregado</span>"
                "<span class='contador' id='kDes'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "</div>";

    // Status Sistema
    conteudo += "<h3>&#128297; Sistema</h3>";
    conteudo += "<div class='info' style='font-size:11px;'>";
    conteudo += "&#128190; RAM: <b id='ramVal'>--</b> KB &nbsp;|&nbsp; ";
    conteudo += "&#127760; IP: <b id='ipVal'>--</b> &nbsp;|&nbsp; ";
    conteudo += "&#9201; Up: <b id='upVal'>--</b></div>";

    // Botões
    conteudo += "<div class='flex'>";
    conteudo += "<button onclick='location.href=\"/bateria\"' "
                "class='btn-green' style='font-size:12px;'>"
                "&#128267; Bateria</button>";
    conteudo += "<button onclick='location.href=\"/previsao\"' "
                "style='background:#1565c0;font-size:12px;'>"
                "&#9728; Previs&atilde;o</button>";
    conteudo += "<button onclick='location.href=\"/historico\"' "
                "style='background:#1e3a5f;font-size:12px;'>"
                "&#128202; Hist</button>";
    conteudo += "<button onclick='location.href=\"/config\"' "
                "class='btn-gray' style='font-size:12px;'>"
                "&#9881; Config</button>";
    conteudo += "<button onclick='location.href=\"/logout\"' "
                "class='btn-red' style='font-size:12px;'>"
                "&#128275; Sair</button>";
    conteudo += "</div>";

    // Script de atualização
    String script = "";
    script += "var capTotal=" + String(cfgBat.capacidadeTotal, 1) + ";";
    script += "var energDisp=" + String(cfgBat.energiaDisponivel, 1) + ";";

    script += F("function fmt(v,d){"
                "return isNaN(v)?'--':v.toFixed(d);}"

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

                // Bateria corrente/potência
                "var iB=d.iB;"
                "var iBel=document.getElementById('iB');"
                "iBel.textContent=fmt(Math.abs(iB),2);"
                "iBel.className='stat-val '+(iB>=0?'stat-ok':'stat-err');"
                "document.getElementById('wB').textContent="
                "fmt(Math.abs(d.wB),1);"
                "document.getElementById('vB').textContent=fmt(d.vB,2);"

                // Estado bateria
                "var stEl=document.getElementById('stB');"
                "if(d.car){"
                "stEl.textContent='Carregando';"
                "stEl.className='stat-val stat-ok';}"
                "else{"
                "stEl.textContent='Descarregando';"
                "stEl.className='stat-val stat-err';}"

                // SoC
                "var soc=d.soc;"
                "document.getElementById('socPct').textContent="
                "fmt(soc,1)+'%';"
                "var bar=document.getElementById('socBar');"
                "bar.style.width=soc+'%';"
                "bar.style.background=corSoc(soc);"

                // Capacidade e energia disponível
                "document.getElementById('capTot').textContent="
                "fmt(capTotal,0);"
                "document.getElementById('whDisp').textContent="
                "fmt(d.whD,0);"

                // Tempo estimado
                "document.getElementById('tStr').textContent="
                "d.tStr||'--';"
                "document.getElementById('tLbl').textContent="
                "d.tLbl||'Tempo';"

                // Ciclos
                "document.getElementById('ciclos').textContent="
                "fmt(d.cic,2);"

                // Contadores
                "document.getElementById('kPai').textContent="
                "fmtCont(d.kPai);"
                "document.getElementById('kCas').textContent="
                "fmtCont(d.kCas);"
                "document.getElementById('kCar').textContent="
                "fmtCont(d.kCar);"
                "document.getElementById('kDes').textContent="
                "fmtCont(d.kDes);"

                // Sistema
                "document.getElementById('ramVal').textContent=d.ram;"
                "document.getElementById('ipVal').textContent=d.ip;"
                "var up=d.up;"
                "var uh=Math.floor(up/3600);"
                "var um=Math.floor((up%3600)/60);"
                "var us=up%60;"
                "function p(n){return n<10?'0'+n:n;}"
                "document.getElementById('upVal').textContent="
                "uh+'h'+p(um)+'m'+p(us)+'s';"

                // ── Resumo previsão ──
                "var rPrev=document.getElementById('resumoPrev');"
                "if(d.prevOk&&rPrev){"
                "rPrev.style.display='block';"
                "document.getElementById('prevRest').textContent="
                "d.prevRest.toFixed(0);"
                "document.getElementById('prevAmanha').textContent="
                "d.prevAmanha.toFixed(0);"
                "var acEl=document.getElementById('prevAcerto');"
                "acEl.textContent=d.prevAcerto.toFixed(1);"
                "acEl.className='stat-val '+"
                "(d.prevAcerto>=70?'stat-ok':"
                "(d.prevAcerto>=40?'stat-warn':'stat-err'));}"

                // ── Alertas ──
                "var al='';"
                "if(d.aPi)al+='<div class=\"msg-err\">"
                "&#9888; ALERTA: Sensor Painel INVERTIDO!</div>';"
                "if(d.aCi)al+='<div class=\"msg-err\">"
                "&#9888; ALERTA: Sensor Casa INVERTIDO!</div>';"
                "if(d.bkp)al+='<div class=\"msg-warn\">"
                "&#128308; BKP ATIVO: Bateria nivel critico!</div>';"
                "if(d.soc<20&&!d.car)al+='<div class=\"msg-warn\">"
                "&#9889; Bateria abaixo de 20%!</div>';"

                // Alertas de previsão
                "if(d.prevBloqRA)al+='<div class=\"msg-warn\">"
                "&#9728; PREVISAO: Rede Alta desativada. '"
                "+d.prevMRA+'</div>';"
                "if(d.prevBloqLE)al+='<div class=\"msg-warn\">"
                "&#9728; PREVISAO: Lamp.Externa desativada. '"
                "+d.prevMLE+'</div>';"
                "if(d.prevOk&&!d.prevBloqRA&&!d.prevBloqLE"
                "&&d.prevAmanha>0){"
                "al+='<div class=\"msg-ok\" style=\"font-size:10px;\">"
                "&#9728; Prev.amanha: '+d.prevAmanha.toFixed(0)+"
                "'Wh | Restante: '+d.prevRest.toFixed(0)+"
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

            if (vnom < 10.0f || vnom > 60.0f) {
                msgHtml = "<div class='msg-err'>&#10007; Tens&atilde;o nominal inv&aacute;lida (10-60V)</div>";
            } else if (ah < 10.0f || ah > 2000.0f) {
                msgHtml = "<div class='msg-err'>&#10007; Amperagem inv&aacute;lida (10-2000Ah)</div>";
            } else if (pdesc < 0.1f || pdesc > 1.0f) {
                msgHtml = "<div class='msg-err'>&#10007; Descarga segura inv&aacute;lida (10-100%)</div>";
            } else if (vmin < 5.0f || vmin >= vnom) {
                msgHtml = "<div class='msg-err'>&#10007; Tens&atilde;o m&iacute;nima inv&aacute;lida</div>";
            } else {
                cfgBat.tensaoNominal      = vnom;
                cfgBat.amperagem          = ah;
                cfgBat.percDescargaSegura = pdesc;
                cfgBat.tensaoMinima       = vmin;
                cfgBat.capacidadeTotal    = vnom * ah;
                cfgBat.energiaDisponivel  = cfgBat.capacidadeTotal * pdesc;
                if (vnom <= 13.0f)      cfgBat.tensaoMaxima = 14.0f;
                else if (vnom <= 26.0f) cfgBat.tensaoMaxima = 28.0f;
                else                    cfgBat.tensaoMaxima = vnom * 1.094f;

                salvarConfigBateria();
                // Reinicia estado com nova config
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
                snprintf(tempBuf, sizeof(tempBuf),
                         "Cap: %.0fWh Disp: %.0fWh",
                         cfgBat.capacidadeTotal, cfgBat.energiaDisponivel);
                adicionarLog(tempBuf);
                logRAMSeNecessario();

                msgHtml = "<div class='msg-ok'>&#10003; Bateria configurada com sucesso!</div>";
            }
        } else if (acao == "resetciclos") {
            estadoBat.ciclos         = 0;
            estadoBat.ciclosParciais = 0;
            salvarEstadoBateria();
            adicionarLog("Ciclos da bateria resetados");
            msgHtml = "<div class='msg-ok'>&#10003; Ciclos resetados!</div>";
        } else if (acao == "resetah") {
            estadoBat.ahDisponiveis = estadoBat.ahMaximos;
            estadoBat.carregada100  = true;
            salvarEstadoBateria();
            adicionarLog("Estado bateria resetado para 100%");
            msgHtml = "<div class='msg-ok'>&#10003; Bateria marcada como 100% carregada!</div>";
        }
    }

    // SoC atual
    float soc = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);

    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;

    String conteudo = msgHtml;

    // Status atual da bateria
    if (existeBateriaConfig()) {
        conteudo += "<h3>&#128202; Status Atual</h3>";
        conteudo += "<div class='stat-grid'>";

        snprintf(tempBuf, sizeof(tempBuf), "%.2f V", sensores.tensaoBateriaEst);
        conteudo += "<div class='stat-box'><div class='stat-label'>Tens&atilde;o Est.</div>"
        "<div class='stat-val stat-ok'>" + String(sensores.tensaoBateriaEst, 2) + "</div>"
        "<div class='stat-unit'>V</div></div>";

        conteudo += "<div class='stat-box'><div class='stat-label'>Carga (SoC)</div>"
        "<div class='stat-val'>" + String(soc, 1) + "</div>"
        "<div class='stat-unit'>%</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", cfgBat.capacidadeTotal);
        conteudo += "<div class='stat-box'><div class='stat-label'>Cap. Total</div>"
        "<div class='stat-val'>" + String(tempBuf) + "</div>"
        "<div class='stat-unit'>Wh</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", whDisp);
        conteudo += "<div class='stat-box'><div class='stat-label'>Dispon&iacute;vel</div>"
        "<div class='stat-val stat-ok'>" + String(tempBuf) + "</div>"
        "<div class='stat-unit'>Wh</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", cfgBat.energiaDisponivel);
        conteudo += "<div class='stat-box'><div class='stat-label'>Seg. Total</div>"
        "<div class='stat-val'>" + String(tempBuf) + "</div>"
        "<div class='stat-unit'>Wh</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.2f", estadoBat.ciclos + estadoBat.ciclosParciais);
        conteudo += "<div class='stat-box'><div class='stat-label'>Ciclos</div>"
        "<div class='stat-val stat-warn'>" + String(tempBuf) + "</div>"
        "<div class='stat-unit'>ciclos</div></div>";
        conteudo += "</div>";

        // Barra SoC
        String corBar = (soc >= 60) ? "#66bb6a" : (soc >= 30) ? "#ffa726" : "#ef5350";
        conteudo += "<div style='margin:8px 0;'>";
        conteudo += "<div style='background:#111;border:1px solid #333;border-radius:4px;height:18px;'>";
        conteudo += "<div style='height:100%;border-radius:4px;width:" + String(soc, 0) + "%;background:" + corBar + ";'></div>";
        conteudo += "</div></div>";

        // Info configuração atual
        conteudo += "<div class='info' style='font-size:11px;'>";
        conteudo += "&#9889; Nominal: <b>" + String(cfgBat.tensaoNominal, 1) + "V</b> &nbsp;|&nbsp; ";
        conteudo += "&#128267; " + String(cfgBat.amperagem, 0) + "Ah &nbsp;|&nbsp; ";
        conteudo += "Desc. segura: <b>" + String(cfgBat.percDescargaSegura * 100.0f, 0) + "%</b><br>";
        conteudo += "V m&iacute;n (BKP): <b>" + String(cfgBat.tensaoMinima, 1) + "V</b> &nbsp;|&nbsp; ";
        conteudo += "V m&aacute;x: <b>" + String(cfgBat.tensaoMaxima, 1) + "V</b> &nbsp;|&nbsp; ";
        conteudo += "AhDisp: <b>" + String(estadoBat.ahDisponiveis, 1) + "Ah</b>";
        conteudo += "</div>";

        // Ações rápidas
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

    // Formulário de configuração
    conteudo += "<h3>&#9881; Configurar Bateria LiFePO4</h3>";
    conteudo += "<form method='POST' action='/bateria'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";

    conteudo += "<label>Tens&atilde;o Nominal (V):</label>";
    conteudo += "<select name='vnom'>";
    float opts[] = {12.8f, 25.6f, 38.4f, 51.2f};
    for (int i = 0; i < 4; i++) {
        conteudo += "<option value='" + String(opts[i], 1) + "'";
        if (fabsf(cfgBat.tensaoNominal - opts[i]) < 0.1f) conteudo += " selected";
        conteudo += ">" + String(opts[i], 1) + "V (";
        if (i == 0) conteudo += "12V 4S)";
        else if (i == 1) conteudo += "24V 8S)";
        else if (i == 2) conteudo += "36V 12S)";
        else conteudo += "48V 16S)";
        conteudo += "</option>";
    }
    conteudo += "</select>";

    conteudo += "<label>Capacidade (Ah):</label>";
    conteudo += "<input type='number' name='ah' min='10' max='2000' step='1' required ";
    conteudo += "value='" + String((int)cfgBat.amperagem) + "' placeholder='Ex: 100'>";

    conteudo += "<label>Descarga Segura (%) - recomendado 80%:</label>";
    conteudo += "<input type='number' name='pdesc' min='10' max='100' step='1' required ";
    conteudo += "value='" + String((int)(cfgBat.percDescargaSegura * 100.0f)) + "' placeholder='Ex: 80'>";

    conteudo += "<label>Tens&atilde;o M&iacute;nima BKP (V) - configurada no controlador:</label>";
    conteudo += "<input type='number' name='vmin' min='5' max='55' step='0.1' required ";
    conteudo += "value='" + String(cfgBat.tensaoMinima, 1) + "' placeholder='Ex: 12.0'>";

    conteudo += "<div class='info' style='margin-top:10px;font-size:11px;'>";
    conteudo += "&#9432; LiFePO4: Tens&atilde;o m&aacute;x autom&aacute;tica:<br>";
    conteudo += "12.8V &rarr; 14.0V &nbsp;|&nbsp; 25.6V &rarr; 28.0V<br>";
    conteudo += "Descarga segura recomendada: 80% (preserva vida &uacute;til)";
    conteudo += "</div>";

    conteudo += "<button type='submit' class='btn-green'>&#128190; Salvar Configura&ccedil;&atilde;o</button>";
    conteudo += "</form>";
    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' style='margin-top:8px;'>"
    "&#8592; Voltar</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Bateria", conteudo));
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

    // Filtro por data
    String filtroData = server.arg("data");
    bool   temFiltro  = filtroData.length() == 10; // YYYY-MM-DD

    adicionarLog("Historico acessado");
    logRAMSeNecessario();

    String conteudo = "";

    // Formulário de busca
    conteudo += "<h3>&#128202; Hist&oacute;rico de Energia</h3>";
    conteudo += "<form method='GET' action='/historico' style='display:flex;gap:6px;margin-bottom:10px;'>";
    conteudo += "<input type='date' name='data' value='" + filtroData + "' "
    "style='flex:1;padding:8px;background:#111;border:1px solid #444;"
    "border-radius:4px;color:#e0e0e0;font-size:12px;'>";
    conteudo += "<button type='submit' style='width:auto;padding:8px 12px;margin:0;"
    "font-size:12px;'>&#128269; Filtrar</button>";
    if (temFiltro) {
        conteudo += "<a href='/historico' style='padding:8px 10px;background:#546e7a;"
        "color:#fff;border-radius:4px;font-size:12px;text-decoration:none;"
        "display:flex;align-items:center;'>&#10005;</a>";
    }
    conteudo += "</form>";

    if (histTotal == 0) {
        conteudo += "<div class='msg-warn'>&#9888; Nenhum registro no hist&oacute;rico ainda.</div>";
        conteudo += "<div class='info' style='font-size:11px;'>O sistema salva um registro "
        "a cada 60 segundos. Aguarde...</div>";
    } else {
        // Totais do período filtrado
        double totalPainelWh  = 0;
        double totalCasaWh    = 0;
        double totalCargaWh   = 0;
        double totalDescWh    = 0;
        int    registrosFilt  = 0;

        // Cabeçalho tabela
        conteudo += "<div style='overflow-x:auto;'>";
        conteudo += "<table style='width:100%;border-collapse:collapse;font-size:10px;'>";
        conteudo += "<thead><tr style='background:#1e3a5f;color:#90caf9;'>";
        conteudo += "<th style='padding:6px 4px;text-align:left;'>Data/Hora</th>";
        conteudo += "<th style='padding:6px 4px;text-align:right;'>&#9728; Painel</th>";
        conteudo += "<th style='padding:6px 4px;text-align:right;'>&#127968; Casa</th>";
        conteudo += "<th style='padding:6px 4px;text-align:right;'>&#8679; Carga</th>";
        conteudo += "<th style='padding:6px 4px;text-align:right;'>&#8681; Desc.</th>";
        conteudo += "</tr></thead><tbody>";

        // Percorre histórico do mais recente para o mais antigo
        int exibidos = 0;
        int maxExibir = temFiltro ? MAX_HISTORICO : 60; // Sem filtro mostra última hora

        for (int i = 0; i < histTotal && exibidos < maxExibir; i++) {
            int idx = ((int)histIdx - 1 - i + MAX_HISTORICO) % MAX_HISTORICO;
            uint32_t ts = historico[idx].timestamp;

            // Filtro por data
            if (temFiltro && ts > 0) {
                struct tm* ti = localtime((time_t*)&ts);
                char dataBuf[12];
                snprintf(dataBuf, sizeof(dataBuf), "%04d-%02d-%02d",
                         ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday);
                if (filtroData != String(dataBuf)) continue;
            }

            // Valores reais (dividir por 10 pois salvamos x10)
            float wPai = historico[idx].whPainel      / 10.0f;
            float wCas = historico[idx].whCasa        / 10.0f;
            float wCar = historico[idx].whCargaBat    / 10.0f;
            float wDes = historico[idx].whDescargaBat / 10.0f;

            totalPainelWh += wPai;
            totalCasaWh   += wCas;
            totalCargaWh  += wCar;
            totalDescWh   += wDes;
            registrosFilt++;

            // Formata data/hora
            char dhBuf[20] = "--/-- --:--";
            if (ts > 0) {
                struct tm* ti = localtime((time_t*)&ts);
                snprintf(dhBuf, sizeof(dhBuf), "%02d/%02d %02d:%02d",
                         ti->tm_mday, ti->tm_mon + 1,
                         ti->tm_hour, ti->tm_min);
            }

            // Linha alternada
            const char* bg = (exibidos % 2 == 0) ? "#0d0d0d" : "#111";
            conteudo += "<tr style='background:" + String(bg) + ";'>";
            conteudo += "<td style='padding:5px 4px;color:#78909c;'>" + String(dhBuf) + "</td>";

            // Painel
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wPai);
            conteudo += "<td style='padding:5px 4px;text-align:right;color:#66bb6a;'>"
            + String(tempBuf) + " Wh</td>";
            // Casa
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wCas);
            conteudo += "<td style='padding:5px 4px;text-align:right;color:#ffa726;'>"
            + String(tempBuf) + " Wh</td>";
            // Carga bat
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wCar);
            conteudo += "<td style='padding:5px 4px;text-align:right;color:#4fc3f7;'>"
            + String(tempBuf) + " Wh</td>";
            // Descarga bat
            snprintf(tempBuf, sizeof(tempBuf), "%.1f", wDes);
            conteudo += "<td style='padding:5px 4px;text-align:right;color:#ef5350;'>"
            + String(tempBuf) + " Wh</td>";

            conteudo += "</tr>";
            exibidos++;
        }
        conteudo += "</tbody></table></div>";

        // Totais do período
        if (registrosFilt > 0) {
            conteudo += "<div style='background:#1e3a5f;border-radius:4px;padding:10px;"
            "margin-top:8px;font-size:11px;'>";
            conteudo += "<b style='color:#90caf9;'>&#8721; Totais do Per&iacute;odo</b><br>";

            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalPainelWh / 1000.0);
            conteudo += "<span style='color:#66bb6a;'>&#9728; Painel: <b>"
            + String(tempBuf) + " kWh</b></span> &nbsp;";

            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalCasaWh / 1000.0);
            conteudo += "<span style='color:#ffa726;'>&#127968; Casa: <b>"
            + String(tempBuf) + " kWh</b></span><br>";

            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalCargaWh / 1000.0);
            conteudo += "<span style='color:#4fc3f7;'>&#8679; Carga Bat: <b>"
            + String(tempBuf) + " kWh</b></span> &nbsp;";

            snprintf(tempBuf, sizeof(tempBuf), "%.3f", totalDescWh / 1000.0);
            conteudo += "<span style='color:#ef5350;'>&#8681; Desc. Bat: <b>"
            + String(tempBuf) + " kWh</b></span>";

            snprintf(tempBuf, sizeof(tempBuf), "%d", registrosFilt);
            conteudo += "<br><span style='color:#546e7a;'>Registros: "
            + String(tempBuf) + " (intervalo: 60s)</span>";
            conteudo += "</div>";
        }

        // Info memória
        snprintf(tempBuf, sizeof(tempBuf),
                 "Hist&oacute;rico: %d/%d registros (%.0f%% cheio)",
                 histTotal, MAX_HISTORICO,
                 (histTotal / (float)MAX_HISTORICO) * 100.0f);
        conteudo += "<div class='info' style='font-size:10px;margin-top:8px;'>"
        "&#128190; " + String(tempBuf) + "<br>";
        conteudo += "Quando cheio, registros antigos s&atilde;o substitu&iacute;dos "
        "automaticamente.</div>";

        if (!temFiltro && histTotal > 60) {
            conteudo += "<div class='msg-warn' style='font-size:11px;'>"
            "&#9432; Exibindo &uacute;ltimos 60 registros. "
            "Use o filtro de data para ver todo o hist&oacute;rico.</div>";
        }
    }

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
    "style='margin-top:10px;font-size:12px;'>&#8592; Voltar</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Hist&oacute;rico", conteudo));
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

    String msgHtml = "";
    String abaAtiva = server.arg("aba");
    if (abaAtiva.length() == 0) abaAtiva = "wifi";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // --- Salvar WiFi ---
        if (acao == "salvar_wifi") {
            String modo  = server.arg("modo");
            bool   usarAP = (modo == "ap");
            String tz    = server.arg("timezone");
            if (tz.length() == 0) tz = "BRT3";

            if (usarAP) {
                salvarWifi("", "", "", "", "", "", tz, true);
                adicionarLog("Config WiFi: modo AP salvo");
                msgHtml = "<div class='msg-ok'>&#10003; Salvo! Reiniciando...</div>";
                server.send(200, "text/html; charset=utf-8",
                            gerarPagina("RRFS - Config", msgHtml));
                delay(3000);
                ESP.restart();
                return;
            }

            String ssid = server.arg("ssid");
            String wpass= server.arg("wpass");
            String ip   = server.arg("ip");
            String gw   = server.arg("gw");
            String mask = server.arg("mask");
            String dns  = server.arg("dns");

            if (ssid.length() == 0 || ip.length() < 7 || gw.length() < 7) {
                msgHtml = "<div class='msg-err'>&#10007; Preencha todos os campos obrigat&oacute;rios</div>";
                abaAtiva = "wifi";
            } else {
                salvarWifi(ssid, wpass, ip, gw, mask, dns, tz, false);
                snprintf(tempBuf, sizeof(tempBuf), "WiFi reconfigurado: %s", ssid.c_str());
                adicionarLog(tempBuf);
                msgHtml = "<div class='msg-ok'>&#10003; WiFi salvo! Reiniciando em 3s...</div>";
                server.send(200, "text/html; charset=utf-8",
                            gerarPagina("RRFS - Config", msgHtml));
                delay(3000);
                ESP.restart();
                return;
            }
        }

        // --- Alterar Senha ---
        else if (acao == "alterar_senha") {
            String userAtual = server.arg("user_atual");
            String passAtual = server.arg("pass_atual");
            String passNova  = server.arg("pass_nova");
            String passConf  = server.arg("pass_conf");
            abaAtiva = "senha";

            if (!verificarLogin(userAtual, passAtual)) {
                msgHtml = "<div class='msg-err'>&#10007; Usu&aacute;rio ou senha atual incorretos</div>";
            } else if (passNova.length() < 4) {
                msgHtml = "<div class='msg-err'>&#10007; Nova senha: m&iacute;nimo 4 caracteres</div>";
            } else if (passNova != passConf) {
                msgHtml = "<div class='msg-err'>&#10007; As senhas n&atilde;o conferem</div>";
            } else {
                salvarCredenciais(userAtual, passNova);
                adicionarLog("Senha alterada com sucesso");
                msgHtml = "<div class='msg-ok'>&#10003; Senha alterada!</div>";
            }
        }

        // --- Exportar (download JSON via link) ---
        else if (acao == "exportar") {
            // Monta JSON de exportação
            String json = "{\"versao\":\"" VERSAO "\",";
            json += "\"bateria\":{";
            json += "\"vnom\":"  + String(cfgBat.tensaoNominal, 2)      + ",";
            json += "\"ah\":"    + String(cfgBat.amperagem, 2)           + ",";
            json += "\"pdesc\":" + String(cfgBat.percDescargaSegura, 3)  + ",";
            json += "\"vmin\":"  + String(cfgBat.tensaoMinima, 2)        + "},";
            json += "\"estado\":{";
            json += "\"ahDisp\":" + String(estadoBat.ahDisponiveis, 3)   + ",";
            json += "\"ciclos\":" + String(estadoBat.ciclos, 2)           + ",";
            json += "\"ciclosP\":" + String(estadoBat.ciclosParciais, 4)  + "},";
            json += "\"contadores\":{";
            json += "\"kwhPai\":"  + String(contadores.kwhPainelTotal,   6) + ",";
            json += "\"kwhCas\":"  + String(contadores.kwhCasaTotal,     6) + ",";
            json += "\"kwhCar\":"  + String(contadores.kwhCargaTotal,    6) + ",";
            json += "\"kwhDes\":"  + String(contadores.kwhDescargaTotal, 6) + "}}";

            server.sendHeader("Content-Disposition",
                              "attachment; filename=rrfs_backup.json");
            server.send(200, "application/json", json);
            adicionarLog("Exportacao de dados realizada");
            return;
}

// --- Importar dados ---
else if (acao == "importar") {
    String json   = server.arg("json_dados");
    String campos = server.arg("importar_campos");
    abaAtiva = "dados";

    bool ok = false;

    // Parse simples do JSON
    if (json.length() > 10) {
        // Importar configuração bateria
        if (campos == "tudo" || campos == "bateria") {
            int p;
            p = json.indexOf("\"vnom\":");
            if (p >= 0) cfgBat.tensaoNominal = json.substring(p+7).toFloat();
            p = json.indexOf("\"ah\":");
            if (p >= 0) cfgBat.amperagem = json.substring(p+5).toFloat();
            p = json.indexOf("\"pdesc\":");
            if (p >= 0) cfgBat.percDescargaSegura = json.substring(p+8).toFloat();
            p = json.indexOf("\"vmin\":");
            if (p >= 0) cfgBat.tensaoMinima = json.substring(p+7).toFloat();
            cfgBat.capacidadeTotal   = cfgBat.tensaoNominal * cfgBat.amperagem;
            cfgBat.energiaDisponivel = cfgBat.capacidadeTotal * cfgBat.percDescargaSegura;
            if (cfgBat.tensaoNominal <= 13.0f)      cfgBat.tensaoMaxima = 14.0f;
            else if (cfgBat.tensaoNominal <= 26.0f) cfgBat.tensaoMaxima = 28.0f;
            else cfgBat.tensaoMaxima = cfgBat.tensaoNominal * 1.094f;
            salvarConfigBateria();
            ok = true;
        }

        // Importar estado bateria
        if (campos == "tudo" || campos == "estado") {
            int p;
            p = json.indexOf("\"ahDisp\":");
            if (p >= 0) estadoBat.ahDisponiveis = json.substring(p+9).toFloat();
            p = json.indexOf("\"ciclos\":");
            if (p >= 0) estadoBat.ciclos = json.substring(p+9).toFloat();
            p = json.indexOf("\"ciclosP\":");
            if (p >= 0) estadoBat.ciclosParciais = json.substring(p+10).toFloat();
            estadoBat.ahMaximos = cfgBat.amperagem * cfgBat.percDescargaSegura;
            salvarEstadoBateria();
            ok = true;
        }

        // Importar contadores
        if (campos == "tudo" || campos == "contadores") {
            int p;
            p = json.indexOf("\"kwhPai\":");
            if (p >= 0) contadores.kwhPainelTotal   = json.substring(p+9).toDouble();
            p = json.indexOf("\"kwhCas\":");
            if (p >= 0) contadores.kwhCasaTotal     = json.substring(p+9).toDouble();
            p = json.indexOf("\"kwhCar\":");
            if (p >= 0) contadores.kwhCargaTotal    = json.substring(p+9).toDouble();
            p = json.indexOf("\"kwhDes\":");
            if (p >= 0) contadores.kwhDescargaTotal = json.substring(p+9).toDouble();
            salvarContadores();
            ok = true;
        }
    }

    if (ok) {
        adicionarLog("Importacao de dados OK");
        msgHtml = "<div class='msg-ok'>&#10003; Dados importados com sucesso!</div>";
    } else {
        msgHtml = "<div class='msg-err'>&#10007; Erro ao importar: JSON inv&aacute;lido</div>";
    }
}
}

// ---- Escaneia redes para aba WiFi ----
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
        snprintf(tempBuf, sizeof(tempBuf), " (%d dBm)", r);
        opcoesRede += String(tempBuf) + "</option>";
    }
    WiFi.scanDelete();
}

// Carrega WiFi atual
String cSsid, cPass, cIp, cGw, cMask, cDns, cTz;
bool   cAp;
carregarWifi(cSsid, cPass, cIp, cGw, cMask, cDns, cTz, cAp);

String conteudo = msgHtml;

// Abas de navegação
conteudo += "<div style='display:flex;gap:4px;margin-bottom:12px;flex-wrap:wrap;'>";
String abas[] = {"wifi", "senha", "dados"};
String abasNome[] = {"&#127760; WiFi", "&#128274; Senha", "&#128190; Dados"};
for (int i = 0; i < 3; i++) {
    String estilo = (abaAtiva == abas[i])
    ? "background:#0277bd;color:#fff;"
    : "background:#1a1a1a;color:#90a4ae;border:1px solid #333;";
    conteudo += "<a href='/config?aba=" + abas[i] + "' style='flex:1;text-align:center;"
    "padding:8px 4px;border-radius:4px;text-decoration:none;font-size:12px;"
    "font-weight:bold;" + estilo + "'>" + abasNome[i] + "</a>";
}
conteudo += "</div>";

// ============ ABA WiFi ============
if (abaAtiva == "wifi") {
    conteudo += "<h3>&#127760; Configura&ccedil;&atilde;o de Rede</h3>";
    conteudo += "<form method='POST' action='/config?aba=wifi'>";
    conteudo += "<input type='hidden' name='acao' value='salvar_wifi'>";

    conteudo += "<label>Modo de Conex&atilde;o:</label>";
    conteudo += "<select name='modo' id='modoSel' onchange='toggleModo()'>";
    conteudo += "<option value='wifi'" + String(!cAp ? " selected" : "") + ">Conectar ao WiFi</option>";
    conteudo += "<option value='ap'"   + String( cAp ? " selected" : "") + ">Ponto de Acesso (AP)</option>";
    conteudo += "</select>";

    conteudo += "<div id='divwifi'>";
    conteudo += "<label>Rede WiFi:</label>";
    conteudo += "<select name='ssid'>";
    if (opcoesRede.length() == 0) {
        conteudo += "<option value=''>Nenhuma rede encontrada</option>";
    } else {
        conteudo += opcoesRede;
    }
    conteudo += "</select>";

    conteudo += "<button type='button' class='btn-orange' style='font-size:11px;margin-top:6px;' "
    "onclick='location.href=\"/config?aba=wifi\"'>&#128259; Atualizar Redes</button>";

    conteudo += "<label>Senha WiFi:</label>";
    conteudo += "<input type='password' name='wpass' maxlength='64' placeholder='Senha do WiFi'>";
    conteudo += "<label>IP Fixo do ESP32:</label>";
    conteudo += "<input type='text' name='ip' required value='" + cIp + "' placeholder='Ex: 192.168.1.100'>";
    conteudo += "<label>Gateway:</label>";
    conteudo += "<input type='text' name='gw' required value='" + cGw + "' placeholder='Ex: 192.168.1.1'>";
    conteudo += "<label>M&aacute;scara:</label>";
    conteudo += "<input type='text' name='mask' required value='" + cMask + "'>";
    conteudo += "<label>DNS:</label>";
    conteudo += "<input type='text' name='dns' required value='" + cDns + "'>";
    conteudo += "</div>";

    conteudo += "<label>Fuso Hor&aacute;rio:</label>";
    conteudo += "<select name='timezone'>";
    // Fusos horários comuns
    struct { const char* val; const char* nome; } fusos[] = {
        {"BRT3",              "Brasil - Bras&iacute;lia (UTC-3)"},
        {"BRT4",              "Brasil - Manaus (UTC-4)"},
        {"BRT5",              "Brasil - Acre (UTC-5)"},
        {"UTC0",              "UTC (GMT+0)"},
        {"EST5EDT,M3.2.0,M11.1.0", "EUA - Leste"},
        {"PST8PDT,M3.2.0,M11.1.0", "EUA - Pacífico"},
        {"CET-1CEST,M3.5.0,M10.5.0/3", "Europa Central"},
    };
    for (int i = 0; i < 7; i++) {
        conteudo += "<option value='" + String(fusos[i].val) + "'";
        if (cTz == fusos[i].val) conteudo += " selected";
        conteudo += ">" + String(fusos[i].nome) + "</option>";
    }
    conteudo += "</select>";

    conteudo += "<button type='submit' class='btn-green'>&#128190; Salvar e Reiniciar</button>";
    conteudo += "</form>";

    conteudo += "<script>";
    conteudo += "function toggleModo(){";
    conteudo += "var m=document.getElementById('modoSel').value;";
    conteudo += "document.getElementById('divwifi').style.display=(m=='wifi'?'block':'none');}";
    conteudo += "toggleModo();";
    conteudo += "</script>";
}

// ============ ABA Senha ============
else if (abaAtiva == "senha") {
    conteudo += "<h3>&#128274; Alterar Senha de Acesso</h3>";
    conteudo += "<form method='POST' action='/config?aba=senha'>";
    conteudo += "<input type='hidden' name='acao' value='alterar_senha'>";
    conteudo += "<label>Usu&aacute;rio:</label>";
    conteudo += "<input type='text' name='user_atual' required autocomplete='off' "
    "placeholder='Usu&aacute;rio atual'>";
    conteudo += "<label>Senha Atual:</label>";
    conteudo += "<input type='password' name='pass_atual' required placeholder='Senha atual'>";
    conteudo += "<label>Nova Senha (m&iacute;n. 4 caracteres):</label>";
    conteudo += "<input type='password' name='pass_nova' required placeholder='Nova senha'>";
    conteudo += "<label>Confirmar Nova Senha:</label>";
    conteudo += "<input type='password' name='pass_conf' required placeholder='Confirme a nova senha'>";
    conteudo += "<button type='submit' class='btn-green'>&#128274; Alterar Senha</button>";
    conteudo += "</form>";
}

// ============ ABA Dados ============
else if (abaAtiva == "dados") {
    conteudo += "<h3>&#128190; Exportar Dados</h3>";
    conteudo += "<div class='info' style='font-size:11px;'>";
    conteudo += "Exporta configura&ccedil;&atilde;o da bateria, estado atual, ciclos e contadores em formato JSON.</div>";
    conteudo += "<form method='POST' action='/config?aba=dados'>";
    conteudo += "<input type='hidden' name='acao' value='exportar'>";
    conteudo += "<button type='submit' class='btn-green'>&#8659; Exportar Backup JSON</button>";
    conteudo += "</form>";

    conteudo += "<h3>&#128228; Importar Dados</h3>";
    conteudo += "<form method='POST' action='/config?aba=dados'>";
    conteudo += "<input type='hidden' name='acao' value='importar'>";
    conteudo += "<label>Selecionar dados a importar:</label>";
    conteudo += "<select name='importar_campos'>";
    conteudo += "<option value='tudo'>Tudo (bateria + estado + contadores)</option>";
    conteudo += "<option value='bateria'>Apenas configura&ccedil;&atilde;o da bateria</option>";
    conteudo += "<option value='estado'>Apenas estado (Ah disp. + ciclos)</option>";
    conteudo += "<option value='contadores'>Apenas contadores kWh</option>";
    conteudo += "</select>";
    conteudo += "<label>Cole o JSON do backup:</label>";
    conteudo += "<textarea name='json_dados' rows='6' style='width:100%;background:#111;"
    "border:1px solid #444;border-radius:4px;color:#e0e0e0;font-size:11px;"
    "padding:8px;box-sizing:border-box;font-family:monospace;' "
    "placeholder='{\"versao\":\"V4.0\",...}'></textarea>";
    conteudo += "<button type='submit' class='btn-orange'>&#8657; Importar Dados</button>";
    conteudo += "</form>";

    // Info sistema
    conteudo += "<h3>&#128297; Informa&ccedil;&otilde;es do Sistema</h3>";
    conteudo += "<div class='info' style='font-size:11px;'>";
    snprintf(tempBuf, sizeof(tempBuf), "%u", (unsigned)esp_get_free_heap_size() / 1024);
    conteudo += "&#128190; RAM livre: <b>" + String(tempBuf) + " KB</b><br>";
    snprintf(tempBuf, sizeof(tempBuf), "%d/%d", histTotal, MAX_HISTORICO);
    conteudo += "&#128202; Hist&oacute;rico: <b>" + String(tempBuf) + " registros</b><br>";
    conteudo += "&#9881; Vers&atilde;o: <b>" VERSAO "</b><br>";
    conteudo += "&#127760; Modo: <b>" + String(modoAP ? "AP" : "WiFi Cliente") + "</b><br>";
    if (!modoAP) {
        conteudo += "&#128246; SSID: <b>" + WiFi.SSID() + "</b><br>";
        conteudo += "&#127760; IP: <b>" + WiFi.localIP().toString() + "</b>";
    }
    conteudo += "</div>";

    // Reset total
    conteudo += "<h3 style='color:#ef5350;'>&#9888; Zona de Perigo</h3>";
    conteudo += "<div class='msg-warn' style='font-size:11px;'>&#9888; O bot&atilde;o BOOT "
    "pressionado por 10s apaga TODAS as configura&ccedil;&otilde;es e reinicia!</div>";
}

conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
"style='margin-top:10px;font-size:12px;'>&#8592; Voltar</button>";

server.send(200, "text/html; charset=utf-8",
            gerarPagina("RRFS Energia - Config", conteudo));
}

// ============================================================
// PÁGINA: /setup-login
// ============================================================
void handleSetupLogin() {
    if (existeCredenciais() && existeWiFiConfig()) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi");
        server.send(302); return;
    }

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String user = server.arg("user");
        String pass = server.arg("pass");
        String conf = server.arg("conf");

        if (user.length() < 3) {
            msgHtml = "<div class='msg-err'>&#10007; Usu&aacute;rio: m&iacute;nimo 3 caracteres</div>";
        } else if (pass.length() < 4) {
            msgHtml = "<div class='msg-err'>&#10007; Senha: m&iacute;nimo 4 caracteres</div>";
        } else if (pass != conf) {
            msgHtml = "<div class='msg-err'>&#10007; As senhas n&atilde;o conferem</div>";
        } else {
            salvarCredenciais(user, pass);
            adicionarLog("Credenciais criadas");
            logRAMSeNecessario();
            server.sendHeader("Location", "/setup-wifi");
            server.send(302); return;
        }
    }

    String conteudo = "<div class='info'>";
    conteudo += "&#9432; <b>Primeira configura&ccedil;&atilde;o.</b> Crie suas credenciais.";
    conteudo += "</div>";
    conteudo += "<h3>&#128274; Criar Conta</h3>";
    conteudo += msgHtml;
    conteudo += "<form method='POST' action='/setup-login'>";
    conteudo += "<label>Usu&aacute;rio (m&iacute;n. 3 caracteres):</label>";
    conteudo += "<input type='text' name='user' maxlength='32' required "
    "placeholder='Nome de usu&aacute;rio' autocomplete='off'>";
    conteudo += "<label>Senha (m&iacute;n. 4 caracteres):</label>";
    conteudo += "<input type='password' name='pass' maxlength='32' required placeholder='Senha'>";
    conteudo += "<label>Confirmar Senha:</label>";
    conteudo += "<input type='password' name='conf' maxlength='32' required "
    "placeholder='Confirme a senha'>";
    conteudo += "<button type='submit'>Criar e Continuar &#8594;</button>";
    conteudo += "</form>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Primeira Configura&ccedil;&atilde;o", conteudo));
}

// ============================================================
// PÁGINA: /setup-wifi
// ============================================================
void handleSetupWifi() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (existeWiFiConfig() && !logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String modo  = server.arg("modo");
        bool usarAP  = (modo == "ap");
        String tz    = server.arg("timezone");
        if (tz.length() == 0) tz = "BRT3";

        if (usarAP) {
            salvarWifi("", "", "", "", "", "", tz, true);
            adicionarLog("Modo AP salvo");
            String cont = "<div class='msg-ok'>&#10003; Configurado! Reiniciando em 5s...</div>";
            cont += "<div class='info'>Modo: <b>Ponto de Acesso</b><br>"
            "SSID: <b>RRFS Energia</b><br>IP: <b>10.0.0.1</b></div>";
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

        if (ssid.length() == 0) {
            msgHtml = "<div class='msg-err'>&#10007; Selecione uma rede</div>";
        } else if (ip.length() < 7 || gw.length() < 7 || mask.length() < 7 || dns.length() < 7) {
            msgHtml = "<div class='msg-err'>&#10007; Preencha todos os campos de rede</div>";
        } else {
            salvarWifi(ssid, wpass, ip, gw, mask, dns, tz, false);
            snprintf(tempBuf, sizeof(tempBuf), "WiFi salvo: %s IP:%s", ssid.c_str(), ip.c_str());
            adicionarLog(tempBuf);
            String cont = "<div class='msg-ok'>&#10003; Salvo! Reiniciando em 5s...</div>";
            cont += "<div class='info'>Rede: <b>" + ssid + "</b><br>"
            "IP: <b>" + ip + "</b><br>Acesse: <b>http://" + ip + "</b></div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - Configurado", cont));
            delay(5000); ESP.restart(); return;
        }
    }

    // Escaneia redes
    adicionarLog("Escaneando WiFi...");
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, false, false, 600);
    snprintf(tempBuf, sizeof(tempBuf), "Redes encontradas: %d", n);
    adicionarLog(tempBuf);

    String conteudo = "<div class='info'>&#9432; Configure a rede. IP Fixo obrigat&oacute;rio.</div>";
    conteudo += "<h3>&#128246; Configurar Rede</h3>";
    conteudo += msgHtml;
    conteudo += "<form method='POST' action='/setup-wifi'>";
    conteudo += "<label>Modo:</label>";
    conteudo += "<select name='modo' id='modoSel' onchange='toggleModo()'>";
    conteudo += "<option value='wifi'>Conectar ao WiFi</option>";
    conteudo += "<option value='ap'>Manter Ponto de Acesso (AP)</option>";
    conteudo += "</select>";
    conteudo += "<div id='divwifi'>";
    conteudo += "<label>Rede WiFi:</label>";
    conteudo += "<select name='ssid'>";
    if (n == 0) {
        conteudo += "<option value=''>Nenhuma rede encontrada</option>";
    } else {
        for (int i = 0; i < n; i++) {
            String s = WiFi.SSID(i);
            int r    = WiFi.RSSI(i);
            conteudo += "<option value='" + s + "'>" + s;
            if      (r > -50) conteudo += " [Excelente]";
            else if (r > -60) conteudo += " [Bom]";
            else if (r > -70) conteudo += " [Regular]";
            else              conteudo += " [Fraco]";
            snprintf(tempBuf, sizeof(tempBuf), " (%d dBm)", r);
            conteudo += String(tempBuf) + "</option>";
        }
    }
    WiFi.scanDelete();
    conteudo += "</select>";
    conteudo += "<button type='button' class='btn-orange' style='font-size:11px;margin-top:6px;' "
    "onclick='location.reload()'>&#128259; Atualizar Redes</button>";
    conteudo += "<label>Senha WiFi:</label>";
    conteudo += "<input type='password' name='wpass' maxlength='64' "
    "placeholder='Deixe vazio se rede aberta'>";
    conteudo += "<label>IP Fixo do ESP32:</label>";
    conteudo += "<input type='text' name='ip' required placeholder='Ex: 192.168.1.100'>";
    conteudo += "<label>Gateway:</label>";
    conteudo += "<input type='text' name='gw' required placeholder='Ex: 192.168.1.1'>";
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
    conteudo += "<option value='UTC0'>UTC (GMT+0)</option>";
    conteudo += "</select>";
    conteudo += "<button type='submit' class='btn-green'>&#128190; Salvar e Reiniciar</button>";
    conteudo += "</form>";
    if (logado) {
        conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
        "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";
    }
    conteudo += "<script>";
    conteudo += "function toggleModo(){";
    conteudo += "var m=document.getElementById('modoSel').value;";
    conteudo += "document.getElementById('divwifi').style.display=(m=='wifi'?'block':'none');}";
    conteudo += "</script>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Configurar Rede", conteudo));
}

// ============================================================
// PÁGINA: /login
// ============================================================
void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (logado) {
        server.sendHeader("Location", "/");
        server.send(302); return;
    }

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String user = server.arg("user");
        String pass = server.arg("pass");
        if (verificarLogin(user, pass)) {
            logado = true;
            snprintf(tempBuf, sizeof(tempBuf), "Login OK: %s", user.c_str());
            adicionarLog(tempBuf);
            logRAMSeNecessario();
            server.sendHeader("Location", "/");
            server.send(302); return;
        } else {
            snprintf(tempBuf, sizeof(tempBuf), "Login FALHOU: %s", user.c_str());
            adicionarLog(tempBuf);
            delay(1000); // Proteção brute force
            msgHtml = "<div class='msg-err'>&#10007; Usu&aacute;rio ou senha incorretos</div>";
        }
    }

    String conteudo = "<div class='info'>&#128274; Acesso seguro ao sistema RRFS Energia</div>";
    conteudo += "<h3>Entrar no Sistema</h3>";
    conteudo += msgHtml;
    conteudo += "<form method='POST' action='/login'>";
    conteudo += "<label>Usu&aacute;rio:</label>";
    conteudo += "<input type='text' name='user' required autocomplete='off' "
    "placeholder='Nome de usu&aacute;rio'>";
    conteudo += "<label>Senha:</label>";
    conteudo += "<input type='password' name='pass' required placeholder='Senha'>";
    conteudo += "<button type='submit'>&#128275; Entrar</button>";
    conteudo += "</form>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Login", conteudo));
}

// ============================================================
// PÁGINA: /inversores
// ============================================================
void handleInversores() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String msgHtml = "";

    // ---- Processa POST ----
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "salvar") {
            float pBaixa = server.arg("pot_baixa").toFloat();
            float pAlta  = server.arg("pot_alta").toFloat();
            float pChav  = server.arg("perc_chav").toFloat();

            if (pBaixa < 100.0f || pBaixa > 10000.0f) {
                msgHtml = "<div class='msg-err'>&#10007; Pot&ecirc;ncia Baixa inv&aacute;lida (100-10000W)</div>";
            } else if (pAlta < 500.0f || pAlta > 50000.0f) {
                msgHtml = "<div class='msg-err'>&#10007; Pot&ecirc;ncia Alta inv&aacute;lida (500-50000W)</div>";
            } else if (pAlta <= pBaixa) {
                msgHtml = "<div class='msg-err'>&#10007; Pot&ecirc;ncia Alta deve ser maior que Baixa</div>";
            } else if (pChav < 30.0f || pChav > 95.0f) {
                msgHtml = "<div class='msg-err'>&#10007; Percentual inv&aacute;lido (30-95%)</div>";
            } else {
                cfgInv.potenciaBaixa   = pBaixa;
                cfgInv.potenciaAlta    = pAlta;
                cfgInv.percChaveamento = pChav;
                cfgInv.configurado     = true;
                recalcularInversor();
                salvarConfigInversor();

                snprintf(tempBuf, sizeof(tempBuf),
                         "Inversores config: Baixa=%.0fW Alta=%.0fW Chav=%.0f%%",
                         pBaixa, pAlta, pChav);
                adicionarLog(tempBuf);
                snprintf(tempBuf, sizeof(tempBuf),
                         "Limites: Chav=%.1fA Ret=%.1fA (base %.1fA)",
                         cfgInv.limiteChaveamento,
                         cfgInv.limiteRetorno,
                         cfgInv.correnteBaixa);
                adicionarLog(tempBuf);

                msgHtml = "<div class='msg-ok'>&#10003; Inversores configurados!</div>";
            }

        } else if (acao == "modo_baixa") {
            // Força modo inversor baixa manualmente
            adicionarLog("Manual: forcando modo Inversor Baixa");
            setRedeAlta(false);
            delay(200);
            setInversorAlta(false);
            estadoInv.modoAlta   = false;
            estadoInv.emProtecao = false;
            maquinaEstado = ESTADO_BAIXA_NORMAL;
            estadoInv.tsUltimoChav = millis();
            msgHtml = "<div class='msg-ok'>&#10003; Modo Inversor Baixa ativado!</div>";

        } else if (acao == "modo_alta") {
            // Força modo inversor alta manualmente
            adicionarLog("Manual: forcando modo Inversor Alta");
            setInversorAlta(true);
            delay(500);
            setRedeAlta(true);
            estadoInv.modoAlta    = true;
            estadoInv.tsLigouAlta = millis();
            maquinaEstado = ESTADO_ALTA_NORMAL;
            estadoInv.tsUltimoChav = millis();
            msgHtml = "<div class='msg-ok'>&#10003; Modo Inversor Alta ativado!</div>";

        } else if (acao == "reset_stats") {
            estadoInv.totalChaveamentos  = 0;
            estadoInv.totalProtecoes     = 0;
            estadoInv.totalReiniciamentos= 0;
            salvarEstadoInversor();
            adicionarLog("Estatisticas dos inversores zeradas");
            msgHtml = "<div class='msg-ok'>&#10003; Estat&iacute;sticas zeradas!</div>";
        }
    }

    // ---- Monta página ----
    String conteudo = msgHtml;

    // ── DIAGRAMA DE STATUS ──
    conteudo += "<h3>&#9889; Diagrama do Sistema</h3>";
    conteudo += "<div id='diagrama' style='background:#000;border:1px solid #333;"
                "border-radius:8px;padding:12px;margin-bottom:10px;'>";
    // Diagrama inicial (atualizado por JS)
    conteudo += "<div style='text-align:center;color:#546e7a;font-size:11px;'>"
                "Carregando diagrama...</div>";
    conteudo += "</div>";

    // ── STATUS EM TEMPO REAL ──
    conteudo += "<h3>&#128202; Status em Tempo Real</h3>";
    conteudo += "<div class='stat-grid'>";

    conteudo += "<div class='stat-box'><div class='stat-label'>Estado Sistema</div>"
                "<div class='stat-val' id='estadoSis' style='font-size:12px;'>--</div>"
                "</div>";

    conteudo += "<div class='stat-box'><div class='stat-label'>Corrente Casa</div>"
                "<div class='stat-val stat-warn' id='iCasaInv'>--</div>"
                "<div class='stat-unit'>A</div></div>";

    conteudo += "<div class='stat-box'><div class='stat-label'>Uso do Inversor Baixa</div>"
                "<div class='stat-val' id='percUso'>--</div>"
                "<div class='stat-unit'>%</div></div>";

    conteudo += "<div class='stat-box'><div class='stat-label'>Modo Ativo</div>"
                "<div class='stat-val' id='modoAtivo'>--</div>"
                "</div>";
    conteudo += "</div>";

    // ── STATUS DOS RELÉS ──
    conteudo += "<h3>&#128290; Estado dos Rel&eacute;s</h3>";
    conteudo += "<div style='background:#111;border:1px solid #333;border-radius:6px;"
                "padding:10px;font-size:12px;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:5px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#127760; Rel&eacute; Rede Alta</span>"
                "<span id='rRedeAlta' style='font-weight:bold;'>--</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:5px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#128267; Rel&eacute; Inv. Baixa</span>"
                "<span id='rInvBaixa' style='font-weight:bold;'>--</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:5px 0;'>"
                "<span style='color:#78909c;'>&#9889; Rel&eacute; Inv. Alta</span>"
                "<span id='rInvAlta' style='font-weight:bold;'>--</span></div>";
    conteudo += "</div>";

    // ── STATUS DOS INVERSORES ──
    conteudo += "<h3>&#9881; Status dos Inversores</h3>";
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>&#128267; Inv. Baixa</div>"
                "<div class='stat-val' id='stBaixaBox'>--</div>"
                "<div class='stat-unit' id='stBaixaUnit'>--</div></div>";
    conteudo += "<div class='stat-box'>"
                "<div class='stat-label'>&#9889; Inv. Alta</div>"
                "<div class='stat-val' id='stAltaBox'>--</div>"
                "<div class='stat-unit' id='stAltaUnit'>--</div></div>";
    conteudo += "</div>";

    // ── LIMITES ──
    if (cfgInv.configurado) {
        conteudo += "<h3>&#128200; Limites Calculados</h3>";
        conteudo += "<div class='info' style='font-size:11px;'>";

        snprintf(tempBuf, sizeof(tempBuf), "%.1f", cfgInv.correnteBaixa);
        conteudo += "&#128267; Inv. Baixa: <b>" + String(cfgInv.potenciaBaixa, 0)
                  + "W</b> &divide; " + String(cfgBat.tensaoNominal, 1)
                  + "V = <b>" + String(tempBuf) + "A m&aacute;x.</b><br>";

        snprintf(tempBuf, sizeof(tempBuf), "%.1f", cfgInv.correnteAlta);
        conteudo += "&#9889; Inv. Alta: <b>" + String(cfgInv.potenciaAlta, 0)
                  + "W</b> &divide; " + String(cfgBat.tensaoNominal, 1)
                  + "V = <b>" + String(tempBuf) + "A m&aacute;x.</b><br>";

        snprintf(tempBuf, sizeof(tempBuf), "%.1f", cfgInv.limiteChaveamento);
        conteudo += "&#9650; Liga Alta quando: <b>&gt; " + String(tempBuf)
                  + "A</b> (" + String(cfgInv.percChaveamento, 0) + "% de "
                  + String(cfgInv.correnteBaixa, 1) + "A)<br>";

        snprintf(tempBuf, sizeof(tempBuf), "%.1f", cfgInv.limiteRetorno);
        conteudo += "&#9660; Retorna Baixa quando: <b>&lt; " + String(tempBuf)
                  + "A</b> (ap&oacute;s 1 minuto)<br>";

        conteudo += "&#9889; Histerese: <b>" + String(HISTERESE_PERC, 0)
                  + "%</b> (evita chaveamento r&aacute;pido)";
        conteudo += "</div>";
    }

    // ── BARRA DE CARGA ──
    conteudo += "<div style='margin:8px 0;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-bottom:3px;'>";
    conteudo += "<span>Carga do Inversor Baixa</span>"
                "<span id='percUsoLabel'>--%</span></div>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:4px;height:14px;position:relative;'>";
    // Linha do limite
    if (cfgInv.configurado) {
        conteudo += "<div style='position:absolute;top:0;height:100%;width:2px;"
                    "background:#ff5722;left:" + String(cfgInv.percChaveamento, 0)
                  + "%;opacity:0.8;'></div>";
        conteudo += "<div style='position:absolute;top:-12px;font-size:8px;"
                    "color:#ff5722;left:" + String(cfgInv.percChaveamento, 0)
                  + "%;'>" + String(cfgInv.percChaveamento, 0) + "%</div>";
    }
    conteudo += "<div id='barraUso' style='height:100%;border-radius:4px;"
                "width:0%;background:#4fc3f7;transition:width 0.8s;'></div>";
    conteudo += "</div></div>";

    // ── ESTATÍSTICAS ──
    conteudo += "<h3>&#128202; Estat&iacute;sticas</h3>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:10px;font-size:11px;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#8646; Total Chaveamentos</span>"
                "<span class='contador' id='totalChav'>0</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#9888; Total Prote&ccedil;&otilde;es</span>"
                "<span class='contador' id='totalProt' style='color:#ef5350;'>0</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;'>"
                "<span style='color:#78909c;'>&#8635; Total Reiniciamentos</span>"
                "<span class='contador' id='totalRein' style='color:#ffa726;'>0</span></div>";
    conteudo += "</div>";

    // Tempo no modo atual
    conteudo += "<div class='info' style='font-size:11px;margin-top:8px;'>";
    conteudo += "&#9201; No modo atual: <b id='tempoModo'>--</b> &nbsp;|&nbsp; ";
    conteudo += "&#9660; Pode retornar em: <b id='tempoRetorno'>--</b></div>";

    // ── CONTROLE MANUAL ──
    conteudo += "<h3>&#9881; Controle Manual</h3>";
    conteudo += "<div class='msg-warn' style='font-size:11px;'>";
    conteudo += "&#9888; Controle manual substitui a automa&ccedil;&atilde;o temporariamente.</div>";
    conteudo += "<div class='flex'>";
    conteudo += "<form method='POST' action='/inversores' style='flex:1'>"
                "<input type='hidden' name='acao' value='modo_baixa'>"
                "<button type='submit' class='btn-green' style='font-size:11px;'>"
                "&#128267; Forcar Baixa</button></form>";
    conteudo += "<form method='POST' action='/inversores' style='flex:1'>"
                "<input type='hidden' name='acao' value='modo_alta'>"
                "<button type='submit' class='btn-orange' style='font-size:11px;'>"
                "&#9889; Forcar Alta</button></form>";
    conteudo += "<form method='POST' action='/inversores' style='flex:1'>"
                "<input type='hidden' name='acao' value='reset_stats'>"
                "<button type='submit' class='btn-gray' style='font-size:11px;'>"
                "&#8635; Zerar Stats</button></form>";
    conteudo += "</div>";

    // ── FORMULÁRIO DE CONFIGURAÇÃO ──
    conteudo += "<h3>&#9881; Configurar Inversores</h3>";
    conteudo += "<form method='POST' action='/inversores'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";

    conteudo += "<label>Pot&ecirc;ncia Inversor Baixa (W) - mais eficiente:</label>";
    conteudo += "<input type='number' name='pot_baixa' min='100' max='10000' "
                "step='50' required value='"
                + String((int)cfgInv.potenciaBaixa)
                + "' placeholder='Ex: 500'>";

    conteudo += "<label>Pot&ecirc;ncia Inversor Alta (W) - mais potente:</label>";
    conteudo += "<input type='number' name='pot_alta' min='500' max='50000' "
                "step='100' required value='"
                + String((int)cfgInv.potenciaAlta)
                + "' placeholder='Ex: 3000'>";

    conteudo += "<label>Percentual para chavear para Alta (%) - padr&atilde;o 70%:</label>";
    conteudo += "<input type='number' name='perc_chav' min='30' max='95' "
                "step='5' required value='"
                + String((int)cfgInv.percChaveamento)
                + "' placeholder='Ex: 70'>";

    // Preview dos cálculos
    if (cfgBat.tensaoNominal > 0 && cfgInv.potenciaBaixa > 0) {
        conteudo += "<div class='info' style='margin-top:8px;font-size:11px;'>";
        conteudo += "&#9432; C&aacute;lculo atual com tens&atilde;o "
                  + String(cfgBat.tensaoNominal, 1) + "V:<br>";
        snprintf(tempBuf, sizeof(tempBuf),
                 "Inv. Baixa: %.0f &divide; %.1f = <b>%.1fA</b><br>",
                 cfgInv.potenciaBaixa, cfgBat.tensaoNominal,
                 cfgInv.correnteBaixa);
        conteudo += String(tempBuf);
        snprintf(tempBuf, sizeof(tempBuf),
                 "Inv. Alta: %.0f &divide; %.1f = <b>%.1fA</b><br>",
                 cfgInv.potenciaAlta, cfgBat.tensaoNominal,
                 cfgInv.correnteAlta);
        conteudo += String(tempBuf);
        snprintf(tempBuf, sizeof(tempBuf),
                 "Liga Alta em: <b>%.1fA</b> | Retorna em: <b>%.1fA</b>",
                 cfgInv.limiteChaveamento, cfgInv.limiteRetorno);
        conteudo += String(tempBuf);
        conteudo += "</div>";
    }

    conteudo += "<button type='submit' class='btn-green'>"
                "&#128190; Salvar Configura&ccedil;&atilde;o</button>";
    conteudo += "</form>";
    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";

    // ── SCRIPT TEMPO REAL ──
    String script = "";
    script += "var iLimite=" + String(cfgInv.limiteChaveamento, 1) + ";";
    script += "var corMax=" + String(cfgInv.correnteBaixa, 1) + ";";
    script += F("function fmt2(v){return isNaN(v)?'--':v.toFixed(1);}"
                "function fmtTempo(s){"
                "var h=Math.floor(s/3600);"
                "var m=Math.floor((s%3600)/60);"
                "var sec=s%60;"
                "if(h>0)return h+'h '+pad(m)+'m';"
                "if(m>0)return m+'m '+pad(sec)+'s';"
                "return sec+'s';}"
                "function pad(n){return n<10?'0'+n:n;}");

    // Diagrama SVG gerado por JavaScript
    script += F("function gerarDiagrama(d){"
                "var mA=d.modoAlta;"
                "var stB=d.stBaixa;"
                "var stA=d.stAlta;"
                "var rRA=d.rRedeAlta;"
                "var cok='#66bb6a';"
                "var cerr='#ef5350';"
                "var coff='#546e7a';"
                "var cwire='#ffa726';"
                "var html='<svg viewBox=\"0 0 320 160\" style=\"width:100%;font-family:Arial;\">';"
                // Painel Solar
                "html+='<rect x=\"5\" y=\"5\" width=\"70\" height=\"35\" rx=\"4\" "
                "fill=\"#1a3a5c\" stroke=\"#1565c0\" stroke-width=\"1\"/>';"
                "html+='<text x=\"40\" y=\"20\" fill=\"#4fc3f7\" font-size=\"9\" "
                "text-anchor=\"middle\">PAINEL</text>';"
                "html+='<text x=\"40\" y=\"32\" fill=\"#81d4fa\" font-size=\"8\" "
                "text-anchor=\"middle\">SOLAR</text>';"
                // Bateria
                "html+='<rect x=\"5\" y=\"60\" width=\"70\" height=\"35\" rx=\"4\" "
                "fill=\"#1a3a1c\" stroke=\"#2e7d32\" stroke-width=\"1\"/>';"
                "html+='<text x=\"40\" y=\"75\" fill=\"#66bb6a\" font-size=\"9\" "
                "text-anchor=\"middle\">BATERIA</text>';"
                "html+='<text x=\"40\" y=\"87\" fill=\"#a5d6a7\" font-size=\"8\" "
                "text-anchor=\"middle\">LiFePO4</text>';"
                // Inversor Baixa
                "var cB=stB?cok:cerr;"
                "html+='<rect x=\"110\" y=\"5\" width=\"80\" height=\"40\" rx=\"4\" "
                "fill=\"#111\" stroke=\"'+cB+'\" stroke-width=\"2\"/>';"
                "html+='<text x=\"150\" y=\"18\" fill=\"'+cB+'\" font-size=\"8\" "
                "text-anchor=\"middle\">INV. BAIXA</text>';"
                "html+='<text x=\"150\" y=\"30\" fill=\"'+cB+'\" font-size=\"9\" "
                "font-weight=\"bold\" text-anchor=\"middle\">"
                "'+(!mA&&stB?'✓ OK':'◉ '+(mA?'STANDBY':'ERRO'))+'</text>';"
                "html+='<text x=\"150\" y=\"40\" fill=\"#546e7a\" font-size=\"7\" "
                "text-anchor=\"middle\">'+d.pBaixa+'W / '+d.iBaixa.toFixed(0)+'A</text>';"
                // Inversor Alta
                "var cA=stA?cok:(mA?cerr:coff);"
                "html+='<rect x=\"110\" y=\"65\" width=\"80\" height=\"40\" rx=\"4\" "
                "fill=\"#111\" stroke=\"'+cA+'\" stroke-width=\"2\"/>';"
                "html+='<text x=\"150\" y=\"78\" fill=\"'+cA+'\" font-size=\"8\" "
                "text-anchor=\"middle\">INV. ALTA</text>';"
                "html+='<text x=\"150\" y=\"90\" fill=\"'+cA+'\" font-size=\"9\" "
                "font-weight=\"bold\" text-anchor=\"middle\">"
                "'+( mA&&stA?'✓ OK':(!mA?'⊘ DESL.':'✗ ERRO'))+'</text>';"
                "html+='<text x=\"150\" y=\"100\" fill=\"#546e7a\" font-size=\"7\" "
                "text-anchor=\"middle\">'+d.pAlta+'W / '+d.iAlta.toFixed(0)+'A</text>';"
                // Rede Baixa (sempre ligada ao inv. baixa)
                "html+='<rect x=\"240\" y=\"5\" width=\"75\" height=\"30\" rx=\"4\" "
                "fill=\"#1a2a1a\" stroke=\"#388e3c\" stroke-width=\"1\"/>';"
                "html+='<text x=\"277\" y=\"18\" fill=\"#81c784\" font-size=\"8\" "
                "text-anchor=\"middle\">REDE BAIXA</text>';"
                "html+='<text x=\"277\" y=\"28\" fill=\"#66bb6a\" font-size=\"7\" "
                "text-anchor=\"middle\">Sempre Ligada</text>';"
                // Rede Alta (comutada)
                "var cRA=rRA?cok:coff;"
                "html+='<rect x=\"240\" y=\"65\" width=\"75\" height=\"30\" rx=\"4\" "
                "fill=\"#111\" stroke=\"'+cRA+'\" stroke-width=\"1\"/>';"
                "html+='<text x=\"277\" y=\"78\" fill=\"'+cRA+'\" font-size=\"8\" "
                "text-anchor=\"middle\">REDE ALTA</text>';"
                "html+='<text x=\"277\" y=\"88\" fill=\"'+cRA+'\" font-size=\"7\" "
                "text-anchor=\"middle\">'+(rRA?'✓ Inv.Alta':'⊘ Inv.Baixa')+'</text>';"
                // Casa
                "html+='<rect x=\"240\" y=\"120\" width=\"75\" height=\"32\" rx=\"4\" "
                "fill=\"#1a1a2a\" stroke=\"#1565c0\" stroke-width=\"1\"/>';"
                "html+='<text x=\"277\" y=\"133\" fill=\"#90caf9\" font-size=\"8\" "
                "text-anchor=\"middle\">CASA</text>';"
                "html+='<text x=\"277\" y=\"145\" fill=\"#64b5f6\" font-size=\"8\" "
                "text-anchor=\"middle\">'+d.iCasa.toFixed(1)+'A</text>';"
                // Fios
                // Bateria → Inv Baixa
                "html+='<line x1=\"75\" y1=\"25\" x2=\"110\" y2=\"25\" "
                "stroke=\"#1565c0\" stroke-width=\"2\"/>';"
                // Bateria → Inv Alta
                "html+='<line x1=\"75\" y1=\"77\" x2=\"110\" y2=\"85\" "
                "stroke=\"'+(mA?'#1565c0':'#333')+'\" stroke-width=\"2\"/>';"
                // Inv Baixa → Rede Baixa
                "html+='<line x1=\"190\" y1=\"20\" x2=\"240\" y2=\"20\" "
                "stroke=\"#388e3c\" stroke-width=\"2\"/>';"
                // Inv Baixa/Alta → Rede Alta
                "var xFrom=mA?190:190;"
                "var yFrom=mA?85:20;"
                "html+='<line x1=\"'+xFrom+'\" y1=\"'+yFrom+'\" x2=\"240\" y2=\"80\" "
                "stroke=\"'+(rRA?cwire:'#333')+'\" stroke-width=\"2\"/>';"
                // Rede Baixa → Casa
                "html+='<line x1=\"277\" y1=\"35\" x2=\"277\" y2=\"120\" "
                "stroke=\"#388e3c\" stroke-width=\"1.5\" stroke-dasharray=\"3\"/>';"
                // Rede Alta → Casa
                "html+='<line x1=\"277\" y1=\"95\" x2=\"277\" y2=\"120\" "
                "stroke=\"'+(rRA?cwire:'#333')+'\" stroke-width=\"1.5\" "
                "stroke-dasharray=\"3\"/>';"
                "html+='</svg>';"
                "document.getElementById('diagrama').innerHTML=html;}"

                "function atualizarInversores(){"
                "var x=new XMLHttpRequest();"
                "x.onreadystatechange=function(){"
                "if(x.readyState==4&&x.status==200){"
                "try{var d=JSON.parse(x.responseText);"
                // Estado sistema
                "var eEl=document.getElementById('estadoSis');"
                "eEl.textContent=d.estado;"
                "eEl.style.color=d.emProtecao?'#ef5350':(d.modoAlta?'#ffa726':'#66bb6a');"
                // Corrente casa
                "document.getElementById('iCasaInv').textContent=d.iCasa.toFixed(2);"
                // Percentual uso
                "var pu=d.percUso;"
                "var puEl=document.getElementById('percUso');"
                "puEl.textContent=pu.toFixed(1);"
                "puEl.className='stat-val '+(pu>d.percChav?'stat-err':(pu>d.percChav*0.8?'stat-warn':'stat-ok'));"
                "document.getElementById('percUsoLabel').textContent=pu.toFixed(1)+'%';"
                // Barra de uso
                "var bar=document.getElementById('barraUso');"
                "bar.style.width=Math.min(pu,100)+'%';"
                "bar.style.background=pu>d.percChav?'#ef5350':(pu>d.percChav*0.8?'#ffa726':'#4fc3f7');"
                // Modo ativo
                "var mEl=document.getElementById('modoAtivo');"
                "mEl.textContent=d.modoAlta?'⚡ ALTA':'🔋 BAIXA';"
                "mEl.className='stat-val '+(d.modoAlta?'stat-warn':'stat-ok');"
                // Relés
                "var r1=document.getElementById('rRedeAlta');"
                "r1.textContent=d.rRedeAlta?'✓ INV.ALTA':'○ INV.BAIXA';"
                "r1.style.color=d.rRedeAlta?'#ffa726':'#66bb6a';"
                "var r2=document.getElementById('rInvBaixa');"
                "r2.textContent=d.rInvBaixa?'○ DESLIGADO':'✓ LIGADO';"
                "r2.style.color=d.rInvBaixa?'#ef5350':'#66bb6a';"
                "var r3=document.getElementById('rInvAlta');"
                "r3.textContent=d.rInvAlta?'✓ LIGADO':'○ DESLIGADO';"
                "r3.style.color=d.rInvAlta?'#ffa726':'#546e7a';"
                // Status inversores
                "var sb=document.getElementById('stBaixaBox');"
                "var sbu=document.getElementById('stBaixaUnit');"
                "sb.textContent=d.stBaixa?'✓ OK':'✗ ERRO';"
                "sb.className='stat-val '+(d.stBaixa?'stat-ok':'stat-err');"
                "sbu.textContent=d.stBaixa?'Funcionando':'Sem sinal';"
                "var sa=document.getElementById('stAltaBox');"
                "var sau=document.getElementById('stAltaUnit');"
                "sa.textContent=d.modoAlta?(d.stAlta?'✓ OK':'✗ ERRO'):'⊘ Desl.';"
                "sa.className='stat-val '+(d.modoAlta?(d.stAlta?'stat-ok':'stat-err'):'stat-warn');"
                "sau.textContent=d.modoAlta?(d.stAlta?'Funcionando':'Sem sinal'):'Desligado';"
                // Estatísticas
                "document.getElementById('totalChav').textContent=d.totalChav;"
                "document.getElementById('totalProt').textContent=d.totalProt;"
                "document.getElementById('totalRein').textContent=d.totalRein;"
                // Tempo
                "document.getElementById('tempoModo').textContent="
                "d.tempoModo>0?fmtTempo(d.tempoModo):'--';"
                "document.getElementById('tempoRetorno').textContent="
                "d.tempoRetorno>0?fmtTempo(d.tempoRetorno):'Pronto';"
                // Diagrama
                "gerarDiagrama(d);"
                "}catch(e){}"
                "}};"
                "x.open('GET','/api/inversores',true);x.send();}"
                "setInterval(atualizarInversores,1500);"
                "atualizarInversores();");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Inversores", conteudo, script));
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
            float pIB = server.arg("perc_invbaixa").toFloat();
            float pRA = server.arg("perc_redealta").toFloat();
            float pLE = server.arg("perc_lampext").toFloat();
            float pLI = server.arg("perc_lampint").toFloat();
            uint8_t mLE = (uint8_t)server.arg("modo_lampext").toInt();
            uint8_t lH  = (uint8_t)server.arg("ligar_hora").toInt();
            uint8_t lM  = (uint8_t)server.arg("ligar_min").toInt();
            uint8_t dH  = (uint8_t)server.arg("desl_hora").toInt();
            uint8_t dM  = (uint8_t)server.arg("desl_min").toInt();

            bool ok = true;
            if (pIB < 0 || pIB > 50)
                { msgHtml="<div class='msg-err'>&#10007; Inv.Baixa: 0-50%</div>"; ok=false; }
            else if (pRA < 0 || pRA > 80)
                { msgHtml="<div class='msg-err'>&#10007; Rede Alta: 0-80%</div>"; ok=false; }
            else if (pLE < 0 || pLE > 80)
                { msgHtml="<div class='msg-err'>&#10007; Lamp.Ext: 0-80%</div>"; ok=false; }
            else if (pLI < 0 || pLI > 50)
                { msgHtml="<div class='msg-err'>&#10007; Lamp.Int: 0-50%</div>"; ok=false; }
            else if (pRA < pLE || pRA < pLI)
                { msgHtml="<div class='msg-err'>&#10007; Rede Alta deve ser &gt;"
                  " Lamp.Ext e Lamp.Int</div>"; ok=false; }
            else if (pLE < pLI)
                { msgHtml="<div class='msg-err'>&#10007; Lamp.Ext deve ser &gt;"
                  " Lamp.Int</div>"; ok=false; }
            else if (mLE == 1 && !ntpSincronizado)
                { msgHtml="<div class='msg-err'>&#10007; Modo Hor&aacute;rio requer"
                  " NTP sincronizado</div>"; ok=false; }

            if (ok) {
                cfgRede.percInvBaixa  = pIB;
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
                         "Rede: IB=%.0f%% RA=%.0f%% LE=%.0f%% LI=%.0f%% "
                         "ModoLamp=%d Lig=%02d:%02d Desl=%02d:%02d",
                         pIB, pRA, pLE, pLI, mLE, lH, lM, dH, dM);
                adicionarLog(tempBuf);
                msgHtml = "<div class='msg-ok'>&#10003; Rede configurada!</div>";
            }

        } else if (acao == "reset_stats") {
            estadoRede.totalCortesInvBaixa = 0;
            estadoRede.totalCortesRedeAlta = 0;
            estadoRede.totalCortesLampExt  = 0;
            estadoRede.totalCortesLampInt  = 0;
            estadoRede.totalLigadasHorario = 0;
            estadoRede.totalLigadasPainel  = 0;
            salvarEstadoRede();
            adicionarLog("Stats rede zeradas");
            msgHtml = "<div class='msg-ok'>&#10003; Zerado!</div>";

        } else if (acao == "ligar_tudo") {
            setLampadaExterna(true);
            setLampadaInterna(true);
            setInversorBaixa(true);
            estadoRede.invBaixaAtivo      = true;
            estadoRede.redeAltaAtiva      = true;
            estadoRede.lampExternaAtiva   = true;
            estadoRede.lampInternaAtiva   = true;
            estadoRede.lampExtBloqEnergia = false;
            estadoRede.aguardandoInvBaixa = false;
            estadoRede.aguardandoRedeAlta = false;
            estadoRede.aguardandoLampExt  = false;
            estadoRede.aguardandoLampInt  = false;
            adicionarLog("Manual: tudo LIGADO");
            msgHtml = "<div class='msg-ok'>&#10003; Tudo ligado!</div>";

        } else if (acao == "toggle_lampext") {
            bool novoEstado = !estadoRede.lampExternaAtiva;
            setLampadaExterna(novoEstado);
            estadoRede.lampExtBloqEnergia = false;
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual: lamp.ext %s",
                     novoEstado ? "LIGADA" : "DESLIGADA");
            adicionarLog(tempBuf);
            msgHtml = novoEstado
                ? "<div class='msg-ok'>&#10003; Lamp.Externa LIGADA!</div>"
                : "<div class='msg-ok'>&#10003; Lamp.Externa DESLIGADA!</div>";

        } else if (acao == "toggle_lampint") {
            bool novoEstado = !estadoRede.lampInternaAtiva;
            setLampadaInterna(novoEstado);
            snprintf(tempBuf, sizeof(tempBuf),
                     "Manual: lamp.int %s",
                     novoEstado ? "LIGADA" : "DESLIGADA");
            adicionarLog(tempBuf);
            msgHtml = novoEstado
                ? "<div class='msg-ok'>&#10003; Lamp.Interna LIGADA!</div>"
                : "<div class='msg-ok'>&#10003; Lamp.Interna DESLIGADA!</div>";
        }
    }

    float whDisp = estadoBat.ahDisponiveis * sensores.tensaoBateriaEst;
    float soc = 0;
    if (estadoBat.ahMaximos > 0)
        soc = (estadoBat.ahDisponiveis / estadoBat.ahMaximos) * 100.0f;
    soc = constrain(soc, 0.0f, 100.0f);
    float eD = cfgBat.energiaDisponivel;

    String conteudo = msgHtml;

    // ── BARRA DE ENERGIA ──
    conteudo += "<h3>&#9889; Status da Rede El&eacute;trica</h3>";
    conteudo += "<div style='margin:8px 0;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-bottom:3px;'>";
    conteudo += "<span>Energia Dispon&iacute;vel</span>";
    conteudo += "<span id='whDispLabel'>-- Wh (--%)</span></div>";
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:4px;height:20px;position:relative;'>";

    // Marcadores na barra
    if (eD > 0) {
        struct { float p; const char* cor; } marcas[] = {
            { cfgRede.percLampInt,  "#ef5350" },
            { cfgRede.percLampExt,  "#ff9800" },
            { cfgRede.percRedeAlta, "#ffc107" },
            { cfgRede.percInvBaixa, "#f44336" },
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

    // ── CARDS DE STATUS ──
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label'>"
                "&#128267; Inv. Baixa</div>"
                "<div class='stat-val' id='stIBbox'>--</div>"
                "<div class='stat-unit' id='stIBunit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>"
                "&#9889; Rede Alta</div>"
                "<div class='stat-val' id='stRAbox'>--</div>"
                "<div class='stat-unit' id='stRAunit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>"
                "&#128161; Lamp. Externa</div>"
                "<div class='stat-val' id='stLEbox'>--</div>"
                "<div class='stat-unit' id='stLEunit'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>"
                "&#128161; Lamp. Interna</div>"
                "<div class='stat-val' id='stLIbox'>--</div>"
                "<div class='stat-unit' id='stLIunit'>--</div></div>";
    conteudo += "</div>";

    // Status lâmpada externa detalhado
    conteudo += "<div id='infoLampExt' style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:8px;font-size:11px;margin:6px 0;'>"
                "Carregando...</div>";

    // Alertas de contagem
    conteudo += "<div id='alertasRede'></div>";

    // ── TABELA DE NÍVEIS ──
    conteudo += "<h3>&#128200; N&iacute;veis de Controle</h3>";
    conteudo += "<div style='overflow-x:auto;'>";
    conteudo += "<table style='width:100%;border-collapse:collapse;font-size:10px;'>";
    conteudo += "<thead><tr style='background:#1e3a5f;color:#90caf9;'>";
    conteudo += "<th style='padding:5px;text-align:left;'>Carga</th>";
    conteudo += "<th style='padding:5px;text-align:center;'>Corte</th>";
    conteudo += "<th style='padding:5px;text-align:center;'>Wh M&iacute;n</th>";
    conteudo += "<th style='padding:5px;text-align:center;'>Cortes</th>";
    conteudo += "</tr></thead><tbody>";

    struct { const char* n; const char* ic; float p; float w;
             uint32_t c; const char* cor; } niveis[] = {
        {"Inv.Baixa","&#128267;", cfgRede.percInvBaixa,
         eD*(cfgRede.percInvBaixa/100.0f),
         estadoRede.totalCortesInvBaixa,"#ef5350"},
        {"Rede Alta","&#9889;",   cfgRede.percRedeAlta,
         eD*(cfgRede.percRedeAlta/100.0f),
         estadoRede.totalCortesRedeAlta,"#ffc107"},
        {"Lamp.Ext", "&#128161;", cfgRede.percLampExt,
         eD*(cfgRede.percLampExt/100.0f),
         estadoRede.totalCortesLampExt, "#ff9800"},
        {"Lamp.Int", "&#128161;", cfgRede.percLampInt,
         eD*(cfgRede.percLampInt/100.0f),
         estadoRede.totalCortesLampInt, "#4fc3f7"},
    };

    for (int i = 0; i < 4; i++) {
        const char* bg = (i%2==0)?"#0d0d0d":"#111";
        conteudo += "<tr style='background:" + String(bg) + ";'>";
        conteudo += "<td style='padding:5px;color:#90caf9;'>"
                  + String(niveis[i].ic) + " "
                  + String(niveis[i].n) + "</td>";
        snprintf(tempBuf,sizeof(tempBuf),"%.0f%%",niveis[i].p);
        conteudo += "<td style='padding:5px;text-align:center;color:"
                  + String(niveis[i].cor)
                  + ";font-weight:bold;'>" + String(tempBuf) + "</td>";
        snprintf(tempBuf,sizeof(tempBuf),"%.0f Wh",niveis[i].w);
        conteudo += "<td style='padding:5px;text-align:center;"
                    "color:#78909c;'>" + String(tempBuf) + "</td>";
        snprintf(tempBuf,sizeof(tempBuf),"%lu",(unsigned long)niveis[i].c);
        conteudo += "<td style='padding:5px;text-align:center;"
                    "color:#546e7a;'>" + String(tempBuf) + "</td>";
        conteudo += "</tr>";
    }
    conteudo += "</tbody></table></div>";

    // Info sistema
    conteudo += "<div class='info' style='font-size:11px;margin-top:6px;'>";
    conteudo += "&#128267; Base: <b>" + String(eD,0) + " Wh</b>"
                " &nbsp;|&nbsp; Histerese: <b>+2%</b>"
                " &nbsp;|&nbsp; Confirma&ccedil;&atilde;o: <b>1 min</b><br>";
    conteudo += "&#127774; Painel: <span id='painelInfo'>--</span>"
                " &nbsp;|&nbsp; &#128336; Hora: <span id='horaInfo'>--</span>";
    conteudo += "</div>";

    // ── CONTROLE MANUAL ──
    conteudo += "<h3>&#9881; Controle Manual</h3>";
    conteudo += "<div class='flex'>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='ligar_tudo'>"
                "<button type='submit' class='btn-green' style='font-size:10px;'>"
                "&#9889; Ligar Tudo</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='toggle_lampext'>"
                "<button type='submit' class='btn-orange' style='font-size:10px;'>"
                "&#128161; Tog.Ext</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='toggle_lampint'>"
                "<button type='submit' class='btn-orange' style='font-size:10px;'>"
                "&#128161; Tog.Int</button></form>";
    conteudo += "<form method='POST' action='/rede' style='flex:1'>"
                "<input type='hidden' name='acao' value='reset_stats'>"
                "<button type='submit' class='btn-gray' style='font-size:10px;'>"
                "&#8635; Zerar</button></form>";
    conteudo += "</div>";

    // ── FORMULÁRIO CONFIGURAÇÃO ──
    conteudo += "<h3>&#9881; Configurar N&iacute;veis de Corte</h3>";
    conteudo += "<div class='msg-warn' style='font-size:10px;'>"
                "&#9432; Base: Energia Dispon&iacute;vel = <b>"
                + String(eD,0) + " Wh</b>. "
                "Ordem: Rede Alta &gt; Lamp.Ext &gt; Lamp.Int.</div>";

    conteudo += "<form method='POST' action='/rede'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";

    // Linhas de corte
    struct { const char* lbl; const char* nm;
             float val; float wh; const char* cor; const char* desc; } campos[] = {
        {"&#128267; Inv.Baixa - Desligar rede baixa (%)",
         "perc_invbaixa", cfgRede.percInvBaixa,
         eD*(cfgRede.percInvBaixa/100.0f), "#ef5350",
         "Desliga toda rede baixa. Corte mais critico. (padrao: 10%)"},
        {"&#9889; Rede Alta - Desligar tomadas (%)",
         "perc_redealta", cfgRede.percRedeAlta,
         eD*(cfgRede.percRedeAlta/100.0f), "#ffc107",
         "Tomadas alta potencia sem energia. (padrao: 30%)"},
        {"&#128161; Lamp.Externa - Corte por energia (%)",
         "perc_lampext",  cfgRede.percLampExt,
         eD*(cfgRede.percLampExt/100.0f),  "#ff9800",
         "Energia minima para manter lamp.externa. (padrao: 25%)"},
        {"&#128161; Lamp.Interna - Corte por energia (%)",
         "perc_lampint",  cfgRede.percLampInt,
         eD*(cfgRede.percLampInt/100.0f),  "#4fc3f7",
         "0% = usa apenas protecao da bateria. (padrao: 0%)"},
    };

    for (int i = 0; i < 4; i++) {
        conteudo += "<label>" + String(campos[i].lbl) + ":</label>";
        conteudo += "<div style='display:flex;gap:8px;align-items:center;'>";
        conteudo += "<input type='number' name='" + String(campos[i].nm)
                  + "' min='0' max='80' step='1' required value='"
                  + String((int)campos[i].val)
                  + "' style='flex:1;' oninput='calcWh(this,"
                  + String(eD,0) + ")'>";
        conteudo += "<span style='color:" + String(campos[i].cor)
                  + ";font-size:11px;min-width:70px;' id='wh_"
                  + String(campos[i].nm) + "'>";
        snprintf(tempBuf,sizeof(tempBuf),"%.0f Wh",campos[i].wh);
        conteudo += String(tempBuf) + "</span></div>";
        conteudo += "<div style='color:#546e7a;font-size:9px;margin-top:1px;'>"
                  + String(campos[i].desc) + "</div>";
    }

    // ── SEÇÃO LÂMPADA EXTERNA - HORÁRIO/PAINEL ──
    conteudo += "<h3>&#128161; Controle Lamp. Externa</h3>";
    conteudo += "<div class='info' style='font-size:10px;'>";
    conteudo += "&#9432; O corte por energia (acima) tem PRIORIDADE sobre"
                " qualquer modo de controle abaixo.</div>";

    conteudo += "<label>Modo de Controle:</label>";
    conteudo += "<select name='modo_lampext' id='modoLampSel' "
                "onchange='toggleModoLamp()'>";
    conteudo += "<option value='0'" + String(cfgRede.modoLampExt==0?" selected":"")
              + ">&#9889; Apenas controle por energia (sem horario)</option>";
    conteudo += "<option value='1'" + String(cfgRede.modoLampExt==1?" selected":"")
              + ">&#128336; Horario Manual (liga/desliga programado)</option>";
    conteudo += "<option value='2'" + String(cfgRede.modoLampExt==2?" selected":"")
              + ">&#127774; Automatico por Painel Solar</option>";
    conteudo += "</select>";

    // Div horário (visível apenas no modo 1)
    conteudo += "<div id='divHorario' style='display:"
              + String(cfgRede.modoLampExt==1?"block":"none") + ";'>";
    conteudo += "<div class='info' style='font-size:10px;'>";
    conteudo += "&#128336; Configure o hor&aacute;rio de ligar e desligar a "
                "l&acirc;mpada externa.<br>";
    conteudo += "Exemplo: Liga 18:00 e Desliga 06:00 = ligada durante a noite.</div>";

    conteudo += "<div style='display:grid;grid-template-columns:1fr 1fr;gap:8px;"
                "margin-top:8px;'>";
    conteudo += "<div><label style='color:#66bb6a;'>&#9650; Ligar &agrave;s:</label>";
    conteudo += "<div style='display:flex;gap:4px;'>";
    conteudo += "<input type='number' name='ligar_hora' min='0' max='23' "
                "style='width:50%;' value='" + String(cfgRede.ligarHora) + "' "
                "placeholder='HH'>";
    conteudo += "<input type='number' name='ligar_min'  min='0' max='59' "
                "style='width:50%;' value='" + String(cfgRede.ligarMin)  + "' "
                "placeholder='MM'>";
    conteudo += "</div></div>";
    conteudo += "<div><label style='color:#ef5350;'>&#9660; Desligar &agrave;s:"
                "</label>";
    conteudo += "<div style='display:flex;gap:4px;'>";
    conteudo += "<input type='number' name='desl_hora' min='0' max='23' "
                "style='width:50%;' value='" + String(cfgRede.desligarHora) + "' "
                "placeholder='HH'>";
    conteudo += "<input type='number' name='desl_min'  min='0' max='59' "
                "style='width:50%;' value='" + String(cfgRede.desligarMin)  + "' "
                "placeholder='MM'>";
    conteudo += "</div></div>";
    conteudo += "</div>";

    // NTP warning
    if (!ntpSincronizado) {
        conteudo += "<div class='msg-warn' style='font-size:10px;margin-top:6px;'>"
                    "&#9888; NTP n&atilde;o sincronizado! Conecte ao WiFi para "
                    "usar o modo hor&aacute;rio.</div>";
    } else {
        char horaStr[10];
        time_t now = time(nullptr);
        struct tm* ti = localtime(&now);
        snprintf(horaStr, sizeof(horaStr), "%02d:%02d",
                 ti->tm_hour, ti->tm_min);
        conteudo += "<div class='msg-ok' style='font-size:10px;margin-top:6px;'>"
                    "&#10003; NTP OK - Hora atual: <b>"
                  + String(horaStr) + "</b></div>";
    }
    conteudo += "</div>"; // fim divHorario

    // Div modo painel (visível apenas no modo 2)
    conteudo += "<div id='divPainel' style='display:"
              + String(cfgRede.modoLampExt==2?"block":"none") + ";'>";
    conteudo += "<div class='info' style='font-size:10px;'>";
    conteudo += "&#127774; O ESP32 monitora a corrente do painel solar.<br>";
    conteudo += "Painel gerando &gt; 0.5A → <b>Lâmpada APAGA</b> (dia)<br>";
    conteudo += "Painel parado = 0A → <b>Lâmpada LIGA</b> (noite/nublado)<br>";
    conteudo += "&#9432; N&atilde;o requer NTP. Detecta amanhecer e anoitecer "
                "automaticamente.<br>";
    conteudo += "&#9432; Painel: <b id='painelStatusConf'>--</b>";
    conteudo += "</div></div>";

    // Estatísticas lâmpada externa
    conteudo += "<div style='background:#111;border:1px solid #333;"
                "border-radius:6px;padding:8px;font-size:10px;margin-top:8px;'>";
    conteudo += "<span style='color:#78909c;'>&#128202; Lamp.Ext: "
                "Ligadas por hor&aacute;rio: <b id='cLigH'>"
              + String(estadoRede.totalLigadasHorario)
              + "</b> &nbsp;|&nbsp; "
                "Desligadas por painel: <b id='cPain'>"
              + String(estadoRede.totalLigadasPainel)
              + "</b></span>";
    conteudo += "</div>";

    conteudo += "<button type='submit' class='btn-green' style='margin-top:12px;'>"
                "&#128190; Salvar Configura&ccedil;&atilde;o</button>";
    conteudo += "</form>";

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>&#8592; Voltar</button>";

    // ── SCRIPT ──
    String script = "";
    script += "var energiaDisp=" + String(eD,1) + ";";
    script += F("function calcWh(el,eD){"
                "var p=parseFloat(el.value)||0;"
                "var id='wh_'+el.name;"
                "var sp=document.getElementById(id);"
                "if(sp)sp.textContent=(eD*(p/100)).toFixed(0)+' Wh';}"

                "function toggleModoLamp(){"
                "var m=parseInt(document.getElementById('modoLampSel').value);"
                "document.getElementById('divHorario').style.display="
                "(m==1?'block':'none');"
                "document.getElementById('divPainel').style.display="
                "(m==2?'block':'none');}"

                "function fmtTempo(s){"
                "var m=Math.floor(s/60);var sc=s%60;"
                "return m+'m '+pad(sc)+'s';}"
                "function pad(n){return n<10?'0'+n:n;}"

                "function atualizarRede(){"
                "var x=new XMLHttpRequest();"
                "x.onreadystatechange=function(){"
                "if(x.readyState==4&&x.status==200){"
                "try{var d=JSON.parse(x.responseText);"

                // Barra energia
                "var p=(d.whDisp/d.whTotal)*100;"
                "p=Math.max(0,Math.min(100,p));"
                "var bEl=document.getElementById('barraEnergia');"
                "bEl.style.width=p+'%';"
                "bEl.style.background=p>=50?'#66bb6a':(p>=25?'#ffa726':'#ef5350');"
                "document.getElementById('whDispLabel').textContent="
                "d.whDisp.toFixed(0)+' Wh ('+d.soc.toFixed(1)+'%)';"

                // Helper status cards
                "function stBox(idB,idU,ativo,aguard,tRest,nOn,nOff){"
                "var b=document.getElementById(idB);"
                "var u=document.getElementById(idU);"
                "if(!b||!u)return;"
                "b.textContent=ativo?'✓ LIGADO':'✗ DESLIGADO';"
                "b.className='stat-val '+(ativo?'stat-ok':'stat-err');"
                "if(aguard&&tRest>0){"
                "u.textContent='⏱ Desl. em '+fmtTempo(tRest);"
                "u.style.color='#ffa726';}else{"
                "u.textContent=ativo?nOn:nOff;"
                "u.style.color='';}}"

                "stBox('stIBbox','stIBunit',d.stIB,d.agIB,d.tIB,"
                "'Rede baixa ativa','Rede baixa OFF');"
                "stBox('stRAbox','stRAunit',d.stRA,d.agRA,d.tRA,"
                "'Tomadas ativas','Tomadas OFF');"
                "stBox('stLEbox','stLEunit',d.stLE,d.agLE,d.tLE,"
                "'Lamp.ext ligada','Lamp.ext OFF');"
                "stBox('stLIbox','stLIunit',d.stLI,d.agLI,d.tLI,"
                "'Lamp.int ligada','Lamp.int OFF');"

                // Info lâmpada externa
                "var modos=['Energia','Horario','Painel'];"
                "var infoLE='';"
                "infoLE+='&#128161; Modo: <b>'+modos[d.mLE]+'</b> &nbsp;|&nbsp; ';"
                "if(d.mLE==1){"
                "infoLE+='&#128336; Liga: <b>'+pad(d.ligH)+':'+pad(d.ligM)+'</b>"
                " &nbsp;|&nbsp; ';"
                "infoLE+='Desliga: <b>'+pad(d.dslH)+':'+pad(d.dslM)+'</b>"
                " &nbsp;|&nbsp; ';"
                "infoLE+='Hora: <b>'+d.horaAtual+'</b><br>';}"
                "if(d.mLE==2){"
                "infoLE+='&#127774; Painel: <b>"
                "'+( d.painelAtivo?'GERANDO (dia)':'PARADO (noite)')+'</b><br>';}"
                "if(d.leBloq)infoLE+='<span style=\"color:#ef5350;\">"
                "&#9888; BLOQUEADA por energia baixa</span><br>';"
                "else if(d.leHor)infoLE+='<span style=\"color:#66bb6a;\">"
                "&#128336; Ligada por hor&aacute;rio</span><br>';"
                "else if(d.lePain&&!d.stLE)infoLE+='<span style=\"color:#ffa726;\">"
                "&#127774; Desligada: painel ativo (dia)</span><br>';"
                "else if(d.lePain&&d.stLE)infoLE+='<span style=\"color:#66bb6a;\">"
                "&#127774; Ligada: painel parado (noite)</span><br>';"
                "document.getElementById('infoLampExt').innerHTML=infoLE;"

                // Painel info
                "var pEl=document.getElementById('painelInfo');"
                "if(pEl){pEl.textContent=d.painelAtivo?"
                "'GERANDO (dia)':'PARADO (noite)';"
                "pEl.style.color=d.painelAtivo?'#66bb6a':'#78909c';}"
                "var pSEl=document.getElementById('painelStatusConf');"
                "if(pSEl){pSEl.textContent=d.painelAtivo?"
                "'GERANDO > 0.5A (dia)':'PARADO 0A (noite)';"
                "pSEl.style.color=d.painelAtivo?'#66bb6a':'#ffa726';}"

                "var hEl=document.getElementById('horaInfo');"
                "if(hEl)hEl.textContent=d.ntp?d.horaAtual:'Sem NTP';"

                // Contadores
                "var cLH=document.getElementById('cLigH');"
                "if(cLH)cLH.textContent=d.cLigH;"
                "var cP=document.getElementById('cPain');"
                "if(cP)cP.textContent=d.cPain;"

                // Alertas
                "var al='';"
                "if(d.agIB)al+='<div class=\"msg-err\">&#9201; Desligando"
                " Inv.Baixa em '+fmtTempo(d.tIB)+'</div>';"
                "if(d.agRA)al+='<div class=\"msg-warn\">&#9201; Desligando"
                " Rede Alta em '+fmtTempo(d.tRA)+'</div>';"
                "if(d.agLE)al+='<div class=\"msg-warn\">&#9201; Desligando"
                " Lamp.Ext em '+fmtTempo(d.tLE)+'</div>';"
                "if(d.agLI)al+='<div class=\"msg-warn\">&#9201; Desligando"
                " Lamp.Int em '+fmtTempo(d.tLI)+'</div>';"
                "document.getElementById('alertasRede').innerHTML=al;"

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
// PÁGINA: /previsao
// ============================================================
void handlePrevisao() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!checarAuth()) return;

    String msgHtml = "";

    // ── Processa POST (salvar configuração) ──
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "salvar") {
            String latStr = server.arg("lat");
            String lonStr = server.arg("lon");
            float  kwp    = server.arg("kwp").toFloat();

            double lat = latStr.toDouble();
            double lon = lonStr.toDouble();

            if (lat == 0.0 || lon == 0.0) {
                msgHtml = "<div class='msg-err'>&#10007; "
                          "Coordenadas inv&aacute;lidas</div>";
            } else if (kwp < 0.1f || kwp > 1000.0f) {
                msgHtml = "<div class='msg-err'>&#10007; "
                          "kWp inv&aacute;lido (0.1 - 1000)</div>";
            } else {
                cfgPrev.latitude     = lat;
                cfgPrev.longitude    = lon;
                cfgPrev.kwpInstalado = kwp;
                cfgPrev.configurado  = true;
                salvarConfigPrevisao();

                snprintf(tempBuf, sizeof(tempBuf),
                         "Prev config: lat=%.10f lon=%.10f kwp=%.2f",
                         lat, lon, kwp);
                adicionarLog(tempBuf);

                // Força busca imediata
                tPrevisao = 0;
                msgHtml = "<div class='msg-ok'>&#10003; "
                          "Configurado! Buscando previs&atilde;o...</div>";
            }
        } else if (acao == "reset_cal") {
            // Reseta calibração para aprender do zero
            for (int h = 0; h < 24; h++) {
                estadoPrev.fatorHorario[h]    = 1.0f;
                estadoPrev.amostrasHorario[h] = 0;
            }
            estadoPrev.taxaAcerto = 0;
            salvarCalibracaoHoraria(true);
            adicionarLog("Calibracao solar resetada");
            msgHtml = "<div class='msg-ok'>&#10003; "
                      "Calibra&ccedil;&atilde;o resetada!</div>";
        }
    }

    // Hora atual para exibição
    time_t agora = time(nullptr);
    struct tm* ti = localtime(&agora);
    int horaAtual = ti->tm_hour;

    String conteudo = msgHtml;

    // ── CONFIGURAÇÃO ──
    conteudo += "<h3>&#128205; Localiza&ccedil;&atilde;o do Painel</h3>";

    // Link Google Maps
    conteudo += "<div class='info' style='font-size:11px;'>";
    conteudo += "&#9432; <b>Como encontrar as coordenadas:</b><br>";
    conteudo += "1. Abra o Google Maps abaixo<br>";
    conteudo += "2. Navegue at&eacute; a localiza&ccedil;&atilde;o "
                "do seu painel solar<br>";
    conteudo += "3. Clique com o bot&atilde;o direito no local exato<br>";
    conteudo += "4. Copie as coordenadas que aparecem (ex: "
                "-22.917549, -47.017843)<br>";
    conteudo += "5. Cole nos campos abaixo com ALTA PRECIS&Atilde;O "
                "(todos os d&iacute;gitos)";
    conteudo += "</div>";

    // Botão Google Maps (nova guia)
    conteudo += "<a href='https://maps.google.com' target='_blank' "
                "style='display:block;text-align:center;padding:10px;"
                "background:#1565c0;color:#fff;border-radius:4px;"
                "text-decoration:none;font-weight:bold;margin:8px 0;'>"
                "&#127759; Abrir Google Maps (nova guia)</a>";

    // Formulário
    conteudo += "<form method='POST' action='/previsao'>";
    conteudo += "<input type='hidden' name='acao' value='salvar'>";

    conteudo += "<label>Latitude (alta precis&atilde;o - todos os "
                "d&iacute;gitos):</label>";
    conteudo += "<input type='text' name='lat' required "
                "placeholder='Ex: -22.917549777384778' "
                "value='" + (cfgPrev.configurado
                    ? String(cfgPrev.latitude, 15) : "")
                + "' style='font-family:monospace;'>";

    conteudo += "<label>Longitude (alta precis&atilde;o - todos os "
                "d&iacute;gitos):</label>";
    conteudo += "<input type='text' name='lon' required "
                "placeholder='Ex: -47.017843958072520' "
                "value='" + (cfgPrev.configurado
                    ? String(cfgPrev.longitude, 15) : "")
                + "' style='font-family:monospace;'>";

    conteudo += "<label>Pot&ecirc;ncia instalada (kWp total dos "
                "pain&eacute;is):</label>";
    conteudo += "<input type='number' name='kwp' min='0.1' max='1000' "
                "step='0.01' required placeholder='Ex: 3.30' "
                "value='" + String(cfgPrev.kwpInstalado, 2) + "'>";

    conteudo += "<button type='submit' class='btn-green'>"
                "&#128190; Salvar e Buscar Previs&atilde;o</button>";
    conteudo += "</form>";

    // ── STATUS ATUAL ──
    if (estadoPrev.dadosValidos) {
        int idxAtual = horaAtual - estadoPrev.horaOffsetAPI;
        if (idxAtual < 0) idxAtual = 0;

        float radAgora  = estadoPrev.radiacaoHora[idxAtual];
        float nuvAgora  = estadoPrev.nuvensPct[idxAtual];
        float gerAgora  = estadoPrev.geracaoEstHora[idxAtual];
        float gerReal   = sensores.potenciaPainel;

        conteudo += "<h3>&#9728; Status Atual</h3>";
        conteudo += "<div class='stat-grid'>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", radAgora);
        conteudo += "<div class='stat-box'>"
                    "<div class='stat-label'>Radia&ccedil;&atilde;o</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf) + "</div>"
                    "<div class='stat-unit'>W/m&sup2;</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", nuvAgora);
        conteudo += "<div class='stat-box'>"
                    "<div class='stat-label'>Cobertura Nuvens</div>"
                    "<div class='stat-val stat-warn'>" + String(tempBuf) + "</div>"
                    "<div class='stat-unit'>%</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", gerAgora);
        conteudo += "<div class='stat-box'>"
                    "<div class='stat-label'>Ger. Estimada</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf) + "</div>"
                    "<div class='stat-unit'>Wh/h</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.0f", gerReal);
        conteudo += "<div class='stat-box'>"
                    "<div class='stat-label'>Ger. Real Agora</div>"
                    "<div class='stat-val stat-ok' id='gerReal'>"
                    + String(tempBuf) + "</div>"
                    "<div class='stat-unit'>W</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.1f",
                 estadoPrev.taxaAcerto);
        conteudo += "<div class='stat-box'>"
                    "<div class='stat-label'>Taxa de Acerto</div>"
                    "<div class='stat-val stat-ok'>" + String(tempBuf) + "</div>"
                    "<div class='stat-unit'>%</div></div>";

        snprintf(tempBuf, sizeof(tempBuf), "%.3f",
                 estadoPrev.fatorHorario[horaAtual]);
        conteudo += "<div class='stat-box'>"
                    "<div class='stat-label'>Fator Cal. Hora</div>"
                    "<div class='stat-val'>" + String(tempBuf) + "</div>"
                    "<div class='stat-unit'>x</div></div>";

        conteudo += "</div>"; // stat-grid

        // ── TOTAIS ──
        conteudo += "<div class='info' style='font-size:11px;'>";
        snprintf(tempBuf, sizeof(tempBuf),
                 "&#9728; Restante hoje: <b>%.0f Wh</b>"
                 " &nbsp;|&nbsp; "
                 "&#128197; Amanha total: <b>%.0f Wh</b>"
                 " &nbsp;|&nbsp; "
                 "&#127968; Consumo medio: <b>%.0f Wh/dia</b>",
                 estadoPrev.totalRestanteHoje,
                 estadoPrev.totalAmanha,
                 estadoPrev.consumoMedioDia);
        conteudo += String(tempBuf);
        conteudo += "</div>";

        // ── DECISÕES AUTOMÁTICAS ──
        conteudo += "<h3>&#129302; Decis&otilde;es Autom&aacute;ticas</h3>";
        conteudo += "<div style='background:#111;border:1px solid #333;"
                    "border-radius:6px;padding:10px;font-size:11px;'>";

        // Rede Alta
        conteudo += "<div style='padding:6px 0;border-bottom:1px solid #1a1a1a;'>";
        conteudo += "<span style='color:#78909c;'>&#9889; Rede Alta: </span>";
        if (estadoPrev.bloqRedeAlta) {
            conteudo += "<span style='color:#ef5350;font-weight:bold;'>"
                        "&#10007; DESATIVADA por previs&atilde;o</span><br>";
            conteudo += "<span style='color:#546e7a;font-size:10px;'>"
                        "Motivo: " + String(estadoPrev.motivoRedeAlta)
                      + "</span>";
        } else {
            conteudo += "<span style='color:#66bb6a;'>&#10003; Normal"
                        " (sem bloqueio de previs&atilde;o)</span>";
        }
        conteudo += "</div>";

        // Lâmpada Externa
        conteudo += "<div style='padding:6px 0;'>";
        conteudo += "<span style='color:#78909c;'>&#128161; Lamp.Externa: </span>";
        if (estadoPrev.bloqLampExt) {
            conteudo += "<span style='color:#ef5350;font-weight:bold;'>"
                        "&#10007; DESATIVADA por previs&atilde;o</span><br>";
            conteudo += "<span style='color:#546e7a;font-size:10px;'>"
                        "Motivo: " + String(estadoPrev.motivoLampExt)
                      + "</span>";
        } else {
            conteudo += "<span style='color:#66bb6a;'>&#10003; Normal"
                        " (sem bloqueio de previs&atilde;o)</span>";
        }
        conteudo += "</div>";
        conteudo += "</div>"; // card

        // ── PREVISÃO PRÓXIMAS HORAS ──
        conteudo += "<h3>&#128202; Pr&oacute;ximas Horas</h3>";
        conteudo += "<div id='graficoHoras' style='background:#000;"
                    "border:1px solid #333;border-radius:8px;"
                    "padding:10px;'>";
        conteudo += "<div style='color:#546e7a;text-align:center;"
                    "font-size:11px;'>Carregando...</div>";
        conteudo += "</div>";

        // ── PREVISÃO AMANHÃ ──
        conteudo += "<h3>&#128197; Previs&atilde;o Amanh&atilde;</h3>";
        conteudo += "<div id='graficoAmanha' style='background:#000;"
                    "border:1px solid #333;border-radius:8px;"
                    "padding:10px;'>";
        conteudo += "<div style='color:#546e7a;text-align:center;"
                    "font-size:11px;'>Carregando...</div>";
        conteudo += "</div>";

        // ── CALIBRAÇÃO HORÁRIA ──
        conteudo += "<h3>&#9881; Calibra&ccedil;&atilde;o por Hora</h3>";
        conteudo += "<div style='overflow-x:auto;'>";
        conteudo += "<table style='width:100%;border-collapse:collapse;"
                    "font-size:9px;'>";
        conteudo += "<thead><tr style='background:#1e3a5f;color:#90caf9;'>";
        conteudo += "<th style='padding:4px;'>Hora</th>";
        conteudo += "<th style='padding:4px;'>Fator</th>";
        conteudo += "<th style='padding:4px;'>Amostras</th>";
        conteudo += "<th style='padding:4px;'>Status</th>";
        conteudo += "</tr></thead><tbody>";

        for (int h = 0; h < 24; h++) {
            float f = estadoPrev.fatorHorario[h];
            uint8_t a = estadoPrev.amostrasHorario[h];
            const char* bg = (h == horaAtual) ? "#1e3a1e" :
                             (h % 2 == 0) ? "#0d0d0d" : "#111";
            const char* corF = (f > 1.1f) ? "#ef5350" :
                               (f < 0.9f) ? "#ffa726" : "#66bb6a";
            const char* status = (a == 0)   ? "Sem dados" :
                                 (a < 5)    ? "Aprendendo" :
                                 (a < 20)   ? "Bom" : "Calibrado";

            conteudo += "<tr style='background:" + String(bg) + ";'>";
            snprintf(tempBuf, sizeof(tempBuf), "%02dh", h);
            conteudo += "<td style='padding:4px;color:#78909c;'>"
                      + String(tempBuf) + "</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%.3f", f);
            conteudo += "<td style='padding:4px;text-align:center;"
                        "color:" + String(corF) + ";font-weight:bold;'>"
                      + String(tempBuf) + "</td>";
            snprintf(tempBuf, sizeof(tempBuf), "%d", (int)a);
            conteudo += "<td style='padding:4px;text-align:center;"
                        "color:#546e7a;'>" + String(tempBuf) + "</td>";
            conteudo += "<td style='padding:4px;color:#546e7a;'>"
                      + String(status) + "</td>";
            conteudo += "</tr>";
        }
        conteudo += "</tbody></table></div>";

        // Reset calibração
        conteudo += "<form method='POST' action='/previsao' "
                    "style='margin-top:8px;'>";
        conteudo += "<input type='hidden' name='acao' value='reset_cal'>";
        conteudo += "<button type='submit' class='btn-orange' "
                    "style='font-size:11px;'>"
                    "&#8635; Resetar Calibra&ccedil;&atilde;o</button>";
        conteudo += "</form>";

    } else {
        // Sem dados ainda
        conteudo += "<div class='msg-warn'>";
        if (!cfgPrev.configurado) {
            conteudo += "&#9888; Configure a localiza&ccedil;&atilde;o "
                        "e kWp acima para iniciar.";
        } else if (modoAP) {
            conteudo += "&#9888; Sem internet (modo AP). Conecte ao WiFi"
                        " para buscar previs&atilde;o.";
        } else {
            conteudo += "&#9888; Aguardando dados meteorol&oacute;gicos...";
        }
        conteudo += "</div>";
        if (cfgPrev.configurado) {
            conteudo += "<div class='info' style='font-size:11px;'>";
            snprintf(tempBuf, sizeof(tempBuf),
                     "Erros consecutivos: %d",
                     (int)estadoPrev.errosConsecutivos);
            conteudo += String(tempBuf);
            conteudo += "</div>";
        }
    }

    // Info última atualização
    if (estadoPrev.tsUltimaAtualizacao > 0) {
        unsigned long decorrido = (millis() -
            estadoPrev.tsUltimaAtualizacao) / 60000;
        conteudo += "<div class='info' style='font-size:10px;'>";
        snprintf(tempBuf, sizeof(tempBuf),
                 "&#128337; &Uacute;ltima atualiza&ccedil;&atilde;o: "
                 "h&aacute; %lu min | "
                 "Pr&oacute;xima em: %lu min",
                 decorrido,
                 60 - min((unsigned long)60, decorrido));
        conteudo += String(tempBuf);
        conteudo += "</div>";
    }

    conteudo += "<button onclick='location.href=\"/\"' class='btn-gray' "
                "style='margin-top:8px;font-size:12px;'>"
                "&#8592; Voltar</button>";

    // ── SCRIPT TEMPO REAL ──
    String script = "";
    script += F("function pad(n){return n<10?'0'+n:n;}");
    script += F("function corNuvem(n){"
                "if(n<30)return'#66bb6a';"
                "if(n<60)return'#ffa726';"
                "return'#ef5350';}"
                "function iconeNuvem(n,c){"
                "if(c>30)return'&#9928;';"   // chuva
                "if(n<20)return'&#9728;';"   // sol
                "if(n<50)return'&#9925;';"   // sol+nuvem
                "return'&#9729;';}"          // nublado

                "function gerarGrafico(containerId,dados,maxG){"
                "var c=document.getElementById(containerId);"
                "if(!c)return;"
                "var h='';"
                "for(var i=0;i<dados.length;i++){"
                "var d=dados[i];"
                "var pct=maxG>0?(d.g/maxG*100):0;"
                "pct=Math.min(pct,100);"
                "var corB=d.n<30?'#66bb6a':(d.n<60?'#ffa726':'#546e7a');"
                "h+='<div style=\"display:flex;align-items:center;"
                "margin-bottom:4px;\">';"
                "h+='<span style=\"width:28px;color:#546e7a;"
                "font-size:9px;text-align:right;margin-right:4px;\">"
                "'+pad(d.h)+'h</span>';"
                "h+='<span style=\"font-size:12px;margin-right:4px;\">"
                "'+iconeNuvem(d.n,d.c)+'</span>';"
                "h+='<div style=\"flex:1;background:#1a1a1a;"
                "border-radius:2px;height:16px;\">';"
                "h+='<div style=\"height:100%;background:'+corB+"
                "';border-radius:2px;width:'+pct.toFixed(0)+"
                "'%;transition:width 0.5s;\"></div></div>';"
                "h+='<span style=\"width:60px;text-align:right;"
                "color:#4fc3f7;font-size:9px;margin-left:4px;\">"
                "'+d.g.toFixed(0)+' Wh</span>';"
                "h+='<span style=\"width:40px;text-align:right;"
                "color:#546e7a;font-size:9px;\">"
                "'+d.n.toFixed(0)+'%&#9729;</span>';"
                "h+='</div>';"
                "}"
                "c.innerHTML=h||'<div style=\"color:#555\">"
                "Sem dados</div>';}"

                "function atualizarPrevisao(){"
                "var x=new XMLHttpRequest();"
                "x.onreadystatechange=function(){"
                "if(x.readyState==4&&x.status==200){"
                "try{var d=JSON.parse(x.responseText);"
                // Atualiza geração real
                "var gr=document.getElementById('gerReal');"
                "if(gr)gr.textContent=d.gerRealAgora.toFixed(0);"
                // Calcula max para escala dos gráficos
                "var maxH=0,maxA=0;"
                "for(var i=0;i<d.horas.length;i++)"
                "if(d.horas[i].g>maxH)maxH=d.horas[i].g;"
                "for(var i=0;i<d.amanha.length;i++)"
                "if(d.amanha[i].g>maxA)maxA=d.amanha[i].g;"
                // Gera gráficos
                "gerarGrafico('graficoHoras',d.horas,maxH);"
                "gerarGrafico('graficoAmanha',d.amanha,maxA);"
                "}catch(e){}"
                "}};"
                "x.open('GET','/api/previsao',true);"
                "x.send();}"
                "setInterval(atualizarPrevisao,10000);"
                "atualizarPrevisao();");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS Energia - Previs&atilde;o Solar",
                            conteudo, script));
}

// ============================================================
// LOGOUT
// ============================================================
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
    server.on("/",               handleRoot);
    server.on("/login",          handleLogin);
    server.on("/logout",         handleLogout);
    server.on("/setup-login",    handleSetupLogin);
    server.on("/setup-wifi",     handleSetupWifi);
    server.on("/bateria",        handleBateria);
    server.on("/inversores",     handleInversores);
    server.on("/rede",           handleRede);
    server.on("/historico",      handleHistorico);
    server.on("/config",         handleConfig);
    server.on("/previsao",       handlePrevisao);
    server.on("/logs",           handleLogs);
    server.on("/api/dados",      handleApiDados);
    server.on("/api/inversores", handleApiInversores);
    server.on("/api/rede",       handleApiRede);
    server.on("/api/previsao",   handleApiPrevisao);
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
    logRAMSeNecessario();

    // ── Bateria ──
    if (existeBateriaConfig()) {
        carregarConfigBateria();
        carregarEstadoBateria();
        snprintf(tempBuf, sizeof(tempBuf),
                 "Bat: %.1fV %.0fAh carregada: %s",
                 cfgBat.tensaoNominal,
                 cfgBat.amperagem,
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
        adicionarLog("Bateria: usando valores padrao");
    }

    // ── Contadores ──
    carregarContadores();
    snprintf(tempBuf, sizeof(tempBuf),
             "Contadores: P=%.2fkWh C=%.2fkWh",
             contadores.kwhPainelTotal,
             contadores.kwhCasaTotal);
    adicionarLog(tempBuf);

    // ── Inversores ──
    carregarConfigInversor();
    carregarEstadoInversor();
    if (existeConfigInversor()) {
        recalcularInversor();
        snprintf(tempBuf, sizeof(tempBuf),
                 "Inv: Baixa=%.0fW(%.1fA) Alta=%.0fW(%.1fA) Chav=%.0f%%",
                 cfgInv.potenciaBaixa, cfgInv.correnteBaixa,
                 cfgInv.potenciaAlta,  cfgInv.correnteAlta,
                 cfgInv.percChaveamento);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Inversores: aguardando configuracao");
    }

    // ── Rede Elétrica ──
    carregarConfigRede();
    carregarEstadoRede();
    if (existeConfigRede()) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Rede: IB=%.0f%% RA=%.0f%% LE=%.0f%% LI=%.0f%%",
                 cfgRede.percInvBaixa, cfgRede.percRedeAlta,
                 cfgRede.percLampExt,  cfgRede.percLampInt);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Rede eletrica: usando valores padrao");
    }

    // ── Previsão Solar ──
    carregarConfigPrevisao();
    carregarCalibracaoHoraria();
    // Inicializa flags de bloqueio como false (seguro)
    estadoPrev.bloqRedeAlta         = false;
    estadoPrev.bloqLampExt          = false;
    estadoPrev.redeAltaFoiDesligada = false;
    estadoPrev.lampExtFoiDesligada  = false;
    estadoPrev.dadosValidos         = false;
    estadoPrev.errosConsecutivos    = 0;
    estadoPrev.geracaoRealAcumHora  = 0;
    estadoPrev.geracaoEstAcumHora   = 0;
    estadoPrev.minutosNaHora        = 0;
    tPrevisao = 0; // força busca imediata após conectar

    if (cfgPrev.configurado) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Prev: lat=%.8f lon=%.8f kwp=%.2fkWp",
                 (float)cfgPrev.latitude,
                 (float)cfgPrev.longitude,
                 cfgPrev.kwpInstalado);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Previsao: aguardando configuracao");
    }

    // ── Hardware ──
    setupInversores();
    setupRede();

    // ── Sensores ──
    delay(500);
    calibrarZeroSensores();
    pinMode(PIN_BKP_BATERIA, INPUT);

    // ── WiFi ──
    bool temCred = existeCredenciais();
    bool temWifi = existeWiFiConfig();

    if (!temCred || !temWifi) {
        adicionarLog("Primeira vez - modo setup AP");
        iniciarAP();
    } else {
        String ssid, pass, ip, gw, mask, dns, tz;
        bool   ap;
        carregarWifi(ssid, pass, ip, gw, mask, dns, tz, ap);

        if (ap) {
            adicionarLog("Modo AP configurado pelo usuario");
            iniciarAP();
        } else {
            adicionarLog("Conectando ao WiFi...");
            if (!iniciarWiFi(ssid, pass, ip, gw, mask, dns)) {
                adicionarLog("Falha WiFi - fallback AP");
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

    adicionarLog("Sistema RRFS pronto!");
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

        // 1. Lê sensores
        //    → atualiza: correntePainel, correnteCasa,
        //      tensaoBateriaEst, potenciaPainel
        //      (reutilizados por todos os sistemas abaixo)
        lerSensores();

        // 2. Atualiza bateria (coulomb counting)
        //    → atualiza: ahDisponiveis, ahMaximos
        //      (reutilizados por controlarRede e previsão)
        if (existeBateriaConfig()) {
            atualizarEstadoBateria(delta);
        }

        // 3. Calibração da previsão solar
        //    → reutiliza: correntePainel (calculado em 1)
        //    → reutiliza: tensaoBateriaEst (calculado em 1)
        //    → reutiliza: dados API já carregados
        //    → ZERO leituras ADC extras
        if (estadoPrev.dadosValidos && cfgPrev.configurado
            && !modoAP) {
            calibrarFatorHorario();
            }

            // 4. Controla inversores
            //    → reutiliza: correnteCasa (calculado em 1)
            //    → reutiliza: limiteChaveamento (pré-calculado)
            if (existeConfigInversor()) {
                controlarInversores();
            }

            // 5. Controla rede elétrica
            //    → reutiliza: ahDisponiveis (calculado em 2)
            //    → reutiliza: tensaoBateriaEst (calculado em 1)
            //    → reutiliza: energiaDisponivel (pré-calculado)
            //    → reutiliza: bloqRedeAlta/bloqLampExt (da previsão)
            //    → ZERO recálculos duplicados
            if (existeBateriaConfig()) {
                controlarRede();
            }
    }

    // ── Previsão Solar - busca API (a cada 1 hora) ──
    // tPrevisao=0 no setup() força busca na primeira execução
    bool podeAtualizarPrev = (!modoAP
    && WiFi.status() == WL_CONNECTED
    && ntpSincronizado
    && cfgPrev.configurado);

    bool deveAtualizarPrev = podeAtualizarPrev
    && ((tPrevisao == 0)
    || (agora - tPrevisao >= INTERVALO_PREVISAO));

    if (deveAtualizarPrev) {
        tPrevisao = agora;
        adicionarLog("Prev: atualizando dados meteorologicos...");
        if (buscarDadosOpenMeteo()) {
            calcularGeracaoEstimada();
            calcularConsumoMedio();   // reutiliza histórico existente
            tomarDecisoesPrev();      // reutiliza valores já calculados
            snprintf(tempBuf, sizeof(tempBuf),
                     "Prev: Hoje=%.0fWh Amanha=%.0fWh Acerto=%.1f%%",
                     estadoPrev.totalRestanteHoje,
                     estadoPrev.totalAmanha,
                     estadoPrev.taxaAcerto);
            adicionarLog(tempBuf);
        } else {
            // Retry mais rápido se falhou (5 min)
            if (estadoPrev.errosConsecutivos > 0) {
                tPrevisao = agora - INTERVALO_PREVISAO + 300000UL;
            }
        }
        logRAMSeNecessario();
    }

    // ── Decisões de previsão (a cada 5 minutos) ──
    // Reavalia condições sem buscar API novamente
    static unsigned long tDecisao = 0;
    if (estadoPrev.dadosValidos && cfgPrev.configurado
        && ntpSincronizado
        && (agora - tDecisao >= 300000UL)) {
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
                while (WiFi.status() != WL_CONNECTED
                    && millis() - t < 15000) {
                    atualizarLED();
                // Mantém controles ativos durante reconexão
                if (existeConfigInversor()) controlarInversores();
                if (existeBateriaConfig())  controlarRede();
                delay(200);
                    }

                    if (WiFi.status() == WL_CONNECTED) {
                        adicionarLog("WiFi reconectado!");
                        ledFixo(true);
                        iniciarNTP();
                        // Força re-busca da previsão após reconexão
                        tPrevisao = 0;
                    } else {
                        adicionarLog("Reconexao falhou - modo AP");
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

            // Salva todos os estados periodicamente
            if (existeConfigInversor()) salvarEstadoInversor();
            if (existeConfigRede())     salvarEstadoRede();
            // Salva calibração se houve mudança
            if (cfgPrev.configurado)
                salvarCalibracaoHoraria(false);
        }

        delay(1);
}

/*

Documentação Completa - Equações do Sistema RRFS Energia V4.0
1. CONVERSÃO ADC → TENSÃO

text

Formula:
VOLTS_POR_BIT = VREF / RESOLUCAO
VOLTS_POR_BIT = 3.3 / 4095 = 0.000806 V/bit

Para que serve:
Converte o valor digital do ADC (0 a 4095)
em tensão real no pino do ESP32

2. DIVISOR DE TENSÃO (5V → 3.3V)

text

Formula:
FATOR_DIVISOR = 3.3 / 5.0 = 0.66

Para que serve:
O sensor ACS758 opera em 5V
O ESP32 suporta no máximo 3.3V
O divisor resistivo reduz o sinal do sensor
para não danificar o ESP32

3. SENSIBILIDADE REAL DO SENSOR (após divisor)

text

Formula:
SENS_50A  = 0.040 V/A × 0.66 = 0.0264 V/A
SENS_100A = 0.020 V/A × 0.66 = 0.0132 V/A

Para que serve:
Define quantos Volts o sinal muda
para cada Ampere de corrente medida
já considerando o divisor de tensão

4. MÉDIA DAS AMOSTRAS ADC

text

Formula:
RAW_MEDIO = (amostra1 + amostra2 + ... + amostra64) / 64

Para que serve:
Reduz ruído eletrônico do ADC
64 amostras aumentam a precisão da leitura
elimina picos e variações momentâneas

5. DELTA ADC (variação em relação ao zero)

text

Formula:
DELTA_RAW = RAW_MEDIO - OFFSET_ZERO

onde OFFSET_ZERO = leitura sem corrente (calibrado na inicialização)

Para que serve:
Encontra a variação real do sinal
O sensor com 0A lê ~2047 (2.5V após divisor)
O delta mostra quanto variou acima ou abaixo do zero
positivo = corrente numa direção
negativo = corrente na direção oposta

6. CONVERSÃO DELTA → CORRENTE REAL

text

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

7. FILTRO DE RUÍDO

text

Formula:
SE |CORRENTE| < 0.3A → CORRENTE = 0

Para que serve:
Elimina leituras falsas causadas por
ruído eletrônico quando não há corrente real
evita acúmulo de erro no coulomb counting

8. DETECÇÃO DE INVERSÃO - PAINEL

text

Formula:
SE CORRENTE_PAINEL > +1.0A → SENSOR INVERTIDO

Ação: CORRENTE_PAINEL = |CORRENTE_PAINEL| e emite alerta

SE CORRENTE_PAINEL entre -0.3A e -1.0A → normal (controlador de carga)
Ação: CORRENTE_PAINEL = 0 (sem alerta)

Para que serve:
O painel sempre gera corrente numa direção
Se o sinal sobe (positivo) significa fios invertidos
Corrente reversa menor que 1A é o consumo
normal do próprio controlador de carga à noite

9. DETECÇÃO DE INVERSÃO - CASA

text

Formula:
SE CORRENTE_CASA < -0.5A → SENSOR INVERTIDO

Ação: CORRENTE_CASA = |CORRENTE_CASA| e emite alerta

Para que serve:
A casa sempre consome corrente numa direção
Se o sinal for negativo os fios estão invertidos
Protege contra cálculos errados de energia

10. CORRENTE DA BATERIA

text

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

11. CAPACIDADE TOTAL DA BATERIA

text

Formula:
CAPACIDADE_TOTAL (Wh) = TENSAO_NOMINAL × AMPERAGEM

Exemplo:
12.8V × 100Ah = 1280 Wh = 1.28 kWh

Para que serve:
Calcula a energia máxima que a bateria
pode armazenar quando completamente carregada

12. ENERGIA DISPONÍVEL SEGURA

text

Formula:
ENERGIA_DISPONIVEL (Wh) = CAPACIDADE_TOTAL × PERC_DESCARGA_SEGURA

Exemplo:
1280 Wh × 0.80 = 1024 Wh

AH_MAXIMOS = AMPERAGEM × PERC_DESCARGA_SEGURA
Exemplo:
100Ah × 0.80 = 80Ah utilizáveis com segurança

Para que serve:
Define o limite seguro de uso da bateria
LiFePO4 pode descarregar até 80% sem danos
usar além deste limite reduz a vida útil

13. TENSÃO MÁXIMA DA BATERIA LiFePO4

text

Formula:
SE TENSAO_NOMINAL <= 13.0V → TENSAO_MAXIMA = 14.0V  (sistema 12V / 4S)
SE TENSAO_NOMINAL <= 26.0V → TENSAO_MAXIMA = 28.0V  (sistema 24V / 8S)
SE TENSAO_NOMINAL >  26.0V → TENSAO_MAXIMA = TENSAO_NOMINAL × 1.094

Para que serve:
Define a tensão de carga completa da bateria LiFePO4
Cada célula LiFePO4 carrega até 3.65V
4 células em série = 14.6V máximo (usa 14.0V como seguro)
8 células em série = 29.2V máximo (usa 28.0V como seguro)

14. COULOMB COUNTING - DELTA DE CARGA

text

Formula:
DELTA_AH = CORRENTE_BATERIA × DELTA_TEMPO_SEGUNDOS / 3600

Para que serve:
Calcula quantos Ah foram adicionados ou removidos
da bateria no intervalo entre duas leituras
dividir por 3600 converte segundos para horas
(1 Ah = 1 Ampere durante 1 hora = 3600 segundos)

Exemplo:
Descarregando 10A por 30 segundos:
DELTA_AH = -10 × 30 / 3600 = -0.0833 Ah removidos

15. ATUALIZAÇÃO DOS AH DISPONÍVEIS

text

Formula:
AH_DISPONIVEIS = AH_DISPONIVEIS + DELTA_AH

Limites de segurança:
AH_DISPONIVEIS = máximo(0, mínimo(AH_DISPONIVEIS, AH_MAXIMOS))

Para que serve:
Rastreia a carga atual da bateria em tempo real
Nunca permite valor negativo ou acima do máximo
É o coração do sistema de gerenciamento de energia

16. DETECÇÃO DE BATERIA 100% CARREGADA

text

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
O painel está gerando mas a corrente vai toda para a casa
A bateria não recebe nem entrega corrente = cheia!
Este evento RECALIBRA o coulomb counting zerando
qualquer erro acumulado ao longo do tempo
É o mesmo método usado em BMSs profissionais

17. SoC - ESTADO DE CARGA (percentual)

text

Formula:
SoC (%) = (AH_DISPONIVEIS / AH_MAXIMOS) × 100

Limites: entre 0% e 100%

Para que serve:
Mostra ao usuário a porcentagem de carga
disponível na bateria de forma intuitiva

18. ESTIMATIVA DE TENSÃO - CURVA OCV LiFePO4

text

Formula:
FATOR_ESCALA = TENSAO_NOMINAL / 12.8

Tabela OCV × SoC para LiFePO4 (base 12.8V):
┌──────┬───────┬───────┬───────┬───────┬───────┐
│ SoC  │  0%   │  10%  │  20%  │  30%  │  40%  │
│ OCV  │ 10.00 │ 12.00 │ 12.50 │ 12.80 │ 12.90 │
├──────┼───────┼───────┼───────┼───────┼───────┤
│ SoC  │  50%  │  60%  │  70%  │  80%  │  90%  │  100% │
│ OCV  │ 13.00 │ 13.10 │ 13.20 │ 13.30 │ 13.50 │ 14.00 │
└──────┴───────┴───────┴───────┴───────┴───────┘

Interpolação linear entre pontos:
INDEX     = inteiro(SoC × 10)
FRACAO    = (SoC × 10) - INDEX
OCV_BASE  = tabela[INDEX] + FRACAO × (tabela[INDEX+1] - tabela[INDEX])
OCV_FINAL = OCV_BASE × FATOR_ESCALA

Para que serve:
Estima a tensão atual da bateria baseada no SoC
sem necessidade de sensor de tensão físico
Serve para exibição e cálculo de potência
Precisão: ±0.05V entre 20% e 80% de SoC

19. POTÊNCIAS

text

Formula:
POTENCIA_PAINEL  (W) = CORRENTE_PAINEL  × TENSAO_ESTIMADA
POTENCIA_CASA    (W) = CORRENTE_CASA    × TENSAO_ESTIMADA
POTENCIA_BATERIA (W) = CORRENTE_BATERIA × TENSAO_ESTIMADA

Positivo = energia entrando na bateria (carregando)
Negativo = energia saindo da bateria (descarregando)

Para que serve:
Calcula a potência instantânea em Watts
de cada parte do sistema
P = V × I  (lei de Ohm para potência)

20. ENERGIA ACUMULADA (Wh)

text

Formula:
WH_PAINEL = POTENCIA_PAINEL × DELTA_TEMPO_SEGUNDOS / 3600
WH_CASA   = POTENCIA_CASA   × DELTA_TEMPO_SEGUNDOS / 3600
WH_BATERIA= POTENCIA_BATERIA× DELTA_TEMPO_SEGUNDOS / 3600

Para que serve:
Calcula a energia em Watt-hora produzida
ou consumida em cada intervalo de leitura
base para os contadores e histórico

21. CONTADORES TOTAIS (kWh)

text

Formula:
KWH_PAINEL_TOTAL   += WH_PAINEL   / 1000
KWH_CASA_TOTAL     += WH_CASA     / 1000
KWH_CARGA_TOTAL    += WH_BATERIA  / 1000  (só quando positivo)
KWH_DESCARGA_TOTAL += |WH_BATERIA|/ 1000  (só quando negativo)

Para que serve:
Acumula toda a energia gerada e consumida
desde o início do sistema
10 dígitos: 000000.0000 kWh
Salvo na memória flash a cada 5 minutos
nunca se perde mesmo sem energia

22. TEMPO ESTIMADO

text

Formula A - Bateria DESCARREGANDO:
TEMPO_HORAS = AH_DISPONIVEIS / |CORRENTE_BATERIA|

Formula B - Bateria CARREGANDO:
AH_FALTAM   = AH_MAXIMOS - AH_DISPONIVEIS
TEMPO_HORAS = AH_FALTAM  / CORRENTE_BATERIA

Conversão para exibição:
HORAS   = inteiro(TEMPO_HORAS)
MINUTOS = inteiro((TEMPO_HORAS - HORAS) × 60)

Exemplos:
Disponível 80Ah consumindo 10A → 80/10 = 8h 00min
Faltam 30Ah carregando a 15A  → 30/15 = 2h 00min
Disponível 60Ah consumindo 25A → 60/25 = 2h 24min

Para que serve:
Informa ao usuário quanto tempo resta de energia
ou quanto tempo para completar a carga
ALTA PRECISÃO pois depende apenas de
corrente medida e Ah conhecidos

23. CICLOS DA BATERIA

text

Formula:
A cada leitura com CORRENTE_BATERIA negativa (descarregando):

DELTA_AH_ABS    = |DELTA_AH|
CICLOS_PARCIAIS = CICLOS_PARCIAIS + (DELTA_AH_ABS / AH_MAXIMOS)

SE CICLOS_PARCIAIS >= 1.0:
CICLOS          = CICLOS + 1
CICLOS_PARCIAIS = CICLOS_PARCIAIS - 1.0

Para que serve:
Conta quantos ciclos completos de descarga
a bateria já realizou
1 ciclo = descarregar 100% da energia disponível
(pode ser em várias descargas parciais)
LiFePO4 suporta tipicamente 2000-6000 ciclos

24. ACUMULADORES DO HISTÓRICO

text

Formula:
A cada segundo acumula:
ACUM_WH_PAINEL   += WH_PAINEL
ACUM_WH_CASA     += WH_CASA
ACUM_WH_CARGA    += WH_BATERIA (se positivo)
ACUM_WH_DESCARGA += |WH_BATERIA| (se negativo)

A cada 60 segundos salva no histórico:
REGISTRO.whPainel      = inteiro(ACUM_WH_PAINEL   × 10)
REGISTRO.whCasa        = inteiro(ACUM_WH_CASA      × 10)
REGISTRO.whCargaBat    = inteiro(ACUM_WH_CARGA     × 10)
REGISTRO.whDescargaBat = inteiro(ACUM_WH_DESCARGA  × 10)

Multiplica por 10 para salvar 1 casa decimal
em inteiro de 16 bits (economiza memória)

Para que serve:
Compacta os dados de energia minuto a minuto
para histórico de 24 horas em RAM
int16 usa 2 bytes vs float que usa 4 bytes
economia de 50% de memória por registro

RESUMO VISUAL DO FLUXO DE CÁLCULO

text

┌─────────────────────────────────────────────────────────────┐
│                    FLUXO DE CÁLCULO                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ADC Bruto (64 amostras)                                    │
│       ↓  Equação 4 (média)                                  │
│  RAW_MEDIO                                                  │
│       ↓  Equação 5 (delta do zero)                          │
│  DELTA_RAW                                                  │
│       ↓  Equação 6 (conversão)                              │
│  CORRENTE_PAINEL   CORRENTE_CASA                            │
│       ↓  Equação 7 (filtro)                                 │
│  CORRENTE FILTRADA                                          │
│       ↓  Equações 8 e 9 (verificação inversão)             │
│  CORRENTE VALIDADA + ALERTAS                                │
│       ↓  Equação 10                                         │
│  CORRENTE_BATERIA                                           │
│       ↓                           ↓                         │
│  Equação 14+15                Equação 17                    │
│  DELTA_AH                     SoC (%)                       │
│  AH_DISPONIVEIS                   ↓                         │
│       ↓                       Equação 18                    │
│  Equação 16                   TENSAO_ESTIMADA               │
│  Detecção 100%                    ↓                         │
│  (recalibração)               Equação 19                    │
│       ↓                       POTENCIAS                     │
│  Equação 22                       ↓                         │
│  TEMPO ESTIMADO               Equação 20+21                 │
│       ↓                       ENERGIA Wh + kWh              │
│  Equação 23                       ↓                         │
│  CICLOS                       Equação 24                    │
│                               HISTÓRICO                     │
└─────────────────────────────────────────────────────────────┘

25. CORRENTE MÁXIMA DOS INVERSORES

Formula:
I_MAX_BAIXA (A) = POTENCIA_BAIXA (W) / TENSAO_NOMINAL (V)
I_MAX_ALTA  (A) = POTENCIA_ALTA  (W) / TENSAO_NOMINAL (V)

Exemplo configuração padrão:
I_MAX_BAIXA = 500  / 12.8 =  39.06A
I_MAX_ALTA  = 3000 / 12.8 = 234.37A

Para que serve:
Define a corrente máxima que cada inversor
pode fornecer baseado na tensão nominal da bateria
Automático: recalcula sempre que tensão muda

26. LIMITE DE CHAVEAMENTO (com histerese)

Formula:
LIMITE_CHAV   = I_MAX_BAIXA × PERC_CHAVEAMENTO / 100
LIMITE_RETORNO= LIMITE_CHAV × (1 - HISTERESE / 100)

Exemplo (70% chaveamento, 5% histerese):
LIMITE_CHAV    = 39.06 × 0.70 = 27.34A
LIMITE_RETORNO = 27.34 × 0.95 = 25.97A

Para que serve:
LIMITE_CHAV:    acima disso liga inversor alta
LIMITE_RETORNO: abaixo disso desliga inversor alta
Histerese evita chaveamento rápido (flutter)
quando a corrente oscila em torno do limite

27. LÓGICA DOS RELÉS

RELÉ REDE ALTA:
HIGH = rede de alta alimentada pelo Inversor Alta
LOW  = rede de alta alimentada pelo Inversor Baixa

RELÉ INVERSOR BAIXA (lógica invertida):
HIGH = inversor DESLIGADO (relé energizado abre circuito)
LOW  = inversor LIGADO    (relé desligado, passagem livre)

RELÉ INVERSOR ALTA (lógica direta):
HIGH = inversor LIGADO    (relé energizado liga inversor)
LOW  = inversor DESLIGADO (relé sem energia, desligado)

Para que serve:
Lógica invertida no relé de baixa é SAFETY FAIL-SAFE:
Se o ESP32 reiniciar ou perder energia, o relé
deseneriza e o inversor de baixa continua ligado
garantindo fornecimento contínuo de energia à casa

28. MÁQUINA DE ESTADOS DOS INVERSORES

ESTADOS:
┌─────────────────────────────────────────────────────┐
│ BAIXA_NORMAL → corrente > limite → TRANSFERINDO_ALTA│
│                proteção detectada → PROTECAO_BAIXA  │
│                                                     │
│ TRANSFERINDO_ALTA → liga alta → aguarda status      │
│                  → comuta rede → ALTA_NORMAL        │
│                  → falha status → BAIXA_NORMAL      │
│                                                     │
│ ALTA_NORMAL → 1 min + corrente baixa→TRANSFER_BAIXA │
│             → proteção → ERRO                       │
│                                                     │
│ TRANSFERINDO_BAIXA → descomuta → desliga alta       │
│                    → BAIXA_NORMAL                   │
│                                                     │
│ PROTECAO_BAIXA → liga alta emergência imediata      │
│               → REINICIANDO_INV                     │
│                                                     │
│ REINICIANDO_INV → espera 3s → verifica status       │
│                → OK: ALTA_NORMAL                    │
│                → falha: desliga/liga → verifica     │
│                → OK: ALTA_NORMAL                    │
│                → falha: fica tentando               │
│                                                     │
│ ERRO → espera 10s → tenta recuperar tudo            │
└─────────────────────────────────────────────────────┘

Para que serve:
Garante transições seguras e controladas
Protege os inversores de chaveamentos perigosos
Recuperação automática de falhas sem intervenção

29. DETECÇÃO DE PROTEÇÃO POR STATUS LED

Formula:
PROTECAO = (INVERSOR_LIGADO == true)
E (SINAL_STATUS == LOW)
E (TEMPO_SEM_STATUS > DEBOUNCE_MS)

Onde DEBOUNCE_MS = 200ms (evita falsos positivos)

Para que serve:
O LED de status dos inversores indica funcionamento OK
Quando o inversor entra em sobrecarga/temperatura
o LED apaga → ESP32 detecta e age imediatamente
Debounce evita reação a piscadas momentâneas do LED

30. TEMPO MÍNIMO NO MODO ALTA (histerese temporal)

Formula:
PODE_RETORNAR = (TEMPO_DESDE_LIGOU_ALTA >= 60000ms)
E (CORRENTE < LIMITE_RETORNO)

Para que serve:
Evita chaveamento imediato de volta para baixa
Motores e cargas pesadas geram picos ao ligar
Esperar 1 minuto garante que a carga se estabilizou
Só então verifica se pode retornar ao inversor baixa

31. PROCESSO DE TRANSFERÊNCIA SEGURA

Transferência Baixa → Alta:
1. Liga inversor alta
2. Aguarda status OK (até 3 segundos)
3. Se OK: comuta relé de rede alta
4. Se falha: desliga alta, permanece na baixa

Transferência Alta → Baixa:
1. Descomuta relé de rede alta (volta para inversor baixa)
2. Aguarda 200ms (estabilização)
3. Desliga inversor alta

Para que serve:
Sequência segura evita momentos sem energia na casa
Sempre garante que há fonte ativa antes de desligar outra
A rede de baixa NUNCA é desligada

32. RECUPERAÇÃO AUTOMÁTICA DE FALHAS

Sequência após proteção do Inversor Baixa:
1. Transfere rede alta para inversor alta (emergência)
2. Aguarda 3 segundos
3. Verifica se inversor baixa voltou sozinho
4. Se não voltou: ciclo de energia (desliga/liga)
5. Aguarda 3 segundos
6. Se voltou: continua no modo alta monitorando
7. Se não voltou: fica em modo alta indefinidamente

Sequência após proteção do Inversor Alta:
1. Estado ERRO ativado
2. Aguarda 10 segundos
3. Tenta retornar para inversor baixa
4. Se OK: retorna para operação normal
5. Se falha: repete em 10 segundos

Para que serve:
Sistema totalmente autônomo sem necessidade de
intervenção humana para recuperar de falhas comuns
como picos de corrente e proteções temporárias

FLUXO VISUAL COMPLETO DO SISTEMA INVERSORES:

BATERIA/PAINEL
│
┌────┴────┐
│         │
▼         ▼
INV.     INV.
BAIXA    ALTA
│   500W    3000W
│         │
├─────────┤ ◄── RELÉ REDE ALTA
│         │     (comutador)
▼         ▼
REDE      REDE
BAIXA     ALTA
(fixa)   (comutada)
│         │
└────┬────┘
│
CASA
(toda carga)

 26. CORRENTE MÁXIMA DOS INVERSORES

 Formula:
 I_MAX_BAIXA (A) = POTENCIA_BAIXA (W) / TENSAO_NOMINAL (V)
 I_MAX_ALTA  (A) = POTENCIA_ALTA  (W) / TENSAO_NOMINAL (V)

 Exemplo configuração padrão:
 I_MAX_BAIXA = 500  / 12.8 =  39.06A
 I_MAX_ALTA  = 3000 / 12.8 = 234.37A

 Para que serve:
 Define a corrente máxima que cada inversor
 pode fornecer baseado na tensão nominal da bateria
 Automático: recalcula sempre que tensão nominal muda
 Reutilizado por: controlarInversores(), handleApiInversores()

 27. LIMITE DE CHAVEAMENTO DOS INVERSORES (com histerese)

 Formula:
 LIMITE_CHAV    = I_MAX_BAIXA × PERC_CHAVEAMENTO / 100
 LIMITE_RETORNO = LIMITE_CHAV × (1 - HISTERESE_INV / 100)

 Exemplo (70% chaveamento, 5% histerese):
 LIMITE_CHAV    = 39.06 × 0.70 = 27.34A
 LIMITE_RETORNO = 27.34 × 0.95 = 25.97A

 Para que serve:
 LIMITE_CHAV:    acima disso liga inversor alta
 LIMITE_RETORNO: abaixo disso desliga inversor alta
 Histerese evita chaveamento rápido (flutter)
 quando a corrente oscila em torno do limite
 Pré-calculado em recalcularInversor()
 Reutilizado a cada segundo em controlarInversores()
 sem recalcular → economiza processamento

 28. LÓGICA DOS RELÉS DOS INVERSORES

 RELÉ REDE ALTA:
 HIGH = rede de alta alimentada pelo Inversor Alta
 LOW  = rede de alta alimentada pelo Inversor Baixa

 RELÉ INVERSOR BAIXA (lógica INVERTIDA - FAIL SAFE):
 HIGH = inversor DESLIGADO (relé energizado abre circuito)
 LOW  = inversor LIGADO    (relé sem energia, passagem livre)

 RELÉ INVERSOR ALTA (lógica DIRETA):
 HIGH = inversor LIGADO    (relé energizado liga inversor)
 LOW  = inversor DESLIGADO (relé sem energia, desligado)

 Para que serve:
 Lógica invertida no relé de baixa é SAFETY FAIL-SAFE:
 Se o ESP32 reiniciar ou perder energia o relé
 deseneriza e o inversor de baixa continua ligado
 garantindo fornecimento contínuo de energia à casa
 sem nenhuma intervenção humana

 29. LÓGICA DOS RELÉS DA REDE ELÉTRICA (FAIL SAFE)

 RELÉ LÂMPADA EXTERNA:
 HIGH = lâmpada DESLIGADA (relé energizado corta energia)
 LOW  = lâmpada LIGADA    (relé sem energia, passagem livre)

 RELÉ LÂMPADA INTERNA:
 HIGH = lâmpada DESLIGADA (relé energizado corta energia)
 LOW  = lâmpada LIGADA    (relé sem energia, passagem livre)

 Para que serve:
 Mesmo FAIL SAFE das lâmpadas:
 Se o ESP32 reiniciar as lâmpadas permanecem LIGADAS
 O corte só acontece quando o ESP32 comanda ativamente
 Protege contra desligamento acidental por falha do sistema

 30. MÁQUINA DE ESTADOS DOS INVERSORES

 ESTADOS:
 ┌──────────────────────────────────────────────────────┐
 │ BAIXA_NORMAL    → corrente > LIMITE_CHAV             │
 │                 → TRANSFERINDO_ALTA                  │
 │                 → proteção detectada                 │
 │                 → PROTECAO_BAIXA                     │
 │                                                      │
 │ TRANSFERINDO_ALTA → liga alta → aguarda status OK    │
 │                 → comuta rede → ALTA_NORMAL          │
 │                 → falha status → BAIXA_NORMAL        │
 │                                                      │
 │ ALTA_NORMAL     → 1 min + corrente < LIMITE_RETORNO  │
 │                 → TRANSFERINDO_BAIXA                 │
 │                 → proteção → ERRO                    │
 │                                                      │
 │ TRANSFERINDO_BAIXA → descomuta → desliga alta        │
 │                 → BAIXA_NORMAL                       │
 │                                                      │
 │ PROTECAO_BAIXA  → liga alta emergência imediata      │
 │                 → REINICIANDO_INV                    │
 │                                                      │
 │ REINICIANDO_INV → espera 3s → verifica status        │
 │                 → OK: ALTA_NORMAL                    │
 │                 → falha: ciclo energia → verifica    │
 │                 → OK: ALTA_NORMAL                    │
 │                 → falha: fica tentando               │
 │                                                      │
 │ ERRO            → espera 10s → tenta recuperar tudo  │
 └──────────────────────────────────────────────────────┘

 Para que serve:
 Garante transições seguras e controladas
 Protege os inversores de chaveamentos perigosos
 Recuperação automática de falhas sem intervenção humana

 31. DETECÇÃO DE PROTEÇÃO POR STATUS LED DOS INVERSORES

 Formula:
 PROTECAO = (INVERSOR_LIGADO == true)
 E (SINAL_STATUS    == LOW)
 E (TEMPO_SEM_SINAL >  DEBOUNCE_MS)

 Onde DEBOUNCE_MS = 200ms

 Para que serve:
 O LED de status do inversor indica funcionamento OK
 Quando o inversor entra em sobrecarga ou temperatura
 o LED apaga → ESP32 detecta e age imediatamente
 Debounce de 200ms evita reação a piscadas momentâneas
 Quando inversor está logicamente desligado o ESP32
 ignora o sinal de status pois sabe que está desligado
 e não gera falso alarme

 32. TEMPO MÍNIMO NO MODO ALTA (histerese temporal)

 Formula:
 PODE_RETORNAR = (TEMPO_DESDE_LIGOU_ALTA >= 60000ms)
 E (CORRENTE_CASA < LIMITE_RETORNO)

 Para que serve:
 Evita chaveamento imediato de volta para baixa
 Motores e cargas pesadas geram picos ao ligar
 Esperar 1 minuto garante que a carga se estabilizou
 Só então verifica se pode retornar ao inversor baixa

 33. PROCESSO DE TRANSFERÊNCIA SEGURA ENTRE INVERSORES

 Transferência Baixa → Alta:
 1. Liga inversor alta
 2. Aguarda status OK (até 3 segundos)
 3. Se OK:    comuta relé de rede alta → casa usa inv.alta
 4. Se falha: desliga inv.alta → permanece na baixa

 Transferência Alta → Baixa:
 1. Descomuta relé rede alta (casa volta para inv.baixa)
 2. Aguarda 200ms (estabilização)
 3. Desliga inversor alta

 Para que serve:
 Sequência segura evita momentos sem energia na casa
 Sempre garante fonte ativa antes de desligar outra
 A rede de baixa NUNCA é desligada durante transferência

 34. ENERGIA DISPONÍVEL (base dos controles de rede)

 Formula:
 WH_DISPONIVEL = AH_DISPONIVEIS × TENSAO_BATERIA_EST

 Onde:
 AH_DISPONIVEIS   → calculado pelo coulomb counting (eq.15)
 TENSAO_BATERIA_EST → calculada pela curva OCV/SoC (eq.18)

 Para que serve:
 Valor base usado por TODOS os controles da rede elétrica
 Calculado UMA VEZ por segundo em atualizarEstadoBateria()
 Reutilizado em controlarRede() e handleApiRede()
 sem recalcular → economiza RAM e processamento
 Representa energia real disponível para uso seguro

 35. LIMITES DE CORTE DA REDE ELÉTRICA (Wh mínimos)

 Formula:
 WH_MIN_INV_BAIXA = ENERGIA_DISPONIVEL × (PERC_INV_BAIXA / 100)
 WH_MIN_REDE_ALTA = ENERGIA_DISPONIVEL × (PERC_REDE_ALTA / 100)
 WH_MIN_LAMP_EXT  = ENERGIA_DISPONIVEL × (PERC_LAMP_EXT  / 100)
 WH_MIN_LAMP_INT  = ENERGIA_DISPONIVEL × (PERC_LAMP_INT  / 100)

 Onde ENERGIA_DISPONIVEL = pré-calculado em carregarConfigBateria()
 = TENSAO_NOMINAL × AMPERAGEM × PERC_DESCARGA_SEGURA

 Exemplo (bateria 12.8V 100Ah 80% descarga, padrões):
 ENERGIA_DISPONIVEL = 12.8 × 100 × 0.80 = 1024 Wh

 WH_MIN_INV_BAIXA = 1024 × 0.10 = 102.4 Wh  (10%)
 WH_MIN_REDE_ALTA = 1024 × 0.30 = 307.2 Wh  (30%)
 WH_MIN_LAMP_EXT  = 1024 × 0.25 = 256.0 Wh  (25%)
 WH_MIN_LAMP_INT  = 1024 × 0.00 =   0.0 Wh  ( 0%)

 Para que serve:
 Define em Wh absolutos quando cada carga deve ser cortada
 Mostra ao usuário o valor real necessário ao lado do %
 Reutiliza ENERGIA_DISPONIVEL já calculada → zero custo RAM

 36. HISTERESE DE RELIGAMENTO DA REDE ELÉTRICA

 Formula:
 WH_RELIGAR_INV_BAIXA = ENERGIA_DISPONIVEL × ((PERC_INV_BAIXA + 2) / 100)
 WH_RELIGAR_REDE_ALTA = ENERGIA_DISPONIVEL × ((PERC_REDE_ALTA + 2) / 100)
 WH_RELIGAR_LAMP_EXT  = ENERGIA_DISPONIVEL × ((PERC_LAMP_EXT  + 2) / 100)
 WH_RELIGAR_LAMP_INT  = ENERGIA_DISPONIVEL × ((PERC_LAMP_INT  + 2) / 100)

 Onde 2 = HISTERESE_REDE_PERC (constante fixa)

 Exemplo com padrões:
 Corte  inv.baixa: < 102.4 Wh (10%)
 Religar inv.baixa: > 122.9 Wh (12%) ← 2% acima do corte

 Corte  rede alta: < 307.2 Wh (30%)
 Religar rede alta: > 327.7 Wh (32%) ← 2% acima do corte

 Para que serve:
 Sem histerese o relé oscilaria rapidamente
 quando a energia está exatamente no limite:
 Ex sem histerese (RUIM):
 307.1 Wh → desliga rede alta
 307.3 Wh → liga rede alta
 307.0 Wh → desliga rede alta  (loop infinito)

 Com histerese (CORRETO):
 307.1 Wh → desliga rede alta
 327.8 Wh → liga rede alta  (só quando recuperou bem)

 37. CONFIRMAÇÃO TEMPORAL DE 1 MINUTO (anti-oscilação)

 Formula:
 CONFIRMAR_CORTE = (WH_DISPONIVEL <= WH_MIN_X)
 E  (TEMPO_ABAIXO   >= 60000ms)

 CANCELAR_CONTAGEM = (WH_DISPONIVEL > WH_MIN_X)
 em qualquer momento antes dos 60s

 Tempo restante para exibição:
 TEMPO_RESTANTE = MAX(0, 60000 - (AGORA - TS_INICIO)) / 1000

 Para que serve:
 Evita corte por queda momentânea de energia
 Ex: nuvem passa na frente do painel por 30 segundos
 Sem confirmação: desliga rede desnecessariamente
 Com confirmação: espera 60s → nuvem passou → cancela
 Protege os relés de chaveamentos desnecessários
 Aumenta vida útil dos relés e dos inversores

 38. SEQUÊNCIA DE CORTES POR PRIORIDADE

 Ordem de desligamento (maior para menor prioridade):
 ┌─────────────────────────────────────────────────────┐
 │ 1. Lâmpada Interna  → corte em 0%  (padrão)        │
 │    Último recurso, quando bateria está no limite     │
 │                                                     │
 │ 2. Lâmpada Externa  → corte em 25% (padrão)        │
 │    Iluminação externa é menos crítica               │
 │                                                     │
 │ 3. Rede Alta        → corte em 30% (padrão)        │
 │    Tomadas de alta potência desligadas              │
 │    Relé rede alta liga MAS inv.alta permanece OFF   │
 │    Tomadas ficam sem tensão                         │
 │                                                     │
 │ 4. Inversor Baixa   → corte em 10% (padrão)        │
 │    Toda rede baixa desligada                        │
 │    Corte mais crítico: casa sem energia             │
 └─────────────────────────────────────────────────────┘

 Validação obrigatória na configuração:
 PERC_REDE_ALTA >= PERC_LAMP_EXT >= PERC_LAMP_INT
 PERC_INV_BAIXA <  PERC_LAMP_INT  (corte mais crítico)

 Para que serve:
 Desliga cargas menos essenciais primeiro
 Preserva ao máximo a energia para cargas críticas
 A iluminação interna é a última a ser desligada
 pois a segurança dos moradores é prioridade

 39. PROTEÇÃO DA BATERIA POR CAMADAS

 Camada 1 - Percentual do usuário (rede elétrica):
 Corta cargas progressivamente conforme configurado
 A bateria ainda tem energia de proteção restante

 Camada 2 - Percentual de descarga segura (bateria):
 ENERGIA_DISPONIVEL = CAPACIDADE_TOTAL × PERC_DESCARGA
 Os 0% da rede = X% da carga física real da bateria

 Camada 3 - BKP do controlador de carga:
 Sinal físico do controlador quando tensão crítica
 ESP32 recalibra SoC para valor crítico imediatamente

 Exemplo com bateria 100Ah 80% descarga segura:
 Capacidade física total:  100Ah = 1280 Wh
 Energia disponível (80%):  80Ah = 1024 Wh (base dos %)
 Energia de proteção (20%): 20Ah =  256 Wh (nunca usada)

 Quando rede desliga a 10% de 1024 Wh = 102 Wh
 A bateria ainda tem: 102 + 256 = 358 Wh físicos
 = ainda tem 27.9% da carga física real
 A bateria NUNCA chega a 0% físico → vida útil preservada

 40. REUTILIZAÇÃO DE CÁLCULOS (economia RAM e CPU)

 Sequência de execução no loop (1x por segundo):

 lerSensores()
 └─ Calcula: tensaoBateriaEst         ← eq.18 (OCV/SoC)
 └─ Calcula: correnteCasa             ← eq.6
 └─ Calcula: correntePainel           ← eq.6
 └─ Calcula: potenciaCasa             ← eq.19
 └─ Calcula: potenciaPainel           ← eq.19

 atualizarEstadoBateria()
 └─ Reutiliza: tensaoBateriaEst       ← já calculado
 └─ Calcula:   ahDisponiveis          ← eq.15 (coulomb)
 └─ Calcula:   whPainel, whCasa       ← eq.20

 controlarInversores()
 └─ Reutiliza: correnteCasa           ← já calculado
 └─ Reutiliza: limiteChaveamento      ← pré-calculado
 └─ ZERO cálculos novos

 controlarRede()
 └─ Reutiliza: ahDisponiveis          ← já calculado
 └─ Reutiliza: tensaoBateriaEst       ← já calculado
 └─ Reutiliza: energiaDisponivel      ← pré-calculado
 └─ Calcula:   whDisponivel = ah × V  ← 1 multiplicação
 └─ Calcula:   whMin×4 = eD × perc×4 ← 4 multiplicações
 └─ TOTAL: apenas 5 operações simples por segundo

 Para que serve:
 Sistema com 4 módulos de controle executando
 em apenas 5 operações matemáticas por segundo
 Memória RAM: sem variáveis duplicadas
 CPU: sem funções trigonométricas ou divisões pesadas
 Resultado: ESP32 com >80% do tempo livre para
 atender requisições web sem atraso

 FLUXO VISUAL COMPLETO DO SISTEMA DE CONTROLE:

 ┌─────────────────────────────────────────────────────┐
 │           SENSORES (1x/segundo)                     │
 │  Painel→corrente  Casa→corrente  BKP→digital        │
 └──────────────────┬──────────────────────────────────┘
 │ reutiliza
 ┌──────────────────▼──────────────────────────────────┐
 │         BATERIA (coulomb counting)                  │
 │  ahDisponiveis  tensaoBateriaEst  SoC%  whDisponiveis│
 └──────────────────┬──────────────────────────────────┘
 │ reutiliza
 ┌───────────┴───────────┐
 │                       │
 ┌──────▼──────┐       ┌────────▼────────┐
 │  INVERSORES │       │  REDE ELÉTRICA  │
 │             │       │                 │
 │ correnteCasa│       │ whDisponiveis   │
 │ vs limite   │       │ vs whMin×4      │
 │             │       │ (1 min confirm) │
 │ 7 estados   │       │ 4 cargas        │
 │             │       │ histerese 2%    │
 └──────┬──────┘       └────────┬────────┘
 │                       │
 ┌──────▼───────────────────────▼────────┐
 │              RELÉS                    │
 │  RedeAlta  InvBaixa  InvAlta          │
 │  LampExt   LampInterna                │
 │  (5 relés controlados)                │
 └───────────────────────────────────────┘

 PINAGEM COMPLETA DO SISTEMA:

 ADC SENSORES:
 GPIO 34 → ACS758 50A  (corrente painel solar)
 GPIO 35 → ACS758 100A (corrente casa)
 GPIO 32 → Sinal BKP controlador de carga

 RELÉS INVERSORES:
 GPIO 25 → Relé Rede Alta   (HIGH=Inv.Alta  LOW=Inv.Baixa)
 GPIO 26 → Relé Inv. Baixa  (LOW=LIGADO     HIGH=DESLIGADO)
 GPIO 27 → Relé Inv. Alta   (HIGH=LIGADO    LOW=DESLIGADO)

 STATUS INVERSORES:
 GPIO 33 → LED Status Inv. Baixa (HIGH=OK)
 GPIO 36 → LED Status Inv. Alta  (HIGH=OK)

 RELÉS REDE ELÉTRICA:
 GPIO 14 → Relé Lamp. Externa (LOW=LIGADA  HIGH=DESLIGADA)
 GPIO 12 → Relé Lamp. Interna (LOW=LIGADA  HIGH=DESLIGADA)

 LED SISTEMA:
 GPIO  2 → LED Azul status (blink=AP  fixo=WiFi)
 GPIO  0 → Botão BOOT (10s = reset total)

 PÁGINAS WEB DO SISTEMA:

 /             → Painel principal (dados em tempo real)
 /bateria      → Config e status da bateria LiFePO4
 /inversores   → Config e controle dos inversores
 /rede         → Config e controle da rede elétrica
 /historico    → Histórico 24h com filtro por data
 /config       → WiFi, senha, backup/restore
 /login        → Autenticação
 /logout       → Encerrar sessão
 /setup-login  → Primeira configuração (credenciais)
 /setup-wifi   → Primeira configuração (rede)

 APIs JSON (atualização em tempo real):
 /api/dados      → sensores, bateria, contadores (2s)
 /api/inversores → estado inversores e relés (1.5s)
 /api/rede       → estado rede e contagens (2s)
 /logs           → logs do sistema (4s)

 NAMESPACES DAS PREFERENCES (flash):

 "rrfs"   → WiFi, credenciais, timezone
 "rrfsb"  → Estado da bateria (Ah, ciclos)
 "rrfsc"  → Contadores kWh totais
 "rrfsi"  → Config inversores (potências, % chaveamento)
 "rrfsis" → Estado inversores (chaveamentos, proteções)
 "rrfsr"  → Config rede elétrica (% de corte)
 "rrfsrs" → Estado rede (total de cortes por carga)

  4 1*. PREVISÃO DE GERAÇÃO SOLAR - OPEN-METEO API

  Sistema de estimativa de geração fotovoltaica com
  aprendizado automático e decisões de economia de energia.

  API UTILIZADA: Open-Meteo (gratuita, sem chave de acesso)
  URL:
  http://api.open-meteo.com/v1/forecast?
  latitude=LAT&longitude=LON
  &hourly=shortwave_radiation,cloudcover,
precipitation_probability
&forecast_days=2
&timezone=auto

Dados recebidos (48 valores cada = hoje + amanhã):
shortwave_radiation      → W/m² (radiação solar por hora)
cloudcover               → % cobertura de nuvens
precipitation_probability→ % probabilidade de chuva

Atualização: a cada 60 minutos (INTERVALO_PREVISAO)
Condição:    WiFi conectado + NTP sincronizado + configurado
Retry:       se falhar → tenta novamente em 5 minutos

────────────────────────────────────────────────────────────
42. PRECISÃO DAS COORDENADAS GEOGRÁFICAS

Formula de armazenamento:
Tipo C++: double (64 bits IEEE 754)
Precisão: 15-17 dígitos significativos

Exemplo de coordenada de alta precisão:
Latitude:  -22.917549777384778
Longitude: -47.017843958072520

float  (32 bits) → 7 dígitos → erro de ~11 metros  ✗
double (64 bits) → 15 dígitos→ erro de ~0.01mm     ✓

Armazenamento na Flash (Preferences não suporta double):
SALVAR:    prefs.putString("lat", String(lat, 15))
CARREGAR:  lat = prefs.getString("lat","0").toDouble()

Precisão preservada na flash: total (15 casas decimais)

Por que alta precisão importa:
A API Open-Meteo usa as coordenadas para calcular
ângulo solar, sombreamento local e irradiância exata
Diferença de 0.001° ≈ 111 metros de erro de posição
→ pode mudar o perfil de geração em ±5%

────────────────────────────────────────────────────────────
43. GERAÇÃO ESTIMADA BRUTA

Formula:
GER_BRUTA (Wh) = RADIACAO (W/m²)
× KWP_INSTALADO (kW)
× EFICIENCIA_PADRAO
× 1 hora

Onde:
RADIACAO         → dado da API Open-Meteo (W/m²)
KWP_INSTALADO    → configurado pelo usuário (kWp)
EFICIENCIA_PADRAO= 0.75 (constante)
1 hora           → cada registro da API = 1 hora

Componentes da eficiência 0.75:
┌─────────────────────────────────────────────┐
│ Perda inversor:       ~8%  → fator 0.92     │
│ Perda nos cabos:      ~3%  → fator 0.97     │
│ Perda temperatura:    ~7%  → fator 0.93     │
│ Perda sujeira/sombra: ~5%  → fator 0.95     │
│ Perda orientação:     ~5%  → fator 0.95     │
│ Total combinado:      ~25% → fator 0.75     │
└─────────────────────────────────────────────┘

Exemplo:
Radiação=800 W/m² × kWp=3.3 × 0.75 = 1980 Wh bruto

Para que serve:
Base do cálculo antes da calibração pelo sistema real
Valor teórico ajustado pelo ângulo solar e condições
da localização específica configurada pelo usuário

────────────────────────────────────────────────────────────
44. FATOR DE CALIBRAÇÃO HORÁRIO (aprendizado automático)

Formula do fator medido:
FATOR_MEDIDO = GER_REAL_ACUMULADA (Wh)
/ GER_EST_ACUMULADA  (Wh)

Limites de segurança:
FATOR_MEDIDO = constrain(FATOR_MEDIDO, 0.20, 1.30)

0.20 → sistema real gera 20% do estimado (mínimo)
1.30 → sistema real gera 130% do estimado (máximo)

Quando calibra:
Condição 1: correntePainel > 0.5A   (painel gerando)
Condição 2: radiacaoAPI    > 50 W/m²(API tem dado válido)
Condição 3: acumulou       >= 10 min (amostra suficiente)

Acumulação por minuto (a cada 1 segundo no loop):
WH_REAL_MIN = (correntePainel × tensaoBateriaEst) / 60
WH_EST_MIN  = geracaoEstHora[idxAtual] / 60

Reutiliza:
correntePainel    → já calculado em lerSensores()
tensaoBateriaEst  → já calculado em lerSensores()
geracaoEstHora[]  → já calculado em calcularGeracaoEstimada()
ZERO leituras ADC extras

────────────────────────────────────────────────────────────
45. MÉDIA MÓVEL PONDERADA DO FATOR (persistência do aprendizado)

Formula:
N_AMOSTRAS = amostrasHorario[hora]  (contador histórico)

SE N_AMOSTRAS > 10:
PESO_HIST = 0.85  (sistema já aprendeu bem)
PESO_NOVO = 0.15
SENÃO:
PESO_HIST = 0.70  (ainda aprendendo)
PESO_NOVO = 0.30

FATOR_NOVO = (FATOR_ANTIGO × PESO_HIST)
+ (FATOR_MEDIDO × PESO_NOVO)

FATOR_FINAL = constrain(FATOR_NOVO, 0.20, 1.30)

Por que pesos diferentes conforme amostras:
Poucas amostras  → confia mais no dado novo (0.30)
Muitas amostras  → confia mais no histórico (0.15)
→ Aprendizado rápido no início, estável depois

Exemplo de evolução do fator:
Dia 1:  fator=1.00 (sem calibração)
Dia 2:  fator=0.72 (aprendeu: painel sombreado de manhã)
Dia 7:  fator=0.68 (estabilizou no valor real)
Dia 30: fator=0.67 (muito estável, erro < 1%)

────────────────────────────────────────────────────────────
46. GERAÇÃO ESTIMADA CALIBRADA POR HORA

Formula:
GER_CALIBRADA (Wh) = GER_BRUTA (Wh)
× FATOR_HORARIO[hora_do_dia]

Limite máximo de segurança:
MAX_POSSIVEL = KWP_INSTALADO × 1000 × 1.10
GER_CALIBRADA = min(GER_CALIBRADA, MAX_POSSIVEL)

Onde FATOR_HORARIO[0..23] = fator aprendido
para cada hora específica do dia

Por que fator por hora e não fator global:
08h: sol baixo no horizonte → fator=0.45 (sombra, ângulo)
12h: sol no zênite          → fator=0.85 (melhor ângulo)
15h: sol inclinando         → fator=0.70 (ângulo médio)

Um único fator global erraria sistematicamente
nas horas de manhã e tarde → fator por hora é
3x mais preciso que fator único global

────────────────────────────────────────────────────────────
47. TOTAL RESTANTE HOJE E TOTAL AMANHÃ

Formula:
HORA_ATUAL = hora do sistema (0-23)
OFFSET_API = índice [0] da API = hora atual

TOTAL_RESTANTE_HOJE (Wh) =
Σ GER_CALIBRADA[i]
para i de 0 até (23 - HORA_ATUAL)

TOTAL_AMANHA (Wh) =
Σ GER_CALIBRADA[i]
para i de (24 - HORA_ATUAL) até (47 - HORA_ATUAL)

Exemplo às 14h com kWp=3.3:
Restante hoje = soma das horas 14h até 18h = 4.2 kWh
Amanhã total  = soma das horas 00h até 23h = 9.8 kWh

Para que serve:
Base para as decisões automáticas de economia
Reutilizado em tomarDecisoesPrev() sem recálculo

────────────────────────────────────────────────────────────
48. TAXA DE ACERTO DO MODELO (precisão da estimativa)

Formula:
ERRO_PERC = |FATOR_MEDIDO - 1.0| × 100

ACERTO_ESTA = constrain(100 - ERRO_PERC, 0, 100)

SE taxaAcerto < 1.0:
taxaAcerto = ACERTO_ESTA   (primeira medição)
SENÃO:
taxaAcerto = taxaAcerto × 0.90
+ ACERTO_ESTA  × 0.10

Interpretação:
100% → estimativa = geração real exata
85% → estimativa erra em média 15%
70% → estimativa erra em média 30%

Exemplos reais esperados:
Dia limpo de verão:  acerto ~90%
Dia parcialmente nublado: ~75%
Dia encoberto:            ~55%
Média anual típica:       ~75-85%

────────────────────────────────────────────────────────────
49. CONSUMO MÉDIO DIÁRIO (base das decisões automáticas)

Formula (média ponderada dos últimos 7 dias):
Para cada dia D (0=hoje, 6=mais antigo):
WH_DIA[D]   = Σ historico[].whCasa / 10.0
PESO_DIA[D] = 7 - D

CONSUMO_MEDIO = Σ(WH_DIA[D] × PESO_DIA[D])
/ Σ(PESO_DIA[D])

Peso por dia:
Hoje (D=0)      → peso 7  (mais relevante)
Ontem (D=1)     → peso 6
2 dias atrás    → peso 5
...
6 dias atrás    → peso 1  (menos relevante)

Requisito mínimo: dia com pelo menos 10 registros
Reutiliza: historico[] (já existente no sistema)
Calculado: 1x por hora (junto com busca da API)

Por que ponderado e não simples:
Padrão de consumo muda: fim de semana vs semana
Dias recentes refletem melhor o comportamento atual
Peso maior para dias recentes = mais preciso

────────────────────────────────────────────────────────────
50. DECISÃO 1: DESLIGAR LÂMPADA EXTERNA NOTURNA

Condição (TODAS devem ser verdadeiras):
A) HORA >= 22h OU HORA < 6h       (período noturno)
B) geracaoAmanha < consumoMedio × 0.60  (previsão ruim)
C) SoC < 35%                      (bateria média-baixa)
D) bloqLampExt == false            (não está bloqueada)

Ação:
bloqLampExt         = true
lampExtFoiDesligada = true
controlarRede() detecta bloqLampExt=true → desliga

Desbloqueio automático:
HORA >= 6h E HORA < 10h E lampExtFoiDesligada=true
→ bloqLampExt = false
→ controle volta ao modo normal (energia/horário/painel)

Threshold 0.60:
geracaoAmanha < consumo × 60% → dia ruim previsto
Ex: consumo=3000Wh → bloqueia se prev < 1800Wh

Por que 35% SoC:
Bateria com mais de 35% aguenta a noite sem risco
Abaixo de 35% + geração ruim amanhã = risco real
Combinação de critérios evita decisões erradas

────────────────────────────────────────────────────────────
51. DECISÃO 2: DESLIGAR REDE ALTA APÓS MEIA-NOITE

Condição (qualquer uma):
A) HORA >= 0h E HORA < 6h         (após meia-noite)
E  (geracaoAmanha < consumoMedio × 0.50   ← OU
SoC < 30%)
E  bloqRedeAlta == false

Ação:
bloqRedeAlta         = true
redeAltaFoiDesligada = true
controlarRede() detecta → PIN_RELE_REDE_ALTA = HIGH
→ PIN_RELE_INV_ALTA  = LOW
→ Tomadas de alta potência sem energia
→ Inversor de alta potência desligado

Por que desligar rede alta à noite:
Geladeira: porta abre menos após meia-noite
→ geladeira fica fria naturalmente até 6h
→ economia de 30-50% do consumo noturno
→ bateria carrega mais durante o dia seguinte
Outros equipamentos de alta potência ficam off
→ reduz descarga da bateria no período crítico

Threshold 0.50:
Previsão < 50% do consumo médio = dia crítico
Ex: consumo=3000Wh → bloqueia se prev < 1500Wh

────────────────────────────────────────────────────────────
52. DECISÃO 3: DESLIGAR REDE ALTA ANTES DA MEIA-NOITE

Condição (TODAS devem ser verdadeiras):
A) HORA >= 21h E HORA < 24h       (tardinha)
B) geracaoAmanha < consumoMedio × 0.35  (crítico)
C) SoC < 25%                      (bateria baixa)
D) bloqRedeAlta == false

Ação: idêntica à Decisão 2

Por que threshold mais rigoroso (0.35 e 25%):
Corte antecipado = situação realmente crítica
Previsão < 35% + bateria < 25% = risco de ficar
sem energia antes do painel gerar amanhã
Dois critérios rigorosos evitam corte desnecessário

Exemplo de situação que ativa:
Bateria: 22% carregada às 21h
Previsão amanhã: 900Wh (dia muito nublado)
Consumo médio: 3000Wh
900 < 3000 × 0.35 = 1050 → condição B verdadeira
22 < 25 → condição C verdadeira
→ Sistema desliga rede alta preventivamente

────────────────────────────────────────────────────────────
53. DECISÃO 4: RELIGAR REDE ALTA AO AMANHECER

Condição (TODAS devem ser verdadeiras):
A) HORA >= 6h E HORA < 12h        (manhã)
B) redeAltaFoiDesligada == true   (foi desligada)
C) whDisponivel > whMinRedeAlta   (energia suficiente)

Onde:
whDisponivel = ahDisponiveis × tensaoBateriaEst
whMinRedeAlta = energiaDisponivel × (percRedeAlta/100)
+ HISTERESE_REDE_PERC(2%)

→ Reutiliza percRedeAlta JÁ CONFIGURADO pelo usuário
→ Reutiliza whDisponivel JÁ CALCULADO em controlarRede()
→ ZERO recálculos adicionais

Ação:
bloqRedeAlta         = false
redeAltaFoiDesligada = false
motivoRedeAlta[0]    = 0 (limpa mensagem)
controlarRede() volta ao controle normal de energia

Por que reutiliza percRedeAlta do usuário:
O usuário JÁ configurou o nível mínimo para rede alta
Não faz sentido criar outro threshold separado
Consistência: o mesmo nível que protege também religa
Zero parâmetros extras para o usuário configurar

────────────────────────────────────────────────────────────
54. INTEGRAÇÃO COM controlarRede() - BLOQUEIO OTIMIZADO

Arquitetura de integração:
┌─────────────────────────────────────────────────────┐
│ tomarDecisoesPrev()   → seta flags de bloqueio      │
│   bloqRedeAlta = true/false                         │
│   bloqLampExt  = true/false                         │
│                                                     │
│ controlarRede()       → verifica flags ANTES        │
│   SE bloqRedeAlta → desliga rede alta               │
│   SE bloqLampExt  → lampExtBloqEnergia=true         │
│   (reutiliza lógica existente de desligamento)      │
└─────────────────────────────────────────────────────┘

Por que não criar função separada:
controlarRede() já tem toda a lógica de relés
Duplicar código = risco de inconsistência
Flags de bloqueio = integração limpa e otimizada
Um único ponto de controle dos relés físicos

Prioridade de controle (maior → menor):
1. Previsão solar (bloqRedeAlta/bloqLampExt)
2. Proteção por energia (whDisp < whMin)
3. Controle normal (horário/painel para lamp.ext)

Reutilização de cálculos em controlarRede():
whDisp   → calculado 1x, usado por 4 controles + previsão
whMinRA  → calculado 1x, usado por controle + decisão 4
soc      → calculado 1x, usado em 4 decisões de previsão
ZERO recálculos duplicados entre os sistemas

────────────────────────────────────────────────────────────
55. PARSE MANUAL DO JSON OPEN-METEO

Formato da resposta da API:
{
"hourly": {
"time": ["2024-01-01T00:00",...],
"shortwave_radiation": [0,0,150,800,...],
"cloudcover": [20,30,45,...],
"precipitation_probability": [0,5,10,...]
}
}

Tamanho típico: 3KB a 5KB
Alocação: String na heap do ESP32 (não no stack)

Algoritmo de parse manual (função parseArrayFloat):
1. Localiza campo: indexOf("\"campo\":[")
2. Avança para o início do array
3. Loop: localiza próxima vírgula OU fechamento ']'
4. Extrai substring → converte com toFloat()
5. Repete até 48 valores ou fim do array

Por que sem ArduinoJson:
ArduinoJson para 48×3 arrays usa ~40KB RAM
Parse manual usa ~200 bytes de RAM adicional
ESP32 tem ~200KB livre → economiza 20% da RAM
Código do parse reutiliza padrão já existente
na função de importação de backup do sistema

Validação da resposta:
SE payload.length() < 100 → descarta
SE nRad < 24 → descarta (dados incompletos)
SE httpCode != 200 → incrementa errosConsecutivos

────────────────────────────────────────────────────────────
56. LÓGICA DE RELÉS NA DESATIVAÇÃO POR PREVISÃO

Quando previsão desliga rede alta:
PIN_RELE_REDE_ALTA  → HIGH (sem energia nas tomadas)
PIN_RELE_INV_ALTA   → LOW  (inversor desligado)
maquinaEstado       → ESTADO_BAIXA_NORMAL
estadoInv.modoAlta  → false

Por que esta combinação:
REDE_ALTA HIGH = relé energizado = corta energia tomadas
INV_ALTA  LOW  = inversor desligado = economiza bateria
Tomadas de alta potência ficam completamente sem energia
Inversor alto não consome em standby

Quando previsão religa rede alta (amanhecer):
bloqRedeAlta = false
controlarRede() detecta e chama setRedeAlta(false)
→ PIN_RELE_REDE_ALTA LOW = tomadas com energia
→ Inversor alta só liga SE necessário (lógica existente)
→ Máquina de estados retorna ao controle automático
NENHUMA ação manual necessária

Fail-safe de reinicialização:
estadoPrev.bloqRedeAlta = false no setup()
→ se ESP32 reiniciar durante bloqueio noturno
→ sistema volta ao estado normal automaticamente
→ rede alta volta ao controle por energia
→ usuário não fica sem tomadas por falha do ESP32

────────────────────────────────────────────────────────────
57. PERSISTÊNCIA DA CALIBRAÇÃO NA FLASH

Dados salvos (namespace "rrfsp"):
"lat"    → String (latitude double, 15 decimais)
"lon"    → String (longitude double, 15 decimais)
"kwp"    → float  (kWp instalado)
"p_ok"   → bool   (flag configurado)

Dados salvos (namespace "rrfspc"):
"f0".."f23"  → float × 24 (fator por hora do dia)
"a0".."a23"  → uint8 × 24 (amostras por hora)
"acerto"     → float       (taxa de acerto global %)
"consmed"    → float       (consumo médio dia Wh)

Política de escrita (economiza flash):
Condição 1: |fatorAtual - fatorAntesSalvar| > 0.02
(mudou mais de 2%)
Condição 2: millis() - tsUltimoSalvamento >= 3600000
(passaram 60 minutos)
Condição 3: forcado == true
(reset de calibração pelo usuário)

Dias nublados = painel não gera = zero calibração
= zero escritas na flash
Dias ensolarados = máximo 1 escrita por hora = 24/dia

Estimativa de vida da flash:
Flash ESP32: ~100.000 ciclos de escrita por célula
Pior caso: 24 escritas/dia × 365 dias = 8.760/ano
Vida estimada: 100.000 / 8.760 = 11+ anos

────────────────────────────────────────────────────────────
58. TIMER DE ATUALIZAÇÃO DA API (1 hora)

Variável de controle:
unsigned long tPrevisao = 0  (0 = força busca imediata)

Lógica no loop():
podeAtualizar = (!modoAP
&& WiFi.status() == WL_CONNECTED
&& ntpSincronizado
&& cfgPrev.configurado)

deveAtualizar = podeAtualizar
&& (tPrevisao == 0
|| agora - tPrevisao >= 3600000)

SE deveAtualizar:
tPrevisao = agora
buscarDadosOpenMeteo()
calcularGeracaoEstimada()
calcularConsumoMedio()
tomarDecisoesPrev()

Retry em caso de falha:
SE errosConsecutivos > 0:
tPrevisao = agora - 3600000 + 300000
→ próxima tentativa em 5 minutos
(não espera 1 hora inteira após falha)

Comportamento em modo AP:
podeAtualizar = false (sem internet)
Sistema usa dados da última busca válida
Decisões baseadas em último dadosValidos

Após reconexão WiFi:
loop() detecta WiFi.status() == WL_CONNECTED
tPrevisao = 0 → força busca imediata
Sistema atualiza dados sem esperar 1 hora

────────────────────────────────────────────────────────────
59. REAVALIAÇÃO DAS DECISÕES (a cada 5 minutos)

Variável de controle:
static unsigned long tDecisao = 0

Lógica no loop():
SE dadosValidos && ntpSincronizado
&& agora - tDecisao >= 300000 (5 min):
tDecisao = agora
tomarDecisoesPrev()

Por que reavaliar sem buscar API:
Dados API válidos por 1 hora
SoC e whDisponivel MUDAM a cada segundo
Uma bateria pode:
Às 23h: SoC=38% → não bloqueia (acima de 35%)
Às 23h30: SoC=33% → bloqueia (caiu abaixo de 35%)
Sem reavaliação: perderia esta mudança por 60 min
Com reavaliação 5 min: detecta em até 5 minutos

Custo computacional:
tomarDecisoesPrev() = apenas comparações simples
Reutiliza: soc, whDisp, geracaoAmanha (já calculados)
Sem HTTP, sem parse, sem flash = custo ≈ zero

────────────────────────────────────────────────────────────
60. FLUXO COMPLETO DO SISTEMA DE PREVISÃO

┌──────────────────────────────────────────────────────────┐
│              SETUP (1x na inicialização)                 │
│  carregarConfigPrevisao()  → lat, lon, kwp da flash      │
│  carregarCalibracaoHoraria()→ fatores[24] da flash       │
│  tPrevisao = 0             → força busca imediata        │
└─────────────────────┬────────────────────────────────────┘
│
┌─────────────────────▼────────────────────────────────────┐
│              LOOP - A CADA 1 HORA                        │
│  buscarDadosOpenMeteo()    → HTTP GET Open-Meteo         │
│    parseArrayFloat()       → extrai 3 arrays × 48h      │
│  calcularGeracaoEstimada() → aplica fatores calibrados   │
│    eq.43 bruto × eq.46 calibrado = geracaoEstHora[48]   │
│  calcularConsumoMedio()    → média 7 dias ponderada      │
│    reutiliza historico[]   → eq.49                       │
│  tomarDecisoesPrev()       → avalia 4 decisões           │
│    reutiliza soc, whDisp   → já calculados               │
└─────────────────────┬────────────────────────────────────┘
│
┌─────────────────────▼────────────────────────────────────┐
│              LOOP - A CADA 1 SEGUNDO                     │
│  lerSensores()             → correntePainel, tensaoEst   │
│  atualizarEstadoBateria()  → ahDisponiveis, soc          │
│  calibrarFatorHorario()    → reutiliza corrente+tensão   │
│    acumula 10 min          → calcula fatorMedido eq.44   │
│    média móvel ponderada   → atualiza fatorHorario[hora] │
│    recalcula estimativas   → calcularGeracaoEstimada()   │
│    salva se mudou >2%      → salvarCalibracaoHoraria()   │
│  controlarInversores()     → lógica existente            │
│  controlarRede()           → verifica bloqRedeAlta       │
│                               verifica bloqLampExt       │
│                               reutiliza whDisp, soc      │
└─────────────────────┬────────────────────────────────────┘
│
┌─────────────────────▼────────────────────────────────────┐
│              LOOP - A CADA 5 MINUTOS                     │
│  tomarDecisoesPrev()       → reavalia com SoC atual      │
│    Decisão 1: lamp.ext noturna  (soc<35% + prev ruim)   │
│    Decisão 2: rede alta 00-06h  (soc<30% OU prev ruim)  │
│    Decisão 3: rede alta 21-24h  (soc<25% E prev crítico) │
│    Decisão 4: religar 06-12h    (soc > nível usuário)   │
└──────────────────────────────────────────────────────────┘

────────────────────────────────────────────────────────────
NAMESPACES DAS PREFERENCES (flash) - ATUALIZADO:

"rrfs"   → WiFi, credenciais, timezone
"rrfsb"  → Estado da bateria (Ah, ciclos)
"rrfsc"  → Contadores kWh totais
"rrfsi"  → Config inversores (potências, % chaveamento)
"rrfsis" → Estado inversores (chaveamentos, proteções)
"rrfsr"  → Config rede elétrica (% de corte)
"rrfsrs" → Estado rede (total de cortes por carga)
"rrfsp"  → Config previsão (lat, lon, kWp)        [NOVO]
"rrfspc" → Calibração horária (fatores[24], acerto)[NOVO]

PÁGINAS WEB - ATUALIZADO:

/             → Painel principal + resumo previsão [+]
/bateria      → Config e status da bateria LiFePO4
/inversores   → Config e controle dos inversores
/rede         → Config e controle da rede elétrica
/previsao     → Previsão solar + calibração        [NOVO]
/historico    → Histórico 24h com filtro por data
/config       → WiFi, senha, backup/restore
/login        → Autenticação
/logout       → Encerrar sessão
/setup-login  → Primeira configuração (credenciais)
/setup-wifi   → Primeira configuração (rede)

APIs JSON - ATUALIZADO:

/api/dados      → sensores+bateria+previsão flags  [+]
/api/inversores → estado inversores e relés
/api/rede       → estado rede e contagens
/api/previsao   → previsão completa + arrays 48h   [NOVO]
/logs           → logs do sistema

CONSTANTES NOVAS ADICIONADAS:

INTERVALO_PREVISAO    = 3600000ms (1 hora)
EFICIENCIA_PADRAO     = 0.75      (rendimento painel)
FATOR_CAL_MIN         = 0.20      (fator mínimo)
FATOR_CAL_MAX         = 1.30      (fator máximo)
PESO_HISTORICO        = 0.70      (média móvel)
PESO_NOVO             = 0.30      (média móvel)
MIN_RADIACAO_CALIBRAR = 50.0 W/m² (mínimo para calibrar)
MIN_CORRENTE_CALIBRAR = 0.5A      (mínimo painel)
HORAS_PREVISAO        = 48        (horas da API)
HORA_MEIA_NOITE       = 0         (decisão noturna)
HORA_AMANHECER        = 6         (decisão manhã)
HORA_ANTECIPAR_CORTE  = 21        (decisão antecipada)
DELTA_FATOR_SALVAR    = 0.02      (threshold flash)
HTTP_TIMEOUT_MS       = 15000ms   (timeout HTTP)
BUFFER_HTTP           = 6144      (buffer JSON 6KB)

*/
