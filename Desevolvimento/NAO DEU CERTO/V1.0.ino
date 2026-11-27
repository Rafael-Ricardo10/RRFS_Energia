/*
 * RRFS Energia V1.0
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

// ============================================================
// PINAGEM
// ============================================================
#define LED_AZUL          2
#define BOTAO_BOOT        0
#define PIN_SENSOR_PAINEL 34   // ACS758 50A  - ADC1_CH6
#define PIN_SENSOR_CASA   35   // ACS758 100A - ADC1_CH7
#define PIN_BKP_BATERIA   32   // Sinal relé controlador de carga (3.3V)

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
#define VERSAO              "V1.0"

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
String gerarPagina(const String& titulo, const String& conteudo, const String& scriptExtra = "") {
    String html = F("<!DOCTYPE html><html><head>"
                    "<meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>"
              "body{background:#0d0d0d;color:#e0e0e0;font-family:Arial,sans-serif;margin:0;padding:8px;}"
              ".card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:16px;"
              "max-width:560px;margin:0 auto;}"
              "h2{color:#4fc3f7;text-align:center;margin:0 0 16px 0;font-size:18px;}"
              "h3{color:#81d4fa;font-size:14px;margin:12px 0 8px 0;"
              "border-bottom:1px solid #333;padding-bottom:6px;}"
              "label{display:block;color:#90a4ae;font-size:12px;margin-top:10px;margin-bottom:3px;}"
              "input,select{width:100%;padding:9px;background:#111;border:1px solid #444;"
              "border-radius:4px;color:#e0e0e0;font-size:13px;box-sizing:border-box;}"
              "input:focus,select:focus{outline:none;border-color:#0277bd;}"
              "button{width:100%;padding:11px;margin-top:12px;background:#0277bd;color:#fff;"
              "border:none;border-radius:4px;font-size:14px;font-weight:bold;cursor:pointer;}"
              "button:hover{background:#039be5;}"
              ".btn-red{background:#c62828;}.btn-red:hover{background:#ef5350;}"
              ".btn-green{background:#2e7d32;}.btn-green:hover{background:#43a047;}"
              ".btn-orange{background:#ef6c00;}.btn-orange:hover{background:#f57c00;}"
              ".btn-gray{background:#546e7a;}.btn-gray:hover{background:#607d8b;}"
              ".msg-ok{background:#1b5e20;color:#a5d6a7;padding:10px;border-radius:4px;"
              "margin:8px 0;text-align:center;font-size:13px;}"
              ".msg-err{background:#b71c1c;color:#ef9a9a;padding:10px;border-radius:4px;"
              "margin:8px 0;text-align:center;font-size:13px;}"
              ".msg-warn{background:#e65100;color:#fff9c4;padding:10px;border-radius:4px;"
              "margin:8px 0;font-size:13px;}"
              ".info{background:#0d47a1;color:#90caf9;padding:10px;border-radius:4px;"
              "margin:8px 0;font-size:12px;line-height:1.7;}"
              ".stat-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:8px 0;}"
              ".stat-box{background:#111;border:1px solid #333;border-radius:6px;padding:10px;"
              "text-align:center;}"
              ".stat-label{color:#546e7a;font-size:10px;text-transform:uppercase;margin-bottom:4px;}"
              ".stat-val{color:#4fc3f7;font-size:18px;font-weight:bold;}"
              ".stat-unit{color:#78909c;font-size:10px;margin-top:2px;}"
              ".stat-ok{color:#66bb6a;}.stat-warn{color:#ffa726;}.stat-err{color:#ef5350;}"
              ".nav{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:12px;}"
              ".nav a{flex:1;min-width:80px;text-align:center;padding:8px 4px;"
              "background:#1e3a5f;color:#90caf9;border-radius:4px;text-decoration:none;"
              "font-size:12px;font-weight:bold;border:1px solid #1565c0;}"
              ".nav a:hover{background:#1565c0;}"
              ".nav a.ativo{background:#0277bd;color:#fff;}"
              ".log-titulo{color:#546e7a;font-size:10px;text-transform:uppercase;"
              "margin-top:16px;margin-bottom:4px;}"
              ".log-box{background:#000;border:1px solid #222;border-radius:4px;padding:8px;"
              "height:160px;overflow-y:auto;font-family:monospace;font-size:10px;color:#76ff03;}"
              ".log-linha{padding:2px 0;border-bottom:1px solid #0a0a0a;}"
              ".flex{display:flex;gap:8px;margin-top:12px;}.flex button{margin-top:0;}"
              ".hora{color:#546e7a;font-size:11px;text-align:right;margin-bottom:8px;}"
              ".ram-info{color:#37474f;font-size:10px;text-align:right;margin-top:4px;}"
              ".contador{font-family:monospace;color:#ffeb3b;font-size:14px;letter-spacing:2px;}"
              "</style></head><body><div class='card'>");

    html += "<h2>&#9889; RRFS Energia " VERSAO "</h2>";

    // Navegação (só mostra se logado)
    if (logado) {
        html += "<div class='nav'>";
        html += "<a href='/' " + String(titulo.indexOf("Painel")>=0?"class='ativo'":"") + ">&#127968; Painel</a>";
        html += "<a href='/bateria' " + String(titulo.indexOf("Bateria")>=0?"class='ativo'":"") + ">&#128267; Bateria</a>";
        html += "<a href='/historico' " + String(titulo.indexOf("Hist")>=0?"class='ativo'":"") + ">&#128202; Hist&oacute;rico</a>";
        html += "<a href='/config' " + String(titulo.indexOf("Config")>=0?"class='ativo'":"") + ">&#9881; Config</a>";
        html += "</div>";
    }

    html += "<div class='hora' id='dh'>--/--/---- --:--:--</div>";
    html += conteudo;
    html += "<div class='log-titulo'>&#128221; LOGS DO SISTEMA</div>";
    html += "<div class='log-box' id='logbox'><div style='color:#555'>Aguardando...</div></div>";
    html += "<div class='ram-info' id='raminfo'></div>";
    html += "</div>";

    // Script base
    html += F("<script>"
              "function pad(n){return n<10?'0'+n:n;}"
              "function atualizarRelogio(){"
              "var d=new Date();"
              "document.getElementById('dh').textContent="
              "pad(d.getDate())+'/'+pad(d.getMonth()+1)+'/'+d.getFullYear()+' '+"
              "pad(d.getHours())+':'+pad(d.getMinutes())+':'+pad(d.getSeconds());}"
              "setInterval(atualizarRelogio,1000);atualizarRelogio();"
              "function atualizarLogs(){"
              "var x=new XMLHttpRequest();"
              "x.onreadystatechange=function(){"
              "if(x.readyState==4&&x.status==200){"
              "try{var d=document.getElementById('logbox');"
              "var a=JSON.parse(x.responseText);"
              "var h='';"
              "for(var i=0;i<a.length;i++){"
              "h+='<div class=\"log-linha\">'+a[i]+'</div>';}"
              "d.innerHTML=h||'<div style=\"color:#555\">Sem logs</div>';"
              "}catch(e){}}};"
              "x.open('GET','/logs',true);x.send();}"
              "setInterval(atualizarLogs,4000);atualizarLogs();");

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
            tempoHoras = estadoBat.ahDisponiveis / fabsf(sensores.correnteBateria > 0.1f ? sensores.correnteBateria : sensores.correnteCasa);
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

    String ip = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();

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
    // Alertas
    snprintf(tempBuf, sizeof(tempBuf),
             "\"aPi\":%d,\"aCi\":%d,\"bkp\":%d,",
             sensores.alertaPainelInvertido ? 1 : 0,
             sensores.alertaCasaInvertida   ? 1 : 0,
             sensores.sinalBKP              ? 1 : 0);
    j += tempBuf;
    // Tempo estimado
    j += "\"tStr\":\"" + tempoStr + "\",";
    j += "\"tLbl\":\"" + tempoLabel + "\",";
    // Contadores (10 dígitos)
    snprintf(tempBuf, sizeof(tempBuf),
             "\"kPai\":%.4f,\"kCas\":%.4f,\"kCar\":%.4f,\"kDes\":%.4f,",
             contadores.kwhPainelTotal, contadores.kwhCasaTotal,
             contadores.kwhCargaTotal,  contadores.kwhDescargaTotal);
    j += tempBuf;
    // Sistema
    snprintf(tempBuf, sizeof(tempBuf),
             "\"ram\":%u,\"ip\":\"%s\",\"modo\":\"%s\",\"up\":%lu",
             ram, ip.c_str(),
             modoAP ? "AP" : "WiFi",
             millis() / 1000);
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

    // Alertas críticos
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
                "color:#546e7a;font-size:10px;margin-bottom:3px;'>";
    conteudo += "<span>Carga da Bateria</span><span id='socPct'>--%</span></div>";
    conteudo += "<div style='background:#111;border:1px solid #333;border-radius:4px;height:14px;'>";
    conteudo += "<div id='socBar' style='height:100%;border-radius:4px;width:0%;"
                "background:#66bb6a;transition:width 1s;'></div></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;"
                "color:#546e7a;font-size:10px;margin-top:3px;'>";
    conteudo += "<span>Cap. Total: <b id='capTot'>--</b> Wh</span>";
    conteudo += "<span>Dispon&iacute;vel: <b id='whDisp'>--</b> Wh</span></div>";
    conteudo += "</div>";

    // Tempo estimado + ciclos
    conteudo += "<div class='stat-grid'>";
    conteudo += "<div class='stat-box'><div class='stat-label' id='tLbl'>Tempo</div>"
                "<div class='stat-val' id='tStr' style='font-size:14px;'>--</div></div>";
    conteudo += "<div class='stat-box'><div class='stat-label'>Ciclos Bateria</div>"
                "<div class='stat-val' id='ciclos'>--</div></div>";
    conteudo += "</div>";

    // Contadores
    conteudo += "<h3>&#128200; Contadores Totais</h3>";
    conteudo += "<div style='background:#111;border:1px solid #333;border-radius:6px;"
                "padding:10px;font-size:11px;'>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#9728; Painel Gerado</span>"
                "<span class='contador' id='kPai'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#127968; Casa Consumido</span>"
                "<span class='contador' id='kCas'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;"
                "border-bottom:1px solid #1a1a1a;'>"
                "<span style='color:#78909c;'>&#128267;&#8679; Bat. Carregado</span>"
                "<span class='contador' id='kCar'>----------</span>"
                "<span style='color:#546e7a;font-size:10px;'> kWh</span></div>";
    conteudo += "<div style='display:flex;justify-content:space-between;padding:4px 0;'>"
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
    conteudo += "<button onclick='location.href=\"/bateria\"' class='btn-green' "
                "style='font-size:12px;'>&#128267; Bateria</button>";
    conteudo += "<button onclick='location.href=\"/historico\"' "
                "style='background:#1565c0;font-size:12px;'>&#128202; Hist&oacute;rico</button>";
    conteudo += "<button onclick='location.href=\"/config\"' class='btn-gray' "
                "style='font-size:12px;'>&#9881; Config</button>";
    conteudo += "<button onclick='location.href=\"/logout\"' class='btn-red' "
                "style='font-size:12px;'>&#128275; Sair</button>";
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
                "if(s>=60)return '#66bb6a';"
                "if(s>=30)return '#ffa726';"
                "return '#ef5350';}"
                "function atualizarDados(){"
                "var x=new XMLHttpRequest();"
                "x.onreadystatechange=function(){"
                "if(x.readyState==4&&x.status==200){"
                "try{var d=JSON.parse(x.responseText);");

    script += F("// Painel"
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
                "document.getElementById('wB').textContent=fmt(Math.abs(d.wB),1);"
                "document.getElementById('vB').textContent=fmt(d.vB,2);"
                // Estado bateria
                "var stEl=document.getElementById('stB');"
                "var stUEl=document.getElementById('stBu');"
                "if(d.car){"
                "stEl.textContent='Carregando';"
                "stEl.className='stat-val stat-ok';}"
                "else{"
                "stEl.textContent='Descarregando';"
                "stEl.className='stat-val stat-err';}"
                // SoC
                "var soc=d.soc;"
                "document.getElementById('socPct').textContent=fmt(soc,1)+'%';"
                "var bar=document.getElementById('socBar');"
                "bar.style.width=soc+'%';"
                "bar.style.background=corSoc(soc);"
                // Capacidade e energia disponível
                "document.getElementById('capTot').textContent=fmt(capTotal,0);"
                "document.getElementById('whDisp').textContent=fmt(d.whD,0);"
                // Tempo estimado
                "document.getElementById('tStr').textContent=d.tStr||'--';"
                "document.getElementById('tLbl').textContent=d.tLbl||'Tempo';"
                // Ciclos
                "document.getElementById('ciclos').textContent=fmt(d.cic,2);"
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
                "function p(n){return n<10?'0'+n:n;}"
                "document.getElementById('upVal').textContent=uh+'h'+p(um)+'m'+p(us)+'s';");

    // Alertas
    script += F("var al='';"
                "if(d.aPi)al+='<div class=\"msg-err\">&#9888; ALERTA: Sensor do Painel INVERTIDO! Verifique os fios do ACS758.</div>';"
                "if(d.aCi)al+='<div class=\"msg-err\">&#9888; ALERTA: Sensor da Casa INVERTIDO! Verifique os fios do ACS758.</div>';"
                "if(d.bkp)al+='<div class=\"msg-warn\">&#128308; BKP ATIVO: Bateria em nivel critico!</div>';"
                "if(d.soc<20&&!d.car)al+='<div class=\"msg-warn\">&#9889; Bateria abaixo de 20%!</div>';"
                "document.getElementById('alertas').innerHTML=al;");

    script += F("}catch(e){}"
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
    "placeholder='{\"versao\":\"V1.0\",...}'></textarea>";
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
    server.on("/",            handleRoot);
    server.on("/login",       handleLogin);
    server.on("/logout",      handleLogout);
    server.on("/setup-login", handleSetupLogin);
    server.on("/setup-wifi",  handleSetupWifi);
    server.on("/bateria",     handleBateria);
    server.on("/historico",   handleHistorico);
    server.on("/config",      handleConfig);
    server.on("/logs",        handleLogs);
    server.on("/api/dados",   handleApiDados);
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

    // Configura ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // 0-3.3V

    adicionarLog("RRFS Energia " VERSAO " iniciando...");
    logRAMSeNecessario();

    // Carrega configurações salvas
    if (existeBateriaConfig()) {
        carregarConfigBateria();
        carregarEstadoBateria();
        snprintf(tempBuf, sizeof(tempBuf),
                 "Bat: %.1fV %.0fAh carregada: %s",
                 cfgBat.tensaoNominal, cfgBat.amperagem,
                 estadoBat.carregada100 ? "sim" : "nao");
        adicionarLog(tempBuf);
    } else {
        // Valores padrão enquanto não configurado
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

    carregarContadores();
    snprintf(tempBuf, sizeof(tempBuf),
             "Contadores: P=%.2fkWh C=%.2fkWh",
             contadores.kwhPainelTotal, contadores.kwhCasaTotal);
    adicionarLog(tempBuf);

    // Calibra zero dos sensores na inicialização
    // (idealmente sem corrente - primeiros segundos)
    delay(500);
    calibrarZeroSensores();

    // Configura pino BKP
    pinMode(PIN_BKP_BATERIA, INPUT);

    // Rede
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
                // NTP só se conectou ao WiFi
                iniciarNTP();
            }
        }
    }

    configurarRotas();
    adicionarLog("Servidor web ATIVO");
    logRAMSeNecessario();

    // Timer inicial de leitura
    tLeitura   = millis();
    tHistorico = millis();
    tWifi      = millis();
    tRAMLog    = millis();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    unsigned long agora = millis();

    verificarReset();
    atualizarLED();

    if (modoAP) dnsServer.processNextRequest();
    server.handleClient();

    // ---- Leitura dos sensores a cada 1 segundo ----
    if (agora - tLeitura >= INTERVALO_LEITURA) {
        float delta = (agora - tLeitura) / 1000.0f;
        tLeitura = agora;

        lerSensores();

        if (existeBateriaConfig()) {
            atualizarEstadoBateria(delta);
        }
    }

    // ---- Salvar histórico a cada 60 segundos ----
    if (agora - tHistorico >= INTERVALO_HISTORICO) {
        tHistorico = agora;
        salvarHistorico();
        logRAMSeNecessario();
    }

    // ---- Verificar WiFi a cada 30 segundos ----
    if (!modoAP && agora - tWifi >= INTERVALO_WIFI_CHK) {
        tWifi = agora;
        if (WiFi.status() != WL_CONNECTED) {
            adicionarLog("WiFi perdido - reconectando...");
            WiFi.reconnect();
            unsigned long t = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
                atualizarLED();
                delay(200);
            }
            if (WiFi.status() == WL_CONNECTED) {
                adicionarLog("WiFi reconectado!");
                ledFixo(true);
                // Re-sincroniza NTP
                iniciarNTP();
            } else {
                adicionarLog("Reconexao falhou - modo AP");
                server.close();
                iniciarAP();
                configurarRotas();
            }
            logRAMSeNecessario();
        }
    }

    // ---- Log RAM a cada 10 minutos ----
    if (agora - tRAMLog >= INTERVALO_RAM_LOG) {
        tRAMLog = agora;
        uint32_t ram = (uint32_t)esp_get_free_heap_size() / 1024;
        snprintf(tempBuf, sizeof(tempBuf), "RAM livre: %u KB", ram);
        adicionarLog(tempBuf);
        contLogs = 0; // reseta contador para não duplicar
    }

    delay(1);
}

/*

Documentação Completa - Equações do Sistema RRFS Energia V1.0
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

*/
