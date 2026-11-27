/*
 * RRFS Energia - Sistema Completo de Gerenciamento de Energia Solar
 * Versão v6.0 - Com sensores, controle de relés e interface web
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <time.h>
#include <Update.h>
#include <HTTPClient.h>

// ============================================================
// PINAGEM
// ============================================================
#define LED_AZUL              2
#define BOTAO_BOOT            0
#define PIN_SENSOR_PAINEL    34   // ACS758 50A  - ADC1_CH6
#define PIN_SENSOR_CASA      35   // ACS758 100A - ADC1_CH7
#define PIN_BKP_BATERIA      32   // Sinal relé controlador (futuro)
#define PIN_RELE_INV_ALTA    27   // LOW=ligado, HIGH=desligado
#define PIN_STATUS_INV_ALTA  36   // Sinal LED inversor (VP)
#define PIN_RELE_REDE_BAIXA  26   // LOW=ligado, HIGH=desligado
#define PIN_RELE_REDE_ALTA   25   // LOW=ligado, HIGH=desligado
#define PIN_RELE_LAMP_EXTERNA 14  // LOW=ligado, HIGH=desligado
#define PIN_RELE_LAMP_INTERNA 12  // LOW=ligado, HIGH=desligado

// ============================================================
// CONSTANTES DO SISTEMA
// ============================================================
#define MAX_LOGS          20
#define LOG_MAX_LEN      100
#define DNS_PORT          53
#define TEMPO_RESET_MS 10000

// Sensores ADC
#define ADC_SAMPLES        50    // Amostras para média
#define ADC_VREF         3300    // mV referência ESP32
#define ADC_BITS         4095    // 12 bits
#define SENSOR_VCC       5000    // mV alimentação sensores

// ACS758 50A (Painel): sensibilidade 40mV/A, offset 2500mV (com VCC 5V)
// Como VCC=5V e ESP32 lê até 3.3V, usar divisor resistivo
// Offset real lido pelo ESP32: 2500 * (3300/5000) = 1650mV
#define PAINEL_SENS_MV_A   40    // mV por Ampere
#define PAINEL_OFFSET_MV 2500    // Offset com VCC 5V (ponto zero)

// ACS758 100A (Casa): sensibilidade 20mV/A, offset 2500mV (com VCC 5V)
#define CASA_SENS_MV_A     20    // mV por Ampere
#define CASA_OFFSET_MV   2500    // Offset com VCC 5V (ponto zero)

// Buffer inversor (sinal LED)
#define INV_THRESHOLD_MV  1500   // >1500mV = inversor ligado
#define INV_BUFFER_MS     2000   // 2s buffer para confirmar estado

// Buffer desligamento relés
#define RELE_BUFFER_MS   60000   // 60s confirmação antes de desligar

// Intervalo leituras
#define INTERVALO_LEITURA_MS  1000  // 1s entre leituras
#define INTERVALO_CONTROLE_MS 5000  // 5s verificação controle

const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// Tempos do processo de detecção  de curto (ms)
#define T_ESPERA_QUEDA_INV   1500   // Tempo para inversor cair após desligar
#define T_ESPERA_STATUS_INV  2500   // Tempo para inversor subir após ligar
#define T_ENTRE_TESTES       1000   // Pausa entre etapas
#define T_MONITOR_INVERSOR   1000   // Janela de monitoramento após pico

// Quantas vezes tenta religar antes de iniciar diagnóstico de curto
#define MAX_TENTATIVAS_RELIGAR  3
// Tempo que aguarda status cair após desligar inversor
#define T_AGUARDA_QUEDA        1500
// Tempo que aguarda status subir após ligar inversor
#define T_AGUARDA_STATUS       3000
// Tempo sem status antes de agir (1 segundo conforme sua lógica)
#define T_STATUS_AUSENTE       1000
// Tempo entre desligar e religar o inversor
#define T_PAUSA_RELIGAR        2000

// Tamanho do buffer de média móvel dos sensores de CORRENTE
#define BUFFER_CORRENTE   10     // 10 amostras = ~10 segundos de estabilização
#define FILTRO_RUIDO_A    0.3f   // Abaixo disso considera zero

// Historico
#define HIST_DIAS        364   // 364 dias na flash
#define HIST_HORAS        24   // 24 horas por dia
#define HIST_KEY_VER       2   // Versão da estrutura

// Previção de geração de energia
#define PREV_HORAS        48    // 48 horas de previsão
#define PREV_URL_MAX     256    // Tamanho máximo da URL
#define PREV_INTERVALO  3600000UL // Atualiza a cada 1 hora (ms)

// ── Em RAM: apenas o dia atual + índices ──────────────────
// NÃO declara mais o array histDias[HIST_DIAS] na RAM!
// Cada dia é lido/escrito individualmente na flash

// Estados da máquina de detecção de curto
enum EstadoCurto {
    CURTO_NORMAL = 0,
    CURTO_RELIGANDO,        // Tentando religar antes do diagnóstico
    CURTO_TESTANDO_ALTA,
    CURTO_TESTANDO_BAIXA,
    CURTO_CONFIRMADO_ALTA,
    CURTO_CONFIRMADO_BAIXA,
    CURTO_AGUARDANDO_RESET
};

// Subestados do teste
enum SubEstadoCurto {
    SUB_IDLE = 0,
    SUB_INVERSOR_DESLIGADO,
    SUB_AGUARDANDO_QUEDA,
    SUB_AGUARDANDO_STATUS,
    SUB_INVERSOR_OK_ALTA_OFF,
    SUB_INVERSOR_OK_LIGAR_ALTA,
};

// ============================================================
// ESTRUTURAS
// ============================================================

// Configuração bateria
struct ConfigBateria {
    float tensaoNominal;    // V (ex: 12.8)
    float capacidadeAh;     // Ah (ex: 100)
    float descargaSegura;   // % (ex: 80)
    float energiaTotal;     // Wh calculado
    float energiaDisponivel;// Wh calculado
    bool  calibrada;        // Foi calibrada?
    float energiaAtual;     // Wh energia atual estimada
    float percentualAtual;  // % atual baseado em energiaDisponivel
};

// Configuração redes/cargas
struct ConfigRedes {
    float corteBaixa;       // % desligar rede baixa + inversor
    float corteAlta;        // % desligar rede alta
    float corteLampExt;     // % desligar lamp externa
    float corteLampInt;     // % desligar lamp interna
};

// Estado dos sensores
struct DadosSensores {
    float correntePainel;   // A (positivo = gerando)
    float correnteCasa;     // A (positivo = consumindo)
    float correnteBateria;  // A (positivo = carregando, negativo = descarregando)
    float potenciaPainel;   // W estimado
    float potenciaCasa;     // W estimado
    float potenciaBateria;  // W estimado
    int   sinalPainel_mV;   // mV bruto sensor painel
    int   sinalCasa_mV;     // mV bruto sensor casa
    int   sinalInversor_mV; // mV bruto sensor inversor
    bool  inversoreAtivo;   // Estado inversor (com buffer)
};

// Estado dos relés
struct EstadoReles {
    bool redeBaixa;         // true = ligada
    bool redesAlta;         // true = ligada
    bool lampExt;           // true = ligada
    bool lampInt;           // true = ligada
    bool inversor;          // true = ligado
};

// Timers buffer desligamento
struct TimersBuffer {
    unsigned long inicioBaixa;
    unsigned long inicioAlta;
    unsigned long inicioLampExt;
    unsigned long inicioLampInt;
    unsigned long inicioInversor;
    bool pendenteBaixa;
    bool pendenteAlta;
    bool pendenteLampExt;
    bool pendenteLampInt;
    // Buffer inversor
    unsigned long inversorUltimoHigh;
    bool          inversorEstadoReal;
};

struct GerenciadorCurto {
    EstadoCurto    estado;
    SubEstadoCurto subEstado;
    unsigned long  tInicio;
    bool           redeAltaCurto;
    bool           redeBaixaCurto;
    bool           monitorando;
    uint8_t        tentativasReligar; // Quantas vezes tentou religar
};

// ============================================================
// VARIÁVEIS DE CALIBRAÇÃO DOS SENSORES
// Salvas na EEPROM via Preferences
// ============================================================
struct CalibSensor {
    int   offsetMV;      // mV quando corrente = 0A (ponto zero real)
    float sensMvA;       // mV por Ampere real (após divisor)
    bool  invertido;     // Sinal invertido?
    bool  calibrado;     // Foi calibrado?
};

// Buffers de média móvel para corrente
struct BufferMediaMovel {
    float  amostras[BUFFER_CORRENTE];
    uint8_t indice;
    uint8_t total;       // Quantas amostras válidas temos
    bool    cheio;       // Buffer preenchido?
};

struct EstimativaTempo {
    bool   valido;
    int    horas;
    int    minutos;
    bool   carregando;   // true=carregando, false=descarregando
    float  taxaWh;       // Wh/h atual (positivo=carregando)
};

// Registro por hora
struct RegistroHora {
    float painelWh;     // Wh gerado pelo painel nesta hora
    float casaWh;       // Wh consumido pela casa nesta hora
    float batCargWh;    // Wh carregado na bateria nesta hora
    float batDescWh;    // Wh descarregado da bateria nesta hora
};

// Registro por dia
struct RegistroDia {
    uint16_t ano;
    uint8_t  mes;
    uint8_t  dia;
    bool     valido;                        // Dia tem dados?
    RegistroHora horas[HIST_HORAS];         // 24 horas
    // Totais do dia (calculados)
    float totalPainelWh;
    float totalCasaWh;
    float totalBatCargWh;
    float totalBatDescWh;
};

// Contadores globais (nunca zerados)
struct ContadoresGlobais {
    float painelKwh;    // Total gerado pelo painel (kWh)
    float casaKwh;      // Total consumido pela casa (kWh)
    float batCargKwh;   // Total carregado na bateria (kWh)
    float batDescKwh;   // Total descarregado da bateria (kWh)
};

// Acumuladores da hora atual (Wh)
struct AcumHora {
    float painelWh;
    float casaWh;
    float batCargWh;
    float batDescWh;
    uint8_t horaAtual;   // Hora sendo acumulada (0-23)
    uint8_t diaAtual;
    uint8_t mesAtual;
    uint16_t anoAtual;
    unsigned long ultimoAcum; // millis() do último acúmulo
};
AcumHora acum = {0, 0, 0, 0, 255, 0, 0, 0, 0};

// Configuração do local
struct ConfigLocal {
    double  latitude;       // Alta precisão: -23.667768108648623
    double  longitude;      // Alta precisão: -48.184803456419935
    float   painelKWp;      // kWp instalado (ex: 2.5)
    float   eficiencia;     // Eficiência do sistema (0.0-1.0)
    bool    configurado;    // Já foi configurado?
    // Calibração automática
    float   fatorCalib;     // Fator de correção aprendido
    uint8_t amostrasCalib;  // Quantas amostras de calibração temos
};

// Previsão por hora
struct PrevisaoHora {
    uint8_t  hora;             // 0-23
    uint8_t  dia;
    uint8_t  mes;
    uint16_t ano;
    float    radiacao;         // W/m² radiação direta normal
    float    radiacaoDifusa;   // W/m² radiação difusa
    float    coberturaNuvens;  // % cobertura de nuvens (0-100)
    float    temperatura;      // °C
    float    geracaoEstimada;  // Wh estimado de geração
    bool     valido;
};

// Resultado da previsão
struct ResultadoPrev {
    PrevisaoHora horas[PREV_HORAS];
    uint8_t      totalHoras;
    float        geracaoHojeRestante;  // Wh restante hoje
    float        geracaoAmanha;        // Wh total amanhã
    float        geracaoTotal48h;      // Wh total 48h
    unsigned long ultimaAtualizacao;   // millis()
    bool         valido;
    char         erro[64];             // Mensagem de erro se falhou
};

// Decisões automáticas do sistema
struct DecisaoSistema {
    bool  desligarRedeAltaNoite;    // Desligar rede alta à noite?
    bool  desligarLampExtNoite;     // Desligar lamp ext à noite?
    float horaDesligarRedeAlta;     // Hora prevista para desligar
    float horaLigarRedeAlta;        // Hora prevista para religar
    bool  alertaEnergiaInsuficiente;// Alerta de energia insuficiente?
    char  motivo[128];              // Motivo da decisão
};

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP   = false;
bool logado   = false;

unsigned long ultimoBlink = 0;
bool          ledState    = false;
bool          blinkAtivo  = true;

unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

char    logs[MAX_LOGS][LOG_MAX_LEN];
uint8_t totalLogs = 0;
uint8_t logIndex  = 0;
char    tempBuf[200];

// Dados do sistema
ConfigBateria  bateria  = {12.8f, 100.0f, 80.0f, 0, 0, false, 0, 0};
ConfigRedes    redes    = {10.0f, 30.0f, 50.0f, 0.0f};
DadosSensores  sensores = {0};
EstadoReles    estados  = {true, true, true, true, false};
TimersBuffer   timers   = {0};

unsigned long ultimaLeitura  = 0;
unsigned long ultimoControle = 0;

// Acumulador coulomb counting (para estimar energia da bateria)
unsigned long ultimoContagem = 0;

GerenciadorCurto curto = {
    CURTO_NORMAL, SUB_IDLE,
    0, false, false, false, 0
};

// Armazena credenciais WiFi globalmente para reconexão
String gWifiSSID = "";
String gWifiPASS = "";
String gWifiIP   = "";
String gWifiGW   = "";
String gWifiMASK = "";
String gWifiDNS  = "";

BufferMediaMovel bufPainel = {0};
BufferMediaMovel bufCasa   = {0};

CalibSensor calibPainel;
CalibSensor calibCasa;

// Apenas o dia atual fica em RAM para acesso rápido
RegistroDia histDiaAtual;       // Dia atual em RAM (~424 bytes)
RegistroDia histDiaTemp;        // Buffer temporário para leitura (~424 bytes)

// Índice e controle (pouquíssima RAM)
uint16_t histIndiceAtual = 0;   // Índice do dia atual (0-363)
uint16_t histTotalDias   = 0;   // Quantos dias válidos

// Buffer circular de dias
RegistroDia      histDias[HIST_DIAS];
ContadoresGlobais contadores     = {0, 0, 0, 0};

ConfigLocal      configLocal  = {0, 0, 0, 0.80f, false, 1.0f, 0};
ResultadoPrev    previsao     = {0};
DecisaoSistema   decisao      = {false, false, 0, 0, false, ""};

unsigned long ultimaPrev      = 0;  // Última busca meteorológica
bool          buscandoPrev    = false;

// ============================================================
// PROTÓTIPOS
// ============================================================
void calcularTotaisDia(RegistroDia& dia);
void salvarContadores();
void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void salvarHistorico();
void carregarHistorico();
void acumularEnergia();

EstimativaTempo calcularTempoRestante() {
    EstimativaTempo est = {false, 0, 0, false, 0};
    if (!bateria.calibrada) return est;
    if (bateria.energiaDisponivel <= 0) return est;

    float taxaWh = sensores.potenciaBateria;
    if (fabsf(taxaWh) < 2.0f) return est;

    est.taxaWh     = taxaWh;
    est.valido     = true;
    est.carregando = (taxaWh > 0);

    float horasRestantes = 0;
    if (est.carregando) {
        float energiaFaltando = bateria.energiaDisponivel - bateria.energiaAtual;
        if (energiaFaltando <= 0) { est.horas = 0; est.minutos = 0; return est; }
        horasRestantes = energiaFaltando / taxaWh;
    } else {
        float energiaUsavel = bateria.energiaAtual;
        if (energiaUsavel <= 0) { est.horas = 0; est.minutos = 0; return est; }
        horasRestantes = energiaUsavel / fabsf(taxaWh);
    }

    if (horasRestantes > 999) horasRestantes = 999;
    est.horas   = (int)horasRestantes;
    est.minutos = (int)((horasRestantes - est.horas) * 60);
    return est;
}

// ============================================================
// CALCULA TEMPO ATÉ UM DETERMINADO % DE CORTE
// Baseado na taxa atual de descarga
// ============================================================
String tempoAteCorte(float pctCorte, float taxaWhH) {
    // taxaWhH = negativo quando descarregando
    if (!bateria.calibrada) return "--";
    if (taxaWhH >= 0) return "--";           // Carregando, sem sentido
    if (bateria.energiaDisponivel <= 0) return "--";

    // Energia no ponto de corte
    float energiaCorte = bateria.energiaDisponivel * (pctCorte / 100.0f);

    // Energia disponível até o corte
    float energiaAteCorte = bateria.energiaAtual - energiaCorte;
    if (energiaAteCorte <= 0) return "Agora";

    // Horas até o corte
    float horas = energiaAteCorte / fabsf(taxaWhH);
    if (horas > 999) return ">999h";

    int h = (int)horas;
    int m = (int)((horas - h) * 60);

    char buf[20];
    if (h == 0) snprintf(buf, sizeof(buf), "%dm",   m);
    else        snprintf(buf, sizeof(buf), "%dh%02dm", h, m);
    return String(buf);
}

// ============================================================
// LOGS
// ============================================================
void adicionarLog(const char* msg) {
    struct tm ti;
    if (getLocalTime(&ti)) {
        // Hora real via NTP
        snprintf(logs[logIndex], LOG_MAX_LEN,
                 "[%02d:%02d:%02d] %s",
                 ti.tm_hour, ti.tm_min, ti.tm_sec, msg);
    } else {
        // Fallback: tempo desde o boot
        unsigned long s = millis() / 1000;
        snprintf(logs[logIndex], LOG_MAX_LEN,
                 "[%02lu:%02lu:%02lu] %s",
                 s / 3600, (s % 3600) / 60, s % 60, msg);
    }
    logIndex = (logIndex + 1) % MAX_LOGS;
    if (totalLogs < MAX_LOGS) totalLogs++;
}

void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf),
             "RAM: %.1f KB livres", (unsigned)esp_get_free_heap_size() / 1024.0f);
    adicionarLog(tempBuf);
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
void setupBotao() {
    pinMode(BOTAO_BOOT, INPUT_PULLUP);
}

void verificarReset() {
    bool pressionado = (digitalRead(BOTAO_BOOT) == LOW);
    if (pressionado && !botaoSegurando) {
        botaoSegurando     = true;
        botaoPressionadoEm = millis();
    }
    if (!pressionado) botaoSegurando = false;

    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET TOTAL por botao BOOT");
        for (int i = 0; i < 15; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(80);
            digitalWrite(LED_AZUL, LOW);  delay(80);
        }
        prefs.begin("rrfs", false);
        prefs.clear();
        prefs.end();
        delay(300);
        ESP.restart();
    }
}

// ============================================================
// RELÉS - SETUP E CONTROLE
// ============================================================
void setupReles() {
    // Todos os relés iniciam DESLIGADOS no hardware
    // O setup() vai ligar na sequência correta
    pinMode(PIN_RELE_INV_ALTA,     OUTPUT); digitalWrite(PIN_RELE_INV_ALTA,     HIGH);
    pinMode(PIN_RELE_REDE_BAIXA,   OUTPUT); digitalWrite(PIN_RELE_REDE_BAIXA,   HIGH);
    pinMode(PIN_RELE_REDE_ALTA,    OUTPUT); digitalWrite(PIN_RELE_REDE_ALTA,    HIGH);
    pinMode(PIN_RELE_LAMP_EXTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_EXTERNA, HIGH);
    pinMode(PIN_RELE_LAMP_INTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_INTERNA, HIGH);

    // Entradas
    pinMode(PIN_STATUS_INV_ALTA, INPUT);
    pinMode(PIN_BKP_BATERIA,     INPUT);

    // Estado inicial: tudo desligado
    // setup() vai ligar na ordem correta
    estados.redeBaixa = false;
    estados.redesAlta = false;
    estados.lampExt   = false;
    estados.lampInt   = false;
    estados.inversor  = false;

    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;

    adicionarLog("Reles iniciados: todos OFF");
}

// Liga/desliga relé com lógica invertida
void setRele(uint8_t pin, bool ligar) {
    digitalWrite(pin, ligar ? LOW : HIGH);
}

void ligarRedeBaixa()  { setRele(PIN_RELE_REDE_BAIXA,   true);  estados.redeBaixa = true;  }
void desligarRedeBaixa(){ setRele(PIN_RELE_REDE_BAIXA,  false); estados.redeBaixa = false; }
void ligarRedeAlta()   { setRele(PIN_RELE_REDE_ALTA,    true);  estados.redesAlta = true;  }
void desligarRedeAlta(){ setRele(PIN_RELE_REDE_ALTA,    false); estados.redesAlta = false; }
void ligarLampExt()    { setRele(PIN_RELE_LAMP_EXTERNA, true);  estados.lampExt   = true;  }
void desligarLampExt() { setRele(PIN_RELE_LAMP_EXTERNA, false); estados.lampExt   = false; }
void ligarLampInt()    { setRele(PIN_RELE_LAMP_INTERNA, true);  estados.lampInt   = true;  }
void desligarLampInt() { setRele(PIN_RELE_LAMP_INTERNA, false); estados.lampInt   = false; }
void ligarInversor()   { setRele(PIN_RELE_INV_ALTA,     true);  estados.inversor  = true;  }
void desligarInversor(){ setRele(PIN_RELE_INV_ALTA,     false); estados.inversor  = false; }

// Adiciona amostra ao buffer e retorna média
float adicionarAmostra(BufferMediaMovel& buf, float novaAmostra) {
    buf.amostras[buf.indice] = novaAmostra;
    buf.indice = (buf.indice + 1) % BUFFER_CORRENTE;
    if (buf.total < BUFFER_CORRENTE) buf.total++;
    if (buf.total == BUFFER_CORRENTE) buf.cheio = true;

    // Calcula média das amostras disponíveis
    float soma = 0;
    for (uint8_t i = 0; i < buf.total; i++) {
        soma += buf.amostras[i];
    }
    return soma / buf.total;
}

// ============================================================
// LEITURA ADC COM MÉDIA
// ============================================================
int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        soma += analogRead(pin);
        delayMicroseconds(100);
    }
    return (int)(soma / ADC_SAMPLES);
}

// Converte leitura ADC para mV (considerando que sinal vem de sensor 5V
// com divisor resistivo para 3.3V do ESP32)
// Se não houver divisor, calibrar offset
int adcParaMv(int adcVal) {
    // Escala: 0-4095 → 0-3300mV
    return (int)((long)adcVal * ADC_VREF / ADC_BITS);
}

// ============================================================
// LEITURA DOS SENSORES ACS758
// ============================================================
/*
 * Os sensores são alimentados em 5V mas o ESP32 lê até 3.3V.
 * É necessário um divisor resistivo (por ex: 10kΩ + 6.8kΩ) na saída do sensor.
 * Fator divisor: 3300/5000 = 0.66
 *
 * ACS758 50A (Painel):
 *   - Sensibilidade: 40mV/A (com VCC=5V)
 *   - Offset (0A): VCC/2 = 2500mV
 *   - Offset lido pelo ESP32 após divisor: ~1650mV
 *   - Corrente positiva = tensão ABAIXO de 1650mV (cabos conforme datasheet)
 *
 * ACS758 100A (Casa):
 *   - Sensibilidade: 20mV/A (com VCC=5V)
 *   - Offset (0A): VCC/2 = 2500mV
 *   - Offset lido pelo ESP32 após divisor: ~1650mV
 *
 * IMPORTANTE: Corrente negativa no painel = controlador consumindo (normal < 1A)
 *             Alertar se painel invertido (corrente negativa acima de 1A)
 */

// Offset real em mV que o ESP32 lê quando corrente = 0
// (deve ser calibrado/ajustado conforme hardware real)
// Com divisor 5V→3.3V: 2500 * 0.66 = 1650mV
#define OFFSET_LIDO_MV  1650

// ============================================================
// LEITURA CORRENTE PAINEL - COM BUFFER MÉDIA MÓVEL
// ============================================================
float lerCorrentePainel() {
    int   adc = lerADC(PIN_SENSOR_PAINEL);
    int   mv  = adcParaMv(adc);
    sensores.sinalPainel_mV = mv;

    // Corrente bruta usando calibração
    float correnteBruta = (float)(mv - calibPainel.offsetMV) / calibPainel.sensMvA;

    // Inverte sinal se necessário
    if (calibPainel.invertido) correnteBruta = -correnteBruta;

    // Filtro de ruído ANTES do buffer
    if (correnteBruta > -FILTRO_RUIDO_A && correnteBruta < FILTRO_RUIDO_A) {
        correnteBruta = 0.0f;
    }

    // Passa pelo buffer de média móvel
    float corrMediaMovel = adicionarAmostra(bufPainel, correnteBruta);

    // Filtro de ruído DEPOIS da média (elimina resíduo)
    if (corrMediaMovel > -FILTRO_RUIDO_A && corrMediaMovel < FILTRO_RUIDO_A) {
        corrMediaMovel = 0.0f;
    }

    return corrMediaMovel;
}

// ============================================================
// LEITURA CORRENTE CASA - COM BUFFER MÉDIA MÓVEL
// ============================================================
float lerCorrenteCasa() {
    int   adc = lerADC(PIN_SENSOR_CASA);
    int   mv  = adcParaMv(adc);
    sensores.sinalCasa_mV = mv;

    // Corrente bruta usando calibração
    float correnteBruta = (float)(mv - calibCasa.offsetMV) / calibCasa.sensMvA;

    // Inverte sinal se necessário
    if (calibCasa.invertido) correnteBruta = -correnteBruta;

    // Casa sempre positiva (consumo)
    // Filtro de ruído ANTES do buffer
    if (correnteBruta < FILTRO_RUIDO_A && correnteBruta > -FILTRO_RUIDO_A) {
        correnteBruta = 0.0f;
    }
    if (correnteBruta < 0.0f) correnteBruta = 0.0f;

    // Passa pelo buffer de média móvel
    float corrMediaMovel = adicionarAmostra(bufCasa, correnteBruta);

    // Filtro final
    if (corrMediaMovel < FILTRO_RUIDO_A) corrMediaMovel = 0.0f;

    return corrMediaMovel;
}

// ============================================================
// STATUS INVERSOR - CORRIGIDO SEM BUFFER QUANDO CHAVE DESLIGADA
// ============================================================
bool lerStatusInversor() {
    int adc = lerADC(PIN_STATUS_INV_ALTA);
    int mv  = adcParaMv(adc);
    sensores.sinalInversor_mV = mv;

    // Se a chave do inversor está DESLIGADA
    // o status NUNCA vai aparecer - retorna false imediatamente
    if (!estados.inversor) {
        timers.inversorEstadoReal = false;
        timers.inversorUltimoHigh = 0;
        return false;
    }

    // Chave LIGADA - monitora o sinal
    bool sinalHigh = (mv >= INV_THRESHOLD_MV);

    if (sinalHigh) {
        timers.inversorUltimoHigh = millis();
        timers.inversorEstadoReal = true;
    } else {
        // Sinal baixo com chave ligada:
        // Usa buffer de 2s para confirmar desligamento
        // (evita falso positivo por oscilação momentânea)
        if (timers.inversorEstadoReal &&
            millis() - timers.inversorUltimoHigh >= INV_BUFFER_MS) {
            timers.inversorEstadoReal = false;
            }
    }

    return timers.inversorEstadoReal;
}

// ============================================================
// MONITORAR INVERSOR - LÓGICA COMPLETA CORRIGIDA
//
// FLUXO EXATO:
//
// [NORMAL]
//   Chave ON + Status OK  → tudo normal, monitorando
//   Chave ON + Status OFF por 1s → inversor em proteção
//     → Desliga chave, aguarda 2s
//     → Liga chave, aguarda status por 3s
//       → Status voltou e ficou estável → era pico → NORMAL
//       → Status não voltou OU voltou e parou:
//           tentativas < MAX → tenta de novo
//           tentativas = MAX → inicia diagnóstico
//
// [DIAGNÓSTICO REDE ALTA]
//   Desliga rede alta, liga inversor
//   → Status OK e estável → rede baixa OK, liga rede alta
//     → Status cai → CURTO REDE ALTA confirmado
//     → Status OK → sistema normal
//   → Status não volta → passa para teste rede baixa
//
// [DIAGNÓSTICO REDE BAIXA]
//   Desliga rede baixa, liga inversor
//   → Status OK e estável → liga rede baixa
//     → Status cai → CURTO REDE BAIXA confirmado
//     → Status OK → sistema normal
//   → Status não volta → FALHA NO INVERSOR
// ============================================================

// Helper interno: desliga inversor e limpa buffer
void _desligarInversorComBuffer() {
    setRele(PIN_RELE_INV_ALTA, false);
    estados.inversor          = false;
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
}

// Helper interno: liga inversor e limpa buffer para receber novo sinal
void _ligarInversorComBuffer() {
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
    setRele(PIN_RELE_INV_ALTA, true);
    estados.inversor = true;
}

// ============================================================
// CÁLCULO DA BATERIA
// ============================================================
void calcularBateria() {
    // Energia total da bateria
    bateria.energiaTotal     = bateria.tensaoNominal * bateria.capacidadeAh;
    // Energia disponível (respeitando descarga segura)
    bateria.energiaDisponivel = bateria.energiaTotal * (bateria.descargaSegura / 100.0f);
}

// Calibração automática: detecta bateria 100% quando corrente bateria ≈ 0
// e painel está gerando energia (BMS cortou o carregamento)
void verificarCalibracaoAutomatica() {
    // Se painel gerando > 0.5A e bateria próxima de 0 (±0.5A)
    // significa que BMS cortou → bateria 100%
    if (sensores.correntePainel > 0.5f &&
        fabsf(sensores.correnteBateria) <= 0.5f &&
        sensores.correnteCasa > 0.1f) {

        // Calibra: bateria está 100% da energia disponível
        if (!bateria.calibrada) {
            bateria.energiaAtual    = bateria.energiaDisponivel;
            bateria.percentualAtual = 100.0f;
            bateria.calibrada       = true;

            // Salva na preferences
            prefs.begin("rrfs", false);
            prefs.putFloat("bat_energia", bateria.energiaAtual);
            prefs.putBool ("bat_calib",   true);
            prefs.end();

            adicionarLog("Bateria calibrada: 100% detectado");
        }
    }
}

// Coulomb counting: integra corrente da bateria no tempo
void atualizarEnergiBateria() {
    if (!bateria.calibrada) return;

    unsigned long agora = millis();
    float deltaT = (float)(agora - ultimoContagem) / 3600000.0f; // horas
    ultimoContagem = agora;

    // Potência entrando/saindo da bateria (W)
    // Positivo = carregando, Negativo = descarregando
    float potBat = sensores.correnteBateria * bateria.tensaoNominal;

    // Integra energia (Wh)
    bateria.energiaAtual += potBat * deltaT;

    // Limita entre 0 e energiaDisponivel
    if (bateria.energiaAtual < 0.0f)
        bateria.energiaAtual = 0.0f;
    if (bateria.energiaAtual > bateria.energiaDisponivel)
        bateria.energiaAtual = bateria.energiaDisponivel;

    // Calcula percentual (0-100% baseado na energia disponível)
    if (bateria.energiaDisponivel > 0.0f)
        bateria.percentualAtual = (bateria.energiaAtual / bateria.energiaDisponivel) * 100.0f;
    else
        bateria.percentualAtual = 0.0f;
}

// ============================================================
// LEITURA COMPLETA DOS SENSORES
// ============================================================
void lerSensores() {
    // Lê correntes
    sensores.correntePainel = lerCorrentePainel();
    sensores.correnteCasa   = lerCorrenteCasa();

    /*
     * CÁLCULO CORRENTE BATERIA:
     * Painel (após controlador) - Casa = Bateria
     * Positivo = carregando, Negativo = descarregando
     *
     * Atenção: corrente negativa no painel (< -0.3A) = alerta de cabos invertidos
     * Corrente negativa pequena (> -1A) = consumo próprio do controlador (normal)
     */
    sensores.correnteBateria = sensores.correntePainel - sensores.correnteCasa;

    // Verifica se cabos do sensor painel estão invertidos
    if (sensores.correntePainel < -2.0f) {
        adicionarLog("AVISO: Sensor painel pode estar invertido!");
    }

    // Calcula potências estimadas usando tensão nominal da bateria
    sensores.potenciaPainel  = sensores.correntePainel  * bateria.tensaoNominal;
    sensores.potenciaCasa    = sensores.correnteCasa    * bateria.tensaoNominal;
    sensores.potenciaBateria = sensores.correnteBateria * bateria.tensaoNominal;

    // Lê status inversor
    sensores.inversoreAtivo = lerStatusInversor();

    // Atualiza energia da bateria
    atualizarEnergiBateria();

    // Verifica calibração automática
    verificarCalibracaoAutomatica();
}

// ============================================================
// CONTROLE DE ENERGIA (com buffer de 60s)
// ============================================================
void controlarEnergia() {
    if (!bateria.calibrada) return;

    float pct  = bateria.percentualAtual;
    unsigned long agora = millis();

    // ---- LÂMPADA EXTERNA - AUTOMÁTICO POR LUZ DO DIA ----
    // Se painel gerando > 0.5A = dia = desliga lamp externa
    // Se painel parou de gerar = anoiteceu = liga lamp externa
    // (desde que energia suficiente e não desligada por outro motivo)
    if (sensores.correntePainel > 0.5f && estados.lampExt) {
        // Dia detectado - desliga lâmpada externa
        desligarLampExt();
        adicionarLog("Lamp.Externa OFF: dia detectado (painel gerando)");
    }
    // Liga lâmpada externa ao anoitecer (painel parou)
    // Só liga se: sem geração + energia suficiente + inversor ativo
    if (sensores.correntePainel <= 0.5f &&
        !estados.lampExt &&
        pct > redes.corteLampExt + 2.0f &&
        estados.inversor &&
        sensores.inversoreAtivo) {
        ligarLampExt();
    adicionarLog("Lamp.Externa ON: anoitecer detectado");
        }

        // ---- LÂMPADA INTERNA ----
        if (pct <= redes.corteLampInt && estados.lampInt) {
            if (!timers.pendenteLampInt) {
                timers.pendenteLampInt = true;
                timers.inicioLampInt   = agora;
            } else if (agora - timers.inicioLampInt >= RELE_BUFFER_MS) {
                desligarLampInt();
                timers.pendenteLampInt = false;
                adicionarLog("Lamp.Interna OFF: energia baixa");
            }
        } else {
            timers.pendenteLampInt = false;
            if (pct > redes.corteLampInt + 2.0f && !estados.lampInt) {
                if (estados.inversor) {
                    ligarLampInt();
                    adicionarLog("Lamp.Interna ON: energia recuperada");
                }
            }
        }

        // ---- LÂMPADA EXTERNA - CORTE POR ENERGIA ----
        // Só desliga por energia se não foi já desligada pelo dia
        if (pct <= redes.corteLampExt && estados.lampExt) {
            if (!timers.pendenteLampExt) {
                timers.pendenteLampExt = true;
                timers.inicioLampExt   = agora;
            } else if (agora - timers.inicioLampExt >= RELE_BUFFER_MS) {
                desligarLampExt();
                timers.pendenteLampExt = false;
                adicionarLog("Lamp.Externa OFF: energia baixa");
            }
        } else {
            timers.pendenteLampExt = false;
        }

        // ---- REDE ALTA ----
        if (pct <= redes.corteAlta && estados.redesAlta) {
            if (!timers.pendenteAlta) {
                timers.pendenteAlta = true;
                timers.inicioAlta   = agora;
            } else if (agora - timers.inicioAlta >= RELE_BUFFER_MS) {
                desligarRedeAlta();
                timers.pendenteAlta = false;
                adicionarLog("Rede Alta OFF: energia baixa");
            }
        } else {
            timers.pendenteAlta = false;
            if (pct > redes.corteAlta + 2.0f && !estados.redesAlta) {
                if (estados.inversor && sensores.inversoreAtivo) {
                    ligarRedeAlta();
                    adicionarLog("Rede Alta ON: energia recuperada");
                }
            }
        }

        // ---- REDE BAIXA + INVERSOR ----
        if (pct <= redes.corteBaixa &&
            (estados.redeBaixa || estados.inversor)) {

            if (!timers.pendenteBaixa) {
                timers.pendenteBaixa = true;
                timers.inicioBaixa   = agora;
            } else if (agora - timers.inicioBaixa >= RELE_BUFFER_MS) {
                if (estados.redesAlta)  { desligarRedeAlta();  delay(200); }
                if (estados.lampExt)    { desligarLampExt();   delay(100); }
                if (estados.lampInt)    { desligarLampInt();   delay(100); }
                if (estados.inversor)   { desligarInversor();  delay(500); }
                if (estados.redeBaixa)  { desligarRedeBaixa(); }
                timers.pendenteBaixa = false;
                adicionarLog("CRITICO: Tudo OFF - energia esgotada");
            }

            } else {
                timers.pendenteBaixa = false;

                if (pct > redes.corteBaixa + 2.0f) {

                    // 1. Liga rede baixa
                    if (!estados.redeBaixa) {
                        ligarRedeBaixa();
                        adicionarLog("Rede Baixa ON: energia recuperada");
                        delay(2000);
                    }

                    // 2. Liga inversor
                    if (!estados.inversor) {
                        ligarInversor();
                        adicionarLog("Inversor ON: energia recuperada");
                        delay(5000);
                        unsigned long t = millis();
                        while (millis() - t < 3000) {
                            if (lerStatusInversor()) break;
                            delay(200);
                        }
                    }

                    // 3. Liga rede alta se energia suficiente
                    if (!estados.redesAlta &&
                        pct > redes.corteAlta + 2.0f &&
                        sensores.inversoreAtivo) {
                        ligarRedeAlta();
                    adicionarLog("Rede Alta ON: energia recuperada");
                    delay(200);
                        }

                        // 4. Lamp externa: só liga se noite (painel sem geração)
                        if (!estados.lampExt &&
                            pct > redes.corteLampExt + 2.0f &&
                            sensores.inversoreAtivo &&
                            sensores.correntePainel <= 0.5f) {  // ← só liga à noite
                                ligarLampExt();
                                adicionarLog("Lamp.Externa ON: energia recuperada (noite)");
                            }

                            // 5. Liga lâmpada interna
                            if (!estados.lampInt &&
                                pct > redes.corteLampInt + 2.0f &&
                                sensores.inversoreAtivo) {
                                ligarLampInt();
                            adicionarLog("Lamp.Interna ON: energia recuperada");
                                }
                }
            }
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
    prefs.putFloat("bat_nrg",   bateria.energiaAtual);
    prefs.putBool ("bat_calib", bateria.calibrada);
    prefs.end();
}

void carregarConfigBateria() {
    prefs.begin("rrfs", true);
    bateria.tensaoNominal  = prefs.getFloat("bat_vnom",  12.8f);
    bateria.capacidadeAh   = prefs.getFloat("bat_ah",    100.0f);
    bateria.descargaSegura = prefs.getFloat("bat_desc",  80.0f);
    bateria.energiaAtual   = prefs.getFloat("bat_nrg",   0.0f);
    bateria.calibrada      = prefs.getBool ("bat_calib", false);
    prefs.end();
    calcularBateria();

    // Recalcula percentual após carregar
    if (bateria.energiaDisponivel > 0)
        bateria.percentualAtual = (bateria.energiaAtual / bateria.energiaDisponivel) * 100.0f;
}

void salvarConfigRedes() {
    prefs.begin("rrfs", false);
    prefs.putFloat("r_baixa",   redes.corteBaixa);
    prefs.putFloat("r_alta",    redes.corteAlta);
    prefs.putFloat("r_lmpext",  redes.corteLampExt);
    prefs.putFloat("r_lmpint",  redes.corteLampInt);
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

// ============================================================
// SALVAR / CARREGAR CALIBRAÇÃO
// ============================================================
void salvarCalibSensores() {
    prefs.begin("rrfs", false);
    prefs.putInt  ("cal_p_off",  calibPainel.offsetMV);
    prefs.putFloat("cal_p_sens", calibPainel.sensMvA);
    prefs.putBool ("cal_p_inv",  calibPainel.invertido);
    prefs.putBool ("cal_p_ok",   calibPainel.calibrado);
    prefs.putInt  ("cal_c_off",  calibCasa.offsetMV);
    prefs.putFloat("cal_c_sens", calibCasa.sensMvA);
    prefs.putBool ("cal_c_inv",  calibCasa.invertido);
    prefs.putBool ("cal_c_ok",   calibCasa.calibrado);
    prefs.end();
    adicionarLog("Calibracao sensores salva");
}

void carregarCalibSensores() {
    prefs.begin("rrfs", true);
    calibPainel.offsetMV  = prefs.getInt  ("cal_p_off",  OFFSET_LIDO_MV);
    calibPainel.sensMvA   = prefs.getFloat("cal_p_sens", PAINEL_SENS_MV_A * 0.66f);
    calibPainel.invertido = prefs.getBool ("cal_p_inv",  false);
    calibPainel.calibrado = prefs.getBool ("cal_p_ok",   false);
    calibCasa.offsetMV    = prefs.getInt  ("cal_c_off",  OFFSET_LIDO_MV);
    calibCasa.sensMvA     = prefs.getFloat("cal_c_sens", CASA_SENS_MV_A * 0.66f);
    calibCasa.invertido   = prefs.getBool ("cal_c_inv",  false);
    calibCasa.calibrado   = prefs.getBool ("cal_c_ok",   false);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "Calib Painel: off=%dmV sens=%.2fmV/A inv=%s",
             calibPainel.offsetMV, calibPainel.sensMvA,
             calibPainel.invertido ? "SIM" : "NAO");
    adicionarLog(tempBuf);

    snprintf(tempBuf, sizeof(tempBuf),
             "Calib Casa: off=%dmV sens=%.2fmV/A inv=%s",
             calibCasa.offsetMV, calibCasa.sensMvA,
             calibCasa.invertido ? "SIM" : "NAO");
    adicionarLog(tempBuf);
}


// ============================================================
// VERIFICAÇÃO DE AUTENTICAÇÃO
// ============================================================
bool verificarAutenticacao() {
    if (!existeCredenciais()) return true;
    return logado;
}

// ============================================================
// MONITORAR INVERSOR - LÓGICA COMPLETA CORRIGIDA
//
// FLUXO:
// 1. Inversor ligado → monitora status
// 2. Status para → aguarda 1s
// 3. Ainda sem status → desliga inversor
// 4. Aguarda 2s → liga inversor novamente
// 5. Status voltou → era pico, tudo normal
// 6. Status não voltou → repete até MAX_TENTATIVAS_RELIGAR
// 7. Esgotou tentativas → inicia diagnóstico de curto
// ============================================================
void monitorarInversor() {

    unsigned long agora = millis();

    // Processos ativos que continuam mesmo com chave desligada
    bool emProcesso = (curto.estado == CURTO_RELIGANDO      ||
    curto.estado == CURTO_TESTANDO_ALTA  ||
    curto.estado == CURTO_TESTANDO_BAIXA);

    // --------------------------------------------------------
    // Chave desligada e sem processo ativo → nada a fazer
    // --------------------------------------------------------
    if (!estados.inversor && !emProcesso) {
        if (curto.estado == CURTO_NORMAL) {
            curto.monitorando       = false;
            curto.tInicio           = 0;
            curto.tentativasReligar = 0;
        }
        return;
    }

    // ========================================================
    // ESTADO NORMAL: monitora status
    // ========================================================
    if (curto.estado == CURTO_NORMAL) {

        // Chave ligada: lê status (o buffer interno já
        // sabe que chave está ligada e monitora corretamente)
        bool statusAtual = lerStatusInversor();

        if (statusAtual) {
            // Status OK → tudo normal
            curto.tInicio           = 0;
            curto.monitorando       = true;
            curto.tentativasReligar = 0;
            return;
        }

        // Chave ligada mas sem status
        if (curto.monitorando) {

            if (curto.tInicio == 0) {
                // Inicia contagem de 1s
                curto.tInicio = agora;
                adicionarLog("Inversor: status perdido - aguardando 1s...");
                return;
            }

            // Dentro do 1s de tolerância
            if (agora - curto.tInicio < T_STATUS_AUSENTE) {
                return;
            }

            // ------------------------------------------------
            // 1s sem status → inversor entrou em proteção
            // ------------------------------------------------
            adicionarLog("Inversor: sem status apos 1s - proteção ativa!");
            adicionarLog("Desligando chave - aguardando 2s para religar...");

            _desligarInversorComBuffer();

            curto.estado    = CURTO_RELIGANDO;
            curto.subEstado = SUB_INVERSOR_DESLIGADO;
            curto.tInicio   = agora;
        }
        return;
    }

    // ========================================================
    // ESTADO: RELIGANDO
    // Tenta religar até MAX_TENTATIVAS_RELIGAR antes do diagnóstico
    // ========================================================
    if (curto.estado == CURTO_RELIGANDO) {

        switch (curto.subEstado) {

            // ----------------------------------------------------
            // Chave desligada - aguarda 2s para religar
            // (status nunca aparece aqui pois chave está off)
            // ----------------------------------------------------
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_PAUSA_RELIGAR) {
                    return; // ainda aguardando 2s
                }

                // 2s passados: liga inversor
                snprintf(tempBuf, sizeof(tempBuf),
                         "2s passados - religando (tentativa %d/%d)...",
                         curto.tentativasReligar + 1, MAX_TENTATIVAS_RELIGAR);
                adicionarLog(tempBuf);

                _ligarInversorComBuffer();

                curto.tInicio   = agora;
                curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("Aguardando status do inversor por 3s...");
                break;

                // ----------------------------------------------------
                // Chave ligada - verifica se status voltou
                // Pode acontecer:
                //   A) Status volta e fica → era pico → NORMAL
                //   B) Status não volta em 3s → tenta de novo ou diagnóstico
                //   C) Status volta e some → conta como falha
                // ----------------------------------------------------
            case SUB_AGUARDANDO_STATUS: {

                bool statusAgora = lerStatusInversor();

                if (statusAgora) {
                    // Status voltou! Verifica se é estável
                    // Aguarda mais 2s monitorando para confirmar
                    if (curto.tInicio == 0 ||
                        // Reseta timer quando status aparece pela primeira vez
                        timers.inversorUltimoHigh == agora) {
                        // (o lerStatusInversor já atualizou inversorUltimoHigh)
                        }

                        // Verifica se status está presente há pelo menos 2s
                        // Usa o timestamp do último HIGH
                        unsigned long tempoComStatus = agora - timers.inversorUltimoHigh;

                    // Se o buffer confirmou estabilidade (status presente > 2s buffer)
                    // E o status ainda está ativo agora → era pico!
                    if (timers.inversorEstadoReal && tempoComStatus < 500) {
                        // Status acabou de aparecer - aguarda confirmação
                        // (o buffer de 2s do lerStatusInversor cuida disso)
                        return;
                    }

                    if (timers.inversorEstadoReal) {
                        // STATUS ESTÁVEL → era apenas um pico de energia
                        curto.estado            = CURTO_NORMAL;
                        curto.subEstado         = SUB_IDLE;
                        curto.tInicio           = 0;
                        curto.monitorando       = true;
                        curto.tentativasReligar = 0;
                        adicionarLog("Inversor OK apos religar!");
                        adicionarLog("Era pico de energia - sistema normal");
                        return;
                    }
                }

                // Status não voltou dentro de 3s
                if (agora - curto.tInicio >= T_AGUARDA_STATUS) {

                    curto.tentativasReligar++;
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Status nao voltou (tentativa %d/%d)",
                             curto.tentativasReligar, MAX_TENTATIVAS_RELIGAR);
                    adicionarLog(tempBuf);

                    // Desliga inversor para nova tentativa ou diagnóstico
                    _desligarInversorComBuffer();
                    curto.tInicio = agora;

                    if (curto.tentativasReligar < MAX_TENTATIVAS_RELIGAR) {
                        // Tenta de novo
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        snprintf(tempBuf, sizeof(tempBuf),
                                 "Nova tentativa em 2s (%d/%d)...",
                                 curto.tentativasReligar + 1,
                                 MAX_TENTATIVAS_RELIGAR);
                        adicionarLog(tempBuf);
                    } else {
                        // Esgotou tentativas → inicia diagnóstico
                        curto.estado    = CURTO_TESTANDO_ALTA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        adicionarLog("=== INICIANDO DIAGNOSTICO DE CURTO ===");
                        adicionarLog("Passo 1: testando rede ALTA isolada...");
                    }
                }
                break;
            }

            default:
                curto.estado    = CURTO_NORMAL;
                curto.subEstado = SUB_IDLE;
                curto.tInicio   = 0;
                break;
        }
        return;
    }

    // ========================================================
    // DIAGNÓSTICO - TESTANDO REDE ALTA
    //
    // Desliga rede alta, liga inversor:
    // → Status OK e estável → rede baixa OK
    //   → Liga rede alta novamente:
    //     → Status cai → CURTO NA REDE ALTA
    //     → Status OK → sistema normal
    // → Status não volta → passa para teste rede baixa
    // ========================================================
    if (curto.estado == CURTO_TESTANDO_ALTA) {

        switch (curto.subEstado) {

            // Aguarda 1.5s com tudo desligado antes de testar
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_AGUARDA_QUEDA) return;

                // Desliga rede alta para testar inversor isolado
                desligarRedeAlta();
            delay(300);

            // Liga inversor SEM rede alta
            _ligarInversorComBuffer();
            curto.tInicio   = agora;
            curto.subEstado = SUB_AGUARDANDO_STATUS;
            adicionarLog("Teste Alta: inversor ligado SEM rede alta");
            break;

            // Aguarda status do inversor (sem rede alta)
            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    // Inversor respondeu sem rede alta!
                    // Rede baixa está OK - aguarda confirmar estabilidade
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("Teste Alta: inversor OK sem rede alta - confirmando...");
                }
                else if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    // Inversor não ligou mesmo sem rede alta
                    // → problema não é a rede alta → testa rede baixa
                    _desligarInversorComBuffer();
                    curto.estado    = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio   = agora;
                    adicionarLog("Teste Alta: inversor nao respondeu - testando rede BAIXA...");
                }
                break;

                // Inversor OK sem rede alta - aguarda 3s de estabilidade
            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    // Verifica se status caiu durante confirmação
                    if (!lerStatusInversor()) {
                        // Instável sem rede alta → problema no inversor
                        _desligarInversorComBuffer();
                        curto.estado    = CURTO_TESTANDO_BAIXA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        curto.tInicio   = agora;
                        adicionarLog("Teste Alta: inversor instavel - testando rede BAIXA...");
                    }
                    return;
                }

                // 3s estável sem rede alta → agora liga rede alta para confirmar
                adicionarLog("Teste Alta: 3s estavel - ligando rede alta para confirmar...");
                ligarRedeAlta();
                curto.tInicio   = agora;
                curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                break;

                // Liga rede alta e verifica se inversor aguenta
            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    // Monitora se status cai logo após ligar rede alta
                    if (!lerStatusInversor() &&
                        agora - curto.tInicio > 500) { // aguarda 500ms de estabilização

                            // ----------------------------------------
                            // CURTO CONFIRMADO NA REDE ALTA
                            // ----------------------------------------
                            adicionarLog("Teste Alta: inversor caiu com rede alta!");
                            adicionarLog("=== CURTO CONFIRMADO: REDE ALTA ===");

                            // Desliga rede alta imediatamente
                            desligarRedeAlta();
                            delay(200);

                            // Desliga e religa inversor
                            _desligarInversorComBuffer();
                            delay(500);
                            _ligarInversorComBuffer();

                            // Aguarda inversor recuperar
                            delay(2000);

                            curto.estado        = CURTO_CONFIRMADO_ALTA;
                            curto.redeAltaCurto = true;
                            curto.subEstado     = SUB_IDLE;
                            curto.tInicio       = 0;
                            adicionarLog("Rede alta BLOQUEADA - inversor operando normalmente");
                            adicionarLog("Acesse /redes para resolver o curto");
                            return;
                        }
                        return;
                }

                // 3s com rede alta e inversor OK → sistema normal!
                if (lerStatusInversor()) {
                    curto.estado            = CURTO_NORMAL;
                    curto.subEstado         = SUB_IDLE;
                    curto.tInicio           = 0;
                    curto.monitorando       = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Teste Alta: sistema OK - era pico no diagnostico");
                } else {
                    // Status sumiu após 3s → ainda com problema
                    _desligarInversorComBuffer();
                    desligarRedeAlta();
                    curto.estado    = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio   = agora;
                    adicionarLog("Teste Alta: problema persistente - testando rede BAIXA...");
                }
                break;

                default: break;
        }
        return;
    }

    // ========================================================
    // DIAGNÓSTICO - TESTANDO REDE BAIXA
    //
    // Desliga rede baixa, liga inversor:
    // → Status OK e estável → liga rede baixa:
    //   → Status cai → CURTO NA REDE BAIXA
    //   → Status OK → sistema normal (religa tudo)
    // → Status não volta → FALHA NO INVERSOR
    // ========================================================
    if (curto.estado == CURTO_TESTANDO_BAIXA) {

        switch (curto.subEstado) {

            // Aguarda antes de testar rede baixa
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_AGUARDA_QUEDA) return;

                // Desliga AMBAS as redes para testar inversor completamente isolado
                desligarRedeAlta();
            desligarRedeBaixa();
            delay(300);

            // Liga inversor sem nenhuma carga
            _ligarInversorComBuffer();
            curto.tInicio   = agora;
            curto.subEstado = SUB_AGUARDANDO_STATUS;
            adicionarLog("Teste Baixa: inversor ligado SEM nenhuma rede");
            break;

            // Aguarda status (sem nenhuma rede)
            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("Teste Baixa: inversor OK sem cargas - confirmando...");
                }
                else if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    // Inversor não liga nem sem carga → FALHA NO INVERSOR
                    _desligarInversorComBuffer();
                    curto.estado    = CURTO_AGUARDANDO_RESET;
                    curto.subEstado = SUB_IDLE;
                    curto.tInicio   = 0;
                    adicionarLog("=== FALHA NO INVERSOR ===");
                    adicionarLog("Nao liga mesmo sem cargas!");
                    adicionarLog("Verifique: fusivel, temperatura, sobrecarga");
                    adicionarLog("Acesse /redes para tentar reset manual");
                }
                break;

                // Inversor OK sem cargas - aguarda 3s de estabilidade
            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        // Instável sem nenhuma carga → problema no inversor
                        _desligarInversorComBuffer();
                        curto.estado    = CURTO_AGUARDANDO_RESET;
                        curto.subEstado = SUB_IDLE;
                        curto.tInicio   = 0;
                        adicionarLog("=== INVERSOR INSTAVEL SEM CARGA ===");
                        adicionarLog("Verifique o equipamento fisicamente");
                    }
                    return;
                }

                // 3s estável → agora liga rede baixa para testar
                adicionarLog("Teste Baixa: 3s estavel - ligando rede baixa para confirmar...");
                ligarRedeBaixa();
                curto.tInicio   = agora;
                curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                break;

                // Liga rede baixa e verifica se inversor aguenta
            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    // Monitora se status cai logo após ligar rede baixa
                    if (!lerStatusInversor() &&
                        agora - curto.tInicio > 500) {

                        // ----------------------------------------
                        // CURTO CONFIRMADO NA REDE BAIXA
                        // ----------------------------------------
                        adicionarLog("Teste Baixa: inversor caiu com rede baixa!");
                    adicionarLog("=== CURTO CONFIRMADO: REDE BAIXA ===");

                    desligarRedeBaixa();
                    delay(200);
                    _desligarInversorComBuffer();
                    delay(500);

                    curto.estado         = CURTO_CONFIRMADO_BAIXA;
                    curto.redeBaixaCurto = true;
                    curto.subEstado      = SUB_IDLE;
                    curto.tInicio        = 0;
                    adicionarLog("Rede baixa e inversor BLOQUEADOS por seguranca");
                    adicionarLog("Acesse /redes para resolver o curto");
                    return;
                        }
                        return;
                }

                // 3s com rede baixa e inversor OK → sistema normal!
                if (lerStatusInversor()) {
                    // Liga tudo de volta
                    ligarRedeAlta();
                    delay(300);
                    ligarLampExt();
                    delay(200);
                    ligarLampInt();

                    curto.estado            = CURTO_NORMAL;
                    curto.subEstado         = SUB_IDLE;
                    curto.tInicio           = 0;
                    curto.monitorando       = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Teste Baixa: sistema completamente normal!");
                    adicionarLog("Todas as redes religadas");
                } else {
                    // Status sumiu → ainda com problema
                    _desligarInversorComBuffer();
                    desligarRedeBaixa();
                    curto.estado    = CURTO_AGUARDANDO_RESET;
                    curto.subEstado = SUB_IDLE;
                    curto.tInicio   = 0;
                    adicionarLog("Problema persistente - aguardando reset manual");
                }
                break;

                default: break;
        }
        return;
    }

    // ========================================================
    // ESTADOS DE PROTEÇÃO PERMANENTE
    // ========================================================

    // Curto na rede alta: mantém alta desligada, inversor e baixa OK
    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        if (estados.redesAlta) desligarRedeAlta();
        // Inversor e rede baixa continuam operando normalmente
        return;
    }

    // Curto na rede baixa OU falha no inversor:
    // Desliga tudo por segurança
    if (curto.estado == CURTO_CONFIRMADO_BAIXA ||
        curto.estado == CURTO_AGUARDANDO_RESET) {
        if (estados.inversor)  _desligarInversorComBuffer();
        if (estados.redeBaixa) desligarRedeBaixa();
        if (estados.redesAlta) desligarRedeAlta();
        return;
        }
}

// ============================================================
// RESET DE CURTO VIA WEB
// Chamado quando usuário confirma que resolveu o problema
// ============================================================
void resetCurtoAlta() {
    if (curto.estado != CURTO_CONFIRMADO_ALTA) return;

    adicionarLog("Reset curto rede alta solicitado");

    // Sequência de retorno seguro:
    // 1. Desliga rede alta (garantia)
    desligarRedeAlta();
    delay(500);

    // 2. Verifica se inversor está OK antes de ligar rede alta
    // Liga apenas rede alta temporariamente para testar
    ligarRedeAlta();
    delay(300);

    if (lerStatusInversor()) {
        // Inversor ainda OK → curto resolvido!
        curto.estado         = CURTO_NORMAL;
        curto.redeAltaCurto  = false;
        curto.subEstado      = SUB_IDLE;
        curto.tInicio        = 0;
        curto.monitorando    = true;
        adicionarLog("Curto rede alta RESOLVIDO - sistema normal");
    } else {
        // Inversor ainda cai com rede alta → curto não resolvido
        desligarRedeAlta();
        adicionarLog("AVISO: Curto rede alta NAO resolvido!");
        adicionarLog("Verifique os equipamentos e tente novamente.");
    }
}

void resetCurtoBaixa() {
    if (curto.estado != CURTO_CONFIRMADO_BAIXA &&
        curto.estado != CURTO_AGUARDANDO_RESET) return;

    adicionarLog("Reset curto rede baixa solicitado");

    // 1. Desliga tudo (garantia)
    desligarRedeBaixa();
    desligarInversor();
    delay(500);

    // 2. Liga inversor primeiro para ver se está OK
    ligarInversor();
    unsigned long t = millis();
    bool invOK = false;

    while (millis() - t < T_ESPERA_STATUS_INV) {
        if (lerStatusInversor()) { invOK = true; break; }
        delay(100);
    }

    if (!invOK) {
        desligarInversor();
        adicionarLog("ERRO: Inversor nao ligou - verifique o equipamento!");
        return;
    }

    // 3. Liga rede baixa para testar
    ligarRedeBaixa();
    delay(500);

    if (lerStatusInversor()) {
        // Inversor OK com rede baixa → curto resolvido!
        curto.estado          = CURTO_NORMAL;
        curto.redeBaixaCurto  = false;
        curto.subEstado       = SUB_IDLE;
        curto.tInicio         = 0;
        curto.monitorando     = true;
        adicionarLog("Curto rede baixa RESOLVIDO - sistema normal");
    } else {
        // Ainda em curto
        desligarRedeBaixa();
        desligarInversor();
        curto.estado = CURTO_CONFIRMADO_BAIXA;
        adicionarLog("AVISO: Curto rede baixa NAO resolvido!");
        adicionarLog("Verifique os equipamentos e tente novamente.");
    }
}

void sincronizarNTP() {
    if (WiFi.status() != WL_CONNECTED) {
        adicionarLog("NTP: sem WiFi - ignorado");
        return;
    }

    // UTC-3 Brasil (Brasilia)
    // Ajuste o primeiro valor se sua região for diferente:
    // UTC-3 = -3 * 3600 = -10800
    // UTC-4 = -4 * 3600 = -14400
    configTime(-3 * 3600, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");

    adicionarLog("NTP: sincronizando...");

    struct tm ti;
    unsigned long t = millis();
    bool ok = false;

    while (millis() - t < 5000) {
        if (getLocalTime(&ti)) {
            ok = true;
            break;
        }
        delay(200);
    }

    if (ok) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "NTP OK: %02d:%02d:%02d %02d/%02d/%04d",
                 ti.tm_hour, ti.tm_min,   ti.tm_sec,
                 ti.tm_mday, ti.tm_mon+1, ti.tm_year + 1900);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("NTP: falhou - usando millis()");
    }
}


void handleOTAUpload() {
    if (!logado) {
        server.send(403, "text/plain", "Nao autorizado");
        return;
    }

    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        adicionarLog("OTA: iniciando upload...");
        snprintf(tempBuf, sizeof(tempBuf),
                 "OTA: arquivo: %s", upload.filename.c_str());
        adicionarLog(tempBuf);

        // Desliga LED blink para indicar OTA
        ledFixo(false);

        // Inicia update com tamanho máximo disponível
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            adicionarLog("OTA ERRO: sem espaco suficiente");
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
        // Pisca LED durante escrita
        static bool ledOTA = false;
        ledOTA = !ledOTA;
        digitalWrite(LED_AZUL, ledOTA);

        // Escreve bloco recebido
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            adicionarLog("OTA ERRO: falha na escrita");
            Update.printError(Serial);
        } else {
            // Log de progresso a cada ~100KB
            static uint32_t ultimoLog = 0;
            if (upload.totalSize - ultimoLog >= 102400) {
                ultimoLog = upload.totalSize;
                snprintf(tempBuf, sizeof(tempBuf),
                         "OTA: %u KB recebidos", upload.totalSize / 1024);
                adicionarLog(tempBuf);
            }
        }
    }
    else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            snprintf(tempBuf, sizeof(tempBuf),
                     "OTA OK: %u bytes gravados", upload.totalSize);
            adicionarLog(tempBuf);
            ledFixo(true);
        } else {
            adicionarLog("OTA ERRO: falha ao finalizar");
            Update.printError(Serial);
            ledBlink();
        }
    }
    else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        adicionarLog("OTA: upload cancelado");
        ledBlink();
    }
}

// ============================================================
// SALVAR UM DIA ESPECÍFICO NA FLASH
// ============================================================
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia) {
    // Cada dia em namespace próprio
    // NVS suporta até ~500KB dependendo da partição
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);

    prefs.begin(ns, false);
    prefs.putBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();
}

// ============================================================
// CARREGAR UM DIA ESPECÍFICO DA FLASH
// ============================================================
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia) {
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);

    prefs.begin(ns, true);
    size_t len = prefs.getBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();

    return (len == sizeof(RegistroDia) && dia.valido);
}

// ============================================================
// SALVAR ÍNDICES DE CONTROLE
// ============================================================
void salvarIndiceHistorico() {
    prefs.begin("rrfs_hidx", false);
    prefs.putUChar ("ver",   HIST_KEY_VER);
    prefs.putUShort("idx",   histIndiceAtual);
    prefs.putUShort("total", histTotalDias);
    prefs.end();
}

// ============================================================
// CARREGAR HISTÓRICO - só carrega índices + dia atual
// ============================================================
void carregarHistorico() {
    prefs.begin("rrfs_hidx", true);
    uint8_t ver = prefs.getUChar("ver", 0);

    if (ver != HIST_KEY_VER) {
        prefs.end();
        // Versão diferente: reseta índices
        histIndiceAtual = 0;
        histTotalDias   = 0;
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
        adicionarLog("Historico: resetado (versao nova)");
        return;
    }

    histIndiceAtual = prefs.getUShort("idx",   0);
    histTotalDias   = prefs.getUShort("total", 0);
    prefs.end();

    // Carrega apenas o dia atual na RAM
    if (histTotalDias > 0) {
        if (!carregarDiaFlash(histIndiceAtual, histDiaAtual)) {
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
        }
    } else {
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
    }

    snprintf(tempBuf, sizeof(tempBuf),
             "Historico: %d/%d dias | idx=%d",
             histTotalDias, HIST_DIAS, histIndiceAtual);
    adicionarLog(tempBuf);
}

// ============================================================
// SALVAR HISTÓRICO (salva dia atual + índices)
// ============================================================
void salvarHistorico() {
    salvarDiaFlash(histIndiceAtual, histDiaAtual);
    salvarIndiceHistorico();
}

// ============================================================
// ACUMULAR ENERGIA - IGUAL MAS USA histDiaAtual
// ============================================================
void acumularEnergia() {
    // Só acumula se tiver NTP sincronizado
    struct tm ti;
    if (!getLocalTime(&ti)) return;

    unsigned long agora    = millis();
    uint8_t  horaAtual     = ti.tm_hour;
    uint8_t  diaAtual      = ti.tm_mday;
    uint8_t  mesAtual      = ti.tm_mon + 1;
    uint16_t anoAtual      = ti.tm_year + 1900;

    // ── INICIALIZAÇÃO NA PRIMEIRA CHAMADA ────────────────────
    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum = agora;
        acum.horaAtual  = horaAtual;
        acum.diaAtual   = diaAtual;
        acum.mesAtual   = mesAtual;
        acum.anoAtual   = anoAtual;

        // Verifica se já existe o dia atual na flash
        bool diaExiste = false;
        if (histTotalDias > 0) {
            // Tenta carregar o índice atual da flash
            RegistroDia dTemp;
            if (carregarDiaFlash(histIndiceAtual, dTemp)) {
                if (dTemp.dia == diaAtual &&
                    dTemp.mes == mesAtual &&
                    dTemp.ano == anoAtual) {
                    // Dia atual já existe - recupera da flash
                    histDiaAtual = dTemp;
                diaExiste    = true;
                adicionarLog("Hist: dia atual recuperado da flash");
                    }
            }
        }

        if (!diaExiste) {
            // Cria novo dia em RAM
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano    = anoAtual;
            histDiaAtual.mes    = mesAtual;
            histDiaAtual.dia    = diaAtual;
            histDiaAtual.valido = true;

            // Salva imediatamente na flash como dia válido
            // Avança índice se necessário
            bool diaNovoNaFlash = false;
            if (histTotalDias == 0) {
                // Primeiro dia
                histIndiceAtual = 0;
                histTotalDias   = 1;
                diaNovoNaFlash  = true;
            } else {
                // Verifica se índice atual tem dia diferente
                RegistroDia dTemp;
                if (carregarDiaFlash(histIndiceAtual, dTemp)) {
                    if (dTemp.dia != diaAtual ||
                        dTemp.mes != mesAtual ||
                        dTemp.ano != anoAtual) {
                        // Avança índice circular
                        histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
                    if (histTotalDias < HIST_DIAS) histTotalDias++;
                    diaNovoNaFlash = true;
                        }
                } else {
                    diaNovoNaFlash = true;
                }
            }

            if (diaNovoNaFlash) {
                salvarDiaFlash(histIndiceAtual, histDiaAtual);
                salvarIndiceHistorico();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Hist: novo dia criado %02d/%02d/%04d idx=%d total=%d",
                         diaAtual, mesAtual, anoAtual,
                         histIndiceAtual, histTotalDias);
                adicionarLog(tempBuf);
            }
        }
        return;
    }

    // ── ACUMULA POTÊNCIA × TEMPO ─────────────────────────────
    float deltaH = (float)(agora - acum.ultimoAcum) / 3600000.0f;
    acum.ultimoAcum = agora;

    // Limita delta para evitar saltos grandes (ex: após sleep)
    if (deltaH > 0.1f) deltaH = 0.1f; // máximo 6 minutos por ciclo

    float pp = sensores.potenciaPainel;
    float pc = sensores.potenciaCasa;
    float pb = sensores.potenciaBateria;

    // Painel: só geração (positivo)
    if (pp > 1.0f)  acum.painelWh   += pp * deltaH;

    // Casa: consumo (sempre positivo)
    if (pc > 0.5f)  acum.casaWh     += pc * deltaH;

    // Bateria: separa carga e descarga
    if (pb > 1.0f)  acum.batCargWh  += pb        * deltaH;
    if (pb < -1.0f) acum.batDescWh  += fabsf(pb) * deltaH;

    // ── DETECTA MUDANÇA DE HORA OU DIA ───────────────────────
    bool mudouHora = (horaAtual != acum.horaAtual);
    bool mudouDia  = (diaAtual  != acum.diaAtual ||
    mesAtual  != acum.mesAtual  ||
    anoAtual  != acum.anoAtual);

    if (mudouHora || mudouDia) {

        // ── Fecha a hora que passou ───────────────────────────
        uint8_t horaIdx = acum.horaAtual % HIST_HORAS;

        // Adiciona ao registro da hora no dia atual (RAM)
        histDiaAtual.horas[horaIdx].painelWh  += acum.painelWh;
        histDiaAtual.horas[horaIdx].casaWh    += acum.casaWh;
        histDiaAtual.horas[horaIdx].batCargWh += acum.batCargWh;
        histDiaAtual.horas[horaIdx].batDescWh += acum.batDescWh;

        // Recalcula totais do dia
        calcularTotaisDia(histDiaAtual);

        // Atualiza contadores globais (nunca zerados)
        contadores.painelKwh  += acum.painelWh  / 1000.0f;
        contadores.casaKwh    += acum.casaWh    / 1000.0f;
        contadores.batCargKwh += acum.batCargWh / 1000.0f;
        contadores.batDescKwh += acum.batDescWh / 1000.0f;

        // Log da hora fechada
        snprintf(tempBuf, sizeof(tempBuf),
                 "Hist hora %02d: P=%.0fWh C=%.0fWh BC=%.0fWh BD=%.0fWh",
                 acum.horaAtual,
                 acum.painelWh, acum.casaWh,
                 acum.batCargWh, acum.batDescWh);
        adicionarLog(tempBuf);

        if (mudouDia) {
            // ── Fecha o dia que passou ────────────────────────
            // Salva dia completo na flash
            salvarDiaFlash(histIndiceAtual, histDiaAtual);

            snprintf(tempBuf, sizeof(tempBuf),
                     "Hist dia fechado %02d/%02d/%04d: "
                     "P=%.0fWh C=%.0fWh BC=%.0fWh BD=%.0fWh",
                     histDiaAtual.dia, histDiaAtual.mes, histDiaAtual.ano,
                     histDiaAtual.totalPainelWh,
                     histDiaAtual.totalCasaWh,
                     histDiaAtual.totalBatCargWh,
                     histDiaAtual.totalBatDescWh);
            adicionarLog(tempBuf);

            // Avança índice circular
            histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
            if (histTotalDias < HIST_DIAS) histTotalDias++;

            // Cria novo dia em RAM
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano    = anoAtual;
            histDiaAtual.mes    = mesAtual;
            histDiaAtual.dia    = diaAtual;
            histDiaAtual.valido = true;

            // Salva novo dia vazio na flash já
            salvarDiaFlash(histIndiceAtual, histDiaAtual);

            snprintf(tempBuf, sizeof(tempBuf),
                     "Hist: novo dia %02d/%02d/%04d idx=%d/%d",
                     diaAtual, mesAtual, anoAtual,
                     histIndiceAtual, HIST_DIAS);
            adicionarLog(tempBuf);

        } else {
            // Mesmo dia: só salva hora atualizada
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
        }

        // Salva índices e contadores
        salvarIndiceHistorico();
        salvarContadores();

        // Zera acumuladores para nova hora
        acum.painelWh   = 0;
        acum.casaWh     = 0;
        acum.batCargWh  = 0;
        acum.batDescWh  = 0;
        acum.horaAtual  = horaAtual;

        if (mudouDia) {
            acum.diaAtual = diaAtual;
            acum.mesAtual = mesAtual;
            acum.anoAtual = anoAtual;
        }
    }

    // ── SALVA DIA ATUAL NA FLASH A CADA 5 MINUTOS ────────────
    // Para não perder dados em caso de queda de energia
    static unsigned long tSalvaParcial = 0;
    if (agora - tSalvaParcial >= 300000UL) { // 5 minutos
        tSalvaParcial = agora;

        // Atualiza hora em andamento na RAM antes de salvar
        uint8_t horaIdxAtual = horaAtual % HIST_HORAS;
        // Cria cópia temporária para não duplicar ao fechar a hora
        RegistroDia diaTemp = histDiaAtual;
        diaTemp.horas[horaIdxAtual].painelWh  += acum.painelWh;
        diaTemp.horas[horaIdxAtual].casaWh    += acum.casaWh;
        diaTemp.horas[horaIdxAtual].batCargWh += acum.batCargWh;
        diaTemp.horas[horaIdxAtual].batDescWh += acum.batDescWh;
        calcularTotaisDia(diaTemp);

        salvarDiaFlash(histIndiceAtual, diaTemp);
        salvarContadores();

        snprintf(tempBuf, sizeof(tempBuf),
                 "Hist salvo: P=%.0fWh C=%.0fWh (parcial hora %02d)",
                 acum.painelWh, acum.casaWh, horaAtual);
        adicionarLog(tempBuf);
    }
}

// ============================================================
// CALCULAR TOTAIS DE UM DIA - Adicionar antes de acumularEnergia()
// ============================================================
void calcularTotaisDia(RegistroDia& dia) {
    dia.totalPainelWh  = 0;
    dia.totalCasaWh    = 0;
    dia.totalBatCargWh = 0;
    dia.totalBatDescWh = 0;
    for (uint8_t h = 0; h < HIST_HORAS; h++) {
        dia.totalPainelWh  += dia.horas[h].painelWh;
        dia.totalCasaWh    += dia.horas[h].casaWh;
        dia.totalBatCargWh += dia.horas[h].batCargWh;
        dia.totalBatDescWh += dia.horas[h].batDescWh;
    }
}

// ============================================================
// SALVAR CONTADORES - Adicionar antes de acumularEnergia()
// ============================================================
void salvarContadores() {
    prefs.begin("rrfs_cnt", false);
    prefs.putFloat("p_kwh",  contadores.painelKwh);
    prefs.putFloat("c_kwh",  contadores.casaKwh);
    prefs.putFloat("bc_kwh", contadores.batCargKwh);
    prefs.putFloat("bd_kwh", contadores.batDescKwh);
    prefs.end();
}

// ============================================================
// CARREGAR CONTADORES - Adicionar antes de acumularEnergia()
// ============================================================
void carregarContadores() {
    prefs.begin("rrfs_cnt", true);
    contadores.painelKwh  = prefs.getFloat("p_kwh",  0.0f);
    contadores.casaKwh    = prefs.getFloat("c_kwh",  0.0f);
    contadores.batCargKwh = prefs.getFloat("bc_kwh", 0.0f);
    contadores.batDescKwh = prefs.getFloat("bd_kwh", 0.0f);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "Contadores: P=%.2fkWh C=%.2fkWh BC=%.2fkWh BD=%.2fkWh",
             contadores.painelKwh, contadores.casaKwh,
             contadores.batCargKwh, contadores.batDescKwh);
    adicionarLog(tempBuf);
}

// ============================================================
// SALVAR / CARREGAR CONFIG LOCAL
// ============================================================
void salvarConfigLocal() {
    prefs.begin("rrfs_loc", false);
    prefs.putDouble("lat",      configLocal.latitude);
    prefs.putDouble("lon",      configLocal.longitude);
    prefs.putFloat ("kwp",      configLocal.painelKWp);
    prefs.putFloat ("ef",       configLocal.eficiencia);
    prefs.putFloat ("fcalib",   configLocal.fatorCalib);
    prefs.putUChar ("ncalib",   configLocal.amostrasCalib);
    prefs.putBool  ("conf",     configLocal.configurado);
    prefs.end();
    adicionarLog("Config local salva");
}

void carregarConfigLocal() {
    prefs.begin("rrfs_loc", true);
    configLocal.latitude       = prefs.getDouble("lat",    0.0);
    configLocal.longitude      = prefs.getDouble("lon",    0.0);
    configLocal.painelKWp      = prefs.getFloat ("kwp",    1.0f);
    configLocal.eficiencia     = prefs.getFloat ("ef",     0.80f);
    configLocal.fatorCalib     = prefs.getFloat ("fcalib", 1.0f);
    configLocal.amostrasCalib  = prefs.getUChar ("ncalib", 0);
    configLocal.configurado    = prefs.getBool  ("conf",   false);
    prefs.end();

    if (configLocal.configurado) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Local: lat=%.6f lon=%.6f kWp=%.2f ef=%.0f%% calib=%.2f(%d)",
                 configLocal.latitude, configLocal.longitude,
                 configLocal.painelKWp, configLocal.eficiencia * 100,
                 configLocal.fatorCalib, configLocal.amostrasCalib);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Local: nao configurado");
    }
}

// ============================================================
// CALIBRAÇÃO AUTOMÁTICA
// Compara geração estimada vs real quando painel está gerando
// ============================================================
void calibrarPrevisao() {
    if (!configLocal.configurado) return;
    if (!previsao.valido) return;
    if (sensores.correntePainel < 1.0f) return; // Precisa de geração significativa

    // Hora atual
    struct tm ti;
    if (!getLocalTime(&ti)) return;
    uint8_t horaAtual = ti.tm_hour;

    // Busca previsão da hora atual
    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (ph.hora == horaAtual && ph.dia == ti.tm_mday &&
            ph.mes == (ti.tm_mon+1) && ph.ano == (ti.tm_year+1900)) {

            if (ph.geracaoEstimada < 10) return; // Estimativa muito baixa

            // Geração real atual (W)
            float geracaoReal = sensores.potenciaPainel;
            // Geração estimada (converte Wh em W médio da hora)
            float geracaoEst  = ph.geracaoEstimada;

            if (geracaoEst < 5) return;

            // Calcula fator de correção desta amostra
            float fatorNovo = geracaoReal / geracaoEst;

            // Limita fator entre 0.1 e 3.0
            if (fatorNovo < 0.1f) fatorNovo = 0.1f;
            if (fatorNovo > 3.0f) fatorNovo = 3.0f;

            // Média móvel ponderada (novas amostras têm mais peso)
            uint8_t maxAmostras = 50;
            if (configLocal.amostrasCalib < maxAmostras) {
                configLocal.amostrasCalib++;
            }

            // Peso: quanto mais amostras, mais estável fica
            float peso = 1.0f / (float)configLocal.amostrasCalib;
            configLocal.fatorCalib = configLocal.fatorCalib * (1.0f - peso)
                                   + fatorNovo * peso;

            // Salva a cada 10 amostras
            if (configLocal.amostrasCalib % 10 == 0) {
                salvarConfigLocal();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Calib solar: fator=%.3f amostras=%d",
                         configLocal.fatorCalib, configLocal.amostrasCalib);
                adicionarLog(tempBuf);
            }
            break;
        }
    }
}

// ============================================================
// CALCULAR GERAÇÃO ESTIMADA POR HORA
// Baseado em radiação solar + kWp instalado + eficiência + calibração
// ============================================================
float calcularGeracaoHora(float radiacaoWm2, float radiacaoDifusa,
                          float nuvens, float tempC) {
    if (!configLocal.configurado) return 0;
    if (configLocal.painelKWp <= 0) return 0;

    // Radiação total = direta + difusa
    float radTotal = radiacaoWm2 + radiacaoDifusa * 0.5f;

    // Irradiância efetiva considerando nuvens
    // (já vem calculada da API, mas nuvens afetam radiação difusa)
    if (radTotal < 0) radTotal = 0;

    // Correção por temperatura
    // Painéis perdem ~0.4% por grau acima de 25°C
    float correcaoTemp = 1.0f;
    if (tempC > 25.0f) {
        correcaoTemp = 1.0f - (tempC - 25.0f) * 0.004f;
        if (correcaoTemp < 0.6f) correcaoTemp = 0.6f;
    }

    // Geração estimada em Wh para esta hora
    // Fórmula: Radiação(W/m²) × Área_painel(m²) × eficiência × correções
    // Área estimada: kWp / (1000 W/m² × eficiência_painel_típica_0.2)
    // Simplificado: kWp × (radiação/1000) × eficiência_sistema
    float geracaoWh = configLocal.painelKWp * 1000.0f  // W de pico
                    * (radTotal / 1000.0f)               // Fração da irradiância de pico
                    * configLocal.eficiencia              // Eficiência do sistema
                    * correcaoTemp                        // Correção temperatura
                    * configLocal.fatorCalib;             // Calibração aprendida

    if (geracaoWh < 0) geracaoWh = 0;

    return geracaoWh;
}

// ============================================================
// BUSCAR DADOS METEOROLÓGICOS DA API OPEN-METEO
// ============================================================
void buscarPrevisaoMeteo() {
    if (!configLocal.configurado) {
        adicionarLog("Prev: local nao configurado");
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        adicionarLog("Prev: sem WiFi");
        return;
    }

    buscandoPrev = true;
    adicionarLog("Prev: buscando dados Open-Meteo...");

    // Monta URL da API Open-Meteo (gratuita, sem chave)
    // Variáveis: direct_normal_irradiance, diffuse_radiation,
    //            cloud_cover, temperature_2m
    char url[PREV_URL_MAX];
    snprintf(url, sizeof(url),
        "http://api.open-meteo.com/v1/forecast"
        "?latitude=%.8f"
        "&longitude=%.8f"
        "&hourly=direct_normal_irradiance,diffuse_radiation,"
                "cloud_cover,temperature_2m"
        "&timezone=America%%2FSao_Paulo"
        "&forecast_days=2"
        "&timeformat=unixtime",
        configLocal.latitude,
        configLocal.longitude);

    WiFiClient client;
    HTTPClient http;

    http.begin(client, url);
    http.setTimeout(10000); // 10s timeout

    int httpCode = http.GET();

    if (httpCode != 200) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "HTTP erro: %d", httpCode);
        adicionarLog(previsao.erro);
        http.end();
        buscandoPrev = false;
        return;
    }

    // Lê resposta
    String payload = http.getString();
    http.end();

    if (payload.length() < 100) {
        snprintf(previsao.erro, sizeof(previsao.erro), "Resposta vazia");
        adicionarLog("Prev: resposta vazia");
        buscandoPrev = false;
        return;
    }

    // ── Parser JSON manual (sem biblioteca extra) ─────────────
    // Open-Meteo retorna arrays de valores separados por vírgula
    // Formato: "direct_normal_irradiance":[val0,val1,...,val47]

    // Hora atual para calcular offset
    struct tm ti;
    if (!getLocalTime(&ti)) {
        adicionarLog("Prev: sem NTP para parsear");
        buscandoPrev = false;
        return;
    }

    // Extrai array de valores da string JSON
    auto extrairArray = [&](const char* chave,
                            float* arr, int maxLen) -> int {
        String busca = "\"";
        busca += chave;
        busca += "\":[";
        int pos = payload.indexOf(busca);
        if (pos < 0) return 0;
        pos += busca.length();

        int count = 0;
        while (count < maxLen) {
            int fim = payload.indexOf(',', pos);
            int fimArr = payload.indexOf(']', pos);

            if (fimArr >= 0 && (fim < 0 || fimArr < fim)) {
                // Último elemento
                arr[count++] = payload.substring(pos, fimArr).toFloat();
                break;
            }
            if (fim < 0) break;
            arr[count++] = payload.substring(pos, fim).toFloat();
            pos = fim + 1;
        }
        return count;
    };

    // Extrai timestamps
    auto extrairTimestamps = [&](long* arr, int maxLen) -> int {
        String busca = "\"time\":[";
        int pos = payload.indexOf(busca);
        if (pos < 0) return 0;
        pos += busca.length();

        int count = 0;
        while (count < maxLen) {
            int fim    = payload.indexOf(',', pos);
            int fimArr = payload.indexOf(']', pos);
            if (fimArr >= 0 && (fim < 0 || fimArr < fim)) {
                arr[count++] = payload.substring(pos, fimArr).toInt();
                break;
            }
            if (fim < 0) break;
            arr[count++] = payload.substring(pos, fim).toInt();
            pos = fim + 1;
        }
        return count;
    };

    // Arrays temporários
    float radDir[PREV_HORAS]  = {0};
    float radDif[PREV_HORAS]  = {0};
    float nuvens[PREV_HORAS]  = {0};
    float temp[PREV_HORAS]    = {0};
    long  stamps[PREV_HORAS]  = {0};

    int nRad  = extrairArray("direct_normal_irradiance", radDir,  PREV_HORAS);
    int nDif  = extrairArray("diffuse_radiation",        radDif,  PREV_HORAS);
    int nNuv  = extrairArray("cloud_cover",              nuvens,  PREV_HORAS);
    int nTemp = extrairArray("temperature_2m",           temp,    PREV_HORAS);
    int nTs   = extrairTimestamps(stamps, PREV_HORAS);

    if (nRad < 1 || nTs < 1) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "Parse falhou: rad=%d ts=%d", nRad, nTs);
        adicionarLog(previsao.erro);
        buscandoPrev = false;
        return;
    }

    int total = min(nRad, min(nTs, PREV_HORAS));

    // Preenche estrutura de previsão
    previsao.totalHoras          = 0;
    previsao.geracaoHojeRestante = 0;
    previsao.geracaoAmanha       = 0;
    previsao.geracaoTotal48h     = 0;

    int hojeAno = ti.tm_year + 1900;
    int hojeMes = ti.tm_mon + 1;
    int hojeDia = ti.tm_mday;

    for (int i = 0; i < total; i++) {
        // Converte timestamp Unix para hora local
        time_t ts = (time_t)stamps[i];
        struct tm* tl = localtime(&ts);

        PrevisaoHora& ph = previsao.horas[i];
        ph.hora           = tl->tm_hour;
        ph.dia            = tl->tm_mday;
        ph.mes            = tl->tm_mon + 1;
        ph.ano            = tl->tm_year + 1900;
        ph.radiacao       = i < nRad  ? radDir[i] : 0;
        ph.radiacaoDifusa = i < nDif  ? radDif[i] : 0;
        ph.coberturaNuvens= i < nNuv  ? nuvens[i] : 0;
        ph.temperatura    = i < nTemp ? temp[i]   : 25;
        ph.valido         = true;

        // Calcula geração estimada para esta hora
        ph.geracaoEstimada = calcularGeracaoHora(
            ph.radiacao, ph.radiacaoDifusa,
            ph.coberturaNuvens, ph.temperatura);

        previsao.geracaoTotal48h += ph.geracaoEstimada;

        // Hoje restante (horas futuras de hoje)
        if (ph.dia == hojeDia && ph.mes == hojeMes && ph.ano == hojeAno) {
            if (ph.hora >= (uint8_t)ti.tm_hour) {
                previsao.geracaoHojeRestante += ph.geracaoEstimada;
            }
        }

        // Amanhã total
        // Amanhã = dia seguinte
        time_t amanhaTs = mktime(&ti) + 86400;
        struct tm* amanha = localtime(&amanhaTs);
        if (ph.dia  == amanha->tm_mday &&
            ph.mes  == (amanha->tm_mon+1) &&
            ph.ano  == (amanha->tm_year+1900)) {
            previsao.geracaoAmanha += ph.geracaoEstimada;
        }

        previsao.totalHoras++;
    }

    previsao.valido           = true;
    previsao.ultimaAtualizacao = millis();
    memset(previsao.erro, 0, sizeof(previsao.erro));

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev OK: hoje+%.0fWh amanha+%.0fWh total48h=%.0fWh",
             previsao.geracaoHojeRestante,
             previsao.geracaoAmanha,
             previsao.geracaoTotal48h);
    adicionarLog(tempBuf);

    // Toma decisões automáticas após nova previsão
    tomarDecisoes();

    buscandoPrev = false;
}

// ============================================================
// TOMAR DECISÕES AUTOMÁTICAS
// Baseado em: previsão, histórico de consumo, bateria atual
// ============================================================
void tomarDecisoes() {
    if (!previsao.valido || !bateria.calibrada) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;

    // ── Calcula consumo médio por hora (baseado no histórico) ──
    float consumoMedioHora = 0;
    int   diasValidos = 0;

    // Usa últimos dias do histórico para média de consumo
    for (int i = 0; i < min((int)histTotalDias, 7); i++) {
        int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
        RegistroDia dTemp;
        bool ok = false;

        if (i == 0) {
            dTemp = histDiaAtual;
            ok    = true;
        } else {
            ok = carregarDiaFlash(idx, dTemp);
        }

        if (ok && dTemp.valido && dTemp.totalCasaWh > 0) {
            consumoMedioHora += dTemp.totalCasaWh / 24.0f;
            diasValidos++;
        }
    }

    if (diasValidos > 0) {
        consumoMedioHora /= diasValidos;
    } else {
        consumoMedioHora = 100.0f; // Estimativa padrão 100Wh/h
    }

    // ── Energia disponível atual ──────────────────────────────
    float energiaAtual    = bateria.energiaAtual;
    float energiaMin      = bateria.energiaDisponivel *
                            (redes.corteBaixa / 100.0f);

    // ── Horas até o amanhecer (estimativa) ───────────────────
    // Considera que o sol nasce às 6h e se põe às 18h (simplificado)
    // Com previsão real, usa quando geração > 0
    int   horaAtual       = ti.tm_hour;
    float horasAteAmanhecer = 0;

    if (horaAtual >= 18 || horaAtual < 6) {
        // Noite: calcula horas até 6h
        if (horaAtual >= 18) {
            horasAteAmanhecer = (24 - horaAtual) + 6;
        } else {
            horasAteAmanhecer = 6 - horaAtual;
        }
    }

    // Usa previsão real para saber quando vai começar a gerar
    float horaInicioGeracao = 6;
    for (int i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (ph.geracaoEstimada > 50 && // > 50Wh na hora
            (ph.dia == ti.tm_mday || ph.dia == ti.tm_mday + 1)) {
            horaInicioGeracao = ph.hora;
            break;
        }
    }

    if (horaAtual >= 18 || horaAtual < 6) {
        if (horaAtual >= 18) {
            horasAteAmanhecer = (24 - horaAtual) + horaInicioGeracao;
        } else {
            horasAteAmanhecer = horaInicioGeracao - horaAtual;
        }
    }

    // ── Energia necessária para passar a noite ────────────────
    float energiaNecessariaNoite = consumoMedioHora * horasAteAmanhecer;
    float energiaDisponivel      = energiaAtual - energiaMin;
    float saldoEnergia           = energiaDisponivel - energiaNecessariaNoite;

    // ── Consumo rede alta (estimativa: 40% do total) ──────────
    // Assume rede alta = 40% do consumo total
    float consumoRedeAltaHora = consumoMedioHora * 0.40f;
    float consumoLampExtHora  = consumoMedioHora * 0.05f;

    // ── DECISÃO 1: Rede Alta à noite ─────────────────────────
    decisao.desligarRedeAltaNoite = false;
    decisao.horaDesligarRedeAlta  = 0;
    decisao.horaLigarRedeAlta     = 0;

    bool eNoite = (horaAtual >= 18 || horaAtual < 6);

    if (eNoite) {
        // À noite: verifica se tem energia para rede alta
        if (saldoEnergia < 0) {
            // Energia insuficiente para passar a noite com rede alta
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Energia insuf. p/ noite: disp=%.0fWh nec=%.0fWh",
                     energiaDisponivel, energiaNecessariaNoite);
        } else if (saldoEnergia < consumoRedeAltaHora * 3) {
            // Margem baixa: desliga rede alta por precaução
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Margem baixa: saldo=%.0fWh <3h rede alta",
                     saldoEnergia);
        }

        // Hora para religar: quando geração > consumo
        decisao.horaLigarRedeAlta = horaInicioGeracao + 1;
    }

    // ── DECISÃO 2: Lâmpada Externa à noite ───────────────────
    decisao.desligarLampExtNoite = false;

    if (eNoite && sensores.correntePainel <= 0.5f) {
        // Se energia muito baixa, desliga lamp ext também
        if (saldoEnergia < consumoLampExtHora * horasAteAmanhecer) {
            decisao.desligarLampExtNoite = true;
        }
    }

    // ── DECISÃO 3: Alerta energia insuficiente ────────────────
    decisao.alertaEnergiaInsuficiente = false;
    if (eNoite && saldoEnergia < 0 &&
        previsao.geracaoAmanha < consumoMedioHora * 12) {
        // Noite com pouca energia E amanhã pouca geração
        decisao.alertaEnergiaInsuficiente = true;
    }

    // Aplica decisões
    if (decisao.desligarRedeAltaNoite && estados.redesAlta &&
        curto.estado == CURTO_NORMAL) {
        desligarRedeAlta();
        adicionarLog("DECISAO: Rede Alta OFF - economia noturna");
        adicionarLog(decisao.motivo);
    }

    if (decisao.desligarLampExtNoite && estados.lampExt) {
        desligarLampExt();
        adicionarLog("DECISAO: Lamp.Ext OFF - economia energia noite");
    }

    // Log do saldo
    snprintf(tempBuf, sizeof(tempBuf),
             "Decisao: saldo=%.0fWh cons_noite=%.0fWh horas=%.1fh",
             saldoEnergia, energiaNecessariaNoite, horasAteAmanhecer);
    adicionarLog(tempBuf);
}

// ============================================================
// VERIFICAR RELIGAMENTO (chamar no loop)
// Religa rede alta de manhã se foi desligada automaticamente
// ============================================================
void verificarReligamentoAuto() {
    if (!previsao.valido) return;
    if (!decisao.desligarRedeAltaNoite) return;
    if (curto.estado != CURTO_NORMAL) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;

    // Verifica se chegou a hora de religar
    bool deveReligar = false;

    // Religa se: painel gerando + bateria com energia suficiente
    if (sensores.correntePainel > 0.5f &&
        bateria.percentualAtual > redes.corteAlta + 5.0f &&
        sensores.inversoreAtivo) {
        deveReligar = true;
    }

    // Ou se chegou a hora programada de religar
    if ((float)ti.tm_hour >= decisao.horaLigarRedeAlta &&
        bateria.percentualAtual > redes.corteAlta + 2.0f &&
        sensores.inversoreAtivo) {
        deveReligar = true;
    }

    if (deveReligar && !estados.redesAlta) {
        ligarRedeAlta();
        decisao.desligarRedeAltaNoite = false;
        adicionarLog("DECISAO: Rede Alta ON - religamento automatico manha");
    }
}

// ============================================================
// CSS COMPARTILHADO (retorna String com estilo)
// ============================================================
// Gerador de página HTML completa
String gerarPagina(const String& titulo, const String& conteudo,
                   bool comNav = true) {
    String html = F("<!DOCTYPE html><html><head>"
                    "<meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>"
        "body{background:#0d0d0d;color:#e0e0e0;font-family:Arial,sans-serif;margin:0;padding:10px;}"
        ".card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:15px;"
               "max-width:520px;margin:0 auto;}"
        "h2{color:#4fc3f7;text-align:center;margin:0 0 15px 0;font-size:18px;}"
        "h3{color:#81d4fa;font-size:14px;margin:12px 0 8px;border-bottom:1px solid #333;padding-bottom:6px;}"
        "label{display:block;color:#90a4ae;font-size:12px;margin-top:10px;margin-bottom:3px;}"
        "input,select{width:100%;padding:9px;background:#111;border:1px solid #444;"
                     "border-radius:4px;color:#e0e0e0;font-size:13px;box-sizing:border-box;}"
        "input:focus,select:focus{outline:none;border-color:#0277bd;}"
        "button{width:100%;padding:11px;margin-top:12px;background:#0277bd;color:#fff;"
               "border:none;border-radius:4px;font-size:14px;font-weight:bold;cursor:pointer;}"
        "button:hover{background:#039be5;}"
        ".btn-r{background:#c62828;}.btn-r:hover{background:#ef5350;}"
        ".btn-g{background:#2e7d32;}.btn-g:hover{background:#43a047;}"
        ".btn-o{background:#ef6c00;}.btn-o:hover{background:#f57c00;}"
        ".btn-s{background:#546e7a;}.btn-s:hover{background:#607d8b;}"
        ".btn-sm{font-size:12px;padding:8px;}"
        ".ok{background:#1b5e20;color:#a5d6a7;padding:10px;border-radius:4px;margin:8px 0;text-align:center;}"
        ".er{background:#b71c1c;color:#ef9a9a;padding:10px;border-radius:4px;margin:8px 0;text-align:center;}"
        ".wn{background:#e65100;color:#ffe0b2;padding:10px;border-radius:4px;margin:8px 0;text-align:center;}"
        ".nf{background:#0d47a1;color:#90caf9;padding:10px;border-radius:4px;margin:8px 0;font-size:12px;line-height:1.5;}"
        ".row{display:flex;gap:8px;margin-top:10px;}"
        ".row button{margin-top:0;}"
        // Navegação inferior
        ".nav{display:flex;gap:4px;margin-top:12px;padding-top:10px;border-top:1px solid #333;}"
        ".nav a{flex:1;text-align:center;padding:9px 4px;background:#1e1e1e;color:#90a4ae;"
               "text-decoration:none;border-radius:4px;font-size:11px;border:1px solid #333;}"
        ".nav a.active{background:#0277bd;color:#fff;border-color:#0277bd;}"
        ".nav a:hover{background:#263238;color:#e0e0e0;}"
        // Cards de status
        ".sc{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:6px 0;}"
        ".sc .lbl{color:#546e7a;font-size:11px;margin-bottom:4px;}"
        ".sc .val{color:#e0e0e0;font-size:20px;font-weight:bold;}"
        ".sc .sub{color:#78909c;font-size:11px;margin-top:2px;}"
        ".sc.ok{border-color:#2e7d32;background:#0a1f0a;}"
        ".sc.wn{border-color:#ef6c00;background:#1f130a;}"
        ".sc.er{border-color:#c62828;background:#1f0a0a;}"
        // Barra de progresso
        ".barra-bg{background:#111;border:1px solid #333;border-radius:4px;"
                  "height:18px;margin:6px 0;overflow:hidden;}"
        ".barra{height:100%;border-radius:4px;transition:width .5s;text-align:center;"
               "line-height:18px;font-size:11px;font-weight:bold;color:#fff;}"
        // Log box
        ".log-t{color:#546e7a;font-size:10px;text-transform:uppercase;margin-top:15px;margin-bottom:4px;}"
        ".log-box{background:#000;border:1px solid #222;border-radius:4px;padding:8px;"
                 "height:160px;overflow-y:auto;font-family:monospace;font-size:10px;color:#76ff03;}"
        ".log-l{padding:2px 0;border-bottom:1px solid #0a0a0a;}"
        "</style></head><body><div class='card'>");
    html += "<h2>&#9889; RRFS Energia</h2>";
    html += conteudo;

    // Navegação
    if (comNav && logado) {
        String pag = server.uri();
        html += "<div class='nav'>";
        html += "<a href='/'         " + String(pag=="/"?        "class='active'":"") + ">&#128202; Painel</a>";
        html += "<a href='/bateria'  " + String(pag=="/bateria"? "class='active'":"") + ">&#128267; Bateria</a>";
        html += "<a href='/redes'    " + String(pag=="/redes"?   "class='active'":"") + ">&#128268; Redes</a>";
        html += "<a href='/sensores' " + String(pag=="/sensores"?"class='active'":"") + ">&#128268; Sensores</a>";
        html += "<a href='/config'   " + String(pag=="/config"?  "class='active'":"") + ">&#9881; Config</a>";
        html += "<a href='/historico' " + String(pag=="/historico"?"class='active'":"") + ">&#128200; Hist.</a>";
        html += "<a href='/previsao' " + String(pag=="/previsao"?"class='active'":"") + ">&#127774; Prev.</a>";
        html += "</div>";
    }

    // Logs
    html += F("<div class='log-t'>&#128221; Logs do sistema</div>"
              "<div class='log-box' id='lb'><div style='color:#666'>Carregando...</div></div>"
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
              "for(var i=0;i<a.length;i++){h+='<div class=\"log-l\">'+a[i]+'</div>';}"
              "d.innerHTML=h||'<div style=\"color:#666\">Sem logs</div>';"
              "d.scrollTop=d.scrollHeight;"
              "}catch(e){}}}"
              ";x.open('GET','/logs',true);x.send();}"
              "setInterval(aL,4000);aL();"
              "</script></body></html>");
    return html;
}

// ============================================================
// HELPERS HTML
// ============================================================

// Barra de progresso colorida
String barraProgresso(float pct, const String& label = "") {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    String cor;
    if (pct >= 50)      cor = "#2e7d32";  // Verde
    else if (pct >= 25) cor = "#ef6c00";  // Laranja
    else                cor = "#c62828";  // Vermelho

    String html = "<div class='barra-bg'><div class='barra' style='width:";
    html += String((int)pct);
    html += "%;background:" + cor + "'>";
    html += String((int)pct) + "%";
    html += "</div></div>";
    if (label.length()) {
        html += "<div style='font-size:11px;color:#546e7a;text-align:center;'>" + label + "</div>";
    }
    return html;
}

// Card de status
String statusCard(const String& icone, const String& label,
                  const String& valor, const String& sub,
                  const String& tipo = "") {
    String cls = "sc";
    if (tipo.length()) cls += " " + tipo;
    String h = "<div class='" + cls + "'>";
    h += "<div class='lbl'>" + icone + " " + label + "</div>";
    h += "<div class='val'>" + valor + "</div>";
    if (sub.length()) h += "<div class='sub'>" + sub + "</div>";
    h += "</div>";
    return h;
}

// Formata corrente com sinal e unidade
String fmtCorrente(float a) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%.1fA", a);
    return String(buf);
}

// Formata potência
String fmtPotencia(float w) {
    char buf[20];
    if (fabsf(w) >= 1000)
        snprintf(buf, sizeof(buf), "%.2fkW", w / 1000.0f);
    else
        snprintf(buf, sizeof(buf), "%.0fW", w);
    return String(buf);
}

// ============================================================
// PÁGINA: / (PAINEL PRINCIPAL)
// ============================================================
void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login"); server.send(302); return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location","/setup-wifi");  server.send(302); return;
    }
    if (!logado) {
        server.sendHeader("Location","/login");       server.send(302); return;
    }

    EstimativaTempo tempo = calcularTempoRestante();

    // ── Estados booleanos ────────────────────────────────────
    bool painelGerando    = (sensores.correntePainel  >  0.5f);
    bool batCarregando    = (sensores.correnteBateria >  0.3f);
    bool batDescarregando = (sensores.correnteBateria < -0.3f);
    bool casaConsumindo   = (sensores.correnteCasa    >  0.3f);
    bool invAtivo         = sensores.inversoreAtivo;
    bool redeAltaON       = estados.redesAlta;
    bool redeBaixaON      = estados.redeBaixa;
    bool lampExtON        = estados.lampExt;
    bool lampIntON        = estados.lampInt;
    bool curtoAlt         = (curto.estado == CURTO_CONFIRMADO_ALTA  || curto.redeAltaCurto);
    bool curtoBx          = (curto.estado == CURTO_CONFIRMADO_BAIXA || curto.redeBaixaCurto);
    bool diagAtivo        = (curto.estado == CURTO_TESTANDO_ALTA    ||
                             curto.estado == CURTO_TESTANDO_BAIXA   ||
                             curto.estado == CURTO_RELIGANDO);

    // ── Textos ───────────────────────────────────────────────
    char strPainel[12], strCasa[12], strBat[12];
    char strPP[12],     strPC[12],   strPB[12];
    snprintf(strPainel,sizeof(strPainel),"%.1fA",fabsf(sensores.correntePainel));
    snprintf(strCasa,  sizeof(strCasa),  "%.1fA",sensores.correnteCasa);
    snprintf(strBat,   sizeof(strBat),   "%.1fA",fabsf(sensores.correnteBateria));
    snprintf(strPP,    sizeof(strPP),    "%.0fW", fabsf(sensores.potenciaPainel));
    snprintf(strPC,    sizeof(strPC),    "%.0fW", sensores.potenciaCasa);
    snprintf(strPB,    sizeof(strPB),    "%.0fW", fabsf(sensores.potenciaBateria));
    char pctBuf[8];
    snprintf(pctBuf,sizeof(pctBuf),"%.0f%%",
             bateria.calibrada ? bateria.percentualAtual : 0.0f);

    // ── Tempos por carga ─────────────────────────────────────
    String tLampInt="--",tLampExt="--",tRedeAlt="--",tRedeBx="--";
    if (batDescarregando && bateria.calibrada) {
        float tx = sensores.potenciaBateria;
        tLampInt = tempoAteCorte(redes.corteLampInt, tx);
        tLampExt = tempoAteCorte(redes.corteLampExt, tx);
        tRedeAlt = tempoAteCorte(redes.corteAlta,    tx);
        tRedeBx  = tempoAteCorte(redes.corteBaixa,   tx);
    }

    // ── Cores ────────────────────────────────────────────────
    // Painel: amarelo se gerando, cinza se não
    String corPainel  = painelGerando    ? "#fdd835" : "#455a64";
    // Bateria: verde/laranja/vermelho por %
    String corBat     = !bateria.calibrada          ? "#455a64" :
                         bateria.percentualAtual>50  ? "#43a047" :
                         bateria.percentualAtual>25  ? "#ef6c00" : "#c62828";
    String corInv     = invAtivo     ? "#4fc3f7" : "#455a64";
    String corAlt     = curtoAlt     ? "#ff6f00" : redeAltaON  ? "#ef5350" : "#455a64";
    String corBaixa   = curtoBx      ? "#ff6f00" : redeBaixaON ? "#42a5f5" : "#455a64";
    String corLampExt = lampExtON    ? "#ffd54f" : "#455a64";
    String corLampInt = lampIntON    ? "#ffd54f" : "#455a64";

    // ── Fluxos de energia (LÓGICA CORRETA) ───────────────────
    // fluxo painel→bateria: painel gerando E bateria carregando
    bool fluxoPB = painelGerando && batCarregando;
    // fluxo painel→casa: painel gerando E casa consumindo
    bool fluxoPC = painelGerando && casaConsumindo;
    // fluxo bateria→casa: bateria descarregando (fornece casa)
    bool fluxoBC = batDescarregando && casaConsumindo;

    // cores das setas conforme fonte
    String corSetaPB = fluxoPB ? "#43a047" : "#1e272e"; // verde=carregando
    String corSetaPC = fluxoPC ? "#fdd835" : "#1e272e"; // amarelo=painel direto
    String corSetaBC = fluxoBC ? "#ef6c00" : "#1e272e"; // laranja=bateria

    String c = "";

    if (!bateria.calibrada) {
        c += "<div class='wn' style='font-size:11px;margin-bottom:6px'>"
             "&#9888; Bateria nao calibrada. "
             "<a href='/bateria' style='color:#ffe0b2'>Calibrar agora</a></div>";
    }

    // ============================================================
    // CSS - página expandida sem max-width no card do diagrama
    // ============================================================
    c += F("<style>"
        // Animação seta horizontal
        "@keyframes flowR{"
            "0%  {stroke-dashoffset:20}"
            "100%{stroke-dashoffset:0}}"
        // Animação seta vertical
        "@keyframes flowD{"
            "0%  {stroke-dashoffset:20}"
            "100%{stroke-dashoffset:0}}"
        ".flow-ativo{"
            "stroke-dasharray:8 4;"
            "animation:flowR 0.6s linear infinite;"
        "}"
        ".flow-inativo{opacity:0.1}"

        // Box componente (painel / bateria)
        ".bx{"
            "border-radius:10px;"
            "border:2px solid #333;"
            "background:#1a1a1a;"
            "padding:10px 8px;"
            "text-align:center;"
        "}"
        ".bx-ic{font-size:28px}"
        ".bx-nm{"
            "font-size:10px;color:#546e7a;"
            "text-transform:uppercase;margin-top:3px"
        "}"
        ".bx-vl{font-size:16px;font-weight:bold;margin-top:4px}"
        ".bx-sb{font-size:10px;color:#607d8b;margin-top:2px}"

        // Box carga (casa)
        ".cg{"
            "border-radius:8px;"
            "border:2px solid #222;"
            "background:#111;"
            "padding:9px 11px;"
            "margin-bottom:5px;"
        "}"
        ".cg-row{"
            "display:flex;"
            "justify-content:space-between;"
            "align-items:center;"
        "}"
        ".cg-nm{font-size:12px;color:#90a4ae}"
        ".cg-st{font-size:13px;font-weight:bold}"
        ".cg-inf{"
            "font-size:10px;color:#546e7a;"
            "margin-top:5px;"
            "padding-top:5px;"
            "border-top:1px solid #1e1e1e;"
        "}"
    "</style>");

    // ============================================================
    // TÍTULO
    // ============================================================
    c += "<h3 style='text-align:center;margin:4px 0 8px'>&#9889; Fluxo de Energia</h3>";

    // ============================================================
    // DIAGRAMA SVG + HTML
    //
    // Layout (largura total disponível):
    //
    //  [  PAINEL  ]---seta PB---[BATERIA]
    //       |                      |
    //     seta PC               seta BC
    //       |                      |
    //       +----------+-----------+
    //                  |
    //             [ CASA / cargas ]
    //
    // Usamos SVG para as setas e divs absolutos para os boxes
    // ============================================================

    // Container relativo para posicionamento
    // Largura 100%, altura suficiente para o SVG
    c += "<div style='position:relative;width:100%;margin-bottom:8px'>";

    // ── SVG das setas ─────────────────────────────────────────
    // Coordenadas (em %):
    //   Painel centro:  x=20%, y=60px
    //   Bateria centro: x=80%, y=60px
    //   Casa topo:      x=50%, y=180px
    //
    // Usamos viewBox para escalar
    c += "<svg width='100%' height='160px' viewBox='0 0 320 160' "
         "xmlns='http://www.w3.org/2000/svg' style='position:absolute;top:0;left:0'>";

    // Definição da seta (marker)
    c += "<defs>"
         "<marker id='ar' markerWidth='6' markerHeight='6' "
         "refX='5' refY='3' orient='auto'>"
         "<path d='M0,0 L6,3 L0,6 Z' fill='currentColor'/>"
         "</marker>"
         "</defs>";

    // ── Seta Painel → Bateria (horizontal, topo) ──
    // Painel centro x=80, Bateria inicio x=240
    c += "<line x1='88' y1='55' x2='232' y2='55' "
         "stroke='" + corSetaPB + "' stroke-width='3' "
         "stroke-linecap='round' "
         "class='" + String(fluxoPB ? "flow-ativo" : "flow-inativo") + "'/>";
    // Ponta da seta
    if (fluxoPB) {
        c += "<polygon points='232,49 244,55 232,61' fill='" + corSetaPB + "'/>";
    }
    // Label corrente na seta PB
    if (fluxoPB) {
        c += "<text x='160' y='48' text-anchor='middle' "
             "font-size='10' fill='" + corSetaPB + "'>";
        c += String(strBat); // corrente indo para bateria
        c += "</text>";
    }

    // ── Seta Painel → Casa (vertical, esquerda) ──
    // Painel base y=88, Casa topo y=140
    c += "<line x1='80' y1='90' x2='80' y2='128' "
         "stroke='" + corSetaPC + "' stroke-width='3' "
         "stroke-linecap='round' "
         "class='" + String(fluxoPC ? "flow-ativo" : "flow-inativo") + "'/>";
    // Seta diagonal para centro
    c += "<line x1='80' y1='128' x2='148' y2='148' "
         "stroke='" + corSetaPC + "' stroke-width='3' "
         "stroke-linecap='round' "
         "class='" + String(fluxoPC ? "flow-ativo" : "flow-inativo") + "'/>";
    if (fluxoPC) {
        c += "<polygon points='142,141 154,149 144,156' fill='" + corSetaPC + "'/>";
    }

    // ── Seta Bateria → Casa (vertical, direita) ──
    c += "<line x1='240' y1='90' x2='240' y2='128' "
         "stroke='" + corSetaBC + "' stroke-width='3' "
         "stroke-linecap='round' "
         "class='" + String(fluxoBC ? "flow-ativo" : "flow-inativo") + "'/>";
    c += "<line x1='240' y1='128' x2='172' y2='148' "
         "stroke='" + corSetaBC + "' stroke-width='3' "
         "stroke-linecap='round' "
         "class='" + String(fluxoBC ? "flow-ativo" : "flow-inativo") + "'/>";
    if (fluxoBC) {
        c += "<polygon points='178,141 166,149 176,156' fill='" + corSetaBC + "'/>";
    }

    c += "</svg>";

    // ── Boxes sobre o SVG ─────────────────────────────────────
    // Linha superior: Painel (esquerda) e Bateria (direita)
    c += "<div style='display:flex;justify-content:space-between;"
         "position:relative;z-index:1'>";

    // PAINEL
    c += "<div class='bx' style='width:42%;border-color:" + corPainel + "'>";
    c += "<div class='bx-ic'>&#9728;</div>";
    c += "<div class='bx-nm'>Painel Solar</div>";
    c += "<div class='bx-vl' style='color:" + corPainel + "'>";
    c += painelGerando ? strPainel : "0.0A";
    c += "</div>";
    c += "<div class='bx-sb'>";
    if (painelGerando) {
        c += String(strPP);
        if (fluxoPB && fluxoPC)     c += " &rarr; Bat+Casa";
        else if (fluxoPB)           c += " &rarr; Bateria";
        else if (fluxoPC)           c += " &rarr; Casa";
    } else {
        c += "Sem gera&ccedil;&atilde;o";
    }
    c += "</div></div>";

    // BATERIA
    {
        String icoBat = batCarregando    ? "&#9650;&#128267;" :
                        batDescarregando ? "&#9660;&#128267;" : "&#128267;";
        String subBat;
        if (!bateria.calibrada) {
            subBat = "Calibrar";
        } else if (batCarregando) {
            subBat = String(strBat) + " entrando";
        } else if (batDescarregando) {
            subBat = String(strBat) + " saindo";
        } else {
            subBat = "Equilibrio";
        }

        c += "<div class='bx' style='width:42%;border-color:" + corBat + "'>";
        c += "<div class='bx-ic'>" + icoBat + "</div>";
        c += "<div class='bx-nm'>Bateria</div>";
        c += "<div class='bx-vl' style='color:" + corBat + "'>";
        c += bateria.calibrada ? String(pctBuf) : "N/Cal";
        c += "</div>";
        c += "<div class='bx-sb'>" + subBat + "</div>";
        c += "</div>";
    }

    c += "</div>"; // fecha linha superior

    // Espaço para o SVG das setas (altura fixa)
    c += "<div style='height:75px'></div>";

    // ── CASA - título ─────────────────────────────────────────
    c += "<div style='background:#1a1a1a;border:2px solid #263238;"
         "border-radius:10px;padding:8px 12px;"
         "display:flex;align-items:center;gap:10px;"
         "margin-bottom:6px;position:relative;z-index:1'>";
    c += "<span style='font-size:26px'>&#127968;</span>";
    c += "<div style='flex:1'>";
    c += "<div style='font-size:10px;color:#546e7a;text-transform:uppercase'>Casa</div>";
    if (casaConsumindo) {
        c += "<div style='font-size:14px;font-weight:bold;color:#4fc3f7'>";
        c += strCasa; c += " &bull; "; c += strPC;
        // Fonte de energia
        if (fluxoPC && fluxoBC)
            c += " <span style='font-size:10px;color:#78909c'>(Painel + Bateria)</span>";
        else if (fluxoPC)
            c += " <span style='font-size:10px;color:#78909c'>(Painel)</span>";
        else if (fluxoBC)
            c += " <span style='font-size:10px;color:#78909c'>(Bateria)</span>";
        c += "</div>";
    } else {
        c += "<div style='font-size:12px;color:#455a64'>Sem consumo detectado</div>";
    }
    c += "</div>";
    if (diagAtivo) {
        c += "<div style='font-size:10px;color:#90caf9;text-align:right'>"
             "&#128270;<br>Diagn.</div>";
    }
    c += "</div>";

    c += "</div>"; // fecha container relativo

    // ── CARGAS ────────────────────────────────────────────────
    // Helper lambda para gerar linha de carga
    auto carga = [&](const char* icone, const char* nome,
                     const String& cor, const String& bgCor,
                     bool ligado, const String& statusTxt,
                     const String& infoTxt) -> String {
        String h = "<div class='cg' style='border-color:" + cor +
                   ";background:" + bgCor + "'>";
        h += "<div class='cg-row'>";
        h += "<span class='cg-nm'>" + String(icone) + " " + String(nome) + "</span>";
        h += "<span class='cg-st' style='color:" + cor + "'>";
        h += (ligado ? "&#9679; " : "&#9675; ") + statusTxt;
        h += "</span></div>";
        if (infoTxt.length() > 0) {
            h += "<div class='cg-inf'>" + infoTxt + "</div>";
        }
        h += "</div>";
        return h;
    };

    // ── Inversor ──
    {
        String st   = invAtivo ? "ATIVO" : (estados.inversor ? "AGUARD" : "OFF");
        String info = "";
        if (diagAtivo)
            info = "<span style='color:#90caf9'>&#128270; Diagnosticando...</span>";
        else if (estados.inversor && !invAtivo)
            info = "<span style='color:#78909c'>Aguardando sinal...</span>";
        c += carga("&#9889;","Inversor", corInv,
                   invAtivo ? "#081824" : "#0d0d0d",
                   invAtivo, st, info);
    }

    // ── Rede Alta ──
    {
        String st   = curtoAlt ? "CURTO!" : (redeAltaON ? "ON" : "OFF");
        String bg   = curtoAlt ? "#1a0f00" : redeAltaON ? "#180808" : "#0d0d0d";
        String info = "";
        if (curtoAlt) {
            info = "<span style='color:#ff6f00'>&#9888; Curto detectado &bull; "
                   "<a href='/redes' style='color:#ffcc80'>Resolver &rarr;</a></span>";
        } else if (redeAltaON && batDescarregando && tRedeAlt != "--") {
            String corT = (tRedeAlt=="Agora") ? "#ef5350" : "#78909c";
            info = "&#9660; Desliga em &lt;" +
                   String(redes.corteAlta,0) + "%&gt;: "
                   "<span style='color:" + corT + ";font-weight:bold'>" +
                   tRedeAlt + "</span>";
        } else if (!redeAltaON && !curtoAlt) {
            info = "Desligada por nivel de energia";
        }
        c += carga("&#128268;","Rede Alta (Potencia)", corAlt, bg,
                   redeAltaON && !curtoAlt, st, info);
    }

    // ── Rede Baixa ──
    {
        String st   = curtoBx ? "CURTO!" : (redeBaixaON ? "ON" : "OFF");
        String bg   = curtoBx ? "#1a0f00" : redeBaixaON ? "#080d18" : "#0d0d0d";
        String info = "";
        if (curtoBx) {
            info = "<span style='color:#ff6f00'>&#9888; Curto detectado &bull; "
                   "<a href='/redes' style='color:#ffcc80'>Resolver &rarr;</a></span>";
        } else if (redeBaixaON && batDescarregando && tRedeBx != "--") {
            String corT = (tRedeBx=="Agora") ? "#ef5350" : "#78909c";
            info = "&#9660; Desliga em &lt;" +
                   String(redes.corteBaixa,0) + "%&gt;: "
                   "<span style='color:" + corT + ";font-weight:bold'>" +
                   tRedeBx + "</span>";
        } else if (!redeBaixaON && !curtoBx) {
            info = "Desligada - nivel critico de energia";
        }
        c += carga("&#128268;","Rede Baixa (Tomadas)", corBaixa, bg,
                   redeBaixaON && !curtoBx, st, info);
    }

    // ── Lâmpada Externa ──
    {
        String info = "";
        if (lampExtON && batDescarregando && tLampExt != "--") {
            String corT = (tLampExt=="Agora") ? "#ef5350" : "#78909c";
            info = "&#9660; Desliga em &lt;" +
                   String(redes.corteLampExt,0) + "%&gt;: "
                   "<span style='color:" + corT + ";font-weight:bold'>" +
                   tLampExt + "</span>";
        } else if (!lampExtON) {
            info = "Desligada";
        }
        c += carga("&#128161;","Lamp. Externa", corLampExt,
                   lampExtON ? "#181500" : "#0d0d0d",
                   lampExtON,
                   lampExtON ? "ON" : "OFF", info);
    }

    // ── Lâmpada Interna ──
    {
        String info = "";
        if (lampIntON && batDescarregando && tLampInt != "--") {
            String corT = (tLampInt=="Agora") ? "#ef5350" : "#78909c";
            info = "&#9660; Desliga em &lt;" +
                   String(redes.corteLampInt,0) + "%&gt;: "
                   "<span style='color:" + corT + ";font-weight:bold'>" +
                   tLampInt + "</span>";
        } else if (!lampIntON) {
            info = "Desligada";
        }
        c += carga("&#128161;","Lamp. Interna", corLampInt,
                   lampIntON ? "#181500" : "#0d0d0d",
                   lampIntON,
                   lampIntON ? "ON" : "OFF", info);
    }

    // Alerta geral curto
    if (curtoAlt || curtoBx || curto.estado == CURTO_AGUARDANDO_RESET) {
        c += "<div style='background:#7f0000;border:2px solid #ef5350;"
             "border-radius:8px;padding:10px;text-align:center;margin-top:4px'>";
        c += "<div style='font-size:13px;font-weight:bold;color:#ef9a9a'>"
             "&#9888; ";
        if (curtoAlt)                               c += "CURTO NA REDE ALTA";
        if (curtoBx)                                c += "CURTO NA REDE BAIXA";
        if (curto.estado==CURTO_AGUARDANDO_RESET)   c += "FALHA NO INVERSOR";
        c += "</div>";
        c += "<a href='/redes' style='color:#ffcc80;font-size:12px'>"
             "Toque aqui para resolver &rarr;</a>";
        c += "</div>";
    }

    // ============================================================
    // BARRA BATERIA
    // ============================================================
    c += "<div style='margin:10px 0'>";
    c += "<div style='display:flex;justify-content:space-between;"
         "align-items:center;margin-bottom:4px'>";
    c += "<span style='font-size:11px;color:#78909c'>&#128267; Bateria</span>";

    if (tempo.valido) {
        char bufT[50];
        if (tempo.carregando) {
            if (tempo.horas==0 && tempo.minutos==0)
                snprintf(bufT,sizeof(bufT),"Carregada!");
            else
                snprintf(bufT,sizeof(bufT),"Completa em %dh%02dm",
                         tempo.horas,tempo.minutos);
        } else {
            if (tempo.horas==0 && tempo.minutos<5)
                snprintf(bufT,sizeof(bufT),"ENERGIA CRITICA!");
            else
                snprintf(bufT,sizeof(bufT),"Restam %dh%02dm",
                         tempo.horas,tempo.minutos);
        }
        String corT = tempo.carregando ? "#43a047" :
                     (tempo.horas==0 && tempo.minutos<30 ? "#ef5350" : "#ef6c00");
        c += "<span style='font-size:11px;font-weight:bold;color:"+corT+"'>";
        c += tempo.carregando ? "&#9650; " : "&#9660; ";
        c += String(bufT)+"</span>";
    }
    c += "</div>";

    if (bateria.calibrada) {
        float  pct     = bateria.percentualAtual;
        String corBrr  = pct>=50?"#2e7d32":pct>=25?"#ef6c00":"#c62828";
        c += "<div class='barra-bg'>"
             "<div class='barra' style='width:"+String((int)pct)+"%;"
             "background:"+corBrr+"'>"+String((int)pct)+"%</div></div>";
        c += "<div style='display:flex;justify-content:space-between;"
             "margin-top:3px;font-size:10px;color:#546e7a'>";
        char bWh[60];
        snprintf(bWh,sizeof(bWh),"%.0f / %.0f Wh",
                 bateria.energiaAtual,bateria.energiaDisponivel);
        c += "<span>"+String(bWh)+"</span>";
        char bPot[30];
        snprintf(bPot,sizeof(bPot),"%.0fW %s",fabsf(sensores.potenciaBateria),
                 batCarregando?"entrando":batDescarregando?"saindo":"equil.");
        c += "<span>"+String(bPot)+"</span></div>";
    } else {
        c += "<div class='wn' style='font-size:11px;text-align:center'>"
             "Nao calibrada - "
             "<a href='/bateria' style='color:#ffe0b2'>calibrar</a></div>";
    }
    c += "</div>";

    // ============================================================
    // CARDS POTÊNCIA
    // ============================================================
    c += "<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
         "gap:6px;margin:4px 0'>";

    auto cardPot = [&](const char* ico, const char* nome,
                       const String& val, const String& sub,
                       const String& cor, bool destaque) -> String {
        String h = "<div class='sc";
        if (destaque) h += " ok";
        h += "' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>"+String(ico)+" "+String(nome)+"</div>";
        h += "<div style='font-size:15px;font-weight:bold;color:"+cor+"'>";
        h += val+"</div>";
        h += "<div class='sub'>"+sub+"</div></div>";
        return h;
    };

    c += cardPot("&#9728;","Painel",
                 painelGerando?strPainel:"0.0A",
                 painelGerando?strPP:"Sem geracao",
                 corPainel, painelGerando);

    c += cardPot("&#127968;","Casa",
                 strCasa, strPC, "#4fc3f7", false);

    {
        String corF  = batCarregando?"#43a047":batDescarregando?"#ef6c00":"#fdd835";
        String sinal = batCarregando?"+":batDescarregando?"-":"~";
        c += cardPot("&#128267;","Bateria",
                     sinal+String(strBat), strPB, corF, false);
    }
    c += "</div>";

    // ============================================================
    // INFO SISTEMA
    // ============================================================
    String ip   = modoAP?WiFi.softAPIP().toString():WiFi.localIP().toString();
    String modo = modoAP?"AP":"WiFi";
    c += "<div class='nf' style='font-size:10px;margin-top:4px'>";
    c += "&#128246; "+modo+" | "+ip;
    if (!modoAP && WiFi.status()==WL_CONNECTED)
        c += " | "+String(WiFi.RSSI())+"dBm";
    snprintf(tempBuf,sizeof(tempBuf)," | RAM:%.1fKB | Up:%lus",
             esp_get_free_heap_size()/1024.0f,millis()/1000);
    c += String(tempBuf)+"</div>";

    c += "<script>setTimeout(function(){location.reload();},8000);</script>";

    server.send(200,"text/html; charset=utf-8",
                gerarPagina("RRFS - Painel",c));
}

// ============================================================
// ENDPOINT: /dados (JSON para AJAX)
// ============================================================
void handleDados() {
    if (!logado) { server.send(403); return; }

    char json[512];
    snprintf(json, sizeof(json),
             "{"
             "\"ip\":%.1f,\"ic\":%.1f,\"ib\":%.1f,"
             "\"pp\":%.0f,\"pc\":%.0f,\"pb\":%.0f,"
             "\"pct\":%.1f,\"enrg\":%.0f,\"disp\":%.0f,"
             "\"inv\":%s,\"rb\":%s,\"ra\":%s,\"le\":%s,\"li\":%s,"
             "\"calib\":%s"
             "}",
             sensores.correntePainel,
             sensores.correnteCasa,
             sensores.correnteBateria,
             sensores.potenciaPainel,
             sensores.potenciaCasa,
             sensores.potenciaBateria,
             bateria.percentualAtual,
             bateria.energiaAtual,
             bateria.energiaDisponivel,
             sensores.inversoreAtivo ? "true":"false",
             estados.redeBaixa  ? "true":"false",
             estados.redesAlta  ? "true":"false",
             estados.lampExt    ? "true":"false",
             estados.lampInt    ? "true":"false",
             bateria.calibrada  ? "true":"false"
    );
    server.send(200, "application/json", json);
}

// ============================================================
// PÁGINA: /bateria
// ============================================================
void handleBateria() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            float vnom = server.arg("vnom").toFloat();
            float ah   = server.arg("ah").toFloat();
            float desc = server.arg("desc").toFloat();

            if (vnom < 6 || vnom > 60) {
                msg = "<div class='er'>&#10007; Tensao invalida (6-60V)</div>";
            } else if (ah < 1 || ah > 5000) {
                msg = "<div class='er'>&#10007; Capacidade invalida (1-5000Ah)</div>";
            } else if (desc < 10 || desc > 100) {
                msg = "<div class='er'>&#10007; Descarga segura invalida (10-100%)</div>";
            } else {
                bateria.tensaoNominal  = vnom;
                bateria.capacidadeAh   = ah;
                bateria.descargaSegura = desc;
                calcularBateria();
                salvarConfigBateria();

                snprintf(tempBuf, sizeof(tempBuf),
                         "Bat config: %.1fV %.0fAh %.0f%%",
                         vnom, ah, desc);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Configuracao salva!</div>";
            }
        }
        else if (acao == "calibrar") {
            float pct = server.arg("pct_atual").toFloat();
            if (pct < 0 || pct > 100) {
                msg = "<div class='er'>&#10007; Percentual invalido (0-100)</div>";
            } else {
                // Calibra: energia atual = pct% da energia disponível
                bateria.energiaAtual    = bateria.energiaDisponivel * (pct / 100.0f);
                bateria.percentualAtual = pct;
                bateria.calibrada       = true;
                ultimoContagem          = millis();
                salvarConfigBateria();

                snprintf(tempBuf, sizeof(tempBuf),
                         "Bat calibrada: %.0f%% = %.0fWh",
                         pct, bateria.energiaAtual);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Bateria calibrada em " +
                      String((int)pct) + "%!</div>";
            }
        }
    }

    // Calcula valores para exibição
    calcularBateria();

    String batCls = "ok";
    if (bateria.percentualAtual < 25) batCls = "er";
    else if (bateria.percentualAtual < 50) batCls = "wn";

    String c = msg;

    // Status atual
    c += "<h3>&#128267; Status da Bateria</h3>";

    if (bateria.calibrada) {
        c += "<div class='sc " + batCls + "'>";
        c += "<div class='lbl'>Carga Atual</div>";
        c += "<div class='val'>" + String(bateria.percentualAtual, 1) + "%</div>";
        char buf[80];
        snprintf(buf, sizeof(buf), "%.0f Wh de %.0f Wh disponíveis",
                 bateria.energiaAtual, bateria.energiaDisponivel);
        c += "<div class='sub'>" + String(buf) + "</div>";
        c += "</div>";
        c += barraProgresso(bateria.percentualAtual);
    } else {
        c += "<div class='sc wn'>";
        c += "<div class='lbl'>Status</div>";
        c += "<div class='val' style='font-size:16px'>Aguardando calibracao</div>";
        c += "</div>";
    }

    // Resumo configuração
    c += "<div class='nf'>";
    char buf[120];
    snprintf(buf, sizeof(buf),
             "Tensao: %.1fV | Cap: %.0fAh | Descarga segura: %.0f%%<br>"
             "Energia total: %.0fWh | Disponivel: %.0fWh",
             bateria.tensaoNominal, bateria.capacidadeAh, bateria.descargaSegura,
             bateria.energiaTotal, bateria.energiaDisponivel);
    c += String(buf) + "</div>";

    // Form configuração
    c += "<h3>&#9881; Configurar Banco de Baterias (LiFePO4)</h3>";
    c += "<form method='POST' action='/bateria'>";
    c += "<input type='hidden' name='acao' value='config'>";
    c += "<label>Tensao Nominal (V):</label>";
    c += "<input type='number' name='vnom' step='0.1' min='6' max='60' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.1f'", bateria.tensaoNominal);
    c += String(tempBuf);
    c += " placeholder='Ex: 12.8 ou 25.6'>";
    c += "<label>Capacidade (Ah):</label>";
    c += "<input type='number' name='ah' step='1' min='1' max='5000' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", bateria.capacidadeAh);
    c += String(tempBuf);
    c += " placeholder='Ex: 100'>";
    c += "<label>Descarga Segura (%):</label>";
    c += "<input type='number' name='desc' step='1' min='10' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", bateria.descargaSegura);
    c += String(tempBuf);
    c += " placeholder='Ex: 80'>";
    c += "<button type='submit' class='btn-g'>&#128190; Salvar Configuracao</button>";
    c += "</form>";

    // Form calibração
    c += "<h3>&#127919; Calibrar Bateria</h3>";
    c += "<div class='nf'>Informe o percentual atual mostrado no display da bateria.</div>";
    c += "<form method='POST' action='/bateria'>";
    c += "<input type='hidden' name='acao' value='calibrar'>";
    c += "<label>Percentual Atual da Bateria (%):</label>";
    c += "<input type='number' name='pct_atual' step='1' min='0' max='100' required ";
    c += "placeholder='Ex: 75'>";
    c += "<button type='submit' class='btn-o'>&#127919; Calibrar Agora</button>";
    c += "</form>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Bateria", c));
}

// ============================================================
// PÁGINA: /redes
// ============================================================
void handleRedes() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // ---- Salvar configurações de corte ----
        if (acao == "config") {
            float cBaixa  = server.arg("c_baixa").toFloat();
            float cAlta   = server.arg("c_alta").toFloat();
            float cLmpExt = server.arg("c_lmpext").toFloat();
            float cLmpInt = server.arg("c_lmpint").toFloat();

            bool ok = true;
            if (cBaixa  < 0 || cBaixa  > 100) ok = false;
            if (cAlta   < 0 || cAlta   > 100) ok = false;
            if (cLmpExt < 0 || cLmpExt > 100) ok = false;
            if (cLmpInt < 0 || cLmpInt > 100) ok = false;

            if (!ok) {
                msg = "<div class='er'>&#10007; Valores devem estar entre 0 e 100%</div>";
            } else {
                redes.corteBaixa   = cBaixa;
                redes.corteAlta    = cAlta;
                redes.corteLampExt = cLmpExt;
                redes.corteLampInt = cLmpInt;
                salvarConfigRedes();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Cortes: Bx=%.0f%% Al=%.0f%% LE=%.0f%% LI=%.0f%%",
                         cBaixa, cAlta, cLmpExt, cLmpInt);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Configuracao salva!</div>";
            }
        }

        // ---- Controle manual dos relés ----
        else if (acao == "rele") {
            String qual  = server.arg("qual");
            String valor = server.arg("valor");
            bool   ligar = (valor == "1");

            if (curto.estado != CURTO_NORMAL) {
                msg = "<div class='wn'>&#9888; Controle bloqueado durante "
                "diagnostico de curto.</div>";
            } else {
                if      (qual == "rb")  { ligar ? ligarRedeBaixa()  : desligarRedeBaixa();  }
                else if (qual == "ra")  { ligar ? ligarRedeAlta()   : desligarRedeAlta();   }
                else if (qual == "le")  { ligar ? ligarLampExt()    : desligarLampExt();    }
                else if (qual == "li")  { ligar ? ligarLampInt()    : desligarLampInt();    }
                else if (qual == "inv") { ligar ? ligarInversor()   : desligarInversor();   }

                snprintf(tempBuf, sizeof(tempBuf),
                         "Rele manual: %s=%s", qual.c_str(), ligar ? "ON" : "OFF");
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Rele atualizado</div>";
            }
        }
    }

    // ============================================================
    // MONTA PÁGINA
    // ============================================================
    String c = msg;

    bool curtoAlt  = (curto.estado == CURTO_CONFIRMADO_ALTA  || curto.redeAltaCurto);
    bool curtoBx   = (curto.estado == CURTO_CONFIRMADO_BAIXA || curto.redeBaixaCurto);
    bool diagAtivo = (curto.estado == CURTO_TESTANDO_ALTA    ||
    curto.estado == CURTO_TESTANDO_BAIXA   ||
    curto.estado == CURTO_RELIGANDO);

    // ── Energia disponível ─────────────────────────────────────
    c += "<h3>&#128202; Energia Disponivel</h3>";
    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual,
                            String(bateria.percentualAtual, 1) +
                            "% | " + String(bateria.energiaAtual, 0) +
                            " Wh de " + String(bateria.energiaDisponivel, 0) + " Wh");
    } else {
        c += "<div class='wn'>&#9888; Bateria nao calibrada - "
        "acesse <a href='/bateria' style='color:#ffe0b2'>Bateria</a></div>";
    }

    // ── Proteção curto ─────────────────────────────────────────
    c += "<h3>&#9874; Protecao Contra Curto</h3>";
    c += htmlStatusCurto();

    // ── Status das cargas com controle ────────────────────────
    c += "<h3>&#128268; Controle das Cargas</h3>";

    // Aviso modo automático lamp externa
    if (sensores.correntePainel > 0.5f) {
        c += "<div class='nf' style='font-size:11px;margin-bottom:6px'>"
        "&#9728; Modo DIA ativo: Lamp. Externa desligada automaticamente.</div>";
    } else {
        c += "<div class='nf' style='font-size:11px;margin-bottom:6px'>"
        "&#127769; Modo NOITE ativo: Lamp. Externa ligada automaticamente.</div>";
    }

    bool bloqueio = (curto.estado != CURTO_NORMAL);
    if (bloqueio) {
        c += "<div class='wn' style='font-size:11px;margin-bottom:6px'>"
        "&#9888; Controle manual bloqueado durante diagnostico de curto.</div>";
    }

    // Helper: linha de controle de carga
    auto linhaRele = [&](const char* icone, const char* nome,
                         const char* chave, bool ativo,
                         bool bloqueado,
                         const String& obs = "") -> String {

                             String cor  = ativo   ? "#43a047" : "#ef5350";
                             String est  = ativo   ? "ON"      : "OFF";
                             if (bloqueado && !ativo) { cor = "#546e7a"; }

                             String h = "<div style='background:#111;border:1px solid #1e1e1e;"
                             "border-radius:6px;padding:8px 10px;margin-bottom:5px'>";

                             // Linha principal: nome + status + botão
                             h += "<div style='display:flex;justify-content:space-between;"
                             "align-items:center'>";

                             // Nome
                             h += "<span style='font-size:12px;color:#90a4ae'>";
                             h += String(icone) + " " + String(nome);
                             h += "</span>";

                             // Status + botão
                             h += "<span style='display:flex;align-items:center;gap:8px'>";
                             h += "<span style='font-size:12px;font-weight:bold;color:" + cor + "'>";
                             h += "&#9679; " + est + "</span>";

                             if (!bloqueado) {
                                 String novoVal = ativo ? "0" : "1";
                                 String btnCls  = ativo ? "btn-r" : "btn-g";
                                 String btnTxt  = ativo ? "Desligar" : "Ligar";
                                 h += "<form method='POST' action='/redes' style='margin:0'>";
                                 h += "<input type='hidden' name='acao'  value='rele'>";
                                 h += "<input type='hidden' name='qual'  value='" + String(chave) + "'>";
                                 h += "<input type='hidden' name='valor' value='" + novoVal + "'>";
                                 h += "<button type='submit' class='" + btnCls + "' "
                                 "style='width:auto;padding:5px 12px;"
                                 "font-size:11px;margin:0'>" + btnTxt + "</button>";
                                 h += "</form>";
                             } else {
                                 h += "<span style='font-size:10px;color:#546e7a;"
                                 "background:#1a1a1a;padding:4px 8px;"
                                 "border-radius:4px'>Bloqueado</span>";
                             }
                             h += "</span></div>";

                             // Observação (linha extra)
                             if (obs.length() > 0) {
                                 h += "<div style='font-size:10px;color:#546e7a;"
                                 "margin-top:5px;padding-top:5px;"
                                 "border-top:1px solid #1a1a1a'>" + obs + "</div>";
                             }

                             h += "</div>";
                             return h;
                         };

                         // Observações por carga
                         String obsInv = "";
                         if (diagAtivo || (estados.inversor && !sensores.inversoreAtivo))
                             obsInv = "&#128270; Diagnosticando sistema...";
    else if (!sensores.inversoreAtivo && estados.inversor)
        obsInv = "Aguardando sinal do inversor";

    String obsAlt = curto.redeAltaCurto ?
    "&#9888; Bloqueada por curto - <a href='/redes#curto' "
    "style='color:#ffcc80'>ver diagnostico</a>" : "";

    String obsLE = sensores.correntePainel > 0.5f ?
    "&#9728; Desligada automaticamente (dia - painel gerando)" :
    "&#127769; Controle automatico por luminosidade do painel";

    String obsBx = curto.redeBaixaCurto ?
    "&#9888; Bloqueada por curto" : "";

    c += linhaRele("&#9889;",   "Inversor",
                   "inv", estados.inversor,
                   bloqueio, obsInv);

    c += linhaRele("&#128268;", "Rede Alta (Equipamentos pesados)",
                   "ra",  estados.redesAlta,
                   bloqueio || curto.redeAltaCurto, obsAlt);

    c += linhaRele("&#128268;", "Rede Baixa (Tomadas normais)",
                   "rb",  estados.redeBaixa,
                   false, obsBx);

    c += linhaRele("&#128161;", "Lamp. Externa",
                   "le",  estados.lampExt,
                   false, obsLE);

    c += linhaRele("&#128161;", "Lamp. Interna",
                   "li",  estados.lampInt,
                   false, "");

    // Status do inversor (somente leitura)
    {
        String cor = sensores.inversoreAtivo ? "#43a047" : "#546e7a";
        String est = sensores.inversoreAtivo ? "ATIVO"   : "INATIVO";
        c += "<div style='background:#111;border:1px solid #1e1e1e;"
        "border-radius:6px;padding:8px 10px;margin-bottom:5px'>";
        c += "<div style='display:flex;justify-content:space-between;"
        "align-items:center'>";
        c += "<span style='font-size:12px;color:#90a4ae'>"
        "&#9889; Inversor (Status LED)</span>";
        c += "<span style='font-size:12px;font-weight:bold;color:" + cor + "'>"
        "&#9679; " + est + "</span>";
        c += "</div>";
        c += "<div style='font-size:10px;color:#546e7a;margin-top:5px;"
        "padding-top:5px;border-top:1px solid #1a1a1a'>"
        "Leitura do sinal LED do inversor (somente monitoramento)</div>";
        c += "</div>";
    }

    // ── Configuração de cortes ─────────────────────────────────
    c += "<h3>&#9881; Configurar Cortes de Energia</h3>";
    c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>"
    "Percentual baseado na energia disponivel da bateria.<br>"
    "Buffer de 60s antes de desligar. Religamento com margem de +2%."
    "</div>";

    c += "<form method='POST' action='/redes'>";
    c += "<input type='hidden' name='acao' value='config'>";

    // Rede Baixa
    c += "<label>&#9889; Rede Baixa + Inversor - Desligar em (%):</label>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteBaixa);
    c += "<input type='number' name='c_baixa' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 10'>";

    // Rede Alta
    c += "<label>&#9889; Rede Alta - Desligar em (%):</label>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteAlta);
    c += "<input type='number' name='c_alta' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 30'>";

    // Lamp Externa
    c += "<label>&#128161; Lamp. Externa - Desligar por energia em (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0 4px'>"
    "&#9728; Lamp. Externa tambem desliga automaticamente durante o dia "
    "(painel gerando &gt; 0.5A).</div>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteLampExt);
    c += "<input type='number' name='c_lmpext' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 50'>";

    // Lamp Interna
    c += "<label>&#128161; Lamp. Interna - Desligar em (%):</label>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteLampInt);
    c += "<input type='number' name='c_lmpint' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 0'>";

    c += "<button type='submit' class='btn-g'>&#128190; Salvar Configuracao</button>";
    c += "</form>";

    // ── Resumo dos cortes ──────────────────────────────────────
    c += "<h3>&#128202; Resumo dos Cortes</h3>";

    auto linhaResumo = [&](const char* icone, const char* nome,
                           float corte, bool ativo,
                           bool emCurto = false) -> String {
                               String cor = ativo    ? "#43a047" :
                               emCurto  ? "#ff6f00" : "#ef5350";
                               String est = ativo    ? "ON"      :
                               emCurto  ? "CURTO"   : "OFF";

                               String h = "<div style='display:flex;justify-content:space-between;"
                               "align-items:center;padding:7px 0;"
                               "border-bottom:1px solid #1a1a1a'>";
                               h += "<span style='font-size:12px;color:#90a4ae'>";
                               h += String(icone) + " " + String(nome) + "</span>";
                               h += "<span style='display:flex;align-items:center;gap:10px'>";
                               h += "<span style='font-size:11px;color:#546e7a'>Corte: ";
                               h += "<b style='color:#78909c'>" + String(corte,0) + "%</b></span>";
                               h += "<span style='font-size:12px;font-weight:bold;color:" + cor + "'>";
                               h += "&#9679; " + est + "</span>";
                               h += "</span></div>";
                               return h;
                           };

                           c += "<div style='background:#111;border:1px solid #1e1e1e;"
                           "border-radius:8px;padding:4px 12px;margin-bottom:8px'>";

                           c += linhaResumo("&#128161;","Lamp. Interna",
                                            redes.corteLampInt, estados.lampInt);

                           c += linhaResumo("&#9889;","Rede Baixa + Inversor",
                                            redes.corteBaixa,   estados.redeBaixa);

                           c += linhaResumo("&#9889;","Rede Alta",
                                            redes.corteAlta,    estados.redesAlta,
                                            curto.redeAltaCurto);

                           c += linhaResumo("&#128161;","Lamp. Externa",
                                            redes.corteLampExt, estados.lampExt);

                           // Linha especial lamp externa - modo dia/noite
                           {
                               String modoDia = sensores.correntePainel > 0.5f ? "DIA" : "NOITE";
                               String corModo = sensores.correntePainel > 0.5f ? "#fdd835" : "#546e7a";
                               c += "<div style='padding:6px 0;font-size:10px;color:#546e7a'>";
                               c += "&#9728; Lamp. Externa - Modo automatico: ";
                               c += "<span style='color:" + corModo + ";font-weight:bold'>" + modoDia + "</span>";
                               if (sensores.correntePainel > 0.5f) {
                                   char bufA[30];
                                   snprintf(bufA, sizeof(bufA), " (painel: %.1fA)", sensores.correntePainel);
                                   c += String(bufA);
                               }
                               c += "</div>";
                           }

                           c += "</div>";

                           server.send(200, "text/html; charset=utf-8",
                                       gerarPagina("RRFS - Redes", c));
}

// ============================================================
// PÁGINA: /sensores - COMPLETA COM CALIBRAÇÃO
// ============================================================
void handleSensores() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    String msg = "";

    // --------------------------------------------------------
    // PROCESSA POST DE CALIBRAÇÃO
    // --------------------------------------------------------
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // ---- CALIBRAR ZERO (offset) ----
        // Usuário garante que corrente = 0A e aperta o botão
        // O ESP32 lê o sinal atual e usa como novo offset
        if (acao == "zero_painel") {
            // Faz 200 amostras para média mais precisa no zero
            long soma = 0;
            for (int i = 0; i < 200; i++) {
                soma += analogRead(PIN_SENSOR_PAINEL);
                delay(5);
            }
            int adcMedio = soma / 200;
            int mvMedio  = adcParaMv(adcMedio);

            calibPainel.offsetMV  = mvMedio;
            calibPainel.calibrado = true;
            salvarCalibSensores();

            snprintf(tempBuf, sizeof(tempBuf),
                     "Painel ZERO calibrado: %dmV (ADC:%d)",
                     mvMedio, adcMedio);
            adicionarLog(tempBuf);

            msg = "<div class='ok'>&#10003; Sensor Painel: zero calibrado em ";
            msg += String(mvMedio) + " mV</div>";
        }

        else if (acao == "zero_casa") {
            long soma = 0;
            for (int i = 0; i < 200; i++) {
                soma += analogRead(PIN_SENSOR_CASA);
                delay(5);
            }
            int adcMedio = soma / 200;
            int mvMedio  = adcParaMv(adcMedio);

            calibCasa.offsetMV  = mvMedio;
            calibCasa.calibrado = true;
            salvarCalibSensores();

            snprintf(tempBuf, sizeof(tempBuf),
                     "Casa ZERO calibrado: %dmV (ADC:%d)",
                     mvMedio, adcMedio);
            adicionarLog(tempBuf);

            msg = "<div class='ok'>&#10003; Sensor Casa: zero calibrado em ";
            msg += String(mvMedio) + " mV</div>";
        }

        // ---- CALIBRAR SENSIBILIDADE ----
        // Usuário informa a corrente real que está passando
        // e o ESP32 calcula a sensibilidade real
        else if (acao == "sens_painel") {
            float corrReal = server.arg("corrente_real").toFloat();

            if (corrReal < 0.5f || corrReal > 50.0f) {
                msg = "<div class='er'>&#10007; Corrente invalida (0.5 a 50A)</div>";
            } else {
                // Lê sinal atual com muitas amostras
                long soma = 0;
                for (int i = 0; i < 200; i++) {
                    soma += analogRead(PIN_SENSOR_PAINEL);
                    delay(5);
                }
                int adcMedio = soma / 200;
                int mvMedio  = adcParaMv(adcMedio);

                // Calcula sensibilidade real:
                // sens = (sinal_atual - offset) / corrente_real
                float diff = (float)(mvMedio - calibPainel.offsetMV);

                if (fabsf(diff) < 10.0f) {
                    msg = "<div class='er'>&#10007; Sinal muito proximo do zero. ";
                    msg += "Use uma corrente maior para calibrar.</div>";
                } else {
                    // Verifica se sinal está invertido
                    if (diff < 0) {
                        calibPainel.invertido = true;
                        diff = -diff;
                        adicionarLog("Painel: sinal invertido detectado - corrigido");
                    } else {
                        calibPainel.invertido = false;
                    }

                    calibPainel.sensMvA   = diff / corrReal;
                    calibPainel.calibrado = true;
                    salvarCalibSensores();

                    snprintf(tempBuf, sizeof(tempBuf),
                             "Painel SENS calibrado: %.2f mV/A (sinal:%dmV corr:%.1fA)",
                             calibPainel.sensMvA, mvMedio, corrReal);
                    adicionarLog(tempBuf);

                    msg = "<div class='ok'>&#10003; Sensibilidade painel: ";
                    msg += String(calibPainel.sensMvA, 2) + " mV/A</div>";
                }
            }
        }

        else if (acao == "sens_casa") {
            float corrReal = server.arg("corrente_real").toFloat();

            if (corrReal < 0.5f || corrReal > 100.0f) {
                msg = "<div class='er'>&#10007; Corrente invalida (0.5 a 100A)</div>";
            } else {
                long soma = 0;
                for (int i = 0; i < 200; i++) {
                    soma += analogRead(PIN_SENSOR_CASA);
                    delay(5);
                }
                int adcMedio = soma / 200;
                int mvMedio  = adcParaMv(adcMedio);

                float diff = (float)(mvMedio - calibCasa.offsetMV);

                if (fabsf(diff) < 10.0f) {
                    msg = "<div class='er'>&#10007; Sinal muito proximo do zero. ";
                    msg += "Ligue uma carga conhecida para calibrar.</div>";
                } else {
                    if (diff < 0) {
                        calibCasa.invertido = true;
                        diff = -diff;
                        adicionarLog("Casa: sinal invertido detectado - corrigido");
                    } else {
                        calibCasa.invertido = false;
                    }

                    calibCasa.sensMvA   = diff / corrReal;
                    calibCasa.calibrado = true;
                    salvarCalibSensores();

                    snprintf(tempBuf, sizeof(tempBuf),
                             "Casa SENS calibrado: %.2f mV/A (sinal:%dmV corr:%.1fA)",
                             calibCasa.sensMvA, mvMedio, corrReal);
                    adicionarLog(tempBuf);

                    msg = "<div class='ok'>&#10003; Sensibilidade casa: ";
                    msg += String(calibCasa.sensMvA, 2) + " mV/A</div>";
                }
            }
        }

        // ---- RESETAR CALIBRAÇÃO ----
        else if (acao == "reset_calib") {
            String qual = server.arg("qual");

            if (qual == "painel") {
                calibPainel.offsetMV  = OFFSET_LIDO_MV;
                calibPainel.sensMvA   = PAINEL_SENS_MV_A * 0.66f;
                calibPainel.invertido = false;
                calibPainel.calibrado = false;
                salvarCalibSensores();
                adicionarLog("Calib painel resetada para padrao");
                msg = "<div class='wn'>&#9888; Calibracao do painel resetada</div>";
            } else if (qual == "casa") {
                calibCasa.offsetMV  = OFFSET_LIDO_MV;
                calibCasa.sensMvA   = CASA_SENS_MV_A * 0.66f;
                calibCasa.invertido = false;
                calibCasa.calibrado = false;
                salvarCalibSensores();
                adicionarLog("Calib casa resetada para padrao");
                msg = "<div class='wn'>&#9888; Calibracao da casa resetada</div>";
            }
        }
    }

    // --------------------------------------------------------
    // Leitura atual para exibição
    // --------------------------------------------------------
    // Lê ADC bruto sem aplicar calibração (para diagnóstico)
    int adcPainel = lerADC(PIN_SENSOR_PAINEL);
    int adcCasa   = lerADC(PIN_SENSOR_CASA);
    int mvPainel  = adcParaMv(adcPainel);
    int mvCasa    = adcParaMv(adcCasa);

    // Correntes com calibração atual
    float corrPainel = sensores.correntePainel;
    float corrCasa   = sensores.correnteCasa;

    // --------------------------------------------------------
    // MONTA PÁGINA
    // --------------------------------------------------------
    String c = msg;

    // ========================================================
    // SENSOR PAINEL SOLAR
    // ========================================================
    c += "<h3>&#9728; Sensor Painel Solar (ACS758 50A)</h3>";

    // Card status atual
    String clsPainel = calibPainel.calibrado ? "ok" : "wn";
    c += "<div class='sc " + clsPainel + "'>";
    c += "<div class='lbl'>Leitura Atual</div>";
    c += "<div class='val'>" + fmtCorrente(corrPainel) + "</div>";

    char buf[120];
    snprintf(buf, sizeof(buf),
             "Sinal: %d mV | ADC: %d | %s",
             mvPainel, adcPainel,
             fmtPotencia(sensores.potenciaPainel).c_str());
    c += "<div class='sub'>" + String(buf) + "</div>";
    c += "</div>";

    // Info calibração atual
    c += "<div class='nf' style='font-size:11px'>";
    snprintf(buf, sizeof(buf),
             "Offset: <b>%d mV</b> | Sens: <b>%.2f mV/A</b> | "
             "Invertido: <b>%s</b> | Calibrado: <b>%s</b>",
             calibPainel.offsetMV,
             calibPainel.sensMvA,
             calibPainel.invertido ? "SIM" : "NAO",
             calibPainel.calibrado ? "SIM" : "NAO");
    c += String(buf);
    c += "</div>";

    // Alerta sinal invertido
    if (corrPainel < -5.0f) {
        // Corrente muito negativa = suspeito de inversão
        c += "<div class='er' style='font-size:11px'>"
        "&#9888; Corrente negativa muito alta (&gt;5A)! "
        "Possivel inversao de cabos do sensor.</div>";
    } else if (corrPainel < -0.3f && corrPainel >= -5.0f) {
        // Corrente negativa pequena = normal (consumo controlador/ESP32)
        c += "<div class='ok' style='font-size:11px'>"
        "&#10003; Corrente negativa pequena = normal. "
        "E o consumo proprio do controlador/ESP32 ("
        + String(fabsf(corrPainel), 2) + "A).</div>";
    }

    // --- PASSO 1: Calibrar zero ---
    c += "<div class='sc' style='margin-top:10px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>";
    c += "&#9312; PASSO 1 - Calibrar Zero (Offset)</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>";
    c += "Garanta que <b>nenhuma corrente</b> passa pelo sensor painel.<br>";
    c += "Desconecte os cabos do painel solar ou espere a noite.<br>";
    c += "Sinal atual: <b>" + String(mvPainel) + " mV</b> | ";
    c += "Offset atual: <b>" + String(calibPainel.offsetMV) + " mV</b>";
    c += "</div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='zero_painel'>";
    c += "<button type='submit' class='btn-o' "
    "onclick='return confirm(\"Confirma calibracao do ZERO do painel?\\n"
    "Certifique-se que nao ha corrente passando pelo sensor!\")'>";
    c += "&#127919; Calibrar Zero do Painel (0A agora)</button>";
    c += "</form>";
    c += "</div>";

    // --- PASSO 2: Calibrar sensibilidade ---
    c += "<div class='sc' style='margin-top:8px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>";
    c += "&#9313; PASSO 2 - Calibrar Sensibilidade</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>";
    c += "Com uma corrente <b>conhecida e estavel</b> passando pelo sensor,<br>";
    c += "informe o valor real (use multimetro ou dados do controlador).<br>";
    c += "Sinal atual: <b>" + String(mvPainel) + " mV</b> | ";
    c += "Corrente calculada: <b>" + fmtCorrente(corrPainel) + "</b>";
    c += "</div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='sens_painel'>";
    c += "<label>Corrente Real Atual (A) - informe o valor do controlador:</label>";
    c += "<input type='number' name='corrente_real' step='0.1' min='0.5' max='50' "
    "required placeholder='Ex: 5.2'>";
    c += "<button type='submit' class='btn-g'>";
    c += "&#127919; Calibrar Sensibilidade do Painel</button>";
    c += "</form>";
    c += "</div>";

    // Reset calibração painel
    c += "<form method='POST' action='/sensores' style='margin-top:6px'>";
    c += "<input type='hidden' name='acao' value='reset_calib'>";
    c += "<input type='hidden' name='qual'  value='painel'>";
    c += "<button type='submit' class='btn-r btn-sm' "
    "onclick='return confirm(\"Resetar calibracao do painel para valores padrao?\")'>";
    c += "&#8635; Resetar Calib Painel para Padrao</button>";
    c += "</form>";

    // ========================================================
    // SENSOR CASA
    // ========================================================
    c += "<h3>&#127968; Sensor Casa (ACS758 100A)</h3>";

    String clsCasa = calibCasa.calibrado ? "ok" : "wn";
    c += "<div class='sc " + clsCasa + "'>";
    c += "<div class='lbl'>Leitura Atual</div>";
    c += "<div class='val'>" + fmtCorrente(corrCasa) + "</div>";
    snprintf(buf, sizeof(buf),
             "Sinal: %d mV | ADC: %d | %s",
             mvCasa, adcCasa,
             fmtPotencia(sensores.potenciaCasa).c_str());
    c += "<div class='sub'>" + String(buf) + "</div>";
    c += "</div>";

    c += "<div class='nf' style='font-size:11px'>";
    snprintf(buf, sizeof(buf),
             "Offset: <b>%d mV</b> | Sens: <b>%.2f mV/A</b> | "
             "Invertido: <b>%s</b> | Calibrado: <b>%s</b>",
             calibCasa.offsetMV,
             calibCasa.sensMvA,
             calibCasa.invertido ? "SIM" : "NAO",
             calibCasa.calibrado ? "SIM" : "NAO");
    c += String(buf);
    c += "</div>";

    // --- PASSO 1: Zero casa ---
    c += "<div class='sc' style='margin-top:10px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>";
    c += "&#9312; PASSO 1 - Calibrar Zero (Offset)</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>";
    c += "Desligue <b>todos os equipamentos</b> da casa (toda carga = 0A).<br>";
    c += "Deixe apenas o ESP32 e roteador ligados (consumo proprio ignorado).<br>";
    c += "Sinal atual: <b>" + String(mvCasa) + " mV</b> | ";
    c += "Offset atual: <b>" + String(calibCasa.offsetMV) + " mV</b>";
    c += "</div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='zero_casa'>";
    c += "<button type='submit' class='btn-o' "
    "onclick='return confirm(\"Confirma calibracao do ZERO da casa?\\n"
    "Certifique-se que NENHUMA carga esta ligada!\")'>";
    c += "&#127919; Calibrar Zero da Casa (0A agora)</button>";
    c += "</form>";
    c += "</div>";

    // --- PASSO 2: Sensibilidade casa ---
    c += "<div class='sc' style='margin-top:8px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>";
    c += "&#9313; PASSO 2 - Calibrar Sensibilidade</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>";
    c += "Ligue uma carga <b>conhecida e estavel</b> (ex: chuveiro, forno).<br>";
    c += "Informe a corrente real medida (use multimetro na entrada).<br>";
    c += "Sinal atual: <b>" + String(mvCasa) + " mV</b> | ";
    c += "Corrente calculada: <b>" + fmtCorrente(corrCasa) + "</b>";
    c += "</div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='sens_casa'>";
    c += "<label>Corrente Real Atual (A) - informe o valor medido:</label>";
    c += "<input type='number' name='corrente_real' step='0.1' min='0.5' max='100' "
    "required placeholder='Ex: 10.5'>";
    c += "<button type='submit' class='btn-g'>";
    c += "&#127919; Calibrar Sensibilidade da Casa</button>";
    c += "</form>";
    c += "</div>";

    // Reset calibração casa
    c += "<form method='POST' action='/sensores' style='margin-top:6px'>";
    c += "<input type='hidden' name='acao' value='reset_calib'>";
    c += "<input type='hidden' name='qual'  value='casa'>";
    c += "<button type='submit' class='btn-r btn-sm' "
    "onclick='return confirm(\"Resetar calibracao da casa para valores padrao?\")'>";
    c += "&#8635; Resetar Calib Casa para Padrao</button>";
    c += "</form>";

    // ========================================================
    // BATERIA CALCULADA
    // ========================================================
    c += "<h3>&#128267; Bateria (Calculado)</h3>";
    String dirBat  = sensores.correnteBateria >= 0 ? "Carregando" : "Descarregando";
    String corBat  = sensores.correnteBateria >= 0 ? "ok" : "er";
    c += "<div class='sc " + corBat + "'>";
    c += "<div class='lbl'>Painel - Casa = Bateria</div>";
    c += "<div class='val'>" + fmtCorrente(sensores.correnteBateria);
    c += " " + dirBat + "</div>";
    c += "<div class='sub'>" + fmtPotencia(sensores.potenciaBateria) + "</div>";
    c += "</div>";

    // ========================================================
    // SENSOR INVERSOR
    // ========================================================
    c += "<h3>&#9889; Sinal Status Inversor</h3>";
    int adcInv = lerADC(PIN_STATUS_INV_ALTA);
    int mvInv  = adcParaMv(adcInv);
    sensores.sinalInversor_mV = mvInv;

    String invStatus = sensores.inversoreAtivo ? "LIGADO"   : "DESLIGADO";
    String invCor    = sensores.inversoreAtivo ? "#43a047"  : "#546e7a";
    c += "<div class='sc'>";
    c += "<div class='lbl'>Status (limiar: " + String(INV_THRESHOLD_MV) + " mV)</div>";
    c += "<div class='val' style='color:" + invCor + "'>" + invStatus + "</div>";
    snprintf(buf, sizeof(buf),
             "Sinal: %d mV | ADC: %d | Buffer: %dms",
             mvInv, adcInv, INV_BUFFER_MS);
    c += "<div class='sub'>" + String(buf) + "</div>";
    c += "</div>";

    // ========================================================
    // SINAL BKP (futuro)
    // ========================================================
    c += "<h3>&#128267; Sinal BKP Bateria (futuro)</h3>";
    int adcBkp = lerADC(PIN_BKP_BATERIA);
    int mvBkp  = adcParaMv(adcBkp);
    c += "<div class='sc'>";
    c += "<div class='lbl'>Monitoramento futuro</div>";
    c += "<div class='val' style='color:#546e7a'>Ignorado</div>";
    snprintf(buf, sizeof(buf), "Sinal: %d mV | ADC: %d", mvBkp, adcBkp);
    c += "<div class='sub'>" + String(buf) + "</div>";
    c += "</div>";

    // ========================================================
    // DIAGNÓSTICO GERAL
    // ========================================================
    c += "<h3>&#128270; Diagnostico</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "<b>Valores esperados com sensores em 5V + divisor 5V→3.3V:</b><br>";
    snprintf(buf, sizeof(buf),
             "Offset esperado: ~1650 mV (VCC/2 * 0.66)<br>"
             "Painel sens. esperada: %.1f mV/A (40 * 0.66)<br>"
             "Casa sens. esperada: %.1f mV/A (20 * 0.66)<br>",
             PAINEL_SENS_MV_A * 0.66f,
             CASA_SENS_MV_A   * 0.66f);
    c += String(buf);
    c += "<br><b>Valores atuais calibrados:</b><br>";
    snprintf(buf, sizeof(buf),
             "Painel: off=%dmV sens=%.2fmV/A<br>"
             "Casa:   off=%dmV sens=%.2fmV/A<br>"
             "Amostras ADC por leitura: %d",
             calibPainel.offsetMV, calibPainel.sensMvA,
             calibCasa.offsetMV,   calibCasa.sensMvA,
             ADC_SAMPLES);
    c += String(buf);
    c += "</div>";

    // Botão atualizar
    c += "<button onclick='location.reload()' class='btn-s' style='margin-top:10px'>";
    c += "&#128259; Atualizar Leituras</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Sensores", c));
}

void handleOTAResult() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    if (Update.hasError()) {
        // Falha no update
        String c = "<div class='er'>";
        c += "<b>&#10007; Falha no Update!</b><br>";
        c += "Verifique se o arquivo .bin e valido.<br>";
        c += "O sistema continuara com o firmware anterior.";
        c += "</div>";
        c += "<button onclick='location.href=\"/config\"' class='btn-s'>";
        c += "&#8592; Voltar</button>";
        server.send(200, "text/html; charset=utf-8",
                    gerarPagina("RRFS - OTA Erro", c));
    } else {
        // Sucesso - reinicia em 3s
        String c = "<div class='ok'>";
        c += "<b>&#10003; Update realizado com sucesso!</b><br>";
        c += "Reiniciando em 3 segundos...<br>";
        c += "Aguarde e recarregue a pagina.";
        c += "</div>";
        c += "<div class='nf'>";
        c += "Apos reiniciar, acesse novamente pelo mesmo IP.<br>";
        c += "Se configurou via WiFi, o IP permanece o mesmo.";
        c += "</div>";

        // Auto reload após 8s
        c += "<script>setTimeout(function(){location.href='/';},8000);</script>";

        server.send(200, "text/html; charset=utf-8",
                    gerarPagina("RRFS - OTA OK", c, false));

        adicionarLog("OTA: reiniciando sistema...");
        delay(3000);
        ESP.restart();
    }
}

// ============================================================
// PÁGINA: /config (Configurações gerais)
// ============================================================
void handleConfig() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // ---- RESET PARCIAL ----
        if (acao == "reset_parcial") {
            // Salva configurações de rede antes de limpar
            prefs.begin("rrfs", true);
            String wssid = prefs.getString("wifi_ssid", "");
            String wpass = prefs.getString("wifi_pass", "");
            String wip   = prefs.getString("wifi_ip",   "");
            String wgw   = prefs.getString("wifi_gw",   "");
            String wmask = prefs.getString("wifi_mask", "255.255.255.0");
            String wdns  = prefs.getString("wifi_dns",  "8.8.8.8");
            bool   wap   = prefs.getBool  ("usar_ap",   true);
            prefs.end();

            // Limpa tudo
            prefs.begin("rrfs", false);
            prefs.clear();

            // Restaura apenas rede
            prefs.putBool  ("usar_ap",   wap);
            prefs.putString("wifi_ssid", wssid);
            prefs.putString("wifi_pass", wpass);
            prefs.putString("wifi_ip",   wip);
            prefs.putString("wifi_gw",   wgw);
            prefs.putString("wifi_mask", wmask);
            prefs.putString("wifi_dns",  wdns);
            prefs.putBool  ("wifi_ok",   true);
            prefs.end();

            adicionarLog("RESET PARCIAL: config rede mantida");

            String c = "<div class='ok'>&#10003; Reset parcial realizado!</div>";
            c += "<div class='nf'>Configuracoes de rede mantidas.<br>";
            c += "Reiniciando em 3s...</div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - Reset", c, false));
            delay(3000);
            ESP.restart();
            return;
        }

        // ---- ALTERAR SENHA ----
        if (acao == "senha") {
            String pAtual = server.arg("p_atual");
            String pNova  = server.arg("p_nova");
            String pConf  = server.arg("p_conf");
            String user   = "";

            prefs.begin("rrfs", true);
            user = prefs.getString("user", "");
            prefs.end();

            if (!verificarLogin(user, pAtual)) {
                msg = "<div class='er'>&#10007; Senha atual incorreta</div>";
            } else if (pNova.length() < 4) {
                msg = "<div class='er'>&#10007; Nova senha: minimo 4 caracteres</div>";
            } else if (pNova != pConf) {
                msg = "<div class='er'>&#10007; Senhas nao conferem</div>";
            } else {
                salvarCredenciais(user, pNova);
                adicionarLog("Senha alterada com sucesso");
                msg = "<div class='ok'>&#10003; Senha alterada!</div>";
            }
        }
    }

    // --------------------------------------------------------
    // MONTA PÁGINA
    // --------------------------------------------------------
    String ip   = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String modo = modoAP ? "Ponto de Acesso (AP)" : "WiFi Cliente";

    String c = msg;

    // --- Info do sistema ---
    c += "<h3>&#128202; Informacoes do Sistema</h3>";
    c += "<div class='nf'>";
    c += "&#128246; Modo: <b>" + modo + "</b><br>";
    c += "&#127760; IP: <b>" + ip + "</b><br>";

    if (!modoAP && WiFi.status() == WL_CONNECTED) {
        c += "&#128246; SSID: <b>" + WiFi.SSID() + "</b><br>";
        snprintf(tempBuf, sizeof(tempBuf),
                 "&#128246; RSSI: <b>%ddBm</b> | Canal: <b>%d</b><br>",
                 WiFi.RSSI(), WiFi.channel());
        c += String(tempBuf);
    }

    // Hora atual
    struct tm ti;
    if (getLocalTime(&ti)) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "&#128336; Hora: <b>%02d:%02d:%02d %02d/%02d/%04d</b><br>",
                 ti.tm_hour, ti.tm_min,   ti.tm_sec,
                 ti.tm_mday, ti.tm_mon+1, ti.tm_year + 1900);
        c += String(tempBuf);
    } else {
        c += "&#128336; Hora: <b>NTP nao sincronizado</b><br>";
    }

    snprintf(tempBuf, sizeof(tempBuf),
             "&#128190; RAM livre: <b>%.1f KB</b><br>"
             "&#9201; Uptime: <b>%lu segundos</b>",
             (unsigned)esp_get_free_heap_size() / 1024.0f,
             millis() / 1000);
    c += String(tempBuf);
    c += "</div>";

    // --- Botões rápidos ---
    c += "<div class='row'>";
    c += "<button onclick='location.href=\"/setup-wifi\"' class='btn-o'>";
    c += "&#128246; Reconfigurar Rede</button>";
    c += "<button onclick='if(confirm(\"Reiniciar o sistema agora?\"))location.href=\"/reiniciar\"' class='btn-s'>";
    c += "&#128260; Reiniciar</button>";
    c += "</div>";

    // --- NTP manual ---
    c += "<h3>&#128336; Sincronizar Hora (NTP)</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "Fuso horario: UTC-3 (Brasilia). ";
    c += "Sincronizacao automatica diaria ativa.";
    c += "</div>";
    c += "<button onclick='location.href=\"/ntp-sync\"' class='btn-s'>";
    c += "&#128336; Sincronizar Agora</button>";

    // --- OTA Update ---
    c += "<h3>&#128640; Atualizar Firmware (OTA)</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "<b>Como atualizar:</b><br>";
    c += "1. Abra o Arduino IDE<br>";
    c += "2. Menu: <b>Sketch &rarr; Exportar Binario Compilado</b><br>";
    c += "3. Localize o arquivo <b>.bin</b> gerado<br>";
    c += "4. Selecione o arquivo abaixo e clique em Enviar<br>";
    c += "<b style='color:#ef9a9a'>&#9888; Nao feche esta pagina durante o upload!</b>";
    c += "</div>";

    // Barra de progresso OTA
    c += "<div id='ota-prog' style='display:none'>";
    c += "<div class='nf' style='text-align:center'>";
    c += "<b id='ota-status'>Enviando...</b><br>";
    c += "<div class='barra-bg' style='margin-top:8px'>";
    c += "<div class='barra' id='ota-barra' style='width:0%;background:#0277bd'>";
    c += "0%</div></div>";
    c += "<span id='ota-bytes' style='font-size:10px;color:#546e7a'></span>";
    c += "</div></div>";

    // Form upload
    c += "<form id='ota-form' method='POST' action='/ota-upload' ";
    c += "enctype='multipart/form-data'>";
    c += "<label>Arquivo Firmware (.bin):</label>";
    c += "<input type='file' id='ota-file' name='firmware' accept='.bin' required ";
    c += "style='margin-bottom:10px'>";
    c += "<div id='ota-info' style='display:none;font-size:11px;color:#546e7a;margin-bottom:8px'>";
    c += "Arquivo: <span id='ota-fname'></span> | ";
    c += "Tamanho: <span id='ota-fsize'></span>";
    c += "</div>";
    c += "<button type='submit' id='ota-btn' class='btn-g' disabled>";
    c += "&#128640; Enviar Firmware</button>";
    c += "</form>";

    // Alterar senha
    c += "<h3>&#128274; Alterar Senha</h3>";
    c += "<form method='POST' action='/config'>";
    c += "<input type='hidden' name='acao' value='senha'>";
    c += "<label>Senha Atual:</label>";
    c += "<input type='password' name='p_atual' required placeholder='Senha atual'>";
    c += "<label>Nova Senha (min. 4 caracteres):</label>";
    c += "<input type='password' name='p_nova'  required placeholder='Nova senha'>";
    c += "<label>Confirmar Nova Senha:</label>";
    c += "<input type='password' name='p_conf'  required placeholder='Confirmar nova senha'>";
    c += "<button type='submit'>&#128190; Alterar Senha</button>";
    c += "</form>";

    // Reset parcial
    c += "<h3>&#9888; Reset Parcial</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "Limpa usuario, bateria e redes. <b>Mantem configuracao WiFi.</b>";
    c += "</div>";
    c += "<form method='POST' action='/config' ";
    c += "onsubmit='return confirm(\"Confirma reset parcial?\\nConfig de rede sera mantida.\")'>";
    c += "<input type='hidden' name='acao' value='reset_parcial'>";
    c += "<button type='submit' class='btn-r'>";
    c += "&#9888; Reset Parcial (Mantem Rede)</button>";
    c += "</form>";

    // Logout
    c += "<button onclick='location.href=\"/logout\"' ";
    c += "class='btn-s' style='margin-top:8px'>";
    c += "&#128275; Sair do Sistema</button>";

    // JavaScript OTA com barra de progresso
    c += F("<script>"
        // Mostra info do arquivo selecionado
        "document.getElementById('ota-file').addEventListener('change',function(e){"
        "var f=e.target.files[0];"
        "if(!f)return;"
        "var kb=(f.size/1024).toFixed(1);"
        "document.getElementById('ota-fname').textContent=f.name;"
        "document.getElementById('ota-fsize').textContent=kb+' KB';"
        "document.getElementById('ota-info').style.display='block';"
        // Habilita botão apenas se for .bin
        "var ok=f.name.toLowerCase().endsWith('.bin');"
        "document.getElementById('ota-btn').disabled=!ok;"
        "if(!ok)alert('Selecione um arquivo .bin valido!');"
        "});"

        // Submit com XMLHttpRequest para mostrar progresso
        "document.getElementById('ota-form').addEventListener('submit',function(e){"
        "e.preventDefault();"
        "var f=document.getElementById('ota-file').files[0];"
        "if(!f){alert('Selecione um arquivo .bin');return;}"
        "if(!f.name.toLowerCase().endsWith('.bin')){"
        "alert('Arquivo invalido! Use apenas .bin');return;}"
        "if(!confirm('Confirma o update do firmware?\\n\\nArquivo: '+f.name+'\\nTamanho: '+(f.size/1024).toFixed(1)+' KB'))return;"

        // Desabilita form e mostra progresso
        "document.getElementById('ota-btn').disabled=true;"
        "document.getElementById('ota-btn').textContent='Enviando...';"
        "document.getElementById('ota-prog').style.display='block';"
        "document.getElementById('ota-form').style.opacity='0.5';"

        "var fd=new FormData();"
        "fd.append('firmware',f);"
        "var xhr=new XMLHttpRequest();"

        // Progresso do upload
        "xhr.upload.addEventListener('progress',function(e){"
        "if(e.lengthComputable){"
        "var pct=Math.round(e.loaded/e.total*100);"
        "var kb=(e.loaded/1024).toFixed(0);"
        "var kbt=(e.total/1024).toFixed(0);"
        "document.getElementById('ota-barra').style.width=pct+'%';"
        "document.getElementById('ota-barra').textContent=pct+'%';"
        "document.getElementById('ota-bytes').textContent=kb+' / '+kbt+' KB';"
        "if(pct>=100){"
        "document.getElementById('ota-status').textContent='Gravando firmware...';"
        "document.getElementById('ota-barra').style.background='#2e7d32';}}"
        "});"

        // Resposta do servidor
        "xhr.onreadystatechange=function(){"
        "if(xhr.readyState==4){"
        "if(xhr.status==200){"
        "document.getElementById('ota-status').textContent='Concluido! Redirecionando...';"
        "setTimeout(function(){location.href='/';},5000);"
        "}else{"
        "document.getElementById('ota-status').textContent='ERRO no upload!';"
        "document.getElementById('ota-barra').style.background='#c62828';"
        "document.getElementById('ota-btn').disabled=false;"
        "document.getElementById('ota-btn').textContent='Tentar Novamente';"
        "document.getElementById('ota-form').style.opacity='1';"
        "}}}"
        ";xhr.open('POST','/ota-upload',true);"
        "xhr.send(fd);"
        "});"
        "</script>");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Configuracoes", c));
}

// ============================================================
// ENDPOINT: /ntp-sync (sincronização manual)
// ============================================================
void handleNTPSync() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }
    adicionarLog("NTP: sincronizacao manual solicitada");
    sincronizarNTP();
    server.sendHeader("Location", "/config");
    server.send(302);
}

// ============================================================
// ENDPOINT: /reiniciar
// ============================================================
void handleReiniciar() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }
    adicionarLog("Reinicio solicitado via web");
    String c = "<div class='ok'>&#10003; Reiniciando em 3 segundos...</div>";
    c += "<div class='nf'>Aguarde e recarregue a pagina.</div>";
    c += "<script>setTimeout(function(){location.href='/';},8000);</script>";
    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Reiniciando", c, false));
    delay(3000);
    ESP.restart();
}

// ============================================================
// ENDPOINT: /logs
// ============================================================
void handleLogs() {
    String json = "[";
    for (int i = 0; i < totalLogs; i++) {
        int idx = ((int)logIndex - 1 - i + MAX_LOGS) % MAX_LOGS;
        if (i > 0) json += ",";
        json += "\"";
        for (int ch = 0; logs[idx][ch]; ch++) {
            char cc = logs[idx][ch];
            if (cc == '"')  json += "\\\"";
            else if (cc == '\\') json += "\\\\";
            else json += cc;
        }
        json += "\"";
    }
    json += "]";
    server.send(200, "application/json", json);
}

// ============================================================
// PÁGINAS DE SETUP E LOGIN (base code mantido)
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
        if (user.length() < 3)  msgHtml = "<div class='er'>&#10007; Minimo 3 caracteres no usuario</div>";
        else if (pass.length() < 4) msgHtml = "<div class='er'>&#10007; Minimo 4 caracteres na senha</div>";
        else if (pass != conf)  msgHtml = "<div class='er'>&#10007; Senhas nao conferem</div>";
        else {
            salvarCredenciais(user, pass);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location", "/setup-wifi"); server.send(302); return;
        }
    }
    String c = "<div class='nf'>&#8505; Primeira configuracao. Crie suas credenciais.</div>";
    c += "<h3>&#128274; Criar Conta</h3>" + msgHtml;
    c += "<form method='POST' action='/setup-login'>";
    c += "<label>Usuario (min. 3 caracteres):</label>";
    c += "<input type='text' name='user' maxlength='32' required autocomplete='off' placeholder='Usuario'>";
    c += "<label>Senha (min. 4 caracteres):</label>";
    c += "<input type='password' name='pass' maxlength='32' required placeholder='Senha'>";
    c += "<label>Confirmar Senha:</label>";
    c += "<input type='password' name='conf' maxlength='32' required placeholder='Confirmar senha'>";
    c += "<button type='submit'>Criar e Continuar &#8594;</button></form>";
    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Primeiro Acesso", c, false));
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
        if (usarAP) {
            salvarWifi("","","","","","",true);
            adicionarLog("Modo AP salvo");
            String cont = "<div class='ok'>&#10003; Salvo! Reiniciando em 5s...</div>";
            cont += "<div class='nf'>SSID: <b>RRFS Energia</b><br>IP: <b>10.0.0.1</b></div>";
            server.send(200,"text/html; charset=utf-8",gerarPagina("RRFS - Salvo",cont,false));
            delay(5000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"), wpass=server.arg("wpass");
        String ip=server.arg("ip"),     gw=server.arg("gw");
        String mask=server.arg("mask"), dns=server.arg("dns");
        if (ssid.length()==0)  msgHtml="<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7)  msgHtml="<div class='er'>Informe o IP fixo</div>";
        else if (gw.length()<7)  msgHtml="<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wpass,ip,gw,mask,dns,false);
            adicionarLog(("WiFi salvo: "+ssid).c_str());
            String cont = "<div class='ok'>&#10003; Salvo! Acesse: http://"+ip+"</div>";
            cont += "<div class='nf'>Reiniciando em 5s...</div>";
            server.send(200,"text/html; charset=utf-8",gerarPagina("RRFS - Salvo",cont,false));
            delay(5000); ESP.restart(); return;
        }
    }

    adicionarLog("Escaneando WiFi...");
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, false, false, 600);
    snprintf(tempBuf, sizeof(tempBuf), "Redes: %d", n);
    adicionarLog(tempBuf);

    String c = "<div class='nf'>&#8505; Configure a rede. IP fixo e obrigatorio.</div>";
    c += "<h3>&#128246; Configurar Rede</h3>" + msgHtml;
    c += "<form method='POST' action='/setup-wifi'>";
    c += "<label>Modo:</label><select name='modo' id='modo' onchange='toggleModo()'>";
    c += "<option value='wifi'>Conectar ao WiFi</option>";
    c += "<option value='ap'>Manter Ponto de Acesso</option></select>";
    c += "<div id='dw'>";
    c += "<label>Rede WiFi:</label><select name='ssid'>";
    if (n == 0) c += "<option value=''>Nenhuma rede</option>";
    else {
        for (int i = 0; i < n; i++) {
            String q = WiFi.RSSI(i) > -60 ? "Bom" : WiFi.RSSI(i) > -75 ? "Regular" : "Fraco";
            c += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i);
            snprintf(tempBuf, sizeof(tempBuf), " [%s %ddBm]", q.c_str(), WiFi.RSSI(i));
            c += String(tempBuf) + "</option>";
        }
    }
    WiFi.scanDelete();
    c += "</select>";
    c += "<button type='button' class='btn-o btn-sm' onclick='location.reload()' style='margin-top:8px'>";
    c += "&#128259; Atualizar Lista</button>";
    c += "<label>Senha WiFi:</label><input type='password' name='wpass' placeholder='Vazio se aberta'>";
    c += "<label>IP Fixo do ESP32:</label><input type='text' name='ip' required placeholder='192.168.1.100'>";
    c += "<label>Gateway:</label><input type='text' name='gw' required placeholder='192.168.1.1'>";
    c += "<label>Mascara:</label><input type='text' name='mask' required value='255.255.255.0'>";
    c += "<label>DNS:</label><input type='text' name='dns' required value='8.8.8.8'>";
    c += "</div>";
    c += "<button type='submit' class='btn-g'>&#128190; Salvar e Reiniciar</button></form>";
    if (logado) c += "<button onclick='location.href=\"/\"' class='btn-s' style='margin-top:8px'>&#8592; Voltar</button>";
    c += "<script>function toggleModo(){";
    c += "document.getElementById('dw').style.display=";
    c += "document.getElementById('modo').value=='wifi'?'block':'none';}</script>";
    server.send(200,"text/html; charset=utf-8",gerarPagina("RRFS - Rede",c,false));
}

void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location","/setup-login"); server.send(302); return;
    }
    if (logado) {
        server.sendHeader("Location","/"); server.send(302); return;
    }
    String msgHtml = "";
    if (server.method() == HTTP_POST) {
        String user = server.arg("user"), pass = server.arg("pass");
        if (verificarLogin(user, pass)) {
            logado = true;
            snprintf(tempBuf, sizeof(tempBuf), "Login OK: %s", user.c_str());
            adicionarLog(tempBuf);
            logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        } else {
            snprintf(tempBuf, sizeof(tempBuf), "Login FALHOU: %s", user.c_str());
            adicionarLog(tempBuf);
            msgHtml = "<div class='er'>&#10007; Usuario ou senha incorretos</div>";
        }
    }
    String c = "<div class='nf'>&#128274; Acesso seguro ao sistema</div>";
    c += "<h3>Entrar</h3>" + msgHtml;
    c += "<form method='POST' action='/login'>";
    c += "<label>Usuario:</label><input type='text' name='user' required autocomplete='off' placeholder='Usuario'>";
    c += "<label>Senha:</label><input type='password' name='pass' required placeholder='Senha'>";
    c += "<button type='submit'>&#128275; Entrar</button></form>";
    server.send(200,"text/html; charset=utf-8",gerarPagina("RRFS - Login",c,false));
}

void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado = false; }
    server.sendHeader("Location","/login"); server.send(302);
}

void handleNotFound() {
    if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else server.send(404,"text/plain","404");
}


// ============================================================
// STATUS DO CURTO PARA A PÁGINA WEB /redes
// Retorna HTML com alertas e botões de reset
// ============================================================
String htmlStatusCurto() {
    String h = "";

    if (curto.estado == CURTO_NORMAL && !curto.redeAltaCurto && !curto.redeBaixaCurto) {
        // Tudo normal - mostra apenas indicador verde
        h += "<div class='ok' style='font-size:12px'>";
        h += "&#10003; Sistema sem curto detectado</div>";
        return h;
    }

    // Diagnóstico em andamento
    if (curto.estado == CURTO_TESTANDO_ALTA || curto.estado == CURTO_TESTANDO_BAIXA) {
        h += "<div class='wn'>";
        h += "&#9874; Diagnosticando curto... Aguarde.</div>";
        return h;
    }

    // Curto confirmado na rede alta
    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        h += "<div class='er'>";
        h += "<b>&#9888; CURTO DETECTADO NA REDE ALTA!</b><br>";
        h += "Equipamentos pesados desconectados.<br>";
        h += "Rede alta desligada por protecao.<br>";
        h += "Rede baixa e inversor operando normalmente.<br><br>";
        h += "<b>1.</b> Verifique e resolva o curto na rede de alta potencia.<br>";
        h += "<b>2.</b> Pressione o botao abaixo apos corrigir.";
        h += "</div>";
        h += "<form method='POST' action='/reset-curto'>";
        h += "<input type='hidden' name='rede' value='alta'>";
        h += "<button type='submit' class='btn-g' style='margin-top:8px'>";
        h += "&#10003; Curto da Rede Alta Resolvido - Testar e Religar</button>";
        h += "</form>";
        return h;
    }

    // Curto confirmado na rede baixa
    if (curto.estado == CURTO_CONFIRMADO_BAIXA) {
        h += "<div class='er'>";
        h += "<b>&#9888; CURTO DETECTADO NA REDE BAIXA!</b><br>";
        h += "Rede baixa e inversor desligados por protecao.<br><br>";
        h += "<b>1.</b> Verifique e resolva o curto na rede de baixa potencia.<br>";
        h += "<b>2.</b> Pressione o botao abaixo apos corrigir.";
        h += "</div>";
        h += "<form method='POST' action='/reset-curto'>";
        h += "<input type='hidden' name='rede' value='baixa'>";
        h += "<button type='submit' class='btn-g' style='margin-top:8px'>";
        h += "&#10003; Curto da Rede Baixa Resolvido - Testar e Religar</button>";
        h += "</form>";
        return h;
    }

    // Falha no próprio inversor
    if (curto.estado == CURTO_AGUARDANDO_RESET) {
        h += "<div class='er'>";
        h += "<b>&#9888; FALHA NO INVERSOR!</b><br>";
        h += "O inversor nao ligou mesmo sem cargas.<br>";
        h += "Verifique o equipamento fisicamente.<br><br>";
        h += "<b>1.</b> Verifique o inversor (sobrecarga, temperatura, fusiveis).<br>";
        h += "<b>2.</b> Pressione o botao abaixo para tentar religar.";
        h += "</div>";
        h += "<form method='POST' action='/reset-curto'>";
        h += "<input type='hidden' name='rede' value='inversor'>";
        h += "<button type='submit' class='btn-o' style='margin-top:8px'>";
        h += "&#9889; Tentar Religar Inversor</button>";
        h += "</form>";
        return h;
    }

    return h;
}

// ============================================================
// PÁGINA /historico - ATUALIZADA PARA LER DA FLASH
// ============================================================
void handleHistorico() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    int filtroAno = server.arg("ano").toInt();
    int filtroMes = server.arg("mes").toInt();
    int filtroDia = server.arg("dia").toInt();

    struct tm ti;
    bool temNTP = getLocalTime(&ti);
    if (filtroAno == 0 && temNTP) filtroAno = ti.tm_year + 1900;
    if (filtroMes == 0 && temNTP) filtroMes = ti.tm_mon + 1;

    String c = "";

    // ── CONTADORES GLOBAIS ────────────────────────────────────
    c += "<h3>&#128200; Contadores Totais</h3>";
    c += "<div style='display:grid;grid-template-columns:1fr 1fr;"
    "gap:6px;margin-bottom:8px'>";

    auto cardCont = [](const char* ico, const char* nome,
                       float kwh, const String& cor) -> String {
                           char buf[24];
                           if (kwh >= 1000)
                               snprintf(buf, sizeof(buf), "%.2f MWh", kwh/1000.0f);
                           else
                               snprintf(buf, sizeof(buf), "%.3f kWh", kwh);
                           String h = "<div style='background:#1a1a1a;border:2px solid #1e1e1e;"
                           "border-radius:8px;padding:10px;text-align:center'>";
                           h += "<div style='font-size:10px;color:#546e7a'>";
                           h += String(ico)+" "+String(nome)+"</div>";
                           h += "<div style='font-size:17px;font-weight:bold;color:"+cor+";"
                           "margin-top:4px'>"+String(buf)+"</div>";
                           h += "</div>";
                           return h;
                       };

                       c += cardCont("&#9728;",   "Painel Gerado",    contadores.painelKwh,  "#fdd835");
                       c += cardCont("&#127968;", "Casa Consumo",     contadores.casaKwh,    "#4fc3f7");
                       c += cardCont("&#9650;",   "Bat. Carregado",   contadores.batCargKwh, "#43a047");
                       c += cardCont("&#9660;",   "Bat. Descarregado",contadores.batDescKwh, "#ef6c00");
                       c += "</div>";

                       // Acumulador hora atual
                       c += "<div class='nf' style='font-size:10px;margin-bottom:8px'>";
                       c += "&#128336; Acumulando hora " + String(acum.horaAtual) + ":00 &rarr; ";
                       char bufAcum[80];
                       snprintf(bufAcum, sizeof(bufAcum),
                                "P=%.1fWh C=%.1fWh BC=%.1fWh BD=%.1fWh",
                                acum.painelWh, acum.casaWh, acum.batCargWh, acum.batDescWh);
                       c += String(bufAcum) + "</div>";

                       // ── FILTRO ────────────────────────────────────────────────
                       c += "<h3>&#128269; Filtrar</h3>";
                       c += "<form method='GET' action='/historico' "
                       "style='display:flex;gap:6px;flex-wrap:wrap;align-items:flex-end'>";

                       c += "<div style='flex:1;min-width:70px'><label style='font-size:11px'>Ano:</label>";
                       snprintf(tempBuf,sizeof(tempBuf),"value='%d'",filtroAno);
                       c += "<input type='number' name='ano' min='2024' max='2099' "+String(tempBuf)+
                       " style='padding:8px;width:100%;background:#111;border:1px solid #444;"
                       "border-radius:4px;color:#e0e0e0;font-size:13px;box-sizing:border-box'></div>";

                       c += "<div style='flex:1;min-width:70px'><label style='font-size:11px'>M&ecirc;s:</label>";
                       c += "<select name='mes' style='padding:8px;width:100%;background:#111;"
                       "border:1px solid #444;border-radius:4px;color:#e0e0e0;"
                       "font-size:12px;box-sizing:border-box'>";
                       const char* meses[]={"Jan","Fev","Mar","Abr","Mai","Jun",
                           "Jul","Ago","Set","Out","Nov","Dez"};
                           for(int m=1;m<=12;m++){
                               c += "<option value='"+String(m)+"'";
                               if(m==filtroMes) c += " selected";
                               c += ">"+String(meses[m-1])+"</option>";
                           }
                           c += "</select></div>";

                           c += "<div style='flex:1;min-width:70px'><label style='font-size:11px'>"
                           "Dia (0=todos):</label>";
                           snprintf(tempBuf,sizeof(tempBuf),"value='%d'",filtroDia);
                           c += "<input type='number' name='dia' min='0' max='31' "+String(tempBuf)+
                           " style='padding:8px;width:100%;background:#111;border:1px solid #444;"
                           "border-radius:4px;color:#e0e0e0;font-size:13px;box-sizing:border-box'></div>";

                           c += "<div style='flex:1;min-width:80px'><label style='font-size:11px'>&nbsp;</label>";
                           c += "<button type='submit' style='width:100%;padding:9px;background:#0277bd;"
                           "color:#fff;border:none;border-radius:4px;font-size:13px;"
                           "font-weight:bold;cursor:pointer'>&#128269; Buscar</button></div>";
                           c += "</form>";

                           // ── RESULTADOS ────────────────────────────────────────────
                           c += "<h3>&#128203; Resultados</h3>";

                           bool encontrou = false;
                           int  diasMostrados = 0;
                           int  maxMostrar = (filtroDia > 0) ? HIST_DIAS : 31; // limita sem dia específico

                           // Percorre do mais recente ao mais antigo
                           for (int i = 0; i < histTotalDias && diasMostrados < maxMostrar; i++) {
                               int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;

                               // Dia atual está em RAM - não precisa ler flash
                               RegistroDia* pDia = nullptr;
                               bool usandoRAM = false;

                               if (i == 0) {
                                   // Dia mais recente = dia atual em RAM
                                   pDia     = &histDiaAtual;
                                   usandoRAM = true;
                               } else {
                                   // Lê da flash para o buffer temporário
                                   if (!carregarDiaFlash(idx, histDiaTemp)) continue;
                                   pDia = &histDiaTemp;
                               }

                               if (!pDia->valido) continue;

                               // Aplica filtro
                               if (filtroAno > 0 && pDia->ano != filtroAno) continue;
                               if (filtroMes > 0 && pDia->mes != filtroMes) continue;
                               if (filtroDia > 0 && pDia->dia != filtroDia) continue;

                               encontrou = true;
                               diasMostrados++;

                               // ── Card do dia ──────────────────────────────────────
                               const char* nomeMes[]={"","Jan","Fev","Mar","Abr","Mai","Jun",
                                   "Jul","Ago","Set","Out","Nov","Dez"};
                                   char titulo[30];
                                   snprintf(titulo,sizeof(titulo),"%02d/%s/%04d",
                                            pDia->dia, nomeMes[pDia->mes], pDia->ano);

                                   // Recalcula totais incluindo hora atual em RAM
                                   if (usandoRAM) calcularTotaisDia(histDiaAtual);

                                   c += "<div style='background:#1a1a1a;border:1px solid #333;"
                                   "border-radius:8px;padding:10px;margin-bottom:8px'>";

                                   // Cabeçalho dia
                                   c += "<div style='display:flex;justify-content:space-between;"
                                   "align-items:center;margin-bottom:6px;padding-bottom:6px;"
                                   "border-bottom:1px solid #2a2a2a'>";
                                   c += "<span style='font-size:13px;font-weight:bold;color:#4fc3f7'>";
                                   c += "&#128197; " + String(titulo);
                                   if (usandoRAM) c += " <span style='font-size:9px;color:#43a047'>(hoje)</span>";
                                   c += "</span>";

                               // Totais resumidos
                               char bufRes[60];
                               snprintf(bufRes,sizeof(bufRes),"%.2f / %.2f kWh",
                                        pDia->totalPainelWh/1000.0f,
                                        pDia->totalCasaWh/1000.0f);
                               c += "<span style='font-size:10px;color:#546e7a'>&#9728;"+
                               String(bufRes)+"</span>";
                               c += "</div>";

                               // Mini cards totais
                               c += "<div style='display:grid;grid-template-columns:1fr 1fr 1fr 1fr;"
                               "gap:4px;margin-bottom:6px'>";

                               auto mCard=[](const char* ic,const char* nm,
                                             float wh,const String& cor)->String{
                                                 char b[20];
                                                 if(wh>=1000) snprintf(b,sizeof(b),"%.2fkWh",wh/1000.0f);
                                                 else         snprintf(b,sizeof(b),"%.0fWh",wh);
                                                 String h="<div style='background:#111;border:1px solid #1e1e1e;"
                                                 "border-radius:6px;padding:6px;text-align:center'>";
                                                 h+="<div style='font-size:9px;color:#546e7a'>"+String(ic)+" "+String(nm)+"</div>";
                                                 h+="<div style='font-size:12px;font-weight:bold;color:"+cor+
                                                 ";margin-top:3px'>"+String(b)+"</div>";
                                                 h+="</div>";
                                                 return h;
                                             };

                                             c += mCard("&#9728;",   "Painel",   pDia->totalPainelWh,  "#fdd835");
                                             c += mCard("&#127968;", "Casa",     pDia->totalCasaWh,    "#4fc3f7");
                                             c += mCard("&#9650;",   "Bat.C",    pDia->totalBatCargWh, "#43a047");
                                             c += mCard("&#9660;",   "Bat.D",    pDia->totalBatDescWh, "#ef6c00");
                                             c += "</div>";

                                             // Tabela horas - só se dia específico
                                             if (filtroDia > 0) {
                                                 c += "<div style='overflow-x:auto'>";
                                                 c += "<table style='width:100%;border-collapse:collapse;font-size:10px'>";
                                                 c += "<tr style='background:#0d0d0d'>"
                                                 "<th style='padding:5px;color:#546e7a;border-bottom:1px solid #222'>Hora</th>"
                                                 "<th style='padding:5px;color:#fdd835;text-align:right;"
                                                 "border-bottom:1px solid #222'>&#9728; Painel</th>"
                                                 "<th style='padding:5px;color:#4fc3f7;text-align:right;"
                                                 "border-bottom:1px solid #222'>&#127968; Casa</th>"
                                                 "<th style='padding:5px;color:#43a047;text-align:right;"
                                                 "border-bottom:1px solid #222'>&#9650; Bat.C</th>"
                                                 "<th style='padding:5px;color:#ef6c00;text-align:right;"
                                                 "border-bottom:1px solid #222'>&#9660; Bat.D</th>"
                                                 "</tr>";

                                                 for(int h=0;h<HIST_HORAS;h++){
                                                     RegistroHora& rh = pDia->horas[h];
                                                     bool eHoraAtual = (usandoRAM && h == acum.horaAtual);
                                                     bool temDados   = (rh.painelWh>0||rh.casaWh>0||
                                                     rh.batCargWh>0||rh.batDescWh>0||eHoraAtual);
                                                     if(!temDados) continue;

                                                     c += "<tr style='border-bottom:1px solid #1a1a1a;";
                                                     if(eHoraAtual) c += "background:#0d1f0d;";
                                                     c += "'>";

                                                     snprintf(tempBuf,sizeof(tempBuf),"%02d:00",h);
                                                     c += "<td style='padding:4px 6px;color:#90a4ae;text-align:center'>";
                                                     c += String(tempBuf);
                                                     if(eHoraAtual) c+=" &#9654;";
                                                     c += "</td>";

                                                     // Adiciona acumulador atual na hora atual
                                                     float painelH = rh.painelWh  + (eHoraAtual ? acum.painelWh  : 0);
                                                     float casaH   = rh.casaWh    + (eHoraAtual ? acum.casaWh    : 0);
                                                     float batCH   = rh.batCargWh + (eHoraAtual ? acum.batCargWh : 0);
                                                     float batDH   = rh.batDescWh + (eHoraAtual ? acum.batDescWh : 0);

                                                     auto tdV=[](float wh,const char* cor)->String{
                                                         char b[16];
                                                         if(wh>=1000) snprintf(b,sizeof(b),"%.2fkWh",wh/1000.0f);
                                                         else         snprintf(b,sizeof(b),"%.0fWh",wh);
                                                         String td="<td style='padding:4px 6px;text-align:right;";
                                                         if(wh>0) td+="color:"+String(cor)+";font-weight:bold";
                                                         else     td+="color:#1e272e";
                                                         td+=">"+String(wh>0?b:"-")+"</td>";
                                                         return td;
                                                     };

                                                     c += tdV(painelH,"#fdd835");
                                                     c += tdV(casaH,  "#4fc3f7");
                                                     c += tdV(batCH,  "#43a047");
                                                     c += tdV(batDH,  "#ef6c00");
                                                     c += "</tr>";
                                                 }

                                                 // Linha total
                                                 c += "<tr style='background:#0d0d0d;border-top:2px solid #333'>"
                                                 "<td style='padding:5px 6px;color:#78909c;font-weight:bold;"
                                                 "text-align:center'>TOTAL</td>";
                                                 auto tdT=[](float wh,const char* cor)->String{
                                                     char b[16];
                                                     if(wh>=1000) snprintf(b,sizeof(b),"%.2fkWh",wh/1000.0f);
                                                     else         snprintf(b,sizeof(b),"%.0fWh",wh);
                                                     return "<td style='padding:5px 6px;text-align:right;"
                                                     "font-weight:bold;color:"+String(cor)+"'>"+String(b)+"</td>";
                                                 };
                                                 c += tdT(pDia->totalPainelWh, "#fdd835");
                                                 c += tdT(pDia->totalCasaWh,   "#4fc3f7");
                                                 c += tdT(pDia->totalBatCargWh,"#43a047");
                                                 c += tdT(pDia->totalBatDescWh,"#ef6c00");
                                                 c += "</tr></table></div>";
                                             }

                                             c += "</div>"; // fecha card dia
                           }

                           if (!encontrou) {
                               c += "<div class='nf' style='text-align:center;padding:20px'>"
                               "&#128203; Nenhum registro encontrado.<br>"
                               "<span style='font-size:10px;color:#546e7a'>"
                               "O historico inicia apos a primeira hora completa com NTP ativo."
                               "</span></div>";
                           }

                           // Info storage
                           c += "<div class='nf' style='font-size:10px;margin-top:8px'>";
                           snprintf(tempBuf, sizeof(tempBuf),
                                    "&#128190; %d/%d dias | Cada dia salvo individualmente na flash "
                                    "| Buffer circular automatico",
                                    histTotalDias, HIST_DIAS);
                           c += String(tempBuf) + "</div>";

                           server.send(200, "text/html; charset=utf-8",
                                       gerarPagina("RRFS - Historico", c));
}

// ============================================================
// PÁGINA: /previsao
// ============================================================
void handlePrevisao() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    String msg = "";

    // ── Processa POST de configuração ────────────────────────
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            String latStr = server.arg("lat");
            String lonStr = server.arg("lon");
            float  kwp    = server.arg("kwp").toFloat();
            float  ef     = server.arg("ef").toFloat();

            // Validações
            double lat = latStr.toDouble();
            double lon = lonStr.toDouble();

            bool ok = true;
            if (lat < -90 || lat > 90)   ok = false;
            if (lon < -180 || lon > 180) ok = false;
            if (kwp < 0.1f || kwp > 100) ok = false;
            if (ef < 10 || ef > 100)     ok = false;

            if (!ok) {
                msg = "<div class='er'>&#10007; Dados invalidos. "
                "Verifique coordenadas e valores.</div>";
            } else {
                configLocal.latitude    = lat;
                configLocal.longitude   = lon;
                configLocal.painelKWp   = kwp;
                configLocal.eficiencia  = ef / 100.0f;
                configLocal.configurado = true;
                // Reseta calibração ao mudar configuração
                configLocal.fatorCalib    = 1.0f;
                configLocal.amostrasCalib = 0;
                salvarConfigLocal();

                // Força busca imediata
                ultimaPrev = 0;

                msg = "<div class='ok'>&#10003; Configuracao salva! "
                "Buscando previsao...</div>";
                adicionarLog("Previsao: nova config salva");
            }
        }

        // Reset calibração manual
        else if (acao == "reset_calib") {
            configLocal.fatorCalib    = 1.0f;
            configLocal.amostrasCalib = 0;
            salvarConfigLocal();
            msg = "<div class='ok'>&#10003; Calibracao resetada</div>";
            adicionarLog("Previsao: calibracao resetada");
        }

        // Forçar atualização
        else if (acao == "atualizar") {
            ultimaPrev = 0;
            msg = "<div class='ok'>&#9679; Atualizando previsao...</div>";
        }
    }

    // ── Monta página ─────────────────────────────────────────
    String c = msg;

    // ── Status da previsão ────────────────────────────────────
    c += "<h3>&#127774; Previsao Solar</h3>";

    if (!configLocal.configurado) {
        c += "<div class='wn'>&#9888; Configure a localizacao abaixo "
        "para ativar a previsao.</div>";
    } else if (!previsao.valido) {
        c += "<div class='wn'>&#128336; Aguardando primeira previsao...</div>";
        if (strlen(previsao.erro) > 0) {
            c += "<div class='er'>Erro: " + String(previsao.erro) + "</div>";
        }
    } else {
        // Tempo desde última atualização
        unsigned long minPassados = (millis() - previsao.ultimaAtualizacao) / 60000;

        c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>";
        c += "&#128336; Ultima atualizacao: ";
        if (minPassados < 60) {
            c += String(minPassados) + " minutos atras";
        } else {
            c += String(minPassados/60) + "h" +
            String(minPassados%60) + "m atras";
        }
        c += " | Proximo em ";
        unsigned long proxMin = (PREV_INTERVALO -
        (millis() - ultimaPrev)) / 60000;
        c += String(proxMin) + " min";
        c += "</div>";

        // Cards principais de previsão
        c += "<div style='display:grid;grid-template-columns:1fr 1fr;"
        "gap:6px;margin-bottom:8px'>";

        // ✅ FUNÇÃO LAMBDA CORRIGIDA - Aceita String
        auto cardPrev = [](const String& ico, const String& titulo,
                           const String& valor, const String& sub,
                           const String& cor) -> String {
                               String h = "<div style='background:#1a1a1a;border:2px solid #1e1e1e;"
                               "border-radius:8px;padding:10px;text-align:center'>";
                               h += "<div style='font-size:10px;color:#546e7a'>";
                               h += ico + " " + titulo + "</div>";
                               h += "<div style='font-size:17px;font-weight:bold;color:" + cor + ";"
                               "margin-top:4px'>" + valor + "</div>";
                               h += "<div style='font-size:10px;color:#546e7a;margin-top:2px'>";
                               h += sub + "</div>";
                               h += "</div>";
                               return h;
                           };

                           char bH[20], bA[20], bT[20];
                           snprintf(bH, sizeof(bH), "%.2f kWh",
                                    previsao.geracaoHojeRestante / 1000.0f);
                           snprintf(bA, sizeof(bA), "%.2f kWh",
                                    previsao.geracaoAmanha / 1000.0f);
                           snprintf(bT, sizeof(bT), "%.2f kWh",
                                    previsao.geracaoTotal48h / 1000.0f);

                           // ✅ CHAMADAS CORRIGIDAS - Convertendo para String
                           c += cardPrev("&#9728;", "Geracao Hoje (restante)",
                                         String(bH), "estimativa ate anoitecer", "#fdd835");

                           c += cardPrev("&#127773;", "Geracao Amanha",
                                         String(bA), "previsao dia todo", "#4fc3f7");

                           c += cardPrev("&#128267;", "Bateria Atual",
                                         String(bateria.percentualAtual, 0) + "%",
                                         String(bateria.energiaAtual, 0) + "Wh disp.", "#43a047");

                           c += cardPrev("&#128200;", "Total 48h",
                                         String(bT), "geracao estimada", "#ef6c00");

                           c += "</div>";

                           // ── Decisões automáticas ──────────────────────────────
                           c += "<h3>&#129504; Decisoes Automaticas</h3>";

                           if (decisao.alertaEnergiaInsuficiente) {
                               c += "<div class='er'>&#9888; ALERTA: Energia pode ser insuficiente "
                               "para a noite! Geracao amanha tambem baixa.</div>";
                           }

                           c += "<div style='background:#111;border:1px solid #1e1e1e;"
                           "border-radius:8px;padding:10px;margin-bottom:8px'>";

                           // Rede alta
                           {
                               String cor = decisao.desligarRedeAltaNoite ? "#ef6c00" : "#43a047";
                               String st  = decisao.desligarRedeAltaNoite ?
                               "Desligada (economia)" : "Normal";
                               c += "<div style='display:flex;justify-content:space-between;"
                               "align-items:center;padding:6px 0;"
                               "border-bottom:1px solid #1a1a1a'>";
                               c += "<span style='font-size:12px;color:#90a4ae'>"
                               "&#128268; Rede Alta</span>";
                               c += "<span style='font-size:11px;font-weight:bold;color:" + cor + "'>";
                               c += st + "</span></div>";
                           }

                           // Lamp externa
                           {
                               String cor = decisao.desligarLampExtNoite ? "#ef6c00" : "#43a047";
                               String st  = sensores.correntePainel > 0.5f ?
                               "Desligada (dia)" :
                               (decisao.desligarLampExtNoite ?
                               "Desligada (economia)" : "Normal");
                               c += "<div style='display:flex;justify-content:space-between;"
                               "align-items:center;padding:6px 0;"
                               "border-bottom:1px solid #1a1a1a'>";
                               c += "<span style='font-size:12px;color:#90a4ae'>"
                               "&#128161; Lamp. Externa</span>";
                               c += "<span style='font-size:11px;font-weight:bold;color:" + cor + "'>";
                               c += st + "</span></div>";
                           }

                           // Motivo
                           if (strlen(decisao.motivo) > 0) {
                               c += "<div style='font-size:10px;color:#546e7a;margin-top:6px'>";
                               c += "&#128220; " + String(decisao.motivo) + "</div>";
                           }

                           c += "</div>";

                           // ── Calibração ────────────────────────────────────────
                           c += "<h3>&#127919; Calibracao Automatica</h3>";
                           c += "<div style='background:#111;border:1px solid #1e1e1e;"
                           "border-radius:8px;padding:10px;margin-bottom:8px'>";

                           // Barra de precisão
                           float precisao = min(100.0f,
                                                (float)configLocal.amostrasCalib / 50.0f * 100.0f);
                           String corPrec = precisao > 60 ? "#43a047" :
                           precisao > 30 ? "#ef6c00" : "#c62828";

                           c += "<div style='display:flex;justify-content:space-between;"
                           "margin-bottom:4px'>";
                           c += "<span style='font-size:11px;color:#78909c'>Precisao da previsao</span>";
                           c += "<span style='font-size:11px;font-weight:bold;color:" + corPrec + "'>";
                           c += String((int)precisao) + "%</span></div>";

                           c += "<div class='barra-bg'><div class='barra' style='width:";
                           c += String((int)precisao) + "%;background:" + corPrec + "'>";
                           c += String((int)precisao) + "%</div></div>";

                           char bufCalib[80];
                           snprintf(bufCalib, sizeof(bufCalib),
                                    "Fator: %.3f | Amostras: %d/50",
                                    configLocal.fatorCalib, configLocal.amostrasCalib);
                           c += "<div style='font-size:10px;color:#546e7a;margin-top:4px'>";
                           c += String(bufCalib) + "</div>";

                           if (configLocal.amostrasCalib < 10) {
                               c += "<div style='font-size:10px;color:#78909c;margin-top:4px'>"
                               "&#128336; Calibracao em andamento. Melhora ao longo dos dias "
                               "com painel gerando energia.</div>";
                           } else if (configLocal.amostrasCalib < 30) {
                               c += "<div style='font-size:10px;color:#ef6c00;margin-top:4px'>"
                               "&#127919; Calibracao parcial. Continue usando para maior "
                               "precisao.</div>";
                           } else {
                               c += "<div style='font-size:10px;color:#43a047;margin-top:4px'>"
                               "&#10003; Boa precisao! Sistema bem calibrado.</div>";
                           }

                           c += "</div>";

                           // Botão forçar atualização
                           c += "<form method='POST' action='/previsao'>";
                           c += "<input type='hidden' name='acao' value='atualizar'>";
                           c += "<button type='submit' class='btn-s'>"
                           "&#128260; Forcar Atualizacao da Previsao</button>";
                           c += "</form>";

                           // Botão reset calibração
                           c += "<form method='POST' action='/previsao' style='margin-top:6px' "
                           "onsubmit='return confirm(\"Resetar calibracao?\")'>";
                           c += "<input type='hidden' name='acao' value='reset_calib'>";
                           c += "<button type='submit' class='btn-r btn-sm'>"
                           "&#8635; Resetar Calibracao</button>";
                           c += "</form>";

                           // ── Tabela de previsão por hora ───────────────────────
                           c += "<h3>&#128336; Previsao por Hora</h3>";

                           struct tm ti2;
                           getLocalTime(&ti2);

                           // Agrupar por dia
                           int diaAnterior = -1;

                           c += "<div style='overflow-x:auto'>";
                           c += "<table style='width:100%;border-collapse:collapse;font-size:10px'>";
                           c += "<tr style='background:#0d0d0d'>"
                           "<th style='padding:5px;color:#546e7a;text-align:center;"
                           "border-bottom:1px solid #222'>Hora</th>"
                           "<th style='padding:5px;color:#fdd835;text-align:right;"
                           "border-bottom:1px solid #222'>&#9728; Rad.</th>"
                           "<th style='padding:5px;color:#81d4fa;text-align:right;"
                           "border-bottom:1px solid #222'>&#9729; Nuv.</th>"
                           "<th style='padding:5px;color:#ef5350;text-align:right;"
                           "border-bottom:1px solid #222'>&#127777; Temp</th>"
                           "<th style='padding:5px;color:#43a047;text-align:right;"
                           "border-bottom:1px solid #222'>&#9889; Ger.</th>"
                           "</tr>";

                           for (int i = 0; i < previsao.totalHoras; i++) {
                               PrevisaoHora& ph = previsao.horas[i];
                               if (!ph.valido) continue;

                               // Cabeçalho do dia
                               if (ph.dia != diaAnterior) {
                                   diaAnterior = ph.dia;
                                   const char* nd[]={"","Jan","Fev","Mar","Abr","Mai","Jun",
                                       "Jul","Ago","Set","Out","Nov","Dez"};
                                       c += "<tr><td colspan='5' style='padding:6px;background:#1a1a1a;"
                                       "color:#4fc3f7;font-weight:bold;font-size:11px'>";
                                       char dtBuf[20];
                                       snprintf(dtBuf, sizeof(dtBuf), "%02d/%s/%04d",
                                                ph.dia, nd[ph.mes], ph.ano);
                                       c += "&#128197; " + String(dtBuf);

                                       // Indica "hoje" ou "amanhã"
                                       if (ph.dia == ti2.tm_mday && ph.mes == (ti2.tm_mon+1))
                                           c += " <span style='color:#43a047'>(hoje)</span>";
                                   else
                                       c += " <span style='color:#546e7a'>(amanha)</span>";

                                   c += "</td></tr>";
                               }

                               // Destaca hora atual
                               bool eAgora = (ph.dia  == ti2.tm_mday &&
                               ph.mes  == (ti2.tm_mon+1) &&
                               ph.hora == (uint8_t)ti2.tm_hour);

                               c += "<tr style='border-bottom:1px solid #1a1a1a;";
                               if (eAgora) c += "background:#0d1f0d;";
                               c += "'>";

                               // Hora
                               c += "<td style='padding:4px 6px;color:#90a4ae;text-align:center'>";
                               snprintf(tempBuf, sizeof(tempBuf), "%02d:00", ph.hora);
                               c += String(tempBuf);
                               if (eAgora) c += "&#9654;";
                               c += "</td>";

                               // Radiação
                               char bR[12];
                               snprintf(bR, sizeof(bR), "%.0fW/m&sup2;",
                                        ph.radiacao + ph.radiacaoDifusa);
                               c += "<td style='padding:4px 6px;text-align:right;color:";
                               c += ph.radiacao > 100 ? "#fdd835" : "#455a64";
                               c += "'>" + String(bR) + "</td>";

                               // Nuvens
                               char bN[8];
                               snprintf(bN, sizeof(bN), "%.0f%%", ph.coberturaNuvens);
                               c += "<td style='padding:4px 6px;text-align:right;color:";
                               c += ph.coberturaNuvens > 70 ? "#78909c" :
                               ph.coberturaNuvens > 30 ? "#90a4ae" : "#4fc3f7";
                               c += "'>" + String(bN) + "</td>";

                               // Temperatura
                               char bT2[8];
                               snprintf(bT2, sizeof(bT2), "%.0f&deg;C", ph.temperatura);
                               c += "<td style='padding:4px 6px;text-align:right;color:#ef5350'>";
                               c += String(bT2) + "</td>";

                               // Geração estimada
                               char bG[16];
                               if (ph.geracaoEstimada >= 1000)
                                   snprintf(bG, sizeof(bG), "%.2fkWh", ph.geracaoEstimada/1000.0f);
                               else
                                   snprintf(bG, sizeof(bG), "%.0fWh", ph.geracaoEstimada);
                               c += "<td style='padding:4px 6px;text-align:right;font-weight:bold;"
                               "color:";
                               c += ph.geracaoEstimada > 500 ? "#43a047" :
                               ph.geracaoEstimada > 100 ? "#fdd835" : "#455a64";
                               c += "'>" + String(ph.geracaoEstimada > 0 ? bG : "-");
                               c += "</td>";

                               c += "</tr>";
                           }
                           c += "</table></div>";
    }

    // ── Configuração ──────────────────────────────────────────
    c += "<h3>&#9881; Configurar Localizacao</h3>";

    // Botão abrir Google Maps
    c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>";
    c += "&#128205; Para obter coordenadas de alta precisao:<br>";
    c += "1. Clique no botao abaixo para abrir o Google Maps<br>";
    c += "2. Localize sua instalacao no mapa<br>";
    c += "3. Clique com o botao direito no local exato<br>";
    c += "4. Copie as coordenadas que aparecem (ex: -23.667768, -48.184803)<br>";
    c += "5. Cole nos campos abaixo";
    c += "</div>";

    c += "<a href='https://www.google.com/maps' target='_blank' "
    "style='display:block;padding:10px;background:#1565c0;"
    "color:#fff;text-align:center;border-radius:6px;"
    "text-decoration:none;font-weight:bold;margin-bottom:10px'>"
    "&#127759; Abrir Google Maps (nova aba)</a>";

    c += "<form method='POST' action='/previsao'>";
    c += "<input type='hidden' name='acao' value='config'>";

    // Latitude
    c += "<label>Latitude (ex: -23.667768108648623):</label>";
    char bufLat[30], bufLon[30];
    snprintf(bufLat, sizeof(bufLat), "%.15f", configLocal.latitude);
    snprintf(bufLon, sizeof(bufLon), "%.15f", configLocal.longitude);
    c += "<input type='text' name='lat' required "
    "placeholder='-23.667768108648623' ";
    if (configLocal.configurado) c += "value='" + String(bufLat) + "'";
    c += ">";

    // Longitude
    c += "<label>Longitude (ex: -48.184803456419935):</label>";
    c += "<input type='text' name='lon' required "
    "placeholder='-48.184803456419935' ";
    if (configLocal.configurado) c += "value='" + String(bufLon) + "'";
    c += ">";

    // kWp
    c += "<label>Potencia Total dos Paineis (kWp):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Ex: 2 paineis de 550W = 1.1 kWp | "
    "4 paineis de 400W = 1.6 kWp</div>";
    c += "<input type='number' name='kwp' step='0.01' "
    "min='0.1' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.2f'", configLocal.painelKWp);
    c += String(tempBuf) + " placeholder='Ex: 1.1'>";

    // Eficiência
    c += "<label>Eficiencia do Sistema (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Tipico: 75-85% | Inclui perdas de cabos, inversor e sujeira</div>";
    c += "<input type='number' name='ef' step='1' "
    "min='10' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'",
             configLocal.eficiencia * 100.0f);
    c += String(tempBuf) + " placeholder='Ex: 80'>";

    c += "<button type='submit' class='btn-g'>"
    "&#128190; Salvar e Ativar Previsao</button>";
    c += "</form>";

    // Info API
    c += "<div class='nf' style='font-size:10px;margin-top:8px'>";
    c += "&#127760; Dados: Open-Meteo (open-meteo.com) - Gratuito, sem chave de API<br>";
    c += "&#128336; Atualizacao automatica a cada 1 hora<br>";
    c += "&#127919; Precisao melhora automaticamente ao longo do tempo";
    c += "</div>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Previsao", c));
}

// ============================================================
// ENDPOINT: /reset-curto
// POST com campo 'rede' = 'alta' | 'baixa' | 'inversor'
// ============================================================
void handleResetCurto() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    if (server.method() == HTTP_POST) {
        String rede = server.arg("rede");

        if (rede == "alta") {
            resetCurtoAlta();
        }
        else if (rede == "baixa") {
            resetCurtoBaixa();
        }
        else if (rede == "inversor") {
            // Tenta religar inversor do zero
            curto.estado    = CURTO_NORMAL;
            curto.subEstado = SUB_IDLE;
            curto.tInicio   = 0;
            curto.monitorando = false;
            adicionarLog("Reset manual do inversor");
            ligarInversor();
        }
    }

    server.sendHeader("Location", "/redes");
    server.send(302);
}

// ============================================================
// ROTAS
// ============================================================
void configurarRotas() {
    server.on("/",            handleRoot);
    server.on("/login",       handleLogin);
    server.on("/logout",      handleLogout);
    server.on("/setup-login", handleSetupLogin);
    server.on("/setup-wifi",  handleSetupWifi);
    server.on("/bateria",     handleBateria);
    server.on("/redes",       handleRedes);
    server.on("/sensores",    handleSensores);
    server.on("/config",      handleConfig);
    server.on("/logs",        handleLogs);
    server.on("/dados",       handleDados);
    server.on("/reset-curto", handleResetCurto);
    server.on("/ntp-sync",    handleNTPSync);
    server.on("/reiniciar",   handleReiniciar);
    server.on("/historico", handleHistorico);
    server.on("/previsao", handlePrevisao);

    // OTA - rota com handler de upload
    server.on("/ota-upload", HTTP_POST,
              handleOTAResult,   // Chamado ao finalizar
              handleOTAUpload    // Chamado durante upload
    );

    server.onNotFound(handleNotFound);
    server.begin();
}

// ============================================================
// REDE
// ============================================================
void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP ativo: RRFS Energia");
    adicionarLog("IP: 10.0.0.1");
    logRAM(); ledBlink();
}

bool iniciarWiFi(const String& ssid, const String& pass,
                  const String& ip,   const String& gw,
                  const String& mask, const String& dns) {

    IPAddress lIP, lGW, lMask, lDNS;

    if (!lIP.fromString(ip))     { adicionarLog("ERRO: IP invalido");   return false; }
    if (!lGW.fromString(gw))     { adicionarLog("ERRO: GW invalido");   return false; }
    if (!lMask.fromString(mask)) { adicionarLog("ERRO: Mask invalida"); return false; }
    if (!lDNS.fromString(dns))   { adicionarLog("ERRO: DNS invalido");  return false; }

    // Salva globalmente para reconexão automática
    gWifiSSID = ssid;
    gWifiPASS = pass;
    gWifiIP   = ip;
    gWifiGW   = gw;
    gWifiMASK = mask;
    gWifiDNS  = dns;

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(300);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    // Configura IP estático
    if (!WiFi.config(lIP, lGW, lMask, lDNS)) {
        adicionarLog("ERRO: Config IP estatico falhou");
        return false;
    }

    // Habilita reconexão automática do driver
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    snprintf(tempBuf, sizeof(tempBuf),
             "Conectando: %s | IP: %s", ssid.c_str(), ip.c_str());
    adicionarLog(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf),
             "GW: %s | DNS: %s", gw.c_str(), dns.c_str());
    adicionarLog(tempBuf);
    ledBlink();

    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long t    = millis();
    int           tent = 0;

    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
        atualizarLED();
        delay(500);
        tent++;
        if (tent % 6 == 0) {
            snprintf(tempBuf, sizeof(tempBuf),
                     "Aguardando WiFi... %ds", tent / 2);
            adicionarLog(tempBuf);
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi OK! IP: %s",
                 WiFi.localIP().toString().c_str());
        adicionarLog(tempBuf);
        snprintf(tempBuf, sizeof(tempBuf),
                 "RSSI: %ddBm | Canal: %d",
                 WiFi.RSSI(), WiFi.channel());
        adicionarLog(tempBuf);
        logRAM();

        modoAP = false;
        ledFixo(true);

        // Sincroniza hora via NTP
        sincronizarNTP();

        return true;
    }

    // Diagnóstico de falha
    switch (WiFi.status()) {
        case WL_NO_SSID_AVAIL:
            adicionarLog("ERRO: SSID nao encontrado");
            adicionarLog("Verifique o nome da rede WiFi");
            break;
        case WL_CONNECT_FAILED:
            adicionarLog("ERRO: Senha incorreta");
            break;
        case WL_CONNECTION_LOST:
            adicionarLog("ERRO: Conexao perdida durante tentativa");
            break;
        case WL_DISCONNECTED:
            adicionarLog("ERRO: WiFi desconectado");
            break;
        default:
            snprintf(tempBuf, sizeof(tempBuf),
                     "ERRO WiFi status: %d", WiFi.status());
            adicionarLog(tempBuf);
    }
    return false;
}

// ============================================================
// RECONEXÃO WIFI INTELIGENTE
// Tenta reconectar usando credenciais salvas globalmente
// Sem acionar modo AP imediatamente - tenta várias vezes
// ============================================================
void tentarReconectarWiFi() {
    static uint8_t tentativasRecon = 0;
    static bool    emReconexao     = false;

    // Evita chamadas simultâneas
    if (emReconexao) return;
    emReconexao = true;

    tentativasRecon++;
    snprintf(tempBuf, sizeof(tempBuf),
             "WiFi perdido - tentativa %d/5", tentativasRecon);
    adicionarLog(tempBuf);

    // Reaplica IP estático antes de reconectar
    // (obrigatório para IP fixo no ESP32)
    if (!gWifiIP.isEmpty()) {
        IPAddress lIP, lGW, lMask, lDNS;
        lIP.fromString(gWifiIP);
        lGW.fromString(gWifiGW);
        lMask.fromString(gWifiMASK);
        lDNS.fromString(gWifiDNS);
        WiFi.config(lIP, lGW, lMask, lDNS);
    }

    WiFi.reconnect();

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        atualizarLED();
        server.handleClient();
        if (modoAP) dnsServer.processNextRequest();
        delay(200);
    }

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi reconectado! RSSI: %ddBm", WiFi.RSSI());
        adicionarLog(tempBuf);

        // Reseta contagem de tentativas
        tentativasRecon = 0;
        ledFixo(true);

        // Se estava em modo AP+STA, volta para STA apenas
        if (WiFi.getMode() == WIFI_AP_STA) {
            adicionarLog("Desativando AP temporario...");
            delay(1000); // Dá tempo do cliente web receber resposta

            WiFi.mode(WIFI_STA);
            WiFi.setSleep(false);

            // Reaplica IP estático após trocar modo
            if (!gWifiIP.isEmpty()) {
                IPAddress lIP, lGW, lMask, lDNS;
                lIP.fromString(gWifiIP);
                lGW.fromString(gWifiGW);
                lMask.fromString(gWifiMASK);
                lDNS.fromString(gWifiDNS);
                WiFi.config(lIP, lGW, lMask, lDNS);
            }

            modoAP = false;

            // Reinicia servidor web
            server.close();
            delay(200);
            configurarRotas();

            adicionarLog("Modo STA restaurado");
        }

        // Sincroniza hora via NTP após reconectar
        sincronizarNTP();

        emReconexao = false;
        return;
    }

    // Todas as tentativas falharam → ativa modo AP temporário
    if (tentativasRecon >= 5) {
        adicionarLog("5 tentativas falharam");
        adicionarLog("Ativando AP temporario...");
        tentativasRecon = 0;

        // Modo duplo AP+STA: usuário acessa pelo AP
        // enquanto ESP32 continua tentando o WiFi
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
        WiFi.softAP(AP_SSID);
        dnsServer.start(DNS_PORT, "*", AP_IP);

        // Reaplica IP estático no modo STA
        if (!gWifiIP.isEmpty()) {
            IPAddress lIP, lGW, lMask, lDNS;
            lIP.fromString(gWifiIP);
            lGW.fromString(gWifiGW);
            lMask.fromString(gWifiMASK);
            lDNS.fromString(gWifiDNS);
            WiFi.config(lIP, lGW, lMask, lDNS);
        }

        modoAP = true;
        ledBlink();

        // Reinicia servidor para aceitar conexões pelo AP
        server.close();
        delay(200);
        configurarRotas();

        adicionarLog("AP temporario ativo: RRFS Energia");
        adicionarLog("IP AP: 10.0.0.1");
        adicionarLog("Continuando tentativas em background...");
    }

    emReconexao = false;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    setupLED();
    setupBotao();
    setupReles();
    ledBlink();

    adicionarLog("RRFS Energia v6.0");
    logRAM();

    // Inicializa calibração com valores padrão
    calibPainel.offsetMV  = OFFSET_LIDO_MV;
    calibPainel.sensMvA   = PAINEL_SENS_MV_A * 0.66f;
    calibPainel.invertido = false;
    calibPainel.calibrado = false;

    calibCasa.offsetMV  = OFFSET_LIDO_MV;
    calibCasa.sensMvA   = CASA_SENS_MV_A * 0.66f;
    calibCasa.invertido = false;
    calibCasa.calibrado = false;

    // Carrega configurações salvas
    carregarConfigBateria();
    carregarConfigRedes();
    carregarContadores();
    carregarHistorico();
    adicionarLog("Historico e contadores carregados");
    carregarCalibSensores();
    calcularBateria();
    ultimoContagem = millis();

    // previção de geraçâo de energia
    carregarConfigLocal();

    adicionarLog("Config carregada");
    snprintf(tempBuf, sizeof(tempBuf),
             "Bat: %.1fV %.0fAh %.0f%% | Calib:%s",
             bateria.tensaoNominal, bateria.capacidadeAh,
             bateria.descargaSegura, bateria.calibrada ? "SIM" : "NAO");
    adicionarLog(tempBuf);

    // *** SEQUÊNCIA DE INICIALIZAÇÃO DAS CARGAS ***
    // Ordem importa: inversor precisa estar ligado antes das redes

    // 1. Liga rede baixa primeiro (roteador precisa iniciar)
    ligarRedeBaixa();
    adicionarLog("Rede baixa ON (roteador)");

    // 2. Aguarda roteador inicializar antes de tentar WiFi
    adicionarLog("Aguardando roteador 8s...");
    unsigned long tRoteador = millis();
    while (millis() - tRoteador < 8000) {
        atualizarLED();
        delay(10);
    }
    adicionarLog("Roteador pronto");

    // 3. Liga o inversor
    ligarInversor();
    adicionarLog("Inversor ON");

    // 4. Aguarda inversor inicializar (inversor demora alguns segundos)
    adicionarLog("Aguardando inversor 5s...");
    unsigned long tInversor = millis();
    while (millis() - tInversor < 5000) {
        atualizarLED();
        delay(10);
    }

    // 5. Verifica se inversor ligou corretamente pelo status LED
    bool invOK = false;
    unsigned long tCheckInv = millis();
    while (millis() - tCheckInv < 3000) {
        if (lerStatusInversor()) {
            invOK = true;
            break;
        }
        delay(200);
    }

    if (invOK) {
        adicionarLog("Inversor STATUS: OK");

        // 6. Liga rede alta (equipamentos pesados)
        ligarRedeAlta();
        adicionarLog("Rede alta ON");
        delay(500);

        // 7. Liga lâmpadas
        ligarLampExt();
        adicionarLog("Lamp. externa ON");
        delay(200);

        ligarLampInt();
        adicionarLog("Lamp. interna ON");

    } else {
        // Inversor não respondeu - pode ser normal se
        // o inversor não tiver carga mínima para o LED acender
        // Liga as redes mesmo assim mas registra o aviso
        adicionarLog("AVISO: Inversor sem status - ligando redes mesmo assim");

        ligarRedeAlta();
        adicionarLog("Rede alta ON");
        delay(500);

        ligarLampExt();
        adicionarLog("Lamp. externa ON");
        delay(200);

        ligarLampInt();
        adicionarLog("Lamp. interna ON");
    }

    logRAM();

    // Inicia rede WiFi
    bool temCred = existeCredenciais();
    bool temWifi = existeWiFiConfig();

    if (!temCred || !temWifi) {
        adicionarLog("Modo setup inicial - AP");
        iniciarAP();
    } else {
        String ssid, pass, ip, gw, mask, dns;
        bool ap;
        carregarWifi(ssid, pass, ip, gw, mask, dns, ap);
        if (ap) {
            adicionarLog("Modo AP configurado");
            iniciarAP();
        } else {
            adicionarLog("Conectando WiFi...");
            if (!iniciarWiFi(ssid, pass, ip, gw, mask, dns)) {
                adicionarLog("Fallback -> AP");
                iniciarAP();
            }
        }
    }

    configurarRotas();
    adicionarLog("Servidor web ATIVO");
    logRAM();

    // Leitura inicial dos sensores
    lerSensores();
    adicionarLog("Sensores iniciados");

    // Inicializa monitoramento curto
    // Monitoramento inicia após boot completo
    curto.monitorando = false;
    curto.estado      = CURTO_NORMAL;
    curto.subEstado   = SUB_IDLE;
    curto.tInicio     = 0;

    adicionarLog("Sistema pronto!");
    logRAM();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    verificarReset();
    atualizarLED();

    if (modoAP) dnsServer.processNextRequest();
    server.handleClient();

    unsigned long agora = millis();

    // --------------------------------------------------------
    // LEITURA DE SENSORES - a cada 1s
    // --------------------------------------------------------
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;
        lerSensores();
        acumularEnergia();
    }

    // --------------------------------------------------------
    // MONITORAMENTO DE CURTO
    // Chama sempre que inversor ligado OU em processo
    // de religamento/diagnóstico (mesmo com inversor desligado)
    // --------------------------------------------------------
    bool emProcessoCurto = (curto.estado == CURTO_RELIGANDO      ||
    curto.estado == CURTO_TESTANDO_ALTA  ||
    curto.estado == CURTO_TESTANDO_BAIXA);

    if (estados.inversor || emProcessoCurto) {
        monitorarInversor();
    } else {
        // Inversor desligado e sem processo ativo
        // Reseta monitoramento apenas se estava monitorando
        if (curto.estado == CURTO_NORMAL && curto.monitorando) {
            curto.monitorando       = false;
            curto.tInicio           = 0;
            curto.tentativasReligar = 0;
        }
    }

    // --------------------------------------------------------
    // CONTROLE DE ENERGIA - a cada 5s
    // Não executa durante diagnóstico de curto ativo
    // Em curto de alta: controla tudo exceto rede alta
    // --------------------------------------------------------
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;

        // Não executa controle durante processo de diagnóstico
        // para não interferir nos testes de curto
        if (!emProcessoCurto) {
            if (curto.estado == CURTO_NORMAL ||
                curto.estado == CURTO_CONFIRMADO_ALTA) {
                controlarEnergia();
                }
                // CURTO_CONFIRMADO_BAIXA e CURTO_AGUARDANDO_RESET:
                // não executa - sistema em proteção total
        }
    }

    // --------------------------------------------------------
    // GERENCIAMENTO WIFI
    // --------------------------------------------------------
    static unsigned long tWifiCheck = 0;
    static unsigned long tWifiRecon = 0;
    static bool          eraAP_STA  = false;

    // Verifica status WiFi a cada 15s
    if (!gWifiSSID.isEmpty() && agora - tWifiCheck >= 15000) {
        tWifiCheck = agora;

        if (WiFi.status() != WL_CONNECTED) {

            // Tenta reconectar a cada 30s
            if (agora - tWifiRecon >= 30000) {
                tWifiRecon = agora;
                tentarReconectarWiFi();
            }

        } else {
            // WiFi conectado

            // Verifica se estava em AP+STA e WiFi voltou sozinho
            if (modoAP && eraAP_STA) {
                adicionarLog("WiFi restaurado - desativando AP temporario");

                // Reaplica IP estático
                IPAddress lIP, lGW, lMask, lDNS;
                lIP.fromString(gWifiIP);
                lGW.fromString(gWifiGW);
                lMask.fromString(gWifiMASK);
                lDNS.fromString(gWifiDNS);
                WiFi.config(lIP, lGW, lMask, lDNS);

                // Volta para modo STA apenas
                WiFi.mode(WIFI_STA);
                WiFi.setSleep(false);

                modoAP    = false;
                eraAP_STA = false;
                ledFixo(true);

                // Reinicia servidor web
                server.close();
                delay(200);
                configurarRotas();

                snprintf(tempBuf, sizeof(tempBuf),
                         "WiFi OK: %s RSSI: %ddBm",
                         WiFi.SSID().c_str(), WiFi.RSSI());
                adicionarLog(tempBuf);

                // Sincroniza NTP após restaurar WiFi
                sincronizarNTP();
            }

            // Detecta transição para AP+STA
            if (modoAP && WiFi.getMode() == WIFI_AP_STA) {
                eraAP_STA = true;
            }

            // Resincroniza NTP uma vez por dia
            static unsigned long tNTPResinc = 0;
            if (agora - tNTPResinc >= 86400000UL) {
                tNTPResinc = agora;
                adicionarLog("NTP: resincronizacao diaria");
                sincronizarNTP();
            }

            // Log RSSI a cada 5min
            static unsigned long tRSSI = 0;
            if (agora - tRSSI >= 300000) {
                tRSSI = agora;
                snprintf(tempBuf, sizeof(tempBuf),
                         "WiFi: %s %ddBm",
                         WiFi.SSID().c_str(), WiFi.RSSI());
                adicionarLog(tempBuf);
            }
        }
    }

    // --------------------------------------------------------
    // SALVA ENERGIA DA BATERIA - a cada 5min
    // --------------------------------------------------------
    static unsigned long tSalva = 0;
    if (bateria.calibrada && agora - tSalva >= 300000) {
        tSalva = agora;
        prefs.begin("rrfs", false);
        prefs.putFloat("bat_nrg", bateria.energiaAtual);
        prefs.end();
    }

    // ── Previsão meteorológica - a cada 1 hora ────────────────
    if (configLocal.configurado &&
        WiFi.status() == WL_CONNECTED &&
        !buscandoPrev &&
        millis() - ultimaPrev >= PREV_INTERVALO) {
        ultimaPrev = millis();
        buscarPrevisaoMeteo();
    }

    // ── Calibração automática - a cada 5 minutos ──────────────
    static unsigned long tCalib = 0;
    if (millis() - tCalib >= 300000) {
        tCalib = millis();
        calibrarPrevisao();
        verificarReligamentoAuto();
    }

    // --------------------------------------------------------
    // LOG RAM - a cada 10min
    // --------------------------------------------------------
    static unsigned long tRAM = 0;
    if (agora - tRAM >= 600000) {
        tRAM = agora;
        logRAM();
    }

    delay(1);
}
