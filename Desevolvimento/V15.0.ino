/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v15.0
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 *
 * MUDANÇAS v15.0:
 * - REMOVIDO: Estimativa de tensão (ComplexidadeAlta/PrecisãoBaixa)
 * - ADICIONADO: Foco em corrente real (sensores ACS758)
 * - ALTERADO: kWp do painel → Amperagem do controlador de carga
 * - ALTERADO: Energia em Ah (corrente × tempo) em vez de Wh
 * - MANTIDO: kW calculado com tensão nominal (apenas informativo)
 * - OTIMIZADO: Remoção de funções desnecessárias (~200 linhas)
 * - ORGANIZADO: Tags [SIS-XXX] em todos os blocos
 * ════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════
// [SIS-INC] INCLUDES
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
// [SIS-HW] PINOS DE HARDWARE
// ═══════════════════════════════════════════════════════════

// ── Interface ──────────────────────────────────────────────
#define LED_AZUL              2
#define BOTAO_BOOT            0

// ── Sensores de Corrente (ADC1 - WiFi safe) ───────────────
#define PIN_SENSOR_PAINEL    34   // ACS758 50A  bidirecional
#define PIN_SENSOR_CASA      35   // ACS758 100A unidirecional

// ── Controle e Status ──────────────────────────────────────
#define PIN_BKP_BATERIA      32
#define PIN_STATUS_INV_ALTA  36

// ── Relés (lógica: HIGH=ligado, LOW=desligado) ─────────────
#define PIN_RELE_INV_ALTA     27
#define PIN_RELE_REDE_BAIXA   26
#define PIN_RELE_REDE_ALTA    25
#define PIN_RELE_LAMP_EXTERNA 14
#define PIN_RELE_LAMP_INTERNA 12

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] CONSTANTES GERAIS
// ═══════════════════════════════════════════════════════════
#define MAX_LOGS          20
#define LOG_MAX_LEN      100
#define DNS_PORT          53
#define TEMPO_RESET_MS 10000

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] SENSORES ADC
// ═══════════════════════════════════════════════════════════
#define ADC_SAMPLES       150     // Amostras por leitura (média)
#define ADC_VREF         3300     // mV referência ESP32
#define ADC_BITS         4095     // Resolução 12 bits

// ── ACS758 50A Bidirecional (Painel) ──────────────────────
// Cabos INVERTIDOS: sinal desce = painel gerando (seguro ESP32)
#define PAINEL_OFFSET_MV  2500    // mV com 0A (VCC/2)
#define PAINEL_SENS_MV_A    40    // mV/A datasheet

// ── ACS758LCB-100B Unidirecional (Casa) ───────────────────
// Offset fixo de fábrica: 600mV | Sinal max(100A)=2600mV
#define CASA_OFFSET_MV     600    // mV com 0A (fixo fabricante)
#define CASA_SENS_MV_A      20    // mV/A datasheet

// ── Filtros ────────────────────────────────────────────────
#define FILTRO_RUIDO_A    0.3f    // Abaixo disso = zero
#define BUFFER_CORRENTE     10    // Tamanho buffer média móvel

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] INVERSOR E RELÉS
// ═══════════════════════════════════════════════════════════
#define INV_THRESHOLD_MV  1500    // Limiar sinal LED inversor
#define INV_BUFFER_MS     2000    // Buffer confirmação inversor (ms)
#define RELE_BUFFER_MS   60000    // Buffer antes de desligar (60s)

// ── Intervalos principais ──────────────────────────────────
#define INTERVALO_LEITURA_MS  1000
#define INTERVALO_CONTROLE_MS 5000

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] PROTEÇÃO CONTRA CURTO
// ═══════════════════════════════════════════════════════════
#define MAX_TENTATIVAS_RELIGAR  3
#define T_ESPERA_STATUS_INV  2500
#define T_STATUS_AUSENTE     1000
#define T_PAUSA_RELIGAR      2000
#define T_AGUARDA_QUEDA      1500
#define T_AGUARDA_STATUS     3000

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] HISTÓRICO
// ═══════════════════════════════════════════════════════════
#define HIST_DIAS        30
#define HIST_HORAS        24
#define HIST_KEY_VER       3      // v3: Ah em vez de Wh
#define HIST_SNAP_PONTOS  60      // Pontos gráfico corrente (60s)

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] PREVISÃO METEOROLÓGICA
// ═══════════════════════════════════════════════════════════
#define PREV_HORAS        48
#define PREV_URL_MAX     256
#define PREV_INTERVALO  3600000UL

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] RESTAURAÇÃO NÃO-BLOQUEANTE
// ═══════════════════════════════════════════════════════════
#define RESTORE_T_BAIXA       2000
#define RESTORE_T_INV_BOOT    5000
#define RESTORE_T_INV_STATUS  3000
#define RESTORE_T_ALTA         200

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] ANTECIPAÇÃO INTELIGENTE REDE ALTA
// ═══════════════════════════════════════════════════════════
#define T_ANTECIP_ESTABILIZA   60000UL
#define T_ANTECIP_REAVAL      300000UL
#define T_ANTECIP_COOLDOWN    900000UL
#define MARGEM_ANTECIP_MIN        30    // minutos

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] IA DE PADRÕES DE CONSUMO
// ═══════════════════════════════════════════════════════════
#define IA_MAX_PADROES         50
#define IA_BUFFER_SNAP        120    // 120s @ 1s/amostra
#define IA_JANELA_2MIN        120
#define IA_JANELA_5MIN        300
#define IA_JANELA_10MIN       600
#define IA_TOLERANCIA_BASE    1.0f
#define IA_PICO_MINIMO        1.5f
#define IA_BASELINE_MINIMO    0.3f
#define IA_TEMPO_APOS_DESLIGAR  600000UL
#define IA_TEMPO_APOS_LIGAR     300000UL
#define IA_INTERVALO_DECISAO     30000UL
#define IA_KEY_VER                  1
#define IA_NS_PADROES        "rrfs_ia"
#define IA_NS_CONFIG         "rrfs_iac"

// ═══════════════════════════════════════════════════════════
// [SIS-IA] CONSTANTES ATUALIZADAS
// ═══════════════════════════════════════════════════════════
#define IA_MIN_DADOS_DECISAO      30     // mínimo amostras para decidir
#define IA_INTERVALO_RECOMEND  60000UL   // 1min entre recomendações
#define IA_INTERVALO_ACAO     300000UL   // 5min entre ações (lamp→rede)
#define IA_INTERVALO_RELIGAR  120000UL   // 2min entre tentativas religar
#define IA_HISTERESE_RELIGAR      5.0f   // Ah acima do corte para religar

// ═══════════════════════════════════════════════════════════
// [SIS-CFG] REDE WIFI (MODO AP)
// ═══════════════════════════════════════════════════════════
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ═══════════════════════════════════════════════════════════
// [SIS-ENUM] PROTEÇÃO CONTRA CURTO
// ═══════════════════════════════════════════════════════════
enum EstadoCurto {
    CURTO_NORMAL = 0,
    CURTO_RELIGANDO,
    CURTO_TESTANDO_ALTA,
    CURTO_TESTANDO_BAIXA,
    CURTO_CONFIRMADO_ALTA,
    CURTO_CONFIRMADO_BAIXA,
    CURTO_AGUARDANDO_RESET
};
enum SubEstadoCurto {
    SUB_IDLE = 0,
    SUB_INVERSOR_DESLIGADO,
    SUB_AGUARDANDO_QUEDA,
    SUB_AGUARDANDO_STATUS,
    SUB_INVERSOR_OK_ALTA_OFF,
    SUB_INVERSOR_OK_LIGAR_ALTA
};

// ═══════════════════════════════════════════════════════════
// [SIS-ENUM] RESTAURAÇÃO E ANTECIPAÇÃO
// ═══════════════════════════════════════════════════════════
enum EstadoRestauro {
    RESTORE_OCIOSO = 0,
    RESTORE_LIGANDO_BAIXA,
    RESTORE_AGUARDANDO_BAIXA,
    RESTORE_LIGANDO_INVERSOR,
    RESTORE_AGUARDANDO_INVERSOR_BOOT,
    RESTORE_CHECANDO_STATUS_INV,
    RESTORE_LIGANDO_ALTA,
    RESTORE_LIGANDO_LAMP_INT,
    RESTORE_CONCLUIDO
};
enum EstadoAntecip {
    ANTECIP_IDLE = 0,
    ANTECIP_TESTANDO,
    ANTECIP_MANTIDO
};

// ═══════════════════════════════════════════════════════════
// [SIS-ENUM] IA
// ═══════════════════════════════════════════════════════════
enum TipoPadrao {
    PADRAO_BASELINE = 0,
    PADRAO_TRANSITORIO,
    PADRAO_CICLICO,
    PADRAO_ALTO_LONGO
};

enum EstadoAprendizado {
    IA_OBSERVANDO = 0,
    IA_DETECTOU_PICO,
    IA_CONFIRMANDO_BASE,
    IA_PADRAO_NOVO
};

enum ModoIAOperacao {
    IA_MODO_MANUAL = 0,
    IA_MODO_IA
};

// NOVO - deve estar aqui, antes das structs
enum EstadoSequenciaIA {
    SEQ_OCIOSO = 0,
    SEQ_RECOMENDOU_LAMP_INT,
    SEQ_DESLIGOU_LAMP_EXT,
    SEQ_DESLIGOU_REDE_ALTA,
    SEQ_DESLIGOU_REDE_BAIXA,
    SEQ_RELIGANDO
};

// ═══════════════════════════════════════════════════════════
// [SIS-STR] CONFIGURAÇÕES
// ═══════════════════════════════════════════════════════════

// ── Bateria ────────────────────────────────────────────────
// Energia agora em Ah (corrente × tempo), não Wh
struct ConfigBateria {
    float tensaoNominal;    // V nominal do banco (ex: 12.8V, 25.6V, 51.2V)
    float capacidadeAh;     // Ah total (ex: 100Ah)
    float descargaSegura;   // % DOD máximo (ex: 80%)
    float ahDisponivel;     // Ah disponíveis = capacidade × descarga% (calculado)
    float ahAtual;          // Ah atuais (coulomb counting em corrente)
    float percentualAtual;  // % atual
    bool  calibrada;
};

// ── Cortes de energia ──────────────────────────────────────
struct ConfigRedes {
    float corteBaixa;       // % para desligar rede baixa + inversor
    float corteAlta;        // % para desligar rede alta
    float corteLampExt;     // % para desligar lâmpada externa
    float corteLampInt;     // % para desligar lâmpada interna
};

// ── Local (previsão) ───────────────────────────────────────
// kWp substituído por amperagem máxima do controlador de carga
struct ConfigLocal {
    double  latitude;
    double  longitude;
    float   ampControlador; // A máximos do controlador de carga (ex: 30A, 60A)
    float   eficiencia;     // Eficiência do sistema (0.0-1.0)
    bool    configurado;
    float   fatorCalib;     // Fator de correção aprendido
    uint8_t amostrasCalib;
};

// ═══════════════════════════════════════════════════════════
// [SIS-STR] SENSORES E ESTADO
// ═══════════════════════════════════════════════════════════

struct CalibSensor {
    int   offsetMV;
    float sensMvA;
    bool  invertido;
    bool  calibrado;
};

struct BufferMediaMovel {
    float   amostras[BUFFER_CORRENTE];
    uint8_t indice;
    uint8_t total;
    bool    cheio;
};

// ── Leituras atuais ────────────────────────────────────────
// REMOVIDO: potenciaPainel/Casa/Bateria em Wh (agora calculado inline)
// ADICIONADO: kW calculado com tensão nominal (apenas informativo)
struct DadosSensores {
    float correntePainel;   // A real (positivo = gerando)
    float correnteCasa;     // A real (positivo = consumindo)
    float correnteSistema;  // A consumo próprio ESP32/controlador
    float correnteBateria;  // A calculado (painel - casa - sistema)
    // kW informativos (calculados com tensão NOMINAL, não estimada)
    float kWPainel;         // correntePainel × tensaoNominal / 1000
    float kWCasa;           // correnteCasa × tensaoNominal / 1000
    float kWSistema;        // correnteSistema × tensaoNominal / 1000
    float kWBateria;        // correnteBateria × tensaoNominal / 1000
    // Sinais brutos ADC
    int   sinalPainel_mV;
    int   sinalCasa_mV;
    int   sinalInversor_mV;
    bool  inversoreAtivo;
};

// ── Estado dos relés ───────────────────────────────────────
struct EstadoReles {
    bool redeBaixa;
    bool redesAlta;
    bool lampExt;
    bool lampInt;
    bool inversor;
};

// ── Timers buffer desligamento ─────────────────────────────
struct TimersBuffer {
    unsigned long inicioBaixa;
    unsigned long inicioAlta;
    unsigned long inicioLampExt;
    unsigned long inicioLampInt;
    bool pendenteBaixa;
    bool pendenteAlta;
    bool pendenteLampExt;
    bool pendenteLampInt;
    // Buffer estado inversor
    unsigned long inversorUltimoHigh;
    bool          inversorEstadoReal;
};

// ── Estimativa de tempo restante ───────────────────────────
// Agora em Ah/h (corrente) em vez de Wh/h
struct EstimativaTempo {
    bool  valido;
    int   horas;
    int   minutos;
    bool  carregando;
    float taxaA;    // A/h atual (positivo=carregando)
};

// ── Proteção contra curto ──────────────────────────────────
struct GerenciadorCurto {
    EstadoCurto    estado;
    SubEstadoCurto subEstado;
    unsigned long  tInicio;
    bool           redeAltaCurto;
    bool           redeBaixaCurto;
    bool           monitorando;
    uint8_t        tentativasReligar;
};

// ═══════════════════════════════════════════════════════════
// [SIS-STR] HISTÓRICO
// ═══════════════════════════════════════════════════════════

// ── Registro por hora (agora em Ah, não Wh) ───────────────
struct RegistroHora {
    float painelAh;     // Ah gerado pelo painel nesta hora
    float casaAh;       // Ah consumido pela casa nesta hora
    float batCargAh;    // Ah carregado na bateria
    float batDescAh;    // Ah descarregado da bateria
};

struct RegistroDia {
    uint16_t ano;
    uint8_t  mes;
    uint8_t  dia;
    bool     valido;
    RegistroHora horas[HIST_HORAS];
    float totalPainelAh;
    float totalCasaAh;
    float totalBatCargAh;
    float totalBatDescAh;
};

// ── Contadores globais (nunca zerados) ─────────────────────
struct ContadoresGlobais {
    float painelAh;     // Total gerado desde sempre
    float casaAh;       // Total consumido desde sempre
    float batCargAh;
    float batDescAh;
};

// ── Acumuladores hora atual ────────────────────────────────
struct AcumHora {
    float painelAh;
    float casaAh;
    float batCargAh;
    float batDescAh;
    uint8_t  horaAtual;
    uint8_t  diaAtual;
    uint8_t  mesAtual;
    uint16_t anoAtual;
    unsigned long ultimoAcum;
};

// ═══════════════════════════════════════════════════════════
// [SIS-STR] PREVISÃO METEOROLÓGICA
// ═══════════════════════════════════════════════════════════

// ── Previsão agora em corrente (A), não Wh ────────────────
struct PrevisaoHora {
    uint8_t  hora;
    uint8_t  dia;
    uint8_t  mes;
    uint16_t ano;
    float    radiacao;          // W/m²
    float    radiacaoDifusa;    // W/m²
    float    coberturaNuvens;   // %
    float    temperatura;       // °C
    float    correnteEstimada;  // A estimado de geração (era geracaoEstimada em Wh)
    bool     valido;
};

struct ResultadoPrev {
    PrevisaoHora horas[PREV_HORAS];
    uint8_t      totalHoras;
    float        correnteHojeRestante; // A médio restante hoje
    float        correnteAmanha;       // A médio estimado amanhã
    unsigned long ultimaAtualizacao;
    bool         valido;
    char         erro[64];
};

// ── Decisões automáticas ───────────────────────────────────
struct DecisaoSistema {
    bool  desligarRedeAltaNoite;
    bool  desligarLampExtNoite;
    bool  desligarLampIntDia;
    float horaDesligarRedeAlta;
    float horaLigarRedeAlta;
    bool  alertaEnergiaInsuficiente;
    char  motivo[128];
};

// ═══════════════════════════════════════════════════════════
// [SIS-STR] RESTAURAÇÃO E ANTECIPAÇÃO
// ═══════════════════════════════════════════════════════════
struct RestauroEnergia {
    EstadoRestauro estado;
    unsigned long  tInicio;
};

struct GerenciadorAntecip {
    EstadoAntecip estado;
    unsigned long tInicio;
    unsigned long tUltimaAval;
    unsigned long tCooldown;
};

// ═══════════════════════════════════════════════════════════
// [SIS-STR] IA DE PADRÕES
// ═══════════════════════════════════════════════════════════

struct PadraoConsumo {
    TipoPadrao tipo;
    float      correnteMedia;
    float      correntePico;
    float      correnteMin;
    float      duracaoMedia;
    float      duracaoMin;
    float      duracaoMax;
    float      energiaAh;
    uint32_t   ocorrencias;
    uint8_t    horaInicio;
    bool       valido;
};

struct BufferSnapshot {
    float    amostras[IA_BUFFER_SNAP];
    uint8_t  indice;
    uint8_t  total;
    bool     cheio;
};

struct EstadoAnalise {
    EstadoAprendizado estado;
    float    baselineAtual;
    float    baselineConfianca;
    float    picoAtual;
    float    picoDuracao;
    unsigned long tInicioPico;
    unsigned long tInicioBase;
    bool     emPico;
    float    correnteAnterior;
    uint8_t  janelasEstavel;
};

struct DecisaoIA {
    bool  desligarLampExt;
    bool  desligarRedeAlta;
    bool  desligarRedeBaixa;
    bool  desligarLampInt;
    bool  ligarLampExt;
    bool  ligarRedeAlta;
    bool  ligarRedeBaixa;
    float ahEstimadoNecessario;
    float ahDisponivel;
    float tempoAteAmanhecer;
    char  motivo[128];
};

// ── DEVE VIR ANTES de GerenciadorIA ───────────────────────
struct TimersSequenciaIA {
    unsigned long tUltimaRecomend;
    unsigned long tUltimaAcao;
    unsigned long tDesligouLampExt;
    unsigned long tDesligouRedeAlta;
    unsigned long tDesligouRedeBaixa;
    unsigned long tDesligouInversor;
    unsigned long tLigouInversor;
    unsigned long tLigouRedeBaixa;
    unsigned long tLigouRedeAlta;
    unsigned long tLigouLampExt;
};

// ── DEVE VIR ANTES de GerenciadorIA ───────────────────────
struct RecomendacaoIA {
    bool          ativa;
    char          texto[128];
    char          detalhe[128];
    uint8_t       nivel;
    unsigned long tCriado;
};

// ── Config IA (sem campo 'ativo') ─────────────────────────
struct ConfigIA {
    ModoIAOperacao modo;
    float          reservaEmergencia;
    bool           configurado;
};

// ── GerenciadorIA - usa todas as structs acima ────────────
struct GerenciadorIA {
    PadraoConsumo       padroes[IA_MAX_PADROES];
    uint8_t             totalPadroes;
    BufferSnapshot      snapshot;
    EstadoAnalise       analise;
    DecisaoIA           decisao;
    TimersSequenciaIA   timersSeq;
    RecomendacaoIA      recomendacao;
    ConfigIA            config;
    EstadoSequenciaIA   sequencia;
    unsigned long       ultimaDecisao;
    unsigned long       ultimoSalvamento;
    uint8_t             indicePadraoMaisAntigo;
    float               saldoAhAtual;
    float               ahNecAmanhecerAtual;
    bool                usandoFallbackManual;
};

// ═══════════════════════════════════════════════════════════
// [SIS-VAR] INSTÂNCIAS GLOBAIS - SISTEMA GERAL
// ═══════════════════════════════════════════════════════════
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP = false;
bool logado = false;

// ── LED e Botão ────────────────────────────────────────────
unsigned long ultimoBlink        = 0;
bool          ledState           = false;
bool          blinkAtivo         = true;
unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

// ── Logs ───────────────────────────────────────────────────
char    logs[MAX_LOGS][LOG_MAX_LEN];
uint8_t totalLogs = 0;
uint8_t logIndex  = 0;
char    tempBuf[200];

// ═══════════════════════════════════════════════════════════
// [SIS-VAR] CONFIGURAÇÕES E ESTADO
// ═══════════════════════════════════════════════════════════
ConfigBateria bateria   = {12.8f, 100.0f, 80.0f, 80.0f, 0.0f, 0.0f, false};
ConfigRedes   redes     = {10.0f, 30.0f, 50.0f, 0.0f};
ConfigLocal   configLocal = {0, 0, 30.0f, 0.80f, false, 1.0f, 0};

// ── Sensores e calibração ──────────────────────────────────
DadosSensores    sensores  = {0};
CalibSensor      calibPainel;
CalibSensor      calibCasa;
bool             autoCalibPainelAtivo = true;   // NOVO
BufferMediaMovel bufPainel  = {0};
BufferMediaMovel bufCasa    = {0};
BufferMediaMovel bufSistema = {0};

// ── Estado dos relés ───────────────────────────────────────
EstadoReles  estados = {true, true, true, true, false};
TimersBuffer timers  = {0};

// ── Proteção curto ─────────────────────────────────────────
GerenciadorCurto curto = {CURTO_NORMAL, SUB_IDLE, 0, false, false, false, 0};

// ── Intervalos ─────────────────────────────────────────────
unsigned long ultimaLeitura  = 0;
unsigned long ultimoControle = 0;
unsigned long ultimoContagem = 0;

// ── Restauração e antecipação ──────────────────────────────
RestauroEnergia   restauro    = {RESTORE_OCIOSO, 0};
GerenciadorAntecip antecipAlta = {ANTECIP_IDLE, 0, 0, 0};

// ═══════════════════════════════════════════════════════════
// [SIS-VAR] HISTÓRICO
// ═══════════════════════════════════════════════════════════
RegistroDia histDiaAtual;
RegistroDia histDiaTemp;
uint16_t    histIndiceAtual = 0;
uint16_t    histTotalDias   = 0;

ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora acum = {0, 0, 0, 0, 255, 0, 0, 0, 0};

// ── Histórico de corrente para gráfico (últimos 60s) ───────
float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx  = 0;
bool    histSnapFull = false;
unsigned long ultimoHistSnap = 0;

// ═══════════════════════════════════════════════════════════
// [SIS-VAR] PREVISÃO E DECISÕES
// ═══════════════════════════════════════════════════════════
ResultadoPrev  previsao = {0};
DecisaoSistema decisao  = {false, false, false, 0, 0, false, ""};
unsigned long  ultimaPrev   = 0;
bool           buscandoPrev = false;
bool           ehDia        = false;

// ═══════════════════════════════════════════════════════════
// [SIS-VAR] WIFI
// ═══════════════════════════════════════════════════════════
String gWifiSSID = "";
String gWifiPASS = "";
String gWifiIP   = "";
String gWifiGW   = "";
String gWifiMASK = "";
String gWifiDNS  = "";

// ── IA ─────────────────────────────────────────────────────
GerenciadorIA ia;

// ═══════════════════════════════════════════════════════════
// [SIS-PROTO] PROTÓTIPOS DE FUNÇÕES
// ═══════════════════════════════════════════════════════════

// Utilitários
EstimativaTempo calcularTempoRestante();
String tempoAteCorte(float pctCorte, float taxaAh);
String fmtCorrente(float a);
String fmtKW(float kw);

// Logs e LED
void adicionarLog(const char* msg);
void logRAM();
void setupLED();
void atualizarLED();
void ledBlink();
void ledFixo(bool v);

// Botão
void setupBotao();
void verificarReset();

// Relés
void setupReles();
void setRele(uint8_t pin, bool ligar);
void ligarRedeBaixa();   void desligarRedeBaixa();
void ligarRedeAlta();    void desligarRedeAlta();
void ligarLampExt();     void desligarLampExt();
void ligarLampInt();     void desligarLampInt();
void ligarInversor();    void desligarInversor();

// Sensores ADC
int   lerADC(uint8_t pin);
int   adcParaMv(int adcVal);
float adicionarAmostra(BufferMediaMovel& buf, float nova);
float lerCorrentePainel();
float lerCorrenteCasa();
bool  lerStatusInversor();
void  lerSensores();

// Bateria (Coulomb Counting em Ah)
void  calcularBateria();
void  verificarCalibracaoAutomatica();
void  atualizarAhBateria();

// Histórico
void calcularTotaisDia(RegistroDia& dia);
void salvarContadores();
void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void salvarHistorico();
void carregarHistorico();
void acumularEnergia();

// Previsão
void  salvarConfigLocal();
void  carregarConfigLocal();
void  calibrarPrevisao();
float calcularCorrenteEstimadaHora(float radWm2, float radDif,
                                   float nuvens, float tempC);
void  buscarPrevisaoMeteo();
void  tomarDecisoes();
void  verificarReligamentoAuto();
float consumoMedioParaHora(uint8_t horaAlvo, bool* temDados = nullptr);
float consumoMedioParaHoraCache(uint8_t horaAlvo, bool* temDados = nullptr);
float referenciaConsumoAntecip();
float horasAteGeracaoSuperar(float refA);
void  avaliarAntecipacaoRedeAlta();

// Controle de energia
void controlarEnergia();
void processarRestauroEnergia();

// Proteção curto
void   monitorarInversor();
void   resetCurtoAlta();
void   resetCurtoBaixa();
String htmlStatusCurto();
void   _desligarInversorComBuffer();
void   _ligarInversorComBuffer();

// IA
void iaInicializar();
void iaSalvarConfig();
void iaCarregarConfig();
void iaSalvarPadroes();
void iaCarregarPadroes();
void iaAdicionarSnapshot(float corrente);
float iaMediaJanela(uint16_t segundos);
float iaDesvioPadraoJanela(uint16_t segundos);
int  iaEncontrarPadraoSimilar(TipoPadrao tipo, float corrente, float duracao);
void iaRegistrarPadrao(TipoPadrao tipo, float corrMedia, float corrPico,
                       float corrMin, float duracao, float energiaAh);
void iaAnalisar();
void iaTomarDecisao();
void iaExecutarDecisoes();
bool iaPodeDesligar(const char* carga);
bool iaPodeLigar(const char* carga);
float iaEstimarAhAteAmanhecer();
bool  iaCorrenteETransitoria();
float iaAhRestantePico();
void  iaLoop();

// Preferences
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
void salvarConfigRedes();
void carregarConfigRedes();
void carregarCalibSensores();

// WiFi
void iniciarAP();
bool iniciarWiFi(const String& ssid, const String& pass,
                 const String& ip,   const String& gw,
                 const String& mask, const String& dns);
void tentarReconectarWiFi();
void sincronizarNTP();

// Web
void configurarRotas();
String gerarPagina(const String& titulo, const String& conteudo, bool comNav = true);
String barraProgresso(float pct, const String& label = "");
String statusCard(const String& icone, const String& label,
                  const String& valor, const String& sub,
                  const String& tipo = "");

// Handlers web
void handleRoot();
void handleDados();
void handleBateria();
void handleRedes();
void handleSensores();
void handleConfig();
void handleHistorico();
void handlePrevisao();
void handleLogs();
void handleSetupLogin();
void handleSetupWifi();
void handleLogin();
void handleLogout();
void handleNotFound();
void handleResetCurto();
void handleNTPSync();
void handleReiniciar();
void handleOTAUpload();
void handleOTAResult();
void handleIA();

// ═══════════════════════════════════════════════════════════
// [SIS-UTIL] UTILITÁRIOS E HELPERS
// ═══════════════════════════════════════════════════════════

// ── Tempo restante de bateria (agora em Ah/h) ─────────────
EstimativaTempo calcularTempoRestante() {
    EstimativaTempo est = {false, 0, 0, false, 0};
    if (!bateria.calibrada) return est;
    if (bateria.ahDisponivel <= 0) return est;

    float taxaA = sensores.correnteBateria;
    if (fabsf(taxaA) < 0.3f) return est;

    est.taxaA      = taxaA;
    est.valido     = true;
    est.carregando = (taxaA > 0);

    float horasRestantes;
    if (est.carregando) {
        float faltando = bateria.ahDisponivel - bateria.ahAtual;
        if (faltando <= 0) { est.horas = 0; est.minutos = 0; return est; }
        horasRestantes = faltando / taxaA;
    } else {
        if (bateria.ahAtual <= 0) { est.horas = 0; est.minutos = 0; return est; }
        horasRestantes = bateria.ahAtual / fabsf(taxaA);
    }

    if (horasRestantes > 999) horasRestantes = 999;
    est.horas   = (int)horasRestantes;
    est.minutos = (int)((horasRestantes - est.horas) * 60);
    return est;
}

// ── Tempo até percentual de corte (com horário estimado) ──
String tempoAteCorte(float pctCorte, float taxaA) {
    if (!bateria.calibrada) return "--";
    if (taxaA >= 0) return "--";
    if (bateria.ahDisponivel <= 0) return "--";

    float ahCorte    = bateria.ahDisponivel * (pctCorte / 100.0f);
    float ahAteCorte = bateria.ahAtual - ahCorte;
    if (ahAteCorte <= 0) return "Agora";

    float horas = ahAteCorte / fabsf(taxaA);
    if (horas > 999) return ">999h";

    int h = (int)horas;
    int m = (int)((horas - h) * 60);

    char buf[20];
    if (h == 0) snprintf(buf, sizeof(buf), "%dm", m);
    else        snprintf(buf, sizeof(buf), "%dh%02dm", h, m);

    String resultado = String(buf);
    struct tm ti;
    if (getLocalTime(&ti)) {
        time_t agora = mktime(&ti);
        time_t alvo  = agora + (time_t)(horas * 3600.0f);
        struct tm tAlvo;
        localtime_r(&alvo, &tAlvo);
        char bufH[8];
        snprintf(bufH, sizeof(bufH), "%02d:%02d", tAlvo.tm_hour, tAlvo.tm_min);
        resultado += " (~" + String(bufH) + ")";
    }
    return resultado;
}

// ── Formatação ─────────────────────────────────────────────
String fmtCorrente(float a) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fA", a);
    return String(buf);
}

String fmtKW(float kw) {
    char buf[16];
    if (fabsf(kw) >= 1.0f)
        snprintf(buf, sizeof(buf), "%.2fkW", kw);
    else
        snprintf(buf, sizeof(buf), "%.0fW", kw * 1000.0f);
    return String(buf);
}

// ═══════════════════════════════════════════════════════════
// [SIS-LOG] SISTEMA DE LOGS
// ═══════════════════════════════════════════════════════════
void adicionarLog(const char* msg) {
    struct tm ti;
    if (getLocalTime(&ti)) {
        snprintf(logs[logIndex], LOG_MAX_LEN, "[%02d:%02d:%02d] %s",
                 ti.tm_hour, ti.tm_min, ti.tm_sec, msg);
    } else {
        unsigned long s = millis() / 1000;
        snprintf(logs[logIndex], LOG_MAX_LEN, "[%02lu:%02lu:%02lu] %s",
                 s/3600, (s%3600)/60, s%60, msg);
    }
    logIndex = (logIndex + 1) % MAX_LOGS;
    if (totalLogs < MAX_LOGS) totalLogs++;
}

void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf),
             "RAM: %.1fKB livres", esp_get_free_heap_size() / 1024.0f);
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [SIS-LED] LED INDICADOR
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
// [SIS-BTN] BOTÃO DE RESET
// ═══════════════════════════════════════════════════════════
void setupBotao() { pinMode(BOTAO_BOOT, INPUT_PULLUP); }

void verificarReset() {
    bool pressionado = (digitalRead(BOTAO_BOOT) == LOW);
    if (pressionado && !botaoSegurando) {
        botaoSegurando = true; botaoPressionadoEm = millis();
    }
    if (!pressionado) botaoSegurando = false;

    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET TOTAL por botao BOOT");
        for (int i = 0; i < 15; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(80);
            digitalWrite(LED_AZUL, LOW);  delay(80);
        }
        nvs_flash_erase();
        nvs_flash_init();
        digitalWrite(LED_AZUL, HIGH); delay(500);
        digitalWrite(LED_AZUL, LOW);  delay(300);
        ESP.restart();
    }
}

// ═══════════════════════════════════════════════════════════
// [SIS-RELE] CONTROLE DE RELÉS
// ═══════════════════════════════════════════════════════════
void setupReles() {
    pinMode(PIN_RELE_INV_ALTA,     OUTPUT); digitalWrite(PIN_RELE_INV_ALTA,     LOW);
    pinMode(PIN_RELE_REDE_BAIXA,   OUTPUT); digitalWrite(PIN_RELE_REDE_BAIXA,   LOW);
    pinMode(PIN_RELE_REDE_ALTA,    OUTPUT); digitalWrite(PIN_RELE_REDE_ALTA,    LOW);
    pinMode(PIN_RELE_LAMP_EXTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_EXTERNA, LOW);
    pinMode(PIN_RELE_LAMP_INTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_INTERNA, LOW);
    pinMode(PIN_STATUS_INV_ALTA, INPUT);
    pinMode(PIN_BKP_BATERIA,     INPUT);
    estados = {false, false, false, false, false};
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
    adicionarLog("Reles: todos OFF");
}

void setRele(uint8_t pin, bool ligar) { digitalWrite(pin, ligar ? HIGH : LOW); }

void ligarRedeBaixa()    { setRele(PIN_RELE_REDE_BAIXA,   true);  estados.redeBaixa = true;  }
void desligarRedeBaixa() { setRele(PIN_RELE_REDE_BAIXA,   false); estados.redeBaixa = false; }
void ligarRedeAlta()     { setRele(PIN_RELE_REDE_ALTA,    true);  estados.redesAlta = true;  }
void desligarRedeAlta()  { setRele(PIN_RELE_REDE_ALTA,    false); estados.redesAlta = false; }
void ligarLampExt()      { setRele(PIN_RELE_LAMP_EXTERNA, true);  estados.lampExt   = true;  }
void desligarLampExt()   { setRele(PIN_RELE_LAMP_EXTERNA, false); estados.lampExt   = false; }
void ligarLampInt()      { setRele(PIN_RELE_LAMP_INTERNA, true);  estados.lampInt   = true;  }
void desligarLampInt()   { setRele(PIN_RELE_LAMP_INTERNA, false); estados.lampInt   = false; }
void ligarInversor()     { setRele(PIN_RELE_INV_ALTA,     true);  estados.inversor  = true;  }
void desligarInversor()  { setRele(PIN_RELE_INV_ALTA,     false); estados.inversor  = false; }

// ═══════════════════════════════════════════════════════════
// [SIS-ADC] LEITURA DE SENSORES
// ═══════════════════════════════════════════════════════════
int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        soma += analogRead(pin);
        delayMicroseconds(100);
    }
    return (int)(soma / ADC_SAMPLES);
}

int adcParaMv(int adcVal) {
    return (int)((long)adcVal * ADC_VREF / ADC_BITS);
}

float adicionarAmostra(BufferMediaMovel& buf, float novaAmostra) {
    buf.amostras[buf.indice] = novaAmostra;
    buf.indice = (buf.indice + 1) % BUFFER_CORRENTE;
    if (buf.total < BUFFER_CORRENTE) buf.total++;
    if (buf.total == BUFFER_CORRENTE) buf.cheio = true;
    float soma = 0;
    for (uint8_t i = 0; i < buf.total; i++) soma += buf.amostras[i];
    return soma / buf.total;
}

// ── Lê corrente do painel com auto-calibração de offset ───
float lerCorrentePainel() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_PAINEL));
    sensores.sinalPainel_mV = mv;

    // Auto-calibração do offset (noturna e diurna)
    static int    offsetAprendido    = PAINEL_OFFSET_MV;
    static bool   offsetCalibrado    = false;
    static long   somaOffset         = 0;
    static int    amostrasOffset     = 0;
    static unsigned long tUltimoZero = 0;
    static int    mvAnterior         = 0;
    static int    amostrasEstaveis   = 0;
    static long   somaOffsetDia      = 0;
    static int    amostrasOffsetDia  = 0;
    static unsigned long tUltimoZeroDia = 0;

    struct tm tiOff;
    bool podeNoite = false, podeDia = false;
    if (getLocalTime(&tiOff)) {
        int hora = tiOff.tm_hour;
        if (hora >= 22 || hora <= 4) {
            podeNoite = true;
        } else {
            int desvio = mv - offsetAprendido;
            if (desvio > 0 && desvio < 80 && abs(mv - mvAnterior) <= 5) {
                amostrasEstaveis++;
                if (amostrasEstaveis >= 30) podeDia = true;
            } else {
                amostrasEstaveis = 0;
            }
        }
    }
    mvAnterior = mv;

    // ── Aprendizado noturno (só roda se autoCalib estiver ATIVO) ──
    if (autoCalibPainelAtivo && podeNoite) {
        if (!tUltimoZero) tUltimoZero = millis();
        somaOffset += mv; amostrasOffset++;
        if (millis() - tUltimoZero >= 30000 && amostrasOffset >= 20) {
            int novo = (int)(somaOffset / amostrasOffset);
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
            somaOffset = 0; amostrasOffset = 0; tUltimoZero = 0;
        }
    } else if (!podeDia) {
        tUltimoZero = 0; somaOffset = 0; amostrasOffset = 0;
    }

    // ── Aprendizado diurno (só roda se autoCalib estiver ATIVO) ──
    if (autoCalibPainelAtivo && podeDia && !podeNoite) {
        if (!tUltimoZeroDia) tUltimoZeroDia = millis();
        somaOffsetDia += mv; amostrasOffsetDia++;
        if (millis() - tUltimoZeroDia >= 60000 && amostrasOffsetDia >= 40) {
            int novo = (int)(somaOffsetDia / amostrasOffsetDia);
            if (abs(novo - PAINEL_OFFSET_MV) <= 120) {
                if (!offsetCalibrado || abs(novo - mv) < abs(offsetAprendido - mv)) {
                    offsetAprendido = novo;
                    offsetCalibrado = true;
                    if (abs(novo - calibPainel.offsetMV) > 3) {
                        calibPainel.offsetMV = novo;
                        snprintf(tempBuf, sizeof(tempBuf),
                                 "Painel offset diurno: %dmV", novo);
                        adicionarLog(tempBuf);
                    }
                }
            }
            somaOffsetDia = 0; amostrasOffsetDia = 0;
            tUltimoZeroDia = 0; amostrasEstaveis = 0;
        }
    } else if (!podeDia) {
        somaOffsetDia = 0; amostrasOffsetDia = 0; tUltimoZeroDia = 0;
    }

    // ── Escolhe qual offset usar ────────────────────────────
    // Se autoCalib estiver DESATIVADO, sempre usa o offset calibrado
    // manualmente (calibPainel.offsetMV), ignorando o aprendizado.
    int offsetUsado = (autoCalibPainelAtivo && offsetCalibrado)
    ? offsetAprendido : calibPainel.offsetMV;

    // Corrente: cabos invertidos → sinal desce = painel gerando
    float corrBruta = -(float)(mv - offsetUsado) / calibPainel.sensMvA;

    // Zona morta dinâmica
    float zonaMorta = offsetCalibrado ?
    (podeNoite ? 0.25f : 0.30f) : 0.50f;

    float correnteSistemaBruta = 0.0f;
    if (corrBruta < 0.0f) {
        float mag = fabsf(corrBruta);
        if (mag >= zonaMorta) {
            correnteSistemaBruta = mag - (zonaMorta * 0.5f);
            if (correnteSistemaBruta < 0) correnteSistemaBruta = 0;
        }
        corrBruta = 0.0f;
    }
    if (corrBruta > 0.0f && corrBruta < 0.15f) corrBruta = 0.0f;

    float corrMedia    = adicionarAmostra(bufPainel, corrBruta);
    if (corrMedia < 0.15f) corrMedia = 0.0f;

    float sistemaMedia = adicionarAmostra(bufSistema, correnteSistemaBruta);
    if (sistemaMedia < (zonaMorta * 0.3f)) sistemaMedia = 0.0f;

    sensores.correnteSistema = (corrMedia < 0.15f) ? sistemaMedia : 0.0f;
    return corrMedia;
}

// ── Lê corrente da casa ────────────────────────────────────
float lerCorrenteCasa() {
    int mv = adcParaMv(lerADC(PIN_SENSOR_CASA));
    sensores.sinalCasa_mV = mv;

    if (mv <= calibCasa.offsetMV) {
        adicionarAmostra(bufCasa, 0.0f);
        return 0.0f;
    }

    float corrBruta = (float)(mv - calibCasa.offsetMV) / calibCasa.sensMvA;
    if (corrBruta < 0) corrBruta = 0;
    if (corrBruta < FILTRO_RUIDO_A) corrBruta = 0;

    float media = adicionarAmostra(bufCasa, corrBruta);
    return (media < FILTRO_RUIDO_A) ? 0.0f : media;
}

// ── Lê status do inversor ──────────────────────────────────
bool lerStatusInversor() {
    int mv = adcParaMv(lerADC(PIN_STATUS_INV_ALTA));
    sensores.sinalInversor_mV = mv;

    if (!estados.inversor) {
        timers.inversorEstadoReal = false;
        timers.inversorUltimoHigh = 0;
        return false;
    }

    bool sinalHigh = (mv >= INV_THRESHOLD_MV);
    if (sinalHigh) {
        timers.inversorUltimoHigh = millis();
        timers.inversorEstadoReal = true;
    } else if (timers.inversorEstadoReal &&
        millis() - timers.inversorUltimoHigh >= INV_BUFFER_MS) {
        timers.inversorEstadoReal = false;
        }
        return timers.inversorEstadoReal;
}

// ── Leitura completa de todos os sensores ──────────────────
void lerSensores() {
    sensores.correntePainel = lerCorrentePainel();
    sensores.correnteCasa   = lerCorrenteCasa();
    sensores.correnteBateria = sensores.correntePainel
    - sensores.correnteCasa
    - sensores.correnteSistema;

    // kW informativos usando tensão NOMINAL (sem estimativa complexa)
    // Calculados uma vez aqui, reutilizados em todo o sistema
    const float vNom = bateria.tensaoNominal;
    sensores.kWPainel   = (sensores.correntePainel  * vNom) / 1000.0f;
    sensores.kWCasa     = (sensores.correnteCasa    * vNom) / 1000.0f;
    sensores.kWSistema  = (sensores.correnteSistema * vNom) / 1000.0f;
    sensores.kWBateria  = (sensores.correnteBateria * vNom) / 1000.0f;

    sensores.inversoreAtivo = lerStatusInversor();
    atualizarAhBateria();
    verificarCalibracaoAutomatica();

    // Histórico de corrente para gráfico
    if (millis() - ultimoHistSnap >= 1000) {
        ultimoHistSnap = millis();
        histCorrPainel[histSnapIdx] = sensores.correntePainel;
        histCorrCasa  [histSnapIdx] = sensores.correnteCasa;
        histCorrBat   [histSnapIdx] = sensores.correnteBateria;
        histSnapIdx = (histSnapIdx + 1) % HIST_SNAP_PONTOS;
        if (histSnapIdx == 0) histSnapFull = true;
    }

    iaAnalisar();
}

// ═══════════════════════════════════════════════════════════
// [SIS-BAT] GERENCIAMENTO DE BATERIA (COULOMB COUNTING EM Ah)
// ═══════════════════════════════════════════════════════════

void calcularBateria() {
    bateria.ahDisponivel = bateria.capacidadeAh * (bateria.descargaSegura / 100.0f);
}

// ── Detecta bateria 100% (BMS cortou carga) ───────────────
void verificarCalibracaoAutomatica() {
    if (sensores.correntePainel > 0.5f &&
        fabsf(sensores.correnteBateria) <= 0.5f &&
        sensores.correnteCasa > 0.1f) {
        if (!bateria.calibrada) {
            bateria.ahAtual        = bateria.ahDisponivel;
            bateria.percentualAtual = 100.0f;
            bateria.calibrada      = true;
            prefs.begin("rrfs", false);
            prefs.putFloat("bat_ah_atual", bateria.ahAtual);
            prefs.putBool ("bat_calib",    true);
            prefs.end();
            adicionarLog("Bateria calibrada: 100% detectado");
        }
        }
}

// ── Coulomb counting em Ah (sem tensão estimada) ──────────
void atualizarAhBateria() {
    if (!bateria.calibrada) return;

    unsigned long agora  = millis();
    // deltaH em horas
    float deltaH = (float)(agora - ultimoContagem) / 3600000.0f;
    ultimoContagem = agora;

    // Integra corrente: positivo = carregando, negativo = descarregando
    bateria.ahAtual += sensores.correnteBateria * deltaH;

    if (bateria.ahAtual < 0) bateria.ahAtual = 0;
    if (bateria.ahAtual > bateria.ahDisponivel)
        bateria.ahAtual = bateria.ahDisponivel;

    if (bateria.ahDisponivel > 0)
        bateria.percentualAtual = (bateria.ahAtual / bateria.ahDisponivel) * 100.0f;
    else
        bateria.percentualAtual = 0;
}

// ═══════════════════════════════════════════════════════════
// [SIS-HIST] HISTÓRICO DE ENERGIA (em Ah)
// ═══════════════════════════════════════════════════════════

void calcularTotaisDia(RegistroDia& dia) {
    dia.totalPainelAh  = 0; dia.totalCasaAh    = 0;
    dia.totalBatCargAh = 0; dia.totalBatDescAh = 0;
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
             "Contadores: P=%.2fAh C=%.2fAh BC=%.2fAh BD=%.2fAh",
             contadores.painelAh, contadores.casaAh,
             contadores.batCargAh, contadores.batDescAh);
    adicionarLog(tempBuf);
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
        adicionarLog("Historico: resetado (versao nova v15)");
        return;
    }
    histIndiceAtual = prefs.getUShort("idx",   0);
    histTotalDias   = prefs.getUShort("total", 0);
    prefs.end();
    if (histTotalDias > 0 && !carregarDiaFlash(histIndiceAtual, histDiaAtual))
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
    snprintf(tempBuf, sizeof(tempBuf),
             "Historico: %d/%d dias | idx=%d",
             histTotalDias, HIST_DIAS, histIndiceAtual);
    adicionarLog(tempBuf);
}

// ── Acumula corrente por hora (Ah = A × h) ────────────────
void acumularEnergia() {
    struct tm ti;
    if (!getLocalTime(&ti)) return;

    unsigned long agora  = millis();
    uint8_t  horaAtual   = ti.tm_hour;
    uint8_t  diaAtual    = ti.tm_mday;
    uint8_t  mesAtual    = ti.tm_mon + 1;
    uint16_t anoAtual    = ti.tm_year + 1900;

    // Inicialização na primeira chamada
    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum = agora;
        acum.horaAtual  = horaAtual;
        acum.diaAtual   = diaAtual;
        acum.mesAtual   = mesAtual;
        acum.anoAtual   = anoAtual;

        bool diaExiste = false;
        if (histTotalDias > 0) {
            RegistroDia dTemp;
            if (carregarDiaFlash(histIndiceAtual, dTemp)) {
                if (dTemp.dia == diaAtual && dTemp.mes == mesAtual &&
                    dTemp.ano == anoAtual) {
                    histDiaAtual = dTemp;
                diaExiste = true;
                adicionarLog("Hist: dia atual recuperado");
                    }
            }
        }
        if (!diaExiste) {
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano = anoAtual; histDiaAtual.mes = mesAtual;
            histDiaAtual.dia = diaAtual; histDiaAtual.valido = true;
            if (histTotalDias == 0) {
                histIndiceAtual = 0; histTotalDias = 1;
            } else {
                RegistroDia dT;
                if (carregarDiaFlash(histIndiceAtual, dT)) {
                    if (dT.dia != diaAtual || dT.mes != mesAtual || dT.ano != anoAtual) {
                        histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
                        if (histTotalDias < HIST_DIAS) histTotalDias++;
                    }
                }
            }
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
            salvarIndiceHistorico();
        }
        return;
    }

    // deltaH em horas (limitado a 6min para evitar saltos)
    float deltaH = (float)(agora - acum.ultimoAcum) / 3600000.0f;
    acum.ultimoAcum = agora;
    if (deltaH > 0.1f) deltaH = 0.1f;

    // Acumula em Ah
    float ip = sensores.correntePainel;
    float ic = sensores.correnteCasa;
    float ib = sensores.correnteBateria;

    if (ip > 0.15f)  acum.painelAh   += ip        * deltaH;
    if (ic > 0.3f)   acum.casaAh     += ic        * deltaH;
    if (ib > 0.3f)   acum.batCargAh  += ib        * deltaH;
    if (ib < -0.3f)  acum.batDescAh  += fabsf(ib) * deltaH;

    bool mudouHora = (horaAtual != acum.horaAtual);
    bool mudouDia  = (diaAtual  != acum.diaAtual ||
    mesAtual  != acum.mesAtual ||
    anoAtual  != acum.anoAtual);

    if (mudouHora || mudouDia) {
        uint8_t horaIdx = acum.horaAtual % HIST_HORAS;
        histDiaAtual.horas[horaIdx].painelAh  += acum.painelAh;
        histDiaAtual.horas[horaIdx].casaAh    += acum.casaAh;
        histDiaAtual.horas[horaIdx].batCargAh += acum.batCargAh;
        histDiaAtual.horas[horaIdx].batDescAh += acum.batDescAh;
        calcularTotaisDia(histDiaAtual);

        contadores.painelAh  += acum.painelAh;
        contadores.casaAh    += acum.casaAh;
        contadores.batCargAh += acum.batCargAh;
        contadores.batDescAh += acum.batDescAh;

        snprintf(tempBuf, sizeof(tempBuf),
                 "Hist h%02d: P=%.2fAh C=%.2fAh BC=%.2fAh BD=%.2fAh",
                 acum.horaAtual, acum.painelAh, acum.casaAh,
                 acum.batCargAh, acum.batDescAh);
        adicionarLog(tempBuf);

        if (mudouDia) {
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
            histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
            if (histTotalDias < HIST_DIAS) histTotalDias++;
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano = anoAtual; histDiaAtual.mes = mesAtual;
            histDiaAtual.dia = diaAtual; histDiaAtual.valido = true;
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
            snprintf(tempBuf, sizeof(tempBuf),
                     "Hist: novo dia %02d/%02d/%04d",
                     diaAtual, mesAtual, anoAtual);
            adicionarLog(tempBuf);
        } else {
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
        }

        salvarIndiceHistorico();
        salvarContadores();

        acum.painelAh   = 0; acum.casaAh    = 0;
        acum.batCargAh  = 0; acum.batDescAh = 0;
        acum.horaAtual  = horaAtual;
        if (mudouDia) {
            acum.diaAtual = diaAtual;
            acum.mesAtual = mesAtual;
            acum.anoAtual = anoAtual;
        }
    }

    // Salva parcial a cada 5 minutos
    static unsigned long tSalvaParcial = 0;
    if (agora - tSalvaParcial >= 300000UL) {
        tSalvaParcial = agora;
        uint8_t hIdx = horaAtual % HIST_HORAS;
        RegistroDia diaTemp = histDiaAtual;
        diaTemp.horas[hIdx].painelAh  += acum.painelAh;
        diaTemp.horas[hIdx].casaAh    += acum.casaAh;
        diaTemp.horas[hIdx].batCargAh += acum.batCargAh;
        diaTemp.horas[hIdx].batDescAh += acum.batDescAh;
        calcularTotaisDia(diaTemp);
        salvarDiaFlash(histIndiceAtual, diaTemp);
        salvarContadores();
    }
}

// ═══════════════════════════════════════════════════════════
// [SIS-PREV] PREVISÃO METEOROLÓGICA (agora em corrente A)
// ═══════════════════════════════════════════════════════════

void salvarConfigLocal() {
    prefs.begin("rrfs_loc", false);
    prefs.putDouble("lat",     configLocal.latitude);
    prefs.putDouble("lon",     configLocal.longitude);
    prefs.putFloat ("amp",     configLocal.ampControlador);
    prefs.putFloat ("ef",      configLocal.eficiencia);
    prefs.putFloat ("fcalib",  configLocal.fatorCalib);
    prefs.putUChar ("ncalib",  configLocal.amostrasCalib);
    prefs.putBool  ("conf",    configLocal.configurado);
    prefs.end();
    adicionarLog("Config local salva");
}

void carregarConfigLocal() {
    prefs.begin("rrfs_loc", true);
    configLocal.latitude      = prefs.getDouble("lat",    0.0);
    configLocal.longitude     = prefs.getDouble("lon",    0.0);
    configLocal.ampControlador= prefs.getFloat ("amp",   30.0f);
    configLocal.eficiencia    = prefs.getFloat ("ef",    0.80f);
    configLocal.fatorCalib    = prefs.getFloat ("fcalib", 1.0f);
    configLocal.amostrasCalib = prefs.getUChar ("ncalib",    0);
    configLocal.configurado   = prefs.getBool  ("conf",  false);
    prefs.end();
    if (configLocal.configurado) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Local: lat=%.6f lon=%.6f amp=%.0fA ef=%.0f%% calib=%.3f(%d)",
                 configLocal.latitude, configLocal.longitude,
                 configLocal.ampControlador,
                 configLocal.eficiencia * 100.0f,
                 configLocal.fatorCalib, configLocal.amostrasCalib);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("Local: nao configurado");
    }
}

// ── Calibração automática da previsão (compara A previsto vs real) ──
void calibrarPrevisao() {
    if (!configLocal.configurado) return;
    if (!previsao.valido) return;
    if (sensores.correntePainel < 1.0f) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;
    uint8_t horaAtual = ti.tm_hour;

    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (ph.hora != horaAtual || ph.dia != ti.tm_mday ||
            ph.mes  != (uint8_t)(ti.tm_mon+1) ||
            ph.ano  != (uint16_t)(ti.tm_year+1900)) continue;
        if (ph.correnteEstimada < 0.5f) return;

        float fatorNovo = sensores.correntePainel / ph.correnteEstimada;
        if (fatorNovo < 0.1f) fatorNovo = 0.1f;
        if (fatorNovo > 3.0f) fatorNovo = 3.0f;

        if (configLocal.amostrasCalib < 50) configLocal.amostrasCalib++;
        float peso = 1.0f / (float)configLocal.amostrasCalib;
        configLocal.fatorCalib = configLocal.fatorCalib * (1.0f - peso)
                               + fatorNovo * peso;

        if (configLocal.amostrasCalib % 10 == 0) {
            salvarConfigLocal();
            snprintf(tempBuf, sizeof(tempBuf),
                     "Calib previsao: fator=%.3f amostras=%d",
                     configLocal.fatorCalib, configLocal.amostrasCalib);
            adicionarLog(tempBuf);
        }
        break;
    }
}

// ── Calcula corrente estimada de geração por hora ──────────
// Substitui calcularGeracaoHora() que retornava Wh
// Agora retorna A estimados baseado na amperagem do controlador
float calcularCorrenteEstimadaHora(float radWm2, float radDif,
                                    float nuvens, float tempC) {
    if (!configLocal.configurado) return 0;
    if (configLocal.ampControlador <= 0) return 0;

    // Radiação total (direta + difusa parcial)
    float radTotal = radWm2 + radDif * 0.5f;
    if (radTotal < 0) radTotal = 0;

    // Fator de irradiância (0.0 a 1.0+) em relação a 1000 W/m²
    float fatorRad = radTotal / 1000.0f;

    // Correção por temperatura (painel perde eficiência acima de 25°C)
    float correcaoTemp = 1.0f;
    if (tempC > 25.0f) {
        correcaoTemp = 1.0f - (tempC - 25.0f) * 0.004f;
        if (correcaoTemp < 0.6f) correcaoTemp = 0.6f;
    }

    // Corrente estimada = ampMax × irradiância × eficiência × temp × calib
    float corrEstimada = configLocal.ampControlador
                       * fatorRad
                       * configLocal.eficiencia
                       * correcaoTemp
                       * configLocal.fatorCalib;

    if (corrEstimada < 0) corrEstimada = 0;
    return corrEstimada;
}

// ── Busca previsão meteorológica da API Open-Meteo ─────────
void buscarPrevisaoMeteo() {
    if (!configLocal.configurado) {
        adicionarLog("Prev: local nao configurado");
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        adicionarLog("Prev: sem WiFi");
        return;
    }
    if (buscandoPrev) return;

    buscandoPrev = true;
    adicionarLog("Prev: buscando Open-Meteo...");

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
        adicionarLog("Prev ERRO: http.begin() falhou");
        buscandoPrev = false;
        return;
    }
    http.addHeader("User-Agent", "ESP32-RRFS/15.0");
    http.addHeader("Accept",     "application/json");
    http.addHeader("Connection", "close");

    int httpCode = http.GET();
    snprintf(tempBuf, sizeof(tempBuf), "Prev HTTP: %d", httpCode);
    adicionarLog(tempBuf);

    if (httpCode != HTTP_CODE_OK) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "HTTP %d", httpCode);
        adicionarLog(previsao.erro);
        http.end();
        buscandoPrev = false;
        return;
    }

    // Verifica RAM antes de ler
    if (esp_get_free_heap_size() < 8000) {
        adicionarLog("Prev ERRO: RAM insuficiente");
        http.end();
        buscandoPrev = false;
        return;
    }

    String payload = http.getString();
    http.end();

    if (payload.length() < 200) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "Resposta curta: %d chars", payload.length());
        adicionarLog(previsao.erro);
        buscandoPrev = false;
        return;
    }

    if (payload.indexOf("\"error\"") >= 0) {
        snprintf(previsao.erro, sizeof(previsao.erro), "API retornou erro");
        adicionarLog(previsao.erro);
        buscandoPrev = false;
        return;
    }

    // ── Parser JSON manual (extrai arrays) ────────────────
    auto extrairFloat = [&](const char* chave, float* arr, int maxLen) -> int {
        String busca = "\""; busca += chave; busca += "\":[";
        int pos = payload.indexOf(busca);
        if (pos < 0) return 0;
        pos += busca.length();
        if (payload.charAt(pos) == 'n') return 0;
        int count = 0;
        char numBuf[20]; int numIdx = 0;
        while (count < maxLen) {
            char ch = payload.charAt(pos);
            if (ch == ']') {
                if (numIdx > 0) { numBuf[numIdx]='\0'; arr[count++]=atof(numBuf); }
                break;
            } else if (ch == ',') {
                if (numIdx > 0) { numBuf[numIdx]='\0'; arr[count++]=atof(numBuf); numIdx=0; }
            } else if (ch == 'n') {
                arr[count++] = 0.0f; pos += 3; numIdx = 0;
            } else if ((ch>='0'&&ch<='9')||ch=='.'||ch=='-') {
                if (numIdx < 18) numBuf[numIdx++] = ch;
            }
            pos++;
            if (pos >= (int)payload.length()) break;
        }
        return count;
    };

    auto extrairLong = [&](const char* chave, long* arr, int maxLen) -> int {
        String busca = "\""; busca += chave; busca += "\":[";
        int pos = payload.indexOf(busca);
        if (pos < 0) return 0;
        pos += busca.length();
        if (payload.charAt(pos) == 'n') return 0;
        int count = 0;
        char numBuf[16]; int numIdx = 0;
        while (count < maxLen) {
            char ch = payload.charAt(pos);
            if (ch == ']') {
                if (numIdx > 0) { numBuf[numIdx]='\0'; arr[count++]=atol(numBuf); }
                break;
            } else if (ch == ',') {
                if (numIdx > 0) { numBuf[numIdx]='\0'; arr[count++]=atol(numBuf); numIdx=0; }
            } else if (ch>='0' && ch<='9') {
                if (numIdx < 14) numBuf[numIdx++] = ch;
            }
            pos++;
            if (pos >= (int)payload.length()) break;
        }
        return count;
    };

    // Aloca arrays na heap
    float* radDir = (float*)malloc(PREV_HORAS * sizeof(float));
    float* radDif = (float*)malloc(PREV_HORAS * sizeof(float));
    float* nuvens = (float*)malloc(PREV_HORAS * sizeof(float));
    float* temp   = (float*)malloc(PREV_HORAS * sizeof(float));
    long*  stamps = (long*) malloc(PREV_HORAS * sizeof(long));

    if (!radDir || !radDif || !nuvens || !temp || !stamps) {
        adicionarLog("Prev ERRO: malloc falhou");
        if (radDir) free(radDir); if (radDif) free(radDif);
        if (nuvens) free(nuvens); if (temp)   free(temp);
        if (stamps) free(stamps);
        buscandoPrev = false;
        return;
    }

    memset(radDir,0,PREV_HORAS*sizeof(float));
    memset(radDif,0,PREV_HORAS*sizeof(float));
    memset(nuvens,0,PREV_HORAS*sizeof(float));
    memset(temp,  0,PREV_HORAS*sizeof(float));
    memset(stamps,0,PREV_HORAS*sizeof(long));

    int nTs  = extrairLong ("time",                     stamps, PREV_HORAS);
    int nRad = extrairFloat("direct_normal_irradiance", radDir, PREV_HORAS);
    int nDif = extrairFloat("diffuse_radiation",        radDif, PREV_HORAS);
    int nNuv = extrairFloat("cloud_cover",              nuvens, PREV_HORAS);
    int nTmp = extrairFloat("temperature_2m",           temp,   PREV_HORAS);

    // Libera payload imediatamente
    payload = ""; payload.clear();

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev parsed: ts=%d rad=%d dif=%d nuv=%d tmp=%d",
             nTs, nRad, nDif, nNuv, nTmp);
    adicionarLog(tempBuf);

    if (nTs < 1 || nRad < 1) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "Parse falhou: ts=%d rad=%d", nTs, nRad);
        adicionarLog(previsao.erro);
        free(radDir); free(radDif); free(nuvens); free(temp); free(stamps);
        buscandoPrev = false;
        return;
    }

    struct tm ti;
    if (!getLocalTime(&ti)) {
        adicionarLog("Prev ERRO: sem NTP");
        free(radDir); free(radDif); free(nuvens); free(temp); free(stamps);
        buscandoPrev = false;
        return;
    }

    int total = nTs;
    if (nRad < total) total = nRad;
    if (total > PREV_HORAS) total = PREV_HORAS;

    previsao.totalHoras          = 0;
    previsao.correnteHojeRestante = 0;
    previsao.correnteAmanha       = 0;

    int hojeAno = ti.tm_year + 1900;
    int hojeMes = ti.tm_mon  + 1;
    int hojeDia = ti.tm_mday;
    int hojeH   = ti.tm_hour;

    struct tm tiAmanha = ti;
    tiAmanha.tm_mday += 1; tiAmanha.tm_hour = 0;
    tiAmanha.tm_min = 0;   tiAmanha.tm_sec  = 0;
    mktime(&tiAmanha);

    int contHojeRest = 0, contAmanha = 0;

    for (int i = 0; i < total; i++) {
        time_t ts = (time_t)stamps[i];
        struct tm tl;
        localtime_r(&ts, &tl);

        PrevisaoHora& ph   = previsao.horas[i];
        ph.hora            = (uint8_t)tl.tm_hour;
        ph.dia             = (uint8_t)tl.tm_mday;
        ph.mes             = (uint8_t)(tl.tm_mon + 1);
        ph.ano             = (uint16_t)(tl.tm_year + 1900);
        ph.radiacao        = i < nRad ? radDir[i] : 0.0f;
        ph.radiacaoDifusa  = i < nDif ? radDif[i] : 0.0f;
        ph.coberturaNuvens = i < nNuv ? nuvens[i] : 0.0f;
        ph.temperatura     = i < nTmp ? temp[i]   : 25.0f;
        ph.valido          = true;

        if (ph.radiacao       < 0) ph.radiacao       = 0;
        if (ph.radiacaoDifusa < 0) ph.radiacaoDifusa = 0;
        if (ph.coberturaNuvens< 0) ph.coberturaNuvens= 0;
        if (ph.coberturaNuvens>100) ph.coberturaNuvens = 100;

        // Corrente estimada em vez de Wh
        ph.correnteEstimada = calcularCorrenteEstimadaHora(
            ph.radiacao, ph.radiacaoDifusa,
            ph.coberturaNuvens, ph.temperatura);

        // Acumula médias
        if (ph.dia == hojeDia && ph.mes == hojeMes &&
            ph.ano == hojeAno && ph.hora >= hojeH) {
            previsao.correnteHojeRestante += ph.correnteEstimada;
            contHojeRest++;
        }
        if (ph.dia == tiAmanha.tm_mday &&
            ph.mes == (uint8_t)(tiAmanha.tm_mon + 1)) {
            previsao.correnteAmanha += ph.correnteEstimada;
            contAmanha++;
        }
        previsao.totalHoras++;
    }

    // Converte soma → média por hora
    if (contHojeRest > 0)
        previsao.correnteHojeRestante /= contHojeRest;
    if (contAmanha > 0)
        previsao.correnteAmanha /= contAmanha;

    free(radDir); free(radDif); free(nuvens); free(temp); free(stamps);

    previsao.valido            = true;
    previsao.ultimaAtualizacao = millis();
    ultimaPrev                 = millis();
    memset(previsao.erro, 0, sizeof(previsao.erro));

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev OK! h=%d medHoje=%.1fA medAmanha=%.1fA",
             previsao.totalHoras,
             previsao.correnteHojeRestante,
             previsao.correnteAmanha);
    adicionarLog(tempBuf);
    logRAM();
    tomarDecisoes();
    buscandoPrev = false;
}

// ═══════════════════════════════════════════════════════════
// [SIS-PREV] CONSUMO HISTÓRICO E DECISÕES
// ═══════════════════════════════════════════════════════════

// ── Consumo médio (em A) para uma hora específica do dia ──
float consumoMedioParaHora(uint8_t horaAlvo, bool* temDados) {
    if (horaAlvo > 23) horaAlvo = 23;
    if (temDados) *temDados = false;

    float soma = 0; int dias = 0;
    int maxDias = min((int)histTotalDias, 14);

    for (int i = 0; i < maxDias; i++) {
        RegistroDia dTemp;
        bool ok;
        if (i == 0) { dTemp = histDiaAtual; ok = true; }
        else {
            int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
            ok = carregarDiaFlash(idx, dTemp);
        }
        if (ok && dTemp.valido && dTemp.horas[horaAlvo].casaAh > 0) {
            soma += dTemp.horas[horaAlvo].casaAh;
            dias++;
        }
    }

    if (dias > 0) {
        if (temDados) *temDados = true;
        return soma / dias; // Ah médio nessa hora
    }

    // Fallback: média diária / 24 em Ah
    float consumoGeral = 0; int diasG = 0;
    int maxDiasG = min((int)histTotalDias, 7);
    for (int i = 0; i < maxDiasG; i++) {
        RegistroDia dTemp;
        bool ok;
        if (i == 0) { dTemp = histDiaAtual; ok = true; }
        else {
            int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
            ok = carregarDiaFlash(idx, dTemp);
        }
        if (ok && dTemp.valido && dTemp.totalCasaAh > 0) {
            consumoGeral += dTemp.totalCasaAh / 24.0f;
            diasG++;
        }
    }
    // 5Ah/h fallback se sem histórico (≈5A × 1h)
    return diasG > 0 ? (consumoGeral / diasG) : 5.0f;
}

// ── Cache de consumo médio por hora (evita leituras flash) ─
float consumoMedioParaHoraCache(uint8_t horaAlvo, bool* temDados) {
    if (horaAlvo > 23) horaAlvo = 23;

    static float   cacheVal[24] = {0};
    static bool    cacheTem[24] = {false};
    static bool    cacheOk[24]  = {false};
    static uint8_t epochHora    = 255;

    uint8_t horaAgora = 255;
    struct tm ti;
    if (getLocalTime(&ti)) horaAgora = (uint8_t)ti.tm_hour;

    if (horaAgora != epochHora) {
        for (int i = 0; i < 24; i++) cacheOk[i] = false;
        epochHora = horaAgora;
    }

    if (!cacheOk[horaAlvo]) {
        bool td = false;
        cacheVal[horaAlvo] = consumoMedioParaHora(horaAlvo, &td);
        cacheTem[horaAlvo] = td;
        cacheOk[horaAlvo]  = true;
    }

    if (temDados) *temDados = cacheTem[horaAlvo];
    return cacheVal[horaAlvo];
}

// ── Referência de consumo atual (A) para antecipação ──────
float referenciaConsumoAntecip() {
    float consumoInstantA = sensores.correnteCasa;
    struct tm ti;
    if (!getLocalTime(&ti)) return consumoInstantA;

    bool  temHist = false;
    float consumoHistA = consumoMedioParaHoraCache((uint8_t)ti.tm_hour, &temHist);

    if (temHist && consumoInstantA > consumoHistA * 1.3f) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "ANTECIP: consumo atual %.1fA > hist %.1fA h%02d - usando hist",
                 consumoInstantA, consumoHistA, ti.tm_hour);
        adicionarLog(tempBuf);
        return consumoHistA;
    }
    return consumoInstantA;
}

// ── Horas até geração prevista superar refA amperes ────────
float horasAteGeracaoSuperar(float refA) {
    if (!previsao.valido) return -1;
    time_t tsAgora = time(nullptr);
    if (tsAgora < 1000000) return -1;

    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (!ph.valido) continue;
        struct tm tC = {0};
        tC.tm_year = ph.ano - 1900; tC.tm_mon  = ph.mes - 1;
        tC.tm_mday = ph.dia;        tC.tm_hour = ph.hora;
        tC.tm_isdst = -1;
        time_t tsH = mktime(&tC);
        if (tsH <= tsAgora) continue;
        if (ph.correnteEstimada > refA)
            return (float)(tsH - tsAgora) / 3600.0f;
    }
    return -1;
}

// ── Toma decisões automáticas baseadas na previsão ─────────
void tomarDecisoes() {
    if (!previsao.valido || !bateria.calibrada) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;

    int  horaAtual = ti.tm_hour;
    bool eNoite    = (horaAtual >= 18 || horaAtual < 6);
    bool eDia      = !eNoite;

    // Consumo médio geral em Ah/h
    float consumoMedioAh = 0; int diasV = 0;
    for (int i = 0; i < min((int)histTotalDias, 7); i++) {
        int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
        RegistroDia dTemp;
        bool ok = (i == 0) ? (dTemp = histDiaAtual, true)
                            : carregarDiaFlash(idx, dTemp);
        if (ok && dTemp.valido && dTemp.totalCasaAh > 0) {
            consumoMedioAh += dTemp.totalCasaAh / 24.0f;
            diasV++;
        }
    }
    if (diasV > 0) consumoMedioAh /= diasV;
    else           consumoMedioAh  = 5.0f; // fallback 5Ah/h

    float ahAtual    = bateria.ahAtual;
    float ahMin      = bateria.ahDisponivel * (redes.corteBaixa / 100.0f);

    decisao.desligarRedeAltaNoite    = false;
    decisao.desligarLampExtNoite     = false;
    decisao.desligarLampIntDia       = false;
    decisao.alertaEnergiaInsuficiente= false;

    // ── Bloco noturno ──────────────────────────────────────
    if (eNoite) {
        float horaInicioGeracao = 6.0f;
        for (int i = 0; i < previsao.totalHoras; i++) {
            PrevisaoHora& ph = previsao.horas[i];
            if (ph.correnteEstimada > 1.0f &&
                (ph.dia == ti.tm_mday || ph.dia == ti.tm_mday + 1)) {
                horaInicioGeracao = ph.hora;
                break;
            }
        }

        float horasAteAmanhecer = (horaAtual >= 18)
            ? (24.0f - horaAtual + horaInicioGeracao)
            : (horaInicioGeracao - horaAtual);

        float ahNecessario  = consumoMedioAh * horasAteAmanhecer;
        float ahDisponivel  = ahAtual - ahMin;
        float saldo         = ahDisponivel - ahNecessario;

        // Rede Alta consome ~40% do total
        float consumoAltaAh = consumoMedioAh * 0.40f;

        if (saldo < 0) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Noite: saldo=%.1fAh disp=%.1fAh nec=%.1fAh",
                     saldo, ahDisponivel, ahNecessario);
        } else if (saldo < consumoAltaAh * 3.0f) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Noite: margem baixa saldo=%.1fAh", saldo);
        }

        decisao.horaLigarRedeAlta = horaInicioGeracao + 1.0f;

        // Lamp ext: só desliga se sem geração e saldo baixo
        if (sensores.correntePainel <= 0.5f) {
            float consumoLampAh = consumoMedioAh * 0.05f;
            if (saldo < consumoLampAh * horasAteAmanhecer)
                decisao.desligarLampExtNoite = true;
        }

        if (saldo < 0 && previsao.correnteAmanha < consumoMedioAh * 0.5f)
            decisao.alertaEnergiaInsuficiente = true;

        if (decisao.desligarRedeAltaNoite &&
            estados.redesAlta && curto.estado == CURTO_NORMAL) {
            desligarRedeAlta();
            antecipAlta.estado    = ANTECIP_IDLE;
            antecipAlta.tCooldown = millis() + T_ANTECIP_COOLDOWN;
            adicionarLog("DECISAO NOITE: Rede Alta OFF");
            adicionarLog(decisao.motivo);
        }
        if (decisao.desligarLampExtNoite && estados.lampExt) {
            desligarLampExt();
            adicionarLog("DECISAO NOITE: Lamp.Ext OFF");
        }

        snprintf(tempBuf, sizeof(tempBuf),
                 "Decisao noite: saldo=%.1fAh nec=%.1fAh h=%.1fh",
                 saldo, ahNecessario, horasAteAmanhecer);
        adicionarLog(tempBuf);
    }

    // ── Bloco diurno ───────────────────────────────────────
    if (eDia) {
        float consumoRestAh = 0;
        for (int h = horaAtual; h < 24; h++)
            consumoRestAh += consumoMedioParaHoraCache((uint8_t)h);

        // Geração restante: soma corrente média × horas futuras
        float geracaoRestAh = 0;
        for (uint8_t i = 0; i < previsao.totalHoras; i++) {
            PrevisaoHora& ph = previsao.horas[i];
            if (!ph.valido) continue;
            if (ph.dia == ti.tm_mday && ph.mes == (uint8_t)(ti.tm_mon+1) &&
                ph.hora >= (uint8_t)horaAtual)
                geracaoRestAh += ph.correnteEstimada; // Ah estimado
        }

        float reservaAh = bateria.ahDisponivel * (redes.corteBaixa / 100.0f);
        float saldoDia  = (ahAtual + geracaoRestAh) - (consumoRestAh + reservaAh);

        snprintf(tempBuf, sizeof(tempBuf),
                 "Decisao dia: bat=%.1fAh gerRest=%.1fAh consRest=%.1fAh saldo=%.1fAh",
                 ahAtual, geracaoRestAh, consumoRestAh, saldoDia);
        adicionarLog(tempBuf);

        if (saldoDia < 0) {
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Deficit diurno: saldo=%.1fAh", saldoDia);

            if (estados.redesAlta && curto.estado == CURTO_NORMAL &&
                antecipAlta.estado == ANTECIP_IDLE) {
                desligarRedeAlta();
                adicionarLog("DECISAO DIA: Rede Alta OFF - deficit projetado");
                adicionarLog(decisao.motivo);
            } else if (bateria.percentualAtual <= redes.corteLampInt + 10.0f) {
                decisao.desligarLampIntDia = true;
            }
        }

        if (decisao.desligarLampIntDia && estados.lampInt) {
            desligarLampInt();
            adicionarLog("DECISAO DIA: Lamp.Interna OFF - deficit critico");
        }
    }
}

// ── Verifica religamento automático pela manhã ─────────────
void verificarReligamentoAuto() {
    if (!previsao.valido) return;
    if (!decisao.desligarRedeAltaNoite) return;
    if (curto.estado != CURTO_NORMAL) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;

    bool deveReligar = false;

    if (sensores.correntePainel > 0.5f &&
        bateria.percentualAtual > redes.corteAlta + 5.0f &&
        sensores.inversoreAtivo)
        deveReligar = true;

    if ((float)ti.tm_hour >= decisao.horaLigarRedeAlta &&
        bateria.percentualAtual > redes.corteAlta + 2.0f &&
        sensores.inversoreAtivo)
        deveReligar = true;

    if (deveReligar && !estados.redesAlta) {
        ligarRedeAlta();
        decisao.desligarRedeAltaNoite = false;
        adicionarLog("DECISAO: Rede Alta ON - religamento auto manha");
    }
}

// ── Antecipação inteligente da Rede Alta ──────────────────
void avaliarAntecipacaoRedeAlta() {
    if (curto.estado != CURTO_NORMAL) {
        antecipAlta.estado = ANTECIP_IDLE;
        return;
    }
    if (!bateria.calibrada) return;

    unsigned long agora = millis();
    float pct = bateria.percentualAtual;

    if (pct > redes.corteAlta + 2.0f) {
        antecipAlta.estado = ANTECIP_IDLE;
        return;
    }

    switch (antecipAlta.estado) {

        case ANTECIP_IDLE: {
            if (antecipAlta.tCooldown && agora < antecipAlta.tCooldown) return;
            antecipAlta.tCooldown = 0;
            if (estados.redesAlta) return;
            if (!estados.inversor || !sensores.inversoreAtivo) return;
            if (!previsao.valido) return;
            if (pct <= redes.corteBaixa + 5.0f) return;
            if (sensores.correntePainel < 0.3f) return;

            struct tm ti;
            if (!getLocalTime(&ti)) return;

            float genAtualA  = sensores.correntePainel;
            float genProxima = -1;
            for (uint8_t i = 0; i < previsao.totalHoras; i++) {
                PrevisaoHora& ph = previsao.horas[i];
                if (!ph.valido) continue;
                if (ph.dia == ti.tm_mday &&
                    ph.mes == (uint8_t)(ti.tm_mon+1) &&
                    ph.hora > (uint8_t)ti.tm_hour) {
                    genProxima = ph.correnteEstimada;
                    break;
                }
            }
            if (genProxima < 0 || genProxima < genAtualA * 0.8f) return;

            ligarRedeAlta();
            antecipAlta.estado  = ANTECIP_TESTANDO;
            antecipAlta.tInicio = agora;
            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIP: Rede Alta TESTE bat=%.0f%% genAtual=%.1fA genProx=%.1fA",
                     pct, genAtualA, genProxima);
            adicionarLog(tempBuf);
            break;
        }

        case ANTECIP_TESTANDO: {
            if (!estados.redesAlta) { antecipAlta.estado = ANTECIP_IDLE; return; }
            if (agora - antecipAlta.tInicio < T_ANTECIP_ESTABILIZA) return;

            float corrBat = sensores.correnteBateria;
            if (corrBat >= -0.3f) {
                antecipAlta.estado      = ANTECIP_MANTIDO;
                antecipAlta.tUltimaAval = agora;
                adicionarLog("ANTECIP: Rede Alta MANTIDA - geracao cobre consumo");
                return;
            }

            float ahAteCorte  = bateria.ahAtual -
                                 (bateria.ahDisponivel * (redes.corteBaixa/100.0f));
            float horasAteCorte = (ahAteCorte <= 0) ? 0.0f :
                                   (ahAteCorte / fabsf(corrBat));

            float refA            = referenciaConsumoAntecip();
            float horasAteMelhora = horasAteGeracaoSuperar(refA);
            float margemH         = MARGEM_ANTECIP_MIN / 60.0f;

            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIP teste: Ibat=%.1fA ref=%.1fA hCorte=%.2fh hMelhora=%.2fh",
                     corrBat, refA, horasAteCorte, horasAteMelhora);
            adicionarLog(tempBuf);

            bool ok = (horasAteMelhora >= 0) &&
                      (horasAteCorte > horasAteMelhora + margemH);
            if (ok) {
                antecipAlta.estado      = ANTECIP_MANTIDO;
                antecipAlta.tUltimaAval = agora;
                adicionarLog("ANTECIP: Rede Alta MANTIDA - geracao melhora a tempo");
            } else {
                desligarRedeAlta();
                antecipAlta.estado    = ANTECIP_IDLE;
                antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
                adicionarLog("ANTECIP: Rede Alta OFF - nao favoravel");
            }
            break;
        }

        case ANTECIP_MANTIDO: {
            if (!estados.redesAlta) { antecipAlta.estado = ANTECIP_IDLE; return; }
            if (agora - antecipAlta.tUltimaAval < T_ANTECIP_REAVAL) return;
            antecipAlta.tUltimaAval = agora;

            float corrBat = sensores.correnteBateria;
            if (corrBat >= -0.3f) return;

            float ahAteCorte  = bateria.ahAtual -
                                 (bateria.ahDisponivel * (redes.corteBaixa/100.0f));
            float horasAteCorte = (ahAteCorte <= 0) ? 0.0f :
                                   (ahAteCorte / fabsf(corrBat));
            float refA            = referenciaConsumoAntecip();
            float horasAteMelhora = horasAteGeracaoSuperar(refA);
            float margemH         = MARGEM_ANTECIP_MIN / 60.0f;

            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIP reaval: Ibat=%.1fA ref=%.1fA hCorte=%.2fh hMelhora=%.2fh",
                     corrBat, refA, horasAteCorte, horasAteMelhora);
            adicionarLog(tempBuf);

            bool ok = (horasAteMelhora >= 0) &&
                      (horasAteCorte > horasAteMelhora + margemH);
            if (!ok) {
                desligarRedeAlta();
                antecipAlta.estado    = ANTECIP_IDLE;
                antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
                adicionarLog("ANTECIP: Rede Alta OFF - reavaliacao desfavoravel");
            }
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [SIS-CTRL] CONTROLE AUTOMÁTICO DE ENERGIA
// ═══════════════════════════════════════════════════════════

// ── Restauração não-bloqueante de energia ─────────────────
void processarRestauroEnergia() {
    if (restauro.estado == RESTORE_OCIOSO) return;
    unsigned long agora = millis();

    switch (restauro.estado) {
        case RESTORE_LIGANDO_BAIXA:
            if (!estados.redeBaixa) {
                ligarRedeBaixa();
                adicionarLog("Rede Baixa ON: restaurada");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_AGUARDANDO_BAIXA;
            break;

        case RESTORE_AGUARDANDO_BAIXA:
            if (agora - restauro.tInicio >= RESTORE_T_BAIXA)
                restauro.estado = RESTORE_LIGANDO_INVERSOR;
            break;

        case RESTORE_LIGANDO_INVERSOR:
            if (!estados.inversor) {
                ligarInversor();
                adicionarLog("Inversor ON: restaurado");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_AGUARDANDO_INVERSOR_BOOT;
            break;

        case RESTORE_AGUARDANDO_INVERSOR_BOOT:
            if (agora - restauro.tInicio >= RESTORE_T_INV_BOOT) {
                restauro.tInicio = agora;
                restauro.estado  = RESTORE_CHECANDO_STATUS_INV;
            }
            break;

        case RESTORE_CHECANDO_STATUS_INV:
            if (lerStatusInversor()) {
                restauro.estado = RESTORE_LIGANDO_ALTA;
            } else if (agora - restauro.tInicio >= RESTORE_T_INV_STATUS) {
                restauro.estado = RESTORE_LIGANDO_ALTA;
            }
            break;

        case RESTORE_LIGANDO_ALTA:
            if (!estados.redesAlta &&
                bateria.percentualAtual > redes.corteAlta + 2.0f &&
                sensores.inversoreAtivo) {
                ligarRedeAlta();
                adicionarLog("Rede Alta ON: restaurada");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_LIGANDO_LAMP_INT;
            break;

        case RESTORE_LIGANDO_LAMP_INT:
            if (agora - restauro.tInicio >= RESTORE_T_ALTA) {
                if (!estados.lampInt &&
                    bateria.percentualAtual > redes.corteLampInt + 2.0f &&
                    sensores.inversoreAtivo) {
                    ligarLampInt();
                    adicionarLog("Lamp.Interna ON: restaurada");
                }
                restauro.estado = RESTORE_CONCLUIDO;
            }
            break;

        case RESTORE_CONCLUIDO:
        default:
            restauro.estado = RESTORE_OCIOSO;
            break;
    }
}

// ── Controla cargas por nível de bateria ──────────────────
void controlarEnergia() {
    if (!bateria.calibrada) return;

    float pct  = bateria.percentualAtual;
    unsigned long agora = millis();

    // ── Lâmpada Externa: controle por horário solar ────────
    // Roda em QUALQUER modo (dia/noite, nao e corte por energia baixa)
    {
        float latRad = 0, declin = 0, declRad = 0;
        float horasLuz = 12.0f, nascerSol = 6.0f, porSol = 18.0f;

        struct tm ti;
        if (getLocalTime(&ti) && configLocal.configurado) {
            int mes  = ti.tm_mon + 1;
            latRad   = (float)(configLocal.latitude * 3.14159 / 180.0);
            declin   = -23.45f * cosf(2.0f*3.14159f*(float)(mes+9.5f)/12.0f);
            declRad  = declin * 3.14159f / 180.0f;
            float cH = -tanf(latRad) * tanf(declRad);
            if (cH >= -1.0f && cH <= 1.0f)
                horasLuz = (2.0f/15.0f)*(acosf(cH)*180.0f/3.14159f);
            nascerSol = 12.0f - (horasLuz / 2.0f);
            porSol    = 12.0f + (horasLuz / 2.0f);
            float horaF = (float)ti.tm_hour + (float)ti.tm_min / 60.0f;
            ehDia = (horaF >= (nascerSol+0.5f) && horaF < (porSol+0.5f));
        } else {
            if (!ehDia && sensores.correntePainel > 1.0f)       ehDia = true;
            else if (ehDia && sensores.correntePainel < 0.3f)   ehDia = false;
        }

        static unsigned long tMudancaLampExt = 0;
        static bool ehDiaConf = false;
        static bool primeiraVez = true;
        if (primeiraVez) { primeiraVez = false; ehDiaConf = ehDia; }

        if (ehDia != ehDiaConf) {
            if (!tMudancaLampExt) tMudancaLampExt = agora;
            else if (agora - tMudancaLampExt >= 60000UL) {
                ehDiaConf = ehDia; tMudancaLampExt = 0;
                if (ehDiaConf) {
                    if (estados.lampExt) { desligarLampExt(); adicionarLog("Lamp.Ext OFF: amanheceu"); }
                } else {
                    if (!estados.lampExt && pct > redes.corteLampExt+2.0f &&
                        estados.inversor && sensores.inversoreAtivo &&
                        (ia.config.modo != IA_MODO_IA || iaPodeLigar("lampExt"))) {
                        ligarLampExt(); adicionarLog("Lamp.Ext ON: anoiteceu");
                        }
                }
            }
        } else {
            tMudancaLampExt = 0;
            if (ehDiaConf && estados.lampExt) {
                desligarLampExt(); adicionarLog("Lamp.Ext OFF: correcao (dia)");
            } else if (!ehDiaConf && !estados.lampExt &&
                pct > redes.corteLampExt+2.0f &&
                estados.inversor && sensores.inversoreAtivo &&
                (ia.config.modo != IA_MODO_IA || iaPodeLigar("lampExt"))) {
                ligarLampExt(); adicionarLog("Lamp.Ext ON: correcao (noite)");
                }
        }
    }

    // ═══════════════════════════════════════════════════════
    // A PARTIR DAQUI: corte por % da bateria.
    // SÓ roda no Modo Manual. No Modo IA quem manda é iaTomarDecisao().
    // ═══════════════════════════════════════════════════════
    if (ia.config.modo == IA_MODO_IA) return;

    // ── Lâmpada Interna ────────────────────────────────────
    if (pct <= redes.corteLampInt && estados.lampInt) {
        if (!timers.pendenteLampInt) {
            timers.pendenteLampInt = true; timers.inicioLampInt = agora;
        } else if (agora - timers.inicioLampInt >= RELE_BUFFER_MS) {
            desligarLampInt(); timers.pendenteLampInt = false;
            adicionarLog("Lamp.Interna OFF: energia baixa");
        }
    } else {
        timers.pendenteLampInt = false;
        if (pct > redes.corteLampInt+2.0f && !estados.lampInt &&
            estados.inversor && sensores.inversoreAtivo) {
            ligarLampInt(); adicionarLog("Lamp.Interna ON: recuperada");
            }
    }

    // ── Lâmpada Externa: corte por energia baixa ──────────
    if (pct <= redes.corteLampExt && estados.lampExt) {
        if (!timers.pendenteLampExt) {
            timers.pendenteLampExt = true; timers.inicioLampExt = agora;
        } else if (agora - timers.inicioLampExt >= RELE_BUFFER_MS) {
            desligarLampExt(); timers.pendenteLampExt = false;
            adicionarLog("Lamp.Ext OFF: energia baixa");
        }
    } else {
        timers.pendenteLampExt = false;
    }

    // ── Rede Alta ──────────────────────────────────────────
    bool antecipAtiva = (antecipAlta.estado != ANTECIP_IDLE);

    if (pct <= redes.corteAlta && estados.redesAlta && !antecipAtiva) {
        if (!timers.pendenteAlta) {
            timers.pendenteAlta = true; timers.inicioAlta = agora;
        } else if (agora - timers.inicioAlta >= RELE_BUFFER_MS) {
            desligarRedeAlta(); timers.pendenteAlta = false;
            adicionarLog("Rede Alta OFF: energia baixa");
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
        timers.pendenteBaixa = true; timers.inicioBaixa = agora;
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
            if (pct > redes.corteBaixa+2.0f &&
                (!estados.redeBaixa || !estados.inversor) &&
                restauro.estado == RESTORE_OCIOSO) {
                restauro.estado = RESTORE_LIGANDO_BAIXA;
            adicionarLog("Iniciando restauracao (nao-bloqueante)");
                }
        }
}

// ═══════════════════════════════════════════════════════════
// [SIS-IA] MOTOR DE APRENDIZADO DE PADRÕES
// ═══════════════════════════════════════════════════════════

void iaSalvarConfig() {
    prefs.begin(IA_NS_CONFIG, false);
    prefs.putUChar("modo",    (uint8_t)ia.config.modo);
    prefs.putFloat("reserva", ia.config.reservaEmergencia);
    prefs.putBool ("conf",    ia.config.configurado);
    prefs.end();
}

void iaCarregarConfig() {
    prefs.begin(IA_NS_CONFIG, true);
    ia.config.modo              = (ModoIAOperacao)prefs.getUChar("modo", 0);
    ia.config.reservaEmergencia = prefs.getFloat("reserva", 5.0f);
    ia.config.configurado       = prefs.getBool ("conf",    false);
    prefs.end();
    if (!ia.config.configurado) {
        ia.config.modo              = IA_MODO_MANUAL;
        ia.config.reservaEmergencia = 5.0f;
        ia.config.configurado       = true;
        iaSalvarConfig();
        adicionarLog("IA: config inicial criada");
    }
    snprintf(tempBuf, sizeof(tempBuf),
             "IA: modo=%s reserva=%.0f%% (sempre aprende)",
             ia.config.modo == IA_MODO_IA ? "IA_ATIVA" : "MANUAL",
             ia.config.reservaEmergencia);
    adicionarLog(tempBuf);
}

void iaSalvarPadroes() {
    prefs.begin(IA_NS_PADROES, false);
    prefs.putUChar("ver",   IA_KEY_VER);
    prefs.putUChar("total", ia.totalPadroes);
    prefs.putUChar("iant",  ia.indicePadraoMaisAntigo);

    bool falhouAlgum = false;
    for (int i = 0; i < IA_MAX_PADROES; i++) {
        char chave[8];
        snprintf(chave, sizeof(chave), "p%02d", i);
        size_t escrito = prefs.putBytes(chave, &ia.padroes[i],
                                        sizeof(PadraoConsumo));
        if (escrito != sizeof(PadraoConsumo)) falhouAlgum = true;
    }
    prefs.end();

    if (falhouAlgum) {
        adicionarLog("IA AVISO: falha ao salvar 1+ padroes individuais");
        nvs_stats_t stats;
        if (nvs_get_stats(NULL, &stats) == ESP_OK) {
            snprintf(tempBuf, sizeof(tempBuf),
                     "NVS: %d entradas livres de %d (usadas=%d)",
                     stats.free_entries, stats.total_entries,
                     stats.used_entries);
            adicionarLog(tempBuf);
        }
    }
}

void iaCarregarPadroes() {
    prefs.begin(IA_NS_PADROES, true);
    uint8_t ver = prefs.getUChar("ver", 0);

    if (ver != IA_KEY_VER) {
        prefs.end();
        memset(ia.padroes, 0, sizeof(PadraoConsumo) * IA_MAX_PADROES);
        ia.totalPadroes = 0; ia.indicePadraoMaisAntigo = 0;

        // Migração: se existia o formato antigo (blob único "dados"),
        // tenta ler dele antes de desistir - evita perder aprendizado
        // já acumulado ao atualizar o firmware.
        prefs.begin(IA_NS_PADROES, true);
        PadraoConsumo bufAntigo[IA_MAX_PADROES];
        size_t lidoAntigo = prefs.getBytes("dados", bufAntigo,
                                           sizeof(PadraoConsumo) * IA_MAX_PADROES);
        uint8_t totalAntigo = prefs.getUChar("total", 0);
        prefs.end();

        if (lidoAntigo == sizeof(PadraoConsumo) * IA_MAX_PADROES &&
            totalAntigo > 0) {
            memcpy(ia.padroes, bufAntigo, sizeof(bufAntigo));
        ia.totalPadroes = totalAntigo;
        adicionarLog("IA: padroes migrados do formato antigo (blob unico)");
        iaSalvarPadroes(); // já grava no formato novo
            } else {
                adicionarLog("IA: padroes resetados (versao nova / sem dados antigos)");
            }
            return;
    }

    ia.totalPadroes           = prefs.getUChar("total", 0);
    ia.indicePadraoMaisAntigo = prefs.getUChar("iant",  0);

    for (int i = 0; i < IA_MAX_PADROES; i++) {
        char chave[8];
        snprintf(chave, sizeof(chave), "p%02d", i);
        size_t lido = prefs.getBytes(chave, &ia.padroes[i],
                                     sizeof(PadraoConsumo));
        if (lido != sizeof(PadraoConsumo)) {
            memset(&ia.padroes[i], 0, sizeof(PadraoConsumo));
        }
    }
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "IA: %d padroes carregados (formato individual)", ia.totalPadroes);
    adicionarLog(tempBuf);
}

void iaInicializar() {
    memset(&ia, 0, sizeof(GerenciadorIA));
    iaCarregarConfig();
    iaCarregarPadroes();
    ia.analise.estado            = IA_OBSERVANDO;
    ia.analise.baselineAtual     = 0.0f;
    ia.analise.baselineConfianca = 0.0f;
    ia.analise.emPico            = false;
    ia.analise.janelasEstavel    = 0;
    ia.sequencia                 = SEQ_OCIOSO;
    ia.recomendacao.ativa        = false;
    ia.usandoFallbackManual      = false;
    adicionarLog("IA: inicializada (aprendizado sempre ativo)");
    logRAM();
}

// ── Define recomendação visível na página principal ────────
void iaSetRecomendacao(uint8_t nivel,
                       const char* texto,
                       const char* detalhe) {
    ia.recomendacao.ativa = true;
    ia.recomendacao.nivel = nivel;
    strncpy(ia.recomendacao.texto,   texto,   127);
    strncpy(ia.recomendacao.detalhe, detalhe, 127);
    ia.recomendacao.texto[127]   = '\0';
    ia.recomendacao.detalhe[127] = '\0';
    ia.recomendacao.tCriado = millis();
    adicionarLog(texto);
}

void iaLimparRecomendacao() {
    ia.recomendacao.ativa = false;
    memset(ia.recomendacao.texto,   0, sizeof(ia.recomendacao.texto));
    memset(ia.recomendacao.detalhe, 0, sizeof(ia.recomendacao.detalhe));
}

// ── Calcula saldo Ah: disponível vs necessário ─────────────

// ── NOVO: verifica se a IA já tem confiança suficiente para agir ──
bool iaTemConfiancaSuficiente() {
    if (ia.totalPadroes < 3) return false;   // poucos padroes distintos
    uint32_t totalOcorrencias = 0;
    for (int i = 0; i < IA_MAX_PADROES; i++)
        if (ia.padroes[i].valido) totalOcorrencias += ia.padroes[i].ocorrencias;
        return (totalOcorrencias >= 10);  // pelo menos 10 eventos observados
}

// ── NOVO: estima consumo esperado somando padroes ponderados por ocorrencia ──
float iaConsumoEsperadoAh(float horas) {
    float totalAh = 0;

    // Baseline continuo (consumo de base que nunca desliga)
    if (ia.analise.baselineConfianca > 0.3f)
        totalAh += ia.analise.baselineAtual * horas;

    // Padroes transitorios/ciclicos/longos: pondera pela frequencia real
    // de ocorrencia observada no historico, nao pelo pico instantaneo.
    int diasHist = max((int)histTotalDias, 1);
    for (int i = 0; i < IA_MAX_PADROES; i++) {
        if (!ia.padroes[i].valido) continue;
        if (ia.padroes[i].tipo == PADRAO_BASELINE) continue; // ja contado acima

        // Frequencia media: quantas vezes esse padrao costuma ocorrer
        // por hora, com base no total de ocorrencias / dias de historico
        float freqPorHora = (float)ia.padroes[i].ocorrencias /
        (float)diasHist / 24.0f;
        float ocorrenciasNoPeriodo = freqPorHora * horas;
        totalAh += ocorrenciasNoPeriodo * ia.padroes[i].energiaAh;
    }
    return totalAh;
}

// Lógica central de decisão
float iaCalcularSaldoAh() {
    if (!bateria.calibrada) return 999.0f; // sem dados = não age

    float ahDisp     = bateria.ahAtual;
    float reservaAh  = bateria.ahDisponivel *
    (ia.config.reservaEmergencia / 100.0f);
    float ahUtil     = ahDisp - reservaAh;

    // Calcula Ah necessários até amanhecer ou próxima geração
    float ahNec = 0.0f;

    struct tm ti;
    if (!getLocalTime(&ti)) {
        // Sem NTP: usa correnteCasa × 4h como estimativa
        ahNec = sensores.correnteCasa * 4.0f;
    } else {
        int hora = ti.tm_hour;
        bool eNoite = (hora >= 18 || hora < 6);

        if (eNoite) {
            // Horas até início da geração
            float horasAte = (hora >= 18)
            ? (24.0f - hora + 6.0f)
            : (6.0f - hora);

            if (iaTemConfiancaSuficiente()) {
                // CORRIGIDO: usa modelo baseado em padroes reais e
                // frequencia de ocorrencia, nao so corrente instantanea.
                // Isso evita superestimar consumo de picos raros/curtos.
                ahNec = iaConsumoEsperadoAh(horasAte);
                ia.usandoFallbackManual = false;
            } else if (ia.snapshot.total >= IA_JANELA_2MIN) {
                // Poucos padroes ainda, mas ja tem alguma media recente:
                // usa media de janela (nunca corrente instantanea pura)
                float corrEst = iaMediaJanela(
                    (uint16_t)min((int)ia.snapshot.total, (int)IA_JANELA_10MIN));
                ahNec = corrEst * horasAte;
                ia.usandoFallbackManual = true;
            } else {
                // Praticamente sem dados: usa corrente atual, mas
                // CLAMPADA para nao multiplicar um pico pontual por
                // muitas horas seguidas.
                float corrClamp = sensores.correnteCasa;
                if (corrClamp > 5.0f) corrClamp = 5.0f;
                ahNec = corrClamp * horasAte;
                ia.usandoFallbackManual = true;
            }
        } else {
            // Dia: calcula se geração prevista cobre consumo restante
            float saldoInstant = sensores.correntePainel
            - sensores.correnteCasa;
            if (saldoInstant >= 0) {
                ia.decisao.ahDisponivel         = ahUtil;
                ia.decisao.ahEstimadoNecessario = 0.0f;
                ia.saldoAhAtual                 = 999.0f;
                ia.usandoFallbackManual         = false;
                return 999.0f; // gerando suficiente
            }

            float horasAteMelhora = horasAteGeracaoSuperar(
                sensores.correnteCasa * 0.8f);
            if (horasAteMelhora < 0) horasAteMelhora = 2.0f;
            ahNec = fabsf(saldoInstant) * horasAteMelhora;
        }
    }

    ia.ahNecAmanhecerAtual = ahNec;
    ia.saldoAhAtual        = ahUtil - ahNec;

    // Preenche os campos usados no dashboard (antes ficavam sempre 0.0)
    ia.decisao.ahDisponivel         = ahUtil;
    ia.decisao.ahEstimadoNecessario = ahNec;

    return ia.saldoAhAtual;
}

// ── Verifica se tem dados suficientes para decidir ─────────
bool iaTeDadosSuficientes() {
    return (ia.snapshot.total >= IA_MIN_DADOS_DECISAO);
}

// ── Verifica anti-oscilação por carga ─────────────────────
bool iaPassouTempo(unsigned long* tRef, unsigned long intervalo) {
    if (*tRef == 0) return true;
    return (millis() - *tRef >= intervalo);
}

void iaAdicionarSnapshot(float corrente) {
    ia.snapshot.amostras[ia.snapshot.indice] = corrente;
    ia.snapshot.indice = (ia.snapshot.indice + 1) % IA_BUFFER_SNAP;
    if (ia.snapshot.total < IA_BUFFER_SNAP) ia.snapshot.total++;
    if (ia.snapshot.total == IA_BUFFER_SNAP) ia.snapshot.cheio = true;
}

float iaMediaJanela(uint16_t segundos) {
    if (ia.snapshot.total == 0) return 0;
    uint8_t n = (uint8_t)min((int)segundos, (int)ia.snapshot.total);
    float soma = 0;
    for (uint8_t i = 0; i < n; i++) {
        int idx = ((int)ia.snapshot.indice - 1 - i + IA_BUFFER_SNAP) % IA_BUFFER_SNAP;
        soma += ia.snapshot.amostras[idx];
    }
    return soma / n;
}

float iaDesvioPadraoJanela(uint16_t segundos) {
    if (ia.snapshot.total < 2) return 0;
    float media = iaMediaJanela(segundos);
    uint8_t n   = (uint8_t)min((int)segundos, (int)ia.snapshot.total);
    float soma  = 0;
    for (uint8_t i = 0; i < n; i++) {
        int idx = ((int)ia.snapshot.indice - 1 - i + IA_BUFFER_SNAP) % IA_BUFFER_SNAP;
        float d = ia.snapshot.amostras[idx] - media;
        soma += d * d;
    }
    return sqrtf(soma / n);
}

int iaEncontrarPadraoSimilar(TipoPadrao tipo, float corrente, float duracao) {
    for (int i = 0; i < IA_MAX_PADROES; i++) {
        if (!ia.padroes[i].valido) continue;
        if (ia.padroes[i].tipo != tipo) continue;
        if (fabsf(ia.padroes[i].correnteMedia - corrente) > 1.5f) continue;
        float difDur = fabsf(ia.padroes[i].duracaoMedia - duracao);
        float pct    = ia.padroes[i].duracaoMedia * 0.30f;
        if (difDur > max(pct, 30.0f)) continue;
        return i;
    }
    return -1;
}

void iaRegistrarPadrao(TipoPadrao tipo, float corrMedia, float corrPico,
                        float corrMin, float duracao, float energiaAh) {
    struct tm ti; uint8_t hora = 0;
    if (getLocalTime(&ti)) hora = ti.tm_hour;

    int idx = iaEncontrarPadraoSimilar(tipo, corrMedia, duracao);
    if (idx >= 0) {
        PadraoConsumo& p = ia.padroes[idx];
        float peso = 1.0f / (float)(p.ocorrencias + 1);
        p.correnteMedia = p.correnteMedia*(1.0f-peso) + corrMedia*peso;
        p.correntePico  = max(p.correntePico, corrPico);
        p.correnteMin   = min(p.correnteMin,  corrMin);
        p.duracaoMedia  = p.duracaoMedia*(1.0f-peso) + duracao*peso;
        p.duracaoMin    = min(p.duracaoMin, duracao);
        p.duracaoMax    = max(p.duracaoMax, duracao);
        p.energiaAh     = p.energiaAh*(1.0f-peso) + energiaAh*peso;
        p.ocorrencias++;
        snprintf(tempBuf, sizeof(tempBuf),
                 "IA: padrao[%d] atualizado I=%.1fA dur=%.0fs (%lu x)",
                 idx, corrMedia, duracao, p.ocorrencias);
        adicionarLog(tempBuf);
    } else {
        int slotNovo = -1;
        for (int i = 0; i < IA_MAX_PADROES; i++) {
            if (!ia.padroes[i].valido) { slotNovo = i; break; }
        }
        if (slotNovo < 0) {
            slotNovo = ia.indicePadraoMaisAntigo;
            ia.indicePadraoMaisAntigo = (ia.indicePadraoMaisAntigo+1) % IA_MAX_PADROES;
        } else {
            if (ia.totalPadroes < IA_MAX_PADROES) ia.totalPadroes++;
        }
        PadraoConsumo& p = ia.padroes[slotNovo];
        p.tipo          = tipo;    p.correnteMedia = corrMedia;
        p.correntePico  = corrPico; p.correnteMin  = corrMin;
        p.duracaoMedia  = duracao; p.duracaoMin    = duracao;
        p.duracaoMax    = duracao; p.energiaAh     = energiaAh;
        p.ocorrencias   = 1;       p.horaInicio    = hora;
        p.valido        = true;
        snprintf(tempBuf, sizeof(tempBuf),
                 "IA: NOVO padrao[%d] tipo=%d I=%.1fA dur=%.0fs",
                 slotNovo, tipo, corrMedia, duracao);
        adicionarLog(tempBuf);
    }

    // Salva com cooldown de 60s
    static unsigned long tUltimoSalva = 0;
    if (millis() - tUltimoSalva >= 60000) {
        tUltimoSalva = millis();
        iaSalvarPadroes();
    }
}

// ── Motor principal de aprendizado (chamado a cada 1s) ────
void iaAnalisar() {
    // IA SEMPRE analisa - independente do modo
    float corr = sensores.correnteCasa;
    unsigned long agora = millis();

    iaAdicionarSnapshot(corr);
    if (ia.snapshot.total < IA_JANELA_2MIN) return;

    EstadoAnalise& an = ia.analise;
    float media2  = iaMediaJanela(IA_JANELA_2MIN);
    float media5  = iaMediaJanela(IA_JANELA_5MIN);
    float desvio2 = iaDesvioPadraoJanela(IA_JANELA_2MIN);
    float desvio5 = iaDesvioPadraoJanela(IA_JANELA_5MIN);

    bool base2 = (desvio2 <= IA_TOLERANCIA_BASE);
    bool base5 = (desvio5 <= IA_TOLERANCIA_BASE);

    if (base2 && media2 >= IA_BASELINE_MINIMO) {
        an.janelasEstavel++;
        if (an.janelasEstavel >= 5) {
            bool mudou = fabsf(media2 - an.baselineAtual) >
            IA_TOLERANCIA_BASE;
            if (mudou || an.baselineConfianca < 0.5f) {
                if (!an.emPico && base5 &&
                    media5 >= IA_BASELINE_MINIMO) {
                    float dur = (agora - an.tInicioBase) / 1000.0f;
                if (dur > 60) {
                    float ahPadrao = (media5 * dur) / 3600.0f;
                    // CORRIGIDO: usa media5+desvio como referencia de
                    // variacao real, em vez de duas amostras soltas
                    float margem   = desvio5 > 0.05f ? desvio5 : 0.05f;
                    float pseudoPico = media5 + margem;
                    float pseudoMin  = media5 - margem;
                    if (pseudoMin < 0) pseudoMin = 0;
                    iaRegistrarPadrao(PADRAO_BASELINE, media5,
                                      pseudoPico, pseudoMin,
                                      dur, ahPadrao);
                }
                    }
                    an.baselineAtual     = media2;
                    an.baselineConfianca = min(1.0f,
                                               an.baselineConfianca + 0.1f);
                    an.tInicioBase       = agora;
            }
            an.janelasEstavel = 0;
        }
    } else {
        an.janelasEstavel = 0;
    }

    float threshold = (an.baselineConfianca > 0.3f)
    ? (an.baselineAtual + IA_PICO_MINIMO)
    : (IA_PICO_MINIMO * 2.0f);
    bool ePico = (corr > threshold);

    if (ePico && !an.emPico) {
        an.emPico = true; an.picoAtual = corr;
        an.tInicioPico = agora; an.estado = IA_DETECTOU_PICO;
    } else if (ePico && an.emPico) {
        if (corr > an.picoAtual) an.picoAtual = corr;
    } else if (!ePico && an.emPico) {
        float dur   = (agora - an.tInicioPico) / 1000.0f;
        float mPico = iaMediaJanela(
            (uint16_t)min((int)dur, IA_JANELA_10MIN));
        float ahPico = (mPico * dur) / 3600.0f;

        TipoPadrao tipo;
        if (dur < 600)       tipo = PADRAO_TRANSITORIO;
        else if (dur < 3600) tipo = PADRAO_ALTO_LONGO;
        else                 tipo = PADRAO_CICLICO;

        if (an.baselineConfianca > 0.2f)
            iaRegistrarPadrao(tipo, mPico, an.picoAtual,
                              an.baselineAtual, dur, ahPico);
            an.emPico = false; an.estado = IA_OBSERVANDO;
    }
    an.correnteAnterior = corr;
}

// ═══════════════════════════════════════════════════════════
// [SIS-IA] MOTOR DE DECISÃO
// ═══════════════════════════════════════════════════════════

bool iaPodeDesligar(const char* carga) {
    unsigned long agora = millis();
    if (strcmp(carga, "lampExt") == 0) {
        if (ia.timersSeq.tDesligouLampExt &&
            agora - ia.timersSeq.tDesligouLampExt < IA_TEMPO_APOS_DESLIGAR)
            return false;
        if (ia.timersSeq.tLigouLampExt &&
            agora - ia.timersSeq.tLigouLampExt < IA_TEMPO_APOS_LIGAR)
            return false;
    } else if (strcmp(carga, "redeAlta") == 0) {
        if (ia.timersSeq.tDesligouRedeAlta &&
            agora - ia.timersSeq.tDesligouRedeAlta < IA_TEMPO_APOS_DESLIGAR)
            return false;
        if (ia.timersSeq.tLigouRedeAlta &&
            agora - ia.timersSeq.tLigouRedeAlta < IA_TEMPO_APOS_LIGAR)
            return false;
    } else if (strcmp(carga, "redeBaixa") == 0) {
        if (ia.timersSeq.tDesligouRedeBaixa &&
            agora - ia.timersSeq.tDesligouRedeBaixa < IA_TEMPO_APOS_DESLIGAR)
            return false;
        if (ia.timersSeq.tLigouRedeBaixa &&
            agora - ia.timersSeq.tLigouRedeBaixa < IA_TEMPO_APOS_LIGAR)
            return false;
    }
    return true;
}

bool iaPodeLigar(const char* carga) {
    unsigned long agora = millis();
    if (strcmp(carga, "lampExt") == 0) {
        if (ia.timersSeq.tLigouLampExt &&
            agora - ia.timersSeq.tLigouLampExt < IA_TEMPO_APOS_LIGAR)
            return false;
        if (ia.timersSeq.tDesligouLampExt &&
            agora - ia.timersSeq.tDesligouLampExt < IA_TEMPO_APOS_DESLIGAR)
            return false;
    } else if (strcmp(carga, "redeAlta") == 0) {
        if (ia.timersSeq.tLigouRedeAlta &&
            agora - ia.timersSeq.tLigouRedeAlta < IA_TEMPO_APOS_LIGAR)
            return false;
        if (ia.timersSeq.tDesligouRedeAlta &&
            agora - ia.timersSeq.tDesligouRedeAlta < IA_TEMPO_APOS_DESLIGAR)
            return false;
    } else if (strcmp(carga, "redeBaixa") == 0) {
        if (ia.timersSeq.tLigouRedeBaixa &&
            agora - ia.timersSeq.tLigouRedeBaixa < IA_TEMPO_APOS_LIGAR)
            return false;
        if (ia.timersSeq.tDesligouRedeBaixa &&
            agora - ia.timersSeq.tDesligouRedeBaixa < IA_TEMPO_APOS_DESLIGAR)
            return false;
    }
    return true;
}

// ── Estima Ah necessários até o amanhecer ─────────────────
float iaEstimarAhAteAmanhecer() {
    if (!bateria.calibrada) return 0;
    struct tm ti;
    if (!getLocalTime(&ti)) return 0;
    int hora = ti.tm_hour;
    if (!(hora >= 18 || hora < 6)) return 0;
    float horasAte = (hora >= 18) ? (24.0f - hora + 6.0f) : (6.0f - hora);
    float corrEff  = (ia.analise.baselineConfianca > 0.3f && !ia.analise.emPico)
                   ? ia.analise.baselineAtual : sensores.correnteCasa;
    return corrEff * horasAte; // Ah = A × h
}

bool iaCorrenteETransitoria() {
    if (!ia.analise.emPico || ia.totalPadroes == 0) return false;
    float corr    = sensores.correnteCasa;
    float duracao = (millis() - ia.analise.tInicioPico) / 1000.0f;
    for (int i = 0; i < IA_MAX_PADROES; i++) {
        if (!ia.padroes[i].valido) continue;
        if (ia.padroes[i].tipo != PADRAO_TRANSITORIO) continue;
        if (fabsf(ia.padroes[i].correnteMedia - corr) > 2.0f) continue;
        if (duracao <= ia.padroes[i].duracaoMax * 1.2f) return true;
    }
    return false;
}

float iaAhRestantePico() {
    if (!ia.analise.emPico) return 0;
    float corr    = sensores.correnteCasa;
    float duracao = (millis() - ia.analise.tInicioPico) / 1000.0f;
    for (int i = 0; i < IA_MAX_PADROES; i++) {
        if (!ia.padroes[i].valido) continue;
        if (ia.padroes[i].tipo != PADRAO_TRANSITORIO &&
            ia.padroes[i].tipo != PADRAO_ALTO_LONGO) continue;
        if (fabsf(ia.padroes[i].correnteMedia - corr) > 2.0f) continue;
        float durRest = max(0.0f, ia.padroes[i].duracaoMedia - duracao);
        return (corr * durRest) / 3600.0f; // Ah restantes
    }
    return (corr * 300.0f) / 3600.0f; // assume 5min
}

// ── Motor de decisão da IA ────────────────────────────────
void iaTomarDecisao() {
    // ── Modo manual: só limpa recomendações, não age ──────
    if (ia.config.modo != IA_MODO_IA) {
        if (ia.recomendacao.ativa &&
            millis() - ia.recomendacao.tCriado > 300000UL)
            iaLimparRecomendacao();
        memset(ia.decisao.motivo, 0, sizeof(ia.decisao.motivo));
        return;
    }

    if (!bateria.calibrada) return;
    if (curto.estado != CURTO_NORMAL) return;

    unsigned long agora = millis();

    // Zera as flags de acao - só o que for decidido aqui embaixo liga.
    ia.decisao.desligarLampExt   = false;
    ia.decisao.desligarRedeAlta  = false;
    ia.decisao.desligarRedeBaixa = false;
    ia.decisao.desligarLampInt   = false;
    ia.decisao.ligarLampExt      = false;
    ia.decisao.ligarRedeAlta     = false;
    ia.decisao.ligarRedeBaixa    = false;

    // ── Sem confianca suficiente + bateria folgada: só observa ──
    if (!iaTemConfiancaSuficiente() &&
        bateria.percentualAtual > redes.corteBaixa + 10.0f) {
        snprintf(ia.decisao.motivo, sizeof(ia.decisao.motivo),
                 "Aprendendo padroes (%d amostras) - bateria folgada, "
                 "sem acao preventiva ainda", ia.snapshot.total);
        if (ia.recomendacao.ativa &&
            millis() - ia.recomendacao.tCriado > 300000UL)
            iaLimparRecomendacao();
        return;
        }

        float saldo    = iaCalcularSaldoAh();
        float pct      = bateria.percentualAtual;
        bool  temDados = iaTeDadosSuficientes();

        // ── FALLBACK: sem dados de consumo + bateria já baixa ──
        if (!temDados && pct <= redes.corteBaixa + 5.0f) {
            ia.usandoFallbackManual = true;
            snprintf(ia.decisao.motivo, sizeof(ia.decisao.motivo),
                     "Fallback manual: bat=%.0f%% sem dados suficientes (%d amostras)",
                     pct, ia.snapshot.total);
            adicionarLog(ia.decisao.motivo);
            if (pct <= redes.corteAlta && estados.redesAlta &&
                iaPodeDesligar("redeAlta")) {
                ia.decisao.desligarRedeAlta = true;
            iaSetRecomendacao(2,
                              "IA: Rede Alta OFF (fallback - bateria baixa, poucos dados)",
                              ia.decisao.motivo);
            iaExecutarDecisoes();
                }
                return;
        }
        ia.usandoFallbackManual = false;

        // ════════════════════════════════════════════════════
        // SALDO POSITIVO: religar em sequencia (com anti-oscilacao real)
        // ════════════════════════════════════════════════════
        if (saldo > IA_HISTERESE_RELIGAR) {

            snprintf(ia.decisao.motivo, sizeof(ia.decisao.motivo),
                     "Saldo positivo: %+.1fAh (disp=%.1fAh nec=%.1fAh) - "
                     "mantendo/religando cargas",
                     saldo, ia.decisao.ahDisponivel, ia.decisao.ahEstimadoNecessario);

            // Inversor não faz parte da hierarquia de cargas (é infra).
            if (!estados.inversor &&
                iaPassouTempo(&ia.timersSeq.tLigouInversor, IA_INTERVALO_RELIGAR)) {
                ligarInversor();
            ia.timersSeq.tLigouInversor = agora;
            adicionarLog("IA: Inversor ON (religamento)");
            return;
                }

                if (estados.inversor && sensores.inversoreAtivo &&
                    !estados.redeBaixa && saldo > IA_HISTERESE_RELIGAR * 2.0f &&
                    iaPodeLigar("redeBaixa")) {
                    ia.decisao.ligarRedeBaixa = true;
                    }

                    if (estados.redeBaixa && sensores.inversoreAtivo &&
                        !estados.redesAlta && saldo > IA_HISTERESE_RELIGAR * 3.0f &&
                        iaPodeLigar("redeAlta")) {
                        ia.decisao.ligarRedeAlta = true;
                        }

                        if (!estados.lampExt && !ehDia &&
                            estados.inversor && sensores.inversoreAtivo &&
                            saldo > IA_HISTERESE_RELIGAR * 2.0f &&
                            iaPodeLigar("lampExt")) {
                            ia.decisao.ligarLampExt = true;
                            }

                            if (ia.sequencia != SEQ_OCIOSO) {
                                ia.sequencia = SEQ_OCIOSO;
                                iaLimparRecomendacao();
                                adicionarLog("IA: Saldo positivo - sequencia de corte encerrada");
                            }

                            iaExecutarDecisoes();
                            return;
        }

        // ════════════════════════════════════════════════════
        // SALDO OK MAS SEM FOLGA GRANDE: só monitora
        // ════════════════════════════════════════════════════
        if (saldo >= 0) {
            snprintf(ia.decisao.motivo, sizeof(ia.decisao.motivo),
                     "Saldo ok, margem baixa: %+.1fAh (disp=%.1fAh nec=%.1fAh)",
                     saldo, ia.decisao.ahDisponivel, ia.decisao.ahEstimadoNecessario);
            if (ia.recomendacao.ativa &&
                millis() - ia.recomendacao.tCriado > 300000UL)
                iaLimparRecomendacao();
            return;
        }

        // ════════════════════════════════════════════════════
        // SALDO NEGATIVO: age, em ordem de menor impacto
        // ════════════════════════════════════════════════════
        snprintf(ia.decisao.motivo, sizeof(ia.decisao.motivo),
                 "Deficit projetado: %+.1fAh (disp=%.1fAh nec=%.1fAh)",
                 saldo, ia.decisao.ahDisponivel, ia.decisao.ahEstimadoNecessario);

        float economiaLampExt = estados.lampExt ? 0.5f : 0.0f;

        // PASSO 1: recomendação (sem ação nos relés)
        if (ia.sequencia == SEQ_OCIOSO &&
            iaPassouTempo(&ia.timersSeq.tUltimaRecomend, IA_INTERVALO_RECOMEND)) {

            ia.timersSeq.tUltimaRecomend = agora;
        char bufRec[128], bufDet[128];
        snprintf(bufRec, sizeof(bufRec),
                 "&#128161; Economia necessaria: %.1fAh", fabsf(saldo));
        snprintf(bufDet, sizeof(bufDet),
                 "Dica: desligue lampadas internas que nao estiver usando "
                 "(bat=%.0f%% | saldo=%+.1fAh)", pct, saldo);
        iaSetRecomendacao(1, bufRec, bufDet);
        ia.sequencia = SEQ_RECOMENDOU_LAMP_INT;
        return;
            }

            // PASSO 2: Lâmpada Externa
            if (ia.sequencia == SEQ_RECOMENDOU_LAMP_INT &&
                estados.lampExt && !ehDia &&
                iaPassouTempo(&ia.timersSeq.tUltimaAcao, IA_INTERVALO_ACAO)) {

                float corrAtual  = iaMediaJanela(60);
            float corrAntes  = ia.analise.baselineAtual;
            bool  usuarioAgiu = (corrAntes > 0.5f && corrAtual < corrAntes * 0.85f);

            if ((!usuarioAgiu || saldo < -economiaLampExt) &&
                iaPodeDesligar("lampExt")) {
                ia.decisao.desligarLampExt = true;
            ia.timersSeq.tUltimaAcao   = agora;
            ia.sequencia = SEQ_DESLIGOU_LAMP_EXT;
            char bufRec[128], bufDet[128];
            snprintf(bufRec, sizeof(bufRec),
                     "&#128161; Lamp. Externa desligada (economia)");
            snprintf(bufDet, sizeof(bufDet),
                     "Saldo insuficiente: %+.1fAh | bat=%.0f%%", saldo, pct);
            iaSetRecomendacao(1, bufRec, bufDet);
                } else if (usuarioAgiu) {
                    ia.sequencia = SEQ_OCIOSO;
                    iaLimparRecomendacao();
                    adicionarLog("IA: usuario agiu - sequencia pausada");
                }
                iaExecutarDecisoes();
                return;
                }

                // PASSO 3: Rede Alta
                bool podePasso3 = (ia.sequencia == SEQ_DESLIGOU_LAMP_EXT ||
                (ia.sequencia == SEQ_RECOMENDOU_LAMP_INT && !estados.lampExt));

                if (podePasso3 && estados.redesAlta &&
                    iaPassouTempo(&ia.timersSeq.tUltimaAcao, IA_INTERVALO_ACAO)) {

                    saldo = iaCalcularSaldoAh(); // reavalia após desligar lamp ext

                    if (saldo < 0 && iaPodeDesligar("redeAlta")) {
                        ia.decisao.desligarRedeAlta = true;
                        ia.timersSeq.tUltimaAcao    = agora;
                        ia.sequencia = SEQ_DESLIGOU_REDE_ALTA;
                        char bufRec[128], bufDet[128];
                        snprintf(bufRec, sizeof(bufRec),
                                 "&#128268; Rede Alta desligada (economia)");
                        snprintf(bufDet, sizeof(bufDet),
                                 "Saldo ainda insuficiente: %+.1fAh | bat=%.0f%%", saldo, pct);
                        iaSetRecomendacao(2, bufRec, bufDet);
                    } else if (saldo >= 0) {
                        ia.sequencia = SEQ_OCIOSO;
                        iaLimparRecomendacao();
                    }
                    iaExecutarDecisoes();
                    return;
                    }

                    // PASSO 4: Rede Baixa + Inversor (só em crise real)
                    bool podePasso4 = (ia.sequencia == SEQ_DESLIGOU_REDE_ALTA ||
                    (ia.sequencia >= SEQ_DESLIGOU_LAMP_EXT && !estados.redesAlta));

                    if (podePasso4 && estados.redeBaixa &&
                        iaPassouTempo(&ia.timersSeq.tUltimaAcao, IA_INTERVALO_ACAO)) {

                        saldo = iaCalcularSaldoAh();
                    bool critico = (pct <= ia.config.reservaEmergencia + 5.0f ||
                    saldo < -(ia.ahNecAmanhecerAtual * 0.5f));

                    if (critico && iaPodeDesligar("redeBaixa")) {
                        if (estados.redesAlta) { desligarRedeAlta(); delay(200); }
                        desligarRedeBaixa();
                        ia.timersSeq.tDesligouRedeBaixa = agora;
                        if (estados.inversor) {
                            delay(500);
                            desligarInversor();
                            ia.timersSeq.tDesligouInversor = agora;
                        }
                        ia.timersSeq.tUltimaAcao = agora;
                        ia.sequencia = SEQ_DESLIGOU_REDE_BAIXA;
                        char bufRec[128], bufDet[128];
                        snprintf(bufRec, sizeof(bufRec),
                                 "&#9888; CRITICO: Rede Baixa + Inversor OFF");
                        snprintf(bufDet, sizeof(bufDet),
                                 "Bateria critica: %.0f%% | Saldo: %+.1fAh", pct, saldo);
                        iaSetRecomendacao(2, bufRec, bufDet);
                        adicionarLog("IA: Rede Baixa + Inversor OFF - CRITICO");
                    }
                        }
}

// ── Executa decisões da IA nos relés ─────────────────────
void iaExecutarDecisoes() {
    if (ia.config.modo != IA_MODO_IA) return;
    if (curto.estado != CURTO_NORMAL) return;

    unsigned long agora = millis();
    DecisaoIA& d = ia.decisao;

    if (d.desligarLampExt && estados.lampExt &&
        iaPodeDesligar("lampExt")) {
        desligarLampExt();
    ia.timersSeq.tDesligouLampExt = agora;
    adicionarLog("IA: Lamp.Ext OFF");
    adicionarLog(ia.decisao.motivo);
        }
        if (d.desligarRedeAlta && estados.redesAlta &&
            iaPodeDesligar("redeAlta")) {
            desligarRedeAlta();
        ia.timersSeq.tDesligouRedeAlta = agora;
        adicionarLog("IA: Rede Alta OFF");
        adicionarLog(ia.decisao.motivo);
            }
            if (d.desligarRedeBaixa && estados.redeBaixa &&
                iaPodeDesligar("redeBaixa")) {
                desligarRedeBaixa();
            ia.timersSeq.tDesligouRedeBaixa = agora;
            adicionarLog("IA: Rede Baixa OFF");
            adicionarLog(ia.decisao.motivo);
                }
                if (d.ligarRedeBaixa && !estados.redeBaixa &&
                    iaPodeLigar("redeBaixa") &&
                    estados.inversor && sensores.inversoreAtivo) {
                    ligarRedeBaixa();
                ia.timersSeq.tLigouRedeBaixa = agora;
                adicionarLog("IA: Rede Baixa ON");
                adicionarLog(ia.decisao.motivo);
                    }
                    if (d.ligarRedeAlta && !estados.redesAlta &&
                        iaPodeLigar("redeAlta") &&
                        estados.inversor && sensores.inversoreAtivo) {
                        ligarRedeAlta();
                    ia.timersSeq.tLigouRedeAlta = agora;
                    adicionarLog("IA: Rede Alta ON");
                    adicionarLog(ia.decisao.motivo);
                        }
                        if (d.ligarLampExt && !estados.lampExt &&
                            iaPodeLigar("lampExt") &&
                            estados.inversor && sensores.inversoreAtivo) {
                            ligarLampExt();
                        ia.timersSeq.tLigouLampExt = agora;
                        adicionarLog("IA: Lamp.Ext ON");
                        adicionarLog(ia.decisao.motivo);
                            }
}

// ── Loop da IA (chamado no loop principal) ────────────────
void iaLoop() {
    // IA SEMPRE monitora
    unsigned long agora = millis();

    if (agora - ia.ultimaDecisao >= IA_INTERVALO_DECISAO) {
        ia.ultimaDecisao = agora;
        iaTomarDecisao();
        iaExecutarDecisoes();
    }
    if (agora - ia.ultimoSalvamento >= 600000UL) {
        ia.ultimoSalvamento = agora;
        iaSalvarPadroes();
    }
}

// ═══════════════════════════════════════════════════════════
// [SIS-CURTO] PROTEÇÃO CONTRA CURTO-CIRCUITO
// ═══════════════════════════════════════════════════════════

void _desligarInversorComBuffer() {
    setRele(PIN_RELE_INV_ALTA, false);
    estados.inversor          = false;
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
    bool emProcesso = (curto.estado == CURTO_RELIGANDO      ||
                       curto.estado == CURTO_TESTANDO_ALTA  ||
                       curto.estado == CURTO_TESTANDO_BAIXA);

    if (!estados.inversor && !emProcesso) {
        if (curto.estado == CURTO_NORMAL) {
            curto.monitorando = false; curto.tInicio = 0;
            curto.tentativasReligar = 0;
        }
        return;
    }

    // ── Normal: monitora status ────────────────────────────
    if (curto.estado == CURTO_NORMAL) {
        bool statusAtual = lerStatusInversor();
        if (statusAtual) {
            curto.tInicio = 0; curto.monitorando = true;
            curto.tentativasReligar = 0;
            return;
        }
        if (curto.monitorando) {
            if (!curto.tInicio) {
                curto.tInicio = agora;
                adicionarLog("Inversor: status perdido - aguardando 1s...");
                return;
            }
            if (agora - curto.tInicio < T_STATUS_AUSENTE) return;
            adicionarLog("Inversor: sem status 1s - protecao ativa!");
            _desligarInversorComBuffer();
            curto.estado = CURTO_RELIGANDO; curto.subEstado = SUB_INVERSOR_DESLIGADO;
            curto.tInicio = agora;
        }
        return;
    }

    // ── Religando ─────────────────────────────────────────
    if (curto.estado == CURTO_RELIGANDO) {
        switch (curto.subEstado) {
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_PAUSA_RELIGAR) return;
                snprintf(tempBuf, sizeof(tempBuf),
                         "Religando tentativa %d/%d...",
                         curto.tentativasReligar+1, MAX_TENTATIVAS_RELIGAR);
                adicionarLog(tempBuf);
                _ligarInversorComBuffer();
                curto.tInicio = agora; curto.subEstado = SUB_AGUARDANDO_STATUS;
                break;

            case SUB_AGUARDANDO_STATUS: {
                bool st = lerStatusInversor();
                if (st && timers.inversorEstadoReal) {
                    curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                    curto.tInicio = 0; curto.monitorando = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Inversor OK apos religar!");
                    return;
                }
                if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    curto.tentativasReligar++;
                    _desligarInversorComBuffer();
                    curto.tInicio = agora;
                    if (curto.tentativasReligar < MAX_TENTATIVAS_RELIGAR) {
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    } else {
                        curto.estado = CURTO_TESTANDO_ALTA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        adicionarLog("=== DIAGNOSTICO DE CURTO ===");
                    }
                }
                break;
            }
            default:
                curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE; break;
        }
        return;
    }

    // ── Testando Rede Alta ─────────────────────────────────
    if (curto.estado == CURTO_TESTANDO_ALTA) {
        switch (curto.subEstado) {
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_AGUARDA_QUEDA) return;
                desligarRedeAlta(); delay(300);
                _ligarInversorComBuffer();
                curto.tInicio = agora; curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("Teste Alta: inversor SEM rede alta");
                break;

            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio = agora; curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("Teste Alta: inversor OK sem rede alta");
                } else if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    _desligarInversorComBuffer();
                    curto.estado = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio = agora;
                    adicionarLog("Teste Alta: sem resposta - testando rede baixa");
                }
                break;

            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        _desligarInversorComBuffer();
                        curto.estado = CURTO_TESTANDO_BAIXA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        curto.tInicio = agora;
                        adicionarLog("Teste Alta: instavel - testando rede baixa");
                    }
                    return;
                }
                adicionarLog("Teste Alta: 3s estavel - ligando rede alta...");
                ligarRedeAlta();
                curto.tInicio = agora; curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                break;

            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor() && agora - curto.tInicio > 500) {
                        adicionarLog("=== CURTO CONFIRMADO: REDE ALTA ===");
                        desligarRedeAlta(); delay(200);
                        _desligarInversorComBuffer(); delay(500);
                        _ligarInversorComBuffer(); delay(2000);
                        curto.estado = CURTO_CONFIRMADO_ALTA;
                        curto.redeAltaCurto = true;
                        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                        adicionarLog("Rede alta BLOQUEADA");
                    }
                    return;
                }
                if (lerStatusInversor()) {
                    curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                    curto.tInicio = 0; curto.monitorando = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Teste Alta: sistema OK");
                } else {
                    _desligarInversorComBuffer(); desligarRedeAlta();
                    curto.estado = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO; curto.tInicio = agora;
                }
                break;
            default: break;
        }
        return;
    }

    // ── Testando Rede Baixa ────────────────────────────────
    if (curto.estado == CURTO_TESTANDO_BAIXA) {
        switch (curto.subEstado) {
            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_AGUARDA_QUEDA) return;
                desligarRedeAlta(); desligarRedeBaixa(); delay(300);
                _ligarInversorComBuffer();
                curto.tInicio = agora; curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("Teste Baixa: inversor SEM cargas");
                break;

            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio = agora; curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("Teste Baixa: inversor OK sem cargas");
                } else if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    _desligarInversorComBuffer();
                    curto.estado = CURTO_AGUARDANDO_RESET;
                    curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                    adicionarLog("=== FALHA NO INVERSOR ===");
                    adicionarLog("Nao liga sem cargas - verifique hardware");
                }
                break;

            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        _desligarInversorComBuffer();
                        curto.estado = CURTO_AGUARDANDO_RESET;
                        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                        adicionarLog("=== INVERSOR INSTAVEL SEM CARGA ===");
                    }
                    return;
                }
                adicionarLog("Teste Baixa: 3s estavel - ligando rede baixa...");
                ligarRedeBaixa();
                curto.tInicio = agora; curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                break;

            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor() && agora - curto.tInicio > 500) {
                        adicionarLog("=== CURTO CONFIRMADO: REDE BAIXA ===");
                        desligarRedeBaixa(); delay(200);
                        _desligarInversorComBuffer(); delay(500);
                        curto.estado = CURTO_CONFIRMADO_BAIXA;
                        curto.redeBaixaCurto = true;
                        curto.subEstado = SUB_IDLE; curto.tInicio = 0;
                        adicionarLog("Rede baixa BLOQUEADA");
                    }
                    return;
                }
                if (lerStatusInversor()) {
                    ligarRedeAlta(); delay(300);
                    ligarLampExt(); delay(200); ligarLampInt();
                    curto.estado = CURTO_NORMAL; curto.subEstado = SUB_IDLE;
                    curto.tInicio = 0; curto.monitorando = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Teste Baixa: sistema OK - tudo religado");
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

    // ── Estados de proteção permanente ────────────────────
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
    adicionarLog("Reset curto rede alta");
    desligarRedeAlta(); delay(500); ligarRedeAlta(); delay(300);
    if (lerStatusInversor()) {
        curto.estado = CURTO_NORMAL; curto.redeAltaCurto = false;
        curto.subEstado = SUB_IDLE; curto.tInicio = 0; curto.monitorando = true;
        adicionarLog("Curto rede alta RESOLVIDO");
    } else {
        desligarRedeAlta();
        adicionarLog("AVISO: Curto rede alta NAO resolvido!");
    }
}

void resetCurtoBaixa() {
    if (curto.estado != CURTO_CONFIRMADO_BAIXA &&
        curto.estado != CURTO_AGUARDANDO_RESET) return;
    adicionarLog("Reset curto rede baixa");
    desligarRedeBaixa(); desligarInversor(); delay(500);
    ligarInversor();
    unsigned long t = millis();
    bool invOK = false;
    while (millis() - t < T_ESPERA_STATUS_INV) {
        if (lerStatusInversor()) { invOK = true; break; }
        delay(100);
    }
    if (!invOK) {
        desligarInversor();
        adicionarLog("ERRO: Inversor nao ligou!");
        return;
    }
    ligarRedeBaixa(); delay(500);
    if (lerStatusInversor()) {
        curto.estado = CURTO_NORMAL; curto.redeBaixaCurto = false;
        curto.subEstado = SUB_IDLE; curto.tInicio = 0; curto.monitorando = true;
        adicionarLog("Curto rede baixa RESOLVIDO");
    } else {
        desligarRedeBaixa(); desligarInversor();
        curto.estado = CURTO_CONFIRMADO_BAIXA;
        adicionarLog("AVISO: Curto rede baixa NAO resolvido!");
    }
}

String htmlStatusCurto() {
    String h = "";
    if (curto.estado == CURTO_NORMAL && !curto.redeAltaCurto && !curto.redeBaixaCurto) {
        h += "<div class='ok' style='font-size:12px'>&#10003; Sistema sem curto</div>";
        return h;
    }
    if (curto.estado == CURTO_TESTANDO_ALTA || curto.estado == CURTO_TESTANDO_BAIXA) {
        h += "<div class='wn'>&#9874; Diagnosticando curto... Aguarde.</div>";
        return h;
    }
    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        h += "<div class='er'><b>&#9888; CURTO NA REDE ALTA!</b><br>";
        h += "Rede alta desligada por protecao.<br><br>";
        h += "<b>1.</b> Resolva o curto.<br>";
        h += "<b>2.</b> Pressione o botao abaixo.</div>";
        h += "<form method='POST' action='/reset-curto'>";
        h += "<input type='hidden' name='rede' value='alta'>";
        h += "<button type='submit' class='btn-g' style='margin-top:8px'>";
        h += "&#10003; Resolvido - Testar e Religar</button></form>";
        return h;
    }
    if (curto.estado == CURTO_CONFIRMADO_BAIXA) {
        h += "<div class='er'><b>&#9888; CURTO NA REDE BAIXA!</b><br>";
        h += "Rede baixa e inversor desligados.<br><br>";
        h += "<b>1.</b> Resolva o curto.<br>";
        h += "<b>2.</b> Pressione o botao abaixo.</div>";
        h += "<form method='POST' action='/reset-curto'>";
        h += "<input type='hidden' name='rede' value='baixa'>";
        h += "<button type='submit' class='btn-g' style='margin-top:8px'>";
        h += "&#10003; Resolvido - Testar e Religar</button></form>";
        return h;
    }
    if (curto.estado == CURTO_AGUARDANDO_RESET) {
        h += "<div class='er'><b>&#9888; FALHA NO INVERSOR!</b><br>";
        h += "Nao liga sem cargas - verifique hardware.</div>";
        h += "<form method='POST' action='/reset-curto'>";
        h += "<input type='hidden' name='rede' value='inversor'>";
        h += "<button type='submit' class='btn-o' style='margin-top:8px'>";
        h += "&#9889; Tentar Religar</button></form>";
    }
    return h;
}

// ═══════════════════════════════════════════════════════════
// [SIS-PREF] PREFERENCES - SALVAR/CARREGAR CONFIGURAÇÕES
// ═══════════════════════════════════════════════════════════

bool existeCredenciais() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("user");
    prefs.end(); return ok;
}
bool existeWiFiConfig() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("wifi_ok");
    prefs.end(); return ok;
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
    return (su == u && sp == p && su.length() > 0);
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
    prefs.putFloat("bat_vnom",   bateria.tensaoNominal);
    prefs.putFloat("bat_ah",     bateria.capacidadeAh);
    prefs.putFloat("bat_desc",   bateria.descargaSegura);
    prefs.putFloat("bat_ah_at",  bateria.ahAtual);
    prefs.putBool ("bat_calib",  bateria.calibrada);
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
        bateria.percentualAtual = (bateria.ahAtual / bateria.ahDisponivel) * 100.0f;
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
void carregarCalibSensores() {
    // ── Valores de fábrica (fallback) ──────────────────────
    calibPainel.offsetMV  = PAINEL_OFFSET_MV;
    calibPainel.sensMvA   = 35.9f;
    calibPainel.invertido = true;
    calibPainel.calibrado = true;

    calibCasa.offsetMV  = 541;
    calibCasa.sensMvA   = 43.3f;
    calibCasa.invertido = false;
    calibCasa.calibrado = true;

    // ── Sobrescreve com calibração salva pelo usuario (se existir) ──
    prefs.begin("rrfs_cal", true);
    if (prefs.isKey("pn_off"))
        calibPainel.offsetMV = prefs.getInt  ("pn_off", calibPainel.offsetMV);
    if (prefs.isKey("pn_sens"))
        calibPainel.sensMvA  = prefs.getFloat("pn_sens", calibPainel.sensMvA);
    if (prefs.isKey("casa_off"))
        calibCasa.offsetMV   = prefs.getInt  ("casa_off", calibCasa.offsetMV);
    if (prefs.isKey("casa_sens"))
        calibCasa.sensMvA    = prefs.getFloat("casa_sens", calibCasa.sensMvA);
    autoCalibPainelAtivo = prefs.getBool("auto_cal", true);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "Sensor Painel: off=%dmV sens=%.1fmV/A autoCal=%s",
             calibPainel.offsetMV, calibPainel.sensMvA,
             autoCalibPainelAtivo ? "ON" : "OFF");
    adicionarLog(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf),
             "Sensor Casa: off=%dmV sens=%.1fmV/A",
             calibCasa.offsetMV, calibCasa.sensMvA);
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [SIS-WIFI] REDE WIFI E RECONEXÃO
// ═══════════════════════════════════════════════════════════

void sincronizarNTP() {
    if (WiFi.status() != WL_CONNECTED) {
        adicionarLog("NTP: sem WiFi"); return;
    }
    configTime(-3*3600, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
    adicionarLog("NTP: sincronizando...");
    struct tm ti; unsigned long t = millis(); bool ok = false;
    while (millis() - t < 5000) {
        if (getLocalTime(&ti)) { ok = true; break; }
        delay(200);
    }
    if (ok) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "NTP OK: %02d:%02d:%02d %02d/%02d/%04d",
                 ti.tm_hour, ti.tm_min, ti.tm_sec,
                 ti.tm_mday, ti.tm_mon+1, ti.tm_year+1900);
        adicionarLog(tempBuf);
    } else {
        adicionarLog("NTP: falhou - usando millis()");
    }
}

void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP ativo: RRFS Energia | IP: 10.0.0.1");
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

    gWifiSSID = ssid; gWifiPASS = pass;
    gWifiIP   = ip;   gWifiGW   = gw;
    gWifiMASK = mask; gWifiDNS  = dns;

    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(300);
    WiFi.mode(WIFI_STA); WiFi.setSleep(false);
    if (!WiFi.config(lIP, lGW, lMask, lDNS)) {
        adicionarLog("ERRO: Config IP falhou"); return false;
    }
    WiFi.setAutoReconnect(true); WiFi.persistent(false);

    snprintf(tempBuf, sizeof(tempBuf),
             "Conectando: %s | IP: %s", ssid.c_str(), ip.c_str());
    adicionarLog(tempBuf);
    ledBlink();
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long t = millis(); int tent = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
        atualizarLED(); delay(500); tent++;
        if (tent % 6 == 0) {
            snprintf(tempBuf, sizeof(tempBuf), "Aguardando WiFi... %ds", tent/2);
            adicionarLog(tempBuf);
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi OK! IP: %s RSSI: %ddBm",
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
        adicionarLog(tempBuf);
        logRAM(); modoAP = false; ledFixo(true);
        sincronizarNTP(); return true;
    }

    snprintf(tempBuf, sizeof(tempBuf), "WiFi FALHOU status: %d", WiFi.status());
    adicionarLog(tempBuf);
    return false;
}

void tentarReconectarWiFi() {
    static bool emRecon = false;
    static uint8_t tentativas = 0;
    if (emRecon) return;
    emRecon = true; tentativas++;

    snprintf(tempBuf, sizeof(tempBuf), "WiFi perdido - tentativa %d/5", tentativas);
    adicionarLog(tempBuf);

    if (!gWifiIP.isEmpty()) {
        IPAddress lIP, lGW, lMask, lDNS;
        lIP.fromString(gWifiIP); lGW.fromString(gWifiGW);
        lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
        WiFi.config(lIP, lGW, lMask, lDNS);
    }
    WiFi.reconnect();

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        atualizarLED(); server.handleClient();
        if (modoAP) dnsServer.processNextRequest();
        delay(200);
    }

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi reconectado! RSSI: %ddBm", WiFi.RSSI());
        adicionarLog(tempBuf);
        tentativas = 0; ledFixo(true);
        if (WiFi.getMode() == WIFI_AP_STA) {
            delay(1000); WiFi.mode(WIFI_STA); WiFi.setSleep(false);
            if (!gWifiIP.isEmpty()) {
                IPAddress lIP, lGW, lMask, lDNS;
                lIP.fromString(gWifiIP); lGW.fromString(gWifiGW);
                lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
                WiFi.config(lIP, lGW, lMask, lDNS);
            }
            modoAP = false; server.close(); delay(200); configurarRotas();
        }
        sincronizarNTP(); emRecon = false; return;
    }

    if (tentativas >= 5) {
        adicionarLog("5 tentativas falharam - ativando AP temporario");
        tentativas = 0;
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK); WiFi.softAP(AP_SSID);
        dnsServer.start(DNS_PORT, "*", AP_IP);
        if (!gWifiIP.isEmpty()) {
            IPAddress lIP, lGW, lMask, lDNS;
            lIP.fromString(gWifiIP); lGW.fromString(gWifiGW);
            lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
            WiFi.config(lIP, lGW, lMask, lDNS);
        }
        modoAP = true; ledBlink();
        server.close(); delay(200); configurarRotas();
        adicionarLog("AP temporario: RRFS Energia | 10.0.0.1");
    }
    emRecon = false;
}

// ═══════════════════════════════════════════════════════════
// [SIS-SETUP] INICIALIZAÇÃO DO SISTEMA v15.0
// ═══════════════════════════════════════════════════════════
void setup() {

    // ── Verifica e repara NVS corrompida ──────────────────
    esp_err_t errNVS = nvs_flash_init();
    if (errNVS == ESP_ERR_NVS_NO_FREE_PAGES ||
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
    errNVS = nvs_flash_init();
        }

        // ── Detecta causa do reset anterior ───────────────────
        esp_reset_reason_t motivoReset = esp_reset_reason();
        if (motivoReset == ESP_RST_BROWNOUT) {
            adicionarLog("!!! RESET ANTERIOR: BROWNOUT !!!");
            adicionarLog("Queda de tensao na alimentacao do ESP32.");
            adicionarLog("Verifique fonte e cabeamento dos reles.");
        } else if (motivoReset == ESP_RST_PANIC) {
            adicionarLog("!!! RESET ANTERIOR: PANIC/CRASH !!!");
        } else if (motivoReset == ESP_RST_TASK_WDT) {
            adicionarLog("!!! RESET ANTERIOR: WATCHDOG !!!");
        }

        // ── Hardware básico ────────────────────────────────────
        setupLED();
        setupBotao();
        setupReles();
        ledBlink();

        adicionarLog("RRFS Energia v15.0");
        adicionarLog("Foco: corrente real (ACS758) | Ah em vez de Wh");
        if (errNVS != ESP_OK)
            adicionarLog("AVISO: NVS reparada apos corrupcao");
    logRAM();

    // ── Carrega calibração dos sensores ───────────────────
    // v15: usa valores calibrados de 2 pontos (removida estimativa de tensão)
    carregarCalibSensores();

    // ── Carrega configurações da flash ─────────────────────
    carregarConfigBateria();
    carregarConfigRedes();
    carregarContadores();
    carregarHistorico();
    carregarConfigLocal();
    adicionarLog("Configs carregadas");

    // ── Calcula parâmetros do banco ────────────────────────
    calcularBateria();

    // ── Log resumo de configuração ─────────────────────────
    snprintf(tempBuf, sizeof(tempBuf),
             "Bat: %.1fV nom | %.0fAh | %.0f%% desc | %.1fAh disp | Calib:%s",
             bateria.tensaoNominal,
             bateria.capacidadeAh,
             bateria.descargaSegura,
             bateria.ahDisponivel,
             bateria.calibrada ? "SIM" : "NAO");
    adicionarLog(tempBuf);

    snprintf(tempBuf, sizeof(tempBuf),
             "Local: amp ctrl=%.0fA ef=%.0f%% conf=%s",
             configLocal.ampControlador,
             configLocal.eficiencia * 100.0f,
             configLocal.configurado ? "SIM" : "NAO");
    adicionarLog(tempBuf);

    // ── Inicializa Coulomb Counting ────────────────────────
    ultimoContagem = millis();
    if (bateria.calibrada) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "Coulomb Counting iniciado: %.1fAh (%.0f%%)",
                 bateria.ahAtual, bateria.percentualAtual);
        adicionarLog(tempBuf);
    }

    // ── Inicializa IA ──────────────────────────────────────
    iaInicializar();

    // ══════════════════════════════════════════════════════
    // SEQUÊNCIA DE INICIALIZAÇÃO DAS CARGAS
    // ══════════════════════════════════════════════════════

    // 1. Rede baixa (alimenta roteador WiFi)
    ligarRedeBaixa();
    adicionarLog("Rede Baixa ON");

    // 2. Aguarda roteador
    adicionarLog("Aguardando roteador 8s...");
    {
        unsigned long t = millis();
        while (millis() - t < 8000) { atualizarLED(); delay(10); }
    }
    adicionarLog("Roteador pronto");

    // 3. Liga inversor
    ligarInversor();
    adicionarLog("Inversor ON");

    // 4. Aguarda boot do inversor
    adicionarLog("Aguardando inversor 5s...");
    {
        unsigned long t = millis();
        while (millis() - t < 5000) { atualizarLED(); delay(10); }
    }

    // 5. Verifica status do inversor
    {
        bool invOK = false;
        unsigned long t = millis();
        while (millis() - t < 3000) {
            if (lerStatusInversor()) { invOK = true; break; }
            delay(200);
        }
        adicionarLog(invOK ? "Inversor STATUS: OK"
        : "AVISO: Inversor sem status - continuando");
    }

    // 6. Liga rede alta
    ligarRedeAlta();
    adicionarLog("Rede Alta ON");
    delay(500);

    // 7. Liga lâmpadas
    ligarLampExt();
    adicionarLog("Lamp. Externa ON");
    delay(200);
    ligarLampInt();
    adicionarLog("Lamp. Interna ON");

    // ── Inicializa monitoramento de curto ─────────────────
    curto.monitorando       = false;
    curto.estado            = CURTO_NORMAL;
    curto.subEstado         = SUB_IDLE;
    curto.tInicio           = 0;
    curto.tentativasReligar = 0;
    curto.redeAltaCurto     = false;
    curto.redeBaixaCurto    = false;

    logRAM();

    // ══════════════════════════════════════════════════════
    // CONFIGURAÇÃO DE REDE WiFi
    // ══════════════════════════════════════════════════════
    bool temCred = existeCredenciais();
    bool temWifi = existeWiFiConfig();

    if (!temCred || !temWifi) {
        adicionarLog("Setup inicial - modo AP");
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

    // ── Servidor web ───────────────────────────────────────
    configurarRotas();
    adicionarLog("Servidor web ATIVO");

    // ── Primeira leitura dos sensores ─────────────────────
    lerSensores();

    snprintf(tempBuf, sizeof(tempBuf),
             "1a leitura: Painel=%.1fA Casa=%.1fA Bat=%.1fA",
             sensores.correntePainel,
             sensores.correnteCasa,
             sensores.correnteBateria);
    adicionarLog(tempBuf);

    // ── Inicializa buffer histórico de corrente ───────────
    for (int i = 0; i < HIST_SNAP_PONTOS; i++) {
        histCorrPainel[i] = 0;
        histCorrCasa[i]   = 0;
        histCorrBat[i]    = 0;
    }
    ultimoHistSnap = millis();

    adicionarLog("Sistema v15 pronto!");
    logRAM();
}

// ═══════════════════════════════════════════════════════════
// [SIS-LOOP] CICLO PRINCIPAL v15.0
// ═══════════════════════════════════════════════════════════
void loop() {
    verificarReset();
    atualizarLED();

    if (modoAP) dnsServer.processNextRequest();
    server.handleClient();

    unsigned long agora = millis();

    // ── Leitura de sensores a cada 1s ─────────────────────
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;
        lerSensores();      // inclui iaAnalisar() e atualizarAhBateria()
        acumularEnergia();
    }

    // ── Monitoramento de curto (todo ciclo) ───────────────
    {
        bool emProcesso = (curto.estado == CURTO_RELIGANDO      ||
        curto.estado == CURTO_TESTANDO_ALTA  ||
        curto.estado == CURTO_TESTANDO_BAIXA);
        if (estados.inversor || emProcesso) {
            monitorarInversor();
        } else {
            if (curto.estado == CURTO_NORMAL && curto.monitorando) {
                curto.monitorando       = false;
                curto.tInicio           = 0;
                curto.tentativasReligar = 0;
            }
        }
    }

    // ── Controle de energia a cada 5s ─────────────────────
    // IMPORTANTE: quando a IA esta ATIVA, ela assume 100% do controle
    // de Rede Alta/Baixa/Lampadas via saldo em Ah. O corte por % (manual)
    // so roda no Modo Manual, pra nao competir com a IA pelos reles.
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;
        bool emProcesso = (curto.estado == CURTO_RELIGANDO      ||
        curto.estado == CURTO_TESTANDO_ALTA  ||
        curto.estado == CURTO_TESTANDO_BAIXA);
        if (!emProcesso) {
            if (curto.estado == CURTO_NORMAL ||
                curto.estado == CURTO_CONFIRMADO_ALTA) {
                controlarEnergia();
                }
        }
    }

    // ── Restauração não-bloqueante (todo ciclo) ───────────
    processarRestauroEnergia();

    // ── IA: decisão a cada 30s ────────────────────────────
    iaLoop();

    // ── Gerenciamento WiFi ────────────────────────────────
    {
        static unsigned long tWifiCheck = 0;
        static unsigned long tWifiRecon = 0;
        static bool          eraAP_STA  = false;

        if (!gWifiSSID.isEmpty() && agora - tWifiCheck >= 15000) {
            tWifiCheck = agora;

            if (WiFi.status() != WL_CONNECTED) {
                if (agora - tWifiRecon >= 30000) {
                    tWifiRecon = agora;
                    tentarReconectarWiFi();
                }
            } else {
                if (modoAP && eraAP_STA) {
                    adicionarLog("WiFi restaurado - desativando AP temp");
                    IPAddress lIP, lGW, lMask, lDNS;
                    lIP.fromString(gWifiIP);   lGW.fromString(gWifiGW);
                    lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
                    WiFi.config(lIP, lGW, lMask, lDNS);
                    WiFi.mode(WIFI_STA);
                    WiFi.setSleep(false);
                    modoAP = false; eraAP_STA = false;
                    ledFixo(true);
                    server.close(); delay(200);
                    configurarRotas();
                    snprintf(tempBuf, sizeof(tempBuf),
                             "WiFi OK: %s %ddBm",
                             WiFi.SSID().c_str(), WiFi.RSSI());
                    adicionarLog(tempBuf);
                    sincronizarNTP();
                }
                if (modoAP && WiFi.getMode() == WIFI_AP_STA)
                    eraAP_STA = true;

                static unsigned long tNTP = 0;
                if (agora - tNTP >= 86400000UL) {
                    tNTP = agora;
                    adicionarLog("NTP: ressinc. diaria");
                    sincronizarNTP();
                }

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
    }

    // ── Salva Ah da bateria a cada 5min ───────────────────
    {
        static unsigned long tSalva = 0;
        if (bateria.calibrada && agora - tSalva >= 300000) {
            tSalva = agora;
            prefs.begin("rrfs", false);
            prefs.putFloat("bat_ah_at", bateria.ahAtual);
            prefs.end();
        }
    }

    // ── Previsão meteorológica a cada 1h ──────────────────
    if (configLocal.configurado       &&
        WiFi.status() == WL_CONNECTED &&
        !buscandoPrev                 &&
        (ultimaPrev == 0 || agora - ultimaPrev >= PREV_INTERVALO)) {
        buscarPrevisaoMeteo();
        }

        // ── Calibração e decisões a cada 5min ─────────────────
        // tomarDecisoes()/verificarReligamentoAuto() sao o motor "antigo"
        // por horario noturno/diurno. So devem tocar reles no Modo Manual;
        // no Modo IA quem decide e exclusivamente iaTomarDecisao().
        {
            static unsigned long tCalib = 0;
            if (agora - tCalib >= 300000) {
                tCalib = agora;
                calibrarPrevisao();
                if (ia.config.modo == IA_MODO_MANUAL) {
                    verificarReligamentoAuto();
                    tomarDecisoes();
                }
            }
        }

        // ── Antecipação Rede Alta a cada 10s ──────────────────
        // Logica exclusiva do Modo Manual - no Modo IA o proprio
        // iaTomarDecisao ja decide religamento antecipado usando o
        // mesmo saldo em Ah + previsao meteorologica.
        {
            static unsigned long tAntecip = 0;
            if (agora - tAntecip >= 10000) {
                tAntecip = agora;
                if (ia.config.modo == IA_MODO_MANUAL) {
                    avaliarAntecipacaoRedeAlta();
                }
            }
        }

        // ── Log RAM a cada 10min ──────────────────────────────
        {
            static unsigned long tRAM = 0;
            if (agora - tRAM >= 600000) {
                tRAM = agora;
                logRAM();
            }
        }

        delay(1);
}

// ═══════════════════════════════════════════════════════════
// [SIS-WEB] HELPERS HTML E CSS
// ═══════════════════════════════════════════════════════════

String gerarPagina(const String& titulo, const String& conteudo, bool comNav) {
    String html = F("<!DOCTYPE html><html><head>"
                    "<meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>"
        "body{background:#0d0d0d;color:#e0e0e0;font-family:Arial,sans-serif;margin:0;padding:10px;}"
        ".card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:15px;"
               "max-width:520px;margin:0 auto;}"
        "h2{color:#4fc3f7;text-align:center;margin:0 0 15px 0;font-size:18px;}"
        "h3{color:#81d4fa;font-size:14px;margin:12px 0 8px;"
           "border-bottom:1px solid #333;padding-bottom:6px;}"
        "label{display:block;color:#90a4ae;font-size:12px;"
              "margin-top:10px;margin-bottom:3px;}"
        "input,select{width:100%;padding:9px;background:#111;border:1px solid #444;"
                     "border-radius:4px;color:#e0e0e0;font-size:13px;box-sizing:border-box;}"
        "input:focus,select:focus{outline:none;border-color:#0277bd;}"
        "button{width:100%;padding:11px;margin-top:12px;background:#0277bd;"
               "color:#fff;border:none;border-radius:4px;font-size:14px;"
               "font-weight:bold;cursor:pointer;}"
        "button:hover{background:#039be5;}"
        ".btn-r{background:#c62828;}.btn-r:hover{background:#ef5350;}"
        ".btn-g{background:#2e7d32;}.btn-g:hover{background:#43a047;}"
        ".btn-o{background:#ef6c00;}.btn-o:hover{background:#f57c00;}"
        ".btn-s{background:#546e7a;}.btn-s:hover{background:#607d8b;}"
        ".btn-sm{font-size:12px;padding:8px;}"
        ".ok{background:#1b5e20;color:#a5d6a7;padding:10px;border-radius:4px;"
            "margin:8px 0;text-align:center;}"
        ".er{background:#b71c1c;color:#ef9a9a;padding:10px;border-radius:4px;"
            "margin:8px 0;text-align:center;}"
        ".wn{background:#e65100;color:#ffe0b2;padding:10px;border-radius:4px;"
            "margin:8px 0;text-align:center;}"
        ".nf{background:#0d47a1;color:#90caf9;padding:10px;border-radius:4px;"
            "margin:8px 0;font-size:12px;line-height:1.5;}"
        ".row{display:flex;gap:8px;margin-top:12px;}"
        ".row button{margin-top:0;}"
        ".nav{display:flex;gap:4px;margin-top:12px;padding-top:10px;"
             "border-top:1px solid #333;}"
        ".nav a{flex:1;text-align:center;padding:9px 4px;background:#1e1e1e;"
               "color:#90a4ae;text-decoration:none;border-radius:4px;"
               "font-size:11px;border:1px solid #333;}"
        ".nav a.active{background:#0277bd;color:#fff;border-color:#0277bd;}"
        ".nav a:hover{background:#263238;color:#e0e0e0;}"
        ".sc{background:#111;border:1px solid #333;border-radius:6px;"
            "padding:10px;margin:6px 0;}"
        ".sc .lbl{color:#546e7a;font-size:11px;margin-bottom:4px;}"
        ".sc .val{color:#e0e0e0;font-size:20px;font-weight:bold;}"
        ".sc .sub{color:#78909c;font-size:11px;margin-top:2px;}"
        ".sc.ok{border-color:#2e7d32;background:#0a1f0a;}"
        ".sc.wn{border-color:#ef6c00;background:#1f130a;}"
        ".sc.er{border-color:#c62828;background:#1f0a0a;}"
        ".barra-bg{background:#111;border:1px solid #333;border-radius:4px;"
                  "height:18px;margin:6px 0;overflow:hidden;}"
        ".barra{height:100%;border-radius:4px;transition:width .5s;"
               "text-align:center;line-height:18px;font-size:11px;"
               "font-weight:bold;color:#fff;}"
        ".log-t{color:#546e7a;font-size:10px;text-transform:uppercase;"
               "margin-top:15px;margin-bottom:4px;}"
        ".log-box{background:#000;border:1px solid #222;border-radius:4px;"
                 "padding:8px;height:160px;overflow-y:auto;"
                 "font-family:monospace;font-size:10px;color:#76ff03;}"
        ".log-l{padding:2px 0;border-bottom:1px solid #0a0a0a;}"
        ".cg{border-radius:8px;border:2px solid #222;background:#111;"
            "padding:9px 11px;margin-bottom:5px;}"
        ".cg-row{display:flex;justify-content:space-between;align-items:center;}"
        ".cg-nm{font-size:12px;color:#90a4ae;}"
        ".cg-st{font-size:13px;font-weight:bold;}"
        ".cg-inf{font-size:10px;color:#546e7a;margin-top:5px;padding-top:5px;"
                "border-top:1px solid #1e1e1e;}"
    "</style></head><body><div class='card'>");

    html += "<h2>&#9889; RRFS Energia v15</h2>";
    html += conteudo;

    if (comNav && logado) {
        String pag = server.uri();
        html += "<div class='nav'>";
        html += "<a href='/'          " + String(pag=="/"?         "class='active'":"") + ">&#128202; Painel</a>";
        html += "<a href='/bateria'   " + String(pag=="/bateria"?  "class='active'":"") + ">&#128267; Bateria</a>";
        html += "<a href='/redes'     " + String(pag=="/redes"?    "class='active'":"") + ">&#128268; Redes</a>";
        html += "<a href='/sensores'  " + String(pag=="/sensores"? "class='active'":"") + ">&#128270; Sens.</a>";
        html += "<a href='/ia'        " + String(pag=="/ia"?       "class='active'":"") + ">&#129302; IA</a>";
        html += "<a href='/config'    " + String(pag=="/config"?   "class='active'":"") + ">&#9881; Config</a>";
        html += "<a href='/historico' " + String(pag=="/historico"?"class='active'":"") + ">&#128200; Hist.</a>";
        html += "<a href='/previsao'  " + String(pag=="/previsao"? "class='active'":"") + ">&#127774; Prev.</a>";
        html += "</div>";
    }

    html += F("<div class='log-t'>&#128221; Logs do sistema</div>"
              "<div class='log-box' id='lb'>"
              "<div style='color:#666'>Carregando...</div></div>"
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
              "for(var i=0;i<a.length;i++)"
              "{h+='<div class=\"log-l\">'+a[i]+'</div>';}"
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
    String cor = pct >= 50 ? "#2e7d32" : pct >= 25 ? "#ef6c00" : "#c62828";
    String html = "<div class='barra-bg'><div class='barra' style='width:";
    html += String((int)pct) + "%;background:" + cor + "'>";
    html += String((int)pct) + "%</div></div>";
    if (label.length())
        html += "<div style='font-size:11px;color:#546e7a;text-align:center'>"
                + label + "</div>";
    return html;
}

String statusCard(const String& icone, const String& label,
                  const String& valor,  const String& sub,
                  const String& tipo) {
    String cls = "sc";
    if (tipo.length()) cls += " " + tipo;
    String h = "<div class='" + cls + "'>";
    h += "<div class='lbl'>" + icone + " " + label + "</div>";
    h += "<div class='val'>" + valor + "</div>";
    if (sub.length()) h += "<div class='sub'>" + sub + "</div>";
    h += "</div>";
    return h;
}

// ═══════════════════════════════════════════════════════════
// [SIS-AUTH] AUTENTICAÇÃO E SETUP INICIAL
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
        String user = server.arg("user");
        String pass = server.arg("pass");
        String conf = server.arg("conf");
        if (user.length() < 3)
            msg = "<div class='er'>&#10007; Minimo 3 caracteres no usuario</div>";
        else if (pass.length() < 4)
            msg = "<div class='er'>&#10007; Minimo 4 caracteres na senha</div>";
        else if (pass != conf)
            msg = "<div class='er'>&#10007; Senhas nao conferem</div>";
        else {
            salvarCredenciais(user, pass);
            adicionarLog("Credenciais criadas");
            server.sendHeader("Location","/setup-wifi"); server.send(302); return;
        }
    }
    String c = "<div class='nf'>&#8505; Primeira configuracao. Crie suas credenciais.</div>";
    c += "<h3>&#128274; Criar Conta</h3>" + msg;
    c += "<form method='POST' action='/setup-login'>";
    c += "<label>Usuario (min. 3 caracteres):</label>";
    c += "<input type='text' name='user' maxlength='32' required "
         "autocomplete='off' placeholder='Usuario'>";
    c += "<label>Senha (min. 4 caracteres):</label>";
    c += "<input type='password' name='pass' maxlength='32' required placeholder='Senha'>";
    c += "<label>Confirmar Senha:</label>";
    c += "<input type='password' name='conf' maxlength='32' required "
         "placeholder='Confirmar senha'>";
    c += "<button type='submit'>Criar e Continuar &#8594;</button></form>";
    server.send(200,"text/html; charset=utf-8",
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
            adicionarLog("Modo AP salvo");
            String cont = "<div class='ok'>&#10003; Salvo! Reiniciando em 5s...</div>";
            cont += "<div class='nf'>SSID: <b>RRFS Energia</b><br>IP: <b>10.0.0.1</b></div>";
            server.send(200,"text/html; charset=utf-8",
                        gerarPagina("RRFS - Salvo", cont, false));
            delay(5000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"), wpass=server.arg("wpass");
        String ip=server.arg("ip"),     gw=server.arg("gw");
        String mask=server.arg("mask"), dns=server.arg("dns");
        if (ssid.length()==0)
            msg = "<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7)
            msg = "<div class='er'>Informe o IP fixo</div>";
        else if (gw.length()<7)
            msg = "<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wpass,ip,gw,mask,dns,false);
            adicionarLog(("WiFi salvo: "+ssid).c_str());
            String cont = "<div class='ok'>&#10003; Salvo! Acesse: http://"+ip+"</div>";
            cont += "<div class='nf'>Reiniciando em 5s...</div>";
            server.send(200,"text/html; charset=utf-8",
                        gerarPagina("RRFS - Salvo", cont, false));
            delay(5000); ESP.restart(); return;
        }
    }
    adicionarLog("Escaneando WiFi...");
    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, false, false, 600);
    snprintf(tempBuf, sizeof(tempBuf), "Redes: %d", n);
    adicionarLog(tempBuf);

    String c = "<div class='nf'>&#8505; Configure a rede. IP fixo obrigatorio.</div>";
    c += "<h3>&#128246; Configurar Rede</h3>" + msg;
    c += "<form method='POST' action='/setup-wifi'>";
    c += "<label>Modo:</label>";
    c += "<select name='modo' id='modo' onchange='toggleModo()'>";
    c += "<option value='wifi'>Conectar ao WiFi</option>";
    c += "<option value='ap'>Manter Ponto de Acesso</option></select>";
    c += "<div id='dw'>";
    c += "<label>Rede WiFi:</label><select name='ssid'>";
    if (n == 0) c += "<option value=''>Nenhuma rede</option>";
    else {
        for (int i = 0; i < n; i++) {
            String q = WiFi.RSSI(i)>-60?"Bom":WiFi.RSSI(i)>-75?"Regular":"Fraco";
            c += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i);
            snprintf(tempBuf,sizeof(tempBuf)," [%s %ddBm]",q.c_str(),WiFi.RSSI(i));
            c += String(tempBuf) + "</option>";
        }
    }
    WiFi.scanDelete();
    c += "</select>";
    c += "<button type='button' class='btn-o btn-sm' onclick='location.reload()' "
         "style='margin-top:8px'>&#128259; Atualizar Lista</button>";
    c += "<label>Senha WiFi:</label>";
    c += "<input type='password' name='wpass' placeholder='Vazio se aberta'>";
    c += "<label>IP Fixo do ESP32:</label>";
    c += "<input type='text' name='ip' required placeholder='192.168.1.100'>";
    c += "<label>Gateway:</label>";
    c += "<input type='text' name='gw' required placeholder='192.168.1.1'>";
    c += "<label>Mascara:</label>";
    c += "<input type='text' name='mask' required value='255.255.255.0'>";
    c += "<label>DNS:</label>";
    c += "<input type='text' name='dns' required value='8.8.8.8'>";
    c += "</div>";
    c += "<button type='submit' class='btn-g'>&#128190; Salvar e Reiniciar</button></form>";
    if (logado)
        c += "<button onclick='location.href=\"/\"' class='btn-s' "
             "style='margin-top:8px'>&#8592; Voltar</button>";
    c += "<script>function toggleModo(){"
         "document.getElementById('dw').style.display="
         "document.getElementById('modo').value=='wifi'?'block':'none';}</script>";
    server.send(200,"text/html; charset=utf-8",
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
        String user = server.arg("user"), pass = server.arg("pass");
        if (verificarLogin(user, pass)) {
            logado = true;
            snprintf(tempBuf,sizeof(tempBuf),"Login OK: %s",user.c_str());
            adicionarLog(tempBuf); logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        } else {
            snprintf(tempBuf,sizeof(tempBuf),"Login FALHOU: %s",user.c_str());
            adicionarLog(tempBuf);
            msg = "<div class='er'>&#10007; Usuario ou senha incorretos</div>";
        }
    }
    String c = "<div class='nf'>&#128274; Acesso seguro ao sistema</div>";
    c += "<h3>Entrar</h3>" + msg;
    c += "<form method='POST' action='/login'>";
    c += "<label>Usuario:</label>";
    c += "<input type='text' name='user' required autocomplete='off' placeholder='Usuario'>";
    c += "<label>Senha:</label>";
    c += "<input type='password' name='pass' required placeholder='Senha'>";
    c += "<button type='submit'>&#128275; Entrar</button></form>";
    server.send(200,"text/html; charset=utf-8",
                gerarPagina("RRFS - Login", c, false));
}

void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado = false; }
    server.sendHeader("Location","/login"); server.send(302);
}

void handleLogs() {
    String json = "[";
    for (int i = 0; i < totalLogs; i++) {
        int idx = ((int)logIndex - 1 - i + MAX_LOGS) % MAX_LOGS;
        if (i > 0) json += ",";
        json += "\"";
        for (int ch = 0; logs[idx][ch]; ch++) {
            char cc = logs[idx][ch];
            if (cc == '"')       json += "\\\"";
            else if (cc == '\\') json += "\\\\";
            else                 json += cc;
        }
        json += "\"";
    }
    json += "]";
    server.send(200,"application/json", json);
}

void handleNotFound() {
    String uri = server.uri();
    // Captive portal detection (Android/iOS/Windows)
    if (uri.indexOf("generate_204") >= 0 || uri.indexOf("gen_204") >= 0) {
        modoAP ? (server.sendHeader("Location","http://10.0.0.1/"),
                  server.send(302,"text/plain",""))
               :  server.send(204,"text/plain","");
        return;
    }
    if (uri.indexOf("hotspot-detect.html") >= 0 ||
        uri.indexOf("library/test/success.html") >= 0) {
        modoAP ? (server.sendHeader("Location","http://10.0.0.1/"),
                  server.send(302,"text/plain",""))
               :  server.send(200,"text/html",
                  "<HTML><HEAD><TITLE>Success</TITLE></HEAD>"
                  "<BODY>Success</BODY></HTML>");
        return;
    }
    if (uri.indexOf("ncsi.txt")    >= 0 || uri.indexOf("connecttest") >= 0 ||
        uri.indexOf("redirect")    >= 0) {
        modoAP ? (server.sendHeader("Location","http://10.0.0.1/"),
                  server.send(302,"text/plain",""))
               :  server.send(200,"text/plain","Microsoft NCSI");
        return;
    }
    if (uri == "/favicon.ico") { server.send(204,"text/plain",""); return; }
    if (modoAP) {
        server.sendHeader("Location","http://10.0.0.1/"); server.send(302);
    } else {
        server.send(404,"text/plain","404");
    }
}

// ═══════════════════════════════════════════════════════════
// [SIS-DASH] DASHBOARD PRINCIPAL
// ═══════════════════════════════════════════════════════════

void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302); return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi");
        server.send(302); return;
    }
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    EstimativaTempo tempo = calcularTempoRestante();

    // ── Flags de estado ────────────────────────────────────
    bool painelGerando    = (sensores.correntePainel  >  0.5f);
    bool batCarregando    = (sensores.correnteBateria >  0.3f);
    bool batDescarregando = (sensores.correnteBateria < -0.3f);
    bool casaConsumindo   = (sensores.correnteCasa    >  0.3f);
    bool sistemaConsum    = (sensores.correnteSistema >  0.15f) &&
                            !painelGerando;
    bool invAtivo         = sensores.inversoreAtivo;
    bool curtoAlt  = (curto.estado == CURTO_CONFIRMADO_ALTA  ||
                      curto.redeAltaCurto);
    bool curtoBx   = (curto.estado == CURTO_CONFIRMADO_BAIXA ||
                      curto.redeBaixaCurto);
    bool diagAtivo = (curto.estado == CURTO_TESTANDO_ALTA    ||
                      curto.estado == CURTO_TESTANDO_BAIXA   ||
                      curto.estado == CURTO_RELIGANDO);

    // ── Buffers de texto ───────────────────────────────────
    char strIP[10], strIC[10], strIB[10], strIS[10];
    char strKWP[10], strKWC[10], strKWB[10];
    snprintf(strIP,  sizeof(strIP),  "%.1fA", sensores.correntePainel);
    snprintf(strIC,  sizeof(strIC),  "%.1fA", sensores.correnteCasa);
    snprintf(strIB,  sizeof(strIB),  "%.1fA",
             fabsf(sensores.correnteBateria));
    snprintf(strIS,  sizeof(strIS),  "%.2fA", sensores.correnteSistema);
    snprintf(strKWP, sizeof(strKWP), "%.0fW",
             sensores.kWPainel * 1000.0f);
    snprintf(strKWC, sizeof(strKWC), "%.0fW",
             sensores.kWCasa   * 1000.0f);
    snprintf(strKWB, sizeof(strKWB), "%.0fW",
             fabsf(sensores.kWBateria) * 1000.0f);

    char pctBuf[8];
    snprintf(pctBuf, sizeof(pctBuf), "%.0f%%",
             bateria.calibrada ? bateria.percentualAtual : 0.0f);

    char strBatSinal[14];
    snprintf(strBatSinal, sizeof(strBatSinal), "%c%.1fA",
             batCarregando ? '+' : batDescarregando ? '-' : '~',
             fabsf(sensores.correnteBateria));

    // ── Estimativas de tempo até corte ────────────────────
    String tLampInt="--", tLampExt="--", tRedeAlt="--", tRedeBx="--";
    bool estimOk = (batDescarregando && bateria.calibrada);
    if (estimOk) {
        float tx = sensores.correnteBateria;
        tLampInt = tempoAteCorte(redes.corteLampInt, tx);
        tLampExt = tempoAteCorte(redes.corteLampExt, tx);
        tRedeAlt = tempoAteCorte(redes.corteAlta,    tx);
        tRedeBx  = tempoAteCorte(redes.corteBaixa,   tx);
    }

    // Helper estimativa desligamento
    auto htmlEstimativa = [&](const String& t) -> String {
        if (!bateria.calibrada || !estimOk) return "";
        if (t == "--") return "";
        String cor = (t == "Agora") ? "#ef5350" : "#ffb74d";
        return "<span style='color:" + cor +
               "'>&#8987; Desliga em: <b>" + t + "</b></span>";
    };

    // ── Cores dos componentes ──────────────────────────────
    String corPainel = painelGerando    ? "#fdd835" : "#455a64";
    String corBat    = !bateria.calibrada           ? "#455a64" :
                        bateria.percentualAtual > 50 ? "#43a047" :
                        bateria.percentualAtual > 25 ? "#ef6c00" : "#c62828";
    String corInv    = invAtivo          ? "#4fc3f7" : "#455a64";
    String corAlt    = curtoAlt          ? "#ff6f00" :
                        estados.redesAlta ? "#ef5350" : "#455a64";
    String corBaixa  = curtoBx           ? "#ff6f00" :
                        estados.redeBaixa ? "#42a5f5" : "#455a64";
    String corLampE  = estados.lampExt   ? "#ffd54f" : "#455a64";
    String corLampI  = estados.lampInt   ? "#ffd54f" : "#455a64";

    String c = "";

    // ══════════════════════════════════════════════════════
    // BLOCO RECOMENDAÇÃO IA (topo da página)
    // ══════════════════════════════════════════════════════
    if (ia.recomendacao.ativa) {
        String corBg, corBd, corTx, ico;
        switch (ia.recomendacao.nivel) {
            case 0:
                corBg = "#0d1f2d"; corBd = "#0277bd";
                corTx = "#90caf9"; ico   = "&#128161;";
                break;
            case 1:
                corBg = "#1f1500"; corBd = "#ef6c00";
                corTx = "#ffe0b2"; ico   = "&#9888;";
                break;
            case 2:
            default:
                corBg = "#1f0a0a"; corBd = "#c62828";
                corTx = "#ef9a9a"; ico   = "&#128680;";
                break;
        }

        unsigned long secsAtras =
            (millis() - ia.recomendacao.tCriado) / 1000UL;
        char bufT[20];
        if (secsAtras < 60)
            snprintf(bufT, sizeof(bufT), "ha %lus", secsAtras);
        else
            snprintf(bufT, sizeof(bufT), "ha %lumin", secsAtras / 60UL);

        c += "<div style='background:" + corBg +
             ";border:2px solid " + corBd +
             ";border-radius:8px;padding:12px;margin-bottom:8px'>";
        c += "<div style='display:flex;justify-content:space-between;"
             "align-items:flex-start'>";
        c += "<div style='font-size:13px;font-weight:bold;color:" +
             corTx + "'>" + ico + " ";
        c += String(ia.recomendacao.texto) + "</div>";
        c += "<div style='font-size:10px;color:#546e7a;"
             "white-space:nowrap;margin-left:8px'>"
             + String(bufT) + "</div>";
        c += "</div>";

        if (strlen(ia.recomendacao.detalhe) > 0) {
            c += "<div style='font-size:11px;color:" + corTx +
                 ";opacity:0.8;margin-top:6px'>";
            c += String(ia.recomendacao.detalhe) + "</div>";
        }

        // Saldo Ah (modo IA ativo)
        if (ia.config.modo == IA_MODO_IA && bateria.calibrada) {
            c += "<div style='display:flex;gap:8px;flex-wrap:wrap;"
                 "margin-top:8px;font-size:10px;border-top:1px solid "
                 + corBd + ";padding-top:6px'>";
            char bufS[160];
            snprintf(bufS, sizeof(bufS),
                     "<span style='color:#78909c'>Disp: "
                     "<b style='color:#4fc3f7'>%.1fAh</b></span> "
                     "<span style='color:#78909c'>Nec: "
                     "<b style='color:#ffb74d'>%.1fAh</b></span> "
                     "<span style='color:%s'>Saldo: "
                     "<b>%+.1fAh</b></span>",
                     ia.decisao.ahDisponivel,
                     ia.decisao.ahEstimadoNecessario,
                     ia.saldoAhAtual >= 0 ? "#43a047" : "#ef5350",
                     ia.saldoAhAtual);
            c += String(bufS) + "</div>";
        }

        // Modo manual: lembrete
        if (ia.config.modo == IA_MODO_MANUAL) {
            c += "<div style='font-size:10px;color:#546e7a;margin-top:6px'>";
            c += "&#129302; Modo Manual: IA recomenda, voce decide. "
                 "<a href='/config' style='color:#90caf9'>"
                 "Ativar IA automatica</a></div>";
        }
        c += "</div>";
    }

    // ── Aviso bateria não calibrada ────────────────────────
    if (!bateria.calibrada) {
        c += "<div class='wn' style='font-size:11px;margin-bottom:6px'>"
             "&#9888; Bateria nao calibrada. "
             "<a href='/bateria' style='color:#ffe0b2'>"
             "Calibrar agora</a></div>";
    }

    // ── Status IA resumido (quando modo IA e sem recomendação) ──
    if (ia.config.modo == IA_MODO_IA && !ia.recomendacao.ativa) {
        String corSeq = "#0277bd";
        String txtSeq = "&#129302; IA ativa - monitorando";
        if (ia.usandoFallbackManual)
            txtSeq = "&#129302; IA ativa - aguardando dados (fallback manual)";
        else if (ia.analise.emPico) {
            float durP = (millis() - ia.analise.tInicioPico) / 1000.0f;
            char bufP[60];
            snprintf(bufP, sizeof(bufP),
                     "&#129302; IA: pico %.2fA por %.0fs",
                     sensores.correnteCasa, durP);
            txtSeq = String(bufP);
            corSeq = "#ef6c00";
        } else if (ia.analise.baselineConfianca > 0.3f) {
            char bufB[60];
            snprintf(bufB, sizeof(bufB),
                     "&#129302; IA: base %.2fA | saldo %+.1fAh",
                     ia.analise.baselineAtual,
                     ia.saldoAhAtual);
            txtSeq = String(bufB);
            corSeq = ia.saldoAhAtual >= 0 ? "#43a047" : "#ef6c00";
        }
        c += "<div style='background:#0d1a0d;border:1px solid " +
             corSeq + ";border-radius:6px;padding:8px;"
             "margin-bottom:8px;font-size:11px;color:" + corSeq + "'>";
        c += txtSeq + "</div>";
    }

    // ── CSS Diagrama ───────────────────────────────────────
    c += F("<style>"
        "@keyframes fluir{to{stroke-dashoffset:-24}}"
        ".fx-linha{stroke-dasharray:6 6;"
        "animation:fluir 1s linear infinite;}"
        ".fx-parada{opacity:0.12;stroke-dasharray:2 4;}"
        ".fx-caixa{filter:drop-shadow(0 2px 6px rgba(0,0,0,.5));}"
        ".fx-lbl{font-family:Arial,sans-serif;font-weight:bold;}"
        ".fx-sub{font-family:Arial,sans-serif;}"
        ".fx-pulse{animation:pulso 1.6s ease-in-out infinite;}"
        "@keyframes pulso{0%,100%{opacity:1}50%{opacity:.55}}"
    "</style>");

    c += "<h3 style='text-align:center;margin:4px 0 10px'>"
         "&#9889; Fluxo de Energia</h3>";

    // ── SVG Diagrama de Fluxo ──────────────────────────────
    {
        bool fluxoPB = painelGerando && batCarregando;
        bool fluxoPC = painelGerando && casaConsumindo;
        bool fluxoBC = batDescarregando && casaConsumindo;
        bool fluxoBS = sistemaConsum;

        String lnPB = fluxoPB ? "#43a047" : "#333";
        String lnPC = fluxoPC ? "#fdd835" : "#333";
        String lnBC = fluxoBC ? "#ef6c00" : "#333";
        String lnBS = fluxoBS ? "#ff9800" : "#333";

        String cPBg = painelGerando    ? "#3a2f00" : "#181818";
        String cPBd = painelGerando    ? "#fdd835" : "#3a3a3a";
        String cBBg = !bateria.calibrada           ? "#181818" :
                       bateria.percentualAtual > 50 ? "#0a2210" :
                       bateria.percentualAtual > 25 ? "#2a1600" : "#2a0808";
        String cCBg = casaConsumindo   ? "#062231" : "#181818";
        String cCBd = casaConsumindo   ? "#4fc3f7" : "#3a3a3a";
        String cSBg = sistemaConsum    ? "#241505" : "#181818";
        String cSBd = sistemaConsum    ? "#ff9800" : "#3a3a3a";

        int vbH = sistemaConsum ? 275 : 230;
        c += "<svg viewBox='0 0 340 " + String(vbH) +
             "' xmlns='http://www.w3.org/2000/svg' "
             "style='width:100%;height:auto;display:block;margin:0 auto'>";

        // Linhas de fluxo
        c += "<path d='M110,55 C160,20 200,20 240,55' fill='none' "
             "stroke='" + lnPB + "' stroke-width='3' "
             "stroke-linecap='round' class='" +
             String(fluxoPB ? "fx-linha" : "fx-parada") + "'/>";
        c += "<path d='M90,90 C80,130 90,150 130,175' fill='none' "
             "stroke='" + lnPC + "' stroke-width='3' "
             "stroke-linecap='round' class='" +
             String(fluxoPC ? "fx-linha" : "fx-parada") + "'/>";
        c += "<path d='M255,90 C260,130 250,150 210,175' fill='none' "
             "stroke='" + lnBC + "' stroke-width='3' "
             "stroke-linecap='round' class='" +
             String(fluxoBC ? "fx-linha" : "fx-parada") + "'/>";
        if (sistemaConsum)
            c += "<path d='M270,95 C270,220 270,220 270,240' "
                 "fill='none' stroke='" + lnBS + "' stroke-width='3' "
                 "stroke-linecap='round' class='fx-linha'/>";

        // Labels nas linhas
        if (fluxoPB) {
            c += "<rect x='143' y='24' width='54' height='16' "
                 "rx='8' fill='#0d0d0d'/>";
            c += "<text x='170' y='36' text-anchor='middle' "
                 "font-size='11' fill='" + lnPB +
                 "' class='fx-lbl'>" + String(strKWP) + "</text>";
        }
        if (fluxoPC) {
            c += "<rect x='60' y='125' width='46' height='15' "
                 "rx='7' fill='#0d0d0d'/>";
            c += "<text x='83' y='136' text-anchor='middle' "
                 "font-size='10' fill='" + lnPC +
                 "' class='fx-lbl'>" + String(strKWC) + "</text>";
        }
        if (fluxoBC) {
            c += "<rect x='232' y='125' width='46' height='15' "
                 "rx='7' fill='#0d0d0d'/>";
            c += "<text x='255' y='136' text-anchor='middle' "
                 "font-size='10' fill='" + lnBC +
                 "' class='fx-lbl'>" + String(strKWB) + "</text>";
        }

        // Caixa Painel
        c += "<g class='fx-caixa" +
             String(painelGerando ? " fx-pulse" : "") + "'>";
        c += "<rect x='20' y='15' width='100' height='80' rx='14' "
             "fill='" + cPBg + "' stroke='" + cPBd +
             "' stroke-width='2'/>";
        c += "<text x='70' y='42' text-anchor='middle' "
             "font-size='24'>&#9728;</text>";
        c += "<text x='70' y='60' text-anchor='middle' "
             "font-size='9' fill='#78909c' class='fx-sub'>"
             "PAINEL SOLAR</text>";
        c += "<text x='70' y='76' text-anchor='middle' "
             "font-size='16' fill='" + cPBd +
             "' class='fx-lbl' id='svg-painel-a'>";
        c += painelGerando ? String(strIP) : String("0.0A");
        c += "</text>";
        c += "<text x='70' y='90' text-anchor='middle' "
             "font-size='9' fill='#546e7a' id='svg-painel-w'>";
        c += painelGerando ? String(strKWP) : String("sem geracao");
        c += "</text>";
        c += "</g>";

        // Caixa Bateria
        c += "<g class='fx-caixa'>";
        c += "<rect x='220' y='15' width='100' height='80' rx='14' "
             "fill='" + cBBg + "' stroke='" + corBat +
             "' stroke-width='2'/>";
        String icoBat = batCarregando   ? "&#9650;&#128267;" :
                        batDescarregando? "&#9660;&#128267;" : "&#128267;";
        c += "<text x='270' y='42' text-anchor='middle' "
             "font-size='22'>" + icoBat + "</text>";
        c += "<text x='270' y='60' text-anchor='middle' "
             "font-size='9' fill='#78909c' class='fx-sub'>"
             "BATERIA</text>";
        c += "<text x='270' y='76' text-anchor='middle' "
             "font-size='16' fill='" + corBat +
             "' class='fx-lbl' id='svg-bat-pct'>";
        c += bateria.calibrada ? String(pctBuf) : String("N/Cal");
        c += "</text>";
        c += "<text x='270' y='90' text-anchor='middle' "
             "font-size='9' fill='#ce93d8' id='svg-bat-c'>"
             + String(strBatSinal) + "</text>";
        c += "</g>";

        // Caixa Casa
        c += "<g class='fx-caixa" +
             String(casaConsumindo ? " fx-pulse" : "") + "'>";
        c += "<rect x='105' y='175' width='130' height='55' rx='14' "
             "fill='" + cCBg + "' stroke='" + cCBd +
             "' stroke-width='2'/>";
        c += "<text x='128' y='210' font-size='22'>&#127968;</text>";
        c += "<text x='225' y='196' text-anchor='end' "
             "font-size='9' fill='#78909c' class='fx-sub'>CASA</text>";
        c += "<text x='225' y='212' text-anchor='end' "
             "font-size='14' fill='" +
             (casaConsumindo ? cCBd : String("#455a64")) +
             "' class='fx-lbl' id='svg-casa-a'>";
        c += casaConsumindo ? String(strIC) : String("0.0A");
        c += "</text>";
        c += "<text x='225' y='224' text-anchor='end' "
             "font-size='9' fill='#546e7a' id='svg-casa-w'>";
        c += casaConsumindo ? String(strKWC) : String("");
        c += "</text>";
        c += "</g>";

        // Caixa Sistema (só quando sem geração)
        if (sistemaConsum) {
            c += "<text x='239' y='207' text-anchor='middle' "
                 "font-size='20' font-weight='bold' "
                 "fill='#ff9800'>+</text>";
            c += "<g class='fx-caixa fx-pulse'>";
            c += "<rect x='245' y='240' width='90' height='68' rx='12' "
                 "fill='" + cSBg + "' stroke='" + cSBd +
                 "' stroke-width='2'/>";
            c += "<text x='290' y='262' text-anchor='middle' "
                 "font-size='18'>&#128268;</text>";
            c += "<text x='290' y='276' text-anchor='middle' "
                 "font-size='8' fill='#78909c' class='fx-sub'>"
                 "ESP32+CTRL</text>";
            c += "<text x='290' y='291' text-anchor='middle' "
                 "font-size='12' fill='" + cSBd +
                 "' class='fx-lbl' id='svg-sistema-a'>"
                 + String(strIS) + "</text>";
            c += "</g>";
        }
        c += "</svg>";
    }

    if (sistemaConsum) {
        c += "<div class='nf' style='font-size:10px;"
             "margin-bottom:8px;text-align:center'>"
             "&#128161; Painel sem geracao: consumo ESP32/controlador "
             "(<b>" + String(strIS) + "</b>) somado ao da casa.</div>";
    }

    // ── Cards de cargas ────────────────────────────────────
    auto carga = [&](const char* ico, const char* nome,
                     const String& cor, const String& bg,
                     bool ligado, const String& statusTxt,
                     const String& info,
                     const String& estimTxt = "") -> String {
        String h = "<div class='cg' style='border-color:" + cor +
                   ";background:" + bg + "'>";
        h += "<div class='cg-row'>";
        h += "<span class='cg-nm'>" + String(ico) +
             " " + String(nome) + "</span>";
        h += "<span class='cg-st' style='color:" + cor + "'>";
        h += (ligado ? "&#9679; " : "&#9675; ") + statusTxt;
        h += "</span></div>";
        if (info.length() || estimTxt.length()) {
            h += "<div class='cg-inf'>";
            if (info.length())     h += "<div>" + info + "</div>";
            if (estimTxt.length()) h += "<div>" + estimTxt + "</div>";
            h += "</div>";
        }
        h += "</div>";
        return h;
    };

    String obsInv = diagAtivo ?
        "<span style='color:#90caf9'>"
        "&#128270; Diagnosticando...</span>" :
        (!sensores.inversoreAtivo && estados.inversor) ?
        "<span style='color:#78909c'>Aguardando sinal...</span>" : "";

    String obsAlt = curto.redeAltaCurto ?
        "&#9888; Bloqueada por curto - "
        "<a href='/redes' style='color:#ffcc80'>resolver</a>" : "";

    // Horário solar lamp externa
    String obsLE = "";
    if (configLocal.configurado) {
        struct tm tiLE;
        if (getLocalTime(&tiLE)) {
            int   mes  = tiLE.tm_mon + 1;
            float latR = (float)(configLocal.latitude * 3.14159 / 180.0);
            float dec  = -23.45f * cosf(2.0f * 3.14159f *
                          (float)(mes + 9.5f) / 12.0f);
            float dR   = dec * 3.14159f / 180.0f;
            float cH   = -tanf(latR) * tanf(dR);
            float hLuz = 12.0f;
            if (cH >= -1 && cH <= 1)
                hLuz = (2.0f/15.0f) *
                       (acosf(cH) * 180.0f / 3.14159f);
            float nS = 12.0f - (hLuz / 2.0f);
            float pS = 12.0f + (hLuz / 2.0f);
            char bufS[80];
            snprintf(bufS, sizeof(bufS),
                     "&#9728; Solar | Nasc: %.0fh%02.0fm "
                     "Por: %.0fh%02.0fm",
                     floorf(nS), (nS - floorf(nS)) * 60,
                     floorf(pS), (pS - floorf(pS)) * 60);
            obsLE = String(bufS);
        }
    } else {
        obsLE = "&#9888; Configure localizacao em Previsao "
                "para controle solar";
    }

    String obsBx = curto.redeBaixaCurto ?
                   "&#9888; Bloqueada por curto" : "";

    c += carga("&#9889;",   "Inversor",
               corInv, invAtivo ? "#081824" : "#0d0d0d",
               invAtivo,
               invAtivo ? "ATIVO" :
               (estados.inversor ? "AGUARD" : "OFF"),
               obsInv);

    c += carga("&#128268;",
               "Rede Alta (Equipamentos pesados)",
               curtoAlt ? "#ff6f00" : corAlt,
               curtoAlt ? "#1a0f00" :
               estados.redesAlta ? "#180808" : "#0d0d0d",
               estados.redesAlta && !curtoAlt,
               curtoAlt ? "CURTO!" :
               (estados.redesAlta ? "ON" : "OFF"),
               obsAlt,
               estados.redesAlta ? htmlEstimativa(tRedeAlt) : "");

    c += carga("&#128268;",
               "Rede Baixa (Tomadas normais)",
               curtoBx ? "#ff6f00" : corBaixa,
               curtoBx ? "#1a0f00" :
               estados.redeBaixa ? "#080d18" : "#0d0d0d",
               estados.redeBaixa && !curtoBx,
               curtoBx ? "CURTO!" :
               (estados.redeBaixa ? "ON" : "OFF"),
               obsBx,
               estados.redeBaixa ? htmlEstimativa(tRedeBx) : "");

    c += carga("&#128161;", "Lamp. Externa",
               corLampE, estados.lampExt ? "#181500" : "#0d0d0d",
               estados.lampExt,
               estados.lampExt ? "ON" : "OFF",
               obsLE,
               estados.lampExt ? htmlEstimativa(tLampExt) : "");

    c += carga("&#128161;", "Lamp. Interna",
               corLampI, estados.lampInt ? "#181500" : "#0d0d0d",
               estados.lampInt,
               estados.lampInt ? "ON" : "OFF",
               "",
               estados.lampInt ? htmlEstimativa(tLampInt) : "");

    // Alerta de curto
    if (curtoAlt || curtoBx ||
        curto.estado == CURTO_AGUARDANDO_RESET) {
        c += "<div style='background:#7f0000;border:2px solid #ef5350;"
             "border-radius:8px;padding:10px;text-align:center;"
             "margin-top:4px'>";
        c += "<div style='font-size:13px;font-weight:bold;"
             "color:#ef9a9a'>&#9888; ";
        if (curtoAlt)
            c += "CURTO NA REDE ALTA";
        if (curtoBx)
            c += "CURTO NA REDE BAIXA";
        if (curto.estado == CURTO_AGUARDANDO_RESET)
            c += "FALHA NO INVERSOR";
        c += "</div>"
             "<a href='/redes' style='color:#ffcc80;font-size:12px'>"
             "Toque aqui para resolver &rarr;</a></div>";
    }

    // ── Barra de Bateria ───────────────────────────────────
    c += "<div style='margin:10px 0'>";
    c += "<div style='display:flex;justify-content:space-between;"
         "align-items:center;margin-bottom:4px'>";
    c += "<span style='font-size:11px;color:#78909c'>"
         "&#128267; Bateria</span>";

    if (tempo.valido) {
        char bufT[50];
        if (tempo.carregando) {
            if (tempo.horas == 0 && tempo.minutos == 0)
                snprintf(bufT, sizeof(bufT), "Carregada!");
            else
                snprintf(bufT, sizeof(bufT),
                         "Completa em %dh%02dm",
                         tempo.horas, tempo.minutos);
        } else {
            if (tempo.horas == 0 && tempo.minutos < 5)
                snprintf(bufT, sizeof(bufT), "ENERGIA CRITICA!");
            else
                snprintf(bufT, sizeof(bufT),
                         "Restam %dh%02dm",
                         tempo.horas, tempo.minutos);
        }
        String corT = tempo.carregando ? "#43a047" :
                      (tempo.horas == 0 && tempo.minutos < 30 ?
                       "#ef5350" : "#ef6c00");
        c += "<span style='font-size:11px;font-weight:bold;color:"
             + corT + "'>";
        c += tempo.carregando ? "&#9650; " : "&#9660; ";
        c += String(bufT) + "</span>";
    }
    c += "</div>";

    // Horário estimado de carga/descarga completa
    if (tempo.valido &&
        !(tempo.horas == 0 && tempo.minutos == 0 && tempo.carregando)) {
        struct tm ti;
        if (getLocalTime(&ti)) {
            time_t agora = mktime(&ti);
            time_t alvo  = agora + (long)tempo.horas * 3600L +
                                   (long)tempo.minutos * 60L;
            struct tm tA;
            localtime_r(&alvo, &tA);
            bool mesmoDia = (tA.tm_mday == ti.tm_mday &&
                             tA.tm_mon  == ti.tm_mon);
            char bufD[40];
            if (mesmoDia) {
                snprintf(bufD, sizeof(bufD),
                         "hoje as %02d:%02d",
                         tA.tm_hour, tA.tm_min);
            } else {
                const char* ds[] = {
                    "Dom","Seg","Ter","Qua","Qui","Sex","Sab"
                };
                struct tm tAmanha = ti;
                tAmanha.tm_mday += 1;
                mktime(&tAmanha);
                bool ehAmanha = (tA.tm_mday == tAmanha.tm_mday &&
                                 tA.tm_mon  == tAmanha.tm_mon);
                if (ehAmanha)
                    snprintf(bufD, sizeof(bufD),
                             "amanha as %02d:%02d",
                             tA.tm_hour, tA.tm_min);
                else
                    snprintf(bufD, sizeof(bufD),
                             "%s %02d/%02d as %02d:%02d",
                             ds[tA.tm_wday],
                             tA.tm_mday, tA.tm_mon + 1,
                             tA.tm_hour, tA.tm_min);
            }
            String corD = tempo.carregando ? "#43a047" : "#ffb74d";
            String icoD = tempo.carregando ? "&#9889;" : "&#128267;";
            String vbD  = tempo.carregando ?
                          "Carga completa" : "Descarrega ate";
            c += "<div style='font-size:10px;color:" + corD +
                 ";text-align:right;margin-top:-2px;margin-bottom:4px'>";
            c += icoD + " " + vbD + ": <b>" + String(bufD) + "</b></div>";
        }
    }

    if (bateria.calibrada) {
        float  pct    = bateria.percentualAtual;
        String corBrr = pct >= 50 ? "#2e7d32" :
                        pct >= 25 ? "#ef6c00" : "#c62828";
        c += "<div class='barra-bg'><div class='barra' id='bat-barra' "
             "style='width:" + String((int)pct) +
             "%;background:" + corBrr + "'>" +
             String((int)pct) + "%</div></div>";
        c += "<div style='display:flex;justify-content:space-between;"
             "margin-top:3px;font-size:10px;color:#546e7a'>";
        char bAh[60];
        snprintf(bAh, sizeof(bAh), "%.1f / %.1f Ah",
                 bateria.ahAtual, bateria.ahDisponivel);
        c += "<span id='bat-ah'>" + String(bAh) + "</span>";
        char bKW[40];
        snprintf(bKW, sizeof(bKW), "%.0fW @ %.1fV nom %s",
                 fabsf(sensores.kWBateria) * 1000.0f,
                 bateria.tensaoNominal,
                 batCarregando  ? "entrada" :
                 batDescarregando ? "saida" : "equil.");
        c += "<span id='bat-kw'>" + String(bKW) + "</span></div>";
    } else {
        c += "<div class='wn' style='font-size:11px;text-align:center'>"
             "Nao calibrada - "
             "<a href='/bateria' style='color:#ffe0b2'>"
             "calibrar</a></div>";
    }
    c += "</div>";

    // ── Cards de Corrente ──────────────────────────────────
    c += "<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
         "gap:6px;margin:4px 0'>";

    // Painel
    {
        String h = "<div class='sc";
        if (painelGerando) h += " ok";
        h += "' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>&#9728; Painel</div>";
        h += "<div style='font-size:18px;font-weight:bold;color:"
             + corPainel + "' id='corr-painel'>";
        h += painelGerando ? String(strIP) : String("0.0A");
        h += "</div>";
        h += "<div class='sub' id='kw-painel' style='color:#546e7a'>";
        h += painelGerando ? String(strKWP) : String("sem geracao");
        h += "</div></div>";
        c += h;
    }

    // Casa
    {
        String h = "<div class='sc' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>&#127968; Casa</div>";
        h += "<div style='font-size:18px;font-weight:bold;color:#4fc3f7'"
             " id='corr-casa'>" + String(strIC) + "</div>";
        h += "<div class='sub' id='kw-casa' style='color:#546e7a'>"
             + String(strKWC) + "</div>";
        h += "</div>";
        c += h;
    }

    // Bateria
    {
        String corF  = batCarregando   ? "#43a047" :
                       batDescarregando? "#ef6c00" : "#fdd835";
        String sinal = batCarregando   ? "+" :
                       batDescarregando? "-" : "~";
        String h = "<div class='sc' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>&#128267; Bateria</div>";
        h += "<div style='font-size:18px;font-weight:bold;color:"
             + corF + "' id='corr-bat'>";
        h += sinal + String(strIB) + "</div>";
        h += "<div class='sub' id='kw-bat' style='color:#546e7a'>"
             + String(strKWB) + "</div>";
        h += "</div>";
        c += h;
    }
    c += "</div>";

    // ── Info Sistema ───────────────────────────────────────
    String ip   = modoAP ? WiFi.softAPIP().toString() :
                           WiFi.localIP().toString();
    String modo = modoAP ? "AP" : "WiFi";
    c += "<div class='nf' style='font-size:10px;margin-top:4px'>";
    c += "&#128246; " + modo + " | " + ip;
    if (!modoAP && WiFi.status() == WL_CONNECTED)
        c += " | " + String(WiFi.RSSI()) + "dBm";
    snprintf(tempBuf, sizeof(tempBuf),
             " | RAM:%.1fKB | Up:%lus",
             esp_get_free_heap_size() / 1024.0f,
             millis() / 1000UL);
    c += String(tempBuf) + "</div>";

    // ── AJAX atualiza a cada 1s ────────────────────────────
    c += F("<script>"
    "function atualizarDados(){"
    "var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){"
    "if(x.readyState==4&&x.status==200){"
    "try{"
    "var d=JSON.parse(x.responseText);"

    "var ip=parseFloat(d.ip||0);"
    "var ic=parseFloat(d.ic||0);"
    "var ib=parseFloat(d.ib||0);"

    "var elIP=document.getElementById('corr-painel');"
    "if(elIP)elIP.textContent=ip>0.5?ip.toFixed(1)+'A':'0.0A';"

    "var elIC=document.getElementById('corr-casa');"
    "if(elIC)elIC.textContent=ic.toFixed(1)+'A';"

    "var elIB=document.getElementById('corr-bat');"
    "if(elIB){"
    "var s=ib>0.3?'+':ib<-0.3?'-':'~';"
    "elIB.textContent=s+Math.abs(ib).toFixed(1)+'A';}"

    "var vnom=parseFloat(d.vnom||12.8);"
    "var kwp=Math.abs(ip*vnom);"
    "var kwc=Math.abs(ic*vnom);"
    "var kwb=Math.abs(ib*vnom);"

    "var elKWP=document.getElementById('kw-painel');"
    "if(elKWP)elKWP.textContent=ip>0.5?kwp.toFixed(0)+'W':'sem geracao';"

    "var elKWC=document.getElementById('kw-casa');"
    "if(elKWC)elKWC.textContent=kwc.toFixed(0)+'W';"

    "var elKWB=document.getElementById('kw-bat');"
    "if(elKWB)elKWB.textContent=kwb.toFixed(0)+'W';"

    "var svPA=document.getElementById('svg-painel-a');"
    "if(svPA)svPA.textContent=ip>0.5?ip.toFixed(1)+'A':'0.0A';"
    "var svPW=document.getElementById('svg-painel-w');"
    "if(svPW)svPW.textContent=ip>0.5?kwp.toFixed(0)+'W':'sem geracao';"
    "var svCA=document.getElementById('svg-casa-a');"
    "if(svCA)svCA.textContent=ic>0.3?ic.toFixed(1)+'A':'0.0A';"
    "var svCW=document.getElementById('svg-casa-w');"
    "if(svCW)svCW.textContent=ic>0.3?kwc.toFixed(0)+'W':'';"
    "var svBC=document.getElementById('svg-bat-c');"
    "if(svBC){"
    "var s=ib>0.3?'+':ib<-0.3?'-':'~';"
    "svBC.textContent=s+Math.abs(ib).toFixed(1)+'A';}"
    "var svBP=document.getElementById('svg-bat-pct');"
    "if(svBP)svBP.textContent=Math.round(d.pct||0)+'%';"
    "var svSA=document.getElementById('svg-sistema-a');"
    "if(svSA&&d.is!==undefined)"
    "svSA.textContent=parseFloat(d.is).toFixed(2)+'A';"

    "var pct=parseFloat(d.pct||0);"
    "var elBarra=document.getElementById('bat-barra');"
    "if(elBarra&&d.calib){"
    "elBarra.style.width=pct+'%';"
    "elBarra.textContent=Math.round(pct)+'%';"
    "var cor=pct>=50?'#2e7d32':pct>=25?'#ef6c00':'#c62828';"
    "elBarra.style.background=cor;}"

    "var elAh=document.getElementById('bat-ah');"
    "if(elAh&&d.calib){"
    "elAh.textContent=parseFloat(d.ah_at||0).toFixed(1)"
    "+' / '+parseFloat(d.ah_disp||0).toFixed(1)+' Ah';}"

    "var elKW=document.getElementById('bat-kw');"
    "if(elKW){"
    "var st=ib>0.3?'entrada':ib<-0.3?'saida':'equil.';"
    "elKW.textContent=kwb.toFixed(0)+'W @ '+vnom.toFixed(1)"
    "+'V nom '+st;}"

    "}catch(e){console.log('err:',e);}}"
    "};x.open('GET','/dados',true);x.send();}"
    "setInterval(atualizarDados,1000);atualizarDados();"
    "setInterval(function(){location.reload();},30000);"
    "</script>");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Painel", c));
}

// ── Endpoint JSON para AJAX ────────────────────────────────
void handleDados() {
    if (!logado) { server.send(403); return; }

    int total = histSnapFull ? HIST_SNAP_PONTOS : histSnapIdx;

    char buf[400];
    snprintf(buf, sizeof(buf),
             "{\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,\"is\":%.2f,"
             "\"kWp\":%.3f,\"kWc\":%.3f,\"kWb\":%.3f,"
             "\"pct\":%.1f,\"ah_at\":%.2f,\"ah_disp\":%.2f,"
             "\"vnom\":%.1f,"
             "\"painel_on\":%s,"
             "\"inv\":%s,\"rb\":%s,\"ra\":%s,\"le\":%s,\"li\":%s,"
             "\"calib\":%s",
             sensores.correntePainel,
             sensores.correnteCasa,
             sensores.correnteBateria,
             sensores.correnteSistema,
             sensores.kWPainel,
             sensores.kWCasa,
             sensores.kWBateria,
             bateria.percentualAtual,
             bateria.ahAtual,
             bateria.ahDisponivel,
             bateria.tensaoNominal,
             sensores.correntePainel > 0.5f ? "true":"false",
             sensores.inversoreAtivo  ? "true":"false",
             estados.redeBaixa  ? "true":"false",
             estados.redesAlta  ? "true":"false",
             estados.lampExt    ? "true":"false",
             estados.lampInt    ? "true":"false",
             bateria.calibrada  ? "true":"false"
    );

    String json = String(buf);

    // Histórico de corrente para gráfico (últimos 60s)
    json += ",\"hist_p\":[";
    for (int i = 0; i < total; i++) {
        int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
        if (i>0) json += ",";
        char vb[10];
        snprintf(vb,sizeof(vb),"%.1f",histCorrPainel[idx]);
        json += String(vb);
    }
    json += "],\"hist_c\":[";
    for (int i = 0; i < total; i++) {
        int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
        if (i>0) json += ",";
        char vb[10];
        snprintf(vb,sizeof(vb),"%.1f",histCorrCasa[idx]);
        json += String(vb);
    }
    json += "],\"hist_b\":[";
    for (int i = 0; i < total; i++) {
        int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
        if (i>0) json += ",";
        char vb[10];
        snprintf(vb,sizeof(vb),"%.1f",histCorrBat[idx]);
        json += String(vb);
    }
    json += "]}";

    server.send(200,"application/json", json);
}

// ═══════════════════════════════════════════════════════════
// [SIS-BAT-WEB] PÁGINA DE BATERIA
// ═══════════════════════════════════════════════════════════

void handleBateria() {
    if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            float vnom = server.arg("vnom").toFloat();
            float ah   = server.arg("ah").toFloat();
            float desc = server.arg("desc").toFloat();
            if (vnom<6||vnom>60)
                msg = "<div class='er'>&#10007; Tensao invalida (6-60V)</div>";
            else if (ah<1||ah>5000)
                msg = "<div class='er'>&#10007; Capacidade invalida (1-5000Ah)</div>";
            else if (desc<10||desc>100)
                msg = "<div class='er'>&#10007; Descarga invalida (10-100%)</div>";
            else {
                bateria.tensaoNominal  = vnom;
                bateria.capacidadeAh   = ah;
                bateria.descargaSegura = desc;
                calcularBateria();
                salvarConfigBateria();
                snprintf(tempBuf,sizeof(tempBuf),
                         "Bat config: %.1fV %.0fAh %.0f%%",
                         vnom,ah,desc);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Configuracao salva!</div>";
            }
        }
        else if (acao == "calibrar") {
            float pctBMS = server.arg("pct_atual").toFloat();
            if (pctBMS<0||pctBMS>100)
                msg = "<div class='er'>&#10007; Percentual invalido (0-100)</div>";
            else {
                // Converte % do BMS (baseado na capacidade FISICA total)
                // para Ah utilizavel dentro da janela de Descarga Segura
                float ahFisicoAtual = bateria.capacidadeAh * (pctBMS / 100.0f);
                float ahReservado   = bateria.capacidadeAh *
                (1.0f - bateria.descargaSegura / 100.0f);
                float ahUtilAtual   = ahFisicoAtual - ahReservado;

                if (ahUtilAtual < 0) ahUtilAtual = 0;
                if (ahUtilAtual > bateria.ahDisponivel)
                    ahUtilAtual = bateria.ahDisponivel;

                bateria.ahAtual = ahUtilAtual;
                bateria.percentualAtual = (bateria.ahDisponivel > 0)
                ? (ahUtilAtual / bateria.ahDisponivel) * 100.0f : 0.0f;
                bateria.calibrada      = true;
                ultimoContagem          = millis();
                salvarConfigBateria();

                snprintf(tempBuf,sizeof(tempBuf),
                         "Bat calibrada: BMS=%.0f%% -> Fisico=%.1fAh "
                         "Reservado=%.1fAh Util=%.1fAh (%.1f%% do disponivel)",
                         pctBMS, ahFisicoAtual, ahReservado,
                         bateria.ahAtual, bateria.percentualAtual);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Bateria calibrada! BMS mostrava "
                + String((int)pctBMS) + "%, sistema ajustou para "
                + String((int)bateria.percentualAtual)
                + "% dentro da faixa segura de uso.</div>";
            }
        }
    }

    calcularBateria();
    String batCls = bateria.percentualAtual<25?"er":
    bateria.percentualAtual<50?"wn":"ok";

    String c = msg;
    c += "<h3>&#128267; Status da Bateria</h3>";

    if (bateria.calibrada) {
        c += "<div class='sc " + batCls + "'>";
        c += "<div class='lbl'>Carga Atual</div>";
        // Corrente (Ah) como valor principal
        char bufAh[60];
        snprintf(bufAh,sizeof(bufAh),"%.1fAh de %.1fAh (%.0f%%)",
                 bateria.ahAtual, bateria.ahDisponivel,
                 bateria.percentualAtual);
        c += "<div class='val'>" + String(bufAh) + "</div>";
        // kWh como informativo
        char bufKWh[60];
        snprintf(bufKWh,sizeof(bufKWh),
                 "%.2f kWh de %.2f kWh (@ %.1fV nominal)",
                 (bateria.ahAtual     * bateria.tensaoNominal)/1000.0f,
                 (bateria.ahDisponivel* bateria.tensaoNominal)/1000.0f,
                 bateria.tensaoNominal);
        c += "<div class='sub'>" + String(bufKWh) + "</div>";
        c += "</div>";
        c += barraProgresso(bateria.percentualAtual);
    } else {
        c += "<div class='sc wn'><div class='lbl'>Status</div>"
        "<div class='val' style='font-size:16px'>"
        "Aguardando calibracao</div></div>";
    }

    c += "<div class='nf'>";
    char bufI[120];
    snprintf(bufI,sizeof(bufI),
             "Tensao nominal: %.1fV | Capacidade: %.0fAh | "
             "Descarga segura: %.0f%%<br>"
             "Disponivel: <b>%.1fAh</b> | "
             "Equivalente: <b>%.2fkWh</b>",
             bateria.tensaoNominal, bateria.capacidadeAh,
             bateria.descargaSegura, bateria.ahDisponivel,
             (bateria.ahDisponivel * bateria.tensaoNominal)/1000.0f);
    c += String(bufI) + "</div>";

    c += "<div class='nf' style='font-size:10px'>"
    "&#128161; <b>Como funciona:</b> O sistema mede a corrente real pelos "
    "sensores ACS758 e integra no tempo (Ah = A × h). "
    "O kWh e calculado usando a tensao nominal configurada "
    "(apenas informativo).</div>";

    c += "<h3>&#9881; Configurar Banco de Baterias (LiFePO4)</h3>";
    c += "<form method='POST' action='/bateria'>";
    c += "<input type='hidden' name='acao' value='config'>";
    c += "<label>Tensao Nominal do Banco (V):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Ex: 12.8V (4S) | 25.6V (8S) | 51.2V (16S)</div>";
    snprintf(tempBuf,sizeof(tempBuf),"value='%.1f'",bateria.tensaoNominal);
    c += "<input type='number' name='vnom' step='0.1' min='6' max='60' required "
    + String(tempBuf) + " placeholder='Ex: 12.8'>";
    c += "<label>Capacidade (Ah):</label>";
    snprintf(tempBuf,sizeof(tempBuf),"value='%.0f'",bateria.capacidadeAh);
    c += "<input type='number' name='ah' step='1' min='1' max='5000' required "
    + String(tempBuf) + " placeholder='Ex: 100'>";
    c += "<label>Descarga Segura (%):</label>";
    snprintf(tempBuf,sizeof(tempBuf),"value='%.0f'",bateria.descargaSegura);
    c += "<input type='number' name='desc' step='1' min='10' max='100' required "
    + String(tempBuf) + " placeholder='Ex: 80'>";
    c += "<button type='submit' class='btn-g'>"
    "&#128190; Salvar Configuracao</button></form>";

    c += "<h3>&#127919; Calibrar Bateria</h3>";
    c += "<div class='nf'>Informe o percentual EXATAMENTE como aparece no "
    "display/BMS da bateria (baseado na capacidade fisica total de "
    + String(bateria.capacidadeAh,0) + "Ah). "
    "O sistema desconta automaticamente a reserva de "
    + String(100.0f - bateria.descargaSegura,0) +
    "% (Descarga Segura) e calcula o percentual UTILIZAVEL "
    "correspondente.</div>";
    c += "<form method='POST' action='/bateria'>";
    c += "<input type='hidden' name='acao' value='calibrar'>";
    c += "<label>Percentual mostrado no Display/BMS (%):</label>";
    c += "<input type='number' name='pct_atual' step='1' min='0' max='100' "
    "required placeholder='Ex: 75 (como aparece no BMS)'>";
    c += "<button type='submit' class='btn-o'>"
    "&#127919; Calibrar Agora</button></form>";

    // Coulomb counting info
    c += "<div class='nf' style='font-size:10px;margin-top:8px'>"
    "&#9889; <b>Coulomb Counting:</b> O sistema integra a corrente "
    "real dos sensores a cada segundo. Mais preciso que estimativa "
    "por tensao. Recalibre apos longa inatividade.</div>";

    server.send(200,"text/html; charset=utf-8",
                gerarPagina("RRFS - Bateria", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-ROUTES] ROTAS DO SERVIDOR WEB
// ═══════════════════════════════════════════════════════════
void configurarRotas() {
    server.on("/",              HTTP_GET,  handleRoot);
    server.on("/dados",         HTTP_GET,  handleDados);
    server.on("/bateria",       HTTP_ANY,  handleBateria);
    server.on("/redes",         HTTP_ANY,  handleRedes);
    server.on("/sensores",      HTTP_ANY,  handleSensores);
    server.on("/ia",            HTTP_ANY,  handleIA);
    server.on("/config",        HTTP_ANY,  handleConfig);
    server.on("/historico",     HTTP_ANY,  handleHistorico);
    server.on("/previsao",      HTTP_ANY,  handlePrevisao);
    server.on("/logs",          HTTP_GET,  handleLogs);
    server.on("/login",         HTTP_ANY,  handleLogin);
    server.on("/logout",        HTTP_GET,  handleLogout);
    server.on("/setup-login",   HTTP_ANY,  handleSetupLogin);
    server.on("/setup-wifi",    HTTP_ANY,  handleSetupWifi);
    server.on("/reset-curto",   HTTP_POST, handleResetCurto);
    server.on("/ntp-sync",      HTTP_POST, handleNTPSync);
    server.on("/reiniciar",     HTTP_POST, handleReiniciar);
    server.on("/ota",           HTTP_GET,  handleOTAResult);
    server.on("/ota-upload",    HTTP_POST,
              []() { handleOTAResult(); },
              handleOTAUpload);
    server.onNotFound(handleNotFound);
    server.begin();
}

// ═══════════════════════════════════════════════════════════
// [SIS-REDES-WEB] PÁGINA DE REDES E RELÉS
// ═══════════════════════════════════════════════════════════
void handleRedes() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // ── Configuração de cortes ─────────────────────────
        if (acao == "config") {
            float cb  = server.arg("corteBaixa").toFloat();
            float ca  = server.arg("corteAlta").toFloat();
            float cle = server.arg("corteLampExt").toFloat();
            float cli = server.arg("corteLampInt").toFloat();

            if (cb < 0 || cb > 50)
                msg = "<div class='er'>&#10007; Corte Baixa invalido (0-50%)</div>";
            else if (ca < cb || ca > 80)
                msg = "<div class='er'>&#10007; Corte Alta deve ser > Baixa e <= 80%</div>";
            else if (cle < 0 || cle > 90)
                msg = "<div class='er'>&#10007; Corte Lamp.Ext invalido (0-90%)</div>";
            else if (cli < 0 || cli > 90)
                msg = "<div class='er'>&#10007; Corte Lamp.Int invalido (0-90%)</div>";
            else {
                redes.corteBaixa   = cb;
                redes.corteAlta    = ca;
                redes.corteLampExt = cle;
                redes.corteLampInt = cli;
                salvarConfigRedes();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Redes: Bx=%.0f%% At=%.0f%% LE=%.0f%% LI=%.0f%%",
                         cb, ca, cle, cli);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Configuracao salva!</div>";
            }
        }

        // ── Controle manual de relés ───────────────────────
        else if (acao == "rele") {
            String qual  = server.arg("rele");
            String valor = server.arg("valor");
            bool ligar   = (valor == "1");

            if (qual == "inv") {
                if (curto.estado == CURTO_CONFIRMADO_BAIXA ||
                    curto.estado == CURTO_AGUARDANDO_RESET) {
                    msg = "<div class='er'>&#9888; Bloqueado por protecao de curto</div>";
                    } else {
                        if (ligar) { ligarInversor();    adicionarLog("Manual: Inversor ON"); }
                        else       { desligarInversor(); adicionarLog("Manual: Inversor OFF"); }
                        msg = "<div class='ok'>&#10003; Inversor " +
                        String(ligar ? "ligado" : "desligado") + "</div>";
                    }
            }
            else if (qual == "ra") {
                if (curto.estado == CURTO_CONFIRMADO_ALTA) {
                    msg = "<div class='er'>&#9888; Bloqueado: curto na rede alta</div>";
                } else {
                    if (ligar) { ligarRedeAlta();    adicionarLog("Manual: Rede Alta ON"); }
                    else       { desligarRedeAlta(); adicionarLog("Manual: Rede Alta OFF"); }
                    msg = "<div class='ok'>&#10003; Rede Alta " +
                    String(ligar ? "ligada" : "desligada") + "</div>";
                }
            }
            else if (qual == "rb") {
                if (curto.estado == CURTO_CONFIRMADO_BAIXA) {
                    msg = "<div class='er'>&#9888; Bloqueado: curto na rede baixa</div>";
                } else {
                    if (ligar) { ligarRedeBaixa();    adicionarLog("Manual: Rede Baixa ON"); }
                    else       { desligarRedeBaixa(); adicionarLog("Manual: Rede Baixa OFF"); }
                    msg = "<div class='ok'>&#10003; Rede Baixa " +
                    String(ligar ? "ligada" : "desligada") + "</div>";
                }
            }
            else if (qual == "le") {
                if (ligar) { ligarLampExt();    adicionarLog("Manual: Lamp.Ext ON"); }
                else       { desligarLampExt(); adicionarLog("Manual: Lamp.Ext OFF"); }
                msg = "<div class='ok'>&#10003; Lamp. Externa " +
                String(ligar ? "ligada" : "desligada") + "</div>";
            }
            else if (qual == "li") {
                if (ligar) { ligarLampInt();    adicionarLog("Manual: Lamp.Int ON"); }
                else       { desligarLampInt(); adicionarLog("Manual: Lamp.Int OFF"); }
                msg = "<div class='ok'>&#10003; Lamp. Interna " +
                String(ligar ? "ligada" : "desligada") + "</div>";
            }
        }
    }

    // ── Helper botão Liga/Desliga ──────────────────────────
    auto btnRele = [](const char* nome, const char* rel,
                      bool estado, bool bloqueado) -> String {
                          String h = "<div class='cg' style='border-color:";
                          h += bloqueado ? "#ff6f00" : (estado ? "#43a047" : "#455a64");
                          h += ";background:" + String(estado ? "#0a1f0a" : "#0d0d0d") + "'>";
                          h += "<div class='cg-row'><span class='cg-nm'>" + String(nome) + "</span>";
                          h += "<span class='cg-st' style='color:" +
                          String(bloqueado ? "#ff9800" : estado ? "#43a047" : "#78909c") + "'>";
                          h += bloqueado ? "&#9888; BLOQ" : (estado ? "&#9679; ON" : "&#9675; OFF");
                          h += "</span></div>";
                          if (!bloqueado) {
                              h += "<div style='display:flex;gap:6px;margin-top:8px'>";
                              h += "<form method='POST' action='/redes' style='flex:1'>";
                              h += "<input type='hidden' name='acao'  value='rele'>";
                              h += "<input type='hidden' name='rele'  value='" + String(rel) + "'>";
                              h += "<input type='hidden' name='valor' value='1'>";
                              h += "<button type='submit' class='btn-g btn-sm'"
                              + String(!estado ? "" : " disabled") + ">&#9679; Ligar</button></form>";
                              h += "<form method='POST' action='/redes' style='flex:1'>";
                              h += "<input type='hidden' name='acao'  value='rele'>";
                              h += "<input type='hidden' name='rele'  value='" + String(rel) + "'>";
                              h += "<input type='hidden' name='valor' value='0'>";
                              h += "<button type='submit' class='btn-r btn-sm'"
                              + String(estado ? "" : " disabled") + ">&#9675; Desligar</button></form>";
                              h += "</div>";
                          }
                          h += "</div>";
                          return h;
                      };

                      bool curtoAlt = (curto.estado == CURTO_CONFIRMADO_ALTA  || curto.redeAltaCurto);
                      bool curtoBx  = (curto.estado == CURTO_CONFIRMADO_BAIXA || curto.redeBaixaCurto);
                      bool curtoInv = (curto.estado == CURTO_CONFIRMADO_BAIXA ||
                      curto.estado == CURTO_AGUARDANDO_RESET);

                      String c = msg;

                      // ── Status de curto ────────────────────────────────────
                      c += "<h3>&#9888; Protecao de Curto</h3>";
                      c += htmlStatusCurto();

                      // ── Aviso quando IA está no comando ────────────────────
                      if (ia.config.modo == IA_MODO_IA) {
                          c += "<div class='nf' style='font-size:11px'>";
                          c += "&#129302; A IA esta ativa e controla Rede Alta, Rede Baixa "
                          "e Lampadas automaticamente por saldo de energia (Ah). "
                          "Os niveis de corte por % abaixo ficam pausados. "
                          "<a href='/ia' style='color:#90caf9'>Ver detalhes da IA &rarr;</a>"
                          "</div>";
                      }

                      // ── Controle manual de relés (sempre disponível) ───────
                      c += "<h3>&#128268; Controle Manual de Cargas</h3>";
                      c += "<div class='nf' style='font-size:10px'>"
                      "&#128161; O controle automatico continua ativo. "
                      "Acoes manuais podem ser sobrescritas pelo sistema.</div>";

                      c += btnRele("&#9889; Inversor",          "inv",
                                   estados.inversor,  curtoInv);
                      c += btnRele("&#128268; Rede Alta",       "ra",
                                   estados.redesAlta, curtoAlt);
                      c += btnRele("&#128268; Rede Baixa",      "rb",
                                   estados.redeBaixa, curtoBx);
                      c += btnRele("&#128161; Lamp. Externa",   "le",
                                   estados.lampExt,   false);
                      c += btnRele("&#128161; Lamp. Interna",   "li",
                                   estados.lampInt,   false);

                      // ── Configuração de cortes: só relevante no Modo Manual ─
                      if (ia.config.modo != IA_MODO_IA) {

                          c += "<h3>&#9889; Niveis de Corte (% Bateria)</h3>";
                          c += "<div class='nf' style='font-size:10px'>"
                          "Hierarquia: Lamp.Int &rarr; Lamp.Ext &rarr; "
                          "Rede Alta &rarr; Rede Baixa + Inversor</div>";

                          c += "<form method='POST' action='/redes'>";
                          c += "<input type='hidden' name='acao' value='config'>";

                          auto campoCorte = [&](const char* lbl, const char* nm,
                                                float val, const char* dica) -> String {
                                                    String h = "<label>" + String(lbl) + ":</label>";
                                                    h += "<div class='nf' style='font-size:10px;margin:3px 0'>"
                                                    + String(dica) + "</div>";
                                                    char vb[20];
                                                    snprintf(vb, sizeof(vb), "value='%.0f'", val);
                                                    h += "<input type='number' name='" + String(nm) +
                                                    "' min='0' max='90' step='1' required " + String(vb) + ">";
                                                    return h;
                                                };

                                                c += campoCorte("&#128161; Corte Lamp. Interna (%)",
                                                                "corteLampInt", redes.corteLampInt,
                                                                "Desliga lamp.interna abaixo deste %. "
                                                                "0% = nunca corta automaticamente");
                                                c += campoCorte("&#128161; Corte Lamp. Externa (%)",
                                                                "corteLampExt", redes.corteLampExt,
                                                                "Desliga lamp.externa por energia baixa "
                                                                "(controle solar e separado)");
                                                c += campoCorte("&#128268; Corte Rede Alta (%)",
                                                                "corteAlta", redes.corteAlta,
                                                                "Desliga equipamentos pesados abaixo deste %");
                                                c += campoCorte("&#128268; Corte Rede Baixa + Inversor (%)",
                                                                "corteBaixa", redes.corteBaixa,
                                                                "Desliga tudo. Religamento automatico ao recuperar");

                                                c += "<button type='submit' class='btn-g'>"
                                                "&#128190; Salvar Niveis</button></form>";

                                                // ── Mini-barras dos cortes atuais ──────────────────────
                                                c += "<h3>&#128202; Cortes Atuais</h3>";
                                                float pct = bateria.calibrada ? bateria.percentualAtual : 0.0f;

                                                auto miniCorte = [&](const char* nm, float corte,
                                                                     bool ativo) -> String {
                                                                         String h = "<div style='margin:6px 0'>";
                                                                         h += "<div style='display:flex;justify-content:space-between;"
                                                                         "font-size:11px;margin-bottom:2px'>";
                                                                         h += "<span style='color:#90a4ae'>" + String(nm) + "</span>";
                                                                         h += "<span style='color:" +
                                                                         String(ativo ? "#43a047" : "#ef5350") + "'>";
                                                                         h += String(ativo ? "&#9679; ON" : "&#9675; OFF") +
                                                                         " | Corte: " + String((int)corte) + "%</span></div>";
                                                                         String corBat = pct >= 50 ? "#43a047" :
                                                                         pct >= 25 ? "#ef6c00" : "#c62828";
                                                                         h += "<div style='position:relative;background:#111;"
                                                                         "border:1px solid #333;border-radius:4px;height:14px'>";
                                                                         h += "<div style='width:" + String((int)pct) +
                                                                         "%;height:100%;background:" + corBat +
                                                                         ";border-radius:3px'></div>";
                                                                         h += "<div style='position:absolute;top:0;left:" +
                                                                         String((int)corte) +
                                                                         "%;width:2px;height:100%;background:#ef5350'></div>";
                                                                         h += "</div></div>";
                                                                         return h;
                                                                     };

                                                                     c += miniCorte("Lamp. Interna", redes.corteLampInt, estados.lampInt);
                                                                     c += miniCorte("Lamp. Externa", redes.corteLampExt, estados.lampExt);
                                                                     c += miniCorte("Rede Alta",     redes.corteAlta,    estados.redesAlta);
                                                                     c += miniCorte("Rede Baixa",    redes.corteBaixa,   estados.redeBaixa);

                                                                     if (bateria.calibrada) {
                                                                         c += "<div style='font-size:11px;color:#90a4ae;text-align:right;"
                                                                         "margin-top:4px'>Bateria atual: <b style='color:#4fc3f7'>"
                                                                         + String((int)pct) + "%</b></div>";
                                                                     }
                      }

                      server.send(200, "text/html; charset=utf-8",
                                  gerarPagina("RRFS - Redes", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-SENS-WEB] PÁGINA DE SENSORES
// ═══════════════════════════════════════════════════════════
void handleSensores() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        // ── Calibração SIMPLES 1 ponto - Painel ────────────
        if (acao == "calib_painel_simples") {
            float correnteReal = server.arg("corrente_real").toFloat();
            if (correnteReal < 0 || correnteReal > 50) {
                msg = "<div class='er'>&#10007; Corrente invalida (0-50A)</div>";
            } else {
                int mvAtual = sensores.sinalPainel_mV;
                // corrBruta = -(mv-offset)/sens  =>  offset = mv + correnteReal*sens
                int novoOffset = (int)((float)mvAtual +
                correnteReal * calibPainel.sensMvA);
                calibPainel.offsetMV = novoOffset;
                prefs.begin("rrfs_cal", false);
                prefs.putInt("pn_off", novoOffset);
                prefs.end();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Calib Painel (1pt): off=%dmV (mv=%d I=%.2fA)",
                         novoOffset, mvAtual, correnteReal);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Sensor Painel calibrado! "
                "Offset=" + String(novoOffset) + "mV</div>";
            }
        }
        // ── Calibração SIMPLES 1 ponto - Casa ──────────────
        else if (acao == "calib_casa_simples") {
            float correnteReal = server.arg("corrente_real").toFloat();
            if (correnteReal < 0 || correnteReal > 100) {
                msg = "<div class='er'>&#10007; Corrente invalida (0-100A)</div>";
            } else {
                int mvAtual = sensores.sinalCasa_mV;
                // corrBruta = (mv-offset)/sens  =>  offset = mv - correnteReal*sens
                int novoOffset = (int)((float)mvAtual -
                correnteReal * calibCasa.sensMvA);
                calibCasa.offsetMV  = novoOffset;
                calibCasa.calibrado = true;
                prefs.begin("rrfs_cal", false);
                prefs.putInt("casa_off", novoOffset);
                prefs.end();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Calib Casa (1pt): off=%dmV (mv=%d I=%.2fA)",
                         novoOffset, mvAtual, correnteReal);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Sensor Casa calibrado! "
                "Offset=" + String(novoOffset) + "mV</div>";
            }
        }
        // ── Liga/desliga autoaprendizado do painel ─────────
        else if (acao == "toggle_autocal") {
            autoCalibPainelAtivo = !autoCalibPainelAtivo;
            prefs.begin("rrfs_cal", false);
            prefs.putBool("auto_cal", autoCalibPainelAtivo);
            prefs.end();
            snprintf(tempBuf, sizeof(tempBuf),
                     "AutoCalib Painel: %s",
                     autoCalibPainelAtivo ? "ATIVADO" : "DESATIVADO");
            adicionarLog(tempBuf);
            msg = "<div class='ok'>&#10003; " + String(tempBuf) + "</div>";
        }
        // ── Calibração AVANÇADA 2 pontos - Casa (mantida) ──
        else if (acao == "calib_casa") {
            float i1 = server.arg("i1").toFloat();
            float mv1 = server.arg("mv1").toFloat();
            float i2 = server.arg("i2").toFloat();
            float mv2 = server.arg("mv2").toFloat();

            if (i1 < 0 || i2 < 0 || i1 >= i2)
                msg = "<div class='er'>&#10007; Correntes invalidas (I1 < I2)</div>";
            else if (mv1 <= 0 || mv2 <= mv1)
                msg = "<div class='er'>&#10007; mV invalidos (mV1 < mV2)</div>";
            else {
                float novaSens = (mv2 - mv1) / (i2 - i1);
                float novoOff  = (int)(mv1 - novaSens * i1);
                if (novaSens < 5 || novaSens > 100)
                    msg = "<div class='er'>&#10007; Sensibilidade fora do range "
                    "(5-100 mV/A)</div>";
                else {
                    calibCasa.sensMvA   = novaSens;
                    calibCasa.offsetMV  = (int)novoOff;
                    calibCasa.calibrado = true;
                    prefs.begin("rrfs_cal", false);
                    prefs.putFloat("casa_sens", novaSens);
                    prefs.putInt  ("casa_off",  (int)novoOff);
                    prefs.end();
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Calib Casa (2pt): off=%dmV sens=%.2fmV/A "
                             "(I1=%.1fA@%dmV I2=%.1fA@%dmV)",
                             (int)novoOff, novaSens,
                             i1, (int)mv1, i2, (int)mv2);
                    adicionarLog(tempBuf);
                    msg = "<div class='ok'>&#10003; Sensor Casa calibrado! "
                    "Sens=" + String(novaSens, 2) + "mV/A "
                    "Off=" + String((int)novoOff) + "mV</div>";
                }
            }
        }
        else if (acao == "reset_calib") {
            prefs.begin("rrfs_cal", false);
            prefs.remove("casa_sens");
            prefs.remove("casa_off");
            prefs.remove("pn_off");
            prefs.remove("pn_sens");
            prefs.end();
            carregarCalibSensores();
            adicionarLog("Calibracao resetada para valores de fabrica");
            msg = "<div class='ok'>&#10003; Calibracao resetada</div>";
        }
    }

    prefs.begin("rrfs_cal", true);
    bool temCalibCustomCasa   = prefs.isKey("casa_off");
    bool temCalibCustomPainel = prefs.isKey("pn_off");
    prefs.end();

    String c = msg;

    // ── Leituras em tempo real ─────────────────────────────
    c += "<h3>&#128270; Leituras em Tempo Real</h3>";

    auto cardSensor = [](const char* titulo,
                         float corrente, int mvBruto,
                         int offsetMV, float sensMvA,
                         const char* desc) -> String {
                             String cor = corrente > 0.3f ? "#43a047" : "#546e7a";
                             String h = "<div class='sc' style='margin-bottom:8px'>";
                             h += "<div class='lbl'>" + String(titulo) + "</div>";
                             h += "<div style='display:flex;justify-content:space-between;"
                             "align-items:center'>";
                             h += "<div class='val' style='color:" + cor + ";font-size:22px'>";
                             char buf[12]; snprintf(buf, sizeof(buf), "%.2fA", corrente);
                             h += String(buf) + "</div>";
                             h += "<div style='text-align:right'>";
                             char buf2[40];
                             snprintf(buf2, sizeof(buf2), "%dmV bruto", mvBruto);
                             h += "<div class='sub'>" + String(buf2) + "</div>";
                             snprintf(buf2, sizeof(buf2), "Off:%dmV Sens:%.1fmV/A",
                                      offsetMV, sensMvA);
                             h += "<div class='sub'>" + String(buf2) + "</div>";
                             h += "</div></div>";
                             h += "<div class='sub' style='margin-top:4px'>" +
                             String(desc) + "</div>";
                             h += "</div>";
                             return h;
                         };

                         c += cardSensor("&#9728; Painel Solar (ACS758 50A Bidirecional)",
                                         sensores.correntePainel,
                                         sensores.sinalPainel_mV,
                                         calibPainel.offsetMV,
                                         calibPainel.sensMvA,
                                         "Cabos invertidos (protecao ESP32) | "
                                         "Positivo = gerando energia");

                         c += cardSensor("&#127968; Casa (ACS758LCB-100B Unidirecional)",
                                         sensores.correnteCasa,
                                         sensores.sinalCasa_mV,
                                         calibCasa.offsetMV,
                                         calibCasa.sensMvA,
                                         "Unidirecional | Offset calibravel | "
                                         "Positivo = consumindo");

                         // Sensor sistema
                         {
                             String h = "<div class='sc' style='margin-bottom:8px'>";
                             h += "<div class='lbl'>&#128268; Sistema (ESP32 + Controlador)</div>";
                             h += "<div style='display:flex;justify-content:space-between;"
                             "align-items:center'>";
                             char buf[12];
                             snprintf(buf, sizeof(buf), "%.3fA", sensores.correnteSistema);
                             String cor = sensores.correnteSistema > 0.15f ? "#ff9800" : "#546e7a";
                             h += "<div class='val' style='color:" + cor + ";font-size:22px'>";
                             h += String(buf) + "</div>";
                             h += "<div style='text-align:right'>";
                             h += "<div class='sub'>Calculado do sensor painel</div>";
                             h += "<div class='sub'>Apenas quando painel inativo</div>";
                             h += "</div></div>";
                             h += "<div class='sub' style='margin-top:4px'>"
                             "Consumo do ESP32, controlador de carga e auxiliares</div>";
                             h += "</div>";
                             c += h;
                         }

                         // Corrente de bateria (calculada)
                         {
                             float ib   = sensores.correnteBateria;
                             String cor = ib > 0.3f  ? "#43a047" :
                             ib < -0.3f ? "#ef6c00" : "#fdd835";
                             String sinal = ib > 0.3f ? "+" : ib < -0.3f ? "-" : "~";
                             String h = "<div class='sc" +
                             String(ib > 0.3f ? " ok" : ib < -0.3f ? " wn" : "") +
                             "' style='margin-bottom:8px'>";
                             h += "<div class='lbl'>&#128267; Bateria (calculado)</div>";
                             h += "<div class='val' style='color:" + cor + "'>";
                             char buf[12];
                             snprintf(buf, sizeof(buf), "%.2fA", fabsf(ib));
                             h += sinal + String(buf) + "</div>";
                             h += "<div class='sub'>";
                             h += String(ib > 0.3f ? "Carregando" :
                             ib < -0.3f ? "Descarregando" : "Em equilibrio");
                             char buf2[60];
                             snprintf(buf2, sizeof(buf2),
                                      " | Painel(%.2fA) - Casa(%.2fA) - Sist(%.3fA)",
                                      sensores.correntePainel,
                                      sensores.correnteCasa,
                                      sensores.correnteSistema);
                             h += String(buf2) + "</div></div>";
                             c += h;
                         }

                         // kW informativos
                         c += "<div class='nf' style='font-size:10px'>";
                         c += "<b>kW informativos</b> (corrente × ";
                         char bufVN[8];
                         snprintf(bufVN, sizeof(bufVN), "%.1f", bateria.tensaoNominal);
                         c += String(bufVN) + "V nominal):<br>";
                         char bufKW[120];
                         snprintf(bufKW, sizeof(bufKW),
                                  "Painel: <b>%.0fW</b> | Casa: <b>%.0fW</b> | "
                                  "Bateria: <b>%.0fW</b> | Sistema: <b>%.0fW</b>",
                                  sensores.kWPainel   * 1000.0f,
                                  sensores.kWCasa     * 1000.0f,
                                  fabsf(sensores.kWBateria) * 1000.0f,
                                  sensores.kWSistema  * 1000.0f);
                         c += String(bufKW);
                         c += "<br><span style='color:#546e7a'>"
                         "Baseado em tensao nominal configurada, nao medida. "
                         "Apenas referencia.</span></div>";

                         // ── Status Inversor ────────────────────────────────────
                         c += "<h3>&#9889; Status Inversor</h3>";
                         {
                             int mvInv = sensores.sinalInversor_mV;
                             bool invAtivo = sensores.inversoreAtivo;
                             String h = "<div class='sc " + String(invAtivo?"ok":"er") + "'>";
                             h += "<div class='cg-row'>";
                             h += "<span class='lbl'>&#9889; Sinal STATUS_INV</span>";
                             h += "<span class='cg-st' style='color:" +
                             String(invAtivo?"#43a047":"#ef5350") + "'>";
                             h += invAtivo ? "&#9679; ATIVO" : "&#9675; INATIVO";
                             h += "</span></div>";
                             char buf2[60];
                             snprintf(buf2, sizeof(buf2),
                                      "ADC: %dmV | Limiar: %dmV | Rele: %s",
                                      mvInv, INV_THRESHOLD_MV,
                                      estados.inversor ? "ON" : "OFF");
                             h += "<div class='sub' style='margin-top:4px'>"
                             + String(buf2) + "</div>";
                             h += "</div>";
                             c += h;
                         }

                         // ══════════════════════════════════════════════════════
                         // CALIBRAÇÃO SIMPLES (1 PONTO) — RECOMENDADA
                         // ══════════════════════════════════════════════════════
                         c += "<h3>&#127919; Calibracao Simples (Recomendada)</h3>";
                         c += "<div class='nf' style='font-size:10px'>"
                         "Compare o valor mostrado aqui com o do <b>controlador de carga</b> "
                         "(painel) ou do <b>alicate amperimetro</b> (casa). "
                         "Digite a corrente REAL que voce esta vendo e salve. "
                         "O sistema calcula o offset automaticamente e "
                         "<b>guarda na memoria</b> (sobrevive a reinicio).</div>";

                         // Painel
                         c += "<form method='POST' action='/sensores'>";
                         c += "<input type='hidden' name='acao' value='calib_painel_simples'>";
                         c += "<label>&#9728; Corrente real do Painel agora (A):</label>";
                         char bufAtualP[16];
                         snprintf(bufAtualP, sizeof(bufAtualP), "%.2f", sensores.correntePainel);
                         c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
                         "Sistema esta mostrando: <b>" + String(bufAtualP) + "A</b> | "
                         "ADC bruto: " + String(sensores.sinalPainel_mV) + "mV</div>";
                         c += "<input type='number' name='corrente_real' step='0.01' min='0' max='50' "
                         "required placeholder='Ex: valor do controlador de carga'>";
                         c += "<button type='submit' class='btn-g'>"
                         "&#9728; Calibrar Sensor Painel</button></form>";
                         if (temCalibCustomPainel)
                             c += "<div class='ok' style='font-size:10px;margin-top:-6px'>"
                             "&#10003; Painel com calibracao personalizada salva</div>";

                         // Casa
                         c += "<form method='POST' action='/sensores' style='margin-top:10px'>";
                         c += "<input type='hidden' name='acao' value='calib_casa_simples'>";
                         c += "<label>&#127968; Corrente real da Casa agora (A):</label>";
                         char bufAtualC[16];
                         snprintf(bufAtualC, sizeof(bufAtualC), "%.2f", sensores.correnteCasa);
                         c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
                         "Sistema esta mostrando: <b>" + String(bufAtualC) + "A</b> | "
                         "ADC bruto: " + String(sensores.sinalCasa_mV) + "mV</div>";
                         c += "<input type='number' name='corrente_real' step='0.01' min='0' max='100' "
                         "required placeholder='Ex: valor do alicate amperimetro'>";
                         c += "<button type='submit' class='btn-g'>"
                         "&#127968; Calibrar Sensor Casa</button></form>";
                         if (temCalibCustomCasa)
                             c += "<div class='ok' style='font-size:10px;margin-top:-6px'>"
                             "&#10003; Casa com calibracao personalizada salva</div>";

                         // ── Toggle autoaprendizado do painel ───────────────────
                         c += "<h3>&#129302; Autoaprendizado do Painel</h3>";
                         c += "<div class='nf' style='font-size:10px'>"
                         "O sistema tenta reaprender o zero do sensor do painel sozinho "
                         "(dia/noite). Isso pode <b>sobrescrever sua calibracao manual</b> "
                         "com o tempo. Se notar deriva depois de calibrar, "
                         "<b>desative</b> aqui.</div>";
                         c += "<div class='sc' style='text-align:center'>";
                         c += "<div class='lbl'>Status atual</div>";
                         c += "<div class='val' style='color:" +
                         String(autoCalibPainelAtivo ? "#43a047" : "#ef6c00") + "'>";
                         c += autoCalibPainelAtivo ? "ATIVADO" : "DESATIVADO";
                         c += "</div></div>";
                         c += "<form method='POST' action='/sensores'>";
                         c += "<input type='hidden' name='acao' value='toggle_autocal'>";
                         c += "<button type='submit' class='" +
                         String(autoCalibPainelAtivo ? "btn-o" : "btn-g") + "'>";
                         c += autoCalibPainelAtivo ?
                         "&#128683; Desativar Autoaprendizado" :
                         "&#9679; Reativar Autoaprendizado";
                         c += "</button></form>";

                         // ══════════════════════════════════════════════════════
                         // CALIBRAÇÃO AVANÇADA (2 PONTOS) — CASA
                         // ══════════════════════════════════════════════════════
                         c += "<h3>&#128295; Calibracao Avancada - Casa (2 Pontos)</h3>";
                         c += "<div class='nf'>";
                         c += "Use apenas se a calibracao simples nao for precisa o suficiente. "
                         "Conecte 2 cargas conhecidas diferentes e anote ADC de cada uma.<br>";
                         c += "<b>Sensor:</b> ACS758LCB-100B | "
                         "<b>Atual:</b> Off=" +
                         String(calibCasa.offsetMV) + "mV Sens=" +
                         String(calibCasa.sensMvA, 1) + "mV/A";
                         c += "</div>";

                         c += "<form method='POST' action='/sensores'>";
                         c += "<input type='hidden' name='acao' value='calib_casa'>";
                         c += "<div style='display:grid;grid-template-columns:1fr 1fr;gap:8px'>";
                         c += "<div>";
                         c += "<label>Ponto 1 - Corrente (A):</label>";
                         c += "<input type='number' name='i1' step='0.1' min='0' max='100' "
                         "required placeholder='Ex: 2.9'>";
                         c += "<label>Ponto 1 - mV lido ADC:</label>";
                         c += "<input type='number' name='mv1' step='1' min='0' max='3300' "
                         "required placeholder='Ex: 666'>";
                         c += "</div><div>";
                         c += "<label>Ponto 2 - Corrente (A):</label>";
                         c += "<input type='number' name='i2' step='0.1' min='0' max='100' "
                         "required placeholder='Ex: 34.0'>";
                         c += "<label>Ponto 2 - mV lido ADC:</label>";
                         c += "<input type='number' name='mv2' step='1' min='0' max='3300' "
                         "required placeholder='Ex: 2011'>";
                         c += "</div></div>";
                         c += "<button type='submit' class='btn-o'>"
                         "&#127919; Calibrar com 2 Pontos</button></form>";

                         if (temCalibCustomCasa || temCalibCustomPainel) {
                             c += "<form method='POST' action='/sensores'>";
                             c += "<input type='hidden' name='acao' value='reset_calib'>";
                             c += "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>"
                             "&#8635; Resetar Tudo para Valores de Fabrica</button></form>";
                         }

                         // ── Informações ADC ────────────────────────────────────
                         c += "<h3>&#128200; Info ADC ESP32</h3>";
                         c += "<div class='nf' style='font-size:10px'>";
                         c += "Resolucao: 12 bits (0-4095) | Vref: 3300mV<br>";
                         c += "Amostras por leitura: " + String(ADC_SAMPLES) + " (media)<br>";
                         c += "Buffer media movel: " + String(BUFFER_CORRENTE) + " amostras<br>";
                         c += "Filtro ruido: " + String(FILTRO_RUIDO_A, 1) + "A<br>";
                         c += "PIN Painel: GPIO" + String(PIN_SENSOR_PAINEL) +
                         " | PIN Casa: GPIO" + String(PIN_SENSOR_CASA);
                         c += "</div>";

                         server.send(200, "text/html; charset=utf-8",
                                     gerarPagina("RRFS - Sensores", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-PREV-WEB] PÁGINA DE PREVISÃO
// ═══════════════════════════════════════════════════════════
void handlePrevisao() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            String latStr = server.arg("lat");
            String lonStr = server.arg("lon");
            String ampStr = server.arg("amp");
            String efStr  = server.arg("ef");

            double lat   = latStr.toDouble();
            double lon   = lonStr.toDouble();
            float  amp   = ampStr.toFloat();
            float  efPct = efStr.toFloat();
            float  ef    = efPct / 100.0f;

            if (lat < -90.0 || lat > 90.0)
                msg = "<div class='er'>&#10007; Latitude invalida (-90 a 90)</div>";
            else if (lon < -180.0 || lon > 180.0)
                msg = "<div class='er'>&#10007; Longitude invalida (-180 a 180)</div>";
            else if (amp < 1.0f || amp > 500.0f)
                msg = "<div class='er'>&#10007; Amperagem invalida (1-500A)</div>";
            else if (efPct < 30.0f || efPct > 100.0f)
                msg = "<div class='er'>&#10007; Eficiencia invalida (30-100%)</div>";
            else {
                configLocal.latitude       = lat;
                configLocal.longitude      = lon;
                configLocal.ampControlador = amp;
                configLocal.eficiencia     = ef;
                configLocal.configurado    = true;
                configLocal.fatorCalib     = 1.0f;
                configLocal.amostrasCalib  = 0;
                salvarConfigLocal();
                snprintf(tempBuf, sizeof(tempBuf),
                         "Previsao: lat=%.8f lon=%.8f amp=%.0fA ef=%.0f%%",
                         lat, lon, amp, efPct);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Configuracao salva! "
                "Buscando previsao...</div>";
                ultimaPrev = 0;
            }
        }
        else if (acao == "buscar") {
            ultimaPrev = 0;
            adicionarLog("Previsao: busca manual solicitada");
            msg = "<div class='ok'>&#10003; Solicitado! "
            "Aguarde ate 30s e recarregue.</div>";
        }
        else if (acao == "reset_calib") {
            configLocal.fatorCalib    = 1.0f;
            configLocal.amostrasCalib = 0;
            salvarConfigLocal();
            adicionarLog("Previsao: calibracao resetada");
            msg = "<div class='ok'>&#10003; Calibracao resetada</div>";
        }
    }

    String c = msg;

    // ── Status da previsão ─────────────────────────────────
    c += "<h3>&#127774; Status da Previsao</h3>";

    if (!configLocal.configurado) {
        c += "<div class='wn'>&#9888; Localizacao nao configurada. "
        "Configure abaixo.</div>";
    } else if (!previsao.valido) {
        c += "<div class='wn'>&#9203; Aguardando dados da API "
        "Open-Meteo...</div>";
        if (strlen(previsao.erro) > 0)
            c += "<div class='er'>&#10007; Ultimo erro: " +
            String(previsao.erro) + "</div>";
    } else {

        // ── Pega hora atual ────────────────────────────────
        struct tm ti;
        bool temTime = getLocalTime(&ti);

        // ══════════════════════════════════════════════════
        // CALCULA TOTAIS: hoje restante e amanhã
        // ══════════════════════════════════════════════════
        float totalHojeRestanteAh = 0.0f;
        float totalAmanhaAh       = 0.0f;
        int   horasHojeRest       = 0;
        int   horasAmanha         = 0;

        // Amanhã: dia seguinte
        struct tm tiAmanha = {0};
        if (temTime) {
            tiAmanha = ti;
            tiAmanha.tm_mday += 1;
            tiAmanha.tm_hour  = 0;
            tiAmanha.tm_min   = 0;
            tiAmanha.tm_sec   = 0;
            mktime(&tiAmanha);
        }

        for (uint8_t i = 0; i < previsao.totalHoras; i++) {
            PrevisaoHora& ph = previsao.horas[i];
            if (!ph.valido) continue;

            if (temTime) {
                // Hoje restante: mesma data e hora >= atual
                bool eHoje = (ph.dia  == (uint8_t)ti.tm_mday &&
                ph.mes  == (uint8_t)(ti.tm_mon+1) &&
                ph.ano  == (uint16_t)(ti.tm_year+1900));
                if (eHoje && ph.hora >= (uint8_t)ti.tm_hour) {
                    // Ah = corrente estimada × 1h (cada entrada = 1 hora)
                    totalHojeRestanteAh += ph.correnteEstimada;
                    horasHojeRest++;
                }

                // Amanhã: dia seguinte completo
                bool eAmanha = (ph.dia == (uint8_t)tiAmanha.tm_mday &&
                ph.mes == (uint8_t)(tiAmanha.tm_mon+1) &&
                ph.ano == (uint16_t)(tiAmanha.tm_year+1900));
                if (eAmanha) {
                    totalAmanhaAh += ph.correnteEstimada;
                    horasAmanha++;
                }
            }
        }

        // Média por hora (para referência)
        float mediaHojeA  = (horasHojeRest > 0)
        ? (totalHojeRestanteAh / horasHojeRest) : 0;
        float mediaAmanhaA = (horasAmanha > 0)
        ? (totalAmanhaAh / horasAmanha) : 0;

        // ── Status geral ───────────────────────────────────
        c += "<div class='sc ok'>";
        c += "<div class='lbl'>Previsao disponivel</div>";
        char bufPrev[80];
        snprintf(bufPrev, sizeof(bufPrev),
                 "%d horas de dados | Calib: %.3f (%d amostras)",
                 previsao.totalHoras,
                 configLocal.fatorCalib,
                 configLocal.amostrasCalib);
        c += "<div class='val' style='font-size:13px'>"
        + String(bufPrev) + "</div>";
        unsigned long secsAtras =
        (millis() - previsao.ultimaAtualizacao) / 1000;
        char bufUlt[30];
        snprintf(bufUlt, sizeof(bufUlt), "Ha %lus", secsAtras);
        c += "<div class='sub'>Atualizado: " + String(bufUlt) + "</div>";
        c += "</div>";

        // ══════════════════════════════════════════════════
        // CARDS: Hoje restante + Amanhã (com total Ah)
        // ══════════════════════════════════════════════════
        c += "<div style='display:grid;grid-template-columns:1fr 1fr;"
        "gap:6px;margin:8px 0'>";

        // Card: Hoje restante
        {
            String h = "<div class='sc' style='border-color:#fdd835'>";
            h += "<div class='lbl'>&#9728; Hoje - Restante</div>";

            // Total Ah em destaque
            char bufAh[20];
            snprintf(bufAh, sizeof(bufAh), "%.1fAh", totalHojeRestanteAh);
            h += "<div style='font-size:22px;font-weight:bold;"
            "color:#fdd835;text-align:center'>"
            + String(bufAh) + "</div>";

            // Detalhes
            h += "<div style='font-size:10px;color:#78909c;"
            "text-align:center;margin-top:4px'>";
            char bufDet[60];
            snprintf(bufDet, sizeof(bufDet),
                     "Media: %.1fA | %dh restantes",
                     mediaHojeA, horasHojeRest);
            h += String(bufDet) + "<br>";

            // kWh informativo
            char bufKWh[30];
            snprintf(bufKWh, sizeof(bufKWh), "~%.2fkWh @ %.1fV nom",
                     (totalHojeRestanteAh * bateria.tensaoNominal) / 1000.0f,
                     bateria.tensaoNominal);
            h += String(bufKWh);
            h += "</div></div>";
            c += h;
        }

        // Card: Amanhã
        {
            String h = "<div class='sc' style='border-color:#4fc3f7'>";
            h += "<div class='lbl'>&#127774; Amanha - Total</div>";

            // Total Ah em destaque
            char bufAh[20];
            snprintf(bufAh, sizeof(bufAh), "%.1fAh", totalAmanhaAh);
            h += "<div style='font-size:22px;font-weight:bold;"
            "color:#4fc3f7;text-align:center'>"
            + String(bufAh) + "</div>";

            // Detalhes
            h += "<div style='font-size:10px;color:#78909c;"
            "text-align:center;margin-top:4px'>";
            char bufDet[60];
            snprintf(bufDet, sizeof(bufDet),
                     "Media: %.1fA | %dh previstas",
                     mediaAmanhaA, horasAmanha);
            h += String(bufDet) + "<br>";

            // kWh informativo
            char bufKWh[30];
            snprintf(bufKWh, sizeof(bufKWh), "~%.2fkWh @ %.1fV nom",
                     (totalAmanhaAh * bateria.tensaoNominal) / 1000.0f,
                     bateria.tensaoNominal);
            h += String(bufKWh);
            h += "</div></div>";
            c += h;
        }
        c += "</div>";

        // ── IA usando previsão ─────────────────────────────
        // Mostra se IA está considerando a previsão nas decisões
        if (ia.config.modo == IA_MODO_IA && bateria.calibrada) {
            c += "<div class='nf' style='font-size:10px'>";
            c += "&#129302; <b>IA usando previsao:</b> ";
            char bufIA[120];
            snprintf(bufIA, sizeof(bufIA),
                     "Saldo atual: <b style='color:%s'>%+.1fAh</b> | "
                     "Nec. ate amanhecer: <b>%.1fAh</b> | "
                     "Geracao prevista: <b style='color:#fdd835'>%.1fAh</b>",
                     ia.saldoAhAtual >= 0 ? "#43a047" : "#ef5350",
                     ia.saldoAhAtual,
                     ia.ahNecAmanhecerAtual,
                     totalHojeRestanteAh);
            c += String(bufIA) + "</div>";
        }

        // Decisões automáticas
        if (decisao.desligarRedeAltaNoite ||
            decisao.desligarLampExtNoite  ||
            decisao.alertaEnergiaInsuficiente) {
            c += "<div class='wn' style='font-size:11px'>";
        c += "<b>&#9888; Decisoes ativas:</b><br>";
        if (decisao.desligarRedeAltaNoite)
            c += "&#128268; Rede Alta OFF (noite - energia insuf.)<br>";
            if (decisao.desligarLampExtNoite)
                c += "&#128161; Lamp.Ext OFF (energia critica)<br>";
            if (decisao.alertaEnergiaInsuficiente)
                c += "&#128267; ALERTA: Energia pode ser insuficiente!<br>";
            if (strlen(decisao.motivo) > 0)
                c += "<span style='color:#78909c'>"
                + String(decisao.motivo) + "</span>";
            c += "</div>";
            }

            // ══════════════════════════════════════════════════
            // TABELA: Próximas 24h com Ah por hora
            // ══════════════════════════════════════════════════
            c += "<h3>&#128200; Proximas 24 Horas</h3>";
            c += "<div class='nf' style='font-size:10px'>"
            "&#128161; <b>Est.(A)</b> = corrente media prevista "
            "naquele horario. "
            "<b>Est.(Ah)</b> = energia gerada em 1 hora "
            "(corrente × 1h). Exemplo: 9.71A das 13h-14h = 9.71Ah.</div>";

            c += "<div style='overflow-x:auto'>";
            c += "<table style='width:100%;border-collapse:collapse;"
            "font-size:11px'>";
            c += "<tr style='color:#546e7a'>";
            c += "<th style='text-align:left;padding:4px'>Hora</th>";
            c += "<th style='padding:4px'>&#9728; Rad</th>";
            c += "<th style='padding:4px'>&#9729; Nuv</th>";
            c += "<th style='padding:4px'>&#127777; Temp</th>";
            c += "<th style='padding:4px'>&#9889; Est.(A)</th>";
            c += "<th style='padding:4px'>&#9889; Est.(Ah)</th>";
            c += "</tr>";

            int exibidos = 0;
            // Acumulador para barra de progresso visual
            float maxCorrTabela = 0.1f;
            // Primeiro passa: acha o máximo para escala
            for (uint8_t i = 0; i < previsao.totalHoras; i++) {
                if (previsao.horas[i].valido &&
                    previsao.horas[i].correnteEstimada > maxCorrTabela)
                    maxCorrTabela = previsao.horas[i].correnteEstimada;
            }

            for (uint8_t i = 0; i < previsao.totalHoras && exibidos < 24; i++) {
                PrevisaoHora& ph = previsao.horas[i];
                if (!ph.valido) continue;

                // Só horários futuros
                if (temTime && ph.dia == ti.tm_mday &&
                    ph.mes == (uint8_t)(ti.tm_mon+1) &&
                    ph.hora < (uint8_t)ti.tm_hour) continue;

                exibidos++;
                bool eHoraAtual = temTime &&
                ph.hora == (uint8_t)ti.tm_hour &&
                ph.dia  == ti.tm_mday;
                bool eAmanha2   = temTime &&
                ph.dia == (uint8_t)tiAmanha.tm_mday &&
                ph.mes == (uint8_t)(tiAmanha.tm_mon+1);

                // Cor de fundo da linha
                String bgLinha = eHoraAtual ? "#0a1f0a" :
                eAmanha2   ? "#0a0a1f" : "";

                c += "<tr style='border-top:1px solid #222;background:"
                + bgLinha + "'>";

                // Hora
                char bufH[16];
                snprintf(bufH, sizeof(bufH), "%02d:00", ph.hora);
                c += "<td style='padding:4px;color:" +
                String(eHoraAtual ? "#43a047" :
                eAmanha2   ? "#90caf9" : "#90a4ae") + "'>";
                if (eHoraAtual) c += "<b>";
                c += String(bufH);
                if (eHoraAtual) c += " &#9664;</b>";
                // Marca início do dia seguinte
                if (eAmanha2 && ph.hora == 0)
                    c += " <span style='font-size:9px;color:#546e7a'>"
                    "(amanha)</span>";
                c += "</td>";

                // Radiação
                char bufR[10];
                snprintf(bufR, sizeof(bufR), "%.0f", ph.radiacao);
                String corRad = ph.radiacao > 500 ? "#fdd835" :
                ph.radiacao > 200 ? "#ef6c00" :
                ph.radiacao > 0   ? "#78909c" : "#333";
                c += "<td style='padding:4px;text-align:center;color:"
                + corRad + "'>" + String(bufR) + "</td>";

                // Nuvens
                char bufN[10];
                snprintf(bufN, sizeof(bufN), "%.0f%%", ph.coberturaNuvens);
                String corNuv = ph.coberturaNuvens > 80 ? "#546e7a" :
                ph.coberturaNuvens > 40 ? "#78909c" : "#90a4ae";
                c += "<td style='padding:4px;text-align:center;color:"
                + corNuv + "'>" + String(bufN) + "</td>";

                // Temperatura
                char bufT[10];
                snprintf(bufT, sizeof(bufT), "%.0f&deg;", ph.temperatura);
                c += "<td style='padding:4px;text-align:center;"
                "color:#78909c'>" + String(bufT) + "</td>";

                // Corrente estimada (A) com mini-barra
                char bufC[10];
                snprintf(bufC, sizeof(bufC), "%.2fA", ph.correnteEstimada);
                String corC = ph.correnteEstimada > 15  ? "#43a047" :
                ph.correnteEstimada > 8   ? "#fdd835" :
                ph.correnteEstimada > 2   ? "#ef6c00" :
                ph.correnteEstimada > 0.1f? "#78909c" : "#333";
                float pctBarra = (ph.correnteEstimada / maxCorrTabela) * 100.0f;
                if (pctBarra > 100) pctBarra = 100;

                c += "<td style='padding:4px;text-align:center'>";
                c += "<div style='font-weight:bold;color:" + corC + "'>"
                + String(bufC) + "</div>";
                // Mini barra visual
                if (ph.correnteEstimada > 0.1f) {
                    c += "<div style='background:#1a1a1a;border-radius:2px;"
                    "height:4px;margin-top:2px;overflow:hidden'>";
                    c += "<div style='width:" + String((int)pctBarra) +
                    "%;height:100%;background:" + corC +
                    ";border-radius:2px'></div></div>";
                }
                c += "</td>";

                // Ah estimado (corrente × 1h = Ah nessa hora)
                // Esta é a energia que será gerada das HH:00 às HH+1:00
                char bufAh[12];
                snprintf(bufAh, sizeof(bufAh), "%.2fAh",
                         ph.correnteEstimada); // × 1h = mesmo valor
                String corAh = ph.correnteEstimada > 15  ? "#43a047" :
                ph.correnteEstimada > 8   ? "#fdd835" :
                ph.correnteEstimada > 2   ? "#ef6c00" :
                ph.correnteEstimada > 0.1f? "#78909c" : "#333";
                c += "<td style='padding:4px;text-align:center;"
                "font-weight:bold;color:" + corAh + "'>";
                if (ph.correnteEstimada > 0.1f)
                    c += String(bufAh);
                else
                    c += "<span style='color:#333'>-</span>";
                c += "</td>";

                c += "</tr>";
            }
            c += "</table></div>";

            // ── Nota explicativa ───────────────────────────────
            c += "<div class='nf' style='font-size:10px;margin-top:6px'>";
            c += "&#128161; <b>Como ler a tabela:</b><br>";
            c += "Est.(A) = corrente media prevista naquele horario<br>";
            c += "Est.(Ah) = energia gerada entre HH:00 e HH+1:00 "
            "(corrente × 1 hora)<br>";
            c += "Exemplo: 13:00 com 9.71A = gera <b>9.71Ah</b> "
            "entre 13h e 14h";
            c += "</div>";

            // Calibração
            c += "<div class='nf' style='font-size:10px;margin-top:6px'>";
            c += "&#127919; <b>Calibracao automatica:</b> ";
            c += "Compara A previsto vs real (quando painel > 1A). ";
            snprintf(tempBuf, sizeof(tempBuf),
                     "Fator: <b>%.4f</b> | Amostras: %d/50",
                     configLocal.fatorCalib,
                     configLocal.amostrasCalib);
            c += String(tempBuf) + "</div>";

            if (configLocal.amostrasCalib > 0) {
                c += "<form method='POST' action='/previsao'>";
                c += "<input type='hidden' name='acao' value='reset_calib'>";
                c += "<button type='submit' class='btn-s btn-sm'>"
                "&#8635; Resetar Calibracao</button></form>";
            }
    }

    // ── Configuração de localização ────────────────────────
    c += "<h3>&#127759; Configurar Localizacao</h3>";
    c += "<div class='nf' style='font-size:10px'>"
    "&#128161; <b>Como obter coordenadas de alta precisao:</b><br>"
    "1. Abra o Google Maps no local do sistema<br>"
    "2. Clique com botao direito &rarr; copie as coordenadas<br>"
    "3. Cole aqui (ex: -23.48049849577555, -46.63508951102103)<br>"
    "Quanto mais casas decimais, mais precisa e a previsao solar!</div>";

    c += "<form method='POST' action='/previsao'>";
    c += "<input type='hidden' name='acao' value='config'>";

    char bufLat[32], bufLon[32];
    if (configLocal.configurado) {
        snprintf(bufLat, sizeof(bufLat), "%.14f", configLocal.latitude);
        snprintf(bufLon, sizeof(bufLon), "%.14f", configLocal.longitude);
    } else {
        snprintf(bufLat, sizeof(bufLat), "%.14f", -15.78000000000000);
        snprintf(bufLon, sizeof(bufLon), "%.14f", -47.93000000000000);
    }

    c += "<label>Latitude:</label>";
    c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
    "Negativo = Sul (Brasil). Ex: -23.48049849577555</div>";
    c += "<input type='number' name='lat' step='any' "
    "min='-90' max='90' required value='"
    + String(bufLat) + "' placeholder='Ex: -23.48049849577555'>";

    c += "<label>Longitude:</label>";
    c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
    "Negativo = Oeste (Brasil). Ex: -46.63508951102103</div>";
    c += "<input type='number' name='lon' step='any' "
    "min='-180' max='180' required value='"
    + String(bufLon) + "' placeholder='Ex: -46.63508951102103'>";

    char bufAmp[12];
    snprintf(bufAmp, sizeof(bufAmp), "%.0f",
             configLocal.configurado ? configLocal.ampControlador : 30.0f);
    c += "<label>Amperagem do Controlador de Carga (A):</label>";
    c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
    "Corrente maxima do seu controlador (ex: 30, 60, 100A)</div>";
    c += "<input type='number' name='amp' step='1' "
    "min='1' max='500' required value='" + String(bufAmp) +
    "' placeholder='Ex: 30'>";

    char bufEf[8];
    snprintf(bufEf, sizeof(bufEf), "%.0f",
             configLocal.configurado
             ? configLocal.eficiencia * 100.0f : 80.0f);
    c += "<label>Eficiencia do Sistema (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
    "Perdas por cabos, MPPT e temperatura. "
    "Tipico: 70-85% para sistemas residenciais</div>";
    c += "<input type='number' name='ef' step='1' "
    "min='30' max='100' required value='" + String(bufEf) +
    "' placeholder='Ex: 80'>";

    c += "<button type='submit' class='btn-g'>"
    "&#128190; Salvar e Buscar Previsao</button></form>";

    if (configLocal.configurado) {
        c += "<form method='POST' action='/previsao'>";
        c += "<input type='hidden' name='acao' value='buscar'>";
        c += "<button type='submit' class='btn-o btn-sm' "
        "style='margin-top:6px'>"
        "&#8635; Buscar Previsao Agora</button></form>";

        c += "<div class='nf' style='font-size:10px;margin-top:8px'>";
        char bufInfo[120];
        snprintf(bufInfo, sizeof(bufInfo),
                 "&#127759; Local: lat=<b>%.8f</b> "
                 "lon=<b>%.8f</b><br>"
                 "Controlador: <b>%.0fA</b> | "
                 "Eficiencia: <b>%.0f%%</b>",
                 configLocal.latitude,
                 configLocal.longitude,
                 configLocal.ampControlador,
                 configLocal.eficiencia * 100.0f);
        c += String(bufInfo) + "</div>";
    }

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Previsao", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-HIST-WEB] PÁGINA DE HISTÓRICO
// ═══════════════════════════════════════════════════════════
void handleHistorico() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String c = "";

    // ── Contadores globais ─────────────────────────────────
    c += "<h3>&#128200; Totais Desde o Inicio</h3>";
    c += "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px'>";

    auto cardTotal = [&](const char* ico, const char* nm,
                         float ah, const char* cor) -> String {
                             String h = "<div class='sc' style='text-align:center'>";
                             h += "<div class='lbl'>" + String(ico) + " " + String(nm) + "</div>";
                             char buf[20];
                             snprintf(buf, sizeof(buf), "%.1fAh", ah);
                             h += "<div class='val' style='color:" + String(cor) + "'>";
                             h += String(buf) + "</div>";
                             // kWh informativo
                             char buf2[24];
                             snprintf(buf2, sizeof(buf2), "%.2fkWh",
                                      (ah * bateria.tensaoNominal) / 1000.0f);
                             h += "<div class='sub'>~" + String(buf2) + "</div>";
                             h += "</div>";
                             return h;
                         };

                         c += cardTotal("&#9728;", "Painel Gerado",
                                        contadores.painelAh, "#fdd835");
                         c += cardTotal("&#127968;", "Casa Consumido",
                                        contadores.casaAh,   "#4fc3f7");
                         c += cardTotal("&#9650;", "Bat. Carregada",
                                        contadores.batCargAh,"#43a047");
                         c += cardTotal("&#9660;", "Bat. Descarregada",
                                        contadores.batDescAh,"#ef6c00");
                         c += "</div>";

                         c += "<div class='nf' style='font-size:10px'>"
                         "kWh calculado com tensao nominal de " +
                         String(bateria.tensaoNominal, 1) +
                         "V (informativo). Valores em Ah sao os reais medidos.</div>";

                         // ── Resumo dos últimos 7 dias ──────────────────────────
                         c += "<h3>&#128197; Ultimos 7 Dias</h3>";

                         int maxDias = min((int)histTotalDias, 7);
                         if (maxDias == 0) {
                             c += "<div class='nf'>Sem historico ainda. "
                             "Aguarde pelo menos 1 hora.</div>";
                         } else {
                             c += "<div style='overflow-x:auto'>";
                             c += "<table style='width:100%;border-collapse:collapse;"
                             "font-size:11px'>";
                             c += "<tr style='color:#546e7a'>";
                             c += "<th style='text-align:left;padding:4px'>Data</th>";
                             c += "<th style='padding:4px'>&#9728; Painel</th>";
                             c += "<th style='padding:4px'>&#127968; Casa</th>";
                             c += "<th style='padding:4px'>&#9650; Bat+</th>";
                             c += "<th style='padding:4px'>&#9660; Bat-</th>";
                             c += "</tr>";

                             for (int i = 0; i < maxDias; i++) {
                                 RegistroDia dTemp;
                                 bool ok;
                                 if (i == 0) { dTemp = histDiaAtual; ok = true; }
                                 else {
                                     int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
                                     ok = carregarDiaFlash(idx, dTemp);
                                 }
                                 if (!ok || !dTemp.valido) continue;

                                 // Adiciona acumuladores do dia atual (parcial)
                                 float pAh = dTemp.totalPainelAh;
                                 float cAh = dTemp.totalCasaAh;
                                 float bCAh= dTemp.totalBatCargAh;
                                 float bDAh= dTemp.totalBatDescAh;
                                 if (i == 0) {
                                     pAh  += acum.painelAh;
                                     cAh  += acum.casaAh;
                                     bCAh += acum.batCargAh;
                                     bDAh += acum.batDescAh;
                                 }

                                 char bufData[16];
                                 snprintf(bufData, sizeof(bufData), "%02d/%02d/%04d",
                                          dTemp.dia, dTemp.mes, dTemp.ano);

                                 bool ehHoje = (i == 0);
                                 c += "<tr style='border-top:1px solid #222;"
                                 "background:" + String(ehHoje?"#0a1f0a":"") + "'>";
                                 c += "<td style='padding:4px;color:" +
                                 String(ehHoje?"#43a047":"#90a4ae") + "'>";
                                 if (ehHoje) c += "<b>";
                                 c += String(bufData);
                                 if (ehHoje) c += " (hoje)</b>";
                                 c += "</td>";

                                 // Painel
                                 char buf[16];
                                 snprintf(buf, sizeof(buf), "%.1f", pAh);
                                 c += "<td style='padding:4px;text-align:center;color:#fdd835'>"
                                 + String(buf) + "Ah</td>";

                                 // Casa
                                 snprintf(buf, sizeof(buf), "%.1f", cAh);
                                 c += "<td style='padding:4px;text-align:center;color:#4fc3f7'>"
                                 + String(buf) + "Ah</td>";

                                 // Bat+
                                 snprintf(buf, sizeof(buf), "%.1f", bCAh);
                                 c += "<td style='padding:4px;text-align:center;color:#43a047'>"
                                 + String(buf) + "Ah</td>";

                                 // Bat-
                                 snprintf(buf, sizeof(buf), "%.1f", bDAh);
                                 c += "<td style='padding:4px;text-align:center;color:#ef6c00'>"
                                 + String(buf) + "Ah</td>";

                                 c += "</tr>";
                             }
                             c += "</table></div>";
                         }

                         // ── Gráfico por hora do dia atual ─────────────────────
                         c += "<h3>&#128200; Hoje - Por Hora (Ah)</h3>";

                         {
                             // Encontra o maior valor para escalar
                             float maxAh = 0.1f;
                             for (int h = 0; h < 24; h++) {
                                 float v = histDiaAtual.horas[h].painelAh +
                                 histDiaAtual.horas[h].casaAh;
                                 if (v > maxAh) maxAh = v;
                             }

                             struct tm ti; int horaAtual = -1;
                             if (getLocalTime(&ti)) horaAtual = ti.tm_hour;

                             c += "<div style='overflow-x:auto'>";
                             c += "<div style='display:flex;align-items:flex-end;gap:2px;"
                             "height:80px;padding:4px 0'>";

                             for (int h = 0; h < 24; h++) {
                                 float pAh = histDiaAtual.horas[h].painelAh;
                                 float cAh = histDiaAtual.horas[h].casaAh;
                                 // Adiciona parcial da hora atual
                                 if (h == horaAtual) {
                                     pAh += acum.painelAh;
                                     cAh += acum.casaAh;
                                 }

                                 float altP = (pAh / maxAh) * 70.0f;
                                 float altC = (cAh / maxAh) * 70.0f;
                                 if (altP > 70) altP = 70;
                                 if (altC > 70) altC = 70;

                                 bool ehAtual = (h == horaAtual);
                                 c += "<div style='flex:1;display:flex;flex-direction:column;"
                                 "align-items:center;min-width:8px'>";
                                 // Barra painel
                                 c += "<div title='" + String(h) + "h Painel:" +
                                 String(pAh, 1) + "Ah' style='width:100%;height:" +
                                 String((int)altP) + "px;background:" +
                                 String(ehAtual ? "#fdd835" : "#a89420") +
                                 ";border-radius:2px 2px 0 0;'></div>";
                                 // Barra casa
                                 c += "<div title='" + String(h) + "h Casa:" +
                                 String(cAh, 1) + "Ah' style='width:100%;height:" +
                                 String((int)altC) + "px;background:" +
                                 String(ehAtual ? "#4fc3f7" : "#1e5f7a") +
                                 ";border-radius:0;'></div>";
                                 // Label hora
                                 if (h % 4 == 0 || ehAtual) {
                                     c += "<div style='font-size:7px;color:#546e7a'>"
                                     + String(h) + "h</div>";
                                 } else {
                                     c += "<div style='font-size:7px;color:transparent'>.</div>";
                                 }
                                 c += "</div>";
                             }
                             c += "</div>";

                             // Legenda
                             c += "<div style='display:flex;gap:12px;font-size:10px;"
                             "color:#78909c;margin-top:4px'>";
                             c += "<span><span style='color:#fdd835'>&#9632;</span> Painel (Ah)</span>";
                             c += "<span><span style='color:#4fc3f7'>&#9632;</span> Casa (Ah)</span>";
                             c += "</div>";
                             c += "</div>";
                         }

                         // ── Gráfico de corrente em tempo real (últimos 60s) ────
                         c += "<h3>&#128200; Corrente em Tempo Real (60s)</h3>";
                         {
                             int total = histSnapFull ? HIST_SNAP_PONTOS : histSnapIdx;
                             if (total < 2) {
                                 c += "<div class='nf'>Aguardando dados... (min 2s)</div>";
                             } else {
                                 float maxC = 0.5f;
                                 for (int i = 0; i < total; i++) {
                                     int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
                                     if (histCorrPainel[idx] > maxC) maxC = histCorrPainel[idx];
                                     if (histCorrCasa[idx]   > maxC) maxC = histCorrCasa[idx];
                                     if (fabsf(histCorrBat[idx]) > maxC) maxC = fabsf(histCorrBat[idx]);
                                 }

                                 c += "<div style='position:relative;height:80px;"
                                 "background:#111;border:1px solid #333;border-radius:4px;"
                                 "overflow:hidden'>";

                                 // Linha zero no meio (para bateria)
                                 c += "<div style='position:absolute;top:50%;left:0;right:0;"
                                 "height:1px;background:#333'></div>";

                                 // SVG das linhas
                                 c += "<svg viewBox='0 0 " + String(total) + " 80' "
                                 "preserveAspectRatio='none' "
                                 "style='width:100%;height:100%;position:absolute;top:0;left:0'>";

                                 auto pontoY = [&](float val, bool bipolar) -> float {
                                     if (bipolar) return 40.0f - (val / maxC) * 38.0f;
                                     else         return 78.0f - (val / maxC) * 76.0f;
                                 };

                                     // Linha painel
                                     String pathP = "M";
                                     for (int i = 0; i < total; i++) {
                                         int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
                                         char pt[20];
                                         snprintf(pt, sizeof(pt), "%d,%.1f", i,
                                                  pontoY(histCorrPainel[idx], false));
                                         if (i > 0) pathP += " L";
                                         pathP += String(pt);
                                     }
                                     c += "<path d='" + pathP + "' fill='none' "
                                     "stroke='#fdd835' stroke-width='1.5'/>";

                                     // Linha casa
                                     String pathC = "M";
                                     for (int i = 0; i < total; i++) {
                                         int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
                                         char pt[20];
                                         snprintf(pt, sizeof(pt), "%d,%.1f", i,
                                                  pontoY(histCorrCasa[idx], false));
                                         if (i > 0) pathC += " L";
                                         pathC += String(pt);
                                     }
                                     c += "<path d='" + pathC + "' fill='none' "
                                     "stroke='#4fc3f7' stroke-width='1.5'/>";

                                     // Linha bateria (bipolar)
                                     String pathB = "M";
                                     for (int i = 0; i < total; i++) {
                                         int idx = histSnapFull ? (histSnapIdx+i)%HIST_SNAP_PONTOS : i;
                                         char pt[20];
                                         snprintf(pt, sizeof(pt), "%d,%.1f", i,
                                                  pontoY(histCorrBat[idx], true));
                                         if (i > 0) pathB += " L";
                                         pathB += String(pt);
                                     }
                                     c += "<path d='" + pathB + "' fill='none' "
                                     "stroke='#ce93d8' stroke-width='1' stroke-dasharray='3 2'/>";

                                     c += "</svg>";

                                     // Escala
                                     char bufMax[12];
                                     snprintf(bufMax, sizeof(bufMax), "%.1fA", maxC);
                                     c += "<div style='position:absolute;top:2px;right:4px;"
                                     "font-size:9px;color:#546e7a'>" + String(bufMax) + "</div>";
                                     c += "<div style='position:absolute;bottom:2px;right:4px;"
                                     "font-size:9px;color:#546e7a'>0A</div>";
                                     c += "</div>";

                                     // Legenda gráfico RT
                                     c += "<div style='display:flex;gap:10px;font-size:10px;"
                                     "color:#78909c;margin-top:4px'>";
                                     c += "<span><span style='color:#fdd835'>&#9660;</span> Painel</span>";
                                     c += "<span><span style='color:#4fc3f7'>&#9660;</span> Casa</span>";
                                     c += "<span><span style='color:#ce93d8'>---</span> Bateria</span>";
                                     char bufAtuais[80];
                                     snprintf(bufAtuais, sizeof(bufAtuais),
                                              " | Agora: P=%.1fA C=%.1fA B=%+.1fA",
                                              sensores.correntePainel,
                                              sensores.correnteCasa,
                                              sensores.correnteBateria);
                                     c += "<span style='color:#90a4ae'>" + String(bufAtuais) + "</span>";
                                     c += "</div>";
                             }
                         }

                         // ── Info histórico ─────────────────────────────────────
                         c += "<div class='nf' style='font-size:10px;margin-top:8px'>";
                         char bufInfo[80];
                         snprintf(bufInfo, sizeof(bufInfo),
                                  "Historico: %d/%d dias gravados | idx=%d",
                                  histTotalDias, HIST_DIAS, histIndiceAtual);
                         c += String(bufInfo);
                         c += " | Versao de dados: v" + String(HIST_KEY_VER);
                         c += "</div>";

                         server.send(200, "text/html; charset=utf-8",
                                     gerarPagina("RRFS - Historico", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-CFG-WEB] PÁGINA DE CONFIGURAÇÃO GERAL
// ═══════════════════════════════════════════════════════════
void handleConfig() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "senha") {
            String passAtual = server.arg("pass_atual");
            String passNova  = server.arg("pass_nova");
            String passConf  = server.arg("pass_conf");
            prefs.begin("rrfs", true);
            String user = prefs.getString("user", "");
            prefs.end();
            if (!verificarLogin(user, passAtual))
                msg = "<div class='er'>&#10007; Senha atual incorreta</div>";
            else if (passNova.length() < 4)
                msg = "<div class='er'>&#10007; Nova senha: minimo 4 chars</div>";
            else if (passNova != passConf)
                msg = "<div class='er'>&#10007; Senhas nao conferem</div>";
            else {
                salvarCredenciais(user, passNova);
                adicionarLog("Senha alterada");
                msg = "<div class='ok'>&#10003; Senha alterada!</div>";
            }
        }
        // ── Limpar historico e contadores ──────────────────
        else if (acao == "reset_historico") {
            String confirmacao = server.arg("confirmar");
            if (confirmacao != "SIM") {
                msg = "<div class='er'>&#10007; Confirmacao invalida. "
                "Nada foi apagado.</div>";
            } else {
                contadores.painelAh  = 0;
                contadores.casaAh    = 0;
                contadores.batCargAh = 0;
                contadores.batDescAh = 0;
                salvarContadores();

                memset(&acum, 0, sizeof(AcumHora));
                acum.horaAtual  = 255;
                acum.ultimoAcum = 0;

                RegistroDia diaVazio;
                memset(&diaVazio, 0, sizeof(RegistroDia));
                for (uint16_t i = 0; i < HIST_DIAS; i++) {
                    salvarDiaFlash(i, diaVazio);
                }

                histIndiceAtual = 0;
                histTotalDias   = 0;
                memset(&histDiaAtual, 0, sizeof(RegistroDia));
                salvarIndiceHistorico();

                for (int i = 0; i < HIST_SNAP_PONTOS; i++) {
                    histCorrPainel[i] = 0;
                    histCorrCasa[i]   = 0;
                    histCorrBat[i]    = 0;
                }
                histSnapIdx  = 0;
                histSnapFull = false;

                adicionarLog("HISTORICO E CONTADORES apagados pelo usuario");
                msg = "<div class='ok'>&#10003; Historico e contadores "
                "apagados! Novo historico iniciado do zero.</div>";
            }
        }
    }

    String c = msg;

    // ── Info sistema ───────────────────────────────────────
    c += "<h3>&#128202; Sistema</h3>";
    c += "<div class='sc'>";
    char bufSys[120];
    snprintf(bufSys, sizeof(bufSys),
             "RRFS Energia v15.0 | RAM: %.1fKB | Up: %lus",
             esp_get_free_heap_size() / 1024.0f,
             millis() / 1000UL);
    c += "<div class='val' style='font-size:12px'>"
    + String(bufSys) + "</div>";
    String wifi = modoAP ?
    "AP: 10.0.0.1" :
    "STA: " + WiFi.localIP().toString() +
    " " + String(WiFi.RSSI()) + "dBm";
    c += "<div class='sub'>" + wifi + "</div>";
    c += "</div>";

    struct tm ti;
    if (getLocalTime(&ti)) {
        char bufHora[40];
        snprintf(bufHora, sizeof(bufHora),
                 "%02d/%02d/%04d %02d:%02d:%02d (UTC-3)",
                 ti.tm_mday, ti.tm_mon+1, ti.tm_year+1900,
                 ti.tm_hour, ti.tm_min, ti.tm_sec);
        c += "<div class='nf' style='font-size:11px;text-align:center'>"
        "&#128336; " + String(bufHora) + "</div>";
    } else {
        c += "<div class='wn' style='font-size:11px'>"
        "&#9888; NTP nao sincronizado</div>";
    }

    // ── Atalho para a página da IA ─────────────────────────
    c += "<h3>&#129302; Inteligencia Artificial</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "Modo atual: <b>" +
    String(ia.config.modo == IA_MODO_IA ? "IA ATIVA" : "MANUAL") +
    "</b> | Padroes aprendidos: " + String(ia.totalPadroes) +
    "/" + String(IA_MAX_PADROES);
    c += "</div>";
    c += "<form method='GET' action='/ia'>";
    c += "<button type='submit' class='btn-g'>"
    "&#129302; Abrir Configuracao da IA</button></form>";

    // ── Alterar Senha ──────────────────────────────────────
    c += "<h3>&#128274; Alterar Senha</h3>";
    c += "<form method='POST' action='/config'>";
    c += "<input type='hidden' name='acao' value='senha'>";
    c += "<label>Senha Atual:</label>";
    c += "<input type='password' name='pass_atual' "
    "required placeholder='Senha atual'>";
    c += "<label>Nova Senha (min 4 chars):</label>";
    c += "<input type='password' name='pass_nova' "
    "required placeholder='Nova senha'>";
    c += "<label>Confirmar Nova Senha:</label>";
    c += "<input type='password' name='pass_conf' "
    "required placeholder='Confirmar'>";
    c += "<button type='submit' class='btn-o'>"
    "&#128274; Alterar Senha</button></form>";

    // ── Limpar Histórico e Contadores ───────────────────────
    c += "<h3>&#128465; Limpar Historico e Contadores</h3>";
    c += "<div class='er' style='font-size:11px'>";
    c += "&#9888; <b>ATENCAO - ACAO PERMANENTE!</b><br>";
    c += "Este botao apaga <b>definitivamente</b>:<br>";
    c += "&bull; Todo o historico diario (todos os dias gravados)<br>";
    c += "&bull; Todos os contadores desde o inicio "
    "(Painel, Casa, Bateria Carga/Descarga em Ah)<br>";
    c += "&bull; O grafico de corrente em tempo real<br><br>";
    c += "O sistema vai <b>comecar um novo historico do zero</b> e os "
    "contadores serao <b>zerados permanentemente</b>. "
    "Esta acao <u>nao pode ser desfeita</u>.";
    c += "</div>";

    c += "<form method='POST' action='/config' "
    "onsubmit='return confirmarResetHistorico(this);'>";
    c += "<input type='hidden' name='acao' value='reset_historico'>";
    c += "<input type='hidden' name='confirmar' value='SIM'>";
    c += "<button type='submit' class='btn-r'>"
    "&#128465; Apagar Historico e Contadores Permanentemente</button></form>";

    c += "<script>"
    "function confirmarResetHistorico(f){"
    "return confirm("
    "'ATENCAO!\\n\\n"
    "Isso vai apagar PERMANENTEMENTE todo o historico e os "
    "contadores desde o inicio.\\n\\n"
    "Esta acao NAO PODE SER DESFEITA.\\n\\n"
    "Tem certeza que deseja continuar?"
    "');"
    "}"
    "</script>";

    // ── Ações do sistema ───────────────────────────────────
    c += "<h3>&#9881; Acoes do Sistema</h3>";

    c += "<form method='POST' action='/ntp-sync'>";
    c += "<button type='submit' class='btn-s btn-sm'"
    + String(WiFi.status()==WL_CONNECTED ? "" : " disabled") + ">"
    "&#128336; Sincronizar Hora (NTP)</button></form>";

    c += "<form method='GET' action='/setup-wifi'>";
    c += "<button type='submit' class='btn-s btn-sm' "
    "style='margin-top:6px'>"
    "&#128246; Reconfigurar WiFi</button></form>";

    c += "<form method='GET' action='/ota'>";
    c += "<button type='submit' class='btn-o btn-sm' "
    "style='margin-top:6px'>"
    "&#128640; Atualizar Firmware (OTA)</button></form>";

    c += "<form method='POST' action='/reiniciar'>";
    c += "<button type='submit' class='btn-r btn-sm' "
    "style='margin-top:6px'>"
    "&#8635; Reiniciar ESP32</button></form>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Config", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-IA-WEB]
// ═══════════════════════════════════════════════════════════

void handleIA() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            uint8_t modo    = server.arg("modo").toInt();
            float   reserva = server.arg("reserva").toFloat();
            if (reserva < 0 || reserva > 30)
                msg = "<div class='er'>&#10007; Reserva invalida (0-30%)</div>";
            else {
                ModoIAOperacao modoAntigo = ia.config.modo;
                ia.config.modo              = (ModoIAOperacao)modo;
                ia.config.reservaEmergencia = reserva;
                iaSalvarConfig();
                snprintf(tempBuf, sizeof(tempBuf),
                         "IA: modo=%s reserva=%.0f%%",
                         modo == IA_MODO_IA ? "IA_ATIVA" : "MANUAL",
                         reserva);
                adicionarLog(tempBuf);

                // Ao mudar de modo, zera timers de sequência para
                // não herdar estado de decisão do modo anterior.
                if (modoAntigo != ia.config.modo) {
                    ia.sequencia = SEQ_OCIOSO;
                    iaLimparRecomendacao();
                    memset(&ia.timersSeq, 0, sizeof(TimersSequenciaIA));
                }
                msg = "<div class='ok'>&#10003; Config IA salva!</div>";
            }
        }
        else if (acao == "reset_ia") {
            memset(ia.padroes, 0,
                   sizeof(PadraoConsumo) * IA_MAX_PADROES);
            ia.totalPadroes           = 0;
            ia.indicePadraoMaisAntigo = 0;
            ia.sequencia              = SEQ_OCIOSO;
            iaLimparRecomendacao();
            iaSalvarPadroes();
            adicionarLog("IA: padroes resetados");
            msg = "<div class='ok'>&#10003; Padroes resetados!</div>";
        }
    }

    String c = msg;

    // ── Explicação rápida ───────────────────────────────────
    c += "<div class='nf' style='font-size:10px'>";
    c += "&#128161; <b>A IA SEMPRE aprende</b>, independente do modo. "
    "Quanto mais tempo ligado, mais precisa fica.<br>";
    c += "<b>Manual:</b> corte por nivel de bateria (%) configurado em "
    "/redes. IA so recomenda, nao age.<br>";
    c += "<b>IA Ativa:</b> a IA assume o controle de Rede Alta, Rede "
    "Baixa e Lampadas com base no saldo real de energia (Ah) e nos "
    "padroes de consumo aprendidos - nao so no nivel de bateria.";
    c += "</div>";

    // ── Status atual ────────────────────────────────────────
    c += "<h3>&#129302; Status Atual</h3>";
    c += "<div class='sc'>";
    char bufIA[100];
    snprintf(bufIA, sizeof(bufIA),
             "Modo: <b>%s</b> | Padroes: %d/%d | "
             "Amostras: %d | Baseline: %.2fA (%.0f%%)",
             ia.config.modo == IA_MODO_IA ? "IA ATIVA" : "MANUAL",
             ia.totalPadroes, IA_MAX_PADROES,
             ia.snapshot.total,
             ia.analise.baselineAtual,
             ia.analise.baselineConfianca * 100.0f);
    c += "<div class='val' style='font-size:12px'>"
    + String(bufIA) + "</div>";

    if (bateria.calibrada) {
        char bufSaldo[80];
        snprintf(bufSaldo, sizeof(bufSaldo),
                 "Saldo: %+.1fAh | Nec.amanhecer: %.1fAh | "
                 "Sequencia: %d",
                 ia.saldoAhAtual,
                 ia.ahNecAmanhecerAtual,
                 (int)ia.sequencia);
        c += "<div class='sub'>" + String(bufSaldo) + "</div>";
    }
    if (ia.usandoFallbackManual) {
        c += "<div class='sub' style='color:#ef6c00'>"
        "&#9888; Usando fallback (poucos dados)</div>";
    }
    c += "</div>";

    // ── Card: Decisão IA atual (só quando IA ativa e tem motivo) ──
    if (ia.config.modo == IA_MODO_IA && strlen(ia.decisao.motivo) > 0) {
        String corDec = "#ef6c00";
        if (ia.decisao.desligarRedeAlta  ||
            ia.decisao.desligarRedeBaixa ||
            ia.decisao.desligarLampExt)
            corDec = "#c62828";
        else if (ia.decisao.ligarRedeAlta ||
            ia.decisao.ligarRedeBaixa)
            corDec = "#2e7d32";

        c += "<div style='background:#1a1a1a;border:1px solid " +
        corDec + ";border-radius:6px;padding:10px;margin-bottom:8px'>";
        c += "<div style='font-size:11px;font-weight:bold;color:" +
        corDec + "'>&#129302; Decisao IA atual:</div>";
        c += "<div style='font-size:11px;color:#e0e0e0;margin-top:4px'>";
        c += String(ia.decisao.motivo);
        c += "</div>";

        if (ia.decisao.ahEstimadoNecessario > 0 || ia.saldoAhAtual != 0) {
            c += "<div style='font-size:10px;color:#78909c;margin-top:6px;"
            "border-top:1px solid #333;padding-top:6px'>";
            char bufDec[160];
            snprintf(bufDec, sizeof(bufDec),
                     "Disponivel: <b style='color:#4fc3f7'>%.1fAh</b> | "
                     "Necessario: <b style='color:#ffb74d'>%.1fAh</b> | "
                     "Saldo: <b style='color:%s'>%+.1fAh</b>",
                     ia.decisao.ahDisponivel,
                     ia.decisao.ahEstimadoNecessario,
                     ia.saldoAhAtual >= 0 ? "#43a047" : "#ef5350",
                     ia.saldoAhAtual);
            c += String(bufDec) + "</div>";
        }
        c += "</div>";
    }

    // ── Recomendação ativa (mesma lógica do painel principal) ──
    if (ia.recomendacao.ativa) {
        String corBg, corBd, corTx, ico;
        switch (ia.recomendacao.nivel) {
            case 0: corBg="#0d1f2d"; corBd="#0277bd"; corTx="#90caf9"; ico="&#128161;"; break;
            case 1: corBg="#1f1500"; corBd="#ef6c00"; corTx="#ffe0b2"; ico="&#9888;";   break;
            default: corBg="#1f0a0a"; corBd="#c62828"; corTx="#ef9a9a"; ico="&#128680;"; break;
        }
        c += "<div style='background:" + corBg + ";border:2px solid " + corBd +
        ";border-radius:8px;padding:12px;margin-bottom:8px'>";
        c += "<div style='font-size:13px;font-weight:bold;color:" + corTx + "'>";
        c += ico + " " + String(ia.recomendacao.texto) + "</div>";
        if (strlen(ia.recomendacao.detalhe) > 0) {
            c += "<div style='font-size:11px;color:" + corTx +
            ";opacity:0.8;margin-top:6px'>" +
            String(ia.recomendacao.detalhe) + "</div>";
        }
        c += "</div>";
    }

    // ── Análise em tempo real ───────────────────────────────
    c += "<div class='nf' style='font-size:10px;margin-bottom:8px'>";
    c += "&#128270; <b>Analise em tempo real:</b> ";
    if (ia.analise.emPico) {
        float durPico = (millis() - ia.analise.tInicioPico) / 1000.0f;
        char bufA[80];
        snprintf(bufA, sizeof(bufA),
                 "&#9650; PICO detectado: %.2fA por %.0fs",
                 sensores.correnteCasa, durPico);
        c += "<span style='color:#ef6c00'>" + String(bufA) + "</span>";
        if (iaCorrenteETransitoria()) {
            char bufAh[40];
            snprintf(bufAh, sizeof(bufAh),
                     " | Transitorio (%.3fAh restante estimado)",
                     iaAhRestantePico());
            c += "<span style='color:#fdd835'>" + String(bufAh) + "</span>";
        }
    } else if (ia.analise.baselineConfianca > 0.2f) {
        char bufA[80];
        snprintf(bufA, sizeof(bufA),
                 "&#9646; Baseline: %.2fA (confianca %.0f%%)",
                 ia.analise.baselineAtual,
                 ia.analise.baselineConfianca * 100.0f);
        c += "<span style='color:#43a047'>" + String(bufA) + "</span>";
    } else {
        c += "<span style='color:#78909c'>Observando... "
        "(aguardando dados suficientes)</span>";
    }
    c += "</div>";

    // ── Configuração da IA ──────────────────────────────────
    c += "<h3>&#9881; Configuracao</h3>";
    c += "<form method='POST' action='/ia'>";
    c += "<input type='hidden' name='acao' value='config'>";

    c += "<label>Modo de Operacao:</label>";
    c += "<select name='modo'>";
    c += "<option value='0'" +
    String(ia.config.modo == IA_MODO_MANUAL ? " selected" : "") +
    ">Manual (corte por nivel de bateria)</option>";
    c += "<option value='1'" +
    String(ia.config.modo == IA_MODO_IA ? " selected" : "") +
    ">IA Ativa (decide por saldo de energia + padroes)</option>";
    c += "</select>";

    c += "<label>Reserva de Emergencia (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:3px 0'>"
    "% minimo que a IA preserva (0-30%). "
    "Abaixo disso = acao critica imediata (Rede Baixa + Inversor OFF).</div>";
    char bufRes[8];
    snprintf(bufRes, sizeof(bufRes), "%.0f",
             ia.config.reservaEmergencia);
    c += "<input type='number' name='reserva' "
    "min='0' max='30' step='1' required value='"
    + String(bufRes) + "'>";

    c += "<button type='submit' class='btn-g'>"
    "&#128190; Salvar Configuracao</button></form>";

    // ── Padrões aprendidos ──────────────────────────────────
    c += "<h3>&#129302; Padroes Aprendidos</h3>";

    if (ia.totalPadroes == 0) {
        c += "<div class='nf' style='text-align:center'>"
        "&#129302; Ainda aprendendo... Aguarde alguns ciclos de consumo.</div>";
    } else {
        const char* tiposNome[] = {
            "Baseline", "Transitorio", "Ciclico", "Alto Longo"
        };
        const char* tiposIco[] = {
            "&#9646;", "&#9650;", "&#8635;", "&#9650;&#9650;"
        };
        const char* tiposCor[] = {
            "#78909c", "#fdd835", "#4fc3f7", "#ef6c00"
        };
        const char* tiposDesc[] = {
            "Consumo continuo de base",
            "Pico curto (liga/desliga rapido)",
            "Ciclico (geladeira, AC...)",
            "Equipamento pesado por tempo longo"
        };

        int contTipo[4] = {0, 0, 0, 0};
        for (int i = 0; i < IA_MAX_PADROES; i++) {
            if (!ia.padroes[i].valido) continue;
            uint8_t t = (uint8_t)ia.padroes[i].tipo;
            if (t < 4) contTipo[t]++;
        }

        for (int t = 0; t < 4; t++) {
            if (contTipo[t] == 0) continue;

            c += "<div style='margin-bottom:10px'>";
            c += "<div style='font-size:11px;font-weight:bold;color:" +
            String(tiposCor[t]) + ";margin-bottom:4px'>";
            c += String(tiposIco[t]) + " " + String(tiposNome[t]);
            c += " <span style='font-weight:normal;color:#546e7a'>("
            + String(tiposDesc[t]) + ")</span></div>";

            for (int i = 0; i < IA_MAX_PADROES; i++) {
                if (!ia.padroes[i].valido) continue;
                if ((int)ia.padroes[i].tipo != t) continue;

                PadraoConsumo& p = ia.padroes[i];

                c += "<div style='background:#111;border:1px solid #2a2a2a;"
                "border-left:3px solid " + String(tiposCor[t]) +
                ";border-radius:4px;padding:8px;margin-bottom:4px'>";

                c += "<div style='display:flex;justify-content:space-between;"
                "align-items:center'>";

                char bufC[16];
                snprintf(bufC, sizeof(bufC), "%.2fA", p.correnteMedia);
                c += "<div>";
                c += "<span style='font-size:18px;font-weight:bold;color:" +
                String(tiposCor[t]) + "'>" + String(bufC) + "</span>";
                c += "<span style='font-size:10px;color:#546e7a'> media</span>";
                c += "</div>";

                c += "<div style='text-align:right'>";
                char bufOc[30];
                snprintf(bufOc, sizeof(bufOc),
                         "%lu ocorrencias", p.ocorrencias);
                c += "<div style='font-size:11px;color:#90a4ae'>"
                + String(bufOc) + "</div>";
                char bufH[20];
                snprintf(bufH, sizeof(bufH),
                         "Tipico: ~%02dh", p.horaInicio);
                c += "<div style='font-size:10px;color:#546e7a'>"
                + String(bufH) + "</div>";
                c += "</div></div>";

                c += "<div style='display:grid;grid-template-columns:"
                "1fr 1fr 1fr;gap:4px;margin-top:6px'>";

                char bufPico[16];
                snprintf(bufPico, sizeof(bufPico), "%.2fA", p.correntePico);
                c += "<div style='background:#0d0d0d;border-radius:4px;"
                "padding:5px;text-align:center'>";
                c += "<div style='font-size:9px;color:#546e7a'>PICO</div>";
                c += "<div style='font-size:13px;font-weight:bold;"
                "color:#ef5350'>" + String(bufPico) + "</div>";
                c += "</div>";

                char bufDur[20];
                float dur = p.duracaoMedia;
                if (dur < 60)
                    snprintf(bufDur, sizeof(bufDur), "%.0fs", dur);
                else if (dur < 3600)
                    snprintf(bufDur, sizeof(bufDur), "%.1fmin", dur/60.0f);
                else
                    snprintf(bufDur, sizeof(bufDur), "%.1fh", dur/3600.0f);

                c += "<div style='background:#0d0d0d;border-radius:4px;"
                "padding:5px;text-align:center'>";
                c += "<div style='font-size:9px;color:#546e7a'>DURACAO</div>";
                c += "<div style='font-size:13px;font-weight:bold;"
                "color:#4fc3f7'>" + String(bufDur) + "</div>";
                c += "</div>";

                char bufAh[16];
                snprintf(bufAh, sizeof(bufAh), "%.3fAh", p.energiaAh);
                c += "<div style='background:#0d0d0d;border-radius:4px;"
                "padding:5px;text-align:center'>";
                c += "<div style='font-size:9px;color:#546e7a'>ENERGIA</div>";
                c += "<div style='font-size:13px;font-weight:bold;"
                "color:#43a047'>" + String(bufAh) + "</div>";
                c += "</div>";

                c += "</div>";

                float maxPossivel = 30.0f;
                float pctBarra = (p.correnteMedia / maxPossivel) * 100.0f;
                if (pctBarra > 100) pctBarra = 100;
                c += "<div style='margin-top:6px'>";
                c += "<div style='display:flex;justify-content:space-between;"
                "font-size:9px;color:#546e7a;margin-bottom:2px'>";
                char bufMinMax[40];
                snprintf(bufMinMax, sizeof(bufMinMax),
                         "Min: %.2fA", p.correnteMin);
                c += "<span>" + String(bufMinMax) + "</span>";
                snprintf(bufMinMax, sizeof(bufMinMax),
                         "Max: %.2fA", p.correntePico);
                c += "<span>" + String(bufMinMax) + "</span>";
                c += "</div>";
                c += "<div class='barra-bg' style='height:8px'>";
                c += "<div class='barra' style='width:" +
                String((int)pctBarra) + "%;background:" +
                String(tiposCor[t]) + ";height:8px'></div>";
                c += "</div></div>";

                if (p.duracaoMin != p.duracaoMax) {
                    char bufDurMM[60];
                    float dMin = p.duracaoMin;
                    float dMax = p.duracaoMax;
                    char sDMin[12], sDMax[12];
                    if (dMin < 60) snprintf(sDMin,sizeof(sDMin),"%.0fs",dMin);
                    else snprintf(sDMin,sizeof(sDMin),"%.1fmin",dMin/60.0f);
                    if (dMax < 60) snprintf(sDMax,sizeof(sDMax),"%.0fs",dMax);
                    else snprintf(sDMax,sizeof(sDMax),"%.1fmin",dMax/60.0f);
                    snprintf(bufDurMM, sizeof(bufDurMM),
                             "Duracao: %s ~ %s", sDMin, sDMax);
                    c += "<div style='font-size:9px;color:#546e7a;"
                    "margin-top:4px'>" + String(bufDurMM) + "</div>";
                }

                c += "</div>";
            }
            c += "</div>";
        }

        c += "<div class='nf' style='font-size:10px'>";
        char bufResumo[80];
        snprintf(bufResumo, sizeof(bufResumo),
                 "Total: %d/%d padroes | "
                 "Baseline conf: %.0f%% | "
                 "Em pico: %s",
                 ia.totalPadroes, IA_MAX_PADROES,
                 ia.analise.baselineConfianca * 100.0f,
                 ia.analise.emPico ? "SIM" : "NAO");
        c += String(bufResumo) + "</div>";

        c += "<form method='POST' action='/ia'>";
        c += "<input type='hidden' name='acao' value='reset_ia'>";
        c += "<button type='submit' class='btn-r btn-sm' "
        "style='margin-top:8px'>"
        "&#128465; Limpar Padroes Aprendidos</button></form>";
    }

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - IA", c));
}

// ═══════════════════════════════════════════════════════════
// [SIS-AUX-WEB] HANDLERS AUXILIARES
// ═══════════════════════════════════════════════════════════

void handleResetCurto() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }
    String rede = server.arg("rede");
    if (rede == "alta")     resetCurtoAlta();
    else if (rede == "baixa"   ||
        rede == "inversor") resetCurtoBaixa();

    server.sendHeader("Location", "/redes");
    server.send(302);
}

void handleNTPSync() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }
    sincronizarNTP();
    server.sendHeader("Location", "/config");
    server.send(302);
}

void handleReiniciar() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }
    adicionarLog("Reinicio solicitado pelo usuario");
    String c = "<div class='ok'>&#10003; Reiniciando em 3s...</div>";
    c += "<div class='nf'>Aguarde e reconecte.</div>";
    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Reiniciando", c, false));
    delay(3000);
    ESP.restart();
}

void handleOTAResult() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302); return;
    }
    String c = "<h3>&#128640; Atualizacao OTA</h3>";
    c += "<div class='nf'>"
    "Selecione o arquivo .bin gerado pelo Arduino IDE.<br>"
    "O ESP32 sera reiniciado automaticamente apos a atualizacao.</div>";
    c += "<div class='wn' style='font-size:11px'>"
    "&#9888; Nao feche a pagina durante o upload!</div>";
    c += "<form method='POST' action='/ota-upload' "
    "enctype='multipart/form-data'>";
    c += "<label>Arquivo de Firmware (.bin):</label>";
    c += "<input type='file' name='firmware' accept='.bin' required "
    "style='background:#111;border:1px solid #444;"
    "border-radius:4px;padding:8px;color:#e0e0e0;width:100%;"
    "box-sizing:border-box;margin-top:4px'>";
    c += "<button type='submit' class='btn-o' style='margin-top:10px'>"
    "&#128640; Enviar e Atualizar</button></form>";
    c += "<form method='GET' action='/config'>";
    c += "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>"
    "&#8592; Voltar</button></form>";
    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - OTA", c));
}

void handleOTAUpload() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        adicionarLog("OTA: upload iniciado");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            adicionarLog("OTA ERRO: begin() falhou");
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize)
            != upload.currentSize) {
            adicionarLog("OTA ERRO: write() falhou");
            }
    }
    else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            snprintf(tempBuf, sizeof(tempBuf),
                     "OTA OK: %u bytes", upload.totalSize);
            adicionarLog(tempBuf);
            String c = "<div class='ok'>&#10003; Firmware atualizado!</div>";
            c += "<div class='nf'>Reiniciando em 3s...</div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - OTA", c, false));
            delay(3000);
            ESP.restart();
        } else {
            adicionarLog("OTA ERRO: end() falhou");
            String c = "<div class='er'>&#10007; Erro no OTA. Tente novamente.</div>";
            c += "<form method='GET' action='/ota'>"
            "<button type='submit' class='btn-s'>Voltar</button></form>";
            server.send(500, "text/html; charset=utf-8",
                        gerarPagina("RRFS - OTA Erro", c, false));
        }
    }
}

/* ============================================================================
 * ============================================================================
 *                         DOCUMENTAÇÃO DO SISTEMA
 *                    RRFS ENERGIA - GERENCIAMENTO SOLAR
 *                              VERSÃO: V15.0
 * ============================================================================
 * ============================================================================
 *
 * Este documento lista todas as "IAs" (inteligências / subsistemas autônomos
 * de decisão) presentes no firmware. Cada IA atua de forma independente,
 * monitorando dados e tomando decisões automáticas sem intervenção do
 * usuário, dentro das regras e limites configurados na interface web.
 *
 * Para cada IA é indicado:
 *   - O que ela faz
 *   - Quando/com que frequência ela roda
 *   - A primeira linha da função principal (para localização rápida
 *     com Ctrl+F no código)
 *
 * ============================================================================
 * MUDANÇAS PRINCIPAIS DA V14 PARA A V15
 * ============================================================================
 *
 *   [REMOVIDO] Estimativa de tensão da bateria (IA 03 e IA 16 da V14 não
 *              existem mais). O sistema não tenta mais "adivinhar" a tensão
 *              via tabela SOC→Tensão. Motivo: complexidade alta e baixa
 *              precisão, já que a tensão real depende de fatores que o
 *              sistema não consegue medir sem sensor físico.
 *
 *   [ALTERADO] Toda a lógica de energia migrou de Wh (watt-hora, estimado)
 *              para Ah (ampere-hora, medido DIRETAMENTE pelos sensores
 *              ACS758 via coulomb counting). Isso elimina a dependência da
 *              tensão estimada e torna os cálculos mais precisos, pois
 *              corrente é o que os sensores realmente medem.
 *
 *   [ALTERADO] Configuração do painel: em vez de "kWp instalado" (que
 *              exigia saber a tensão real do arranjo), agora se configura
 *              a "Amperagem máxima do controlador de carga" - um dado que
 *              o usuário sempre tem (está na etiqueta do controlador).
 *
 *   [MANTIDO]  kW/W ainda são exibidos na interface, mas agora são
 *              PURAMENTE INFORMATIVOS: corrente medida × tensão NOMINAL
 *              configurada (não medida, não estimada). Servem só como
 *              referência visual, nunca são usados internamente por
 *              nenhuma IA para tomar decisões.
 *
 *   [NOVO]     Sistema de recomendações da IA 16 (antiga IA 18): agora a
 *              IA mostra mensagens explicativas no topo da página principal
 *              antes/durante de agir ("Economia necessária: X Ah"), mesmo
 *              no Modo Manual, para o usuário entender o raciocínio.
 *
 *   [NOVO]     Timers anti-oscilação por carga individual na IA 16
 *              (iaPodeDesligar/iaPodeLigar), impedindo que a mesma carga
 *              seja desligada e religada repetidamente em curto espaço
 *              de tempo, mesmo que o saldo de energia oscile perto de zero.
 *
 *   [NOVO]     Modo de fallback automático: se a IA 16 estiver em Modo IA
 *              mas ainda não tiver dados suficientes (< 30 amostras) E a
 *              bateria já estiver baixa, o sistema usa os limites manuais
 *              de corte (página Redes) como rede de segurança, em vez de
 *              arriscar uma decisão com poucos dados.
 *
 *   [ALTERADO] Configuração de Modo (Manual/IA Ativa) e Reserva de
 *              Emergência migrou da página /redes para a página /config.
 *
 *   [OTIMIZADO] Remoção de ~200 linhas de código relacionadas à estimativa
 *              de tensão, reduzindo uso de RAM e simplificando manutenção.
 *
 * ============================================================================
 * ÍNDICE DAS IAs (V15)
 * ============================================================================
 *
 *  IA 01 - Proteção Automática Contra Curto-Circuito (Diagnóstico)
 *  IA 02 - Calibração Automática do Offset do Sensor de Corrente do Painel
 *  IA 03 - Calibração Automática de 100% da Bateria (Detecção de Carga Cheia)
 *  IA 04 - Calibração Automática da Previsão de Geração Solar
 *  IA 05 - Motor de Previsão de Geração Solar (Cálculo Hora a Hora, em Ah)
 *  IA 06 - Motor de Decisão Automática de Energia (Noturno + Diurno)
 *  IA 07 - Antecipação Inteligente de Religamento da Rede Alta
 *  IA 08 - Controle Automático de Cargas por Nível de Bateria
 *  IA 09 - Controle Solar Automático da Lâmpada Externa (Nascer/Pôr do Sol)
 *  IA 10 - Máquina de Restauração Não-Bloqueante de Energia
 *  IA 11 - Religamento Automático Matinal da Rede Alta
 *  IA 12 - Reconexão WiFi Inteligente com Fallback para Modo AP
 *  IA 13 - Cache Inteligente de Consumo Histórico por Hora
 *  IA 14 - Estimador de Tempo Restante de Bateria (em Ah/h)
 *  IA 15 - Aprendizado de Padrões de Consumo da Casa (Motor de Padrões)
 *  IA 16 - Motor de Decisão da IA de Padrões (Modo IA / Modo Manual)
 *
 *  [REMOVIDAS EM V15]:
 *    - Calibração Automática da Tensão Estimada da Bateria (antiga IA 03)
 *    - Estimador de Tensão da Bateria (antiga IA 16)
 *
 * ============================================================================
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 01 - PROTEÇÃO AUTOMÁTICA CONTRA CURTO-CIRCUITO (DIAGNÓSTICO)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Monitora continuamente o sinal de status do inversor. Se o sinal cai
 *   inesperadamente, o sistema desliga o inversor, espera, e tenta religar
 *   até 3 vezes. Se falhar nas 3 tentativas, entra em modo de diagnóstico
 *   automático: isola a Rede Alta e testa o inversor sozinho; se o problema
 *   persistir, isola também a Rede Baixa e testa de novo. Assim o sistema
 *   identifica sozinho SE o curto está na rede alta, na rede baixa, ou se
 *   é uma falha do próprio inversor - sem o usuário precisar testar rede
 *   por rede manualmente.
 * QUANDO RODA:
 *   Todo ciclo do loop() enquanto o inversor está ligado ou durante o
 *   processo de diagnóstico (não bloqueia o resto do sistema).
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void monitorarInversor() {
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 02 - CALIBRAÇÃO AUTOMÁTICA DO OFFSET DO SENSOR DE CORRENTE DO PAINEL
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   O sensor ACS758 do painel tem um offset de fábrica (2500mV) que pode
 *   variar de sensor para sensor. A IA aprende o offset REAL em duas
 *   janelas de oportunidade:
 *
 *   NOTURNA (22h-4h): quando não há geração solar alguma, aprende o offset
 *   real medindo o sinal por 30s com alta confiança.
 *
 *   DIURNA (complementar): quando o sinal está acima do offset (painel
 *   claramente parado) e estável por pelo menos 60s (variação ≤5mV entre
 *   amostras consecutivas), complementa o aprendizado iniciado de noite.
 *
 *   O offset aprendido é usado em vez do valor de datasheet, aumentando
 *   a precisão da leitura de corrente do painel ao longo do tempo, sem
 *   calibração manual.
 *
 *   [MANTIDO DA V14] ZONA MORTA DINÂMICA: o limiar de "consumo próprio do
 *   sistema" (ESP32 + controlador + sensores) é ajustado automaticamente:
 *   - Sem calibração:      0.50A (conservador, evita falso consumo)
 *   - Com calib. noturna:  0.25A (offset confiável, limiar menor)
 *   - Com calib. diurna:   0.30A (ligeiramente mais conservador)
 *   Isso evita que o erro de offset não calibrado seja contabilizado
 *   como "consumo real do sistema".
 *
 *   [NOVO EM V15] Essa mesma leitura agora calcula diretamente a corrente
 *   de "Sistema" (ESP32 + controlador) como um subproduto do cálculo de
 *   offset, sem precisar de sensor dedicado, e alimenta o card "Sistema"
 *   exibido na página principal e na página de Sensores.
 *
 *   [SUPORTE À CALIBRAÇÃO MANUAL] O offset aprendido automaticamente pode
 *   ser complementado/sobrescrito por calibração manual feita na página
 *   /sensores (1 ponto ou 2 pontos), salva na flash (namespace "rrfs_cal")
 *   e recarregada no boot.
 * QUANDO RODA:
 *   Dentro da leitura de corrente do painel, a cada 1 segundo.
 *   Aprende de fato na janela noturna (22h-4h) e na janela diurna
 *   quando o sinal está estável.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   float lerCorrentePainel() {
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 03 - CALIBRAÇÃO AUTOMÁTICA DE 100% DA BATERIA (DETECÇÃO DE CARGA CHEIA)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Detecta automaticamente quando a bateria atinge 100% de carga: se o
 *   painel está gerando, a casa está consumindo, mas a corrente líquida
 *   da bateria está próxima de zero, significa que o BMS cortou a carga
 *   porque a bateria está cheia. Nesse momento o sistema marca a bateria
 *   como calibrada em 100%, permitindo que o coulomb counting (contagem
 *   de energia em Ah) comece com uma referência confiável, sem o usuário
 *   precisar informar manualmente o % inicial.
 * QUANDO RODA:
 *   A cada 1 segundo, dentro da leitura de sensores (só age 1x, até a
 *   bateria ser calibrada).
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void verificarCalibracaoAutomatica() {
 *
 *  [Numeração V14: esta era a IA 04]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 04 - CALIBRAÇÃO AUTOMÁTICA DA PREVISÃO DE GERAÇÃO SOLAR
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Compara a geração REAL medida pelo sensor do painel (em Amperes) com
 *   a geração que a IA 05 (previsão) tinha estimado para aquela mesma
 *   hora, e ajusta um fator de calibração (0.1x a 3.0x) por média móvel.
 *   Assim, com o passar dos dias, a previsão de geração fica cada vez
 *   mais precisa PARA O PAINEL ESPECÍFICO daquela instalação (considerando
 *   sombreamento, sujeira, orientação real dos painéis etc), em vez de
 *   usar só o cálculo teórico genérico.
 *
 *   [ALTERADO EM V15] A comparação agora é feita diretamente em Amperes
 *   (corrente real vs corrente estimada), sem conversão via tensão -
 *   eliminando uma fonte de erro que existia na V14 (Wh dependia de
 *   tensão estimada).
 * QUANDO RODA:
 *   A cada 5 minutos (dentro do ciclo de decisões), mas só realmente
 *   calibra quando o painel está gerando (>1A) e há dado de previsão
 *   válido para a hora atual.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void calibrarPrevisao() {
 *
 *  [Numeração V14: esta era a IA 05]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 05 - MOTOR DE PREVISÃO DE GERAÇÃO SOLAR (CÁLCULO HORA A HORA, EM Ah)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Busca dados meteorológicos (radiação solar direta e difusa, cobertura
 *   de nuvens, temperatura) da API gratuita Open-Meteo para as próximas
 *   48 horas, e converte esses dados em uma estimativa de GERAÇÃO EM
 *   AMPERES do painel para cada uma dessas horas (cada hora = 1A médio
 *   equivale a 1Ah gerado naquela hora), considerando:
 *
 *   [ALTERADO EM V15] A amperagem MÁXIMA do controlador de carga
 *   configurada (em vez de kWp instalado), eficiência do sistema, perda
 *   por temperatura acima de 25°C, e o fator de calibração aprendido
 *   pela IA 04.
 *
 *   É a base de dados que TODAS as outras IAs de decisão (06 e 07)
 *   usam para "ver o futuro" em termos de energia disponível (Ah).
 * QUANDO RODA:
 *   A cada 1 hora (ou quando forçado manualmente na página de Previsão).
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void buscarPrevisaoMeteo() {
 *   (cálculo por hora em: float calcularCorrenteEstimadaHora(...) )
 *
 *  [Numeração V14: esta era a IA 06. Função renomeada de
 *   calcularGeracaoHora() para calcularCorrenteEstimadaHora(), e o
 *   retorno mudou de Wh estimado para A (corrente) estimada]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 06 - MOTOR DE DECISÃO AUTOMÁTICA DE ENERGIA (NOTURNO + DIURNO)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   É o "cérebro" central de decisões de longo prazo. Tem duas frentes:
 *
 *   NOTURNA: calcula quantos Ah serão necessários até o amanhecer
 *   (baseado no consumo médio histórico em Ah/hora) e, se a bateria não
 *   tiver saldo suficiente, desliga preventivamente a Rede Alta e/ou a
 *   Lâmpada Externa ainda à noite, evitando chegar no corte crítico de
 *   energia.
 *
 *   DIURNA: soma a geração PREVISTA restante do dia em Ah (IA 05) + a
 *   energia atual da bateria (Ah), e compara com o consumo estimado das
 *   horas restantes do dia (usando a média histórica específica de cada
 *   hora em Ah, via IA 13). Se detectar déficit projetado, desliga a
 *   Rede Alta e, em situação já crítica, a Lâmpada Interna também - mesmo
 *   de dia, não só à noite.
 *
 *   [ALTERADO EM V15] Todos os cálculos internos (saldo, consumo médio,
 *   geração restante) agora são feitos inteiramente em Ah, eliminando
 *   a necessidade de conversão via tensão estimada da bateria.
 * QUANDO RODA:
 *   A cada 5 minutos.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void tomarDecisoes() {
 *
 *  [Numeração V14: esta era a IA 07]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 07 - ANTECIPAÇÃO INTELIGENTE DE RELIGAMENTO DA REDE ALTA
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Normalmente a Rede Alta só liga quando a bateria ultrapassa o %
 *   configurado. Esta IA antecipa esse religamento: se a geração solar
 *   está ativa e em tendência de alta (hora seguinte prevista maior que a
 *   atual, em Ah), ela liga a Rede Alta em modo TESTE mesmo com a bateria
 *   abaixo do limite, aguarda 60s para a corrente estabilizar, mede o
 *   consumo REAL, calcula quantas horas faltam para a bateria chegar no
 *   corte-baixo com esse consumo (usando Ah disponível ÷ corrente de
 *   descarga), e compara com quanto tempo a previsão diz que falta para
 *   a geração melhorar. Se for favorável, mantém ligada e reavalia a
 *   cada 5 minutos; se não for, desliga de novo e espera 15 min (cooldown)
 *   antes de tentar de novo. Também trata picos de consumo temporários
 *   (torradeira, forno) usando o histórico da hora como referência em vez
 *   do pico instantâneo (IA 13).
 * QUANDO RODA:
 *   A cada 10 segundos.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void avaliarAntecipacaoRedeAlta() {
 *
 *  [Numeração V14: esta era a IA 08]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 08 - CONTROLE AUTOMÁTICO DE CARGAS POR NÍVEL DE BATERIA
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Aplica em tempo real os limites de % configurados pelo usuário na
 *   página de Redes: desliga Lâmpada Interna, Lâmpada Externa, Rede Alta,
 *   e por fim Rede Baixa + Inversor, conforme a bateria vai descarregando
 *   (% calculado a partir de Ah atual ÷ Ah disponível), sempre com buffer
 *   de 60s antes de desligar (evita flapping por picos momentâneos) e
 *   margem de +2% para religar (evita liga/desliga repetido perto do
 *   limiar). É a camada de segurança que garante que a bateria nunca
 *   passe do limite crítico configurado.
 *
 *   SEQUÊNCIA DE DESLIGAMENTO (ordem de prioridade):
 *     1º - Lâmpada Externa  (menor impacto no conforto)
 *     2º - Rede Alta        (equipamentos de alto consumo)
 *     3º - Rede Baixa       (geladeira, TV, roteador)
 *     4º - Lâmpada Interna  (reserva de emergência configurável)
 *
 *   SEQUÊNCIA DE RELIGAMENTO (amanhecer / bateria recuperada):
 *     1º - Rede Baixa
 *     2º - Rede Alta
 *     3º - Lâmpada Externa (somente à noite)
 *     4º - Lâmpada Interna (se necessário)
 * QUANDO RODA:
 *   A cada 5 segundos.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void controlarEnergia() {
 *
 *  [Numeração V14: esta era a IA 09]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 09 - CONTROLE SOLAR AUTOMÁTICO DA LÂMPADA EXTERNA (NASCER/PÔR DO SOL)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Calcula matematicamente o horário de nascer e pôr do sol do dia (a
 *   partir da latitude configurada e da declinação solar do mês), e liga/
 *   desliga a Lâmpada Externa automaticamente nesses horários - sem
 *   depender de sensor de luminosidade (LDR). Tem buffer de 60s de
 *   confirmação para evitar oscilar perto da transição dia/noite. Se a
 *   localização não estiver configurada, cai num modo alternativo baseado
 *   na corrente do painel (dia = painel gerando >1A).
 * QUANDO RODA:
 *   Dentro de controlarEnergia() (IA 08), a cada 5 segundos.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL (trecho dentro de controlarEnergia):
 *   void controlarEnergia() {
 *   (bloco: "LÂMPADA EXTERNA - CONTROLE POR HORÁRIO SOLAR")
 *
 *  [Numeração V14: esta era a IA 10]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 10 - MÁQUINA DE RESTAURAÇÃO NÃO-BLOQUEANTE DE ENERGIA
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Quando a energia volta a ficar disponível após um corte, existe uma
 *   sequência correta de religamento (Rede Baixa → aguarda roteador →
 *   Inversor → aguarda boot → checa status → Rede Alta → Lâmpada Interna),
 *   com tempos de espera entre cada passo. Esta IA executa essa sequência
 *   inteira em segundo plano, SEM travar o loop principal - a interface
 *   web e o WiFi continuam respondendo normalmente durante toda a
 *   restauração.
 * QUANDO RODA:
 *   Todo ciclo do loop(), mas só executa passos quando está ativa (disparada
 *   pela IA 08 quando detecta energia recuperada).
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void processarRestauroEnergia() {
 *
 *  [Numeração V14: esta era a IA 11]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 11 - RELIGAMENTO AUTOMÁTICO MATINAL DA REDE ALTA
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Quando a IA 06 desligou a Rede Alta à noite por economia, esta IA
 *   monitora a manhã seguinte e religa a Rede Alta automaticamente assim
 *   que: (a) o painel volta a gerar E a bateria está com boa margem acima
 *   do corte, OU (b) chegou no horário previsto de início de geração
 *   (calculado pela IA 06) com a bateria em nível seguro. Fecha o ciclo
 *   iniciado pela decisão noturna.
 * QUANDO RODA:
 *   A cada 5 minutos.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void verificarReligamentoAuto() {
 *
 *  [Numeração V14: esta era a IA 12]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 12 - RECONEXÃO WIFI INTELIGENTE COM FALLBACK PARA MODO AP
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Monitora a conexão WiFi e, se cair, tenta reconectar automaticamente
 *   usando as credenciais salvas. Após 5 tentativas falhas, ativa um Modo
 *   Ponto de Acesso (AP) TEMPORÁRIO em paralelo (modo AP+STA), permitindo
 *   acessar o sistema localmente mesmo sem WiFi doméstico, enquanto
 *   continua tentando reconectar em segundo plano. Assim que a rede volta,
 *   desativa o AP temporário sozinho e volta ao modo cliente normal.
 * QUANDO RODA:
 *   Checagem a cada 15 segundos; tentativa de reconexão a cada 30 segundos.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void tentarReconectarWiFi() {
 *
 *  [Numeração V14: esta era a IA 13]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 13 - CACHE INTELIGENTE DE CONSUMO HISTÓRICO POR HORA
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Calcula a média histórica de consumo específica de cada hora do dia
 *   (ex: "às 8h a casa consome em média X Ah", olhando até 14 dias de
 *   histórico salvo na flash). É usada pelas IAs 06 e 07 para prever o
 *   consumo esperado e distinguir picos temporários de consumo normal.
 *   Como cada leitura de histórico envolve acesso à memória flash (NVS),
 *   esta IA mantém um CACHE em RAM dos 24 valores (um por hora), só
 *   recalculando quando a hora do relógio muda - evitando leituras de
 *   flash repetidas e desnecessárias a cada poucos segundos.
 * QUANDO RODA:
 *   Sob demanda, chamada pelas IAs 06 e 07 (a cada 5 min e a cada 10s
 *   respectivamente), mas só efetivamente recalcula 1x por hora.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   float consumoMedioParaHoraCache(uint8_t horaAlvo, bool* temDados) {
 *
 *  [Numeração V14: esta era a IA 14]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 14 - ESTIMADOR DE TEMPO RESTANTE DE BATERIA (EM Ah/h)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Calcula, em tempo real, quanto tempo falta para a bateria terminar de
 *   carregar (se estiver carregando) ou quanto tempo falta até descarregar
 *   completamente (se estiver descarregando), com base na CORRENTE atual
 *   medida (Ah disponível ÷ corrente de carga/descarga em A). É o valor
 *   mostrado no painel principal ("Restam 3h20m" etc), agora sem depender
 *   de nenhuma estimativa de tensão.
 *
 *   [NOVO EM V15] Também calcula o horário estimado de corte para cada
 *   carga individual (tempoAteCorte), mostrando "Desliga em: Xh (~HH:MM)"
 *   para Rede Alta, Rede Baixa, Lâmpada Externa e Lâmpada Interna,
 *   baseado no % de corte configurado de cada uma.
 * QUANDO RODA:
 *   Sob demanda, chamada sempre que a página principal é gerada/atualizada.
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   EstimativaTempo calcularTempoRestante() {
 *   (horário por carga em: String tempoAteCorte(float pctCorte, float taxaA) )
 *
 *  [Numeração V14: esta era a IA 15]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 15 - APRENDIZADO DE PADRÕES DE CONSUMO DA CASA (MOTOR DE PADRÕES)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Monitora continuamente a corrente da casa e aprende, de forma
 *   totalmente automática, os diferentes padrões de consumo presentes
 *   na instalação, sem necessidade de configuração manual pelo usuário.
 *
 *   TIPOS DE PADRÃO APRENDIDOS:
 *     BASELINE: corrente estável por 2 a 5 minutos = consumo base da
 *       casa (geladeira em standby + roteador + TV + notebook, etc).
 *       Tolerância de ±1A para flutuações normais.
 *
 *     TRANSITÓRIO: corrente alta por menos de 10 minutos = equipamento
 *       de uso rápido (chaleira elétrica, lancheira, torradeira).
 *       A IA aprende a corrente típica, duração mínima/máxima e
 *       energia consumida (em Ah) de cada ocorrência.
 *
 *     ALTO LONGO: corrente alta por 10 min a 1 hora = uso prolongado
 *       (microondas longo, forno elétrico, máquina de lavar em ciclo).
 *
 *     CÍCLICO: corrente que sobe e desce periodicamente = compressor
 *       da geladeira, ar condicionado com termostato, etc.
 *
 *   MECANISMO DE APRENDIZADO:
 *     - Buffer circular de 120 amostras (2 minutos a 1s/amostra)
 *     - Análise de médias em janelas de 2, 3, 4, 5 e 10 minutos
 *     - Cálculo de desvio padrão para detectar estabilidade
 *     - Padrões similares são CONSOLIDADOS (média móvel ponderada)
 *       em vez de duplicados, mantendo o banco de padrões enxuto
 *     - Até 50 padrões salvos na flash NVS com substituição circular
 *       dos mais antigos quando a memória está cheia
 *     - Salva na flash automaticamente a cada 60s quando há novidades,
 *       e força salvamento completo a cada 10 minutos
 *
 *   BASELINE DE CONFIANÇA:
 *     A IA mantém um índice de confiança (0 a 100%) no baseline
 *     detectado. Quanto maior a confiança, menor a zona morta usada
 *     para detectar picos, e mais precisa fica a decisão da IA 16.
 *     A confiança cresce com confirmações consecutivas de estabilidade
 *     e cai quando o padrão muda muito.
 *
 *   [ALTERADO EM V15] Toda energia registrada por padrão agora é em Ah
 *   (energiaAh), calculada diretamente de corrente × tempo, sem
 *   conversão via tensão.
 *
 *   ESTA IA FUNCIONA EM BACKGROUND SEMPRE, mesmo no Modo Manual -
 *   acumulando conhecimento sobre a casa que será usado pela IA 16
 *   quando/se o usuário ativar o Modo IA.
 * QUANDO RODA:
 *   A cada 1 segundo, chamada dentro de lerSensores().
 *   Só registra padrões novos/atualizados quando há dados suficientes
 *   (mínimo 2 minutos de histórico no buffer).
 * PRIMEIRA LINHA DA FUNÇÃO PRINCIPAL:
 *   void iaAnalisar() {
 *
 *  [Numeração V14: esta era a IA 17]
 *
 *
 * ────────────────────────────────────────────────────────────────────────
 * IA 16 - MOTOR DE DECISÃO DA IA DE PADRÕES (MODO IA / MODO MANUAL)
 * ────────────────────────────────────────────────────────────────────────
 * O QUE FAZ:
 *   Usa os padrões aprendidos pela IA 15 para tomar decisões inteligentes
 *   de desligamento e religamento das cargas, com foco em NUNCA deixar
 *   a bateria esgotar e NUNCA oscilar (liga/desliga repetido).
 *
 *   MODO MANUAL: a IA 16 não executa ações nos relés, mas continua
 *   avaliando e registrando decisões que TOMARIA nos logs. O usuário
 *   mantém controle total. A IA 15 segue aprendendo em background.
 *
 *   MODO IA: a IA 16 toma decisões automáticas baseadas em:
 *
 *   1. SALDO DE ENERGIA ATÉ O AMANHECER (EM Ah):
 *      Calcula energia disponível (Ah atual - reserva de emergência)
 *      menos energia necessária até o amanhecer (consumo estimado ×
 *      horas restantes). Se o saldo for negativo, inicia a sequência
 *      de desligamento preventivo.
 *
 *   [NOVO EM V15] MODO FALLBACK DE SEGURANÇA: se houver poucos dados
 *      de consumo aprendidos (< 30 amostras) E a bateria já estiver
 *      próxima do corte, a IA usa os limites manuais configurados na
 *      página de Redes como rede de segurança, evitando decisão
 *      arriscada com base em dado insuficiente.
 *
 *   2. SEQUÊNCIA DE DESLIGAMENTO (ordem, do menos ao mais impactante):
 *      1º - Recomendação de desligar lâmpadas internas não essenciais
 *           (aviso, sem ação automática)
 *      2º - Lâmpada Externa
 *      3º - Rede Alta
 *      4º - Rede Baixa + Inversor (somente em situação CRÍTICA real:
 *           bateria perto da reserva de emergência)
 *
 *   3. INTELIGÊNCIA ANTI-DESLIGAMENTO PRECIPITADO:
 *      Antes de desligar a Rede Alta, verifica se a corrente atual é
 *      um PICO TRANSITÓRIO CONHECIDO (aprendido pela IA 15, ex:
 *      chaleira que dura 3 min). Calcula a energia que esse pico ainda
 *      vai consumir (iaAhRestantePico) e considera no saldo antes de
 *      decidir.
 *
 *   4. SEQUÊNCIA DE RELIGAMENTO (saldo positivo, > histerese de 5Ah):
 *      Religa as cargas na ordem: Inversor → Rede Baixa → Rede Alta →
 *      Lâmpada Externa (se noite), sempre respeitando os intervalos
 *      mínimos entre tentativas (IA_INTERVALO_RELIGAR = 2min).
 *
 *   5. ANTI-OSCILAÇÃO ABSOLUTA:
 *      [NOVO EM V15] Implementado via funções dedicadas
 *      iaPodeDesligar()/iaPodeLigar() que verificam, POR CARGA
 *      INDIVIDUAL, os timers desde a última ação:
 *        Após desligar uma carga: mínimo 10 minutos antes de religar.
 *        Após ligar uma carga:    mínimo 5 minutos antes de desligar.
 *      Isso vale mesmo que o usuário interfira manualmente.
 *
 *   6. RESERVA DE EMERGÊNCIA CONFIGURÁVEL:
 *      O usuário define (agora na página /config) qual % da bateria é
 *      "intocável" - reservado exclusivamente para manter a Lâmpada
 *      Interna acesa (iluminação de emergência). Padrão: 5%.
 *
 *   [NOVO EM V15] SISTEMA DE RECOMENDAÇÕES VISUAIS:
 *      A IA agora exibe mensagens no topo da página principal
 *      (iaSetRecomendacao/iaLimparRecomendacao) explicando o que está
 *      fazendo e por quê ("Economia necessária: X Ah", "Rede Alta
 *      desligada - saldo insuficiente", etc.), com níveis de urgência
 *      (0=informativo, 1=atenção, 2=crítico) representados por cores
 *      diferentes na interface. Funciona tanto no Modo IA quanto no
 *      Modo Manual (nesse caso só recomenda, não age).
 *
 *   INTERAÇÃO COM OUTRAS IAs:
 *     - Recebe dados da IA 15 (padrões e baseline)
 *     - Complementa (não substitui) as decisões da IA 06 e IA 08
 *     - Respeita bloqueios da IA 01 (curto-circuito tem prioridade)
 *     - Opera em paralelo com a IA 07 (antecipação rede alta)
 * QUANDO RODA:
 *   Avaliação a cada 30 segundos (iaTomarDecisao + iaExecutarDecisoes).
 *   Salvamento de padrões forçado a cada 10 minutos.
 * PRIMEIRA LINHA DAS FUNÇÕES PRINCIPAIS:
 *   float iaCalcularSaldoAh() {  (cálculo central de saldo em Ah)
 *   void iaTomarDecisao() {      (motor de decisão)
 *   void iaExecutarDecisoes() {  (aplica nos relés)
 *   void iaLoop() {              (orquestra tudo no loop principal)
 *
 *  [Numeração V14: esta era a IA 18]
 *
 *
 * ============================================================================
 * RESUMO - FLUXO DE INTERAÇÃO ENTRE AS IAs (V15)
 * ============================================================================
 *
 *   IA 05 (Previsão, em Ah) ──alimenta──> IA 06 (Decisão) e IA 07 (Antecip.)
 *   IA 13 (Cache consumo) ──alimenta──> IA 06 e IA 07
 *   IA 04 (Calib. previsão) ──refina──> IA 05
 *   IA 15 (Padrões de consumo) ──alimenta──> IA 16 (Decisão de padrões)
 *   IA 02, 03 (Calibrações de sensor/bateria) ──alimentam──> todas as
 *       demais, pois toda decisão depende de leituras precisas
 *   IA 06 e IA 07 ──comandam──> IA 08 (que aplica as regras de corte/religa)
 *   IA 16 ──complementa──> IA 08 (camada adicional de inteligência)
 *   IA 08 ──dispara──> IA 10 (quando energia volta) e IA 01 (proteção segue
 *       ativa em paralelo, tem prioridade sobre todas as outras em caso
 *       de curto real)
 *   IA 11 ──fecha o ciclo──> aberto pela decisão noturna da IA 06
 *   IA 01 (Curto-circuito) ──tem PRIORIDADE MÁXIMA──> bloqueia IA 16,
 *       IA 08, IA 07 e IA 06 durante diagnóstico ativo
 *
 *
 * ============================================================================
 * RESUMO - MEMÓRIA E PERFORMANCE (V15)
 * ============================================================================
 *
 *   FLASH NVS (armazenamento persistente):
 *     rrfs         - Config geral, WiFi, bateria (Ah), redes, credenciais
 *     rrfs_cal     - [NOVO EM V15] Calibração manual dos sensores de
 *                    corrente (painel e casa), 1 ou 2 pontos
 *     rrfs_loc     - Config local (amperagem controlador, eficiência) +
 *                    calibração previsão (IA 04/05)
 *     rrfs_cnt     - Contadores globais de energia (Ah acumulados,
 *                    era kWh na V14)
 *     rrfs_hidx    - Índices do histórico diário (IA 13)
 *     rh000..rh363 - Histórico diário (364 dias, agora em Ah - versão
 *                    de dados HIST_KEY_VER=3, incompatível com V14)
 *     rrfs_ia      - Padrões aprendidos pela IA 15 (50 × ~40 bytes = ~2KB)
 *     rrfs_iac     - Config da IA 16 (modo, reserva emergência)
 *
 *     [REMOVIDO EM V15]: rrfs_tv (calibração de tensão estimada - não
 *     existe mais, pois a estimativa de tensão foi removida)
 *
 *   RAM (uso em tempo de execução):
 *     GerenciadorIA  - ~8KB  (padrões + buffer snapshot + estado)
 *     RegistroDia    - ~848B (2 × dia atual + temp, em RAM)
 *     Logs           - ~2KB  (20 linhas × 100 chars)
 *     Histórico 60s  - ~1.9KB (3 arrays de corrente × 60 floats)
 *     Previsão 48h   - ~2.4KB (48 × PrevisaoHora, agora com correnteEstimada)
 *     Total estimado: ~15KB de RAM usada pelo sistema
 *     RAM livre típica: >200KB (ESP32 tem ~320KB disponível)
 *
 *   OTIMIZAÇÕES IMPLEMENTADAS (V14 + V15):
 *     - Histórico diário: só 1 dia em RAM, demais na flash
 *     - Cache de consumo histórico: evita leituras de flash repetidas
 *     - Parser JSON manual: evita biblioteca JSON pesada
 *     - Malloc/free na busca meteorológica: arrays temporários na heap
 *     - Buffer snapshot circular: sem realocação
 *     - Padrões IA: substituição circular sem realocação
 *     - Decisões IA: sem alocação dinâmica (tudo em estruturas estáticas)
 *     - [NOVO V15] Remoção de ~200 linhas de estimativa de tensão,
 *       reduzindo RAM usada e complexidade de manutenção
 *
 *
 * ============================================================================
 * RESUMO - CONFIGURAÇÕES DISPONÍVEIS NA INTERFACE WEB (V15)
 * ============================================================================
 *
 *   PÁGINA /bateria:
 *     - Tensão nominal do banco (V) - usada só para cálculo de kW
 *       informativo, não para energia
 *     - Capacidade (Ah)
 *     - Descarga segura (%)
 *     - Calibração manual do % atual (define o Ah inicial)
 *
 *   PÁGINA /redes:
 *     - Corte por % para cada carga (Rede Baixa, Alta, Lâmpadas)
 *     - Controle manual individual de cada relé
 *     - Status/reset de proteção de curto-circuito
 *     - Visualização dos padrões aprendidos (tabela, só se Modo IA ativo)
 *     - Decisão atual da IA e saldo Ah (só se Modo IA ativo)
 *     [ALTERADO EM V15] Modo de operação (Manual/IA) e Reserva de
 *     Emergência MOVERAM para a página /config
 *
 *   PÁGINA /sensores:
 *     - Leituras em tempo real dos sensores (Painel, Casa, Sistema, Bateria)
 *     - [NOVO EM V15] Calibração simples de 1 ponto (informa a corrente
 *       real vista no controlador/alicate, sistema calcula o offset)
 *     - Calibração avançada de 2 pontos (Casa)
 *     - Toggle para ativar/desativar o autoaprendizado de offset do
 *       painel (IA 02), útil quando se prefere manter calibração manual
 *     - Status da calibração de offset (IA 02)
 *     - Status do sinal do inversor
 *
 *   PÁGINA /previsao:
 *     [ALTERADO EM V15] Latitude, longitude, AMPERAGEM DO CONTROLADOR DE
 *     CARGA (em vez de kWp), eficiência
 *     - Tabela de previsão hora a hora (48h) em A e Ah
 *     - Status e precisão da calibração automática (IA 04)
 *     - Decisões automáticas ativas (IA 06)
 *     - Forçar atualização da previsão
 *     - Totais de "hoje restante" e "amanhã" em Ah
 *
 *   PÁGINA /historico:
 *     - Contadores totais de energia (Ah, desde sempre)
 *     - Histórico diário navegável (364 dias) em Ah
 *     - Gráfico por hora do dia atual (barras Painel/Casa em Ah)
 *     - Gráfico de corrente em tempo real (últimos 60s)
 *
 *   PÁGINA /config:
 *     [NOVO EM V15] Modo de operação da IA: Manual ou IA Ativa
 *     [NOVO EM V15] Reserva de emergência (%)
 *     [NOVO EM V15] Reset dos padrões aprendidos pela IA 15
 *     - Reconfigurar rede WiFi
 *     - Sincronizar NTP manualmente
 *     - Atualizar firmware via OTA (upload .bin)
 *     - Alterar senha de acesso
 *     - Reiniciar ESP32
 *
 *
 * ============================================================================
 *                         FIM DA DOCUMENTAÇÃO - V15.0
 * ============================================================================
 */
