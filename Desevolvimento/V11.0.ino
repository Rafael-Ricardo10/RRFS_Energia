/*
 * ============================================================
 * RRFS ENERGIA - SISTEMA COMPLETO DE GERENCIAMENTO DE ENERGIA SOLAR
 * ============================================================
 * Versão: v11.0
 * Plataforma: ESP32
 *
 * DESCRIÇÃO:
 * Sistema inteligente para gerenciamento de energia solar com:
 * - Monitoramento de corrente (painéis e consumo)
 * - Estimativa de tensão e SOC de bateria LiFePO4
 * - Controle automático de cargas por nível de bateria
 * - Proteção contra curto-circuito com diagnóstico automático
 * - Histórico de 364 dias com dados por hora
 * - Previsão meteorológica para otimização de uso
 * - Interface web completa
 * - Calibração automática de sensores e previsões
 * ============================================================
 */

// ============================================================
// INCLUDES
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <time.h>
#include <Update.h>
#include <HTTPClient.h>
#include <nvs_flash.h>

// ============================================================
// PINAGEM DO HARDWARE
// ============================================================

// ── PINOS DE INTERFACE ─────────────────────────────────────
#define LED_AZUL              2   // LED indicador status
#define BOTAO_BOOT            0   // Botão reset (10s pressionado)

// ── PINOS SENSORES DE CORRENTE (ADC1 - WiFi safe) ─────────
#define PIN_SENSOR_PAINEL    34   // ACS758 50A  - ADC1_CH6 - Corrente painel solar
#define PIN_SENSOR_CASA      35   // ACS758 100A - ADC1_CH7 - Corrente consumo casa

// ── PINOS CONTROLE E STATUS ────────────────────────────────
#define PIN_BKP_BATERIA      32   // Sinal relé controlador (futuro uso)
#define PIN_STATUS_INV_ALTA  36   // Entrada: Sinal LED do inversor (VP)

// ── PINOS RELÉS (Lógica invertida: LOW=ligado, HIGH=desligado) ──
#define PIN_RELE_INV_ALTA    27   // Relé chave do inversor
#define PIN_RELE_REDE_BAIXA  26   // Relé rede baixa (tomadas/essencial)
#define PIN_RELE_REDE_ALTA   25   // Relé rede alta (equipamentos pesados)
#define PIN_RELE_LAMP_EXTERNA 14  // Relé lâmpada externa
#define PIN_RELE_LAMP_INTERNA 12  // Relé lâmpada interna

// ============================================================
// CONSTANTES DO SISTEMA - GERAL
// ============================================================
#define MAX_LOGS          20      // Quantidade máxima de logs em RAM
#define LOG_MAX_LEN      100      // Tamanho máximo de cada linha de log
#define DNS_PORT          53      // Porta DNS para modo AP
#define TEMPO_RESET_MS 10000      // Tempo botão pressionado para reset (ms)

// ============================================================
// CONSTANTES DO SISTEMA - SENSORES ADC
// ============================================================
#define ADC_SAMPLES        150     // Amostras para média de cada leitura
#define ADC_VREF         3300     // mV - Tensão de referência ESP32
#define ADC_BITS         4095     // Resolução 12 bits

// ── SENSOR ACS758 50A BIDIRECIONAL (PAINEL) ────────────────
// Sinal DIRETO no ESP32 (sem divisor resistivo)
// VCC = 5V | Offset = VCC/2 = 2500mV | Sensibilidade = 40mV/A
// Conexão invertida proposital: tensão só desce (proteção ESP32)
// Com painel gerando: sinal < 2500mV (seguro para ESP32)
// Com corrente reversa: sinal > 2500mV (aviso automático)
#define PAINEL_OFFSET_MV    2500  // mV com 0A (VCC/2)
#define PAINEL_SENS_MV_A      40  // mV por Ampere (datasheet)

// ── SENSOR ACS758LCB-100B UNIDIRECIONAL (CASA) ─────────────
// Sinal DIRETO no ESP32 (sem divisor resistivo)
// VCC = 5V | Offset = 600mV (fixo de fábrica!) | Sens = 20mV/A
// Sinal máximo: 600 + (100A × 20mV/A) = 2600mV → OK no ESP32
#define CASA_OFFSET_MV       600  // mV com 0A (fixo fabricante)
#define CASA_SENS_MV_A        42  // mV por Ampere (datasheet)

// ── FILTRO E BUFFER ────────────────────────────────────────
#define FILTRO_RUIDO_A       0.3f // Corrente abaixo disso = zero
#define BUFFER_CORRENTE       10  // Tamanho buffer média móvel

// ── FILTRO DE RUÍDO ────────────────────────────────────────
#define FILTRO_RUIDO_A       0.3f // Corrente abaixo = zero
#define BUFFER_CORRENTE        10  // Tamanho buffer média móvel

// ============================================================
// CONSTANTES DO SISTEMA - INVERSOR E RELÉS
// ============================================================
#define INV_THRESHOLD_MV  1500    // Limiar sinal LED inversor (>1500mV = ON)
#define INV_BUFFER_MS     2000    // Buffer confirmação estado inversor (ms)
#define RELE_BUFFER_MS   60000    // Buffer antes de desligar relés (60s)

// ── INTERVALOS DE LEITURA E CONTROLE ───────────────────────
#define INTERVALO_LEITURA_MS  1000   // Leitura sensores a cada 1s
#define INTERVALO_CONTROLE_MS 5000   // Controle energia a cada 5s

// ============================================================
// CONSTANTES DO SISTEMA - PROTEÇÃO CONTRA CURTO
// ============================================================
#define MAX_TENTATIVAS_RELIGAR  3       // Tentativas antes do diagnóstico
#define T_ESPERA_QUEDA_INV   1500       // Aguarda inversor cair (ms)
#define T_ESPERA_STATUS_INV  2500       // Aguarda inversor subir (ms)
#define T_ENTRE_TESTES       1000       // Pausa entre etapas de teste (ms)
#define T_MONITOR_INVERSOR   1000       // Janela monitoramento após pico (ms)
#define T_AGUARDA_QUEDA      1500       // Tempo aguarda status cair
#define T_AGUARDA_STATUS     3000       // Tempo aguarda status subir
#define T_STATUS_AUSENTE     1000       // Tempo sem status antes de agir
#define T_PAUSA_RELIGAR      2000       // Tempo entre desligar e religar

// ============================================================
// CONSTANTES DO SISTEMA - HISTÓRICO
// ============================================================
#define HIST_DIAS        364      // Dias salvos na flash (quase 1 ano)
#define HIST_HORAS        24      // Horas por dia
#define HIST_KEY_VER       2      // Versão da estrutura (para migração)
#define HIST_TENSAO_PONTOS 60     // Pontos gráfico tensão (últimos 60s)

// ============================================================
// CONSTANTES DO SISTEMA - PREVISÃO METEOROLÓGICA
// ============================================================
#define PREV_HORAS        48      // Horas de previsão (48h)
#define PREV_URL_MAX     256      // Tamanho máximo URL da API
#define PREV_INTERVALO  3600000UL // Intervalo atualização (1h em ms)

// ============================================================
// MÁQUINA DE ESTADOS - RESTAURAÇÃO NÃO BLOQUEANTE DE ENERGIA
// ============================================================

// Tempos da sequência (equivalentes aos delays antigos, porém não bloqueantes)
#define RESTORE_T_BAIXA        2000
#define RESTORE_T_INV_BOOT     5000
#define RESTORE_T_INV_STATUS   3000
#define RESTORE_T_INV_POLL      200
#define RESTORE_T_ALTA           200

// ── ANTECIPAÇÃO INTELIGENTE DA REDE ALTA ───────────────────
#define T_ANTECIP_ESTABILIZA   60000UL   // 60s aguardando corrente estabilizar apos ligar em modo teste
#define T_ANTECIP_REAVAL      300000UL   // Reavalia a cada 5 min enquanto mantida ligada
#define T_ANTECIP_COOLDOWN    900000UL   // 15 min de espera apos desistir, antes de tentar de novo
#define MARGEM_ANTECIP_MIN        30     // Margem de seguranca (minutos) entre "tempo ate corte" e "tempo ate geracao melhorar"

// ============================================================
// CONSTANTES DO SISTEMA - REDE WIFI (MODO AP)
// ============================================================
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ============================================================
// ENUMERAÇÕES - MÁQUINA DE ESTADOS PROTEÇÃO CURTO
// ============================================================

// ── Estados principais do diagnóstico ──────────────────────
enum EstadoCurto {
    CURTO_NORMAL = 0,           // Operação normal
    CURTO_RELIGANDO,            // Tentando religar antes do diagnóstico
    CURTO_TESTANDO_ALTA,        // Diagnosticando rede alta
    CURTO_TESTANDO_BAIXA,       // Diagnosticando rede baixa
    CURTO_CONFIRMADO_ALTA,      // Curto confirmado na rede alta
    CURTO_CONFIRMADO_BAIXA,     // Curto confirmado na rede baixa
    CURTO_AGUARDANDO_RESET      // Falha no inversor - aguarda reset manual
};

// ── Subestados do processo de teste ───────────────────────
enum SubEstadoCurto {
    SUB_IDLE = 0,               // Ocioso
    SUB_INVERSOR_DESLIGADO,     // Inversor desligado - aguardando
    SUB_AGUARDANDO_QUEDA,       // Aguardando sinal cair
    SUB_AGUARDANDO_STATUS,      // Aguardando sinal subir
    SUB_INVERSOR_OK_ALTA_OFF,   // Inversor OK sem rede alta
    SUB_INVERSOR_OK_LIGAR_ALTA  // Ligando rede alta para testar
};

// ============================================================
// MÁQUINA DE ESTADOS - RESTAURAÇÃO NÃO BLOQUEANTE DE ENERGIA
// ============================================================
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

// ── Estados da maquina de antecipacao da Rede Alta ─────────
enum EstadoAntecip {
    ANTECIP_IDLE = 0,     // Nada acontecendo, aguardando condicoes favoraveis
    ANTECIP_TESTANDO,     // Ligou em modo teste, aguardando estabilizar (60s)
    ANTECIP_MANTIDO       // Decidiu manter ligada, reavaliando periodicamente
};

// ============================================================
// ESTRUTURAS DE DADOS - CONFIGURAÇÃO
// ============================================================

// ── CONFIGURAÇÃO DA BATERIA ────────────────────────────────
struct ConfigBateria {
    float tensaoNominal;        // V (ex: 12.8V para 4S LiFePO4)
    float capacidadeAh;         // Ah (ex: 100Ah)
    float descargaSegura;       // % (ex: 80% = DOD máximo)
    float energiaTotal;         // Wh (calculado: V × Ah)
    float energiaDisponivel;    // Wh (calculado: total × descarga%)
    bool  calibrada;            // Foi calibrada?
    float energiaAtual;         // Wh energia atual estimada
    float percentualAtual;      // % atual baseado em energiaDisponivel
};

// ── CONFIGURAÇÃO DOS CORTES DE ENERGIA ─────────────────────
struct ConfigRedes {
    float corteBaixa;           // % para desligar rede baixa + inversor
    float corteAlta;            // % para desligar rede alta
    float corteLampExt;         // % para desligar lâmpada externa
    float corteLampInt;         // % para desligar lâmpada interna
};

// ── CONFIGURAÇÃO DO LOCAL (PREVISÃO) ───────────────────────
struct ConfigLocal {
    double  latitude;           // Coordenada latitude (alta precisão)
    double  longitude;          // Coordenada longitude (alta precisão)
    float   painelKWp;          // kWp instalado (ex: 2.5kWp)
    float   eficiencia;         // Eficiência do sistema (0.0-1.0)
    bool    configurado;        // Já foi configurado?
    // Calibração automática da previsão
    float   fatorCalib;         // Fator de correção aprendido
    uint8_t amostrasCalib;      // Quantidade de amostras de calibração
};

// ============================================================
// ESTRUTURAS DE DADOS - SENSORES E ESTADO
// ============================================================

// ── CALIBRAÇÃO DE SENSOR DE CORRENTE ───────────────────────
struct CalibSensor {
    int   offsetMV;             // mV quando corrente = 0A (ponto zero real)
    float sensMvA;              // mV por Ampere real (após divisor)
    bool  invertido;            // Sinal invertido? (cabos trocados)
    bool  calibrado;            // Foi calibrado manualmente?
};

// ── BUFFER DE MÉDIA MÓVEL ──────────────────────────────────
struct BufferMediaMovel {
    float   amostras[BUFFER_CORRENTE];  // Array de amostras
    uint8_t indice;                     // Índice atual (circular)
    uint8_t total;                      // Quantas amostras válidas
    bool    cheio;                      // Buffer completamente preenchido?
};

// ── DADOS DOS SENSORES (LEITURAS ATUAIS) ───────────────────
struct DadosSensores {
    float correntePainel;       // A (positivo = gerando)
    float correnteCasa;         // A (positivo = consumindo)
    float correnteBateria;      // A (positivo = carregando, negativo = descarregando)
    float potenciaPainel;       // W estimado (corrente × tensão)
    float potenciaCasa;         // W estimado
    float potenciaBateria;      // W estimado
    int   sinalPainel_mV;       // mV bruto sensor painel
    int   sinalCasa_mV;         // mV bruto sensor casa
    int   sinalInversor_mV;     // mV bruto sensor inversor (LED)
    bool  inversoreAtivo;       // Estado inversor (com buffer temporal)
};

// ── ESTIMATIVA DE TENSÃO DA BATERIA ────────────────────────
struct EstimativaTensao {
    float   tensaoAtual;        // V estimado atual
    float   tensaoMaxima;       // V quando 100% carregando (painel ON)
    float   tensaoNominalReal;  // V tensão nominal do banco (células × 3.2V)
    float   fatorBanco;         // Multiplicador células série (4S=4.0, 8S=8.0)
    float   fatorCalibTensao;   // Fator de calibração aprendido
    uint8_t amostrasCalib;      // Amostras de calibração coletadas
    bool    painelAtivo;        // Painel gerando agora?
};

// ── ESTADO DOS RELÉS ───────────────────────────────────────
struct EstadoReles {
    bool redeBaixa;             // true = ligada
    bool redesAlta;             // true = ligada
    bool lampExt;               // true = ligada
    bool lampInt;               // true = ligada
    bool inversor;              // true = ligado
};

// ── TIMERS PARA BUFFER DE DESLIGAMENTO ─────────────────────
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
    // Buffer estado inversor
    unsigned long inversorUltimoHigh;
    bool          inversorEstadoReal;
};

// ── ESTIMATIVA DE TEMPO RESTANTE ───────────────────────────
struct EstimativaTempo {
    bool   valido;              // Estimativa é válida?
    int    horas;               // Horas restantes
    int    minutos;             // Minutos restantes
    bool   carregando;          // true=carregando, false=descarregando
    float  taxaWh;              // Wh/h atual (positivo=carregando)
};

// ── GERENCIADOR DE PROTEÇÃO CONTRA CURTO ───────────────────
struct GerenciadorCurto {
    EstadoCurto    estado;      // Estado atual da máquina
    SubEstadoCurto subEstado;   // Subestado do processo
    unsigned long  tInicio;     // Timestamp início do estado atual
    bool           redeAltaCurto;   // Rede alta em curto?
    bool           redeBaixaCurto;  // Rede baixa em curto?
    bool           monitorando;     // Está monitorando inversor?
    uint8_t        tentativasReligar; // Tentativas de religar antes do diagnóstico
};

// ============================================================
// ESTRUTURAS DE DADOS - HISTÓRICO
// ============================================================

// ── REGISTRO POR HORA ──────────────────────────────────────
struct RegistroHora {
    float painelWh;             // Wh gerado pelo painel nesta hora
    float casaWh;               // Wh consumido pela casa nesta hora
    float batCargWh;            // Wh carregado na bateria nesta hora
    float batDescWh;            // Wh descarregado da bateria nesta hora
};

// ── REGISTRO POR DIA ───────────────────────────────────────
struct RegistroDia {
    uint16_t ano;
    uint8_t  mes;
    uint8_t  dia;
    bool     valido;            // Dia tem dados?
    RegistroHora horas[HIST_HORAS]; // 24 horas (~384 bytes)
    // Totais do dia (calculados)
    float totalPainelWh;
    float totalCasaWh;
    float totalBatCargWh;
    float totalBatDescWh;
};

// ── CONTADORES GLOBAIS (NUNCA ZERADOS) ─────────────────────
struct ContadoresGlobais {
    float painelKwh;            // Total gerado pelo painel desde sempre (kWh)
    float casaKwh;              // Total consumido pela casa desde sempre (kWh)
    float batCargKwh;           // Total carregado na bateria (kWh)
    float batDescKwh;           // Total descarregado da bateria (kWh)
};

// ── ACUMULADORES DA HORA ATUAL ─────────────────────────────
struct AcumHora {
    float painelWh;
    float casaWh;
    float batCargWh;
    float batDescWh;
    uint8_t  horaAtual;         // Hora sendo acumulada (0-23)
    uint8_t  diaAtual;
    uint8_t  mesAtual;
    uint16_t anoAtual;
    unsigned long ultimoAcum;   // millis() do último acúmulo
};

// ============================================================
// ESTRUTURAS DE DADOS - PREVISÃO METEOROLÓGICA
// ============================================================

// ── PREVISÃO POR HORA ──────────────────────────────────────
struct PrevisaoHora {
    uint8_t  hora;              // 0-23
    uint8_t  dia;
    uint8_t  mes;
    uint16_t ano;
    float    radiacao;          // W/m² radiação direta normal
    float    radiacaoDifusa;    // W/m² radiação difusa
    float    coberturaNuvens;   // % cobertura de nuvens (0-100)
    float    temperatura;       // °C
    float    geracaoEstimada;   // Wh estimado de geração
    bool     valido;
};

// ── RESULTADO DA PREVISÃO ──────────────────────────────────
struct ResultadoPrev {
    PrevisaoHora horas[PREV_HORAS];     // Array de 48 horas
    uint8_t      totalHoras;            // Quantas horas preenchidas
    float        geracaoHojeRestante;   // Wh restante hoje
    float        geracaoAmanha;         // Wh total amanhã
    float        geracaoTotal48h;       // Wh total 48h
    unsigned long ultimaAtualizacao;    // millis() da última busca
    bool         valido;                // Previsão válida?
    char         erro[64];              // Mensagem de erro se falhou
};

// ── DECISÕES AUTOMÁTICAS DO SISTEMA ────────────────────────
struct DecisaoSistema {
    bool  desligarRedeAltaNoite;        // Desligar rede alta à noite?
    bool  desligarLampExtNoite;         // Desligar lamp ext à noite?
    bool  desligarLampIntDia;           // NOVO: Desligar lamp interna por deficit critico DIURNO
    float horaDesligarRedeAlta;         // Hora prevista para desligar
    float horaLigarRedeAlta;            // Hora prevista para religar
    bool  alertaEnergiaInsuficiente;    // Alerta de energia insuficiente?
    char  motivo[128];                  // Motivo da decisão
};

// ============================================================
// MÁQUINA DE ESTADOS - RESTAURAÇÃO NÃO BLOQUEANTE DE ENERGIA
// ============================================================
struct RestauroEnergia {
    EstadoRestauro estado;
    unsigned long  tInicio;
};
RestauroEnergia restauro = { RESTORE_OCIOSO, 0 };

struct GerenciadorAntecip {
    EstadoAntecip estado;
    unsigned long tInicio;      // Timestamp de quando entrou em TESTANDO
    unsigned long tUltimaAval;  // Timestamp da ultima reavaliacao (estado MANTIDO)
    unsigned long tCooldown;    // Timestamp ate quando NAO tentar de novo apos desistir (0 = sem cooldown)
};
GerenciadorAntecip antecipAlta = { ANTECIP_IDLE, 0, 0, 0 };

// ============================================================
// VARIÁVEIS GLOBAIS - SISTEMA GERAL
// ============================================================
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP   = false;          // true = modo Access Point
bool logado   = false;          // Usuário logado na interface web?

// ── LED E BOTÃO ────────────────────────────────────────────
unsigned long ultimoBlink = 0;
bool          ledState    = false;
bool          blinkAtivo  = true;

unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

// ── SISTEMA DE LOGS ────────────────────────────────────────
char    logs[MAX_LOGS][LOG_MAX_LEN];
uint8_t totalLogs = 0;
uint8_t logIndex  = 0;
char    tempBuf[200];           // Buffer temporário para sprintf

// ============================================================
// VARIÁVEIS GLOBAIS - CONFIGURAÇÕES E ESTADO
// ============================================================

// ── CONFIGURAÇÕES ──────────────────────────────────────────
ConfigBateria  bateria  = {12.8f, 100.0f, 80.0f, 0, 0, false, 0, 0};
ConfigRedes    redes    = {10.0f, 30.0f, 50.0f, 0.0f};
ConfigLocal    configLocal = {0, 0, 0, 0.80f, false, 1.0f, 0};

// ── SENSORES E CALIBRAÇÃO ──────────────────────────────────
DadosSensores      sensores = {0};
CalibSensor        calibPainel;
CalibSensor        calibCasa;
BufferMediaMovel   bufPainel = {0};
BufferMediaMovel   bufCasa   = {0};

// ── ESTIMATIVA DE TENSÃO ───────────────────────────────────
EstimativaTensao estimTensao = {
    12.8f, 14.0f, 12.8f,
    1.0f, 1.0f, 0, false
};

// ── ESTADO DOS RELÉS ───────────────────────────────────────
EstadoReles    estados  = {true, true, true, true, false};
TimersBuffer   timers   = {0};

// ── PROTEÇÃO CONTRA CURTO ──────────────────────────────────
GerenciadorCurto curto = {
    CURTO_NORMAL, SUB_IDLE,
    0, false, false, false, 0
};

// ── INTERVALOS DE TEMPO ────────────────────────────────────
unsigned long ultimaLeitura  = 0;
unsigned long ultimoControle = 0;
unsigned long ultimoContagem = 0;  // Coulomb counting bateria

// ============================================================
// VARIÁVEIS GLOBAIS - HISTÓRICO
// ============================================================

// ── Dia atual em RAM (acesso rápido) ───────────────────────
RegistroDia histDiaAtual;           // ~424 bytes
RegistroDia histDiaTemp;            // Buffer temporário para leitura

// ── Índices de controle ────────────────────────────────────
uint16_t histIndiceAtual = 0;       // Índice do dia atual (0-363)
uint16_t histTotalDias   = 0;       // Quantos dias válidos no histórico

// ── Contadores e acumuladores ──────────────────────────────
ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora acum = {0, 0, 0, 0, 255, 0, 0, 0, 0};

// ── Histórico de tensão para gráfico (últimos 60s) ─────────
float   histTensaoV[HIST_TENSAO_PONTOS] = {0};
float   histCorrPainel[HIST_TENSAO_PONTOS] = {0};
float   histCorrCasa[HIST_TENSAO_PONTOS]   = {0};
float   histCorrBat[HIST_TENSAO_PONTOS]    = {0};
uint8_t histTensaoIdx   = 0;
bool    histTensaoFull  = false;
unsigned long ultimoHistTensao = 0;

// ============================================================
// VARIÁVEIS GLOBAIS - PREVISÃO E DECISÕES
// ============================================================
ResultadoPrev    previsao     = {0};
DecisaoSistema decisao = {false, false, false, 0, 0, false, ""};


unsigned long ultimaPrev      = 0;      // Última busca meteorológica
bool          buscandoPrev    = false;  // Busca em andamento?

// ============================================================
// VARIÁVEIS GLOBAIS - REDE WIFI
// ============================================================
// Credenciais WiFi salvas globalmente para reconexão automática
String gWifiSSID = "";
String gWifiPASS = "";
String gWifiIP   = "";
String gWifiGW   = "";
String gWifiMASK = "";
String gWifiDNS  = "";

// ============================================================
// VARIÁVEIS GLOBAIS - Iluminaçâo Externa
// ============================================================
bool ehDia = false; // false = noite, true = dia

// ============================================================
// PROTÓTIPOS DE FUNÇÕES - ORGANIZADO POR SISTEMA
// ============================================================

// ── SISTEMA: UTILIDADES E HELPERS ──────────────────────────
EstimativaTempo calcularTempoRestante();
String tempoAteCorte(float pctCorte, float taxaWhH);
String fmtCorrente(float a);
String fmtPotencia(float w);

// ── SISTEMA: LOGS ──────────────────────────────────────────
void adicionarLog(const char* msg);
void logRAM();

// ── SISTEMA: LED ───────────────────────────────────────────
void setupLED();
void atualizarLED();
void ledBlink();
void ledFixo(bool v);

// ── SISTEMA: BOTÃO ─────────────────────────────────────────
void setupBotao();
void verificarReset();

// ── SISTEMA: RELÉS ─────────────────────────────────────────
void setupReles();
void setRele(uint8_t pin, bool ligar);
void ligarRedeBaixa();
void desligarRedeBaixa();
void ligarRedeAlta();
void desligarRedeAlta();
void ligarLampExt();
void desligarLampExt();
void ligarLampInt();
void desligarLampInt();
void ligarInversor();
void desligarInversor();

// ── SISTEMA: SENSORES ADC ──────────────────────────────────
int lerADC(uint8_t pin);
int adcParaMv(int adcVal);
float adicionarAmostra(BufferMediaMovel& buf, float novaAmostra);
float lerCorrentePainel();
float lerCorrenteCasa();
bool lerStatusInversor();
void lerSensores();

// ── SISTEMA: BATERIA ───────────────────────────────────────
void calcularBateria();
void verificarCalibracaoAutomatica();
void atualizarEnergiBateria();
void calcularFatorBanco();
float soc2tensaoLiFePO4(float soc);
float estimarTensaoBateria();
void calibrarTensaoAutomatica();
void salvarCalibTensao();
void carregarCalibTensao();

// ── SISTEMA: HISTÓRICO ─────────────────────────────────────
void calcularTotaisDia(RegistroDia& dia);
void salvarContadores();
void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void salvarHistorico();
void carregarHistorico();
void acumularEnergia();

// ── SISTEMA: PREVISÃO METEOROLÓGICA ────────────────────────
void salvarConfigLocal();
void carregarConfigLocal();
void calibrarPrevisao();
float calcularGeracaoHora(float radiacaoWm2, float radiacaoDifusa,
                          float nuvens, float tempC);
void buscarPrevisaoMeteo();
void tomarDecisoes();
void verificarReligamentoAuto();

float consumoMedioParaHora(uint8_t horaAlvo, bool* temDados = nullptr);
float horasAteGeracaoSuperar(float refWatts);
void  avaliarAntecipacaoRedeAlta();

// ── SISTEMA: CONTROLE DE ENERGIA ───────────────────────────
void controlarEnergia();

// ── SISTEMA: PROTEÇÃO CONTRA CURTO ─────────────────────────
void monitorarInversor();
void resetCurtoAlta();
void resetCurtoBaixa();
String htmlStatusCurto();

// ── SISTEMA: PREFERENCES ───────────────────────────────────
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

// ── SISTEMA: REDE WIFI ─────────────────────────────────────
void iniciarAP();
bool iniciarWiFi(const String& ssid, const String& pass,
                 const String& ip,   const String& gw,
                 const String& mask, const String& dns);
void tentarReconectarWiFi();
void sincronizarNTP();

// ── SISTEMA: SERVIDOR WEB ──────────────────────────────────
void configurarRotas();
String gerarPagina(const String& titulo, const String& conteudo,
                   bool comNav = true);
String barraProgresso(float pct, const String& label = "");
String statusCard(const String& icone, const String& label,
                  const String& valor, const String& sub,
                  const String& tipo = "");

// ── HANDLERS WEB ───────────────────────────────────────────
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



// ============================================================
// SISTEMA: UTILIDADES E HELPERS
// ============================================================

// ── Calcula tempo restante de bateria ──────────────────────
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

// ── Calcula tempo até um percentual de corte ───────────────
String tempoAteCorte(float pctCorte, float taxaWhH) {
    if (!bateria.calibrada) return "--";
    if (taxaWhH >= 0) return "--";
    if (bateria.energiaDisponivel <= 0) return "--";

    float energiaCorte = bateria.energiaDisponivel * (pctCorte / 100.0f);
    float energiaAteCorte = bateria.energiaAtual - energiaCorte;
    if (energiaAteCorte <= 0) return "Agora";

    float horas = energiaAteCorte / fabsf(taxaWhH);
    if (horas > 999) return ">999h";

    int h = (int)horas;
    int m = (int)((horas - h) * 60);

    char buf[20];
    if (h == 0) snprintf(buf, sizeof(buf), "%dm",   m);
    else        snprintf(buf, sizeof(buf), "%dh%02dm", h, m);
    return String(buf);
}

// ── Formata corrente com sinal e unidade ───────────────────
String fmtCorrente(float a) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%.1fA", a);
    return String(buf);
}

// ── Formata potência ───────────────────────────────────────
String fmtPotencia(float w) {
    char buf[20];
    if (fabsf(w) >= 1000)
        snprintf(buf, sizeof(buf), "%.2fkW", w / 1000.0f);
    else
        snprintf(buf, sizeof(buf), "%.0fW", w);
    return String(buf);
}

// ============================================================
// SISTEMA: LOGS
// ============================================================

// ── Adiciona mensagem ao sistema de logs ───────────────────
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

// ── Log de memória RAM disponível ──────────────────────────
void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf),
             "RAM: %.1f KB livres", (unsigned)esp_get_free_heap_size() / 1024.0f);
    adicionarLog(tempBuf);
}

// ============================================================
// SISTEMA: LED INDICADOR
// ============================================================

// ── Inicializa LED ─────────────────────────────────────────
void setupLED() {
    pinMode(LED_AZUL, OUTPUT);
    digitalWrite(LED_AZUL, LOW);
}

// ── Atualiza estado do LED (blink se ativo) ────────────────
void atualizarLED() {
    if (!blinkAtivo) return;
    if (millis() - ultimoBlink >= 500) {
        ultimoBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_AZUL, ledState);
    }
}

// ── Ativa modo blink ───────────────────────────────────────
void ledBlink() {
    blinkAtivo = true;
    ultimoBlink = millis();
}

// ── LED fixo (ligado ou desligado) ─────────────────────────
void ledFixo(bool v) {
    blinkAtivo = false;
    digitalWrite(LED_AZUL, v);
}

// ============================================================
// SISTEMA: BOTÃO DE RESET
// ============================================================

// ── Inicializa botão ───────────────────────────────────────
void setupBotao() {
    pinMode(BOTAO_BOOT, INPUT_PULLUP);
}

// ── Verifica reset por botão (10s pressionado) ─────────────
void verificarReset() {
    bool pressionado = (digitalRead(BOTAO_BOOT) == LOW);

    if (pressionado && !botaoSegurando) {
        botaoSegurando     = true;
        botaoPressionadoEm = millis();
    }

    if (!pressionado) botaoSegurando = false;

    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET TOTAL por botao BOOT - apagando TUDO (fabrica)");

        // Aviso visual antes de apagar (15 piscadas rápidas)
        for (int i = 0; i < 15; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(80);
            digitalWrite(LED_AZUL, LOW);  delay(80);
        }

        // ══════════════════════════════════════════════════════════
        // APAGA A PARTIÇÃO NVS INTEIRA
        // Isso limpa TODOS os namespaces de uma vez, incluindo:
        // - rrfs, rrfs_tv, rrfs_loc, rrfs_cnt, rrfs_hidx
        // - TODOS os rh000..rh363 (histórico diário) sem precisar
        //   listar um por um
        // - Qualquer partição/página corrompida também é eliminada
        //   e recriada do zero
        // ══════════════════════════════════════════════════════════
        esp_err_t errErase = nvs_flash_erase();
        if (errErase != ESP_OK) {
            adicionarLog("AVISO: erro ao apagar NVS - tentando mesmo assim");
        }

        // Recria a partição NVS limpa e pronta para uso
        esp_err_t errInit = nvs_flash_init();
        if (errInit != ESP_OK) {
            adicionarLog("ERRO CRITICO: falha ao recriar NVS apos reset");
        } else {
            adicionarLog("NVS apagada e recriada com sucesso");
        }

        // LED fixo aceso confirmando conclusão antes de reiniciar
        digitalWrite(LED_AZUL, HIGH);
        delay(500);
        digitalWrite(LED_AZUL, LOW);
        delay(300);

        adicionarLog("Reiniciando sistema limpo...");
        delay(300);
        ESP.restart();
    }
}

// ============================================================
// SISTEMA: CONTROLE DE RELÉS
// ============================================================

// ── Inicializa todos os relés ──────────────────────────────
void setupReles() {
    // Todos os relés iniciam DESLIGADOS no hardware
    pinMode(PIN_RELE_INV_ALTA,     OUTPUT); digitalWrite(PIN_RELE_INV_ALTA,     LOW);
    pinMode(PIN_RELE_REDE_BAIXA,   OUTPUT); digitalWrite(PIN_RELE_REDE_BAIXA,   LOW);
    pinMode(PIN_RELE_REDE_ALTA,    OUTPUT); digitalWrite(PIN_RELE_REDE_ALTA,    LOW);
    pinMode(PIN_RELE_LAMP_EXTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_EXTERNA, LOW);
    pinMode(PIN_RELE_LAMP_INTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_INTERNA, LOW);

    // Entradas
    pinMode(PIN_STATUS_INV_ALTA, INPUT);
    pinMode(PIN_BKP_BATERIA,     INPUT);

    // Estado inicial: tudo desligado
    estados.redeBaixa = false;
    estados.redesAlta = false;
    estados.lampExt   = false;
    estados.lampInt   = false;
    estados.inversor  = false;

    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;

    adicionarLog("Reles iniciados: todos OFF");
}

// ── Liga/desliga relé (lógica invertida) ───────────────────
void setRele(uint8_t pin, bool ligar) {
    digitalWrite(pin, ligar ? HIGH : LOW);
}

// ── Funções individuais de controle de relés ───────────────
void ligarRedeBaixa()   { setRele(PIN_RELE_REDE_BAIXA,   true);  estados.redeBaixa = true;  }
void desligarRedeBaixa(){ setRele(PIN_RELE_REDE_BAIXA,   false); estados.redeBaixa = false; }
void ligarRedeAlta()    { setRele(PIN_RELE_REDE_ALTA,    true);  estados.redesAlta = true;  }
void desligarRedeAlta() { setRele(PIN_RELE_REDE_ALTA,    false); estados.redesAlta = false; }
void ligarLampExt()     { setRele(PIN_RELE_LAMP_EXTERNA, true);  estados.lampExt   = true;  }
void desligarLampExt()  { setRele(PIN_RELE_LAMP_EXTERNA, false); estados.lampExt   = false; }
void ligarLampInt()     { setRele(PIN_RELE_LAMP_INTERNA, true);  estados.lampInt   = true;  }
void desligarLampInt()  { setRele(PIN_RELE_LAMP_INTERNA, false); estados.lampInt   = false; }
void ligarInversor()    { setRele(PIN_RELE_INV_ALTA,     true);  estados.inversor  = true;  }
void desligarInversor() { setRele(PIN_RELE_INV_ALTA,     false); estados.inversor  = false; }

// ============================================================
// SISTEMA: LEITURA DE SENSORES ADC
// ============================================================

// ── Lê ADC com média de múltiplas amostras ─────────────────
int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        soma += analogRead(pin);
        delayMicroseconds(100);
    }
    return (int)(soma / ADC_SAMPLES);
}

// ── Converte valor ADC para mV ─────────────────────────────
int adcParaMv(int adcVal) {
    return (int)((long)adcVal * ADC_VREF / ADC_BITS);
}

// ── Adiciona amostra ao buffer média móvel ─────────────────
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

// ── Lê corrente do painel solar ────────────────────────────
float lerCorrentePainel() {
    int  adc = lerADC(PIN_SENSOR_PAINEL);
    int  mv  = adcParaMv(adc);
    sensores.sinalPainel_mV = mv;

    // ══════════════════════════════════════════════════════════
    // AUTO-CALIBRAÇÃO DO OFFSET EM TEMPO REAL
    // O offset real pode diferir do datasheet (2500mV)
    // Aprende o offset real quando corrente é claramente zero
    // ══════════════════════════════════════════════════════════
    static int    offsetAprendido    = PAINEL_OFFSET_MV; // começa com 2500
    static bool   offsetCalibrado    = false;
    static long   somaOffset         = 0;
    static int    amostrasOffset     = 0;
    static unsigned long tUltimoZero = 0;

    // Condições para aprender o offset:
    // - Noite (sem sol): hora entre 20h e 5h
    // - OU corrente da casa também ~zero (sistema em repouso)
    struct tm tiOff;
    bool podeAprender = false;

    if (getLocalTime(&tiOff)) {
        int hora = tiOff.tm_hour;
        // Madrugada: certamente não há geração solar
        if (hora >= 22 || hora <= 4) {
            podeAprender = true;
        }
    }

    // Aprende offset em janelas de 30 segundos durante a noite
    if (podeAprender) {
        if (tUltimoZero == 0) tUltimoZero = millis();

        somaOffset += mv;
        amostrasOffset++;

        // Após 30 segundos de amostras noturnas
        if (millis() - tUltimoZero >= 30000 && amostrasOffset >= 20) {
            int offsetNovo = (int)(somaOffset / amostrasOffset);

            // Aceita só se estiver próximo do esperado (±100mV)
            if (abs(offsetNovo - PAINEL_OFFSET_MV) <= 100) {
                offsetAprendido = offsetNovo;
                offsetCalibrado = true;

                // Atualiza a calibração global
                if (abs(offsetNovo - calibPainel.offsetMV) > 5) {
                    calibPainel.offsetMV = offsetNovo;
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Painel offset aprendido: %dmV (era %dmV)",
                             offsetNovo, PAINEL_OFFSET_MV);
                    adicionarLog(tempBuf);
                }
            }
            // Reseta para próxima janela
            somaOffset     = 0;
            amostrasOffset = 0;
            tUltimoZero    = 0;
        }
    } else {
        // Durante o dia: reseta acumulador
        tUltimoZero    = 0;
        somaOffset     = 0;
        amostrasOffset = 0;
    }

    // Usa offset aprendido ou padrão
    int offsetUsado = offsetCalibrado ?
    offsetAprendido :
    calibPainel.offsetMV;

    // ══════════════════════════════════════════════════════════
    // CÁLCULO DA CORRENTE
    // Cabos INVERTIDOS: sinal DESCE = painel gerando (positivo)
    //                   sinal SOBE  = corrente reversa (negativo)
    // ══════════════════════════════════════════════════════════
    float correnteBruta = (float)(mv - offsetUsado)
    / calibPainel.sensMvA;

    // Inverte: cabos invertidos propositalmente
    correnteBruta = -correnteBruta;

    // ══════════════════════════════════════════════════════════
    // FILTRO INTELIGENTE - ZONA MORTA ADAPTATIVA
    // Zona morta maior para correntes negativas (direção errada)
    // Zona morta menor para correntes positivas (geração real)
    // ══════════════════════════════════════════════════════════

    // Corrente negativa (direção errada):
    // Só aceita se for maior que 1A (evita ruído)
    if (correnteBruta < 0.0f) {
        if (correnteBruta > -1.0f) {
            // Ruído ou erro de offset - trata como zero
            correnteBruta = 0.0f;
        } else {
            // Corrente reversa real > 1A: avisa e zera
            static unsigned long tAvisoRev = 0;
            if (millis() - tAvisoRev > 60000) {
                tAvisoRev = millis();
                adicionarLog("AVISO: Corrente reversa no painel > 1A!");
                adicionarLog("Verifique conexoes do sensor!");
            }
            correnteBruta = 0.0f; // Zera mesmo assim por segurança
        }
    }

    // Corrente positiva (geração real):
    // Zona morta menor - detecta até ~0.15A
    if (correnteBruta > 0.0f && correnteBruta < 0.15f) {
        correnteBruta = 0.0f;
    }

    // ══════════════════════════════════════════════════════════
    // BUFFER DE MÉDIA MÓVEL
    // ══════════════════════════════════════════════════════════
    float corrMedia = adicionarAmostra(bufPainel, correnteBruta);

    // Filtro final na média
    if (corrMedia < 0.0f) corrMedia = 0.0f;
    if (corrMedia > 0.0f && corrMedia < 0.15f) corrMedia = 0.0f;

    return corrMedia;
}

// ── Lê corrente da casa ────────────────────────────────────
float lerCorrenteCasa() {
    int   adc = lerADC(PIN_SENSOR_CASA);
    int   mv  = adcParaMv(adc);
    sensores.sinalCasa_mV = mv;

    // Sensor ACS758LCB-100B unidirecional - sinal DIRETO no ESP32
    // Offset FIXO = 600mV (fabricante) | Sensibilidade = 40mV/A
    // Sinal abaixo do offset = ruído ou sem corrente = zero
    if (mv <= calibCasa.offsetMV) {
        adicionarAmostra(bufCasa, 0.0f);
        return 0.0f;
    }

    float correnteBruta = (float)(mv - calibCasa.offsetMV)
    / calibCasa.sensMvA;

    // Unidirecional: nunca negativo
    if (correnteBruta < 0.0f) correnteBruta = 0.0f;

    // Filtro de ruído
    if (correnteBruta < FILTRO_RUIDO_A) {
        correnteBruta = 0.0f;
    }

    // Passa pelo buffer de média móvel
    float corrMediaMovel = adicionarAmostra(bufCasa, correnteBruta);

    // Filtro final
    if (corrMediaMovel < FILTRO_RUIDO_A) {
        corrMediaMovel = 0.0f;
    }

    return corrMediaMovel;
}

// ── Lê status do inversor (sinal LED) ──────────────────────
bool lerStatusInversor() {
    int adc = lerADC(PIN_STATUS_INV_ALTA);
    int mv  = adcParaMv(adc);
    sensores.sinalInversor_mV = mv;

    // Se a chave do inversor está DESLIGADA, status nunca aparece
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
        // Sinal baixo com chave ligada: usa buffer de 2s
        if (timers.inversorEstadoReal &&
            millis() - timers.inversorUltimoHigh >= INV_BUFFER_MS) {
            timers.inversorEstadoReal = false;
            }
    }

    return timers.inversorEstadoReal;
}

// ── Leitura completa de todos os sensores ──────────────────
void lerSensores() {
    // Lê correntes
    sensores.correntePainel = lerCorrentePainel();
    sensores.correnteCasa   = lerCorrenteCasa();

    // Corrente bateria = Painel - Casa
    sensores.correnteBateria = sensores.correntePainel - sensores.correnteCasa;

    // Estima tensão atual
    estimTensao.tensaoAtual = estimarTensaoBateria();

    // Calcula potências com tensão estimada
    sensores.potenciaPainel  = sensores.correntePainel  * estimTensao.tensaoAtual;
    sensores.potenciaCasa    = sensores.correnteCasa    * estimTensao.tensaoAtual;
    sensores.potenciaBateria = sensores.correnteBateria * estimTensao.tensaoAtual;

    // Lê status inversor
    sensores.inversoreAtivo = lerStatusInversor();

    // Atualiza energia da bateria
    atualizarEnergiBateria();

    // Verificações
    verificarCalibracaoAutomatica();

    // Calibração automática de tensão
    static unsigned long tCalibV = 0;
    if (millis() - tCalibV >= 60000) {
        tCalibV = millis();
        calibrarTensaoAutomatica();
    }

    // Atualiza histórico de tensão (para gráfico)
    if (millis() - ultimoHistTensao >= 1000) {
        ultimoHistTensao = millis();

        histTensaoV     [histTensaoIdx] = estimTensao.tensaoAtual;
        histCorrPainel  [histTensaoIdx] = sensores.correntePainel;
        histCorrCasa    [histTensaoIdx] = sensores.correnteCasa;
        histCorrBat     [histTensaoIdx] = sensores.correnteBateria;

        histTensaoIdx = (histTensaoIdx + 1) % HIST_TENSAO_PONTOS;
        if (histTensaoIdx == 0) histTensaoFull = true;
    }
}

// ============================================================
// SISTEMA: GERENCIAMENTO DE BATERIA
// ============================================================

// ── Calcula parâmetros da bateria ──────────────────────────
void calcularBateria() {
    bateria.energiaTotal      = bateria.tensaoNominal * bateria.capacidadeAh;
    bateria.energiaDisponivel = bateria.energiaTotal * (bateria.descargaSegura / 100.0f);

    // Recalcula fator do banco
    calcularFatorBanco();
}

// ── Calibração automática (detecta 100%) ───────────────────
void verificarCalibracaoAutomatica() {
    // Se painel gerando > 0.5A e bateria próxima de 0 (±0.5A)
    // significa que BMS cortou → bateria 100%
    if (sensores.correntePainel > 0.5f &&
        fabsf(sensores.correnteBateria) <= 0.5f &&
        sensores.correnteCasa > 0.1f) {

        if (!bateria.calibrada) {
            bateria.energiaAtual    = bateria.energiaDisponivel;
            bateria.percentualAtual = 100.0f;
            bateria.calibrada       = true;

            prefs.begin("rrfs", false);
            prefs.putFloat("bat_energia", bateria.energiaAtual);
            prefs.putBool ("bat_calib",   true);
            prefs.end();

            adicionarLog("Bateria calibrada: 100% detectado");
        }
        }
}

// ── Coulomb counting (integração de corrente) ──────────────
void atualizarEnergiBateria() {
    if (!bateria.calibrada) return;

    unsigned long agora  = millis();
    float deltaT = (float)(agora - ultimoContagem) / 3600000.0f;
    ultimoContagem = agora;

    // Usa tensão ESTIMADA para cálculo de potência real
    float potBat = sensores.correnteBateria * estimTensao.tensaoAtual;

    // Integra energia (Wh)
    bateria.energiaAtual += potBat * deltaT;

    // Limita
    if (bateria.energiaAtual < 0.0f)
        bateria.energiaAtual = 0.0f;
    if (bateria.energiaAtual > bateria.energiaDisponivel)
        bateria.energiaAtual = bateria.energiaDisponivel;

    // Percentual
    if (bateria.energiaDisponivel > 0.0f)
        bateria.percentualAtual = (bateria.energiaAtual / bateria.energiaDisponivel) * 100.0f;
    else
        bateria.percentualAtual = 0.0f;
}

// ── Tabela SOC → Tensão LiFePO4 ────────────────────────────
float soc2tensaoLiFePO4(float soc) {
    // Tabela SOC → V por célula (repouso)
    static const float socPts[]  = {  0,   5,  10,  15,  20,  25,  30,  40,  50,  60,  70,  80,  90,  95, 100};
    static const float vPts[]    = {3.00f,3.13f,3.20f,3.23f,3.26f,3.28f,3.30f,3.32f,3.33f,3.34f,3.35f,3.36f,3.37f,3.38f,3.40f};
    const int n = 15;

    if (soc <= 0)   return vPts[0];
    if (soc >= 100) return vPts[n-1];

    // Interpolação linear
    for (int i = 0; i < n - 1; i++) {
        if (soc >= socPts[i] && soc <= socPts[i+1]) {
            float t = (soc - socPts[i]) / (socPts[i+1] - socPts[i]);
            return vPts[i] + t * (vPts[i+1] - vPts[i]);
        }
    }
    return vPts[n-1];
}

// ── Calcula fator do banco (número de células série) ───────
void calcularFatorBanco() {
    // Número de células em série (LiFePO4 = 3.2V nominal)
    float nCelulas = bateria.tensaoNominal / 3.2f;

    // Arredonda para o inteiro mais próximo (4S, 8S, 12S, 16S...)
    int nCelulasInt = (int)(nCelulas + 0.5f);
    if (nCelulasInt < 4)  nCelulasInt = 4;   // mínimo 12V (4S)
    if (nCelulasInt > 16) nCelulasInt = 16;  // máximo 48V (16S)

    estimTensao.fatorBanco = (float)nCelulasInt;

    // Tensão máxima de carga (3.65V/célula)
    estimTensao.tensaoMaxima = 3.5f * nCelulasInt * estimTensao.fatorCalibTensao;

    // Tensão nominal real do banco
    estimTensao.tensaoNominalReal = 3.2f * nCelulasInt;

    snprintf(tempBuf, sizeof(tempBuf),
             "Banco: %dS | Vmax=%.1fV | Vnom=%.1fV",
             nCelulasInt, estimTensao.tensaoMaxima, estimTensao.tensaoNominalReal);
    adicionarLog(tempBuf);
}

// ── Estima tensão atual da bateria ─────────────────────────
float estimarTensaoBateria() {
    estimTensao.painelAtivo = (sensores.correntePainel > 0.5f);

    float tensao = 0;

    if (estimTensao.painelAtivo) {
        // Painel gerando: tensão de flutuação/carga
        float pct = bateria.calibrada ? bateria.percentualAtual : 80.0f;

        if (pct >= 95.0f) {
            tensao = estimTensao.tensaoMaxima;
        } else {
            float vRepouso = soc2tensaoLiFePO4(pct) *
            estimTensao.fatorBanco *
            estimTensao.fatorCalibTensao;
            float fatorCarga = 0.3f + (pct / 100.0f) * 0.7f;
            tensao = vRepouso + (estimTensao.tensaoMaxima - vRepouso) * fatorCarga;
        }

    } else {
        // Painel parado: tensão de repouso/descarga
        float pct = bateria.calibrada ? bateria.percentualAtual : 50.0f;

        float vCelula = soc2tensaoLiFePO4(pct);
        tensao = vCelula * estimTensao.fatorBanco * estimTensao.fatorCalibTensao;

        // Ajuste por corrente de descarga (queda resistiva)
        float ri = 0.005f * estimTensao.fatorBanco;
        if (sensores.correnteBateria < -0.5f) {
            tensao -= fabsf(sensores.correnteBateria) * ri;
        }
    }

    // Limita entre tensão mínima e máxima
    float vMin = 3.0f  * estimTensao.fatorBanco;
    float vMax = 3.5f * estimTensao.fatorBanco * 1.02f;
    if (tensao < vMin) tensao = vMin;
    if (tensao > vMax) tensao = vMax;

    return tensao;
}

// ── Calibração automática da tensão ────────────────────────
void calibrarTensaoAutomatica() {
    // Condições: painel parado, bateria em repouso, calibrada
    if (sensores.correntePainel   >  0.3f) return;
    if (fabsf(sensores.correnteBateria) > 0.5f) return;
    if (!bateria.calibrada) return;

    float energiaEsperadaPorV = bateria.energiaDisponivel * (bateria.percentualAtual / 100.0f);
    float energiaReal = bateria.energiaAtual;

    if (energiaEsperadaPorV < 1.0f) return;

    float fatorNovo = energiaReal / energiaEsperadaPorV;

    // Limita fator entre 0.85 e 1.15 (±15%)
    if (fatorNovo < 0.85f) fatorNovo = 0.85f;
    if (fatorNovo > 1.15f) fatorNovo = 1.15f;

    // Média móvel ponderada
    uint8_t maxAmostras = 100;
    if (estimTensao.amostrasCalib < maxAmostras) {
        estimTensao.amostrasCalib++;
    }

    float peso = 1.0f / (float)(estimTensao.amostrasCalib + 1);
    estimTensao.fatorCalibTensao = estimTensao.fatorCalibTensao * (1.0f - peso) + fatorNovo * peso;

    calcularFatorBanco();

    // Salva a cada 20 amostras
    if (estimTensao.amostrasCalib % 20 == 0) {
        prefs.begin("rrfs_tv", false);
        prefs.putFloat("fcalib",  estimTensao.fatorCalibTensao);
        prefs.putUChar("namostras", estimTensao.amostrasCalib);
        prefs.end();

        snprintf(tempBuf, sizeof(tempBuf),
                 "Calib tensao: fator=%.4f amostras=%d",
                 estimTensao.fatorCalibTensao, estimTensao.amostrasCalib);
        adicionarLog(tempBuf);
    }
}

// ── Salva calibração de tensão ─────────────────────────────
void salvarCalibTensao() {
    prefs.begin("rrfs_tv", false);
    prefs.putFloat("fcalib",    estimTensao.fatorCalibTensao);
    prefs.putUChar("namostras", estimTensao.amostrasCalib);
    prefs.end();
}

// ── Carrega calibração de tensão ───────────────────────────
void carregarCalibTensao() {
    prefs.begin("rrfs_tv", true);
    estimTensao.fatorCalibTensao = prefs.getFloat("fcalib",    1.0f);
    estimTensao.amostrasCalib    = prefs.getUChar("namostras", 0);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "Calib tensao carregada: fator=%.4f amostras=%d",
             estimTensao.fatorCalibTensao, estimTensao.amostrasCalib);
    adicionarLog(tempBuf);
}

// ============================================================
// SISTEMA: HISTÓRICO DE ENERGIA
// ============================================================

// ── Calcula totais de um dia ───────────────────────────────
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

// ── Salva contadores globais ───────────────────────────────
void salvarContadores() {
    prefs.begin("rrfs_cnt", false);
    prefs.putFloat("p_kwh",  contadores.painelKwh);
    prefs.putFloat("c_kwh",  contadores.casaKwh);
    prefs.putFloat("bc_kwh", contadores.batCargKwh);
    prefs.putFloat("bd_kwh", contadores.batDescKwh);
    prefs.end();
}

// ── Carrega contadores globais ─────────────────────────────
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

// ── Salva um dia específico na flash ───────────────────────
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia) {
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);

    prefs.begin(ns, false);
    prefs.putBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();
}

// ── Carrega um dia específico da flash ─────────────────────
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia) {
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);

    prefs.begin(ns, true);
    size_t len = prefs.getBytes("d", &dia, sizeof(RegistroDia));
    prefs.end();

    return (len == sizeof(RegistroDia) && dia.valido);
}

// ── Salva índices de controle do histórico ─────────────────
void salvarIndiceHistorico() {
    prefs.begin("rrfs_hidx", false);
    prefs.putUChar ("ver",   HIST_KEY_VER);
    prefs.putUShort("idx",   histIndiceAtual);
    prefs.putUShort("total", histTotalDias);
    prefs.end();
}

// ── Salva histórico (dia atual + índices) ──────────────────
void salvarHistorico() {
    salvarDiaFlash(histIndiceAtual, histDiaAtual);
    salvarIndiceHistorico();
}

// ── Carrega histórico (índices + dia atual) ────────────────
void carregarHistorico() {
    prefs.begin("rrfs_hidx", true);
    uint8_t ver = prefs.getUChar("ver", 0);

    if (ver != HIST_KEY_VER) {
        prefs.end();
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

// ── Acumula energia por hora ───────────────────────────────
void acumularEnergia() {
    struct tm ti;
    if (!getLocalTime(&ti)) return;

    unsigned long agora    = millis();
    uint8_t  horaAtual     = ti.tm_hour;
    uint8_t  diaAtual      = ti.tm_mday;
    uint8_t  mesAtual      = ti.tm_mon + 1;
    uint16_t anoAtual      = ti.tm_year + 1900;

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
                if (dTemp.dia == diaAtual && dTemp.mes == mesAtual && dTemp.ano == anoAtual) {
                    histDiaAtual = dTemp;
                    diaExiste    = true;
                    adicionarLog("Hist: dia atual recuperado da flash");
                }
            }
        }

        if (!diaExiste) {
            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano    = anoAtual;
            histDiaAtual.mes    = mesAtual;
            histDiaAtual.dia    = diaAtual;
            histDiaAtual.valido = true;

            bool diaNovoNaFlash = false;
            if (histTotalDias == 0) {
                histIndiceAtual = 0;
                histTotalDias   = 1;
                diaNovoNaFlash  = true;
            } else {
                RegistroDia dTemp;
                if (carregarDiaFlash(histIndiceAtual, dTemp)) {
                    if (dTemp.dia != diaAtual || dTemp.mes != mesAtual || dTemp.ano != anoAtual) {
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
                         diaAtual, mesAtual, anoAtual, histIndiceAtual, histTotalDias);
                adicionarLog(tempBuf);
            }
        }
        return;
    }

    // Acumula potência × tempo
    float deltaH = (float)(agora - acum.ultimoAcum) / 3600000.0f;
    acum.ultimoAcum = agora;

    if (deltaH > 0.1f) deltaH = 0.1f;

    float pp = sensores.potenciaPainel;
    float pc = sensores.potenciaCasa;
    float pb = sensores.potenciaBateria;

    if (pp > 1.0f)  acum.painelWh   += pp * deltaH;
    if (pc > 0.5f)  acum.casaWh     += pc * deltaH;
    if (pb > 1.0f)  acum.batCargWh  += pb        * deltaH;
    if (pb < -1.0f) acum.batDescWh  += fabsf(pb) * deltaH;

    // Detecta mudança de hora ou dia
    bool mudouHora = (horaAtual != acum.horaAtual);
    bool mudouDia  = (diaAtual != acum.diaAtual || mesAtual != acum.mesAtual || anoAtual != acum.anoAtual);

    if (mudouHora || mudouDia) {
        uint8_t horaIdx = acum.horaAtual % HIST_HORAS;

        histDiaAtual.horas[horaIdx].painelWh  += acum.painelWh;
        histDiaAtual.horas[horaIdx].casaWh    += acum.casaWh;
        histDiaAtual.horas[horaIdx].batCargWh += acum.batCargWh;
        histDiaAtual.horas[horaIdx].batDescWh += acum.batDescWh;

        calcularTotaisDia(histDiaAtual);

        contadores.painelKwh  += acum.painelWh  / 1000.0f;
        contadores.casaKwh    += acum.casaWh    / 1000.0f;
        contadores.batCargKwh += acum.batCargWh / 1000.0f;
        contadores.batDescKwh += acum.batDescWh / 1000.0f;

        snprintf(tempBuf, sizeof(tempBuf),
                 "Hist hora %02d: P=%.0fWh C=%.0fWh BC=%.0fWh BD=%.0fWh",
                 acum.horaAtual, acum.painelWh, acum.casaWh, acum.batCargWh, acum.batDescWh);
        adicionarLog(tempBuf);

        if (mudouDia) {
            salvarDiaFlash(histIndiceAtual, histDiaAtual);

            snprintf(tempBuf, sizeof(tempBuf),
                     "Hist dia fechado %02d/%02d/%04d: P=%.0fWh C=%.0fWh BC=%.0fWh BD=%.0fWh",
                     histDiaAtual.dia, histDiaAtual.mes, histDiaAtual.ano,
                     histDiaAtual.totalPainelWh, histDiaAtual.totalCasaWh,
                     histDiaAtual.totalBatCargWh, histDiaAtual.totalBatDescWh);
            adicionarLog(tempBuf);

            histIndiceAtual = (histIndiceAtual + 1) % HIST_DIAS;
            if (histTotalDias < HIST_DIAS) histTotalDias++;

            memset(&histDiaAtual, 0, sizeof(RegistroDia));
            histDiaAtual.ano    = anoAtual;
            histDiaAtual.mes    = mesAtual;
            histDiaAtual.dia    = diaAtual;
            histDiaAtual.valido = true;

            salvarDiaFlash(histIndiceAtual, histDiaAtual);

            snprintf(tempBuf, sizeof(tempBuf),
                     "Hist: novo dia %02d/%02d/%04d idx=%d/%d",
                     diaAtual, mesAtual, anoAtual, histIndiceAtual, HIST_DIAS);
            adicionarLog(tempBuf);
        } else {
            salvarDiaFlash(histIndiceAtual, histDiaAtual);
        }

        salvarIndiceHistorico();
        salvarContadores();

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

    // Salva dia atual na flash a cada 5 minutos
    static unsigned long tSalvaParcial = 0;
    if (agora - tSalvaParcial >= 300000UL) {
        tSalvaParcial = agora;

        uint8_t horaIdxAtual = horaAtual % HIST_HORAS;
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
// SISTEMA: PREVISÃO METEOROLÓGICA
// ============================================================

// ── Salva configuração do local ────────────────────────────
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

// ── Carrega configuração do local ──────────────────────────
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

// ── Calibração automática da previsão ──────────────────────
void calibrarPrevisao() {
    if (!configLocal.configurado) return;
    if (!previsao.valido) return;
    if (sensores.correntePainel < 1.0f) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;
    uint8_t horaAtual = ti.tm_hour;

    // Busca previsão da hora atual
    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (ph.hora == horaAtual && ph.dia == ti.tm_mday &&
            ph.mes == (ti.tm_mon+1) && ph.ano == (ti.tm_year+1900)) {

            if (ph.geracaoEstimada < 10) return;

            float geracaoReal = sensores.potenciaPainel;
            float geracaoEst  = ph.geracaoEstimada;

            if (geracaoEst < 5) return;

            float fatorNovo = geracaoReal / geracaoEst;

            if (fatorNovo < 0.1f) fatorNovo = 0.1f;
            if (fatorNovo > 3.0f) fatorNovo = 3.0f;

            uint8_t maxAmostras = 50;
            if (configLocal.amostrasCalib < maxAmostras) {
                configLocal.amostrasCalib++;
            }

            float peso = 1.0f / (float)configLocal.amostrasCalib;
            configLocal.fatorCalib = configLocal.fatorCalib * (1.0f - peso) + fatorNovo * peso;

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

// ── Calcula geração estimada por hora ──────────────────────
float calcularGeracaoHora(float radiacaoWm2, float radiacaoDifusa,
                          float nuvens, float tempC) {
    if (!configLocal.configurado) return 0;
    if (configLocal.painelKWp <= 0) return 0;

    float radTotal = radiacaoWm2 + radiacaoDifusa * 0.5f;

    if (radTotal < 0) radTotal = 0;

    // Correção por temperatura
    float correcaoTemp = 1.0f;
    if (tempC > 25.0f) {
        correcaoTemp = 1.0f - (tempC - 25.0f) * 0.004f;
        if (correcaoTemp < 0.6f) correcaoTemp = 0.6f;
    }

    float geracaoWh = configLocal.painelKWp * 1000.0f
                    * (radTotal / 1000.0f)
                    * configLocal.eficiencia
                    * correcaoTemp
                    * configLocal.fatorCalib;

    if (geracaoWh < 0) geracaoWh = 0;

    return geracaoWh;
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
    if (buscandoPrev) {
        adicionarLog("Prev: ja em andamento");
        return;
    }

    buscandoPrev = true;
    adicionarLog("Prev: buscando Open-Meteo...");

    // ══════════════════════════════════════════════════════════
    // MONTA URL
    // ══════════════════════════════════════════════════════════
    char url[PREV_URL_MAX];
    snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/forecast"
             "?latitude=%.6f"
             "&longitude=%.6f"
             "&hourly=direct_normal_irradiance"
             ",diffuse_radiation"
             ",cloud_cover"
             ",temperature_2m"
             "&timezone=America%%2FSao_Paulo"
             "&forecast_days=2"
             "&timeformat=unixtime",
             (float)configLocal.latitude,
             (float)configLocal.longitude);

    snprintf(tempBuf, sizeof(tempBuf), "Prev URL: %.80s...", url);
    adicionarLog(tempBuf);

    // ══════════════════════════════════════════════════════════
    // FAZ REQUISIÇÃO HTTP
    // ══════════════════════════════════════════════════════════
    WiFiClient client;
    HTTPClient http;

    http.setReuse(false);
    http.setTimeout(15000);        // 15s timeout
    http.setConnectTimeout(10000); // 10s conexão

    if (!http.begin(client, url)) {
        adicionarLog("Prev ERRO: http.begin() falhou");
        buscandoPrev = false;
        return;
    }

    // Headers para evitar bloqueio por bot
    http.addHeader("User-Agent",
                   "ESP32-RRFS/9.0 (Energy Monitor)");
    http.addHeader("Accept",
                   "application/json");
    http.addHeader("Connection", "close");

    int httpCode = http.GET();

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev HTTP code: %d", httpCode);
    adicionarLog(tempBuf);

    if (httpCode != HTTP_CODE_OK) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "HTTP %d: %s",
                 httpCode,
                 http.errorToString(httpCode).c_str());
        adicionarLog(previsao.erro);
        http.end();
        buscandoPrev = false;
        return;
    }

    // ══════════════════════════════════════════════════════════
    // LÊ RESPOSTA EM STREAM (evita estourar RAM)
    // ══════════════════════════════════════════════════════════
    int contentLen = http.getSize();
    snprintf(tempBuf, sizeof(tempBuf),
             "Prev: tamanho resposta: %d bytes", contentLen);
    adicionarLog(tempBuf);

    // Verifica RAM disponível
    uint32_t ramLivre = esp_get_free_heap_size();
    snprintf(tempBuf, sizeof(tempBuf),
             "Prev: RAM livre: %lu bytes", ramLivre);
    adicionarLog(tempBuf);

    if (ramLivre < 8000) {
        adicionarLog("Prev ERRO: RAM insuficiente!");
        http.end();
        buscandoPrev = false;
        return;
    }

    // Lê payload completo
    String payload = http.getString();
    http.end();

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev: payload lido: %d chars", payload.length());
    adicionarLog(tempBuf);

    if (payload.length() < 200) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "Resposta muito curta: %d chars",
                 payload.length());
        adicionarLog(previsao.erro);
        buscandoPrev = false;
        return;
    }

    // Verifica se tem erro na resposta JSON
    if (payload.indexOf("\"error\"") >= 0 ||
        payload.indexOf("\"reason\"") >= 0) {

        // Extrai mensagem de erro
        int posErr = payload.indexOf("\"reason\":\"");
    if (posErr >= 0) {
        posErr += 10;
        int posFim = payload.indexOf("\"", posErr);
        String errMsg = payload.substring(posErr,
                                          min(posErr + 60, posFim));
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "API erro: %s", errMsg.c_str());
    } else {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "API retornou erro");
    }
    adicionarLog(previsao.erro);
    buscandoPrev = false;
    return;
        }

        // ══════════════════════════════════════════════════════════
        // PARSER JSON MANUAL ROBUSTO
        // ══════════════════════════════════════════════════════════

        // Função helper para extrair array de floats
        auto extrairArrayFloat = [&](const char* chave,
                                     float* arr,
                                     int maxLen) -> int {
                                         String busca = "\"";
                                         busca += chave;
                                         busca += "\":[";

                                         int pos = payload.indexOf(busca);
                                         if (pos < 0) {
                                             snprintf(tempBuf, sizeof(tempBuf),
                                                      "Prev: chave '%s' nao encontrada", chave);
                                             adicionarLog(tempBuf);
                                             return 0;
                                         }
                                         pos += busca.length();

                                         // Verifica se não é null
                                         if (payload.charAt(pos) == 'n') return 0;

                                         int count = 0;
                                         char numBuf[20];
                                         int  numIdx = 0;

                                         while (count < maxLen) {
                                             char ch = payload.charAt(pos);

                                             if (ch == ']') {
                                                 // Fim do array
                                                 if (numIdx > 0) {
                                                     numBuf[numIdx] = '\0';
                                                     arr[count++] = atof(numBuf);
                                                 }
                                                 break;
                                             }
                                             else if (ch == ',') {
                                                 if (numIdx > 0) {
                                                     numBuf[numIdx] = '\0';
                                                     arr[count++]   = atof(numBuf);
                                                     numIdx         = 0;
                                                 }
                                             }
                                             else if (ch == 'n') {
                                                 // null value
                                                 arr[count++] = 0.0f;
                                                 pos += 3; // pula "ull"
                                                 numIdx = 0;
                                             }
                                             else if ((ch >= '0' && ch <= '9') ||
                                                 ch == '.' || ch == '-') {
                                                 if (numIdx < 18) {
                                                     numBuf[numIdx++] = ch;
                                                 }
                                                 }

                                                 pos++;

                                             // Segurança: não passa do fim do payload
                                             if (pos >= (int)payload.length()) break;
                                         }

                                         return count;
                                     };

                                     // Função helper para extrair array de timestamps (long)
                                     auto extrairArrayLong = [&](const char* chave,
                                                                 long* arr,
                                                                 int maxLen) -> int {
                                                                     String busca = "\"";
                                                                     busca += chave;
                                                                     busca += "\":[";

                                                                     int pos = payload.indexOf(busca);
                                                                     if (pos < 0) {
                                                                         snprintf(tempBuf, sizeof(tempBuf),
                                                                                  "Prev: chave '%s' nao encontrada", chave);
                                                                         adicionarLog(tempBuf);
                                                                         return 0;
                                                                     }
                                                                     pos += busca.length();

                                                                     if (payload.charAt(pos) == 'n') return 0;

                                                                     int  count  = 0;
                                                                     char numBuf[16];
                                                                     int  numIdx = 0;

                                                                     while (count < maxLen) {
                                                                         char ch = payload.charAt(pos);

                                                                         if (ch == ']') {
                                                                             if (numIdx > 0) {
                                                                                 numBuf[numIdx] = '\0';
                                                                                 arr[count++]   = atol(numBuf);
                                                                             }
                                                                             break;
                                                                         }
                                                                         else if (ch == ',') {
                                                                             if (numIdx > 0) {
                                                                                 numBuf[numIdx] = '\0';
                                                                                 arr[count++]   = atol(numBuf);
                                                                                 numIdx         = 0;
                                                                             }
                                                                         }
                                                                         else if (ch >= '0' && ch <= '9') {
                                                                             if (numIdx < 14) {
                                                                                 numBuf[numIdx++] = ch;
                                                                             }
                                                                         }

                                                                         pos++;
                                                                         if (pos >= (int)payload.length()) break;
                                                                     }

                                                                     return count;
                                                                 };

                                                                 // ── Arrays temporários ─────────────────────────────────────
                                                                 // Alocados na heap para não estourar a stack
                                                                 float* radDir  = (float*)malloc(PREV_HORAS * sizeof(float));
                                                                 float* radDif  = (float*)malloc(PREV_HORAS * sizeof(float));
                                                                 float* nuvens  = (float*)malloc(PREV_HORAS * sizeof(float));
                                                                 float* temp    = (float*)malloc(PREV_HORAS * sizeof(float));
                                                                 long*  stamps  = (long*) malloc(PREV_HORAS * sizeof(long));

                                                                 // Verifica se malloc funcionou
                                                                 if (!radDir || !radDif || !nuvens || !temp || !stamps) {
                                                                     adicionarLog("Prev ERRO: malloc falhou!");
                                                                     if (radDir) free(radDir);
                                                                     if (radDif) free(radDif);
                                                                     if (nuvens) free(nuvens);
                                                                     if (temp)   free(temp);
                                                                     if (stamps) free(stamps);
                                                                     buscandoPrev = false;
                                                                     return;
                                                                 }

                                                                 // Inicializa arrays
                                                                 memset(radDir, 0, PREV_HORAS * sizeof(float));
                                                                 memset(radDif, 0, PREV_HORAS * sizeof(float));
                                                                 memset(nuvens, 0, PREV_HORAS * sizeof(float));
                                                                 memset(temp,   0, PREV_HORAS * sizeof(float));
                                                                 memset(stamps, 0, PREV_HORAS * sizeof(long));

                                                                 // Extrai os dados
                                                                 int nTs  = extrairArrayLong ("time",
                                                                                              stamps, PREV_HORAS);
                                                                 int nRad = extrairArrayFloat("direct_normal_irradiance",
                                                                                              radDir, PREV_HORAS);
                                                                 int nDif = extrairArrayFloat("diffuse_radiation",
                                                                                              radDif, PREV_HORAS);
                                                                 int nNuv = extrairArrayFloat("cloud_cover",
                                                                                              nuvens, PREV_HORAS);
                                                                 int nTmp = extrairArrayFloat("temperature_2m",
                                                                                              temp,   PREV_HORAS);

                                                                 // Libera payload da RAM imediatamente
                                                                 payload = "";
                                                                 payload.clear();

                                                                 snprintf(tempBuf, sizeof(tempBuf),
                                                                          "Prev parsed: ts=%d rad=%d dif=%d nuv=%d tmp=%d",
                                                                          nTs, nRad, nDif, nNuv, nTmp);
                                                                 adicionarLog(tempBuf);

                                                                 // Verifica se extraiu dados mínimos
                                                                 if (nTs < 1 || nRad < 1) {
                                                                     snprintf(previsao.erro, sizeof(previsao.erro),
                                                                              "Parse falhou: ts=%d rad=%d", nTs, nRad);
                                                                     adicionarLog(previsao.erro);
                                                                     free(radDir); free(radDif);
                                                                     free(nuvens); free(temp); free(stamps);
                                                                     buscandoPrev = false;
                                                                     return;
                                                                 }

                                                                 // ══════════════════════════════════════════════════════════
                                                                 // PROCESSA OS DADOS
                                                                 // ══════════════════════════════════════════════════════════
                                                                 struct tm ti;
                                                                 if (!getLocalTime(&ti)) {
                                                                     adicionarLog("Prev AVISO: sem NTP - usando UTC-3 manual");
                                                                     // Continua mesmo sem NTP confirmado
                                                                     time_t agora = time(nullptr);
                                                                     if (agora < 1000000) {
                                                                         adicionarLog("Prev ERRO: clock nao inicializado");
                                                                         free(radDir); free(radDif);
                                                                         free(nuvens); free(temp); free(stamps);
                                                                         buscandoPrev = false;
                                                                         return;
                                                                     }
                                                                     localtime_r(&agora, &ti);
                                                                 }

                                                                 int total = nTs;
                                                                 if (nRad < total) total = nRad;
                                                                 if (total > PREV_HORAS) total = PREV_HORAS;

                                                                 previsao.totalHoras          = 0;
    previsao.geracaoHojeRestante = 0;
    previsao.geracaoAmanha       = 0;
    previsao.geracaoTotal48h     = 0;

    int hojeAno = ti.tm_year + 1900;
    int hojeMes = ti.tm_mon  + 1;
    int hojeDia = ti.tm_mday;
    int hojeH   = ti.tm_hour;

    // Calcula timestamp de amanhã
    struct tm tiAmanha = ti;
    tiAmanha.tm_mday  += 1;
    tiAmanha.tm_hour   = 0;
    tiAmanha.tm_min    = 0;
    tiAmanha.tm_sec    = 0;
    mktime(&tiAmanha); // normaliza

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev: hoje=%02d/%02d/%04d h=%02d amanha=%02d/%02d",
             hojeDia, hojeMes, hojeAno, hojeH,
             tiAmanha.tm_mday, tiAmanha.tm_mon + 1);
    adicionarLog(tempBuf);

    for (int i = 0; i < total; i++) {
        time_t   ts = (time_t)stamps[i];

        // Converte timestamp para hora local
        struct tm tl;
        localtime_r(&ts, &tl);

        PrevisaoHora& ph    = previsao.horas[i];
        ph.hora             = (uint8_t)tl.tm_hour;
        ph.dia              = (uint8_t)tl.tm_mday;
        ph.mes              = (uint8_t)(tl.tm_mon + 1);
        ph.ano              = (uint16_t)(tl.tm_year + 1900);
        ph.radiacao         = i < nRad ? radDir[i] : 0.0f;
        ph.radiacaoDifusa   = i < nDif ? radDif[i] : 0.0f;
        ph.coberturaNuvens  = i < nNuv ? nuvens[i] : 0.0f;
        ph.temperatura      = i < nTmp ? temp[i]   : 25.0f;
        ph.valido           = true;

        // Garante valores positivos
        if (ph.radiacao      < 0) ph.radiacao      = 0;
        if (ph.radiacaoDifusa< 0) ph.radiacaoDifusa= 0;
        if (ph.coberturaNuvens<0) ph.coberturaNuvens= 0;
        if (ph.coberturaNuvens>100) ph.coberturaNuvens = 100;

        ph.geracaoEstimada = calcularGeracaoHora(
            ph.radiacao,
            ph.radiacaoDifusa,
            ph.coberturaNuvens,
            ph.temperatura
        );

        previsao.geracaoTotal48h += ph.geracaoEstimada;

        // Acumula geração de hoje (horas restantes)
        if (ph.dia  == hojeDia &&
            ph.mes  == hojeMes &&
            ph.ano  == hojeAno &&
            ph.hora >= hojeH) {
            previsao.geracaoHojeRestante += ph.geracaoEstimada;
            }

            // Acumula geração de amanhã
            if (ph.dia == tiAmanha.tm_mday &&
                ph.mes == (uint8_t)(tiAmanha.tm_mon + 1) &&
                ph.ano == (uint16_t)(tiAmanha.tm_year + 1900)) {
                previsao.geracaoAmanha += ph.geracaoEstimada;
                }

                previsao.totalHoras++;
    }

    // Libera memória
    free(radDir);
    free(radDif);
    free(nuvens);
    free(temp);
    free(stamps);

    // ══════════════════════════════════════════════════════════
    // FINALIZA
    // ══════════════════════════════════════════════════════════
    previsao.valido            = true;
    previsao.ultimaAtualizacao = millis();
    ultimaPrev                 = millis();
    memset(previsao.erro, 0, sizeof(previsao.erro));

    snprintf(tempBuf, sizeof(tempBuf),
             "Prev OK! horas=%d hoje=%.0fWh amanha=%.0fWh",
             previsao.totalHoras,
             previsao.geracaoHojeRestante,
             previsao.geracaoAmanha);
    adicionarLog(tempBuf);

    logRAM();

    tomarDecisoes();

    buscandoPrev = false;
}

// ── Consumo médio histórico específico de uma hora do dia ──
// Em vez de usar a média diária genérica (totalCasaWh/24), calcula a média
// real de consumo NAQUELA hora especifica (ex: media do consumo as 8h),
// olhando ate 14 dias de historico. Muito mais preciso para decisoes.
// Se "temDados" for passado, retorna true/false indicando se havia
// historico REAL daquela hora (nao o fallback generico).
float consumoMedioParaHora(uint8_t horaAlvo, bool* temDados) {
    if (horaAlvo > 23) horaAlvo = 23;
    if (temDados) *temDados = false;

    float soma = 0;
    int   dias = 0;
    int   maxDias = min((int)histTotalDias, 14);

    for (int i = 0; i < maxDias; i++) {
        RegistroDia dTemp;
        bool ok;

        if (i == 0) {
            dTemp = histDiaAtual;
            ok    = true;
        } else {
            int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
            ok = carregarDiaFlash(idx, dTemp);
        }

        if (ok && dTemp.valido && dTemp.horas[horaAlvo].casaWh > 0) {
            soma += dTemp.horas[horaAlvo].casaWh;
            dias++;
        }
    }

    if (dias > 0) {
        if (temDados) *temDados = true;
        return soma / dias;
    }

    // Fallback: sem dados especificos dessa hora ainda - usa media diaria geral
    float consumoGeral = 0;
    int   diasGeral = 0;
    int   maxDiasG = min((int)histTotalDias, 7);

    for (int i = 0; i < maxDiasG; i++) {
        RegistroDia dTemp;
        bool ok;
        if (i == 0) { dTemp = histDiaAtual; ok = true; }
        else {
            int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
            ok = carregarDiaFlash(idx, dTemp);
        }
        if (ok && dTemp.valido && dTemp.totalCasaWh > 0) {
            consumoGeral += dTemp.totalCasaWh / 24.0f;
            diasGeral++;
        }
    }

    // temDados continua false aqui - este e so um fallback generico,
    // nao representa um historico confiavel daquela hora especifica.
    return diasGeral > 0 ? (consumoGeral / diasGeral) : 100.0f;
}

// ── Calcula a referencia de consumo (W) a usar na avaliacao de antecipacao ──
// Retorna o consumo instantaneo, OU a media historica daquela hora se o
// instantaneo estiver bem acima dela (indicando pico provavelmente temporario).
float referenciaConsumoAntecip() {
    float consumoInstantW = sensores.correnteCasa * estimTensao.tensaoAtual;

    struct tm ti;
    if (!getLocalTime(&ti)) return consumoInstantW;

    bool  temHist = false;
    float consumoHistW = consumoMedioParaHora((uint8_t)ti.tm_hour, &temHist);

    if (temHist && consumoInstantW > consumoHistW * 1.3f) {
        // Consumo atual esta bem acima do normal para essa hora - o
        // historico indica que isso costuma ser um pico temporario.
        snprintf(tempBuf, sizeof(tempBuf),
                 "ANTECIPACAO: consumo atual %.0fW > historico %.0fW p/ %02dh - "
                 "tratando como pico temporario, usando historico como referencia",
                 consumoInstantW, consumoHistW, ti.tm_hour);
        adicionarLog(tempBuf);
        return consumoHistW;
    }

    return consumoInstantW;
}

// ── Antecipa o religamento da Rede Alta quando a geracao esta favoravel ────
// Mesmo com a bateria ainda abaixo do % configurado em "corteAlta", se a
// geracao atual + tendencia de previsao indicar que vai sobrar energia,
// liga a Rede Alta em modo TESTE, mede o consumo real por 60s, e decide
// manter ligada ou desistir com base no tempo estimado ate a bateria
// chegar no corte-baixo vs. tempo previsto ate a geracao melhorar.
void avaliarAntecipacaoRedeAlta() {

    // Nunca mexe durante diagnostico/curto
    if (curto.estado != CURTO_NORMAL) {
        if (antecipAlta.estado != ANTECIP_IDLE) {
            antecipAlta.estado = ANTECIP_IDLE;
        }
        return;
    }

    if (!bateria.calibrada) return;

    unsigned long agora = millis();
    float pct = bateria.percentualAtual;

    // Se a bateria ja esta acima do limite normal, a regra padrao
    // (controlarEnergia) ja cuida de ligar - nao precisamos antecipar.
    if (pct > redes.corteAlta + 2.0f) {
        antecipAlta.estado = ANTECIP_IDLE;
        return;
    }

    switch (antecipAlta.estado) {

        // ────────────────────────────────────────────────────────
        case ANTECIP_IDLE: {

            // Cooldown apos ultima tentativa mal sucedida
            if (antecipAlta.tCooldown != 0) {
                if (agora < antecipAlta.tCooldown) return;
                antecipAlta.tCooldown = 0;
            }

            if (estados.redesAlta)                        return; // ja ligada
            if (!estados.inversor || !sensores.inversoreAtivo) return;
            if (!previsao.valido)                          return;
            if (pct <= redes.corteBaixa + 5.0f)            return; // margem minima de seguranca
            if (sensores.correntePainel < 0.3f)            return; // precisa ter geracao agora

            // Verifica se a geracao esta em tendencia de ALTA
            // (compara geracao atual com a estimativa da proxima hora)
            struct tm ti;
            if (!getLocalTime(&ti)) return;

            float genAtualW  = sensores.potenciaPainel;
            float genProxima = -1;

            for (uint8_t i = 0; i < previsao.totalHoras; i++) {
                PrevisaoHora& ph = previsao.horas[i];
                if (!ph.valido) continue;
                if (ph.dia == ti.tm_mday && ph.mes == (uint8_t)(ti.tm_mon + 1) &&
                    ph.hora > (uint8_t)ti.tm_hour) {
                    genProxima = ph.geracaoEstimada;
                break;
                    }
            }

            if (genProxima < 0) return;                 // sem dado da proxima hora
            if (genProxima < genAtualW * 0.8f) return;   // geracao nao esta subindo o suficiente

            // Condicoes favoraveis - liga em modo TESTE
            ligarRedeAlta();
            antecipAlta.estado  = ANTECIP_TESTANDO;
            antecipAlta.tInicio = agora;

            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIPACAO: Rede Alta em modo TESTE (bat=%.0f%% gerAtual=%.0fW gerProx=%.0fW)",
                     pct, genAtualW, genProxima);
            adicionarLog(tempBuf);
            break;
        }

        // ────────────────────────────────────────────────────────
        case ANTECIP_TESTANDO: {

            if (!estados.redesAlta) { antecipAlta.estado = ANTECIP_IDLE; return; }

            if (agora - antecipAlta.tInicio < T_ANTECIP_ESTABILIZA) return;

            // Corrente ja estabilizou (60s se passaram) - avalia
            float potBat = sensores.potenciaBateria; // negativo = descarregando

            if (potBat >= -1.0f) {
                // Geracao ja cobre o consumo (nem esta puxando da bateria)
                antecipAlta.estado      = ANTECIP_MANTIDO;
                antecipAlta.tUltimaAval = agora;
                adicionarLog("ANTECIPACAO: Rede Alta MANTIDA - geracao ja cobre o consumo");
                return;
            }

            float energiaAteCorte = bateria.energiaAtual -
            (bateria.energiaDisponivel * (redes.corteBaixa / 100.0f));
            float horasAteCorte = (energiaAteCorte <= 0) ? 0.0f :
            (energiaAteCorte / fabsf(potBat));

            float refConsumoW     = referenciaConsumoAntecip();
            float horasAteMelhora = horasAteGeracaoSuperar(refConsumoW);
            float margemH         = MARGEM_ANTECIP_MIN / 60.0f;

            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIPACAO teste: Pbat=%.0fW ref=%.0fW horas_ate_corte=%.2fh horas_ate_melhora=%.2fh",
                     potBat, refConsumoW, horasAteCorte, horasAteMelhora);
            adicionarLog(tempBuf);

            bool favoravel = (horasAteMelhora >= 0) &&
            (horasAteCorte > horasAteMelhora + margemH);

            if (favoravel) {
                antecipAlta.estado      = ANTECIP_MANTIDO;
                antecipAlta.tUltimaAval = agora;
                adicionarLog("ANTECIPACAO: Rede Alta MANTIDA - geracao deve melhorar a tempo");
            } else {
                desligarRedeAlta();
                antecipAlta.estado    = ANTECIP_IDLE;
                antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
                adicionarLog("ANTECIPACAO: Rede Alta desligada de volta - condicao nao favoravel");
            }
            break;
        }

        // ────────────────────────────────────────────────────────
        case ANTECIP_MANTIDO: {

            if (!estados.redesAlta) { antecipAlta.estado = ANTECIP_IDLE; return; }
            if (agora - antecipAlta.tUltimaAval < T_ANTECIP_REAVAL) return;
            antecipAlta.tUltimaAval = agora;

            float potBat = sensores.potenciaBateria;
            if (potBat >= -1.0f) return; // continua cobrindo o consumo - tudo certo

            float energiaAteCorte = bateria.energiaAtual -
            (bateria.energiaDisponivel * (redes.corteBaixa / 100.0f));
            float horasAteCorte = (energiaAteCorte <= 0) ? 0.0f :
            (energiaAteCorte / fabsf(potBat));

            float refConsumoW     = referenciaConsumoAntecip();
            float horasAteMelhora = horasAteGeracaoSuperar(refConsumoW);
            float margemH         = MARGEM_ANTECIP_MIN / 60.0f;

            snprintf(tempBuf, sizeof(tempBuf),
                     "ANTECIPACAO reaval: Pbat=%.0fW ref=%.0fW horas_ate_corte=%.2fh horas_ate_melhora=%.2fh",
                     potBat, refConsumoW, horasAteCorte, horasAteMelhora);
            adicionarLog(tempBuf);

            bool favoravel = (horasAteMelhora >= 0) &&
            (horasAteCorte > horasAteMelhora + margemH);

            if (!favoravel) {
                desligarRedeAlta();
                antecipAlta.estado    = ANTECIP_IDLE;
                antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;
                adicionarLog("ANTECIPACAO: Rede Alta desligada - reavaliacao ficou desfavoravel");
            }
            break;
        }
    }
}

// ── Quantas horas faltam para a geracao prevista superar refWatts ──
// Varre a previsao (ate 48h) procurando a primeira hora FUTURA cuja geracao
// estimada supera refWatts. Retorna -1 se nao encontrar dentro da janela.
float horasAteGeracaoSuperar(float refWatts) {
    if (!previsao.valido) return -1;

    time_t tsAgora = time(nullptr);
    if (tsAgora < 1000000) return -1; // relogio nao inicializado

    for (uint8_t i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (!ph.valido) continue;

        struct tm tCompara = {0};
        tCompara.tm_year  = ph.ano - 1900;
        tCompara.tm_mon   = ph.mes - 1;
        tCompara.tm_mday  = ph.dia;
        tCompara.tm_hour  = ph.hora;
        tCompara.tm_min   = 0;
        tCompara.tm_sec   = 0;
        tCompara.tm_isdst = -1;

        time_t tsHora = mktime(&tCompara);
        if (tsHora <= tsAgora) continue; // so horas futuras

        if (ph.geracaoEstimada > refWatts) {
            return (float)(tsHora - tsAgora) / 3600.0f;
        }
    }
    return -1;
}

// ── Toma decisões automáticas baseadas na previsão ─────────
void tomarDecisoes() {
    if (!previsao.valido || !bateria.calibrada) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;

    int horaAtual = ti.tm_hour;
    bool eNoite = (horaAtual >= 18 || horaAtual < 6);
    bool eDia   = !eNoite;

    // ══════════════════════════════════════════════════════════
    // CONSUMO MEDIO GERAL (usado nos calculos noturnos, como antes)
    // ══════════════════════════════════════════════════════════
    float consumoMedioHora = 0;
    int   diasValidos = 0;

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
        consumoMedioHora = 100.0f;
    }

    float energiaAtual = bateria.energiaAtual;
    float energiaMin   = bateria.energiaDisponivel * (redes.corteBaixa / 100.0f);

    // ══════════════════════════════════════════════════════════
    // BLOCO NOTURNO (igual ao original: decide se desliga rede alta
    // e lamp externa pensando na noite inteira ate o amanhecer)
    // ══════════════════════════════════════════════════════════
    float horasAteAmanhecer = 0;

    if (eNoite) {
        if (horaAtual >= 18) {
            horasAteAmanhecer = (24 - horaAtual) + 6;
        } else {
            horasAteAmanhecer = 6 - horaAtual;
        }
    }

    float horaInicioGeracao = 6;
    for (int i = 0; i < previsao.totalHoras; i++) {
        PrevisaoHora& ph = previsao.horas[i];
        if (ph.geracaoEstimada > 50 &&
            (ph.dia == ti.tm_mday || ph.dia == ti.tm_mday + 1)) {
            horaInicioGeracao = ph.hora;
        break;
            }
    }

    if (eNoite) {
        if (horaAtual >= 18) {
            horasAteAmanhecer = (24 - horaAtual) + horaInicioGeracao;
        } else {
            horasAteAmanhecer = horaInicioGeracao - horaAtual;
        }
    }

    float energiaNecessariaNoite = consumoMedioHora * horasAteAmanhecer;
    float energiaDisponivelNoite = energiaAtual - energiaMin;
    float saldoEnergiaNoite      = energiaDisponivelNoite - energiaNecessariaNoite;

    float consumoRedeAltaHora = consumoMedioHora * 0.40f;
    float consumoLampExtHora  = consumoMedioHora * 0.05f;

    decisao.desligarRedeAltaNoite = false;
    decisao.horaDesligarRedeAlta  = 0;
    decisao.horaLigarRedeAlta     = 0;
    decisao.desligarLampExtNoite  = false;
    decisao.alertaEnergiaInsuficiente = false;

    if (eNoite) {
        if (saldoEnergiaNoite < 0) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Energia insuf. p/ noite: disp=%.0fWh nec=%.0fWh",
                     energiaDisponivelNoite, energiaNecessariaNoite);
        } else if (saldoEnergiaNoite < consumoRedeAltaHora * 3) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Margem baixa: saldo=%.0fWh <3h rede alta",
                     saldoEnergiaNoite);
        }

        decisao.horaLigarRedeAlta = horaInicioGeracao + 1;

        if (sensores.correntePainel <= 0.5f) {
            if (saldoEnergiaNoite < consumoLampExtHora * horasAteAmanhecer) {
                decisao.desligarLampExtNoite = true;
            }
        }

        if (saldoEnergiaNoite < 0 &&
            previsao.geracaoAmanha < consumoMedioHora * 12) {
            decisao.alertaEnergiaInsuficiente = true;
            }

            if (decisao.desligarRedeAltaNoite && estados.redesAlta &&
                curto.estado == CURTO_NORMAL) {
                desligarRedeAlta();
            antecipAlta.estado    = ANTECIP_IDLE;
        antecipAlta.tCooldown = millis() + T_ANTECIP_COOLDOWN;
        adicionarLog("DECISAO NOITE: Rede Alta OFF - economia noturna");
        adicionarLog(decisao.motivo);
                }

                if (decisao.desligarLampExtNoite && estados.lampExt) {
                    desligarLampExt();
                    adicionarLog("DECISAO NOITE: Lamp.Ext OFF - economia energia noite");
                }

                snprintf(tempBuf, sizeof(tempBuf),
                         "Decisao noite: saldo=%.0fWh cons_noite=%.0fWh horas=%.1fh",
                         saldoEnergiaNoite, energiaNecessariaNoite, horasAteAmanhecer);
                adicionarLog(tempBuf);
    }

    // ══════════════════════════════════════════════════════════
    // BLOCO DIURNO (NOVO) - avalia se a energia da bateria + a
    // geracao PREVISTA restante do dia da conta do consumo restante
    // do dia (hora a hora, usando media historica especifica de
    // cada hora). Se houver deficit projetado, o sistema toma
    // decisoes tambem de dia, nao so a noite.
    // ══════════════════════════════════════════════════════════
    decisao.desligarLampIntDia = false;

    if (eDia && previsao.geracaoHojeRestante >= 0) {

        float consumoRestanteDia = 0;
        for (int h = horaAtual; h < 24; h++) {
            consumoRestanteDia += consumoMedioParaHora((uint8_t)h);
        }

        // Reserva de seguranca: nao pode encostar no corte-baixo
        float reservaMinima = bateria.energiaDisponivel * (redes.corteBaixa / 100.0f);

        float saldoDia = (energiaAtual + previsao.geracaoHojeRestante) -
        (consumoRestanteDia + reservaMinima);

        snprintf(tempBuf, sizeof(tempBuf),
                 "Decisao dia: bat=%.0fWh gerRestHoje=%.0fWh consumoRestDia=%.0fWh saldo=%.0fWh",
                 energiaAtual, previsao.geracaoHojeRestante,
                 consumoRestanteDia, saldoDia);
        adicionarLog(tempBuf);

        if (saldoDia < 0) {

            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Deficit diurno projetado: saldo=%.0fWh", saldoDia);

            // Prioridade 1: desliga Rede Alta (maior carga) se estiver ligada
            // e nao estiver sendo mantida de proposito pela antecipacao.
            if (estados.redesAlta && curto.estado == CURTO_NORMAL &&
                antecipAlta.estado == ANTECIP_IDLE) {
                desligarRedeAlta();
            adicionarLog("DECISAO DIA: Rede Alta OFF - deficit projetado para o resto do dia");
            adicionarLog(decisao.motivo);
                }
                // Prioridade 2 (situacao ja critica, perto do corte de lamp interna):
                // desliga a lampada interna tambem, mesmo de dia.
                else if (bateria.percentualAtual <= redes.corteLampInt + 10.0f) {
                    decisao.desligarLampIntDia = true;
                }
        }
    }

    if (decisao.desligarLampIntDia && estados.lampInt) {
        desligarLampInt();
        adicionarLog("DECISAO DIA: Lamp.Interna OFF - deficit critico projetado, energia curta");
        adicionarLog(decisao.motivo);
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
        sensores.inversoreAtivo) {
        deveReligar = true;
    }

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
// SISTEMA: CONTROLE AUTOMÁTICO DE ENERGIA
// ============================================================

// ── Processa a restauração de energia de forma NÃO BLOQUEANTE ──
void processarRestauroEnergia() {
    if (restauro.estado == RESTORE_OCIOSO) return;

    unsigned long agora = millis();

    switch (restauro.estado) {

        case RESTORE_LIGANDO_BAIXA:
            if (!estados.redeBaixa) {
                ligarRedeBaixa();
                adicionarLog("Rede Baixa ON: energia recuperada");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_AGUARDANDO_BAIXA;
            break;

        case RESTORE_AGUARDANDO_BAIXA:
            if (agora - restauro.tInicio >= RESTORE_T_BAIXA) {
                restauro.estado = RESTORE_LIGANDO_INVERSOR;
            }
            break;

        case RESTORE_LIGANDO_INVERSOR:
            if (!estados.inversor) {
                ligarInversor();
                adicionarLog("Inversor ON: energia recuperada");
            }
            restauro.tInicio = agora;
            restauro.estado  = RESTORE_AGUARDANDO_INVERSOR_BOOT;
            break;

        case RESTORE_AGUARDANDO_INVERSOR_BOOT:
            // Equivale ao antigo delay(5000) - aguarda boot do inversor
            if (agora - restauro.tInicio >= RESTORE_T_INV_BOOT) {
                restauro.tInicio = agora;
                restauro.estado  = RESTORE_CHECANDO_STATUS_INV;
            }
            break;

        case RESTORE_CHECANDO_STATUS_INV:
            // Equivale ao antigo while(3000ms){ if(lerStatusInversor())break; delay(200); }
            if (lerStatusInversor()) {
                restauro.estado = RESTORE_LIGANDO_ALTA;
            } else if (agora - restauro.tInicio >= RESTORE_T_INV_STATUS) {
                // Timeout sem status - segue mesmo assim (comportamento original)
                restauro.estado = RESTORE_LIGANDO_ALTA;
            }
            // Não faz polling mais rápido que RESTORE_T_INV_POLL para não
            // sobrecarregar leitura ADC
            break;

        case RESTORE_LIGANDO_ALTA:
            if (!estados.redesAlta &&
                bateria.percentualAtual > redes.corteAlta + 2.0f &&
                sensores.inversoreAtivo) {
                ligarRedeAlta();
            adicionarLog("Rede Alta ON: energia recuperada");
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
                adicionarLog("Lamp.Interna ON: energia recuperada");
                    }
                    restauro.estado = RESTORE_CONCLUIDO;
            }
            break;

        case RESTORE_CONCLUIDO:
            restauro.estado = RESTORE_OCIOSO;
            break;

        default:
            restauro.estado = RESTORE_OCIOSO;
            break;
    }
}

// ── Controla cargas baseado no nível da bateria ────────────
void controlarEnergia() {
    if (!bateria.calibrada) return;

    float pct  = bateria.percentualAtual;
    unsigned long agora = millis();

    // ══════════════════════════════════════════════════════════
    // LÂMPADA EXTERNA - CONTROLE POR HORÁRIO SOLAR
    // ══════════════════════════════════════════════════════════

    bool ehDia = false;

    if (configLocal.configurado) {
        struct tm ti;
        if (getLocalTime(&ti)) {
            int hora = ti.tm_hour;
            int mes  = ti.tm_mon + 1;

            float latRad  = (float)(configLocal.latitude * 3.14159 / 180.0);
            float declin  = -23.45f * cosf(
                2.0f * 3.14159f * (float)(mes + 9.5f) / 12.0f
            );
            float declRad = declin * 3.14159f / 180.0f;
            float cosH = -tanf(latRad) * tanf(declRad);

            float horasLuz = 12.0f;
            if (cosH >= -1.0f && cosH <= 1.0f) {
                horasLuz = (2.0f / 15.0f) *
                (acosf(cosH) * 180.0f / 3.14159f);
            }

            float nascerSol = 12.0f - (horasLuz / 2.0f);
            float porSol    = 12.0f + (horasLuz / 2.0f);

            float horaAtual = (float)hora + (float)ti.tm_min / 60.0f;
            ehDia = (horaAtual >= (nascerSol + 0.5f) &&
            horaAtual <  (porSol    - 0.5f));

            static bool ultimoEhDia = false;
            if (ehDia != ultimoEhDia) {
                ultimoEhDia = ehDia;
                char bufSol[100];
                snprintf(bufSol, sizeof(bufSol),
                         "Solar: %s | Nascer=%.2fh Por=%.2fh Luz=%.1fh",
                         ehDia ? "DIA" : "NOITE",
                         nascerSol, porSol, horasLuz);
                adicionarLog(bufSol);
            }
        }
    } else {
        static bool diaFallback = false;
        if (!diaFallback && sensores.correntePainel > 1.0f) {
            diaFallback = true;
        } else if (diaFallback && sensores.correntePainel < 0.3f) {
            diaFallback = false;
        }
        ehDia = diaFallback;
    }

    // Controle da lâmpada externa com buffer de 60s
    {
        static unsigned long tMudancaLampExt = 0;
        static bool          ehDiaConfirmado = false;
        static bool          primeiraVez = true;

        if (primeiraVez) {
            primeiraVez      = false;
            ehDiaConfirmado  = ehDia;
            tMudancaLampExt  = 0;
        }

        if (ehDia != ehDiaConfirmado) {
            if (tMudancaLampExt == 0) {
                tMudancaLampExt = agora;
            } else if (agora - tMudancaLampExt >= 60000UL) {
                ehDiaConfirmado = ehDia;
                tMudancaLampExt = 0;

                if (ehDiaConfirmado) {
                    if (estados.lampExt) {
                        desligarLampExt();
                        adicionarLog("Lamp.Externa OFF: amanheceu (horario solar)");
                    }
                } else {
                    if (!estados.lampExt &&
                        pct > redes.corteLampExt + 2.0f &&
                        estados.inversor &&
                        sensores.inversoreAtivo) {
                        ligarLampExt();
                    adicionarLog("Lamp.Externa ON: anoiteceu (horario solar)");
                        }
                }
            }
        } else {
            tMudancaLampExt = 0;

            if (ehDiaConfirmado && estados.lampExt) {
                desligarLampExt();
                adicionarLog("Lamp.Externa OFF: correcao estado (dia)");
            } else if (!ehDiaConfirmado &&
                !estados.lampExt &&
                pct > redes.corteLampExt + 2.0f &&
                estados.inversor &&
                sensores.inversoreAtivo) {
                ligarLampExt();
            adicionarLog("Lamp.Externa ON: correcao estado (noite)");
                }
        }
    }

    // ══════════════════════════════════════════════════════════
    // LÂMPADA INTERNA
    // ══════════════════════════════════════════════════════════

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
            if (estados.inversor && sensores.inversoreAtivo) {
                ligarLampInt();
                adicionarLog("Lamp.Interna ON: energia recuperada");
            }
        }
    }

    // ══════════════════════════════════════════════════════════
    // LÂMPADA EXTERNA - CORTE POR ENERGIA BAIXA
    // ══════════════════════════════════════════════════════════

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

    // ══════════════════════════════════════════════════════════
    // REDE ALTA
    // ══════════════════════════════════════════════════════════

    // Enquanto a antecipacao inteligente decidiu manter a Rede Alta ligada
    // de proposito (TESTANDO ou MANTIDO), a regra automatica de corte por
    // percentual nao deve derrubar - quem decide desligar nessa situacao
    // e a propria avaliarAntecipacaoRedeAlta().
    bool antecipAtiva = (antecipAlta.estado != ANTECIP_IDLE);

    if (pct <= redes.corteAlta && estados.redesAlta && !antecipAtiva) {
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

    // ══════════════════════════════════════════════════════════
    // REDE BAIXA + INVERSOR
    // AGORA NÃO BLOQUEANTE: apenas dispara a máquina de estados.
    // O processamento real ocorre em processarRestauroEnergia(),
    // chamada a cada iteração do loop() - a interface web e o
    // WiFi continuam respondendo durante toda a sequência.
    // ══════════════════════════════════════════════════════════

    if (pct <= redes.corteBaixa &&
        (estados.redeBaixa || estados.inversor)) {

        // Cancela qualquer restauração em andamento - prioridade é desligar
        restauro.estado = RESTORE_OCIOSO;

    // Cancela tambem qualquer antecipacao em andamento - se a bateria
    // chegou no corte baixo, nao ha mais sentido em manter a Rede Alta
    // ligada por antecipacao.
    antecipAlta.estado    = ANTECIP_IDLE;
    antecipAlta.tCooldown = agora + T_ANTECIP_COOLDOWN;

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
                bool precisaRestaurar = (!estados.redeBaixa || !estados.inversor);

                if (precisaRestaurar && restauro.estado == RESTORE_OCIOSO) {
                    restauro.estado = RESTORE_LIGANDO_BAIXA;
                    adicionarLog("Iniciando restauracao de energia (nao-bloqueante)");
                }
            }
        }
}

// ============================================================
// SISTEMA: PROTEÇÃO CONTRA CURTO-CIRCUITO
// ============================================================

// ── Helper: desliga inversor e limpa buffer ────────────────
void _desligarInversorComBuffer() {
    setRele(PIN_RELE_INV_ALTA, false);
    estados.inversor          = false;
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
}

// ── Helper: liga inversor e limpa buffer ───────────────────
void _ligarInversorComBuffer() {
    timers.inversorEstadoReal = false;
    timers.inversorUltimoHigh = 0;
    setRele(PIN_RELE_INV_ALTA, true);
    estados.inversor = true;
}

// ── Monitora inversor e diagnostica curtos ─────────────────
void monitorarInversor() {
    unsigned long agora = millis();

    bool emProcesso = (curto.estado == CURTO_RELIGANDO      ||
                       curto.estado == CURTO_TESTANDO_ALTA  ||
                       curto.estado == CURTO_TESTANDO_BAIXA);

    if (!estados.inversor && !emProcesso) {
        if (curto.estado == CURTO_NORMAL) {
            curto.monitorando       = false;
            curto.tInicio           = 0;
            curto.tentativasReligar = 0;
        }
        return;
    }

    // ── ESTADO NORMAL: monitora status ───────────────────────
    if (curto.estado == CURTO_NORMAL) {

        bool statusAtual = lerStatusInversor();

        if (statusAtual) {
            curto.tInicio           = 0;
            curto.monitorando       = true;
            curto.tentativasReligar = 0;
            return;
        }

        if (curto.monitorando) {

            if (curto.tInicio == 0) {
                curto.tInicio = agora;
                adicionarLog("Inversor: status perdido - aguardando 1s...");
                return;
            }

            if (agora - curto.tInicio < T_STATUS_AUSENTE) {
                return;
            }

            adicionarLog("Inversor: sem status apos 1s - proteção ativa!");
            adicionarLog("Desligando chave - aguardando 2s para religar...");

            _desligarInversorComBuffer();

            curto.estado    = CURTO_RELIGANDO;
            curto.subEstado = SUB_INVERSOR_DESLIGADO;
            curto.tInicio   = agora;
        }
        return;
    }

    // ── ESTADO: RELIGANDO ─────────────────────────────────────
    if (curto.estado == CURTO_RELIGANDO) {

        switch (curto.subEstado) {

            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_PAUSA_RELIGAR) {
                    return;
                }

                snprintf(tempBuf, sizeof(tempBuf),
                         "2s passados - religando (tentativa %d/%d)...",
                         curto.tentativasReligar + 1, MAX_TENTATIVAS_RELIGAR);
                adicionarLog(tempBuf);

                _ligarInversorComBuffer();

                curto.tInicio   = agora;
                curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("Aguardando status do inversor por 3s...");
                break;

            case SUB_AGUARDANDO_STATUS: {

                bool statusAgora = lerStatusInversor();

                if (statusAgora) {
                    unsigned long tempoComStatus = agora - timers.inversorUltimoHigh;

                    if (timers.inversorEstadoReal && tempoComStatus < 500) {
                        return;
                    }

                    if (timers.inversorEstadoReal) {
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

                if (agora - curto.tInicio >= T_AGUARDA_STATUS) {

                    curto.tentativasReligar++;
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Status nao voltou (tentativa %d/%d)",
                             curto.tentativasReligar, MAX_TENTATIVAS_RELIGAR);
                    adicionarLog(tempBuf);

                    _desligarInversorComBuffer();
                    curto.tInicio = agora;

                    if (curto.tentativasReligar < MAX_TENTATIVAS_RELIGAR) {
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        snprintf(tempBuf, sizeof(tempBuf),
                                 "Nova tentativa em 2s (%d/%d)...",
                                 curto.tentativasReligar + 1,
                                 MAX_TENTATIVAS_RELIGAR);
                        adicionarLog(tempBuf);
                    } else {
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

    // ── DIAGNÓSTICO - TESTANDO REDE ALTA ─────────────────────
    if (curto.estado == CURTO_TESTANDO_ALTA) {

        switch (curto.subEstado) {

            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_AGUARDA_QUEDA) return;

                desligarRedeAlta();
                delay(300);

                _ligarInversorComBuffer();
                curto.tInicio   = agora;
                curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("Teste Alta: inversor ligado SEM rede alta");
                break;

            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("Teste Alta: inversor OK sem rede alta - confirmando...");
                }
                else if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
                    _desligarInversorComBuffer();
                    curto.estado    = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio   = agora;
                    adicionarLog("Teste Alta: inversor nao respondeu - testando rede BAIXA...");
                }
                break;

            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        _desligarInversorComBuffer();
                        curto.estado    = CURTO_TESTANDO_BAIXA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        curto.tInicio   = agora;
                        adicionarLog("Teste Alta: inversor instavel - testando rede BAIXA...");
                    }
                    return;
                }

                adicionarLog("Teste Alta: 3s estavel - ligando rede alta para confirmar...");
                ligarRedeAlta();
                curto.tInicio   = agora;
                curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                break;

            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor() && agora - curto.tInicio > 500) {

                        adicionarLog("Teste Alta: inversor caiu com rede alta!");
                        adicionarLog("=== CURTO CONFIRMADO: REDE ALTA ===");

                        desligarRedeAlta();
                        delay(200);

                        _desligarInversorComBuffer();
                        delay(500);
                        _ligarInversorComBuffer();

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

                if (lerStatusInversor()) {
                    curto.estado            = CURTO_NORMAL;
                    curto.subEstado         = SUB_IDLE;
                    curto.tInicio           = 0;
                    curto.monitorando       = true;
                    curto.tentativasReligar = 0;
                    adicionarLog("Teste Alta: sistema OK - era pico no diagnostico");
                } else {
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

    // ── DIAGNÓSTICO - TESTANDO REDE BAIXA ────────────────────
    if (curto.estado == CURTO_TESTANDO_BAIXA) {

        switch (curto.subEstado) {

            case SUB_INVERSOR_DESLIGADO:
                if (agora - curto.tInicio < T_AGUARDA_QUEDA) return;

                desligarRedeAlta();
                desligarRedeBaixa();
                delay(300);

                _ligarInversorComBuffer();
                curto.tInicio   = agora;
                curto.subEstado = SUB_AGUARDANDO_STATUS;
                adicionarLog("Teste Baixa: inversor ligado SEM nenhuma rede");
                break;

            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("Teste Baixa: inversor OK sem cargas - confirmando...");
                }
                else if (agora - curto.tInicio >= T_AGUARDA_STATUS) {
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

            case SUB_INVERSOR_OK_ALTA_OFF:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor()) {
                        _desligarInversorComBuffer();
                        curto.estado    = CURTO_AGUARDANDO_RESET;
                        curto.subEstado = SUB_IDLE;
                        curto.tInicio   = 0;
                        adicionarLog("=== INVERSOR INSTAVEL SEM CARGA ===");
                        adicionarLog("Verifique o equipamento fisicamente");
                    }
                    return;
                }

                adicionarLog("Teste Baixa: 3s estavel - ligando rede baixa para confirmar...");
                ligarRedeBaixa();
                curto.tInicio   = agora;
                curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                break;

            case SUB_INVERSOR_OK_LIGAR_ALTA:
                if (agora - curto.tInicio < T_AGUARDA_STATUS) {
                    if (!lerStatusInversor() && agora - curto.tInicio > 500) {

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

                if (lerStatusInversor()) {
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

    // ── ESTADOS DE PROTEÇÃO PERMANENTE ────────────────────────
    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        if (estados.redesAlta) desligarRedeAlta();
        return;
    }

    if (curto.estado == CURTO_CONFIRMADO_BAIXA ||
        curto.estado == CURTO_AGUARDANDO_RESET) {
        if (estados.inversor)  _desligarInversorComBuffer();
        if (estados.redeBaixa) desligarRedeBaixa();
        if (estados.redesAlta) desligarRedeAlta();
        return;
    }
}

// ── Reset de curto via web (rede alta) ─────────────────────
void resetCurtoAlta() {
    if (curto.estado != CURTO_CONFIRMADO_ALTA) return;

    adicionarLog("Reset curto rede alta solicitado");

    desligarRedeAlta();
    delay(500);

    ligarRedeAlta();
    delay(300);

    if (lerStatusInversor()) {
        curto.estado         = CURTO_NORMAL;
        curto.redeAltaCurto  = false;
        curto.subEstado      = SUB_IDLE;
        curto.tInicio        = 0;
        curto.monitorando    = true;
        adicionarLog("Curto rede alta RESOLVIDO - sistema normal");
    } else {
        desligarRedeAlta();
        adicionarLog("AVISO: Curto rede alta NAO resolvido!");
        adicionarLog("Verifique os equipamentos e tente novamente.");
    }
}

// ── Reset de curto via web (rede baixa) ────────────────────
void resetCurtoBaixa() {
    if (curto.estado != CURTO_CONFIRMADO_BAIXA &&
        curto.estado != CURTO_AGUARDANDO_RESET) return;

    adicionarLog("Reset curto rede baixa solicitado");

    desligarRedeBaixa();
    desligarInversor();
    delay(500);

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

    ligarRedeBaixa();
    delay(500);

    if (lerStatusInversor()) {
        curto.estado          = CURTO_NORMAL;
        curto.redeBaixaCurto  = false;
        curto.subEstado       = SUB_IDLE;
        curto.tInicio         = 0;
        curto.monitorando     = true;
        adicionarLog("Curto rede baixa RESOLVIDO - sistema normal");
    } else {
        desligarRedeBaixa();
        desligarInversor();
        curto.estado = CURTO_CONFIRMADO_BAIXA;
        adicionarLog("AVISO: Curto rede baixa NAO resolvido!");
        adicionarLog("Verifique os equipamentos e tente novamente.");
    }
}

// ── HTML status do curto para página web ───────────────────
String htmlStatusCurto() {
    String h = "";

    if (curto.estado == CURTO_NORMAL && !curto.redeAltaCurto && !curto.redeBaixaCurto) {
        h += "<div class='ok' style='font-size:12px'>";
        h += "&#10003; Sistema sem curto detectado</div>";
        return h;
    }

    if (curto.estado == CURTO_TESTANDO_ALTA || curto.estado == CURTO_TESTANDO_BAIXA) {
        h += "<div class='wn'>";
        h += "&#9874; Diagnosticando curto... Aguarde.</div>";
        return h;
    }

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
// SISTEMA: PREFERENCES (SALVAR/CARREGAR CONFIGURAÇÕES)
// ============================================================

// ── Verifica se existem credenciais ────────────────────────
bool existeCredenciais() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("user");
    prefs.end();
    return ok;
}

// ── Verifica se existe configuração WiFi ───────────────────
bool existeWiFiConfig() {
    prefs.begin("rrfs", true);
    bool ok = prefs.isKey("wifi_ok");
    prefs.end();
    return ok;
}

// ── Salva credenciais de login ─────────────────────────────
void salvarCredenciais(const String& u, const String& p) {
    prefs.begin("rrfs", false);
    prefs.putString("user", u);
    prefs.putString("pass", p);
    prefs.end();
}

// ── Verifica login ─────────────────────────────────────────
bool verificarLogin(const String& u, const String& p) {
    prefs.begin("rrfs", true);
    String su = prefs.getString("user", "");
    String sp = prefs.getString("pass", "");
    prefs.end();
    return (su == u && sp == p && su.length() > 0);
}

// ── Salva configuração WiFi ────────────────────────────────
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

// ── Carrega configuração WiFi ──────────────────────────────
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

// ── Salva configuração da bateria ──────────────────────────
void salvarConfigBateria() {
    prefs.begin("rrfs", false);
    prefs.putFloat("bat_vnom",  bateria.tensaoNominal);
    prefs.putFloat("bat_ah",    bateria.capacidadeAh);
    prefs.putFloat("bat_desc",  bateria.descargaSegura);
    prefs.putFloat("bat_nrg",   bateria.energiaAtual);
    prefs.putBool ("bat_calib", bateria.calibrada);
    prefs.end();
}

// ── Carrega configuração da bateria ────────────────────────
void carregarConfigBateria() {
    prefs.begin("rrfs", true);
    bateria.tensaoNominal  = prefs.getFloat("bat_vnom",  12.8f);
    bateria.capacidadeAh   = prefs.getFloat("bat_ah",    100.0f);
    bateria.descargaSegura = prefs.getFloat("bat_desc",  80.0f);
    bateria.energiaAtual   = prefs.getFloat("bat_nrg",   0.0f);
    bateria.calibrada      = prefs.getBool ("bat_calib", false);
    prefs.end();
    calcularBateria();

    if (bateria.energiaDisponivel > 0)
        bateria.percentualAtual = (bateria.energiaAtual / bateria.energiaDisponivel) * 100.0f;
}

// ── Salva configuração das redes ───────────────────────────
void salvarConfigRedes() {
    prefs.begin("rrfs", false);
    prefs.putFloat("r_baixa",   redes.corteBaixa);
    prefs.putFloat("r_alta",    redes.corteAlta);
    prefs.putFloat("r_lmpext",  redes.corteLampExt);
    prefs.putFloat("r_lmpint",  redes.corteLampInt);
    prefs.end();
}

// ── Carrega configuração das redes ─────────────────────────
void carregarConfigRedes() {
    prefs.begin("rrfs", true);
    redes.corteBaixa   = prefs.getFloat("r_baixa",  10.0f);
    redes.corteAlta    = prefs.getFloat("r_alta",   30.0f);
    redes.corteLampExt = prefs.getFloat("r_lmpext", 50.0f);
    redes.corteLampInt = prefs.getFloat("r_lmpint",  0.0f);
    prefs.end();
}

// ── Carrega calibração dos sensores ────────────────────────
void carregarCalibSensores() {
    // PAINEL: ACS758 50A Bidirecional
    // Offset inicial = 2500mV (será refinado automaticamente à noite)
    calibPainel.offsetMV  = PAINEL_OFFSET_MV;  // 2500mV inicial
    calibPainel.sensMvA   = PAINEL_SENS_MV_A;  // 40mV/A
    calibPainel.invertido = true;               // Cabos invertidos
    calibPainel.calibrado = true;

    // CASA: ACS758LCB-100B Unidirecional
    // Offset FIXO = 600mV | Sens = 40mV/A (corrigido)
    calibCasa.offsetMV  = CASA_OFFSET_MV;      // 600mV
    calibCasa.sensMvA   = CASA_SENS_MV_A;      // 40mV/A
    calibCasa.invertido = false;
    calibCasa.calibrado = true;

    snprintf(tempBuf, sizeof(tempBuf),
             "Sensor Painel: off=%dmV sens=%.0fmV/A "
             "(offset sera refinado a noite)",
             calibPainel.offsetMV, calibPainel.sensMvA);
    adicionarLog(tempBuf);

    snprintf(tempBuf, sizeof(tempBuf),
             "Sensor Casa: off=%dmV sens=%.0fmV/A",
             calibCasa.offsetMV, calibCasa.sensMvA);
    adicionarLog(tempBuf);
}

// ============================================================
// SISTEMA: REDE WIFI E RECONEXÃO
// ============================================================

// ── Sincroniza hora via NTP ────────────────────────────────
void sincronizarNTP() {
    if (WiFi.status() != WL_CONNECTED) {
        adicionarLog("NTP: sem WiFi - ignorado");
        return;
    }

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

// ── Inicia modo Access Point ───────────────────────────────
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

// ── Inicia WiFi cliente ────────────────────────────────────
bool iniciarWiFi(const String& ssid, const String& pass,
                  const String& ip,   const String& gw,
                  const String& mask, const String& dns) {

    IPAddress lIP, lGW, lMask, lDNS;

    if (!lIP.fromString(ip))     { adicionarLog("ERRO: IP invalido");   return false; }
    if (!lGW.fromString(gw))     { adicionarLog("ERRO: GW invalido");   return false; }
    if (!lMask.fromString(mask)) { adicionarLog("ERRO: Mask invalida"); return false; }
    if (!lDNS.fromString(dns))   { adicionarLog("ERRO: DNS invalido");  return false; }

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

    if (!WiFi.config(lIP, lGW, lMask, lDNS)) {
        adicionarLog("ERRO: Config IP estatico falhou");
        return false;
    }

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
                 "WiFi OK! IP: %s", WiFi.localIP().toString().c_str());
        adicionarLog(tempBuf);
        snprintf(tempBuf, sizeof(tempBuf),
                 "RSSI: %ddBm | Canal: %d", WiFi.RSSI(), WiFi.channel());
        adicionarLog(tempBuf);
        logRAM();

        modoAP = false;
        ledFixo(true);

        sincronizarNTP();

        return true;
    }

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

// ── Reconexão WiFi inteligente ─────────────────────────────
void tentarReconectarWiFi() {
    static uint8_t tentativasRecon = 0;
    static bool    emReconexao     = false;

    if (emReconexao) return;
    emReconexao = true;

    tentativasRecon++;
    snprintf(tempBuf, sizeof(tempBuf),
             "WiFi perdido - tentativa %d/5", tentativasRecon);
    adicionarLog(tempBuf);

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

        tentativasRecon = 0;
        ledFixo(true);

        if (WiFi.getMode() == WIFI_AP_STA) {
            adicionarLog("Desativando AP temporario...");
            delay(1000);

            WiFi.mode(WIFI_STA);
            WiFi.setSleep(false);

            if (!gWifiIP.isEmpty()) {
                IPAddress lIP, lGW, lMask, lDNS;
                lIP.fromString(gWifiIP);
                lGW.fromString(gWifiGW);
                lMask.fromString(gWifiMASK);
                lDNS.fromString(gWifiDNS);
                WiFi.config(lIP, lGW, lMask, lDNS);
            }

            modoAP = false;

            server.close();
            delay(200);
            configurarRotas();

            adicionarLog("Modo STA restaurado");
        }

        sincronizarNTP();

        emReconexao = false;
        return;
    }

    if (tentativasRecon >= 5) {
        adicionarLog("5 tentativas falharam");
        adicionarLog("Ativando AP temporario...");
        tentativasRecon = 0;

        WiFi.mode(WIFI_AP_STA);
        WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
        WiFi.softAP(AP_SSID);
        dnsServer.start(DNS_PORT, "*", AP_IP);

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
// INTERFACE WEB - PÁGINAS PRINCIPAIS
// ============================================================

// ============================================================
// SISTEMA: FUNÇÕES AUXILIARES HTML
// ============================================================

// ── Gera página HTML completa com template ─────────────────
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
        ".nav{display:flex;gap:4px;margin-top:12px;padding-top:10px;border-top:1px solid #333;}"
        ".nav a{flex:1;text-align:center;padding:9px 4px;background:#1e1e1e;color:#90a4ae;"
               "text-decoration:none;border-radius:4px;font-size:11px;border:1px solid #333;}"
        ".nav a.active{background:#0277bd;color:#fff;border-color:#0277bd;}"
        ".nav a:hover{background:#263238;color:#e0e0e0;}"
        ".sc{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:6px 0;}"
        ".sc .lbl{color:#546e7a;font-size:11px;margin-bottom:4px;}"
        ".sc .val{color:#e0e0e0;font-size:20px;font-weight:bold;}"
        ".sc .sub{color:#78909c;font-size:11px;margin-top:2px;}"
        ".sc.ok{border-color:#2e7d32;background:#0a1f0a;}"
        ".sc.wn{border-color:#ef6c00;background:#1f130a;}"
        ".sc.er{border-color:#c62828;background:#1f0a0a;}"
        ".barra-bg{background:#111;border:1px solid #333;border-radius:4px;"
                  "height:18px;margin:6px 0;overflow:hidden;}"
        ".barra{height:100%;border-radius:4px;transition:width .5s;text-align:center;"
               "line-height:18px;font-size:11px;font-weight:bold;color:#fff;}"
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

// ── Barra de progresso colorida ────────────────────────────
String barraProgresso(float pct, const String& label) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    String cor;
    if (pct >= 50)      cor = "#2e7d32";
    else if (pct >= 25) cor = "#ef6c00";
    else                cor = "#c62828";

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

// ── Card de status ─────────────────────────────────────────
String statusCard(const String& icone, const String& label,
                  const String& valor, const String& sub,
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

// ============================================================
// HANDLERS WEB: AUTENTICAÇÃO E SETUP
// ============================================================

// ── Setup inicial: criar credenciais ───────────────────────
void handleSetupLogin() {
    if (existeCredenciais() && existeWiFiConfig()) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location","/setup-wifi"); server.send(302); return;
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

// ── Setup: configurar WiFi ─────────────────────────────────
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

// ── Login ──────────────────────────────────────────────────
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

// ── Logout ─────────────────────────────────────────────────
void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado = false; }
    server.sendHeader("Location","/login"); server.send(302);
}

// ── Not Found ──────────────────────────────────────────────
void handleNotFound() {
    if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else server.send(404,"text/plain","404");
}

// ── Endpoint: /logs (JSON) ─────────────────────────────────
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
// HANDLERS WEB: PÁGINA PRINCIPAL (DASHBOARD)
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

    char strPainel[12], strCasa[12], strBat[12];
    char strPP[12],     strPC[12],   strPB[12];
    snprintf(strPainel, sizeof(strPainel), "%.1fA", fabsf(sensores.correntePainel));
    snprintf(strCasa,   sizeof(strCasa),   "%.1fA", sensores.correnteCasa);
    snprintf(strBat,    sizeof(strBat),    "%.1fA", fabsf(sensores.correnteBateria));
    snprintf(strPP,     sizeof(strPP),     "%.0fW", fabsf(sensores.potenciaPainel));
    snprintf(strPC,     sizeof(strPC),     "%.0fW", sensores.potenciaCasa);
    snprintf(strPB,     sizeof(strPB),     "%.0fW", fabsf(sensores.potenciaBateria));

    char pctBuf[8];
    snprintf(pctBuf, sizeof(pctBuf), "%.0f%%",
             bateria.calibrada ? bateria.percentualAtual : 0.0f);

    // ── Corrente da bateria formatada com sinal (usada na caixa da bateria) ──
    char strBatSinal[14];
    {
        char sinalBat = batCarregando ? '+' : batDescarregando ? '-' : '~';
        snprintf(strBatSinal, sizeof(strBatSinal), "%c%.1fA",
                 sinalBat, fabsf(sensores.correnteBateria));
    }

    String tLampInt="--", tLampExt="--", tRedeAlt="--", tRedeBx="--";
    if (batDescarregando && bateria.calibrada) {
        float tx = sensores.potenciaBateria;
        tLampInt = tempoAteCorte(redes.corteLampInt, tx);
        tLampExt = tempoAteCorte(redes.corteLampExt, tx);
        tRedeAlt = tempoAteCorte(redes.corteAlta,    tx);
        tRedeBx  = tempoAteCorte(redes.corteBaixa,   tx);
    }

    String corPainel  = painelGerando    ? "#fdd835" : "#455a64";
    String corBat     = !bateria.calibrada          ? "#455a64" :
                         bateria.percentualAtual>50  ? "#43a047" :
                         bateria.percentualAtual>25  ? "#ef6c00" : "#c62828";
    String corInv     = invAtivo         ? "#4fc3f7" : "#455a64";
    String corAlt     = curtoAlt         ? "#ff6f00" : redeAltaON  ? "#ef5350" : "#455a64";
    String corBaixa   = curtoBx          ? "#ff6f00" : redeBaixaON ? "#42a5f5" : "#455a64";
    String corLampExt = lampExtON        ? "#ffd54f" : "#455a64";
    String corLampInt = lampIntON        ? "#ffd54f" : "#455a64";

    bool fluxoPB = painelGerando && batCarregando;
    bool fluxoPC = painelGerando && casaConsumindo;
    bool fluxoBC = batDescarregando && casaConsumindo;

    String c = "";

    if (!bateria.calibrada) {
        c += "<div class='wn' style='font-size:11px;margin-bottom:6px'>"
             "&#9888; Bateria nao calibrada. "
             "<a href='/bateria' style='color:#ffe0b2'>Calibrar agora</a></div>";
    }

    // ── CSS Diagrama ───────────────────────────────────────────
    c += F("<style>"
        "@keyframes fluir{to{stroke-dashoffset:-24}}"
        ".fx-linha{stroke-dasharray:6 6;animation:fluir 1s linear infinite;}"
        ".fx-parada{opacity:0.12;stroke-dasharray:2 4;}"
        ".fx-caixa{filter:drop-shadow(0 2px 6px rgba(0,0,0,0.5));}"
        ".fx-lbl{font-family:Arial,sans-serif;font-weight:bold;}"
        ".fx-sub{font-family:Arial,sans-serif;}"
        ".fx-pulse{animation:pulso 1.6s ease-in-out infinite;}"
        "@keyframes pulso{0%,100%{opacity:1}50%{opacity:.55}}"
    "</style>");

    // ── Diagrama de Fluxo SVG ──────────────────────────────────
    c += "<h3 style='text-align:center;margin:4px 0 10px'>&#9889; Fluxo de Energia</h3>";

    {
        String cPainelBg = painelGerando ? "#3a2f00" : "#181818";
        String cPainelBd = painelGerando ? "#fdd835" : "#3a3a3a";
        String cBatBg    = !bateria.calibrada ? "#181818" :
                            bateria.percentualAtual>50 ? "#0a2210" :
                            bateria.percentualAtual>25 ? "#2a1600" : "#2a0808";
        String cBatBd    = corBat;
        String cCasaBg   = casaConsumindo ? "#062231" : "#181818";
        String cCasaBd   = casaConsumindo ? "#4fc3f7" : "#3a3a3a";

        String lnPB = fluxoPB ? "#43a047" : "#333";
        String lnPC = fluxoPC ? "#fdd835" : "#333";
        String lnBC = fluxoBC ? "#ef6c00" : "#333";

        c += "<svg viewBox='0 0 340 230' xmlns='http://www.w3.org/2000/svg' "
             "style='width:100%;height:auto;display:block;margin:0 auto' id='svg-fluxo'>";

        // Linhas de conexão
        c += "<path d='M110,55 C160,20 200,20 240,55' fill='none' stroke='" + lnPB +
             "' stroke-width='3' stroke-linecap='round' class='" +
             String(fluxoPB?"fx-linha":"fx-parada") + "'/>";
        c += "<path d='M90,90 C80,130 90,150 130,175' fill='none' stroke='" + lnPC +
             "' stroke-width='3' stroke-linecap='round' class='" +
             String(fluxoPC?"fx-linha":"fx-parada") + "'/>";
        c += "<path d='M255,90 C260,130 250,150 210,175' fill='none' stroke='" + lnBC +
             "' stroke-width='3' stroke-linecap='round' class='" +
             String(fluxoBC?"fx-linha":"fx-parada") + "'/>";

        // Rótulos nas linhas (mostram POTÊNCIA, não corrente)
        if (fluxoPB) {
            // Potência indo do painel para a bateria
            c += "<rect x='143' y='24' width='54' height='16' rx='8' fill='#0d0d0d'/>";
            c += "<text x='170' y='36' text-anchor='middle' font-size='11' fill='" +
                 lnPB + "' class='fx-lbl' id='svg-bat-w'>" + String(strPB) + "</text>";
        }
        if (fluxoPC) {
            // Potência indo do painel para a casa (CORRIGIDO: antes usava strPP por engano)
            c += "<rect x='60' y='125' width='46' height='15' rx='7' fill='#0d0d0d'/>";
            c += "<text x='83' y='136' text-anchor='middle' font-size='10' fill='" +
                 lnPC + "' class='fx-lbl' id='svg-pc-w'>" + String(strPC) + "</text>";
        }
        if (fluxoBC) {
            // Potência indo da bateria para a casa
            c += "<rect x='232' y='125' width='46' height='15' rx='7' fill='#0d0d0d'/>";
            c += "<text x='255' y='136' text-anchor='middle' font-size='10' fill='" +
                 lnBC + "' class='fx-lbl' id='svg-pb-w'>" + String(strPB) + "</text>";
        }

        // Caixa Painel Solar
        c += "<g class='fx-caixa" + String(painelGerando?" fx-pulse":"") + "'>";
        c += "<rect x='20' y='15' width='100' height='80' rx='14' fill='" + cPainelBg +
             "' stroke='" + cPainelBd + "' stroke-width='2'/>";
        c += "<text x='70' y='42' text-anchor='middle' font-size='24'>&#9728;</text>";
        c += "<text x='70' y='60' text-anchor='middle' font-size='9' fill='#78909c' "
             "class='fx-sub'>PAINEL SOLAR</text>";
        c += "<text x='70' y='78' text-anchor='middle' font-size='15' fill='" +
             cPainelBd + "' class='fx-lbl' id='svg-painel-a'>";
        c += painelGerando ? String(strPainel) : String("0.0A");
        c += "</text>";
        c += "<text x='70' y='90' text-anchor='middle' font-size='9' fill='#546e7a'"
             " id='svg-painel-w'>";
        c += painelGerando ? String(strPP) : String("sem geracao");
        c += "</text>";
        c += "</g>";

        // Caixa Bateria
        c += "<g class='fx-caixa'>";
        c += "<rect x='220' y='15' width='100' height='80' rx='14' fill='" + cBatBg +
             "' stroke='" + cBatBd + "' stroke-width='2'/>";
        String icoBat = batCarregando ? "&#9650;&#128267;" :
                        batDescarregando ? "&#9660;&#128267;" : "&#128267;";
        c += "<text x='270' y='42' text-anchor='middle' font-size='22'>" + icoBat + "</text>";
        c += "<text x='270' y='60' text-anchor='middle' font-size='9' fill='#78909c' "
             "class='fx-sub'>BATERIA</text>";
        c += "<text x='270' y='78' text-anchor='middle' font-size='16' fill='" +
             cBatBd + "' class='fx-lbl' id='svg-bat-pct'>";
        c += bateria.calibrada ? String(pctBuf) : String("N/Cal");
        c += "</text>";
        // CORRIGIDO: aqui mostrava a tensao estimada (svg-bat-v). Agora mostra a
        // corrente da bateria (svg-bat-c), ja que a tensao ja aparece no card
        // "Tensao Estimada Bateria" logo abaixo do fluxo.
        c += "<text x='270' y='90' text-anchor='middle' font-size='9' fill='#ce93d8'"
             " id='svg-bat-c'>" + String(strBatSinal) + "</text>";
        c += "</g>";

        // Caixa Casa
        c += "<g class='fx-caixa" + String(casaConsumindo?" fx-pulse":"") + "'>";
        c += "<rect x='105' y='175' width='130' height='55' rx='14' fill='" + cCasaBg +
             "' stroke='" + cCasaBd + "' stroke-width='2'/>";
        c += "<text x='128' y='210' font-size='22'>&#127968;</text>";
        c += "<text x='225' y='196' text-anchor='end' font-size='9' fill='#78909c' "
             "class='fx-sub'>CASA</text>";
        c += "<text x='225' y='214' text-anchor='end' font-size='14' fill='" +
             (casaConsumindo ? cCasaBd : String("#455a64")) +
             "' class='fx-lbl' id='svg-casa-a'>";
        c += casaConsumindo ? String(strCasa) : String("0.0A");
        c += "</text>";
        c += "</g>";

        c += "</svg>";
    }

    // ── CSS Cargas ─────────────────────────────────────────────
    c += F("<style>"
        ".cg{border-radius:8px;border:2px solid #222;background:#111;"
            "padding:9px 11px;margin-bottom:5px;}"
        ".cg-row{display:flex;justify-content:space-between;align-items:center;}"
        ".cg-nm{font-size:12px;color:#90a4ae}"
        ".cg-st{font-size:13px;font-weight:bold}"
        ".cg-inf{font-size:10px;color:#546e7a;margin-top:5px;padding-top:5px;"
                "border-top:1px solid #1e1e1e;}"
    "</style>");

    // ── Cards de Cargas ────────────────────────────────────────
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
        if (infoTxt.length() > 0)
            h += "<div class='cg-inf'>" + infoTxt + "</div>";
        h += "</div>";
        return h;
    };

    String obsInv = "";
    if (diagAtivo)
        obsInv = "<span style='color:#90caf9'>&#128270; Diagnosticando...</span>";
    else if (!sensores.inversoreAtivo && estados.inversor)
        obsInv = "<span style='color:#78909c'>Aguardando sinal...</span>";

    String obsAlt = curto.redeAltaCurto ?
        "&#9888; Bloqueada por curto - <a href='/redes' style='color:#ffcc80'>resolver</a>" : "";

    // Informa modo de controle da lamp ext
    String obsLE = "";
    if (configLocal.configurado) {
        struct tm tiLE;
        if (getLocalTime(&tiLE)) {
            int mesLE  = tiLE.tm_mon + 1;
            float latR = (float)(configLocal.latitude * 3.14159 / 180.0);
            float dec  = -23.45f * cosf(2.0f*3.14159f*(float)(mesLE+9.5f)/12.0f);
            float dR   = dec * 3.14159f / 180.0f;
            float cH   = -tanf(latR)*tanf(dR);
            float hLuz = 12.0f;
            if (cH>=-1&&cH<=1) hLuz=(2.0f/15.0f)*(acosf(cH)*180.0f/3.14159f);
            float nSol = 12.0f-(hLuz/2.0f);
            float pSol = 12.0f+(hLuz/2.0f);
            char bufObs[80];
            snprintf(bufObs,sizeof(bufObs),
                     "&#9728; Horario solar | Nasc: %.0fh%02.0fm Por: %.0fh%02.0fm",
                     floorf(nSol),(nSol-floorf(nSol))*60,
                     floorf(pSol),(pSol-floorf(pSol))*60);
            obsLE = String(bufObs);
        }
    } else {
        obsLE = "&#9888; Configure localizacao em Previsao para controle solar preciso";
    }

    String obsBx = curto.redeBaixaCurto ? "&#9888; Bloqueada por curto" : "";

    c += carga("&#9889;",   "Inversor", corInv,
               invAtivo?"#081824":"#0d0d0d", invAtivo,
               invAtivo ? "ATIVO" : (estados.inversor ? "AGUARD" : "OFF"), obsInv);

    c += carga("&#128268;", "Rede Alta (Equipamentos pesados)", corAlt,
               curtoAlt ? "#1a0f00" : redeAltaON ? "#180808" : "#0d0d0d",
               redeAltaON&&!curtoAlt,
               curtoAlt ? "CURTO!" : (redeAltaON ? "ON" : "OFF"), obsAlt);

    c += carga("&#128268;", "Rede Baixa (Tomadas normais)", corBaixa,
               curtoBx ? "#1a0f00" : redeBaixaON ? "#080d18" : "#0d0d0d",
               redeBaixaON&&!curtoBx,
               curtoBx ? "CURTO!" : (redeBaixaON ? "ON" : "OFF"), obsBx);

    c += carga("&#128161;", "Lamp. Externa", corLampExt,
               lampExtON?"#181500":"#0d0d0d", lampExtON,
               lampExtON?"ON":"OFF", obsLE);

    c += carga("&#128161;", "Lamp. Interna", corLampInt,
               lampIntON?"#181500":"#0d0d0d", lampIntON,
               lampIntON?"ON":"OFF", "");

    // Alerta de curto
    if (curtoAlt || curtoBx || curto.estado == CURTO_AGUARDANDO_RESET) {
        c += "<div style='background:#7f0000;border:2px solid #ef5350;"
             "border-radius:8px;padding:10px;text-align:center;margin-top:4px'>";
        c += "<div style='font-size:13px;font-weight:bold;color:#ef9a9a'>&#9888; ";
        if (curtoAlt)                             c += "CURTO NA REDE ALTA";
        if (curtoBx)                              c += "CURTO NA REDE BAIXA";
        if (curto.estado==CURTO_AGUARDANDO_RESET) c += "FALHA NO INVERSOR";
        c += "</div>";
        c += "<a href='/redes' style='color:#ffcc80;font-size:12px'>"
             "Toque aqui para resolver &rarr;</a>";
        c += "</div>";
    }

    // ── Card Tensão ────────────────────────────────────────────
    {
        String corV = estimTensao.painelAtivo ? "#fdd835" :
                      estimTensao.tensaoAtual >
                      (estimTensao.tensaoNominalReal * 0.95f) ? "#43a047" :
                      estimTensao.tensaoAtual >
                      (estimTensao.tensaoNominalReal * 0.85f) ? "#ef6c00" : "#c62828";

        String statusV = estimTensao.painelAtivo ?
                         "&#9728; Carga (painel ON)" :
                         sensores.correnteBateria < -0.5f ?
                         "&#9660; Descarga" : "&#9679; Repouso";

        c += "<div class='sc' style='margin:8px 0;border-color:" + corV +
             ";background:#0d0d0d'>";
        c += "<div style='display:flex;justify-content:space-between;align-items:center'>";
        c += "<div>";
        c += "<div class='lbl'>&#9889; Tensao Estimada Bateria</div>";
        char bufVA[12];
        snprintf(bufVA, sizeof(bufVA), "%.2fV", estimTensao.tensaoAtual);
        c += "<div style='font-size:32px;font-weight:bold;color:" +
             corV + "' id='tensao-val'>";
        c += String(bufVA) + "</div>";
        c += "<div style='font-size:11px;color:#78909c'>" + statusV + "</div>";
        c += "</div>";
        c += "<div style='text-align:right;font-size:10px;color:#546e7a'>";
        char bufVmax[8], bufVnom[8];
        snprintf(bufVmax, sizeof(bufVmax), "%.1fV", estimTensao.tensaoMaxima);
        snprintf(bufVnom, sizeof(bufVnom), "%.1fV", estimTensao.tensaoNominalReal);
        c += "Vmax: <b style='color:#90a4ae'>" + String(bufVmax) + "</b><br>";
        c += "Vnom: <b style='color:#90a4ae'>" + String(bufVnom) + "</b><br>";
        c += "Cal: <b style='color:#90a4ae'>" +
             String(estimTensao.fatorCalibTensao, 3) + "</b><br>";
        c += estimTensao.amostrasCalib > 20 ?
             "<span style='color:#43a047'>&#10003; Calibrado</span>" :
             "<span style='color:#ef6c00'>&#128336; Calibrando</span>";
        c += " (" + String(estimTensao.amostrasCalib) + ")";
        c += "</div></div></div>";
    }

    // ── Barra Bateria ──────────────────────────────────────────
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
                     (tempo.horas==0&&tempo.minutos<30 ? "#ef5350":"#ef6c00");
        c += "<span style='font-size:11px;font-weight:bold;color:"+corT+"'>";
        c += tempo.carregando ? "&#9650; " : "&#9660; ";
        c += String(bufT) + "</span>";
    }
    c += "</div>";

    if (bateria.calibrada) {
        float  pct    = bateria.percentualAtual;
        String corBrr = pct>=50?"#2e7d32":pct>=25?"#ef6c00":"#c62828";
        c += "<div class='barra-bg'><div class='barra' id='bat-barra' style='width:"+
             String((int)pct)+"%;background:"+corBrr+"'>"+
             String((int)pct)+"%</div></div>";
        c += "<div style='display:flex;justify-content:space-between;"
             "margin-top:3px;font-size:10px;color:#546e7a'>";
        char bWh[60];
        snprintf(bWh,sizeof(bWh),"%.0f / %.0f Wh",
                 bateria.energiaAtual,bateria.energiaDisponivel);
        c += "<span id='bat-wh'>" + String(bWh) + "</span>";
        char bPot[40];
        snprintf(bPot,sizeof(bPot),"%.0fW @ %.2fV %s",
                 fabsf(sensores.potenciaBateria),
                 estimTensao.tensaoAtual,
                 batCarregando?"entrada":batDescarregando?"saida":"equil.");
        c += "<span id='bat-pot'>" + String(bPot) + "</span></div>";
    } else {
        c += "<div class='wn' style='font-size:11px;text-align:center'>"
             "Nao calibrada - "
             "<a href='/bateria' style='color:#ffe0b2'>calibrar</a></div>";
    }
    c += "</div>";

    // ── Cards Potência (com IDs para AJAX) ─────────────────────
    c += "<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
         "gap:6px;margin:4px 0'>";

    // Card Painel
    {
        String h = "<div class='sc";
        if (painelGerando) h += " ok";
        h += "' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>&#9728; Painel</div>";
        h += "<div style='font-size:15px;font-weight:bold;color:" +
             corPainel + "' id='corr-painel'>";
        h += painelGerando ? String(strPainel) : String("0.0A");
        h += "</div>";
        h += "<div class='sub' id='pot-painel'>";
        h += painelGerando ? String(strPP) : String("Sem geracao");
        h += "</div></div>";
        c += h;
    }

    // Card Casa
    {
        String h = "<div class='sc' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>&#127968; Casa</div>";
        h += "<div style='font-size:15px;font-weight:bold;color:#4fc3f7'"
             " id='corr-casa'>";
        h += String(strCasa);
        h += "</div>";
        h += "<div class='sub' id='pot-casa'>" + String(strPC) + "</div>";
        h += "</div>";
        c += h;
    }

    // Card Bateria
    {
        String corF  = batCarregando ?"#43a047":
                       batDescarregando?"#ef6c00":"#fdd835";
        String sinal = batCarregando ?"+":
                       batDescarregando?"-":"~";
        char bufB[30];
        snprintf(bufB,sizeof(bufB),"%.0fW @ %.2fV",
                 fabsf(sensores.potenciaBateria),
                 estimTensao.tensaoAtual);
        String h = "<div class='sc' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>&#128267; Bateria</div>";
        h += "<div style='font-size:15px;font-weight:bold;color:" +
             corF + "' id='corr-bat'>";
        h += sinal + String(strBat);
        h += "</div>";
        h += "<div class='sub' id='pot-bat'>" + String(bufB) + "</div>";
        h += "</div>";
        c += h;
    }
    c += "</div>";

    // ── Info Sistema ───────────────────────────────────────────
    String ip   = modoAP?WiFi.softAPIP().toString():WiFi.localIP().toString();
    String modo = modoAP?"AP":"WiFi";
    c += "<div class='nf' style='font-size:10px;margin-top:4px'>";
    c += "&#128246; "+modo+" | "+ip;
    if (!modoAP && WiFi.status()==WL_CONNECTED)
        c += " | "+String(WiFi.RSSI())+"dBm";
    snprintf(tempBuf,sizeof(tempBuf)," | RAM:%.1fKB | Up:%lus",
             esp_get_free_heap_size()/1024.0f, millis()/1000);
    c += String(tempBuf)+"</div>";

    // ── AJAX atualização a cada 1 segundo ──────────────────────
    c += F("<script>"
    "function atualizarDados(){"
    "var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){"
    "if(x.readyState==4&&x.status==200){"
    "try{"
    "var d=JSON.parse(x.responseText);"

    // Tensão
    "var elV=document.getElementById('tensao-val');"
    "if(elV)elV.textContent=parseFloat(d.vat||0).toFixed(2)+'V';"

    // Correntes (declaradas antes de usar em qualquer lugar)
    "var ip=parseFloat(d.ip||0);"
    "var ic=parseFloat(d.ic||0);"
    "var ib=parseFloat(d.ib||0);"

    // SVG Bateria: corrente (com sinal) e %
    "var svBC=document.getElementById('svg-bat-c');"
    "if(svBC){"
    "var sB=ib>0.3?'+':ib<-0.3?'-':'~';"
    "svBC.textContent=sB+Math.abs(ib).toFixed(1)+'A';}"
    "var svBP=document.getElementById('svg-bat-pct');"
    "if(svBP)svBP.textContent=Math.round(d.pct||0)+'%';"

    // Correntes nos cards
    "var elIP=document.getElementById('corr-painel');"
    "if(elIP)elIP.textContent=Math.abs(ip).toFixed(1)+'A';"

    "var elIC=document.getElementById('corr-casa');"
    "if(elIC)elIC.textContent=ic.toFixed(1)+'A';"

    "var elIB=document.getElementById('corr-bat');"
    "if(elIB){"
    "var s=ib>0.3?'+':ib<-0.3?'-':'~';"
    "elIB.textContent=s+Math.abs(ib).toFixed(1)+'A';}"

    // SVG correntes
    "var svPA=document.getElementById('svg-painel-a');"
    "if(svPA)svPA.textContent=ip>0.5?Math.abs(ip).toFixed(1)+'A':'0.0A';"
    "var svCA=document.getElementById('svg-casa-a');"
    "if(svCA)svCA.textContent=ic>0.3?ic.toFixed(1)+'A':'0.0A';"

    // Potências nos cards
    "var pp=parseFloat(d.pp||0);"
    "var pc=parseFloat(d.pc||0);"
    "var pb=parseFloat(d.pb||0);"

    "var elPP=document.getElementById('pot-painel');"
    "if(elPP)elPP.textContent=ip>0.5?Math.abs(pp).toFixed(0)+'W':'Sem geracao';"

    "var elPC=document.getElementById('pot-casa');"
    "if(elPC)elPC.textContent=Math.abs(pc).toFixed(0)+'W';"

    "var elPB=document.getElementById('pot-bat');"
    "if(elPB){"
    "var vat=parseFloat(d.vat||0);"
    "elPB.textContent=Math.abs(pb).toFixed(0)+'W @ '+vat.toFixed(2)+'V';}"

    // Barra bateria
    "var pct=parseFloat(d.pct||0);"
    "var elBarra=document.getElementById('bat-barra');"
    "if(elBarra&&d.calib){"
    "elBarra.style.width=pct+'%';"
    "elBarra.textContent=Math.round(pct)+'%';"
    "var cor=pct>=50?'#2e7d32':pct>=25?'#ef6c00':'#c62828';"
    "elBarra.style.background=cor;}"

    // Wh bateria
    "var elWh=document.getElementById('bat-wh');"
    "if(elWh&&d.calib){"
    "elWh.textContent=Math.round(d.enrg||0)+' / '+Math.round(d.disp||0)+' Wh';}"

    // Pot bateria info
    "var elBPot=document.getElementById('bat-pot');"
    "if(elBPot){"
    "var st=ib>0.3?'entrada':ib<-0.3?'saida':'equil.';"
    "elBPot.textContent=Math.abs(pb).toFixed(0)+'W @ '+"
    "parseFloat(d.vat||0).toFixed(2)+'V '+st;}"

    "}catch(e){console.log('AJAX err:',e);}}"
    "};x.open('GET','/dados',true);x.send();}"

    // Atualiza a cada 1 segundo
    "setInterval(atualizarDados,1000);"
    "atualizarDados();"

    // Reload completo a cada 30s (atualiza SVG e estados dos relés)
    "setInterval(function(){location.reload();},30000);"
    "</script>");

    server.send(200,"text/html; charset=utf-8",
                gerarPagina("RRFS - Painel",c));
}

// ── Endpoint: /dados (JSON para AJAX) ─────────────────────
void handleDados() {
    if (!logado) { server.send(403); return; }

    int total = histTensaoFull ? HIST_TENSAO_PONTOS : histTensaoIdx;

    String json = "{";

    char buf[512];
    snprintf(buf, sizeof(buf),
             "\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
             "\"pp\":%.1f,\"pc\":%.1f,\"pb\":%.1f,"
             "\"pct\":%.1f,\"enrg\":%.0f,\"disp\":%.0f,"
             "\"vat\":%.2f,\"vmax\":%.2f,\"vnom\":%.2f,"
             "\"painel_on\":%s,"
             "\"inv\":%s,\"rb\":%s,\"ra\":%s,\"le\":%s,\"li\":%s,"
             "\"calib\":%s,\"ncalib\":%d",
             sensores.correntePainel,
             sensores.correnteCasa,
             sensores.correnteBateria,
             sensores.potenciaPainel,
             sensores.potenciaCasa,
             sensores.potenciaBateria,
             bateria.percentualAtual,
             bateria.energiaAtual,
             bateria.energiaDisponivel,
             estimTensao.tensaoAtual,
             estimTensao.tensaoMaxima,
             estimTensao.tensaoNominalReal,
             estimTensao.painelAtivo ? "true" : "false",
             sensores.inversoreAtivo ? "true":"false",
             estados.redeBaixa  ? "true":"false",
             estados.redesAlta  ? "true":"false",
             estados.lampExt    ? "true":"false",
             estados.lampInt    ? "true":"false",
             bateria.calibrada  ? "true":"false",
             estimTensao.amostrasCalib
    );
    json += String(buf);

    json += ",\"hist_v\":[";
    for (int i = 0; i < total; i++) {
        int idx = histTensaoFull ?
        (histTensaoIdx + i) % HIST_TENSAO_PONTOS : i;
        if (i > 0) json += ",";
        char vb[10];
        snprintf(vb, sizeof(vb), "%.2f", histTensaoV[idx]);
        json += String(vb);
    }
    json += "],\"hist_p\":[";
    for (int i = 0; i < total; i++) {
        int idx = histTensaoFull ?
        (histTensaoIdx + i) % HIST_TENSAO_PONTOS : i;
        if (i > 0) json += ",";
        char vb[10];
        snprintf(vb, sizeof(vb), "%.1f",
                 histCorrPainel[idx] * histTensaoV[idx]);
        json += String(vb);
    }
    json += "],\"hist_c\":[";
    for (int i = 0; i < total; i++) {
        int idx = histTensaoFull ?
        (histTensaoIdx + i) % HIST_TENSAO_PONTOS : i;
        if (i > 0) json += ",";
        char vb[10];
        snprintf(vb, sizeof(vb), "%.1f",
                 histCorrCasa[idx] * histTensaoV[idx]);
        json += String(vb);
    }
    json += "],\"hist_b\":[";
    for (int i = 0; i < total; i++) {
        int idx = histTensaoFull ?
        (histTensaoIdx + i) % HIST_TENSAO_PONTOS : i;
        if (i > 0) json += ",";
        char vb[10];
        snprintf(vb, sizeof(vb), "%.1f",
                 histCorrBat[idx] * histTensaoV[idx]);
        json += String(vb);
    }
    json += "]}";

    server.send(200, "application/json", json);
}

// ============================================================
// HANDLERS WEB: PÁGINA DE BATERIA
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

    calcularBateria();

    String batCls = "ok";
    if (bateria.percentualAtual < 25) batCls = "er";
    else if (bateria.percentualAtual < 50) batCls = "wn";

    String c = msg;

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

    c += "<div class='nf'>";
    char buf[120];
    snprintf(buf, sizeof(buf),
             "Tensao: %.1fV | Cap: %.0fAh | Descarga segura: %.0f%%<br>"
             "Energia total: %.0fWh | Disponivel: %.0fWh",
             bateria.tensaoNominal, bateria.capacidadeAh, bateria.descargaSegura,
             bateria.energiaTotal, bateria.energiaDisponivel);
    c += String(buf) + "</div>";

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
// HANDLERS WEB: PÁGINA DE REDES
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

    String c = msg;

    bool curtoAlt  = (curto.estado == CURTO_CONFIRMADO_ALTA  || curto.redeAltaCurto);
    bool curtoBx   = (curto.estado == CURTO_CONFIRMADO_BAIXA || curto.redeBaixaCurto);
    bool diagAtivo = (curto.estado == CURTO_TESTANDO_ALTA    ||
                      curto.estado == CURTO_TESTANDO_BAIXA   ||
                      curto.estado == CURTO_RELIGANDO);

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

    c += "<h3>&#9874; Protecao Contra Curto</h3>";
    c += htmlStatusCurto();

    c += "<h3>&#128268; Controle das Cargas</h3>";

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

    auto linhaRele = [&](const char* icone, const char* nome,
                         const char* chave, bool ativo,
                         bool bloqueado,
                         const String& obs = "") -> String {

        String cor  = ativo   ? "#43a047" : "#ef5350";
        String est  = ativo   ? "ON"      : "OFF";
        if (bloqueado && !ativo) { cor = "#546e7a"; }

        String h = "<div style='background:#111;border:1px solid #1e1e1e;"
        "border-radius:6px;padding:8px 10px;margin-bottom:5px'>";

        h += "<div style='display:flex;justify-content:space-between;"
        "align-items:center'>";

        h += "<span style='font-size:12px;color:#90a4ae'>";
        h += String(icone) + " " + String(nome);
        h += "</span>";

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

        if (obs.length() > 0) {
            h += "<div style='font-size:10px;color:#546e7a;"
            "margin-top:5px;padding-top:5px;"
            "border-top:1px solid #1a1a1a'>" + obs + "</div>";
        }

        h += "</div>";
        return h;
    };

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

    c += "<h3>&#9881; Configurar Cortes de Energia</h3>";
    c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>"
    "Percentual baseado na energia disponivel da bateria.<br>"
    "Buffer de 60s antes de desligar. Religamento com margem de +2%."
    "</div>";

    c += "<form method='POST' action='/redes'>";
    c += "<input type='hidden' name='acao' value='config'>";

    c += "<label>&#9889; Rede Baixa + Inversor - Desligar em (%):</label>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteBaixa);
    c += "<input type='number' name='c_baixa' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 10'>";

    c += "<label>&#9889; Rede Alta - Desligar em (%):</label>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteAlta);
    c += "<input type='number' name='c_alta' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 30'>";

    c += "<label>&#128161; Lamp. Externa - Desligar por energia em (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0 4px'>"
    "&#9728; Lamp. Externa tambem desliga automaticamente durante o dia "
    "(painel gerando &gt; 0.5A).</div>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteLampExt);
    c += "<input type='number' name='c_lmpext' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 50'>";

    c += "<label>&#128161; Lamp. Interna - Desligar em (%):</label>";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteLampInt);
    c += "<input type='number' name='c_lmpint' step='1' min='0' max='100' required ";
    c += String(tempBuf) + " placeholder='Padrao: 0'>";

    c += "<button type='submit' class='btn-g'>&#128190; Salvar Configuracao</button>";
    c += "</form>";

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
// HANDLERS WEB: PÁGINA DE SENSORES
// ============================================================

void handleSensores() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    // Leituras atuais dos ADCs
    int   adcPainel  = lerADC(PIN_SENSOR_PAINEL);
    int   adcCasa    = lerADC(PIN_SENSOR_CASA);
    int   mvPainel   = adcParaMv(adcPainel);
    int   mvCasa     = adcParaMv(adcCasa);
    float corrPainel = sensores.correntePainel;
    float corrCasa   = sensores.correnteCasa;
    int   adcInv     = lerADC(PIN_STATUS_INV_ALTA);
    int   mvInv      = adcParaMv(adcInv);

    String c = "";

    // ══════════════════════════════════════════════════════════
    // SENSOR PAINEL - ACS758 50A BIDIRECIONAL
    // ══════════════════════════════════════════════════════════
    c += "<h3>&#9728; Sensor Painel (ACS758 50A Bidirecional)</h3>";

    // Calcula corrente bruta sem filtro para debug
    float desvioPainel = (float)(mvPainel - calibPainel.offsetMV);
    float corrBrutaPainel = -(desvioPainel / calibPainel.sensMvA);

    // Determina estado do sinal
    String estadoSinalP = "";
    String corEstadoP   = "";
    if (mvPainel < calibPainel.offsetMV - 20) {
        estadoSinalP = "&#9660; GERANDO (sinal desceu abaixo offset)";
        corEstadoP   = "#43a047";
    } else if (mvPainel > calibPainel.offsetMV + 20) {
        estadoSinalP = "&#9650; REVERSO (sinal subiu acima offset)";
        corEstadoP   = "#ef5350";
    } else {
        estadoSinalP = "&#9679; ZONA MORTA (proximo do offset)";
        corEstadoP   = "#ef6c00";
    }

    // Card principal painel
    String clsPainel = corrPainel > 0.5f ? "ok" :
    corrPainel > 0.1f ? "wn" : "sc";
    c += "<div class='sc " + clsPainel + "'>";
    c += "<div class='lbl'>Corrente Medida (filtrada)</div>";
    c += "<div class='val'>" + fmtCorrente(corrPainel) + "</div>";
    char bufSP[80];
    snprintf(bufSP, sizeof(bufSP),
             "Potencia: %.0fW @ %.2fV",
             fabsf(sensores.potenciaPainel),
             estimTensao.tensaoAtual);
    c += "<div class='sub'>" + String(bufSP) + "</div>";
    c += "</div>";

    // Debug detalhado painel
    c += "<div class='nf' style='font-size:11px'>";
    c += "<b>&#128270; Dados do Sinal (Depuracao):</b><br>";

    char bufDebP[400];
    snprintf(bufDebP, sizeof(bufDebP),
             "&#9679; Sinal ADC: <b>%d</b> (0-4095)<br>"
             "&#9679; Sinal mV: <b>%d mV</b><br>"
             "&#9679; Offset datasheet: <b>%d mV</b><br>"
             "&#9679; Offset em uso: <b>%d mV</b> %s<br>"
             "&#9679; Desvio do offset: <b>%+.0f mV</b><br>"
             "&#9679; Corrente bruta (sem filtro): <b>%+.2f A</b><br>"
             "&#9679; Corrente filtrada (final): <b>%.2f A</b><br>"
             "&#9679; Sensibilidade: <b>%.0f mV/A</b><br>",
             adcPainel,
             mvPainel,
             PAINEL_OFFSET_MV,
             calibPainel.offsetMV,
             calibPainel.offsetMV != PAINEL_OFFSET_MV ?
             "(auto-calibrado &#10003;)" : "(datasheet)",
             desvioPainel,
             corrBrutaPainel,
             corrPainel,
             calibPainel.sensMvA);
    c += String(bufDebP);

    c += "&#9679; Estado do sinal: <b style='color:" +
    corEstadoP + "'>" + estadoSinalP + "</b><br>";
    c += "&#9679; Cabos: <b>INVERTIDOS propositalmente</b> "
    "(protecao ESP32 - sinal so desce)<br>";
    c += "&#9679; Auto-calib offset: <b>aprende durante a noite (22h-4h)</b><br>";
    c += "&#9679; Zona morta: <b>positivo &gt;0.15A | negativo &gt;1A</b>";
    c += "</div>";

    // Aviso offset diferente do datasheet
    if (calibPainel.offsetMV != PAINEL_OFFSET_MV) {
        char bufOff[80];
        snprintf(bufOff, sizeof(bufOff),
                 "&#10003; Offset auto-calibrado: %dmV "
                 "(datasheet: %dmV | erro corrigido: %dmV)",
                 calibPainel.offsetMV,
                 PAINEL_OFFSET_MV,
                 calibPainel.offsetMV - PAINEL_OFFSET_MV);
        c += "<div class='ok' style='font-size:11px;margin-top:4px'>";
        c += String(bufOff) + "</div>";
    } else {
        c += "<div class='wn' style='font-size:11px;margin-top:4px'>"
        "&#128336; Offset ainda nao auto-calibrado. "
        "Sera refinado automaticamente entre 22h e 4h "
        "com sistema em repouso.</div>";
    }

    // Especificacoes tecnicas
    c += "<div class='nf' style='font-size:10px;margin-top:6px'>";
    c += "<b>Especificacoes (Datasheet):</b> "
    "ACS758 50A Bidirecional | "
    "VCC=5V | Offset=2500mV (VCC/2) | Sens=40mV/A | "
    "Faixa: -50A a +50A<br>";
    c += "<span style='color:#a5d6a7'>"
    "&#10003; Cabos invertidos: sinal so desce - "
    "ESP32 protegido contra sobretensao!</span><br>";
    c += "<span style='color:#90caf9'>"
    "&#128336; Offset refinado automaticamente durante a noite"
    "</span>";
    c += "</div>";

    // ══════════════════════════════════════════════════════════
    // SENSOR CASA - ACS758LCB-100B UNIDIRECIONAL
    // ══════════════════════════════════════════════════════════
    c += "<h3>&#127968; Sensor Casa (ACS758LCB-100B Unidirecional)</h3>";

    // Calcula corrente bruta sem filtro para debug
    float desvioCasa     = (float)(mvCasa - calibCasa.offsetMV);
    float corrBrutaCasa  = desvioCasa > 0 ?
    desvioCasa / calibCasa.sensMvA : 0.0f;

    // Determina estado do sinal casa
    String estadoSinalC = "";
    String corEstadoC   = "";
    if (mvCasa < calibCasa.offsetMV) {
        estadoSinalC = "&#9679; ABAIXO OFFSET (sem corrente)";
        corEstadoC   = "#546e7a";
    } else if (mvCasa < calibCasa.offsetMV + 40) {
        estadoSinalC = "&#9679; ZONA MORTA (corrente minima)";
        corEstadoC   = "#ef6c00";
    } else {
        estadoSinalC = "&#9650; CONSUMINDO (corrente positiva)";
        corEstadoC   = "#4fc3f7";
    }

    // Card principal casa
    String clsCasa = corrCasa > 1.0f ? "ok" :
    corrCasa > 0.3f ? "wn" : "sc";
    c += "<div class='sc " + clsCasa + "'>";
    c += "<div class='lbl'>Corrente Medida (filtrada)</div>";
    c += "<div class='val'>" + fmtCorrente(corrCasa) + "</div>";
    char bufSC[80];
    snprintf(bufSC, sizeof(bufSC),
             "Potencia: %.0fW @ %.2fV",
             sensores.potenciaCasa,
             estimTensao.tensaoAtual);
    c += "<div class='sub'>" + String(bufSC) + "</div>";
    c += "</div>";

    // Debug detalhado casa
    c += "<div class='nf' style='font-size:11px'>";
    c += "<b>&#128270; Dados do Sinal (Depuracao):</b><br>";

    char bufDebC[400];
    snprintf(bufDebC, sizeof(bufDebC),
             "&#9679; Sinal ADC: <b>%d</b> (0-4095)<br>"
             "&#9679; Sinal mV: <b>%d mV</b><br>"
             "&#9679; Offset fabrica: <b>%d mV</b> (fixo - sensor unidirecional)<br>"
             "&#9679; Desvio do offset: <b>%+.0f mV</b><br>"
             "&#9679; Corrente bruta (sem filtro): <b>%.2f A</b><br>"
             "&#9679; Corrente filtrada (final): <b>%.2f A</b><br>"
             "&#9679; Sensibilidade: <b>%.0f mV/A</b><br>"
             "&#9679; Sinal maximo (100A): <b>%d mV</b> (seguro no ESP32)<br>"
             "&#9679; Margem disponivel: <b>%d mV</b>",
             adcCasa,
             mvCasa,
             calibCasa.offsetMV,
             desvioCasa,
             corrBrutaCasa,
             corrCasa,
             calibCasa.sensMvA,
             (int)(calibCasa.offsetMV + 100 * calibCasa.sensMvA),
             3300 - mvCasa);
    c += String(bufDebC);
    c += "<br>&#9679; Estado do sinal: <b style='color:" +
    corEstadoC + "'>" + estadoSinalC + "</b>";
    c += "</div>";

    // Especificacoes tecnicas casa
    c += "<div class='nf' style='font-size:10px;margin-top:6px'>";
    c += "<b>Especificacoes (Datasheet):</b> "
    "ACS758LCB-100B Unidirecional | "
    "VCC=3-5.5V | Offset=600mV (fixo) | Sens=40mV/A | "
    "Faixa: 0A a +100A<br>";
    c += "<span style='color:#a5d6a7'>"
    "&#10003; Sinal maximo (100A) = ";
    char bufMax[20];
    snprintf(bufMax, sizeof(bufMax), "%dmV",
             (int)(calibCasa.offsetMV + 100 * calibCasa.sensMvA));
    c += String(bufMax);
    c += " - seguro no ESP32 (limite 3300mV)!</span>";
    c += "</div>";

    // ══════════════════════════════════════════════════════════
    // BATERIA - CALCULADA (Painel - Casa)
    // ══════════════════════════════════════════════════════════
    c += "<h3>&#128267; Bateria (Calculada: Painel - Casa)</h3>";

    float corrBat = sensores.correnteBateria;
    String dirBat = corrBat > 0.3f  ? "Carregando" :
    corrBat < -0.3f ? "Descarregando" : "Equilibrio";
    String clsBat = corrBat > 0.3f  ? "ok" :
    corrBat < -0.3f ? "wn" : "sc";

    c += "<div class='sc " + clsBat + "'>";
    c += "<div class='lbl'>Painel - Casa = Bateria</div>";
    c += "<div class='val'>";

    char bufCBat[20];
    snprintf(bufCBat, sizeof(bufCBat), "%+.2fA", corrBat);
    c += String(bufCBat);
    c += " | " + dirBat + "</div>";

    char bufPBat[60];
    snprintf(bufPBat, sizeof(bufPBat),
             "%.0fW @ %.2fV | SOC: %.1f%%",
             fabsf(sensores.potenciaBateria),
             estimTensao.tensaoAtual,
             bateria.calibrada ? bateria.percentualAtual : 0.0f);
    c += "<div class='sub'>" + String(bufPBat) + "</div>";
    c += "</div>";

    // Calculo detalhado
    c += "<div class='nf' style='font-size:11px'>";
    char bufCalcBat[200];
    snprintf(bufCalcBat, sizeof(bufCalcBat),
             "&#9679; Painel: <b>%+.2fA</b><br>"
             "&#9679; Casa: <b>%.2fA</b><br>"
             "&#9679; Bateria: <b>%+.2fA</b> "
             "(Painel - Casa)<br>"
             "&#9679; Potencia bateria: <b>%.0fW</b><br>"
             "&#9679; Tensao estimada: <b>%.2fV</b><br>"
             "&#9679; Energia atual: <b>%.0fWh</b> de <b>%.0fWh</b>",
             sensores.correntePainel,
             sensores.correnteCasa,
             corrBat,
             fabsf(sensores.potenciaBateria),
             estimTensao.tensaoAtual,
             bateria.energiaAtual,
             bateria.energiaDisponivel);
    c += String(bufCalcBat) + "</div>";

    // ══════════════════════════════════════════════════════════
    // STATUS INVERSOR
    // ══════════════════════════════════════════════════════════
    c += "<h3>&#9889; Sinal Status Inversor</h3>";

    String invSt  = sensores.inversoreAtivo ? "LIGADO"  : "DESLIGADO";
    String invCor = sensores.inversoreAtivo ? "#43a047" : "#546e7a";
    String invCls = sensores.inversoreAtivo ? "ok"      : "sc";

    c += "<div class='sc " + invCls + "'>";
    c += "<div class='lbl'>Status LED Inversor</div>";
    c += "<div class='val' style='color:" + invCor + "'>" + invSt + "</div>";
    char bufInv[100];
    snprintf(bufInv, sizeof(bufInv),
             "Sinal: %d mV | ADC: %d | Limiar: %d mV | Buffer: %dms",
             mvInv, adcInv, INV_THRESHOLD_MV, INV_BUFFER_MS);
    c += "<div class='sub'>" + String(bufInv) + "</div>";
    c += "</div>";

    c += "<div class='nf' style='font-size:11px'>";
    char bufInvD[100];
    snprintf(bufInvD, sizeof(bufInvD),
             "&#9679; Chave inversor: <b>%s</b><br>"
             "&#9679; Status LED: <b>%s</b><br>"
             "&#9679; Sinal mV: <b>%d mV</b> (limiar: %d mV)",
             estados.inversor ? "LIGADA" : "DESLIGADA",
             sensores.inversoreAtivo ? "ATIVO" : "INATIVO",
             mvInv, INV_THRESHOLD_MV);
    c += String(bufInvD) + "</div>";

    // ══════════════════════════════════════════════════════════
    // DIAGNÓSTICO DO BANCO LiFePO4
    // ══════════════════════════════════════════════════════════
    c += "<h3>&#128270; Diagnostico Banco LiFePO4</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    char bufDiag[300];
    snprintf(bufDiag, sizeof(bufDiag),
             "&#9679; Celulas em serie: <b>%.0fS</b><br>"
             "&#9679; Tensao nominal: <b>%.2fV</b> "
             "(%.0f x 3.2V)<br>"
             "&#9679; Tensao maxima carga: <b>%.2fV</b> "
             "(%.0f x 3.5V)<br>"
             "&#9679; Tensao minima corte: <b>%.2fV</b> "
             "(%.0f x 3.0V)<br>"
             "&#9679; Tensao estimada atual: <b>%.2fV</b><br>"
             "&#9679; Fator calibracao tensao: <b>%.4f</b> "
             "(%d amostras)<br>"
             "&#9679; Amostras por leitura ADC: <b>%d</b><br>"
             "&#9679; Buffer media movel: <b>%d amostras</b>",
             estimTensao.fatorBanco,
             estimTensao.tensaoNominalReal,
             estimTensao.fatorBanco,
             estimTensao.tensaoMaxima,
             estimTensao.fatorBanco,
             3.0f * estimTensao.fatorBanco,
             estimTensao.fatorBanco,
             estimTensao.tensaoAtual,
             estimTensao.fatorCalibTensao,
             estimTensao.amostrasCalib,
             ADC_SAMPLES,
             BUFFER_CORRENTE);
    c += String(bufDiag) + "</div>";

    // Status calibração tensão
    if (estimTensao.amostrasCalib >= 20) {
        c += "<div class='ok' style='font-size:11px;margin-top:4px'>"
        "&#10003; Tensao bem calibrada (" +
        String(estimTensao.amostrasCalib) +
        " amostras)</div>";
    } else {
        c += "<div class='wn' style='font-size:11px;margin-top:4px'>"
        "&#128336; Calibrando tensao... " +
        String(estimTensao.amostrasCalib) +
        "/20 amostras minimas</div>";
    }

    // ══════════════════════════════════════════════════════════
    // RESUMO GERAL DO SISTEMA
    // ══════════════════════════════════════════════════════════
    c += "<h3>&#128202; Resumo Geral</h3>";
    c += "<div style='display:grid;grid-template-columns:1fr 1fr;"
    "gap:6px;margin-bottom:8px'>";

    auto cardResumo = [](const String& ico, const String& titulo,
                         const String& valor, const String& sub,
                         const String& cor) -> String {
                             String h = "<div style='background:#1a1a1a;border:2px solid #1e1e1e;"
                             "border-radius:8px;padding:10px;text-align:center'>";
                             h += "<div style='font-size:10px;color:#546e7a'>" +
                             ico + " " + titulo + "</div>";
                             h += "<div style='font-size:16px;font-weight:bold;color:" +
                             cor + ";margin-top:4px'>" + valor + "</div>";
                             h += "<div style='font-size:10px;color:#546e7a;margin-top:2px'>" +
                             sub + "</div>";
                             h += "</div>";
                             return h;
                         };

                         char bP[20], bC[20], bB[20], bV[20];
                         snprintf(bP, sizeof(bP), "%.2fA / %.0fW",
                                  corrPainel, fabsf(sensores.potenciaPainel));
                         snprintf(bC, sizeof(bC), "%.2fA / %.0fW",
                                  corrCasa, sensores.potenciaCasa);
                         snprintf(bB, sizeof(bB), "%+.2fA / %.0fW",
                                  corrBat, fabsf(sensores.potenciaBateria));
                         snprintf(bV, sizeof(bV), "%.2fV",
                                  estimTensao.tensaoAtual);

                         c += cardResumo("&#9728;",   "Painel Solar",
                                         String(bP), "geracao atual", "#fdd835");
                         c += cardResumo("&#127968;", "Consumo Casa",
                                         String(bC), "consumo atual", "#4fc3f7");
                         c += cardResumo("&#128267;", "Bateria",
                                         String(bB),
                                         bateria.calibrada ?
                                         String(bateria.percentualAtual,0)+"%" : "N/Cal",
                                         corrBat > 0.3f ? "#43a047" :
                                         corrBat < -0.3f ? "#ef6c00" : "#fdd835");
                         c += cardResumo("&#9889;",   "Tensao Banco",
                                         String(bV),
                                         estimTensao.painelAtivo ?
                                         "painel ON" : "repouso/desc.",
                                         "#ce93d8");
                         c += "</div>";

                         // Info configuração sensores
                         c += "<div class='nf' style='font-size:10px'>";
                         c += "<b>Configuracao atual dos sensores:</b><br>";
                         char bufCfg[200];
                         snprintf(bufCfg, sizeof(bufCfg),
                                  "Painel: offset=%dmV sens=%.0fmV/A "
                                  "inv=%s calib=%s<br>"
                                  "Casa: offset=%dmV sens=%.0fmV/A "
                                  "inv=%s calib=%s",
                                  calibPainel.offsetMV,
                                  calibPainel.sensMvA,
                                  calibPainel.invertido ? "SIM" : "NAO",
                                  calibPainel.calibrado ? "SIM" : "NAO",
                                  calibCasa.offsetMV,
                                  calibCasa.sensMvA,
                                  calibCasa.invertido ? "SIM" : "NAO",
                                  calibCasa.calibrado ? "SIM" : "NAO");
                         c += String(bufCfg) + "</div>";

                         // Botão atualizar
                         c += "<button onclick='location.reload()' "
                         "class='btn-s' style='margin-top:10px'>"
                         "&#128259; Atualizar Leituras</button>";

                         server.send(200, "text/html; charset=utf-8",
                                     gerarPagina("RRFS - Sensores", c));
}

// ============================================================
// HANDLERS WEB: OTA UPLOAD
// ============================================================

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

        ledFixo(false);

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            adicionarLog("OTA ERRO: sem espaco suficiente");
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
        static bool ledOTA = false;
        ledOTA = !ledOTA;
        digitalWrite(LED_AZUL, ledOTA);

        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            adicionarLog("OTA ERRO: falha na escrita");
            Update.printError(Serial);
        } else {
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

void handleOTAResult() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    if (Update.hasError()) {
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
        String c = "<div class='ok'>";
        c += "<b>&#10003; Update realizado com sucesso!</b><br>";
        c += "Reiniciando em 3 segundos...<br>";
        c += "Aguarde e recarregue a pagina.";
        c += "</div>";
        c += "<div class='nf'>";
        c += "Apos reiniciar, acesse novamente pelo mesmo IP.<br>";
        c += "Se configurou via WiFi, o IP permanece o mesmo.";
        c += "</div>";

        c += "<script>setTimeout(function(){location.href='/';},8000);</script>";

        server.send(200, "text/html; charset=utf-8",
                    gerarPagina("RRFS - OTA OK", c, false));

        adicionarLog("OTA: reiniciando sistema...");
        delay(3000);
        ESP.restart();
    }
}

// ============================================================
// HANDLERS WEB: PÁGINA DE CONFIGURAÇÃO
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

        if (acao == "reset_parcial") {
            prefs.begin("rrfs", true);
            String wssid = prefs.getString("wifi_ssid", "");
            String wpass = prefs.getString("wifi_pass", "");
            String wip   = prefs.getString("wifi_ip",   "");
            String wgw   = prefs.getString("wifi_gw",   "");
            String wmask = prefs.getString("wifi_mask", "255.255.255.0");
            String wdns  = prefs.getString("wifi_dns",  "8.8.8.8");
            bool   wap   = prefs.getBool  ("usar_ap",   true);
            prefs.end();

            prefs.begin("rrfs", false);
            prefs.clear();
            prefs.putBool  ("usar_ap",   wap);
            prefs.putString("wifi_ssid", wssid);
            prefs.putString("wifi_pass", wpass);
            prefs.putString("wifi_ip",   wip);
            prefs.putString("wifi_gw",   wgw);
            prefs.putString("wifi_mask", wmask);
            prefs.putString("wifi_dns",  wdns);
            prefs.putBool  ("wifi_ok",   true);
            prefs.end();

            prefs.begin("rrfs_tv", false);   prefs.clear(); prefs.end();
            prefs.begin("rrfs_loc", false);  prefs.clear(); prefs.end();
            prefs.begin("rrfs_cnt", false);  prefs.clear(); prefs.end();
            prefs.begin("rrfs_hidx", false); prefs.clear(); prefs.end();

            adicionarLog("RESET PARCIAL: tudo apagado, config de rede mantida");

            String c = "<div class='ok'>&#10003; Reset parcial realizado!</div>";
            c += "<div class='nf'>Historico, contadores e configuracoes apagados.<br>";
            c += "Configuracao de rede mantida.<br>";
            c += "Reiniciando em 3s...</div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - Reset", c, false));
            delay(3000);
            ESP.restart();
            return;
        }

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

    String ip   = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String modo = modoAP ? "Ponto de Acesso (AP)" : "WiFi Cliente";

    String c = msg;

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

    c += "<div class='row'>";
    c += "<button onclick='location.href=\"/setup-wifi\"' class='btn-o'>";
    c += "&#128246; Reconfigurar Rede</button>";
    c += "<button onclick='if(confirm(\"Reiniciar o sistema agora?\"))location.href=\"/reiniciar\"' class='btn-s'>";
    c += "&#128260; Reiniciar</button>";
    c += "</div>";

    c += "<h3>&#128336; Sincronizar Hora (NTP)</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "Fuso horario: UTC-3 (Brasilia). ";
    c += "Sincronizacao automatica diaria ativa.";
    c += "</div>";
    c += "<button onclick='location.href=\"/ntp-sync\"' class='btn-s'>";
    c += "&#128336; Sincronizar Agora</button>";

    c += "<h3>&#128640; Atualizar Firmware (OTA)</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "<b>Como atualizar:</b><br>";
    c += "1. Abra o Arduino IDE<br>";
    c += "2. Menu: <b>Sketch &rarr; Exportar Binario Compilado</b><br>";
    c += "3. Localize o arquivo <b>.bin</b> gerado<br>";
    c += "4. Selecione o arquivo abaixo e clique em Enviar<br>";
    c += "<b style='color:#ef9a9a'>&#9888; Nao feche esta pagina durante o upload!</b>";
    c += "</div>";

    c += "<div id='ota-prog' style='display:none'>";
    c += "<div class='nf' style='text-align:center'>";
    c += "<b id='ota-status'>Enviando...</b><br>";
    c += "<div class='barra-bg' style='margin-top:8px'>";
    c += "<div class='barra' id='ota-barra' style='width:0%;background:#0277bd'>";
    c += "0%</div></div>";
    c += "<span id='ota-bytes' style='font-size:10px;color:#546e7a'></span>";
    c += "</div></div>";

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

    c += "<h3>&#9888; Reset Parcial</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "Apaga usuario, bateria, redes, historico e contadores. <b>Mantem apenas a configuracao de WiFi atual.</b>";
    c += "</div>";
    c += "<form method='POST' action='/config' ";
    c += "onsubmit='return confirm(\"Confirma reset parcial?\\nConfig de rede sera mantida.\")'>";
    c += "<input type='hidden' name='acao' value='reset_parcial'>";
    c += "<button type='submit' class='btn-r'>";
    c += "&#9888; Reset Parcial (Mantem Rede)</button>";
    c += "</form>";

    c += "<button onclick='location.href=\"/logout\"' ";
    c += "class='btn-s' style='margin-top:8px'>";
    c += "&#128275; Sair do Sistema</button>";

    c += F("<script>"
        "document.getElementById('ota-file').addEventListener('change',function(e){"
        "var f=e.target.files[0];"
        "if(!f)return;"
        "var kb=(f.size/1024).toFixed(1);"
        "document.getElementById('ota-fname').textContent=f.name;"
        "document.getElementById('ota-fsize').textContent=kb+' KB';"
        "document.getElementById('ota-info').style.display='block';"
        "var ok=f.name.toLowerCase().endsWith('.bin');"
        "document.getElementById('ota-btn').disabled=!ok;"
        "if(!ok)alert('Selecione um arquivo .bin valido!');"
        "});"

        "document.getElementById('ota-form').addEventListener('submit',function(e){"
        "e.preventDefault();"
        "var f=document.getElementById('ota-file').files[0];"
        "if(!f){alert('Selecione um arquivo .bin');return;}"
        "if(!f.name.toLowerCase().endsWith('.bin')){"
        "alert('Arquivo invalido! Use apenas .bin');return;}"
        "if(!confirm('Confirma o update do firmware?\\n\\nArquivo: '+f.name+'\\nTamanho: '+(f.size/1024).toFixed(1)+' KB'))return;"

        "document.getElementById('ota-btn').disabled=true;"
        "document.getElementById('ota-btn').textContent='Enviando...';"
        "document.getElementById('ota-prog').style.display='block';"
        "document.getElementById('ota-form').style.opacity='0.5';"

        "var fd=new FormData();"
        "fd.append('firmware',f);"
        "var xhr=new XMLHttpRequest();"

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
// HANDLERS WEB: PÁGINA DE HISTÓRICO
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

    c += "<div class='nf' style='font-size:10px;margin-bottom:8px'>";
    c += "&#128336; Acumulando hora " + String(acum.horaAtual) + ":00 &rarr; ";
    char bufAcum[80];
    snprintf(bufAcum, sizeof(bufAcum),
             "P=%.1fWh C=%.1fWh BC=%.1fWh BD=%.1fWh",
             acum.painelWh, acum.casaWh, acum.batCargWh, acum.batDescWh);
    c += String(bufAcum) + "</div>";

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

    c += "<h3>&#128203; Resultados</h3>";

    bool encontrou = false;
    int  diasMostrados = 0;
    int  maxMostrar = (filtroDia > 0) ? HIST_DIAS : 31;

    for (int i = 0; i < histTotalDias && diasMostrados < maxMostrar; i++) {
        int idx = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;

        RegistroDia* pDia = nullptr;
        bool usandoRAM = false;

        if (i == 0) {
            pDia     = &histDiaAtual;
            usandoRAM = true;
        } else {
            if (!carregarDiaFlash(idx, histDiaTemp)) continue;
            pDia = &histDiaTemp;
        }

        if (!pDia->valido) continue;

        if (filtroAno > 0 && pDia->ano != filtroAno) continue;
        if (filtroMes > 0 && pDia->mes != filtroMes) continue;
        if (filtroDia > 0 && pDia->dia != filtroDia) continue;

        encontrou = true;
        diasMostrados++;

        const char* nomeMes[]={"","Jan","Fev","Mar","Abr","Mai","Jun",
            "Jul","Ago","Set","Out","Nov","Dez"};
        char titulo[30];
        snprintf(titulo,sizeof(titulo),"%02d/%s/%04d",
                 pDia->dia, nomeMes[pDia->mes], pDia->ano);

        if (usandoRAM) calcularTotaisDia(histDiaAtual);

        c += "<div style='background:#1a1a1a;border:1px solid #333;"
        "border-radius:8px;padding:10px;margin-bottom:8px'>";

        c += "<div style='display:flex;justify-content:space-between;"
        "align-items:center;margin-bottom:6px;padding-bottom:6px;"
        "border-bottom:1px solid #2a2a2a'>";
        c += "<span style='font-size:13px;font-weight:bold;color:#4fc3f7'>";
        c += "&#128197; " + String(titulo);
        if (usandoRAM) c += " <span style='font-size:9px;color:#43a047'>(hoje)</span>";
        c += "</span>";

        char bufRes[60];
        snprintf(bufRes,sizeof(bufRes),"%.2f / %.2f kWh",
                 pDia->totalPainelWh/1000.0f,
                 pDia->totalCasaWh/1000.0f);
        c += "<span style='font-size:10px;color:#546e7a'>&#9728;"+
        String(bufRes)+"</span>";
        c += "</div>";

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

        c += "</div>";
    }

    if (!encontrou) {
        c += "<div class='nf' style='text-align:center;padding:20px'>"
        "&#128203; Nenhum registro encontrado.<br>"
        "<span style='font-size:10px;color:#546e7a'>"
        "O historico inicia apos a primeira hora completa com NTP ativo."
        "</span></div>";
    }

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
// HANDLERS WEB: PÁGINA DE PREVISÃO
// ============================================================

void handlePrevisao() {
    if (!logado) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "config") {
            String latStr = server.arg("lat");
            String lonStr = server.arg("lon");
            float  kwp    = server.arg("kwp").toFloat();
            float  ef     = server.arg("ef").toFloat();

            double lat = latStr.toDouble();
            double lon = lonStr.toDouble();

            bool ok = true;
            if (lat < -90  || lat > 90)  ok = false;
            if (lon < -180 || lon > 180) ok = false;
            if (kwp < 0.1f || kwp > 100) ok = false;
            if (ef  < 10   || ef  > 100) ok = false;

            if (!ok) {
                msg = "<div class='er'>&#10007; Dados invalidos. "
                "Verifique coordenadas e valores.</div>";
            } else {
                configLocal.latitude      = lat;
                configLocal.longitude     = lon;
                configLocal.painelKWp     = kwp;
                configLocal.eficiencia    = ef / 100.0f;
                configLocal.configurado   = true;
                configLocal.fatorCalib    = 1.0f;
                configLocal.amostrasCalib = 0;
                salvarConfigLocal();

                // Força busca imediata
                ultimaPrev    = 0;
                previsao.valido = false;
                memset(previsao.erro, 0, sizeof(previsao.erro));

                msg = "<div class='ok'>&#10003; Configuracao salva! "
                "Buscando previsao agora...</div>";
                adicionarLog("Previsao: nova config salva - forcando busca");

                // Busca imediata
                buscarPrevisaoMeteo();
            }
        }

        else if (acao == "reset_calib") {
            configLocal.fatorCalib    = 1.0f;
            configLocal.amostrasCalib = 0;
            salvarConfigLocal();
            msg = "<div class='ok'>&#10003; Calibracao resetada</div>";
            adicionarLog("Previsao: calibracao resetada");
        }

        else if (acao == "atualizar") {
            ultimaPrev      = 0;
            previsao.valido = false;
            memset(previsao.erro, 0, sizeof(previsao.erro));
            adicionarLog("Previsao: atualizacao forcada pelo usuario");
            buscarPrevisaoMeteo();
            msg = "<div class='ok'>&#9679; Previsao atualizada!</div>";
        }
    }

    String c = msg;

    c += "<h3>&#127774; Previsao Solar</h3>";

    // ══════════════════════════════════════════════════════════
    // BLOCO DE STATUS - CORRIGIDO COM DEBUG
    // ══════════════════════════════════════════════════════════

    if (!configLocal.configurado) {
        c += "<div class='wn'>&#9888; Configure a localizacao "
        "abaixo para ativar a previsao.</div>";

    } else if (!previsao.valido) {

        c += "<div class='wn' style='text-align:center;"
        "padding:15px'>"
        "&#128336; Aguardando previsao...</div>";

        // Debug detalhado
        c += "<div class='nf' style='font-size:11px'>";
        c += "<b>&#128270; Status do sistema:</b><br><br>";

        // Status WiFi
        bool wifiOK = (WiFi.status() == WL_CONNECTED);
        c += "&#128246; WiFi: <b>";
        if (wifiOK) {
            c += "<span style='color:#43a047'>Conectado";
            c += " (" + WiFi.SSID() + " ";
            c += String(WiFi.RSSI()) + "dBm)</span>";
        } else {
            c += "<span style='color:#ef5350'>DESCONECTADO!"
            " Necessario para previsao.</span>";
        }
        c += "</b><br>";

        // Status NTP
        struct tm tiDbg;
        bool ntpOK = getLocalTime(&tiDbg);
        c += "&#128336; NTP/Hora: <b>";
        if (ntpOK) {
            char bufHora[30];
            snprintf(bufHora, sizeof(bufHora),
                     "%02d:%02d:%02d %02d/%02d/%04d",
                     tiDbg.tm_hour, tiDbg.tm_min,   tiDbg.tm_sec,
                     tiDbg.tm_mday, tiDbg.tm_mon+1,
                     tiDbg.tm_year + 1900);
            c += "<span style='color:#43a047'>";
            c += String(bufHora) + "</span>";
        } else {
            c += "<span style='color:#ef5350'>NAO sincronizado!"
            " Necessario para previsao.</span>";
        }
        c += "</b><br>";

        // Coordenadas
        char bufCoord[80];
        snprintf(bufCoord, sizeof(bufCoord),
                 "%.6f, %.6f",
                 (float)configLocal.latitude,
                 (float)configLocal.longitude);
        c += "&#128205; Coordenadas: <b style='color:#90a4ae'>";
        c += String(bufCoord) + "</b><br>";

        // kWp e eficiência
        char bufKwp[50];
        snprintf(bufKwp, sizeof(bufKwp),
                 "%.2f kWp | Eficiencia: %.0f%%",
                 configLocal.painelKWp,
                 configLocal.eficiencia * 100.0f);
        c += "&#9728; Painel: <b style='color:#90a4ae'>";
        c += String(bufKwp) + "</b><br>";

        // RAM livre
        snprintf(tempBuf, sizeof(tempBuf),
                 "%.1f KB", esp_get_free_heap_size() / 1024.0f);
        c += "&#128190; RAM livre: <b style='color:#90a4ae'>";
        c += String(tempBuf) + "</b><br>";

        // Último erro
        if (strlen(previsao.erro) > 0) {
            c += "<br>&#10007; Ultimo erro: "
            "<b style='color:#ef9a9a'>";
            c += String(previsao.erro);
            c += "</b><br>";
        }

        // Tempo desde última tentativa
        if (ultimaPrev > 0) {
            unsigned long segsAtras = (millis() - ultimaPrev) / 1000;
            snprintf(tempBuf, sizeof(tempBuf),
                     "&#9201; Ultima tentativa: <b>%lus atras</b><br>",
                     segsAtras);
            c += String(tempBuf);

            unsigned long proxMs = PREV_INTERVALO -
            (millis() - ultimaPrev);
            if (proxMs < PREV_INTERVALO) {
                snprintf(tempBuf, sizeof(tempBuf),
                         "&#9201; Proxima tentativa em: "
                         "<b>%lu min</b><br>",
                         proxMs / 60000);
                c += String(tempBuf);
            }
        } else {
            c += "&#9201; Nenhuma tentativa ainda.<br>";
        }

        c += "</div>";

        // Botões de ação
        if (!wifiOK) {
            c += "<div class='er' style='font-size:11px'>"
            "&#9888; Conecte o ESP32 ao WiFi para "
            "buscar a previsao.</div>";
        } else if (!ntpOK) {
            c += "<div class='er' style='font-size:11px'>"
            "&#9888; Hora nao sincronizada. "
            "<a href='/ntp-sync' style='color:#ef9a9a'>"
            "Sincronizar NTP agora</a></div>";
        } else {
            c += "<form method='POST' action='/previsao' "
            "style='margin-top:8px'>";
            c += "<input type='hidden' name='acao' "
            "value='atualizar'>";
            c += "<button type='submit' class='btn-o'>"
            "&#128260; Buscar Previsao Agora</button>";
            c += "</form>";
        }

    } else {

        // ══════════════════════════════════════════════════════
        // PREVISÃO VÁLIDA - MOSTRA DADOS
        // ══════════════════════════════════════════════════════

        unsigned long msPassados = millis() - previsao.ultimaAtualizacao;
        unsigned long minPassados = msPassados / 60000;

        c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>";
        if (minPassados < 60) {
            snprintf(tempBuf, sizeof(tempBuf),
                     "&#128336; Atualizado: %lu min atras",
                     minPassados);
        } else {
            snprintf(tempBuf, sizeof(tempBuf),
                     "&#128336; Atualizado: %luh%02lum atras",
                     minPassados / 60, minPassados % 60);
        }
        c += String(tempBuf);

        unsigned long proxMs = PREV_INTERVALO -
        (millis() - ultimaPrev);
        if (proxMs < PREV_INTERVALO) {
            snprintf(tempBuf, sizeof(tempBuf),
                     " | Proximo em %lu min",
                     proxMs / 60000);
            c += String(tempBuf);
        }
        c += "</div>";

        // Cards resumo
        c += "<div style='display:grid;grid-template-columns:1fr 1fr;"
        "gap:6px;margin-bottom:8px'>";

        auto cardPrev = [](const String& ico,
                           const String& titulo,
                           const String& valor,
                           const String& sub,
                           const String& cor) -> String {
                               String h = "<div style='background:#1a1a1a;"
                               "border:2px solid #1e1e1e;"
                               "border-radius:8px;padding:10px;"
                               "text-align:center'>";
                               h += "<div style='font-size:10px;color:#546e7a'>";
                               h += ico + " " + titulo + "</div>";
                               h += "<div style='font-size:17px;font-weight:bold;"
                               "color:" + cor + ";margin-top:4px'>";
                               h += valor + "</div>";
                               h += "<div style='font-size:10px;color:#546e7a;"
                               "margin-top:2px'>" + sub + "</div>";
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

                           c += cardPrev("&#9728;",   "Geracao Hoje (rest.)",
                                         String(bH),  "ate anoitecer",   "#fdd835");
                           c += cardPrev("&#127773;", "Geracao Amanha",
                                         String(bA),  "dia todo",         "#4fc3f7");
                           c += cardPrev("&#128267;", "Bateria Atual",
                                         String(bateria.percentualAtual, 0) + "%",
                                         String(bateria.energiaAtual, 0) + "Wh",
                                         "#43a047");
                           c += cardPrev("&#128200;", "Total 48h",
                                         String(bT),  "geracao estimada", "#ef6c00");
                           c += "</div>";

                           // ── Decisões automáticas ───────────────────────────────
                           c += "<h3>&#129504; Decisoes Automaticas</h3>";

                           if (decisao.alertaEnergiaInsuficiente) {
                               c += "<div class='er'>"
                               "&#9888; ALERTA: Energia pode ser insuficiente "
                               "para a noite! Geracao amanha tambem baixa.</div>";
                           }

                           c += "<div style='background:#111;border:1px solid #1e1e1e;"
                           "border-radius:8px;padding:10px;margin-bottom:8px'>";

                           {
                               String cor = decisao.desligarRedeAltaNoite ?
                               "#ef6c00" : "#43a047";
                               String st  = decisao.desligarRedeAltaNoite ?
                               "Desligada (economia)" : "Normal";
                               c += "<div style='display:flex;justify-content:"
                               "space-between;align-items:center;padding:6px 0;"
                               "border-bottom:1px solid #1a1a1a'>";
                               c += "<span style='font-size:12px;color:#90a4ae'>"
                               "&#128268; Rede Alta</span>";
                               c += "<span style='font-size:11px;font-weight:bold;"
                               "color:" + cor + "'>" + st + "</span></div>";
                           }

                           {
                               String cor = decisao.desligarLampExtNoite ?
                               "#ef6c00" : "#43a047";
                               String st  = sensores.correntePainel > 0.5f ?
                               "Desligada (dia)" :
                               decisao.desligarLampExtNoite ?
                               "Desligada (economia)" : "Normal";
                               c += "<div style='display:flex;justify-content:"
                               "space-between;align-items:center;padding:6px 0;"
                               "border-bottom:1px solid #1a1a1a'>";
                               c += "<span style='font-size:12px;color:#90a4ae'>"
                               "&#128161; Lamp. Externa</span>";
                               c += "<span style='font-size:11px;font-weight:bold;"
                               "color:" + cor + "'>" + st + "</span></div>";
                           }

                           if (strlen(decisao.motivo) > 0) {
                               c += "<div style='font-size:10px;color:#546e7a;"
                               "margin-top:6px'>&#128220; ";
                               c += String(decisao.motivo) + "</div>";
                           }
                           c += "</div>";

                           // ── Calibração automática ──────────────────────────────
                           c += "<h3>&#127919; Calibracao Automatica</h3>";
                           c += "<div style='background:#111;border:1px solid #1e1e1e;"
                           "border-radius:8px;padding:10px;margin-bottom:8px'>";

                           float precisao = min(100.0f,
                                                (float)configLocal.amostrasCalib /
                                                50.0f * 100.0f);
                           String corPrec = precisao > 60 ? "#43a047" :
                           precisao > 30 ? "#ef6c00" : "#c62828";

                           c += "<div style='display:flex;justify-content:space-between;"
                           "margin-bottom:4px'>";
                           c += "<span style='font-size:11px;color:#78909c'>"
                           "Precisao da previsao</span>";
                           c += "<span style='font-size:11px;font-weight:bold;color:"
                           + corPrec + "'>";
                           c += String((int)precisao) + "%</span></div>";

                           c += "<div class='barra-bg'><div class='barra' style='width:";
                           c += String((int)precisao) + "%;background:";
                           c += corPrec + "'>";
                           c += String((int)precisao) + "%</div></div>";

                           char bufCalib[80];
                           snprintf(bufCalib, sizeof(bufCalib),
                                    "Fator: %.3f | Amostras: %d/50",
                                    configLocal.fatorCalib,
                                    configLocal.amostrasCalib);
                           c += "<div style='font-size:10px;color:#546e7a;"
                           "margin-top:4px'>";
                           c += String(bufCalib) + "</div>";

                           if (configLocal.amostrasCalib < 10) {
                               c += "<div style='font-size:10px;color:#78909c;"
                               "margin-top:4px'>&#128336; Calibracao em andamento."
                               " Melhora ao longo dos dias.</div>";
                           } else if (configLocal.amostrasCalib < 30) {
                               c += "<div style='font-size:10px;color:#ef6c00;"
                               "margin-top:4px'>&#127919; Calibracao parcial."
                               " Continue usando para maior precisao.</div>";
                           } else {
                               c += "<div style='font-size:10px;color:#43a047;"
                               "margin-top:4px'>&#10003; Boa precisao!"
                               " Sistema bem calibrado.</div>";
                           }
                           c += "</div>";

                           // Botões
                           c += "<form method='POST' action='/previsao'>";
                           c += "<input type='hidden' name='acao' value='atualizar'>";
                           c += "<button type='submit' class='btn-s'>"
                           "&#128260; Forcar Atualizacao</button>";
                           c += "</form>";

                           c += "<form method='POST' action='/previsao' "
                           "style='margin-top:6px' "
                           "onsubmit='return confirm(\"Resetar calibracao?\")'>";
                           c += "<input type='hidden' name='acao' value='reset_calib'>";
                           c += "<button type='submit' class='btn-r btn-sm'>"
                           "&#8635; Resetar Calibracao</button>";
                           c += "</form>";

                           // ── Tabela previsão por hora ───────────────────────────
                           c += "<h3>&#128336; Previsao por Hora</h3>";

                           struct tm ti2;
                           getLocalTime(&ti2);
                           int diaAnterior = -1;

                           c += "<div style='overflow-x:auto'>";
                           c += "<table style='width:100%;border-collapse:collapse;"
                           "font-size:10px'>";
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

                               // Separador de dia
                               if ((int)ph.dia != diaAnterior) {
                                   diaAnterior = ph.dia;
                                   const char* nd[] = {"","Jan","Fev","Mar","Abr",
                                       "Mai","Jun","Jul","Ago",
                                       "Set","Out","Nov","Dez"};
                                       char dtBuf[24];
                                       snprintf(dtBuf, sizeof(dtBuf), "%02d/%s/%04d",
                                                ph.dia, nd[ph.mes], ph.ano);

                                       c += "<tr><td colspan='5' style='padding:6px;"
                                       "background:#1a1a1a;color:#4fc3f7;"
                                       "font-weight:bold;font-size:11px'>";
                                       c += "&#128197; " + String(dtBuf);

                                       if (ph.dia  == ti2.tm_mday &&
                                           ph.mes  == (uint8_t)(ti2.tm_mon + 1))
                                           c += " <span style='color:#43a047'>"
                                           "(hoje)</span>";
                                       else
                                           c += " <span style='color:#546e7a'>"
                                           "(amanha)</span>";
                                       c += "</td></tr>";
                               }

                               // Linha hora atual
                               bool eAgora = (ph.dia  == (uint8_t)ti2.tm_mday &&
                               ph.mes  == (uint8_t)(ti2.tm_mon+1) &&
                               ph.hora == (uint8_t)ti2.tm_hour);

                               c += "<tr style='border-bottom:1px solid #1a1a1a;";
                               if (eAgora) c += "background:#0d1f0d;";
                               c += "'>";

                               // Hora
                               c += "<td style='padding:4px 6px;color:#90a4ae;"
                               "text-align:center'>";
                               snprintf(tempBuf, sizeof(tempBuf),
                                        "%02d:00", ph.hora);
                               c += String(tempBuf);
                               if (eAgora) c += " &#9654;";
                               c += "</td>";

                               // Radiação
                               char bR[16];
                               snprintf(bR, sizeof(bR), "%.0f W/m2",
                                        ph.radiacao + ph.radiacaoDifusa);
                               c += "<td style='padding:4px 6px;text-align:right;"
                               "color:";
                               c += (ph.radiacao > 100) ? "#fdd835" : "#455a64";
                               c += "'>" + String(bR) + "</td>";

                               // Nuvens
                               char bN[8];
                               snprintf(bN, sizeof(bN), "%.0f%%",
                                        ph.coberturaNuvens);
                               c += "<td style='padding:4px 6px;text-align:right;"
                               "color:";
                               c += ph.coberturaNuvens > 70 ? "#78909c" :
                               ph.coberturaNuvens > 30 ? "#90a4ae" : "#4fc3f7";
                               c += "'>" + String(bN) + "</td>";

                               // Temperatura
                               char bT2[8];
                               snprintf(bT2, sizeof(bT2), "%.0f C", ph.temperatura);
                               c += "<td style='padding:4px 6px;text-align:right;"
                               "color:#ef5350'>" + String(bT2) + "</td>";

                               // Geração estimada
                               char bG[16];
                               if (ph.geracaoEstimada >= 1000)
                                   snprintf(bG, sizeof(bG), "%.2f kWh",
                                            ph.geracaoEstimada / 1000.0f);
                                   else
                                       snprintf(bG, sizeof(bG), "%.0f Wh",
                                                ph.geracaoEstimada);

                                       c += "<td style='padding:4px 6px;text-align:right;"
                                       "font-weight:bold;color:";
                                   c += ph.geracaoEstimada > 500 ? "#43a047" :
                                   ph.geracaoEstimada > 100 ? "#fdd835" : "#455a64";
                               c += "'>";
                               c += ph.geracaoEstimada > 0 ? String(bG) : "-";
                               c += "</td></tr>";
                           }
                           c += "</table></div>";
    }

    // ══════════════════════════════════════════════════════════
    // FORMULÁRIO DE CONFIGURAÇÃO (sempre visível)
    // ══════════════════════════════════════════════════════════

    c += "<h3>&#9881; Configurar Localizacao</h3>";

    c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>";
    c += "&#128205; Como obter coordenadas precisas:<br>";
    c += "1. Abra o Google Maps abaixo<br>";
    c += "2. Localize sua instalacao<br>";
    c += "3. Clique com botao direito no local<br>";
    c += "4. Copie as coordenadas (ex: -23.667768, -48.184803)<br>";
    c += "5. Cole nos campos abaixo";
    c += "</div>";

    c += "<a href='https://www.google.com/maps' target='_blank' "
    "style='display:block;padding:10px;background:#1565c0;"
    "color:#fff;text-align:center;border-radius:6px;"
    "text-decoration:none;font-weight:bold;"
    "margin-bottom:10px'>"
    "&#127759; Abrir Google Maps</a>";

    c += "<form method='POST' action='/previsao'>";
    c += "<input type='hidden' name='acao' value='config'>";

    char bufLat[30], bufLon[30];
    snprintf(bufLat, sizeof(bufLat), "%.8f",
             (float)configLocal.latitude);
    snprintf(bufLon, sizeof(bufLon), "%.8f",
             (float)configLocal.longitude);

    c += "<label>Latitude:</label>";
    c += "<input type='text' name='lat' required "
    "placeholder='-23.667768' ";
    if (configLocal.configurado)
        c += "value='" + String(bufLat) + "'";
    c += ">";

    c += "<label>Longitude:</label>";
    c += "<input type='text' name='lon' required "
    "placeholder='-48.184803' ";
    if (configLocal.configurado)
        c += "value='" + String(bufLon) + "'";
    c += ">";

    c += "<label>Potencia dos Paineis (kWp):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Ex: 2 paineis 550W = 1.1 kWp</div>";
    c += "<input type='number' name='kwp' step='0.01' "
    "min='0.1' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf),
             "value='%.2f'", configLocal.painelKWp);
    c += String(tempBuf) + " placeholder='Ex: 1.1'>";

    c += "<label>Eficiencia do Sistema (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Tipico: 75-85%</div>";
    c += "<input type='number' name='ef' step='1' "
    "min='10' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf),
             "value='%.0f'",
             configLocal.eficiencia * 100.0f);
    c += String(tempBuf) + " placeholder='Ex: 80'>";

    c += "<button type='submit' class='btn-g'>"
    "&#128190; Salvar e Buscar Previsao</button>";
    c += "</form>";

    c += "<div class='nf' style='font-size:10px;margin-top:8px'>";
    c += "&#127760; API: Open-Meteo (gratuito, sem chave)<br>";
    c += "&#128336; Atualizacao automatica a cada 1 hora<br>";
    c += "&#127919; Precisao melhora automaticamente";
    c += "</div>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Previsao", c));
}

// ============================================================
// HANDLERS WEB: ENDPOINTS AUXILIARES
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
// CONFIGURAÇÃO DE ROTAS
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
    server.on("/historico",   handleHistorico);
    server.on("/previsao",    handlePrevisao);

    server.on("/ota-upload", HTTP_POST,
              handleOTAResult,
              handleOTAUpload
    );

    server.onNotFound(handleNotFound);
    server.begin();
}

// ============================================================
// SETUP - INICIALIZAÇÃO DO SISTEMA
// ============================================================
void setup() {
    // ── Verifica e repara NVS corrompida (proteção contra falhas de boot) ──
    esp_err_t errNVS = nvs_flash_init();
    if (errNVS == ESP_ERR_NVS_NO_FREE_PAGES ||
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
    errNVS = nvs_flash_init();
        }

        // ── Detecta se o reset anterior foi causado por brownout ───
        esp_reset_reason_t motivoReset = esp_reset_reason();
        if (motivoReset == ESP_RST_BROWNOUT) {
            adicionarLog("!!! RESET ANTERIOR: BROWNOUT DETECTADO !!!");
            adicionarLog("Queda de tensao na alimentacao do ESP32.");
            adicionarLog("Verifique: fonte, capacitor de desacoplamento,");
            adicionarLog("proximidade da fiacao do rele/inversor.");
        } else if (motivoReset == ESP_RST_PANIC) {
            adicionarLog("!!! RESET ANTERIOR: PANIC/CRASH DO FIRMWARE !!!");
        } else if (motivoReset == ESP_RST_TASK_WDT) {
            adicionarLog("!!! RESET ANTERIOR: WATCHDOG (codigo travado) !!!");
        }

        // Inicializa LED e botão
        setupLED();
        setupBotao();
        setupReles();
        ledBlink();

        adicionarLog("RRFS Energia v11.0");
        if (errNVS != ESP_OK) {
            adicionarLog("AVISO: NVS reparada apos corrupcao detectada");
        }
        logRAM();

        // ── Inicializa calibração sensores com valores de fábrica ──
        // PAINEL: ACS758 50A Bidirecional
        // Sinal DIRETO no ESP32 | Cabos INVERTIDOS (proteção)
        // Painel gerando → tensão DESCE (seguro para ESP32)
        // Corrente reversa → tensão SOBE (aviso automático)
        calibPainel.offsetMV  = PAINEL_OFFSET_MV;  // 2500mV (VCC/2)
        calibPainel.sensMvA   = PAINEL_SENS_MV_A;  // 40mV/A (datasheet)
        calibPainel.invertido = true;               // Cabos invertidos propositalmente
        calibPainel.calibrado = true;               // Calibrado de fábrica

        // CASA: ACS758LCB-100B Unidirecional
        // Sinal DIRETO no ESP32 | Offset fixo 600mV
        // Sinal máximo (100A): 600 + 100×20 = 2600mV (seguro no ESP32)
        calibCasa.offsetMV  = CASA_OFFSET_MV;      // 600mV (fixo fabricante)
        calibCasa.sensMvA   = CASA_SENS_MV_A;      // 20mV/A (datasheet)
        calibCasa.invertido = false;                // Unidirecional - sem inversão
        calibCasa.calibrado = true;                 // Calibrado de fábrica

        // ── Inicializa estimativa de tensão ────────────────────────
        estimTensao.fatorCalibTensao = 1.0f;
        estimTensao.amostrasCalib    = 0;
        estimTensao.tensaoAtual      = 12.8f;
        estimTensao.tensaoMaxima     = 14.0f;
        estimTensao.tensaoNominalReal= 12.8f;
        estimTensao.fatorBanco       = 4.0f;
        estimTensao.painelAtivo      = false;

        // ── Carrega configurações da flash ─────────────────────────
        carregarConfigBateria();
        carregarConfigRedes();
        carregarContadores();
        carregarHistorico();
        adicionarLog("Historico e contadores carregados");

        // Carrega valores do datasheet para os sensores
        carregarCalibSensores();

        // Carrega calibração de tensão aprendida
        carregarCalibTensao();

        // Calcula parâmetros do banco de baterias
        calcularBateria();

        snprintf(tempBuf, sizeof(tempBuf),
                 "Banco LiFePO4: %.0fS | Vmax=%.2fV | Vnom=%.2fV | Cal=%.4f",
                 estimTensao.fatorBanco,
                 estimTensao.tensaoMaxima,
                 estimTensao.tensaoNominalReal,
                 estimTensao.fatorCalibTensao);
        adicionarLog(tempBuf);

        ultimoContagem = millis();

        // ── Estimativa inicial de tensão baseada no SOC salvo ──────
        if (bateria.calibrada && bateria.energiaDisponivel > 0) {
            float socInicial = bateria.percentualAtual;
            estimTensao.tensaoAtual = soc2tensaoLiFePO4(socInicial)
            * estimTensao.fatorBanco
            * estimTensao.fatorCalibTensao;
            snprintf(tempBuf, sizeof(tempBuf),
                     "Tensao inicial estimada: %.2fV (SOC=%.0f%%)",
                     estimTensao.tensaoAtual, socInicial);
            adicionarLog(tempBuf);
        }

        // ── Carrega configuração local (previsão meteorológica) ────
        carregarConfigLocal();

        // ── Log resumo da configuração ─────────────────────────────
        adicionarLog("Config carregada");
        snprintf(tempBuf, sizeof(tempBuf),
                 "Bat: %.1fV %.0fAh %.0f%% | Calib:%s | Tensao:%.2fV",
                 bateria.tensaoNominal,
                 bateria.capacidadeAh,
                 bateria.descargaSegura,
                 bateria.calibrada ? "SIM" : "NAO",
                 estimTensao.tensaoAtual);
        adicionarLog(tempBuf);

        snprintf(tempBuf, sizeof(tempBuf),
                 "Sensor Painel: off=%dmV sens=%.0fmV/A inv=SIM",
                 calibPainel.offsetMV, calibPainel.sensMvA);
        adicionarLog(tempBuf);

        snprintf(tempBuf, sizeof(tempBuf),
                 "Sensor Casa: off=%dmV sens=%.0fmV/A",
                 calibCasa.offsetMV, calibCasa.sensMvA);
        adicionarLog(tempBuf);

        // ══════════════════════════════════════════════════════════
        // SEQUÊNCIA DE INICIALIZAÇÃO DAS CARGAS
        // ══════════════════════════════════════════════════════════

        // 1. Liga rede baixa primeiro (alimenta roteador WiFi)
        ligarRedeBaixa();
        adicionarLog("Rede baixa ON (roteador)");

        // 2. Aguarda roteador inicializar
        adicionarLog("Aguardando roteador 8s...");
        unsigned long tRoteador = millis();
        while (millis() - tRoteador < 8000) {
            atualizarLED();
            delay(10);
        }
        adicionarLog("Roteador pronto");

        // 3. Liga inversor
        ligarInversor();
        adicionarLog("Inversor ON");

        // 4. Aguarda inversor inicializar
        adicionarLog("Aguardando inversor 5s...");
        unsigned long tInversor = millis();
        while (millis() - tInversor < 5000) {
            atualizarLED();
            delay(10);
        }

        // 5. Verifica status do inversor
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
        } else {
            adicionarLog("AVISO: Inversor sem status - continuando mesmo assim");
        }

        // 6. Liga rede alta
        ligarRedeAlta();
        adicionarLog("Rede alta ON");
        delay(500);

        // 7. Liga lâmpadas
        ligarLampExt();
        adicionarLog("Lamp. externa ON");
        delay(200);

        ligarLampInt();
        adicionarLog("Lamp. interna ON");

        logRAM();

        // ══════════════════════════════════════════════════════════
        // CONFIGURAÇÃO DE REDE WIFI
        // ══════════════════════════════════════════════════════════
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

        // ── Inicia servidor web ────────────────────────────────────
        configurarRotas();
        adicionarLog("Servidor web ATIVO");

        // ── Leitura inicial dos sensores ───────────────────────────
        lerSensores();

        estimTensao.tensaoAtual = estimarTensaoBateria();
        snprintf(tempBuf, sizeof(tempBuf),
                 "Tensao apos 1a leitura: %.2fV | Painel:%s",
                 estimTensao.tensaoAtual,
                 estimTensao.painelAtivo ? "ON" : "OFF");
        adicionarLog(tempBuf);

        // ── Inicializa histórico de tensão (gráfico 60s) ───────────
        for (int i = 0; i < HIST_TENSAO_PONTOS; i++) {
            histTensaoV[i]    = estimTensao.tensaoAtual;
            histCorrPainel[i] = 0;
            histCorrCasa[i]   = 0;
            histCorrBat[i]    = 0;
        }
        ultimoHistTensao = millis();

        // ── Inicializa monitoramento de curto ──────────────────────
        curto.monitorando       = false;
        curto.estado            = CURTO_NORMAL;
        curto.subEstado         = SUB_IDLE;
        curto.tInicio           = 0;
        curto.tentativasReligar = 0;
        curto.redeAltaCurto     = false;
        curto.redeBaixaCurto    = false;

        adicionarLog("Sistema pronto!");
        logRAM();
}

// ============================================================
// LOOP - CICLO PRINCIPAL DO SISTEMA
// ============================================================

void loop() {
    verificarReset();
    atualizarLED();

    if (modoAP) dnsServer.processNextRequest();
    server.handleClient();

    unsigned long agora = millis();

    // Leitura de sensores a cada 1s
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;
        lerSensores();
        acumularEnergia();
    }

    // Monitoramento de curto
    bool emProcessoCurto = (curto.estado == CURTO_RELIGANDO      ||
    curto.estado == CURTO_TESTANDO_ALTA  ||
    curto.estado == CURTO_TESTANDO_BAIXA);

    if (estados.inversor || emProcessoCurto) {
        monitorarInversor();
    } else {
        if (curto.estado == CURTO_NORMAL && curto.monitorando) {
            curto.monitorando       = false;
            curto.tInicio           = 0;
            curto.tentativasReligar = 0;
        }
    }

    // Controle de energia a cada 5s
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;

        if (!emProcessoCurto) {
            if (curto.estado == CURTO_NORMAL ||
                curto.estado == CURTO_CONFIRMADO_ALTA) {
                controlarEnergia();
                }
        }
    }

    // ★ Processa a restauração de energia em TODA iteração,
    // sem bloquear o loop, mantendo web/WiFi sempre responsivos
    processarRestauroEnergia();

    // Gerenciamento WiFi
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
                adicionarLog("WiFi restaurado - desativando AP temporario");

                IPAddress lIP, lGW, lMask, lDNS;
                lIP.fromString(gWifiIP);
                lGW.fromString(gWifiGW);
                lMask.fromString(gWifiMASK);
                lDNS.fromString(gWifiDNS);
                WiFi.config(lIP, lGW, lMask, lDNS);

                WiFi.mode(WIFI_STA);
                WiFi.setSleep(false);

                modoAP    = false;
                eraAP_STA = false;
                ledFixo(true);

                server.close();
                delay(200);
                configurarRotas();

                snprintf(tempBuf, sizeof(tempBuf),
                         "WiFi OK: %s RSSI: %ddBm",
                         WiFi.SSID().c_str(), WiFi.RSSI());
                adicionarLog(tempBuf);

                sincronizarNTP();
            }

            if (modoAP && WiFi.getMode() == WIFI_AP_STA) {
                eraAP_STA = true;
            }

            static unsigned long tNTPResinc = 0;
            if (agora - tNTPResinc >= 86400000UL) {
                tNTPResinc = agora;
                adicionarLog("NTP: resincronizacao diaria");
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

    // Salva energia da bateria a cada 5min
    static unsigned long tSalva = 0;
    if (bateria.calibrada && agora - tSalva >= 300000) {
        tSalva = agora;
        prefs.begin("rrfs", false);
        prefs.putFloat("bat_nrg", bateria.energiaAtual);
        prefs.end();
    }

    // Previsão meteorológica a cada 1 hora
    if (configLocal.configurado        &&
        WiFi.status() == WL_CONNECTED  &&
        !buscandoPrev                  &&
        (ultimaPrev == 0               ||
        millis() - ultimaPrev >= PREV_INTERVALO)) {

        buscarPrevisaoMeteo();
        }

        // Calibração automática e decisões a cada 5 minutos
        // (tomarDecisoes() roda aqui tambem, o dia todo - antes so era
        // chamada 1x por hora, dentro de buscarPrevisaoMeteo())
        static unsigned long tCalib = 0;
        if (millis() - tCalib >= 300000) {
            tCalib = millis();
            calibrarPrevisao();
            verificarReligamentoAuto();
            tomarDecisoes();
        }

        // Avaliação da antecipação inteligente da Rede Alta, a cada 10s
        // (ela mesma controla seus proprios tempos internos de
        // espera/estabilizacao/reavaliacao)
        static unsigned long tAntecip = 0;
        if (millis() - tAntecip >= 10000) {
            tAntecip = millis();
            avaliarAntecipacaoRedeAlta();
        }

        // Log RAM a cada 10min
        static unsigned long tRAM = 0;
        if (agora - tRAM >= 600000) {
            tRAM = agora;
            logRAM();
        }

        delay(1);
}
