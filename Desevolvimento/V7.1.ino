/*
 * ============================================================
 * RRFS ENERGIA - SISTEMA COMPLETO DE GERENCIAMENTO DE ENERGIA SOLAR
 * ============================================================
 * Versão: v7.0
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
#define ADC_SAMPLES        50     // Amostras para média de cada leitura
#define ADC_VREF         3300     // mV - Tensão de referência ESP32
#define ADC_BITS         4095     // Resolução 12 bits
#define SENSOR_VCC       5000     // mV - Alimentação dos sensores ACS758

// ── SENSOR ACS758 50A (PAINEL) ─────────────────────────────
// Sensibilidade: 40mV/A com VCC=5V
// Offset (0A): VCC/2 = 2500mV
// Com divisor resistivo 5V→3.3V: offset lido = ~1650mV
#define PAINEL_SENS_MV_A   40     // mV por Ampere (datasheet)
#define PAINEL_OFFSET_MV 2500     // Offset com VCC 5V (ponto zero)

// ── SENSOR ACS758 100A (CASA) ──────────────────────────────
// Sensibilidade: 20mV/A com VCC=5V
// Offset (0A): VCC/2 = 2500mV
#define CASA_SENS_MV_A     20     // mV por Ampere (datasheet)
#define CASA_OFFSET_MV   2500     // Offset com VCC 5V (ponto zero)

// ── CALIBRAÇÃO ADC ─────────────────────────────────────────
#define OFFSET_LIDO_MV  1650      // Offset padrão lido pelo ESP32 após divisor
#define FILTRO_RUIDO_A    0.3f    // Corrente abaixo disso = zero
#define BUFFER_CORRENTE   10      // Tamanho buffer média móvel

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
    float horaDesligarRedeAlta;         // Hora prevista para desligar
    float horaLigarRedeAlta;            // Hora prevista para religar
    bool  alertaEnergiaInsuficiente;    // Alerta de energia insuficiente?
    char  motivo[128];                  // Motivo da decisão
};

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
DecisaoSistema   decisao      = {false, false, 0, 0, false, ""};

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
void salvarCalibSensores();
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
// SISTEMA: CONTROLE DE RELÉS
// ============================================================

// ── Inicializa todos os relés ──────────────────────────────
void setupReles() {
    // Todos os relés iniciam DESLIGADOS no hardware
    pinMode(PIN_RELE_INV_ALTA,     OUTPUT); digitalWrite(PIN_RELE_INV_ALTA,     HIGH);
    pinMode(PIN_RELE_REDE_BAIXA,   OUTPUT); digitalWrite(PIN_RELE_REDE_BAIXA,   HIGH);
    pinMode(PIN_RELE_REDE_ALTA,    OUTPUT); digitalWrite(PIN_RELE_REDE_ALTA,    HIGH);
    pinMode(PIN_RELE_LAMP_EXTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_EXTERNA, HIGH);
    pinMode(PIN_RELE_LAMP_INTERNA, OUTPUT); digitalWrite(PIN_RELE_LAMP_INTERNA, HIGH);

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
    digitalWrite(pin, ligar ? LOW : HIGH);
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

    // Filtro de ruído DEPOIS da média
    if (corrMediaMovel > -FILTRO_RUIDO_A && corrMediaMovel < FILTRO_RUIDO_A) {
        corrMediaMovel = 0.0f;
    }

    return corrMediaMovel;
}

// ── Lê corrente da casa ────────────────────────────────────
float lerCorrenteCasa() {
    int   adc = lerADC(PIN_SENSOR_CASA);
    int   mv  = adcParaMv(adc);
    sensores.sinalCasa_mV = mv;

    // Corrente bruta usando calibração
    float correnteBruta = (float)(mv - calibCasa.offsetMV) / calibCasa.sensMvA;

    // Inverte sinal se necessário
    if (calibCasa.invertido) correnteBruta = -correnteBruta;

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

    // Aviso cabos invertidos
    if (sensores.correntePainel < -2.0f) {
        adicionarLog("AVISO: Sensor painel pode estar invertido!");
    }

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

    buscandoPrev = true;
    adicionarLog("Prev: buscando dados Open-Meteo...");

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
    http.setTimeout(10000);

    int httpCode = http.GET();

    if (httpCode != 200) {
        snprintf(previsao.erro, sizeof(previsao.erro),
                 "HTTP erro: %d", httpCode);
        adicionarLog(previsao.erro);
        http.end();
        buscandoPrev = false;
        return;
    }

    String payload = http.getString();
    http.end();

    if (payload.length() < 100) {
        snprintf(previsao.erro, sizeof(previsao.erro), "Resposta vazia");
        adicionarLog("Prev: resposta vazia");
        buscandoPrev = false;
        return;
    }

    // Parser JSON manual
    struct tm ti;
    if (!getLocalTime(&ti)) {
        adicionarLog("Prev: sem NTP para parsear");
        buscandoPrev = false;
        return;
    }

    auto extrairArray = [&](const char* chave, float* arr, int maxLen) -> int {
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
                arr[count++] = payload.substring(pos, fimArr).toFloat();
                break;
            }
            if (fim < 0) break;
            arr[count++] = payload.substring(pos, fim).toFloat();
            pos = fim + 1;
        }
        return count;
    };

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

    previsao.totalHoras          = 0;
    previsao.geracaoHojeRestante = 0;
    previsao.geracaoAmanha       = 0;
    previsao.geracaoTotal48h     = 0;

    int hojeAno = ti.tm_year + 1900;
    int hojeMes = ti.tm_mon + 1;
    int hojeDia = ti.tm_mday;

    for (int i = 0; i < total; i++) {
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

        ph.geracaoEstimada = calcularGeracaoHora(
            ph.radiacao, ph.radiacaoDifusa,
            ph.coberturaNuvens, ph.temperatura);

        previsao.geracaoTotal48h += ph.geracaoEstimada;

        if (ph.dia == hojeDia && ph.mes == hojeMes && ph.ano == hojeAno) {
            if (ph.hora >= (uint8_t)ti.tm_hour) {
                previsao.geracaoHojeRestante += ph.geracaoEstimada;
            }
        }

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

    tomarDecisoes();

    buscandoPrev = false;
}

// ── Toma decisões automáticas baseadas na previsão ─────────
void tomarDecisoes() {
    if (!previsao.valido || !bateria.calibrada) return;

    struct tm ti;
    if (!getLocalTime(&ti)) return;

    // Calcula consumo médio por hora
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

    float energiaAtual    = bateria.energiaAtual;
    float energiaMin      = bateria.energiaDisponivel * (redes.corteBaixa / 100.0f);

    int   horaAtual       = ti.tm_hour;
    float horasAteAmanhecer = 0;

    if (horaAtual >= 18 || horaAtual < 6) {
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

    if (horaAtual >= 18 || horaAtual < 6) {
        if (horaAtual >= 18) {
            horasAteAmanhecer = (24 - horaAtual) + horaInicioGeracao;
        } else {
            horasAteAmanhecer = horaInicioGeracao - horaAtual;
        }
    }

    float energiaNecessariaNoite = consumoMedioHora * horasAteAmanhecer;
    float energiaDisponivel      = energiaAtual - energiaMin;
    float saldoEnergia           = energiaDisponivel - energiaNecessariaNoite;

    float consumoRedeAltaHora = consumoMedioHora * 0.40f;
    float consumoLampExtHora  = consumoMedioHora * 0.05f;

    decisao.desligarRedeAltaNoite = false;
    decisao.horaDesligarRedeAlta  = 0;
    decisao.horaLigarRedeAlta     = 0;

    bool eNoite = (horaAtual >= 18 || horaAtual < 6);

    if (eNoite) {
        if (saldoEnergia < 0) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Energia insuf. p/ noite: disp=%.0fWh nec=%.0fWh",
                     energiaDisponivel, energiaNecessariaNoite);
        } else if (saldoEnergia < consumoRedeAltaHora * 3) {
            decisao.desligarRedeAltaNoite = true;
            snprintf(decisao.motivo, sizeof(decisao.motivo),
                     "Margem baixa: saldo=%.0fWh <3h rede alta",
                     saldoEnergia);
        }

        decisao.horaLigarRedeAlta = horaInicioGeracao + 1;
    }

    decisao.desligarLampExtNoite = false;

    if (eNoite && sensores.correntePainel <= 0.5f) {
        if (saldoEnergia < consumoLampExtHora * horasAteAmanhecer) {
            decisao.desligarLampExtNoite = true;
        }
    }

    decisao.alertaEnergiaInsuficiente = false;
    if (eNoite && saldoEnergia < 0 &&
        previsao.geracaoAmanha < consumoMedioHora * 12) {
        decisao.alertaEnergiaInsuficiente = true;
    }

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

    snprintf(tempBuf, sizeof(tempBuf),
             "Decisao: saldo=%.0fWh cons_noite=%.0fWh horas=%.1fh",
             saldoEnergia, energiaNecessariaNoite, horasAteAmanhecer);
    adicionarLog(tempBuf);
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

// ── Controla cargas baseado no nível da bateria ────────────
void controlarEnergia() {
    if (!bateria.calibrada) return;

    float pct  = bateria.percentualAtual;
    unsigned long agora = millis();

    // ── LÂMPADA EXTERNA - AUTOMÁTICO POR LUZ DO DIA ──────────
    if (sensores.correntePainel > 0.5f && estados.lampExt) {
        desligarLampExt();
        adicionarLog("Lamp.Externa OFF: dia detectado (painel gerando)");
    }

    if (sensores.correntePainel <= 0.5f &&
        !estados.lampExt &&
        pct > redes.corteLampExt + 2.0f &&
        estados.inversor &&
        sensores.inversoreAtivo) {
        ligarLampExt();
        adicionarLog("Lamp.Externa ON: anoitecer detectado");
    }

    // ── LÂMPADA INTERNA ───────────────────────────────────────
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

    // ── LÂMPADA EXTERNA - CORTE POR ENERGIA ──────────────────
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

    // ── REDE ALTA ─────────────────────────────────────────────
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

    // ── REDE BAIXA + INVERSOR ────────────────────────────────
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

            if (!estados.redeBaixa) {
                ligarRedeBaixa();
                adicionarLog("Rede Baixa ON: energia recuperada");
                delay(2000);
            }

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

            if (!estados.redesAlta &&
                pct > redes.corteAlta + 2.0f &&
                sensores.inversoreAtivo) {
                ligarRedeAlta();
                adicionarLog("Rede Alta ON: energia recuperada");
                delay(200);
            }

            if (!estados.lampExt &&
                pct > redes.corteLampExt + 2.0f &&
                sensores.inversoreAtivo &&
                sensores.correntePainel <= 0.5f) {
                ligarLampExt();
                adicionarLog("Lamp.Externa ON: energia recuperada (noite)");
            }

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

// ── Salva calibração dos sensores ──────────────────────────
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

// ── Carrega calibração dos sensores ────────────────────────
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

    String corSetaPB = fluxoPB ? "#43a047" : "#1e272e";
    String corSetaPC = fluxoPC ? "#fdd835" : "#1e272e";
    String corSetaBC = fluxoBC ? "#ef6c00" : "#1e272e";

    String c = "";

    if (!bateria.calibrada) {
        c += "<div class='wn' style='font-size:11px;margin-bottom:6px'>"
             "&#9888; Bateria nao calibrada. "
             "<a href='/bateria' style='color:#ffe0b2'>Calibrar agora</a></div>";
    }

    // CSS animações
    c += F("<style>"
        "@keyframes flowR{0%{stroke-dashoffset:20}100%{stroke-dashoffset:0}}"
        "@keyframes flowD{0%{stroke-dashoffset:20}100%{stroke-dashoffset:0}}"
        ".flow-ativo{stroke-dasharray:8 4;animation:flowR 0.6s linear infinite;}"
        ".flow-inativo{opacity:0.1}"
        ".bx{border-radius:10px;border:2px solid #333;background:#1a1a1a;"
            "padding:10px 8px;text-align:center;}"
        ".bx-ic{font-size:28px}"
        ".bx-nm{font-size:10px;color:#546e7a;text-transform:uppercase;margin-top:3px}"
        ".bx-vl{font-size:16px;font-weight:bold;margin-top:4px}"
        ".bx-sb{font-size:10px;color:#607d8b;margin-top:2px}"
        ".cg{border-radius:8px;border:2px solid #222;background:#111;"
            "padding:9px 11px;margin-bottom:5px;}"
        ".cg-row{display:flex;justify-content:space-between;align-items:center;}"
        ".cg-nm{font-size:12px;color:#90a4ae}"
        ".cg-st{font-size:13px;font-weight:bold}"
        ".cg-inf{font-size:10px;color:#546e7a;margin-top:5px;padding-top:5px;"
                "border-top:1px solid #1e1e1e;}"
    "</style>");

    // Diagrama SVG
    c += "<h3 style='text-align:center;margin:4px 0 8px'>&#9889; Fluxo de Energia</h3>";
    c += "<div style='position:relative;width:100%;margin-bottom:8px'>";

    c += "<svg width='100%' height='160px' viewBox='0 0 320 160' "
         "xmlns='http://www.w3.org/2000/svg' style='position:absolute;top:0;left:0'>";
    c += "<defs>"
         "<marker id='ar' markerWidth='6' markerHeight='6' "
         "refX='5' refY='3' orient='auto'>"
         "<path d='M0,0 L6,3 L0,6 Z' fill='currentColor'/>"
         "</marker></defs>";

    c += "<line x1='88' y1='55' x2='232' y2='55' "
         "stroke='" + corSetaPB + "' stroke-width='3' stroke-linecap='round' "
         "class='" + String(fluxoPB?"flow-ativo":"flow-inativo") + "'/>";
    if (fluxoPB) {
        c += "<polygon points='232,49 244,55 232,61' fill='" + corSetaPB + "'/>";
        c += "<text x='160' y='48' text-anchor='middle' font-size='10' fill='"
             + corSetaPB + "'>" + String(strBat) + "</text>";
    }

    c += "<line x1='80' y1='90' x2='80' y2='128' "
         "stroke='" + corSetaPC + "' stroke-width='3' stroke-linecap='round' "
         "class='" + String(fluxoPC?"flow-ativo":"flow-inativo") + "'/>";
    c += "<line x1='80' y1='128' x2='148' y2='148' "
         "stroke='" + corSetaPC + "' stroke-width='3' stroke-linecap='round' "
         "class='" + String(fluxoPC?"flow-ativo":"flow-inativo") + "'/>";
    if (fluxoPC)
        c += "<polygon points='142,141 154,149 144,156' fill='" + corSetaPC + "'/>";

    c += "<line x1='240' y1='90' x2='240' y2='128' "
         "stroke='" + corSetaBC + "' stroke-width='3' stroke-linecap='round' "
         "class='" + String(fluxoBC?"flow-ativo":"flow-inativo") + "'/>";
    c += "<line x1='240' y1='128' x2='172' y2='148' "
         "stroke='" + corSetaBC + "' stroke-width='3' stroke-linecap='round' "
         "class='" + String(fluxoBC?"flow-ativo":"flow-inativo") + "'/>";
    if (fluxoBC)
        c += "<polygon points='178,141 166,149 176,156' fill='" + corSetaBC + "'/>";

    c += "</svg>";

    c += "<div style='display:flex;justify-content:space-between;"
         "position:relative;z-index:1'>";

    // Box Painel
    c += "<div class='bx' style='width:42%;border-color:" + corPainel + "'>";
    c += "<div class='bx-ic'>&#9728;</div>";
    c += "<div class='bx-nm'>Painel Solar</div>";
    c += "<div class='bx-vl' style='color:" + corPainel + "'>";
    c += painelGerando ? String(strPainel) : "0.0A";
    c += "</div><div class='bx-sb'>";
    if (painelGerando) {
        c += String(strPP);
        if (fluxoPB && fluxoPC)   c += " &rarr; Bat+Casa";
        else if (fluxoPB)         c += " &rarr; Bateria";
        else if (fluxoPC)         c += " &rarr; Casa";
    } else {
        c += "Sem gera&ccedil;&atilde;o";
    }
    c += "</div></div>";

    // Box Bateria
    {
        String icoBat = batCarregando    ? "&#9650;&#128267;" :
                        batDescarregando ? "&#9660;&#128267;" : "&#128267;";
        String subBat;
        if (!bateria.calibrada)        subBat = "Calibrar";
        else if (batCarregando)        subBat = String(strBat) + " entrando";
        else if (batDescarregando)     subBat = String(strBat) + " saindo";
        else                           subBat = "Equilibrio";

        char bufVbox[10];
        snprintf(bufVbox, sizeof(bufVbox), "%.2fV", estimTensao.tensaoAtual);

        c += "<div class='bx' style='width:42%;border-color:" + corBat + "'>";
        c += "<div class='bx-ic'>" + icoBat + "</div>";
        c += "<div class='bx-nm'>Bateria</div>";
        c += "<div class='bx-vl' style='color:" + corBat + "'>";
        c += bateria.calibrada ? String(pctBuf) : "N/Cal";
        c += "</div>";
        c += "<div style='font-size:11px;font-weight:bold;color:#ce93d8;"
             "margin-top:2px' id='tensao-val'>";
        c += String(bufVbox);
        c += "</div>";
        c += "<div class='bx-sb'>" + subBat + "</div>";
        c += "</div>";
    }

    c += "</div>";
    c += "<div style='height:75px'></div>";

    // Box Casa
    c += "<div style='background:#1a1a1a;border:2px solid #263238;"
         "border-radius:10px;padding:8px 12px;"
         "display:flex;align-items:center;gap:10px;"
         "margin-bottom:6px;position:relative;z-index:1'>";
    c += "<span style='font-size:26px'>&#127968;</span>";
    c += "<div style='flex:1'>";
    c += "<div style='font-size:10px;color:#546e7a;text-transform:uppercase'>Casa</div>";
    if (casaConsumindo) {
        c += "<div style='font-size:14px;font-weight:bold;color:#4fc3f7'>";
        c += String(strCasa) + " &bull; " + String(strPC);
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
    if (diagAtivo)
        c += "<div style='font-size:10px;color:#90caf9;text-align:right'>"
             "&#128270;<br>Diagn.</div>";
    c += "</div>";
    c += "</div>";

    // Cargas
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
    "&#9888; Bloqueada por curto - <a href='/redes#curto' "
    "style='color:#ffcc80'>ver diagnostico</a>" : "";

    String obsLE = sensores.correntePainel > 0.5f ?
    "&#9728; Desligada automaticamente (dia - painel gerando)" :
    "&#127769; Controle automatico por luminosidade do painel";

    String obsBx = curto.redeBaixaCurto ?
    "&#9888; Bloqueada por curto" : "";

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

    // Card tensão
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
        c += "<div style='display:flex;justify-content:space-between;"
             "align-items:center'>";

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
        c += "Vmax: <b style='color:#90a4ae'>";
        char bufVmax[8];
        snprintf(bufVmax, sizeof(bufVmax), "%.1fV", estimTensao.tensaoMaxima);
        c += String(bufVmax) + "</b><br>";
        c += "Vnom: <b style='color:#90a4ae'>";
        char bufVnom[8];
        snprintf(bufVnom, sizeof(bufVnom), "%.1fV", estimTensao.tensaoNominalReal);
        c += String(bufVnom) + "</b><br>";
        c += "Cal: <b style='color:#90a4ae'>";
        c += String(estimTensao.fatorCalibTensao, 3) + "</b><br>";
        c += estimTensao.amostrasCalib > 20 ?
             "<span style='color:#43a047'>&#10003; Calibrado</span>" :
             "<span style='color:#ef6c00'>&#128336; Calibrando</span>";
        c += " (" + String(estimTensao.amostrasCalib) + ")";
        c += "</div>";
        c += "</div></div>";
    }

    // Gráficos canvas
    c += "<div style='margin:8px 0'>";
    c += "<div style='font-size:11px;color:#546e7a;margin-bottom:4px'>"
         "&#128200; Fluxo de Potencia - ultimos 60s</div>";

    c += "<canvas id='gc' height='120' "
         "style='width:100%;background:#0a0a0a;"
         "border:1px solid #1e1e1e;border-radius:6px'></canvas>";

    c += "<div style='display:flex;gap:10px;justify-content:center;"
         "margin:4px 0;flex-wrap:wrap'>";
    c += "<span style='font-size:10px;color:#fdd835'>&#9632; Painel</span>";
    c += "<span style='font-size:10px;color:#4fc3f7'>&#9632; Casa</span>";
    c += "<span style='font-size:10px;color:#43a047'>&#9632; Bat.Entrada</span>";
    c += "<span style='font-size:10px;color:#ef6c00'>&#9632; Bat.Saida</span>";
    c += "</div>";

    c += "<div style='font-size:11px;color:#546e7a;margin:6px 0 4px'>"
         "&#9889; Tensao Bateria - ultimos 60s</div>";
    c += "<canvas id='gv' height='70' "
         "style='width:100%;background:#0a0a0a;"
         "border:1px solid #1e1e1e;border-radius:6px'></canvas>";
    c += "<div style='display:flex;gap:10px;justify-content:center;"
         "margin-top:4px'>";
    c += "<span style='font-size:10px;color:#ce93d8'>&#9632; Tensao (V)</span>";
    c += "</div>";
    c += "</div>";

    // JavaScript gráficos
    c += F("<script>"
    "var GD={p:[],c:[],bi:[],bo:[],v:[]};"
    "var GMAX=60;"

    "function drawPot(){"
    "var cv=document.getElementById('gc');"
    "if(!cv)return;"
    "cv.width=cv.offsetWidth||cv.parentElement.offsetWidth;"
    "var ctx=cv.getContext('2d');"
    "var W=cv.width,H=cv.height;"
    "ctx.clearRect(0,0,W,H);"

    "ctx.strokeStyle='#1a1a1a';ctx.lineWidth=1;"
    "for(var i=0;i<=4;i++){"
    "var y=H*i/4;"
    "ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();}"

    "ctx.strokeStyle='#2d2d2d';ctx.lineWidth=1;"
    "ctx.beginPath();ctx.moveTo(0,H/2);ctx.lineTo(W,H/2);ctx.stroke();"

    "var mx=100;"
    "var all=GD.p.concat(GD.c,GD.bi,GD.bo);"
    "for(var i=0;i<all.length;i++)mx=Math.max(mx,Math.abs(all[i]));"
    "mx=Math.ceil(mx/100)*100;"

    "ctx.fillStyle='#546e7a';ctx.font='9px Arial';"
    "ctx.fillText(mx+'W',2,10);"
    "ctx.fillText('0',2,H/2+4);"
    "ctx.fillText('-'+mx+'W',2,H-2);"

    "function drawLinha(arr,cor){"
    "if(arr.length<2)return;"
    "var n=arr.length,dx=W/(GMAX-1);"
    "ctx.strokeStyle=cor;ctx.lineWidth=1.5;"
    "ctx.beginPath();"
    "for(var i=0;i<n;i++){"
    "var x=(GMAX-n+i)*dx;"
    "var y=H/2-(arr[i]/mx)*(H/2-10);"
    "if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}"
    "ctx.stroke();"
    "ctx.globalAlpha=0.07;ctx.fillStyle=cor;"
    "ctx.beginPath();"
    "ctx.moveTo((GMAX-n)*dx,H/2);"
    "for(var i=0;i<n;i++){"
    "var x=(GMAX-n+i)*dx;"
    "var y=H/2-(arr[i]/mx)*(H/2-10);"
    "ctx.lineTo(x,y);}"
    "ctx.lineTo((GMAX-1)*dx,H/2);"
    "ctx.fill();ctx.globalAlpha=1.0;}"

    "drawLinha(GD.p,'#fdd835');"
    "drawLinha(GD.c,'#4fc3f7');"
    "drawLinha(GD.bi,'#43a047');"
    "drawLinha(GD.bo,'#ef6c00');"

    "if(GD.p.length>0){"
    "var l=GD.p.length-1;"
    "ctx.font='bold 10px Arial';var xL=W-50;"
    "ctx.fillStyle='#fdd835';"
    "ctx.fillText(GD.p[l].toFixed(0)+'W',xL,12);"
    "ctx.fillStyle='#4fc3f7';"
    "ctx.fillText(GD.c[l].toFixed(0)+'W',xL,24);"
    "ctx.fillStyle='#43a047';"
    "ctx.fillText(GD.bi[l].toFixed(0)+'W',xL,36);"
    "ctx.fillStyle='#ef6c00';"
    "ctx.fillText(GD.bo[l].toFixed(0)+'W',xL,48);"
    "}}"

    "function drawTensao(){"
    "var cv=document.getElementById('gv');"
    "if(!cv)return;"
    "cv.width=cv.offsetWidth||cv.parentElement.offsetWidth;"
    "var ctx=cv.getContext('2d');"
    "var W=cv.width,H=cv.height;"
    "ctx.clearRect(0,0,W,H);"
    "if(GD.v.length<2)return;"

    "var vMn=9999,vMx=0;"
    "for(var i=0;i<GD.v.length;i++){"
    "if(GD.v[i]>0.1){"
    "vMn=Math.min(vMn,GD.v[i]);"
    "vMx=Math.max(vMx,GD.v[i]);}}"
    "if(vMn===9999)return;"
    "vMn=parseFloat((vMn-0.5).toFixed(1));"
    "vMx=parseFloat((vMx+0.5).toFixed(1));"
    "var vR=vMx-vMn;if(vR<0.5)vR=0.5;"

    "ctx.strokeStyle='#1a1a1a';ctx.lineWidth=1;"
    "for(var i=0;i<=3;i++){"
    "ctx.beginPath();ctx.moveTo(0,H*i/3);"
    "ctx.lineTo(W,H*i/3);ctx.stroke();}"

    "ctx.fillStyle='#546e7a';ctx.font='9px Arial';"
    "ctx.fillText(vMx.toFixed(1)+'V',2,10);"
    "ctx.fillText(((vMn+vMx)/2).toFixed(1)+'V',2,H/2+4);"
    "ctx.fillText(vMn.toFixed(1)+'V',2,H-2);"

    "var n=GD.v.length,dx=W/(GMAX-1);"
    "ctx.strokeStyle='#ce93d8';ctx.lineWidth=2;"
    "ctx.beginPath();"
    "for(var i=0;i<n;i++){"
    "var x=(GMAX-n+i)*dx;"
    "var y=H-((GD.v[i]-vMn)/vR)*(H-12)-2;"
    "if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}"
    "ctx.stroke();"

    "ctx.globalAlpha=0.1;ctx.fillStyle='#ce93d8';"
    "ctx.beginPath();ctx.moveTo((GMAX-n)*dx,H);"
    "for(var i=0;i<n;i++){"
    "var x=(GMAX-n+i)*dx;"
    "var y=H-((GD.v[i]-vMn)/vR)*(H-12)-2;"
    "ctx.lineTo(x,y);}"
    "ctx.lineTo((GMAX-1)*dx,H);ctx.fill();"
    "ctx.globalAlpha=1.0;"

    "ctx.fillStyle='#ce93d8';ctx.font='bold 11px Arial';"
    "ctx.fillText(GD.v[n-1].toFixed(2)+'V',W-48,13);}"

    "function atualizarGraficos(){"
    "var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){"
    "if(x.readyState==4&&x.status==200){"
    "try{"
    "var d=JSON.parse(x.responseText);"
    "var pb=d.pb||0;"

    "GD.p.push(parseFloat(d.pp)||0);"
    "GD.c.push(parseFloat(d.pc)||0);"
    "GD.bi.push(pb>0?pb:0);"
    "GD.bo.push(pb<0?-pb:0);"
    "GD.v.push(parseFloat(d.vat)||0);"

    "while(GD.p.length>GMAX)GD.p.shift();"
    "while(GD.c.length>GMAX)GD.c.shift();"
    "while(GD.bi.length>GMAX)GD.bi.shift();"
    "while(GD.bo.length>GMAX)GD.bo.shift();"
    "while(GD.v.length>GMAX)GD.v.shift();"

    "drawPot();drawTensao();"

    "var elV=document.getElementById('tensao-val');"
    "if(elV)elV.textContent=parseFloat(d.vat||0).toFixed(2)+'V';"

    "}catch(e){}}"
    "};x.open('GET','/dados',true);x.send();}"

    "setInterval(atualizarGraficos,1000);"
    "atualizarGraficos();"
    "window.addEventListener('resize',function(){drawPot();drawTensao();});"
    "</script>");

    // Barra bateria
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
        c += "<div class='barra-bg'><div class='barra' style='width:"+
             String((int)pct)+"%;background:"+corBrr+"'>"+
             String((int)pct)+"%</div></div>";
        c += "<div style='display:flex;justify-content:space-between;"
             "margin-top:3px;font-size:10px;color:#546e7a'>";
        char bWh[60];
        snprintf(bWh,sizeof(bWh),"%.0f / %.0f Wh",
                 bateria.energiaAtual,bateria.energiaDisponivel);
        c += "<span>" + String(bWh) + "</span>";
        char bPot[40];
        snprintf(bPot,sizeof(bPot),"%.0fW @ %.2fV %s",
                 fabsf(sensores.potenciaBateria),
                 estimTensao.tensaoAtual,
                 batCarregando?"entrada":batDescarregando?"saida":"equil.");
        c += "<span>" + String(bPot) + "</span></div>";
    } else {
        c += "<div class='wn' style='font-size:11px;text-align:center'>"
             "Nao calibrada - "
             "<a href='/bateria' style='color:#ffe0b2'>calibrar</a></div>";
    }
    c += "</div>";

    // Cards potência
    c += "<div style='display:grid;grid-template-columns:1fr 1fr 1fr;"
         "gap:6px;margin:4px 0'>";

    auto cardPot = [&](const char* ico, const char* nome,
                       const String& corrente, const String& potencia,
                       const String& cor, bool destaque) -> String {
        String h = "<div class='sc";
        if (destaque) h += " ok";
        h += "' style='padding:8px;text-align:center'>";
        h += "<div class='lbl'>" + String(ico) + " " + String(nome) + "</div>";
        h += "<div style='font-size:15px;font-weight:bold;color:" + cor + "'>";
        h += corrente + "</div>";
        h += "<div class='sub'>" + potencia + "</div>";
        h += "</div>";
        return h;
    };

    {
        char bufP[20];
        snprintf(bufP,sizeof(bufP),"%.0fW",fabsf(sensores.potenciaPainel));
        c += cardPot("&#9728;","Painel",
                     painelGerando?String(strPainel):"0.0A",
                     painelGerando?String(bufP):"Sem geracao",
                     corPainel, painelGerando);
    }

    {
        char bufC[20];
        snprintf(bufC,sizeof(bufC),"%.0fW",sensores.potenciaCasa);
        c += cardPot("&#127968;","Casa",
                     String(strCasa),String(bufC),
                     "#4fc3f7",false);
    }

    {
        String corF  = batCarregando?"#43a047":batDescarregando?"#ef6c00":"#fdd835";
        String sinal = batCarregando?"+":batDescarregando?"-":"~";
        char bufB[30];
        snprintf(bufB,sizeof(bufB),"%.0fW @ %.2fV",
                 fabsf(sensores.potenciaBateria),
                 estimTensao.tensaoAtual);
        c += cardPot("&#128267;","Bateria",
                     sinal+String(strBat),
                     String(bufB),corF,false);
    }
    c += "</div>";

    // Info sistema
    String ip   = modoAP?WiFi.softAPIP().toString():WiFi.localIP().toString();
    String modo = modoAP?"AP":"WiFi";
    c += "<div class='nf' style='font-size:10px;margin-top:4px'>";
    c += "&#128246; "+modo+" | "+ip;
    if (!modoAP && WiFi.status()==WL_CONNECTED)
        c += " | "+String(WiFi.RSSI())+"dBm";
    snprintf(tempBuf,sizeof(tempBuf)," | RAM:%.1fKB | Up:%lus",
             esp_get_free_heap_size()/1024.0f,millis()/1000);
    c += String(tempBuf)+"</div>";

    c += "<script>setTimeout(function(){location.reload();},10000);</script>";

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

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "zero_painel") {
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
                     "Painel ZERO: %dmV (ADC:%d)", mvMedio, adcMedio);
            adicionarLog(tempBuf);
            msg = "<div class='ok'>&#10003; Painel zero: " +
                  String(mvMedio) + " mV</div>";
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
                     "Casa ZERO: %dmV (ADC:%d)", mvMedio, adcMedio);
            adicionarLog(tempBuf);
            msg = "<div class='ok'>&#10003; Casa zero: " +
                  String(mvMedio) + " mV</div>";
        }
        else if (acao == "sens_painel") {
            float corrReal = server.arg("corrente_real").toFloat();
            if (corrReal < 0.5f || corrReal > 50.0f) {
                msg = "<div class='er'>&#10007; Corrente invalida (0.5-50A)</div>";
            } else {
                long soma = 0;
                for (int i = 0; i < 200; i++) {
                    soma += analogRead(PIN_SENSOR_PAINEL);
                    delay(5);
                }
                int   adcMedio = soma / 200;
                int   mvMedio  = adcParaMv(adcMedio);
                float diff     = (float)(mvMedio - calibPainel.offsetMV);
                if (fabsf(diff) < 10.0f) {
                    msg = "<div class='er'>&#10007; Sinal proximo do zero. "
                          "Use corrente maior.</div>";
                } else {
                    if (diff < 0) {
                        calibPainel.invertido = true;
                        diff = -diff;
                    } else {
                        calibPainel.invertido = false;
                    }
                    calibPainel.sensMvA   = diff / corrReal;
                    calibPainel.calibrado = true;
                    salvarCalibSensores();
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Painel SENS: %.2fmV/A", calibPainel.sensMvA);
                    adicionarLog(tempBuf);
                    msg = "<div class='ok'>&#10003; Painel sens: " +
                          String(calibPainel.sensMvA,2) + " mV/A</div>";
                }
            }
        }
        else if (acao == "sens_casa") {
            float corrReal = server.arg("corrente_real").toFloat();
            if (corrReal < 0.5f || corrReal > 100.0f) {
                msg = "<div class='er'>&#10007; Corrente invalida (0.5-100A)</div>";
            } else {
                long soma = 0;
                for (int i = 0; i < 200; i++) {
                    soma += analogRead(PIN_SENSOR_CASA);
                    delay(5);
                }
                int   adcMedio = soma / 200;
                int   mvMedio  = adcParaMv(adcMedio);
                float diff     = (float)(mvMedio - calibCasa.offsetMV);
                if (fabsf(diff) < 10.0f) {
                    msg = "<div class='er'>&#10007; Sinal proximo do zero. "
                          "Ligue uma carga.</div>";
                } else {
                    if (diff < 0) {
                        calibCasa.invertido = true;
                        diff = -diff;
                    } else {
                        calibCasa.invertido = false;
                    }
                    calibCasa.sensMvA   = diff / corrReal;
                    calibCasa.calibrado = true;
                    salvarCalibSensores();
                    snprintf(tempBuf, sizeof(tempBuf),
                             "Casa SENS: %.2fmV/A", calibCasa.sensMvA);
                    adicionarLog(tempBuf);
                    msg = "<div class='ok'>&#10003; Casa sens: " +
                          String(calibCasa.sensMvA,2) + " mV/A</div>";
                }
            }
        }
        else if (acao == "reset_calib") {
            String qual = server.arg("qual");
            if (qual == "painel") {
                calibPainel.offsetMV  = OFFSET_LIDO_MV;
                calibPainel.sensMvA   = PAINEL_SENS_MV_A * 0.66f;
                calibPainel.invertido = false;
                calibPainel.calibrado = false;
                salvarCalibSensores();
                msg = "<div class='wn'>&#9888; Calib painel resetada</div>";
            } else if (qual == "casa") {
                calibCasa.offsetMV  = OFFSET_LIDO_MV;
                calibCasa.sensMvA   = CASA_SENS_MV_A * 0.66f;
                calibCasa.invertido = false;
                calibCasa.calibrado = false;
                salvarCalibSensores();
                msg = "<div class='wn'>&#9888; Calib casa resetada</div>";
            } else if (qual == "tensao") {
                estimTensao.fatorCalibTensao = 1.0f;
                estimTensao.amostrasCalib    = 0;
                salvarCalibTensao();
                calcularFatorBanco();
                msg = "<div class='wn'>&#9888; Calib tensao resetada</div>";
                adicionarLog("Calib tensao resetada");
            }
        }
    }

    int adcPainel = lerADC(PIN_SENSOR_PAINEL);
    int adcCasa   = lerADC(PIN_SENSOR_CASA);
    int mvPainel  = adcParaMv(adcPainel);
    int mvCasa    = adcParaMv(adcCasa);
    float corrPainel = sensores.correntePainel;
    float corrCasa   = sensores.correnteCasa;

    String c = msg;

    c += "<h3>&#9889; Tensao Estimada da Bateria</h3>";

    String corVest = estimTensao.painelAtivo ? "#fdd835" :
                     estimTensao.tensaoAtual >
                     estimTensao.tensaoNominalReal * 0.95f ? "#43a047" :
                     estimTensao.tensaoAtual >
                     estimTensao.tensaoNominalReal * 0.85f ? "#ef6c00" : "#c62828";

    c += "<div class='sc' style='border-color:" + corVest +
         ";background:#0d0d0d;margin-bottom:6px'>";
    c += "<div style='display:flex;justify-content:space-between;align-items:center'>";
    c += "<div>";
    c += "<div class='lbl'>Tensao Atual Estimada</div>";
    char bufVest[12];
    snprintf(bufVest, sizeof(bufVest), "%.2fV", estimTensao.tensaoAtual);
    c += "<div style='font-size:36px;font-weight:bold;color:" +
         corVest + "' id='tensao-val'>" + String(bufVest) + "</div>";
    String stV = estimTensao.painelAtivo ?
                 "&#9728; Carregando (painel ON)" :
                 sensores.correnteBateria < -0.5f ?
                 "&#9660; Descarregando" : "&#9679; Repouso";
    c += "<div style='font-size:11px;color:#78909c;margin-top:4px'>" +
         stV + "</div>";
    c += "</div>";

    c += "<div style='text-align:right;font-size:11px;color:#546e7a'>";
    char bufBanco[120];
    snprintf(bufBanco, sizeof(bufBanco),
             "Banco: <b style='color:#90a4ae'>%.0fS</b><br>"
             "Vmax carga: <b style='color:#fdd835'>%.2fV</b><br>"
             "Vnom banco: <b style='color:#90a4ae'>%.2fV</b><br>"
             "Vmin corte: <b style='color:#ef5350'>%.2fV</b>",
             estimTensao.fatorBanco,
             estimTensao.tensaoMaxima,
             estimTensao.tensaoNominalReal,
             3.0f * estimTensao.fatorBanco);
    c += String(bufBanco) + "</div>";
    c += "</div></div>";

    c += "<canvas id='gvS' height='90' "
         "style='width:100%;background:#0a0a0a;"
         "border:1px solid #1e1e1e;border-radius:6px;margin-bottom:4px'>"
         "</canvas>";
    c += "<div style='display:flex;justify-content:space-between;"
         "font-size:10px;color:#546e7a;margin-bottom:8px'>";
    c += "<span>&#9589; 60s atras</span>";
    c += "<span style='color:#ce93d8'>&#9632; Tensao (V)</span>";
    c += "<span>Agora &#9589;</span></div>";

    c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>";
    c += "<b>Calibracao da Estimativa:</b> ";
    c += "Fator=<b>" + String(estimTensao.fatorCalibTensao, 4) + "</b> | ";
    c += "Amostras=<b>" + String(estimTensao.amostrasCalib) + "</b> | ";
    if (estimTensao.amostrasCalib < 10)
        c += "<span style='color:#ef6c00'>Iniciando calibracao...</span>";
    else if (estimTensao.amostrasCalib < 30)
        c += "<span style='color:#fdd835'>Calibracao parcial</span>";
    else
        c += "<span style='color:#43a047'>&#10003; Boa precisao</span>";
    c += "</div>";

    c += "<div class='nf' style='font-size:10px'>";
    c += "<b>Tabela SOC &rarr; Tensao (banco " +
         String(estimTensao.tensaoNominalReal, 1) + "V):</b><br>";
    c += "<div style='display:grid;grid-template-columns:repeat(5,1fr);"
         "gap:3px;margin-top:6px'>";

    static const float socTab[] = {0,10,20,30,40,50,60,70,80,90,95,100};
    const int nSoc = 12;
    for (int i = 0; i < nSoc; i++) {
        float vC = soc2tensaoLiFePO4(socTab[i]) *
                   estimTensao.fatorBanco *
                   estimTensao.fatorCalibTensao;
        bool eAtual = bateria.calibrada &&
                      (bateria.percentualAtual >= socTab[i]) &&
                      (i == nSoc-1 ||
                       bateria.percentualAtual < socTab[i+1]);
        c += "<div style='background:" + String(eAtual?"#0d47a1":"#111") +
             ";border:1px solid " + String(eAtual?"#1565c0":"#1e1e1e") +
             ";border-radius:4px;padding:5px 3px;text-align:center'>";
        c += "<div style='font-size:9px;color:#546e7a'>";
        c += String((int)socTab[i]) + "%</div>";
        c += "<div style='font-size:11px;font-weight:bold;color:" +
             String(eAtual?"#4fc3f7":"#78909c") + "'>";
        char vb[8];
        snprintf(vb, sizeof(vb), "%.2fV", vC);
        c += String(vb) + "</div></div>";
    }
    c += "</div>";

    char bufVmax[10];
    snprintf(bufVmax, sizeof(bufVmax), "%.2fV", estimTensao.tensaoMaxima);
    c += "<div style='margin-top:6px;padding:6px;background:#181200;"
         "border:1px solid #332200;border-radius:4px;font-size:10px'>";
    c += "&#9728; <b>Painel ON (100% carga):</b> ";
    c += "<span style='color:#fdd835;font-weight:bold;font-size:12px'>";
    c += String(bufVmax) + "</span>";
    c += " (3.65V/cel × " + String((int)estimTensao.fatorBanco) + "S × ";
    c += String(estimTensao.fatorCalibTensao,3) + ")";
    c += "</div>";
    c += "</div>";

    c += "<form method='POST' action='/sensores' style='margin-top:6px' "
         "onsubmit='return confirm(\"Resetar calibracao de tensao?\")'>";
    c += "<input type='hidden' name='acao' value='reset_calib'>";
    c += "<input type='hidden' name='qual'  value='tensao'>";
    c += "<button type='submit' class='btn-r btn-sm'>"
         "&#8635; Resetar Calibracao de Tensao</button>";
    c += "</form>";

    c += "<h3>&#9728; Sensor Painel Solar (ACS758 50A)</h3>";

    String clsPainel = calibPainel.calibrado ? "ok" : "wn";
    c += "<div class='sc " + clsPainel + "'>";
    c += "<div class='lbl'>Leitura Atual</div>";
    c += "<div class='val'>" + fmtCorrente(corrPainel) + "</div>";
    char bufSP[120];
    snprintf(bufSP, sizeof(bufSP),
             "Sinal: %d mV | ADC: %d | Pot: %.0fW @ %.2fV",
             mvPainel, adcPainel,
             fabsf(sensores.potenciaPainel),
             estimTensao.tensaoAtual);
    c += "<div class='sub'>" + String(bufSP) + "</div>";
    c += "</div>";

    c += "<div class='nf' style='font-size:11px'>";
    char bufCP[120];
    snprintf(bufCP, sizeof(bufCP),
             "Offset: <b>%d mV</b> | Sens: <b>%.2f mV/A</b> | "
             "Invertido: <b>%s</b> | Calibrado: <b>%s</b>",
             calibPainel.offsetMV, calibPainel.sensMvA,
             calibPainel.invertido ? "SIM" : "NAO",
             calibPainel.calibrado ? "SIM" : "NAO");
    c += String(bufCP) + "</div>";

    if (corrPainel < -5.0f) {
        c += "<div class='er' style='font-size:11px'>"
             "&#9888; Corrente negativa alta! Possivel inversao de cabos.</div>";
    }

    c += "<div class='sc' style='margin-top:10px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>"
         "&#9312; Passo 1 - Calibrar Zero</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>"
         "Garanta <b>corrente zero</b> no sensor painel.<br>"
         "Sinal atual: <b>" + String(mvPainel) + " mV</b> | "
         "Offset atual: <b>" + String(calibPainel.offsetMV) + " mV</b></div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='zero_painel'>";
    c += "<button type='submit' class='btn-o' "
         "onclick='return confirm(\"Confirma zero do painel?\\n"
         "Certifique-se de corrente zero!\")'>"
         "&#127919; Calibrar Zero Painel</button>";
    c += "</form></div>";

    c += "<div class='sc' style='margin-top:8px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>"
         "&#9313; Passo 2 - Calibrar Sensibilidade</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>"
         "Com corrente <b>conhecida e estavel</b> passando.<br>"
         "Sinal: <b>" + String(mvPainel) + " mV</b> | "
         "Leitura atual: <b>" + fmtCorrente(corrPainel) + "</b></div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='sens_painel'>";
    c += "<label>Corrente Real (A):</label>";
    c += "<input type='number' name='corrente_real' step='0.1' "
         "min='0.5' max='50' required placeholder='Ex: 5.2'>";
    c += "<button type='submit' class='btn-g'>"
         "&#127919; Calibrar Sensibilidade Painel</button>";
    c += "</form></div>";

    c += "<form method='POST' action='/sensores' style='margin-top:6px'>";
    c += "<input type='hidden' name='acao' value='reset_calib'>";
    c += "<input type='hidden' name='qual'  value='painel'>";
    c += "<button type='submit' class='btn-r btn-sm' "
         "onclick='return confirm(\"Resetar calibracao do painel?\")'>"
         "&#8635; Resetar Calib Painel</button>";
    c += "</form>";

    c += "<h3>&#127968; Sensor Casa (ACS758 100A)</h3>";

    String clsCasa = calibCasa.calibrado ? "ok" : "wn";
    c += "<div class='sc " + clsCasa + "'>";
    c += "<div class='lbl'>Leitura Atual</div>";
    c += "<div class='val'>" + fmtCorrente(corrCasa) + "</div>";
    char bufSC[120];
    snprintf(bufSC, sizeof(bufSC),
             "Sinal: %d mV | ADC: %d | Pot: %.0fW @ %.2fV",
             mvCasa, adcCasa,
             sensores.potenciaCasa,
             estimTensao.tensaoAtual);
    c += "<div class='sub'>" + String(bufSC) + "</div>";
    c += "</div>";

    c += "<div class='nf' style='font-size:11px'>";
    char bufCC[120];
    snprintf(bufCC, sizeof(bufCC),
             "Offset: <b>%d mV</b> | Sens: <b>%.2f mV/A</b> | "
             "Invertido: <b>%s</b> | Calibrado: <b>%s</b>",
             calibCasa.offsetMV, calibCasa.sensMvA,
             calibCasa.invertido ? "SIM" : "NAO",
             calibCasa.calibrado ? "SIM" : "NAO");
    c += String(bufCC) + "</div>";

    c += "<div class='sc' style='margin-top:10px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>"
         "&#9312; Passo 1 - Calibrar Zero</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>"
         "Desligue <b>todas as cargas</b> da casa.<br>"
         "Sinal atual: <b>" + String(mvCasa) + " mV</b> | "
         "Offset atual: <b>" + String(calibCasa.offsetMV) + " mV</b></div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='zero_casa'>";
    c += "<button type='submit' class='btn-o' "
         "onclick='return confirm(\"Confirma zero da casa?\\n"
         "Certifique-se de NENHUMA carga ligada!\")'>"
         "&#127919; Calibrar Zero Casa</button>";
    c += "</form></div>";

    c += "<div class='sc' style='margin-top:8px'>";
    c += "<div class='lbl' style='font-size:12px;color:#81d4fa'>"
         "&#9313; Passo 2 - Calibrar Sensibilidade</div>";
    c += "<div style='font-size:11px;color:#78909c;margin:6px 0'>"
         "Com carga <b>conhecida e estavel</b> ligada.<br>"
         "Sinal: <b>" + String(mvCasa) + " mV</b> | "
         "Leitura: <b>" + fmtCorrente(corrCasa) + "</b></div>";
    c += "<form method='POST' action='/sensores'>";
    c += "<input type='hidden' name='acao' value='sens_casa'>";
    c += "<label>Corrente Real (A):</label>";
    c += "<input type='number' name='corrente_real' step='0.1' "
         "min='0.5' max='100' required placeholder='Ex: 10.5'>";
    c += "<button type='submit' class='btn-g'>"
         "&#127919; Calibrar Sensibilidade Casa</button>";
    c += "</form></div>";

    c += "<form method='POST' action='/sensores' style='margin-top:6px'>";
    c += "<input type='hidden' name='acao' value='reset_calib'>";
    c += "<input type='hidden' name='qual'  value='casa'>";
    c += "<button type='submit' class='btn-r btn-sm' "
         "onclick='return confirm(\"Resetar calibracao da casa?\")'>"
         "&#8635; Resetar Calib Casa</button>";
    c += "</form>";

    c += "<h3>&#128267; Bateria (Calculado)</h3>";
    String dirBat = sensores.correnteBateria >= 0 ? "Carregando":"Descarregando";
    String corBt  = sensores.correnteBateria >= 0 ? "ok":"er";
    c += "<div class='sc " + corBt + "'>";
    c += "<div class='lbl'>Painel - Casa = Bateria</div>";
    c += "<div class='val'>" + fmtCorrente(sensores.correnteBateria);
    c += " " + dirBat + "</div>";
    char bufPB2[40];
    snprintf(bufPB2, sizeof(bufPB2), "%.0fW @ %.2fV",
             fabsf(sensores.potenciaBateria),
             estimTensao.tensaoAtual);
    c += "<div class='sub'>" + String(bufPB2) + "</div>";
    c += "</div>";

    c += "<h3>&#9889; Sinal Status Inversor</h3>";
    int adcInv = lerADC(PIN_STATUS_INV_ALTA);
    int mvInv  = adcParaMv(adcInv);
    sensores.sinalInversor_mV = mvInv;
    String invSt  = sensores.inversoreAtivo ? "LIGADO"  : "DESLIGADO";
    String invCor = sensores.inversoreAtivo ? "#43a047" : "#546e7a";
    c += "<div class='sc'>";
    c += "<div class='lbl'>Status (limiar: " + String(INV_THRESHOLD_MV) + " mV)</div>";
    c += "<div class='val' style='color:" + invCor + "'>" + invSt + "</div>";
    char bufInv[60];
    snprintf(bufInv, sizeof(bufInv),
             "Sinal: %d mV | ADC: %d | Buffer: %dms",
             mvInv, adcInv, INV_BUFFER_MS);
    c += "<div class='sub'>" + String(bufInv) + "</div>";
    c += "</div>";

    c += "<h3>&#128270; Diagnostico</h3>";
    c += "<div class='nf' style='font-size:11px'>";
    c += "<b>Banco LiFePO4 detectado:</b><br>";
    char bufDiag[160];
    snprintf(bufDiag, sizeof(bufDiag),
             "Celulas em serie: <b>%.0fS</b><br>"
             "Tensao nominal: <b>%.2fV</b> (%.0f x 3.2V)<br>"
             "Tensao maxima carga: <b>%.2fV</b> (%.0f x 3.65V)<br>"
             "Tensao minima corte: <b>%.2fV</b> (%.0f x 3.0V)<br>"
             "Fator calibracao: <b>%.4f</b> (%d amostras)<br>"
             "ADC por leitura: <b>%d amostras</b>",
             estimTensao.fatorBanco,
             estimTensao.tensaoNominalReal,
             estimTensao.fatorBanco,
             estimTensao.tensaoMaxima,
             estimTensao.fatorBanco,
             3.0f * estimTensao.fatorBanco,
             estimTensao.fatorBanco,
             estimTensao.fatorCalibTensao,
             estimTensao.amostrasCalib,
             ADC_SAMPLES);
    c += String(bufDiag) + "</div>";

    c += "<button onclick='location.reload()' class='btn-s' style='margin-top:10px'>"
         "&#128259; Atualizar</button>";

    c += F("<script>"
    "var GSV={v:[]};var GSMAX=60;"
    "function drawGSV(){"
    "var cv=document.getElementById('gvS');"
    "if(!cv)return;"
    "cv.width=cv.offsetWidth||cv.parentElement.offsetWidth;"
    "var ctx=cv.getContext('2d'),W=cv.width,H=cv.height;"
    "ctx.clearRect(0,0,W,H);"
    "if(GSV.v.length<2)return;"
    "var vMn=9999,vMx=0;"
    "for(var i=0;i<GSV.v.length;i++){"
    "if(GSV.v[i]>0.1){"
    "vMn=Math.min(vMn,GSV.v[i]);"
    "vMx=Math.max(vMx,GSV.v[i]);}}"
    "if(vMn===9999)return;"
    "vMn=parseFloat((vMn-0.5).toFixed(1));"
    "vMx=parseFloat((vMx+0.5).toFixed(1));"
    "var vR=vMx-vMn;if(vR<0.3)vR=0.3;"
    "ctx.strokeStyle='#1a1a1a';ctx.lineWidth=1;"
    "for(var i=0;i<=4;i++){"
    "ctx.beginPath();ctx.moveTo(0,H*i/4);"
    "ctx.lineTo(W,H*i/4);ctx.stroke();}"
    "ctx.fillStyle='#546e7a';ctx.font='9px Arial';"
    "ctx.fillText(vMx.toFixed(1)+'V',2,10);"
    "ctx.fillText(vMn.toFixed(1)+'V',2,H-2);"
    "var n=GSV.v.length,dx=W/(GSMAX-1);"
    "ctx.strokeStyle='#ce93d8';ctx.lineWidth=2;"
    "ctx.beginPath();"
    "for(var i=0;i<n;i++){"
    "var x=(GSMAX-n+i)*dx;"
    "var y=H-((GSV.v[i]-vMn)/vR)*(H-14)-2;"
    "if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}"
    "ctx.stroke();"
    "ctx.globalAlpha=0.12;ctx.fillStyle='#ce93d8';"
    "ctx.beginPath();ctx.moveTo((GSMAX-n)*dx,H);"
    "for(var i=0;i<n;i++){"
    "var x=(GSMAX-n+i)*dx;"
    "var y=H-((GSV.v[i]-vMn)/vR)*(H-14)-2;"
    "ctx.lineTo(x,y);}"
    "ctx.lineTo((GSMAX-1)*dx,H);ctx.fill();"
    "ctx.globalAlpha=1.0;"
    "ctx.fillStyle='#ce93d8';"
    "var last=n-1;"
    "var lx=(GSMAX-n+last)*dx;"
    "var ly=H-((GSV.v[last]-vMn)/vR)*(H-14)-2;"
    "ctx.beginPath();ctx.arc(lx,ly,3,0,2*Math.PI);ctx.fill();"
    "ctx.font='bold 12px Arial';"
    "ctx.fillText(GSV.v[last].toFixed(2)+'V',W-52,14);}"

    "function atuGS(){"
    "var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){"
    "if(x.readyState==4&&x.status==200){"
    "try{"
    "var d=JSON.parse(x.responseText);"
    "GSV.v.push(parseFloat(d.vat)||0);"
    "while(GSV.v.length>GSMAX)GSV.v.shift();"
    "drawGSV();"
    "var el=document.getElementById('tensao-val');"
    "if(el)el.textContent=parseFloat(d.vat||0).toFixed(2)+'V';"
    "}catch(e){}}}"
    ";x.open('GET','/dados',true);x.send();}"
    "setInterval(atuGS,1000);atuGS();"
    "window.addEventListener('resize',drawGSV);"
    "</script>");

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
    c += "Limpa usuario, bateria e redes. <b>Mantem configuracao WiFi.</b>";
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
                configLocal.fatorCalib    = 1.0f;
                configLocal.amostrasCalib = 0;
                salvarConfigLocal();

                ultimaPrev = 0;

                msg = "<div class='ok'>&#10003; Configuracao salva! "
                "Buscando previsao...</div>";
                adicionarLog("Previsao: nova config salva");
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
            ultimaPrev = 0;
            msg = "<div class='ok'>&#9679; Atualizando previsao...</div>";
        }
    }

    String c = msg;

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
        unsigned long minPassados = (millis() - previsao.ultimaAtualizacao) / 60000;

        c += "<div class='nf' style='font-size:11px;margin-bottom:8px'>";
        c += "&#128336; Ultima atualizacao: ";
        if (minPassados < 60) {
            c += String(minPassados) + " minutos atras";
        } else {
            c += String(minPassados/60) + "h" + String(minPassados%60) + "m atras";
        }
        c += " | Proximo em ";
        unsigned long proxMin = (PREV_INTERVALO - (millis() - ultimaPrev)) / 60000;
        c += String(proxMin) + " min";
        c += "</div>";

        c += "<div style='display:grid;grid-template-columns:1fr 1fr;"
        "gap:6px;margin-bottom:8px'>";

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
        snprintf(bH, sizeof(bH), "%.2f kWh", previsao.geracaoHojeRestante / 1000.0f);
        snprintf(bA, sizeof(bA), "%.2f kWh", previsao.geracaoAmanha / 1000.0f);
        snprintf(bT, sizeof(bT), "%.2f kWh", previsao.geracaoTotal48h / 1000.0f);

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

        c += "<h3>&#129504; Decisoes Automaticas</h3>";

        if (decisao.alertaEnergiaInsuficiente) {
            c += "<div class='er'>&#9888; ALERTA: Energia pode ser insuficiente "
            "para a noite! Geracao amanha tambem baixa.</div>";
        }

        c += "<div style='background:#111;border:1px solid #1e1e1e;"
        "border-radius:8px;padding:10px;margin-bottom:8px'>";

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

        if (strlen(decisao.motivo) > 0) {
            c += "<div style='font-size:10px;color:#546e7a;margin-top:6px'>";
            c += "&#128220; " + String(decisao.motivo) + "</div>";
        }

        c += "</div>";

        c += "<h3>&#127919; Calibracao Automatica</h3>";
        c += "<div style='background:#111;border:1px solid #1e1e1e;"
        "border-radius:8px;padding:10px;margin-bottom:8px'>";

        float precisao = min(100.0f, (float)configLocal.amostrasCalib / 50.0f * 100.0f);
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

        c += "<form method='POST' action='/previsao'>";
        c += "<input type='hidden' name='acao' value='atualizar'>";
        c += "<button type='submit' class='btn-s'>"
        "&#128260; Forcar Atualizacao da Previsao</button>";
        c += "</form>";

        c += "<form method='POST' action='/previsao' style='margin-top:6px' "
        "onsubmit='return confirm(\"Resetar calibracao?\")'>";
        c += "<input type='hidden' name='acao' value='reset_calib'>";
        c += "<button type='submit' class='btn-r btn-sm'>"
        "&#8635; Resetar Calibracao</button>";
        c += "</form>";

        c += "<h3>&#128336; Previsao por Hora</h3>";

        struct tm ti2;
        getLocalTime(&ti2);

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

                if (ph.dia == ti2.tm_mday && ph.mes == (ti2.tm_mon+1))
                    c += " <span style='color:#43a047'>(hoje)</span>";
                else
                    c += " <span style='color:#546e7a'>(amanha)</span>";

                c += "</td></tr>";
            }

            bool eAgora = (ph.dia  == ti2.tm_mday &&
                           ph.mes  == (ti2.tm_mon+1) &&
                           ph.hora == (uint8_t)ti2.tm_hour);

            c += "<tr style='border-bottom:1px solid #1a1a1a;";
            if (eAgora) c += "background:#0d1f0d;";
            c += "'>";

            c += "<td style='padding:4px 6px;color:#90a4ae;text-align:center'>";
            snprintf(tempBuf, sizeof(tempBuf), "%02d:00", ph.hora);
            c += String(tempBuf);
            if (eAgora) c += "&#9654;";
            c += "</td>";

            char bR[12];
            snprintf(bR, sizeof(bR), "%.0fW/m&sup2;", ph.radiacao + ph.radiacaoDifusa);
            c += "<td style='padding:4px 6px;text-align:right;color:";
            c += ph.radiacao > 100 ? "#fdd835" : "#455a64";
            c += "'>" + String(bR) + "</td>";

            char bN[8];
            snprintf(bN, sizeof(bN), "%.0f%%", ph.coberturaNuvens);
            c += "<td style='padding:4px 6px;text-align:right;color:";
            c += ph.coberturaNuvens > 70 ? "#78909c" :
                 ph.coberturaNuvens > 30 ? "#90a4ae" : "#4fc3f7";
            c += "'>" + String(bN) + "</td>";

            char bT2[8];
            snprintf(bT2, sizeof(bT2), "%.0f&deg;C", ph.temperatura);
            c += "<td style='padding:4px 6px;text-align:right;color:#ef5350'>";
            c += String(bT2) + "</td>";

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

    c += "<h3>&#9881; Configurar Localizacao</h3>";

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

    c += "<label>Latitude (ex: -23.667768108648623):</label>";
    char bufLat[30], bufLon[30];
    snprintf(bufLat, sizeof(bufLat), "%.15f", configLocal.latitude);
    snprintf(bufLon, sizeof(bufLon), "%.15f", configLocal.longitude);
    c += "<input type='text' name='lat' required "
    "placeholder='-23.667768108648623' ";
    if (configLocal.configurado) c += "value='" + String(bufLat) + "'";
    c += ">";

    c += "<label>Longitude (ex: -48.184803456419935):</label>";
    c += "<input type='text' name='lon' required "
    "placeholder='-48.184803456419935' ";
    if (configLocal.configurado) c += "value='" + String(bufLon) + "'";
    c += ">";

    c += "<label>Potencia Total dos Paineis (kWp):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Ex: 2 paineis de 550W = 1.1 kWp | "
    "4 paineis de 400W = 1.6 kWp</div>";
    c += "<input type='number' name='kwp' step='0.01' "
    "min='0.1' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.2f'", configLocal.painelKWp);
    c += String(tempBuf) + " placeholder='Ex: 1.1'>";

    c += "<label>Eficiencia do Sistema (%):</label>";
    c += "<div class='nf' style='font-size:10px;margin:4px 0'>"
    "Tipico: 75-85% | Inclui perdas de cabos, inversor e sujeira</div>";
    c += "<input type='number' name='ef' step='1' "
    "min='10' max='100' required ";
    snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", configLocal.eficiencia * 100.0f);
    c += String(tempBuf) + " placeholder='Ex: 80'>";

    c += "<button type='submit' class='btn-g'>"
    "&#128190; Salvar e Ativar Previsao</button>";
    c += "</form>";

    c += "<div class='nf' style='font-size:10px;margin-top:8px'>";
    c += "&#127760; Dados: Open-Meteo (open-meteo.com) - Gratuito, sem chave de API<br>";
    c += "&#128336; Atualizacao automatica a cada 1 hora<br>";
    c += "&#127919; Precisao melhora automaticamente ao longo do tempo";
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
    // Inicializa LED e botão
    setupLED();
    setupBotao();
    setupReles();
    ledBlink();

    adicionarLog("RRFS Energia v7.0");
    logRAM();

    // Inicializa calibração sensores com valores padrão
    calibPainel.offsetMV  = OFFSET_LIDO_MV;
    calibPainel.sensMvA   = PAINEL_SENS_MV_A * 0.66f;
    calibPainel.invertido = false;
    calibPainel.calibrado = false;

    calibCasa.offsetMV  = OFFSET_LIDO_MV;
    calibCasa.sensMvA   = CASA_SENS_MV_A * 0.66f;
    calibCasa.invertido = false;
    calibCasa.calibrado = false;

    // Inicializa estimativa de tensão
    estimTensao.fatorCalibTensao = 1.0f;
    estimTensao.amostrasCalib    = 0;
    estimTensao.tensaoAtual      = 12.8f;
    estimTensao.tensaoMaxima     = 14.6f;
    estimTensao.tensaoNominalReal= 12.8f;
    estimTensao.fatorBanco       = 4.0f;
    estimTensao.painelAtivo      = false;

    // Carrega configurações
    carregarConfigBateria();
    carregarConfigRedes();
    carregarContadores();
    carregarHistorico();
    adicionarLog("Historico e contadores carregados");

    carregarCalibSensores();
    carregarCalibTensao();

    calcularBateria();

    snprintf(tempBuf, sizeof(tempBuf),
             "Banco LiFePO4: %.0fS | Vmax=%.2fV | Vnom=%.2fV | Cal=%.4f",
             estimTensao.fatorBanco,
             estimTensao.tensaoMaxima,
             estimTensao.tensaoNominalReal,
             estimTensao.fatorCalibTensao);
    adicionarLog(tempBuf);

    ultimoContagem = millis();

    // Estimativa inicial de tensão
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

    carregarConfigLocal();

    adicionarLog("Config carregada");
    snprintf(tempBuf, sizeof(tempBuf),
             "Bat: %.1fV %.0fAh %.0f%% | Calib:%s | Tensao:%.2fV",
             bateria.tensaoNominal, bateria.capacidadeAh,
             bateria.descargaSegura,
             bateria.calibrada ? "SIM" : "NAO",
             estimTensao.tensaoAtual);
    adicionarLog(tempBuf);

    // Sequência de inicialização das cargas
    ligarRedeBaixa();
    adicionarLog("Rede baixa ON (roteador)");

    adicionarLog("Aguardando roteador 8s...");
    unsigned long tRoteador = millis();
    while (millis() - tRoteador < 8000) {
        atualizarLED();
        delay(10);
    }
    adicionarLog("Roteador pronto");

    ligarInversor();
    adicionarLog("Inversor ON");

    adicionarLog("Aguardando inversor 5s...");
    unsigned long tInversor = millis();
    while (millis() - tInversor < 5000) {
        atualizarLED();
        delay(10);
    }

    bool invOK = false;
    unsigned long tCheckInv = millis();
    while (millis() - tCheckInv < 3000) {
        if (lerStatusInversor()) { invOK = true; break; }
        delay(200);
    }

    if (invOK) {
        adicionarLog("Inversor STATUS: OK");
    } else {
        adicionarLog("AVISO: Inversor sem status - continuando mesmo assim");
    }

    ligarRedeAlta();
    adicionarLog("Rede alta ON");
    delay(500);

    ligarLampExt();
    adicionarLog("Lamp. externa ON");
    delay(200);
    ligarLampInt();
    adicionarLog("Lamp. interna ON");

    logRAM();

    // Rede WiFi
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

    // Leitura inicial dos sensores
    lerSensores();

    estimTensao.tensaoAtual = estimarTensaoBateria();
    snprintf(tempBuf, sizeof(tempBuf),
             "Tensao apos 1a leitura: %.2fV | Painel:%s",
             estimTensao.tensaoAtual,
             estimTensao.painelAtivo ? "ON" : "OFF");
    adicionarLog(tempBuf);

    // Inicializa histórico de tensão
    for (int i = 0; i < HIST_TENSAO_PONTOS; i++) {
        histTensaoV[i]      = estimTensao.tensaoAtual;
        histCorrPainel[i]   = 0;
        histCorrCasa[i]     = 0;
        histCorrBat[i]      = 0;
    }
    ultimoHistTensao = millis();

    // Inicializa monitoramento curto
    curto.monitorando = false;
    curto.estado      = CURTO_NORMAL;
    curto.subEstado   = SUB_IDLE;
    curto.tInicio     = 0;

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
    if (configLocal.configurado &&
        WiFi.status() == WL_CONNECTED &&
        !buscandoPrev &&
        millis() - ultimaPrev >= PREV_INTERVALO) {
        ultimaPrev = millis();
        buscarPrevisaoMeteo();
    }

    // Calibração automática e decisões a cada 5 minutos
    static unsigned long tCalib = 0;
    if (millis() - tCalib >= 300000) {
        tCalib = millis();
        calibrarPrevisao();
        verificarReligamentoAuto();
    }

    // Log RAM a cada 10min
    static unsigned long tRAM = 0;
    if (agora - tRAM >= 600000) {
        tRAM = agora;
        logRAM();
    }

    delay(1);
}
