/*
 * ════════════════════════════════════════════════════════════════════════════════
 * RRFS ENERGIA v32 - SISTEMA DE GERENCIAMENTO DE ENERGIA SOLAR ESP32
 * ════════════════════════════════════════════════════════════════════════════════
 *
 * ÍNDICE DE BLOCOS (PARTE 1)
 *   BLOCO 01  Includes
 *   BLOCO 02  Pinos (hardware)
 *   BLOCO 03  Constantes do sistema
 *   BLOCO 04  Estruturas de dados
 *   BLOCO 05  Variáveis globais
 *   BLOCO 06  Protótipos
 *   BLOCO 07  Logs persistentes
 *   BLOCO 08  LED e botão
 *   BLOCO 09  Relés
 *   BLOCO 10  Sensor ADC painel + banco de baterias + leitura consolidada
 *   BLOCO 11  BLE BMS (NimBLE)
 *   BLOCO 12  Sinais analógicos (AC Baixa / AC Alta / Casa)
 *   BLOCO 13  NTP e tempo
 *   BLOCO 14  Previsão solar (Open-Meteo, 8 modelos + pesos + calibração)
 *   BLOCO 15  Histórico de energia (30 dias)
 *   BLOCO 16  Proteção por SOC + autonomia
 *   BLOCO 17  Máquina de estados Inversor Baixa
 *   BLOCO 18  PADRÕES DE CONSUMO (NOVO v32)
 *   BLOCO 19  Excesso de energia solar
 *   BLOCO 20  Máquina de estados Inversor Alta
 *   BLOCO 21  Planejador de energia (usa os padrões)
 *   BLOCO 22  Iluminação externa
 *   BLOCO 23  Controle geral
 *   BLOCO 24  Preferências / NVS
 *   BLOCO 25  WiFi e AP
 *   BLOCO 26  Utilitários web
 *   BLOCO 27  setup()
 * ════════════════════════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 01 — INCLUDES
// ═══════════════════════════════════════════════════════════════════════════════════
#include <HTTPClient.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <Update.h>
#include <LittleFS.h>          // v32: guarda os padrões de consumo (30 dias) fora do NVS
#include <NimBLEDevice.h>
#include <esp_heap_caps.h>

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 02 — DEFINIÇÃO DE PINOS
// ═══════════════════════════════════════════════════════════════════════════════════
#define LED_AZUL               2
#define BOTAO_BOOT             0
#define PIN_SENSOR_PAINEL     34
#define PIN_AC_SINAL_BAIXA    35
#define PIN_RELE_INV_BAIXA    26
#define PIN_RELE_LAMP_EXT     33
#define PIN_AC_SINAL_ALTA     36
#define PIN_SINAL_CASA        39
#define PIN_RELE_INV_ALTA     25

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 03 — CONSTANTES DO SISTEMA
// ═══════════════════════════════════════════════════════════════════════════════════

// ── DNS e Web Server ────────────────────────────────────────────────────────────
#define DNS_PORT               53
#define TEMPO_RESET_MS      10000

// ── Logs Persistentes ───────────────────────────────────────────────────────────
#define MAX_LOGS              100
#define LOG_MAX_LEN            64
#define LOG_NVS_NS        "rrfs_log"
#define LOG_NVS_MAX            12

// ── ADC ─────────────────────────────────────────────────────────────────────────
#define ADC_VREF             3300
#define ADC_BITS             4095

// ── Sensor Corrente Painel ──────────────────────────────────────────────────────
#define PAINEL_OFFSET_MV     2500
#define PAINEL_SENS_MV_A     35.9f
#define FILTRO_RUIDO_A        0.15f
#define BUFFER_CORRENTE        10

// ── BLE BMS JBD/LanmaoBMS ──────────────────────────────────────────────────────
#define BMS_SERVICE_UUID  "0000ff00-0000-1000-8000-00805f9b34fb"
#define BMS_RX_UUID       "0000ff01-0000-1000-8000-00805f9b34fb"
#define BMS_TX_UUID       "0000ff02-0000-1000-8000-00805f9b34fb"
#define BLE_RECONECT_MS    15000UL
#define BLE_TIMEOUT_MS      8000UL
#define BLE_LEITURA_MS      1000UL
#define BLE_SCAN_DURACAO_S     5
#define BMS_FILTRO_RUIDO_A  0.1f
#define BMS_DADO_VALIDO_MS 10000UL

// ── Sinal AC ────────────────────────────────────────────────────────────────────
#define AC_JANELA_MS            40
#define AC_BUFFER_SUAV           6
#define AC_CONFIRMA_MS         400UL
#define AC_HISTERESE_LIGA_MV    50
#define AC_HISTERESE_DESLIGA_MV 40
#define AC_LIMIAR_MV_DEFAULT   400

// ── Sinal Inversor Alta + Sinal da Casa ─────────────────────────────────────────
#define AC_ALTA_LIGA_CONFIRMA_MS      100UL
#define AC_ALTA_DESLIGA_CONFIRMA_MS  5000UL
#define CASA_LIGA_CONFIRMA_MS        5000UL
#define CASA_DESLIGA_CONFIRMA_MS    10000UL

// ── Inversor Baixa: mesmo procedimento do Alta (NOVO) ──
#define BAIXA_AGUARDA_AC_MS    10000UL
#define BAIXA_PERDEU_AC_MS     30000UL
#define BAIXA_MIN_OFF_MS      180000UL
#define BAIXA_RETRY_LONGO_MS  600000UL

// ── Máquinas Inversor Baixa/Alta (diagnóstico) ──────────────────────────────────
#define DIAG_AGUARDA_AC_MS   3000UL
#define DIAG_DESLIGA_MS      3000UL
#define DIAG_RETRY_LONGO_MS 60000UL
#define DIAG_AGUARDA_AC_ALTA_MS 10000UL
#define DIAG_MAX_TENTATIVAS     3
#define ALTA_MIN_OFF_MS      180000UL
#define ALTA_MIN_ON_MS        30000UL
#define ALTA_PERDEU_AC_MS     30000UL
#define ALTA_RETRY_LONGO_MS  600000UL

// ── Excesso de energia solar ────────────────────────────────────────────────────
#define EXC_SOC_ENTRADA_PCT      99.0f
#define EXC_SOC_CONF_MANTEM_PCT  98.0f
#define EXC_SOC_QUEDA_PCT         1.0f
#define EXC_SOC_SAIDA_PCT        90.0f
#define EXC_PAINEL_MIN_A          0.8f
#define EXC_BAT_CARGA_ZERO_A      0.5f
#define EXC_BAT_DESC_ENTRADA_A    0.6f
#define EXC_BAT_VOLTA_CARGA_A     0.3f
#define EXC_PREV_SOBRA_W        100.0f
#define EXC_PREV_SOBRA_PROX_W     0.0f
#define EXC_CONFIRMA_MS       60000UL
#define EXC_CONF_TOLERA_MS    15000UL
#define EXC_GRACA_MS         120000UL
#define EXC_QUEDA_CONFIRMA_MS 30000UL
#define EXC_SEM_SOL_CONFIRMA_MS 20000UL
#define EXC_ATIVO_RAPIDO_MS  300000UL
#define EXC_COOLDOWN_MS      300000UL
#define EXC_COOLDOWN_FALHA_MS 900000UL
#define EXC_FIM_FRACAO_PICO       0.35f
#define EXC_HORA_FIM_PADRAO       15

// ── Lâmpada Externa ─────────────────────────────────────────────────────────────
#define LAMP_EXT_HORA_LIGA      19
#define LAMP_EXT_MIN_LIGA        0
#define LAMP_EXT_HORA_DESLIGA    5
#define LAMP_EXT_MIN_DESLIGA    30
#define LAMP_EXT_CHECK_MS    60000UL

// ── Timers Gerais ───────────────────────────────────────────────────────────────
#define INTERVALO_LEITURA_MS   500UL
#define INTERVALO_CONTROLE_MS 1000UL
#define INTERVALO_NTP_MS    3600000UL
#define INTERVALO_PREVISAO_MS 3600000UL
#define WIFI_RECON_INTERVALO  15000UL
#define WIFI_RECON_TIMEOUT     8000UL
#define WIFI_CHECK_INTERVALO   5000UL
#define WIFI_MAX_FALHAS            3

// ── Histórico ───────────────────────────────────────────────────────────────────
#define HIST_DIAS              30
#define HIST_HORAS             24
#define HIST_KEY_VER           28
#define HIST_SNAP_PONTOS       60

// ── Previsão Solar ──────────────────────────────────────────────────────────────
#define PREV_DIAS               7
#define PREV_TOTAL_HORAS      168
#define PREV_FATOR_INICIAL      0.80f
#define PREV_CTRL_EFIC          0.95f
#define PREV_RAD_CORTE         40.0f
#define PREV_RAD_PLENA        300.0f
#define PREV_SOC_LIMITE        92.0f    // SOC >= isso (ou excesso ativo): painel limitado pela demanda
#define PREV_PESO_SUPERESTIMA   1.00f
#define PREV_PESO_SUBESTIMA     0.35f
#define PREV_ALFA_BAIXA         0.20f
#define PREV_ALFA_ALTA          0.08f
#define PREV_USAR_INCLINADO        0
#define PREV_MODELOS               8    // v32: modelos meteorologicos comparados
#define PREV_HEAP_MIN_MULTI    28000    // heap contiguo minimo p/ baixar os 8 modelos de uma vez

// ── NTP ─────────────────────────────────────────────────────────────────────────
#define NTP_SERVER1       "pool.ntp.org"
#define NTP_SERVER2       "time.google.com"

// ── BLE Scan ────────────────────────────────────────────────────────────────────
#define BLE_LISTA_MAX      20
#define BLE_SCAN_LISTA_S    8

// ── Proteção Bateria ────────────────────────────────────────────────────────────
#define PROT_HISTERESE_PCT 5.0f

// ── WiFi emergência ─────────────────────────────────────────────────────────────
#define WIFI_EMERG_RETRY_MS   60000UL     // era 90000
#define WIFI_EMERG_JANELA_MS  20000UL     // era 10000 (STA varre canais com o AP no ar)
#define WIFI_AP_APOS_MS       20000UL

// ── Planejador de energia ───────────────────────────────────────────────────────
#define PLANO_HORIZONTE_H        36
#define PLANO_INTERVALO_MS     5000UL
#define PLANO_CONFIRMA_BLOQ_MS 300000UL
#define PLANO_CONFIRMA_LIB_MS  600000UL
#define PLANO_FATOR_GERACAO     0.85f
#define PLANO_MARGEM_PCT        3.0f
#define PLANO_MARGEM_LIB_PCT    3.0f
#define LAMP_HISTERESE_PCT      5.0f

// ── Padrões de consumo (NOVO v32) ───────────────────────────────────────────────
#define PAD_ARQ              "/pad30.bin"
#define PAD_ARQ_TMP          "/pad30.tmp"
#define PAD_MAGIC            0x50414433UL
#define PAD_EV_DIA_MAX            20      // eventos guardados por dia (mantém os mais energéticos)
#define PAD_CLUSTERS               8
#define PAD_MIN_THR_A           1.0f
#define PAD_K_DEV               3.5f
#define PAD_CONF_S                 6
#define PAD_FIM_S                  8
#define PAD_MIN_DUR_S              8
#define PAD_MIN_MEDIA_A         0.8f
#define PAD_MAX_EVENTO_S     14400UL
#define PAD_TAU_BASE_S        300.0f
#define PAD_IGNORA_MS        75000UL
#define PAD_DIAS_PESO           0.92f
#define PAD_PORDIA_MIN          0.08f
#define PAD_MIN_HORAS_APR         24      // horas de dados p/ o planejador poder agir
#define PAD_COB_MIN_S        1800.0f      // hora só conta como "monitorada" com 30 min ativos
#define PAD_SNAP_MS            30000UL    // corrente estavel em novo patamar por 30 s => nova base

// ── Sessão de login por cookie ──────────────────────────────────────────────────
#define SESSAO_MAX         4
#define SESSAO_TIMEOUT_MS  43200000UL

// ── Saúde da bateria ────────────────────────────────────────────────────────────
#define SAUDE_NS              "rrfs_saude"
#define SAUDE_N_CAP               12      // medidas de capacidade guardadas
#define SAUDE_N_RI                 8      // amostras de resistência (mediana)
#define SAUDE_N_RI_DIA            30      // pontos diários de resistência
#define SAUDE_JAN_MIN_PCT         20      // variação mínima de SOC p/ valer a janela
#define SAUDE_SOC_MIN             10
#define SAUDE_SOC_MAX             96      // acima disso: fase de absorção, não conta
#define SAUDE_SOC_SALTO_MAX        3      // salto de SOC > 3%: BMS recalibrou, descarta
#define SAUDE_JAN_MAX_MS   86400000UL     // janela expira em 24 h
#define SAUDE_DT_MAX_MS        3000UL     // buraco >3 s na leitura: descarta a janela
#define SAUDE_FRAME_MAX_MS     6000UL     // BMS precisa estar atualizando
#define RI_MIN_DI                8.0f     // degrau mínimo de corrente (A)
#define RI_ESTAVEL_A             1.5f     // corrente antes/depois precisa estar estável
#define RI_FRAME_GAP_MS        4500UL
#define RI_MIN_MOHM              0.5f
#define RI_MAX_MOHM            300.0f

// ── WiFi AP Padrão ──────────────────────────────────────────────────────────────
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ── Modelos do Open-Meteo comparados (edite se algum for recusado) ──────────────
static const char* const PREV_MODELO_ID[PREV_MODELOS] = {
    "ecmwf_ifs025", "gfs_seamless", "icon_seamless", "gem_seamless",
    "meteofrance_seamless", "jma_seamless", "ukmo_seamless", "cma_grapes_global"
};

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 04 — ESTRUTURAS DE DADOS
// ═══════════════════════════════════════════════════════════════════════════════════

struct ConfigBanco { uint8_t nS, nP, modo; bool configurado; };

struct DadosBMS {
    float corrente, tensao, temperatura, soc;
    uint16_t ciclos;
    bool valido;
    unsigned long tUltimaDado;
    float capacidadeAh;
};

struct DadosBanco {
    float tensaoTotal, correnteTotal, potencia, socPct;
    uint16_t ciclos;
    float capacidadeTotal;
};

struct ConfigProtecao { float reservaPct; bool ativa; };

enum EstadoBLE { BLE_DESCONECTADO = 0, BLE_ESCANEANDO, BLE_CONECTANDO, BLE_CONECTADO, BLE_FALHA };

struct BleItem { char mac[18]; char nome[24]; int rssi; bool jbd; };
struct ConfigBMS { char macAddress[18]; bool configurada; };

struct EstadoBLECtrl {
    EstadoBLE     estado;
    unsigned long tUltimoTentativa, tUltimaLeitura, tConectando;
    bool          notificacaoAtiva;
    uint8_t       falhasConsec;
    uint8_t       bufferRX[64];
    uint8_t       bufferLen;
};

struct CalibSensor { int offsetMV; float sensMvA; bool calibrado; };

struct BufferCorr { float amostras[BUFFER_CORRENTE]; uint8_t indice, total; };

struct DadosSensores {
    float correntePainel, correnteBateria, correnteCasa;
    float potenciaPainel, potenciaBateria, potenciaCasa;
    float tensaoBanco, temperaturaBMS, socBMS;
    uint16_t ciclosBMS;
    float capacidadeBanco;
    int   sinalPainel_mV;
};

struct EstadoReles { bool lampExt, invBaixa, invAlta; };
struct ConfigInversores { bool invertReleInvBaixa, invertReleLampExt, invertReleInvAlta; };

struct SinalAC {
    float         mediaMV;
    bool          presente, aguardandoLigar, aguardandoDesligar;
    unsigned long tInicioConfirma;
};

enum EstadoBaixa { INV_A_DESLIGADO = 0, INV_A_LIGADO, INV_A_DIAG_AGUARDA, INV_A_DIAG_OFF };

struct MaquinaBaixa {
    EstadoBaixa    estado;
    unsigned long tInicioEstado;
    uint8_t       tentativas;
    uint16_t      totalLigadas, totalFalhas;
};

enum EstadoAlta { INV_ALTA_DESLIGADO = 0, INV_ALTA_LIGADO, INV_ALTA_DIAG_AGUARDA, INV_ALTA_DIAG_OFF };

struct MaquinaAlta {
    EstadoAlta    estado;
    unsigned long tInicioEstado;
    uint8_t       tentativas;
    uint16_t      totalLigadas, totalFalhas;
};

// ── Histórico ───────────────────────────────────────────────────────────────────
struct RegistroHora { float painelAh, casaAh, batCargAh, batDescAh; };

struct RegistroDia {
    uint16_t     ano;
    uint8_t      mes, dia;
    bool         valido;
    RegistroHora horas[HIST_HORAS];
    float totalPainelAh, totalCasaAh, totalBatCargAh, totalBatDescAh;
};

struct ContadoresGlobais { float painelAh, casaAh, batCargAh, batDescAh; };

struct AcumHora {
    float         painelAh, casaAh, batCargAh, batDescAh;
    uint8_t       horaAtual, diaAtual, mesAtual;
    uint16_t      anoAtual;
    unsigned long ultimoAcum;
};

struct ConfigTempo {
    int  fusoHoras, fusoMinutos;
    bool ntpAtivo, sincronizado;
    int  manualAnno, manualMes, manualDia, manualHora, manualMin;
};

// ── Previsão Solar (v32: radiação por modelo) ───────────────────────────────────
struct ConfigPrevisao {
    bool  ativa;
    float latitude, longitude, painelKW, fatorCalib;
    bool  configurada;
    float controladorA;
};

struct DadosPrevisao {
    bool          valido;
    bool          modoUnico;                                   // true = so best_match (fallback)
    bool          modeloOk[PREV_MODELOS];
    uint16_t      radModelo[PREV_MODELOS][PREV_TOTAL_HORAS];   // W/m2, 0xFFFF = sem dado
    float         ahPrevisto[PREV_TOTAL_HORAS];                // combinado + calibrado
    float         ahDia[PREV_DIAS];
    unsigned long tAtualizacao;
    int           horaBase, diaBase, mesBase, anoBase;
};

// Estatistica de acerto por modelo (persistida) -> define o peso de cada um
struct EstatModelos {
    float    err[PREV_MODELOS];
    float    prev[PREV_MODELOS];
    uint16_t n[PREV_MODELOS];
    float    peso[PREV_MODELOS];
};

struct ConfigLimiaresSinal { int limiarAcMV, limiarAcAltaMV, limiarCasaMV; };
struct ConfigLimiares { float pctLampExt; };

struct EstimativaTempo {
    bool  valido;
    int   horas, minutos;
    bool  carregando;
    float taxaA, ahDisp, ahFalta;
    bool  naReserva;
};

// ── Planejador ──────────────────────────────────────────────────────────────────
struct SimPlano {
    float nivelMin;
    int   idxMin, idxFalta;
    float cons, ger;
    int   fimH, fimD, faltaH, faltaD;
    int   idxExc, excH, excD;     // v32b: primeira hora em que a bateria enche e ainda sobra geracao
};

struct PlanoEnergia {
    int   verConf;      // -2 protecao, -1 sem dados, 0 ok, 1 lampada off basta, 2 economizar
    int   verPend;
    int   semMotivo;    // 1 previsao, 2 BMS, 3 relogio, 4 consumo
    bool  lampBloqueada;
    bool  usaHistorico;
    bool  usaPadroes;   // v32: previsao de consumo por padroes (>= 1 dia completo)
    bool  consIniciado;
    bool  sobraSolar;   // NOVO: bateria cheia + painel cobrindo a casa
    float consBase;     // A da casa SEM lampada e SEM eventos (base estavel)
    float consLenta;    // media lenta da casa (fallback sem padroes)
    float lampA;        // A da lampada (aprendido pelo degrau da base)
    float perfil[24];   // fallback: media horaria da casa (inclui eventos)
    bool  perfilOk[24];
    float baseP[24];    // v32: base horaria historica (sem lampada, sem eventos)
    bool  baseOk[24];
    float evAbs[72];    // v32: Ah esperados de eventos, hora absoluta (0 = hoje 00h)
    float evRestAh;     // v32: Ah de eventos esperados ate o fim de hoje
    int   horaPico;
    float picoA;
    uint8_t horaPerfil;
    SimPlano com, sem;
    float margemAh, reducaoA, reducaoW;
    unsigned long tPend, analisandoAte;
};

struct EstatPrev {
    float    err = 0, prev = 0, real = 0;
    uint16_t n = 0;
    float    hErr = 0, hPrev = 0, hReal = 0;
    uint16_t hN = 0;
    uint8_t  dia = 0;
    uint8_t  horaAv = 255;
    bool     limitado = false;     // v32: SOC alto ou excesso: painel limitado pela demanda
    bool     pular = true;
    bool     carregado = false;
};

struct ExcessoSolar {
    uint8_t       fase;        // 0 monitorando, 1 confirmando, 2 ativo
    bool          pede;
    uint8_t       motivo;
    unsigned long tConf, tAtiv, tFimConf, tCool, coolMs;
    uint16_t      eventos;
    bool          fimGeracao;
    int           diaBloq;
    char          ultimoFim[48];
};

struct SessaoLogin {
    operator bool() const;
    SessaoLogin& operator=(bool v);
};

struct DiaCompacto {
    uint16_t ano;
    uint8_t  mes, dia, valido;
    uint16_t h[HIST_HORAS][4];
};

// ── PADRÕES DE CONSUMO (NOVO v32) ───────────────────────────────────────────────
// Um evento = subida temporaria da corrente da casa acima da base estavel.
struct EvPadrao {
    uint16_t iniMin;     // minuto do dia em que comecou
    uint16_t durS;       // duracao (s)
    uint16_t mediaDA;    // corrente media ACIMA da base (A x 10)
    uint8_t  picoA;      // pico acima da base (A)
    uint8_t  solarPct;   // % do tempo em que o painel cobria a casa
};

// Um dia de padroes (slot igual ao do historico: histIndiceAtual)
struct DiaPadroes {
    uint16_t ano;
    uint8_t  mes, dia;
    uint8_t  n;                       // eventos guardados
    uint8_t  flut;                    // flutuação típica da base (A x 10)
    uint8_t  perdidos;                // eventos fracos descartados por falta de espaço
    uint32_t cob;                     // bit h = hora h monitorada (inclui horas de sobra solar)
    uint8_t  base[24];                // base horária sem lâmpada (A x 10), 255 = sem dado
    uint8_t  sob[24];                 // Ah x10 consumidos acima da base durante sobra solar
    EvPadrao ev[PAD_EV_DIA_MAX];
};

// Um "tipo de padrao" aprendido (ex.: forno, micro-ondas, compressor)
struct ClusterPad {
    bool   valido;
    float  cA, cDur, eAh, wSum;
    float  porDia;      // ocorrências esperadas por dia (soma das taxas horárias)
    float  solarPct;
    float  picoA;
    float  hist[24];    // soma ponderada de ocorrências por hora de início
    float  rate[24];    // ocorrências por hora, por dia monitorado
    float  restHoje;
    uint8_t feitos;
};

// Detector em tempo real (base estavel + eventos)
struct DetPad {
    bool          iniciado;
    float         xs, base, dev, curta;
    bool          emEvento;
    unsigned long tIni, tSub, tAbaixo, tUlt;
    float         somaAs, pico, solarAs;
    uint16_t      iniMin;
    unsigned long ignoraAte;
    bool          fimIgnora, medLamp, lampLigou, refOk, lampAnt, invAnt;
    float         baseAntes, ref;
    uint8_t       hHora;
    float         hSoma;
    uint16_t      hN;
    bool          sobraAnt;
    float         sobAh;
    float         hAtivoS;
    uint16_t      hAno;
    uint8_t       hMes, hDia, hSlot;
    bool          hSlotOk;
    bool          altaAnt;            // v32b: estado anterior do rele do Inv.Alta
    unsigned long tDesv;              // v32b: desde quando a corrente esta fora da base (Inv.Alta OFF)
};

// ── Saúde da bateria ────────────────────────────────────────────────────────────
struct CapMed { uint16_t ano; uint8_t mes, dia; int8_t delta; float cap, nom; };
struct RiDia  { uint16_t ano; uint8_t mes, dia; float mohm; };

struct SaudeSalva {                      // persistida no NVS (blob)
    uint8_t  ver, nCap, nDia, riN, riIdx;
    bool     riBaseOk;
    uint16_t riTotal;
    float    riBase;                     // referência: mediana das 8 primeiras medidas (mΩ)
    float    nomManual;                  // capacidade nominal informada (0 = usar a do BMS)
    float    riRing[SAUDE_N_RI];
    CapMed   cap[SAUDE_N_CAP];
    RiDia    dia[SAUDE_N_RI_DIA];
};

struct SaudeRun {                        // só em RAM
    bool          janAtiva;
    int           socIni, socAnt;
    float         ahNet;
    unsigned long tIni, tUlt;
    unsigned long tUltFrame, t1;
    float         i1, v1, i2, v2;
    uint8_t       nFr, estRi;
    float         preI, preV, posI, posV;
    bool          sujo;
    unsigned long tSalvo;
};

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 05 — VARIÁVEIS GLOBAIS
// ═══════════════════════════════════════════════════════════════════════════════════

PlanoEnergia plano;
EstatPrev    estPrev;
EstatModelos estMod;
SessaoLogin  logado;

WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP = false;

unsigned long ultimoBlink = 0;
bool ledState = false, blinkAtivo = true;
unsigned long botaoPressionadoEm = 0;
bool botaoSegurando = false;

char     logs[MAX_LOGS][LOG_MAX_LEN];
uint16_t totalLogs = 0;
uint8_t  logIndex = 0;
bool     logsCarregados = false;
char     tempBuf[256];

ConfigBanco banco = {1, 1, 1, false};
DadosBMS dadosBMS = {0.0f, 0.0f, 25.0f, 0.0f, 0, false, 0, 0.0f};
DadosBanco dadosBanco = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f};

ConfigBMS     cfgBMS   = {"", false};
EstadoBLECtrl ctrlBLE  = {BLE_DESCONECTADO, 0, 0, 0, false, 0, {0}, 0};
BLEClient*                bleClient      = nullptr;
BLERemoteService*         bleService     = nullptr;
BLERemoteCharacteristic*  bleCharRX      = nullptr;
BLERemoteCharacteristic*  bleCharTX2     = nullptr;
BLERemoteCharacteristic*  bleCharTX      = nullptr;
BLERemoteCharacteristic*  bleCharRX2     = nullptr;
BLEScan*                  bleScan        = nullptr;
bool                      bleEncontrou   = false;
volatile uint16_t bleVistos = 0;
BLEAdvertisedDevice*      bleDispositivo = nullptr;
BleItem       bleLista[BLE_LISTA_MAX];
volatile uint8_t bleListaN        = 0;
volatile bool    bleScanSolicitado = false;
volatile bool    bleScanAtivo      = false;
volatile bool    bleModoLista      = false;
bool             bleIniciado       = false;
bool             bleTxResposta     = false;
volatile uint32_t bleRxTotal    = 0;
volatile bool     bleRxPend     = false;
uint8_t           bleRxDump[20];
volatile uint8_t  bleRxDumpLen  = 0;
uint8_t           bleRxLogRest  = 0;
bool              bleTxAmbos    = false;
unsigned long     bleTesteAte   = 0;
bool              bleManualOff  = false;

DadosSensores sensores   = {0};
CalibSensor   calibPainel;
BufferCorr    bufPainel   = {{0}, 0, 0};
BufferCorr    bufBateria  = {{0}, 0, 0};

EstadoReles    estados = {false, false, false};
ConfigInversores cfgInv = {false, false, false};

SinalAC acBaixa   = {0.0f, false, false, false, 0};
SinalAC acAlta    = {0.0f, false, false, false, 0};
SinalAC sinalCasa = {0.0f, false, false, false, 0};

MaquinaBaixa maqBaixa = {INV_A_DESLIGADO, 0, 0, 0, 0};
bool diagRetryLongo = false;
MaquinaAlta maqAlta = {INV_ALTA_DESLIGADO, 0, 0, 0, 0};
bool diagRetryLongoAlta = false;

ConfigLimiaresSinal limiarSinal = {AC_LIMIAR_MV_DEFAULT, AC_LIMIAR_MV_DEFAULT, AC_LIMIAR_MV_DEFAULT};
ConfigLimiares limiares = {20.0f};

unsigned long ultimaLeitura = 0, ultimoControle = 0, ultimoContagem = 0, ultimoLampCheck = 0;

ConfigTempo   cfgTempo   = {-3, 0, false, false, 2024, 1, 1, 0, 0};
unsigned long tUltimoNTP = 0;
bool          ntpSincronizado = false;

ConfigPrevisao cfgPrevisao    = {false, 0, 0, 1.0f, 1.0f, false};
DadosPrevisao  previsao       = {false};
unsigned long  tUltimaPrevisao = 0;

String        gWifiSSID, gWifiPASS, gWifiIP, gWifiGW, gWifiMASK, gWifiDNS;
unsigned long tUltimoCheckWifi = 0, tUltimoReconWifi = 0, tUltimaReconWifi2 = 0;
uint8_t       wifiFalhasConsec = 0;
bool          wifiReconectando = false;
unsigned long tInicioRecon     = 0;

RegistroDia       histDiaAtual;
uint16_t          histIndiceAtual = 0, histTotalDias = 0;
ContadoresGlobais contadores = {0, 0, 0, 0};
AcumHora          acum       = {0, 0, 0, 0, 255, 0, 0, 0, 0};
float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx   = 0;
bool    histSnapFull  = false;
unsigned long ultimoHistSnap = 0;
uint8_t  diaCache = 0, mesCache = 0;
uint16_t anoCache = 0;
unsigned long ultimaAtualizacaoDia = 0;

// Padrões de consumo (v32)
DiaPadroes padHist[HIST_DIAS];        // 30 slots, mesmo indice do historico (~3,8 KB de RAM)
ClusterPad padCl[PAD_CLUSTERS];
uint8_t    padNCl   = 0;
uint8_t    padDias  = 0;              // dias completos usados no aprendizado
bool       padFsOk  = false;
bool       padSujo  = false;
DetPad     pad      = {};

bool          modoTeste       = false;
unsigned long tInicioTeste    = 0;
unsigned long TIMEOUT_TESTE_MS = 30000UL;

unsigned long tInicioSistema      = 0;
bool          sistemaEstabilizado = false;

ConfigProtecao protecao = {15.0f, false};

bool otaAtivo = false, otaOk = false, otaErro = false;
unsigned long tOtaUltimo = 0;
bool otaPrepSolicitado = false, otaPronto = false, otaUploadIniciado = false;

static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t logSalvoTotal = 0;

unsigned long tUltimoRetryEmerg     = 0;
bool          emergTentandoWifi     = false;
unsigned long tWifiPerdido = 0;
unsigned long tInicioEmergTentativa = 0;

SaudeSalva saude;
SaudeRun   saudeR;

uint16_t   padHorasCob = 0;           // horas monitoradas somadas (todos os dias)

static uint16_t ahParaU16(float v) {
    if (v < 0.0f) v = 0.0f;
    v *= 100.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

ExcessoSolar excesso = {0, false, 0, 0, 0, 0, 0, EXC_COOLDOWN_MS, 0, false, 0, ""};
unsigned long tUltimoOffAlta = 0;
static float excSocRef = 0.0f;

// ── Sessão de login por cookie ──────────────────────────────────────────────────
static char          sessTok[SESSAO_MAX][33];
static unsigned long sessUso[SESSAO_MAX];

static bool lerCookieSessao(char* out, size_t n) {
    out[0] = '\0';
    if (!server.hasHeader("Cookie")) return false;
    String ck = server.header("Cookie");
    int p = ck.indexOf("RRFS_SID=");
    if (p < 0) return false;
    p += 9;
    int e = ck.indexOf(';', p);
    String v = (e < 0) ? ck.substring(p) : ck.substring(p, e);
    v.trim();
    if (v.length() != 32) return false;
    strncpy(out, v.c_str(), n - 1);
    out[n - 1] = '\0';
    return true;
}

SessaoLogin::operator bool() const {
    char tok[33];
    if (!lerCookieSessao(tok, sizeof(tok))) return false;
    unsigned long agora = millis();
    for (int i = 0; i < SESSAO_MAX; i++) {
        if (sessTok[i][0] && strcmp(sessTok[i], tok) == 0) {
            if (agora - sessUso[i] > SESSAO_TIMEOUT_MS) { sessTok[i][0] = '\0'; return false; }
            sessUso[i] = agora;
            return true;
        }
    }
    return false;
}

SessaoLogin& SessaoLogin::operator=(bool v) {
    if (v) {
        uint32_t r[4];
        for (int i = 0; i < 4; i++) r[i] = esp_random();
        char tok[33];
        snprintf(tok, sizeof(tok), "%08x%08x%08x%08x",
                 (unsigned)r[0], (unsigned)r[1], (unsigned)r[2], (unsigned)r[3]);
        unsigned long agora = millis();
        int slot = -1; unsigned long maiorIdade = 0;
        for (int i = 0; i < SESSAO_MAX; i++) {
            if (!sessTok[i][0]) { slot = i; break; }
            unsigned long idade = agora - sessUso[i];
            if (slot < 0 || idade > maiorIdade) { slot = i; maiorIdade = idade; }
        }
        strncpy(sessTok[slot], tok, 32);
        sessTok[slot][32] = '\0';
        sessUso[slot] = agora;
        server.sendHeader("Set-Cookie", String("RRFS_SID=") + tok + "; Path=/; HttpOnly; SameSite=Lax");
    } else {
        char tok[33];
        if (lerCookieSessao(tok, sizeof(tok))) {
            for (int i = 0; i < SESSAO_MAX; i++)
                if (sessTok[i][0] && strcmp(sessTok[i], tok) == 0) sessTok[i][0] = '\0';
        }
        server.sendHeader("Set-Cookie", "RRFS_SID=; Path=/; Max-Age=0");
    }
    return *this;
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 06 — PROTÓTIPOS
// ═══════════════════════════════════════════════════════════════════════════════════

// Logs
void adicionarLog(const char* msg);
void logRAM();
void carregarLogsPersistentes();
void salvarLogNVS(uint8_t pos, const char* msg);
void salvarLogsPendentes(bool forcar = false);

// LED e Botão
void setupLED(); void atualizarLED(); void ledBlink(); void ledFixo(bool v);
void setupBotao(); void verificarReset();

// Relés
void setupReles();
void ligarInvBaixa(); void desligarInvBaixa();
void ligarInvAlta();  void desligarInvAlta();
void ligarLampExt();  void desligarLampExt();

// ADC e Sensores
int   lerADC(uint8_t pin);
float adicionarAmostraCorr(BufferCorr& buf, float nova);
float lerCorrentePainel();
void  calcularDadosBanco();
void  lerSensores();

// BLE BMS
void   salvarConfigBMS(); void carregarConfigBMS();
void   salvarConfigBanco(); void carregarConfigBanco();
bool   parsearDadosBMS(const uint8_t* data, size_t len);
void   bleNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);
void   conectarBMS(); void desconectarBMS(); void gerenciarBLE();
bool   bleSolicitarDados();
String statusBLEStr();
void   bleIniciar();
void   executarScanLista();
static bool bleDescobrirCaracteristicas();
static void bleRegistrar(BLERemoteCharacteristic* ch);

// Sinais analógicos
void processarSinaisAnalogicos();
bool acBaixaPresente(); bool acAltaPresente(); bool sinalCasaPresente();

// NTP
void sincronizarNTP();
bool obterTempoAtual(struct tm& t);
String formatarDataHora(); String formatarHora();
void salvarConfigTempo(); void carregarConfigTempo();

// Previsão Solar
void salvarConfigPrevisao(); void carregarConfigPrevisao();
bool buscarPrevisaoOpenMeteo();
void calcularAhPrevisto();
void calibrarPrevisao();
void carregarEstatPrev();
float getAhPrevHoraAtual(); float getAhPrevRestanteDia();
int   getIndiceHoraAtual();
void  gerenciarPrevisao();

// Histórico
void calcularTotaisDia(RegistroDia& dia);
void salvarContadores(); void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico(); void carregarHistorico();
void acumularEnergia(); void atualizarDiaAtual();

// Padrões de consumo (v32)
void padFsIniciar();
void padCarregarArquivo();
void padroesSalvarSeSujo(bool forcar = false);
void padroesApagarTudo();
void padroesSincronizarDia(uint16_t a, uint8_t m, uint8_t d);
void atualizarPadroes();
void padroesRecalcular();
void padroesPrever(int hAgora, int minAgora);
int  padAchaCluster(float media, float durS, bool criar);

// Controle
void maquinaInvBaixa(); void maquinaInvAlta();
void controlarLampExt(); void controlarEnergia();
EstimativaTempo calcularTempoRestante();

// Proteção
bool bmsFresca();
void salvarConfigProtecao(); void carregarConfigProtecao(); void atualizarProtecao();

// Preferências
bool existeCredenciais(); bool existeWiFiConfig();
void salvarCredenciais(const String& u, const String& p);
bool verificarLogin(const String& u, const String& p);
void salvarWifi(const String& ssid, const String& pass, const String& ip, const String& gw, const String& mask, const String& dns, bool ap);
void carregarWifi(String& ssid, String& pass, String& ip, String& gw, String& mask, String& dns, bool& ap);
void salvarConfigInversores(); void carregarConfigInversores();
void salvarLimiaresSinal(); void carregarLimiaresSinal();
void salvarLimiares(); void carregarLimiares();
void carregarCalibSensores();

// WiFi
void iniciarAP(); void iniciarAPEmergencial();
bool iniciarWiFiNB(const String& ssid, const String& pass, const String& ip, const String& gw, const String& mask, const String& dns);
void gerenciarWiFi();

// Web Server
void configurarRotas();
String gerarPagina(const String& titulo, const String& conteudo, bool comNav = true);
String barraProgresso(float pct, const String& label = "");
String nomeEstadoBaixa(EstadoBaixa e);
String nomeEstadoAlta(EstadoAlta e);
void addf(String& c, const char* fmt, ...);
void htmlEsc(const char* s, String& out);
void paginaInicio(const char* titulo);

// OTA
void handleOTA(); void handleOTAUpload(); void handleOTAPrep();
void handleOTAEstado(); void handleOTAVersao(); void handleOTAFim();
void otaPrepararBLE();

void verificarTimeoutTeste();

// Excesso solar
void atualizarExcesso();
void handleExcesso();
bool altaRepousoOK();

// Saúde da bateria
void carregarSaudeBateria();
void atualizarSaudeBateria();
void salvarSaudeSeSujo(bool forcar = false);

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 07 — LOGS PERSISTENTES
// ═══════════════════════════════════════════════════════════════════════════════════

void carregarLogsPersistentes() {
    if (logsCarregados) return;
    prefs.begin(LOG_NVS_NS, false);
    if (prefs.getUChar("ver", 0) != 2) {
        prefs.clear();
        prefs.putUChar("ver", 2);
    }
    uint16_t total = prefs.getUShort("total", 0);
    uint8_t  idx   = prefs.getUChar ("idx",   0);
    totalLogs = total;
    logIndex  = (idx < MAX_LOGS) ? idx : 0;
    uint16_t ini = (total > LOG_NVS_MAX) ? (total - LOG_NVS_MAX) : 0;
    for (uint16_t i = ini; i < total; i++) {
        char chave[8];
        snprintf(chave, sizeof(chave), "l%02u", (unsigned)(i % LOG_NVS_MAX));
        String s = prefs.getString(chave, "");
        if (s.length() > 0) {
            uint8_t pos = i % MAX_LOGS;
            strncpy(logs[pos], s.c_str(), LOG_MAX_LEN - 1);
            logs[pos][LOG_MAX_LEN - 1] = '\0';
        }
    }
    prefs.end();
    logSalvoTotal  = totalLogs;
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
    char entrada[LOG_MAX_LEN];
    portENTER_CRITICAL(&logMux);
    totalLogs++;
    snprintf(entrada, sizeof(entrada), "[%05d] %s", totalLogs, msg);
    uint8_t pos = logIndex;
    strncpy(logs[pos], entrada, LOG_MAX_LEN - 1);
    logs[pos][LOG_MAX_LEN - 1] = '\0';
    logIndex = (logIndex + 1) % MAX_LOGS;
    portEXIT_CRITICAL(&logMux);
}

void salvarLogsPendentes(bool forcar) {
    static unsigned long tUlt = 0;
    unsigned long agora = millis();
    if (!forcar && agora - tUlt < 10000UL) return;
    tUlt = agora;

    uint16_t total; uint8_t idx;
    portENTER_CRITICAL(&logMux);
    total = totalLogs;
    idx   = logIndex;
    portEXIT_CRITICAL(&logMux);

    if (total == logSalvoTotal) return;

    uint32_t ini = logSalvoTotal;
    if ((uint32_t)(total - logSalvoTotal) > LOG_NVS_MAX) ini = total - LOG_NVS_MAX;

    prefs.begin(LOG_NVS_NS, false);
    for (uint32_t n = ini + 1; n <= (uint32_t)total; n++) {
        uint8_t posRam = (n - 1) % MAX_LOGS;
        char chave[8];
        snprintf(chave, sizeof(chave), "l%02u", (unsigned)((n - 1) % LOG_NVS_MAX));
        char copia[LOG_MAX_LEN];
        portENTER_CRITICAL(&logMux);
        memcpy(copia, logs[posRam], LOG_MAX_LEN);
        portEXIT_CRITICAL(&logMux);
        copia[LOG_MAX_LEN - 1] = '\0';
        prefs.putString(chave, copia);
    }
    prefs.putUShort("total", total);
    prefs.putUChar ("idx",   idx);
    prefs.end();
    logSalvoTotal = total;
}

void logRAM() {
    nvs_stats_t st;
    unsigned nu = 0, nt = 0;
    if (nvs_get_stats(NULL, &st) == ESP_OK) { nu = st.used_entries; nt = st.total_entries; }
    snprintf(tempBuf, sizeof(tempBuf), "RAM %.0fK max%.0fK r%d NVS%u/%u",
             esp_get_free_heap_size() / 1024.0f,
             heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024.0f,
             (int)esp_reset_reason(), nu, nt);
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 08 — LED E BOTÃO
// ═══════════════════════════════════════════════════════════════════════════════════

void setupLED()  { pinMode(LED_AZUL, OUTPUT); digitalWrite(LED_AZUL, LOW); }

void atualizarLED() {
    if (!blinkAtivo) return;
    if (millis() - ultimoBlink >= 500) {
        ultimoBlink = millis();
        ledState    = !ledState;
        digitalWrite(LED_AZUL, ledState);
    }
}

void ledBlink()      { blinkAtivo = true; ultimoBlink = millis(); }
void ledFixo(bool v) { blinkAtivo = false; digitalWrite(LED_AZUL, v); }

void setupBotao() { pinMode(BOTAO_BOOT, INPUT_PULLUP); }

void verificarReset() {
    bool p = (digitalRead(BOTAO_BOOT) == LOW);
    if (p && !botaoSegurando) {
        botaoSegurando = true;
        botaoPressionadoEm = millis();
    }
    if (!p) botaoSegurando = false;
    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET TOTAL botao");
        for (int i = 0; i < 15; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(80);
            digitalWrite(LED_AZUL, LOW);  delay(80);
        }
        if (padFsOk) LittleFS.remove(PAD_ARQ);
        nvs_flash_erase();
        nvs_flash_init();
        ESP.restart();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 09 — RELÉS
// ═══════════════════════════════════════════════════════════════════════════════════

void setupReles() {
    pinMode(PIN_RELE_INV_BAIXA, OUTPUT);
    pinMode(PIN_RELE_LAMP_EXT,  OUTPUT);
    pinMode(PIN_RELE_INV_ALTA,  OUTPUT);
    digitalWrite(PIN_RELE_INV_BAIXA, cfgInv.invertReleInvBaixa ? HIGH : LOW);
    digitalWrite(PIN_RELE_LAMP_EXT,  cfgInv.invertReleLampExt  ? HIGH : LOW);
    digitalWrite(PIN_RELE_INV_ALTA,  cfgInv.invertReleInvAlta  ? HIGH : LOW);
    estados = {false, false, false};
    adicionarLog("Reles OFF v32");
}

void ligarInvBaixa()    {
    digitalWrite(PIN_RELE_INV_BAIXA, cfgInv.invertReleInvBaixa ? LOW  : HIGH);
    estados.invBaixa = true;
    adicionarLog("InvBaixa ON");
}

void desligarInvBaixa() {
    digitalWrite(PIN_RELE_INV_BAIXA, cfgInv.invertReleInvBaixa ? HIGH : LOW);
    estados.invBaixa = false;
    adicionarLog("InvBaixa OFF");
}

void ligarInvAlta()    {
    digitalWrite(PIN_RELE_INV_ALTA, cfgInv.invertReleInvAlta ? LOW  : HIGH);
    estados.invAlta = true;
    adicionarLog("InvAlta ON");
}

void desligarInvAlta() {
    digitalWrite(PIN_RELE_INV_ALTA, cfgInv.invertReleInvAlta ? HIGH : LOW);
    if (estados.invAlta) {
        tUltimoOffAlta = millis();
        if (tUltimoOffAlta == 0) tUltimoOffAlta = 1;
    }
    estados.invAlta = false;
    adicionarLog("InvAlta OFF");
}

bool altaRepousoOK() {
    return (tUltimoOffAlta == 0) || (millis() - tUltimoOffAlta >= ALTA_MIN_OFF_MS);
}

void ligarLampExt()    {
    digitalWrite(PIN_RELE_LAMP_EXT, cfgInv.invertReleLampExt ? LOW  : HIGH);
    estados.lampExt = true;
}

void desligarLampExt() {
    digitalWrite(PIN_RELE_LAMP_EXT, cfgInv.invertReleLampExt ? HIGH : LOW);
    estados.lampExt = false;
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 10 — SENSOR ADC PAINEL + BANCO + LEITURA CONSOLIDADA
// ═══════════════════════════════════════════════════════════════════════════════════

int lerADC(uint8_t pin) {
    long soma = 0;
    for (int i = 0; i < 30; i++) soma += analogRead(pin);
    return (int)(soma / 30);
}

float adicionarAmostraCorr(BufferCorr& buf, float nova) {
    buf.amostras[buf.indice] = nova;
    buf.indice = (buf.indice + 1) % BUFFER_CORRENTE;
    if (buf.total < BUFFER_CORRENTE) buf.total++;
    float soma = 0;
    for (uint8_t i = 0; i < buf.total; i++) soma += buf.amostras[i];
    return soma / buf.total;
}

float lerCorrentePainel() {
    int raw = lerADC(PIN_SENSOR_PAINEL);
    int mv  = (int)((long)raw * ADC_VREF / ADC_BITS);
    sensores.sinalPainel_mV = mv;
    float corrBruta = -(float)(mv - calibPainel.offsetMV) / calibPainel.sensMvA;
    if (corrBruta < 0.15f) corrBruta = 0.0f;
    float media = adicionarAmostraCorr(bufPainel, corrBruta);
    return (media < 0.15f) ? 0.0f : media;
}

// Banco: 1=unico | 2=paralelo (I,Cap x nP) | 3=serie (V x nS) | 4=serie+paralelo
void calcularDadosBanco() {
    if (!dadosBMS.valido) return;

    float v = dadosBMS.tensao, i = dadosBMS.corrente, cap = dadosBMS.capacidadeAh;
    float nS = (float)banco.nS, nP = (float)banco.nP;

    switch (banco.modo) {
        case 2:  i *= nP; cap *= nP; break;
        case 3:  v *= nS; break;
        case 4:  v *= nS; i *= nP; cap *= nP; break;
        default: break;
    }
    dadosBanco.tensaoTotal     = v;
    dadosBanco.correnteTotal   = i;
    dadosBanco.capacidadeTotal = cap;
    dadosBanco.socPct          = dadosBMS.soc;
    dadosBanco.ciclos          = dadosBMS.ciclos;
    dadosBanco.potencia        = v * i;
}

void lerSensores() {
    esp_task_wdt_reset();

    sensores.correntePainel = lerCorrentePainel();

    bool bmsOK = dadosBMS.valido &&
                 (millis() - dadosBMS.tUltimaDado < BMS_DADO_VALIDO_MS);

    if (bmsOK) {
        calcularDadosBanco();

        float corrFilt = adicionarAmostraCorr(bufBateria, dadosBanco.correnteTotal);
        if (fabsf(corrFilt) < BMS_FILTRO_RUIDO_A) corrFilt = 0.0f;

        sensores.correnteBateria  = corrFilt;
        sensores.tensaoBanco      = dadosBanco.tensaoTotal;
        sensores.temperaturaBMS   = dadosBMS.temperatura;
        sensores.socBMS           = dadosBMS.soc;
        sensores.ciclosBMS        = dadosBMS.ciclos;
        sensores.capacidadeBanco  = dadosBanco.capacidadeTotal;
    } else if (millis() - dadosBMS.tUltimaDado > 30000UL) {
        sensores.correnteBateria = 0.0f;
    }

    float casa = sensores.correntePainel - sensores.correnteBateria;
    if (casa < 0.0f) casa = 0.0f;
    sensores.correnteCasa = casa;

    float vBanco = (sensores.tensaoBanco > 1.0f) ? sensores.tensaoBanco : 12.0f;
    sensores.potenciaPainel  = sensores.correntePainel  * vBanco;
    sensores.potenciaBateria = sensores.correnteBateria * vBanco;
    sensores.potenciaCasa    = sensores.correnteCasa    * vBanco;

    esp_task_wdt_reset();

    if (millis() - ultimoHistSnap >= 1000) {
        ultimoHistSnap = millis();
        histCorrPainel[histSnapIdx] = sensores.correntePainel;
        histCorrCasa  [histSnapIdx] = sensores.correnteCasa;
        histCorrBat   [histSnapIdx] = sensores.correnteBateria;
        histSnapIdx = (histSnapIdx + 1) % HIST_SNAP_PONTOS;
        if (histSnapIdx == 0) histSnapFull = true;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 11 — BLE BMS (JBD/LanmaoBMS) — NIMBLE
// ═══════════════════════════════════════════════════════════════════════════════════

void salvarConfigBMS() {
    prefs.begin("rrfs_bms", false);
    prefs.putString("mac", cfgBMS.macAddress);
    prefs.putBool  ("cfg", cfgBMS.configurada);
    prefs.putUChar ("ver", 25);
    prefs.end();
    adicionarLog("BMS config salva");
}

void carregarConfigBMS() {
    prefs.begin("rrfs_bms", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver == 25) {
        String mac = prefs.getString("mac", "");
        strncpy(cfgBMS.macAddress, mac.c_str(), sizeof(cfgBMS.macAddress)-1);
        cfgBMS.macAddress[sizeof(cfgBMS.macAddress)-1] = '\0';
        cfgBMS.configurada = prefs.getBool("cfg", false);
    } else {
        cfgBMS.configurada = false;
        cfgBMS.macAddress[0] = '\0';
    }
    prefs.end();
    if (cfgBMS.configurada)
        adicionarLog(("BMS MAC: " + String(cfgBMS.macAddress)).c_str());
}

void salvarConfigBanco() {
    prefs.begin("rrfs_banco", false);
    prefs.putUChar("nS",   banco.nS);
    prefs.putUChar("nP",   banco.nP);
    prefs.putUChar("modo", banco.modo);
    prefs.putBool ("cfg",  banco.configurado);
    prefs.putUChar("ver",  26);
    prefs.end();

    char buf[96];
    snprintf(buf, sizeof(buf), "Banco salvo: Modo%d | %dS%dP (%d baterias)",
             banco.modo, banco.nS, banco.nP, banco.nS * banco.nP);
    adicionarLog(buf);
}

void carregarConfigBanco() {
    prefs.begin("rrfs_banco", true);
    uint8_t ver = prefs.getUChar("ver", 0);

    if (ver == 26) {
        banco.nS          = prefs.getUChar("nS",   1);
        banco.nP          = prefs.getUChar("nP",   1);
        banco.modo        = prefs.getUChar("modo", 1);
        banco.configurado = prefs.getBool ("cfg",  false);
    } else {
        banco = {1, 1, 1, false};
    }

    if (banco.nS < 1 || banco.nS > 8) banco.nS = 1;
    if (banco.nP < 1 || banco.nP > 8) banco.nP = 1;
    if (banco.modo < 1 || banco.modo > 4) banco.modo = 1;

    prefs.end();

    if (banco.configurado) {
        char buf[96];
        snprintf(buf, sizeof(buf), "Banco carregado: Modo%d | %dS%dP",
                 banco.modo, banco.nS, banco.nP);
        adicionarLog(buf);
    }
}

static uint16_t crc16Modbus(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static void montarPedido(uint8_t* out, uint16_t reg, uint16_t qtd) {
    out[0] = 0xA5; out[1] = 0xA5; out[2] = 0x00; out[3] = 0x03;
    out[4] = (uint8_t)(reg >> 8); out[5] = (uint8_t)(reg & 0xFF);
    out[6] = (uint8_t)(qtd >> 8); out[7] = (uint8_t)(qtd & 0xFF);
    uint16_t c = crc16Modbus(out + 2, 6);
    out[8] = (uint8_t)(c & 0xFF);
    out[9] = (uint8_t)(c >> 8);
}

bool parsearDadosBMS(const uint8_t* d, size_t len) {
    if (len < 9) return false;
    if (d[0] != 0xA5 || d[1] != 0xA5 || d[3] != 0x03) return false;

    size_t bc    = d[4];
    size_t total = 5 + bc + 2;
    if (len < total) return false;

    uint16_t c = crc16Modbus(d + 2, total - 4);
    bool crcOk = (d[total - 2] == (c & 0xFF)) && (d[total - 1] == (c >> 8));
    static bool avisouCrc = false;
    if (!crcOk && !avisouCrc) {
        avisouCrc = true;
        adicionarLog("BMS: CRC difere (aceito)");
    }

    const uint8_t* r = d + 5;
    auto u16 = [&](int i) -> uint16_t {
        return (uint16_t)(((uint16_t)r[2 * i] << 8) | r[2 * i + 1]);
    };

    // JANELA A: regs 0..5 (corrente, tensao, SOC)
    if (bc == 12) {
        int32_t iRaw = (int32_t)(((uint32_t)u16(0) << 16) | u16(1));
        float corr = iRaw * 0.01f;
        int32_t vRaw = (int32_t)(((uint32_t)u16(2) << 16) | u16(3));
        float tens = vRaw * 0.01f;
        uint16_t soc = u16(4);

        if (tens < 0.0f || tens > 200.0f) return false;
        if (corr < -500.0f || corr > 500.0f) return false;
        if (soc > 100) return false;

        dadosBMS.corrente    = corr;
        dadosBMS.tensao      = tens;
        dadosBMS.soc         = (float)soc;
        dadosBMS.valido      = true;
        dadosBMS.tUltimaDado = millis();
        return true;
    }

    // JANELA B: regs 10..12 (capacidade, ciclos)
    if (bc == 6) {
        uint32_t capRaw = ((uint32_t)u16(0) << 16) | u16(1);
        float cap = capRaw * 0.01f;
        uint16_t ciclos = u16(2);
        if (cap > 100000.0f) return false;
        dadosBMS.capacidadeAh = cap;
        dadosBMS.ciclos       = ciclos;
        return true;
    }
    return false;
}

void bleNotifyCallback(BLERemoteCharacteristic* pChar,
                       uint8_t* pData, size_t length, bool isNotify) {
    if (length == 0) return;

    bleRxTotal++;

    if (bleRxLogRest > 0 && !bleRxPend) {
        uint8_t n = (length > sizeof(bleRxDump)) ? sizeof(bleRxDump) : (uint8_t)length;
        memcpy(bleRxDump, pData, n);
        bleRxDumpLen = n;
        bleRxPend    = true;
    }

    if (length > 64) return;

    if (ctrlBLE.bufferLen == 0) {
        if (!(length >= 2 && pData[0] == 0xA5 && pData[1] == 0xA5)) return;
    }

    if (ctrlBLE.bufferLen + length > sizeof(ctrlBLE.bufferRX)) {
        ctrlBLE.bufferLen = 0;
        return;
    }
    memcpy(ctrlBLE.bufferRX + ctrlBLE.bufferLen, pData, length);
    ctrlBLE.bufferLen += length;

    if (ctrlBLE.bufferLen < 5) return;

    size_t esperado = 5 + (size_t)ctrlBLE.bufferRX[4] + 2;
    if (esperado > sizeof(ctrlBLE.bufferRX)) {
        ctrlBLE.bufferLen = 0;
        return;
    }
    if (ctrlBLE.bufferLen < esperado) return;

    bool ok = parsearDadosBMS(ctrlBLE.bufferRX, ctrlBLE.bufferLen);
    ctrlBLE.bufferLen = 0;
    if (ok) ctrlBLE.falhasConsec = 0;   // v32: sem log periodico "BMS OK"
}

bool bleSolicitarDados() {
    if (ctrlBLE.estado != BLE_CONECTADO) return false;
    if (!bleCharTX || !bleClient || !bleClient->isConnected()) return false;

    static uint8_t janela = 0;
    uint8_t pedido[10];

    if (janela == 0) montarPedido(pedido, 0, 6);
    else             montarPedido(pedido, 10, 3);
    janela ^= 1;

    try {
        bleCharTX->writeValue(pedido, sizeof(pedido), false);
        return true;
    } catch (...) {
        adicionarLog("BMS: erro write");
        return false;
    }
}

String statusBLEStr() {
    if (bleManualOff && ctrlBLE.estado != BLE_CONECTADO)
        return F("Desligado (manual)");
    switch (ctrlBLE.estado) {
        case BLE_DESCONECTADO: return F("Desconectado");
        case BLE_ESCANEANDO:   return F("Escaneando");
        case BLE_CONECTANDO:   return F("Conectando");
        case BLE_CONECTADO:    return F("Conectado");
        case BLE_FALHA:        return F("Falha");
        default:               return F("?");
    }
}

class BLECallbackScan : public BLEAdvertisedDeviceCallbacks {
    void onResult(const BLEAdvertisedDevice* device) override {
        bleVistos++;

        String mac = String(device->getAddress().toString().c_str());
        mac.toUpperCase();

        if (bleModoLista) {
            for (uint8_t i = 0; i < bleListaN; i++) {
                if (strcmp(bleLista[i].mac, mac.c_str()) == 0) {
                    bleLista[i].rssi = device->getRSSI();
                    return;
                }
            }
            if (bleListaN >= BLE_LISTA_MAX) return;

            String nm = device->haveName() ? String(device->getName().c_str()) : String("");
            String nl = nm; nl.toLowerCase();
            bool jbd = device->isAdvertisingService(BLEUUID(BMS_SERVICE_UUID)) ||
                       nl.indexOf("bms") >= 0 || nl.indexOf("jbd") >= 0 ||
                       nl.indexOf("xiaoxiang") >= 0 || nl.indexOf("lanmao") >= 0;
            if (nm.length() == 0 && !jbd) return;

            BleItem& it = bleLista[bleListaN];
            strncpy(it.mac, mac.c_str(), sizeof(it.mac) - 1);
            it.mac[sizeof(it.mac) - 1] = '\0';
            if (nm.length() == 0) nm = "(sem nome)";
            strncpy(it.nome, nm.c_str(), sizeof(it.nome) - 1);
            it.nome[sizeof(it.nome) - 1] = '\0';
            it.rssi = device->getRSSI();
            it.jbd  = jbd;
            bleListaN++;
            return;
        }

        if (!cfgBMS.configurada) return;
        if (!bleEncontrou && mac == String(cfgBMS.macAddress)) {
            bleEncontrou   = true;
            if (bleDispositivo) { delete bleDispositivo; bleDispositivo = nullptr; }
            bleDispositivo = new BLEAdvertisedDevice(*device);
            bleScan->stop();
        }
    }
};

// NimBLE-Arduino 2.x: onDisconnect tem o parametro "int reason"
class BLEClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* c) override {}
    void onDisconnect(BLEClient* c, int reason) override {
        ctrlBLE.estado           = BLE_DESCONECTADO;
        ctrlBLE.notificacaoAtiva = false;
        ctrlBLE.bufferLen        = 0;
        snprintf(tempBuf, sizeof(tempBuf), "BMS: BLE desconectado (motivo %d)", reason);
        adicionarLog(tempBuf);
    }
};

void bleIniciar() {
    if (bleIniciado) return;
    BLEDevice::init("RRFS");
    bleScan = BLEDevice::getScan();
    static BLECallbackScan cbScan;
    bleScan->setScanCallbacks(&cbScan, false);
    bleScan->setActiveScan(true);
    bleScan->setInterval(160);
    bleScan->setWindow(40);
    bleIniciado = true;
    logRAM();
}

static void wdtTimeout(uint32_t ms) {
    esp_task_wdt_config_t c = {
        .timeout_ms     = ms,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_reconfigure(&c);
}

void executarScanLista() {
    bleIniciar();
    bleListaN    = 0;
    bleVistos    = 0;
    bleModoLista = true;
    bleScanAtivo = true;
    adicionarLog("BLE: buscando dispositivos");

    bleScan->stop();
    bleScan->clearResults();
    bleScan->start((uint32_t)BLE_SCAN_LISTA_S * 1000UL, false);   // NimBLE 2.x: ms

    unsigned long tFimScan = millis() + (unsigned long)BLE_SCAN_LISTA_S * 1000UL;
    while (millis() < tFimScan) {
        esp_task_wdt_reset();
        server.handleClient();
        delay(50);
    }
    bleScan->stop();

    bleModoLista = false;
    bleScanAtivo = false;
    snprintf(tempBuf, sizeof(tempBuf), "BLE: %d lista / %u vistos",
             (int)bleListaN, (unsigned)bleVistos);
    adicionarLog(tempBuf);
}

static bool bleDescobrirCaracteristicas() {
    bleService = nullptr;
    bleCharRX = nullptr; bleCharRX2 = nullptr;
    bleCharTX = nullptr; bleCharTX2 = nullptr;

    BLERemoteService* s = bleClient->getService(
        BLEUUID("0000fffa-0000-1000-8000-00805f9b34fb"));

    if (!s) {
        const std::vector<BLERemoteService*>& servicos = bleClient->getServices(true);
        for (auto& srv : servicos) {
            String u = String(srv->getUUID().toString().c_str());
            u.toLowerCase();
            if (u.indexOf("fffa") >= 0) { s = srv; break; }
        }
    }

    if (!s) { adicionarLog("BMS: serviço FFFA não achado"); return false; }

    BLERemoteCharacteristic* tx = s->getCharacteristic(
        BLEUUID("0000fffb-0000-1000-8000-00805f9b34fb"));
    BLERemoteCharacteristic* rx = s->getCharacteristic(
        BLEUUID("0000fffc-0000-1000-8000-00805f9b34fb"));

    if (!tx) { adicionarLog("BMS: FFFB (TX) não achada"); return false; }
    if (!rx) { adicionarLog("BMS: FFFC (RX) não achada"); return false; }
    if (!rx->canNotify()) { adicionarLog("BMS: FFFC sem notify"); return false; }

    bleService = s;
    bleCharTX = tx;
    bleCharRX = rx;
    return true;
}

static void bleRegistrar(BLERemoteCharacteristic* ch) {
    if (!ch) return;
    bool ind = !ch->canNotify() && ch->canIndicate();
    bool ok  = ch->subscribe(!ind, bleNotifyCallback, true);
    if (!ok) { adicionarLog("BMS: falha ao assinar notify/indicate"); return; }
    adicionarLog("BMS: CCCD ativado");
}

void conectarBMS() {
    if (!cfgBMS.configurada || cfgBMS.macAddress[0] == '\0') return;
    if (ctrlBLE.estado == BLE_CONECTADO || ctrlBLE.estado == BLE_CONECTANDO) return;

    bleIniciar();
    adicionarLog("BMS: iniciando scan BLE");

    ctrlBLE.estado    = BLE_ESCANEANDO;
    ctrlBLE.bufferLen = 0;
    bleEncontrou      = false;
    bleModoLista      = false;
    bleVistos         = 0;

    if (bleDispositivo) { delete bleDispositivo; bleDispositivo = nullptr; }

    bleScan->stop();
    bleScan->clearResults();
    bleScan->start((uint32_t)BLE_SCAN_DURACAO_S * 1000UL, false);

    unsigned long tFimScan = millis() + (unsigned long)BLE_SCAN_DURACAO_S * 1000UL;
    while (millis() < tFimScan && !bleEncontrou) {
        esp_task_wdt_reset();
        server.handleClient();
        delay(50);
    }
    bleScan->stop();

    if (!bleEncontrou || !bleDispositivo) {
        ctrlBLE.estado = BLE_FALHA;
        ctrlBLE.falhasConsec++;
        snprintf(tempBuf, sizeof(tempBuf), "BMS: nao achada (vi %u disp)", (unsigned)bleVistos);
        adicionarLog(tempBuf);
        return;
    }

    adicionarLog("BMS: encontrada, conectando");

    ctrlBLE.estado      = BLE_CONECTANDO;
    ctrlBLE.tConectando = millis();

    if (!bleClient) {
        bleClient = BLEDevice::createClient();
        static BLEClientCallback cbCliente;
        bleClient->setClientCallbacks(&cbCliente, false);
        bleClient->setConnectTimeout(10000);
    } else if (bleClient->isConnected()) {
        bleClient->disconnect();
    }

    wdtTimeout(60000);
    esp_task_wdt_reset();
    bool conectou = bleClient->connect(bleDispositivo);
    esp_task_wdt_reset();
    wdtTimeout(30000);

    if (!conectou) {
        ctrlBLE.estado = BLE_FALHA;
        ctrlBLE.falhasConsec++;
        adicionarLog("BMS: falha conexão BLE");
        return;
    }

    wdtTimeout(60000);
    bool achou = bleDescobrirCaracteristicas();
    wdtTimeout(30000);
    esp_task_wdt_reset();

    if (!achou || !bleCharRX || !bleCharTX) {
        bleClient->disconnect();
        ctrlBLE.estado = BLE_FALHA;
        ctrlBLE.falhasConsec++;
        adicionarLog("BMS: serviço/características não achados");
        return;
    }

    bleRxTotal   = 0;
    bleRxPend    = false;
    bleRxLogRest = 3;

    bleRegistrar(bleCharRX);
    ctrlBLE.notificacaoAtiva = true;

    bleTxResposta = false;
    bleTxAmbos    = false;
    bleCharTX2    = nullptr;
    bleCharRX2    = nullptr;

    for (int i = 0; i < 3; i++) { delay(100); esp_task_wdt_reset(); }

    ctrlBLE.estado         = BLE_CONECTADO;
    ctrlBLE.falhasConsec   = 0;
    ctrlBLE.tUltimaLeitura = 0;
    ctrlBLE.tConectando    = millis();

    adicionarLog("BMS: BLE conectado OK");
    logRAM();
}

void desconectarBMS() {
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    ctrlBLE.estado           = BLE_DESCONECTADO;
    ctrlBLE.notificacaoAtiva = false;
    ctrlBLE.bufferLen        = 0;
}

void gerenciarBLE() {
    if (bleRxPend) {
        char b[56];
        int p = snprintf(b, sizeof(b), "RX%u:", (unsigned)bleRxDumpLen);
        for (uint8_t i = 0; i < bleRxDumpLen && p < (int)sizeof(b) - 3; i++)
            p += snprintf(b + p, sizeof(b) - p, "%02X", bleRxDump[i]);
        adicionarLog(b);
        if (bleRxLogRest > 0) bleRxLogRest--;
        bleRxPend = false;
    }

    if (bleScanSolicitado) {
        bleScanSolicitado = false;
        executarScanLista();
        return;
    }

    if (!cfgBMS.configurada) return;
    if (bleManualOff) return;

    unsigned long agora = millis();
    bool emTeste = (bleTesteAte != 0 && (long)(bleTesteAte - agora) > 0);

    if (ctrlBLE.estado == BLE_CONECTADO) {
        if (emTeste) return;

        if (agora - ctrlBLE.tConectando > 20000UL &&
            (dadosBMS.tUltimaDado == 0 ||
             agora - dadosBMS.tUltimaDado > 20000UL)) {
            char b[60];
            snprintf(b, sizeof(b), "BMS: sem dados (rx=%u)", (unsigned)bleRxTotal);
            adicionarLog(b);
            desconectarBMS();
            ctrlBLE.falhasConsec++;
            return;
        }

        if (agora - ctrlBLE.tUltimaLeitura >= BLE_LEITURA_MS) {
            ctrlBLE.tUltimaLeitura = agora;
            esp_task_wdt_reset();
            server.handleClient();
            if (!bleSolicitarDados()) {
                ctrlBLE.estado = BLE_DESCONECTADO;
                ctrlBLE.falhasConsec++;
                adicionarLog("BMS: erro solicitação");
            }
            esp_task_wdt_reset();
        }
        return;
    }

    if (ctrlBLE.estado == BLE_FALHA || ctrlBLE.estado == BLE_DESCONECTADO) {
        unsigned long intervalo = BLE_RECONECT_MS;
        if (ctrlBLE.falhasConsec >  5) intervalo =  60000UL;
        if (ctrlBLE.falhasConsec > 10) intervalo = 300000UL;

        if (agora - ctrlBLE.tUltimoTentativa >= intervalo) {
            ctrlBLE.tUltimoTentativa = agora;
            server.handleClient();
            esp_task_wdt_reset();
            conectarBMS();
            esp_task_wdt_reset();
            server.handleClient();
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 12 — SINAIS ANALÓGICOS: INV.BAIXA (35) + INV.ALTA (36) + SINAL DA CASA (39)
// ═══════════════════════════════════════════════════════════════════════════════════

static void atualizarSinalAC(SinalAC& s, int limiar,
                             unsigned long confLiga, unsigned long confDesliga,
                             const char* msgOn, const char* msgOff, unsigned long agora) {
    int limLiga = limiar + AC_HISTERESE_LIGA_MV;
    int limDesl = limiar - AC_HISTERESE_DESLIGA_MV;
    if (limDesl < 10) limDesl = 10;

    bool acima  = (s.mediaMV >= limLiga);
    bool abaixo = (s.mediaMV <  limDesl);

    if (!s.presente) {
        if (acima) {
            if (!s.aguardandoLigar) {
                s.aguardandoLigar    = true;
                s.tInicioConfirma    = agora;
                s.aguardandoDesligar = false;
            } else if (agora - s.tInicioConfirma >= confLiga) {
                s.presente        = true;
                s.aguardandoLigar = false;
                adicionarLog(msgOn);
            }
        } else {
            s.aguardandoLigar = false;
        }
    } else {
        if (abaixo) {
            if (!s.aguardandoDesligar) {
                s.aguardandoDesligar = true;
                s.tInicioConfirma    = agora;
                s.aguardandoLigar    = false;
            } else if (agora - s.tInicioConfirma >= confDesliga) {
                s.presente           = false;
                s.aguardandoDesligar = false;
                adicionarLog(msgOff);
            }
        } else {
            s.aguardandoDesligar = false;
        }
    }
}

void processarSinaisAnalogicos() {
    static float   bufBaixa[AC_BUFFER_SUAV] = {0};
    static float   bufAlta [AC_BUFFER_SUAV] = {0};
    static float   bufCasa [AC_BUFFER_SUAV] = {0};
    static uint8_t bufIdx = 0, bufTot = 0;

    unsigned long agora = millis();

    server.handleClient();
    if (modoAP || WiFi.getMode() == WIFI_AP_STA)
        dnsServer.processNextRequest();
    esp_task_wdt_reset();

    long somaBaixa = 0, somaAlta = 0, somaCasa = 0;
    int  contADC = 0;
    unsigned long tJanela = millis();
    while (millis() - tJanela < AC_JANELA_MS) {
        somaBaixa += analogRead(PIN_AC_SINAL_BAIXA);
        somaAlta  += analogRead(PIN_AC_SINAL_ALTA);
        somaCasa  += analogRead(PIN_SINAL_CASA);
        contADC++;
        delayMicroseconds(150);
        if (contADC % 20 == 0) esp_task_wdt_reset();
    }
    if (contADC == 0) return;

    float mvBaixa = ((float)somaBaixa / contADC * ADC_VREF) / ADC_BITS;
    float mvAlta  = ((float)somaAlta  / contADC * ADC_VREF) / ADC_BITS;
    float mvCasa  = ((float)somaCasa  / contADC * ADC_VREF) / ADC_BITS;

    server.handleClient();
    esp_task_wdt_reset();

    bufBaixa[bufIdx] = mvBaixa;
    bufAlta [bufIdx] = mvAlta;
    bufCasa [bufIdx] = mvCasa;
    bufIdx = (bufIdx + 1) % AC_BUFFER_SUAV;
    if (bufTot < AC_BUFFER_SUAV) bufTot++;

    float sB = 0, sA = 0, sC = 0;
    for (uint8_t i = 0; i < bufTot; i++) { sB += bufBaixa[i]; sA += bufAlta[i]; sC += bufCasa[i]; }
    acBaixa.mediaMV   = sB / bufTot;
    acAlta.mediaMV    = sA / bufTot;
    sinalCasa.mediaMV = sC / bufTot;

    atualizarSinalAC(acBaixa, limiarSinal.limiarAcMV,
                     AC_CONFIRMA_MS, AC_CONFIRMA_MS,
                     "AC Baixa: PRESENTE", "AC Baixa: QUEDA", agora);

    atualizarSinalAC(acAlta, limiarSinal.limiarAcAltaMV,
                     AC_ALTA_LIGA_CONFIRMA_MS, AC_ALTA_DESLIGA_CONFIRMA_MS,
                     "AC Alta: PRESENTE", "AC Alta: QUEDA (confirmada)", agora);

    atualizarSinalAC(sinalCasa, limiarSinal.limiarCasaMV,
                     CASA_LIGA_CONFIRMA_MS, CASA_DESLIGA_CONFIRMA_MS,
                     "Casa: PEDIU Inv.Alta", "Casa: SOLTOU pedido Inv.Alta", agora);
}

bool acBaixaPresente()   { return acBaixa.presente;   }
bool acAltaPresente()    { return acAlta.presente;    }
bool sinalCasaPresente() { return sinalCasa.presente; }

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 13 — SINCRONIZAÇÃO NTP E TEMPO
// ═══════════════════════════════════════════════════════════════════════════════════

void salvarConfigTempo() {
    prefs.begin("rrfs_time", false);
    prefs.putInt ("fuso_h",    cfgTempo.fusoHoras);
    prefs.putInt ("fuso_m",    cfgTempo.fusoMinutos);
    prefs.putBool("ntp_ativo", cfgTempo.ntpAtivo);
    prefs.putInt ("man_ano",   cfgTempo.manualAnno);
    prefs.putInt ("man_mes",   cfgTempo.manualMes);
    prefs.putInt ("man_dia",   cfgTempo.manualDia);
    prefs.putInt ("man_hora",  cfgTempo.manualHora);
    prefs.putInt ("man_min",   cfgTempo.manualMin);
    prefs.putUChar("ver", 25);
    prefs.end();
}

void carregarConfigTempo() {
    prefs.begin("rrfs_time", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver != 25) {
        prefs.end();
        cfgTempo = {-3, 0, false, false, 2024, 1, 1, 0, 0};
        return;
    }
    cfgTempo.fusoHoras    = prefs.getInt ("fuso_h",   -3);
    cfgTempo.fusoMinutos  = prefs.getInt ("fuso_m",    0);
    cfgTempo.ntpAtivo     = prefs.getBool("ntp_ativo", false);
    cfgTempo.manualAnno   = prefs.getInt ("man_ano",   2024);
    cfgTempo.manualMes    = prefs.getInt ("man_mes",   1);
    cfgTempo.manualDia    = prefs.getInt ("man_dia",   1);
    cfgTempo.manualHora   = prefs.getInt ("man_hora",  0);
    cfgTempo.manualMin    = prefs.getInt ("man_min",   0);
    cfgTempo.sincronizado = false;
    prefs.end();
}

void sincronizarNTP() {
    if (!cfgTempo.ntpAtivo) return;
    if (modoAP || WiFi.status() != WL_CONNECTED) {
        adicionarLog("NTP: sem WiFi");
        tUltimoNTP = millis() - INTERVALO_NTP_MS + 120000UL;
        return;
    }
    if (WiFi.RSSI() < -85) {
        adicionarLog("NTP: sinal fraco");
        tUltimoNTP = millis() - INTERVALO_NTP_MS + 120000UL;
        return;
    }

    adicionarLog("NTP: iniciando");
    long off = (long)cfgTempo.fusoHoras * 3600L + (long)cfgTempo.fusoMinutos * 60L;
    configTime(off, 0, NTP_SERVER1, NTP_SERVER2);

    unsigned long tInicio = millis();
    struct tm ti;
    bool ok = false;
    while (millis() - tInicio < 5000UL) {
        esp_task_wdt_reset();
        server.handleClient();
        if (modoAP || WiFi.getMode() == WIFI_AP_STA)
            dnsServer.processNextRequest();
        if (getLocalTime(&ti, 50)) { ok = true; break; }
        delay(50);
    }
    if (ok) {
        ntpSincronizado       = true;
        cfgTempo.sincronizado = true;
        tUltimoNTP            = millis();
        adicionarLog("NTP: OK");
    } else {
        tUltimoNTP = millis() - INTERVALO_NTP_MS + 120000UL;
        adicionarLog("NTP: timeout");
    }
}

bool obterTempoAtual(struct tm& t) {
    if (ntpSincronizado && cfgTempo.ntpAtivo) return getLocalTime(&t, 100);
    memset(&t, 0, sizeof(struct tm));
    t.tm_year = cfgTempo.manualAnno - 1900;
    t.tm_mon  = cfgTempo.manualMes  - 1;
    t.tm_mday = cfgTempo.manualDia;
    t.tm_hour = cfgTempo.manualHora;
    t.tm_min  = cfgTempo.manualMin;
    return true;
}

String formatarDataHora() {
    struct tm t;
    if (!obterTempoAtual(t)) return F("--/--/---- --:--");
    char buf[24];
    snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d",
             t.tm_mday, t.tm_mon+1, t.tm_year+1900, t.tm_hour, t.tm_min);
    return String(buf);
}

String formatarHora() {
    struct tm t;
    if (!obterTempoAtual(t)) return F("--:--");
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    return String(buf);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 14 — PREVISÃO SOLAR (Open-Meteo, 8 modelos com pesos + calibração)
// ═══════════════════════════════════════════════════════════════════════════════════
//
// Regra de ouro: o painel deve gerar IGUAL OU ACIMA do previsto. Gerar menos so e
// normal quando o painel esta LIMITADO PELA DEMANDA (bateria cheia / excesso solar
// ativo). Essas horas sao "censuradas": real < previsto nao pune modelo nem fator.
//
void salvarConfigPrevisao() {
    prefs.begin("rrfs_prev", false);
    prefs.putBool ("ativa",       cfgPrevisao.ativa);
    prefs.putFloat("lat",         cfgPrevisao.latitude);
    prefs.putFloat("lon",         cfgPrevisao.longitude);
    prefs.putFloat("painel_kw",   cfgPrevisao.painelKW);
    prefs.putFloat("fator_calib", cfgPrevisao.fatorCalib);
    prefs.putFloat("ctrl_a",      cfgPrevisao.controladorA);
    prefs.putBool ("configurada", cfgPrevisao.configurada);
    prefs.end();
}

void carregarConfigPrevisao() {
    prefs.begin("rrfs_prev", true);
    cfgPrevisao.configurada  = prefs.getBool ("configurada", false);
    cfgPrevisao.ativa        = prefs.getBool ("ativa",       false);
    cfgPrevisao.latitude     = prefs.getFloat("lat",         0.0f);
    cfgPrevisao.longitude    = prefs.getFloat("lon",         0.0f);
    cfgPrevisao.painelKW     = prefs.getFloat("painel_kw",   0.0f);
    cfgPrevisao.fatorCalib   = prefs.getFloat("fator_calib", PREV_FATOR_INICIAL);
    cfgPrevisao.controladorA = prefs.getFloat("ctrl_a",      0.0f);
    prefs.end();
    if (cfgPrevisao.fatorCalib < 0.3f || cfgPrevisao.fatorCalib > 2.0f)
        cfgPrevisao.fatorCalib = PREV_FATOR_INICIAL;
}

int getIndiceHoraAtual() {
    if (!previsao.valido) return -1;
    struct tm t; if (!obterTempoAtual(t)) return -1;
    auto toDias = [](int a, int m, int d) -> long {
        if (m < 3) { m += 12; a--; }
        return (long)365*a + a/4 - a/100 + a/400 + (153*m-457)/5 + d;
    };
    long diasBase  = toDias(previsao.anoBase, previsao.mesBase, previsao.diaBase);
    long diasAtual = toDias(t.tm_year+1900, t.tm_mon+1, t.tm_mday);
    long diff = (diasAtual - diasBase)*24L + (t.tm_hour - previsao.horaBase);
    if (diff < 0 || diff >= PREV_TOTAL_HORAS) return -1;
    return (int)diff;
}

float getAhPrevHoraAtual() {
    int idx = getIndiceHoraAtual();
    if (idx < 0 || idx >= PREV_TOTAL_HORAS) return 0.0f;
    return previsao.ahPrevisto[idx];
}

float getAhPrevRestanteDia() {
    int idx = getIndiceHoraAtual();
    if (idx < 0) return 0.0f;
    struct tm t;
    if (!obterTempoAtual(t)) return 0.0f;
    int hFalt = 24 - t.tm_hour;
    float total = 0.0f;
    for (int i = 0; i < hFalt && (idx + i) < PREV_TOTAL_HORAS; i++) {
        float w = (i == 0) ? (60 - t.tm_min) / 60.0f : 1.0f;
        total += previsao.ahPrevisto[idx + i] * w;
    }
    return total;
}

// ── Estatística por modelo (persistida) ─────────────────────────────────────────
void carregarEstatPrev() {
    if (estPrev.carregado) return;
    estPrev.carregado = true;
    prefs.begin("rrfs_prev", true);
    estPrev.err  = prefs.getFloat ("st_err",  0.0f);
    estPrev.prev = prefs.getFloat ("st_prev", 0.0f);
    estPrev.real = prefs.getFloat ("st_real", 0.0f);
    estPrev.n    = prefs.getUShort("st_n",    0);
    size_t len = prefs.getBytes("mstat", &estMod, sizeof(estMod));
    prefs.end();
    if (len != sizeof(estMod)) {
        memset(&estMod, 0, sizeof(estMod));
        for (int m = 0; m < PREV_MODELOS; m++) estMod.peso[m] = 1.0f;
    }
}

static void salvarEstatPrev() {
    prefs.begin("rrfs_prev", false);
    prefs.putFloat ("st_err",  estPrev.err);
    prefs.putFloat ("st_prev", estPrev.prev);
    prefs.putFloat ("st_real", estPrev.real);
    prefs.putUShort("st_n",    estPrev.n);
    prefs.putBytes ("mstat",   &estMod, sizeof(estMod));
    prefs.end();
}

// Peso de cada modelo = 1/erro_relativo^2 (normalizado). Modelos sem historico = media.
static void prevAtualizarPesos() {
    float w[PREV_MODELOS], soma = 0; int conhecidos = 0;
    for (int m = 0; m < PREV_MODELOS; m++) {
        w[m] = -1.0f;
        if (estMod.n[m] >= 5 && estMod.prev[m] > 0.01f) {
            float r = estMod.err[m] / estMod.prev[m];
            if (r < 0.05f) r = 0.05f;
            w[m] = 1.0f / (r * r);
            soma += w[m]; conhecidos++;
        }
    }
    float media = conhecidos ? soma / conhecidos : 1.0f;
    float mx = 0;
    for (int m = 0; m < PREV_MODELOS; m++) { if (w[m] < 0) w[m] = media; if (w[m] > mx) mx = w[m]; }
    for (int m = 0; m < PREV_MODELOS; m++) {
        float p = (mx > 0) ? w[m] / mx : 1.0f;
        estMod.peso[m] = (p < 0.03f) ? 0.03f : p;
    }
}

// Radiacao efetiva: corta luz muito fraca e aplica perda em baixa irradiancia
static float prevRadEfetiva(float rad) {
    if (rad < PREV_RAD_CORTE) return 0.0f;
    float f = (rad >= PREV_RAD_PLENA) ? 1.0f : 0.70f + 0.30f * (rad / PREV_RAD_PLENA);
    return rad * f;
}

// Radiacao (W/m2) -> Ah na hora, ja com fator calibrado e teto do controlador
static float prevAhDeRad(float rad) {
    float vBanco = (dadosBMS.valido && dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;
    float tetoA  = (cfgPrevisao.controladorA > 0.0f) ? cfgPrevisao.controladorA * PREV_CTRL_EFIC : 0.0f;
    float a = prevRadEfetiva(rad) * cfgPrevisao.painelKW / vBanco * cfgPrevisao.fatorCalib;
    if (tetoA > 0.0f && a > tetoA) a = tetoA;
    return a;
}

void calcularAhPrevisto() {
    carregarEstatPrev();
    for (int d = 0; d < PREV_DIAS; d++) previsao.ahDia[d] = 0.0f;

    for (int h = 0; h < PREV_TOTAL_HORAS; h++) {
        // Open-Meteo: indice i = MEDIA DA HORA ANTERIOR. Energia de h:00..h+1:00 esta em h+1.
        int   src = h + 1;
        float rad = 0.0f, sw = 0.0f;
        if (src < PREV_TOTAL_HORAS) {
            for (int m = 0; m < PREV_MODELOS; m++) {
                if (!previsao.modeloOk[m]) continue;
                uint16_t r = previsao.radModelo[m][src];
                if (r == 0xFFFF) continue;
                float w = previsao.modoUnico ? 1.0f : estMod.peso[m];
                rad += w * (float)r;
                sw  += w;
            }
        }
        rad = (sw > 0.0f) ? rad / sw : 0.0f;

        float corrA = prevAhDeRad(rad);
        previsao.ahPrevisto[h] = corrA;
        int dIdx = h / 24;
        if (dIdx < PREV_DIAS) previsao.ahDia[dIdx] += corrA;
    }
}

void calibrarPrevisao() {
    carregarEstatPrev();
    if (!previsao.valido || !cfgPrevisao.configurada) return;

    struct tm t;
    if (!obterTempoAtual(t)) return;

    // Painel limitado pela demanda: bateria quase cheia OU excesso solar em curso
    if (bmsFresca() && (dadosBMS.soc >= PREV_SOC_LIMITE || excesso.fase != 0))
        estPrev.limitado = true;

    if (estPrev.horaAv == 255) {
        estPrev.horaAv   = t.tm_hour;
        estPrev.pular    = true;
        estPrev.limitado = false;
        return;
    }
    if (t.tm_hour == estPrev.horaAv) return;
    if (acum.horaAtual != t.tm_hour) return;

    int  horaFim = estPrev.horaAv;
    bool lim     = estPrev.limitado;
    bool pular   = estPrev.pular;
    estPrev.horaAv   = t.tm_hour;
    estPrev.limitado = false;
    estPrev.pular    = false;

    if (pular) return;
    if (t.tm_hour != (horaFim + 1) % 24) return;
    if (t.tm_hour == 0) return;
    if (!bmsFresca()) return;

    int idx = getIndiceHoraAtual();
    if (idx < 1) return;

    float ahReal    = histDiaAtual.horas[horaFim].painelAh;
    float ahPrevCal = previsao.ahPrevisto[idx - 1];
    if (ahPrevCal < 0.3f) return;

    float dif = ahPrevCal - ahReal;                  // >0 = previu ACIMA
    // Hora censurada: painel limitado pela demanda e entregou MENOS que o previsto.
    // Isso e natural (bateria cheia / pouca carga) e nao e erro da previsao.
    bool censurada = lim && (dif > 0.0f);

    // ── Estatistica geral (assimetrica) ───────────────────────────────────────
    if (!censurada) {
        if (estPrev.dia != (uint8_t)t.tm_mday) {
            estPrev.dia = t.tm_mday;
            estPrev.hErr = estPrev.hPrev = estPrev.hReal = 0;
            estPrev.hN = 0;
        }
        float erro = (dif > 0.0f) ? dif * PREV_PESO_SUPERESTIMA : -dif * PREV_PESO_SUBESTIMA;
        const float k = 0.97f;
        estPrev.err  = estPrev.err  * k + erro;
        estPrev.prev = estPrev.prev * k + ahPrevCal;
        estPrev.real = estPrev.real * k + ahReal;
        if (estPrev.n < 65000) estPrev.n++;
        estPrev.hErr  += erro;
        estPrev.hPrev += ahPrevCal;
        estPrev.hReal += ahReal;
        estPrev.hN++;
    }

    // ── Pesos por modelo (so no modo multi-modelo) ────────────────────────────
    if (!previsao.modoUnico) {
        const float k = 0.97f;
        for (int m = 0; m < PREV_MODELOS; m++) {
            if (!previsao.modeloOk[m]) continue;
            uint16_t r = previsao.radModelo[m][idx];       // mesma hora: src = (idx-1)+1
            if (r == 0xFFFF) continue;
            float pm = prevAhDeRad((float)r);
            if (pm < 0.3f && ahReal < 0.3f) continue;
            float d = pm - ahReal;
            if (lim && d > 0.0f) continue;                 // censurado
            float e = (d > 0.0f) ? d * PREV_PESO_SUPERESTIMA : -d * PREV_PESO_SUBESTIMA;
            estMod.err[m]  = estMod.err[m]  * k + e;
            estMod.prev[m] = estMod.prev[m] * k + pm;
            if (estMod.n[m] < 65000) estMod.n[m]++;
        }
        prevAtualizarPesos();
    }
    salvarEstatPrev();

    // ── Calibracao do fator (conservadora; censurada nao ensina) ──────────────
    if (censurada) { calcularAhPrevisto(); return; }
    if (ahReal < 0.1f)    { calcularAhPrevisto(); return; }
    if (ahPrevCal < 1.0f) { calcularAhPrevisto(); return; }
    float tetoA = (cfgPrevisao.controladorA > 0.0f) ? cfgPrevisao.controladorA * PREV_CTRL_EFIC : 0.0f;
    if (tetoA > 0.0f && ahPrevCal >= tetoA * 0.98f) { calcularAhPrevisto(); return; }

    float ahPrevBruto = ahPrevCal / cfgPrevisao.fatorCalib;
    float nf = ahReal / ahPrevBruto;
    if (nf < 0.2f) nf = 0.2f;
    if (nf > 2.0f) nf = 2.0f;

    float alfa = (nf < cfgPrevisao.fatorCalib) ? PREV_ALFA_BAIXA : PREV_ALFA_ALTA;
    cfgPrevisao.fatorCalib += alfa * (nf - cfgPrevisao.fatorCalib);
    if (cfgPrevisao.fatorCalib < 0.3f) cfgPrevisao.fatorCalib = 0.3f;
    if (cfgPrevisao.fatorCalib > 2.0f) cfgPrevisao.fatorCalib = 2.0f;

    calcularAhPrevisto();
    prefs.begin("rrfs_prev", false);
    prefs.putFloat("fator_calib", cfgPrevisao.fatorCalib);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf), "Prev h%02d: real %.1f prev %.1f fator %.2f",
             horaFim, ahReal, ahPrevCal, cfgPrevisao.fatorCalib);
    adicionarLog(tempBuf);
}

// Le um array JSON de numeros. null -> -1. Retorna quantidade lida.
static int prevParseArr(const String& body, const char* chave, float* out) {
    int pos = body.indexOf(chave);
    if (pos < 0) return 0;
    pos += strlen(chave);
    const char* s = body.c_str() + pos;
    int count = 0;
    while (count < PREV_TOTAL_HORAS && *s) {
        while (*s == ' ') s++;
        if (*s == ']') break;
        float val;
        if (strncmp(s, "null", 4) == 0) { val = -1.0f; s += 4; }
        else {
            char* fim;
            val = strtof(s, &fim);
            if (fim == s) break;
            s = fim;
            if (val < 0.0f) val = 0.0f;
        }
        out[count++] = val;
        while (*s && *s != ',' && *s != ']') s++;
        if (*s == ',') s++;
    }
    return count;
}

#if PREV_USAR_INCLINADO
  #define PREV_VAR_NOME "global_tilted_irradiance"
#else
  #define PREV_VAR_NOME "shortwave_radiation"
#endif

static int prevHttp(bool multi, String& body) {
    char url[640];
    char extra[56] = "";
    #if PREV_USAR_INCLINADO
    float incl = fabsf(cfgPrevisao.latitude);
    if (incl < 10.0f) incl = 10.0f;
    if (incl > 40.0f) incl = 40.0f;
    int azim = (cfgPrevisao.latitude >= 0.0f) ? 0 : 180;
    snprintf(extra, sizeof(extra), "&tilt=%.0f&azimuth=%d", incl, azim);
    #endif
    int n = snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&hourly=" PREV_VAR_NOME "%s&forecast_days=7&timezone=auto",
             cfgPrevisao.latitude, cfgPrevisao.longitude, extra);
    if (multi) {
        n += snprintf(url + n, sizeof(url) - n, "&models=");
        for (int m = 0; m < PREV_MODELOS && n < (int)sizeof(url) - 40; m++)
            n += snprintf(url + n, sizeof(url) - n, "%s%s", m ? "," : "", PREV_MODELO_ID[m]);
    }

    HTTPClient http;
    http.begin(url);
    http.setTimeout(multi ? 12000 : 6000);
    http.setConnectTimeout(3000);

    esp_task_wdt_reset();
    server.handleClient();
    int code = http.GET();
    esp_task_wdt_reset();
    if (code == 200) body = http.getString();   // getString trata resposta chunked
    http.end();
    esp_task_wdt_reset();
    server.handleClient();
    return code;
}

bool buscarPrevisaoOpenMeteo() {
    if (WiFi.status() != WL_CONNECTED || !cfgPrevisao.configurada) return false;
    if (WiFi.RSSI() < -85) { adicionarLog("Prev: sinal fraco"); return false; }

    carregarEstatPrev();

    bool multi = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) >= PREV_HEAP_MIN_MULTI;
    String body;
    int code = prevHttp(multi, body);
    if (code == 400 && multi) {                    // algum modelo recusado: volta ao modo unico
        adicionarLog("Prev: modelos recusados, modo unico");
        multi = false;
        body = "";
        code = prevHttp(false, body);
    }
    if (code != 200) {
        snprintf(tempBuf, sizeof(tempBuf), "Prev: HTTP %d", code);
        adicionarLog(tempBuf);
        return false;
    }
    if (body.length() < 100) { adicionarLog("Prev: resposta vazia"); return false; }

    // ── Data/hora base vinda da propria API ───────────────────────────────────
    int anoB = 0, mesB = 0, diaB = 0, horaB = 0;
    int pT = body.indexOf("\"time\":[\"");
    if (pT >= 0) {
        pT += 9;
        anoB  = body.substring(pT,      pT + 4).toInt();
        mesB  = body.substring(pT + 5,  pT + 7).toInt();
        diaB  = body.substring(pT + 8,  pT + 10).toInt();
        horaB = body.substring(pT + 11, pT + 13).toInt();
    }
    if (anoB < 2020 || mesB < 1 || mesB > 12 || diaB < 1 || diaB > 31) {
        struct tm tn;
        if (!obterTempoAtual(tn)) return false;
        anoB = tn.tm_year + 1900; mesB = tn.tm_mon + 1; diaB = tn.tm_mday; horaB = 0;
    }

    int pU = body.indexOf("\"utc_offset_seconds\":");
    if (pU >= 0) {
        long offApi = body.substring(pU + 21).toInt();
        long offDev = (long)cfgTempo.fusoHoras * 3600L + (long)cfgTempo.fusoMinutos * 60L;
        if (offApi != offDev) {
            snprintf(tempBuf, sizeof(tempBuf), "Prev: fuso API %ldh != relogio %ldh",
                     offApi / 3600L, offDev / 3600L);
            adicionarLog(tempBuf);
        }
    }

    // ── Parser por modelo ─────────────────────────────────────────────────────
    static float tmp[PREV_TOTAL_HORAS];
    char chave[96];
    int  okCount = 0;
    for (int m = 0; m < PREV_MODELOS; m++) { previsao.modeloOk[m] = false; }

    if (multi) {
        for (int m = 0; m < PREV_MODELOS; m++) {
            snprintf(chave, sizeof(chave), "\"" PREV_VAR_NOME "_%s\":[", PREV_MODELO_ID[m]);
            int count = prevParseArr(body, chave, tmp);
            int validos = 0;
            for (int i = 0; i < count; i++) if (tmp[i] >= 0.0f) validos++;
            if (count < 24 || validos < 24) continue;       // modelo sem dados
            for (int i = 0; i < PREV_TOTAL_HORAS; i++)
                previsao.radModelo[m][i] = (i < count && tmp[i] >= 0.0f)
                                           ? (uint16_t)(tmp[i] + 0.5f) : 0xFFFF;
            previsao.modeloOk[m] = true;
            okCount++;
        }
    } else {
        snprintf(chave, sizeof(chave), "\"" PREV_VAR_NOME "\":[");
        int count = prevParseArr(body, chave, tmp);
        if (count >= 24) {
            for (int i = 0; i < PREV_TOTAL_HORAS; i++)
                previsao.radModelo[0][i] = (i < count && tmp[i] >= 0.0f)
                                           ? (uint16_t)(tmp[i] + 0.5f) : 0xFFFF;
            previsao.modeloOk[0] = true;
            okCount = 1;
        }
    }
    body = "";
    if (okCount == 0) { adicionarLog("Prev: sem dados de radiacao"); return false; }

    previsao.modoUnico    = !multi;
    previsao.horaBase     = horaB;
    previsao.diaBase      = diaB;
    previsao.mesBase      = mesB;
    previsao.anoBase      = anoB;
    previsao.tAtualizacao = millis();
    previsao.valido       = true;
    prevAtualizarPesos();
    calcularAhPrevisto();
    snprintf(tempBuf, sizeof(tempBuf), "Prev OK %d modelo(s)%s", okCount, multi ? "" : " (unico)");
    adicionarLog(tempBuf);
    return true;
}

void gerenciarPrevisao() {
    if (!cfgPrevisao.ativa || !cfgPrevisao.configurada) return;
    if (WiFi.status() != WL_CONNECTED || modoAP) return;
    if (millis() - tUltimaReconWifi2 < 30000UL) return;

    unsigned long agora = millis();
    if (!previsao.valido || agora - tUltimaPrevisao >= INTERVALO_PREVISAO_MS) {
        tUltimaPrevisao = agora;
        esp_task_wdt_reset();
        server.handleClient();
        buscarPrevisaoOpenMeteo();
        esp_task_wdt_reset();
        server.handleClient();
    }
    calibrarPrevisao();
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 15 — HISTÓRICO DE ENERGIA (30 DIAS)
// ═══════════════════════════════════════════════════════════════════════════════════

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
    DiaCompacto c;
    memset(&c, 0, sizeof(c));
    c.ano = dia.ano; c.mes = dia.mes; c.dia = dia.dia; c.valido = dia.valido ? 1 : 0;
    for (int h = 0; h < HIST_HORAS; h++) {
        c.h[h][0] = ahParaU16(dia.horas[h].painelAh);
        c.h[h][1] = ahParaU16(dia.horas[h].casaAh);
        c.h[h][2] = ahParaU16(dia.horas[h].batCargAh);
        c.h[h][3] = ahParaU16(dia.horas[h].batDescAh);
    }
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);
    prefs.begin(ns, false);
    size_t w = prefs.putBytes("d", &c, sizeof(c));
    prefs.end();
    static bool avisou = false;
    if (w != sizeof(c) && !avisou) {
        avisou = true;
        adicionarLog("NVS CHEIO: falha gravar historico");
    }
}

bool carregarDiaFlash(uint16_t idx, RegistroDia& dia) {
    char ns[12];
    snprintf(ns, sizeof(ns), "rh%03d", idx % HIST_DIAS);
    DiaCompacto c;
    prefs.begin(ns, true);
    size_t len = prefs.getBytes("d", &c, sizeof(c));
    prefs.end();
    if (len != sizeof(c) || !c.valido) return false;
    memset(&dia, 0, sizeof(RegistroDia));
    dia.ano = c.ano; dia.mes = c.mes; dia.dia = c.dia; dia.valido = true;
    for (int h = 0; h < HIST_HORAS; h++) {
        dia.horas[h].painelAh  = c.h[h][0] / 100.0f;
        dia.horas[h].casaAh    = c.h[h][1] / 100.0f;
        dia.horas[h].batCargAh = c.h[h][2] / 100.0f;
        dia.horas[h].batDescAh = c.h[h][3] / 100.0f;
    }
    calcularTotaisDia(dia);
    return true;
}

void salvarIndiceHistorico() {
    prefs.begin("rrfs_hidx", false);
    prefs.putUChar ("ver",   HIST_KEY_VER);
    prefs.putUShort("idx",   histIndiceAtual);
    prefs.putUShort("total", histTotalDias);
    prefs.end();
}

void carregarHistorico() {
    prefs.begin("rrfs_hidx", true);
    uint8_t ver = prefs.getUChar("ver", 0);
    if (ver != HIST_KEY_VER) {
        prefs.end();
        for (uint16_t i = 0; i < HIST_DIAS; i++) {
            char ns[12];
            snprintf(ns, sizeof(ns), "rh%03d", i);
            prefs.begin(ns, false);
            prefs.clear();
            prefs.end();
        }
        histIndiceAtual = 0;
        histTotalDias = 0;
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
        salvarIndiceHistorico();
        adicionarLog("Hist reset v32 (compacto)");
        return;
    }
    histIndiceAtual = prefs.getUShort("idx",   0);
    histTotalDias   = prefs.getUShort("total", 0);
    prefs.end();
    if (histTotalDias > 0 && !carregarDiaFlash(histIndiceAtual, histDiaAtual))
        memset(&histDiaAtual, 0, sizeof(RegistroDia));
}

void atualizarDiaAtual() {
    unsigned long agora = millis();
    if (agora - ultimaAtualizacaoDia < 60000UL && diaCache != 0) return;
    ultimaAtualizacaoDia = agora;
    struct tm t;
    if (obterTempoAtual(t)) {
        diaCache = t.tm_mday;
        mesCache = t.tm_mon+1;
        anoCache = t.tm_year+1900;
    } else if (!diaCache) {
        diaCache = 1; mesCache = 1; anoCache = 2024;
    }
}

void acumularEnergia() {
    // Relógio não confiável (NTP ligado e ainda não sincronizou): não abre dia falso
    if (cfgTempo.ntpAtivo && !ntpSincronizado) return;
    atualizarDiaAtual();
    unsigned long agora = millis();
    struct tm t;
    uint8_t  hA=0, dA=diaCache, mA=mesCache;
    uint16_t aA=anoCache;
    if (obterTempoAtual(t)) {
        hA=t.tm_hour; dA=t.tm_mday; mA=t.tm_mon+1; aA=t.tm_year+1900;
    } else {
        hA=(agora/3600000UL)%24;
    }

    if (acum.ultimoAcum == 0) {
        acum.ultimoAcum=agora;
        acum.horaAtual=hA; acum.diaAtual=dA; acum.mesAtual=mA; acum.anoAtual=aA;
        bool diaEx=false;
        if (histTotalDias>0) {
            RegistroDia dT;
            if (carregarDiaFlash(histIndiceAtual,dT) &&
                dT.dia==dA && dT.mes==mA && dT.ano==aA)
                { histDiaAtual=dT; diaEx=true; }
        }
        if (!diaEx) {
            memset(&histDiaAtual,0,sizeof(RegistroDia));
            histDiaAtual.ano=aA; histDiaAtual.mes=mA; histDiaAtual.dia=dA; histDiaAtual.valido=true;
            if (histTotalDias==0) {
                histIndiceAtual=0;
                histTotalDias=1;
            } else {
                RegistroDia dT;
                if (carregarDiaFlash(histIndiceAtual,dT) &&
                    (dT.dia!=dA||dT.mes!=mA||dT.ano!=aA)) {
                    histIndiceAtual=(histIndiceAtual+1)%HIST_DIAS;
                    if (histTotalDias<HIST_DIAS) histTotalDias++;
                }
            }
            salvarDiaFlash(histIndiceAtual,histDiaAtual);
            salvarIndiceHistorico();
        }
        padroesSincronizarDia(aA, mA, dA);      // v32: slot de padroes acompanha o dia
        return;
    }

    float dH=(float)(agora-acum.ultimoAcum)/3600000.0f;
    acum.ultimoAcum=agora;
    if (dH>0.1f) dH=0.1f;

    float ip=sensores.correntePainel;
    float ic=sensores.correnteCasa;
    float ib=sensores.correnteBateria;

    if (ip>0.15f)  acum.painelAh  += ip*dH;
    if (ic>0.3f)   acum.casaAh    += ic*dH;
    if (ib>0.3f)   acum.batCargAh += ib*dH;
    if (ib<-0.3f)  acum.batDescAh += fabsf(ib)*dH;

    bool mudouHora=(hA!=acum.horaAtual);
    bool mudouDia =(dA!=acum.diaAtual||mA!=acum.mesAtual||aA!=acum.anoAtual);

    if (mudouHora||mudouDia) {
        uint8_t hi=acum.horaAtual%HIST_HORAS;
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
            salvarDiaFlash(histIndiceAtual,histDiaAtual);
            padroesSalvarSeSujo(true);          // v32: fecha os padroes do dia que acabou
            histIndiceAtual=(histIndiceAtual+1)%HIST_DIAS;
            if (histTotalDias<HIST_DIAS) histTotalDias++;
            memset(&histDiaAtual,0,sizeof(RegistroDia));
            histDiaAtual.ano=aA; histDiaAtual.mes=mA; histDiaAtual.dia=dA; histDiaAtual.valido=true;
            padroesSincronizarDia(aA, mA, dA);  // v32: slot novo, vazio
        }
        salvarDiaFlash(histIndiceAtual,histDiaAtual);
        salvarIndiceHistorico();
        salvarContadores();
        acum.painelAh=acum.casaAh=acum.batCargAh=acum.batDescAh=0;
        acum.horaAtual=hA;
        if (mudouDia) { acum.diaAtual=dA; acum.mesAtual=mA; acum.anoAtual=aA; }
    }

    static unsigned long tParcial=0;
    if (agora-tParcial>=300000UL) {
        tParcial=agora;
        salvarDiaFlash(histIndiceAtual,histDiaAtual);
        salvarContadores();
        padroesSalvarSeSujo();                  // v32
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 16 — PROTEÇÃO POR SOC MÍNIMO + AUTONOMIA
// ═══════════════════════════════════════════════════════════════════════════════════

bool bmsFresca() {
    return dadosBMS.valido && (millis() - dadosBMS.tUltimaDado < 30000UL);
}

void salvarConfigProtecao() {
    prefs.begin("rrfs_prot", false);
    prefs.putFloat("reserva", protecao.reservaPct);
    prefs.end();
}

void carregarConfigProtecao() {
    prefs.begin("rrfs_prot", true);
    protecao.reservaPct = prefs.getFloat("reserva", 15.0f);
    bool travada        = prefs.getBool ("ativa", false);
    prefs.end();
    if (protecao.reservaPct < 0.0f || protecao.reservaPct > 50.0f)
        protecao.reservaPct = 15.0f;
    protecao.ativa = travada;        // sobrevive ao reinicio: so libera com SOC OK do BMS
    if (travada) adicionarLog("Prot: ATIVA restaurada do boot");
}

static void protSalvarEstado(bool ativa) {
    prefs.begin("rrfs_prot", false);
    prefs.putBool("ativa", ativa);
    prefs.end();
}

void atualizarProtecao() {
    static unsigned long tSemBMS = 0;
    const unsigned long LIBERA_SEM_BMS_MS = 1800000UL;   // 30 min sem BMS libera (0 = nunca libera)

    if (!cfgBMS.configurada) {
        if (protecao.ativa) { protecao.ativa = false; protSalvarEstado(false); }
        tSemBMS = 0;
        return;
    }

    unsigned long agora = millis();

    if (!bmsFresca()) {
        // MANTEM a protecao travada enquanto o BMS estiver sem dados
        if (protecao.ativa) {
            if (tSemBMS == 0) tSemBMS = agora ? agora : 1;
            if (LIBERA_SEM_BMS_MS > 0 && agora - tSemBMS >= LIBERA_SEM_BMS_MS) {
                protecao.ativa = false;
                protSalvarEstado(false);
                tSemBMS = 0;
                adicionarLog("Prot: BMS 30min sem dados, liberado");
            }
        } else {
            tSemBMS = 0;
        }
        return;
    }
    tSemBMS = 0;

    float soc = dadosBMS.soc;
    if (!protecao.ativa) {
        if (soc <= protecao.reservaPct) {
            protecao.ativa = true;
            protSalvarEstado(true);
            snprintf(tempBuf, sizeof(tempBuf), "PROTECAO ON: SOC %.0f%% <= %.0f%%",
                     soc, protecao.reservaPct);
            adicionarLog(tempBuf);
        }
    } else {
        if (soc >= protecao.reservaPct + PROT_HISTERESE_PCT) {
            protecao.ativa = false;
            protSalvarEstado(false);
            snprintf(tempBuf, sizeof(tempBuf), "PROTECAO OFF: SOC %.0f%%", soc);
            adicionarLog(tempBuf);
        }
    }
}

EstimativaTempo calcularTempoRestante() {
    EstimativaTempo est = {false, 0, 0, false, 0.0f, 0.0f, 0.0f, false};

    if (!bmsFresca()) return est;

    float cap = dadosBanco.capacidadeTotal;
    if (cap < 1.0f) return est;

    float soc = dadosBMS.soc;
    est.ahDisp = cap * (soc - protecao.reservaPct) / 100.0f;
    if (est.ahDisp < 0.0f) est.ahDisp = 0.0f;
    est.ahFalta = cap * (100.0f - soc) / 100.0f;

    float i = sensores.correnteBateria;
    est.taxaA      = i;
    est.carregando = (i > 0.0f);

    if (fabsf(i) < 0.3f) return est;

    float ah;
    if (est.carregando) {
        ah = est.ahFalta;
    } else {
        if (est.ahDisp <= 0.0f) { est.naReserva = true; return est; }
        ah = est.ahDisp;
    }
    if (ah <= 0.0f) return est;

    float h = ah / fabsf(i);
    if (h > 999.0f) h = 999.0f;
    est.horas   = (int)h;
    est.minutos = (int)((h - est.horas) * 60.0f);
    est.valido  = true;
    return est;
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 16B — SAÚDE DA BATERIA (capacidade por janela de SOC + resistência interna)
// ═══════════════════════════════════════════════════════════════════════════════════
// CAPACIDADE: integra a corrente do banco entre duas TRANSICOES de SOC (x% -> x+1%),
//   o que elimina o erro de quantizacao do SOC inteiro. Janela >= 20%, SOC 10..96%.
//   Cap = Ah liquidos / (deltaSOC/100). OBS: o SOC do BMS e um contador de Ah, entao
//   isto mede principalmente a coerencia BMS/shunt, nao o desgaste das celulas.
// RESISTENCIA: degrau de corrente >= 8 A entre dois quadros consecutivos do BMS
//   (V e I do mesmo instante). R = dV/dI (por bateria, em mOhm). Mediana das ultimas 8.
//   Referencia = mediana das 8 primeiras medidas. E o indicador principal.

static const char* const SAUDE_TXT_GERAL[4] = {"Excelente", "Boa", "Atencao", "Degradada"};
static const char* const SAUDE_TXT_CAP[4]   = {"Excelente", "Boa", "Regular", "Degradada"};
static const char* const SAUDE_TXT_RI[4]    = {"Normal", "Levemente elevada", "Elevada", "Alta"};

static long saudeDias(int a, int m, int d) {
    if (m < 3) { m += 12; a--; }
    return (long)365 * a + a / 4 - a / 100 + a / 400 + (153 * m - 457) / 5 + d;
}

static float saudeMediana(const float* v, int n) {
    float t[12];
    if (n <= 0) return 0.0f;
    if (n > 12) n = 12;
    for (int i = 0; i < n; i++) t[i] = v[i];
    for (int i = 1; i < n; i++) {
        float k = t[i]; int j = i - 1;
        while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
        t[j + 1] = k;
    }
    return (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

static float saudeNominal() {
    if (saude.nomManual > 1.0f) return saude.nomManual;
    return dadosBanco.capacidadeTotal;
}

static int saudeLvCap(float pct) { return pct >= 95.0f ? 0 : pct >= 85.0f ? 1 : pct >= 80.0f ? 2 : 3; }
static int saudeLvRi(float ratio) { return ratio <= 1.15f ? 0 : ratio <= 1.30f ? 1 : ratio <= 1.60f ? 2 : 3; }

// SOH atual (mediana das ultimas 5 medidas). false se ainda nao ha 2 medidas.
static bool saudeCapPct(float& pct, float& capMed) {
    if (saude.nCap < 2) return false;
    int k = (saude.nCap < 5) ? saude.nCap : 5;
    float s[5], c[5];
    for (int i = 0; i < k; i++) {
        const CapMed& m = saude.cap[saude.nCap - 1 - i];
        s[i] = (m.nom > 1.0f) ? 100.0f * m.cap / m.nom : 0.0f;
        c[i] = m.cap;
    }
    pct    = saudeMediana(s, k);
    capMed = saudeMediana(c, k);
    return true;
}

// Tendencia em %/mes (minimos quadrados). So com >= 4 medidas e >= 14 dias de intervalo.
static bool saudeTendencia(float& pctMes) {
    int n = saude.nCap;
    if (n < 4) return false;
    long d0 = saudeDias(saude.cap[0].ano, saude.cap[0].mes, saude.cap[0].dia);
    long d1 = saudeDias(saude.cap[n - 1].ano, saude.cap[n - 1].mes, saude.cap[n - 1].dia);
    if (d1 - d0 < 14) return false;
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) {
        const CapMed& m = saude.cap[i];
        if (m.nom < 1.0f) return false;
        float x = (float)(saudeDias(m.ano, m.mes, m.dia) - d0);
        float y = 100.0f * m.cap / m.nom;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float den = n * sxx - sx * sx;
    if (fabsf(den) < 1e-3f) return false;
    pctMes = ((n * sxy - sx * sy) / den) * 30.0f;
    return true;
}

static bool saudeRiAtual(float& mohm) {
    if (saude.riN == 0) return false;
    mohm = saudeMediana(saude.riRing, saude.riN);
    return true;
}

void salvarSaudeSeSujo(bool forcar) {
    if (!saudeR.sujo) return;
    unsigned long agora = millis();
    if (!forcar && saudeR.tSalvo != 0 && agora - saudeR.tSalvo < 600000UL) return;
    saudeR.tSalvo = agora;
    prefs.begin(SAUDE_NS, false);
    prefs.putBytes("s", &saude, sizeof(saude));
    prefs.end();
    saudeR.sujo = false;
}

void carregarSaudeBateria() {
    memset(&saudeR, 0, sizeof(saudeR));
    saudeR.socAnt = -1;
    memset(&saude, 0, sizeof(saude));
    SaudeSalva tmp;
    prefs.begin(SAUDE_NS, true);
    size_t len = prefs.getBytes("s", &tmp, sizeof(tmp));
    prefs.end();
    if (len == sizeof(tmp) && tmp.ver == 1 && tmp.nCap <= SAUDE_N_CAP &&
        tmp.nDia <= SAUDE_N_RI_DIA && tmp.riN <= SAUDE_N_RI && tmp.riIdx < SAUDE_N_RI)
        saude = tmp;
    else
        saude.ver = 1;
    if (!(saude.nomManual >= 0.0f && saude.nomManual <= 2000.0f)) saude.nomManual = 0.0f;
    if (saude.nCap || saude.riTotal) {
        snprintf(tempBuf, sizeof(tempBuf), "Saude: %d medida(s) cap, %u medida(s) R",
                 (int)saude.nCap, (unsigned)saude.riTotal);
        adicionarLog(tempBuf);
    }
}

static void saudeAddCap(float cap, float nom, int delta) {
    CapMed m; memset(&m, 0, sizeof(m));
    struct tm t;
    if (obterTempoAtual(t)) { m.ano = t.tm_year + 1900; m.mes = t.tm_mon + 1; m.dia = t.tm_mday; }
    m.delta = (int8_t)delta; m.cap = cap; m.nom = nom;
    if (saude.nCap >= SAUDE_N_CAP) {
        memmove(&saude.cap[0], &saude.cap[1], sizeof(CapMed) * (SAUDE_N_CAP - 1));
        saude.nCap = SAUDE_N_CAP - 1;
    }
    saude.cap[saude.nCap++] = m;
    saudeR.sujo = true;
    salvarSaudeSeSujo(true);
    snprintf(tempBuf, sizeof(tempBuf), "Saude: cap %.1fAh (%.0f%%) janela %d%%",
             cap, 100.0f * cap / nom, delta);
    adicionarLog(tempBuf);
}

static void saudeCapAtualizar(unsigned long agora) {
    SaudeRun& r = saudeR;
    unsigned long dtMs = (r.tUlt == 0) ? 0 : agora - r.tUlt;
    r.tUlt = agora;

    bool fresca = dadosBMS.valido && (agora - dadosBMS.tUltimaDado < SAUDE_FRAME_MAX_MS);
    if (!fresca || dtMs > SAUDE_DT_MAX_MS) { r.janAtiva = false; r.socAnt = -1; return; }

    int soc = (int)(dadosBMS.soc + 0.5f);
    if (r.socAnt < 0) { r.socAnt = soc; return; }

    if (r.janAtiva) r.ahNet += sensores.correnteBateria * (float)dtMs / 3600000.0f;

    if (soc == r.socAnt) return;
    int salto = abs(soc - r.socAnt);
    r.socAnt = soc;

    if (salto > SAUDE_SOC_SALTO_MAX || soc < SAUDE_SOC_MIN || soc > SAUDE_SOC_MAX) {
        r.janAtiva = false;                       // recalibracao do BMS ou fora da faixa
        return;
    }
    if (!r.janAtiva || agora - r.tIni > SAUDE_JAN_MAX_MS) {
        r.janAtiva = true; r.socIni = soc; r.ahNet = 0.0f; r.tIni = agora;
        return;
    }

    int delta = soc - r.socIni;
    if (abs(delta) < SAUDE_JAN_MIN_PCT) return;

    float nom = saudeNominal();
    bool sinalOk = ((delta > 0) == (r.ahNet > 0.0f));
    if (nom > 1.0f && sinalOk) {
        float cap = r.ahNet / ((float)delta / 100.0f);
        if (cap >= 0.5f * nom && cap <= 1.3f * nom) saudeAddCap(cap, nom, abs(delta));
        else adicionarLog("Saude: medida de capacidade descartada");
    }
    r.socIni = soc; r.ahNet = 0.0f; r.tIni = agora;   // encadeia a proxima janela
}

static void saudeRiAmostra(float R, float dI) {
    saude.riRing[saude.riIdx] = R;
    saude.riIdx = (saude.riIdx + 1) % SAUDE_N_RI;
    if (saude.riN < SAUDE_N_RI) saude.riN++;
    if (saude.riTotal < 65000) saude.riTotal++;

    float med = saudeMediana(saude.riRing, saude.riN);
    if (!saude.riBaseOk && saude.riN >= SAUDE_N_RI) {
        saude.riBase = med; saude.riBaseOk = true;
        snprintf(tempBuf, sizeof(tempBuf), "Saude: referencia R = %.1f mOhm", med);
        adicionarLog(tempBuf);
    } else if (saude.riTotal <= SAUDE_N_RI) {
        snprintf(tempBuf, sizeof(tempBuf), "Saude: R=%.1f mOhm (dI %.0fA)", R, fabsf(dI));
        adicionarLog(tempBuf);
    }

    struct tm t;
    if (obterTempoAtual(t)) {
        int a = t.tm_year + 1900, m = t.tm_mon + 1, d = t.tm_mday;
        RiDia* u = saude.nDia ? &saude.dia[saude.nDia - 1] : nullptr;
        if (u && u->ano == a && u->mes == m && u->dia == d) {
            u->mohm = med;
        } else {
            if (saude.nDia >= SAUDE_N_RI_DIA) {
                memmove(&saude.dia[0], &saude.dia[1], sizeof(RiDia) * (SAUDE_N_RI_DIA - 1));
                saude.nDia = SAUDE_N_RI_DIA - 1;
            }
            RiDia& n = saude.dia[saude.nDia++];
            n.ano = a; n.mes = m; n.dia = d; n.mohm = med;
        }
    }
    saudeR.sujo = true;
}

static void saudeRiAtualizar(unsigned long agora) {
    SaudeRun& r = saudeR;
    if (!dadosBMS.valido) { r.nFr = 0; r.estRi = 0; return; }
    unsigned long tf = dadosBMS.tUltimaDado;
    if (tf == r.tUltFrame) return;                 // sem quadro novo
    r.tUltFrame = tf;

    float I = dadosBMS.corrente, V = dadosBMS.tensao;
    if (r.nFr > 0 && tf - r.t1 > RI_FRAME_GAP_MS) { r.nFr = 0; r.estRi = 0; }

    if (r.estRi == 1) {                            // quadro de confirmacao pos-degrau
        r.estRi = 0;
        float dI = r.posI - r.preI;
        if (fabsf(I - r.posI) <= fmaxf(RI_ESTAVEL_A, 0.15f * fabsf(dI))) {
            float R   = (r.posV - r.preV) / dI * 1000.0f;
            float soc = dadosBMS.soc;
            if (R >= RI_MIN_MOHM && R <= RI_MAX_MOHM && soc >= 10.0f && soc <= 98.0f)
                saudeRiAmostra(R, dI);
        }
    } else if (r.nFr >= 2) {                       // procura degrau: antes estavel, agora salto
        if (fabsf(I - r.i1) >= RI_MIN_DI && fabsf(r.i1 - r.i2) <= RI_ESTAVEL_A) {
            r.preI = r.i1; r.preV = r.v1;
            r.posI = I;    r.posV = V;
            r.estRi = 1;
        }
    }
    r.i2 = r.i1; r.v2 = r.v1;
    r.i1 = I;    r.v1 = V;  r.t1 = tf;
    if (r.nFr < 2) r.nFr++;
}

void atualizarSaudeBateria() {
    if (otaAtivo || modoTeste) return;
    unsigned long agora = millis();
    saudeCapAtualizar(agora);
    saudeRiAtualizar(agora);
    salvarSaudeSeSujo(false);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 17 — MÁQUINA DE ESTADOS INVERSOR BAIXA
// ═══════════════════════════════════════════════════════════════════════════════════

void maquinaInvBaixa() {
    if (modoTeste || otaAtivo) return;
    unsigned long agora = millis();

    static bool veioDeLigadoB = false;

    if (!sistemaEstabilizado && agora - tInicioSistema >= 120000UL) {
        sistemaEstabilizado = true;
        adicionarLog("InvA: sistema estabilizado");
    }

    // PROTECAO: desliga na hora, em qualquer estado
    if (protecao.ativa) {
        if (estados.invBaixa) {
            desligarInvBaixa();
            adicionarLog("InvA: OFF protecao (reserva)");
        }
        if (maqBaixa.estado != INV_A_DESLIGADO) {
            maqBaixa.estado        = INV_A_DESLIGADO;
            maqBaixa.tInicioEstado = agora;
        }
        diagRetryLongo      = false;
        veioDeLigadoB       = false;
        maqBaixa.tentativas = 0;
        return;
    }

    // Janela de boot: liga direto
    if (!sistemaEstabilizado) {
        if (!estados.invBaixa) {
            ligarInvBaixa();
            maqBaixa.estado        = INV_A_LIGADO;
            maqBaixa.tInicioEstado = agora;
            adicionarLog("InvA: ON (boot)");
        }
        return;
    }

    switch (maqBaixa.estado) {

        case INV_A_DESLIGADO:
            diagRetryLongo = false;
            ligarInvBaixa();
            veioDeLigadoB          = false;
            maqBaixa.estado        = INV_A_DIAG_AGUARDA;
            maqBaixa.tInicioEstado = agora;
            maqBaixa.tentativas    = 0;
            adicionarLog("InvA: ligou, aguardando AC");
            break;

        case INV_A_LIGADO:
            if (!acBaixaPresente()) {
                veioDeLigadoB          = true;
                maqBaixa.estado        = INV_A_DIAG_AGUARDA;
                maqBaixa.tInicioEstado = agora;
                maqBaixa.tentativas    = 0;
                adicionarLog("InvA: AC perdido, aguardando 30s");
            }
            break;

        case INV_A_DIAG_AGUARDA: {
            unsigned long limite = veioDeLigadoB ? BAIXA_PERDEU_AC_MS : BAIXA_AGUARDA_AC_MS;
            if (acBaixaPresente()) {
                maqBaixa.estado        = INV_A_LIGADO;
                maqBaixa.tInicioEstado = agora;
                maqBaixa.tentativas    = 0;
                maqBaixa.totalLigadas++;
                veioDeLigadoB          = false;
                adicionarLog("InvA: AC OK, LIGADO");
            } else if (agora - maqBaixa.tInicioEstado >= limite) {
                desligarInvBaixa();
                maqBaixa.estado        = INV_A_DIAG_OFF;
                maqBaixa.tInicioEstado = agora;
                maqBaixa.tentativas++;
                snprintf(tempBuf, sizeof(tempBuf), "InvA: sem AC %lus, tent %d/%d",
                         limite / 1000UL, maqBaixa.tentativas, DIAG_MAX_TENTATIVAS);
                adicionarLog(tempBuf);
            }
            break;
        }

        case INV_A_DIAG_OFF: {
            unsigned long espera = diagRetryLongo ? BAIXA_RETRY_LONGO_MS : BAIXA_MIN_OFF_MS;
            if (agora - maqBaixa.tInicioEstado >= espera) {
                if (diagRetryLongo) {
                    diagRetryLongo      = false;
                    maqBaixa.tentativas = 0;
                } else if (maqBaixa.tentativas >= DIAG_MAX_TENTATIVAS) {
                    diagRetryLongo         = true;
                    maqBaixa.totalFalhas++;
                    maqBaixa.tInicioEstado = agora;
                    adicionarLog("InvA: falha, nova tentativa em 10min");
                    break;
                }
                ligarInvBaixa();
                veioDeLigadoB          = false;
                maqBaixa.estado        = INV_A_DIAG_AGUARDA;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvA: religando diagnostico");
            }
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 18 — PADRÕES DE CONSUMO (v32b: eventos = uso do Inv. Alta pedido pela casa)
// ═══════════════════════════════════════════════════════════════════════════════════
//
// MODELO
//  - Inv.BAIXA: cargas pequenas e constantes (luz, celular, notebook, TV, roteador).
//    Gera a BASE ESTAVEL da casa. Ela pode mudar de patamar, mas e estavel.
//  - Inv.ALTA: cargas pesadas e TEMPORARIAS (compressor, forno, maquina de lavar).
//    Os dois inversores estao no mesmo cabo: a corrente do Alta SOMA a da base.
//  - EVENTO (padrao) = corrente acima da base enquanto o Inv.Alta esta energizado
//    por PEDIDO DA CASA (GPIO39). Guarda: hora, duracao, A medio/pico acima da base.
//    Energia do evento: Ah = A_medio x duracao / 3600.
//  - Com o Inv.Alta DESLIGADO nao existem eventos: qualquer mudanca de corrente
//    e carga do Inv.Baixa e a base apenas se ajusta (suave, ou salto apos 30 s).
//  - Excesso solar (Inv.Alta ligado pelo sistema, sem pedido da casa): so observa,
//    nada e registrado nem aprendido.
//  - Reconhecimento por APROXIMACAO: corrente +-0,5 A (ou 20%) e duracao ~2x.
//  - Aprendizado 1x/hora: vezes por dia e horario tipico de cada padrao (dias
//    recentes pesam mais). O planejador converte em Ah esperados por hora.
//  Armazenamento: 30 dias em RAM (padHist) salvos em LittleFS (/pad30.bin).
//

// Bateria cheia e painel cobrindo a casa: o consumo vem da SOBRA dos paineis.
static bool sobraSolarAgora() {
    static unsigned long tUltOk = 0;
    unsigned long agora = millis();

    if (!bmsFresca() || protecao.ativa || modoTeste || otaAtivo) { tUltOk = 0; return false; }

    float soc = dadosBMS.soc;
    float ip  = sensores.correntePainel;
    float ib  = sensores.correnteBateria;

    bool cond = (excesso.fase == 2) ||
    (soc >= EXC_SOC_CONF_MANTEM_PCT &&
    ip  >= EXC_PAINEL_MIN_A &&
    ib  >  -EXC_BAT_DESC_ENTRADA_A &&
    ib  <=  EXC_BAT_CARGA_ZERO_A);

    if (cond) { tUltOk = agora; if (tUltOk == 0) tUltOk = 1; }
    return (tUltOk != 0) && (agora - tUltOk < 20000UL);
}

static inline DiaPadroes& padHoje() { return padHist[histIndiceAtual % HIST_DIAS]; }

static void padLimparSlot(DiaPadroes& p, uint16_t a, uint8_t m, uint8_t d) {
    memset(&p, 0, sizeof(p));
    p.ano = a; p.mes = m; p.dia = d;
    memset(p.base, 255, sizeof(p.base));
}

// ── Arquivo (LittleFS) ──────────────────────────────────────────────────────────
struct PadHdr { uint32_t magic; uint16_t ver; uint16_t tam; uint32_t crc; };

static uint32_t padCrc(const uint8_t* d, size_t n) {
    uint32_t c = 0xFFFFFFFFUL;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xEDB88320UL : (c >> 1);
    }
    return ~c;
}

void padFsIniciar() {
    padFsOk = LittleFS.begin(true);
    if (!padFsOk) {
        adicionarLog("Padroes: LittleFS FALHOU (so RAM). Use particao com SPIFFS");
        return;
    }
    snprintf(tempBuf, sizeof(tempBuf), "Padroes: FS %uK de %uK usados",
             (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024));
    adicionarLog(tempBuf);
}

static bool padLerArquivo(const char* nome) {
    if (!LittleFS.exists(nome)) return false;
    File f = LittleFS.open(nome, "r");
    if (!f) return false;
    PadHdr h; memset(&h, 0, sizeof(h));
    bool ok = (f.size() == sizeof(PadHdr) + sizeof(padHist)) &&
    f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) &&
    h.magic == PAD_MAGIC && h.tam == sizeof(DiaPadroes);
    if (ok) ok = (f.read((uint8_t*)padHist, sizeof(padHist)) == sizeof(padHist));
    f.close();
    if (ok && padCrc((const uint8_t*)padHist, sizeof(padHist)) != h.crc) ok = false;
    return ok;
}

void padCarregarArquivo() {
    for (int i = 0; i < HIST_DIAS; i++) padLimparSlot(padHist[i], 0, 0, 0);
    if (!padFsOk) return;
    bool ok = padLerArquivo(PAD_ARQ);
    if (!ok) ok = padLerArquivo(PAD_ARQ_TMP);
    if (!ok) {
        for (int i = 0; i < HIST_DIAS; i++) padLimparSlot(padHist[i], 0, 0, 0);
        adicionarLog("Padroes: sem arquivo valido, inicio limpo");
        return;
    }
    int n = 0;
    for (int i = 0; i < HIST_DIAS; i++) if (padHist[i].ano) n++;
    snprintf(tempBuf, sizeof(tempBuf), "Padroes: %d dia(s) carregados", n);
    adicionarLog(tempBuf);
}

void padroesSalvarSeSujo(bool forcar) {
    if (!padFsOk) return;
    if (!padSujo && !forcar) return;
    static bool avisou = false;

    File f = LittleFS.open(PAD_ARQ_TMP, "w");
    if (!f) {
        if (!avisou) { avisou = true; adicionarLog("Padroes: falha abrir arquivo"); }
        return;
    }
    PadHdr h;
    h.magic = PAD_MAGIC; h.ver = 1; h.tam = sizeof(DiaPadroes);
    h.crc = padCrc((const uint8_t*)padHist, sizeof(padHist));
    size_t w = f.write((const uint8_t*)&h, sizeof(h));
    w += f.write((const uint8_t*)padHist, sizeof(padHist));
    f.close();
    if (w != sizeof(h) + sizeof(padHist)) {
        LittleFS.remove(PAD_ARQ_TMP);
        if (!avisou) { avisou = true; adicionarLog("Padroes: gravacao incompleta (FS cheio?)"); }
        return;
    }
    LittleFS.remove(PAD_ARQ);
    if (!LittleFS.rename(PAD_ARQ_TMP, PAD_ARQ)) {
        if (!avisou) { avisou = true; adicionarLog("Padroes: falha no rename"); }
        return;
    }
    padSujo = false;
}

void padroesApagarTudo() {
    for (int i = 0; i < HIST_DIAS; i++) padLimparSlot(padHist[i], 0, 0, 0);
    memset(padCl, 0, sizeof(padCl));
    padNCl = 0; padDias = 0; padHorasCob = 0; padSujo = false;
    memset(&pad, 0, sizeof(pad));
    if (padFsOk) { LittleFS.remove(PAD_ARQ); LittleFS.remove(PAD_ARQ_TMP); }
    plano.usaPadroes = false;
    memset(plano.evAbs, 0, sizeof(plano.evAbs));
    memset(plano.baseOk, 0, sizeof(plano.baseOk));
    memset(plano.baseP, 0, sizeof(plano.baseP));
    plano.evRestAh = 0;
}

void padroesSincronizarDia(uint16_t a, uint8_t m, uint8_t d) {
    DiaPadroes& p = padHoje();
    if (p.ano == a && p.mes == m && p.dia == d) return;
    padLimparSlot(p, a, m, d);
    for (int c = 0; c < PAD_CLUSTERS; c++) padCl[c].feitos = 0;
    plano.horaPerfil = 255;
    padSujo = true;
}

// ── Clusters (padrões) ──────────────────────────────────────────────────────────
// Aproximação: corrente ±0,5 A (ou ±20%) e duração dentro de ~2x.
int padAchaCluster(float media, float durS, bool criar) {
    int best = -1; float bd = 1e9f;
    for (int c = 0; c < padNCl; c++) {
        if (!padCl[c].valido) continue;
        float ta = fmaxf(0.5f, 0.20f * padCl[c].cA);
        float da = fabsf(media - padCl[c].cA) / ta;
        float dd = fabsf(logf(fmaxf(durS, 1.0f) / fmaxf(padCl[c].cDur, 1.0f))) / 0.7f;
        if (da <= 1.0f && dd <= 1.0f && (da + dd) < bd) { bd = da + dd; best = c; }
    }
    if (best >= 0 || !criar) return best;

    if (padNCl < PAD_CLUSTERS) {
        ClusterPad& n = padCl[padNCl];
        memset(&n, 0, sizeof(n));
        n.valido = true; n.cA = media; n.cDur = durS;
        return padNCl++;
    }
    for (int c = 0; c < padNCl; c++) {
        float da = fabsf(media - padCl[c].cA) / fmaxf(0.5f, 0.20f * padCl[c].cA);
        float dd = fabsf(logf(fmaxf(durS, 1.0f) / fmaxf(padCl[c].cDur, 1.0f))) / 0.7f;
        if (da + dd < bd) { bd = da + dd; best = c; }
    }
    return best;
}

static void padAcumulaEvento(const EvPadrao& e, float w) {
    float media = e.mediaDA / 10.0f;
    float dur   = (float)e.durS;
    int ci = padAchaCluster(media, dur, true);
    if (ci < 0) return;
    ClusterPad& c = padCl[ci];
    float eAh = media * dur / 3600.0f;             // Ah = A medio x tempo
    float nw  = c.wSum + w;
    c.cA       = (c.cA * c.wSum + media * w) / nw;
    c.cDur     = (c.cDur * c.wSum + dur * w) / nw;
    c.eAh      = (c.eAh * c.wSum + eAh * w) / nw;
    c.solarPct = (c.solarPct * c.wSum + e.solarPct * w) / nw;
    if (e.picoA > c.picoA) c.picoA = e.picoA;
    c.wSum = nw;
    c.hist[(e.iniMin / 60) % 24] += w;
}

static void padAdicionarEvento(const EvPadrao& e) {
    DiaPadroes& p = padHoje();
    if (p.ano == 0) {
        static bool av = false;
        if (!av) { av = true; adicionarLog("Padroes: slot do dia nao sincronizado"); }
        return;
    }
    if (p.n < PAD_EV_DIA_MAX) {
        p.ev[p.n++] = e;
    } else {
        int mi = 0; uint32_t me = 0xFFFFFFFFUL;
        for (int k = 0; k < PAD_EV_DIA_MAX; k++) {
            uint32_t en = (uint32_t)p.ev[k].mediaDA * p.ev[k].durS;
            if (en < me) { me = en; mi = k; }
        }
        uint32_t ne = (uint32_t)e.mediaDA * e.durS;
        if (ne > me) p.ev[mi] = e;
        if (p.perdidos < 255) p.perdidos++;
    }
    padSujo = true;
    int c = padAchaCluster(e.mediaDA / 10.0f, (float)e.durS, false);
    if (c >= 0 && padCl[c].feitos < 250) padCl[c].feitos++;
}

static void padDescartaEvento() { pad.emEvento = false; pad.tAbaixo = 0; pad.tSub = 0; }

static void padFinalizarEvento(unsigned long fimMs) {
    float durS = (float)(fimMs - pad.tIni) / 1000.0f;
    padDescartaEvento();
    if (durS < PAD_MIN_DUR_S) return;
    float media = pad.somaAs / durS;
    if (media < PAD_MIN_MEDIA_A) return;

    EvPadrao e;
    e.iniMin   = pad.iniMin;
    e.durS     = (durS > 65535.0f) ? 65535 : (uint16_t)durS;
    float md   = media * 10.0f; if (md > 65535.0f) md = 65535.0f;
    e.mediaDA  = (uint16_t)md;
    e.picoA    = (pad.pico > 255.0f) ? 255 : (uint8_t)(pad.pico + 0.5f);
    float sp   = 100.0f * pad.solarAs / durS; if (sp > 100.0f) sp = 100.0f;
    e.solarPct = (uint8_t)sp;
    padAdicionarEvento(e);

    if (media >= 4.0f || durS >= 300.0f) {
        snprintf(tempBuf, sizeof(tempBuf), "Padrao: +%.1fA %lus %02u:%02u (%.2fAh)",
                 media, (unsigned long)durS, e.iniMin / 60, e.iniMin % 60, media * durS / 3600.0f);
        adicionarLog(tempBuf);
    }
}

// Fecha a hora que acabou: grava base, consumo da sobra e marca a hora como monitorada
static void padFecharHora() {
    bool ok = pad.hSlotOk;
    if (ok) {
        DiaPadroes& hd = padHist[pad.hSlot % HIST_DIAS];
        if (hd.ano == pad.hAno && hd.mes == pad.hMes && hd.dia == pad.hDia) {
            uint8_t h = pad.hHora % 24;
            if (pad.hN >= 30) {
                float m = pad.hSoma / pad.hN * 10.0f;
                if (m < 0.0f) m = 0.0f;
                if (m > 254.0f) m = 254.0f;
                hd.base[h] = (uint8_t)(m + 0.5f);
                hd.flut = (uint8_t)fminf(255.0f, pad.dev * 10.0f);
            }
            if (pad.sobAh > 0.0f) {
                float v = hd.sob[h] + pad.sobAh * 10.0f;
                if (v > 255.0f) v = 255.0f;
                hd.sob[h] = (uint8_t)(v + 0.5f);
            }
            if (pad.hAtivoS >= PAD_COB_MIN_S) hd.cob |= (1UL << h);
            padSujo = true;
        }
    }
    pad.hSoma = 0; pad.hN = 0; pad.sobAh = 0.0f; pad.hAtivoS = 0.0f;
    pad.hSlotOk = false;
}

// Detector em tempo real (1x/s)
void atualizarPadroes() {
    unsigned long agora = millis();
    if (pad.tUlt != 0 && agora - pad.tUlt < 1000UL) return;
    float dt = (pad.tUlt == 0) ? 1.0f : (float)(agora - pad.tUlt) / 1000.0f;
    if (dt > 5.0f) dt = 5.0f;
    pad.tUlt = agora;

    if (modoTeste || otaAtivo || !bmsFresca()) { padDescartaEvento(); return; }
    bool relogioOK = cfgTempo.ntpAtivo && ntpSincronizado;
    struct tm t;
    if (!relogioOK || acum.ultimoAcum == 0 || !obterTempoAtual(t)) return;

    float x = sensores.correnteCasa;
    if (!pad.iniciado) {
        pad.xs = x; pad.base = x; pad.dev = 0.3f; pad.curta = x;
        pad.lampAnt = estados.lampExt; pad.invAnt = estados.invBaixa;
        pad.altaAnt = estados.invAlta;
        pad.hHora = t.tm_hour; pad.hSlotOk = false;
        pad.iniciado = true;
        return;
    }
    pad.xs    += 0.4f * (x - pad.xs);
    pad.curta += (dt / (15.0f + dt)) * (x - pad.curta);

    if ((uint8_t)t.tm_hour != pad.hHora) { padFecharHora(); pad.hHora = t.tm_hour; }

    DiaPadroes& hj = padHoje();
    if (hj.ano != (uint16_t)(t.tm_year + 1900) || hj.mes != (uint8_t)(t.tm_mon + 1) ||
        hj.dia != (uint8_t)t.tm_mday) return;
    if (!pad.hSlotOk) {
        pad.hSlot = histIndiceAtual % HIST_DIAS;
        pad.hAno = hj.ano; pad.hMes = hj.mes; pad.hDia = hj.dia;
        pad.hSlotOk = true;
    }

    // ── SOBRA SOLAR: monitora, mas nao aprende nada ─────────────────────────────
    if (sobraSolarAgora()) {
        if (!pad.sobraAnt) {
            pad.sobraAnt = true;
            adicionarLog("Padroes: sobra solar, aprendizado pausado");
        }
        pad.hAtivoS += dt;
        float extra = pad.xs - pad.base;
        if (extra > 0.0f) pad.sobAh += extra * dt / 3600.0f;
        if (pad.xs < pad.base) {
            float a = dt / (PAD_TAU_BASE_S + dt);
            pad.base += a * (pad.xs - pad.base);
        }
        pad.lampAnt   = estados.lampExt;
        pad.invAnt    = estados.invBaixa;
        pad.altaAnt   = estados.invAlta;
        pad.medLamp   = false;
        pad.fimIgnora = false;
        pad.tDesv     = 0;
        padDescartaEvento();
        pad.ignoraAte = agora + 20000UL;
        return;
    }
    if (pad.sobraAnt) {
        pad.sobraAnt = false;
        adicionarLog("Padroes: sobra terminou, aprendizado retomado");
    }

    // ── degrau de base: lampada / Inv.Baixa mudou de estado ─────────────────────
    bool lampMud = (estados.lampExt  != pad.lampAnt);
    bool invMud  = (estados.invBaixa != pad.invAnt);
    if (lampMud || invMud) {
        bool limpo = lampMud && !invMud && !pad.emEvento && !estados.invAlta && agora >= pad.ignoraAte;
        pad.medLamp   = limpo;
        pad.refOk     = false;
        if (limpo) { pad.baseAntes = pad.base; pad.lampLigou = estados.lampExt; }
        pad.ignoraAte = agora + PAD_IGNORA_MS;
        pad.fimIgnora = true;
        pad.lampAnt   = estados.lampExt;
        pad.invAnt    = estados.invBaixa;
        pad.tDesv     = 0;
        padDescartaEvento();
    }

    if (agora < pad.ignoraAte) {
        if (pad.medLamp && !pad.refOk && (long)(pad.ignoraAte - agora) <= 15000L) {
            pad.ref = pad.curta; pad.refOk = true;
        }
        pad.altaAnt = estados.invAlta;
        return;
    }
    if (pad.fimIgnora) {
        pad.fimIgnora = false;
        if (pad.medLamp && pad.refOk && fabsf(pad.curta - pad.ref) < 0.3f) {
            float delta = pad.lampLigou ? (pad.curta - pad.baseAntes) : (pad.baseAntes - pad.curta);
            if (delta >= 0.2f && delta <= 15.0f) {
                plano.lampA = (plano.lampA > 0.05f) ? (plano.lampA * 0.5f + delta * 0.5f) : delta;
                prefs.begin("rrfs_plano", false);
                prefs.putFloat("lamp_a2", plano.lampA);
                prefs.end();
                snprintf(tempBuf, sizeof(tempBuf), "Plano: lampada ~%.1fA (degrau %.1fA)", plano.lampA, delta);
                adicionarLog(tempBuf);
            }
        }
        pad.medLamp = false;
        pad.base = pad.curta;
        pad.xs   = pad.curta;
        pad.altaAnt = estados.invAlta;
        return;
    }

    pad.hAtivoS += dt;                             // hora monitorada

    // ── contexto do Inv.Alta ────────────────────────────────────────────────────
    // usuario = Inv.Alta energizado POR PEDIDO DA CASA. Excesso solar sem pedido = so observa.
    bool altaEnerg = estados.invAlta && acAltaPresente();
    bool usuario   = altaEnerg && sinalCasaPresente();
    float thr = fmaxf(PAD_MIN_THR_A, PAD_K_DEV * pad.dev);

    if (!pad.emEvento) {
        // ── detecção: so com o Inv.Alta ligado a pedido da casa ─────────────────
        if (usuario && pad.xs > pad.base + thr) {
            if (pad.tSub == 0) pad.tSub = agora;
            if (agora - pad.tSub >= PAD_CONF_S * 1000UL) {
                pad.emEvento = true;
                pad.tIni     = pad.tSub;
                pad.tAbaixo  = 0;
                pad.pico     = pad.xs - pad.base;
                pad.somaAs   = (pad.xs - pad.base) * PAD_CONF_S;
                pad.solarAs  = 0;
                int mi = t.tm_hour * 60 + t.tm_min - (int)((agora - pad.tSub) / 60000UL);
                if (mi < 0) mi += 1440;
                pad.iniMin   = (uint16_t)mi;
            }
        } else {
            pad.tSub = 0;
        }

        // ── base estavel ─────────────────────────────────────────────────────────
        if (!pad.emEvento) {
            float a = dt / (PAD_TAU_BASE_S + dt);
            float d = pad.xs - pad.base;
            if (estados.invAlta) {
                // Inv.Alta ligado: a corrente extra e dele, nao da base. Base congelada (so desce).
                pad.tDesv = 0;
                if (d < 0.0f) pad.base += a * d;
            } else if (fabsf(d) > thr) {
                // Inv.Alta desligado: a mudanca e carga do Inv.Baixa (notebook, TV...).
                // Se ficar assim por 30 s, a base salta para o novo patamar estavel.
                if (pad.tDesv == 0) pad.tDesv = agora;
                if (agora - pad.tDesv >= PAD_SNAP_MS) {
                    pad.base  = pad.curta;
                    pad.tDesv = 0;
                }
            } else {
                pad.tDesv = 0;
                pad.dev  += a * (fabsf(d) - pad.dev);
                pad.base += a * d;
            }
            pad.hSoma += pad.base - (estados.lampExt ? plano.lampA : 0.0f);
            pad.hN++;
        }
    } else {
        // ── evento em andamento: integra A acima da base (Ah = A x tempo) ───────
        float extra = pad.xs - pad.base; if (extra < 0.0f) extra = 0.0f;
        pad.somaAs += extra * dt;
        if (extra > pad.pico) pad.pico = extra;
        if (sensores.correntePainel > 0.3f && sensores.correntePainel >= pad.xs * 0.9f)
            pad.solarAs += dt;

        // Inv.Alta desligou: a carga pesada acabou
        if (!estados.invAlta) {
            padFinalizarEvento(agora);
            pad.altaAnt = estados.invAlta;
            return;
        }

        float endThr = fmaxf(0.5f, thr * 0.5f);
        if (extra < endThr) {
            if (pad.tAbaixo == 0) pad.tAbaixo = agora;
            if (agora - pad.tAbaixo >= PAD_FIM_S * 1000UL) {
                padFinalizarEvento(pad.tAbaixo);
                pad.altaAnt = estados.invAlta;
                return;
            }
        } else {
            pad.tAbaixo = 0;
        }
        if (agora - pad.tIni > PAD_MAX_EVENTO_S) {
            padDescartaEvento();
            pad.base = pad.xs;
            adicionarLog("Padrao: carga fixa (>4h), nova base");
        }
    }
    pad.altaAnt = estados.invAlta;
}

// ── Aprendizado (1x por hora) ───────────────────────────────────────────────────
void padroesRecalcular() {
    float somaP[24], somaB[24], wB[24], covW[24];
    uint8_t nP[24];
    for (int h = 0; h < 24; h++) { somaP[h] = somaB[h] = wB[h] = covW[h] = 0; nP[h] = 0; }
    memset(padCl, 0, sizeof(padCl));
    padNCl = 0;

    int diasPad = 0, usados = 0, horasCob = 0;
    int dias = (histTotalDias > HIST_DIAS) ? HIST_DIAS : histTotalDias;

    for (int i = dias - 1; i >= 0; i--) {
        RegistroDia d; bool ok;
        int slot = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
        if (i == 0) { d = histDiaAtual; ok = true; }
        else ok = carregarDiaFlash(slot, d);
        if (!ok || !d.valido) continue;
        usados++;

        const DiaPadroes& p = padHist[slot];
        bool pOk = (p.ano == d.ano && p.mes == d.mes && p.dia == d.dia);

        for (int h = 0; h < 24; h++) {
            float v = d.horas[h].casaAh;
            if (pOk) v -= p.sob[h] / 10.0f;
            if (v > 0.05f) { somaP[h] += v; nP[h]++; }
        }
        if (!pOk || p.cob == 0) { esp_task_wdt_reset(); continue; }

        float w = powf(PAD_DIAS_PESO, (float)i);
        diasPad++;
        for (int h = 0; h < 24; h++) {
            if (p.cob & (1UL << h)) { covW[h] += w; horasCob++; }
            if (p.base[h] != 255)   { somaB[h] += w * p.base[h] / 10.0f; wB[h] += w; }
        }
        for (int k = 0; k < p.n && k < PAD_EV_DIA_MAX; k++) {
            int hs = (p.ev[k].iniMin / 60) % 24;
            if (p.cob & (1UL << hs)) padAcumulaEvento(p.ev[k], w);
        }
        esp_task_wdt_reset();
    }

    plano.horaPico = -1; plano.picoA = 0.0f;
    for (int h = 0; h < 24; h++) {
        plano.perfilOk[h] = (nP[h] > 0);
        plano.perfil[h]   = nP[h] ? somaP[h] / nP[h] : 0.0f;
        if (plano.perfilOk[h] && plano.perfil[h] > plano.picoA) {
            plano.picoA = plano.perfil[h]; plano.horaPico = h;
        }
        plano.baseOk[h] = (wB[h] > 0);
        plano.baseP[h]  = wB[h] > 0 ? somaB[h] / wB[h] : 0.0f;
    }
    plano.usaHistorico = (usados > 0);

    int j = 0;
    for (int c = 0; c < padNCl; c++) {
        ClusterPad& k = padCl[c];
        float por = 0.0f;
        for (int h = 0; h < 24; h++) {
            k.rate[h] = (covW[h] > 0.0f) ? k.hist[h] / covW[h] : 0.0f;
            por += k.rate[h];
        }
        k.porDia = por;
        if (por < PAD_PORDIA_MIN) continue;
        if (j != c) padCl[j] = padCl[c];
        j++;
    }
    for (int c = j; c < padNCl; c++) memset(&padCl[c], 0, sizeof(ClusterPad));
    padNCl = j;
    padDias = diasPad;
    padHorasCob = horasCob;
    plano.usaPadroes = (horasCob >= PAD_MIN_HORAS_APR);

    DiaPadroes& hj = padHoje();
    for (int k = 0; k < hj.n && k < PAD_EV_DIA_MAX; k++) {
        int c = padAchaCluster(hj.ev[k].mediaDA / 10.0f, (float)hj.ev[k].durS, false);
        if (c >= 0 && padCl[c].feitos < 250) padCl[c].feitos++;
    }

    snprintf(tempBuf, sizeof(tempBuf), "Padroes: %dd %dh cob, %d tipo(s)", diasPad, horasCob, (int)padNCl);
    adicionarLog(tempBuf);
}

// espalha a energia (Ah) de uma ocorrencia pelas horas que ela cobre (max 6 h)
static void padAddEv(float* arr, int absH, float ah, float durS) {
    int nh = (int)ceilf(durS / 3600.0f);
    if (nh < 1) nh = 1;
    if (nh > 6) nh = 6;
    float parte = ah / nh;
    for (int k = 0; k < nh; k++) {
        int idx = absH + k;
        if (idx >= 0 && idx < 72) arr[idx] += parte;
    }
}

// Previsão de eventos em Ah: taxa por hora x energia de cada padrao.
// Um pico alto e curto gera poucos Ah, entao pesa pouco no planejador.
void padroesPrever(int hAgora, int minAgora) {
    for (int i = 0; i < 72; i++) plano.evAbs[i] = 0.0f;
    plano.evRestAh = 0.0f;
    if (!plano.usaPadroes) return;
    if (hAgora < 0) hAgora = 0;
    if (hAgora > 23) hAgora = 23;
    if (minAgora < 0) minAgora = 0;
    if (minAgora > 59) minAgora = 59;
    float fracAgora = (60 - minAgora) / 60.0f;

    for (int c = 0; c < padNCl; c++) {
        ClusterPad& k = padCl[c];
        k.restHoje = 0.0f;
        if (!k.valido || k.porDia < PAD_PORDIA_MIN) continue;
        for (int h = hAgora; h < 24; h++) {
            float occ = k.rate[h];
            if (occ <= 0.0f) continue;
            padAddEv(plano.evAbs, h, occ * k.eAh, k.cDur);
            k.restHoje += (h == hAgora) ? occ * fracAgora : occ;
        }
        for (int d = 1; d <= 2; d++)
            for (int h = 0; h < 24; h++) {
                float occ = k.rate[h];
                if (occ > 0.0f) padAddEv(plano.evAbs, d * 24 + h, occ * k.eAh, k.cDur);
            }
    }
    plano.evRestAh = plano.evAbs[hAgora] * fracAgora;
    for (int h = hAgora + 1; h < 24; h++) plano.evRestAh += plano.evAbs[h];
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 19 — EXCESSO DE ENERGIA SOLAR → liga Inv.Alta sozinho
// ═══════════════════════════════════════════════════════════════════════════════════
static const char* excMotivoTxt(uint8_t m) {
    switch (m) {
        case 0:  return "condicoes OK";
        case 1:  return "teste/OTA em andamento";
        case 2:  return "sistema estabilizando";
        case 3:  return "BMS sem dados";
        case 4:  return "protecao da bateria ativa";
        case 5:  return "aguardando relogio NTP";
        case 6:  return "fim da geracao do dia";
        case 7:  return "casa ja pediu o Inv.Alta";
        case 8:  return "pausa apos ultimo excesso";
        case 9:  return "bateria ainda nao esta cheia";
        case 10: return "painel gerando pouco";
        case 11: return "bateria descarregando";
        case 12: return "bateria ainda carregando";
        case 13: return "repouso do compressor (3 min)";
        case 14: return "previsao nao mostra sobra de energia";
        case 20: return "tolerancia: aguardando controlador subir";
        case 21: return "SOC caiu 1%: confirmando fim do excesso";
        case 22: return "excesso mantido";
        default: return "?";
    }
}

static bool excPrevisaoPermiteEntrada(float consumoA) {
    if (!previsao.valido || !cfgPrevisao.ativa || !cfgPrevisao.configurada) return true;
    int idx = getIndiceHoraAtual();
    if (idx < 0) return true;
    float vB      = (dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;
    float ahAtual = previsao.ahPrevisto[idx];
    float ahProx  = (idx + 1 < PREV_TOTAL_HORAS) ? previsao.ahPrevisto[idx + 1] : 0.0f;
    return ((ahAtual - consumoA) * vB >= EXC_PREV_SOBRA_W) &&
           ((ahProx  - consumoA) * vB >= EXC_PREV_SOBRA_PROX_W);
}

static void excTerminar(const char* motivo, bool falha) {
    unsigned long agora = millis();
    bool casaMantem = sinalCasaPresente();
    excesso.fase     = 0;
    excesso.pede     = false;
    excesso.tConf    = 0;
    excesso.tFimConf = 0;
    excesso.tCool    = agora;
    excesso.coolMs   = falha ? EXC_COOLDOWN_FALHA_MS : EXC_COOLDOWN_MS;
    excesso.motivo   = 8;
    strncpy(excesso.ultimoFim, motivo, sizeof(excesso.ultimoFim) - 1);
    excesso.ultimoFim[sizeof(excesso.ultimoFim) - 1] = '\0';
    snprintf(tempBuf, sizeof(tempBuf), "Excesso FIM: %s", motivo);
    adicionarLog(tempBuf);
    if (casaMantem) adicionarLog("Excesso FIM: casa pediu, Inv.Alta segue ligado");
}

static bool excFimDeGeracao(const struct tm& t) {
    if (previsao.valido && cfgPrevisao.ativa && cfgPrevisao.configurada) {
        int idx = getIndiceHoraAtual();
        if (idx >= 0) {
            int ini = idx - t.tm_hour;
            if (ini < 0) ini = 0;
            float pico = 0.0f; int hPico = 0;
            for (int i = 0; i < 24 && ini + i < PREV_TOTAL_HORAS; i++) {
                if (previsao.ahPrevisto[ini + i] > pico) { pico = previsao.ahPrevisto[ini + i]; hPico = i; }
            }
            if (pico < 0.5f) return true;
            float ahH = previsao.ahPrevisto[idx];
            return (t.tm_hour > hPico && ahH < pico * EXC_FIM_FRACAO_PICO);
        }
    }
    return t.tm_hour >= EXC_HORA_FIM_PADRAO;
}

void atualizarExcesso() {
    static unsigned long tSemSol = 0, tFalhaConf = 0, tMLog = 0;
    static uint8_t mLog = 255;
    unsigned long agora = millis();

    if (modoTeste || otaAtivo) {
        excesso.fase = 0; excesso.pede = false; excesso.tConf = 0; excesso.tFimConf = 0;
        excesso.motivo = 1;
        return;
    }

    bool  bms       = bmsFresca();
    float soc       = dadosBMS.soc;
    float ip        = sensores.correntePainel;
    float ib        = sensores.correnteBateria;
    bool  casaQuer  = sinalCasaPresente();
    bool  relogioOK = cfgTempo.ntpAtivo && ntpSincronizado;

    struct tm t;
    obterTempoAtual(t);

    bool fimGer = relogioOK && excFimDeGeracao(t);
    excesso.fimGeracao = fimGer;
    excesso.diaBloq    = fimGer ? 1 : 0;

    // ══ FASE 2: EXCESSO ATIVO ══════════════════════════════════════════════════
    if (excesso.fase == 2) {
        bool rapido = (agora - excesso.tAtiv) < EXC_ATIVO_RAPIDO_MS;

        if (!bms)                    { excTerminar("BMS sem dados", true); return; }
        if (protecao.ativa)          { excTerminar("protecao da bateria", true); return; }
        if (soc < EXC_SOC_SAIDA_PCT) { excTerminar("SOC abaixo do minimo de saida", true); return; }
        if (maqAlta.estado == INV_ALTA_DIAG_OFF) { excTerminar("Inv.Alta nao energizou", true); return; }

        if (!estados.invAlta) {
            excesso.tAtiv    = agora;
            excesso.tFimConf = 0;
            excSocRef        = soc;
            tSemSol          = 0;
            excesso.motivo   = 20;
            return;
        }

        if (soc > excSocRef) excSocRef = soc;

        if (agora - excesso.tAtiv < EXC_GRACA_MS) {
            excesso.tFimConf = 0;
            tSemSol          = 0;
            excesso.motivo   = 20;
            return;
        }

        if (ip < 0.5f) {
            if (tSemSol == 0) tSemSol = agora;
            if (agora - tSemSol >= EXC_SEM_SOL_CONFIRMA_MS) {
                tSemSol = 0;
                excTerminar("painel sem geracao", rapido);
                return;
            }
            excesso.motivo = 21;
            return;
        }
        tSemSol = 0;

        bool caiu       = (soc <= excSocRef - EXC_SOC_QUEDA_PCT + 0.01f);
        bool carregando = (ib > EXC_BAT_VOLTA_CARGA_A);
        if (caiu && !carregando) {
            if (excesso.tFimConf == 0) excesso.tFimConf = agora;
            if (agora - excesso.tFimConf >= EXC_QUEDA_CONFIRMA_MS) {
                excTerminar("SOC caiu 1%: painel nao cobre a casa", rapido);
                return;
            }
            excesso.motivo = 21;
        } else {
            excesso.tFimConf = 0;
            excesso.motivo   = 22;
        }
        return;
    }

    // ══ FASE 0/1: DETECTANDO ═══════════════════════════════════════════════════
    float socMin = (excesso.fase == 1) ? EXC_SOC_CONF_MANTEM_PCT : EXC_SOC_ENTRADA_PCT;
    uint8_t m = 0;
    if      (!sistemaEstabilizado)                         m = 2;
    else if (!bms || !cfgBMS.configurada)                  m = 3;
    else if (protecao.ativa)                               m = 4;
    else if (!relogioOK)                                   m = 5;
    else if (fimGer)                                       m = 6;
    else if (casaQuer || estados.invAlta)                  m = 7;
    else if (excesso.tCool != 0 && agora - excesso.tCool < excesso.coolMs) m = 8;
    else if (!altaRepousoOK())                             m = 13;
    else if (soc < socMin)                                 m = 9;
    else if (ip < EXC_PAINEL_MIN_A)                        m = 10;
    else if (ib < -EXC_BAT_DESC_ENTRADA_A)                 m = 11;
    else if (ib > EXC_BAT_CARGA_ZERO_A)                    m = 12;
    else if (!excPrevisaoPermiteEntrada(sensores.correnteCasa)) m = 14;

    if (m == 0) mLog = 255;
    else if (m != mLog && soc >= 98.0f && agora - tMLog > 60000UL) {
        mLog = m; tMLog = agora;
        snprintf(tempBuf, sizeof(tempBuf), "Excesso aguarda: %s", excMotivoTxt(m));
        adicionarLog(tempBuf);
    }

    bool suave = (m == 9 || m == 10 || m == 11 || m == 12 || m == 14);
    if (m != 0 && suave && excesso.fase == 1) {
        if (tFalhaConf == 0) tFalhaConf = agora;
        if (agora - tFalhaConf < EXC_CONF_TOLERA_MS) { excesso.motivo = m; return; }
    }
    if (m == 0) tFalhaConf = 0;

    excesso.motivo = m;

    if (m != 0) {
        if (excesso.fase == 1) {
            snprintf(tempBuf, sizeof(tempBuf), "Excesso cancelado: %s", excMotivoTxt(m));
            adicionarLog(tempBuf);
        }
        excesso.fase  = 0;
        excesso.tConf = 0;
        tFalhaConf    = 0;
        return;
    }

    if (excesso.fase == 0) {
        excesso.fase  = 1;
        excesso.tConf = agora;
        tFalhaConf    = 0;
        adicionarLog("Excesso detectado (bat cheia), confirmando 60s");
        return;
    }

    if (agora - excesso.tConf >= EXC_CONFIRMA_MS) {
        excesso.fase     = 2;
        excesso.pede     = true;
        excesso.tAtiv    = agora;
        excesso.tFimConf = 0;
        excSocRef        = soc;
        tSemSol          = 0;
        excesso.eventos++;
        snprintf(tempBuf, sizeof(tempBuf), "Excesso ON SOC%.0f P%.1fA B%.1fA C%.1fA",
                 soc, ip, ib, sensores.correnteCasa);
        adicionarLog(tempBuf);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 20 — MÁQUINA DE ESTADOS INVERSOR ALTA
// ═══════════════════════════════════════════════════════════════════════════════════
// Só liga quando a CASA pede (pino 39) ou há excesso solar, e a bateria está acima
// da reserva. O pino 36 (acAlta) é só DIAGNÓSTICO da saída AC.

void maquinaInvAlta() {
    if (modoTeste || otaAtivo) return;
    unsigned long agora = millis();

    if (!sistemaEstabilizado) {
        if (estados.invAlta) {
            desligarInvAlta();
            maqAlta.estado        = INV_ALTA_DESLIGADO;
            maqAlta.tInicioEstado = agora;
        }
        return;
    }

    bool batOK    = !protecao.ativa;
    bool casaQuer = sinalCasaPresente();
    bool quer     = casaQuer || excesso.pede;

    static bool          veioDeLigado = false;
    static unsigned long tLogRep      = 0;

    switch (maqAlta.estado) {

        case INV_ALTA_DESLIGADO:
            diagRetryLongoAlta = false;
            if (quer && batOK) {
                if (altaRepousoOK()) {
                    ligarInvAlta();
                    veioDeLigado          = false;
                    maqAlta.estado        = INV_ALTA_DIAG_AGUARDA;
                    maqAlta.tInicioEstado = agora;
                    maqAlta.tentativas    = 0;
                    adicionarLog(casaQuer ? "InvAlta: casa pediu, ligou"
                                          : "InvAlta: excesso solar, ligou");
                } else if (agora - tLogRep > 30000UL) {
                    tLogRep = agora;
                    snprintf(tempBuf, sizeof(tempBuf), "InvAlta: repouso compressor %lus",
                             (unsigned long)((ALTA_MIN_OFF_MS - (agora - tUltimoOffAlta)) / 1000UL));
                    adicionarLog(tempBuf);
                }
            } else if (estados.invAlta) {
                desligarInvAlta();
            }
            break;

        case INV_ALTA_LIGADO:
            if (!batOK) {
                desligarInvAlta();
                maqAlta.estado        = INV_ALTA_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvAlta: DESLIGADO reserva bat");
            } else if (!quer) {
                if (agora - maqAlta.tInicioEstado >= ALTA_MIN_ON_MS) {
                    desligarInvAlta();
                    maqAlta.estado        = INV_ALTA_DESLIGADO;
                    maqAlta.tInicioEstado = agora;
                    adicionarLog("InvAlta: sem pedido (casa/excesso), OFF");
                }
            } else if (!acAltaPresente()) {
                veioDeLigado          = true;
                maqAlta.estado        = INV_ALTA_DIAG_AGUARDA;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas    = 0;
                adicionarLog("InvAlta: AC perdido, aguardando 30s");
            }
            break;

        case INV_ALTA_DIAG_AGUARDA: {
            unsigned long limite = veioDeLigado ? ALTA_PERDEU_AC_MS : DIAG_AGUARDA_AC_ALTA_MS;
            if (!batOK) {
                desligarInvAlta();
                maqAlta.estado        = INV_ALTA_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvAlta: reserva bat (diag)");
            } else if (!quer) {
                desligarInvAlta();
                maqAlta.estado        = INV_ALTA_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvAlta: sem pedido (diag)");
            } else if (acAltaPresente()) {
                maqAlta.estado        = INV_ALTA_LIGADO;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas    = 0;
                maqAlta.totalLigadas++;
                veioDeLigado          = false;
                adicionarLog("InvAlta: AC OK, LIGADO");
            } else if (agora - maqAlta.tInicioEstado >= limite) {
                desligarInvAlta();
                maqAlta.estado        = INV_ALTA_DIAG_OFF;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas++;
                snprintf(tempBuf, sizeof(tempBuf), "InvAlta: sem AC %lus, tent %d/%d",
                         limite / 1000UL, maqAlta.tentativas, DIAG_MAX_TENTATIVAS);
                adicionarLog(tempBuf);
            }
            break;
        }

        case INV_ALTA_DIAG_OFF:
            if (!quer || !batOK) {
                diagRetryLongoAlta    = false;
                veioDeLigado          = false;
                maqAlta.estado        = INV_ALTA_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog(!batOK ? "InvAlta: reserva bat (off diag)"
                                    : "InvAlta: sem pedido (off diag)");
            } else {
                unsigned long espera = diagRetryLongoAlta ? ALTA_RETRY_LONGO_MS : ALTA_MIN_OFF_MS;
                if (agora - maqAlta.tInicioEstado >= espera) {
                    if (diagRetryLongoAlta) {
                        diagRetryLongoAlta = false;
                        maqAlta.tentativas = 0;
                    } else if (maqAlta.tentativas >= DIAG_MAX_TENTATIVAS) {
                        diagRetryLongoAlta    = true;
                        maqAlta.totalFalhas++;
                        maqAlta.tInicioEstado = agora;
                        adicionarLog("InvAlta: falha, nova tentativa em 10min");
                        break;
                    }
                    ligarInvAlta();
                    veioDeLigado          = false;
                    maqAlta.estado        = INV_ALTA_DIAG_AGUARDA;
                    maqAlta.tInicioEstado = agora;
                    adicionarLog("InvAlta: religando diagnostico");
                }
            }
            break;
    }
}
// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 21 — PLANEJADOR DE ENERGIA (padrões em Ah + previsão solar + próximo excesso)
// ═══════════════════════════════════════════════════════════════════════════════════
// consumo(h) = base(h) + eventos esperados(h) [Ah] + lampada(h)
// Eventos do Inv.Alta entram so como ENERGIA (Ah): pico de corrente curto nao
// causa falso positivo. Tambem calcula a proxima hora com excesso solar previsto.

void carregarPlanoEnergia() {
    memset(&plano, 0, sizeof(plano));
    plano.verConf    = -1;
    plano.verPend    = -1;
    plano.horaPerfil = 255;
    plano.horaPico   = -1;
    plano.com.excH = plano.sem.excH = -1;
    prefs.begin("rrfs_plano", true);
    plano.lampA = prefs.getFloat("lamp_a2", 0.0f);
    prefs.end();
    if (plano.lampA < 0.0f || plano.lampA > 15.0f) plano.lampA = 0.0f;
}

void salvarPlanoEnergia() {
    prefs.begin("rrfs_plano", false);
    prefs.putFloat("lamp_a2", plano.lampA);
    prefs.end();
}

static float lampFracHora(int h, float gerRaw) {
    if (gerRaw > 0.5f) return 0.0f;
    if (h >= LAMP_EXT_HORA_LIGA || h < LAMP_EXT_HORA_DESLIGA) return 1.0f;
    if (h == LAMP_EXT_HORA_DESLIGA) return LAMP_EXT_MIN_DESLIGA / 60.0f;
    return 0.0f;
}

static SimPlano planoSimular(bool comLamp, int horaAgora, int minAgora, int idx,
                             float nivel0, float capUtil) {
    SimPlano r;
    memset(&r, 0, sizeof(r));
    r.nivelMin = 1e9f;
    r.idxFalta = -1;
    r.idxExc   = -1;
    float nivel  = nivel0;
    float f0     = (60 - minAgora) / 60.0f;
    float somaC  = 0, somaG = 0;

    // escala da base historica pela base atual (efeito some em ~12 h)
    float fEsc = 1.0f;
    if (plano.usaPadroes && plano.baseOk[horaAgora] && plano.baseP[horaAgora] > 0.2f) {
        fEsc = plano.consBase / plano.baseP[horaAgora];
        if (fEsc < 0.6f) fEsc = 0.6f;
        if (fEsc > 1.6f) fEsc = 1.6f;
    }

    for (int i = 0; i < PLANO_HORIZONTE_H; i++) {
        int   h    = (horaAgora + i) % 24;
        float frac = (i == 0) ? f0 : 1.0f;
        float gRaw = (idx + i < PREV_TOTAL_HORAS) ? previsao.ahPrevisto[idx + i] : 0.0f;
        float lf   = lampFracHora(h, gRaw);

        float semLamp, ev = 0.0f;
        if (plano.usaPadroes) {
            if (i == 0 || !plano.baseOk[h]) {
                semLamp = plano.consBase;
            } else {
                float dec = 1.0f - (float)i / 12.0f;
                if (dec < 0.0f) dec = 0.0f;
                semLamp = plano.baseP[h] * (1.0f + (fEsc - 1.0f) * dec);
            }
            int ab = horaAgora + i;
            if (ab < 72) ev = plano.evAbs[ab];          // Ah esperados dos equipamentos pesados
        } else {
            if (i == 0 || !plano.perfilOk[h]) {
                semLamp = plano.consBase;
            } else {
                semLamp = plano.perfil[h] - lf * plano.lampA;
                if (semLamp < 0.0f) semLamp = 0.0f;
            }
        }
        float cons = semLamp + ev + (comLamp ? lf * plano.lampA : 0.0f);
        float ger  = gRaw * PLANO_FATOR_GERACAO;

        float novo = nivel + (ger - cons) * frac;
        // excesso previsto: bateria enche e ainda sobra geracao (energia que seria desperdicada)
        if (r.idxExc < 0 && ger > cons && novo > capUtil + 0.05f) r.idxExc = i;
        nivel = (novo > capUtil) ? capUtil : novo;
        somaC += cons * frac;
        somaG += ger  * frac;

        if (nivel < r.nivelMin) {
            r.nivelMin = nivel;
            r.idxMin   = i;
            r.cons     = somaC;
            r.ger      = somaG;
        }
        if (nivel < 0.0f && r.idxFalta < 0) r.idxFalta = i;
    }

    int a = horaAgora + r.idxMin + 1;
    r.fimH = a % 24;
    r.fimD = a / 24;
    if (r.idxFalta >= 0) {
        a = horaAgora + r.idxFalta + 1;
        r.faltaH = a % 24;
        r.faltaD = a / 24;
    } else {
        r.faltaH = -1;
        r.faltaD = 0;
    }
    if (r.idxExc >= 0) {
        a = horaAgora + r.idxExc;
        r.excH = a % 24;
        r.excD = a / 24;
    } else {
        r.excH = -1;
        r.excD = 0;
    }
    return r;
}

void atualizarPlanoEnergia() {
    static unsigned long tUlt = 0, tSem = 0;
    static bool sobraLogada = false;

    if (modoTeste || otaAtivo) { plano.sobraSolar = false; return; }

    unsigned long agora = millis();
    if (tUlt != 0 && agora - tUlt < PLANO_INTERVALO_MS) return;
    unsigned long dt = (tUlt == 0) ? PLANO_INTERVALO_MS : (agora - tUlt);
    tUlt = agora;

    bool sobra = sobraSolarAgora();

    // ── 1) Consumo base da casa (sem lampada) ───────────────────────────────
    // consLenta (fallback) NAO e atualizada com o Inv.Alta ligado: os picos dos
    // equipamentos pesados nao fazem parte da base. Em sobra so pode descer.
    if (bmsFresca()) {
        float casa = sensores.correnteCasa;
        float base = casa - (estados.lampExt ? plano.lampA : 0.0f);
        if (base < 0.0f) base = 0.0f;
        if (!plano.consIniciado) { plano.consLenta = base; plano.consIniciado = true; }
        else if (!estados.invAlta && (!sobra || base < plano.consLenta))
            plano.consLenta += ((float)dt / (600000.0f + dt)) * (base - plano.consLenta);

        if (plano.usaPadroes && pad.iniciado) {
            float b = pad.base - (estados.lampExt ? plano.lampA : 0.0f);
            plano.consBase = (b < 0.0f) ? 0.0f : b;
        } else {
            plano.consBase = plano.consLenta;
        }
    }

    // ── 2) Aprendizado dos padroes (a cada virada de hora) ──────────────────
    struct tm t;
    bool tempoOK = obterTempoAtual(t);
    if (tempoOK && t.tm_hour != plano.horaPerfil) {
        plano.horaPerfil = t.tm_hour;
        padroesRecalcular();
    }
    if (tempoOK) padroesPrever(t.tm_hour, t.tm_min);

    // ── 3) Protecao ja ativa: o planejador nao interfere ────────────────────
    if (protecao.ativa) {
        plano.sobraSolar    = false;
        plano.verConf       = -2;
        plano.verPend       = -1;
        plano.lampBloqueada = false;
        plano.reducaoA = plano.reducaoW = 0;
        return;
    }

    // ── 4) EXCESSO REAL: Inv.Alta ligado com a sobra dos paineis ────────────
    bool excReal = (excesso.fase == 2);
    plano.sobraSolar = excReal;
    if (excReal) {
        if (!sobraLogada) {
            sobraLogada = true;
            adicionarLog("Plano: excesso ativo, pode ligar o maximo");
        }
        plano.verConf       = 0;
        plano.verPend       = 0;
        plano.tPend         = agora;
        plano.lampBloqueada = false;
        plano.analisandoAte = 0;
        plano.reducaoA = plano.reducaoW = 0;
        plano.semMotivo     = 0;
        tSem = 0;
        return;
    }
    sobraLogada = false;

    // ── 5) Pre-requisitos ───────────────────────────────────────────────────
    bool  relogioOK = tempoOK && cfgTempo.ntpAtivo && ntpSincronizado;
    int   idx = getIndiceHoraAtual();
    float cap = dadosBanco.capacidadeTotal;
    int   sm  = 0;
    if (!cfgPrevisao.ativa || !cfgPrevisao.configurada || !previsao.valido) sm = 1;
    else if (!bmsFresca() || !cfgBMS.configurada)                           sm = 2;
    else if (!relogioOK || idx < 0)                                         sm = 3;
    else if (cap < 1.0f || !plano.consIniciado)                             sm = 4;
    else if (!plano.usaPadroes)                                             sm = 5;   // pouca rotina aprendida: nao bloqueia a lampada

    if (sm) {
        plano.semMotivo = sm;
        if (tSem == 0) tSem = agora;
        if (agora - tSem > 900000UL) {
            if (plano.verConf != -1) adicionarLog("Plano: sem dados, modo normal");
            plano.verConf       = -1;
            plano.verPend       = -1;
            plano.lampBloqueada = false;
            plano.reducaoA = plano.reducaoW = 0;
        }
        return;
    }
    tSem = 0;
    plano.semMotivo = 0;

    // ── 6) Simulacao com e sem lampada ──────────────────────────────────────
    float soc     = dadosBMS.soc;
    float nivel0  = cap * (soc - protecao.reservaPct) / 100.0f;
    if (nivel0 < 0.0f) nivel0 = 0.0f;
    float capUtil = cap * (100.0f - protecao.reservaPct) / 100.0f;
    float margem  = cap * PLANO_MARGEM_PCT / 100.0f;
    plano.margemAh = margem;

    plano.com = planoSimular(true,  t.tm_hour, t.tm_min, idx, nivel0, capUtil);
    plano.sem = planoSimular(false, t.tm_hour, t.tm_min, idx, nivel0, capUtil);

    float margemCom = margem;
    if (plano.lampBloqueada) margemCom += cap * PLANO_MARGEM_LIB_PCT / 100.0f;
    bool comOk = (plano.com.nivelMin >= margemCom);
    bool semOk = (plano.sem.nivelMin >= margem);
    int  ver   = comOk ? 0 : (semOk ? 1 : 2);

    // ── 7) Confirmacao temporal do veredito ─────────────────────────────────
    if (plano.verConf < 0) {
        plano.verConf = ver;
        plano.verPend = ver;
        plano.tPend   = agora;
    } else {
        if (ver != plano.verPend) { plano.verPend = ver; plano.tPend = agora; }
        if (ver != plano.verConf) {
            unsigned long espera = (ver > plano.verConf) ? PLANO_CONFIRMA_BLOQ_MS
            : PLANO_CONFIRMA_LIB_MS;
            if (agora - plano.tPend >= espera) {
                plano.verConf = ver;
                const char* m = (ver == 0) ? "Plano: energia OK, lampada liberada"
                : (ver == 1) ? "Plano: lampada OFF basta"
                : "Plano: ECONOMIZAR energia";
                adicionarLog(m);
            }
        }
    }

    bool bloq = (plano.verConf >= 1);
    if (bloq != plano.lampBloqueada) {
        plano.lampBloqueada = bloq;
        if (bloq) plano.analisandoAte = agora + 600000UL;
    }

    // ── 8) Quanto precisa reduzir (nunca negativo) ──────────────────────────
    plano.reducaoA = 0; plano.reducaoW = 0;
    if (plano.verConf == 2) {
        float falta = margem - plano.sem.nivelMin;
        if (falta > 0.0f) {
            int periodo = plano.sem.idxMin + 1;
            if (periodo < 1) periodo = 1;
            float vB = (dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;
            plano.reducaoA = falta / periodo;
            plano.reducaoW = plano.reducaoA * vB;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 22 — CONTROLE ILUMINAÇÃO EXTERNA
// ═══════════════════════════════════════════════════════════════════════════════════

void controlarLampExt() {
    if (modoTeste) return;

    if (protecao.ativa && estados.lampExt) {
        desligarLampExt();
        adicionarLog("LampExt OFF: reserva bateria");
        return;
    }

    if (plano.lampBloqueada) {
        if (estados.lampExt) {
            desligarLampExt();
            adicionarLog("LampExt OFF: economia (plano)");
        }
        return;
    }

    unsigned long agora = millis();
    if (agora - ultimoLampCheck < LAMP_EXT_CHECK_MS) return;
    ultimoLampCheck = agora;

    static bool socBloq = false;
    if (bmsFresca()) {
        if (!socBloq && dadosBMS.soc <= limiares.pctLampExt) socBloq = true;
        else if (socBloq && dadosBMS.soc >= limiares.pctLampExt + LAMP_HISTERESE_PCT) socBloq = false;
    } else {
        socBloq = false;
    }

    bool gerando = (sensores.correntePainel > 0.5f);
    bool batOK   = !protecao.ativa && !socBloq;

    struct tm t;
    bool tempoOK = obterTempoAtual(t);

    if (gerando && estados.lampExt) {
        desligarLampExt();
        adicionarLog("LampExt OFF: solar");
        return;
    }
    if (estados.lampExt && !batOK) {
        desligarLampExt();
        adicionarLog("LampExt OFF: SOC baixo");
        return;
    }
    if (estados.lampExt && tempoOK) {
        if (t.tm_hour == LAMP_EXT_HORA_DESLIGA && t.tm_min >= LAMP_EXT_MIN_DESLIGA) {
            desligarLampExt();
            adicionarLog("LampExt OFF: 5:30");
            return;
        }
        if (t.tm_hour > LAMP_EXT_HORA_DESLIGA && t.tm_hour < 7) {
            desligarLampExt();
            adicionarLog("LampExt OFF: apos 5:30");
            return;
        }
    }
    if (!gerando && !estados.lampExt && batOK && estados.invBaixa && tempoOK) {
        bool noturno = false;
        if (t.tm_hour >= LAMP_EXT_HORA_LIGA) noturno = true;
        else if (t.tm_hour < LAMP_EXT_HORA_DESLIGA) noturno = true;
        else if (t.tm_hour == LAMP_EXT_HORA_DESLIGA &&
                 t.tm_min  <  LAMP_EXT_MIN_DESLIGA)  noturno = true;
        if (noturno) {
            ligarLampExt();
            char msgBuf[48];
            snprintf(msgBuf, sizeof(msgBuf), "LampExt ON: %02d:%02d", t.tm_hour, t.tm_min);
            adicionarLog(msgBuf);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 23 — CONTROLE GERAL DE ENERGIA
// ═══════════════════════════════════════════════════════════════════════════════════

void controlarEnergia() {
    if (modoTeste) return;
    atualizarProtecao();
    atualizarSaudeBateria();     // saude da bateria (capacidade + resistencia)
    atualizarPadroes();          // detector de padroes (base + eventos)
    atualizarPlanoEnergia();
    atualizarExcesso();          // decide ANTES da maquina do Inv.Alta
    maquinaInvBaixa();
    maquinaInvAlta();
    controlarLampExt();
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 24 — PREFERÊNCIAS E NVS FLASH
// ═══════════════════════════════════════════════════════════════════════════════════

bool existeCredenciais() {
    prefs.begin("rrfs",true);
    bool ok=prefs.isKey("user");
    prefs.end();
    return ok;
}

bool existeWiFiConfig() {
    prefs.begin("rrfs",true);
    bool ok=prefs.isKey("wifi_ok");
    prefs.end();
    return ok;
}

// Assistente de configuracao inicial (ordem: login > bms > banco > calib > wifi)
uint8_t obterSetupEtapa() {
    prefs.begin("rrfs", true);
    uint8_t e = prefs.getUChar("setup_etp", 0);
    prefs.end();
    return e;
}

void avancarSetupEtapa(uint8_t minima) {
    prefs.begin("rrfs", false);
    uint8_t atual = prefs.getUChar("setup_etp", 0);
    if (minima > atual) prefs.putUChar("setup_etp", minima);
    prefs.end();
}

String proximoPassoSetup() {
    uint8_t e = obterSetupEtapa();
    if (e < 1) return "/setup-bms";
    if (e < 2) return "/setup-banco";
    if (e < 3) return "/setup-calib";
    return "/setup-wifi";
}

void salvarCredenciais(const String& u, const String& p) {
    prefs.begin("rrfs",false);
    prefs.putString("user",u);
    prefs.putString("pass",p);
    prefs.end();
}

bool verificarLogin(const String& u, const String& p) {
    prefs.begin("rrfs",true);
    String su=prefs.getString("user",""), sp=prefs.getString("pass","");
    prefs.end();
    return (su==u && sp==p && su.length()>0);
}

void salvarWifi(const String& ssid, const String& pass,
                const String& ip,   const String& gw,
                const String& mask, const String& dns, bool ap) {
    prefs.begin("rrfs",false);
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

void carregarWifi(String& ssid, String& pass, String& ip,
                  String& gw,   String& mask,  String& dns, bool& ap) {
    prefs.begin("rrfs",true);
    ap   = prefs.getBool  ("usar_ap",   true);
    ssid = prefs.getString("wifi_ssid", "");
    pass = prefs.getString("wifi_pass", "");
    ip   = prefs.getString("wifi_ip",   "");
    gw   = prefs.getString("wifi_gw",   "");
    mask = prefs.getString("wifi_mask", "255.255.255.0");
    dns  = prefs.getString("wifi_dns",  "8.8.8.8");
    prefs.end();
}

void salvarConfigInversores() {
    prefs.begin("rrfs_inv",false);
    prefs.putBool ("inv_inva", cfgInv.invertReleInvBaixa);
    prefs.putBool ("inv_lamp", cfgInv.invertReleLampExt);
    prefs.putBool ("inv_alta", cfgInv.invertReleInvAlta);
    prefs.putUChar("ver", 27);
    prefs.end();
}

void carregarConfigInversores() {
    prefs.begin("rrfs_inv",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if (ver==27) {
        cfgInv.invertReleInvBaixa = prefs.getBool("inv_inva",false);
        cfgInv.invertReleLampExt  = prefs.getBool("inv_lamp", false);
        cfgInv.invertReleInvAlta  = prefs.getBool("inv_alta", false);
    } else if (ver==25) {
        cfgInv.invertReleInvBaixa = prefs.getBool("inv_inva",false);
        cfgInv.invertReleLampExt  = prefs.getBool("inv_lamp", false);
        cfgInv.invertReleInvAlta  = false;
    } else {
        cfgInv = {false,false,false};
    }
    prefs.end();
    adicionarLog("Config inv v32");
}

void salvarLimiaresSinal() {
    prefs.begin("rrfs_lsig",false);
    prefs.putInt("lim_ac",      limiarSinal.limiarAcMV);
    prefs.putInt("lim_ac_alta", limiarSinal.limiarAcAltaMV);
    prefs.putInt("lim_casa",    limiarSinal.limiarCasaMV);
    prefs.end();
}

void carregarLimiaresSinal() {
    prefs.begin("rrfs_lsig",true);
    limiarSinal.limiarAcMV     = prefs.getInt("lim_ac",      AC_LIMIAR_MV_DEFAULT);
    limiarSinal.limiarAcAltaMV = prefs.getInt("lim_ac_alta",  AC_LIMIAR_MV_DEFAULT);
    limiarSinal.limiarCasaMV   = prefs.getInt("lim_casa",     AC_LIMIAR_MV_DEFAULT);
    prefs.end();
}

void salvarLimiares() {
    prefs.begin("rrfs_lim",false);
    prefs.putFloat("lim_lamp", limiares.pctLampExt);
    prefs.putUChar("ver", 25);
    prefs.end();
}

void carregarLimiares() {
    prefs.begin("rrfs_lim",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if (ver==25) limiares.pctLampExt = prefs.getFloat("lim_lamp", 20.0f);
    else         limiares.pctLampExt = 20.0f;
    prefs.end();
}

void carregarCalibSensores() {
    calibPainel = {PAINEL_OFFSET_MV, PAINEL_SENS_MV_A, true};
    prefs.begin("rrfs_cal",true);
    if (prefs.isKey("pn_off"))  calibPainel.offsetMV = prefs.getInt  ("pn_off",  calibPainel.offsetMV);
    if (prefs.isKey("pn_sens")) calibPainel.sensMvA  = prefs.getFloat("pn_sens", calibPainel.sensMvA);
    prefs.end();
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 25 — GERENCIAMENTO WIFI E AP
// ═══════════════════════════════════════════════════════════════════════════════════

void iniciarAP() {
    modoAP = true;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT, "*", AP_IP);
    adicionarLog("AP RRFS 10.0.0.1");
    logRAM();
    ledBlink();
}

static void aplicarIPFixoSTA() {
    if (gWifiIP.isEmpty()) return;
    IPAddress lIP, lGW, lMask, lDNS;
    if (!lIP.fromString(gWifiIP) || !lGW.fromString(gWifiGW) ||
        !lMask.fromString(gWifiMASK)) return;
    if (!lDNS.fromString(gWifiDNS)) lDNS.fromString("8.8.8.8");
    WiFi.config(lIP, lGW, lMask, lDNS);
}

static unsigned long tRefreshAP = 0;

static bool subirSoftAP() {
    if ((WiFi.getMode() & WIFI_AP) == 0) {
        WiFi.mode(WIFI_AP_STA);
        delay(100);
    }
    WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    bool ok = WiFi.softAP(AP_SSID, nullptr, 1, 0, 4);
    if (!ok || WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
        WiFi.mode(WIFI_OFF);
        delay(200);
        WiFi.mode(WIFI_AP_STA);
        delay(100);
        WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
        ok = WiFi.softAP(AP_SSID, nullptr, 1, 0, 4);
    }
    return ok && WiFi.softAPIP() != IPAddress(0, 0, 0, 0);
}

void iniciarAPEmergencial() {
    wifiFalhasConsec  = 0;
    wifiReconectando  = false;
    tInicioRecon      = 0;
    emergTentandoWifi = false;
    tWifiPerdido      = 0;
    tUltimoRetryEmerg = millis();
    tRefreshAP        = millis();

    WiFi.setAutoReconnect(false);
    WiFi.persistent(false);
    WiFi.disconnect(false, false);
    delay(100);
    esp_task_wdt_reset();

    // reset limpo do radio (destrava pilha STA/AP presa)
    WiFi.mode(WIFI_OFF);
    delay(300);
    esp_task_wdt_reset();
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    WiFi.setSleep(true);          // BLE + WiFi exigem modem sleep ligado

    bool apOk = false;
    for (int t = 0; t < 3 && !apOk; t++) {
        apOk = subirSoftAP();
        if (!apOk) { delay(300); esp_task_wdt_reset(); }
    }

    dnsServer.stop();
    dnsServer.start(DNS_PORT, "*", AP_IP);
    modoAP = true;
    ledBlink();

    server.stop();
    delay(50);
    server.begin();

    if (apOk) adicionarLog("AP emergencial NO AR: RRFS Energia 10.0.0.1");
    else      adicionarLog("AP emergencial FALHOU, tentando de novo");
    esp_task_wdt_reset();
}

bool iniciarWiFiNB(const String& ssid, const String& pass,
                   const String& ip,   const String& gw,
                   const String& mask, const String& dns) {
    // guarda ANTES de validar: o AP emergencial precisa disso para tentar voltar
    gWifiSSID=ssid; gWifiPASS=pass; gWifiIP=ip; gWifiGW=gw; gWifiMASK=mask; gWifiDNS=dns;

    IPAddress lIP, lGW, lMask, lDNS;
    if (!lIP.fromString(ip))     return false;
    if (!lGW.fromString(gw))     return false;
    if (!lMask.fromString(mask)) return false;
    if (!lDNS.fromString(dns))   lDNS.fromString("8.8.8.8");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.config(lIP, lGW, lMask, lDNS);
    WiFi.begin(ssid.c_str(), pass.c_str());

    adicionarLog("WiFi conectando");
    ledBlink();
    unsigned long t = millis();
    while (WiFi.status()!=WL_CONNECTED && millis()-t<15000) {
        atualizarLED();
        esp_task_wdt_reset();
        delay(100);
    }
    if (WiFi.status()==WL_CONNECTED) {
        adicionarLog("WiFi OK");
        server.stop();
        delay(50);
        server.begin();
        modoAP=false;
        wifiFalhasConsec=0;
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

    if (otaAtivo) return;

    wifi_mode_t modoAtual = WiFi.getMode();

    // ── MODO AP PURO (escolhido pelo usuario) ──────────────────────────────
    if (modoAP && modoAtual == WIFI_AP) {
        if (WiFi.softAPIP() == IPAddress(0,0,0,0)) {
            adicionarLog("AP caiu: reiniciando");
            iniciarAP();
        }
        return;
    }

    // ── MODO AP EMERGENCIAL (AP+STA, tentando voltar ao roteador) ──────────
    if (modoAP) {
        if (modoAtual != WIFI_AP_STA || WiFi.softAPIP() == IPAddress(0,0,0,0)) {
            adicionarLog("AP emergencial fora do ar: subindo de novo");
            iniciarAPEmergencial();
            return;
        }

        if (WiFi.status() == WL_CONNECTED) {
            adicionarLog("WiFi: roteador voltou, encerrando AP emergencial");
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            WiFi.setSleep(true);
            WiFi.setAutoReconnect(true);
            dnsServer.stop();
            modoAP = false;
            server.stop();
            delay(50);
            server.begin();
            ledFixo(true);
            tUltimaReconWifi2 = millis();
            emergTentandoWifi = false;
            wifiFalhasConsec  = 0;
            tWifiPerdido      = 0;
            return;
        }

        if (!emergTentandoWifi) {
            if (agora - tUltimoRetryEmerg >= WIFI_EMERG_RETRY_MS) {
                tUltimoRetryEmerg = agora;
                if (gWifiSSID.isEmpty()) return;
                emergTentandoWifi     = true;
                tInicioEmergTentativa = agora;
                WiFi.setAutoReconnect(false);
                WiFi.disconnect(false, false);
                delay(50);
                aplicarIPFixoSTA();
                WiFi.begin(gWifiSSID.c_str(), gWifiPASS.c_str());
                adicionarLog("WiFi: tentando voltar ao roteador");
            } else if (WiFi.softAPgetStationNum() == 0 && agora - tRefreshAP >= 600000UL) {
                // ninguem conectado ao AP ha 10 min: refaz o radio para garantir o SSID no ar
                adicionarLog("AP: refresh preventivo do radio");
                iniciarAPEmergencial();
            }
            return;
        }

        if (agora - tInicioEmergTentativa >= WIFI_EMERG_JANELA_MS) {
            emergTentandoWifi = false;
            WiFi.disconnect(false, false);
            WiFi.setAutoReconnect(false);
            if (WiFi.softAPIP() == IPAddress(0,0,0,0)) {
                iniciarAPEmergencial();
                return;
            }
            adicionarLog("WiFi: roteador ausente, AP segue no ar");
        }
        return;
    }

    // ── MODO STA NORMAL ────────────────────────────────────────────────────
    if (WiFi.status() == WL_CONNECTED) {
        if (wifiReconectando) {
            adicionarLog("WiFi: reconectado OK");
            server.stop();
            delay(50);
            server.begin();
            ledFixo(true);
            tUltimaReconWifi2 = millis();
        }
        wifiFalhasConsec = 0;
        wifiReconectando = false;
        tInicioRecon     = 0;
        tWifiPerdido     = 0;
        return;
    }

    if (tWifiPerdido == 0) {
        tWifiPerdido = agora;
        adicionarLog("WiFi: conexao perdida");
    }

    if (agora - tWifiPerdido >= WIFI_AP_APOS_MS || gWifiSSID.isEmpty()) {
        adicionarLog("WiFi: roteador fora, subindo AP");
        iniciarAPEmergencial();
        return;
    }

    if (agora - tUltimoReconWifi >= WIFI_RECON_INTERVALO) {
        tUltimoReconWifi = agora;
        wifiFalhasConsec++;
        wifiReconectando = true;
        tInicioRecon     = agora;
        // reinicia a pilha STA de verdade (WiFi.reconnect() nao destrava)
        WiFi.disconnect(false, false);
        delay(50);
        aplicarIPFixoSTA();
        WiFi.begin(gWifiSSID.c_str(), gWifiPASS.c_str());
        snprintf(tempBuf, sizeof(tempBuf), "WiFi: reconectando (%d)", wifiFalhasConsec);
        adicionarLog(tempBuf);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 26 — FUNÇÕES AUXILIARES WEB
// ═══════════════════════════════════════════════════════════════════════════════════

String nomeEstadoBaixa(EstadoBaixa e) {
    switch(e) {
        case INV_A_DESLIGADO:    return F("DESLIGADO");
        case INV_A_LIGADO:       return F("LIGADO");
        case INV_A_DIAG_AGUARDA: return F("AGUARD AC");
        case INV_A_DIAG_OFF:     return F("DIAG OFF");
        default:                 return F("?");
    }
}

String nomeEstadoAlta(EstadoAlta e) {
    switch(e) {
        case INV_ALTA_DESLIGADO:    return F("DESLIGADO");
        case INV_ALTA_LIGADO:       return F("LIGADO");
        case INV_ALTA_DIAG_AGUARDA: return F("AGUARD AC");
        case INV_ALTA_DIAG_OFF:     return F("DIAG OFF");
        default:                    return F("?");
    }
}

void addf(String& c, const char* fmt, ...) {
    char buf[768];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    c += buf;
}

void htmlEsc(const char* s, String& out) {
    for (; *s; s++) {
        char ch = *s;
        if      (ch == '<') out += "&lt;";
        else if (ch == '>') out += "&gt;";
        else if (ch == '&') out += "&amp;";
        else if ((unsigned char)ch < 32) out += ' ';
        else out += ch;
    }
}

void verificarTimeoutTeste() {
    if (!modoTeste) return;
    if (millis() - tInicioTeste >= TIMEOUT_TESTE_MS) {
        modoTeste        = false;
        TIMEOUT_TESTE_MS = 30000UL;
        maqBaixa.estado        = INV_A_DESLIGADO;
        maqBaixa.tInicioEstado = millis();
        maqAlta.estado         = INV_ALTA_DESLIGADO;
        maqAlta.tInicioEstado  = millis();
        adicionarLog("Modo teste: timeout");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 27 — SETUP
// ═══════════════════════════════════════════════════════════════════════════════════

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
        errNVS == ESP_ERR_NVS_NEW_VERSION_FOUND)
        adicionarLog("NVS aviso");

    switch (esp_reset_reason()) {
        case ESP_RST_BROWNOUT: adicionarLog("RESET BROWNOUT"); break;
        case ESP_RST_PANIC:    adicionarLog("RESET PANIC");    break;
        case ESP_RST_TASK_WDT: adicionarLog("RESET WDT");     break;
        case ESP_RST_SW:       adicionarLog("Reset SW");       break;
        case ESP_RST_POWERON:  adicionarLog("Power ON");       break;
        default:
            snprintf(tempBuf,sizeof(tempBuf),"Reset %d",(int)esp_reset_reason());
            adicionarLog(tempBuf);
            break;
    }

    esp_task_wdt_reset();
    setupLED();
    setupBotao();
    carregarConfigInversores();
    setupReles();

    pinMode(PIN_SENSOR_PAINEL,  INPUT);
    pinMode(PIN_AC_SINAL_BAIXA, INPUT);
    pinMode(PIN_AC_SINAL_ALTA,  INPUT);
    pinMode(PIN_SINAL_CASA,     INPUT);

    ledBlink();
    logRAM();
    esp_task_wdt_reset();

    carregarCalibSensores();
    carregarLimiaresSinal();
    carregarLimiares();
    carregarHistorico();
    carregarContadores();
    carregarConfigTempo();
    carregarConfigPrevisao();
    carregarEstatPrev();
    carregarConfigBMS();
    carregarConfigBanco();
    carregarConfigProtecao();
    carregarSaudeBateria();
    carregarPlanoEnergia();
    padFsIniciar();               // v32: LittleFS dos padroes
    padCarregarArquivo();         // v32: 30 dias de padroes -> RAM
    esp_task_wdt_reset();

    ultimoContagem = millis();
    tInicioSistema      = millis();
    sistemaEstabilizado = false;
    tUltimaReconWifi2   = 0;

    // ── WIFI: conecta normal OU sobe AP (configurado OU emergencial) ──────
    bool usarAP = true;
    if (existeWiFiConfig()) {
        String ssid,pass,ip,gw,mask,dns;
        carregarWifi(ssid,pass,ip,gw,mask,dns,usarAP);
        esp_task_wdt_reset();
        if (!usarAP) {
            if (ssid.length() > 0 && iniciarWiFiNB(ssid,pass,ip,gw,mask,dns)) {
                // conectado
            } else {
                adicionarLog("WiFi boot falhou: subindo AP emergencial");
                iniciarAPEmergencial();   // sempre sobe o SSID "RRFS Energia"
            }
        }
    }
    esp_task_wdt_reset();
    if (usarAP) iniciarAP();

    if (!modoAP && cfgTempo.ntpAtivo) {
        esp_task_wdt_reset();
        sincronizarNTP();
    }

    if (cfgBMS.configurada) adicionarLog("BMS: aguardando conexao BLE no loop");

    esp_task_wdt_reset();
    configurarRotas();
    adicionarLog("Sistema pronto v32 (padroes)");
    logRAM();
    esp_task_wdt_reset();
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 28 — LOOP
// ═══════════════════════════════════════════════════════════════════════════════════

void loop() {
    esp_task_wdt_reset();
    verificarReset();
    atualizarLED();

    if (otaPrepSolicitado) otaPrepararBLE();

    if (otaAtivo) {
        if (modoAP || WiFi.getMode()==WIFI_AP_STA)
            dnsServer.processNextRequest();
        server.handleClient();
        esp_task_wdt_reset();

        if (otaErro && !otaOk) {
            adicionarLog("OTA falhou: reiniciando");
            salvarLogsPendentes(true);
            delay(1500);
            ESP.restart();
        }
        unsigned long limite = otaUploadIniciado ? 20000UL : 60000UL;
        if (!otaOk && millis() - tOtaUltimo > limite) {
            adicionarLog("OTA: timeout, reiniciando");
            salvarLogsPendentes(true);
            Update.abort();
            delay(500);
            ESP.restart();
        }
        return;
    }

    verificarTimeoutTeste();

    if (modoAP || WiFi.getMode()==WIFI_AP_STA)
        dnsServer.processNextRequest();
    server.handleClient();
    esp_task_wdt_reset();

    unsigned long agora = millis();

    // ── Leitura sensores (500ms) ───────────────────────────────────────────
    if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
        ultimaLeitura = agora;

        esp_task_wdt_reset();
        lerSensores();
        server.handleClient();
        esp_task_wdt_reset();

        processarSinaisAnalogicos();
        server.handleClient();
        esp_task_wdt_reset();

        controlarEnergia();
        acumularEnergia();
        server.handleClient();
        esp_task_wdt_reset();
    }

    // ── Controle extra (1s) ────────────────────────────────────────────────
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;
        controlarLampExt();
        esp_task_wdt_reset();
        server.handleClient();
    }

    // ── BLE BMS ────────────────────────────────────────────────────────────
    gerenciarBLE();
    esp_task_wdt_reset();
    server.handleClient();

    // ── NTP (1 hora) ───────────────────────────────────────────────────────
    if (cfgTempo.ntpAtivo && !modoAP &&
        WiFi.status()==WL_CONNECTED &&
        agora - tUltimoNTP >= INTERVALO_NTP_MS) {
        sincronizarNTP();
        esp_task_wdt_reset();
    }

    // ── Previsão solar ─────────────────────────────────────────────────────
    if (!modoAP && WiFi.status()==WL_CONNECTED) {
        gerenciarPrevisao();
        esp_task_wdt_reset();
    }

    // ── WiFi ───────────────────────────────────────────────────────────────
    gerenciarWiFi();
    esp_task_wdt_reset();

    // ── Salvar logs (10s) ──────────────────────────────────────────────────
    salvarLogsPendentes();

    // ── Log RAM (10 min) ───────────────────────────────────────────────────
    {
        static unsigned long tRAM = 0;
        if (agora - tRAM >= 600000UL) {
            tRAM = agora;
            logRAM();
        }
    }

    esp_task_wdt_reset();
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 29 — CSS RESPONSIVO
// ═══════════════════════════════════════════════════════════════════════════════════

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
.btn-b{background:#4a148c}.btn-b:hover{background:#6a1b9a}
.btn-sm{font-size:12px;padding:8px}
.ok{background:#1b5e20;color:#a5d6a7;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.er{background:#b71c1c;color:#ef9a9a;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.wn{background:#e65100;color:#ffe0b2;padding:10px;border-radius:4px;margin:8px 0;text-align:center}
.nf{background:#0d47a1;color:#90caf9;padding:10px;border-radius:4px;margin:8px 0;font-size:12px;line-height:1.5}
.al{background:#4a0000;border:2px solid #c62828;color:#ef9a9a;padding:10px;border-radius:6px;margin:6px 0;font-size:12px}
.nav{display:flex;gap:4px;margin-top:12px;padding-top:10px;border-top:1px solid #333;flex-wrap:wrap}
.nav a{flex:1;min-width:36px;text-align:center;padding:9px 4px;background:#1e1e1e;color:#90a4ae;text-decoration:none;border-radius:4px;font-size:11px;border:1px solid #333}
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
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.grid4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px}
.pd{background:#0a1a0a;border:1px solid #1b5e20;border-radius:6px;padding:10px;margin:4px 0}
.pd .dt{font-size:11px;color:#43a047;font-weight:bold;margin-bottom:4px}
.pd .dv{font-size:18px;font-weight:bold;color:#a5d6a7}
.pd .ds{font-size:10px;color:#546e7a;margin-top:2px}
.sig-box{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:5px 0}
.sig-lbl{font-size:11px;color:#90a4ae;font-weight:bold;margin-bottom:4px}
.sig-val{font-size:18px;font-weight:bold;margin:3px 0}
.sig-sub{font-size:10px;color:#546e7a}
.bms-card{background:#0a0a1a;border:2px solid #4a148c;border-radius:10px;padding:12px;margin:6px 0}
.bms-title{font-size:11px;color:#ce93d8;font-weight:bold;margin-bottom:8px}
.bms-val{font-size:20px;font-weight:bold;color:#e1bee7}
.bms-sub{font-size:10px;color:#7b1fa2;margin-top:3px}
.banco-card{background:#0a1a2a;border:2px solid #01579b;border-radius:10px;padding:12px;margin:6px 0}
.banco-title{font-size:11px;color:#81d4fa;font-weight:bold;margin-bottom:8px}
.banco-badge{display:inline-block;background:#01579b;color:#e1f5fe;font-size:16px;font-weight:bold;padding:6px 14px;border-radius:6px;letter-spacing:2px}
.pulse{animation:pulse 1s infinite}
@keyframes pulse{0%{opacity:1}50%{opacity:.5}100%{opacity:1}}
table.tb{width:100%;border-collapse:collapse;font-size:11px}
table.tb th{color:#546e7a;padding:4px;text-align:center}
table.tb td{padding:4px;text-align:center;border-top:1px solid #222}
a.dia{display:block;text-decoration:none;color:#e0e0e0}
)RAW";

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 30 — HELPERS HTML
// ═══════════════════════════════════════════════════════════════════════════════════

static const char NAV_LOGS_JS[] PROGMEM =
    "<div class='lb'>📋 Logs</div>"
    "<div class='lbx' id='lb'><div style='color:#666'>...</div></div>"
    "</div>"
    "<script>"
    "function aL(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{var d=document.getElementById('lb');"
    "var a=JSON.parse(x.responseText);var h='';"
    "for(var i=0;i<a.length;i++)h+='<div class=\"ll\">'+a[i]+'</div>';"
    "d.innerHTML=h||'<div style=\"color:#666\">Sem logs</div>';"
    "d.scrollTop=d.scrollHeight;}catch(e){}}};"
    "x.open('GET','/logs',true);x.send();}"
    "setInterval(aL,4000);aL();"
    "</script></body></html>";

static String navHtml() {
    String pag = server.uri();
    String n = F("<div class='nav'>");
    static const char* const lk[] = {"/","/bateria","/controle","/sensores","/previsao","/historico","/config"};
    static const char* const ic[] = {"📊","🔋","⚙️","🔍","☀️","📈","⚙️"};
    for (int i = 0; i < 7; i++) {
        n += "<a href='"; n += lk[i]; n += "' ";
        if (pag == lk[i]) n += "class='active'";
        n += ">"; n += ic[i]; n += "</a>";
    }
    n += F("</div>");
    return n;
}

String barraProgresso(float pct, const String& label) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    String cor = pct >= 50 ? "#2e7d32" : pct >= 25 ? "#ef6c00" : "#c62828";
    char buf[200];
    snprintf(buf, sizeof(buf),
        "<div class='bb'><div class='bv' style='width:%d%%;background:%s'>%d%%</div></div>",
        (int)pct, cor.c_str(), (int)pct);
    String h = String(buf);
    if (label.length())
        h += "<div style='font-size:11px;color:#546e7a;text-align:center'>" + label + "</div>";
    return h;
}

String gerarPagina(const String& titulo, const String& conteudo, bool comNav) {
    String html;
    html.reserve(conteudo.length() + 3000);
    html  = F("<!DOCTYPE html><html><head>"
              "<meta charset='utf-8'>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += "<title>" + titulo + "</title>";
    html += F("<style>");
    html += CSS;
    html += F("</style></head><body><div class='card'>");
    html += F("<h2>⚡ RRFS Energia v32</h2>");
    html += conteudo;
    if (comNav && logado) html += navHtml();
    html += FPSTR(NAV_LOGS_JS);
    return html;
}

void paginaInicio(const char* titulo) {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html;charset=utf-8", "");
    server.sendContent(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'><title>"));
    server.sendContent(String(titulo));
    server.sendContent(F("</title><style>"));
    server.sendContent_P(CSS);
    server.sendContent(F("</style></head><body><div class='card'>"
                         "<h2>⚡ RRFS Energia v32</h2>"));
}

void paginaFim(bool comNav = true) {
    if (comNav && logado) server.sendContent(navHtml());
    server.sendContent_P(NAV_LOGS_JS);
    server.sendContent("");
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 31 — API /dados (JSON)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleDados() {
    if (!logado) { server.send(403); return; }

    unsigned long tempoBaixa = (millis() - maqBaixa.tInicioEstado) / 1000UL;
    unsigned long tempoAlta  = (millis() - maqAlta.tInicioEstado)  / 1000UL;
    EstimativaTempo tempo   = calcularTempoRestante();
    float ahH = 0, ahR = 0;
    if (cfgPrevisao.ativa && previsao.valido) {
        ahH = getAhPrevHoraAtual();
        ahR = getAhPrevRestanteDia();
    }
    unsigned long tBMSSeg = dadosBMS.tUltimaDado > 0
        ? (millis() - dadosBMS.tUltimaDado) / 1000UL : 9999UL;

    char buf[2200];
    snprintf(buf, sizeof(buf),
             "{"
             "\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
             "\"wp\":%.0f,\"wc\":%.0f,\"wb\":%.0f,"
             "\"v_banco\":%.2f,\"soc\":%.1f,\"ciclos\":%d,"
             "\"cap_banco\":%.2f,"
             "\"nS\":%d,\"nP\":%d,"
             "\"inva\":%s,\"le\":%s,"
             "\"est_baixa\":%d,\"nome_baixa\":\"%s\","
             "\"tempo_baixa\":%lu,\"lig_baixa\":%d,\"fal_baixa\":%d,"
             "\"ac_baixa_mv\":%.0f,\"ac_baixa_ok\":%s,"
             "\"inv_alta\":%s,"
             "\"est_alta\":%d,\"nome_alta\":\"%s\","
             "\"tempo_alta\":%lu,\"lig_alta\":%d,\"fal_alta\":%d,"
             "\"ac_alta_mv\":%.0f,\"ac_alta_ok\":%s,"
             "\"casa_mv\":%.0f,\"casa_pede\":%s,"
             "\"lim_lamp\":%.0f,"
             "\"lim_ac_baixa\":%d,\"lim_ac_alta\":%d,\"lim_casa\":%d,"
             "\"tempo_val\":%s,\"tempo_h\":%d,\"tempo_m\":%d,\"tempo_carg\":%s,"
             "\"mv_painel\":%d,"
             "\"prev_ok\":%s,\"ah_prev_h\":%.2f,\"ah_prev_r\":%.2f,"
             "\"ntp\":%s,\"modo_teste\":%s,"
             "\"ram\":%.1f,\"uptime\":%lu,"
             "\"data_hora\":\"%s\","
             "\"bms_corr\":%.2f,\"bms_tensao\":%.2f,"
             "\"bms_soc\":%.1f,\"bms_ciclos\":%d,\"bms_cap\":%.2f,"
             "\"bms_valido\":%s,\"bms_t_seg\":%lu,"
             "\"ble_estado\":%d,\"ble_str\":\"%s\","
             "\"bms_cfg\":%s,\"bms_mac\":\"%s\","
             "\"ble_falhas\":%d,"
             "\"reserva\":%.0f,\"prot\":%s,"
             "\"ah_disp\":%.1f,\"ah_falta\":%.1f,\"na_res\":%s,"
             "\"ctrl_a\":%.0f"
             "}",
             sensores.correntePainel, sensores.correnteCasa, sensores.correnteBateria,
             sensores.potenciaPainel, sensores.potenciaCasa, fabsf(sensores.potenciaBateria),
             sensores.tensaoBanco, sensores.socBMS, (int)sensores.ciclosBMS,
             sensores.capacidadeBanco,
             (int)banco.nS, (int)banco.nP,
             estados.invBaixa  ? "true" : "false",
             estados.lampExt   ? "true" : "false",
             (int)maqBaixa.estado, nomeEstadoBaixa(maqBaixa.estado).c_str(),
             tempoBaixa, (int)maqBaixa.totalLigadas, (int)maqBaixa.totalFalhas,
             acBaixa.mediaMV, acBaixa.presente ? "true" : "false",
             estados.invAlta   ? "true" : "false",
             (int)maqAlta.estado, nomeEstadoAlta(maqAlta.estado).c_str(),
             tempoAlta, (int)maqAlta.totalLigadas, (int)maqAlta.totalFalhas,
             acAlta.mediaMV, acAlta.presente ? "true" : "false",
             sinalCasa.mediaMV, sinalCasa.presente ? "true" : "false",
             limiares.pctLampExt,
             limiarSinal.limiarAcMV, limiarSinal.limiarAcAltaMV, limiarSinal.limiarCasaMV,
             tempo.valido     ? "true" : "false",
             tempo.horas, tempo.minutos,
             tempo.carregando ? "true" : "false",
             sensores.sinalPainel_mV,
             previsao.valido  ? "true" : "false", ahH, ahR,
             ntpSincronizado  ? "true" : "false",
             modoTeste        ? "true" : "false",
             esp_get_free_heap_size() / 1024.0f, millis() / 1000UL,
             formatarDataHora().c_str(),
             dadosBMS.corrente, dadosBMS.tensao,
             dadosBMS.soc, (int)dadosBMS.ciclos, dadosBMS.capacidadeAh,
             dadosBMS.valido  ? "true" : "false", tBMSSeg,
             (int)ctrlBLE.estado, statusBLEStr().c_str(),
             cfgBMS.configurada ? "true" : "false", cfgBMS.macAddress,
             (int)ctrlBLE.falhasConsec,
             protecao.reservaPct, protecao.ativa ? "true" : "false",
             tempo.ahDisp, tempo.ahFalta, tempo.naReserva ? "true" : "false",
             cfgPrevisao.controladorA
    );
    server.send(200, "application/json", buf);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 32 — AUTENTICAÇÃO E SETUP
// ═══════════════════════════════════════════════════════════════════════════════════

void handleSetupLogin() {
    if (existeCredenciais() && existeWiFiConfig())
    { server.sendHeader("Location","/login"); server.send(302); return; }
    if (existeCredenciais() && !existeWiFiConfig())
    { server.sendHeader("Location", proximoPassoSetup()); server.send(302); return; }

    String msg = "";
    if (server.method() == HTTP_POST) {
        String u=server.arg("user"), p=server.arg("pass"), cf=server.arg("conf");
        if (u.length()<3)      msg = "<div class='er'>Min 3 chars usuario</div>";
        else if (p.length()<4) msg = "<div class='er'>Min 4 chars senha</div>";
        else if (p!=cf)        msg = "<div class='er'>Senhas nao conferem</div>";
        else {
            salvarCredenciais(u,p);
            logado = true;
            adicionarLog("Conta criada, iniciando assistente");
            server.sendHeader("Location","/setup-bms");
            server.send(302);
            return;
        }
    }
    String c = F("<div class='nf'>ℹ Primeiro acesso.</div><h3>🔐 Criar Conta</h3>");
    c += msg;
    c += F("<form method='POST' action='/setup-login'>"
    "<label>Usuario:</label><input type='text' name='user' maxlength='32' required autocomplete='off'>"
    "<label>Senha:</label><input type='password' name='pass' maxlength='32' required>"
    "<label>Confirmar:</label><input type='password' name='conf' maxlength='32' required>"
    "<button type='submit'>Criar →</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS Energia - Setup",c,false));
}

void handleSetupWifi() {
    if (!existeCredenciais())
    { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (!existeWiFiConfig()) {
        if (obterSetupEtapa() < 3)
        { server.sendHeader("Location", proximoPassoSetup()); server.send(302); return; }
    } else if (!logado) {
        server.sendHeader("Location","/login"); server.send(302); return;
    }
    String msg = "";
    if (server.method() == HTTP_POST) {
        String modo = server.arg("modo");
        if (modo == "ap") {
            salvarWifi("","","","","","",true);
            server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS Energia - Salvo",F("<div class='ok'>✓ Reiniciando...</div>"
                "<div class='nf'>SSID: <b>RRFS Energia</b> | IP: <b>10.0.0.1</b></div>"),false));
            delay(2000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"), wp=server.arg("wpass"),
               ip=server.arg("ip"),   gw=server.arg("gw"),
               mask=server.arg("mask"), dns=server.arg("dns");
        if (ssid.isEmpty())     msg="<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7) msg="<div class='er'>Informe o IP</div>";
        else if (gw.length()<7) msg="<div class='er'>Informe o Gateway</div>";
        else {
            salvarWifi(ssid,wp,ip,gw,mask,dns,false);
            server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS Energia - Salvo","<div class='ok'>✓ Acesse: http://"+ip+
                "</div><div class='nf'>Reiniciando...</div>",false));
            delay(2000); ESP.restart(); return;
        }
    }
    static int16_t nRedes=-1; static bool scanIniciado=false;
    if (!scanIniciado) {
        if (WiFi.getMode()==WIFI_AP) { WiFi.mode(WIFI_AP_STA); delay(100); }
        if (WiFi.status()!=WL_CONNECTED) { WiFi.setAutoReconnect(false); WiFi.disconnect(); delay(100); }
        esp_task_wdt_reset();
        WiFi.scanDelete(); WiFi.scanNetworks(true,false,false,300);
        scanIniciado=true; nRedes=-1; adicionarLog("Scan WiFi iniciado");
    }
    nRedes = WiFi.scanComplete();
    String c = F("<div class='nf'>ℹ IP fixo obrigatorio.</div><h3>📶 Rede</h3>");
    c += msg;
    c += F("<form method='POST' action='/setup-wifi'>"
           "<label>Modo:</label>"
           "<select name='modo' id='modo' onchange='toggleModo()'>"
           "<option value='wifi'>Conectar ao WiFi</option>"
           "<option value='ap'>Ponto de Acesso</option></select>"
           "<div id='dw'><label>Rede:</label><select name='ssid'>");
    bool escaneando=false;
    if (nRedes==WIFI_SCAN_RUNNING) { c+=F("<option>Escaneando...</option>"); escaneando=true; }
    else if (nRedes==WIFI_SCAN_FAILED||nRedes==0) {
        c+=F("<option value=''>Nenhuma rede</option>");
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
    } else if (nRedes>0) {
        for (int i=0;i<nRedes;i++) {
            String q=WiFi.RSSI(i)>-60?"Bom":WiFi.RSSI(i)>-75?"Regular":"Fraco";
            char b2[32]; snprintf(b2,sizeof(b2)," [%s %ddBm]",q.c_str(),WiFi.RSSI(i));
            c+="<option value='"+WiFi.SSID(i)+"'>"+WiFi.SSID(i)+String(b2)+"</option>";
        }
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
    }
    c+=F("</select>"
         "<button type='button' class='btn-o btn-sm' onclick='location.reload()' style='margin-top:8px'>🔄 Atualizar</button>"
         "<label>Senha WiFi:</label><input type='password' name='wpass' placeholder='Vazio se aberta'>"
         "<label>IP Fixo:</label><input type='text' name='ip' required placeholder='192.168.1.100'>"
         "<label>Gateway:</label><input type='text' name='gw' required placeholder='192.168.1.1'>"
         "<label>Mascara:</label><input type='text' name='mask' required value='255.255.255.0'>"
         "<label>DNS:</label><input type='text' name='dns' required value='8.8.8.8'></div>"
         "<button type='submit' class='btn-g'>💾 Salvar</button></form>");
    if (logado) c+=F("<button onclick='location.href=\"/\"' class='btn-s' style='margin-top:8px'>← Voltar</button>");
    c+=F("<script>function toggleModo(){"
         "document.getElementById('dw').style.display="
         "document.getElementById('modo').value=='wifi'?'block':'none';}</script>");
    if (escaneando) c+=F("<script>setTimeout(function(){location.reload();},3000);</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS Energia - Rede",c,false));
}

void handleLogin() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (logado) { server.sendHeader("Location","/"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        if (verificarLogin(server.arg("user"),server.arg("pass"))) {
            logado=true; adicionarLog("Login OK"); logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        }
        adicionarLog("Login FALHOU");
        msg=F("<div class='er'>✗ Usuario/senha incorretos</div>");
    }
    String c=F("<div class='nf'>🔐 Acesso seguro</div><h3>Entrar</h3>");
    c+=msg;
    c+=F("<form method='POST' action='/login'>"
         "<label>Usuario:</label><input type='text' name='user' required autocomplete='off'>"
         "<label>Senha:</label><input type='password' name='pass' required>"
         "<button type='submit'>🔓 Entrar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS Energia - Login",c,false));
}

void handleLogout() {
    if (logado) { adicionarLog("Logout"); logado=false; }
    server.sendHeader("Location","/login"); server.send(302);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 33 — ASSISTENTE DE CONFIGURAÇÃO INICIAL
// Ordem: Conta -> BMS -> Banco -> Calibração do Painel -> Rede -> Reinicia
// ═══════════════════════════════════════════════════════════════════════════════════

String wizardPasso(int atual, const char* titulo) {
    char buf[80];
    snprintf(buf, sizeof(buf), "<div class='nf' style='text-align:center'>Passo %d de 4 — %s</div>",
             atual, titulo);
    return String(buf);
}

// Script comum de busca de BMS (usado no assistente e na página /bms)
static void enviarScriptScanBMS(const char* acaoUrl) {
    String c = F("<script>var _poll=null;var _url='");
    c += acaoUrl;
    c += F("';"
 "function sel(mac){"
 "var f=document.createElement('form');f.method='POST';f.action=_url;"
 "var a=document.createElement('input');a.type='hidden';a.name='acao';a.value='salvar_mac';f.appendChild(a);"
 "var m=document.createElement('input');m.type='hidden';m.name='mac';m.value=mac;f.appendChild(m);"
 "document.body.appendChild(f);f.submit();}"
 "function iniciarScan(){"
 "document.getElementById('btnScan').disabled=true;"
 "document.getElementById('scan_st').textContent='Buscando... (ate 10s)';"
 "document.getElementById('scan_lista').innerHTML='';"
 "var x=new XMLHttpRequest();x.open('GET','/bms-scan',true);x.send();"
 "if(_poll)clearInterval(_poll);_poll=setInterval(lerLista,1500);}"
 "function lerLista(){var x=new XMLHttpRequest();"
 "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
 "try{var d=JSON.parse(x.responseText);rl(d);"
 "if(!d.scan){clearInterval(_poll);_poll=null;"
 "document.getElementById('btnScan').disabled=false;"
 "document.getElementById('scan_st').textContent=d.itens.length?"
 "'Concluido: '+d.itens.length+' dispositivo(s). ★ = provavel BMS':"
 "'Nenhum dispositivo. Feche o app da BMS e tente de novo.';}"
 "}catch(e){}}};x.open('GET','/bms-lista',true);x.send();}"
 "function rl(d){var el=document.getElementById('scan_lista');el.innerHTML='';"
 "var it=d.itens.slice();it.sort(function(a,b){return (b.jbd-a.jbd)||(b.rssi-a.rssi);});"
 "for(var i=0;i<it.length;i++){(function(o){"
 "var row=document.createElement('div');row.className='cg';"
 "row.style.borderColor=o.jbd?'#7b1fa2':'#222';"
 "var l=document.createElement('div');l.className='cr';"
 "var t=document.createElement('span');t.className='cn';"
 "t.textContent=(o.jbd?'★ ':'')+o.nome+' | '+o.mac+' | '+o.rssi+'dBm';"
 "var b=document.createElement('button');b.textContent='Usar';"
 "b.className='btn-g btn-sm';b.style.width='auto';b.style.marginTop='0';"
 "b.onclick=function(){sel(o.mac);};"
 "l.appendChild(t);l.appendChild(b);row.appendChild(l);el.appendChild(row);})(it[i]);}}"
 "</script>");
    server.sendContent(c);
}

static bool macValido(String& mac) {
    mac.trim(); mac.toUpperCase();
    if (mac.length() != 17) return false;
    for (int i = 0; i < 17; i++) {
        char ch = mac[i];
        if (i % 3 == 2) { if (ch != ':') return false; }
        else if (!isxdigit(ch)) return false;
    }
    return true;
}

static void aplicarMacBMS(const String& mac) {
    strncpy(cfgBMS.macAddress, mac.c_str(), sizeof(cfgBMS.macAddress) - 1);
    cfgBMS.macAddress[sizeof(cfgBMS.macAddress) - 1] = '\0';
    cfgBMS.configurada = true;
    salvarConfigBMS();
    desconectarBMS();
    bleManualOff = false;
    ctrlBLE.tUltimoTentativa = 0;
    ctrlBLE.falhasConsec     = 0;
    dadosBMS.valido          = false;
}

// ── Passo 1: BMS Bluetooth ───────────────────────────────────────────────────────
void handleSetupBMS() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (existeWiFiConfig()) {
        server.sendHeader("Location", logado ? "/bms" : "/login"); server.send(302); return;
    }

    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "salvar_mac") {
            String mac = server.arg("mac");
            if (!macValido(mac)) {
                msg = "<div class='er'>✗ MAC invalido. Formato: AA:BB:CC:DD:EE:FF</div>";
            } else {
                aplicarMacBMS(mac);
                adicionarLog("Setup: BMS selecionada");
                avancarSetupEtapa(1);
                server.sendHeader("Location","/setup-banco");
                server.send(302);
                return;
            }
        } else if (acao == "pular") {
            adicionarLog("Setup: BMS pulada");
            avancarSetupEtapa(1);
            server.sendHeader("Location","/setup-banco");
            server.send(302);
            return;
        }
    }

    paginaInicio("RRFS Energia - Setup BMS");
    String c = wizardPasso(1, "Conectar a Bateria (BMS)");
    c += msg;
    c += F("<h3>📡 Conectar a BMS Bluetooth</h3>"
           "<div class='nf' style='font-size:11px'>"
           "1) Ligue a bateria e feche o app LanmaoBMS no celular (ele trava a conexao).<br>"
           "2) Toque em <b>Buscar BMS</b> e escolha a sua na lista.<br>"
           "3) Se preferir, informe o MAC manualmente abaixo.</div>"
           "<button type='button' class='btn-b' id='btnScan' onclick='iniciarScan()'>🔍 Buscar BMS</button>"
           "<div id='scan_st' style='text-align:center;font-size:11px;color:#78909c;margin:6px 0'></div>"
           "<div id='scan_lista'></div>"
           "<details style='margin-top:8px'><summary style='font-size:12px;color:#78909c'>"
           "Informar MAC manualmente</summary>"
           "<form method='POST' action='/setup-bms'>"
           "<input type='hidden' name='acao' value='salvar_mac'>"
           "<label>MAC (AA:BB:CC:DD:EE:FF):</label>"
           "<input type='text' name='mac' maxlength='17' placeholder='AA:BB:CC:DD:EE:FF' "
           "style='font-family:monospace;text-transform:uppercase' required>"
           "<button type='submit' class='btn-b btn-sm'>💾 Salvar e continuar</button></form>"
           "</details>"
           "<form method='POST' action='/setup-bms' style='margin-top:14px'>"
           "<input type='hidden' name='acao' value='pular'>"
           "<button type='submit' class='btn-s btn-sm'>Pular por enquanto →</button></form>"
           "<div class='nf' style='font-size:10px'>Voce podera configurar a BMS depois, "
           "na pagina Config → BMS Bluetooth.</div>");
    server.sendContent(c);
    enviarScriptScanBMS("/setup-bms");
    paginaFim(false);
}

// ── Campos do banco (usado no assistente e na página Bateria) ───────────────────
static void campoBanco(String& c, const char* acaoUrl) {
    c += F("<form method='POST' action='");
    c += acaoUrl;
    c += F("'><input type='hidden' name='acao' value='config_banco'>"
           "<label>Modo de Configuracao:</label>"
           "<select name='modo' id='modo_select' onchange='mostrarCampos()'>");
    static const char* const nomesModo[] = {
        "", "1 - Unico (1 bateria)", "2 - Paralelo (nP)",
        "3 - Serie (nS)", "4 - Serie + Paralelo (nS x nP)"
    };
    for (int m = 1; m <= 4; m++)
        addf(c, "<option value='%d'%s>%s</option>", m, banco.modo == m ? " selected" : "", nomesModo[m]);
    c += F("</select>");

    addf(c, "<div id='modo1' class='nf' style='margin-top:12px;display:%s'>"
            "Sistema usa <b>1 bateria</b>. Valores direto da BMS.</div>",
            banco.modo == 1 ? "block" : "none");

    addf(c, "<div id='modo2' style='margin-top:12px;display:%s'>", banco.modo == 2 ? "block" : "none");
    c += F("<div class='nf' style='font-size:11px'>PARALELO: tensao constante | corrente x nP | capacidade x nP</div>"
           "<label>Baterias em paralelo (nP):</label>");
    addf(c, "<input type='number' name='nP_paralelo' min='1' max='8' value='%d'></div>", banco.nP);

    addf(c, "<div id='modo3' style='margin-top:12px;display:%s'>", banco.modo == 3 ? "block" : "none");
    c += F("<div class='nf' style='font-size:11px'>SERIE: tensao x nS | corrente constante | capacidade constante</div>"
           "<label>Baterias em serie (nS):</label>");
    addf(c, "<input type='number' name='nS_serie' min='1' max='8' value='%d'></div>", banco.nS);

    addf(c, "<div id='modo4' style='margin-top:12px;display:%s'>", banco.modo == 4 ? "block" : "none");
    c += F("<div class='nf' style='font-size:11px'>SERIE+PARALELO: tensao x nS | corrente x nP | capacidade x nP</div>"
           "<label>Baterias em serie (nS):</label>");
    addf(c, "<input type='number' name='nS_completo' min='1' max='8' value='%d'>", banco.nS);
    c += F("<label>Grupos em paralelo (nP):</label>");
    addf(c, "<input type='number' name='nP_completo' min='1' max='8' value='%d'></div>", banco.nP);
}

static const char JS_MOSTRAR_CAMPOS[] PROGMEM =
    "<script>function mostrarCampos(){var m=document.getElementById('modo_select').value;"
    "for(var i=1;i<=4;i++)document.getElementById('modo'+i).style.display=(m==String(i))?'block':'none';}"
    "</script>";

// Lê os campos do formulário do banco. Retorna false se modo invalido.
static bool aplicarFormBanco() {
    int modo = server.arg("modo").toInt();
    if (modo < 1 || modo > 4) return false;
    int nS = 1, nP = 1;
    if (modo == 2) nP = server.arg("nP_paralelo").toInt();
    else if (modo == 3) nS = server.arg("nS_serie").toInt();
    else if (modo == 4) { nS = server.arg("nS_completo").toInt(); nP = server.arg("nP_completo").toInt(); }
    if (nS < 1 || nS > 8) nS = 1;
    if (nP < 1 || nP > 8) nP = 1;
    banco.modo = modo; banco.nS = nS; banco.nP = nP; banco.configurado = true;
    salvarConfigBanco();
    calcularDadosBanco();
    return true;
}

// ── Passo 2: Banco de Baterias ───────────────────────────────────────────────────
void handleSetupBanco() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (existeWiFiConfig()) {
        server.sendHeader("Location", logado ? "/bateria" : "/login"); server.send(302); return;
    }
    if (obterSetupEtapa() < 1) { server.sendHeader("Location","/setup-bms"); server.send(302); return; }

    String msg = "";
    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config_banco") {
            if (!aplicarFormBanco()) msg = "<div class='er'>Modo invalido (1-4)</div>";
            else {
                adicionarLog("Setup: banco configurado");
                avancarSetupEtapa(2);
                server.sendHeader("Location","/setup-calib");
                server.send(302);
                return;
            }
        } else if (acao == "pular") {
            adicionarLog("Setup: banco pulado");
            avancarSetupEtapa(2);
            server.sendHeader("Location","/setup-calib");
            server.send(302);
            return;
        }
    }

    paginaInicio("RRFS Energia - Setup Banco");
    String c = wizardPasso(2, "Configurar o Banco de Baterias");
    c += msg;
    c += F("<div class='nf' style='font-size:11px'>Diga como as baterias estao ligadas entre si. "
           "Isso ajusta tensao, corrente e capacidade total calculadas a partir da leitura da BMS.</div>");

    if (bmsFresca())
        addf(c, "<div class='sc' style='text-align:center'><div class='lbl'>BMS conectada</div>"
                "<div class='val' style='color:#43a047'>%.2fV | SOC %.0f%%</div></div>",
                dadosBMS.tensao, dadosBMS.soc);
    else
        c += F("<div class='wn' style='font-size:11px'>BMS ainda sem dados (ok, pode configurar mesmo assim).</div>");

    campoBanco(c, "/setup-banco");
    c += F("<button type='submit' class='btn-g' style='margin-top:16px'>💾 Salvar e continuar →</button></form>"
           "<form method='POST' action='/setup-banco' style='margin-top:14px'>"
           "<input type='hidden' name='acao' value='pular'>"
           "<button type='submit' class='btn-s btn-sm'>Pular por enquanto →</button></form>"
           "<div class='nf' style='font-size:10px'>Voce podera configurar o banco depois, "
           "na pagina Bateria.</div>");
    c += FPSTR(JS_MOSTRAR_CAMPOS);
    server.sendContent(c);
    paginaFim(false);
}

// Calibra o offset do sensor do painel usando a corrente da bateria como referencia
static String calibrarPainelAuto(bool wizard) {
    if (!bmsFresca()) {
        return wizard
            ? "<div class='er'>✗ BMS sem dados ainda. Aguarde a conexao ou volte ao passo "
              "<a href='/setup-bms' style='color:#ef9a9a'>BMS</a>.</div>"
            : "<div class='er'>✗ BMS sem dados. Conecte a BMS pela pagina "
              "<a href='/bms' style='color:#ef9a9a'>Bluetooth</a> antes de calibrar.</div>";
    }
    float iRef = sensores.correnteBateria;
    if (iRef <= 0.05f) {
        return "<div class='er'>✗ Corrente da bateria esta em ~0A ou negativa. "
               "Deixe so o <b>controlador de carga</b> e a <b>bateria</b> ligados, "
               "com sol no painel, e tente de novo.</div>";
    }
    int novoOff = (int)((float)sensores.sinalPainel_mV + iRef * calibPainel.sensMvA);
    calibPainel.offsetMV = novoOff;
    prefs.begin("rrfs_cal", false);
    prefs.putInt("pn_off", novoOff);
    prefs.end();
    char bufLog[90];
    snprintf(bufLog, sizeof(bufLog), "Calib painel: ref=%.2fA offset=%d", iRef, novoOff);
    adicionarLog(bufLog);
    return "<div class='ok'>✓ Calibrado com " + String(iRef,2) + " A (corrente da bateria)!</div>";
}

// ── Passo 3: Calibração do sensor do painel ──────────────────────────────────────
void handleSetupCalib() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (existeWiFiConfig()) {
        server.sendHeader("Location", logado ? "/sensores" : "/login"); server.send(302); return;
    }
    if (obterSetupEtapa() < 2) { server.sendHeader("Location","/setup-banco"); server.send(302); return; }

    String msg = "";
    bool calibrado = false;

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "calib_painel") {
            msg = calibrarPainelAuto(true);
            calibrado = msg.indexOf("class='ok'") >= 0;
        } else if (acao == "continuar" || acao == "pular") {
            adicionarLog(acao == "pular" ? "Setup: calibracao pulada" : "Setup: calibracao concluida");
            avancarSetupEtapa(3);
            server.sendHeader("Location","/setup-wifi");
            server.send(302);
            return;
        }
    }

    paginaInicio("RRFS Energia - Setup Calibracao");
    String c = wizardPasso(3, "Calibrar o Sensor do Painel");
    c += msg;
    c += F("<div class='nf' style='font-size:12px;line-height:1.6'>"
           "<b>📋 Instrucoes:</b><br>"
           "1) Deixe <b>apenas o controlador de carga e a bateria</b> ligados "
           "(desligue inversores, luzes, tudo que consome da bateria).<br>"
           "2) Com o painel gerando energia (precisa de sol), <b>toda a corrente do painel "
           "vai para a bateria</b>: a corrente do painel e da bateria devem ficar iguais.<br>"
           "3) Aperte <b>Calibrar Agora</b> — o sistema usa a corrente da bateria (BMS) como "
           "referencia exata, nao precisa digitar nada.<br>"
           "<span style='color:#90a4ae'>Obs: a leitura da BMS atualiza a cada ~1s, entao pode haver "
           "um pequeno atraso entre o valor do painel e o da bateria na tela — isso e normal.</span>"
           "</div>"
           "<div class='grid2'>"
           "<div class='sc' style='text-align:center'><div class='lbl'>☀️ Corrente do Painel</div>"
           "<div id='cal_p' class='val' style='color:#fdd835'>--</div></div>"
           "<div class='sc' style='text-align:center'><div class='lbl'>🔋 Corrente da Bateria</div>"
           "<div id='cal_b' class='val' style='color:#4fc3f7'>--</div></div></div>"
           "<div id='cal_cmp' style='text-align:center;font-size:12px;margin-top:4px;color:#546e7a'>"
           "Aguardando leituras...</div>"
           "<form method='POST' action='/setup-calib' style='margin-top:10px'>"
           "<input type='hidden' name='acao' value='calib_painel'>"
           "<button type='submit' class='btn-g'>🎯 Calibrar Agora</button></form>");

    if (calibrado) {
        c += F("<form method='POST' action='/setup-calib' style='margin-top:10px'>"
               "<input type='hidden' name='acao' value='continuar'>"
               "<button type='submit' class='btn-g'>✓ Calibrado, continuar →</button></form>");
    } else {
        c += F("<form method='POST' action='/setup-calib' style='margin-top:10px'>"
               "<input type='hidden' name='acao' value='pular'>"
               "<button type='submit' class='btn-s btn-sm'>Pular por enquanto →</button></form>"
               "<div class='nf' style='font-size:10px'>Sem sol agora? Pode calibrar depois na pagina "
               "Sensores.</div>");
    }

    c += F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{var d=JSON.parse(x.responseText);"
    "sT('cal_p',d.ip.toFixed(2)+' A');sT('cal_b',d.ib.toFixed(2)+' A');"
    "var dif=Math.abs(d.ip-d.ib);var el=document.getElementById('cal_cmp');"
    "if(d.ib<=0.05){el.style.color='#ef9a9a';el.textContent='⚠️ Corrente da bateria muito baixa, "
    "confirme que ha sol e tudo mais esta desligado.';}"
    "else if(dif<0.3){el.style.color='#43a047';el.textContent='✓ Valores proximos, pode calibrar.';}"
    "else{el.style.color='#ef6c00';el.textContent='Diferenca de '+dif.toFixed(2)+' A (pode ser so o atraso de ~1s da BMS).';}"
    "}catch(e){}}};x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,1500);aD();"
    "</script>");
    server.sendContent(c);
    paginaFim(false);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 34 — DASHBOARD (/)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleRoot() {
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (!existeWiFiConfig())  { server.sendHeader("Location", proximoPassoSetup()); server.send(302); return; }
    if (!logado)              { server.sendHeader("Location","/login");        server.send(302); return; }

    paginaInicio("RRFS Energia - Painel");

    if (!cfgBMS.configurada)
        server.sendContent(F("<div class='wn' style='font-size:11px'>📍 BMS nao configurada. "
             "<a href='/bms' style='color:#ffe0b2'>Configurar BMS</a></div>"));

    server.sendContent(F("<div style='display:flex;justify-content:space-between;align-items:center;"
         "font-size:11px;color:#546e7a;margin-bottom:6px'>"
         "<span id='dh_txt'>⏱️ --/--/---- --:--</span>"
         "<span id='upd_ind' style='color:#43a047;font-size:10px'>• ao vivo</span></div>"));

    // ── Banco de Baterias ────────────────────────────────────────────────
    server.sendContent(F("<h3>🔋 Banco de Baterias</h3>"
         "<div class='banco-card'>"
         "<div class='banco-title'>⚡ Configuracao do Banco</div>"
         "<div class='cr' style='margin-bottom:8px'>"
         "<div id='banco_badge' class='banco-badge'>--</div>"
         "<div style='text-align:right'>"
         "<div style='font-size:10px;color:#546e7a'>Total baterias</div>"
         "<div id='banco_total' style='font-size:16px;font-weight:bold;color:#4fc3f7'>--</div>"
         "</div></div>"
         "<div class='grid4'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>SOC</div>"
         "<div id='bms_soc' class='bms-val' style='font-size:18px'>--</div><div class='bms-sub'>%</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>Tensao</div>"
         "<div id='banco_v' style='font-size:18px;font-weight:bold;color:#4fc3f7'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>V</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>Corrente</div>"
         "<div id='banco_i' style='font-size:18px;font-weight:bold'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>A</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>Ciclos</div>"
         "<div id='bms_ciclos' style='font-size:18px;font-weight:bold;color:#90a4ae'>--</div>"
         "<div style='font-size:9px;color:#546e7a'>BMS</div></div>"
         "</div>"
         "<div class='bb' style='margin-top:8px'><div id='soc_bv' class='bv' style='width:0%'>0%</div></div>"
         "<div class='grid2' style='margin-top:6px'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>Capacidade</div>"
         "<div id='banco_cap' style='font-size:14px;font-weight:bold'>--</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>⚡ Potencia</div>"
         "<div id='banco_w' style='font-size:14px;font-weight:bold;color:#fdd835'>--</div></div>"
         "</div>"
         "<div id='banco_dir' style='text-align:center;margin-top:6px;font-size:12px'>--</div>"
         "<div id='ble_status' style='text-align:center;font-size:10px;color:#546e7a;margin-top:4px'>BLE: --</div>"
         "</div>"));

    // ── Autonomia ────────────────────────────────────────────────────────
    server.sendContent(F("<div class='pd' style='margin:6px 0;border-color:#01579b;background:#0a1a2a'>"
         "<div id='tempo_ico' class='dt' style='color:#81d4fa'>⏱️ Autonomia</div>"
         "<div id='tempo_txt' class='dv' style='color:#e1f5fe'>--</div>"
         "<div id='tempo_sub' class='ds'>calculando...</div></div>"));

    // ── Saúde da bateria ─────────────────────────────────────────────────
    server.sendContent(F("<div class='pd' style='margin:6px 0;border-color:#00695c;background:#06181a'>"
         "<div class='dt' style='color:#4db6ac'>🩺 Saude da Bateria</div>"
         "<div id='sd_geral' class='dv' style='font-size:15px'>--</div>"
         "<div class='grid2' style='margin-top:6px'>"
         "<div class='sc' style='text-align:center;padding:8px'><div class='lbl'>Capacidade (BMS)</div>"
         "<div id='sd_cap' style='font-size:16px;font-weight:bold'>--</div>"
         "<div id='sd_cap_s' class='sub'>--</div></div>"
         "<div class='sc' style='text-align:center;padding:8px'><div class='lbl'>Resistencia interna</div>"
         "<div id='sd_ri' style='font-size:16px;font-weight:bold'>--</div>"
         "<div id='sd_ri_s' class='sub'>--</div></div></div>"
         "<div class='ds' style='margin-top:6px'><a href='/saude' style='color:#4db6ac'>Ver detalhes e evolucao</a></div></div>"));
    server.sendContent(F(R"JS(<script>
var _SC=['#43a047','#9ccc65','#ef6c00','#ef5350'];
function _s(i,t){var e=document.getElementById(i);if(e)e.innerHTML=t;}
function atualizarSaude(s){
 var cg=s.lg>=0?_SC[s.lg]:'#78909c';
 _s('sd_geral','<span style="color:'+cg+'">'+(s.lg>=0?s.g_txt:'Coletando dados')+'</span>');
 if(s.cap_ok){
  _s('sd_cap','<span style="color:'+_SC[s.lc]+'">'+s.soh.toFixed(0)+'%</span>');
  _s('sd_cap_s',s.cap_med.toFixed(0)+' de '+s.cap_nom.toFixed(0)+' Ah | '+s.c_txt);
 }else{
  _s('sd_cap','--');
  _s('sd_cap_s',s.jan?('medindo: '+s.jan_ini+'% → '+s.jan_soc+'% ('+s.jan_ah.toFixed(1)+' Ah)'):('aguardando janela de 20% de SOC ('+s.n_cap+'/2)'));
 }
 if(s.ri_ok){
  var cr=s.lr>=0?_SC[s.lr]:'#e0e0e0';
  _s('sd_ri','<span style="color:'+cr+'">'+s.ri.toFixed(1)+' mΩ</span>');
  _s('sd_ri_s',s.ri_base_ok?('ref '+s.ri_base.toFixed(1)+' mΩ | '+s.ri_ratio.toFixed(2)+'x | '+s.r_txt):('medindo referencia ('+s.ri_n+'/8)'));
 }else{
  _s('sd_ri','--');
  _s('sd_ri_s','aguardando uma carga forte (ex.: Inv.Alta ligar)');
 }
}
function aSd(){var x=new XMLHttpRequest();
 x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){
  try{atualizarSaude(JSON.parse(x.responseText));}catch(e){}}};
 x.open('GET','/saude-dados',true);x.send();}
setInterval(aSd,10000);aSd();
</script>)JS"));

    // ── Planejador de energia ────────────────────────────────────────────
    server.sendContent(F("<div id='plano_box' class='pd' style='margin:6px 0;border-color:#7b1fa2;background:#12081a'>"
    "<div id='plano_tit' class='dt' style='color:#ce93d8'>🧠 Planejador de energia</div>"
    "<div id='plano_txt' class='dv' style='font-size:14px;color:#e1bee7'>--</div>"
    "<div id='plano_det' class='ds' style='font-size:11px;line-height:1.5'></div>"
    "<div id='plano_ps' class='ds' style='font-size:11px;line-height:1.6;margin-top:8px;padding:8px;"
    "border:1px solid #4a148c;border-radius:6px;background:#0d0615'>🧩 carregando...</div>"
    "<div id='plano_pad' class='ds' style='font-size:11px;line-height:1.5;margin-top:6px;color:#b39ddb'></div>"
    "<div id='plano_tips' class='ds' style='font-size:11px;line-height:1.5;margin-top:6px'></div>"
    "</div>"));
    server.sendContent(F(R"JS(<script>
function $p(i){return document.getElementById(i);}
function hh(h){return (h<10?'0':'')+h+'h';}
function qd(dd,h){return (dd==0?'hoje ':dd==1?'amanha ':'em 2 dias ')+hh(h);}
function fd(s){s=Math.round(s);
 if(s>=3600)return Math.floor(s/3600)+'h'+('0'+Math.floor((s%3600)/60)).slice(-2)+'m';
 if(s>=60)return Math.floor(s/60)+'min';return s+'s';}
function statusPadrao(p){
 var vb=p.vb>1?p.vb:12,t='',d='';
 var bs='Base: <b>'+p.pbase.toFixed(1)+' A</b> ('+Math.round(p.pbase*vb)+' W) | flutuacao ±'+p.pdev.toFixed(1)+' A | '+
        'evento acima de +'+p.pthr.toFixed(1)+' A';
 if(p.ps==0){
  t='<span style="color:#78909c">⚪ Detector aguardando</span>';
  d='Precisa de BMS com dados e relogio NTP sincronizado.';
 }else if(p.ps==1){
  t='<span style="color:#fdd835">☀️ Sobra solar — aprendizado pausado</span>';
  d='O consumo agora sai dos paineis, nao da bateria. Nada e registrado nem aprendido.';
 }else if(p.ps==2){
  t='<span style="color:#90a4ae">⏳ Estabilizando</span>';
  d='Lampada ou inversor mudou de estado: aguardando a corrente assentar (ate ~75 s).';
 }else if(p.ps==3){
  t='<span style="color:#43a047">🟢 Corrente estavel</span>';
  d=bs+'<br>Uso normal da casa, nenhum equipamento forte ligado.';
 }else if(p.ps==4){
  t='<span style="color:#fdd835">🟡 Corrente subindo — confirmando</span>';
  d=bs+'<br>Se ficar acima por 6 s vira um evento.';
 }else if(p.ps==5){
     t='<span style="color:#4fc3f7">🔵 Padrao conhecido #'+p.pk+' em andamento</span>';
    d='Agora: <b>+'+p.pe.toFixed(1)+' A</b> (~'+Math.round(p.pe*vb)+' W) acima da base, ha '+fd(p.pt)+'.<br>'+
    'Padrao salvo: +'+p.pcA.toFixed(1)+' A por ~'+fd(p.pcD)+'. E um consumo temporario ja conhecido: o planejador nao se confunde.<br>'+bs;
 }else{
     t='<span style="color:#ce93d8">🟣 Corrente NOVA — aprendendo</span>';
    d='Agora: <b>+'+p.pe.toFixed(1)+' A</b> (~'+Math.round(p.pe*vb)+' W) acima da base, ha '+fd(p.pt)+'.<br>'+
    'Nao bate com nenhum padrao salvo. Sera registrada quando terminar e entra no aprendizado na proxima hora.<br>'+bs;
 }
 return '<b>🧩 Corrente da casa agora:</b> '+t+'<br>'+d;
}
function atualizarPlano(p){
    var vb=p.vb>1?p.vb:12;
    function w(a){return Math.round(a*vb)+' Wh';}
    var tx='',dt='',tp='',pd='';
    if(p.est==-1&&!p.sobra){
        var m=['','ative a previsao solar (pagina ☀️)','BMS sem dados','ative o NTP (relogio)','medindo o consumo...',
                         'aprendendo a rotina: '+p.pad_h+' de '+p.pad_min+' h de dados (a iluminacao ainda nao sera bloqueada)'];
                         tx='Aguardando dados';dt=m[p.sm]||'';
    }else if(p.est==-2){
        tx='<span style="color:#ef5350">⚠️ Protecao da bateria ativa</span>';
    dt='SOC abaixo da reserva: inversor e iluminacao desligados.';
    }else if(p.sobra){
        var pot=(p.prev_a>p.ic)?Math.round((p.prev_a-p.ic)*vb):0;
        tx='<span style="color:#43a047">☀️ Excesso de energia — Inv. Alta ligado, aproveite!</span>';
    dt='Bateria cheia ('+p.soc.toFixed(0)+'%) e os paineis geram mais do que a casa precisa '+
    '(painel '+p.ip.toFixed(1)+' A | casa '+p.ic.toFixed(1)+' A).<br>'+
    'O consumo agora sai da <b>sobra dos paineis</b>, nao da bateria, e nenhuma economia e necessaria.'+
    (pot>0?'<br>Os paineis ainda podem entregar ate ~<b>'+pot+' W</b> a mais (previsao desta hora).':'');
    tp='<b>☀️ Aproveite enquanto o sol esta forte:</b><br>'+
    '• Maquina de lavar/secar, ferro de passar, forno e micro-ondas.<br>'+
    '• Bomba d\'agua, aspirador, chuveiro/aquecedor eletrico se houver.<br>'+
    '• Carregue notebooks, celulares, ferramentas e power banks.<br>'+
    '• Ar-condicionado/ventilador: pre-resfrie a casa.<br>'+
    '• O Inv. Alta fica ligado enquanto a casa pedir; se nao pedir, desliga quando o excesso acabar.';
    pd='🧩 Padroes: aprendizado pausado durante o excesso.';
    }else{
        var c='Consumo base da casa: '+p.cons_a.toFixed(1)+' A ('+Math.round(p.cons_a*vb)+' W)';
    if(p.hist&&p.pico_h>=0)c+=' | maior consumo costuma ser as '+hh(p.pico_h)+' ('+p.pico_a.toFixed(1)+' A)';
    if(p.est==0){
        tx='<span style="color:#43a047">✓ Energia suficiente</span> — iluminacao liberada';
    dt='Menor sobra prevista: '+p.c_min.toFixed(1)+' Ah acima da reserva ('+qd(p.c_fd,p.c_fh)+').<br>'+
    'Ate la: consumo previsto '+p.c_cons.toFixed(1)+' Ah ('+w(p.c_cons)+') | geracao prevista '+p.c_ger.toFixed(1)+' Ah ('+w(p.c_ger)+').<br>'+c;
    }else if(p.est==1){
        tx='<span style="color:#ef6c00">💡 Iluminacao externa desligada para economizar</span>';
    var a1=p.c_min<0?'faltariam '+(-p.c_min).toFixed(1)+' Ah (bateria na reserva '+(p.c_lh>=0?qd(p.c_ld,p.c_lh):'')+')':'sobraria so '+p.c_min.toFixed(1)+' Ah';
    dt='Motivo: ligada, '+a1+' ate '+qd(p.c_fd,p.c_fh)+'.<br>'+
    'Demanda prevista ate la: '+p.c_cons.toFixed(1)+' Ah ('+w(p.c_cons)+') | geracao prevista: '+p.c_ger.toFixed(1)+' Ah ('+w(p.c_ger)+').<br>'+
    'Desligada: sobra minima de '+p.s_min.toFixed(1)+' Ah → <b>suficiente</b>.<br>'+
    (p.analis?'<span style="color:#fdd835">🔎 Analisando: confirmando com o consumo real...</span>':'<span style="color:#43a047">✓ Confirmado: o desligamento foi suficiente.</span>')+'<br>'+c;
    }else{
        var meta=Math.max(0,p.cons_a-p.red_a);
        if(p.red_a<0.05||p.falta_ah<=0){
            tx='<span style="color:#fdd835">🔎 Reavaliando: o cenario melhorou</span>';
    dt='A previsao ja nao mostra falta de energia. Confirmando antes de liberar tudo.<br>'+c;
        }else{
            tx='<span style="color:#ef5350">⚠️ Economize energia ate '+qd(p.s_fd,p.s_fh)+'</span>';
    dt='Mesmo sem a iluminacao externa faltam '+p.falta_ah.toFixed(1)+' Ah ate '+qd(p.s_fd,p.s_fh)+
    (p.s_lh>=0?' (bateria chegaria na reserva '+qd(p.s_ld,p.s_lh)+')':'')+'.<br>'+
    'Demanda prevista: '+p.s_cons.toFixed(1)+' Ah ('+w(p.s_cons)+') | geracao prevista: '+p.s_ger.toFixed(1)+' Ah ('+w(p.s_ger)+').<br>'+
    '<b>Reduza ~'+p.red_a.toFixed(1)+' A (~'+Math.round(p.red_w)+' W)</b>: de '+p.cons_a.toFixed(1)+' A para no maximo '+meta.toFixed(1)+' A ('+Math.round(meta*vb)+' W).<br>'+c;
    tp='<b>💡 Dicas ate '+qd(p.s_fd,p.s_fh)+':</b><br>'+
    '• A geracao prevista nao cobre a casa durante o dia e ainda carregar a bateria para a noite.<br>'+
    '• Evite forno, micro-ondas, panela eletrica e maquina de lavar ate depois das '+hh(p.s_fh)+'.<br>'+
    '• Geladeira: regule para menos frio e evite abrir a porta.<br>'+
    '• Desligue TV, computador e carregadores sem uso (standby).<br>'+
    '• Evite ar-condicionado e aquecedores; apague luzes internas desnecessarias.';
        }
    }
    if(p.pad){
        pd='🧩 Padroes: '+p.pad_dias+' dia(s) aprendidos, '+p.pad_n+' tipo(s) de uso forte. '+
    'Ainda esperados hoje: <b>'+p.ev_rest.toFixed(1)+' Ah</b> ('+w(p.ev_rest)+') em equipamentos pesados.';
    }else{
        pd='🧩 Padroes: coletando rotina ('+p.pad_h+' de '+p.pad_min+' h). Ate la o planejador nao desliga a iluminacao.';
    }
    }
    if(p.x_h>=0&&!p.sobra)pd+='<br>☀️ Proximo excesso de energia previsto: <b>'+qd(p.x_d,p.x_h)+'</b>.';
    $p('plano_txt').innerHTML=tx;$p('plano_det').innerHTML=dt;
    $p('plano_tips').innerHTML=tp;$p('plano_pad').innerHTML=pd;
    $p('plano_ps').innerHTML=statusPadrao(p);
}
function aPl(){var x=new XMLHttpRequest();
    x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){
        try{atualizarPlano(JSON.parse(x.responseText));}catch(e){}}};
        x.open('GET','/plano',true);x.send();}
        setInterval(aPl,5000);aPl();
        function atualizarPS(s){
            var e=$p('prev_acc');if(!e)return;
            if(s.acc<0){e.innerHTML='🎯 Precisao: coletando dados ('+s.n+'/5 horas)';return;}
            var c=s.acc>=85?'#43a047':s.acc>=70?'#9ccc65':s.acc>=50?'#ef6c00':'#ef5350';
    var q=s.acc>=85?'Excelente':s.acc>=70?'Boa':s.acc>=50?'Regular':'Fraca';
    e.innerHTML='🎯 Precisao: <b style="color:'+c+'">'+s.acc.toFixed(0)+'% ('+q+')</b>';
        }
        function aPS(){var x=new XMLHttpRequest();
            x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){
                try{atualizarPS(JSON.parse(x.responseText));}catch(e){}}};
                x.open('GET','/prevstat',true);x.send();}
                setInterval(aPS,30000);aPS();
                </script>)JS"));

    // ── Inversores ───────────────────────────────────────────────────────
    server.sendContent(F("<h3>⚡ Inversor Baixa Potencia</h3>"
         "<div id='card_baixa' class='cg'><div class='cr'>"
         "<span class='cn'>⚡ Sempre tenta ligar</span><span id='st_baixa' class='cs'>--</span></div>"
         "<div id='sub_baixa' class='ci'>--</div></div>"
         "<h3>⚡ Inversor Alta Potencia</h3>"
         "<div id='card_alta' class='cg'><div class='cr'>"
         "<span class='cn'>⚡ Liga sob demanda (casa ou excesso solar)</span><span id='st_alta' class='cs'>--</span></div>"
         "<div id='sub_alta' class='ci'>--</div></div>"));

    // ── Excesso de energia solar (esconde quando a geração do dia acabou) ─
    server.sendContent(F("<div id='exc_box' class='pd' style='margin:6px 0;border-color:#ef6c00;background:#1a1000;display:none'>"
         "<div class='dt' style='color:#ffb74d'>☀️ Excesso de energia solar</div>"
         "<div id='exc_txt' class='dv' style='font-size:14px;color:#ffe0b2'>--</div>"
         "<div id='exc_det' class='ds' style='font-size:11px;line-height:1.5'></div></div>"));
    server.sendContent(F(R"JS(<script>
function aEx(){var x=new XMLHttpRequest();
 x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){try{
  var e=JSON.parse(x.responseText),t='',d='';
  var box=document.getElementById('exc_box');
  if(e.fim_ger&&e.fase!=2){box.style.display='none';return;}
  box.style.display='block';
  if(e.fase==2){
   box.style.borderColor='#43a047';
   t='<span style="color:#43a047">✓ Excesso ATIVO</span> — Inv.Alta ligado com a sobra do painel';
   if(e.graca_s>0)d='Aguardando o painel subir a corrente: '+e.graca_s+'s de tolerancia (a bateria pode ceder energia).';
   else if(e.fim_on)d='<span style="color:#ef6c00">Bateria voltou a descarregar: '+e.motivo+(e.fim_s>0?' ('+e.fim_s+'s)':'')+'</span>';
   else d='Painel cobrindo a casa. Desliga se o SOC cair 1% e nao recuperar (se a casa nao pedir).';
   d+='<br>Ativo ha '+e.ativo_s+'s | SOC '+e.soc.toFixed(0)+'% | Painel '+e.ip.toFixed(1)+'A | Bateria '+e.ib.toFixed(1)+'A';
  }else if(e.fase==1){
   box.style.borderColor='#fdd835';
   t='<span style="color:#fdd835">🔎 Excesso detectado: confirmando ('+e.conf_s+'s)</span>';
   d='Bateria cheia ('+e.soc.toFixed(0)+'%), corrente da bateria zerada e painel gerando '+e.ip.toFixed(1)+'A. Mantendo por 1 min, depois liga o Inv.Alta.';
  }else{
   box.style.borderColor='#ef6c00';
   t='Monitorando: '+e.motivo+(e.cool_s>0?' ('+e.cool_s+'s)':'');
   d='Entra em excesso com SOC >= '+e.lim_soc.toFixed(0)+'%, painel gerando e bateria sem descarregar por 1 min.';
   if(e.ultimo_fim)d+='<br>Ultimo fim: '+e.ultimo_fim;
  }
  d+='<br>Excessos aproveitados (desde o boot): '+e.eventos;
  document.getElementById('exc_txt').innerHTML=t;
  document.getElementById('exc_det').innerHTML=d;
 }catch(err){}}};
 x.open('GET','/excesso',true);x.send();}
setInterval(aEx,2000);aEx();
</script>)JS"));

    // ── Sinais AC + Casa ─────────────────────────────────────────────────
    server.sendContent(F("<h3>📶 Sinais AC e Pedido da Casa</h3>"
         "<div class='sig-box'><div class='cr'>"
         "<span class='sig-lbl'>GPIO35 — AC Inversor Baixa</span>"
         "<span id='ac_baixa_txt' class='sig-val' style='font-size:14px'>--</span></div>"
         "<div id='ac_baixa_sub' class='sig-sub'>--</div>"
         "<div class='bb'><div id='ac_baixa_bv' class='bv' style='width:0%'></div></div></div>"
         "<div class='sig-box'><div class='cr'>"
         "<span class='sig-lbl'>GPIO36 — AC Inversor Alta (diagnostico)</span>"
         "<span id='ac_alta_txt' class='sig-val' style='font-size:14px'>--</span></div>"
         "<div id='ac_alta_sub' class='sig-sub'>--</div>"
         "<div class='bb'><div id='ac_alta_bv' class='bv' style='width:0%'></div></div></div>"
         "<div class='sig-box' style='border-color:#4a148c'><div class='cr'>"
         "<span class='sig-lbl'>GPIO39 — Pedido da Casa (liga Inv.Alta)</span>"
         "<span id='casa_txt' class='sig-val' style='font-size:14px'>--</span></div>"
         "<div id='casa_sub' class='sig-sub'>--</div>"
         "<div class='bb'><div id='casa_bv' class='bv' style='width:0%'></div></div></div>"));

    // ── Previsão Solar ───────────────────────────────────────────────────
    if (cfgPrevisao.ativa && cfgPrevisao.configurada) {
        server.sendContent(F("<div id='prev_box' class='pd' style='display:none'>"
             "<div class='dt'>☀️ Previsao Solar</div>"
             "<div class='grid2'>"
             "<div style='text-align:center'><div style='font-size:10px;color:#546e7a'>Esta hora</div>"
             "<div id='prev_h' class='dv'>--</div></div>"
             "<div style='text-align:center'><div style='font-size:10px;color:#546e7a'>Restante hoje</div>"
             "<div id='prev_r' class='dv'>--</div></div></div>"
             "<div id='prev_acc' class='ds' style='margin-top:6px;font-size:11px'>🎯 Precisao: --</div>"
             "<div class='ds'><a href='/previsao' style='color:#43a047'>Ver completo</a></div></div>"));
    }

    // ── SVG Fluxo ────────────────────────────────────────────────────────
    server.sendContent(F("<h3 style='text-align:center;margin:8px 0 6px'>⚡ Fluxo de Energia</h3>"
         "<svg viewBox='0 0 340 220' style='width:100%;height:auto;display:block'>"
         "<path id='ln_pb' d='M110,50 C160,20 200,20 240,50' fill='none' stroke='#333' stroke-width='3'/>"
         "<path id='ln_pc' d='M90,85 C80,125 90,145 130,168' fill='none' stroke='#333' stroke-width='3'/>"
         "<path id='ln_bc' d='M255,85 C260,125 250,145 210,168' fill='none' stroke='#333' stroke-width='3'/>"
         "<g><rect x='20' y='10' width='100' height='80' rx='12' fill='#181818' id='rect_p' stroke='#455a64' stroke-width='2'/>"
         "<text x='70' y='36' text-anchor='middle' font-size='20'>☀️</text>"
         "<text x='70' y='50' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>"
         "<text id='sv_ip' x='70' y='65' text-anchor='middle' font-size='13' fill='#455a64' font-weight='bold'>0.0A</text>"
         "<text id='sv_wp' x='70' y='80' text-anchor='middle' font-size='11' fill='#455a64'>0W</text></g>"
         "<g><rect x='220' y='10' width='100' height='80' rx='12' fill='#181818' id='rect_b' stroke='#455a64' stroke-width='2'/>"
         "<text id='sv_ico_b' x='270' y='34' text-anchor='middle' font-size='18'>🔋</text>"
         "<text x='270' y='48' text-anchor='middle' font-size='9' fill='#78909c'>BANCO</text>"
         "<text id='sv_soc' x='270' y='63' text-anchor='middle' font-size='13' fill='#455a64' font-weight='bold'>--%</text>"
         "<text id='sv_ib' x='270' y='78' text-anchor='middle' font-size='11' fill='#455a64'>--</text></g>"
         "<g><rect id='rect_c' x='105' y='163' width='130' height='55' rx='10' fill='#181818' stroke='#3a3a3a' stroke-width='2'/>"
         "<text x='130' y='195' font-size='18'>🏠</text>"
         "<text id='sv_ico_inv' x='145' y='175' text-anchor='start' font-size='9' fill='#78909c'>CASA</text>"
         "<text id='sv_ic' x='228' y='190' text-anchor='end' font-size='13' fill='#3a3a3a' font-weight='bold'>--</text>"
         "<text id='sv_wc' x='228' y='205' text-anchor='end' font-size='11' fill='#3a3a3a'>--</text></g>"
         "</svg>"));

    // ── Grid correntes + info ────────────────────────────────────────────
    server.sendContent(F("<div class='grid3' style='gap:6px;margin:4px 0'>"
         "<div class='sc' style='padding:8px;text-align:center'><div class='lbl'>☀️ Painel</div>"
         "<div id='cc_p' style='font-size:14px;font-weight:bold'>--</div>"
         "<div id='cw_p' style='font-size:11px;color:#78909c'>--</div></div>"
         "<div class='sc' style='padding:8px;text-align:center'><div class='lbl'>🏠 Casa</div>"
         "<div id='cc_c' style='font-size:14px;font-weight:bold;color:#4fc3f7'>--</div>"
         "<div id='cw_c' style='font-size:11px;color:#78909c'>--</div></div>"
         "<div class='sc' style='padding:8px;text-align:center'><div class='lbl'>🔋 Banco</div>"
         "<div id='cc_b' style='font-size:14px;font-weight:bold'>--</div>"
         "<div id='cw_b' style='font-size:11px;color:#78909c'>--</div></div></div>"
         "<div id='sys_info' class='nf' style='font-size:10px'>Carregando...</div>"));

    // ── JAVASCRIPT ───────────────────────────────────────────────────────
    server.sendContent(F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function sBc(id,c){var e=document.getElementById(id);if(e)e.style.borderColor=c;}"
    "function sAttr(id,a,v){var e=document.getElementById(id);if(e)e.setAttribute(a,v);}"
    "function atualizarDash(d){"
    "sT('dh_txt','⏱️ '+d.data_hora+(d.ntp?' <span style=\"color:#43a047\">•NTP</span>':''));"
    "sT('banco_badge',d.nS+'S'+d.nP+'P');"
    "sT('banco_total',(d.nS*d.nP)+' bat');"
    "var cSOC=d.soc>=50?'#43a047':d.soc>=25?'#ef6c00':'#c62828';"
    "sT('bms_soc','<span style=\"color:'+cSOC+'\">'+d.soc.toFixed(0)+'</span>');"
    "var socBv=document.getElementById('soc_bv');"
    "if(socBv){socBv.style.width=Math.min(100,d.soc)+'%';socBv.style.background=cSOC;socBv.textContent=d.soc.toFixed(0)+'%';}"
    "sT('banco_v',d.v_banco.toFixed(1));"
    "var cIB=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
    "sT('banco_i','<span style=\"color:'+cIB+'\">'+Math.abs(d.ib).toFixed(2)+'</span>');"
    "sT('bms_ciclos',d.ciclos);"
    "sT('banco_cap',d.cap_banco.toFixed(2)+' Ah');"
    "sT('banco_w',Math.round(Math.abs(d.wb))+'W');"
    "var dirStr=d.ib>0.2?'↑ Carregando '+d.ib.toFixed(2)+'A':d.ib<-0.2?'↓ Descarregando '+Math.abs(d.ib).toFixed(2)+'A':'↔ Parado';"
    "var dirCol=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
    "sT('banco_dir','<span style=\"color:'+dirCol+'\">'+dirStr+'</span>');"
    "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#43a047':'#ef5350';"
    "sT('ble_status','BLE: <span style=\"color:'+cBLE+'\">'+d.ble_str+'</span> | '+d.bms_t_seg+'s atras');"
    "var cBaixa=d.inva?'#fdd835':'#546e7a';"
    "sBc('card_baixa',cBaixa);"
    "sT('st_baixa','<span style=\"color:'+cBaixa+'\">'+(d.inva?'• ON':'○ OFF')+'</span>');"
    "sT('sub_baixa',d.nome_baixa+' | '+d.tempo_baixa+'s | Ligou:'+d.lig_baixa+' | Falhas:'+d.fal_baixa);"
    "var cAlta=d.inv_alta?'#fdd835':'#546e7a';"
    "sBc('card_alta',cAlta);"
    "sT('st_alta','<span style=\"color:'+cAlta+'\">'+(d.inv_alta?'• ON':'○ OFF')+'</span>');"
    "sT('sub_alta',d.nome_alta+' | '+d.tempo_alta+'s | Ligou:'+d.lig_alta+' | Falhas:'+d.fal_alta+"
    "' | '+(d.casa_pede?'<span style=\"color:#43a047\">🏠 Casa PEDIU</span>':'<span style=\"color:#546e7a\">🏠 Casa nao pediu</span>'));"
    "var cAcBaixa=d.ac_baixa_ok?'#43a047':'#546e7a';"
    "sT('ac_baixa_txt','<span style=\"color:'+cAcBaixa+'\">'+(d.ac_baixa_ok?'• OK':'○ SEM AC')+'</span>');"
    "sT('ac_baixa_sub',d.ac_baixa_mv.toFixed(0)+' mV');"
    "var bAcBaixa=document.getElementById('ac_baixa_bv');"
    "if(bAcBaixa){bAcBaixa.style.width=Math.min((d.ac_baixa_mv/3300)*100,100)+'%';bAcBaixa.style.background=cAcBaixa;}"
    "var cAcAlta=d.ac_alta_ok?'#43a047':'#546e7a';"
    "sT('ac_alta_txt','<span style=\"color:'+cAcAlta+'\">'+(d.ac_alta_ok?'• OK':'○ SEM AC')+'</span>');"
    "sT('ac_alta_sub',d.ac_alta_mv.toFixed(0)+' mV');"
    "var bAcAlta=document.getElementById('ac_alta_bv');"
    "if(bAcAlta){bAcAlta.style.width=Math.min((d.ac_alta_mv/3300)*100,100)+'%';bAcAlta.style.background=cAcAlta;}"
    "var cCasa=d.casa_pede?'#43a047':'#546e7a';"
    "sT('casa_txt','<span style=\"color:'+cCasa+'\">'+(d.casa_pede?'• PEDINDO':'○ SEM PEDIDO')+'</span>');"
    "sT('casa_sub',d.casa_mv.toFixed(0)+' mV');"
    "var bCasa=document.getElementById('casa_bv');"
    "if(bCasa){bCasa.style.width=Math.min((d.casa_mv/3300)*100,100)+'%';bCasa.style.background=cCasa;}"
    "var pBox=document.getElementById('prev_box');"
    "if(pBox){if(d.prev_ok&&(d.ah_prev_h>0||d.ah_prev_r>0)){"
    "pBox.style.display='block';var vbp=d.v_banco>1?d.v_banco:12;"
    "sT('prev_h',d.ah_prev_h.toFixed(1)+' Ah · '+Math.round(d.ah_prev_h*vbp)+' W');"
    "sT('prev_r',d.ah_prev_r.toFixed(1)+' Ah · '+Math.round(d.ah_prev_r*vbp)+' Wh');"
    "}else pBox.style.display='none';}"
    "if(d.tempo_val){"
    "var hm=d.tempo_h+'h '+(d.tempo_m<10?'0':'')+d.tempo_m+'min';"
    "if(d.tempo_carg){"
    "sT('tempo_ico','↑ Carregando');"
    "sT('tempo_txt','<span style=\"color:#43a047\">'+hm+'</span>');"
    "sT('tempo_sub','para completar 100% | faltam '+d.ah_falta.toFixed(1)+' Ah');"
    "}else{"
    "sT('tempo_ico','↓ Descarregando');"
    "sT('tempo_txt','<span style=\"color:#ef6c00\">'+hm+'</span>');"
    "sT('tempo_sub','ate a reserva de '+d.reserva.toFixed(0)+'% | util: '+d.ah_disp.toFixed(1)+' Ah');}"
    "}else{"
    "sT('tempo_ico',d.prot?'⚠️ PROTECAO ATIVA':'↔ Autonomia');"
    "sT('tempo_txt','--');"
    "sT('tempo_sub',d.prot?'SOC abaixo da reserva: inversor e iluminacao desligados':'corrente baixa para estimar | util: '+d.ah_disp.toFixed(1)+' Ah');}"
    "var painelGer=(d.ip>0.5);var batCarg=(d.ib>0.2);var batDesc=(d.ib<-0.2);"
    "var algumInvLigado=(d.inva||d.inv_alta);"
    "var cP=painelGer?'#fdd835':'#455a64';"
    "var cBat=d.soc>=50?'#43a047':d.soc>=25?'#ef6c00':'#c62828';"
    "var cCas=algumInvLigado?'#fdd835':'#3a3a3a';"
    "var lPB=(painelGer&&batCarg)?'#43a047':'#333';"
    "var lPC=(painelGer&&d.ic>0.3)?'#fdd835':'#333';"
    "var lBC=(batDesc&&d.ic>0.3)?'#ef6c00':'#333';"
    "sAttr('rect_p','stroke',cP);sAttr('rect_b','stroke',cBat);sAttr('rect_c','stroke',cCas);"
    "sAttr('ln_pb','stroke',lPB);sAttr('ln_pc','stroke',lPC);sAttr('ln_bc','stroke',lBC);"
    "var svIp=document.getElementById('sv_ip');"
    "if(svIp){svIp.textContent=painelGer?d.ip.toFixed(1)+'A':'0.0A';svIp.style.fill=cP;}"
    "var svWp=document.getElementById('sv_wp');"
    "if(svWp){svWp.textContent=painelGer?Math.round(d.wp)+'W':'0W';svWp.style.fill=cP;}"
    "sT('sv_ico_b',batCarg?'↑🔋':batDesc?'↓🔋':'🔋');"
    "var svSoc=document.getElementById('sv_soc');"
    "if(svSoc){svSoc.textContent=d.soc.toFixed(0)+'%';svSoc.style.fill=cBat;}"
    "var svIb=document.getElementById('sv_ib');"
    "if(svIb){svIb.textContent=(batCarg?'+':batDesc?'-':'~')+Math.abs(d.ib).toFixed(1)+'A';svIb.style.fill=cBat;}"
    "sT('sv_ico_inv',algumInvLigado?'🏠⚡':'🏠●');"
    "var svIc=document.getElementById('sv_ic');"
    "if(svIc){svIc.textContent=d.ic.toFixed(1)+'A';svIc.style.fill=cCas;}"
    "var svWc=document.getElementById('sv_wc');"
    "if(svWc){svWc.textContent=Math.round(d.wc)+'W';svWc.style.fill=cCas;}"
    "var cP2=painelGer?'#fdd835':'#455a64';"
    "var cB2=batCarg?'#43a047':batDesc?'#ef6c00':'#fdd835';"
    "sT('cc_p','<span style=\"color:'+cP2+'\">'+(painelGer?d.ip.toFixed(1)+'A':'0.0A')+'</span>');"
    "sT('cw_p',painelGer?Math.round(d.wp)+'W':'0W');"
    "sT('cc_c',d.ic.toFixed(1)+'A');sT('cw_c',Math.round(d.wc)+'W');"
    "var siBI=(d.ib>0.2?'+':d.ib<-0.2?'-':'~')+Math.abs(d.ib).toFixed(1)+'A';"
    "sT('cc_b','<span style=\"color:'+cB2+'\">'+siBI+'</span>');"
    "sT('cw_b',Math.round(Math.abs(d.wb))+'W');"
    "var si='📶 '+d.ram.toFixed(1)+'KB RAM | Up '+d.uptime+'s | Cap '+d.cap_banco.toFixed(1)+'Ah';"
    "if(d.modo_teste)si+=' | <span style=\"color:#ef6c00\">MODO TESTE</span>';"
    "sT('sys_info',si);"
    "var ind=document.getElementById('upd_ind');"
    "if(ind){ind.style.opacity='1';setTimeout(function(){if(ind)ind.style.opacity='0.3';},300);}}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarDash(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>"));

    paginaFim(true);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 35 — BMS BLUETOOTH (página)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleBMSScan() {
    if (!logado) { server.send(403); return; }
    if (!bleScanAtivo) bleScanSolicitado = true;
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleBMSLista() {
    if (!logado) { server.send(403); return; }
    String j = "{\"scan\":";
    j += (bleScanAtivo || bleScanSolicitado) ? "true" : "false";
    j += ",\"itens\":[";
    uint8_t n = bleListaN;
    for (uint8_t i = 0; i < n; i++) {
        if (i) j += ",";
        j += "{\"mac\":\""; j += bleLista[i].mac; j += "\",\"nome\":\"";
        for (const char* p = bleLista[i].nome; *p; p++) {
            char ch = *p;
            if (ch == '"' || ch == '\\' || (unsigned char)ch < 32 || (unsigned char)ch > 126) ch = ' ';
            j += ch;
        }
        j += "\",\"rssi\":"; j += String(bleLista[i].rssi);
        j += ",\"jbd\":";    j += bleLista[i].jbd ? "true" : "false";
        j += "}";
    }
    j += "]}";
    server.send(200, "application/json", j);
}

void handleBMS() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "salvar_mac") {
            String mac = server.arg("mac");
            if (!macValido(mac)) {
                msg = "<div class='er'>✗ MAC invalido. Formato: AA:BB:CC:DD:EE:FF</div>";
            } else {
                aplicarMacBMS(mac);
                msg = "<div class='ok'>✓ BMS salva (" + mac + "). Conectando...</div>";
            }
        } else if (acao == "reconectar") {
            desconectarBMS();
            bleManualOff = false;
            ctrlBLE.tUltimoTentativa = 0;
            ctrlBLE.falhasConsec     = 0;
            msg = "<div class='ok'>✓ Conectando a BMS...</div>";
            adicionarLog("BMS: conexao manual");
        } else if (acao == "desconectar") {
            bleManualOff = true;
            desconectarBMS();
            msg = "<div class='ok'>✓ BMS desconectada. Nao reconecta sozinha.</div>";
            adicionarLog("BMS: desconexao manual");
        } else if (acao == "limpar") {
            bleManualOff = false;
            desconectarBMS();
            cfgBMS.configurada   = false;
            cfgBMS.macAddress[0] = '\0';
            dadosBMS.valido      = false;
            salvarConfigBMS();
            msg = "<div class='ok'>✓ BMS removida.</div>";
            adicionarLog("BMS: config removida");
        }
    }

    paginaInicio("RRFS Energia - BMS");
    String c = msg;
    c += F("<div class='nf'>📡 <b>BMS Bluetooth JBD/LanmaoBMS</b><br>"
           "1) Feche o app LanmaoBMS no celular.<br>"
           "2) Toque em <b>Buscar BMS</b> e escolha a sua.<br>"
           "3) Banco e reserva: pagina <a href='/bateria' style='color:#90caf9'>Bateria</a>.</div>"
           "<h3>⚙️ BMS selecionada</h3>");
    if (cfgBMS.configurada) {
        char macBuf[96];
        snprintf(macBuf, sizeof(macBuf),
            "<div class='ok' style='font-family:monospace;letter-spacing:2px'>✓ <b>%s</b></div>", cfgBMS.macAddress);
        c += macBuf;
    } else {
        c += F("<div class='wn'>⚠️ Nenhuma BMS selecionada</div>");
    }
    c += F("<button type='button' class='btn-b' id='btnScan' onclick='iniciarScan()'>🔍 Buscar BMS</button>"
           "<div id='scan_st' style='text-align:center;font-size:11px;color:#78909c;margin:6px 0'></div>"
           "<div id='scan_lista'></div>"
           "<details style='margin-top:8px'><summary style='font-size:12px;color:#78909c'>"
           "Informar MAC manualmente</summary>"
           "<form method='POST' action='/bms'>"
           "<input type='hidden' name='acao' value='salvar_mac'>"
           "<label>MAC (AA:BB:CC:DD:EE:FF):</label>"
           "<input type='text' name='mac' maxlength='17' placeholder='AA:BB:CC:DD:EE:FF' "
           "style='font-family:monospace;text-transform:uppercase' required>"
           "<button type='submit' class='btn-b btn-sm'>💾 Salvar MAC</button></form></details>");
    server.sendContent(c);
    enviarScriptScanBMS("/bms");

    c = F("<h3>📊 Status BLE</h3>"
           "<div class='bms-card'>"
           "<div class='grid2'>"
           "<div><div style='font-size:10px;color:#7b1fa2'>Estado BLE</div>"
           "<div id='b_estado' class='bms-val' style='font-size:16px'>--</div></div>"
           "<div><div style='font-size:10px;color:#7b1fa2'>Falhas</div>"
           "<div id='b_falhas' class='bms-val' style='font-size:16px'>--</div></div></div>"
           "<div class='grid3' style='margin-top:8px'>"
           "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Corrente</div>"
           "<div id='b_corr' class='bms-val'>--</div><div id='b_dir' class='bms-sub'>--</div></div>"
           "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Tensao</div>"
           "<div id='b_tensao' class='bms-val'>--</div><div class='bms-sub'>V/bat</div></div>"
           "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Capacidade</div>"
           "<div id='b_cap' class='bms-val'>--</div><div class='bms-sub'>Ah</div></div></div>"
           "<div class='grid3' style='margin-top:6px'>"
           "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>SOC</div>"
           "<div id='b_soc' class='bms-val'>--</div><div class='bms-sub'>%</div></div>"
           "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Ciclos</div>"
           "<div id='b_ciclos' class='bms-val'>--</div><div class='bms-sub'>BMS</div></div>"
           "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Ultima</div>"
           "<div id='b_ultima' class='bms-val' style='font-size:14px'>--</div>"
           "<div class='bms-sub'>seg atras</div></div></div></div>"
           "<h3>⚙️ Acoes da BMS</h3>"
           "<form method='POST' action='/bms'><input type='hidden' name='acao' value='reconectar'>"
           "<button type='submit' class='btn-g btn-sm'>▶ Conectar / Reconectar</button></form>"
           "<form method='POST' action='/bms' style='margin-top:6px'>"
           "<input type='hidden' name='acao' value='desconectar'>"
           "<button type='submit' class='btn-o btn-sm'>⏹ Desconectar</button></form>"
           "<form method='POST' action='/bms' style='margin-top:6px' onsubmit='return confirm(\"Remover BMS?\")'>"
           "<input type='hidden' name='acao' value='limpar'>"
           "<button type='submit' class='btn-r btn-sm'>🗑 Remover BMS</button></form>"
           "<form method='GET' action='/config'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>← Voltar</button></form>"
           "<script>"
           "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
           "function atualizarBMS(d){"
           "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#a5d6a7':'#ef9a9a';"
           "sT('b_estado','<span style=\"color:'+cBLE+'\">'+d.ble_str+'</span>');"
           "sT('b_falhas','<span style=\"color:'+(d.ble_falhas>3?'#ef5350':'#a5d6a7')+'\">'+d.ble_falhas+'</span>');"
           "var cI=d.bms_corr>0.2?'#43a047':d.bms_corr<-0.2?'#ef6c00':'#fdd835';"
           "sT('b_corr','<span style=\"color:'+cI+'\">'+Math.abs(d.bms_corr).toFixed(2)+'A</span>');"
           "var dS=d.bms_corr>0.2?'↑ Carg':d.bms_corr<-0.2?'↓ Desc':'↔ Parado';"
           "sT('b_dir','<span style=\"color:'+cI+'\">'+dS+'</span>');"
           "sT('b_tensao',d.bms_tensao.toFixed(2));sT('b_cap',d.bms_cap.toFixed(2));"
           "var cS=d.bms_soc>=50?'#43a047':d.bms_soc>=25?'#ef6c00':'#c62828';"
           "sT('b_soc','<span style=\"color:'+cS+'\">'+d.bms_soc.toFixed(0)+'</span>');"
           "sT('b_ciclos',d.bms_ciclos);sT('b_ultima',d.bms_t_seg);}"
           "function aD(){var x=new XMLHttpRequest();"
           "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
           "try{atualizarBMS(JSON.parse(x.responseText));}catch(e){}}};"
           "x.open('GET','/dados',true);x.send();}"
           "setInterval(aD,3000);aD();</script>");
    server.sendContent(c);
    paginaFim(true);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 36 — BATERIA (banco + proteção)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleBateria() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config_banco") {
            if (!aplicarFormBanco()) msg = "<div class='er'>Modo invalido (1-4)</div>";
            else {
                char msgBuf[110];
                snprintf(msgBuf, sizeof(msgBuf), "✓ Banco salvo: Modo%d | %dS%dP (%d baterias)",
                         banco.modo, banco.nS, banco.nP, banco.nS * banco.nP);
                msg = "<div class='ok'>" + String(msgBuf) + "</div>";
            }
        } else if (acao == "protecao") {
            String s = server.arg("reserva");
            s.trim(); s.replace(',', '.');
            float r = s.toFloat();
            if (s.length() == 0 || r < 0.0f || r > 50.0f) {
                msg = "<div class='er'>Reserva: 0 a 50%</div>";
            } else {
                protecao.reservaPct = r;
                salvarConfigProtecao();
                atualizarProtecao();
                snprintf(tempBuf, sizeof(tempBuf), "Reserva bateria: %.0f%%", r);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>✓ Reserva salva: " + String((int)r) + "%</div>";
            }
        }
    }

    paginaInicio("RRFS Energia - Bateria");
    String c = msg;
    EstimativaTempo est = calcularTempoRestante();

    const char* modoNome = "";
    if (banco.modo == 1) modoNome = "UNICO";
    else if (banco.modo == 2) modoNome = "PARALELO";
    else if (banco.modo == 3) modoNome = "SERIE";
    else if (banco.modo == 4) modoNome = "SERIE+PARALELO";

    c += F("<h3>🔋 Status Atual do Banco</h3><div class='banco-card'>");
    addf(c, "<div class='banco-title'>Modo %d: %s | %dS%dP (%d baterias)</div>",
         banco.modo, modoNome, banco.nS, banco.nP, banco.nS * banco.nP);
    addf(c,
         "<div class='grid4' style='margin-top:8px'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Tensao</div>"
         "<div style='font-size:16px;font-weight:bold;color:#4fc3f7'>%.2fV</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>Corrente</div>"
         "<div style='font-size:16px;font-weight:bold;color:#43a047'>%.2fA</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#546e7a'>Capacidade</div>"
         "<div style='font-size:16px;font-weight:bold;color:#fdd835'>%.0fAh</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#90a4ae'>Potencia</div>"
         "<div style='font-size:16px;font-weight:bold;color:#ce93d8'>%.0fW</div></div></div></div>",
         dadosBanco.tensaoTotal, dadosBanco.correnteTotal,
         dadosBanco.capacidadeTotal, dadosBanco.potencia);
    server.sendContent(c);
    c = "";

    c += F("<h3>⚡ Protecao da Bateria</h3>");
    if (protecao.ativa)
        c += F("<div class='al'>⚠️ PROTECAO ATIVA: SOC abaixo da reserva. "
               "Inversores e iluminacao externa desligados.</div>");

    addf(c,
         "<div class='nf' style='font-size:11px'>Ao chegar na reserva o sistema desliga os "
         "<b>Inversores</b> e a <b>Iluminacao Externa</b>. Religa quando o SOC passar de "
         "<b>%.0f%%</b> (reserva + %.0f%%).</div>",
         protecao.reservaPct + PROT_HISTERESE_PCT, PROT_HISTERESE_PCT);

    if (bmsFresca()) {
        addf(c,
             "<div class='sc' style='text-align:center'>"
             "<div class='lbl'>Energia utilizavel ate a reserva</div>"
             "<div class='val' style='color:#4fc3f7'>%.1f Ah</div>"
             "<div class='sub'>SOC %.0f%% | reserva %.0f%% | banco %.0f Ah</div></div>",
             est.ahDisp, dadosBMS.soc, protecao.reservaPct, dadosBanco.capacidadeTotal);
    } else {
        c += F("<div class='wn' style='font-size:11px'>BMS sem dados: protecao inativa.</div>");
    }

    c += F("<form method='POST' action='/bateria'>"
           "<input type='hidden' name='acao' value='protecao'>"
           "<label>Reserva de protecao (% da bateria que nao sera usada):</label>");
    addf(c, "<input type='number' name='reserva' min='0' max='50' step='1' required value='%.0f'>",
         protecao.reservaPct);
    c += F("<button type='submit' class='btn-g'>💾 Salvar Reserva</button></form>"
           "<h3>⚙️ Configurar Banco</h3>");
    server.sendContent(c);
    c = "";

    campoBanco(c, "/bateria");
    c += F("<button type='submit' class='btn-g' style='margin-top:16px'>💾 Salvar Configuracao</button></form>");
    server.sendContent(c);
    c = "";

    bool bmsOK = bmsFresca() && (millis() - dadosBMS.tUltimaDado < 15000UL);
    c += F("<h3>📡 BMS Unitaria (Dados Brutos)</h3>");
    addf(c,
         "<div class='bms-card'><div class='grid3'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Corrente</div><div class='bms-val'>%.2fA</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Tensao</div><div class='bms-val'>%.2fV</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Capacidade</div><div class='bms-val'>%.0fAh</div></div>"
         "</div>",
         dadosBMS.corrente, dadosBMS.tensao, dadosBMS.capacidadeAh);
    addf(c,
         "<div class='grid3' style='margin-top:6px'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>SOC</div><div class='bms-val'>%.0f%%</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Ciclos</div><div class='bms-val'>%d</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>BLE</div>"
         "<div class='bms-val' style='color:%s'>%s</div></div></div></div>",
         dadosBMS.soc, (int)dadosBMS.ciclos,
         bmsOK ? "#a5d6a7" : "#ef9a9a", bmsOK ? "OK" : "OFF");
    c += FPSTR(JS_MOSTRAR_CAMPOS);
    server.sendContent(c);
    paginaFim(true);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 37 — CONTROLE
// ═══════════════════════════════════════════════════════════════════════════════════

static void addSelectInversao(String& c, const char* rotulo, const char* nome, bool inv) {
    c += "<label>"; c += rotulo; c += "</label><select name='"; c += nome; c += "'>";
    c += inv
        ? F("<option value='0'>Normal</option><option value='1' selected>Invertido</option>")
        : F("<option value='0' selected>Normal</option><option value='1'>Invertido</option>");
    c += F("</select>");
}

static void addFormLimiar(String& c, const char* titulo, const char* nota,
                          const char* acao, int valor, const char* label, const char* hint) {
    c += "<h3>"; c += titulo; c += "</h3>";
    if (nota && nota[0]) { c += "<div class='nf' style='font-size:11px'>"; c += nota; c += "</div>"; }
    c += "<form method='POST' action='/controle'><input type='hidden' name='acao' value='";
    c += acao; c += "'><label>"; c += label; c += "</label>";
    addf(c, "<input type='number' name='%s' min='50' max='2000' step='10' required value='%d'>", acao, valor);
    c += "<div class='nf' style='font-size:11px'>"; c += hint; c += "</div>";
    c += F("<button type='submit' class='btn-g'>💾 Salvar</button></form>");
}

static bool salvarLimiarSinalForm(const char* campo, int& destino, const char* logmsg, String& msg) {
    int lim = server.arg(campo).toInt();
    if (lim < 50 || lim > 2000) { msg = "<div class='er'>Limiar 50-2000mV</div>"; return false; }
    destino = lim;
    salvarLimiaresSinal();
    adicionarLog(logmsg);
    msg = "<div class='ok'>✓ Limiar salvo!</div>";
    return true;
}

void handleControle() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        auto exigeTeste = [&]() -> bool {
            if (!modoTeste) { msg = "<div class='er'>Ative o teste!</div>"; return false; }
            return true;
        };
        if (acao == "lim_lamp") {
            float ll = server.arg("lim_lamp").toFloat();
            if (ll<0||ll>95) msg="<div class='er'>SOC Lampada: 0-95%</div>";
            else {
                limiares.pctLampExt = ll;
                salvarLimiares();
                adicionarLog("Limiar lamp salvo");
                msg="<div class='ok'>✓ Salvo!</div>";
            }
        } else if (acao == "reles") {
            cfgInv.invertReleInvBaixa = (server.arg("inv_inva")=="1");
            cfgInv.invertReleLampExt  = (server.arg("inv_lamp")=="1");
            cfgInv.invertReleInvAlta  = (server.arg("inv_alta")=="1");
            salvarConfigInversores();
            if (estados.invBaixa) ligarInvBaixa(); else desligarInvBaixa();
            if (estados.lampExt)  ligarLampExt();  else desligarLampExt();
            if (estados.invAlta)  ligarInvAlta();  else desligarInvAlta();
            adicionarLog("Inversao reles salva");
            msg="<div class='ok'>✓ Inversao salva!</div>";
        } else if (acao == "lim_ac")       { salvarLimiarSinalForm("lim_ac",       limiarSinal.limiarAcMV,     "Limiar AC Baixa salvo", msg);
        } else if (acao == "lim_ac_alta")  { salvarLimiarSinalForm("lim_ac_alta",  limiarSinal.limiarAcAltaMV, "Limiar AC Alta salvo", msg);
        } else if (acao == "lim_casa")     { salvarLimiarSinalForm("lim_casa",     limiarSinal.limiarCasaMV,   "Limiar Sinal Casa salvo", msg);
        } else if (acao=="entrar_teste") {
            modoTeste=true; tInicioTeste=millis(); TIMEOUT_TESTE_MS=60000UL;
            adicionarLog("Modo teste ATIVO");
            msg="<div class='ok'>✓ Modo teste ativo 60s!</div>";
        } else if (acao=="sair_teste") {
            modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
            desligarInvBaixa(); desligarLampExt(); desligarInvAlta();
            maqBaixa.estado=INV_A_DESLIGADO; maqBaixa.tInicioEstado=millis();
            maqAlta.estado=INV_ALTA_DESLIGADO; maqAlta.tInicioEstado=millis();
            adicionarLog("Modo teste encerrado");
            msg="<div class='ok'>✓ Teste encerrado!</div>";
        }
        else if (acao=="teste_a_on")    { if (exigeTeste()) { ligarInvBaixa();    msg="<div class='ok'>InvBaixa ON</div>"; } }
        else if (acao=="teste_a_off")   { if (exigeTeste()) { desligarInvBaixa(); msg="<div class='ok'>InvBaixa OFF</div>"; } }
        else if (acao=="teste_alta_on") { if (exigeTeste()) { ligarInvAlta();     msg="<div class='ok'>InvAlta ON</div>"; } }
        else if (acao=="teste_alta_off"){ if (exigeTeste()) { desligarInvAlta();  msg="<div class='ok'>InvAlta OFF</div>"; } }
        else if (acao=="teste_lamp_on") { if (exigeTeste()) { ligarLampExt();     msg="<div class='ok'>Lamp ON</div>"; } }
        else if (acao=="teste_lamp_off"){ if (exigeTeste()) { desligarLampExt();  msg="<div class='ok'>Lamp OFF</div>"; } }
    }

    if (modoTeste && millis()-tInicioTeste>=TIMEOUT_TESTE_MS) {
        modoTeste=false; adicionarLog("Modo teste: timeout");
    }

    String c = msg;
    c+=F("<div class='nf' style='font-size:11px'>⚡ <b>Controle v32</b> — Inv.Baixa (automatico) + Inv.Alta (sob demanda da casa / excesso solar)</div>"
         "<h3>📊 Status ao Vivo</h3>"
         "<div id='ctrl_card_baixa' class='cg'><div class='cr'>"
         "<span class='cn'>⚡ Inv. Baixa</span><span id='ctrl_st_baixa' class='cs'>--</span></div>"
         "<div id='ctrl_sub_baixa' class='ci'>--</div></div>"
         "<div id='ctrl_card_alta' class='cg'><div class='cr'>"
         "<span class='cn'>⚡ Inv. Alta</span><span id='ctrl_st_alta' class='cs'>--</span></div>"
         "<div id='ctrl_sub_alta' class='ci'>--</div></div>"
         "<div id='ctrl_card_lamp' class='cg'><div class='cr'>"
         "<span class='cn'>💡 Lampada Ext</span><span id='ctrl_lamp' class='cs'>--</span></div></div>"
         "<div class='sc' style='margin-top:6px'><div class='lbl'>🔋 SOC Banco (BMS)</div>"
         "<div class='bb'><div id='ctrl_soc_bv' class='bv' style='width:0%'>0%</div></div>"
         "<div id='ctrl_soc_info' style='font-size:11px;color:#546e7a;text-align:center'>Aguardando BMS...</div></div>"
         "<h3>⚙️ Limiar Lampada</h3>"
         "<div class='nf' style='font-size:11px'>SOC% minimo (da BMS) para ligar a lampada externa.</div>"
         "<form method='POST' action='/controle'><input type='hidden' name='acao' value='lim_lamp'>"
         "<label>SOC% Minimo para Lampada:</label>");
    addf(c, "<input type='number' name='lim_lamp' min='0' max='95' step='1' required value='%.0f'>", limiares.pctLampExt);
    c+=F("<div class='nf' style='font-size:11px;margin-top:4px'>"
         "SOC atual: <b id='ctrl_soc_live'>--</b>% | Limiar lamp: <b id='ctrl_lim_l_live'>--</b>%</div>"
         "<button type='submit' class='btn-g'>💾 Salvar Limiar</button></form>");

    char hBuf[100];
    snprintf(hBuf,sizeof(hBuf),"Liga >%dmV | Desliga <%dmV | Confirma em 400ms",
             limiarSinal.limiarAcMV+AC_HISTERESE_LIGA_MV, limiarSinal.limiarAcMV-AC_HISTERESE_DESLIGA_MV);
    addFormLimiar(c, "📶 Limiar Sinal AC — Inv. Baixa (GPIO35)", "", "lim_ac",
                  limiarSinal.limiarAcMV, "Limiar AC (mV):", hBuf);
    snprintf(hBuf,sizeof(hBuf),"Liga >%dmV | Desliga <%dmV | Confirma em 5s",
             limiarSinal.limiarAcAltaMV+AC_HISTERESE_LIGA_MV, limiarSinal.limiarAcAltaMV-AC_HISTERESE_DESLIGA_MV);
    addFormLimiar(c, "📶 Limiar Sinal AC — Inv. Alta (GPIO36)",
                  "Este sinal e so diagnostico: confirma se a saida do Inv.Alta realmente energizou.",
                  "lim_ac_alta", limiarSinal.limiarAcAltaMV, "Limiar AC Alta (mV):", hBuf);
    snprintf(hBuf,sizeof(hBuf),"Pede >%dmV | Cancela <%dmV | Confirma em 5s",
             limiarSinal.limiarCasaMV+AC_HISTERESE_LIGA_MV, limiarSinal.limiarCasaMV-AC_HISTERESE_DESLIGA_MV);
    addFormLimiar(c, "🏠 Limiar Sinal da Casa (GPIO39)",
                  "Quando a casa manda esse sinal, o sistema tenta ligar o Inv.Alta (se a bateria permitir).",
                  "lim_casa", limiarSinal.limiarCasaMV, "Limiar Sinal Casa (mV):", hBuf);
    server.sendContent("");   // (nada: pagina e enviada ao final com gerarPagina)

    c+=F("<h3>🔄 Inversao de Reles</h3>"
         "<form method='POST' action='/controle'><input type='hidden' name='acao' value='reles'>");
    addSelectInversao(c, "Rele Inv. Baixa (GPIO26):", "inv_inva", cfgInv.invertReleInvBaixa);
    addSelectInversao(c, "Rele Inv. Alta (GPIO25):",  "inv_alta", cfgInv.invertReleInvAlta);
    addSelectInversao(c, "Rele Lamp. Ext (GPIO33):",  "inv_lamp", cfgInv.invertReleLampExt);
    c+=F("<button type='submit' class='btn-o'>💾 Salvar Inversao</button></form><h3>🧪 Teste de Reles</h3>");

    if (modoTeste) {
        unsigned long tr=(millis()-tInicioTeste>=TIMEOUT_TESTE_MS)?0
            :(TIMEOUT_TESTE_MS-(millis()-tInicioTeste))/1000UL;
        char tBuf[60];
        snprintf(tBuf,sizeof(tBuf),"⚠️ MODO TESTE — Timeout em %lus",tr);
        c+="<div class='al'>"+String(tBuf)+"</div>";
        c+=F("<form method='POST' action='/controle'>"
             "<input type='hidden' name='acao' value='sair_teste'>"
             "<button type='submit' class='btn-r'>⏹ Sair do Teste</button></form>");

        auto btnTeste=[&](const char* ico,const char* nome,const char* acOn,const char* acOff,bool on){
            String cor=on?"#43a047":"#546e7a";
            c+="<div class='cg' style='border-color:"+cor+";margin-top:8px'><div class='cr'>"
               "<span class='cn'>"+String(ico)+" "+String(nome)+"</span>"
               "<span class='cs' style='color:"+cor+"'>"+(on?"ON":"OFF")+"</span></div>"
               "<div class='grid2' style='gap:6px;margin-top:8px'>"
               "<form method='POST' action='/controle' style='margin:0'>"
               "<input type='hidden' name='acao' value='"+String(acOn)+"'>"
               "<button type='submit' class='btn-g btn-sm'>↑ LIGAR</button></form>"
               "<form method='POST' action='/controle' style='margin:0'>"
               "<input type='hidden' name='acao' value='"+String(acOff)+"'>"
               "<button type='submit' class='btn-r btn-sm'>↓ DESLIGAR</button></form></div></div>";
        };
        btnTeste("⚡","Inv. Baixa (GPIO26)","teste_a_on","teste_a_off",estados.invBaixa);
        btnTeste("⚡","Inv. Alta (GPIO25)","teste_alta_on","teste_alta_off",estados.invAlta);
        btnTeste("💡","Lampada (GPIO33)","teste_lamp_on","teste_lamp_off",estados.lampExt);
    } else {
        c+=F("<div class='nf' style='font-size:11px'>⚠️ Monitor automatico suspenso durante o teste.</div>"
             "<form method='POST' action='/controle'>"
             "<input type='hidden' name='acao' value='entrar_teste'>"
             "<button type='submit' class='btn-o'>▶ Ativar Modo Teste (60s)</button></form>");
    }

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function sBc(id,c){var e=document.getElementById(id);if(e)e.style.borderColor=c;}"
    "function atualizarCtrl(d){"
    "var cBaixa=d.inva?'#fdd835':'#546e7a';sBc('ctrl_card_baixa',cBaixa);"
    "sT('ctrl_st_baixa','<span style=\"color:'+cBaixa+'\">'+(d.inva?'• ON':'○ OFF')+'</span>');"
    "sT('ctrl_sub_baixa',d.nome_baixa+' | '+d.tempo_baixa+'s | Ligou:'+d.lig_baixa+' | F:'+d.fal_baixa);"
    "var cAlta=d.inv_alta?'#fdd835':'#546e7a';sBc('ctrl_card_alta',cAlta);"
    "sT('ctrl_st_alta','<span style=\"color:'+cAlta+'\">'+(d.inv_alta?'• ON':'○ OFF')+'</span>');"
    "sT('ctrl_sub_alta',d.nome_alta+' | '+d.tempo_alta+'s | Ligou:'+d.lig_alta+' | F:'+d.fal_alta+' | '+(d.casa_pede?'🏠 Casa pediu':'🏠 sem pedido'));"
    "var cL=d.le?'#fdd835':'#546e7a';sBc('ctrl_card_lamp',cL);"
    "sT('ctrl_lamp','<span style=\"color:'+cL+'\">'+(d.le?'• ON':'○ OFF')+'</span>');"
    "var soc=d.soc;var cor=soc>=50?'#2e7d32':soc>=25?'#ef6c00':'#c62828';"
    "var bv=document.getElementById('ctrl_soc_bv');"
    "if(bv){bv.style.width=Math.min(100,soc)+'%';bv.style.background=cor;bv.textContent=Math.round(soc)+'%';}"
    "sT('ctrl_soc_info',d.bms_valido?'SOC: '+soc.toFixed(0)+'% | Ciclos: '+d.ciclos:'BMS desconectada');"
    "sT('ctrl_soc_live',soc.toFixed(0));sT('ctrl_lim_l_live',d.lim_lamp.toFixed(0));}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarCtrl(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS Energia - Controle",c));
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 38 — PREVISÃO SOLAR (página)
// ═══════════════════════════════════════════════════════════════════════════════════

void handlePrevisao() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "config_prev") {
            String sLat = server.arg("lat"), sLon = server.arg("lon");
            String sPkw = server.arg("painel_kw"), sCtl = server.arg("ctrl_a");
            sLat.trim(); sLon.trim(); sPkw.trim(); sCtl.trim();
            sLat.replace(',', '.'); sLon.replace(',', '.');
            sPkw.replace(',', '.'); sCtl.replace(',', '.');

            float lat = sLat.toFloat(), lon = sLon.toFloat();
            float pkw = sPkw.toFloat();
            float ctl = sCtl.length() ? sCtl.toFloat() : 0.0f;
            bool ativ = (server.arg("ativa") == "1");

            if (sLat.length() == 0 || sLon.length() == 0) msg = "<div class='er'>Informe latitude e longitude</div>";
            else if (pkw <= 0 || pkw > 100)   msg = "<div class='er'>Potencia 0.1-100kW</div>";
            else if (lat < -90 || lat > 90)   msg = "<div class='er'>Latitude invalida</div>";
            else if (lon < -180 || lon > 180) msg = "<div class='er'>Longitude invalida</div>";
            else if (ctl < 0 || ctl > 500)    msg = "<div class='er'>Controlador: 0 a 500 A</div>";
            else {
                cfgPrevisao.latitude = lat; cfgPrevisao.longitude = lon;
                cfgPrevisao.painelKW = pkw; cfgPrevisao.controladorA = ctl;
                cfgPrevisao.ativa = ativ;   cfgPrevisao.configurada = true;
                salvarConfigPrevisao();
                if (ativ && WiFi.status() == WL_CONNECTED) { previsao.valido = false; tUltimaPrevisao = 0; }
                adicionarLog("Prev config salva");
                msg = "<div class='ok'>✓ Configurado!</div>";
            }
        } else if (acao == "buscar_agora") {
            previsao.valido = false; tUltimaPrevisao = 0; esp_task_wdt_reset();
            msg = buscarPrevisaoOpenMeteo()
                ? "<div class='ok'>✓ Atualizado!</div>"
                : "<div class='er'>✗ Falha. Verifique WiFi.</div>";
        } else if (acao == "reset_calib_prev") {
            cfgPrevisao.fatorCalib = PREV_FATOR_INICIAL; salvarConfigPrevisao();
            estPrev.err = estPrev.prev = estPrev.real = 0;
            estPrev.n = 0;
            estPrev.hErr = estPrev.hPrev = estPrev.hReal = 0;
            estPrev.hN = 0;
            memset(&estMod, 0, sizeof(estMod));
            for (int m = 0; m < PREV_MODELOS; m++) estMod.peso[m] = 1.0f;
            prefs.begin("rrfs_prev", false);
            prefs.putFloat("st_err", 0); prefs.putFloat("st_prev", 0); prefs.putFloat("st_real", 0);
            prefs.putUShort("st_n", 0); prefs.putBytes("mstat", &estMod, sizeof(estMod));
            prefs.end();
            if (previsao.valido) calcularAhPrevisto();
            adicionarLog("Calib prev reset"); msg = "<div class='ok'>✓ Reset OK</div>";
        }
    }

    paginaInicio("RRFS Energia - Previsao");
    String c = msg;
    EstimativaTempo est = calcularTempoRestante();

    c += F("<h3>🔋 Energia da Bateria Agora</h3>");
    if (bmsFresca()) {
        addf(c,
             "<div class='grid2'>"
             "<div class='pd'><div class='dt'>Utilizavel ate a reserva</div>"
             "<div id='pv_disp' class='dv'>%.1f Ah</div>"
             "<div class='ds'>SOC %.0f%% | reserva %.0f%%</div></div>"
             "<div class='pd'><div class='dt' id='pv_t_lbl'>Tempo</div>"
             "<div id='pv_tempo' class='dv'>--</div>"
             "<div id='pv_t_sub' class='ds'>--</div></div></div>",
             est.ahDisp, dadosBMS.soc, protecao.reservaPct);
    } else {
        c += F("<div class='wn'>BMS sem dados</div>");
    }
    server.sendContent(c);
    c = "";

    if (cfgPrevisao.configurada && cfgPrevisao.ativa) {
        c += F("<h3>🎯 Precisao da Previsao</h3>"
        "<div class='pd' style='border-color:#01579b;background:#0a1a2a'>"
        "<div class='dt' style='color:#81d4fa'>Taxa de acerto (ultimos ~3 dias)</div>"
        "<div id='pa_acc' class='dv' style='color:#e1f5fe'>--</div>"
        "<div class='bb'><div id='pa_bar' class='bv' style='width:0%'></div></div>"
        "<div id='pa_bias' class='ds' style='font-size:11px;line-height:1.5'>--</div>"
        "<div id='pa_hoje' class='ds' style='font-size:11px;line-height:1.5;margin-top:4px'>--</div>"
        "<div id='pa_mod' class='ds' style='font-size:11px;line-height:1.6;margin-top:6px;color:#90caf9'>--</div>"
        "<div class='ds' style='margin-top:6px;line-height:1.5'>"
        "O painel deve gerar IGUAL OU ACIMA do previsto. Horas em que a bateria esta cheia "
        "(SOC &ge; 92%) ou o excesso solar esta ativo nao contam: o painel so entrega o que a casa consome. "
        "Os 8 modelos meteorologicos ganham peso conforme acertam.</div></div>");
        server.sendContent(c);
        c = "";
    }

    c += F("<h3>⚙️ Configurar</h3>"
    "<form method='POST' action='/previsao'>"
    "<input type='hidden' name='acao' value='config_prev'>"
    "<div class='nf' style='font-size:11px'>No Google Maps, clique com o botao direito "
    "no local do painel e toque nas coordenadas para copiar. Cole no campo Latitude.</div>"
    "<button type='button' class='btn-s btn-sm' onclick='abrirMapa()'>📍 Abrir Google Maps</button>"
    "<label>Latitude:</label>");
    addf(c, "<input type='text' inputmode='decimal' autocomplete='off' name='lat' id='lat' required value='%.6f'>", cfgPrevisao.latitude);
    c += F("<label>Longitude:</label>");
    addf(c, "<input type='text' inputmode='decimal' autocomplete='off' name='lon' id='lon' required value='%.6f'>", cfgPrevisao.longitude);
    c += F("<label>Potencia dos paineis (kW):</label>");
    addf(c, "<input type='text' inputmode='decimal' name='painel_kw' required value='%.2f'>",
         cfgPrevisao.painelKW > 0 ? cfgPrevisao.painelKW : 1.0f);
    c += F("<label>Controlador de carga (A) - 0 = sem limite:</label>");
    addf(c, "<input type='text' inputmode='decimal' name='ctrl_a' value='%.0f'>", cfgPrevisao.controladorA);

    float vB = (dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;
    if (cfgPrevisao.controladorA > 0)
        addf(c, "<div class='nf' style='font-size:11px'>Limite do controlador: %.0f A x %.1f V = <b>%.0f W</b>.</div>",
             cfgPrevisao.controladorA, vB, cfgPrevisao.controladorA * vB);
    else
        c += F("<div class='nf' style='font-size:11px'>Sem limite: a previsao usa so a potencia dos paineis.</div>");

    c += F("<label>Ativar:</label><select name='ativa'>");
    c += cfgPrevisao.ativa
        ? F("<option value='1' selected>Sim</option><option value='0'>Nao</option>")
        : F("<option value='1'>Sim</option><option value='0' selected>Nao</option>");
    c += F("</select>");
    addf(c, "<div class='nf' style='font-size:11px'>Fator calibracao: %.3f</div>", cfgPrevisao.fatorCalib);
    c += F("<button type='submit' class='btn-g'>💾 Salvar</button></form><h3>☀️ Status</h3>");
    server.sendContent(c);
    c = "";

    if (!cfgPrevisao.configurada) c += F("<div class='wn'>Nao configurada</div>");
    else if (!cfgPrevisao.ativa)  c += F("<div class='wn'>Desativada</div>");
    else if (!previsao.valido) {
        c += F("<div class='wn'>Aguardando dados...</div>"
        "<form method='POST' action='/previsao'><input type='hidden' name='acao' value='buscar_agora'>"
        "<button type='submit' class='btn-o btn-sm'>↻ Buscar Agora</button></form>");
    } else {
        c += F("<form method='POST' action='/previsao'><input type='hidden' name='acao' value='buscar_agora'>"
        "<button type='submit' class='btn-o btn-sm'>↻ Atualizar</button></form>"
        "<form method='POST' action='/previsao' style='margin-top:4px'>"
        "<input type='hidden' name='acao' value='reset_calib_prev'>"
        "<button type='submit' class='btn-s btn-sm'>↻ Reset Calib</button></form>"
        "<h3>⏱️ Previsao</h3>"
        "<div class='grid2'>"
        "<div class='pd'><div class='dt'>Esta hora</div><div id='pv_h' class='dv'>--</div></div>"
        "<div class='pd'><div class='dt'>Restante hoje</div><div id='pv_r' class='dv'>--</div>"
        "<div class='ds'>Ate meia-noite</div></div></div>"
        "<h3>📅 7 Dias</h3>");
        server.sendContent(c);
        c = "";

        struct tm tA; bool tT = obterTempoAtual(tA);
        float mx = 0;
        for (int x = 0; x < PREV_DIAS; x++) if (previsao.ahDia[x] > mx) mx = previsao.ahDia[x];
        for (int d = 0; d < PREV_DIAS; d++) {
            char nd[16];
            if (d == 0)      snprintf(nd, sizeof(nd), "Hoje");
            else if (d == 1) snprintf(nd, sizeof(nd), "Amanha");
            else if (tT) {
                struct tm td = tA; td.tm_mday += d; mktime(&td);
                snprintf(nd, sizeof(nd), "%02d/%02d", td.tm_mday, td.tm_mon + 1);
            } else snprintf(nd, sizeof(nd), "Dia+%d", d);
            float pt = mx > 0 ? (previsao.ahDia[d] / mx) * 100.0f : 0;
            addf(c,
                 "<div style='margin:4px 0'>"
                 "<div style='display:flex;justify-content:space-between;font-size:11px;margin-bottom:2px'>"
                 "<span style='color:#90a4ae'>%s</span>"
                 "<span style='color:#fdd835;font-weight:bold'>%.1f Ah · %.0f Wh</span></div>"
                 "<div class='bb'><div class='bv' style='width:%d%%;background:%s'></div></div></div>",
                 nd, previsao.ahDia[d], previsao.ahDia[d] * vB, (int)pt, d == 0 ? "#0277bd" : "#1b5e20");
            server.sendContent(c);
            c = "";
        }
    }

    c += F(R"JS(<script>
function abrirMapa(){
 var la=document.getElementById('lat').value.replace(',','.').trim();
 var lo=document.getElementById('lon').value.replace(',','.').trim();
 var u='https://www.google.com/maps';
 if(la&&lo&&!isNaN(la)&&!isNaN(lo)&&(parseFloat(la)!==0||parseFloat(lo)!==0))
  u='https://www.google.com/maps/search/?api=1&query='+encodeURIComponent(la+','+lo);
 window.open(u,'_blank','noopener');
}
document.getElementById('lat').addEventListener('input',function(){
 var m=this.value.match(/(-?\d+[.]\d+)\s*[,;\s]\s*(-?\d+[.]\d+)/);
 if(m){this.value=m[1];document.getElementById('lon').value=m[2];}
});
function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}
function aD(){var x=new XMLHttpRequest();
 x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){
 try{var d=JSON.parse(x.responseText);
  var vb=d.v_banco>1?d.v_banco:12;
  sT('pv_h',d.ah_prev_h.toFixed(2)+' Ah · '+Math.round(d.ah_prev_h*vb)+' W');
  sT('pv_r',d.ah_prev_r.toFixed(2)+' Ah · '+Math.round(d.ah_prev_r*vb)+' Wh');
  sT('pv_disp',d.ah_disp.toFixed(1)+' Ah · '+Math.round(d.ah_disp*vb)+' Wh');
  if(d.tempo_val){
   var hm=d.tempo_h+'h '+(d.tempo_m<10?'0':'')+d.tempo_m+'min';
   if(d.tempo_carg){sT('pv_t_lbl','Para carregar 100%');sT('pv_tempo',hm);sT('pv_t_sub','faltam '+d.ah_falta.toFixed(1)+' Ah');}
   else{sT('pv_t_lbl','Ate a reserva');sT('pv_tempo',hm);sT('pv_t_sub','consumo '+Math.abs(d.ib).toFixed(1)+' A');}
  }else{sT('pv_t_lbl',d.prot?'PROTECAO ATIVA':'Tempo');sT('pv_tempo','--');sT('pv_t_sub','sem corrente para estimar');}
 }catch(e){}}};
 x.open('GET','/dados',true);x.send();}
setInterval(aD,5000);aD();
function cAcc(a){return a>=85?'#43a047':a>=70?'#9ccc65':a>=50?'#ef6c00':'#ef5350';}
function qAcc(a){return a>=85?'Excelente':a>=70?'Boa':a>=50?'Regular':'Fraca';}
var NM=['ECMWF','GFS','ICON','GEM','Meteo-France','JMA','UKMO','CMA'];
function aPS(){var x=new XMLHttpRequest();
 x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){
  try{var s=JSON.parse(x.responseText);
   if(!document.getElementById('pa_acc'))return;
   var mt='';
   if(s.mu){mt='Modo modelo unico (best match). Modelos multiplos indisponiveis agora.';}
   else{mt='Pesos dos modelos: ';for(var i=0;i<s.mw.length;i++){if(s.mo[i])mt+=NM[i]+' <b>'+Math.round(s.mw[i]*100)+'%</b> · ';}}
   sT('pa_mod',mt);
   if(s.acc<0){
    sT('pa_acc','Coletando dados...');
    sT('pa_bias','Horas avaliadas: '+s.n+' de 5 necessarias. Precisa de horas com sol e bateria abaixo de 92%.');
    sT('pa_hoje','');
   }else{
    var c=cAcc(s.acc);
    sT('pa_acc','<span style="color:'+c+'">'+s.acc.toFixed(0)+'% — '+qAcc(s.acc)+'</span>');
    var b=document.getElementById('pa_bar');
    if(b){b.style.width=s.acc+'%';b.style.background=c;b.textContent=s.acc.toFixed(0)+'%';}
    var dif=s.bias-100,tb;
    if(Math.abs(dif)<5)tb='Sem tendencia: real e previsto estao proximos.';
    else if(dif<0)tb='O real tem ficado '+Math.abs(dif).toFixed(0)+'% <b>abaixo</b> da previsao (o sistema ajusta o fator aos poucos).';
    else tb='O real tem ficado '+dif.toFixed(0)+'% <b>acima</b> da previsao (bom: previsao conservadora).';
    sT('pa_bias',tb+'<br>Horas avaliadas: '+s.n+' | Fator de calibracao: '+s.fator.toFixed(3));
    if(s.hn>=2)sT('pa_hoje','Hoje ('+s.hn+' h avaliadas): real '+s.h_real.toFixed(1)+' Ah x previsto '+s.h_prev.toFixed(1)+' Ah | acerto hora a hora '+s.hacc.toFixed(0)+'%');
    else sT('pa_hoje','Hoje: ainda poucas horas avaliadas.');
   }
  }catch(e){}}};
 x.open('GET','/prevstat',true);x.send();}
setInterval(aPS,10000);aPS();
</script>)JS");
    server.sendContent(c);
    paginaFim(true);
}
// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 39 — HISTÓRICO + PADRÕES DE CONSUMO (página)
//   /historico          -> status, padrões aprendidos, previsão, lista dos 30 dias
//   /historico?dia=N    -> detalhe completo do dia (N = 0 hoje, 1 ontem, ...)
// ═══════════════════════════════════════════════════════════════════════════════════

static void enviarSeTiver(String& c) {
    if (c.length() == 0) return;
    server.sendContent(c);
    c = "";
}

// Estado atual do detector de padrões
//  ps: 0 parado | 1 sobra solar (pausado) | 2 estabilizando | 3 corrente estável
//      4 subindo (confirmando) | 5 evento de padrão CONHECIDO | 6 evento NOVO (aprendendo)
//  pk: número do padrão (1..n) quando ps==5
static void padStatusAtual(int& ps, int& pk, float& extra, float& elapS) {
    ps = 0; pk = 0; extra = 0.0f; elapS = 0.0f;
    unsigned long agora = millis();
    if (!pad.iniciado || modoTeste || otaAtivo || !bmsFresca()) return;

    extra = pad.xs - pad.base;
    if (extra < 0.0f) extra = 0.0f;

    if (pad.sobraAnt)         { ps = 1; return; }
    if (agora < pad.ignoraAte) { ps = 2; return; }

    if (pad.emEvento) {
        elapS = (float)(agora - pad.tIni) / 1000.0f;
        float mediaAteAgora = (elapS > 1.0f) ? pad.somaAs / elapS : extra;
        int best = -1; float bd = 1e9f;
        for (int c = 0; c < padNCl; c++) {
            if (!padCl[c].valido) continue;
            float ta = fmaxf(0.5f, 0.20f * padCl[c].cA);
            float d1 = fabsf(extra         - padCl[c].cA) / ta;
            float d2 = fabsf(mediaAteAgora - padCl[c].cA) / ta;
            float da = (d1 < d2) ? d1 : d2;
            if (elapS > padCl[c].cDur * 2.5f + 30.0f) continue;   // durou demais p/ esse padrão
            if (da <= 1.0f && da < bd) { bd = da; best = c; }
        }
        if (best >= 0) { ps = 5; pk = best + 1; }
        else           { ps = 6; }
        return;
    }
    ps = (pad.tSub != 0) ? 4 : 3;
}

static void fmtDur(char* o, size_t n, float s) {
    uint32_t u = (uint32_t)s;
    if (u >= 3600)    snprintf(o, n, "%luh%02lum", (unsigned long)(u / 3600), (unsigned long)((u % 3600) / 60));
    else if (u >= 60) snprintf(o, n, "%lumin", (unsigned long)(u / 60));
    else              snprintf(o, n, "%lus", (unsigned long)u);
}

// horários típicos: horas com taxa >= 0,15 ocorrência/dia
static void padHorarios(const ClusterPad& k, char* out, size_t n) {
    out[0] = 0; size_t len = 0; int q = 0;
    for (int h = 0; h < 24 && q < 8; h++) {
        if (k.rate[h] < 0.15f) continue;
        int r = snprintf(out + len, n - len, "%02dh(~%d%%) ", h, (int)fminf(99.0f, k.rate[h] * 100.0f));
        if (r < 0 || (size_t)r >= n - len) break;
        len += r; q++;
    }
    if (!q) snprintf(out, n, "--");
}

// próximas vezes que costuma repetir (próximas 24 h)
static void padProximas(const ClusterPad& k, int hNow, char* out, size_t n) {
    out[0] = 0; size_t len = 0; int q = 0;
    for (int s = 0; s < 24 && q < 3; s++) {
        int ab = hNow + s, h = ab % 24;
        if (k.rate[h] < 0.15f) continue;
        int r = snprintf(out + len, n - len, "%s%s %02dh (~%d%%)", q ? ", " : "",
                         ab >= 24 ? "amanha" : "hoje", h, (int)fminf(99.0f, k.rate[h] * 100.0f));
        if (r < 0 || (size_t)r >= n - len) break;
        len += r; q++;
    }
    if (!q) snprintf(out, n, "sem previsao");
}

// Eventos de um dia (tabela com padrão, W e Wh)
static void htmlEventosDia(String& c, const DiaPadroes& p, float vB) {
    if (p.n == 0) { c += F("<div class='nf'>Nenhum evento de consumo forte registrado neste dia.</div>"); return; }
    c += F("<div style='overflow-x:auto'><table class='tb'><tr>"
    "<th>Inicio</th><th>Dur.</th><th>+A</th><th>W</th><th>Wh</th><th>Sol</th><th>Pad.</th></tr>");
    uint8_t ord[PAD_EV_DIA_MAX];
    for (uint8_t i = 0; i < p.n; i++) ord[i] = i;
    for (uint8_t i = 1; i < p.n; i++) {
        uint8_t k = ord[i]; int j = i - 1;
        while (j >= 0 && p.ev[ord[j]].iniMin > p.ev[k].iniMin) { ord[j+1] = ord[j]; j--; }
        ord[j+1] = k;
    }
    float somaWh = 0, somaAh = 0;
    for (uint8_t i = 0; i < p.n; i++) {
        const EvPadrao& e = p.ev[ord[i]];
        float media = e.mediaDA / 10.0f;
        float ah = media * e.durS / 3600.0f;
        somaAh += ah; somaWh += ah * vB;
        char dur[16], pid[8];
        fmtDur(dur, sizeof(dur), e.durS);
        int ci = padAchaCluster(media, (float)e.durS, false);
        if (ci >= 0) snprintf(pid, sizeof(pid), "#%d", ci + 1); else snprintf(pid, sizeof(pid), "novo");
        addf(c, "<tr><td>%02u:%02u</td><td>%s</td><td>+%.1f</td><td>%.0f</td><td>%.0f</td><td>%u%%</td><td>%s</td></tr>",
             e.iniMin / 60, e.iniMin % 60, dur, media, media * vB, ah * vB, e.solarPct, pid);
    }
    c += F("</table></div>");
    addf(c, "<div class='nf' style='font-size:11px'>Total dos eventos: <b>%.0f Wh</b> (%.1f Ah)%s</div>",
         somaWh, somaAh, p.perdidos ? " — alguns eventos fracos nao couberam no registro do dia" : "");
}

// Tabela hora a hora de um dia
static void htmlHorasDia(String& c, const RegistroDia& d, const DiaPadroes* p) {
    c += F("<div style='overflow-x:auto'><table class='tb'><tr>"
    "<th>Hora</th><th>Painel</th><th>Casa</th><th>Bat+</th><th>Bat-</th><th>Base</th></tr>");
    for (int h = 0; h < 24; h++) {
        const RegistroHora& r = d.horas[h];
        if (r.painelAh < 0.005f && r.casaAh < 0.005f && r.batCargAh < 0.005f && r.batDescAh < 0.005f) continue;
        char base[12] = "--";
        if (p && p->base[h] != 255) snprintf(base, sizeof(base), "%.1fA", p->base[h] / 10.0f);
        addf(c, "<tr><td>%02dh</td><td style='color:#fdd835'>%.1f</td><td style='color:#4fc3f7'>%.1f</td>"
        "<td style='color:#43a047'>%.1f</td><td style='color:#ef6c00'>%.1f</td><td>%s</td></tr>",
        h, r.painelAh, r.casaAh, r.batCargAh, r.batDescAh, base);
    }
    c += F("</table></div>");
}

static bool slotDiaValido(int i, RegistroDia& d, const DiaPadroes*& p) {
    int slot = ((int)histIndiceAtual - i + HIST_DIAS) % HIST_DIAS;
    bool ok = (i == 0) ? (d = histDiaAtual, true) : carregarDiaFlash(slot, d);
    if (!ok || !d.valido) return false;
    const DiaPadroes& q = padHist[slot];
    p = (q.ano == d.ano && q.mes == d.mes && q.dia == d.dia) ? &q : nullptr;
    return true;
}

// Padrões aprendidos: um cartão por padrão
static void htmlPadroesAprendidos(String& c, float vB, int hNow, bool tOk) {
    if (padNCl == 0) {
        c += F("<div class='nf' style='font-size:11px'>Nenhum equipamento forte recorrente identificado ainda.</div>");
        return;
    }
    int psAtual, pkAtual; float exA, elA;
    padStatusAtual(psAtual, pkAtual, exA, elA);

    float evDiaAh = 0;
    for (int k = 0; k < padNCl; k++) {
        const ClusterPad& cl = padCl[k];
        char dur[16], hs[96], nx[96];
        fmtDur(dur, sizeof(dur), cl.cDur);
        padHorarios(cl, hs, sizeof(hs));
        if (tOk) padProximas(cl, hNow, nx, sizeof(nx)); else snprintf(nx, sizeof(nx), "--");
        evDiaAh += cl.porDia * cl.eAh;
        bool agoraAtivo = (psAtual == 5 && pkAtual == k + 1);
        addf(c, "<div class='pd' style='border-color:%s;background:#12081a'>"
        "<div class='dt' style='color:#ce93d8'>🧩 Padrao %d — +%.1f A (~%.0f W) por ~%s%s</div>"
        "<div class='ds' style='font-size:11px;line-height:1.7;color:#b0bec5'>"
        "Pico: +%.0f A | Energia por vez: <b>%.2f Ah (%.0f Wh)</b><br>"
        "Frequencia: <b>%.1f vez(es)/dia</b> | Consumo no dia: <b>%.0f Wh</b> (%.1f Ah)<br>",
             agoraAtivo ? "#43a047" : "#7b1fa2", k + 1, cl.cA, cl.cA * vB, dur,
             agoraAtivo ? " — <span style='color:#a5d6a7'>EM ANDAMENTO</span>" : "",
             cl.picoA, cl.eAh, cl.eAh * vB,
             cl.porDia, cl.porDia * cl.eAh * vB, cl.porDia * cl.eAh);
        addf(c, "Horarios tipicos: %s<br>Proximas vezes: <b>%s</b><br>"
        "Ja ocorreu hoje: <b>%d</b> | Falta hoje: <b>%.1f</b> | Painel cobria: %.0f%%</div></div>",
        hs, nx, (int)cl.feitos, cl.restHoje, cl.solarPct);
        enviarSeTiver(c);
    }
    float baseDiaAh = 0;
    for (int h = 0; h < 24; h++) baseDiaAh += plano.baseOk[h] ? plano.baseP[h] : plano.consBase;
    addf(c, "<div class='sc' style='text-align:center'><div class='lbl'>Consumo estimado da casa por dia (sem lampada)</div>"
    "<div class='val' style='color:#4fc3f7'>%.0f Wh</div>"
    "<div class='sub'>Base %.0f Wh + padroes %.0f Wh | %.1f Ah no total</div></div>",
    (baseDiaAh + evDiaAh) * vB, baseDiaAh * vB, evDiaAh * vB, baseDiaAh + evDiaAh);
    addf(c, "<div class='nf' style='font-size:11px'>Aprendido com <b>%d dia(s)</b> e <b>%d h</b> monitoradas "
    "(dias recentes pesam mais; horas de sobra solar nao contam). Ainda esperado hoje: <b>%.1f Ah</b> (%.0f Wh).</div>",
         (int)padDias, (int)padHorasCob, plano.evRestAh, plano.evRestAh * vB);
}

static void historicoDetalhe(int idia) {
    RegistroDia d; const DiaPadroes* p = nullptr;
    paginaInicio("RRFS Energia - Dia");
    String c;
    float vB = (dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;
    if (idia < 0 || idia >= HIST_DIAS || idia >= (int)histTotalDias || !slotDiaValido(idia, d, p)) {
        c = F("<div class='er'>Dia nao encontrado</div>");
    } else {
        addf(c, "<h3>📅 %02d/%02d/%04d%s</h3>", d.dia, d.mes, d.ano, idia == 0 ? " (hoje)" : "");
        addf(c, "<div class='grid2'>"
        "<div class='sc' style='text-align:center'><div class='lbl'>☀️ Painel</div><div class='val' style='color:#fdd835'>%.1fAh</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>🏠 Casa</div><div class='val' style='color:#4fc3f7'>%.1fAh</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>↑ Bateria carregou</div><div class='val' style='color:#43a047'>%.1fAh</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>↓ Bateria cedeu</div><div class='val' style='color:#ef6c00'>%.1fAh</div></div></div>",
        d.totalPainelAh, d.totalCasaAh, d.totalBatCargAh, d.totalBatDescAh);
        if (p) {
            int hc = 0; for (int h = 0; h < 24; h++) if (p->cob & (1UL << h)) hc++;
            addf(c, "<div class='nf' style='font-size:11px'>🧩 Eventos fortes: <b>%d</b> | horas monitoradas: <b>%d</b> | "
            "flutuacao da base: ±%.1f A</div>", (int)p->n, hc, p->flut / 10.0f);

            c += F("<h3>🧩 Resumo por padrao</h3>");
            if (p->n == 0) {
                c += F("<div class='nf'>Sem eventos neste dia.</div>");
            } else {
                float cnt[PAD_CLUSTERS + 1], wh[PAD_CLUSTERS + 1];
                for (int i = 0; i <= PAD_CLUSTERS; i++) { cnt[i] = 0; wh[i] = 0; }
                for (int k = 0; k < p->n; k++) {
                    float media = p->ev[k].mediaDA / 10.0f;
                    int ci = padAchaCluster(media, (float)p->ev[k].durS, false);
                    if (ci < 0 || ci >= PAD_CLUSTERS) ci = PAD_CLUSTERS;
                    cnt[ci] += 1; wh[ci] += media * p->ev[k].durS / 3600.0f * vB;
                }
                c += F("<div style='overflow-x:auto'><table class='tb'><tr><th>Padrao</th><th>Vezes</th><th>Wh no dia</th></tr>");
                float tot = 0;
                for (int i = 0; i <= PAD_CLUSTERS; i++) {
                    if (cnt[i] <= 0) continue;
                    tot += wh[i];
                    if (i < PAD_CLUSTERS && i < padNCl)
                        addf(c, "<tr><td>#%d +%.1fA (~%.0fW)</td><td>%.0f</td><td>%.0f</td></tr>",
                             i + 1, padCl[i].cA, padCl[i].cA * vB, cnt[i], wh[i]);
                        else
                            addf(c, "<tr><td>Outros</td><td>%.0f</td><td>%.0f</td></tr>", cnt[i], wh[i]);
                }
                addf(c, "<tr><td><b>Total</b></td><td></td><td><b>%.0f</b></td></tr></table></div>", tot);
            }
            c += F("<h3>⚡ Eventos do dia</h3>");
            htmlEventosDia(c, *p, vB);
        } else {
            c += F("<h3>🧩 Padroes de consumo do dia</h3><div class='nf'>Sem dados de padroes para este dia.</div>");
        }
        c += F("<h3>🕐 Hora a hora (Ah)</h3><div class='nf' style='font-size:10px'>Base = consumo estavel da casa, sem eventos e sem a lampada.</div>");
        htmlHorasDia(c, d, p);
    }
    c += F("<form method='GET' action='/historico'><button type='submit' class='btn-s btn-sm' style='margin-top:10px'>← Voltar ao historico</button></form>");
    server.sendContent(c);
    paginaFim(true);
}

void handleHistorico() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    if (server.hasArg("dia")) { historicoDetalhe(server.arg("dia").toInt()); return; }

    paginaInicio("RRFS Energia - Historico");
    String c;
    struct tm t; bool tOk = obterTempoAtual(t);
    float vB = (dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;

    // ── Totais acumulados ───────────────────────────────────────────────────
    c += F("<h3>📈 Totais Acumulados</h3><div class='grid2'>");
    auto cT=[&](const char* ico,const char* nm,float ah,const char* cor){
        char buf[20]; snprintf(buf,sizeof(buf),"%.1fAh",ah);
        c+="<div class='sc' style='text-align:center'><div class='lbl'>"+String(ico)+" "+String(nm)+"</div>"
        "<div class='val' style='color:"+String(cor)+"'>"+String(buf)+"</div></div>";
    };
    cT("☀️","Painel", contadores.painelAh,  "#fdd835");
    cT("🏠","Casa",   contadores.casaAh,    "#4fc3f7");
    cT("↑","Bat+",    contadores.batCargAh, "#43a047");
    cT("↓","Bat-",    contadores.batDescAh, "#ef6c00");
    c += F("</div>");
    enviarSeTiver(c);

    // ── Hoje: status ao vivo ────────────────────────────────────────────────
    DiaPadroes& hj = padHoje();
    int hcob = 0; for (int h = 0; h < 24; h++) if (hj.cob & (1UL << h)) hcob++;

    c += F("<h3>🧩 Hoje — como o sistema conhece o consumo da casa</h3>");

    int ps, pk; float exA, elA;
    padStatusAtual(ps, pk, exA, elA);
    switch (ps) {
        case 0: c += F("<div class='nf' style='font-size:11px'>⚪ Detector aguardando BMS e relogio NTP.</div>"); break;
        case 1: c += F("<div class='nf' style='font-size:11px'>☀️ Sobra solar: o consumo vem dos paineis, nada e aprendido.</div>"); break;
        case 2: c += F("<div class='nf' style='font-size:11px'>⏳ Estabilizando apos lampada/inversor mudar de estado.</div>"); break;
        case 3: addf(c, "<div class='ok' style='font-size:12px'>🟢 Corrente estavel: %.1f A (%.0f W), flutuacao ±%.1f A</div>",
            pad.base, pad.base * vB, pad.dev); break;
        case 4: c += F("<div class='wn' style='font-size:12px'>🟡 Corrente subindo, confirmando se e um evento...</div>"); break;
        case 5: {
            char dur[16], dh[16]; fmtDur(dur, sizeof(dur), padCl[pk-1].cDur); fmtDur(dh, sizeof(dh), elA);
            addf(c, "<div class='nf' style='font-size:12px;background:#01579b'>🔵 Padrao conhecido <b>#%d</b> em andamento: +%.1f A (~%.0f W) ha %s "
            "(tipico: +%.1f A por ~%s). Consumo temporario ja previsto.</div>",
                 pk, exA, exA * vB, dh, padCl[pk-1].cA, dur); break;
        }
        case 6: {
            char dh[16]; fmtDur(dh, sizeof(dh), elA);
            addf(c, "<div class='nf' style='font-size:12px;background:#4a148c'>🟣 Corrente NOVA: +%.1f A (~%.0f W) ha %s. "
            "Nao bate com nenhum padrao salvo: sera registrada ao terminar e aprendida.</div>",
            exA, exA * vB, dh); break;
        }
    }

    if (!plano.usaPadroes)
        addf(c, "<div class='nf' style='font-size:11px'>🔎 Aprendendo a rotina: <b>%d de %d h</b> de dados. "
        "Ate completar, o planejador <b>nao</b> desliga a iluminacao externa.</div>",
        (int)padHorasCob, (int)PAD_MIN_HORAS_APR);
    addf(c, "<div class='grid3'>"
    "<div class='sc' style='text-align:center'><div class='lbl'>Base agora</div><div class='val' style='font-size:16px;color:#4fc3f7'>%.1fA</div><div class='sub'>%.0fW</div></div>"
    "<div class='sc' style='text-align:center'><div class='lbl'>Flutuacao</div><div class='val' style='font-size:16px'>±%.1fA</div></div>"
    "<div class='sc' style='text-align:center'><div class='lbl'>Eventos hoje</div><div class='val' style='font-size:16px;color:#ce93d8'>%d</div></div></div>",
    plano.consBase, plano.consBase * vB, pad.iniciado ? pad.dev : 0.0f, (int)hj.n);
    addf(c, "<div class='nf' style='font-size:10px'>Horas monitoradas hoje: %d | dias aprendidos: %d | horas totais: %d</div>",
         hcob, (int)padDias, (int)padHorasCob);

    c += F("<div class='lbl' style='color:#90a4ae;font-size:11px;margin-top:8px'>Eventos ja detectados hoje:</div>");
    enviarSeTiver(c);
    htmlEventosDia(c, hj, vB);
    enviarSeTiver(c);

    // ── Padrões aprendidos ──────────────────────────────────────────────────
    c += F("<h3>📚 Padroes aprendidos</h3>");
    if (!plano.usaPadroes)
        c += F("<div class='nf' style='font-size:11px'>Os padroes abaixo ainda sao parciais.</div>");
    enviarSeTiver(c);
    htmlPadroesAprendidos(c, vB, tOk ? t.tm_hour : 0, tOk);
    enviarSeTiver(c);

    // ── Previsão das próximas horas ─────────────────────────────────────────
    if (plano.usaPadroes && tOk) {
        c += F("<div class='lbl' style='color:#90a4ae;font-size:11px;margin-top:8px'>Previsao de consumo das proximas horas (base + eventos esperados):</div>"
        "<div style='overflow-x:auto'><table class='tb'><tr><th>Hora</th><th>Base</th><th>Eventos</th><th>Total</th><th>Solar prev.</th></tr>");
        int idx = getIndiceHoraAtual();
        for (int i = 0; i < 12; i++) {
            int ab = t.tm_hour + i; if (ab >= 72) break;
            int h = ab % 24;
            float base = plano.baseOk[h] ? plano.baseP[h] : plano.consBase;
            if (i == 0) base = plano.consBase;
            float ev = plano.evAbs[ab];
            float sol = (idx >= 0 && idx + i < PREV_TOTAL_HORAS) ? previsao.ahPrevisto[idx + i] : 0.0f;
            addf(c, "<tr><td>%02dh%s</td><td>%.1f</td><td style='color:#ce93d8'>%.1f</td><td style='color:#4fc3f7'>%.1f</td><td style='color:#fdd835'>%.1f</td></tr>",
                 h, ab >= 24 ? "+1" : "", base, ev, base + ev, sol);
        }
        c += F("</table></div><div class='nf' style='font-size:10px'>Valores em Ah. Nao inclui a lampada externa.</div>");
    }
    enviarSeTiver(c);

    // ── Lista dos 30 dias ───────────────────────────────────────────────────
    c += F("<h3>📅 Ultimos 30 dias</h3>");
    int maxD = histTotalDias > HIST_DIAS ? HIST_DIAS : histTotalDias;
    if (maxD == 0) c += F("<div class='nf'>Sem historico ainda</div>");
    enviarSeTiver(c);
    for (int i = 0; i < maxD; i++) {
        RegistroDia d; const DiaPadroes* p = nullptr;
        if (!slotDiaValido(i, d, p)) continue;
        addf(c, "<a class='dia' href='/historico?dia=%d'><div class='cg'><div class='cr'>"
        "<span class='cn'><b>%02d/%02d/%04d</b>%s</span>"
        "<span class='cs' style='font-size:11px'><span style='color:#fdd835'>☀️%.0f</span> "
        "<span style='color:#4fc3f7'>🏠%.0f</span> "
        "<span style='color:#ce93d8'>🧩%d</span> ›</span></div></div></a>",
        i, d.dia, d.mes, d.ano, i == 0 ? " (hoje)" : "",
             d.totalPainelAh, d.totalCasaAh, p ? (int)p->n : 0);
        enviarSeTiver(c);
    }
    c += F("<div style='font-size:10px;color:#546e7a'>☀️ painel (Ah) · 🏠 casa (Ah) · 🧩 eventos fortes. Toque num dia para ver tudo.</div>");

    // ── Gráfico tempo real (60s) ────────────────────────────────────────────
    c += F("<h3>📊 Tempo Real (60s)</h3>");
    int total = histSnapFull ? HIST_SNAP_PONTOS : histSnapIdx;
    if (total < 2) { c += F("<div class='nf'>Aguardando...</div>"); }
    else {
        float maxC = 0.5f;
        for (int i = 0; i < total; i++) {
            int idx = histSnapFull ? (histSnapIdx + i) % HIST_SNAP_PONTOS : i;
            if (histCorrPainel[idx] > maxC) maxC = histCorrPainel[idx];
            if (histCorrCasa[idx]   > maxC) maxC = histCorrCasa[idx];
            if (fabsf(histCorrBat[idx]) > maxC) maxC = fabsf(histCorrBat[idx]);
        }
        c += "<div style='height:80px;background:#111;border:1px solid #333;border-radius:4px'>"
        "<svg viewBox='0 0 " + String(total) + " 80' preserveAspectRatio='none' style='width:100%;height:100%'>";
        auto py = [&](float v)->float{ return 78.0f - (v / maxC) * 76.0f; };
        String pP = "M", pC = "M", pB = "M";
        for (int i = 0; i < total; i++) {
            int idx = histSnapFull ? (histSnapIdx + i) % HIST_SNAP_PONTOS : i;
            char pt[20];
            snprintf(pt, sizeof(pt), "%d,%.1f", i, py(histCorrPainel[idx])); if (i>0) pP += "L"; pP += pt;
            snprintf(pt, sizeof(pt), "%d,%.1f", i, py(histCorrCasa[idx]));   if (i>0) pC += "L"; pC += pt;
            snprintf(pt, sizeof(pt), "%d,%.1f", i, py(fabsf(histCorrBat[idx]))); if (i>0) pB += "L"; pB += pt;
        }
        c += "<path d='" + pP + "' fill='none' stroke='#fdd835' stroke-width='2'/>"
        "<path d='" + pC + "' fill='none' stroke='#4fc3f7' stroke-width='2'/>"
        "<path d='" + pB + "' fill='none' stroke='#ce93d8' stroke-width='1' stroke-dasharray='3,2'/></svg></div>";
        c += F("<div style='font-size:10px;color:#546e7a;margin-top:4px'>"
        "<span style='color:#fdd835'>━</span> Painel "
        "<span style='color:#4fc3f7;margin-left:8px'>━</span> Casa "
        "<span style='color:#ce93d8;margin-left:8px'>━</span> Banco</div>");
    }
    c += F("<script>setInterval(function(){location.reload();},30000);</script>");
    enviarSeTiver(c);
    paginaFim(true);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 40 — CONFIGURAÇÕES
// ═══════════════════════════════════════════════════════════════════════════════════

// prefs.clear() libera as entradas do NVS; sobrescrever blobs falha quando o NVS esta cheio.
static bool apagarHistoricoCompleto() {
    // 1) dias do historico (um namespace por slot)
    for (uint16_t i = 0; i < HIST_DIAS; i++) {
        char ns[12];
        snprintf(ns, sizeof(ns), "rh%03d", i);
        prefs.begin(ns, false);
        prefs.clear();
        prefs.end();
        esp_task_wdt_reset();
    }

    // 2) contadores acumulados
    prefs.begin("rrfs_cnt", false);
    prefs.clear();
    prefs.end();

    // 3) indice do historico
    prefs.begin("rrfs_hidx", false);
    prefs.clear();
    prefs.end();

    // 4) RAM
    contadores = {0, 0, 0, 0};
    memset(&acum, 0, sizeof(AcumHora));
    acum.horaAtual  = 255;
    acum.ultimoAcum = 0;
    histIndiceAtual = 0;
    histTotalDias   = 0;
    memset(&histDiaAtual, 0, sizeof(RegistroDia));
    memset(histCorrPainel, 0, sizeof(histCorrPainel));
    memset(histCorrCasa,   0, sizeof(histCorrCasa));
    memset(histCorrBat,    0, sizeof(histCorrBat));
    histSnapIdx  = 0;
    histSnapFull = false;
    diaCache = 0;                       // forca recalcular o dia

    // 5) grava indice novo (agora ha espaco livre) e contadores zerados
    salvarIndiceHistorico();
    salvarContadores();

    // 6) padroes de consumo (RAM + arquivo)
    padroesApagarTudo();

    // 7) confere na flash: nenhum dia pode ter sobrado
    int sobrou = 0;
    for (uint16_t i = 0; i < HIST_DIAS; i++) {
        RegistroDia tmp;
        if (carregarDiaFlash(i, tmp)) sobrou++;
    }
    prefs.begin("rrfs_hidx", true);
    uint16_t totalFlash = prefs.getUShort("total", 0xFFFF);
    prefs.end();

    snprintf(tempBuf, sizeof(tempBuf), "Hist apagado: sobraram %d dia(s), total flash %u",
             sobrou, (unsigned)totalFlash);
    adicionarLog(tempBuf);
    logRAM();
    return (sobrou == 0 && totalFlash == 0);
}

void handleConfig() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        String acao=server.arg("acao");
        if (acao=="senha") {
            prefs.begin("rrfs",true); String usr=prefs.getString("user",""); prefs.end();
            String pA=server.arg("pass_atual"),pN=server.arg("pass_nova"),pC=server.arg("pass_conf");
            if (!verificarLogin(usr,pA))  msg="<div class='er'>Senha atual incorreta</div>";
            else if (pN.length()<4)       msg="<div class='er'>Min 4 chars</div>";
            else if (pN!=pC)              msg="<div class='er'>Senhas nao conferem</div>";
            else { salvarCredenciais(usr,pN); adicionarLog("Senha alterada"); msg="<div class='ok'>✓ Alterada!</div>"; }
        } else if (acao=="tempo") {
            int fh=server.arg("fuso_h").toInt(),fm=server.arg("fuso_m").toInt();
            bool ntp=(server.arg("ntp_ativo")=="1");
            if (fh<-12||fh>14) msg="<div class='er'>Fuso -12 a +14</div>";
            else {
                cfgTempo.fusoHoras=fh; cfgTempo.fusoMinutos=fm; cfgTempo.ntpAtivo=ntp;
                if (!ntp) {
                    cfgTempo.manualAnno=server.arg("man_ano").toInt();
                    cfgTempo.manualMes =server.arg("man_mes").toInt();
                    cfgTempo.manualDia =server.arg("man_dia").toInt();
                    cfgTempo.manualHora=server.arg("man_hora").toInt();
                    cfgTempo.manualMin =server.arg("man_min").toInt();
                }
                salvarConfigTempo(); adicionarLog("Tempo salvo");
                if (ntp&&WiFi.status()==WL_CONNECTED){ ntpSincronizado=false; sincronizarNTP(); }
                msg="<div class='ok'>✓ Salvo!</div>";
            }
        } else if (acao=="reset_historico") {
            if (server.arg("confirmar")!="SIM") msg="<div class='er'>Confirmacao invalida</div>";
            else {
                bool ok = apagarHistoricoCompleto();
                msg = ok ? "<div class='ok'>✓ Historico e padroes apagados (confirmado na flash)!</div>"
                         : "<div class='er'>✗ Apagou na RAM, mas a flash ainda tem dados. Veja os logs.</div>";
            }
        }
    }
    String c=msg;
    c+=F("<h3>📊 Sistema</h3><div class='sc'>"
         "<div id='cfg_sys' class='val' style='font-size:12px'>Carregando...</div>"
         "<div id='cfg_hora' class='sub'></div></div>"
         "<h3>⏰ Data e Hora</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='tempo'><label>Fuso (h):</label>");
    addf(c,"<input type='number' name='fuso_h' min='-12' max='14' step='1' required value='%d'>",cfgTempo.fusoHoras);
    c+=F("<label>Minutos fuso:</label>");
    addf(c,"<input type='number' name='fuso_m' min='0' max='59' step='1' value='%d'>",cfgTempo.fusoMinutos);
    c+=F("<label>NTP:</label><select name='ntp_ativo' id='ntpSel' onchange='toggleNTP()'>");
    c+=cfgTempo.ntpAtivo
        ?F("<option value='1' selected>Sim</option><option value='0'>Nao-Manual</option>")
        :F("<option value='1'>Sim</option><option value='0' selected>Nao-Manual</option>");
    c+=F("</select><div id='manualTime' style='display:");
    c+=cfgTempo.ntpAtivo?"none":"block"; c+=F("'>");
    addf(c,"<label>Ano:</label><input type='number' name='man_ano' min='2024' max='2099' value='%d'>",cfgTempo.manualAnno);
    addf(c,"<label>Mes:</label><input type='number' name='man_mes' min='1' max='12' value='%d'>",cfgTempo.manualMes);
    addf(c,"<label>Dia:</label><input type='number' name='man_dia' min='1' max='31' value='%d'>",cfgTempo.manualDia);
    addf(c,"<label>Hora:</label><input type='number' name='man_hora' min='0' max='23' value='%d'>",cfgTempo.manualHora);
    addf(c,"<label>Minuto:</label><input type='number' name='man_min' min='0' max='59' value='%d'>",cfgTempo.manualMin);
    c+=F("</div><button type='submit' class='btn-g'>💾 Salvar Hora</button></form>"
         "<script>function toggleNTP(){"
         "document.getElementById('manualTime').style.display="
         "document.getElementById('ntpSel').value=='1'?'none':'block';}</script>"
         "<h3>🔐 Senha</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='senha'>"
         "<label>Atual:</label><input type='password' name='pass_atual' required>"
         "<label>Nova:</label><input type='password' name='pass_nova' required>"
         "<label>Confirmar:</label><input type='password' name='pass_conf' required>"
         "<button type='submit' class='btn-o'>Alterar</button></form>"
         "<h3>🗑️ Limpar Historico e Padroes</h3>"
         "<form method='POST' action='/config' onsubmit='return confirm(\"Apagar historico e padroes?\")'>"
         "<input type='hidden' name='acao' value='reset_historico'>"
         "<input type='hidden' name='confirmar' value='SIM'>"
         "<button type='submit' class='btn-r'>Apagar</button></form>"
         "<h3>⚙️ Acoes</h3>"
         "<form method='GET' action='/logs-salvos'><button type='submit' class='btn-s btn-sm'>📋 Logs</button></form>"
         "<form method='GET' action='/bms'><button type='submit' class='btn-b btn-sm' style='margin-top:6px'>📡 BMS Bluetooth</button></form>"
         "<form method='GET' action='/setup-wifi'><button type='submit' class='btn-s btn-sm' style='margin-top:6px'>📶 WiFi</button></form>"
         "<form method='GET' action='/ota'><button type='submit' class='btn-o btn-sm' style='margin-top:6px'>🚀 OTA</button></form>"
         "<form method='GET' action='/logout'><button type='submit' class='btn-s btn-sm' style='margin-top:6px'>🚪 Logout</button></form>"
         "<form method='POST' action='/reiniciar'><button type='submit' class='btn-r btn-sm' style='margin-top:6px'>↻ Reiniciar</button></form>"
         "<script>"
         "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
         "function aD(){var x=new XMLHttpRequest();"
         "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
         "try{var d=JSON.parse(x.responseText);"
         "sT('cfg_sys','v32 | RAM '+d.ram.toFixed(1)+'KB | Up '+d.uptime+'s | Banco '+d.nS+'S'+d.nP+'P | Cap '+d.cap_banco.toFixed(1)+'Ah'"
         "+(d.modo_teste?' | <span style=\"color:#ef6c00\">MODO TESTE</span>':''));"
         "sT('cfg_hora','⏰ '+d.data_hora+(d.ntp?' <span style=\"color:#43a047\">•NTP</span>':''));"
         "}catch(e){}}};x.open('GET','/dados',true);x.send();}"
         "setInterval(aD,5000);aD();</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS Energia - Config",c));
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 41 — LOGS (handlers)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleLogs() {
    if (!logado) { server.send(403, "application/json", "[]"); return; }
    if (!logsCarregados) carregarLogsPersistentes();
    String json="[";
    bool primeiro=true;
    uint16_t inicio=totalLogs>50?totalLogs-50:0;
    for (uint16_t num=inicio;num<totalLogs;num++) {
        uint8_t idx=num%MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        if (!primeiro) json+=",";
        primeiro=false; json+="\"";
        for (int ch=0;logs[idx][ch]&&ch<LOG_MAX_LEN;ch++) {
            char cc=logs[idx][ch];
            if(cc=='"')      json+="\\\"";
            else if(cc=='\\')json+="\\\\";
            else if(cc=='\n')json+=" ";
            else if(cc=='\r')continue;
            else json+=cc;
        }
        json+="\"";
    }
    json+="]";
    server.send(200,"application/json",json);
}

void handleLogsSalvos() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    if (!logsCarregados) carregarLogsPersistentes();

    uint16_t total;
    portENTER_CRITICAL(&logMux);
    total = totalLogs;
    portEXIT_CRITICAL(&logMux);

    paginaInicio("RRFS Energia - Logs");
    String c = F("<h3>📋 Logs Salvos</h3>");
    c += "<div class='nf'>Total gerado: <b>" + String(total) +
         "</b> | ultimos " + String(MAX_LOGS) +
         " (mais novo primeiro) | <a href='/logs-txt' style='color:#90caf9'>baixar .txt</a></div>"
         "<div class='lbx' style='height:400px'>";
    server.sendContent(c);
    c = "";

    for (uint16_t k = 0; k < MAX_LOGS && k < total; k++) {
        uint32_t n   = (uint32_t)total - k;
        uint8_t  pos = (n - 1) % MAX_LOGS;
        char copia[LOG_MAX_LEN];
        portENTER_CRITICAL(&logMux);
        memcpy(copia, logs[pos], LOG_MAX_LEN);
        portEXIT_CRITICAL(&logMux);
        copia[LOG_MAX_LEN - 1] = '\0';
        if (copia[0] == '\0') continue;
        c += "<div class='ll'>";
        htmlEsc(copia, c);
        c += "</div>";
        if (c.length() > 800) { server.sendContent(c); c = ""; }
    }
    c += F("</div><form method='GET' action='/config'>"
           "<button type='submit' class='btn-s btn-sm' style='margin-top:8px'>← Voltar</button></form>");
    server.sendContent(c);
    paginaFim(true);
}

void handleLogsTxt() {
    if (!logado) { server.send(403, "text/plain", "403"); return; }
    if (!logsCarregados) carregarLogsPersistentes();

    uint16_t total;
    portENTER_CRITICAL(&logMux);
    total = totalLogs;
    portEXIT_CRITICAL(&logMux);

    server.sendHeader("Content-Disposition", "attachment; filename=rrfs_logs.txt");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain; charset=utf-8", "");

    uint16_t qtd = (total > MAX_LOGS) ? MAX_LOGS : total;
    String c = "";
    for (uint16_t k = 0; k < qtd; k++) {
        uint32_t n   = (uint32_t)total - qtd + 1 + k;
        uint8_t  pos = (n - 1) % MAX_LOGS;
        char copia[LOG_MAX_LEN];
        portENTER_CRITICAL(&logMux);
        memcpy(copia, logs[pos], LOG_MAX_LEN);
        portEXIT_CRITICAL(&logMux);
        copia[LOG_MAX_LEN - 1] = '\0';
        if (copia[0] == '\0') continue;
        c += copia;
        c += "\n";
        if (c.length() > 800) { server.sendContent(c); c = ""; }
    }
    if (c.length()) server.sendContent(c);
    server.sendContent("");
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 42 — APIs JSON: /excesso /plano /prevstat
// ═══════════════════════════════════════════════════════════════════════════════════

void handleExcesso() {
    if (!logado) { server.send(403); return; }
    unsigned long agora = millis();
    auto restante = [&](unsigned long ini, unsigned long dur) -> long {
        unsigned long d = agora - ini;
        return (d >= dur) ? 0L : (long)((dur - d) / 1000UL);
    };
    long confS = 0, gracaS = 0, fimS = 0, coolS = 0, ativoS = 0;
    bool fimOn = false;
    if (excesso.fase == 1) confS = restante(excesso.tConf, EXC_CONFIRMA_MS);
    if (excesso.fase == 2) {
        gracaS = restante(excesso.tAtiv, EXC_GRACA_MS);
        ativoS = (long)((agora - excesso.tAtiv) / 1000UL);
        if (excesso.tFimConf) { fimOn = true; fimS = restante(excesso.tFimConf, EXC_QUEDA_CONFIRMA_MS); }
    }
    if (excesso.fase == 0 && excesso.tCool && excesso.motivo == 8)
        coolS = restante(excesso.tCool, excesso.coolMs);

    char buf[700];
    snprintf(buf, sizeof(buf),
             "{\"fase\":%d,\"motivo\":\"%s\",\"conf_s\":%ld,\"graca_s\":%ld,\"fim_s\":%ld,\"fim_on\":%s,"
             "\"cool_s\":%ld,\"ativo_s\":%ld,\"eventos\":%u,\"fim_ger\":%s,\"ultimo_fim\":\"%s\","
             "\"soc\":%.0f,\"ip\":%.2f,\"ib\":%.2f,\"alta_on\":%s,\"casa\":%s,\"lim_soc\":%.0f}",
             (int)excesso.fase, excMotivoTxt(excesso.motivo), confS, gracaS, fimS,
             fimOn ? "true" : "false", coolS, ativoS,
             (unsigned)excesso.eventos, excesso.fimGeracao ? "true" : "false", excesso.ultimoFim,
             dadosBMS.soc, sensores.correntePainel, sensores.correnteBateria,
             estados.invAlta ? "true" : "false", sinalCasaPresente() ? "true" : "false",
             (float)EXC_SOC_ENTRADA_PCT);
    server.send(200, "application/json", buf);
}

void handlePlano() {
    if (!logado) { server.send(403); return; }
    float vB = (dadosBanco.tensaoTotal > 1.0f) ? dadosBanco.tensaoTotal : 12.0f;
    bool analis = (plano.verPend != plano.verConf) ||
    (plano.analisandoAte != 0 && (long)(plano.analisandoAte - millis()) > 0);

    float faltaAh = plano.margemAh - plano.sem.nivelMin;
    if (faltaAh < 0.0f) faltaAh = 0.0f;

    int   idxP  = getIndiceHoraAtual();
    float prevA = (previsao.valido && idxP >= 0) ? previsao.ahPrevisto[idxP] : 0.0f;

    // status ao vivo do detector de padrões
    int psT, pkT; float peT, ptT;
    padStatusAtual(psT, pkT, peT, ptT);
    float thr = fmaxf(PAD_MIN_THR_A, PAD_K_DEV * pad.dev);
    float pcA = (pkT > 0 && pkT <= padNCl) ? padCl[pkT - 1].cA   : 0.0f;
    float pcD = (pkT > 0 && pkT <= padNCl) ? padCl[pkT - 1].cDur : 0.0f;

    char buf[2000];
    snprintf(buf, sizeof(buf),
             "{\"est\":%d,\"sm\":%d,\"bloq\":%s,\"analis\":%s,\"hist\":%s,\"vb\":%.2f,"
             "\"cons_a\":%.2f,\"lamp_a\":%.2f,\"margem\":%.1f,\"pico_h\":%d,\"pico_a\":%.2f,"
             "\"c_min\":%.1f,\"c_fh\":%d,\"c_fd\":%d,\"c_lh\":%d,\"c_ld\":%d,\"c_cons\":%.1f,\"c_ger\":%.1f,"
             "\"s_min\":%.1f,\"s_fh\":%d,\"s_fd\":%d,\"s_lh\":%d,\"s_ld\":%d,\"s_cons\":%.1f,\"s_ger\":%.1f,"
             "\"red_a\":%.2f,\"red_w\":%.0f,"
             "\"pad\":%s,\"pad_dias\":%d,\"pad_n\":%d,\"ev_rest\":%.1f,"
             "\"sobra\":%s,\"falta_ah\":%.1f,\"prev_a\":%.1f,\"soc\":%.0f,\"ip\":%.1f,\"ic\":%.1f,"
             "\"pad_h\":%d,\"pad_min\":%d,"
             "\"ps\":%d,\"pk\":%d,\"pe\":%.2f,\"pt\":%.0f,\"pbase\":%.2f,\"pdev\":%.2f,\"pthr\":%.2f,"
             "\"pcA\":%.2f,\"pcD\":%.0f,\"x_h\":%d,\"x_d\":%d}",
             plano.verConf, plano.semMotivo,
             plano.lampBloqueada ? "true" : "false",
             analis ? "true" : "false",
             plano.usaHistorico ? "true" : "false",
             vB,
             plano.consBase, plano.lampA, plano.margemAh, plano.horaPico, plano.picoA,
             plano.com.nivelMin, plano.com.fimH, plano.com.fimD, plano.com.faltaH, plano.com.faltaD,
             plano.com.cons, plano.com.ger,
             plano.sem.nivelMin, plano.sem.fimH, plano.sem.fimD, plano.sem.faltaH, plano.sem.faltaD,
             plano.sem.cons, plano.sem.ger,
             plano.reducaoA, plano.reducaoW,
             plano.usaPadroes ? "true" : "false", (int)padDias, (int)padNCl, plano.evRestAh,
             plano.sobraSolar ? "true" : "false", faltaAh, prevA,
             dadosBMS.soc, sensores.correntePainel, sensores.correnteCasa,
             (int)padHorasCob, (int)PAD_MIN_HORAS_APR,
             psT, pkT, peT, ptT, pad.iniciado ? pad.base : 0.0f, pad.iniciado ? pad.dev : 0.0f, thr,
             pcA, pcD, plano.com.excH, plano.com.excD);
    server.send(200, "application/json", buf);
}

void handlePrevStat() {
    if (!logado) { server.send(403); return; }
    carregarEstatPrev();
    float acc = -1.0f, bias = 0.0f, hacc = -1.0f;
    if (estPrev.n >= 5 && estPrev.prev > 0.01f) {
        acc = 100.0f * (1.0f - estPrev.err / estPrev.prev);
        if (acc < 0.0f) acc = 0.0f;
        if (acc > 100.0f) acc = 100.0f;
        bias = 100.0f * estPrev.real / estPrev.prev;
    }
    if (estPrev.hN >= 2 && estPrev.hPrev > 0.01f) {
        hacc = 100.0f * (1.0f - estPrev.hErr / estPrev.hPrev);
        if (hacc < 0.0f) hacc = 0.0f;
        if (hacc > 100.0f) hacc = 100.0f;
    }
    // pesos normalizados (soma = 1) dos modelos disponiveis
    float soma = 0;
    for (int m = 0; m < PREV_MODELOS; m++) if (previsao.modeloOk[m]) soma += estMod.peso[m];
    String mw = "[", mo = "[";
    for (int m = 0; m < PREV_MODELOS; m++) {
        if (m) { mw += ","; mo += ","; }
        mw += String((soma > 0 && previsao.modeloOk[m]) ? estMod.peso[m] / soma : 0.0f, 3);
        mo += previsao.modeloOk[m] ? "true" : "false";
    }
    mw += "]"; mo += "]";

    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"n\":%u,\"acc\":%.1f,\"bias\":%.0f,\"hn\":%u,\"hacc\":%.1f,"
             "\"h_real\":%.1f,\"h_prev\":%.1f,\"fator\":%.3f,\"mu\":%s,",
             (unsigned)estPrev.n, acc, bias, (unsigned)estPrev.hN, hacc,
             estPrev.hReal, estPrev.hPrev, cfgPrevisao.fatorCalib,
             previsao.modoUnico ? "true" : "false");
    String j = String(buf) + "\"mw\":" + mw + ",\"mo\":" + mo + "}";
    server.send(200, "application/json", j);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 43 — UTILITÁRIOS (reiniciar, 404)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleReiniciar() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    adicionarLog("Reinicio usuario");
    server.send(200,"text/html;charset=utf-8",
                gerarPagina("RRFS Energia - Reiniciando",F("<div class='ok'>✓ Reiniciando em 3s...</div>"),false));
    salvarLogsPendentes(true);
    padroesSalvarSeSujo(true);
    delay(3000);
    ESP.restart();
}

void handleNotFound() {
    String uri=server.uri();
    if (uri.indexOf("generate_204")>=0||uri.indexOf("gen_204")>=0||
        uri.indexOf("hotspot")>=0||uri.indexOf("ncsi.txt")>=0||
        uri.indexOf("connecttest")>=0) {
        if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302,"text/plain",""); }
        else server.send(204,"text/plain","");
        return;
    }
    if (uri=="/favicon.ico") { server.send(204); return; }
    if (modoAP) { server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else server.send(404,"text/plain","404");
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 44 — OTA (atualização de firmware)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleOTA() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    paginaInicio("RRFS Energia - OTA");

    server.sendContent(F("<h3>🚀 Atualizar Firmware</h3>"
    "<div class='nf'>Envie o arquivo <b>.ino.bin</b> "
    "(Sketch → Exportar binario compilado). "
    "NAO use <i>bootloader</i>, <i>merged</i> ou <i>partitions</i>.<br>"
    "Feche outras abas. O Bluetooth sera desligado.</div>"
    "<form id='fOTA'>"
    "<input type='file' id='fArq' name='firmware' accept='.bin' required "
    "style='background:#111;border:1px solid #444;border-radius:4px;padding:8px;"
    "color:#e0e0e0;width:100%;box-sizing:border-box'>"
    "<div id='prog' style='display:none;margin-top:8px'>"
    "<div class='bb'><div class='bv' id='pBar' style='width:0%;background:#0277bd'>0%</div></div>"
    "<div style='text-align:center;font-size:12px;color:#78909c;margin-top:4px' id='pTxt'>Preparando...</div></div>"
    "<button type='submit' class='btn-o' style='margin-top:10px' id='btnOTA'>🚀 Enviar</button></form>"
    "<form method='GET' action='/config'>"
    "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>← Voltar</button></form>"));

    server.sendContent(F(R"JS(<script>
var verAntes='';
function $(i){return document.getElementById(i);}
function st(t,cor,bar){$('pTxt').textContent=t;$('pTxt').style.color=cor||'#78909c';if(bar)$('pBar').style.background=bar;}
function falha(t){st(t,'#ef9a9a','#c62828');$('btnOTA').disabled=false;}
function sucesso(){
 $('pBar').style.width='100%';$('pBar').textContent='100%';
 st('✓ Atualizado! Reiniciado.','#a5d6a7','#2e7d32');
 setTimeout(function(){location.href='/';},4000);
}
function get(url,cb,to){
 var x=new XMLHttpRequest();x.timeout=to||3000;
 x.onreadystatechange=function(){if(x.readyState==4)cb(x.status,x.responseText);};
 x.open('GET',url,true);x.send();
}
function conferir(n){
 get('/ota-versao',function(s,t){
  if(s==200&&verAntes&&t!==verAntes){sucesso();return;}
  if(n<=0){falha(s==200?'A versao nao mudou: atualizacao falhou. Compile de novo.':
   'Sem resposta do sistema. Aguarde 1 minuto.');return;}
  setTimeout(function(){conferir(n-1);},2000);
 },3000);
}
function enviar(f){
 var fd=new FormData();fd.append('firmware',f);
 var x=new XMLHttpRequest();
 st('Enviando...','#78909c','#0277bd');
 x.upload.onprogress=function(e){
  if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);
   $('pBar').style.width=p+'%';$('pBar').textContent=p+'%';
   if(p>=100)st('Gravando...');}
 };
 x.onreadystatechange=function(){
  if(x.readyState!=4)return;
  if(x.status==200||x.status==0){st('Conferindo reinicio...','#ffe0b2');conferir(30);}
  else falha('Erro '+x.status+': falhou.');
 };
 x.open('POST','/ota-upload?size='+f.size,true);x.send(fd);
}
function esperar(f,n){
 get('/ota-estado',function(s,t){
  if(s==200&&t.indexOf('true')>=0){enviar(f);return;}
  if(n<=0){falha('A preparacao demorou. Reinicie o ESP32.');return;}
  setTimeout(function(){esperar(f,n-1);},700);
 },3000);
}
$('fOTA').onsubmit=function(e){
 e.preventDefault();
 var f=$('fArq').files[0];
 if(!f||!f.name.endsWith('.bin')){alert('Selecione um arquivo .bin');return;}
 if(/bootloader|merged|partitions/i.test(f.name)){alert('Arquivo errado. Use o .ino.bin');return;}
 $('prog').style.display='block';$('btnOTA').disabled=true;
 st('Preparando (desligando o Bluetooth)...','#ffe0b2','#0277bd');
 get('/ota-versao',function(s,t){
  verAntes=(s==200)?t:'';
  get('/ota-prep',function(s2){
   if(s2!=200){falha('Falha ao preparar: '+s2);return;}
   esperar(f,40);
  },5000);
 },3000);
};
</script>)JS"));

    server.sendContent(F("</div></body></html>"));
    server.sendContent("");
}

void otaPrepararBLE() {
    otaPrepSolicitado = false;
    adicionarLog("OTA: liberando BLE");

    otaAtivo          = true;
    otaOk             = false;
    otaErro           = false;
    otaUploadIniciado = false;
    tOtaUltimo        = millis();

    padroesSalvarSeSujo(true);     // v32: garante que os padroes nao se percam no reinicio

    if (bleClient && bleClient->isConnected()) bleClient->disconnect();

    unsigned long t0 = millis();
    while (millis() - t0 < 800UL) {
        esp_task_wdt_reset();
        server.handleClient();
        delay(20);
    }

    desconectarBMS();
    if (bleIniciado) {
        BLEDevice::deinit(true);
        bleIniciado = false;
    }
    bleClient      = nullptr;
    bleScan        = nullptr;
    bleDispositivo = nullptr;

    WiFi.setSleep(false);

    snprintf(tempBuf, sizeof(tempBuf), "OTA pronta RAM %.0fK", esp_get_free_heap_size() / 1024.0f);
    adicionarLog(tempBuf);

    tOtaUltimo = millis();
    otaPronto  = true;
}

void handleOTAPrep() {
    if (!logado) { server.send(403, "application/json", "{\"ok\":false}"); return; }
    if (!otaAtivo) otaPrepSolicitado = true;
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleOTAEstado() {
    if (!logado) { server.send(403, "application/json", "{\"pronto\":false}"); return; }
    server.send(200, "application/json", otaPronto ? "{\"pronto\":true}" : "{\"pronto\":false}");
}

void handleOTAVersao() {
    server.send(200, "text/plain", String(__DATE__) + " " + String(__TIME__));
}

void handleOTAUpload() {
    if (!logado) return;
    HTTPUpload& up = server.upload();
    char b[72];

    if (up.status == UPLOAD_FILE_START) {
        String fn = up.filename; fn.toLowerCase();
        if (!otaAtivo || !otaPronto) {
            adicionarLog("OTA: sem preparacao");
            otaErro = true;
            return;
        }
        if (fn.indexOf("bootloader") >= 0 || fn.indexOf("merged") >= 0 ||
            fn.indexOf("partitions") >= 0) {
            adicionarLog("OTA: arquivo errado");
            otaErro = true;
            return;
        }
        otaUploadIniciado = true;
        tOtaUltimo = millis();
        size_t sz = (size_t)server.arg("size").toInt();
        snprintf(b, sizeof(b), "OTA inicio %uB", (unsigned)sz);
        adicionarLog(b);
        esp_task_wdt_reset();
        if (!Update.begin(sz > 0 ? sz : UPDATE_SIZE_UNKNOWN)) {
            snprintf(b, sizeof(b), "OTA ERRO begin %d", (int)Update.getError());
            adicionarLog(b);
            otaErro = true;
        }

    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (!otaAtivo || otaErro) return;
        esp_task_wdt_reset();
        tOtaUltimo = millis();
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
            snprintf(b, sizeof(b), "OTA ERRO write %d", (int)Update.getError());
            adicionarLog(b);
            otaErro = true;
            Update.abort();
        }

    } else if (up.status == UPLOAD_FILE_END) {
        if (!otaAtivo || otaErro) return;
        esp_task_wdt_reset();
        if (Update.end(true)) {
            otaOk = true;
            adicionarLog("OTA OK");
        } else {
            snprintf(b, sizeof(b), "OTA ERRO end %d", (int)Update.getError());
            adicionarLog(b);
            otaErro = true;
        }

    } else if (up.status == UPLOAD_FILE_ABORTED) {
        adicionarLog("OTA abortada");
        Update.abort();
        otaErro = true;
    }
}

void handleOTAFim() {
    if (!logado) { server.send(403, "text/plain", "ERRO"); return; }

    if (otaOk && !otaErro) {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", "OK");
        salvarLogsPendentes(true);
        delay(1500);
        ESP.restart();
    } else {
        otaErro = true;
        server.send(500, "text/plain", "ERRO");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 45 — SENSORES (página + calibração)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleSensores() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg="";
    if (server.method()==HTTP_POST) {
        String acao=server.arg("acao");
        if (acao=="calib_painel") {
            msg = calibrarPainelAuto(false);
        } else if (acao=="reset_calib") {
            prefs.begin("rrfs_cal",false);
            prefs.remove("pn_off"); prefs.remove("pn_sens");
            prefs.end(); carregarCalibSensores();
            adicionarLog("Calib reset");
            msg="<div class='ok'>✓ Resetado</div>";
        }
    }
    String c=msg;
    c+=F("<div class='nf' style='font-size:11px'>"
         "Corrente do banco via BMS BLE × nP | Casa = Painel - Banco</div>"
         "<h3>🔍 Leituras ao Vivo</h3>"
         "<div class='sc'><div class='lbl'>☀️ Painel Solar (GPIO34)</div>"
         "<div id='sv_p_corr' class='val'>--</div>"
         "<div id='sv_p_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div id='sv_p_sub' class='sub'>--</div></div>"
         "<div class='bms-card'><div class='bms-title'>🔋 Banco Baterias (BMS BLE)</div>"
         "<div class='grid3'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>I banco</div>"
         "<div id='sv_b_corr' class='bms-val'>--</div><div id='sv_b_dir' class='bms-sub'>--</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>V banco</div>"
         "<div id='sv_b_v' class='bms-val'>--</div><div class='bms-sub'>V</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Capacidade</div>"
         "<div id='sv_b_cap' class='bms-val'>--</div><div class='bms-sub'>Ah</div></div></div>"
         "<div class='grid3' style='margin-top:6px'>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>SOC</div>"
         "<div id='sv_b_soc' class='bms-val'>--</div><div class='bms-sub'>%</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>Ciclos</div>"
         "<div id='sv_b_ciclos' class='bms-val'>--</div></div>"
         "<div style='text-align:center'><div style='font-size:9px;color:#7b1fa2'>BLE</div>"
         "<div id='sv_b_ble' class='bms-val' style='font-size:12px'>--</div></div></div></div>"
         "<div class='sc'><div class='lbl'>🏠 Casa (calculada)</div>"
         "<div id='sv_c_corr' class='val'>--</div>"
         "<div id='sv_c_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div class='sub'>= Painel - Banco</div></div>"
         "<h3>☀️ Calibrar Painel</h3>"
         "<div class='nf' style='font-size:12px;line-height:1.6'>"
         "<b>📋 Instrucoes:</b><br>"
         "1) <b>Desligue tudo</b> (Inversor Baixa) pela pagina <a href='/controle' style='color:#90caf9'>Controle</a>.<br>"
         "2) Deixe <b>apenas o controlador de carga e a bateria</b> ligados, com o painel gerando energia (com sol).<br>"
         "3) Com tudo desligado, <b>toda a corrente do painel vai para a bateria</b>. "
         "Aperte o botao abaixo — o sistema usa a <b>Corrente da Bateria</b> (BMS) como referencia exata."
         "</div>"
         "<div class='sc' style='text-align:center'>"
         "<div class='lbl'>🎯 Referencia atual (corrente da bateria)</div>"
         "<div id='calib_ref' class='val' style='color:#4fc3f7'>--</div>"
         "<div class='sub'>Esse sera o valor usado ao calibrar</div></div>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='calib_painel'>"
         "<button type='submit' class='btn-g'>🎯 Calibrar Agora</button></form>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='reset_calib'>"
         "<button type='submit' class='btn-r btn-sm' style='margin-top:8px'>↻ Reset Calibracao</button></form>");

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function atualizarSens(d){"
    "var cP=d.ip>0.5?'#fdd835':'#455a64';"
    "sT('sv_p_corr','<span style=\"color:'+cP+'\">'+d.ip.toFixed(2)+'A</span>');"
    "sT('sv_p_pot',Math.round(d.wp)+'W');");
    char sB[120];
    snprintf(sB,sizeof(sB),"sT('sv_p_sub',d.mv_painel+' mV | Off:%d | S:%.1f');",
             calibPainel.offsetMV, calibPainel.sensMvA);
    c+=sB;
    c+=F("var cI=d.ib>0.2?'#43a047':d.ib<-0.2?'#ef6c00':'#fdd835';"
         "sT('sv_b_corr','<span style=\"color:'+cI+'\">'+Math.abs(d.ib).toFixed(2)+'A</span>');"
         "var dS=d.ib>0.2?'↑ Carg':d.ib<-0.2?'↓ Desc':'↔ Parado';"
         "sT('sv_b_dir','<span style=\"color:'+cI+'\">'+dS+'</span>');"
         "sT('sv_b_v',d.v_banco.toFixed(2));sT('sv_b_cap',d.cap_banco.toFixed(2));"
         "var cS=d.soc>=50?'#43a047':d.soc>=25?'#ef6c00':'#c62828';"
         "sT('sv_b_soc','<span style=\"color:'+cS+'\">'+d.soc.toFixed(0)+'</span>');"
         "sT('sv_b_ciclos',d.ciclos);"
         "var cBLE=d.bms_valido&&d.bms_t_seg<15?'#a5d6a7':'#ef9a9a';"
         "sT('sv_b_ble','<span style=\"color:'+cBLE+'\">'+d.ble_str+'</span>');"
         "sT('sv_c_corr',d.ic.toFixed(2)+'A');sT('sv_c_pot',Math.round(d.wc)+'W');"
         "var cRef=d.ib>0.05?'#43a047':'#ef9a9a';"
         "sT('calib_ref','<span style=\"color:'+cRef+'\">'+d.ib.toFixed(2)+' A</span>'+(d.ib<=0.05?' ⚠️':''));}"
         "function aD(){var x=new XMLHttpRequest();"
         "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
         "try{atualizarSens(JSON.parse(x.responseText));}catch(e){}}};"
         "x.open('GET','/dados',true);x.send();}"
         "setInterval(aD,2000);aD();</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS Energia - Sensores",c));
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 45B — SAÚDE DA BATERIA (API + página)
// ═══════════════════════════════════════════════════════════════════════════════════

void handleSaudeDados() {
    if (!logado) { server.send(403); return; }
    float pct = 0, capMed = 0, tend = 0, ri = 0;
    bool capOk  = saudeCapPct(pct, capMed);
    bool tendOk = saudeTendencia(tend);
    bool riOk   = saudeRiAtual(ri);
    bool baseOk = saude.riBaseOk && saude.riBase > 0.01f;
    float ratio = (riOk && baseOk) ? ri / saude.riBase : 0.0f;

    int lc = capOk ? saudeLvCap(pct) : -1;
    int lr = (riOk && baseOk) ? saudeLvRi(ratio) : -1;
    int lg = (lc > lr) ? lc : lr;                       // pior dos dois (-1 se nenhum)

    char buf[900];
    snprintf(buf, sizeof(buf),
             "{\"cap_ok\":%s,\"soh\":%.1f,\"cap_med\":%.1f,\"cap_nom\":%.1f,\"n_cap\":%d,"
             "\"tend_ok\":%s,\"tend\":%.2f,"
             "\"jan\":%s,\"jan_ini\":%d,\"jan_soc\":%d,\"jan_ah\":%.2f,"
             "\"ri_ok\":%s,\"ri\":%.1f,\"ri_base_ok\":%s,\"ri_base\":%.1f,\"ri_ratio\":%.2f,\"ri_n\":%d,"
             "\"lc\":%d,\"lr\":%d,\"lg\":%d,\"c_txt\":\"%s\",\"r_txt\":\"%s\",\"g_txt\":\"%s\"}",
             capOk ? "true" : "false", pct, capMed, saudeNominal(), (int)saude.nCap,
             tendOk ? "true" : "false", tend,
             saudeR.janAtiva ? "true" : "false", saudeR.socIni, saudeR.socAnt, saudeR.ahNet,
             riOk ? "true" : "false", ri, baseOk ? "true" : "false", saude.riBase, ratio, (int)saude.riN,
             lc, lr, lg,
             lc >= 0 ? SAUDE_TXT_CAP[lc] : "-",
             lr >= 0 ? SAUDE_TXT_RI[lr] : "-",
             lg >= 0 ? SAUDE_TXT_GERAL[lg] : "-");
    server.send(200, "application/json", buf);
}

void handleSaudeBateria() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }
    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");
        if (acao == "nominal") {
            String s = server.arg("nom"); s.trim(); s.replace(',', '.');
            float v = s.toFloat();
            if (s.length() == 0 || v < 0.0f || v > 2000.0f) {
                msg = "<div class='er'>Capacidade: 0 a 2000 Ah (0 = usar a do BMS)</div>";
            } else {
                saude.nomManual = v;
                saudeR.janAtiva = false;
                saudeR.sujo = true; salvarSaudeSeSujo(true);
                msg = "<div class='ok'>✓ Capacidade nominal salva</div>";
            }
        } else if (acao == "reset_ri") {
            saude.riBaseOk = false; saude.riBase = 0; saude.riN = 0; saude.riIdx = 0; saude.riTotal = 0;
            memset(saude.riRing, 0, sizeof(saude.riRing));
            saudeR.estRi = 0; saudeR.nFr = 0;
            saudeR.sujo = true; salvarSaudeSeSujo(true);
            adicionarLog("Saude: referencia de R reiniciada");
            msg = "<div class='ok'>✓ Nova referencia de resistencia sera medida</div>";
        } else if (acao == "reset_saude") {
            float nm = saude.nomManual;
            memset(&saude, 0, sizeof(saude));
            saude.ver = 1; saude.nomManual = nm;
            saudeR.janAtiva = false; saudeR.estRi = 0; saudeR.nFr = 0; saudeR.socAnt = -1;
            saudeR.sujo = true; salvarSaudeSeSujo(true);
            adicionarLog("Saude: tudo zerado");
            msg = "<div class='ok'>✓ Medidas de saude apagadas</div>";
        }
    }

    paginaInicio("RRFS Energia - Saude da Bateria");
    String c = msg;

    float pct = 0, capMed = 0, tend = 0, ri = 0;
    bool capOk  = saudeCapPct(pct, capMed);
    bool tendOk = saudeTendencia(tend);
    bool riOk   = saudeRiAtual(ri);
    bool baseOk = saude.riBaseOk && saude.riBase > 0.01f;
    float ratio = (riOk && baseOk) ? ri / saude.riBase : 0.0f;
    float nom   = saudeNominal();
    int lc = capOk ? saudeLvCap(pct) : -1;
    int lr = (riOk && baseOk) ? saudeLvRi(ratio) : -1;
    int lg = (lc > lr) ? lc : lr;
    static const char* const COR[4] = {"#43a047", "#9ccc65", "#ef6c00", "#ef5350"};

    // ── Resumo ──────────────────────────────────────────────────────────────
    c += F("<h3>🩺 Saude da Bateria</h3>");
    if (lg >= 0)
        addf(c, "<div class='pd' style='border-color:%s'><div class='dt' style='color:%s'>Estado geral</div>"
        "<div class='dv' style='color:%s'>%s</div>"
        "<div class='ds'>Vale o pior entre capacidade e resistencia interna.</div></div>",
        COR[lg], COR[lg], COR[lg], SAUDE_TXT_GERAL[lg]);
    else
        c += F("<div class='nf'>🔎 Coletando dados. A resistencia aparece na primeira carga forte "
        "(ex.: Inv.Alta ligando) e a capacidade apos duas janelas de 20% de SOC.</div>");

    // ── Resistência interna ─────────────────────────────────────────────────
    c += F("<h3>⚡ Resistencia interna (indicador principal)</h3>");
    if (riOk) {
        addf(c, "<div class='grid3'>"
        "<div class='sc' style='text-align:center'><div class='lbl'>Atual</div><div class='val' style='color:%s'>%.1f mΩ</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>Referencia</div><div class='val'>%s</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>Variacao</div><div class='val' style='color:%s'>%s</div></div></div>",
        lr >= 0 ? COR[lr] : "#e0e0e0", ri,
        baseOk ? String(saude.riBase, 1).c_str() : "--",
             lr >= 0 ? COR[lr] : "#e0e0e0",
             baseOk ? (String(ratio, 2) + "x").c_str() : "--");
        if (lr >= 0)
            addf(c, "<div class='nf' style='font-size:11px'>Situacao: <b>%s</b> | %u medida(s) no total</div>",
                 SAUDE_TXT_RI[lr], (unsigned)saude.riTotal);
            else
                addf(c, "<div class='nf' style='font-size:11px'>Medindo a referencia: %d de %d amostras.</div>",
                     (int)saude.riN, SAUDE_N_RI);
    } else {
        c += F("<div class='nf'>Nenhuma medida ainda. Ela acontece sozinha quando a corrente da bateria "
        "muda de forma brusca (>= 8 A), por exemplo ao ligar/desligar o Inv. Alta.</div>");
    }
    enviarSeTiver(c);

    if (saude.nDia >= 2) {
        float mn = 1e9f, mx = 0.0f;
        for (int i = 0; i < saude.nDia; i++) {
            if (saude.dia[i].mohm < mn) mn = saude.dia[i].mohm;
            if (saude.dia[i].mohm > mx) mx = saude.dia[i].mohm;
        }
        float span = (mx - mn < 0.5f) ? 0.5f : (mx - mn);
        c += F("<div style='height:70px;background:#111;border:1px solid #333;border-radius:4px'>"
        "<svg viewBox='0 0 300 60' preserveAspectRatio='none' style='width:100%;height:100%'>"
        "<path fill='none' stroke='#4db6ac' stroke-width='2' d='M");
        for (int i = 0; i < saude.nDia; i++) {
            char pt[24];
            float x = i * (300.0f / (saude.nDia - 1));
            float y = 55.0f - (saude.dia[i].mohm - mn) / span * 48.0f;
            snprintf(pt, sizeof(pt), "%s%.1f,%.1f", i ? "L" : "", x, y);
            c += pt;
        }
        addf(c, "'/></svg></div><div style='font-size:10px;color:#546e7a'>Resistencia por dia (%.1f a %.1f mΩ)</div>", mn, mx);

        c += F("<div style='overflow-x:auto'><table class='tb'><tr><th>Dia</th><th>R (mΩ)</th><th>vs ref.</th></tr>");
        int ini = (saude.nDia > 14) ? saude.nDia - 14 : 0;
        for (int i = saude.nDia - 1; i >= ini; i--) {
            const RiDia& d = saude.dia[i];
            char rel[12] = "--";
            if (baseOk) snprintf(rel, sizeof(rel), "%.2fx", d.mohm / saude.riBase);
            addf(c, "<tr><td>%02u/%02u</td><td>%.1f</td><td>%s</td></tr>", d.dia, d.mes, d.mohm, rel);
        }
        c += F("</table></div>");
    }
    enviarSeTiver(c);

    // ── Capacidade ──────────────────────────────────────────────────────────
    c += F("<h3>🔋 Capacidade (contador do BMS)</h3>");
    if (capOk) {
        addf(c, "<div class='grid3'>"
        "<div class='sc' style='text-align:center'><div class='lbl'>Medida</div><div class='val' style='color:%s'>%.0f%%</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>Ah</div><div class='val'>%.0f</div><div class='sub'>de %.0f nominal</div></div>"
        "<div class='sc' style='text-align:center'><div class='lbl'>Tendencia</div><div class='val'>%s</div></div></div>",
        COR[lc], pct, capMed, nom,
        tendOk ? (String(tend, 1) + "%/mes").c_str() : "--");
        addf(c, "<div class='nf' style='font-size:11px'>Situacao: <b>%s</b>. A tendencia so aparece com 4+ medidas "
        "e 14+ dias de intervalo.</div>", SAUDE_TXT_CAP[lc]);
    } else {
        addf(c, "<div class='nf'>Sem medidas suficientes (%d de 2). Uma medida precisa de uma variacao de 20%% "
        "de SOC entre 10%% e 96%%, com o BMS sem falhas de leitura.</div>", (int)saude.nCap);
    }
    if (saudeR.janAtiva)
        addf(c, "<div class='wn' style='font-size:11px'>📏 Medindo agora: SOC %d%% → %d%% | %.2f Ah liquidos</div>",
             saudeR.socIni, saudeR.socAnt, saudeR.ahNet);

        if (saude.nCap > 0) {
            c += F("<div style='overflow-x:auto'><table class='tb'><tr><th>Dia</th><th>Janela</th><th>Cap (Ah)</th><th>SOH</th></tr>");
            for (int i = saude.nCap - 1; i >= 0; i--) {
                const CapMed& m = saude.cap[i];
                addf(c, "<tr><td>%02u/%02u</td><td>%d%%</td><td>%.1f</td><td>%.0f%%</td></tr>",
                     m.dia, m.mes, (int)m.delta, m.cap, m.nom > 1.0f ? 100.0f * m.cap / m.nom : 0.0f);
            }
            c += F("</table></div>");
        }
        enviarSeTiver(c);

        // ── Como ler + ajustes ──────────────────────────────────────────────────
        c += F("<h3>ℹ️ Como ler</h3><div class='nf' style='font-size:11px;line-height:1.6'>"
        "<b>Resistencia interna:</b> mede o quanto a tensao cai quando a corrente salta. Sobe conforme as celulas "
        "envelhecem. A referencia e a mediana das 8 primeiras medidas (a bateria ja pode ter algum desgaste "
        "quando a medicao comecou). Mais de 1,3x a referencia merece atencao.<br>"
        "<b>Capacidade:</b> o SOC do BMS e um contador de Ah contra a capacidade programada nele. "
        "Por isso este valor mostra se BMS e sensor estao coerentes, e <b>nao</b> prova que as celulas estao novas. "
        "Uma queda real aqui indica erro do shunt ou capacidade reprogramada no BMS.<br>"
        "A resistencia varia com temperatura e SOC; o BMS nao informa temperatura neste firmware. "
        "Use sempre a <b>tendencia</b>, nao um valor isolado.</div>"
        "<h3>⚙️ Ajustes</h3>"
        "<form method='POST' action='/saude'><input type='hidden' name='acao' value='nominal'>"
        "<label>Capacidade nominal do banco (Ah) — 0 = usar a do BMS:</label>");
        addf(c, "<input type='text' inputmode='decimal' name='nom' value='%.0f'>", saude.nomManual);
        c += F("<button type='submit' class='btn-g btn-sm'>💾 Salvar</button></form>"
        "<form method='POST' action='/saude' style='margin-top:6px' onsubmit='return confirm(\"Medir nova referencia de resistencia?\")'>"
        "<input type='hidden' name='acao' value='reset_ri'>"
        "<button type='submit' class='btn-o btn-sm'>↻ Nova referencia de resistencia</button></form>"
        "<form method='POST' action='/saude' style='margin-top:6px' onsubmit='return confirm(\"Apagar todas as medidas de saude?\")'>"
        "<input type='hidden' name='acao' value='reset_saude'>"
        "<button type='submit' class='btn-r btn-sm'>🗑 Apagar medidas</button></form>"
        "<script>setInterval(function(){location.reload();},30000);</script>");
        enviarSeTiver(c);
        paginaFim(true);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BLOCO 46 — ROTAS WEB
// ═══════════════════════════════════════════════════════════════════════════════════

void configurarRotas() {
    static const char* hdrs[] = {"Cookie"};
    server.collectHeaders(hdrs, 1);

    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/dados",       HTTP_GET,  handleDados);
    server.on("/bateria",     HTTP_ANY,  handleBateria);
    server.on("/controle",    HTTP_ANY,  handleControle);
    server.on("/sensores",    HTTP_ANY,  handleSensores);
    server.on("/bms",         HTTP_ANY,  handleBMS);
    server.on("/config",      HTTP_ANY,  handleConfig);
    server.on("/historico",   HTTP_ANY,  handleHistorico);    // ?dia=N = detalhe do dia
    server.on("/previsao",    HTTP_ANY,  handlePrevisao);
    server.on("/logs",        HTTP_GET,  handleLogs);
    server.on("/logs-salvos", HTTP_GET,  handleLogsSalvos);
    server.on("/login",       HTTP_ANY,  handleLogin);
    server.on("/logout",      HTTP_GET,  handleLogout);
    server.on("/setup-login", HTTP_ANY,  handleSetupLogin);
    server.on("/setup-wifi",  HTTP_ANY,  handleSetupWifi);
    server.on("/reiniciar",   HTTP_POST, handleReiniciar);
    server.on("/ota",         HTTP_GET,  handleOTA);
    server.on("/ota-upload",  HTTP_POST, handleOTAFim, handleOTAUpload);
    server.on("/bms-scan",    HTTP_GET,  handleBMSScan);
    server.on("/bms-lista",   HTTP_GET,  handleBMSLista);
    server.on("/ota-prep",    HTTP_GET,  handleOTAPrep);
    server.on("/ota-estado",  HTTP_GET,  handleOTAEstado);
    server.on("/ota-versao",  HTTP_GET,  handleOTAVersao);
    server.on("/plano",       HTTP_GET,  handlePlano);
    server.on("/prevstat",    HTTP_GET,  handlePrevStat);
    server.on("/excesso",     HTTP_GET,  handleExcesso);
    server.on("/setup-bms",   HTTP_ANY,  handleSetupBMS);
    server.on("/setup-banco", HTTP_ANY,  handleSetupBanco);
    server.on("/setup-calib", HTTP_ANY,  handleSetupCalib);
    server.on("/logs-txt",    HTTP_GET,  handleLogsTxt);
    server.on("/saude",       HTTP_ANY,  handleSaudeBateria);
    server.on("/saude-dados", HTTP_GET,  handleSaudeDados);
    server.onNotFound(handleNotFound);
    server.begin();
    adicionarLog("Rotas OK v32");
}
