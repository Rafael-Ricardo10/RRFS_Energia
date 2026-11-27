/*
 * ════════════════════════════════════════════════════════════
 * RRFS ENERGIA v23 - DUAL INVERSOR + PREVISÃO SOLAR
 * Sistema de Gerenciamento de Energia Solar - ESP32
 * ════════════════════════════════════════════════════════════
 * v23 - Nova lógica dual inversor:
 *   - Inv.Baixa: liga sempre, desliga só em bat=0%
 *   - Inv.Alta:  controlado por sinal DC da casa (>1V)
 *   - Diagnóstico AC para ambos inversores
 *   - Sinais AC estáveis (sem modo rápido)
 * ════════════════════════════════════════════════════════════
 */

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

// ═══════════════════════════════════════════════════════════
// [HW] PINOS
// ═══════════════════════════════════════════════════════════
#define LED_AZUL                2
#define BOTAO_BOOT              0
#define PIN_SENSOR_PAINEL      34   // Fio  Branco
#define PIN_SENSOR_CASA        35   // Fio verde
#define PIN_AC_SINAL_BAIXA     32   // AC do inversor de baixa potência Fio Amarelo
#define PIN_AC_SINAL_ALTA      33   // AC do inversor de alta potência Fio Preto
#define PIN_CASA_SINAL_ALTA    36    // Sinal DC da casa (>1V = liga inv.alta)  Fio Vermelho
#define PIN_RELE_INV_BAIXA     25   // fio amarelo indo pro rele
#define PIN_RELE_INV_ALTA      27    // Fio marom indo pro rele
#define PIN_RELE_LAMP_EXT      12     // Fio azul indo pro rele

// ═══════════════════════════════════════════════════════════
// [CFG] CONSTANTES
// ═══════════════════════════════════════════════════════════
#define DNS_PORT                53
#define TEMPO_RESET_MS       10000
#define MAX_LOGS               100
#define LOG_MAX_LEN             64
#define LOG_NVS_NS         "rrfs_log"

// ADC
#define ADC_SAMPLES            100
#define ADC_VREF              3300
#define ADC_BITS              4095

// Sensores corrente
#define PAINEL_OFFSET_MV      2500
#define PAINEL_SENS_MV_A      35.9f
#define CASA_OFFSET_MV         541
#define CASA_SENS_MV_A        43.3f
#define FILTRO_RUIDO_A         0.3f
#define BUFFER_CORRENTE         10

// ═══════════════════════════════════════════════════════════
// [AC] CONSTANTES v23.5
// ═══════════════════════════════════════════════════════════

// AC — janela e confirmação
#define AC_JANELA_MS            40
#define AC_BUFFER_SUAV           6
#define AC_CONFIRMA_MS         400UL
#define AC_HISTERESE_LIGA_MV    50
#define AC_HISTERESE_DESLIGA_MV 40
#define AC_LIMIAR_MV_DEFAULT   400

// Sinal casa DC
#define CASA_SINAL_LIMIAR_MV  1000
#define CASA_SINAL_JANELA_MS    40
#define CASA_SINAL_BUFFER        6
#define CASA_SINAL_CONFIRMA_MS 400UL
#define CASA_SINAL_HISTERESE    100
#define CASA_AMOSTRAS_RAPIDAS   20     // ← FALTAVA ESTA

// Diagnóstico inversor
#define DIAG_AGUARDA_AC_MS    8000UL
#define DIAG_DESLIGA_MS       5000UL
#define DIAG_MAX_TENTATIVAS      3

// Lâmpada externa
#define LAMP_EXT_HORA_LIGA      19
#define LAMP_EXT_MIN_LIGA        0
#define LAMP_EXT_HORA_DESLIGA    5
#define LAMP_EXT_MIN_DESLIGA    30
#define LAMP_EXT_CHECK_MS    60000UL

// Intervalos — APENAS UMA VEZ (remova a duplicata!)
#define INTERVALO_LEITURA_MS   500UL
#define INTERVALO_CONTROLE_MS 1000UL
#define INTERVALO_NTP_MS    3600000UL
#define INTERVALO_PREVISAO_MS 3600000UL

// WiFi
#define WIFI_RECON_INTERVALO  30000UL
#define WIFI_RECON_TIMEOUT    10000UL
#define WIFI_CHECK_INTERVALO  15000UL
#define WIFI_MAX_FALHAS            5

// Histórico
#define HIST_DIAS              30
#define HIST_HORAS             24
#define HIST_KEY_VER           25
#define HIST_SNAP_PONTOS       60

// Previsão
#define PREV_DIAS               7
#define PREV_TOTAL_HORAS      168

// NTP
#define NTP_SERVER1        "pool.ntp.org"
#define NTP_SERVER2        "time.google.com"
#define NTP_TIMEOUT_MS      3000UL

// WiFi AP
const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

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

struct ConfigLimiares {
    float pctLampExt;          // % mínimo bat para lâmpada
    float pctMinInvAlta;       // % mínimo bat para inv.alta
};

struct CalibSensor {
    int   offsetMV;
    float sensMvA;
    bool  calibrado;
};

struct BufferCorr {
    float   amostras[BUFFER_CORRENTE];
    uint8_t indice;
    uint8_t total;
};

struct DadosSensores {
    float correntePainel;
    float correnteCasa;
    float correnteBateria;
    float potenciaPainel;
    float potenciaCasa;
    float potenciaBateria;
    int   sinalPainel_mV;
    int   sinalCasa_mV;
};

struct EstadoReles {
    bool lampExt;
    bool invAlta;
    bool invBaixa;
};

struct ConfigInversores {
    bool  invertReleInvAlta;
    bool  invertReleInvBaixa;
    bool  invertReleLampExt;
};

// Sinal AC — para cada inversor
struct SinalAC {
    float         mediaMV;
    bool          presente;
    bool          aguardandoLigar;
    bool          aguardandoDesligar;
    unsigned long tInicioConfirma;
    // buffer interno estático gerenciado na função
};

// Sinal DC da casa (pede para ligar inv.alta)
struct SinalCasaAlta {
    float         mediaMV;
    bool          pedindo;          // casa está pedindo inv.alta
    bool          aguardandoLigar;
    bool          aguardandoDesligar;
    unsigned long tInicioConfirma;
};

// Máquina de estados inv.baixa
enum EstadoBaixa {
    INV_B_DESLIGADO = 0,
    INV_B_LIGADO,
    INV_B_DIAG_AGUARDA,   // aguardando AC após ligar
    INV_B_DIAG_OFF,       // desligado no diagnóstico
};

// Máquina de estados inv.alta
enum EstadoAlta {
    INV_A_DESLIGADO = 0,
    INV_A_LIGADO,
    INV_A_DIAG_AGUARDA,   // aguardando AC após ligar
    INV_A_DIAG_OFF,       // desligado no diagnóstico
};

struct MaquinaBaixa {
    EstadoBaixa   estado;
    unsigned long tInicioEstado;
    uint8_t       tentativas;
    uint16_t      totalLigadas;
    uint16_t      totalFalhas;
};

struct MaquinaAlta {
    EstadoAlta    estado;
    unsigned long tInicioEstado;
    uint8_t       tentativas;
    uint16_t      totalLigadas;
    uint16_t      totalFalhas;
};

struct RegistroHora {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
};

struct RegistroDia {
    uint16_t     ano;
    uint8_t      mes, dia;
    bool         valido;
    RegistroHora horas[HIST_HORAS];
    float totalPainelAh, totalCasaAh;
    float totalBatCargAh, totalBatDescAh;
};

struct ContadoresGlobais {
    float painelAh, casaAh;
    float batCargAh, batDescAh;
};

struct AcumHora {
    float         painelAh, casaAh;
    float         batCargAh, batDescAh;
    uint8_t       horaAtual, diaAtual, mesAtual;
    uint16_t      anoAtual;
    unsigned long ultimoAcum;
};

struct ConfigTempo {
    int  fusoHoras;
    int  fusoMinutos;
    bool ntpAtivo;
    bool sincronizado;
    int  manualAnno, manualMes, manualDia;
    int  manualHora, manualMin;
};

struct ConfigPrevisao {
    bool  ativa;
    float latitude, longitude;
    float painelKW, controladorA, fatorCalib;
    bool  configurada;
};

struct DadosPrevisao {
    bool          valido;
    float         radiacao[PREV_TOTAL_HORAS];
    float         ahPrevisto[PREV_TOTAL_HORAS];
    float         ahDia[PREV_DIAS];
    unsigned long tAtualizacao;
    int           horaBase, diaBase, mesBase, anoBase;
};

struct EstimativaTempo {
    bool  valido;
    int   horas, minutos;
    bool  carregando;
    float taxaA;
};

struct ConfigLimiaresSinal {
    int limiarAcMV;   // limiar AC para ambos inversores
};

// ═══════════════════════════════════════════════════════════
// [VAR] VARIÁVEIS GLOBAIS
// ═══════════════════════════════════════════════════════════
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP  = false;
bool logado  = false;

unsigned long ultimoBlink        = 0;
bool          ledState           = false;
bool          blinkAtivo         = true;
unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

char     logs[MAX_LOGS][LOG_MAX_LEN];
uint16_t totalLogs      = 0;
uint8_t  logIndex       = 0;
bool     logsCarregados = false;
char     tempBuf[256];

ConfigBateria  bateria  = {12.8f, 100.0f, 80.0f, 80.0f, 0.0f, 0.0f, false};
ConfigLimiares limiares = {20.0f, 20.0f};  // lamp=20%, invAlta=20%

DadosSensores  sensores   = {0};
CalibSensor    calibPainel, calibCasa;
BufferCorr     bufPainel  = {{0},0,0}, bufCasa = {{0},0,0};

EstadoReles    estados    = {false, false, false};

ConfigInversores cfgInv = {false, false, false};

// Sinais AC
SinalAC acBaixa = {0.0f, false, false, false, 0};
SinalAC acAlta  = {0.0f, false, false, false, 0};

// Sinal DC casa
SinalCasaAlta casaSinal = {0.0f, false, false, false, 0};

// Máquinas de estado
MaquinaBaixa maqBaixa = {INV_B_DESLIGADO, 0, 0, 0, 0};
MaquinaAlta  maqAlta  = {INV_A_DESLIGADO, 0, 0, 0, 0};

ConfigLimiaresSinal limiarSinal = {AC_LIMIAR_MV_DEFAULT};

unsigned long ultimaLeitura   = 0;
unsigned long ultimoControle  = 0;
unsigned long ultimoContagem  = 0;
unsigned long ultimoLampCheck = 0;

ConfigTempo   cfgTempo    = {-3,0,false,false,2024,1,1,0,0};
unsigned long tUltimoNTP  = 0;
bool          ntpSincronizado = false;

ConfigPrevisao cfgPrevisao  = {false,0,0,1.0f,30.0f,1.0f,false};
DadosPrevisao  previsao     = {false};
unsigned long  tUltimaPrevisao = 0;

String        gWifiSSID,gWifiPASS,gWifiIP,gWifiGW,gWifiMASK,gWifiDNS;
unsigned long tUltimoCheckWifi = 0;
unsigned long tUltimoReconWifi = 0;
uint8_t       wifiFalhasConsec = 0;
bool          wifiReconectando = false;
unsigned long tInicioRecon     = 0;

RegistroDia       histDiaAtual;
uint16_t          histIndiceAtual = 0, histTotalDias = 0;
ContadoresGlobais contadores = {0,0,0,0};
AcumHora          acum       = {0,0,0,0,255,0,0,0,0};

float   histCorrPainel[HIST_SNAP_PONTOS] = {0};
float   histCorrCasa  [HIST_SNAP_PONTOS] = {0};
float   histCorrBat   [HIST_SNAP_PONTOS] = {0};
uint8_t histSnapIdx   = 0;
bool    histSnapFull  = false;
unsigned long ultimoHistSnap = 0;

uint8_t  diaCache = 0, mesCache = 0;
uint16_t anoCache = 0;
unsigned long ultimaAtualizacaoDia = 0;

bool modoTeste = false;
unsigned long tInicioTeste     = 0;
unsigned long TIMEOUT_TESTE_MS = 30000UL;

// Proteção contra liga/desliga no boot e reconexões WiFi
unsigned long tInicioSistema     = 0;
bool          sistemaEstabilizado = false;
unsigned long tUltimaReconWifi2  = 0;  // Detecta reconexão

// ═══════════════════════════════════════════════════════════
// [PROTO] PROTÓTIPOS
// ═══════════════════════════════════════════════════════════
EstimativaTempo calcularTempoRestante();
inline float kW(float a);

void adicionarLog(const char* msg);
void logRAM();
void carregarLogsPersistentes();
void salvarLogNVS(uint8_t pos, const char* msg);

void setupLED(); void atualizarLED();
void ledBlink(); void ledFixo(bool v);
void setupBotao(); void verificarReset();

void setupReles();
void ligarInvAlta();    void desligarInvAlta();
void ligarInvBaixa();   void desligarInvBaixa();
void ligarLampExt();    void desligarLampExt();

int   lerADC(uint8_t pin);
float adicionarAmostraCorr(BufferCorr& buf, float nova);
float lerCorrentePainel();
float lerCorrenteCasa();
void  calcularPotencias();
void  lerSensores();

void processarSinalAC(SinalAC& sinal, uint8_t pin, const char* nome);
void processarSinalCasa();
bool acBaixaPresente();
bool acAltaPresente();
bool casaPedindoAlta();

void calcularBateria();
void verificarCalibracaoAutomatica();
void atualizarAhBateria();

void   sincronizarNTP();
bool   obterTempoAtual(struct tm& t);
String formatarDataHora();
String formatarHora();
void   salvarConfigTempo(); void carregarConfigTempo();

void   salvarConfigPrevisao(); void carregarConfigPrevisao();
bool   buscarPrevisaoOpenMeteo();
void   calcularAhPrevisto(); void calibrarPrevisao();
float  getAhPrevHoraAtual(); float getAhPrevRestanteDia();
int    getIndiceHoraAtual(); void gerenciarPrevisao();

void calcularTotaisDia(RegistroDia& dia);
void salvarContadores(); void carregarContadores();
void salvarDiaFlash(uint16_t idx, const RegistroDia& dia);
bool carregarDiaFlash(uint16_t idx, RegistroDia& dia);
void salvarIndiceHistorico();
void carregarHistorico();
void acumularEnergia(); void atualizarDiaAtual();

void maquinaInvBaixa();
void maquinaInvAlta();
void controlarLampExt();
void controlarEnergia();

bool existeCredenciais(); bool existeWiFiConfig();
void salvarCredenciais(const String& u, const String& p);
bool verificarLogin(const String& u, const String& p);
void salvarWifi(const String& ssid,const String& pass,
                const String& ip,const String& gw,
                const String& mask,const String& dns,bool ap);
void carregarWifi(String& ssid,String& pass,String& ip,
                  String& gw,String& mask,String& dns,bool& ap);
void salvarConfigBateria(); void carregarConfigBateria();
void salvarLimiares();      void carregarLimiares();
void carregarCalibSensores();
void salvarConfigInversores(); void carregarConfigInversores();
void salvarLimiaresSinal();    void carregarLimiaresSinal();

void iniciarAP();
bool iniciarWiFiNB(const String& ssid,const String& pass,
                   const String& ip,const String& gw,
                   const String& mask,const String& dns);
void gerenciarWiFi();
void configurarRotas();

String gerarPagina(const String& titulo,
                   const String& conteudo, bool comNav = true);
String barraProgresso(float pct, const String& label = "");
String nomeEstadoBaixa(EstadoBaixa e);
String nomeEstadoAlta(EstadoAlta e);

void handleRoot();      void handleDados();
void handleBateria();   void handleRedes();
void handleSensores();  void handleConfig();
void handleHistorico(); void handlePrevisao();
void handleLogs();      void handleLogsSalvos();
void handleSetupLogin();void handleSetupWifi();
void handleLogin();     void handleLogout();
void handleNotFound();  void handleReiniciar();
void handleOTA();       void handleOTAUpload();
void handleControle();

// ═══════════════════════════════════════════════════════════
// [UTIL]
// ═══════════════════════════════════════════════════════════
inline float kW(float a){ return (a*bateria.tensaoNominal)/1000.0f; }

EstimativaTempo calcularTempoRestante() {
    EstimativaTempo est={false,0,0,false,0};
    if (!bateria.calibrada||bateria.ahDisponivel<=0) return est;
    float taxa=sensores.correnteBateria;
    if (fabsf(taxa)<0.3f) return est;
    est.taxaA=taxa; est.valido=true; est.carregando=(taxa>0);
    float h;
    if (est.carregando){
        float falt=bateria.ahDisponivel-bateria.ahAtual;
        if (falt<=0) return est;
        h=falt/taxa;
    } else {
        if (bateria.ahAtual<=0) return est;
        h=bateria.ahAtual/fabsf(taxa);
    }
    if (h>999) h=999;
    est.horas=(int)h;
    est.minutos=(int)((h-est.horas)*60);
    return est;
}

// ═══════════════════════════════════════════════════════════
// [LOG]
// ═══════════════════════════════════════════════════════════
void carregarLogsPersistentes() {
    if (logsCarregados) return;
    prefs.begin(LOG_NVS_NS,true);
    uint16_t total=prefs.getUShort("total",0);
    uint8_t  idx  =prefs.getUChar ("idx",  0);
    totalLogs=total; logIndex=(idx<MAX_LOGS)?idx:0;
    uint16_t ini=(total>MAX_LOGS)?(total-MAX_LOGS):0;
    for (uint16_t i=ini;i<total;i++){
        uint8_t pos=i%MAX_LOGS; char chave[8];
        snprintf(chave,sizeof(chave),"l%03d",pos);
        String s=prefs.getString(chave,"");
        if (s.length()>0){
            strncpy(logs[pos],s.c_str(),LOG_MAX_LEN-1);
            logs[pos][LOG_MAX_LEN-1]='\0';
        }
    }
    prefs.end(); logsCarregados=true;
}

void salvarLogNVS(uint8_t pos,const char* msg){
    char chave[8]; snprintf(chave,sizeof(chave),"l%03d",pos);
    prefs.begin(LOG_NVS_NS,false);
    prefs.putString(chave,msg);
    prefs.putUShort("total",totalLogs);
    prefs.putUChar ("idx",  logIndex);
    prefs.end();
}

void adicionarLog(const char* msg){
    if (!logsCarregados) carregarLogsPersistentes();
    totalLogs++;
    char entrada[LOG_MAX_LEN];
    snprintf(entrada,sizeof(entrada),"[%05d] %s",totalLogs,msg);
    strncpy(logs[logIndex],entrada,LOG_MAX_LEN-1);
    logs[logIndex][LOG_MAX_LEN-1]='\0';
    if (totalLogs%5==0) salvarLogNVS(logIndex,logs[logIndex]);
    logIndex=(logIndex+1)%MAX_LOGS;
}

void logRAM(){
    snprintf(tempBuf,sizeof(tempBuf),"RAM %.1fKB | reset %d",
             esp_get_free_heap_size()/1024.0f,(int)esp_reset_reason());
    adicionarLog(tempBuf);
}

// ═══════════════════════════════════════════════════════════
// [LED]
// ═══════════════════════════════════════════════════════════
void setupLED(){ pinMode(LED_AZUL,OUTPUT); digitalWrite(LED_AZUL,LOW); }
void atualizarLED(){
    if (!blinkAtivo) return;
    if (millis()-ultimoBlink>=500){
        ultimoBlink=millis(); ledState=!ledState;
        digitalWrite(LED_AZUL,ledState);
    }
}
void ledBlink(){ blinkAtivo=true; ultimoBlink=millis(); }
void ledFixo(bool v){ blinkAtivo=false; digitalWrite(LED_AZUL,v); }

// ═══════════════════════════════════════════════════════════
// [BTN]
// ═══════════════════════════════════════════════════════════
void setupBotao(){ pinMode(BOTAO_BOOT,INPUT_PULLUP); }

void verificarReset(){
    bool p=(digitalRead(BOTAO_BOOT)==LOW);
    if (p&&!botaoSegurando){ botaoSegurando=true; botaoPressionadoEm=millis(); }
    if (!p) botaoSegurando=false;
    if (botaoSegurando&&millis()-botaoPressionadoEm>=TEMPO_RESET_MS){
        adicionarLog("RESET TOTAL botao");
        for (int i=0;i<15;i++){
            digitalWrite(LED_AZUL,HIGH); delay(80);
            digitalWrite(LED_AZUL,LOW);  delay(80);
        }
        nvs_flash_erase(); nvs_flash_init(); ESP.restart();
    }
}

// ═══════════════════════════════════════════════════════════
// [RELE]
// ═══════════════════════════════════════════════════════════
void setupReles(){
    pinMode(PIN_RELE_INV_BAIXA,OUTPUT);
    pinMode(PIN_RELE_INV_ALTA, OUTPUT);
    pinMode(PIN_RELE_LAMP_EXT, OUTPUT);
    digitalWrite(PIN_RELE_INV_BAIXA,cfgInv.invertReleInvBaixa?HIGH:LOW);
    digitalWrite(PIN_RELE_INV_ALTA, cfgInv.invertReleInvAlta ?HIGH:LOW);
    digitalWrite(PIN_RELE_LAMP_EXT, cfgInv.invertReleLampExt ?HIGH:LOW);
    estados={false,false,false};
    adicionarLog("Reles OFF v23");
}

void ligarInvAlta(){
    digitalWrite(PIN_RELE_INV_ALTA,cfgInv.invertReleInvAlta?LOW:HIGH);
    estados.invAlta=true;
    adicionarLog("InvAlta ON");
}
void desligarInvAlta(){
    digitalWrite(PIN_RELE_INV_ALTA,cfgInv.invertReleInvAlta?HIGH:LOW);
    estados.invAlta=false;
    adicionarLog("InvAlta OFF");
}
void ligarInvBaixa(){
    digitalWrite(PIN_RELE_INV_BAIXA,cfgInv.invertReleInvBaixa?LOW:HIGH);
    estados.invBaixa=true;
    adicionarLog("InvBaixa ON");
}
void desligarInvBaixa(){
    digitalWrite(PIN_RELE_INV_BAIXA,cfgInv.invertReleInvBaixa?HIGH:LOW);
    estados.invBaixa=false;
    adicionarLog("InvBaixa OFF");
}
void ligarLampExt(){
    digitalWrite(PIN_RELE_LAMP_EXT,cfgInv.invertReleLampExt?LOW:HIGH);
    estados.lampExt=true;
}
void desligarLampExt(){
    digitalWrite(PIN_RELE_LAMP_EXT,cfgInv.invertReleLampExt?HIGH:LOW);
    estados.lampExt=false;
}

// ═══════════════════════════════════════════════════════════
// [ADC]
// ═══════════════════════════════════════════════════════════
int lerADC(uint8_t pin){
    // Reduzido de 100 para 30 amostras — suficiente para sensores DC
    // Evita bloquear o loop por longos períodos
    long soma=0;
    for (int i=0;i<30;i++){
        soma+=analogRead(pin);
    }
    return (int)(soma/30);
}

float adicionarAmostraCorr(BufferCorr& buf,float nova){
    buf.amostras[buf.indice]=nova;
    buf.indice=(buf.indice+1)%BUFFER_CORRENTE;
    if (buf.total<BUFFER_CORRENTE) buf.total++;
    float soma=0;
    for (uint8_t i=0;i<buf.total;i++) soma+=buf.amostras[i];
    return soma/buf.total;
}

float lerCorrentePainel(){
    int raw=lerADC(PIN_SENSOR_PAINEL);
    int mv=(int)((long)raw*ADC_VREF/ADC_BITS);
    sensores.sinalPainel_mV=mv;
    float corrBruta=-(float)(mv-calibPainel.offsetMV)/calibPainel.sensMvA;
    if (corrBruta<0.15f) corrBruta=0.0f;
    float media=adicionarAmostraCorr(bufPainel,corrBruta);
    return (media<0.15f)?0.0f:media;
}

float lerCorrenteCasa(){
    int raw=lerADC(PIN_SENSOR_CASA);
    int mv=(int)((long)raw*ADC_VREF/ADC_BITS);
    sensores.sinalCasa_mV=mv;
    if (mv<=calibCasa.offsetMV){ adicionarAmostraCorr(bufCasa,0.0f); return 0.0f; }
    float corr=(float)(mv-calibCasa.offsetMV)/calibCasa.sensMvA;
    if (corr<FILTRO_RUIDO_A) corr=0.0f;
    float media=adicionarAmostraCorr(bufCasa,corr);
    return (media<FILTRO_RUIDO_A)?0.0f:media;
}

void calcularPotencias(){
    float vn=bateria.tensaoNominal;
    sensores.potenciaPainel  = sensores.correntePainel  * vn;
    sensores.potenciaCasa    = sensores.correnteCasa    * vn;
    sensores.potenciaBateria = sensores.correnteBateria * vn;
}

void lerSensores(){
    esp_task_wdt_reset();
    sensores.correntePainel  = lerCorrentePainel();
    sensores.correnteCasa    = lerCorrenteCasa();
    sensores.correnteBateria = sensores.correntePainel - sensores.correnteCasa;
    calcularPotencias();
    esp_task_wdt_reset();
    atualizarAhBateria();
    verificarCalibracaoAutomatica();
    if (millis()-ultimoHistSnap>=1000){
        ultimoHistSnap=millis();
        histCorrPainel[histSnapIdx]=sensores.correntePainel;
        histCorrCasa  [histSnapIdx]=sensores.correnteCasa;
        histCorrBat   [histSnapIdx]=sensores.correnteBateria;
        histSnapIdx=(histSnapIdx+1)%HIST_SNAP_PONTOS;
        if (histSnapIdx==0) histSnapFull=true;
    }
}

// ═══════════════════════════════════════════════════════════
// [AC] DETECÇÃO SINAL AC v23.5 — JANELA 40ms ORIGINAL
//
// Volta à lógica original que funcionava:
// Janela 40ms cobre 2+ ciclos de 60Hz corretamente
// Correção: server.handleClient() antes e depois da janela
//           para servidor não travar durante os 40ms
// Chamada a cada 500ms (não 200ms) para dar respiro ao servidor
// ═══════════════════════════════════════════════════════════
void processarSinalAC(SinalAC& sinal, uint8_t pin, const char* nome) {

    static float   bufBaixa[AC_BUFFER_SUAV] = {0};
    static uint8_t idxBaixa = 0, totBaixa = 0;
    static float   bufAlta[AC_BUFFER_SUAV]  = {0};
    static uint8_t idxAlta  = 0, totAlta  = 0;

    float*   buf = (pin == PIN_AC_SINAL_BAIXA) ? bufBaixa : bufAlta;
    uint8_t* bi  = (pin == PIN_AC_SINAL_BAIXA) ? &idxBaixa : &idxAlta;
    uint8_t* bt  = (pin == PIN_AC_SINAL_BAIXA) ? &totBaixa : &totAlta;

    unsigned long agora = millis();

    // ── Serve o cliente antes de bloquear 40ms ───────────────
    server.handleClient();
    if (modoAP || WiFi.getMode() == WIFI_AP_STA)
        dnsServer.processNextRequest();
    esp_task_wdt_reset();

    // ── Janela de 40ms original — cobre 2+ ciclos de 60Hz ────
    long   somaADC = 0;
    int    contADC = 0;
    unsigned long tJanela = millis();

    while (millis() - tJanela < AC_JANELA_MS) {
        somaADC += analogRead(pin);
        contADC++;
        delayMicroseconds(150);
        if (contADC % 20 == 0) esp_task_wdt_reset();
    }
    if (contADC == 0) return;

    float mediaMV = ((float)somaADC / contADC * ADC_VREF) / ADC_BITS;

    // ── Serve o cliente após os 40ms ─────────────────────────
    server.handleClient();
    esp_task_wdt_reset();

    // ── Buffer de suavização ─────────────────────────────────
    buf[*bi] = mediaMV;
    *bi = (*bi + 1) % AC_BUFFER_SUAV;
    if (*bt < AC_BUFFER_SUAV) (*bt)++;

    float soma = 0;
    for (uint8_t i = 0; i < *bt; i++) soma += buf[i];
    sinal.mediaMV = soma / (*bt);

    // ── Limiares com histerese ───────────────────────────────
    int limiarLiga    = limiarSinal.limiarAcMV + AC_HISTERESE_LIGA_MV;
    int limiarDesliga = limiarSinal.limiarAcMV - AC_HISTERESE_DESLIGA_MV;
    if (limiarDesliga < 10) limiarDesliga = 10;

    bool acimaDeLiga   = (sinal.mediaMV >= limiarLiga);
    bool abaixoDesliga = (sinal.mediaMV <  limiarDesliga);

    // ── Máquina de confirmação ───────────────────────────────
    if (!sinal.presente) {
        if (acimaDeLiga) {
            if (!sinal.aguardandoLigar) {
                sinal.aguardandoLigar    = true;
                sinal.tInicioConfirma    = agora;
                sinal.aguardandoDesligar = false;
            } else if (agora - sinal.tInicioConfirma >= AC_CONFIRMA_MS) {
                sinal.presente        = true;
                sinal.aguardandoLigar = false;
                memset(buf, 0, AC_BUFFER_SUAV * sizeof(float));
                *bi = 0; *bt = 0;
                char msg[48];
                snprintf(msg, sizeof(msg), "AC %s: PRESENTE", nome);
                adicionarLog(msg);
            }
        } else {
            sinal.aguardandoLigar = false;
        }
    } else {
        if (abaixoDesliga) {
            if (!sinal.aguardandoDesligar) {
                sinal.aguardandoDesligar = true;
                sinal.tInicioConfirma    = agora;
                sinal.aguardandoLigar    = false;
            } else if (agora - sinal.tInicioConfirma >= AC_CONFIRMA_MS) {
                sinal.presente           = false;
                sinal.aguardandoDesligar = false;
                memset(buf, 0, AC_BUFFER_SUAV * sizeof(float));
                *bi = 0; *bt = 0;
                char msg[48];
                snprintf(msg, sizeof(msg), "AC %s: QUEDA", nome);
                adicionarLog(msg);
            }
        } else {
            sinal.aguardandoDesligar = false;
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [CASA] SINAL DC DA CASA — pede ligar inv.alta
//
// ESP32 3.3V pelo fio → queda ~1V esperada
// Válido acima de 1000mV
// Buffer de suavização para evitar oscilação
// ═══════════════════════════════════════════════════════════
void processarSinalCasa() {

    static float   bufCasaSig[CASA_SINAL_BUFFER] = {0};
    static uint8_t bufIdx = 0, bufTot = 0;

    unsigned long agora = millis();

    // Serve cliente antes de bloquear
    server.handleClient();
    esp_task_wdt_reset();

    // ── Janela de 40ms ───────────────────────────────────────
    long   somaADC = 0;
    int    contADC = 0;
    unsigned long tJanela = millis();

    while (millis() - tJanela < CASA_SINAL_JANELA_MS) {
        somaADC += analogRead(PIN_CASA_SINAL_ALTA);
        contADC++;
        delayMicroseconds(150);
        if (contADC % 20 == 0) esp_task_wdt_reset();
    }
    if (contADC == 0) return;

    float mediaMV = ((float)somaADC / contADC * ADC_VREF) / ADC_BITS;

    // Serve cliente após bloquear
    server.handleClient();
    esp_task_wdt_reset();

    // ── Buffer de suavização ─────────────────────────────────
    bufCasaSig[bufIdx] = mediaMV;
    bufIdx = (bufIdx + 1) % CASA_SINAL_BUFFER;
    if (bufTot < CASA_SINAL_BUFFER) bufTot++;

    float soma = 0;
    for (uint8_t i = 0; i < bufTot; i++) soma += bufCasaSig[i];
    casaSinal.mediaMV = soma / bufTot;

    // ── Limiares com histerese ───────────────────────────────
    int limLiga    = CASA_SINAL_LIMIAR_MV + CASA_SINAL_HISTERESE;
    int limDesliga = CASA_SINAL_LIMIAR_MV - CASA_SINAL_HISTERESE;

    bool acima  = (casaSinal.mediaMV >= limLiga);
    bool abaixo = (casaSinal.mediaMV <  limDesliga);

    // ── Confirmação ──────────────────────────────────────────
    if (!casaSinal.pedindo) {
        if (acima) {
            if (!casaSinal.aguardandoLigar) {
                casaSinal.aguardandoLigar    = true;
                casaSinal.tInicioConfirma    = agora;
                casaSinal.aguardandoDesligar = false;
            } else if (agora - casaSinal.tInicioConfirma >= CASA_SINAL_CONFIRMA_MS) {
                casaSinal.pedindo         = true;
                casaSinal.aguardandoLigar = false;
                adicionarLog("Casa: PEDIU InvAlta");
            }
        } else {
            casaSinal.aguardandoLigar = false;
        }
    } else {
        if (abaixo) {
            if (!casaSinal.aguardandoDesligar) {
                casaSinal.aguardandoDesligar = true;
                casaSinal.tInicioConfirma    = agora;
                casaSinal.aguardandoLigar    = false;
            } else if (agora - casaSinal.tInicioConfirma >= CASA_SINAL_CONFIRMA_MS) {
                casaSinal.pedindo            = false;
                casaSinal.aguardandoDesligar = false;
                adicionarLog("Casa: CORTOU sinal InvAlta");
            }
        } else {
            casaSinal.aguardandoDesligar = false;
        }
    }
}

bool acBaixaPresente() { return acBaixa.presente; }
bool acAltaPresente()  { return acAlta.presente;  }
bool casaPedindoAlta() { return casaSinal.pedindo; }

// ═══════════════════════════════════════════════════════════
// [BAT]
// ═══════════════════════════════════════════════════════════
void calcularBateria(){
    bateria.ahDisponivel=bateria.capacidadeAh*(bateria.descargaSegura/100.0f);
}

void verificarCalibracaoAutomatica(){
    if (sensores.correntePainel>0.5f&&fabsf(sensores.correnteBateria)<=0.5f&&
        sensores.correnteCasa>0.1f&&!bateria.calibrada){
        bateria.ahAtual=bateria.ahDisponivel;
        bateria.percentualAtual=100.0f;
        bateria.calibrada=true;
        prefs.begin("rrfs",false);
        prefs.putFloat("bat_ah_at",bateria.ahAtual);
        prefs.putBool("bat_calib",true);
        prefs.end();
        adicionarLog("Bat auto-calib 100%");
    }
}

void atualizarAhBateria(){
    if (!bateria.calibrada) return;
    unsigned long agora=millis();
    float deltaH=(float)(agora-ultimoContagem)/3600000.0f;
    ultimoContagem=agora;
    bateria.ahAtual+=sensores.correnteBateria*deltaH;
    if (bateria.ahAtual<0) bateria.ahAtual=0;
    if (bateria.ahAtual>bateria.ahDisponivel) bateria.ahAtual=bateria.ahDisponivel;
    if (bateria.ahDisponivel>0)
        bateria.percentualAtual=(bateria.ahAtual/bateria.ahDisponivel)*100.0f;
}

// ═══════════════════════════════════════════════════════════
// [NTP]
// ═══════════════════════════════════════════════════════════
void salvarConfigTempo(){
    prefs.begin("rrfs_time",false);
    prefs.putInt ("fuso_h",   cfgTempo.fusoHoras);
    prefs.putInt ("fuso_m",   cfgTempo.fusoMinutos);
    prefs.putBool("ntp_ativo",cfgTempo.ntpAtivo);
    prefs.putInt ("man_ano",  cfgTempo.manualAnno);
    prefs.putInt ("man_mes",  cfgTempo.manualMes);
    prefs.putInt ("man_dia",  cfgTempo.manualDia);
    prefs.putInt ("man_hora", cfgTempo.manualHora);
    prefs.putInt ("man_min",  cfgTempo.manualMin);
    prefs.putUChar("ver",25);
    prefs.end();
}

void carregarConfigTempo(){
    prefs.begin("rrfs_time",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if (ver!=25){ prefs.end(); cfgTempo={-3,0,false,false,2024,1,1,0,0}; return; }
    cfgTempo.fusoHoras  =prefs.getInt ("fuso_h",  -3);
    cfgTempo.fusoMinutos=prefs.getInt ("fuso_m",   0);
    cfgTempo.ntpAtivo   =prefs.getBool("ntp_ativo",false);
    cfgTempo.manualAnno =prefs.getInt ("man_ano",2024);
    cfgTempo.manualMes  =prefs.getInt ("man_mes",   1);
    cfgTempo.manualDia  =prefs.getInt ("man_dia",   1);
    cfgTempo.manualHora =prefs.getInt ("man_hora",  0);
    cfgTempo.manualMin  =prefs.getInt ("man_min",   0);
    cfgTempo.sincronizado=false;
    prefs.end();
}

void sincronizarNTP() {
    if (!cfgTempo.ntpAtivo) return;
    if (modoAP) { adicionarLog("NTP: modo AP"); return; }
    if (WiFi.status() != WL_CONNECTED) { adicionarLog("NTP: sem WiFi"); return; }

    // Proteção extra: só tenta se WiFi estável (RSSI ok)
    if (WiFi.RSSI() < -85) { adicionarLog("NTP: sinal fraco"); return; }

    adicionarLog("NTP: iniciando");
    long off = (long)cfgTempo.fusoHoras*3600L + (long)cfgTempo.fusoMinutos*60L;
    configTime(off, 0, NTP_SERVER1, NTP_SERVER2);

    // Timeout reduzido de 3s para 1.5s
    // handleClient() chamado a cada 50ms para não travar servidor
    unsigned long tInicio = millis();
    struct tm ti;
    bool ok = false;

    while (millis() - tInicio < 1500UL) {
        esp_task_wdt_reset();
        server.handleClient();  // ← servidor responde durante NTP
        if (modoAP || WiFi.getMode() == WIFI_AP_STA)
            dnsServer.processNextRequest();
        if (getLocalTime(&ti, 50)) { ok = true; break; }
        delay(50);
    }

    if (ok){
        ntpSincronizado = true;
        cfgTempo.sincronizado = true;
        tUltimoNTP = millis();
        adicionarLog("NTP: OK");
    } else {
        ntpSincronizado = false;
        adicionarLog("NTP: timeout");
    }
}

bool obterTempoAtual(struct tm& t){
    if (ntpSincronizado&&cfgTempo.ntpAtivo) return getLocalTime(&t,100);
    memset(&t,0,sizeof(struct tm));
    t.tm_year=cfgTempo.manualAnno-1900; t.tm_mon=cfgTempo.manualMes-1;
    t.tm_mday=cfgTempo.manualDia; t.tm_hour=cfgTempo.manualHora;
    t.tm_min=cfgTempo.manualMin;
    return true;
}

String formatarDataHora(){
    struct tm t;
    if (!obterTempoAtual(t)) return F("--/--/---- --:--");
    char buf[24];
    snprintf(buf,sizeof(buf),"%02d/%02d/%04d %02d:%02d",
             t.tm_mday,t.tm_mon+1,t.tm_year+1900,t.tm_hour,t.tm_min);
    return String(buf);
}

String formatarHora(){
    struct tm t;
    if (!obterTempoAtual(t)) return F("--:--");
    char buf[8];
    snprintf(buf,sizeof(buf),"%02d:%02d",t.tm_hour,t.tm_min);
    return String(buf);
}

// ═══════════════════════════════════════════════════════════
// [PREV]
// ═══════════════════════════════════════════════════════════
void salvarConfigPrevisao(){
    prefs.begin("rrfs_prev",false);
    prefs.putBool ("ativa",      cfgPrevisao.ativa);
    prefs.putFloat("lat",        cfgPrevisao.latitude);
    prefs.putFloat("lon",        cfgPrevisao.longitude);
    prefs.putFloat("painel_kw",  cfgPrevisao.painelKW);
    prefs.putFloat("ctrl_a",     cfgPrevisao.controladorA);
    prefs.putFloat("fator_calib",cfgPrevisao.fatorCalib);
    prefs.putBool ("configurada",cfgPrevisao.configurada);
    prefs.end();
}

void carregarConfigPrevisao(){
    prefs.begin("rrfs_prev",true);
    cfgPrevisao.configurada =prefs.getBool ("configurada",false);
    cfgPrevisao.ativa       =prefs.getBool ("ativa",      false);
    cfgPrevisao.latitude    =prefs.getFloat("lat",        0.0f);
    cfgPrevisao.longitude   =prefs.getFloat("lon",        0.0f);
    cfgPrevisao.painelKW    =prefs.getFloat("painel_kw",  0.0f);
    cfgPrevisao.controladorA=prefs.getFloat("ctrl_a",     0.0f);
    cfgPrevisao.fatorCalib  =prefs.getFloat("fator_calib",1.0f);
    prefs.end();
}

int getIndiceHoraAtual(){
    if (!previsao.valido) return -1;
    struct tm t; if (!obterTempoAtual(t)) return -1;
    auto toDias=[](int a,int m,int d)->long{
        if(m<3){m+=12;a--;} return (long)365*a+a/4-a/100+a/400+(153*m-457)/5+d;
    };
    long diasBase =toDias(previsao.anoBase,previsao.mesBase,previsao.diaBase);
    long diasAtual=toDias(t.tm_year+1900,t.tm_mon+1,t.tm_mday);
    long diff=(diasAtual-diasBase)*24L+(t.tm_hour-previsao.horaBase);
    if (diff<0||diff>=PREV_TOTAL_HORAS) return -1;
    return (int)diff;
}

float getAhPrevHoraAtual(){
    int idx=getIndiceHoraAtual();
    if (idx<0||idx>=PREV_TOTAL_HORAS) return 0.0f;
    return previsao.ahPrevisto[idx];
}

float getAhPrevRestanteDia(){
    int idx=getIndiceHoraAtual(); if(idx<0) return 0.0f;
    struct tm t; if(!obterTempoAtual(t)) return 0.0f;
    int hFalt=24-t.tm_hour; float total=0.0f;
    for(int i=0;i<hFalt&&(idx+i)<PREV_TOTAL_HORAS;i++) total+=previsao.ahPrevisto[idx+i];
    return total;
}

void calcularAhPrevisto(){
    float vn=bateria.tensaoNominal>0.1f?bateria.tensaoNominal:12.8f;
    for(int d=0;d<PREV_DIAS;d++) previsao.ahDia[d]=0.0f;
    for(int h=0;h<PREV_TOTAL_HORAS;h++){
        float rad=previsao.radiacao[h]; if(rad<0) rad=0;
        float corrA=(rad/1000.0f)*(cfgPrevisao.painelKW*1000.0f)/vn;
        if(corrA>cfgPrevisao.controladorA) corrA=cfgPrevisao.controladorA;
        corrA*=cfgPrevisao.fatorCalib;
        previsao.ahPrevisto[h]=corrA;
        int dIdx=h/24; if(dIdx<PREV_DIAS) previsao.ahDia[dIdx]+=corrA;
    }
}

void calibrarPrevisao(){
    if(!previsao.valido||!cfgPrevisao.configurada) return;
    int idx=getIndiceHoraAtual(); if(idx<0) return;
    float ahReal=acum.painelAh, ahPrev=previsao.ahPrevisto[idx];
    if(ahPrev<0.1f||ahReal<0.1f) return;
    float nf=ahReal/ahPrev;
    cfgPrevisao.fatorCalib=cfgPrevisao.fatorCalib*0.90f+nf*0.10f;
    if(cfgPrevisao.fatorCalib<0.3f) cfgPrevisao.fatorCalib=0.3f;
    if(cfgPrevisao.fatorCalib>2.0f) cfgPrevisao.fatorCalib=2.0f;
    calcularAhPrevisto();
    prefs.begin("rrfs_prev",false); prefs.putFloat("fator_calib",cfgPrevisao.fatorCalib); prefs.end();
}

bool buscarPrevisaoOpenMeteo(){
    if (WiFi.status() != WL_CONNECTED) return false;
    if (!cfgPrevisao.configurada) return false;

    // Proteção: só busca se WiFi estável
    if (WiFi.RSSI() < -85){
        adicionarLog("Prev: sinal fraco");
        return false;
    }

    char url[256];
    snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&hourly=shortwave_radiation&forecast_days=7&timezone=auto",
             cfgPrevisao.latitude, cfgPrevisao.longitude);

    HTTPClient http;
    http.begin(url);
    http.setTimeout(5000);        // ← reduzido de 10s para 5s
    http.setConnectTimeout(3000); // ← timeout de conexão TCP

    esp_task_wdt_reset();
    server.handleClient(); // ← antes do GET
    int code = http.GET();
    esp_task_wdt_reset();
    server.handleClient(); // ← após o GET

    if (code != 200){
        http.end();
        snprintf(tempBuf, sizeof(tempBuf), "Prev: HTTP %d", code);
        adicionarLog(tempBuf);
        return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    String body = "";
    body.reserve(8192);
    unsigned long tL = millis();

    while (http.connected() && millis() - tL < 5000UL){
        if (stream->available()){
            char c = stream->read();
            body += c;
            if (body.length() > 16000) break;
        }
        // A cada 200 bytes lidos, cede ao servidor
        if (body.length() % 200 == 0){
            esp_task_wdt_reset();
            server.handleClient();
        }
    }
    http.end();

    int posArr = body.indexOf("shortwave_radiation\":[");
    if (posArr < 0) return false;
    posArr += 22;
    int count = 0, pos = posArr;

    while (count < PREV_TOTAL_HORAS && pos < (int)body.length()){
        while (pos < (int)body.length() &&
            body[pos] != '-' && body[pos] != '.' &&
            (body[pos] < '0' || body[pos] > '9')){
            if (body[pos] == ']') goto fimParser;
            pos++;
            }
            if (pos >= (int)body.length()) break;
            float val = 0.0f; bool neg = false;
        if (body[pos] == '-'){ neg = true; pos++; }
        while (pos < (int)body.length() && body[pos] >= '0' && body[pos] <= '9')
            val = val * 10.0f + (body[pos++] - '0');
        if (pos < (int)body.length() && body[pos] == '.'){
            pos++; float dec = 0.1f;
            while (pos < (int)body.length() && body[pos] >= '0' && body[pos] <= '9')
                val += (body[pos++] - '0') * dec, dec *= 0.1f;
        }
        if (neg) val = -val;
        previsao.radiacao[count++] = val;
    }
    fimParser:
    if (count < 24) return false;
    for (int i = count; i < PREV_TOTAL_HORAS; i++) previsao.radiacao[i] = 0.0f;

    struct tm t;
    if (obterTempoAtual(t)){
        previsao.horaBase = t.tm_hour;
        previsao.diaBase  = t.tm_mday;
        previsao.mesBase  = t.tm_mon + 1;
        previsao.anoBase  = t.tm_year + 1900;
    }
    previsao.tAtualizacao = millis();
    previsao.valido = true;
    calcularAhPrevisto();
    snprintf(tempBuf, sizeof(tempBuf), "Prev OK %d h", count);
    adicionarLog(tempBuf);
    return true;
}

void gerenciarPrevisao(){
    if (!cfgPrevisao.ativa || !cfgPrevisao.configurada) return;
    if (WiFi.status() != WL_CONNECTED) return;
    if (modoAP) return;

    // Só busca previsão se WiFi estável há pelo menos 30s
    // Evita tentar durante reconexões instáveis
    if (millis() - tUltimaReconWifi2 < 30000UL) return;

    unsigned long agora = millis();
    if (!previsao.valido || agora - tUltimaPrevisao >= INTERVALO_PREVISAO_MS){
        tUltimaPrevisao = agora;
        esp_task_wdt_reset();
        server.handleClient(); // ← antes de buscar
        buscarPrevisaoOpenMeteo();
        esp_task_wdt_reset();
        server.handleClient(); // ← após buscar
    }

    static unsigned long tUltimaCalib = 0;
    if (agora - tUltimaCalib >= 3600000UL){
        tUltimaCalib = agora;
        calibrarPrevisao();
    }
}

// ═══════════════════════════════════════════════════════════
// [HIST]
// ═══════════════════════════════════════════════════════════
void calcularTotaisDia(RegistroDia& dia){
    dia.totalPainelAh=dia.totalCasaAh=dia.totalBatCargAh=dia.totalBatDescAh=0;
    for(uint8_t h=0;h<HIST_HORAS;h++){
        dia.totalPainelAh +=dia.horas[h].painelAh;
        dia.totalCasaAh   +=dia.horas[h].casaAh;
        dia.totalBatCargAh+=dia.horas[h].batCargAh;
        dia.totalBatDescAh+=dia.horas[h].batDescAh;
    }
}

void salvarContadores(){
    prefs.begin("rrfs_cnt",false);
    prefs.putFloat("p_ah", contadores.painelAh);
    prefs.putFloat("c_ah", contadores.casaAh);
    prefs.putFloat("bc_ah",contadores.batCargAh);
    prefs.putFloat("bd_ah",contadores.batDescAh);
    prefs.end();
}

void carregarContadores(){
    prefs.begin("rrfs_cnt",true);
    contadores.painelAh =prefs.getFloat("p_ah", 0.0f);
    contadores.casaAh   =prefs.getFloat("c_ah", 0.0f);
    contadores.batCargAh=prefs.getFloat("bc_ah",0.0f);
    contadores.batDescAh=prefs.getFloat("bd_ah",0.0f);
    prefs.end();
}

void salvarDiaFlash(uint16_t idx,const RegistroDia& dia){
    char ns[12]; snprintf(ns,sizeof(ns),"rh%03d",idx%HIST_DIAS);
    prefs.begin(ns,false); prefs.putBytes("d",&dia,sizeof(RegistroDia)); prefs.end();
}

bool carregarDiaFlash(uint16_t idx,RegistroDia& dia){
    char ns[12]; snprintf(ns,sizeof(ns),"rh%03d",idx%HIST_DIAS);
    prefs.begin(ns,true); size_t len=prefs.getBytes("d",&dia,sizeof(RegistroDia)); prefs.end();
    return(len==sizeof(RegistroDia)&&dia.valido);
}

void salvarIndiceHistorico(){
    prefs.begin("rrfs_hidx",false);
    prefs.putUChar ("ver",  HIST_KEY_VER);
    prefs.putUShort("idx",  histIndiceAtual);
    prefs.putUShort("total",histTotalDias);
    prefs.end();
}

void carregarHistorico(){
    prefs.begin("rrfs_hidx",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if(ver!=HIST_KEY_VER){
        prefs.end(); histIndiceAtual=0; histTotalDias=0;
        memset(&histDiaAtual,0,sizeof(RegistroDia));
        adicionarLog("Hist reset v23"); return;
    }
    histIndiceAtual=prefs.getUShort("idx",  0);
    histTotalDias  =prefs.getUShort("total",0);
    prefs.end();
    if(histTotalDias>0&&!carregarDiaFlash(histIndiceAtual,histDiaAtual))
        memset(&histDiaAtual,0,sizeof(RegistroDia));
}

void atualizarDiaAtual(){
    unsigned long agora=millis();
    if(agora-ultimaAtualizacaoDia<60000UL&&diaCache!=0) return;
    ultimaAtualizacaoDia=agora;
    struct tm t;
    if(obterTempoAtual(t)){ diaCache=t.tm_mday; mesCache=t.tm_mon+1; anoCache=t.tm_year+1900; }
    else if(!diaCache){ diaCache=1; mesCache=1; anoCache=2024; }
}

void acumularEnergia(){
    atualizarDiaAtual();
    unsigned long agora=millis();
    struct tm t; uint8_t hA=0,dA=diaCache,mA=mesCache; uint16_t aA=anoCache;
    if(obterTempoAtual(t)){ hA=t.tm_hour; dA=t.tm_mday; mA=t.tm_mon+1; aA=t.tm_year+1900; }
    else{ hA=(agora/3600000UL)%24; }

    if(acum.ultimoAcum==0){
        acum.ultimoAcum=agora; acum.horaAtual=hA; acum.diaAtual=dA;
        acum.mesAtual=mA; acum.anoAtual=aA;
        bool diaEx=false;
        if(histTotalDias>0){
            RegistroDia dT;
            if(carregarDiaFlash(histIndiceAtual,dT)&&dT.dia==dA&&dT.mes==mA&&dT.ano==aA)
                { histDiaAtual=dT; diaEx=true; }
        }
        if(!diaEx){
            memset(&histDiaAtual,0,sizeof(RegistroDia));
            histDiaAtual.ano=aA; histDiaAtual.mes=mA;
            histDiaAtual.dia=dA; histDiaAtual.valido=true;
            if(histTotalDias==0){ histIndiceAtual=0; histTotalDias=1; }
            else{
                RegistroDia dT;
                if(carregarDiaFlash(histIndiceAtual,dT)&&(dT.dia!=dA||dT.mes!=mA||dT.ano!=aA)){
                    histIndiceAtual=(histIndiceAtual+1)%HIST_DIAS;
                    if(histTotalDias<HIST_DIAS) histTotalDias++;
                }
            }
            salvarDiaFlash(histIndiceAtual,histDiaAtual); salvarIndiceHistorico();
        }
        return;
    }

    float dH=(float)(agora-acum.ultimoAcum)/3600000.0f;
    acum.ultimoAcum=agora; if(dH>0.1f) dH=0.1f;
    float ip=sensores.correntePainel,ic=sensores.correnteCasa,ib=sensores.correnteBateria;
    if(ip>0.15f) acum.painelAh+=ip*dH;
    if(ic>0.3f)  acum.casaAh  +=ic*dH;
    if(ib>0.3f)  acum.batCargAh+=ib*dH;
    if(ib<-0.3f) acum.batDescAh+=fabsf(ib)*dH;

    bool mudouHora=(hA!=acum.horaAtual);
    bool mudouDia =(dA!=acum.diaAtual||mA!=acum.mesAtual||aA!=acum.anoAtual);
    if(mudouHora||mudouDia){
        uint8_t hi=acum.horaAtual%HIST_HORAS;
        histDiaAtual.horas[hi].painelAh +=acum.painelAh;
        histDiaAtual.horas[hi].casaAh   +=acum.casaAh;
        histDiaAtual.horas[hi].batCargAh+=acum.batCargAh;
        histDiaAtual.horas[hi].batDescAh+=acum.batDescAh;
        calcularTotaisDia(histDiaAtual);
        contadores.painelAh +=acum.painelAh; contadores.casaAh   +=acum.casaAh;
        contadores.batCargAh+=acum.batCargAh; contadores.batDescAh+=acum.batDescAh;
        if(mudouDia){
            salvarDiaFlash(histIndiceAtual,histDiaAtual);
            histIndiceAtual=(histIndiceAtual+1)%HIST_DIAS;
            if(histTotalDias<HIST_DIAS) histTotalDias++;
            memset(&histDiaAtual,0,sizeof(RegistroDia));
            histDiaAtual.ano=aA; histDiaAtual.mes=mA;
            histDiaAtual.dia=dA; histDiaAtual.valido=true;
        }
        salvarDiaFlash(histIndiceAtual,histDiaAtual);
        salvarIndiceHistorico(); salvarContadores();
        acum.painelAh=acum.casaAh=acum.batCargAh=acum.batDescAh=0;
        acum.horaAtual=hA;
        if(mudouDia){ acum.diaAtual=dA; acum.mesAtual=mA; acum.anoAtual=aA; }
    }
    static unsigned long tParcial=0;
    if(agora-tParcial>=300000UL){
        tParcial=agora;
        salvarDiaFlash(histIndiceAtual,histDiaAtual);
        salvarContadores();
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] MÁQUINA INV. BAIXA v23.1
//
// PROTEÇÃO SERVIDORES:
// - Nos primeiros 120s após boot: NÃO desliga por falha AC
// - Após reconexão WiFi: aguarda 30s antes de diagnosticar
// - Só desliga se bateria = 0%
// ═══════════════════════════════════════════════════════════
void maquinaInvBaixa() {
    if (modoTeste) return;

    unsigned long agora = millis();

    // ── Proteção inicial (120s após boot) ─────────────────────
    if (!sistemaEstabilizado){
        if (agora - tInicioSistema < 120000UL){
            // Primeiros 2 minutos: apenas garante que está ligado
            if (!estados.invBaixa && bateria.percentualAtual > 0){
                ligarInvBaixa();
                maqBaixa.estado = INV_B_LIGADO;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvB: ON (protecao boot)");
            }
            return; // NÃO faz diagnóstico ainda
        } else {
            sistemaEstabilizado = true;
            adicionarLog("InvB: sistema estabilizado");
        }
    }

    // ── Proteção reconexão WiFi (30s) ──────────────────────────
    if (wifiReconectando || (agora - tUltimaReconWifi2 < 30000UL)){
        // Durante reconexão: mantém estado atual, não diagnostica
        return;
    }

    // Bateria esgotada = percentual <= 0%
    bool batEsgotada = bateria.calibrada && (bateria.percentualAtual <= 0.0f);

    switch (maqBaixa.estado) {

        case INV_B_DESLIGADO: {
            if (!batEsgotada) {
                ligarInvBaixa();
                maqBaixa.estado        = INV_B_DIAG_AGUARDA;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvB: ligou, aguardando AC");
            }
            break;
        }

        case INV_B_LIGADO: {
            // Desliga APENAS se bateria esgotou
            if (batEsgotada) {
                desligarInvBaixa();
                maqBaixa.estado        = INV_B_DESLIGADO;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvB: DESLIGADO bat=0%");
                break;
            }
            // AC perdido → diagnóstico (mas NÃO desliga ainda)
            if (!acBaixaPresente()) {
                maqBaixa.estado        = INV_B_DIAG_AGUARDA;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvB: AC perdido, diagnostico");
            }
            break;
        }

        case INV_B_DIAG_AGUARDA: {
            // Desliga se bateria esgotou durante diagnóstico
            if (batEsgotada) {
                desligarInvBaixa();
                maqBaixa.estado        = INV_B_DESLIGADO;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvB: DESLIGADO bat=0% (diag)");
                break;
            }
            // AC voltou → OK, volta para LIGADO
            if (acBaixaPresente()) {
                maqBaixa.estado        = INV_B_LIGADO;
                maqBaixa.tInicioEstado = agora;
                maqBaixa.tentativas    = 0;
                maqBaixa.totalLigadas++;
                adicionarLog("InvB: AC OK, LIGADO normal");
                break;
            }
            // Timeout aguardando AC
            if (agora - maqBaixa.tInicioEstado >= DIAG_AGUARDA_AC_MS) {
                desligarInvBaixa();
                maqBaixa.estado        = INV_B_DIAG_OFF;
                maqBaixa.tInicioEstado = agora;
                maqBaixa.tentativas++;
                snprintf(tempBuf,sizeof(tempBuf),
                         "InvB: sem AC, tentativa %d/%d",
                         maqBaixa.tentativas, DIAG_MAX_TENTATIVAS);
                adicionarLog(tempBuf);
            }
            break;
        }

        case INV_B_DIAG_OFF: {
            if (batEsgotada) {
                maqBaixa.estado        = INV_B_DESLIGADO;
                maqBaixa.tInicioEstado = agora;
                adicionarLog("InvB: DESLIGADO bat=0% (off)");
                break;
            }
            if (agora - maqBaixa.tInicioEstado >= DIAG_DESLIGA_MS) {
                if (maqBaixa.tentativas >= DIAG_MAX_TENTATIVAS) {
                    // Falha definitiva → aguarda 60s e tenta novamente
                    maqBaixa.tentativas = 0;
                    maqBaixa.totalFalhas++;
                    adicionarLog("InvB: falha definitiva, retry 60s");
                    maqBaixa.tInicioEstado = agora + 55000UL;
                } else {
                    ligarInvBaixa();
                    maqBaixa.estado        = INV_B_DIAG_AGUARDA;
                    maqBaixa.tInicioEstado = agora;
                    adicionarLog("InvB: religando para diagnostico");
                }
            }
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [CTRL] MÁQUINA INV. ALTA
//
// Liga: casa pede E bateria >= limiar configurado
// Desliga: casa corta sinal
// Diagnóstico igual ao inv.baixa
// ═══════════════════════════════════════════════════════════
void maquinaInvAlta() {
    if (modoTeste) return;

    unsigned long agora = millis();
    bool casaPede  = casaPedindoAlta();
    bool batOK     = !bateria.calibrada ||
                     (bateria.percentualAtual >= limiares.pctMinInvAlta);

    switch (maqAlta.estado) {

        case INV_A_DESLIGADO: {
            // Liga apenas se casa pede E bateria tem energia
            if (casaPede && batOK) {
                ligarInvAlta();
                maqAlta.estado        = INV_A_DIAG_AGUARDA;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: ligou, aguardando AC");
            }
            break;
        }

        case INV_A_LIGADO: {
            // Casa cortou sinal → desliga imediatamente
            if (!casaPede) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: casa cortou, desligando");
                break;
            }
            // Bateria acabou → desliga
            if (!batOK) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: DESLIGADO bat insuf");
                break;
            }
            // AC perdido → diagnóstico
            if (!acAltaPresente()) {
                maqAlta.estado        = INV_A_DIAG_AGUARDA;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: AC perdido, diagnostico");
            }
            break;
        }

        case INV_A_DIAG_AGUARDA: {
            // Casa cortou durante diagnóstico → desliga
            if (!casaPede) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: casa cortou (diag)");
                break;
            }
            // Bateria insuficiente
            if (!batOK) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: bat insuf (diag)");
                break;
            }
            // AC voltou → OK
            if (acAltaPresente()) {
                maqAlta.estado        = INV_A_LIGADO;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas    = 0;
                maqAlta.totalLigadas++;
                adicionarLog("InvA: AC OK, LIGADO normal");
                break;
            }
            // Timeout
            if (agora - maqAlta.tInicioEstado >= DIAG_AGUARDA_AC_MS) {
                desligarInvAlta();
                maqAlta.estado        = INV_A_DIAG_OFF;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas++;
                snprintf(tempBuf,sizeof(tempBuf),
                    "InvA: sem AC, tentativa %d/%d",
                    maqAlta.tentativas, DIAG_MAX_TENTATIVAS);
                adicionarLog(tempBuf);
            }
            break;
        }

        case INV_A_DIAG_OFF: {
            // Casa cortou → desliga definitivo
            if (!casaPede) {
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                maqAlta.tentativas    = 0;
                adicionarLog("InvA: casa cortou (off diag)");
                break;
            }
            if (!batOK) {
                maqAlta.estado        = INV_A_DESLIGADO;
                maqAlta.tInicioEstado = agora;
                adicionarLog("InvA: bat insuf (off diag)");
                break;
            }
            if (agora - maqAlta.tInicioEstado >= DIAG_DESLIGA_MS) {
                if (maqAlta.tentativas >= DIAG_MAX_TENTATIVAS) {
                    maqAlta.tentativas = 0;
                    maqAlta.totalFalhas++;
                    adicionarLog("InvA: falha definitiva, retry 60s");
                    maqAlta.tInicioEstado = agora + 55000UL;
                } else {
                    ligarInvAlta();
                    maqAlta.estado        = INV_A_DIAG_AGUARDA;
                    maqAlta.tInicioEstado = agora;
                    adicionarLog("InvA: religando para diagnostico");
                }
            }
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [LAMP] ILUMINAÇÃO EXTERNA v23.1
//
// REGRAS:
// 1) DESLIGA imediatamente se começa geração solar (>0.5A)
// 2) DESLIGA se bateria < limiar configurado
// 3) DESLIGA às 5:30 (fim da noite)
// 4) LIGA às 19:00 SE: não gerando + bateria OK + invBaixa ligado
// ═══════════════════════════════════════════════════════════
void controlarLampExt(){
    if (modoTeste) return;
    unsigned long agora = millis();
    if (agora - ultimoLampCheck < LAMP_EXT_CHECK_MS) return;
    ultimoLampCheck = agora;

    bool gerando = (sensores.correntePainel > 0.5f);
    bool batOK   = !bateria.calibrada ||
    (bateria.percentualAtual > limiares.pctLampExt);

    struct tm t;
    bool tempoOK = obterTempoAtual(t);

    // ── PRIORIDADE 1: DESLIGA se começou a gerar ──────────────
    if (gerando && estados.lampExt){
        desligarLampExt();
        adicionarLog("LampExt OFF: gerando solar");
        return;
    }

    // ── PRIORIDADE 2: DESLIGA se bateria fraca ────────────────
    if (estados.lampExt && !batOK){
        desligarLampExt();
        adicionarLog("LampExt OFF: bat fraca");
        return;
    }

    // ── PRIORIDADE 3: DESLIGA às 5:30 (fim da noite) ──────────
    if (estados.lampExt && tempoOK){
        // Desliga entre 5:30 e 6:59 (antes do nascer do sol típico)
        if (t.tm_hour == LAMP_EXT_HORA_DESLIGA && t.tm_min >= LAMP_EXT_MIN_DESLIGA){
            desligarLampExt();
            adicionarLog("LampExt OFF: 5:30 programado");
            return;
        }
        if (t.tm_hour > LAMP_EXT_HORA_DESLIGA && t.tm_hour < 7){
            desligarLampExt();
            adicionarLog("LampExt OFF: apos 5:30");
            return;
        }
    }

    // ── PRIORIDADE 4: LIGA às 19:00 ────────────────────────────
    // Condições: não gerando + bateria OK + invBaixa ligado + horário válido
    if (!gerando && !estados.lampExt && batOK && estados.invBaixa && tempoOK){
        // Liga entre 19:00 e 5:29 (período noturno)
        bool periodoNoturno = false;

        if (t.tm_hour >= LAMP_EXT_HORA_LIGA){
            // Das 19:00 às 23:59
            periodoNoturno = true;
        } else if (t.tm_hour < LAMP_EXT_HORA_DESLIGA){
            // Das 0:00 às 4:59
            periodoNoturno = true;
        } else if (t.tm_hour == LAMP_EXT_HORA_DESLIGA && t.tm_min < LAMP_EXT_MIN_DESLIGA){
            // Das 5:00 às 5:29
            periodoNoturno = true;
        }

        if (periodoNoturno){
            ligarLampExt();
            char msgBuf[48];
            snprintf(msgBuf, sizeof(msgBuf), "LampExt ON: %02d:%02d", t.tm_hour, t.tm_min);
            adicionarLog(msgBuf);
        }
    }
}

void controlarEnergia(){
    if (modoTeste) return;
    maquinaInvBaixa();
    maquinaInvAlta();
    controlarLampExt();
}

// ═══════════════════════════════════════════════════════════
// [PREF]
// ═══════════════════════════════════════════════════════════
bool existeCredenciais(){
    prefs.begin("rrfs",true); bool ok=prefs.isKey("user"); prefs.end(); return ok;
}
bool existeWiFiConfig(){
    prefs.begin("rrfs",true); bool ok=prefs.isKey("wifi_ok"); prefs.end(); return ok;
}
void salvarCredenciais(const String& u,const String& p){
    prefs.begin("rrfs",false); prefs.putString("user",u); prefs.putString("pass",p); prefs.end();
}
bool verificarLogin(const String& u,const String& p){
    prefs.begin("rrfs",true);
    String su=prefs.getString("user",""),sp=prefs.getString("pass","");
    prefs.end();
    return(su==u&&sp==p&&su.length()>0);
}
void salvarWifi(const String& ssid,const String& pass,const String& ip,
                const String& gw,const String& mask,const String& dns,bool ap){
    prefs.begin("rrfs",false);
    prefs.putBool("usar_ap",ap); prefs.putString("wifi_ssid",ssid);
    prefs.putString("wifi_pass",pass); prefs.putString("wifi_ip",ip);
    prefs.putString("wifi_gw",gw); prefs.putString("wifi_mask",mask);
    prefs.putString("wifi_dns",dns); prefs.putBool("wifi_ok",true);
    prefs.end();
}
void carregarWifi(String& ssid,String& pass,String& ip,
                  String& gw,String& mask,String& dns,bool& ap){
    prefs.begin("rrfs",true);
    ap  =prefs.getBool  ("usar_ap",  true);
    ssid=prefs.getString("wifi_ssid","");
    pass=prefs.getString("wifi_pass","");
    ip  =prefs.getString("wifi_ip",  "");
    gw  =prefs.getString("wifi_gw",  "");
    mask=prefs.getString("wifi_mask","255.255.255.0");
    dns =prefs.getString("wifi_dns", "8.8.8.8");
    prefs.end();
}
void salvarConfigBateria(){
    prefs.begin("rrfs",false);
    prefs.putFloat("bat_vnom", bateria.tensaoNominal);
    prefs.putFloat("bat_ah",   bateria.capacidadeAh);
    prefs.putFloat("bat_desc", bateria.descargaSegura);
    prefs.putFloat("bat_ah_at",bateria.ahAtual);
    prefs.putBool ("bat_calib",bateria.calibrada);
    prefs.end();
}
void carregarConfigBateria(){
    prefs.begin("rrfs",true);
    bateria.tensaoNominal =prefs.getFloat("bat_vnom", 12.8f);
    bateria.capacidadeAh  =prefs.getFloat("bat_ah",  100.0f);
    bateria.descargaSegura=prefs.getFloat("bat_desc", 80.0f);
    bateria.ahAtual       =prefs.getFloat("bat_ah_at",  0.0f);
    bateria.calibrada     =prefs.getBool ("bat_calib", false);
    prefs.end(); calcularBateria();
    if(bateria.ahDisponivel>0)
        bateria.percentualAtual=(bateria.ahAtual/bateria.ahDisponivel)*100.0f;
}
void salvarLimiares(){
    prefs.begin("rrfs",false);
    prefs.putFloat("lim_lamp",   limiares.pctLampExt);
    prefs.putFloat("lim_inv_a",  limiares.pctMinInvAlta);
    prefs.end();
}
void carregarLimiares(){
    prefs.begin("rrfs",true);
    limiares.pctLampExt   =prefs.getFloat("lim_lamp",  20.0f);
    limiares.pctMinInvAlta=prefs.getFloat("lim_inv_a", 20.0f);
    prefs.end();
}
void carregarCalibSensores(){
    calibPainel={PAINEL_OFFSET_MV,PAINEL_SENS_MV_A,true};
    calibCasa  ={CASA_OFFSET_MV,  CASA_SENS_MV_A,  true};
    prefs.begin("rrfs_cal",true);
    if(prefs.isKey("pn_off"))   calibPainel.offsetMV=prefs.getInt  ("pn_off",  calibPainel.offsetMV);
    if(prefs.isKey("pn_sens"))  calibPainel.sensMvA =prefs.getFloat("pn_sens", calibPainel.sensMvA);
    if(prefs.isKey("casa_off")) calibCasa.offsetMV  =prefs.getInt  ("casa_off",calibCasa.offsetMV);
    if(prefs.isKey("casa_sens"))calibCasa.sensMvA   =prefs.getFloat("casa_sens",calibCasa.sensMvA);
    prefs.end();
}
void salvarConfigInversores(){
    prefs.begin("rrfs_inv",false);
    prefs.putBool ("inv_inva",  cfgInv.invertReleInvAlta);
    prefs.putBool ("inv_invb",  cfgInv.invertReleInvBaixa);
    prefs.putBool ("inv_lamp",  cfgInv.invertReleLampExt);
    prefs.putUChar("ver",25);
    prefs.end();
}
void carregarConfigInversores(){
    prefs.begin("rrfs_inv",true);
    uint8_t ver=prefs.getUChar("ver",0);
    if(ver==25){
        cfgInv.invertReleInvAlta =prefs.getBool("inv_inva",false);
        cfgInv.invertReleInvBaixa=prefs.getBool("inv_invb",false);
        cfgInv.invertReleLampExt =prefs.getBool("inv_lamp", false);
    }
    prefs.end();
    adicionarLog("Config inv v23 carregada");
}
void salvarLimiaresSinal(){
    prefs.begin("rrfs_lsig",false);
    prefs.putInt("lim_ac",limiarSinal.limiarAcMV);
    prefs.end();
}
void carregarLimiaresSinal(){
    prefs.begin("rrfs_lsig",true);
    limiarSinal.limiarAcMV=prefs.getInt("lim_ac",AC_LIMIAR_MV_DEFAULT);
    prefs.end();
}

// ═══════════════════════════════════════════════════════════
// [WIFI]
// ═══════════════════════════════════════════════════════════
void iniciarAP(){
    modoAP=true;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP,AP_GW,AP_MASK);
    WiFi.softAP(AP_SSID);
    dnsServer.start(DNS_PORT,"*",AP_IP);
    adicionarLog("AP RRFS 10.0.0.1");
    logRAM(); ledBlink();
}

bool iniciarWiFiNB(const String& ssid,const String& pass,
                   const String& ip,const String& gw,
                   const String& mask,const String& dns){
    IPAddress lIP,lGW,lMask,lDNS;
    if(!lIP.fromString(ip))    return false;
    if(!lGW.fromString(gw))    return false;
    if(!lMask.fromString(mask))return false;
    if(!lDNS.fromString(dns))  lDNS.fromString("8.8.8.8");
    gWifiSSID=ssid; gWifiPASS=pass;
    gWifiIP=ip; gWifiGW=gw; gWifiMASK=mask; gWifiDNS=dns;
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.config(lIP,lGW,lMask,lDNS);
    WiFi.begin(ssid.c_str(),pass.c_str());
    adicionarLog("WiFi conectando"); ledBlink();
    unsigned long t=millis();
    while(WiFi.status()!=WL_CONNECTED&&millis()-t<15000){
        atualizarLED(); esp_task_wdt_reset(); delay(100);
    }
    if(WiFi.status()==WL_CONNECTED){
        adicionarLog("WiFi OK");
        server.stop(); delay(50); server.begin();
        modoAP=false; wifiFalhasConsec=0; ledFixo(true);
        return true;
    }
    adicionarLog("WiFi FALHOU"); return false;
}

void gerenciarWiFi(){
    unsigned long agora=millis();
    if(agora-tUltimoCheckWifi<WIFI_CHECK_INTERVALO) return;
    tUltimoCheckWifi=agora;
    if(modoAP&&WiFi.getMode()==WIFI_AP){
        if(WiFi.softAPIP()==IPAddress(0,0,0,0)){ adicionarLog("AP caiu: reiniciando"); iniciarAP(); }
        return;
    }
    if(WiFi.status()==WL_CONNECTED){
        if(wifiReconectando){
            adicionarLog("WiFi: reconectado OK");
            if(WiFi.getMode()==WIFI_AP_STA){
                WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA);
                dnsServer.stop(); modoAP=false;
                adicionarLog("AP temporario encerrado");
            }
            server.stop(); delay(50); server.begin(); ledFixo(true);
        }
        wifiFalhasConsec=0; wifiReconectando=false; tInicioRecon=0;
        tUltimaReconWifi2 = millis(); // ← PROTEÇÃO INV.BAIXA (v23.1)
        return;
    }
    if(agora-tUltimoReconWifi<WIFI_RECON_INTERVALO) return;
    tUltimoReconWifi=agora; wifiFalhasConsec++;
    snprintf(tempBuf,sizeof(tempBuf),"WiFi: falha %d/%d",wifiFalhasConsec,WIFI_MAX_FALHAS);
    adicionarLog(tempBuf);
    if(!wifiReconectando){
        wifiReconectando=true; tInicioRecon=agora;
        if(!gWifiIP.isEmpty()){
            IPAddress lIP,lGW,lMask,lDNS;
            lIP.fromString(gWifiIP); lGW.fromString(gWifiGW);
            lMask.fromString(gWifiMASK); lDNS.fromString(gWifiDNS);
            WiFi.config(lIP,lGW,lMask,lDNS);
        }
        WiFi.reconnect(); adicionarLog("WiFi: tentando reconectar"); return;
    }
    if(agora-tInicioRecon>=WIFI_RECON_TIMEOUT){
        wifiReconectando=false; tInicioRecon=0;
        if(wifiFalhasConsec>=WIFI_MAX_FALHAS){
            adicionarLog("WiFi: sem sinal, AP emergencial");
            wifiFalhasConsec=0;
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAPConfig(AP_IP,AP_GW,AP_MASK);
            WiFi.softAP(AP_SSID);
            dnsServer.stop(); dnsServer.start(DNS_PORT,"*",AP_IP);
            modoAP=true; ledBlink();
            adicionarLog("AP emergencial: 10.0.0.1");
            if(!gWifiSSID.isEmpty()) WiFi.begin(gWifiSSID.c_str(),gWifiPASS.c_str());
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [HELPERS WEB]
// ═══════════════════════════════════════════════════════════
String nomeEstadoBaixa(EstadoBaixa e){
    switch(e){
        case INV_B_DESLIGADO:   return F("DESLIGADO");
        case INV_B_LIGADO:      return F("LIGADO");
        case INV_B_DIAG_AGUARDA:return F("AGUARD AC");
        case INV_B_DIAG_OFF:    return F("DIAG OFF");
        default:                return F("?");
    }
}
String nomeEstadoAlta(EstadoAlta e){
    switch(e){
        case INV_A_DESLIGADO:   return F("DESLIGADO");
        case INV_A_LIGADO:      return F("LIGADO");
        case INV_A_DIAG_AGUARDA:return F("AGUARD AC");
        case INV_A_DIAG_OFF:    return F("DIAG OFF");
        default:                return F("?");
    }
}

// ═══════════════════════════════════════════════════════════
// [SETUP]
// ═══════════════════════════════════════════════════════════
void setup(){
    esp_task_wdt_config_t wdt_config={
        .timeout_ms=30000,.idle_core_mask=0,.trigger_panic=true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    logsCarregados=false;
    carregarLogsPersistentes();

    esp_err_t errNVS=nvs_flash_init();
    if(errNVS==ESP_ERR_NVS_NO_FREE_PAGES||errNVS==ESP_ERR_NVS_NEW_VERSION_FOUND)
        adicionarLog("NVS aviso");

    switch(esp_reset_reason()){
        case ESP_RST_BROWNOUT: adicionarLog("RESET BROWNOUT"); break;
        case ESP_RST_PANIC:    adicionarLog("RESET PANIC");    break;
        case ESP_RST_TASK_WDT: adicionarLog("RESET WDT");     break;
        case ESP_RST_SW:       adicionarLog("Reset SW");       break;
        case ESP_RST_POWERON:  adicionarLog("Power ON");       break;
        default:
            snprintf(tempBuf,sizeof(tempBuf),"Reset %d",(int)esp_reset_reason());
            adicionarLog(tempBuf); break;
    }

    esp_task_wdt_reset();
    setupLED(); setupBotao();
    carregarConfigInversores();
    setupReles();
    pinMode(PIN_AC_SINAL_BAIXA, INPUT);
    pinMode(PIN_AC_SINAL_ALTA,  INPUT);
    pinMode(PIN_CASA_SINAL_ALTA,INPUT);
    ledBlink();
    adicionarLog("RRFS v23.1");
    logRAM();
    esp_task_wdt_reset();

    carregarConfigBateria();
    carregarLimiares();
    carregarCalibSensores();
    carregarLimiaresSinal();
    carregarHistorico();
    carregarContadores();
    carregarConfigTempo();
    carregarConfigPrevisao();
    esp_task_wdt_reset();

    ultimoContagem=millis();

    // ── PROTEÇÃO INV.BAIXA (v23.1) ────────────────────────────
    tInicioSistema      = millis();
    sistemaEstabilizado = false;
    tUltimaReconWifi2   = 0;
    // ──────────────────────────────────────────────────────────

    bool usarAP=true;
    if(existeWiFiConfig()){
        String ssid,pass,ip,gw,mask,dns;
        carregarWifi(ssid,pass,ip,gw,mask,dns,usarAP);
        esp_task_wdt_reset();
        if(!usarAP&&ssid.length()>0)
            if(iniciarWiFiNB(ssid,pass,ip,gw,mask,dns)) usarAP=false;
    }
    esp_task_wdt_reset();
    if(usarAP) iniciarAP();
    if(!modoAP&&cfgTempo.ntpAtivo){ esp_task_wdt_reset(); sincronizarNTP(); }
    esp_task_wdt_reset();
    configurarRotas();
    adicionarLog("Sistema pronto v23.1");
    logRAM();
    esp_task_wdt_reset();
}

// ═══════════════════════════════════════════════════════════
// [LOOP]
// ═══════════════════════════════════════════════════════════
void loop(){
    esp_task_wdt_reset();
    verificarReset();
    atualizarLED();

    // Server SEMPRE primeiro
    if(modoAP||WiFi.getMode()==WIFI_AP_STA)
        dnsServer.processNextRequest();
    server.handleClient();
    esp_task_wdt_reset();

    unsigned long agora=millis();

    // ── Leitura sensores e sinais AC (a cada 500ms) ──────────
    // 500ms entre leituras = servidor tem 380ms livres
    // (500ms - 3x40ms janela AC = 380ms para handleClient)
    if(agora-ultimaLeitura>=INTERVALO_LEITURA_MS){
        ultimaLeitura=agora;

        // Sensores de corrente (rápido ~5ms)
        esp_task_wdt_reset();
        lerSensores();
        server.handleClient();
        esp_task_wdt_reset();

        // AC Baixa — 40ms bloqueante (handleClient dentro)
        processarSinalAC(acBaixa, PIN_AC_SINAL_BAIXA, "Baixa");
        server.handleClient();
        esp_task_wdt_reset();

        // AC Alta — 40ms bloqueante (handleClient dentro)
        processarSinalAC(acAlta, PIN_AC_SINAL_ALTA, "Alta");
        server.handleClient();
        esp_task_wdt_reset();

        // Sinal casa DC (rápido)
        processarSinalCasa();
        server.handleClient();
        esp_task_wdt_reset();

        // Controle
        controlarEnergia();
        acumularEnergia();
        server.handleClient();
        esp_task_wdt_reset();
    }

    // ── Controle extra (a cada 1s) ───────────────────────────
    if(agora-ultimoControle>=INTERVALO_CONTROLE_MS){
        ultimoControle=agora;
        controlarLampExt();
        esp_task_wdt_reset();
        server.handleClient();
    }

    // ── NTP (a cada hora) ────────────────────────────────────
    if(cfgTempo.ntpAtivo&&!modoAP&&
        WiFi.status()==WL_CONNECTED&&
        agora-tUltimoNTP>=INTERVALO_NTP_MS){
        sincronizarNTP();
    esp_task_wdt_reset();
        }

        // ── Previsão solar ───────────────────────────────────────
        if(!modoAP&&WiFi.status()==WL_CONNECTED){
            gerenciarPrevisao();
            esp_task_wdt_reset();
        }

        // ── WiFi ─────────────────────────────────────────────────
        gerenciarWiFi();
        esp_task_wdt_reset();

        // ── Salva Ah bateria a cada 5 min ────────────────────────
        {
            static unsigned long tSalva=0;
            if(bateria.calibrada&&agora-tSalva>=300000UL){
                tSalva=agora;
                esp_task_wdt_reset();
                prefs.begin("rrfs",false);
                prefs.putFloat("bat_ah_at",bateria.ahAtual);
                prefs.end();
                esp_task_wdt_reset();
            }
        }

        // ── Log RAM a cada 10 min ────────────────────────────────
        {
            static unsigned long tRAM=0;
            if(agora-tRAM>=600000UL){ tRAM=agora; logRAM(); }
        }

        esp_task_wdt_reset();
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
.inv-card{border-radius:10px;border:2px solid #333;background:#111;padding:12px;margin:5px 0}
.inv-title{font-size:12px;color:#90a4ae;font-weight:bold;margin-bottom:6px}
.inv-val{font-size:22px;font-weight:bold}
.inv-sub{font-size:10px;color:#546e7a;margin-top:4px;line-height:1.6}
.pd{background:#0a1a0a;border:1px solid #1b5e20;border-radius:6px;padding:10px;margin:4px 0}
.pd .dt{font-size:11px;color:#43a047;font-weight:bold;margin-bottom:4px}
.pd .dv{font-size:18px;font-weight:bold;color:#a5d6a7}
.pd .ds{font-size:10px;color:#546e7a;margin-top:2px}
.sig-box{background:#111;border:1px solid #333;border-radius:6px;padding:10px;margin:5px 0}
.sig-lbl{font-size:11px;color:#90a4ae;font-weight:bold;margin-bottom:4px}
.sig-val{font-size:18px;font-weight:bold;margin:3px 0}
.sig-sub{font-size:10px;color:#546e7a}
.pulse{animation:pulse 1s infinite}
@keyframes pulse{0%{opacity:1}50%{opacity:0.5}100%{opacity:1}}
)RAW";

// ═══════════════════════════════════════════════════════════
// [WEB] HELPERS
// ═══════════════════════════════════════════════════════════
String barraProgresso(float pct, const String& label) {
    if (pct<0) pct=0; if (pct>100) pct=100;
    String cor=pct>=50?"#2e7d32":pct>=25?"#ef6c00":"#c62828";
    char buf[200];
    snprintf(buf,sizeof(buf),
        "<div class='bb'><div class='bv' style='width:%d%%;background:%s'>%d%%</div></div>",
        (int)pct,cor.c_str(),(int)pct);
    String h=String(buf);
    if (label.length())
        h+="<div style='font-size:11px;color:#546e7a;text-align:center'>"+label+"</div>";
    return h;
}

String gerarPagina(const String& titulo, const String& conteudo, bool comNav) {
    String html;
    html.reserve(9000);
    html =F("<!DOCTYPE html><html><head>"
            "<meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html+="<title>"+titulo+"</title>";
    html+=F("<style>"); html+=CSS; html+=F("</style></head><body><div class='card'>");
    html+=F("<h2>&#9889; RRFS v23</h2>");
    html+=conteudo;
    if (comNav && logado){
        String pag=server.uri();
        html+=F("<div class='nav'>");
        html+="<a href='/' "          +(pag=="/"         ?String("class='active'"):String(""))+">&#128202;</a>";
        html+="<a href='/bateria' "   +(pag=="/bateria"  ?String("class='active'"):String(""))+">&#128267;</a>";
        html+="<a href='/controle' "  +(pag=="/controle" ?String("class='active'"):String(""))+">&#9881;</a>";
        html+="<a href='/sensores' "  +(pag=="/sensores" ?String("class='active'"):String(""))+">&#128270;</a>";
        html+="<a href='/previsao' "  +(pag=="/previsao" ?String("class='active'"):String(""))+">&#127774;</a>";
        html+="<a href='/historico' " +(pag=="/historico"?String("class='active'"):String(""))+">&#128200;</a>";
        html+="<a href='/config' "    +(pag=="/config"   ?String("class='active'"):String(""))+">&#128736;</a>";
        html+=F("</div>");
    }
    html+=F("<div class='lb'>&#128221; Logs</div>"
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
            "</script></body></html>");
    return html;
}

// ═══════════════════════════════════════════════════════════
// [API] /dados — JSON completo para atualização das páginas
// ═══════════════════════════════════════════════════════════
void handleDados(){
    if (!logado){ server.send(403); return; }

    unsigned long tempoB=(millis()-maqBaixa.tInicioEstado)/1000UL;
    unsigned long tempoA=(millis()-maqAlta.tInicioEstado)/1000UL;

    EstimativaTempo tempo=calcularTempoRestante();

    float ahH=0,ahR=0;
    if (cfgPrevisao.ativa&&previsao.valido){
        ahH=getAhPrevHoraAtual();
        ahR=getAhPrevRestanteDia();
    }

    char buf[1280];
    snprintf(buf,sizeof(buf),
        "{"
        "\"ip\":%.2f,\"ic\":%.2f,\"ib\":%.2f,"
        "\"wp\":%.0f,\"wc\":%.0f,\"wb\":%.0f,"
        "\"pct\":%.1f,\"ah_at\":%.2f,\"ah_disp\":%.2f,"
        "\"vnom\":%.1f,\"calib\":%s,"
        "\"invb\":%s,\"inva\":%s,\"le\":%s,"
        "\"ntp\":%s,\"prev_ok\":%s,"
        "\"ac_baixa_mv\":%.0f,\"ac_baixa_ok\":%s,"
        "\"ac_alta_mv\":%.0f,\"ac_alta_ok\":%s,"
        "\"casa_mv\":%.0f,\"casa_pede\":%s,"
        "\"casa_lig\":%s,\"casa_des\":%s,"
        "\"est_baixa\":%d,\"nome_baixa\":\"%s\","
        "\"est_alta\":%d,\"nome_alta\":\"%s\","
        "\"tempo_b\":%lu,\"tempo_a\":%lu,"
        "\"lig_b\":%d,\"fal_b\":%d,"
        "\"lig_a\":%d,\"fal_a\":%d,"
        "\"lim_inv_a\":%.0f,\"lim_lamp\":%.0f,"
        "\"tempo_val\":%s,\"tempo_h\":%d,\"tempo_m\":%d,\"tempo_carg\":%s,"
        "\"mv_painel\":%d,\"mv_casa\":%d,"
        "\"ah_prev_h\":%.2f,\"ah_prev_r\":%.2f,"
        "\"modo_teste\":%s,"
        "\"ram\":%.1f,"
        "\"uptime\":%lu,"
        "\"data_hora\":\"%s\""
        "}",
        sensores.correntePainel, sensores.correnteCasa, sensores.correnteBateria,
        sensores.potenciaPainel, sensores.potenciaCasa, fabsf(sensores.potenciaBateria),
        bateria.percentualAtual, bateria.ahAtual, bateria.ahDisponivel,
        bateria.tensaoNominal,
        bateria.calibrada?"true":"false",
        estados.invBaixa?"true":"false",
        estados.invAlta ?"true":"false",
        estados.lampExt ?"true":"false",
        ntpSincronizado?"true":"false",
        previsao.valido?"true":"false",
        acBaixa.mediaMV,
        acBaixa.presente?"true":"false",
        acAlta.mediaMV,
        acAlta.presente?"true":"false",
        casaSinal.mediaMV,
        casaSinal.pedindo?"true":"false",
        casaSinal.aguardandoLigar?"true":"false",
        casaSinal.aguardandoDesligar?"true":"false",
        (int)maqBaixa.estado, nomeEstadoBaixa(maqBaixa.estado).c_str(),
        (int)maqAlta.estado,  nomeEstadoAlta(maqAlta.estado).c_str(),
        tempoB, tempoA,
        maqBaixa.totalLigadas, maqBaixa.totalFalhas,
        maqAlta.totalLigadas,  maqAlta.totalFalhas,
        limiares.pctMinInvAlta, limiares.pctLampExt,
        tempo.valido?"true":"false",
        tempo.horas, tempo.minutos,
        tempo.carregando?"true":"false",
        sensores.sinalPainel_mV, sensores.sinalCasa_mV,
        ahH, ahR,
        modoTeste?"true":"false",
        esp_get_free_heap_size()/1024.0f,
        millis()/1000UL,
        formatarDataHora().c_str()
    );
    server.send(200,"application/json",buf);
}

// ═══════════════════════════════════════════════════════════
// [AUTH]
// ═══════════════════════════════════════════════════════════
void handleSetupLogin(){
    if (existeCredenciais()&&existeWiFiConfig()){server.sendHeader("Location","/login");server.send(302);return;}
    if (existeCredenciais()&&!existeWiFiConfig()){server.sendHeader("Location","/setup-wifi");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        String u=server.arg("user"),p=server.arg("pass"),cf=server.arg("conf");
        if (u.length()<3)      msg="<div class='er'>Min 3 chars usuario</div>";
        else if (p.length()<4) msg="<div class='er'>Min 4 chars senha</div>";
        else if (p!=cf)        msg="<div class='er'>Senhas nao conferem</div>";
        else{ salvarCredenciais(u,p); server.sendHeader("Location","/setup-wifi"); server.send(302); return; }
    }
    String c=F("<div class='nf'>&#8505; Primeiro acesso.</div><h3>&#128274; Criar Conta</h3>");
    c+=msg;
    c+=F("<form method='POST' action='/setup-login'>"
         "<label>Usuario:</label><input type='text' name='user' maxlength='32' required autocomplete='off'>"
         "<label>Senha:</label><input type='password' name='pass' maxlength='32' required>"
         "<label>Confirmar:</label><input type='password' name='conf' maxlength='32' required>"
         "<button type='submit'>Criar &#8594;</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Setup",c,false));
}

void handleSetupWifi(){
    if (!existeCredenciais()){server.sendHeader("Location","/setup-login");server.send(302);return;}
    if (existeWiFiConfig()&&!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        String modo=server.arg("modo");
        if (modo=="ap"){
            salvarWifi("","","","","","",true);
            server.send(200,"text/html;charset=utf-8",
                        gerarPagina("Salvo",F("<div class='ok'>&#10003; Reiniciando...</div>"
                        "<div class='nf'>SSID: <b>RRFS Energia</b> | IP: <b>10.0.0.1</b></div>"),false));
            delay(2000); ESP.restart(); return;
        }
        String ssid=server.arg("ssid"),wp=server.arg("wpass"),ip=server.arg("ip"),
        gw=server.arg("gw"),mask=server.arg("mask"),dns=server.arg("dns");
        if (ssid.isEmpty())     msg="<div class='er'>Selecione uma rede</div>";
        else if (ip.length()<7) msg="<div class='er'>Informe o IP</div>";
        else if (gw.length()<7) msg="<div class='er'>Informe o Gateway</div>";
        else{
            salvarWifi(ssid,wp,ip,gw,mask,dns,false);
            String ct="<div class='ok'>&#10003; Acesse: http://"+ip+"</div><div class='nf'>Reiniciando...</div>";
            server.send(200,"text/html;charset=utf-8",gerarPagina("Salvo",ct,false));
            delay(2000); ESP.restart(); return;
        }
    }

    // ═══════════════════════════════════════════════════════
    // SCAN DE REDES — CORRIGIDO v23.1
    // Bug: scan nao funciona em modo AP puro (WIFI_AP)
    //      e tentativas de conexao STA bloqueiam o scan
    // Fix: forca WIFI_AP_STA + para tentativas de conexao
    //      antes de escanear
    // ═══════════════════════════════════════════════════════
    static int16_t nRedes=-1; static bool scanIniciado=false;

    if (!scanIniciado){
        // 1) Se estiver em AP puro, muda para AP_STA (AP continua ativo)
        if (WiFi.getMode()==WIFI_AP){
            WiFi.mode(WIFI_AP_STA);
            delay(100);
            adicionarLog("Scan: modo AP_STA");
        }
        // 2) Para tentativas de conexao STA que bloqueiam o scan
        //    (AP emergencial fica tentando senha antiga em loop)
        if (WiFi.status()!=WL_CONNECTED){
            WiFi.setAutoReconnect(false);
            WiFi.disconnect();
            delay(100);
        }
        esp_task_wdt_reset();
        // 3) Inicia scan assincrono
        WiFi.scanDelete();
        WiFi.scanNetworks(true,false,false,300);
        scanIniciado=true; nRedes=-1;
        adicionarLog("Scan WiFi iniciado");
    }
    nRedes=WiFi.scanComplete();

    String c=F("<div class='nf'>&#8505; IP fixo obrigatorio.</div><h3>&#128246; Rede</h3>");
    c+=msg;
    c+=F("<form method='POST' action='/setup-wifi'>"
    "<label>Modo:</label>"
    "<select name='modo' id='modo' onchange='toggleModo()'>"
    "<option value='wifi'>Conectar ao WiFi</option>"
    "<option value='ap'>Ponto de Acesso</option></select>"
    "<div id='dw'><label>Rede:</label><select name='ssid'>");

    bool escaneando=false;
    if (nRedes==WIFI_SCAN_RUNNING){
        // Scan em andamento — recarrega sozinho em 3s
        c+=F("<option>Escaneando... aguarde</option>");
        escaneando=true;
    }
    else if (nRedes==WIFI_SCAN_FAILED){
        // Scan falhou — permite tentar de novo
        c+=F("<option value=''>Falha no scan - clique Atualizar</option>");
        scanIniciado=false;
        adicionarLog("Scan WiFi FALHOU");
    }
    else if (nRedes==0){
        c+=F("<option value=''>Nenhuma rede encontrada</option>");
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
        adicionarLog("Scan: 0 redes");
    }
    else if (nRedes>0){
        for (int i=0;i<nRedes;i++){
            String q=WiFi.RSSI(i)>-60?"Bom":WiFi.RSSI(i)>-75?"Regular":"Fraco";
            char buf[32]; snprintf(buf,sizeof(buf)," [%s %ddBm]",q.c_str(),WiFi.RSSI(i));
            c+="<option value='"+WiFi.SSID(i)+"'>"+WiFi.SSID(i)+String(buf)+"</option>";
        }
        snprintf(tempBuf,sizeof(tempBuf),"Scan: %d redes",nRedes);
        adicionarLog(tempBuf);
        WiFi.scanDelete(); scanIniciado=false; nRedes=-1;
    }

    c+=F("</select>"
    "<button type='button' class='btn-o btn-sm' onclick='location.reload()'"
    " style='margin-top:8px'>&#128259; Atualizar</button>"
    "<label>Senha WiFi:</label><input type='password' name='wpass' placeholder='Vazio se aberta'>"
    "<label>IP Fixo:</label><input type='text' name='ip' required placeholder='192.168.1.100'>"
    "<label>Gateway:</label><input type='text' name='gw' required placeholder='192.168.1.1'>"
    "<label>Mascara:</label><input type='text' name='mask' required value='255.255.255.0'>"
    "<label>DNS:</label><input type='text' name='dns' required value='8.8.8.8'></div>"
    "<button type='submit' class='btn-g'>&#128190; Salvar</button></form>");
    if (logado) c+=F("<button onclick='location.href=\"/\"' class='btn-s' style='margin-top:8px'>&#8592; Voltar</button>");
    c+=F("<script>function toggleModo(){"
    "document.getElementById('dw').style.display="
    "document.getElementById('modo').value=='wifi'?'block':'none';}</script>");

    // Auto-recarrega enquanto o scan estiver rodando
    if (escaneando)
        c+=F("<script>setTimeout(function(){location.reload();},3000);</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Rede",c,false));
}

void handleLogin(){
    if (!existeCredenciais()){server.sendHeader("Location","/setup-login");server.send(302);return;}
    if (logado){server.sendHeader("Location","/");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        if (verificarLogin(server.arg("user"),server.arg("pass"))){
            logado=true; adicionarLog("Login OK"); logRAM();
            server.sendHeader("Location","/"); server.send(302); return;
        }
        adicionarLog("Login FALHOU");
        msg=F("<div class='er'>&#10007; Usuario/senha incorretos</div>");
    }
    String c=F("<div class='nf'>&#128274; Acesso seguro</div><h3>Entrar</h3>");
    c+=msg;
    c+=F("<form method='POST' action='/login'>"
         "<label>Usuario:</label><input type='text' name='user' required autocomplete='off'>"
         "<label>Senha:</label><input type='password' name='pass' required>"
         "<button type='submit'>&#128275; Entrar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Login",c,false));
}

void handleLogout(){
    if (logado){ adicionarLog("Logout"); logado=false; }
    server.sendHeader("Location","/login"); server.send(302);
}

// ═══════════════════════════════════════════════════════════
// [DASH] PÁGINA PRINCIPAL
// ═══════════════════════════════════════════════════════════
void handleRoot(){
    if (!existeCredenciais()){server.sendHeader("Location","/setup-login");server.send(302);return;}
    if (!existeWiFiConfig()) {server.sendHeader("Location","/setup-wifi"); server.send(302);return;}
    if (!logado)             {server.sendHeader("Location","/login");       server.send(302);return;}

    String c="";

    if (!bateria.calibrada)
        c+=F("<div class='wn' style='font-size:11px'>&#9888; Bateria nao calibrada. "
             "<a href='/bateria' style='color:#ffe0b2'>Calibrar</a></div>");

    // Data/hora
    c+=F("<div style='display:flex;justify-content:space-between;align-items:center;"
         "font-size:11px;color:#546e7a;margin-bottom:6px'>"
         "<span id='dh_txt'>&#128336; --/--/---- --:--</span>"
         "<span id='upd_ind' style='color:#43a047;font-size:10px'>&#9679; ao vivo</span>"
         "</div>");

    // ── Inversores ──────────────────────────────────────────
    c+=F("<h3>&#9889; Inversores</h3>"
         "<div class='grid2'>"
         // Inv Baixa
         "<div id='card_b' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#128268; Inv. Baixa</span>"
         "<span id='st_b' class='cs'>--</span>"
         "</div>"
         "<div id='sub_b' class='ci'>--</div>"
         "</div>"
         // Inv Alta
         "<div id='card_a' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#9889; Inv. Alta</span>"
         "<span id='st_a' class='cs'>--</span>"
         "</div>"
         "<div id='sub_a' class='ci'>--</div>"
         "</div>"
         "</div>");

    // ── Sinal Casa DC ───────────────────────────────────────
    c+=F("<h3>&#127968; Sinal Casa</h3>"
         "<div class='sig-box'>"
         "<div class='cr'>"
         "<span class='sig-lbl'>&#128246; GPIO36 (DC)</span>"
         "<span id='casa_txt' class='sig-val'>--</span>"
         "</div>"
         "<div id='casa_sub' class='sig-sub'>-- mV</div>"
         "<div class='bb' style='margin-top:4px'>"
         "<div id='casa_bv' class='bv' style='width:0%'></div></div>"
         "</div>");

    // ── Sinais AC ───────────────────────────────────────────
    c+=F("<h3>&#128246; Sinais AC</h3>"
         "<div class='grid2'>"
         "<div class='sig-box'>"
         "<div class='sig-lbl'>Inv. Baixa (GPIO32)</div>"
         "<div id='ac_b_txt' class='sig-val' style='font-size:14px'>--</div>"
         "<div id='ac_b_sub' class='sig-sub'>--</div>"
         "<div class='bb'><div id='ac_b_bv' class='bv' style='width:0%'></div></div>"
         "</div>"
         "<div class='sig-box'>"
         "<div class='sig-lbl'>Inv. Alta (GPIO33)</div>"
         "<div id='ac_a_txt' class='sig-val' style='font-size:14px'>--</div>"
         "<div id='ac_a_sub' class='sig-sub'>--</div>"
         "<div class='bb'><div id='ac_a_bv' class='bv' style='width:0%'></div></div>"
         "</div>"
         "</div>");

    // ── Previsão solar ──────────────────────────────────────
    if (cfgPrevisao.ativa&&cfgPrevisao.configurada){
        c+=F("<div id='prev_box' class='pd' style='display:none'>"
             "<div class='dt'>&#127774; Previsao Solar</div>"
             "<div class='grid2'>"
             "<div style='text-align:center'>"
             "<div style='font-size:10px;color:#546e7a'>Esta hora</div>"
             "<div id='prev_h' class='dv'>--</div></div>"
             "<div style='text-align:center'>"
             "<div style='font-size:10px;color:#546e7a'>Restante hoje</div>"
             "<div id='prev_r' class='dv'>--</div></div>"
             "</div>"
             "<div class='ds'><a href='/previsao' style='color:#43a047'>Ver completo</a></div>"
             "</div>");
    }

    // ── SVG Fluxo ───────────────────────────────────────────
    c+=F("<h3 style='text-align:center;margin:8px 0 6px'>&#9889; Fluxo</h3>"
         "<svg viewBox='0 0 340 220' style='width:100%;height:auto;display:block'>"
         "<path id='ln_pb' d='M110,50 C160,20 200,20 240,50' fill='none' stroke='#333' stroke-width='3'/>"
         "<path id='ln_pc' d='M90,85 C80,125 90,145 130,168' fill='none' stroke='#333' stroke-width='3'/>"
         "<path id='ln_bc' d='M255,85 C260,125 250,145 210,168' fill='none' stroke='#333' stroke-width='3'/>"
         // Painel
         "<g><rect x='20' y='10' width='100' height='80' rx='12' fill='#181818' id='rect_p' stroke='#455a64' stroke-width='2'/>"
         "<text x='70' y='36' text-anchor='middle' font-size='20'>&#9728;</text>"
         "<text x='70' y='50' text-anchor='middle' font-size='9' fill='#78909c'>PAINEL</text>"
         "<text id='sv_ip' x='70' y='65' text-anchor='middle' font-size='13' fill='#455a64' font-weight='bold'>0.0A</text>"
         "<text id='sv_wp' x='70' y='80' text-anchor='middle' font-size='11' fill='#455a64'>0W</text>"
         "</g>"
         // Bateria
         "<g><rect x='220' y='10' width='100' height='80' rx='12' fill='#181818' id='rect_b' stroke='#455a64' stroke-width='2'/>"
         "<text id='sv_ico_b' x='270' y='34' text-anchor='middle' font-size='18'>&#128267;</text>"
         "<text x='270' y='48' text-anchor='middle' font-size='9' fill='#78909c'>BATERIA</text>"
         "<text id='sv_pct' x='270' y='63' text-anchor='middle' font-size='13' fill='#455a64' font-weight='bold'>--</text>"
         "<text id='sv_ib' x='270' y='78' text-anchor='middle' font-size='11' fill='#455a64'>--</text>"
         "</g>"
         // Casa
         "<g><rect id='rect_c' x='105' y='163' width='130' height='55' rx='10' fill='#181818' stroke='#3a3a3a' stroke-width='2'/>"
         "<text x='130' y='195' font-size='18'>&#127968;</text>"
         "<text id='sv_ico_inv' x='145' y='175' text-anchor='start' font-size='9' fill='#78909c'>CASA</text>"
         "<text id='sv_ic' x='228' y='190' text-anchor='end' font-size='13' fill='#3a3a3a' font-weight='bold'>--</text>"
         "<text id='sv_wc' x='228' y='205' text-anchor='end' font-size='11' fill='#3a3a3a'>--</text>"
         "</g></svg>");

    // ── Barra bateria ───────────────────────────────────────
    c+=F("<div style='margin:8px 0'>"
         "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:4px'>"
         "<span style='font-size:11px;color:#78909c'>&#128267; Bateria</span>"
         "<span id='bat_tempo' style='font-size:11px;font-weight:bold;color:#78909c'>--</span>"
         "</div>"
         "<div class='bb'><div id='bat_bv' class='bv' style='width:0%'>0%</div></div>"
         "<div id='bat_ah' style='font-size:11px;color:#546e7a;text-align:center;margin-top:2px'>--</div>"
         "</div>");

    // ── Grid correntes ──────────────────────────────────────
    c+=F("<div class='grid3' style='gap:6px;margin:4px 0'>"
         "<div class='sc' style='padding:8px;text-align:center'>"
         "<div class='lbl'>&#9728; Painel</div>"
         "<div id='cc_p' style='font-size:14px;font-weight:bold'>--</div>"
         "<div id='cw_p' style='font-size:11px;color:#78909c'>--</div>"
         "</div>"
         "<div class='sc' style='padding:8px;text-align:center'>"
         "<div class='lbl'>&#127968; Casa</div>"
         "<div id='cc_c' style='font-size:14px;font-weight:bold;color:#4fc3f7'>--</div>"
         "<div id='cw_c' style='font-size:11px;color:#78909c'>--</div>"
         "</div>"
         "<div class='sc' style='padding:8px;text-align:center'>"
         "<div class='lbl'>&#128267; Bat</div>"
         "<div id='cc_b' style='font-size:14px;font-weight:bold'>--</div>"
         "<div id='cw_b' style='font-size:11px;color:#78909c'>--</div>"
         "</div>"
         "</div>");

    // ── Info sistema ────────────────────────────────────────
    c+=F("<div id='sys_info' class='nf' style='font-size:10px'>Carregando...</div>");

    // ── JAVASCRIPT ──────────────────────────────────────────
    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function sC(id,c){var e=document.getElementById(id);if(e)e.style.color=c;}"
    "function sBc(id,c){var e=document.getElementById(id);if(e)e.style.borderColor=c;}"
    "function sAttr(id,a,v){var e=document.getElementById(id);if(e)e.setAttribute(a,v);}"

    "function atualizarDash(d){"

    // Data/hora
    "sT('dh_txt','&#128336; '+d.data_hora+(d.ntp?' <span style=\"color:#43a047\">&#9679;NTP</span>':''));"

    // Inv Baixa
    "var cB=d.invb?'#4fc3f7':'#546e7a';"
    "sBc('card_b',cB);"
    "sT('st_b','<span style=\"color:'+cB+'\">'+(d.invb?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "sT('sub_b',d.nome_baixa+' | '+d.tempo_b+'s | F:'+d.fal_b);"

    // Inv Alta
    "var cA=d.inva?'#fdd835':'#546e7a';"
    "sBc('card_a',cA);"
    "sT('st_a','<span style=\"color:'+cA+'\">'+(d.inva?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "var subA=d.nome_alta+' | '+d.tempo_a+'s | F:'+d.fal_a;"
    "subA+=d.casa_pede?' | Casa: pede':' | Casa: nao pede';"
    "sT('sub_a',subA);"

    // Sinal casa DC
    "var cCasa=d.casa_pede?'#43a047':d.casa_lig?'#f9a825':'#546e7a';"
    "var tCasa=d.casa_pede?'&#9679; PEDINDO':d.casa_lig?'&#9650; Aguard...':'&#9675; Inativo';"
    "sT('casa_txt','<span style=\"color:'+cCasa+'\">'+tCasa+'</span>');"
    "sT('casa_sub',d.casa_mv.toFixed(0)+' mV | Limiar: 1000mV');"
    "var bCasa=document.getElementById('casa_bv');"
    "if(bCasa){var pC=Math.min((d.casa_mv/3300)*100,100);bCasa.style.width=pC+'%';bCasa.style.background=cCasa;}"

    // AC Baixa
    "var cAcB=d.ac_baixa_ok?'#43a047':'#546e7a';"
    "sT('ac_b_txt','<span style=\"color:'+cAcB+'\">'+(d.ac_baixa_ok?'&#9679; OK':'&#9675; SEM AC')+'</span>');"
    "sT('ac_b_sub',d.ac_baixa_mv.toFixed(0)+' mV');"
    "var bAcB=document.getElementById('ac_b_bv');"
    "if(bAcB){bAcB.style.width=Math.min((d.ac_baixa_mv/3300)*100,100)+'%';bAcB.style.background=cAcB;}"

    // AC Alta
    "var cAcA=d.ac_alta_ok?'#43a047':'#546e7a';"
    "sT('ac_a_txt','<span style=\"color:'+cAcA+'\">'+(d.ac_alta_ok?'&#9679; OK':'&#9675; SEM AC')+'</span>');"
    "sT('ac_a_sub',d.ac_alta_mv.toFixed(0)+' mV');"
    "var bAcA=document.getElementById('ac_a_bv');"
    "if(bAcA){bAcA.style.width=Math.min((d.ac_alta_mv/3300)*100,100)+'%';bAcA.style.background=cAcA;}"

    // Previsão
    "var pBox=document.getElementById('prev_box');"
    "if(pBox){"
    "  if(d.prev_ok&&(d.ah_prev_h>0||d.ah_prev_r>0)){"
    "    pBox.style.display='block';"
    "    sT('prev_h',d.ah_prev_h.toFixed(1)+' Ah');"
    "    sT('prev_r',d.ah_prev_r.toFixed(1)+' Ah');"
    "  } else pBox.style.display='none';"
    "}"

    // SVG
    "var painelGer=(d.ip>0.5);"
    "var batCarg=(d.ib>0.3);"
    "var batDesc=(d.ib<-0.3);"
    "var cP=painelGer?'#fdd835':'#455a64';"
    "var cBat=!d.calib?'#455a64':d.pct>=50?'#43a047':d.pct>=25?'#ef6c00':'#c62828';"
    "var cCas=d.inva?'#fdd835':d.invb?'#4fc3f7':'#3a3a3a';"
    "var lPB=(painelGer&&batCarg)?'#43a047':'#333';"
    "var lPC=(painelGer&&d.ic>0.3)?'#fdd835':'#333';"
    "var lBC=(batDesc&&d.ic>0.3)?'#ef6c00':'#333';"
    "sAttr('rect_p','stroke',cP);sAttr('rect_b','stroke',cBat);sAttr('rect_c','stroke',cCas);"
    "sAttr('ln_pb','stroke',lPB);sAttr('ln_pc','stroke',lPC);sAttr('ln_bc','stroke',lBC);"
    "var svIp=document.getElementById('sv_ip');"
    "if(svIp){svIp.textContent=painelGer?d.ip.toFixed(1)+'A':'0.0A';svIp.style.fill=cP;}"
    "var svWp=document.getElementById('sv_wp');"
    "if(svWp){svWp.textContent=painelGer?Math.round(d.wp)+'W':'0W';svWp.style.fill=cP;}"
    "var icoB=batCarg?'&#9650;&#128267;':batDesc?'&#9660;&#128267;':'&#128267;';"
    "sT('sv_ico_b',icoB);"
    "var svPct=document.getElementById('sv_pct');"
    "if(svPct){svPct.textContent=d.calib?d.pct.toFixed(0)+'%':'N/Cal';svPct.style.fill=cBat;}"
    "var siB=batCarg?'+':batDesc?'-':'~';"
    "var svIb=document.getElementById('sv_ib');"
    "if(svIb){svIb.textContent=siB+Math.abs(d.ib).toFixed(1)+'A/'+Math.round(Math.abs(d.wb))+'W';svIb.style.fill=cBat;}"
    "var icoInv=d.inva?'&#9889;':d.invb?'&#128268;':'&#9711;';"
    "sT('sv_ico_inv','CASA '+icoInv);"
    "var svIc=document.getElementById('sv_ic');"
    "if(svIc){svIc.textContent=d.ic.toFixed(1)+'A';svIc.style.fill=cCas;}"
    "var svWc=document.getElementById('sv_wc');"
    "if(svWc){svWc.textContent=Math.round(d.wc)+'W';svWc.style.fill=cCas;}"

    // Barra bateria
    "var pctB=d.calib?Math.max(0,Math.min(100,d.pct)):0;"
    "var corBat=pctB>=50?'#2e7d32':pctB>=25?'#ef6c00':'#c62828';"
    "var batBv=document.getElementById('bat_bv');"
    "if(batBv){batBv.style.width=pctB+'%';batBv.style.background=corBat;batBv.textContent=Math.round(pctB)+'%';}"
    "sT('bat_ah',d.calib?d.ah_at.toFixed(1)+'Ah de '+d.ah_disp.toFixed(1)+'Ah':'Nao calibrada');"
    "var btEl=document.getElementById('bat_tempo');"
    "if(btEl){"
    "  if(d.tempo_val){"
    "    btEl.style.color=d.tempo_carg?'#43a047':'#ef6c00';"
    "    btEl.innerHTML=(d.tempo_carg?'&#9650;':'&#9660;')+d.tempo_h+'h'+String(d.tempo_m).padStart(2,'0')+'m';"
    "  } else btEl.textContent='';"
    "}"

    // Grid correntes
    "var cP2=painelGer?'#fdd835':'#455a64';"
    "var cB2=batCarg?'#43a047':batDesc?'#ef6c00':'#fdd835';"
    "sT('cc_p','<span style=\"color:'+cP2+'\">'+(painelGer?d.ip.toFixed(1)+'A':'0.0A')+'</span>');"
    "sT('cw_p',painelGer?Math.round(d.wp)+'W':'0W');"
    "sT('cc_c',d.ic.toFixed(1)+'A');"
    "sT('cw_c',Math.round(d.wc)+'W');"
    "var siBI=(d.ib>0.3?'+':d.ib<-0.3?'-':'~')+Math.abs(d.ib).toFixed(1)+'A';"
    "sT('cc_b','<span style=\"color:'+cB2+'\">'+siBI+'</span>');"
    "sT('cw_b',Math.round(Math.abs(d.wb))+'W');"

    // Info sistema
    "var si='&#128246; '+d.ram.toFixed(1)+'KB RAM | Up '+d.uptime+'s';"
    "if(d.modo_teste)si+=' | <span style=\"color:#ef6c00\">MODO TESTE</span>';"
    "sT('sys_info',si);"

    // Indicador ao vivo
    "var ind=document.getElementById('upd_ind');"
    "if(ind){ind.style.opacity='1';setTimeout(function(){if(ind)ind.style.opacity='0.3';},300);}"
    "}"

    // Loop fetch
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarDash(JSON.parse(x.responseText));}catch(e){console.log(e);}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Painel",c));
}

// ═══════════════════════════════════════════════════════════
// [CONTROLE] PÁGINA DE CONTROLE — substitui /inversores
// ═══════════════════════════════════════════════════════════
void handleControle(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String msg="";

    if (server.method()==HTTP_POST){
        String acao=server.arg("acao");

        if (acao=="limiares"){
            float la=server.arg("lim_inv_a").toFloat();
            float ll=server.arg("lim_lamp").toFloat();
            if (la<0||la>95) msg="<div class='er'>Inv.Alta: 0-95%</div>";
            else if (ll<0||ll>95) msg="<div class='er'>Lampada: 0-95%</div>";
            else{
                limiares.pctMinInvAlta=la;
                limiares.pctLampExt   =ll;
                salvarLimiares();
                adicionarLog("Limiares salvos");
                msg="<div class='ok'>&#10003; Limiares salvos!</div>";
            }
        } else if (acao=="reles"){
            cfgInv.invertReleInvAlta =(server.arg("inv_inva")=="1");
            cfgInv.invertReleInvBaixa=(server.arg("inv_invb")=="1");
            cfgInv.invertReleLampExt =(server.arg("inv_lamp")=="1");
            salvarConfigInversores();
            // Reaplicar estado atual nos relés
            if (estados.invAlta)  ligarInvAlta();  else desligarInvAlta();
            if (estados.invBaixa) ligarInvBaixa(); else desligarInvBaixa();
            if (estados.lampExt)  ligarLampExt();  else desligarLampExt();
            adicionarLog("Inversao reles salva");
            msg="<div class='ok'>&#10003; Inversao salva!</div>";
        } else if (acao=="lim_ac"){
            int lim=server.arg("lim_ac").toInt();
            if (lim<50||lim>2000) msg="<div class='er'>Limiar 50-2000mV</div>";
            else{
                limiarSinal.limiarAcMV=lim;
                salvarLimiaresSinal();
                adicionarLog("Limiar AC salvo");
                msg="<div class='ok'>&#10003; Limiar AC salvo!</div>";
            }
        } else if (acao=="entrar_teste"){
            modoTeste=true; tInicioTeste=millis(); TIMEOUT_TESTE_MS=60000UL;
            adicionarLog("Modo teste ATIVO 60s");
            msg="<div class='ok'>&#10003; Modo teste ativo 60s!</div>";
        } else if (acao=="sair_teste"){
            modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
            desligarInvAlta(); desligarInvBaixa(); desligarLampExt();
            maqBaixa.estado=INV_B_DESLIGADO; maqBaixa.tInicioEstado=millis();
            maqAlta.estado =INV_A_DESLIGADO; maqAlta.tInicioEstado =millis();
            adicionarLog("Modo teste encerrado");
            msg="<div class='ok'>&#10003; Teste encerrado!</div>";
        } else if (acao=="teste_b_on"){
            if (!modoTeste) msg="<div class='er'>Ative o modo teste!</div>";
            else{ ligarInvBaixa(); adicionarLog("TESTE: InvBaixa ON"); msg="<div class='ok'>InvBaixa ON</div>"; }
        } else if (acao=="teste_b_off"){
            if (!modoTeste) msg="<div class='er'>Ative o modo teste!</div>";
            else{ desligarInvBaixa(); adicionarLog("TESTE: InvBaixa OFF"); msg="<div class='ok'>InvBaixa OFF</div>"; }
        } else if (acao=="teste_a_on"){
            if (!modoTeste) msg="<div class='er'>Ative o modo teste!</div>";
            else{ ligarInvAlta(); adicionarLog("TESTE: InvAlta ON"); msg="<div class='ok'>InvAlta ON</div>"; }
        } else if (acao=="teste_a_off"){
            if (!modoTeste) msg="<div class='er'>Ative o modo teste!</div>";
            else{ desligarInvAlta(); adicionarLog("TESTE: InvAlta OFF"); msg="<div class='ok'>InvAlta OFF</div>"; }
        } else if (acao=="teste_lamp_on"){
            if (!modoTeste) msg="<div class='er'>Ative o modo teste!</div>";
            else{ ligarLampExt(); adicionarLog("TESTE: Lamp ON"); msg="<div class='ok'>Lamp ON</div>"; }
        } else if (acao=="teste_lamp_off"){
            if (!modoTeste) msg="<div class='er'>Ative o modo teste!</div>";
            else{ desligarLampExt(); adicionarLog("TESTE: Lamp OFF"); msg="<div class='ok'>Lamp OFF</div>"; }
        }
    }

    // Timeout teste
    if (modoTeste && millis()-tInicioTeste>=TIMEOUT_TESTE_MS){
        modoTeste=false; adicionarLog("Modo teste: timeout");
    }

    String c=msg;
    c+=F("<div class='nf' style='font-size:11px'>&#9889; <b>Controle v23</b></div>");

    // ── Status ao vivo ───────────────────────────────────────
    c+=F("<h3>&#128202; Status ao Vivo</h3>"
         "<div class='grid2'>"
         // Inv Baixa
         "<div id='ctrl_card_b' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#128268; Inv. Baixa</span>"
         "<span id='ctrl_st_b' class='cs'>--</span>"
         "</div>"
         "<div id='ctrl_sub_b' class='ci'>--</div>"
         "</div>"
         // Inv Alta
         "<div id='ctrl_card_a' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#9889; Inv. Alta</span>"
         "<span id='ctrl_st_a' class='cs'>--</span>"
         "</div>"
         "<div id='ctrl_sub_a' class='ci'>--</div>"
         "</div>"
         "</div>"
         // Sinal casa e Lamp
         "<div class='grid2'>"
         "<div id='ctrl_card_casa' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#127968; Sinal Casa</span>"
         "<span id='ctrl_casa' class='cs'>--</span>"
         "</div>"
         "<div id='ctrl_casa_mv' class='ci'>-- mV</div>"
         "</div>"
         "<div id='ctrl_card_lamp' class='cg'>"
         "<div class='cr'>"
         "<span class='cn'>&#128161; Lampada</span>"
         "<span id='ctrl_lamp' class='cs'>--</span>"
         "</div>"
         "</div>"
         "</div>"
         // Bateria
         "<div class='sc' style='margin-top:6px'>"
         "<div class='lbl'>&#128267; Bateria</div>"
         "<div class='bb'><div id='ctrl_bat_bv' class='bv' style='width:0%'>0%</div></div>"
         "<div id='ctrl_bat_info' style='font-size:11px;color:#546e7a;text-align:center'>--</div>"
         "</div>");

    // ── Limiares ─────────────────────────────────────────────
    c+=F("<h3>&#9881; Limiares de Energia</h3>"
         "<div class='nf' style='font-size:11px'>"
         "&#128267; Define o % minimo da bateria para ligar cada dispositivo.</div>"
         "<form method='POST' action='/controle'>"
         "<input type='hidden' name='acao' value='limiares'>"
         "<label>% Minimo para Inv. Alta (padrao: 20%):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",limiares.pctMinInvAlta);
    c+="<input type='number' name='lim_inv_a' min='0' max='95' step='1' required value='"+String(tempBuf)+"'>";
    c+=F("<label>% Minimo para Lampada Ext (padrao: 20%):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",limiares.pctLampExt);
    c+="<input type='number' name='lim_lamp' min='0' max='95' step='1' required value='"+String(tempBuf)+"'>";

    // Mostra limiar atual ao vivo
    c+=F("<div id='ctrl_lim_info' class='nf' style='font-size:11px;margin-top:6px'>"
         "Bat atual: <b id='ctrl_pct_live'>--</b>% | "
         "InvAlta: <b id='ctrl_lim_a_live'>--</b>% | "
         "Lamp: <b id='ctrl_lim_l_live'>--</b>%"
         "</div>");
    c+=F("<button type='submit' class='btn-g'>&#128190; Salvar Limiares</button></form>");

    // ── Limiar AC ────────────────────────────────────────────
    c+=F("<h3>&#128246; Limiar Sinal AC</h3>"
         "<div class='nf' style='font-size:11px'>"
         "Aplica-se a ambos os inversores (Baixa e Alta).<br>"
         "Liga acima de limiar+50mV | Desliga abaixo de limiar-40mV</div>"
         "<form method='POST' action='/controle'>"
         "<input type='hidden' name='acao' value='lim_ac'>"
         "<label>Limiar AC (mV):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",limiarSinal.limiarAcMV);
    c+="<input type='number' name='lim_ac' min='50' max='2000' step='10' required value='"+String(tempBuf)+"'>";
    {
        char hBuf[80];
        snprintf(hBuf,sizeof(hBuf),
            "Liga > %dmV | Desliga < %dmV",
            limiarSinal.limiarAcMV+AC_HISTERESE_LIGA_MV,
            limiarSinal.limiarAcMV-AC_HISTERESE_DESLIGA_MV);
        c+="<div class='nf' style='font-size:11px'>&#9432; "+String(hBuf)+"</div>";
    }
    c+=F("<button type='submit' class='btn-g'>&#128190; Salvar Limiar AC</button></form>");

    // ── Inversão de relés ─────────────────────────────────────
    c+=F("<h3>&#128260; Inversao de Reles</h3>"
         "<div class='nf' style='font-size:11px'>"
         "<b>Normal:</b> HIGH=Liga | <b>Invertido:</b> LOW=Liga</div>"
         "<form method='POST' action='/controle'>"
         "<input type='hidden' name='acao' value='reles'>"
         "<label>Rele Inv. Alta (GPIO27):</label>"
         "<select name='inv_inva'>");
    c+=cfgInv.invertReleInvAlta
        ?F("<option value='0'>Normal</option><option value='1' selected>Invertido</option>")
        :F("<option value='0' selected>Normal</option><option value='1'>Invertido</option>");
    c+=F("</select><label>Rele Inv. Baixa (GPIO25):</label><select name='inv_invb'>");
    c+=cfgInv.invertReleInvBaixa
        ?F("<option value='0'>Normal</option><option value='1' selected>Invertido</option>")
        :F("<option value='0' selected>Normal</option><option value='1'>Invertido</option>");
    c+=F("</select><label>Rele Lamp. Ext (GPIO12):</label><select name='inv_lamp'>");
    c+=cfgInv.invertReleLampExt
        ?F("<option value='0'>Normal</option><option value='1' selected>Invertido</option>")
        :F("<option value='0' selected>Normal</option><option value='1'>Invertido</option>");
    c+=F("</select><button type='submit' class='btn-o'>&#128190; Salvar Inversao</button></form>");

    // ── Modo Teste ───────────────────────────────────────────
    c+=F("<h3>&#128268; Teste de Reles</h3>");
    if (modoTeste){
        unsigned long tr=(millis()-tInicioTeste>=TIMEOUT_TESTE_MS)?0:
                         (TIMEOUT_TESTE_MS-(millis()-tInicioTeste))/1000UL;
        char tBuf[60];
        snprintf(tBuf,sizeof(tBuf),"&#9888; MODO TESTE — Timeout em %lus",tr);
        c+="<div class='al'>"+String(tBuf)+"</div>";
        c+=F("<form method='POST' action='/controle'>"
             "<input type='hidden' name='acao' value='sair_teste'>"
             "<button type='submit' class='btn-r'>&#9209; Sair do Teste</button></form>");

        // Botões individuais
        auto btnTeste=[&](const char* ico,const char* nome,
                          const char* acOn,const char* acOff,bool on){
            String cor=on?"#43a047":"#546e7a";
            c+="<div class='cg' style='border-color:"+cor+";margin-top:8px'>";
            c+="<div class='cr'>";
            c+="<span class='cn'>"+String(ico)+" "+String(nome)+"</span>";
            c+="<span class='cs' style='color:"+cor+"'>"+(on?"ON":"OFF")+"</span></div>";
            c+="<div class='grid2' style='gap:6px;margin-top:8px'>";
            c+="<form method='POST' action='/controle' style='margin:0'>"
               "<input type='hidden' name='acao' value='"+String(acOn)+"'>"
               "<button type='submit' class='btn-g btn-sm'>&#9650; LIGAR</button></form>";
            c+="<form method='POST' action='/controle' style='margin:0'>"
               "<input type='hidden' name='acao' value='"+String(acOff)+"'>"
               "<button type='submit' class='btn-r btn-sm'>&#9660; DESLIGAR</button></form>";
            c+="</div></div>";
        };
        btnTeste("&#128268;","Inv. Baixa (GPIO25)",
                 "teste_b_on","teste_b_off",estados.invBaixa);
        btnTeste("&#9889;","Inv. Alta (GPIO27)",
                 "teste_a_on","teste_a_off",estados.invAlta);
        btnTeste("&#128161;","Lampada Ext (GPIO12)",
                 "teste_lamp_on","teste_lamp_off",estados.lampExt);
    } else {
        c+=F("<div class='nf' style='font-size:11px'>"
             "&#9888; Monitor automatico suspenso durante o teste.</div>"
             "<form method='POST' action='/controle'>"
             "<input type='hidden' name='acao' value='entrar_teste'>"
             "<button type='submit' class='btn-o'>&#9654; Ativar Modo Teste (60s)</button></form>");
    }

    // ── JAVASCRIPT ───────────────────────────────────────────
    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function sBc(id,c){var e=document.getElementById(id);if(e)e.style.borderColor=c;}"

    "function atualizarCtrl(d){"

    // Inv Baixa
    "var cB=d.invb?'#4fc3f7':'#546e7a';"
    "sBc('ctrl_card_b',cB);"
    "sT('ctrl_st_b','<span style=\"color:'+cB+'\">'+(d.invb?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "sT('ctrl_sub_b',d.nome_baixa+' | '+d.tempo_b+'s');"

    // Inv Alta
    "var cA=d.inva?'#fdd835':'#546e7a';"
    "sBc('ctrl_card_a',cA);"
    "sT('ctrl_st_a','<span style=\"color:'+cA+'\">'+(d.inva?'&#9679; ON':'&#9675; OFF')+'</span>');"
    "sT('ctrl_sub_a',d.nome_alta+' | Bat>='+d.lim_inv_a.toFixed(0)+'%');"

    // Sinal casa
    "var cCasa=d.casa_pede?'#43a047':d.casa_lig?'#f9a825':'#546e7a';"
    "var tCasa=d.casa_pede?'&#9679; Pede':d.casa_lig?'&#9650; Conf...':'&#9675; Inativo';"
    "sBc('ctrl_card_casa',cCasa);"
    "sT('ctrl_casa','<span style=\"color:'+cCasa+'\">'+tCasa+'</span>');"
    "sT('ctrl_casa_mv',d.casa_mv.toFixed(0)+' mV');"

    // Lamp
    "var cL=d.le?'#fdd835':'#546e7a';"
    "sBc('ctrl_card_lamp',cL);"
    "sT('ctrl_lamp','<span style=\"color:'+cL+'\">'+(d.le?'&#9679; ON':'&#9675; OFF')+'</span>');"

    // Bateria
    "var pct=d.calib?Math.max(0,Math.min(100,d.pct)):0;"
    "var cor=pct>=50?'#2e7d32':pct>=25?'#ef6c00':'#c62828';"
    "var bv=document.getElementById('ctrl_bat_bv');"
    "if(bv){bv.style.width=pct+'%';bv.style.background=cor;bv.textContent=Math.round(pct)+'%';}"
    "sT('ctrl_bat_info',d.calib?d.ah_at.toFixed(1)+'Ah de '+d.ah_disp.toFixed(1)+'Ah':'Nao calibrada');"

    // Info limiares ao vivo
    "sT('ctrl_pct_live',d.calib?d.pct.toFixed(0):'?');"
    "sT('ctrl_lim_a_live',d.lim_inv_a.toFixed(0));"
    "sT('ctrl_lim_l_live',d.lim_lamp.toFixed(0));"
    "}"

    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarCtrl(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Controle",c));
}

// ═══════════════════════════════════════════════════════════
// [SENS-WEB]
// ═══════════════════════════════════════════════════════════
void handleSensores(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        String acao=server.arg("acao");
        if (acao=="calib_painel"){
            float iReal=server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>50) msg="<div class='er'>0-50A</div>";
            else{
                int novoOff=(int)((float)sensores.sinalPainel_mV+iReal*calibPainel.sensMvA);
                calibPainel.offsetMV=novoOff;
                prefs.begin("rrfs_cal",false); prefs.putInt("pn_off",novoOff); prefs.end();
                adicionarLog("Calib painel OK"); msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao=="calib_casa"){
            float iReal=server.arg("corrente_real").toFloat();
            if (iReal<0||iReal>100) msg="<div class='er'>0-100A</div>";
            else{
                int novoOff=(int)((float)sensores.sinalCasa_mV-iReal*calibCasa.sensMvA);
                calibCasa.offsetMV=novoOff;
                prefs.begin("rrfs_cal",false); prefs.putInt("casa_off",novoOff); prefs.end();
                adicionarLog("Calib casa OK"); msg="<div class='ok'>&#10003; Calibrado!</div>";
            }
        } else if (acao=="reset_calib"){
            prefs.begin("rrfs_cal",false);
            prefs.remove("pn_off"); prefs.remove("pn_sens");
            prefs.remove("casa_off"); prefs.remove("casa_sens");
            prefs.end(); carregarCalibSensores();
            adicionarLog("Calib reset"); msg="<div class='ok'>&#10003; Resetado</div>";
        }
    }
    String c=msg;
    c+=F("<h3>&#128270; Leituras ao Vivo</h3>"
         "<div class='sc'><div class='lbl'>&#9728; Painel Solar</div>"
         "<div id='sv_p_corr' class='val'>--</div>"
         "<div id='sv_p_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div id='sv_p_sub' class='sub'>--</div></div>"
         "<div class='sc'><div class='lbl'>&#127968; Casa</div>"
         "<div id='sv_c_corr' class='val'>--</div>"
         "<div id='sv_c_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div id='sv_c_sub' class='sub'>--</div></div>"
         "<div class='sc'><div class='lbl'>&#128267; Bateria (calculado)</div>"
         "<div id='sv_b_corr' class='val'>--</div>"
         "<div id='sv_b_pot' style='font-size:13px;color:#78909c'>--</div>"
         "<div class='sub'>Painel - Casa</div></div>");

    c+=F("<h3>&#9728; Calibrar Painel</h3>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='calib_painel'>"
         "<label>Corrente real (A):</label>"
         "<input type='number' name='corrente_real' step='0.01' min='0' max='50' required>"
         "<button type='submit' class='btn-g'>&#128190; Calibrar Painel</button></form>"
         "<h3>&#127968; Calibrar Casa</h3>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='calib_casa'>"
         "<label>Corrente real (A):</label>"
         "<input type='number' name='corrente_real' step='0.01' min='0' max='100' required>"
         "<button type='submit' class='btn-g'>&#128190; Calibrar Casa</button></form>"
         "<form method='POST' action='/sensores'>"
         "<input type='hidden' name='acao' value='reset_calib'>"
         "<button type='submit' class='btn-r btn-sm' style='margin-top:8px'>&#8635; Reset Calibracao</button></form>");

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function atualizarSens(d){"
    "var cP=d.ip>0.5?'#fdd835':'#455a64';"
    "sT('sv_p_corr','<span style=\"color:'+cP+'\">'+d.ip.toFixed(2)+'A</span>');"
    "sT('sv_p_pot',Math.round(d.wp)+'W');");
    char sensBuf[80];
    snprintf(sensBuf,sizeof(sensBuf),
        "sT('sv_p_sub',d.mv_painel+' mV | Off:%d | S:%.1f');",
        calibPainel.offsetMV, calibPainel.sensMvA);
    c+=sensBuf;
    c+=F("sT('sv_c_corr',d.ic.toFixed(2)+'A');"
         "sT('sv_c_pot',Math.round(d.wc)+'W');");
    snprintf(sensBuf,sizeof(sensBuf),
        "sT('sv_c_sub',d.mv_casa+' mV | Off:%d | S:%.1f');",
        calibCasa.offsetMV, calibCasa.sensMvA);
    c+=sensBuf;
    c+=F("var sB=d.ib>0.3?'+':d.ib<-0.3?'-':'~';"
         "var cB=d.ib>0.3?'#43a047':d.ib<-0.3?'#ef6c00':'#fdd835';"
         "sT('sv_b_corr','<span style=\"color:'+cB+'\">'+sB+Math.abs(d.ib).toFixed(2)+'A</span>');"
         "sT('sv_b_pot',Math.round(Math.abs(d.wb))+'W');}"
         "function aD(){var x=new XMLHttpRequest();"
         "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
         "try{atualizarSens(JSON.parse(x.responseText));}catch(e){}}};"
         "x.open('GET','/dados',true);x.send();}"
         "setInterval(aD,2000);aD();"
         "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Sensores",c));
}

// ═══════════════════════════════════════════════════════════
// [BAT-WEB]
// ═══════════════════════════════════════════════════════════
void handleBateria(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        String acao=server.arg("acao");
        if (acao=="config"){
            float vn=server.arg("vnom").toFloat(),
                  ah=server.arg("ah").toFloat(),
                  dc=server.arg("desc").toFloat();
            if (vn<6||vn>60) msg="<div class='er'>Tensao 6-60V</div>";
            else if (ah<1||ah>5000) msg="<div class='er'>Cap 1-5000Ah</div>";
            else if (dc<10||dc>100) msg="<div class='er'>Desc 10-100%</div>";
            else{
                bateria.tensaoNominal=vn; bateria.capacidadeAh=ah; bateria.descargaSegura=dc;
                calcularBateria(); salvarConfigBateria();
                adicionarLog("Bat config salva"); msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao=="calibrar"){
            float pBMS=server.arg("pct_atual").toFloat();
            if (pBMS<0||pBMS>100) msg="<div class='er'>0-100%</div>";
            else{
                float ahFis=bateria.capacidadeAh*(pBMS/100.0f);
                float ahRes=bateria.capacidadeAh*(1.0f-bateria.descargaSegura/100.0f);
                float ahUt=ahFis-ahRes;
                if (ahUt<0) ahUt=0;
                if (ahUt>bateria.ahDisponivel) ahUt=bateria.ahDisponivel;
                bateria.ahAtual=ahUt;
                bateria.percentualAtual=(bateria.ahDisponivel>0)?(ahUt/bateria.ahDisponivel)*100.0f:0.0f;
                bateria.calibrada=true; ultimoContagem=millis();
                salvarConfigBateria(); adicionarLog("Bat calibrada");
                msg="<div class='ok'>&#10003; Calibrada!</div>";
            }
        }
    }
    String c=msg;
    c+=F("<h3>&#128267; Status ao Vivo</h3>"
         "<div id='bat_warn'></div>"
         "<div class='bb'><div id='bat_bv2' class='bv' style='width:0%'>0%</div></div>"
         "<div id='bat_info' class='nf' style='text-align:center'>Aguardando...</div>"
         "<div class='grid2' style='gap:6px;margin-top:8px'>"
         "<div class='sc' style='text-align:center'><div class='lbl'>Ah Atual</div>"
         "<div id='bat_ah_at' class='val' style='font-size:16px;color:#43a047'>--</div></div>"
         "<div class='sc' style='text-align:center'><div class='lbl'>Ah Disponivel</div>"
         "<div class='val' style='font-size:16px;color:#4fc3f7'>");
    char ahD[12]; snprintf(ahD,sizeof(ahD),"%.1f Ah",bateria.ahDisponivel);
    c+=String(ahD);
    c+=F("</div></div>"
         "<div class='sc' style='text-align:center'><div class='lbl'>Potencia Bat</div>"
         "<div id='bat_pot' class='val' style='font-size:16px'>--</div></div>"
         "<div class='sc' style='text-align:center'><div class='lbl'>Tensao Nominal</div>"
         "<div class='val' style='font-size:16px;color:#90a4ae'>");
    char vN[12]; snprintf(vN,sizeof(vN),"%.1f V",bateria.tensaoNominal);
    c+=String(vN);
    c+=F("</div></div></div>"
         "<div id='bat_tempo2' style='text-align:center;margin:6px 0;font-size:12px'></div>"
         "<h3>&#9881; Configurar</h3>"
         "<form method='POST' action='/bateria'>"
         "<input type='hidden' name='acao' value='config'>"
         "<label>Tensao Nominal (V):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.1f",bateria.tensaoNominal);
    c+="<input type='number' name='vnom' step='0.1' min='6' max='60' required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",bateria.capacidadeAh);
    c+=F("<label>Capacidade (Ah):</label>");
    c+="<input type='number' name='ah' step='1' min='1' max='5000' required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",bateria.descargaSegura);
    c+=F("<label>Descarga Segura (%):</label>");
    c+="<input type='number' name='desc' step='1' min='10' max='100' required value='"+String(tempBuf)+"'>";
    c+=F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>"
         "<h3>&#127919; Calibrar</h3>"
         "<div class='nf'>Informe o % atual do BMS/Display</div>"
         "<form method='POST' action='/bateria'>"
         "<input type='hidden' name='acao' value='calibrar'>"
         "<label>% no BMS:</label>"
         "<input type='number' name='pct_atual' step='1' min='0' max='100' required>"
         "<button type='submit' class='btn-o'>&#127919; Calibrar</button></form>");

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function atualizarBat(d){"
    "var pct=d.calib?Math.max(0,Math.min(100,d.pct)):0;"
    "var cor=pct>=50?'#2e7d32':pct>=25?'#ef6c00':'#c62828';"
    "var bv=document.getElementById('bat_bv2');"
    "if(bv){bv.style.width=pct+'%';bv.style.background=cor;bv.textContent=Math.round(pct)+'%';}"
    "if(d.calib){"
    "  sT('bat_info',d.ah_at.toFixed(1)+'Ah de '+d.ah_disp.toFixed(1)+'Ah ('+d.pct.toFixed(0)+'%)');"
    "  sT('bat_ah_at',d.ah_at.toFixed(1)+' Ah');"
    "  sT('bat_warn','');"
    "}else{"
    "  sT('bat_info','Nao calibrada');"
    "  sT('bat_ah_at','--');"
    "  sT('bat_warn','<div class=\"wn\">Aguardando calibracao</div>');"
    "}"
    "var corP=d.ib>0.3?'#43a047':d.ib<-0.3?'#ef6c00':'#fdd835';"
    "sT('bat_pot','<span style=\"color:'+corP+'\">'+Math.round(Math.abs(d.wb))+'W</span>');"
    "if(d.tempo_val){"
    "  var corT=d.tempo_carg?'#43a047':'#ef6c00';"
    "  sT('bat_tempo2','<span style=\"color:'+corT+'\">'+"
    "    (d.tempo_carg?'&#9650; Completa em: ':'&#9660; Esgota em: ')+"
    "    d.tempo_h+'h'+String(d.tempo_m).padStart(2,'0')+'m</span>');"
    "} else sT('bat_tempo2','');}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{atualizarBat(JSON.parse(x.responseText));}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,2000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Bateria",c));
}

// ═══════════════════════════════════════════════════════════
// [PREV-WEB]
// ═══════════════════════════════════════════════════════════
void handlePrevisao(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        String acao=server.arg("acao");
        if (acao=="config_prev"){
            float lat=server.arg("lat").toFloat(),lon=server.arg("lon").toFloat();
            float pkw=server.arg("painel_kw").toFloat(),ctA=server.arg("ctrl_a").toFloat();
            bool ativ=(server.arg("ativa")=="1");
            if (pkw<=0||pkw>100) msg="<div class='er'>Potencia 0.1-100kW</div>";
            else if (ctA<=0||ctA>500) msg="<div class='er'>Corrente 1-500A</div>";
            else if (lat<-90||lat>90) msg="<div class='er'>Latitude invalida</div>";
            else if (lon<-180||lon>180) msg="<div class='er'>Longitude invalida</div>";
            else{
                cfgPrevisao.latitude=lat; cfgPrevisao.longitude=lon;
                cfgPrevisao.painelKW=pkw; cfgPrevisao.controladorA=ctA;
                cfgPrevisao.ativa=ativ; cfgPrevisao.configurada=true;
                salvarConfigPrevisao();
                if (ativ&&WiFi.status()==WL_CONNECTED){ previsao.valido=false; tUltimaPrevisao=0; }
                adicionarLog("Prev config salva"); msg="<div class='ok'>&#10003; Configurado!</div>";
            }
        } else if (acao=="buscar_agora"){
            previsao.valido=false; tUltimaPrevisao=0; esp_task_wdt_reset();
            msg=buscarPrevisaoOpenMeteo()
                ?"<div class='ok'>&#10003; Atualizado!</div>"
                :"<div class='er'>&#10007; Falha. Verifique WiFi.</div>";
        } else if (acao=="reset_calib_prev"){
            cfgPrevisao.fatorCalib=1.0f; salvarConfigPrevisao();
            if (previsao.valido) calcularAhPrevisto();
            adicionarLog("Calib prev reset"); msg="<div class='ok'>&#10003; Reset OK</div>";
        }
    }
    String c=msg;
    c+=F("<h3>&#9881; Configurar</h3>"
         "<form method='POST' action='/previsao'>"
         "<input type='hidden' name='acao' value='config_prev'>"
         "<label>Latitude:</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.6f",cfgPrevisao.latitude);
    c+="<input type='number' name='lat' step='0.000001' min='-90' max='90' required value='"+String(tempBuf)+"'>";
    c+=F("<label>Longitude:</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.6f",cfgPrevisao.longitude);
    c+="<input type='number' name='lon' step='0.000001' min='-180' max='180' required value='"+String(tempBuf)+"'>";
    c+=F("<label>Potencia Painel (kW):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.2f",cfgPrevisao.painelKW>0?cfgPrevisao.painelKW:1.0f);
    c+="<input type='number' name='painel_kw' step='0.01' min='0.1' max='100' required value='"+String(tempBuf)+"'>";
    c+=F("<label>Corrente Max Controlador (A):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%.0f",cfgPrevisao.controladorA>0?cfgPrevisao.controladorA:30.0f);
    c+="<input type='number' name='ctrl_a' step='1' min='1' max='500' required value='"+String(tempBuf)+"'>";
    c+=F("<label>Ativar:</label><select name='ativa'>");
    c+=cfgPrevisao.ativa
        ?F("<option value='1' selected>Sim</option><option value='0'>Nao</option>")
        :F("<option value='1'>Sim</option><option value='0' selected>Nao</option>");
    c+=F("</select>");
    char bFator[48]; snprintf(bFator,sizeof(bFator),"Fator calibracao: %.3f",cfgPrevisao.fatorCalib);
    c+="<div class='nf' style='font-size:11px'>"+String(bFator)+"</div>";
    c+=F("<button type='submit' class='btn-g'>&#128190; Salvar</button></form>"
         "<h3>&#127774; Status</h3>");

    if (!cfgPrevisao.configurada){ c+=F("<div class='wn'>Nao configurada</div>"); }
    else if (!cfgPrevisao.ativa) { c+=F("<div class='wn'>Desativada</div>"); }
    else if (!previsao.valido){
        c+=F("<div class='wn'>Aguardando dados...</div>"
             "<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='buscar_agora'>"
             "<button type='submit' class='btn-o btn-sm'>&#8635; Buscar Agora</button></form>");
    } else {
        c+=F("<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='buscar_agora'>"
             "<button type='submit' class='btn-o btn-sm'>&#8635; Atualizar</button></form>"
             "<form method='POST' action='/previsao'>"
             "<input type='hidden' name='acao' value='reset_calib_prev'>"
             "<button type='submit' class='btn-s btn-sm' style='margin-top:4px'>&#8635; Reset Calib</button></form>"
             "<h3>&#9200; Previsao Atual</h3>"
             "<div class='grid2'>"
             "<div class='pd'><div class='dt'>Esta hora</div>"
             "<div id='pv_h' class='dv'>--</div></div>"
             "<div class='pd'><div class='dt'>Restante hoje</div>"
             "<div id='pv_r' class='dv'>--</div>"
             "<div class='ds'>Ate meia-noite</div></div>"
             "</div>");

        c+=F("<h3>&#128197; 7 Dias</h3>");
        struct tm tA; bool tT=obterTempoAtual(tA);
        float mx=0;
        for (int x=0;x<PREV_DIAS;x++) if(previsao.ahDia[x]>mx) mx=previsao.ahDia[x];
        for (int d=0;d<PREV_DIAS;d++){
            String nd;
            if(d==0) nd="Hoje"; else if(d==1) nd="Amanha";
            else if(tT){
                struct tm td=tA; td.tm_mday+=d; mktime(&td);
                char db[12]; snprintf(db,sizeof(db),"%02d/%02d",td.tm_mday,td.tm_mon+1); nd=String(db);
            } else nd="Dia+"+String(d);
            char vB[20]; snprintf(vB,sizeof(vB),"%.1f Ah",previsao.ahDia[d]);
            float pt=mx>0?(previsao.ahDia[d]/mx)*100.0f:0;
            c+="<div style='margin:4px 0'>"
               "<div style='display:flex;justify-content:space-between;font-size:11px;margin-bottom:2px'>"
               "<span style='color:#90a4ae'>"+nd+"</span>"
               "<span style='color:#fdd835;font-weight:bold'>"+String(vB)+"</span></div>"
               "<div class='bb'><div class='bv' style='width:"+String((int)pt)+
               "%;background:"+(d==0?"#0277bd":"#1b5e20")+"'></div></div></div>";
        }

        c+=F("<script>"
        "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
        "function aD(){var x=new XMLHttpRequest();"
        "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
        "try{var d=JSON.parse(x.responseText);"
        "sT('pv_h',d.ah_prev_h.toFixed(2)+' Ah');"
        "sT('pv_r',d.ah_prev_r.toFixed(2)+' Ah');"
        "}catch(e){}}};"
        "x.open('GET','/dados',true);x.send();}"
        "setInterval(aD,10000);aD();"
        "</script>");
    }
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Previsao",c));
}

// ═══════════════════════════════════════════════════════════
// [HIST-WEB]
// ═══════════════════════════════════════════════════════════
void handleHistorico(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String c="";
    c+=F("<h3>&#128200; Totais Acumulados</h3><div class='grid2'>");
    auto cT=[&](const char* ico,const char* nm,float ah,const char* cor){
        char buf[20]; snprintf(buf,sizeof(buf),"%.1fAh",ah);
        c+="<div class='sc' style='text-align:center'><div class='lbl'>"+String(ico)+" "+String(nm)+"</div>"
           "<div class='val' style='color:"+String(cor)+"'>"+String(buf)+"</div></div>";
    };
    cT("&#9728;","Painel",contadores.painelAh,"#fdd835");
    cT("&#127968;","Casa",contadores.casaAh,"#4fc3f7");
    cT("&#9650;","Bat+",contadores.batCargAh,"#43a047");
    cT("&#9660;","Bat-",contadores.batDescAh,"#ef6c00");
    c+=F("</div><h3>&#128200; Ultimos 7 Dias</h3>");
    int maxD=histTotalDias<7?histTotalDias:7;
    if (maxD==0){ c+=F("<div class='nf'>Sem historico</div>"); }
    else{
        c+=F("<div style='overflow-x:auto'><table style='width:100%;border-collapse:collapse;font-size:11px'>"
             "<tr style='color:#546e7a'><th style='text-align:left;padding:4px'>Data</th>"
             "<th style='padding:4px'>Painel</th><th style='padding:4px'>Casa</th></tr>");
        for (int i=0;i<maxD;i++){
            RegistroDia dT;
            bool ok=(i==0)?(dT=histDiaAtual,true):carregarDiaFlash(((int)histIndiceAtual-i+HIST_DIAS)%HIST_DIAS,dT);
            if (!ok||!dT.valido) continue;
            char bD[16]; snprintf(bD,sizeof(bD),"%02d/%02d/%04d",dT.dia,dT.mes,dT.ano);
            char buf[10];
            c+="<tr style='border-top:1px solid #222'><td style='padding:4px'>"+String(bD)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalPainelAh); c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td>";
            snprintf(buf,sizeof(buf),"%.1f",dT.totalCasaAh);   c+="<td style='padding:4px;text-align:center'>"+String(buf)+"</td></tr>";
        }
        c+=F("</table></div>");
    }
    c+=F("<h3>&#128200; Tempo Real (60s)</h3>");
    int total=histSnapFull?HIST_SNAP_PONTOS:histSnapIdx;
    if (total<2){ c+=F("<div class='nf'>Aguardando...</div>"); }
    else{
        float maxC=0.5f;
        for (int i=0;i<total;i++){
            int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
            if(histCorrPainel[idx]>maxC)maxC=histCorrPainel[idx];
            if(histCorrCasa[idx]>maxC)maxC=histCorrCasa[idx];
            if(fabsf(histCorrBat[idx])>maxC)maxC=fabsf(histCorrBat[idx]);
        }
        c+="<div style='height:80px;background:#111;border:1px solid #333;border-radius:4px'>"
           "<svg viewBox='0 0 "+String(total)+" 80' preserveAspectRatio='none' style='width:100%;height:100%'>";
        auto py=[&](float v)->float{ return 78.0f-(v/maxC)*76.0f; };
        String pP="M",pC="M";
        for (int i=0;i<total;i++){
            int idx=histSnapFull?(histSnapIdx+i)%HIST_SNAP_PONTOS:i;
            char pt[20];
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrPainel[idx]));
            if(i>0)pP+="L"; pP+=String(pt);
            snprintf(pt,sizeof(pt),"%d,%.1f",i,py(histCorrCasa[idx]));
            if(i>0)pC+="L"; pC+=String(pt);
        }
        c+="<path d='"+pP+"' fill='none' stroke='#fdd835' stroke-width='2'/>"
           "<path d='"+pC+"' fill='none' stroke='#4fc3f7' stroke-width='2'/></svg></div>";
        c+=F("<div style='font-size:10px;color:#546e7a;margin-top:4px'>"
             "<span style='color:#fdd835'>&#9644;</span> Painel "
             "<span style='color:#4fc3f7;margin-left:8px'>&#9644;</span> Casa</div>");
    }
    c+=F("<script>setInterval(function(){location.reload();},30000);</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Historico",c));
}

// ═══════════════════════════════════════════════════════════
// [CFG-WEB]
// ═══════════════════════════════════════════════════════════
void handleConfig(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String msg="";
    if (server.method()==HTTP_POST){
        String acao=server.arg("acao");
        if (acao=="senha"){
            prefs.begin("rrfs",true); String usr=prefs.getString("user",""); prefs.end();
            String pA=server.arg("pass_atual"),pN=server.arg("pass_nova"),pC=server.arg("pass_conf");
            if (!verificarLogin(usr,pA)) msg="<div class='er'>Senha atual incorreta</div>";
            else if (pN.length()<4) msg="<div class='er'>Min 4 chars</div>";
            else if (pN!=pC) msg="<div class='er'>Senhas nao conferem</div>";
            else{ salvarCredenciais(usr,pN); adicionarLog("Senha alterada"); msg="<div class='ok'>&#10003; Alterada!</div>"; }
        } else if (acao=="tempo"){
            int fh=server.arg("fuso_h").toInt(),fm=server.arg("fuso_m").toInt();
            bool ntp=(server.arg("ntp_ativo")=="1");
            if (fh<-12||fh>14) msg="<div class='er'>Fuso -12 a +14</div>";
            else{
                cfgTempo.fusoHoras=fh; cfgTempo.fusoMinutos=fm; cfgTempo.ntpAtivo=ntp;
                if (!ntp){
                    cfgTempo.manualAnno=server.arg("man_ano").toInt();
                    cfgTempo.manualMes =server.arg("man_mes").toInt();
                    cfgTempo.manualDia =server.arg("man_dia").toInt();
                    cfgTempo.manualHora=server.arg("man_hora").toInt();
                    cfgTempo.manualMin =server.arg("man_min").toInt();
                }
                salvarConfigTempo(); adicionarLog("Tempo salvo");
                if (ntp&&WiFi.status()==WL_CONNECTED){ ntpSincronizado=false; sincronizarNTP(); }
                msg="<div class='ok'>&#10003; Salvo!</div>";
            }
        } else if (acao=="reset_historico"){
            if (server.arg("confirmar")!="SIM") msg="<div class='er'>Confirmacao invalida</div>";
            else{
                contadores={0,0,0,0}; salvarContadores();
                memset(&acum,0,sizeof(AcumHora)); acum.horaAtual=255; acum.ultimoAcum=0;
                RegistroDia dV; memset(&dV,0,sizeof(RegistroDia));
                for (uint16_t i=0;i<HIST_DIAS;i++) salvarDiaFlash(i,dV);
                histIndiceAtual=0; histTotalDias=0;
                memset(&histDiaAtual,0,sizeof(RegistroDia)); salvarIndiceHistorico();
                memset(histCorrPainel,0,sizeof(histCorrPainel));
                memset(histCorrCasa,  0,sizeof(histCorrCasa));
                memset(histCorrBat,   0,sizeof(histCorrBat));
                histSnapIdx=0; histSnapFull=false;
                adicionarLog("HIST apagado"); msg="<div class='ok'>&#10003; Zerado!</div>";
            }
        }
    }
    String c=msg;
    c+=F("<h3>&#128202; Sistema</h3>"
         "<div class='sc'>"
         "<div id='cfg_sys' class='val' style='font-size:12px'>Carregando...</div>"
         "<div id='cfg_hora' class='sub'></div>"
         "</div>");

    c+=F("<h3>&#128336; Data e Hora</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='tempo'>"
         "<label>Fuso (h):</label>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.fusoHoras);
    c+="<input type='number' name='fuso_h' min='-12' max='14' step='1' required value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.fusoMinutos);
    c+=F("<label>Minutos fuso:</label>");
    c+="<input type='number' name='fuso_m' min='0' max='59' step='1' value='"+String(tempBuf)+"'>";
    c+=F("<label>NTP:</label>"
         "<select name='ntp_ativo' id='ntpSel' onchange='toggleNTP()'>");
    c+=cfgTempo.ntpAtivo
        ?F("<option value='1' selected>Sim</option><option value='0'>Nao-Manual</option>")
        :F("<option value='1'>Sim</option><option value='0' selected>Nao-Manual</option>");
    c+=F("</select><div id='manualTime' style='display:");
    c+=cfgTempo.ntpAtivo?"none":"block"; c+=F("'>");
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualAnno);
    c+=F("<label>Ano:</label>"); c+="<input type='number' name='man_ano' min='2024' max='2099' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualMes);
    c+=F("<label>Mes:</label>"); c+="<input type='number' name='man_mes' min='1' max='12' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualDia);
    c+=F("<label>Dia:</label>"); c+="<input type='number' name='man_dia' min='1' max='31' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualHora);
    c+=F("<label>Hora:</label>"); c+="<input type='number' name='man_hora' min='0' max='23' value='"+String(tempBuf)+"'>";
    snprintf(tempBuf,sizeof(tempBuf),"%d",cfgTempo.manualMin);
    c+=F("<label>Minuto:</label>"); c+="<input type='number' name='man_min' min='0' max='59' value='"+String(tempBuf)+"'>";
    c+=F("</div><button type='submit' class='btn-g'>&#128190; Salvar Hora</button></form>"
         "<script>function toggleNTP(){"
         "document.getElementById('manualTime').style.display="
         "document.getElementById('ntpSel').value=='1'?'none':'block';}</script>"
         "<h3>&#128274; Senha</h3>"
         "<form method='POST' action='/config'>"
         "<input type='hidden' name='acao' value='senha'>"
         "<label>Atual:</label><input type='password' name='pass_atual' required>"
         "<label>Nova:</label><input type='password' name='pass_nova' required>"
         "<label>Confirmar:</label><input type='password' name='pass_conf' required>"
         "<button type='submit' class='btn-o'>Alterar</button></form>"
         "<h3>&#128465; Limpar Historico</h3>"
         "<form method='POST' action='/config' onsubmit='return confirm(\"Apagar permanentemente?\")'>"
         "<input type='hidden' name='acao' value='reset_historico'>"
         "<input type='hidden' name='confirmar' value='SIM'>"
         "<button type='submit' class='btn-r'>Apagar</button></form>"
         "<h3>&#9881; Acoes</h3>"
         "<form method='GET' action='/logs-salvos'>"
         "<button type='submit' class='btn-s btn-sm'>&#128221; Logs</button></form>"
         "<form method='GET' action='/setup-wifi'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>&#128246; WiFi</button></form>"
         "<form method='GET' action='/ota'>"
         "<button type='submit' class='btn-o btn-sm' style='margin-top:6px'>&#128640; OTA</button></form>"
         "<form method='GET' action='/logout'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>&#128275; Logout</button></form>"
         "<form method='POST' action='/reiniciar'>"
         "<button type='submit' class='btn-r btn-sm' style='margin-top:6px'>&#8635; Reiniciar</button></form>");

    c+=F("<script>"
    "function sT(id,t){var e=document.getElementById(id);if(e)e.innerHTML=t;}"
    "function aD(){var x=new XMLHttpRequest();"
    "x.onreadystatechange=function(){if(x.readyState==4&&x.status==200){"
    "try{var d=JSON.parse(x.responseText);"
    "sT('cfg_sys','v23 | RAM '+d.ram.toFixed(1)+'KB | Up '+d.uptime+'s'+"
    "(d.modo_teste?' | <span style=\"color:#ef6c00\">MODO TESTE</span>':''));"
    "sT('cfg_hora','&#128336; '+d.data_hora+(d.ntp?' <span style=\"color:#43a047\">NTP OK</span>':''));"
    "}catch(e){}}};"
    "x.open('GET','/dados',true);x.send();}"
    "setInterval(aD,5000);aD();"
    "</script>");

    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Config",c));
}

// ═══════════════════════════════════════════════════════════
// [AUX-WEB]
// ═══════════════════════════════════════════════════════════
void handleLogs(){
    if (!logsCarregados) carregarLogsPersistentes();
    String json="["; bool primeiro=true;
    uint16_t inicio=totalLogs>50?totalLogs-50:0;
    for (uint16_t num=inicio;num<totalLogs;num++){
        uint8_t idx=num%MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        if (!primeiro) json+=",";
        primeiro=false; json+="\"";
        for (int ch=0;logs[idx][ch]&&ch<LOG_MAX_LEN;ch++){
            char cc=logs[idx][ch];
            if (cc=='"') json+="\\\"";
            else if (cc=='\\') json+="\\\\";
            else if (cc=='\n') json+=" ";
            else if (cc=='\r') continue;
            else json+=cc;
        }
        json+="\"";
    }
    json+="]";
    server.send(200,"application/json",json);
}

void handleLogsSalvos(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    if (!logsCarregados) carregarLogsPersistentes();
    String c=F("<h3>&#128221; Logs Salvos</h3>");
    c+="<div class='nf'>Total: <b>"+String(totalLogs)+"</b></div>";
    c+=F("<div class='lbx' style='height:400px'>");
    uint16_t inicio=totalLogs>MAX_LOGS?totalLogs-MAX_LOGS:0;
    for (uint16_t num=inicio;num<totalLogs;num++){
        uint8_t idx=num%MAX_LOGS;
        if (logs[idx][0]=='\0') continue;
        c+="<div class='ll'>"+String(logs[idx])+"</div>";
    }
    c+=F("</div><form method='GET' action='/config'>"
         "<button type='submit' class='btn-s btn-sm' style='margin-top:8px'>&#8592; Voltar</button></form>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - Logs",c));
}

void handleReiniciar(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    adicionarLog("Reinicio usuario");
    server.send(200,"text/html;charset=utf-8",
        gerarPagina("Reiniciando",F("<div class='ok'>&#10003; Reiniciando em 3s...</div>"),false));
    delay(3000); ESP.restart();
}

void handleNotFound(){
    String uri=server.uri();
    if (uri.indexOf("generate_204")>=0||uri.indexOf("gen_204")>=0||
        uri.indexOf("hotspot")>=0||uri.indexOf("ncsi.txt")>=0||
        uri.indexOf("connecttest")>=0){
        if (modoAP){ server.sendHeader("Location","http://10.0.0.1/"); server.send(302,"text/plain",""); }
        else server.send(204,"text/plain","");
        return;
    }
    if (uri=="/favicon.ico"){ server.send(204); return; }
    if (modoAP){ server.sendHeader("Location","http://10.0.0.1/"); server.send(302); }
    else server.send(404,"text/plain","404");
}

// ═══════════════════════════════════════════════════════════
// [OTA]
// ═══════════════════════════════════════════════════════════
void handleOTA(){
    if (!logado){server.sendHeader("Location","/login");server.send(302);return;}
    String c=F("<h3>&#128640; Atualizar Firmware</h3>"
               "<div class='nf'>Selecione o arquivo .bin compilado para ESP32</div>"
               "<form method='POST' action='/ota-upload' enctype='multipart/form-data' id='fOTA'>"
               "<input type='file' id='fArq' name='firmware' accept='.bin' required "
               "style='background:#111;border:1px solid #444;border-radius:4px;padding:8px;"
               "color:#e0e0e0;width:100%;box-sizing:border-box'>"
               "<div id='prog' style='display:none;margin-top:8px'>"
               "<div class='bb'><div class='bv' id='pBar' style='width:0%;background:#0277bd'>0%</div></div>"
               "<div style='text-align:center;font-size:12px;color:#78909c;margin-top:4px' id='pTxt'>Enviando...</div></div>"
               "<button type='submit' class='btn-o' style='margin-top:10px' id='btnOTA'>&#128640; Enviar</button></form>"
               "<form method='GET' action='/config'>"
               "<button type='submit' class='btn-s btn-sm' style='margin-top:6px'>&#8592; Voltar</button></form>"
               "<script>"
               "document.getElementById('fOTA').onsubmit=function(e){"
               "e.preventDefault();"
               "var f=document.getElementById('fArq').files[0];"
               "if(!f||!f.name.endsWith('.bin')){alert('Selecione .bin');return;}"
               "var fd=new FormData();fd.append('firmware',f);"
               "var x=new XMLHttpRequest();"
               "document.getElementById('prog').style.display='block';"
               "document.getElementById('btnOTA').disabled=true;"
               "x.upload.onprogress=function(e){"
               "if(e.lengthComputable){"
               "var p=Math.round(e.loaded/e.total*100);"
               "document.getElementById('pBar').style.width=p+'%';"
               "document.getElementById('pBar').textContent=p+'%';"
               "if(p>=100)document.getElementById('pTxt').textContent='Gravando...';}};"
               "x.onreadystatechange=function(){if(x.readyState==4){"
               "if(x.status==200||x.status==0){"
               "document.getElementById('pBar').style.background='#2e7d32';"
               "document.getElementById('pBar').style.width='100%';"
               "document.getElementById('pBar').textContent='100%';"
               "document.getElementById('pTxt').textContent='&#10003; Gravado! Reiniciando...';"
               "document.getElementById('pTxt').style.color='#a5d6a7';"
               "setTimeout(function(){location.href='/';},6000);"
               "}else{"
               "document.getElementById('pTxt').textContent='Erro '+x.status;"
               "document.getElementById('pBar').style.background='#c62828';"
               "document.getElementById('btnOTA').disabled=false;}}};"
               "x.open('POST','/ota-upload',true);x.send(fd);};"
               "</script>");
    server.send(200,"text/html;charset=utf-8",gerarPagina("RRFS - OTA",c));
}

void handleOTAUpload(){
    if (!logado){server.send(403);return;}
    HTTPUpload& up=server.upload();
    if (up.status==UPLOAD_FILE_START){
        adicionarLog("OTA inicio"); esp_task_wdt_reset();
        desligarInvAlta();
        modoTeste=true; tInicioTeste=millis(); TIMEOUT_TESTE_MS=600000UL;
        esp_task_wdt_reset();
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)){
            adicionarLog("OTA ERRO begin"); modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
        }
    } else if (up.status==UPLOAD_FILE_WRITE){
        esp_task_wdt_reset();
        if (Update.write(up.buf,up.currentSize)!=up.currentSize) adicionarLog("OTA ERRO write");
    } else if (up.status==UPLOAD_FILE_END){
        esp_task_wdt_reset();
        if (Update.end(true)){
            adicionarLog("OTA OK");
            if (bateria.calibrada){ prefs.begin("rrfs",false); prefs.putFloat("bat_ah_at",bateria.ahAtual); prefs.end(); }
            server.send(200,"text/plain","OK");
            delay(1500); ESP.restart();
        } else {
            adicionarLog("OTA ERRO end");
            modoTeste=false; TIMEOUT_TESTE_MS=30000UL;
            server.send(500,"text/plain","ERRO");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// [ROTAS]
// ═══════════════════════════════════════════════════════════
void configurarRotas(){
    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/dados",       HTTP_GET,  handleDados);
    server.on("/bateria",     HTTP_ANY,  handleBateria);
    server.on("/controle",    HTTP_ANY,  handleControle);
    server.on("/sensores",    HTTP_ANY,  handleSensores);
    server.on("/config",      HTTP_ANY,  handleConfig);
    server.on("/historico",   HTTP_ANY,  handleHistorico);
    server.on("/previsao",    HTTP_ANY,  handlePrevisao);
    server.on("/logs",        HTTP_GET,  handleLogs);
    server.on("/logs-salvos", HTTP_GET,  handleLogsSalvos);
    server.on("/login",       HTTP_ANY,  handleLogin);
    server.on("/logout",      HTTP_GET,  handleLogout);
    server.on("/setup-login", HTTP_ANY,  handleSetupLogin);
    server.on("/setup-wifi",  HTTP_ANY,  handleSetupWifi);
    server.on("/reiniciar",   HTTP_POST, handleReiniciar);
    server.on("/ota",         HTTP_GET,  handleOTA);
    server.on("/ota-upload",  HTTP_POST, [](){}, handleOTAUpload);
    server.onNotFound(handleNotFound);
    server.begin();
    adicionarLog("Rotas OK v23");
}
