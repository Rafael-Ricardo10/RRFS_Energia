/*
 * RRFS Energia - Sistema Completo de Gerenciamento de Energia Solar
 * Versão 1.0 - Com sensores, controle de relés e interface web
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <time.h>

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

// Estados da máquina de detecção de curto
enum EstadoCurto {
    CURTO_NORMAL = 0,       // Tudo OK
    CURTO_TESTANDO_ALTA,    // Testando se curto é na rede alta
    CURTO_TESTANDO_BAIXA,   // Testando se curto é na rede baixa
    CURTO_CONFIRMADO_ALTA,  // Curto confirmado na rede alta
    CURTO_CONFIRMADO_BAIXA, // Curto confirmado na rede baixa
    CURTO_AGUARDANDO_RESET  // Aguardando usuário resolver e pressionar botão web
};

// Subestados do teste
enum SubEstadoCurto {
    SUB_IDLE = 0,
    SUB_INVERSOR_DESLIGADO,     // Acabou de desligar o inversor
    SUB_AGUARDANDO_QUEDA,       // Aguardando sinal do inversor cair
    SUB_RELIGAR_SEM_ALTA,       // Religar inversor sem rede alta
    SUB_RELIGAR_SEM_BAIXA,      // Religar inversor sem rede baixa
    SUB_AGUARDANDO_STATUS,      // Aguardando status do inversor subir
    SUB_INVERSOR_OK_ALTA_OFF,   // Inversor OK sem rede alta (alta é o curto)
    SUB_INVERSOR_OK_LIGAR_ALTA, // Ligando rede alta para confirmar
    SUB_INVERSOR_OK_ALTA_ON,    // Inversor OK com rede alta (foi pico, normal)
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
    unsigned long  tInicio;         // Tempo do início da etapa
    bool           redeAltaCurto;   // Curto confirmado na rede alta
    bool           redeBaixaCurto;  // Curto confirmado na rede baixa
    int            tentativas;      // Quantas tentativas de reset já fez
    bool           monitorando;     // Está monitorando o inversor ativo?
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
    0, false, false, 0, false
};

// Armazena credenciais WiFi globalmente para reconexão
String gWifiSSID = "";
String gWifiPASS = "";
String gWifiIP   = "";
String gWifiGW   = "";
String gWifiMASK = "";
String gWifiDNS  = "";

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
             "RAM: %u bytes livres", (unsigned)esp_get_free_heap_size());
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
    // Todos os relés iniciam DESLIGADOS (HIGH = desligado na lógica invertida)
    pinMode(PIN_RELE_INV_ALTA,    OUTPUT); digitalWrite(PIN_RELE_INV_ALTA,    HIGH);
    pinMode(PIN_RELE_REDE_BAIXA,  OUTPUT); digitalWrite(PIN_RELE_REDE_BAIXA,  HIGH);
    pinMode(PIN_RELE_REDE_ALTA,   OUTPUT); digitalWrite(PIN_RELE_REDE_ALTA,   HIGH);
    pinMode(PIN_RELE_LAMP_EXTERNA,OUTPUT); digitalWrite(PIN_RELE_LAMP_EXTERNA,HIGH);
    pinMode(PIN_RELE_LAMP_INTERNA,OUTPUT); digitalWrite(PIN_RELE_LAMP_INTERNA,HIGH);

    // Entradas
    pinMode(PIN_STATUS_INV_ALTA, INPUT);
    pinMode(PIN_BKP_BATERIA,     INPUT);

    estados.redeBaixa = false;
    estados.redesAlta = false;
    estados.lampExt   = false;
    estados.lampInt   = false;
    estados.inversor  = false;

    timers.inversorEstadoReal = false;
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

float lerCorrentePainel() {
    int adc    = lerADC(PIN_SENSOR_PAINEL);
    int mv     = adcParaMv(adc);
    sensores.sinalPainel_mV = mv;

    // Sensibilidade 40mV/A com VCC 5V → após divisor: 40 * 0.66 = ~26.4mV/A
    // Mas vamos usar o valor real do divisor
    float sens = PAINEL_SENS_MV_A * 0.66f; // mV/A após divisor
    float corrente = (float)(mv - OFFSET_LIDO_MV) / sens;

    // Filtro: ignore valores muito pequenos (ruído < 0.3A)
    if (corrente > -0.3f && corrente < 0.3f) corrente = 0.0f;

    return corrente;
}

float lerCorrenteCasa() {
    int adc    = lerADC(PIN_SENSOR_CASA);
    int mv     = adcParaMv(adc);
    sensores.sinalCasa_mV = mv;

    // Sensibilidade 20mV/A com VCC 5V → após divisor: 20 * 0.66 = ~13.2mV/A
    float sens = CASA_SENS_MV_A * 0.66f;
    float corrente = (float)(mv - OFFSET_LIDO_MV) / sens;

    // Corrente da casa é sempre positiva (consumo)
    // Filtro ruído
    if (corrente < 0.5f) corrente = 0.0f;

    return corrente;
}

// ============================================================
// LEITURA STATUS INVERSOR COM BUFFER
// ============================================================
bool lerStatusInversor() {
    int adc = lerADC(PIN_STATUS_INV_ALTA);
    int mv  = adcParaMv(adc);
    sensores.sinalInversor_mV = mv;

    bool sinalHigh = (mv >= INV_THRESHOLD_MV);

    if (sinalHigh) {
        // Sinal alto: atualiza timestamp do último HIGH
        timers.inversorUltimoHigh = millis();
        timers.inversorEstadoReal = true;
    } else {
        // Sinal baixo: só confirma desligado após buffer de 2s
        if (timers.inversorEstadoReal &&
            millis() - timers.inversorUltimoHigh >= INV_BUFFER_MS) {
            timers.inversorEstadoReal = false;
        }
    }

    return timers.inversorEstadoReal;
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
    if (sensores.correntePainel < -1.0f) {
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
    if (!bateria.calibrada) return; // Sem calibração, não controla

    float pct = bateria.percentualAtual;
    unsigned long agora = millis();

    // ---- LÂMPADA INTERNA (corte em 0%) ----
    if (pct <= redes.corteLampInt && estados.lampInt) {
        if (!timers.pendenteLampInt) {
            timers.pendenteLampInt  = true;
            timers.inicioLampInt    = agora;
        } else if (agora - timers.inicioLampInt >= RELE_BUFFER_MS) {
            desligarLampInt();
            timers.pendenteLampInt = false;
            adicionarLog("Lamp.Interna desligada: energia baixa");
        }
    } else {
        timers.pendenteLampInt = false;
        // Liga se tiver energia suficiente (margem +2%)
        if (pct > redes.corteLampInt + 1.0f && !estados.lampInt) {
            ligarLampInt();
            adicionarLog("Lamp.Interna ligada: energia recuperada");
        }
    }

    // ---- LÂMPADA EXTERNA (ex: corte em 50%) ----
    if (pct <= redes.corteLampExt && estados.lampExt) {
        if (!timers.pendenteLampExt) {
            timers.pendenteLampExt = true;
            timers.inicioLampExt   = agora;
        } else if (agora - timers.inicioLampExt >= RELE_BUFFER_MS) {
            desligarLampExt();
            timers.pendenteLampExt = false;
            adicionarLog("Lamp.Externa desligada: energia baixa");
        }
    } else {
        timers.pendenteLampExt = false;
        if (pct > redes.corteLampExt + 1.0f && !estados.lampExt) {
            ligarLampExt();
            adicionarLog("Lamp.Externa ligada: energia recuperada");
        }
    }

    // ---- REDE ALTA (ex: corte em 30%) ----
    if (pct <= redes.corteAlta && estados.redesAlta) {
        if (!timers.pendenteAlta) {
            timers.pendenteAlta  = true;
            timers.inicioAlta    = agora;
        } else if (agora - timers.inicioAlta >= RELE_BUFFER_MS) {
            desligarRedeAlta();
            timers.pendenteAlta = false;
            adicionarLog("Rede Alta desligada: energia baixa");
        }
    } else {
        timers.pendenteAlta = false;
        if (pct > redes.corteAlta + 1.0f && !estados.redesAlta) {
            ligarRedeAlta();
            adicionarLog("Rede Alta ligada: energia recuperada");
        }
    }

    // ---- REDE BAIXA + INVERSOR (ex: corte em 10%) ----
    if (pct <= redes.corteBaixa && (estados.redeBaixa || estados.inversor)) {
        if (!timers.pendenteBaixa) {
            timers.pendenteBaixa  = true;
            timers.inicioBaixa    = agora;
        } else if (agora - timers.inicioBaixa >= RELE_BUFFER_MS) {
            desligarRedeBaixa();
            desligarInversor();
            timers.pendenteBaixa = false;
            adicionarLog("Rede Baixa+Inversor desligados: energia critica");
        }
    } else {
        timers.pendenteBaixa = false;
        if (pct > redes.corteBaixa + 1.0f) {
            if (!estados.redeBaixa) {
                ligarRedeBaixa();
                adicionarLog("Rede Baixa ligada: energia recuperada");
            }
            if (!estados.inversor) {
                ligarInversor();
                adicionarLog("Inversor ligado: energia recuperada");
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
// VERIFICAÇÃO DE AUTENTICAÇÃO
// ============================================================
bool verificarAutenticacao() {
    if (!existeCredenciais()) return true;
    return logado;
}

// ============================================================
// FUNÇÃO PRINCIPAL DE MONITORAMENTO DO INVERSOR
// Chame esta função no loop() quando o inversor estiver ligado
// ============================================================
void monitorarInversor() {

    // Só monitora se o inversor está ligado pelo relé
    if (!estados.inversor) {
        curto.monitorando = false;
        return;
    }

    // -----------------------------------------------------------
    // FASE 0: MONITORAMENTO NORMAL
    // Detecta queda inesperada do status com inversor ligado
    // -----------------------------------------------------------
    if (curto.estado == CURTO_NORMAL) {

        bool statusAtual = lerStatusInversor();

        if (!statusAtual && curto.monitorando) {
            // Status caiu! Inversor parou enquanto estava ligado
            // Aguarda 1 segundo para ver se volta sozinho (pico de corrente)
            if (curto.tInicio == 0) {
                curto.tInicio = millis();
                adicionarLog("CURTO: Inversor parou - aguardando 1s...");
            }

            if (millis() - curto.tInicio >= T_MONITOR_INVERSOR) {
                // Ainda sem status após 1s → inicia diagnóstico
                curto.tInicio  = millis();
                curto.estado   = CURTO_TESTANDO_ALTA;
                curto.subEstado = SUB_INVERSOR_DESLIGADO;
                curto.tentativas = 0;
                adicionarLog("CURTO: Iniciando diagnostico...");

                // Passo 1: Desliga o inversor
                desligarInversor();
            }
        } else {
            // Status OK: reseta timer e marca como monitorando
            curto.tInicio    = 0;
            curto.monitorando = true;
        }
        return;
    }

    // -----------------------------------------------------------
    // FASE 1: TESTANDO SE O CURTO É NA REDE ALTA
    // Sequência: desliga inversor → aguarda queda → desliga rede alta
    //            → liga inversor → aguarda status
    //            Se status OK: rede alta é o curto
    //            Se status NOK: testa rede baixa
    // -----------------------------------------------------------
    if (curto.estado == CURTO_TESTANDO_ALTA) {
        unsigned long agora = millis();

        switch (curto.subEstado) {

            case SUB_INVERSOR_DESLIGADO:
                // Inversor acabou de ser desligado, aguarda sinal cair
                if (agora - curto.tInicio >= T_ESPERA_QUEDA_INV) {
                    // Desliga rede alta e mantém rede baixa
                    desligarRedeAlta();
                    delay(200);
                    // Liga inversor sem rede alta
                    ligarInversor();
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_AGUARDANDO_STATUS;
                    adicionarLog("CURTO TESTE: Inversor s/ rede alta...");
                }
                break;

            case SUB_AGUARDANDO_STATUS:
                // Aguarda status do inversor aparecer
                if (lerStatusInversor()) {
                    // Inversor ligou normalmente sem rede alta
                    // → Rede alta provavelmente é o problema
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF;
                    adicionarLog("CURTO TESTE: Inversor OK s/ alta - confirmar...");
                }
                else if (agora - curto.tInicio >= T_ESPERA_STATUS_INV) {
                    // Inversor não ligou mesmo sem rede alta
                    // → Problema não é na rede alta, testa rede baixa
                    desligarInversor();
                    curto.estado    = CURTO_TESTANDO_BAIXA;
                    curto.subEstado = SUB_INVERSOR_DESLIGADO;
                    curto.tInicio   = agora;
                    adicionarLog("CURTO TESTE: Problema nao e alta - testando baixa...");
                }
                break;

            case SUB_INVERSOR_OK_ALTA_OFF:
                // Inversor está OK sem rede alta
                // Aguarda um pouco para confirmar estabilidade
                if (agora - curto.tInicio >= T_ESPERA_STATUS_INV) {
                    if (lerStatusInversor()) {
                        // Confirmado: inversor estável sem rede alta
                        // Agora liga rede alta para confirmar que ela é o problema
                        ligarRedeAlta();
                        curto.tInicio   = agora;
                        curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA;
                        adicionarLog("CURTO TESTE: Ligando rede alta para confirmar...");
                    } else {
                        // Inversor parou de novo sem rede alta → outro problema
                        desligarInversor();
                        curto.estado    = CURTO_TESTANDO_BAIXA;
                        curto.subEstado = SUB_INVERSOR_DESLIGADO;
                        curto.tInicio   = agora;
                        adicionarLog("CURTO TESTE: Instavel s/ alta - testando baixa...");
                    }
                }
                break;

            case SUB_INVERSOR_OK_LIGAR_ALTA:
                // Ligou rede alta com inversor funcionando
                // Aguarda ver se inversor cai (confirma curto na alta)
                if (agora - curto.tInicio >= T_ESPERA_STATUS_INV) {
                    if (!lerStatusInversor()) {
                        // Inversor caiu ao ligar rede alta → CURTO NA REDE ALTA!
                        desligarRedeAlta();
                        desligarInversor();
                        delay(500);
                        // Restaura: liga inversor e rede baixa, desliga rede alta
                        ligarInversor();
                        delay(300);
                        ligarRedeBaixa();

                        curto.estado         = CURTO_CONFIRMADO_ALTA;
                        curto.redeAltaCurto  = true;
                        curto.subEstado      = SUB_IDLE;
                        adicionarLog("!!! CURTO CONFIRMADO NA REDE ALTA !!!");
                        adicionarLog("Rede alta desligada. Acesse /redes para resetar.");
                    } else {
                        // Inversor manteve status com rede alta → ERA APENAS UM PICO!
                        // Tudo normal, restaura estado original
                        curto.estado    = CURTO_NORMAL;
                        curto.subEstado = SUB_IDLE;
                        curto.tInicio   = 0;
                        curto.monitorando = true;
                        adicionarLog("CURTO: Era pico de corrente - sistema normal");
                    }
                }
                break;

            default:
                break;
        }
        return;
    }

    // -----------------------------------------------------------
    // FASE 2: TESTANDO SE O CURTO É NA REDE BAIXA
    // Sequência: desliga inversor → desliga rede baixa (mantém alta desligada)
    //            → liga inversor → aguarda status
    //            Se status OK: rede baixa é o curto
    //            Se status NOK: problema no próprio inversor
    // -----------------------------------------------------------
    if (curto.estado == CURTO_TESTANDO_BAIXA) {
        unsigned long agora = millis();

        switch (curto.subEstado) {

            case SUB_INVERSOR_DESLIGADO:
                // Aguarda queda do sinal do inversor
                if (agora - curto.tInicio >= T_ESPERA_QUEDA_INV) {
                    // Desliga rede baixa
                    desligarRedeBaixa();
                    delay(200);
                    // Liga inversor sem rede baixa E sem rede alta
                    ligarInversor();
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_AGUARDANDO_STATUS;
                    adicionarLog("CURTO TESTE: Inversor s/ rede baixa...");
                }
                break;

            case SUB_AGUARDANDO_STATUS:
                if (lerStatusInversor()) {
                    // Inversor ligou sem rede baixa → rede baixa é o problema
                    // Verifica estabilidade
                    curto.tInicio   = agora;
                    curto.subEstado = SUB_INVERSOR_OK_ALTA_OFF; // Reutiliza estado de verificação
                    adicionarLog("CURTO TESTE: Inversor OK s/ baixa - confirmar...");
                }
                else if (agora - curto.tInicio >= T_ESPERA_STATUS_INV) {
                    // Inversor não liga mesmo sem nada conectado
                    // → Problema no próprio inversor
                    desligarInversor();
                    curto.estado    = CURTO_AGUARDANDO_RESET;
                    curto.subEstado = SUB_IDLE;
                    adicionarLog("!!! FALHA NO INVERSOR - Verifique o equipamento !!!");
                    adicionarLog("Acesse /redes para resetar apos verificar.");
                }
                break;

            case SUB_INVERSOR_OK_ALTA_OFF:
                // Inversor estável sem rede baixa, agora liga rede baixa para confirmar
                if (agora - curto.tInicio >= T_ESPERA_STATUS_INV) {
                    if (lerStatusInversor()) {
                        // Liga rede baixa para confirmar
                        ligarRedeBaixa();
                        curto.tInicio   = agora;
                        curto.subEstado = SUB_INVERSOR_OK_LIGAR_ALTA; // Reutiliza
                        adicionarLog("CURTO TESTE: Ligando rede baixa para confirmar...");
                    } else {
                        // Inversor instável sem rede baixa
                        desligarInversor();
                        curto.estado    = CURTO_AGUARDANDO_RESET;
                        curto.subEstado = SUB_IDLE;
                        adicionarLog("!!! INVERSOR INSTAVEL - Verifique o equipamento !!!");
                    }
                }
                break;

            case SUB_INVERSOR_OK_LIGAR_ALTA:
                // Ligou rede baixa, aguarda ver se inversor cai
                if (agora - curto.tInicio >= T_ESPERA_STATUS_INV) {
                    if (!lerStatusInversor()) {
                        // Inversor caiu ao ligar rede baixa → CURTO NA REDE BAIXA!
                        desligarRedeBaixa();
                        desligarInversor();
                        delay(500);
                        // Mantém inversor desligado e rede baixa desligada
                        // Avisa usuário

                        curto.estado          = CURTO_CONFIRMADO_BAIXA;
                        curto.redeBaixaCurto  = true;
                        curto.subEstado       = SUB_IDLE;
                        adicionarLog("!!! CURTO CONFIRMADO NA REDE BAIXA !!!");
                        adicionarLog("Rede baixa e inversor desligados.");
                        adicionarLog("Acesse /redes para resetar apos corrigir.");
                    } else {
                        // Inversor OK com rede baixa → ERA PICO!
                        curto.estado    = CURTO_NORMAL;
                        curto.subEstado = SUB_IDLE;
                        curto.tInicio   = 0;
                        curto.monitorando = true;
                        adicionarLog("CURTO: Era pico de corrente - sistema normal");
                    }
                }
                break;

            default:
                break;
        }
        return;
    }

    // -----------------------------------------------------------
    // FASE 3: CURTO CONFIRMADO - MANTÉM ESTADO DE PROTEÇÃO
    // Sistema fica em modo de proteção até usuário resolver
    // -----------------------------------------------------------
    if (curto.estado == CURTO_CONFIRMADO_ALTA) {
        // Garante que rede alta permanece desligada
        if (estados.redesAlta) {
            desligarRedeAlta();
        }
        // Inversor e rede baixa continuam funcionando normalmente
        return;
    }

    if (curto.estado == CURTO_CONFIRMADO_BAIXA) {
        // Garante que rede baixa e inversor permanecem desligados
        if (estados.redeBaixa) desligarRedeBaixa();
        if (estados.inversor)  desligarInversor();
        return;
    }

    if (curto.estado == CURTO_AGUARDANDO_RESET) {
        // Garante tudo desligado até reset manual
        if (estados.inversor)  desligarInversor();
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
    if (!existeCredenciais()) { server.sendHeader("Location","/setup-login"); server.send(302); return; }
    if (!existeWiFiConfig())  { server.sendHeader("Location","/setup-wifi");  server.send(302); return; }
    if (!logado)              { server.sendHeader("Location","/login");        server.send(302); return; }

    // Classe bateria
    String batCls = "ok";
    if (bateria.percentualAtual < 25) batCls = "er";
    else if (bateria.percentualAtual < 50) batCls = "wn";

    // Direção bateria
    String dirBat;
    String corBat = "#4fc3f7";
    if (sensores.correnteBateria > 0.3f) {
        dirBat = "&#11014; Carregando";
        corBat = "#43a047";
    } else if (sensores.correnteBateria < -0.3f) {
        dirBat = "&#11015; Descarregando";
        corBat = "#ef5350";
    } else {
        dirBat = "&#11020; Equilibrio";
        corBat = "#fdd835";
    }

    String c = "";

    // Status calibração
    if (!bateria.calibrada) {
        c += "<div class='wn'>&#9888; Bateria nao calibrada. Acesse a pagina Bateria.</div>";
    }

    // Seção energia
    c += "<h3>&#128262; Energia em Tempo Real</h3>";

    // Painel
    String clsPainel = sensores.correntePainel > 0.5f ? "ok" : "sc";
    c += statusCard("&#9728;", "Painel Solar",
                    fmtCorrente(sensores.correntePainel),
                    fmtPotencia(sensores.potenciaPainel),
                    sensores.correntePainel > 0.5f ? "ok" : "");

    // Casa
    c += statusCard("&#127968;", "Consumo da Casa",
                    fmtCorrente(sensores.correnteCasa),
                    fmtPotencia(sensores.potenciaCasa), "");

    // Bateria
    c += "<div class='sc " + batCls + "'>";
    c += "<div class='lbl'>&#128267; Bateria</div>";
    c += "<div class='val' style='color:" + corBat + "'>";
    c += fmtCorrente(fabsf(sensores.correnteBateria)) + " " + dirBat;
    c += "</div>";
    c += "<div class='sub'>" + fmtPotencia(fabsf(sensores.potenciaBateria)) + "</div>";
    c += "</div>";

    // Barra bateria
    c += "<div style='margin:8px 0'>";
    if (bateria.calibrada) {
        char buf[60];
        snprintf(buf, sizeof(buf), "%.0fWh de %.0fWh disponíveis",
                 bateria.energiaAtual, bateria.energiaDisponivel);
        c += barraProgresso(bateria.percentualAtual, String(buf));
    } else {
        c += barraProgresso(0, "Aguardando calibracao");
    }
    c += "</div>";

    // Status relés
    c += "<h3>&#128268; Status das Cargas</h3>";

    auto statusRele = [](const char* nome, bool ativo) -> String {
        String h = "<div style='display:flex;justify-content:space-between;";
        h += "align-items:center;padding:6px 0;border-bottom:1px solid #1a1a1a'>";
        h += "<span style='font-size:13px;color:#90a4ae'>" + String(nome) + "</span>";
        if (ativo)
            h += "<span style='color:#43a047;font-size:12px;font-weight:bold'>&#9679; ON</span>";
        else
            h += "<span style='color:#ef5350;font-size:12px;font-weight:bold'>&#9675; OFF</span>";
        h += "</div>";
        return h;
    };

    c += "<div class='sc'>";
    c += statusRele("&#128268; Rede Baixa",     estados.redeBaixa);
    c += statusRele("&#9889; Rede Alta",         estados.redesAlta);
    c += statusRele("&#128161; Lamp. Externa",   estados.lampExt);
    c += statusRele("&#128161; Lamp. Interna",   estados.lampInt);
    c += statusRele("&#9889; Inversor (Chave)",  estados.inversor);
    c += statusRele("&#9889; Inversor (Status)", sensores.inversoreAtivo);
    c += "</div>";

    // Info rede
    String ip    = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String modo  = modoAP ? "AP" : "WiFi";
    c += "<div class='nf' style='margin-top:10px;font-size:11px'>";
    c += "&#128246; " + modo + " | IP: " + ip;
    if (!modoAP && WiFi.status() == WL_CONNECTED) {
        c += " | RSSI: " + String(WiFi.RSSI()) + "dBm";
    }
    snprintf(tempBuf, sizeof(tempBuf), " | RAM: %uB | Up: %lus",
             (unsigned)esp_get_free_heap_size(), millis()/1000);
    c += String(tempBuf);
    c += "</div>";

    // Auto refresh dados via AJAX
    c += F("<script>"
           "function aD(){"
           "var x=new XMLHttpRequest();"
           "x.onreadystatechange=function(){"
           "if(x.readyState==4&&x.status==200){"
           "try{"
           "var d=JSON.parse(x.responseText);"
           // Aqui poderíamos atualizar valores dinamicamente
           // Por simplicidade, recarrega a página a cada 10s
           "}catch(e){}}}"
           ";x.open('GET','/dados',true);x.send();}"
           "setTimeout(function(){location.reload();},10000);"
           "</script>");

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Painel", c));
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

        // Salvar configurações de corte
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
                adicionarLog("Config redes salva");
                snprintf(tempBuf, sizeof(tempBuf),
                         "Cortes: Bx=%.0f%% Al=%.0f%% LE=%.0f%% LI=%.0f%%",
                         cBaixa, cAlta, cLmpExt, cLmpInt);
                adicionarLog(tempBuf);
                msg = "<div class='ok'>&#10003; Configuracao de redes salva!</div>";
            }
        }

        // Controle manual dos relés
        else if (acao == "rele") {
            String qual  = server.arg("qual");
            String valor = server.arg("valor");
            bool   ligar = (valor == "1");

            // Não permite controle manual se houver curto ativo
            if (curto.estado != CURTO_NORMAL) {
                msg = "<div class='wn'>&#9888; Controle bloqueado durante diagnostico de curto.</div>";
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

    // --------------------------------------------------------
    // MONTA PÁGINA
    // --------------------------------------------------------
    String c = msg;

    // --- Status da bateria ---
    c += "<h3>&#128202; Energia Disponivel</h3>";
    if (bateria.calibrada) {
        c += barraProgresso(bateria.percentualAtual,
                            String(bateria.percentualAtual, 1) +
                            "% | " + String(bateria.energiaAtual, 0) +
                            "Wh de " + String(bateria.energiaDisponivel, 0) + "Wh");
    } else {
        c += "<div class='wn'>&#9888; Bateria nao calibrada - acesse a pagina Bateria</div>";
    }

    // --- Status de curto ---
    c += "<h3>&#9874; Protecao contra Curto</h3>";
    c += htmlStatusCurto();

    // --- Status das cargas ---
    c += "<h3>&#128268; Status das Cargas</h3>";
    c += "<div class='sc'>";

    // Função auxiliar para linha de status com botões ON/OFF
    auto linhaRele = [&](const char* icone, const char* nome,
                         const char* chave, bool ativo,
                         bool bloqueado = false) -> String {

                             String cor = ativo ? "#43a047" : "#ef5350";
                             String est = ativo ? "ON" : "OFF";

                             String h = "<div style='display:flex;justify-content:space-between;"
                             "align-items:center;padding:7px 0;"
                             "border-bottom:1px solid #1a1a1a'>";
                             h += "<span style='font-size:12px;color:#90a4ae'>";
                             h += String(icone) + " " + String(nome) + "</span>";
                             h += "<span style='display:flex;align-items:center;gap:6px'>";
                             h += "<span style='color:" + cor + ";font-size:11px;font-weight:bold'>";
                             h += "&#9679; " + est + "</span>";

                             if (!bloqueado) {
                                 // Botão toggle
                                 String novoValor = ativo ? "0" : "1";
                                 String btnCls    = ativo ? "btn-r" : "btn-g";
                                 String btnTxt    = ativo ? "Desligar" : "Ligar";
                                 h += "<form method='POST' action='/redes' style='margin:0'>";
                                 h += "<input type='hidden' name='acao'  value='rele'>";
                                 h += "<input type='hidden' name='qual'  value='" + String(chave) + "'>";
                                 h += "<input type='hidden' name='valor' value='" + novoValor + "'>";
                                 h += "<button type='submit' class='" + btnCls +
                                 "' style='width:auto;padding:4px 10px;font-size:11px;margin:0'>";
                                 h += btnTxt + "</button></form>";
                             } else {
                                 h += "<span style='color:#546e7a;font-size:10px'>Bloqueado</span>";
                             }

                             h += "</span></div>";
                             return h;
                         };

                         bool bloqueio = (curto.estado != CURTO_NORMAL);

                         c += linhaRele("&#128268;", "Rede Baixa (tomadas normais)",
                                        "rb",  estados.redeBaixa,  false); // Rede baixa nunca bloqueada
                         c += linhaRele("&#9889;",   "Rede Alta (equip. pesados)",
                                        "ra",  estados.redesAlta,
                                        bloqueio || curto.redeAltaCurto);
                         c += linhaRele("&#128161;", "Lamp. Externa",
                                        "le",  estados.lampExt,    false);
                         c += linhaRele("&#128161;", "Lamp. Interna",
                                        "li",  estados.lampInt,    false);
                         c += linhaRele("&#9889;",   "Inversor (Chave Rele)",
                                        "inv", estados.inversor,
                                        bloqueio);
                         // Status do inversor (leitura apenas, sem controle)
                         {
                             String cor = sensores.inversoreAtivo ? "#43a047" : "#546e7a";
                             String est = sensores.inversoreAtivo ? "ATIVO" : "INATIVO";
                             c += "<div style='display:flex;justify-content:space-between;"
                             "align-items:center;padding:7px 0'>";
                             c += "<span style='font-size:12px;color:#90a4ae'>&#9889; Inversor (Status LED)</span>";
                             c += "<span style='color:" + cor + ";font-size:11px;font-weight:bold'>";
                             c += "&#9679; " + est + "</span>";
                             c += "</div>";
                         }
                         c += "</div>";

                         // --- Informação rede baixa ---
                         c += "<div class='nf' style='font-size:11px;margin-top:8px'>";
                         c += "&#8505; <b>Rede Baixa:</b> sempre ligada (roteador WiFi conectado).<br>";
                         c += "O corte automatico da rede baixa so ocorre em energia critica (";
                         c += String(redes.corteBaixa, 0) + "%).";
                         c += "</div>";

                         // --- Configuração de cortes ---
                         c += "<h3>&#9881; Configurar Cortes de Energia</h3>";
                         c += "<div class='nf' style='font-size:11px'>";
                         c += "% baseado na energia disponivel da bateria.<br>";
                         c += "Buffer de 60s antes de desligar. Religamento com margem +2%.";
                         c += "</div>";

                         c += "<form method='POST' action='/redes'>";
                         c += "<input type='hidden' name='acao' value='config'>";

                         // Corte rede baixa
                         c += "<label>&#9889; Rede Baixa + Inversor - Desligar em (%):</label>";
                         c += "<input type='number' name='c_baixa' step='1' min='0' max='100' required ";
                         snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteBaixa);
                         c += String(tempBuf) + " placeholder='Padrao: 10'>";

                         // Corte rede alta
                         c += "<label>&#9889; Rede Alta - Desligar em (%):</label>";
                         c += "<input type='number' name='c_alta' step='1' min='0' max='100' required ";
                         snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteAlta);
                         c += String(tempBuf) + " placeholder='Padrao: 30'>";

                         // Corte lamp externa
                         c += "<label>&#128161; Lamp. Externa - Desligar em (%):</label>";
                         c += "<input type='number' name='c_lmpext' step='1' min='0' max='100' required ";
                         snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteLampExt);
                         c += String(tempBuf) + " placeholder='Padrao: 50'>";

                         // Corte lamp interna
                         c += "<label>&#128161; Lamp. Interna - Desligar em (%):</label>";
                         c += "<input type='number' name='c_lmpint' step='1' min='0' max='100' required ";
                         snprintf(tempBuf, sizeof(tempBuf), "value='%.0f'", redes.corteLampInt);
                         c += String(tempBuf) + " placeholder='Padrao: 0'>";

                         c += "<button type='submit' class='btn-g'>&#128190; Salvar Configuracao</button>";
                         c += "</form>";

                         // --- Tabela visual dos cortes ---
                         c += "<h3>&#128202; Resumo dos Cortes</h3>";
                         c += "<div class='sc' style='font-size:11px'>";

                         auto linhaCorte = [&](const char* nome, float corte,
                                               bool ativo, bool emCurto = false) -> String {
                                                   String corCls = ativo ? "#43a047" : "#ef5350";
                                                   String est    = ativo ? "ON" : "OFF";
                                                   if (emCurto) { corCls = "#ef6c00"; est = "CURTO"; }

                                                   char buf[100];
                                                   snprintf(buf, sizeof(buf),
                                                            "<div style='display:flex;justify-content:space-between;"
                                                            "padding:4px 0;border-bottom:1px solid #1a1a1a'>"
                                                            "<span style='color:#90a4ae'>%s</span>"
                                                            "<span>Corte: <b>%.0f%%</b> &nbsp;"
                                                            "<span style='color:%s'>&#9679; %s</span></span></div>",
                                                            nome, corte, corCls.c_str(), est);
                                                   return String(buf);
                                               };

                                               c += linhaCorte("&#128161; Lamp. Interna",
                                                               redes.corteLampInt, estados.lampInt);
                                               c += linhaCorte("&#9889; Rede Baixa + Inv",
                                                               redes.corteBaixa,  estados.redeBaixa);
                                               c += linhaCorte("&#9889; Rede Alta",
                                                               redes.corteAlta,   estados.redesAlta, curto.redeAltaCurto);
                                               c += linhaCorte("&#128161; Lamp. Externa",
                                                               redes.corteLampExt, estados.lampExt);
                                               c += "</div>";

                                               server.send(200, "text/html; charset=utf-8",
                                                           gerarPagina("RRFS - Redes", c));
}

// ============================================================
// PÁGINA: /sensores
// ============================================================
void handleSensores() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    // Faz leitura fresh para exibir
    char buf[80];

    String c = "";
    c += "<h3>&#128268; Status dos Sensores</h3>";

    // Sensor Painel
    c += "<div class='sc'>";
    c += "<div class='lbl'>&#9728; Sensor Painel Solar (ACS758 50A)</div>";
    snprintf(buf, sizeof(buf), "Sinal: %d mV", sensores.sinalPainel_mV);
    c += "<div class='val'>" + fmtCorrente(sensores.correntePainel) + "</div>";
    c += "<div class='sub'>" + String(buf) + " | " + fmtPotencia(sensores.potenciaPainel) + "</div>";

    if (sensores.correntePainel < -1.0f) {
        c += "<div class='er' style='margin-top:5px;font-size:11px'>";
        c += "&#9888; AVISO: Corrente negativa alta! Verifique cabos do sensor.</div>";
    } else if (sensores.correntePainel < 0) {
        c += "<div class='wn' style='margin-top:5px;font-size:11px'>";
        c += "&#8505; Corrente negativa leve = consumo proprio do controlador (normal &lt; 1A)</div>";
    }
    c += "</div>";

    // Sensor Casa
    c += "<div class='sc'>";
    c += "<div class='lbl'>&#127968; Sensor Casa (ACS758 100A)</div>";
    c += "<div class='val'>" + fmtCorrente(sensores.correnteCasa) + "</div>";
    snprintf(buf, sizeof(buf), "Sinal: %d mV", sensores.sinalCasa_mV);
    c += "<div class='sub'>" + String(buf) + " | " + fmtPotencia(sensores.potenciaCasa) + "</div>";
    c += "</div>";

    // Bateria calculada
    String dirBat = sensores.correnteBateria >= 0 ? "Carregando" : "Descarregando";
    c += "<div class='sc'>";
    c += "<div class='lbl'>&#128267; Bateria (Calculado: Painel - Casa)</div>";
    c += "<div class='val'>" + fmtCorrente(sensores.correnteBateria) + " " + dirBat + "</div>";
    c += "<div class='sub'>" + fmtPotencia(sensores.potenciaBateria) + "</div>";
    c += "</div>";

    // Sensor Inversor
    c += "<div class='sc'>";
    c += "<div class='lbl'>&#9889; Sinal Status Inversor</div>";
    snprintf(buf, sizeof(buf), "%d mV (limiar: %d mV)",
             sensores.sinalInversor_mV, INV_THRESHOLD_MV);
    String invStatus = sensores.inversoreAtivo ? "LIGADO" : "DESLIGADO";
    String invCor    = sensores.inversoreAtivo ? "#43a047" : "#ef5350";
    c += "<div class='val' style='color:" + invCor + "'>" + invStatus + "</div>";
    c += "<div class='sub'>" + String(buf) + "</div>";
    c += "</div>";

    // Sinal BKP (futuro)
    c += "<div class='sc'>";
    c += "<div class='lbl'>&#128267; Sinal BKP Bateria (futuro)</div>";
    int adcBkp = analogRead(PIN_BKP_BATERIA);
    int mvBkp  = adcParaMv(adcBkp);
    snprintf(buf, sizeof(buf), "%d mV (ADC: %d)", mvBkp, adcBkp);
    c += "<div class='val' style='color:#546e7a'>Ignorado</div>";
    c += "<div class='sub'>" + String(buf) + "</div>";
    c += "</div>";

    // Offset e calibração
    c += "<div class='nf' style='font-size:11px'>";
    c += "&#8505; Offset esperado (0A): ~" + String(OFFSET_LIDO_MV) + " mV<br>";
    c += "Painel sens: " + String(PAINEL_SENS_MV_A * 0.66f, 1) + " mV/A | ";
    c += "Casa sens: " + String(CASA_SENS_MV_A * 0.66f, 1) + " mV/A<br>";
    c += "Amostras ADC: " + String(ADC_SAMPLES);
    c += "</div>";

    // Botão refresh
    c += "<button onclick='location.reload()' class='btn-s' style='margin-top:10px'>";
    c += "&#128259; Atualizar Leituras</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Sensores", c));
}

// ============================================================
// PÁGINA: /config (Configurações gerais)
// ============================================================
void handleConfig() {
    if (!logado) { server.sendHeader("Location","/login"); server.send(302); return; }

    String msg = "";

    if (server.method() == HTTP_POST) {
        String acao = server.arg("acao");

        if (acao == "reset_parcial") {
            // Reset parcial: limpa tudo EXCETO rede
            prefs.begin("rrfs", true);
            String wssid = prefs.getString("wifi_ssid","");
            String wpass = prefs.getString("wifi_pass","");
            String wip   = prefs.getString("wifi_ip","");
            String wgw   = prefs.getString("wifi_gw","");
            String wmask = prefs.getString("wifi_mask","255.255.255.0");
            String wdns  = prefs.getString("wifi_dns","8.8.8.8");
            bool   wap   = prefs.getBool  ("usar_ap", true);
            prefs.end();

            prefs.begin("rrfs", false);
            prefs.clear(); // limpa tudo
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

            String cont = "<div class='ok'>&#10003; Reset parcial realizado!</div>";
            cont += "<div class='nf'>Configuracoes de rede mantidas.<br>Reiniciando em 3s...</div>";
            server.send(200, "text/html; charset=utf-8",
                        gerarPagina("RRFS - Reset", cont, false));
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
            user = prefs.getString("user","");
            prefs.end();

            if (!verificarLogin(user, pAtual)) {
                msg = "<div class='er'>&#10007; Senha atual incorreta</div>";
            } else if (pNova.length() < 4) {
                msg = "<div class='er'>&#10007; Nova senha: minimo 4 caracteres</div>";
            } else if (pNova != pConf) {
                msg = "<div class='er'>&#10007; Senhas nao conferem</div>";
            } else {
                salvarCredenciais(user, pNova);
                adicionarLog("Senha alterada");
                msg = "<div class='ok'>&#10003; Senha alterada com sucesso!</div>";
            }
        }
    }

    String ip   = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String modo = modoAP ? "Ponto de Acesso (AP)" : "WiFi Cliente";

    String c = msg;

    // Info sistema
    c += "<h3>&#128202; Informacoes do Sistema</h3>";
    c += "<div class='nf'>";
    c += "&#128246; Modo: " + modo + "<br>";
    c += "&#127760; IP: " + ip + "<br>";
    if (!modoAP && WiFi.status() == WL_CONNECTED) {
        c += "&#128246; SSID: " + WiFi.SSID() + "<br>";
        c += "&#128246; RSSI: " + String(WiFi.RSSI()) + " dBm<br>";
    }
    snprintf(tempBuf, sizeof(tempBuf),
             "&#128190; RAM: %uB | Uptime: %lus",
             (unsigned)esp_get_free_heap_size(), millis()/1000);
    c += String(tempBuf);
    c += "</div>";

    // Configuração rede
    c += "<h3>&#128246; Rede WiFi</h3>";
    c += "<button onclick='location.href=\"/setup-wifi\"' class='btn-o'>";
    c += "&#9881; Reconfigurar Rede WiFi</button>";

    // Alterar senha
    c += "<h3>&#128274; Alterar Senha</h3>";
    c += "<form method='POST' action='/config'>";
    c += "<input type='hidden' name='acao' value='senha'>";
    c += "<label>Senha Atual:</label>";
    c += "<input type='password' name='p_atual' required placeholder='Senha atual'>";
    c += "<label>Nova Senha (min. 4 caracteres):</label>";
    c += "<input type='password' name='p_nova' required placeholder='Nova senha'>";
    c += "<label>Confirmar Nova Senha:</label>";
    c += "<input type='password' name='p_conf' required placeholder='Confirme a nova senha'>";
    c += "<button type='submit'>&#128190; Alterar Senha</button>";
    c += "</form>";

    // Reset parcial
    c += "<h3>&#9888; Reset Parcial</h3>";
    c += "<div class='nf'>Limpa usuarios, bateria e redes. <b>Mantem configuracao WiFi.</b></div>";
    c += "<form method='POST' action='/config' ";
    c += "onsubmit='return confirm(\"Confirma reset parcial? Config de rede sera mantida.\")'>";
    c += "<input type='hidden' name='acao' value='reset_parcial'>";
    c += "<button type='submit' class='btn-r'>&#9888; Reset Parcial (Mantem Rede)</button>";
    c += "</form>";

    // Logout
    c += "<button onclick='location.href=\"/logout\"' class='btn-s' style='margin-top:8px'>";
    c += "&#128275; Sair do Sistema</button>";

    server.send(200, "text/html; charset=utf-8",
                gerarPagina("RRFS - Config", c));
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

    adicionarLog("RRFS Energia v2.0");
    logRAM();

    // Carrega configurações
    carregarConfigBateria();
    carregarConfigRedes();
    calcularBateria();
    ultimoContagem = millis();

    adicionarLog("Config carregada");
    snprintf(tempBuf, sizeof(tempBuf),
             "Bat: %.1fV %.0fAh %.0f%% | Calib:%s",
             bateria.tensaoNominal, bateria.capacidadeAh,
             bateria.descargaSegura, bateria.calibrada ? "SIM" : "NAO");
    adicionarLog(tempBuf);

    // *** REDE BAIXA LIGADA IMEDIATAMENTE ***
    // Roteador está na rede baixa - precisa estar ligado
    // para o ESP32 conseguir conectar ao WiFi
    ligarRedeBaixa();
    adicionarLog("Rede baixa ON (roteador)");

    // Aguarda roteador inicializar (caso ESP32 ligue junto com roteador)
    // Blink LED durante espera
    adicionarLog("Aguardando roteador...");
    unsigned long tRoteador = millis();
    while (millis() - tRoteador < 8000) {
        atualizarLED();
        delay(10);
    }
    adicionarLog("Iniciando conexao...");

    // Inicia rede
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

    // Inicializa monitoramento do inversor
    curto.monitorando = false;
    curto.estado      = CURTO_NORMAL;
    curto.subEstado   = SUB_IDLE;
    curto.tInicio     = 0;
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
    }

    // --------------------------------------------------------
    // MONITORAMENTO DE CURTO - apenas se inversor ligado
    // --------------------------------------------------------
    if (estados.inversor) {
        monitorarInversor();
    } else {
        // Garante que monitoramento reseta quando inversor desliga
        if (curto.monitorando) {
            curto.monitorando = false;
            curto.tInicio     = 0;
        }
    }

    // --------------------------------------------------------
    // CONTROLE DE ENERGIA - a cada 5s
    // Não executa se houver curto total (baixa ou inversor)
    // Em curto de alta: controla tudo exceto rede alta
    // --------------------------------------------------------
    if (agora - ultimoControle >= INTERVALO_CONTROLE_MS) {
        ultimoControle = agora;

        if (curto.estado == CURTO_NORMAL ||
            curto.estado == CURTO_CONFIRMADO_ALTA) {
            controlarEnergia();
            }
            // CURTO_CONFIRMADO_BAIXA e CURTO_AGUARDANDO_RESET:
            // não executa controle automático - sistema em proteção
    }

    // --------------------------------------------------------
    // GERENCIAMENTO WIFI
    // --------------------------------------------------------
    static unsigned long tWifiCheck  = 0;   // Timer verificação status
    static unsigned long tWifiRecon  = 0;   // Timer entre tentativas
    static bool          eraAP_STA   = false;
    static unsigned long tNTPResinc  = 0;   // Timer resincronização NTP diária

    // Verifica status WiFi a cada 15s
    if (!gWifiSSID.isEmpty() && agora - tWifiCheck >= 15000) {
        tWifiCheck = agora;

        if (WiFi.status() != WL_CONNECTED) {

            // Só tenta reconectar a cada 30s para não travar o sistema
            if (agora - tWifiRecon >= 30000) {
                tWifiRecon = agora;
                tentarReconectarWiFi();
            }

        } else {
            // ---- WiFi conectado ----

            // Verifica se estava em modo AP+STA e o WiFi voltou sozinho
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

                // Reseta timer NTP
                tNTPResinc = agora;
            }

            // Detecta transição para AP+STA
            // (ocorre quando tentarReconectarWiFi ativa o AP temporário
            //  mas o WiFi reconecta sozinho pelo autoReconnect)
            if (modoAP && WiFi.getMode() == WIFI_AP_STA) {
                eraAP_STA = true;
            }

            // Resincroniza NTP uma vez por dia (86400000ms = 24h)
            // para corrigir desvios de clock
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
    // Preserva estado mesmo após queda de energia
    // --------------------------------------------------------
    static unsigned long tSalva = 0;
    if (bateria.calibrada && agora - tSalva >= 300000) {
        tSalva = agora;
        prefs.begin("rrfs", false);
        prefs.putFloat("bat_nrg", bateria.energiaAtual);
        prefs.end();
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
