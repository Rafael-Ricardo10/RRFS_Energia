/*
 * RRFS Energia - Sistema de Gerenciamento de Energia
 * VERSÃO CORRIGIDA - Segurança + WiFi Fix
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>

// ============================================================
// DEFINIÇÕES
// ============================================================
#define LED_AZUL       2
#define BOTAO_BOOT     0
#define MAX_LOGS       15
#define LOG_MAX_LEN    100
#define DNS_PORT       53
#define TEMPO_RESET_MS 10000

const char*     AP_SSID = "RRFS Energia";
const IPAddress AP_IP  (10, 0, 0, 1);
const IPAddress AP_GW  (10, 0, 0, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

// ============================================================
// VARIÁVEIS GLOBAIS
// ============================================================
WebServer   server(80);
Preferences prefs;
DNSServer   dnsServer;

bool modoAP = false;
bool logado = false;

unsigned long ultimoBlink = 0;
bool          ledState    = false;
bool          blinkAtivo  = true;

unsigned long botaoPressionadoEm = 0;
bool          botaoSegurando     = false;

char    logs[MAX_LOGS][LOG_MAX_LEN];
uint8_t totalLogs = 0;
uint8_t logIndex  = 0;
char    tempBuf[160];

// ============================================================
// LOGS
// ============================================================
void adicionarLog(const char* msg) {
    unsigned long s = millis() / 1000;
    snprintf(logs[logIndex], LOG_MAX_LEN, "[%02lu:%02lu] %s", s / 60, s % 60, msg);
    logIndex = (logIndex + 1) % MAX_LOGS;
    if (totalLogs < MAX_LOGS) totalLogs++;
}

void logRAM() {
    snprintf(tempBuf, sizeof(tempBuf), "RAM: %u bytes", (unsigned)esp_get_free_heap_size());
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
        botaoSegurando = true;
        botaoPressionadoEm = millis();
    }
    if (!pressionado) botaoSegurando = false;

    if (botaoSegurando && millis() - botaoPressionadoEm >= TEMPO_RESET_MS) {
        adicionarLog("RESET por botao BOOT");
        for (int i = 0; i < 10; i++) {
            digitalWrite(LED_AZUL, HIGH); delay(100);
            digitalWrite(LED_AZUL, LOW);  delay(100);
        }
        prefs.begin("rrfs", false);
        prefs.clear();
        prefs.end();
        delay(500);
        ESP.restart();
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

// ============================================================
// VERIFICAÇÃO DE SEGURANÇA
// ============================================================
bool verificarAutenticacao() {
    // Permite acesso sem login apenas para setup inicial
    if (!existeCredenciais()) return true;
    // Se tem credenciais, precisa estar logado
    return logado;
}

// ============================================================
// FUNÇÃO PARA GERAR HTML COMPLETO
// ============================================================
String gerarPagina(const String& titulo, const String& conteudo) {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>" + titulo + "</title>";
    html += "<style>";
    html += "body{background:#0d0d0d;color:#e0e0e0;font-family:Arial,sans-serif;margin:0;padding:10px;}";
    html += ".card{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:20px;max-width:500px;margin:0 auto;}";
    html += "h2{color:#4fc3f7;text-align:center;margin:0 0 20px 0;}";
    html += "h3{color:#81d4fa;font-size:16px;margin:15px 0 10px 0;border-bottom:1px solid #333;padding-bottom:8px;}";
    html += "label{display:block;color:#90a4ae;font-size:13px;margin-top:12px;margin-bottom:4px;}";
    html += "input,select{width:100%;padding:10px;background:#111;border:1px solid #444;";
    html += "border-radius:4px;color:#e0e0e0;font-size:14px;box-sizing:border-box;}";
    html += "input:focus,select:focus{outline:none;border-color:#0277bd;}";
    html += "button{width:100%;padding:12px;margin-top:15px;background:#0277bd;color:#fff;";
    html += "border:none;border-radius:4px;font-size:15px;font-weight:bold;cursor:pointer;}";
    html += "button:hover{background:#039be5;}";
    html += ".btn-red{background:#c62828;}.btn-red:hover{background:#ef5350;}";
    html += ".btn-green{background:#2e7d32;}.btn-green:hover{background:#43a047;}";
    html += ".btn-orange{background:#ef6c00;}.btn-orange:hover{background:#f57c00;}";
    html += ".msg-ok{background:#1b5e20;color:#a5d6a7;padding:12px;border-radius:4px;margin:10px 0;text-align:center;}";
    html += ".msg-err{background:#b71c1c;color:#ef9a9a;padding:12px;border-radius:4px;margin:10px 0;text-align:center;}";
    html += ".msg-warn{background:#f57f17;color:#fff9c4;padding:12px;border-radius:4px;margin:10px 0;text-align:center;}";
    html += ".info{background:#0d47a1;color:#90caf9;padding:12px;border-radius:4px;margin:10px 0;font-size:13px;line-height:1.6;}";
    html += ".log-titulo{color:#546e7a;font-size:11px;text-transform:uppercase;margin-top:20px;margin-bottom:5px;}";
    html += ".log-box{background:#000;border:1px solid #222;border-radius:4px;padding:10px;";
    html += "height:200px;overflow-y:auto;font-family:monospace;font-size:11px;color:#76ff03;}";
    html += ".log-linha{padding:3px 0;border-bottom:1px solid #111;}";
    html += ".flex{display:flex;gap:10px;margin-top:15px;}.flex button{margin-top:0;}";
    html += ".small-btn{font-size:12px;padding:8px;}";
    html += "</style></head><body><div class='card'>";
    html += "<h2>&#9889; RRFS Energia</h2>";
    html += conteudo;
    html += "<div class='log-titulo'>&#128221; LOGS DO SISTEMA</div>";
    html += "<div class='log-box' id='logbox'><div style='color:#666'>Carregando...</div></div>";
    html += "</div>";
    html += "<script>";
    html += "function atualizarLogs(){";
    html += "var x=new XMLHttpRequest();";
    html += "x.onreadystatechange=function(){";
    html += "if(x.readyState==4&&x.status==200){";
    html += "try{";
    html += "var d=document.getElementById('logbox');";
    html += "var a=JSON.parse(x.responseText);";
    html += "var h='';";
    html += "for(var i=0;i<a.length;i++){";
    html += "h+='<div class=\"log-linha\">'+a[i]+'</div>';}";
    html += "d.innerHTML=h||'<div style=\"color:#666\">Sem logs</div>';";
    html += "}catch(e){}}};";
    html += "x.open('GET','/logs',true);x.send();}";
    html += "setInterval(atualizarLogs,3000);";
    html += "atualizarLogs();";
    html += "</script></body></html>";
    return html;
}

// ============================================================
// PÁGINA: /setup-login
// ============================================================
void handleSetupLogin() {
    // Setup inicial - sempre permitido
    if (existeCredenciais() && existeWiFiConfig()) {
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }
    if (existeCredenciais() && !existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi");
        server.send(302);
        return;
    }

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String user = server.arg("user");
        String pass = server.arg("pass");
        String conf = server.arg("conf");

        if (user.length() < 3) {
            msgHtml = "<div class='msg-err'>&#10007; Usuário precisa ter no mínimo 3 caracteres</div>";
        } else if (pass.length() < 4) {
            msgHtml = "<div class='msg-err'>&#10007; Senha precisa ter no mínimo 4 caracteres</div>";
        } else if (pass != conf) {
            msgHtml = "<div class='msg-err'>&#10007; As senhas não conferem</div>";
        } else {
            salvarCredenciais(user, pass);
            adicionarLog("Credenciais criadas");
            logRAM();
            server.sendHeader("Location", "/setup-wifi");
            server.send(302);
            return;
        }
    }

    String conteudo = "<div class='info'>";
    conteudo += "&#9432; <b>Primeira configuração do sistema.</b><br>";
    conteudo += "Crie suas credenciais de acesso seguro.";
    conteudo += "</div>";
    conteudo += "<h3>&#128274; Criar Conta de Acesso</h3>";
    conteudo += msgHtml;
    conteudo += "<form method='POST' action='/setup-login'>";
    conteudo += "<label>Nome de Usuário (mínimo 3 caracteres):</label>";
    conteudo += "<input type='text' name='user' maxlength='32' required ";
    conteudo += "placeholder='Digite o nome de usuário' autocomplete='off'>";
    conteudo += "<label>Senha (mínimo 4 caracteres):</label>";
    conteudo += "<input type='password' name='pass' maxlength='32' required ";
    conteudo += "placeholder='Digite a senha'>";
    conteudo += "<label>Confirmar Senha:</label>";
    conteudo += "<input type='password' name='conf' maxlength='32' required ";
    conteudo += "placeholder='Digite a senha novamente'>";
    conteudo += "<button type='submit'>Criar e Continuar &#8594;</button>";
    conteudo += "</form>";

    String html = gerarPagina("RRFS Energia - Primeira Configuracao", conteudo);
    server.send(200, "text/html; charset=utf-8", html);
}

// ============================================================
// PÁGINA: /setup-wifi
// ============================================================
void handleSetupWifi() {
    // Se não tem credenciais, precisa criar primeiro
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302);
        return;
    }

    // Se já tem WiFi configurado E não está logado, bloqueia
    if (existeWiFiConfig() && !logado) {
        adicionarLog("Tentativa acesso /setup-wifi sem login");
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String modo = server.arg("modo");
        bool usarAP = (modo == "ap");

        if (usarAP) {
            salvarWifi("", "", "", "", "", "", true);
            adicionarLog("Modo AP salvo pelo usuario");
            logRAM();

            String cont = "<div class='msg-ok'>&#10003; Configuração salva com sucesso!</div>";
            cont += "<div class='info'>Modo: <b>Ponto de Acesso</b><br>";
            cont += "SSID: <b>RRFS Energia</b><br>IP: <b>10.0.0.1</b><br><br>";
            cont += "Reiniciando em 5 segundos...</div>";

            server.send(200, "text/html; charset=utf-8", gerarPagina("RRFS - Configurado", cont));
            delay(5000);
            ESP.restart();
            return;
        }

        String ssid = server.arg("ssid");
        String wpass = server.arg("wpass");
        String ip = server.arg("ip");
        String gw = server.arg("gw");
        String mask = server.arg("mask");
        String dns = server.arg("dns");

        if (ssid.length() == 0) {
            msgHtml = "<div class='msg-err'>&#10007; Selecione uma rede WiFi</div>";
        } else if (ip.length() < 7) {
            msgHtml = "<div class='msg-err'>&#10007; Informe o IP fixo</div>";
        } else if (gw.length() < 7) {
            msgHtml = "<div class='msg-err'>&#10007; Informe o Gateway</div>";
        } else if (mask.length() < 7) {
            msgHtml = "<div class='msg-err'>&#10007; Informe a Máscara</div>";
        } else if (dns.length() < 7) {
            msgHtml = "<div class='msg-err'>&#10007; Informe o DNS</div>";
        } else {
            salvarWifi(ssid, wpass, ip, gw, mask, dns, false);
            snprintf(tempBuf, sizeof(tempBuf), "WiFi salvo: %s", ssid.c_str());
            adicionarLog(tempBuf);
            snprintf(tempBuf, sizeof(tempBuf), "IP: %s GW: %s", ip.c_str(), gw.c_str());
            adicionarLog(tempBuf);
            logRAM();

            String cont = "<div class='msg-ok'>&#10003; Configuração salva!</div>";
            cont += "<div class='info'>Rede: <b>" + ssid + "</b><br>";
            cont += "IP: <b>" + ip + "</b><br>";
            cont += "Gateway: <b>" + gw + "</b><br>";
            cont += "DNS: <b>" + dns + "</b><br><br>";
            cont += "Acesse após reiniciar:<br><b>http://" + ip + "</b><br><br>";
            cont += "Reiniciando em 5 segundos...</div>";

            server.send(200, "text/html; charset=utf-8", gerarPagina("RRFS - Configurado", cont));
            delay(5000);
            ESP.restart();
            return;
        }
    }

    // Escaneia redes
    adicionarLog("Escaneando WiFi...");
    logRAM();
    WiFi.scanDelete(); // Limpa scan anterior
    int n = WiFi.scanNetworks(false, false, false, 600);
    snprintf(tempBuf, sizeof(tempBuf), "Redes WiFi: %d", n);
    adicionarLog(tempBuf);

    String conteudo = "<div class='info'>&#9432; Configure a rede. <b>IP Fixo é obrigatório.</b></div>";

    // Aviso se estiver em modo AP de fallback
    if (modoAP && existeWiFiConfig()) {
        conteudo += "<div class='msg-warn'>&#9888; WiFi configurado não está disponível. ";
        conteudo += "Sistema em modo AP.</div>";
    }

    conteudo += "<h3>&#128246; Configurar Rede</h3>";
    conteudo += msgHtml;
    conteudo += "<form method='POST' action='/setup-wifi'>";
    conteudo += "<label>Modo de Conexão:</label>";
    conteudo += "<select name='modo' id='modo' onchange='toggleModo()'>";
    conteudo += "<option value='wifi'>Conectar ao WiFi</option>";
    conteudo += "<option value='ap'>Manter Ponto de Acesso (AP)</option>";
    conteudo += "</select>";
    conteudo += "<div id='divwifi'>";
    conteudo += "<label>Rede WiFi Disponível:</label>";
    conteudo += "<select name='ssid' id='selssid'>";

    if (n == 0) {
        conteudo += "<option value=''>Nenhuma rede encontrada</option>";
    } else {
        for (int i = 0; i < n; i++) {
            String ssid = WiFi.SSID(i);
            int rssi = WiFi.RSSI(i);
            wifi_auth_mode_t encType = WiFi.encryptionType(i);

            conteudo += "<option value='" + ssid + "'>";
            conteudo += ssid;

            // Sinal WiFi
            if (rssi > -50) conteudo += " [Excelente]";
            else if (rssi > -60) conteudo += " [Bom]";
            else if (rssi > -70) conteudo += " [Regular]";
            else conteudo += " [Fraco]";

            snprintf(tempBuf, sizeof(tempBuf), " (%d dBm)", rssi);
            conteudo += String(tempBuf);

            if (encType == WIFI_AUTH_OPEN) conteudo += " [Aberta]";
            conteudo += "</option>";
        }
    }
    WiFi.scanDelete();

    conteudo += "</select>";

    // Botão para atualizar lista de redes
    conteudo += "<button type='button' class='btn-orange small-btn' ";
    conteudo += "onclick='location.reload()'>&#128259; Atualizar Lista de Redes</button>";

    conteudo += "<label>Senha do WiFi:</label>";
    conteudo += "<input type='password' name='wpass' maxlength='64' ";
    conteudo += "placeholder='Deixe vazio se rede aberta'>";
    conteudo += "<label>IP Fixo do ESP32:</label>";
    conteudo += "<input type='text' name='ip' required placeholder='Ex: 192.168.1.100'>";
    conteudo += "<label>Gateway (IP do Roteador):</label>";
    conteudo += "<input type='text' name='gw' required placeholder='Ex: 192.168.1.1'>";
    conteudo += "<label>Máscara de Sub-rede:</label>";
    conteudo += "<input type='text' name='mask' required value='255.255.255.0'>";
    conteudo += "<label>DNS:</label>";
    conteudo += "<input type='text' name='dns' required value='8.8.8.8'>";
    conteudo += "</div>";
    conteudo += "<button type='submit' class='btn-green'>&#128190; Salvar e Reiniciar</button>";
    conteudo += "</form>";

    // Botão voltar se já logado
    if (logado) {
        conteudo += "<button onclick='location.href=\"/\"' style='background:#546e7a;margin-top:10px'>";
        conteudo += "&#8592; Voltar</button>";
    }

    conteudo += "<script>";
    conteudo += "function toggleModo(){";
    conteudo += "var m=document.getElementById('modo').value;";
    conteudo += "document.getElementById('divwifi').style.display=";
    conteudo += "(m=='wifi'?'block':'none');}";
    conteudo += "</script>";

    server.send(200, "text/html; charset=utf-8", gerarPagina("RRFS - Configurar Rede", conteudo));
}

// ============================================================
// PÁGINA: /login
// ============================================================
void handleLogin() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302);
        return;
    }
    if (logado) {
        server.sendHeader("Location", "/");
        server.send(302);
        return;
    }

    String msgHtml = "";

    if (server.method() == HTTP_POST) {
        String user = server.arg("user");
        String pass = server.arg("pass");

        if (verificarLogin(user, pass)) {
            logado = true;
            snprintf(tempBuf, sizeof(tempBuf), "Login OK: %s", user.c_str());
            adicionarLog(tempBuf);
            logRAM();
            server.sendHeader("Location", "/");
            server.send(302);
            return;
        } else {
            snprintf(tempBuf, sizeof(tempBuf), "Login FALHOU: %s", user.c_str());
            adicionarLog(tempBuf);
            msgHtml = "<div class='msg-err'>&#10007; Usuário ou senha incorretos</div>";
        }
    }

    String conteudo = "<div class='info'>&#128274; Acesso seguro ao sistema</div>";
    conteudo += "<h3>Entrar no Sistema</h3>";
    conteudo += msgHtml;
    conteudo += "<form method='POST' action='/login'>";
    conteudo += "<label>Usuário:</label>";
    conteudo += "<input type='text' name='user' required autocomplete='off' ";
    conteudo += "placeholder='Nome de usuário'>";
    conteudo += "<label>Senha:</label>";
    conteudo += "<input type='password' name='pass' required placeholder='Senha'>";
    conteudo += "<button type='submit'>&#128275; Entrar</button>";
    conteudo += "</form>";

    server.send(200, "text/html; charset=utf-8", gerarPagina("RRFS - Login", conteudo));
}

// ============================================================
// PÁGINA: / (principal) - PROTEGIDA
// ============================================================
void handleRoot() {
    if (!existeCredenciais()) {
        server.sendHeader("Location", "/setup-login");
        server.send(302);
        return;
    }
    if (!existeWiFiConfig()) {
        server.sendHeader("Location", "/setup-wifi");
        server.send(302);
        return;
    }

    // VERIFICAÇÃO DE SEGURANÇA
    if (!logado) {
        adicionarLog("Acesso negado: nao autenticado");
        server.sendHeader("Location", "/login");
        server.send(302);
        return;
    }

    adicionarLog("Pagina principal acessada");
    logRAM();

    String ip = modoAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String modoStr = modoAP ? "Ponto de Acesso (AP)" : "WiFi Cliente";

    String conteudo = "<div class='msg-ok'>&#10003; Sistema operacional e autenticado!</div>";
    conteudo += "<h3>&#128202; Status do Sistema</h3>";
    conteudo += "<div class='info'>";
    conteudo += "&#128246; Modo: <b>" + modoStr + "</b><br>";
    conteudo += "&#127760; IP: <b>" + ip + "</b><br>";

    if (!modoAP && WiFi.status() == WL_CONNECTED) {
        conteudo += "&#128246; SSID: <b>" + WiFi.SSID() + "</b><br>";
        conteudo += "&#128246; RSSI: <b>" + String(WiFi.RSSI()) + " dBm</b><br>";
    }

    snprintf(tempBuf, sizeof(tempBuf), "&#128190; RAM: %u bytes<br>", (unsigned)esp_get_free_heap_size());
    conteudo += String(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf), "&#9201; Uptime: %lu segundos", millis() / 1000);
    conteudo += String(tempBuf);
    conteudo += "</div>";
    conteudo += "<p style='text-align:center;color:#666;margin-top:15px;'>";
    conteudo += "&#9889; Sistema pronto para próximas funcionalidades...</p>";
    conteudo += "<div class='flex'>";
    conteudo += "<button onclick='location.href=\"/setup-wifi\"' class='btn-orange'>&#128246; Rede</button>";
    conteudo += "<button class='btn-red' onclick='location.href=\"/logout\"'>&#128275; Sair</button>";
    conteudo += "</div>";

    server.send(200, "text/html; charset=utf-8", gerarPagina("RRFS Energia - Painel", conteudo));
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
        for (int c = 0; logs[idx][c]; c++) {
            char ch = logs[idx][c];
            if (ch == '"') json += "\\\"";
            else if (ch == '\\') json += "\\\\";
            else json += ch;
        }
        json += "\"";
    }
    json += "]";
    server.send(200, "application/json", json);
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
// ROTAS
// ============================================================
void configurarRotas() {
    server.on("/", handleRoot);
    server.on("/login", handleLogin);
    server.on("/logout", handleLogout);
    server.on("/setup-login", handleSetupLogin);
    server.on("/setup-wifi", handleSetupWifi);
    server.on("/logs", handleLogs);
    server.onNotFound(handleNotFound);
    server.begin();
}

// ============================================================
// REDE
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
    logRAM();
    ledBlink();
}

bool iniciarWiFi(const String& ssid, const String& pass,
                  const String& ip, const String& gw,
                  const String& mask, const String& dns) {

    IPAddress lIP, lGW, lMask, lDNS;

    if (!lIP.fromString(ip)) {
        adicionarLog("ERRO: IP invalido");
        return false;
    }
    if (!lGW.fromString(gw)) {
        adicionarLog("ERRO: Gateway invalido");
        return false;
    }
    if (!lMask.fromString(mask)) {
        adicionarLog("ERRO: Mascara invalida");
        return false;
    }
    if (!lDNS.fromString(dns)) {
        adicionarLog("ERRO: DNS invalido");
        return false;
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    WiFi.mode(WIFI_STA);

    // Desabilita power save para melhor conexão
    WiFi.setSleep(false);

    // Configura IP estático ANTES de conectar
    if (!WiFi.config(lIP, lGW, lMask, lDNS)) {
        adicionarLog("ERRO: Falha config IP estatico");
        return false;
    }

    snprintf(tempBuf, sizeof(tempBuf), "Conectando: %s", ssid.c_str());
    adicionarLog(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf), "IP: %s", ip.c_str());
    adicionarLog(tempBuf);
    snprintf(tempBuf, sizeof(tempBuf), "GW: %s", gw.c_str());
    adicionarLog(tempBuf);

    ledBlink();

    // Conecta ao WiFi
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long t = millis();
    int tentativa = 0;

    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
        atualizarLED();
        delay(500);
        tentativa++;

        if (tentativa % 4 == 0) {
            snprintf(tempBuf, sizeof(tempBuf), "Tentando... %ds", tentativa / 2);
            adicionarLog(tempBuf);
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        snprintf(tempBuf, sizeof(tempBuf),
                 "WiFi OK! IP: %s", WiFi.localIP().toString().c_str());
        adicionarLog(tempBuf);
        snprintf(tempBuf, sizeof(tempBuf), "RSSI: %d dBm", WiFi.RSSI());
        adicionarLog(tempBuf);
        logRAM();
        modoAP = false;
        ledFixo(true);
        return true;
    }

    // Diagnóstico de falha
    switch (WiFi.status()) {
        case WL_NO_SSID_AVAIL:
            adicionarLog("ERRO: SSID nao encontrado");
            break;
        case WL_CONNECT_FAILED:
            adicionarLog("ERRO: Senha incorreta");
            break;
        case WL_CONNECTION_LOST:
            adicionarLog("ERRO: Conexao perdida");
            break;
        case WL_DISCONNECTED:
            adicionarLog("ERRO: WiFi desconectado");
            break;
        default:
            snprintf(tempBuf, sizeof(tempBuf), "ERRO: Status %d", WiFi.status());
            adicionarLog(tempBuf);
    }

    return false;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    setupLED();
    setupBotao();
    ledBlink();

    adicionarLog("RRFS Energia v1.0 SECURE");
    logRAM();

    bool temCred = existeCredenciais();
    bool temWifi = existeWiFiConfig();

    if (!temCred || !temWifi) {
        adicionarLog("Primeira vez - modo setup");
        iniciarAP();
    } else {
        String ssid, pass, ip, gw, mask, dns;
        bool ap;
        carregarWifi(ssid, pass, ip, gw, mask, dns, ap);

        if (ap) {
            adicionarLog("Modo AP configurado");
            iniciarAP();
        } else {
            adicionarLog("Tentando conectar WiFi...");
            if (!iniciarWiFi(ssid, pass, ip, gw, mask, dns)) {
                adicionarLog("Fallback -> Modo AP");
                iniciarAP();
            }
        }
    }

    configurarRotas();
    adicionarLog("Servidor web ATIVO");
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

    // Verifica WiFi a cada 30s
    static unsigned long tWifi = 0;
    if (!modoAP && millis() - tWifi > 30000) {
        tWifi = millis();
        if (WiFi.status() != WL_CONNECTED) {
            adicionarLog("WiFi perdido - reconectando");
            WiFi.reconnect();

            unsigned long t = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
                atualizarLED();
                delay(500);
            }

            if (WiFi.status() == WL_CONNECTED) {
                adicionarLog("WiFi reconectado");
                ledFixo(true);
            } else {
                adicionarLog("Reconexao falhou - AP ativo");
                server.close();
                iniciarAP();
                configurarRotas();
            }
            logRAM();
        }
    }

    // Log RAM a cada 60s
    static unsigned long tRAM = 0;
    if (millis() - tRAM > 60000) {
        tRAM = millis();
        logRAM();
    }

    delay(1);
}
