// =============================================================
// PROJETO AUTOMACAO MARICA - CAIXA CONTROLE
// Arquitetura: ESP-NOW Canal 2 fixo | Wi-Fi sob demanda
// Modos Wi-Fi: ciclo automático (10 min) | OTA | servidor web (BTN1 hold 3s)
// Base de rádio: Gemini (promiscuous → set_channel → esp_now_init)
// CPU: 80 MHz em ESP-NOW | OTA não altera clock (Controle opera 240 MHz fixo)
//
// Alteração 2026-07-18 (marica-088/090/092/093):
//   - loop_sinaleira() apaga todos os LEDs quando não há pacote da Bomba
//     há mais de SINALEIRA_TIMEOUT_MS (5min) -- antes exibia indefinidamente
//     o último nivel_atual/bomba_ligada cacheado como se fosse corrente,
//     mascarando silêncio prolongado da Bomba (incidente de mau contato
//     físico identificado por Peter em campo).
//   - Nova função bomba_esta_offline() centraliza essa checagem (fonte
//     única de verdade), reaproveitada em registrar_telemetria() e
//     loop_supabase() -- novo campo bomba_offline enviado ao Supabase
//     (telemetria_marica), para que o dashboard também saiba distinguir
//     dado confiável de dado cacheado (marica-094/097/098).
//
// Alterações 2026-07-27 (marica-151/152, revisado pela Gemini):
//   - loop_led_erro() deixa de olhar só agua_offline: agora agrega
//     agua_offline + bomba_erro_bitmask (timeout/ladrão/PZEM) + o novo bit
//     ESTADO_RELE_COLADO de bomba_estado_bitmask (marica-149/150, Bomba).
//     LED único de "algo errado" nos dois lados do sistema hidráulico.
//   - rota_config(): catálogo de erro exibe RELE:Colado quando aplicável;
//     ladrão relabelado de "ERRO3:Ladrao" pra "Ladrao:ATIVO" (reclassificação
//     de rótulo só -- continua no mesmo bit, mesmo comportamento de bloqueio
//     na Bomba, continua acendendo o LED); aviso de comando_falhou adicionado
//     (fecha o gap remanescente do marica-131 -- antes só tinha visibilidade
//     via Serial/logs, agora também no servidor web local).
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <marica_protocol.h>

// -------------------------------------------------------------
// SUPABASE
// -------------------------------------------------------------
// SUPABASE_URL e SUPABASE_KEY vem de secrets.h (incluido via marica_protocol.h,
// gitignored -- nunca commitar valores reais). Copie secrets.h.example para
// secrets.h e preencha com o seu projeto. Enquanto SUPABASE_URL contiver
// "SEU_PROJETO" (placeholder), o envio ao Supabase fica automaticamente
// desativado (ver checagens abaixo) -- nao precisa comentar codigo pra testar
// sem Supabase configurado.

// -------------------------------------------------------------
// GPIOs
// -------------------------------------------------------------
#define GPIO_LED_VERDE   27
#define GPIO_LED_AMARELO 26
#define GPIO_LED_VERM_S  25
#define GPIO_LED_AZUL    33
#define GPIO_LED_ERRO    13
#define GPIO_LED_BRANCO  32
#define GPIO_BUZZER      15
#define GPIO_BTN1        14   // clique curto = liga bomba | hold 3s = servidor web
#define GPIO_BTN2        12   // parada de emergência

// -------------------------------------------------------------
// PARÂMETROS TEMPORAIS
// -------------------------------------------------------------
#define BOOT_DELAY_WIFI_MS       60000UL  // 1 min antes do 1º ciclo Wi-Fi
#define CICLO_WIFI_MS           600000UL  // intervalo entre ciclos normal
#define CICLO_WIFI_BOMBA_MS     120000UL  // intervalo durante bombeamento (2 min)
#define JANELA_OTA_MS           600000UL  // janela máxima OTA
#define JANELA_WEBSERVER_MS     300000UL  // janela máxima servidor web
#define BTN1_HOLD_MS              3000UL  // hold para abrir servidor web
#define PING_TIMEOUT_S                 1  // timeout TCP por celular
#define MAX_CELULARES                  4  // slots de IP monitorados
#define MAX_SESSOES                   20  // sessões de bomba em RAM
#define MAX_TELEMETRIA                24  // pontos de telemetria em RAM
#define INTERVALO_TELEMETRIA_MS   300000UL

// -------------------------------------------------------------
// SINALEIRA — detecção de silêncio da Bomba (2026-07-18)
// -------------------------------------------------------------
// Folga confortável acima do intervalo normal de telemetria da Bomba
// (120s com bomba desligada, 10s com bomba ligada -- confirmado em
// main_bomba.cpp/loop_telemetria(), marica-093). Abaixo desse limiar,
// nivel_atual/bomba_ligada são considerados desatualizados demais para
// exibição -- a sinaleira apaga por completo em vez de mostrar estado
// cacheado como se fosse corrente (marica-090/092).
#define SINALEIRA_TIMEOUT_MS      300000UL

// -------------------------------------------------------------
// NVS
// -------------------------------------------------------------
#define NVS_NS_CTRL  "marica_ctrl"
#define NVS_NS_NET   "marica_net"

// -------------------------------------------------------------
// MODOS WI-FI INTERNOS
// -------------------------------------------------------------
typedef enum : uint8_t {
    WIFI_MODO_NENHUM    = 0,
    WIFI_MODO_CICLO     = 1,
    WIFI_MODO_WEBSERVER = 2,
    WIFI_MODO_OTA       = 3
} ModoWifi;

// -------------------------------------------------------------
// MODO OPERACIONAL DA BOMBA
// -------------------------------------------------------------
typedef enum : uint8_t {
    MODO_AUTOMATICO     = 1,
    MODO_SEMIAUTOMATICO = 2
} ModoOperacao;

// -------------------------------------------------------------
// SESSÃO DE BOMBA
// -------------------------------------------------------------
// -------------------------------------------------------------
// ORIGEM DO COMANDO DE LIGAR — 2026-07-25 (dashboard, informativo)
// -------------------------------------------------------------
// Local à Controle, não trafega por ESP-NOW (a Controle já sabe a origem no
// momento em que decide mandar CMD_LIGA_BOMBA -- não precisa perguntar pra
// Bomba). Substitui a ideia inicial de inferir por celular_presente no
// momento da abertura da sessão -- essa segunda abordagem confundiria BTN1
// pressionado com celular na rede como "Automático", quando na verdade foi
// acionamento manual. Rastreando a origem real, essa ambiguidade não existe.
enum OrigemComandoLiga : uint8_t {
    ORIGEM_LIGA_BTN1       = 0,  // botão físico da Caixa Controle
    ORIGEM_LIGA_WEB        = 1,  // botão "Ligar Bomba" da página web
    ORIGEM_LIGA_AUTOMATICO = 2   // loop_automatico_presenca() -- celular detectado na rede
};

struct SessaoBomba {
    uint32_t inicio_ms;
    uint32_t fim_ms;
    uint32_t duracao_s;
    float    nivel_inicio_cm;
    float    nivel_fim_cm;
    float    variacao_cm;
    uint8_t  motivo_desliga;  // 0=nivel, 1=manual, 2=erro
    uint8_t  origem_liga;     // OrigemComandoLiga -- 2026-07-25
    bool     completa;
    bool     ja_enviada;      // true após envio bem-sucedido ao Supabase
    uint32_t pzem_w_medio;    // potência média consolidada no fechamento da sessão
};

static SessaoBomba sessoes[MAX_SESSOES];
static uint8_t     sessao_idx    = 0;
static uint8_t     sessao_count  = 0;
static bool        sessao_aberta = false;
static volatile bool ciclo_wifi_imediato = false;  // dispara ciclo Wi-Fi sem aguardar intervalo

// Acumulador de potência PZEM durante sessão ativa — para cálculo de média
static uint32_t    pzem_acum_w       = 0;
static uint16_t    pzem_acum_amostras = 0;

// -------------------------------------------------------------
// BUFFER DE TELEMETRIA
// -------------------------------------------------------------
struct PontoTelemetria {
    time_t   timestamp_epoch;  // hora real (UTC, via NTP) — não millis(), para reconstruir created_at no Supabase
    float    nivel_cm;
    bool     bomba_ligada;
    uint8_t  erros_ativos;
    uint32_t pzem_w;
    float    pzem_v;
    float    pzem_a;
    float    pzem_pf;
    float    pzem_kwh;
    bool     bomba_offline;  // marica-098: true se ultimo_pacote_bomba estava
                              // além de SINALEIRA_TIMEOUT_MS no momento do registro
    uint8_t  agua_motivo_status;    // 2026-07-25
    uint8_t  bomba_estado_bitmask;  // 2026-07-25
    uint8_t  bomba_causa_desligamento;  // 2026-07-25
    bool     ntp_sincronizado;      // 2026-07-25
    bool     wifi_falhou;           // 2026-07-25
    bool     comando_falhou;        // 2026-07-25
};

static PontoTelemetria buffer_telemetria[MAX_TELEMETRIA];
static uint8_t         telem_idx   = 0;
static uint8_t         telem_count = 0;

// -------------------------------------------------------------
// RING BUFFER DE LOGS INTERNOS
// 50 entradas em RAM — impressas no Serial e acessíveis via /logs
// -------------------------------------------------------------
#define MAX_LOGS 50
static String  log_interno[MAX_LOGS];
static uint8_t log_idx   = 0;
static uint8_t log_count = 0;

// -------------------------------------------------------------
// ESTADO DO SISTEMA (atualizado via PacketStatusCompleto)
// -------------------------------------------------------------
static float        nivel_atual          = 0.0f;
static bool         bomba_ligada         = false;
static bool         agua_erro_sensor     = false;
static bool         agua_offline         = false;  // silêncio de rádio Água→Bomba (marica-123/125)
static bool         agua_ladrao_ativo    = false;  // estado do ladrão repassado pela Bomba
static uint8_t      erros_ativos         = 0;
static uint8_t      agua_motivo_status   = 0;      // MotivoStatusAgua repassado -- 2026-07-25
static uint8_t      bomba_estado_bitmask = 0;      // BitmaskEstadoBomba repassado -- 2026-07-25
static uint8_t      bomba_causa_desligamento = 0;  // CausaDesligamentoBomba repassado -- 2026-07-25
static volatile bool ultimo_comando_falhou       = false;  // 2026-07-25 -- escrita em cb_envio()
                                                             // (task ESP-NOW), lida no loop/envio
                                                             // Supabase -- volatile, mesmo padrão
                                                             // de ultimo_pacote_bomba
static bool         wifi_ultima_tentativa_falhou = false;  // 2026-07-25 -- setado em
                                                             // conectar_wifi_nao_bloqueante()
static uint32_t     pzem_potencia_w      = 0;
static float         pzem_tensao_v        = 0.0f;
static float         pzem_corrente_a      = 0.0f;
static float         pzem_fp              = 0.0f;
static float         pzem_energia_kwh     = 0.0f;
static volatile uint32_t ultimo_pacote_bomba  = 0;  // escrita em cb_recepcao() (task ESP-NOW),
                                                     // lida no loop() principal -- volatile
                                                     // evita cache de registrador (mesmo padrão
                                                     // de ciclo_wifi_imediato/ota_requisitado)
static bool         celular_presente     = false;
static ModoOperacao modo_atual           = MODO_SEMIAUTOMATICO;
static uint8_t      origem_ultimo_comando_liga = ORIGEM_LIGA_BTN1;  // 2026-07-25 -- setada
                                               // nos 3 pontos que mandam CMD_LIGA_BOMBA,
                                               // consumida por abrir_sessao_bomba()
static uint32_t     falhas_supabase      = 0;
static bool         espnow_ok            = false;
static bool         comando_desliga_pendente = false;

// Horímetro
static uint32_t tempo_total_bomba_s = 0;

// Parâmetros operacionais da Bomba — espelho para envio via CMD_SET_NIVEIS
// Valores padrão alinhados com os defaults da Caixa Bomba
static uint8_t  cfg_liga_cm         = 60;
static uint8_t  cfg_desliga_cm      = 40;
static uint8_t  cfg_nivel_seguranca = 85;   // limite superior de segurança (cm)
static uint8_t  cfg_manual_min      = 50;   // nível mínimo p/ liberar partida manual/auto (cm) -- marica-035
static uint16_t cfg_timeout_m       = 60;

// Geometria física da caixa d'água (medidas confirmadas em campo)
// 2026-07-30: CAIXA_CHEIA_CM recalibrado de 40.0f para 30.0f por Peter (medição
// direta em campo) -- valor antigo (40) estava desatualizado em relação à
// geometria real. Único ponto de definição -- calcular_pct() e o novo pacote
// PKT_STATUS_CARDPUTER (Controle→Cardputer) leem daqui, ninguém duplica a constante.
#define CAIXA_CHEIA_CM  30.0f  // distância sensor→água quando reservatório cheio  → 100%
#define CAIXA_VAZIA_CM  70.0f  // distância sensor→água quando reservatório vazio  → 0%

// OTA
static volatile bool ota_requisitado = false;
static bool          ota_ativo       = false;
static uint32_t      inicio_ota      = 0;

// Wi-Fi
static ModoWifi  modo_wifi         = WIFI_MODO_NENHUM;
static uint32_t  inicio_wifi       = 0;
static uint32_t  ultimo_ciclo_wifi = 0;
static bool      wifi_conectado    = false;
static bool      wifi_em_blink     = false;  // LED branco piscante durante Wi-Fi

// Servidor web
static bool          webserver_ativo       = false;
static volatile bool webserver_requisitado = false;  // flag para acionamento seguro fora do callback
WebServer            servidor(80);

// Parâmetros de configuração
// IPs dos celulares monitorados p/ detecção de presença (loop_automatico_presenca).
// Ajuste para os IPs reais dos dispositivos na sua rede (reserva de DHCP recomendada).
static char ip_celulares[MAX_CELULARES][16] = {
    "192.168.1.95",
    "192.168.1.101",
    "",
    ""
};

Preferences prefs;

// -------------------------------------------------------------
// FORWARD DECLARATIONS
// -------------------------------------------------------------
void iniciar_espnow();
bool conectar_wifi_nao_bloqueante();
void desconectar_wifi();
void iniciar_ota();
void encerrar_ota();
void loop_ota();
void abrir_webserver();
void fechar_webserver();
void loop_ciclo_wifi();
void loop_ping_presenca();
void loop_supabase();
void descarregar_buffer();
void enviar_comando(uint8_t tipo_cmd, bool ignorar_nivel = false);
void abrir_sessao_bomba();
void fechar_sessao_bomba(uint8_t motivo);
void registrar_telemetria();
bool bomba_esta_offline();
void carregar_params();
void salvar_params_ctrl();
void salvar_params_net();
void bip(uint8_t n);
void ativar_buzzer_erro();
void loop_buzzer_erro();
void loop_buzzer_nivel();
void loop_led_azul();
void loop_led_branco();
void loop_sinaleira();
void loop_led_erro();
void loop_automatico_presenca();
void loop_botoes();
void cb_envio(const uint8_t* mac_addr, esp_now_send_status_t status);
void cb_recepcao(const uint8_t* mac_addr, const uint8_t* dados, int len);
void rota_raiz();
void rota_config();
void rota_salvar_config();
void rota_cmd();
void rota_logs();
void rota_set_niveis();
void rota_encerrar();
void registrar_log(String msg);
void enviar_config_niveis(uint8_t liga, uint8_t desliga, uint8_t seguranca, uint8_t manual_min, uint16_t timeout_min);
void enviar_status_cardputer();  // 2026-07-30 -- push periódico p/ modo Monitor do Cardputer
bool enviar_sessao_supabase(const SessaoBomba& s, uint32_t pzem_medio);
void enviar_sessoes_pendentes();
// -------------------------------------------------------------
bool enviar_sessao_supabase(const SessaoBomba& s, uint32_t pzem_medio) {
    if (!wifi_conectado) return false;
    if (String(SUPABASE_URL).indexOf("SEU_PROJETO") >= 0) return false;

    struct tm info;
    bool ntp_ok = getLocalTime(&info, 10);

    char ts_inicio[30] = "null";
    char ts_fim[30]    = "null";

    if (ntp_ok) {
        uint32_t agora_ms   = millis();
        int32_t  off_inicio = (int32_t)(agora_ms - s.inicio_ms) / 1000;
        int32_t  off_fim    = (int32_t)(agora_ms - s.fim_ms)    / 1000;

        time_t now_t = mktime(&info);
        time_t t_ini = now_t - off_inicio;
        time_t t_fim = now_t - off_fim;

        struct tm* ti = gmtime(&t_ini);
        strftime(ts_inicio, sizeof(ts_inicio), "\"%Y-%m-%dT%H:%M:%SZ\"", ti);
        struct tm* tf = gmtime(&t_fim);
        strftime(ts_fim, sizeof(ts_fim), "\"%Y-%m-%dT%H:%M:%SZ\"", tf);
    }

    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/sessoes_marica";
    http.begin(url);
    http.addHeader("Content-Type",  "application/json");
    http.addHeader("apikey",        SUPABASE_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
    http.addHeader("Prefer",        "return=minimal");

    String body = "{";
    body += "\"inicio\":"          + String(ts_inicio)             + ",";
    body += "\"fim\":"             + String(ts_fim)                + ",";
    body += "\"duracao_s\":"       + String(s.duracao_s)           + ",";
    body += "\"nivel_inicio_cm\":" + String(s.nivel_inicio_cm, 1)  + ",";
    body += "\"nivel_fim_cm\":"    + String(s.nivel_fim_cm, 1)     + ",";
    body += "\"variacao_cm\":"     + String(s.variacao_cm, 1)      + ",";
    body += "\"motivo_desliga\":"  + String(s.motivo_desliga)      + ",";
    body += "\"origem_liga\":"     + String(s.origem_liga)         + ",";
    body += "\"pzem_w_medio\":"    + String(pzem_medio);
    body += "}";

    int code = http.POST(body);
    http.end();

    if (code > 0 && code < 300) {
        registrar_log("[SUPABASE] Sessao enviada. " + String(s.duracao_s) + "s");
        return true;
    } else {
        registrar_log("[SUPABASE] Falha sessao HTTP " + String(code));
        return false;
    }
}

// -------------------------------------------------------------
// UTILITÁRIO — HORA LOCAL (NTP)
// Retorna HH:MM:SS após sincronização, ou uptime em segundos antes disso
// -------------------------------------------------------------
static String obter_hora() {
    struct tm info;
    if (!getLocalTime(&info, 10)) return String(millis() / 1000) + "s";
    char buf[10];
    strftime(buf, sizeof(buf), "%H:%M:%S", &info);
    return String(buf);
}

// Silencia o buzzer fora do horário permitido (09:00–22:00)
// Sem sincronização NTP, permite bipes para não suprimir alertas vitais no boot
static bool buzzer_permitido() {
    struct tm info;
    if (!getLocalTime(&info, 10)) return true;
    return (info.tm_hour >= 9 && info.tm_hour < 22);
}

// Converte distância do sensor em percentagem de nível do reservatório
// Caixa cheia = CAIXA_CHEIA_CM (100%) | Caixa vazia = CAIXA_VAZIA_CM (0%)
static uint8_t calcular_pct(float distancia_cm) {
    if (distancia_cm <= CAIXA_CHEIA_CM) return 100;
    if (distancia_cm >= CAIXA_VAZIA_CM) return 0;
    float pct = (CAIXA_VAZIA_CM - distancia_cm) / (CAIXA_VAZIA_CM - CAIXA_CHEIA_CM) * 100.0f;
    return (uint8_t)pct;
}
// Verifica se o horário atual permite partidas automáticas da bomba (09:00–18:00)
// Retorna false se NTP não estiver sincronizado — segurança por omissão
static bool horario_bomba_permitido() {
    struct tm info;
    if (!getLocalTime(&info, 10)) return false;
    return (info.tm_hour >= 9 && info.tm_hour < 18);
}

void registrar_log(String msg) {
    String linha = "[" + obter_hora() + "] " + msg;
    Serial.println(linha);
    log_interno[log_idx] = linha;
    log_idx = (log_idx + 1) % MAX_LOGS;
    if (log_count < MAX_LOGS) log_count++;
}

// -------------------------------------------------------------
// NVS — CARGA E SALVAR
// -------------------------------------------------------------
void carregar_params() {
    prefs.begin(NVS_NS_CTRL, true);
    tempo_total_bomba_s = prefs.getUInt("horimetro", 0);
    cfg_liga_cm         = prefs.getUChar("cfg_liga",  60);
    cfg_desliga_cm      = prefs.getUChar("cfg_desl",  40);
    cfg_nivel_seguranca = prefs.getUChar("cfg_seg",   85);
    cfg_manual_min      = prefs.getUChar("cfg_man",   50);
    cfg_timeout_m       = prefs.getUShort("cfg_time", 60);
    prefs.end();

    prefs.begin(NVS_NS_NET, true);
    for (uint8_t i = 0; i < MAX_CELULARES; i++) {
        char chave[10];
        snprintf(chave, sizeof(chave), "ip_cel_%d", i);
        prefs.getString(chave, ip_celulares[i], sizeof(ip_celulares[i]));
    }
    prefs.end();
}

void salvar_params_ctrl() {
    prefs.begin(NVS_NS_CTRL, false);
    prefs.putUInt("horimetro", tempo_total_bomba_s);
    prefs.putUChar("cfg_liga",  cfg_liga_cm);
    prefs.putUChar("cfg_desl",  cfg_desliga_cm);
    prefs.putUChar("cfg_seg",   cfg_nivel_seguranca);
    prefs.putUChar("cfg_man",   cfg_manual_min);
    prefs.putUShort("cfg_time", cfg_timeout_m);
    prefs.end();
}

void salvar_params_net() {
    prefs.begin(NVS_NS_NET, false);
    for (uint8_t i = 0; i < MAX_CELULARES; i++) {
        char chave[10];
        snprintf(chave, sizeof(chave), "ip_cel_%d", i);
        prefs.putString(chave, ip_celulares[i]);
    }
    prefs.end();
}

// -------------------------------------------------------------
// RÁDIO — BASE GEMINI (NÃO MODIFICAR)
// -------------------------------------------------------------
void iniciar_espnow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL_SEGURANCA_PADRAO, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[CONTROLE] Erro ESP-NOW."));
        espnow_ok = false;
        return;
    }

    esp_now_register_send_cb(cb_envio);
    esp_now_register_recv_cb(cb_recepcao);

    esp_now_peer_info_t p = {};
    p.channel = 0;
    p.encrypt = false;

    esp_now_del_peer(MAC_BOMBA);
    memcpy(p.peer_addr, MAC_BOMBA, 6);
    esp_now_add_peer(&p);

    esp_now_del_peer(MAC_CARDPUTER);
    memcpy(p.peer_addr, MAC_CARDPUTER, 6);
    esp_now_add_peer(&p);

    espnow_ok = true;
    registrar_log("[CONTROLE] ESP-NOW pronto. Canal: " + String(CANAL_SEGURANCA_PADRAO));
}

// -------------------------------------------------------------
// WI-FI — CONEXÃO NÃO BLOQUEANTE COM ABORT POR BOTÃO
// (Regra 4 do relatório de arquitetura)
// -------------------------------------------------------------
bool conectar_wifi_nao_bloqueante() {
    esp_now_deinit();
    espnow_ok = false;

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(200);

    WiFi.mode(WIFI_STA);
    WiFi.config(IP_CONTROLE, IP_GATEWAY, IP_MASCARA, IP_DNS);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print(F("[CONTROLE] Wi-Fi conectando"));

    uint32_t t_inicio = millis();

    // Trava de armar: BTN1 só pode abortar após ser solto pelo menos uma vez.
    // Evita que o hold de 3s que disparou o servidor web cancele a conexão
    // imediatamente — o dedo ainda está físicamente no botão ao entrar aqui.
    bool btn1_armado = (digitalRead(GPIO_BTN1) == HIGH);

    while (WiFi.status() != WL_CONNECTED) {
        esp_task_wdt_reset();  // evita WDT timeout durante conexão (~20s)

        // Timeout 20s
        if (millis() - t_inicio > 20000UL) {
            Serial.println(F("\n[CONTROLE] Wi-Fi timeout."));
            wifi_conectado = false;
            wifi_ultima_tentativa_falhou = true;  // 2026-07-25 -- só timeout real conta como
                                                   // falha; abort por botão é ação intencional
            return false;
        }

        // Arma o BTN1 assim que o dedo for retirado (transição LOW → HIGH)
        if (!btn1_armado && digitalRead(GPIO_BTN1) == HIGH) {
            btn1_armado = true;
        }

        // Abort por BTN1 (liga bomba) — só atua após armado
        if (btn1_armado && digitalRead(GPIO_BTN1) == LOW) {
            Serial.println(F("\n[CONTROLE] BTN1 pressionado — abortando Wi-Fi."));
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_OFF);
            delay(200);
            iniciar_espnow();
            origem_ultimo_comando_liga = ORIGEM_LIGA_BTN1;  // 2026-07-25
            enviar_comando(CMD_LIGA_BOMBA);
            bip(1);
            wifi_conectado = false;
            return false;
        }

        // Abort por BTN2 (para bomba) — emergência, sem debounce
        if (digitalRead(GPIO_BTN2) == LOW) {
            Serial.println(F("\n[CONTROLE] BTN2 pressionado — abortando Wi-Fi."));
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_OFF);
            delay(200);
            iniciar_espnow();
            comando_desliga_pendente = true;
            enviar_comando(CMD_DESLIGA_BOMBA);
            bip(1);
            wifi_conectado = false;
            return false;
        }

        delay(50);
        Serial.print('.');
    }

    Serial.println();
    wifi_conectado = true;
    wifi_em_blink  = true;
    wifi_ultima_tentativa_falhou = false;  // 2026-07-25 -- sucesso limpa a flag

    // Sincroniza relógio via NTP — fuso horário Brasília (UTC-3)
    configTime(-10800, 0, "pool.ntp.org", "time.nist.gov");

    registrar_log("[WIFI] OK. IP: " + WiFi.localIP().toString() + " Canal:" + String(WiFi.channel()));
    return true;
}

void desconectar_wifi() {
    wifi_em_blink = false;
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    wifi_conectado = false;
    Serial.println(F("[CONTROLE] Wi-Fi encerrado. Retomando ESP-NOW."));
    iniciar_espnow();
}

// -------------------------------------------------------------
// OTA
// -------------------------------------------------------------
void iniciar_ota() {
    if (ota_ativo) return;
    ota_ativo       = true;
    ota_requisitado = false;
    inicio_ota      = millis();
    modo_wifi       = WIFI_MODO_OTA;

    Serial.println(F("[OTA] Iniciando..."));

    if (!conectar_wifi_nao_bloqueante()) {
        ota_ativo = false;
        modo_wifi = WIFI_MODO_NENHUM;
        return;
    }

    ArduinoOTA.setHostname("caixa_controle");
    ArduinoOTA.onStart([]() {
        Serial.println(F("[OTA] Gravacao iniciada."));
        esp_task_wdt_reset();
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        esp_task_wdt_reset();
        static uint8_t ultimo_pct = 255;
        uint8_t pct = (progress * 100) / total;
        if (pct != ultimo_pct && pct % 10 == 0) {
            Serial.printf("[OTA] %d%%\n", pct);
            ultimo_pct = pct;
        }
    });
    ArduinoOTA.onEnd([]() {
        Serial.println(F("[OTA] Concluido. Reiniciando..."));
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("[OTA] Erro [%u]\n", e);
    });
    ArduinoOTA.begin();
    Serial.println(F("[OTA] Aguardando upload (10 min)..."));
}

void encerrar_ota() {
    ArduinoOTA.end();
    ota_ativo       = false;
    ota_requisitado = false;
    modo_wifi       = WIFI_MODO_NENHUM;
    desconectar_wifi();
}

void loop_ota() {
    if (!ota_ativo) return;
    ArduinoOTA.handle();
    esp_task_wdt_reset();
    if (millis() - inicio_ota >= JANELA_OTA_MS) {
        Serial.println(F("[OTA] Timeout. Encerrando."));
        encerrar_ota();
    }
}

// -------------------------------------------------------------
// CICLO WI-FI AUTOMÁTICO
// Primeira janela: 5 min após boot. Subsequentes: 10 min.
// Duração dinâmica — encerra ao concluir as tarefas.
// -------------------------------------------------------------
void loop_ciclo_wifi() {
    if (modo_wifi != WIFI_MODO_NENHUM) return;

    // Intervalo: boot delay no primeiro ciclo, reduzido durante bombeamento, normal no resto
    uint32_t intervalo;
    if (ultimo_ciclo_wifi == 0)  intervalo = BOOT_DELAY_WIFI_MS;
    else if (bomba_ligada)       intervalo = CICLO_WIFI_BOMBA_MS;
    else                         intervalo = CICLO_WIFI_MS;

    // Ciclo imediato: disparado quando bomba liga ou desliga
    if (!ciclo_wifi_imediato && millis() - ultimo_ciclo_wifi < intervalo) return;
    ciclo_wifi_imediato = false;

    registrar_log(bomba_ligada ? "[CICLO] Wi-Fi — bomba ativa." : "[CICLO] Iniciando janela Wi-Fi.");
    ultimo_ciclo_wifi = millis();
    modo_wifi         = WIFI_MODO_CICLO;

    if (!conectar_wifi_nao_bloqueante()) {
        modo_wifi = WIFI_MODO_NENHUM;
        return;
    }

    // Tarefas em sequência — janela dinâmica, encerra ao terminar
    loop_ping_presenca();
    loop_supabase();
    descarregar_buffer();
    enviar_sessoes_pendentes();

    // Encerra Wi-Fi imediatamente após as tarefas
    modo_wifi = WIFI_MODO_NENHUM;
    desconectar_wifi();

    (void)PacketComandoBomba{};  // evita warning — CMD_MODO removido do protocolo v2

    registrar_log("[CICLO] Concluido. Celular: " + String(celular_presente ? "PRESENTE" : "AUSENTE"));
}

// -------------------------------------------------------------
// PING DE PRESENÇA — TCP Socket (Regra 3)
// -------------------------------------------------------------
static bool ping_tcp(const char* ip) {
    if (strlen(ip) == 0) return false;

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(80);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return false;

    // Socket não-bloqueante — nunca trava o Watchdog
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);

    int ret = connect(sock, (struct sockaddr*)&addr, sizeof(addr));

    if (ret == 0) {
        // Porta 80 aberta — presença confirmada
        close(sock);
        return true;
    }

    if (errno == EINPROGRESS) {
        // Handshake em andamento — aguarda até PING_TIMEOUT_S via select()
        fd_set fdset;
        FD_ZERO(&fdset);
        FD_SET(sock, &fdset);
        struct timeval tv = {PING_TIMEOUT_S, 0};

        ret = select(sock + 1, NULL, &fdset, NULL, &tv);

        if (ret == 1) {
            int so_error = 0;
            socklen_t len = sizeof(so_error);
            getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
            close(sock);
            // so_error == 0           → porta aberta
            // ECONNREFUSED (111)      → celular rejeitou ativamente → PRESENTE
            // ECONNRESET   (104)      → celular enviou TCP RST      → PRESENTE
            return (so_error == 0 || so_error == ECONNREFUSED || so_error == ECONNRESET);
        }

        // ret == 0 → timeout → celular ausente ou fora de alcance
        close(sock);
        return false;
    }

    // Falha imediata de rede local
    int err = errno;
    close(sock);
    return (err == ECONNREFUSED || err == ECONNRESET);
}

void loop_ping_presenca() {
    if (!wifi_conectado) return;
    bool presente = false;
    for (uint8_t i = 0; i < MAX_CELULARES; i++) {
        if (strlen(ip_celulares[i]) == 0) continue;
        bool ok = ping_tcp(ip_celulares[i]);
        Serial.printf("[PING] cel%d (%s): %s\n", i, ip_celulares[i], ok ? "OK" : "--");
        if (ok) presente = true;
    }

    if (presente != celular_presente) {
        celular_presente = presente;
        modo_atual       = presente ? MODO_AUTOMATICO : MODO_SEMIAUTOMATICO;
        registrar_log("[SYS] Modo alternado para: " + String(presente ? "AUTOMATICO" : "SEMIAUTOMATICO"));
    }
}

// -------------------------------------------------------------
// SUPABASE
// -------------------------------------------------------------
void loop_supabase() {
    if (!wifi_conectado) return;
    if (String(SUPABASE_URL).indexOf("SEU_PROJETO") >= 0) return;

    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/telemetria_marica";
    http.begin(url);
    http.addHeader("Content-Type",  "application/json");
    http.addHeader("apikey",        SUPABASE_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
    http.addHeader("Prefer",        "return=minimal");

    String body = "{";
    body += "\"nivel_cm\":"          + String(nivel_atual, 1)                      + ",";
    body += "\"bomba_ligada\":"      + String(bomba_ligada     ? "true" : "false")  + ",";
    body += "\"erro_sensor\":"       + String(agua_erro_sensor ? "true" : "false")  + ",";
    body += "\"agua_offline\":"      + String(agua_offline ? "true" : "false")      + ",";
    body += "\"ladrao_ativo\":"      + String(agua_ladrao_ativo ? "true" : "false") + ",";
    body += "\"celular_presente\":"  + String(celular_presente ? "true" : "false")  + ",";
    body += "\"erros_ativos\":"      + String(erros_ativos)                         + ",";
    body += "\"pzem_w\":"            + String(pzem_potencia_w)                      + ",";
    body += "\"pzem_v\":"            + String(pzem_tensao_v, 1)                     + ",";
    body += "\"pzem_a\":"            + String(pzem_corrente_a, 3)                   + ",";
    body += "\"pzem_pf\":"           + String(pzem_fp, 2)                           + ",";
    body += "\"pzem_kwh\":"          + String(pzem_energia_kwh, 3)                  + ",";
    body += "\"bomba_offline\":"     + String(bomba_esta_offline() ? "true" : "false") + ",";
    body += "\"agua_motivo_status\":"   + String(agua_motivo_status)                       + ",";
    body += "\"bomba_estado_bitmask\":" + String(bomba_estado_bitmask)                     + ",";
    body += "\"bomba_causa_desligamento\":" + String(bomba_causa_desligamento)           + ",";
    {
        struct tm _info_ntp;
        body += "\"ntp_sincronizado\":" + String(getLocalTime(&_info_ntp, 10) ? "true" : "false") + ",";
    }
    body += "\"wifi_falhou\":"       + String(wifi_ultima_tentativa_falhou ? "true" : "false") + ",";
    body += "\"comando_falhou\":"    + String(ultimo_comando_falhou ? "true" : "false")         + ",";
    body += "\"wifi_rssi\":"         + String(WiFi.RSSI());
    body += "}";

    int code = http.POST(body);
    if (code > 0 && code < 300) {
        falhas_supabase = 0;
        registrar_log(F("[SUPABASE] Telemetria enviada."));
    } else {
        falhas_supabase++;
        Serial.printf("[SUPABASE] Falha HTTP %d\n", code);
    }
    http.end();
}

// Converte epoch (UTC) para ISO8601 aceito pela coluna created_at do Supabase.
// Retorna string vazia se epoch inválido (NTP não sincronizado no momento da captura) —
// nesse caso o ponto é enviado sem created_at explícito e o Supabase usa o horário do insert.
static String epochParaISO8601(time_t epoch) {
    if (epoch < 1700000000) return "";  // sentinela: antes de ~2023 = NTP não sincronizado
    struct tm* t = gmtime(&epoch);
    char buf[25];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", t);
    return String(buf);
}

// -------------------------------------------------------------
// BUFFER DE TELEMETRIA — descarga em lote para o Supabase
// O ponteiro só é limpo após HTTP 200/201 confirmado
// -------------------------------------------------------------
void descarregar_buffer() {
    if (!wifi_conectado) return;
    if (String(SUPABASE_URL).indexOf("SEU_PROJETO") >= 0) return;
    if (telem_count == 0) return;

    Serial.printf("[BUFFER] Descarregando %d pontos.\n", telem_count);

    String body = "[";
    for (uint8_t i = 0; i < telem_count; i++) {
        PontoTelemetria& p = buffer_telemetria[i];
        if (i > 0) body += ",";
        body += "{";
        String iso = epochParaISO8601(p.timestamp_epoch);
        if (iso.length() > 0) body += "\"created_at\":\"" + iso + "\",";
        body += "\"nivel_cm\":"      + String(p.nivel_cm, 1)  + ",";
        body += "\"bomba_ligada\":"  + String(p.bomba_ligada ? "true" : "false") + ",";
        body += "\"erros_ativos\":"  + String(p.erros_ativos) + ",";
        body += "\"pzem_w\":"        + String(p.pzem_w)       + ",";
        body += "\"pzem_v\":"        + String(p.pzem_v, 1)    + ",";
        body += "\"pzem_a\":"        + String(p.pzem_a, 3)    + ",";
        body += "\"pzem_pf\":"       + String(p.pzem_pf, 2)   + ",";
        body += "\"pzem_kwh\":"      + String(p.pzem_kwh, 3)  + ",";
        body += "\"bomba_offline\":" + String(p.bomba_offline ? "true" : "false") + ",";
        body += "\"agua_motivo_status\":"   + String(p.agua_motivo_status)   + ",";
        body += "\"bomba_estado_bitmask\":" + String(p.bomba_estado_bitmask) + ",";
        body += "\"bomba_causa_desligamento\":" + String(p.bomba_causa_desligamento) + ",";
        body += "\"ntp_sincronizado\":" + String(p.ntp_sincronizado ? "true" : "false") + ",";
        body += "\"wifi_falhou\":"      + String(p.wifi_falhou    ? "true" : "false") + ",";
        body += "\"comando_falhou\":"   + String(p.comando_falhou ? "true" : "false");
        body += "}";
    }
    body += "]";

    HTTPClient http;
    String url = String(SUPABASE_URL) + "/rest/v1/telemetria_marica";
    http.begin(url);
    http.addHeader("Content-Type",  "application/json");
    http.addHeader("apikey",        SUPABASE_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
    http.addHeader("Prefer",        "return=minimal");

    int code = http.POST(body);
    http.end();

    // Limpa apenas com confirmação HTTP
    if (code == 200 || code == 201) {
        uint8_t enviados = telem_count;
        telem_idx   = 0;
        telem_count = 0;
        registrar_log("[BUFFER] Descarga confirmada. " + String(enviados) + " pontos enviados.");
    } else {
        Serial.printf("[BUFFER] Falha HTTP %d — buffer preservado.\n", code);
    }
}

// -------------------------------------------------------------
// SUPABASE — ENVIO DE SESSÕES PENDENTES NO CICLO WI-FI
// Envia sessões completas que ainda não foram sincronizadas
// Usa índice de controle para não reenviar sessões já enviadas
// -------------------------------------------------------------
void enviar_sessoes_pendentes() {
    if (!wifi_conectado) return;
    if (String(SUPABASE_URL).indexOf("SEU_PROJETO") >= 0) return;
    if (sessao_count == 0) return;

    uint8_t total    = min((uint8_t)sessao_count, (uint8_t)MAX_SESSOES);
    uint8_t enviadas = 0;

    for (uint8_t i = 0; i < total; i++) {
        uint8_t idx    = (sessao_idx - total + i + MAX_SESSOES) % MAX_SESSOES;
        SessaoBomba& s = sessoes[idx];
        if (!s.completa)  continue;  // sessão ainda aberta
        if (s.ja_enviada) continue;  // já foi enviada
        if (enviar_sessao_supabase(s, s.pzem_w_medio)) {  // usa valor congelado no fechamento
            s.ja_enviada = true;
            enviadas++;
        }
        esp_task_wdt_reset();
    }

    if (enviadas > 0)
        registrar_log("[SUPABASE] " + String(enviadas) + " sessao(es) sincronizada(s).");
}
void abrir_sessao_bomba() {
    if (sessao_aberta) return;
    sessao_aberta = true;
    SessaoBomba& s    = sessoes[sessao_idx];
    s.inicio_ms       = millis();
    s.fim_ms          = 0;
    s.duracao_s       = 0;
    s.nivel_inicio_cm = nivel_atual;
    s.nivel_fim_cm    = 0.0f;
    s.variacao_cm     = 0.0f;
    s.motivo_desliga  = 0;
    s.origem_liga     = origem_ultimo_comando_liga;  // 2026-07-25
    s.completa        = false;
    s.ja_enviada      = false;

    // Zera acumulador de potência para esta sessão
    pzem_acum_w        = 0;
    pzem_acum_amostras = 0;

    registrar_log("[SESSAO] Aberta. Nivel inicio: " + String(nivel_atual, 1) + " cm");
}

void fechar_sessao_bomba(uint8_t motivo) {
    if (!sessao_aberta) return;
    sessao_aberta    = false;
    SessaoBomba& s   = sessoes[sessao_idx];
    s.fim_ms         = millis();
    s.duracao_s      = (s.fim_ms - s.inicio_ms) / 1000UL;
    s.nivel_fim_cm   = nivel_atual;
    s.variacao_cm    = s.nivel_fim_cm - s.nivel_inicio_cm;
    s.motivo_desliga = motivo;
    s.completa       = true;

    // Potência média consolidada e congelada na struct no momento do fechamento
    s.pzem_w_medio = (pzem_acum_amostras > 0)
                     ? (pzem_acum_w / pzem_acum_amostras)
                     : 0;

    registrar_log("[SESSAO] Fechada. " + String(s.duracao_s) + "s | " +
                  String(s.nivel_inicio_cm, 0) + "->" + String(s.nivel_fim_cm, 0) +
                  "cm | Delta:" + String(s.variacao_cm, 1) + "cm | motivo:" + String(motivo) +
                  " | PZEM medio:" + String(s.pzem_w_medio) + "W");

    tempo_total_bomba_s += s.duracao_s;
    salvar_params_ctrl();

    sessao_idx = (sessao_idx + 1) % MAX_SESSOES;
    if (sessao_count < MAX_SESSOES) sessao_count++;

    // Envia sessão ao Supabase se Wi-Fi estiver ativo neste momento
    if (wifi_conectado && String(SUPABASE_URL).indexOf("SEU_PROJETO") < 0) {
        if (enviar_sessao_supabase(s, s.pzem_w_medio)) {
            s.ja_enviada = true;
        }
    }
}

// -------------------------------------------------------------
// TELEMETRIA PERIÓDICA
// -------------------------------------------------------------
void registrar_telemetria() {
    static uint32_t ultimo = 0;
    if (millis() - ultimo < INTERVALO_TELEMETRIA_MS) return;
    ultimo = millis();

    PontoTelemetria& p = buffer_telemetria[telem_idx];
    p.timestamp_epoch = time(nullptr);  // 0 se NTP ainda não sincronizou — tratado no envio
    p.nivel_cm     = nivel_atual;
    p.bomba_ligada = bomba_ligada;
    p.erros_ativos = erros_ativos;
    p.pzem_w       = pzem_potencia_w;
    p.pzem_v       = pzem_tensao_v;
    p.pzem_a       = pzem_corrente_a;
    p.pzem_pf      = pzem_fp;
    p.pzem_kwh     = pzem_energia_kwh;
    p.bomba_offline = bomba_esta_offline();
    p.agua_motivo_status   = agua_motivo_status;
    p.bomba_estado_bitmask = bomba_estado_bitmask;
    p.bomba_causa_desligamento = bomba_causa_desligamento;
    struct tm _info_ntp;
    p.ntp_sincronizado = getLocalTime(&_info_ntp, 10);
    p.wifi_falhou    = wifi_ultima_tentativa_falhou;
    p.comando_falhou = ultimo_comando_falhou;

    telem_idx = (telem_idx + 1) % MAX_TELEMETRIA;
    if (telem_count < MAX_TELEMETRIA) telem_count++;
}

// -------------------------------------------------------------
// CALLBACKS ESP-NOW
// -------------------------------------------------------------
void cb_envio(const uint8_t* m, esp_now_send_status_t s) {
    if (s != ESP_NOW_SEND_SUCCESS) {
        Serial.println(F("[CONTROLE] Falha envio ESP-NOW."));
        ultimo_comando_falhou = true;  // 2026-07-25 -- visível remotamente agora (marica-131)
    } else {
        ultimo_comando_falhou = false;
    }
}

void cb_recepcao(const uint8_t* mac_addr, const uint8_t* dados, int len) {
    if (len < 1) return;
    uint8_t tipo = dados[0];

    // Pacote principal da Bomba → Controle
    if (tipo == PKT_STATUS_COMPLETO && len >= (int)sizeof(PacketStatusCompleto)) {
        PacketStatusCompleto pkt;
        memcpy(&pkt, dados, sizeof(pkt));

        bool bomba_era    = bomba_ligada;
        bool agua_err_era = agua_erro_sensor;
        bool agua_off_era = agua_offline;
        uint8_t erros_era = erros_ativos;

        nivel_atual         = pkt.agua_distancia_cm;
        agua_erro_sensor    = pkt.agua_erro_sensor;
        agua_offline        = pkt.agua_offline;
        agua_ladrao_ativo   = pkt.agua_ladrao_ativo;
        bomba_ligada        = pkt.bomba_rele_estado;
        erros_ativos        = pkt.bomba_erro_bitmask;
        pzem_potencia_w     = pkt.pzem_potencia_w;
        pzem_tensao_v       = pkt.pzem_tensao_v;
        pzem_corrente_a     = pkt.pzem_corrente_a;
        pzem_fp             = pkt.pzem_fp;
        pzem_energia_kwh    = pkt.pzem_energia_kwh;
        agua_motivo_status   = pkt.agua_motivo_status;    // 2026-07-25
        bomba_estado_bitmask = pkt.bomba_estado_bitmask;  // 2026-07-25
        bomba_causa_desligamento = pkt.bomba_causa_desligamento;  // 2026-07-25
        ultimo_pacote_bomba = millis();

        // Acumula potência durante sessão ativa para cálculo de média
        if (sessao_aberta && pkt.pzem_potencia_w > 0) {
            pzem_acum_w        += pkt.pzem_potencia_w;
            pzem_acum_amostras++;
        }

        // TRADUÇÃO DE ESTADO → LOG CENTRALIZADO
        if (bomba_ligada && !bomba_era)
            registrar_log(F("[BOMBA] Rele ATRACADO (Motor ON)"));
        if (!bomba_ligada && bomba_era)
            registrar_log(F("[BOMBA] Rele DESARMADO (Motor OFF)"));
        if (erros_ativos != erros_era)
            registrar_log("[BOMBA] Erros: 0x" + String(erros_ativos, HEX));
        if (agua_erro_sensor && !agua_err_era)
            registrar_log(F("[AGUA] ALERTA: Sensor de nivel offline/invalido"));
        if (!agua_erro_sensor && agua_err_era)
            registrar_log(F("[AGUA] Sensor de nivel restaurado"));
        if (agua_offline && !agua_off_era)
            registrar_log(F("[AGUA] Silencio de radio (Agua->Bomba)."));
        if (!agua_offline && agua_off_era)
            registrar_log(F("[AGUA] Comunicacao restaurada (Agua->Bomba)."));

        // Gerência de sessões
        if (bomba_ligada && !bomba_era) abrir_sessao_bomba();
        if (!bomba_ligada && bomba_era) {
            uint8_t motivo = 0;
            if      (erros_ativos)               motivo = 2;
            else if (comando_desliga_pendente)    motivo = 1;
            comando_desliga_pendente = false;
            fechar_sessao_bomba(motivo);
        }

        // Dispara ciclo Wi-Fi imediato quando bomba liga ou desliga
        if (bomba_ligada != bomba_era) {
            ciclo_wifi_imediato = true;
        }

        if (bomba_ligada != bomba_era) bip(1);

        // Registro inteligente de nível — apenas quando variar > 1 cm
        // Evita lotar o ring buffer com leituras repetidas idênticas
        static float nivel_ultimo_log = -999.0f;
        if (fabsf(nivel_atual - nivel_ultimo_log) >= 1.0f) {
            uint8_t pct = calcular_pct(nivel_atual);
            registrar_log("[NIVEL] " + String(pct) + "% (" + String(nivel_atual, 0) + " cm)" +
                          (bomba_ligada ? " [bomba ON]" : ""));
            nivel_ultimo_log = nivel_atual;
        }

        Serial.printf("[%s] [CONTROLE] Nivel=%.1f cm | Bomba=%s | Erros=0x%02X | PZEM=%uW\n",
                      obter_hora().c_str(), nivel_atual, bomba_ligada ? "ON" : "OFF",
                      erros_ativos, pzem_potencia_w);
    }

    // Comando OTA — Cardputer
    if (tipo == CMD_OTA) {
        Serial.println(F("[CONTROLE] CMD_OTA recebido."));
        ota_requisitado = true;
    }

    // Comando servidor web — Cardputer
    if (tipo == CMD_WEB_SERVER) {
        registrar_log(F("[CONTROLE] CMD_WEB_SERVER recebido via Cardputer."));
        if (modo_wifi == WIFI_MODO_NENHUM && !bomba_ligada) {
            webserver_requisitado = true;  // flag — abrir_webserver() executada no loop()
        } else {
            registrar_log(F("[CONTROLE] CMD_WEB_SERVER ignorado — Wi-Fi ativo ou bomba ligada."));
        }
    }
}

// -------------------------------------------------------------
// ENVIO DE COMANDOS PARA A BOMBA
// -------------------------------------------------------------
void enviar_comando(uint8_t tipo_cmd, bool ignorar_nivel) {
    if (!espnow_ok) {
        digitalWrite(GPIO_BUZZER, LOW); delay(50); digitalWrite(GPIO_BUZZER, HIGH);
        Serial.println(F("[CONTROLE] Cmd rejeitado — ESP-NOW inativo."));
        return;
    }
    PacketComandoBomba cmd = {};
    cmd.tipo              = tipo_cmd;
    cmd.horario_permitido = horario_bomba_permitido();
    cmd.ignorar_nivel     = ignorar_nivel;
    esp_now_send(MAC_BOMBA, (uint8_t*)&cmd, sizeof(cmd));
}

void enviar_config_niveis(uint8_t liga, uint8_t desliga, uint8_t seguranca, uint8_t manual_min, uint16_t timeout_min) {
    if (!espnow_ok) {
        digitalWrite(GPIO_BUZZER, LOW); delay(50); digitalWrite(GPIO_BUZZER, HIGH);
        registrar_log(F("[CONTROLE] Erro TX: ESP-NOW inativo."));
        return;
    }
    PacketConfigNiveis pkt = {};
    pkt.tipo                = CMD_SET_NIVEIS;
    pkt.nivel_liga_cm       = liga;
    pkt.nivel_desliga_cm    = desliga;
    pkt.nivel_seguranca_cm  = seguranca;
    pkt.nivel_manual_min_cm = manual_min;
    pkt.timeout_minutos     = timeout_min;
    esp_now_send(MAC_BOMBA, (uint8_t*)&pkt, sizeof(pkt));
    registrar_log("[ESP-NOW] TX Config → Liga:" + String(liga) +
                  "cm Desliga:" + String(desliga) +
                  "cm Seg:" + String(seguranca) +
                  "cm Manual:" + String(manual_min) +
                  "cm TMax:" + String(timeout_min) + "m");
}

// -------------------------------------------------------------
// STATUS PARA O CARDPUTER (modo Monitor) — 2026-07-30
// -------------------------------------------------------------
// Push periódico, não sob demanda (ver loop() -- mesmo padrão do keep-alive
// CMD_PING_CONTROLE já usado com a Bomba). nivel_pct sai pronto de
// calcular_pct() -- fonte única de verdade, o Cardputer só exibe. Silêncio
// de rádio (Cardputer desligado/fora de alcance) não gera erro aqui: send
// falha via cb_envio() como qualquer outro pacote, sem tratamento especial --
// o próprio Cardputer detecta o silêncio do lado dele (timeout de pacote).
void enviar_status_cardputer() {
    if (!espnow_ok) return;

    PacketStatusCardputer pkt = {};
    pkt.tipo               = PKT_STATUS_CARDPUTER;
    pkt.nivel_pct           = calcular_pct(nivel_atual);
    pkt.nivel_distancia_cm  = nivel_atual;
    pkt.bomba_ligada        = bomba_ligada;
    pkt.modo_atual          = (uint8_t)modo_atual;
    pkt.agua_erro_sensor    = agua_erro_sensor;
    pkt.agua_offline        = agua_offline;
    pkt.bomba_offline       = bomba_esta_offline();

    esp_now_send(MAC_CARDPUTER, (uint8_t*)&pkt, sizeof(pkt));
}

// -------------------------------------------------------------
// SERVIDOR WEB (BTN1 hold 3s)
// -------------------------------------------------------------
void rota_raiz() {
    // Redireciona / para /config
    servidor.sendHeader("Location", "/config");
    servidor.send(302, "text/plain", "");
}

void rota_config() {
    inicio_wifi = millis();  // reseta cronômetro de inatividade
    servidor.sendHeader("Cache-Control", "no-cache");
    servidor.setContentLength(CONTENT_LENGTH_UNKNOWN);
    servidor.send(200, "text/html; charset=utf-8", "");

    servidor.sendContent(F("<!DOCTYPE html><html><head>"));
    servidor.sendContent(F("<meta charset='utf-8'>"));
    servidor.sendContent(F("<meta name='viewport' content='width=device-width,initial-scale=1'>"));
    servidor.sendContent(F("<title>Marica — Config</title>"));
    servidor.sendContent(F("<style>"));
    servidor.sendContent(F(":root{--bg:#0f1117;--panel:#1a1d27;--border:#2a2d3a;--accent:#00d4aa;--warn:#f5a623;--err:#e53e3e;--txt:#e2e8f0;--txt2:#718096;}"));
    servidor.sendContent(F("*{box-sizing:border-box;margin:0;padding:0;}"));
    servidor.sendContent(F("body{background:var(--bg);color:var(--txt);font-family:'Courier New',monospace;padding:16px;max-width:480px;margin:auto;}"));
    servidor.sendContent(F("h1{color:var(--accent);font-size:1rem;letter-spacing:.2em;text-transform:uppercase;border-bottom:1px solid var(--border);padding-bottom:8px;margin-bottom:20px;}"));
    servidor.sendContent(F("h2{font-size:.7rem;letter-spacing:.25em;text-transform:uppercase;color:var(--txt2);margin-bottom:12px;margin-top:20px;}"));
    servidor.sendContent(F(".card{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:16px;margin-bottom:14px;}"));
    servidor.sendContent(F(".row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid var(--border);}"));
    servidor.sendContent(F(".row:last-child{border-bottom:none;}"));
    servidor.sendContent(F(".lbl{color:var(--txt2);font-size:.8rem;}"));
    servidor.sendContent(F(".val{font-size:.9rem;font-weight:bold;}"));
    servidor.sendContent(F(".on{color:var(--accent);}.off{color:var(--txt2);}.warn{color:var(--warn);}.err{color:var(--err);}"));
    servidor.sendContent(F(".nivel{font-size:2.2rem;text-align:center;color:var(--accent);padding:12px 0;letter-spacing:.1em;}"));
    servidor.sendContent(F("input[type=text]{width:100%;padding:8px;background:#0f1117;border:1px solid var(--border);color:var(--txt);border-radius:4px;font-family:inherit;font-size:.85rem;margin-top:6px;}"));
    servidor.sendContent(F("label{font-size:.78rem;color:var(--txt2);display:block;margin-top:12px;}"));
    servidor.sendContent(F(".btn{display:block;width:100%;padding:10px;border:none;border-radius:4px;cursor:pointer;font-family:inherit;font-size:.85rem;letter-spacing:.08em;margin-top:8px;text-align:center;}"));
    servidor.sendContent(F(".btn-g{background:var(--accent);color:#0f1117;font-weight:bold;}"));
    servidor.sendContent(F(".btn-r{background:var(--err);color:#fff;}"));
    servidor.sendContent(F(".btn-s{background:var(--panel);border:1px solid var(--border);color:var(--txt);}"));
    servidor.sendContent(F(".warn-box{background:#1a1500;border:1px solid var(--warn);border-radius:4px;padding:8px 12px;font-size:.75rem;color:var(--warn);margin-bottom:14px;}"));
    servidor.sendContent(F("</style></head><body>"));

    servidor.sendContent(F("<h1>&#9632; Marica Controle</h1>"));
    servidor.sendContent(F("<div class='warn-box'>&#9888; Dados congelados — ESP-NOW suspenso durante sessao web.</div>"));

    // STATUS
    servidor.sendContent(F("<div class='card'>"));
    servidor.sendContent(F("<h2>Status</h2>"));

    if (agua_erro_sensor) {
        servidor.sendContent(F("<div class='nivel err'>SENSOR ERR</div>"));
    } else {
        uint8_t pct = calcular_pct(nivel_atual);
        servidor.sendContent("<div class='nivel'>" + String(pct) + "%</div>");
        servidor.sendContent("<div style='text-align:center;color:var(--txt2);font-size:.75rem;margin-top:-8px;margin-bottom:8px;'>"
                             + String(nivel_atual, 0) + " cm</div>");
    }

    servidor.sendContent("<div class='row'><span class='lbl'>Bomba</span><span class='val " +
        String(bomba_ligada ? "on" : "off") + "'>" +
        String(bomba_ligada ? "LIGADA" : "DESLIGADA") + "</span></div>");

    servidor.sendContent("<div class='row'><span class='lbl'>Celular</span><span class='val " +
        String(celular_presente ? "on" : "off") + "'>" +
        String(celular_presente ? "PRESENTE" : "AUSENTE") + "</span></div>");

    servidor.sendContent("<div class='row'><span class='lbl'>Modo</span><span class='val'>" +
        String(modo_atual == MODO_AUTOMATICO ? "AUTOMATICO" : "SEMIAUTOMATICO") + "</span></div>");

    // Erros/estados por bitmask -- 2026-07-27: ladrão relabelado (não é mais
    // rotulado "ERRO3", é flag de estado decorrente de evento -- zero mudança
    // de comportamento, continua no mesmo bit e continua acendendo o LED
    // vermelho). Novo: relé colado, vindo de bomba_estado_bitmask (marica-152).
    servidor.sendContent(F("<div class='row'><span class='lbl'>Erros</span><span class='val'>"));
    if (erros_ativos == 0 && !(bomba_estado_bitmask & ESTADO_RELE_COLADO)) {
        servidor.sendContent(F("<span class='on'>OK</span>"));
    } else {
        if (erros_ativos & ERRO_1_TIMEOUT)
            servidor.sendContent(F("<span class='err'>ERRO1:Timeout </span>"));
        if (erros_ativos & ERRO_3_LADRAO)
            servidor.sendContent(F("<span class='err'>Ladrao:ATIVO </span>"));
        if (erros_ativos & ERRO_5_PZEM)
            servidor.sendContent(F("<span class='err'>ERRO5:PZEM </span>"));
        if (bomba_estado_bitmask & ESTADO_RELE_COLADO)
            servidor.sendContent(F("<span class='err'>RELE:Colado </span>"));
    }
    servidor.sendContent(F("</span></div>"));

    // 2026-07-27 -- fecha o gap remanescente do marica-131: falha de comando via
    // rota_cmd() (ações da página web) tinha visibilidade só no Serial/logs.
    if (ultimo_comando_falhou) {
        servidor.sendContent(F("<div class='warn-box'>&#9888; Ultimo comando falhou no envio ESP-NOW.</div>"));
    }

    servidor.sendContent("<div class='row'><span class='lbl'>Consumo</span><span class='val'>" +
        String(pzem_potencia_w) + " W</span></div>");

    servidor.sendContent("<div class='row'><span class='lbl'>Ultimo pkt</span><span class='val'>" +
        String(ultimo_pacote_bomba > 0 ?
               String((millis() - ultimo_pacote_bomba) / 1000) + "s" : "--") + "</span></div>");

    servidor.sendContent("<div class='row'><span class='lbl'>Horimetro</span><span class='val'>" +
        String(tempo_total_bomba_s / 3600) + "h " +
        String((tempo_total_bomba_s % 3600) / 60) + "min</span></div>");

    servidor.sendContent("<div class='row'><span class='lbl'>Supabase</span><span class='val " +
        String(falhas_supabase == 0 ? "on" : "warn") + "'>" +
        String(falhas_supabase == 0 ? "OK" : String(falhas_supabase) + " falhas") +
        "</span></div>");
    servidor.sendContent(F("</div>"));

    // HISTÓRICO DE SESSÕES
    servidor.sendContent(F("<div class='card'>"));
    servidor.sendContent(F("<h2>Sessoes de Bomba</h2>"));
    if (sessao_count == 0) {
        servidor.sendContent(F("<div class='row'><span class='lbl'>Sem sessoes</span></div>"));
    } else {
        uint8_t total = min((uint8_t)sessao_count, (uint8_t)MAX_SESSOES);
        for (int8_t i = total - 1; i >= 0; i--) {
            uint8_t idx = (sessao_idx - 1 - i + MAX_SESSOES) % MAX_SESSOES;
            SessaoBomba& s = sessoes[idx];
            if (!s.completa) continue;
            const char* mot = (s.motivo_desliga == 0) ? "nivel" :
                              (s.motivo_desliga == 1) ? "manual" : "erro";
            servidor.sendContent("<div class='row'><span class='lbl'>" +
                String(s.duracao_s) + "s | " +
                String(s.nivel_inicio_cm, 0) + "->" +
                String(s.nivel_fim_cm, 0) + "cm | " + mot +
                "</span><span class='val " +
                String(s.variacao_cm >= 0 ? "on" : "warn") + "'>" +
                String(s.variacao_cm >= 0 ? "+" : "") +
                String(s.variacao_cm, 1) + "cm</span></div>");
        }
    }
    servidor.sendContent(F("</div>"));

    // COMANDOS
    servidor.sendContent(F("<div class='card'>"));
    servidor.sendContent(F("<h2>Comandos</h2>"));
    servidor.sendContent(F("<a href='/cmd?a=liga' class='btn btn-g'>Ligar Bomba</a>"));
    servidor.sendContent(F("<a href='/cmd?a=desliga' class='btn btn-r' style='margin-top:6px'>Desligar Bomba</a>"));
    servidor.sendContent(F("<a href='/cmd?a=reset' class='btn btn-s' style='margin-top:14px; color:var(--warn); border-color:var(--warn)'>&#9888; Resetar Bloqueios (NVS)</a>"));
    servidor.sendContent(F("<a href='/logs' class='btn btn-s' style='margin-top:8px'>&#128464; Ver Logs Internos</a>"));
    servidor.sendContent(F("</div>"));

    // LIMIARES OPERACIONAIS DA BOMBA
    servidor.sendContent(F("<div class='card'>"));
    servidor.sendContent(F("<h2>Limiares de Operacao (Bomba)</h2>"));
    servidor.sendContent(F("<form action='/set_niveis' method='get'>"));
    servidor.sendContent(F("<div class='row' style='border:none;padding:0;'>"));
    servidor.sendContent(F("<div style='width:31%;'>"));
    servidor.sendContent(F("<label>Ligar em (cm)</label>"));
    servidor.sendContent("<input type='number' name='liga' value='" + String(cfg_liga_cm) + "'>");
    servidor.sendContent(F("</div><div style='width:31%;'>"));
    servidor.sendContent(F("<label>Desligar em (cm)</label>"));
    servidor.sendContent("<input type='number' name='desliga' value='" + String(cfg_desliga_cm) + "'>");
    servidor.sendContent(F("</div><div style='width:31%;'>"));
    servidor.sendContent(F("<label>Seguranca (cm)</label>"));
    servidor.sendContent("<input type='number' name='seguranca' value='" + String(cfg_nivel_seguranca) + "'>");
    servidor.sendContent(F("</div></div>"));
    servidor.sendContent(F("<label style='margin-top:10px;'>Nivel Minimo p/ Partida Manual (cm)</label>"));
    servidor.sendContent("<input type='number' name='manual' value='" + String(cfg_manual_min) + "'>");
    servidor.sendContent(F("<label style='margin-top:10px;'>Timeout Maximo (minutos)</label>"));
    servidor.sendContent("<input type='number' name='timeout' value='" + String(cfg_timeout_m) + "'>");
    servidor.sendContent(F("<button class='btn btn-g' type='submit' style='margin-top:14px'>Gravar e Enviar p/ Bomba</button>"));
    servidor.sendContent(F("</form></div>"));

    // CONFIGURAÇÃO DE CELULARES
    servidor.sendContent(F("<div class='card'>"));
    servidor.sendContent(F("<h2>IPs Monitorados (Presenca)</h2>"));
    servidor.sendContent(F("<form action='/salvar_config' method='get'>"));
    for (uint8_t i = 0; i < MAX_CELULARES; i++) {
        servidor.sendContent("<label>Celular " + String(i + 1) + "</label>");
        servidor.sendContent("<input type='text' name='cel" + String(i) +
                             "' value='" + String(ip_celulares[i]) + "'>");
    }
    servidor.sendContent(F("<button class='btn btn-g' type='submit' style='margin-top:14px'>Salvar</button>"));
    servidor.sendContent(F("</form></div>"));

    // ENCERRAR
    servidor.sendContent(F("<div class='card'>"));
    servidor.sendContent(F("<a href='/encerrar' class='btn btn-s'>&#10006; Encerrar Servidor Web</a>"));
    servidor.sendContent(F("</div>"));

    servidor.sendContent(F("</body></html>"));
    servidor.sendContent("");
}

void rota_logs() {
    inicio_wifi = millis();  // reseta inatividade
    servidor.setContentLength(CONTENT_LENGTH_UNKNOWN);
    servidor.send(200, "text/html; charset=utf-8", "");

    servidor.sendContent(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"));
    servidor.sendContent(F("<meta name='viewport' content='width=device-width,initial-scale=1'>"));
    servidor.sendContent(F("<title>Logs do Sistema</title>"));
    servidor.sendContent(F("<style>"));
    servidor.sendContent(F("body{background:#0f1117;color:#718096;font-family:'Courier New',monospace;padding:16px;max-width:600px;margin:auto;}"));
    servidor.sendContent(F("h1{color:#fff;font-size:1rem;letter-spacing:.2em;text-transform:uppercase;border-bottom:1px solid #2a2d3a;padding-bottom:8px;margin-bottom:16px;}"));
    servidor.sendContent(F(".entry{font-size:.75rem;line-height:1.6;padding:2px 0;border-bottom:1px solid #1a1d27;}"));
    servidor.sendContent(F(".lvl{color:#00d4aa;}"));  // nível em verde destaque
    servidor.sendContent(F(".btn{display:block;width:100%;padding:12px;background:#1a1d27;color:#e2e8f0;text-align:center;text-decoration:none;border-radius:4px;margin-top:20px;border:1px solid #2a2d3a;}"));
    servidor.sendContent(F("</style></head><body>"));
    servidor.sendContent(F("<h1>&#128464; Logs & Niveis</h1>"));

    if (log_count == 0) {
        servidor.sendContent(F("<div class='entry'>Nenhum log registrado ainda.</div>"));
    } else {
        uint8_t total = min((uint8_t)log_count, (uint8_t)MAX_LOGS);
        for (int i = 0; i < total; i++) {
            uint8_t current = (log_idx - log_count + i + MAX_LOGS) % MAX_LOGS;
            String entry = log_interno[current];
            // Entradas de nível em verde — destaque visual para leitura rápida
            if (entry.indexOf("[NIVEL]") != -1) {
                servidor.sendContent("<div class='entry lvl'>" + entry + "</div>");
            } else {
                servidor.sendContent("<div class='entry'>" + entry + "</div>");
            }
        }
    }

    servidor.sendContent(F("<a href='/config' class='btn'>&#8592; Voltar</a>"));
    servidor.sendContent(F("</body></html>"));
    servidor.sendContent("");
}

void rota_salvar_config() {
    inicio_wifi = millis();  // reseta cronômetro de inatividade
    for (uint8_t i = 0; i < MAX_CELULARES; i++) {
        char chave[8];
        snprintf(chave, sizeof(chave), "cel%d", i);
        if (servidor.hasArg(chave)) {
            String val = servidor.arg(chave);
            val.trim();
            val.toCharArray(ip_celulares[i], sizeof(ip_celulares[i]));
        }
    }
    salvar_params_net();
    Serial.println(F("[WEB] IPs salvos."));
    servidor.sendHeader("Location", "/config");
    servidor.send(302, "text/plain", "");
}

void rota_cmd() {
    inicio_wifi = millis();  // reseta cronômetro de inatividade

    String acao = servidor.arg("a");
    uint8_t cmd_a_enviar = 0;
    bool    ignorar_nivel = false;

    if (acao == "liga") {
        cmd_a_enviar  = CMD_LIGA_BOMBA;
        ignorar_nivel = true;  // botão web: liga independente do nível (Peter, 2026-07-24)
        origem_ultimo_comando_liga = ORIGEM_LIGA_WEB;  // 2026-07-25
        Serial.printf("[%s] [WEB] Cmd: Liga Bomba (ignorando nivel).\n", obter_hora().c_str());
    } else if (acao == "desliga") {
        comando_desliga_pendente = true;
        cmd_a_enviar = CMD_DESLIGA_BOMBA;
        Serial.printf("[%s] [WEB] Cmd: Desliga Bomba.\n", obter_hora().c_str());
    } else if (acao == "reset") {
        cmd_a_enviar = CMD_RESET_ERROS;
        Serial.printf("[%s] [WEB] Cmd: Resetar Bloqueios.\n", obter_hora().c_str());
    }

    if (cmd_a_enviar != 0) {
        // 1. Envia página de transição com meta-refresh de 6s
        // O navegador aguarda e reconecta sozinho ao servidor que será reaberto
        String html = F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
        html += F("<meta http-equiv='refresh' content='6;url=/config'>");
        html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
        html += F("<style>body{background:#0f1117;color:#00d4aa;font-family:monospace;text-align:center;padding:50px;}</style>");
        html += F("</head><body><h2>Transmitindo via Radio...</h2><p>Aguarde a reconexao (6s).</p></body></html>");
        servidor.send(200, "text/html", html);

        // 2. Aguarda o HTML ser transmitido ao celular antes de derrubar o Wi-Fi
        delay(300);

        // 3. Derruba Wi-Fi e acorda o ESP-NOW
        Serial.println(F("[WEB] Suspendendo Wi-Fi para disparar o radio..."));
        servidor.stop();
        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        delay(200);

        iniciar_espnow();
        delay(100);
        enviar_comando(cmd_a_enviar, ignorar_nivel);
        delay(100);

        // 4. Reconecta Wi-Fi e restaura o servidor para o meta-refresh do navegador
        Serial.println(F("[WEB] Retomando servidor web..."));
        if (conectar_wifi_nao_bloqueante()) {
            webserver_ativo = true;
            modo_wifi       = WIFI_MODO_WEBSERVER;
            inicio_wifi     = millis();
            servidor.begin();
        } else {
            // Falha na reconexão — encerra completamente
            webserver_ativo = false;
            modo_wifi       = WIFI_MODO_NENHUM;
            iniciar_espnow();
        }
    } else {
        servidor.sendHeader("Location", "/config");
        servidor.send(302, "text/plain", "");
    }
}

void rota_encerrar() {
    servidor.sendHeader("Location", "/config");
    servidor.send(302, "text/plain", "");
    fechar_webserver();
}

void rota_set_niveis() {
    inicio_wifi = millis();

    uint8_t  nova_liga      = servidor.hasArg("liga")      ? (uint8_t)servidor.arg("liga").toInt()      : cfg_liga_cm;
    uint8_t  novo_desliga   = servidor.hasArg("desliga")   ? (uint8_t)servidor.arg("desliga").toInt()   : cfg_desliga_cm;
    uint8_t  nova_seguranca = servidor.hasArg("seguranca") ? (uint8_t)servidor.arg("seguranca").toInt() : cfg_nivel_seguranca;
    uint8_t  novo_manual    = servidor.hasArg("manual")    ? (uint8_t)servidor.arg("manual").toInt()    : cfg_manual_min;
    uint16_t novo_timeout   = servidor.hasArg("timeout")   ? (uint16_t)servidor.arg("timeout").toInt()  : cfg_timeout_m;

    // Validação: liga > desliga, segurança > liga, timeout > 0, manual > 0
    // (ordem completa esperada pela Bomba, marica-032: seguranca > liga > manual_min)
    if (nova_liga <= novo_desliga || nova_seguranca <= nova_liga || novo_timeout == 0 ||
        novo_manual == 0 || novo_manual >= nova_liga) {
        String html = F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
        html += F("<meta http-equiv='refresh' content='4;url=/config'>");
        html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
        html += F("<style>body{background:#0f1117;color:#e53e3e;font-family:monospace;text-align:center;padding:50px;}</style>");
        html += F("</head><body><h2>&#9888; Parametros invalidos</h2>");
        html += F("<p>Ligar deve ser maior que Desligar.<br>Seguranca deve ser maior que Ligar.<br>Ligar deve ser maior que o Nivel Minimo Manual.<br>Timeout e Nivel Minimo devem ser maiores que zero.</p>");
        html += F("<p>Voltando em 4s...</p></body></html>");
        servidor.send(200, "text/html", html);
        registrar_log(F("[WEB] SET_NIVEIS rejeitado: parametros invalidos."));
        return;
    }

    cfg_liga_cm         = nova_liga;
    cfg_desliga_cm      = novo_desliga;
    cfg_nivel_seguranca = nova_seguranca;
    cfg_manual_min      = novo_manual;
    cfg_timeout_m       = novo_timeout;
    salvar_params_ctrl();
    registrar_log("[WEB] Limiares salvos. Liga:" + String(cfg_liga_cm) +
                  "cm Desliga:" + String(cfg_desliga_cm) +
                  "cm Seg:" + String(cfg_nivel_seguranca) +
                  "cm Manual:" + String(cfg_manual_min) +
                  "cm TMax:" + String(cfg_timeout_m) + "m");

    String html = F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
    html += F("<meta http-equiv='refresh' content='6;url=/config'>");
    html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
    html += F("<style>body{background:#0f1117;color:#00d4aa;font-family:monospace;text-align:center;padding:50px;}</style>");
    html += F("</head><body><h2>Transmitindo parametros via Radio...</h2><p>Aguarde a reconexao (6s).</p></body></html>");
    servidor.send(200, "text/html", html);
    delay(300);

    servidor.stop();
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(200);

    iniciar_espnow();
    delay(100);
    enviar_config_niveis(cfg_liga_cm, cfg_desliga_cm, cfg_nivel_seguranca, cfg_manual_min, cfg_timeout_m);
    delay(100);

    if (conectar_wifi_nao_bloqueante()) {
        webserver_ativo = true;
        modo_wifi       = WIFI_MODO_WEBSERVER;
        inicio_wifi     = millis();
        servidor.begin();
    } else {
        webserver_ativo = false;
        modo_wifi       = WIFI_MODO_NENHUM;
        iniciar_espnow();
    }
}

void abrir_webserver() {
    if (webserver_ativo) return;
    if (!conectar_wifi_nao_bloqueante()) return;
    modo_wifi       = WIFI_MODO_WEBSERVER;
    inicio_wifi     = millis();
    webserver_ativo = true;
    servidor.on("/",             rota_raiz);
    servidor.on("/config",       rota_config);
    servidor.on("/salvar_config",rota_salvar_config);
    servidor.on("/cmd",          rota_cmd);
    servidor.on("/logs",         rota_logs);
    servidor.on("/set_niveis",   rota_set_niveis);
    servidor.on("/encerrar",     rota_encerrar);
    servidor.begin();
    registrar_log(F("[WEB] Servidor aberto."));
    bip(3);
}

void fechar_webserver() {
    servidor.stop();
    webserver_ativo = false;
    modo_wifi       = WIFI_MODO_NENHUM;
    desconectar_wifi();
    Serial.println(F("[WEB] Servidor encerrado."));
}

// -------------------------------------------------------------
// BUZZER
// -------------------------------------------------------------
void bip(uint8_t n) {
    if (!buzzer_permitido()) return;
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(GPIO_BUZZER, LOW);  delay(100);
        digitalWrite(GPIO_BUZZER, HIGH); delay(100);
    }
}

static uint32_t buzzer_erro_inicio = 0;
static bool     buzzer_erro_ativo  = false;

void ativar_buzzer_erro() {
    if (!buzzer_permitido()) return;
    buzzer_erro_ativo  = true;
    buzzer_erro_inicio = millis();
}

void loop_buzzer_erro() {
    if (!buzzer_erro_ativo) return;
    if (millis() - buzzer_erro_inicio > 10000UL) {  // 10 segundos máximo
        buzzer_erro_ativo = false;
        digitalWrite(GPIO_BUZZER, HIGH);  // desliga (lógica invertida)
        return;
    }
    static uint32_t ultimo_bip = 0;
    if (millis() - ultimo_bip >= 1000) {
        ultimo_bip = millis();
        digitalWrite(GPIO_BUZZER, LOW);  delay(200);
        digitalWrite(GPIO_BUZZER, HIGH);
    }
}

// -------------------------------------------------------------
// LEDs
// -------------------------------------------------------------
void loop_led_azul() {
    static bool     estado = false;
    static uint32_t ultimo = 0;
    bool tudo_ok = espnow_ok &&
                   (ultimo_pacote_bomba > 0) &&
                   (millis() - ultimo_pacote_bomba < 60000UL) &&
                   (erros_ativos == 0);
    if (tudo_ok) {
        bool novo = ((millis() % 1000) < 800);
        if (novo != estado) { estado = novo; digitalWrite(GPIO_LED_AZUL, estado); }
    } else {
        if (millis() - ultimo >= 100) {
            ultimo = millis(); estado = !estado;
            digitalWrite(GPIO_LED_AZUL, estado);
        }
    }
}

// LED branco: estático em ESP-NOW | piscante durante Wi-Fi (Regra 5)
void loop_led_branco() {
    if (wifi_em_blink) {
        static uint32_t ultimo = 0; static bool estado = false;
        if (millis() - ultimo >= 250) {
            ultimo = millis(); estado = !estado;
            digitalWrite(GPIO_LED_BRANCO, estado);
        }
        return;
    }
    // Estático: HIGH = automático (celular presente) | LOW = semiautomático
    digitalWrite(GPIO_LED_BRANCO, celular_presente ? HIGH : LOW);
}

// Verdadeiro quando não há pacote da Bomba há mais de SINALEIRA_TIMEOUT_MS
// (ou nenhum pacote desde o boot). Fonte única de verdade -- usada pela
// sinaleira (visual), pela telemetria periódica e pelo envio direto ao
// Supabase, para que os três caminhos concordem sobre "dado confiável"
// (marica-090/092/094/098).
bool bomba_esta_offline() {
    return (ultimo_pacote_bomba == 0) ||
           (millis() - ultimo_pacote_bomba >= SINALEIRA_TIMEOUT_MS);
}

void loop_sinaleira() {
    // Sem comunicação recente da Bomba -- nivel_atual/bomba_ligada são o último
    // estado cacheado, não o estado corrente. Apaga tudo em vez de exibir dado
    // desatualizado como se fosse válido (marica-090/092/093).
    if (bomba_esta_offline()) {
        digitalWrite(GPIO_LED_VERDE,   LOW);
        digitalWrite(GPIO_LED_AMARELO, LOW);
        digitalWrite(GPIO_LED_VERM_S,  LOW);
        return;
    }

    if (bomba_ligada) {
        static uint32_t ultimo_anim = 0; static uint8_t fase = 0;
        if (millis() - ultimo_anim >= 600) { ultimo_anim = millis(); fase = (fase + 1) % 4; }
        digitalWrite(GPIO_LED_VERM_S,  fase == 0 ? HIGH : LOW);
        digitalWrite(GPIO_LED_AMARELO, fase == 1 ? HIGH : LOW);
        digitalWrite(GPIO_LED_VERDE,   fase == 2 ? HIGH : LOW);
        return;
    }

    // Sinaleira estática — geometria real do reservatório:
    //   40 cm = cheio (100%) | 70 cm = vazio (0%)
    float n = nivel_atual;
    digitalWrite(GPIO_LED_VERDE,   LOW);
    digitalWrite(GPIO_LED_AMARELO, LOW);
    digitalWrite(GPIO_LED_VERM_S,  LOW);

    if (n <= 0.0f) return;  // sem leitura — todos apagados

    if      (n <= 55.0f) { digitalWrite(GPIO_LED_VERDE,   HIGH); }  // cheio (sensor <= 55cm)
    else if (n <= 65.0f) { digitalWrite(GPIO_LED_AMARELO, HIGH); }  // médio (55cm < sensor <= 65cm)
    else                 { digitalWrite(GPIO_LED_VERM_S,  HIGH); }  // baixo / crítico (sensor > 65cm)
}

// Buzzer de nível baixo — 3 bipes a cada 30 min quando sinaleira vermelha + semiautomático
// Só dispara no horário permitido (09:00–22:00, verificado via buzzer_permitido())
void loop_buzzer_nivel() {
    // Condição: nível crítico (sensor > 65cm), modo semiautomático, horário permitido
    bool nivel_critico = (nivel_atual > 65.0f && nivel_atual > 0.0f && !bomba_ligada);
    if (!nivel_critico || modo_atual != MODO_SEMIAUTOMATICO || !buzzer_permitido()) return;

    static uint32_t ultimo_aviso = 0;
    const uint32_t INTERVALO_AVISO_MS = 1800000UL;  // 30 minutos

    if (millis() - ultimo_aviso >= INTERVALO_AVISO_MS) {
        ultimo_aviso = millis();
        bip(3);
        registrar_log(F("[SYS] Aviso sonoro: nivel critico em modo semiautomatico."));
    }
}

// LED vermelho individual (GPIO_LED_ERRO) — 2026-07-27: aceso enquanto agua_offline
// (silêncio de rádio Água→Bomba) OU qualquer erro da Bomba (bomba_erro_bitmask:
// ERRO_1_TIMEOUT/ERRO_3_LADRAO/ERRO_5_PZEM) OU relé colado (ESTADO_RELE_COLADO em
// bomba_estado_bitmask, marica-149/152) estiver ativo. Único indicador físico
// dedicado a "algo está errado", cobrindo os dois lados do sistema hidráulico
// (decisão de Peter). ERRO_3_LADRAO continua acendendo o LED mesmo reclassificado
// de "erro" pra "flag de estado decorrente de evento" -- reclassificação é só de
// rótulo/texto (ver rota_config()), zero mudança de comportamento ou sinalização.
// Água e as flags locais da própria Controle (ntp_sincronizado/wifi_falhou/
// comando_falhou) ficam FORA de propósito. Apaga quando tudo resolver. Texto
// literal de cada causa fica no dashboard e em rota_config() (/config).
// (silêncio de rádio Água→Bomba, marica-123/125), apagado caso contrário.
// Design deliberadamente simples (decisão de Peter, 2026-07-25): substitui o
// esquema anterior de pulsos por bitmask (erros_ativos), que nunca chegou a
// ser ativado em produção e não cobria agua_offline (que não é um bit do
// bitmask — é um booleano à parte, marica-124). Não pisca, não usa buzzer.
void loop_led_erro() {
    bool erro_ativo = agua_offline
                     || (erros_ativos & (ERRO_1_TIMEOUT | ERRO_3_LADRAO | ERRO_5_PZEM))
                     || (bomba_estado_bitmask & ESTADO_RELE_COLADO);
    digitalWrite(GPIO_LED_ERRO, erro_ativo ? HIGH : LOW);
}

// -------------------------------------------------------------
// AUTOMÁTICO POR PRESENÇA DE CELULAR (2026-07-25)
// -------------------------------------------------------------
// Quando um celular cadastrado está presente na rede (MODO_AUTOMATICO,
// loop_ping_presenca), a bomba liga sozinha ao atingir o nível configurado
// -- sem essa checagem, MODO_AUTOMATICO só acendia o LED branco e não
// tinha nenhum efeito sobre o acionamento da bomba.
//
// Usa o mesmo CMD_LIGA_BOMBA do botão físico/web (ignorar_nivel=false por
// padrão) -- ou seja, passa pelas 5 travas inteiras de ligar_bomba() na
// Bomba (sensor inválido, caixa cheia, segurança, erros físicos, debounce).
// Esta função só decide QUANDO tentar; quem decide se pode ligar continua
// sendo a Bomba, como em qualquer outro acionamento.
//
// Trava de horário (09:00-18:00, horario_bomba_permitido) é EXCLUSIVA deste
// modo automático -- decisão de Peter, 2026-07-25: o acionamento manual
// (botão físico ou botão "Ligar" da web) nunca teve essa trava e continua
// sem ela, pois pode ser necessário encher a caixa fora desse horário.
//
// Cooldown de 5 min entre tentativas: evita reenvio a cada iteração do
// loop() (~5-10ms) caso o comando se perca no ar ou seja recusado por
// alguma trava da Bomba que esta função não enxerga.
void loop_automatico_presenca() {
    static uint32_t ultima_tentativa = 0;
    const uint32_t  COOLDOWN_MS = 300000UL;  // 5 minutos

    if (modo_atual != MODO_AUTOMATICO) return;   // celular ausente — não atua
    if (bomba_ligada)                  return;   // já ligada — nada a fazer
    if (bomba_esta_offline())          return;   // dado da Bomba desatualizado — não decide às cegas
    if (agua_offline || agua_erro_sensor) return; // não confia no nível sem a Água OK
    if (nivel_atual < (float)cfg_liga_cm) return; // ainda não atingiu o limiar
    if (!horario_bomba_permitido())    return;   // fora do horário permitido — trava exclusiva do automático

    if (millis() - ultima_tentativa < COOLDOWN_MS) return;
    ultima_tentativa = millis();

    registrar_log(F("[AUTO] Celular presente + nivel no limiar. Ligando bomba (automatico)."));
    origem_ultimo_comando_liga = ORIGEM_LIGA_AUTOMATICO;  // 2026-07-25
    enviar_comando(CMD_LIGA_BOMBA);
}

// -------------------------------------------------------------
// BOTÕES
// -------------------------------------------------------------
void loop_botoes() {
    static uint32_t btn1_pressionado    = 0;
    static bool     btn1_contando       = false;
    static bool     webserver_gatilhado = false;
    static uint32_t ultimo_btn2         = 0;

    bool btn1_estado = (digitalRead(GPIO_BTN1) == LOW);

    if (btn1_estado && !btn1_contando) {
        btn1_contando       = true;
        btn1_pressionado    = millis();
        webserver_gatilhado = false;
    }

    if (btn1_estado && btn1_contando && !webserver_gatilhado) {
        if (millis() - btn1_pressionado >= BTN1_HOLD_MS) {
            webserver_gatilhado = true;
            if (modo_wifi == WIFI_MODO_NENHUM && !bomba_ligada) {
                registrar_log(F("[CMD] BTN1 Hold 3s: abrindo servidor web."));
                abrir_webserver();
            }
        }
    }

    if (!btn1_estado && btn1_contando) {
        btn1_contando = false;
        uint32_t dur = millis() - btn1_pressionado;
        if (dur >= 50 && !webserver_gatilhado) {
            registrar_log(F("[CMD] BTN1 clique: Ligar bomba."));
            origem_ultimo_comando_liga = ORIGEM_LIGA_BTN1;  // 2026-07-25
            enviar_comando(CMD_LIGA_BOMBA);
            bip(1);
        }
    }

    // BTN2 — emergência: aborta qualquer modo Wi-Fi ativo antes de disparar o rádio
    if (digitalRead(GPIO_BTN2) == LOW && millis() - ultimo_btn2 > 300) {
        ultimo_btn2 = millis();
        registrar_log(F("[CMD] BTN2: EMERGENCIA - Desligar bomba."));
        comando_desliga_pendente = true;
        if (modo_wifi != WIFI_MODO_NENHUM) {
            fechar_webserver();
            delay(100);
        }
        enviar_comando(CMD_DESLIGA_BOMBA);
        bip(1);
    }
}

// -------------------------------------------------------------
// SETUP
// -------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    // Hardware Watchdog
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    pinMode(GPIO_LED_VERDE,   OUTPUT); digitalWrite(GPIO_LED_VERDE,   LOW);
    pinMode(GPIO_LED_AMARELO, OUTPUT); digitalWrite(GPIO_LED_AMARELO, LOW);
    pinMode(GPIO_LED_VERM_S,  OUTPUT); digitalWrite(GPIO_LED_VERM_S,  LOW);
    pinMode(GPIO_LED_AZUL,    OUTPUT); digitalWrite(GPIO_LED_AZUL,    LOW);
    pinMode(GPIO_LED_ERRO,    OUTPUT); digitalWrite(GPIO_LED_ERRO,    LOW);
    pinMode(GPIO_LED_BRANCO,  OUTPUT); digitalWrite(GPIO_LED_BRANCO,  LOW);
    pinMode(GPIO_BUZZER,      OUTPUT); digitalWrite(GPIO_BUZZER,      HIGH);
    pinMode(GPIO_BTN1, INPUT_PULLUP);
    pinMode(GPIO_BTN2, INPUT_PULLUP);

    carregar_params();
    registrar_log(F("=== SISTEMA INICIADO ==="));

    // Boot direto em ESP-NOW — Wi-Fi autorizado só após 5 min
    iniciar_espnow();
    ultimo_ciclo_wifi = 0;  // flag: primeiro ciclo usa BOOT_DELAY_WIFI_MS

    bip(2);

    Serial.println(F("=========================================="));
    Serial.println(F(" CAIXA CONTROLE - PRODUCAO MARICA"));
    Serial.printf( " ESP-NOW Canal: %d\n", CANAL_SEGURANCA_PADRAO);
    Serial.println(F(" BTN1 clique=Liga | BTN1 3s=Web | BTN2=Para"));
    Serial.println(F(" 1a janela Wi-Fi: 1 min apos boot (NTP)"));
    Serial.println(F("=========================================="));
}

// -------------------------------------------------------------
// LOOP
// -------------------------------------------------------------
void loop() {
    esp_task_wdt_reset();

    // Botões — sempre processados primeiro, inclusive durante servidor web
    loop_botoes();

    // Gatilho OTA via serial (bancada)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'o' || c == 'O') {
            registrar_log(F("[CMD] Gatilho OTA recebido via Serial."));
            ota_requisitado = true;
        }
    }

    // OTA — prioridade absoluta
    if (ota_requisitado && !ota_ativo && modo_wifi == WIFI_MODO_NENHUM) {
        iniciar_ota();
    }
    if (ota_ativo) {
        loop_ota();
        return;
    }

    // Servidor web via Cardputer — flag levantada no callback, executada aqui em segurança
    if (webserver_requisitado && !webserver_ativo && modo_wifi == WIFI_MODO_NENHUM) {
        webserver_requisitado = false;
        abrir_webserver();
    }

    // Servidor web ativo
    if (webserver_ativo) {
        servidor.handleClient();
        // MANTIDOS PARA TESTE
        loop_led_branco();
        loop_sinaleira();
        loop_buzzer_nivel();
        loop_led_erro();  // ativado 2026-07-25 — LED vermelho fixo p/ agua_offline
        // DESATIVADOS PARA TESTE — descomentar em produção
        // loop_led_azul();
        // loop_buzzer_erro();
        if (millis() - inicio_wifi >= JANELA_WEBSERVER_MS) {
            Serial.printf("[%s] [WEB] Timeout. Encerrando.\n", obter_hora().c_str());
            fechar_webserver();
        }
        return;
    }

    // Modo normal — ESP-NOW ativo
    loop_ciclo_wifi();
    registrar_telemetria();

    // Keep-alive de presença — envia CMD_PING_CONTROLE para a Bomba a cada 10 min
    // Primeiro disparo aos 5s de uptime — evita race condition de boot com a Bomba
    // que assume modo automático após 8 min sem sinal da Controle
    {
        static uint32_t ultimo_ping_bomba = millis() - 600000UL + 5000UL;
        if (espnow_ok && millis() - ultimo_ping_bomba >= 600000UL) {
            ultimo_ping_bomba = millis();
            PacketComandoBomba ping = {};
            ping.tipo              = CMD_PING_CONTROLE;
            ping.horario_permitido = horario_bomba_permitido();
            esp_now_send(MAC_BOMBA, (uint8_t*)&ping, sizeof(ping));
            Serial.printf("[%s] [CONTROLE] Ping de presenca enviado. Horario:%s\n",
                          obter_hora().c_str(),
                          ping.horario_permitido ? "PERMITIDO" : "BLOQUEADO");
        }
    }

    // Status para o Cardputer (modo Monitor) — a cada 10s, 2026-07-30
    // Intervalo bem mais curto que o keep-alive da Bomba (10min) de propósito:
    // aqui é UI de consumo direto (gráfico sempre visível), não keep-alive de
    // presença. Cardputer detecta silêncio prolongado do lado dele.
    {
        static uint32_t ultimo_status_cardputer = 0;
        if (espnow_ok && millis() - ultimo_status_cardputer >= 10000UL) {
            ultimo_status_cardputer = millis();
            enviar_status_cardputer();
        }
    }
    // MANTIDOS PARA TESTE
    loop_led_branco();
    loop_sinaleira();
    loop_buzzer_nivel();
    loop_led_erro();            // ativado 2026-07-25 — LED vermelho fixo p/ agua_offline
    loop_automatico_presenca(); // ativado 2026-07-25 — liga bomba sozinha se celular presente
    // DESATIVADOS PARA TESTE — descomentar em produção
    // loop_led_azul();
    // loop_buzzer_erro();

    delay(5);
}
