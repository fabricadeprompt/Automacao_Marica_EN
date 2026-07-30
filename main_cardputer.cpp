// =============================================================
// PROJETO AUTOMACAO MARICA - FERRAMENTA CARDPUTER
// Ativador OTA Portátil + Monitor de Nível — dispara CMD_OTA via ESP-NOW
// A gravação do firmware é feita pelo PlatformIO via Wi-Fi
// (upload_protocol = espota) após a caixa entrar em modo OTA.
//
// 2026-07-30 — Firmware pro concurso (Hackster, marica-183): Cardputer ganha
// um segundo papel, fixado por ímã na porta da geladeira, monitor gráfico do
// nível da Caixa Água (modo MONITOR) — além do ativador OTA/Web original
// (modo OPÇÕES, o menu abaixo). Dado vem pronto (PKT_STATUS_CARDPUTER,
// marica_protocol.h) via push periódico da Caixa Controle — o Cardputer não
// reimplementa cálculo de percentual, só exibe o que a Controle já calculou.
//
// [1] → Caixa Água     — IP 192.168.1.92
// [2] → Caixa Bomba    — IP 192.168.1.91
// [3] → Caixa Controle — IP 192.168.1.90 (OTA)
// [4] → Caixa Controle — IP 192.168.1.90 (Servidor Web)
// [5] → Caixa Água     — IP 192.168.1.92 (Reboot)
// [6] → Volta pro modo Monitor (não usa ESP-NOW, é só troca de tela local)
//
// Navegação: ';'(↑) e '.'(↓) movem cursor | Espaço aciona item
// Modo MONITOR: tela sempre ligada, sem timeout — é a tela de repouso.
//   Qualquer tecla entra no modo OPÇÕES (acorda sem executar, como sempre).
// Modo OPÇÕES: tela apaga após 1min de inatividade — qualquer tecla reacende
//   de volta no próprio modo OPÇÕES, onde parou (marica-185; NÃO retorna ao
//   Monitor sozinho — só o item [6] leva de volta).
// ACK via esp_now_send_cb (L2) com timeout de 5s
// CPU: 80MHz
//
// Keycodes confirmados em campo:
//   ↑  = 59 (';')
//   ↓  = 46 ('.')
//   OK = 32 (Espaço)
//   Enter físico não emite keycode — não utilizado
// =============================================================

#include <M5Cardputer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <marica_protocol.h>

// -------------------------------------------------------------
// FORWARD DECLARATIONS
// -------------------------------------------------------------
void iniciar_radio_cardputer();
void desenhar_menu();
void desenhar_aguardando(const char* nome);
void desenhar_ack_ok(const char* nome, const char* ip);
void desenhar_ack_timeout(const char* nome);
void desenhar_tela_apagada();
void acionar_item(int idx);
void atualizar_barra_status();
void cb_send(const uint8_t* mac, esp_now_send_status_t status);
void cb_recepcao(const uint8_t* mac, const uint8_t* dados, int len);
void desenhar_monitor_estatico();
void atualizar_monitor_dinamico();

// -------------------------------------------------------------
// CORES (RGB565)
// -------------------------------------------------------------
#define COR_FUNDO        0x0000
#define COR_BORDA        0x2104
#define COR_TITULO       0x07E8
#define COR_AGUA         0x07FF
#define COR_BOMBA        0xFD20
#define COR_CONTROLE     0xC81F
#define COR_AZUL         0x001F
#define COR_BADGE_AGUA   0x0230
#define COR_BADGE_BOMBA  0x6000
#define COR_BADGE_CTRL   0x2008
#define COR_BADGE_WEB    0x0010
#define COR_BADGE_MON    0x0410
#define COR_MUTED        0x4208
#define COR_OK           0x07E8
#define COR_ERRO         0xF800
#define COR_AMARELO      0xFFE0
#define COR_VERDE_ESCURO 0x0320
#define COR_VERMELHO_ESC 0x3000
#define COR_LARANJA      0xFC00  // reboot — ação destrutiva
#define COR_AGUA_FILL    0x04DF  // água do tanque no modo Monitor (azul mais saturado)
#define COR_TANQUE_BORDA 0x6B6D  // contorno do tanque (cinza-azulado)
#define COR_MODO_AUTO    0x051F  // azul claro -- badge "AUTOMATICO"

// -------------------------------------------------------------
// KEYCODES CONFIRMADOS EM CAMPO
// -------------------------------------------------------------
#define KEY_CIMA  59   // ';'
#define KEY_BAIXO 46   // '.'
#define KEY_OK    32   // Espaço

// -------------------------------------------------------------
// ITENS DO MENU (modo OPÇÕES)
// -------------------------------------------------------------
struct ItemMenu {
    char        tecla;
    const char* label;
    const char* ip;
    const uint8_t* mac;
    uint16_t    cor;
    uint16_t    cor_badge;
    bool        eh_web;
    bool        eh_reboot;
    bool        eh_monitor;  // 2026-07-30 -- item local, não envia nada por ESP-NOW
};

static const ItemMenu MENU[] = {
    { '1', "CAIXA AGUA",     "192.168.1.92", MAC_AGUA,     COR_AGUA,      COR_BADGE_AGUA,  false, false, false },
    { '2', "CAIXA BOMBA",    "192.168.1.91", MAC_BOMBA,    COR_BOMBA,     COR_BADGE_BOMBA, false, false, false },
    { '3', "CTRLE OTA",      "192.168.1.90", MAC_CONTROLE, COR_CONTROLE,  COR_BADGE_CTRL,  false, false, false },
    { '4', "CTRLE WEB",      "192.168.1.90", MAC_CONTROLE, COR_AZUL,      COR_BADGE_WEB,   true,  false, false },
    { '5', "AGUA REBOOT",    "192.168.1.92", MAC_AGUA,     COR_LARANJA,   COR_BADGE_AGUA,  false, true,  false },
    { '6', "VOLTAR MONITOR", "",              nullptr,      COR_AGUA_FILL, COR_BADGE_MON,   false, false, true  },
};
static const int N_ITENS = (int)(sizeof(MENU) / sizeof(MENU[0]));

// -------------------------------------------------------------
// CONSTANTES DE LAYOUT (modo OPÇÕES)
// -------------------------------------------------------------
static const int VISIVEIS      = 3;
static const int ALTURA_HEADER = 18;
static const int ALTURA_STATUS = 20;
static const int ALTURA_LINHA  = (135 - ALTURA_HEADER - ALTURA_STATUS) / VISIVEIS;

// -------------------------------------------------------------
// CONSTANTES DE LAYOUT (modo MONITOR)
// -------------------------------------------------------------
static const int MON_TANQUE_X  = 14;
static const int MON_TANQUE_Y  = 22;
static const int MON_TANQUE_W  = 66;
static const int MON_TANQUE_H  = 88;
static const int MON_N_COLUNAS = 16;                              // colunas da onda
static const int MON_LARG_COL  = MON_TANQUE_W / MON_N_COLUNAS;    // largura de cada coluna
static const float MON_AMPLITUDE_ONDA = 2.2f;                     // px — "leve" de propósito

// -------------------------------------------------------------
// CONSTANTES DE TEMPO
// -------------------------------------------------------------
static const unsigned long TIMEOUT_TELA_MS   = 60000UL;  // 1min -- só no modo OPÇÕES (marica-185/186)
static const unsigned long TIMEOUT_ACK_MS    =  5000UL;
static const unsigned long TEMPO_FEEDBACK_MS =  3000UL;
static const unsigned long MON_ANIM_MS       =   180UL;   // cadência da onda
static const unsigned long MON_STALE_MS      = 30000UL;   // 3x o intervalo de push da Controle (10s)
static const unsigned long MON_ANTIQUEIMA_MS = 600000UL;  // 10min -- micro-deslocamento anti-retenção
static const uint8_t       BRILHO_NORMAL     = 128;
static const uint8_t       BRILHO_MONITOR    = 128;  // marica-186: default provisório, mesmo do modo
                                                       // Opções -- calibração real adiada pra campo

// -------------------------------------------------------------
// ESTADO DA MÁQUINA (modo OPÇÕES)
// -------------------------------------------------------------
enum EstadoTela { MONITOR, MENU_ATIVO, AGUARDANDO_ACK, ACK_OK, ACK_TIMEOUT, TELA_APAGADA };

static EstadoTela    estado            = MONITOR;  // 2026-07-30: boot direto no modo Monitor
static int           cursor_idx        = 0;
static int           scroll_idx        = 0;
static bool          radio_ok          = false;
static volatile bool ack_flag          = false;
static volatile bool ack_recebido      = false;
static int           idx_acionado      = 0;

static unsigned long t_ultimo_toque    = 0;
static unsigned long t_comando_enviado = 0;
static unsigned long t_feedback_inicio = 0;

// -------------------------------------------------------------
// ESTADO DO MODO MONITOR (atualizado via PKT_STATUS_CARDPUTER)
// -------------------------------------------------------------
// 2026-07-30, achado da revisão Gemini (marica-190): os 7 campos abaixo vêm
// de um único pacote e precisam chegar ao loop() como um conjunto
// consistente -- nunca uma mistura de campo novo com campo velho, o que
// aconteceria se cb_recepcao() (task ESP-NOW, possivelmente outro núcleo)
// escrevesse os 7 e o loop() lesse os 7 sem nenhuma seção crítica entre eles.
// volatile sozinho não resolve isso (evita só cache de registrador por
// campo, não consistência entre campos) -- daí o portMUX_TYPE: toda escrita
// em cb_recepcao() e toda leitura em atualizar_monitor_dinamico() usam o
// mesmo mon_mux, então nunca existe uma leitura no meio de uma escrita.
static portMUX_TYPE mon_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t  mon_nivel_pct          = 0;
static volatile float    mon_nivel_distancia_cm = 0.0f;
static volatile bool     mon_bomba_ligada       = false;
static volatile uint8_t  mon_modo_atual         = 0;
static volatile bool     mon_agua_erro_sensor   = false;
static volatile bool     mon_agua_offline       = false;
static volatile bool     mon_bomba_offline      = false;
static volatile uint32_t mon_ultimo_pacote_ms   = 0;  // 0 == nunca recebeu nenhum pacote

static unsigned long mon_t_ultima_animacao   = 0;
static unsigned long mon_t_ultimo_antiqueima = 0;
static float         mon_fase_onda           = 0.0f;
static int           mon_deslocamento_antiqueima = 0;  // 0..3, ciclo de micro-shift
static bool           mon_estatico_desenhado     = false;  // força redesenho do "chrome" na 1a
                                                             // vez ou depois de um shift anti-retenção

// -------------------------------------------------------------
// SETUP
// -------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(BRILHO_MONITOR);

    t_ultimo_toque = millis();
    iniciar_radio_cardputer();
    desenhar_monitor_estatico();  // boot já entra direto no modo Monitor
}

// -------------------------------------------------------------
// LOOP
// -------------------------------------------------------------
void loop() {
    M5Cardputer.update();
    unsigned long agora = millis();

    // --- Leitura de tecla ---
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {

        // Modo Monitor: qualquer tecla entra no modo Opções, sem executar nada
        // (mesmo espírito do "acorda sem acionar" que já existia na tela apagada)
        if (estado == MONITOR) {
            estado = MENU_ATIVO;
            M5Cardputer.Display.setBrightness(BRILHO_NORMAL);
            t_ultimo_toque = agora;
            desenhar_menu();
            atualizar_barra_status();
            delay(30);
            return;
        }

        // Tela apagada (só ocorre vindo do modo Opções): qualquer tecla reacende
        // de volta no próprio modo Opções, sem executar comando (marica-185)
        if (estado == TELA_APAGADA) {
            estado = MENU_ATIVO;
            M5Cardputer.Display.setBrightness(BRILHO_NORMAL);
            t_ultimo_toque = agora;
            desenhar_menu();
            atualizar_barra_status();
            delay(30);
            return;
        }

        t_ultimo_toque = agora;

        // Estados de feedback: qualquer tecla volta ao menu imediatamente
        if (estado == ACK_OK || estado == ACK_TIMEOUT) {
            estado = MENU_ATIVO;
            desenhar_menu();
            atualizar_barra_status();
            delay(30);
            return;
        }

        // Aguardando ACK: ignora teclado
        if (estado == AGUARDANDO_ACK) {
            return;
        }

        // Menu ativo: navegação e acionamento
        if (estado == MENU_ATIVO) {

            // Navegação ↑
            if (M5Cardputer.Keyboard.isKeyPressed(KEY_CIMA)) {
                if (cursor_idx > 0) {
                    cursor_idx--;
                    if (cursor_idx < scroll_idx) scroll_idx = cursor_idx;
                    desenhar_menu();
                    atualizar_barra_status();
                }
            }
            // Navegação ↓
            else if (M5Cardputer.Keyboard.isKeyPressed(KEY_BAIXO)) {
                if (cursor_idx < N_ITENS - 1) {
                    cursor_idx++;
                    if (cursor_idx >= scroll_idx + VISIVEIS) scroll_idx = cursor_idx - VISIVEIS + 1;
                    desenhar_menu();
                    atualizar_barra_status();
                }
            }
            // Espaço aciona item sob o cursor
            else if (M5Cardputer.Keyboard.isKeyPressed(KEY_OK)) {
                acionar_item(cursor_idx);
            }
        }
    }

    // --- Máquina de estados assíncrona ---

    // Apagar tela por inatividade -- só no modo Opções (marica-185/186:
    // Monitor nunca apaga sozinho, é a tela de repouso do dispositivo)
    if (estado == MENU_ATIVO && (agora - t_ultimo_toque >= TIMEOUT_TELA_MS)) {
        estado = TELA_APAGADA;
        desenhar_tela_apagada();
        return;
    }

    // Verificar ACK ou timeout de rádio
    if (estado == AGUARDANDO_ACK) {
        if (ack_flag) {
            ack_flag = false;
            t_feedback_inicio = agora;
            if (ack_recebido) {
                estado = ACK_OK;
                desenhar_ack_ok(MENU[idx_acionado].label, MENU[idx_acionado].ip);
            } else {
                estado = ACK_TIMEOUT;
                desenhar_ack_timeout(MENU[idx_acionado].label);
            }
            t_ultimo_toque = agora;
        } else if (agora - t_comando_enviado >= TIMEOUT_ACK_MS) {
            ack_flag = false;
            estado = ACK_TIMEOUT;
            t_feedback_inicio = agora;
            desenhar_ack_timeout(MENU[idx_acionado].label);
            t_ultimo_toque = agora;
        }
    }

    // Retorno automático ao menu após feedback
    if ((estado == ACK_OK || estado == ACK_TIMEOUT) &&
        (agora - t_feedback_inicio >= TEMPO_FEEDBACK_MS)) {
        estado = MENU_ATIVO;
        t_ultimo_toque = agora;
        desenhar_menu();
        atualizar_barra_status();
    }

    // Animação do modo Monitor -- roda independente de tecla, é a tela de repouso
    if (estado == MONITOR) {
        if (!mon_estatico_desenhado) {
            desenhar_monitor_estatico();
        }
        if (agora - mon_t_ultima_animacao >= MON_ANIM_MS) {
            mon_t_ultima_animacao = agora;
            atualizar_monitor_dinamico();
        }
        // Anti-retenção: desloca levemente o texto estático a cada ~10min
        // (a onda em si já cobre o tanque -- isso cobre os rótulos fixos)
        if (agora - mon_t_ultimo_antiqueima >= MON_ANTIQUEIMA_MS) {
            mon_t_ultimo_antiqueima = agora;
            mon_deslocamento_antiqueima = (mon_deslocamento_antiqueima + 1) % 4;
            mon_estatico_desenhado = false;  // força redesenho do chrome na próxima volta do loop
        }
    }

    delay(20);
}

// -------------------------------------------------------------
// ACIONAR ITEM
// -------------------------------------------------------------
void acionar_item(int idx) {
    if (idx < 0 || idx >= N_ITENS) return;

    idx_acionado = idx;
    const ItemMenu& item = MENU[idx_acionado];

    // Item local (Voltar Monitor) -- não usa ESP-NOW, não espera ACK
    if (item.eh_monitor) {
        estado = MONITOR;
        mon_estatico_desenhado = false;  // redesenha o chrome do zero ao entrar
        M5Cardputer.Display.setBrightness(BRILHO_MONITOR);
        t_ultimo_toque = millis();
        return;
    }

    estado = AGUARDANDO_ACK;
    ack_flag     = false;
    ack_recebido = false;
    t_comando_enviado = millis();

    desenhar_aguardando(item.label);

    if (item.eh_web) {
        PacketComandoWebServer cmd = {};
        cmd.tipo = CMD_WEB_SERVER;
        esp_now_send(item.mac, (uint8_t*)&cmd, sizeof(cmd));
    } else if (item.eh_reboot) {
        PacketComandoReboot cmd = {};
        cmd.tipo = CMD_REBOOT;
        esp_now_send(item.mac, (uint8_t*)&cmd, sizeof(cmd));
    } else {
        PacketComandoOTA cmd = {};
        cmd.tipo = CMD_OTA;
        esp_now_send(item.mac, (uint8_t*)&cmd, sizeof(cmd));
    }
}

// -------------------------------------------------------------
// CALLBACK SEND — L2 (apenas setar flags, nunca bloquear)
// -------------------------------------------------------------
void cb_send(const uint8_t* mac, esp_now_send_status_t status) {
    ack_recebido = (status == ESP_NOW_SEND_SUCCESS);
    ack_flag     = true;
}

// -------------------------------------------------------------
// CALLBACK RECEPÇÃO — 2026-07-30 (status pro modo Monitor)
// -------------------------------------------------------------
// Só trata PKT_STATUS_CARDPUTER; qualquer outro tipo é ignorado (não deveria
// chegar nenhum, mas não custa checar tipo+tamanho antes do memcpy).
void cb_recepcao(const uint8_t* mac, const uint8_t* dados, int len) {
    if (len < 1) return;
    uint8_t tipo = dados[0];

    if (tipo == PKT_STATUS_CARDPUTER && len >= (int)sizeof(PacketStatusCardputer)) {
        PacketStatusCardputer pkt;
        memcpy(&pkt, dados, sizeof(pkt));

        portENTER_CRITICAL(&mon_mux);
        mon_nivel_pct          = pkt.nivel_pct;
        mon_nivel_distancia_cm = pkt.nivel_distancia_cm;
        mon_bomba_ligada       = pkt.bomba_ligada;
        mon_modo_atual         = pkt.modo_atual;
        mon_agua_erro_sensor   = pkt.agua_erro_sensor;
        mon_agua_offline       = pkt.agua_offline;
        mon_bomba_offline      = pkt.bomba_offline;
        mon_ultimo_pacote_ms   = millis();
        portEXIT_CRITICAL(&mon_mux);
    }
}

// -------------------------------------------------------------
// CONFIGURAÇÃO DE RÁDIO — BASE GEMINI (NÃO MODIFICAR)
// -------------------------------------------------------------
void iniciar_radio_cardputer() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL_SEGURANCA_PADRAO, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        radio_ok = false;
        atualizar_barra_status();
        return;
    }

    esp_now_register_send_cb(cb_send);
    esp_now_register_recv_cb(cb_recepcao);  // 2026-07-30 -- necessário pro modo Monitor

    esp_now_peer_info_t peer = {};
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_del_peer(MAC_AGUA);
    memcpy(peer.peer_addr, MAC_AGUA, 6);
    esp_now_add_peer(&peer);

    esp_now_del_peer(MAC_BOMBA);
    memcpy(peer.peer_addr, MAC_BOMBA, 6);
    esp_now_add_peer(&peer);

    esp_now_del_peer(MAC_CONTROLE);
    memcpy(peer.peer_addr, MAC_CONTROLE, 6);
    esp_now_add_peer(&peer);

    radio_ok = true;
    atualizar_barra_status();
}

// -------------------------------------------------------------
// DESENHO — BARRA DE STATUS (modo OPÇÕES)
// -------------------------------------------------------------
void atualizar_barra_status() {
    auto& d = M5Cardputer.Display;
    int y = 135 - ALTURA_STATUS;

    d.fillRect(0, y, 240, ALTURA_STATUS, COR_FUNDO);
    d.drawFastHLine(0, y, 240, COR_BORDA);
    d.setTextSize(1);

    // Canal
    d.setTextColor(COR_MUTED);
    d.setCursor(4, y + 6);
    d.printf("CH%d", CANAL_SEGURANCA_PADRAO);

    // Indicadores de scroll
    bool tem_cima  = (scroll_idx > 0);
    bool tem_baixo = (scroll_idx + VISIVEIS < N_ITENS);
    if (tem_cima || tem_baixo) {
        d.setCursor(34, y + 6);
        d.setTextColor(COR_MUTED);
        if (tem_cima && tem_baixo) d.print("^ v");
        else if (tem_cima)         d.print("^");
        else                       d.print("v");
    }

    // Tecla de acionamento
    d.setTextColor(COR_MUTED);
    d.setCursor(70, y + 6);
    d.print("[SPACE]=OK");

    // Status do rádio
    if (radio_ok) {
        d.setTextColor(COR_OK);
        d.setCursor(168, y + 6);
        d.print("Radio OK");
    } else {
        d.setTextColor(COR_ERRO);
        d.setCursor(153, y + 6);
        d.print("Radio ERRO");
    }
}

// -------------------------------------------------------------
// DESENHO — MENU PRINCIPAL (modo OPÇÕES)
// -------------------------------------------------------------
void desenhar_menu() {
    auto& d = M5Cardputer.Display;
    d.fillScreen(COR_FUNDO);

    // Header
    d.setTextSize(1);
    d.setTextColor(COR_TITULO);
    d.setCursor(4, 5);
    d.print("MARICA  OTA / WEB");
    d.drawFastHLine(0, ALTURA_HEADER, 240, COR_BORDA);

    int y_base = ALTURA_HEADER + 2;

    for (int i = 0; i < VISIVEIS; i++) {
        int idx = scroll_idx + i;
        if (idx >= N_ITENS) break;

        const ItemMenu& item  = MENU[idx];
        int  y           = y_base + i * ALTURA_LINHA;
        bool selecionado = (idx == cursor_idx);

        // Fundo destacado no item sob o cursor
        if (selecionado) {
            d.fillRect(0, y, 240, ALTURA_LINHA - 2, 0x1082);
        }

        // Badge com tecla
        d.fillRoundRect(3, y + 4, 20, 14, 2, item.cor_badge);
        d.setTextSize(1);
        d.setTextColor(item.cor);
        d.setCursor(5, y + 7);
        d.printf("[%c]", item.tecla);

        // Label em fonte 2 (12x16px)
        d.setTextSize(2);
        d.setTextColor(selecionado ? 0xFFFF : 0xC618);
        d.setCursor(28, y + 3);
        d.print(item.label);

        // IP em fonte 1 (item local "Voltar Monitor" não tem IP -- fica em branco)
        d.setTextSize(1);
        d.setTextColor(COR_MUTED);
        d.setCursor(28, y + 21);
        d.print(item.ip);

        // Separador inferior
        if (i < VISIVEIS - 1) {
            d.drawFastHLine(0, y + ALTURA_LINHA - 2, 240, COR_BORDA);
        }
    }
}

// -------------------------------------------------------------
// DESENHO — TELA APAGADA (backlight desligado, só modo OPÇÕES)
// -------------------------------------------------------------
void desenhar_tela_apagada() {
    M5Cardputer.Display.fillScreen(COR_FUNDO);
    M5Cardputer.Display.setBrightness(0);
}

// -------------------------------------------------------------
// DESENHO — AGUARDANDO ACK
// -------------------------------------------------------------
void desenhar_aguardando(const char* nome) {
    auto& d = M5Cardputer.Display;
    d.fillScreen(COR_FUNDO);

    d.setTextSize(1);
    d.setTextColor(COR_MUTED);
    d.setCursor(4, 5);
    d.print("ENVIANDO COMANDO...");
    d.drawFastHLine(0, 16, 240, COR_BORDA);

    d.setTextSize(2);
    d.setTextColor(COR_AMARELO);
    d.setCursor(8, 38);
    d.print(nome);

    d.setTextSize(1);
    d.setTextColor(COR_MUTED);
    d.setCursor(8, 68);
    d.print("Aguardando confirmacao...");
    d.setCursor(8, 82);
    d.printf("Timeout: %ds", (int)(TIMEOUT_ACK_MS / 1000));
}

// -------------------------------------------------------------
// DESENHO — ACK OK
// -------------------------------------------------------------
void desenhar_ack_ok(const char* nome, const char* ip) {
    auto& d = M5Cardputer.Display;
    d.fillScreen(COR_FUNDO);

    d.fillRect(4, 18, 232, 92, COR_VERDE_ESCURO);
    d.drawRect(4, 18, 232, 92, COR_OK);

    d.setTextSize(1);
    d.setTextColor(COR_OK);
    d.setCursor(12, 26);
    d.print("[OK] COMANDO RECEBIDO");

    d.setTextSize(2);
    d.setTextColor(0xFFFF);
    d.setCursor(12, 42);
    d.print(nome);

    d.setTextSize(1);
    d.setTextColor(COR_MUTED);
    d.setCursor(12, 70);
    d.print("Confirmado na camada L2");
    d.setCursor(12, 82);
    d.printf("PlatformIO: %s", ip);

    d.setTextColor(COR_MUTED);
    d.setCursor(4, 120);
    d.print("Voltando ao menu em 3s...");
}

// -------------------------------------------------------------
// DESENHO — ACK TIMEOUT
// -------------------------------------------------------------
void desenhar_ack_timeout(const char* nome) {
    auto& d = M5Cardputer.Display;
    d.fillScreen(COR_FUNDO);

    d.fillRect(4, 18, 232, 92, COR_VERMELHO_ESC);
    d.drawRect(4, 18, 232, 92, COR_ERRO);

    d.setTextSize(1);
    d.setTextColor(COR_ERRO);
    d.setCursor(12, 26);
    d.print("TIMEOUT - SEM RESPOSTA");

    d.setTextSize(2);
    d.setTextColor(0xC618);
    d.setCursor(12, 42);
    d.print(nome);

    d.setTextSize(1);
    d.setTextColor(COR_MUTED);
    d.setCursor(12, 70);
    d.print("Caixa nao respondeu.");
    d.setCursor(12, 82);
    d.print("Verifique alimentacao.");

    d.setTextColor(COR_MUTED);
    d.setCursor(4, 120);
    d.print("Voltando ao menu em 3s...");
}

// -------------------------------------------------------------
// DESENHO — MODO MONITOR: "CHROME" ESTÁTICO
// -------------------------------------------------------------
// Desenhado uma vez ao entrar no Monitor, e de novo a cada ciclo de
// anti-retenção (mon_deslocamento_antiqueima muda) -- por isso lê o
// deslocamento atual em vez de assumir posição fixa.
void desenhar_monitor_estatico() {
    auto& d = M5Cardputer.Display;
    d.fillScreen(COR_FUNDO);

    int dx = mon_deslocamento_antiqueima;  // 0..3 px -- imperceptível, mas nunca 2 vezes seguidas igual
    int dy = (mon_deslocamento_antiqueima % 2);

    // Header
    d.setTextSize(1);
    d.setTextColor(COR_TITULO);
    d.setCursor(4 + dx, 5 + dy);
    d.print("MARICA  CAIXA D'AGUA");
    d.drawFastHLine(0, ALTURA_HEADER, 240, COR_BORDA);

    // Contorno do tanque -- posição fixa, nunca desloca (âncora visual)
    d.drawRoundRect(MON_TANQUE_X, MON_TANQUE_Y, MON_TANQUE_W, MON_TANQUE_H, 3, COR_TANQUE_BORDA);

    // Escala 0% / 50% / 100% ao lado do tanque, com tick conectando ao tanque
    int x_escala       = MON_TANQUE_X + MON_TANQUE_W + 8 + dx;
    int x_tanque_borda = MON_TANQUE_X + MON_TANQUE_W;
    d.setTextSize(1);
    d.setTextColor(COR_MUTED);

    int y_100 = MON_TANQUE_Y + dy;
    d.drawFastHLine(x_tanque_borda, y_100 + 3, 6, COR_TANQUE_BORDA);
    d.setCursor(x_escala, y_100);
    d.print("100%");

    int y_50 = MON_TANQUE_Y + MON_TANQUE_H / 2 - 4 + dy;
    d.drawFastHLine(x_tanque_borda, y_50 + 3, 6, COR_TANQUE_BORDA);
    d.setCursor(x_escala, y_50);
    d.print("50%");

    int y_0 = MON_TANQUE_Y + MON_TANQUE_H - 8 + dy;
    d.drawFastHLine(x_tanque_borda, y_0 + 3, 6, COR_TANQUE_BORDA);
    d.setCursor(x_escala, y_0);
    d.print("0%");

    // Rodapé -- área reservada pro número grande de % e pros badges de modo/bomba
    // é preenchida por atualizar_monitor_dinamico(), não aqui
    d.drawFastHLine(0, 135 - 22, 240, COR_BORDA);
    d.setTextSize(1);
    d.setTextColor(COR_MUTED);
    d.setCursor(4 + dx, 135 - 15);
    d.print("Toque p/ opcoes OTA/Web");
    d.setCursor(168, 135 - 15);
    d.printf("CH%d", CANAL_SEGURANCA_PADRAO);

    mon_estatico_desenhado = true;
    mon_t_ultima_animacao  = 0;  // força a parte dinâmica a redesenhar tudo por cima do chrome novo
}

// -------------------------------------------------------------
// DESENHO — MODO MONITOR: PARTE DINÂMICA (tanque + textos que mudam)
// -------------------------------------------------------------
// Chamada a cada MON_ANIM_MS (~180ms). Redesenha só a área do tanque (onda)
// e o bloco de texto dinâmico (%, modo, bomba) -- nunca fillScreen() aqui,
// evita flicker e economiza energia (tela sempre ligada no modo Monitor).
void atualizar_monitor_dinamico() {
    auto& d = M5Cardputer.Display;

    // Snapshot atômico dos 7 campos -- mesma seção crítica de cb_recepcao()
    // (marica-190/191, revisão Gemini). A partir daqui a função só usa as
    // cópias locais, nunca os globais mon_* diretamente -- garante que todo
    // o frame é desenhado com um conjunto consistente de valores, mesmo que
    // um pacote novo chegue no meio do desenho.
    uint8_t  nivel_pct;
    float    nivel_distancia_cm;
    bool     bomba_ligada;
    uint8_t  modo_atual;
    bool     agua_erro_sensor;
    bool     agua_offline;
    bool     bomba_offline;
    uint32_t ultimo_pacote_ms;

    portENTER_CRITICAL(&mon_mux);
    nivel_pct          = mon_nivel_pct;
    nivel_distancia_cm = mon_nivel_distancia_cm;
    bomba_ligada        = mon_bomba_ligada;
    modo_atual          = mon_modo_atual;
    agua_erro_sensor    = mon_agua_erro_sensor;
    agua_offline        = mon_agua_offline;
    bomba_offline       = mon_bomba_offline;
    ultimo_pacote_ms    = mon_ultimo_pacote_ms;
    portEXIT_CRITICAL(&mon_mux);

    unsigned long agora = millis();
    bool nunca_recebeu  = (ultimo_pacote_ms == 0);
    bool desatualizado  = nunca_recebeu || (agora - ultimo_pacote_ms > MON_STALE_MS);
    bool dado_confiavel = !desatualizado && !agua_erro_sensor && !agua_offline && !bomba_offline;

    int dx = mon_deslocamento_antiqueima;

    // --- Interior do tanque ---
    int ix = MON_TANQUE_X + 2, iy = MON_TANQUE_Y + 2;
    int iw = MON_TANQUE_W - 4, ih = MON_TANQUE_H - 4;
    d.fillRect(ix, iy, iw, ih, COR_FUNDO);  // limpa o quadro anterior

    if (!dado_confiavel) {
        // Sem dado confiável: tanque cinza vazio + aviso, nunca número congelado
        d.drawFastHLine(ix, iy + ih - 1, iw, COR_MUTED);
        d.setTextSize(1);
        d.setTextColor(COR_ERRO);
        d.setCursor(MON_TANQUE_X + MON_TANQUE_W + 8 + dx, MON_TANQUE_Y + MON_TANQUE_H / 2 + 12);
        d.print(nunca_recebeu ? "AGUARD." : (desatualizado ? "SEM SINAL" :
                (bomba_offline ? "BOMBA OFF" : "SENSOR ERR")));
    } else {
        int altura_px = map((int)nivel_pct, 0, 100, 0, ih);
        int y_base    = iy + ih - altura_px;

        for (int i = 0; i < MON_N_COLUNAS; i++) {
            int x = ix + i * MON_LARG_COL;
            float onda    = MON_AMPLITUDE_ONDA * sinf((i * 0.9f) + mon_fase_onda);
            int   y_topo  = y_base - (int)onda;
            if (y_topo < iy)      y_topo = iy;
            if (y_topo > iy + ih) y_topo = iy + ih;
            int altura_col = (iy + ih) - y_topo;
            if (altura_col > 0) {
                d.fillRect(x, y_topo, MON_LARG_COL, altura_col, COR_AGUA_FILL);
            }
        }
        mon_fase_onda += 0.15f;
        if (mon_fase_onda > 6.2832f) mon_fase_onda -= 6.2832f;  // wrap em 2*PI, evita overflow lento
    }

    // --- Bloco de texto dinâmico (número grande, distância, modo, bomba) ---
    // 2026-07-30, achado da revisão Gemini (marica-191): a altura de limpeza
    // precisa cobrir as DUAS linhas (número % em fonte 3 + "xx cm" em fonte 1
    // logo abaixo) por inteiro, senão dígito mais estreito (ex: "9cm" depois
    // de "100cm") deixa resíduo do dígito antigo na borda inferior. 44px
    // cobre as duas linhas com folga e ainda emenda exatamente onde começa o
    // bloco de badges (y_badges), sem sobra nem buraco entre os dois fillRect.
    int x_txt = MON_TANQUE_X + MON_TANQUE_W + 40 + dx;
    int y_num = MON_TANQUE_Y + 4 + (dx % 2);
    d.fillRect(x_txt - 4, y_num - 2, 240 - (x_txt - 4) - 2, 44, COR_FUNDO);  // limpa área do número

    d.setTextSize(3);
    if (dado_confiavel) {
        d.setTextColor(nivel_pct <= 15 ? COR_ERRO : (nivel_pct <= 35 ? COR_AMARELO : COR_OK));
        d.setCursor(x_txt, y_num);
        d.printf("%u%%", (unsigned)nivel_pct);
    } else {
        d.setTextColor(COR_MUTED);
        d.setCursor(x_txt, y_num);
        d.print("--%");
    }

    if (dado_confiavel) {
        d.setTextSize(1);
        d.setTextColor(COR_MUTED);
        d.setCursor(x_txt, y_num + 30);
        d.printf("%d cm", (int)nivel_distancia_cm);
    }

    // Badges modo + bomba -- área abaixo do número
    int y_badges = MON_TANQUE_Y + 46;
    d.fillRect(x_txt - 4, y_badges, 240 - (x_txt - 4) - 2, 36, COR_FUNDO);

    d.setTextSize(1);
    d.setCursor(x_txt, y_badges);
    if (dado_confiavel) {
        bool automatico = (modo_atual == 1);  // MODO_AUTOMATICO == 1 (marica_protocol.h)
        d.setTextColor(automatico ? COR_MODO_AUTO : COR_AMARELO);
        d.print(automatico ? "AUTOMATICO" : "SEMIAUTOM.");
    } else {
        d.setTextColor(COR_MUTED);
        d.print("MODO ?");
    }

    d.setCursor(x_txt, y_badges + 14);
    if (dado_confiavel) {
        d.setTextColor(bomba_ligada ? COR_OK : COR_MUTED);
        d.print(bomba_ligada ? "BOMBA LIGADA" : "BOMBA DESLIGADA");
    } else {
        d.setTextColor(COR_MUTED);
        d.print("BOMBA ?");
    }
}
