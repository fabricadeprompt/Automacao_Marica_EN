// =============================================================
// PROJETO AUTOMACAO MARICA - CAIXA BOMBA
// Hub Central — Protocolo v2
// Base de rádio: Gemini (promiscuous → set_channel → esp_now_init)
//
// Fluxo de dados:
//   Recebe: PacketTelemetriaAgua (0x20) da Água
//   Envia:  PacketStatusCompleto (0x35) para a Controle
//   Recebe: PacketComandoBomba (0xA1/0xA2) da Controle
//   Recebe: PacketComandoOTA (0xB0) do Cardputer
//
// OTA via serial (bancada): envie 'o' no Monitor Serial
// OTA em campo: Cardputer envia CMD_OTA (0xB0) via ESP-NOW
//
// Alterações 2026-07-25:
//   - Repassa agua_motivo_status (novo campo de PacketTelemetriaAgua) sem
//     alteração para PacketStatusCompleto -- puramente informativo p/ dashboard.
//   - Novo campo bomba_estado_bitmask em PacketStatusCompleto, expõe
//     em_quarentena/em_bloqueio/modo_forcado (já existiam, nunca eram
//     transmitidos). Campo SEPARADO de bomba_erro_bitmask de propósito --
//     nunca lido de volta por nenhuma decisão de segurança da Bomba.
//
// Alterações 2026-07-27 (marica-149/150/152, revisado pela Gemini):
//   - Auto-teste de relé colado: após cada desligamento legítimo, isola K1 e
//     K2 (um de cada vez) e checa via PZEM se a potência fica ~0. Falha só
//     sinaliza (ESTADO_RELE_COLADO em bomba_estado_bitmask), nunca bloqueia.
//     Roda só no loop() (nunca em cb_recepcao) -- leituras PZEM bloqueiam
//     até ~500ms cada. Pré-checagem cobre os dois relés já colados antes do
//     teste; NÃO cobre colagem que ocorra durante a própria Fase 1 (limitação
//     física da topologia em série, sem mitigação possível em firmware).
//   - pzem_ler() mudou de assinatura: agora retorna por referência em vez de
//     escrever direto nas globais pzem_tensao_v/etc -- evita que uma leitura
//     do auto-teste vaze como telemetria real se um pacote ESP-NOW chegar
//     (task separada) no meio do teste e disparar enviar_status().
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <HardwareSerial.h>
#include <marica_protocol.h>

// -------------------------------------------------------------
// MAC DESTA PLACA
// -------------------------------------------------------------
static const uint8_t MAC_PROPRIA[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x02}; // TODO: MAC desta placa (deve bater com MAC_BOMBA em secrets/protocol)

// -------------------------------------------------------------
// PINOS
// -------------------------------------------------------------
#define GPIO_K1        18  // Relé K1 físico (novo) — dual-relay safety switching, marica-116/118/122
#define GPIO_K2        19  // Relé K2 físico (existente) — em série com K1 na bobina da contatora
#define GPIO_PZEM_RX   16  // ESP32 RX ← TX do PZEM-004T (via divisor 10k/20k, 5V→3,3V)
#define GPIO_PZEM_TX   17  // ESP32 TX → RX do PZEM-004T (ligação direta, sem divisor)
// Sensor do ladrão: GPIO 18 da Caixa Água (XKC-Y26S-V) — não há GPIO local na Bomba

#define RELE_LIGA    LOW
#define RELE_DESLIGA HIGH

// -------------------------------------------------------------
// PARÂMETROS OPERACIONAIS
// -------------------------------------------------------------
#define QUARENTENA_MS         300000UL  // 5 min após boot
// TEMPO_MAX_OPERACAO_MS removido — substituído por timeout_operacao_minutos (dinâmico via NVS)
#define TEMPO_BLOQUEIO_MS      900000UL // 15 min após erro de timeout
#define SILENCIO_AGUA_MS        60000UL // 60s sem pacote da Água → desliga
#define SILENCIO_CONTROLE_MS     1200000UL // 20 min sem pacote da Controle → assume automático
#define SILENCIO_CONTROLE_BOOT_MS  480000UL // 8 min de carência no boot (quarentena 5min + 3min margem)
#define DISPAROS_MODO_FORCADO       2   // disparos de timeout → modo forçado
#define OTA_TIMEOUT_MS        600000UL  // 10 min janela OTA
#define WDT_TIMEOUT_S              30
#define PZEM_BAUD              9600
#define PZEM_ADDR              0xF8     // endereço geral — único PZEM no barramento TTL (não usa 0x01)
#define PZEM_INTERVALO_MS      5000UL   // coleta PZEM a cada 5s
#define PZEM_FALHAS_MAX            5    // falhas consecutivas → ERRO_5_PZEM

// Auto-teste de relé colado — 2026-07-27 (marica-149/150)
#define AUTOTESTE_PZEM_MARGEM_W    10.0f  // acima do ruído do PZEM em repouso (tipicamente
                                            // ~0W a vazio), bem abaixo do limiar de 100W usado
                                            // pela Controle para detecção de sessão manual —
                                            // valor inicial, ajustar com leitura real de campo
// 2026-08-07 -- 300ms -> 1500ms (achado de revisão Gemini, marica-221/222, validado
// de forma independente contra relatos de terceiros sobre a lib do PZEM-004T): o chip
// tem taxa de atualização interna de ~1Hz -- ler antes disso corre dois riscos, não só
// um. Com 300ms: (a) a pré-checagem podia ainda refletir a carga anterior ao desligamento
// (falso positivo, caso observado em campo); (b) mais grave, as Fases 1/2 podiam ler ~0W
// mesmo com o outro relé genuinamente colado puxando corrente de verdade, porque o PZEM
// não teve tempo de integrar a nova carga -- falso negativo, o auto-teste reportando "OK"
// pra um relé realmente soldado. 1500ms garante margem sobre o ciclo de 1s do chip.
#define AUTOTESTE_ESTABILIZA_MS    1500UL  // espera antes de consultar o PZEM após comutar o relé

// Valores padrão dos níveis — usados apenas se NVS estiver vazio
#define NIVEL_LIGA_CM_PADRAO        80
#define NIVEL_DESLIGA_CM_PADRAO     20
#define NIVEL_SEGURANCA_CM_PADRAO   85  // limite superior — leitura acima = sensor inválido
#define NIVEL_MANUAL_MIN_CM_PADRAO  50  // nível mínimo para partida manual e automática

#define PZEM_SERIAL_PORT Serial2

// -------------------------------------------------------------
// ESTADO DO SISTEMA
// -------------------------------------------------------------
static float    nivel_atual_cm      = 0.0f;
static bool     agua_erro_hardware  = false;  // erro físico do sensor (vindo do pacote da Água)
static uint8_t  agua_motivo_status  = 0;      // MotivoStatusAgua repassado -- 2026-07-25,
                                               // puramente informativo (dashboard)
static uint8_t  ultima_causa_desligamento = CAUSA_DESLIGA_MANUAL;  // 2026-07-25 -- causa
                                               // do último desligar_bomba(), p/ dashboard
static bool     agua_erro_sensor    = false;  // erro real reportado pela própria Água (condensação/hardware)
static bool     agua_offline        = false;  // silêncio de rádio Água→Bomba >60s (SILENCIO_AGUA_MS) --
                                               // sinal distinto de agua_erro_sensor (marica-123/125)
static bool     ladrao_ativo        = false;  // estado do ladrão repassado pela Água
static bool     bomba_ligada       = false;
static bool     em_quarentena      = true;
static uint32_t inicio_quarentena  = 0;
static uint32_t inicio_operacao    = 0;
static bool     em_bloqueio        = false;
static uint32_t inicio_bloqueio    = 0;
static uint8_t  erros_ativos       = 0;
static uint32_t ultimo_pacote_agua     = 0;
static uint32_t ultimo_pacote_controle = 0;
static uint8_t  contador_disparos  = 0;
static bool     modo_forcado       = false;
static bool     modo_automatico    = false;

// Horário de operação automática — atualizado via PacketComandoBomba da Controle
// false por padrão: sem NTP confirmado, partidas automáticas bloqueadas
static bool     horario_permitido  = false;

// Níveis operacionais e timeout — carregados da NVS no boot
static uint8_t  nivel_liga_cm            = NIVEL_LIGA_CM_PADRAO;
static uint8_t  nivel_desliga_cm         = NIVEL_DESLIGA_CM_PADRAO;
static uint8_t  nivel_seguranca_cm       = NIVEL_SEGURANCA_CM_PADRAO;
static uint8_t  nivel_manual_min_cm      = NIVEL_MANUAL_MIN_CM_PADRAO;
static uint16_t timeout_operacao_minutos = 60;

// Proteção de nível zero — desliga bomba se zero persistir 2 min com bomba ligada
#define NIVEL_ZERO_TIMEOUT_MS  120000UL  // 2 minutos
static uint32_t nivel_zero_inicio = 0;   // millis() quando o zero foi detectado com bomba ligada
static bool     nivel_zero_ativo  = false;

// PZEM-004T v4.0 (TTL) — processados em float internamente
static float    pzem_tensao_v    = 0.0f;
static float    pzem_corrente_a  = 0.0f;
static float    pzem_potencia_w  = 0.0f;  // cast para uint32_t ao empacotar
static float    pzem_energia_kwh = 0.0f;
static float    pzem_freq_hz     = 0.0f;
static float    pzem_fp          = 0.0f;
static uint8_t  pzem_falhas      = 0;     // falhas consecutivas
static bool     pzem_ok          = true;  // false → ERRO_5_PZEM

// OTA
static volatile bool ota_requisitado = false;
static bool          ota_ativo       = false;
static uint32_t      inicio_ota      = 0;

// Auto-teste de relé colado — 2026-07-27 (marica-149/150)
static volatile bool autoteste_pendente     = false;  // setado em desligar_bomba() (roda também
                                                        // dentro do callback ESP-NOW cb_recepcao),
                                                        // consumido só no loop() — mesmo padrão
                                                        // já usado para ota_requisitado
static bool           rele_colado_detectado = false;  // resultado do último auto-teste,
                                                        // vai em bomba_estado_bitmask

// 2026-08-07 (marica-234) -- timestamp do último desligamento, pra loop_pzem() detectar se
// um desligar_bomba() aconteceu DURANTE a espera bloqueante de pzem_ler() (até ~500ms na UART)
// e descartar a leitura obsoleta em vez de sobrescrever o 0.0f que desligar_bomba() já tinha
// gravado. Sem isso, o fix do marica-232 sozinho tem uma corrida residual: cb_recepcao() roda
// em task separada da ESP-NOW (mesmo mecanismo do marica-156) e pode chamar desligar_bomba()
// enquanto loop_pzem() está no meio da leitura -- a leitura em voo já reflete a carga de ANTES
// do corte (~480W), e ao retornar sobrescreve o zero recém-gravado, recriando o falso positivo
// de "sessão manual" no próximo enviar_status(). volatile: escrita possível em task diferente
// da leitura (mesmo padrão de autoteste_pendente acima).
static volatile uint32_t ultimo_desligamento_ms = 0;

Preferences prefs;

// -------------------------------------------------------------
// FORWARD DECLARATIONS
// -------------------------------------------------------------
void iniciar_espnow();
void ligar_bomba(bool ignorar_nivel = false);
void desligar_bomba(bool por_erro, uint8_t motivo = CAUSA_DESLIGA_MANUAL);
void enviar_status(bool autoteste_concluido = false);  // 2026-08-06 -- default preserva
                                                          // os outros 14 pontos de chamada
                                                          // sem precisar tocar em nenhum
void iniciar_ota();
void encerrar_ota();
void loop_ota();
void loop_pzem();
bool pzem_ler(float &v, float &i, float &p, float &kwh, float &hz, float &fp);
void executar_autoteste_rele();
void verificar_silencio_agua();
void verificar_silencio_controle();
void verificar_tempo_max();
void verificar_nivel_zero();
void verificar_sensor_ladrao();
void logica_nivel();
void loop_telemetria();
void cb_envio(const uint8_t* mac_addr, esp_now_send_status_t status);
void cb_recepcao(const uint8_t* mac_addr, const uint8_t* dados, int len);

// -------------------------------------------------------------
// GESTÃO DE RÁDIO — BASE GEMINI (NÃO MODIFICAR)
// -------------------------------------------------------------
void iniciar_espnow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL_SEGURANCA_PADRAO, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[BOMBA] Erro ao iniciar ESP-NOW."));
        return;
    }

    esp_now_register_send_cb(cb_envio);
    esp_now_register_recv_cb(cb_recepcao);

    esp_now_peer_info_t peer = {};
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_del_peer(MAC_AGUA);
    memcpy(peer.peer_addr, MAC_AGUA, 6);
    esp_now_add_peer(&peer);

    esp_now_del_peer(MAC_CONTROLE);
    memcpy(peer.peer_addr, MAC_CONTROLE, 6);
    esp_now_add_peer(&peer);

    esp_now_del_peer(MAC_CARDPUTER);
    memcpy(peer.peer_addr, MAC_CARDPUTER, 6);
    esp_now_add_peer(&peer);

    Serial.printf("[BOMBA] ESP-NOW pronto. Canal: %d\n", CANAL_SEGURANCA_PADRAO);
}

// -------------------------------------------------------------
// OTA
// -------------------------------------------------------------
void iniciar_ota() {
    if (ota_ativo) return;

    setCpuFrequencyMhz(240);  // eleva clock para pilha TCP/IP

    if (bomba_ligada) {
        desligar_bomba(false, CAUSA_DESLIGA_OTA);
        Serial.println(F("[OTA] Bomba desligada por segurança."));
    }

    ota_ativo       = true;
    ota_requisitado = false;
    inicio_ota      = millis();

    esp_now_deinit();
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(300);

    WiFi.mode(WIFI_STA);
    WiFi.config(IP_BOMBA, IP_GATEWAY, IP_MASCARA, IP_DNS);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print(F("[OTA] Conectando ao Wi-Fi"));
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
        delay(500);
        Serial.print('.');
        esp_task_wdt_reset();
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[OTA] Falha Wi-Fi. Abortando."));
        setCpuFrequencyMhz(80);
        encerrar_ota();
        return;
    }

    Serial.printf("[OTA] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());

    ArduinoOTA.setHostname("caixa_bomba");
    ArduinoOTA.onStart([]() {
        Serial.println(F("[OTA] Upload iniciado."));
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
    ota_ativo       = false;
    ota_requisitado = false;
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(300);
    setCpuFrequencyMhz(80);
    iniciar_espnow();
    Serial.println(F("[OTA] Encerrado. ESP-NOW restaurado."));
}

void loop_ota() {
    if (!ota_ativo) return;
    ArduinoOTA.handle();
    esp_task_wdt_reset();
    if (millis() - inicio_ota >= OTA_TIMEOUT_MS) {
        Serial.println(F("[OTA] Timeout. Encerrando."));
        encerrar_ota();
    }
}

// -------------------------------------------------------------
// CALLBACKS ESP-NOW
// -------------------------------------------------------------
void cb_envio(const uint8_t* m, esp_now_send_status_t s) {
    if (s != ESP_NOW_SEND_SUCCESS)
        Serial.println(F("[BOMBA] Falha envio ESP-NOW."));
}

void cb_recepcao(const uint8_t* mac_addr, const uint8_t* dados, int len) {
    if (len < 1) return;
    uint8_t tipo = dados[0];

    // Telemetria da Caixa Água → Bomba
    if (tipo == PKT_TELEMETRIA_AGUA && len >= (int)sizeof(PacketTelemetriaAgua)) {
        PacketTelemetriaAgua pkt;
        memcpy(&pkt, dados, sizeof(pkt));
        nivel_atual_cm     = pkt.distancia_cm;
        agua_erro_hardware = pkt.erro_sensor;
        agua_motivo_status = pkt.motivo_status;  // 2026-07-25 -- repassado sem alteração
        ultimo_pacote_agua = millis();

        // Repassa estado do ladrão para o pacote de status
        ladrao_ativo = pkt.ladrao_ativo;

        // ERRO_3_LADRAO: seta independente do estado da bomba
        // Se o ladrão está ativo, há transbordo — é sempre um estado anormal
        if (pkt.ladrao_ativo) {
            if (!(erros_ativos & ERRO_3_LADRAO)) {
                Serial.println(F("[BOMBA] ERRO 3: Ladrao ativo! Setando erro."));
                erros_ativos |= ERRO_3_LADRAO;
                enviar_status();
            }
            if (bomba_ligada) {
                Serial.println(F("[BOMBA] Ladrao ativo com bomba ligada. Desligando."));
                desligar_bomba(true, CAUSA_DESLIGA_LADRAO);
            }
        }
        // Auto-cura: ladrão desativado → limpa ERRO_3_LADRAO
        if (!pkt.ladrao_ativo && (erros_ativos & ERRO_3_LADRAO)) {
            erros_ativos &= ~ERRO_3_LADRAO;
            Serial.println(F("[BOMBA] Ladrao desativado. Erro 3 limpo."));
            enviar_status();
        }

        Serial.printf("[BOMBA] Agua: %.1f cm | ErrHw=%s | Ladrao=%s\n",
                      nivel_atual_cm,
                      agua_erro_hardware ? "SIM" : "nao",
                      pkt.ladrao_ativo   ? "SIM" : "nao");
    }

    // Comando direto da Controle
    if (tipo == CMD_LIGA_BOMBA) {
        ultimo_pacote_controle = millis();
        modo_automatico = false;  // Controle retomou controle
        PacketComandoBomba pkt_cmd = {};
        if (len >= (int)sizeof(PacketComandoBomba)) {
            memcpy(&pkt_cmd, dados, sizeof(pkt_cmd));
            horario_permitido = pkt_cmd.horario_permitido;
        }
        if (!em_quarentena && !em_bloqueio && !modo_forcado) {
            Serial.println(F("[BOMBA] CMD_LIGA recebido. Verificando intertravamentos..."));
            ligar_bomba(pkt_cmd.ignorar_nivel);  // CMD manual — não verifica horario_permitido
        } else {
            Serial.println(F("[BOMBA] CMD_LIGA rejeitado (quarentena/bloqueio/forcado)."));
        }
    }

    if (tipo == CMD_DESLIGA_BOMBA) {
        ultimo_pacote_controle = millis();
        modo_automatico = false;  // Controle retomou controle
        if (len >= (int)sizeof(PacketComandoBomba)) {
            PacketComandoBomba pkt_cmd;
            memcpy(&pkt_cmd, dados, sizeof(pkt_cmd));
            horario_permitido = pkt_cmd.horario_permitido;
        }
        Serial.println(F("[BOMBA] CMD_DESLIGA recebido."));
        desligar_bomba(false, CAUSA_DESLIGA_MANUAL);
    }

    // Keep-alive de presença da Controle — atualiza timestamp e horário sem ação hidráulica
    if (tipo == CMD_PING_CONTROLE) {
        ultimo_pacote_controle = millis();
        modo_automatico = false;
        if (len >= (int)sizeof(PacketComandoBomba)) {
            PacketComandoBomba pkt_cmd;
            memcpy(&pkt_cmd, dados, sizeof(pkt_cmd));
            horario_permitido = pkt_cmd.horario_permitido;
        }
        Serial.printf("[BOMBA] Ping da Controle recebido. Horario:%s\n",
                      horario_permitido ? "PERMITIDO" : "BLOQUEADO");
    }

    // Reset remoto de bloqueios — limpa modo forçado, travas e NVS
    if (tipo == CMD_RESET_ERROS) {
        ultimo_pacote_controle = millis();
        modo_automatico   = false;
        if (len >= (int)sizeof(PacketComandoBomba)) {
            PacketComandoBomba pkt_cmd;
            memcpy(&pkt_cmd, dados, sizeof(pkt_cmd));
            horario_permitido = pkt_cmd.horario_permitido;
        }
        contador_disparos = 0;
        modo_forcado      = false;
        em_bloqueio       = false;
        erros_ativos     &= ~ERRO_1_TIMEOUT;
        prefs.begin("marica", false);
        prefs.putUChar("disparos", 0);
        prefs.end();
        Serial.println(F("[BOMBA] CMD_RESET recebido. Travas e NVS zeradas."));
        enviar_status();
    }

    // Configuração remota de níveis e timeout
    if (tipo == CMD_SET_NIVEIS && len >= (int)sizeof(PacketConfigNiveis)) {
        PacketConfigNiveis pkt_cfg;
        memcpy(&pkt_cfg, dados, sizeof(pkt_cfg));

        // Valida: liga > desliga, segurança > liga, manual_min < liga, timeout > 0,
        // segurança > SINALEIRA_AMARELO_MAX_CM (2026-08-07, marica-227 -- sem isso,
        // reconfigurar nivel_seguranca_cm pra <= SINALEIRA_AMARELO_MAX_CM torna o
        // vermelho estático da sinaleira inalcançável; Bomba valida como autoridade
        // final, não confia só na checagem da Controle em rota_set_niveis())
        if (pkt_cfg.nivel_liga_cm > pkt_cfg.nivel_desliga_cm &&
            pkt_cfg.nivel_seguranca_cm > pkt_cfg.nivel_liga_cm &&
            pkt_cfg.nivel_manual_min_cm < pkt_cfg.nivel_liga_cm &&
            pkt_cfg.timeout_minutos > 0 &&
            (float)pkt_cfg.nivel_seguranca_cm > SINALEIRA_AMARELO_MAX_CM) {

            nivel_liga_cm            = pkt_cfg.nivel_liga_cm;
            nivel_desliga_cm         = pkt_cfg.nivel_desliga_cm;
            nivel_seguranca_cm       = pkt_cfg.nivel_seguranca_cm;
            nivel_manual_min_cm      = pkt_cfg.nivel_manual_min_cm;
            timeout_operacao_minutos = pkt_cfg.timeout_minutos;

            prefs.begin("marica", false);
            prefs.putUChar("n_liga",     nivel_liga_cm);
            prefs.putUChar("n_desliga",  nivel_desliga_cm);
            prefs.putUChar("n_seg",      nivel_seguranca_cm);
            prefs.putUChar("n_man",      nivel_manual_min_cm);
            prefs.putUShort("t_max_min", timeout_operacao_minutos);
            prefs.end();

            Serial.printf("[BOMBA] CMD_SET_NIVEIS. Liga:%dcm Desliga:%dcm Seg:%dcm ManMin:%dcm Timeout:%dmin\n",
                          nivel_liga_cm, nivel_desliga_cm, nivel_seguranca_cm,
                          nivel_manual_min_cm, timeout_operacao_minutos);
            enviar_status();
        } else {
            Serial.println(F("[BOMBA] CMD_SET_NIVEIS rejeitado (parametros invalidos)."));
        }
    }

    // OTA via Cardputer
    if (tipo == CMD_OTA) {
        Serial.println(F("[BOMBA] CMD_OTA recebido. Flag setada."));
        ota_requisitado = true;
    }
}

// -------------------------------------------------------------
// CONTROLE DO RELÉ
// -------------------------------------------------------------
void ligar_bomba(bool ignorar_nivel) {
    if (bomba_ligada) return;

    // Intertravamento 1 — sensor inválido (NUNCA pulado, mesmo com ignorar_nivel)
    if (agua_erro_sensor) {
        Serial.println(F("[BOMBA] Partida bloqueada — sensor da Agua invalido."));
        return;
    }

    if (!ignorar_nivel) {
        // Intertravamento 2 — caixa cheia demais para justificar bombeamento
        // nivel_atual_cm é distância sensor→água: valores MENORES = mais água
        // Bloqueia se nivel_atual_cm < nivel_manual_min_cm (caixa acima do limiar mínimo configurado)
        if (nivel_atual_cm > 0.0f && nivel_atual_cm < (float)nivel_manual_min_cm) {
            Serial.printf("[BOMBA] Partida bloqueada — caixa cheia demais (%.0f cm < %d cm minimo).\n",
                          nivel_atual_cm, nivel_manual_min_cm);
            return;
        }

        // Intertravamento 3 — reservatório já cheio
        if (nivel_atual_cm > 0.0f && nivel_atual_cm <= (float)nivel_desliga_cm) {
            Serial.printf("[BOMBA] Partida bloqueada — reservatorio cheio (%.0f cm <= %d cm).\n",
                          nivel_atual_cm, nivel_desliga_cm);
            return;
        }
    } else {
        Serial.println(F("[BOMBA] Comando web: ignorando intertravamentos de nivel (2/3)."));
    }

    // Intertravamento 4 — leitura acima do limite de segurança (sensor anômalo)
    // NUNCA pulado -- é proteção contra leitura anômala/transbordo, categoria diferente
    // dos intertravamentos 2/3 (que só evitam bombeamento desnecessário).
    if (nivel_atual_cm > (float)nivel_seguranca_cm) {
        Serial.printf("[BOMBA] Partida bloqueada — nivel acima do limite de seguranca (%.0f cm > %d cm).\n",
                      nivel_atual_cm, nivel_seguranca_cm);
        return;
    }

    // Intertravamento 5 — erros físicos ativos (ERRO_5_PZEM mascarado — monitor-only)
    if (erros_ativos & ~ERRO_5_PZEM) {
        Serial.printf("[BOMBA] Partida bloqueada — erro fisico ativo: 0x%02X\n",
                      erros_ativos & ~ERRO_5_PZEM);
        return;
    }

    // Debounce de leitura — aguarda 500ms com leitura estável antes de atuar no relé
    // Evita que glitch momentâneo do sensor cause partida e estalo indesejado
    float nivel_verificacao = nivel_atual_cm;
    delay(500);
    if (fabsf(nivel_atual_cm - nivel_verificacao) > 2.0f) {
        Serial.println(F("[BOMBA] Partida bloqueada — leitura instavel (debounce)."));
        return;
    }

    bomba_ligada    = true;
    inicio_operacao = millis();
    digitalWrite(GPIO_K1, RELE_LIGA);
    digitalWrite(GPIO_K2, RELE_LIGA);
    Serial.println(F("[BOMBA] Bomba LIGADA (K1+K2)."));
    enviar_status();
}

void desligar_bomba(bool por_erro, uint8_t motivo) {
    if (!bomba_ligada && !por_erro) return;
    bomba_ligada = false;
    ultima_causa_desligamento = motivo;  // 2026-07-25
    digitalWrite(GPIO_K1, RELE_DESLIGA);
    digitalWrite(GPIO_K2, RELE_DESLIGA);
    // 2026-08-07 (marica-232) -- zera a leitura de potência/corrente ANTES de enviar_status().
    // Sem isso, o pacote deste exato instante carregava pzem_potencia_w desatualizado (só
    // refrescado por loop_pzem() a cada 5s) junto com bomba_ligada=false -- padrão que o
    // gatilho detectar_sessao_manual_marica() (Supabase) interpreta como "chave física Lukma
    // acionada" (bomba_ligada=false + pzem_w>100W). Fisicamente correto zerar aqui: sem
    // corrente fluindo pelos relés abertos, a leitura real é ~0 -- loop_pzem() reconfirma
    // isso no próximo ciclo de qualquer forma. Não afeta a detecção de relé colado, que usa
    // leituras próprias e dedicadas (executar_autoteste_rele()), nunca este campo global.
    pzem_potencia_w  = 0.0f;
    pzem_corrente_a  = 0.0f;
    ultimo_desligamento_ms = millis();  // 2026-08-07 (marica-234) -- loop_pzem() usa isso
                                         // pra descartar leitura em voo obsoleta (ver comentário
                                         // na declaração da variável)
    Serial.println(F("[BOMBA] Bomba DESLIGADA (K1+K2)."));
    autoteste_pendente = true;  // 2026-07-27 — dispara auto-teste no próximo loop() (marica-149/150)
    enviar_status();
}

// -------------------------------------------------------------
// PACOTE STATUS COMPLETO — Bomba → Controle
// -------------------------------------------------------------
void enviar_status(bool autoteste_concluido) {
    PacketStatusCompleto pkt = {};
    pkt.tipo               = PKT_STATUS_COMPLETO;
    pkt.agua_distancia_cm  = nivel_atual_cm;
    pkt.agua_erro_sensor   = agua_erro_sensor;
    pkt.agua_ladrao_ativo  = ladrao_ativo;
    pkt.bomba_rele_estado  = bomba_ligada;
    pkt.bomba_erro_bitmask = erros_ativos;
    pkt.pzem_potencia_w    = (uint32_t)pzem_potencia_w;
    pkt.pzem_tensao_v      = pzem_tensao_v;
    pkt.pzem_corrente_a    = pzem_corrente_a;
    pkt.pzem_fp            = pzem_fp;
    pkt.pzem_energia_kwh   = pzem_energia_kwh;
    pkt.agua_offline       = agua_offline;
    pkt.agua_motivo_status = agua_motivo_status;  // 2026-07-25 -- repassado sem alteração
    pkt.bomba_estado_bitmask =                    // 2026-07-25 -- só informativo, ver enum
          (em_quarentena         ? ESTADO_QUARENTENA   : 0)
        | (em_bloqueio           ? ESTADO_BLOQUEIO     : 0)
        | (modo_forcado          ? ESTADO_MODO_FORCADO : 0)
        | (rele_colado_detectado ? ESTADO_RELE_COLADO  : 0);  // 2026-07-27 -- marica-149/152
    pkt.bomba_causa_desligamento = ultima_causa_desligamento;  // 2026-07-25
    pkt.autoteste_concluido = autoteste_concluido;  // 2026-08-06 -- true só quando esta
                                                     // chamada vem de executar_autoteste_rele()
    esp_now_send(MAC_CONTROLE, (uint8_t*)&pkt, sizeof(pkt));
}

// -------------------------------------------------------------
// PZEM-004T v4.0 (TTL) — MODBUS RTU sobre UART simples
// Endereço geral 0xF8 (não endereçável em barramento — único dispositivo)
// Mapa de registradores idêntico ao usado anteriormente com o PZEM-016
// (mesma família de chip de medição), verificado contra o datasheet oficial
// do PZEM-004T v3.0/v4.0 antes desta alteração.
// -------------------------------------------------------------
static uint16_t modbus_crc(const uint8_t* buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else               { crc >>= 1; }
        }
    }
    return crc;
}

// Lê 10 registradores (0x0000–0x0009) do PZEM-004T
// Registradores:
//   0x0000        — Tensão (×0,1 V)
//   0x0001–0x0002 — Corrente (×0,001 A — Low Word First)
//   0x0003–0x0004 — Potência (×0,1 W — Low Word First)
//   0x0005–0x0006 — Energia (×0,001 kWh — Low Word First)
//   0x0007        — Frequência (×0,1 Hz)
//   0x0008        — Fator de Potência (×0,01)
//   0x0009        — Status de Alarme (não decodificado)
// 2026-07-27 — assinatura mudou de bool pzem_ler() pra receber os resultados por
// referência em vez de escrever direto nas globais pzem_tensao_v/etc. Motivo (achado
// na revisão Gemini do marica-149): cb_recepcao() roda em task separada do ESP-NOW: se
// um pacote chegar durante o auto-teste (que também chama esta função), o antigo
// pzem_ler() alteraria as globais no meio do teste, e enviar_status() disparado pelo
// callback vazaria a leitura parcial/de teste como se fosse telemetria real pra
// Controle/Supabase. Com saída por referência, cada chamador decide o que fazer com o
// resultado — loop_pzem() continua atualizando as globais normalmente (ver abaixo);
// executar_autoteste_rele() usa variáveis locais, sem tocar no estado global.
bool pzem_ler(float &v, float &i, float &p, float &kwh, float &hz, float &fp) {
    uint8_t req[] = {PZEM_ADDR, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
    uint16_t crc = modbus_crc(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    while (PZEM_SERIAL_PORT.available()) PZEM_SERIAL_PORT.read();
    PZEM_SERIAL_PORT.write(req, sizeof(req));

    uint32_t t0 = millis();
    while (PZEM_SERIAL_PORT.available() < 25 && millis() - t0 < 500) delay(1);
    if (PZEM_SERIAL_PORT.available() < 25) return false;

    uint8_t resp[25];
    PZEM_SERIAL_PORT.readBytes(resp, 25);

    uint16_t crc_recv = resp[23] | ((uint16_t)resp[24] << 8);
    if (modbus_crc(resp, 23) != crc_recv) return false;

    uint16_t reg[10];
    for (uint8_t i2 = 0; i2 < 10; i2++)
        reg[i2] = ((uint16_t)resp[3 + i2*2] << 8) | resp[4 + i2*2];

    v   = reg[0] * 0.1f;
    i   = ((uint32_t)reg[2] << 16 | reg[1]) * 0.001f;
    p   = ((uint32_t)reg[4] << 16 | reg[3]) * 0.1f;
    kwh = ((uint32_t)reg[6] << 16 | reg[5]) * 0.001f;
    hz  = reg[7] * 0.1f;
    fp  = reg[8] * 0.01f;

    return true;
}

void loop_pzem() {
    static uint32_t ultimo_pzem = 0;
    if (millis() - ultimo_pzem < PZEM_INTERVALO_MS) return;
    ultimo_pzem = millis();

    // 2026-08-07 (marica-234) -- snapshot ANTES da leitura bloqueante (até ~500ms na UART).
    // Comparado depois contra ultimo_desligamento_ms pra detectar se um desligar_bomba()
    // aconteceu enquanto pzem_ler() estava em voo -- nesse caso a leitura reflete a carga
    // de ANTES do corte e deve ser descartada, não sobrescrever o 0.0f já gravado.
    uint32_t t_inicio_leitura = millis();

    float v, i, p, kwh, hz, fp;
    if (pzem_ler(v, i, p, kwh, hz, fp)) {
        // Comparação segura contra overflow de millis() (mesmo idioma de silencio/timeout
        // já usado no arquivo): >= 0 significa que o desligamento aconteceu depois que a
        // leitura começou -- descarta, pois desligar_bomba() já é a fonte de verdade mais
        // recente pro estado de potência/corrente.
        if ((int32_t)(ultimo_desligamento_ms - t_inicio_leitura) >= 0) {
            Serial.println(F("[PZEM] Leitura descartada -- desligamento ocorreu durante a espera da UART."));
            return;
        }
        pzem_tensao_v    = v;
        pzem_corrente_a  = i;
        pzem_potencia_w  = p;
        pzem_energia_kwh = kwh;
        pzem_freq_hz     = hz;
        pzem_fp          = fp;
        pzem_falhas = 0;
        if (pzem_ok == false) {
            pzem_ok = true;
            erros_ativos &= ~ERRO_5_PZEM;
            Serial.println(F("[PZEM] Comunicacao restaurada."));
        }
        Serial.printf("[PZEM] U=%.1fV I=%.3fA P=%.1fW E=%.3fkWh F=%.1fHz FP=%.2f\n",
                      pzem_tensao_v, pzem_corrente_a, pzem_potencia_w,
                      pzem_energia_kwh, pzem_freq_hz, pzem_fp);
    } else {
        pzem_falhas++;
        Serial.printf("[PZEM] Falha %d/%d.\n", pzem_falhas, PZEM_FALHAS_MAX);
        if (pzem_falhas >= PZEM_FALHAS_MAX && pzem_ok) {
            pzem_ok = false;
            erros_ativos |= ERRO_5_PZEM;

            // Zera leituras instantâneas para evitar telemetria de dado fantasma
            // (energia acumulada é mantida — não é uma leitura instantânea)
            pzem_tensao_v    = 0.0f;
            pzem_corrente_a  = 0.0f;
            pzem_potencia_w  = 0.0f;
            pzem_freq_hz     = 0.0f;
            pzem_fp          = 0.0f;

            Serial.println(F("[PZEM] ERRO 5: Falha critica Modbus. Dados zerados."));
            enviar_status();
        }
    }
}

// -------------------------------------------------------------
// AUTO-TESTE DE RELÉ COLADO — 2026-07-27 (marica-149/150/152)
// -------------------------------------------------------------
// Roda só a partir do loop() (nunca de dentro de cb_recepcao) porque faz até 3
// leituras bloqueantes do PZEM via UART (pzem_ler(), até ~500ms cada) — rodar
// isso dentro de um callback ESP-NOW causaria WDT reset ou perda de pacotes
// (achado da revisão Gemini, ponto de boas práticas). Isola K1 e K2 um de cada
// vez; com o outro relé aberto, a potência real esperada é ~0. Falha do teste
// NÃO bloqueia a operação normal da bomba — só sinaliza via ESTADO_RELE_COLADO
// em bomba_estado_bitmask (nunca lido por decisão de segurança).
//
// Limite conhecido (revisão Gemini + Peter, 2026-07-27): se K1 e K2 colarem
// fechados SIMULTANEAMENTE, a Fase 1 (fechar K1 pra isolar K2) pode reenergizar
// a bomba de verdade contra um K2 já preso — risco físico real (tranco elétrico/
// hidráulico), sem solução em firmware pra colagem dupla simultânea (marica-140).
// A pré-checagem abaixo cobre o caso de ambos já estarem colados ANTES deste
// desligamento (não fecha nenhum relé nesse caso); não cobre o caso em que K2
// solda durante esta própria tentativa de abertura, com K1 abrindo normalmente
// — esse só aparece quando a Fase 1 fecha o K1 de novo, e não tem mitigação
// possível: é a física do relé em série, não um bug de lógica.
// -------------------------------------------------------------
// GUARD DE CONCORRÊNCIA — ligar_bomba() durante o auto-teste (2026-08-07, marica-224)
// -------------------------------------------------------------
// Revisão Gemini, 3ª rodada sobre o auto-teste: o guard "passivo" (só abortar sem
// tocar em GPIO) introduzido na 2ª rodada (marica-223) tinha duas lacunas reais:
// (1) nenhuma checagem existia ANTES das duas escritas incondicionais no topo da
// função -- se ligar_bomba() rodou via cb_recepcao() no intervalo entre
// desligar_bomba() setar autoteste_pendente=true e o loop() chamar esta função,
// bomba_ligada já chegava true e as duas primeiras linhas desligavam fisicamente
// uma bomba que tinha acabado de ligar; (2) mesmo com guard logo após cada
// delay()/pzem_ler(), existe uma janela estreita (nível de instrução, não de
// segundos) entre o guard aprovar (bomba_ligada==false) e o digitalWrite seguinte
// executar -- um scheduler preemptivo pode intercalar cb_recepcao() bem nesse meio.
// Fechar essa janela por completo exigiria seção crítica (portMUX, já usado no
// Cardputer pra um problema análogo) em ligar_bomba()/desligar_bomba()/aqui --
// fora do escopo desta correção pontual. Mitigação adotada: em vez de abortar
// passivamente (que pode deixar o hardware num estado que não é nem o do teste
// nem o do comando concorrente), o guard RESTAURA ativamente K1=LIGA/K2=LIGA
// sempre que detecta bomba_ligada=true. Se a janela estreita acima ainda assim
// for atingida, o PRÓXIMO guard (no máximo ~1,5s depois, o tempo de um
// delay(AUTOTESTE_ESTABILIZA_MS)) já corrige de volta -- troca "pode ficar
// dessincronizado até o próximo ciclo completo da bomba" por "autocorrige em
// até ~1,5s". Consolidado num único helper (não 9 blocos repetidos) por
// sugestão da revisão -- reduz risco de um ponto de chamada divergir dos outros
// com o tempo. Função, não macro: evita armadilhas de higiene de macro em C++
// (o "return" do chamador continua explícito em cada ponto de uso).
static bool autoteste_guard_bomba_ligou() {
    if (!bomba_ligada) return false;
    digitalWrite(GPIO_K1, RELE_LIGA);
    digitalWrite(GPIO_K2, RELE_LIGA);
    autoteste_pendente = false;
    Serial.println(F("[BOMBA] Auto-teste abortado -- bomba ligada durante o teste (comando concorrente). Rele restaurado."));
    return true;
}

void executar_autoteste_rele() {
    Serial.println(F("[BOMBA] Auto-teste de rele: iniciando."));

    // 2026-08-07 (marica-224, revisão Gemini 3ª rodada): guard adicional ANTES das
    // duas escritas incondicionais abaixo -- lacuna real: se ligar_bomba() rodou via
    // cb_recepcao() no intervalo entre desligar_bomba() setar autoteste_pendente=true
    // e o loop() chegar a chamar esta função, bomba_ligada já está true quando a função
    // COMEÇA a executar, e as duas linhas seguintes desligariam fisicamente uma bomba
    // que acabou de ligar legitimamente, sem nenhuma checagem prévia.
    if (autoteste_guard_bomba_ligou()) return;

    digitalWrite(GPIO_K1, RELE_DESLIGA);
    digitalWrite(GPIO_K2, RELE_DESLIGA);
    // 2026-08-07 -- debounce da pré-checagem alinhado a AUTOTESTE_ESTABILIZA_MS (ver
    // definição da macro pro valor atual e o porquê), mesma constante usada nas Fases
    // 1/2 abaixo. Antes usava 50ms fixo, sem relação documentada com o tempo real de
    // assentamento do PZEM após comutar o relé -- achado de campo (marica-221):
    // pré-checagem sinalizou "rele colado" logo após a bomba desligar de uma carga real
    // (~480W por ~14min); telemetria seguinte (pzem_w=0 constante por dezenas de minutos,
    // fora do auto-teste) contradiz relé fisicamente soldado -- consistente com o PZEM
    // ainda não ter assentado a leitura em só 50ms. Revisão Gemini (marica-222) apontou
    // risco maior no mesmo mecanismo: 300ms também era insuficiente e arriscava falso
    // negativo nas Fases 1/2 (relé colado de verdade passando como "OK").
    delay(AUTOTESTE_ESTABILIZA_MS);
    if (autoteste_guard_bomba_ligou()) return;

    // Pré-checagem — se já houver carga com os dois relés comandados abertos, os
    // dois provavelmente já estão colados fechados (marica-140). Não fecha K1
    // pra isolar K2 nesse caso — sinaliza falha direto, sem religar nada.
    float v0 = 0.0f, i0 = 0.0f, p0 = 0.0f, kwh0 = 0.0f, hz0 = 0.0f, fp0 = 0.0f;  // 2026-08-07,
                                        // marica-225: inicializados -- pzem_ler() retornando false
                                        // (CRC/timeout UART) não toca nessas variáveis, então sem
                                        // inicialização o valor seria lixo de stack
    bool ok_pre = pzem_ler(v0, i0, p0, kwh0, hz0, fp0);
    if (autoteste_guard_bomba_ligou()) return;
    if (ok_pre && p0 > AUTOTESTE_PZEM_MARGEM_W) {
        Serial.printf("[BOMBA] Auto-teste: carga ja presente antes do teste (%.1fW) -- ambos os reles suspeitos. Fases puladas.\n", p0);
        rele_colado_detectado = true;
        autoteste_pendente = false;
        // 2026-08-06 -- sempre envia ao concluir (mesmo se já era colado antes), pra
        // Controle poder carimbar quando o último auto-teste rodou de fato.
        enviar_status(true);
        return;
    }

    digitalWrite(GPIO_K1, RELE_LIGA);           // Fase 1 — K1 sozinho
    delay(AUTOTESTE_ESTABILIZA_MS);
    float v1 = 0.0f, i1 = 0.0f, p1 = 0.0f, kwh1 = 0.0f, hz1 = 0.0f, fp1 = 0.0f;  // 2026-08-07,
                                        // marica-225 -- ver comentário na declaração de v0 acima
    bool  ok_k1 = pzem_ler(v1, i1, p1, kwh1, hz1, fp1);
    if (autoteste_guard_bomba_ligou()) return;
    digitalWrite(GPIO_K1, RELE_DESLIGA);
    delay(100);
    if (autoteste_guard_bomba_ligou()) return;

    digitalWrite(GPIO_K2, RELE_LIGA);           // Fase 2 — K2 sozinho
    delay(AUTOTESTE_ESTABILIZA_MS);
    float v2 = 0.0f, i2 = 0.0f, p2 = 0.0f, kwh2 = 0.0f, hz2 = 0.0f, fp2 = 0.0f;  // 2026-08-07,
                                        // marica-225 -- ver comentário na declaração de v0 acima
    bool  ok_k2 = pzem_ler(v2, i2, p2, kwh2, hz2, fp2);
    if (autoteste_guard_bomba_ligou()) return;
    digitalWrite(GPIO_K2, RELE_DESLIGA);        // estado final: os dois abertos

    bool falha = (ok_k1 && p1 > AUTOTESTE_PZEM_MARGEM_W) ||
                 (ok_k2 && p2 > AUTOTESTE_PZEM_MARGEM_W);

    if (falha != rele_colado_detectado) {
        rele_colado_detectado = falha;
        Serial.printf("[BOMBA] Auto-teste: %s (K1=%.1fW K2=%.1fW margem=%.1fW)\n",
                      falha ? "FALHOU - rele colado" : "OK", p1, p2, AUTOTESTE_PZEM_MARGEM_W);
    } else {
        Serial.printf("[BOMBA] Auto-teste concluido: K1=%.1fW K2=%.1fW (sem mudanca de estado).\n", p1, p2);
    }
    // 2026-08-06 -- sempre envia ao concluir agora (antes só enviava se o resultado
    // mudasse) -- prova pro Controle/dashboard que o teste rodou agora, não só
    // quando o estado de rele_colado_detectado muda.
    enviar_status(true);

    autoteste_pendente = false;
}

// -------------------------------------------------------------
// PROTEÇÕES
// -------------------------------------------------------------

// Silêncio da Água > 60s → desliga bomba + sinaliza agua_offline (distinto de
// agua_erro_sensor, que passa a refletir só o erro real reportado pela própria
// Água). Não queima bit do bitmask — usa os dois booleanos no pacote.
void verificar_silencio_agua() {
    if (ultimo_pacote_agua == 0) {
        agua_offline = true;  // sem dados iniciais = offline por precaução
        return;
    }

    bool silencio = (millis() - ultimo_pacote_agua > SILENCIO_AGUA_MS);

    if (silencio) {
        if (!agua_offline) {
            agua_offline = true;
            Serial.println(F("[BOMBA] Silencio da Agua >60s — offline."));
        }
        if (bomba_ligada) {
            Serial.println(F("[BOMBA] Desligando por ausencia de dados da Agua."));
            desligar_bomba(true, CAUSA_DESLIGA_SILENCIO_AGUA);
        }
        // agua_erro_sensor NAO e tocado aqui -- mantem o ultimo valor real
        // conhecido antes do silencio (mesmo comportamento "congelado" que
        // nivel_atual_cm ja tem). logica_nivel() cobre os dois com OR, entao
        // isso nao compromete seguranca -- so preserva o ultimo dado real
        // conhecido em vez de mascara-lo.
    } else {
        // Auto-cura: rádio voltou — assume o estado real do hardware da Água
        agua_offline      = false;
        agua_erro_sensor  = agua_erro_hardware;
    }
}

// Timeout de operação → ERRO_1_TIMEOUT
void verificar_tempo_max() {
    if (!bomba_ligada) return;
    // Conversão segura: minutos → ms sem overflow de uint32_t (máx ~71 min em ms)
    uint32_t timeout_ms = (uint32_t)timeout_operacao_minutos * 60UL * 1000UL;
    if (millis() - inicio_operacao >= timeout_ms) {
        Serial.printf("[BOMBA] ERRO 1: Tempo maximo excedido (%d min).\n",
                      timeout_operacao_minutos);
        erros_ativos |= ERRO_1_TIMEOUT;
        desligar_bomba(true, CAUSA_DESLIGA_TIMEOUT);

        contador_disparos++;
        prefs.begin("marica", false);
        prefs.putUChar("disparos", contador_disparos);
        prefs.end();

        em_bloqueio     = true;
        inicio_bloqueio = millis();

        if (contador_disparos >= DISPAROS_MODO_FORCADO) {
            modo_forcado = true;
            Serial.println(F("[BOMBA] Modo FORCADO ativado — partidas bloqueadas."));
        }
        enviar_status();
    }
}

// Sensor do ladrão: processado no cb_recepcao via PacketTelemetriaAgua.ladrao_ativo
// ERRO_3_LADRAO é setado/limpo em cb_recepcao independente do estado da bomba
void verificar_sensor_ladrao() {
    // Sem ação local — ladrão gerenciado remotamente pela Caixa Água
}

// Proteção de nível zero — desliga bomba se sensor marcar zero por 2 min consecutivos
// Evita que bomba fique ligada indefinidamente com sensor inválido ou deslocado
void verificar_nivel_zero() {
    if (!bomba_ligada) {
        nivel_zero_ativo  = false;
        nivel_zero_inicio = 0;
        return;
    }

    if (nivel_atual_cm <= 0.0f) {
        if (!nivel_zero_ativo) {
            nivel_zero_ativo  = true;
            nivel_zero_inicio = millis();
            Serial.println(F("[BOMBA] Nivel zero detectado com bomba ligada. Iniciando temporizador."));
        } else if (millis() - nivel_zero_inicio >= NIVEL_ZERO_TIMEOUT_MS) {
            Serial.println(F("[BOMBA] SEGURANCA: Nivel zero persistente por 2min. Desligando bomba."));
            desligar_bomba(true, CAUSA_DESLIGA_NIVEL_ZERO);
            nivel_zero_ativo = false;
        }
    } else {
        // Nível voltou — cancela temporizador
        if (nivel_zero_ativo) {
            Serial.println(F("[BOMBA] Nivel restaurado. Temporizador de zero cancelado."));
        }
        nivel_zero_ativo  = false;
        nivel_zero_inicio = 0;
    }
}

// Silêncio da Controle → assume modo automático
// Boot sem Controle: carência de 8 min (quarentena 5 min + 3 min margem)
// Em operação: silêncio de 20 min sem CMD_LIGA, CMD_DESLIGA ou CMD_PING_CONTROLE
void verificar_silencio_controle() {
    bool silencio;

    if (ultimo_pacote_controle == 0) {
        // Nunca recebeu pacote da Controle — usa carência de boot
        silencio = (millis() >= SILENCIO_CONTROLE_BOOT_MS);
    } else {
        // Já recebeu ao menos um pacote — usa silêncio de 20 min
        silencio = (millis() - ultimo_pacote_controle >= SILENCIO_CONTROLE_MS);
    }

    if (silencio && !modo_automatico) {
        modo_automatico = true;
        Serial.println(F("[BOMBA] Controle ausente. Assumindo modo AUTOMATICO."));
    }
}

// -------------------------------------------------------------
// LÓGICA DE NÍVEL AUTOMÁTICO
// -------------------------------------------------------------
void logica_nivel() {
    if (nivel_atual_cm <= 0.0f) return;
    if (agua_erro_sensor || agua_offline) return;  // cobre erro real E silêncio de rádio

    // Desliga se leitura acima do limite de segurança — sensor anômalo
    if (nivel_atual_cm > (float)nivel_seguranca_cm) {
        if (bomba_ligada) {
            Serial.printf("[BOMBA] SEGURANCA: Nivel %.1f cm acima do limite %d cm. Desligando.\n",
                          nivel_atual_cm, nivel_seguranca_cm);
            desligar_bomba(true, CAUSA_DESLIGA_SEGURANCA);
        }
        return;
    }

    // Liga quando nível baixo — ERRO_5_PZEM mascarado (monitor-only)
    if (!bomba_ligada && nivel_atual_cm >= (float)nivel_liga_cm) {
        if (!modo_forcado && !em_quarentena && !em_bloqueio && modo_automatico) {
            if (!horario_permitido) {
                Serial.println(F("[BOMBA] Partida automatica bloqueada — fora do horario permitido."));
            } else {
                Serial.printf("[BOMBA] Nivel %.1f cm >= %d cm. Ligando (auto).\n",
                              nivel_atual_cm, nivel_liga_cm);
                ligar_bomba();
            }
        }
    }

    // Desliga quando nível alto — proteção incondicional em todos os modos
    if (bomba_ligada && nivel_atual_cm <= (float)nivel_desliga_cm) {
        Serial.printf("[BOMBA] Nivel %.1f cm <= %d cm. Desligando.\n",
                      nivel_atual_cm, nivel_desliga_cm);
        desligar_bomba(false, CAUSA_DESLIGA_NIVEL_CHEIO);
    }

    // Desliga se erro físico/hidráulico surgir durante operação
    if (bomba_ligada && (erros_ativos & ~ERRO_5_PZEM)) {
        Serial.printf("[BOMBA] Erro fisico durante operacao (0x%02X). Desligando.\n",
                      erros_ativos & ~ERRO_5_PZEM);
        desligar_bomba(true, CAUSA_DESLIGA_ERRO_FISICO);
    }
}

// -------------------------------------------------------------
// TELEMETRIA PERIÓDICA
// -------------------------------------------------------------
void loop_telemetria() {
    static uint32_t ultimo_envio = 0;
    // Frequência maior com bomba ligada — Controle precisa de dados em tempo real
    uint32_t intervalo = bomba_ligada ? 10000UL : 120000UL;
    if (millis() - ultimo_envio >= intervalo) {
        ultimo_envio = millis();
        enviar_status();
    }
}

// -------------------------------------------------------------
// SETUP
// -------------------------------------------------------------
void setup() {
    setCpuFrequencyMhz(80);  // operação normal em 80 MHz

    Serial.begin(115200);
    delay(500);

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    // Relés — estado seguro inicial (dual-relay em série, K1+K2, marica-118)
    // digitalWrite antes do pinMode evita micro-pulso LOW no boot
    // OUTPUT_OPEN_DRAIN: HIGH = alta impedância (corta corrente fantasma) | LOW = GND (liga relé)
    digitalWrite(GPIO_K1, RELE_DESLIGA);
    digitalWrite(GPIO_K2, RELE_DESLIGA);
    pinMode(GPIO_K1, OUTPUT_OPEN_DRAIN);
    pinMode(GPIO_K2, OUTPUT_OPEN_DRAIN);

    // UART TTL — PZEM-004T v4.0 (ligação direta, sem MAX485)
    PZEM_SERIAL_PORT.begin(PZEM_BAUD, SERIAL_8N1, GPIO_PZEM_RX, GPIO_PZEM_TX);

    // NVS — carrega estado persistente
    prefs.begin("marica", true);
    contador_disparos        = prefs.getUChar("disparos",  0);
    nivel_liga_cm            = prefs.getUChar("n_liga",    NIVEL_LIGA_CM_PADRAO);
    nivel_desliga_cm         = prefs.getUChar("n_desliga", NIVEL_DESLIGA_CM_PADRAO);
    nivel_seguranca_cm       = prefs.getUChar("n_seg",     NIVEL_SEGURANCA_CM_PADRAO);
    nivel_manual_min_cm      = prefs.getUChar("n_man",     NIVEL_MANUAL_MIN_CM_PADRAO);
    timeout_operacao_minutos = prefs.getUShort("t_max_min", 60);
    prefs.end();

    if (contador_disparos >= DISPAROS_MODO_FORCADO) {
        modo_forcado = true;
        Serial.println(F("[BOMBA] Modo FORCADO ativo por historico NVS."));
    }

    Serial.println(F("=========================================="));
    Serial.println(F(" CAIXA BOMBA - PRODUCAO MARICA v2"));
    Serial.printf(" MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  MAC_PROPRIA[0], MAC_PROPRIA[1], MAC_PROPRIA[2],
                  MAC_PROPRIA[3], MAC_PROPRIA[4], MAC_PROPRIA[5]);
    Serial.printf(" NVS: disparos=%d liga=%dcm desliga=%dcm\n",
                  contador_disparos, nivel_liga_cm, nivel_desliga_cm);
    Serial.println(F(" Modo inicial: SEMIAUTOMATICO (aguarda Controle ou 20min)"));
    Serial.println(F(" OTA serial: envie 'o' no Monitor Serial"));
    Serial.println(F("=========================================="));

    iniciar_espnow();

    inicio_quarentena = millis();
    Serial.println(F("[BOMBA] Quarentena iniciada (5 min)."));
}

// -------------------------------------------------------------
// LOOP
// -------------------------------------------------------------
void loop() {
    esp_task_wdt_reset();

    // Gatilho OTA via serial (bancada)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'o' || c == 'O') {
            Serial.println(F("[OTA] Gatilho serial recebido."));
            ota_requisitado = true;
        }
    }

    // OTA — prioridade absoluta
    if (ota_requisitado && !ota_ativo) iniciar_ota();
    if (ota_ativo) {
        loop_ota();
        return;
    }

    // PZEM — executa sempre, inclusive durante quarentena e bloqueio
    loop_pzem();

    uint32_t agora = millis();

    // Quarentena pós-boot — gerencia transição sem bloquear o loop
    if (em_quarentena) {
        if (agora - inicio_quarentena >= QUARENTENA_MS) {
            em_quarentena = false;
            Serial.println(F("[BOMBA] Quarentena finalizada."));
            enviar_status();
        }
    }

    // Bloqueio pós-timeout — gerencia transição sem bloquear o loop
    if (em_bloqueio) {
        if (agora - inicio_bloqueio >= TEMPO_BLOQUEIO_MS) {
            em_bloqueio   = false;
            erros_ativos &= ~ERRO_1_TIMEOUT;
            Serial.println(F("[BOMBA] Bloqueio encerrado."));
            enviar_status();
        }
    }

    // Proteções — executam sempre para garantir auditoria e telemetria contínua
    verificar_silencio_agua();
    verificar_silencio_controle();
    verificar_tempo_max();
    verificar_nivel_zero();
    verificar_sensor_ladrao();

    // Auto-teste de relé colado — só aqui no loop principal, nunca dentro do
    // callback ESP-NOW (delay/UART bloqueante) — 2026-07-27 (marica-149/150)
    if (autoteste_pendente) executar_autoteste_rele();

    // Lógica de nível — bloqueada internamente se quarentena/bloqueio/forcado
    logica_nivel();

    // Telemetria periódica — sempre ativa, mantém a Controle informada
    loop_telemetria();

    delay(10);
}
