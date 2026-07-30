// =============================================================
// PROJETO AUTOMACAO MARICA - CAIXA AGUA
// Protocolo v2 — Nó de Telemetria Upstream
// Base de rádio: Gemini (NÃO MODIFICAR)
// Sensor: AJ-SR04M-2 modo Trigger/Echo (sem R19)
//
// Fluxo:
//   Envia: PacketTelemetriaAgua (0x20) → Caixa Bomba
//   Recebe: PacketStatusCompleto (0x35) ← Caixa Bomba (para delta do filtro)
//   Recebe: PacketComandoOTA (0xB0) ← Cardputer
//
// OTA em campo: Cardputer envia CMD_OTA (0xB0) via ESP-NOW
// Canal fixo: 2 (CANAL_SEGURANCA_PADRAO)
//
// Alterações v2.2:
//   - Compensação de multicaminho acústico (condensação no transdutor)
//   - modo_reflexao: divide bruto por 2 quando eco secundário detectado
//   - Ativação: salto > 15 cm E bruto ≈ 2×ultima_valida (±10 cm)
//   - Desativação: bruto retorna à proximidade de ultima_valida (±10 cm)
//
// Alterações v2.3:
//   - Detecção de condensação no boot/pós-OTA (ponto cego do v2.2)
//   - NIVEL_CHEIO_REFERENCIA_CM=40: distância típica com caixa cheia
//   - Se primeira leitura ≈ 2×40=80 cm (±10 cm), ativa modo_reflexao
//     antes de inicializar o filtro — resolve eco já ativo no boot
//
// Alterações v2.4:
//   - modo_reflexao (operação contínua) agora exige REFLEXAO_CONFIRMA_N
//     leituras consecutivas para ativar OU desativar — mesmo padrão já
//     usado no filtro principal (FILTRO_CONFIRMA_RESET) e no ladrão
//     (LADRAO_CONFIRMA_N). Evita que um único eco espúrio isolado ligue
//     ou desligue a compensação por engano. Detecção de boot/pós-OTA
//     mantida sem confirmação (janela estreita ±4cm, evento único).
//   - REFLEXAO_TIMEOUT_MS: condensação persistente por mais de 20min
//     deixa de ser tratada como "corrigida". modo_reflexao é forçado a
//     false, o filtro é resetado, e reflexao_bloqueada impede reentrada
//     automática — a leitura só volta a ser aceita quando o eco PRIMÁRIO
//     (não compensado) reconfirmar plausibilidade contra a última leitura
//     real conhecida. Enquanto isso, reporta erro_sensor=true a cada
//     ciclo pelo caminho já existente (MAX_FALHAS_SENSOR) — sem qualquer
//     mudança de protocolo, Supabase ou dashboard. Motivado por incidente
//     de 2026-07-17 (marica-076/077): sensor travado 6h30+ em leitura
//     constante por condensação persistente, sem qualquer sinalização.
//
// Alterações v2.5:
//   - INSTABILIDADE_TIMEOUT_MS: rejeições persistentes do filtro que NÃO
//     batem no padrão de eco dobrado (não ativam modo_reflexao) — ex:
//     turbulência/espuma na superfície durante o próprio bombeamento —
//     agora também têm timeout (3min, só conta com bomba_ligada=true).
//     Antes, o ramo "Filtro instável" podia repetir ultima_valida com
//     erro_sensor=false indefinidamente, sem nunca sinalizar falha,
//     mesmo com a água possivelmente subindo de verdade. Achado da
//     revisão de segurança de 2026-07-25 (Claude + Gemini, duas rodadas
//     de revisão externa) — não motivado por incidente de campo, é
//     preventivo.
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <marica_protocol.h>

// MAC desta placa
static const uint8_t MAC_PROPRIA[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x01}; // TODO: MAC desta placa (deve bater com MAC_AGUA em marica_protocol.h)

// -------------------------------------------------------------
// PINOS DO SENSOR
// -------------------------------------------------------------
#define PIN_TRIG   17  // Fio amarelo — direto
#define PIN_ECHO   16  // Fio verde  — divisor R1=1kΩ + R2=2kΩ → 3,33V
#define PIN_LADRAO 18  // XKC-Y26S-V OUT — divisor → 3,33V | NPN: LOW = ativo

// -------------------------------------------------------------
// PARÂMETROS DO FILTRO ADAPTATIVO
// -------------------------------------------------------------
#define FILTRO_N               10
#define FILTRO_DELTA_OFF        5.0f   // δ com bomba desligada (cm)
#define FILTRO_DELTA_ON        15.0f   // δ com bomba ligada (cm)
#define FILTRO_MAX_REJEICOES   30      // Rejeições consecutivas antes do reset forçado
                                       // (aumentado de 10 para resistir a interferências transitórias)
#define FILTRO_CONFIRMA_RESET   3      // Confirmações após reset da janela
#define FILTRO_DELTA_PLAUSIVEL 20.0f   // Desvio máximo aceito no primeiro valor pós-reset
                                       // em relação a ultima_valida. Impede que leituras
                                       // espúrias se tornem nova referência do filtro.

// -------------------------------------------------------------
// PARÂMETROS DE DETECÇÃO DE CONDENSAÇÃO (modo_reflexao)
// -------------------------------------------------------------
#define REFLEXAO_CONFIRMA_N        3      // leituras consecutivas p/ ativar OU desativar
                                           // (mesmo padrão de FILTRO_CONFIRMA_RESET/LADRAO_CONFIRMA_N —
                                           // evita disparo por um único eco espúrio isolado)
#define REFLEXAO_TIMEOUT_MS  1200000UL    // 20 min — condensação persistente além disso deixa de
                                           // ser tratada como "corrigida" e passa a reportar falha

// -------------------------------------------------------------
// PARÂMETRO DE TIMEOUT — INSTABILIDADE GENÉRICA DO FILTRO (v2.5)
// -------------------------------------------------------------
// Cobre rejeições persistentes que NÃO batem no padrão de eco dobrado (não
// ativam modo_reflexao) — ex: turbulência/espuma na superfície durante o
// próprio bombeamento. Sem isso, enviar_telemetria(ultima_valida, false, ...)
// no ramo "Filtro instável" (heartbeat) poderia repetir um nível CONGELADO
// indefinidamente, sem nunca sinalizar erro_sensor=true, enquanto a água
// pode estar subindo de verdade com a bomba ligada. Achado da revisão de
// segurança de 2026-07-25 (Claude + Gemini, duas rodadas).
// Só conta com a bomba ligada — parado, instabilidade prolongada não é
// urgente (não há enchimento em curso).
#define INSTABILIDADE_TIMEOUT_MS 180000UL  // 3 min — mais curto que os 20min da
                                            // condensação por ser um risco mais
                                            // urgente (bombeamento ativo em curso)

// Referência física de caixa cheia — usada na detecção de condensação no boot.
// Distância típica do sensor até a lâmina de água com reservatório cheio (cm).
// Valores menores indicam enchimento manual acima do ladrão — também válidos.
// Eco secundário de boot esperado: ≈ 2 × 37 = 74 cm.
// Janela de detecção: [70, 78] cm (±4 cm) — exclui faixa de caixa vazia (~85–90 cm).
#define NIVEL_CHEIO_REFERENCIA_CM 37.0f

// -------------------------------------------------------------
// PARÂMETROS DE OPERAÇÃO
// -------------------------------------------------------------
#define INTERVALO_ENVIO_MS   5000UL   // envia telemetria a cada 5s
#define JANELA_OTA_MS      600000UL   // 10 min janela OTA
#define WDT_TIMEOUT_S           30
#define ESPNOW_RETRY_MS      10000UL
#define MAX_FALHAS_SENSOR        3

// Confirmação do sensor do ladrão — anti-falso-positivo
// 3 leituras consecutivas com intervalo de 1s = 3s de confirmação
// Ignora respingos e vibração mecânica no cano do ladrão
#define LADRAO_CONFIRMA_N        3    // leituras consecutivas necessárias
#define LADRAO_INTERVALO_MS   1000UL  // intervalo entre leituras de confirmação

// -------------------------------------------------------------
// ESTADO DO SISTEMA
// -------------------------------------------------------------
static volatile bool bomba_ligada    = false;  // atualizado pelo PacketStatusCompleto
static bool          ladrao_ativo    = false;  // confirmado após LADRAO_CONFIRMA_N leituras
static bool          ota_ativo       = false;
static uint32_t      inicio_ota      = 0;
static bool          espnow_ok       = false;
static uint8_t       falhas_sensor   = 0;
static uint8_t       confirma_pos_reset = 0;
static volatile bool ota_requisitado    = false;
static volatile bool reboot_requisitado = false;  // CMD_REBOOT do Cardputer

// ultima_valida promovida a variável de módulo para ser acessível em filtrar().
// Valor inicial 999.0f é sentinela de boot (fisicamente impossível — ler_sensor()
// restringe saídas válidas a 2.0–400.0f). Desativa a guarda de plausibilidade
// na primeira leitura, quando não há histórico ainda.
static float         ultima_valida   = 999.0f;

// modo_reflexao: compensação de multicaminho acústico (condensação no transdutor).
// Quando ativo, o valor bruto do sensor é dividido por 2 antes de entrar no filtro.
// Ativado quando bruto ≈ 2×ultima_valida com salto > 15 cm (eco secundário).
// Desativado quando bruto retorna à proximidade de ultima_valida (condensação dissipada).
static bool          modo_reflexao   = false;

// v2.4 — confirmação de entrada/saída e timeout de condensação persistente.
static uint32_t modo_reflexao_inicio    = 0;      // millis() de quando modo_reflexao ativou
static uint8_t  confirma_reflexao_entra = 0;      // leituras consecutivas confirmando entrada
static uint8_t  confirma_reflexao_sai   = 0;      // leituras consecutivas confirmando saída
static bool     reflexao_bloqueada      = false;  // pós-timeout: impede reentrada automática
                                                   // até o eco PRIMÁRIO (não compensado) reconfirmar
static bool     aguardando_reconfirmacao = false; // pós-timeout: reporta erro_sensor=true a cada
                                                   // ciclo até uma leitura nova ser confirmada

// v2.5 — timeout do "filtro instável" (rejeições persistentes fora do padrão
// de eco dobrado). instabilidade_inicio marca o millis() da PRIMEIRA vez que
// se entrou no ramo "Filtro instável" desde a última leitura válida — não é
// reiniciado a cada rejeição individual, só quando o filtro volta a confirmar
// um valor (recuperação) ou no boot.
static uint32_t instabilidade_inicio = 0;
static bool     instabilidade_ativa  = false;

// -------------------------------------------------------------
// FILTRO ADAPTATIVO — totalmente em float
// -------------------------------------------------------------
static float   janela[FILTRO_N] = {};
static uint8_t janela_idx       = 0;
static uint8_t janela_count     = 0;
static uint8_t rejeicoes_consec = 0;

static float calcular_mediana() {
    uint8_t n = (janela_count < FILTRO_N) ? janela_count : FILTRO_N;
    float tmp[FILTRO_N];
    memcpy(tmp, janela, n * sizeof(float));
    for (uint8_t i = 1; i < n; i++) {
        float key = tmp[i]; int8_t j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    return tmp[n / 2];
}

static float calcular_media() {
    uint8_t n = (janela_count < FILTRO_N) ? janela_count : FILTRO_N;
    if (n == 0) return 0.0f;
    float soma = 0.0f;
    for (uint8_t i = 0; i < n; i++) soma += janela[i];
    return soma / n;
}

static void janela_push(float valor) {
    janela[janela_idx % FILTRO_N] = valor;
    janela_idx++;
    if (janela_count < FILTRO_N) janela_count++;
}

static void janela_reset() {
    janela_idx         = 0;
    janela_count       = 0;
    rejeicoes_consec   = 0;
    confirma_pos_reset = 0;
    Serial.println(F("[AGUA] Filtro: janela resetada."));
}

// Retorna valor filtrado (float cm) ou -1.0f se rejeitada/em confirmação.
// Usa ultima_valida (variável de módulo) para validação de plausibilidade pós-reset.
static float filtrar(float bruto) {
    float delta = bomba_ligada ? FILTRO_DELTA_ON : FILTRO_DELTA_OFF;

    if (janela_count == 0) {
        // Primeiro valor após reset: verifica plausibilidade em relação ao histórico.
        // ultima_valida <= 400.0f garante que a checagem só ocorre se já há histórico.
        // No boot, ultima_valida=999.0f (sentinela impossível) — condição falsa, aceita tudo.
        if (ultima_valida <= 400.0f) {
            float desvio_hist = fabsf(bruto - ultima_valida);
            if (desvio_hist > FILTRO_DELTA_PLAUSIVEL) {
                Serial.printf("[AGUA] Filtro: pós-reset rejeitado por implausibilidade "
                              "(bruto=%.1f valida=%.1f desvio=%.1f max=%.1f)\n",
                              bruto, ultima_valida, desvio_hist, FILTRO_DELTA_PLAUSIVEL);
                // Não inicializa a janela — aguarda uma leitura plausível
                return -1.0f;
            }
        }
        janela_push(bruto);
        rejeicoes_consec   = 0;
        confirma_pos_reset = 1;
        return -1.0f;
    }

    // Fase de confirmação pós-reset
    if (confirma_pos_reset > 0 && confirma_pos_reset < FILTRO_CONFIRMA_RESET) {
        float desvio = fabsf(bruto - janela[0]);
        if (desvio <= delta) {
            janela_push(bruto);
            confirma_pos_reset++;
            Serial.printf("[AGUA] Confirmacao pos-reset: %d/%d\n",
                          confirma_pos_reset, FILTRO_CONFIRMA_RESET);
            if (confirma_pos_reset >= FILTRO_CONFIRMA_RESET) {
                Serial.println(F("[AGUA] Novo valor confirmado."));
                return calcular_media();
            }
        } else {
            Serial.printf("[AGUA] Confirmacao falhou (desvio=%.1f). Reiniciando.\n", desvio);
            janela_reset();
            janela_push(bruto);
            confirma_pos_reset = 1;
        }
        return -1.0f;
    }

    // Operação normal
    float desvio = fabsf(bruto - calcular_mediana());

    if (desvio > delta) {
        rejeicoes_consec++;
        Serial.printf("[AGUA] Rejeitada: %.1f cm (desvio=%.1f delta=%.1f)\n",
                      bruto, desvio, delta);
        if (rejeicoes_consec >= FILTRO_MAX_REJEICOES) {
            janela_reset();
            // Não chama janela_push aqui: a próxima leitura entrará pelo
            // bloco (janela_count == 0) com validação de plausibilidade.
        }
        return -1.0f;
    }

    rejeicoes_consec   = 0;
    confirma_pos_reset = 0;
    janela_push(bruto);
    return calcular_media();
}

// -------------------------------------------------------------
// LEITURA DO SENSOR (Trigger/Echo) — retorna float cm
// -------------------------------------------------------------
static float ler_sensor() {
    digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long dur = pulseIn(PIN_ECHO, HIGH, 20000);
    if (dur == 0) return -1.0f;

    float cm = (dur * 0.0343f) / 2.0f;
    return (cm < 2.0f || cm > 400.0f) ? -1.0f : cm;
}

// -------------------------------------------------------------
// FORWARD DECLARATIONS
// -------------------------------------------------------------
void cb_envio(const uint8_t* mac_addr, esp_now_send_status_t status);
void cb_recepcao(const uint8_t* mac_addr, const uint8_t* dados, int len);
bool iniciar_espnow();
void iniciar_ota();
void encerrar_ota();
void enviar_telemetria(float distancia_cm, bool erro_sensor, bool ladrao, uint8_t motivo);

// -------------------------------------------------------------
// GESTÃO DE RÁDIO — BASE GEMINI (NÃO MODIFICAR)
// -------------------------------------------------------------
bool iniciar_espnow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL_SEGURANCA_PADRAO, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[AGUA] Erro ao iniciar ESP-NOW."));
        return false;
    }

    esp_now_register_send_cb(cb_envio);
    esp_now_register_recv_cb(cb_recepcao);

    esp_now_peer_info_t peer = {};
    peer.channel = 0;
    peer.encrypt = false;

    // Água envia apenas para a Bomba — hub central do ecossistema
    esp_now_del_peer(MAC_BOMBA);
    memcpy(peer.peer_addr, MAC_BOMBA, 6);
    esp_now_add_peer(&peer);

    // Cardputer — fonte do CMD_OTA em campo
    esp_now_del_peer(MAC_CARDPUTER);
    memcpy(peer.peer_addr, MAC_CARDPUTER, 6);
    esp_now_add_peer(&peer);

    Serial.printf("[AGUA] ESP-NOW pronto. Canal: %d\n", CANAL_SEGURANCA_PADRAO);
    return true;
}

// -------------------------------------------------------------
// CALLBACKS ESP-NOW
// -------------------------------------------------------------
void cb_envio(const uint8_t* m, esp_now_send_status_t s) {
    if (s != ESP_NOW_SEND_SUCCESS)
        Serial.println(F("[AGUA] Falha no envio."));
}

void cb_recepcao(const uint8_t* mac_addr, const uint8_t* dados, int len) {
    if (len < 1) return;
    uint8_t tipo = dados[0];

    // PacketStatusCompleto da Bomba — lê estado do relé para ajustar delta do filtro
    if (tipo == PKT_STATUS_COMPLETO && len >= (int)sizeof(PacketStatusCompleto)) {
        PacketStatusCompleto pkt;
        memcpy(&pkt, dados, sizeof(pkt));
        bool era = bomba_ligada;
        bomba_ligada = pkt.bomba_rele_estado;
        if (bomba_ligada != era) {
            Serial.printf("[AGUA] Bomba: %s → delta=%.1f cm\n",
                          bomba_ligada ? "LIGADA" : "DESLIGADA",
                          bomba_ligada ? FILTRO_DELTA_ON : FILTRO_DELTA_OFF);
        }
        return;
    }

    // CMD_OTA do Cardputer — nunca chama iniciar_ota() no callback
    // esp_now_deinit() dentro da task do rádio causa deadlock
    if (tipo == CMD_OTA) {
        Serial.println(F("[AGUA] CMD_OTA recebido. Flag setada."));
        ota_requisitado = true;
        return;
    }

    // CMD_REBOOT do Cardputer — reinício imediato via flag; nunca reiniciar no callback
    if (tipo == CMD_REBOOT) {
        Serial.println(F("[AGUA] CMD_REBOOT recebido. Reiniciando em 1s..."));
        reboot_requisitado = true;
        return;
    }
}

// -------------------------------------------------------------
// ENVIO DE TELEMETRIA — PacketTelemetriaAgua (0x20) → Bomba
// -------------------------------------------------------------
void enviar_telemetria(float distancia_cm, bool erro_sensor, bool ladrao, uint8_t motivo) {
    PacketTelemetriaAgua pkt = {};
    pkt.tipo          = PKT_TELEMETRIA_AGUA;
    pkt.distancia_cm  = erro_sensor ? 0.0f : distancia_cm;
    pkt.erro_sensor   = erro_sensor;
    pkt.ladrao_ativo  = ladrao;
    pkt.motivo_status = motivo;  // 2026-07-25 — informativo, não influencia nenhuma decisão
    esp_now_send(MAC_BOMBA, (uint8_t*)&pkt, sizeof(pkt));
}

// Determina o motivo_status atual a partir do estado do módulo — usado no
// envio emergencial de ladrão (linha ~608), que não está dentro do bloco de
// decisão principal e não conhece o motivo específico do ciclo em curso.
static uint8_t motivo_status_atual() {
    if (falhas_sensor >= MAX_FALHAS_SENSOR)      return MOTIVO_SEM_ECO;
    if (aguardando_reconfirmacao || reflexao_bloqueada) return MOTIVO_CONDENSACAO_TIMEOUT;
    if (instabilidade_ativa)                     return MOTIVO_INSTABILIDADE;
    if (modo_reflexao)                           return MOTIVO_COMPENSANDO;
    return MOTIVO_OK;
}

// -------------------------------------------------------------
// OTA
// -------------------------------------------------------------
void iniciar_ota() {
    if (WiFi.status() == WL_CONNECTED) return;

    ota_requisitado = false;  // limpa flag aqui — responsabilidade da função, não do loop
    setCpuFrequencyMhz(240);

    ota_ativo  = true;
    inicio_ota = millis();

    Serial.println(F("[AGUA] OTA: desinicializando ESP-NOW..."));
    esp_now_deinit();
    espnow_ok = false;

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(300);
    esp_task_wdt_reset();

    WiFi.mode(WIFI_STA);
    WiFi.config(IP_AGUA, IP_GATEWAY, IP_MASCARA, IP_DNS);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print(F("[AGUA] OTA: conectando ao Wi-Fi"));
    uint8_t t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 40) {
        delay(500); Serial.print('.'); t++;
        esp_task_wdt_reset();
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[AGUA] OTA: falha Wi-Fi. Retomando ESP-NOW."));
        ota_ativo = false;
        setCpuFrequencyMhz(80);
        espnow_ok = iniciar_espnow();
        return;
    }

    Serial.printf("[AGUA] OTA: Wi-Fi OK. IP: %s\n",
                  WiFi.localIP().toString().c_str());

    ArduinoOTA.setHostname("caixa_agua");
    ArduinoOTA.onStart([]() {
        Serial.println(F("[AGUA] OTA: gravacao iniciada."));
        esp_task_wdt_reset();
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        esp_task_wdt_reset();
        static uint8_t ultimo_pct = 255;
        uint8_t pct = (progress * 100) / total;
        if (pct != ultimo_pct && pct % 10 == 0) {
            Serial.printf("[AGUA] OTA: %d%%\n", pct);
            ultimo_pct = pct;
        }
    });
    ArduinoOTA.onEnd([]() {
        Serial.println(F("[AGUA] OTA: concluido. Reiniciando..."));
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("[AGUA] OTA: erro %u\n", e);
    });
    ArduinoOTA.begin();

    Serial.printf("[AGUA] OTA: janela aberta por %lu min.\n",
                  JANELA_OTA_MS / 60000UL);
}

void encerrar_ota() {
    ota_ativo = false;
    ArduinoOTA.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);

    setCpuFrequencyMhz(80);

    janela_reset();
    falhas_sensor = 0;

    Serial.println(F("[AGUA] OTA: encerrado. Retomando ESP-NOW."));
    espnow_ok = iniciar_espnow();
}

// -------------------------------------------------------------
// SETUP
// -------------------------------------------------------------
void setup() {
    setCpuFrequencyMhz(80);

    Serial.begin(115200);
    delay(500);

    pinMode(PIN_TRIG,   OUTPUT);
    pinMode(PIN_ECHO,   INPUT);
    pinMode(PIN_LADRAO, INPUT_PULLUP);  // NPN: LOW = ladrão ativo
    digitalWrite(PIN_TRIG, LOW);

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    Serial.println(F("========================================="));
    Serial.println(F(" CAIXA AGUA - PRODUCAO MARICA v2.4"));
    Serial.printf(" MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  MAC_PROPRIA[0], MAC_PROPRIA[1], MAC_PROPRIA[2],
                  MAC_PROPRIA[3], MAC_PROPRIA[4], MAC_PROPRIA[5]);
    Serial.println(F(" Sensor: AJ-SR04M-2 | Trig:GPIO17 Echo:GPIO16"));
    Serial.printf(" Filtro: N=%d dOff=%.1f dOn=%.1f MaxRej=%d Conf=%d Plaus=%.1f\n",
                  FILTRO_N, FILTRO_DELTA_OFF, FILTRO_DELTA_ON,
                  FILTRO_MAX_REJEICOES, FILTRO_CONFIRMA_RESET, FILTRO_DELTA_PLAUSIVEL);
    Serial.printf(" Reflexao: ConfirmaN=%d TimeoutMin=%lu\n",
                  REFLEXAO_CONFIRMA_N, REFLEXAO_TIMEOUT_MS / 60000UL);
    Serial.printf(" Canal fixo: %d | OTA: %s\n",
                  CANAL_SEGURANCA_PADRAO, IP_AGUA.toString().c_str());
    Serial.println(F("========================================="));

    espnow_ok = iniciar_espnow();
    if (!espnow_ok)
        Serial.println(F("[AGUA] ESP-NOW falhou no boot. Retry em 10s."));
}

// -------------------------------------------------------------
// LOOP
// -------------------------------------------------------------
void loop() {
    esp_task_wdt_reset();

    // OTA — sinalizado pelo callback ou pelo serial (bancada)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'o' || c == 'O') {
            Serial.println(F("[AGUA] Gatilho OTA serial recebido."));
            ota_requisitado = true;
        }
    }

    if (ota_requisitado && !ota_ativo) {
        iniciar_ota();
    }

    // Reboot remoto — solicitado pelo Cardputer via CMD_REBOOT
    // Executado no loop() para garantir contexto seguro (fora de callbacks)
    if (reboot_requisitado) {
        Serial.println(F("[AGUA] Executando reboot remoto..."));
        delay(200);  // tempo para serial fluir
        ESP.restart();
    }

    if (ota_ativo) {
        ArduinoOTA.handle();
        if (millis() - inicio_ota >= JANELA_OTA_MS) encerrar_ota();
        delay(10);
        return;
    }

    // Recuperação de ESP-NOW em caso de falha de inicialização
    if (!espnow_ok) {
        static uint32_t ultima_tentativa = 0;
        if (millis() - ultima_tentativa >= ESPNOW_RETRY_MS) {
            ultima_tentativa = millis();
            Serial.println(F("[AGUA] Tentando reiniciar ESP-NOW..."));
            espnow_ok = iniciar_espnow();
        }
        delay(10);
        return;
    }

    // Statics do loop
    static uint32_t ultimo_envio = 0;

    // -------------------------------------------------------------
    // SENSOR DO LADRÃO — confirmação anti-falso-positivo
    // 3 leituras consecutivas com intervalo de 1s = 3s de confirmação
    // Ignora respingos e vibração mecânica no cano do ladrão
    // -------------------------------------------------------------
    {
        static uint8_t  confirma_ladrao     = 0;
        static bool     ultimo_ladrao       = false;
        static uint32_t ultimo_ladrao_ms    = 0;

        bool leitura_ladrao = (digitalRead(PIN_LADRAO) == LOW);

        if (leitura_ladrao) {
            // Pino ativo — avança confirmação a cada LADRAO_INTERVALO_MS
            if (millis() - ultimo_ladrao_ms >= LADRAO_INTERVALO_MS) {
                ultimo_ladrao_ms = millis();
                if (confirma_ladrao < LADRAO_CONFIRMA_N) confirma_ladrao++;
            }
        } else {
            // Pino inativo — limpa confirmação e ancora timestamp
            confirma_ladrao  = 0;
            ultimo_ladrao_ms = millis();  // ancora: evita disparo imediato após longa pausa
        }

        bool ladrao_confirmado = (confirma_ladrao >= LADRAO_CONFIRMA_N);

        // Disparo imediato se estado confirmado mudou
        if (ladrao_confirmado != ultimo_ladrao) {
            ultimo_ladrao = ladrao_confirmado;
            ladrao_ativo  = ladrao_confirmado;
            Serial.printf("[AGUA] %s ladrao confirmado! Despachando emergencia.\n",
                          ladrao_ativo ? "TRANSBORDO" : "Ladrao liberado.");
            // Envia imediatamente com estado atualizado — não espera ciclo de 5s
            enviar_telemetria(ultima_valida, (falhas_sensor >= MAX_FALHAS_SENSOR), ladrao_ativo,
                               motivo_status_atual());
            ultimo_envio = millis();  // reseta timer para evitar duplo envio imediato
        }
    }

    // -------------------------------------------------------------
    // Leitura e envio periódico com heartbeat garantido
    // Transmite sempre a cada ciclo para nunca atingir o watchdog de 60s da Bomba.
    // Se o filtro estiver convergindo/rejeitando, envia o último valor estável.
    // -------------------------------------------------------------
    if (millis() - ultimo_envio >= INTERVALO_ENVIO_MS) {
        ultimo_envio = millis();

        float bruto = ler_sensor();

        // -------------------------------------------------------------
        // PRÉ-PROCESSADOR: COMPENSAÇÃO DE MULTICAMINHO ACÚSTICO
        // Detecta condensação no transdutor pela assinatura matemática
        // do eco secundário: leitura bruta ≈ 2× a última distância real.
        //
        // Física: condensação bloqueia eco primário. A onda percorre
        // 4× a distância real (água→teto→água→sensor), resultando em
        // leitura = 4d ÷ 2 (divisão interna do sensor) = 2d.
        //
        // Enquanto modo_reflexao=true, bruto é dividido por 2 antes
        // de entrar no filtro — telemetria contínua mesmo com sensor cego.
        // -------------------------------------------------------------
        if (bruto > 0.0f) {
            // ---------------------------------------------------------
            // DETECÇÃO DE CONDENSAÇÃO NO BOOT / PÓS-OTA
            // Ponto cego do v2.2: com ultima_valida=999.0f (sentinela),
            // o pré-processador normal não tem referência para comparar.
            // Se a primeira leitura cai na faixa do eco secundário esperado
            // (≈ 2×NIVEL_CHEIO_REFERENCIA_CM ±10 cm), ativa modo_reflexao
            // imediatamente antes de inicializar o filtro.
            // Aplica-se tanto ao boot físico quanto ao pós-OTA, pois ambos
            // reiniciam ultima_valida para 999.0f.
            // ---------------------------------------------------------
            if (ultima_valida > 400.0f && !modo_reflexao) {
                float eco_esperado  = NIVEL_CHEIO_REFERENCIA_CM * 2.0f;  // 74 cm
                float desvio_boot   = fabsf(bruto - eco_esperado);
                if (desvio_boot <= 4.0f) {
                    modo_reflexao = true;
                    Serial.printf("[AGUA] CONDENSACAO no boot! Eco 2x detectado."
                                  " Bruto:%.1f EcoEsperado:%.1f Real estimado:%.1f\n",
                                  bruto, eco_esperado, bruto / 2.0f);
                }
            }

            // ---------------------------------------------------------
            // PRÉ-PROCESSADOR NORMAL (operação contínua)
            // v2.4: entrada/saída exigem REFLEXAO_CONFIRMA_N leituras
            // consecutivas. reflexao_bloqueada (pós-timeout) impede
            // reentrada automática até o eco primário reconfirmar.
            // ---------------------------------------------------------
            if (ultima_valida <= 400.0f && !reflexao_bloqueada) {
                if (!modo_reflexao) {
                    // Entrada: salto > 15 cm E bruto ≈ 2 × ultima_valida (±10 cm)
                    float desvio_dobro = fabsf(bruto - (ultima_valida * 2.0f));
                    bool condicao_entrada = ((bruto - ultima_valida) > 15.0f && desvio_dobro <= 10.0f);
                    if (condicao_entrada) {
                        confirma_reflexao_entra++;
                        Serial.printf("[AGUA] Possivel condensacao (%d/%d). Bruto:%.1f Real estimado:%.1f\n",
                                      confirma_reflexao_entra, REFLEXAO_CONFIRMA_N, bruto, ultima_valida);
                        if (confirma_reflexao_entra >= REFLEXAO_CONFIRMA_N) {
                            modo_reflexao        = true;
                            modo_reflexao_inicio = millis();
                            confirma_reflexao_entra = 0;
                            Serial.printf("[AGUA] CONDENSACAO confirmada! Eco 2x ativo."
                                          " Bruto:%.1f Real estimado:%.1f\n",
                                          bruto, ultima_valida);
                        }
                    } else {
                        confirma_reflexao_entra = 0;
                    }
                } else {
                    // Já em modo_reflexao — checa timeout de condensação persistente primeiro
                    if (millis() - modo_reflexao_inicio >= REFLEXAO_TIMEOUT_MS) {
                        Serial.printf("[AGUA] CONDENSACAO persistente ha %lu min. Timeout — "
                                      "tratando como falha de sensor, nao mais como correcao.\n",
                                      REFLEXAO_TIMEOUT_MS / 60000UL);
                        modo_reflexao             = false;
                        reflexao_bloqueada        = true;
                        aguardando_reconfirmacao  = true;
                        confirma_reflexao_entra   = 0;
                        confirma_reflexao_sai     = 0;
                        janela_reset();
                    } else {
                        // Saída: condensação dissipada — bruto volta à proximidade de ultima_valida
                        float desvio_real = fabsf(bruto - ultima_valida);
                        if (desvio_real <= 10.0f) {
                            confirma_reflexao_sai++;
                            Serial.printf("[AGUA] Possivel dissipacao (%d/%d). Bruto:%.1f\n",
                                          confirma_reflexao_sai, REFLEXAO_CONFIRMA_N, bruto);
                            if (confirma_reflexao_sai >= REFLEXAO_CONFIRMA_N) {
                                modo_reflexao = false;
                                confirma_reflexao_sai = 0;
                                Serial.printf("[AGUA] CONDENSACAO dissipada confirmada. Eco primario"
                                              " restaurado. Bruto:%.1f\n", bruto);
                            }
                        } else {
                            confirma_reflexao_sai = 0;
                        }
                    }
                }
            }

            // Aplica correção geométrica se em modo reflexão (boot ou normal)
            if (modo_reflexao) {
                bruto = bruto / 2.0f;
            }
        }

        if (bruto < 0.0f) {
            falhas_sensor++;
            Serial.printf("[AGUA] Sensor: sem leitura (%d/%d)\n",
                          falhas_sensor, MAX_FALHAS_SENSOR);
            if (falhas_sensor >= MAX_FALHAS_SENSOR) {
                Serial.println(F("[AGUA] Falha persistente. Enviando erro de hardware."));
                enviar_telemetria(0.0f, true, ladrao_ativo, MOTIVO_SEM_ECO);
            }
        } else {
            falhas_sensor = 0;

            // Enquanto reflexao_bloqueada, exclui do filtro qualquer leitura que
            // ainda tenha a assinatura do eco dobrado (≈2×ultima_valida) — mesmo
            // que ela caiba dentro de FILTRO_DELTA_PLAUSIVEL por coincidência
            // numérica (pode acontecer quando o nível real é baixo o bastante
            // para que o dobro caia perto do limiar). Sem isso, o filtro poderia
            // reconfirmar um valor ainda comprometido como se fosse válido.
            bool ainda_parece_dobrado = false;
            if (reflexao_bloqueada) {
                float desvio_dobro_bloq = fabsf(bruto - (ultima_valida * 2.0f));
                ainda_parece_dobrado = (desvio_dobro_bloq <= 10.0f);
            }

            if (ainda_parece_dobrado) {
                Serial.printf("[AGUA] Pos-timeout: leitura ainda parece dobrada (Bruto:%.1f)."
                              " Aguardando eco primario genuino.\n", bruto);
                enviar_telemetria(0.0f, true, ladrao_ativo, MOTIVO_CONDENSACAO_TIMEOUT);
            } else {
                float filtrado = filtrar(bruto);

                if (filtrado >= 0.0f) {
                    ultima_valida = filtrado;
                    // Recuperação confirmada: eco primário reconfirmou plausibilidade
                    // pós-timeout — libera reentrada normal de modo_reflexao.
                    if (aguardando_reconfirmacao || reflexao_bloqueada) {
                        Serial.println(F("[AGUA] Leitura reconfirmada pos-timeout. Retomando operacao normal."));
                    }
                    aguardando_reconfirmacao = false;
                    reflexao_bloqueada       = false;
                    // v2.5 — leitura válida confirmada encerra qualquer episódio de
                    // instabilidade em andamento (recuperação real, não só reset de janela).
                    instabilidade_ativa  = false;
                    instabilidade_inicio = 0;
                    Serial.printf("[AGUA] Bruta:%.1f cm -> Filtrada:%.1f cm\n",
                                  bruto, filtrado);
                    enviar_telemetria(filtrado, false, ladrao_ativo,
                                       modo_reflexao ? MOTIVO_COMPENSANDO : MOTIVO_OK);
                } else if (aguardando_reconfirmacao) {
                    // Pós-timeout, ainda sem leitura nova confirmada — reporta falha
                    // pelo caminho já existente (mesma semântica de MAX_FALHAS_SENSOR),
                    // sem qualquer mudança de protocolo, Supabase ou dashboard.
                    Serial.println(F("[AGUA] Aguardando reconfirmacao pos-condensacao. Reportando falha de sensor."));
                    enviar_telemetria(0.0f, true, ladrao_ativo, MOTIVO_CONDENSACAO_TIMEOUT);
                } else {
                    // v2.5 — timeout de instabilidade genérica (achado da revisão de
                    // segurança 2026-07-25). Marca o início do episódio na primeira
                    // vez que cai aqui; só ESCALA para erro_sensor=true se a bomba
                    // estiver ligada — parada, instabilidade prolongada não é urgente.
                    if (!instabilidade_ativa) {
                        instabilidade_ativa  = true;
                        instabilidade_inicio = millis();
                    }

                    if (bomba_ligada && (millis() - instabilidade_inicio >= INSTABILIDADE_TIMEOUT_MS)) {
                        Serial.printf("[AGUA] Filtro instavel ha %lus com bomba ligada. "
                                      "Reportando falha de sensor (nao confia mais em heartbeat).\n",
                                      (millis() - instabilidade_inicio) / 1000UL);
                        enviar_telemetria(0.0f, true, ladrao_ativo, MOTIVO_INSTABILIDADE);
                    } else {
                        Serial.printf("[AGUA] Filtro instavel. Heartbeat: %.1f cm\n", ultima_valida);
                        enviar_telemetria(ultima_valida, false, ladrao_ativo, MOTIVO_INSTABILIDADE);
                    }
                }
            }
        }
    }

    delay(10);
}
