#pragma once
#include <Arduino.h>
// =============================================================
// PROJETO AUTOMACAO MARICA — Protocolo Compartilhado v2
// Canal fixo: 2 (CANAL_SEGURANCA_PADRAO)
// Fluxo: Água → Bomba (PKT_TELEMETRIA_AGUA)
//        Bomba → Controle (PKT_STATUS_COMPLETO)
//        Controle → Bomba (CMD_LIGA_BOMBA / CMD_DESLIGA_BOMBA)
//        Cardputer → Controle (CMD_WEB_SERVER)
//        Controle → Cardputer (PKT_STATUS_CARDPUTER, 2026-07-30 — modo Monitor)
// =============================================================

// -------------------------------------------------------------
// MACs OFICIAIS
// -------------------------------------------------------------
// Substitua pelos MACs reais das suas placas. Para descobrir o MAC de um
// ESP32: rode `Serial.println(WiFi.macAddress());` no setup() antes de
// iniciar o ESP-NOW, e leia pelo Monitor Serial.
static const uint8_t MAC_AGUA[]      = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x01}; // TODO: MAC da Caixa Agua
static const uint8_t MAC_BOMBA[]     = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x02}; // TODO: MAC da Caixa Bomba
static const uint8_t MAC_CONTROLE[]  = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x03}; // TODO: MAC da Caixa Controle
static const uint8_t MAC_CARDPUTER[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x04}; // TODO: MAC do Cardputer

// -------------------------------------------------------------
// RÁDIO
// -------------------------------------------------------------
#define CANAL_SEGURANCA_PADRAO 2  // Canal fixo ESP-NOW em todo o ecossistema

// -------------------------------------------------------------
// REDE WI-FI (Caixa Controle — janela OTA e ciclo internet)
// -------------------------------------------------------------
// WIFI_SSID e WIFI_PASS NAO ficam aqui -- vem de secrets.h (gitignored).
// Copie secrets.h.example para secrets.h e preencha com suas credenciais.
#include "secrets.h"

// Endereco IP fixo -- ajuste para a faixa da sua rede local
static const IPAddress IP_GATEWAY (192, 168, 1, 1);
static const IPAddress IP_MASCARA (255, 255, 255, 0);
static const IPAddress IP_DNS     (192, 168, 1, 1);
static const IPAddress IP_CONTROLE(192, 168, 1, 90);
static const IPAddress IP_BOMBA   (192, 168, 1, 91);
static const IPAddress IP_AGUA    (192, 168, 1, 92);

// -------------------------------------------------------------
// IDENTIFICADORES DE PACOTE
// -------------------------------------------------------------
enum TipoPacote : uint8_t {
    PKT_TELEMETRIA_AGUA  = 0x20,  // Água → Bomba
    PKT_STATUS_COMPLETO  = 0x35,  // Bomba → Controle
    CMD_LIGA_BOMBA       = 0xA1,  // Controle → Bomba
    CMD_DESLIGA_BOMBA    = 0xA2,  // Controle → Bomba
    CMD_PING_CONTROLE    = 0xA3,  // Controle → Bomba (keep-alive de presença, 1 byte)
    CMD_RESET_ERROS      = 0xA4,  // Controle → Bomba (limpa modo forçado e NVS)
    CMD_SET_NIVEIS       = 0xA5,  // Controle → Bomba (configura níveis e timeout)
    CMD_WEB_SERVER       = 0xA6,  // Cardputer → Controle (abre servidor web)
    CMD_OTA              = 0xB0,  // Cardputer → qualquer caixa
    CMD_REBOOT           = 0xB1,  // Cardputer → qualquer caixa (reboot imediato)
    PKT_STATUS_CARDPUTER = 0x40   // Controle → Cardputer (push periódico p/ modo Monitor,
                                   // 2026-07-30). Faixa 0x4x separada de 0x2x/0x3x de propósito --
                                   // não é telemetria entre caixas de força, é status já
                                   // processado (pct pronto) para um display de consumo.
};

// -------------------------------------------------------------
// BITMASK DE ERROS DA CAIXA BOMBA
// -------------------------------------------------------------
// ATENÇÃO: este campo é lido por decisões de segurança reais na Bomba
// (intertravamento 5 de ligar_bomba() e o desligamento por erro durante
// operação, ambos fazem `erros_ativos & ~ERRO_5_PZEM`) — nunca adicionar
// bits aqui que não devem ATIVAMENTE bloquear/desligar a bomba quando
// setados. Estados informativos (quarentena/bloqueio/modo_forçado) vão em
// BitmaskEstadoBomba, campo separado, propositalmente nunca lido por
// nenhuma decisão de segurança.
enum BitmaskErroBomba : uint8_t {
    ERRO_1_TIMEOUT = (1 << 0),  // Tempo máximo de operação atingido (60 min)
    ERRO_3_LADRAO  = (1 << 2),  // Transbordo detectado (GPIO 22 / XKC-Y26S-V)
    ERRO_5_PZEM    = (1 << 4)   // Falha crítica de comunicação Modbus RTU
};

// -------------------------------------------------------------
// BITMASK DE ESTADO DA CAIXA BOMBA — 2026-07-25 (dashboard, informativo)
// -------------------------------------------------------------
// Campo separado de BitmaskErroBomba de propósito (ver nota acima) — só
// EXPÕE estados que a Bomba já calcula e usa internamente para suas
// próprias decisões (em_quarentena/em_bloqueio/modo_forcado já existiam
// e já gate CMD_LIGA_BOMBA antes desta mudança); nunca é lido de volta
// por nenhuma lógica de decisão, só populado no envio do status.
enum BitmaskEstadoBomba : uint8_t {
    ESTADO_QUARENTENA   = (1 << 0),  // em_quarentena — 5min pós-boot, partidas bloqueadas
    ESTADO_BLOQUEIO     = (1 << 1),  // em_bloqueio — cooldown temporário pós-timeout
    ESTADO_MODO_FORCADO = (1 << 2),  // modo_forcado — 2 timeouts seguidos, travado até
                                      // reset manual via web (CMD_RESET_ERROS)
    ESTADO_RELE_COLADO  = (1 << 3)   // 2026-07-27 — falha do auto-teste pós-desligamento
                                      // (K1 ou K2 não abriu quando deveria, leitura PZEM fora
                                      // da margem com o outro relé isolado). Puramente
                                      // informativo — nunca lido por decisão de segurança da
                                      // Bomba, por construção da própria arquitetura de teste.
};

// -------------------------------------------------------------
// CÓDIGOS DE MOTIVO_STATUS (Caixa Água) — 2026-07-25
// -------------------------------------------------------------
// Detalha a CAUSA de agua_erro_sensor (hoje um booleano só) para exibição no
// dashboard. Não influencia nenhuma decisão de segurança -- é puramente
// informativo, um campo a mais nas structs já existentes. As decisões de
// segurança continuam 100% baseadas em erro_sensor/agua_erro_sensor, como já
// eram antes desta mudança.
enum MotivoStatusAgua : uint8_t {
    MOTIVO_OK                  = 0,  // Leitura normal, sem nenhuma condição especial
    MOTIVO_COMPENSANDO         = 1,  // modo_reflexao ativo, dentro dos 20min -- leitura
                                      // considerada válida (compensada), não é erro
    MOTIVO_SEM_ECO             = 2,  // Falha física do sensor (pulseIn sem retorno, 3x)
    MOTIVO_CONDENSACAO_TIMEOUT = 3,  // Condensação persistente >20min (pós-timeout v2.4)
    MOTIVO_INSTABILIDADE       = 4   // Filtro instável >3min com bomba ligada (v2.5)
};

// -------------------------------------------------------------
// CÓDIGOS DE MOTIVO DE DESLIGAMENTO DA BOMBA — 2026-07-25
// -------------------------------------------------------------
// Detalha a CAUSA do último desligar_bomba() -- hoje todos os 9 pontos de
// chamada (manual/nível cheio/segurança/nível zero/silêncio da Água/
// timeout/ladrão/erro físico genérico/OTA) resultavam no mesmo pacote,
// indistinguíveis para o dashboard. Puramente informativo -- não influencia
// nenhuma decisão de segurança, e propositalmente NÃO é gravado em
// bomba_erro_bitmask (mesmo motivo do BitmaskEstadoBomba: esse campo é lido
// por decisões de segurança reais, e nível_cheio/segurança são transitórios,
// não deveriam persistir como "erro ativo" entre ciclos).
enum CausaDesligamentoBomba : uint8_t {
    CAUSA_DESLIGA_MANUAL        = 0,  // botão físico (BTN2) ou CMD_DESLIGA_BOMBA (web/Controle)
    CAUSA_DESLIGA_NIVEL_CHEIO   = 1,  // nivel_desliga_cm atingido -- desligamento normal esperado
    CAUSA_DESLIGA_SEGURANCA     = 2,  // nivel_seguranca_cm excedido -- sensor anômalo/deslocado
    CAUSA_DESLIGA_NIVEL_ZERO    = 3,  // leitura zero (sentinela) persistente por 2min
    CAUSA_DESLIGA_SILENCIO_AGUA = 4,  // silêncio de rádio Água→Bomba >60s
    CAUSA_DESLIGA_TIMEOUT       = 5,  // tempo máximo de operação excedido (ERRO_1_TIMEOUT)
    CAUSA_DESLIGA_LADRAO        = 6,  // transbordo confirmado (ERRO_3_LADRAO)
    CAUSA_DESLIGA_ERRO_FISICO   = 7,  // catch-all: erros_ativos setado durante operação,
                                        // via caminho diferente dos específicos acima
    CAUSA_DESLIGA_OTA           = 8   // desligada por segurança para iniciar OTA
};

// -------------------------------------------------------------
// ESTRUTURAS DE PACOTE
// -------------------------------------------------------------

// Upstream primário: Água → Bomba
struct __attribute__((packed)) PacketTelemetriaAgua {
    uint8_t tipo;           // PKT_TELEMETRIA_AGUA (0x20)
    float   distancia_cm;   // Leitura filtrada do sensor ultrassônico (cm)
    bool    erro_sensor;    // true = timeout físico no pulseIn (sensor inválido)
    bool    ladrao_ativo;   // true = transbordo confirmado (GPIO 18 / XKC-Y26S-V)
    uint8_t motivo_status;  // MotivoStatusAgua -- 2026-07-25. Detalha a causa de
                            // erro_sensor para o dashboard. Campo adicionado no FINAL
                            // de propósito (mesmo padrão do agua_offline em
                            // PacketStatusCompleto) -- não desloca offsets existentes,
                            // truncamento seguro com firmware antigo durante o rollout.
};

// Upstream consolidado: Bomba → Controle
// A Bomba agrega telemetria da Água + seus próprios dados e envia este pacote
struct __attribute__((packed)) PacketStatusCompleto {
    uint8_t  tipo;               // PKT_STATUS_COMPLETO (0x35)
    float    agua_distancia_cm;  // Repassado da Caixa Água (filtrado)
    bool     agua_erro_sensor;   // Repassado da Caixa Água (falha sensor)
    bool     agua_ladrao_ativo;  // Repassado da Caixa Água (ladrão confirmado)
    bool     bomba_rele_estado;  // Estado físico do relé K1 (HIGH = ligado)
    uint8_t  bomba_erro_bitmask; // Bitmask de erros locais da Bomba
    uint32_t pzem_potencia_w;    // Potência instantânea PZEM-004T (Watts inteiros)
    float    pzem_tensao_v;      // Tensão instantânea PZEM-004T (V)
    float    pzem_corrente_a;    // Corrente instantânea PZEM-004T (A)
    float    pzem_fp;            // Fator de potência PZEM-004T (0.00–1.00)
    float    pzem_energia_kwh;   // Energia acumulada PZEM-004T (kWh, total do medidor)
    bool     agua_offline;       // Silêncio de rádio Água→Bomba >60s (SILENCIO_AGUA_MS) --
                                  // distinto de agua_erro_sensor (erro real reportado pela
                                  // própria Água). Campo adicionado no FINAL da struct de
                                  // propósito -- não desloca offsets dos campos existentes,
                                  // mantendo compatibilidade de truncamento seguro com
                                  // firmware antigo durante o rollout (Bomba primeiro).
    uint8_t  agua_motivo_status;  // Repassado de PacketTelemetriaAgua.motivo_status --
                                  // 2026-07-25. Também no final, mesmo motivo.
    uint8_t  bomba_estado_bitmask; // BitmaskEstadoBomba -- 2026-07-25. Campo NOVO e
                                  // SEPARADO de bomba_erro_bitmask (ver nota no enum) --
                                  // puramente informativo para o dashboard, nunca lido
                                  // por nenhuma decisão de segurança da Bomba.
    uint8_t  bomba_causa_desligamento; // CausaDesligamentoBomba -- 2026-07-25. Também
                                  // no final, mesmo motivo. Emissor desta rodada é a
                                  // própria Bomba (não a Água) -- ordem de flash só
                                  // Bomba -> Controle para este campo específico.
};

// Downstream: Controle → Bomba (3 bytes)
struct __attribute__((packed)) PacketComandoBomba {
    uint8_t tipo;              // CMD_LIGA_BOMBA (0xA1), CMD_DESLIGA_BOMBA (0xA2),
                               // CMD_PING_CONTROLE (0xA3), CMD_RESET_ERROS (0xA4)
    bool    horario_permitido; // true = entre 09:00 e 18:00 (horário de partidas automáticas)
    bool    ignorar_nivel;     // true = pula intertravamentos de nível 2/3 (nivel_manual_min_cm,
                               // nivel_desliga_cm) em ligar_bomba() -- NÃO pula o intertravamento 4
                               // (nivel_seguranca_cm/anomalia) nem 1/5 (sensor/erros físicos).
                               // Usado exclusivamente pelo botão "Ligar Bomba" da página web da
                               // Controle; BTN1 físico e lógica automática sempre mandam false.
};

// Configuração remota de parâmetros operacionais — Controle → Bomba (7 bytes)
struct __attribute__((packed)) PacketConfigNiveis {
    uint8_t  tipo;                 // CMD_SET_NIVEIS (0xA5)
    uint8_t  nivel_liga_cm;        // Nível para ligar a bomba (cm)
    uint8_t  nivel_desliga_cm;     // Nível para desligar a bomba (cm)
    uint16_t timeout_minutos;      // Tempo máximo de operação (minutos)
    uint8_t  nivel_seguranca_cm;   // Limite superior de segurança — desliga se ultrapassado
    uint8_t  nivel_manual_min_cm;  // Nível mínimo para partida manual e automática (cm)
};

// Comando OTA — Cardputer → qualquer caixa (1 byte)
struct __attribute__((packed)) PacketComandoOTA {
    uint8_t tipo;  // CMD_OTA (0xB0)
};

// Comando servidor web — Cardputer → Controle (1 byte)
struct __attribute__((packed)) PacketComandoWebServer {
    uint8_t tipo;  // CMD_WEB_SERVER (0xA6)
};

// Comando reboot — Cardputer → qualquer caixa (1 byte)
// Causa reinício imediato do ESP32 alvo via ESP.restart()
// Útil para forçar reconvergência do filtro sem acesso físico
struct __attribute__((packed)) PacketComandoReboot {
    uint8_t tipo;  // CMD_REBOOT (0xB1)
};

// -------------------------------------------------------------
// Status pronto para exibição — Controle → Cardputer (2026-07-30)
// -------------------------------------------------------------
// Push periódico (não sob demanda) para alimentar o modo Monitor do Cardputer
// (gráfico de nível fixado na porta da geladeira). Campos já vêm PRONTOS --
// nivel_pct é calculado pela própria Controle via calcular_pct(), mesma fonte
// única usada no servidor web local (marica-153: um único lugar calcula,
// todo o resto só exibe). O Cardputer NÃO reimplementa a conversão
// distância→percentual nem duplica CAIXA_CHEIA_CM/CAIXA_VAZIA_CM.
struct __attribute__((packed)) PacketStatusCardputer {
    uint8_t  tipo;                // PKT_STATUS_CARDPUTER (0x40)
    uint8_t  nivel_pct;           // 0-100, já calculado por calcular_pct() na Controle
    float    nivel_distancia_cm;  // distância bruta (cm) -- só para exibir "xx cm", mesmo
                                   // padrão do servidor web local (rota_raiz())
    bool     bomba_ligada;        // Repassado de bomba_ligada (Controle)
    uint8_t  modo_atual;          // ModoOperacao -- MODO_AUTOMATICO (1) / MODO_SEMIAUTOMATICO (2)
    bool     agua_erro_sensor;    // Repassado -- sensor da Água com falha física
    bool     agua_offline;        // Repassado -- silêncio de rádio Água→Bomba
    bool     bomba_offline;       // De bomba_esta_offline() -- mesma fonte única já usada em
                                   // registrar_telemetria() e loop_sinaleira(). Se true, o resto
                                   // do pacote é dado cacheado, não corrente -- Cardputer deve
                                   // tratar como indisponível, não exibir como se fosse atual.
};
