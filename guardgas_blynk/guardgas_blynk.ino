/* ============================================================================
 *  GuardGás — FIRMWARE FINAL (Blynk)  v2
 *  MQ-2 + LED verde + LED vermelho + Wi-Fi (hotspot do iPhone) + Blynk.
 *
 *  Ligações:
 *    MQ-2 VCC -> VIN (5 V)      MQ-2 GND -> GND
 *    MQ-2 AO  -> 10 kΩ -> GPIO34, e GPIO34 -> 20 kΩ -> GND  (divisor 5 V -> 3,3 V)
 *    GPIO16 -> LED verde    -> 220 Ω -> GND
 *    GPIO17 -> LED vermelho -> 220 Ω -> GND
 *    Antena externa no conector u.FL do módulo WROOM-32U.
 *
 *  LEDs:
 *    AQUECENDO -> verde piscando rápido (primeiro minuto)
 *    NORMAL    -> verde aceso
 *    ATENÇÃO   -> vermelho piscando devagar
 *    ALARME    -> vermelho aceso
 *    FALHA     -> verde e vermelho alternando (sensor solto, em curto ou travado)
 *
 *  Divisão do código (o ESP32 tem dois núcleos):
 *    - loop(), núcleo 1: sensor, estados e LEDs. Não depende da rede.
 *      Watchdog: se travar, a placa reinicia sozinha.
 *    - tarefaRede(), núcleo 0: Wi-Fi e Blynk. Só envia dados.
 *
 *  Defesas implementadas neste código:
 *    - Alarme local independente da rede (deauth, queda do hotspot ou do Blynk
 *      não apagam os LEDs).
 *    - Nenhum comando remoto é aceito. Qualquer escrita vinda do Blynk é
 *      descartada, gera o evento "comando_bloqueado" e o estado real é
 *      reenviado na hora. O estado também é reenviado a cada 30 s, então um
 *      valor falso no painel não dura.
 *    - Conexão com o Blynk por TLS (porta 443) com certificado validado.
 *    - Só conecta em redes WPA2 ou superior: um ponto de acesso falso aberto
 *      com o mesmo nome é recusado.
 *    - Senha do Wi-Fi não é gravada na flash pelo ESP32 (WiFi.persistent(false)).
 *    - Evento "reinicio" a cada religamento, com o motivo (energia, queda de
 *      tensão, watchdog, erro). Corte de energia e tentativa de regravar ou ler
 *      a placa exigem reiniciá-la, então ficam registrados.
 *    - Limite de eventos (1 por tipo a cada 60 s, 80 por dia) contra flood.
 *    - Sem OTA, sem comandos pela serial, limiares fixos no código.
 *    - MODO_DEPURACAO 0 tira do Monitor Serial o nome da rede, o IP e as
 *      mensagens da biblioteca.
 *
 *  Antes de gravar:
 *    1) Preencha o secrets.h (hotspot e token do Blynk).
 *    2) Cole abaixo os três limiares da calibração.
 *    3) Biblioteca "Blynk" (Volodymyr Shymanskyy) instalada.
 *    4) Para a apresentação, mude MODO_DEPURACAO para 0.
 *  Monitor Serial em 115200. NUNCA teste com chama perto do sensor.
 * ========================================================================== */

#define MODO_DEPURACAO 1        // 1 = testes (mostra rede e Blynk na serial) | 0 = apresentação

#include "secrets.h"            // PRIMEIRO: define BLYNK_TEMPLATE_ID antes da biblioteca
#if MODO_DEPURACAO
  #define BLYNK_PRINT Serial
  #define LOG(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(...) do {} while (0)
#endif
#define BLYNK_HEARTBEAT 30      // tráfego frequente: o iPhone derruba aparelhos parados por 90 s

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32_SSL.h>   // TLS na porta 443, certificado do servidor validado
#include <esp_system.h>
#include <esp_arduino_version.h>

// ---------------- Pinos ----------------
const int PIN_MQ2       = 34;   // AO do MQ-2 via divisor 10k/20k (ADC1, funciona com Wi-Fi)
const int PIN_LED_VERDE = 16;
const int PIN_LED_VERM  = 17;

// ---------------- Limiares (COLE AQUI OS VALORES DA CALIBRAÇÃO) ----------------
// Escala 0–4095. UP/DOWN = histerese.
const int TH_ATENCAO_UP   = 1500;   // NORMAL  -> ATENÇÃO
const int TH_ATENCAO_DOWN = 1200;   // ATENÇÃO -> NORMAL (e saída do ALARME)
const int TH_ALARME_UP    = 2400;   // ATENÇÃO -> ALARME

const unsigned long TEMPO_AQUECIMENTO_MS = 60000;  // 1 min
const unsigned long INTERVALO_LEITURA_MS = 200;
const unsigned long INTERVALO_SERIAL_MS  = 1000;
const int AMOSTRAS_MEDIA = 10;
const int N_CONFIRMA     = 5;    // leituras seguidas para mudar de estado
const int FALHA_MIN = 20, FALHA_MAX = 4080, N_FALHA = 25;

// ---------------- Rede ----------------
const unsigned long ENVIO_LEITURA_MS   = 30000;   // V0 periódico
const unsigned long TENTATIVA_WIFI_MS  = 15000;
const unsigned long TENTATIVA_BLYNK_MS = 10000;
const unsigned long TIMEOUT_BLYNK_MS   = 5000;
const unsigned long EVENTO_MIN_MS      = 60000;   // por tipo de evento
const unsigned long JANELA_DIA_MS      = 24UL * 60UL * 60UL * 1000UL;
const int           EVENTOS_MAX_DIA    = 80;      // o plano gratuito permite 100

// ---------------- Estados ----------------
enum Estado { AQUECENDO, NORMAL, ATENCAO, ALARME, FALHA };

// Compartilhados entre os dois núcleos. Só o loop() escreve; a rede só lê.
volatile Estado estado = AQUECENDO;
volatile int    leituraAtual = 0;

void diagnosticoFalha(int leitura);

int  bufMedia[AMOSTRAS_MEDIA];
int  idxMedia = 0, qtdMedia = 0;
long somaMedia = 0;
int  contConfirma = 0, contFalha = 0;
unsigned long tUltimaLeitura = 0, tUltimoSerial = 0;

const char* nomeEstado(Estado e) {
  switch (e) {
    case AQUECENDO: return "AQUECENDO";
    case NORMAL:    return "NORMAL";
    case ATENCAO:   return "ATENCAO";
    case ALARME:    return "ALARME";
    case FALHA:     return "FALHA";
  }
  return "?";
}

// ---------------- Leitura com média móvel ----------------
int lerSensor() {
  int v = analogRead(PIN_MQ2);
  somaMedia -= bufMedia[idxMedia];
  bufMedia[idxMedia] = v;
  somaMedia += v;
  idxMedia = (idxMedia + 1) % AMOSTRAS_MEDIA;
  if (qtdMedia < AMOSTRAS_MEDIA) qtdMedia++;
  return (int)(somaMedia / qtdMedia);
}

// Muda de estado só depois de N_CONFIRMA leituras seguidas pedindo a mudança.
void pedir(Estado destino) {
  if (++contConfirma >= N_CONFIRMA) {
    estado = destino;
    contConfirma = 0;
  }
}

// ---------------- Máquina de estados ----------------
void atualizarEstado(int leitura) {
  // Sensor com leitura impossível por muito tempo = FALHA
  if (leitura <= FALHA_MIN || leitura >= FALHA_MAX) {
    if (++contFalha >= N_FALHA && estado != ALARME) {
      if (estado != FALHA) diagnosticoFalha(leitura);
      estado = FALHA;
      contConfirma = 0;
    }
    return;
  }
  contFalha = 0;
  if (estado == FALHA) { estado = NORMAL; contConfirma = 0; }

  switch (estado) {
    case AQUECENDO:
      if (millis() >= TEMPO_AQUECIMENTO_MS) { estado = NORMAL; contConfirma = 0; }
      break;

    case NORMAL:
      if (leitura > TH_ATENCAO_UP) pedir(ATENCAO); else contConfirma = 0;
      break;

    case ATENCAO:
      if (leitura > TH_ALARME_UP)         pedir(ALARME);
      else if (leitura < TH_ATENCAO_DOWN) pedir(NORMAL);
      else contConfirma = 0;
      break;

    case ALARME:
      // Sai sozinho quando o ar limpa (confirmado por N leituras).
      if (leitura < TH_ATENCAO_DOWN) pedir(NORMAL); else contConfirma = 0;
      break;

    default: break;
  }
}

// ---------------- LEDs ----------------
void atualizarSaidas() {
  bool piscaRapido = (millis() / 150) % 2;
  bool piscaLento  = (millis() / 500) % 2;
  bool verde = false, vermelho = false;

  switch (estado) {
    case AQUECENDO: verde = piscaRapido;                        break;
    case NORMAL:    verde = true;                               break;
    case ATENCAO:   vermelho = piscaLento;                      break;
    case ALARME:    vermelho = true;                            break;
    case FALHA:     verde = piscaLento; vermelho = !piscaLento; break;
  }

  digitalWrite(PIN_LED_VERDE, verde);
  digitalWrite(PIN_LED_VERM, vermelho);
}

// ---------------- Teste de LEDs e diagnóstico ----------------
void testeLeds() {
  digitalWrite(PIN_LED_VERDE, HIGH); delay(400); digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_VERM,  HIGH); delay(400); digitalWrite(PIN_LED_VERM,  LOW);
  digitalWrite(PIN_LED_VERDE, HIGH); digitalWrite(PIN_LED_VERM, HIGH); delay(400);
  digitalWrite(PIN_LED_VERDE, LOW);  digitalWrite(PIN_LED_VERM, LOW);
}

void diagnosticoFalha(int leitura) {
  if (leitura <= FALHA_MIN)
    Serial.println("[DIAG] Leitura ~0: AO solto, GPIO34 sem sinal, GND do MQ-2 aberto ou divisor em curto.");
  else
    Serial.println("[DIAG] Leitura ~4095: AO ligado direto em 3,3 V, divisor ausente, ou fio no pino DO em vez do AO.");
}

// ============================================================================
//  REDE (núcleo 0): Wi-Fi + Blynk. Só envia; nunca obedece comandos.
// ============================================================================
enum TipoEvento { EV_ATENCAO, EV_ALARME, EV_FALHA, EV_REINICIO, EV_COMANDO, EV_TOTAL };
const char* const CODIGO_EVENTO[EV_TOTAL] = { "atencao", "alarme", "falha", "reinicio", "comando_bloqueado" };

unsigned long tUltimoEvento[EV_TOTAL];
bool          eventoJaEnviado[EV_TOTAL];
int           eventosNaJanela = 0;
unsigned long tInicioJanela = 0;

int           comandosBloqueados = 0;   // tentativas de escrita vindas do Blynk (mesma tarefa da rede)
int           comandosAvisados = 0;

const char* motivoReinicio() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "ligado ou botao EN";
    case ESP_RST_BROWNOUT: return "queda de tensao";
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT:      return "watchdog (travamento)";
    case ESP_RST_PANIC:    return "erro de firmware";
    case ESP_RST_SW:       return "reinicio por software";
    case ESP_RST_EXT:      return "reset externo";
    default:               return "outro";
  }
}

void enviarEvento(TipoEvento tipo, const char* descricao) {
  unsigned long agora = millis();
  if (agora - tInicioJanela >= JANELA_DIA_MS) { tInicioJanela = agora; eventosNaJanela = 0; }

  if (eventosNaJanela >= EVENTOS_MAX_DIA) {
    LOG("[rede] limite diario de eventos atingido; evento %s nao enviado.\n", CODIGO_EVENTO[tipo]);
    return;
  }
  if (eventoJaEnviado[tipo] && agora - tUltimoEvento[tipo] < EVENTO_MIN_MS) {
    LOG("[rede] evento %s ignorado (menos de 60 s desde o ultimo).\n", CODIGO_EVENTO[tipo]);
    return;
  }

  Blynk.logEvent(CODIGO_EVENTO[tipo], descricao);
  eventoJaEnviado[tipo] = true;
  tUltimoEvento[tipo] = agora;
  eventosNaJanela++;
  LOG("[rede] evento enviado: %s (%s)\n", CODIGO_EVENTO[tipo], descricao);
}

void notificarEstado(Estado e, int leitura) {
  char descricao[40];
  snprintf(descricao, sizeof(descricao), "Leitura %d", leitura);
  switch (e) {
    case ATENCAO: enviarEvento(EV_ATENCAO, descricao); break;
    case ALARME:  enviarEvento(EV_ALARME,  descricao); break;
    case FALHA:   enviarEvento(EV_FALHA,   descricao); break;
    default: break;
  }
}

// Qualquer escrita vinda do Blynk (app, painel, API) cai aqui e é descartada.
BLYNK_WRITE_DEFAULT() {
  (void)request;
  comandosBloqueados++;
  LOG("[seguranca] escrita recebida no pino V%d foi ignorada.\n", (int)request.pin);
}

void tarefaRede(void* parametro) {
  (void)parametro;
  WiFi.persistent(false);          // não grava a senha do Wi-Fi na flash
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 3)
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);   // recusa redes abertas, WEP e WPA1
#endif
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
  Blynk.config(BLYNK_AUTH_TOKEN);  // servidor padrão blynk.cloud, porta 443 com TLS

  unsigned long tTentativaWifi = millis();
  unsigned long tTentativaBlynk = 0, tUltimoEnvio = 0;
  int  estadoEnviado = -1;          // -1 = nada enviado nesta conexão
  bool wifiOk = false;
  bool reinicioAvisado = false;

  for (;;) {
    unsigned long agora = millis();

    if (WiFi.status() != WL_CONNECTED) {
      if (wifiOk) { LOG("[rede] Wi-Fi caiu; o alarme local continua funcionando.\n"); wifiOk = false; }
      if (agora - tTentativaWifi >= TENTATIVA_WIFI_MS) {
        tTentativaWifi = agora;
        LOG("[rede] conectando ao Wi-Fi \"%s\"...\n", WIFI_SSID);
        WiFi.disconnect(false);
        WiFi.begin(WIFI_SSID, WIFI_SENHA);
      }
    } else {
      if (!wifiOk) {
        wifiOk = true;
        LOG("[rede] Wi-Fi conectado: IP=%s sinal=%d dBm\n",
            WiFi.localIP().toString().c_str(), WiFi.RSSI());
      }

      if (!Blynk.connected()) {
        if (agora - tTentativaBlynk >= TENTATIVA_BLYNK_MS) {
          tTentativaBlynk = agora;
          LOG("[rede] conectando ao Blynk (TLS)...\n");
          if (Blynk.connect(TIMEOUT_BLYNK_MS)) {
            LOG("[rede] Blynk conectado.\n");
            estadoEnviado = -1;     // reenvia o estado atual (e avisa se estiver em alerta)
          } else {
            LOG("[rede] falha ao conectar no Blynk; nova tentativa em 10 s.\n");
          }
        }
      } else {
        Blynk.run();

        if (!reinicioAvisado) {     // uma vez por religamento
          char descricao[48];
          snprintf(descricao, sizeof(descricao), "Motivo: %s", motivoReinicio());
          enviarEvento(EV_REINICIO, descricao);
          reinicioAvisado = true;
        }

        Estado e = estado;          // cópia: o loop() pode mudar a qualquer momento
        int    l = leituraAtual;

        int bloqueados = comandosBloqueados;
        if (bloqueados != comandosAvisados) {
          char descricao[48];
          snprintf(descricao, sizeof(descricao), "%d tentativa(s) de comando ignorada(s)", bloqueados);
          enviarEvento(EV_COMANDO, descricao);
          comandosAvisados = bloqueados;
          // Alguém tentou mudar um valor: sobrescreve na hora com o estado real.
          Blynk.virtualWrite(V1, nomeEstado(e));
          Blynk.virtualWrite(V0, l);
        }

        if ((int)e != estadoEnviado) {
          Blynk.virtualWrite(V1, nomeEstado(e));
          Blynk.virtualWrite(V0, l);
          notificarEstado(e, l);
          estadoEnviado = (int)e;
          tUltimoEnvio = agora;
        } else if (agora - tUltimoEnvio >= ENVIO_LEITURA_MS) {
          // Reenvia também o estado: um valor falso no painel dura no máximo 30 s.
          Blynk.virtualWrite(V1, nomeEstado(e));
          Blynk.virtualWrite(V0, l);
          tUltimoEnvio = agora;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================================
//  SETUP / LOOP (núcleo 1): detecção e alarme local
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_VERM, OUTPUT);
  analogSetPinAttenuation(PIN_MQ2, ADC_11db);   // leitura até ~3,3 V
  testeLeds();
  Serial.printf("[GuardGas] Iniciado (motivo: %s). Aquecendo o MQ-2 por 1 minuto...\n", motivoReinicio());

  // Rede em outra tarefa, no núcleo 0, para nunca travar a detecção.
  xTaskCreatePinnedToCore(tarefaRede, "rede", 12288, NULL, 1, NULL, 0);

  enableLoopWDT();   // watchdog: se o loop() travar, a placa reinicia
}

void loop() {
  unsigned long agora = millis();
  if (agora - tUltimaLeitura >= INTERVALO_LEITURA_MS) {
    tUltimaLeitura = agora;
    int leitura = lerSensor();
    leituraAtual = leitura;
    atualizarEstado(leitura);
    if (agora - tUltimoSerial >= INTERVALO_SERIAL_MS) {
      tUltimoSerial = agora;
      Serial.printf("leitura=%4d  estado=%s\n", leitura, nomeEstado(estado));
    }
  }
  atualizarSaidas();   // fora do intervalo para o pisca ficar suave
}
