/* ============================================================================
 *  GuardGás — TESTE DE WI-FI (diagnóstico da conexão com o hotspot)
 *  Só valida a conexão. Não lê o MQ-2 e não faz alarme.
 *  Ligações: só os LEDs (verde GPIO16, vermelho GPIO17); o MQ-2 pode ficar ligado.
 *
 *  Antes de gravar, edite o secrets.h (mesma pasta) com o nome e a senha do
 *  hotspot do iPhone (2,4 GHz, "Maximizar Compatibilidade" ligado).
 *  Usa as mesmas regras de rede do firmware final (só WPA2 ou superior).
 *  Monitor Serial em 115200.
 *
 *  LED verde: aceso = conectado | piscando devagar = sem conexão.
 *  LED vermelho: não usado neste teste.
 *
 *  A conexão é não bloqueante: o loop() nunca fica preso esperando a rede.
 *  É a mesma ideia que será usada no firmware final, em que o alarme local
 *  não pode depender do Wi-Fi.
 * ========================================================================== */

#include <WiFi.h>
#include "secrets.h"
#include <esp_arduino_version.h>

const int PIN_LED_VERDE = 16;
const int PIN_LED_VERM  = 17;

const unsigned long TENTATIVA_MS = 15000;  // tempo máximo de cada tentativa
const unsigned long RELATORIO_MS = 5000;   // intervalo do relatório de status

volatile bool conectado = false;
unsigned long tInicioTentativa = 0, tRelatorio = 0, tPisca = 0;
unsigned long numTentativas = 0;
bool ledPisca = false;

const char* nomeMotivo(uint8_t r) {
  switch (r) {
    case 2:   return "autenticacao expirou";
    case 8:   return "desconectado pelo roteador/proprio";
    case 15:  return "handshake falhou (senha errada e a causa mais comum)";
    case 200: return "perdeu o sinal (beacon timeout)";
    case 201: return "rede nao encontrada (nome errado, 5 GHz ou fora de alcance)";
    case 202: return "falha de autenticacao (senha errada ou tipo de seguranca)";
    case 203: return "falha de associacao";
    case 204: return "handshake timeout (senha errada ou sinal fraco)";
    default:  return "outro motivo";
  }
}

void aoEvento(WiFiEvent_t evento, WiFiEventInfo_t info) {
  switch (evento) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[wifi] associado ao roteador, aguardando IP...");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      conectado = true;
      Serial.printf("[wifi] CONECTADO  IP=%s  sinal=%d dBm  canal=%d\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      conectado = false;
      Serial.printf("[wifi] sem conexao (motivo %d: %s)\n",
                    info.wifi_sta_disconnected.reason,
                    nomeMotivo(info.wifi_sta_disconnected.reason));
      break;
    default:
      break;
  }
}

void escanearRedes() {
  Serial.println("[wifi] procurando redes de 2,4 GHz ao redor...");
  int n = WiFi.scanNetworks();
  bool achou = false;
  for (int i = 0; i < n; i++) {
    bool alvo = (WiFi.SSID(i) == WIFI_SSID);
    if (alvo) achou = true;
    Serial.printf("  %s %-32s sinal=%4d dBm  canal=%2d\n",
                  alvo ? ">>" : "  ", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
  }
  if (n <= 0) Serial.println("  nenhuma rede encontrada (confira a antena u.FL do modulo -U).");
  else if (!achou) Serial.printf("  A rede \"%s\" NAO apareceu. Confira o nome, se e 2,4 GHz e a distancia.\n", WIFI_SSID);
  WiFi.scanDelete();
}

void iniciarConexao() {
  numTentativas++;
  tInicioTentativa = millis();
  Serial.printf("[wifi] tentativa %lu: conectando em \"%s\"...\n", numTentativas, WIFI_SSID);
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_VERM, OUTPUT);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_VERM, LOW);
  delay(500);

  Serial.println();
  Serial.println("=== GuardGas: teste de Wi-Fi ===");

  WiFi.persistent(false);            // nao grava a senha na flash do ESP32
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 3)
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);   // igual ao firmware final: recusa redes abertas
#endif
  WiFi.onEvent(aoEvento);
  Serial.printf("[wifi] MAC do ESP32: %s\n", WiFi.macAddress().c_str());

  escanearRedes();
  iniciarConexao();
}

void loop() {
  unsigned long agora = millis();

  // Reconexão: se a tentativa passou do tempo, tenta de novo.
  if (!conectado && agora - tInicioTentativa >= TENTATIVA_MS) iniciarConexao();

  // LED verde: aceso conectado, pisca devagar sem conexão.
  if (conectado) {
    digitalWrite(PIN_LED_VERDE, HIGH);
  } else if (agora - tPisca >= 500) {
    tPisca = agora;
    ledPisca = !ledPisca;
    digitalWrite(PIN_LED_VERDE, ledPisca);
  }

  // Relatório periódico.
  if (agora - tRelatorio >= RELATORIO_MS) {
    tRelatorio = agora;
    if (conectado)
      Serial.printf("[status] conectado  IP=%s  sinal=%d dBm\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else
      Serial.println("[status] sem conexao (o loop continua rodando normalmente)");
  }
}
