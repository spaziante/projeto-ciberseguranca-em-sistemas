/* ============================================================================
 *  GuardGás — ASSISTENTE DE CALIBRAÇÃO
 *  Mesmas ligações do protótipo: MQ-2 AO -> divisor 10k/20k -> GPIO34.
 *
 *  Este programa NÃO faz alarme. Ele só mede e sugere os limiares.
 *  A leitura usa a mesma média móvel do protótipo (10 amostras a cada 200 ms),
 *  então os números servem direto no guardgas_blynk.ino (firmware final).
 *
 *  Como usar (Monitor Serial em 115200; digite a letra e aperte Enter):
 *    1) Deixe o sensor aquecer de 5 a 10 minutos (o tempo aparece na tela).
 *    2) Com ar limpo, envie  b  -> mede o BASE por 60 s.
 *    3) Envie  p  e, durante os 60 s seguintes, exponha o sensor a um estímulo
 *       seguro (fósforo recém-apagado a ~10 cm, ou gás de isqueiro APAGADO a
 *       15-20 cm, em local ventilado). Mede o PICO.
 *    4) O programa mostra os três limiares prontos para colar no firmware final.
 *    r  reinicia as medições.  NUNCA use chama perto do sensor.
 * ========================================================================== */

const int PIN_MQ2 = 34;

const unsigned long INTERVALO_MS = 200;
const unsigned long DURACAO_MEDIDA_MS = 60000;
const unsigned long AQUECIMENTO_MIN_MS = 5UL * 60UL * 1000UL;
const int AMOSTRAS_MEDIA = 10;

int  buf[AMOSTRAS_MEDIA];
int  idx = 0, qtd = 0;
long soma = 0;

enum Modo { OCIOSO, MEDINDO_BASE, MEDINDO_PICO };
Modo modo = OCIOSO;
unsigned long tInicioMedida = 0, tUltimaLeitura = 0, tUltimoPrint = 0, tUltimoAquec = 0;

// Resultados
bool temBase = false, temPico = false;
float base = 0, baseMin = 0, baseMax = 0, baseDesvio = 0;
int pico = 0;

// Acumuladores da medida em andamento
double somaMed = 0, somaQuad = 0;
long nMed = 0;
int minMed = 4095, maxMed = 0;

int lerMedia() {
  int v = analogRead(PIN_MQ2);
  soma -= buf[idx];
  buf[idx] = v;
  soma += v;
  idx = (idx + 1) % AMOSTRAS_MEDIA;
  if (qtd < AMOSTRAS_MEDIA) qtd++;
  return (int)(soma / qtd);
}

void ajuda() {
  Serial.println();
  Serial.println("=== GuardGas: assistente de calibracao ===");
  Serial.println("b = medir BASE (ar limpo, 60 s)   p = medir PICO (com estimulo seguro, 60 s)   r = reiniciar");
  Serial.println("Deixe o sensor aquecer de 5 a 10 minutos antes de medir.");
  Serial.println();
}

void iniciarMedida(Modo m) {
  modo = m;
  tInicioMedida = millis();
  somaMed = 0; somaQuad = 0; nMed = 0;
  minMed = 4095; maxMed = 0;
  if (m == MEDINDO_BASE) Serial.println(">> Medindo BASE por 60 s. Mantenha o ar limpo, sem nada perto do sensor...");
  else                   Serial.println(">> Medindo PICO por 60 s. Exponha o sensor ao estimulo seguro agora...");
}

void sugerirLimiares() {
  if (!(temBase && temPico)) return;
  float span = pico - base;
  Serial.println();
  Serial.printf("BASE = %.0f (min %.0f, max %.0f, desvio %.1f)   PICO = %d   SPAN = %.0f\n",
                base, baseMin, baseMax, baseDesvio, pico, span);
  if (span < 4 * baseDesvio + 100) {
    Serial.println("ATENCAO: o estimulo ficou fraco perto do ruido do ar limpo. Repita o PICO (p) chegando mais perto do sensor.");
    return;
  }
  int thDown  = (int)(base + 0.10f * span);
  int thAtenc = (int)(base + 0.25f * span);
  int thAlarm = (int)(base + 0.60f * span);
  Serial.println("Cole no topo de guardgas_blynk.ino:");
  Serial.printf("const int TH_ATENCAO_DOWN = %d;\n", thDown);
  Serial.printf("const int TH_ATENCAO_UP   = %d;\n", thAtenc);
  Serial.printf("const int TH_ALARME_UP    = %d;\n", thAlarm);
  if (thAtenc - thDown < 2 * baseDesvio)
    Serial.println("Aviso: a histerese (ATENCAO_UP - ATENCAO_DOWN) ficou pequena perto do ruido. Considere subir ATENCAO_UP.");
  Serial.println();
}

void finalizarMedida() {
  if (nMed == 0) { modo = OCIOSO; return; }
  if (modo == MEDINDO_BASE) {
    base = somaMed / nMed;
    baseMin = minMed; baseMax = maxMed;
    double var = somaQuad / nMed - (double)base * base;
    baseDesvio = var > 0 ? sqrt(var) : 0;
    temBase = true;
    Serial.printf(">> BASE concluida: media %.0f, min %d, max %d, desvio %.1f\n", base, minMed, maxMed, baseDesvio);
    Serial.println("   Agora envie p e exponha o sensor a um estimulo seguro.");
  } else {
    pico = maxMed;
    temPico = true;
    Serial.printf(">> PICO concluido: maximo %d\n", pico);
  }
  modo = OCIOSO;
  sugerirLimiares();
}

void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(PIN_MQ2, ADC_11db);
  ajuda();
}

void loop() {
  unsigned long agora = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') {
      if (agora < AQUECIMENTO_MIN_MS) Serial.println("Aviso: o sensor ainda esta aquecendo (menos de 5 min); a medida pode sair deslocada.");
      iniciarMedida(MEDINDO_BASE);
    } else if (c == 'p' || c == 'P') {
      iniciarMedida(MEDINDO_PICO);
    } else if (c == 'r' || c == 'R') {
      modo = OCIOSO; temBase = false; temPico = false;
      Serial.println(">> Medicoes reiniciadas.");
    }
  }

  if (agora - tUltimaLeitura >= INTERVALO_MS) {
    tUltimaLeitura = agora;
    int leitura = lerMedia();

    if (modo != OCIOSO) {
      somaMed += leitura; somaQuad += (double)leitura * leitura; nMed++;
      if (leitura < minMed) minMed = leitura;
      if (leitura > maxMed) maxMed = leitura;
      if (agora - tInicioMedida >= DURACAO_MEDIDA_MS) finalizarMedida();
    }

    if (agora - tUltimoPrint >= 1000) {
      tUltimoPrint = agora;
      if (modo == OCIOSO)
        Serial.printf("leitura=%4d   (ocioso, aquecimento: %lu min)\n", leitura, agora / 60000UL);
      else
        Serial.printf("leitura=%4d   medindo %s: %lu/60 s   max ate agora=%d\n", leitura,
                      modo == MEDINDO_BASE ? "BASE" : "PICO",
                      (agora - tInicioMedida) / 1000UL, maxMed);
    }
  }
}
