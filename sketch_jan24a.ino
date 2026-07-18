#include <Arduino.h>
#include <Adafruit_GFX.h>    // Biblioteca gráfica base
#include <Adafruit_ST7735.h> // Biblioteca do display ST7735
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- MAPEAMENTO DE PINOS ESP32-C3 ---
#define TFT_CS     2  // Ligado no pino CS do display
#define TFT_DC     7  // Ligado no pino A0 do display
#define TFT_RST   -1  // Sem pino digital! Ligue o RESET do display direto no 3.3V

// Pinos SPI Fixos do Hardware do ESP32-C3:
// SCK  -> GPIO 4 (Ligado no pino SCK do display)
// MOSI -> GPIO 6 (Ligado no pino SDA do display)

#define ONE_WIRE_BUS 5  // Sensor de temperatura
#define POT_PIN      0  // Potenciômetro
#define BTN_PIN      1  // Botão de comando
#define HEATER_PIN   3  // Saída do Aquecedor (Transistor/Mosfet/Relé)
#define LED_PIN      8  // LED indicador interno da placa

// --- DEFINIÇÃO DE CORES (RGB565) ---
#define ST7735_BLACK   0x0000
#define ST7735_WHITE   0xFFFF
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_BLUE    0x001F
#define ST7735_YELLOW  0xFFE0
#define ST7735_GRAY    0x8410

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// --- ESTADOS DO SISTEMA ---
enum ScreenState { SELECT_TIME, SELECT_POWER, SELECT_TEMP, CONTROL_MODE };
ScreenState currentScreen = SELECT_TIME;

bool showGraphicView = false; // Alterna entre a visualização de texto e de gráficos

unsigned long totalTime_ms = 0;
int maxPowerPercent = 10; 
float tempRefConfig = 40.0; 
unsigned long startTime_ms = 0;
unsigned long lastBtnTime = 0;
bool lastBtnState = HIGH;

// Variáveis para o ajuste acumulador nos extremos do POT
unsigned long lastIncrementTime = 0;
unsigned long accumulatedTime_ms = 0;
float tempOffset = 0.0; 

// Parâmetros PI
float Kp = 0.5;
float Ki = 0.01;
float Ts = 1.0;
float uPI = 0.0;
float e_prev = 0.0;
int dutySteps = 0; 

const unsigned long windowMs = 5000; 
unsigned long windowStartMs = 0;
unsigned long lastSampleTime = 0;
float ultimaTempValida = 0;

bool heaterOn = false;
bool processActive = false;

// Controle de taxa de atualização Exclusivo do Gráfico
unsigned long lastGraphUpdateTime = 0;
const unsigned long graphUpdateInterval = 500; // 0.5 segundos

// --- CONFIGURAÇÃO DO HISTÓRICO GRÁFICO ---
#define GRAPH_WIDTH 120
#define GRAPH_HEIGHT 80
#define GRAPH_X_START 10
#define GRAPH_Y_START 25

struct PointData {
  float maxTemp;
  float minTemp;
  float avgTemp; // Média calculada por pixel
  int count;
};

PointData graphHistory[GRAPH_WIDTH];
int currentGraphX = 0;
unsigned long msPerPixel = 0;
unsigned long nextPixelWindowMs = 0;

// Variáveis temporárias de amostragem para o pixel atual do gráfico
float currentPixelMax = -99.0;
float currentPixelMin = 999.0;
float currentPixelSum = 0.0;
int currentPixelCount = 0;

// Protótipos das funções
void resetProject();
void drawGraphBackground(float ref);
void displaySelectTime(unsigned long timeMs);
void displaySelectPower(int power);
void displaySelectTemp(float temp);
void displayControl(float t, float r, unsigned long rem, bool h, int d, int pLimit);
void displayControlWait(float t, float r, unsigned long total, int pLimit);
void displayControlGraphic(float r, unsigned long total);

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW); 
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  
  sensors.begin();
  sensors.setWaitForConversion(false); 

  // Inicializa o display no chip ST7735S
  tft.initR(INITR_BLACKTAB); 
  tft.setRotation(3); // Rotação de 90° Anti-horária (Tela em modo Paisagem: 160x128)
  tft.fillScreen(ST7735_BLACK);
  
  resetProject();
}

void resetProject() {
  digitalWrite(HEATER_PIN, LOW);
  currentScreen = SELECT_TIME;
  processActive = false;
  showGraphicView = false;
  totalTime_ms = 0;
  accumulatedTime_ms = 0;
  tempOffset = 0.0;
  currentGraphX = 0;
  
  // Limpa o banco de dados do histórico gráfico
  for(int i = 0; i < GRAPH_WIDTH; i++) {
    graphHistory[i] = {-1.0, -1.0, -1.0, 0};
  }
  
  // Limpa o buffer acumulador do pixel corrente
  currentPixelMax = -99.0;
  currentPixelMin = 999.0;
  currentPixelSum = 0.0;
  currentPixelCount = 0;

  tft.fillScreen(ST7735_BLACK);
  windowStartMs = millis();
}

void updatePI(float tempAtual, float tempRef) {
  if (tempAtual < -50) return; 
  float e = tempRef - tempAtual;
  uPI += Kp * (e - e_prev) + Ki * Ts * e;
  e_prev = e;
  if (uPI > 10.0) uPI = 10.0;
  if (uPI < 0.0)  uPI = 0.0;
  dutySteps = (int)(uPI + 0.5);
}

void loop() {
  bool btnState = digitalRead(BTN_PIN);
  
  // Tratador de clique e clique longo (2 segundos para reset completo)
  if (btnState == LOW) {
    if (lastBtnState == HIGH) {
      lastBtnTime = millis();
    } else if (millis() - lastBtnTime > 2000) {
      resetProject();
      lastBtnState = LOW;
      delay(500); 
      return;
    }
  } else if (btnState == HIGH && lastBtnState == LOW) {
    if (millis() - lastBtnTime < 2000 && millis() - lastBtnTime > 50) {
      tft.fillScreen(ST7735_BLACK); // Força limpeza completa ao mudar fisicamente de estado
      if (currentScreen == SELECT_TIME) currentScreen = SELECT_POWER;
      else if (currentScreen == SELECT_POWER) currentScreen = SELECT_TEMP;
      else if (currentScreen == SELECT_TEMP) currentScreen = CONTROL_MODE;
      else if (currentScreen == CONTROL_MODE) {
        if (!processActive && (millis() - startTime_ms > totalTime_ms) && totalTime_ms > 0) {
          showGraphicView = true;
        } else {
          if (!processActive) {
            processActive = true;
            startTime_ms = millis();
            uPI = 0; e_prev = 0;
            msPerPixel = totalTime_ms / GRAPH_WIDTH;
            nextPixelWindowMs = startTime_ms + msPerPixel;
          } else {
            showGraphicView = !showGraphicView;
          }
        }
      }
    }
  }
  lastBtnState = btnState;

  int potRaw = analogRead(POT_PIN);

  // Amostragem regular do sensor térmico (a cada 1 segundo)
  if (millis() - lastSampleTime >= 1000) {
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t > -50 && t < 150) {
      ultimaTempValida = t;
      
      if (processActive) {
        if (t > currentPixelMax) currentPixelMax = t;
        if (t < currentPixelMin) currentPixelMin = t;
        currentPixelSum += t;
        currentPixelCount++;
      }
    }
    lastSampleTime = millis();
  }

  // --- ESCALONAMENTO DOS POTENCIÔMETROS (Resposta rápida nativa) ---
  if (currentScreen == SELECT_TIME) {
    unsigned long baseTime = (potRaw / 4095.0) * 36000000UL;
    if (potRaw > 4050) {
      if (millis() - lastIncrementTime >= 1000) {
        accumulatedTime_ms += 900000UL; 
        lastIncrementTime = millis();
      }
    } else { lastIncrementTime = millis(); }
    totalTime_ms = baseTime + accumulatedTime_ms;
    displaySelectTime(totalTime_ms); // Atualização em tempo real (Sem delay)
  } 
  else if (currentScreen == SELECT_POWER) {
    maxPowerPercent = (potRaw / 4095.0) * 100;
    displaySelectPower(maxPowerPercent); // Atualização em tempo real (Sem delay)
  } 
  else if (currentScreen == SELECT_TEMP) {
    float baseTemp = 25.0 + (potRaw / 4095.0) * 55.0;
    if (potRaw > 4050) {
      if (millis() - lastIncrementTime >= 500) {
        if ((baseTemp + tempOffset) < 150.0) tempOffset += 0.5; 
        lastIncrementTime = millis();
      }
    } else if (potRaw < 50) {
      if (millis() - lastIncrementTime >= 500) {
        if (tempOffset > 0) tempOffset -= 0.5; 
        lastIncrementTime = millis();
      }
    } else { lastIncrementTime = millis(); }
    tempRefConfig = baseTemp + tempOffset;
    displaySelectTemp(tempRefConfig); // Atualização em tempo real (Sem delay)
  }
  else if (currentScreen == CONTROL_MODE) {
    maxPowerPercent = (potRaw / 4095.0) * 100;

    if (processActive) {
      updatePI(ultimaTempValida, tempRefConfig);

      unsigned long now = millis();
      if (now - windowStartMs >= windowMs) windowStartMs = now;
      unsigned long relTime = now - windowStartMs;
      
      float esforco = dutySteps / 10.0;
      unsigned long limitedOnTime = (unsigned long)(esforco * (maxPowerPercent / 100.0) * windowMs);

      if (relTime < limitedOnTime && dutySteps > 0) {
        digitalWrite(HEATER_PIN, HIGH);
        heaterOn = true;
      } else {
        digitalWrite(HEATER_PIN, LOW);
        heaterOn = false;
      }

      // --- MIGRAR PIXEL POR PIXEL NO EIXO X ---
      if (now >= nextPixelWindowMs && currentGraphX < GRAPH_WIDTH) {
        if (currentPixelCount > 0) {
          graphHistory[currentGraphX] = {
            currentPixelMax,
            currentPixelMin,
            (float)(currentPixelSum / currentPixelCount),
            1
          };
        }
        currentGraphX++;
        nextPixelWindowMs = startTime_ms + ((currentGraphX + 1) * msPerPixel);
        
        currentPixelMax = -99.0;
        currentPixelMin = 999.0;
        currentPixelSum = 0.0;
        currentPixelCount = 0;
      }

      unsigned long elapsed = millis() - startTime_ms;
      unsigned long remaining = (totalTime_ms > elapsed) ? totalTime_ms - elapsed : 0;

      if (remaining <= 0) {
        processActive = false;
        digitalWrite(HEATER_PIN, LOW);
        showGraphicView = true; 
        tft.fillScreen(ST7735_BLACK);
      }

      // --- GERENCIADOR EXCLUSIVO DA TAXA DE REFRESH DO GRÁFICO ---
      if (showGraphicView) {
        if (millis() - lastGraphUpdateTime >= graphUpdateInterval) {
          lastGraphUpdateTime = millis();
          displayControlGraphic(tempRefConfig, totalTime_ms);
        }
      } else {
        displayControl(ultimaTempValida, tempRefConfig, remaining, heaterOn, dutySteps, maxPowerPercent); // Texto livre de lag
      }

    } else {
      // Rotina parada (Espera ou Finalizada)
      unsigned long elapsed = millis() - startTime_ms;
      if (elapsed >= totalTime_ms && totalTime_ms > 0) {
        // Se finalizou e está exibindo o gráfico, aplica o filtro de 0.5s para renderizar estável
        if (millis() - lastGraphUpdateTime >= graphUpdateInterval) {
          lastGraphUpdateTime = millis();
          displayControlGraphic(tempRefConfig, totalTime_ms); 
        }
      } else {
        displayControlWait(ultimaTempValida, tempRefConfig, totalTime_ms, maxPowerPercent); // Sem lag
      }
      digitalWrite(HEATER_PIN, LOW);
    }
  }
}

// --- FUNÇÕES GRÁFICAS DE DESENHO ---

void drawGraphBackground(float ref) {
  tft.drawRect(GRAPH_X_START - 1, GRAPH_Y_START - 1, GRAPH_WIDTH + 2, GRAPH_HEIGHT + 2, ST7735_GRAY);
  int centerY = GRAPH_Y_START + (GRAPH_HEIGHT / 2);
  tft.drawFastHLine(GRAPH_X_START, centerY, GRAPH_WIDTH, ST7735_WHITE);
}

void displayControlGraphic(float r, unsigned long total) {
  // --- INFORMAÇÕES DO CANTO SUPERIOR (TEXTO DINÂMICO COM FUNDO PRETO) ---
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK); 
  tft.setCursor(5, 6);
  
  // Imprime no formato: "L: x% | PI: x% | T: xx.x"
  tft.print("L: "); tft.print(maxPowerPercent); tft.print("% ");
  tft.print("| PI: "); tft.print(dutySteps * 10); tft.print("% ");
  tft.print("| T:"); tft.print(ultimaTempValida, 1); tft.print("   "); // Espaços limpam dígitos extras

  // Desenha o grid estático do gráfico
  drawGraphBackground(r);

  // --- VALORES NOS EXTREMOS DO EIXO Y (CANTO DIREITO) ---
  float valMaxY = r * 1.10;
  float valMinY = r * 0.90;
  float yRange = valMaxY - valMinY;
  if (yRange <= 0) yRange = 1.0; 

  int centerY = GRAPH_Y_START + (GRAPH_HEIGHT / 2);
  tft.setTextColor(ST7735_GRAY, ST7735_BLACK);
  
  tft.setCursor(GRAPH_X_START + GRAPH_WIDTH + 4, GRAPH_Y_START - 3);
  tft.print(valMaxY, 1); 
  tft.setCursor(GRAPH_X_START + GRAPH_WIDTH + 4, centerY - 3);
  tft.print(r, 1);       
  tft.setCursor(GRAPH_X_START + GRAPH_WIDTH + 4, GRAPH_Y_START + GRAPH_HEIGHT - 5);
  tft.print(valMinY, 1); 

  // --- DESENHO DOS PIXELS DO GRÁFICO ---
  for (int x = 0; x < currentGraphX; x++) {
    if (graphHistory[x].count > 0) {
      int screenX = GRAPH_X_START + x;

      int yMax = GRAPH_Y_START + GRAPH_HEIGHT - (int)(((graphHistory[x].maxTemp - valMinY) / yRange) * GRAPH_HEIGHT);
      int yMin = GRAPH_Y_START + GRAPH_HEIGHT - (int)(((graphHistory[x].minTemp - valMinY) / yRange) * GRAPH_HEIGHT);
      int yAvg = GRAPH_Y_START + GRAPH_HEIGHT - (int)(((graphHistory[x].avgTemp - valMinY) / yRange) * GRAPH_HEIGHT);

      yMax = constrain(yMax, GRAPH_Y_START, GRAPH_Y_START + GRAPH_HEIGHT - 1);
      yMin = constrain(yMin, GRAPH_Y_START, GRAPH_Y_START + GRAPH_HEIGHT - 1);
      yAvg = constrain(yAvg, GRAPH_Y_START, GRAPH_Y_START + GRAPH_HEIGHT - 1);

      tft.drawPixel(screenX, yMax, ST7735_RED);   
      tft.drawPixel(screenX, yMin, ST7735_BLUE);  
      tft.drawPixel(screenX, yAvg, ST7735_GREEN); 
    }
  }

  // --- CRONÔMETRO NO CANTO INFERIOR (Formato: Passou | Total) ---
  unsigned long tempoPassou_ms = 0;
  if (processActive) {
    tempoPassou_ms = millis() - startTime_ms;
    if (tempoPassou_ms > total) tempoPassou_ms = total;
  } else if (totalTime_ms > 0 && (millis() - startTime_ms >= total)) {
    tempoPassou_ms = total;
  }

  unsigned long h_pass = tempoPassou_ms / 3600000UL;
  unsigned long m_pass = (tempoPassou_ms % 3600000UL) / 60000UL;
  unsigned long s_pass = (tempoPassou_ms % 60000UL) / 1000UL;
  
  unsigned long h_tot = total / 3600000UL;
  unsigned long m_tot = (total % 3600000UL) / 60000UL;
  unsigned long s_tot = (total % 60000UL) / 1000UL;

  char footerStr[32];
  sprintf(footerStr, "%02lu:%02lu:%02lu | %02lu:%02lu:%02lu", 
          h_pass, m_pass, s_pass, 
          h_tot, m_tot, s_tot);

  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(10, 115);
  tft.print(footerStr);

  // --- POSICIONAMENTO DA BOLA INDICADORA NO RODAPÉ ---
  int ballX = tft.getCursorX() + 10;
  tft.fillRect(ballX - 5, 114, 18, 10, ST7735_BLACK); 

  if (processActive && heaterOn) {
    tft.fillCircle(ballX, 118, 4, ST7735_RED);   
  } else {
    tft.fillCircle(ballX, 118, 4, ST7735_BLUE);  
  }
}

void displaySelectTime(unsigned long timeMs) {
  tft.setTextWrap(false); tft.setFont(); tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  tft.setCursor(10, 15); tft.print("1. DEFINIR TEMPO:");
  unsigned long h = timeMs / 3600000UL;
  unsigned long m = (timeMs % 3600000UL) / 60000UL;
  unsigned long s = (timeMs % 60000UL) / 1000UL;
  char timeStr[12]; sprintf(timeStr, "%02lu:%02lu:%02lu", h, m, s);
  tft.setTextSize(2); tft.setTextColor(ST7735_WHITE, ST7735_BLACK); 
  tft.setCursor(30, 55); tft.print(timeStr);
  tft.setTextSize(1); tft.setTextColor(ST7735_GRAY, ST7735_BLACK);
  tft.setCursor(10, 105); tft.print("Mantenha no MAX p/ somar horas");
}

void displaySelectPower(int power) {
  tft.setFont(); tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  tft.setCursor(10, 15); tft.print("2. POTENCIA LIMITE:");
  tft.setTextSize(2); tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(60, 55); tft.print(power); tft.print("%   "); 
  tft.setTextSize(1); tft.setTextColor(ST7735_GRAY, ST7735_BLACK);
  tft.setCursor(10, 105); tft.print("Ajuste a forca max. do sistema");
}

void displaySelectTemp(float temp) {
  tft.setFont(); tft.setTextSize(1);
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  tft.setCursor(10, 15); tft.print("3. TEMPERATURA ALVO:");
  tft.setTextSize(2); tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(45, 55); tft.print(temp, 1); tft.print(" C ");
  tft.setTextSize(1); tft.setTextColor(ST7735_GRAY, ST7735_BLACK);
  tft.setCursor(10, 105); tft.print("Mantenha no MAX p/ passar de 80C");
}

void displayControl(float t, float r, unsigned long rem, bool h, int d, int pLimit) {
  tft.setFont(); tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(10, 15); tft.print("Temp: "); tft.setTextColor(ST7735_GREEN, ST7735_BLACK); tft.print(t, 1); tft.print(" C   ");
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(10, 32); tft.print("Ref : "); tft.print(r, 1); tft.print(" C");
  tft.setCursor(10, 55); tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  tft.print("LIMITE POT.: "); tft.print(pLimit); tft.print("%   ");
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(10, 75); tft.print("Saida PI  : "); tft.print(d * 10); tft.print("%  ");
  tft.setCursor(10, 92); tft.print("Status    : ");
  if (h) { tft.setTextColor(ST7735_RED, ST7735_BLACK); tft.print("AQUECENDO"); }
  else { tft.setTextColor(ST7735_BLUE, ST7735_BLACK); tft.print("DESLIGADO"); }
  unsigned long h_rem = rem / 3600000UL;
  unsigned long m_rem = (rem % 3600000UL) / 60000UL;
  unsigned long s_rem = (rem % 60000UL) / 1000UL;
  char timeStr[12]; sprintf(timeStr, "%02lu:%02lu:%02lu", h_rem, m_rem, s_rem);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(10, 112); tft.print("RESTANTE  : ");
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK); tft.print(timeStr);
}

void displayControlWait(float t, float r, unsigned long total, int pLimit) {
  tft.setFont(); tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  tft.setCursor(10, 15);  tft.print("4. CONFIGURADO COMPLETO!");
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(10, 32);  tft.print("Clique no botao p/ iniciar");
  tft.setCursor(10, 60);  tft.print("Temp Inicial: "); tft.print(t, 1); tft.print(" C  ");
  tft.setCursor(10, 77);  tft.print("Temp Alvo   : "); tft.print(r, 1); tft.print(" C  ");
  tft.setCursor(10, 94);  tft.print("Pot. Limite : "); tft.print(pLimit); tft.print("%  ");
  unsigned long h = total / 3600000UL;
  unsigned long m = (total % 3600000UL) / 60000UL;
  unsigned long s = (total % 60000UL) / 1000UL;
  char tStr[12]; sprintf(tStr, "%02lu:%02lu:%02lu", h, m, s);
  tft.setCursor(10, 112); tft.print("Tempo Total : ");
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK); tft.print(tStr);
}