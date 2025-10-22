#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Definições do display OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// Definições do LED
#define LED_PIN 2

// Pinos I2C (conforme seu JSON)
#define I2C_SDA 8
#define I2C_SCL 9

// Instâncias dos dispositivos
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

// Buffer para armazenar as últimas medidas
const int BUFFER_SIZE = 10;
float accelX_buffer[BUFFER_SIZE];
float accelY_buffer[BUFFER_SIZE];
float accelZ_buffer[BUFFER_SIZE];
int buffer_index = 0;

// Variáveis para detecção de mudança
float last_avgX = 0, last_avgY = 0, last_avgZ = 0;
const float ACCEL_THRESHOLD = 0.5; // 0.5 m/s²
bool led_state = false;

// Declaração das funções
float calculateAverage(float buffer[]);
bool checkAccelerationChange(float currentX, float currentY, float currentZ);
void updateDisplay(float x, float y, float z, bool changed);
void scanI2CDevices();

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);
  
  // Configura o pino do LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Inicializa o barramento I2C com os pinos específicos
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Aguarda inicialização dos dispositivos I2C
  delay(1000);
  
  Serial.println("=== SISTEMA MPU6050 + SSD1306 ===");
  Serial.println("Inicializando barramento I2C...");
  
  // Detecta dispositivos I2C
  scanI2CDevices();

  // Inicializa o display OLED
  Serial.println("Inicializando display SSD1306...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("❌ Falha na inicialização do SSD1306!");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
  Serial.println("✅ SSD1306 inicializado com sucesso!");
  
  // Inicializa o MPU6050
  Serial.println("Inicializando MPU6050...");
  
  bool mpuFound = false;
  
  // Tenta endereço padrão 0x68
  if(mpu.begin(0x68)) {
    Serial.println("✅ MPU6050 encontrado no endereço 0x68!");
    mpuFound = true;
  } 
  // Tenta endereço alternativo 0x69
  else if(mpu.begin(0x69)) {
    Serial.println("✅ MPU6050 encontrado no endereço 0x69!");
    mpuFound = true;
  }
  
  if(!mpuFound) {
    Serial.println("❌ Falha na inicialização do MPU6050!");
    Serial.println("Verifique as conexões I2C.");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  }
  
  // Configura o MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Limpa o display e exibe mensagem inicial
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Sistema Iniciado");
  display.println("MPU6050 + SSD1306");
  display.println("Dispositivos OK");
  display.println("Aguardando dados...");
  display.display();
  delay(2000);
  
  // Inicializa o buffer com zeros
  for(int i = 0; i < BUFFER_SIZE; i++) {
    accelX_buffer[i] = 0;
    accelY_buffer[i] = 0;
    accelZ_buffer[i] = 0;
  }
  
  // Lê primeira medição para inicializar as variáveis
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  last_avgX = a.acceleration.x;
  last_avgY = a.acceleration.y;
  last_avgZ = a.acceleration.z;
  
  Serial.println("✅ Sistema inicializado com sucesso!");
  Serial.println("=== DADOS DE ACELERAÇÃO ===");
  Serial.println("Eixo X\t\tEixo Y\t\tEixo Z\t\tLED");
  Serial.println("(m/s²)\t\t(m/s²)\t\t(m/s²)\t\tEstado");
  Serial.println("----------------------------------------");
}

void loop() {
  // Lê os dados do sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Armazena no buffer circular
  accelX_buffer[buffer_index] = a.acceleration.x;
  accelY_buffer[buffer_index] = a.acceleration.y;
  accelZ_buffer[buffer_index] = a.acceleration.z;
  
  // Atualiza o índice do buffer
  buffer_index = (buffer_index + 1) % BUFFER_SIZE;
  
  // Calcula as médias das últimas 10 medidas
  float avgX = calculateAverage(accelX_buffer);
  float avgY = calculateAverage(accelY_buffer);
  float avgZ = calculateAverage(accelZ_buffer);
  
  // Verifica se houve mudança significativa na aceleração
  bool acceleration_changed = checkAccelerationChange(avgX, avgY, avgZ);
  
  // Acende o LED se detectou mudança
  if (acceleration_changed) {
    digitalWrite(LED_PIN, HIGH);
    led_state = true;
  } else {
    digitalWrite(LED_PIN, LOW);
    led_state = false;
  }
  
  // Atualiza as últimas médias
  last_avgX = avgX;
  last_avgY = avgY;
  last_avgZ = avgZ;
  
  // Apresenta no display os valores médios
  updateDisplay(avgX, avgY, avgZ, acceleration_changed);
  
  // Envia as informações via terminal serial
  Serial.printf("%.2f\t\t%.2f\t\t%.2f\t\t", avgX, avgY, avgZ);
  if (acceleration_changed) {
    Serial.println("LIGADO");
  } else {
    Serial.println("DESLIGADO");
  }
  
  // Aguarda 200ms para próxima leitura
  delay(200);
}

// Função para calcular média
float calculateAverage(float buffer[]) {
  float sum = 0;
  for(int i = 0; i < BUFFER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / BUFFER_SIZE;
}

// Função para verificar mudança na aceleração
bool checkAccelerationChange(float currentX, float currentY, float currentZ) {
  float deltaX = abs(currentX - last_avgX);
  float deltaY = abs(currentY - last_avgY);
  float deltaZ = abs(currentZ - last_avgZ);
  
  // Retorna true se qualquer eixo tiver variação maior que 0.5 m/s²
  return (deltaX > ACCEL_THRESHOLD || deltaY > ACCEL_THRESHOLD || deltaZ > ACCEL_THRESHOLD);
}

// Função para atualizar o display
void updateDisplay(float x, float y, float z, bool changed) {
  display.clearDisplay();
  display.setCursor(0,0);
  
  display.println("Aceleracao Media");
  display.println("----------------");
  
  // Destaque especial para o eixo X conforme solicitado
  display.print("X: ");
  display.print(x, 2);
  display.println(" m/s²");
  
  display.print("Y: ");
  display.print(y, 2);
  display.println(" m/s²");
  
  display.print("Z: ");
  display.print(z, 2);
  display.println(" m/s²");
  
  display.println("----------------");
  
  if(changed) {
    display.println("MUDANCA DETECTADA!");
    display.println("LED: LIGADO");
  } else {
    display.println("Estavel");
    display.println("LED: DESLIGADO");
  }
  
  display.display();
}

// Função para escanear dispositivos I2C
void scanI2CDevices() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Escaneando dispositivos I2C...");
  
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("✅ Dispositivo I2C: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identifica dispositivos conhecidos
      if(address == 0x3C) Serial.print(" (SSD1306)");
      if(address == 0x68) Serial.print(" (MPU6050)");
      if(address == 0x69) Serial.print(" (MPU6050)");
      
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("❌ Nenhum dispositivo I2C encontrado!");
  } else {
    Serial.println("✅ Busca I2C concluída.");
  }
}