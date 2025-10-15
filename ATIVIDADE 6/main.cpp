#include <Arduino.h>
#include <esp_timer.h>

// Definições dos pinos
#define BUTTON_A_GPIO    0
#define BUTTON_B_GPIO    1
#define LED1_GPIO        2
#define LED2_GPIO        3
#define BUZZER_GPIO      4

// Variáveis globais
volatile bool button_b_enabled = true;
volatile bool led1_state = false;
volatile bool led2_state = false;

// Flags para interrupções (evitar usar delay em ISR)
volatile bool button_a_pressed = false;
volatile bool button_b_pressed = false;
unsigned long last_buzzer_time = 0;
bool buzzer_active = false;

// Handler de interrupção do Botão A
void IRAM_ATTR button_a_isr_handler() {
  button_a_pressed = true;
}

// Handler de interrupção do Botão B
void IRAM_ATTR button_b_isr_handler() {
  button_b_pressed = true;
}

// Handler do timer (chamado a cada 2 segundos)
void timer_isr_handler(void* arg) {
  led2_state = !led2_state;
  digitalWrite(LED2_GPIO, led2_state);
}

// Configuração do Buzzer
void activate_buzzer() {
  if (!button_b_enabled) {
    Serial.println("Buzzer desativado via UART");
    return;
  }
  
  Serial.println("Buzzer ativado por 1500ms");
  buzzer_active = true;
  last_buzzer_time = millis();
  
  // Usar tone para o buzzer (mais simples no Arduino)
  tone(BUZZER_GPIO, 2000); // 2kHz
}

void setup() {
  Serial.begin(115200);
  
  // Configurar pinos
  pinMode(LED1_GPIO, OUTPUT);
  pinMode(LED2_GPIO, OUTPUT);
  pinMode(BUZZER_GPIO, OUTPUT);
  pinMode(BUTTON_A_GPIO, INPUT_PULLUP);
  pinMode(BUTTON_B_GPIO, INPUT_PULLUP);
  
  // LEDs começam desligados
  digitalWrite(LED1_GPIO, LOW);
  digitalWrite(LED2_GPIO, LOW);
  digitalWrite(BUZZER_GPIO, LOW);
  
  // Configurar interrupções dos botões (borda de descida)
  attachInterrupt(digitalPinToInterrupt(BUTTON_A_GPIO), button_a_isr_handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_B_GPIO), button_b_isr_handler, FALLING);
  
  // Configurar timer para LED2 (2 segundos)
  esp_timer_handle_t timer_handle;
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_isr_handler,
    .name = "blink_timer"
  };
  
  esp_timer_create(&timer_args, &timer_handle);
  esp_timer_start_periodic(timer_handle, 2000000); // 2 segundos
  
  Serial.println("\n=== Sistema de Interrupções - ESP32-S3 ===");
  Serial.println("Botão A (GPIO0): Alterna LED1 (GPIO2)");
  Serial.println("Botão B (GPIO1): Ativa Buzzer (GPIO4) por 1.5s");
  Serial.println("Timer: Alterna LED2 (GPIO3) a cada 2s");
  Serial.println("UART: 'a' desativa Botão B, 'b' ativa Botão B");
  Serial.println("Sistema inicializado e pronto!");
  Serial.println("Aguardando eventos...\n");
}

void loop() {
  // Processar Botão A
  if (button_a_pressed) {
    button_a_pressed = false;
    led1_state = !led1_state;
    digitalWrite(LED1_GPIO, led1_state);
    Serial.printf("Botão A pressionado - LED1: %s\n", led1_state ? "LIGADO" : "DESLIGADO");
    delay(50); // Debounce
  }
  
  // Processar Botão B
  if (button_b_pressed) {
    button_b_pressed = false;
    Serial.print("Botão B pressionado - ");
    if (button_b_enabled) {
      activate_buzzer();
    } else {
      Serial.println("Função desativada via UART");
    }
    delay(50); // Debounce
  }
  
  // Controlar tempo do buzzer
  if (buzzer_active && (millis() - last_buzzer_time >= 1500)) {
    buzzer_active = false;
    noTone(BUZZER_GPIO);
    digitalWrite(BUZZER_GPIO, LOW);
  }
  
  // Processar comandos UART
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'a') {
      button_b_enabled = false;
      Serial.println("Comando UART 'a': Função do Botão B DESATIVADA");
    } else if (command == 'b') {
      button_b_enabled = true;
      Serial.println("Comando UART 'b': Função do Botão B ATIVADA");
    } else if (command != '\n' && command != '\r') {
      Serial.printf("Caractere recebido: '%c' (0x%02x)\n", command, command);
    }
  }
  
  delay(10); // Pequeno delay para evitar sobrecarga
}