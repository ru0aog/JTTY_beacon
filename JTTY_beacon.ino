/**    
 * ============================================================================    
 *  JTTY_beacon.ino — пример JTTY-передатчика на RP2040  
 *  ver.1.01 от 2026-10-16, автор RU0AOG  
 * ============================================================================    
 *  
 *  ОПИСАНИЕ РАБОТЫ  
 *  - по нажатию кнопки USR (пин 24) в эфир уходит заранее составленное сообщение  
 *  - любая команда, принятая по USB-Serial, также запускает передачу  
 *  - сообщение хранится в переменной JTTY_MESSAGE[]  
 *  - частота определяется переменной JTTY_FREQ_HZ  
 *  - пин, на котором генерируется радиосигнал назначается переменной PIN_RF_OUT  
 * ============================================================================    
 */    
  
#include <Arduino.h>    
#include "jtty_codec.h"        // Протокол (пакер и FEC)  
#include "mfsk_tonegen.h"      // Аппаратный PIO-DDS драйвер  
    
// ============================ ГЛОБАЛЬНЫЕ ПАРАМЕТРЫ ===========================    
  
char JTTY_MESSAGE[] = "CQ RU0AOG CQ TESTING BEACON";    // Передаваемая фраза (до 80 символов)  
uint32_t               JTTY_FREQ_HZ = 10001500UL;       // Несущая JTTY-маяка: центр полосы сигнала, Гц.   
static const uint8_t     PIN_RF_OUT = 15;               // Пин ВЧ-выхода PIO-DDS    
static const uint8_t PIN_USR_BUTTON = 24;               // Пин кнопки USR (INPUT_PULLUP)    
static const uint8_t PIN_LED_BUTTON = 25;               // Пин индикаторного светодиода  
  
  
  
// ============================ СЛУЖЕБНЫЙ КОД =================================    
  
/**    
 * Передать одну посылку JTTY: все кадры сообщения подряд.    
 */    
static void jtty_transmit(const char* text)    
{    
    // 1. Упаковка текста в кадры JTTY  
    JttyFrameList fl;    
    if (!jtty_pack_message(text, &fl, JTTY_EXCHANGE_UNKNOWN, -1)) {    
        Serial.println(F("[JTTY] Ошибка упаковки сообщения"));    
        return;    
    }    
    Serial.printf("[JTTY] \"%s\" -> кадров: %d\n", text, fl.nframes);    
  
    // 2. Включаем передачу (переводим пин ВЧ в режим выхода)  
    tonegen_key(true);    
  
    // 3. Поочередное излучение кадров  
    for (int f = 0; f < fl.nframes; f++) {    
        uint8_t itone[JTTY_NSYMS];    
        jtty_payload_to_tones(fl.frames[f], itone);   // FEC кодирование кадра  
          
        for (int i = 0; i < JTTY_NSYMS; i++) {    
            tonegen_set_tone(itone[i]);         // Фазово-непрерывная смена тона (PIO)  
            tonegen_wait_us(JTTY_SYMBOL_US);    // Точный временной слот 32 мс  
        }    
    }    
      
    // 4. Выключаем несущую (пин ВЧ в Z-состояние)  
    tonegen_key(false);    
    Serial.println(F("[JTTY] Передача завершена"));    
}    
  
/**  
 * Общий триггер передачи: светодиод + лог источника + посылка JTTY_MESSAGE.  
 */  
static void start_tx(const char* source)  
{  
    digitalWrite(PIN_LED_BUTTON, HIGH);             // зажигаем индикаторный светодиод
    Serial.printf("[TX] Триггер: %s\n", source);    // вывод в СОМ-порт сообщения о начале передачи
    jtty_transmit(JTTY_MESSAGE);                    // запуск передачи
    digitalWrite(PIN_LED_BUTTON, LOW);              // гасим светодиод
    while (Serial.available()) Serial.read();       // очищаем буфер СОМ-порта
}  
  
// ============================ ARDUINO =======================================    
  
void setup()    
{    
    Serial.begin(115200);  
    Serial.setTimeout(100);   // readStringUntil не должен блокировать loop() надолго  
  
    // ожидание коннекта - до 5 сек  
    uint32_t timeout = millis();  
    while (!Serial && (millis() - timeout < 5000)) {  
    }  
  
    Serial.println(F("\n[JTTY TX Beacon] Инициализация..."));    
    
    pinMode(PIN_USR_BUTTON, INPUT_PULLUP);  
    pinMode(PIN_LED_BUTTON, OUTPUT);  
    
    // Расчет базовой частоты (f0) для нижнего тона сетки JTTY  
    double f0 = (double)JTTY_FREQ_HZ - JTTY_CENTER_OFFSET;    
    
    // Инициализируем генератор частоты с использованием дизеринга (Core 1)  
    bool init_ok = tonegen_init(    
        PIN_RF_OUT,     
        (uint32_t)(f0 + 0.5),   // Округляем до ближайшего целого Гц  
        JTTY_BAUD_HZ,     
        JTTY_NTONES,     
        TONEGEN_ENGINE_DITHER,     
        0                       // 0 = автоопределение clk_sys_hz  
    );    
    
    if (!init_ok) {    
        Serial.println(F("[TONEGEN] КРИТИЧЕСКАЯ ОШИБКА: Не удалось занять PIO SM!"));    
        while(1) delay(100);    
    }    
    
    // Включаем narrowband режим для спектральной чистоты JTTY  
    tonegen_set_narrowband(true);    
    
    Serial.printf("[JTTY] Готов. Центр: %lu Гц, Текст: \"%s\"\n",    
                  JTTY_FREQ_HZ, JTTY_MESSAGE);  
    Serial.println(F("[JTTY] Отправьте любую команду в порт для запуска передачи"));  
}    
    
void loop()    
{  
    // --- Триггер 1: кнопка USR ---  
    static bool btn_prev = true;    
    bool btn_now = digitalRead(PIN_USR_BUTTON);    
  
    if (btn_prev && !btn_now) {                // Детектируем нажатие (фронт вниз)  
        delay(30);                             // Программный антидребезг  
        if (digitalRead(PIN_USR_BUTTON) == LOW) {    
            start_tx("button");  
        }    
    }    
    btn_prev = btn_now;  
  
    // --- Триггер 2: любая команда из Serial ---  
    if (Serial.available() > 0) {  
        String cmd = Serial.readStringUntil('\n');  
        cmd.trim();  
        Serial.printf("[RX] Команда: \"%s\" -> старт передачи\n", cmd.c_str());  
        start_tx("serial");  
    }  
}