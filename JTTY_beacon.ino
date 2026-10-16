/**  
 * ============================================================================  
 *  jtty_tx_example.ino — пример JTTY-передатчика на RP2040 (ИСПРАВЛЕННЫЙ)  
 * ============================================================================  
 */  
  
#include <Arduino.h>  
#include "jtty_codec.h"        // Протокол (пакер и FEC)
#include "mfsk_tonegen.h"      // Аппаратный PIO-DDS драйвер
  
// ============================ ГЛОБАЛЬНЫЕ ПАРАМЕТРЫ ===========================  
  
// Несущая JTTY-маяка: центр полосы сигнала, Гц.  
uint32_t JTTY_FREQ_HZ = 10000000UL;            // 10 МГц  
  
// Передаваемая фраза (до 80 символов)
char JTTY_MESSAGE[] = "CQ RU0AOG CQ TESTING BEACON";  
  
// Пин кнопки USR (INPUT_PULLUP)  
static const uint8_t PIN_USR_BUTTON = 15;  
  
// Пин ВЧ-выхода PIO-DDS  
static const uint8_t PIN_RF_OUT = 7;  
  
// !!! ВНИМАНИЕ: Константы JTTY_BAUD_HZ, JTTY_SYMBOL_US, JTTY_NTONES, !!!
// !!! JTTY_NSYMS и JTTY_CENTER_OFFSET удалены отсюда, так как они   !!!
// !!! автоматически импортируются из "jtty_codec.h"                 !!!
  
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
  
// ============================ ARDUINO =======================================  
  
void setup()  
{  
    Serial.begin(115200);  
    delay(300);  
    Serial.println(F("\n[JTTY TX Beacon] Инициализация..."));  
  
    pinMode(PIN_USR_BUTTON, INPUT_PULLUP);  
  
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
}  
  
void loop()  
{  
    static bool btn_prev = true;  
    bool btn_now = digitalRead(PIN_USR_BUTTON);  
  
    if (btn_prev && !btn_now) {                // Детектируем нажатие (фронт вниз)
        delay(30);                             // Программный антидребезг  
        if (digitalRead(PIN_USR_BUTTON) == LOW) {  
            Serial.println(F("[USR] Кнопка нажата -> Старт передачи"));  
            jtty_transmit(JTTY_MESSAGE);  
        }  
    }  
    btn_prev = btn_now;  
}
