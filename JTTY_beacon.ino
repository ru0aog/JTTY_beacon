/**  
 * ============================================================================  
 *  jtty_tx_example.ino — пример JTTY-передатчика на RP2040  
 * ----------------------------------------------------------------------------  
 *  Назначение:  
 *    Демонстрация использования библиотек jtty_codec (упаковка + FEC) и  
 *    tonegen (PIO-формирователь MFSK-тонов). По нажатию кнопки USR  
 *    передаётся одна JTTY-посылка:  "CQ RU0AOG CQ TESTING BEACON"  
 *    на частоте 10.000.000 Гц.  
 *  
 *  Что делает скетч:  
 *    1. Инициализирует PIO-DDS (tonegen) и Serial для отладки.  
 *    2. Опрашивает кнопку USR с программным антидребезгом.  
 *    3. По нажатию:  
 *         - jtty_pack_message() разбивает текст на кадры (CALL + TEXT5);  
 *         - jtty_payload_to_tones() превращает каждый 34-битный payload  
 *           в массив из 59 тонов (13 синхро + 46 данных, CRC-12 + TBCC);  
 *         - кадры излучаются подряд, каждый символ — 32 мс (1.888 с/кадр).  
 *  
 *  Частотный план:  
 *    JTTY — 4-FSK, шаг тонов 31.25 Гц, полоса ≈ 94 Гц.  
 *    JTTY_FREQ_HZ задаёт ЦЕНТР полосы; нижний тон сетки:  
 *        f0 = JTTY_FREQ_HZ - 46.875 Гц  
 *    т.е. тоны: f0, f0+31.25, f0+62.5, f0+93.75 Гц.  
 *  
 *  Зависимости (обе должны лежать рядом со скетчем или в libraries):  
 *    jtty_codec.h / jtty_codec.cpp   — протокол (без привязки к железу);  
 *    tonegen.h    / tonegen.cpp     — PIO-DDS: tg_hardware_init(),  
 *                                     tg_prepare_grid(), tg_set_tone(),  
 *                                     tg_tx_begin(), tg_operation_set().  
 *  
 *  ВНИМАНИЕ — ПИНЫ:  
 *    PIN_USR_BUTTON и PIN_RF_OUT заданы типовыми — СВЕРЬ с разводкой  
 *    своей платы: в основном проекте выход DDS и кнопки определены  
 *    в vfo_hardware.h. Неверный PIN_RF_OUT — нет сигнала.  
 * ============================================================================  
 */  
  
#include <Arduino.h>  
#include "jtty_codec.h"   // jtty_pack_message(), jtty_payload_to_tones(), JttyFrameList  
#include "tonegen.h"      // tg_hardware_init(), tg_prepare_grid(), tg_set_tone(), tg_tx_begin()  
  
// ============================ ГЛОБАЛЬНЫЕ ПАРАМЕТРЫ ===========================  
  
// Несущая JTTY-маяка: центр полосы сигнала, Гц.  
uint32_t JTTY_FREQ_HZ = 10000000UL;            // 10 МГц  
  
// Передаваемая фраза (до 80 символов; CALL-атомы и свободный текст TEXT5).  
// Алфавит TEXT5: 0-9 A-Z и знаки " +-./? ..." — см. JTTY_ALPHABET в jtty_codec.  
char JTTY_MESSAGE[] = "CQ RU0AOG CQ TESTING BEACON";  
  
// Пин кнопки USR (к замыканию на GND, поэтому INPUT_PULLUP).  
// !!! Подставь реальный GPIO из разводки платы !!!  
static const uint8_t PIN_USR_BUTTON = 15;  
  
// Пин ВЧ-выхода PIO-DDS (должен совпадать с тем, что жёг tonegen/vfo_hardware).  
// Если tg_hardware_init() сам привязывает выход к пину из своей конфигурации,  
// эта константа не используется — оставлена для наглядности.  
static const uint8_t PIN_RF_OUT = 7;           // !!! сверить с платой !!!  
  
// ======================== ПАРАМЕТРЫ МОДУЛЯЦИИ JTTY ==========================  
  
static const float    JTTY_BAUD_HZ   = 31.25f;    // символьная скорость = шаг тонов  
static const uint32_t JTTY_SYMBOL_US = 32000UL;   // 32 мс на символ  
static const uint8_t  JTTY_NTONES    = 4;         // 4-FSK  
static const uint8_t  JTTY_NSYMS     = 59;        // 13 sync + 46 data  
static const float    JTTY_CENTER_OFFSET = 46.875f; // 1.5 * 31.25 Гц — центрирование  
  
// ============================ СЛУЖЕБНЫЙ КОД =================================  
  
/**  
 * Выдержать паузу одного символа с точным дедлайном по micros().  
 * Возвращает false, если нужно прервать передачу (здесь — никогда;  
 * место для аварийных условий: watchdog, стоп-команда и т.п.).  
 */  
static bool wait_symbol()  
{  
    static uint32_t deadline = 0;  
    if (deadline == 0) deadline = micros() + JTTY_SYMBOL_US;  
    while ((int32_t)(micros() - deadline) < 0) {  
        delayMicroseconds(50);      // короткое ожидание, CPU почти свободен  
        watchdog_update();          // кормим вотчдог RP2040 (Arduino-порт)  
    }  
    deadline += JTTY_SYMBOL_US;     // следующий дедлайн — без накопления дрейфа  
    return true;  
}  
  
/**  
 * Передать одну посылку JTTY: все кадры сообщения подряд.  
 * Блокирующая: длительность = nframes * 1.888 с.  
 */  
static void jtty_transmit(const char* text)  
{  
    // --- 1. Упаковка: текст -> массив 34-битных payload'ов ---  
    JttyFrameList fl;  
    if (!jtty_pack_message(text, &fl, JTTY_EXCHANGE_UNKNOWN, -1)) {  
        Serial.println(F("[JTTY] ошибка упаковки сообщения"));  
        return;  
    }  
    Serial.printf("[JTTY] \"%s\" -> кадров: %d\n", text, fl.nframes);  
  
    // --- 2. Частотная сетка: f0 = центр - 46.875 Гц, шаг 31.25, 4 тона ---  
    double f0 = (double)JTTY_FREQ_HZ - JTTY_CENTER_OFFSET;  
    tg_prepare_grid((uint32_t)(f0 + 0.5), JTTY_BAUD_HZ, JTTY_NTONES);  
  
    // --- 3. Передача кадров ---  
    tg_tx_begin();                  // включаем DDS/ВЧ-тракт  
    for (int f = 0; f < fl.nframes; f++) {  
        uint8_t itone[JTTY_NSYMS];  
        jtty_payload_to_tones(fl.frames[f], itone);   // CRC-12 + TBCC + is13  
        for (int i = 0; i < JTTY_NSYMS; i++) {  
            tg_set_tone(itone[i]);  // мгновенное переключение тона (PIO)  
            if (!wait_symbol()) { tg_operation_set(false); return; }  
        }  
    }  
    tg_operation_set(false);        // несущая OFF  
    Serial.println(F("[JTTY] передача завершена"));  
}  
  
// ============================ ARDUINO =======================================  
  
void setup()  
{  
    Serial.begin(115200);  
    delay(300);  
    Serial.println(F("\n[JTTY TX example] init..."));  
  
    pinMode(PIN_USR_BUTTON, INPUT_PULLUP);     // кнопка на GND  
  
    tg_hardware_init();                        // PIO-DDS: несущая выключена  
    // tg_prepare_grid() вызывается внутри jtty_transmit() — при каждом старте,  
    // чтобы JTTY_FREQ_HZ можно было менять "на лету" между нажатиями.  
  
    Serial.printf("[JTTY] готов. Частота: %lu Гц, текст: \"%s\"\n",  
                  JTTY_FREQ_HZ, JTTY_MESSAGE);  
}  
  
void loop()  
{  
    // --- Опрос кнопки USR с антидребезгом 30 мс и ожиданием отпускания ---  
    static bool btn_prev = true;               // true = не нажата (pullup)  
    bool btn_now = digitalRead(PIN_USR_BUTTON);  
  
    if (btn_prev && !btn_now) {                // фронт нажатия  
        delay(30);                             // антидребезг  
        if (digitalRead(PIN_USR_BUTTON) == LOW) {  
            Serial.println(F("[USR] нажатие -> передача JTTY"));  
            jtty_transmit(JTTY_MESSAGE);       // блокирует на n*1.888 с  
        }  
    }  
    btn_prev = btn_now;  
}