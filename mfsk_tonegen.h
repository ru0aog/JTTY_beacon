/**  
 * ============================================================================  
 *  mfsk_tonegen.h — Автономный генератор сетки MFSK-тонов для RP2040
 *  ver.1.00 от 2026-10-16, автор RU0AOG
 * ============================================================================  
 *  Назначение: аппаратный DDS на автомате PIO. Генерирует меандр на одном  
 *  GPIO с программной сеткой до 64 тонов:  
 *  
 *      f(tone) = base_hz + tone * step_hz,   tone = 0 .. ntones-1  
 *  
 *  Как это работает (кратко):  
 *  - PIO SM исполняет 2-тактовую программу set pins,1 / set pins,0 →  
 *    меандр с частотой f = clk_sys / (2 * clkdiv).  
 *  - clkdiv имеет формат 24.8: 16 бит целой части + 8 бит дроби.  
 *    Остаточная ошибка квантования (~единицы Гц) добирается FRAC-DDS-  
 *    дизером: Core 1 каждый цикл пишет clkdiv = int + frac/256 + jitter,  
 *    где jitter формируется xorshift32-ГПСЧ и маской глубины.  
 *  - Смена тона фазово-непрерывна: накопители дизера НЕ сбрасываются,  
 *    поэтому на границах символов нет фазового разрыва (важно для  
 *    узкополосного спектра JTTY: 4 тона × 31.25 Гц).  
 *  
 *  Режимы:  
 *  - TONEGEN_ENGINE_SIMPLE: одно ядро, прямая запись clkdiv. Дешевле,  
 *    но ошибка квантования до ~±(f²·2/clk_sys) Гц и без дизера —  
 *    близкие спуры на уровне −20…−30 dBc.  
 *  - TONEGEN_ENGINE_DITHER (по умолчанию): Core 1 + DDS-остаток —  
 *    точность ~мГц, спуры размазаны в шумовую подошву.  
 *  
 * ============================================================================  
 */  
#pragma once  
#include <stdint.h>  
#include <stdbool.h>  
  
#ifdef __cplusplus  
extern "C" {  
#endif  
  
/** Максимум тонов в сетке (как VFO_TONES_COUNT в vfo_hardware). */  
#define TONEGEN_MAX_TONES   64  
  
/** Движок дизеринга. */  
typedef enum {  
    TONEGEN_ENGINE_SIMPLE = 0,  /**< одно ядро, целочисленно-дробный clkdiv */  
    TONEGEN_ENGINE_DITHER = 1   /**< Core 1: FRAC-DDS + MASH-дизер          */  
} tonegen_engine_t;  
  
/**  
 * Инициализация генератора.  
 *  
 * @param pin        GPIO выхода меандра (например, 2).  
 * @param base_hz    частота нижнего тона (tone 0), Гц. Для JTTY с  
 *                   центрированием: f_center − 1.5 * step.  
 * @param step_hz    шаг сетки, Гц (JTTY: 31.25).  
 * @param ntones     число тонов (1..TONEGEN_MAX_TONES; JTTY: 4).  
 * @param engine     TONEGEN_ENGINE_DITHER или TONEGEN_ENGINE_SIMPLE.  
 * @param clk_sys_hz тактовая частота системы, Гц (0 = текущая).  
 * @return true при успехе.  
 */  
bool tonegen_init(uint8_t pin, uint32_t base_hz, double step_hz,  
                  uint8_t ntones, tonegen_engine_t engine,  
                  uint32_t clk_sys_hz);  
  
/** Выбор тона мгновенно (фазово-непрерывно в DITHER-режиме). */  
void tonegen_set_tone(uint8_t tone_index);  
  
/** Ключ передатчика: true = пин излучает, false = пин в вход/ноль. */  
void tonegen_key(bool on);  
  
/** Узкополосный режим: принудительно выключает FRAC-дизер (чистые линии  
 *  для Olivia/JTTY, где спур-гребёнка хуже квантованной линии). */  
void tonegen_set_narrowband(bool on);  
  
/** Блокирующее ожидание слота символа (обёртка для точного тайминга). */  
void tonegen_wait_us(uint32_t us);  
  
/** Освобождение PIO SM и ядра 1. */  
void tonegen_deinit(void);  
  
/** Фактическая частота тона после квантования, Гц (диагностика). */  
double tonegen_actual_hz(uint8_t tone_index);  
  
#ifdef __cplusplus  
}  
#endif