/**  
 * ============================================================================  
 *  mfsk_tonegen.h — Автономный генератор сетки MFSK-тонов для RP2040  
 *  ver.1.05 от 2026-10-16, автор RU0AOG  
 * ============================================================================  
 *  Назначение: аппаратный DDS на автомате PIO. Генерирует меандр на одном  
 *  GPIO с программной сеткой до 64 тонов:  
 *  
 *      f(tone) = base_hz + tone * step_hz,   tone = 0 .. ntones-1  
 *  
 *  Полный перенос движка vfo_hardware (RP2040_beacon):  
 *  - Многокритериальный автотюн PLL: матричный скан refdiv×fbdiv×(p1,p2),  
 *    метрика спуров (FRAC8-отстройка, уровень dBc, вето простых дробей,  
 *    якоря k/8, гейт clk·int, FORB-арбитраж 2^40/2^42), защита от пустого  
 *    скана.  
 *  - Динамическое перепрограммирование pll_sys: detach clk_peri,  
 *    ожидание LOCK, пересчёт BAUDR флэша, самопроверка с 3 ретраями  
 *    и фолбэком на номинал, двухранговый сдвиг напряжения VREG.  
 *  - Двухъядерный дизеринг Core 1: MASH-1 / MASH-1-1 (автовыбор по маске),  
 *    TPDF-дизер FRAC-байта, RPDF-дизер целой части шага, спинлок-снимок,  
 *    сброс накопителей по флагу dither_reset_pending.  
 *  - Фазово-непрерывная смена тона (накопители НЕ сбрасываются).  
 *  - Авто-BOOST: вход/выход на оптимальный clk_sys под целевую частоту.  
 *  
 * ============================================================================  
 */  
#pragma once  
#include <stdint.h>  
#include <stdbool.h>  
#include <Arduino.h>  

/* Паспортный номинал кварца; коррекция в ppb задаётся сеттером  
 * (замена Flash-хранению [XOSC_PPB] из родительского проекта). */  
#define TG_XOSC_NOMINAL_HZ     12000000ULL   // кварц 12.000000 МГц  
#define TG_XOSC_DEFAULT_PPB    (+18000LL)    // дефолтная коррекция в bpm
  
extern int32_t tg_xosc_corr_ppb;  
  
/* Скорректированная частота кварца — считается в рантайме. */  
static inline uint64_t tg_calibrated_xosc_hz(void) {  
    // Всё в int64: знак коррекции должен сохраниться.  
    int64_t hz = (int64_t)TG_XOSC_NOMINAL_HZ +  
                 (int64_t)TG_XOSC_NOMINAL_HZ * (int64_t)tg_xosc_corr_ppb / 1000000000LL;  
    return (hz > 0) ? (uint64_t)hz : (uint64_t)TG_XOSC_NOMINAL_HZ;  
}  
void tonegen_set_xosc_corr_ppb(int32_t ppb);  
  
/* ------------------------- Лимиты частотного плана ------------------------ */  
#define TG_CLK_SYS_NOMINAL_HZ  133000000ULL  // электрический номинал без вольтмода  
#define TG_CLK_SYS_MIN_HZ       48000000ULL  // нижняя граница поиска оптимума  
#define TG_CLK_SYS_MAX_HZ      420000000ULL  // АБСОЛЮТНЫЙ аппаратный барьер  
#define TG_PLL_MAX_HZ         4300000000ULL  // АБСОЛЮТНЫЙ аппаратный барьер VCO  
#define TG_SIMPLE_FRAC_INT_MAX      50       // выше — вето простых дробей не действует  
  
/* Паспортные лимиты (замена flash-значениям vfo_hardware).  
 * По умолчанию — штатный номинал; можно поднять через setter, если  
 * плата доказала оверклок. */  
extern uint32_t tg_max_clk_sys_hz;   // паспорт = 133 МГц  
extern uint32_t tg_max_pll_vco_hz;   // паспорт = 1600 МГц  
  
void tonegen_set_clock_limits(uint32_t clk_sys_hz, uint32_t pll_vco_hz); // 0 = не менять  
  
/* Эффективный лимит поиска: паспорт, но не выше аппаратного барьера */  
static inline uint64_t tg_effective_clk_max(void) {  
    uint64_t lim = (uint64_t)tg_max_clk_sys_hz;  
    return (lim < TG_CLK_SYS_MAX_HZ) ? lim : TG_CLK_SYS_MAX_HZ;  
}  
static inline uint64_t tg_effective_pll_max(void) {  
    uint64_t lim = (uint64_t)tg_max_pll_vco_hz;  
    return (lim < TG_PLL_MAX_HZ) ? lim : TG_PLL_MAX_HZ;  
}  
  
/* ------------------------------ Конфигурация PLL -------------------------- */  
struct TgPllConfig {  
    uint32_t fbdiv;  
    uint32_t p1;  
    uint32_t p2;  
    uint64_t clk_sys_hz;  
    uint32_t vsel;  
    bool     is_oc;  
    uint32_t refdiv;  
};  
  
/* Архитектурное разделение профилей тактирования */  
extern TgPllConfig tg_pll_nominal;     // базовый режим (133 МГц)  
extern TgPllConfig tg_pll_overclock;   // активный boost-профиль текущего сеанса  
extern TgPllConfig tg_pll_ceiling;     // абсолютный жёсткий потолок  
extern volatile bool tg_clk_boosted;  
  
/* true, если паспорт выше стандартного номинала (133 МГц / 1600 МГц VCO) */  
static inline bool tg_passport_allows_boost(void) {  
    return (tg_effective_clk_max() > TG_CLK_SYS_NOMINAL_HZ) ||  
           (tg_effective_pll_max() > 1600000000ULL);  
}  
  
/* ------------------------------ Параметры тона ---------------------------- */  
struct TgToneParams {  
    uint32_t pio_int;  
    uint32_t pio_frac;  
    uint32_t dds_step;  
    uint32_t target_freq_chz;  
};  
  
#define TONEGEN_MAX_TONES  64  
#define TG_TONE_NONE      255  
  
extern TgToneParams __attribute__((section(".time_critical.tg_tones"))) tg_tones[TONEGEN_MAX_TONES];  
  
/* --------------------------------- API ----------------------------------- */  
/** Движок дизеринга. */  
typedef enum {  
    TONEGEN_ENGINE_SIMPLE = 0,  /**< одно ядро, целочисленно-дробный clkdiv */  
    TONEGEN_ENGINE_DITHER = 1   /**< Core 1: FRAC-DDS + MASH-1-1 дизер      */  
} tonegen_engine_t;  
  
/**  
 * Инициализация генератора (полный цикл: автотюн PLL + PIO + Core 1).  
 *  
 * @param pin        GPIO выхода меандра.  
 * @param base_hz    частота нижнего тона (tone 0), Гц.  
 * @param step_hz    шаг сетки, Гц (JTTY: 31.25).  
 * @param ntones     число тонов (1..TONEGEN_MAX_TONES; JTTY: 4).  
 * @param engine     TONEGEN_ENGINE_DITHER (рекомендуется) или SIMPLE.  
 * @param clk_limit_hz верхний лимит clk_sys для автотюна, Гц  
 *                     (0 = паспорт tg_max_clk_sys_hz). При переданном  
 *                     TG_CLK_SYS_NOMINAL_HZ автотюн ищет в пределах 133 МГц.  
 * @return true при успехе.  
 */  
bool tonegen_init(uint8_t pin, uint32_t base_hz, double step_hz,  
                  uint8_t ntones, tonegen_engine_t engine,  
                  uint32_t clk_limit_hz);  
  
/** Выбор тона мгновенно (фазово-непрерывно в DITHER-режиме). */  
void tonegen_set_tone(uint8_t tone_index);  
  
/** Ключ передатчика: true = пин излучает, false = пин в вход (Z). */  
void tonegen_key(bool on);  
  
/** Узкополосный режим: принудительно выключает FRAC-дизер. */  
void tonegen_set_narrowband(bool on);  
  
/** Блокирующее ожидание слота символа. */  
void tonegen_wait_us(uint32_t us);  
  
/** Освобождение PIO SM и ядра 1. */  
void tonegen_deinit(void);  
  
/** Фактическая частота тона после квантования, Гц (диагностика). */  
double tonegen_actual_hz(uint8_t tone_index);  
  
/* --- Автотюн и boost (полный перенос из vfo_hardware) --- */  
TgPllConfig tonegen_find_optimal_pll(unsigned int target_frequency_hz,  
                                     uint64_t max_clk_limit = 0); // 0 = auto  
void tonegen_clk_boost_arm(void);  
void tonegen_clk_boost_enter(unsigned int target_freq_hz);  
void tonegen_clk_boost_exit(void);

