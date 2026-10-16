/**  
 * ============================================================================  
 *  mfsk_tonegen.cpp — программный генератор тонов радиочастоты на RP2040  
 *  ver.1.02 от 2026-10-16, автор RU0AOG  
 * ============================================================================  
 *  МНОГОКРИТЕРИАЛЬНЫЙ АВТОТЮН PLL И ДВУХЪЯДЕРНЫЙ ДИЗЕРИНГ  
 *  Полный перенос движка vfo_hardware.cpp (v2.23) без сокращений:  
 *  метрики спуров, матричный PLL-скан, динамическое переключение clk_sys,  
 *  MASH-1/MASH-1-1, TPDF/RPDF-дизер, boost-профили.  
 *  
 *  ИСПОЛЬЗУЕМЫЕ БИБЛИОТЕКИ И МОДУЛИ  
 *  - Raspberry Pi Pico SDK (BSD-3-Clause)  
 *  - Arduino core RP2040 (LGPL-2.1)  
 *  - Сторонний код отсутствует: дизеринг-движок, автотюн PLL с метрикой  
 *    спуров, PIO-программа — оригинальная разработка автора проекта.  
 *  
 * ============================================================================  
 */  
  
#include "mfsk_tonegen.h"  
  
#include "hardware/pio.h"  
#include "hardware/clocks.h"  
#include "hardware/pll.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "hardware/vreg.h"  
#include "hardware/structs/ssi.h"  
#include "hardware/structs/pll.h"  
#include "pico/multicore.h"  
#include <math.h>  
#include <string.h>  
  
// ДИАГНОСТИКА  
#define BOOST_DEBUG              0   // 0 — выключить отладочную печать блока разгона  
#define TG_PLL_DEBUG             0   // 0 — выключить отладочную печать подбора кандидатов  
#define PLL_RUNTIME_DEBUG        0   // 0 — выключить отладочную печать блока PLL  
  
// ============================================================================  
// МНОГОКРИТЕРИАЛЬНЫЙ АВТОТЮН PLL (ВЕРСИЯ С REFDIV И CTZ-ФИЛЬТРОМ)  
// ============================================================================  
#define TG_INT_PENALTY         500000000ULL // жёсткий запрет pio_int < 4  
#define TG_SPUR_MIN_OFFSET_HZ  1000000ULL   // запретный пояс FRAC8-спура, Гц  
#define TG_DMIN_CEILING        280000000ULL // гейт близкой гребёнки дизера (~6.5% от 2^32)  
#define TG_K8_BELT             (1u << 25)   // пояс отсечения у якорей k/8  
#define TG_SPUR_HORIZON_HZ     3000000ULL   // спур ближе 3 МГц — штраф за уровень  
  
// Структура детальной калькуляции штрафов  
struct MetricBreakdown {  
    uint32_t raw_step;  
    uint32_t frac;  
    uint32_t ctz_val;  
    uint64_t ctz_penalty;  
    uint64_t center_dist;  
    uint64_t mash_penalty;  
    uint64_t clk_penalty;  
    uint64_t int_penalty;  
    uint64_t spur_power_pen;   // аналитическая цена уровня FRAC8-спура (∝ A/D² + prox³)  
    uint64_t total_metric;  
};  
  
// ============================================================================  
// ПРОТОТИПЫ ВНУТРЕННИХ ФУНКЦИЙ ФАЙЛА  
// ============================================================================  
static void detach_peripheral_clock();  
static void __not_in_flash_func(tg_set_clk_sys)(const TgPllConfig& cfg, uint32_t vsel);  
void __not_in_flash_func(tonegen_dither_reset)(void);  
static TgToneParams calculate_raw_params_mhz(uint64_t clk_sys_hz, uint64_t mhz_target);  
TgPllConfig tonegen_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit);  
static uint64_t tg_pll_metric_verbose(uint64_t clk_sys_hz, uint64_t target_mhz, MetricBreakdown *b, TgToneParams *out = nullptr);  
static void __not_in_flash_func(tg_core1_entry)();  
static float frac_spur_level_db(uint16_t pio_int, uint8_t pio_frac);  
static uint64_t frac_spur_min_off_hz(uint16_t pio_int, uint8_t pio_frac,  
                                     uint64_t clk_sys_hz);  
void tonegen_clk_boost_arm(void);  
static void tg_rebuild_tone_table(uint64_t base_freq_mhz, uint64_t step_mhz);  
static vreg_voltage vsel_for(uint64_t clk_hz);  
static uint16_t vsel_to_mv(uint32_t vsel);  
  
static uint64_t tg_base_mhz = 0;   // базовый тон сетки, мГц  
static uint64_t tg_step_mhz = 0;   // шаг сетки, мГц  
  
// Глобальные профили тактирования  
TgPllConfig tg_pll_nominal   = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
TgPllConfig tg_pll_overclock = { 69,  3, 2, 138004025ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
TgPllConfig tg_pll_ceiling   = { 95,  3, 1, 380011083ULL, (uint32_t)VREG_VOLTAGE_1_30,    true,  1 };  
  
volatile bool tg_clk_boosted = false;  
  
// Паспортные лимиты (замена flash-значениям; поднимаются setter'ом)  
uint32_t tg_max_clk_sys_hz = 133000000u;  
uint32_t tg_max_pll_vco_hz = 1600000000u;  
  
// Коррекция кварца (замена Flash [XOSC_PPB])  
int32_t tg_xosc_corr_ppb = (int32_t)TG_XOSC_DEFAULT_PPB;  
  
void tonegen_set_clock_limits(uint32_t clk_sys_hz, uint32_t pll_vco_hz) {  
    if (clk_sys_hz)  tg_max_clk_sys_hz = clk_sys_hz;  
    if (pll_vco_hz)  tg_max_pll_vco_hz = pll_vco_hz;  
}  
void tonegen_set_xosc_corr_ppb(int32_t ppb) { tg_xosc_corr_ppb = ppb; }  
  
// PIO-программа меандра (цикл из 2 тактов)  
static const uint16_t tg_square_instructions[] = {  
    0xe001,  
    0xe000  
};  
static const pio_program_t tg_square_program = {  
    .instructions = tg_square_instructions,  
    .length = 2,  
    .origin = -1,  
    .pio_version = 0  
};  
  
// Физическое размещение массива в RAM (.time_critical — гарантия ОЗУ)  
TgToneParams __attribute__((section(".time_critical.tg_tones"))) tg_tones[TONEGEN_MAX_TONES];  
  
// Состояние генератора  
static PIO      tg_pio = pio0;  
static uint     tg_sm = 0;  
static uint     tg_offset = 0;  
static bool     tg_prog_loaded = false;  
static uint8_t  tg_pin = 255;  
static uint8_t  tg_ntones = 0;  
static tonegen_engine_t tg_engine = TONEGEN_ENGINE_DITHER;  
static bool     tg_running = false;  
  
static uint8_t current_active_tone = TG_TONE_NONE;  
static volatile bool tone_changed = false;  
static volatile bool dither_reset_pending = false;  // флаг сброса накопителей MASH-дизера  
  
// Межъядерный аппаратный спинлок  
static spin_lock_t* tg_spin_lock = nullptr;  
  
// Переменные обмена данными между ядрами (защищены спинлоком)  
static volatile uint32_t dds_step = 0;  
static volatile uint32_t target_pio_int = 8;  
static volatile uint32_t target_pio_frac8 = 0;  
// Глубина FRAC-дизера как битовая маска ГПСЧ, выбирается по pio_int  
// при смене тона: 0x03 → ±3, 0x01 → ±1, 0x00 → выключен.  
static volatile uint32_t tg_frac_dither_mask = 0x03u;  
volatile uint8_t tg_narrowband_mode = 0;   // 0 = обычный дизер, 1 = узкая полоса  
  
// Состояние встроенного ГПСЧ Xorshift32  
#define TG_RAND_SEED_INIT 0xACE1u  
static volatile uint32_t xorshift_state = TG_RAND_SEED_INIT;  
  
static uint32_t current_clk_sys_hz = 120000000;  
  
static double   cached_step_hz = 100.0;  
static uint32_t cached_base_freq_hz = 3500000;  
  
/**  
 * @brief Отсоединение периферийного домена clk_peri от системной шины.  
 *  
 * Переводит clk_peri на безопасный источник (PLL_USB 48 МГц), изолируя  
 * периферию от домена clk_sys на время перепрограммирования pll_sys.  
 */  
static void __not_in_flash_func(detach_peripheral_clock)() {  
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * 1000000, 48 * 1000000);  
}  
  
/**  
 * @brief Безопасное динамическое перепрограммирование pll_sys и напряжения vreg.  
 *  
 * Двухранговый сдвиг частоты и питания, пересчёт делителя XIP_SSI (BAUDR),  
 * аппаратный мониторинг LOCK, самопроверка частоты с 3 ретраями и  
 * фолбэк на номинал.  
 */  
static void __not_in_flash_func(tg_set_clk_sys)(const TgPllConfig& cfg_in, uint32_t vsel) {  
    TgPllConfig cfg = cfg_in;  
    if (cfg.refdiv == 0) cfg.refdiv = 1;   // защита от неинициализированного поля  
  
    // p1=1 недокументированно делит пополам на VCO < ~2.8 ГГц — отклоняем  
    if (cfg.p1 < 2) {  
#if TG_PLL_DEBUG  
        Serial.printf("[PLL] REJECT: p1=%lu forbidden — fallback nominal\n",  
                    (unsigned long)cfg.p1);  
#endif  
        cfg = tg_pll_nominal;  
    }  
  
    uint32_t target_clk_hz = (uint32_t)(((uint64_t)cfg.fbdiv * tg_calibrated_xosc_hz()) / (uint64_t)(cfg.refdiv * cfg.p1 * cfg.p2));  
    bool is_overclocking = (target_clk_hz > current_clk_sys_hz);  
  
    if (is_overclocking) {  
        vreg_set_voltage((vreg_voltage)vsel);  
        busy_wait_us(500);  
    }  
  
    detach_peripheral_clock();  
    uint32_t ints_status = save_and_disable_interrupts();  
  
    // clk_sys -> XOSC 12 МГц (SCK флэша 6 МГц, безопасно)  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                    tg_calibrated_xosc_hz(), tg_calibrated_xosc_hz());  
    // Округление ВВЕРХ до следующего кратного ref_hz: гарантирует, что  
    // целочисленное деление pll_init даст ровно cfg.fbdiv.  
    uint64_t ref_hz = ((uint64_t)tg_calibrated_xosc_hz() + cfg.refdiv - 1u) / cfg.refdiv;  
    uint32_t vco_nominal_hz = (uint32_t)(ref_hz * (uint64_t)cfg.fbdiv);  
  
#if PLL_RUNTIME_DEBUG  
    Serial.printf("[PLL] pre: CS=0x%08lX refdiv_hw=%lu XOSC=%lu kHz\n",  
                  (unsigned long)pll_sys_hw->cs,  
                  (unsigned long)(pll_sys_hw->cs & PLL_CS_REFDIV_BITS),  
                  (unsigned long)frequency_count_khz(CLOCKS_FC0_SRC_VALUE_XOSC_CLKSRC));  
#endif  
  
    uint32_t cs_pre  = pll_sys_hw->cs;  
  
    pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
  
    // === АППАРАТНЫЙ МОНИТОРИНГ ЗАЩЁЛКИ ФАЗЫ PLL (LOCK BIT) ===  
    uint32_t timeout = 20000;  
    while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
    bool pll_locked = (timeout != 0);  
  
    // === BAUDR: ОГРАНИЧЕНИЕ ТАКТИРОВАНИЯ ФЛЭША СТРОГО <= 48 МГц ===  
    const uint32_t FLASH_SAFE_MAX_HZ = 48000000u;  
    uint32_t ssi_baud = (uint32_t)((target_clk_hz + FLASH_SAFE_MAX_HZ - 1) / FLASH_SAFE_MAX_HZ);  
  
    ssi_baud = (ssi_baud + 1u) & ~1u;   // округление вверх до чётного (требование SSI)  
    if (ssi_baud < 2u)  ssi_baud = 2u;  
    if (ssi_baud > 34u) ssi_baud = 34u;  
  
    ssi_hw->ssienr = 0;                 // выключить SSI — иначе baudr не запишется  
    ssi_hw->baudr  = ssi_baud;  
    ssi_hw->ssienr = 1;                 // включить обратно  
  
    // clk_sys -> PLL_SYS (целевая частота)  
    clock_configure(clk_sys,  
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                    target_clk_hz, target_clk_hz);  
  
    restore_interrupts(ints_status);  
  
#if PLL_RUNTIME_DEBUG  
    Serial.printf("[PLL] pre: CS=0x%08lX refdiv_hw=%lu\n",  
                  (unsigned long)cs_pre,  
                  (unsigned long)(cs_pre & PLL_CS_REFDIV_BITS));  
    busy_wait_us(20);  
    uint32_t post_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
    Serial.printf("[PLL] post-switch clk=%u kHz (expect=%u kHz)\n",  
                  (unsigned)post_khz, (unsigned)(cfg.clk_sys_hz / 1000u));  
#endif  
  
    // --- Самопроверка: при промахе повторяем ВЕСЬ цикл переключения ---  
    bool locked_ok = false;  
    uint32_t meas_khz = 0;  
    uint32_t expect_khz = cfg.clk_sys_hz / 1000u;  
  
    for (int attempt = 0; attempt < 3; attempt++) {  
        busy_wait_us(300);  
        meas_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);  
        if (meas_khz > expect_khz * 997u / 1000u &&  
            meas_khz < expect_khz * 1003u / 1000u) {  
            locked_ok = true;  
            break;  
        }  
#if PLL_RUNTIME_DEBUG  
        Serial.printf("[PLL] MISMATCH: meas=%u kHz expect=%u kHz — retry %d\n",  
                      (unsigned)meas_khz, (unsigned)expect_khz, attempt);  
#endif  
        // полный ре-секвенс: назад на XOSC, переинициализация PLL, вперёд  
        uint32_t ints2 = save_and_disable_interrupts();  
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                        tg_calibrated_xosc_hz(), tg_calibrated_xosc_hz());  
        pll_init(pll_sys, cfg.refdiv, vco_nominal_hz, cfg.p1, cfg.p2);  
        timeout = 20000;  
        while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
#if TG_PLL_DEBUG  
        Serial.printf("[PLL] retry-dbg: LOCK=%d CS=0x%08lX FB=%lu PRIM=0x%08lX meas=%u kHz\n",  
                      (int)(pll_sys_hw->cs & PLL_CS_LOCK_BITS ? 1 : 0),  
                      (unsigned long)pll_sys_hw->cs,  
                      (unsigned long)pll_sys_hw->fbdiv_int,  
                      (unsigned long)pll_sys_hw->prim,  
                      (unsigned)meas_khz);  
#endif  
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                        target_clk_hz, target_clk_hz);  
        restore_interrupts(ints2);  
    }  
    uint32_t meas_khz_final = meas_khz;  
#if PLL_RUNTIME_DEBUG  
    if (!locked_ok) {  
        Serial.printf("[PLL] FALLBACK reason: pll_locked=%d last_meas=%u expect=%u kHz\n",  
                      (int)pll_locked, (unsigned)meas_khz_final, (unsigned)expect_khz);  
    }  
#endif  
  
    if (!locked_ok) {  
        // 3 промаха подряд — безопасный откат на номинал,  
        // чтобы таблица тонов не считалась от несуществующей частоты  
#if PLL_RUNTIME_DEBUG  
        Serial.printf("[PLL] FAIL x3 — fallback to nominal\n");  
#endif  
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,  
                        tg_calibrated_xosc_hz(), tg_calibrated_xosc_hz());  
  
        uint64_t ref_hz_fb = ((uint64_t)tg_calibrated_xosc_hz() + tg_pll_nominal.refdiv - 1u)  
                            / tg_pll_nominal.refdiv;  
        pll_init(pll_sys, tg_pll_nominal.refdiv,  
                (uint32_t)(ref_hz_fb * (uint64_t)tg_pll_nominal.fbdiv),  
                tg_pll_nominal.p1, tg_pll_nominal.p2);  
  
        timeout = 20000;  
        while (!(pll_sys_hw->cs & PLL_CS_LOCK_BITS) && --timeout);  
        clock_configure(clk_sys,  
                        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,  
                        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,  
                        tg_pll_nominal.clk_sys_hz, tg_pll_nominal.clk_sys_hz);  
        current_clk_sys_hz = tg_pll_nominal.clk_sys_hz;  
        return;  
    }  
  
    current_clk_sys_hz = meas_khz * 1000u;   // реальность, а не желаемое cfg.clk_sys_hz  
  
#if PLL_RUNTIME_DEBUG  
    Serial.printf("[PLL] sys_ctrl=0x%08lX selected=0x%02lX\n",  
                  (unsigned long)clocks_hw->clk[clk_sys].ctrl,  
                  (unsigned long)clocks_hw->clk[clk_sys].selected);  
    if (!pll_locked) {  
        Serial.printf("[PLL] !!! NO LOCK: fbdiv=%lu target=%lu Hz — выход НЕДОСТОВЕРЕН\n",  
                    (unsigned long)cfg.fbdiv, (unsigned long)target_clk_hz);  
    }  
    uint32_t real_clk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;  
    Serial.printf("[PLL] measured clk_sys=%lu Hz (target %lu)\n",  
                (unsigned long)real_clk, (unsigned long)target_clk_hz);  
#endif  
  
    if (!is_overclocking) {  
        busy_wait_us(100);  
        vreg_set_voltage((vreg_voltage)vsel);  
    }  
#if PLL_RUNTIME_DEBUG  
    Serial.printf("[PLL] CS=0x%08lX (refdiv=%lu)  FB=%lu  PRIM=0x%08lX\n",  
              (unsigned long)pll_sys_hw->cs,  
              (unsigned long)(pll_sys_hw->cs & PLL_CS_REFDIV_BITS),  
              (unsigned long)pll_sys_hw->fbdiv_int,  
              (unsigned long)pll_sys_hw->prim);  
#endif  
  
    current_clk_sys_hz = cfg.clk_sys_hz;  
}  
  
// Помощник автоматического определения напряжения ядра под частоту шины  
static vreg_voltage vsel_for(uint64_t clk_hz) {  
    if (clk_hz >= 250000000ULL)      return VREG_VOLTAGE_1_30;  
    else if (clk_hz >= 200000000ULL) return VREG_VOLTAGE_1_25;  
    else if (clk_hz > 133000000ULL)  return VREG_VOLTAGE_1_15;  
    return VREG_VOLTAGE_DEFAULT;  
}  
  
static uint16_t vsel_to_mv(uint32_t vsel) {  
    // vreg_voltage: VREG_VOLTAGE_0_80 == 5, шаг 50 мВ, максимум 1_30 == 1300 мВ  
    return (uint16_t)(900 + (vsel - VREG_VOLTAGE_0_90) * 50);  
}  
  
/** @brief Взвести флаг BOOST до следующего tonegen_init()/boost_enter().  
 *  Атомарно для RP2040 (single-core write). */  
void tonegen_clk_boost_arm(void) {  
    tg_clk_boosted = true;  
}  
  
void tonegen_clk_boost_enter(unsigned int target_freq_hz) {  
    if (target_freq_hz == 0) {  
        tg_clk_boosted = true;  
        return;  
    }  
  
    // 1. Поиск оптимума (один проход)  
    TgPllConfig opt = tonegen_find_optimal_pll(target_freq_hz, tg_effective_clk_max());  
  
    // 2. Сравнение метрик  
    uint64_t target_mhz = (uint64_t)target_freq_hz * 1000ULL;  
    TgToneParams dummy;  
    MetricBreakdown brk_opt;  
    uint64_t metric_opt = tg_pll_metric_verbose(opt.clk_sys_hz, target_mhz, &brk_opt, &dummy);  
    MetricBreakdown brk_cur;  
    uint64_t metric_cur = tg_pll_metric_verbose(current_clk_sys_hz, target_mhz, &brk_cur, &dummy);
  
    if (metric_opt >= metric_cur || opt.clk_sys_hz == current_clk_sys_hz) {  
        cached_base_freq_hz = target_freq_hz;  
        tg_base_mhz = (uint64_t)target_freq_hz * 1000ULL;   // мГц  
  
        // Перестраиваем таблицу под новые параметры  
        tg_rebuild_tone_table(tg_base_mhz, tg_step_mhz);  
  
        // Записываем новые параметры в защищённые спинлоком переменные.  
        // tonegen_set_tone(0) внутри взведёт tone_changed = true!  
        tonegen_set_tone(0);  
  
        // Микропауза: Core 1 гарантированно увидит флаг и применит шаг  
        // БЕЗ перезапуска ядра.  
        busy_wait_us(10);  
  
#if BOOST_DEBUG  
        Serial.printf("[BOOST  ] SKIP-RETUNE (SOFT TUNE) clk=%lu kHz  target=%lu Hz\n",  
                      (unsigned long)(current_clk_sys_hz / 1000ULL),  
                      (unsigned long)target_freq_hz);  
#endif  
        return;  
    }  
  
    // 3. Остановка Core 1 — PLL-переключение рвёт тактирование  
    multicore_reset_core1();  
    busy_wait_us(100);  
    cached_base_freq_hz = target_freq_hz;  
  
    // 4. Напряжение: профиль поиска с подстраховкой таблицей  
    uint32_t vsel = opt.vsel;  
    if (vsel == 0 || vsel < (uint32_t)VREG_VOLTAGE_1_10) {  
        vsel = (uint32_t)vsel_for(opt.clk_sys_hz);  
    }  
  
    // 5. Сдвиг частоты и питания (внутри: детач clk_peri, сброс PLL)  
    tg_set_clk_sys(opt, vsel);  
  
    // 6. Пересчёт таблиц под новую clk_sys  
    tg_rebuild_tone_table(tg_base_mhz, tg_step_mhz);  
    tonegen_set_tone(0);  
    tone_changed = true;          // страховка для Core 1  
  
    tg_pll_overclock = opt;       // активный профиль сеанса  
  
    if (tg_engine == TONEGEN_ENGINE_DITHER) {  
        multicore_launch_core1(tg_core1_entry);  
        tg_running = true;  
    }  
  
    uint32_t vco_print_mhz = (uint32_t)(((uint64_t)opt.fbdiv * tg_calibrated_xosc_hz()) / (1000000ULL * opt.refdiv));  
  
#if BOOST_DEBUG  
    Serial.printf("[BOOST  ] ON   clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)\n",  
                opt.clk_sys_hz / 1000000.0,  
                (unsigned long)opt.refdiv,  
                (unsigned long)opt.fbdiv,  
                (unsigned long)opt.p1,  
                (unsigned long)opt.p2,  
                (unsigned long)vco_print_mhz,  
                (unsigned)vsel_to_mv(vsel),  
                opt.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,  
                (unsigned)ssi_hw->baudr);  
#endif  
    tg_clk_boosted = true;  
}  
  
/**  
 * @brief Выход из boost: возврат на tg_pll_nominal и штатное напряжение.  
 * Понижение VREG выполняется ПОСЛЕ снижения частоты (логика в tg_set_clk_sys).  
 */  
void tonegen_clk_boost_exit(void) {  
    if (!tg_clk_boosted) return;  
  
    tg_set_clk_sys(tg_pll_nominal, VREG_VOLTAGE_DEFAULT);  
    tg_rebuild_tone_table((uint64_t)cached_base_freq_hz * 1000ULL,  
                          (uint64_t)(cached_step_hz * 1000.0));  
  
    // Пересчёт таблицы тонов обратно под номинальную clk_sys  
    uint32_t save = spin_lock_blocking(tg_spin_lock);  
    uint64_t base_mhz = (uint64_t)cached_base_freq_hz * 1000ULL;  
    uint64_t step_mhz = (uint64_t)(cached_step_hz * 1000.0);  
    for (int i = 0; i < TONEGEN_MAX_TONES; i++) {  
        tg_tones[i] = calculate_raw_params_mhz(current_clk_sys_hz,  
                                               base_mhz + (uint64_t)i * step_mhz);  
    }  
    current_active_tone = TG_TONE_NONE;  
    tone_changed = true;  
    spin_unlock(tg_spin_lock, save);  
  
    uint64_t vco_mhz = (tg_calibrated_xosc_hz() * (uint64_t)tg_pll_nominal.fbdiv)  
                       / ((uint64_t)tg_pll_nominal.refdiv * 1000000ULL);  
  
#if BOOST_DEBUG  
    Serial.printf("[BOOST  ] OFF  clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4llu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)\n",  
                  tg_pll_nominal.clk_sys_hz / 1000000.0,  
                  (unsigned long)tg_pll_nominal.refdiv,  
                  (unsigned long)tg_pll_nominal.fbdiv,  
                  (unsigned long)tg_pll_nominal.p1,  
                  (unsigned long)tg_pll_nominal.p2,  
                  (unsigned long long)vco_mhz,  
                  (unsigned)vsel_to_mv((uint32_t)VREG_VOLTAGE_DEFAULT),  
                  tg_pll_nominal.clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,  
                  (unsigned)ssi_hw->baudr);  
#endif  
    tg_clk_boosted = false;  
}  
  
/**  
 * @brief Одношаговый ГПСЧ Xorshift32 с глобальным состоянием в ОЗУ.  
 */  
static inline __attribute__((always_inline)) uint32_t __not_in_flash_func(tg_xorshift32)() {  
    uint32_t x = xorshift_state;  
    x ^= x << 13;  
    x ^= x >> 17;  
    x ^= x << 5;  
    xorshift_state = x;  
    return x;  
}  
  
/**  
 * @brief Точка входа второго ядра (Core 1): бесконечный горячий цикл дизеринга.  
 *        Полный перенос vfo_core1_entry: MASH-1 / MASH-1-1 по маске,  
 *        TPDF FRAC-дизер, RPDF на целую часть шага.  
 */  
static void __not_in_flash_func(tg_core1_entry)() {  
    // Локальные резидентные копии параметров тона  
    int32_t  l_step  = 0;  
    int32_t  l_int   = 8;  
    int32_t  l_frac  = 0;  
    uint32_t l_dmask = 0;  
  
    // Состояние накопителей  
    uint32_t loc_acc1 = 0;  
    uint32_t loc_acc2 = 0;  
    uint32_t loc_m2_carry_prev = 0;  
    uint32_t loc_rand_state = xorshift_state;  
  
    // Прямой указатель на регистр SM PIO  
    volatile uint32_t *clkdiv_reg = &tg_pio->sm[tg_sm].clkdiv;  
  
    const uint32_t poll_mask = (1u << 6) - 1u;  
    uint32_t poll_cnt = 0;  
  
    while (true) {  
        // Разряженный опрос флага обмена данными (внутри лога)  
        if ((poll_cnt & poll_mask) == 0u) {  
            if (__builtin_expect(tone_changed, 0)) {  
                uint32_t save = spin_lock_blocking(tg_spin_lock);  
                l_step  = (int32_t)dds_step;  
                l_int   = (int32_t)target_pio_int;  
                l_frac  = (int32_t)target_pio_frac8;  
                l_dmask = tg_frac_dither_mask;  
                if (dither_reset_pending) {  
                    // Полный сброс: только при инициализации/перестройке  
                    loc_acc1 = 0;  
                    loc_acc2 = 0;  
                    loc_m2_carry_prev = 0;  
                    loc_rand_state = xorshift_state;  
                    dither_reset_pending = false;  
                }  
                // При обычной смене тона аккумуляторы сохраняются — фаза не прыгает.  
                tone_changed = false;  
                spin_unlock(tg_spin_lock, save);  
            }  
        }  
        poll_cnt++;  
  
        // === 1. ГПСЧ XORSHIFT32 ===  
        loc_rand_state ^= loc_rand_state << 13;  
        loc_rand_state ^= loc_rand_state >> 17;  
        loc_rand_state ^= loc_rand_state << 5;  
  
        // === 2. ДИЗЕРИНГ ЦЕЛОЙ ЧАСТИ ШАГА (4 бита RPDF) ===  
        int32_t r_bits = (int32_t)(loc_rand_state & 0x0Fu);  
        int32_t step = l_step + ((r_bits << 1) - 15);  
  
        int32_t total_correction = 0;  
  
        // === 3. АВТОНОМНОЕ ПЕРЕКЛЮЧЕНИЕ ПОРЯДКА MASH ===  
        if (l_dmask == 0u) {  
            // ВЧ-РЕЖИМ: Однокаскадный жёсткий MASH-1 (0 или +1)  
            uint32_t old_acc = loc_acc1;  
            loc_acc1 += (uint32_t)step;  
            total_correction = (loc_acc1 < old_acc);  
        } else {  
            // НЧ-РЕЖИМ: Каскадный прецизионный MASH-1-1  
            uint32_t carry1 = __builtin_add_overflow(loc_acc1, (uint32_t)step, &loc_acc1) ? 1u : 0u;  
            uint32_t carry2 = __builtin_add_overflow(loc_acc2, loc_acc1, &loc_acc2) ? 1u : 0u;  
  
            total_correction = (int32_t)carry1 + (int32_t)carry2 - (int32_t)loc_m2_carry_prev;  
            loc_m2_carry_prev = carry2;  
        }  
  
        // === 4. СБОРКА ТЕКУЩЕЙ ФРАКЦИИ ===  
        int32_t current_frac = l_frac + total_correction;  
        int32_t current_int  = l_int;  
  
        // === 5. МАТЕМАТИЧЕСКИЙ ТРЕУГОЛЬНЫЙ TPDF ФРАК-ДИЗЕР ===  
        if (l_dmask != 0u) {  
            uint32_t r1 = (loc_rand_state >> 8)  & l_dmask;  
            uint32_t r2 = (loc_rand_state >> 16) & l_dmask;  
            current_frac += (int32_t)(r1 + r2) - (int32_t)l_dmask;  
        }  
  
        // Выравнивание знака и переноса фракции (current_frac >> 8)  
        current_int  += (current_frac >> 8);  
        current_frac &= 0xFF;  
  
        // Финальная запись конфигурационного слова в регистр clkdiv PIO  
        *clkdiv_reg = ((uint32_t)current_int << 16) | ((uint32_t)current_frac << 8);  
    }  
}  
  
// Минимальное КОЛЬЦЕВОЕ расстояние dds_step до якорей k/8 (единицы 2^-32).  
// Якоря: k * 2^29, k = 0..7. Максимум результата = 2^28.  
static uint32_t dds_step_min_dist_to_k8(uint32_t step) {  
    uint32_t best = (1u << 28);          // теоретический максимум  
    for (uint32_t k = 0; k < 8; k++) {  
        uint32_t a = k << 29;            // якорь k/8 шкалы  
        uint32_t d = step - a;           // беззнаковая разность — уже на кольце  
        if (d > 0x80000000u) d = 0u - d; // min(d, 2^32 - d)  
        if (d < best) best = d;  
    }  
    return best;  
}  
  
// Штраф за мощность спура: pen = 2^30 · (A/16384) · (64/D)²  
static uint64_t frac_spur_power_pen(uint16_t pio_int, uint8_t pio_frac) {  
    if (pio_frac == 0) return 0;  
    uint64_t D_x256 = (uint64_t)pio_int * 256ULL + pio_frac;  
    uint64_t A      = (uint64_t)pio_frac * (256u - pio_frac);       // ≤16384  
    uint64_t pen = ((uint64_t)A << 46) / (D_x256 * D_x256);  
    return pen;  
}  
  
static uint64_t tg_pll_metric_verbose(uint64_t clk_sys_hz, uint64_t target_mhz,  
                                      MetricBreakdown *b, TgToneParams *out) {  
    TgToneParams test = calculate_raw_params_mhz(clk_sys_hz, target_mhz);  
    if (out) *out = test;  
  
    memset(b, 0, sizeof(MetricBreakdown));  
    b->frac = test.pio_frac;  
    b->raw_step = test.dds_step;  
  
    // 1. Идеальный узел: ни дробной части, ни остатка DDS  
    if (test.pio_frac == 0 && test.dds_step == 0) {  
        return 0;  
    }  
  
    // 1a. Уровень FRAC8-спура: штраф ~ A/D²  
    // Отстройку спура считаем ОДИН раз и переиспользуем в level-пенальти ниже.  
    uint64_t spur_off = frac_spur_min_off_hz(test.pio_int, test.pio_frac, clk_sys_hz);  
  
    b->spur_power_pen = frac_spur_power_pen(test.pio_int, test.pio_frac);  
    if (spur_off != UINT64_MAX && spur_off < TG_SPUR_HORIZON_HZ) {  
        uint64_t prox   = TG_SPUR_HORIZON_HZ - spur_off;  
        uint64_t prox_q = prox / 1024ULL;  
        b->spur_power_pen += (prox_q * prox_q * prox_q) << 4;  
    }  
    b->total_metric += b->spur_power_pen;           // позиция спура учтена  
  
    // 2. Цена frac=0: вся дробная работа ложится на MASH  
    if (test.pio_frac == 0) {  
        uint64_t act = (test.dds_step <= (0xFFFFFFFFu - test.dds_step))  
                    ? (uint64_t)test.dds_step  
                    : (0xFFFFFFFFu - (uint64_t)test.dds_step);  
        uint64_t frac0_pen = (act >> 4) + (1ULL << 36);  
        b->total_metric += frac0_pen;  
    }  
  
    // 2b. Простые дроби FRAC8 → короткий период паттерна → когерентный спур  
    {  
        uint32_t f = test.pio_frac;  
        uint32_t ctz = (f == 0) ? 8 : (uint32_t)__builtin_ctz(f);  
        b->ctz_val = ctz;  
        if (ctz >= 5) {                                   // знаменатель ≤ 8  
            b->ctz_penalty = (1ULL << (ctz + 26)) + (1ULL << 33);  
        }  
        b->total_metric += b->ctz_penalty;  
    }  
  
    // 3. Подошва: clk·int к эталону 400M·55  
    {  
        const uint64_t DEN_REF = 400000000ULL * 55ULL;  
        uint64_t dens = clk_sys_hz * (uint64_t)test.pio_int;  
        if (dens < DEN_REF) {  
            b->clk_penalty = ((DEN_REF - dens) * (1ULL << 30)) / DEN_REF;  
        } else {  
            b->clk_penalty = 0;  
        }  
        b->total_metric += b->clk_penalty;  
    }  
  
    // 3b. Близость dds_step к якорям k/8 → idle-тоны MASH  
    if (test.dds_step != 0) {  
        uint32_t dist_k8 = dds_step_min_dist_to_k8(test.dds_step);  
        b->center_dist = dist_k8;  
        if (dist_k8 < TG_K8_BELT) {  
            uint64_t deficit = (uint64_t)(TG_K8_BELT - dist_k8);  
            b->mash_penalty = (deficit * deficit) >> 14;  
        }  
        b->total_metric += b->mash_penalty;  
    }  
  
    // 4. Запрет малых INT  
    if (test.pio_int < 4) {  
        b->int_penalty = TG_INT_PENALTY;  
        b->total_metric += b->int_penalty;  
    }  
  
    // Level-пенальти: спуры хуже -30 dBc дорожают квадратично,  
    // с коэффициентом дальности kdist: у несущей штраф полный,  
    // за горизонтом ФНЧ — остаточный.  
    if (test.pio_frac != 0 && test.pio_int != 0) {  
        float lvl_db = frac_spur_level_db(test.pio_int, test.pio_frac);  
        if (lvl_db > -30.0f) {  
            float over = lvl_db + 30.0f;  
            uint64_t base = (uint64_t)(over * over) << 22;  
  
            float kdist;  
            if      (spur_off == UINT64_MAX)          kdist = 1.0f;  
            else if (spur_off >= TG_SPUR_HORIZON_HZ)  kdist = 0.1f;  
            else kdist = 1.0f - 0.9f * ((float)spur_off / (float)TG_SPUR_HORIZON_HZ);  
  
            uint64_t pen = (uint64_t)((double)base * kdist);  
            b->spur_power_pen += pen;  
            b->total_metric   += pen;  
        }  
    }  
  
    return b->total_metric;  
}  
  
// Метрика кандидата  
static uint64_t tg_pll_metric(uint64_t clk_sys_hz, uint64_t target_mhz, TgToneParams *out) {  
    MetricBreakdown b;  
    return tg_pll_metric_verbose(clk_sys_hz, target_mhz, &b, out);  
}  
  
/**  
 * @brief Прецизионный расчёт параметров PIO-делителя для целевой частоты  
 *        в миллигерцах (0.001 Гц). Целочисленная арифметика.  
 */  
static TgToneParams calculate_raw_params_mhz(uint64_t clk_sys_hz, uint64_t mhz_target) {  
    // Границы КВ-диапазона в миллигерцах (1.0 .. 40.0 МГц)  
    if (mhz_target < 100000000ULL)   mhz_target = 100000000ULL;  
    if (mhz_target > 40000000000ULL) mhz_target = 40000000000ULL;  
  
    uint64_t clocks_per_period = 2ULL;  
    uint64_t vfo_denom = mhz_target * clocks_per_period;  
  
    // Делитель PIO в формате 16.8: int + frac8/256  
    uint64_t pio_div_fixed8 = ((clk_sys_hz * 256ULL) * 1000ULL) / vfo_denom;  
    uint8_t  frac8 = (uint8_t)(pio_div_fixed8 & 0xFFu);  
  
    TgToneParams params;  
  
    params.pio_int  = pio_div_fixed8 >> 8;  
    params.pio_frac = frac8;  
  
    params.target_freq_chz = (uint32_t)(mhz_target / 10ULL);  
  
    if (params.pio_int < 2) {  
        params.pio_int  = 2;  
        params.pio_frac = 0;  
    }  
  
    // Суб-LSB остаток: ошибка квантования ниже одного frac8-LSB делителя.  
    uint64_t clk_sys_rem   = ((clk_sys_hz * 256ULL) * 1000ULL) % vfo_denom;  
    uint64_t intermediate  = (clk_sys_rem << 16) / vfo_denom;  
    uint64_t remainder_low = (clk_sys_rem << 16) % vfo_denom;  
  
    params.dds_step = (uint32_t)((intermediate << 16)  
                               + ((remainder_low << 16) / vfo_denom));  
  
    // Принудительная нечётность шага: gcd(dds_step, 2^32) = 1,  
    // период паттерна = 2^32 отсчётов → гребёнка в шумовую полку.  
    if (params.dds_step != 0) params.dds_step |= 1u;  
  
    return params;  
}  
  
// Хелпер: ближайшая к несущей FRAC8-спуровая линия среди первых 4 гармоник  
// периода переполнения дробного байта для делителя D = int + frac/256.  
static uint64_t frac_spur_min_off_hz(uint16_t pio_int, uint8_t pio_frac, uint64_t clk_sys_hz) {  
    if (pio_frac == 0) return UINT64_MAX;               // деление целочисленное — спуров нет  
    uint64_t div_x256 = (uint64_t)pio_int * 256ULL + pio_frac;  
    if (div_x256 == 0) return UINT64_MAX;  
    uint64_t best = UINT64_MAX;  
    for (uint32_t k = 1; k <= 4; k++) {  
        uint32_t m  = ((uint32_t)pio_frac * k) & 0xFFu;  
        uint32_t fd = (m < 256u - m) ? m : 256u - m;    // 0..128  
        if (fd == 0) continue;                          // гармоника на тоне — пропускаем  
        uint64_t off = clk_sys_hz * fd / div_x256;      // отстройка гармоники, Гц  
        if (off < best) best = off;  
    }  
    return best;  
}  
  
// Оценка уровня главной пары FRAC8-спуров, dBc.  
// Модель: пилообразная ФМ с глубиной Δφ = f·(256−f)/(256·D) такта VCO;  
// L1 ≈ 10·log10( f·(256−f) / (32·π²·D²) ) — потолок, без sinc-огибающей.  
static float frac_spur_level_db(uint16_t pio_int, uint8_t pio_frac) {  
    if (pio_frac == 0 || pio_int == 0) return -999.0f;  
    float D = (float)pio_int + (float)pio_frac / 256.0f;  
    float f = (float)pio_frac;  
    float p2 = f * (256.0f - f);  
    float lvl = 10.0f * log10f(p2 / (32.0f * 9.8696f * D * D));  
    return lvl - 21.0f;   // поправка, откалиброванная на 3-х измеренных точках  
}  
  
// ============================================================================  
// МАТРИЧНЫЙ СКАН PLL: refdiv × fbdiv × (p1,p2) с полным арбитражем  
// ============================================================================  
TgPllConfig tonegen_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit) {  
  
    uint64_t crystal_hz = tg_calibrated_xosc_hz();  
    uint64_t target_mhz = (uint64_t)target_frequency_hz * 1000ULL; // миллигерцы  
  
    uint64_t clk_limit = (max_clk_limit == 0) ? tg_effective_clk_max() : max_clk_limit;  
  
    // Дефолтная безопасная конфигурация на случай сбоя  
    TgPllConfig best_pll = { 133, 6, 2, 133000000ULL, (uint32_t)VREG_VOLTAGE_DEFAULT, false, 1 };  
  
    // Границы КВ-диапазона (1.0 .. 30.0 МГц) в миллигерцах  
    if (target_mhz < 1000000000ULL)  target_mhz = 1000000000ULL;  
    if (target_mhz > 30000000000ULL) target_mhz = 30000000000ULL;  
  
    const uint64_t min_allowed_clk = TG_CLK_SYS_MIN_HZ;  
    const uint64_t max_allowed_clk = clk_limit;  
  
#if TG_PLL_DEBUG  
    Serial.printf("[PLLDBG] limits: clk_sys<=%.1f MHz, VCO<=%.0f MHz\n",  
              tg_effective_clk_max() / 1e6, tg_effective_pll_max() / 1e6);  
    Serial.printf("[PLLDBG] scan range: %.3f .. %.3f MHz (target=%u)\n",  
              min_allowed_clk / 1e6, max_allowed_clk / 1e6, target_frequency_hz);  
#endif  
  
    // ---- Классификаторы и состояние арбитража ----  
    uint64_t min_dds_metric    = UINT64_MAX;  
    bool     best_forbidden    = true;  
    uint32_t n_arbit           = 0;  
  
    // --- отдельный трекер лучшего frac=0-кандидата ---  
    uint64_t best_frac0_metric   = UINT64_MAX;  
    uint64_t best_frac0_clk      = 0;  
    uint32_t best_frac0_fbdiv    = 0, best_frac0_p1 = 0, best_frac0_p2 = 0;  
    uint32_t best_frac0_refdiv   = 0;  
    uint32_t n_frac0_seen        = 0;  
  
#if TG_PLL_DEBUG  
    struct CandLog {  
        uint64_t clk, vco, metric, spur_off;  
        uint32_t fbdiv, p1, p2, refdiv, step;  
        uint16_t pio_int;  
        uint8_t  pio_frac;  
        bool     forbidden, clean;  
        float    spur_lvl;  
    };  
    CandLog top10[20];  
    for (int i = 0; i < 20; i++) top10[i].metric = UINT64_MAX, top10[i].clk = 0;  
#endif  
  
    // ---- Матричный скан: refdiv × fbdiv × (p1,p2) ----  
    for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
        uint64_t ref_hz = crystal_hz / refdiv;   // REFDIV: 1,2,3 → 12,6,4 МГц  
  
        for (uint32_t p1 = 2; p1 <= 7; p1++) {   // p1 >= 2: режим p1=1 делит ровно на 2 при VCO < ~2.8 ГГц  
            for (uint32_t p2 = 1; p2 <= 7; p2++) {  
                if (p1 < p2) continue;   // ВЧ-правило: старший постделитель первым  
                uint32_t pdiv_total = p1 * p2;  
                for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
                    uint64_t vco_hz = ref_hz * fbdiv;  
  
                    if (vco_hz < 750000000ULL) continue;                    // нижний предел VCO (даташит)  
                    if (vco_hz > tg_effective_pll_max()) continue;          // верхний предел VCO (паспорт)  
  
                    uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;  
                    if (clk_sys_hz < min_allowed_clk ||  
                        clk_sys_hz > clk_limit) continue;  
  
                    // Единая метрика кандидата  
                    TgToneParams test;  
                    MetricBreakdown brk;  
                    uint64_t current_dds_metric =  
                        tg_pll_metric_verbose(clk_sys_hz, target_mhz, &brk, &test);  
  
                    if (test.pio_int < 2) continue;  
  
                    // Гейт плотности коррекции clk·int.  
                    // max(clk·int) ≈ clk_max² / (2·f_out); берём min(4e9, 90% потолка),  
                    // чтобы скан не пустел на ВЧ.  
                    {  
                        uint64_t f_out_hz = target_mhz / 1000ULL;  
                        uint64_t prod_max = (max_allowed_clk * max_allowed_clk) / (2ULL * f_out_hz);  
                        uint64_t gate     = 4000000000ULL;  
                        uint64_t cap      = (prod_max * 9ULL) / 10ULL;  
                        if (gate > cap) gate = cap;  
                        if (clk_sys_hz * (uint64_t)test.pio_int < gate) continue;  
                    }  
  
                    // === КЛАСС 1: запретный пояс FRAC8 (многогармонический, k=1..4) ===  
                    uint64_t spur_off_hz = frac_spur_min_off_hz(test.pio_int,  
                                                                test.pio_frac,  
                                                                clk_sys_hz);  
  
                    bool simple_frac = false;  
                    // Вето простых дробей действует только на ВЧ/СЧ (малые/средние int).  
                    // На НЧ гармоники падают как ~1/D² — вето отключаем.  
                    if (test.pio_frac != 0u && test.pio_int <= TG_SIMPLE_FRAC_INT_MAX) {  
                        uint32_t danger_zone = (test.pio_int <= 14) ? 16u : 8u;  
  
                        for (uint32_t n = 2; n <= 4 && !simple_frac; n++) {  
                            for (uint32_t k = 1; k < n; k++) {  
                                uint32_t anchor = (k * 256u) / n;  
                                uint32_t d = (test.pio_frac > anchor) ? test.pio_frac - anchor  
                                                                      : anchor - test.pio_frac;  
                                if (d < danger_zone) { simple_frac = true; break; }  
                            }  
                        }  
                    }  
  
                    bool frac_forbidden = (test.pio_frac != 0u) &&  
                                          ((spur_off_hz < TG_SPUR_MIN_OFFSET_HZ) || simple_frac);  
  
                    // === КЛАСС 2: чистота дизера по СЫРОМУ dmin ===  
                    uint32_t dd  = test.dds_step;  
                    uint32_t dmin_raw = (dd < (uint32_t)(0u - dd)) ? dd
                                                                   : (uint32_t)(0u - dd);  
                    bool candidate_clean = (dmin_raw < TG_DMIN_CEILING);  
                    bool     cand_frac_zero = (test.pio_frac == 0u);  
                    uint32_t cand_d8 = dds_step_min_dist_to_k8(test.dds_step);  
                    (void)cand_frac_zero; (void)cand_d8;  
                    n_arbit++;  
  
                    if (test.pio_frac == 0) {  
                        n_frac0_seen++;  
                        if (current_dds_metric < best_frac0_metric) {  
                            best_frac0_metric = current_dds_metric;  
                            best_frac0_clk    = clk_sys_hz;  
                            best_frac0_fbdiv  = fbdiv;  
                            best_frac0_p1     = p1;  
                            best_frac0_p2     = p2;  
                            best_frac0_refdiv = refdiv;  
                        }  
                    }  
  
#if TG_PLL_DEBUG  
                    // TOP-20 без дублей по clk_sys  
                    bool dup = false;  
                    for (int i = 0; i < 20; i++)  
                        if (top10[i].clk == clk_sys_hz) { dup = true; break; }  
                    if (!dup && current_dds_metric < top10[19].metric) {  
                        int pos = 19;  
                        while (pos > 0 && current_dds_metric < top10[pos-1].metric) pos--;  
                        for (int k = 19; k > pos; k--) top10[k] = top10[k-1];  
                        top10[pos].clk = clk_sys_hz;  top10[pos].vco = vco_hz;  
                        top10[pos].metric = current_dds_metric;  
                        top10[pos].fbdiv = fbdiv; top10[pos].p1 = p1;  
                        top10[pos].p2 = p2;         top10[pos].refdiv = refdiv;  
                        top10[pos].step = test.dds_step;  
                        top10[pos].pio_int = test.pio_int;  
                        top10[pos].pio_frac = test.pio_frac;  
                        top10[pos].spur_off = spur_off_hz;  
                        top10[pos].spur_lvl = frac_spur_level_db(test.pio_int,  
                                                                 test.pio_frac);  
                        top10[pos].forbidden = frac_forbidden;  
                        top10[pos].clean = candidate_clean;  
                    }  
#endif  
  
                    // === АРБИТРАЖ ===  
                    // Вето по запретному поясу FRAC8: кандидат с simple_frac  
                    // или спуром < TG_SPUR_MIN_OFFSET_HZ не может выиграть,  
                    // если существует хотя бы один незапретный кандидат.  
                    // Реализуем как доминирующий штраф — заодно сохраняется  
                    // fallback, если ВСЕ кандидаты запретные.  
                    uint64_t arb_metric = current_dds_metric;  
                    if (frac_forbidden) arb_metric += (1ULL << 40);  
                    // frac=0 с остаточным dds_step — близкий idle-тон DDS/MASH,  
                    // который ФНЧ НЕ снимает (отстройка ~единицы кГц из-за  
                    // кварцевой коррекции — точного целого попадания нет).  
                    // Штраф строго выше запретного пояса FRAC8 (2^42 > 2^40),  
                    // чтобы любой дробный кандидат с фильтруемым спуром побеждал.  
                    if (test.pio_frac == 0u && test.dds_step != 0u)  
                        arb_metric += (1ULL << 42);  
  
                    bool prefer = (arb_metric < min_dds_metric) ||  
                                  (arb_metric == min_dds_metric &&  
                                   clk_sys_hz > best_pll.clk_sys_hz);  
  
                    if (prefer) {  
                        best_forbidden  = frac_forbidden;  
                        min_dds_metric  = arb_metric;  
  
                        best_pll.fbdiv      = fbdiv;  
                        best_pll.p1         = p1;  
                        best_pll.p2         = p2;  
                        best_pll.refdiv     = refdiv;  
                        best_pll.clk_sys_hz = clk_sys_hz;  
                        best_pll.vsel       = (uint32_t)vsel_for(clk_sys_hz);  
                        best_pll.is_oc      = (clk_sys_hz > TG_CLK_SYS_NOMINAL_HZ);  
                    }  
                }  
            }  
        }  
    }  
  
#if TG_PLL_DEBUG  
    Serial.printf("[PLLDBG] frac0: seen=%lu", (unsigned long)n_frac0_seen);  
    if (n_frac0_seen) {  
        Serial.printf("  best: clk=%.3f MHz refdiv=%lu fbdiv=%lu p1=%lu p2=%lu metric=%llu\n",  
                      best_frac0_clk / 1e6,  
                      (unsigned long)best_frac0_refdiv,  
                      (unsigned long)best_frac0_fbdiv,  
                      (unsigned long)best_frac0_p1,  
                      (unsigned long)best_frac0_p2,  
                      (unsigned long long)best_frac0_metric);  
    } else {  
        Serial.printf("  (нет frac=0-кандидатов на этой сетке)\n");  
    }  
#endif  
  
    // === ЗАЩИТА ОТ ПУСТОГО СКАНА: встаём на макс. доказанный clk ===  
    if (n_arbit == 0) {  
        // Ищем наивысший clk_sys <= паспортного лимита с валидными делителями.  
        // Границы VCO те же, что в основном скане (750 МГц .. паспорт).  
        uint64_t fb_clk_target = tg_effective_clk_max();   // доказанный потолок clk  
        uint64_t best_fb_clk   = 0;  
        TgPllConfig fb_pll     = best_pll;                 // стартуем с дефолта 133M  
  
        for (uint32_t refdiv = 1; refdiv <= 3; refdiv++) {  
            uint64_t ref_hz = crystal_hz / refdiv;  
            for (uint32_t p1 = 2; p1 <= 7; p1++) {  
                for (uint32_t p2 = 1; p2 <= 7; p2++) {  
                    if (p1 < p2) continue;  
                    uint32_t pdiv_total = p1 * p2;  
                    for (uint32_t fbdiv = 16; fbdiv <= 320; fbdiv++) {  
                        uint64_t vco_hz = ref_hz * fbdiv;  
                        if (vco_hz < 750000000ULL) continue;  
                        if (vco_hz > tg_effective_pll_max()) continue;  
                        uint64_t clk_sys_hz = vco_hz / (uint64_t)pdiv_total;  
                        if (clk_sys_hz < min_allowed_clk) continue;  
                        if (clk_sys_hz > fb_clk_target)   continue;   // не выше доказанного  
                        if (clk_sys_hz > best_fb_clk) {  
                            best_fb_clk        = clk_sys_hz;  
                            fb_pll.fbdiv       = fbdiv;  
                            fb_pll.p1          = p1;  
                            fb_pll.p2          = p2;  
                            fb_pll.refdiv      = refdiv;  
                            fb_pll.clk_sys_hz  = clk_sys_hz;  
                            fb_pll.vsel        = (uint32_t)vsel_for(clk_sys_hz);  
                            fb_pll.is_oc       = (clk_sys_hz > TG_CLK_SYS_NOMINAL_HZ);  
                        }  
                    }  
                }  
            }  
        }  
  
#if TG_PLL_DEBUG  
        if (best_fb_clk > 0)  
            Serial.printf("[PLLDBG] SCAN EMPTY — fallback to passport max clk=%.3f MHz\n",  
                          best_fb_clk / 1e6);  
        else  
            Serial.printf("[PLLDBG] SCAN EMPTY — fallback to nominal 133 MHz\n");  
#endif  
        return (best_fb_clk > 0) ? fb_pll : best_pll;  
    }  
  
    // Страховка: лучше fallback, чем мусор.  
    if (best_pll.clk_sys_hz * 1ULL <= 0) {   // заглушка, best_pll валиден  
        return best_pll;  
    }  
  
#if TG_PLL_DEBUG  
    // === МОДЕРНИЗИРОВАННЫЙ ВЫРАВНЕННЫЙ ВЫВОД ТАБЛИЦЫ TOP-20 ===  
    Serial.println(F("[PLLDBG]   #  |  clk_sys    |    VCO       | R |  F  | P1| P2|    Metric   | Frac | Spur_Off  |  Spur_Lvl  | Статус"));  
    Serial.println(F("[PLLDBG] -----+-------------+--------------+---+-----+---+---+-------------+------+-----------+------------+------------------"));  
  
    for (int i = 0; i < 20; i++) {  
        if (top10[i].clk == 0) break;  
  
        unsigned long long spur_k_hz = (top10[i].spur_off == UINT64_MAX) ? 0ULL : (top10[i].spur_off / 1000ULL);  
  
        const char* forbidden_str = top10[i].forbidden ? "[FORB]" : "      ";  
        const char* clean_str     = top10[i].clean     ? "       " : " [DIRTY]";  
  
        Serial.printf("[PLLDBG] [%2d] | %7.3f MHz | %8.3f MHz | %1u | %3lu | %1lu | %1lu | %11llu | %4u | %5llu kHz | %6.1f dBc | %s%s\n",  
                i,  
                top10[i].clk / 1000000.0,  
                top10[i].vco / 1000000.0,  
                (unsigned)top10[i].refdiv,  
                (unsigned long)top10[i].fbdiv,  
                (unsigned long)top10[i].p1,  
                (unsigned long)top10[i].p2,  
                (unsigned long long)top10[i].metric,  
                (unsigned)top10[i].pio_frac,  
                spur_k_hz,  
                (double)top10[i].spur_lvl,  
                forbidden_str,  
                clean_str);  
    }  
    Serial.println(F("[PLLDBG] -----+-------------+--------------+---+-----+---+---+-------------+------+-----------+------------+------------------"));  
  
    // Победитель: пересчёт параметров для печати  
    TgToneParams wp;  
    tg_pll_metric(best_pll.clk_sys_hz, target_mhz, &wp);  
    uint64_t w_spur = frac_spur_min_off_hz(wp.pio_int, wp.pio_frac,  
                                         best_pll.clk_sys_hz);  
    float w_lvl = frac_spur_level_db(wp.pio_int, wp.pio_frac);  
    Serial.printf("[PLLDBG] WINNER: clk=%.3f MHz refdiv=%lu fbdiv=%lu p1=%lu p2=%lu "  
                  "metric=%llu  int=%u frac=%u step=0x%08lX  spur_off=%llu kHz "  
                  "spur_lvl=%.1f dBc%s\n",  
                best_pll.clk_sys_hz / 1e6,  
                (unsigned long)best_pll.refdiv,  
                (unsigned long)best_pll.fbdiv,  
                (unsigned long)best_pll.p1,  
                (unsigned long)best_pll.p2,  
                (unsigned long long)min_dds_metric,  
                (unsigned)wp.pio_int, (unsigned)wp.pio_frac,  
                (unsigned long)wp.dds_step,  
                (unsigned long long)(w_spur == UINT64_MAX ? 0 : w_spur / 1000ULL),  
                (double)w_lvl,  
                best_forbidden ? " [FORBIDDEN]" : "");  
#endif  
    (void)best_forbidden;  
    return best_pll;  
}  
  
  
// === ПОТОКОБЕЗОПАСНЫЙ ПЕРЕСЧЁТ ТАБЛИЦЫ ТОНОВ ===  
static void tg_rebuild_tone_table(uint64_t base_freq_mhz, uint64_t step_mhz) {  
    for (int i = 0; i < TONEGEN_MAX_TONES; i++) {  
        uint64_t tone_freq_mhz = base_freq_mhz + ((uint64_t)i * step_mhz);  
        tg_tones[i] = calculate_raw_params_mhz(current_clk_sys_hz, tone_freq_mhz);  
    }  
    // Сбросить кэш активного тона, чтобы следующая установка применила новые значения  
    current_active_tone = TG_TONE_NONE;  
}  
  
  
// === ИНИЦИАЛИЗАЦИЯ И СТАРТ СИСТЕМЫ ===  
/**  
 * @brief Полная инициализация и запуск генератора (PLL + PIO + дизеринг).  
 *  
 * Сквозная синхронизация частотного плана: матричный автотюн PLL под  
 * целевую сетку, настройка fixed-point Q8.8 делителей PIO и 32-битного  
 * DDS-остатка строго на базе физически захваченной частоты clk_sys.  
 */  
bool tonegen_init(uint8_t pin, uint32_t base_hz, double step_hz,  
                  uint8_t ntones, tonegen_engine_t engine,  
                  uint32_t clk_limit_hz) {  
    if (ntones == 0 || ntones > TONEGEN_MAX_TONES) return false;  
  
    tg_pin    = pin;  
    tg_ntones = ntones;  
    tg_engine = engine;  
  
    cached_base_freq_hz = base_hz;  
    cached_step_hz      = step_hz;  
  
    // Жёсткий перезапуск изолированного ядра перед изменением таблиц тонов  
    if (tg_running) {  
        multicore_reset_core1();  
        tg_running = false;  
    }  
  
    // Глушим выход ДО передёргивания PLL: работающий SM во время  
    // перезахвата PLL выплюнул бы меандр на съезжающей частоте.  
    if (tg_prog_loaded) {  
        pio_sm_set_enabled(tg_pio, tg_sm, false);  
        pio_sm_set_consecutive_pindirs(tg_pio, tg_sm, tg_pin, 1, false);  
    }  
  
    current_active_tone = TG_TONE_NONE;  
    xorshift_state = TG_RAND_SEED_INIT + time_us_32();  
  
    if (tg_spin_lock == nullptr) {  
        int lock_id = spin_lock_claim_unused(true);  
        tg_spin_lock = spin_lock_init(lock_id);  
    }  
  
    // Верхний лимит автотюна: аргумент вызова или паспорт  
    uint64_t limit = (clk_limit_hz != 0) ? (uint64_t)clk_limit_hz  
                                         : tg_effective_clk_max();  
    if (limit > TG_CLK_SYS_MAX_HZ) limit = TG_CLK_SYS_MAX_HZ;  
  
    TgPllConfig target_pll;  
  
    // ========================================================================  
    // АРХИТЕКТУРНЫЙ АРБИТРАЖ ЧАСТОТЫ ШИНЫ  
    // Честный сквозной автотюн при каждом старте (без эффекта памяти)  
    // ========================================================================  
    if (tg_clk_boosted) {  
        // Если активирован BOOST — матричный поиск ЛУЧШЕЙ частоты PLL  
        // строго под целевую частоту в пределах стабильного потолка  
        target_pll = tonegen_find_optimal_pll(base_hz, limit);  
        tg_pll_overclock = target_pll;  
    } else {  
        // Буст спит — штатный автотюн в пределах переданного лимита  
        target_pll = tonegen_find_optimal_pll(base_hz,  
                                              (clk_limit_hz != 0) ? limit  
                                                                  : TG_CLK_SYS_NOMINAL_HZ);  
    }  
  
    // Безопасное рантайм-напряжение ядра (VREG)  
    uint32_t selected_vsel = target_pll.vsel;  
    if (selected_vsel == 0 || selected_vsel < (uint32_t)VREG_VOLTAGE_1_10) {  
        selected_vsel = (target_pll.clk_sys_hz > TG_CLK_SYS_NOMINAL_HZ)  
                        ? (uint32_t)vsel_for(target_pll.clk_sys_hz)  
                        : (uint32_t)VREG_VOLTAGE_DEFAULT;  
    }  
  
    // Физически прошиваем делители в регистры PLL RP2040  
    // (обновит и физическую частоту, и current_clk_sys_hz!)  
    tg_set_clk_sys(target_pll, selected_vsel);  
  
    if (tg_clk_boosted) {  
        tg_pll_overclock = target_pll;  
#if BOOST_DEBUG  
        Serial.printf("[BOOST  ] ON   clk_sys=%7.3f MHz  refdiv=%lu  fbdiv=%3lu  p1=%lu  p2=%lu  VCO=%4lu MHz  VSEL=%4u mV  FLASH=%6.3f MHz (BAUDR=%u)\n",  
                      (double)current_clk_sys_hz / 1000000.0,  
                      (unsigned long)target_pll.refdiv,  
                      (unsigned long)target_pll.fbdiv,  
                      (unsigned long)target_pll.p1,  
                      (unsigned long)target_pll.p2,  
                      (unsigned long)(target_pll.fbdiv * (uint32_t)(tg_calibrated_xosc_hz() / 1000000ULL)),  
                      (unsigned)vsel_to_mv(selected_vsel),  
                      (double)current_clk_sys_hz / 1000000.0 / (double)ssi_hw->baudr,  
                      (unsigned)ssi_hw->baudr);  
#endif  
    }  
  
    // Конфигурация и запуск конечного автомата (State Machine) PIO  
    if (!tg_prog_loaded) {  
        tg_offset = pio_add_program(tg_pio, &tg_square_program);  
        tg_prog_loaded = true;  
    }  
    pio_sm_config c = pio_get_default_sm_config();  
    sm_config_set_wrap(&c, tg_offset + 0, tg_offset + 1);  
    sm_config_set_set_pins(&c, tg_pin, 1);  
  
    pio_gpio_init(tg_pio, tg_pin);  
    pio_sm_set_consecutive_pindirs(tg_pio, tg_sm, tg_pin, 1, false);  // пин в Z — несущая ВЫКЛ

    pio_sm_init(tg_pio, tg_sm, tg_offset, &c);  
    pio_sm_set_enabled(tg_pio, tg_sm, true);                          // SM крутится, выход отключён 
  
    // Привязка сетки к миллигерцам  
    tg_base_mhz = (uint64_t)base_hz * 1000ULL;  
    tg_step_mhz = (uint64_t)(step_hz * 1000.0);  
  
    // Синхронный потокобезопасный расчёт всей таблицы тонов  
    tg_rebuild_tone_table(tg_base_mhz, tg_step_mhz);  
    tonegen_set_tone(0);  
    tonegen_dither_reset();         // сброс накопителей фазы MASH-дизера  
  
#if TG_PLL_DEBUG  
    TgToneParams real_base_params = tg_tones[0];  
    Serial.printf("\n--- TG Runtime Diagnostics ---\n");  
    Serial.printf("Target Freq: %u Hz (Grid: %.2f Hz)\n", base_hz,  
                  (double)real_base_params.target_freq_chz / 100.0);  
    Serial.printf("clk_sys    : %u Hz (PLL: refdiv=%lu, fbdiv=%lu, p1=%lu, p2=%lu)\n",  
                  current_clk_sys_hz,  
                  (unsigned long)target_pll.refdiv,  
                  (unsigned long)target_pll.fbdiv,  
                  (unsigned long)target_pll.p1,  
                  (unsigned long)target_pll.p2);  
    Serial.printf("PLL CS     : 0x%08lX (refdiv=%lu)  FB=%lu  PRIM=0x%08lX\n",  
                  (unsigned long)pll_sys_hw->cs,  
                  (unsigned long)(pll_sys_hw->cs & PLL_CS_REFDIV_BITS),  
                  (unsigned long)pll_sys_hw->fbdiv_int,  
                  (unsigned long)pll_sys_hw->prim);  
    Serial.printf("PIO Regs   : INT=%u, FRAC=%u\n", real_base_params.pio_int, real_base_params.pio_frac);  
    Serial.printf("DDS Step   : 0x%08lX (%lu)  dist_to_k8=%u LSB\n",  
                (unsigned long)real_base_params.dds_step,  
                (unsigned long)real_base_params.dds_step,  
                (unsigned)dds_step_min_dist_to_k8(real_base_params.dds_step));  
    Serial.printf("-------------------------------\n");  
#endif  
  
    // Безопасный атомарный пуск высокоскоростного дизер-конвейера на Core 1  
    if (tg_engine == TONEGEN_ENGINE_DITHER) {  
        tone_changed = true;  
        multicore_launch_core1(tg_core1_entry);  
        tg_running = true;  
    } else {  
        // SIMPLE: одно ядро — пишем clkdiv напрямую  
        uint32_t save = spin_lock_blocking(tg_spin_lock);  
        uint32_t div_q8 = (tg_tones[0].pio_int << 8) | tg_tones[0].pio_frac;  
        spin_unlock(tg_spin_lock, save);  
        tg_pio->sm[tg_sm].clkdiv = div_q8 << 8;  
    }  
    return true;  
}  
  
  
// === ОБСЛУЖИВАНИЕ ТОНОВ ===  
  
void tonegen_set_narrowband(bool on) {  
    uint32_t save = spin_lock_blocking(tg_spin_lock);  
    tg_narrowband_mode = on ? 1u : 0u;  
    spin_unlock(tg_spin_lock, save);  
}  
  
/**  
 * @brief Мгновенная смена активного тона частотной сетки (FSK-переход).  
 *  
 * Перезагружает параметры PIO-делителя и DDS-шаг для движка дизеринга  
 * без разрыва фазы несущей: изменение применяется на следующей итерации  
 * модулятора, а не мгновенной записью в регистр `clkdiv`.  
 */  
void __not_in_flash_func(tonegen_set_tone)(uint8_t tone_index) {  
    if (tone_index >= TONEGEN_MAX_TONES) return;  
    if (tone_index == current_active_tone) return;  
  
    uint32_t save = spin_lock_blocking(tg_spin_lock);  
    target_pio_int   = tg_tones[tone_index].pio_int;  
    target_pio_frac8 = tg_tones[tone_index].pio_frac;  
    dds_step         = tg_tones[tone_index].dds_step;  
  
    // Маску выбираем здесь же, в атомарной сессии — чтобы Core1 прочитал  
    // консистентный снимок (int/frac/step/mask) в одном захвате лока.  
    // Глубина FRAC-дизера по величине делителя: ±N единиц FRAC-байта  
    // — это относительная ФМ-глубина ~mask/(256·int). На малых int  
    // (ВЧ-диапазоны) даже ±1 поднимает ближнюю юбку — выключаем.  
    if (tg_narrowband_mode) {  
        tg_frac_dither_mask = 0x00u;  // узкая полоса: дизер выключен — чистые линии  
    } else if (target_pio_int >= 38) {  
        tg_frac_dither_mask = 0x03u;  // ±3  (НЧ: int 38+)  
    } else if (target_pio_int >= 19) {  
        tg_frac_dither_mask = 0x01u;  // ±1  (СЧ: int 19–37)  
    } else {  
        tg_frac_dither_mask = 0x00u;  // off (ВЧ: int ≤18)  
    }  
  
    // Накопители НЕ сбрасываем: переход тонов фазово-непрерывный,  
    // спектрального всплеска на границах символов нет.  
    tone_changed    = true;        // ← поднимаем флаг последним  
    spin_unlock(tg_spin_lock, save);  
  
    current_active_tone = tone_index;  
  
    // SIMPLE-режим без Core 1: пишем делитель сразу  
    if (tg_engine == TONEGEN_ENGINE_SIMPLE) {  
        uint32_t div_q8 = (tg_tones[tone_index].pio_int << 8) |  
                          tg_tones[tone_index].pio_frac;  
        tg_pio->sm[tg_sm].clkdiv = div_q8 << 8;  
    }  
}  
  
/**  
 * @brief Полный сброс состояния MASH-дизера и накопителей фазы.  
 * Вызывается только при (пере)инициализации и после перестройки  
 * таблицы/clk_sys. НЕ вызывать при манипуляции тонов внутри передачи.  
 */  
void __not_in_flash_func(tonegen_dither_reset)(void) {  
    uint32_t save = spin_lock_blocking(tg_spin_lock);  
    dither_reset_pending = true;   // Core 1 обнулит локальные накопители  
    spin_unlock(tg_spin_lock, save);  
}  
  
/**  
 * @brief Ключ передатчика: коммутирует направление выходного пина PIO.  
 * true  — пин в режим выхода (несущая в эфире),  
 * false — пин в Z-состояние (вход).  
 */  
void __not_in_flash_func(tonegen_key)(bool on) {  
    pio_sm_set_consecutive_pindirs(tg_pio, tg_sm, tg_pin, 1, on);  
}  
  
/** @brief Блокирующее ожидание слота символа, мкс. */  
void tonegen_wait_us(uint32_t us) {  
    busy_wait_us(us);  
}  
  
/**  
 * @brief Фактическая частота тона после квантования делителей, Гц.  
 * f = clk_sys / (2 · (int + frac/256 + dds_effect)), где дизерный  
 * остаток в среднем добавляет dds_step/2^32 к дробной части.  
 */  
double tonegen_actual_hz(uint8_t tone_index) {  
    if (tone_index >= tg_ntones) return 0.0;  
    TgToneParams p = tg_tones[tone_index];  
    double div = (double)p.pio_int + (double)p.pio_frac / 256.0  
               + (double)p.dds_step / 4294967296.0 / 256.0;  
    return (double)current_clk_sys_hz / (2.0 * div);  
}  
  
/** @brief Освобождение PIO SM и ядра 1. */  
void tonegen_deinit(void) {  
    if (tg_running) {  
        multicore_reset_core1();  
        tg_running = false;  
    }  
    if (tg_prog_loaded) {  
        pio_sm_set_enabled(tg_pio, tg_sm, false);  
        pio_sm_set_consecutive_pindirs(tg_pio, tg_sm, tg_pin, 1, false);  
        pio_remove_program(tg_pio, &tg_square_program, tg_offset);  
        tg_prog_loaded = false;  
    }  
    if (tg_spin_lock) {  
        spin_lock_unclaim(spin_lock_get_num(tg_spin_lock));  
        tg_spin_lock = nullptr;  
    }  
    current_active_tone = TG_TONE_NONE;  
}
