/**  
 * ============================================================================  
 *  mfsk_tonegen.cpp — программный генератор тонов радиочастоты на ядре RP2040
 *  ver.1.00 от 2026-10-16, автор RU0AOG
 * ============================================================================  
 *  Краткое описание:  
 *   - 2-тактовая PIO-программа меандра (pio_square_instructions);  
 *   - таблица тонов {pio_int, pio_frac, dds_step} в .time_critical-секции;  
 *   - дизер-движок Core 1: xorshift32 + DDS-накопитель остатка + маска глубины;  
 *   - атомарная смена тона через аппаратный spin_lock;  
 *   - фазово-непрерывный переход: накопители не сбрасываются.  
 * ============================================================================  
 */  
#include "mfsk_tonegen.h"  
  
#include "hardware/pio.h"  
#include "hardware/clocks.h"  
#include "hardware/sync.h"  
#include "hardware/timer.h"  
#include "pico/multicore.h"  
  
/* ---------------------------------------------------------------------------  
 * 1. PIO-программа меандра — копия pio_square_program из vfo_hardware.cpp.  
 *    0xe001 = set pins,1 ; 0xe000 = set pins,0 ; wrap 0..1 → период 2 такта.  
 *    f_out = clk_sys / (2 * clkdiv), clkdiv в формате 24.8.  
 * ------------------------------------------------------------------------- */  
static const uint16_t tg_square_instructions[] = { 0xe001, 0xe000 };  
static const pio_program_t tg_square_program = {  
    .instructions = tg_square_instructions,  
    .length       = 2,  
    .origin       = -1,  
    .pio_version  = 0  
};  
  
/* ---------------------------------------------------------------------------  
 * 2. Параметры одного тона — аналог VfoParameters.  
 *    pio_int  : целая часть делителя PIO (16 бит поля clkdiv).  
 *    pio_frac : дробная часть делителя (8 бит поля clkdiv).  
 *    dds_step : шаг 32-битного DDS-накопителя остатка — добирает  
 *               под-LSB ошибку frac-байта (разрешение ~мГц).  
 * ------------------------------------------------------------------------- */  
typedef struct {  
    uint32_t pio_int;  
    uint32_t pio_frac;  
    uint32_t dds_step;  
    double   freq_hz;      /* для tonegen_actual_hz() */  
} tg_params_t;  
  
static tg_params_t tg_tones[TONEGEN_MAX_TONES]  
    __attribute__((section(".time_critical.tg_tones")));  
  
/* --- Состояние генератора --- */  
static uint8_t  tg_pin        = 255;  
static uint8_t  tg_ntones     = 0;  
static PIO      tg_pio        = pio0;  
static uint     tg_sm         = 0;  
static uint     tg_offset     = 0;  
static bool     tg_prog_loaded = false;  
static uint32_t tg_clk_sys    = 0;  
static tonegen_engine_t tg_engine = TONEGEN_ENGINE_DITHER;  
static bool     tg_key_down   = false;  
static bool     tg_running    = false;  
#define TG_TONE_NONE 255  
static volatile uint8_t  tg_active_tone = TG_TONE_NONE;  
  
/* --- Межъядерные переменные обмена (spin_lock, как в vfo_hardware) --- */  
static spin_lock_t *tg_lock = nullptr;  
static volatile bool     tg_tone_changed = false;  
static volatile uint32_t tg_target_int   = 8;  
static volatile uint32_t tg_target_frac  = 0;  
static volatile uint32_t tg_dds_step     = 0;  
static volatile uint32_t tg_dds_accum    = 0;   /* накопитель остатка      */  
static volatile uint32_t tg_xorshift     = 0xACE1u;  
static volatile uint32_t tg_dither_mask  = 0x03; /* глубина FRAC-дизера    */  
static volatile uint8_t  tg_narrowband   = 0;  
static volatile bool     tg_core1_quit   = false;  
  
/* ---------------------------------------------------------------------------  
 * 3. Расчёт параметров тона — упрощённый calculate_raw_params_mhz().  
 *  
 *    div_24_8 = clk_sys * 256 / (2 * f)  
 *    pio_int  = div_24_8 >> 8            (насыщение: 2 .. 0xFFFF)  
 *    pio_frac = div_24_8 & 0xFF  
 *    dds_step = остаток ниже 8 бит, масштабированный в 2^32:  
 *               err = clk_sys/(2f) − (int + frac/256)  
 *               step = err * 2^32 / (1/256)  →  округлённый остаток  
 * ------------------------------------------------------------------------- */  
static void tg_calc_tone(uint64_t clk_hz, double f_hz, tg_params_t *out)  
{  
    /* Нужна частота > 0 и делитель >= 2 (минимальный clkdiv PIO). */  
    if (f_hz < 1.0) f_hz = 1.0;  
  
    /* Делитель в формате Q24.8, с избытком разрядности для остатка. */  
    double div_exact = (double)clk_hz / (2.0 * f_hz);  
    if (div_exact < 2.0)     div_exact = 2.0;  
    if (div_exact > 65535.996) div_exact = 65535.996;  
  
    uint32_t div_q8   = (uint32_t)(div_exact * 256.0 + 0.5);  
    if (div_q8 < (2u << 8)) div_q8 = 2u << 8;  
    if (div_q8 > (0xFFFFu << 8) + 0xFFu) div_q8 = (0xFFFFu << 8) + 0xFFu;  
  
    out->pio_int  = div_q8 >> 8;  
    out->pio_frac = div_q8 & 0xFFu;  
  
    /* Остаточная ошибка ниже одного LSB frac-байта → в шаг DDS. */  
    double rem = div_exact - (double)div_q8 / 256.0;   /* |rem| <= 1/512 */  
    long long step = (long long)(rem * 256.0 * 4294967296.0 + (rem >= 0 ? 0.5 : -0.5));  
    out->dds_step = (uint32_t)step;  
  
    /* Фактическая частота (без учёта дизера): clk/(2*div_q8/256). */  
    out->freq_hz = (double)clk_hz * 128.0 / (double)div_q8;  
}  
  
/* ---------------------------------------------------------------------------  
 * 4. Дизер-движок Core 1 — упрощённый аналог vfo_core1_entry.  
 *    Каждую итерацию:  
 *      acc += dds_step;  перенос acc (overflow) → +1 к frac;  
 *      jitter = (xorshift32() & mask) − bias  → ±N единиц frac;  
 *      clkdiv = (int << 16) | ((frac + carry + jitter) & 0xFF) << 8;  
 *    Пишет clkdiv каждый цикл — десятки МГц обновлений → спур размазан.  
 *    Фаза несущей непрерывна: регистр clkdiv PIO применяет новое значение  
 *    на границе текущего полупериода, накопители не сбрасываются.  
 * ------------------------------------------------------------------------- */  
static void __not_in_flash_func(tg_core1_entry)(void)  
{  
    uint32_t acc = 0, xs = tg_xorshift;  
    while (!tg_core1_quit) {  
        if (tg_tone_changed) {  
            tg_tone_changed = false;  
            /* Консистентный снимок уже лежит в tg_target_* (записан под локом). */  
        }  
        acc += tg_dds_step;  
        uint32_t carry = (acc < tg_dds_step) ? 1u : 0u;  
  
        xs ^= xs << 13; xs ^= xs >> 17; xs ^= xs << 5;   /* xorshift32 */  
        uint32_t mask  = tg_narrowband ? 0u : tg_dither_mask;  
        int32_t  jit   = (int32_t)(xs & mask) - (int32_t)(mask >> 1);  
  
        int32_t frac = (int32_t)tg_target_frac + (int32_t)carry + jit;  
        if (frac < 0) frac = 0;  
        if (frac > 0xFF) frac = 0xFF;  
  
        tg_pio->sm[tg_sm].clkdiv = (tg_target_int << 16) | ((uint32_t)frac << 8);  
    }  
    tg_xorshift = xs;   /* сохранить состояние ГПСЧ */  
}  
  
/* ---------------------------------------------------------------------------  
 * 5. Публичный API  
 * ------------------------------------------------------------------------- */  
bool tonegen_init(uint8_t pin, uint32_t base_hz, double step_hz,  
                  uint8_t ntones, tonegen_engine_t engine,  
                  uint32_t clk_sys_hz)  
{  
    if (ntones == 0 || ntones > TONEGEN_MAX_TONES) return false;  
    if (clk_sys_hz == 0) clk_sys_hz = clock_get_hz(clk_sys);  
  
    tg_pin = pin; tg_ntones = ntones; tg_engine = engine;  
    tg_clk_sys = clk_sys_hz;  
  
    if (tg_lock == nullptr) {  
        int id = spin_lock_claim_unused(true);  
        tg_lock = spin_lock_init(id);  
    }  
  
    /* SM глушим до готовности таблицы — иначе при съезде делителя  
       на выходе был бы меандр вне полосы (как в vfo_hardware_init). */  
    if (tg_prog_loaded) {  
        pio_sm_set_enabled(tg_pio, tg_sm, false);  
    } else {  
        tg_offset = pio_add_program(tg_pio, &tg_square_program);  
        tg_prog_loaded = true;  
    }  
  
    pio_sm_config c = pio_get_default_sm_config();  
    sm_config_set_wrap(&c, tg_offset + 0, tg_offset + 1);  
    sm_config_set_set_pins(&c, tg_pin, 1);  
    pio_gpio_init(tg_pio, tg_pin);  
    pio_sm_set_consecutive_pindirs(tg_pio, tg_sm, tg_pin, 1, true);  
    pio_sm_init(tg_pio, tg_sm, tg_offset, &c);  
  
    /* Таблица тонов под текущий clk_sys. */  
    for (uint8_t i = 0; i < tg_ntones; i++)  
        tg_calc_tone(tg_clk_sys, base_hz + (double)i * step_hz, &tg_tones[i]);  
    tg_active_tone = TG_TONE_NONE;  
    tg_xorshift = 0xACE1u ^ time_us_32();  
    tg_dds_accum = 0;  
    tg_tone_changed = false;  
  
    /* Запуск движка дизера на втором ядре. */  
    if (engine == TONEGEN_ENGINE_DITHER && !tg_running) {  
        tg_core1_quit = false;  
        multicore_launch_core1(tg_core1_entry);  
        tg_running = true;  
    }  
  
    tonegen_set_tone(0);  
    pio_sm_set_enabled(tg_pio, tg_sm, true);  
    tg_key_down = false;  
    return true;  
}  
  
void __not_in_flash_func(tonegen_set_tone)(uint8_t tone_index)  
{  
    if (tone_index >= tg_ntones) return;  
    if (tone_index == tg_active_tone) return;  
  
    uint32_t save = spin_lock_blocking(tg_lock);  
    tg_target_int  = tg_tones[tone_index].pio_int;  
    tg_target_frac = tg_tones[tone_index].pio_frac;  
    tg_dds_step    = tg_tones[tone_index].dds_step;  
  
    /* Глубина FRAC-дизера по величине делителя (из vfo_set_tone_instant):  
       на ВЧ (малый int) даже ±1 к frac поднимает ближнюю юбку. */  
    if (tg_narrowband)            tg_dither_mask = 0x00;  
    else if (tg_target_int >= 38) tg_dither_mask = 0x03;  
    else if (tg_target_int >= 19) tg_dither_mask = 0x01;  
    else                          tg_dither_mask = 0x00;  
  
    if (tg_engine == TONEGEN_ENGINE_SIMPLE) {  
        /* Без второго ядра: прямая запись clkdiv (без дизера). */  
        tg_pio->sm[tg_sm].clkdiv =  
            (tg_target_int << 16) | (tg_target_frac << 8);  
    } else {  
        tg_tone_changed = true;   /* флаг последним — атомарный снимок */  
    }  
    spin_unlock(tg_lock, save);  
  
    tg_active_tone = tone_index;  
}  
  
void tonegen_key(bool on)  
{  
    tg_key_down = on;  
    /* Ключ = направление пина: вход = несущая снята (высокий импеданс),  
       выход = излучает меандр. Та же механика, что в vfo_operation_set. */  
    pio_sm_set_consecutive_pindirs(tg_pio, tg_sm, tg_pin, 1, on);  
}  
  
void tonegen_set_narrowband(bool on)  
{  
    uint32_t save = spin_lock_blocking(tg_lock);  
    tg_narrowband = on ? 1u : 0u;  
    spin_unlock(tg_lock, save);  
}  
  
void tonegen_wait_us(uint32_t us)  
{  
    busy_wait_us(us);   /* точный таймерный слот; при желании — со сторожком */  
}  
  
void tonegen_deinit(void)  
{  
    tonegen_key(false);  
    pio_sm_set_enabled(tg_pio, tg_sm, false);  
    if (tg_running) {  
        tg_core1_quit = true;  
        tg_running = false;  
    }  
}  
  
double tonegen_actual_hz(uint8_t tone_index)  
{  
    if (tone_index >= tg_ntones) return 0.0;  
    return tg_tones[tone_index].freq_hz;  
}