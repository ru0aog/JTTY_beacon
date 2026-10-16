/**  
 * ============================================================================  
 *  jtty_codec.h — Автономная библиотека кодирования протокола JTTY (WSJT-X 3.3)  
 * ============================================================================  
 *  
 *  Назначение:  
 *    Полный TX-конвейер от текстового сообщения до массива индексов тонов,  
 *    готового к передаче любым MFSK-драйвером. Библиотека НЕ зависит от  
 *    железа, Arduino и глобального состояния — только <stdint.h>, <string.h>.  
 *  
 *  Портировано из WSJT-X 3.3 (GPLv3, авторы: Joe Taylor K1JT,  
 *  Steve Franke K9AN и др.):  
 *    - packjt77.f90        — pack28(), ihashcall(), токены CQ/QRZ/DE  
 *    - jtty_source_codec.f90 — атомы источника, ALPHABET, finish_struct()  
 *    - tbcc.f90            — CRC-12(0x80F) + tail-biting свёртка K=10, r=1/2,  
 *                          полиномы 0o1167/0o1545, Gray-маппинг дибитов  
 *    - nhash.c             — Jenkins hashlittle (токены CQ_aaaa/QRZ_aaaa)  
 *  
 *  Параметры протокола JTTY:  
 *    - 4-FSK, символ 32 мс (31.25 Бод), разнос тонов 31.25 Гц  
 *    - Кадр 1.888 с: 13 синхронизационных тонов + 46 тонов данных  
 *    - Полезная нагрузка кадра: 34 бита (32 бита атома + резерв + EOM)  
 *    - Сообщение: до 16 кадров, EOM ставится на последнем  
 *  
 *  Типичное использование (минимум):  
 *      uint8_t itone[59];  
 *      if (jtty_encode_message("CQ RU0AOG CQ", itone)) { ... один кадр ... }  
 *  
 *  Полный контроль:  
 *      JttyAtom a; ...; jtty_pack_atom(&a, true, &p34);  
 *      jtty_payload_to_tones(p34, itone);  
 * ============================================================================  
 */  
  
#ifndef JTTY_CODEC_H  
#define JTTY_CODEC_H  
  
#include <stdint.h>  
#include <stdbool.h>  
#include <stddef.h>  
  
#ifdef __cplusplus  
extern "C" {  
#endif  
  
/* ------------------------------------------------------------------  
 * Константы физического уровня и формата кадра  
 * ------------------------------------------------------------------ */  
#define JTTY_SYMBOL_US      32000UL   /* длительность одного символа, мкс   */  
#define JTTY_BAUD_HZ        31.25     /* скорость манипуляции = шаг тонов   */  
#define JTTY_NTONES         4         /* 4-FSK                              */  
#define JTTY_NSYNC          13        /* тонов в sync-преамбуле             */  
#define JTTY_NDATA          46        /* тонов данных (46 дибитов)          */  
#define JTTY_NSYMS          59        /* всего символов в кадре             */  
#define JTTY_MAX_FRAMES     16        /* максимум кадров в сообщении        */  
#define JTTY_PAYLOAD_BITS   34        /* 32 (атом) + 1 резерв + 1 EOM       */  
#define JTTY_INFO_BITS      46        /* 34 + 12 бит CRC                    */  
#define JTTY_CENTER_OFFSET  46.875    /* f0 = центр_полосы - 1.5*BAUD       */  
  
/* Синхронизационная преамбула (первые 13 тонов кадра) */  
extern const uint8_t JTTY_IS13[JTTY_NSYNC];  
  
/* Алфавит TEXT5-атома: 64 символа, индекс = 6 бит (5 символов = 30 бит) */  
extern const char JTTY_ALPHABET[65];  
  
/* ------------------------------------------------------------------  
 * Типы атомов источника (jtty_source_codec.f90)  
 * ------------------------------------------------------------------  
 * 32-битное тело атома кодируется схемой i2.n2:  
 *   i2=0/1 : CALL  — n28(28) + n2(2) + i2(2)  
 *   i2=2   : STRUCT30 — body(27) + family(3) + 2  
 *   i2=3   : TEXT5  — top30(30) + 3  
 */  
enum {  
    JTTY_ATOM_CALL           = 0,   /* позывной + действие (CQ/TU/AGN)      */  
    JTTY_ATOM_EXCH_NUM       = 1,   /* обмен: число (RST, серийник, зона)   */  
    JTTY_ATOM_EXCH_LOC       = 2,   /* обмен: локатор/секция                */  
    JTTY_ATOM_EXCH_PAIR      = 3,   /* обмен: пара значений                 */  
    JTTY_ATOM_EXCH_NUM_TIME  = 4,   /* обмен: число + время                 */  
    JTTY_ATOM_CONTROL        = 5,   /* служебные фразы (AGN?, TU, QRZ?...)  */  
    JTTY_ATOM_GRID4          = 6,   /* 4-символьный локатор                 */  
    JTTY_ATOM_TEXT5          = 7    /* свободный текст, 5 символов/атом     */  
};  
  
/* Подтипы CALL-атома */  
enum {  
    JTTY_CALL_CQ       = 0,   /* "CQ <call> CQ"   -> i2=0, n2=0 */  
    JTTY_CALL_CALL     = 1,   /* "<call>"         -> i2=0, n2=1 */  
    JTTY_CALL_TU_CQ    = 2,   /* "TU <call> CQ"   -> i2=0, n2=2 */  
    JTTY_CALL_CALL_TU  = 3,   /* "<call> TU"      -> i2=0, n2=3 */  
    JTTY_CALL_CALL_AGN = 4,   /* "<call> AGN?"    -> i2=1, n2=0 */  
    JTTY_CALL_TU_NOW   = 5    /* "TU NOW <call>"  -> i2=1, n2=1 */  
};  
  
/* Подтипы CONTROL-атома (CONTROL_TEXT из jtty_source_codec.f90) */  
enum {  
    JTTY_CONTROL_AGN = 0, JTTY_CONTROL_CALL = 1, JTTY_CONTROL_AGN_CALL = 2,  
    JTTY_CONTROL_NR = 3,  JTTY_CONTROL_AGN_NR = 4, JTTY_CONTROL_EXCH = 5,  
    JTTY_CONTROL_STATE = 6, JTTY_CONTROL_SECTION = 7, JTTY_CONTROL_ZONE = 8,  
    JTTY_CONTROL_GRID = 9,  JTTY_CONTROL_RPRT = 10, JTTY_CONTROL_QSL_TU = 11,  
    JTTY_CONTROL_TU = 12,   JTTY_CONTROL_QRZ = 13,  JTTY_CONTROL_QSO_B4 = 14,  
    JTTY_CONTROL_WAIT = 15, JTTY_CONTROL_NIL = 16,  JTTY_CONTROL_OK = 17  
};  
  
/* role: контекст поля обмена (0 — отдельное поле, 1 — полный обмен) */  
enum { JTTY_ROLE_FIELD_ONLY = 0, JTTY_ROLE_FULL = 1 };  
  
/* Профиль обмена (для будущего контест-пакера; маяку не нужен) */  
enum { JTTY_EXCHANGE_UNKNOWN = 0, JTTY_EXCHANGE_FIELD_DAY = 1,  
       JTTY_EXCHANGE_RTTY = 2 };  
  
/* ------------------------------------------------------------------  
 * Структуры данных  
 * ------------------------------------------------------------------ */  
  
/* Атом источника — аналог jtty_source_atom из Fortran.  
 * Заполняется либо напрямую, либо пакером jtty_pack_message(). */  
typedef struct {  
    int kind;            /* JTTY_ATOM_*                                   */  
    int subtype;         /* действие / подтип атома                       */  
    int role;            /* JTTY_ROLE_*                                   */  
    int value;           /* числовое поле (для EXCH-атомов)               */  
    int value2;          /* второе числовое поле (для EXCH_PAIR)          */  
    char text[14];       /* позывной (до 13) или текст (до 5 для TEXT5)   */  
} JttyAtom;  
  
/* Список кадров сообщения: массив 34-битных payload'ов */  
typedef struct {  
    uint64_t frames[JTTY_MAX_FRAMES];  /* payload34: n32<<2 | eom          */  
    int nframes;                       /* фактическое число кадров, 1..16  */  
} JttyFrameList;  
  
/* ------------------------------------------------------------------  
 * API  
 * ------------------------------------------------------------------ */  
  
/* --- pack28 / ihashcall (packjt77.f90) --- */  
  
/* Упаковка позывного/токена в 28 бит.  
 * Возврат: 0..0x0FFFFFFF при успехе, -1 при ошибке.  
 * Пространство значений:  
 *   0..2              спец-токены DE/QRZ/CQ  
 *   3..1002           CQ_nnn  
 *   1003..33770       CQ_aaaa/QRZ_aaaa (Jenkins 15-битный хэш)  
 *   2063592..6257895  22-битный хэш нестандартных позывных (<...>)  
 *   > 6257896         стандартные позывные (6-символьная сетка) */  
int32_t jtty_pack28(const char* call);  
  
/* Хэш позывного m бит (m = 10/12/22) — порт ihashcall(). */  
int jtty_ihashcall(const char* call13, int m);  
  
/* Признак «стандартного» позывного по критерию эталона:  
 * pack28 попал в диапазон стандартной сетки. */  
bool jtty_is_standard_call(const char* call);  
  
/* --- Атомарный уровень --- */  
  
/* Атом + флаг EOM -> 34-битный payload (n32<<2 | eom).  
 * eom=true ставится только на последнем кадре сообщения. */  
bool jtty_pack_atom(const JttyAtom* atom, bool eom, uint64_t* payload34);  
  
/* Массив атомов -> массив payload'ов. is_final: 1/0 или -1 (final). */  
bool jtty_pack_atoms(const JttyAtom* atoms, int natoms,  
                     uint64_t* frames, int* nframes, int is_final);  
  
/* --- Уровень сообщения (маячный пакер) ---  
 * Поддерживаемые формы:  
 *   "CQ <call> [CQ]"   -> CALL(subtype CQ) + хвост TEXT5  
 *   "<call>"           -> CALL(subtype CALL)  
 *   "<call> TU"        -> CALL(CALL_TU); "TU <call> CQ" -> CALL(TU_CQ)  
 *   любой иной текст   -> цепочка TEXT5-атомов по 5 символов  
 * Возврат: true и fl->nframes кадров в fl->frames. */  
bool jtty_pack_message(const char* msg, JttyFrameList* fl,  
                       int exchange_profile, int is_final);  
  
/* --- FEC + формирование кадра --- */  
  
/* 34-битный payload -> 59 индексов тонов (0..3):  
 *   CRC-12(0x80F) поверх MSB-first payload -> tail-biting свёртка  
 *   K=10 (0o1167/0o1545) -> Gray-маппинг дибитов -> is13 + 46 тонов. */  
void jtty_payload_to_tones(uint64_t payload34, uint8_t* itone59);  
  
/* Удобная обёртка: сообщение -> массивы тонов для всех кадров.  
 * tones — буфер 59*JTTY_MAX_FRAMES байт; возвращает число кадров или 0. */  
int jtty_encode_message(const char* msg, uint8_t* tones);  
  
#ifdef __cplusplus  
}  
#endif  
  
#endif /* JTTY_CODEC_H */