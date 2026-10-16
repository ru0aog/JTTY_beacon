/**    
 * ============================================================================    
 *  jtty_codec.cpp — Реализация библиотеки кодирования JTTY (WSJT-X 3.3)  
 *  ver.1.10 от 2026-10-16, код портирован RU0AOG  
 * ============================================================================    
 *  См. jtty_codec.h — обзор протокола, API и атрибуция.    
 *    
 *  Структура файла:    
 *    1. Алфавит    
 *    2. Атом TEXT5 -> 32-битное тело (jtty_pack_atom)    
 *    3. Пакер сообщения (нормализация + нарезка по 5 символов)    
 *    4. FEC: CRC-12 + tail-biting свёртка + Gray-маппинг -> 59 тонов  
 *    
 *  Портировано из WSJT-X 3.3 (GPLv3, авторы: Joe Taylor K1JT,    
 *  Steve Franke K9AN и др.):    
 *    - jtty_source_codec.f90 — TEXT5-атом источника, ALPHABET    
 *    - tbcc.f90            — CRC-12(0x80F) + tail-biting свёртка K=10, r=1/2,    
 *                          полиномы 0o1167/0o1545, Gray-маппинг дибитов    
 * ============================================================================    
 */    
  
#include "jtty_codec.h"    
#include <string.h>    
#include <ctype.h>    
  
/* ============================================================================    
 * 1. Алфавит    
 * ============================================================================ */    
  
/* Sync-преамбула: 13 фиксированных индексов тонов в начале каждого кадра. */    
const uint8_t JTTY_IS13[JTTY_NSYNC] = {0,2,2,3,0,0,3,2,1,3,1,2,0};    
  
/* Алфавит источника (jtty_source_codec.f90), 64 символа.    
 * Индекс символа = 6 бит; TEXT5 упаковывает 5 символов = 30 бит.    
 * Пробел — индекс 36, '=' в алфавите НЕТ (важно для телеметрии!). */    
const char JTTY_ALPHABET[65] =    
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";    
  
static int source_index(char c) {    
    const char* p = strchr(JTTY_ALPHABET, c);    
    return p ? (int)(p - JTTY_ALPHABET) : -1;    
}    
  
/* ============================================================================    
 * 2. Атом TEXT5 -> 32-битное тело (jtty_source_codec.f90)    
 *    i2=3: n32 = top30(30 бит)<<2 | 3    
 * ============================================================================ */    
  
bool jtty_pack_atom(const JttyAtom* atom, bool eom, uint64_t* payload34)    
{    
    uint32_t n32 = 0;    
    switch (atom->kind) {    
    case JTTY_ATOM_TEXT5: {                /* i2=3: top30(30)+3 */    
        uint32_t top30 = 0;    
        for (int i = 0; i < 5; i++) {    
            char c = atom->text[i] ? atom->text[i] : ' ';    
            int ix = source_index(c);    
            if (ix < 0) return false;    
            top30 = 64u * top30 + (uint32_t)ix;    
        }    
        n32 = 4u * top30 + 3u;    
        break;    
    }    
    default:    
        return false;                      /* CALL/EXCH/CONTROL/GRID4 удалены */    
    }    
    /* frame(1:32)=n32 MSB-first, frame(33)='0' резерв, frame(34)=EOM */    
    *payload34 = ((uint64_t)n32 << 2) | (eom ? 1ULL : 0ULL);    
    return true;    
}    
  
bool jtty_pack_atoms(const JttyAtom* atoms, int natoms,    
                     uint64_t* frames, int* nframes, int is_final)    
{    
    bool final_ok = (is_final < 0) || (is_final != 0);    
    *nframes = 0;    
    if (natoms < 1 || natoms > JTTY_MAX_FRAMES) return false;    
    for (int i = 0; i < natoms; i++) {    
        bool eom = (i == natoms - 1) && final_ok;    
        if (!jtty_pack_atom(&atoms[i], eom, &frames[i])) return false;    
    }    
    *nframes = natoms;    
    return true;    
}    
  
/* ============================================================================    
 * 3. Пакер сообщения — только свободный текст    
 *    Нормализация (uppercase, '~'->' ', мусор->'#', схлопывание пробелов),    
 *    затем нарезка TEXT5-атомами по 5 символов. Позывные НЕ распознаются —    
 *    строка "CQ RU0AOG CQ TESTING BEACON" уходит как чистый текст;    
 *    приёмник WSJT-X декодирует TEXT5-атомы штатно.    
 * ============================================================================ */    
static void normalize_jtty(char* s) {    
    /* верхний регистр; '~'->' '; недопустимые символы -> '#';    
     * схлопывание пробелов и trim */    
    for (char* p = s; *p; p++) {    
        *p = (char)toupper((unsigned char)*p);    
        if (*p == '~') *p = ' ';    
        if (source_index(*p) < 0) *p = '#';    
    }    
    char* r = s; char* w = s; bool sp = true;    
    while (*r) {    
        if (*r == ' ') { if (!sp) *w++ = ' '; sp = true; }    
        else { *w++ = *r; sp = false; }    
        r++;    
    }    
    if (w > s && w[-1] == ' ') w--;    
    *w = 0;    
}    
  
bool jtty_pack_message(const char* umsg_in, JttyFrameList* fl,    
                       int exchange_profile, int is_final)    
{    
    (void)exchange_profile;    
    char msg[81];    
    strncpy(msg, umsg_in, 80); msg[80] = 0;    
    normalize_jtty(msg);    
    if (!msg[0]) return false;    
  
    /* Весь текст — TEXT5-атомами по 5 символов */    
    JttyAtom atoms[JTTY_MAX_FRAMES];    
    int na = 0;    
    int len = (int)strlen(msg);    
    for (int pos = 0; pos < len && na < JTTY_MAX_FRAMES; pos += 5) {    
        JttyAtom* a = &atoms[na];    
        memset(a, 0, sizeof(*a));    
        a->kind = JTTY_ATOM_TEXT5;    
        int k = 0;    
        for (int i = pos; i < pos + 5 && i < len; i++) a->text[k++] = msg[i];    
        while (k < 5) a->text[k++] = ' ';    
        a->text[5] = 0;    
        na++;    
    }    
    if (na == 0) return false;    
    if (len > na * 5) return false;   /* текст не влез в лимит кадров */    
  
    return jtty_pack_atoms(atoms, na, fl->frames, &fl->nframes, is_final);    
}    
  
/* ============================================================================    
 * 4. FEC: CRC-12(0x80F) + tail-biting свёртка K=10 (0o1167/0o1545)    
 *    + Gray-маппинг дибитов -> 59 индексов тонов (is13 + 46 данных)    
 * ============================================================================ */    
#define JTTY_NU        9          /* constraint length - 1 (K=10)          */    
#define JTTY_G0        01167      /* образующие полиномы (octal, tbcc.f90) */    
#define JTTY_G1        01545    
#define JTTY_CRC_POLY  0x80F      /* CRC-12 полином                        */    
  
static inline int parity32(uint32_t x) {    
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;    
    return (int)(x & 1);    
}    
  
/* CRC-12 поверх 34 бит payload (MSB-first), как tbcc_append_crc из tbcc.f90 */    
static uint16_t jtty_crc12(const uint8_t* info_bits /*34*/) {    
    uint16_t reg = 0;    
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++) {    
        uint16_t fb = (uint16_t)(((reg >> 11) ^ info_bits[i]) & 1);    
        reg = (uint16_t)(((reg << 1) & 0xFFF) ^ (fb ? JTTY_CRC_POLY : 0));    
    }    
    return reg;    
}    
  
void jtty_payload_to_tones(uint64_t payload34, uint8_t* itone /*59*/)    
{    
    /* info[0..33] = payload MSB-first; info[34..45] = CRC MSB-first */    
    uint8_t info[JTTY_INFO_BITS];    
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++)    
        info[i] = (uint8_t)((payload34 >> (33 - i)) & 1);    
    uint16_t crc = jtty_crc12(info);    
    for (int i = 0; i < 12; i++)    
        info[JTTY_PAYLOAD_BITS + i] = (uint8_t)((crc >> (11 - i)) & 1);    
  
    /* Tail-biting: стартовое состояние 9-битного регистра = последние    
     * NU бит информационного слова (tbcc.f90, tbcc_encode). */    
    uint16_t reg10 = 0;    
    for (int t = 0; t < JTTY_NU; t++)    
        reg10 = (uint16_t)((reg10 << 1) | info[JTTY_INFO_BITS - JTTY_NU + t]);    
  
    for (int i = 0; i < JTTY_NSYNC; i++) itone[i] = JTTY_IS13[i];    
  
    /* 46 дибитов данных; Gray-маппинг из tbcc_encode:    
     * (b0,b1): 00->0, 01->1, 11->2, 10->3  == GRAY4[2*b0+b1] */    
    static const uint8_t GRAY4[4] = {0, 1, 3, 2};    
    for (int i = 0; i < JTTY_INFO_BITS; i++) {    
        reg10 = (uint16_t)(((reg10 << 1) | info[i]) & 0x3FF);    
        uint8_t b0 = (uint8_t)parity32(reg10 & JTTY_G0);    
        uint8_t b1 = (uint8_t)parity32(reg10 & JTTY_G1);    
        itone[JTTY_NSYNC + i] = GRAY4[(b0 << 1) | b1];    
    }    
}    
  
int jtty_encode_message(const char* msg, uint8_t* tones)    
{    
    JttyFrameList fl;    
    if (!jtty_pack_message(msg, &fl, 0, -1))    
        return 0;    
    for (int f = 0; f < fl.nframes; f++)    
        jtty_payload_to_tones(fl.frames[f], tones + (size_t)f * JTTY_NSYMS);    
    return fl.nframes;    
}