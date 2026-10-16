/**  
 * ============================================================================  
 *  jtty_codec.cpp — Реализация библиотеки кодирования JTTY (WSJT-X 3.3)  
 * ============================================================================  
 *  См. jtty_codec.h — обзор протокола, API и атрибуция.  
 *  
 *  Структура файла:  
 *    1. Алфавит и таблицы  
 *    2. nhash (Jenkins hashlittle) — токены CQ_aaaa/QRZ_aaaa  
 *    3. ihashcall — 22/12/10-битные хэши позывных  
 *    4. pack28 — упаковка позывного в 28 бит  
 *    5. Атомы -> 32-битное тело (jtty_pack_atom)  
 *    6. Пакер сообщения (маячный набор правил)  
 *    7. FEC: CRC-12 + tail-biting свёртка + Gray-маппинг -> 59 тонов  
 * ============================================================================  
 */  
  
#include "jtty_codec.h"  
#include <string.h>  
#include <ctype.h>  
  
/* ============================================================================  
 * 1. Алфавит и таблицы  
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
 * 2. nhash — Jenkins hashlittle (эталон nhash.c из WSJT-X)  
 *    Используется только для токенов CQ_aaaa/QRZ_aaaa: initval=146,  
 *    результат маскируется & 32767 (15 бит).  
 * ============================================================================ */  
static inline uint32_t rot32(uint32_t x, int k) {  
    return (x << k) | (x >> (32 - k));  
}  
  
/* Один раунд перемешивания hashlittle (макросы mix/final из nhash.c  
 * заменены inline-функциями — побочных эффектов макросов здесь нет). */  
static inline void nh_mix(uint32_t* pa, uint32_t* pb, uint32_t* pc) {  
    uint32_t a = *pa, b = *pb, c = *pc;  
    a-=c; a^=rot32(c,4);  c+=b;  
    b-=a; b^=rot32(a,6);  a+=c;  
    c-=b; c^=rot32(b,8);  b+=a;  
    a-=c; a^=rot32(c,16); c+=b;  
    b-=a; b^=rot32(a,19); a+=c;  
    c-=b; c^=rot32(b,4);  b+=a;  
    *pa = a; *pb = b; *pc = c;  
}  
  
static inline void nh_final(uint32_t* pa, uint32_t* pb, uint32_t* pc) {  
    uint32_t a = *pa, b = *pb, c = *pc;  
    c^=b; c-=rot32(b,14);  
    a^=c; a-=rot32(c,11);  
    b^=a; b-=rot32(a,25);  
    c^=b; c-=rot32(b,16);  
    a^=c; a-=rot32(c,4);  
    b^=a; b-=rot32(a,14);  
    c^=b; c-=rot32(b,24);  
    *pa = a; *pb = b; *pc = c;  
}  
  
static int32_t nhash_bytes(const void* key, size_t length, int32_t initval) {  
    const uint8_t* k = (const uint8_t*)key;  
    uint32_t a = 0xdeadbeefu + (uint32_t)length + (uint32_t)initval;  
    uint32_t b = a, c = a;  
    /* Основной цикл: по 12 байт за раунд, little-endian сборка слов */  
    while (length > 12) {  
        a += k[0] | ((uint32_t)k[1]<<8) | ((uint32_t)k[2]<<16) | ((uint32_t)k[3]<<24);  
        b += k[4] | ((uint32_t)k[5]<<8) | ((uint32_t)k[6]<<16) | ((uint32_t)k[7]<<24);  
        c += k[8] | ((uint32_t)k[9]<<8) | ((uint32_t)k[10]<<16)| ((uint32_t)k[11]<<24);  
        nh_mix(&a,&b,&c);  
        k += 12; length -= 12;  
    }  
    /* Хвост 0..12 байт: fall-through по эталону */  
    switch (length) {  
        case 12: c += (uint32_t)k[11]<<24; /* fall through */  
        case 11: c += (uint32_t)k[10]<<16; /* fall through */  
        case 10: c += (uint32_t)k[9]<<8;   /* fall through */  
        case 9:  c += k[8];                /* fall through */  
        case 8:  b += (uint32_t)k[7]<<24;  /* fall through */  
        case 7:  b += (uint32_t)k[6]<<16;  /* fall through */  
        case 6:  b += (uint32_t)k[5]<<8;   /* fall through */  
        case 5:  b += k[4];                /* fall through */  
        case 4:  a += (uint32_t)k[3]<<24;  /* fall through */  
        case 3:  a += (uint32_t)k[2]<<16;  /* fall through */  
        case 2:  a += (uint32_t)k[1]<<8;   /* fall through */  
        case 1:  a += k[0]; break;  
        case 0:  return (int32_t)(c & 32767);  
    }  
    nh_final(&a,&b,&c);  
    return (int32_t)(c & 32767);  
}  
  
/* hash() из packjt77.f90: nhash(s, len, 146) & 32767 */  
static int jtty_hash15(const char* s, size_t len) {  
    return (int)nhash_bytes(s, len, 146);  
}  
  
/* ============================================================================  
 * 3. ihashcall — хэш нестандартного позывного (packjt77.f90)  
 *    n8 = свёртка по основанию 38 первых 11 символов (алфавит C38),  
 *    хэш = старшие m бит произведения 47055833459 * n8 (low 64 бита).  
 * ============================================================================ */  
int jtty_ihashcall(const char* call13, int m) {  
    static const char C38[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";  
    uint64_t n8 = 0;  
    char buf[12];  
    memset(buf, ' ', 11); buf[11] = 0;  
    size_t l = strlen(call13); if (l > 11) l = 11;  
    memcpy(buf, call13, l);  
    for (int i = 0; i < 11; i++) {  
        const char* p = strchr(C38, buf[i]);  
        n8 = 38 * n8 + (uint64_t)(p ? (p - C38) : 0);  
    }  
    uint64_t prod = 47055833459ULL * n8;   /* переполнение = low 64 бита */  
    if (m <= 0) return 0;  
    return (int)(prod >> (64 - m));  
}  
  
/* ============================================================================  
 * 4. pack28 — порт pack28() из packjt77.f90  
 * ============================================================================ */  
#define PACK28_NTOKENS  2063592L  
#define PACK28_MAX22    4194304L  
  
/* Алфавиты позиций 6-символьной сетки стандартного позывного */  
static const char P28_A1[38] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";  
static const char P28_A2[37] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";  
static const char P28_A3[11] = "0123456789";  
static const char P28_A4[28] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";  
  
static int  idx_in(const char* tab, char c) {  
    const char* p = strchr(tab, c);  
    return p ? (int)(p - tab) : -1;  
}  
static bool is_dig(char c) { return c >= '0' && c <= '9'; }  
static bool is_let(char c) { return c >= 'A' && c <= 'Z'; }  
  
int32_t jtty_pack28(const char* call)  
{  
    /* c13: до 13 символов, дополнена пробелами справа (Fortran adjustl) */  
    char c13[14];  
    memset(c13, ' ', 13); c13[13] = 0;  
    size_t L = strlen(call); if (L > 13) L = 13;  
    memcpy(c13, call, L);  
  
    /* Workaround'ы префиксов — применяются к c13 ДО токенов и iarea:  
     * 3DA0xxx -> 3D0xxx (Свазиленд), 3Xyyy -> Qyyy (Гвинея) */  
    if (!strncmp(c13, "3DA0", 4)) {  
        c13[2] = '0';  
        memmove(c13 + 3, c13 + 4, 9);  
    }  
    if (c13[0] == '3' && c13[1] == 'X' && is_let(c13[2]))  
        c13[0] = 'Q';  
  
    int n = 13;  
    while (n > 0 && c13[n - 1] == ' ') n--;   /* len_trim(c13) */  
  
    /* Спец-токены: DE=0, QRZ=1, CQ=2 (порядок эталона!) */  
    if (n >= 2 && !strncmp(c13, "DE", 2)  && (n == 2 || c13[2] == ' ')) return 0;  
    if (n >= 3 && !strncmp(c13, "QRZ", 3) && (n == 3 || c13[3] == ' ')) return 1;  
    if (n >= 2 && !strncmp(c13, "CQ", 2)  && (n == 2 || c13[2] == ' ')) return 2;  
  
    /* CQ_nnn: "CQ" + пробел + 1..3 цифры -> n28 = 3 + nnn */  
    if (n >= 4 && !strncmp(c13, "CQ ", 3)) {  
        bool digits = true; int nd = 0, val = 0;  
        for (int i = 3; i < n; i++) {  
            if (!is_dig(c13[i])) { digits = false; break; }  
            val = val * 10 + (c13[i] - '0'); nd++;  
        }  
        if (digits && nd >= 1 && nd <= 3)  
            return (3 + val) & 0x0FFFFFFFL;  
        /* CQ_aaaa: 4 буквы -> n28 = 3 + 1000 + hash15 */  
        if (n == 8) {  
            bool lets = true;  
            for (int i = 4; i < 8; i++)  
                if (!is_let(c13[i])) { lets = false; break; }  
            if (lets)  
                return (3 + 1000 + jtty_hash15(c13 + 4, 4)) & 0x0FFFFFFFL;  
        }  
    }  
    /* QRZ_aaaa — тот же диапазон буквенных токенов */  
    if (n == 8 && !strncmp(c13, "QRZ ", 4)) {  
        bool lets = true;  
        for (int i = 4; i < 8; i++)  
            if (!is_let(c13[i])) { lets = false; break; }  
        if (lets)  
            return (3 + 1000 + jtty_hash15(c13 + 4, 4)) & 0x0FFFFFFFL;  
    }  
  
    /* Стандартный позывной: iarea = позиция первой цифры при обходе  
     * справа (Fortran 1-based; здесь 0-based + 1 -> iarea1 ∈ {2,3}) */  
    int iarea1 = 0;  
    for (int i = n - 1; i >= 0; i--) {  
        if (is_dig(c13[i])) { iarea1 = i + 1; break; }  
    }  
    if ((iarea1 != 2 && iarea1 != 3) || n > 6) {  
        /* Нестандартный -> 22-битный хэш (<...>-позывной) */  
        int32_t n22 = jtty_ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    /* 6-символьная сетка: iarea=2 -> ' '+c13(1:5); iarea=3 -> c13(1:6) */  
    char cs[7];  
    memset(cs, ' ', 6); cs[6] = 0;  
    if (iarea1 == 2) { cs[0] = ' '; memcpy(cs + 1, c13, 5); }  
    else             { memcpy(cs, c13, 6); }  
  
    int i1 = idx_in(P28_A1, cs[0]);   /* ' '+цифра+буква   (37) */  
    int i2 = idx_in(P28_A2, cs[1]);   /* цифра+буква       (36) */  
    int i3 = idx_in(P28_A3, cs[2]);   /* цифра             (10) */  
    int i4 = idx_in(P28_A4, cs[3]);   /* ' '+буква         (27) */  
    int i5 = idx_in(P28_A4, cs[4]);  
    int i6 = idx_in(P28_A4, cs[5]);  
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0 || i6 < 0) {  
        int32_t n22 = jtty_ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    int64_t n28 = 36LL*10*27*27*27*i1 + 10LL*27*27*27*i2 + 27LL*27*27*i3  
                + 27LL*27*i4 + 27LL*i5 + i6;  
    n28 += (int64_t)PACK28_NTOKENS + PACK28_MAX22;  
    return (int32_t)(n28 & 0x0FFFFFFFL);  
}  
  
bool jtty_is_standard_call(const char* call) {  
    return jtty_pack28(call) > (int32_t)(PACK28_NTOKENS + PACK28_MAX22);  
}  
  
/* ============================================================================  
 * 5. Атом -> 32-битное тело (pack_jtty_atom из jtty_source_codec.f90)  
 * ============================================================================ */  
  
/* finish_struct: n32 = (body<<5) | (family<<2) | 2 — семейства STRUCT30 */  
static inline uint32_t finish_struct(uint32_t body, int family) {  
    return 4u * (8u * body + (uint32_t)family) + 2u;  
}  
  
/* GRID4 (AA00..RR99) -> индекс ((a*18+b)*10+c)*10+d */  
static bool grid4_to_index(const char* g, int* idx) {  
    if (!g[0] || !g[1] || !g[2] || !g[3]) return false;  
    if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R') return false;  
    if (!isdigit((unsigned char)g[2]) || !isdigit((unsigned char)g[3])) return false;  
    *idx = ((g[0]-'A')*18 + (g[1]-'A'))*100 + (g[2]-'0')*10 + (g[3]-'0');  
    return true;  
}  
  
bool jtty_pack_atom(const JttyAtom* atom, bool eom, uint64_t* payload34)  
{  
    uint32_t n32 = 0;  
    switch (atom->kind) {  
    case JTTY_ATOM_CALL: {                 /* i2=0/1: n28(28)+n2(2)+i2(2) */  
        if (atom->subtype < 0 || atom->subtype > 5) return false;  
        if (!jtty_is_standard_call(atom->text)) return false;  
        int32_t n28 = jtty_pack28(atom->text);  
        if (n28 < 0) return false;  
        int i2, n2;  
        if (atom->subtype <= 3) { i2 = 0; n2 = atom->subtype; }  
        else                    { i2 = 1; n2 = atom->subtype - 4; }  
        n32 = ((uint32_t)n28 << 4) | ((uint32_t)n2 << 2) | (uint32_t)i2;  
        break;  
    }  
    case JTTY_ATOM_EXCH_NUM: {             /* family 0 */  
        if (atom->role < 0 || atom->role > 1) return false;  
        if (atom->subtype < 0 || atom->subtype > 15) return false;  
        if (atom->value < 0 || atom->value > 131071) return false;  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)atom->value << 5);  
        n32 = finish_struct(body, 0);  
        break;  
    }  
    case JTTY_ATOM_EXCH_LOC: {             /* family 1 */  
        if (atom->role < 0 || atom->role > 1) return false;  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x1FFFF) << 5);  
        n32 = finish_struct(body, 1);  
        break;  
    }  
    case JTTY_ATOM_EXCH_PAIR: {            /* family 2 */  
        uint32_t body = ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x3FF) << 12)  
                      | ((uint32_t)(atom->value2 & 0x3FF) << 2);  
        n32 = finish_struct(body, 2);   /* раскладку pair_data сверить с эталоном */  
        break;  
    }  
    case JTTY_ATOM_EXCH_NUM_TIME: {        /* family 3 */  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x1FFFF) << 5);  
        n32 = finish_struct(body, 3);  
        break;  
    }  
    case JTTY_ATOM_CONTROL: {              /* family 4, misc=CONTROL(0) */  
        if (atom->subtype < 0 || atom->subtype > 17) return false;  
        uint32_t body = (0u << 23) | ((uint32_t)atom->subtype << 16);  
        n32 = finish_struct(body, 4);  
        break;  
    }  
    case JTTY_ATOM_GRID4: {                /* family 4, misc=GRID4(1) */  
        if (atom->role < 0 || atom->role > 1) return false;  
        int gi;  
        if (!grid4_to_index(atom->text, &gi)) return false;  
        uint32_t data = ((uint32_t)atom->role << 22) | ((uint32_t)gi << 7);  
        uint32_t body = (1u << 23) | data;  
        n32 = finish_struct(body, 4);  
        break;  
    }  
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
        return false;  
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
 * 6. Пакер сообщения — маячный набор правил  
 *    Полный DP-паковщик pack_jtty (contest-грамматика) — отдельный слой;  
 *    здесь: CALL-атомы для "CQ <call> [CQ]", "<call>", "<call> TU",  
 *    "TU <call> CQ"; весь остальной текст — TEXT5-цепочка по 5 символов.  
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
  
/* Эвристика «похоже на позывной» для выбора CALL-атома в разборе;  
 * финальную валидацию делает сам pack28. */  
static bool is_callsign_word(const char* s) {  
    size_t n = strlen(s);  
    if (n < 3 || n > 6) return false;  
    if (strchr(s, '/') || strchr(s, '<')) return false;  
    int nd = 0;  
    for (size_t i = 0; s[i]; i++) if (isdigit((unsigned char)s[i])) nd++;  
    return nd >= 1;  
}  
  
bool jtty_pack_message(const char* umsg_in, JttyFrameList* fl,  
                       int exchange_profile, int is_final)  
{  
    (void)exchange_profile;  
    char msg[81];  
    strncpy(msg, umsg_in, 80); msg[80] = 0;  
    normalize_jtty(msg);  
    if (!msg[0]) return false;  
  
    JttyAtom atoms[JTTY_MAX_FRAMES];  
    int na = 0;  
  
    /* Разбор по словам */  
    char buf[81];  
    strcpy(buf, msg);  
    char* w[8]; int nw = 0;  
    for (char* t = strtok(buf, " "); t && nw < 8; t = strtok(NULL, " "))  
        w[nw++] = t;  
  
    if (nw >= 2 && strcmp(w[0], "CQ") == 0 && is_callsign_word(w[1])) {  
        /* "CQ <call> [CQ]" -> CALL-атом subtype CQ; хвост уйдёт в TEXT5 */  
        memset(&atoms[na], 0, sizeof(JttyAtom));  
        atoms[na].kind = JTTY_ATOM_CALL;  
        atoms[na].subtype = JTTY_CALL_CQ;  
        atoms[na].role = JTTY_ROLE_FIELD_ONLY;  
        strncpy(atoms[na].text, w[1], 13);  
        na++;  
        int consumed = 3 + (int)strlen(w[1]);       /* "CQ " + call */  
        char* rest = msg + consumed;  
        while (*rest == ' ') rest++;  
        if (strncmp(rest, "CQ", 2) == 0 && (rest[2] == 0 || rest[2] == ' ')) {  
            rest += 2;  
            while (*rest == ' ') rest++;  
        }  
        memmove(msg, rest, strlen(rest) + 1);  
    } else if (nw >= 1 && is_callsign_word(w[0]) && nw <= 2) {  
        /* "<call>" | "<call> TU" */  
        int subtype = JTTY_CALL_CALL;  
        if (nw == 2 && strcmp(w[1], "TU") == 0) subtype = JTTY_CALL_CALL_TU;  
        memset(&atoms[na], 0, sizeof(JttyAtom));  
        atoms[na].kind = JTTY_ATOM_CALL;  
        atoms[na].subtype = subtype;  
        atoms[na].role = JTTY_ROLE_FIELD_ONLY;  
        strncpy(atoms[na].text, w[0], 13);  
        na++;  
        msg[0] = 0;  
    }  
  
    /* Остаток сообщения — TEXT5-атомами по 5 символов */  
    int len = (int)strlen(msg);  
    for (int pos = 0; pos < len && na < JTTY_MAX_FRAMES; pos += 5) {  
        JttyAtom* a = &atoms[na];  
        memset(a, 0, sizeof(*a));  
        a->kind = JTTY_ATOM_TEXT5;  
        a->role = JTTY_ROLE_FIELD_ONLY;  
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
 * 7. FEC: CRC-12(0x80F) + tail-biting свёртка K=10 (0o1167/0o1545)  
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
    if (!jtty_pack_message(msg, &fl, JTTY_EXCHANGE_UNKNOWN, -1))  
        return 0;  
    for (int f = 0; f < fl.nframes; f++)  
        jtty_payload_to_tones(fl.frames[f], tones + (size_t)f * JTTY_NSYMS);  
    return fl.nframes;  
}