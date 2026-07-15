/*==========================================================================
 *  lz77_robust.c  -- classic LZ77 (offset,length,next_char) over a ring-buffer
 *                    sliding window, made robust + self-verifying.
 *
 *  Token grammar (classic LZ77):  each token = (offset, length, next_char)
 *     - copy `length` bytes from `offset` back, then append `next_char`
 *     - offset == 0 means "no match" (pure literal: just next_char)
 *     - every token advances the cursor by length+1, so output == input length
 *
 *  The decoder copies the match ONE BYTE AT A TIME.  That is what makes
 *  length > offset (self-overlapping / RLE-style) matches reconstruct
 *  correctly: e.g. (offset=2,length=6) keeps reading bytes it just wrote.
 *========================================================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SEARCH_BUFFER_SIZE    255           /* history depth  (max offset)  */
#define LOOKAHEAD_BUFFER_SIZE  64            /* max match window             */
#define WINDOW_SIZE (SEARCH_BUFFER_SIZE + LOOKAHEAD_BUFFER_SIZE)

#define MAX_TOKENS 200000
#define MAX_BYTES  200000

typedef struct { uint16_t offset; uint16_t length; uint8_t next_char; } Token;

/* ---- circular ring buffer holding the whole window (history + lookahead) */
typedef struct {
    uint8_t  buffer[WINDOW_SIZE];
    uint16_t head, tail, count;
} LZ77_FIFO;

static void    fifo_init(LZ77_FIFO *f){ f->head = f->tail = f->count = 0; }

static bool fifo_enqueue(LZ77_FIFO *f, uint8_t d){
    if (f->count >= WINDOW_SIZE) return false;          /* overflow guard   */
    f->buffer[f->head] = d;
    f->head = (f->head + 1) % WINDOW_SIZE;
    f->count++;
    return true;
}

static uint8_t fifo_dequeue(LZ77_FIFO *f){
    if (f->count == 0) return 0;                         /* underflow guard  */
    uint8_t d = f->buffer[f->tail];
    f->tail = (f->tail + 1) % WINDOW_SIZE;
    f->count--;
    return d;
}

/* peek relative to tail (oldest byte); used to scan the window */
static uint8_t fifo_peek(LZ77_FIFO *f, uint16_t off){
    if (off >= f->count) return 0;
    return f->buffer[(f->tail + off) % WINDOW_SIZE];
}

/*--------------------------------------------------------------------------
 *  Compressor.  Returns number of tokens written into `tokens`.
 *  Works on an explicit (input,in_len) so it is binary-safe (no strlen,
 *  embedded zero bytes are fine).
 *------------------------------------------------------------------------*/
static int lz77_compress(const uint8_t *input, int in_len, Token *tokens){
    LZ77_FIFO w; fifo_init(&w);
    int search_size = 0, lookahead_size = 0, in_pos = 0, ntok = 0;

    /* pre-fill the lookahead */
    while (lookahead_size < LOOKAHEAD_BUFFER_SIZE && in_pos < in_len){
        fifo_enqueue(&w, input[in_pos++]);
        lookahead_size++;
    }

    while (lookahead_size > 0){
        int best_off = 0, best_len = 0;
        int max_match = lookahead_size - 1;             /* leave 1 for next_char */
        if (max_match < 0) max_match = 0;

        /* scan every history start position i (cursor sits at search_size) */
        for (int i = 0; i < search_size; i++){
            int cl = 0;
            /* extend; peek(search_size+cl) may read into the lookahead, which
               is how a match can run longer than its offset (overlap)         */
            while (cl < max_match &&
                   fifo_peek(&w, i + cl) == fifo_peek(&w, search_size + cl))
                cl++;
            if (cl > best_len){ best_len = cl; best_off = search_size - i; }
        }

        tokens[ntok].offset    = (uint16_t)best_off;
        tokens[ntok].length    = (uint16_t)best_len;
        tokens[ntok].next_char = fifo_peek(&w, search_size + best_len);
        ntok++;

        /* slide the window forward by (length + 1) */
        int step = best_len + 1;
        for (int s = 0; s < step; s++){
            if (search_size >= SEARCH_BUFFER_SIZE) fifo_dequeue(&w);  /* drop oldest */
            else                                   search_size++;     /* history grows */
            if (in_pos < in_len) fifo_enqueue(&w, input[in_pos++]);   /* refill lookahead */
            else                 lookahead_size--;                    /* EOF: lookahead drains */
        }
    }
    return ntok;
}

/*--------------------------------------------------------------------------
 *  Decompressor.  Byte-at-a-time copy => correct for length > offset.
 *------------------------------------------------------------------------*/
static int lz77_decompress(const Token *tokens, int ntok, uint8_t *out){
    int pos = 0;
    for (int t = 0; t < ntok; t++){
        int off = tokens[t].offset;
        int len = tokens[t].length;
        for (int k = 0; k < len; k++){          /* one byte at a time -> overlap-safe */
            out[pos] = out[pos - off];
            pos++;
        }
        out[pos++] = tokens[t].next_char;
    }
    return pos;
}

/*-------------------------------- demos --------------------------------*/
static Token  g_tokens[MAX_TOKENS];
static uint8_t g_out[MAX_BYTES];

static void show_tokens(const char *name, const uint8_t *in, int n){
    int ntok = lz77_compress(in, n, g_tokens);
    printf("\n== %s :  \"%.*s\"  (%d bytes) ==\n", name, n, in, n);
    printf("%-8s %-8s %-10s %s\n", "Offset", "Length", "Next", "");
    for (int t = 0; t < ntok; t++){
        uint8_t c = g_tokens[t].next_char;
        char cs[8];
        if (c >= 32 && c < 127) snprintf(cs, sizeof cs, "'%c'", c);
        else                    snprintf(cs, sizeof cs, "0x%02x", c);
        const char *tag = (g_tokens[t].offset > 0 &&
                           g_tokens[t].length > g_tokens[t].offset) ? "<-- overlap (len>off)" : "";
        printf("%-8u %-8u %-10s %s\n",
               g_tokens[t].offset, g_tokens[t].length, cs, tag);
    }
}

static int round_trip(const char *name, const uint8_t *in, int n){
    int ntok    = lz77_compress(in, n, g_tokens);
    int out_len = lz77_decompress(g_tokens, ntok, g_out);
    int ok = (out_len == n) && (memcmp(g_out, in, n) == 0);
    printf("  %-16s in=%5d tokens=%5d  -> %s\n",
           name, n, ntok, ok ? "ROUND-TRIP OK" : "*** FAIL ***");
    return ok;
}

int main(void){
    /* ---- token showcases, including the overlap case you asked for ---- */
    show_tokens("abracadabra", (const uint8_t*)"abracadabra", 11);
    show_tokens("xyxyzyxyxyz", (const uint8_t*)"xyxyzyxyxyz", 11);   /* length>offset */
    show_tokens("aaaaaaaa",    (const uint8_t*)"aaaaaaaa",     8);   /* pure RLE      */

    /* ---- round-trip verification on tougher inputs ---- */
    printf("\n-- round-trip verification --\n");
    int all = 1;
    all &= round_trip("abracadabra", (const uint8_t*)"abracadabra", 11);
    all &= round_trip("xyxyzyxyxyz", (const uint8_t*)"xyxyzyxyxyz", 11);
    all &= round_trip("ababab...",   (const uint8_t*)"abababababababababab", 20);
    all &= round_trip("rle_a x40",   (const uint8_t*)"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 40);
    all &= round_trip("english",
        (const uint8_t*)"the quick brown fox jumps over the lazy dog the quick brown fox", 62);
    all &= round_trip("single",      (const uint8_t*)"Q", 1);
    all &= round_trip("empty",       (const uint8_t*)"", 0);

    /* long input that exceeds the window (forces dequeue + overlap together) */
    static uint8_t big[5000];
    int bn = 0;
    const char *pat = "xyxyzyxyxyz_";            /* repeats an overlap-friendly pattern */
    while (bn < 5000) big[bn] = pat[bn % 12], bn++;
    all &= round_trip("long_5000", big, 5000);

    /* all-distinct bytes (worst case: every token a literal) */
    static uint8_t distinct[256];
    for (int i = 0; i < 256; i++) distinct[i] = (uint8_t)i;
    all &= round_trip("distinct_256", distinct, 256);

    printf("\n%s\n", all ? "ALL ROUND-TRIPS PASSED" : "SOME ROUND-TRIPS FAILED");
    return all ? 0 : 1;
}
