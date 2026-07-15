#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Define sliding window parameters for embedded systems
#define SEARCH_BUFFER_SIZE 15
#define LOOKAHEAD_BUFFER_SIZE 15
#define WINDOW_SIZE (SEARCH_BUFFER_SIZE + LOOKAHEAD_BUFFER_SIZE)

// --------------------------------------------------------
// 1. Circular FIFO Ring Buffer Implementation
// --------------------------------------------------------
typedef struct {
    uint8_t buffer[WINDOW_SIZE];
    uint16_t head;  // Where we insert new data
    uint16_t tail;  // Where the oldest data sits
    uint16_t count; // Total characters currently in the buffer
} LZ77_FIFO;

void fifo_init(LZ77_FIFO *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

bool fifo_enqueue(LZ77_FIFO *fifo, uint8_t data) {
    if (fifo->count >= WINDOW_SIZE) return false; // Buffer Overflow
    fifo->buffer[fifo->head] = data;
    fifo->head = (fifo->head + 1) % WINDOW_SIZE;
    fifo->count++;
    return true;
}

uint8_t fifo_dequeue(LZ77_FIFO *fifo) {
    if (fifo->count == 0) return 0; // Buffer Underflow
    uint8_t data = fifo->buffer[fifo->tail];
    fifo->tail = (fifo->tail + 1) % WINDOW_SIZE;
    fifo->count--;
    return data;
}

// Peek at a value relative to the tail (oldest data) without removing it.
// This is critical for the LZ77 algorithm to search the sliding window.
uint8_t fifo_peek(LZ77_FIFO *fifo, uint16_t offset) {
    if (offset >= fifo->count) return 0;
    uint16_t index = (fifo->tail + offset) % WINDOW_SIZE;
    return fifo->buffer[index];
}


// --------------------------------------------------------
// 2. LZ77 Compression over FIFO
// --------------------------------------------------------

// Helper function to shift the sliding window over the FIFO
void shift_window(LZ77_FIFO *fifo, int *search_size, int *lookahead_size, int step, const char **input_stream) {
    for (int i = 0; i < step; i++) {
        // 1. Manage the Search Buffer (Tail side of FIFO)
        if (*search_size >= SEARCH_BUFFER_SIZE) {
            fifo_dequeue(fifo); // Drop the oldest character permanently
        } else {
            (*search_size)++;   // Allow the search buffer to grow until it hits the max size
        }

        // 2. Manage the Lookahead Buffer (Head side of FIFO)
        if (**input_stream != '\0') {
            fifo_enqueue(fifo, **input_stream);
            (*input_stream)++; // Read next char from our simulated incoming stream
        } else {
            // EOF reached, the lookahead buffer simply shrinks until empty
            (*lookahead_size)--;
        }
    }
}

void compressLZ77_FIFO(const char* input_stream) {
    LZ77_FIFO window;
    fifo_init(&window);

    int search_size = 0;
    int lookahead_size = 0;

    printf("%-10s | %-10s | %-15s\n", "Offset", "Length", "Next Character");
    printf("----------------------------------------\n");

    // Pre-fill the Lookahead Buffer with the start of the data stream
    while (lookahead_size < LOOKAHEAD_BUFFER_SIZE && *input_stream != '\0') {
        fifo_enqueue(&window, *input_stream);
        input_stream++;
        lookahead_size++;
    }

    // Main Compression Loop
    while (lookahead_size > 0) {
        int best_offset = 0;
        int best_length = 0;

        // We must leave at least 1 character for the "Next Character" token,
        // unless there is exactly 1 character left in the buffer.
        int max_match_len = lookahead_size - 1;
        if (max_match_len < 0) max_match_len = 0;

        // Scan the Search Buffer portion of the FIFO
        for (int i = 0; i < search_size; i++) {
            int current_length = 0;

            // Check how many characters match between the search area and lookahead area
            while (current_length < max_match_len && 
                   fifo_peek(&window, i + current_length) == fifo_peek(&window, search_size + current_length)) {
                current_length++;
            }

            // Update if we found a strictly longer match
            if (current_length > best_length) {
                best_length = current_length;
                best_offset = search_size - i; // Distance from cursor back to match
            }
        }

        // Extract the next character immediately following the match
        char next_char = fifo_peek(&window, search_size + best_length);

        // Output the Token
        if (lookahead_size == 1 && best_length == 0 && next_char == '\0') {
             printf("%-10d | %-10d | EOF\n", best_offset, best_length); // Edge case handling
        } else {
             printf("%-10d | %-10d | '%c'\n", best_offset, best_length, next_char);
        }

        // Advance the sliding window by (Length + 1)
        int step = best_length + 1;
        shift_window(&window, &search_size, &lookahead_size, step, &input_stream);
    }
}

int main() {
    // Simulating a stream of data arriving from a sensor or file
    const char* data_stream = "abracadabra";
    
    printf("Streaming Input: \"%s\"\n\n", data_stream);
    compressLZ77_FIFO(data_stream);
    
    return 0;
}