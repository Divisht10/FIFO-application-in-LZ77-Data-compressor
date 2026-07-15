#include <stdio.h>
#include <string.h>

// Define sliding window parameters
#define SEARCH_BUFFER_SIZE 15
#define LOOKAHEAD_BUFFER_SIZE 15

// Token structure representing (Offset, Length, Next_Character)
typedef struct {
    int offset;
    int length;
    char next_char;
} LZ77Token;

void compressLZ77(const char* input) {
    int input_len = strlen(input);
    int cursor = 0; // The divider between Search and Lookahead buffers

    printf("%-10s | %-10s | %-15s\n", "Offset", "Length", "Next Character");
    printf("----------------------------------------\n");

    while (cursor < input_len) {
        int best_offset = 0;
        int best_length = 0;

        // Calculate the leftmost boundary of the search window
        int search_start = cursor - SEARCH_BUFFER_SIZE;
        if (search_start < 0) {
            search_start = 0;
        }

        // Scan the search buffer for the longest matching prefix
        for (int i = search_start; i < cursor; i++) {
            int current_length = 0;

            // Count matching characters, ensuring we don't exceed the Lookahead limit
            // or the total length of the input string
            while (current_length < LOOKAHEAD_BUFFER_SIZE &&
                   cursor + current_length < input_len &&
                   input[i + current_length] == input[cursor + current_length]) {
                current_length++;
            }

            // Update if we found a strictly longer match
            if (current_length > best_length) {
                best_length = current_length;
                best_offset = cursor - i;
            }
        }

        // Determine the single character immediately following the match
        char next_char = '\0';
        if (cursor + best_length < input_len) {
            next_char = input[cursor + best_length];
        }

        // Output the generated token
        if (next_char == '\n') {
            printf("%-10d | %-10d | '\\n'\n", best_offset, best_length);
        } else if (next_char == '\0') {
            printf("%-10d | %-10d | EOF\n", best_offset, best_length);
        } else {
            printf("%-10d | %-10d | '%c'\n", best_offset, best_length, next_char);
        }

        // Slide the window forward by the length of the match plus the next character
        cursor += best_length + 1;
    }
}

int main() {
    const char* data = "abracadabra";
    
    printf("Input Data: \"%s\"\n\n", data);
    compressLZ77(data);
    
    return 0;
}