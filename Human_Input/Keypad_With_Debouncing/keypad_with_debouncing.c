/**
 * Eduardo Diaz
 * 
 * Keypad With Debouncing (Built off of Keypad Demo)
 * 
 * 
 * 
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

#include "pt_cornell_rp2040_v1_4.h"


// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;


char keytext[40];

typedef enum {
    STATE_NOT_PRESSED,
    STATE_MAYBE_PRESSED,
    STATE_PRESSED,
    STATE_MAYBE_NOT_PRESSED
} KeypadState;

KeypadState current_state = STATE_NOT_PRESSED;

int scan_keypad() {
    int val;
    // Scan the keypad!
    // Some variables
    static int i ;
    static uint32_t keypad ;
    for (i=0; i<KEYROWS; i++) {
        // Set a row high
        gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                        (scancodes[i] << BASE_KEYPAD_PIN)) ;
        // Small delay required
        sleep_us(1) ; 
        // Read the keycode
        keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
        // Break if button(s) are pressed
        if (keypad & button) break;
    }
    // If we found a button . . .
    if (keypad & button) {
        // Look for a valid keycode.
        for (i=0; i<NUMKEYS; i++) {
            if (keypad == keycodes[i]) {
                // Save keypad value
                val = keypad;
                break;
            }
        }
        // If we don't find one, report invalid keycode
        if (i==NUMKEYS) (val = -1) ;
    }
    // Otherwise, indicate invalid/non-pressed buttons
    else (val=-1) ;
    
    return val;
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int possible_keycode;

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        switch (current_state) {
            case STATE_NOT_PRESSED:
                possible_keycode = scan_keypad();
                if (possible_keycode == -1) {
                    current_state = STATE_NOT_PRESSED;
                } else {
                    current_state = STATE_MAYBE_PRESSED;
                }
                break;
            case STATE_MAYBE_PRESSED:
                if (possible_keycode == scan_keypad()) {
                    // Print key to terminal
                    for (int i=0; i<NUMKEYS; i++) {
                        if (possible_keycode == keycodes[i]) {
                            printf("\n%d", i);
                            break;
                        }
                    }
                    current_state = STATE_PRESSED;
                } else {
                    current_state = STATE_NOT_PRESSED;
                }
                break;
            case STATE_PRESSED:
                if (possible_keycode == scan_keypad()) {
                    // Do nothing remain in STATE_PRESSED state
                    current_state = STATE_PRESSED;
                } else {
                    current_state = STATE_MAYBE_NOT_PRESSED;
                }
                break;
            case STATE_MAYBE_NOT_PRESSED:
                if (possible_keycode == scan_keypad()) {
                    // Transition back to pressed state
                    current_state = STATE_PRESSED;
                } else {
                    current_state = STATE_NOT_PRESSED;
                }
                break;
        }

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}


int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    printf("Raspberry Pi Pico\n");
    printf("Keypad With Debouncing Demo\n");
    printf("Eduardo Diaz\n");
    printf("Key Pressed: \n");

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
