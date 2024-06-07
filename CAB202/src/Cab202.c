#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h> // For boolean type
#include <avr/eeprom.h>

// Constants
#define NUM_BUTTONS 4
#define NUM_SEGMENTS 7
#define SEED_ADDR ((uint8_t*)10)
#define HIGHSCORE_ADDR ((uint8_t*)20)
#define DEBOUNCE_DELAY 50 // 50 ms debounce delay

const uint16_t noteFrequencies[NUM_BUTTONS] = {330, 277, 440, 165};

// Function prototypes
void initializeGame();
void generateSequence();
void displaySimonSequence();
bool getUserInput();
void displayFailPattern();
void displaySuccessPattern();
void increaseScore();
void handleUARTCommands();
uint8_t readHighScore();
void writeHighScore(uint8_t score);
void clearHighScore();
void debounceButtons();
void updatePlaybackDelay();
void playTone(uint16_t frequency);
void silent();
void uartInit();
void uartTransmit(uint8_t data);
uint8_t uartReceive();
unsigned long millis();

// Global variables
volatile uint8_t sequenceLength;
volatile uint8_t sequence[25];
volatile uint8_t userSequence[25];
volatile uint8_t score;
volatile uint8_t buttonState;
volatile uint8_t lastButtonState;
volatile unsigned long lastDebounceTime = 0;

// Millis counter
volatile unsigned long timer0_millis = 0;

// Timer interrupt for millis
ISR(TCA0_OVF_vect) {
    timer0_millis++;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

// UART RX interrupt
ISR(USART0_RXC_vect) {
    handleUARTCommands();
}

// Pin change interrupt for buttons
ISR(PORTA_PORT_vect) {
    debounceButtons();
    PORTA.INTFLAGS = 0xFF; // Clear all interrupt flags
}

int main() {
    initializeGame();
    sei();

    while (1) {
        generateSequence();
        displaySimonSequence();

        if (!getUserInput()) {
            displayFailPattern();
            break;
        }

        displaySuccessPattern();
        increaseScore();
    }

    return 0;
}

void initializeGame() {
    // Initialize pins
    VPORTA.DIR = 0x00; // Port A as input for buttons
    VPORTB.DIR = 0xFF; // Port B as output for 7-segment display and LEDs

    // Initialize Timer for millis
    TCA0.SINGLE.PER = 250; // Period for ~1ms overflow
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;

    // Enable pin change interrupt
    PORTA.PIN0CTRL |= PORT_ISC_FALLING_gc; // Configure pin change interrupt on falling edge
    PORTA.PIN1CTRL |= PORT_ISC_FALLING_gc; // Configure pin change interrupt on falling edge
    PORTA.PIN2CTRL |= PORT_ISC_FALLING_gc; // Configure pin change interrupt on falling edge
    PORTA.PIN3CTRL |= PORT_ISC_FALLING_gc; // Configure pin change interrupt on falling edge

    // Initialize UART
    uartInit();

    // Initialize variables
    sequenceLength = 1;
    score = 0;
    clearHighScore();
    buttonState = 0;
    lastButtonState = 0;
}

void generateSequence() {
    uint32_t state_lfsr = eeprom_read_dword((uint32_t*)SEED_ADDR);
    uint32_t mask = 0xE2023CAB;

    for (uint8_t i = 0; i < sequenceLength; i++) {
        bool bit = state_lfsr & 1;
        state_lfsr >>= 1;
        if (bit) {
            state_lfsr ^= mask;
        }
        sequence[i] = state_lfsr & 0b11;
    }
}

void displaySimonSequence() {
    for (uint8_t i = 0; i < sequenceLength; i++) {
        VPORTB.OUT = (1 << sequence[i]);
        _delay_ms(500);
        VPORTB.OUT = 0x00;
        _delay_ms(500);

        playTone(noteFrequencies[sequence[i]]);
        _delay_ms(250);
        silent();
        _delay_ms(250);
    }
}

bool getUserInput() {
    for (uint8_t i = 0; i < sequenceLength; i++) {
        while (buttonState == 0) {}

        if (buttonState != sequence[i]) {
            return false;
        }

        VPORTB.OUT = (1 << buttonState);
        _delay_ms(500);
        VPORTB.OUT = 0x00;
        _delay_ms(500);

        while (buttonState != 0) {}
    }
    return true;
}

void displayFailPattern() {
    VPORTB.OUT = (1 << 6);
    _delay_ms(1000);
    silent();
    VPORTB.OUT = 0x00;
}

void displaySuccessPattern() {
    VPORTB.OUT = 0xFF;
    _delay_ms(1000);
    silent();
    VPORTB.OUT = 0x00;
}

void increaseScore() {
    score++;
    if (score > readHighScore()) {
        writeHighScore(score);
    }
}

void handleUARTCommands() {
    uint8_t command = uartReceive();

    switch (command) {
        case 'R':
            sequenceLength = 1;
            score = 0;
            clearHighScore();
            break;
        case 'I':
            // Increase playback frequency
            break;
        case 'D':
            // Decrease playback frequency
            break;
        case 'S':
            // Set new seed
            break;
        default:
            break;
    }
}

uint8_t readHighScore() {
    return eeprom_read_byte(HIGHSCORE_ADDR);
}

void writeHighScore(uint8_t score) {
    eeprom_write_byte(HIGHSCORE_ADDR, score);
}

void clearHighScore() {
    eeprom_write_byte(HIGHSCORE_ADDR, 0);
}

void debounceButtons() {
    uint8_t reading = VPORTA.IN & 0x0F;

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
        }
    }

    lastButtonState = reading;
}

void updatePlaybackDelay() {
    // Read the potentiometer value and update playback delay
}

void playTone(uint16_t frequency) {
    // Generate tone with specified frequency
}

void silent() {
    // Turn off tone
}

void uartInit() {
    uint16_t baud = (F_CPU / (16UL * 9600)) - 1; // UL ensures proper arithmetic for unsigned long
    USART0.BAUD = (uint16_t)baud;
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_RXCIE_bm;
}

void uartTransmit(uint8_t data) {
    while (!(USART0.STATUS & USART_DREIF_bm)) {}
    USART0.TXDATAL = data;
}

uint8_t uartReceive() {
    while (!(USART0.STATUS & USART_RXCIF_bm)) {}
    return USART0.RXDATAL;
}

unsigned long millis() {
    unsigned long m;
    uint8_t oldSREG = SREG;
    cli();
    m = timer0_millis;
    SREG = oldSREG;
    return m;
}