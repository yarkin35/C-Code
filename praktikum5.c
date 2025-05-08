#include "stm32f4xx.h"          // Grundlegende STM32F4 Definitionen und Registerzugriff
#include "_mcpr_stm32f407.h"    // Board-spezifische Funktionen und Einstellungen
//#include "display.h"
#include "display.h"            // Display-Treiberfunktionen (wird eingebunden)
#include <inttypes.h>           // Standardisierte Integer-Typen wie uint32_t

// Globale Variablen für zeitmessung und status
volatile uint32_t ms_counter = 0; // Zählt die vergangenen Millisekunden (wird im Timer7-Interrupt erhöht)
volatile uint32_t seconds_since_reset = 0; // Zählt die vergangenen Sekunden seit Reset (wird im Timer7-Interrupt erhöht)
volatile uint32_t backlight_timer = 0; // Timer für die Hintergrundbeleuchtung (und PWM-Dimming)
volatile uint8_t user_button_pressed = 0; // Speichert, ob der User-Button aktuell gedrückt ist (1=gedrückt, 0=nicht gedrückt)

// Funktion zum Setzen der Display-Helligkeit per PWM (Prozentwert 0...100)
void LCD_SetDimPercentage( uint8_t pcnt )
{   
    // Prüfen, ob der Prozentwert <= 0 ist (Display komplett dunkel)
    if ( pcnt <= 0 )
    {
        TIM4->CCR2 = 0; // PWM-Vergleichswert auf 0, Display aus
    } else if ( pcnt > 100 ) {
        TIM4->CCR2 = 1000; // Maximalwert: 100% Helligkeit
    } else {
        TIM4->CCR2  = (pcnt*10);    // PWM-Wert skalieren, 0 zählt mit
    }
}
 
// Funktion zur Ausgabe eines 16-Bit-Worts an das LCD (wie im Original)
void  LCD_Output16BitWord(uint16_t data)
{
    // Maske zum Löschen der relevanten Bits in GPIOD
    uint16_t deleted = 0b1100011100000011;
    GPIOD->ODR &= ~deleted;
    // Maske zum Löschen der relevanten Bits in GPIOE
    uint16_t deletee = 0b1111111110000000;
    GPIOE->ODR &= ~deletee;
    // Masken für die Bitmanipulation
    uint16_t maskd1 = 0b0000000000000011;
    uint16_t maskd2 = 0b0000000000001100;
    uint16_t maskd3 = 0b1110000000000000;
    uint16_t maske1 = 0b0001111111110000;   
    uint16_t temp;
    // Bits für GPIOD zusammensetzen
    uint16_t d = (data & maskd1) << 14;
    temp = (data & maskd2) >> 2;
    d |= temp;
    temp = (data & maskd3) >> 5;
    d |= temp;
    // Bits für GPIOE zusammensetzen
    uint16_t e = (data & maske1) << 3;
    // Daten an die GPIO-Ports ausgeben
    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
    return;
}
 
// Initialisierung aller Ports und Timer (NEU)
void InitPorts()
{   
    // Systemtakt initialisieren
    mcpr_SetSystemCoreClock(); // 84MHz Systemtakt einstellen
 
    // GPIO-Takt für GPIOD und GPIOA aktivieren
    RCC->AHB1ENR |= 1 << 3 | 1; // GPIOD (LEDs) und GPIOA (Button) aktivieren
    GPIOD->MODER |= 1 << 26; // Orange LED (PD13) als Output konfigurieren
    
    LCD_Init(); // LCD initialisieren
    
    //------ TIMER 7 ---------------------------------------------
    RCC->APB1ENR |= 1<<5; // Timer 7 Clock aktivieren
    TIM7->PSC = 999;      // Prescaler: 84MHz / (999 + 1) = 84kHz
    TIM7->ARR = 83;       // Auto-Reload: 84kHz / (83 + 1) = 1kHz (1ms)
    TIM7->DIER |= 1;      // Update-Interrupt aktivieren
    NVIC_EnableIRQ(TIM7_IRQn);          // Timer7-Interrupt im NVIC aktivieren
    NVIC_SetPriority(TIM7_IRQn, 0);     // Interrupt-Priorität setzen (hoch)
    TIM7->CR1 |= 1;       // Timer starten
        
    //------ TIMER 4 ---------------------------------------------
    RCC->APB1ENR |= 1<<2; // Timer 4 Clock aktivieren
    TIM4->PSC = 559;      // Prescaler: 84MHz / (559 + 1) = 150kHz
    TIM4->ARR = 999;      // Auto-Reload: 150kHz / (999 + 1) = 150Hz
    // PWM-Basisfrequenz: 150Hz
    
    // TIM4 Channel 2 = Alternate Function Output für PD13 (PWM für Backlight)
    // CCMR1 OC2M (Bits 12-14) = 110: PWM-Modus 1
    TIM4->CCMR1 &= ~(1<<12);    
    TIM4->CCMR1 |= 3<<13;
    TIM4->CCER  |= 1<<4;        // CC2E (Bit 4) = 1: Output Enable für Channel 2
    TIM4->CCR2  = TIM4->ARR;    // Startwert: 100% Helligkeit (Compare = ARR)
    
    // PD13 Alternate Function Mode (AF2 für TIM4_CH2)
    GPIOD->MODER &= 0xF3FFFFFF;     // MODER13 auf 10 (Alternate Function)
    GPIOD->MODER |= 0x08000000;     // MODER13 = 10
    // Alternate Function Register: AFRH13 = 0010 -> AF2 (TIM4_CH2)
    GPIOD->AFR[1]    &= ~(0xFF<<20);
    GPIOD->AFR[1]    |= 2<<20;
    // Hinweis: AFR[1] für Pins 8-15
    
    TIM4->CR1 |= 1;                 // Timer4 Counter starten
}
 
// Timer 7 Interrupt-Service-Routine (wird jede ms aufgerufen)
void TIM7_IRQHandler(void) {
    if (TIM7->SR & 1) { // Überlauf-Flag prüfen
        TIM7->SR &= ~1; // Überlauf-Flag zurücksetzen
        ms_counter++; // Millisekunden hochzählen
 
        // Jede volle Sekunde (1000ms) Sekunden hochzählen
        if (ms_counter % 1000 == 0) {
            seconds_since_reset++;
        }
 
        if(backlight_timer>0) { // Wenn der Backlight-Timer aktiv ist
            backlight_timer--;  // Timer dekrementieren (jede ms)
        }
    }
}
 
// Warteschleife für delay_ms Millisekunden (busy waiting)
void wait(uint32_t delay_ms) {
    uint32_t start_time = ms_counter;
    while ((ms_counter - start_time) < delay_ms) {
        // Busy-Waiting: tut nichts, wartet nur ab
    }  
}
 
// Hauptprogramm mit PWM-Dimming-Logik
int main(void) {
    InitPorts(); // Alle Ports und Timer initialisieren
    //LEDs_InitPorts(); // (Nicht verwendet)
    
    LCD_SetDimPercentage(30);   // Helligkeit beim Start auf 30% setzen
 
    // Endlosschleife
    while(1) {
        uint32_t ms_start = ms_counter; // Startzeit merken für Loop-Timing
        if (GPIOA -> IDR & 1) { // User-Taste (PA0) abfragen
            user_button_pressed = 1; // Taste gedrückt
            backlight_timer = 20000; // 20 Sekunden für Backlight (wird runtergezählt)
            //GPIOE->ODR |= (1 << 13); // (optional) Orange LED einschalten
            //GPIOD->ODR ^= (1 << 12); // (optional) Grüne LED toggeln
        } else {
            user_button_pressed = 0; // Taste nicht gedrückt
            //GPIOD->ODR &= ~(1 << 12); // (optional) Grüne LED ausschalten
        }
        // Helligkeit dynamisch anpassen: von 100% (bei Tastendruck) auf 30% (nach Ablauf des Timers)
        LCD_SetDimPercentage( 30 + ( 70 * backlight_timer ) / 10000 );   // Formel für sanftes Dimmen

        /*
        if (backlight_timer == 0) {
            GPIOE->ODR &= ~(1 << 13); // (optional) Orange LED ausschalten
        }
        */
 
        // Hier könnte eine Display-Ausgabe erfolgen, z.B. display_time(seconds_since_reset)
        
        // 50 ms Loop-Timing einhalten
        uint32_t ms_dif = ms_start - ms_counter;
        if (ms_dif <= 50) {
                wait(50 - ms_dif); // Warten, bis 50 ms vergangen sind
        }
    }
}
