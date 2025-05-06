#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include "display.h"
#include <inttypes.h>
#include <stdio.h>

// Globale Variablen
volatile uint32_t ms_counter = 0; // Millisekunden-Zähler
volatile uint32_t seconds_since_reset = 0; // Sekunden seit Reset
volatile uint32_t backlight_timer = 0; // Timer für PD13 (Orange LED & Hintergrundbeleuchtung)
volatile uint8_t user_button_pressed = 0; // Status der User-Taste
volatile uint8_t loops = 0; // Zählen der While Durchläufe

//Prototyp für anzeige funktionen
void display_time(uint32_t seconds);

//funktion zur ausgabe eines 16-bit-worts an das lcd (bitmapping auf GPIO)
void  LCD_Output16BitWord(uint16_t data)
{

    uint16_t deleted = 0b1100011100000011; //maske für zu löschende bits in gpio
    GPIOD->ODR &= ~deleted; //löschen der bits
    
    uint16_t deletee = 0b1111111110000000; // maske für zu löschende bits
    GPIOE->ODR &= ~deletee; //löschen der bits
    
    //masken fuer die bitmanipulation
    uint16_t maskd1 = 0b0000000000000011; 
    uint16_t maskd2 = 0b0000000000001100; 
    uint16_t maskd3 = 0b1110000000000000;
    uint16_t maske1 = 0b0001111111110000;
    uint16_t temp;
    //bits für gpiod zusammensetzen
    uint16_t d = (data & maskd1) << 14;
    temp = (data & maskd2) >> 2;
    d |= temp;
    temp = (data & maskd3) >> 5;
    d |= temp;
    //bits für gpioe zusammensetzen
    uint16_t e = (data & maske1) << 3;
    
    //bits setzen
    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
    
    return;
}

// Timer 7 Interrupt-Service-Routine
void TIM7_IRQHandler(void) {
    if (TIM7->SR & 1) { // Überlauf-Flag prüfen
        TIM7->SR &= ~1; // Überlauf-Flag zurücksetzen
        ms_counter++;  //ms hochzaehlen

				// 1-Sekunden-Takt, also jede sek erhöhen
        if (ms_counter % 1000 == 0) {
            seconds_since_reset++;
        }
				
				// 10-Sekunden-Timer für Orange LED & Hintergrundbeleuchtung
        if (backlight_timer > 0) {
            backlight_timer--;
        }
    }
}
//warteschleife für delay_ms millisekunden (busy waiting)
void wait(uint32_t delay_ms) {
    uint32_t start_time = ms_counter;
    while ((ms_counter - start_time) < delay_ms) {
        // warten bis zeit abgelaufen
    }  
}

int main(void) {

    mcpr_SetSystemCoreClock(); //systemtskt initialisieren
    //LEDs_InitPorts();

    // Timer 7 konfigurieren
    RCC->APB1ENR |= (1 << 5); // Timer 7 Clock aktivieren
    TIM7->PSC = 83;           // Prescaler: 84 MHz / (83 + 1) = 1 MHz
    TIM7->ARR = 999;          // Auto-Reload: 1 MHz / (999 + 1) = 1 kHz (1 ms)
    TIM7->DIER |= 1;          // Update-Interrupt aktivieren
    NVIC_EnableIRQ(TIM7_IRQn); // Timer 7 Interrupt im NVIC aktivieren
    NVIC_SetPriority(TIM7_IRQn, 5); // Interrupt-Priorität setzen
    TIM7->CR1 |= 1;           // Timer starten
	
		// Pins auf Output setzen
		RCC->AHB1ENR |= 1 << 3 | 1; //GPIOD auf an
		GPIOD->MODER |= 1 << 24; // Grüne LED auf output
		GPIOD->MODER |= 1 << 26; // Orange LED auf output
	
		LCD_Init();

    // Endlosschleife
    while(1) {
				
				uint32_t ms_start = ms_counter;
        if ((GPIOA -> IDR & 1) != 0) { // User-Taste abfragen
					 
            user_button_pressed = 1; // Taste gedrückt
            backlight_timer = 10000; // 10 Sekunden
            GPIOD->ODR |= (1 << 13); // PD13 (Orange LED & Hintergrundbeleuchtung) einschalten
            if (loops >= 2){
                    GPIOD->ODR ^= (1 << 12); // PD12 (Grüne LED) einschalten
                    loops = 0;
            }
        } else {
            user_button_pressed = 0; // Taste nicht gedrückt
            GPIOD->ODR &= ~(1 << 12); // PD12 (Grüne LED) ausschalten
        }

        if (backlight_timer == 0) {
            GPIOD->ODR &= ~(1 << 13); // PD13 (Orange LED & Hintergrundbeleuchtung) ausschalten
        }

                display_time(seconds_since_reset); //sekundenanzeige auf display aktualisieren
        
        loops++; //durchläufer zählen
        
        uint32_t ms_dif = ms_start - ms_counter; // 50 ms loop timing einhalten
        if (ms_dif <= 50) {
                wait(50 - ms_dif); // 50 ms warten
        }
    }
}
//Anzeige der sek aud dem lcd
void display_time(uint32_t seconds) {
		LCD_ClearDisplay( 0xFE00 ); //display löschen
		char str[32];
		sprintf(str, "%u", seconds); //sekunden als string
		LCD_WriteString( 10, 10, 0xFFFF, 0x0000, str); // string ausgeben
}
