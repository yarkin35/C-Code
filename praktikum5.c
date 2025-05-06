#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
//#include "display.h"
#include "display.h"
#include <inttypes.h>
 
// Globale Variablen für zeitmessung und status
volatile uint32_t ms_counter = 0; // Millisekunden-Zähler
volatile uint32_t seconds_since_reset = 0; // Sekunden seit Reset
volatile uint32_t backlight_timer = 0; // Timer für PD13 (Orange LED & Hintergrundbeleuchtung)
volatile uint8_t user_button_pressed = 0; // Status der User-Taste
 
void LCD_SetDimPercentage( uint8_t pcnt )
{   //Input prüfen
    if ( pcnt <= 0 )
    {
        TIM4->CCR2 = 0; // 0% helligkeit
    } else if ( pcnt > 100 ) {
        
        TIM4->CCR2 = 999; //100% helligkeit
    } else {
        
        TIM4->CCR2  = (pcnt*10) - 1;    //Compare Wert Skalieren | -1 weil 0 mitgezählt
    }
}
 //funktion zur ausgabe eines 16-bit-worts an das lcd (wie im original)
void  LCD_Output16BitWord(uint16_t data)
{
 
    uint16_t deleted = 0b1100011100000011;
    GPIOD->ODR &= ~deleted;
    uint16_t deletee = 0b1111111110000000;
    GPIOE->ODR &= ~deletee;
    uint16_t maskd1 = 0b0000000000000011;
    uint16_t maskd2 = 0b0000000000001100;
    uint16_t maskd3 = 0b1110000000000000;
    uint16_t maske1 = 0b0001111111110000;   
    uint16_t temp;
    uint16_t d = (data & maskd1) << 14;
    temp = (data & maskd2) >> 2;
    d |= temp;
    temp = (data & maskd3) >> 5;
    d |= temp;
    uint16_t e = (data & maske1) << 3;
    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
    return;
}
 
 
 // initialisierung aller ports und timer (NEU)
void InitPorts()
{   //Initialisierung
 
    // Einer Clock den Haupttakt zuweisen
    mcpr_SetSystemCoreClock(); //systemtakt initialisieren
 
    // Pins auf Output setzen
    RCC->AHB1ENR |= 1 << 3 | 1; //GPIOD auf an
    GPIOD->MODER |= 1 << 26; // Orange LED auf output
    
    LCD_Init(); //LCD initialisieren
    
 
        //------ TIMER 7 ---------------------------------------------
    RCC->APB1ENR |= 1<<5; //Enable Timer 7
    TIM7->PSC = 999;                                //Prescaler wert 1000
    TIM7->ARR = 83;                                 //ARR wert 84 dass 84Mil/84k -> 1MHz -> 1 mal pro ms
    TIM7->DIER |= 1;                                //Interrupt enable Pin enable
    NVIC_EnableIRQ(TIM7_IRQn);          //Interrupt an
    NVIC_SetPriority(TIM7_IRQn, 0); //Interrpt Priorität semi-hoch
    TIM7->CR1 |= 1;                                 //Counter enable
        
        //------ TIMER 4 ---------------------------------------------
    RCC->APB1ENR |= 1<<2; //Enable Timer 4
    TIM4->PSC = 559;    //Prescaler wert 560
    TIM4->ARR = 999;    //ARR wert 1000 dass 84Mil/560000 = 150 mal pro sekunde von ARR reset
                                        //Zwischen jedem ARR reset findet PWM statt -> Basistakt immer 150Hz
                                        
    // TIM4 Channel 2 = Alternate Function output für PD13
    // CCMR1 OC2M(Bits 12-14) = 110 damit Signal unter Compare-Wert = 1 / drüber = 0
    TIM4->CCMR1 &= ~(1<<12);    
    TIM4->CCMR1 |= 3<<13;
    TIM4->CCER  |= 1<<4;            //CC2E(Bit 4) = 1 -> Comparemodus ein
    TIM4->CCR2  = TIM4->ARR;    //Compare-Wert am Anfang gleich Counter-Max -> 100%
    
    // PD13 Alternate Function Mode
    GPIOD->MODER &= 0xF3FFFFFF;     //MODER Nullen
    GPIOD->MODER |= 0x08000000;     //MODER13 = 1|0 Alternate Fuction
    
    //AFRH13 = 0010 -> AF2 -> TIM4 Channel 2 Output
    GPIOD->AFR[1]    &= ~(0xFF<<20);
    GPIOD->AFR[1]    |= 2<<20;
    //NICHT "AFRL | H" SONDERN Array aus AFR Registern -> AFR[0] | AFR[1]
    
    TIM4->CR1 |= 1;                     //Counter enable
}
 
// Timer 7 Interrupt-Service-Routine
void TIM7_IRQHandler(void) {
    if (TIM7->SR & 1) { // Überlauf-Flag prüfen
        TIM7->SR &= ~1; // Überlauf-Flag zurücksetzen
        ms_counter++; //millisekunden hochzählen
 
        // 1-Sekunden-Takt
        if (ms_counter % 1000 == 0) {
            seconds_since_reset++;
        }
 
        if(backlight_timer>0) { // 10 Sekunden Timer für Display
            backlight_timer--;                                                                                       // Display counter runter
        }
    }
}
 
//Warteschleife für delay_ms millisekunden (busy waiting)
void wait(uint32_t delay_ms) {
    uint32_t start_time = ms_counter;
    while ((ms_counter - start_time) < delay_ms) {
        // Busy-Waiting
    }  
}
 
//Hauptprogramm mit PMw-Dimming-Logik
int main(void) {
    InitPorts();
    //LEDs_InitPorts();
    
    LCD_SetDimPercentage(30);   //Helligkeit auf 30 initialisieren
 
    // Endlosschleife
    while(1) {
        uint32_t ms_start = ms_counter;
        if (GPIOA -> IDR & 1) { // User-Taste abfragen
            user_button_pressed = 1; // Taste gedrückt
            backlight_timer = 20000; // 10 Sekunden #################### EVENTUELL HIER AUF 20000 ERHÖHEN 20k weil Formel im Interrupt unter 10k dimmt -> 10sek hell dann dimmen
            //GPIOE->ODR |= (1 << 13); // PD13 (Orange LED & Hintergrundbeleuchtung) einschalten
            //GPIOD->ODR ^= (1 << 12); // PD12 (Grüne LED) einschalten
        } else {
            user_button_pressed = 0; // Taste nicht gedrückt
            //GPIOD->ODR &= ~(1 << 12); // PD12 (Grüne LED) ausschalten
        }
				LCD_SetDimPercentage( 30 + ( 70 * backlight_timer ) / 10000 );   // Von 100 clean runter auf 30
        /*
        if (backlight_timer == 0) {
            GPIOE->ODR &= ~(1 << 13); // PD13 (Orange LED & Hintergrundbeleuchtung) ausschalten
        }
        */
 
        // Wenn Display Anzeige implementiert ist, dann hier display_time(seconds_since_reset) aufrufen
        // 50 ms loop timing einhalten
        uint32_t ms_dif = ms_start - ms_counter;
        if (ms_dif <= 50) {
                wait(50 - ms_dif); // 50 ms warten
        }
    }
}