#include "stm32f10x.h"
#include "stdio.h"

#define LCD_ADDR (0x27 << 1)  // I2C address for the LCD
#define LCD_BACKLIGHT (1 << 3)  // Backlight control bit
#define LCD_COMMAND 0
#define LCD_DATA 1

int adc_value;
int press_count = 0;  // Variable to keep track of the number of button presses

// Simple delay loop for STM32 running at 36 MHz
void delay_ms(uint32_t delay) {
    for (volatile uint32_t i = 0; i < (delay * 3600); i++);
}

void I2C1_Init(void) {
    RCC->APB2ENR |= (1 << 3);  // Enable GPIOB clock
    RCC->APB1ENR |= (1 << 21); // Enable I2C1 clock

    // Configure PB6 and PB7 for I2C (alternate function open-drain)
    GPIOB->CRL &= ~(0xFF000000);      // Clear mode and config for PB6, PB7
    GPIOB->CRL |= (0xFF000000);       // Set to AF Open-Drain mode at 50MHz for PB6, PB7

    // Reset and initialize I2C1
    I2C1->CR1 |= (1 << 15);           // Reset I2C1
    I2C1->CR1 &= ~(1 << 15);          // Clear reset bit
    I2C1->CR2 = 36;                   // Set peripheral clock frequency (36MHz)
    I2C1->CCR = 180;                  // Set SCL clock speed (100kHz)
    I2C1->TRISE = 37;                 // Maximum rise time (1000ns / T(PCLK))
    I2C1->CR1 |= (1 << 0);            // Enable I2C1
}

void I2C1_Write(uint8_t address, uint8_t *data, uint8_t size) {
    I2C1->CR1 |= (1 << 8);  // Generate START condition
    while (!(I2C1->SR1 & (1 << 0)));  // Wait for START to be generated

    I2C1->DR = address;  // Send device address
    while (!(I2C1->SR1 & (1 << 1)));  // Wait for ADDR flag
    (void)I2C1->SR2;  // Clear ADDR flag by reading SR1 and SR2

    // Send data
    for (int i = 0; i < size; i++) {
        I2C1->DR = data[i];
        while (!(I2C1->SR1 & (1 << 7)));  // Wait for TXE
    }

    I2C1->CR1 |= (1 << 9);  // Generate STOP condition
}

void LCD_Send(uint8_t data, uint8_t rs) {
    uint8_t high_nibble = (data & 0xF0) | LCD_BACKLIGHT | rs;
    uint8_t low_nibble = ((data << 4) & 0xF0) | LCD_BACKLIGHT | rs;

    uint8_t data_arr[4] = {
        high_nibble | (1 << 2),  // En = 1
        high_nibble & ~(1 << 2), // En = 0
        low_nibble | (1 << 2),   // En = 1
        low_nibble & ~(1 << 2)   // En = 0
    };

    I2C1_Write(LCD_ADDR, data_arr, 4);
}

void LCD_SendCommand(uint8_t cmd) {
    LCD_Send(cmd, LCD_COMMAND);
    delay_ms(2);  // Command delay
}

void LCD_SendData(uint8_t data) {
    LCD_Send(data, LCD_DATA);
    delay_ms(2);  // Data delay
}

void LCD_Init(void) {
    delay_ms(50);  // Wait for LCD to power-up

    LCD_SendCommand((1 << 5) | (1 << 3));  // Function set: 4-bit mode, 2 lines, 5x8 dots
    delay_ms(5);

    LCD_SendCommand((1 << 3) | (1 << 2));  // Display control: display ON, cursor OFF, blink OFF
    delay_ms(5);

    LCD_SendCommand((1 << 0));             // Clear display
    delay_ms(5);

    LCD_SendCommand((1 << 2) | (1 << 1));  // Entry mode set: increment cursor, no shift
    delay_ms(5);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0 ? 0x00 : 0x00) + col;
    LCD_SendCommand(0x80 | addr);
}

void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

// Interrupt handler for PA0 button press
void EXTI0_IRQHandler(void) {
    // Check if the interrupt was triggered by PA0
    if (EXTI->PR & (1U << 0)) {
        // Clear the interrupt pending bit
        EXTI->PR |= (1U << 0);
			
				press_count++;
			
				if(press_count%2 == 1)
				{
					GPIOC->BSRR = (1U << 13);
				}
				else
				{
					GPIOC->BSRR = (1U << (13 + 16));
				}

    }
}

int main(void)
{
    // Enable clocks for ADC1, GPIOA, GPIOB, and AFIO
    RCC->APB2ENR |= (1U << 9);  // Enable ADC1 clock
    RCC->APB2ENR |= (1U << 2);  // Enable GPIOA clock
    RCC->APB2ENR |= (1U << 3);  // Enable GPIOB clock
    RCC->APB2ENR |= (1U << 0);  // Enable AFIO clock
		RCC->APB2ENR |= (1U << 4);  // Enable GPIOC clock

    // Initialize I2C and LCD
    I2C1_Init();
    LCD_Init();

		// Configure PC13 as output
    GPIOC->CRH &= ~(3U << 20);  // Clear MODE13 bits
    GPIOC->CRH |= (1U << 20);   // Set MODE13 to 01 (output mode, max speed 10 MHz)
    GPIOC->CRH &= ~(3U << 22);  // Set CNF13 to 00 (general-purpose output push-pull)

    // ADC configuration
    ADC1->CR2 |= (1U << 23);    // TSVREFE bit in CR2
    ADC1->CR2 |= (1U << 1);     // Enable continuous conversion mode (CONT bit in CR2)
    ADC1->SMPR1 |= (7U << 18);  // Set maximum sample time for channel 16 (239.5 cycles, bits 20:18 in SMPR1)
    ADC1->SQR1 &= ~(0xF << 20); // Conversion sequence length = 1 (bits 23:20 in SQR1)
    ADC1->SQR3 = 16;            // Select channel 16 for the first conversion (bits 4:0 in SQR3)
    ADC1->CR2 |= (1U << 0);     // Set ADON bit again to start the conversion
    ADC1->CR2 |= (1U << 22);    // Start conversion of regular channels (SWSTART bit in CR2)
    ADC1->CR2 |= (1U << 1);     // ADC continuous mode

    // Configure PA0 as input
    GPIOA->CRL &= ~(3U << 0);   // Clear mode bits for PA0
    GPIOA->CRL &= ~(3U << 2);   // Clear CNF bits for PA0
    GPIOA->CRL |= (1U << 2);    // Set PA0 as input

    // Configure EXTI for PA0
    AFIO->EXTICR[0] &= ~(0xF);   // Clear EXTI0 configuration
    AFIO->EXTICR[0] |= (0);      // Set EXTI0 to PA0
    EXTI->IMR |= (1U << 0);      // Unmask interrupt for EXTI0
    EXTI->FTSR |= (1U << 0);     // Trigger interrupt on falling edge

		LCD_SetCursor(0, 0);  // Set the cursor position on the LCD.
		LCD_Print("I2C LCD FOR TEMP"); // Print the ADC value on the LCD.
		delay_ms(500);

		GPIOC->BSRR = (1U << (13 + 16));
		
    NVIC_EnableIRQ(EXTI0_IRQn);  // Enable EXTI0 interrupt

    while (1) 
    {
        // ADC functionality
        if (ADC1->SR & (1U << 1)) 
        {
            adc_value = ADC1->DR;
						if (press_count%2 == 0)
						{
							if (adc_value < 2210)
							{
									GPIOC->BSRR = (1U << 13);  // Set PC13 high
							}
							else
							{
									GPIOC->BSRR = (1U << (13 + 16)); // Set PC13 low (reset)
							}
							// Update LCD with ADC value
							LCD_SetCursor(0, 0x40);          // Set cursor position
							char buffer[20];
							sprintf(buffer, "ADC Value: %d    ", adc_value);
							LCD_Print(buffer);            // Print ADC value on LCD
							delay_ms(500);                // Delay for readability
						}
					
						if (press_count%2 == 1)
						{
							// Update LCD with ADC value
							LCD_SetCursor(0, 0x40);          // Set cursor position
							LCD_Print("Fan is turned ON");            // Print on LCD
							delay_ms(500);                // Delay for readability
						}
        }
    }
}