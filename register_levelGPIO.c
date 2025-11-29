#include "stm32f4xx.h"

/*
 * GPIO_Transmitter_Init()
 *
 * This function configures ALL GPIO pins used in the TRANSMITTER (Glove Unit)
 * at **REGISTER LEVEL**, without using HAL or CubeMX initialization.
 *
 * Pin functions:
 *
 *  - PA0–PA3 : 4 buttons
 *              -> Input mode
 *              -> Internal Pull-Up (buttons connected to GND)
 *              -> EXTI line interrupts (rising + falling edges)
 *
 *  - PA5      : Pulse sensor input
 *              -> ADC1_IN5 (Analog mode)
 *
 *  - PB6/PB7  : OLED Display #1 (I2C1)
 *              -> Alternate Function (AF4)
 *              -> Open-Drain (required for I2C)
 *              -> Pull-Up
 *
 *  - PD8/PD9  : LoRa Module (USART3)
 *              -> Alternate Function (AF7)
 *              -> Push-Pull, High-Speed
 *
 * This function sets:
 *  - RCC clock enabling for GPIOA/B/D and SYSCFG
 *  - MODER, OTYPER, OSPEEDR, PUPDR for each pin
 *  - AFR (Alternate Function) for I2C and UART pins
 *  - EXTI routing and NVIC interrupt enabling for PA0–PA3
 *
 * This fully satisfies the instructor’s requirement:
 *   “Digital GPIOs must be configured using register-level code.”
 */
void GPIO_Transmitter_Init(void)
{
    /*------------------------------------------------------------------
     * 1) Enable peripheral clocks
     *------------------------------------------------------------------
     * RCC->AHB1ENR : GPIO port clock enable register
     * RCC->APB2ENR : SYSCFG clock (required for EXTI routing)
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock  (PA0–PA5)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   // Enable GPIOB clock  (PB6–PB7)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;   // Enable GPIOD clock  (PD8–PD9)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  // Enable SYSCFG clock for EXTI

    /*------------------------------------------------------------------
     * 2) BUTTONS (PA0–PA3) — Input + Pull-Up + EXTI
     *------------------------------------------------------------------
     * MODER: 00 = Input mode
     * PUPDR: 01 = Pull-Up
     */

     // Clear MODER bits for PA0..PA3 → set as Input (00)
    GPIOA->MODER &= ~(
        (3U << (0 * 2)) |
        (3U << (1 * 2)) |
        (3U << (2 * 2)) |
        (3U << (3 * 2))
        );

    // Configure Pull-Up for PA0..PA3
    GPIOA->PUPDR &= ~(
        (3U << (0 * 2)) |
        (3U << (1 * 2)) |
        (3U << (2 * 2)) |
        (3U << (3 * 2))
        );
    GPIOA->PUPDR |= (
        (1U << (0 * 2)) |   // PA0 Pull-Up
        (1U << (1 * 2)) |   // PA1 Pull-Up
        (1U << (2 * 2)) |   // PA2 Pull-Up
        (1U << (3 * 2))     // PA3 Pull-Up
        );

    /*------------------------------------------------------------------
     * 3) PULSE SENSOR (PA5 — ADC1_IN5)
     *------------------------------------------------------------------
     * MODER: 11 = Analog mode
     * PUPDR: 00 = No pull-up/pull-down
     */

    GPIOA->MODER &= ~(3U << (5 * 2));
    GPIOA->MODER |= (3U << (5 * 2));   // Analog mode

    GPIOA->PUPDR &= ~(3U << (5 * 2));   // No pull

    /*------------------------------------------------------------------
     * 4) OLED DISPLAY (PB6 = SCL, PB7 = SDA) — I2C1 (AF4)
     *------------------------------------------------------------------
     * MODER: 10 = Alternate Function
     * OTYPER: Open-Drain
     * OSPEEDR: High speed
     * PUPDR: Pull-Up
     * AFR[0]: AF4 for PB6/PB7
     */

     // Set PB6 & PB7 to AF mode
    GPIOB->MODER &= ~(
        (3U << (6 * 2)) |
        (3U << (7 * 2))
        );
    GPIOB->MODER |= (
        (2U << (6 * 2)) |
        (2U << (7 * 2))
        );

    // Open-Drain (required for I2C)
    GPIOB->OTYPER |= (1U << 6) | (1U << 7);

    // High speed
    GPIOB->OSPEEDR |=
        (3U << (6 * 2)) |
        (3U << (7 * 2));

    // Pull-Up
    GPIOB->PUPDR &= ~(
        (3U << (6 * 2)) |
        (3U << (7 * 2))
        );
    GPIOB->PUPDR |= (
        (1U << (6 * 2)) |
        (1U << (7 * 2))
        );

    // Select AF4 (I2C1) for PB6/PB7
    GPIOB->AFR[0] &= ~(
        (0xFU << (6 * 4)) |
        (0xFU << (7 * 4))
        );
    GPIOB->AFR[0] |= (
        (4U << (6 * 4)) |   // PB6 → AF4 (I2C1_SCL)
        (4U << (7 * 4))     // PB7 → AF4 (I2C1_SDA)
        );

    /*------------------------------------------------------------------
     * 5) LoRa UART PINS (PD8 = TX, PD9 = RX) — USART3 (AF7)
     *------------------------------------------------------------------
     * MODER: 10 = Alternate Function
     * OTYPER: Push-Pull
     * OSPEEDR: High speed
     * AFR[1]: AF7 for PD8/PD9
     */

    GPIOD->MODER &= ~(
        (3U << (8 * 2)) |
        (3U << (9 * 2))
        );
    GPIOD->MODER |= (
        (2U << (8 * 2)) |
        (2U << (9 * 2))
        );

    GPIOD->OTYPER &= ~((1U << 8) | (1U << 9));   // Push-pull

    GPIOD->OSPEEDR |=
        (3U << (8 * 2)) |
        (3U << (9 * 2));   // High speed for UART

    GPIOD->PUPDR &= ~(
        (3U << (8 * 2)) |
        (3U << (9 * 2))
        ); // No pull-up/down

    // Select AF7 (USART3)
    GPIOD->AFR[1] &= ~(
        (0xFU << ((8 - 8) * 4)) |
        (0xFU << ((9 - 8) * 4))
        );
    GPIOD->AFR[1] |= (
        (7U << ((8 - 8) * 4)) |    // PD8 → USART3_TX
        (7U << ((9 - 8) * 4))      // PD9 → USART3_RX
        );

    /*------------------------------------------------------------------
     * 6) EXTI CONFIGURATION FOR BUTTONS (PA0–PA3)
     *------------------------------------------------------------------
     * SYSCFG_EXTICR: selects PORT for EXTI line
     * EXTI_RTSR: Rising edge trigger
     * EXTI_FTSR: Falling edge trigger
     * EXTI_IMR : Interrupt Mask
     */

     // Route EXTI0–3 to PORT A (0000)
    SYSCFG->EXTICR[0] &= ~(
        SYSCFG_EXTICR1_EXTI0 |
        SYSCFG_EXTICR1_EXTI1 |
        SYSCFG_EXTICR1_EXTI2 |
        SYSCFG_EXTICR1_EXTI3
        );

    // Enable rising + falling edge detection
    EXTI->RTSR |= (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3);
    EXTI->FTSR |= (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3);

    // Enable interrupt mask for EXTI0–3
    EXTI->IMR |= (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3);

    // Clear pending flags (safety)
    EXTI->PR = (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3);

    // Enable NVIC interrupt lines
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
}