/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code puts the microcontroller to
*              hibernate mode and wakeups on input button press.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
#define DELAY_LONG_MS           (500)   /* milliseconds */
#define LED_BLINK_COUNT         (8)     /* LED toggle count */
#define GPIO_INTERRUPT_PRIORITY (1u)    /* GPIO Interrupt priority */


/*******************************************************************************
* Global Variables
********************************************************************************/
cy_stc_sysint_t button_press_intr_config =
{
.intrSrc = CYBSP_USER_BTN2_IRQ,
.intrPriority = GPIO_INTERRUPT_PRIORITY,
};

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void gpio_interrupt_handler(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It configures and initializes the GPIO
*  interrupt, blinks the LED and enter in hibernate mode. On user button
*  press it wakesup.
*  Hibernate wakeup source used in this example is P2.0 (User Button 2)
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t count = 0;
    uint32_t delay_led_blink = DELAY_LONG_MS;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Check the IO status. If current status is frozen, unfreeze the system. */
    if (Cy_SysPm_GetIoFreezeStatus())
    {
        /* Unfreeze the system */
        Cy_SysPm_IoUnfreeze();
    }

    /*Initializing the interrupt for button action */
    Cy_SysInt_Init(&button_press_intr_config, gpio_interrupt_handler);
    NVIC_EnableIRQ(button_press_intr_config.intrSrc);

    /* Enable global interrupts */
    __enable_irq();

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "PDL : Hibernate GPIO Wakeup "
           "****************** \r\n\n");

    /* Check the reset reason */
    if(CY_SYSLIB_RESET_HIB_WAKEUP == (Cy_SysLib_GetResetReason() &
    CY_SYSLIB_RESET_HIB_WAKEUP))
    {
        /* Wait a bit to avoid glitches from the button press */
        Cy_SysLib_Delay(DELAY_LONG_MS);
        /* The reset has occurred on a wakeup from Hibernate power mode */
        printf("Wake up from the Hibernate mode.\r\n");
    }

    for (;;)
    {
        /* Invert the USER LED state */
        for (count = 0; count < LED_BLINK_COUNT; count++)
        {
            Cy_GPIO_Inv (CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
            Cy_SysLib_Delay(delay_led_blink);
        }

        printf("Entering hibernate mode. Press the user button to wakeup.\r\n");
        Cy_SysLib_Delay(DELAY_LONG_MS);

        /* Go to hibernate and Configure a low logic level for the second wakeup-pin */
        /* Pin P2.0 (User Button 2) used as a wakeup pin */
        Cy_SysPm_SetHibernateWakeupSource(CY_SYSPM_HIBERNATE_PIN0_LOW);

        /* Jump to Hibernate Mode */
        Cy_SysPm_SystemEnterHibernate();
    }
}


/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void gpio_interrupt_handler(void)
{
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM);
}


/* [] END OF FILE */
