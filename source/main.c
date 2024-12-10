/*******************************************************************************
* \file main.c
*
* Description: This code example demonstrates the communication between EZI2C
*              slave and I2C master. This is an integrated I2C master and EZI2C
*              slave code where in the master is configured to send command
*              packets to EZI2C slave to control a user LED on the board.
* 
* Related Document: See README.md
*
********************************************************************************
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

/*******************************************************************************
* Header Files
******************************************************************************/
#include "ezi2c_slave.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "i2c_master.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define OFF                 (1UL)
#define ON                  (0UL)
#define CMD_TO_CMD_DELAY    (1000UL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Debug UART variables */
static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t            DEBUG_UART_hal_obj;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function of the CE.
* It initializes the GPIO for LED, debug UART, EZI2C slave and the I2C mater and
* then prints the CE name through the UART terminal. In while loop, it sends the
* LED toggle commands to the EZI2C slave through I2C master and then reads its
* status. Also, it checks for the message received in the slave receive buffer
* and changes LED output based on the command received in the buffer.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint8_t cmd = ON;
    uint32_t status;
    uint8_t  buffer[WRITE_PACKET_SIZE] = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure retarget-io to use the debug UART port */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable UART */
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART setup failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget IO */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* retarget IO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("*************************************************\r\n");
    printf("           PDL:I2C Master EZI2C Slave            \r\n");
    printf("*************************************************\r\n\n");

    /* Initialize I2C Slave */
    printf(">> Initializing EZI2C Slave..... ");
    status = init_ezi2c_slave();
    if(status != I2C_SUCCESS)
    {
        CY_ASSERT(0);
    }
    printf("Done\r\n");

    /* Initialize I2C Master */
    printf(">> Initializing I2C master..... ");
    status = init_i2c_master();
    if(status == I2C_FAILURE)
    {
        CY_ASSERT(0);
    }
    printf("Done\r\n");

    /* Enable interrupts */
    __enable_irq();

    for(;;)
    {
        /* Create packet to be sent to the slave.  */
        buffer[MESG_ADDR_POS] = EZI2C_BUFFER_ADDRESS;
        buffer[MESG_SOP_POS] = MESG_SOP;
        buffer[MESG_EOP_POS] = MESG_EOP;
        buffer[MESG_CMD_POS] = cmd;

        /* Send packet with LED toggle command to the slave. */
        uint8_t res = write_packet_to_ezi2c(buffer, WRITE_PACKET_SIZE);

        if (TRANSFER_CMPLT == res)
        {
            printf("I2C: Write complete\r\n");

            /* Read response packet from the slave. */
            if (TRANSFER_CMPLT == read_packet_from_ezi2c())
            {
                printf("I2C: Read Complete\r\n");

                /* Next command to be written. */
                cmd = (cmd == ON) ? OFF : ON;
            }

            /* Check the EZI2C Slave buffer for the received command. If
             * the received packet is valid, change the status of LED
             * based on the command received.
             */
            check_ezi2c_buffer();

            /* Give 1 Second delay between commands. */
            Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
        }
        else
        {
            printf("Write failed with return val: %d\r\n", res);

            /* Give 1 Second delay between commands. */
            Cy_SysLib_Delay(CMD_TO_CMD_DELAY);
        }
    }
}
