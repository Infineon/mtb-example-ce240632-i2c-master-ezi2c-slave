/******************************************************************************
* File Name:   i2c_master.h
*
* Description: This file contains the I2C master function interface and MACROS
* defined for I2C master.
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

#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define I2C_SUCCESS             (0UL)
#define I2C_FAILURE             (1UL)

#define TRANSFER_CMPLT          (0x00UL)
#define READ_CMPLT              (TRANSFER_CMPLT)

/* I2C master send packet index */
#define MESG_ADDR_POS         (0UL)
#define MESG_SOP_POS          (1UL)
#define MESG_CMD_POS          (2UL)
#define MESG_EOP_POS          (3UL)

/* I2C master send/receive packet start and end markers */
#define MESG_SOP              (0x01UL)
#define MESG_EOP              (0x17UL)

#define WRITE_PACKET_SIZE       (0x04UL)

/* Start address of slave buffer */
#define EZI2C_BUFFER_ADDRESS    (0x00)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
uint8_t write_packet_to_ezi2c(uint8_t* write_buffer, uint32_t buffer_size);
uint8_t read_packet_from_ezi2c(void);
uint32_t init_i2c_master(void);

#endif /* I2C_MASTER_H_ */
