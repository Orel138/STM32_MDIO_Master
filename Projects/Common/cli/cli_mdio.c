/*
 * Copyright (c) 2026 Orel138
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Project Specific */
#include "cli.h"
#include "cli_prv.h"
#include "logging.h"

/* Standard Lib */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

/* Argument indexes */
#define VERB_ARG_INDEX       1
#define PHY_ADDR_INDEX       2
#define PHY_REG_INDEX        3
#define PHY_VALUE_INDEX      4

/* Local function prototype */
static void vCommand_MDIO( ConsoleIO_t * pxCIO,
                           uint32_t ulArgc,
                           char * ppcArgv[] );

/* Helper: convert 16-bit value to binary string (16 chars + nul) */
static void uint16_to_binary(uint16_t value, char *buf)
{
    for (int i = 15; i >= 0; --i)
    {
        buf[15 - i] = (value & (1u << i)) ? '1' : '0';
    }
    buf[16] = '\0';
}

/* CLI command definition */
const CLI_Command_Definition_t xCommandDef_mdio =
{
    .pcCommand            = "mdio",
    .pcHelpString         =
        "mdio:\r\n"
        "    Perform MDIO read/write operations.\r\n"
        "    Usage:\r\n"
        "    mdio write <PHYAddress> <PHY_Reg> <Reg_value>\r\n"
        "    mdio read  <PHYAddress> <PHY_Reg>\r\n",
    .pxCommandInterpreter = vCommand_MDIO
};

#define VERB_ARG_INDEX       1
#define OBJECT_TYPE_INDEX    2

static void vCommand_MDIO( ConsoleIO_t * pxCIO,
                           uint32_t ulArgc,
                           char * ppcArgv[] )
{
    const char * pcVerb = NULL;
    BaseType_t xSuccess = pdFALSE;

    if( ulArgc > VERB_ARG_INDEX )
    {
        pcVerb = ppcArgv[ VERB_ARG_INDEX ];

        /* -------------------------------------------------- */
        /* MDIO WRITE                                         */
        /* -------------------------------------------------- */
        if( 0 == strcmp( pcVerb, "write" ) )
        {
            if( ulArgc > PHY_VALUE_INDEX )
            {
                uint8_t  phyAddr  = (uint8_t)  strtoul( ppcArgv[ PHY_ADDR_INDEX ], NULL, 0 );
                uint8_t  regAddr  = (uint8_t)  strtoul( ppcArgv[ PHY_REG_INDEX  ], NULL, 0 );
                uint16_t regValue = (uint16_t) strtoul( ppcArgv[ PHY_VALUE_INDEX ], NULL, 0 );

                if( HAL_ETH_WritePHYRegister(&heth, phyAddr, regAddr, regValue) == HAL_OK)
                {
                    pxCIO->print( "MDIO WRITE: OK\r\n" );
                    xSuccess = pdTRUE;
                }
                else
                {
                    pxCIO->print( "MDIO WRITE: ERROR\r\n" );
                }
            }
        }
        /* -------------------------------------------------- */
        /* MDIO READ                                          */
        /* -------------------------------------------------- */
        else if( 0 == strcmp( pcVerb, "read" ) )
        {
            if( ulArgc > PHY_REG_INDEX )
            {
                uint8_t  phyAddr = (uint8_t) strtoul( ppcArgv[ PHY_ADDR_INDEX ], NULL, 0 );
                uint8_t  regAddr = (uint8_t) strtoul( ppcArgv[ PHY_REG_INDEX  ], NULL, 0 );
                uint16_t regValue = 0;

                if( HAL_ETH_ReadPHYRegister(&heth, phyAddr, regAddr, &regValue) == HAL_OK)
                {
                    char buffer[48];
                    char bin[17];

                    uint16_to_binary(regValue, bin);

                    snprintf( buffer, sizeof(buffer),
                              "MDIO READ: OK 0x%04X 0b%s\r\n", regValue, bin );

                    pxCIO->print( buffer );
                    xSuccess = pdTRUE;
                }
                else
                {
                    pxCIO->print( "MDIO READ: ERROR\r\n" );
                }
            }
        }
    }

    /* Print help if command failed */
    if( xSuccess == pdFALSE )
    {
        pxCIO->print( xCommandDef_mdio.pcHelpString );
    }
}
