/**
 * @file   UART_soc.c
 *
 * @brief  This file defines the UART interface structure specific to AM571x
 */
/*
 * Copyright (c) 2014 - 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================*/

#include <ti/csl/csl_utils.h>
#include <ti/csl/soc/am571x/src/cslr_soc.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/soc/UART_soc.h>
#ifdef UART_DMA_ENABLE
#include <ti/sdo/edma3/drv/edma3_drv.h>
#endif

/*This configuration needs to be chnaged for AM572x-GP-EVM.Right now it is configured for DRA75X*/

/* UART configuration structure */
UART_HwAttrs uartInitCfg[CSL_UART_PER_CNT] =
{ {
#ifdef _TMS320C6X
        CSL_DSP_UART1_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER, /* Use an interrupt vector # other than one used by EDMA */
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART1_REGS,
        104,
#else
        CSL_IPU_UART1_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_0_MODULE_FREQ,
        CSL_EDMA3_CHA_UART0_RX,
        CSL_EDMA3_CHA_UART0_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART2_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART2_REGS,
        105,
#else
        CSL_IPU_UART2_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_1_MODULE_FREQ,
        CSL_EDMA3_CHA_UART1_RX,
        CSL_EDMA3_CHA_UART1_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART3_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART3_REGS,
        106,
#else
        CSL_IPU_UART3_REGS,
        45,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_2_MODULE_FREQ,
        CSL_EDMA3_CHA_UART2_RX,
        CSL_EDMA3_CHA_UART2_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART4_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART4_REGS,
        102,
#else
        CSL_IPU_UART4_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_3_MODULE_FREQ,
        CSL_EDMA3_CHA_UART3_RX,
        CSL_EDMA3_CHA_UART3_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART5_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART5_REGS,
        137,
#else
        CSL_IPU_UART5_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_4_MODULE_FREQ,
        CSL_EDMA3_CHA_UART4_RX,
        CSL_EDMA3_CHA_UART4_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART6_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART6_REGS,
        138,
#else
        CSL_IPU_UART6_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_5_MODULE_FREQ,
        CSL_EDMA3_CHA_UART5_RX,
        CSL_EDMA3_CHA_UART5_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART7_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART7_REGS,
        104,
#else
        CSL_IPU_UART7_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_6_MODULE_FREQ,
        CSL_EDMA3_CHA_UART6_RX,
        CSL_EDMA3_CHA_UART6_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART8_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART8_REGS,
        104,
#else
        CSL_IPU_UART8_REGS,
        24,
#endif
        CSL_INTC_EVENTID_UARTINT7,
        CSL_UART_7_MODULE_FREQ,
        CSL_EDMA3_CHA_UART7_RX,
        CSL_EDMA3_CHA_UART7_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART9_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART9_REGS,
        104,
#else
        CSL_IPU_UART9_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_8_MODULE_FREQ,
        CSL_EDMA3_CHA_UART8_RX,
        CSL_EDMA3_CHA_UART8_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
    {
#ifdef _TMS320C6X
        CSL_DSP_UART10_REGS,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
#elif defined(__ARM_ARCH_7A__)
        CSL_MPU_UART10_REGS,
        104,
#else
        CSL_IPU_UART10_REGS,
        24,
#endif
        34,  /* used only for C6x, reserved DSP1_IRQ_34 */
        CSL_UART_9_MODULE_FREQ,
        CSL_EDMA3_CHA_UART9_RX,
        CSL_EDMA3_CHA_UART9_TX,
        0,
        0,
        0,
        0,
        0,
        NULL,
        UART_RXTRIGLVL_8,
        UART_TXTRIGLVL_56,
        TRUE, /* default DMA mode */
        FALSE, /* Loopback disabled by default */
		TRUE, /* Interrupt enabled by default */
    },
};

/* UART objects */
UART_V1_Object UartObjects[CSL_UART_PER_CNT];

/* UART configuration structure */
CSL_PUBLIC_CONST UART_Config UART_config[CSL_UART_PER_CNT + 1U] = {
    {
        &UART_FxnTable_v1,
        &UartObjects[0],
        &uartInitCfg[0]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[1],
        &uartInitCfg[1]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[2],
        &uartInitCfg[2]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[3],
        &uartInitCfg[3]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[4],
        &uartInitCfg[4]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[5],
        &uartInitCfg[5]
    },
    {
        &UART_FxnTable_v1,
        &UartObjects[6],
        &uartInitCfg[6]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[7],
        &uartInitCfg[7]
    },

    {
        &UART_FxnTable_v1,
        &UartObjects[8],
        &uartInitCfg[8]
    },
    {
        &UART_FxnTable_v1,
        &UartObjects[9],
        &uartInitCfg[9]
    },
    {NULL, NULL, NULL}
};

/**
 * \brief  This API gets the SoC level of UART intial configuration
 *
 * \param  index     UART instance index.
 * \param  cfg       Pointer to UART SOC initial config.
 *
 * \return 0 success: -1: error
 *
 */
int32_t UART_socGetInitCfg(uint32_t index, UART_HwAttrs *cfg)
{
    int32_t ret = 0;

    if (index < CSL_UART_PER_CNT)
    {
        *cfg = uartInitCfg[index];
    }
    else
    {
        ret = (int32_t)(-1);
    }

    return ret;
}

/**
 * \brief  This API sets the SoC level of UART intial configuration
 *
 * \param  index     UART instance index.
 * \param  cfg       Pointer to UART SOC initial config.
 *
 * \return           0 success: -1: error
 *
 */
int32_t UART_socSetInitCfg(uint32_t index, const UART_HwAttrs *cfg)
{
    int32_t ret = 0;

    if (index < CSL_UART_PER_CNT)
    {
        uartInitCfg[index] = *cfg;
    }
    else
    {
        ret = (int32_t)(-1);
    }

    return ret;
}

