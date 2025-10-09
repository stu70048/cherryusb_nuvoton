/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Template for M55M1 series MCU
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#define USE_USB_APLL1_CLOCK        1
#define RTE_USBD0 1
/* Private functions ---------------------------------------------------------*/
__WEAK void SetUsbdCLK(void)
{
#if (RTE_USBD0)
#if (USE_USB_APLL1_CLOCK)
    /* Select USB clock source as PLL/2 and USB clock divider as 2 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(2));
#else
    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif
    /* Enable OTG0 module clock */
    CLK_EnableModuleClock(OTG0_MODULE);

    /* Select USB role to USBD and Enable PHY */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD0_MODULE);
#endif
#if (RTE_USBD1)
    /* Enable External RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT */
#if (__HXT == 19200000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_19_2M);
#elif (__HXT == 20000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_20_0M);
#elif (__HXT == 24000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);
#elif (__HXT == 16000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_16_0M);
#elif (__HXT == 26000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_26_0M);
#elif (__HXT == 32000000UL)
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_32_0M);
#else
#warning This HXT cannot make HSUSB work properly.
#endif

    /* Set HSUSB role to HSUSBD */
    SET_HSUSBDROLE();

    /* Enable HSUSB PHY */
    SYS_Enable_HSUSB_PHY();

    /* Enable HSUSBD peripheral clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);
#endif
};

__WEAK void SetUsbdMFP(void)
{
#if (RTE_USBD0)
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();
#endif
#if (RTE_USBD1)
    /* fixed-function pins */
#endif
};

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
    
    /* Enable Usbd driver module clock */
    SetUsbdCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    
    /* Set USBD driver module MFP */
    SetUsbdMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("Hello World\n");

    extern void cdc_acm_msc_init(uint8_t busid, uint32_t reg_base);

    cdc_acm_msc_init(0, (uint32_t)USBD);
    /* Got no where to go, just loop forever */
    while (1) {
    extern void cdc_acm_data_send_with_dtr_test(uint8_t busid);
      cdc_acm_data_send_with_dtr_test(0);
    };
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
