//=============================================================================
// Name        : mhai_stm32f411xe.hpp
// Author      : Julien Combattelli
// EMail       : julien.combattelli@gmail.com
// Date        : Jan 26, 2019
// Version     : 1.0.0
// Copyright   : This file is part of the Modern Hardware Abstraction Interface
//               (MHAI) project which is released under MIT license.
//               See file LICENSE.txt for full license details.
//=============================================================================

#ifndef MHAI_DEVICE_MHAI_STM32F411XE_HPP_
#define MHAI_DEVICE_MHAI_STM32F411XE_HPP_

#include "mhai_cortexm4.hpp"

namespace mhai
{
namespace cortexm4
{
enum class IRQn
{
    // Cortex-M4 Processor Exceptions Numbers
    NonMaskableInt = -14,   // 2 Non Maskable Interrupt
    MemoryManagement = -12, // 4 Cortex-M4 Memory Management Interrupt
    BusFault = -11,         // 5 Cortex-M4 Bus Fault Interrupt
    UsageFault = -10,       // 6 Cortex-M4 Usage Fault Interrupt
    SVCall = -5,            // 11 Cortex-M4 SV Call Interrupt
    DebugMonitor = -4,      // 12 Cortex-M4 Debug Monitor Interrupt
    PendSV = -2,            // 14 Cortex-M4 Pend SV Interrupt
    SysTick = -1,           // 15 Cortex-M4 System Tick Interrupt
    //  STM32 specific Interrupt Numbers
    WWDG = 0,                // Window WatchDog Interrupt
    PVD = 1,                 // PVD through EXTI Line detection Interrupt
    TAMP_STAMP = 2,          // Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP = 3,            // RTC Wakeup interrupt through the EXTI line
    FLASH = 4,               // FLASH global Interrupt
    RCC = 5,                 // RCC global Interrupt
    EXTI0 = 6,               // EXTI Line0 Interrupt
    EXTI1 = 7,               // EXTI Line1 Interrupt
    EXTI2 = 8,               // EXTI Line2 Interrupt
    EXTI3 = 9,               // EXTI Line3 Interrupt
    EXTI4 = 10,              // EXTI Line4 Interrupt
    DMA1_Stream0 = 11,       // DMA1 Stream 0 global Interrupt
    DMA1_Stream1 = 12,       // DMA1 Stream 1 global Interrupt
    DMA1_Stream2 = 13,       // DMA1 Stream 2 global Interrupt
    DMA1_Stream3 = 14,       // DMA1 Stream 3 global Interrupt
    DMA1_Stream4 = 15,       // DMA1 Stream 4 global Interrupt
    DMA1_Stream5 = 16,       // DMA1 Stream 5 global Interrupt
    DMA1_Stream6 = 17,       // DMA1 Stream 6 global Interrupt
    ADC = 18,                // ADC1, ADC2 and ADC3 global Interrupts
    EXTI9_5 = 23,            // External Line[9:5] Interrupts
    TIM1_BRK_TIM9 = 24,      // TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_TIM10 = 25,      // TIM1 Update Interrupt and TIM10 global interrupt
    TIM1_TRG_COM_TIM11 = 26, // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC = 27,            // TIM1 Capture Compare Interrupt
    TIM2 = 28,               // TIM2 global Interrupt
    TIM3 = 29,               // TIM3 global Interrupt
    TIM4 = 30,               // TIM4 global Interrupt
    I2C1_EV = 31,            // I2C1 Event Interrupt
    I2C1_ER = 32,            // I2C1 Error Interrupt
    I2C2_EV = 33,            // I2C2 Event Interrupt
    I2C2_ER = 34,            // I2C2 Error Interrupt
    SPI1 = 35,               // SPI1 global Interrupt
    SPI2 = 36,               // SPI2 global Interrupt
    USART1 = 37,             // USART1 global Interrupt
    USART2 = 38,             // USART2 global Interrupt
    EXTI15_10 = 40,          // External Line[15:10] Interrupts
    RTC_Alarm = 41,          // RTC Alarm (A and B) through EXTI Line Interrupt
    OTG_FS_WKUP = 42,        // USB OTG FS Wakeup through EXTI line interrupt
    DMA1_Stream7 = 47,       // DMA1 Stream7 Interrupt
    SDIO = 49,               // SDIO global Interrupt
    TIM5 = 50,               // TIM5 global Interrupt
    SPI3 = 51,               // SPI3 global Interrupt
    DMA2_Stream0 = 56,       // DMA2 Stream 0 global Interrupt
    DMA2_Stream1 = 57,       // DMA2 Stream 1 global Interrupt
    DMA2_Stream2 = 58,       // DMA2 Stream 2 global Interrupt
    DMA2_Stream3 = 59,       // DMA2 Stream 3 global Interrupt
    DMA2_Stream4 = 60,       // DMA2 Stream 4 global Interrupt
    OTG_FS = 67,             // USB OTG FS global Interrupt
    DMA2_Stream5 = 68,       // DMA2 Stream 5 global interrupt
    DMA2_Stream6 = 69,       // DMA2 Stream 6 global interrupt
    DMA2_Stream7 = 70,       // DMA2 Stream 7 global interrupt
    USART6 = 71,             // USART6 global interrupt
    I2C3_EV = 72,            // I2C3 event interrupt
    I2C3_ER = 73,            // I2C3 error interrupt
    FPU = 81,                // FPU global interrupt
    SPI4 = 84,               // SPI4 global Interrupt
    SPI5 = 85                // SPI5 global Interrupt
};

} // namespace cortexm4

namespace stm32f411xe
{
///////////////////////////////////////////////////////////////////////////////
// Base addresses for memory mapping
///////////////////////////////////////////////////////////////////////////////

constexpr std::uint32_t FLASH_BASE = 0x08000000U;      // FLASH(up to 1 MB) base address in the alias region
constexpr std::uint32_t SRAM1_BASE = 0x20000000U;      // SRAM1(128 KB) base address in the alias region
constexpr std::uint32_t PERIPH_BASE = 0x40000000U;     // Peripheral base address in the alias region
constexpr std::uint32_t SRAM1_BB_BASE = 0x22000000U;   // SRAM1(128 KB) base address in the bit-band region
constexpr std::uint32_t PERIPH_BB_BASE = 0x42000000U;  // Peripheral base address in the bit-band region
constexpr std::uint32_t BKPSRAM_BB_BASE = 0x42480000U; // Backup SRAM(4 KB) base address in the bit-band region
constexpr std::uint32_t FLASH_END = 0x0807FFFFU;       // FLASH end address
constexpr std::uint32_t FLASH_OTP_BASE = 0x1FFF7800U;  // Base address of : (up to 528 Bytes) embedded FLASH OTP Area
constexpr std::uint32_t FLASH_OTP_END = 0x1FFF7A0FU;   // End address of : (up to 528 Bytes) embedded FLASH OTP Area

// Peripheral memory map
constexpr std::uint32_t APB1PERIPH_BASE = PERIPH_BASE;
constexpr std::uint32_t APB2PERIPH_BASE = PERIPH_BASE + 0x00010000U;
constexpr std::uint32_t AHB1PERIPH_BASE = PERIPH_BASE + 0x00020000U;
constexpr std::uint32_t AHB2PERIPH_BASE = PERIPH_BASE + 0x10000000U;

// APB1 peripherals
constexpr std::uint32_t TIM2_BASE = APB1PERIPH_BASE + 0x0000U;
constexpr std::uint32_t TIM3_BASE = APB1PERIPH_BASE + 0x0400U;
constexpr std::uint32_t TIM4_BASE = APB1PERIPH_BASE + 0x0800U;
constexpr std::uint32_t TIM5_BASE = APB1PERIPH_BASE + 0x0C00U;
constexpr std::uint32_t RTC_BASE = APB1PERIPH_BASE + 0x2800U;
constexpr std::uint32_t WWDG_BASE = APB1PERIPH_BASE + 0x2C00U;
constexpr std::uint32_t IWDG_BASE = APB1PERIPH_BASE + 0x3000U;
constexpr std::uint32_t I2S2ext_BASE = APB1PERIPH_BASE + 0x3400U;
constexpr std::uint32_t SPI2_BASE = APB1PERIPH_BASE + 0x3800U;
constexpr std::uint32_t SPI3_BASE = APB1PERIPH_BASE + 0x3C00U;
constexpr std::uint32_t I2S3ext_BASE = APB1PERIPH_BASE + 0x4000U;
constexpr std::uint32_t USART2_BASE = APB1PERIPH_BASE + 0x4400U;
constexpr std::uint32_t I2C1_BASE = APB1PERIPH_BASE + 0x5400U;
constexpr std::uint32_t I2C2_BASE = APB1PERIPH_BASE + 0x5800U;
constexpr std::uint32_t I2C3_BASE = APB1PERIPH_BASE + 0x5C00U;
constexpr std::uint32_t PWR_BASE = APB1PERIPH_BASE + 0x7000U;

// APB2 peripherals
constexpr std::uint32_t TIM1_BASE = APB2PERIPH_BASE + 0x0000U;
constexpr std::uint32_t USART1_BASE = APB2PERIPH_BASE + 0x1000U;
constexpr std::uint32_t USART6_BASE = APB2PERIPH_BASE + 0x1400U;
constexpr std::uint32_t ADC1_BASE = APB2PERIPH_BASE + 0x2000U;
constexpr std::uint32_t ADC1_COMMON_BASE = APB2PERIPH_BASE + 0x2300U;
constexpr std::uint32_t SDIO_BASE = APB2PERIPH_BASE + 0x2C00U;
constexpr std::uint32_t SPI1_BASE = APB2PERIPH_BASE + 0x3000U;
constexpr std::uint32_t SPI4_BASE = APB2PERIPH_BASE + 0x3400U;
constexpr std::uint32_t SYSCFG_BASE = APB2PERIPH_BASE + 0x3800U;
constexpr std::uint32_t EXTI_BASE = APB2PERIPH_BASE + 0x3C00U;
constexpr std::uint32_t TIM9_BASE = APB2PERIPH_BASE + 0x4000U;
constexpr std::uint32_t TIM10_BASE = APB2PERIPH_BASE + 0x4400U;
constexpr std::uint32_t TIM11_BASE = APB2PERIPH_BASE + 0x4800U;
constexpr std::uint32_t SPI5_BASE = APB2PERIPH_BASE + 0x5000U;

// AHB1 peripherals
constexpr std::uint32_t GPIOA_BASE = AHB1PERIPH_BASE + 0x0000U;
constexpr std::uint32_t GPIOB_BASE = AHB1PERIPH_BASE + 0x0400U;
constexpr std::uint32_t GPIOC_BASE = AHB1PERIPH_BASE + 0x0800U;
constexpr std::uint32_t GPIOD_BASE = AHB1PERIPH_BASE + 0x0C00U;
constexpr std::uint32_t GPIOE_BASE = AHB1PERIPH_BASE + 0x1000U;
constexpr std::uint32_t GPIOH_BASE = AHB1PERIPH_BASE + 0x1C00U;
constexpr std::uint32_t CRC_BASE = AHB1PERIPH_BASE + 0x3000U;
constexpr std::uint32_t RCC_BASE = AHB1PERIPH_BASE + 0x3800U;
constexpr std::uint32_t FLASH_R_BASE = AHB1PERIPH_BASE + 0x3C00U;
constexpr std::uint32_t DMA1_BASE = AHB1PERIPH_BASE + 0x6000U;
constexpr std::uint32_t DMA1_Stream0_BASE = DMA1_BASE + 0x010U;
constexpr std::uint32_t DMA1_Stream1_BASE = DMA1_BASE + 0x028U;
constexpr std::uint32_t DMA1_Stream2_BASE = DMA1_BASE + 0x040U;
constexpr std::uint32_t DMA1_Stream3_BASE = DMA1_BASE + 0x058U;
constexpr std::uint32_t DMA1_Stream4_BASE = DMA1_BASE + 0x070U;
constexpr std::uint32_t DMA1_Stream5_BASE = DMA1_BASE + 0x088U;
constexpr std::uint32_t DMA1_Stream6_BASE = DMA1_BASE + 0x0A0U;
constexpr std::uint32_t DMA1_Stream7_BASE = DMA1_BASE + 0x0B8U;
constexpr std::uint32_t DMA2_BASE = AHB1PERIPH_BASE + 0x6400U;
constexpr std::uint32_t DMA2_Stream0_BASE = DMA2_BASE + 0x010U;
constexpr std::uint32_t DMA2_Stream1_BASE = DMA2_BASE + 0x028U;
constexpr std::uint32_t DMA2_Stream2_BASE = DMA2_BASE + 0x040U;
constexpr std::uint32_t DMA2_Stream3_BASE = DMA2_BASE + 0x058U;
constexpr std::uint32_t DMA2_Stream4_BASE = DMA2_BASE + 0x070U;
constexpr std::uint32_t DMA2_Stream5_BASE = DMA2_BASE + 0x088U;
constexpr std::uint32_t DMA2_Stream6_BASE = DMA2_BASE + 0x0A0U;
constexpr std::uint32_t DMA2_Stream7_BASE = DMA2_BASE + 0x0B8U;

// Debug MCU registers base address
constexpr std::uint32_t DBGMCU_BASE = 0xE0042000U;
// USB registers base address
constexpr std::uint32_t USB_OTG_FS_PERIPH_BASE = 0x50000000U;

constexpr std::uint32_t USB_OTG_GLOBAL_BASE = 0x000U;
constexpr std::uint32_t USB_OTG_DEVICE_BASE = 0x800U;
constexpr std::uint32_t USB_OTG_IN_ENDPOINT_BASE = 0x900U;
constexpr std::uint32_t USB_OTG_OUT_ENDPOINT_BASE = 0xB00U;
constexpr std::uint32_t USB_OTG_EP_REG_SIZE = 0x20U;
constexpr std::uint32_t USB_OTG_HOST_BASE = 0x400U;
constexpr std::uint32_t USB_OTG_HOST_PORT_BASE = 0x440U;
constexpr std::uint32_t USB_OTG_HOST_CHANNEL_BASE = 0x500U;
constexpr std::uint32_t USB_OTG_HOST_CHANNEL_SIZE = 0x20U;
constexpr std::uint32_t USB_OTG_PCGCCTL_BASE = 0xE00U;
constexpr std::uint32_t USB_OTG_FIFO_BASE = 0x1000U;
constexpr std::uint32_t USB_OTG_FIFO_SIZE = 0x1000U;

constexpr std::uint32_t UID_BASE = 0x1FFF7A10U;       // Unique device ID register base address
constexpr std::uint32_t FLASHSIZE_BASE = 0x1FFF7A22U; // FLASH Size register base address
constexpr std::uint32_t PACKAGE_BASE = 0x1FFF7BF0U;   // Package size register base address

///////////////////////////////////////////////////////////////////////////////
// Peripherals registers offset
///////////////////////////////////////////////////////////////////////////////

// ADC registers offset
constexpr std::uint32_t ADC_SR_offset = 0x00;
constexpr std::uint32_t ADC_CR1_offset = 0x04;
constexpr std::uint32_t ADC_CR2_offset = 0x08;
constexpr std::uint32_t ADC_SMPR1_offset = 0x0C;
constexpr std::uint32_t ADC_SMPR2_offset = 0x10;
constexpr std::uint32_t ADC_JOFR1_offset = 0x14;
constexpr std::uint32_t ADC_JOFR2_offset = 0x18;
constexpr std::uint32_t ADC_JOFR3_offset = 0x1C;
constexpr std::uint32_t ADC_JOFR4_offset = 0x20;
constexpr std::uint32_t ADC_HTR_offset = 0x24;
constexpr std::uint32_t ADC_LTR_offset = 0x28;
constexpr std::uint32_t ADC_SQR1_offset = 0x2C;
constexpr std::uint32_t ADC_SQR2_offset = 0x30;
constexpr std::uint32_t ADC_SQR3_offset = 0x34;
constexpr std::uint32_t ADC_JSQR_offset = 0x38;
constexpr std::uint32_t ADC_JDR1_offset = 0x3C;
constexpr std::uint32_t ADC_JDR2_offset = 0x40;
constexpr std::uint32_t ADC_JDR3_offset = 0x44;
constexpr std::uint32_t ADC_JDR4_offset = 0x48;
constexpr std::uint32_t ADC_DR_offset = 0x4C;

// ADC_Common registers offset
constexpr std::uint32_t ADC_Common_CSR_offset = 0x300;
constexpr std::uint32_t ADC_Common_CCR_offset = 0x304;
constexpr std::uint32_t ADC_Common_CDR_offset = 0x308;

// CRC registers offset
constexpr std::uint32_t CRC_DR_offset = 0x00;
constexpr std::uint8_t CRC_IDR_offset_8 = 0x04;
constexpr std::uint32_t CRC_IDR_offset = 0x04;
constexpr std::uint32_t CRC_CR_offset = 0x08;

// DBGMCU registers offset
constexpr std::uint32_t DBGMCU_IDCODE_offset = 0x00;
constexpr std::uint32_t DBGMCU_CR_offset = 0x04;
constexpr std::uint32_t DBGMCU_APB1FZ_offset = 0x08;
constexpr std::uint32_t DBGMCU_APB2FZ_offset = 0x0C;

// DMA_Stream registers offset
constexpr std::uint32_t DMA_Stream_CR_offset = 0x00;
constexpr std::uint32_t DMA_Stream_NDTR_offset = 0x04;
constexpr std::uint32_t DMA_Stream_PAR_offset = 0x08;
constexpr std::uint32_t DMA_Stream_M0AR_offset = 0x0C;
constexpr std::uint32_t DMA_Stream_M1AR_offset = 0x10;
constexpr std::uint32_t DMA_Stream_FCR_offset = 0x14;

// DMA registers offset
constexpr std::uint32_t DMA_LISR_offset = 0x00;
constexpr std::uint32_t DMA_HISR_offset = 0x04;
constexpr std::uint32_t DMA_LIFCR_offset = 0x08;
constexpr std::uint32_t DMA_HIFCR_offset = 0x0C;

// EXTI registers offset
constexpr std::uint32_t EXTI_IMR_offset = 0x00;
constexpr std::uint32_t EXTI_EMR_offset = 0x04;
constexpr std::uint32_t EXTI_RTSR_offset = 0x08;
constexpr std::uint32_t EXTI_FTSR_offset = 0x0C;
constexpr std::uint32_t EXTI_SWIER_offset = 0x10;
constexpr std::uint32_t EXTI_PR_offset = 0x14;

// FLASH registers offset
constexpr std::uint32_t FLASH_ACR_offset = 0x00;
constexpr std::uint32_t FLASH_KEYR_offset = 0x04;
constexpr std::uint32_t FLASH_OPTKEYR_offset = 0x08;
constexpr std::uint32_t FLASH_SR_offset = 0x0C;
constexpr std::uint32_t FLASH_CR_offset = 0x10;
constexpr std::uint32_t FLASH_OPTCR_offset = 0x14;
constexpr std::uint32_t FLASH_OPTCR1_offset = 0x18;

// GPIO registers offset
constexpr std::uint32_t GPIO_MODER_offset = 0x00;
constexpr std::uint32_t GPIO_OTYPER_offset = 0x04;
constexpr std::uint32_t GPIO_OSPEEDR_offset = 0x08;
constexpr std::uint32_t GPIO_PUPDR_offset = 0x0C;
constexpr std::uint32_t GPIO_IDR_offset = 0x10;
constexpr std::uint32_t GPIO_ODR_offset = 0x14;
constexpr std::uint32_t GPIO_BSRR_offset = 0x18;
constexpr std::uint32_t GPIO_LCKR_offset = 0x1C;
constexpr std::uint32_t GPIO_AFR_offset = 0x20;

// SYSCFG registers offset
constexpr std::uint32_t SYSCFG_MEMRMP_offset = 0x00;
constexpr std::uint32_t SYSCFG_PMC_offset = 0x04;
constexpr std::uint32_t SYSCFG_EXTICR_offset = 0x08;
constexpr std::uint32_t SYSCFG_CMPCR_offset = 0x20;

// I2C registers offset
constexpr std::uint32_t I2C_CR1_offset = 0x00;
constexpr std::uint32_t I2C_CR2_offset = 0x04;
constexpr std::uint32_t I2C_OAR1_offset = 0x08;
constexpr std::uint32_t I2C_OAR2_offset = 0x0C;
constexpr std::uint32_t I2C_DR_offset = 0x10;
constexpr std::uint32_t I2C_SR1_offset = 0x14;
constexpr std::uint32_t I2C_SR2_offset = 0x18;
constexpr std::uint32_t I2C_CCR_offset = 0x1C;
constexpr std::uint32_t I2C_TRISE_offset = 0x20;
constexpr std::uint32_t I2C_FLTR_offset = 0x24;

// IWDG registers offset
constexpr std::uint32_t IWDG_KR_offset = 0x00;
constexpr std::uint32_t IWDG_PR_offset = 0x04;
constexpr std::uint32_t IWDG_RLR_offset = 0x08;
constexpr std::uint32_t IWDG_SR_offset = 0x0C;

// PWR registers offset
constexpr std::uint32_t PWR_CR_offset = 0x00;
constexpr std::uint32_t PWR_CSR_offset = 0x04;

// RCC registers offset
constexpr std::uint32_t RCC_CR_offset = 0x00;
constexpr std::uint32_t RCC_PLLCFGR_offset = 0x04;
constexpr std::uint32_t RCC_CFGR_offset = 0x08;
constexpr std::uint32_t RCC_CIR_offset = 0x0C;
constexpr std::uint32_t RCC_AHB1RSTR_offset = 0x10;
constexpr std::uint32_t RCC_AHB2RSTR_offset = 0x14;
constexpr std::uint32_t RCC_AHB3RSTR_offset = 0x18;
constexpr std::uint32_t RCC_APB1RSTR_offset = 0x20;
constexpr std::uint32_t RCC_APB2RSTR_offset = 0x24;
constexpr std::uint32_t RCC_AHB1ENR_offset = 0x30;
constexpr std::uint32_t RCC_AHB2ENR_offset = 0x34;
constexpr std::uint32_t RCC_AHB3ENR_offset = 0x38;
constexpr std::uint32_t RCC_APB1ENR_offset = 0x40;
constexpr std::uint32_t RCC_APB2ENR_offset = 0x44;
constexpr std::uint32_t RCC_AHB1LPENR_offset = 0x50;
constexpr std::uint32_t RCC_AHB2LPENR_offset = 0x54;
constexpr std::uint32_t RCC_AHB3LPENR_offset = 0x58;
constexpr std::uint32_t RCC_APB1LPENR_offset = 0x60;
constexpr std::uint32_t RCC_APB2LPENR_offset = 0x64;
constexpr std::uint32_t RCC_BDCR_offset = 0x70;
constexpr std::uint32_t RCC_CSR_offset = 0x74;
constexpr std::uint32_t RCC_SSCGR_offset = 0x80;
constexpr std::uint32_t RCC_PLLI2SCFGR_offset = 0x84;
constexpr std::uint32_t RCC_DCKCFGR_offset = 0x8C;

// RTC registers offset
constexpr std::uint32_t RTC_TR_offset = 0x00;
constexpr std::uint32_t RTC_DR_offset = 0x04;
constexpr std::uint32_t RTC_CR_offset = 0x08;
constexpr std::uint32_t RTC_ISR_offset = 0x0C;
constexpr std::uint32_t RTC_PRER_offset = 0x10;
constexpr std::uint32_t RTC_WUTR_offset = 0x14;
constexpr std::uint32_t RTC_CALIBR_offset = 0x18;
constexpr std::uint32_t RTC_ALRMAR_offset = 0x1C;
constexpr std::uint32_t RTC_ALRMBR_offset = 0x20;
constexpr std::uint32_t RTC_WPR_offset = 0x24;
constexpr std::uint32_t RTC_SSR_offset = 0x28;
constexpr std::uint32_t RTC_SHIFTR_offset = 0x2C;
constexpr std::uint32_t RTC_TSTR_offset = 0x30;
constexpr std::uint32_t RTC_TSDR_offset = 0x34;
constexpr std::uint32_t RTC_TSSSR_offset = 0x38;
constexpr std::uint32_t RTC_CALR_offset = 0x3C;
constexpr std::uint32_t RTC_TAFCR_offset = 0x40;
constexpr std::uint32_t RTC_ALRMASSR_offset = 0x44;
constexpr std::uint32_t RTC_ALRMBSSR_offset = 0x48;
constexpr std::uint32_t RTC_BKP0R_offset = 0x50;
constexpr std::uint32_t RTC_BKP1R_offset = 0x54;
constexpr std::uint32_t RTC_BKP2R_offset = 0x58;
constexpr std::uint32_t RTC_BKP3R_offset = 0x5C;
constexpr std::uint32_t RTC_BKP4R_offset = 0x60;
constexpr std::uint32_t RTC_BKP5R_offset = 0x64;
constexpr std::uint32_t RTC_BKP6R_offset = 0x68;
constexpr std::uint32_t RTC_BKP7R_offset = 0x6C;
constexpr std::uint32_t RTC_BKP8R_offset = 0x70;
constexpr std::uint32_t RTC_BKP9R_offset = 0x74;
constexpr std::uint32_t RTC_BKP10R_offset = 0x78;
constexpr std::uint32_t RTC_BKP11R_offset = 0x7C;
constexpr std::uint32_t RTC_BKP12R_offset = 0x80;
constexpr std::uint32_t RTC_BKP13R_offset = 0x84;
constexpr std::uint32_t RTC_BKP14R_offset = 0x88;
constexpr std::uint32_t RTC_BKP15R_offset = 0x8C;
constexpr std::uint32_t RTC_BKP16R_offset = 0x90;
constexpr std::uint32_t RTC_BKP17R_offset = 0x94;
constexpr std::uint32_t RTC_BKP18R_offset = 0x98;
constexpr std::uint32_t RTC_BKP19R_offset = 0x9C;

// SDIO registers offset
constexpr std::uint32_t SDIO_POWER_offset = 0x00;
constexpr std::uint32_t SDIO_CLKCR_offset = 0x04;
constexpr std::uint32_t SDIO_ARG_offset = 0x08;
constexpr std::uint32_t SDIO_CMD_offset = 0x0C;
constexpr std::uint32_t SDIO_RESPCMD_offset = 0x10;
constexpr std::uint32_t SDIO_RESP1_offset = 0x14;
constexpr std::uint32_t SDIO_RESP2_offset = 0x18;
constexpr std::uint32_t SDIO_RESP3_offset = 0x1C;
constexpr std::uint32_t SDIO_RESP4_offset = 0x20;
constexpr std::uint32_t SDIO_DTIMER_offset = 0x24;
constexpr std::uint32_t SDIO_DLEN_offset = 0x28;
constexpr std::uint32_t SDIO_DCTRL_offset = 0x2C;
constexpr std::uint32_t SDIO_DCOUNT_offset = 0x30;
constexpr std::uint32_t SDIO_STA_offset = 0x34;
constexpr std::uint32_t SDIO_ICR_offset = 0x38;
constexpr std::uint32_t SDIO_MASK_offset = 0x3C;
constexpr std::uint32_t SDIO_FIFOCNT_offset = 0x48;
constexpr std::uint32_t SDIO_FIFO_offset = 0x80;

// SPI registers offset
constexpr std::uint32_t SPI_CR1_offset = 0x00;
constexpr std::uint32_t SPI_CR2_offset = 0x04;
constexpr std::uint32_t SPI_SR_offset = 0x08;
constexpr std::uint32_t SPI_DR_offset = 0x0C;
constexpr std::uint32_t SPI_CRCPR_offset = 0x10;
constexpr std::uint32_t SPI_RXCRCR_offset = 0x14;
constexpr std::uint32_t SPI_TXCRCR_offset = 0x18;
constexpr std::uint32_t SPI_I2S_I2SCFGR_offset = 0x1C;
constexpr std::uint32_t SPI_I2S_I2SPR_offset = 0x20;

// TIM registers offset
constexpr std::uint32_t TIM_CR1_offset = 0x00;
constexpr std::uint32_t TIM_CR2_offset = 0x04;
constexpr std::uint32_t TIM_SMCR_offset = 0x08;
constexpr std::uint32_t TIM_DIER_offset = 0x0C;
constexpr std::uint32_t TIM_SR_offset = 0x10;
constexpr std::uint32_t TIM_EGR_offset = 0x14;
constexpr std::uint32_t TIM_CCMR1_offset = 0x18;
constexpr std::uint32_t TIM_CCMR2_offset = 0x1C;
constexpr std::uint32_t TIM_CCER_offset = 0x20;
constexpr std::uint32_t TIM_CNT_offset = 0x24;
constexpr std::uint32_t TIM_PSC_offset = 0x28;
constexpr std::uint32_t TIM_ARR_offset = 0x2C;
constexpr std::uint32_t TIM_RCR_offset = 0x30;
constexpr std::uint32_t TIM_CCR1_offset = 0x34;
constexpr std::uint32_t TIM_CCR2_offset = 0x38;
constexpr std::uint32_t TIM_CCR3_offset = 0x3C;
constexpr std::uint32_t TIM_CCR4_offset = 0x40;
constexpr std::uint32_t TIM_BDTR_offset = 0x44;
constexpr std::uint32_t TIM_DCR_offset = 0x48;
constexpr std::uint32_t TIM_DMAR_offset = 0x4C;
constexpr std::uint32_t TIM_OR_offset = 0x50;

// USART registers offset
constexpr std::uint32_t USART_SR_offset = 0x00;
constexpr std::uint32_t USART_DR_offset = 0x04;
constexpr std::uint32_t USART_BRR_offset = 0x08;
constexpr std::uint32_t USART_CR1_offset = 0x0C;
constexpr std::uint32_t USART_CR2_offset = 0x10;
constexpr std::uint32_t USART_CR3_offset = 0x14;
constexpr std::uint32_t USART_GTPR_offset = 0x18;

// WWDG registers offset
constexpr std::uint32_t WWDG_CR_offset = 0x00;
constexpr std::uint32_t WWDG_CFR_offset = 0x04;
constexpr std::uint32_t WWDG_SR_offset = 0x08;

// USB_OTG core registers offset
constexpr std::uint32_t USB_OTG_Global_GOTGCTL_offset = 0x000;
constexpr std::uint32_t USB_OTG_Global_GOTGINT_offset = 0x004;
constexpr std::uint32_t USB_OTG_Global_GAHBCFG_offset = 0x008;
constexpr std::uint32_t USB_OTG_Global_GUSBCFG_offset = 0x00C;
constexpr std::uint32_t USB_OTG_Global_GRSTCTL_offset = 0x010;
constexpr std::uint32_t USB_OTG_Global_GINTSTS_offset = 0x014;
constexpr std::uint32_t USB_OTG_Global_GINTMSK_offset = 0x018;
constexpr std::uint32_t USB_OTG_Global_GRXSTSR_offset = 0x01C;
constexpr std::uint32_t USB_OTG_Global_GRXSTSP_offset = 0x020;
constexpr std::uint32_t USB_OTG_Global_GRXFSIZ_offset = 0x024;
constexpr std::uint32_t USB_OTG_Global_DIEPTXF0_HNPTXFSIZ_offset = 0x028;
constexpr std::uint32_t USB_OTG_Global_HNPTXSTS_offset = 0x02C;
constexpr std::uint32_t USB_OTG_Global_GCCFG_offset = 0x038;
constexpr std::uint32_t USB_OTG_Global_CID_offset = 0x03C;
constexpr std::uint32_t USB_OTG_Global_HPTXFSIZ_offset = 0x100;
constexpr std::uint32_t USB_OTG_Global_DIEPTXF_offset = 0x104;

// USB_OTG device registers offset
constexpr std::uint32_t USB_OTG_Device_DCFG_offset = 0x800;
constexpr std::uint32_t USB_OTG_Device_DCTL_offset = 0x804;
constexpr std::uint32_t USB_OTG_Device_DSTS_offset = 0x808;
constexpr std::uint32_t USB_OTG_Device_DIEPMSK_offset = 0x810;
constexpr std::uint32_t USB_OTG_Device_DOEPMSK_offset = 0x814;
constexpr std::uint32_t USB_OTG_Device_DAINT_offset = 0x818;
constexpr std::uint32_t USB_OTG_Device_DAINTMSK_offset = 0x81C;
constexpr std::uint32_t USB_OTG_Device_DVBUSDIS_offset = 0x828;
constexpr std::uint32_t USB_OTG_Device_DVBUSPULSE_offset = 0x82C;
constexpr std::uint32_t USB_OTG_Device_DTHRCTL_offset = 0x830;
constexpr std::uint32_t USB_OTG_Device_DIEPEMPMSK_offset = 0x834;
constexpr std::uint32_t USB_OTG_Device_DEACHINT_offset = 0x838;
constexpr std::uint32_t USB_OTG_Device_DEACHMSK_offset = 0x83C;
constexpr std::uint32_t USB_OTG_Device_DINEP1MSK_offset = 0x844;
constexpr std::uint32_t USB_OTG_Device_DOUTEP1MSK_offset = 0x884;

// USB_OTG_IN_Endpoint-specific registers offset
constexpr std::uint32_t USB_OTG_INEndpoint_DIEPCTL_offset = 0x00;
constexpr std::uint32_t USB_OTG_INEndpoint_DIEPINT_offset = 0x08;
constexpr std::uint32_t USB_OTG_INEndpoint_DIEPTSIZ_offset = 0x10;
constexpr std::uint32_t USB_OTG_INEndpoint_DIEPDMA_offset = 0x14;
constexpr std::uint32_t USB_OTG_INEndpoint_DTXFSTS_offset = 0x18;

// USB_OTG_OUT_Endpoint-specific registers offset
constexpr std::uint32_t USB_OTG_OUTEndPoint_DOEPCTL_offset = 0x00;
constexpr std::uint32_t USB_OTG_OUTEndPoint_DOEPINT_offset = 0x08;
constexpr std::uint32_t USB_OTG_OUTEndPoint_DOEPTSIZ_offset = 0x10;
constexpr std::uint32_t USB_OTG_OUTEndPoint_DOEPDMA_offset = 0x14;

// USB_OTG_Host mode registers offset
constexpr std::uint32_t USB_OTG_Host_HCFG_offset = 0x400;
constexpr std::uint32_t USB_OTG_Host_HFIR_offset = 0x404;
constexpr std::uint32_t USB_OTG_Host_HFNUM_offset = 0x408;
constexpr std::uint32_t USB_OTG_Host_HPTXSTS_offset = 0x410;
constexpr std::uint32_t USB_OTG_Host_HAINT_offset = 0x414;
constexpr std::uint32_t USB_OTG_Host_HAINTMSK_offset = 0x418;

// USB_OTG_Host channel registers offset
constexpr std::uint32_t USB_OTG_HostChannel_HCCHAR_offset = 0x500;
constexpr std::uint32_t USB_OTG_HostChannel_HCSPLT_offset = 0x504;
constexpr std::uint32_t USB_OTG_HostChannel_HCINT_offset = 0x508;
constexpr std::uint32_t USB_OTG_HostChannel_HCINTMSK_offset = 0x50C;
constexpr std::uint32_t USB_OTG_HostChannel_HCTSIZ_offset = 0x510;
constexpr std::uint32_t USB_OTG_HostChannel_HCDMA_offset = 0x514;
//
//constexpr std::uint32_t periph_base = 0x40000000U;
//constexpr std::uint32_t apb1periph_base = periph_base;
//constexpr std::uint32_t apb2periph_base = periph_base + 0x00010000U;
//constexpr std::uint32_t ahb1periph_base = periph_base + 0x00020000U;
//constexpr std::uint32_t ahb2periph_base = periph_base + 0x10000000U;
//
//constexpr std::uint32_t rcc_base = ahb1periph_base + 0x3800u;
//
//constexpr std::uint32_t rcc_ahb1enr_offset = 0x30;
//
//constexpr std::uint32_t rcc_ahb1enr = rcc_base + rcc_ahb1enr_offset;
//
constexpr std::uint32_t rcc_ahb1enr_gpioaen = 0x1u << 0u;
constexpr std::uint32_t rcc_ahb1enr_gpioben = 0x1u << 1u;
constexpr std::uint32_t rcc_ahb1enr_gpiocen = 0x1u << 2u;
constexpr std::uint32_t rcc_ahb1enr_gpioden = 0x1u << 3u;
//
//constexpr std::uint32_t gpioa_base = ahb1periph_base + 0x0000u;
//constexpr std::uint32_t gpiob_base = ahb1periph_base + 0x0400u;
//constexpr std::uint32_t gpioc_base = ahb1periph_base + 0x0800u;
//constexpr std::uint32_t gpiod_base = ahb1periph_base + 0x0c00u;
//constexpr std::uint32_t gpioe_base = ahb1periph_base + 0x1000u;
//constexpr std::uint32_t gpioh_base = ahb1periph_base + 0x1c00u;
//
//constexpr std::uint32_t exti_base = apb2periph_base + 0x3C00U;
//
//constexpr std::uint32_t gpio_moder_offset = 0x00;
//constexpr std::uint32_t gpio_pupdr_offset = 0x0C;
//
//constexpr std::uint32_t exti_imr_offset = 0x00;

enum class Port
{
    A,
    B,
    C,
    D
};

///////////////////////////////////////////////////////////////////////////////
// Peripherals, registers and operations definitions
///////////////////////////////////////////////////////////////////////////////

struct Rcc : protected Periph<RCC_BASE>
{
    using AHB1ENR = Register32<RCC_AHB1ENR_offset>;

    template<Port... port>
    static constexpr void enable() noexcept
    {
        AHB1ENR::set_bit<((0x1u << static_cast<std::uint32_t>(port)), ...)>();
        volatile std::uint32_t tmpreg = AHB1ENR::get_bit<rcc_ahb1enr_gpiocen>();
        (void)tmpreg;
    }
};

enum class Pin
{
    _00,
    _01,
    _02,
    _03,
    _04,
    _05,
    _06,
    _07,
    _08,
    _09,
    _10,
    _11,
    _12,
    _13,
    _14,
    _15
};

enum class GpioMode
{
    Input,
    Output,
    Alternate,
    Analog
};

enum class PullMode
{
    No,
    Up,
    Down
};

template<std::uint32_t gpio_base>
struct Gpio : protected Periph<gpio_base>
{
    using moder = typename Gpio::template Register32<GPIO_MODER_offset>;
    using pupdr = typename Gpio::template Register32<GPIO_PUPDR_offset>;

    template<Pin pin, GpioMode mode>
    static constexpr void setPinMode() noexcept
    {
        auto pinx2_ = static_cast<std::uint32_t>(pin) * 2;
        auto mode_ = static_cast<std::uint32_t>(mode);
        moder::modify(0b11 << pinx2_, mode_ << pinx2_);
    }

    template<Pin pin, PullMode mode>
    static constexpr void setPullMode() noexcept
    {
        auto pinx2_ = static_cast<std::uint32_t>(pin) * 2;
        auto mode_ = static_cast<std::uint32_t>(mode);
        moder::modify(0b11 << pinx2_, mode_ << pinx2_);
    }
};

using GpioA = Gpio<GPIOA_BASE>;
using GpioB = Gpio<GPIOB_BASE>;
using GpioC = Gpio<GPIOC_BASE>;
using GpioD = Gpio<GPIOD_BASE>;

enum class ExtILine
{
    _00,
    _01,
    _02,
    _03,
    _04,
    _05,
    _06,
    _07,
    _08,
    _09,
    _10,
    _11,
    _12,
    _13,
    _14,
    _15
};

struct Exti : protected Periph<EXTI_BASE>
{
    using imr = Register32<EXTI_IMR_offset>;

    template<ExtILine line>
    static constexpr void enableIT() noexcept
    {
        imr::set_bit<0x1U << static_cast<std::uint32_t>(line)>();
    }
};

} // namespace stm32f411xe
} // namespace mhai

#endif // MHAI_DEVICE_MHAI_STM32F411XE_HPP_
