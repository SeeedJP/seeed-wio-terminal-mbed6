/**
 * \file
 *
 * \brief SAM D21 System Interrupt Driver
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef SYSTEM_INTERRUPT_FEATURES_H_INCLUDED
#define SYSTEM_INTERRUPT_FEATURES_H_INCLUDED

#if !defined(__DOXYGEN__)

#if 0
/* Generates a interrupt vector table enum list entry for a given module type
   and index (e.g. "SYSTEM_INTERRUPT_MODULE_TC0 = TC0_IRQn,"). */
#  define _MODULE_IRQn(n, module) \
		SYSTEM_INTERRUPT_MODULE_##module##n = module##n##_IRQn,

/* Generates interrupt vector table enum list entries for all instances of a
   given module type on the selected device. */
#  define _SYSTEM_INTERRUPT_MODULES(name) \
		MREPEAT(name##_INST_NUM, _MODULE_IRQn, name)
#endif

#  define _SYSTEM_INTERRUPT_IPSR_MASK              0x0000003f
#  define _SYSTEM_INTERRUPT_PRIORITY_MASK          0x00000003

#  define _SYSTEM_INTERRUPT_EXTERNAL_VECTOR_START  0

#  define _SYSTEM_INTERRUPT_SYSTICK_PRI_POS        30
#endif

/**
 * \addtogroup asfdoc_sam0_system_interrupt_group
 * @{
 */

/**
 * \brief Table of possible system interrupt/exception vector numbers.
 *
 * Table of all possible interrupt and exception vector indexes within the
 * SAM D21 device. Check peripherals configuration in SAM D21 datasheet for
 * available vector index for specific device.
 *
 */
#if defined(__DOXYGEN__)
/** \note The actual enumeration name is "system_interrupt_vector". */
enum system_interrupt_vector_samd21 {
#else
enum system_interrupt_vector {
#endif
	/** Interrupt vector index for a NMI interrupt */
	SYSTEM_INTERRUPT_NON_MASKABLE      = NonMaskableInt_IRQn,
	/** Interrupt vector index for a Hard Fault memory access exception */
	SYSTEM_INTERRUPT_HARD_FAULT        = HardFault_IRQn,
	/** Interrupt vector index for a Supervisor Call exception */
	SYSTEM_INTERRUPT_SV_CALL           = SVCall_IRQn,
	/** Interrupt vector index for a Pending Supervisor interrupt */
	SYSTEM_INTERRUPT_PENDING_SV        = PendSV_IRQn,
	/** Interrupt vector index for a System Tick interrupt */
	SYSTEM_INTERRUPT_SYSTICK           = SysTick_IRQn,

	/** Interrupt vector index for a Power Manager peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_PM         = PM_IRQn,
	/** Interrupt vector index for a Watch Dog peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_WDT        = WDT_IRQn,
	/** Interrupt vector index for a Real Time Clock peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_RTC        = RTC_IRQn,
	/** Interrupt vector index for an External Interrupt peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_0 = EIC_EXTINT_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_1 = EIC_EXTINT_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_2 = EIC_EXTINT_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_3 = EIC_EXTINT_3_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_4 = EIC_EXTINT_4_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_5 = EIC_EXTINT_5_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_6 = EIC_EXTINT_6_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_7 = EIC_EXTINT_7_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_8 = EIC_EXTINT_8_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_9 = EIC_EXTINT_9_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_10 = EIC_EXTINT_10_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_11 = EIC_EXTINT_11_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_12 = EIC_EXTINT_12_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_13 = EIC_EXTINT_13_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_14 = EIC_EXTINT_14_IRQn,
	SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_15 = EIC_EXTINT_15_IRQn,
	/** Interrupt vector index for a Non Volatile Memory Controller interrupt */
	SYSTEM_INTERRUPT_MODULE_NVMCTRL_0    = NVMCTRL_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_NVMCTRL_1    = NVMCTRL_1_IRQn,
	/** Interrupt vector index for a Direct Memory Access interrupt */
	SYSTEM_INTERRUPT_MODULE_DMA_0        = DMAC_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_DMA_1        = DMAC_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_DMA_2        = DMAC_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_DMA_3        = DMAC_3_IRQn,
	SYSTEM_INTERRUPT_MODULE_DMA_OTHER    = DMAC_OTHER_IRQn,
#if defined(__DOXYGEN__) || defined(ID_USB)
	/** Interrupt vector index for a Universal Serial Bus interrupt */
	SYSTEM_INTERRUPT_MODULE_USB_OTHER    = USB_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_USB_SOF_HSOF = USB_SOF_HSOF_IRQn,
	SYSTEM_INTERRUPT_MODULE_USB_TRCPT0   = USB_TRCPT0_IRQn,
	SYSTEM_INTERRUPT_MODULE_USB_TRCPT1   = USB_TRCPT1_IRQn,
#endif
	/** Interrupt vector index for an Event System interrupt */
	SYSTEM_INTERRUPT_MODULE_EVSYS_0      = EVSYS_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_EVSYS_1      = EVSYS_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_EVSYS_2      = EVSYS_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_EVSYS_3      = EVSYS_3_IRQn,
	SYSTEM_INTERRUPT_MODULE_EVSYS_OTHER  = EVSYS_OTHER_IRQn,
#if defined(__DOXYGEN__)
	/** Interrupt vector index for a SERCOM peripheral interrupt.
	 *
	 *  Each specific device may contain several SERCOM peripherals; each module
	 *  instance will have its own entry in the table, with the instance number
	 *  substituted for "n" in the entry name (e.g.
	 *  \c SYSTEM_INTERRUPT_MODULE_SERCOM0).
	 */
	SYSTEM_INTERRUPT_MODULE_SERCOMn    = SERCOMn_IRQn,

	/** Interrupt vector index for a Timer/Counter Control peripheral interrupt.
	 *
	 *  Each specific device may contain several TCC peripherals; each module
	 *  instance will have its own entry in the table, with the instance number
	 *  substituted for "n" in the entry name (e.g.
	 *  \c SYSTEM_INTERRUPT_MODULE_TCC0).
	 */
	SYSTEM_INTERRUPT_MODULE_TCCn        = TCCn_IRQn,

	/** Interrupt vector index for a Timer/Counter peripheral interrupt.
	 *
	 *  Each specific device may contain several TC peripherals; each module
	 *  instance will have its own entry in the table, with the instance number
	 *  substituted for "n" in the entry name (e.g.
	 *  \c SYSTEM_INTERRUPT_MODULE_TC3).
	 */
	SYSTEM_INTERRUPT_MODULE_TCn        = TCn_IRQn,
#else
	SYSTEM_INTERRUPT_MODULE_SERCOM0_0      = SERCOM0_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM0_1      = SERCOM0_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM0_2      = SERCOM0_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM0_OTHER  = SERCOM0_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM1_0      = SERCOM1_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM1_1      = SERCOM1_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM1_2      = SERCOM1_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM1_OTHER  = SERCOM1_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM2_0      = SERCOM2_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM2_1      = SERCOM2_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM2_2      = SERCOM2_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM2_OTHER  = SERCOM2_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM3_0      = SERCOM3_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM3_1      = SERCOM3_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM3_2      = SERCOM3_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM3_OTHER  = SERCOM3_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM4_0      = SERCOM4_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM4_1      = SERCOM4_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM4_2      = SERCOM4_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM4_OTHER  = SERCOM4_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM5_0      = SERCOM5_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM5_1      = SERCOM5_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM5_2      = SERCOM5_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM5_OTHER  = SERCOM5_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM6_0      = SERCOM6_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM6_1      = SERCOM6_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM6_2      = SERCOM6_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM6_OTHER  = SERCOM6_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM7_0      = SERCOM7_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM7_1      = SERCOM7_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM7_2      = SERCOM7_2_IRQn,
	SYSTEM_INTERRUPT_MODULE_SERCOM7_OTHER  = SERCOM7_OTHER_IRQn,

	SYSTEM_INTERRUPT_MODULES_TCC0_OTHER = TCC0_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC0_MC0   = TCC0_MC0_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC0_MC1   = TCC0_MC1_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC0_MC2   = TCC0_MC2_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC0_MC3   = TCC0_MC3_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC0_MC4   = TCC0_MC4_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC0_MC5   = TCC0_MC5_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC1_OTHER = TCC1_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC1_MC0   = TCC1_MC0_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC1_MC1   = TCC1_MC1_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC1_MC2   = TCC1_MC2_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC1_MC3   = TCC1_MC3_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC2_OTHER = TCC2_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC2_MC0   = TCC2_MC0_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC2_MC1   = TCC2_MC1_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC2_MC2   = TCC2_MC2_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC3_OTHER = TCC3_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC3_MC0   = TCC3_MC0_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC3_MC1   = TCC3_MC1_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC4_OTHER = TCC4_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC4_MC0   = TCC4_MC0_IRQn,
	SYSTEM_INTERRUPT_MODULES_TCC4_MC1   = TCC4_MC1_IRQn,

	SYSTEM_INTERRUPT_MODULE_TC0        = TC0_IRQn,
	SYSTEM_INTERRUPT_MODULE_TC1        = TC1_IRQn,
	SYSTEM_INTERRUPT_MODULE_TC2        = TC2_IRQn,
	SYSTEM_INTERRUPT_MODULE_TC3        = TC3_IRQn,
	SYSTEM_INTERRUPT_MODULE_TC4        = TC4_IRQn,
	SYSTEM_INTERRUPT_MODULE_TC5        = TC5_IRQn,
#  if defined(ID_TC6)
	SYSTEM_INTERRUPT_MODULE_TC6        = TC6_IRQn,
#  endif
#  if defined(ID_TC7)
	SYSTEM_INTERRUPT_MODULE_TC7        = TC7_IRQn,
#  endif
#endif

#if defined(__DOXYGEN__) || defined(ID_ADC)
	/** Interrupt vector index for an Analog-to-Digital peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_ADC        = ADC_IRQn,
#endif

#if defined(__DOXYGEN__) || defined(ID_AC)
	/** Interrupt vector index for an Analog Comparator peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_AC         = AC_IRQn,
#endif

#if defined(__DOXYGEN__) || defined(ID_DAC)
	/** Interrupt vector index for a Digital-to-Analog peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_DAC_OTHER    = DAC_OTHER_IRQn,
	SYSTEM_INTERRUPT_MODULE_DAC_EMPTY_0  = DAC_EMPTY_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_DAC_EMPTY_1  = DAC_EMPTY_1_IRQn,
	SYSTEM_INTERRUPT_MODULE_DAC_RESRDY_0 = DAC_RESRDY_0_IRQn,
	SYSTEM_INTERRUPT_MODULE_DAC_RESRDY_1 = DAC_RESRDY_1_IRQn,
#endif
#if defined(__DOXYGEN__) || defined(ID_PTC)
	/** Interrupt vector index for a Peripheral Touch Controller peripheral
	 *  interrupt */
	SYSTEM_INTERRUPT_MODULE_PTC        = PTC_IRQn,
#endif
#if defined(__DOXYGEN__) || defined(ID_I2S)
	/** Interrupt vector index for a Inter-IC Sound Interface peripheral
	 *  interrupt */
	SYSTEM_INTERRUPT_MODULE_I2S        = I2S_IRQn,
#endif
#if defined(__DOXYGEN__) || defined(ID_AC1)
	/** Interrupt vector index for an Analog Comparator 1 peripheral interrupt */
	SYSTEM_INTERRUPT_MODULE_AC1        = AC1_IRQn,
#endif
};

/** @} */

#endif
