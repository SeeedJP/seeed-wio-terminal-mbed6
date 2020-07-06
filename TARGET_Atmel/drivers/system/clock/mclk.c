
#include <cmsis.h>
#include <compiler.h>
#include "mclk_drv.h"

/**
 * \name Bus Clock Masking
 * @{
 */

/**
 * \brief Set bits in the clock mask for the AHB bus.
 *
 * This function will set bits in the clock mask for the AHB bus.
 * Any bits set to 1 will enable that clock, 0 bits in the mask
 * will be ignored.
 *
 * \param[in] ahb_mask  AHB clock mask to enable
 */
void system_ahb_clock_set_mask(
		const uint32_t ahb_mask)
{
	MCLK_REGS->MCLK_AHBMASK |= ahb_mask;
}

/**
 * \brief Clear bits in the clock mask for the AHB bus.
 *
 * This function will clear bits in the clock mask for the AHB bus.
 * Any bits set to 1 will disable that clock, 0 bits in the mask
 * will be ignored.
 *
 * \param[in] ahb_mask  AHB clock mask to disable
 */
void system_ahb_clock_clear_mask(
		const uint32_t ahb_mask)
{
	MCLK_REGS->MCLK_AHBMASK &= ~ahb_mask;
}

/**
 * \brief Set bits in the clock mask for an APBx bus.
 *
 * This function will set bits in the clock mask for an APBx bus.
 * Any bits set to 1 will enable the corresponding module clock, zero bits in
 * the mask will be ignored.
 *
 * \param[in] mask  APBx clock mask, a \c SYSTEM_CLOCK_APB_APBx constant from
 *                  the device header files
 * \param[in] bus   Bus to set clock mask bits for, a mask of \c PM_APBxMASK_*
 *                  constants from the device header files
 *
 * \returns Status indicating the result of the clock mask change operation.
 *
 * \retval STATUS_ERR_INVALID_ARG  Invalid bus given
 * \retval STATUS_OK               The clock mask was set successfully
 */
enum status_code system_apb_clock_set_mask(
		const enum system_clock_apb_bus bus,
		const uint32_t mask)
{
	switch (bus) {
		case SYSTEM_CLOCK_APB_APBA:
			MCLK_REGS->MCLK_APBAMASK |= mask;
			break;

		case SYSTEM_CLOCK_APB_APBB:
			MCLK_REGS->MCLK_APBBMASK |= mask;
			break;

		case SYSTEM_CLOCK_APB_APBC:
			MCLK_REGS->MCLK_APBCMASK |= mask;
			break;

		case SYSTEM_CLOCK_APB_APBD:
			MCLK_REGS->MCLK_APBDMASK |= mask;
			break;

		default:
			Assert(false);
			return STATUS_ERR_INVALID_ARG;

	}

	return STATUS_OK;
}

/**
 * \brief Clear bits in the clock mask for an APBx bus.
 *
 * This function will clear bits in the clock mask for an APBx bus.
 * Any bits set to 1 will disable the corresponding module clock, zero bits in
 * the mask will be ignored.
 *
 * \param[in] mask  APBx clock mask, a \c SYSTEM_CLOCK_APB_APBx constant from
 *                  the device header files
 * \param[in] bus   Bus to clear clock mask bits
 *
 * \returns Status indicating the result of the clock mask change operation.
 *
 * \retval STATUS_ERR_INVALID_ARG  Invalid bus ID was given
 * \retval STATUS_OK               The clock mask was changed successfully
 */
enum status_code system_apb_clock_clear_mask(
		const enum system_clock_apb_bus bus,
		const uint32_t mask)
{
	switch (bus) {
		case SYSTEM_CLOCK_APB_APBA:
			MCLK_REGS->MCLK_APBAMASK &= ~mask;
			break;

		case SYSTEM_CLOCK_APB_APBB:
			MCLK_REGS->MCLK_APBBMASK &= ~mask;
			break;

		case SYSTEM_CLOCK_APB_APBC:
			MCLK_REGS->MCLK_APBCMASK &= ~mask;
			break;

		default:
			Assert(false);
			return STATUS_ERR_INVALID_ARG;
	}

	return STATUS_OK;
}

/**
 * @}
 */
