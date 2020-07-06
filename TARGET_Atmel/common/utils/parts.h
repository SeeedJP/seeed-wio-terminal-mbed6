#ifndef ATMEL_PARTS_H
#define ATMEL_PARTS_H

/**
 * \defgroup part_macros_group Atmel part identification macros
 *
 * This collection of macros identify which series and families that the various
 * Atmel parts belong to. These can be used to select part-dependent sections of
 * code at compile time.
 *
 * @{
 */

/**
 * \defgroup uc3_part_macros_group AVR UC3 parts
 * @{
 */

/**
 * \name AVR UC3 A series
 * @{
 */
#if defined(__AT32UC3A0128__) || defined(__AVR32_UC3A0128__) || \
	defined(__AT32UC3A0256__) || defined(__AVR32_UC3A0256__) || \
	defined(__AT32UC3A0512__) || defined(__AVR32_UC3A0512__)
#define UC3A0 (1)
#else
#define UC3A0 (0)
#endif

#if defined(__AT32UC3A1128__) || defined(__AVR32_UC3A1128__) || \
	defined(__AT32UC3A1256__) || defined(__AVR32_UC3A1256__) || \
	defined(__AT32UC3A1512__) || defined(__AVR32_UC3A1512__)
#define UC3A1 (1)
#else
#define UC3A1 (0)
#endif

#if defined(__AT32UC3A364__) || defined(__AVR32_UC3A364__)   || \
	defined(__AT32UC3A364S__) || defined(__AVR32_UC3A364S__)  || \
	defined(__AT32UC3A3128__) || defined(__AVR32_UC3A3128__)  || \
	defined(__AT32UC3A3128S__) || defined(__AVR32_UC3A3128S__) || \
	defined(__AT32UC3A3256__) || defined(__AVR32_UC3A3256__)  || \
	defined(__AT32UC3A3256S__) || defined(__AVR32_UC3A3256S__)
#define UC3A3 (1)
#else
#define UC3A3 (0)
#endif

#if defined(__AT32UC3A464__) || defined(__AVR32_UC3A464__)   || \
	defined(__AT32UC3A464S__) || defined(__AVR32_UC3A464S__)  || \
	defined(__AT32UC3A4128__) || defined(__AVR32_UC3A4128__)  || \
	defined(__AT32UC3A4128S__) || defined(__AVR32_UC3A4128S__) || \
	defined(__AT32UC3A4256__) || defined(__AVR32_UC3A4256__)  || \
	defined(__AT32UC3A4256S__) || defined(__AVR32_UC3A4256S__)
#define UC3A4 (1)
#else
#define UC3A4 (0)
#endif
/** @} */

/**
 * \name AVR UC3 B series
 * @{
 */
#if defined(__AT32UC3B064__) || defined(__AVR32_UC3B064__)  || \
	defined(__AT32UC3B0128__) || defined(__AVR32_UC3B0128__) || \
	defined(__AT32UC3B0256__) || defined(__AVR32_UC3B0256__) || \
	defined(__AT32UC3B0512__) || defined(__AVR32_UC3B0512__)
#define UC3B0 (1)
#else
#define UC3B0 (0)
#endif

#if defined(__AT32UC3B164__) || defined(__AVR32_UC3B164__)  || \
	defined(__AT32UC3B1128__) || defined(__AVR32_UC3B1128__) || \
	defined(__AT32UC3B1256__) || defined(__AVR32_UC3B1256__) || \
	defined(__AT32UC3B1512__) || defined(__AVR32_UC3B1512__)
#define UC3B1 (1)
#else
#define UC3B1 (0)
#endif
/** @} */

/**
 * \name AVR UC3 C series
 * @{
 */
#if defined(__AT32UC3C064C__) || defined(__AVR32_UC3C064C__)  || \
	defined(__AT32UC3C0128C__) || defined(__AVR32_UC3C0128C__) || \
	defined(__AT32UC3C0256C__) || defined(__AVR32_UC3C0256C__) || \
	defined(__AT32UC3C0512C__) || defined(__AVR32_UC3C0512C__)
#define UC3C0 (1)
#else
#define UC3C0 (0)
#endif

#if defined(__AT32UC3C164C__) || defined(__AVR32_UC3C164C__)  || \
	defined(__AT32UC3C1128C__) || defined(__AVR32_UC3C1128C__) || \
	defined(__AT32UC3C1256C__) || defined(__AVR32_UC3C1256C__) || \
	defined(__AT32UC3C1512C__) || defined(__AVR32_UC3C1512C__)
#define UC3C1 (1)
#else
#define UC3C1 (0)
#endif

#if defined(__AT32UC3C264C__) || defined(__AVR32_UC3C264C__)  || \
	defined(__AT32UC3C2128C__) || defined(__AVR32_UC3C2128C__) || \
	defined(__AT32UC3C2256C__) || defined(__AVR32_UC3C2256C__) || \
	defined(__AT32UC3C2512C__) || defined(__AVR32_UC3C2512C__)
#define UC3C2 (1)
#else
#define UC3C2 (0)
#endif
/** @} */

/**
 * \name AVR UC3 D series
 * @{
 */
#if defined(__AT32UC64D3__) || defined(__AVR32_UC64D3__)  || \
	defined(__AT32UC128D3__) || defined(__AVR32_UC128D3__)
#define UC3D3 (1)
#else
#define UC3D3 (0)
#endif

#if defined(__AT32UC64D4__) || defined(__AVR32_UC64D4__)  || \
	defined(__AT32UC128D4__) || defined(__AVR32_UC128D4__)
#define UC3D4 (1)
#else
#define UC3D4 (0)
#endif
/** @} */

/**
 * \name AVR UC3 L series
 * @{
 */
#if defined(__AT32UC3L016__) || defined(__AVR32_UC3L016__) || \
	defined(__AT32UC3L032__) || defined(__AVR32_UC3L032__) || \
	defined(__AT32UC3L064__) || defined(__AVR32_UC3L064__)
#define UC3L0 (1)
#else
#define UC3L0 (0)
#endif

#if defined(__AT32UC3L0128__) || defined(__AVR32_UC3L0128__)
#define UC3L0128 (1)
#else
#define UC3L0128 (0)
#endif

#if defined(__AT32UC3L0256__) || defined(__AVR32_UC3L0256__)
#define UC3L0256 (1)
#else
#define UC3L0256 (0)
#endif

#if defined(__AT32UC64L3U__) || defined(__AVR32_UC64L3U__)  || \
	defined(__AT32UC128L3U__) || defined(__AVR32_UC128L3U__) || \
	defined(__AT32UC256L3U__) || defined(__AVR32_UC256L3U__)
#define UC3L3 (1)
#else
#define UC3L3 (0)
#endif

#if defined(__AT32UC64L4U__) || defined(__AVR32_UC64L4U__)  || \
	defined(__AT32UC128L4U__) || defined(__AVR32_UC128L4U__) || \
	defined(__AT32UC256L4U__) || defined(__AVR32_UC256L4U__)
#define UC3L4 (1)
#else
#define UC3L4 (0)
#endif

#define UC3L3_L4 (UC3L3 || UC3L4)
/** @} */

/**
 * \name AVR UC3 families
 * @{
 */
/** AVR UC3 A family */
#define UC3A (UC3A0 || UC3A1 || UC3A3 || UC3A4)

/** AVR UC3 B family */
#define UC3B (UC3B0 || UC3B1)

/** AVR UC3 C family */
#define UC3C (UC3C0 || UC3C1 || UC3C2)

/** AVR UC3 D family */
#define UC3D (UC3D3 || UC3D4)

/** AVR UC3 L family */
#define UC3L (UC3L0 || UC3L0128 || UC3L0256 || UC3L3_L4)
/** @} */

/** AVR UC3 product line */
#define UC3  (UC3A || UC3B || UC3C || UC3D || UC3L)

/** @} */

/**
 * \defgroup xmega_part_macros_group AVR XMEGA parts
 * @{
 */

/**
 * \name AVR XMEGA A series
 * @{
 */
#if defined(__ATxmega64A1__) || defined(__AVR_ATxmega64A1__)  || \
	defined(__ATxmega128A1__) || defined(__AVR_ATxmega128A1__)
#define XMEGA_A1 (1)
#else
#define XMEGA_A1 (0)
#endif

#if defined(__ATxmega64A3__) || defined(__AVR_ATxmega64A3__)  || \
	defined(__ATxmega128A3__) || defined(__AVR_ATxmega128A3__) || \
	defined(__ATxmega192A3__) || defined(__AVR_ATxmega192A3__) || \
	defined(__ATxmega256A3__) || defined(__AVR_ATxmega256A3__)
#define XMEGA_A3 (1)
#else
#define XMEGA_A3 (0)
#endif

#if defined(__ATxmega256A3B__) || defined(__AVR_ATxmega256A3B__)
#define XMEGA_A3B (1)
#else
#define XMEGA_A3B (0)
#endif

#if defined(__ATxmega16A4__) || defined(__AVR_ATxmega16A4__) || \
	defined(__ATxmega32A4__) || defined(__AVR_ATxmega32A4__)
#define XMEGA_A4 (1)
#else
#define XMEGA_A4 (0)
#endif
/** @} */

/**
 * \name AVR XMEGA AU series
 * @{
 */
#if defined(__ATxmega64A1U__) || defined(__AVR_ATxmega64A1U__)  || \
	defined(__ATxmega128A1U__) || defined(__AVR_ATxmega128A1U__)
#define XMEGA_A1U (1)
#else
#define XMEGA_A1U (0)
#endif

#if defined(__ATxmega64A3U__) || defined(__AVR_ATxmega64A3U__)  || \
	defined(__ATxmega128A3U__) || defined(__AVR_ATxmega128A3U__) || \
	defined(__ATxmega192A3U__) || defined(__AVR_ATxmega192A3U__) || \
	defined(__ATxmega256A3U__) || defined(__AVR_ATxmega256A3U__)
#define XMEGA_A3U (1)
#else
#define XMEGA_A3U (0)
#endif

#if defined(__ATxmega256A3BU__) || defined(__AVR_ATxmega256A3BU__)
#define XMEGA_A3BU (1)
#else
#define XMEGA_A3BU (0)
#endif

#if defined(__ATxmega16A4U__) || defined(__AVR_ATxmega16A4U__)  || \
	defined(__ATxmega32A4U__) || defined(__AVR_ATxmega32A4U__)  || \
	defined(__ATxmega64A4U__) || defined(__AVR_ATxmega64A4U__)  || \
	defined(__ATxmega128A4U__) || defined(__AVR_ATxmega128A4U__)
#define XMEGA_A4U (1)
#else
#define XMEGA_A4U (0)
#endif
/** @} */

/**
 * \name AVR XMEGA B series
 * @{
 */
#if defined(__ATxmega64B1__) || defined(__AVR_ATxmega64B1__)  || \
	defined(__ATxmega128B1__) || defined(__AVR_ATxmega128B1__)
#define XMEGA_B1  (1)
#else
#define XMEGA_B1  (0)
#endif

#if defined(__ATxmega64B3__) || defined(__AVR_ATxmega64B3__)  || \
	defined(__ATxmega128B3__) || defined(__AVR_ATxmega128B3__)
#define XMEGA_B3  (1)
#else
#define XMEGA_B3  (0)
#endif
/** @} */

/**
 * \name AVR XMEGA C series
 * @{
 */
#if defined(__ATxmega384C3__) || defined(__AVR_ATxmega384C3__)  || \
	defined(__ATxmega256C3__) || defined(__AVR_ATxmega256C3__)  || \
	defined(__ATxmega192C3__) || defined(__AVR_ATxmega192C3__)  || \
	defined(__ATxmega128C3__) || defined(__AVR_ATxmega128C3__)  || \
	defined(__ATxmega64C3__) || defined(__AVR_ATxmega64C3__)   || \
	defined(__ATxmega32C3__) || defined(__AVR_ATxmega32C3__)
#define XMEGA_C3 (1)
#else
#define XMEGA_C3 (0)
#endif

#if defined(__ATxmega32C4__) || defined(__AVR_ATxmega32C4__)  || \
	defined(__ATxmega16C4__) || defined(__AVR_ATxmega16C4__)
#define XMEGA_C4 (1)
#else
#define XMEGA_C4 (0)
#endif
/** @} */

/**
 * \name AVR XMEGA D series
 * @{
 */
#if defined(__ATxmega32D3__) || defined(__AVR_ATxmega32D3__)  || \
	defined(__ATxmega64D3__) || defined(__AVR_ATxmega64D3__)  || \
	defined(__ATxmega128D3__) || defined(__AVR_ATxmega128D3__) || \
	defined(__ATxmega192D3__) || defined(__AVR_ATxmega192D3__) || \
	defined(__ATxmega256D3__) || defined(__AVR_ATxmega256D3__) || \
	defined(__ATxmega384D3__) || defined(__AVR_ATxmega384D3__)
#define XMEGA_D3 (1)
#else
#define XMEGA_D3 (0)
#endif

#if defined(__ATxmega16D4__) || defined(__AVR_ATxmega16D4__)  || \
	defined(__ATxmega32D4__) || defined(__AVR_ATxmega32D4__)  || \
	defined(__ATxmega64D4__) || defined(__AVR_ATxmega64D4__)  || \
	defined(__ATxmega128D4__) || defined(__AVR_ATxmega128D4__)
#define XMEGA_D4 (1)
#else
#define XMEGA_D4 (0)
#endif
/** @} */

/**
 * \name AVR XMEGA E series
 * @{
 */
#if defined(__ATxmega8E5__) || defined(__AVR_ATxmega8E5__)   || \
	defined(__ATxmega16E5__) || defined(__AVR_ATxmega16E5__)  || \
	defined(__ATxmega32E5__) || defined(__AVR_ATxmega32E5__)
#define XMEGA_E5 (1)
#else
#define XMEGA_E5 (0)
#endif
/** @} */


/**
 * \name AVR XMEGA families
 * @{
 */
/** AVR XMEGA A family */
#define XMEGA_A (XMEGA_A1 || XMEGA_A3 || XMEGA_A3B || XMEGA_A4)

/** AVR XMEGA AU family */
#define XMEGA_AU (XMEGA_A1U || XMEGA_A3U || XMEGA_A3BU || XMEGA_A4U)

/** AVR XMEGA B family */
#define XMEGA_B (XMEGA_B1 || XMEGA_B3)

/** AVR XMEGA C family */
#define XMEGA_C (XMEGA_C3 || XMEGA_C4)

/** AVR XMEGA D family */
#define XMEGA_D (XMEGA_D3 || XMEGA_D4)

/** AVR XMEGA E family */
#define XMEGA_E (XMEGA_E5)
/** @} */


/** AVR XMEGA product line */
#define XMEGA (XMEGA_A || XMEGA_AU || XMEGA_B || XMEGA_C || XMEGA_D || XMEGA_E)

/** @} */

/**
 * \defgroup mega_part_macros_group megaAVR parts
 *
 * \note These megaAVR groupings are based on the groups in AVR Libc for the
 * part header files. They are not names of official megaAVR device series or
 * families.
 *
 * @{
 */

/**
 * \name ATmegaxx0/xx1 subgroups
 * @{
 */
#if defined(__ATmega640__) || defined(__AVR_ATmega640__)  || \
	defined(__ATmega1280__) || defined(__AVR_ATmega1280__) || \
	defined(__ATmega2560__) || defined(__AVR_ATmega2560__)
#define MEGA_XX0 (1)
#else
#define MEGA_XX0 (0)
#endif

#if defined(__ATmega1281__) || defined(__AVR_ATmega1281__) || \
	defined(__ATmega2561__) || defined(__AVR_ATmega2561__)
#define MEGA_XX1 (1)
#else
#define MEGA_XX1 (0)
#endif

/** @} */

/**
 * \name megaAVR groups
 * @{
 */
/** ATmegaxx0/xx1 group */
#define MEGA_XX0_1 (MEGA_XX0 || MEGA_XX1)

/** ATmegaxx4 group */
#if defined(__ATmega164A__) || defined(__AVR_ATmega164A__)  || \
	defined(__ATmega164PA__) || defined(__AVR_ATmega164PA__) || \
	defined(__ATmega324A__) || defined(__AVR_ATmega324A__)  || \
	defined(__ATmega324PA__) || defined(__AVR_ATmega324PA__) || \
	defined(__ATmega644__) || defined(__AVR_ATmega644__)   || \
	defined(__ATmega644A__) || defined(__AVR_ATmega644A__)  || \
	defined(__ATmega644PA__) || defined(__AVR_ATmega644PA__) || \
	defined(__ATmega1284P__) || defined(__AVR_ATmega1284P__)   || \
	defined(__ATmega128RFA1__) || defined(__AVR_ATmega128RFA1__)
#define MEGA_XX4 (1)
#else
#define MEGA_XX4 (0)
#endif

/** ATmegaxx4 group */
#if defined(__ATmega164A__) || defined(__AVR_ATmega164A__)  || \
	defined(__ATmega164PA__) || defined(__AVR_ATmega164PA__) || \
	defined(__ATmega324A__) || defined(__AVR_ATmega324A__)  || \
	defined(__ATmega324PA__) || defined(__AVR_ATmega324PA__) || \
	defined(__ATmega644A__) || defined(__AVR_ATmega644A__)  || \
	defined(__ATmega644PA__) || defined(__AVR_ATmega644PA__) || \
	defined(__ATmega1284P__) || defined(__AVR_ATmega1284P__)
#define MEGA_XX4_A (1)
#else
#define MEGA_XX4_A (0)
#endif

/** ATmegaxx8 group */
#if defined(__ATmega48__) || defined(__AVR_ATmega48__)    || \
	defined(__ATmega48A__) || defined(__AVR_ATmega48A__)   || \
	defined(__ATmega48PA__) || defined(__AVR_ATmega48PA__)  || \
	defined(__ATmega88__) || defined(__AVR_ATmega88__)    || \
	defined(__ATmega88A__) || defined(__AVR_ATmega88A__)   || \
	defined(__ATmega88PA__) || defined(__AVR_ATmega88PA__)  || \
	defined(__ATmega168__) || defined(__AVR_ATmega168__)   || \
	defined(__ATmega168A__) || defined(__AVR_ATmega168A__)  || \
	defined(__ATmega168PA__) || defined(__AVR_ATmega168PA__) || \
	defined(__ATmega328__) || defined(__AVR_ATmega328__)   || \
	defined(__ATmega328P__) || defined(__AVR_ATmega328P__)
#define MEGA_XX8 (1)
#else
#define MEGA_XX8 (0)
#endif

/** ATmegaxx8A/P/PA group */
#if defined(__ATmega48A__) || defined(__AVR_ATmega48A__)   || \
	defined(__ATmega48PA__) || defined(__AVR_ATmega48PA__)  || \
	defined(__ATmega88A__) || defined(__AVR_ATmega88A__)   || \
	defined(__ATmega88PA__) || defined(__AVR_ATmega88PA__)  || \
	defined(__ATmega168A__) || defined(__AVR_ATmega168A__)  || \
	defined(__ATmega168PA__) || defined(__AVR_ATmega168PA__) || \
	defined(__ATmega328P__) || defined(__AVR_ATmega328P__)
#define MEGA_XX8_A (1)
#else
#define MEGA_XX8_A (0)
#endif

/** ATmegaxx group */
#if defined(__ATmega16__) || defined(__AVR_ATmega16__)   || \
	defined(__ATmega16A__) || defined(__AVR_ATmega16A__)  || \
	defined(__ATmega32__) || defined(__AVR_ATmega32__)   || \
	defined(__ATmega32A__) || defined(__AVR_ATmega32A__)  || \
	defined(__ATmega64__) || defined(__AVR_ATmega64__)   || \
	defined(__ATmega64A__) || defined(__AVR_ATmega64A__)  || \
	defined(__ATmega128__) || defined(__AVR_ATmega128__)  || \
	defined(__ATmega128A__) || defined(__AVR_ATmega128A__)
#define MEGA_XX (1)
#else
#define MEGA_XX (0)
#endif

/** ATmegaxxA/P/PA group */
#if defined(__ATmega16A__) || defined(__AVR_ATmega16A__)  || \
	defined(__ATmega32A__) || defined(__AVR_ATmega32A__)  || \
	defined(__ATmega64A__) || defined(__AVR_ATmega64A__)  || \
	defined(__ATmega128A__) || defined(__AVR_ATmega128A__)
#define MEGA_XX_A (1)
#else
#define MEGA_XX_A (0)
#endif

/** ATmegaxxRFA1 group */
#if (defined(__ATmega128RFA1__) || defined(__AVR_ATmega128RFA1__))
#define MEGA_RFA1 (1)
#else
#define MEGA_RFA1 (0)
#endif

#if defined(__ATmega64RFR2__) || defined(__AVR_ATmega64RFR2__)   || \
	defined(__ATmega128RFR2__) || defined(__AVR_ATmega128RFR2__)  || \
	defined(__ATmega256RFR2__) || defined(__AVR_ATmega256RFR2__)  || \
	defined(__ATmega644RFR2__) || defined(__AVR_ATmega644RFR2__)  || \
	defined(__ATmega1284RFR2__) || defined(__AVR_ATmega1284RFR2__) || \
	defined(__ATmega2564RFR2__) || defined(__AVR_ATmega2564RFR2__)
#define MEGA_RFR2 (1)
#else
#define MEGA_RFA1 (0)
#endif

/** ATmegaxxRFxx group */
#define MEGA_RF (MEGA_RFA1 || MEGA_RFR2)

/**
 * \name ATmegaxx_un0/un1/un2 subgroups
 * @{
 */
#if defined(__ATmega16__) || defined(__AVR_ATmega16__)    || \
	defined(__ATmega16A__) || defined(__AVR_ATmega16A__)   || \
	defined(__ATmega32__) || defined(__AVR_ATmega32__)    || \
	defined(__ATmega32A__) || defined(__AVR_ATmega32A__)
#define MEGA_XX_UN0 (1)
#else
#define MEGA_XX_UN0 (0)
#endif

/** ATmegaxx group without power reduction and
 *  And interrupt sense register.
 */
#if defined(__ATmega64__) || defined(__AVR_ATmega64__)    || \
	defined(__ATmega64A__) || defined(__AVR_ATmega64A__)   || \
	defined(__ATmega128__) || defined(__AVR_ATmega128__)   || \
	defined(__ATmega128A__) || defined(__AVR_ATmega128A__)
#define MEGA_XX_UN1 (1)
#else
#define MEGA_XX_UN1 (0)
#endif

/** ATmegaxx group without power reduction and
 *  And interrupt sense register.
 */
#if defined(__ATmega169P__) || defined(__AVR_ATmega169P__)  || \
	defined(__ATmega169PA__) || defined(__AVR_ATmega169PA__) || \
	defined(__ATmega329P__) || defined(__AVR_ATmega329P__)  || \
	defined(__ATmega329PA__) || defined(__AVR_ATmega329PA__)
#define MEGA_XX_UN2 (1)
#else
#define MEGA_XX_UN2 (0)
#endif

/** Devices added to complete megaAVR offering.
 *  Please do not use this group symbol as it is not intended
 *  to be permanent: the devices should be regrouped.
 */
#if defined(__AT90CAN128__) || defined(__AVR_AT90CAN128__)     || \
	defined(__AT90CAN32__) || defined(__AVR_AT90CAN32__)      || \
	defined(__AT90CAN64__) || defined(__AVR_AT90CAN64__)      || \
	defined(__AT90PWM1__) || defined(__AVR_AT90PWM1__)       || \
	defined(__AT90PWM216__) || defined(__AVR_AT90PWM216__)     || \
	defined(__AT90PWM2B__) || defined(__AVR_AT90PWM2B__)      || \
	defined(__AT90PWM316__) || defined(__AVR_AT90PWM316__)     || \
	defined(__AT90PWM3B__) || defined(__AVR_AT90PWM3B__)      || \
	defined(__AT90PWM81__) || defined(__AVR_AT90PWM81__)      || \
	defined(__AT90USB1286__) || defined(__AVR_AT90USB1286__)    || \
	defined(__AT90USB1287__) || defined(__AVR_AT90USB1287__)    || \
	defined(__AT90USB162__) || defined(__AVR_AT90USB162__)     || \
	defined(__AT90USB646__) || defined(__AVR_AT90USB646__)     || \
	defined(__AT90USB647__) || defined(__AVR_AT90USB647__)     || \
	defined(__AT90USB82__) || defined(__AVR_AT90USB82__)      || \
	defined(__ATmega1284__) || defined(__AVR_ATmega1284__)     || \
	defined(__ATmega162__) || defined(__AVR_ATmega162__)      || \
	defined(__ATmega164P__) || defined(__AVR_ATmega164P__)     || \
	defined(__ATmega165A__) || defined(__AVR_ATmega165A__)     || \
	defined(__ATmega165P__) || defined(__AVR_ATmega165P__)     || \
	defined(__ATmega165PA__) || defined(__AVR_ATmega165PA__)    || \
	defined(__ATmega168P__) || defined(__AVR_ATmega168P__)     || \
	defined(__ATmega169A__) || defined(__AVR_ATmega169A__)     || \
	defined(__ATmega16M1__) || defined(__AVR_ATmega16M1__)     || \
	defined(__ATmega16U2__) || defined(__AVR_ATmega16U2__)     || \
	defined(__ATmega16U4__) || defined(__AVR_ATmega16U4__)     || \
	defined(__ATmega256RFA2__) || defined(__AVR_ATmega256RFA2__)  || \
	defined(__ATmega324P__) || defined(__AVR_ATmega324P__)     || \
	defined(__ATmega325__) || defined(__AVR_ATmega325__)      || \
	defined(__ATmega3250__) || defined(__AVR_ATmega3250__)     || \
	defined(__ATmega3250A__) || defined(__AVR_ATmega3250A__)    || \
	defined(__ATmega3250P__) || defined(__AVR_ATmega3250P__)    || \
	defined(__ATmega3250PA__) || defined(__AVR_ATmega3250PA__)   || \
	defined(__ATmega325A__) || defined(__AVR_ATmega325A__)     || \
	defined(__ATmega325P__) || defined(__AVR_ATmega325P__)     || \
	defined(__ATmega325PA__) || defined(__AVR_ATmega325PA__)    || \
	defined(__ATmega329__) || defined(__AVR_ATmega329__)      || \
	defined(__ATmega3290__) || defined(__AVR_ATmega3290__)     || \
	defined(__ATmega3290A__) || defined(__AVR_ATmega3290A__)    || \
	defined(__ATmega3290P__) || defined(__AVR_ATmega3290P__)    || \
	defined(__ATmega3290PA__) || defined(__AVR_ATmega3290PA__)   || \
	defined(__ATmega329A__) || defined(__AVR_ATmega329A__)     || \
	defined(__ATmega32M1__) || defined(__AVR_ATmega32M1__)     || \
	defined(__ATmega32U2__) || defined(__AVR_ATmega32U2__)     || \
	defined(__ATmega32U4__) || defined(__AVR_ATmega32U4__)     || \
	defined(__ATmega48P__) || defined(__AVR_ATmega48P__)      || \
	defined(__ATmega644P__) || defined(__AVR_ATmega644P__)     || \
	defined(__ATmega645__) || defined(__AVR_ATmega645__)      || \
	defined(__ATmega6450__) || defined(__AVR_ATmega6450__)     || \
	defined(__ATmega6450A__) || defined(__AVR_ATmega6450A__)    || \
	defined(__ATmega6450P__) || defined(__AVR_ATmega6450P__)    || \
	defined(__ATmega645A__) || defined(__AVR_ATmega645A__)     || \
	defined(__ATmega645P__) || defined(__AVR_ATmega645P__)     || \
	defined(__ATmega649__) || defined(__AVR_ATmega649__)      || \
	defined(__ATmega6490__) || defined(__AVR_ATmega6490__)     || \
	defined(__ATmega6490A__) || defined(__AVR_ATmega6490A__)    || \
	defined(__ATmega6490P__) || defined(__AVR_ATmega6490P__)    || \
	defined(__ATmega649A__) || defined(__AVR_ATmega649A__)     || \
	defined(__ATmega649P__) || defined(__AVR_ATmega649P__)     || \
	defined(__ATmega64M1__) || defined(__AVR_ATmega64M1__)     || \
	defined(__ATmega64RFA2__) || defined(__AVR_ATmega64RFA2__)   || \
	defined(__ATmega8__) || defined(__AVR_ATmega8__)        || \
	defined(__ATmega8515__) || defined(__AVR_ATmega8515__)     || \
	defined(__ATmega8535__) || defined(__AVR_ATmega8535__)     || \
	defined(__ATmega88P__) || defined(__AVR_ATmega88P__)      || \
	defined(__ATmega8A__) || defined(__AVR_ATmega8A__)       || \
	defined(__ATmega8U2__) || defined(__AVR_ATmega8U2__)
#define MEGA_UNCATEGORIZED (1)
#else
#define MEGA_UNCATEGORIZED (0)
#endif

/** Unspecified group */
#define MEGA_UNSPECIFIED (MEGA_XX_UN0 || MEGA_XX_UN1 || MEGA_XX_UN2 || \
	MEGA_UNCATEGORIZED)

/** @} */

/** megaAVR product line */
#define MEGA (MEGA_XX0_1 || MEGA_XX4 || MEGA_XX8 || MEGA_XX || MEGA_RF || \
	MEGA_UNSPECIFIED)

/** @} */

/**
 * \defgroup tiny_part_macros_group tinyAVR parts
 *
 * @{
 */

/**
 * \name tinyAVR groups
 * @{
 */

/** Devices added to complete tinyAVR offering.
 *  Please do not use this group symbol as it is not intended
 *  to be permanent: the devices should be regrouped.
 */
#if defined(__ATtiny10__) || defined(__AVR_ATtiny10__)    || \
	defined(__ATtiny13__) || defined(__AVR_ATtiny13__)    || \
	defined(__ATtiny13A__) || defined(__AVR_ATtiny13A__)   || \
	defined(__ATtiny1634__) || defined(__AVR_ATtiny1634__)  || \
	defined(__ATtiny167__) || defined(__AVR_ATtiny167__)   || \
	defined(__ATtiny20__) || defined(__AVR_ATtiny20__)    || \
	defined(__ATtiny2313__) || defined(__AVR_ATtiny2313__)  || \
	defined(__ATtiny2313A__) || defined(__AVR_ATtiny2313A__) || \
	defined(__ATtiny24__) || defined(__AVR_ATtiny24__)    || \
	defined(__ATtiny24A__) || defined(__AVR_ATtiny24A__)   || \
	defined(__ATtiny25__) || defined(__AVR_ATtiny25__)    || \
	defined(__ATtiny26__) || defined(__AVR_ATtiny26__)    || \
	defined(__ATtiny261__) || defined(__AVR_ATtiny261__)   || \
	defined(__ATtiny261A__) || defined(__AVR_ATtiny261A__)  || \
	defined(__ATtiny4__) || defined(__AVR_ATtiny4__)     || \
	defined(__ATtiny40__) || defined(__AVR_ATtiny40__)    || \
	defined(__ATtiny4313__) || defined(__AVR_ATtiny4313__)  || \
	defined(__ATtiny43U__) || defined(__AVR_ATtiny43U__)   || \
	defined(__ATtiny44__) || defined(__AVR_ATtiny44__)    || \
	defined(__ATtiny44A__) || defined(__AVR_ATtiny44A__)   || \
	defined(__ATtiny45__) || defined(__AVR_ATtiny45__)    || \
	defined(__ATtiny461__) || defined(__AVR_ATtiny461__)   || \
	defined(__ATtiny461A__) || defined(__AVR_ATtiny461A__)  || \
	defined(__ATtiny48__) || defined(__AVR_ATtiny48__)    || \
	defined(__ATtiny5__) || defined(__AVR_ATtiny5__)     || \
	defined(__ATtiny828__) || defined(__AVR_ATtiny828__)   || \
	defined(__ATtiny84__) || defined(__AVR_ATtiny84__)    || \
	defined(__ATtiny84A__) || defined(__AVR_ATtiny84A__)   || \
	defined(__ATtiny85__) || defined(__AVR_ATtiny85__)    || \
	defined(__ATtiny861__) || defined(__AVR_ATtiny861__)   || \
	defined(__ATtiny861A__) || defined(__AVR_ATtiny861A__)  || \
	defined(__ATtiny87__) || defined(__AVR_ATtiny87__)    || \
	defined(__ATtiny88__) || defined(__AVR_ATtiny88__)    || \
	defined(__ATtiny9__) || defined(__AVR_ATtiny9__)
#define TINY_UNCATEGORIZED (1)
#else
#define TINY_UNCATEGORIZED (0)
#endif
/** @} */

/** tinyAVR product line */
#define TINY (TINY_UNCATEGORIZED)

/** @} */

/**
 * \defgroup sam_part_macros_group SAM parts
 * @{
 */

/**
 * \name SAM3S series
 * @{
 */
#if defined(__SAM3S1A__) ||	\
	defined(__SAM3S1B__) ||	\
	defined(__SAM3S1C__)
#define SAM3S1 (1)
#else
#define SAM3S1 (0)
#endif

#if defined(__SAM3S2A__) ||	\
	defined(__SAM3S2B__) ||	\
	defined(__SAM3S2C__)
#define SAM3S2 (1)
#else
#define SAM3S2 (0)
#endif

#if defined(__SAM3S4A__) ||	\
	defined(__SAM3S4B__) ||	\
	defined(__SAM3S4C__)
#define SAM3S4 (1)
#else
#define SAM3S4 (0)
#endif

#if defined(__SAM3S8B__) ||	\
	defined(__SAM3S8C__)
#define SAM3S8 (1)
#else
#define SAM3S8 (0)
#endif

#if defined(__SAM3SD8B__) || \
	defined(__SAM3SD8C__)
#define SAM3SD8 (1)
#else
#define SAM3SD8 (0)
#endif
/** @} */

/**
 * \name SAM3U series
 * @{
 */
#if defined(__SAM3U1C__) ||	\
	defined(__SAM3U1E__)
#define SAM3U1 (1)
#else
#define SAM3U1 (0)
#endif

#if defined(__SAM3U2C__) ||	\
	defined(__SAM3U2E__)
#define SAM3U2 (1)
#else
#define SAM3U2 (0)
#endif

#if defined(__SAM3U4C__) ||	\
	defined(__SAM3U4E__)
#define SAM3U4 (1)
#else
#define SAM3U4 (0)
#endif

/** @} */

/**
 * \name SAM3N series
 * @{
 */
#if defined(__SAM3N00A__) ||	\
	defined(__SAM3N00B__)
#define SAM3N00 (1)
#else
#define SAM3N00 (0)
#endif

#if defined(__SAM3N0A__) ||	\
	defined(__SAM3N0B__) ||	\
	defined(__SAM3N0C__)
#define SAM3N0 (1)
#else
#define SAM3N0 (0)
#endif

#if defined(__SAM3N1A__) ||	\
	defined(__SAM3N1B__) ||	\
	defined(__SAM3N1C__)
#define SAM3N1 (1)
#else
#define SAM3N1 (0)
#endif

#if defined(__SAM3N2A__) ||	\
	defined(__SAM3N2B__) ||	\
	defined(__SAM3N2C__)
#define SAM3N2 (1)
#else
#define SAM3N2 (0)
#endif

#if defined(__SAM3N4A__) ||	\
	defined(__SAM3N4B__) ||	\
	defined(__SAM3N4C__)
#define SAM3N4 (1)
#else
#define SAM3N4 (0)
#endif
/** @} */

/**
 * \name SAM3X series
 * @{
 */
#if defined(__SAM3X4C__) ||	\
	defined(__SAM3X4E__)
#define SAM3X4 (1)
#else
#define SAM3X4 (0)
#endif

#if defined(__SAM3X8C__) ||	\
	defined(__SAM3X8E__) ||	\
	defined(__SAM3X8H__)
#define SAM3X8 (1)
#else
#define SAM3X8 (0)
#endif
/** @} */

/**
 * \name SAM3A series
 * @{
 */
#if defined(__SAM3A4C__)
#define SAM3A4 (1)
#else
#define SAM3A4 (0)
#endif

#if defined(__SAM3A8C__)
#define SAM3A8 (1)
#else
#define SAM3A8 (0)
#endif
/** @} */

/**
 * \name SAM4S series
 * @{
 */
#if defined(__SAM4S2A__) || \
 	defined(__SAM4S2B__) || \
 	defined(__SAM4S2C__)
#define SAM4S2 (1)
#else
#define SAM4S2 (0)
#endif

#if defined(__SAM4S4A__) || \
 	defined(__SAM4S4B__) || \
 	defined(__SAM4S4C__)
#define SAM4S4 (1)
#else
#define SAM4S4 (0)
#endif

#if defined(__SAM4S8B__) ||	\
	defined(__SAM4S8C__)
#define SAM4S8 (1)
#else
#define SAM4S8 (0)
#endif

#if defined(__SAM4S16B__) || \
	defined(__SAM4S16C__)
#define SAM4S16 (1)
#else
#define SAM4S16 (0)
#endif

#if defined(__SAM4SA16B__) || \
	defined(__SAM4SA16C__)
#define SAM4SA16 (1)
#else
#define SAM4SA16 (0)
#endif

#if defined(__SAM4SD16B__) || \
	defined(__SAM4SD16C__)
#define SAM4SD16 (1)
#else
#define SAM4SD16 (0)
#endif

#if defined(__SAM4SD32B__) || \
	defined(__SAM4SD32C__)
#define SAM4SD32 (1)
#else
#define SAM4SD32 (0)
#endif
/** @} */

/**
 * \name SAM4L series
 * @{
 */
#if defined(__SAM4LS2A__) || \
	defined(__SAM4LS2B__) || \
	defined(__SAM4LS2C__) || \
	defined(__SAM4LS4A__) || \
	defined(__SAM4LS4B__) || \
	defined(__SAM4LS4C__) || \
	defined(__SAM4LS8A__) || \
	defined(__SAM4LS8B__) || \
	defined(__SAM4LS8C__)
#define SAM4LS (1)
#else
#define SAM4LS (0)
#endif

#if defined(__SAM4LC2A__) || \
	defined(__SAM4LC2B__) || \
	defined(__SAM4LC2C__) || \
	defined(__SAM4LC4A__) || \
	defined(__SAM4LC4B__) || \
	defined(__SAM4LC4C__) || \
	defined(__SAM4LC8A__) || \
	defined(__SAM4LC8B__) || \
	defined(__SAM4LC8C__)
#define SAM4LC (1)
#else
#define SAM4LC (0)
#endif
/** @} */

/**
 * \name SAMD51A series
 * @{
 */
#if defined(__SAMD51P19A__)
#define SAMD51A (1)
#else
#define SAMD51A (0)
#endif
/** @} */

/**
 * \name SAMD20 series
 * @{
 */
#if defined(__SAMD20J14__) || \
	defined(__SAMD20J15__) || \
	defined(__SAMD20J16__) || \
	defined(__SAMD20J17__) || \
	defined(__SAMD20J18__)
#define SAMD20J (1)
#else
#define SAMD20J (0)
#endif

#if defined(__SAMD20G14__)  || \
	defined(__SAMD20G15__)  || \
	defined(__SAMD20G16__)  || \
	defined(__SAMD20G17__)  || \
	defined(__SAMD20G17U__) || \
	defined(__SAMD20G18__)  || \
	defined(__SAMD20G18U__)
#define SAMD20G (1)
#else
#define SAMD20G (0)
#endif

#if defined(__SAMD20E14__) || \
	defined(__SAMD20E15__) || \
	defined(__SAMD20E16__) || \
	defined(__SAMD20E17__) || \
	defined(__SAMD20E18__) || \
	defined(__SAMD20E1F__)
#define SAMD20E (1)
#else
#define SAMD20E (0)
#endif
/** @} */

/**
 * \name SAMD21 series
 * @{
 */
#if defined(__SAMD21J15A__) || \
	defined(__SAMD21J16A__) || \
	defined(__SAMD21J17A__) || \
	defined(__SAMD21J18A__)
#define SAMD21J (1)	
#else
#define SAMD21J (0)
#endif

#if defined(__SAMD21G15A__) || \
	defined(__SAMD21G16A__) || \
	defined(__SAMD21G17A__) || \
	defined(__SAMD21G18A__)
#define SAMD21G (1)
#else
#define SAMD21G (0)
#endif

#if defined(__SAMD21E15A__) || \
	defined(__SAMD21E16A__) || \
	defined(__SAMD21E17A__) || \
	defined(__SAMD21E18A__)
#define SAMD21E (1)
#else
#define SAMD21E (0)
#endif
/** @} */

/**
 * \name SAMR21 series
 * @{
 */
#if defined(__SAMR21G16A__) || \
	defined(__SAMR21G17A__) || \
	defined(__SAMR21G18A__)
#define SAMR21G (1)
#else
#define SAMR21G (0)
#endif

#if defined(__SAMR21E16A__) || \
	defined(__SAMR21E17A__) || \
	defined(__SAMR21E18A__)
#define SAMR21E (1)
#else
#define SAMR21E (0)
#endif
/** @} */

/**
 * \name SAMD10 series
 * @{
 */
#if defined(__SAMD10C12A__) || \
	defined(__SAMD10C13A__) || \
	defined(__SAMD10C14A__)
#define SAMD10C (1)
#else
#define SAMD10C (0)
#endif

#if defined(__SAMD10D12AS__) || \
	defined(__SAMD10D13AS__) || \
	defined(__SAMD10D14AS__)
#define SAMD10DS (1)
#else
#define SAMD10DS (0)
#endif

#if defined(__SAMD10D12AM__) || \
	defined(__SAMD10D13AM__) || \
	defined(__SAMD10D14AM__)
#define SAMD10DM (1)
#else
#define SAMD10DM (0)
#endif
/** @} */

/**
 * \name SAMD11 series
 * @{
 */
#if defined(__SAMD11C14A__)
#define SAMD11C (1)
#else
#define SAMD11C (0)
#endif

#if defined(__SAMD11D14AS__)
#define SAMD11DS (1)
#else
#define SAMD11DS (0)
#endif

#if defined(__SAMD11D14AM__)
#define SAMD11DM (1)
#else
#define SAMD11DM (0)
#endif
/** @} */

/**
 * \name SAML21 series
 * @{
 */
#if defined(__SAML21E15A__) || \
	defined(__SAML21E16A__) || \
	defined(__SAML21E17A__) || \
	defined(__SAML21E18A__)
#define SAML21E (1)
#else
#define SAML21E (0)
#endif

#if defined(__SAML21G16A__) || \
	defined(__SAML21G17A__) || \
	defined(__SAML21G18A__)
#define SAML21G (1)
#else
#define SAML21G (0)
#endif

#if defined(__SAML21J16A__) || \
	defined(__SAML21J17A__) || \
	defined(__SAML21J18A__)
#define SAML21J (1)
#else
#define SAML21J (0)
#endif
/** @} */

/**
 * \name SAM4E series
 * @{
 */
#if defined(__SAM4E8C__) || \
	defined(__SAM4E8E__)
#define SAM4E8 (1)
#else
#define SAM4E8 (0)
#endif

#if defined(__SAM4E16C__) || \
	defined(__SAM4E16E__)
#define SAM4E16 (1)
#else
#define SAM4E16 (0)
#endif
/** @} */

/**
 * \name SAM4N series
 * @{
 */
#if defined(__SAM4N8A__) || \
	defined(__SAM4N8B__) || \
	defined(__SAM4N8C__)
#define SAM4N8 (1)
#else
#define SAM4N8 (0)
#endif

#if defined(__SAM4N16B__) || \
	defined(__SAM4N16C__)
#define SAM4N16 (1)
#else
#define SAM4N16 (0)
#endif
/** @} */

/**
 * \name SAM4C series
 * @{
 */
#if defined(__SAM4C8C_0__)
#define SAM4C8_0 (1)
#else
#define SAM4C8_0 (0)
#endif

#if defined(__SAM4C8C_1__)
#define SAM4C8_1 (1)
#else
#define SAM4C8_1 (0)
#endif

#define SAM4C8 (SAM4C8_0 || SAM4C8_1)

#if defined(__SAM4C16C_0__)
#define SAM4C16_0 (1)
#else
#define SAM4C16_0 (0)
#endif

#if defined(__SAM4C16C_1__)
#define SAM4C16_1 (1)
#else
#define SAM4C16_1 (0)
#endif

#define SAM4C16 (SAM4C16_0 || SAM4C16_1)

#if defined(__SAM4C32C_0__) ||\
	defined(__SAM4C32E_0__)
#define SAM4C32_0 (1)
#else
#define SAM4C32_0 (0)
#endif

#if defined(__SAM4C32C_1__) ||\
	defined(__SAM4C32E_1__)
#define SAM4C32_1 (1)
#else
#define SAM4C32_1 (0)
#endif

#define SAM4C32 (SAM4C32_0 || SAM4C32_1)

/** @} */

/**
 * \name SAM4CM series
 * @{
 */
#if defined(__SAM4CMP8C_0__)
#define SAM4CMP8_0 (1)
#else
#define SAM4CMP8_0 (0)
#endif

#if defined(__SAM4CMP8C_1__)
#define SAM4CMP8_1 (1)
#else
#define SAM4CMP8_1 (0)
#endif

#define SAM4CMP8 (SAM4CMP8_0 || SAM4CMP8_1)

#if defined(__SAM4CMP16C_0__)
#define SAM4CMP16_0 (1)
#else
#define SAM4CMP16_0 (0)
#endif

#if defined(__SAM4CMP16C_1__)
#define SAM4CMP16_1 (1)
#else
#define SAM4CMP16_1 (0)
#endif

#define SAM4CMP16 (SAM4CMP16_0 || SAM4CMP16_1)

#if defined(__SAM4CMP32C_0__)
#define SAM4CMP32_0 (1)
#else
#define SAM4CMP32_0 (0)
#endif

#if defined(__SAM4CMP32C_1__)
#define SAM4CMP32_1 (1)
#else
#define SAM4CMP32_1 (0)
#endif

#define SAM4CMP32 (SAM4CMP32_0 || SAM4CMP32_1)

#if defined(__SAM4CMS8C_0__)
#define SAM4CMS8_0 (1)
#else
#define SAM4CMS8_0 (0)
#endif

#if defined(__SAM4CMS8C_1__)
#define SAM4CMS8_1 (1)
#else
#define SAM4CMS8_1 (0)
#endif

#define SAM4CMS8 (SAM4CMS8_0 || SAM4CMS8_1)

#if defined(__SAM4CMS16C_0__)
#define SAM4CMS16_0 (1)
#else
#define SAM4CMS16_0 (0)
#endif

#if defined(__SAM4CMS16C_1__)
#define SAM4CMS16_1 (1)
#else
#define SAM4CMS16_1 (0)
#endif

#define SAM4CMS16 (SAM4CMS16_0 || SAM4CMS16_1)

#if defined(__SAM4CMS32C_0__)
#define SAM4CMS32_0 (1)
#else
#define SAM4CMS32_0 (0)
#endif

#if defined(__SAM4CMS32C_1__)
#define SAM4CMS32_1 (1)
#else
#define SAM4CMS32_1 (0)
#endif

#define SAM4CMS32 (SAM4CMS32_0 || SAM4CMS32_1)

/** @} */

/**
 * \name SAM4CP series
 * @{
 */
#if defined(__SAM4CP16B_0__)
#define SAM4CP16_0 (1)
#else
#define SAM4CP16_0 (0)
#endif

#if defined(__SAM4CP16B_1__)
#define SAM4CP16_1 (1)
#else
#define SAM4CP16_1 (0)
#endif

#define SAM4CP16 (SAM4CP16_0 || SAM4CP16_1)
/** @} */

/**
 * \name SAMG series
 * @{
 */
#if defined(__SAMG51G18__)
#define SAMG51 (1)
#else
#define SAMG51 (0)
#endif

#if defined(__SAMG53G19__) ||\
	defined(__SAMG53N19__)
#define SAMG53 (1)
#else
#define SAMG53 (0)
#endif

#if defined(__SAMG54G19__) ||\
	defined(__SAMG54J19__) ||\
	defined(__SAMG54N19__)
#define SAMG54 (1)
#else
#define SAMG54 (0)
#endif

#if defined(__SAMG55G18__) ||\
	defined(__SAMG55G19__) ||\
	defined(__SAMG55J18__) ||\
	defined(__SAMG55J19__) ||\
	defined(__SAMG55N19__)
#define SAMG55 (1)
#else
#define SAMG55 (0)
#endif

/** @} */
/**
 * \name SAM families
 * @{
 */
/** SAM3S Family */
#define SAM3S (SAM3S1 || SAM3S2 || SAM3S4 || SAM3S8 || SAM3SD8)

/** SAM3U Family */
#define SAM3U (SAM3U1 || SAM3U2 || SAM3U4)

/** SAM3N Family */
#define SAM3N (SAM3N00 || SAM3N0 || SAM3N1 || SAM3N2 || SAM3N4)

/** SAM3XA Family */
#define SAM3XA (SAM3X4 || SAM3X8 || SAM3A4 || SAM3A8)

/** SAM4S Family */
#define SAM4S (SAM4S2 || SAM4S4 || SAM4S8 || SAM4S16 || SAM4SA16 || SAM4SD16 || SAM4SD32)

/** SAM4L Family */
#define SAM4L (SAM4LS || SAM4LC)

/** SAMD51 Family */
#define SAMD51 (SAMD51A)

/** SAMD20 Family */
#define SAMD20 (SAMD20J || SAMD20G || SAMD20E)

/** SAMD21 Family */
#define SAMD21 (SAMD21J || SAMD21G || SAMD21E)

/** SAMD10 Family */
#define SAMD10 (SAMD10C || SAMD10DS || SAMD10DM)

/** SAMD11 Family */
#define SAMD11 (SAMD11C || SAMD11DS || SAMD11DM)

/** SAMD Family */
#define SAMD   (SAMD51 || SAMD20 || SAMD21 || SAMD10 || SAMD11)

/** SAMR21 Family */
#define SAMR21 (SAMR21G || SAMR21E)

/** SAML21 Family */
#define SAML21 (SAML21J || SAML21G || SAML21E)

/** SAM4E Family */
#define SAM4E (SAM4E8 || SAM4E16)

/** SAM4N Family */
#define SAM4N (SAM4N8 || SAM4N16)

/** SAM4C Family */
#define SAM4C_0 (SAM4C8_0 || SAM4C16_0 || SAM4C32_0)
#define SAM4C_1 (SAM4C8_1 || SAM4C16_1 || SAM4C32_1)
#define SAM4C   (SAM4C8 || SAM4C16 || SAM4C32)

/** SAM4CM Family */
#define SAM4CM_0 (SAM4CMP8_0 || SAM4CMP16_0 || SAM4CMP32_0 || SAM4CMS8_0 || \
			SAM4CMS16_0 || SAM4CMS32_0)
#define SAM4CM_1 (SAM4CMP8_1 || SAM4CMP16_1 || SAM4CMP32_1 || SAM4CMS8_1 || \
			SAM4CMS16_1 || SAM4CMS32_1)
#define SAM4CM   (SAM4CMP8 || SAM4CMP16 || SAM4CMP32 || SAM4CMS8 || \
			SAM4CMS16 || SAM4CMS32)

/** SAM4CP Family */
#define SAM4CP_0 (SAM4CP16_0)
#define SAM4CP_1 (SAM4CP16_1)
#define SAM4CP   (SAM4CP16)

/** SAMG Family */
#define SAMG (SAMG51 || SAMG53 || SAMG54 || SAMG55)

/** SAM0 product line (cortex-m0+) */
#define SAM0 (SAMD20 || SAMD21 || SAMR21 || SAMD10 || SAMD11 || SAML21)

/** @} */

/** SAM product line */
#define SAM (SAM3S || SAM3U || SAM3N || SAM3XA || SAM4S || SAM4L || SAM4E || \
		SAM0 || SAM4N || SAM4C || SAM4CM || SAM4CP || SAMG || SAMD51)

/** @} */

/** @} */

/** @} */

#endif /* ATMEL_PARTS_H */
