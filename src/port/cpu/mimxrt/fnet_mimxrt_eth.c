/**************************************************************************
*
* Copyright 2018 by Andrey Butok. FNET Community.
*
***************************************************************************
*
*  Licensed under the Apache License, Version 2.0 (the "License"); you may
*  not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
*  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
***************************************************************************
*
*  Ethernet driver interafce.
*
***************************************************************************/

#include "fnet.h"
#if FNET_MIMXRT && FNET_CFG_CPU_ETH0
#define CLRSET(reg, clear, set) ((reg) = ((reg) & ~(clear)) | (set))
#define RMII_PAD_INPUT_PULLDOWN 0x30E9
#define RMII_PAD_INPUT_PULLUP   0xB0E9
#define RMII_PAD_CLOCK          0x0031

#include "port/netif/fec/fnet_fec.h"

#if FNET_CFG_CPU_MIMXRT1176
/*======================== i.MX RT1176 (NXP MIMXRT1170-EVKB) =================
 * Board bring-up for the 10/100 ENET (FEC @0x40424000) + RTL8201 PHY (MDIO
 * addr 3).  Transplanted logic-identical from the HW-verified rt1170/evkb
 * milestone-1 driver (cores/imxrt1176/enet.c -- HW-proven: PHYID 1C:C816,
 * board answers ping), which itself follows the NXP SDK evkbmimxrt1170 enet
 * example (pin_mux.c / hardware_init.c / fsl_clock.c / fsl_anatop_ai.c).
 * One-off register addresses are carried locally so this port stays
 * self-contained (no SoC vendor header required).
 *
 * HW-CRITICAL: the boot ROM leaves SysPll1 powered down (SYS_PLL1_CTRL =
 * 0x4000).  MDIO + PHY link work without it, but the RMII 50 MHz ref-clock is
 * SysPll1Div2/10 -- without the PLL the MAC moves ZERO frames.  All AI-protocol
 * polls are BOUNDED so QEMU (no analog-interface model) proceeds instead of
 * hanging; on silicon DONE/STABLE assert in ~us.
 *===========================================================================*/

/* Busy-wait provided by the Arduino core (delay.c). */
extern void delayMicroseconds(fnet_uint32_t usec);

/* ---- IOMUXC: RMII + MDIO pads (SW_MUX / SW_PAD) ---- */
#define FNET1176_MUX_AD_32        (*(volatile fnet_uint32_t *)0x400E818Cu) /* ENET_MDC   ALT3 */
#define FNET1176_MUX_AD_33        (*(volatile fnet_uint32_t *)0x400E8190u) /* ENET_MDIO  ALT3 */
#define FNET1176_MUX_DISP_B2_02   (*(volatile fnet_uint32_t *)0x400E821Cu) /* TXD0  ALT1 */
#define FNET1176_MUX_DISP_B2_03   (*(volatile fnet_uint32_t *)0x400E8220u) /* TXD1  ALT1 */
#define FNET1176_MUX_DISP_B2_04   (*(volatile fnet_uint32_t *)0x400E8224u) /* TX_EN ALT1 */
#define FNET1176_MUX_DISP_B2_05   (*(volatile fnet_uint32_t *)0x400E8228u) /* REF_CLK ALT2+SION */
#define FNET1176_MUX_DISP_B2_06   (*(volatile fnet_uint32_t *)0x400E822Cu) /* RXD0  ALT1+SION */
#define FNET1176_MUX_DISP_B2_07   (*(volatile fnet_uint32_t *)0x400E8230u) /* RXD1  ALT1+SION */
#define FNET1176_MUX_DISP_B2_08   (*(volatile fnet_uint32_t *)0x400E8234u) /* RX_EN ALT1 */
#define FNET1176_MUX_DISP_B2_09   (*(volatile fnet_uint32_t *)0x400E8238u) /* RX_ER ALT1 */
#define FNET1176_PAD_DISP_B2_02   (*(volatile fnet_uint32_t *)0x400E8460u)
#define FNET1176_PAD_DISP_B2_03   (*(volatile fnet_uint32_t *)0x400E8464u)
#define FNET1176_PAD_DISP_B2_04   (*(volatile fnet_uint32_t *)0x400E8468u)
#define FNET1176_PAD_DISP_B2_05   (*(volatile fnet_uint32_t *)0x400E846Cu)
#define FNET1176_PAD_DISP_B2_06   (*(volatile fnet_uint32_t *)0x400E8470u)
#define FNET1176_PAD_DISP_B2_07   (*(volatile fnet_uint32_t *)0x400E8474u)
#define FNET1176_PAD_DISP_B2_08   (*(volatile fnet_uint32_t *)0x400E8478u)
#define FNET1176_PAD_DISP_B2_09   (*(volatile fnet_uint32_t *)0x400E847Cu)
#define FNET1176_MUX_LPSR_12      (*(volatile fnet_uint32_t *)0x40C08030u) /* GPIO12_IO12 (PHY reset) */
#define FNET1176_PAD_LPSR_12      (*(volatile fnet_uint32_t *)0x40C08070u)
#define FNET1176_SION             0x10u

/* ---- IOMUXC SELECT_INPUT daisy regs (from fsl_iomuxc.h) ---- */
#define FNET1176_DAISY_MDIO       (*(volatile fnet_uint32_t *)0x400E84ACu)
#define FNET1176_DAISY_REF_CLK    (*(volatile fnet_uint32_t *)0x400E84A8u)
#define FNET1176_DAISY_RXD0       (*(volatile fnet_uint32_t *)0x400E84B0u)
#define FNET1176_DAISY_RXD1       (*(volatile fnet_uint32_t *)0x400E84B4u)
#define FNET1176_DAISY_RXEN       (*(volatile fnet_uint32_t *)0x400E84B8u)
#define FNET1176_DAISY_RXER       (*(volatile fnet_uint32_t *)0x400E84BCu)

/* ---- GPR + LPSR GPIO12 (PHY hardware reset line) ---- */
#define FNET1176_GPR4             (*(volatile fnet_uint32_t *)0x400E4010u)
#define FNET1176_GPR4_REF_CLK_DIR (1u << 1)
#define FNET1176_GPR28            (*(volatile fnet_uint32_t *)0x400E4070u)
#define FNET1176_GPR28_CACHE_ENET (1u << 7)
#define FNET1176_GPIO12_GDIR      (*(volatile fnet_uint32_t *)0x40C70004u)
#define FNET1176_GPIO12_DR_SET    (*(volatile fnet_uint32_t *)0x40C70084u)
#define FNET1176_GPIO12_DR_CLEAR  (*(volatile fnet_uint32_t *)0x40C70088u)

/* ---- CCM: ENET1 clock root (root 51) + LPCG112 gate ---- */
#define FNET1176_CCM_ROOT51       (*(volatile fnet_uint32_t *)0x40CC1980u)
#define FNET1176_CCM_ROOT_MUX(x)  (((fnet_uint32_t)(x) << 8) & 0x700u)
#define FNET1176_CCM_ROOT_DIV(x)  (((fnet_uint32_t)(x) << 0) & 0x0FFu)
#define FNET1176_CCM_LPCG112      (*(volatile fnet_uint32_t *)0x40CC6E00u)

/* ---- ANADIG / ANATOP-AI: SysPll1 (one overlapping block @0x40C84000) ---- */
#define FNET1176_SP1_CTRL    (*(volatile fnet_uint32_t *)0x40C842C0u)
#define  FNET1176_SP1_ENCLK  0x00002000u
#define  FNET1176_SP1_GATE   0x00004000u
#define  FNET1176_SP1_DIV2   0x02000000u
#define  FNET1176_SP1_DIV5   0x04000000u
#define  FNET1176_SP1_STABLE 0x20000000u
#define FNET1176_AI1G_CTRL   (*(volatile fnet_uint32_t *)0x40C84850u)
#define FNET1176_AI1G_WDATA  (*(volatile fnet_uint32_t *)0x40C84860u)
#define FNET1176_AI1G_RDATA  (*(volatile fnet_uint32_t *)0x40C84870u)
#define FNET1176_AILDO_CTRL  (*(volatile fnet_uint32_t *)0x40C84820u)
#define FNET1176_AILDO_WDAT  (*(volatile fnet_uint32_t *)0x40C84830u)
#define FNET1176_AILDO_RDAT  (*(volatile fnet_uint32_t *)0x40C84840u)
#define FNET1176_PMU_LDOPLL  (*(volatile fnet_uint32_t *)0x40C84500u)
#define FNET1176_PMU_REFCTL  (*(volatile fnet_uint32_t *)0x40C84570u)
#define  FNET1176_AI_ADDRM   0x000000FFu
#define  FNET1176_AI_RWB     0x00010000u
#define  FNET1176_AI1G_TOG   0x00000100u
#define  FNET1176_AI1G_DONE  0x00000200u
#define  FNET1176_PMU_TOG    0x00010000u
#define  FNET1176_PMU_VREF   0x00000010u
#define  FNET1176_AIR_C0     0x00u
#define  FNET1176_AIR_C0SET  0x04u
#define  FNET1176_AIR_C0CLR  0x08u
#define  FNET1176_AIR_C2     0x20u
#define  FNET1176_AIR_C3     0x30u
#define  FNET1176_AIR_LDO0   0x00u
#define  FNET1176_1G_HOLDR   0x00002000u
#define  FNET1176_1G_PWRUP   0x00004000u
#define  FNET1176_1G_EN      0x00008000u
#define  FNET1176_1G_BYP     0x00010000u
#define  FNET1176_1G_REGEN   0x00400000u
#define  FNET1176_1G_DIVM    0x0000007Fu
#define  FNET1176_LDO_LINR   0x00000001u
#define  FNET1176_LDO_LIMIT  0x00000004u
#define  FNET1176_LDO_1V0    0x00000100u

/* AI transport: 1G PLL interface (toggle + wait-done handshake, bounded). */
static void fnet1176_ai1g_write(fnet_uint32_t a, fnet_uint32_t d)
{
    fnet_uint32_t pre = FNET1176_AI1G_CTRL & FNET1176_AI1G_DONE, to = 100000u, t;
    FNET1176_AI1G_CTRL &= ~FNET1176_AI_RWB;                              /* write mode */
    t = FNET1176_AI1G_CTRL; t = (t & ~FNET1176_AI_ADDRM) | (a & FNET1176_AI_ADDRM); FNET1176_AI1G_CTRL = t;
    FNET1176_AI1G_WDATA = d;
    FNET1176_AI1G_CTRL ^= FNET1176_AI1G_TOG;                             /* kick */
    while (((FNET1176_AI1G_CTRL & FNET1176_AI1G_DONE) == pre) && --to) { }
}
static fnet_uint32_t fnet1176_ai1g_read(fnet_uint32_t a)
{
    fnet_uint32_t pre = FNET1176_AI1G_CTRL & FNET1176_AI1G_DONE, to = 100000u, t;
    t = FNET1176_AI1G_CTRL | FNET1176_AI_RWB; FNET1176_AI1G_CTRL = t;    /* read mode */
    t = FNET1176_AI1G_CTRL; t = (t & ~FNET1176_AI_ADDRM) | (a & FNET1176_AI_ADDRM); FNET1176_AI1G_CTRL = t;
    FNET1176_AI1G_CTRL ^= FNET1176_AI1G_TOG;
    while (((FNET1176_AI1G_CTRL & FNET1176_AI1G_DONE) == pre) && --to) { }
    return FNET1176_AI1G_RDATA;
}
/* AI transport: LDO interface (toggle PMU_LDO_PLL, no done-wait). */
static void fnet1176_aildo_write(fnet_uint32_t a, fnet_uint32_t d)
{
    fnet_uint32_t t;
    FNET1176_AILDO_CTRL &= ~FNET1176_AI_RWB;
    t = FNET1176_AILDO_CTRL; t = (t & ~FNET1176_AI_ADDRM) | (a & FNET1176_AI_ADDRM); FNET1176_AILDO_CTRL = t;
    FNET1176_AILDO_WDAT = d;
    FNET1176_PMU_LDOPLL ^= FNET1176_PMU_TOG;
}
static fnet_uint32_t fnet1176_aildo_read(fnet_uint32_t a)
{
    fnet_uint32_t t = FNET1176_AILDO_CTRL | FNET1176_AI_RWB; FNET1176_AILDO_CTRL = t;
    t = FNET1176_AILDO_CTRL; t = (t & ~FNET1176_AI_ADDRM) | (a & FNET1176_AI_ADDRM); FNET1176_AILDO_CTRL = t;
    FNET1176_PMU_LDOPLL ^= FNET1176_PMU_TOG;
    return FNET1176_AILDO_RDAT;
}
static void fnet1176_sys_pll1_init(void)
{
    fnet_uint32_t to, r;
    if (FNET1176_SP1_CTRL & FNET1176_SP1_STABLE) return;                 /* already locked */
    /* PLL LDO (1.0 V) enable + 100us soft-start (skip if already on). */
    if (fnet1176_aildo_read(FNET1176_AIR_LDO0) != (FNET1176_LDO_1V0 | FNET1176_LDO_LINR)) {
        fnet1176_aildo_write(FNET1176_AIR_LDO0, FNET1176_LDO_1V0 | FNET1176_LDO_LINR | FNET1176_LDO_LIMIT);
        delayMicroseconds(100);
        fnet1176_aildo_write(FNET1176_AIR_LDO0, FNET1176_LDO_1V0 | FNET1176_LDO_LINR);
        FNET1176_PMU_REFCTL |= FNET1176_PMU_VREF;
    }
    fnet1176_ai1g_write(FNET1176_AIR_C0SET, FNET1176_1G_BYP);            /* bypass on */
    FNET1176_SP1_CTRL |= FNET1176_SP1_ENCLK;                             /* sw enable clk */
    fnet1176_ai1g_write(FNET1176_AIR_C3, 0x0FFFFFFFu);                   /* denominator 2^28-1 */
    fnet1176_ai1g_write(FNET1176_AIR_C2, 178956970u);                    /* numerator 0x0AAAAAAA */
    r = fnet1176_ai1g_read(FNET1176_AIR_C0); r = (r & ~FNET1176_1G_DIVM) | (41u & FNET1176_1G_DIVM);
    fnet1176_ai1g_write(FNET1176_AIR_C0, r);                             /* loop divider 41 -> 1000 MHz */
    fnet1176_ai1g_write(FNET1176_AIR_C0SET, FNET1176_1G_REGEN);          /* PLL reg enable */
    delayMicroseconds(100);
    fnet1176_ai1g_write(FNET1176_AIR_C0SET, FNET1176_1G_PWRUP | FNET1176_1G_HOLDR); /* power up */
    fnet1176_ai1g_write(FNET1176_AIR_C0SET, FNET1176_1G_HOLDR);          /* toggle hold-ring-off */
    delayMicroseconds(225);
    fnet1176_ai1g_write(FNET1176_AIR_C0CLR, FNET1176_1G_HOLDR);
    to = 1000000u; while (((FNET1176_SP1_CTRL & FNET1176_SP1_STABLE) == 0u) && --to) { } /* wait lock */
    fnet1176_ai1g_write(FNET1176_AIR_C0SET, FNET1176_1G_EN);             /* enable clk out */
    FNET1176_SP1_CTRL &= ~FNET1176_SP1_GATE;                             /* ungate */
    FNET1176_SP1_CTRL |= FNET1176_SP1_DIV2;                              /* /2 tap (500 MHz -> ENET) */
    FNET1176_SP1_CTRL &= ~FNET1176_SP1_DIV5;
    fnet1176_ai1g_write(FNET1176_AIR_C0CLR, FNET1176_1G_BYP);            /* bypass off */
}
static void fnet1176_clock_init(void)
{
    fnet1176_sys_pll1_init();   /* SysPll1 1 GHz + /2 (500 MHz) = the ENET root source */
    /* ENET1 clock root (root 51) <- SysPll1Div2 (mux 4), divide-by-10 -> 50 MHz
       (SDK BOARD_InitModuleClock {.mux=4,.div=10}; ROOT DIV field = divider-1). */
    FNET1176_CCM_ROOT51 = FNET1176_CCM_ROOT_MUX(4) | FNET1176_CCM_ROOT_DIV(9);
    FNET1176_CCM_LPCG112 = 1u;  /* ungate the ENET peripheral clock */
}
static void fnet1176_pins_init(void)
{
    /* MDC / MDIO: ALT3, no SION; SDK leaves PAD_CTL at default (no write). */
    FNET1176_MUX_AD_32 = 3u;                  /* ENET_MDC  */
    FNET1176_MUX_AD_33 = 3u;                  /* ENET_MDIO */
    FNET1176_DAISY_MDIO = 1u;
    /* TXD0/TXD1/TX_EN: ALT1, no SION, PAD_CTL = 0x02. */
    FNET1176_MUX_DISP_B2_02 = 1u;  FNET1176_PAD_DISP_B2_02 = 0x02u;
    FNET1176_MUX_DISP_B2_03 = 1u;  FNET1176_PAD_DISP_B2_03 = 0x02u;
    FNET1176_MUX_DISP_B2_04 = 1u;  FNET1176_PAD_DISP_B2_04 = 0x02u;
    /* REF_CLK: ALT2 + SION, daisy=1, PAD_CTL = 0x03 (50 MHz RMII ref clock). */
    FNET1176_MUX_DISP_B2_05 = 2u | FNET1176_SION;
    FNET1176_DAISY_REF_CLK = 1u;
    FNET1176_PAD_DISP_B2_05 = 0x03u;
    /* RXD0/RXD1: ALT1 + SION, daisy=1, PAD_CTL = 0x06. */
    FNET1176_MUX_DISP_B2_06 = 1u | FNET1176_SION;  FNET1176_DAISY_RXD0 = 1u;  FNET1176_PAD_DISP_B2_06 = 0x06u;
    FNET1176_MUX_DISP_B2_07 = 1u | FNET1176_SION;  FNET1176_DAISY_RXD1 = 1u;  FNET1176_PAD_DISP_B2_07 = 0x06u;
    /* RX_EN/RX_ER: ALT1, no SION, daisy=1, PAD_CTL = 0x06. */
    FNET1176_MUX_DISP_B2_08 = 1u;  FNET1176_DAISY_RXEN = 1u;  FNET1176_PAD_DISP_B2_08 = 0x06u;
    FNET1176_MUX_DISP_B2_09 = 1u;  FNET1176_DAISY_RXER = 1u;  FNET1176_PAD_DISP_B2_09 = 0x06u;
    /* 50 MHz ENET_REF_CLK is an OUTPUT from the SoC to the external PHY. */
    FNET1176_GPR4 |= FNET1176_GPR4_REF_CLK_DIR;
    /* ERR050396: DMA buffers live in OCRAM; clear CACHE_ENET so ENET writes
       bypass the CM7-TCM sparse-write path. */
    FNET1176_GPR28 &= ~FNET1176_GPR28_CACHE_ENET;
}
static void fnet1176_phy_reset(void)
{
    /* RTL8201 hardware reset via GPIO12_IO12 (pad GPIO_LPSR_12, ALT 0xA);
       >=10 ms low, >=150 ms release -- per SDK hardware_init.c. */
    FNET1176_MUX_LPSR_12 = 0xAu;
    FNET1176_PAD_LPSR_12 = 0x0Eu;
    FNET1176_GPIO12_GDIR |= (1u << 12);
    FNET1176_GPIO12_DR_CLEAR = (1u << 12);
    delayMicroseconds(10000);
    FNET1176_GPIO12_DR_SET = (1u << 12);
    delayMicroseconds(150000);
}
static void fnet1176_eth_io_init(void)
{
    fnet1176_clock_init();
    fnet1176_pins_init();
    fnet1176_phy_reset();
}
#endif /* FNET_CFG_CPU_MIMXRT1176 */

static fnet_return_t fnet_mimxrt_eth_init(fnet_netif_t *netif);
static fnet_return_t fnet_mimxrt_eth_phy_init(fnet_netif_t *netif);

/************************************************************************
* Ethernet interface structure.
*************************************************************************/
static fnet_eth_if_t fnet_mimxrt_eth0_if =
{
    .eth_prv = &fnet_fec0_if,                       /* Points to Ethernet driver-specific control data structure. */
    .eth_mac_number = 0,                            /* MAC module number. */
    .eth_output = fnet_fec_output,                  /* Ethernet driver output.*/
    .eth_phy_addr = FNET_CFG_CPU_ETH0_PHY_ADDR,     /* Set default PHY address */
    .eth_cpu_init = fnet_mimxrt_eth_init,
    .eth_cpu_phy_init = fnet_mimxrt_eth_phy_init,
#if FNET_CFG_MULTICAST
    .eth_multicast_join = fnet_fec_multicast_join,  /* Ethernet driver join multicast group.*/
    .eth_multicast_leave = fnet_fec_multicast_leave /* Ethernet driver leave multicast group.*/
#endif
};

fnet_netif_t fnet_cpu_eth0_if =
{
    .netif_name = FNET_CFG_CPU_ETH0_NAME,     /* Network interface name.*/
    .netif_mtu = FNET_CFG_CPU_ETH0_MTU,       /* Maximum transmission unit.*/
    .netif_prv = &fnet_mimxrt_eth0_if,        /* Points to interface specific data structure.*/
    .netif_api = &fnet_fec_api                /* Interface API */
};

/************************************************************************
* DESCRIPTION: Ethernet IO initialization.
*************************************************************************/
static fnet_return_t fnet_mimxrt_eth_init(fnet_netif_t *netif)
{
#if FNET_CFG_CPU_ETH_IO_INIT
#if FNET_CFG_CPU_MIMXRT1176
    fnet1176_eth_io_init();
#endif
#if FNET_CFG_CPU_MIMXRT1052 || FNET_CFG_CPU_MIMXRT1062
      CCM_CCGR1 |= CCM_CCGR1_ENET(CCM_CCGR_ON);
      // configure PLL6 for 50 MHz, pg 1173
      CCM_ANALOG_PLL_ENET_CLR = CCM_ANALOG_PLL_ENET_POWERDOWN
        | CCM_ANALOG_PLL_ENET_BYPASS | 0x0F;
      CCM_ANALOG_PLL_ENET_SET = CCM_ANALOG_PLL_ENET_ENABLE | CCM_ANALOG_PLL_ENET_BYPASS
        /*| CCM_ANALOG_PLL_ENET_ENET2_REF_EN*/ | CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN
        /*| CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT(1)*/ | CCM_ANALOG_PLL_ENET_DIV_SELECT(1);
      while (!(CCM_ANALOG_PLL_ENET & CCM_ANALOG_PLL_ENET_LOCK)) ; // wait for PLL lock
      CCM_ANALOG_PLL_ENET_CLR = CCM_ANALOG_PLL_ENET_BYPASS;
    //  Serial.printf("PLL6 = %08X (should be 80202001)\n", CCM_ANALOG_PLL_ENET);
      // Drive ENET_REF_CLK (B1_10) as an OUTPUT from PLL6 -- set ENET1_TX_CLK_DIR.
      // This is the same on the EVKB as on the Teensy: the i.MX RT generates the
      // 50 MHz RMII reference clock and feeds it to the PHY.  The EVKB's
      // KSZ8081RNB needs that 50 MHz supplied to it (it does not synthesise its
      // own).  The NXP SDK board boot-clock (clock_config.c) clears this bit as a
      // default, but the lwip/enet example re-sets it to output in
      // BOARD_InitHardware() (IOMUXC_EnableMode(..., kIOMUXC_GPR_ENET1TxClkOutputDir,
      // true)) with CLOCK_InitEnetPll(enableClkOutput=true) -- so the final state
      // is TX_CLK_DIR = 1.  Clearing it starves the PHY of its reference clock:
      // MDIO still works (it is clocked by MDC), but the link never comes up and
      // auto-negotiation never completes.
      CLRSET(IOMUXC_GPR_GPR1, IOMUXC_GPR_GPR1_ENET1_CLK_SEL | IOMUXC_GPR_GPR1_ENET_IPG_CLK_S_EN,
        IOMUXC_GPR_GPR1_ENET1_TX_CLK_DIR);
    //  Serial.printf("GPR1 = %08X\n", IOMUXC_GPR_GPR1);

      // configure pins
#if defined(ARDUINO_MIMXRT1060_EVKB)
      // EVKB: KSZ8081 reset on AD_B0_09.  The Teensy core routes AD_B0 pads to
      // the fast GPIO6 bank (GPR26 = 0xFFFFFFFF in startup.c), so drive GPIO6.9.
      // There is no separate power pin (AD_B0_10 is the PHY interrupt, unused).
      IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_09 = 5; // ALT5 = GPIO
      GPIO6_GDIR |= (1<<9);
      GPIO6_DR_CLEAR = (1<<9); // hold PHY in reset
      // KSZ8081 address (2) is strapped on the EVKB; set the RMII RX pads to
      // pull-up (per the SDK) rather than driving Teensy DP83825 strap values.
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = RMII_PAD_INPUT_PULLUP;
#else
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14 = 5; // Reset   B0_14 Alt5 GPIO7.15
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15 = 5; // Power   B0_15 Alt5 GPIO7.14
      GPIO7_GDIR |= (1<<14) | (1<<15);
      GPIO7_DR_SET = (1<<15);   // power on
      GPIO7_DR_CLEAR = (1<<14); // reset PHY chip
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = RMII_PAD_INPUT_PULLDOWN; // PhyAdd[0] = 0
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = RMII_PAD_INPUT_PULLDOWN; // PhyAdd[1] = 1
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = RMII_PAD_INPUT_PULLUP;   // Master/Slave = slave mode
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = RMII_PAD_INPUT_PULLDOWN; // Auto MDIX Enable
#endif
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_07 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_08 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_09 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_10 = RMII_PAD_CLOCK;
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_05 = 3; // RXD1    B1_05 Alt3, pg 525
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04 = 3; // RXD0    B1_04 Alt3, pg 524
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10 = 6 | 0x10; // REFCLK  B1_10 Alt6, pg 530
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_11 = 3; // RXER    B1_11 Alt3, pg 531
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_06 = 3; // RXEN    B1_06 Alt3, pg 526
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_09 = 3; // TXEN    B1_09 Alt3, pg 529
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_07 = 3; // TXD0    B1_07 Alt3, pg 527
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_08 = 3; // TXD1    B1_08 Alt3, pg 528
#if defined(ARDUINO_MIMXRT1060_EVKB)
      // EVKB routes ENET MDC/MDIO to EMC_40/EMC_41 (ALT4), not B1_14/B1_15.
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_41 = 4; // MDIO  EMC_41 Alt4
      IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_40 = 4; // MDC   EMC_40 Alt4
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_41 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_40 = RMII_PAD_INPUT_PULLUP;
      IOMUXC_ENET_MDIO_SELECT_INPUT = 1; // GPIO_EMC_41_ALT4
#else
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_15 = 0; // MDIO    B1_15 Alt0, pg 535
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14 = 0; // MDC     B1_14 Alt0, pg 534
      IOMUXC_ENET_MDIO_SELECT_INPUT = 2; // GPIO_B1_15_ALT0, pg 792
#endif
      IOMUXC_ENET0_RXDATA_SELECT_INPUT = 1; // GPIO_B1_04_ALT3, pg 792
      IOMUXC_ENET1_RXDATA_SELECT_INPUT = 1; // GPIO_B1_05_ALT3, pg 793
      IOMUXC_ENET_RXEN_SELECT_INPUT = 1; // GPIO_B1_06_ALT3, pg 794
      IOMUXC_ENET_RXERR_SELECT_INPUT = 1; // GPIO_B1_11_ALT3, pg 795
      IOMUXC_ENET_IPG_CLK_RMII_SELECT_INPUT = 1; // GPIO_B1_10_ALT6, pg 791
      delayMicroseconds(2);
#if defined(ARDUINO_MIMXRT1060_EVKB)
      GPIO6_DR_SET = (1<<9); // release KSZ8081 reset
#else
      GPIO7_DR_SET = (1<<14); // start PHY chip
#endif
      ENET_MSCR = ENET_MSCR_MII_SPEED(9);
      delayMicroseconds(5);
    //  Serial.printf("RCSR:%04X, LEDCR:%04X, PHYCR %04X\n",
    //    mdio_read(0, 0x17), mdio_read(0, 0x18), mdio_read(0, 0x19));

#if !defined(ARDUINO_MIMXRT1060_EVKB)
      // DP83825 (Teensy): LEDCR offset 0x18, set LED_Link_Polarity, pg 62
      _fnet_eth_phy_write(netif, 0x18, 0x0280); // LED shows link status, active high
      // RCSR offset 0x17, set RMII_Clock_Select, pg 61
      _fnet_eth_phy_write(netif, 0x17, 0x0081); // config for 50 MHz clock input
#else
      // EVKB KSZ8081RNB: the 50 MHz RMII reference-clock mode (PHY Control 2
      // reg 0x1F bit 7, REFCLK_SELECT) is selected in fnet_mimxrt_eth_phy_init()
      // below -- NOT here.  The generic _fnet_eth_phy_init() issues a PHY soft
      // reset (which reloads the KSZ8081 config straps and would wipe the bit)
      // and only then calls the board phy-init hook, so the bit must be set from
      // that hook to survive.  (The DP83825 RCSR/LEDCR registers do not apply.)
#endif

    //  Serial.printf("RCSR:%04X, LEDCR:%04X, PHYCR %04X\n",
    //    mdio_read(0, 0x17), mdio_read(0, 0x18), mdio_read(0, 0x19));

    //  printhex("MDIO PHY ID2 (LAN8720A is 0007, DP83825I is 2000): ", mdio_read(0, 2));
    //  printhex("MDIO PHY ID3 (LAN8720A is C0F?, DP83825I is A140): ", mdio_read(0, 3));
    //  printhex("BMCR: ", mdio_read(0, 0));
    //  printhex("BMSR: ", mdio_read(0, 1));

#endif /* FNET_CFG_CPU_MIMXRT1052 || FNET_CFG_CPU_MIMXRT1062 */
#endif /*!FNET_CFG_CPU_ETH_IO_INIT*/
    return FNET_OK;
}

/************************************************************************
* DESCRIPTION: Ethernet Physical Transceiver initialization and/or reset.
*************************************************************************/
static fnet_return_t fnet_mimxrt_eth_phy_init(fnet_netif_t *netif)
{
#if FNET_CFG_CPU_MIMXRT1176
    /* RTL8201 @ MDIO addr 3: generic clause-22 only (no vendor registers --
       HW-verified in milestone 1; Zephyr drives this PHY generically too).
       Advertise the full 10/100 ability set here: the generic
       _fnet_eth_phy_init() has just soft-reset the PHY and restarts
       auto-negotiation right after this hook returns, so the ANAR write
       (0x01E1, same value as the HW-verified enet.c sequence) takes effect
       for the negotiation that brings the link up at 100BASE-TX full duplex. */
    _fnet_eth_phy_write(netif, FNET_ETH_MII_REG_ANAR,
                        (fnet_uint16_t)(FNET_ETH_MII_REG_ANAR_100_FULLDUPLEX |
                                        FNET_ETH_MII_REG_ANAR_100_HALFDUPLEX |
                                        FNET_ETH_MII_REG_ANAR_10_FULLDUPLEX  |
                                        FNET_ETH_MII_REG_ANAR_10_HALFDUPLEX  |
                                        FNET_ETH_MII_REG_ANAR_IEEE8023));
#endif
#if FNET_CFG_CPU_MIMXRT1052 || FNET_CFG_CPU_MIMXRT1062
#if defined(ARDUINO_MIMXRT1060_EVKB)
    /* EVKB on-board KSZ8081RNB: select 50 MHz RMII reference-clock mode, exactly
       as the NXP SDK does for this board (it builds the KSZ8081 driver with
       -DFSL_FEATURE_PHYKSZ8081_USE_RMII50M_MODE): set PHY Control 2 (reg 0x1F)
       bit 7 (REFCLK_SELECT).  Without it the KSZ8081 stays in 25 MHz mode and the
       link never comes up, even though the MAC drives 50 MHz on ENET_REF_CLK.

       This MUST be done here, in the board phy-init hook: the generic
       _fnet_eth_phy_init() that calls us has just issued a PHY soft reset (which
       reloads the KSZ8081 config straps and clears this bit) and will restart
       auto-negotiation immediately after we return -- so setting REFCLK_SELECT
       here is both late enough to survive the reset and early enough to take
       effect before auto-negotiation.  (Doing it in eth_cpu_init runs before that
       soft reset and gets wiped.) */
    fnet_uint16_t ctl2 = 0;
    _fnet_eth_phy_read(netif, 0x1F, &ctl2);
    _fnet_eth_phy_write(netif, 0x1F, (fnet_uint16_t)(ctl2 | 0x0080u)); /* REFCLK_SELECT = 50 MHz */

    /* Advertise the full 10/100 ability set before auto-negotiation restarts.
       The EVKB KSZ8081's reset/strap default only advertises 10 Mbps (ANAR reads
       0x8061), so the link comes up at 10BASE-T -- which the FEC, fixed at 100M
       (RMII_10T not set), cannot talk to.  The NXP SDK's PHY_KSZ8081_Init writes
       this same advertisement (reg 0x04 = 0x01E1) for exactly this reason; the
       generic _fnet_eth_phy_init() restarts auto-negotiation right after we
       return, so the new advertisement takes effect and the link comes up at
       100BASE-TX full-duplex to match the MAC. */
    _fnet_eth_phy_write(netif, FNET_ETH_MII_REG_ANAR,
                        (fnet_uint16_t)(FNET_ETH_MII_REG_ANAR_100_FULLDUPLEX |
                                        FNET_ETH_MII_REG_ANAR_100_HALFDUPLEX |
                                        FNET_ETH_MII_REG_ANAR_10_FULLDUPLEX  |
                                        FNET_ETH_MII_REG_ANAR_10_HALFDUPLEX  |
                                        FNET_ETH_MII_REG_ANAR_IEEE8023));
#endif /* ARDUINO_MIMXRT1060_EVKB */
#endif /* (FNET_CFG_CPU_MIMXRT1052 || FNET_CFG_CPU_MIMXRT1062) & KSZ8081RNB PHY */

    return FNET_OK;

}

/* If vector table is in ROM, pre-install FNET ISR for ENET Receive Frame interrupt*/
#if !FNET_CFG_CPU_VECTOR_TABLE_IS_IN_RAM
void ENET_IRQHandler (void)
{
    FNET_ISR_HANDLER();
}
#endif

#endif /* FNET_MIMXRT && FNET_CFG_CPU_ETH0 */
