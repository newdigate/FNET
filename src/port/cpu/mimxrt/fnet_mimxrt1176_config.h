/**************************************************************************
*
* Copyright 2026 by Nicholas Newdigate. FNET Community.
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
*  MIMXRT1176 (NXP MIMXRT1170-EVKB) specific configuration file.
*
***************************************************************************/

/************************************************************************
 * !!!DO NOT MODIFY THIS FILE!!!
 ************************************************************************/

#ifndef _FNET_MIMXRT1176_CONFIG_H_
#define _FNET_MIMXRT1176_CONFIG_H_

#define FNET_MIMXRT                                     (1)

/* CM7 core clock frequency: in Hz (OverDrive). */
#ifndef FNET_CFG_CPU_CLOCK_HZ
    #define FNET_CFG_CPU_CLOCK_HZ                       (996000000U)
#endif

/* Maximum number of incoming frames buffered by the Ethernet module. */
#ifndef FNET_CFG_CPU_ETH_RX_BUFS_MAX
    #define FNET_CFG_CPU_ETH_RX_BUFS_MAX                (4)
#endif

/* RT1170-EVKB 10/100 port: RTL8201 PHY at MDIO address 3
   (HW-verified milestone 1: PHYID 1C:C816). */
#ifndef FNET_CFG_CPU_ETH0_PHY_ADDR
    #define FNET_CFG_CPU_ETH0_PHY_ADDR                  (3)
#endif

/* No flash driver so far. */
#define FNET_CFG_CPU_FLASH                              (0)

/* ENET_1G / ENET_QOS are out of scope — single interface. */
#define FNET_CFG_CPU_ETH1                               (0)

/* FNET_CFG_TIMER_ALT=1 (application timer via millis()) — HW timers unused. */
#ifndef FNET_CFG_MIMXRT_TIMER_PIT
    #define FNET_CFG_MIMXRT_TIMER_PIT                   (0)
#endif
#ifndef FNET_CFG_MIMXRT_TIMER_QTMR
    #define FNET_CFG_MIMXRT_TIMER_QTMR                  (1)
#endif

/* HW checksum offload OFF: the QEMU imx.enet model does not implement TX
   checksum insertion — offloaded (zeroed) checksums would be dropped by SLIRP
   and the gates could never pass.  Software checksums everywhere; enabling
   offload on real HW is a later, HW-only experiment. */
#ifndef FNET_CFG_CPU_ETH_HW_TX_IP_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_TX_IP_CHECKSUM          (0)
#endif
#ifndef FNET_CFG_CPU_ETH_HW_TX_PROTOCOL_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_TX_PROTOCOL_CHECKSUM    (0)
#endif
#ifndef FNET_CFG_CPU_ETH_HW_RX_IP_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_RX_IP_CHECKSUM          (0)
#endif
#ifndef FNET_CFG_CPU_ETH_HW_RX_PROTOCOL_CHECKSUM
    #define FNET_CFG_CPU_ETH_HW_RX_PROTOCOL_CHECKSUM    (0)
#endif

/* Discard frames with MAC layer errors. */
#ifndef FNET_CFG_CPU_ETH_HW_RX_MAC_ERR
    #define FNET_CFG_CPU_ETH_HW_RX_MAC_ERR              (1)
#endif

/* ENET DMA buffer descriptors + buffers go to OCRAM via the core's DMAMEM
   section (.dmabuffers, zero-initialized by startup): the D-cache is OFF in
   this core (coherent without maintenance, which fnet_fec.c never performs)
   and erratum ERR050396 forbids ENET DMA into CM7 TCM regardless. */
#ifndef FNET_CFG_CPU_NONCACHEABLE_SECTION
    #define FNET_CFG_CPU_NONCACHEABLE_SECTION           ".dmabuffers"
#endif

#endif /* _FNET_MIMXRT1176_CONFIG_H_ */
