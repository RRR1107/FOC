/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_SOC_IP_FEATURE_H
#define HPM_SOC_IP_FEATURE_H

/* GPTMR related feature */
#define HPM_IP_FEATURE_GPTMR_MONITOR 1
#define HPM_IP_FEATURE_GPTMR_OP_MODE 1
#define HPM_IP_FEATURE_GPTMR_CNT_MODE 1

/* UART related feature */
#define HPM_IP_FEATURE_UART_RX_IDLE_DETECT 1
#define HPM_IP_FEATURE_UART_FCRR 1
#define HPM_IP_FEATURE_UART_RX_EN 1
#define HPM_IP_FEATURE_UART_E00018_FIX 1
#define HPM_IP_FEATURE_UART_9BIT_MODE 1
#define HPM_IP_FEATURE_UART_ADDR_MATCH 1
#define HPM_IP_FEATURE_UART_TRIG_MODE 1
#define HPM_IP_FEATURE_UART_FINE_FIFO_THRLD 1
#define HPM_IP_FEATURE_UART_IIR2 1

/* I2C related feature */
#define HPM_IP_FEATURE_I2C_SUPPORT_RESET 1

/* SPI related feature */
#define HPM_IP_FEATURE_SPI_NEW_TRANS_COUNT 1
#define HPM_IP_FEATURE_SPI_CS_SELECT 1
#define HPM_IP_FEATURE_SPI_SUPPORT_DIRECTIO 1

/* DMAV2 related feature */
#define HPM_IP_FEATURE_DMAV2_BURST_IN_FIXED_TRANS 1
#define HPM_IP_FEATURE_DMAV2_BYTE_ORDER_SWAP 1

/* ADC16 related feature */
#define HPM_IP_FEATURE_ADC16_HAS_MOT_EN 1

/* DAO related feature */
#define HPM_IP_FEATURE_DAO_DATA_FORMAT_CONFIG 1

/* QEIV2 related feature */
#define HPM_IP_FEATURE_QEIV2_ONESHOT_MODE 1
#define HPM_IP_FEATURE_QEIV2_SW_RESTART_TRG 1
#define HPM_IP_FEATURE_QEIV2_TIMESTAMP 1
#define HPM_IP_FEATURE_QEIV2_ADC_THRESHOLD 1

/* RDC related feature */
#define HPM_IP_FEATURE_RDC_IIR 1

/* SEI related feature */
#define HPM_IP_FEATURE_SEI_RX_LATCH_FEATURE 1
#define HPM_IP_FEATURE_SEI_ASYNCHRONOUS_MODE_V2 1
#define HPM_IP_FEATURE_SEI_TIMEOUT_REWIND_FEATURE 1
#define HPM_IP_FEATURE_SEI_HAVE_DAT10_31 1
#define HPM_IP_FEATURE_SEI_HAVE_INTR64_255 1
#define HPM_IP_FEATURE_SEI_HAVE_CTRL2_12 1
#define HPM_IP_FEATURE_SEI_HAVE_PTCD 1

/* ENET related feature */
#define HPM_IP_FEATURE_ENET_HAS_MII_MODE 1

/* FFA related feature */
#define HPM_IP_FEATURE_FFA_FP32 1

#endif /* HPM_SOC_IP_FEATURE_H */