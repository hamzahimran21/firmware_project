/*
 * ATSAMS70Q21 Register Definitions
 * Minimal definitions for GPIO, SPI, PMC (Power Management), UART
 */

#ifndef SAMS70_H
#define SAMS70_H

#include <stdint.h>

/* Base addresses */
#define PERIPH_BASE         0x40000000UL

/* Power Management Controller (PMC) */
#define PMC_BASE            (PERIPH_BASE + 0x000E0600UL)
#define PMC_SCER            (*(volatile uint32_t *)(PMC_BASE + 0x00))
#define PMC_SCDR            (*(volatile uint32_t *)(PMC_BASE + 0x04))
#define PMC_PCER0           (*(volatile uint32_t *)(PMC_BASE + 0x10))
#define PMC_PCDR0           (*(volatile uint32_t *)(PMC_BASE + 0x14))
#define PMC_PCSR0           (*(volatile uint32_t *)(PMC_BASE + 0x18))
#define CKGR_MOR            (*(volatile uint32_t *)(PMC_BASE + 0x20))
#define CKGR_MCFR           (*(volatile uint32_t *)(PMC_BASE + 0x24))
#define CKGR_PLLAR          (*(volatile uint32_t *)(PMC_BASE + 0x28))
#define PMC_MCKR            (*(volatile uint32_t *)(PMC_BASE + 0x30))
#define PMC_SR              (*(volatile uint32_t *)(PMC_BASE + 0x68))

/* PMC Peripheral IDs */
#define ID_PIOA             10
#define ID_PIOB             11
#define ID_PIOC             12
#define ID_PIOD             16
#define ID_SPI0             21
#define ID_UART0            7
#define ID_UART1            8

/* PMC register bit definitions */
#define PMC_MOR_KEY         (0x37 << 16)
#define PMC_MOR_MOSCXTEN    (1 << 0)
#define PMC_MOR_MOSCXTBY    (1 << 1)
#define PMC_MOR_MOSCRCEN    (1 << 3)
#define PMC_MOR_MOSCSEL     (1 << 24)

#define PMC_PLLAR_ONE       (1 << 29)
#define PMC_PLLAR_DIVA(x)   ((x) & 0xFF)
#define PMC_PLLAR_MULA(x)   (((x) & 0x7FF) << 16)
#define PMC_PLLAR_PLLACOUNT(x) (((x) & 0x3F) << 8)

#define PMC_MCKR_CSS_PLLA   (2 << 0)
#define PMC_MCKR_CSS_MAIN   (1 << 0)
#define PMC_MCKR_PRES_CLK   (0 << 4)
#define PMC_MCKR_PRES_CLK_2 (1 << 4)
#define PMC_MCKR_MDIV_PCK_DIV2 (1 << 8)

#define PMC_SR_MOSCXTS      (1 << 0)
#define PMC_SR_LOCKA        (1 << 1)
#define PMC_SR_MCKRDY       (1 << 3)

/* Parallel I/O Controller (PIO) */
#define PIOA_BASE           (PERIPH_BASE + 0x000E0E00UL)
#define PIOB_BASE           (PERIPH_BASE + 0x000E1000UL)
#define PIOC_BASE           (PERIPH_BASE + 0x000E1200UL)
#define PIOD_BASE           (PERIPH_BASE + 0x000E1400UL)

typedef struct {
    volatile uint32_t PIO_PER;      /* 0x00 PIO Enable */
    volatile uint32_t PIO_PDR;      /* 0x04 PIO Disable */
    volatile uint32_t PIO_PSR;      /* 0x08 PIO Status */
    uint32_t reserved0;
    volatile uint32_t PIO_OER;      /* 0x10 Output Enable */
    volatile uint32_t PIO_ODR;      /* 0x14 Output Disable */
    volatile uint32_t PIO_OSR;      /* 0x18 Output Status */
    uint32_t reserved1;
    volatile uint32_t PIO_IFER;     /* 0x20 Glitch Input Filter Enable */
    volatile uint32_t PIO_IFDR;     /* 0x24 Glitch Input Filter Disable */
    volatile uint32_t PIO_IFSR;     /* 0x28 Glitch Input Filter Status */
    uint32_t reserved2;
    volatile uint32_t PIO_SODR;     /* 0x30 Set Output Data */
    volatile uint32_t PIO_CODR;     /* 0x34 Clear Output Data */
    volatile uint32_t PIO_ODSR;     /* 0x38 Output Data Status */
    volatile uint32_t PIO_PDSR;     /* 0x3C Pin Data Status */
    volatile uint32_t PIO_IER;      /* 0x40 Interrupt Enable */
    volatile uint32_t PIO_IDR;      /* 0x44 Interrupt Disable */
    volatile uint32_t PIO_IMR;      /* 0x48 Interrupt Mask */
    volatile uint32_t PIO_ISR;      /* 0x4C Interrupt Status */
    volatile uint32_t PIO_MDER;     /* 0x50 Multi-driver Enable */
    volatile uint32_t PIO_MDDR;     /* 0x54 Multi-driver Disable */
    volatile uint32_t PIO_MDSR;     /* 0x58 Multi-driver Status */
    uint32_t reserved3;
    volatile uint32_t PIO_PUDR;     /* 0x60 Pull-up Disable */
    volatile uint32_t PIO_PUER;     /* 0x64 Pull-up Enable */
    volatile uint32_t PIO_PUSR;     /* 0x68 Pull-up Status */
    uint32_t reserved4;
    volatile uint32_t PIO_ABCDSR[2]; /* 0x70-0x74 Peripheral ABCD Select */
    uint32_t reserved5[2];
    volatile uint32_t PIO_IFSCDR;   /* 0x80 Input Filter Slow Clock Disable */
    volatile uint32_t PIO_IFSCER;   /* 0x84 Input Filter Slow Clock Enable */
    volatile uint32_t PIO_IFSCSR;   /* 0x88 Input Filter Slow Clock Status */
    volatile uint32_t PIO_SCDR;     /* 0x8C Slow Clock Divider Debouncing */
    volatile uint32_t PIO_PPDDR;    /* 0x90 Pad Pull-down Disable */
    volatile uint32_t PIO_PPDER;    /* 0x94 Pad Pull-down Enable */
    volatile uint32_t PIO_PPDSR;    /* 0x98 Pad Pull-down Status */
} PIO_TypeDef;

#define PIOA                ((PIO_TypeDef *)PIOA_BASE)
#define PIOB                ((PIO_TypeDef *)PIOB_BASE)
#define PIOC                ((PIO_TypeDef *)PIOC_BASE)
#define PIOD                ((PIO_TypeDef *)PIOD_BASE)

/* SPI Controller */
#define SPI0_BASE           (PERIPH_BASE + 0x00008000UL)

typedef struct {
    volatile uint32_t SPI_CR;       /* 0x00 Control Register */
    volatile uint32_t SPI_MR;       /* 0x04 Mode Register */
    volatile uint32_t SPI_RDR;      /* 0x08 Receive Data Register */
    volatile uint32_t SPI_TDR;      /* 0x0C Transmit Data Register */
    volatile uint32_t SPI_SR;       /* 0x10 Status Register */
    volatile uint32_t SPI_IER;      /* 0x14 Interrupt Enable */
    volatile uint32_t SPI_IDR;      /* 0x18 Interrupt Disable */
    volatile uint32_t SPI_IMR;      /* 0x1C Interrupt Mask */
    uint32_t reserved0[4];
    volatile uint32_t SPI_CSR[4];   /* 0x30-0x3C Chip Select Register */
} SPI_TypeDef;

#define SPI0                ((SPI_TypeDef *)SPI0_BASE)

/* SPI Control Register bits */
#define SPI_CR_SPIEN        (1 << 0)
#define SPI_CR_SPIDIS       (1 << 1)
#define SPI_CR_SWRST        (1 << 7)

/* SPI Mode Register bits */
#define SPI_MR_MSTR         (1 << 0)
#define SPI_MR_PS           (1 << 1)
#define SPI_MR_MODFDIS      (1 << 4)
#define SPI_MR_PCS(x)       (((x) & 0xF) << 16)

/* SPI Status Register bits */
#define SPI_SR_RDRF         (1 << 0)
#define SPI_SR_TDRE         (1 << 1)
#define SPI_SR_TXEMPTY      (1 << 9)

/* SPI Chip Select Register bits */
#define SPI_CSR_CPOL        (1 << 0)
#define SPI_CSR_NCPHA       (1 << 1)
#define SPI_CSR_CSNAAT      (1 << 2)   /* CS Not Active After Transfer */
#define SPI_CSR_CSAAT       (1 << 3)   /* CS Active After Transfer */
#define SPI_CSR_BITS_8      (0 << 4)
#define SPI_CSR_BITS_16     (8 << 4)
#define SPI_CSR_SCBR(x)     (((x) & 0xFF) << 8)
#define SPI_CSR_DLYBS(x)    (((x) & 0xFF) << 16)
#define SPI_CSR_DLYBCT(x)   (((x) & 0xFF) << 24)

/* UART Controller */
#define UART0_BASE          (PERIPH_BASE + 0x000E0800UL)

typedef struct {
    volatile uint32_t UART_CR;      /* 0x00 Control Register */
    volatile uint32_t UART_MR;      /* 0x04 Mode Register */
    volatile uint32_t UART_IER;     /* 0x08 Interrupt Enable */
    volatile uint32_t UART_IDR;     /* 0x0C Interrupt Disable */
    volatile uint32_t UART_IMR;     /* 0x10 Interrupt Mask */
    volatile uint32_t UART_SR;      /* 0x14 Status Register */
    volatile uint32_t UART_RHR;     /* 0x18 Receive Holding Register */
    volatile uint32_t UART_THR;     /* 0x1C Transmit Holding Register */
    volatile uint32_t UART_BRGR;    /* 0x20 Baud Rate Generator */
} UART_TypeDef;

#define UART0               ((UART_TypeDef *)UART0_BASE)

/* UART Control Register bits */
#define UART_CR_RSTRX       (1 << 2)
#define UART_CR_RSTTX       (1 << 3)
#define UART_CR_RXEN        (1 << 4)
#define UART_CR_RXDIS       (1 << 5)
#define UART_CR_TXEN        (1 << 6)
#define UART_CR_TXDIS       (1 << 7)

/* UART Status Register bits */
#define UART_SR_RXRDY       (1 << 0)
#define UART_SR_TXRDY       (1 << 1)
#define UART_SR_TXEMPTY     (1 << 9)

/*
 * Watchdog Timer (WDT)
 * Used to auto-reset MCU if code hangs
 */
#define WDT_BASE            (PERIPH_BASE + 0x000E1850UL)

typedef struct {
    volatile uint32_t WDT_CR;       /* 0x00 Control Register */
    volatile uint32_t WDT_MR;       /* 0x04 Mode Register */
    volatile uint32_t WDT_SR;       /* 0x08 Status Register */
} WDT_TypeDef;

#define WDT                 ((WDT_TypeDef *)WDT_BASE)

/* WDT Control Register */
#define WDT_CR_WDRSTT       (1 << 0)        /* Restart watchdog */
#define WDT_CR_KEY          (0xA5 << 24)    /* Write key */

/* WDT Mode Register */
#define WDT_MR_WDV(x)       ((x) & 0xFFF)           /* Watchdog counter value */
#define WDT_MR_WDFIEN       (1 << 12)               /* Fault interrupt enable */
#define WDT_MR_WDRSTEN      (1 << 13)               /* Reset enable */
#define WDT_MR_WDRPROC      (1 << 14)               /* Reset processor */
#define WDT_MR_WDDIS        (1 << 15)               /* Disable watchdog */
#define WDT_MR_WDD(x)       (((x) & 0xFFF) << 16)   /* Watchdog delta value */
#define WDT_MR_WDDBGHLT     (1 << 28)               /* Halt in debug */
#define WDT_MR_WDIDLEHLT    (1 << 29)               /* Halt in idle */

/* WDT Status Register */
#define WDT_SR_WDUNF        (1 << 0)        /* Underflow */
#define WDT_SR_WDERR        (1 << 1)        /* Error */

#endif /* SAMS70_H */
