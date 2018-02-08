#include<stdint.h>

/*DEFINE*/
#define DELAY_SIZE 100

#define LPC_AHB_BASE  (0x50000000UL)
#define LPC_GPIO_0_BASE  (LPC_AHB_BASE + 0x00000)
#define LPC_GPIO_1_BASE  (LPC_AHB_BASE + 0x10000)
/* General Purpose Input/Output (GPIO) */
struct lpc_gpio {
    volatile uint32_t mask;       /* 0x00 : Pin mask, affects data, out, set, clear and invert */
    volatile uint32_t in;         /* 0x04 : Port data Register (R/-) */
    volatile uint32_t out;        /* 0x08 : Port output Register (R/W) */
    volatile uint32_t set;        /* 0x0C : Port output set Register (-/W) */
    volatile uint32_t clear;      /* 0x10 : Port output clear Register (-/W) */
    volatile uint32_t toggle;     /* 0x14 : Port output invert Register (-/W) */
    uint32_t reserved[2];
    volatile uint32_t data_dir;   /* 0x20 : Data direction Register (R/W) */
    /* [.....] */
};
#define LPC_GPIO_0      ((struct lpc_gpio *) LPC_GPIO_0_BASE)
#define LPC_GPIO_1      ((struct lpc_gpio *) LPC_GPIO_1_BASE)

#define nop() __asm volatile ("nop")

/* GPIO Ports */
#define LED_RED    5
#define LED_GREEN  4
/* Switch */
#define SWITCH_LED_RED    (1 << LED_RED)
#define SWITCH_LED_GREEN   (1 << LED_GREEN)

/* Cortex M0 core interrupt handlers */
void Reset_Handler(void);

void NMI_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));

void HardFault_Handler(void) __attribute__ ((weak, alias ("Dummy_Handler")));

void Dummy_Handler(void);

int main(void);

/* WATCHDOG */
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_WDT_BASE           (LPC_APB0_BASE + 0x04000)

extern unsigned int _end_text;
extern unsigned int _start_data;
extern unsigned int _end_data;
extern unsigned int _start_bss;
extern unsigned int _end_bss;

/* Watchdog Timer (WDT) */
struct lpc_watchdog
{
    volatile uint32_t mode;          /* 0x000 : Watchdog mode register (R/W) */
    volatile uint32_t timer_const;   /* 0x004 : Watchdog timer constant register (R/W) */
    volatile uint32_t feed_seqence;  /* 0x008 : Watchdog feed sequence register ( /W) */
    volatile const uint32_t timer_value;  /* 0x00C : Watchdog timer value register (R/ ) */
    volatile uint32_t clk_src_sel;   /* 0x010 : Wathdog Clock Source Selection Register (R/W) */
    volatile uint32_t warning_int_compare; /* 0x014 : Watchdog Warning Interrupt compare value. */
    volatile uint32_t window_compare;      /* 0x018 : Watchdog Window compare value. */
};

#define LPC_WDT         ((struct lpc_watchdog *) LPC_WDT_BASE)

/* Stop the watchdog */
void stop_watchdog(void)
{
    struct lpc_watchdog* wdt = LPC_WDT;
    wdt->mode = 0; /* Stop Watchdog timer, and do not block any other config */
    wdt->feed_seqence = 0xAA;
    wdt->feed_seqence = 0x55;
}

/* BOOTSTRAP */
extern unsigned int _end_stack;
void *vector_table[] __attribute__ ((section(".vectors"))) = {
        &_end_stack,
        Reset_Handler,
        NMI_Handler,
        HardFault_Handler,
        0,
        0, /* 5 */
        0,
        /* Entry 7 (8th entry) must contain the 2’s complement of the check-sum
           of table entries 0 through 6. This causes the checksum of the first 8
           table entries to be 0 */
        (void *)0xDEADBEEF, /* Actually, this is done using an external tool. */
        0,
        /* [....².] voir le code du module GPIO-Demo pour la suite */
};

void Dummy_Handler(void) {
    while (1);
}

void Reset_Handler(void)
{
    unsigned int *src, *dst;

    /* Copy data section from flash to RAM */
    src = &_end_text;
    dst = &_start_data;
    while (dst < &_end_data)
        *dst++ = *src++;

    /* Clear the bss section */
    dst = &_start_bss;
    while (dst < &_end_bss)
        *dst++ = 0;

    /* Our program entry ! */
    main();
}


/* PROGRAM */
void system_init(void) {
    stop_watchdog();
}


int main(void) {
    struct lpc_gpio *gpio1 = LPC_GPIO_1;
    static int memoire[DELAY_SIZE] = {0};
    int i = 0;
    /* Micro-controller init */
    system_init();
    /* Configure the Status Led pins */
    gpio1->data_dir |= (1 << LED_GREEN) | (1 << LED_RED);
    /* Turn Green Led ON */
    gpio1->set = SWITCH_LED_GREEN;
    gpio1->clear = SWITCH_LED_RED;

    while (1) {
        /* Make a backup of current leds state */
        memoire[i++] = gpio1->out;
        if (i >= DELAY_SIZE) {
            i = 0;
        }
        gpio1->toggle = SWITCH_LED_GREEN;
        /* Change the red led according to the oldest green state we have */
        if (memoire[i] & (1 << LED_GREEN)) {
            gpio1->set = (1 << LED_RED);
        } else {
            gpio1->clear = (1 << LED_RED);
        }
    }
}
