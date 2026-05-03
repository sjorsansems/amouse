/*
 * amcfg - minimal DOS configuration tool for amouse over serial COM port.
 *
 * Uses direct 16550 UART register I/O (same as DOS mouse drivers and Putty).
 * BIOS INT 14h is unreliable on many systems for COM2+ so we bypass it.
 * Serial settings are fixed to amouse console defaults: 1200, 7N1.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h>   /* inp() / outp() */
#include <time.h>

/* Standard 16550 UART base addresses for COM1..COM4 */
static const unsigned int COM_BASE[4] = { 0x3F8, 0x2F8, 0x3E8, 0x2E8 };

/* 16550 register offsets */
#define UART_RBR  0   /* Receive Buffer Register (read, DLAB=0) */
#define UART_THR  0   /* Transmit Holding Register (write, DLAB=0) */
#define UART_DLL  0   /* Divisor Latch Low (DLAB=1) */
#define UART_DLH  1   /* Divisor Latch High (DLAB=1) */
#define UART_IER  1   /* Interrupt Enable Register */
#define UART_FCR  2   /* FIFO Control Register */
#define UART_LCR  3   /* Line Control Register */
#define UART_MCR  4   /* Modem Control Register */
#define UART_LSR  5   /* Line Status Register */

#define LSR_DR    0x01  /* Data Ready */
#define LSR_THRE  0x20  /* Transmit Holding Register Empty */

#define LCR_DLAB  0x80  /* Divisor Latch Access Bit */

static unsigned int g_uart_base = 0;

static int parse_com_port(const char *s) {
    if (!s) return -1;
    if ((s[0] == 'C' || s[0] == 'c') &&
        (s[1] == 'O' || s[1] == 'o') &&
        (s[2] == 'M' || s[2] == 'm') &&
        s[3] >= '1' && s[3] <= '4' &&
        s[4] == '\0') {
        return (s[3] - '1'); /* 0-based index */
    }
    return -1;
}

/*
 * Initialise UART for 1200 baud, 7 data bits, no parity, 1 stop bit.
 * Divisor for 1200 baud from 1.8432 MHz crystal: 115200 / 1200 = 96 = 0x60.
 */
static void uart_init_1200_7n1(void) {
    unsigned int base = g_uart_base;
    /* Disable interrupts */
    outp(base + UART_IER, 0x00);
    /* Enable FIFO, clear TX/RX, 14-byte threshold */
    outp(base + UART_FCR, 0xC7);
    /* Set DLAB to access divisor latches */
    outp(base + UART_LCR, LCR_DLAB);
    /* 1200 baud: divisor = 96 */
    outp(base + UART_DLL, 0x60);
    outp(base + UART_DLH, 0x00);
    /* 7 data bits (0x02), no parity (00), 1 stop bit (0) = 0x02; clear DLAB */
    outp(base + UART_LCR, 0x02);
    /* RTS + DTR on */
    outp(base + UART_MCR, 0x03);
}

static void uart_tx_char(unsigned char c) {
    unsigned int base = g_uart_base;
    /* Wait until Transmit Holding Register is empty */
    while (!(inp(base + UART_LSR) & LSR_THRE)) {}
    outp(base + UART_THR, c);
}

static int uart_rx_ready(void) {
    return (inp(g_uart_base + UART_LSR) & LSR_DR) ? 1 : 0;
}

static unsigned char uart_rx_char(void) {
    return (unsigned char)inp(g_uart_base + UART_RBR);
}

static void serial_send_str(int port, const char *s) {
    (void)port;
    while (*s) {
        uart_tx_char((unsigned char)(*s));
        s++;
    }
}

/* Thin wrappers so drain_rx / open_console can keep using port arg */
static void bios_serial_tx_char(int port, unsigned char c) {
    (void)port;
    uart_tx_char(c);
}

static int bios_serial_rx_ready(int port) {
    (void)port;
    return uart_rx_ready();
}

static int bios_serial_rx_char(int port, unsigned char *out) {
    (void)port;
    if (out) *out = uart_rx_char();
    return 1;
}

/*
 * Drain RX bytes.
 * Waits up to max_wait_ms total.  Once data starts flowing, exits after
 * 300 ms of silence so all output at 1200 baud is captured before returning.
 * Uses (ms * CLOCKS_PER_SEC) / 1000 to convert ms to ticks, which is correct
 * for any CLOCKS_PER_SEC value (18, 100, 1000, ...).
 */
/* Verbose hex+ASCII dump, used by 'test' command */
static void drain_rx_verbose(int port, unsigned long max_wait_ms) {
    unsigned long tps = (unsigned long)CLOCKS_PER_SEC;
    /* 500 ms silence = robust detection of transmission end at 1200 baud */
    unsigned long silence_ticks = (500UL * tps) / 1000UL;
    unsigned long max_deadline;
    unsigned long silence_deadline;
    unsigned char c;
    int got_any = 0;
    int count = 0;

    if (silence_ticks == 0) silence_ticks = 1;
    max_deadline     = (unsigned long)clock() + (max_wait_ms * tps) / 1000UL;
    silence_deadline = (unsigned long)clock() + silence_ticks;

    while ((unsigned long)clock() < max_deadline) {
        if (bios_serial_rx_ready(port)) {
            bios_serial_rx_char(port, &c);
            printf("%02X(%c) ", (unsigned)c, (c >= 32 && c < 127) ? c : '.');
            count++;
            if (count % 8 == 0) puts("");
            got_any = 1;
            silence_deadline = (unsigned long)clock() + silence_ticks;
        } else if (got_any && (unsigned long)clock() >= silence_deadline) {
            break;
        }
    }
    if (count % 8 != 0) puts("");
    printf("[%d bytes received]\n", count);
}

static void drain_rx(int port, unsigned long max_wait_ms) {
    unsigned long tps = (unsigned long)CLOCKS_PER_SEC;
    unsigned long silence_ticks = (300UL * tps) / 1000UL;
    unsigned long max_deadline;
    unsigned long silence_deadline;
    unsigned char c;
    int got_any = 0;

    if (silence_ticks == 0) silence_ticks = 1;

    max_deadline     = (unsigned long)clock() + (max_wait_ms * tps) / 1000UL;
    silence_deadline = (unsigned long)clock() + silence_ticks;

    while ((unsigned long)clock() < max_deadline) {
        if (bios_serial_rx_ready(port)) {
            bios_serial_rx_char(port, &c);
            putchar((int)(c & 0x7F));    /* 7-bit serial: mask high bit */
            got_any = 1;
            silence_deadline = (unsigned long)clock() + silence_ticks;
        } else if (got_any && (unsigned long)clock() >= silence_deadline) {
            break;
        }
    }
}

static void open_console(int port) {
    /*
     * amouse enters config console when it receives '\b' on serial RX.
     * The Pico polls serial RX once per second, so we must wait at least
     * 1 s before the trigger is detected.  After detection it sends a
     * ~320-byte banner at 1200 baud (~2.4 s).  4000 ms covers both.
     */
    bios_serial_tx_char(port, '\b');
    drain_rx(port, 4000);
}

static void close_console(int port) {
    serial_send_str(port, "0\r");
    drain_rx(port, 800);
}

static void usage(void) {
    puts("amcfg - DOS config utility for amouse");
    puts("Usage:");
    puts("  AMCFG COMx show");
    puts("  AMCFG COMx speed <2..30> [save]");
    puts("  AMCFG COMx proto <0..2> [save]");
    puts("  AMCFG COMx swap [0|1] [save]");
    puts("  AMCFG COMx test    (debug: hex dump all received bytes)");
    puts("");
    puts("Examples:");
    puts("  AMCFG COM1 speed 11 save");
    puts("  AMCFG COM1 proto 2 save");
    puts("  AMCFG COM1 swap 1 save");
    puts("  AMCFG COM1 show");
}

static void do_save(int port) {
    /* Main menu -> Flash menu -> Write settings -> back to main menu */
    serial_send_str(port, "6\r");
    drain_rx(port, 1000);
    serial_send_str(port, "3\r");
    drain_rx(port, 2000);  /* Flash write can take a moment */
    serial_send_str(port, "0\r");
    drain_rx(port, 800);
}

int main(int argc, char **argv) {
    int port;
    int want_save = 0;

    if (argc < 3) {
        usage();
        return 1;
    }

    port = parse_com_port(argv[1]);
    if (port < 0) {
        puts("Error: first argument must be COM1..COM4");
        return 1;
    }

    g_uart_base = COM_BASE[port];
    uart_init_1200_7n1();

    if (strcmp(argv[2], "test") == 0) {
        printf("UART base=0x%03X  CLOCKS_PER_SEC=%ld\n",
               (unsigned)g_uart_base, (long)CLOCKS_PER_SEC);
        puts("Sending \\b, waiting 10s for response (hex dump):");
        bios_serial_tx_char(port, '\b');
        drain_rx_verbose(port, 10000);
        puts("Test done.");
        return 0;
    }

    open_console(port);

    if (strcmp(argv[2], "show") == 0) {
        serial_send_str(port, "2\r");
        drain_rx(port, 1000);
    } else if (strcmp(argv[2], "speed") == 0) {
        char cmd[32];
        if (argc < 4) {
            puts("Error: speed requires value 2..30");
            close_console(port);
            return 1;
        }
        sprintf(cmd, "3 %d\r", atoi(argv[3]));
        serial_send_str(port, cmd);
        drain_rx(port, 1000);
        if (argc >= 5 && strcmp(argv[4], "save") == 0) want_save = 1;
    } else if (strcmp(argv[2], "proto") == 0) {
        char cmd[32];
        if (argc < 4) {
            puts("Error: proto requires value 0..2");
            close_console(port);
            return 1;
        }
        sprintf(cmd, "4 %d\r", atoi(argv[3]));
        serial_send_str(port, cmd);
        drain_rx(port, 1000);
        if (argc >= 5 && strcmp(argv[4], "save") == 0) want_save = 1;
    } else if (strcmp(argv[2], "swap") == 0) {
        if (argc >= 4 && (argv[3][0] == '0' || argv[3][0] == '1') && argv[3][1] == '\0') {
            char cmd[16];
            sprintf(cmd, "5 %c\r", argv[3][0]);
            serial_send_str(port, cmd);
            if (argc >= 5 && strcmp(argv[4], "save") == 0) want_save = 1;
        } else {
            serial_send_str(port, "5\r"); /* toggle */
            if (argc >= 4 && strcmp(argv[3], "save") == 0) want_save = 1;
        }
        drain_rx(port, 1000);
    } else {
        puts("Error: unknown command");
        usage();
        close_console(port);
        return 1;
    }

    if (want_save) {
        do_save(port);
    }

    close_console(port);
    return 0;
}
