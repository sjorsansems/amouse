/*
 * amconfig - advanced DOS configuration tool for amouse over serial COM port.
 * Features: automatic COM port scan, interactive menu UI.
 *
 * Uses direct 16550 UART register I/O (same as DOS mouse drivers and Putty).
 * Serial settings: 1200, 7N1 (amouse defaults).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Standard 16550 UART base addresses for COM1..COM4 */
static const unsigned int COM_BASE[4] = { 0x3F8, 0x2F8, 0x3E8, 0x2E8 };
static const char *COM_NAMES[4] = { "COM1", "COM2", "COM3", "COM4" };

/* 16550 register offsets */
#define UART_RBR  0
#define UART_THR  0
#define UART_DLL  0
#define UART_DLH  1
#define UART_IER  1
#define UART_FCR  2
#define UART_LCR  3
#define UART_MCR  4
#define UART_LSR  5

#define LSR_DR    0x01
#define LSR_THRE  0x20
#define LCR_DLAB  0x80

static unsigned int g_uart_base = 0;
static int g_uart_port = -1;

/* ===== PORT PARSING ===== */

static int parse_com_port(const char *s) {
    if (!s) return -1;
    if ((s[0] == 'C' || s[0] == 'c') &&
        (s[1] == 'O' || s[1] == 'o') &&
        (s[2] == 'M' || s[2] == 'm') &&
        s[3] >= '1' && s[3] <= '4' &&
        s[4] == '\0') {
        return (s[3] - '1'); /* 0-based: COM1=0, COM2=1, ... */
    }
    return -1;
}

/* ===== UART I/O ===== */

static void uart_init_1200_7n1(void) {
    unsigned int base = g_uart_base;
    outp(base + UART_IER, 0x00);
    outp(base + UART_FCR, 0xC7);
    outp(base + UART_LCR, LCR_DLAB);
    outp(base + UART_DLL, 0x60);
    outp(base + UART_DLH, 0x00);
    outp(base + UART_LCR, 0x02);
    outp(base + UART_MCR, 0x03);
}

static void uart_tx_char(unsigned char c) {
    unsigned int base = g_uart_base;
    while (!(inp(base + UART_LSR) & LSR_THRE)) {}
    outp(base + UART_THR, c);
}

static int uart_rx_ready(void) {
    return (inp(g_uart_base + UART_LSR) & LSR_DR) ? 1 : 0;
}

static unsigned char uart_rx_char(void) {
    return (unsigned char)inp(g_uart_base + UART_RBR);
}

static void uart_send_str(const char *s) {
    while (*s) {
        uart_tx_char((unsigned char)(*s));
        s++;
    }
}

static int uart_drain_rx_check(unsigned long max_wait_ms) {
    unsigned long tps = (unsigned long)CLOCKS_PER_SEC;
    unsigned long silence_ticks = (500UL * tps) / 1000UL;
    unsigned long max_deadline;
    unsigned long silence_deadline;
    unsigned char c;
    int got_any = 0;

    if (silence_ticks == 0) silence_ticks = 1;
    max_deadline     = (unsigned long)clock() + (max_wait_ms * tps) / 1000UL;
    silence_deadline = (unsigned long)clock() + silence_ticks;

    while ((unsigned long)clock() < max_deadline) {
        if (uart_rx_ready()) {
            c = uart_rx_char();
            got_any = 1;
            silence_deadline = (unsigned long)clock() + silence_ticks;
        } else if (got_any && (unsigned long)clock() >= silence_deadline) {
            break;
        }
    }
    return got_any;
}

static void uart_drain_rx(unsigned long max_wait_ms) {
    uart_drain_rx_check(max_wait_ms);
}

static int uart_open_console(void) {
    uart_tx_char('\b');
    uart_drain_rx(4000);
    return 1;
}

static void uart_close_console(void) {
    uart_send_str("0\r");
    uart_drain_rx(800);
}

/* ===== PORT SCAN ===== */

static int scan_ports(void) {
    int i;
    int got_response;

    printf("\n");
    printf("========================================\n");
    printf("  AMOUSE PORT SCANNER\n");
    printf("========================================\n\n");
    printf("Scanning COM ports for amouse...\n\n");

    for (i = 0; i < 4; i++) {
        printf("  Testing %s...", COM_NAMES[i]);
        fflush(stdout);

        g_uart_base = COM_BASE[i];
        g_uart_port = i;

        uart_init_1200_7n1();
        uart_tx_char('\b');
        got_response = uart_drain_rx_check(4000);  /* Pico polls every 1 sec, banner takes 2.4 sec at 1200 baud */

        if (got_response) {
            printf(" Found! <%s>\n", COM_NAMES[i]);
            return i;
        } else {
            printf(" No\n");
        }
    }

    printf("\nNo amouse found on any COM port.\n");
    return -1;
}

/* ===== MAIN MENU ===== */

static void show_menu(int port) {
    printf("\n");
    printf("========================================\n");
    printf("  AMOUSE CONFIGURATION UTILITY v2.0\n");
    printf("========================================\n");
    printf("Connected to: %s\n\n", COM_NAMES[port]);
    printf("1 - Show current settings\n");
    printf("2 - Set sensitivity (2-30)\n");
    printf("3 - Set mouse protocol (0-2)\n");
    printf("    0=MS two-button  1=Logitech  2=MS wheel\n");
    printf("4 - Swap left/right buttons\n");
    printf("5 - Save settings to flash\n");
    printf("0 - Exit\n");
    printf("\nEnter choice: ");
}

/* ===== MAIN ===== */

int main(int argc, char *argv[]) {
    int port;
    char choice;
    int value;
    char cmd[32];

    /* Check if COM port was specified on command line */
    if (argc > 1) {
        port = parse_com_port(argv[1]);
        if (port < 0) {
            printf("Usage: AMCONFIG [COMx]\n");
            printf("  COMx = COM1, COM2, COM3, or COM4\n");
            printf("  If COMx omitted, automatic port scan is performed.\n");
            return 1;
        }
        printf("\nDirect connect to %s...\n", COM_NAMES[port]);
        g_uart_base = COM_BASE[port];
        g_uart_port = port;
        uart_init_1200_7n1();
        printf("Connected.\n");
    } else {
        /* Scan for amouse */
        port = scan_ports();
        if (port < 0) {
            return 1;
        }
    }

    g_uart_base = COM_BASE[port];
    g_uart_port = port;
    uart_init_1200_7n1();

    /* Main loop */
    while (1) {
        show_menu(port);
        scanf(" %c", &choice);
        getchar();  /* consume newline */

        switch (choice) {
            case '1': {
                /* Show */
                printf("\nFetching settings...\n");
                uart_open_console();
                uart_send_str("2\r");
                uart_drain_rx(1000);
                uart_close_console();
                printf("\n[Settings displayed above]\n");
                break;
            }

            case '2': {
                /* Set sensitivity */
                printf("\nEnter sensitivity (2-30): ");
                scanf("%d", &value);
                getchar();
                if (value >= 2 && value <= 30) {
                    uart_open_console();
                    sprintf(cmd, "3 %d\r", value);
                    uart_send_str(cmd);
                    uart_drain_rx(1000);
                    uart_close_console();
                    printf("Sensitivity set to %d.\n", value);
                } else {
                    printf("Invalid value (must be 2-30).\n");
                }
                break;
            }

            case '3': {
                /* Set protocol */
                printf("\nEnter protocol (0=MS2btn, 1=Logitech, 2=Wheel): ");
                scanf("%d", &value);
                getchar();
                if (value >= 0 && value <= 2) {
                    uart_open_console();
                    sprintf(cmd, "4 %d\r", value);
                    uart_send_str(cmd);
                    uart_drain_rx(1000);
                    uart_close_console();
                    printf("Protocol set to %d.\n", value);
                } else {
                    printf("Invalid protocol (must be 0-2).\n");
                }
                break;
            }

            case '4': {
                /* Swap buttons */
                printf("\nSwap buttons? (0=normal, 1=swap): ");
                scanf("%d", &value);
                getchar();
                if (value == 0 || value == 1) {
                    uart_open_console();
                    sprintf(cmd, "5 %d\r", value);
                    uart_send_str(cmd);
                    uart_drain_rx(1000);
                    uart_close_console();
                    printf("Button config set to %d.\n", value);
                } else {
                    printf("Invalid value (must be 0 or 1).\n");
                }
                break;
            }

            case '5': {
                /* Save to flash */
                printf("\nSaving settings to flash...\n");
                uart_open_console();
                uart_send_str("6\r");
                uart_drain_rx(1000);
                uart_send_str("3\r");
                uart_drain_rx(2000);
                uart_send_str("0\r");
                uart_drain_rx(800);
                uart_close_console();
                printf("Settings saved.\n");
                break;
            }

            case '0': {
                printf("\nGoodbye!\n");
                return 0;
            }

            default:
                printf("Invalid choice.\n");
        }
    }

    return 0;
}
