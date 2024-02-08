#include "common.h"

gnrc_netif_t *netif;
uint8_t null_deveui[8] = {0,0,0,0,0,0,0,0};
static uint8_t msg[222];

static const shell_command_t shell_commands[] =
{
    { "cpuid",          "print CPU ID",                       cpuid_cmd     },
    { "loramac_save",   "save LoRaWAN MAC params to FRAM",    loramac_save  },
    { "loramac_erase",  "erase LoRaWAN MAC params from FRAM", loramac_erase },
    { "loramac_dump",   "dump LoRaWAN MAC params from FRAM",  loramac_dump  },
    { "sleep",          "enter deep sleep",                   sleep_cmd     },
    { NULL,             NULL,                                 NULL          }
};

int main(void)
{
    int enter_shell = 0;

    gpio_init(TCXO_PWR_PIN, GPIO_OUT);
    gpio_set(TCXO_PWR_PIN);

    gpio_init(BTN0_PIN, BTN0_MODE);
    if (gpio_read(BTN0_PIN) == 0) {
        enter_shell = 1;
    }

    fram_init();

    /* Receive LoRaWAN packets in GNRC pktdump */
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);

    netif_t *iface = netif_get_by_name("3");
    netif = container_of(iface, gnrc_netif_t, netif);
    uint8_t value = CONFIG_LORAMAC_DEFAULT_DR;
    if (netif_set_opt(iface, NETOPT_LORAWAN_DR, 0, &value, sizeof(value)) != 0) {
        puts("WARNING: cannot set datarate");
    }
    if (!enter_shell) {
        restore_loramac();
        if (memcmp(netif->lorawan.deveui, null_deveui, sizeof(null_deveui)) != 0) {
            if (netif->lorawan.mac.mlme.activation == MLME_ACTIVATION_NONE) {
                puts("Trying to join LoRaWAN network");
                netopt_enable_t en = NETOPT_ENABLE;
                if (netif_set_opt(iface, NETOPT_LINK, 0, &en, sizeof(en)) < 0) {
                    puts("ERROR: unable to set link up");
                    enter_shell = 1;
                } else {
                    // TODO: optimize wait time
                    ztimer_sleep(ZTIMER_MSEC, 2000);
                }
            }
        } else {
            enter_shell = 1;
        }
    }

    if (enter_shell) {
        puts("Initialization successful - starting the shell now");

        char line_buf[SHELL_DEFAULT_BUFSIZE];
        shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    } else {
        puts("Initialization successful - starting measure cycle");

        init_sensors();
        int len = read_sensors(msg, sizeof(msg));
        deinit_sensors();

        if (len > 0) {
            printf("Will send:\n");
            od_hex_dump(msg, len, 0);
            if (send_message(msg, len) == 0) {
                puts("Packet sent, now waiting for RX windows.");
                // TODO: optimize wait time
                ztimer_sleep(ZTIMER_MSEC, 5000);
            } else {
                puts("Error sending message.");
            }
        } else {
            puts("Error reading sensors.");
        }

        // enter deep sleep
        puts("Sleeping...");
        int sleep_secs = 30;
        fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
        fram_off();
        gpio_clear(ACME0_POWER_PIN);

        netopt_state_t state = NETOPT_STATE_SLEEP;
        netif_set_opt(iface, NETOPT_STATE, 0, &state, sizeof(netopt_state_t));
        gpio_clear(TCXO_PWR_PIN);
        gpio_clear(TX_OUTPUT_SEL_PIN);
        saml21_extwake_t extwake = { .pin=EXTWAKE_PIN6, .polarity=EXTWAKE_HIGH, .flags=EXTWAKE_IN_PU };
        saml21_backup_mode_enter(0, extwake, sleep_secs, 1);
    }

    return 0;
}
