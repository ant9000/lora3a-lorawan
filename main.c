#include "common.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static uint8_t msg[222];

static const shell_command_t shell_commands[] =
{
    { "cpuid",          "print CPU ID",                         cpuid_cmd       },
    { "loramac_restore","restore LoRaWAN MAC params from FRAM", loramac_restore },
    { "loramac_save",   "save LoRaWAN MAC params to FRAM",      loramac_save    },
    { "loramac_erase",  "erase LoRaWAN MAC params from FRAM",   loramac_erase   },
    { "loramac_dump",   "dump LoRaWAN MAC params from FRAM",    loramac_dump    },
    { "config_erase",   "erase configuration params from FRAM", config_erase    },
    { "config_dump",    "dump configuration params from FRAM",  config_dump     },
    { "config_set",     "set configuration params into FRAM",   config_set      },
    { "sleep",          "enter deep sleep",                     sleep_cmd       },
#ifdef MODULE_SENSEAIR
    { "senseair_on",     "turn on Senseair",                    senseair_on     },
    { "senseair_off",    "turn off Senseair",                   senseair_off    },
    { "senseair_measure","read Senseair data",                  senseair_measure},
    { "senseair_calib",  "force Senseair ABC calibration",      senseair_calib  },
#endif
    { NULL,             NULL,                                   NULL            }
};

#define CONFIG_WRITE_PORT   42
void packet_received(uint8_t fport, const uint8_t *payload, size_t size) {
    puts("PACKET CALLBACK:");
    printf("fport: %d\n", fport);
    puts("payload:");
    od_hex_dump(payload, size, OD_WIDTH_DEFAULT);
    if ((fport == CONFIG_WRITE_PORT) && (size > 2)) {
        printf("[FRAM CONFIG] ");
        uint16_t address = (payload[0] << 8) + payload[1];
        size -= 2;
        if (address + size < CONFIG_SIZE) {
            if (fram_write(CONFIG_OFFSET + address, (uint8_t *)&(payload[2]), size) == 0) {
                printf("INFO: wrote %d bytes at address 0x%04x\n", size, address);
            } else {
                printf("ERROR: could not write %d bytes at address 0x%04x\n", size, address);
            }
        } else {
            printf("ERROR: size %d is beyond available space %d\n", size, CONFIG_SIZE);
        }
    }
}

int main(void)
{
    int enter_shell = 0;

    gpio_init(TCXO_PWR_PIN, GPIO_OUT);
    gpio_set(TCXO_PWR_PIN);

    gpio_init(BTN0_PIN, BTN0_MODE);
    if (gpio_read(BTN0_PIN) == 0) {
        enter_shell = 1;
    }

    compute_state();

    gnrc_netif_t *netif = radio_init(packet_received);
    if (!enter_shell) {
        if (restore_loramac()) {
            if (netif->lorawan.mac.mlme.activation == MLME_ACTIVATION_NONE) {
                puts("Trying to join LoRaWAN network");
                netopt_enable_t en = NETOPT_ENABLE;
                if (netif_set_opt(&netif->netif, NETOPT_LINK, 0, &en, sizeof(en)) < 0) {
                    puts("ERROR: unable to set link up");
                    enter_shell = 1;
                } else {
                    int res;
                    netopt_enable_t enabled;
                    ztimer_now_t start = ztimer_now(ZTIMER_MSEC), elapsed = start;
                    do {
                        res = netif_get_opt(&netif->netif, NETOPT_LINK, 0, &enabled, sizeof(enabled));
                        if (res >=0 && enabled == NETOPT_ENABLE) {
                            break;
                        }
                        ztimer_sleep(ZTIMER_MSEC, 100);
                        elapsed = ztimer_now(ZTIMER_MSEC) - start;
                    } while (elapsed < (CONFIG_LORAMAC_DEFAULT_JOIN_DELAY2 * 1000 + 1500));
                    if (enabled == NETOPT_ENABLE) {
                        printf("Join successful in %lu msec.\n", elapsed);
                    } else if (elapsed >= (CONFIG_LORAMAC_DEFAULT_JOIN_DELAY2 * 1000 + 1500)) {
                        printf("Join timed out after %lu msec.\n", elapsed);
                        goto sleep;
                    } else {
                        puts("Join failed: check network parameters.\n");
                        enter_shell = 1;
                    }
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
                ztimer_sleep(ZTIMER_MSEC, LORAMAC_DEFAULT_RX2_DELAY + 1500);
            } else {
                puts("Error sending message.");
            }
        } else {
            puts("Error reading sensors.");
        }

sleep:
        // enter deep sleep
        printf("Sleeping for %d s...\n", h10_state.sleep_secs);
        fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
        fram_off();

        netopt_state_t state = NETOPT_STATE_SLEEP;
        netif_set_opt(&netif->netif, NETOPT_STATE, 0, &state, sizeof(netopt_state_t));
        gpio_clear(TCXO_PWR_PIN);
        gpio_clear(TX_OUTPUT_SEL_PIN);
        saml21_extwake_t extwake = { .pin=EXTWAKE_PIN6, .polarity=EXTWAKE_HIGH, .flags=EXTWAKE_IN_PU };
        saml21_backup_mode_enter(0, extwake, h10_state.sleep_secs, 0);
    }

    return 0;
}
