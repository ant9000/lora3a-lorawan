/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 * @file
 * @brief       Test application for GNRC LoRaWAN
 *
 * @author      José Ignacio Alamos <jose.alamos@haw-hamburg.de>
 * @}
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "thread.h"
#include "shell.h"

#include "board.h"

#include "net/gnrc/netapi.h"
#include "net/gnrc/netif.h"

#include "net/gnrc/pktbuf.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/pktdump.h"
#include "net/loramac.h"
#include "net/gnrc/lorawan.h"

#include "saml21_backup_mode.h"
#include "periph/gpio.h"
#include "periph/cpuid.h"

#include "fram.h"
#include "od.h"

#include "phydat.h"
#include "saul_reg.h"

#include "net/gnrc.h"
#include "net/gnrc/netif/hdr.h"
#include "ztimer.h"

#define LORAMAC_OFFSET  0

static gnrc_netif_t *netif;

int loramac_save(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
    puts("LoRaWAN MAC parameters saved");
    return 0;
}

int loramac_erase(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    gnrc_netif_lorawan_t lorawan;
    memset(&lorawan, 0, sizeof(lorawan));
    fram_write(LORAMAC_OFFSET, (uint8_t *)&lorawan, sizeof(lorawan));
    puts("LoRaWAN MAC parameters cleared");
    return 0;
}

int loramac_dump(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    gnrc_netif_lorawan_t lorawan;
    fram_read(LORAMAC_OFFSET, &lorawan, sizeof(lorawan));
    puts("LoRaWAN MAC parameters:");
    od_hex_dump(&lorawan, sizeof(lorawan), 0);
    return 0;
}

int sleep_cmd(int argc, char **argv)
{
    if (argc != 2) {
        puts("usage: sleep <seconds>");
        return -1;
    }
    int sleep_secs = atoi(argv[1]);
    if (sleep_secs <= 0) {
        puts("Sleep time should be non negative.");
        return -1;
    }

    fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
    saml21_extwake_t extwake = { .pin=EXTWAKE_PIN6, .polarity=EXTWAKE_HIGH, .flags=EXTWAKE_IN_PU };
    saml21_backup_mode_enter(RADIO_OFF_NOT_REQUESTED, extwake, sleep_secs, 1);
    return 0;
}

int cpuid_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint8_t cid[CPUID_LEN];
    cpuid_get(cid);
    od_hex_dump(cid, sizeof(cid), 0);
    return 0;
}

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

    if (IS_USED(MODULE_SENSEAIR)) {
        extern void auto_init_senseair(void);
        auto_init_senseair();
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");

    /* Receive LoRaWAN packets in GNRC pktdump */
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);

    netif_t *iface = netif_get_by_name("3");
    netif = container_of(iface, gnrc_netif_t, netif);
    uint8_t zero[8] = {0,0,0,0,0,0,0,0};

    /* read lorawan from FRAM */
    gnrc_netif_lorawan_t lorawan;
    fram_init();
    fram_read(LORAMAC_OFFSET, &lorawan, sizeof(lorawan));
    if (memcmp(lorawan.deveui, zero, sizeof(zero)) != 0) {
        puts("Found LoRaWAN params in FRAM, restoring");
        memcpy(netif->lorawan.appskey, lorawan.appskey, sizeof(netif->lorawan.appskey));
        memcpy(netif->lorawan.fnwksintkey, lorawan.fnwksintkey, sizeof(netif->lorawan.fnwksintkey));
        memcpy(&netif->lorawan.mac.mcps, &lorawan.mac.mcps, sizeof(netif->lorawan.mac.mcps));
        netif->lorawan.mac.mcps.msdu = NULL;
        memcpy(&netif->lorawan.mac.mlme, &lorawan.mac.mlme, sizeof(netif->lorawan.mac.mlme));
        memcpy(netif->lorawan.mac.channel, lorawan.mac.channel, sizeof(netif->lorawan.mac.channel));
        netif->lorawan.mac.channel_mask = lorawan.mac.channel_mask;
        netif->lorawan.mac.dev_addr = lorawan.mac.dev_addr;
        netif->lorawan.mac.dl_settings = lorawan.mac.dl_settings;
        memcpy(netif->lorawan.mac.dr_range, lorawan.mac.dr_range, sizeof(netif->lorawan.mac.dr_range));
        netif->lorawan.mac.rx_delay = lorawan.mac.rx_delay;
        netif->lorawan.mac.last_dr = lorawan.mac.last_dr;
        netif->lorawan.mac.last_chan_idx = lorawan.mac.last_chan_idx;
        netif->lorawan.flags = lorawan.flags;
        netif->lorawan.demod_margin = lorawan.demod_margin;
        netif->lorawan.num_gateways = lorawan.num_gateways;
        netif->lorawan.datarate = lorawan.datarate;
        netif->lorawan.port = lorawan.port;
        netif->lorawan.ack_req = lorawan.ack_req;
        netif->lorawan.otaa = lorawan.otaa;

        memset(&lorawan, 0, sizeof(lorawan));
        fram_write(LORAMAC_OFFSET, (uint8_t *)&lorawan, sizeof(lorawan));

        mlme_confirm_t confirm;
        confirm.type = MLME_JOIN;
        confirm.status = GNRC_LORAWAN_REQ_STATUS_SUCCESS;
        gnrc_lorawan_mlme_confirm(&netif->lorawan.mac, &confirm);
    }
    if (memcmp(netif->lorawan.deveui, zero, sizeof(zero)) != 0) {
        if (netif->lorawan.mac.mlme.activation == MLME_ACTIVATION_NONE) {
            puts("Trying to join LoRaWAN network");
            netopt_enable_t en = NETOPT_ENABLE;
            if (netif_set_opt(iface, NETOPT_LINK, 0, &en, sizeof(en)) < 0) {
                puts("ERROR: unable to set link up");
                enter_shell = 1;
            }
        }
    } else {
        enter_shell = 1;
    }

    if (enter_shell) {
        char line_buf[SHELL_DEFAULT_BUFSIZE];
        shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    } else {
        // read sensor
        phydat_t res;
        char msg[16];
        saul_reg_t *dev = saul_reg_find_nth(4); // CO2
        if (dev == NULL) {
            puts("No SAUL devices present");
        } else {
            saul_reg_read(dev, &res);
            snprintf(msg, sizeof(msg), "co2:%u", res.val[0]);
            printf("Will send: '%s'\n", msg);
        }

        // send message
        uint8_t addr[GNRC_NETIF_L2ADDR_MAXLEN];
        size_t addr_len;
        gnrc_pktsnip_t *pkt, *hdr;
        gnrc_netif_hdr_t *nethdr;
        uint8_t flags = 0x00;

        /* parse address */
        addr_len = gnrc_netif_addr_from_str("42", addr);

        /* put packet together */
        pkt = gnrc_pktbuf_add(NULL, msg, strlen(msg), GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            printf("error: packet buffer full\n");
            return 1;
        }
        hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
        if (hdr == NULL) {
            printf("error: packet buffer full\n");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        pkt = gnrc_pkt_prepend(pkt, hdr);
        nethdr = (gnrc_netif_hdr_t *)hdr->data;
        nethdr->flags = flags;
        /* and send it */
        if (gnrc_netif_send(netif, pkt) < 1) {
            printf("error: unable to send\n");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        puts("Sent, now waiting for RX windows.");
        ztimer_sleep(ZTIMER_MSEC, 10000);

        // enter deep sleep
        puts("Sleeping...");
        int sleep_secs = 30;
        fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
        saml21_extwake_t extwake = { .pin=EXTWAKE_PIN6, .polarity=EXTWAKE_HIGH, .flags=EXTWAKE_IN_PU };
        saml21_backup_mode_enter(RADIO_OFF_NOT_REQUESTED, extwake, sleep_secs, 1);
    }

    return 0;
}
