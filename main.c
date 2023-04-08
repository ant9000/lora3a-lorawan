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

#include "saml21_cpu_debug.h"
#include "periph/gpio.h"

#include "fram.h"

static gnrc_netif_t *netif;

int loramac_save(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    fram_write(0, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
    puts("LoRaWAN MAC parameters saved");
    return 0;
}

int loramac_erase(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    fram_erase();
    puts("LoRaWAN MAC parameters cleared");
    return 0;
}

static const shell_command_t shell_commands[] =
{
    { "loramac_save",   "save LoRaWAN MAC params to FRAM",    loramac_save  },
    { "loramac_erase",  "erase LoRaWAN MAC params from FRAM", loramac_erase },
    { NULL,             NULL,                                 NULL          }
};

int main(void)
{
    gpio_init(TCXO_PWR_PIN, GPIO_OUT);
    gpio_set(TCXO_PWR_PIN);

    /* start the shell */
    puts("Initialization successful - starting the shell now");

    /* Receive LoRaWAN packets in GNRC pktdump */
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);

    netif_t *iface = netif_get_by_name("3");
    netif = container_of(iface, gnrc_netif_t, netif);
    printf("netif->lorawan: %p (%d bytes)\n", &netif->lorawan, sizeof(netif->lorawan));

    /* read lorawan from FRAM */
    gnrc_netif_lorawan_t lorawan;
    fram_init();
    fram_read(0, &lorawan, sizeof(lorawan));
    if (memcmp(lorawan.deveui, netif->lorawan.deveui, sizeof(netif->lorawan.deveui)) == 0) {
        puts("Found LoRaWAN params in FRAM, restoring");
        memcpy(netif->lorawan.appskey, lorawan.appskey, sizeof(netif->lorawan.appskey));
        memcpy(netif->lorawan.fnwksintkey, lorawan.fnwksintkey, sizeof(netif->lorawan.fnwksintkey));
        netif->lorawan.mac.mcps.fcnt = lorawan.mac.mcps.fcnt;
        netif->lorawan.mac.mcps.fcnt_down = lorawan.mac.mcps.fcnt_down;
        memcpy(netif->lorawan.mac.mcps.mhdr_mic, lorawan.mac.mcps.mhdr_mic, sizeof(netif->lorawan.mac.mcps.mhdr_mic));
        netif->lorawan.mac.mlme.activation = lorawan.mac.mlme.activation;
        netif->lorawan.mac.mlme.nid = lorawan.mac.mlme.nid;
        memcpy(netif->lorawan.mac.mlme.dev_nonce, lorawan.mac.mlme.dev_nonce, sizeof(netif->lorawan.mac.mlme.dev_nonce));
        memcpy(netif->lorawan.mac.channel, lorawan.mac.channel, sizeof(netif->lorawan.mac.channel));
        netif->lorawan.mac.channel_mask = lorawan.mac.channel_mask;
        netif->lorawan.mac.dev_addr = lorawan.mac.dev_addr;
        netif->lorawan.mac.dl_settings = lorawan.mac.dl_settings;
        memcpy(netif->lorawan.mac.dr_range, lorawan.mac.dr_range, sizeof(netif->lorawan.mac.dr_range));
        netif->lorawan.mac.rx_delay = lorawan.mac.rx_delay;
        netif->lorawan.mac.last_dr = lorawan.mac.last_dr;
        netif->lorawan.mac.last_chan_idx = lorawan.mac.last_chan_idx;
        mlme_confirm_t confirm;
        confirm.type = MLME_JOIN;
        confirm.status = GNRC_LORAWAN_REQ_STATUS_SUCCESS;
        gnrc_lorawan_mlme_confirm(&netif->lorawan.mac, &confirm);
    } else {
        puts("Trying to join LoRaWAN network");
        netopt_enable_t en = NETOPT_ENABLE;
        if (netif_set_opt(iface, NETOPT_LINK, 0, &en, sizeof(en)) < 0) {
            puts("ERROR: unable to set link up");
            return 1;
        }
        puts("Join packet sent, wait for the interface to be up");
    }

    puts("NOTE: remember to call loramac_save on first join and after each packet sent");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
