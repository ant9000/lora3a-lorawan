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
#include "od.h"

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

static const shell_command_t shell_commands[] =
{
    { "loramac_save",   "save LoRaWAN MAC params to FRAM",    loramac_save  },
    { "loramac_erase",  "erase LoRaWAN MAC params from FRAM", loramac_erase },
    { "loramac_dump",   "dump LoRaWAN MAC params from FRAM",  loramac_dump  },
    { NULL,             NULL,                                 NULL          }
};

static netdev_event_cb_t original_iface_cb;
static void iface_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netif_t *netif = dev->context;

    if (event == NETDEV_EVENT_ISR) {
        event_post(&netif->evq[GNRC_NETIF_EVQ_INDEX_PRIO_LOW], &netif->event_isr);
    }
    else {
        original_iface_cb(dev, event);
        fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
    }
}

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
    original_iface_cb = netif->dev->event_callback;
    netif->dev->event_callback = iface_cb;

    /* read lorawan from FRAM */
    gnrc_netif_lorawan_t lorawan;
    fram_init();
    fram_read(LORAMAC_OFFSET, &lorawan, sizeof(lorawan));
    if (memcmp(lorawan.deveui, netif->lorawan.deveui, sizeof(netif->lorawan.deveui)) == 0) {
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

        mlme_confirm_t confirm;
        confirm.type = MLME_JOIN;
        confirm.status = GNRC_LORAWAN_REQ_STATUS_SUCCESS;
        gnrc_lorawan_mlme_confirm(&netif->lorawan.mac, &confirm);
    }
    if (netif->lorawan.mac.mlme.activation == MLME_ACTIVATION_NONE) {
        puts("Trying to join LoRaWAN network");
        netopt_enable_t en = NETOPT_ENABLE;
        if (netif_set_opt(iface, NETOPT_LINK, 0, &en, sizeof(en)) < 0) {
            puts("ERROR: unable to set link up");
        }
    }

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
