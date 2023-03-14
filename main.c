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

#include "saml21_cpu_debug.h"
#include "periph/gpio.h"

int main(void)
{
    gpio_set(TCXO_PWR_PIN);

    /* start the shell */
    puts("Initialization successful - starting the shell now");

    /* Receive LoRaWAN packets in GNRC pktdump */
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);

    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);

    netif_t *iface = netif_get_by_name("3");
    netopt_enable_t en = NETOPT_ENABLE;
    if (netif_set_opt(iface, NETOPT_LINK, 0, &en, sizeof(en)) < 0) {
        puts("ERROR: unable to set link up");
    }

//  saml21_cpu_debug();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
