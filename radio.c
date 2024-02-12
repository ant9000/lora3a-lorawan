#include "common.h"

static gnrc_netif_t *netif = NULL;

gnrc_netif_t *radio_init(void) {
    /* Receive LoRaWAN packets in GNRC pktdump */
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL, gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);
    netif_t *iface = netif_iter(NULL);
    netif = container_of(iface, gnrc_netif_t, netif);
    return netif;
}

void save_loramac(void) {
    fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
}

int restore_loramac(void) {
    uint8_t null_deveui[8] = {0,0,0,0,0,0,0,0};
    /* read lorawan from FRAM */
    gnrc_netif_lorawan_t lorawan;
    fram_read(LORAMAC_OFFSET, &lorawan, sizeof(lorawan));
    if (memcmp(lorawan.deveui, null_deveui, sizeof(null_deveui)) != 0) {
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
    return (memcmp(netif->lorawan.deveui, null_deveui, sizeof(null_deveui)) != 0);
}

int send_message(uint8_t *buffer, size_t len) {
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;

    uint8_t port = CONFIG_LORAMAC_DEFAULT_TX_PORT;

    /* put packet together */
    pkt = gnrc_pktbuf_add(NULL, buffer, len, GNRC_NETTYPE_UNDEF);
    if (pkt == NULL) {
        printf("error: packet buffer full\n");
        return 1;
    }
    hdr = gnrc_netif_hdr_build(NULL, 0, &port, sizeof(port));
    if (hdr == NULL) {
        printf("error: packet buffer full\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }
    pkt = gnrc_pkt_prepend(pkt, hdr);
    nethdr = (gnrc_netif_hdr_t *)hdr->data;
    nethdr->flags = flags;

    /* set power */
    uint16_t power = CONFIG_LORAMAC_DEFAULT_TX_POWER;
    netif_set_opt(&netif->netif, NETOPT_TX_POWER, 0, &power, sizeof(power));

    /* and send it */
    if (gnrc_netif_send(netif, pkt) < 1) {
        printf("error: unable to send\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }

    return 0;
}
