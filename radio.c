#include "common.h"

#include "thread.h"
#include "msg.h"
#include "net/gnrc.h"

static gnrc_netif_t *netif = NULL;
static kernel_pid_t radio_pid = KERNEL_PID_UNDEF;
static gnrc_netreg_entry_t entry;
static char _stack[THREAD_STACKSIZE_MAIN];
static msg_t _msg_queue[8];
static radio_cb_t radio_cb = NULL;

static void *_eventloop(void *arg);
static void _parse(gnrc_pktsnip_t *pkt);

gnrc_netif_t *radio_init(radio_cb_t cb) {
    radio_pid = thread_create(_stack, sizeof(_stack), THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, _eventloop, NULL, "radio");
    gnrc_netreg_entry_init_pid(&entry, GNRC_NETREG_DEMUX_CTX_ALL, radio_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &entry);
    netif_t *iface = netif_iter(NULL);
    netif = container_of(iface, gnrc_netif_t, netif);
    radio_cb = cb;
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

uint32_t loramac_frame_counter(void) {
    return netif->lorawan.mac.mcps.fcnt;
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

    /* and send it */
    if (gnrc_netif_send(netif, pkt) < 1) {
        printf("error: unable to send\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }

    return 0;
}

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;

    /* setup the message queue */
    msg_init_queue(_msg_queue, ARRAY_SIZE(_msg_queue));

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                _parse(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            default:
                break;
        }
    }

    /* never reached */
    return NULL;
}

static void _parse(gnrc_pktsnip_t *pkt)
{
    int snips = 0;
    int size = 0;
    uint8_t fport = 0;
    uint8_t *payload = NULL;
    size_t payload_size = 0;
    gnrc_pktsnip_t *snip = pkt;
    while (snip != NULL) {
        if (snip->type == GNRC_NETTYPE_NETIF) {
            gnrc_netif_hdr_t *hdr = snip->data;
            fport = *gnrc_netif_hdr_get_dst_addr(hdr);
        } else if (snip->type == GNRC_NETTYPE_UNDEF) {
            payload = (uint8_t *)snip->data;
            payload_size = snip->size;
        }
        ++snips;
        size += snip->size;
        snip = snip->next;
    }

    if (radio_cb) {
        radio_cb(fport, payload, payload_size);
    } else {
        puts("PKTDUMP: packet received");
        printf("fport: %d\n", fport);
        puts("payload:");
        od_hex_dump(payload, size, OD_WIDTH_DEFAULT);
    }
    gnrc_pktbuf_release(pkt);
}
