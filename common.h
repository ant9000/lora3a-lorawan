#ifndef COMMON_H
#define COMMON_H

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
#include "periph/adc.h"

#include "fram.h"
#include "od.h"

#include "net/gnrc.h"
#include "net/gnrc/netif/hdr.h"
#include "ztimer.h"

#define LORAMAC_OFFSET   0
#define SENSEAIR_OFFSET  (LORAMAC_OFFSET + sizeof(gnrc_netif_lorawan_t))

#ifndef SLEEP_SECS
#define SLEEP_SECS 60
#endif

int loramac_restore(int argc, char **argv);
int loramac_save(int argc, char **argv);
int loramac_erase(int argc, char **argv);
int loramac_dump(int argc, char **argv);
int sleep_cmd(int argc, char **argv);
int cpuid_cmd(int argc, char **argv);

int init_sensors(void);
int read_sensors(uint8_t *msg, size_t len);
int deinit_sensors(void);

typedef void (*radio_cb_t)(uint8_t fport, const uint8_t *payload, size_t size);
gnrc_netif_t *radio_init(radio_cb_t cb);
int restore_loramac(void);
void save_loramac(void);
int send_message(uint8_t *buffer, size_t len);

#ifdef COMPRESS
int heatshrink_compress(const uint8_t *input, size_t input_len, uint8_t *output, size_t output_len);
#endif
#endif
