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

// offset for fram persistent data
#define LORAMAC_OFFSET   0
#define SENSEAIR_OFFSET  (LORAMAC_OFFSET + sizeof(gnrc_netif_lorawan_t))
#define CONFIG_OFFSET    1024
#define CONFIG_SIZE      1024
#define CONFIG_MAGIC     0x0a

#ifndef VCC_LOW
#define VCC_LOW      2900
#endif
#ifndef VCC_HIGH
#define VCC_HIGH     3500
#endif
#ifndef VPANEL_LOW
#define VPANEL_LOW   2000
#endif
#ifndef VPANEL_HIGH
#define VPANEL_HIGH  4000
#endif
#ifndef SLEEP_SECS
#define SLEEP_SECS   60
#endif
#ifndef BME68X_ENERGY_MIN
#define BME68X_ENERGY_MIN     0
#endif
#ifndef SPS30_ENERGY_MIN
#define SPS30_ENERGY_MIN      0
#endif
#ifndef SENSEAIR_ENERGY_MIN
#define SENSEAIR_ENERGY_MIN   0
#endif

typedef struct {
  uint8_t magic;
  int16_t vcc_low;
  int16_t vcc_high;
  int16_t vpanel_low;
  int16_t vpanel_high;
  uint16_t sleep_secs;
  uint8_t bme68x_energy_min;
  uint8_t sps30_energy_min;
  uint8_t senseair_energy_min;
  // ...
} h10_config_t;

typedef struct {
  uint8_t storage  :4;
  uint8_t charging :4;
} energy_t;

typedef union {
  energy_t levels;
  uint8_t value;
} energy_state_t;

typedef struct {
  h10_config_t config;
  int16_t vcc;
  int16_t vpanel;
  energy_state_t energy_state;
  uint16_t sleep_secs;
  // ...
} h10_state_t;

extern h10_state_t h10_state;

void compute_state(void);

int loramac_restore(int argc, char **argv);
int loramac_save(int argc, char **argv);
int loramac_erase(int argc, char **argv);
int loramac_dump(int argc, char **argv);
int config_erase(int argc, char **argv);
int config_dump(int argc, char **argv);
int config_set(int argc, char **argv);
int sleep_cmd(int argc, char **argv);
int cpuid_cmd(int argc, char **argv);

int init_sensors(void);
int read_sensors(uint8_t *msg, size_t len);
int deinit_sensors(void);

typedef void (*radio_cb_t)(uint8_t fport, const uint8_t *payload, size_t size);
gnrc_netif_t *radio_init(radio_cb_t cb);
int restore_loramac(void);
void save_loramac(void);
uint32_t loramac_frame_counter(void);
int send_message(uint8_t *buffer, size_t len);

#endif
