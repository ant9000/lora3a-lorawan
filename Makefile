APPLICATION = lora3a-lorawan
BOARD ?= berta-h10
RIOTBASE ?= $(CURDIR)/../RIOT
LORA3ABASE ?= $(CURDIR)/../lora3a-boards
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += saml21_cpu_debug
USEMODULE += saml21_backup_mode
USEMODULE += fram
USEMODULE += periph_cpuid

USEMODULE += netdev_default
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_lorawan
USEMODULE += gnrc_pktdump
USEMODULE += gnrc_txtsnd

USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += shell_extra_commands
USEMODULE += od_string

# needed for floating point
CFLAGS += -DTHREAD_STACKSIZE_DEFAULT=2048

CFLAGS += -DSX127X_PARAM_PASELECT=SX127X_PA_RFO

DEVEUI ?= 0000000000000000
APPEUI ?= 0000000000000000
APPKEY ?= 00000000000000000000000000000000

CFLAGS += -DCONFIG_LORAMAC_DEV_EUI_DEFAULT=\"$(DEVEUI)\"
CFLAGS += -DCONFIG_LORAMAC_APP_EUI_DEFAULT=\"$(APPEUI)\"
CFLAGS += -DCONFIG_LORAMAC_APP_KEY_DEFAULT=\"$(APPKEY)\"

CFLAGS += -DCONFIG_GNRC_NETIF_LORAWAN_NETIF_HDR
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_JOIN_PROCEDURE_OTAA
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_TX_MODE_UNCNF
CFLAGS += -DCONFIG_LORAMAC_REGION_EU_868
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_TX_POWER_14
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_DR5
ifndef CONFIG_GNRC_PKTBUF_SIZE
  CFLAGS += -DCONFIG_GNRC_PKTBUF_SIZE=512
endif

USEMODULE += printf_float

CFLAGS += -DACME0_BUS_MODE=MODE_I2C

SENSEAIR_PORT ?=
SPS30_PORT ?= 1
BME688_PORT ?= 2

ifneq (, $(SENSEAIR_PORT))
  USEMODULE += senseair
  CFLAGS += -DSENSEAIR_I2C_DEV=ACME$(SENSEAIR_PORT)_I2C_DEV -DSENSEAIR_POWER_PIN=ACME$(SENSEAIR_PORT)_POWER_PIN
  ifeq ($(SENSEAIR_PORT), 0)
    CFLAGS += -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PA,19\)
  endif
  ifeq ($(SENSEAIR_PORT), 1)
    CFLAGS += -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PB,23\)
  endif
  ifeq ($(SENSEAIR_PORT), 2)
    CFLAGS += -DSENSEAIR_ENABLE_PIN=GPIO_PIN\(PA,7\)
  endif
endif

ifneq (,$(SPS30_PORT))
  USEMODULE += sps30
  CFLAGS += -DACME$(SPS30_PORT)_BUS_MODE=MODE_I2C -DSPS30_PARAM_I2C_DEV=ACME$(SPS30_PORT)_I2C_DEV
endif

ifneq (,$(BME688_PORT))
  USEMODULE += bme68x bme68x_i2c bme68x_fp
  CFLAGS += -DACME$(BME688_PORT)_BUS_MODE=MODE_I2C
  SENSOR1 := "{.ifsel=BME68X_I2C_INTF,.intf.i2c.dev=ACME$(BME688_PORT)_I2C_DEV,.intf.i2c.addr=BME68X_I2C_ADDR_1}"
  SENSOR2 := "{.ifsel=BME68X_I2C_INTF,.intf.i2c.dev=ACME$(BME688_PORT)_I2C_DEV,.intf.i2c.addr=BME68X_I2C_ADDR_2}"
  CFLAGS += -DBME68X_PARAMS_I2C="${SENSOR1},${SENSOR2}"
endif

include $(RIOTBASE)/Makefile.include
