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
JOIN ?= OTAA
COMPRESS ?= 1
BSEC ?=

USEMODULE += saml21_cpu_debug
USEMODULE += saml21_backup_mode
USEMODULE += fram
USEMODULE += periph_cpuid
USEMODULE += periph_adc

USEMODULE += netdev_default
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_lorawan
USEMODULE += gnrc_txtsnd

USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += shell_extra_commands
USEMODULE += od_string

ifneq (,$(COMPRESS))
  USEMODULE += heatshrink_compression
endif

CFLAGS += -DSX127X_PARAM_PASELECT=SX127X_PA_RFO

CFLAGS += -DACME0_BUS_MODE=MODE_I2C -DFRAM_ENABLE_PIN=ACME0_POWER_PIN -DACME0_POWER_PIN_INITIAL_VALUE=0

DEVEUI ?= 0000000000000000
CFLAGS += -DCONFIG_LORAMAC_DEV_EUI_DEFAULT=\"$(DEVEUI)\"

ifeq ($(JOIN), OTAA)
  APPEUI ?= 0000000000000000
  APPKEY ?= 00000000000000000000000000000000
  CFLAGS += -DCONFIG_LORAMAC_DEFAULT_JOIN_PROCEDURE_OTAA
  CFLAGS += -DCONFIG_LORAMAC_APP_EUI_DEFAULT=\"$(APPEUI)\"
  CFLAGS += -DCONFIG_LORAMAC_APP_KEY_DEFAULT=\"$(APPKEY)\"
else
  DEVADDR ?= 00000000
  APPSKEY ?= 00000000000000000000000000000000
  NWKSKEY ?= 00000000000000000000000000000000
  CFLAGS += -DCONFIG_LORAMAC_DEFAULT_JOIN_PROCEDURE_ABP
  CFLAGS += -DCONFIG_LORAMAC_DEV_ADDR_DEFAULT=\"$(DEVADDR)\"
  CFLAGS += -DCONFIG_LORAMAC_APP_SKEY_DEFAULT=\"$(APPSKEY)\"
  CFLAGS += -DCONFIG_LORAMAC_NWK_SKEY_DEFAULT=\"$(NWKSKEY)\"
endif

CFLAGS += -DCONFIG_GNRC_NETIF_LORAWAN_NETIF_HDR
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_TX_MODE_UNCNF
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_TX_PORT=42
CFLAGS += -DCONFIG_LORAMAC_REGION_EU_868
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_TX_POWER_14
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_DR_5
CFLAGS += -DCONFIG_LORAMAC_DEFAULT_RX2_DR_3

USEMODULE += printf_float

SLEEP_SECS ?= 60
SENSEAIR_PORT ?=
SPS30_PORT ?= 1
SPS30_MEAS_COUNT ?= 1
SPS30_MEAS_SKIP ?= 0
BME68X_PORT ?= 2
BME68X_FP ?= 0
BME68X_NUMOF ?= 2

ifneq (, $(SLEEP_SECS))
  CFLAGS += -DSLEEP_SECS=$(SLEEP_SECS)
endif

ifneq (, $(SENSEAIR_PORT))
  USEMODULE += senseair
  CFLAGS += -DACME$(SENSEAIR_PORT)_BUS_MODE=MODE_I2C -DSENSEAIR_I2C_DEV=ACME$(SENSEAIR_PORT)_I2C_DEV
  CFLAGS += -DSENSEAIR_POWER_PIN=ACME$(SENSEAIR_PORT)_POWER_PIN -DACME$(SENSEAIR_PORT)_POWER_PIN_INITIAL_VALUE=0
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
  CFLAGS += -DSPS30_POWER_PIN=ACME$(SPS30_PORT)_POWER_PIN -DACME$(SPS30_PORT)_POWER_PIN_INITIAL_VALUE=0
  CFLAGS += -DSPS30_MEASURES_COUNT=$(SPS30_MEAS_COUNT) -DSPS30_MEASURES_SKIP=$(SPS30_MEAS_SKIP)
endif

ifneq (,$(BME68X_PORT))
  USEMODULE += bme68x bme68x_i2c
  ifeq ($(BME68X_FP),1)
    USEMODULE += bme68x_fp
  endif
  CFLAGS += -DACME$(BME68X_PORT)_BUS_MODE=MODE_I2C -DBME68X_POWER_PIN=ACME$(BME68X_PORT)_POWER_PIN
  CFLAGS += -DACME$(BME68X_PORT)_POWER_PIN_INITIAL_VALUE=0
  SENSORS := "{.ifsel=BME68X_I2C_INTF,.intf.i2c.dev=ACME$(BME68X_PORT)_I2C_DEV,.intf.i2c.addr=BME68X_I2C_ADDR_1}"
  PROF_LEN := 10
  TEMP_PROF := "{320,100,100,100,200,200,200,320,320,320}"
  DUR_PROF := "{700,280,1400,4200,700,700,700,700,700,700}"
  ifeq ($(BME68X_NUMOF), 2)
    SENSORS += ",{.ifsel=BME68X_I2C_INTF,.intf.i2c.dev=ACME$(BME68X_PORT)_I2C_DEV,.intf.i2c.addr=BME68X_I2C_ADDR_2}"
    TEMP_PROF += ",{320,100,100,100,200,200,200,320,320,320}"
    DUR_PROF += ",{700,280,1400,420,700,700,700,700,700,700}"
  endif
  CFLAGS += -DBME68X_PARAMS_I2C="${SENSORS}" -DBME68X_PROF_LEN=$(PROF_LEN) -DBME68X_TEMP_PROF="$(TEMP_PROF)" -DBME68X_DUR_PROF="$(DUR_PROF)"
endif

ifneq (,$(BSEC))
  USEMODULE += bosch_bsec
  USEMODULE += ztimer64_usec
  USEMODULE += ztimer_periph_rtc
  CFLAGS += -DBSEC_CONFIG=BME688_SEL_33V_300S_4D -DBSEC_SAMPLE_RATE=BSEC_SAMPLE_RATE_ULP
endif

include $(RIOTBASE)/Makefile.include
