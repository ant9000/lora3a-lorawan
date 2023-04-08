APPLICATION = lora3a-lorawan
BOARD ?= lora3a-h10
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
USEMODULE += saul_default
USEMODULE += fram

USEMODULE += netdev_default
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_lorawan
USEMODULE += gnrc_pktdump
USEMODULE += gnrc_txtsnd

USEMODULE += shell
USEMODULE += shell_cmds_default
USEMODULE += shell_extra_commands

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
ifndef CONFIG_GNRC_PKTBUF_SIZE
  CFLAGS += -DCONFIG_GNRC_PKTBUF_SIZE=512
endif

include $(RIOTBASE)/Makefile.include
