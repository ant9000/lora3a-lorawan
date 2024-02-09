#include "common.h"

int loramac_restore(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    restore_loramac();
    puts("LoRaWAN MAC parameters restored");
    return 0;
}

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

int sleep_cmd(int argc, char **argv)
{
    if (argc != 2) {
        puts("usage: sleep <seconds>");
        return -1;
    }
    int sleep_secs = atoi(argv[1]);
    if (sleep_secs <= 0) {
        puts("Sleep time should be non negative.");
        return -1;
    }

    fram_write(LORAMAC_OFFSET, (uint8_t *)&netif->lorawan, sizeof(netif->lorawan));
    saml21_extwake_t extwake = { .pin=EXTWAKE_PIN6, .polarity=EXTWAKE_HIGH, .flags=EXTWAKE_IN_PU };
    saml21_backup_mode_enter(RADIO_OFF_NOT_REQUESTED, extwake, sleep_secs, 1);
    return 0;
}

int cpuid_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint8_t cid[CPUID_LEN];
    cpuid_get(cid);
    od_hex_dump(cid, sizeof(cid), 0);
    return 0;
}

