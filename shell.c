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
    save_loramac();
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

int config_erase(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    h10_config_t config;
    memset(&config, 0, sizeof(config));
    fram_write(CONFIG_OFFSET, (uint8_t *)&config, sizeof(config));
    puts("Configuration parameters cleared");
    return 0;
}

int config_dump(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    h10_config_t config;
    fram_read(CONFIG_OFFSET, &config, sizeof(config));
    if (config.magic == CONFIG_MAGIC) {
        puts("Configuration parameters:");
        printf(
            "  vcc_low: %d, vcc_high: %d, vpanel_low: %d, vpanel_high: %d, sleep_secs: %d,\n"
            "  bme68x_energy_min: %d, sps30_energy_min: %d, senseair_energy_min: %d,\n",
            config.vcc_low, config.vcc_high, config.vpanel_low, config.vpanel_high, config.sleep_secs,
            config.bme68x_energy_min, config.sps30_energy_min, config.senseair_energy_min
        );
        printf("  sleep_secs_mult: {");
        for(size_t i=0; i<ARRAY_SIZE(config.sleep_secs_mult); i++) {
            printf("%d%s", config.sleep_secs_mult[i], (i<ARRAY_SIZE(config.sleep_secs_mult)-1? ", " : "}\n"));
        }
    } else {
        puts("No configuration parameters are present in FRAM");
    }
    return 0;
}

int config_set(int argc, char **argv)
{
    if (argc < 3) {
        puts("usage: config_set <param> <value>");
        return -1;
    }

    uint8_t sleep_secs_mult[8] = SLEEP_SECS_MULT;
    h10_config_t config;
    fram_read(CONFIG_OFFSET, &config, sizeof(config));
    if (config.magic != CONFIG_MAGIC) {
        // apply defaults
        config.magic = CONFIG_MAGIC;
        config.vcc_low = VCC_LOW;
        config.vcc_high = VCC_HIGH;
        config.vpanel_low = VPANEL_LOW;
        config.vpanel_high = VPANEL_HIGH;
        config.sleep_secs = SLEEP_SECS;
        config.bme68x_energy_min = BME68X_ENERGY_MIN;
        config.sps30_energy_min = SPS30_ENERGY_MIN;
        config.senseair_energy_min = SENSEAIR_ENERGY_MIN;
        memcpy(config.sleep_secs_mult, sleep_secs_mult, ARRAY_SIZE(sleep_secs_mult));
    }

    int16_t value = atoi(argv[2]);
    if (strcmp(argv[1], "vcc_low")==0) {
        if (value >= VCC_LOW) {
            config.vcc_low = value;
        } else {
            printf("Vcc_low %d is less than %d\n", value, VCC_LOW);
            return -1;
        }
    } else if (strcmp(argv[1], "vcc_high")==0) {
        if (value > h10_state.config.vcc_low) {
            config.vcc_high = value;
        } else {
            printf("Vcc_high %d is less than %d\n", value, h10_state.config.vcc_low);
            return -1;
        }
    } else if (strcmp(argv[1], "vpanel_low")==0) {
        if (value >= VPANEL_LOW) {
            config.vpanel_low = value;
        } else {
            printf("Vpanel_low %d is less than %d\n", value, VPANEL_LOW);
            return -1;
        }
    } else if (strcmp(argv[1], "vpanel_high")==0) {
        if (value > h10_state.config.vpanel_low) {
            config.vpanel_high = value;
        } else {
            printf("Vpanel_high %d is less than %d\n", value, h10_state.config.vpanel_low);
            return -1;
        }
    } else if (strcmp(argv[1], "sleep_secs")==0) {
        if (value > 0) {
            config.sleep_secs = (uint16_t)value;
        } else {
            printf("Sleep secs %d should be > 0\n", value);
            return -1;
        }
    } else if (strcmp(argv[1], "bme68x_energy_min")==0) {
        config.bme68x_energy_min = (uint8_t)(0xFF & value);
    } else if (strcmp(argv[1], "sps30_energy_min")==0) {
        config.sps30_energy_min = (uint8_t)(0xFF & value);
    } else if (strcmp(argv[1], "senseair_energy_min")==0) {
        config.senseair_energy_min = (uint8_t)(0xFF & value);
    } else if (strcmp(argv[1], "sleep_secs_mult")==0) {
        int i;
        int n = ARRAY_SIZE(sleep_secs_mult);
        if (argc != n + 2) {
            printf("Sleep secs mult needs %d values, but %d were provided\n", n, argc-2);
            return -1;
        }
        // ensure sleep sec multiplier starts at 1 and is monotonic
        sleep_secs_mult[0] = value;
        uint8_t mult_ok = (sleep_secs_mult[0] == 1 ? 1 : 0);
        for (i=1; mult_ok && i<n; i++) {
            sleep_secs_mult[i] = atoi(argv[i+2]);
            mult_ok = (sleep_secs_mult[i] >= sleep_secs_mult[i-1] ? 1 : 0);
        }
        if (mult_ok) {
            for (i=0; i<n; i++) {
                config.sleep_secs_mult[i] = sleep_secs_mult[i];
            }
        } else {
            printf("Sleep secs mult values should start at 1 and be monotonic\n");
            return -1;
        }
    } else {
        printf("Parameter '%s' unknown.", argv[1]);
        puts("Valid parameters: vcc_low, vcc_high, vpanel_low, vpanel_high, sleep_secs, bme68x_energy_min, sps30_energy_min, senseair_energy_min, sleep_secs_mult.");
    }
    fram_write(CONFIG_OFFSET, (uint8_t *)&config, sizeof(config));

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

    save_loramac();
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

