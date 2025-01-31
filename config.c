#include "common.h"

#define ENABLE_DEBUG 1
#include "debug.h"

h10_state_t h10_state;

void restore_config(void)
{
    size_t i;
    h10_config_t config;
    // apply defaults
    h10_state.config.vcc_low = VCC_LOW;
    h10_state.config.vcc_high = VCC_HIGH;
    h10_state.config.vpanel_low = VPANEL_LOW;
    h10_state.config.vpanel_high = VPANEL_HIGH;
    h10_state.config.sleep_secs = SLEEP_SECS;
    h10_state.config.bme68x_energy_min = BME68X_ENERGY_MIN;
    h10_state.config.sps30_energy_min = SPS30_ENERGY_MIN;
    h10_state.config.senseair_energy_min = SENSEAIR_ENERGY_MIN;
    uint8_t sleep_secs_mult[8] = SLEEP_SECS_MULT;
    memcpy(h10_state.config.sleep_secs_mult, sleep_secs_mult, ARRAY_SIZE(sleep_secs_mult));
    // read values from FRAM
    fram_read(CONFIG_OFFSET, &config, sizeof(config));
    if (config.magic == CONFIG_MAGIC) {
        puts("Found config params in FRAM, restoring");
        if (config.vcc_low > VCC_LOW) {
            h10_state.config.vcc_low = config.vcc_low;
        }
        if (config.vcc_high > h10_state.config.vcc_low) {
            h10_state.config.vcc_high = config.vcc_high;
        }
        if (config.vpanel_low > VPANEL_LOW) {
            h10_state.config.vpanel_low = config.vpanel_low;
        }
        if (config.vpanel_high > h10_state.config.vpanel_low) {
            h10_state.config.vpanel_high = config.vpanel_high;
        }
        if (config.sleep_secs > 0) {
            h10_state.config.sleep_secs = config.sleep_secs;
        }
        h10_state.config.bme68x_energy_min = config.bme68x_energy_min;
        h10_state.config.sps30_energy_min = config.sps30_energy_min;
        h10_state.config.senseair_energy_min = config.senseair_energy_min;
        // ensure sleep sec multiplier starts at 1 and is monotonic
        uint8_t mult_ok = (config.sleep_secs_mult[0] == 1 ? 1 : 0);
        for (i=1; mult_ok && i<ARRAY_SIZE(config.sleep_secs_mult); i++) {
            mult_ok = (config.sleep_secs_mult[i] >= config.sleep_secs_mult[i-1] ? 1 : 0);
        }
        if (mult_ok) {
            for (i=0; i<ARRAY_SIZE(config.sleep_secs_mult); i++) {
                h10_state.config.sleep_secs_mult[i] = config.sleep_secs_mult[i];
            }
        }
    }
#if ENABLE_DEBUG
    DEBUG(
        "CONFIG = {\n"
        "  vcc_low: %d, vcc_high: %d, vpanel_low: %d, vpanel_high: %d, sleep_secs: %d,\n"
        "  bme68x_energy_min: %d, sps30_energy_min: %d, senseair_energy_min: %d,\n"
        "  sleep_secs_mult: {",
        h10_state.config.vcc_low, h10_state.config.vcc_high,
        h10_state.config.vpanel_low, h10_state.config.vpanel_high,
        h10_state.config.sleep_secs,
        h10_state.config.bme68x_energy_min,
        h10_state.config.sps30_energy_min,
        h10_state.config.senseair_energy_min
    );
    for (i=0; i<ARRAY_SIZE(h10_state.config.sleep_secs_mult); i++) {
        DEBUG("%d%s",
            h10_state.config.sleep_secs_mult[i],
            (i < ARRAY_SIZE(h10_state.config.sleep_secs_mult) - 1 ? ", " : "}\n")
        );
    }
    DEBUG("}\n");
#endif
}

void compute_state(void)
{
    // FRAM initialization ensures ACME0_POWER_PIN is on (needed before reading vpanel)
    fram_init();
    restore_config();

    int32_t vcc_raw = adc_sample(0, ADC_RES_16BIT);
    int32_t vpanel_raw = adc_sample(1, ADC_RES_16BIT);
    h10_state.vcc = (vcc_raw * 4 * 1000) >> 16; // rescaled vcc/4 to 1V=65535 counts
    h10_state.vpanel = (vpanel_raw * (220 + 75) / 75 * 1000) >> 16; // adapted to real resistor partition factor (75k over 220k)
    DEBUG("VCC: %ld, VCC rescaled: %d\n", vcc_raw, h10_state.vcc);
    DEBUG("Vpanel: %ld, Vpanel rescaled: %d\n", vpanel_raw, h10_state.vpanel);

    // compute storage energy level
    if (h10_state.vcc < h10_state.config.vcc_low) {
        h10_state.energy_state.levels.storage = 0;
    } else if (h10_state.vcc > h10_state.config.vcc_high) {
        h10_state.energy_state.levels.storage = 7;
    } else {
        h10_state.energy_state.levels.storage = 7 * (h10_state.vcc - h10_state.config.vcc_low) / (h10_state.config.vcc_high - h10_state.config.vcc_low);
    }

    // compute charging energy level
    if (h10_state.vpanel < h10_state.config.vpanel_low) {
        h10_state.energy_state.levels.charging = 0;
    } else if (h10_state.vpanel > h10_state.config.vpanel_high) {
        h10_state.energy_state.levels.charging = 7;
    } else {
        // TODO: charging is related non-linearly with vpanel
        h10_state.energy_state.levels.charging = 7 * (h10_state.vpanel - h10_state.config.vpanel_low) / (h10_state.config.vpanel_high - h10_state.config.vpanel_low);
    }
    DEBUG("Energy state: 0x%02x\n", h10_state.energy_state.value);

    // adjust sleep_secs according to energy value
    int idx = (7 - h10_state.energy_state.levels.storage - h10_state.energy_state.levels.charging / 2);
    if (idx < 0) {
        idx = 0;
    } else if (idx > 7) {
        idx = 7;
    }
    h10_state.sleep_secs = h10_state.config.sleep_secs * h10_state.config.sleep_secs_mult[idx];
    DEBUG("Sleep secs: %d\n", h10_state.sleep_secs);
}
