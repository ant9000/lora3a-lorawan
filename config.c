#include "common.h"

#define ENABLE_DEBUG 1
#include "debug.h"

h10_state_t h10_state;

void restore_config(void)
{
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
    }
    DEBUG(
        "CONFIG = {\n"
        "  vcc_low: %d, vcc_high: %d, vpanel_low: %d, vpanel_high: %d, sleep_secs: %d,\n"
        "  bme68x_energy_min: %d, sps30_energy_min: %d, senseair_energy_min: %d\n"
        "}\n",
        h10_state.config.vcc_low, h10_state.config.vcc_high,
        h10_state.config.vpanel_low, h10_state.config.vpanel_high,
        h10_state.config.sleep_secs,
        h10_state.config.bme68x_energy_min,
        h10_state.config.sps30_energy_min,
        h10_state.config.senseair_energy_min
    );
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
    uint16_t mult[8] = {1, 2, 5, 10, 20, 40, 60, 120};
    int idx = (7 - h10_state.energy_state.levels.storage - h10_state.energy_state.levels.charging / 2);
    if (idx < 0) {
        idx = 0;
    } else if (idx > 7) {
        idx = 7;
    }
    h10_state.sleep_secs = h10_state.config.sleep_secs * mult[idx];
    DEBUG("Sleep secs: %d\n", h10_state.sleep_secs);
}
