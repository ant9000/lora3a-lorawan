#include "common.h"

#include "thread.h"
#include "ps.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#ifdef MODULE_BME68X
#include "bme68x.h"
#include "bme68x_params.h"
static bme68x_t bme68x[BME68X_NUMOF];
#ifdef MODULE_BME68X_FP
#define BME68X_TYPE double
#else
#define BME68X_TYPE int32_t
#endif
/* Heater temperature in degree Celsius */
static uint16_t temp_prof[2][10] = {
    { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 },
    { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 },
};
/* Heating duration in milliseconds */
static uint16_t dur_prof[2][10] = {
    { 700, 280, 1400, 4200, 700, 700, 700, 700, 700, 700 },
    { 700, 280, 1400, 420, 700, 700, 700, 700, 700, 700 },
};
#endif

#ifdef MODULE_SPS30
#include "sps30.h"
#include "sps30_params.h"
#define SPS30_WAKEUP_DELAY_MS   (2 * MS_PER_SEC)
#define SPS30_STARTUP_DELAY_MS (10 * MS_PER_SEC)
#define SPS30_POLL_DELAY_MS     (1 * MS_PER_SEC)
sps30_t sps30;
bool sps30_init_done = false;
#endif

typedef struct __attribute__((packed)) {
    int32_t vcc;
    int32_t vpanel;
#ifdef MODULE_BME68X
    BME68X_TYPE temperature[2];
    BME68X_TYPE pressure[2];
    BME68X_TYPE humidity[2];
    BME68X_TYPE gas_resistance[2][10];
#endif
#ifdef MODULE_SPS30
#ifndef MODULE_BME68X_FP
    float mc_pm1;
    float mc_pm2_5;
    float mc_pm4;
#endif
    float mc_pm10;
#ifndef MODULE_BME68X_FP
    float nc_pm0_5;
    float nc_pm1;
    float nc_pm2_5;
    float nc_pm4;
    float nc_pm10;
    float ps;
#endif
#endif
} sensors_data_t;
static sensors_data_t sensor_data;

char sensors_thread_stack[3][THREAD_STACKSIZE_MEDIUM];

int init_sensors(void) {
#ifdef SPS30_POWER_PIN
        gpio_init(SPS30_POWER_PIN, GPIO_OUT);
        gpio_set(SPS30_POWER_PIN);
#endif
#ifdef BME68X_POWER_PIN
        gpio_init(BME68X_POWER_PIN, GPIO_OUT);
        gpio_set(BME68X_POWER_PIN);
#endif

#ifdef MODULE_BME68X
    int res;
    for (size_t i = 0; i < BME68X_NUMOF; i++) {
        DEBUG("Initialize BME68X sensor %u at address 0x%02x... ", i, bme68x_params[i].intf.i2c.addr);
        bme68x_t *dev = &bme68x[i];
        res = bme68x_init(dev, &bme68x_params[i]);
        if (res == BME68X_OK) {
            DEBUG("OK.\n");
            // TODO: restore config from FRAM
            dev->config.op_mode = BME68X_SEQUENTIAL_MODE;
            dev->config.sensors.filter = BME68X_FILTER_SIZE_15;
            dev->config.sensors.odr = BME68X_ODR_NONE;
            dev->config.sensors.os_hum = BME68X_OS_16X;
            dev->config.sensors.os_pres = BME68X_OS_16X;
            dev->config.sensors.os_temp = BME68X_OS_16X;
            dev->config.heater.enable = BME68X_ENABLE;
            dev->config.heater.heatr_temp_prof = temp_prof[i];
            dev->config.heater.heatr_dur_prof = dur_prof[i];
            dev->config.heater.profile_len = ARRAY_SIZE(temp_prof[i]);
            res = bme68x_apply_config(dev);
        }
        if (res != BME68X_OK) {
            DEBUG("failed.\n");
            bme68x[i].sensor.chip_id = 0;
        }
    }
#endif

#ifdef MODULE_SPS30
    DEBUG("Initialize SPS30 sensor ...");
    ztimer_sleep(ZTIMER_MSEC, SPS30_WAKEUP_DELAY_MS);
    if(sps30_init(&sps30, &sps30_params[0]) == SPS30_OK) {
        sps30_init_done = true;
        DEBUG("OK.\n");
    } else {
        DEBUG("failed.\n");
    }
#endif
    return 0;
}

int deinit_sensors(void) {
#ifdef SPS30_POWER_PIN
        gpio_clear(SPS30_POWER_PIN);
#endif
#ifdef BME68X_POWER_PIN
        gpio_clear(BME68X_POWER_PIN);
#endif
    return 0;
}

#ifdef MODULE_BME68X
void *read_bme68x_thread(void *arg) {
    int i = (int)arg;
    int res;
    bme68x_data_t data[3];
    uint32_t delay;
    uint8_t n_fields;
    bme68x_t *dev = &bme68x[i];

    res = bme68x_start_measure(dev);
    if (res == BME68X_OK) {
        int sample_count = 0;
        do {
            delay = bme68x_get_measure_duration(dev) + (dev->config.heater.heatr_dur_prof[sample_count] * 1000);
            bme68x_wait_us(dev, delay);
            res = bme68x_get_measure_data(dev, data, &n_fields);
            if (res == BME68X_W_NO_NEW_DATA) {
                continue;
            } else if (res != BME68X_OK) {
                DEBUG("Cannot get measure data for BME68x sensor %d, sample %d.\n", i, sample_count);
                break;
            }
            for(int n=0; n < n_fields; n++) {
                DEBUG(
#ifdef MODULE_BME68X_FP
                    "BME68X[%d].%d idx=%d, temp=%.2f, press=%.2f, hum=%.2f, gas=%.2f, status=0x%02x\n",
#else
                    "BME68X[%d].%d idx=%d, temp=%d, press=%ld, hum=%ld, gas=%ld, status=0x%02x\n",
#endif
                    i, sample_count, data[n].gas_index, data[n].temperature, data[n].pressure, data[n].humidity,
                    data[n].gas_resistance, data[n].status
                );
                if(sample_count == 0) {
                    sensor_data.temperature[i] = data[n].temperature;
                    sensor_data.pressure[i] = data[n].pressure;
                    sensor_data.humidity[i] = data[n].humidity;
                }
                sensor_data.gas_resistance[i][data[n].gas_index] = data[n].gas_resistance;
                sample_count++;
            }
        } while(sample_count < dev->config.heater.profile_len);
    } else {
        DEBUG("Cannot start measure for BME68x sensor %d.\n", i);
    }
    return NULL;
}
#endif

#ifdef MODULE_SPS30
static void _sps30_print_val_row(char *typ1, char *typ2, char *unit, float val)
{
    printf("| %-5s %4s:%3"PRIu32".%03"PRIu32" %-8s |\n", typ1, typ2,
           (uint32_t)val, ((uint32_t)((val + 0.0005) * 1000)) % 1000, unit);
}
void *read_sps30_thread(void *arg) {
    (void)arg;
    sps30_data_t data;
    int res;

    res = sps30_start_measurement(&sps30);
    if (res == SPS30_OK) {
        ztimer_sleep(ZTIMER_MSEC, SPS30_STARTUP_DELAY_MS);
        while (1) {
            bool ready = sps30_data_ready(&sps30, &res);
            if (!ready) {
                if (res != SPS30_OK) {
                    DEBUG("Data ready error for SPS30 sensor.\n");
                    break;
                }
                /* try again after some time */
puts("@");
                ztimer_sleep(ZTIMER_MSEC, SPS30_POLL_DELAY_MS);
                continue;
            }
            res = sps30_read_measurement(&sps30, &data);
            if (res == SPS30_OK) {
#ifndef MODULE_BME68X_FP
                sensor_data.mc_pm1 = data.mc_pm1;
                sensor_data.mc_pm2_5 = data.mc_pm2_5;
                sensor_data.mc_pm4 = data.mc_pm4;
#endif
                sensor_data.mc_pm10 = data.mc_pm10;
#ifndef MODULE_BME68X_FP
                sensor_data.nc_pm0_5 = data.nc_pm0_5;
                sensor_data.nc_pm1 = data.nc_pm1;
                sensor_data.nc_pm2_5 = data.nc_pm2_5;
                sensor_data.nc_pm4 = data.nc_pm4;
                sensor_data.nc_pm10 = data.nc_pm10;
                sensor_data.ps = data.ps;
#endif
                puts("\nv==== SPS30 measurements ====v");
                _sps30_print_val_row("MC PM",  "1.0", "[µg/m³]", data.mc_pm1);
                _sps30_print_val_row("MC PM",  "2.5", "[µg/m³]", data.mc_pm2_5);
                _sps30_print_val_row("MC PM",  "4.0", "[µg/m³]", data.mc_pm4);
                _sps30_print_val_row("MC PM", "10.0", "[µg/m³]", data.mc_pm10);
                _sps30_print_val_row("NC PM",  "0.5", "[#/cm³]", data.nc_pm0_5);
                _sps30_print_val_row("NC PM",  "1.0", "[#/cm³]", data.nc_pm1);
                _sps30_print_val_row("NC PM",  "2.5", "[#/cm³]", data.nc_pm2_5);
                _sps30_print_val_row("NC PM",  "4.0", "[#/cm³]", data.nc_pm4);
                _sps30_print_val_row("NC PM", "10.0", "[#/cm³]", data.nc_pm10);
                _sps30_print_val_row("TPS",       "",    "[µm]", data.ps);
                puts("+----------------------------+");
                puts("| MC:  Mass Concentration    |");
                puts("| NC:  Number Concentration  |");
                puts("| TPS: Typical Particle Size |");
                puts("^==============================^");
            } else {
                DEBUG("Cannot get measure data for SPS30 sensor.\n");
            }
            sps30_stop_measurement(&sps30);
            break;
        }
    } else {
        DEBUG("Cannot start measure for SPS30 sensor.\n");
    }
    return NULL;
}
#endif

int read_sensors(uint8_t *msg, size_t len) {
    memset(&sensor_data, 0, sizeof(sensor_data));

    int32_t vcc_raw = adc_sample(0, ADC_RES_16BIT);
    int32_t vpanel_raw = adc_sample(1, ADC_RES_16BIT);
    sensor_data.vcc = (vcc_raw * 4 * 1000) >> 16; // rescaled vcc/4 to 1V=65535 counts
    sensor_data.vpanel = (vpanel_raw * (220 + 75) / 75 * 1000) >> 16; // adapted to real resistor partition factor (75k over 220k)
    DEBUG("VCC: %ld, VCC rescaled: %ld\n", vcc_raw, sensor_data.vcc);
    DEBUG("Vpanel: %ld, Vpanel rescaled: %ld\n", vpanel_raw, sensor_data.vpanel);

    kernel_pid_t sensors_pid[3];
    unsigned i, sensors = 0;
#ifdef MODULE_BME68X
    char bme68x_thread_names[2][20] = { "read_bme68x_thread0", "read_bme68x_thread1" };
    for (i = 0; i < BME68X_NUMOF; i++) {
        if (bme68x[i].sensor.chip_id == BME68X_CHIP_ID) {
            sensors_pid[sensors] = thread_create(
                sensors_thread_stack[sensors], sizeof(sensors_thread_stack[sensors]),
                THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                read_bme68x_thread, (void *)i, bme68x_thread_names[i]
            );
            sensors++;
        }
    }
#endif

#ifdef MODULE_SPS30
    if (sps30_init_done) {
        sensors_pid[sensors] = thread_create(
            sensors_thread_stack[sensors], sizeof(sensors_thread_stack[sensors]),
            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
            read_sps30_thread, NULL, "read_sps30_thread"
        );
        sensors++;
    }
#endif

    unsigned pending, counter = 0;
    do {
        pending = 0;
        for (i=0; i<sensors; i++) {
            pending += thread_getstatus(sensors_pid[i]) != STATUS_NOT_FOUND ? 1 : 0;
        }
        if (counter++ % 1000 == 0) {
            ps();
        }
        if (pending) {
            ztimer_sleep(ZTIMER_MSEC, 5);
        }
    } while (pending > 0);
    size_t n = sizeof(sensor_data);
    assert(n < len);
    memcpy(msg, &sensor_data, n);
    return n;
}
