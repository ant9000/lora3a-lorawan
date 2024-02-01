#include "common.h"

#include "thread.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#ifdef MODULE_BME68X
#include "bme68x.h"
#include "bme68x_params.h"
static bme68x_t bme68x[BME68X_NUMOF];
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

typedef struct {
    double temperature[2];
    double pressure[2];
    double humidity[2];
    double gas_resistance[2][10];
} sensors_data_t;
static sensors_data_t sensor_data;

char sensors_thread_stack[3][THREAD_STACKSIZE_MAIN];

int init_sensors(void) {
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

    return 0;
}

int deinit_sensors(void) {
#ifdef SENSEAIR_POWER_PIN
        gpio_clear(SENSEAIR_POWER_PIN);
#endif
#ifdef BME68X_POWER_PIN
        gpio_clear(BME68X_POWER_PIN);
#endif
    return 0;
}

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
                    "BME68X[%d].%d idx=%d, temp=%.2f, press=%.2f, hum=%.2f, gas=%.2f, status=0x%02x\n",
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

int read_sensors(uint8_t *msg, size_t len) {
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

    unsigned pending;
    do {
        pending = 0;
        for (i=0; i<sensors; i++) {
            pending += thread_getstatus(sensors_pid[i]) != STATUS_NOT_FOUND ? 1 : 0;
        }
        if (pending) {
            ztimer_sleep(ZTIMER_MSEC, 5);
        }
    } while (pending > 0);
    return snprintf((char *)msg, len, "TODO:BME+SPS");
}
