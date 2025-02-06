#include "common.h"

#include "thread.h"
#include "ps.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#ifdef MODULE_HEATSHRINK_COMPRESSION
#include "heatshrink.h"
#endif

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
static uint16_t temp_prof[BME68X_NUMOF][BME68X_PROF_LEN] = { BME68X_TEMP_PROF };
/* Heating duration in milliseconds */
static uint16_t dur_prof[BME68X_NUMOF][BME68X_PROF_LEN] = { BME68X_DUR_PROF };
#else
#define BME68X_NUMOF 0
#endif

#ifdef MODULE_BOSCH_BSEC
#include "periph/rtc.h"
#include "rtc_utils.h"
#include "ztimer64.h"
#include "bosch_bsec.h"
static int64_t time_start, time_stamp;
static bsec_bme_settings_t bsec_sensor_settings[BME68X_NUMOF];
static bool bsec_init_done = false;
bsec_library_return_t setup_for_iaq(void *instance);
bsec_library_return_t process_data(bme68x_t *dev, int64_t tstamp_ns, bme68x_data_t data, int32_t process_data, uint8_t i);
#endif

#ifdef MODULE_SPS30
#include "sps30.h"
#include "sps30_params.h"
#define SPS30_WAKEUP_DELAY_MS   (2 * MS_PER_SEC)
#define SPS30_STARTUP_DELAY_MS (10 * MS_PER_SEC)
#define SPS30_POLL_DELAY_MS     (1 * MS_PER_SEC)
static sps30_t sps30;
static bool sps30_init_done = false;
#define SPS30_NUMOF 1
#ifndef SPS30_MEASURES_COUNT
#define SPS30_MEASURES_COUNT 1
#endif
#ifndef SPS30_MEASURES_SKIP
#define SPS30_MEASURES_SKIP 0
#endif
#else
#define SPS30_NUMOF 0
#endif

#ifdef MODULE_SENSEAIR
#include "senseair.h"
#include "senseair_params.h"
static senseair_t senseair;
static senseair_abc_data_t senseair_calib_data;
static bool senseair_init_done = false;
#define SENSEAIR_NUMOF 1
#else
#define SENSEAIR_NUMOF 0
#endif

typedef struct __attribute__((packed)) {
   int16_t vcc;
   int16_t vpanel;
#ifdef MODULE_BME68X
#ifdef MODULE_BME68X_FP
   double temperature[BME68X_NUMOF];
#else
   int16_t temperature[BME68X_NUMOF];
#endif
   BME68X_TYPE pressure[BME68X_NUMOF];
   BME68X_TYPE humidity[BME68X_NUMOF];
   BME68X_TYPE gas_resistance[BME68X_NUMOF][BME68X_PROF_LEN];
#endif
#ifdef MODULE_SPS30
   float mc_pm1;
   float mc_pm2_5;
   float mc_pm4;
   float mc_pm10;
   float nc_pm0_5;
   float nc_pm1;
   float nc_pm2_5;
   float nc_pm4;
   float nc_pm10;
   float ps;
#endif
#ifdef MODULE_SENSEAIR
   uint16_t conc_ppm;
   int16_t temp_cC;
#endif
#ifdef MODULE_BOSCH_BSEC
   uint16_t iaq[BME68X_NUMOF];
   uint8_t iaq_accuracy[BME68X_NUMOF];
#endif
} sensors_data_t;
static sensors_data_t sensor_data;

#if MODULE_BME68X + MODULE_SPS30 + MODULE_SENSEAIR
char sensors_thread_stack[BME68X_NUMOF + SPS30_NUMOF + SENSEAIR_NUMOF][THREAD_STACKSIZE_MEDIUM + THREAD_EXTRA_STACKSIZE_PRINTF];
#endif

int init_sensors(void) {
#ifdef SPS30_POWER_PIN
       // bail out if energy is not enough
       if (h10_state.energy_state.levels.storage >= h10_state.config.sps30_energy_min) {
           gpio_init(SPS30_POWER_PIN, GPIO_OUT);
           gpio_set(SPS30_POWER_PIN);
       }
#endif
#ifdef BME68X_POWER_PIN
       if (h10_state.energy_state.levels.storage >= h10_state.config.bme68x_energy_min) {
           gpio_init(BME68X_POWER_PIN, GPIO_OUT);
           gpio_set(BME68X_POWER_PIN);
       }
#endif
#ifdef SENSEAIR_POWER_PIN
       if (h10_state.energy_state.levels.storage >= h10_state.config.senseair_energy_min) {
           gpio_init(SENSEAIR_POWER_PIN, GPIO_OUT);
           gpio_set(SENSEAIR_POWER_PIN);
       }
#endif

#ifdef MODULE_BME68X
   int res;
   if (h10_state.energy_state.levels.storage >= h10_state.config.bme68x_energy_min) {
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
   }
#endif

#ifdef MODULE_BOSCH_BSEC
    struct tm time;
    rtc_get_time(&time);
    printf("RTC time: %04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year + 1900, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    time_start = (int64_t)rtc_mktime(&time) * 1000000000;

    bsec_library_return_t bsec_res;
    memset(bsec_sensor_settings, 0, sizeof(bsec_sensor_settings));
    fram_read(BSEC_OFFSET, bsec_state, sizeof(bsec_state));
    if ((bsec_res = bsec_init()) != BSEC_OK) {
        printf("[ERROR] BSEC init failed: %s.\n", bsec_errno(bsec_res));
        goto bsec_out;
    }
    for (size_t i = 0; i < BME68X_NUMOF; i++) {
        if ((bsec_res = setup_for_iaq(bsec_inst[i])) != BSEC_OK) {
            printf("[ERROR] setup_for_iaq failed for inst[%d]: %s.\n", i, bsec_errno(bsec_res));
            goto bsec_out;
        }
    }
    DEBUG("Initialized BSEC library\n");
    bsec_init_done = true;
bsec_out:
    (void)bsec_init_done;
#endif

#ifdef MODULE_SPS30
   if (h10_state.energy_state.levels.storage >= h10_state.config.sps30_energy_min) {
       DEBUG("Initialize SPS30 sensor ...");
       ztimer_sleep(ZTIMER_MSEC, SPS30_WAKEUP_DELAY_MS);
       if(sps30_init(&sps30, &sps30_params[0]) == SPS30_OK) {
           sps30_init_done = true;
           DEBUG("OK.\n");
       } else {
           DEBUG("failed.\n");
       }
   }
#endif

#ifdef MODULE_SENSEAIR
   if (h10_state.energy_state.levels.storage >= h10_state.config.senseair_energy_min) {
       DEBUG("Initialize SENSEAIR sensor ...");
       if (senseair_init(&senseair, &senseair_params[0]) == SENSEAIR_OK) {
           senseair_init_done = true;
           DEBUG("OK.\n");
       } else {
           DEBUG("Senseair init failed.\n");
       }
       memset(&senseair_calib_data, 0, sizeof(senseair_calib_data));
       if (fram_read(SENSEAIR_OFFSET, &senseair_calib_data, sizeof(senseair_calib_data))) {
           puts("FRAM read failed.");
       } else {
           if (senseair_write_abc_data(&senseair, &senseair_calib_data) == SENSEAIR_OK) {
               puts("ABC data restored to sensor.");
           }
           else {
               puts("ABC data not available.");
           }
       }
   }
#endif
   return 0;
}

int deinit_sensors(void) {
#ifdef SPS30_POWER_PIN
    gpio_clear(SPS30_POWER_PIN);
#endif
#ifdef MODULE_BOSCH_BSEC
    if (bsec_init_done) {
        bsec_library_return_t res;
        if ((res = bsec_deinit()) == BSEC_OK) {
            fram_write(BSEC_OFFSET, (uint8_t *)bsec_state, sizeof(bsec_state));
        } else {
            printf("[ERROR] BSEC deinit failed: %s.\n", bsec_errno(res));
        }
   }
#endif
#ifdef BME68X_POWER_PIN
   gpio_clear(BME68X_POWER_PIN);
#endif
#if MODULE_SENSEAIR
   if (h10_state.energy_state.levels.storage >= h10_state.config.senseair_energy_min) {
       if (senseair_read_abc_data(&senseair, &senseair_calib_data) == SENSEAIR_OK) {
           puts("Saving SENSEAIR calibration data to FRAM.");
           if (fram_write(SENSEAIR_OFFSET, (uint8_t *)&senseair_calib_data, sizeof(senseair_calib_data))) {
               puts("FRAM write failed.");
           }
       }
   }
#endif
#ifdef SENSEAIR_POWER_PIN
   gpio_clear(SENSEAIR_POWER_PIN);
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

   // bail out if energy is not enough
   if (h10_state.energy_state.levels.storage < h10_state.config.bme68x_energy_min) return NULL;

   res = bme68x_start_measure(dev);
   if (res == BME68X_OK) {
       int sample_count = 0;
       do {
           delay = bme68x_get_measure_duration(dev) + (dev->config.heater.heatr_dur_prof[sample_count] * 1000);
           int retry = 3;
           do {
              bme68x_wait_us(dev, delay);
              res = bme68x_get_measure_data(dev, data, &n_fields);
           } while ((res==BME68X_W_NO_NEW_DATA) && ((retry--)>0));
           if (res != BME68X_OK) {
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

#ifdef MODULE_BOSCH_BSEC
   ztimer_sleep(ZTIMER_MSEC, 200);
   int measures = 0;
   while (measures == 0) {
       time_stamp = time_start + ztimer64_now(ZTIMER64_USEC) * 1000;
       if (time_stamp >= bsec_sensor_settings[i].next_call) {
            // ask BSEC for sensor settings
            res = bsec_sensor_control_m(bsec_inst[i], time_stamp, &bsec_sensor_settings[i]);
            if (res != BSEC_OK) {
                if (res < BSEC_OK) {
                    printf("[ERROR] bsec_sensor_control_m for bsec_inst[%d]: %s.\n", i, bsec_errno(res));
                    return NULL;
                } else {
                    printf("[WARNING] bsec_sensor_control for bsec_inst[%d]: %s.\n", i, bsec_errno(res));
                }
            }
            // configure sensor as dictated
            if ((bsec_sensor_settings[i].op_mode == BME68X_FORCED_MODE) || (dev->config.op_mode != bsec_sensor_settings[i].op_mode)) {
                    bsec_apply_configuration(&bsec_sensor_settings[i], dev, i);
                    printf("BSEC[%d] applied configuration to sensor\n", i);
            }
        }
        // - read sensor data
        if (bsec_sensor_settings[i].trigger_measurement && bsec_sensor_settings[i].op_mode != BME68X_SLEEP_MODE) {
            bme68x_get_measure_data(dev, data, &n_fields);
            if (n_fields) {
                for (size_t j = 0; j < n_fields; j++) {
                    if (data[j].status & BME68X_GASM_VALID_MSK) {
                        if ((res = process_data(dev, time_stamp, data[j], bsec_sensor_settings[i].process_data, i)) == BSEC_OK) {
                            printf("BSEC[%d]: IAQ=%d (%d)\n", i, sensor_data.iaq[i], sensor_data.iaq_accuracy[i]);
                            measures++;
                        } else {
                            printf("[ERROR] process_data failed for bsec_inst[%d]: %s.\n", i, bsec_errno(res));
                            return NULL;
                        }
                    }
                    if (bsec_sensor_settings[i].op_mode == BME68X_FORCED_MODE) break; // exit for
                }
            }
        }
    }
#endif
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
    int n = 0;

    // bail out if energy is not enough
    if (h10_state.energy_state.levels.storage < h10_state.config.sps30_energy_min) return NULL;

    res = sps30_start_measurement(&sps30);
    if (res == SPS30_OK) {
        ztimer_sleep(ZTIMER_MSEC, SPS30_STARTUP_DELAY_MS);
        while (n < SPS30_MEASURES_COUNT + SPS30_MEASURES_SKIP) {
            bool ready = sps30_data_ready(&sps30, &res);
            if (!ready) {
                if (res != SPS30_OK) {
                    DEBUG("Data ready error for SPS30 sensor.\n");
                    break;
                }
                /* try again after some time */
                ztimer_sleep(ZTIMER_MSEC, SPS30_POLL_DELAY_MS);
                continue;
            }
            res = sps30_read_measurement(&sps30, &data);
            if (res == SPS30_OK) {
                if (n >= SPS30_MEASURES_SKIP) {
#ifndef MODULE_BME68X_FP
                    sensor_data.mc_pm1 += data.mc_pm1;
                    sensor_data.mc_pm2_5 += data.mc_pm2_5;
                    sensor_data.mc_pm4 += data.mc_pm4;
#endif
                    sensor_data.mc_pm10 += data.mc_pm10;
#ifndef MODULE_BME68X_FP
                    sensor_data.nc_pm0_5 += data.nc_pm0_5;
                    sensor_data.nc_pm1 += data.nc_pm1;
                    sensor_data.nc_pm2_5 += data.nc_pm2_5;
                    sensor_data.nc_pm4 += data.nc_pm4;
                    sensor_data.nc_pm10 += data.nc_pm10;
                    sensor_data.ps += data.ps;
#endif
                }
                printf("\nv==== SPS30 measure %d ====v\n", n);
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
                /* increment measure count */
                n++;
            } else {
                DEBUG("Cannot get measure data for SPS30 sensor.\n");
            }
        }
        sps30_stop_measurement(&sps30);
#ifndef MODULE_BME68X_FP
        sensor_data.mc_pm1 /= SPS30_MEASURES_COUNT;
        sensor_data.mc_pm2_5 /= SPS30_MEASURES_COUNT;
        sensor_data.mc_pm4 /= SPS30_MEASURES_COUNT;
#endif
        sensor_data.mc_pm10 /= SPS30_MEASURES_COUNT;
#ifndef MODULE_BME68X_FP
        sensor_data.nc_pm0_5 /= SPS30_MEASURES_COUNT;
        sensor_data.nc_pm1 /= SPS30_MEASURES_COUNT;
        sensor_data.nc_pm2_5 /= SPS30_MEASURES_COUNT;
        sensor_data.nc_pm4 /= SPS30_MEASURES_COUNT;
        sensor_data.nc_pm10 /= SPS30_MEASURES_COUNT;
        sensor_data.ps /= SPS30_MEASURES_COUNT;
#endif
    } else {
        DEBUG("Cannot start measure for SPS30 sensor.\n");
    }
    return NULL;
}
#endif

#ifdef MODULE_SENSEAIR
void *read_senseair_thread(void *arg) {
    (void)arg;
    uint16_t conc_ppm;
    int16_t temp_cC;

    // bail out if energy is not enough
    if (h10_state.energy_state.levels.storage < h10_state.config.senseair_energy_min) return NULL;

    if (senseair_read(&senseair, &conc_ppm, &temp_cC) == SENSEAIR_OK) {
        sensor_data.conc_ppm = conc_ppm;
        sensor_data.temp_cC = temp_cC;
        printf("[SENSEAIR] Concentration: %d ppm\n", conc_ppm);
        printf("[SENSEAIR] Temperature: %4.2f °C\n", (temp_cC / 100.));
    } else {
        DEBUG("Cannot read SENSAIR.\n");
    }
    return NULL;
}
#endif

#ifdef MODULE_BOSCH_BSEC
bsec_library_return_t setup_for_iaq(void *instance) {
    bsec_sensor_configuration_t virtual_sensors[1];
    bsec_sensor_configuration_t sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    float sample_rate = BSEC_SAMPLE_RATE;

    virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    virtual_sensors[0].sample_rate = sample_rate;

    return bsec_update_subscription_m(instance, virtual_sensors, ARRAY_SIZE(virtual_sensors), sensor_settings, &n_sensor_settings);
}

bsec_library_return_t process_data(bme68x_t *dev, int64_t tstamp_ns, bme68x_data_t data, int32_t process_data, uint8_t i)
{
	bsec_library_return_t bsec_status;
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
    uint8_t index = 0;

    bsec_status = bsec_process_data(tstamp_ns, data, process_data, &dev[i], i, bsec_outputs, &num_bsec_outputs);
    if (bsec_status == BSEC_OK) {
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
                case BSEC_OUTPUT_IAQ:
                    sensor_data.iaq[i] = (uint16_t)bsec_outputs[index].signal;
                    sensor_data.iaq_accuracy[i] = bsec_outputs[index].accuracy;
                    break;
                default:
                    continue;
            }
        }
    }
    return bsec_status;
}
#endif

int read_sensors(uint8_t *msg, size_t len) {
    memset(&sensor_data, 0, sizeof(sensor_data));
    sensor_data.vcc = h10_state.vcc;
    sensor_data.vpanel = h10_state.vpanel;

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

#ifdef MODULE_SENSEAIR
    if (senseair_init_done) {
        sensors_pid[sensors] = thread_create(
            sensors_thread_stack[sensors], sizeof(sensors_thread_stack[sensors]),
            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
            read_senseair_thread, NULL, "read_senseair_thread"
        );
        sensors++;
    }
#endif

    if (sensors) {
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
    }

    msg[0] = 0;
#ifdef MODULE_BME68X
    msg[0] |= (BME68X_NUMOF & 0x03) << 0; // number of bme68x sensors (2 bits)
#ifdef MODULE_BME68X_FP
    msg[0] |= 1 << 2; // floating point?
#endif
#endif
#ifdef MODULE_SPS30
    msg[0] |= 1 << 3;         // sps30
#endif
#ifdef MODULE_SENSEAIR
    msg[0] |= 1 << 4;         // senseair
#endif
#ifdef MODULE_BOSCH_BSEC
    msg[0] |= 1 << 5;         // bsec
#endif
    size_t N = sizeof(sensor_data);
#ifdef MODULE_HEATSHRINK_COMPRESSION
    uint8_t compressed[256];
    size_t n = heatshrink_compress((uint8_t *)&sensor_data, N, compressed, sizeof(compressed));
printf("Sizes: max = %d, sensor data = %d, compressed data = %d\n", len, N, n);
    if (n > 0 && n < len && n < N) {
        msg[0] |= 1 << 7; // compressed
        memcpy(msg + 1, compressed, n);
    } else {
        n = (N < len - 1) ? N : len - 1; // truncating is better than failure
        memcpy(msg + 1, &sensor_data, n);
    }
#else
    size_t n = (N < len - 1) ? N : len - 1; // truncating is better than failure
    memcpy(msg + 1, &sensor_data, n);
#endif
    return n + 1;
}
