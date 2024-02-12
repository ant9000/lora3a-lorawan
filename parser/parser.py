#!/usr/bin/env python3

import fileinput, time, base64, heatshrink2, struct

for n, line in enumerate(fileinput.input()):
    print("#### RECORD %d ####" % n)
    print("[%s]" % time.strftime("%Y-%m-%d %H:%M:%S"))
    try:
        data = base64.b64decode(line.strip())
        hdr = data[0]
        data = data[1:]
        bme68x_num = (hdr & 0x03)
        bme68x_fp = (hdr & 0x04 != 0)
        sps30_num = (hdr & 0x08 != 0)
        is_compressed = (hdr & 0x10 != 0)
        print("BME68X:            ", bme68x_num)
        print("BME68X_FP:         ", bme68x_fp)
        print("SPS30:             ", sps30_num)
        print("Compressed payload:", is_compressed)
        if is_compressed:
            data = heatshrink2.decompress(data, window_sz2=8, lookahead_sz2=4)
        print("Length:            ",len(data))
        print("Raw data:", end="")
        for i, c in enumerate(data):
            if i % 16 == 0:
                print("\n  ", end="")
            print("%02x " % c, end="")
        print("")
        offset = 0
        fmt = "2i"
        vcc, vpanel = struct.unpack_from(fmt, data, offset)
        offset += struct.calcsize(fmt)
        print("Vcc = %.3f V" % (vcc / 1000))
        print("Vpanel = %.3f V" % (vpanel / 1000))
        if bme68x_num:
            fmt = "h" * bme68x_num
            if bme68x_fp:
                fmt = "d" * bme68x_num
            temp = struct.unpack_from(fmt, data, offset)
            offset += struct.calcsize(fmt)
            fmt = "i" * bme68x_num
            if bme68x_fp:
                fmt = "d" * bme68x_num
            press = struct.unpack_from(fmt, data, offset)
            offset += struct.calcsize(fmt)
            hum = struct.unpack_from(fmt, data, offset)
            offset += struct.calcsize(fmt)
            fmt = "10i" * bme68x_num
            if bme68x_fp:
                fmt = "10d" * bme68x_num
            gas = struct.unpack_from(fmt, data, offset)
            offset += struct.calcsize(fmt)
            for i in range(bme68x_num):
                print("[BME68X_{}] temp = {}, press = {}, hum = {}, gas = {}".format(i, temp[i], press[i], hum[i], gas[i:(i+10)]))
        if sps30_num:
            fmt = "10f"
            mc_pm1, mc_pm2_5, mc_pm4, mc_pm10, nc_pm0_5, nc_pm1, nc_pm2_5, nc_pm4, nc_pm10, ps = struct.unpack_from(fmt, data, offset)
            print("MC PM  1   =", mc_pm1)
            print("MC PM  2.5 =", mc_pm2_5)
            print("MC PM  4   =", mc_pm4)
            print("MC PM 10   =", mc_pm10)
            print("NC PM  0.5 =", nc_pm0_5)
            print("NC PM  1   =", nc_pm1)
            print("NC PM  2.5 =", nc_pm2_5)
            print("NC PM  4   =", nc_pm4)
            print("NC PM 10   =", nc_pm10)
            print("PS         =", ps)
        print("")
    except Exception as e:
        print("Not a valid record. Error was:", e)
