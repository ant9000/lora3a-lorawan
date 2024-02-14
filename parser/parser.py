#!/usr/bin/env python3

import collections, heatshrink2, struct

def parse_record(data):
    Header  = collections.namedtuple('Header', 'bme68x_num bme68x_fp sps30_num is_compressed')
    Sensors = collections.namedtuple('Sensors', 'vcc vpanel bme68x sps30')
    Bme68x  = collections.namedtuple('Bme68x', 'temp press hum gas')
    Sps30   = collections.namedtuple('Sps30', 'mc_pm1 mc_pm2_5 mc_pm4 mc_pm10 nc_pm0_5 nc_pm1 nc_pm2_5 nc_pm4 nc_pm10 ps')
    Output  = collections.namedtuple('Output', 'header data sensors')
    hdr = data[0]
    data = data[1:]
    bme68x_num = (hdr & 0x03)
    bme68x_fp = (hdr & 0x04 != 0)
    sps30_num = (hdr & 0x08 != 0)
    is_compressed = (hdr & 0x10 != 0)
    header = Header(bme68x_num, bme68x_fp, sps30_num, is_compressed)
    if is_compressed:
        data = heatshrink2.decompress(data, window_sz2=8, lookahead_sz2=4)
    offset = 0
    fmt = "2h"
    vcc, vpanel = struct.unpack_from(fmt, data, offset)
    offset += struct.calcsize(fmt)
    sensors = Sensors(vcc, vpanel, [], [])
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
            n = i * 10
            sensors.bme68x.append(Bme68x(temp[i], press[i], hum[i], gas[n:(n+10)]))
    if sps30_num:
        fmt = "10f"
        sensors.sps30.append(Sps30(*struct.unpack_from(fmt, data, offset)))
    output = Output(header, data, sensors)
    return output

if __name__ == "__main__":
    import fileinput, time, base64

    for n, line in enumerate(fileinput.input()):
        print("#### RECORD %d ####" % n)
        print("[%s]" % time.strftime("%Y-%m-%d %H:%M:%S"))
        line = line.strip()
        print("Line: '%s'" % line)
        if not len(line):
            continue
        try:
            try:
                data = base64.b64decode(line)
            except:
                data = bytes.fromhex(line)
            output = parse_record(data)

            print("BME68X:            ", output.header.bme68x_num)
            print("BME68X_FP:         ", output.header.bme68x_fp)
            print("SPS30:             ", output.header.sps30_num)
            print("Compressed payload:", output.header.is_compressed)
            print("Length:            ",len(data))
            print("Raw data:", end="")
            for i, c in enumerate(output.data):
                if i % 16 == 0:
                    print("\n  ", end="")
                print("%02x " % c, end="")
            print("")
            print("Vcc = %.3f V" % (output.sensors.vcc / 1000))
            print("Vpanel = %.3f V" % (output.sensors.vpanel / 1000))
            for i, bme68x in enumerate(output.sensors.bme68x):
                print("[BME68X_{}] temp = {}, press = {}, hum = {}, gas = {}".format(i, bme68x.temp, bme68x.press, bme68x.hum, bme68x.gas))
            if output.sensors.sps30:
                sps30 = output.sensors.sps30[0]
                print("MC PM  1   =", sps30.mc_pm1)
                print("MC PM  2.5 =", sps30.mc_pm2_5)
                print("MC PM  4   =", sps30.mc_pm4)
                print("MC PM 10   =", sps30.mc_pm10)
                print("NC PM  0.5 =", sps30.nc_pm0_5)
                print("NC PM  1   =", sps30.nc_pm1)
                print("NC PM  2.5 =", sps30.nc_pm2_5)
                print("NC PM  4   =", sps30.nc_pm4)
                print("NC PM 10   =", sps30.nc_pm10)
                print("PS         =", sps30.ps)
            print("")

        except Exception as e:
            print("Not a valid record. Error was:", e)
