function heatshrink_decode(inp) {
    const HEATSHRINK_WINDOW_BITS    = 8;
    const HEATSHRINK_LOOKAHEAD_BITS = 4;

    const HSDR_POLL_EMPTY         =  0; /* input exhausted */
    const HSDR_POLL_MORE          =  1; /* more data remaining, call again w/ fresh output buffer */
    const HSDR_POLL_ERROR_UNKNOWN = -1;

    const HSDR_FINISH_DONE       =  0; /* output is done */
    const HSDR_FINISH_MORE       =  1; /* more output remains */
    const HSDR_FINISH_ERROR_NULL = -1; /* NULL arguments */

    const HSDS_TAG_BIT           = 0; /* tag bit */
    const HSDS_YIELD_LITERAL     = 1; /* ready to yield literal byte */
    const HSDS_BACKREF_INDEX_MSB = 2; /* most significant byte of index */
    const HSDS_BACKREF_INDEX_LSB = 3; /* least significant byte of index */
    const HSDS_BACKREF_COUNT_MSB = 4; /* most significant byte of count */
    const HSDS_BACKREF_COUNT_LSB = 5; /* least significant byte of count */
    const HSDS_YIELD_BACKREF     = 6; /* ready to yield back-reference */

    const NO_BITS = 0xFFFF;

    function assert(val, msg) {
        if (!msg) { msg = "ASSERTION ERROR"; }
        if (!val) { throw new Error(msg); }
    }

    function heatshrink_decoder_poll(hsd, oi) {
        oi.output_size = 0;
        while (1) {
            let in_state = hsd.state;
            switch (in_state) {
                case HSDS_TAG_BIT:
                    hsd.state = st_tag_bit(hsd);
                    break;
                case HSDS_YIELD_LITERAL:
                    hsd.state = st_yield_literal(hsd, oi);
                    break;
                case HSDS_BACKREF_INDEX_MSB:
                    hsd.state = st_backref_index_msb(hsd);
                    break;
                case HSDS_BACKREF_INDEX_LSB:
                    hsd.state = st_backref_index_lsb(hsd);
                    break;
                case HSDS_BACKREF_COUNT_MSB:
                    hsd.state = st_backref_count_msb(hsd);
                    break;
                case HSDS_BACKREF_COUNT_LSB:
                    hsd.state = st_backref_count_lsb(hsd);
                    break;
                case HSDS_YIELD_BACKREF:
                    hsd.state = st_yield_backref(hsd, oi);
                    break;
                default:
                    return HSDR_POLL_ERROR_UNKNOWN;
            }

            if (hsd.state === in_state) {
                if (oi.output_size === oi.buf_size) { return HSDR_POLL_MORE; }
                return HSDR_POLL_EMPTY;
            }
        }
    }

    function st_tag_bit(hsd) {
        let bits = get_bits(hsd, 1);
        if (bits === NO_BITS) {
            return HSDS_TAG_BIT;
        } else if (bits) {
            return HSDS_YIELD_LITERAL;
        } else if (HEATSHRINK_WINDOW_BITS > 8) {
            return HSDS_BACKREF_INDEX_MSB;
        } else {
            hsd.output_index = 0;
            return HSDS_BACKREF_INDEX_LSB;
        }
    }

    function st_yield_literal(hsd, oi) {
        if ((oi.output_size) < oi.buf_size) {
            let byte = get_bits(hsd, 8);
            if (byte === NO_BITS) { return HSDS_YIELD_LITERAL; }
            let mask = (1 << HEATSHRINK_WINDOW_BITS) - 1;
            let c = byte & 0xFF;
            hsd.buffer[(hsd.head_index++) & mask] = c;
            push_byte(hsd, oi, c);
            return HSDS_TAG_BIT;
        } else {
            return HSDS_YIELD_LITERAL;
        }
    }

    function st_backref_index_msb(hsd) {
        let bit_ct = HEATSHRINK_WINDOW_BITS;
        assert(bit_ct > 8, "st_backref_index_msb: bit_ct <= 8");
        let bits = get_bits(hsd, bit_ct - 8);
        if (bits === NO_BITS) { return HSDS_BACKREF_INDEX_MSB; }
        hsd.output_index = bits << 8;
        return HSDS_BACKREF_INDEX_LSB;
    }

    function st_backref_index_lsb(hsd) {
        let bit_ct = HEATSHRINK_WINDOW_BITS;
        let bits = get_bits(hsd, bit_ct < 8 ? bit_ct : 8);
        if (bits === NO_BITS) { return HSDS_BACKREF_INDEX_LSB; }
        hsd.output_index |= bits;
        hsd.output_index++;
        let br_bit_ct = HEATSHRINK_LOOKAHEAD_BITS;
        hsd.output_count = 0;
        return br_bit_ct > 8 ? HSDS_BACKREF_COUNT_MSB : HSDS_BACKREF_COUNT_LSB;
    }

    function st_backref_count_msb(hsd) {
        let br_bit_ct = HEATSHRINK_LOOKAHEAD_BITS;
        assert(br_bit_ct > 8, "st_backref_count_msb: br_bit_ct <= 8");
        let bits = get_bits(hsd, br_bit_ct - 8);
        if (bits === NO_BITS) { return HSDS_BACKREF_COUNT_MSB; }
        hsd.output_count = bits << 8;
        return HSDS_BACKREF_COUNT_LSB;
    }

    function st_backref_count_lsb(hsd) {
        let br_bit_ct = HEATSHRINK_LOOKAHEAD_BITS;
        let bits = get_bits(hsd, br_bit_ct < 8 ? br_bit_ct : 8);
        if (bits === NO_BITS) { return HSDS_BACKREF_COUNT_LSB; }
        hsd.output_count |= bits;
        hsd.output_count++;
        return HSDS_YIELD_BACKREF;
    }

    function st_yield_backref(hsd, oi) {
        let count = oi.buf_size - oi.output_size;
        if (count > 0) {
            let i = 0;
            if (hsd.output_count < count) { count = hsd.output_count; }
            let mask = (1 << HEATSHRINK_WINDOW_BITS) - 1;
            let neg_offset = hsd.output_index;
            assert(neg_offset <= (mask + 1), "st_yield_backref: neg_offset > (mask + 1)");
            assert(count <= ((1 << HEATSHRINK_LOOKAHEAD_BITS)), "st_yield_backref: count > ((1 << HEATSHRINK_LOOKAHEAD_BITS))");
            for (i = 0; i < count; i++) {
                let c = hsd.buffer[(hsd.head_index - neg_offset) & mask];
                push_byte(hsd, oi, c);
                hsd.buffer[hsd.head_index & mask] = c;
                hsd.head_index++;
            }
            hsd.output_count -= count;
            if (hsd.output_count === 0) { return HSDS_TAG_BIT; }
        }
        return HSDS_YIELD_BACKREF;
    }

    function get_bits(hsd, count) {
        let accumulator = 0;
        let i = 0;
        if (count > 15) { return NO_BITS; }
        if (hsd.input_size === 0) {
            if (hsd.bit_index < (1 << (count - 1))) { return NO_BITS; }
        }
        for (i = 0; i < count; i++) {
            if (hsd.bit_index === 0x00) {
                if (hsd.input_size === 0) { return NO_BITS; }
                hsd.current_byte = hsd.input_buffer[hsd.input_index++];
                if (hsd.input_index === hsd.input_size) {
                    hsd.input_index = 0;
                    hsd.input_size = 0;
                }
                hsd.bit_index = 0x80;
            }

            accumulator <<= 1;
            if (hsd.current_byte & hsd.bit_index) { accumulator |= 0x01; }
            hsd.bit_index >>= 1;
        }
        return accumulator;
    }

    function heatshrink_decoder_finish(hsd) {
        switch (hsd.state) {
            case HSDS_TAG_BIT:
            case HSDS_BACKREF_INDEX_LSB:
            case HSDS_BACKREF_INDEX_MSB:
            case HSDS_BACKREF_COUNT_LSB:
            case HSDS_BACKREF_COUNT_MSB:
            case HSDS_YIELD_LITERAL:
                return hsd.input_size === 0 ? HSDR_FINISH_DONE : HSDR_FINISH_MORE;
            default:
                return HSDR_FINISH_MORE;
        }
    }

    function push_byte(hsd, oi, byte) {
        oi.buf[oi.output_size++] = byte;
    }

    //////////////////////////////////////////////
    let hsd = {};
    hsd.input_buffer = new Uint8Array(inp);
    hsd.buffer = new Uint8Array(1 << HEATSHRINK_WINDOW_BITS);
    hsd.state = HSDS_TAG_BIT;
    hsd.input_size = hsd.input_buffer.byteLength;
    hsd.input_index = 0;
    hsd.bit_index = 0x00;
    hsd.current_byte = 0x00;
    hsd.output_count = 0;
    hsd.output_index = 0;
    hsd.head_index = 0;
    let oi = {};
    oi.buf = new Uint8Array(hsd.input_size * HEATSHRINK_WINDOW_BITS);
    oi.buf_size = oi.buf.byteLength;
    let res = heatshrink_decoder_poll(hsd, oi);
    assert(res == HSDR_POLL_EMPTY, "heatshrink_decode: res != HSDR_POLL_EMPTY");
    res = heatshrink_decoder_finish(hsd);
    assert(res == HSDR_FINISH_DONE, "heatshrink_decode: res != HSDR_FINISH_DONE");
    return oi.buf.slice(0, oi.output_size);
}

function decodeUplink(input) {
    var bytes = input.bytes;
    var fPort = input.fPort;
    var variables = input.variables;  // Not used in this example but could be used for configuration.

    var decoded = {};
    var header = {
        bme68x_num: 0,
        bme68x_fp: false,
        sps30_num: false,
        senseair_num: false,
        bsec_num: false,
        is_compressed: false
    };
    var sensors = {
        vcc: 0,
        vpanel: 0,
        bme68x: [],
        sps30: [],
        senseair: [],
        bsec: []
    };

    // Header decoding
    header.bme68x_num = bytes[0] & 0x03;
    header.bme68x_fp = (bytes[0] & 0x04) !== 0;
    header.sps30_num = (bytes[0] & 0x08) !== 0;
    header.senseair_num = (bytes[0] & 0x10) !== 0;
    header.bsec_num = (bytes[0] & 0x20) !== 0;
    header.is_compressed = (bytes[0] & 0x80) !== 0;

    var data = new Uint8Array(bytes).slice(1); // Start after the header
    if (header.is_compressed) {
        data = heatshrink_decode(data);
    }

    var view = new DataView(data.buffer);
    var offset = 0;

    // Reading sensor power supply voltages
    sensors.vcc = view.getInt16(offset, true); // Assuming little endian
    offset += 2;
    sensors.vpanel = view.getInt16(offset, true);
    offset += 2;

    // Parse BME68X sensor data if present
    if (header.bme68x_num) {
        var bme68x_data = [];
        for (var i = 0; i < header.bme68x_num; i++) {
            bme68x_data.push({});
        }

        // Temperature
        for (var i = 0; i < header.bme68x_num; i++) {
            if (header.bme68x_fp) {
                bme68x_data[i].temp = view.getFloat64(offset, true);
                offset += 8;
            } else {
                bme68x_data[i].temp = view.getInt16(offset, true);
                offset += 2;
            }
        }

        // Pressure
        for (var i = 0; i < header.bme68x_num; i++) {
            if (header.bme68x_fp) {
                bme68x_data[i].press = view.getFloat64(offset, true);
                offset += 8;
            } else {
                bme68x_data[i].press = view.getInt32(offset, true);
                offset += 4;
            }
        }

        // Humidity
        for (var i = 0; i < header.bme68x_num; i++) {
            if (header.bme68x_fp) {
                bme68x_data[i].hum = view.getFloat64(offset, true);
                offset += 8;
            } else {
                bme68x_data[i].hum = view.getInt32(offset, true);
                offset += 4;
            }
        }

        // Gas
        for (var i = 0; i < header.bme68x_num; i++) {
            var gas = [];
            for (var j = 0; j < 10; j++) {
                if (header.bme68x_fp) {
                    gas.push(view.getFloat64(offset, true));
                    offset += 8;
                } else {
                    gas.push(view.getInt32(offset, true));
                    offset += 4;
                }
            }
            bme68x_data[i].gas = gas;
        }

        sensors.bme68x = bme68x_data;
    }

    // Parse SPS30 sensor data if present
    if (header.sps30_num) {
        var sps30_data = [];
        for (var k = 0; k < 10; k++) {
            sps30_data.push(view.getFloat32(offset, true));
            offset += 4;
        }
        sensors.sps30.push(sps30_data);
    }

    // Parse Senseair sensor data if present
    if (header.senseair_num) {
        var senseair_data = [];
        senseair_data.push(view.getUint16(offset, true));
        offset += 2;
        senseair_data.push(view.getInt16(offset, true));
        offset += 2;
        sensors.senseair.push(senseair_data);
    }

    // Parse BSEC sensor data if present
    if (header.bsec_num) {
        var bsec_data = [];
        for (var i = 0; i < header.bme68x_num; i++) {
            bsec_data.push({});
        }

        // IAQ
        for (var i = 0; i < header.bme68x_num; i++) {
            bsec_data[i].iaq = view.getUint16(offset, true);
            offset += 2;
        }

        // Accuracy
        for (var i = 0; i < header.bme68x_num; i++) {
            bsec_data[i].accuracy = view.getUint8(offset, true);
            offset += 1;
        }

        sensors.bsec = bsec_data;
    }

    decoded.header = header;
    decoded.sensors = sensors;

    return {
        data: decoded
    };
}


// Encode downlink function.
//
// Input is an object with the following fields:
// - data = Object representing the payload that must be encoded.
// - variables = Object containing the configured device variables.
//
// Output must be an object with the following fields:
// - bytes = Byte array containing the downlink payload.
//function encodeDownlink(input) {
//  return {
//    bytes: [225, 230, 255, 0]
//  };
//}
