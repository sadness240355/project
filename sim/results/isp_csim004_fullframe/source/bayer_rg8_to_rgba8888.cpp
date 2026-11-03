#include "bayer_rg8_to_rgba8888.hpp"

namespace {

struct isp_config_t {
    ap_uint<4>  enable;
    ap_uint<8>  black_level;
    ap_uint<16> wb_gain_r;
    ap_uint<16> wb_gain_g;
    ap_uint<16> wb_gain_b;
    ap_int<16>  ccm_00;
    ap_int<16>  ccm_01;
    ap_int<16>  ccm_02;
    ap_int<16>  ccm_10;
    ap_int<16>  ccm_11;
    ap_int<16>  ccm_12;
    ap_int<16>  ccm_20;
    ap_int<16>  ccm_21;
    ap_int<16>  ccm_22;
};

// IEC 61966-2-1 linear-light to sRGB transfer curve, sampled at every 8-bit
// input code. Twelve physical dual-port ROM banks provide 24 concurrent
// lookups (R/G/B for eight pixels), so gamma does not increase the loop II.
#define SRGB_LUT_VALUES \
      0,  13,  22,  28,  34,  38,  42,  46,  50,  53,  56,  59,  61,  64,  66,  69, \
     71,  73,  75,  77,  79,  81,  83,  85,  86,  88,  90,  92,  93,  95,  96,  98, \
     99, 101, 102, 104, 105, 106, 108, 109, 110, 112, 113, 114, 115, 117, 118, 119, \
    120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, \
    137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 148, 149, 150, 151, \
    152, 153, 154, 155, 155, 156, 157, 158, 159, 159, 160, 161, 162, 163, 163, 164, \
    165, 166, 167, 167, 168, 169, 170, 170, 171, 172, 173, 173, 174, 175, 175, 176, \
    177, 178, 178, 179, 180, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 187, \
    188, 189, 189, 190, 190, 191, 192, 192, 193, 194, 194, 195, 196, 196, 197, 197, \
    198, 199, 199, 200, 200, 201, 202, 202, 203, 203, 204, 205, 205, 206, 206, 207, \
    208, 208, 209, 209, 210, 210, 211, 212, 212, 213, 213, 214, 214, 215, 215, 216, \
    216, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 224, 224, \
    225, 226, 226, 227, 227, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, \
    233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238, 238, 239, 239, 240, 240, \
    241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 246, 247, 247, 248, \
    248, 249, 249, 250, 250, 251, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255

#define DECLARE_SRGB_ROM(N) \
    static const ap_uint<8> srgb_lut##N[256] = { SRGB_LUT_VALUES }
DECLARE_SRGB_ROM(0);
DECLARE_SRGB_ROM(1);
DECLARE_SRGB_ROM(2);
DECLARE_SRGB_ROM(3);
DECLARE_SRGB_ROM(4);
DECLARE_SRGB_ROM(5);
DECLARE_SRGB_ROM(6);
DECLARE_SRGB_ROM(7);
DECLARE_SRGB_ROM(8);
DECLARE_SRGB_ROM(9);
DECLARE_SRGB_ROM(10);
DECLARE_SRGB_ROM(11);
#undef DECLARE_SRGB_ROM
#undef SRGB_LUT_VALUES
static ap_uint<8> lane8(ap_uint<64> word, int lane)
{
#pragma HLS INLINE
    return word.range(lane * 8 + 7, lane * 8);
}

static ap_uint<8> preprocess_sample(ap_uint<8> raw, ap_uint<16> gain,
                                    const isp_config_t &cfg)
{
#pragma HLS INLINE
    ap_uint<8> corrected = raw;
    if (cfg.enable & ISP_ENABLE_BLC)
        corrected = (raw > cfg.black_level) ?
                    (ap_uint<8>)(raw - cfg.black_level) : (ap_uint<8>)0;

    if (!(cfg.enable & ISP_ENABLE_WB))
        return corrected;

    ap_uint<25> product = (ap_uint<25>)corrected * (ap_uint<25>)gain;
    ap_uint<17> scaled =
        (product + (1u << (ISP_FRAC_BITS - 1))) >> ISP_FRAC_BITS;
    return (scaled > 255) ? (ap_uint<8>)255 : (ap_uint<8>)scaled;
}

static ap_uint<64> preprocess_word(ap_uint<64> raw_word,
                                   ap_uint<1> row_odd,
                                   const isp_config_t &cfg)
{
#pragma HLS INLINE
    ap_uint<64> corrected_word = 0;
    for (int lane = 0; lane < BAYER_PPC; ++lane) {
#pragma HLS UNROLL
        const ap_uint<1> col_odd = (ap_uint<1>)(lane & 1);
        ap_uint<16> gain = cfg.wb_gain_g;
        if (!row_odd && !col_odd)
            gain = cfg.wb_gain_r;
        else if (row_odd && col_odd)
            gain = cfg.wb_gain_b;
        corrected_word.range(lane * 8 + 7, lane * 8) =
            preprocess_sample(lane8(raw_word, lane), gain, cfg);
    }
    return corrected_word;
}

static ap_uint<8> ccm_channel(ap_uint<8> r, ap_uint<8> g, ap_uint<8> b,
                              ap_int<16> cr, ap_int<16> cg, ap_int<16> cb)
{
#pragma HLS INLINE
    ap_int<28> acc =
        (ap_int<9>)r * cr + (ap_int<9>)g * cg + (ap_int<9>)b * cb;
    if (acc <= 0)
        return 0;

    ap_int<28> scaled =
        (acc + (1 << (ISP_FRAC_BITS - 1))) >> ISP_FRAC_BITS;
    return (scaled > 255) ? (ap_uint<8>)255 : (ap_uint<8>)scaled;
}

static void apply_ccm(ap_uint<8> r_in, ap_uint<8> g_in, ap_uint<8> b_in,
                      const isp_config_t &cfg,
                      ap_uint<8> &r_out, ap_uint<8> &g_out, ap_uint<8> &b_out)
{
#pragma HLS INLINE
    if (!(cfg.enable & ISP_ENABLE_CCM)) {
        r_out = r_in;
        g_out = g_in;
        b_out = b_in;
        return;
    }

    r_out = ccm_channel(r_in, g_in, b_in,
                        cfg.ccm_00, cfg.ccm_01, cfg.ccm_02);
    g_out = ccm_channel(r_in, g_in, b_in,
                        cfg.ccm_10, cfg.ccm_11, cfg.ccm_12);
    b_out = ccm_channel(r_in, g_in, b_in,
                        cfg.ccm_20, cfg.ccm_21, cfg.ccm_22);
}

static void apply_srgb_group(ap_uint<8> linear[24], ap_uint<8> encoded[24])
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=linear complete
#pragma HLS ARRAY_PARTITION variable=encoded complete
#pragma HLS BIND_STORAGE variable=srgb_lut0  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut1  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut2  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut3  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut4  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut5  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut6  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut7  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut8  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut9  type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut10 type=rom_2p impl=bram
#pragma HLS BIND_STORAGE variable=srgb_lut11 type=rom_2p impl=bram

    encoded[0]  = srgb_lut0[linear[0]];
    encoded[1]  = srgb_lut0[linear[1]];
    encoded[2]  = srgb_lut1[linear[2]];
    encoded[3]  = srgb_lut1[linear[3]];
    encoded[4]  = srgb_lut2[linear[4]];
    encoded[5]  = srgb_lut2[linear[5]];
    encoded[6]  = srgb_lut3[linear[6]];
    encoded[7]  = srgb_lut3[linear[7]];
    encoded[8]  = srgb_lut4[linear[8]];
    encoded[9]  = srgb_lut4[linear[9]];
    encoded[10] = srgb_lut5[linear[10]];
    encoded[11] = srgb_lut5[linear[11]];
    encoded[12] = srgb_lut6[linear[12]];
    encoded[13] = srgb_lut6[linear[13]];
    encoded[14] = srgb_lut7[linear[14]];
    encoded[15] = srgb_lut7[linear[15]];
    encoded[16] = srgb_lut8[linear[16]];
    encoded[17] = srgb_lut8[linear[17]];
    encoded[18] = srgb_lut9[linear[18]];
    encoded[19] = srgb_lut9[linear[19]];
    encoded[20] = srgb_lut10[linear[20]];
    encoded[21] = srgb_lut10[linear[21]];
    encoded[22] = srgb_lut11[linear[22]];
    encoded[23] = srgb_lut11[linear[23]];
}

static ap_uint<8> avg2(ap_uint<8> a, ap_uint<8> b)
{
#pragma HLS INLINE
    ap_uint<9> sum = (ap_uint<9>)a + (ap_uint<9>)b + 1;
    return (ap_uint<8>)(sum >> 1);
}

static ap_uint<8> avg4(ap_uint<8> a, ap_uint<8> b,
                       ap_uint<8> c, ap_uint<8> d)
{
#pragma HLS INLINE
    ap_uint<11> sum = (ap_uint<11>)a + (ap_uint<11>)b +
                      (ap_uint<11>)c + (ap_uint<11>)d + 2;
    return (ap_uint<8>)(sum >> 2);
}

// The 24-bit value is arranged so little-endian DDR byte order is R,G,B.
static ap_uint<24> pack_rgb(ap_uint<8> r, ap_uint<8> g, ap_uint<8> b)
{
#pragma HLS INLINE
    ap_uint<24> p = 0;
    p.range(7, 0)   = r;
    p.range(15, 8)  = g;
    p.range(23, 16) = b;
    return p;
}

// Interpolate eight adjacent Bayer pixels and emit their 24 contiguous RGB bytes.
static ap_uint<192> interpolate_group(
    ap_uint<64> north_word,
    ap_uint<64> center_word,
    ap_uint<64> south_word,
    ap_uint<8> north_left,
    ap_uint<8> center_left,
    ap_uint<8> south_left,
    ap_uint<8> north_right,
    ap_uint<8> center_right,
    ap_uint<8> south_right,
    ap_uint<1> row_odd,
    const isp_config_t &cfg)
{
#pragma HLS INLINE
    ap_uint<8> n[BAYER_PPC];
    ap_uint<8> c[BAYER_PPC];
    ap_uint<8> s[BAYER_PPC];
#pragma HLS ARRAY_PARTITION variable=n complete
#pragma HLS ARRAY_PARTITION variable=c complete
#pragma HLS ARRAY_PARTITION variable=s complete

    for (int i = 0; i < BAYER_PPC; ++i) {
#pragma HLS UNROLL
        n[i] = lane8(north_word, i);
        c[i] = lane8(center_word, i);
        s[i] = lane8(south_word, i);
    }

    ap_uint<8> linear[24];
    ap_uint<8> encoded[24];
#pragma HLS ARRAY_PARTITION variable=linear complete
#pragma HLS ARRAY_PARTITION variable=encoded complete
    ap_uint<192> packed = 0;
    for (int i = 0; i < BAYER_PPC; ++i) {
#pragma HLS UNROLL
        const ap_uint<1> col_odd = (ap_uint<1>)(i & 1);

        const ap_uint<8> center = c[i];
        const ap_uint<8> north  = n[i];
        const ap_uint<8> south  = s[i];
        const ap_uint<8> west   = (i == 0) ? center_left : c[i - 1];
        const ap_uint<8> east   = (i == BAYER_PPC - 1) ? center_right : c[i + 1];

        const ap_uint<8> nw = (i == 0) ? north_left : n[i - 1];
        const ap_uint<8> ne = (i == BAYER_PPC - 1) ? north_right : n[i + 1];
        const ap_uint<8> sw = (i == 0) ? south_left : s[i - 1];
        const ap_uint<8> se = (i == BAYER_PPC - 1) ? south_right : s[i + 1];

        ap_uint<8> r;
        ap_uint<8> g;
        ap_uint<8> b;

        // BayerRG / RGGB phase:
        // even row: R G R G ...
        // odd  row: G B G B ...
        if (!row_odd && !col_odd) {             // R site
            r = center;
            g = avg4(north, south, west, east);
            b = avg4(nw, ne, sw, se);
        } else if (row_odd && col_odd) {         // B site
            r = avg4(nw, ne, sw, se);
            g = avg4(north, south, west, east);
            b = center;
        } else if (!row_odd && col_odd) {        // G site on an R row
            r = avg2(west, east);
            g = center;
            b = avg2(north, south);
        } else {                                 // G site on a B row
            r = avg2(north, south);
            g = center;
            b = avg2(west, east);
        }

        apply_ccm(r, g, b, cfg,
                  linear[i * 3], linear[i * 3 + 1], linear[i * 3 + 2]);
    }
    if (cfg.enable & ISP_ENABLE_GAMMA) {
        apply_srgb_group(linear, encoded);
    } else {
        for (int i = 0; i < 24; ++i) {
#pragma HLS UNROLL
            encoded[i] = linear[i];
        }
    }

    for (int i = 0; i < BAYER_PPC; ++i) {
#pragma HLS UNROLL
        packed.range(i * 24 + 23, i * 24) =
            pack_rgb(encoded[i * 3], encoded[i * 3 + 1], encoded[i * 3 + 2]);
    }
    return packed;
}

static ap_uint<192> make_group(
    ap_uint<64> north_word,
    ap_uint<64> center_word,
    ap_uint<64> south_word,
    ap_uint<8> north_left,
    ap_uint<8> center_left,
    ap_uint<8> south_left,
    ap_uint<8> north_right,
    ap_uint<8> center_right,
    ap_uint<8> south_right,
    ap_uint<16> out_row,
    const isp_config_t &cfg)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    return interpolate_group(north_word, center_word, south_word,
                             north_left, center_left, south_left,
                             north_right, center_right, south_right,
                             (ap_uint<1>)(out_row & 1), cfg);
}

} // namespace

void bayer_rg8_to_rgba8888(
    hls::stream<bayer8_axis_t> &s_axis_bayer,
    hls::stream<rgb8_axis_t> &m_axis_rgb,
    ap_uint<16> width,
    ap_uint<16> height,
    ap_uint<4>  isp_enable,
    ap_uint<8>  black_level,
    ap_uint<16> wb_gain_r,
    ap_uint<16> wb_gain_g,
    ap_uint<16> wb_gain_b,
    ap_int<16>  ccm_00,
    ap_int<16>  ccm_01,
    ap_int<16>  ccm_02,
    ap_int<16>  ccm_10,
    ap_int<16>  ccm_11,
    ap_int<16>  ccm_12,
    ap_int<16>  ccm_20,
    ap_int<16>  ccm_21,
    ap_int<16>  ccm_22)
{
#pragma HLS INTERFACE axis register both port=s_axis_bayer
#pragma HLS INTERFACE axis register both port=m_axis_rgb
#pragma HLS INTERFACE s_axilite port=width  bundle=control offset=0x10
#pragma HLS INTERFACE s_axilite port=height bundle=control offset=0x18
#pragma HLS INTERFACE s_axilite port=isp_enable  bundle=control offset=0x20
#pragma HLS INTERFACE s_axilite port=black_level bundle=control offset=0x28
#pragma HLS INTERFACE s_axilite port=wb_gain_r    bundle=control offset=0x30
#pragma HLS INTERFACE s_axilite port=wb_gain_g    bundle=control offset=0x38
#pragma HLS INTERFACE s_axilite port=wb_gain_b    bundle=control offset=0x40
#pragma HLS INTERFACE s_axilite port=ccm_00 bundle=control offset=0x48
#pragma HLS INTERFACE s_axilite port=ccm_01 bundle=control offset=0x50
#pragma HLS INTERFACE s_axilite port=ccm_02 bundle=control offset=0x58
#pragma HLS INTERFACE s_axilite port=ccm_10 bundle=control offset=0x60
#pragma HLS INTERFACE s_axilite port=ccm_11 bundle=control offset=0x68
#pragma HLS INTERFACE s_axilite port=ccm_12 bundle=control offset=0x70
#pragma HLS INTERFACE s_axilite port=ccm_20 bundle=control offset=0x78
#pragma HLS INTERFACE s_axilite port=ccm_21 bundle=control offset=0x80
#pragma HLS INTERFACE s_axilite port=ccm_22 bundle=control offset=0x88
#pragma HLS INTERFACE s_axilite port=return bundle=control
#pragma HLS ALLOCATION function instances=make_group limit=1

    isp_config_t cfg;
    cfg.enable     = isp_enable;
    cfg.black_level = black_level;
    cfg.wb_gain_r  = wb_gain_r;
    cfg.wb_gain_g  = wb_gain_g;
    cfg.wb_gain_b  = wb_gain_b;
    cfg.ccm_00 = ccm_00; cfg.ccm_01 = ccm_01; cfg.ccm_02 = ccm_02;
    cfg.ccm_10 = ccm_10; cfg.ccm_11 = ccm_11; cfg.ccm_12 = ccm_12;
    cfg.ccm_20 = ccm_20; cfg.ccm_21 = ccm_21; cfg.ccm_22 = ccm_22;

    // Each BRAM entry stores eight Bayer pixels. Two complete rows provide the
    // north and center lines while the current stream word supplies the south.
    static ap_uint<64> row_m1[BAYER_MAX_GROUPS];
    static ap_uint<64> row_m2[BAYER_MAX_GROUPS];
#pragma HLS BIND_STORAGE variable=row_m1 type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=row_m2 type=ram_2p impl=bram
#pragma HLS DEPENDENCE variable=row_m1 inter false
#pragma HLS DEPENDENCE variable=row_m2 inter false

    if (width == 0 || height == 0 ||
        width > BAYER_MAX_WIDTH || height > BAYER_MAX_HEIGHT ||
        (width & (BAYER_PPC - 1)) != 0) {
        return;
    }

    const ap_uint<16> groups = width >> 3;

    // Read source rows. Row r produces row r-1 once its south neighbor exists.
    // The first source row initializes both line memories, implementing top-edge
    // replication without buffering a complete frame.
    for (ap_uint<16> r = 0; r < height; ++r) {
        ap_uint<64> prev_north = 0;
        ap_uint<64> prev_center = 0;
        ap_uint<64> prev_south = 0;
        ap_uint<8> left_north = 0;
        ap_uint<8> left_center = 0;
        ap_uint<8> left_south = 0;
        bool have_prev = false;

        // Do not read and write the same BRAM address in one clock.  Although
        // the inferred RAM is configured READ_FIRST, a cross-port same-address
        // collision is needlessly implementation-sensitive.  Hold the update
        // for group g and commit it while group g+1 is being read.  The two
        // line buffers still use one read and one write port each, so II=1 and
        // the BRAM count are unchanged.
        ap_uint<16> pending_group = 0;
        ap_uint<64> pending_m1 = 0;
        ap_uint<64> pending_m2 = 0;
        bool pending_write = false;

        for (ap_uint<16> g = 0; g < groups; ++g) {
#pragma HLS PIPELINE II=1
            bayer8_axis_t in = s_axis_bayer.read();
            const ap_uint<64> south_word = preprocess_word(
                in.data, (ap_uint<1>)(r & 1), cfg);

            ap_uint<64> center_word;
            ap_uint<64> north_word;
            if (r == 0) {
                center_word = south_word;
                north_word  = south_word;
            } else {
                center_word = row_m1[g];
                north_word  = row_m2[g];
            }

            // The reads above address g; the writes below address g-1 after
            // the pipeline has started.  Therefore there is no read/write
            // collision on either physical BRAM port.
            if (pending_write) {
                row_m1[pending_group] = pending_m1;
                row_m2[pending_group] = pending_m2;
            }
            pending_group = g;
            pending_m1 = south_word;
            pending_m2 = (r == 0) ? south_word : center_word;
            pending_write = true;

            if (r != 0) {
                if (!have_prev) {
                    prev_north  = north_word;
                    prev_center = center_word;
                    prev_south  = south_word;
                    left_north  = lane8(north_word, 0);
                    left_center = lane8(center_word, 0);
                    left_south  = lane8(south_word, 0);
                    have_prev = true;
                } else {
                    const ap_uint<16> out_row = r - 1;
                    const ap_uint<16> out_group = g - 1;
                    const ap_uint<192> group_data = make_group(
                        prev_north, prev_center, prev_south,
                        left_north, left_center, left_south,
                        lane8(north_word, 0),
                        lane8(center_word, 0),
                        lane8(south_word, 0), out_row, cfg);
                    const ap_uint<1> group_sof =
                        (out_row == 0 && out_group == 0) ? 1 : 0;
                    const ap_uint<1> group_last =
                        (out_row == height - 1 && out_group == groups - 1) ? 1 : 0;
                    rgb8_axis_t out;
                    out.data = group_data;
                    out.keep = (ap_uint<24>)-1;
                    out.strb = (ap_uint<24>)-1;
                    out.user = group_sof;
                    out.last = group_last;
                    out.id   = 0;
                    out.dest = 0;
                    m_axis_rgb.write(out);

                    left_north  = lane8(prev_north,  BAYER_PPC - 1);
                    left_center = lane8(prev_center, BAYER_PPC - 1);
                    left_south  = lane8(prev_south,  BAYER_PPC - 1);
                    prev_north  = north_word;
                    prev_center = center_word;
                    prev_south  = south_word;
                }
            }
        }

        // Commit the final group after its old row values have been consumed.
        // This shares the existing line-end flush interval and adds no steady-
        // state throughput cost.
        if (pending_write) {
            row_m1[pending_group] = pending_m1;
            row_m2[pending_group] = pending_m2;
        }

        // Replicate the last column for the east edge. This costs one processing
        // iteration per line, unchanged from the 192-bit-output implementation.
        if (r != 0 && have_prev) {
            const ap_uint<16> out_row = r - 1;
            const ap_uint<16> out_group = groups - 1;
            const ap_uint<192> group_data = make_group(
                prev_north, prev_center, prev_south,
                left_north, left_center, left_south,
                lane8(prev_north,  BAYER_PPC - 1),
                lane8(prev_center, BAYER_PPC - 1),
                lane8(prev_south,  BAYER_PPC - 1), out_row, cfg);
            const ap_uint<1> group_sof =
                (out_row == 0 && out_group == 0) ? 1 : 0;
            const ap_uint<1> group_last =
                (out_row == height - 1 && out_group == groups - 1) ? 1 : 0;
            rgb8_axis_t out;
            out.data = group_data;
            out.keep = (ap_uint<24>)-1;
            out.strb = (ap_uint<24>)-1;
            out.user = group_sof;
            out.last = group_last;
            out.id   = 0;
            out.dest = 0;
            m_axis_rgb.write(out);
        }
    }

    // Bottom-edge flush: row_m1 is the final source row and row_m2 is the row
    // above it. Replicate row_m1 as the missing south row.
    {
        ap_uint<64> prev_north = 0;
        ap_uint<64> prev_center = 0;
        ap_uint<64> prev_south = 0;
        ap_uint<8> left_north = 0;
        ap_uint<8> left_center = 0;
        ap_uint<8> left_south = 0;
        bool have_prev = false;

        for (ap_uint<16> g = 0; g < groups; ++g) {
#pragma HLS PIPELINE II=1
            const ap_uint<64> north_word  = row_m2[g];
            const ap_uint<64> center_word = row_m1[g];
            const ap_uint<64> south_word  = center_word;

            if (!have_prev) {
                prev_north  = north_word;
                prev_center = center_word;
                prev_south  = south_word;
                left_north  = lane8(north_word, 0);
                left_center = lane8(center_word, 0);
                left_south  = lane8(south_word, 0);
                have_prev = true;
            } else {
                const ap_uint<16> out_row = height - 1;
                const ap_uint<16> out_group = g - 1;
                const ap_uint<192> group_data = make_group(
                    prev_north, prev_center, prev_south,
                    left_north, left_center, left_south,
                    lane8(north_word, 0),
                    lane8(center_word, 0),
                    lane8(south_word, 0), out_row, cfg);
                const ap_uint<1> group_sof =
                    (out_row == 0 && out_group == 0) ? 1 : 0;
                const ap_uint<1> group_last =
                    (out_group == groups - 1) ? 1 : 0;
                rgb8_axis_t out;
                out.data = group_data;
                out.keep = (ap_uint<24>)-1;
                out.strb = (ap_uint<24>)-1;
                out.user = group_sof;
                out.last = group_last;
                out.id   = 0;
                out.dest = 0;
                m_axis_rgb.write(out);

                left_north  = lane8(prev_north,  BAYER_PPC - 1);
                left_center = lane8(prev_center, BAYER_PPC - 1);
                left_south  = lane8(prev_south,  BAYER_PPC - 1);
                prev_north  = north_word;
                prev_center = center_word;
                prev_south  = south_word;
            }
        }

        if (have_prev) {
            const ap_uint<16> out_row = height - 1;
            const ap_uint<16> out_group = groups - 1;
            const ap_uint<192> group_data = make_group(
                prev_north, prev_center, prev_south,
                left_north, left_center, left_south,
                lane8(prev_north,  BAYER_PPC - 1),
                lane8(prev_center, BAYER_PPC - 1),
                lane8(prev_south,  BAYER_PPC - 1), out_row, cfg);
            const ap_uint<1> group_sof =
                (out_row == 0 && out_group == 0) ? 1 : 0;
            const ap_uint<1> group_last = 1;
            rgb8_axis_t out;
            out.data = group_data;
            out.keep = (ap_uint<24>)-1;
            out.strb = (ap_uint<24>)-1;
            out.user = group_sof;
            out.last = group_last;
            out.id   = 0;
            out.dest = 0;
            m_axis_rgb.write(out);
        }
    }
}
