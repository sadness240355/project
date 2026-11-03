#ifndef BAYER_RG8_TO_RGBA8888_HPP
#define BAYER_RG8_TO_RGBA8888_HPP

#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <hls_stream.h>

// Camera Link Full supplies eight BayerRG8 samples per 64-bit input beat.
static const int BAYER_PPC        = 8;
static const int RGB_PPC_OUT      = 8;  // 8 RGB888 pixels per 192-bit output beat
static const int BAYER_MAX_WIDTH  = 4096;
static const int BAYER_MAX_HEIGHT = 4096;
static const int BAYER_MAX_GROUPS = BAYER_MAX_WIDTH / BAYER_PPC;

// Fixed-point formats used by the programmable ISP stages.
// White-balance gains are unsigned Q4.12 (1.0 == 0x1000).
// Color-correction coefficients are signed Q4.12
// (1.0 == 0x1000, -1.0 == 0xf000).
static const int ISP_FRAC_BITS = 12;

enum isp_enable_bits {
    ISP_ENABLE_BLC   = 1 << 0,
    ISP_ENABLE_WB    = 1 << 1,
    ISP_ENABLE_CCM   = 1 << 2,
    ISP_ENABLE_GAMMA = 1 << 3
};

// TUSER[0] marks start of frame. TLAST marks the final output beat of a frame.
typedef ap_axiu<64,  1, 1, 1> bayer8_axis_t;
typedef ap_axiu<192, 1, 1, 1> rgb8_axis_t;

// One invocation converts exactly one frame.
// width must be a positive multiple of 8 and no larger than BAYER_MAX_WIDTH.
// Each input beat produces one full 24-byte RGB888 output beat. TUSER[0]
// marks the first group and TLAST marks the final group of the frame. A
// downstream byte packer combines these compact 192-bit beats into contiguous
// 512-bit DDR writes without inserting padding between RGB pixels.
// Keep the legacy top-level function name so the existing Vivado XCI can be
// upgraded in place. Its output interface and payload are compact RGB888.
void bayer_rg8_to_rgba8888(
    hls::stream<bayer8_axis_t>  &s_axis_bayer,
    hls::stream<rgb8_axis_t>    &m_axis_rgb,
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
    ap_int<16>  ccm_22);

#endif
