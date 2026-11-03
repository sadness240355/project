/* SPDX-License-Identifier: MIT */
/*
 * Minimal Hikrobot Camera Link serial configurator for the raw BayerRG8 path.
 *
 * Supported camera mode:
 *   BayerRG8 / PFNC 0x01080009
 *   Request DeviceTapGeometry 0x010A0101 after PixelFormat.  Some camera
 *   firmware resets geometry when PixelFormat changes, so the runner verifies
 *   the final readback.  The FPGA is fixed at matching 80-bit/10-byte Deca capture.
 *   user-selected Width, Height and AcquisitionFrameRate
 */

#include "edev_camera_io.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define CSR_MAP_SIZE 0x1000u
#define RESP_BUF_SIZE 4096u

#define HIK_BAYERRG8_PIXEL_FORMAT   "0x01080009"
#define HIK_DECA10_TAP_GEOMETRY    "0x010A0101"
#define HIK_FRAME_RATE_ENABLE_FLAG 1073741824u
#define HIK_FRAME_RATE_SCALE       10000.0

struct fpga_serial {
    int fd;
    volatile uint8_t *csr;
};

static long long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000ll + (long long)ts.tv_nsec / 1000000ll;
}

static uint32_t csr_read32(struct fpga_serial *fpga, uint32_t reg)
{
    return *(volatile uint32_t *)(fpga->csr + reg);
}

static void csr_write32(struct fpga_serial *fpga, uint32_t reg, uint32_t value)
{
    *(volatile uint32_t *)(fpga->csr + reg) = value;
    (void)csr_read32(fpga, CAMERA_REG_CONTROL);
}

static int parse_u32(const char *s, uint32_t *value)
{
    char *end = NULL;
    unsigned long v;

    if (!s || !value)
        return -1;
    errno = 0;
    v = strtoul(s, &end, 0);
    if (errno || end == s || *end != '\0' || v > UINT32_MAX)
        return -1;
    *value = (uint32_t)v;
    return 0;
}

static int parse_u64(const char *s, uint64_t *value)
{
    char *end = NULL;
    unsigned long long v;

    if (!s || !value)
        return -1;
    errno = 0;
    v = strtoull(s, &end, 0);
    if (errno || end == s || *end != '\0')
        return -1;
    *value = (uint64_t)v;
    return 0;
}

static int encode_frame_rate(const char *s, uint32_t *code)
{
    char *end = NULL;
    double fps;
    double scaled;
    uint64_t raw;

    if (!s || !code)
        return -1;
    errno = 0;
    fps = strtod(s, &end);
    if (errno || end == s || *end != '\0' || !isfinite(fps) || fps < 0.0 || fps > 10000.0)
        return -1;

    scaled = fps * HIK_FRAME_RATE_SCALE;
    raw = (uint64_t)(scaled + 0.5);
    if (raw > UINT32_MAX - HIK_FRAME_RATE_ENABLE_FLAG)
        return -1;
    *code = raw ? HIK_FRAME_RATE_ENABLE_FLAG + (uint32_t)raw : 0u;
    return 0;
}

static int fpga_open(struct fpga_serial *fpga, const char *ctrl, uint64_t csr_base)
{
    memset(fpga, 0, sizeof(*fpga));
    fpga->fd = -1;
    fpga->csr = MAP_FAILED;
    fpga->fd = open(ctrl, O_RDWR | O_CLOEXEC);
    if (fpga->fd < 0)
        return -1;
    fpga->csr = mmap(NULL, CSR_MAP_SIZE, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fpga->fd, (off_t)csr_base);
    if (fpga->csr == MAP_FAILED) {
        close(fpga->fd);
        fpga->fd = -1;
        return -1;
    }
    return 0;
}

static void fpga_close(struct fpga_serial *fpga)
{
    if (!fpga)
        return;
    if (fpga->csr != MAP_FAILED)
        munmap((void *)fpga->csr, CSR_MAP_SIZE);
    if (fpga->fd >= 0)
        close(fpga->fd);
}

static int fpga_configure_uart_baud(struct fpga_serial *fpga, uint32_t baud)
{
    uint32_t clk_hz;
    uint32_t divisor;
    uint32_t readback;
    uint32_t status;

    if (!fpga || fpga->csr == MAP_FAILED || !baud) {
        errno = EINVAL;
        return -1;
    }

    clk_hz = csr_read32(fpga, CAMERA_REG_SERIAL_CLOCK_HZ);
    if (!clk_hz || clk_hz == UINT32_MAX)
        clk_hz = CAMERA_DEFAULT_AXIL_CLOCK_HZ;

    divisor = (clk_hz + baud / 2u) / baud;
    if (divisor < CAMERA_SERIAL_MIN_BAUD_DIV) {
        fprintf(stderr,
                "UART clock %u Hz cannot generate requested %u baud\n",
                clk_hz, baud);
        errno = ERANGE;
        return -1;
    }

    csr_write32(fpga, CAMERA_REG_SERIAL_BAUD_DIV, divisor);
    usleep(1000);
    readback = csr_read32(fpga, CAMERA_REG_SERIAL_BAUD_DIV);
    status = csr_read32(fpga, CAMERA_REG_SERIAL_STATUS);
    if (readback != divisor ||
        (status & CAMERA_SERIAL_STATUS_BAUD_DIV_ERR)) {
        fprintf(stderr,
                "UART divider setup failed: clk=%u baud=%u wrote=%u read=%u status=0x%08x\n",
                clk_hz, baud, divisor, readback, status);
        errno = EIO;
        return -1;
    }

    fprintf(stderr,
            "UART configured: clk=%u Hz target=%u baud divisor=%u actual=%.3f baud\n",
            clk_hz, baud, divisor, (double)clk_hz / (double)divisor);
    return 0;
}

static int fpga_wait_tx_ready(struct fpga_serial *fpga, int timeout_ms)
{
    long long deadline = now_ms() + timeout_ms;
    for (;;) {
        uint32_t st = csr_read32(fpga, CAMERA_REG_SERIAL_STATUS);
        if (st & CAMERA_SERIAL_STATUS_TX_READY)
            return 0;
        if (now_ms() >= deadline) {
            fprintf(stderr, "timeout waiting serial TX ready, status=0x%08x\n", st);
            errno = ETIMEDOUT;
            return -1;
        }
        usleep(200);
    }
}

static int fpga_drain_response(struct fpga_serial *fpga, char *buf, size_t len,
                               int idle_ms, int total_ms)
{
    long long deadline = now_ms() + total_ms;
    long long idle_deadline = now_ms() + idle_ms;
    size_t used = 0;
    int saw = 0;

    if (buf && len)
        buf[0] = '\0';
    while (now_ms() < deadline) {
        uint32_t st = csr_read32(fpga, CAMERA_REG_SERIAL_STATUS);
        if (st & CAMERA_SERIAL_STATUS_RX_VALID) {
            uint32_t r = csr_read32(fpga, CAMERA_REG_SERIAL_RXDATA);
            if (r & CAMERA_SERIAL_RXDATA_VALID) {
                saw = 1;
                if (buf && len > 1 && used + 1 < len) {
                    buf[used++] = (char)(r & 0xffu);
                    buf[used] = '\0';
                }
            }
            idle_deadline = now_ms() + idle_ms;
            continue;
        }
        if (saw && now_ms() >= idle_deadline)
            break;
        usleep(1000);
    }
    return (int)used;
}

static int fpga_send_cmd(struct fpga_serial *fpga, const char *cmd,
                         char *resp, size_t resp_len, int timeout_ms)
{
    size_t len = strlen(cmd);

    csr_write32(fpga, CAMERA_REG_SERIAL_CTRL, CAMERA_SERIAL_CTRL_CLEAR);
    (void)fpga_drain_response(fpga, NULL, 0, 0, 0);

    for (size_t i = 0; i < len; i++) {
        if (fpga_wait_tx_ready(fpga, timeout_ms) < 0)
            return -1;
        csr_write32(fpga, CAMERA_REG_SERIAL_TXDATA, (uint8_t)cmd[i]);
    }
    if (len == 0 || cmd[len - 1] != '\r') {
        if (fpga_wait_tx_ready(fpga, timeout_ms) < 0)
            return -1;
        csr_write32(fpga, CAMERA_REG_SERIAL_TXDATA, (uint8_t)'\r');
    }

    return fpga_drain_response(fpga, resp, resp_len, 120, timeout_ms) >= 0 ? 0 : -1;
}

static void print_response(FILE *out, const char *s)
{
    const unsigned char *p = (const unsigned char *)s;
    if (!s || !*s) {
        fprintf(out, "(no response)\n");
        return;
    }
    for (; *p; p++) {
        if (*p == '\r')
            continue;
        if (*p == '\n' || *p == '\t')
            fputc(*p, out);
        else if (*p >= 0x20 && *p < 0x7f)
            fputc(*p, out);
        else
            fprintf(out, "\\x%02x", *p);
    }
    if (p == (const unsigned char *)s || p[-1] != '\n')
        fputc('\n', out);
}

static int response_failed(const char *s)
{
    return s && (strstr(s, "Failed!") ||
                 strstr(s, "Wrong input format") ||
                 strstr(s, "NOT supported") ||
                 strstr(s, "not supported"));
}

static int response_valid_enough(const char *s)
{
    return s && *s && !response_failed(s);
}

static int command_is_write(const char *cmd)
{
    return cmd && cmd[0] == 'w' && (cmd[1] == ' ' || cmd[1] == '\t');
}

static int extract_write_feature(const char *cmd, char *feature, size_t feature_len)
{
    const char *p;
    size_t n = 0;

    if (!command_is_write(cmd) || !feature || feature_len == 0)
        return -1;
    p = cmd + 1;
    while (*p == ' ' || *p == '\t')
        p++;
    while (*p && *p != ' ' && *p != '\t' && n + 1 < feature_len)
        feature[n++] = *p++;
    feature[n] = '\0';
    return n ? 0 : -1;
}

static int run_cmd_checked(struct fpga_serial *fpga, const char *cmd,
                           int timeout_ms, int write_settle_ms)
{
    char resp[RESP_BUF_SIZE];

    printf("# %s\n", cmd);
    printf("HKVS > %s\n", cmd);
    if (fpga_send_cmd(fpga, cmd, resp, sizeof(resp), timeout_ms) < 0)
        return -1;
    print_response(stdout, resp);

    if (response_failed(resp)) {
        errno = EIO;
        return -1;
    }

    if (!command_is_write(cmd) && !response_valid_enough(resp)) {
        fprintf(stderr, "invalid or missing response for '%s'\n", cmd);
        errno = EIO;
        return -1;
    }

    if (command_is_write(cmd) && !response_valid_enough(resp)) {
        char feature[96];
        char read_cmd[128];
        char verify[RESP_BUF_SIZE];

        if (extract_write_feature(cmd, feature, sizeof(feature)) == 0) {
            if (write_settle_ms > 0)
                usleep((useconds_t)write_settle_ms * 1000u);
            snprintf(read_cmd, sizeof(read_cmd), "r %s", feature);
            fprintf(stderr,
                    "warning: no immediate response for '%s'; verifying with '%s'\n",
                    cmd, read_cmd);
            if (fpga_send_cmd(fpga, read_cmd, verify, sizeof(verify), timeout_ms) < 0)
                return -1;
            if (response_valid_enough(verify)) {
                fprintf(stderr, "warning: verified %s after missing response; continuing\n", feature);
                printf("HKVS > %s\n", read_cmd);
                print_response(stdout, verify);
                return 0;
            }
            fprintf(stderr, "verification failed after missing response for '%s'\n", cmd);
            print_response(stderr, verify);
        }
        errno = EIO;
        return -1;
    }
    return 0;
}

static int run_printf_cmd(struct fpga_serial *fpga, int timeout_ms, int write_settle_ms,
                          const char *fmt, ...)
{
    char cmd[160];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(cmd, sizeof(cmd), fmt, ap);
    va_end(ap);
    if (n < 0 || n >= (int)sizeof(cmd)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    return run_cmd_checked(fpga, cmd, timeout_ms, write_settle_ms);
}

static int response_get_u32(const char *resp, const char *feature, uint32_t *value)
{
    char needle[128];
    const char *p;
    char *end = NULL;
    unsigned long v;

    if (!resp || !feature || !value ||
        snprintf(needle, sizeof(needle), "get %s:", feature) >= (int)sizeof(needle))
        return -1;
    p = strstr(resp, needle);
    if (!p)
        return -1;
    p += strlen(needle);
    errno = 0;
    v = strtoul(p, &end, 0);
    if (errno || end == p || v > UINT32_MAX)
        return -1;
    *value = (uint32_t)v;
    return 0;
}

/* Check the current transport's limits, not the sensor brochure maximum.
 * Some firmware reports the requested Width even after a rejected write. */
static int read_feature_u32(struct fpga_serial *fpga, const char *node,
                            uint32_t *value, int timeout_ms)
{
    char cmd[128], resp[RESP_BUF_SIZE];
    snprintf(cmd, sizeof(cmd), "r %s", node);
    printf("# %s\n", cmd);
    if (fpga_send_cmd(fpga, cmd, resp, sizeof(resp), timeout_ms) < 0)
        return -1;
    print_response(stdout, resp);
    if (response_failed(resp) || response_get_u32(resp, node, value) < 0) {
        fprintf(stderr, "cannot verify camera %s; refusing geometry update\n", node);
        errno = EPROTO;
        return -1;
    }
    return 0;
}

static int check_transport_geometry(struct fpga_serial *fpga, uint32_t width,
                                    uint32_t height, int timeout_ms)
{
    uint32_t max_width = 0, max_height = 0;
    if (read_feature_u32(fpga, "WidthMax", &max_width, timeout_ms) < 0 ||
        read_feature_u32(fpga, "HeightMax", &max_height, timeout_ms) < 0)
        return -1;
    if (!max_width || !max_height || width > max_width || height > max_height) {
        fprintf(stderr, "camera transport geometry exceeds current mode: requested=%ux%u "
                "WidthMax=%u HeightMax=%u. A nominal Width readback is not proof "
                "of an accepted ROI. Select a supported size or adapt the link mode.\n",
                width, height, max_width, max_height);
        errno = ERANGE;
        return -1;
    }
    return 0;
}

static int run_u32_target(struct fpga_serial *fpga, const char *feature,
                          uint32_t target, int timeout_ms, int write_settle_ms)
{
    char write_cmd[160];
    char read_cmd[128];
    char resp[RESP_BUF_SIZE];
    uint32_t actual = 0;
    int saved_errno;

    if (snprintf(write_cmd, sizeof(write_cmd), "w %s %u", feature, target) >=
        (int)sizeof(write_cmd)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    if (run_cmd_checked(fpga, write_cmd, timeout_ms, write_settle_ms) == 0)
        return 0;

    saved_errno = errno;
    if (!strcmp(feature, "Width") || !strcmp(feature, "Height")) {
        fprintf(stderr, "ROI write rejected: %s=%u; refusing to accept a nominal "
                "readback as successful configuration\n", feature, target);
        return -1;
    }
    if (write_settle_ms > 0)
        usleep((useconds_t)write_settle_ms * 1000u);
    if (snprintf(read_cmd, sizeof(read_cmd), "r %s", feature) >=
        (int)sizeof(read_cmd)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    fprintf(stderr,
            "warning: write of %s=%u was rejected; verifying the effective value\n",
            feature, target);
    printf("# %s\n", read_cmd);
    printf("HKVS > %s\n", read_cmd);
    if (fpga_send_cmd(fpga, read_cmd, resp, sizeof(resp), timeout_ms) < 0)
        return -1;
    print_response(stdout, resp);
    if (!response_failed(resp) &&
        response_get_u32(resp, feature, &actual) == 0 && actual == target) {
        fprintf(stderr,
                "warning: %s is locked but already equals %u; continuing\n",
                feature, target);
        return 0;
    }
    fprintf(stderr,
            "effective %s mismatch after rejected write: expected=%u actual=%u\n",
            feature, target, actual);
    errno = saved_errno ? saved_errno : EIO;
    return -1;
}

static void print_serial_status(struct fpga_serial *fpga)
{
    uint32_t st = csr_read32(fpga, CAMERA_REG_SERIAL_STATUS);
    uint32_t div = csr_read32(fpga, CAMERA_REG_SERIAL_BAUD_DIV);
    uint32_t clk_hz = csr_read32(fpga, CAMERA_REG_SERIAL_CLOCK_HZ);
    uint32_t rx_count = csr_read32(fpga, CAMERA_REG_SERIAL_RX_COUNT);
    uint32_t tx_count = csr_read32(fpga, CAMERA_REG_SERIAL_TX_COUNT);
    double actual_baud = div ? (double)clk_hz / (double)div : 0.0;

    fprintf(stderr,
            "serial status=0x%08x clk=%uHz baud_div=%u actual_baud=%.3f "
            "rx_count=%u tx_count=%u%s%s%s%s%s%s%s\n",
            st, clk_hz, div, actual_baud, rx_count, tx_count,
            (st & CAMERA_SERIAL_STATUS_TX_READY) ? " tx_ready" : "",
            (st & CAMERA_SERIAL_STATUS_TX_BUSY) ? " tx_busy" : "",
            (st & CAMERA_SERIAL_STATUS_RX_VALID) ? " rx_valid" : "",
            (st & CAMERA_SERIAL_STATUS_RX_BUSY) ? " rx_busy" : "",
            (st & CAMERA_SERIAL_STATUS_RX_FRAME_ERR) ? " rx_frame_err" : "",
            (st & CAMERA_SERIAL_STATUS_RX_OVERFLOW) ? " rx_overflow" : "",
            (st & CAMERA_SERIAL_STATUS_BAUD_DIV_ERR) ? " baud_div_error" : "");
}

static void print_polarity(struct fpga_serial *fpga)
{
    uint32_t pol = csr_read32(fpga, CAMERA_REG_SERIAL_POLARITY);
    uint32_t cc_inv = (pol & CAMERA_SERIAL_POLARITY_CC_INVERT_MASK) >>
                      CAMERA_SERIAL_POLARITY_CC_INVERT_SHIFT;
    fprintf(stderr,
            "serial polarity: raw=0x%08x ser_rx_inv=%u ser_tx_inv=%u cc_inv=0x%x rx_raw=%u rx_effective=%u\n",
            pol,
            !!(pol & CAMERA_SERIAL_POLARITY_RX_INVERT),
            !!(pol & CAMERA_SERIAL_POLARITY_TX_INVERT),
            cc_inv,
            !!(pol & CAMERA_SERIAL_POLARITY_RX_RAW_HIGH),
            !!(pol & CAMERA_SERIAL_POLARITY_RX_EFF_HIGH));
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Only the Orin BayerRG8 Camera Link mode is supported.\n"
            "Options:\n"
            "  --configure-bayer8       write BayerRG8 mode plus Width/Height/FPS and request 10-byte 80-bit geometry after PixelFormat\n"
            "  --width N               active Width, required with --configure-bayer8\n"
            "  --height N              active Height, required with --configure-bayer8\n"
            "  --fps FPS               frame rate, required with --configure-bayer8\n"
            "  --test-pattern N        TestPattern value, default 0\n"
            "  --readback              read back key camera parameters after configuration\n"
            "  --cmd TEXT              send one raw camera serial command; may repeat\n"
            "  --ctrl PATH             control device, default %s\n"
            "  --csr-base ADDR         camera CSR BAR2 offset, default 0x%llx\n"
            "  --cmd-timeout-ms N      serial command timeout, default 5000\n"
            "  --write-settle-ms N     wait before write verification, default 100\n"
            "  --serial-status         print FPGA UART status\n"
            "  --show-polarity         print FPGA UART polarity register\n"
            "  --fpga                  accepted for old script invocation, no other backend exists\n",
            prog, CAMERA_DEFAULT_CTRL_DEVICE,
            (unsigned long long)CAMERA_DEFAULT_CSR_BASE);
}

int main(int argc, char **argv)
{
    const char *ctrl = CAMERA_DEFAULT_CTRL_DEVICE;
    uint64_t csr_base = CAMERA_DEFAULT_CSR_BASE;
    uint32_t width = 0, height = 0, test_pattern = 0;
    uint32_t frame_rate_code = 0;
    const char *fps_arg = NULL;
    int configure_bayer8 = 0;
    int readback = 0;
    int serial_status = 0;
    int show_polarity = 0;
    int timeout_ms = 5000;
    int write_settle_ms = 100;
    const char *raw_cmds[64];
    unsigned int raw_cmd_count = 0;
    struct fpga_serial fpga;
    int ret = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--fpga")) {
            /* Only FPGA CSR serial is supported. */
        } else if (!strcmp(argv[i], "--configure-bayer8")) {
            configure_bayer8 = 1;
        } else if (!strcmp(argv[i], "--width") && i + 1 < argc) {
            if (parse_u32(argv[++i], &width)) { usage(argv[0]); return 2; }
        } else if (!strcmp(argv[i], "--height") && i + 1 < argc) {
            if (parse_u32(argv[++i], &height)) { usage(argv[0]); return 2; }
        } else if ((!strcmp(argv[i], "--fps") || !strcmp(argv[i], "--frame-rate")) && i + 1 < argc) {
            fps_arg = argv[++i];
        } else if (!strcmp(argv[i], "--test-pattern") && i + 1 < argc) {
            if (parse_u32(argv[++i], &test_pattern)) { usage(argv[0]); return 2; }
        } else if (!strcmp(argv[i], "--cmd") && i + 1 < argc) {
            if (raw_cmd_count >= sizeof(raw_cmds) / sizeof(raw_cmds[0])) {
                fprintf(stderr, "too many --cmd entries\n");
                return 2;
            }
            raw_cmds[raw_cmd_count++] = argv[++i];
        } else if (!strcmp(argv[i], "--ctrl") && i + 1 < argc) {
            ctrl = argv[++i];
        } else if (!strcmp(argv[i], "--csr-base") && i + 1 < argc) {
            if (parse_u64(argv[++i], &csr_base)) { usage(argv[0]); return 2; }
        } else if (!strcmp(argv[i], "--cmd-timeout-ms") && i + 1 < argc) {
            uint32_t v;
            if (parse_u32(argv[++i], &v)) { usage(argv[0]); return 2; }
            timeout_ms = (int)v;
        } else if (!strcmp(argv[i], "--write-settle-ms") && i + 1 < argc) {
            uint32_t v;
            if (parse_u32(argv[++i], &v)) { usage(argv[0]); return 2; }
            write_settle_ms = (int)v;
        } else if (!strcmp(argv[i], "--readback")) {
            readback = 1;
        } else if (!strcmp(argv[i], "--serial-status")) {
            serial_status = 1;
        } else if (!strcmp(argv[i], "--show-polarity")) {
            show_polarity = 1;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 2;
        }
    }

    if (configure_bayer8) {
        if (!width || !height || width > 2432u || height > 2048u ||
            (width & 7u) || !fps_arg ||
            encode_frame_rate(fps_arg, &frame_rate_code)) {
            fprintf(stderr,
                    "--configure-bayer8 requires width=8..2432 divisible by 8, "
                    "height=1..2048 and a valid --fps\n");
            return 2;
        }
    }

    if (fpga_open(&fpga, ctrl, csr_base) < 0) {
        fprintf(stderr, "open/mmap FPGA serial CSR failed: %s\n", strerror(errno));
        return 1;
    }
    if (fpga_configure_uart_baud(&fpga, CAMERA_DEFAULT_SERIAL_BAUD) < 0) {
        fprintf(stderr, "configure FPGA camera UART failed: %s\n", strerror(errno));
        fpga_close(&fpga);
        return 1;
    }

    if (show_polarity)
        print_polarity(&fpga);
    if (serial_status)
        print_serial_status(&fpga);

    for (unsigned int i = 0; i < raw_cmd_count; i++) {
        if (run_cmd_checked(&fpga, raw_cmds[i], timeout_ms, write_settle_ms) < 0) {
            fprintf(stderr, "command failed: %s\n", raw_cmds[i]);
            ret = 1;
            goto out;
        }
    }

    if (configure_bayer8) {
        /* PixelFormat can restore transport/ROI defaults on MV-CH050 firmware.
         * Program it first, then apply the final 80-bit Deca tap geometry.
         * ReverseX/ReverseY also change the Bayer phase: if either retained
         * value is nonzero, PixelFormat still says BayerRG8 but the fixed RGGB
         * demosaic produces false colours. Force both directions to normal,
         * then move the ROI origin to (0,0) before setting Width/Height. */
        if (run_cmd_checked(&fpga, "w DeviceClockFrequency 0", timeout_ms, write_settle_ms) < 0 ||
            run_cmd_checked(&fpga, "w PixelFormat " HIK_BAYERRG8_PIXEL_FORMAT, timeout_ms, write_settle_ms) < 0 ||
            run_cmd_checked(&fpga, "w DeviceTapGeometry " HIK_DECA10_TAP_GEOMETRY, timeout_ms, write_settle_ms) < 0 ||
            run_u32_target(&fpga, "ReverseX", 0u, timeout_ms, write_settle_ms) < 0 ||
            run_u32_target(&fpga, "ReverseY", 0u, timeout_ms, write_settle_ms) < 0 ||
            run_u32_target(&fpga, "OffsetX", 0u, timeout_ms, write_settle_ms) < 0 ||
            run_u32_target(&fpga, "OffsetY", 0u, timeout_ms, write_settle_ms) < 0 ||
            check_transport_geometry(&fpga, width, height, timeout_ms) < 0 ||
            run_u32_target(&fpga, "Width", width, timeout_ms, write_settle_ms) < 0 ||
            run_u32_target(&fpga, "Height", height, timeout_ms, write_settle_ms) < 0 ||
            run_printf_cmd(&fpga, timeout_ms, write_settle_ms, "w AcquisitionFrameRate %u", frame_rate_code) < 0 ||
            run_printf_cmd(&fpga, timeout_ms, write_settle_ms, "w TestPattern %u", test_pattern) < 0) {
            fprintf(stderr, "BayerRG8 configuration failed\n");
            ret = 1;
            goto out;
        }
    }

    if (readback) {
        const char *reads[] = {
            "r Width", "r Height", "r ReverseX", "r ReverseY",
            "r OffsetX", "r OffsetY",
            "r PixelFormat", "r DeviceTapGeometry",
            "r AcquisitionFrameRate", "r TestPattern", "r TriggerMode"
        };
        for (unsigned int i = 0; i < sizeof(reads) / sizeof(reads[0]); i++) {
            if (run_cmd_checked(&fpga, reads[i], timeout_ms, write_settle_ms) < 0) {
                fprintf(stderr, "readback failed: %s\n", reads[i]);
                ret = 1;
                goto out;
            }
        }
    }

out:
    if (serial_status)
        print_serial_status(&fpga);
    fpga_close(&fpga);
    return ret;
}
