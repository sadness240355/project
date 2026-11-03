#ifndef VIDEO_SESSION_WAIT_H
#define VIDEO_SESSION_WAIT_H
#include "video_common.h"
#include "video_notify.h"
#include <poll.h>
#include <inttypes.h>
#include <signal.h>

/* An idle interval is not permission to destroy an established continuous
 * camera session. Keep the QP/MR until END or a control-channel disconnect.
 * Finite diagnostics still have a strict idle deadline. */
static int video_wait_rdma_frame(const uint8_t *slot, uint32_t payload,
                                uint64_t expected, int peer_fd,
                                int continuous, uint64_t idle_ns,
                                const volatile sig_atomic_t *stop_requested,
                                struct video_header *frame)
{
    const uint64_t *commit =
        (const uint64_t *)(slot + video_meta_offset(payload) + 64);
    uint64_t deadline = video_ns() + idle_ns;
    int reported_idle = 0;
    for (;;) {
        uint64_t serial = __atomic_load_n(commit, __ATOMIC_ACQUIRE);
        if (serial == expected) {
            memcpy(frame, slot + video_meta_offset(payload), sizeof(*frame));
            if (reported_idle)
                fprintf(stderr, "RX_IDLE waiting_source=0 session_alive=1\n");
            return 0;
        }
        if (serial > expected) {
            fprintf(stderr, "FAIL: invalid RDMA slot generation\n");
            return -1;
        }
        struct pollfd poller = {peer_fd, POLLIN, 0};
        int ready = poll(&poller, 1, 0);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "FAIL: video control poll error\n");
            return -1;
        }
        if (ready > 0 && (poller.revents & POLLIN)) {
            if (rn_mlx5_recv_all(peer_fd, frame, sizeof(*frame))) {
                fprintf(stderr, "FAIL: video control disconnected\n");
                return -1;
            }
            if (ntohl(frame->magic) != VIDEO_MAGIC ||
                ntohl(frame->version) != VIDEO_VERSION ||
                ntohl(frame->type) != VIDEO_END) {
                fprintf(stderr, "FAIL: unexpected TCP frame in RDMA-notify mode\n");
                return -1;
            }
            return 0;
        }
        if (ready > 0 &&
            (poller.revents & (POLLHUP | POLLERR | POLLNVAL))) {
            fprintf(stderr, "FAIL: video control disconnected\n");
            return -1;
        }
        if (stop_requested && *stop_requested)
            return 1;
        uint64_t now = video_ns();
        if (now >= deadline) {
            if (!continuous) {
                fprintf(stderr, "FAIL: RDMA frame notification timeout\n");
                return -1;
            }
            fprintf(stderr, "RX_IDLE waiting_source=1 session_alive=1 mr_retained=1 next_frame=%" PRIu64 "\n",
                    expected);
            reported_idle = 1;
            deadline = now + idle_ns;
        }
        usleep(20);
    }
}
#endif
