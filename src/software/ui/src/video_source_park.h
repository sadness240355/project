#ifndef VIDEO_SOURCE_PARK_H
#define VIDEO_SOURCE_PARK_H
#include <stdio.h>
#include <unistd.h>

/* Per-session marker; only expected source pauses suppress a frame timeout.
 * It never authorizes releasing DMA memory or discarding ownership. */
static inline int video_source_park_requested(const char *ring_path)
{
    char marker[4096];
    int n;
    if (!ring_path || !*ring_path)
        return 0;
    n = snprintf(marker, sizeof(marker), "%s.park", ring_path);
    if (n < 0 || (size_t)n >= sizeof(marker))
        return 0;
    return access(marker, F_OK) == 0;
}
#endif
