#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdint.h>

#define DMAFD_SOCK  "/tmp/cam_dma.sock"

static volatile int g_running = 1;
static void sig_handler(int s) { (void)s; g_running = 0; }

static int recv_fd(int sock, int *out_idx, size_t *out_size)
{
    char cmsg_buf[CMSG_SPACE(sizeof(int))];
    struct { int index; size_t size; } meta;
    struct iovec iov = { .iov_base=&meta, .iov_len=sizeof(meta) };
    struct msghdr msg = {0};
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);

    if (recvmsg(sock, &msg, 0) < 0) return -1;

    struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
    if (!cmsg || cmsg->cmsg_type != SCM_RIGHTS) return -1;

    int fd;
    memcpy(&fd, CMSG_DATA(cmsg), sizeof(int));
    *out_idx  = meta.index;
    *out_size = meta.size;
    return fd;
}

int main(void)
{
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("DMA-BUF Zero-Copy Consumer\n");
    printf("Connecting to reactive_imu via %s\n\n", DMAFD_SOCK);

    int frame_count = 0;

    while (g_running) {
        /* Connect to DMA-BUF server */
        int sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock < 0) { perror("socket"); sleep(1); continue; }

        struct sockaddr_un addr = {0};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, DMAFD_SOCK, sizeof(addr.sun_path)-1);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock);
            usleep(500000);
            continue;
        }

        /* Receive DMA-BUF fd via SCM_RIGHTS */
        int    buf_index = -1;
        size_t buf_size  = 0;
        int dma_fd = recv_fd(sock, &buf_index, &buf_size);
        close(sock);

        if (dma_fd < 0 || buf_size == 0) {
            usleep(500000);
            continue;
        }

        /* mmap the DMA-BUF fd — zero copy access */
        void *ptr = mmap(NULL, buf_size, PROT_READ, MAP_SHARED, dma_fd, 0);
        if (ptr == MAP_FAILED) {
            perror("mmap DMA-BUF");
            close(dma_fd);
            usleep(500000);
            continue;
        }

        /* Read raw pixel statistics */
        const uint16_t *pixels = (const uint16_t *)ptr;
        int width = 3280;
        uint32_t sum_r=0, sum_g=0, sum_b=0, count=0;
        for (int x = 0; x < 200 && x+1 < width; x += 2) {
            sum_r += pixels[x]         >> 2;
            sum_g += ((pixels[x+1] >> 2) + (pixels[width+x] >> 2)) / 2;
            sum_b += pixels[width+x+1] >> 2;
            count++;
        }

        uint16_t mn=0xFFFF, mx=0;
        size_t total = buf_size / sizeof(uint16_t);
        size_t step  = total / 10000;
        if (step < 1) step = 1;
        for (size_t i = 0; i < total; i += step) {
            uint16_t v = pixels[i] >> 2;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }

        frame_count++;
        printf("[CONSUMER] frame=%4d  buf_idx=%d  local_fd=%d  "
               "size=%zu bytes\n",
               frame_count, buf_index, dma_fd, buf_size);
        printf("           RAW10: R=%3u G=%3u B=%3u  "
               "min=%3u max=%3u range=%3u\n",
               count ? sum_r/count : 0,
               count ? sum_g/count : 0,
               count ? sum_b/count : 0,
               mn, mx, mx-mn);
        printf("           [zero-copy via SCM_RIGHTS DMA-BUF transfer]\n\n");

        munmap(ptr, buf_size);
        close(dma_fd);

        usleep(500000);
    }

    printf("Consumer stopped. %d frames accessed zero-copy.\n", frame_count);
    return 0;
}
