/**
 * Read FixedParams from device to get actual S2D calibration values.
 * FixedParams contains: ZPD, ZPPS, EmitterD, nConstShift, nParamCoeff, etc.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <libusb.h>

#define VID 0x2bc5
#define PID 0x0403

#define SEND_MAGIC 0x4D47
#define RECV_MAGIC 0x4252

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);

    int rc = libusb_init(NULL);
    if (rc < 0) { fprintf(stderr, "libusb init failed\n"); return 1; }

    libusb_device_handle* dev = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!dev) { fprintf(stderr, "Device not found\n"); libusb_exit(NULL); return 1; }

    rc = libusb_claim_interface(dev, 0);
    if (rc < 0) { fprintf(stderr, "Claim failed: %d\n", rc); return 1; }

    // Read flash from address 0x10000 (camera params region)
    // Try multiple regions to find S2D params
    uint32_t addrs[] = {0x10000, 0x10040, 0x20000, 0x30000, 0x40000};
    int sizes[] = {256, 256, 256, 256, 256};

    for (int i = 0; i < 5; i++) {
        // Build SendCmd readFlash: cmdId=0x0019
        // SendCmd packet: [0x4D47][sizeHW][cmdId][seq][data...]
        // data: [addr(4)][size(2)]
        uint8_t sendBuf[64];
        uint16_t* hw = (uint16_t*)sendBuf;
        hw[0] = SEND_MAGIC;
        hw[1] = 6;  // 3 more halfwords = 6 bytes
        hw[2] = 0x0019;  // SC_READ_FLASH
        hw[3] = 1;  // seq

        uint32_t addr = addrs[i];
        uint16_t rsize = sizes[i];
        memcpy(&sendBuf[8], &addr, 4);
        memcpy(&sendBuf[12], &rsize, 2);

        int totalSend = 14;

        rc = libusb_control_transfer(dev, 0x41, 0, 0, 0, sendBuf, totalSend, 3000);
        if (rc < 0) {
            fprintf(stderr, "Send failed for addr 0x%x: %d\n", addr, rc);
            continue;
        }

        // Read response
        uint8_t recvBuf[512];
        rc = libusb_control_transfer(dev, 0xC1, 0, 0, 0, recvBuf, sizeof(recvBuf), 3000);
        if (rc < 10) {
            fprintf(stderr, "Recv failed for addr 0x%x: %d\n", addr, rc);
            continue;
        }

        uint16_t* rhw = (uint16_t*)recvBuf;
        if (rhw[0] != RECV_MAGIC) {
            fprintf(stderr, "Bad magic for addr 0x%x\n", addr);
            continue;
        }

        uint16_t errCode = rhw[4];
        int dataLen = rc - 10;  // subtract header

        printf("\nFlash addr=0x%05x size=%d: error=%u dataLen=%d\n", addr, rsize, errCode, dataLen);

        if (errCode != 0) {
            printf("  Error response\n");
            continue;
        }

        // Print raw data
        uint8_t* data = recvBuf + 10;
        printf("  Hex:");
        for (int j = 0; j < dataLen && j < 64; j++) {
            if (j % 16 == 0) printf("\n  %04x:", j);
            printf(" %02x", data[j]);
        }
        printf("\n");

        // Try to interpret as floats (S2D params)
        if (dataLen >= 32) {
            float* fdata = (float*)data;
            printf("  As floats:");
            for (int j = 0; j < 8 && j*4 < dataLen; j++) {
                printf(" [%.6g]", fdata[j]);
            }
            printf("\n");
        }
    }

    // Also try getFixedParams (XnHostProtocol cmd 4)
    {
        uint8_t sendBuf[16];
        uint16_t* hw = (uint16_t*)sendBuf;
        hw[0] = SEND_MAGIC;
        hw[1] = 0;  // no extra data
        hw[2] = 0x0004;  // XN_GET_FIXED_PARAMS
        hw[3] = 2;

        int rc = libusb_control_transfer(dev, 0x41, 0, 0, 0, sendBuf, 8, 3000);
        printf("\ngetFixedParams send: rc=%d\n", rc);

        if (rc >= 0) {
            uint8_t recvBuf[1024];
            rc = libusb_control_transfer(dev, 0xC1, 0, 0, 0, recvBuf, sizeof(recvBuf), 3000);
            printf("getFixedParams recv: rc=%d\n", rc);
            if (rc >= 10) {
                uint16_t* rhw = (uint16_t*)recvBuf;
                printf("  magic=0x%04x err=%u dataLen=%d\n", rhw[0], rhw[4], rc-10);
                if (rhw[0] == RECV_MAGIC && rhw[4] == 0) {
                    uint8_t* data = recvBuf + 10;
                    int dataLen = rc - 10;
                    printf("  Hex:");
                    for (int j = 0; j < dataLen && j < 128; j++) {
                        if (j % 16 == 0) printf("\n  %04x:", j);
                        printf(" %02x", data[j]);
                    }
                    printf("\n  As floats:");
                    float* fdata = (float*)data;
                    for (int j = 0; j < 32 && j*4 < dataLen; j++) {
                        printf(" [%.6g]", fdata[j]);
                    }
                    printf("\n");
                }
            }
        }
    }

    libusb_release_interface(dev, 0);
    libusb_close(dev);
    libusb_exit(NULL);
    return 0;
}
