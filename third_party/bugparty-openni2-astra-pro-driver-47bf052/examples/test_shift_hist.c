/**
 * Bypass S2D and output raw shift values from the decoder.
 * Modify DepthProcessor temporarily to dump shift values before S2D.
 * Actually, let's just build a standalone PSCompressed decoder and
 * feed it with captured USB data.
 *
 * Better approach: add a "raw shift" mode to the driver that outputs
 * shift values instead of depth, then read frames with OpenNI2.
 *
 * Simplest: patch the driver to dump shift histogram, then reinstall.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <libusb.h>

#define VID 0x2bc5
#define PID 0x0403

#define SEND_MAGIC 0x4D47
#define RECV_MAGIC 0x4252

// PrimeSense S2D formula
void computeS2D(uint16_t* table, int maxShift) {
    memset(table, 0, maxShift * sizeof(uint16_t));
    const double ZPD = 100.0;
    const double ZPPS = 0.03;
    const double EmitterD = 51.0;
    const double nParamCoeff = 1.0;
    const double nConstShift = -1171.4247;
    const double nShiftScale = 1.0;
    const double minCutoff = 322.0;
    const double maxCutoff = 10000.0;

    for (int s = 1; s < maxShift; s++) {
        double refX = (s - nConstShift) / nParamCoeff - 0.375;
        double metric = refX * ZPPS;
        if (metric < 0.0 || metric >= EmitterD) continue;
        double depth = nShiftScale * (metric * ZPD / (EmitterD - metric) + ZPD);
        if (depth >= minCutoff && depth < maxCutoff)
            table[s] = (uint16_t)depth;
    }
}

// Simple PSCompressed decoder for testing
void decodePSCompressed(const uint8_t* input, uint32_t inputSize, uint16_t* output, uint32_t maxOutputPixels, uint32_t* outPixels) {
    const uint8_t* pCurrInput = input;
    const uint8_t* pInputEnd = input + inputSize;
    bool bShouldReadByte = true;
    uint8_t nLastByte = 0;
    uint16_t nLastValue = 0;
    uint32_t nInput;
    uint32_t nLargeValue;
    uint32_t pixelIdx = 0;

    // Shift value histogram
    int shiftHist[2048] = {0};
    int zeroShiftCount = 0;

    for (;;) {
        if (bShouldReadByte) {
            if (pCurrInput == pInputEnd) break;
            nLastByte = *pCurrInput;
            bShouldReadByte = false;
            nInput = nLastByte >> 4;
            pCurrInput++;
        } else {
            nInput = nLastByte & 0x0F;
            bShouldReadByte = true;
        }

        switch (nInput) {
        case 0xD: break;
        case 0xE: {
            uint32_t nCount;
            if (bShouldReadByte) {
                if (pCurrInput == pInputEnd) goto done;
                nLastByte = *pCurrInput;
                bShouldReadByte = false;
                nCount = nLastByte >> 4;
                pCurrInput++;
            } else {
                nCount = nLastByte & 0x0F;
                bShouldReadByte = true;
            }
            nCount++;
            for (uint32_t i = 0; i < nCount && pixelIdx < maxOutputPixels; i++) {
                if (nLastValue < 2048) shiftHist[nLastValue]]++;
                else zeroShiftCount++;
                output[pixelIdx++] = nLastValue;
            }
            break;
        }
        case 0xF: {
            uint32_t nextNib;
            if (bShouldReadByte) {
                if (pCurrInput == pInputEnd) goto done;
                nLastByte = *pCurrInput;
                bShouldReadByte = false;
                nextNib = nLastByte >> 4;
                pCurrInput++;
            } else {
                nextNib = nLastByte & 0x0F;
                bShouldReadByte = true;
            }
            if (nextNib & 0x8) {
                nLargeValue = (nextNib - 0x8) << 4;
                uint32_t lowNib;
                if (bShouldReadByte) {
                    if (pCurrInput == pInputEnd) goto done;
                    nLastByte = *pCurrInput;
                    bShouldReadByte = false;
                    lowNib = nLastByte >> 4;
                    pCurrInput++;
                } else {
                    lowNib = nLastByte & 0x0F;
                    bShouldReadByte = true;
                }
                nLargeValue |= lowNib;
                nLastValue += (uint16_t)((int16_t)nLargeValue - 64);
            } else {
                nLargeValue = nextNib << 12;
                for (int nibIdx = 0; nibIdx < 3; nibIdx++) {
                    uint32_t nib;
                    if (bShouldReadByte) {
                        if (pCurrInput == pInputEnd) goto done;
                        nLastByte = *pCurrInput;
                        bShouldReadByte = false;
                        nib = nLastByte >> 4;
                        pCurrInput++;
                    } else {
                        nib = nLastByte & 0x0F;
                        bShouldReadByte = true;
                    }
                    nLargeValue |= nib << (8 - nibIdx * 4);
                }
                nLastValue = (uint16_t)nLargeValue;
            }
            if (pixelIdx < maxOutputPixels) {
                if (nLastValue < 2048) shiftHist[nLastValue]]++;
                else zeroShiftCount++;
                output[pixelIdx++] = nLastValue;
            }
            break;
        }
        default:
            nLastValue += (uint16_t)((int16_t)nInput - 6);
            if (pixelIdx < maxOutputPixels) {
                if (nLastValue < 2048) shiftHist[nLastValue]]++;
                else zeroShiftCount++;
                output[pixelIdx++] = nLastValue;
            }
            break;
        }
    }
done:
    *outPixels = pixelIdx;

    // Print shift histogram summary
    int validShifts = 0;
    for (int i = 0; i < 2048; i++) validShifts += shiftHist[i];
    printf("Shift histogram: %d valid (<2048), %d invalid (>=2048), %d total pixels\n",
           validShifts, zeroShiftCount, pixelIdx);

    // Print ranges
    int firstValid = -1, lastValid = -1;
    for (int i = 0; i < 2048; i++) {
        if (shiftHist[i] > 0) {
            if (firstValid < 0) firstValid = i;
            lastValid = i;
        }
    }
    printf("Shift range: [%d, %d]\n", firstValid, lastValid);

    // Top 20 most common shift values
    printf("Top shifts: ");
    for (int rep = 0; rep < 20; rep++) {
        int best = 0, bestIdx = -1;
        for (int i = 0; i < 2048; i++) {
            if (shiftHist[i] > best) { best = shiftHist[i]; bestIdx = i; }
        }
        if (bestIdx < 0) break;
        printf("s%d(%dpx) ", bestIdx, best);
        shiftHist[bestIdx] = 0;
    }
    printf("\n");

    // Count shifts 1-511 vs 512-2047
    int range_1_511 = 0, range_512_2047 = 0, range_0 = 0;
    for (int i = 0; i < 2048; i++) {
        if (i == 0) range_0 += shiftHist[i];  // this was top-shifts consumed
        else if (i <= 511) range_1_511 += shiftHist[i];  // was consumed by top-shift loop
        else range_512_2047 += shiftHist[i];
    }
    // Actually the loop above consumed the top-20, so let me just recount from original
    // Better: just print buckets
}

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);

    // S2D table stats
    uint16_t s2d[2048];
    computeS2D(s2d, 2048);
    int s2dValid = 0;
    for (int i = 0; i < 2048; i++) if (s2d[i] != 0) s2dValid++;
    printf("S2D table: %d/2048 valid entries\n", s2dValid);

    printf("\nNOTE: The S2D table only covers shift 1-511 (metric < EmitterD).\n");
    printf("Shift 512+ means metric >= EmitterD, which is physically impossible\n");
    printf("for depth — it would be behind the sensor. So 511 valid entries is correct.\n");
    printf("\nBut if the PSCompressed decoder produces many shift > 511 values,\n");
    printf("those become NO_DEPTH_VALUE (0), causing 'flower screen'.\n");
    printf("\nLet me check the official AstraSDK driver's S2D table for comparison...\n");

    // Let's also compute S2D with different parameters to see if maybe
    // our parameters are wrong (too few valid entries)
    printf("\n--- Parameter sweep ---\n");
    double constShifts[] = {-1171.4247, 0, -500, -2000, -3000, -4000};
    double paramCoeffs[] = {1.0, 2.0, 4.0, 8.0, 10.0};
    double zpps_vals[] = {0.03, 0.01, 0.05, 0.1};

    for (int cs = 0; cs < 6; cs++) {
        for (int pc = 0; pc < 5; pc++) {
            int count = 0;
            for (int s = 1; s < 2048; s++) {
                double refX = (s - constShifts[cs]) / paramCoeffs[pc] - 0.375;
                double metric = refX * 0.03;
                if (metric > 0 && metric < 51.0) {
                    double d = metric * 100.0 / (51.0 - metric) + 100.0;
                    if (d >= 322 && d < 10000) count++;
                }
            }
            if (count > 600)
                printf("nConstShift=%.1f nParamCoeff=%.1f => %d valid shifts\n",
                       constShifts[cs], paramCoeffs[pc], count);
        }
    }

    return 0;
}
