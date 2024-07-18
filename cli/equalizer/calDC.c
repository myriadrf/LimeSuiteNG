/* ************************************************************************
   FILE:	equAPI.c
   COMMENT:	Test bench for FDQI equiliser
   CONTENT:
   AUTHOR:	Lime Microsystems
   DATE:	May 18, 2020
   REVISION:	
   ************************************************************************ */

/* Include section */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#include "FDQIdesign.h"

void Terminate50Ohm(lms_device_t *pLmsControl, bool terminate);
void SetupTXRXPLL(lms_device_t *lmsControl, bool single);

int main(int argc, char *argv[])
{
    lms_device_t *device = NULL;
    lms_device_t *dummy_dev = NULL;
    lms_info_str_t list[8];
    lms_stream_t dummy_stream;
    int n = 0;
    if ((n = LMS_GetDeviceList(list)) < 0)
        ;
    if (n < 1)
        return -1;

    int16_t code_I, code_Q, code_Alpha; // temporary data
    int32_t numSamples = 32 * 1024; // FFT points
    double fs = 122.88;
    double refClk_MHz = 2.0 * fs; // sampling frequancy [MHz]
    double LOfreq = 2300; //3489.42; // LO frequency, 2300; //
    double freq; // dummy

    FILE *fid = NULL;
    int chA = 1;
    int loopback = 1;
    int lms_index = 0; // LMS chip index
    int slaveDevice = 0;

    if (argc > 1)
    {
        lms_index = atoi(argv[1]);
        if (lms_index < 0)
            exit(-1);
    }
    else
    {
        printf("[Warn] The argument is required.\n");
        exit(-1);
    }

    std::string str1;
    std::string str2;
    std::string str3; //new line

    str1 = list[lms_index];
    str2 = "LitePCIe0";
    str3 = "LitePCIe1"; //new line
    if (str1.compare(0, 9, str2) != 0 && str1.compare(0, 9, str3) != 0) //modified line
    {
        printf("[Warn] It is not LitePCIe0 board.\n");
        exit(-1);
    }

    if (LMS_Open(&device, list[lms_index], NULL))
    {
        printf("[Warn] Device cannot be opened.\n");
        printf("[Warn] Try to start application with sudo.\n");
        exit(-1);
    }

    if (device != NULL)
    {
        printf("[Info] Device is opened: %s.\n", list[lms_index]);
    }

    if (str1.compare(0, 9, str3) == 0)
    {
        slaveDevice = 1;
        printf("[Info] Opening master device\n");
        int master_index = -1;
        for (int i = 0; i < sizeof(list) / sizeof(list[0]); i++)
        {
            str1 = list[i];
            if (str1.compare(0, 9, str2) == 0)
            {
                master_index = i;
                break;
            }
        }

        if (!master_index)
        {
            printf("[Warn] Master device could not be found.\n");
            exit(-1);
        }

        if (LMS_Open(&dummy_dev, list[master_index], NULL) != 0)
        {
            printf("[Warn] Master device cannot be opened.\n");
            exit(-1);
        }

        printf("[Info] Device is opened: %s.\n", list[master_index]);

        //stream structure
        dummy_stream.channel = 4; //channel number
        dummy_stream.fifoSize = 1024 * 1024; //fifo size in samples
        dummy_stream.throughputVsLatency = 1.0; //optimize for max throughput
        dummy_stream.isTx = false; //RX channel
        dummy_stream.dataFmt = lms_stream_t::LMS_FMT_I16; //16-bit integers
        if (LMS_SetupStream(dummy_dev, &dummy_stream) != 0)
        {
            printf("[Warn] Failed to setup stream.\n");
            exit(-1);
        }
        LMS_StartStream(&dummy_stream);
        LMS_WriteFPGAReg(dummy_dev, 0xFFFF, 1 << 2); // ????
        LMS_WriteFPGAReg(dummy_dev, 0xA, 1); // ????
        printf("[Info] Stream has started\n");
    }

    //FDQIE_CalibrateDC(device,numSamples);

    if (LMS_GetLOFrequency(device, LMS_CH_TX, 0, &freq) != 0)
        error(device);
    LOfreq = freq / 1e6;
    printf("[INFO] Center frequency: %+5.1f MHz\n", LOfreq);

    int TcodeI, TcodeQ = 0;
    bool samePLL = true;

    Terminate50Ohm(device, 1);
    SetupTXRXPLL(device, !samePLL); // first, make different LOs

    //  the API function behaves strange, please check
    lime::LMS7_Device *lms_dev = (lime::LMS7_Device *)device;
    lime::LMS7002M *lms = lms_dev->GetLMS(0);
    if (lms->SetFrequencySX(LMS_CH_RX, LOfreq * 1e6) != 0)
        error(device);

    FDQIE_CalibrateTransmitterDC(device, numSamples, LOfreq, refClk_MHz, 0.9975, &TcodeI, &TcodeQ);
    //FDQIE_MeasurementTransmitterDC (device, numSamples, LOfreq, refClk_MHz, 0.9975);

    SetupTXRXPLL(device, samePLL); // make same LOs
    if (lms->SetFrequencySX(LMS_CH_TX, LOfreq * 1e6) != 0)
        error(device);

    Terminate50Ohm(device, 0);
    LMS_Close(device);

    if (dummy_dev != NULL)
    {
        LMS_StopStream(&dummy_stream);
        LMS_DestroyStream(dummy_dev, &dummy_stream);
        LMS_Close(dummy_dev);
    }
}
