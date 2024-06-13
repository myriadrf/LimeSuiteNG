#include <octave/oct.h>
#include <octave/Cell.h>
#include <octave/ov-struct.h>
#include <vector>
#include <string>

#include "lime/LimeSuite.h"

const int maxChCnt = 2;
const float scaleFactor = 32768.0f;

lms_device_t* lmsDev = NULL;
lms_stream_t streamRx[maxChCnt];
lms_stream_t streamTx[maxChCnt];

struct complex16_t
{
    int16_t i;
    int16_t q;
};

bool IsWFMRunning = false;
complex16_t* rxBuffers = NULL;
complex16_t* txBuffers = NULL;

void StopStream()
{
    if(lmsDev == NULL)
        return;
    for (int i = 0; i < maxChCnt; i++)
    {
        LMS_StopStream(&streamRx[i]);
        LMS_StopStream(&streamTx[i]);
        if (streamRx[i].handle)
            LMS_DestroyStream(lmsDev,&streamRx[i]);
        if (streamTx[i].handle)
            LMS_DestroyStream(lmsDev,&streamTx[i]);
        streamRx[i].handle = 0;
        streamTx[i].handle = 0;
    }
}

void FreeResources()
{
    if(lmsDev)
    {
        if(IsWFMRunning)
            LMS_EnableTxWFM(lmsDev, 0, false);
        StopStream();
        LMS_Close(lmsDev);
        lmsDev = NULL;
    }
    if(rxBuffers)
    {
        delete rxBuffers;
        rxBuffers = NULL;
    }
    if(txBuffers)
    {
        delete txBuffers;
        txBuffers = NULL;
    }
}

void PrintDeviceInfo(lms_device_t* port)
{
    if(port == NULL || lmsDev == NULL)
        return;
    const lms_dev_info_t* info = LMS_GetDeviceInfo(lmsDev);
    if(info == NULL)
        octave_stdout << "Failed to get device info" << std::endl;
    else
    {
        octave_stdout << "Connected to device: " << info->deviceName
                      << " FW: " << info->firmwareVersion << " HW: " << info->hardwareVersion
                      << " Protocol: " << info->protocolVersion
                      << " GW: " << info->gatewareVersion << std::endl;
    }
}

static void LogHandler(int level, const char *msg)
{
    const char* levelText[] = {"CRITICAL: " , "ERROR: ", "WARNING: ", "INFO: ", "DEBUG: "};
    if (level < 0)
        level = 0;
    else if (level >= 4)
        return;   //do not output debug messages
    octave_stdout << levelText[level] << msg << std::endl;
}

DEFUN_DLD (LimeGetDeviceList, args, nargout,
"LIST = LimeGetDeviceList() - Returns available device list")
{
    if (args.length ()!=0)
    {
        print_usage();
        return octave_value(-1);
    }
    lms_info_str_t dev_list[64];
    int devCount = LMS_GetDeviceList((lms_info_str_t*)&dev_list);
    dim_vector dim(devCount, 1);
    Cell c;
    octave_value_list retval;
    //devNames.resize(devCount, 0, string(""));
    for(int i=0; i<devCount; ++i)
    {
        c.insert(octave_value (dev_list[i], '"'),i,0);
        //retval(i) =  octave_value (dev_list[i], '"');
    }
    return octave_value(c);//retval;
}

DEFUN_DLD (LimeInitialize, args, nargout,
"LimeInitialize(DEV) - connect to device and allocate buffer memory\n\
DEV [optional] - device name to connect, obtained via LimeGetDeviceList()")
{
    FreeResources();
    int status;
    int nargin = args.length ();

    if (args.length ()>1)
    {
        print_usage();
        return octave_value(-1);
    }

    if(nargin > 0)
    {
        std::string deviceName = args(0).string_value();
        status = LMS_Open(&lmsDev, deviceName.c_str(), NULL);
    }
    else
        status = LMS_Open(&lmsDev, NULL, NULL);

    LMS_RegisterLogHandler(LogHandler);

    LMS_Synchronize(lmsDev,false);
    if(status != 0)
    {
        return octave_value(status);
    }
    PrintDeviceInfo(lmsDev);

    for (int i = 0; i < maxChCnt; i++)
    {
        streamRx[i].handle = 0;
        streamTx[i].handle = 0;
    }

    return octave_value(0);
}


DEFUN_DLD (LimeDestroy, args, nargout,
"LimeDestroy() - Stop all streams, deallocate memory and disconnect device")
{
    FreeResources();
    return octave_value_list();
}

DEFUN_DLD (LimeLoadConfig, args, nargout,
"LimeLoadConfig(FILENAME) - Load configuration from file FILENAME")
{
    if(lmsDev == NULL)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }

    int nargin = args.length();

    if(nargin != 1)
    {
        print_usage();
        return octave_value(-1);
    }

    std::string filename = args(0).string_value();
    octave_stdout << "LimeLoadConfig loading: " << filename << std::endl;
    if (LMS_LoadConfig(lmsDev, filename.c_str())!=0)
    {
         return octave_value(-1);
    }

    int chCnt = LMS_GetNumChannels(lmsDev, LMS_CH_RX);
    chCnt = chCnt > maxChCnt ? maxChCnt : chCnt;
    for(int ch=0; ch< chCnt; ++ch) //set antenna to update RF switches
    {
        int ant = LMS_GetAntenna(lmsDev, LMS_CH_RX, ch);
        if(ant < 0 || LMS_SetAntenna(lmsDev, LMS_CH_RX, ch, ant) < 0)
             octave_stdout << "Error setting Rx antenna for ch: " << ch << std::endl;
        ant = LMS_GetAntenna(lmsDev, LMS_CH_TX, ch);
        if(ant < 0 || LMS_SetAntenna(lmsDev, LMS_CH_TX, ch, ant) < 0)
             octave_stdout << "Error setting Tx antenna for ch: " << ch << std::endl;
    }

    octave_stdout << "Config loaded successfully: " << std::endl;

    return octave_value(0);
}

DEFUN_DLD (LimeStartStreaming, args, nargout,
"LimeStartStreaming(FIFOSIZE, CHANNELS) - starts sample streaming from selected channels\n\
 FIFOSIZE [optional] - buffer size in samples to be used by library (default: 4 MSamples)\
 CHANNELS [optional] - array of channels to be used [rx0 ; rx1 ; tx0 ; tx1] (default: rx0)")
{
    int nargin = args.length();
    if(lmsDev == NULL)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }
    bool tx[maxChCnt] = {false};
    bool rx[maxChCnt] = {false};
    int fifoSize = 4*1024*1024; //4MS
    int channels = 1;

    if ((nargin > 2))
    {
        print_usage();
        return octave_value(-1);
    }

    if(nargin > 0 && args(0).int_value() > 0)
        fifoSize = args(0).int_value();

    if (nargin == 2)
    {
        int rowCnt = args(1).char_matrix_value().rows();;
        rowCnt = rowCnt < maxChCnt*2 ? rowCnt : maxChCnt*2;
        for (int i = 0 ; i < rowCnt ; i++)
        {
            std::string entry = args(1).char_matrix_value().row_as_string(i);
            if (entry == "rx0" || entry == "rx")
                rx[0] = true;
            else if (entry == "rx1")
                rx[1] = true;
            else if (entry == "tx0" || entry == "tx")
                tx[0] = true;
            else if (entry == "tx1")
                tx[1] = true;
            else
            {
                octave_stdout << "Invalid channel parameter" << std::endl;
                return octave_value(-1);
            }
        }
    }
    else
        rx[0] = true;

    for (int i = 0; i < maxChCnt; i++)
    {
        if (rx[i])
        {
            streamRx[i].channel = i;
            streamRx[i].fifoSize = fifoSize;
            streamRx[i].dataFmt = lms_stream_t::LMS_FMT_I16;
            streamRx[i].isTx = false;
            streamRx[i].throughputVsLatency = 0.5;
            LMS_SetupStream(lmsDev, &streamRx[i]);
        }
        if (tx[i])
        {
            streamTx[i].channel = i;
            streamTx[i].fifoSize = fifoSize;
            streamTx[i].dataFmt = lms_stream_t::LMS_FMT_I16;
            streamTx[i].isTx = true;
            streamTx[i].throughputVsLatency = 0.5;
            LMS_SetupStream(lmsDev, &streamTx[i]);
        }
    }

    for (int i = 0; i < maxChCnt; i++)
    {
        if (rx[i])
        {
            if (!rxBuffers)
                rxBuffers = new complex16_t[fifoSize/2];
            LMS_StartStream(&streamRx[i]);
        }
        if (tx[i])
        {
            if (!txBuffers)
                txBuffers = new complex16_t[fifoSize/2];
            LMS_StartStream(&streamTx[i]);
        }
    }

    return octave_value_list();
}

DEFUN_DLD (LimeStopStreaming, args, nargout,
"LimeStopStreaming() - Stop Receiver and Transmitter threads")
{
    if(lmsDev == NULL)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }
    octave_stdout << "StopStreaming" << std::endl;
    StopStream();
    return octave_value_list();
}

DEFUN_DLD (LimeReceiveSamples, args, ,
"SIGNAL = LimeReceiveSamplesLarge( N, CH) - receive N samples from Rx channel CH.\n\
CH parameter is optional, valid values are 0 and 1")
{
    if (!rxBuffers)
    {
        octave_stdout << "Rx streaming not initialized" << std::endl;
        return octave_value_list();;
    }

    int nargin = args.length();
    if (nargin != 2 && nargin != 1)
    {
        print_usage ();
        return octave_value_list ();
    }
    const int samplesToReceive = args(0).int_value ();

    unsigned chIndex;
    if (nargin == 2)
    {
        chIndex = args(1).int_value();
        if (chIndex >= maxChCnt)
        {
            octave_stdout << "Invalid channel number" << std::endl;
            return octave_value_list();;
        }
    }
    else
    {
        for (chIndex = 0; chIndex < maxChCnt; chIndex++)
            if (streamRx[chIndex].handle != 0)
                break;
    }

    Complex val=Complex(0.0,0.0);
    ComplexRowVector iqData( samplesToReceive, val ); // index 0 to N-1

    const int timeout_ms = 1000;
    lms_stream_meta_t meta;
    int samplesCollected = 0;
    int retries = 5;
    while(samplesCollected < samplesToReceive && retries--)
    {
        int samplesToRead = 0;
        if(samplesToReceive-samplesCollected > streamRx[chIndex].fifoSize/2)
            samplesToRead = streamRx[chIndex].fifoSize/2;
        else
            samplesToRead = samplesToReceive-samplesCollected;
        int samplesRead = LMS_RecvStream(&streamRx[chIndex], (void*)rxBuffers, samplesToRead, &meta, timeout_ms);
        if (samplesRead < 0)
        {
            octave_stdout << "Error reading samples" << std::endl;
            return octave_value(-1);
        }
        for(int i=0; i<samplesRead; ++i)
        {
            iqData(samplesCollected+i)=Complex(rxBuffers[i].i/scaleFactor,rxBuffers[i].q/scaleFactor);
        }
        samplesCollected += samplesRead;
    }
    return octave_value(iqData);
}

DEFUN_DLD (LimeTransmitSamples, args, ,
"LimeTransmitSamples( SIGNAL, CH) - sends normalized complex SIGNAL to Tx channel CH\n\
CH parameter is optional, valid values are 0 and 1")
{
    if (!txBuffers)
    {
        octave_stdout << "Tx streaming not initialized" << std::endl;
        return octave_value(-1);
    }

    int nargin = args.length();
    int check=0;
    if (nargin != 2 && nargin != 1)
    {
        print_usage ();
        return octave_value(-1);
    }

    int chIndex = 0;
    if (nargin == 2)
    {
        chIndex = args(1).int_value ();
        if (chIndex >= maxChCnt)
        {
            octave_stdout << "Invalid channel number" << std::endl;
            return octave_value(-1);
        }
    }
    else
    {
        for (chIndex = 0; chIndex < maxChCnt; chIndex++)
            if (streamTx[chIndex].handle != 0)
                break;
    }

    ComplexRowVector    iqData=args(0).complex_row_vector_value();
    dim_vector          iqDataSize=iqData.dims();
    int samplesCount = iqDataSize(0) > iqDataSize(1) ? iqDataSize(0) : iqDataSize(1);

    for(int i=0; i < samplesCount; ++i)
    {
        octave_value    iqDatum = scaleFactor*iqData(i);
        Complex iqDatum2 = iqDatum.complex_value();
        short i_sample = iqDatum2.real(); //
        short q_sample = iqDatum2.imag(); //
        txBuffers[i].i = i_sample;
        txBuffers[i].q = q_sample;
    }

    const int timeout_ms = 1000;
    lms_stream_meta_t meta;
    meta.waitForTimestamp = false;
    meta.timestamp = 0;
    int samplesWrite = samplesCount;
    samplesWrite = LMS_SendStream(&streamTx[chIndex], (const void*)txBuffers, samplesCount, &meta, timeout_ms);

    return octave_value (samplesWrite);
}


DEFUN_DLD (LimeTransceiveSamples, args, ,
"RXSIGNAL = LimeTransceiveSamples( TXSIGNAL, RXOFFSET, CH) - transmit TXSIGNAL and receive RXSIGNAL (same length as TXSIGNAL).\n\
 RXOFFSET [optional] - number of samples to skip at the beginning of receive (default 0)\n\
 CH [optional] - channel to use for transmit and receive, valid values are 0 and 1 (default 0)")
{
    if (!rxBuffers)
    {
        octave_stdout << "Rx streaming not initialized" << std::endl;
        return octave_value_list();
    }
    if (!txBuffers)
    {
        octave_stdout << "Tx streaming not initialized" << std::endl;
        return octave_value_list();
    }

    int nargin = args.length ();
    if (nargin == 0 || nargin > 3)
    {
        print_usage ();
        return octave_value_list();
    }

    unsigned chIndex = 0;
    if (nargin == 3)
    {
        chIndex = args(2).int_value ();
        if ((chIndex >= maxChCnt) || (streamRx[chIndex].handle == 0) || (streamTx[chIndex].handle == 0))
        {
            octave_stdout << "Invalid channel" << std::endl;
            return octave_value_list();
        }
    }

    //transmit part
    const int timeout_ms = 1000;
    lms_stream_meta_t meta = {0, false, false};
    ComplexRowVector    iqDataTx=args(0).complex_row_vector_value();
    dim_vector          iqDataSize=iqDataTx.dims();
    const int samplesCount = iqDataSize(0) > iqDataSize(1) ? iqDataSize(0) : iqDataSize(1);

    for(int i=0; i < samplesCount; ++i)
    {
        octave_value    iqDatum = scaleFactor*iqDataTx(i);
        Complex iqDatum2 = iqDatum.complex_value();
        short i_sample = iqDatum2.real(); //
        short q_sample = iqDatum2.imag(); //
        txBuffers[i].i = i_sample;
        txBuffers[i].q = q_sample;
    }
    const int samplesWrite = LMS_SendStream(&streamTx[chIndex], (const void*)txBuffers, samplesCount, &meta, timeout_ms);
    if (samplesWrite != samplesCount)
        octave_stdout << "Error transmitting samples: send " << samplesWrite +"/" +samplesCount << std::endl;

    //Receive part
    int offset = nargin > 1 ? args(1).int_value() : 0;
    if (offset < 0)
    {
        octave_stdout << "Invalid RXOFFSET value" << std::endl;
        offset = 0;
    }
    Complex val=Complex(0.0,0.0);
    ComplexRowVector iqDataRx( samplesCount, val ); // index 0 to N-1

    int samplesCollected = 0;
    int retries = 5;
    while(samplesCollected < samplesCount && retries--)
    {
        int samplesToRead = 0;
        if(samplesCount-samplesCollected > streamRx[chIndex].fifoSize/2)
            samplesToRead = streamRx[chIndex].fifoSize/2;
        else
            samplesToRead = samplesCount-samplesCollected;
        int samplesRead = LMS_RecvStream(&streamRx[chIndex], (void*)rxBuffers, samplesToRead, &meta, timeout_ms);
        if (samplesRead < 0)
        {
            octave_stdout << "Error reading samples" << std::endl;
            return octave_value(-1);
        }

        if (offset >= samplesRead)
        {
            offset -= samplesRead;
            continue;
        }

        for(int i=offset; i<samplesRead; ++i)
            iqDataRx(samplesCollected++)=Complex(rxBuffers[i].i/scaleFactor,rxBuffers[i].q/scaleFactor);
        offset = 0;
    }
    return octave_value(iqDataRx);
}

DEFUN_DLD (LimeLoopWFMStart, args, ,
"LimeLoopWFMStart(SIGNAL) - upload SIGNAL to device RAM for repeated transmitting")
{
    if(lmsDev == NULL)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }
    int nargin = args.length ();
    int check=0;
    if (nargin != 2 && nargin != 1)
    {
        print_usage ();
        return octave_value_list ();
    }

    const int chCount = nargin;
    const int timeout_ms = 1000;

    ComplexRowVector    iqData=args(0).complex_row_vector_value();
    dim_vector          iqDataSize=iqData.dims();
    int samplesCount = iqDataSize(0) > iqDataSize(1) ? iqDataSize(0) : iqDataSize(1);
    int wfmLength = samplesCount;

    complex16_t **wfmBuffers = new complex16_t*[chCount];
    for(int i=0; i<chCount; ++i)
        wfmBuffers[i] = new complex16_t[samplesCount];

    for (int ch = 0; ch < chCount; ch++)
        for(int i=0; i < samplesCount; ++i)
        {
            octave_value    iqDatum = 2047*iqData(i);
            Complex iqDatum2 = iqDatum.complex_value();
            float i_sample = iqDatum2.real(); //
            float q_sample = iqDatum2.imag(); //
            wfmBuffers[ch][i].i = i_sample;
            wfmBuffers[ch][i].q = q_sample;
        }

    LMS_UploadWFM(lmsDev, (const void**)wfmBuffers, chCount, samplesCount, 0);
    LMS_EnableTxWFM(lmsDev, 0, true);
    for(int i=0; i<chCount; ++i)
        delete wfmBuffers[i];
    delete wfmBuffers;
    IsWFMRunning = true;
    return octave_value_list();
}

DEFUN_DLD (LimeLoopWFMStop, args, ,
"LimeTxLoopWFMStop() - stop transmitting samples from device RAM")
{
    if(lmsDev == NULL)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }
    if(IsWFMRunning)
        LMS_EnableTxWFM(lmsDev, 0, false);
    IsWFMRunning = false;
    return octave_value ();
}

DEFUN_DLD (LimeGetStreamStatus, args, nargout,
"LimeGetStreamStatus() - Get Stream Status")
{
    if(lmsDev == NULL)
    {
        octave_stdout << "LimeSuite not initialized" << std::endl;
        return octave_value(-1);
    }

    octave_scalar_map st;
    lms_stream_status_t status;
    for (int i = 0; i < maxChCnt; i++)
    {
        if ((streamRx[i].handle) && (LMS_GetStreamStatus(&streamRx[i], &status) == 0))
        {
            st.assign("fifo_size", status.fifoSize);
            st.assign("rx_data_rate", status.linkRate);
            std::string ch = std::string("rx") + char('0'+i);
            st.assign(ch+"_fifo_filled", status.fifoFilledCount);
            st.assign(ch+"_fifo_overruns", status.overrun);
            st.assign(ch+"_lost_packets", status.droppedPackets);
        }
        if ((streamTx[i].handle)&& (LMS_GetStreamStatus(&streamTx[i], &status) == 0))
        {
            st.assign("fifo_size", status.fifoSize);
            st.assign("tx_data_rate", status.linkRate);
            std::string ch = std::string("tx") + char('0'+i);
            st.assign(ch+"_fifo_filled", status.fifoFilledCount);
            st.assign(ch+"_fifo_underrun", status.underrun);
        }
    }

    return octave_value (st);
}

class ResourceDeallocator
{
public:
    ResourceDeallocator() {};
    ~ResourceDeallocator()
    {
        FreeResources();
    };
};

ResourceDeallocator gResources;
