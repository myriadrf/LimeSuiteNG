#include "fftviewer_frFFTviewer.h"
#include <wx/timer.h>
#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <vector>
#include "limesuiteng/Logger.h"
#include "OpenGLGraph.h"
#include "limesuiteng/LMS7002M.h"
#include "DSP/FFT/FFT.h"
#include <fstream>
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamMeta.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/complex.h"
#include <array>

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

void fftviewer_frFFTviewer::Update()
{
}

bool fftviewer_frFFTviewer::Initialize(SDRDevice* pDataPort)
{
    StopStreaming();
    device = pDataPort;

    if (!device)
    {
        btnStartStop->Disable();
        return true;
    }
    btnStartStop->Enable();

    lmsIndex = 0;
    cmbRFSOC->Clear();
    const SDRDescriptor& desc = device->GetDescriptor();

    if (desc.rfSOC.size() == 0)
        return false;

    for (size_t i = 0; i < desc.rfSOC.size(); ++i)
    {
        cmbRFSOC->Append(desc.rfSOC[i].name);
    }

    uint8_t channelCount = desc.rfSOC.at(0).channelCount;
    if (channelCount <= 1)
    {
        cmbMode->Clear();
        cmbMode->Append("SISO Ch.A");
        cmbMode->SetSelection(0);

        cmbChannelVisibility->Clear();
        cmbChannelVisibility->Append("A");
        cmbChannelVisibility->SetSelection(0);
    }
    else
    {
        std::vector<wxString> modeChoices;
        for (int i = 0; i < 16; ++i)
            modeChoices.push_back(wxString::Format("%04X", i));

        cmbMode->Set(modeChoices.size(), modeChoices.data());
        cmbMode->SetSelection(1);
        cmbMode->GetContainingSizer()->Layout(); // update the width of the box

        cmbChannelVisibility->Set(modeChoices.size(), modeChoices.data());
        cmbChannelVisibility->SetSelection(0xF);
        cmbChannelVisibility->GetContainingSizer()->Layout(); // update the width of the box
    }

    cmbRFSOC->SetSelection(0);
    SetNyquistFrequency();

    sbStreamConfig->Layout();

    wxCommandEvent evt;
    //show only A channel at startup
    evt.SetInt(0x1);
    OnChannelVisibilityChange(evt);
    return true;
}

fftviewer_frFFTviewer::fftviewer_frFFTviewer(wxWindow* parent, wxWindowID id)
    : frFFTviewer(parent, id)
    , mStreamRunning(false)
    , device(nullptr)
    , mGUIupdater(new wxTimer(this, wxID_ANY))
    , sampleRate(0)
    , mChannelsMask(1)
{
    captureSamples.store(false);
    averageCount.store(50);
    spinAvgCount->SetValue(averageCount);
    updateGUI.store(true);
    windowFunctionID.store(false);
    enableFFT.store(false);
#ifndef __unix__
    SetIcon(wxIcon(_("aaaaAPPicon")));
#endif
    SetSize(1000, 700);
    mFFTpanel->settings.useVBO = true;

    for (int i = 0; i < cMaxChCount; ++i)
    {
        mTimeDomainPanel->AddSerie(new cDataSerie());
        mTimeDomainPanel->AddSerie(new cDataSerie());
        mConstellationPanel->AddSerie(new cDataSerie());
        mFFTpanel->AddSerie(new cDataSerie());
    }

    mFFTpanel->series[0]->color = 0xFF0000FF;
    mFFTpanel->series[1]->color = 0x0000FFFF;
    mFFTpanel->series[2]->color = 0x000000FF;
    mFFTpanel->series[3]->color = 0x00FFFFFF;
    mFFTpanel->SetDrawingMode(GLG_LINE);
    mFFTpanel->settings.gridXlines = 15;
    mFFTpanel->SetInitialDisplayArea(-16000000, 16000000, -115, 0);

    mFFTpanel->settings.title = "FFT"s;
    mFFTpanel->settings.titleXaxis = "Frequency(MHz)"s;
    mFFTpanel->settings.titleYaxis = "Amplitude(dBFS)"s;
    mFFTpanel->settings.xUnits = ""s;
    mFFTpanel->settings.gridXprec = 3;
    //mFFTpanel->settings.yUnits = "dB"s;
    mFFTpanel->settings.markersEnabled = true;

    mFFTpanel->settings.marginLeft = 40;
    mFFTpanel->settings.staticGrid = true;

    mTimeDomainPanel->settings.useVBO = true;

    mTimeDomainPanel->SetInitialDisplayArea(0, 1024, -1, 1);
    mTimeDomainPanel->settings.title = "IQ samples"s;
    mTimeDomainPanel->series[0]->color = 0x000000FF;
    mTimeDomainPanel->series[1]->color = 0x0000FFFF;
    mTimeDomainPanel->series[2]->color = 0x00FF00FF;
    mTimeDomainPanel->series[3]->color = 0x00FFFFFF;
    mTimeDomainPanel->series[4]->color = 0xFF0000FF;
    mTimeDomainPanel->series[5]->color = 0xFF00FFFF;
    mTimeDomainPanel->series[6]->color = 0xFF0FF0FF;
    mTimeDomainPanel->series[7]->color = 0xF0F0F0FF;
    mTimeDomainPanel->settings.marginLeft = 48;
    mConstellationPanel->settings.useVBO = true;
    mConstellationPanel->series[0]->color = 0xFF0000FF;
    mConstellationPanel->series[1]->color = 0x0000FFFF;
    mConstellationPanel->series[2]->color = 0x00FF00FF;
    mConstellationPanel->series[3]->color = 0x00FFFFFF;
    mConstellationPanel->SetInitialDisplayArea(-1, 1, -1, 1);
    mConstellationPanel->SetDrawingMode(GLG_POINTS);
    mConstellationPanel->settings.title = "I versus Q"s;
    mConstellationPanel->settings.titleXaxis = "I"s;
    mConstellationPanel->settings.titleYaxis = "Q"s;
    mConstellationPanel->settings.gridXlines = 8;
    mConstellationPanel->settings.gridYlines = 8;
    mConstellationPanel->settings.marginLeft = 48;

    Connect(wxEVT_THREAD, wxThreadEventHandler(fftviewer_frFFTviewer::OnUpdatePlots), nullptr, this);
    Connect(wxEVT_TIMER, wxTimerEventHandler(fftviewer_frFFTviewer::OnUpdateStats), nullptr, this);

    wxCommandEvent evt;
    //show only A channel at startup
    evt.SetInt(1);
    OnChannelVisibilityChange(evt);
}

fftviewer_frFFTviewer::~fftviewer_frFFTviewer()
{
    Disconnect(wxEVT_THREAD, wxThreadEventHandler(fftviewer_frFFTviewer::OnUpdatePlots), nullptr, this);
    Disconnect(wxEVT_TIMER, wxTimerEventHandler(fftviewer_frFFTviewer::OnUpdateStats), nullptr, this);

    if (mStreamRunning == true)
    {
        StopStreaming();
    }

    if (mGUIupdater != nullptr)
    {
        delete mGUIupdater;
    }
}

bool fftviewer_frFFTviewer::Show(bool show)
{
    if (!show && mStreamRunning)
    {
        StopStreaming();
    }

    return frFFTviewer::Show(show);
}

void fftviewer_frFFTviewer::OnWindowFunctionChanged(wxCommandEvent& event)
{
    windowFunctionID.store(cmbWindowFunc->GetSelection());
}

void fftviewer_frFFTviewer::OnbtnStartStop(wxCommandEvent& event)
{
    if (threadProcessing.joinable() == false)
    {
        SetNyquistFrequency();
        StartStreaming();
    }
    else
        StopStreaming();
}

void fftviewer_frFFTviewer::StartStreaming()
{
    if (!device)
    {
        wxMessageBox(_("FFTviewer: Connection not initialized"), _("ERROR"));
        return;
    }
    txtNyquistFreqMHz->Disable();
    spinFFTsize->Disable();
    sbStreamConfig->GetStaticBox()->Disable();

    stopProcessing.store(false);
    updateGUI.store(true);

    const int fftSize = spinFFTsize->GetValue();
    fftFreqAxis.resize(fftSize);
    double nyquistMHz;
    txtNyquistFreqMHz->GetValue().ToDouble(&nyquistMHz);
    const float step = 2 * nyquistMHz / fftSize;
    for (int i = 0; i < fftSize; ++i)
        fftFreqAxis[i] = 1e6 * (-nyquistMHz + (i + 1) * step);
    timeXAxis.resize(fftSize);
    for (int i = 0; i < fftSize; ++i)
        timeXAxis[i] = i;

    if (chkCaptureToFile->GetValue() == true)
    {
        captureSamples.store(true);
        wxFileDialog dlg(this, _("Save samples file"), "", "", "Text (*.txt)|*.txt", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
        if (dlg.ShowModal() == wxID_CANCEL)
            captureSamples.store(false);
        else
            captureFilename = dlg.GetPath().ToStdString();
    }
    else
        captureSamples.store(false);
    chkCaptureToFile->Disable();
    spinCaptureCount->Disable();
    lmsIndex = cmbRFSOC->GetSelection();
    if (lmsIndex < 0)
        return;
    if (mStreamRunning.load() == true)
        return;

    lastPlotsUpdateTime = std::chrono::steady_clock::now();

    for (int i = 0; i < cMaxChCount; ++i)
    {
        mTimeDomainPanel->series[2 * i]->AssignValues(nullptr, nullptr, 0);
        mTimeDomainPanel->series[2 * i + 1]->AssignValues(nullptr, nullptr, 0);
    }
    for (int i = 0; i < cMaxChCount; ++i)
        mConstellationPanel->series[i]->AssignValues(nullptr, nullptr, 0);
    for (int ch = 0; ch < cMaxChCount; ++ch)
    {
        mFFTpanel->series[ch]->AssignValues(nullptr, nullptr, 0);
    }

    mChannelsMask = cmbMode->GetSelection();
    channelRelativeToAbsoluteIndex.clear();

    for (uint8_t i = 0; i < 32; ++i)
    {
        if (mChannelsMask & (1 << i))
            channelRelativeToAbsoluteIndex.push_back(i);
    }
    threadProcessing = std::thread(StreamingLoop, this, fftSize, cmbMode->GetSelection(), 0);

    btnStartStop->SetLabel(_("STOP"));
    mGUIupdater->Start(500);
}

void fftviewer_frFFTviewer::StopStreaming()
{
    sbStreamConfig->GetStaticBox()->Enable();
    txtNyquistFreqMHz->Enable();
    mGUIupdater->Stop();
    if (mStreamRunning.load() == false)
        return;
    stopProcessing.store(true);
    threadProcessing.join();
    btnStartStop->SetLabel(_("START"));
    spinFFTsize->Enable();
    chkCaptureToFile->Enable();
    spinCaptureCount->Enable();
    cmbChannelVisibility->Enable();
}

void fftviewer_frFFTviewer::OnUpdateStats(wxTimerEvent& event)
{
    if (mStreamRunning.load() == false)
        return;

    StreamStats rxStats;
    StreamStats txStats;
    if (!stream)
        return;

    stream->StreamStatus(&rxStats, &txStats);

    float RxFilled = 100.0 * rxStats.FIFO.ratio();
    gaugeRxBuffer->SetValue(static_cast<int>(RxFilled));
    lblRxDataRate->SetLabel(printDataRate(rxStats.dataRate_Bps));

    float TxFilled = 100.0 * txStats.FIFO.ratio();
    gaugeTxBuffer->SetValue(static_cast<int>(TxFilled));
    lblTxDataRate->SetLabel(printDataRate(txStats.dataRate_Bps));
}

void fftviewer_frFFTviewer::OnUpdatePlots(wxThreadEvent& event)
{
    double dbOffset = cmbFmt->GetSelection() == 1 ? 93.319 : 69.2369;
    if (mStreamRunning.load() == false)
        return;

    const uint32_t fftSize = streamData.fftBins[0].size();

    if (chkEnPwr->GetValue())
    {
        float chPwr[2] = { 0, 0 };
        double cFreq[2] = { 0, 0 };
        txtCenterOffset1->GetValue().ToDouble(&cFreq[0]);
        txtCenterOffset2->GetValue().ToDouble(&cFreq[1]);
        double bw[2] = { 1, 1 };
        txtBW1->GetValue().ToDouble(&bw[0]);
        txtBW2->GetValue().ToDouble(&bw[1]);

        for (int c = 0; c < 2; ++c)
        {
            float f0 = (cFreq[c] - bw[c] / 2) * 1e6;
            float fn = (cFreq[c] + bw[c] / 2) * 1e6;
            float sum = 0;
            const int lmsch = mFFTpanel->series[0]->visible ? 0 : 1;
            for (uint32_t i = 0; i < fftSize; ++i)
                if (f0 <= fftFreqAxis[i] && fftFreqAxis[i] <= fn)
                {
                    sum += streamData.fftBins[lmsch][i];
                }
            chPwr[c] = sum;
        }

        float pwr1 = (chPwr[0] != 0 ? (10 * log10(chPwr[0])) - dbOffset : -300);
        lblPower1->SetLabel(wxString::Format("%.3f", pwr1));
        float pwr2 = (chPwr[1] != 0 ? (10 * log10(chPwr[1])) - dbOffset : -300);
        lblPower2->SetLabel(wxString::Format("%.3f", pwr2));
        lbldBc->SetLabel(wxString::Format("%.3f", pwr2 - pwr1));
    }

    if (fftSize > 0)
    {
        uint32_t channelCount = channelRelativeToAbsoluteIndex.size();
        if (chkFreezeTimeDomain->IsChecked() == false)
        {
            for (uint32_t i = 0; i < channelCount; ++i)
            {
                uint32_t destinationChannel = channelRelativeToAbsoluteIndex[i];
                mTimeDomainPanel->series[2 * destinationChannel]->AssignValues(
                    &timeXAxis[0], streamData.samplesI[i].data(), streamData.samplesI[i].size());
                mTimeDomainPanel->series[2 * destinationChannel + 1]->AssignValues(
                    &timeXAxis[0], streamData.samplesQ[i].data(), streamData.samplesQ[i].size());
            }
        }
        if (chkFreezeConstellation->IsChecked() == false)
        {
            for (uint32_t i = 0; i < channelCount; ++i)
            {
                uint32_t destinationChannel = channelRelativeToAbsoluteIndex[i];
                mConstellationPanel->series[destinationChannel]->AssignValues(
                    streamData.samplesI[i].data(), streamData.samplesQ[i].data(), streamData.samplesQ[i].size());
            }
        }
        if (chkFreezeFFT->IsChecked() == false)
        {
            std::unique_lock lk{ streamData.mutex };
            for (uint32_t ch = 0; ch < channelCount; ++ch)
            {
                uint32_t destinationChannel = channelRelativeToAbsoluteIndex[ch];
                for (uint32_t s = 0; s < fftSize; ++s)
                {
                    streamData.fftBins[ch][s] = (streamData.fftBins[ch][s] != 0 ? (10 * log10(streamData.fftBins[ch][s])) : -300);
                }
                mFFTpanel->series[destinationChannel]->AssignValues(&fftFreqAxis[0], &streamData.fftBins[ch][0], fftSize);
            }
        }
    }

    if (chkFreezeTimeDomain->IsChecked() == false)
    {
        mTimeDomainPanel->Refresh();
        mTimeDomainPanel->Draw();
    }

    if (chkFreezeConstellation->IsChecked() == false)
    {
        mConstellationPanel->Refresh();
        mConstellationPanel->Draw();
    }

    if (chkFreezeFFT->IsChecked() == false)
    {
        mFFTpanel->Refresh();
        mFFTpanel->Draw();
        enableFFT.store(true);
    }
    else
        enableFFT.store(false);

    lastPlotsUpdateTime = std::chrono::steady_clock::now();
    updateGUI.store(true);
}

void fftviewer_frFFTviewer::StreamingLoop(
    fftviewer_frFFTviewer* pthis, const unsigned int fftSize, const uint8_t channelEnablesMask, const uint32_t format)
{
    const bool runTx = pthis->chkEnTx->GetValue();

    StreamConfig config;
    uint8_t channelsCount = 0;

    for (uint8_t i = 0; i < 8; ++i)
    {
        if (channelEnablesMask & (1 << i))
        {
            config.channels.at(TRXDir::Rx).push_back(i);
            if (runTx)
                config.channels.at(TRXDir::Tx).push_back(i);
            ++channelsCount;
        }
    }

    if (channelsCount == 0)
        return;

    int avgCount = pthis->spinAvgCount->GetValue();
    auto wndFunction = static_cast<lime::FFT::WindowFunctionType>(pthis->windowFunctionID.load());
    bool fftEnabled = true;
    pthis->enableFFT.store(true);

    bool syncTx = pthis->chkEnSync->GetValue();

    DataToGUI localDataResults;
    localDataResults.nyquist_Hz = 7.68e6;
    for (uint8_t i = 0; i < cMaxChCount; i++)
    {
        localDataResults.samplesI[i].resize(fftSize, 0);
        localDataResults.samplesQ[i].resize(fftSize, 0);
        localDataResults.fftBins[i].resize(fftSize, 0);
        pthis->streamData.samplesI[i].resize(fftSize);
        pthis->streamData.samplesQ[i].resize(fftSize);
        pthis->streamData.fftBins[i].resize(fftSize);
    }

    std::vector<complex32f_t> captureBuffer[cMaxChCount];
    uint32_t samplesToCapture = 0;
    uint32_t samplesCaptured = 0;
    if (pthis->captureSamples.load() == true)
        for (uint8_t ch = 0; ch < channelsCount; ++ch)
        {
            samplesToCapture = pthis->spinCaptureCount->GetValue();
            captureBuffer[ch].resize(samplesToCapture);
        }

    auto fmt = pthis->cmbFmt->GetSelection() == 1 ? DataFormat::I16 : DataFormat::I12;

    config.format = DataFormat::F32;
    config.linkFormat = fmt;
    const uint8_t chipIndex = pthis->lmsIndex;

    pthis->stream = pthis->device->StreamCreate(config, chipIndex);
    pthis->stream->Start();

    pthis->mStreamRunning.store(true);
    StreamTxMeta txMeta{};
    txMeta.hasTimestamp = syncTx;
    txMeta.flags = StreamTxMeta::EndOfBurst;
    int fftCounter = 0;

    StreamRxMeta rxMeta{};

    lime::complex32f_t** buffers = new lime::complex32f_t*[channelsCount];
    FFT fft{ channelsCount, fftSize, wndFunction };

    fft.SetAverageCount(avgCount);
    fft.SetResultsCallback(
        [](const std::vector<std::vector<float>>& bins, void* userData) {
            auto* pthis = reinterpret_cast<DataToGUI*>(userData);
            std::unique_lock lk{ pthis->mutex };

            for (std::size_t ch = 0; ch < bins.size(); ++ch)
            {
                pthis->fftBins[ch] = bins[ch];
            }

            pthis->isDataReady.store(true);
            pthis->cv.notify_all();
        },
        &localDataResults);

    for (uint8_t i = 0; i < channelsCount; ++i)
    {
        buffers[i] = new complex32f_t[fftSize];
    }

    while (pthis->stopProcessing.load() == false)
    {
        uint32_t samplesPopped;
        samplesPopped = pthis->stream->Receive(buffers, fftSize, &rxMeta);
        if (samplesPopped <= 0)
            continue;

        if (pthis->chkSwapIQ->IsChecked())
        {
            for (uint32_t c = 0; c < channelsCount; ++c)
                for (uint32_t i = 0; i < samplesPopped; ++i)
                {
                    complex32f_t temp(buffers[c][i].imag(), buffers[c][i].real());
                    buffers[c][i] = temp;
                }
        }
        if (pthis->chkRealFFT->IsChecked())
        {
            for (uint32_t c = 0; c < channelsCount; ++c)
                for (uint32_t i = 0; i < samplesPopped; ++i)
                    buffers[c][i].imag(0);
        }

        int64_t rxTS = rxMeta.timestamp.GetTicks();

        if (runTx)
        {
            txMeta.timestamp = Timespec(0, rxTS + 1020 * 128, pthis->sampleRate);
            pthis->stream->Transmit(buffers, fftSize, &txMeta);
        }

        if (pthis->captureSamples.load())
        {
            for (int ch = 0; ch < channelsCount; ++ch)
            {
                uint32_t samplesToCopy = min(samplesPopped, samplesToCapture);
                if (samplesToCopy <= 0)
                    break;
                for (uint32_t i = 0; i < samplesToCopy; ++i)
                {
                    captureBuffer[ch][samplesCaptured + i].real(buffers[ch][i].real() * 32767);
                    captureBuffer[ch][samplesCaptured + i].imag(buffers[ch][i].imag() * 32767);
                }
                samplesToCapture -= samplesToCopy;
                samplesCaptured += samplesToCopy;
            }
        }

        for (int ch = 0; ch < channelsCount; ++ch)
        {
            //take only first buffer for time domain display
            //reset fftBins for accumulation
            if (fftCounter == 0)
                for (unsigned i = 0; i < fftSize; ++i)
                {
                    if (fftEnabled)
                        localDataResults.fftBins[ch][i] = 0;
                    localDataResults.samplesI[ch][i] = buffers[ch][i].real();
                    localDataResults.samplesQ[ch][i] = buffers[ch][i].imag();
                }
            if (fftEnabled)
            {
                int samplesRemaining = samplesPopped;
                while (samplesRemaining > 0 && !pthis->stopProcessing.load())
                {
                    samplesRemaining -= fft.PushSamples(buffers, samplesRemaining, samplesPopped - samplesRemaining);
                }
            }
        }
        ++fftCounter;
        fftEnabled = pthis->enableFFT.load();

        if (pthis->updateGUI.load() == true && (!fftEnabled || localDataResults.isDataReady.load()))
        {
            auto timeSinceLastPlot = std::chrono::steady_clock::now() - pthis->lastPlotsUpdateTime;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceLastPlot) < std::chrono::milliseconds(16))
            {
                continue;
            }

            if (pthis->stopProcessing.load() == false)
            {
                std::scoped_lock lk{ pthis->streamData.mutex, localDataResults.mutex };

                pthis->streamData = localDataResults;
                wxThreadEvent* evt = new wxThreadEvent();
                evt->SetEventObject(pthis);
                pthis->updateGUI.store(false);
                pthis->QueueEvent(evt);
            }
            fftCounter = 0;
            avgCount = pthis->averageCount.load();
            auto wndFunctionSelection = static_cast<lime::FFT::WindowFunctionType>(pthis->windowFunctionID.load());
            for (uint8_t ch = 0; ch < channelsCount; ++ch)
            {
                fft.SetAverageCount(avgCount);
                fft.SetWindowFunction(wndFunctionSelection);
            }
        }
    }

    pthis->stopProcessing.store(true);
    pthis->stream->Stop();
    pthis->stream.reset();

    for (uint8_t i = 0; i < channelsCount; ++i)
        delete[] buffers[i];

    delete[] buffers;
    pthis->mStreamRunning.store(false);
}

wxString fftviewer_frFFTviewer::printDataRate(float dataRate)
{
    if (dataRate > 1000000)
        return wxString::Format(_("%.3f MB/s"), dataRate / 1000000.0);
    else if (dataRate > 1000)
        return wxString::Format(_("%.3f KB/s"), dataRate / 1000.0);
    else
        return wxString::Format(_("%.0f B/s"), dataRate);
}

void fftviewer_frFFTviewer::SetNyquistFrequency()
{
    double freqHz = 20e6;
    int index = cmbRFSOC->GetSelection();
    if (index < 0)
        return;
    if (device)
        freqHz = device->GetSampleRate(index, TRXDir::Rx, 0);
    if (freqHz <= 0)
        return;
    sampleRate = freqHz;
    txtNyquistFreqMHz->SetValue(wxString::Format(_("%2.5f"), freqHz / 2e6));
    mFFTpanel->SetInitialDisplayArea(-freqHz / 2, freqHz / 2, -115, 0);
}

void fftviewer_frFFTviewer::OnStreamChange(wxCommandEvent& event)
{
    // SetNyquistFrequency();

    // int tmp = cmbChannelVisibility->GetSelection();
    // cmbChannelVisibility->Clear();
    // cmbChannelVisibility->Append(_T("A"));
    // cmbChannelVisibility->Append(_T("B"));
    // if (cmbMode->GetSelection() == 2)
    //     cmbChannelVisibility->Append(_T("A&B"));
    // else if (tmp > 1)
    //     tmp = 0;
    // cmbChannelVisibility->SetSelection(tmp);
}

void fftviewer_frFFTviewer::OnFmtChange(wxCommandEvent& event)
{
    int val = event.GetInt();
    int max = 1.0; //val == 1 ? 32800 : 2050;
    if (val != cmbFmt->GetSelection())
        cmbFmt->SetSelection(val);
    mTimeDomainPanel->SetInitialDisplayArea(0, 1024, -max, max);
    mConstellationPanel->SetInitialDisplayArea(-max, max, -max, max);
}

void fftviewer_frFFTviewer::OnEnPwr(wxCommandEvent& event)
{
    bool en = event.GetInt();
    txtCenterOffset1->Enable(en);
    txtCenterOffset2->Enable(en);
    txtBW1->Enable(en);
    txtBW2->Enable(en);
}

void fftviewer_frFFTviewer::OnChannelVisibilityChange(wxCommandEvent& event)
{
    bool visibilities[cMaxChCount];

    for (int i = 0; i < cMaxChCount; ++i)
        visibilities[i] = cmbChannelVisibility->GetSelection() & (1 << i);

    for (int i = 0; i < cMaxChCount; ++i)
    {
        mTimeDomainPanel->series[2 * i]->visible = visibilities[i];
        mTimeDomainPanel->series[2 * i + 1]->visible = visibilities[i];
        mConstellationPanel->series[i]->visible = visibilities[i];
        mFFTpanel->series[i]->visible = visibilities[i];
    }
}

void fftviewer_frFFTviewer::OnAvgChange(wxSpinEvent& event)
{
    averageCount.store(spinAvgCount->GetValue());
}

void fftviewer_frFFTviewer::OnAvgChangeEnter(wxCommandEvent& event)
{
    averageCount.store(spinAvgCount->GetValue());
}

void fftviewer_frFFTviewer::OnWindowFunctionChange(wxCommandEvent& event)
{
    windowFunctionID.store(cmbWindowFunc->GetSelection());
}
