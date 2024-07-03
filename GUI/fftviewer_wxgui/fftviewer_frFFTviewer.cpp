#include "fftviewer_frFFTviewer.h"
#include <wx/timer.h>
#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <vector>
#include "limesuiteng/Logger.h"
#include "OpenGLGraph.h"
#include "kissFFT/kiss_fft.h"
#include "limesuiteng/LMS7002M.h"
#include "DSP/FFT/FFT.h"
#include <fstream>
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamConfig.h"
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

    for (size_t i = 0; i < desc.rfSOC.size(); ++i)
    {
        cmbRFSOC->Append(desc.rfSOC[i].name);
    }

    uint8_t channelCount = desc.rfSOC.at(0).channelCount;
    if (channelCount <= 1)
    {
        cmbMode->Clear();
        cmbMode->Append("SISO");
        cmbMode->SetSelection(0);

        cmbChannelVisibility->Clear();
        cmbChannelVisibility->Append("A");
        cmbChannelVisibility->SetSelection(0);
    }
    else
    {
        constexpr uint8_t modeChoicesItemCount = 2;
        const std::array<const wxString, modeChoicesItemCount> modeChoices{ "SISO", "MIMO" };
        cmbMode->Set(modeChoicesItemCount, modeChoices.data());
        cmbMode->SetSelection(0);
        cmbMode->GetContainingSizer()->Layout(); // update the width of the box

        constexpr uint8_t channelVisibilityChoicesItemCount = 3;
        const std::array<const wxString, channelVisibilityChoicesItemCount> channelVisibilityChoices{ "A", "B", "A&B" };
        cmbChannelVisibility->Set(channelVisibilityChoicesItemCount, channelVisibilityChoices.data());
        cmbChannelVisibility->SetSelection(0);
        cmbChannelVisibility->GetContainingSizer()->Layout(); // update the width of the box
    }

    cmbRFSOC->SetSelection(0);
    SetNyquistFrequency();

    sbStreamConfig->Layout();
    return true;
}

fftviewer_frFFTviewer::fftviewer_frFFTviewer(wxWindow* parent, wxWindowID id)
    : frFFTviewer(parent, id)
    , mStreamRunning(false)
    , device(nullptr)
    , mGUIupdater(new wxTimer(this, wxID_ANY))
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
    mFFTpanel->AddSerie(new cDataSerie());
    mFFTpanel->AddSerie(new cDataSerie());
    mFFTpanel->series[0]->color = 0xFF0000FF;
    mFFTpanel->series[1]->color = 0x0000FFFF;
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
    mTimeDomainPanel->AddSerie(new cDataSerie());
    mTimeDomainPanel->AddSerie(new cDataSerie());
    mTimeDomainPanel->AddSerie(new cDataSerie());
    mTimeDomainPanel->AddSerie(new cDataSerie());
    mTimeDomainPanel->SetInitialDisplayArea(0, 1024, -1, 1);
    mTimeDomainPanel->settings.title = "IQ samples"s;
    mTimeDomainPanel->series[0]->color = 0xFF0000FF;
    mTimeDomainPanel->series[1]->color = 0x0000FFFF;
    mTimeDomainPanel->series[2]->color = 0xFF00FFFF;
    mTimeDomainPanel->series[3]->color = 0x00FFFFFF;
    mTimeDomainPanel->settings.marginLeft = 48;
    mConstellationPanel->settings.useVBO = true;
    mConstellationPanel->AddSerie(new cDataSerie());
    mConstellationPanel->AddSerie(new cDataSerie());
    mConstellationPanel->series[0]->color = 0xFF0000FF;
    mConstellationPanel->series[1]->color = 0x0000FFFF;
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
    evt.SetInt(0);
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
    switch (cmbMode->GetSelection() % 2)
    {
    case 0: //SISO
        cmbChannelVisibility->SetSelection(0);
        cmbChannelVisibility->Disable();
        threadProcessing = std::thread(StreamingLoop, this, fftSize, 1, 0);
        break;
    case 1: //MIMO
        threadProcessing = std::thread(StreamingLoop, this, fftSize, 2, 0);
        break;
    }

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
    const uint8_t chipIndex = lmsIndex;
    device->StreamStatus(chipIndex, &rxStats, &txStats);

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

    const int fftSize = streamData.fftBins[0].size();

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
            for (int i = 0; i < fftSize; ++i)
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
        if (chkFreezeTimeDomain->IsChecked() == false)
        {
            mTimeDomainPanel->series[0]->AssignValues(&timeXAxis[0], &streamData.samplesI[0][0], streamData.samplesI[0].size());
            mTimeDomainPanel->series[1]->AssignValues(&timeXAxis[0], &streamData.samplesQ[0][0], streamData.samplesQ[0].size());
            mTimeDomainPanel->series[2]->AssignValues(&timeXAxis[0], &streamData.samplesI[1][0], streamData.samplesI[1].size());
            mTimeDomainPanel->series[3]->AssignValues(&timeXAxis[0], &streamData.samplesQ[1][0], streamData.samplesQ[1].size());
        }
        if (chkFreezeConstellation->IsChecked() == false)
        {
            mConstellationPanel->series[0]->AssignValues(
                &streamData.samplesI[0][0], &streamData.samplesQ[0][0], streamData.samplesQ[0].size());
            mConstellationPanel->series[1]->AssignValues(
                &streamData.samplesI[1][0], &streamData.samplesQ[1][0], streamData.samplesQ[1].size());
        }
        if (chkFreezeFFT->IsChecked() == false)
        {
            for (int ch = 0; ch < 2; ++ch)
            {
                for (int s = 0; s < fftSize; ++s)
                {
                    streamData.fftBins[ch][s] = (streamData.fftBins[ch][s] != 0 ? (10 * log10(streamData.fftBins[ch][s])) : -300);
                }
            }
            mFFTpanel->series[0]->AssignValues(&fftFreqAxis[0], &streamData.fftBins[0][0], fftSize);
            mFFTpanel->series[1]->AssignValues(&fftFreqAxis[0], &streamData.fftBins[1][0], fftSize);
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

    updateGUI.store(true);
}

void fftviewer_frFFTviewer::StreamingLoop(
    fftviewer_frFFTviewer* pthis, const unsigned int fftSize, const int channelsCount, const uint32_t format)
{
    const bool runTx = pthis->chkEnTx->GetValue();
    int avgCount = pthis->spinAvgCount->GetValue();
    auto wndFunction = static_cast<lime::FFT::WindowFunctionType>(pthis->windowFunctionID.load());
    bool fftEnabled = true;

    bool syncTx = pthis->chkEnSync->GetValue();

    vector<float> wndCoef;
    lime::FFT::GenerateWindowCoefficients(wndFunction, fftSize, wndCoef);

    lime::complex32f_t** buffers;

    DataToGUI localDataResults;
    localDataResults.nyquist_Hz = 7.68e6;
    for (unsigned i = 0; i < cMaxChCount; i++)
    {
        localDataResults.samplesI[i].resize(fftSize, 0);
        localDataResults.samplesQ[i].resize(fftSize, 0);
        localDataResults.fftBins[i].resize(fftSize, 0);
        pthis->streamData.samplesI[i].resize(fftSize);
        pthis->streamData.samplesQ[i].resize(fftSize);
        pthis->streamData.fftBins[i].resize(fftSize);
    }
    buffers = new lime::complex32f_t*[channelsCount];
    for (int i = 0; i < channelsCount; ++i)
        buffers[i] = new complex32f_t[fftSize];

    vector<complex32f_t> captureBuffer[cMaxChCount];
    uint32_t samplesToCapture = 0;
    uint32_t samplesCaptured = 0;
    if (pthis->captureSamples.load() == true)
        for (int ch = 0; ch < channelsCount; ++ch)
        {
            samplesToCapture = pthis->spinCaptureCount->GetValue();
            captureBuffer[ch].resize(samplesToCapture);
            samplesCaptured = 0;
        }

    auto fmt = pthis->cmbFmt->GetSelection() == 1 ? DataFormat::I16 : DataFormat::I12;

    StreamConfig config;

    config.format = DataFormat::F32;
    config.linkFormat = fmt;
    for (int i = 0; i < channelsCount; ++i)
    {
        config.channels.at(TRXDir::Rx).push_back(i);
        if (runTx)
        {
            config.channels.at(TRXDir::Tx).push_back(i);
        }
    }

    kiss_fft_cfg m_fftCalcPlan = kiss_fft_alloc(fftSize, 0, nullptr, nullptr);
    kiss_fft_cpx* m_fftCalcIn = new kiss_fft_cpx[fftSize];
    kiss_fft_cpx* m_fftCalcOut = new kiss_fft_cpx[fftSize];

    const uint8_t chipIndex = pthis->lmsIndex;

    // TODO: check if actually needed
    /*std::vector<std::vector<complex32f_t>> txPattern(2);
    for (uint32_t i = 0; i < txPattern.size(); ++i)
    {
        txPattern[i].resize(fftSize);
        float srcI[8]; // = {1.0, 0.0, -1.0, 0.0};
        float srcQ[8]; // = {-1.0, 0.0, 1.0, 0.0};
        for (int j = 0; j < 8; ++j)
        {
            srcI[j] = cos(j * 2 * 3.141592 / 8); // = {1.0, 0.0, -1.0, 0.0};
            srcQ[j] = sin(j * 2 * 3.141592 / 8); // = {-1.0, 0.0, 1.0, 0.0};
        }

        float ampl = 1.0; //(j+1)*(1.0/(txPacketCount+1));
        for (uint32_t k = 0; k < fftSize; ++k)
        {
            txPattern[i][k].q = srcQ[k & 7] * ampl;
            txPattern[i][k].i = srcI[k & 7] * ampl;
        }
    }

    const lime::complex32f_t* src[2] = { txPattern[0].data(), txPattern[1].data() };*/

    try
    {
        pthis->device->StreamSetup(config, chipIndex);
        pthis->device->StreamStart(chipIndex);
    } catch (std::logic_error& e)
    {
        lime::error("%s", e.what());
    } catch (std::runtime_error& e)
    {
        lime::error("%s", e.what());
    }

    // uint16_t regVal = 0;
    // TODO:
    // if (LMS_ReadFPGAReg(pthis->device, 0x0008, &regVal) == 0)
    // {
    //     wxCommandEvent* e = new wxCommandEvent(wxEVT_COMMAND_CHOICE_SELECTED);
    //     e->SetInt((regVal&2) ? 0 : 1);
    //     wxQueueEvent(pthis->cmbFmt, e);
    // }

    pthis->mStreamRunning.store(true);
    StreamMeta txMeta{};
    txMeta.waitForTimestamp = syncTx;
    txMeta.flushPartialPacket = true;
    int fftCounter = 0;

    StreamMeta rxMeta{};

    while (pthis->stopProcessing.load() == false)
    {
        do
        {
            uint32_t samplesPopped;
            samplesPopped = pthis->device->StreamRx(chipIndex, buffers, fftSize, &rxMeta);
            if (samplesPopped <= 0)
                continue;

            int64_t rxTS = rxMeta.timestamp;

            if (runTx)
            {
                txMeta.timestamp = rxTS + 1020 * 128;
                pthis->device->StreamTx(chipIndex, buffers, fftSize, &txMeta);
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
                    if (wndFunction == lime::FFT::WindowFunctionType::NONE)
                    {
                        for (unsigned i = 0; i < fftSize; ++i)
                        {
                            m_fftCalcIn[i].r = buffers[ch][i].real();
                            m_fftCalcIn[i].i = buffers[ch][i].imag();
                        }
                    }
                    else
                    {
                        for (unsigned i = 0; i < fftSize; ++i)
                        {
                            m_fftCalcIn[i].r = buffers[ch][i].real() * wndCoef[i];
                            m_fftCalcIn[i].i = buffers[ch][i].imag() * wndCoef[i];
                        }
                    }

                    kiss_fft(m_fftCalcPlan, m_fftCalcIn, m_fftCalcOut);

                    int output_index = 0;
                    for (unsigned i = fftSize / 2 + 1; i < fftSize; ++i)
                        localDataResults.fftBins[ch][output_index++] +=
                            m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;
                    for (unsigned i = 0; i < fftSize / 2 + 1; ++i)
                        localDataResults.fftBins[ch][output_index++] +=
                            m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i;
                }
            }
            ++fftCounter;
        } while (fftCounter < avgCount && pthis->stopProcessing.load() == false);

        if (fftCounter >= avgCount && pthis->updateGUI.load() == true)
        {
            //shift fft
            if (fftEnabled)
            {
                for (int ch = 0; ch < channelsCount; ++ch)
                {
                    for (unsigned s = 0; s < fftSize; ++s)
                    {
                        const float div = static_cast<float>(fftCounter) * fftSize * fftSize;
                        localDataResults.fftBins[ch][s] /= div;
                    }
                }
            }
            if (pthis->stopProcessing.load() == false)
            {
                pthis->streamData = localDataResults;
                wxThreadEvent* evt = new wxThreadEvent();
                evt->SetEventObject(pthis);
                pthis->updateGUI.store(false);
                pthis->QueueEvent(evt);
            }
            fftCounter = 0;
            fftEnabled = pthis->enableFFT.load();
            avgCount = pthis->averageCount.load();
            auto wndFunctionSelection = static_cast<lime::FFT::WindowFunctionType>(pthis->windowFunctionID.load());
            if (wndFunctionSelection != wndFunction)
            {
                wndFunction = wndFunctionSelection;
                lime::FFT::GenerateWindowCoefficients(wndFunction, fftSize, wndCoef);
            }
        }
    }

    /*  if(pthis->captureSamples.load() == true)
    {
        ofstream fout;
        fout.open(pthis->captureFilename);
        fout << "AI\tAQ";
        if(channelsCount > 1)
            fout << "\tBI\tBQ";
        fout << endl;

        int samplesCnt = captureBuffer[0].size();
        for(int i=0; i<samplesCnt; ++i)
        {
            for(int ch=0; ch<channelsCount; ++ch)
            {
                fout << captureBuffer[ch][i].i << "\t" << captureBuffer[ch][i].q << "\t";
            }
            fout << endl;
        }
        fout.close();

        string filename = pthis->captureFilename+".absdbfs";
        fout.open(filename);
        fout << "AI\tAQ";
        if(channelsCount > 1)
            fout << "\tBI\tBQ";
        fout << endl;

        for(int i=0; i<samplesCnt; ++i)
        {
            for(int ch=0; ch<channelsCount; ++ch)
            {
                fout
                << (captureBuffer[ch][i].i == 0 ? -67 : 20*log10(abs(captureBuffer[ch][i].i)/2048))<< "\t"
                << (captureBuffer[ch][i].q == 0 ? -67 : 20*log10(abs(captureBuffer[ch][i].q)/2048))<< "\t";
            }
            fout << endl;
        }
        fout.close();
    }*/

    kiss_fft_free(m_fftCalcPlan);
    pthis->stopProcessing.store(true);
    pthis->device->StreamStop(chipIndex);
    pthis->device->StreamDestroy(chipIndex);

    for (int i = 0; i < channelsCount; ++i)
        delete[] buffers[i];
    delete[] buffers;
    delete[] m_fftCalcIn;
    delete[] m_fftCalcOut;
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
    txtNyquistFreqMHz->SetValue(wxString::Format(_("%2.5f"), freqHz / 2e6));
    mFFTpanel->SetInitialDisplayArea(-freqHz / 2, freqHz / 2, -115, 0);
}

void fftviewer_frFFTviewer::OnStreamChange(wxCommandEvent& event)
{
    SetNyquistFrequency();

    int tmp = cmbChannelVisibility->GetSelection();
    cmbChannelVisibility->Clear();
    cmbChannelVisibility->Append(_T("A"));
    cmbChannelVisibility->Append(_T("B"));
    if (cmbMode->GetSelection() % 2 == 1)
        cmbChannelVisibility->Append(_T("A&B"));
    else if (tmp > 1)
        tmp = 0;
    cmbChannelVisibility->SetSelection(tmp);
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

    switch (event.GetInt())
    {
    case 0:
        visibilities[0] = true;
        visibilities[1] = false;
        break;
    case 1:
        visibilities[0] = false;
        visibilities[1] = true;
        break;
    case 2:
        visibilities[0] = true;
        visibilities[1] = true;
        break;
    default:
        visibilities[0] = false;
        visibilities[1] = false;
        break;
    }

    mTimeDomainPanel->series[0]->visible = visibilities[0];
    mTimeDomainPanel->series[1]->visible = visibilities[0];
    mTimeDomainPanel->series[2]->visible = visibilities[1];
    mTimeDomainPanel->series[3]->visible = visibilities[1];
    mConstellationPanel->series[0]->visible = visibilities[0];
    mConstellationPanel->series[1]->visible = visibilities[1];
    mFFTpanel->series[0]->visible = visibilities[0];
    mFFTpanel->series[1]->visible = visibilities[1];
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
