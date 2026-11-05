#include "SDRConfiguration_view.h"
#include "commonWxHeaders.h"

#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"

#include <wx/settings.h>
#include <wx/spinctrl.h>
#include <wx/utils.h>

#include <algorithm>
#include <cmath>

using namespace lime;

namespace {

constexpr double MHz = 1e6; // GUI shows frequencies in MHz, the API takes Hz
constexpr uint8_t CLK_REFERENCE_ID = 0; // LMS7002M::ClockID::CLK_REFERENCE

double ParseNumber(const wxTextCtrl* field)
{
    double value = 0;
    const wxString text = field->GetValue().Trim().Trim(false);
    if (!text.ToDouble(&value) && !text.ToCDouble(&value))
        return 0;
    return value;
}

double ParseMHz(const wxTextCtrl* field)
{
    return ParseNumber(field) * MHz;
}

wxString FormatMHz(double hz)
{
    wxString text = wxString::Format("%.6f", hz / MHz);
    // strip insignificant trailing zeros, keep at least one digit after the separator
    while (text.length() > 1 && text.EndsWith("0") && !text.EndsWith(".0") && !text.EndsWith(",0"))
        text.RemoveLast();
    return text;
}

// Oversample selection index -> SDRConfig/SetSampleRate oversample value: 0 = max (device decides), 1 = bypass, 2^n
const wxString oversampleNames[] = { "max", "bypass", "x2", "x4", "x8", "x16", "x32" };

uint8_t OversampleFromSelection(int selection)
{
    return selection > 0 ? (1 << (selection - 1)) : 0;
}

wxString DirName(TRXDir dir)
{
    return dir == TRXDir::Rx ? "Rx" : "Tx";
}

wxString ChannelLetter(uint8_t channel)
{
    return wxString::Format("%c", static_cast<char>('A' + channel));
}

wxString ChannelName(TRXDir dir, uint8_t channel)
{
    return DirName(dir) + " channel " + ChannelLetter(channel);
}

/// Number of channels of the given direction
uint8_t ChannelCount(const RFSOCDescriptor& descriptor, TRXDir dir)
{
    // The descriptor has a single channel count; directions are handled separately,
    // so a per-direction count can be plugged in here.
    return std::min<uint8_t>(descriptor.channelCount, SDRConfig::MAX_CHANNEL_COUNT);
}

} // namespace

// -------------------------------------------------------------------------------------------------
// SOCConfig_view
// -------------------------------------------------------------------------------------------------

SOCConfig_view::SOCConfig_view(wxWindow* parent, SDRDevice* sdrDevice, uint8_t index, StatusReporter reporter)
    : wxPanel(parent, wxID_ANY)
    , device(sdrDevice)
    , socIndex(index)
    , report(std::move(reporter))
{
    const RFSOCDescriptor& descriptor = device->GetDescriptor().rfSOC.at(socIndex);
    // sized once: the event handlers keep references to the elements
    rxChannels.resize(ChannelCount(descriptor, TRXDir::Rx));
    txChannels.resize(ChannelCount(descriptor, TRXDir::Tx));

    wxStaticBoxSizer* socBox = new wxStaticBoxSizer(wxVERTICAL, this, descriptor.name);
    wxWindow* base = socBox->GetStaticBox();

    socBox->Add(CreateSharedSection(base), wxSizerFlags().Expand().Border(wxALL, 2));

    wxBoxSizer* directionsSizer = new wxBoxSizer(wxHORIZONTAL);
    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
        if (!Channels(dir).empty())
            directionsSizer->Add(CreateDirectionSection(base, dir), wxSizerFlags().Border(wxALL, 2));
    socBox->Add(directionsSizer);

    SetSizerAndFit(socBox);
    SetImmediateMode(immediateMode);
}

wxSizer* SOCConfig_view::CreateSharedSection(wxWindow* parent)
{
    wxStaticBoxSizer* box = new wxStaticBoxSizer(wxVERTICAL, parent, "Shared (all channels)");
    wxWindow* base = box->GetStaticBox();

    const wxSizerFlags labelFlags = wxSizerFlags().CenterVertical().Right();
    const wxSizerFlags ctrlFlags = wxSizerFlags().CenterVertical().Expand();

    wxFlexGridSizer* grid = new wxFlexGridSizer(4, 4, 8);

    double refClockHz = 0;
    try
    {
        refClockHz = device->GetClockFreq(CLK_REFERENCE_ID, socIndex * 2);
    } catch (std::exception&)
    {
    }

    // Reference clock
    grid->Add(new wxStaticText(base, wxID_ANY, "Reference clock (MHz):"), labelFlags);
    refClock = new wxTextCtrl(base, wxID_ANY, FormatMHz(refClockHz), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    refClock->SetToolTip("Reference clock frequency. 0 keeps the device's current reference.");
    BindValueField(refClock, [this]() { ApplyReferenceClock(); });
    grid->Add(refClock, ctrlFlags);

    grid->Add(new wxStaticText(base, wxID_ANY, "Source:"), labelFlags);
    wxArrayString sources;
    sources.Add("Internal");
    sources.Add("External");
    refClockSource = new wxChoice(base, wxID_ANY, wxDefaultPosition, wxDefaultSize, sources);
    refClockSource->SetSelection(0);
    refClockSource->SetToolTip("Reference clock source (SDRConfig::referenceClockSource). Applied in Aggregate mode only.");
    grid->Add(refClockSource, ctrlFlags);

    // Sampling
    grid->Add(new wxStaticText(base, wxID_ANY, "Sample rate (MHz):"), labelFlags);
    sampleRate = new wxTextCtrl(base, wxID_ANY, "10", wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    {
        const auto& range = device->GetDescriptor().rfSOC.at(socIndex).samplingRateRange;
        if (range.max > 0)
            sampleRate->SetToolTip(wxString::Format("Range: %s - %s MHz", FormatMHz(range.min), FormatMHz(range.max)));
    }
    BindValueField(sampleRate, [this]() { ApplySampleRate(); });
    grid->Add(sampleRate, ctrlFlags);

    grid->Add(new wxStaticText(base, wxID_ANY, "Oversample:"), labelFlags);
    oversample = new wxChoice(base, wxID_ANY, wxDefaultPosition, wxDefaultSize, WXSIZEOF(oversampleNames), oversampleNames);
    oversample->SetSelection(2);
    oversample->SetToolTip("RF (ADC/DAC) rate = sample rate x oversample, shared by Rx and Tx");
    oversample->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) { ApplySampleRate(); });
    grid->Add(oversample, ctrlFlags);

    // LO
    wxString loTooltip;
    {
        const auto& range = device->GetDescriptor().rfSOC.at(socIndex).frequencyRange;
        if (range.max > 0)
            loTooltip = wxString::Format("Range: %s - %s MHz", FormatMHz(range.min), FormatMHz(range.max));
    }
    grid->Add(new wxStaticText(base, wxID_ANY, "Rx LO (MHz):"), labelFlags);
    rxLO = new wxTextCtrl(base, wxID_ANY, "1000", wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    rxLO->SetToolTip(loTooltip);
    BindValueField(rxLO, [this]() { ApplyLO(TRXDir::Rx); });
    grid->Add(rxLO, ctrlFlags);

    grid->Add(new wxStaticText(base, wxID_ANY, "Tx LO (MHz):"), labelFlags);
    txLO = new wxTextCtrl(base, wxID_ANY, "1000", wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    txLO->SetToolTip(loTooltip);
    BindValueField(txLO, [this]() { ApplyLO(TRXDir::Tx); });
    grid->Add(txLO, ctrlFlags);

    grid->AddSpacer(0);
    tdd = new wxCheckBox(base, wxID_ANY, "TDD (Rx uses Tx LO)");
    tdd->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent&) { OnTDDChanged(); });
    grid->Add(tdd, ctrlFlags);

    box->Add(grid, wxSizerFlags().Border(wxALL, 2));
    return box;
}

void SOCConfig_view::CreateDirectionControls(wxWindow* parent, uint8_t channel, TRXDir dir)
{
    const RFSOCDescriptor& descriptor = device->GetDescriptor().rfSOC.at(socIndex);
    ChannelDirectionGUI& gui = Channels(dir).at(channel);
    const wxString name = ChannelName(dir, channel);

    gui.enable = new wxCheckBox(parent, wxID_ANY, "");
    gui.enable->SetValue(true);
    gui.enable->Bind(wxEVT_CHECKBOX, [this, channel, dir, name](wxCommandEvent& event) {
        const bool enable = event.IsChecked();
        ApplyImmediate(name + (enable ? " enable" : " disable"),
            [this, channel, dir, enable]() { return device->EnableChannel(socIndex, dir, channel, enable); });
    });

    wxArrayString antennaNames;
    const auto paths = descriptor.pathNames.find(dir);
    if (paths != descriptor.pathNames.end())
        for (const auto& pathName : paths->second)
            antennaNames.Add(pathName);
    gui.antenna = new wxChoice(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, antennaNames);
    // index 0 is usually "NONE"
    gui.antenna->SetSelection(antennaNames.size() > 1 ? 1 : 0);
    gui.antenna->Bind(wxEVT_CHOICE, [this, channel, dir, name, &gui](wxCommandEvent&) {
        const int path = gui.antenna->GetSelection();
        if (path < 0)
            return;
        ApplyImmediate(name + " antenna " + gui.antenna->GetString(path),
            [this, channel, dir, path]() { return device->SetAntenna(socIndex, dir, channel, path); });
    });

    // Generic gain: a single dB value, distributed by the device among its amplifiers
    Range<double> gainRange(0, 100);
    const auto ranges = descriptor.gainRange.find(dir);
    if (ranges != descriptor.gainRange.end())
    {
        const auto generic = ranges->second.find(eGainTypes::GENERIC);
        if (generic != ranges->second.end())
            gainRange = generic->second;
    }
    gui.gain = new wxSpinCtrlDouble(parent,
        wxID_ANY,
        "",
        wxDefaultPosition,
        wxDefaultSize,
        wxSP_ARROW_KEYS | wxTE_PROCESS_ENTER,
        gainRange.min,
        gainRange.max,
        std::clamp(0.0, gainRange.min, gainRange.max),
        1.0);
    gui.gain->SetDigits(0);
    gui.gain->SetToolTip(wxString::Format("Generic gain, %g - %g dB", gainRange.min, gainRange.max));
    gui.gain->Bind(wxEVT_SPINCTRLDOUBLE, [this, channel, dir, name](wxSpinDoubleEvent& event) {
        const double value = event.GetValue();
        ApplyImmediate(wxString::Format("%s gain %g dB", name, value),
            [this, channel, dir, value]() { return device->SetGain(socIndex, dir, channel, eGainTypes::GENERIC, value); });
    });

    gui.lpf = new wxTextCtrl(parent, wxID_ANY, "0", wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    {
        const auto lpfRange = descriptor.lowPassFilterRange.find(dir);
        if (lpfRange != descriptor.lowPassFilterRange.end())
            gui.lpf->SetToolTip(wxString::Format(
                "Low pass filter bandwidth, %s - %s MHz", FormatMHz(lpfRange->second.min), FormatMHz(lpfRange->second.max)));
    }
    BindValueField(gui.lpf, [this, channel, dir, name, &gui]() {
        const double bandwidth = ParseMHz(gui.lpf);
        ApplyImmediate(name + " LPF " + FormatMHz(bandwidth) + " MHz",
            [this, channel, dir, bandwidth]() { return device->SetLowPassFilter(socIndex, dir, channel, bandwidth); });
    });

    gui.nco = new wxTextCtrl(parent, wxID_ANY, "0", wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER);
    gui.nco->SetToolTip("NCO frequency offset from the LO. 0 disables the NCO.");
    BindValueField(gui.nco, [this, channel, dir, name, &gui]() {
        const double offset = ParseMHz(gui.nco);
        ApplyImmediate(name + " NCO " + FormatMHz(offset) + " MHz",
            [this, channel, dir, offset]() { return device->SetNCOFrequency(socIndex, dir, channel, 0, offset, 0); });
    });

    gui.calibrate = new wxCheckBox(parent, wxID_ANY, "");
    gui.calibrate->SetToolTip("Run DC/IQ calibration after applying the configuration");

    gui.btnCalibrate = new wxButton(parent, wxID_ANY, "Calibrate");
    gui.btnCalibrate->SetToolTip("Run DC/IQ calibration now, using the LPF bandwidth (or the sample rate if LPF is 0)");
    gui.btnCalibrate->Bind(wxEVT_BUTTON, [this, channel, dir](wxCommandEvent&) { Calibrate(channel, dir); });
}

wxSizer* SOCConfig_view::CreateDirectionSection(wxWindow* parent, TRXDir dir)
{
    wxStaticBoxSizer* box = new wxStaticBoxSizer(wxVERTICAL, parent, DirName(dir) + " channels");
    wxWindow* base = box->GetStaticBox();

    std::vector<ChannelDirectionGUI>& channels = Channels(dir);
    for (uint8_t ch = 0; ch < channels.size(); ++ch)
        CreateDirectionControls(base, ch, dir);

    const wxSizerFlags labelFlags = wxSizerFlags().CenterVertical().Right();
    const wxSizerFlags ctrlFlags = wxSizerFlags().CenterVertical().Expand();
    const wxSizerFlags checkFlags = wxSizerFlags().Center();

    // label column + one column per channel
    wxFlexGridSizer* grid = new wxFlexGridSizer(1 + channels.size(), 4, 8);
    grid->AddSpacer(0);
    for (uint8_t ch = 0; ch < channels.size(); ++ch)
        grid->Add(new wxStaticText(base, wxID_ANY, ChannelLetter(ch)), checkFlags);

    // one row per parameter, one column per channel
    auto row = [&](const wxString& label, auto member, const wxSizerFlags& flags) {
        if (label.IsEmpty())
            grid->AddSpacer(0);
        else
            grid->Add(new wxStaticText(base, wxID_ANY, label), labelFlags);
        for (const auto& gui : channels)
            grid->Add(gui.*member, flags);
    };
    row("Enable:", &ChannelDirectionGUI::enable, checkFlags);
    row("Antenna:", &ChannelDirectionGUI::antenna, ctrlFlags);
    row("Gain (dB):", &ChannelDirectionGUI::gain, ctrlFlags);
    row("LPF (MHz):", &ChannelDirectionGUI::lpf, ctrlFlags);
    row("NCO (MHz):", &ChannelDirectionGUI::nco, ctrlFlags);
    row("Calibrate on apply:", &ChannelDirectionGUI::calibrate, checkFlags);
    row("", &ChannelDirectionGUI::btnCalibrate, ctrlFlags);

    box->Add(grid, wxSizerFlags().Border(wxALL, 2));
    return box;
}

void SOCConfig_view::BindValueField(wxTextCtrl* field, std::function<void()> onCommit)
{
    // Enter always commits, so the same value can be re-sent; focus loss commits only edited values
    field->Bind(wxEVT_TEXT_ENTER, [field, onCommit](wxCommandEvent&) {
        field->DiscardEdits();
        onCommit();
    });
    field->Bind(wxEVT_KILL_FOCUS, [field, onCommit](wxFocusEvent& event) {
        event.Skip();
        if (!field->IsModified())
            return;
        field->DiscardEdits();
        onCommit();
    });
}

void SOCConfig_view::ApplyImmediate(const wxString& what, const std::function<OpStatus()>& operation)
{
    if (!immediateMode || !device)
        return;

    wxBusyCursor busy;
    try
    {
        const OpStatus status = operation();
        if (status == OpStatus::Success)
            report(what + ": OK", false);
        else
            report(what + ": " + ToString(status), true);
    } catch (std::exception& e)
    {
        report(what + ": " + e.what(), true);
    }
}

void SOCConfig_view::SetImmediateMode(bool immediate)
{
    immediateMode = immediate;
    // Reference clock source and calibrate-on-apply exist only in SDRConfig, no individual API for them
    refClockSource->Enable(!immediate);
    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
        for (auto& gui : Channels(dir))
            gui.calibrate->Enable(!immediate);
}

double SOCConfig_view::GetSampleRate() const
{
    return ParseMHz(sampleRate);
}

uint8_t SOCConfig_view::GetOversample() const
{
    return OversampleFromSelection(oversample->GetSelection());
}

void SOCConfig_view::ApplyReferenceClock()
{
    const double frequency = ParseMHz(refClock);
    if (frequency <= 0)
        return;
    ApplyImmediate("Reference clock " + FormatMHz(frequency) + " MHz",
        [this, frequency]() { return device->SetClockFreq(CLK_REFERENCE_ID, frequency, socIndex * 2); });
}

void SOCConfig_view::ApplySampleRate()
{
    const double rate = GetSampleRate();
    const uint8_t ovs = GetOversample();
    // SetSampleRate configures Rx and Tx together with the same oversampling
    ApplyImmediate(wxString::Format("Sample rate %s MHz, oversample %s", FormatMHz(rate), oversample->GetStringSelection()),
        [this, rate, ovs]() { return device->SetSampleRate(socIndex, TRXDir::Rx, 0, rate, ovs); });
}

void SOCConfig_view::ApplyLO(TRXDir dir)
{
    if (dir == TRXDir::Rx && tdd->IsChecked())
        return; // Rx follows Tx LO

    const double frequency = ParseMHz(dir == TRXDir::Rx ? rxLO : txLO);
    const bool alsoRx = dir == TRXDir::Tx && tdd->IsChecked();
    ApplyImmediate(
        DirName(dir) + " LO " + FormatMHz(frequency) + " MHz" + (alsoRx ? " (TDD)" : ""), [this, dir, frequency, alsoRx]() {
            OpStatus status = device->SetFrequency(socIndex, dir, 0, frequency);
            if (status == OpStatus::Success && alsoRx)
                status = device->SetFrequency(socIndex, TRXDir::Rx, 0, frequency);
            return status;
        });
}

void SOCConfig_view::OnTDDChanged()
{
    const bool enabled = tdd->IsChecked();
    rxLO->Enable(!enabled);
    // Tx LO stays as is, Rx is retuned either to Tx LO or back to its own value
    const double frequency = ParseMHz(enabled ? txLO : rxLO);
    ApplyImmediate(wxString("TDD ") + (enabled ? "on" : "off") + ", Rx LO " + FormatMHz(frequency) + " MHz",
        [this, frequency]() { return device->SetFrequency(socIndex, TRXDir::Rx, 0, frequency); });
}

void SOCConfig_view::Calibrate(uint8_t channel, TRXDir dir)
{
    if (!device)
        return;

    const ChannelDirectionGUI& gui = Channels(dir).at(channel);
    double bandwidth = ParseMHz(gui.lpf);
    if (bandwidth <= 0)
        bandwidth = GetSampleRate();

    const wxString what = wxString::Format("%s calibration (%s MHz)", ChannelName(dir, channel), FormatMHz(bandwidth));
    wxBusyCursor busy;
    try
    {
        const OpStatus status = device->Calibrate(socIndex, dir, channel, bandwidth);
        report(
            what + ": " + (status == OpStatus::Success ? wxString("OK") : wxString(ToString(status))), status != OpStatus::Success);
    } catch (std::exception& e)
    {
        report(what + ": " + e.what(), true);
    }
}

void SOCConfig_view::FillConfig(SDRConfig& config) const
{
    config.referenceClockFreq = ParseMHz(refClock);
    config.referenceClockSource = refClockSource->GetSelection();

    const double rate = GetSampleRate();
    const uint8_t ovs = GetOversample();
    const double txFrequency = ParseMHz(txLO);
    const double rxFrequency = tdd->IsChecked() ? txFrequency : ParseMHz(rxLO);

    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
    {
        const auto& channels = Channels(dir);
        for (uint8_t i = 0; i < channels.size(); ++i)
        {
            const ChannelDirectionGUI& gui = channels[i];
            ChannelConfig::Direction& d = config.channel[i].GetDirection(dir);
            d.centerFrequency = dir == TRXDir::Rx ? rxFrequency : txFrequency;
            d.enabled = gui.enable->IsChecked();
            d.sampleRate = rate;
            d.oversample = ovs;
            d.path = std::max(0, gui.antenna->GetSelection());
            d.gain[eGainTypes::GENERIC] = gui.gain->GetValue();
            d.lpf = ParseMHz(gui.lpf);
            d.NCOoffset = ParseMHz(gui.nco);
            d.calibrate = gui.calibrate->IsChecked() ? CalibrationFlag::DCIQ : CalibrationFlag::NONE;
        }
    }
}

void SOCConfig_view::ReadFromDevice()
{
    if (!device)
        return;

    // Read each value on its own, so one unsupported getter does not prevent the others
    auto tryRead = [this](const wxString& what, const std::function<void()>& read) {
        try
        {
            read();
        } catch (std::exception& e)
        {
            report("Read " + what + ": " + e.what(), true);
        }
    };

    tryRead("reference clock", [this]() {
        const double frequency = device->GetClockFreq(CLK_REFERENCE_ID, socIndex * 2);
        if (frequency > 0)
            refClock->ChangeValue(FormatMHz(frequency));
    });

    tryRead("sample rate", [this]() {
        const double rate = device->GetSampleRate(socIndex, TRXDir::Rx, 0);
        if (rate > 0)
            sampleRate->ChangeValue(FormatMHz(rate));
    });

    tryRead("LO", [this]() {
        const double rx = device->GetFrequency(socIndex, TRXDir::Rx, 0);
        const double tx = device->GetFrequency(socIndex, TRXDir::Tx, 0);
        txLO->ChangeValue(FormatMHz(tx));
        // in TDD the Rx readback is the Tx LO; keep the own Rx LO value for when TDD is turned off
        const bool isTDD = std::fabs(rx - tx) <= 20;
        tdd->SetValue(isTDD);
        rxLO->Enable(!isTDD);
        if (!isTDD)
            rxLO->ChangeValue(FormatMHz(rx));
    });

    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
    {
        auto& channels = Channels(dir);
        for (uint8_t i = 0; i < channels.size(); ++i)
        {
            ChannelDirectionGUI& gui = channels[i];
            const wxString name = ChannelName(dir, i);

            tryRead(name + " antenna", [this, &gui, i, dir]() {
                const unsigned path = device->GetAntenna(socIndex, dir, i);
                if (path < gui.antenna->GetCount())
                    gui.antenna->SetSelection(path);
            });
            tryRead(name + " gain", [this, &gui, i, dir]() {
                double value = 0;
                if (device->GetGain(socIndex, dir, i, eGainTypes::GENERIC, value) == OpStatus::Success)
                    gui.gain->SetValue(value);
            });
            tryRead(name + " LPF",
                [this, &gui, i, dir]() { gui.lpf->ChangeValue(FormatMHz(device->GetLowPassFilter(socIndex, dir, i))); });
            tryRead(name + " NCO", [this, &gui, i, dir]() {
                double phase = 0;
                gui.nco->ChangeValue(FormatMHz(device->GetNCOFrequency(socIndex, dir, i, 0, phase)));
            });
        }
    }
}

// -------------------------------------------------------------------------------------------------
// SDRConfiguration_view
// -------------------------------------------------------------------------------------------------

SDRConfiguration_view::SDRConfiguration_view(wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style)
    : ISOCPanel(parent, id, pos, size, style)
    , sdrDevice(nullptr)
    , mainSizer(new wxBoxSizer(wxVERTICAL))
    , socSizer(new wxBoxSizer(wxVERTICAL))
{
    wxBoxSizer* modeSizer = new wxBoxSizer(wxHORIZONTAL);
    const wxSizerFlags flags = wxSizerFlags().CenterVertical().Border(wxALL, 4);

    const wxString modes[] = { "Immediate", "Aggregate" };
    modeSelect = new wxRadioBox(
        this, wxID_ANY, "Configuration mode", wxDefaultPosition, wxDefaultSize, WXSIZEOF(modes), modes, 1, wxRA_SPECIFY_ROWS);
    modeSelect->SetSelection(0);
    modeSelect->SetItemToolTip(0, "Every changed parameter is configured to the device immediately, using that parameter's API");
    modeSelect->SetItemToolTip(1,
        "Parameters are only collected. \"Apply\" initializes the device and configures all of them at once "
        "(SDRDevice::Configure)");
    modeSelect->Bind(wxEVT_RADIOBOX, [this](wxCommandEvent&) { OnModeChanged(); });
    modeSizer->Add(modeSelect, flags);

    btnApply = new wxButton(this, wxID_ANY, "Apply");
    btnApply->SetToolTip("Initialize the device and configure all parameters");
    btnApply->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { ApplyConfiguration(); });
    modeSizer->Add(btnApply, flags);

    btnReadBack = new wxButton(this, wxID_ANY, "Read from device");
    btnReadBack->SetToolTip("Update the fields with the device's current settings");
    btnReadBack->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { UpdateGUI(); });
    modeSizer->Add(btnReadBack, flags);

    mainSizer->Add(modeSizer);
    mainSizer->Add(socSizer);

    txtStatus = new wxStaticText(this, wxID_ANY, "");
    mainSizer->Add(txtStatus, wxSizerFlags().Expand().Border(wxALL, 4));

    OnModeChanged();
    SetSizerAndFit(mainSizer);
}

bool SDRConfiguration_view::Initialize(void* device)
{
    sdrDevice = reinterpret_cast<lime::SDRDevice*>(device);

    socSizer->Clear(true);
    socGUI.clear();

    btnApply->Enable(sdrDevice && modeSelect->GetSelection() == 1);
    btnReadBack->Enable(sdrDevice != nullptr);

    if (!sdrDevice)
    {
        ReportStatus("No device", false);
        SetSizerAndFit(mainSizer);
        return false;
    }

    const SDRDescriptor& desc = sdrDevice->GetDescriptor();
    for (size_t i = 0; i < desc.rfSOC.size(); ++i)
    {
        SOCConfig_view* soc = new SOCConfig_view(
            this, sdrDevice, i, [this](const wxString& message, bool isError) { ReportStatus(message, isError); });
        soc->SetImmediateMode(modeSelect->GetSelection() == 0);
        socSizer->Add(soc, wxSizerFlags().Border(wxALL, 2));
        socGUI.push_back(soc);
    }
    ReportStatus("", false);
    SetSizerAndFit(mainSizer);
    return true;
}

void SDRConfiguration_view::UpdateGUI()
{
    if (!sdrDevice)
        return;

    wxBusyCursor busy;
    ReportStatus("", false);
    for (auto* soc : socGUI)
        soc->ReadFromDevice();
    if (txtStatus->GetLabel().IsEmpty())
        ReportStatus("Settings read from device", false);
}

void SDRConfiguration_view::OnModeChanged()
{
    const bool aggregateMode = modeSelect->GetSelection() == 1;
    btnApply->Enable(sdrDevice && aggregateMode);
    for (auto* soc : socGUI)
        soc->SetImmediateMode(!aggregateMode);
    ReportStatus(aggregateMode ? "Aggregate mode: press Apply to initialize the device and configure all parameters"
                               : "Immediate mode: changes are applied as they are made",
        false);
}

void SDRConfiguration_view::ApplyConfiguration()
{
    if (!sdrDevice)
        return;

    wxBusyCursor busy;
    try
    {
        OpStatus status = sdrDevice->Init();
        if (status != OpStatus::Success)
        {
            ReportStatus("Device initialization failed: " + ToString(status), true);
            return;
        }

        for (auto* soc : socGUI)
        {
            SDRConfig config;
            soc->FillConfig(config);
            status = sdrDevice->Configure(config, soc->GetSOCIndex());
            if (status != OpStatus::Success)
            {
                ReportStatus(wxString::Format("%s configuration failed: %s",
                                 sdrDevice->GetDescriptor().rfSOC.at(soc->GetSOCIndex()).name,
                                 ToString(status).c_str()),
                    true);
                return;
            }
        }
    } catch (std::exception& e) // std::logic_error: settings problem, std::runtime_error: communications problem
    {
        ReportStatus(wxString("Configure failed: ") + e.what(), true);
        return;
    }
    ReportStatus("Configuration applied", false);
}

void SDRConfiguration_view::ReportStatus(const wxString& message, bool isError)
{
    txtStatus->SetForegroundColour(isError ? *wxRED : wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT));
    txtStatus->SetLabel(message);
}
