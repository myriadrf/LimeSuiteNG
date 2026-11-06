#include "SDRConfiguration_view.h"
#include "commonWxHeaders.h"

#include "limesuiteng/IRFChannel.h"
#include "limesuiteng/IRFSOC.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/ToString.h"

#include "limesuiteng/IDCCorrector.h"
#include "limesuiteng/IQuadratureErrorCorrector.h"

#include <wx/grid.h>
#include <wx/renderer.h>
#include <wx/settings.h>
#include <wx/spinctrl.h>
#include <wx/utils.h>
#include <wx/wupdlock.h>

#include <algorithm>
#include <cmath>

using namespace lime;

namespace {

constexpr double MHz = 1e6; // GUI shows frequencies in MHz, the API takes Hz
constexpr uint8_t CLK_REFERENCE_ID = 0; // LMS7002M::ClockID::CLK_REFERENCE

double ParseNumber(wxString text)
{
    double value = 0;
    text.Trim().Trim(false);
    if (!text.ToDouble(&value) && !text.ToCDouble(&value))
        return 0;
    return value;
}

double ParseMHz(const wxString& text)
{
    return ParseNumber(text) * MHz;
}

double ParseMHz(const wxTextCtrl* field)
{
    return ParseMHz(field->GetValue());
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

wxString FormatGain(double dB)
{
    return wxString::Format("%g", dB);
}

// wxGridCellBoolEditor's default string values, plus a third one for lime::TriState::Undetermined
const wxString cellTrue = "1";
const wxString cellFalse = "";
const wxString cellUndetermined = "?";

wxString TriStateToCell(TriState state)
{
    switch (state)
    {
    case TriState::Enabled:
        return cellTrue;
    case TriState::Disabled:
        return cellFalse;
    default:
        return cellUndetermined;
    }
}

/// @brief Checkbox renderer that also shows an undetermined state.
class TriStateBoolRenderer : public wxGridCellBoolRenderer
{
  public:
    void Draw(wxGrid& grid, wxGridCellAttr& attr, wxDC& dc, const wxRect& rect, int row, int col, bool isSelected) override
    {
        if (grid.GetCellValue(row, col) != cellUndetermined)
        {
            wxGridCellBoolRenderer::Draw(grid, attr, dc, rect, row, col, isSelected);
            return;
        }
        wxGridCellRenderer::Draw(grid, attr, dc, rect, row, col, isSelected); // background
        wxRendererNative& renderer = wxRendererNative::Get();
        const wxRect box = wxRect(renderer.GetCheckBoxSize(&grid)).CentreIn(rect);
        int flags = wxCONTROL_UNDETERMINED;
        if (!grid.IsEnabled())
            flags |= wxCONTROL_DISABLED;
        renderer.DrawCheckBox(&grid, dc, box, flags);
    }
    wxGridCellRenderer* Clone() const override { return new TriStateBoolRenderer; }
};

/// @brief Checkbox editor that accepts the undetermined state, editing it starts from unchecked.
class TriStateBoolEditor : public wxGridCellBoolEditor
{
  public:
    void BeginEdit(int row, int col, wxGrid* grid) override
    {
        Normalize(row, col, grid);
        wxGridCellBoolEditor::BeginEdit(row, col, grid);
    }
#if wxCHECK_VERSION(3, 1, 4)
    wxGridActivationResult TryActivate(int row, int col, wxGrid* grid, const wxGridActivationSource& actSource) override
    {
        Normalize(row, col, grid);
        return wxGridCellBoolEditor::TryActivate(row, col, grid, actSource);
    }
#endif
    wxGridCellEditor* Clone() const override { return new TriStateBoolEditor; }

  private:
    // wxGridCellBoolEditor asserts on values other than its true/false strings
    static void Normalize(int row, int col, wxGrid* grid)
    {
        if (grid->GetCellValue(row, col) == cellUndetermined)
            grid->GetTable()->SetValue(row, col, cellFalse);
    }
};

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
    rxGUI.channels.resize(ChannelCount(descriptor, TRXDir::Rx));
    txGUI.channels.resize(ChannelCount(descriptor, TRXDir::Tx));

    wxStaticBoxSizer* socBox = new wxStaticBoxSizer(wxVERTICAL, this, descriptor.name);
    wxWindow* base = socBox->GetStaticBox();

    socBox->Add(CreateSharedSection(base), wxSizerFlags().Expand().Border(wxALL, 2));

    wxBoxSizer* directionsSizer = new wxBoxSizer(wxVERTICAL);
    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
        if (!Channels(dir).empty())
            directionsSizer->Add(CreateDirectionSection(base, dir), wxSizerFlags().Expand().Border(wxALL, 2));
    socBox->Add(directionsSizer, wxSizerFlags().Expand());

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

void SOCConfig_view::CreateChannelTable(wxWindow* parent, TRXDir dir)
{
    using Column = DirectionGUI::Column;
    const RFSOCDescriptor& descriptor = device->GetDescriptor().rfSOC.at(socIndex);
    DirectionGUI& gui = Direction(dir);
    const int rows = gui.channels.size();

    const auto paths = descriptor.pathNames.find(dir);
    if (paths != descriptor.pathNames.end())
        for (const auto& pathName : paths->second)
            gui.antennaNames.Add(pathName);

    // Generic gain: a single dB value, distributed by the device among its amplifiers
    const auto ranges = descriptor.gainRange.find(dir);
    if (ranges != descriptor.gainRange.end())
    {
        const auto generic = ranges->second.find(eGainTypes::GENERIC);
        if (generic != ranges->second.end())
            gui.gainRange = generic->second;
    }

    gui.columnTooltips[Column::COL_ENABLE] = "Enable the channel.\nUndetermined if only some parts of the channel are enabled.";
    gui.columnTooltips[Column::COL_ANTENNA] = "Antenna (RF path)";
    gui.columnTooltips[Column::COL_GAIN] = wxString::Format("Generic gain, %g - %g dB", gui.gainRange.min, gui.gainRange.max);
    gui.columnTooltips[Column::COL_LPF] = "Low pass filter bandwidth";
    {
        const auto lpfRange = descriptor.lowPassFilterRange.find(dir);
        if (lpfRange != descriptor.lowPassFilterRange.end())
            gui.columnTooltips[Column::COL_LPF] +=
                wxString::Format(", %s - %s MHz", FormatMHz(lpfRange->second.min), FormatMHz(lpfRange->second.max));
    }
    gui.columnTooltips[Column::COL_NCO] = "NCO frequency offset from the LO. 0 disables the NCO.";
    gui.columnTooltips[Column::COL_CALIBRATE] = "DC/IQ calibration, using the LPF bandwidth (or the sample rate if LPF is 0).\n"
                                                "Immediate mode: checking it runs the calibration now.\n"
                                                "Aggregate mode: the calibration runs after applying the configuration.";
    const wxString liveTooltip = "\nApplied to the device immediately, in both configuration modes.";
    gui.columnTooltips[Column::COL_DC_ENABLE] = "Enable the DC offset corrector." + liveTooltip;
    gui.columnTooltips[Column::COL_DC_I] = "DC offset correction, I component." + liveTooltip;
    gui.columnTooltips[Column::COL_DC_Q] = "DC offset correction, Q component." + liveTooltip;
    gui.columnTooltips[Column::COL_QEC_ENABLE] = "Enable the IQ gain and phase imbalance corrector.\n"
                                                 "Undetermined if only one of them is enabled." +
                                                 liveTooltip;
    gui.columnTooltips[Column::COL_QEC_GAIN] = "IQ gain imbalance correction (dB)." + liveTooltip;
    gui.columnTooltips[Column::COL_QEC_PHASE] = "IQ phase imbalance correction (degrees)." + liveTooltip;

    wxGrid* table = new wxGrid(parent, wxID_ANY);
    gui.table = table;
    table->CreateGrid(rows, Column::COL_COUNT);
    table->DisableDragRowSize();
    table->DisableDragGridSize();
    table->SetRowLabelSize(wxGRID_AUTOSIZE);
    table->SetColLabelSize(wxGRID_AUTOSIZE);
    for (int row = 0; row < rows; ++row)
        table->SetRowLabelValue(row, ChannelLetter(row));

    table->SetColLabelValue(Column::COL_ENABLE, "Enable");
    table->SetColLabelValue(Column::COL_ANTENNA, "Antenna");
    table->SetColLabelValue(Column::COL_GAIN, "Gain (dB)");
    table->SetColLabelValue(Column::COL_LPF, "LPF (MHz)");
    table->SetColLabelValue(Column::COL_NCO, "NCO (MHz)");
    table->SetColLabelValue(Column::COL_CALIBRATE, "Calibrate");
    table->SetColLabelValue(Column::COL_DC_ENABLE, "DC enable");
    table->SetColLabelValue(Column::COL_DC_I, "DCI");
    table->SetColLabelValue(Column::COL_DC_Q, "DCQ");
    table->SetColLabelValue(Column::COL_QEC_ENABLE, "QEC enable");
    table->SetColLabelValue(Column::COL_QEC_GAIN, "QEC (dB)");
    table->SetColLabelValue(Column::COL_QEC_PHASE, "QEC (deg)");

    auto setColumn = [table](int col, wxGridCellEditor* editor, wxGridCellRenderer* renderer, int hAlign) {
        wxGridCellAttr* attr = new wxGridCellAttr;
        attr->SetEditor(editor);
        if (renderer)
            attr->SetRenderer(renderer);
        attr->SetAlignment(hAlign, wxALIGN_CENTER_VERTICAL);
        table->SetColAttr(col, attr);
    };
    setColumn(Column::COL_ENABLE, new TriStateBoolEditor, new TriStateBoolRenderer, wxALIGN_CENTER);
    setColumn(Column::COL_ANTENNA, new wxGridCellChoiceEditor(gui.antennaNames), nullptr, wxALIGN_LEFT);
    setColumn(Column::COL_GAIN, new wxGridCellFloatEditor, nullptr, wxALIGN_RIGHT);
    setColumn(Column::COL_LPF, new wxGridCellFloatEditor, nullptr, wxALIGN_RIGHT);
    setColumn(Column::COL_NCO, new wxGridCellFloatEditor, nullptr, wxALIGN_RIGHT);
    setColumn(Column::COL_CALIBRATE, new wxGridCellBoolEditor, new wxGridCellBoolRenderer, wxALIGN_CENTER);
    setColumn(Column::COL_DC_ENABLE, new TriStateBoolEditor, new TriStateBoolRenderer, wxALIGN_CENTER);
    // DC I/Q editors are set per cell, the range depends on the channel's corrector
    setColumn(Column::COL_DC_I, new wxGridCellNumberEditor, nullptr, wxALIGN_RIGHT);
    setColumn(Column::COL_DC_Q, new wxGridCellNumberEditor, nullptr, wxALIGN_RIGHT);
    setColumn(Column::COL_QEC_ENABLE, new TriStateBoolEditor, new TriStateBoolRenderer, wxALIGN_CENTER);
    setColumn(Column::COL_QEC_GAIN, new wxGridCellFloatEditor, nullptr, wxALIGN_RIGHT);
    setColumn(Column::COL_QEC_PHASE, new wxGridCellFloatEditor, nullptr, wxALIGN_RIGHT);

    // index 0 is usually "NONE"
    const wxString defaultAntenna = gui.antennaNames.empty() ? wxString() : gui.antennaNames[gui.antennaNames.size() > 1 ? 1 : 0];
    for (int row = 0; row < rows; ++row)
    {
        table->SetCellValue(row, Column::COL_ENABLE, cellTrue);
        table->SetCellValue(row, Column::COL_ANTENNA, defaultAntenna);
        table->SetCellValue(row, Column::COL_GAIN, FormatGain(std::clamp(0.0, gui.gainRange.min, gui.gainRange.max)));
        table->SetCellValue(row, Column::COL_LPF, "0");
        table->SetCellValue(row, Column::COL_NCO, "0");
        table->SetCellValue(row, Column::COL_CALIBRATE, cellFalse);
        InitChannelBlocks(row, dir);
    }

    table->AutoSize();
    // fit the longest antenna name and leave room for typing numbers
    int antennaWidth = table->GetColSize(Column::COL_ANTENNA);
    for (const wxString& name : gui.antennaNames)
        antennaWidth = std::max(antennaWidth, table->GetTextExtent(name).x + 32);
    table->SetColSize(Column::COL_ANTENNA, antennaWidth);
    for (int col : { Column::COL_GAIN, Column::COL_LPF, Column::COL_NCO })
        table->SetColSize(col, std::max(table->GetColSize(col), table->GetTextExtent("0000.000000").x));
    for (int col : { Column::COL_DC_I, Column::COL_DC_Q, Column::COL_QEC_GAIN, Column::COL_QEC_PHASE })
        table->SetColSize(col, std::max(table->GetColSize(col), table->GetTextExtent("-000000").x + 24));
    table->ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
    table->InvalidateBestSize();

    table->Bind(wxEVT_GRID_CELL_CHANGED, [this, dir](wxGridEvent& event) { OnChannelCellChanged(dir, event); });

    // wxGrid has no per-column tooltips, show them while hovering a column
    auto showTooltip = [this, table, dir](wxMouseEvent& event, bool isLabel) {
        event.Skip();
        wxWindow* window = static_cast<wxWindow*>(event.GetEventObject());
        const wxPoint position = table->CalcUnscrolledPosition(event.GetPosition());
        const int col = table->XToCol(position.x);
        const int row = isLabel ? -1 : table->YToRow(position.y);
        const wxString tip = GetCellTooltip(dir, row, col);
        if (window->GetToolTipText() != tip)
            window->SetToolTip(tip);
    };
    table->GetGridWindow()->Bind(wxEVT_MOTION, [showTooltip](wxMouseEvent& event) { showTooltip(event, false); });
    table->GetGridColLabelWindow()->Bind(wxEVT_MOTION, [showTooltip](wxMouseEvent& event) { showTooltip(event, true); });
}

void SOCConfig_view::OnChannelCellChanged(TRXDir dir, wxGridEvent& event)
{
    using Column = DirectionGUI::Column;
    DirectionGUI& gui = Direction(dir);
    const int row = event.GetRow();
    if (row < 0 || row >= static_cast<int>(gui.channels.size()))
        return;
    const uint8_t channel = row;
    const wxString name = ChannelName(dir, channel);

    switch (event.GetCol())
    {
    case Column::COL_ENABLE: {
        const bool enable = IsChannelEnabled(dir, channel);
        ApplyImmediate(name + (enable ? " enable" : " disable"),
            [this, channel, dir, enable]() { return device->EnableChannel(socIndex, dir, channel, enable); });
        break;
    }
    case Column::COL_ANTENNA: {
        const int path = GetAntenna(dir, channel);
        if (path < 0)
            return;
        ApplyImmediate(name + " antenna " + gui.antennaNames[path],
            [this, channel, dir, path]() { return device->SetAntenna(socIndex, dir, channel, path); });
        break;
    }
    case Column::COL_GAIN: {
        const double value = GetGain(dir, channel);
        gui.table->SetCellValue(row, Column::COL_GAIN, FormatGain(value));
        ApplyImmediate(wxString::Format("%s gain %g dB", name, value),
            [this, channel, dir, value]() { return device->SetGain(socIndex, dir, channel, eGainTypes::GENERIC, value); });
        break;
    }
    case Column::COL_LPF: {
        const double bandwidth = GetLPF(dir, channel);
        gui.table->SetCellValue(row, Column::COL_LPF, FormatMHz(bandwidth));
        ApplyImmediate(name + " LPF " + FormatMHz(bandwidth) + " MHz",
            [this, channel, dir, bandwidth]() { return device->SetLowPassFilter(socIndex, dir, channel, bandwidth); });
        break;
    }
    case Column::COL_NCO: {
        const double offset = GetNCO(dir, channel);
        gui.table->SetCellValue(row, Column::COL_NCO, FormatMHz(offset));
        ApplyImmediate(name + " NCO " + FormatMHz(offset) + " MHz",
            [this, channel, dir, offset]() { return device->SetNCOFrequency(socIndex, dir, channel, 0, offset, 0); });
        break;
    }
    case Column::COL_CALIBRATE:
        // Immediate mode: setting it runs the calibration. Aggregate mode: it is only a SDRConfig flag
        if (immediateMode && IsCalibrateOnApply(dir, channel))
            Calibrate(channel, dir);
        break;
    case Column::COL_DC_ENABLE: {
        IDCCorrector* dc = gui.channels[channel].dc;
        if (!dc)
            return;
        const bool enable = gui.table->GetCellValue(row, Column::COL_DC_ENABLE) == cellTrue;
        Execute(name + (enable ? " DC corrector enable" : " DC corrector disable"), [dc, enable]() { return dc->Enabled(enable); });
        UpdateChannelBlocks(channel, dir);
        break;
    }
    case Column::COL_DC_I:
    case Column::COL_DC_Q: {
        const ChannelDirectionGUI& blocks = gui.channels[channel];
        IDCCorrector* dc = blocks.dc;
        if (!dc)
            return;
        auto readCell = [&](int col) {
            const long value = std::lround(ParseNumber(gui.table->GetCellValue(row, col)));
            const long clamped = std::clamp(value, std::lround(blocks.dcRange.min), std::lround(blocks.dcRange.max));
            gui.table->SetCellValue(row, col, wxString::Format("%ld", clamped));
            return static_cast<int16_t>(clamped);
        };
        const complex16_t offset(readCell(Column::COL_DC_I), readCell(Column::COL_DC_Q));
        Execute(wxString::Format("%s DC offset I %d, Q %d", name, offset.real(), offset.imag()),
            [dc, offset]() { return dc->SetDCOffset(offset); });
        break;
    }
    case Column::COL_QEC_ENABLE: {
        IQuadratureErrorCorrector* qec = gui.channels[channel].qec;
        if (!qec)
            return;
        const bool enable = gui.table->GetCellValue(row, Column::COL_QEC_ENABLE) == cellTrue;
        Execute(name + (enable ? " QEC enable" : " QEC disable"), [qec, enable]() { return qec->Enabled(enable); });
        UpdateChannelBlocks(channel, dir);
        break;
    }
    case Column::COL_QEC_GAIN:
    case Column::COL_QEC_PHASE: {
        const ChannelDirectionGUI& blocks = gui.channels[channel];
        IQuadratureErrorCorrector* qec = blocks.qec;
        if (!qec)
            return;
        auto readCell = [&](int col, const Range<float>& range) {
            const float value = std::clamp<float>(ParseNumber(gui.table->GetCellValue(row, col)), range.min, range.max);
            gui.table->SetCellValue(row, col, wxString::Format("%g", value));
            return value;
        };
        const float gain = readCell(Column::COL_QEC_GAIN, blocks.qecGainRange);
        const float phase = readCell(Column::COL_QEC_PHASE, blocks.qecPhaseRange);
        Execute(wxString::Format("%s QEC gain %g dB, phase %g deg", name, gain, phase),
            [qec, gain, phase]() { return qec->SetImbalance(gain, phase); });
        break;
    }
    default:
        break;
    }
}

bool SOCConfig_view::IsChannelEnabled(TRXDir dir, uint8_t channel) const
{
    return Direction(dir).table->GetCellValue(channel, DirectionGUI::COL_ENABLE) == cellTrue;
}

int SOCConfig_view::GetAntenna(TRXDir dir, uint8_t channel) const
{
    const DirectionGUI& gui = Direction(dir);
    return gui.antennaNames.Index(gui.table->GetCellValue(channel, DirectionGUI::COL_ANTENNA));
}

double SOCConfig_view::GetGain(TRXDir dir, uint8_t channel) const
{
    const DirectionGUI& gui = Direction(dir);
    const double value = ParseNumber(gui.table->GetCellValue(channel, DirectionGUI::COL_GAIN));
    return std::clamp(value, gui.gainRange.min, gui.gainRange.max);
}

double SOCConfig_view::GetLPF(TRXDir dir, uint8_t channel) const
{
    return ParseMHz(Direction(dir).table->GetCellValue(channel, DirectionGUI::COL_LPF));
}

double SOCConfig_view::GetNCO(TRXDir dir, uint8_t channel) const
{
    return ParseMHz(Direction(dir).table->GetCellValue(channel, DirectionGUI::COL_NCO));
}

bool SOCConfig_view::IsCalibrateOnApply(TRXDir dir, uint8_t channel) const
{
    return Direction(dir).table->GetCellValue(channel, DirectionGUI::COL_CALIBRATE) == cellTrue;
}

IRFChannel* SOCConfig_view::GetRFChannel(uint8_t channel, TRXDir dir) const
{
    if (!device)
        return nullptr;
    try
    {
        IRFSOC* soc = device->GetRFSOC(socIndex);
        return soc ? soc->GetChannel(dir, channel) : nullptr;
    } catch (std::exception&)
    {
        return nullptr;
    }
}

void SOCConfig_view::InitChannelBlocks(uint8_t channel, TRXDir dir)
{
    ChannelDirectionGUI& gui = Channels(dir).at(channel);
    wxGrid* table = Direction(dir).table;
    IRFChannel* rfChannel = GetRFChannel(channel, dir);
    try
    {
        gui.dc = rfChannel ? rfChannel->GetDCCorrector() : nullptr;
        gui.qec = rfChannel ? rfChannel->GetQuadratureErrorCorrector() : nullptr;
        if (gui.dc)
            gui.dcRange = gui.dc->GetRange();
        if (gui.qec)
        {
            gui.qecGainRange = gui.qec->GetGainRange();
            gui.qecPhaseRange = gui.qec->GetPhaseRange();
        }
    } catch (std::exception& e)
    {
        report(ChannelName(dir, channel) + " blocks: " + e.what(), true);
        gui.dc = nullptr;
        gui.qec = nullptr;
    }

    if (gui.dc)
        for (int col : { DirectionGUI::COL_DC_I, DirectionGUI::COL_DC_Q })
            table->SetCellEditor(
                channel, col, new wxGridCellNumberEditor(std::lround(gui.dcRange.min), std::lround(gui.dcRange.max)));

    for (int col : { DirectionGUI::COL_DC_ENABLE, DirectionGUI::COL_DC_I, DirectionGUI::COL_DC_Q })
        SetCellEditable(dir, channel, col, gui.dc != nullptr);
    for (int col : { DirectionGUI::COL_QEC_ENABLE, DirectionGUI::COL_QEC_GAIN, DirectionGUI::COL_QEC_PHASE })
        SetCellEditable(dir, channel, col, gui.qec != nullptr);
    UpdateChannelBlocks(channel, dir);
}

void SOCConfig_view::SetCellEditable(TRXDir dir, int row, int col, bool editable)
{
    wxGrid* table = Direction(dir).table;
    table->SetReadOnly(row, col, !editable);
    table->SetCellBackgroundColour(row, col, wxSystemSettings::GetColour(editable ? wxSYS_COLOUR_WINDOW : wxSYS_COLOUR_BTNFACE));
    table->RefreshBlock(row, col, row, col);
}

wxString SOCConfig_view::GetCellTooltip(TRXDir dir, int row, int col) const
{
    const DirectionGUI& gui = Direction(dir);
    if (col < 0 || col >= DirectionGUI::COL_COUNT)
        return wxString();
    wxString tip = gui.columnTooltips[col];
    if (row < 0 || row >= static_cast<int>(gui.channels.size()))
        return tip;

    const ChannelDirectionGUI& channel = gui.channels[row];
    auto range = [](const Range<float>& r, const char* units) {
        return wxString::Format("\nRange: %g - %g%s", r.min, r.max, units);
    };
    switch (col)
    {
    case DirectionGUI::COL_DC_ENABLE:
    case DirectionGUI::COL_DC_I:
    case DirectionGUI::COL_DC_Q:
        if (!channel.dc)
            return tip + "\nNot available on this channel";
        return col == DirectionGUI::COL_DC_ENABLE ? tip : tip + range(channel.dcRange, "");
    case DirectionGUI::COL_QEC_ENABLE:
        return channel.qec ? tip : tip + "\nNot available on this channel";
    case DirectionGUI::COL_QEC_GAIN:
        return channel.qec ? tip + range(channel.qecGainRange, " dB") : tip + "\nNot available on this channel";
    case DirectionGUI::COL_QEC_PHASE:
        return channel.qec ? tip + range(channel.qecPhaseRange, " deg") : tip + "\nNot available on this channel";
    default:
        return tip;
    }
}

void SOCConfig_view::UpdateChannelBlocks(uint8_t channel, TRXDir dir)
{
    const ChannelDirectionGUI& gui = Channels(dir).at(channel);
    wxGrid* table = Direction(dir).table;
    try
    {
        if (gui.dc)
        {
            const TriState state = gui.dc->IsEnabled();
            // undetermined: the offset may still be in use, keep it editable
            const bool enabled = state != TriState::Disabled;
            const complex16_t offset = gui.dc->GetDCOffset();
            table->SetCellValue(channel, DirectionGUI::COL_DC_ENABLE, TriStateToCell(state));
            table->SetCellValue(channel, DirectionGUI::COL_DC_I, wxString::Format("%d", offset.real()));
            table->SetCellValue(channel, DirectionGUI::COL_DC_Q, wxString::Format("%d", offset.imag()));
            SetCellEditable(dir, channel, DirectionGUI::COL_DC_I, enabled);
            SetCellEditable(dir, channel, DirectionGUI::COL_DC_Q, enabled);
        }
        if (gui.qec)
        {
            const TriState state = gui.qec->IsEnabled();
            table->SetCellValue(channel, DirectionGUI::COL_QEC_ENABLE, TriStateToCell(state));
            // undetermined: one of the corrections is active, keep the values editable
            SetCellEditable(dir, channel, DirectionGUI::COL_QEC_GAIN, state != TriState::Disabled);
            SetCellEditable(dir, channel, DirectionGUI::COL_QEC_PHASE, state != TriState::Disabled);

            float gain = 0;
            float phase = 0;
            if (gui.qec->GetImbalance(gain, phase) == OpStatus::Success)
            {
                table->SetCellValue(channel, DirectionGUI::COL_QEC_GAIN, wxString::Format("%g", gain));
                table->SetCellValue(channel, DirectionGUI::COL_QEC_PHASE, wxString::Format("%g", phase));
            }
        }
    } catch (std::exception& e)
    {
        report("Read " + ChannelName(dir, channel) + " blocks: " + e.what(), true);
    }
}

wxSizer* SOCConfig_view::CreateDirectionSection(wxWindow* parent, TRXDir dir)
{
    wxStaticBoxSizer* box = new wxStaticBoxSizer(wxVERTICAL, parent, DirName(dir) + " channels");
    CreateChannelTable(box->GetStaticBox(), dir);
    box->Add(Direction(dir).table, wxSizerFlags().Border(wxALL, 2));
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
    if (immediateMode)
        Execute(what, operation);
}

void SOCConfig_view::Execute(const wxString& what, const std::function<OpStatus()>& operation)
{
    if (!device)
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
    // Reference clock source exists only in SDRConfig, there is no individual API for it
    refClockSource->Enable(!immediate);
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

    double bandwidth = GetLPF(dir, channel);
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
    // calibration changes the DC and IQ correction
    UpdateChannelBlocks(channel, dir);
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
        for (uint8_t i = 0; i < Channels(dir).size(); ++i)
        {
            ChannelConfig::Direction& d = config.channel[i].GetDirection(dir);
            d.centerFrequency = dir == TRXDir::Rx ? rxFrequency : txFrequency;
            d.enabled = IsChannelEnabled(dir, i);
            d.sampleRate = rate;
            d.oversample = ovs;
            d.path = std::max(0, GetAntenna(dir, i));
            d.gain[eGainTypes::GENERIC] = GetGain(dir, i);
            d.lpf = GetLPF(dir, i);
            d.NCOoffset = GetNCO(dir, i);
            d.calibrate = IsCalibrateOnApply(dir, i) ? CalibrationFlag::DCIQ : CalibrationFlag::NONE;
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
        DirectionGUI& gui = Direction(dir);
        for (uint8_t i = 0; i < gui.channels.size(); ++i)
        {
            const wxString name = ChannelName(dir, i);

            tryRead(name + " enable", [this, &gui, i, dir]() {
                IRFChannel* rfChannel = GetRFChannel(i, dir);
                if (rfChannel)
                    gui.table->SetCellValue(i, DirectionGUI::COL_ENABLE, TriStateToCell(rfChannel->IsEnabled()));
            });
            tryRead(name + " antenna", [this, &gui, i, dir]() {
                const unsigned path = device->GetAntenna(socIndex, dir, i);
                if (path < gui.antennaNames.size())
                    gui.table->SetCellValue(i, DirectionGUI::COL_ANTENNA, gui.antennaNames[path]);
            });
            tryRead(name + " gain", [this, &gui, i, dir]() {
                double value = 0;
                if (device->GetGain(socIndex, dir, i, eGainTypes::GENERIC, value) == OpStatus::Success)
                    gui.table->SetCellValue(i, DirectionGUI::COL_GAIN, FormatGain(value));
            });
            tryRead(name + " LPF", [this, &gui, i, dir]() {
                gui.table->SetCellValue(i, DirectionGUI::COL_LPF, FormatMHz(device->GetLowPassFilter(socIndex, dir, i)));
            });
            tryRead(name + " NCO", [this, &gui, i, dir]() {
                double phase = 0;
                gui.table->SetCellValue(i, DirectionGUI::COL_NCO, FormatMHz(device->GetNCOFrequency(socIndex, dir, i, 0, phase)));
            });
            UpdateChannelBlocks(i, dir);
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
    wxWindowUpdateLocker noUpdates(this);
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
