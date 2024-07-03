#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif //__BORLANDC__

#include <wx/msgdlg.h>

#include "chips/LMS7002M/lms7002_mainPanel.h"

#include "limeGUIFrame.h"
#include "dlgAbout.h"
#include "fftviewer_frFFTviewer.h"
#include "chips/ADF4002/ADF4002_wxgui.h"
#include "chips/Si5351C/Si5351C_wxgui.h"
#include "chips/CDCM6208/CDCM6208_wxgui.h"
#include "LMS_Programming/LMS_Programming_wxgui.h"
#include "utility/pnlMiniLog.h"
#include "FPGAcontrols_wxgui.h"
#include "utility/SPI_wxgui.h"
#include <wx/string.h>
#include "utility/dlgDeviceInfo.h"
#include <functional>
#include "boards/pnlBoardControls.h"
#include "protocols/LMSBoards.h"
#include "utility/SPI_wxgui.h"
#include "events.h"
#include "ISOCPanel.h"
#include "SDRConfiguration_view.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "DeviceTreeNode.h"
#include "limesuiteng/Logger.h"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

static constexpr int controlColumn = 1;

limeGUIFrame* limeGUIFrame::obj_ptr = nullptr;

int limeGUIFrame::m_lmsSelection = 0;

void limeGUIFrame::OnGlobalLogEvent(const lime::LogLevel level, const std::string& message)
{
    if (obj_ptr == nullptr || obj_ptr->mMiniLog == nullptr)
        return;
    wxCommandEvent evt;
    evt.SetString(message);
    evt.SetEventType(LOG_MESSAGE);
    evt.SetInt(static_cast<int>(level));
    wxPostEvent(obj_ptr, evt);
}

struct DeviceTreeItemData : public wxTreeItemData {
    DeviceTreeItemData(const std::shared_ptr<DeviceTreeNode> soc)
        : wxTreeItemData()
        , soc(soc)
    {
    }

    ~DeviceTreeItemData() override
    {
        if (gui)
            gui->Destroy();
    }

    ISOCPanel* gui{ nullptr };
    std::shared_ptr<DeviceTreeNode> soc;
};

static bool FoundDevice(const wxString& criteria, DeviceHandle& outHandle, uint32_t& outIndex)
{
    std::vector<lime::DeviceHandle> allDevices;
    allDevices = lime::DeviceRegistry::enumerate();
    for (uint32_t i = 0; i < allDevices.size(); ++i)
    {
        const lime::DeviceHandle& device = allDevices[i];

        if (device.name.find(criteria) != std::string::npos)
        {
            outHandle = device;
            outIndex = i;
            return true;
        }
    }
    return false;
}

static std::vector<std::string> SplitString(const std::string& s, const char delimiter)
{
    std::vector<std::string> list;

    size_t right = 0;
    size_t left = 0;

    while ((right = s.find(delimiter, left)) != std::string::npos)
    {
        std::string str = s.substr(left, right - left);
        list.push_back(str);
        left = right + 1;
    }
    if (left != s.size())
        list.push_back(s.substr(left));
    return list;
}

static bool GetTreeNode(const wxTreeCtrl* treeControl,
    const wxTreeItemId current,
    wxTreeItemIdValue& cookie,
    const std::vector<std::string>& path,
    const uint32_t currentDepth,
    wxTreeItemId& result)
{
    if (treeControl->GetItemText(current).ToStdString() != path.at(currentDepth))
        return false;

    if (currentDepth + 1 == path.size())
    {
        result = current;
        return true;
    }

    wxTreeItemId child = treeControl->GetFirstChild(current, cookie);
    while (child.IsOk())
    {
        if (GetTreeNode(treeControl, child, cookie, path, currentDepth + 1, result))
            return true;
        child = treeControl->GetNextSibling(child);
    }
    return false;
}

static bool GetTreeNode(wxTreeCtrl* treeControl, const wxString& branch, wxTreeItemId& result)
{
    if (branch.IsEmpty())
        return false;
    const std::vector<std::string> nodes = SplitString(branch.ToStdString(), '/');

    wxTreeItemId root = treeControl->GetRootItem();
    wxTreeItemIdValue cookie;
    wxTreeItemId deviceLevelNode = treeControl->GetFirstChild(root, cookie);

    return GetTreeNode(treeControl, deviceLevelNode, cookie, nodes, 0, result);
}

limeGUIFrame::limeGUIFrame(wxWindow* parent, const AppArgs& appArgs)
    : wxFrame(parent, wxNewId(), _("Lime Suite NG"))
    , lmsControl(nullptr)
    , mMiniLog(new pnlMiniLog(this, wxNewId()))
    , fftviewer(new fftviewer_frFFTviewer(this, wxNewId()))
    , mbar(new wxMenuBar(0))
    , fileMenu(new wxMenu())
    , mnuModules(new wxMenu())
    , helpMenu(new wxMenu())
    , contentSizer(new wxFlexGridSizer(0, 2, 0, 0))
    , mContent(nullptr)
    , pnlDeviceConnection(new DeviceConnectionPanel(this, wxNewId()))
{
    obj_ptr = this;

    wxMenuItem* menuFileQuit = new wxMenuItem(
        fileMenu, idMenuQuit, wxString(wxT("&Quit")) + wxT('\t') + wxT("Alt+F4"), wxT("Quit the application"), wxITEM_NORMAL);
    fileMenu->Append(menuFileQuit);
    Connect(menuFileQuit->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(limeGUIFrame::OnQuit));
    mbar->Append(fileMenu, wxT("&File"));

    mbar->Append(mnuModules, wxT("Modules"));

    wxMenuItem* menuHelpAbout = new wxMenuItem(helpMenu,
        idMenuAbout,
        wxString(wxT("&About")) + wxT('\t') + wxT("F1"),
        wxT("Show info about this application"),
        wxITEM_NORMAL);
    helpMenu->Append(menuHelpAbout);
    Connect(menuHelpAbout->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(limeGUIFrame::OnAbout));

    mbar->Append(helpMenu, wxT("&Help"));
    SetMenuBar(mbar);

    statusBar = CreateStatusBar(3, wxSTB_DEFAULT_STYLE, wxNewId());
    const int mainColumns = 1;
    mainSizer = new wxFlexGridSizer(mainColumns, 0, 0);
    mainSizer->AddGrowableCol(0);
    mainSizer->AddGrowableRow(1);
    mainSizer->SetFlexibleDirection(wxBOTH);
    mainSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    mainSizer->Add(pnlDeviceConnection, 0, wxEXPAND, 0);

    m_scrolledWindow1 = new wxScrolledWindow(this, wxNewId(), wxDefaultPosition, wxDefaultSize, wxHSCROLL | wxVSCROLL);
    m_scrolledWindow1->SetScrollRate(5, 5);

    contentSizer->AddGrowableCol(1);
    contentSizer->AddGrowableRow(0);
    contentSizer->SetFlexibleDirection(wxBOTH);
    contentSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    deviceTree = new wxTreeCtrl(
        m_scrolledWindow1, wxNewId(), wxPoint(0, 0), wxSize(200, 385), wxTR_HAS_BUTTONS | wxTR_SINGLE | wxTR_ROW_LINES);
    contentSizer->Add(deviceTree, 0, wxEXPAND, 0);

    m_scrolledWindow1->SetSizerAndFit(contentSizer);
    mainSizer->Add(m_scrolledWindow1, 4, wxEXPAND, 5);

    mainSizer->Add(mMiniLog, 1, wxEXPAND, 5);

    //SetSizer( mainSizer );
    SetSizerAndFit(mainSizer);
    // Layout();
    // Fit();

#ifndef __unix__
    SetIcon(wxIcon(_("aaaaAPPicon")));
#endif

    AddModule(fftviewer, "fftviewer"s);

    SPI_wxgui* spigui = new SPI_wxgui(this, wxNewId());
    AddModule(spigui, "SPI"s);

    boardControlsGui = new pnlBoardControls(this, wxNewId());
    AddModule(boardControlsGui, "Board controls"s);

    programmer = new LMS_Programming_wxgui(this, wxNewId());
    AddModule(programmer, "Programming"s);

    int x, y1, y2;
    m_scrolledWindow1->GetVirtualSize(&x, &y1);
    mMiniLog->GetSize(nullptr, &y2);

    wxCommandEvent event;
    //OnControlBoardConnect(event);

    //UpdateConnections(lmsControl);

    // mnuCacheValues->Check(false);
    const int statusWidths[] = { -1, -3, -3 };
    statusBar->SetStatusWidths(3, statusWidths);
    Bind(limeEVT_SDR_HANDLE_SELECTED, wxCommandEventHandler(limeGUIFrame::OnDeviceHandleChange), this);
    Connect(LOG_MESSAGE, wxCommandEventHandler(limeGUIFrame::OnLogMessage), nullptr, this);
    lime::registerLogHandler(&limeGUIFrame::OnGlobalLogEvent);

    deviceTree->Bind(
        wxEVT_TREE_SEL_CHANGED, wxTreeEventHandler(limeGUIFrame::DeviceTreeSelectionChanged), this, deviceTree->GetId());

    DeviceHandle handle;
    uint32_t initialIndex;
    if (!appArgs.device.IsEmpty() && FoundDevice(appArgs.device, handle, initialIndex))
    {
        pnlDeviceConnection->SetSelection(initialIndex);
        wxCommandEvent event(limeEVT_SDR_HANDLE_SELECTED, GetId());
        event.SetString(handle.Serialize());
        ProcessWindowEvent(event);

        wxTreeItemId searchTreeID;
        if (GetTreeNode(deviceTree, appArgs.searchTree, searchTreeID))
        {
            deviceTree->SetFocusedItem(searchTreeID);
        }
    }
}

limeGUIFrame::~limeGUIFrame()
{
    for (auto iter : mModules)
        iter.second->Destroy();

    OnDeviceDisconnect();

    Disconnect(idMenuQuit, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(limeGUIFrame::OnQuit));
    Disconnect(idMenuAbout, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(limeGUIFrame::OnAbout));
    Disconnect(LOG_MESSAGE, wxCommandEventHandler(limeGUIFrame::OnLogMessage), nullptr, this);

    obj_ptr = nullptr;
}

void limeGUIFrame::OnClose(wxCloseEvent& event)
{
    Destroy();
}

void limeGUIFrame::OnQuit(wxCommandEvent& event)
{
    Destroy();
}

void limeGUIFrame::OnAbout(wxCommandEvent& event)
{
    dlgAbout dlg(this);
    dlg.ShowModal();
}

void limeGUIFrame::UpdateConnections(SDRDevice* device)
{
    for (auto iter : mModules)
    {
        iter.second->Initialize(device);
    }
}

void limeGUIFrame::OnDeviceDisconnect()
{
    if (fftviewer)
        fftviewer->StopStreaming();
    if (lmsControl)
    {
        const SDRDescriptor& info = lmsControl->GetDescriptor();
        statusBar->SetStatusText(_("Control port: Not Connected"), controlColumn);
        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetInt(static_cast<int>(lime::LogLevel::Info));
        evt.SetString("Disconnected: "s + info.name);
        wxPostEvent(this, evt);
        UpdateConnections(nullptr);
        lime::DeviceRegistry::freeDevice(lmsControl);
        lmsControl = nullptr;
    }

    deviceTree->DeleteAllItems();
    mContent = nullptr;
}

void CreateBranch(wxTreeCtrl* treeRoot, wxTreeItemId parentId, const std::shared_ptr<DeviceTreeNode> node)
{
    wxTreeItemId branchId = treeRoot->AppendItem(parentId, node->name, 0, 0, new DeviceTreeItemData(node));
    if (node->children.size() == 0)
        return;

    for (const auto& soc : node->children)
        CreateBranch(treeRoot, branchId, soc);
}

void FillDeviceTree(wxTreeCtrl* root, lime::SDRDevice* device, wxWindow* parentWindow)
{
    if (!root || !device)
        return;

    SDRConfiguration_view* sdrUI = new SDRConfiguration_view(parentWindow, wxNewId());
    sdrUI->Setup(device);
    sdrUI->Hide();

    std::shared_ptr<DeviceTreeNode> node = device->GetDescriptor().socTree;
    DeviceTreeItemData* treeRootData = new DeviceTreeItemData(node);
    treeRootData->gui = sdrUI;
    wxTreeItemId rootId = root->AddRoot(node->name, 0, 0, treeRootData);

    for (const auto& soc : node->children)
        CreateBranch(root, rootId, soc);
    root->ExpandAll();
    root->SelectItem(rootId, true);
}

void limeGUIFrame::OnDeviceHandleChange(wxCommandEvent& event)
{
    OnDeviceDisconnect();
    // event.GetString() is the target device handle text
    try
    {
        if (event.GetString().length() == 0)
            return;

        DeviceHandle handle(event.GetString().ToStdString());
        lmsControl = lime::DeviceRegistry::makeDevice(handle);

        if (!lmsControl)
        {
            wxMessageBox("Failed to connect to: " + event.GetString(), wxT("Connection error"), wxICON_ERROR);
            return;
        }

        //bind callback for spi data logging
        const SDRDescriptor& info = lmsControl->GetDescriptor();
        wxString controlDev = _("Device: ");
        controlDev.Append(handle.ToString());
        double refClk = lmsControl->GetClockFreq(LMS_CLOCK_REF,
            0); // use reference clock of the 0th channel
        controlDev.Append(wxString::Format(_(" FW:%s HW:%s Protocol:%s GW:%s.%s Ref Clk: %1.2f MHz"),
            info.firmwareVersion,
            info.hardwareVersion,
            info.protocolVersion,
            info.gatewareVersion,
            info.gatewareRevision,
            refClk / 1e6));
        statusBar->SetStatusText(controlDev, controlColumn);

        FillDeviceTree(deviceTree, lmsControl, m_scrolledWindow1);
        wxTreeEvent eee(wxEVT_TREE_SEL_CHANGED, deviceTree, deviceTree->GetFocusedItem());
        DeviceTreeSelectionChanged(eee);

        wxCommandEvent evt;
        evt.SetEventType(LOG_MESSAGE);
        evt.SetInt(static_cast<int>(lime::LogLevel::Info));
        evt.SetString(_("Connected ") + controlDev);
        wxPostEvent(this, evt);
        UpdateConnections(lmsControl);

        Fit();
    } catch (std::runtime_error& e)
    {
        lime::error("Failed to connect "s + e.what());
    }
}

void limeGUIFrame::OnLogMessage(wxCommandEvent& event)
{
    if (mMiniLog)
        mMiniLog->HandleMessage(event);
}

void limeGUIFrame::AddModule(IModuleFrame* module, const std::string& title)
{
    wxWindowID moduleId = module->GetId();
    lime::debug("Add module %i", moduleId);
    wxMenuItem* item;
    item = new wxMenuItem(mnuModules, moduleId, title, wxEmptyString, wxITEM_NORMAL);
    mnuModules->Append(item);

    mModules[moduleId] = module;
    module->Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(limeGUIFrame::OnModuleClose), nullptr, this);
    Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(limeGUIFrame::OnShowModule));
}

void limeGUIFrame::RemoveModule(IModuleFrame* module)
{
}

void limeGUIFrame::OnModuleClose(wxCloseEvent& event)
{
    lime::debug("Close module %i", event.GetId());
    IModuleFrame* module = mModules.at(event.GetId());
    if (module)
    {
        module->Show(false);
    }
}

void limeGUIFrame::OnShowModule(wxCommandEvent& event)
{
    lime::debug("show module %i", event.GetId());
    IModuleFrame* module = mModules.at(event.GetId());
    if (module) //it's already opened
    {
        module->Initialize(lmsControl);
        module->Show(true);
        module->Iconize(false); // restore the window if minimized
        module->SetFocus(); // focus on my window
        module->Raise(); // bring window to front
    }
}

ISOCPanel* CreateGUI(wxWindow* parent, eDeviceTreeNodeClass DeviceTreeNodeClass, void* socPtr)
{
    switch (DeviceTreeNodeClass)
    {
    case eDeviceTreeNodeClass::ADF4002: {
        ADF4002_wxgui* adfPanel = new ADF4002_wxgui(parent, wxNewId());
        adfPanel->Initialize(reinterpret_cast<lime::ADF4002*>(socPtr));
        return adfPanel;
    }
    case eDeviceTreeNodeClass::CDCM6208: {
        CDCM6208_panelgui* cdcmPanel = new CDCM6208_panelgui(parent, wxNewId());
        cdcmPanel->Initialize(reinterpret_cast<CDCM_Dev*>(socPtr));
        return cdcmPanel;
    }
    case eDeviceTreeNodeClass::LMS7002M: {
        lms7002_mainPanel* lmsPanel = new lms7002_mainPanel(parent, wxNewId());
        lmsPanel->Initialize(reinterpret_cast<LMS7002M*>(socPtr));
        return lmsPanel;
    }
    case eDeviceTreeNodeClass::SDRDevice: {
        SDRConfiguration_view* sdrPanel = new SDRConfiguration_view(parent, wxNewId());
        sdrPanel->Setup(reinterpret_cast<SDRDevice*>(socPtr));
        sdrPanel->Hide();
        return sdrPanel;
    }
    default:
        lime::warning("Unrecognized device class(%u)", static_cast<uint8_t>(DeviceTreeNodeClass));
        return nullptr;
    }
}

void limeGUIFrame::DeviceTreeSelectionChanged(wxTreeEvent& event)
{
    DeviceTreeItemData* item = reinterpret_cast<DeviceTreeItemData*>(deviceTree->GetItemData(event.GetItem()));
    if (item->gui != nullptr && mContent == item->gui)
    {
        return;
    }

    if (item->gui == nullptr)
        item->gui = CreateGUI(m_scrolledWindow1, item->soc->DeviceTreeNodeClass, item->soc->ptr);

    if (mContent && mContent != item->gui)
    {
        contentSizer->Remove(1);
        mContent->Hide();
        mContent = nullptr;
    }

    mContent = item->gui;
    if (!mContent)
        return;
    mContent->Show();
    contentSizer->Add(mContent, 0, 0, 5);
    contentSizer->Layout();
    m_scrolledWindow1->SetSizerAndFit(contentSizer);
    Layout();
    Fit();
}
