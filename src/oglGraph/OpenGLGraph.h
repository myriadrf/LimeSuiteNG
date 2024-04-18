/**
@file	OpenGLGraph.h
@author Lime Microsystems
@brief	Header for OpenGLGraph.h
*/

#ifndef OPENGL_GRAPH
#define OPENGL_GRAPH

#include <wx/glcanvas.h>
#include <wx/menu.h>

class wxTimer;
class wxTimerEvent;

#include <string>
#include <vector>

enum eOGLGMouseButton { OGLG_LEFT, OGLG_RIGHT, OGLG_MIDDLE };

enum eOGLGActionState {
    OGLG_IDLE,
    OGLG_ZOOMIN,
    OGLG_PAN,
    OGLG_SCALE,
    OGLG_ADD_MARKER,
    OGLG_MOVE_MARKER,
    OGLG_REMOVE_MARKER,
    OGLG_SEARCH_PEAK
};

enum { OGLG_FIT = 3000, OGLG_LOCKASPECT, OGLG_HELP_MOUSE, OGLG_ADD_MARK, OGLG_SHOW_MARKERS_MENU, OGLG_RESET };

enum eDrawingMode { GLG_POINTS = 0, GLG_LINE };

template<class T> struct sRect {
    sRect(T X1, T X2, T Y1, T Y2)
    {
        x1 = X1;
        x2 = X2;
        y1 = Y1;
        y2 = Y2;
    }
    T x1, x2, y1, y2;
    void set(T X1, T X2, T Y1, T Y2)
    {
        x1 = X1;
        x2 = X2;
        y1 = Y1;
        y2 = Y2;
    }
};

/** @brief Structure for storing color information as 4 floats */
struct GLG_color {
    /**
      @brief Construct a new GLG_color object using a specified color.
      @param rgba An integer representing the RGBA color values. First 8 bits are the red value, next 8 are green, then blue, then the alpha.
     */
    GLG_color(unsigned int rgba)
    {
        red = (rgba >> 24) / 255.0;
        green = ((rgba >> 16) & 0xFF) / 255.0;
        blue = ((rgba >> 8) & 0xFF) / 255.0;
        alpha = (rgba & 0xFF) / 255.0;
    }

    /** @brief Construct a new GLG_color object using a default color of #797979FF. */
    GLG_color()
        : red(0.5)
        , green(0.5)
        , blue(0.5)
        , alpha(1.0)
    {
    }

    /**
      @brief Get the stored color as a packed integer value.
      @return The packed integer value.
     */
    unsigned int getColor4b()
    {
        unsigned int color = 0;
        color |= static_cast<unsigned int>(255 * red) << 24;
        color |= static_cast<unsigned int>(255 * green) << 16;
        color |= static_cast<unsigned int>(255 * blue) << 8;
        color |= static_cast<unsigned int>(255 * alpha);
        return color;
    }

    float red; ///< The red component of the color
    float green; ///< The green component of the color
    float blue; ///< The blue component of the color
    float alpha; ///< The alpha component of the color
};

/// @brief Color data series
class cDataSerie
{
  public:
    /// @brief Constructor for the class.
    /// Initializes it with the color #000000FF and an initial allocation size of 10.
    cDataSerie()
        : size(0)
        , allocatedSize(0)
        , vboIndex(0)
        , color(0x000000FF)
        , visible(true)
        , modified(true)
        , values(nullptr)
    {
        Initialize(10);
    };

    /// @brief Destructor for the class.
    /// Frees up @ref values upon being called.
    ~cDataSerie()
    {
        if (values != nullptr)
        {
            delete[] values;
            values = nullptr;
        }
    };

    /// @brief Assigns the given values into the series.
    /// @param xserie The x values to assign.
    /// @param yserie The y values to assign.
    /// @param count The amount of values to assign.
    void AssignValues(float* xserie, float* yserie, unsigned int count)
    {
        if (2 * count > allocatedSize && count > 0)
            Initialize(2 * count);
        if (xserie && yserie)
        {
            int index = 0;
            for (unsigned i = 0; i < count; ++i)
            {
                values[index] = xserie[i];
                ++index;
                values[index] = yserie[i];
                ++index;
            }
        }
        modified = true;
        size = count;
    }

    /// @brief Assigns the given values into the series.
    /// @param valuesXY The x and y value pairs to assign.
    /// @param length The length of the memory array to assign.
    void AssignValues(float* valuesXY, unsigned int length)
    {
        if (length > allocatedSize && length > 0)
            Initialize(length);
        if (valuesXY)
            memcpy(values, valuesXY, (length) * sizeof(float));
        modified = true;
        size = length / 2;
    }

    /// @brief Clears the series.
    void Clear()
    {
        if (values != nullptr)
        {
            delete[] values;
            values = nullptr;
        }
        size = 0;
        allocatedSize = 0;
        modified = true;
    }

    /// @brief Sets up the series, capable of holding @p count elements.
    /// @param count The amount of elements to allocate memory for.
    void Initialize(unsigned int count)
    {
        Clear();
        size = count;
        values = new float[2 * count];
        memset(values, 0, sizeof(float) * 2 * count);
        allocatedSize = count;
    }

    /// @brief Adds a single X and Y value into the series.
    /// @param x The X value to add.
    /// @param y The Y value to add.
    void AddXY(float x, float y)
    {
        if (size < allocatedSize)
        {
            values[2 * size] = x;
            values[2 * size + 1] = y;
            ++size;
        }
        else
        {
            float* newVal = new float[size * 2];
            allocatedSize = size * 2;
            memcpy(newVal, values, sizeof(float) * size * 2);
            delete[] values;
            values = newVal;
            values[2 * size] = x;
            values[2 * size + 1] = y;
            ++size;
        }
        modified = true;
    }

    unsigned int size; ///< The currently used size.
    unsigned int allocatedSize; ///< The currently allocated size.
    unsigned int vboIndex; ///< The vertex buffer object index.
    GLG_color color; ///< The color to be used.
    bool visible; ///< Whether the series is visible or not.
    bool modified; ///< Whether the series has been modified since it was last checked.

    float* values; ///< The value array.
};

struct GLG_settings {
    GLG_settings();
    std::string title;
    std::string titleXaxis;
    std::string titleYaxis;

    std::string xUnits;
    std::string yUnits;

    bool drawGridX;
    bool drawGridY;
    bool drawTitle;
    bool drawTitleX;
    bool drawTitleY;

    int windowWidth;
    int windowHeight;
    int dataViewWidth;
    int dataViewHeight;

    int marginTop;
    int marginBottom;
    int marginLeft;
    int marginRight;
    bool useVBO;

    GLG_color backgroundColor;
    GLG_color titlesColor;
    GLG_color dataViewBackgroundColor;
    GLG_color dataViewPerimeterColor;
    GLG_color gridColor;
    eDrawingMode graphType;

    sRect<double> visibleArea;

    float gridXstart;
    float gridYstart;
    float gridXspacing;
    float gridYspacing;
    int gridXlines;
    int gridYlines;
    int gridXprec;
    int gridYprec;
    int gridValuesHeight;

    float pointsSize;
    float fontSize;

    bool staticGrid;
    bool lock_aspect;
    bool markersEnabled;
    float gridXoffset;
};

struct OGLMarker {
    OGLMarker()
        : color(0x007F00FF)
    {
        used = false;
        posX = 0;
        posY = 0;
        size = 10;
        iposX = 0;
        iposY = 0;
        dataValueIndex = 0;
        id = 0;
        show = true;
    }
    float posX, posY; //data view position
    int iposX, iposY; //window position
    float size;
    GLG_color color;
    int dataValueIndex;
    int id;
    bool used;
    bool show;
};

class GLFont;
class dlgMarkers;

class OpenGLGraph : public wxGLCanvas
{
    friend class dlgMarkers;

  public:
    static const int GLCanvasAttributes[8];

    OpenGLGraph(wxWindow* parent,
        wxWindowID id,
        const wxPoint& pos,
        const wxSize& size,
        long style,
        const wxString& name = wxEmptyString,
        const int* args = GLCanvasAttributes);
    virtual ~OpenGLGraph();

    bool Initialize(int width, int height);
    void Resize(int w, int h);

    void AddSerie(cDataSerie* serie);
    void RemoveSeries(unsigned int i);

    void SetInitialDisplayArea(float minx, float maxx, float miny, float maxy);
    void SetDisplayArea(float minx, float maxx, float miny, float maxy);
    void ZoomY(float centerY, float spanY);
    void ZoomX(float centerX, float spanX);
    void Zoom(float centerX, float centerY, float spanX, float spanY);
    void ZoomRect(int x1, int x2, int y1, int y2);
    void Pan(float dx, float dy);

    void SetDrawingMode(eDrawingMode mode);
    void Draw();

    bool SaveConfig(char* file);
    bool LoadConfig(char* file);

    std::vector<cDataSerie*> series;

    GLG_settings settings;
    void SettingsChanged();

    void OnMouseDown(int mouseButton, int X, int Y);
    void OnMouseUp(int mouseButton, int X, int Y);
    void OnMouseMove(int X, int Y);
    void ResetView();
    void Fit();

    void SetMarker(int id, float xValue, bool enabled, bool show);
    void GetMarker(int id, float& xValue, float& yValue, bool& enabled, bool& show);
    int AddMarker(int posX);
    int AddMarkerAtValue(float xValue);
    void RemoveMarker();
    void RemoveMarker(int id);
    void MoveMarker(int markerID, int posX);
    void ChangeMarker(int markerID, float xValue);
    bool SearchPeak();

    void SetInfoMessage(const char* msg, unsigned int index);

    // events
    void render(wxPaintEvent& evt);
    void resized(wxSizeEvent& evt);
    void mouseMoved(wxMouseEvent& event);
    void mouseWheelMoved(wxMouseEvent& event);
    void mouseReleased(wxMouseEvent& event);
    void rightClick(wxMouseEvent& event);
    void leftClick(wxMouseEvent& event);
    void middleClick(wxMouseEvent& event);
    void mouseLeftWindow(wxMouseEvent& event);
    void keyPressed(wxKeyEvent& event);
    void keyReleased(wxKeyEvent& event);
    void onFit(wxCommandEvent& event);
    void onMouseHelp(wxCommandEvent& event);
    void onAddMarker(wxCommandEvent& event);
    void onRemoveMarker(wxCommandEvent& event);
    void onShowMarkersMenu(wxCommandEvent& event);
    void onLockAspect(wxCommandEvent& event);
    void onSearchPeak(wxCommandEvent& event);
    void OnTimer(wxTimerEvent& event);
    void onReset(wxCommandEvent& event);
    DECLARE_EVENT_TABLE()

  private:
    bool oglOk;
    bool glInitialized;
    void UpdateInfoDisplay();
    std::vector<std::string> info_msg;
    std::vector<std::string> info_msg_toDisplay;
    static const unsigned int mMarkerColors[];

    dlgMarkers* mMarkersDlg;
    void ShowMenu(int x, int y);
    wxMenu m_popmenu;
    void setupViewport(int w, int h);
    wxGLContext* m_DrawingContext;
    eOGLGActionState m_actionState;

    bool viewChanged;
    bool initialized;
    sRect<float> initialDisplayArea;
    sRect<int> m_MouseCoord;
    float m_lastSpanX;
    float m_lastSpanY;

    bool isInsideDataView(int X, int Y);
    void dataViewPixelToValue(int x, int y, float& valX, float& valY);

    void DrawStaticElements();
    void DrawMarkers();
    void CalculateGrid();

    void switchToWindowView();
    void switchToDataView();

    GLvoid glRenderText(float posx, float posy, float angle, float scale, unsigned int rgba, const char* fmt, ...);
    int TextWidthInPixels(const char* str);
    int NumberWidthInPixels(float num, unsigned int prec = 0);
    int LineHeight();

    float calcAxisGrid(GLG_settings st, bool xAxis);

    GLFont* m_font;

    std::vector<OGLMarker> markers;
    int m_selectedMarker;
    unsigned m_maxMarkers;
    int clickedOnMarker(int X, int Y);

    bool m_currentlyDrawing;
    wxTimer* m_timer;
    wxGLContext* m_glContext;

    static bool hasNotRecentEnoughOpenGLVersionWarningBeenThrownYet;
};

#endif
