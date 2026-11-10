#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <wx/event.h>
#include <wx/gdicmn.h>
#include <wx/panel.h>
#include <wx/string.h>

#include "GUI_Utils.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/ShakeTune/KlipperReport.hpp"
#include "libslic3r/ShakeTune/Vibrations.hpp"

class Button;
class CheckBox;
class ComboBox;
class Label;
class wxDC;
class wxFlexGridSizer;
class wxStaticText;

namespace Slic3r {

class Moonraker;

namespace GUI {

class Worker;

// Each motor's vibration energy against speed, with its resonance bands shaded, and Shake&Tune's
// own results for every direction. Interactive: a legend whose entries show and hide their series,
// panning and zooming on both axes (mouse, keyboard and scroll bars) and a readout at the cursor.
class ResonanceChart : public wxPanel
{
public:
    explicit ResonanceChart(wxWindow *parent);
    ~ResonanceChart() override;
    // `motor_names` label the two motors' series in the legend.
    void set_profile(std::optional<ShakeTune::VibrationsProfile> profile, std::array<wxString, 2> motor_names);
    void rescale();

private:
    enum Series { MotorCurveA, MotorCurveB, MotorBandsA, MotorBandsB, QuietSpeeds, FlaggedSpeeds, VibrationMetric, SeriesCount };
    // A range of the data, in mm/s and energy.
    struct View
    {
        double x0{ 0. }, x1{ 1. }, y0{ 0. }, y1{ 1. };
    };
    // Where everything goes for the current client size.
    struct Layout
    {
        wxRect                                 plot, x_axis, y_axis, h_bar, v_bar;
        std::vector<std::pair<wxRect, Series>> legend;
    };
    enum class Drag { None, Pan, HorizontalThumb, VerticalThumb };

    void   on_paint(wxPaintEvent &event);
    void   on_left_down(wxMouseEvent &event);
    void   on_left_up(wxMouseEvent &event);
    void   on_motion(wxMouseEvent &event);
    void   on_wheel(wxMouseEvent &event);
    void   on_key(wxKeyEvent &event);
    void   end_drag(bool commit);
    Layout layout(wxDC &dc) const;
    View   full_view() const;
    // Shake&Tune draws its vibration metric on an axis of its own; here it is scaled so its peak
    // reaches the motors' highest energy.
    double metric_scale() const;
    void   set_view(View view);
    void   zoom(bool horizontal, double factor, double anchor);
    void   pan(double dx_fraction, double dy_fraction);
    void   toggle(Series s);
    bool   has_series(Series s) const;
    bool   visible(Series s) const { return has_series(s) && !m_hidden[s]; }
    // The scroll bar thumb, empty when the whole axis is in view.
    wxRect thumb(bool horizontal) const;
    wxString series_label(Series s) const;

    std::optional<ShakeTune::VibrationsProfile> m_profile;
    std::array<wxString, 2>                     m_motor_names;
    std::array<bool, SeriesCount>               m_hidden{};
    View                                        m_view;
    // Laid out by the last paint, for hit testing.
    Layout                                      m_layout;
    Drag                                        m_drag{ Drag::None };
    wxPoint                                     m_drag_start;
    View                                        m_drag_view;
    std::optional<wxPoint>                      m_hover;
};

// Reads a Shake&Tune vibrations profile, from the printer through Moonraker or from a .stdata
// file, and proposes printer settings from it: the motor resonance bands, the outer wall
// resonance avoidance range and, from the printer's input shapers, the acceleration limits.
class ShakeTuneImportDialog : public DPIDialog
{
public:
    // `printer_config` is the edited printer preset: its host is asked for the data and its
    // values are shown as the ones that would change.
    ShakeTuneImportDialog(wxWindow *parent, const DynamicPrintConfig &printer_config);
    ~ShakeTuneImportDialog() override;

    // The settings to change, after ShowModal() returned wxID_OK.
    const DynamicPrintConfig &changes() const { return m_changes; }

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;
    void on_sys_color_changed() override;

private:
    struct Row
    {
        ::CheckBox     *check{ nullptr };
        wxStaticText   *name{ nullptr };
        wxStaticText   *value{ nullptr };
        DynamicPrintConfig change;
    };

    void fetch_printer();
    void analyse_remote(size_t recording);
    void analyse_file(const std::string &path);
    void open_file();
    void show_profile(const ShakeTune::VibrationsProfile &profile);
    void clear_rows();
    void add_row(const wxString &name, const wxString &value, DynamicPrintConfig change, bool checked);
    void set_status(const wxString &text, bool error = false);
    wxString motor_name(size_t motor, ShakeTune::MotorKinematics kinematics) const;

    const DynamicPrintConfig                    m_printer_config;
    std::shared_ptr<Moonraker>                  m_host;
    std::unique_ptr<Worker>                     m_worker;
    std::optional<ShakeTune::KlipperMotion>     m_motion;
    std::vector<ShakeTune::RemoteRecording>     m_recordings;
    DynamicPrintConfig                          m_changes;

    ::ComboBox      *m_recording_combo{ nullptr };
    ::Button        *m_open_file_btn{ nullptr };
    ::Label         *m_status{ nullptr };
    ResonanceChart  *m_chart{ nullptr };
    ::Label         *m_summary{ nullptr };
    wxFlexGridSizer *m_rows_sizer{ nullptr };
    ::Label         *m_hint{ nullptr };
    std::vector<Row> m_rows;
};

} // namespace GUI
} // namespace Slic3r
