#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <wx/panel.h>

#include "GUI_Utils.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/ShakeTune/KlipperReport.hpp"
#include "libslic3r/ShakeTune/Vibrations.hpp"

class Button;
class CheckBox;
class ComboBox;
class Label;
class wxFlexGridSizer;
class wxStaticText;

namespace Slic3r {

class Moonraker;

namespace GUI {

class Worker;

// Each motor's vibration energy against speed, with its resonance bands shaded and the speeds
// Shake&Tune flags for every direction marked.
class ResonanceChart : public wxPanel
{
public:
    explicit ResonanceChart(wxWindow *parent);
    void set_profile(std::optional<ShakeTune::VibrationsProfile> profile);
    void rescale();

private:
    void on_paint(wxPaintEvent &event);

    std::optional<ShakeTune::VibrationsProfile> m_profile;
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
