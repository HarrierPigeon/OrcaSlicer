#include "ShakeTuneImportDialog.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

#include <wx/dcclient.h>
#include <wx/dcgraph.h>
#include <wx/filedlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "format.hpp"
#include "Jobs/BoostThreadWorker.hpp"
#include "Jobs/PlaterWorker.hpp"
#include "MainFrame.hpp"
#include "Widgets/Button.hpp"
#include "Widgets/CheckBox.hpp"
#include "Widgets/ComboBox.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/Label.hpp"
#include "Widgets/StateColor.hpp"
#include "libslic3r/ShakeTune/InputShaperLimits.hpp"
#include "libslic3r/ShakeTune/MotorResonance.hpp"
#include "slic3r/Utils/Http.hpp"
#include "slic3r/Utils/Moonraker.hpp"

namespace Slic3r {
namespace GUI {

namespace {

// Distinct on both the light and the dark background.
const wxColour MOTOR_COLOURS[2] = { wxColour(0xE6, 0x7E, 0x22), wxColour(0x34, 0x98, 0xDB) };

std::string number(double v, int decimals = 0)
{
    return Slic3r::format(decimals == 0 ? "%.0f" : "%.1f", v);
}

std::string speed_list(const std::vector<double> &speeds)
{
    std::string out;
    for (double s : speeds)
        out += (out.empty() ? "" : ", ") + number(s);
    return out;
}

std::string range_list(const std::vector<ShakeTune::ProfileRange> &ranges)
{
    std::string out;
    for (const ShakeTune::ProfileRange &r : ranges)
        out += (out.empty() ? "" : ", ") + number(r.lo) + "-" + number(r.hi);
    return out;
}

// Moonraker serves a file of its "config" root at server/files/config/<path>.
std::string config_file_url(const std::string &path)
{
    std::string url = "server/files/config";
    size_t      start = 0;
    while (start <= path.size()) {
        size_t end = path.find('/', start);
        if (end == std::string::npos)
            end = path.size();
        url += "/" + Http::url_encode(path.substr(start, end - start));
        start = end + 1;
    }
    return url;
}

// The direction-averaged speeds Shake&Tune flags, as one range for the outer-wall-only
// resonance avoidance: from the end of the quiet range below the strongest peak to the start of
// the quiet range above it.
std::optional<ShakeTune::SpeedBand> outer_wall_avoidance_range(const ShakeTune::VibrationsProfile &p)
{
    if (p.peak_speeds.empty() || p.speeds.empty())
        return std::nullopt;
    double peak = p.peak_speeds.front(), best = -1.;
    for (double s : p.peak_speeds) {
        const size_t i = size_t(std::lower_bound(p.speeds.begin(), p.speeds.end(), s) - p.speeds.begin());
        if (i < p.vibration_metric.size() && p.vibration_metric[i] > best) {
            best = p.vibration_metric[i];
            peak = s;
        }
    }
    double lo = p.speeds.front(), hi = p.speeds.back();
    for (const ShakeTune::ProfileRange &r : p.good_speed_ranges) {
        if (r.hi <= peak)
            lo = std::max(lo, r.hi);
        if (r.lo >= peak)
            hi = std::min(hi, r.lo);
    }
    if (!(hi > lo))
        return std::nullopt;
    return ShakeTune::SpeedBand{ std::floor(lo), std::ceil(hi) };
}

} // namespace

// ---------------------------------------------------------------------------------------------------

ResonanceChart::ResonanceChart(wxWindow *parent) : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE)
{
    SetBackgroundColour(*wxWHITE);
    rescale();
    Bind(wxEVT_PAINT, &ResonanceChart::on_paint, this);
}

void ResonanceChart::set_profile(std::optional<ShakeTune::VibrationsProfile> profile)
{
    m_profile = std::move(profile);
    Refresh();
}

void ResonanceChart::rescale() { SetMinSize(FromDIP(wxSize(560, 220))); }

void ResonanceChart::on_paint(wxPaintEvent &)
{
    wxPaintDC pdc(this);
    wxGCDC    dc(pdc);
    const wxSize size = GetClientSize();
    dc.SetBackground(wxBrush(GetBackgroundColour()));
    dc.Clear();

    const bool     dark = wxGetApp().dark_mode();
    const wxColour text_colour(dark ? wxColour(0xC8, 0xC8, 0xC8) : wxColour(0x50, 0x50, 0x50));
    const wxColour grid_colour(dark ? wxColour(0x50, 0x50, 0x50) : wxColour(0xDD, 0xDD, 0xDD));
    dc.SetFont(Label::Body_12);
    dc.SetTextForeground(text_colour);

    if (!m_profile || m_profile->speeds.size() < 2) {
        const wxString msg = _L("No vibrations profile loaded");
        const wxSize   ext = dc.GetTextExtent(msg);
        dc.DrawText(msg, (size.x - ext.x) / 2, (size.y - ext.y) / 2);
        return;
    }
    const ShakeTune::VibrationsProfile &p = *m_profile;

    const int    text_h = dc.GetTextExtent("0").y;
    const wxRect plot(FromDIP(12), text_h + FromDIP(12), size.x - FromDIP(24), size.y - 2 * text_h - FromDIP(20));
    if (plot.width <= 0 || plot.height <= 0)
        return;
    const double max_speed = p.speeds.back();
    double       max_energy = 0.;
    for (const std::vector<double> &curve : p.motor_curve)
        for (double e : curve)
            max_energy = std::max(max_energy, e);
    if (!(max_energy > 0.))
        max_energy = 1.;
    auto x_of = [&](double speed) { return plot.x + int(std::lround(speed / max_speed * plot.width)); };
    auto y_of = [&](double energy) { return plot.GetBottom() - int(std::lround(std::max(0., energy) / (max_energy * 1.05) * plot.height)); };

    // Speed grid and labels.
    const double step = max_speed > 300. ? 100. : max_speed > 120. ? 25. : 10.;
    for (double s = 0.; s <= max_speed + 1e-6; s += step) {
        dc.SetPen(wxPen(grid_colour, 1));
        dc.DrawLine(x_of(s), plot.y, x_of(s), plot.GetBottom());
        const wxString label = from_u8(number(s));
        const int      w     = dc.GetTextExtent(label).x;
        dc.DrawText(label, std::clamp(x_of(s) - w / 2, 0, size.x - w), plot.GetBottom() + FromDIP(4));
    }
    const wxString axis = _L("Speed (mm/s)");
    dc.DrawText(axis, plot.GetRight() - dc.GetTextExtent(axis).x, plot.GetBottom() + FromDIP(4) + text_h);

    // Resonance bands, then the curves on top.
    for (size_t m = 0; m < 2; ++m) {
        const wxColour c = MOTOR_COLOURS[m];
        dc.SetPen(*wxTRANSPARENT_PEN);
        dc.SetBrush(wxBrush(wxColour(c.Red(), c.Green(), c.Blue(), 60)));
        for (const ShakeTune::SpeedBand &b : p.motor_bands[m])
            dc.DrawRectangle(x_of(b.lo), plot.y, std::max(1, x_of(b.hi) - x_of(b.lo)), plot.height);
    }
    // Speeds Shake&Tune flags for every direction.
    dc.SetPen(wxPen(text_colour, 1, wxPENSTYLE_SHORT_DASH));
    for (double s : p.peak_speeds)
        dc.DrawLine(x_of(s), plot.y, x_of(s), plot.GetBottom());
    for (size_t m = 0; m < 2; ++m) {
        const std::vector<double> &curve = p.motor_curve[m];
        if (curve.size() != p.speeds.size())
            continue;
        std::vector<wxPoint> pts;
        pts.reserve(curve.size());
        for (size_t i = 0; i < curve.size(); ++i)
            pts.emplace_back(x_of(p.speeds[i]), y_of(curve[i]));
        dc.SetPen(wxPen(MOTOR_COLOURS[m], FromDIP(2)));
        dc.DrawLines(int(pts.size()), pts.data());
    }
    dc.SetPen(wxPen(text_colour, 1));
    dc.DrawLine(plot.x, plot.GetBottom(), plot.GetRight(), plot.GetBottom());
}

// ---------------------------------------------------------------------------------------------------

ShakeTuneImportDialog::ShakeTuneImportDialog(wxWindow *parent, const DynamicPrintConfig &printer_config)
    : DPIDialog(parent ? parent : static_cast<wxWindow *>(wxGetApp().mainframe), wxID_ANY, _L("Import from Shake&Tune"),
                wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX)
    , m_printer_config(printer_config)
{
    SetBackgroundColour(*wxWHITE);
    SetFont(Label::Body_14);
    m_worker = std::make_unique<PlaterWorker<BoostThreadWorker>>(this, nullptr, "shaketune_import");

    auto *top = new wxBoxSizer(wxVERTICAL);
    auto *content = new wxBoxSizer(wxVERTICAL);

    auto *source = new wxBoxSizer(wxHORIZONTAL);
    source->Add(new Label(this, _L("Vibrations profile")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(8));
    m_recording_combo = new ::ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition, FromDIP(wxSize(320, -1)), 0, nullptr, wxCB_READONLY);
    m_recording_combo->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
        const int sel = m_recording_combo->GetSelection();
        if (sel >= 0 && size_t(sel) < m_recordings.size())
            analyse_remote(size_t(sel));
    });
    source->Add(m_recording_combo, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(8));
    m_open_file_btn = new ::Button(this, _L("Open file") + dots);
    m_open_file_btn->SetStyle(ButtonStyle::Regular, ButtonType::Parameter);
    m_open_file_btn->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { open_file(); });
    source->Add(m_open_file_btn, 0, wxALIGN_CENTER_VERTICAL);
    content->Add(source, 0, wxEXPAND | wxBOTTOM, FromDIP(8));

    m_status = new Label(this, wxEmptyString, LB_AUTO_WRAP);
    content->Add(m_status, 0, wxEXPAND | wxBOTTOM, FromDIP(8));

    m_chart = new ResonanceChart(this);
    content->Add(m_chart, 1, wxEXPAND | wxBOTTOM, FromDIP(8));

    m_summary = new Label(this, wxEmptyString, LB_AUTO_WRAP);
    content->Add(m_summary, 0, wxEXPAND | wxBOTTOM, FromDIP(12));

    content->Add(new Label(this, Label::Head_14, _L("Printer settings to change")), 0, wxBOTTOM, FromDIP(6));
    m_rows_sizer = new wxFlexGridSizer(3, FromDIP(4), FromDIP(10));
    m_rows_sizer->AddGrowableCol(2, 1);
    content->Add(m_rows_sizer, 0, wxEXPAND | wxBOTTOM, FromDIP(8));

    m_hint = new Label(this, _L("The stepper speed ranges take effect where the process setting \"Avoid motor resonances\" "
                                "(Speed page) is turned on."), LB_AUTO_WRAP);
    content->Add(m_hint, 0, wxEXPAND);

    top->Add(content, 1, wxEXPAND | wxALL, FromDIP(16));

    auto *buttons = new DialogButtons(this, { "OK", "Cancel" });
    buttons->GetOK()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        m_changes.clear();
        for (const Row &row : m_rows)
            if (row.check->GetValue())
                m_changes.apply(row.change);
        EndModal(wxID_OK);
    });
    buttons->GetCANCEL()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { EndModal(wxID_CANCEL); });
    top->Add(buttons, 0, wxEXPAND);

    SetSizerAndFit(top);
    SetMinSize(GetSize());
    CenterOnParent();
    wxGetApp().UpdateDlgDarkUI(this);

    // Moonraker answers on the printer's own host even when the host type is OctoPrint, which it
    // emulates for uploads.
    if (!m_printer_config.opt_string("print_host").empty()) {
        DynamicPrintConfig host_config = m_printer_config;
        m_host                         = std::make_shared<Moonraker>(&host_config);
        fetch_printer();
    } else {
        m_recording_combo->Enable(false);
        set_status(_L("This printer has no network host set up, so its recordings cannot be listed. Open a .stdata file instead."));
    }
}

ShakeTuneImportDialog::~ShakeTuneImportDialog()
{
    m_worker->cancel_all();
    m_worker->wait_for_idle();
}

wxString ShakeTuneImportDialog::motor_name(size_t motor, ShakeTune::MotorKinematics kinematics) const
{
    if (kinematics == ShakeTune::MotorKinematics::CoreXY)
        return motor == 0 ? _L("X stepper (motor A)") : _L("Y stepper (motor B)");
    return motor == 0 ? _L("X stepper") : _L("Y stepper");
}

void ShakeTuneImportDialog::set_status(const wxString &text, bool error)
{
    m_status->SetLabel(text);
    const wxColour normal("#262E30");
    m_status->SetForegroundColour(error ? wxColour(0xD0, 0x1B, 0x1B) : wxGetApp().dark_mode() ? StateColor::darkModeColorFor(normal) : normal);
    m_status->Wrap(m_status->GetSize().x > 0 ? m_status->GetSize().x : FromDIP(560));
    Layout();
}

void ShakeTuneImportDialog::fetch_printer()
{
    struct Result
    {
        std::optional<ShakeTune::KlipperMotion> motion;
        std::vector<ShakeTune::RemoteRecording> recordings;
        std::string                             error;
    };
    auto result = std::make_shared<Result>();
    set_status(format_wxstr(_L("Reading the Shake&Tune recordings on %1%…"), m_host->get_host()));
    queue_job(*m_worker,
        [host = m_host, result](Job::Ctl &ctl) {
            auto     cancelled = [&ctl] { return ctl.was_canceled(); };
            wxString error;
            std::string body;
            if (host->get("printer/objects/query?configfile=settings", body, error, 20, cancelled)) {
                ShakeTune::KlipperMotion motion;
                std::string              parse_error;
                if (ShakeTune::parse_klipper_motion(body, motion, parse_error))
                    result->motion = motion;
            }
            if (ctl.was_canceled())
                return;
            body.clear();
            if (!host->get("server/files/list?root=config", body, error, 20, cancelled)) {
                result->error = into_u8(error);
                return;
            }
            std::vector<ShakeTune::RemoteRecording> all;
            if (!ShakeTune::parse_remote_recordings(body, all, result->error))
                return;
            for (ShakeTune::RemoteRecording &r : all)
                if (r.kind == "vibrations")
                    result->recordings.push_back(std::move(r));
        },
        [this, result](bool canceled, std::exception_ptr &eptr) {
            if (eptr) {
                try { std::rethrow_exception(eptr); }
                catch (const std::exception &ex) { result->error = ex.what(); }
                eptr = nullptr;
            }
            if (canceled)
                return;
            m_motion     = result->motion;
            m_recordings = result->recordings;
            if (!result->error.empty()) {
                set_status(format_wxstr(_L("Could not read the printer's files: %1%"), from_u8(result->error)), true);
                return;
            }
            m_recording_combo->Clear();
            for (const ShakeTune::RemoteRecording &r : m_recordings) {
                const size_t slash = r.path.find_last_of('/');
                m_recording_combo->Append(from_u8(slash == std::string::npos ? r.path : r.path.substr(slash + 1)));
            }
            if (m_recordings.empty()) {
                if (m_motion && !m_motion->has_shaketune)
                    set_status(_L("Shake&Tune is not set up on this printer."), true);
                else
                    set_status(_L("The printer holds no raw vibrations profile. Add keep_raw_data: True to its [shaketune] "
                                  "section, restart Klipper and run CREATE_VIBRATIONS_PROFILE."), true);
                return;
            }
            m_recording_combo->SetSelection(0);
            analyse_remote(0);
        });
}

void ShakeTuneImportDialog::analyse_remote(size_t index)
{
    struct Result
    {
        std::optional<ShakeTune::VibrationsProfile> profile;
        std::string                                 error;
    };
    auto              result   = std::make_shared<Result>();
    const std::string path     = m_recordings[index].path;
    const double      max_freq = m_motion ? 5. * m_motion->shaketune_max_freq : ShakeTune::DEFAULT_VIBRATIONS_MAX_FREQ;
    set_status(_L("Downloading and analysing the vibrations profile…"));
    replace_job(*m_worker,
        [host = m_host, result, path, max_freq](Job::Ctl &ctl) {
            wxString    error;
            std::string bytes;
            if (!host->get(config_file_url(path), bytes, error, 300, [&ctl] { return ctl.was_canceled(); })) {
                result->error = into_u8(error);
                return;
            }
            std::vector<ShakeTune::Measurement> measurements;
            if (ctl.was_canceled() || !ShakeTune::parse_stdata(bytes, measurements, result->error))
                return;
            bytes.clear();
            if (ctl.was_canceled())
                return;
            result->profile = ShakeTune::analyze_vibrations(measurements, max_freq, result->error);
        },
        [this, result](bool canceled, std::exception_ptr &eptr) {
            if (eptr) {
                try { std::rethrow_exception(eptr); }
                catch (const std::exception &ex) { result->error = ex.what(); }
                eptr = nullptr;
            }
            if (canceled)
                return;
            if (!result->profile) {
                set_status(format_wxstr(_L("Could not analyse the vibrations profile: %1%"), from_u8(result->error)), true);
                return;
            }
            set_status(wxEmptyString);
            show_profile(*result->profile);
        });
}

void ShakeTuneImportDialog::analyse_file(const std::string &path)
{
    struct Result
    {
        std::optional<ShakeTune::VibrationsProfile> profile;
        std::string                                 error;
    };
    auto         result   = std::make_shared<Result>();
    const double max_freq = m_motion ? 5. * m_motion->shaketune_max_freq : ShakeTune::DEFAULT_VIBRATIONS_MAX_FREQ;
    set_status(_L("Analysing the vibrations profile…"));
    replace_job(*m_worker,
        [result, path, max_freq](Job::Ctl &ctl) {
            std::vector<ShakeTune::Measurement> measurements;
            if (!ShakeTune::load_stdata(path, measurements, result->error) || ctl.was_canceled())
                return;
            result->profile = ShakeTune::analyze_vibrations(measurements, max_freq, result->error);
        },
        [this, result](bool canceled, std::exception_ptr &eptr) {
            if (eptr) {
                try { std::rethrow_exception(eptr); }
                catch (const std::exception &ex) { result->error = ex.what(); }
                eptr = nullptr;
            }
            if (canceled)
                return;
            if (!result->profile) {
                set_status(format_wxstr(_L("Could not analyse the vibrations profile: %1%"), from_u8(result->error)), true);
                return;
            }
            set_status(wxEmptyString);
            show_profile(*result->profile);
        });
}

void ShakeTuneImportDialog::open_file()
{
    wxFileDialog dlg(this, _L("Open a Shake&Tune vibrations profile"), wxEmptyString, wxEmptyString,
                     format_wxstr("%1% (*.stdata)|*.stdata", _L("Shake&Tune recordings")), wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() != wxID_OK)
        return;
    m_recording_combo->SetSelection(-1);
    analyse_file(into_u8(dlg.GetPath()));
}

void ShakeTuneImportDialog::clear_rows()
{
    m_rows_sizer->Clear(true);
    m_rows.clear();
}

void ShakeTuneImportDialog::add_row(const wxString &name, const wxString &value, DynamicPrintConfig change, bool checked)
{
    Row row;
    row.check = new ::CheckBox(this);
    row.check->SetValue(checked);
    row.name   = new wxStaticText(this, wxID_ANY, name);
    row.value  = new wxStaticText(this, wxID_ANY, value);
    row.change = std::move(change);
    m_rows_sizer->Add(row.check, 0, wxALIGN_CENTER_VERTICAL);
    m_rows_sizer->Add(row.name, 0, wxALIGN_CENTER_VERTICAL);
    m_rows_sizer->Add(row.value, 0, wxALIGN_CENTER_VERTICAL | wxEXPAND);
    m_rows.push_back(std::move(row));
}

void ShakeTuneImportDialog::show_profile(const ShakeTune::VibrationsProfile &p)
{
    m_chart->set_profile(p);

    // What Shake&Tune itself reports, then what each motor adds to it.
    wxString summary;
    if (p.peak_speeds.empty())
        summary = _L("All directions: no vibration peaks.");
    else
        summary = format_wxstr(_L("All directions: vibration peaks at %1% mm/s."), from_u8(speed_list(p.peak_speeds)));
    if (!p.good_speed_ranges.empty())
        summary += " " + format_wxstr(_L("Quietest speeds: %1% mm/s."), from_u8(range_list(p.good_speed_ranges)));
    if (!p.good_angle_ranges.empty())
        summary += " " + format_wxstr(_L("Quietest directions: %1% degrees."), from_u8(range_list(p.good_angle_ranges)));
    // xgettext:no-c-format, no-boost-format
    summary += " " + format_wxstr(_L("Symmetry: %1%%%."), from_u8(number(p.symmetry_pct)));
    for (size_t m = 0; m < 2; ++m) {
        summary += "\n";
        if (p.motor_bands[m].empty())
            summary += format_wxstr(_L("%1%: no resonance found."), motor_name(m, p.kinematics));
        else
            summary += format_wxstr(_L("%1%: resonates at %2% mm/s, avoid %3% mm/s."), motor_name(m, p.kinematics),
                                    from_u8(speed_list(p.motor_peak_speeds[m])), from_u8(ShakeTune::format_speed_bands(p.motor_bands[m])));
    }
    m_summary->SetLabel(summary);
    m_summary->Wrap(m_summary->GetSize().x > 0 ? m_summary->GetSize().x : FromDIP(560));

    clear_rows();
    const wxString none = _L("none");
    auto           arrow = [](const wxString &from, const wxString &to) { return format_wxstr("%1% → %2%", from, to); };

    // Motor layout.
    {
        const auto kin = p.kinematics == ShakeTune::MotorKinematics::CoreXY ? MotorResonanceKinematics::CoreXY : MotorResonanceKinematics::Cartesian;
        const auto *cur = m_printer_config.option<ConfigOptionEnum<MotorResonanceKinematics>>("motor_resonance_kinematics");
        auto        label = [](MotorResonanceKinematics k) { return k == MotorResonanceKinematics::CoreXY ? _L("CoreXY") : _L("Cartesian"); };
        DynamicPrintConfig change;
        change.set_key_value("motor_resonance_kinematics", new ConfigOptionEnum<MotorResonanceKinematics>(kin));
        add_row(_L("Motor layout"), arrow(cur ? label(cur->value) : none, label(kin)), std::move(change), true);
    }
    // Stepper resonance ranges.
    for (size_t m = 0; m < 2; ++m) {
        const std::string key  = m == 0 ? "motor_resonance_bands_x" : "motor_resonance_bands_y";
        const std::string cur  = m_printer_config.has(key) ? m_printer_config.opt_string(key) : std::string();
        const std::string next = ShakeTune::format_speed_bands(p.motor_bands[m]);
        DynamicPrintConfig change;
        change.set_key_value(key, new ConfigOptionString(next));
        add_row(format_wxstr(_L("%1% resonant speeds"), motor_name(m, p.kinematics)),
                arrow(cur.empty() ? none : from_u8(cur), next.empty() ? none : format_wxstr(_L("%1% mm/s"), from_u8(next))),
                std::move(change), true);
    }
    // Acceleration limits from the input shapers the printer runs.
    if (m_motion) {
        const std::optional<ShakeTune::KlipperMotion::Shaper> shapers[2] = { m_motion->shaper_x, m_motion->shaper_y };
        for (size_t axis = 0; axis < 2; ++axis) {
            if (!shapers[axis])
                continue;
            const auto shaper = ShakeTune::make_input_shaper(shapers[axis]->type, shapers[axis]->freq, shapers[axis]->damping_ratio);
            if (!shaper)
                continue;
            const double accel = ShakeTune::round_max_accel(ShakeTune::input_shaper_max_accel(*shaper, m_motion->square_corner_velocity));
            const std::string key = axis == 0 ? "machine_max_acceleration_x" : "machine_max_acceleration_y";
            const auto *cur = m_printer_config.option<ConfigOptionFloats>(key);
            if (cur == nullptr || cur->values.empty())
                continue;
            ConfigOptionFloats next = *cur;
            next.values[0]          = accel;
            DynamicPrintConfig change;
            change.set_key_value(key, next.clone());
            wxString shaper_text = from_u8(shapers[axis]->type);
            shaper_text.MakeUpper();
            add_row(axis == 0 ? _L("Max acceleration X") : _L("Max acceleration Y"),
                    format_wxstr(_L("%1% → %2% mm/s² (%3% shaper at %4% Hz)"), from_u8(number(cur->values[0])), from_u8(number(accel)),
                                 shaper_text, from_u8(number(shapers[axis]->freq, 1))),
                    std::move(change), true);
        }
    }
    // The single outer-wall range, for the existing direction-blind resonance avoidance.
    if (const std::optional<ShakeTune::SpeedBand> range = outer_wall_avoidance_range(p)) {
        DynamicPrintConfig change;
        change.set_key_value("resonance_avoidance", new ConfigOptionBool(true));
        change.set_key_value("min_resonance_avoidance_speed", new ConfigOptionFloat(range->lo));
        change.set_key_value("max_resonance_avoidance_speed", new ConfigOptionFloat(range->hi));
        const bool on  = m_printer_config.has("resonance_avoidance") && m_printer_config.opt_bool("resonance_avoidance");
        const wxString cur = on ? format_wxstr(_L("%1%-%2% mm/s"), from_u8(number(m_printer_config.opt_float("min_resonance_avoidance_speed"))),
                                               from_u8(number(m_printer_config.opt_float("max_resonance_avoidance_speed"))))
                                : _L("off");
        add_row(_L("Outer wall resonance avoidance (all directions)"),
                arrow(cur, format_wxstr(_L("%1%-%2% mm/s"), from_u8(number(range->lo)), from_u8(number(range->hi)))), std::move(change), false);
    }

    wxGetApp().UpdateDlgDarkUI(this);
    Layout();
    GetSizer()->SetSizeHints(this);
}

void ShakeTuneImportDialog::on_dpi_changed(const wxRect &)
{
    m_recording_combo->Rescale();
    m_open_file_btn->Rescale();
    for (Row &row : m_rows)
        row.check->Rescale();
    m_chart->rescale();
    GetSizer()->SetSizeHints(this);
    Refresh();
}

void ShakeTuneImportDialog::on_sys_color_changed()
{
    wxGetApp().UpdateDlgDarkUI(this);
    m_chart->Refresh();
}

} // namespace GUI
} // namespace Slic3r
