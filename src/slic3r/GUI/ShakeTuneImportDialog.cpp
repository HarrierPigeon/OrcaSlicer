#include "ShakeTuneImportDialog.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

#include <wx/dcbuffer.h>
#include <wx/dcclient.h>
#include <wx/dcgraph.h>
#include <wx/cursor.h>
#include <wx/filedlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include "GUI.hpp"
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

namespace {

// The view never spans less than this share of the data.
constexpr double MIN_VIEW_SHARE = 1. / 200.;
// Scroll steps move the view by this share of its span; a click beside a scroll bar thumb by a page.
constexpr double SCROLL_STEP = 0.1;
constexpr double SCROLL_PAGE = 0.9;
// Each wheel notch zooms by this factor.
constexpr double WHEEL_ZOOM = 0.85;

const wxColour QUIET_COLOUR(0x27, 0xAE, 0x60);
const wxColour METRIC_COLOUR(0xE9, 0x1E, 0x63);

// A tick step giving about `count` ticks over `span`: 1, 2 or 5 times a power of ten.
double tick_step(double span, double count)
{
    const double raw  = span / std::max(1., count);
    const double mag  = std::pow(10., std::floor(std::log10(raw)));
    const double norm = raw / mag;
    return (norm < 1.5 ? 1. : norm < 3. ? 2. : norm < 7. ? 5. : 10.) * mag;
}

// The curve `ys`, sampled at `xs`, at x.
double sample(const std::vector<double> &xs, const std::vector<double> &ys, double x)
{
    if (xs.empty() || xs.size() != ys.size())
        return 0.;
    if (x <= xs.front())
        return ys.front();
    if (x >= xs.back())
        return ys.back();
    const size_t i = size_t(std::upper_bound(xs.begin(), xs.end(), x) - xs.begin());
    const double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

wxColour with_alpha(const wxColour &c, unsigned char alpha) { return wxColour(c.Red(), c.Green(), c.Blue(), alpha); }

} // namespace

ResonanceChart::ResonanceChart(wxWindow *parent)
    : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE | wxWANTS_CHARS)
{
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    SetBackgroundColour(*wxWHITE);
    // Shake&Tune's own metric is on the chart for comparison with its graph, but off at first.
    m_hidden[VibrationMetric] = true;
    rescale();
    Bind(wxEVT_PAINT, &ResonanceChart::on_paint, this);
    Bind(wxEVT_LEFT_DOWN, &ResonanceChart::on_left_down, this);
    Bind(wxEVT_LEFT_UP, &ResonanceChart::on_left_up, this);
    Bind(wxEVT_LEFT_DCLICK, [this](wxMouseEvent &event) {
        if (m_layout.plot.Contains(event.GetPosition()))
            set_view(full_view());
        else
            on_left_down(event);
    });
    Bind(wxEVT_MOTION, &ResonanceChart::on_motion, this);
    Bind(wxEVT_MOUSEWHEEL, &ResonanceChart::on_wheel, this);
    Bind(wxEVT_KEY_DOWN, &ResonanceChart::on_key, this);
    Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent &event) {
        m_hover.reset();
        Refresh();
        event.Skip();
    });
    // Losing the mouse mid-drag cancels the drag.
    Bind(wxEVT_MOUSE_CAPTURE_LOST, [this](wxMouseCaptureLostEvent &) { end_drag(false); });
}

ResonanceChart::~ResonanceChart()
{
    if (HasCapture())
        ReleaseMouse();
}

void ResonanceChart::set_profile(std::optional<ShakeTune::VibrationsProfile> profile, std::array<wxString, 2> motor_names)
{
    end_drag(false);
    m_profile     = std::move(profile);
    m_motor_names = std::move(motor_names);
    m_view        = full_view();
    Refresh();
}

void ResonanceChart::rescale()
{
    SetMinSize(FromDIP(wxSize(640, 320)));
    Refresh();
}

bool ResonanceChart::has_series(Series s) const
{
    if (!m_profile || m_profile->speeds.size() < 2)
        return false;
    const ShakeTune::VibrationsProfile &p = *m_profile;
    switch (s) {
    case MotorCurveA:
    case MotorCurveB: return p.motor_curve[s - MotorCurveA].size() == p.speeds.size();
    case MotorBandsA:
    case MotorBandsB: return !p.motor_bands[s - MotorBandsA].empty();
    case QuietSpeeds: return !p.good_speed_ranges.empty();
    case FlaggedSpeeds: return !p.peak_speeds.empty();
    case VibrationMetric: return p.vibration_metric.size() == p.speeds.size();
    default: return false;
    }
}

wxString ResonanceChart::series_label(Series s) const
{
    switch (s) {
    case MotorCurveA:
    case MotorCurveB: return m_motor_names[s - MotorCurveA];
    // TRN %1% is a stepper motor, such as "X stepper (motor A)".
    case MotorBandsA:
    case MotorBandsB: return format_wxstr(_L("%1% resonance bands"), m_motor_names[s - MotorBandsA]);
    case QuietSpeeds: return _L("Quietest speeds (Shake&Tune)");
    case FlaggedSpeeds: return _L("Vibration peaks (Shake&Tune)");
    case VibrationMetric: return _L("Vibration metric (Shake&Tune, scaled)");
    default: return wxEmptyString;
    }
}

ResonanceChart::View ResonanceChart::full_view() const
{
    View v;
    if (!m_profile || m_profile->speeds.size() < 2)
        return v;
    const ShakeTune::VibrationsProfile &p = *m_profile;
    v.x0 = 0.;
    v.x1 = p.speeds.back();
    // Scaled to the motors' curves on show, or to both when neither is.
    auto top_of = [&](bool only_visible) {
        double top = 0.;
        for (Series s : { MotorCurveA, MotorCurveB })
            if (has_series(s) && (!only_visible || visible(s)))
                for (double e : p.motor_curve[s - MotorCurveA])
                    top = std::max(top, e);
        return top;
    };
    double top = top_of(true);
    if (!(top > 0.))
        top = top_of(false);
    v.y1 = (top > 0. ? top : 1.) * 1.05;
    return v;
}

double ResonanceChart::metric_scale() const
{
    if (!has_series(VibrationMetric))
        return 1.;
    double motors = 0., metric = 0.;
    for (const std::vector<double> &curve : m_profile->motor_curve)
        for (double e : curve)
            motors = std::max(motors, e);
    for (double e : m_profile->vibration_metric)
        metric = std::max(metric, e);
    return metric > 0. && motors > 0. ? motors / metric : 1.;
}

void ResonanceChart::set_view(View v)
{
    const View full = full_view();
    auto clamp_axis = [](double &a, double &b, double lo, double hi) {
        const double span = std::clamp(b - a, (hi - lo) * MIN_VIEW_SHARE, hi - lo);
        a                 = std::clamp(a, lo, hi - span);
        b                 = a + span;
    };
    clamp_axis(v.x0, v.x1, full.x0, full.x1);
    clamp_axis(v.y0, v.y1, full.y0, full.y1);
    m_view = v;
    Refresh();
}

void ResonanceChart::zoom(bool horizontal, double factor, double anchor)
{
    View    v = m_view;
    double &a = horizontal ? v.x0 : v.y0;
    double &b = horizontal ? v.x1 : v.y1;
    a         = anchor - (anchor - a) * factor;
    b         = anchor + (b - anchor) * factor;
    set_view(v);
}

void ResonanceChart::pan(double dx_fraction, double dy_fraction)
{
    View         v  = m_view;
    const double dx = (v.x1 - v.x0) * dx_fraction, dy = (v.y1 - v.y0) * dy_fraction;
    v.x0 += dx;
    v.x1 += dx;
    v.y0 += dy;
    v.y1 += dy;
    set_view(v);
}

void ResonanceChart::toggle(Series s)
{
    // An energy axis showing everything keeps showing everything when the curves on show change.
    const View old_full = full_view();
    const bool whole_y  = m_view.y0 <= old_full.y0 && m_view.y1 >= old_full.y1;
    m_hidden[s]         = !m_hidden[s];
    View v              = m_view;
    if (whole_y) {
        const View full = full_view();
        v.y0            = full.y0;
        v.y1            = full.y1;
    }
    set_view(v);
}

wxRect ResonanceChart::thumb(bool horizontal) const
{
    const View   full = full_view();
    const wxRect bar  = horizontal ? m_layout.h_bar : m_layout.v_bar;
    const double full_span = horizontal ? full.x1 - full.x0 : full.y1 - full.y0;
    const double span      = horizontal ? m_view.x1 - m_view.x0 : m_view.y1 - m_view.y0;
    if (!m_profile || bar.IsEmpty() || !(full_span > 0.) || span >= full_span * 0.999)
        return wxRect();
    const int length = horizontal ? bar.width : bar.height;
    const int size   = std::max(FromDIP(16), int(std::lround(span / full_span * length)));
    // The vertical bar runs from the highest energy at the top down to zero.
    const double start = horizontal ? (m_view.x0 - full.x0) / full_span : (full.y1 - m_view.y1) / full_span;
    const int    pos   = std::min(length - size, int(std::lround(start * length)));
    return horizontal ? wxRect(bar.x + pos, bar.y, size, bar.height) : wxRect(bar.x, bar.y + pos, bar.width, size);
}

ResonanceChart::Layout ResonanceChart::layout(wxDC &dc) const
{
    Layout       l;
    const wxSize size   = GetClientSize();
    const int    text_h = dc.GetTextExtent("0").y;
    const int    gap    = FromDIP(6);
    const int    bar    = FromDIP(8);

    // Legend entries flow in rows above the plot.
    const int swatch = FromDIP(18);
    const int row_h  = text_h + FromDIP(4);
    int       x = 0, y = 0;
    for (int i = 0; i < SeriesCount; ++i) {
        const Series s = Series(i);
        if (!has_series(s))
            continue;
        const int w = swatch + gap + dc.GetTextExtent(series_label(s)).x;
        if (x > 0 && x + w > size.x) {
            x = 0;
            y += row_h;
        }
        l.legend.emplace_back(wxRect(x, y, w, row_h), s);
        x += w + FromDIP(16);
    }
    const int legend_h = l.legend.empty() ? 0 : y + row_h + gap;

    const int y_axis_w = dc.GetTextExtent("000.00").x + gap;
    const int plot_top = legend_h + text_h + gap; // the energy axis title goes above the plot
    const int plot_bottom = size.y - bar - gap - 2 * text_h - gap;
    l.plot   = wxRect(y_axis_w, plot_top, size.x - y_axis_w - bar - gap, plot_bottom - plot_top);
    l.x_axis = wxRect(l.plot.x, l.plot.GetBottom() + 1, l.plot.width, 2 * text_h + gap);
    l.y_axis = wxRect(0, l.plot.y, y_axis_w, l.plot.height);
    l.h_bar  = wxRect(l.plot.x, l.x_axis.GetBottom() + 1, l.plot.width, bar);
    l.v_bar  = wxRect(l.plot.GetRight() + gap, l.plot.y, bar, l.plot.height);
    return l;
}

void ResonanceChart::on_paint(wxPaintEvent &)
{
    wxAutoBufferedPaintDC pdc(this);
    wxGCDC                dc(pdc);
    const wxSize          size = GetClientSize();
    dc.SetBackground(wxBrush(GetBackgroundColour()));
    dc.Clear();

    const bool     dark = wxGetApp().dark_mode();
    const wxColour text_colour(dark ? wxColour(0xC8, 0xC8, 0xC8) : wxColour(0x50, 0x50, 0x50));
    const wxColour muted_colour(dark ? wxColour(0x6E, 0x6E, 0x6E) : wxColour(0xAA, 0xAA, 0xAA));
    const wxColour grid_colour(dark ? wxColour(0x48, 0x48, 0x48) : wxColour(0xE2, 0xE2, 0xE2));
    dc.SetFont(Label::Body_12);
    dc.SetTextForeground(text_colour);

    if (!m_profile || m_profile->speeds.size() < 2) {
        m_layout = Layout();
        const wxString msg = _L("No vibrations profile loaded");
        const wxSize   ext = dc.GetTextExtent(msg);
        dc.DrawText(msg, (size.x - ext.x) / 2, (size.y - ext.y) / 2);
        return;
    }
    const ShakeTune::VibrationsProfile &p = *m_profile;
    m_layout                              = layout(dc);
    const Layout &l                       = m_layout;
    if (l.plot.width <= 0 || l.plot.height <= 0)
        return;
    const View &v      = m_view;
    const int   text_h = dc.GetTextExtent("0").y;
    auto x_of = [&](double speed) { return int(std::lround(l.plot.x + (speed - v.x0) / (v.x1 - v.x0) * l.plot.width)); };
    auto y_of = [&](double energy) { return int(std::lround(l.plot.GetBottom() - (energy - v.y0) / (v.y1 - v.y0) * l.plot.height)); };

    // Legend: an entry that is switched off is greyed.
    for (const auto &[rect, s] : l.legend) {
        const bool     on  = visible(s);
        const wxRect   sw(rect.x, rect.y + rect.height / 2 - FromDIP(5), FromDIP(18), FromDIP(10));
        const wxColour col = s == MotorCurveA || s == MotorBandsA ? MOTOR_COLOURS[0] :
                             s == MotorCurveB || s == MotorBandsB ? MOTOR_COLOURS[1] :
                             s == QuietSpeeds                     ? QUIET_COLOUR :
                             s == VibrationMetric                 ? METRIC_COLOUR :
                                                                    text_colour;
        const wxColour shown = on ? col : muted_colour;
        if (s == MotorBandsA || s == MotorBandsB || s == QuietSpeeds) {
            dc.SetPen(*wxTRANSPARENT_PEN);
            dc.SetBrush(wxBrush(with_alpha(shown, 90)));
            dc.DrawRectangle(sw);
        } else {
            dc.SetPen(wxPen(shown, s == FlaggedSpeeds ? 1 : FromDIP(2), s == FlaggedSpeeds ? wxPENSTYLE_SHORT_DASH : wxPENSTYLE_SOLID));
            dc.DrawLine(sw.x, sw.y + sw.height / 2, sw.GetRight(), sw.y + sw.height / 2);
        }
        dc.SetTextForeground(on ? text_colour : muted_colour);
        dc.DrawText(series_label(s), sw.GetRight() + FromDIP(6), rect.y + (rect.height - text_h) / 2);
    }
    dc.SetTextForeground(text_colour);

    // Speed axis: grid, tick labels, title.
    const double x_step     = tick_step(v.x1 - v.x0, l.plot.width / double(FromDIP(70)));
    const int    x_decimals = x_step < 1. ? 1 : 0;
    for (double s = std::ceil(v.x0 / x_step) * x_step; s <= v.x1 + x_step * 1e-6; s += x_step) {
        dc.SetPen(wxPen(grid_colour, 1));
        dc.DrawLine(x_of(s), l.plot.y, x_of(s), l.plot.GetBottom());
        const wxString label = from_u8(number(s, x_decimals));
        const int      w     = dc.GetTextExtent(label).x;
        dc.DrawText(label, std::clamp(x_of(s) - w / 2, l.plot.x - w / 2, l.plot.GetRight() - w / 2), l.x_axis.y + FromDIP(2));
    }
    const wxString x_title = _L("Speed (mm/s)");
    dc.DrawText(x_title, l.plot.GetRight() - dc.GetTextExtent(x_title).x, l.x_axis.y + FromDIP(2) + text_h);

    // Energy axis: tick labels in units of a power of ten, given in the title.
    const double y_step   = tick_step(v.y1 - v.y0, l.plot.height / double(FromDIP(40)));
    const int    exponent = int(std::floor(std::log10(std::max(v.y1, 1e-300))));
    const double unit     = std::pow(10., exponent);
    const int    y_decimals = y_step / unit >= 1. ? 0 : y_step / unit >= 0.1 ? 1 : 2;
    for (double e = std::ceil(v.y0 / y_step) * y_step; e <= v.y1 + y_step * 1e-6; e += y_step) {
        dc.SetPen(wxPen(grid_colour, 1));
        dc.DrawLine(l.plot.x, y_of(e), l.plot.GetRight(), y_of(e));
        const wxString label = from_u8(Slic3r::format(y_decimals == 0 ? "%.0f" : y_decimals == 1 ? "%.1f" : "%.2f", e / unit));
        const wxSize   ext   = dc.GetTextExtent(label);
        dc.DrawText(label, l.y_axis.GetRight() - FromDIP(6) - ext.x, std::clamp(y_of(e) - ext.y / 2, l.plot.y - ext.y / 2, l.plot.GetBottom() - ext.y / 2));
    }
    // TRN Title of the chart's vertical axis; %1% is its unit, a power of ten such as "1e7".
    dc.DrawText(format_wxstr(_L("Energy (×%1%)"), from_u8(Slic3r::format("1e%d", exponent))), 0, l.plot.y - text_h - FromDIP(6));

    // The data, clipped to the plot.
    {
        wxDCClipper clip(dc, l.plot);
        auto shade = [&](double lo, double hi, const wxColour &c, unsigned char alpha) {
            dc.SetPen(*wxTRANSPARENT_PEN);
            dc.SetBrush(wxBrush(with_alpha(c, alpha)));
            const int x0 = std::max(x_of(lo), l.plot.x - 1), x1 = std::min(x_of(hi), l.plot.GetRight() + 1);
            if (x1 > x0)
                dc.DrawRectangle(x0, l.plot.y, x1 - x0, l.plot.height);
        };
        for (size_t m = 0; m < 2; ++m)
            if (visible(Series(MotorBandsA + m)))
                for (const ShakeTune::SpeedBand &b : p.motor_bands[m])
                    shade(b.lo, b.hi, MOTOR_COLOURS[m], 60);
        if (visible(FlaggedSpeeds)) {
            dc.SetPen(wxPen(text_colour, 1, wxPENSTYLE_SHORT_DASH));
            for (double s : p.peak_speeds)
                dc.DrawLine(x_of(s), l.plot.y, x_of(s), l.plot.GetBottom());
        }
        auto curve = [&](const std::vector<double> &ys, const wxPen &pen) {
            std::vector<wxPoint> pts;
            pts.reserve(ys.size());
            for (size_t i = 0; i < ys.size(); ++i)
                pts.emplace_back(x_of(p.speeds[i]), y_of(ys[i]));
            dc.SetPen(pen);
            dc.DrawLines(int(pts.size()), pts.data());
        };
        if (visible(VibrationMetric)) {
            std::vector<double> scaled = p.vibration_metric;
            for (double &e : scaled)
                e *= metric_scale();
            curve(scaled, wxPen(METRIC_COLOUR, FromDIP(1), wxPENSTYLE_LONG_DASH));
        }
        // Shake&Tune's quietest speeds, as a strip along the bottom of the plot so the motor bands
        // above stay readable.
        if (visible(QuietSpeeds)) {
            dc.SetPen(*wxTRANSPARENT_PEN);
            dc.SetBrush(wxBrush(with_alpha(QUIET_COLOUR, 200)));
            const int strip = FromDIP(6);
            for (const ShakeTune::ProfileRange &r : p.good_speed_ranges) {
                const int x0 = std::max(x_of(r.lo), l.plot.x - 1), x1 = std::min(x_of(r.hi), l.plot.GetRight() + 1);
                if (x1 > x0)
                    dc.DrawRectangle(x0, l.plot.GetBottom() - strip, x1 - x0, strip);
            }
        }
        for (size_t m = 0; m < 2; ++m)
            if (visible(Series(MotorCurveA + m)))
                curve(p.motor_curve[m], wxPen(MOTOR_COLOURS[m], FromDIP(2)));

        // Readout at the cursor: the speed, and each curve on show there.
        if (m_hover && m_drag == Drag::None && l.plot.Contains(*m_hover)) {
            const int    hx    = m_hover->x;
            const double speed = v.x0 + double(hx - l.plot.x) / l.plot.width * (v.x1 - v.x0);
            dc.SetPen(wxPen(muted_colour, 1));
            dc.DrawLine(hx, l.plot.y, hx, l.plot.GetBottom());
            std::vector<wxString> lines{ format_wxstr(_L("%1% mm/s"), from_u8(number(speed, 1))) };
            for (size_t m = 0; m < 2; ++m) {
                if (!visible(Series(MotorCurveA + m)))
                    continue;
                const double e = sample(p.speeds, p.motor_curve[m], speed);
                dc.SetPen(wxPen(MOTOR_COLOURS[m], FromDIP(2)));
                dc.SetBrush(wxBrush(GetBackgroundColour()));
                dc.DrawCircle(hx, y_of(e), FromDIP(3));
                const bool in_band = std::any_of(p.motor_bands[m].begin(), p.motor_bands[m].end(),
                                                 [speed](const ShakeTune::SpeedBand &b) { return b.contains(speed); });
                const wxString value = from_u8(Slic3r::format("%.3g", e));
                // TRN %1% is a stepper motor, %2% its vibration energy at the cursor.
                lines.push_back(in_band ? format_wxstr(_L("%1%: %2%, in a resonance band"), m_motor_names[m], value) :
                                          format_wxstr("%1%: %2%", m_motor_names[m], value));
            }
            if (visible(VibrationMetric)) {
                double peak = 0.;
                for (double e : p.vibration_metric)
                    peak = std::max(peak, e);
                const double share = peak > 0. ? 100. * sample(p.speeds, p.vibration_metric, speed) / peak : 0.;
                // TRN %1% is "Vibration metric (Shake&Tune)", %2% its value at the cursor as a share of its highest value.
                // xgettext:no-c-format, no-boost-format
                lines.push_back(format_wxstr(_L("%1%: %2%%% of its peak"), series_label(VibrationMetric), from_u8(number(share))));
            }
            int box_w = 0;
            for (const wxString &line : lines)
                box_w = std::max(box_w, dc.GetTextExtent(line).x);
            const int pad = FromDIP(6);
            wxRect    box(hx + FromDIP(12), l.plot.y + pad, box_w + 2 * pad, int(lines.size()) * text_h + 2 * pad);
            if (box.GetRight() > l.plot.GetRight())
                box.x = hx - FromDIP(12) - box.width;
            dc.SetPen(wxPen(grid_colour, 1));
            dc.SetBrush(wxBrush(with_alpha(GetBackgroundColour(), 225)));
            dc.DrawRoundedRectangle(box, FromDIP(3));
            for (size_t i = 0; i < lines.size(); ++i)
                dc.DrawText(lines[i], box.x + pad, box.y + pad + int(i) * text_h);
        }
    }
    dc.SetPen(wxPen(text_colour, 1));
    dc.DrawLine(l.plot.x, l.plot.GetBottom(), l.plot.GetRight(), l.plot.GetBottom());
    dc.DrawLine(l.plot.x, l.plot.y, l.plot.x, l.plot.GetBottom());

    // Scroll bars, while zoomed in on their axis.
    for (bool horizontal : { true, false }) {
        const wxRect t = thumb(horizontal);
        if (t.IsEmpty())
            continue;
        const wxRect bar = horizontal ? l.h_bar : l.v_bar;
        dc.SetPen(*wxTRANSPARENT_PEN);
        dc.SetBrush(wxBrush(grid_colour));
        dc.DrawRoundedRectangle(bar, bar.GetHeight() < bar.GetWidth() ? bar.GetHeight() / 2 : bar.GetWidth() / 2);
        dc.SetBrush(wxBrush(m_drag == (horizontal ? Drag::HorizontalThumb : Drag::VerticalThumb) ? text_colour : muted_colour));
        dc.DrawRoundedRectangle(t, t.GetHeight() < t.GetWidth() ? t.GetHeight() / 2 : t.GetWidth() / 2);
    }
}

void ResonanceChart::on_left_down(wxMouseEvent &event)
{
    event.Skip();
    SetFocus();
    if (!m_profile)
        return;
    const wxPoint pos = event.GetPosition();
    for (const auto &[rect, s] : m_layout.legend)
        if (rect.Contains(pos)) {
            toggle(s);
            return;
        }
    const wxRect h = thumb(true), vt = thumb(false);
    if (!h.IsEmpty() && h.Contains(pos))
        m_drag = Drag::HorizontalThumb;
    else if (!h.IsEmpty() && m_layout.h_bar.Contains(pos))
        pan(pos.x < h.x ? -SCROLL_PAGE : SCROLL_PAGE, 0.);
    else if (!vt.IsEmpty() && vt.Contains(pos))
        m_drag = Drag::VerticalThumb;
    else if (!vt.IsEmpty() && m_layout.v_bar.Contains(pos))
        pan(0., pos.y < vt.y ? SCROLL_PAGE : -SCROLL_PAGE);
    else if (m_layout.plot.Contains(pos))
        m_drag = Drag::Pan;
    if (m_drag != Drag::None) {
        m_drag_start = pos;
        m_drag_view  = m_view;
        if (!HasCapture())
            CaptureMouse();
        Refresh();
    }
}

void ResonanceChart::on_left_up(wxMouseEvent &event)
{
    event.Skip();
    if (m_drag != Drag::None)
        end_drag(true);
}

void ResonanceChart::end_drag(bool commit)
{
    if (m_drag != Drag::None && !commit)
        m_view = m_drag_view;
    m_drag = Drag::None;
    if (HasCapture())
        ReleaseMouse();
    Refresh();
}

void ResonanceChart::on_motion(wxMouseEvent &event)
{
    event.Skip();
    const wxPoint pos = event.GetPosition();
    if (m_drag != Drag::None && !event.LeftIsDown())
        end_drag(true);
    if (m_drag != Drag::None) {
        const wxPoint d    = pos - m_drag_start;
        const View    full = full_view();
        View          v    = m_drag_view;
        double        dx = 0., dy = 0.;
        switch (m_drag) {
        case Drag::Pan:
            dx = -double(d.x) / std::max(1, m_layout.plot.width) * (v.x1 - v.x0);
            dy = double(d.y) / std::max(1, m_layout.plot.height) * (v.y1 - v.y0);
            break;
        case Drag::HorizontalThumb: dx = double(d.x) / std::max(1, m_layout.h_bar.width) * (full.x1 - full.x0); break;
        case Drag::VerticalThumb: dy = -double(d.y) / std::max(1, m_layout.v_bar.height) * (full.y1 - full.y0); break;
        default: break;
        }
        v.x0 += dx;
        v.x1 += dx;
        v.y0 += dy;
        v.y1 += dy;
        set_view(v);
        return;
    }
    m_hover = pos;
    const bool over_legend = std::any_of(m_layout.legend.begin(), m_layout.legend.end(),
                                         [&pos](const std::pair<wxRect, Series> &item) { return item.first.Contains(pos); });
    SetCursor(over_legend ? wxCursor(wxCURSOR_HAND) : wxNullCursor);
    Refresh();
}

void ResonanceChart::on_wheel(wxMouseEvent &event)
{
    if (!m_profile || m_layout.plot.IsEmpty() || event.GetWheelDelta() == 0) {
        event.Skip();
        return;
    }
    const wxPoint pos    = event.GetPosition();
    const Layout &l      = m_layout;
    const double  steps  = double(event.GetWheelRotation()) / event.GetWheelDelta();
    const double  factor = std::pow(WHEEL_ZOOM, steps);
    const bool    sideways = event.GetWheelAxis() == wxMOUSE_WHEEL_HORIZONTAL;
    const double  speed  = m_view.x0 + double(pos.x - l.plot.x) / l.plot.width * (m_view.x1 - m_view.x0);
    const double  energy = m_view.y0 + double(l.plot.GetBottom() - pos.y) / l.plot.height * (m_view.y1 - m_view.y0);
    if (l.x_axis.Contains(pos) || l.h_bar.Contains(pos)) {
        sideways ? pan(SCROLL_STEP * steps, 0.) : zoom(true, factor, speed);
    } else if (l.y_axis.Contains(pos) || l.v_bar.Contains(pos)) {
        zoom(false, factor, energy);
    } else if (l.plot.Contains(pos)) {
        if (event.CmdDown())
            event.ShiftDown() ? zoom(false, factor, energy) : zoom(true, factor, speed);
        else if (sideways)
            pan(SCROLL_STEP * steps, 0.);
        else if (event.ShiftDown())
            pan(-SCROLL_STEP * steps, 0.);
        else
            pan(0., SCROLL_STEP * steps);
    } else
        event.Skip();
}

void ResonanceChart::on_key(wxKeyEvent &event)
{
    if (!m_profile) {
        event.Skip();
        return;
    }
    const double mid_speed  = 0.5 * (m_view.x0 + m_view.x1);
    const double mid_energy = 0.5 * (m_view.y0 + m_view.y1);
    switch (event.GetKeyCode()) {
    case '+':
    case '=':
    case WXK_NUMPAD_ADD: zoom(true, 0.8, mid_speed); break;
    case '-':
    case WXK_NUMPAD_SUBTRACT: zoom(true, 1.25, mid_speed); break;
    case WXK_PAGEUP: zoom(false, 0.8, mid_energy); break;
    case WXK_PAGEDOWN: zoom(false, 1.25, mid_energy); break;
    case WXK_LEFT: pan(-SCROLL_STEP, 0.); break;
    case WXK_RIGHT: pan(SCROLL_STEP, 0.); break;
    case WXK_UP: pan(0., SCROLL_STEP); break;
    case WXK_DOWN: pan(0., -SCROLL_STEP); break;
    case '0':
    case WXK_NUMPAD0:
    case WXK_HOME: set_view(full_view()); break;
    default: event.Skip();
    }
}

// ---------------------------------------------------------------------------------------------------

ShakeTuneImportDialog::ShakeTuneImportDialog(wxWindow *parent, const DynamicPrintConfig &printer_config)
    : DPIDialog(parent ? parent : static_cast<wxWindow *>(wxGetApp().mainframe), wxID_ANY, _L("Import from Shake&Tune"),
                wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX | wxRESIZE_BORDER)
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
    content->Add(m_chart, 1, wxEXPAND | wxBOTTOM, FromDIP(4));
    // TRN %1% is the Ctrl key's name, with "+", such as "Ctrl+" (or "⌘" on macOS).
    auto *chart_hint = new Label(this, Label::Body_12,
                                 format_wxstr(_L("Drag or scroll to move around: Shift+scroll moves sideways. %1%scroll zooms speed, "
                                                 "%1%Shift+scroll zooms energy, and scrolling over an axis zooms it. Double-click "
                                                 "resets the view. Click an entry in the legend to show or hide it."),
                                              from_u8(shortkey_ctrl_prefix())),
                                 LB_AUTO_WRAP);
    content->Add(chart_hint, 0, wxEXPAND | wxBOTTOM, FromDIP(8));

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
    m_chart->set_profile(p, { motor_name(0, p.kinematics), motor_name(1, p.kinematics) });

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
