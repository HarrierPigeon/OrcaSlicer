#include "SettingsDiffTable.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <numeric>
#include <tuple>

#include <wx/control.h>
#include <wx/dcbuffer.h>
#include <wx/dcgraph.h>

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "UnsavedChangesDialog.hpp"
#include "format.hpp"
#include "Widgets/Label.hpp"
#include "Widgets/StateColor.hpp"

namespace Slic3r { namespace GUI {

namespace {

// Include / exclude colours, [light base, dark base]: green and red, each at least 3:1 against the
// table background. The marks differ in shape too, so colour is never the only signal.
constexpr const char* INCLUDE_COLORS[2] = { "#1A7F37", "#3FB950" };
constexpr const char* EXCLUDE_COLORS[2] = { "#CF222E", "#F85149" };

wxColour blend(const wxColour& fg, const wxColour& bg, double alpha)
{
    auto mix = [alpha](unsigned char f, unsigned char b) { return (unsigned char) std::lround(f * alpha + b * (1. - alpha)); };
    return wxColour(mix(fg.Red(), bg.Red()), mix(fg.Green(), bg.Green()), mix(fg.Blue(), bg.Blue()));
}

wxString single_line(wxString text)
{
    text.Replace("\r", "");
    text.Replace("\n", " ");
    text.Replace("\t", " ");
    return text;
}

// Between the headings sharing a line, and before the setting they lead to.
const wxString PATH_SEPARATOR = " / ";
// Per level of nesting, and the room a heading's fold arrow takes (DIP).
constexpr int INDENT  = 16;
constexpr int CHEVRON = 16;

bool is_color_value(const wxString& value)
{
    return value.length() == 7 && value[0] == '#' && wxColour(value).IsOk();
}

} // namespace

// Every colour is one the app already uses for the same purpose (a background for a background, a
// border for a line), mapped to its dark-mode counterpart.
struct SettingsDiffTable::Colors
{
    bool     dark;
    wxColour background, band, sub_band, preset_band, hover, border;
    wxColour text, band_text, dimmed, header, header_text;
    wxColour include, exclude, neutral, focus;

    Colors()
    {
        auto role = [](const char* key) { return StateColor::darkModeColorFor(wxColour(key)); };
        dark        = wxGetApp().dark_mode();
        background  = role("#FFFFFF");      // window background
        band        = role("#DFDFDF");      // buttons
        sub_band    = blend(band, background, 0.5); // a group's heading, between its page's and its settings
        preset_band = role("#CECECE");      // selected tab
        hover       = role("#E5F0EE");      // focused combo box item: the Orca colour at 10%
        border      = role("#DBDBDB");      // input borders
        text        = StateColor::EnsureContrast(role("#363636"), background, 4.5); // labels
        band_text   = StateColor::EnsureContrast(role("#262E30"), preset_band, 4.5); // button text
        dimmed      = role("#6B6B6B");      // disabled text
        header      = role("#3B4446");      // tab bar
        header_text = StateColor::EnsureContrast(*wxWHITE, header, 4.5);
        neutral     = role("#009688");      // the Orca colour
        focus       = neutral;
        include     = wxColour(INCLUDE_COLORS[dark]);
        exclude     = wxColour(EXCLUDE_COLORS[dark]);
    }
};

SettingsDiffTable::SettingsDiffTable(wxWindow* parent)
    : wxScrolledCanvas(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL | wxWANTS_CHARS | wxFULL_REPAINT_ON_RESIZE | wxBORDER_NONE)
{
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    // The header and the legend stay in place, so a scroll repaints instead of shifting pixels.
    EnableScrolling(false, false);
    ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_DEFAULT);
    update_layout();

    Bind(wxEVT_PAINT, &SettingsDiffTable::on_paint, this);
    Bind(wxEVT_LEFT_DOWN, &SettingsDiffTable::on_left_down, this);
    Bind(wxEVT_LEFT_DCLICK, &SettingsDiffTable::on_left_down, this);
    Bind(wxEVT_MOTION, &SettingsDiffTable::on_motion, this);
    Bind(wxEVT_KEY_DOWN, &SettingsDiffTable::on_key_down, this);
    Bind(wxEVT_RIGHT_DOWN, [this](wxMouseEvent& evt) { show_full_values(hit_test(evt.GetPosition()).row); });
    Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent& evt) {
        m_hover = Hit();
        UnsetToolTip();
        Refresh();
        evt.Skip();
    });
    Bind(wxEVT_SET_FOCUS, [this](wxFocusEvent& evt) {
        m_key_nav = !wxGetMouseState().LeftIsDown();
        Refresh();
        evt.Skip();
    });
    Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent& evt) { Refresh(); evt.Skip(); });
    Bind(wxEVT_SIZE, [this](wxSizeEvent& evt) { Refresh(); evt.Skip(); });
}

void SettingsDiffTable::set_items(std::vector<SettingsDiffItem> items)
{
    m_items = std::move(items);
    m_included.assign(m_items.size(), true);

    // Presets, pages and groups each in order of first appearance, their settings kept together.
    std::map<wxString, size_t> ranks;
    auto rank = [&ranks](const wxString& path) { return ranks.emplace(path, ranks.size()).first->second; };
    std::vector<std::tuple<size_t, size_t, size_t>> keys;
    for (const SettingsDiffItem& item : m_items) {
        const wxString page = item.preset + '\x1f' + item.category;
        keys.emplace_back(rank(item.preset), rank(page), rank(page + '\x1f' + item.group));
    }
    m_order.resize(m_items.size());
    std::iota(m_order.begin(), m_order.end(), 0);
    std::stable_sort(m_order.begin(), m_order.end(), [&keys](size_t a, size_t b) { return keys[a] < keys[b]; });

    m_tree = build_tree(0, m_order.size(), 0, wxString());
    m_collapsed.clear();
    m_focus_row = 0;
    m_hover     = Hit();
    rebuild_rows();
    Scroll(-1, 0);
    Refresh();
}

std::vector<SettingsDiffTable::Node> SettingsDiffTable::build_tree(size_t first, size_t last, int level, const wxString& parent_key) const
{
    std::vector<Node> nodes;
    if (RowKind(level) == RowKind::Option) {
        for (size_t pos = first; pos < last; ++pos)
            nodes.push_back({ RowKind::Option, item_at(pos).label, wxString(), pos, pos + 1, {} });
        return nodes;
    }
    auto name = [this, level](size_t pos) -> const wxString& {
        const SettingsDiffItem& item = item_at(pos);
        return RowKind(level) == RowKind::Preset ? item.preset : RowKind(level) == RowKind::Category ? item.category : item.group;
    };
    for (size_t pos = first; pos < last;) {
        size_t end = pos + 1;
        while (end < last && name(end) == name(pos))
            ++end;
        const wxString    key      = parent_key + '\x1f' + name(pos);
        std::vector<Node> children = build_tree(pos, end, level + 1, key);
        if (name(pos).empty()) // no heading at this level: what is below it stands in its place
            std::move(children.begin(), children.end(), std::back_inserter(nodes));
        else
            nodes.push_back({ RowKind(level), name(pos), key, pos, end, std::move(children) });
        pos = end;
    }
    return nodes;
}

void SettingsDiffTable::rebuild_rows()
{
    m_rows.clear();
    m_sections.clear();
    for (const Node& node : m_tree)
        flatten(node, 0, wxString(), RowKind::Option, true);
    m_focus_row = std::clamp(m_focus_row, 0, std::max(0, int(m_rows.size()) - 1));
    update_layout();
}

// A heading with a single entry below it shares that entry's line ("Page / Group"), down to the
// setting itself, so a lone change takes one row instead of three.
void SettingsDiffTable::flatten(const Node& node, int indent, const wxString& prefix, RowKind band, bool visible)
{
    band = std::min(band, node.kind);
    if (node.kind == RowKind::Option) {
        if (visible)
            m_rows.push_back({ RowKind::Option, band, indent, node.first, node.last, node.text, prefix, wxString(), false });
        return;
    }
    const wxString path = prefix.empty() ? node.text : prefix + PATH_SEPARATOR + node.text;
    if (node.children.size() == 1) {
        flatten(node.children.front(), indent, path, band, visible);
        return;
    }
    const bool collapsed = m_collapsed.count(node.key) > 0;
    m_sections.push_back(node.key);
    if (visible)
        m_rows.push_back({ node.kind, band, indent, node.first, node.last, path, wxString(), node.key, collapsed });
    for (const Node& child : node.children)
        flatten(child, indent + 1, wxString(), RowKind::Option, visible && !collapsed);
}

size_t SettingsDiffTable::included_count() const
{
    return std::count(m_included.begin(), m_included.end(), true);
}

int SettingsDiffTable::preferred_height() const
{
    return m_header_h + m_footer_h + std::clamp(int(m_rows.size()), 8, 18) * m_row_h;
}

void SettingsDiffTable::rescale()
{
    update_layout();
    Refresh();
}

bool SettingsDiffTable::toggleable(const Hit& hit) const
{
    if (hit.row == NO_ROW)
        return false;
    if (hit.row == HEADER_ROW)
        return hit.part == Part::Toggle && range_mark(0, m_order.size()) != Mark::Locked;
    const Row& row = m_rows[hit.row];
    if (range_mark(row.first, row.last) == Mark::Locked)
        return false;
    // A setting is also chosen by clicking the value wanted.
    return hit.part == Part::Toggle || (row.kind == RowKind::Option && (hit.part == Part::New || hit.part == Part::Current));
}

bool SettingsDiffTable::collapsible(const Hit& hit) const
{
    if (hit.row == HEADER_ROW)
        return hit.part == Part::Label && !m_sections.empty();
    // A heading's title runs on under the New Value column.
    return hit.row >= 0 && !m_rows[hit.row].key.empty() && (hit.part == Part::Label || hit.part == Part::New);
}

void SettingsDiffTable::toggle_collapsed(int row)
{
    if (row == HEADER_ROW) {
        // The header folds every section, or opens them all once they are folded.
        if (m_collapsed.size() < m_sections.size())
            m_collapsed.insert(m_sections.begin(), m_sections.end());
        else
            m_collapsed.clear();
    } else {
        const wxString key = m_rows[row].key;
        if (m_collapsed.erase(key) == 0)
            m_collapsed.insert(key);
    }
    rebuild_rows();
    m_hover = Hit();
    UnsetToolTip();
    Refresh();
}

SettingsDiffTable::Mark SettingsDiffTable::range_mark(size_t first, size_t last) const
{
    size_t on = 0, off = 0;
    for (size_t pos = first; pos < last; ++pos)
        if (!item_at(pos).locked)
            ++(m_included[m_order[pos]] ? on : off);
    return on + off == 0 ? Mark::Locked : off == 0 ? Mark::Include : on == 0 ? Mark::Exclude : Mark::Mixed;
}

void SettingsDiffTable::set_range(size_t first, size_t last, bool include)
{
    bool any = false;
    for (size_t pos = first; pos < last; ++pos)
        if (!item_at(pos).locked && m_included[m_order[pos]] != include) {
            m_included[m_order[pos]] = include;
            any = true;
        }
    if (any)
        changed();
}

void SettingsDiffTable::toggle(const Hit& hit)
{
    if (!toggleable(hit))
        return;
    const size_t first = hit.row == HEADER_ROW ? 0 : m_rows[hit.row].first;
    const size_t last  = hit.row == HEADER_ROW ? m_order.size() : m_rows[hit.row].last;
    if (hit.part == Part::Toggle)
        // A partly included section becomes fully included first.
        set_range(first, last, range_mark(first, last) != Mark::Include);
    else
        set_range(first, last, hit.part == Part::New);
}

void SettingsDiffTable::changed()
{
    // The cell under the pointer may now read differently; the next motion re-derives it.
    m_hover = Hit();
    UnsetToolTip();
    Refresh();
    if (m_on_change)
        m_on_change();
}

SettingsDiffTable::Columns SettingsDiffTable::columns() const
{
    const int width = GetClientSize().x;
    Columns c;
    c.toggle_w  = FromDIP(44);
    c.label_w   = std::max(FromDIP(170), (width - c.toggle_w) * 38 / 100);
    c.value_w   = std::max(FromDIP(60), (width - c.toggle_w - c.label_w) / 3);
    c.new_x     = c.label_w;
    c.toggle_x  = c.new_x + c.value_w;
    c.current_x = c.toggle_x + c.toggle_w;
    c.result_x  = c.current_x + c.value_w;
    return c;
}

wxRect SettingsDiffTable::part_rect(const Columns& c, Part part, int y, int h) const
{
    switch (part) {
    case Part::Label:   return wxRect(0, y, c.label_w, h);
    case Part::New:     return wxRect(c.new_x, y, c.value_w, h);
    case Part::Toggle:  return wxRect(c.toggle_x, y, c.toggle_w, h);
    case Part::Current: return wxRect(c.current_x, y, c.result_x - c.current_x, h);
    case Part::Result:  return wxRect(c.result_x, y, std::max(0, GetClientSize().x - c.result_x), h);
    default:            return wxRect();
    }
}

wxRect SettingsDiffTable::label_rect(const Columns& c, const Row& row, int y) const
{
    // A heading has no values beside it, so it may run on under the New Value column.
    const int pad = FromDIP(10), x = pad + row.indent * FromDIP(INDENT) + FromDIP(CHEVRON);
    return wxRect(x, y, (row.kind == RowKind::Option ? c.new_x : c.toggle_x) - x - pad, m_row_h);
}

int SettingsDiffTable::row_top(int row) const
{
    return m_header_h + (row - GetViewStart().y) * m_row_h;
}

SettingsDiffTable::Hit SettingsDiffTable::hit_test(const wxPoint& pos) const
{
    const wxSize size = GetClientSize();
    Hit hit;
    if (pos.x < 0 || pos.x >= size.x || pos.y < 0 || pos.y >= size.y - m_footer_h)
        return hit;
    if (pos.y >= m_header_h) {
        const int row = (pos.y - m_header_h) / m_row_h + GetViewStart().y;
        if (row >= int(m_rows.size()))
            return hit;
        hit.row = row;
    } else
        hit.row = HEADER_ROW;
    const Columns c = columns();
    hit.part = pos.x < c.new_x ? Part::Label : pos.x < c.toggle_x ? Part::New : pos.x < c.current_x ? Part::Toggle :
               pos.x < c.result_x ? Part::Current : Part::Result;
    return hit;
}

void SettingsDiffTable::update_layout()
{
    int text_w = 0, text_h = 0;
    GetTextExtent("Mg", &text_w, &text_h, nullptr, nullptr, &::Label::Body_13);
    m_row_h    = std::max(FromDIP(26), text_h + FromDIP(10));
    m_header_h = m_row_h + FromDIP(4);
    m_footer_h = m_row_h + FromDIP(4);
    SetMinSize(wxSize(FromDIP(560), m_header_h + m_footer_h + 6 * m_row_h));
    // One scroll unit is one row; the header and the legend are part of the scrolled height.
    SetScrollRate(0, m_row_h);
    SetVirtualSize(wxDefaultCoord, m_header_h + m_footer_h + int(m_rows.size()) * m_row_h);
    AdjustScrollbars();
}

void SettingsDiffTable::ensure_visible(int row)
{
    const int visible = std::max(1, (GetClientSize().y - m_header_h - m_footer_h) / m_row_h);
    const int top     = GetViewStart().y;
    if (row < top)
        Scroll(-1, row);
    else if (row >= top + visible)
        Scroll(-1, row - visible + 1);
}

void SettingsDiffTable::move_focus(int row)
{
    if (m_rows.empty())
        return;
    m_focus_row = std::clamp(row, 0, int(m_rows.size()) - 1);
    m_key_nav   = true;
    ensure_visible(m_focus_row);
    Refresh();
}

void SettingsDiffTable::show_full_values(int row)
{
    if (row < 0 || m_rows[row].kind != RowKind::Option)
        return;
    const SettingsDiffItem& item = item_at(m_rows[row].first);
    FullCompareDialog(item.label, item.new_value, item.current_value, _L("New Value"), _L("Current Value")).ShowModal();
}

wxString SettingsDiffTable::tooltip_for(const Hit& hit)
{
    if (hit.row == NO_ROW || hit.part == Part::None)
        return wxEmptyString;
    if (hit.part == Part::Toggle) {
        if (hit.row >= 0 && m_rows[hit.row].kind == RowKind::Option)
            return item_at(m_rows[hit.row].first).locked ? _L("This change is always included.") : _L("Include or exclude this setting.");
        return toggleable(hit) ? _L("Include or exclude every setting below.") : wxString();
    }
    if (hit.row == HEADER_ROW)
        return collapsible(hit) ? _L("Collapse or expand all sections.") :
               hit.part == Part::Result ?
                   _L("What each setting ends up as: the new value where it is included, otherwise the current value.") : wxString();

    // Whatever does not fit its cell is shown in full.
    const Row&    row = m_rows[hit.row];
    const Columns c   = columns();
    wxString      text;
    int           width = part_rect(c, hit.part, 0, m_row_h).width - 2 * FromDIP(10);
    const wxFont* font  = &::Label::Body_13;
    if (row.kind != RowKind::Option) {
        if (hit.part != Part::Label && hit.part != Part::New)
            return wxEmptyString;
        text  = row.text;
        width = label_rect(c, row, 0).width;
        font  = &::Label::Head_13;
    } else if (hit.part == Part::Label) {
        text  = row.prefix.empty() ? row.text : row.prefix + PATH_SEPARATOR + row.text;
        width = label_rect(c, row, 0).width;
        if (!row.prefix.empty())
            font = &::Label::Head_13;
    } else {
        const SettingsDiffItem& item     = item_at(row.first);
        const bool              included = m_included[m_order[row.first]];
        text = hit.part == Part::New || (hit.part == Part::Result && included) ? item.new_value : item.current_value;
        if (hit.part == Part::Result && included)
            font = &::Label::Head_13;
        if (is_color_value(text))
            width -= FromDIP(18);
    }
    int text_w = 0, text_h = 0;
    GetTextExtent(text, &text_w, &text_h, nullptr, nullptr, font);
    if (text.Find('\n') == wxNOT_FOUND && text_w <= width)
        return wxEmptyString;
    const size_t max_len = 800;
    return text.length() > max_len ? text.Left(max_len) + dots : text;
}

bool SettingsDiffTable::draw_text(wxDC& dc, const wxString& text, const wxRect& rect, const wxColour& colour, const wxFont& font) const
{
    if (rect.width <= 0)
        return false;
    dc.SetFont(font);
    dc.SetTextForeground(colour);
    const wxString shown = wxControl::Ellipsize(text, dc, wxELLIPSIZE_END, rect.width, wxELLIPSIZE_FLAGS_NONE);
    dc.DrawText(shown, rect.x, rect.y + (rect.height - dc.GetCharHeight()) / 2);
    return shown != text;
}

void SettingsDiffTable::draw_mark(wxDC& dc, const Colors& colors, const wxRect& cell, Mark mark, bool hovered, int size) const
{
    const wxRect box(cell.x + (cell.width - size) / 2, cell.y + (cell.height - size) / 2, size, size);
    wxColour     stroke = colors.dimmed;
    if (mark == Mark::Locked) {
        dc.SetBrush(*wxTRANSPARENT_BRUSH);
        dc.SetPen(wxPen(colors.dimmed, 1));
    } else {
        wxColour fill = mark == Mark::Include ? colors.include : mark == Mark::Exclude ? colors.exclude : colors.neutral;
        if (hovered)
            fill = fill.ChangeLightness(colors.dark ? 118 : 86);
        const wxColour dark_stroke(27, 27, 30);
        stroke = StateColor::ContrastRatio(*wxWHITE, fill) >= StateColor::ContrastRatio(dark_stroke, fill) ? *wxWHITE : dark_stroke;
        dc.SetBrush(wxBrush(fill));
        dc.SetPen(*wxTRANSPARENT_PEN);
    }
    dc.DrawRoundedRectangle(box, size * 0.28);

    wxPen pen(stroke, std::max(1, size / 10));
    pen.SetCap(wxCAP_ROUND);
    dc.SetPen(pen);
    const int cx = box.x + size / 2, cy = box.y + size / 2;
    const int a  = std::lround(size * 0.27);
    switch (mark) {
    case Mark::Include:
    case Mark::Locked: { // an arrow carrying the new value over to the result
        const int head = std::lround(a * 0.75);
        dc.DrawLine(cx - a, cy, cx + a, cy);
        dc.DrawLine(cx + a, cy, cx + a - head, cy - head);
        dc.DrawLine(cx + a, cy, cx + a - head, cy + head);
        break;
    }
    case Mark::Exclude:
        dc.DrawLine(cx - a, cy - a, cx + a, cy + a);
        dc.DrawLine(cx - a, cy + a, cx + a, cy - a);
        break;
    case Mark::Mixed:
        dc.DrawLine(cx - a, cy, cx + a, cy);
        break;
    }
}

void SettingsDiffTable::draw_chevron(wxDC& dc, int x, int y, int h, bool collapsed, const wxColour& colour) const
{
    wxPen pen(colour, std::max(1, FromDIP(2)));
    pen.SetCap(wxCAP_ROUND);
    dc.SetPen(pen);
    const int a = FromDIP(4), cx = x + FromDIP(CHEVRON) / 2 - FromDIP(2), cy = y + h / 2;
    if (collapsed) { // pointing at the title
        dc.DrawLine(cx - a / 2, cy - a, cx + a / 2, cy);
        dc.DrawLine(cx + a / 2, cy, cx - a / 2, cy + a);
    } else {         // pointing at what is below
        dc.DrawLine(cx - a, cy - a / 2, cx, cy + a / 2);
        dc.DrawLine(cx, cy + a / 2, cx + a, cy - a / 2);
    }
}

void SettingsDiffTable::draw_row(wxDC& dc, const Colors& colors, const Columns& c, int row_idx, int y)
{
    const Row&   row    = m_rows[row_idx];
    const bool   option = row.kind == RowKind::Option;
    const wxRect rect(0, y, GetClientSize().x, m_row_h);
    const int    pad = FromDIP(10);

    // The band is that of the topmost heading on the row, so every page stays one band wide
    // even when a lone setting shares its line. A setting's whole row is tinted by what
    // happens to it: included or excluded, a shade deeper under the pointer.
    wxColour bg = row.band == RowKind::Preset ? colors.preset_band : row.band == RowKind::Category ? colors.band :
                  row.band == RowKind::Group ? colors.sub_band : colors.background;
    const bool hovered_row = m_hover.row == row_idx;
    if (option) {
        const bool included = m_included[m_order[row.first]];
        bg = blend(included ? colors.include : colors.exclude, bg, (colors.dark ? 0.26 : 0.14) + (hovered_row ? 0.08 : 0.));
    } else if (option && hovered_row)
        bg = colors.hover;
    const wxColour text_colour = StateColor::EnsureContrast(row.band <= RowKind::Category ? colors.band_text : colors.text, bg, 4.5);
    const wxColour dimmed      = StateColor::EnsureContrast(colors.dimmed, bg, 4.5);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(bg));
    dc.DrawRectangle(rect);

    // Guides down the left tie a row to the headings it sits under.
    dc.SetPen(wxPen(colors.border, 1));
    for (int level = 0; level < row.indent; ++level) {
        const int x = pad + level * FromDIP(INDENT) + FromDIP(CHEVRON) / 2 - FromDIP(2);
        dc.DrawLine(x, y, x, y + m_row_h);
    }
    if (!row.key.empty())
        draw_chevron(dc, pad + row.indent * FromDIP(INDENT), y, m_row_h, row.collapsed, text_colour);

    wxRect label = label_rect(c, row, y);
    if (row.prefix.empty())
        draw_text(dc, row.text, label, text_colour, option ? ::Label::Body_13 : ::Label::Head_13);
    else {
        // The setting's own name stays readable: the headings before it give way first.
        const wxString name = PATH_SEPARATOR + row.text;
        dc.SetFont(::Label::Body_13);
        const int name_w = dc.GetTextExtent(name).x;
        dc.SetFont(::Label::Head_13);
        const wxString prefix = wxControl::Ellipsize(row.prefix, dc, wxELLIPSIZE_END, std::max(label.width - name_w, label.width / 3), wxELLIPSIZE_FLAGS_NONE);
        const int      prefix_w = dc.GetTextExtent(prefix).x;
        draw_text(dc, prefix, wxRect(label.x, y, prefix_w, m_row_h), text_colour, ::Label::Head_13);
        label.x     += prefix_w;
        label.width -= prefix_w;
        draw_text(dc, name, label, text_colour, ::Label::Body_13);
    }

    auto draw_value = [&](const wxString& value, Part part, const wxColour& colour, const wxFont& font) {
        wxRect cell = part_rect(c, part, y, m_row_h);
        cell.Deflate(pad, 0);
        if (is_color_value(value)) {
            const int side = FromDIP(12);
            dc.SetPen(wxPen(colors.border, 1));
            dc.SetBrush(wxBrush(wxColour(value)));
            dc.DrawRectangle(cell.x, cell.y + (cell.height - side) / 2, side, side);
            cell.x     += side + FromDIP(6);
            cell.width -= side + FromDIP(6);
        }
        if (value.empty())
            draw_text(dc, wxString::FromUTF8("\xE2\x80\x94"), cell, dimmed, font);
        else
            draw_text(dc, single_line(value), cell, colour, font);
    };

    if (option) {
        const SettingsDiffItem& item = item_at(row.first);
        const bool included = m_included[m_order[row.first]];
        // The value under the pointer is outlined: clicking it picks that side.
        for (Part part : { Part::New, Part::Current })
            if (!item.locked && m_hover == Hit{ row_idx, part }) {
                wxRect r = part_rect(c, part, y, m_row_h);
                r.Deflate(FromDIP(3), FromDIP(2));
                dc.SetPen(wxPen(part == Part::New ? (included ? colors.include : colors.exclude) : colors.neutral, 1));
                dc.SetBrush(*wxTRANSPARENT_BRUSH);
                dc.DrawRoundedRectangle(r, FromDIP(4));
            }
        draw_value(item.new_value, Part::New, text_colour, ::Label::Body_13);
        draw_value(item.current_value, Part::Current, text_colour, ::Label::Body_13);
        // A result that leaves the setting as it is recedes.
        if (included)
            draw_value(item.new_value, Part::Result, text_colour, ::Label::Head_13);
        else
            draw_value(item.current_value, Part::Result, dimmed, ::Label::Body_13);
    } else if (row.collapsed) {
        // A folded section says how much it holds; its mark says how much of that is included.
        const int n = int(row.last - row.first);
        draw_value(format_wxstr(_L_PLURAL("%1% setting", "%1% settings", n), n), Part::Current, dimmed, ::Label::Body_13);
    }
    draw_mark(dc, colors, part_rect(c, Part::Toggle, y, m_row_h), range_mark(row.first, row.last),
              toggleable(Hit{ row_idx, Part::Toggle }) && m_hover == Hit{ row_idx, Part::Toggle }, FromDIP(20));

    if (m_key_nav && m_focus_row == row_idx && HasFocus()) {
        dc.SetPen(wxPen(colors.focus, FromDIP(2)));
        dc.SetBrush(*wxTRANSPARENT_BRUSH);
        dc.DrawRectangle(wxRect(rect).Deflate(FromDIP(1)));
    }
}

void SettingsDiffTable::draw_header(wxDC& dc, const Colors& colors, const Columns& c)
{
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(colors.header));
    dc.DrawRectangle(0, 0, GetClientSize().x, m_header_h);

    auto title = [&](const wxString& text, Part part, int shift = 0) {
        wxRect cell = part_rect(c, part, 0, m_header_h);
        cell.Deflate(FromDIP(10), 0);
        cell.x     += shift;
        cell.width -= shift;
        draw_text(dc, text, cell, colors.header_text, ::Label::Body_13);
    };
    if (!m_sections.empty())
        draw_chevron(dc, FromDIP(10), 0, m_header_h, m_collapsed.size() == m_sections.size(), colors.header_text);
    title(_L("Settings"), Part::Label, FromDIP(CHEVRON));
    title(_L("New Value"), Part::New);
    title(_L("Current Value"), Part::Current);
    title(_L("Result"), Part::Result);
    const Hit toggle{ HEADER_ROW, Part::Toggle };
    draw_mark(dc, colors, part_rect(c, Part::Toggle, 0, m_header_h), range_mark(0, m_order.size()), toggleable(toggle) && m_hover == toggle, FromDIP(20));
}

void SettingsDiffTable::draw_footer(wxDC& dc, const Colors& colors)
{
    const wxSize size = GetClientSize();
    const wxRect rect(0, size.y - m_footer_h, size.x, m_footer_h);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(colors.background));
    dc.DrawRectangle(rect);
    dc.SetPen(wxPen(colors.border, 1));
    dc.DrawLine(rect.x, rect.y, rect.GetRight() + 1, rect.y);

    // The legend: what each mark does, by shape as well as by colour.
    const int pad = FromDIP(10), mark_size = FromDIP(16);
    int       x   = pad;
    dc.SetFont(::Label::Body_12);
    const std::pair<Mark, wxString> legend[] = {
        { Mark::Include, _L("Use new value") }, { Mark::Exclude, _L("Keep current value") }, { Mark::Mixed, _L_CONTEXT("Mixed", "Settings diff") } };
    for (const auto& [kind, text] : legend) {
        draw_mark(dc, colors, wxRect(x, rect.y, mark_size, rect.height), kind, false, mark_size);
        x += mark_size + FromDIP(6);
        const int width = dc.GetTextExtent(text).x;
        draw_text(dc, text, wxRect(x, rect.y, width, rect.height), colors.text, ::Label::Body_12);
        x += width + FromDIP(16);
    }
    // TRN Settings diff legend: %1% of the %2% changed settings are saved or transferred.
    const wxString summary = format_wxstr(_L_PLURAL("%1% of %2% included", "%1% of %2% included", included_count()), included_count(), count());
    const int      width   = dc.GetTextExtent(summary).x;
    if (x + width + pad <= size.x)
        draw_text(dc, summary, wxRect(size.x - pad - width, rect.y, width, rect.height), colors.dimmed, ::Label::Body_12);
}

void SettingsDiffTable::on_paint(wxPaintEvent&)
{
    wxAutoBufferedPaintDC paint_dc(this);
    wxGCDC                dc(paint_dc);
    const Colors          colors;
    const Columns         c    = columns();
    const wxSize          size = GetClientSize();

    dc.SetBackground(wxBrush(colors.background));
    dc.Clear();

    const int body_bottom = size.y - m_footer_h;
    dc.SetClippingRegion(0, m_header_h, size.x, std::max(0, body_bottom - m_header_h));
    for (int row = std::max(0, GetViewStart().y); row < int(m_rows.size()) && row_top(row) < body_bottom; ++row)
        draw_row(dc, colors, c, row, row_top(row));
    dc.DestroyClippingRegion();

    draw_header(dc, colors, c);
    draw_footer(dc, colors);

    dc.SetPen(wxPen(colors.border, 1));
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.DrawRectangle(0, 0, size.x, size.y);
}

void SettingsDiffTable::on_left_down(wxMouseEvent& evt)
{
    SetFocus();
    m_key_nav = false;
    const Hit hit = hit_test(evt.GetPosition());
    if (hit.row >= 0)
        m_focus_row = hit.row;
    if (evt.LeftDClick()) {
        // The first click of the pair has done its work; the second shows a setting in full
        // (a heading folds once, a mark switches once).
        if (hit.row >= 0 && hit.part != Part::Toggle && !collapsible(hit))
            show_full_values(hit.row);
    } else if (collapsible(hit))
        toggle_collapsed(hit.row);
    else if (toggleable(hit))
        toggle(hit);
    Refresh();
}

void SettingsDiffTable::on_motion(wxMouseEvent& evt)
{
    const Hit hit = hit_test(evt.GetPosition());
    if (hit != m_hover) {
        m_hover = hit;
        const wxString tip = tooltip_for(hit);
        if (tip.empty())
            UnsetToolTip();
        else
            SetToolTip(tip);
        SetCursor(toggleable(hit) || collapsible(hit) ? wxCursor(wxCURSOR_HAND) : wxNullCursor);
        Refresh();
    }
    evt.Skip();
}

void SettingsDiffTable::on_key_down(wxKeyEvent& evt)
{
    const int page = std::max(1, (GetClientSize().y - m_header_h - m_footer_h) / m_row_h - 1);
    switch (evt.GetKeyCode()) {
    case WXK_UP:       move_focus(m_focus_row - 1); break;
    case WXK_DOWN:     move_focus(m_focus_row + 1); break;
    case WXK_PAGEUP:   move_focus(m_focus_row - page); break;
    case WXK_PAGEDOWN: move_focus(m_focus_row + page); break;
    case WXK_HOME:     move_focus(0); break;
    case WXK_END:      move_focus(int(m_rows.size()) - 1); break;
    case WXK_LEFT:
    case WXK_RIGHT:
        if (!m_rows.empty()) {
            // As in a tree: Left folds a heading or goes up to it, Right opens it or goes on.
            const Row& row  = m_rows[m_focus_row];
            const bool fold = evt.GetKeyCode() == WXK_LEFT;
            if (!row.key.empty() && row.collapsed != fold) {
                m_key_nav = true;
                toggle_collapsed(m_focus_row);
            } else if (!fold)
                move_focus(m_focus_row + 1);
            else if (row.indent > 0) {
                int parent = m_focus_row;
                while (parent > 0 && m_rows[parent].indent >= row.indent)
                    --parent;
                move_focus(parent);
            }
        }
        break;
    case WXK_SPACE:
        if (!m_rows.empty()) {
            m_key_nav = true;
            toggle(Hit{ m_focus_row, Part::Toggle });
            Refresh();
        }
        break;
    case WXK_TAB:
        // wxWANTS_CHARS keeps the arrow keys here, and Tab with them: hand it back.
        Navigate(evt.ShiftDown() ? wxNavigationKeyEvent::IsBackward : wxNavigationKeyEvent::IsForward);
        break;
    default: evt.Skip();
    }
}

}} // namespace Slic3r::GUI
