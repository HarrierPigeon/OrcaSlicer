#pragma once

#include <functional>
#include <set>
#include <vector>

#include <wx/scrolwin.h>

namespace Slic3r { namespace GUI {

// One changed setting shown by SettingsDiffTable.
struct SettingsDiffItem
{
    wxString preset;        // top-level heading; empty when the table lists a single preset
    wxString category;      // settings page
    wxString group;         // option group on that page
    wxString label;
    wxString new_value;
    wxString current_value;
    bool     locked{ false }; // always included: cannot be picked apart from the rest
};

// The changed settings of one or more presets as a diff to pick from: New Value on the left,
// Current Value on the right, a mark between them that includes or excludes the new value, and
// the Result of that choice. Preset, page and group headings carry a mark for everything below
// them and fold away; a heading with a single entry below it shares that entry's line.
//
// The table paints itself, in the app's colours and their dark-mode counterparts. A native tree
// control cannot tint cells per state or draw the same three-state section marks on every
// platform. Include and exclude differ by shape as well as by colour.
class SettingsDiffTable : public wxScrolledCanvas
{
public:
    explicit SettingsDiffTable(wxWindow* parent);

    void   set_items(std::vector<SettingsDiffItem> items);

    size_t count() const { return m_items.size(); }
    bool   included(size_t item) const { return m_included[item]; }
    size_t included_count() const;
    void   set_on_change(std::function<void()> on_change) { m_on_change = std::move(on_change); }

    // Height that shows the rows without scrolling, within sensible bounds.
    int    preferred_height() const;
    void   rescale();

private:
    enum class RowKind { Preset, Category, Group, Option };
    enum class Part { None, Label, New, Toggle, Current, Result };
    enum class Mark { Include, Exclude, Mixed, Locked };

    // The settings grouped by preset, page and group; m_rows is what of it is on show.
    struct Node
    {
        RowKind           kind;
        wxString          text;
        wxString          key;         // identifies a heading while rows come and go
        size_t            first, last; // [first, last) in m_order
        std::vector<Node> children;
    };
    struct Row
    {
        RowKind  kind;        // Option, or the level of the heading
        RowKind  band;        // the topmost heading sharing the line: decides the background
        int      indent;
        size_t   first, last; // [first, last) in m_order
        wxString text;        // the heading(s), or the setting's label
        wxString prefix;      // headings sharing a setting's line
        wxString key;         // of a heading that folds; empty otherwise
        bool     collapsed;
    };
    struct Hit
    {
        int  row{ NO_ROW };
        Part part{ Part::None };
        bool operator==(const Hit& o) const { return row == o.row && part == o.part; }
        bool operator!=(const Hit& o) const { return !(*this == o); }
    };
    struct Columns { int label_w, value_w, toggle_w, new_x, toggle_x, current_x, result_x; };
    struct Colors;

    static constexpr int NO_ROW     = -1;
    static constexpr int HEADER_ROW = -2;

    const SettingsDiffItem& item_at(size_t pos) const { return m_items[m_order[pos]]; }
    std::vector<Node> build_tree(size_t first, size_t last, int level, const wxString& parent_key) const;
    void    rebuild_rows();
    void    flatten(const Node& node, int indent, const wxString& prefix, RowKind band, bool visible);
    bool    collapsible(const Hit& hit) const;
    void    toggle_collapsed(int row);
    bool    toggleable(const Hit& hit) const;
    Mark    range_mark(size_t first, size_t last) const;
    void    set_range(size_t first, size_t last, bool include);
    void    toggle(const Hit& hit);
    void    changed();

    Columns columns() const;
    wxRect  part_rect(const Columns& c, Part part, int y, int h) const;
    wxRect  label_rect(const Columns& c, const Row& row, int y) const;
    int     row_top(int row) const;
    Hit     hit_test(const wxPoint& pos) const;
    void    update_layout();
    void    ensure_visible(int row);
    void    move_focus(int row);
    void    show_full_values(int row);
    wxString tooltip_for(const Hit& hit);

    void    draw_row(wxDC& dc, const Colors& colors, const Columns& c, int row, int y);
    void    draw_header(wxDC& dc, const Colors& colors, const Columns& c);
    void    draw_footer(wxDC& dc, const Colors& colors);
    void    draw_mark(wxDC& dc, const Colors& colors, const wxRect& cell, Mark mark, bool hovered, int size) const;
    void    draw_chevron(wxDC& dc, int x, int y, int h, bool collapsed, const wxColour& colour) const;
    // Returns whether the text had to be shortened to fit.
    bool    draw_text(wxDC& dc, const wxString& text, const wxRect& rect, const wxColour& colour, const wxFont& font) const;

    void    on_paint(wxPaintEvent& evt);
    void    on_left_down(wxMouseEvent& evt);
    void    on_motion(wxMouseEvent& evt);
    void    on_key_down(wxKeyEvent& evt);

    std::vector<SettingsDiffItem> m_items;
    std::vector<bool>             m_included; // per item
    std::vector<size_t>           m_order;    // items grouped by preset, page and group
    std::vector<Node>             m_tree;
    std::vector<Row>              m_rows;
    std::vector<wxString>         m_sections;  // keys of the headings that fold
    std::set<wxString>            m_collapsed;
    std::function<void()>         m_on_change;


    int  m_row_h{ 0 };
    int  m_header_h{ 0 };
    int  m_footer_h{ 0 };
    Hit  m_hover;
    int  m_focus_row{ 0 };
    bool m_key_nav{ false }; // the focused row is outlined only while the keyboard drives the table
};

}} // namespace Slic3r::GUI
