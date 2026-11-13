# Settings diff

When a preset with unsaved changes is about to be left behind (switching printer, process or
filament, opening or creating a project, closing the application), the changes are shown as a
diff to pick from. A project made by someone else arrives as exactly such a set of changes
against the user's own presets, so being able to take some of them and leave the rest is the
point.

The dialog is `UnsavedChangesDialog`; the table in it is `SettingsDiffTable`
(`src/slic3r/GUI/SettingsDiffTable.{hpp,cpp}`). The same dialog also copies per-nozzle settings
between the tools of a printer, with a grid of its own (see
[per-nozzle-settings.md](per-nozzle-settings.md)); the table is not used there.

## What the table shows

One row per changed setting, in the order of the settings pages, grouped by page and option
group, and by preset type when several presets are listed at once:

| Column        | Content                                                               |
|---------------|-----------------------------------------------------------------------|
| Settings      | the setting, under its page and group                                 |
| New Value     | the modified value                                                    |
| mark          | include (arrow) or exclude (cross) the new value                      |
| Current Value | the value the saved preset holds                                      |
| Result        | the new value where included, otherwise the current value, greyed out |

Every setting starts included. A setting is switched by its mark or by clicking the value
wanted. Page, group and preset rows and
the header carry a mark for everything below them: an arrow when all of it is included, a cross
when none is, a dash when mixed. Clicking a mixed mark includes everything first.

Headings fold: click one (or press Left / Right on it) to hide or show what is below it, and the
arrow in the table header folds or opens them all. A folded heading keeps its mark and says how
many settings it holds.

A heading with a single entry below it shares that entry's line, the way a file tree shows
`src/app` for a folder holding only one folder: a page with one group reads `Page / Group`, and
a lone setting takes one row, `Page / Group / Setting`, with its values on it. Such a row
keeps the band of its topmost heading, so every page is still one band in the list. When the
line is too narrow the headings are shortened before the setting's own name.

A changed extruder count is always included. It resizes every per-extruder option, so it cannot
be left out on its own. So is a change the saved preset has no value to go back to: an option
it lacks, or one value of a vector it holds empty.

## What the choice does

- **Save** writes the included values to the preset; excluded settings are put back to their
  current value first.
- **Transfer** carries the included values to the preset being switched to (or keeps them for
  the new project). Excluded settings take whatever that preset holds, which the Result column
  does not know: it shows the current value of the preset being left. When the edited preset
  itself comes through the switch (a dependent preset that stays compatible, a preset kept for
  a new project), the excluded changes are put back to the saved values at that point
  (`Tab::cache_config_revert`, applied with the cache).
- **Discard** drops everything, whatever is included.
- **Cancel**, like closing the window, goes back: whatever was about to happen does not, and
  the changes stay as they were.

Save and Transfer are disabled while nothing is included.

A per-extruder option appears as one row per extruder, keyed `<option>#<index>`, and those keys
go through to `ConfigBase::apply_only`, which sets a single index. One extruder's retraction can
therefore be taken without the other's.

Dirty options that have no row (they are not presented in the settings tabs, such as
`printer_settings_id`) are not part of the choice: they are saved with the preset,
never transferred.

## Colours

The table paints itself at paint time from colours the app already uses for the same purpose
(window background, button and tab backgrounds, label and disabled text, input borders, the Orca
colour), mapped through `StateColor::darkModeColorFor`, so it follows light and dark mode. Text is
held to a 4.5:1 contrast against the cell it is drawn on (`StateColor::EnsureContrast`).

Include is green and exclude red, each with a lighter variant for the dark base and at least 3:1
against the table background. Colour is never the only signal: the marks differ in shape, an
included result is bold and an excluded one greyed.

## Keyboard

The table takes focus as one control. Up, Down, Page Up, Page Down, Home and End move between
rows, Left and Right fold and open the focused heading (Left elsewhere goes up to the heading
above), Space switches the focused row or section, and Tab leaves the table. Right-click a
setting, or double-click one of its cells, to see long values in full.
