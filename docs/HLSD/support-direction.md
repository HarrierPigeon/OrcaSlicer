# Support direction

Support normally stacks straight along the slice frame's Z: each support layer is the layer above
projected down, so a column is always perpendicular to the layers. `support_direction` (per object,
Support page, advanced) lets the column lean along a direction fixed in the printer's frame instead:

| value      | support axis                                                                                              |
|------------|-----------------------------------------------------------------------------------------------------------|
| `layers`   | the layer normal (default, stock behaviour)                                                               |
| `vertical` | the bed normal, +Z of the frame the object is placed in                                                   |
| `custom`   | the bed-frame direction given by `support_direction_tilt_x/y` (`Geometry::tilt_up_direction`), in the convention of the `build_plate_tilt_x/y` printer options: up = normalize(tan(tilt_y), tan(tilt_x), 1), so 0/0 is vertical and the same tilts as the build plate grow the support against the plate's gravity |

`support_direction_max_lean` limits the lean in effect, whatever asked for it.

On a flat bed the bed normal is the layer normal, so `vertical` changes nothing. The point is the
belt printer: `BeltSliceStrategy` rotates the object by the belt angle before slicing, so the layers
are tilted and a column that follows them climbs away from the belt at that angle. With `vertical`
the direction is the belt normal in the slice frame; the column grows perpendicular to the belt and
lands on the belt plane. A wall that is vertical on the bed is then perpendicular to the support
direction rather than a 45-degree overhang, so it needs no support.

## How the lean is produced

None of the three generators (normal, classic tree, organic tree) knows about the direction.
`Support/SupportDirection.hpp` resolves the configured bed-frame direction into the object's slice
frame (`support_lean()`, through the same rotation the slices went through) and then, for the
duration of `_generate_support_material()`, `ScopedSupportShear` translates every layer's geometry
in place by `-z · (u.xy / u.z)`, where `u` is the unit support axis. That shear maps a column along
`u` onto a vertical one, so the unmodified generator, run against the sheared object, produces
exactly the leaning support. The scope then translates the object layers back and the generated
support layers by the inverse amount at their own `print_z`, including on an exception. A shear
preserves z, so layers stay layers and an object/support contact at a given height stays a contact.

Things that read object geometry during generation see the sheared frame consistently:

- `PrintObject::slice_support_volumes()` shears the painted enforcer/blocker slices the same way;
- the generators' own build-plate tilt shift (the belt overhang compensation) is skipped while a
  lean is active, so the tilt is not counted twice;
- `BeltFloorContext` is re-expressed through the same slope, so a leaning column still terminates
  on the belt rather than on where the belt would be in the un-sheared frame;
- the belt surface counts as support from underneath in every generator's overhang detection, and a
  small island resting on it is not a sharp tail. Classic tree support always did this; the normal
  and organic generators do it only while a lean is active, because the shear flattens the object's
  belt-contact face from the belt angle to `atan(shear / (1 + shear²))` (26.6° on a 45° belt), under
  the usual overhang threshold, and that face would otherwise read as an overhang on every layer.

`PrintObject::support_lean()` reports the lean only while the scope is alive, so nothing outside
support generation can mistake sheared geometry for the slice frame; afterwards it is inactive and
the free function `support_lean(const PrintObject &)` still tells what was applied.
`support_direction_bed_frame()` is the shared resolution of the options into a bed-frame vector;
the 3D scene draws it as an arrow above each selected instance whose object leans its support.

## Constraints and consequences

- **Overhangs are judged against the support direction, not the layer normal.** In the sheared
  frame a face's inclination is its inclination relative to `u`. That is what makes `vertical`
  mean "support what would fall" on a belt.
- **The lean is limited by `support_direction_max_lean`** (45° by default, which admits a 45° belt
  exactly; a steeper belt needs a higher limit). A leaning column
  overhangs itself by exactly the lean angle, so past `SupportLean::safe_lean_deg()` (45°) it
  needs tuning and test prints, and generation warns. A direction beyond the limit keeps its
  azimuth and is pulled back to the limit, with a warning; a direction pointing straight down has
  no azimuth and falls back to following the layers. Tree branch angles add to the lean.
- The tree generators clip support to the bed outline as a single polygon; in the sheared frame
  that outline is off by the shear at height, so a leaning column near the bed edge can be trimmed
  a little early or late.
- Per-part (model volume) overrides of the direction are not plumbed: the object's
  `PrintObjectConfig` decides.
