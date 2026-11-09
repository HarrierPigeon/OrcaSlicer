# Motor resonance avoidance — High Level Design

## Purpose and scope

Vertical fine artifacts (VFAs) come from a stepper running at a speed where it, or what it
drives, resonates. Which speeds those are differs per machine and per motor, and on a CoreXY it
depends on the direction of the move as well as its speed: a move along X turns both motors at
1/√2 of the toolhead speed, a move along the 45° diagonal turns only motor A, at the full speed.

Klipper's Shake&Tune measures this: `CREATE_VIBRATIONS_PROFILE` drives each motor on its own
(Cartesian: along 0° and 90°; CoreXY: along 45° and 135°) across a range of speeds and records the
toolhead's vibration. This subsystem reads those recordings, finds each motor's resonant speed
ranges, and slows the moves that would put a motor inside one. It also turns the printer's input
shapers into acceleration limits and the direction-averaged result into the single range used by
the older outer-wall `resonance_avoidance`.

## Settings

- Printer (`MachineEnvelopeConfig`): `motor_resonance_kinematics` (Cartesian or CoreXY: how a
  move's direction maps onto the two steppers) and `motor_resonance_bands_x` / `_y`, text such as
  `45-52, 96-104`. A band's speed is Shake&Tune's motor speed: the toolhead speed of a move along
  that motor's own direction. The X stepper is motor A on a CoreXY, as in Klipper.
- Process (`PrintRegionConfig`): `motor_resonance_avoidance` picks what is slowed: outer walls,
  all walls or all extrusions. It is a region setting so an object can override it, which the VFA
  comparison relies on.

`Print::validate` rejects band text that does not parse. Changing any of these only invalidates the
G-code export step.

## Analysis (`libslic3r/ShakeTune/`)

- `Vibrations`: reads a `.stdata` file (a zstd stream of JSON lines, one recording per line; zstd
  comes from the copy c-blosc bundles) and runs Shake&Tune's machine vibrations analysis, translated
  from its Python with credit: each recording's PSD (Klipper's Welch estimate), the energy per motor
  and speed, the projection of both motors onto every direction, the vibration metric, its peaks,
  the quiet speed and direction ranges and the symmetry. On top of that, each motor's own energy
  curve gets the same peak detection, and the band around each peak is its half-prominence width,
  never narrower than one measured speed step either side. The analysis also runs from the
  per-recording energies alone (`analyze_vibrations_energy`), which is how real printer data is
  kept in the tests.
- `InputShaperLimits`: Klipper's shaper definitions and its smoothing model; the max acceleration
  is where smoothing reaches Klipper's 0.12 mm target, the figure Klipper and Shake&Tune print.
- `KlipperReport`: parses Moonraker's replies: the `.stdata` files under the `config` root (found by
  name, since Shake&Tune's `result_folder` can be any folder there) and the saved configuration's
  kinematics, square corner velocity, input shapers and `[shaketune]` section.
- `MotorResonance`: the band text format, the per-motor speed factors of a move and
  `resonance_safe_speed`: the fastest speed, no faster than the requested one, at which neither
  motor is inside a band. Candidates are band lower edges mapped back to toolhead speed. A move that
  could only clear the bands below half its speed is left alone.

## G-code (`GCode/MotorResonanceAvoider`)

Speeds are only final after the `CoolingBuffer`, which rescales feedrates to meet the minimum
layer time and could put a motor back into a band. So the avoider is a pipeline stage after cooling
and before adaptive pressure advance, which reads feedrates. `GCode::_extrude_piece` brackets the moves of
each path whose role the region's scope covers with `;_MOTOR_RESONANCE_START` / `_END`. Between them
the avoider rewrites the feedrate of each G1 extrusion move from the move's direction, writes the
original feedrate back on the next move that relies on it, and drops the markers. Arcs, travel,
retraction and moves outside a bracket keep their speed. With no bands the avoider is not created
and no markers are written.

## Import (`GUI/ShakeTuneImportDialog`)

The printer settings' Motion ability page opens the dialog. Through the printer's network host it
lists the vibrations recordings on the printer, newest first, downloads and analyses the chosen one
on a worker thread, and charts each motor's energy against speed with its bands. A local `.stdata`
file works without a host. Each proposed change is a checkbox showing the current and new value:
the motor layout, both stepper band lists, the X and Y acceleration limits from the input shapers
the printer runs (Klipper flavour caps print acceleration at the lower of the two), and the
outer-wall range around the strongest direction-averaged peak (off by default).

The printer must keep the raw recordings: `keep_raw_data: True` in `[shaketune]`. Shake&Tune
deletes them once the graph is drawn otherwise.

## VFA test

The VFA tower can print with avoidance disabled, enabled, or both side by side. Vase mode prints
one object, so the comparison prints two single-wall towers without it, the right one with
`motor_resonance_avoidance` set to outer walls on the object.
