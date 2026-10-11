# Canine Gait Lab

A native desktop simulator for studying how dogs of different breeds and sizes walk, trot, pace,
canter, gallop and jump. It animates the dog from the **side** (sagittal plane), the **front**
(frontal plane) or from **above**, all driven by one 3-D model. Underneath, it plots the motion as
functions of the gait cycle: joint angles, angular velocity, footfall pattern, centre-of-mass
motion, trunk pitch and roll, limb ab/adduction, estimated vertical and mediolateral ground
reaction forces, and paw paths. It also imports measured kinematics as CSV so you can overlay them on the
model and quantify the difference.

> **The built-in motion is a model, not measured data.** It is useful for teaching, for planning
> studies and for generating hypotheses. Do not report its numbers as observations; validate any
> conclusion against motion-capture or force-plate data. The app labels which is which at all times.

## Run

Requires Rust (stable, edition 2024).

```sh
cargo run --release
```

Open it preconfigured:

```sh
canine_gait_lab --breed greyhound --gait rotary_gallop --fr 12 --compare "great dane" --tab footfall
canine_gait_lab --breed "border collie" --gait standing_jump --obstacle-cm 45
canine_gait_lab --breed labrador --gait pace --view front --tab frontal
```

`--view` takes `side`, `front` or `top`.

Batch-export model series without the GUI:

```sh
canine_gait_lab --list-breeds
canine_gait_lab --export out.csv --breed beagle --gait walk --fr 0.2 --height-cm 38 --hz 200 --cycles 3
```

`cargo test --release` runs the model checks (see **Verification** below).

## What you can do

- **Subject:** ten breed presets (Chihuahua to Great Dane, including chondrodystrophic Dachshund
  and brachycephalic French Bulldog). You can scale any of them to a different withers height;
  body mass scales with height³.
- **Gait:** stand, walk (lateral sequence), trot, pace, canter, transverse gallop, rotary gallop,
  standing jump and running jump. Speed is set as a **Froude number** (Fr = v² / g·h, with h the
  hip height), so two dogs at the same Fr move at dynamically similar speeds.
- **Playback:** 0.02× to 2× speed, 5 ms frame stepping (200 Hz), and a cycle scrubber. Clicking
  or dragging in any chart also moves the cursor.
- **Canvas:** body silhouette, a skeleton overlay with live joint angles, trails for the paw,
  carpus/hock and COM, ground-contact bars, GRF arrows, a centimetre grid and distance ticks.
  Scroll to zoom. A red ring marks a limb that is fully extended.
- **Views:** switch between **Side | Front | Top** in the top bar, or press **V** to cycle.
  - *Front:* the dog walks toward you, with its left on screen right. It shows trunk roll
    (girdle tilt), lateral COM sway against the line of travel, live ab/adduction labels,
    vertical + ML GRF arrows, frontal paw loops, and dimension lines for the fore and hind step
    width.
  - *Top:* footprints from the last two strides (tracking and step width), the support polygon of
    the paws on the ground, the COM projection (with a red ring when it falls outside the
    polygon) and ML GRF arrows.
- **Compare:** a second breed in its own lane or overlaid as a ghost, at the same pixel scale so
  size differences are true. The clock can run in real time or be synchronised by gait phase.
- **Charts:**
  - Joint angles or angular velocity for all 12 joints, as small multiples.
  - Hildebrand-style footfall diagram.
  - COM height, vertical velocity, forward velocity and trunk pitch.
  - Fore, hind and total vertical GRF.
  - Frontal plane: limb ab/adduction, lateral COM position, trunk roll and ML GRF.
  - Paw paths relative to the girdles, sagittal or frontal.
  - A metrics table: stride length and frequency, duty factor, footfall phases, peak GRF, COM
    excursion, pitch range, step width, lateral sway, roll range, peak ML GRF, and joint range of
    motion (including ab/adduction) for every limb.
- **Data:**
  - Import CSV to overlay measured curves as dashed lines.
  - Per-joint RMSE, mean difference and Pearson r between your data and the model.
  - Optionally drive the animation from the imported angles (forward kinematics).
  - Export the model series at any sampling rate.

## Conventions

- **Coordinates:** metres. x is cranial (direction of travel), y is up and z is lateral, positive
  toward the dog's **left**. In the side view the viewer sees the dog's left side, so LF and LH
  are the near limbs.
- **Ab/adduction:** the tilt of the plane each limb flexes in (it contains the shoulder or hip
  joint and the paw), measured relative to the rolled trunk. Positive means abducted (paw away
  from the midline). Sagittal joint angles are measured inside that plane.
- **Roll:** rotation of the trunk about its long axis, positive when the left side is up.
- **Joint angles:** included angle on the **flexor side**, in degrees. 180° means the segments are
  straight; values above 180° mean hyperextension. For example, the carpus is about 190–200° under
  load.
  - Shoulder: scapula vs humerus, caudal side.
  - Elbow: humerus vs radius/ulna, cranial side.
  - Carpus: radius vs metacarpus, palmar side.
  - Hip: pelvic axis (hip → iliac crest) vs femur, cranial side.
  - Stifle: femur vs tibia, caudal side.
  - Tarsus: tibia vs metatarsus, dorsal side.
- **Gait cycle:** 0 % is the reference touchdown (left hind for symmetric gaits, trailing right
  hind for the left-lead canter and gallops). Jumps are plotted against time.
- **GRF:** body weights (%BW in charts). ML GRF is positive toward the dog's left.

## CSV format

One row per sample, comma-separated, with a header row. Lines starting with `#` are comments, and
blank cells count as missing.

| Column | Meaning |
|---|---|
| `cycle_pct` **or** `time_s` | Position in the stride. With only `time_s`, the file must hold exactly one stride and is normalised to 0–100 %. |
| `LF_shoulder_deg`, `LF_elbow_deg`, `LF_carpus_deg`, and the same for `RF` | Forelimb joint angles |
| `LH_hip_deg`, `LH_stifle_deg`, `LH_tarsus_deg`, and the same for `RH` | Hindlimb joint angles |
| `{LF,RF,LH,RH}_contact` | 1 = stance, 0 = swing (optional) |
| `{LF,RF,LH,RH}_grf_bw`, `com_y_m`, `pitch_deg` | Optional |
| `{LF,RF,LH,RH}_abd_deg`, `{LF,RF,LH,RH}_grf_ml_bw`, `{LF,RF,LH,RH}_paw_z_m`, `com_z_m`, `roll_deg` | Optional, frontal plane (overlaid on the Frontal charts) |

Other columns are listed as ignored. Parse errors give the line number and the column.

If `cycle_pct` wraps around (several strides in one file), the strides are pooled and plotted as
points.

Exports use the same names, so an export can be re-imported. `samples/example_trot_labrador.csv`
is a **synthetic demonstration file**: model output plus 1.5° pseudo-random noise, labelled as such
in its header. Use it to try the import, overlay and statistics.

## Model

All of the following is in `src/model/`. Every constant is named and commented.

1. **Skeleton (`breed.rs`).**
   - Each preset gives relative segment lengths (scapula, humerus, radius, metacarpus, femur,
     tibia, metatarsus) and standing segment orientations.
   - Lengths are scaled so the standing forelimb reaches the ground from the scapular pivot at
     93 % of withers height.
   - The COM sits over the point dividing the paw base in the fore/hind weight split (default
     60/40).
   - Presets live in `assets/breeds.json`. If that file exists in the working directory, it is
     read at start-up, so you can add or adjust subjects without recompiling.
2. **Gait timing (`gait.rs`).**
   - Footfall phase offsets come from Hildebrand's gait formula. Duty factor falls with speed
     inside each gait.
   - Stride length follows Alexander & Jayes' (1983) dynamic-similarity relation,
     λ/h ≈ 2.3·Fr^0.3. Stride period is λ/v.
   - If stance would exceed limb reach, the duty factor is shortened, and the app says so.
3. **Limb kinematics (`kinematics.rs`, `sim.rs`).**
   - In stance, each paw is fixed to the ground (no slip; tested).
   - In swing, the paw follows a trunk-relative cubic profile with a clearance bump.
   - The distal segment (metacarpus/metatarsus) follows a phase template with extra
     hyperextension under load.
   - Two-link inverse kinematics solves the elbow (bending caudally) and the stifle (bending
     cranially).
   - Scapular rotation follows limb protraction. The scapula can glide on the thoracic sling and
     the pelvis can give at the lumbosacral joint. If stance still over-extends a limb, the trunk
     is lowered just enough, and the lowering is reported in Metrics.
4. **Forces and COM (`grf.rs`, `com.rs`).**
   - Each stance has a vertical GRF profile: half-sine for running gaits, double-hump for walking.
     Profiles are scaled so the stride impulse equals body weight × period, split fore/hind.
   - COM vertical motion and trunk pitch come from integrating those forces over the cycle
     (impulse–momentum), so the COM curves are consistent with the force curves.
   - Fore–aft horizontal forces are not modelled.
5. **Frontal plane (`frontal.rs`, `com.rs`, `sim.rs`).**
   - Presets add `chest_width`, `fore_track` and `hind_track` (all / withers height; optional in
     the JSON, with defaults 0.30 / 0.26 / 0.24). Shoulder and hip joints sit at 45 % and 40 % of
     half the chest width from the midline.
   - Step width narrows with speed as the paws converge toward the midline (single tracking):
     `track · (floor + (1 − floor)·e^(−Fr/k))`. The floor depends on the gait: pacing keeps a wider
     base than trotting or galloping. During swing, the paw adds a small outward arc.
   - Each limb flexes in its own plane through the proximal joint and the paw, tilted by
     ab/adduction. The sagittal two-link solution runs inside that plane, so segment lengths stay
     exact (tested) and paws stay fixed in all three axes during stance.
   - ML GRF is taken along the line from the paw toward the COM:
     `−F_y · (z_paw − z̄) / h_COM`, where `z̄` is the load-weighted mean paw position, which gives
     zero ML impulse per stride. Lateral COM sway comes from integrating it. Roll comes from
     integrating the vertical-load moment about the long axis, as pitch does, with a radius of
     gyration of 0.8 × withers height. The result is large sway and roll in walk and pace and
     little in trot, where the diagonal pairs cancel.
6. **Jumps (`jump.rs`).**
   - Phases: crouch or gather, constant-acceleration push-off, exact ballistic flight,
     forelimb-first landing with constant deceleration, then recovery steps.
   - Take-off vertical speed is √(2g·Δh), from the apex height needed to clear the obstacle with
     the tucked paws.
   - GRF follows from F = m(ÿ + g) and is shared among the paws on the ground.

### Known limitations

- The frontal-plane layer is a first-order extension. Track widths and the speed-convergence
  constants are approximations, limb axial rotation and trunk yaw are not modelled, and jumps are
  treated as bilaterally symmetric (no sway or roll).
- ML GRF assumes a force line through the COM. Read its magnitude as an estimate.
- Joint-angle templates are qualitative shapes chosen to look and behave plausibly. They are not
  fitted to a dataset, so read curve shapes as indicative, not normative.
- Breed proportions are approximations from breed standards. Replace them with your subjects'
  morphometrics for real use.
- COM excursion in the canter is probably overestimated, because horizontal forces and spinal
  energy storage are not modelled.
- The jump is a planned trajectory, not an optimal-control or muscle-driven solution.

Literature cited in the code comments (Hildebrand 1965, 1977; Alexander & Jayes 1983; Maes et al.
2008; Hudson et al. 2012; Flash & Hogan 1985) gives the *form* of the relations used. Check the
primary sources before citing any constant.

## Verification

`cargo test --release` checks:

- Inverse/forward kinematics round trip, anatomical bend direction and standing joint angles.
- Stance fraction equals the duty factor for every gait.
- No paw slip during stance across breeds and gaits.
- Stride GRF impulse equals body weight, with the correct fore/hind split.
- The same Froude number gives the same dimensionless stride across sizes.
- The jump apex reaches the planned height within 1 %, and no limb is loaded in flight.
- Every breed and gait combination produces finite values.
- Frontal plane: no lateral paw slip in stance, and 3-D segment lengths are preserved. In
  symmetric gaits, sway and roll are half-cycle mirror images with zero mean. ML GRF impulse is
  zero over a stride (canter included). Pace sways and rolls more than trot, and step width
  narrows with speed.
- CSV export → import round trip, and parser error reporting.

## Layout

```
src/main.rs            entry point, CLI
src/app.rs             UI state and layout
src/plots.rs           chart panels
src/render/dog.rs      side view, scene and shared drawing helpers
src/render/front.rs    front (frontal-plane) view
src/render/top.rs      top (plan) view: footprints, support polygon
src/model/             biomechanical model
src/io/                CSV import / export
assets/breeds.json     editable breed presets
samples/               synthetic example data
```
