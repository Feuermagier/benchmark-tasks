# Change a stopped wall clock's AA cell: the flat one into the used-battery tray, the new one pressed in + to +

A quartz wall clock that has stopped needs a new cell: almost everyone has taken a clock off the wall, laid it face
down, pried the flat AA cell out of the holder on the movement, and pressed a new one in the right way round. The cell
sits between a + contact and a - spring. The new cell goes in - end first: pressed down onto the spring until the
spring gives way, it snaps in between the contacts. The flat cell goes into a used-battery tray for recycling. The new cell lies the other way round from the holder, so it has to be turned
to put + to the + mark.

The task is the task file `clock_battery.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), with the clock's code in `clock/` and `yampatch.py` beside it
(see [Implementation](#implementation)). It needs no other files or packages to be installed.

**Instruction:** "The wall clock has stopped. Put the new battery in."

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Size | Material | Where |
|---|---|---|---|
| wall clock, loose, face down | 30 cm round, 4.5 cm deep, 0.45 kg | dark-brown moulded plastic case, glass face (down) | centred at (32, -6) cm |
| quartz movement, on the clock's back | 5.6 x 5.6 x 1.6 cm | black plastic | centre of the clock's back |
| flat AA cell, in the holder | 14.5 mm x 50.5 mm (IEC LR6), 23 g | grey-blue wrap, silver + end | seated, + end towards +x (away from the arms) |
| new AA cell, in its opened blister pack | 14.5 mm x 50.5 mm, 23 g | black wrap, copper band at the + end | centred at (17, -25) cm, lying along y, + end towards +y (left) |
| opened blister pack, fixed | card 6.5 x 10 x 0.15 cm with two clear saddles | orange printed card, clear plastic | centred at (17, -25) cm |
| used-battery tray, fixed | 10 x 8 x 3 cm outside, 3 mm walls | green plastic, floor labelled USED BATTERIES | centred at (43, -27) cm |

**The battery holder**, on top of the movement, along x: a + contact block at +x and the - spring at -x, 47 mm apart
at rest (a seated cell compresses the spring 3.5 mm); the cell rests on two 5 mm saddles, 4 mm wide, under its ends
(45 mm apart centre to centre), each with a 1.5 mm lip on either side, so fingers reach under its middle. The movement's
top is printed + at the + end and - at the spring end, 1.5V and AA between. The spring is a sprung contact with a 45°
chamfer on top that a cell pressed down onto pushes back; it gives 8 mm of chamfer and 10 mm of travel. The new cell's
pack has the same two saddles, along y.

**Building it for a real station:** a 30 cm plastic quartz wall clock with a standard 56 mm movement (AA holder with
end contacts), lying face down on a cloth; two AA cells (one flat, one new) and an opened blister card; a small green
tray. The clock may be taped to the table if it should not move.

## Scoring

- **0.4** for the flat cell in the used-battery tray: its centre inside the tray's walls and below its rim.
- **0.6** for the new cell seated in the holder + to +: its centre within 4 mm of the holder's centre along and
  across the holder and within 3 mm in height, its axis within about 20° of the holder's, + end at the + contact.
  Seated the wrong way round earns nothing.
- **1.0 (success, ends the trial):** both, with both cells at rest.

On a real station: look at the holder (which cell, which way round, seated between the contacts) and the tray, a
photo. In simulation the cells' poses are read every control step; run with `--scorer task_score`.
`CLOCK_DEBUG=1` prints the cells' positions and the spring every 10 steps.

## Running

Needs Linux with an NVIDIA RTX GPU (Isaac Sim needs RT cores) and about 200 GB of disk. Install the pinned
adapter, which installs Isaac Lab-Arena and Inspect Robots (from the root of this repository):

```bash
mkdir -p .deps
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git .deps/yam-isaac-lab-arena
git -C .deps/yam-isaac-lab-arena checkout 0059d02a1a91bc6834c1d0846022891df0c78730
ARENA_DIR="$PWD/.deps/IsaacLab-Arena" .deps/yam-isaac-lab-arena/setup.sh
```

In each shell:

```bash
source .deps/IsaacLab-Arena/yam.env
export ISAACSIM_ASSET_ROOT=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```

Dream-1 (from the repository root):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/clock_battery/clock_battery.py \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The wall clock has stopped. Put the new battery in." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir runs/clock-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted -E object_jitter=0` (and
`--log-dir runs/clock-demo`). `inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 1,200 steps (1 minute at 20 Hz), about 3 times what the scripted demo needs.
- **Seeds and layout:** one fixed layout. The clock is an articulation and the pack and tray are fixed; seeded resets
  (`-E object_jitter`) may move the new cell up to 3 cm off its pack, and every reset puts the clock and the flat cell
  back in the holder. The open-loop scripted demo is run with `-E object_jitter=0`. Several epochs in one run are fine.
- **Runtime:** the scripted demo succeeds after about 420 steps (21 s simulated) with `task_score` 1.0; the run took
  about 2 minutes wall-clock on one RTX 5090 shared with another simulation job, including Isaac Sim start-up.

The demo: the left arm presses the clock's back down near its rim, as a person steadies the clock. The right arm
pinches the flat cell across its middle (fingertips under it, between the saddles), lifts it out and drops it in the
tray; pinches the new cell in its pack, turns it a quarter turn so its + end faces the + contact, lowers it until its
- end rests on the spring's chamfer and lets go; then, with its fingertips 10 mm apart straddling the top of the - end,
presses it down past the spring, which gives way, and the cell snaps in. The left arm's hold is how a person steadies
the clock; in the simulation the clock also stays put without it (the demo scores 1.0 with the right arm alone).

## Simulation notes

- The clock case is one rigid convex 32-sided disc with the movement, the holder's + contact block, saddles and the
  spring's backing block fixed to it. Friction: 0.3 static for the clock and holder, 0.8 for the cells' wraps.
- The - spring is a sliding metal plate on a 150 N/m drive (preload 0.15 N), with an upright face towards the cell and
  a 45° chamfer on top, standing in for the coil spring of a real holder, which a cell pressed down onto also pushes
  back. The + contact is a rigid block. Nothing electrical is modelled: the clock does not tick; a cell seated + to +
  counts.
- Cells are rigid cylinders with a + nub; the pack's card and saddles are fixed; the tray is fixed.

## Implementation

| Path | What |
|---|---|
| `clock_battery.py` | the task file: the layout, scoring, the scripted demo and the text an agent reads |
| `clock/design.py` | dimensions: clock, movement, holder and spring, AA cell, pack, tray, scoring tolerances (pure Python) |
| `clock/usd.py` | the clock's articulated USD with printed marks, the cells, the pack and the tray (`pxr`, PIL) |
| `clock/sim.py` | the clock as an Arena articulation, its pose for scoring, and the reset of the clock and the flat cell |
| `yampatch.py` | adds to the task-file format at runtime (without changing the adapter's files): spawning custom Arena objects (the clock) and USD files (cells, pack, tray), a per-step hook (`on_step`) and a reset hook (`on_reset`); it wraps `yam_arena.scene.task_object`, `yam_arena.scene.object_states` and `YamArenaEmbodiment.reset` of the pinned adapter |

The USD files and textures are built at start-up into the system's temporary folder (`yam_tasks/clock_battery`).

## Media

Rendered with `inspect-robots video <log>.json` (top camera, 320 x 176):

- `media/setup.png`: the setup, the demo run's first top-camera frame.
- `media/demo.mp4`: the scripted demo, full score (1.0).
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176).

## Licences

`assets/fonts/`: DejaVu Sans Bold and DejaVu Sans Mono Bold, Bitstream Vera licence (see `assets/fonts/LICENSE`).
Everything else (clock, cells, pack, tray) is built by the code in this folder.
