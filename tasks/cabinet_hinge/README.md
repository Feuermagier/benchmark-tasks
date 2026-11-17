# Lift a cabinet door hanging from its top hinge back level, clip the loose bottom hinge onto its plate, shut it

Kitchen and bathroom cabinet doors hang on concealed clip-on cup hinges: each hinge's arm clips onto a mounting plate
screwed inside the cabinet, so a door comes off and goes back on without tools. When a door is knocked or overloaded,
its bottom hinge pops off the plate and the door hangs from the top hinge alone, drooping at its free edge, with the
loose hinge arm sticking out beside its plate. Putting it back is a familiar two-handed job: lift the door's free edge
until it hangs level (only then does the arm's hook meet the plate), and with the other hand press the arm onto the plate
until it clicks; then shut the door to check that it closes. The lift is what lets it clip: with the door level the
hook meets the plate's lug, which the drooping door holds 13 mm away.

The task is the task file `cabinet_hinge.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), with the cabinet's code in `hinge/` and `yampatch.py` beside
it (see [Implementation](#implementation)). It needs no other files or packages to be installed.

**Instruction:** "The cabinet door has come off its bottom hinge. Put it back on and shut the door."

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Size | Material | Where |
|---|---|---|---|
| wall cabinet (carcass), fixed | 30 cm wide x 30 cm deep x 40 cm tall outside, on four 5 cm feet | 16 mm white melamine-faced chipboard, 4 mm hardboard back | open front towards the arms, front edge at x 51 cm, centred at y 0; clamped to the table |
| door | 29.6 x 40 x 1.6 cm, 1.3 kg | melamine-faced chipboard, light oak finish | hinged on the cabinet's left side panel, full overlay, standing open at 90 degrees in front of that panel |
| bar handle | 12 mm bar, 128 mm hole centres, 30 mm off the door | brushed steel | on the door's outer face, 35 mm from its free edge, centred 11 cm above its bottom edge (low, as on a wall cabinet) |
| 2 concealed hinges | 110-degree clip-on cup hinges, 35 mm cup, full overlay (e.g. Blum CLIP top 110° 71B3550) | nickel-plated steel | cups 21.5 mm from the door's hinge edge, 8 cm from its top and bottom |
| 2 mounting plates | cruciform clip-on plates, 0 mm (e.g. Blum 173L6100): 50 mm bar, 10 mm high | nickel-plated steel | on the left side panel's inner face, front ends 37 mm from the front edge, at the hinges' heights (8 and 32 cm above the carcass's bottom) |

**The starting state:** the top hinge is clipped onto its plate. The bottom hinge is unclipped (its release pressed and
the arm lifted off its plate) and swung about 12 degrees off it, its rear 2 cm from the plate. The door hangs from the
top hinge alone and has drooped in its own plane until its free edge is 15 mm lower than level; this moves the bottom
hinge's cup 13 mm into the cabinet, past its plate's front lug. The door hangs still, open at 90 degrees.

**Building it for a real station:** a small flat-pack wall cabinet with a 300 x 400 mm full-overlay door on two
clip-on 110-degree hinges and 0 mm cruciform plates, standing on four 50 mm adjustable feet and clamped to the table
(a wall cabinet sits at a person's chest height; here it stands at the robot's working height). Unclip the bottom hinge,
let the door hang, and measure the free edge's droop: it depends on the hinges' play and the cups' seat in the
chipboard, and should be about 15 mm (adjust the top hinge's depth screw if not).

## Scoring

- **0.5:** the bottom hinge clipped back onto its plate (its arm's hook on the plate's lug and its latch snapped over the
  plate's tail, so it does not come off when pulled).
- **1.0 (success, ends the trial):** clipped, and the door shut: within 3 degrees of closed (its free edge within 15 mm
  of the carcass's front edge), at rest.
- Anything else, including the door shut but still hanging from one hinge, scores 0.

On a real station: look at the bottom hinge (or try to lift it off) and measure the door's free edge from the carcass,
a photo. In simulation the cabinet's joints are read every control step; run with `--scorer task_score`.
`HINGE_DEBUG=1` prints the door's swing, its droop and the bottom arm's angle every 10 steps.

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
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/cabinet_hinge/cabinet_hinge.py \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The cabinet door has come off its bottom hinge. Put it back on and shut the door." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir runs/hinge-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted -E object_jitter=0` (and
`--log-dir runs/hinge-demo`). `inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 2,400 steps (2 minutes at 20 Hz), about 5 times what the scripted demo needs.
- **Seeds and layout:** one fixed layout. The cabinet is a fixed articulation, so seeded resets (`-E object_jitter`)
  do not move anything; every reset unclips the bottom hinge and puts the door back as above. Several epochs in one
  run are fine.
- **Runtime:** the scripted demo succeeds after about 500 steps (25 s simulated) with `task_score` 1.0; the run takes
  about 2 minutes wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: the left arm, pointing forward, pinches the door's free edge half way up and lifts it along the arc it
droops on until the door hangs level. The right arm, pointing into the cabinet with its fingers closed (one above the
other), reaches in past the loose arm's rear and presses it sideways onto the plate with a finger's side until it
clicks on; the press aims 5 cm past the plate, as the arm's position control is soft (a teleoperator pushing firmly
does the same with the leader arm). The left arm lets go; the right arm goes round the door's free edge and pushes the
door shut by its outer face. Two arms are needed, lifting and pressing at once: let go of the door and it droops back,
taking the hook off the lug. The demo's grippers point forward and tilt, which the adapter's waypoints (a
straight-down grip turned about the vertical) do not offer: `hinge/orient.py` adds that for the demo (see below); the
demo is an open-loop list of dense waypoints, like a teleoperated run.

## Simulation notes

- The carcass with both plates and the clipped top hinge's arm is one fixed body. The door hangs on the top hinge as a
  revolute joint about the hinge axis (0.3 N m s/rad of friction) and a second revolute in the door's own plane, the
  droop: its 3 degrees of play on a light 10 N m/rad spring, so the door's weight (its moment about the top hinge,
  about 2 N m) takes it to the droop's end, and lifting the free edge with about 6.7 N brings it back to level.
- The loose bottom arm turns about the hinge axis on the door, on the hinge's own light spring (0.3 N m/rad) holding
  it 12 degrees off its plate. It collides with the plate.
- The clip: the arm's spring-loaded latch lever is modelled as a sprung tongue on the plate's tail (a 12 x 10 mm
  steel slide on a 3,300 N/m spring), standing 3.5 mm off the plate's face, under the arm's rear. When the door hangs
  within 0.35 degrees of level (the arm's hook within 1.5 mm of the plate's lug), the door is held still (swinging under
  5 degrees per second) and the arm is pressed flat onto the plate (within 0.8 degrees, its rear within 1.3 mm),
  pushing the tongue down to 1.1 mm against 8 N, the arm clips on and holds from then on: the droop is held level and
  the arm stays on its plate as the door swings. Clip-on hinges snap on with a firm press; their makers publish no
  force, so 8 N is our estimate. Swinging the door shut does not clip the arm: the door pushes it only through the
  hinge's own light spring (under 5 N at the latch), and the loose arm, caught on its plate, stops the door about 14
  degrees short of shut. The hook's and latch's shapes are not modelled. Unclipping is not modelled; neither is pulling
  the top hinge off.
- Friction 0.6 static, 0.5 dynamic (melamine, steel).

## Implementation

| Path | What |
|---|---|
| `cabinet_hinge.py` | the task file: the layout, scoring, the scripted demo and the text an agent reads |
| `hinge/design.py` | dimensions: carcass, door, handle, hinges and plates, the droop and the clip, scoring tolerances (pure Python) |
| `hinge/usd.py` | the cabinet's articulated USD (`pxr`) |
| `hinge/sim.py` | the cabinet as an Arena articulation, the bottom hinge's clip, and the reset |
| `hinge/orient.py` | forward-pointing and tilted grips for the scripted demo: a waypoint's `yaw` may carry a full gripper rotation; it wraps `yam_arena.demo.grip_rotation` in the running process (the adapter's files are not changed) |
| `yampatch.py` | adds to the task-file format at runtime (without changing the adapter's files): spawning custom Arena objects (the cabinet), a per-step hook (`on_step`) and a reset hook (`on_reset`); it wraps `yam_arena.scene.task_object`, `yam_arena.scene.object_states` and `YamArenaEmbodiment.reset` of the pinned adapter |

The USD file is built at start-up into the system's temporary folder (`yam_tasks/cabinet_hinge`).

## Media

Rendered with `inspect-robots video <log>.json` (top camera, 320 x 176):

- `media/setup.png`: the setup, the demo run's third top-camera frame (the first is rendered before the reset puts the door in its starting state).
- `media/demo.mp4`: the scripted demo, full score (1.0).
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176).

## Licences

Everything (cabinet, door, hinges) is built by the code in this folder; no third-party assets.
