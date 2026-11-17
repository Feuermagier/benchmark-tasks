# Put a 1U server's front bezel back on: right end's hook tabs into the rack ear, then swing it shut until it latches

Rack servers wear a front bezel over their drive bays: a locking plastic cover that keeps dust and hands off the
hot-swap drives. Data-center technicians take it off for every drive swap and put it back afterwards, the way the
servers' manuals describe: hook the bezel's right end into the right rack ear (two tabs into two slots), then swing its
left end in until its spring latch clicks into the left ear.

The task is the task file `server_bezel.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (YAM arms at the calibrated `station` in
Isaac Lab-Arena 0.3.1 / Isaac Lab 3.0 / Isaac Sim 6.1), with the rack's code in `bezel/` and `yampatch.py` beside it
(see [Implementation](#implementation)). It needs no other files or packages to be installed.

**Instruction:** "The drive swap is done. Put the server's front bezel back on."

## Setup

Coordinates: x forward from the line between the two arm bases, y to the left, from the midpoint between the bases.

| Object | Size | Material | Where |
|---|---|---|---|
| open-frame rack, fixed | 6U, 19-inch, 50 cm deep, 30 cm tall: four 16 mm posts, base and top frames | black powder-coated steel | front posts at x 42 cm, centred at y 0; clamped to the table |
| 1U server, fixed in the rack | short-depth chassis 43.4 x 45 x 4.3 cm, with two rack ears (2.4 cm wide) bolted to the front posts | grey steel | at U4: its front 17 cm above the table (centre), front face at x 42 cm; four 2.5-inch drive bays across its front |
| front bezel | 43 x 4.2 x 2.5 cm, 0.25 kg; two hook tabs (8 x 6 x 8 mm) on the back of its right end, a spring latch with a release button in its left end, a key lock | black plastic, grey grille | lying face up on the table, its long axis along y, centred at (25, 0) cm |

**The ears:** the right ear has two hook slots, 24 mm apart, for the bezel's tabs; the left ear has the latch's catch.
**Building it for a real station:** any 1U server with a hook-and-latch front bezel (most major brands' 1U servers have
one), in a small 6U open-frame rack on the table, with the bezel unlocked.

## Scoring

- **0.4:** the bezel's right end hooked into the right rack ear: both tabs in their slots, the bezel hanging on them.
- **1.0 (success, ends the trial):** the bezel latched shut on the server's front, at rest.
- Anything else (the bezel anywhere else, or held against the front but not hooked) scores 0.

On a real station: a photo of the server's front (the bezel flush between the ears) and a light tug on its left end (it
stays on). In simulation the bezel's state is read every control step; run with `--scorer task_score`.
`BEZEL_DEBUG=1` prints the bezel's offset from its place and its angles every 10 steps.

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
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/server_bezel/server_bezel.py \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The drive swap is done. Put the server's front bezel back on." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir runs/bezel-dream
```

Scripted full-score demo: the same command with `--policy yam_scripted -E object_jitter=0` (and
`--log-dir runs/bezel-demo`). `inspect-robots video <log>.json` renders the stored frames to MP4.

- **Run limit:** 1,800 steps (90 s at 20 Hz), about 6 times what the scripted demo needs.
- **Seeds and layout:** one fixed layout. The rack and bezel are one fixed articulation, so seeded resets
  (`-E object_jitter`) do not move anything; every reset puts the bezel back on the table as above. Several epochs in
  one run are fine.
- **Runtime:** the scripted demo succeeds after about 280 steps (14 s simulated) with `task_score` 1.0; the run takes
  about 2 minutes wall-clock on one RTX 5090, including Isaac Sim start-up.

The demo: both arms, pointing down, pinch the bezel across its height 4.5 cm from each end, lift it and turn it upright
together (its face towards the arms, the grippers turning to point into the rack) in front of the server; swing its
left end 12 degrees out, push its right end in so the tabs go into their slots, swing the left end in until the latch
clicks, and let go. Why two arms: the bezel is 43 cm long and 2.5 cm thick, so turning it from face up to upright
and keeping it square within 3 degrees while its right end goes into the ear, its left end held 12 degrees out, takes
a grip at each end. Once its tabs are in, the two arms do different things at once: the right arm stays at the ear,
keeping the right end square, while the left arm swings the left end in on a 38 cm radius, its gripper turning with
the bezel. The scorer does not check which arm does what. The
demo's grippers point forward, which the adapter's waypoints (a straight-down grip turned about the vertical) do not
offer: `bezel/orient.py` adds that for the demo (see below); the demo is an open-loop list of dense waypoints, like a
teleoperated run. While the bezel is upright it aims 8 mm high, as the bezel sits that much low in the arms' grip.

## Simulation notes

- The rack, the server and its ears are one fixed body. The bezel is a rigid box (with visual grille, lock, release
  button and tabs) that moves freely: in the articulation it hangs on a chain of joints at its hook pivot (slides along
  x, y and z, turns about y and x, and its swing about the vertical) with no drives and 5 g bodies, which behaves as a
  free body; it collides with the table, the rack and the server.
- The hook: when the bezel's hook pivot is within 3 mm of its place, the bezel within 3 degrees of upright and square,
  and its left end swung out between 5 and 40 degrees (the tabs go into their slots at an angle; a bezel pressed flat
  onto the front meets the ear beside them), its tabs are taken to be in their slots: from then on it is held there (stiff
  drives) and can only swing about the tabs. The latch: swung shut to within 0.6 degrees (its left end within 4.5 mm of
  the ear), it latches and holds. The tabs' and latch's shapes and the latch's click force are not modelled, nor
  unhooking or the lock.
- Friction 0.6 static, 0.5 dynamic (plastic, painted steel).

## Implementation

| Path | What |
|---|---|
| `server_bezel.py` | the task file: the layout, scoring, the scripted demo and the text an agent reads |
| `bezel/design.py` | dimensions: rack, server, ears, bezel, the hook and latch tolerances (pure Python) |
| `bezel/usd.py` | the rack's articulated USD with the bezel's joint chain (`pxr`) |
| `bezel/sim.py` | the rack as an Arena articulation, the bezel's hook and latch, and the reset |
| `bezel/orient.py` | forward-pointing grips for the scripted demo: a waypoint's `yaw` may carry a full gripper rotation; it wraps `yam_arena.demo.grip_rotation` in the running process (the adapter's files are not changed) |
| `yampatch.py` | adds to the task-file format at runtime (without changing the adapter's files): spawning custom Arena objects (the rack), a per-step hook (`on_step`) and a reset hook (`on_reset`); it wraps `yam_arena.scene.task_object`, `yam_arena.scene.object_states` and `YamArenaEmbodiment.reset` of the pinned adapter |

The USD file is built at start-up into the system's temporary folder (`yam_tasks/server_bezel`).

## Media

Rendered with `inspect-robots video <log>.json` (top camera, 320 x 176):

- `media/setup.png`: the setup, the demo run's third top-camera frame (the first is rendered before the reset puts the
  bezel on the table).
- `media/demo.mp4`: the scripted demo, full score (1.0).
- `media/dream.mp4`: the Dream-1 run at Dream's settings (320 x 176).

## Licences

Everything (rack, server, bezel) is built by the code in this folder; no third-party assets.
