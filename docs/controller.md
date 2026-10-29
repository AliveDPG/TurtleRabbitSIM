# The controller shipped in this repo

Design notes for Controller.py / nav.py / vision.py. Replace them with your own shuttle code if you like; README.md explains how they are run.

| File | What it does |
|---|---|
| `Controller.py` | Sim glue: reads state/radar, schedules camera captures, attitude PD, status log. Boilerplate above the markers is untouched. |
| `nav.py` | `ShuttleNav` (state fusion + time history), `CVTrack`/`DebrisTracker` (world-frame constant-velocity Kalman tracks, radar + camera fusion, radar clock-offset estimate), `Guidance` (tube filter, planned-path closest approach, burn/coast/trim law). Pure numpy. |
| `vision.py` | Binary threshold + connected-component blob finder, pixel → bearing, camera yaw/pitch geometry (wiki convention). |
| `sim_offline.py` | Point-mass rehearsal with the competition numbers (0.5 m/s², 500 debris / 200 radar-visible, 120 Hz round-robin radar noise model). `python sim_offline.py 8` |

## Design

* **World frame.** Every radar return is `shuttle_pos(t_meas) + rel` using `ShuttleNav.pos_at(t_meas)`, so our own burns never look like debris velocity changes. The radar stamps with its *own* start time; `DebrisTracker` estimates the offset as the running minimum of `(t_now − newest stamp)`.
* **Tracks.** Constant-velocity KF per rock with a slow-drift prior (σ_v0 = 0.05 m/s, σ_a = 0.01 m/s²) so 2 m radar noise averages down instead of becoming velocity. Association is nearest-neighbour in the world frame with a size-consistency check; duplicates are rejected by (stamp, position).
* **Camera fusion.** Blob bearing + radar range → anisotropic position update (tight across the line of sight, radar σ along it). If the image object exposes depth (`as_Depth*`/`PixelsDepth` — probed and logged at the first capture), camera-only tracks are created for the ~300 rocks the radar never reports.
* **Tube + closest approach.** Our *planned* trajectory (burn to `v_cruise`, then coast) is sampled against every track; only tracks whose closest approach is within `tube_radius` of the path count. A conflict is `miss < rock radius + 3 m shuttle + 4 m margin + 3σ`.
* **One burn, coast, trims.** Along-track: accelerate at 0.5 m/s² to `v_cruise`, then coast with a ±0.15 m/s deadband. Cross-track: when a conflict exists, pick the perpendicular direction needing the least Δv to clear *all* threats (exact forbidden-interval solve), with 2 m hysteresis; drift back to the corridor only if the correction is worth it (> 1 m/s far, > 0.3 m/s near) and provably safe. Final 20 s: pursuit straight at the target.
* **Attitude.** Body +x on the velocity vector so the cameras look where we are going; PD saturated at 0.05 rad/s².

## Knobs (top of `Controller.py`)

`v_cruise` / `v_arrive` (22 m/s → ~80 s flight, time score ≈ 1.5; fuel is not a constraint: a whole run uses ~0.04 kg of the 1.15 kg nominal), `tube_radius`, `margin_m`, `k_sigma`, `CAMERA_PERIOD_S`, `BLOB_THRESHOLD`, `CAMERA_ONLY_WHEN_COASTING`.

## Things to verify in the real sim (logged on screen)

1. `Attitude convention: R|RT|frames` — which quaternion convention matched the API.
2. `Image API attrs: [...]` — whether a depth accessor exists.
3. `tracks=` should settle near 200 (radar `NumDebris` in the sim config is 200 of 500 spawned).
4. If blobs appear mirrored (camera trims go the wrong way), flip `right_is_minus_y` in `vision.pixel_to_dir_cam`.
