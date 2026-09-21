# AGENTS.md — `aruco-reader`

Shared rules for the whole system live in **`~/grow/AGENTS.md`** (the
`deployment` repo): workflow (issue → branch → PR), sourcing, build,
conventions, traps. Read that first. This file is only what is specific to
this repo.

## Git identity

Commit here as **`aruco-reader-agent`** so history shows which project's agent did
what. Once per clone (the setting is repo-local, not versioned):

```bash
git config user.name  "aruco-reader-agent"
git config user.email "aruco-reader-agent@grow.local"
```

Check with `git config user.name` before committing. Humans committing by
hand override with `git -c user.name=… -c user.email=… commit`.

## Specific

- Env: `ARUCO_DICTIONARY` (`DICT_4X4_250`), `ARUCO_ONLY_IDS` (whitelist).
- One instance per camera via remapping:
  `ros2 run aruco_reader start --ros-args -r image:=/upward_camera/image`.
- Contract: raw observations only (IDs, corners, centers, stamps); `-1`
  when nothing is visible. No poses, no TF.
- Facility-owned package; the truck-camera instance runs on the truck Pi
  (DESIGN §8.1). Replaces `truck_ws/scripts/aruco_detector.py`.
- `DetectorParameters_create()` is gone in OpenCV ≥ 4.7; fine on pinned 4.6.
