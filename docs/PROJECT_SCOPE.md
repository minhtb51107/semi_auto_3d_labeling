# PROJECT SCOPE — Semi-Auto 3D Cuboid Labeling

> **Phase 0 — Scope & Architecture Proposal**
> Created: 2026-10-01

---

## 1. Problem Statement

Manual 3D cuboid annotation of LiDAR point-cloud sequences is extremely
time-consuming. An annotator must redraw or copy-paste a cuboid **every frame**
for every tracked object, manually adjusting position, size, and yaw.
This project builds a **semi-automatic propagation tool** that carries a
human-confirmed cuboid forward through consecutive frames, so the annotator
only needs to **correct drift** rather than redraw from scratch.

## 2. Target User

- **LiDAR annotation teams** using CVAT for 3D point-cloud labeling.
- Annotators who already know how to draw 3D cuboids in CVAT but spend most of
  their time re-creating them frame-by-frame.

## 3. Pain Point

| Manual workflow | With this tool |
|---|---|
| Draw cuboid in frame 0 | Draw cuboid in frame 0 |
| Redraw cuboid in frame 1 | Review proposed cuboid, nudge if needed |
| Redraw cuboid in frame 2 | Review proposed cuboid, nudge if needed |
| … | … |
| **N full draws** | **1 draw + (N−1) lightweight corrections** |

## 4. Input / Output

| Item | Description |
|---|---|
| **Input — point clouds** | Consecutive LiDAR frames (`.pcd`, `.bin`, or `.ply`). One file per frame. |
| **Input — seed cuboid** | A 3D cuboid annotation for **at least one frame**, exported from CVAT (XML or JSON). |
| **Output** | Predicted 3D cuboid annotations for subsequent frames, importable back into CVAT for review. |

### Cuboid Representation (Cuboid3D)

```
center:  (x, y, z)       — metres, in LiDAR sensor frame
size:    (length, width, height)  — metres
yaw:     θ               — radians, rotation around the Z-axis (up)
label:   str              — e.g. "car"
track_id: int             — unique object identity across frames
frame:   int              — frame index (0-based)
```

## 5. Initial Object Class

- **vehicle / car** only.
- Other classes (pedestrian, cyclist, truck, …) are deferred to later phases.

## 6. In Scope

| # | Item |
|---|---|
| 1 | Cuboid3D data model (Python dataclass / Pydantic) |
| 2 | Point-cloud I/O (read `.pcd` / `.bin`) |
| 3 | Frame-to-frame cuboid propagation via **ICP-based alignment** of cropped point-cloud regions |
| 4 | Simple constant-velocity motion prior as fallback |
| 5 | CVAT export parser (read seed annotations from CVAT XML / JSON) |
| 6 | CVAT import generator (write predicted annotations back to CVAT format) |
| 7 | Evaluation script: compare predicted cuboids vs. ground-truth corrections |
| 8 | CLI entry point (`python -m semi_auto_3d_labeling propagate …`) |

## 7. Explicitly Out of Scope

| # | Item | Reason |
|---|---|---|
| 1 | Training or fine-tuning a 3D detection / tracking model | Phase 0 constraint |
| 2 | 3D instance segmentation | Phase 0 constraint |
| 3 | Building a new annotation UI | CVAT is the review interface |
| 4 | Real-time / online processing | Batch-mode is sufficient |
| 5 | Multi-class support beyond "car" | Deferred |
| 6 | Camera / image fusion | LiDAR-only for now |
| 7 | Full SLAM / odometry pipeline | We assume ego-motion is either negligible or provided |

## 8. Success Criteria

| ID | Criterion | Metric |
|---|---|---|
| SC-1 | **Fewer full redraws** | ≥ 70 % of propagated cuboids accepted with ≤ minor correction |
| SC-2 | **Stable track identity** | Track ID stays consistent; no ID switches within a continuous sequence |
| SC-3 | **Bounded position drift** | Centre offset < 0.3 m per frame on average |
| SC-4 | **Bounded size drift** | Dimension change < 5 % per frame on average |
| SC-5 | **Bounded yaw drift** | Heading error < 3° per frame on average |
| SC-6 | **Correct CVAT round-trip** | Export → propagate → import produces valid CVAT annotations with no data loss |

## 9. Technical Risks

### 9.1 Coordinate Systems
- LiDAR sensor frame vs. world / ego frame.
- CVAT may use a different axis convention (e.g. right-hand Y-up vs. Z-up).
- **Mitigation:** document and unit-test every transform; keep one canonical frame internally.

### 9.2 Yaw Convention
- Some datasets use yaw = 0 → +X; others → +Y. Counter-clockwise vs. clockwise.
- **Mitigation:** define a single convention in `Cuboid3D` and convert at I/O boundaries.

### 9.3 Frame Numbering
- CVAT frame indices may not match file-system ordering of `.pcd` files.
- **Mitigation:** explicit frame-index ↔ filename mapping file.

### 9.4 Point-Cloud Sparsity
- Far-away or partially visible cars may have very few points (< 20), making ICP unreliable.
- **Mitigation:** fall back to constant-velocity prior when point count is below threshold.

### 9.5 Occlusion
- Temporary full occlusion (e.g. behind a building) loses all points.
- **Mitigation:** hold last-known cuboid + velocity model for a configurable number of frames; mark confidence = low.

### 9.6 CVAT Import / Export Format
- CVAT 3D annotation format (Datumaro XML, CVAT 3D JSON) may change between versions.
- **Mitigation:** pin to a known CVAT version; write format-version-specific adapters.

### 9.7 Ego-Motion
- If the sensor is on a moving vehicle, raw point clouds are in different ego poses per frame.
- **Mitigation:** accept optional ego-pose file (4×4 transforms); if absent, assume stationary sensor.

---
