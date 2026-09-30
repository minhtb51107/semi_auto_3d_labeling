# ARCHITECTURE PROPOSAL — Semi-Auto 3D Cuboid Labeling

> **Phase 0 — Design Only (no implementation yet)**
> Created: 2026-10-01

---

## 1. High-Level Data Flow

```
┌──────────────┐     ┌───────────────────┐     ┌──────────────────┐
│  Point-Cloud  │────▶│  Propagation      │────▶│  CVAT Import     │
│  Frames (.pcd)│     │  Engine           │     │  Generator       │
└──────────────┘     │                   │     └────────┬─────────┘
                     │  ┌─────────────┐  │              │
┌──────────────┐     │  │ Cuboid3D    │  │              ▼
│  Seed Cuboid  │────▶│  │ Propagator  │  │     ┌──────────────────┐
│  (CVAT export)│     │  └─────────────┘  │     │  CVAT            │
└──────────────┘     │  ┌─────────────┐  │     │  (human review   │
                     │  │ Motion Prior│  │     │   & correction)  │
┌──────────────┐     │  └─────────────┘  │     └────────┬─────────┘
│  Ego-Pose    │────▶│                   │              │
│  (optional)   │     └───────────────────┘     ┌───────▼──────────┐
└──────────────┘                                │  Corrected       │
                                                │  Annotations     │
                                                │  (next seed)     │
                                                └──────────────────┘
```

### Loop

1. **Human** draws/confirms a cuboid in CVAT for frame *k*.
2. **Tool** reads seed cuboid + point-cloud frames *k* … *k+N*.
3. **Propagation engine** predicts cuboids for frames *k+1* … *k+N*.
4. **Tool** writes predicted annotations → CVAT import file.
5. **Human** reviews in CVAT, corrects any drift.
6. Corrected cuboid becomes the new seed → repeat.

---

## 2. Module Breakdown

```
semi_auto_3d_labeling/
├── docs/
│   ├── PROJECT_SCOPE.md
│   └── ARCHITECTURE.md          ← you are here
├── src/
│   └── semi_auto_3d/
│       ├── __init__.py
│       ├── __main__.py           # CLI entry point
│       ├── models/
│       │   ├── __init__.py
│       │   └── cuboid.py         # Cuboid3D dataclass
│       ├── io/
│       │   ├── __init__.py
│       │   ├── pointcloud.py     # Read .pcd / .bin / .ply
│       │   ├── cvat_export.py    # Parse CVAT annotation export
│       │   └── cvat_import.py    # Generate CVAT annotation import
│       ├── propagation/
│       │   ├── __init__.py
│       │   ├── icp.py            # ICP-based cuboid alignment
│       │   ├── motion_prior.py   # Constant-velocity fallback
│       │   └── propagator.py     # Orchestrator: crop → align → update
│       ├── transform/
│       │   ├── __init__.py
│       │   └── ego_motion.py     # Apply ego-pose transforms
│       └── eval/
│           ├── __init__.py
│           └── metrics.py        # IoU, centre error, yaw error
├── tests/
│   ├── test_cuboid.py
│   ├── test_pointcloud_io.py
│   ├── test_cvat_roundtrip.py
│   └── test_propagation.py
├── pyproject.toml
└── README.md
```

---

## 3. Key Components

### 3.1 `Cuboid3D` (data model)

```python
@dataclass
class Cuboid3D:
    center: tuple[float, float, float]   # (x, y, z) in metres
    size: tuple[float, float, float]     # (length, width, height)
    yaw: float                           # radians, CCW around Z
    label: str = "car"
    track_id: int = 0
    frame: int = 0
    confidence: float = 1.0              # 1.0 = human-confirmed
```

Convenience methods:
- `corners() → ndarray (8, 3)` — 3D bounding-box corners
- `crop_points(points) → ndarray` — return points inside the cuboid
- `iou_3d(other) → float` — 3D Intersection-over-Union
- `to_cvat_dict() / from_cvat_dict()` — serialization

### 3.2 Propagation Strategy

```
Frame k (seed)                Frame k+1 (prediction)
┌─────────────┐               ┌─────────────┐
│  Cuboid_k   │──── step 1 ──▶│  Cuboid_k+1  │
│  + PC_k     │               │  (predicted) │
└─────────────┘               └─────────────┘

Step 1: Crop — extract points within enlarged cuboid from PC_k and PC_{k+1}
Step 2: Align — run ICP on the two point crops to get ΔT (rotation + translation)
Step 3: Apply — transform Cuboid_k by ΔT → Cuboid_{k+1}
Step 4: Validate — if ICP fitness < threshold → fall back to motion prior
Step 5: Confidence — set confidence based on #points and ICP fitness
```

#### Why ICP (Iterative Closest Point)?

| Criterion | ICP | Kalman filter | Learning-based tracker |
|---|---|---|---|
| No training data needed | ✅ | ✅ | ❌ |
| Uses actual geometry | ✅ | ❌ | ✅ |
| Simple to implement | ✅ | ✅ | ❌ |
| Handles rotation well | ✅ | Needs augmentation | ✅ |
| Works with sparse points | ⚠️ fallback needed | ✅ | ❌ |

ICP is chosen as the primary method because it directly uses point geometry,
needs no training, and handles rotation naturally. A constant-velocity motion
prior provides a fallback for sparse/occluded frames.

### 3.3 Motion Prior (fallback)

```python
# Constant-velocity model
velocity = (cuboid_k.center - cuboid_{k-1}.center)  # if k > 0
cuboid_{k+1}.center = cuboid_k.center + velocity
cuboid_{k+1}.yaw = cuboid_k.yaw + delta_yaw
cuboid_{k+1}.size = cuboid_k.size                    # size held constant
cuboid_{k+1}.confidence = 0.5                        # lower confidence
```

### 3.4 CVAT Integration

```
CVAT Export (XML/JSON)          Our Tool                  CVAT Import (XML/JSON)
┌────────────────────┐    ┌──────────────────┐    ┌────────────────────┐
│ <track id="0">     │───▶│ parse seed       │    │ <track id="0">     │
│   <shape frame="0">│    │ cuboids          │    │   <shape frame="0">│
│     x, y, z, ...   │    │                  │    │     (original)     │
│   </shape>         │    │ propagate        │    │   <shape frame="1">│
│ </track>           │    │ frames 1..N      │───▶│     (predicted)    │
└────────────────────┘    └──────────────────┘    │   <shape frame="2">│
                                                  │     (predicted)    │
                                                  │ </track>           │
                                                  └────────────────────┘
```

- **Export format:** CVAT for video, 3D cuboid track, Datumaro or CVAT XML 1.1.
- **Import format:** Same schema; predicted shapes appended to existing tracks.
- Track ID is preserved so CVAT treats them as the same object.

---

## 4. Coordinate System Convention (Internal)

| Axis | Direction | Notes |
|---|---|---|
| **X** | Forward (vehicle heading) | |
| **Y** | Left | |
| **Z** | Up | |
| **Yaw** | CCW positive around Z | 0 = +X direction |

All conversions to/from CVAT or dataset-specific systems happen at the I/O
boundary (`cvat_export.py`, `cvat_import.py`).

---

## 5. Dependencies (Planned)

| Package | Purpose |
|---|---|
| `numpy` | Array math |
| `open3d` | Point-cloud I/O, ICP |
| `scipy` | Spatial transforms (optional) |
| `lxml` or `xml.etree` | CVAT XML parsing |
| `pydantic` ≥ 2 | Data validation (optional, can use dataclasses) |
| `click` or `typer` | CLI |
| `pytest` | Testing |

---

## 6. Non-Functional Requirements

- **Python ≥ 3.10**
- **No GPU required** for Phase 1 (ICP is CPU-only in Open3D)
- **Batch processing** — not real-time
- **Tested on Windows** (user environment) but should work cross-platform

---
