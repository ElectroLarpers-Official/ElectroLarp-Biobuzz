# Vision (Arducam Hive detection)

Loads only when working under `vision/`. **Planned — no vision code exists right now** (`ArduVision` was removed; it will be re-added later). These are the constraints and findings to build it from. When re-adding, target SDK 12.0: `AprilTagDetection` is abstract, `id`/`decisionMargin` are on `AprilTagSingleDetection`, and Hive clusters may come back as `AprilTagClusterDetection`. Add the camera name to `HardwareNames` and the table in the root `CLAUDE.md` (§4 Hardware names).

Second camera (Arducam), separate from the floor-facing Limelight. Purpose: AprilTag-detect whether the alliance Hive is up or tipping, using the Hive Cell AprilTag clusters (4 distinct 36h11 tags per cluster, on the Cell undersides).

**Given constraints**
- Camera is **physically rotated to the right** to maximize vertical FOV — mount decision is made, don't re-derive.
- Detection must be **heavily lightweight**; runs on the **Control Hub** (no coprocessor), so inefficient settings directly cost FPS/looptime.
- Must return a value usable from auto classes; class belongs in `util/`; use current Ivy syntax, cross-checked against official FTC vision docs.

### Camera legality / hardware
- FTC only requires a **UVC** webcam — not restricted to Logitech; that's just what FIRST pre-calibrated. Arducam USB UVC modules (OV9782, IMX219, IMX291 boards) are UVC-compliant and work with VisionPortal like any webcam.
- Trade-off: no entry in the SDK's `builtinwebcamcalibrations.xml`, so accurate **pose** would need manual lens-intrinsics calibration (AprilTag Camera Calibration utility OpMode). Irrelevant here — see below.
- Two webcams on one Control Hub: both into the CH USB 3.0 port via a hub (see "AprilTag Switchable Cameras" / Vision MultiPortal sample). Under MJPEG, resolutions below ~432×240 can degrade one stream — bench-test the actual combo. (Limelight is a network coprocessor, so it doesn't compete for CH USB bandwidth.)

### Key finding: skip pose, keep ID-only
- The SDK only computes full pose (ftcPose/robotPose, 30+ fields) for a tag **if that tag has size metadata in the AprilTagLibrary**. Without sized entries, detections return **ID + decisionMargin only** — no pose math runs.
- Since the util only needs "which cluster is visible," this is the single biggest lightweight win: **never** call `setLensIntrinsics()` or pass a sized tag library. Also means **no Arducam lens calibration is needed at all** for this util.

### Rotation
- SDK 8.2+ defaults to `SENSOR_NATIVE` — no frame rotation. FTC docs say forcing stream rotation has "rather high" overhead.
- Since the camera is already physically rotated, **do not** request software rotation — leave `SENSOR_NATIVE`, pay zero cost. Only consequence: ftcPose X/Y axes would be swapped — irrelevant, pose is never read.

### CPU / looptime levers (ordered by real cost)
1. **Resolution** — biggest lever. 320×240 or 424×240 is plenty for ID at on-robot range; 640×480 only for pose/long range (not needed).
2. **Decimation** — `aprilTagProcessor.setDecimation(n)`, changeable mid-match. SDK sample (C920 @ 640×480): decimation 2 ≈ 10 FPS / ~10 ft on a 2" tag; decimation 3 (default) ≈ 30 FPS / ~4 ft. Higher decimation likely fine at close fixed range — bench-test at actual distance.
3. **Stream format: MJPEG**, not YUY2 (YUY2 caps out or drops one of two webcams above ~640×360).
4. **`enableLiveView(false)`** — free win, no competition downside.
5. **Disable draw overlays** — `setDrawAxes(false)`, `setDrawCubeProjection(false)`, `setDrawTagOutline(false)`. Meaningless without pose anyway.
6. **`visionPortal.setProcessorEnabled(aprilTag, false/true)`** — cheap, non-blocking toggle; pause detection during phases where Hive state isn't checked (e.g. mid-path-follow), re-enable before the check. Safe from an Ivy `instant()`.
7. **`stopStreaming()`/`resumeStreaming()`** — bigger win but **blocking**. Only at clear phase boundaries, never in a tight loop or `periodic()`.
- Note: `getDetections()` is a cheap accessor over already-computed results — throttling *calls* saves nothing. The cost is the background detection pass per frame, which is why 1–4 matter.

### ⚠️ Open TODOs
- **Exact tag IDs for BIOBUZZ Hive Cells not yet pulled** from the manual's tag-assignment table — current arrays are placeholders. Confirm before relying on this.
- Confirm whether "underside" placement means the visible cluster belongs to the **currently-up** or **currently-down** cell — this flips which state the util should report as tipped/scored. Field geometry not fully nailed down.

