# H98 PCB localization prototype

The supplied photograph is a 1280 x 1024 monochrome image. The H98 drawing is
available as both DWG and ASCII DXF. The simplified DXF's `KEEPOUTLAYER`
contains the board outline and four 1.20 mm corner circles, encoded as paired
arcs. Its outline measures approximately
33.79 x 20.94 mm. Using this mechanical layer avoids matching thousands of
copper and silkscreen primitives that do not define the board edge.

## Interactive anchor selection

Run `python3 tools/select_pcb_holes.py` in a Python environment with PySide6.
The CAD and camera image open as previews. Loading files and editing anchors
do not run registration.

1. Choose A1, A2, A3, and A4 in turn and click the corresponding CAD fixture
   hole for each slot. All four assignments are required to enable registration.
2. For an additional probe pad, click its CAD circle (for example C6), then
   click **Add probe-pad anchor**, then click the matching center in the camera
   image. You can also press **Add probe-pad anchor** before selecting the CAD
   circle. The highlighted circle and instruction below the button indicate the
   current step. Wheel zoom and right/middle-button panning work in both views.
3. Click **Register CAD to image** to detect the holes, refine the probe-pad
   clicks, fit the pose, and draw the overlay. Adding or removing a pair clears
   the old registration; press the button again to apply the edited selection.

The displayed H98 recipe uses the known four-hole layout and visible CAD contour
for coarse matching, or a taught SIFT camera reference, then fits the assigned
holes with strong detections and any paired probe pads. Weak fixture detections
are excluded from the fit. In this snapshot,
the lower-right fixture hole is weak, so three fixture detections are used.
The probe list distinguishes detected centers from manual clicks. The runtime
includes image-feature detection, probe refinement, and pose fitting; it excludes
file loading and overlay drawing. The recipe panel reports the used anchor count,
scale, and angle after registration.

**Save anchor recipe** saves A1–A4 assignments and the selected probe pads by
CAD handle, layer, center, and radius. **Load anchor recipe** restores those
selections after checking them against the open CAD. The loader also accepts
older `pcbvision.fixture_anchors.v1` files, which contain fixture holes only.
Recipes do not store image clicks as reusable measurements.

Opening another camera image preserves the CAD selections and resets their
image coordinates to **awaiting detection**. When Register is clicked, the
fixture-hole pose predicts a search location for each selected pad. Only a fresh
local image detection is added to the pose fit; a pad marked **not found;
excluded** contributes no measurement. Click that CAD circle and pick its image
center again to supply or correct a manual correspondence. Registration remains
explicit after loading a recipe or another image.

Each used pad supplies another CAD/image center pair to the robust similarity
fit for translation, rotation, and scale. Clear pads distributed across the board
can reduce random localization error and improve rotation estimation. Glare,
wrong correspondences, lens distortion, and systematic CAD-to-pad offsets can
still bias the result. A manual click is used when its local refinement fails,
and is identified in the list. Local Hough detection does not establish subpixel
accuracy. Per-pad residuals and the overall fit residual describe consistency
with the fitted model; they do not establish placement accuracy or the 0.1 mm
tolerance without independent measurements.

## Hikvision live view and snapshots

Click **Hikvision camera…** in the selector to open the dedicated camera window.
Its backend is adapted from `cadrefs/cadviewer/camera/hikvision.py` and uses the
installed Hikrobot/Hikvision MVS SDK. It has no runtime dependency on `cadrefs`.
On this Linux machine MVS is expected at `/opt/MVS`; for a different installation,
set `HIKVISION_MVS_ROOT` before launching. The Python SDK wrappers must be under
`Samples/64/Python/MvImport`, with the matching native MVS libraries installed.
The SDK is loaded only when opening the camera window, so image-file workflows
remain available without it.

1. Click **Find cameras** if necessary, choose a device, and click **Open camera**.
2. Click **Start live view** to focus and adjust lighting; **Stop live view**
   pauses the preview. Opening a camera reads its existing settings rather than
   applying arbitrary defaults.
3. Adjust exposure in microseconds, gain, gamma, auto exposure, or mirrors and
   click **Apply parameters**. Unsupported controls are disabled. Parameter
   ranges and applied values are read back from the camera; errors appear at
   the bottom of the window. Manual exposure is disabled while auto exposure is
   selected. Settings are applied while acquisition is stopped, then live view
   resumes if it was running.
4. Click **Snapshot → registration**. This stops live acquisition temporarily,
   performs a software trigger, receives a new full-resolution BGR image, and
   loads it directly into the main camera-image area, with no intermediate file.
   Live view resumes afterward if it was running. A failed capture reports an
   error and leaves the previous registration image intact.
5. Return to the main window and click **Register CAD to image** when ready.
   Snapshots preserve A1–A4 and probe-pad selections, clear previous pixel
   measurements/overlays, and wait for explicit registration. Vision timing
   continues to exclude camera acquisition and display rendering.

The preview is limited to 18 updates per second and a maximum side of 1280 pixels;
snapshots always use native camera resolution. SDK access runs in a dedicated
worker thread, including capture waits and settings changes. Closing either the
camera window or the selector stops the worker and releases the device. Opening
the camera window again creates a new connection session.

## Coarse pose, partial contours, and a SIFT reference

The original detector was tuned to the first snapshot: it seeded only from the
two top holes, always fitted holes 1–3, treated only hole 4 as potentially weak,
and rejected any outline extending beyond the image. Its score used circular
features without checking the CAD contour. A near-rectangular hole pattern and
many circular pads can produce incorrect identities, particularly under rotation
or when a different hole becomes hidden. The local pad search covers only 7 px
around its seed, so an incorrect coarse pose also causes probe-pad detection
failures. Fixed Hough radii (8–20 px) and the 12–24 px/mm scale interval remain
prototype assumptions.

Coarse registration now evaluates all six CAD-hole pairs against distinct image
circles, requiring at least three supporting holes. An oriented edge-distance
score ranks the candidate CAD contours using the best 65% of their visible
samples. This tolerates missing boundary sections and contours partly outside
the image without demanding a closed segmented silhouette. Obscured sections
are not invented as image measurements. The contour ranks poses; it is not used
as a precise edge fit. At least 35% of the outline must lie within the image.
Any hole can be missing or weak; only strong observed centers enter the final
fixture fit. Circle-only and contour-only scenes can still be ambiguous if their
visible geometry is symmetric or the background supplies matching edges.

To add image texture as orientation evidence:

1. Register a clear image and visually verify its overlay and hole identities.
2. Click **Teach SIFT from verified overlay**. Features are extracted inside
   the projected PCB boundary, with a small erosion to exclude background.
3. Optionally **Save SIFT reference…** to an `.npz` file. Load it with the same
   CAD using **Load SIFT reference…**. This file contains image feature
   descriptors and the CAD-to-reference pose; the anchor recipe JSON separately
   stores the selected holes and probe pads.
4. Open another image or capture a snapshot. The reference persists. Click
   **Register CAD to image**. SIFT matches reference photo features to the new
   photo, estimates translation/rotation/scale using RANSAC, and requires
   distributed inliers and supporting fixture-circle detections. Target SIFT
   extraction uses a maximum side of 960 px; coordinates are converted back to
   native pixels. Anchor detection remains at native resolution.
5. Read the coarse method and contour support in the result panel. If SIFT is
   rejected, the detector falls back to holes plus contour and shows the reason.
   **Clear SIFT reference** selects that fallback explicitly. Opening another CAD
   also clears the reference. These actions do not trigger registration.

SIFT compares photographs of the same PCB, rather than matching a sparse CAD
line drawing directly to a photograph. The taught pose must be correct; teaching
an incorrect overlay preserves that offset. The final fixture/probe fit still
uses measured centers, so SIFT improves initialization and identity selection
without itself establishing fine positioning accuracy. Three strong fixture
circle detections are still required. Distortion, tilt/perspective, changed
glare, mirrored camera settings, and out-of-plane movement remain limitations of
the current similarity model. No lens-undistortion calibration is applied by
this selector yet.

Reproduce the saved-image and transformed-image checks:

```bash
python3 tools/bench_pcb_registration.py --output-dir /tmp/pcb_registration_analysis
```

On 2026-11-10, three saved 1280 × 1024 snapshots returned the visually expected
overlays. Three-run median times were 191–193 ms for holes plus contour and
237–242 ms with a taught SIFT reference on this machine. The two new snapshots
provided 193 and 226 SIFT RANSAC inliers. Timing includes coarse and final
fixture-hole fitting, and excludes file I/O, teaching, drawing, and optional
probe-pad refinement. Controlled transformations through 45°, 90°, 135°, 180°,
and −90° plus a translation kept CAD hole identities consistent, with maximum
hole-position variation up to 2.83 px relative to the transformed baseline pose.
At approximately 17.8 px/mm this is about 0.16 mm; these are resampling consistency
checks, not independent accuracy measurements or evidence of 0.1 mm capability.
Separate tests exercise arbitrary missing holes and clipped/partly occluded
contours. Actual images spanning the field of view are still needed to assess
production robustness and lens-related error.

`tools/align_pcb_cad.py` produces an exploratory overlay and JSON pose report:

```bash
python3 tools/align_pcb_cad.py \
  --image /home/hotcat/Downloads/uploadPCB/Image_20261109165458976.bmp \
  --dxf /home/hotcat/Downloads/uploadPCB/H98_simple.dxf \
  --overlay /tmp/pcb_cad_overlay.png \
  --report /tmp/pcb_pose_estimate.json
```

Pass `--reference /path/to/pcb_sift.npz` to use a saved SIFT reference in the CLI.

The script searches for circular features and matches their arrangement to
the four CAD centers. Supply six pixel coordinates to `--anchors` to override
automatic detection with manually measured centers for holes 1, 2, and 3.
For this snapshot, three holes are robustly matched from the image. A weaker
local circle match is available for the lower-right hole, but it is excluded
from the pose fit because reflection obscures much of its edge. The JSON report
records observed and CAD-fitted centers separately. The green line is the CAD
outline; orange circles are nominal CAD hole sizes; red crosses are fitted
centers.

The image shows strong glare, uneven contrast, wrinkles in the thin film, and
partial occlusion at the lower-right hole. Thresholding alone selects the
background material and internal reflections, so it is not a dependable board
segmentation method. Registration against the CAD mechanical outline and
corner-hole pattern gives a useful pose estimate. The lower-right feature's
local match is weak and must be validated over real production images before
the system relies on it for placement.

This snapshot supports a proof of concept, not a production accuracy claim.
The image does not carry verified camera calibration or a stage coordinate
transform. For production, collect
images over the full expected position/rotation range and across lighting and
film variation; calibrate the camera and fixture/stage coordinates; establish
hole centers from clean images or a gauge; report residuals and reject weak or
inconsistent detections. Validate error and repeatability against the fixture
and required tolerance before allowing a placement move.

Lighting improvements that do not require a backlight include a diffuse dome
or coaxial front light, cross-polarized light and lens filter, and a short
exposure with controlled gain. A second exposure may help if the line can pause.
These changes can improve edge visibility, but the vision code should still
measure confidence and refuse uncertain results.
