# Video and tracking

Video, detection and tracking run locally in a private child owned by the Mission Planner plugin. Tracking moves
the gimbal; aircraft flight and payload operation remain separate operator actions.

## Video inputs

In **Settings -> Video**, add a named **Network stream** or **Camera / capture card**
input and Save. Editing a definition does not switch playback. Choose the active
input in Dashboard or Video; that selection is saved for the aircraft and updates
HUD and both views. A stopped player stays stopped. Removing the final input leaves
video unconfigured. Profiles use `VideoInputs` and `SelectedVideoInputName`.

For local capture, click **Refresh** and choose the Windows device. Refresh reads
metadata without opening cameras. The saved device path is exact; a missing device
is not replaced by another. Refresh after reconnecting or changing USB ports.
Mission Planner's `ksvideosrc` negotiates raw/MJPEG modes up to 1920x1080 at 30 FPS;
DirectShow-only virtual cameras are unsupported. Startup may briefly probe other
capture drivers. Busy/missing/unsupported devices report an error.

HUD, Dashboard and Video share one stream per local device, with independent
playback/selection state. The camera stays active while any player uses it.
Local capture always uses a one-frame latest-image queue.

![Video controls and active input selector](images/video-page.png)

Network inputs accept RTSP codecs supported by the installed GStreamer decoder,
or RTP/H.264 UDP such as `udp://@:5600`. RTSP offers TCP/UDP and a 0-5000 ms
jitter buffer. Its settings do not affect UDP/local capture. The decoded queue
holds 1-120 frames; **Drop old frames** bounds old queue/jitter data. Display and
appsink still retain the latest frame, so playback is not lossless recording.
Configure bitrate, resolution and encoding at the source. NOMAD accepts the endpoint;
camera-server administration and network setup remain external.
For EDGE T3 receivers that reset the connection during RTSP negotiation, select UDP
and enable **Skip RTSP OPTIONS and keep-alives**. Leave it off for servers that
require RTSP keep-alives. Existing inputs default to off.
URLs may contain credentials: review exports before sharing.

## Vision and gimbal tracking

### Per-input gimbal outputs

In **Settings → Video → Gimbal outputs → Configure**, leave servo outputs off to
use Mission Planner's native mount, or enable one to three of **Pitch**, **Yaw** and
**Roll**. Each enabled axis has a distinct flight-controller output (1–16), minimum
and maximum PWM, minimum and maximum angle in degrees, and reversal. PWM must stay
within 500–2500 µs; increasing angle limits must fit ±180° and contain the calibrated
zero target. Existing inputs keep native mount control.

Configure the flight-controller outputs to accept servo commands in Mission Planner
before use; NOMAD does not reassign servo functions. ArduPilot's
[servo-command requirements](https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-set-servo)
still apply. Targets use the selected MP connection and its native ACK/retry handling.
Acknowledgements do not establish physical position.

The existing joystick screen-axis mapping selects which axes the pad moves.
Automatic region tracking steers the configured pitch/yaw axes and requires level
roll. Tracking travel limits must fit the servo calibration. After changing a mapping,
confirm the calibration again before automatic tracking. Changing inputs retires
gimbal requests and requires a centered physical-stick sample before motion resumes.

A direct payload output may share a gimbal channel when its PWM calibration fits
the gimbal's limits. Manual slider/actions on that channel disarm automatic tracking;
tracking requires explicit Start again. Native timed-repeat payloads cannot share a
gimbal output. Commands already dispatched through MP may still finish after input
retirement; no remaining axes are sent from a retired gimbal request.

Choose **Object** or **Area**, then click or drag a selection. **Detect** can propose
recognized objects but is optional. Once visually verified, select **Start tracking**
to center the target with the gimbal. Detection/selection alone sends no commands.
The same controls work full screen. **Stop tracking** revokes future requests;
an already-dispatched host helper may finish.

| Mode | Target and limits |
|---|---|
| Standard Object (CPU, default) | Separates foreground inside a tight selection and aims at the current whole-object bounds centre. Rotation, small targets, occlusion and lookalikes can require reselection. |
| GPU Object | Explicit per-input EfficientTAM tracker on an NVIDIA DirectX 12 adapter through DirectML. A model/adapter/runtime failure is reported without switching tracker. |
| Area | Follows the original fixed point on an approximately planar surface, retaining its original reference and one verified working view. Use distinctive texture. |

### Object tracking

Select tightly around the whole object with visible edges. Drag padding is not
part of the measured object. The aim can lie between legs or inside a hole;
foreground evidence establishes identity separately from the aiming pixel.
The CPU tracker retains original appearance and one verified working view.
Predicted masks are not learned. Small selections can use a larger processing
image, which costs CPU but cannot recover detail absent from the camera.

Recovery needs three consistent matches. The CPU tracker can search globally
using retained appearance/features, but uniform colour alone is insufficient.
Its short local recovery window can still accept an indistinguishable nearby decoy;
this case is unqualified. A completely unseen side or full occlusion may need reselection.

**Settings -> Video -> Object tracker** selects GPU Object tracking for that input;
restart the video view for the setting to take effect. The tested model/runtime and
licenses are embedded, hash-verified and extracted to the plugin's vision cache.
A model load is shared by later selections in the same MP session. Only admitted
frames update model memory. Rejected frames and stale gaps send no gimbal requests;
recovery requires three admitted frames. After 15 consecutive unadmitted frames,
reselection is required. The bounds reflect visible segmentation; compressed-away
details are absent. Model memory cannot reliably distinguish identical objects.

### Area tracking and memory

Use Area for a fixed point on grass, pavement or a wall. Include distinctive
local texture and surrounding features. The original point remains fixed as
perspective changes; verified working-view updates cannot relocate it.

| State | Behavior |
|---|---|
| Verified | An explicitly active session may steer within calibration limits |
| Searching / Outside | Retains the reference; sends no new steering requests |
| Ambiguous | Rejects competing/repetitive matches; retains the reference |

After visual uncertainty, three unique matches are required to resume an active
intent. **Stop tracking**, manual input or stale imagery requires explicit
**Start tracking** again after verification; visual recovery alone never rearms it.
**Clear**, source/aircraft/profile changes discard the reference. Another view taking
tracking ownership disarms the previous owner. These activation interlocks apply to Object too.
Repetitive tiles, featureless surfaces, moving vegetation, parallax and large view
changes can remain Searching. A retained reference does not establish current position.

### Camera and mount calibration

Configure each camera before enabling tracking:

- Use fixed zoom, rectilinear projection and square pixels; enter horizontal FOV.
- Enter actual pitch/yaw travel, correction directions and requested-angle rate
  (1-60 degrees/s). Yaw supports -180/+180 degrees; new inputs start at -90/+90.
- Enable **Drop old frames** with a one-frame decode queue. Start from a manual
  target within the travel range. Confirm calibration only after hardware checks.

Changing source type/device clears calibration confirmation. Mount control uses
the selected MP connection and configured ArduPilot mount backend. Tracking requests
pitch/yaw with roll zero; fresh aircraft attitude/rates compensate body yaw inside
the same travel/rate bounds. Requested/reported target angles may be estimates,
not measured orientation. Check axes and actual motion on the installed mount.
Decoder receipt age does not measure exposure delay or detect every upstream freeze.

### GPS and altitude search prediction

**Ground prediction** is optional and off by default. Use it only for a horizontal
surface at a known height relative to HOME; zero means HOME elevation, not terrain
beneath the camera. Leave it off for walls, uneven terrain or unknown elevation.
Fresh GPS/altitude, consistent HOME and a mount-target estimate reproject a verified
point into a coarse search region. The amber diamond is a hint and never authorizes
unseen-target steering. A contradictory unique visual match disables the prior.

**Camera delay** (0-2000 ms) declares estimated camera/encoder delay separately from
RTSP buffering. It requires bracketing telemetry history. Stale/missing data, changed
HOME/binding, shallow rays and unsupported ranges disable the hint. Visual tracking
remains independent. No measured target coordinate is produced.

### Aircraft heading marker

The blue line around the crosshair shows aircraft heading relative to the camera:
top aligned, sides left/right, bottom behind. It uses attitude, not GPS course.
Hover over status for the source and estimate classification. Stale/unknown
aircraft or camera data removes it.

![Aircraft heading marker](images/video-heading.png)

### Save observations

**Save observation** writes PNG + JSON to `NOMAD/observations` under the host plugin
data directory. It pairs pixels with detections, input/model/frame provenance,
tracking state/aim and separately labeled aircraft GPS and submitted mount requests.
Detector-assisted selection retains the original model label through model changes.
URLs are omitted. Aircraft position, search hints and submitted targets are not
measured object position or physical mount feedback.

### Replace the detection model

The bundled YOLOX Nano detector uses CPU inference and COCO vehicle classes by
default. Tracking is independent of detection. Open **Manage models** on Video or
**Manage** beside Detection model in Settings:

1. Add trusted ONNX weights or Duplicate a definition, then name it.
2. Keep labels in model-output order; Import labels reads an ordered text file.
   Select classes to detect. For a People preset, duplicate the built-in model and select person.
3. Set input size, score and NMS thresholds. **Test model** validates loading/tensors.
4. Save, then select the model on Video. Cancel leaves the draft unsaved.

Definitions live in `NOMAD/models`, shared across aircraft/cameras. They reference
weights in place; removing a definition does not delete weights. See the
[JSON example](https://github.com/McGill-Aerial-Design/NOMAD/blob/main/mission_planner/examples/vision-yolox.json).
Imported relative weight paths resolve beside the JSON. A damaged/missing model
leaves plain video available without silently substituting another model.

Model changes clear old boxes immediately; only the latest load may publish.
Loading and inference use independent latest-frame lanes. Live video continues while
analysis is pending; overlays appear only on their matching pixels. Overlay alignment
adds at most 100 ms of deliberate preview delay, and slow work falls back to plain
frames. Plain HUD video bypasses alignment. Tracking uses independently delivered
results with the original receipt age; receiving an old result never makes it fresh.
After failure, select a valid model and press Detect to retry.
Supported exports are raw YOLOX grid ONNX, static float NCHW BGR 0-255,
top-left letterboxing/padding 114 and strides 8/16/32. Hash/tensor checks remain
required. Other architectures need an `IVideoDetector` adapter.

## Media failure and recovery

A decoder or native vision crash/hang retires automatic tracking and affects all
NOMAD media views together. Mission Planner retains aircraft connectivity and ordinary
controls. One automatic restart restores plain video with detection/tracking off;
select a fresh target and press Track explicitly to resume. A second worker failure
requires Start/Retry. Saved profiles/model choices are not rewritten during recovery.
Cold GPU loading stays visible as Loading tracker; if the selected image expires
during loading, select a current image after the model is ready.
Model validation errors keep plain video available without choosing another model.

Stop and view closure invalidate steering immediately and clean up asynchronously.
An already-submitted MP mount request may still finish, and the last mount setpoint
may remain active. Worker loss does not send a neutral/hold/termination command.
A stalled MP-native request can keep custom outputs busy until it actually completes.
GPU driver faults and system resource exhaustion can still affect the wider machine.
