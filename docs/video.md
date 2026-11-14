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
After 500 ms without a fresh received frame, the video view labels the retained image
as stale and the HUD clears its owned image. Fresh frames restore playback without
a restart. This receipt check cannot detect a camera that keeps sending frozen pixels.

![Video controls and active input selector](images/video-page.png)

The preview fills the page width, with payload controls below it. Choose an input,
press Play, then click or drag on the video to select a target. **Start tracking**
moves the gimbal after calibration and fresh aircraft telemetry are available.
Failed prerequisites appear below the video and remain visible as frames arrive.
**More** reveals detection model selection, **Manage models**, **Save observation**
and **History**. The playback, tracking and status controls remain available in full screen.

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
still apply. Targets use the selected MP connection. Manual commands and the first
tracking pose use native ACK/retry handling. Once those axes are acknowledged,
active tracking streams fresh absolute PWM positions through MP without waiting
for each update's ACK. A new tracking owner, connection, camera or calibration
requires acknowledgement again. Streamed updates do not confirm acceptance;
acknowledgements do not establish physical position.

These calibrations support normal position servos: PWM selects an angle.
Continuous-rotation servos need a neutral/speed calibration and are not supported
by this position mapping. Servo tracking uses image-error PID feedback, a filtered
derivative, limited acceleration and anti-windup to move the requested position.
The servo's own controller still drives its motor. The camera delay estimate and
receiver buffering reduce feedback gain for delayed imagery. Set the camera delay
to its measured value; wrong delay or angle calibration can still cause poor tracking.
The tracking rate setting limits requested angle change, not measured shaft speed.
Stop, loss of verified imagery and manual input clear the controller state;
busy requests do not advance it. No old position sequence is queued for later replay.

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
For a separated, non-thin object the embedded ViTTrack model proposes each box near
the motion prediction; optical flow seeded with that displacement aligns the retained
outline, which must pass the continuity and masked identity checks before steering.
A confident, uncontested proposal skips segmentation; every fourth frame, or after a
rejected proposal, segmentation refreshes the outline and the views it learns from.
Otherwise sparse forward/backward optical flow proposes local motion. A bounded foreground
pass runs first; a failed appearance/continuity check retries full segmentation.
Both paths must pass the same identity checks before steering. Continuity is measured
from the motion prediction, so an object faster than its own size per frame stays local.
Unchanged appearances do not rebuild the working reference.
Small targets with a tight outline, weak foreground separation or an escaping foreground
component can use the embedded ViTTrack model on the CPU. Its original template stays
fixed; rejected frames cannot train it or move accepted search bounds. Three matched
updates are required before it takes over. If it rejects the initial frames,
a usable foreground tracker is retained. Two plausible competing model peaks retire
an established selection when recent appearance independently confirms the original
object is present; a later quorum cannot resolve that identity ambiguity. Competing
peaks without that appearance witness pause tracking.
Recovery also requires agreement with the last verified appearance and three
consistent matches. Five seconds without a verified match retires this template
selection. Its search follows accepted bounds plus the last measured velocity, coasting
for up to ten missed frames; provisional matches never move the accepted bounds.
Predicted masks are not learned. Small selections can use a larger processing
image, which costs CPU but cannot recover detail absent from the camera.

Recovery needs three consistent matches. The CPU tracker can search globally
using retained appearance/features, but uniform colour alone is insufficient.
Its short local recovery window can still accept an indistinguishable nearby decoy;
this case is unqualified. Overlapping lookalikes can also produce merged model bounds.
A completely unseen side or full occlusion may need reselection.

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
Checked local flow uses a smaller feature budget, with competing-placement checks
on every frame. Failed local flow and recovery retain the full feature budget.
Checked flow from the original operator reference also supports the initial
three-match confirmation when stronger corners elsewhere dominate the global budget.
Receipt gaps and loss retire that local continuity; three distinctive feature matches
are required before flow can authorize tracking again.
Working views reuse descriptors from the current frame near measured plane inliers.
If that sparse set cannot support a new view, the tracker detects reference features
near the measured inliers before learning it.

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
Fresh tracking samples can drive roughly 30 Hz gimbal requests when processing and
native command completion keep up. Busy requests drop that sample; no command backlog
is replayed. Recovery and difficult segmentation can take longer than one video frame.
Delayed analysis pauses steering while fresh video, aircraft telemetry and control
ownership preserve the tracking intent. A fresh verified result can resume it.
Video/UI input gaps and telemetry or ownership loss still require an explicit restart.

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

**More → Save observation** writes PNG + JSON to `NOMAD/observations` under the host plugin
data directory. It pairs pixels with detections, input/model/frame provenance,
tracking state/aim and separately labeled aircraft GPS and submitted mount requests.
Detector-assisted selection retains the original model label through model changes.
URLs are omitted. Aircraft position, search hints and submitted targets are not
measured object position or physical mount feedback.

### Replace the detection model

The bundled YOLOX Nano detector uses CPU inference and COCO vehicle classes by
default. Tracking is independent of detection. Open **More → Manage models** on Video or
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
