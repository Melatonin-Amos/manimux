# Shot contract for the editor

| Shot | Requirement | Composition |
| --- | --- | --- |
| Primary GUI | Required; 30–60 s preferred, ≥12 s useful execution | Full clean RoboGUI with fixed view, visible robot/object camera, task/run state |
| External robot | Optional; same take | Fixed landscape view, both gripper/work area visible, no fast pan |
| Evaluation | Optional; 10–30 s preferred | Human reviews result, enters honest score and saves; actual visible acknowledgement |

Keep 3 seconds of handles where possible. Nominate in/out in seconds from the
submitted file's first frame, not wall-clock time. The main interval should show
meaningful continuous manipulation (approach, grasp, move, place), not only idle
waiting or a rotating 3D model. Never trim around a failure to claim success. A
failure can be useful footage when labelled accurately.

Keep composition consistent across takes on the same station. Do not force every
embodiment into the YAM layout: fit the relevant arms/mobile base into view and
retain the camera panel that best shows the physical task. Preserve readable
operator controls while giving the scene most of the available space. Avoid
pointer motion over the action, tooltip popups, menus, scrolling and viewpoint
orbit during the nominated interval.

1080p30 native is enough for small wall cells and a full-screen lead shot. 1440p30
native is preferable when already available. Do not record 60 fps unless the
station already sustains it without affecting control. 720p30 is valid for smaller
tiles; identify it as such. Do not pad or upscale screenshots into a fake higher
resolution. Capture an appropriate viewport rather than stretching the image.

For an optional external clip, record `offset_from_gui_seconds`: the external
clip's first frame time minus the GUI clip's first frame time. Example: if the
external recording starts 2 seconds later, offset is +2; an event at GUI time 8 s
appears at external time 6 s. Use a shared visible event/timebase and record the
method; use null when unknown. Never invent alignment or move a robot just to
create a synchronization cue.

If there is no live physical camera in RoboGUI, record a synchronized external
view and explain the limitation. This remains a composition exception for editor
review, not a silent pass as camera-visible GUI footage.

Before marking visual review complete, watch the full nominated interval and check
for freezes, repeated stale camera frames, jump cuts, unrelated motion, clipped
text, hidden task objects, private content and recorder overlays. File metadata
alone cannot establish smooth motion or real execution.
