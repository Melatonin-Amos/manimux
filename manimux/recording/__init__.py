"""Recording configuration stays usable without the storage runtime installed."""

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from manimux.recording.episode import EpisodeRecorder

__all__ = ["EpisodeRecorder"]


def __getattr__(name: str):
    if name == "EpisodeRecorder":
        from manimux.recording.episode import EpisodeRecorder

        return EpisodeRecorder
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def recording_parameters(**options) -> dict:
    """Preserve recording and video defaults without opening files or starting encoders."""
    from manimux.config_validation import merge_parameters

    values = merge_parameters({
        "enabled": True,
        "video_fps": 0.0,
        "video_codec": "mp4v",
        "video_queue_size": 8,
    }, options, 'recording', extra=())
    if values["enabled"] is not True:
        raise ValueError("recording cannot be disabled")
    fps = values["video_fps"]
    if (
        isinstance(fps, bool)
        or not isinstance(fps, (int, float))
        or not math.isfinite(fps)
        or fps < 0
    ):
        raise ValueError("recording.video_fps must be finite and non-negative; 0 disables video")
    if fps > 0:
        period_ns = 1_000_000_000 / fps
        if not math.isfinite(period_ns) or round(period_ns) <= 0:
            raise ValueError("recording.video_fps must yield a finite, positive nanosecond period")
    codec = values["video_codec"]
    if not isinstance(codec, str) or len(codec) != 4 or not codec.isascii():
        raise ValueError("recording.video_codec must contain exactly four ASCII characters")
    size = values["video_queue_size"]
    if type(size) is not int or size <= 0:
        raise ValueError("recording.video_queue_size must be a positive integer")
    return values
