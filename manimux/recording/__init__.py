from manimux.recording.episode import EpisodeRecorder

__all__ = ["EpisodeRecorder"]


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
    return values
