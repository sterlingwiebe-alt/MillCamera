"""Camera RTSP URL from the environment, so the camera login is not in the
code or the repository.

Set CAM_URL once per machine (user environment variable), e.g.

    setx CAM_URL rtsp://USER:PASSWORD@192.168.0.2:554/cam0_0

cam_url()          -> that URL (default stream)
cam_url("cam0_1")  -> the same camera/login with another stream path
cam_url("rtsp://...") -> returned unchanged (explicit URL wins)
"""
from __future__ import annotations

import os
from urllib.parse import urlsplit, urlunsplit

# no credentials here: without CAM_URL set the camera will refuse the stream
DEFAULT_CAM_URL = "rtsp://192.168.0.2:554/cam0_0"


def cam_url(stream: str | None = None) -> str:
    url = os.environ.get("CAM_URL", DEFAULT_CAM_URL)
    if not stream:
        return url
    if "://" in stream:
        return stream
    parts = urlsplit(url)
    return urlunsplit(parts._replace(path="/" + stream.lstrip("/")))
