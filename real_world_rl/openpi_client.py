"""Torch-free client for the openpi websocket policy protocol."""
from __future__ import annotations

from typing import Any

import msgpack
import numpy as np


def _wire_dtype(dtype: np.dtype) -> np.dtype:
    if dtype.kind in "VOc":
        raise ValueError(f"unsupported openpi dtype {dtype}")
    return dtype


def _encode_numpy(value: Any) -> dict:
    if isinstance(value, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": value.tobytes(),
            b"dtype": _wire_dtype(value.dtype).str,
            b"shape": value.shape,
        }
    if isinstance(value, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": value.item(),
            b"dtype": _wire_dtype(value.dtype).str,
        }
    raise TypeError(f"cannot encode {type(value).__name__} for openpi")


def _decode_numpy(value: dict) -> Any:
    if b"__ndarray__" in value:
        dtype = _wire_dtype(np.dtype(value[b"dtype"]))
        return np.frombuffer(value[b"data"], dtype=dtype).reshape(value[b"shape"])
    if b"__npgeneric__" in value:
        return _wire_dtype(np.dtype(value[b"dtype"])).type(value[b"data"])
    return value


def packb(value: dict) -> bytes:
    return msgpack.packb(value, default=_encode_numpy, use_bin_type=True)


def unpackb(frame: bytes) -> Any:
    return msgpack.unpackb(frame, object_hook=_decode_numpy, raw=False)


class OpenPiError(RuntimeError):
    """A policy connection or protocol failure."""


class OpenPiClient:
    """One connection with one outstanding inference at a time."""

    def __init__(self, host: str, port: int = 8000, *, api_key: str | None = None,
                 timeout_s: float = 30.0) -> None:
        # Keep the optional dependency lazy, including for codec-only users.
        try:
            from websockets.exceptions import WebSocketException
            from websockets.sync.client import connect
        except ModuleNotFoundError as error:
            if (error.name or "").split(".")[0] != "websockets":
                raise
            raise OpenPiError("install the openpi-client extra: pip install -e '.[openpi-client]'") from error

        self._timeout_s = timeout_s
        self._connection_errors = (OSError, WebSocketException)
        uri = f"ws://{host}:{port}" if ":" not in host else f"ws://[{host.strip('[]')}]:{port}"
        headers = {"Authorization": f"Api-Key {api_key}"} if api_key else None
        try:
            self._socket = connect(
                uri, additional_headers=headers, compression=None, max_size=None,
                open_timeout=timeout_s, close_timeout=1,
            )
        except self._connection_errors as error:
            raise OpenPiError(f"could not connect to openpi at {uri}: {error}") from error
        try:
            self.metadata = self._receive("metadata")
        except BaseException:
            self.close()
            raise

    def _receive(self, phase: str) -> dict:
        try:
            frame = self._socket.recv(timeout=self._timeout_s)
        except self._connection_errors as error:
            raise OpenPiError(f"openpi {phase} receive failed: {error}") from error
        if isinstance(frame, str):
            raise OpenPiError(f"openpi server error during {phase}: {frame}")
        try:
            result = unpackb(frame)
        except (ValueError, TypeError, KeyError, OverflowError, msgpack.UnpackException) as error:
            raise OpenPiError(f"invalid openpi {phase} frame: {error}") from error
        if not isinstance(result, dict):
            raise OpenPiError(f"openpi {phase} must be a map")
        return result

    def infer(self, observation: dict) -> dict:
        try:
            self._socket.send(packb(observation))
        except self._connection_errors as error:
            raise OpenPiError(f"openpi request send failed: {error}") from error
        return self._receive("inference")

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> OpenPiClient:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
