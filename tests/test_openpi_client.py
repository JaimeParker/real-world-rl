"""Openpi's msgpack-numpy contract without a learning stack."""
import subprocess
import sys

import msgpack
import numpy as np
import pytest

from real_world_rl.openpi_client import packb, unpackb


def test_codec_uses_openpi_binary_markers_and_text_dtype():
    image = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)[:, ::-1]
    actions = np.array([[0.5, -0.25]], dtype=">f4")
    timing = np.float32(1.25)
    payload = {"observation/image": image, "actions": actions, "infer_ms": timing, "prompt": "抬起"}
    expected = {
        "observation/image": {
            b"__ndarray__": True, b"data": image.tobytes(), b"dtype": "|u1", b"shape": [2, 3, 3],
        },
        "actions": {
            b"__ndarray__": True, b"data": bytes.fromhex("3f000000be800000"),
            b"dtype": ">f4", b"shape": [1, 2],
        },
        "infer_ms": {b"__npgeneric__": True, b"data": 1.25, b"dtype": np.dtype("f4").str},
        "prompt": "抬起",
    }
    frame = msgpack.packb(expected, use_bin_type=True)
    assert packb(payload) == frame
    result = unpackb(frame)
    np.testing.assert_array_equal(result["observation/image"], image)
    np.testing.assert_array_equal(result["actions"], actions)
    assert result["actions"].dtype == actions.dtype
    assert isinstance(result["infer_ms"], np.float32)
    assert result["prompt"] == "抬起"


@pytest.mark.parametrize("dtype", [object, complex, [("value", "f4")]])
def test_codec_rejects_dtypes_excluded_by_openpi(dtype):
    with pytest.raises(ValueError, match="unsupported openpi dtype"):
        packb({"array": np.zeros(1, dtype=dtype)})
    with pytest.raises(ValueError, match="unsupported openpi dtype"):
        unpackb(msgpack.packb({
            b"__ndarray__": True, b"dtype": np.dtype(dtype).str, b"shape": [1], b"data": b""
        }))


def test_client_imports_without_learning_or_websockets():
    subprocess.run([sys.executable, "-c", """
import sys
for name in ('torch', 'rl_garden', 'gymnasium', 'websockets'):
    sys.modules[name] = None
from real_world_rl.openpi_client import packb, unpackb
assert unpackb(packb({'prompt': 'test'})) == {'prompt': 'test'}
"""], check=True)
