import hashlib
import sys
from types import ModuleType, SimpleNamespace

from robot_infra.devices.provenance import sdk_artifacts


def test_sdk_fingerprints_facade_and_loaded_implementation(tmp_path, monkeypatch):
    facade = tmp_path / '__init__.py'
    binary = tmp_path / '_native.so'
    facade.write_bytes(b'facade')
    binary.write_bytes(b'native implementation')
    module = ModuleType('_test_native')
    module.__file__ = str(binary)
    monkeypatch.setitem(sys.modules, module.__name__, module)
    implementation = type('Robot', (), {'__module__': module.__name__})
    result = sdk_artifacts(SimpleNamespace(__file__=str(facade), __name__='sdk'),
                           implementation_type=implementation)
    assert [row['sha256'] for row in result['artifacts']] == [
        hashlib.sha256(b'facade').hexdigest(), hashlib.sha256(b'native implementation').hexdigest()]
    binary.write_bytes(b'changed implementation')
    changed = sdk_artifacts(module)['artifacts'][0]['sha256']
    assert changed != result['artifacts'][1]['sha256']


def test_missing_artifacts_are_evidence_gaps_not_fabricated_hashes(tmp_path):
    result = sdk_artifacts(SimpleNamespace(__file__=str(tmp_path / 'missing.so')),
                           implementation_type=type('Robot', (), {'__module__': '_missing_sdk'}))
    for row in result['artifacts']:
        assert row['sha256'] is None
        assert row['unavailable_reason']


def test_python_wrapper_includes_native_superclass_module(tmp_path, monkeypatch):
    path = tmp_path / '_native.so'
    path.write_bytes(b'native superclass')
    native = ModuleType('_test_native_base')
    native.__file__ = str(path)
    monkeypatch.setitem(sys.modules, native.__name__, native)
    base = type('NativeRobot', (), {'__module__': native.__name__})
    wrapper = type('Robot', (base,), {'__module__': __name__})
    artifacts = sdk_artifacts(SimpleNamespace(), implementation_type=wrapper)['artifacts']
    row = next(row for row in artifacts if row['role'] == 'implementation_base')
    assert row['sha256'] == hashlib.sha256(b'native superclass').hexdigest()
