import pytest
from orchestrator.blackboard import Blackboard


def test_get_set_default():
    bb = Blackboard()
    assert bb.get("missing") is None
    assert bb.get("missing", 42) == 42
    bb.set("key", "value")
    assert bb.get("key") == "value"
    assert bb.has("key")
    assert not bb.has("other")


def test_attribute_access():
    bb = Blackboard()
    bb.recovery_target = {"x": 1}
    assert bb.recovery_target == {"x": 1}
    assert "recovery_target" in bb


def test_attribute_access_missing_raises():
    bb = Blackboard()
    with pytest.raises(AttributeError):
        _ = bb.nope


def test_clear_and_snapshot():
    bb = Blackboard({"a": 1, "b": 2})
    assert bb.snapshot() == {"a": 1, "b": 2}
    bb.clear("a")
    assert not bb.has("a")
    bb.clear()
    assert bb.snapshot() == {}
