"""Unit tests for RMI ResetHandler and Context.reset()."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from rmi.reset import ResetHandler, ResetMode


class DummyProfile:
    def __init__(self, homing: dict | None = None) -> None:
        self.raw_data = {"homing": homing} if homing is not None else {}
        self.parts = {}


class DummyContext:
    def __init__(self, profile: DummyProfile | None = None) -> None:
        self.profile = profile or DummyProfile()
        self.node = MagicMock()
        self.robot = MagicMock()

    def make_node(self, name: str) -> MagicMock:
        node = MagicMock()
        node.activate.return_value.__enter__ = MagicMock()
        node.activate.return_value.__exit__ = MagicMock()
        return node


def test_reset_mode_parsing() -> None:
    assert ResetMode.parse("auto") == ResetMode.AUTO
    assert ResetMode.parse("SIM") == ResetMode.SIM
    assert ResetMode.parse("homing") == ResetMode.HOMING
    assert ResetMode.parse("manual") == ResetMode.INTERACTIVE
    assert ResetMode.parse("none") == ResetMode.NONE
    with pytest.raises(ValueError):
        ResetMode.parse("invalid_mode")


def test_reset_none_mode() -> None:
    ctx = DummyContext()
    handler = ResetHandler(ctx)
    assert handler.reset(ResetMode.NONE) is True


def test_reset_custom_mode() -> None:
    ctx = DummyContext()
    handler = ResetHandler(ctx)
    called = []

    def my_fn(c: DummyContext) -> bool:
        called.append(c)
        return True

    res = handler.reset(ResetMode.CUSTOM, custom_fn=my_fn)
    assert res is True
    assert called == [ctx]

    with pytest.raises(ValueError):
        handler.reset(ResetMode.CUSTOM, custom_fn=None)


def test_reset_sim_mode_fails_if_no_service(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_srv = MagicMock()
    monkeypatch.setattr("rmi.reset.ResetWorld", fake_srv)

    ctx = DummyContext()
    # Mock node create_client wait_for_service returning False
    mock_client = MagicMock()
    mock_client.wait_for_service.return_value = False
    ctx.node.create_client.return_value = mock_client

    handler = ResetHandler(ctx)
    with pytest.raises(RuntimeError):
        handler.reset(ResetMode.SIM, timeout_sec=0.1)


def test_reset_sim_mode_succeeds_when_service_returns_ok(monkeypatch: pytest.MonkeyPatch) -> None:
    # Ensure ResetWorld is available in test
    fake_srv = MagicMock()
    monkeypatch.setattr("rmi.reset.ResetWorld", fake_srv)

    ctx = DummyContext()
    mock_client = MagicMock()
    mock_client.wait_for_service.return_value = True

    mock_future = MagicMock()
    mock_future.done.return_value = True
    mock_resp = MagicMock()
    mock_resp.success = True
    mock_future.result.return_value = mock_resp
    mock_client.call_async.return_value = mock_future

    ctx.node.create_client.return_value = mock_client

    handler = ResetHandler(ctx)
    assert handler.reset(ResetMode.SIM, keyframe="home") is True
    mock_client.call_async.assert_called_once()
