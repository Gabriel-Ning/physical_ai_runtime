from pathlib import Path

import pytest
from orchestrator.cli import _load_context_factory


def test_context_factory_loader_uses_explicit_module_callable_contract():
    assert _load_context_factory("pathlib:Path") is Path


def test_context_factory_loader_rejects_ambiguous_spec():
    with pytest.raises(ValueError, match="module:callable"):
        _load_context_factory("missing_separator")


def test_context_factory_loader_rejects_non_callable_attribute():
    with pytest.raises(TypeError, match="not callable"):
        _load_context_factory("os:name")
