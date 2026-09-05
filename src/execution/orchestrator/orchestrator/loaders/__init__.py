"""Tree-file loaders (currently XML; see :mod:`orchestrator.loaders.xml`)."""

from .xml import load_tree, load_tree_from_string

__all__ = ["load_tree", "load_tree_from_string"]
