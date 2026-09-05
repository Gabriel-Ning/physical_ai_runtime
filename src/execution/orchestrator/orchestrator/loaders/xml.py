"""XML tree-file loader.

Tree file shape (attributes become leaf/decorator/control-node config;
element nesting becomes ``children``/``child``)::

    <BehaviorTree name="pick_and_place">
      <RecordEpisode task="pick">
        <Sequence>
          <RunPolicy name="PickPolicy" policy="Pick_v1" />
          <Fallback>
            <RunPolicy name="AlignPolicy" policy="Align_v2" />
            <ExecuteRecoveryPlan planner="default" planner_source="Planner" />
          </Fallback>
          <RunPolicy name="PlacePolicy" policy="Place_v1" />
        </Sequence>
      </RecordEpisode>
    </BehaviorTree>
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from ..registry import NodeRegistry
from ..tree import Node


def _coerce(value: str) -> Any:
    lowered = value.strip().lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    return value


def _build_node(element: ET.Element, registry: NodeRegistry) -> Node:
    tag = element.tag
    config = {key: _coerce(value) for key, value in element.attrib.items()}
    name = config.pop("name", None)
    arity = registry.child_arity(tag)

    if arity == "many":
        config["children"] = [_build_node(child, registry) for child in element]
    elif arity == "one":
        children = list(element)
        if len(children) != 1:
            raise ValueError(f"<{tag}> requires exactly one child element, got {len(children)}")
        config["child"] = _build_node(children[0], registry)
    elif list(element):
        raise ValueError(f"<{tag}> is a leaf node and must not contain child elements")

    return registry.create(tag, name=name, **config)


def load_tree(source: str | Path, registry: NodeRegistry) -> Node:
    """Parse one ``<BehaviorTree>`` XML file into a bound-free Node tree.

    The returned root still needs :meth:`orchestrator.tree.Node.bind` (done
    automatically by :class:`orchestrator.runtime.BehaviorTreeRuntime`).
    """
    root_element = ET.parse(str(source)).getroot()
    if root_element.tag != "BehaviorTree":
        raise ValueError(f"tree file root must be <BehaviorTree>, got <{root_element.tag}>")
    children = list(root_element)
    if len(children) != 1:
        raise ValueError("<BehaviorTree> must contain exactly one root node")
    return _build_node(children[0], registry)


def load_tree_from_string(xml_text: str, registry: NodeRegistry) -> Node:
    root_element = ET.fromstring(xml_text)
    if root_element.tag != "BehaviorTree":
        raise ValueError(f"tree file root must be <BehaviorTree>, got <{root_element.tag}>")
    children = list(root_element)
    if len(children) != 1:
        raise ValueError("<BehaviorTree> must contain exactly one root node")
    return _build_node(children[0], registry)
