from orchestrator.registry import NodeRegistry, default_registry
from orchestrator.tree import Node, Status


class _Dummy(Node):
    def update(self):
        return Status.SUCCESS


def test_default_registry_lists_control_and_leaf_tags():
    registry = default_registry()
    tags = set(registry.known_tags())
    assert {"Sequence", "Fallback", "Parallel", "Retry"} <= tags
    assert {"RunPolicy", "ExecuteRecoveryPlan", "RecordEpisode", "PolicySkill"} <= tags


def test_catalog_reports_child_arity_for_web_ui():
    registry = default_registry()
    by_tag = {entry["tag"]: entry for entry in registry.catalog()}
    assert by_tag["Sequence"]["child_arity"] == "many"
    assert by_tag["RecordEpisode"]["child_arity"] == "one"
    assert by_tag["RunPolicy"]["child_arity"] == "none"


def test_register_duplicate_tag_raises():
    registry = NodeRegistry()
    registry.register("Dummy", _Dummy)
    try:
        registry.register("Dummy", _Dummy)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for duplicate tag")


def test_create_unknown_tag_raises():
    registry = NodeRegistry()
    try:
        registry.create("Nope")
    except KeyError:
        pass
    else:
        raise AssertionError("expected KeyError for unknown tag")
