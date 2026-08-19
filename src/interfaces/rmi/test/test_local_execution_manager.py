from pathlib import Path
from types import SimpleNamespace

from controller_manager_msgs.srv import ListControllers, SwitchController
from rmi import EmbodimentConfig
from rmi.execution import LocalExecutionManager

PROFILE = (
    Path(__file__).resolve().parents[4]
    / "apps"
    / "profiles"
    / "fr3_pika_single_arm.yaml"
)


class FakeServiceClient:
    def __init__(self, response, *, ready=True):
        self.response = response
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = __import__("asyncio").get_running_loop().create_future()
        future.set_result(self.response)
        return future


class FakeNode:
    def __init__(self, switch_response, list_response):
        self.clients = [
            FakeServiceClient(switch_response),
            FakeServiceClient(list_response),
        ]
        self.created = []

    def create_client(self, service_type, name):
        self.created.append((service_type, name))
        return self.clients[len(self.created) - 1]


def _controller(name, state):
    return SimpleNamespace(name=name, state=state)


def test_acquire_skips_switch_when_target_controller_is_already_active():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(
            controller=[
                _controller("franka_arm_jtc", "active"),
                _controller("franka_arm_jsic", "inactive"),
                _controller("franka_arm_tsjic", "inactive"),
            ]
        ),
    )
    em = LocalExecutionManager(EmbodimentConfig.from_yaml(PROFILE), node=node)

    generation = em.acquire("Planner")

    assert generation == 1
    assert em.get_allocations()["arm"]["provider"] == "Planner"
    assert node.created == [
        (SwitchController, "/controller_manager/switch_controller"),
        (ListControllers, "/controller_manager/list_controllers"),
    ]
    assert node.clients[0].requests == []


def test_release_clears_allocation_without_switching():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(
            controller=[
                _controller("franka_arm_jtc", "active"),
                _controller("franka_arm_jsic", "inactive"),
                _controller("franka_arm_tsjic", "inactive"),
            ]
        ),
    )
    em = LocalExecutionManager(EmbodimentConfig.from_yaml(PROFILE), node=node)
    em.acquire("Planner")
    node.clients[0].requests.clear()
    node.clients[1].requests.clear()

    assert em.release("Planner") is True
    assert em.get_allocations() == {}
    assert node.clients[0].requests == []
    assert node.clients[1].requests == []
