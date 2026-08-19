import asyncio
from types import SimpleNamespace

import pytest
from controller_manager_msgs.srv import ListControllers, SwitchController
from rmi import ControllerManagerError, ControllerManagerClient


class FakeServiceClient:
    def __init__(self, response, *, ready=True):
        self.response = response
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = asyncio.get_running_loop().create_future()
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


def test_strict_switch_is_verified_with_list_controllers():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(
            controller=[
                _controller("arm_jtc", "inactive"),
                _controller("arm_servo", "active"),
            ]
        ),
    )
    client = ControllerManagerClient(node, "/controller_manager/", timeout_sec=2.0)

    asyncio.run(
        client.switch_controller(activate=("arm_servo",), deactivate=("arm_jtc",))
    )

    request = node.clients[0].requests[0]
    assert request.strictness == SwitchController.Request.STRICT
    assert request.activate_controllers == ["arm_servo"]
    assert request.deactivate_controllers == ["arm_jtc"]
    assert node.created == [
        (SwitchController, "/controller_manager/switch_controller"),
        (ListControllers, "/controller_manager/list_controllers"),
    ]


def test_rejected_switch_raises_and_is_not_verified():
    node = FakeNode(
        SimpleNamespace(ok=False),
        SimpleNamespace(controller=[_controller("arm_servo", "inactive")]),
    )
    client = ControllerManagerClient(node, "/controller_manager")

    with pytest.raises(ControllerManagerError, match="rejected STRICT switch"):
        asyncio.run(client.switch_controller(activate=("arm_servo",), deactivate=()))

    assert len(node.clients[0].requests) == 1
    assert len(node.clients[1].requests) == 1


def test_unloaded_activate_is_skipped():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(controller=[_controller("arm_jsic", "inactive")]),
    )
    client = ControllerManagerClient(node, "/controller_manager")

    asyncio.run(
        client.switch_controller(
            activate=("pika_gripper_fwd",),
            deactivate=(),
        )
    )

    assert node.clients[0].requests == []


def test_wrong_post_switch_state_raises():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(controller=[_controller("arm_servo", "inactive")]),
    )
    client = ControllerManagerClient(node, "/controller_manager")

    with pytest.raises(ControllerManagerError, match="arm_servo"):
        asyncio.run(client.switch_controller(activate=("arm_servo",), deactivate=()))


def test_active_controllers_filters_declared_conflict_set():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(
            controller=[
                _controller("arm_jtc", "inactive"),
                _controller("arm_servo", "active"),
                _controller("joint_state_broadcaster", "active"),
            ]
        ),
    )
    client = ControllerManagerClient(node, "/controller_manager")

    active = asyncio.run(client.active_controllers(("arm_jtc", "arm_servo")))

    assert active == ("arm_servo",)


def test_deactivate_only_switch_is_supported_and_verified():
    node = FakeNode(
        SimpleNamespace(ok=True),
        SimpleNamespace(controller=[_controller("arm_servo", "inactive")]),
    )
    client = ControllerManagerClient(node, "/controller_manager")

    asyncio.run(client.switch_controller(activate=(), deactivate=("arm_servo",)))

    request = node.clients[0].requests[0]
    assert request.activate_controllers == []
    assert request.deactivate_controllers == ["arm_servo"]


def test_unavailable_service_has_bounded_failure():
    node = FakeNode(SimpleNamespace(ok=True), SimpleNamespace(controller=[]))
    node.clients[0].ready = False
    client = ControllerManagerClient(node, "/controller_manager", timeout_sec=0.1)

    with pytest.raises(TimeoutError, match="switch_controller unavailable"):
        asyncio.run(client.switch_controller(activate=("arm_servo",), deactivate=()))
