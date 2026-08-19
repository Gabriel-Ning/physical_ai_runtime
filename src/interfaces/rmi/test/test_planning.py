from types import MappingProxyType, SimpleNamespace

import pytest
from rmi import (
    CartesianStreamer,
    JointStreamer,
    Observation,
    PlannerCatalog,
    Planner,
    Resolver,
)
from rmi.planning import (
    CartesianState,
    JointHorizonPoint,
    JointHorizonResult,
    JointState,
    PlanPoint,
    PlanResult,
    PoseHorizonPoint,
    PoseHorizonResult,
    ResolveResult,
)


class _Backend:
    def __init__(self) -> None:
        self.warmups = 0
        self.calls = []
        self.world = "unset"

    def warmup(self) -> None:
        self.warmups += 1

    def update_world(self, world) -> None:
        self.world = world

    def plan(self, start_state, target, options):
        self.calls.append((start_state, target, options))
        return PlanResult(
            valid=True,
            joint_names=start_state.joint_names,
            points=[PlanPoint(positions=target.positions, time_from_start_s=1.0)],
        )


class _ResolverBackend(_Backend):
    def resolve(self, start_state, target, options):
        self.calls.append((start_state, target, options))
        return ResolveResult(
            valid=True,
            joint_names=start_state.joint_names,
            positions=[0.7, 0.8],
            diagnostics={"solver": "fake_ik"},
        )


class _JointStreamerBackend(_Backend):
    def reset(self, state):
        self.reset_state = state

    def update_target(self, target):
        self.target = target

    def step(self, state, dt):
        self.calls.append((state, dt))
        return JointHorizonResult(
            valid=True,
            points=[JointHorizonPoint(positions=[0.4, 0.5], time_from_start_s=dt)],
        )


class _CartesianStreamerBackend(_Backend):
    def reset(self, state):
        self.reset_state = state

    def update_target(self, target):
        self.target = target

    def step(self, state, dt):
        self.calls.append((state, dt))
        return PoseHorizonResult(
            valid=True,
            points=[
                PoseHorizonPoint(
                    position_xyz=(0.1, 0.2, 0.3),
                    orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                    time_from_start_s=dt,
                )
            ],
        )


def _observation() -> Observation:
    return Observation(
        data=MappingProxyType(
            {
                "joint_names": ("j1", "j2"),
                "joint_positions": (0.1, 0.2),
                "joint_velocities": (0.3, 0.4),
            }
        ),
        source_time_s=12.5,
        receive_time_s=12.6,
    )


def test_planner_reads_timestamped_robot_state_without_executing() -> None:
    backend = _Backend()
    planner = Planner("fake", backend)
    robot = SimpleNamespace(state=_observation())
    target = JointState(joint_names=["j1", "j2"], positions=[0.8, 0.9])

    result = planner.plan(robot=robot, target=target, options={"attempts": 2})

    assert result.valid
    start, received_target, options = backend.calls[0]
    assert start == JointState(
        joint_names=["j1", "j2"],
        positions=[0.1, 0.2],
        velocities=[0.3, 0.4],
        stamp_s=12.5,
    )
    assert received_target is target
    assert options == {"attempts": 2}


def test_explicit_start_does_not_read_robot() -> None:
    backend = _Backend()
    planner = Planner("fake", backend)
    start = JointState(joint_names=["j1"], positions=[0.0], stamp_s=3.0)
    target = JointState(joint_names=["j1"], positions=[1.0])

    planner.plan(robot=SimpleNamespace(), start=start, target=target)

    assert backend.calls[0][0] is start


def test_catalog_constructs_and_warms_backend_lazily_once() -> None:
    backend = _Backend()
    factory_calls = []
    catalog = PlannerCatalog()
    catalog.register("CuRobo", lambda: factory_calls.append(True) or backend)

    assert catalog.available() == ["curobo"]
    assert factory_calls == []
    first = catalog.make("curobo")
    second = catalog.make("CUROBO")

    assert factory_calls == [True]
    assert backend.warmups == 1
    assert first._backend is second._backend


def test_planner_returns_explicit_invalid_result_for_malformed_observation() -> None:
    planner = Planner("fake", _Backend())
    observation = Observation(
        data={"joint_names": ("j1", "j2"), "joint_positions": (0.1,)},
        source_time_s=1.0,
        receive_time_s=1.1,
    )

    result = planner.plan(
        robot=SimpleNamespace(state=observation),
        target=JointState(joint_names=["j1"], positions=[0.2]),
    )

    assert not result.valid
    assert "mismatched joint names" in result.reason


def test_planner_returns_explicit_invalid_result_for_stale_observation() -> None:
    backend = _Backend()
    planner = Planner("fake", backend, clock=lambda: 20.0)

    result = planner.plan(
        robot=SimpleNamespace(state=_observation()),
        target=JointState(joint_names=["j1", "j2"], positions=[0.8, 0.9]),
        max_state_age_s=1.0,
    )

    assert not result.valid
    assert "state is stale" in result.reason
    assert result.diagnostics["state_age_s"] == pytest.approx(7.4)
    assert backend.calls == []


def test_resolver_is_a_distinct_one_request_ik_facade() -> None:
    backend = _ResolverBackend()
    resolver = Resolver("ik", backend)
    target = CartesianState(position_xyz=(0.4, 0.0, 0.5))

    result = resolver.resolve(robot=SimpleNamespace(state=_observation()), target=target)

    assert result.valid
    assert result.positions == [0.7, 0.8]
    assert result.diagnostics == {"solver": "fake_ik"}
    assert backend.calls[0][0].stamp_s == 12.5


def test_joint_streamer_exposes_app_owned_reset_target_step_cycle() -> None:
    backend = _JointStreamerBackend()
    streamer = JointStreamer("mpc", backend)
    robot = SimpleNamespace(state=_observation())
    target = JointState(joint_names=["j1", "j2"], positions=[0.8, 0.9])

    streamer.reset(robot=robot)
    streamer.set_target(target)
    result = streamer.step(robot=robot, dt=0.02)

    assert backend.reset_state.stamp_s == 12.5
    assert backend.target is target
    assert result.valid
    assert result.points[0].time_from_start_s == 0.02


def test_cartesian_streamer_requires_explicit_measured_pose() -> None:
    backend = _CartesianStreamerBackend()
    streamer = CartesianStreamer("ndcurves", backend)
    current = CartesianState(position_xyz=(0.0, 0.0, 0.3), stamp_s=4.0)
    target = CartesianState(position_xyz=(0.2, 0.0, 0.3))

    streamer.reset(current)
    streamer.set_target(target)
    result = streamer.step(current, dt=0.02)

    assert backend.reset_state is current
    assert backend.target is target
    assert backend.calls == [(current, 0.02)]
    assert result.valid


def test_catalog_keeps_backend_families_in_separate_namespaces() -> None:
    planner = _Backend()
    resolver = _ResolverBackend()
    joint_streamer = _JointStreamerBackend()
    cartesian_streamer = _CartesianStreamerBackend()
    catalog = PlannerCatalog()
    catalog.register("shared", lambda: planner, warmup_on_create=False)
    catalog.register_resolver("shared", lambda: resolver, warmup_on_create=False)
    catalog.register_joint_streamer(
        "shared", lambda: joint_streamer, warmup_on_create=False
    )
    catalog.register_cartesian_streamer(
        "shared", lambda: cartesian_streamer, warmup_on_create=False
    )

    assert isinstance(catalog.make("shared"), Planner)
    assert isinstance(catalog.make_resolver("shared"), Resolver)
    assert isinstance(catalog.make_joint_streamer("shared"), JointStreamer)
    assert isinstance(
        catalog.make_cartesian_streamer("shared"), CartesianStreamer
    )
