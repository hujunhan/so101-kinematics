import numpy as np
import pytest
from so101_core.arm import SafeArm, StallError
from so101_core.config import JointMap
from so101_core.convert import lerobot_to_rad, rad_to_lerobot
from so101_core.model import JOINTS
from so101_core.safety import SafetyConfig

from so101_kinematics import s3
from so101_kinematics.models import PinArm


class FakeBus:
    """Servos that follow the goal with a first-order lag (same idea as so101-core's tests)."""

    def __init__(self, jm, blocked=None):
        self.jm, self.blocked = jm, blocked
        self.q = np.zeros(len(JOINTS))
        self.goal = self.q.copy()
        self.is_connected, self.is_calibrated, self.torque = False, True, False
        self.sent = []

    def connect(self):
        self.is_connected = True

    def disconnect(self, disable_torque=True):
        self.is_connected = False

    def disable_torque(self):
        self.torque = False

    def sync_read(self, name, normalize=True, num_retry=0):
        if self.torque:
            step = 0.5 * (self.goal - self.q)
            if self.blocked is not None:
                step[self.blocked] = 0.0
            self.q = self.q + step
        return {k.removesuffix(".pos"): v for k, v in rad_to_lerobot(self.q, self.jm).items()}

    def sync_write(self, name, values, normalize=True):
        if normalize:
            self.goal = lerobot_to_rad(values, self.jm)
            self.sent.append(self.goal.copy())


class FakeRobot:
    def __init__(self, bus):
        self.bus = bus

    def configure(self):
        self.bus.torque = True


def make_arm(blocked=None):
    jm = JointMap()
    bus = FakeBus(jm, blocked)
    arm = SafeArm(
        safety=SafetyConfig(rate_hz=1000.0), joint_map=jm, robot=FakeRobot(bus), confirm_release=lambda: None
    )
    return arm.connect(torque=True), bus


FAST = dict(speed_cm_s=50.0, ramp_speed=np.radians(2000))  # the fake runs at 1 kHz, so go quickly


@pytest.fixture(scope="module")
def kin():
    return PinArm()


def test_draw_follows_the_path_and_writes_a_log(kin, tmp_path):
    arm, bus = make_arm()
    out = []
    path = s3.draw(arm, kin, "square", confirm=lambda m: True, log_dir=tmp_path, out=out.append, **FAST)
    log = np.load(path)
    assert log["q_cmd"].shape == log["q_meas"].shape and len(log["t"]) == len(log["q_cmd"])
    assert str(log["status"]) == "complete"
    # the command follows the plan; the (fake) servo stays close to the command
    assert np.allclose(log["q_cmd"][1:], log["planned"][1:], atol=1e-9)
    assert np.degrees(np.abs(log["q_meas"] - log["q_cmd"]).max()) < 2.0
    assert any(line.startswith("complete") for line in out)


def test_nothing_moves_if_the_user_declines(kin, tmp_path):
    arm, bus = make_arm()
    assert (
        s3.draw(arm, kin, "circle", confirm=lambda m: False, log_dir=tmp_path, out=lambda s: None, **FAST)
        is None
    )
    assert bus.sent == []  # no command ever reached the servos
    assert list(tmp_path.iterdir()) == []


def test_draw_refuses_when_preflight_would_change_the_path(kin, tmp_path, monkeypatch):
    arm, bus = make_arm()
    monkeypatch.setattr(s3, "preflight", lambda ticks, safety: (1, 0))
    with pytest.raises(RuntimeError, match="preflight"):
        s3.draw(arm, kin, "circle", confirm=lambda m: True, log_dir=tmp_path, out=lambda s: None, **FAST)
    assert bus.sent == []


def test_a_blocked_joint_stops_the_arm_and_holds(kin, tmp_path):
    arm, bus = make_arm(blocked=1)
    with pytest.raises(StallError):
        s3.draw(arm, kin, "circle", confirm=lambda m: True, log_dir=tmp_path, out=lambda s: None, **FAST)
    assert np.allclose(bus.goal, bus.q)  # holding, not pushing on


def test_readonly_prints_joints_and_tool_position(kin):
    arm = SafeArm(
        safety=SafetyConfig(),
        joint_map=JointMap(),
        robot=FakeRobot(FakeBus(JointMap())),
        confirm_release=lambda: None,
    )
    arm.connect(torque=False)
    out = []
    s3.read_only(arm, kin, seconds=0.3, out=out.append)
    assert out and "tool (cm)" in out[0]


def test_a_stall_while_drawing_keeps_the_log(kin, tmp_path, monkeypatch):
    arm, bus = make_arm()
    real_follow = arm.follow

    def follow_then_block(*a, **k):
        bus.blocked = 2  # the elbow gets stuck once drawing has started
        return real_follow(*a, **k)

    monkeypatch.setattr(arm, "follow", follow_then_block)
    with pytest.raises(StallError):
        s3.draw(
            arm,
            kin,
            "circle",
            confirm=lambda m: True,
            stall_s=0.01,
            log_dir=tmp_path,
            out=lambda s: None,
            **FAST,
        )
    (log_file,) = tmp_path.iterdir()
    log = np.load(log_file)
    assert str(log["status"]).startswith("stopped: StallError") and len(log["t"]) > 0
