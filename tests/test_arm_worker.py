"""`ArmWorker` 的离线测试（假臂，不需要硬件）。

最重要的一条是 §7.4 的硬要求：**worker 卡在 `movej` 期间，旁路急停仍能发出**。
那条判据的价值在于：如果哪天有人把急停改成"投进 worker 队列"，它会**红**。
"""
from __future__ import annotations

import threading
import time

import pytest

from liteteleop import arm_worker, servo
from liteteleop.arm_worker import (
    ROLE_MASTER,
    ROLE_SLAVE,
    ArmWorker,
    Snapshot,
    StateHookMissing,
    TeleopParams,
    attach_state_hook,
)


# ────────────────────────── 状态钩子（唯一私有耦合）──────────────────────────

class _FakeAck:
    def __init__(self):
        self.state = None

    def _on_status(self, payload: bytes) -> None:      # SDK 的钩子缝隙
        pass

    def feed(self, st) -> None:
        self.state = st
        self._on_status(b"<frame>")


class _FakeState:
    def __init__(self, q):
        self.q = list(q)
        self.dq = [0.0] * len(q)
        self.tau = [0.0] * len(q)
        self.joints = [type("J", (), {"t_mos": 30.0, "t_coil": 28.0, "err": 1})()
                       for _ in q]
        self.mode_name = "MOVE_J"
        self.enabled = True
        self.faulted = False
        self.joint_fault = 0
        self.flags = 0
        self.flag_names = []
        self.seq = 0


def test_state_hook_missing_raises():
    """SDK 若改了内部结构，**启动即报** —— 不要"连上了但界面不动"。"""
    class _NoSeam:
        pass

    with pytest.raises(StateHookMissing):
        attach_state_hook(_NoSeam(), lambda s: None)


def test_state_hook_forwards_every_frame():
    arm = type("A", (), {})()
    arm._a = _FakeAck()
    got = []
    attach_state_hook(arm, got.append)
    arm._a.feed(_FakeState([0.1] * 7))
    arm._a.feed(_FakeState([0.2] * 7))
    assert len(got) == 2 and got[-1].q[0] == 0.2


# ────────────────────────── 急停旁路（§7.4 硬要求）──────────────────────────

class _StuckArm:
    """`movej` 会长时间阻塞 —— 模拟 worker 卡在收尾 `movej` 里。"""

    def __init__(self):
        self.in_movej = threading.Event()
        self.estop_called = threading.Event()
        self.movej_block = 2.0

    def movej(self, q, speed=1.0):
        self.in_movej.set()
        time.sleep(self.movej_block)

    def emergency_stop(self):
        self.estop_called.set()


def test_estop_bypasses_a_worker_stuck_in_movej():
    """⛔ **急停不许排在 worker 队列后面**（那最长要等 `move_timeout`）。

    判别力：若有人把 `emergency_stop` 改成投进 `_cmds` 队列，本用例会**超时失败**。
    """
    arm = _StuckArm()
    w = ArmWorker()
    w._arm = arm
    threading.Thread(target=lambda: arm.movej([0.0] * 7), daemon=True).start()
    assert arm.in_movej.wait(1.0), "假臂没进 movej"

    t0 = time.monotonic()
    w.emergency_stop()                                  # ⛔ 旁路，不排队
    assert arm.estop_called.wait(0.5), "急停被卡在 movej 后面了 —— 旁路失效"
    assert time.monotonic() - t0 < 0.5


def test_estop_without_arm_reports_failure():
    w = ArmWorker()
    w.emergency_stop()
    assert w.emergency_outcome() == (False, "未连接")


def test_estop_outcome_captures_exception():
    class _Bad:
        def emergency_stop(self):
            raise RuntimeError("链路断了")

    w = ArmWorker()
    w._arm = _Bad()
    w.emergency_stop()
    for _ in range(50):
        if w.emergency_outcome() is not None:
            break
        time.sleep(0.01)
    ok, err = w.emergency_outcome()
    assert ok is False and "链路断了" in err


# ────────────────────────── 快照 ──────────────────────────

def test_snapshot_defaults_are_safe():
    """未连接时快照必须是一份**安全的空值**，不是 None。"""
    s = ArmWorker().snapshot()
    assert isinstance(s, Snapshot)
    assert s.connected is False and s.teleop_active is False
    assert s.q == [] and s.err == []


def test_snapshot_reflects_pushed_state():
    w = ArmWorker()
    w._push(_FakeState([0.5] * 7))
    s = w.snapshot()
    assert s.q == [0.5] * 7
    assert s.err == [1] * 7
    assert s.enabled is True


def test_push_does_not_require_on_state_callback():
    """`on_state=None` 也要能推（回调炸了不该影响 SDK 读线程）。"""
    w = ArmWorker(on_state=None)
    w._push(_FakeState([0.0] * 7))                      # 不该抛
    assert w.snapshot().mode_name == "MOVE_J"


def test_on_state_callback_exception_is_swallowed():
    """⚠ 回调在 **SDK 读线程**上跑 —— 它抛异常会污染 SDK 的读路径，必须吞掉。"""
    def boom(_s):
        raise ValueError("GUI 炸了")

    w = ArmWorker(on_state=boom)
    w._push(_FakeState([0.0] * 7))                      # 不该抛出去


# ────────────────────────── 角色校验 ──────────────────────────

def test_bad_role_rejected():
    """⚠ 角色是**遥操参数**（不是构造参数）⇒ 非法值在 `TeleopParams` 上就被拒。"""
    with pytest.raises(ValueError, match="role"):
        TeleopParams(role="observer")


def test_snapshot_role_is_undetermined_until_teleop_starts():
    """⚠ 角色是**遥操参数** ⇒ 连上但没启遥操时快照里它该是空的（不许替用户猜）。

    判别力：把 `Snapshot.role` 的默认值改回 `ROLE_MASTER`（或让 `snapshot()`
    回落到某个默认角色）时第一条断言必红。
    """
    w = ArmWorker()
    assert w.snapshot().role == "", "没启过遥操 ⇒ 角色未定"
    w._params = TeleopParams(ROLE_SLAVE)
    assert w.snapshot().role == ROLE_SLAVE


def test_master_pub_is_resident_but_rebuilt_when_params_change(monkeypatch):
    """⚠⚠ 主臂的发布端**建一次、活一个进程**（跨遥操会话不重建），
    但 `arm_id` / 端口是**遥操参数** ⇒ 它们变了必须重建。

    ⚠ 为什么必须常驻：每轮遥操拆了重建有**两个**实测过的后果 ——
    ① 停下时没人关 ⇒ 端口不释放 ⇒ 第二次「启动」报 `Address already in use`
    （表现：**只能启动一次**）；② 就算补上关闭，同端口拆了重建会让匹配
    **间歇性**建立不起来（实测 12 轮里 5 轮 `matching=False`）。

    判别力：改回"每轮新建"⇒ 前两条断言红；去掉 `_pub_meta` 比较 ⇒
    最后那条「换了 arm_id 还复用旧端点」的断言红。
    """
    made = []

    class _FakePub:
        def __init__(self, port, key):
            self.port, self.key, self.closed = port, key, False
            made.append(self)

        def put(self, _b):
            pass

        def close(self):
            self.closed = True

    monkeypatch.setattr(arm_worker.link, "Listener", _FakePub)
    w = ArmWorker()
    p1 = TeleopParams(ROLE_MASTER, arm_id="armA", jport=17447)
    a = w._ensure_pub(p1)
    assert w._ensure_pub(p1) is a, "同一组参数 ⇒ 必须复用（常驻）"
    assert len(made) == 1, f"不该反复建，实际建了 {len(made)} 个"

    p2 = TeleopParams(ROLE_MASTER, arm_id="armB", jport=17447)
    c = w._ensure_pub(p2)
    assert c is not a, "arm_id 变了 ⇒ 必须重建（否则发的还是旧 topic）"
    assert len(made) == 2
    assert a.closed, "旧端点必须关掉 —— 不关就是那个「端口不释放」的 bug"
    assert not c.closed

    w._close_pub()
    assert c.closed


class _TimeShim:
    """替掉 `arm_worker` 里的 `time` —— 让速率统计的 1 s 窗口可控。

    ⚠ 只 `monkeypatch.setattr(arm_worker, "time", shim)` 换掉**模块属性**，
    不去动真的 `time` 模块（那会波及全进程）。
    """

    def __init__(self, t0: float = 1000.0):
        self.t = t0

    def monotonic(self) -> float:
        return self.t

    def sleep(self, _s: float) -> None:
        pass


def test_rate_counters_actually_advance(monkeypatch):
    """⚠⚠ 两个速率必须**真的被算出来** —— 不是只在 dataclass 里声明。

    `Snapshot.state_hz` 从前**全仓无人赋值**（恒 0）⇒ 顶栏「控制频率」与主从链路卡
    那个 Hz 格子**永远显示 `—`**。用户 2026-09-29 报的就是这个。

    判别力：把 `_push` 里的 `_state_tick()` / 发布环里的 `_link_tick()` /
    `_on_wire` 里的 `_link_tick()` 任一去掉 ⇒ 对应断言必红。
    """
    shim = _TimeShim()
    monkeypatch.setattr(arm_worker, "time", shim)
    w = ArmWorker()

    for _ in range(50):                                  # 不足 1 s ⇒ 不出数
        w._state_tick()
    assert w.snapshot().state_hz == 0.0, "不到 1 s 的窗口不该出数"
    shim.t += 1.0
    w._state_tick()                                      # 第 51 帧触发结算
    assert 45 <= w.snapshot().state_hz <= 55, w.snapshot().state_hz

    w._link_reset()                                      # 链路速率是独立一组
    assert w.snapshot().link_hz == 0.0, "reset 后要归 0（停遥操就该显示 —）"
    for _ in range(20):
        w._link_tick()
    shim.t += 1.0
    w._link_tick()
    assert 18 <= w.snapshot().link_hz <= 22, w.snapshot().link_hz


def test_push_feeds_the_state_rate(monkeypatch):
    """⚠⚠ **接线**判据：`_push`（每来一帧状态）必须喂 `_state_tick`。

    判别力：去掉 `_push` 里那句 `self._state_tick()` 时本用例必红。
    ⚠ 只单测 `_state_tick` 本身**抓不到"根本没人调它"** —— 而"只声明不赋值"
    （这个字段从前的状态）恰恰就是"没人调"这一种，所以必须有这条。
    """
    shim = _TimeShim()
    monkeypatch.setattr(arm_worker, "time", shim)
    w = ArmWorker()
    for _ in range(50):
        w._push(_FakeState([0.0] * 7))
    assert w.snapshot().state_hz == 0.0, "不到 1 s 的窗口不该出数"
    shim.t += 1.0
    w._push(_FakeState([0.0] * 7))
    assert 45 <= w.snapshot().state_hz <= 55, \
        f"状态帧速率没被 `_push` 喂出来：{w.snapshot().state_hz}"


def test_master_loop_reports_a_publish_rate():
    """⚠ **接线**：主臂发布环每发一帧要喂 `_link_tick`，否则「发布 Hz」永远是 `—`。

    判别力：去掉 `_run_master` 里那句 `self._link_tick()` 时本用例必红。
    起**真 zenoh** Listener（不是 mock），发满一个 1 s 窗口再断言。
    """
    import socket

    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()

    class _Arm:
        def zero_g_start(self):
            pass

        def zero_g_stop(self):
            pass

        def get_state(self, refresh=False):
            class _S:
                q = [0.0] * 7
                dq = [0.0] * 7

            class _R:
                value = _S()

            return _R()

    w = ArmWorker()
    w._arm = _Arm()
    p = TeleopParams(ROLE_MASTER, jport=port)
    w._teleop_want = True
    th = threading.Thread(target=w._run_master, args=(p,), daemon=True)
    try:
        th.start()
        time.sleep(2.2)                       # 0.5 s engage + 一个完整 1 s 窗口
        w._teleop_want = False
        th.join(timeout=3.0)
        snap = w.snapshot()
        assert snap.frames_sent > 50, f"没发出去几帧：{snap.frames_sent}"
        assert snap.link_hz > 50, \
            f"主臂发布速率没被统计出来：{snap.link_hz}（去掉 _link_tick 就会是 0）"
    finally:
        w._teleop_want = False
        w._close_pub()


def _run_master_with_fake_arm(monkeypatch, st_factory, seconds=3.0):
    """跑一次 `_run_master`，返回 (耗时, 日志, 发出的帧数)。

    ⚠ 不建真 zenoh：把 `_ensure_pub` 换成一个只数数的假发布端。判据只关心
    "主臂失能/断流后循环会不会自己退出"，与链路无关。
    """
    logs = []

    class _Pub:
        def __init__(self):
            self.n = 0

        def put(self, _payload):
            self.n += 1

    class _Arm:
        def zero_g_start(self):
            pass

        def zero_g_stop(self):
            pass

        def get_state(self, refresh=False):
            class _R:
                value = st_factory()

            return _R()

    w = ArmWorker(on_log=logs.append)
    w._arm = _Arm()
    pub = _Pub()
    w._ensure_pub = lambda _p: pub
    p = TeleopParams(ROLE_MASTER, jport=17447)
    w._teleop_want = True
    t0 = time.monotonic()
    w._run_master(p)
    return time.monotonic() - t0, logs, pub.n


def test_master_loop_ends_when_master_arm_loses_enable():
    """⛔ 主臂失能/故障 ⇒ 主臂侧**立即结束遥操**（不再发帧）。

    场景：主臂掉电/急停后臂软掉。旧实现按缓存 q 继续发 —— 从臂跟着下坠的臂走，
    重新上电还会朝跳回去的 q 甩过去。停流后从臂由 200 ms 看门狗持位。
    判别力：去掉 `_run_master` 里的失能判据时，本用例会跑满 `_teleop_want`
    （不会自己退出）而失败。
    """
    ticks = {"n": 0}

    def _st():
        ticks["n"] += 1
        s = _FakeState([0.0] * 7)
        s.seq = ticks["n"]
        s.enabled = ticks["n"] <= 20          # 0.1 s 后"掉电"
        s.faulted = not s.enabled
        s.flags = 0x0240 if s.enabled else 0x0185
        s.joint_fault = 0 if s.enabled else 0x007F
        return s

    elapsed, logs, sent = _run_master_with_fake_arm(None, _st)
    assert elapsed < 2.5, f"主臂失能后循环没有退出：{elapsed:.2f}s"
    assert any("结束遥操" in m for m in logs), f"没有说明为什么结束：{logs}"
    assert sent >= 10, f"失能前应该已经发过帧：{sent}"


def test_master_loop_ends_when_status_stream_goes_silent():
    """主臂 MCU 也掉（USB 断）时 `seq` 不再前进 ⇒ 同样结束遥操。

    ⚠ 这条与 `enabled` 判据是**两回事**：链路死掉时缓存里的 `enabled`
    永远停在最后一次的真值（True），只看它永远判不出来。
    """
    def _st():
        s = _FakeState([0.0] * 7)
        s.seq = 7                              # 冻结
        return s

    elapsed, logs, _sent = _run_master_with_fake_arm(None, _st)
    assert elapsed < 2.5, f"状态流静默后循环没有退出：{elapsed:.2f}s"
    assert any("没有新帧" in m for m in logs), f"没有报告断流：{logs}"


def test_starting_teleop_requires_params():
    """启动遥操**必须**给 params（角色/arm_id/peer/端口都在里面）。

    ⚠ 必须**同步**抛 —— 若留到 worker 线程里再报，界面上表现为"点了按钮没反应"。
    """
    w = ArmWorker()
    with pytest.raises(ValueError, match="params"):
        w.set_teleop(True)


def test_teleop_topic_matches_server_convention():
    """topic 由**遥操参数**里的 `arm_id` 派生（不再挂在 worker 上）。"""
    assert TeleopParams(ROLE_MASTER, arm_id="armB").key == "litearm/v4/armB/teleop"


# ────────────── 调用顺序（真机踩过：movej 用了被改软的 mit_kp）──────────────

class _FakeEndpoint:
    def close(self):
        pass


class _FakeLimits:
    lo = [-1.0] * 7
    hi = [1.0] * 7


class _ArmedForSlave:
    move_timeout = 3.0

    def get_state(self, refresh=False):
        return type("M", (), {"value": type("S", (), {"q": [0.0] * 7})()})()

    def movej(self, q, speed=1.0):
        pass


def test_slave_aligns_then_follows_and_writes_no_firmware_parameters(monkeypatch):
    """⚠⚠ 从臂的顺序是 **对齐 → 跟随**，而且**不写任何固件参数**（用户裁决 2026-09-28）。

    写 `mit_kp` 那条路已经**证伪**：静态托得住（0.0004 rad），但**动态跟随会位置越界
    断轴 J2/J4**（真机 24 s 后 `FAULT FB_STALE POS_VIOL`）。
    ⇒ 用固件出厂刚度，抖动靠 kd 预算收紧限速去压。

    判别力：谁要是让从臂去**写固件参数**（`set_joint_param` / `set_ff_vec`），本用例会红
    —— `_NoWriteArm` 只放行**读**，一写就抛（`save_params` 更是直接 AssertionError）。
    """
    order = []
    seen = {}
    monkeypatch.setattr(servo, "align_to_master",
                        lambda *a, **k: (order.append("align"), [0.0] * 7)[1])

    def _fake_follow(*a, **k):
        order.append("follow")
        seen.update(k)
        return True

    monkeypatch.setattr(servo, "follow", _fake_follow)
    monkeypatch.setattr(arm_worker.link, "Connector",
                        lambda *a, **k: _FakeEndpoint())
    monkeypatch.setattr(arm_worker, "read_safe_limits", lambda arm, **k: _FakeLimits())

    class _NoWriteArm(_ArmedForSlave):
        """**读**参数是允许的（要算 kd 预算），**写**不许。"""

        def __init__(self):
            self.writes = []
            self.jp = [type("P", (), {"kp": 400.0, "kd": 5.0, "tau_max": 78.0,
                                       "q_min": -1.5, "q_max": 1.5})()
                   for _ in range(7)]

        @property
        def params(self):
            return self

        def all_joint_params(self):
            return list(self.jp)

        def get_ff_vec(self, item):
            return type("M", (), {"value": [6.0] * 7})()

        def set_joint_param(self, idx, kp, kd, tau_max):
            self.writes.append(("kp/kd", kp, kd))

        def set_ff_vec(self, item, values):
            assert item == 15, "kd_extra 在 0x26 向量表 item 15"
            self.writes.append(("kd_extra", list(values)))

        def save_params(self):
            raise AssertionError("⛔ 绝不许调 save_params（整扇区擦写、不可逆）")

        def __getattr__(self, name):
            raise AttributeError(name)

    w = ArmWorker()
    w._arm = _NoWriteArm()
    w._run_slave(TeleopParams(role=ROLE_SLAVE))
    assert order == ["align", "follow"], f"顺序必须是 对齐 → 跟随，实际 {order}"
    # ⛔ 速度上限必须**逐字是 litearm-server 的配置值**（用户裁决 2026-09-28，真机实测）。
    #   曾按 kd 预算收紧（`0.30·tau_max/kd_eff`）⇒ J3/J4 只剩 **11%**、腕部只剩 **9~15%**
    #   ⇒ 用户实测「跟随太慢，有明显的延迟」。
    #   判别力：谁把那个预算加回来，本用例会红。
    assert seen["speed_limit"] == list(servo.DEFAULT_SPEED_LIMIT), (
        f"speed_limit 必须是 server 原值 {servo.DEFAULT_SPEED_LIMIT}，"
        f"实际 {seen['speed_limit']}")



def test_align_move_timeout_is_thirty_seconds():
    """用户裁决 2026-09-28：**同步时间 3 s → 30 s**。

    ⚠ 它是 `Arm(move_timeout=)`，SDK 明说**连接后不许改** ⇒ 只能在构造时定。
    ⚠ 但 `movej` **到位就立刻返回**，所以这只是**上限**、不是固定等待 —— 到点就跟。
    """
    assert arm_worker.ALIGN_MOVE_TIMEOUT == 30.0
    assert ArmWorker().move_timeout == 30.0


# ────────────── 臂维护动作：使能 / 清错 / 复位 / 回零 ──────────────


class _MaintArm:
    """记下 SDK 调用的假臂。`enabled` 决定 `go_home` 的预检走哪条。"""

    def __init__(self, enabled=True, boom=None):
        self.enabled = enabled
        self.boom = boom                # 非 None ⇒ 那个方法抛异常
        self.calls = []

    def _hit(self, name):
        self.calls.append(name)
        if self.boom == name:
            raise RuntimeError(f"{name} 炸了")

    def get_state(self, refresh=False):
        return type("M", (), {"value": type("S", (), {"enabled": self.enabled})()})()

    def enable(self):
        self._hit("enable")

    def clear_faults(self):
        self._hit("clear_faults")

    def reset(self):
        self._hit("reset")

    def home(self):
        self._hit("home")
        return type("S", (), {"q": [0.0] * N_JOINTS})()


def _drain_once(w):
    """跑掉队列里排着的那一条命令。"""
    w._drain(0.0)


def test_maintenance_commands_reach_the_arm():
    """四个维护动作都要**真的落到 SDK 上**（名字别接错）。"""
    for method, sdk in (("enable_arm", "enable"), ("clear_faults", "clear_faults"),
                        ("reset_arm", "reset"), ("go_home", "home")):
        w = ArmWorker()
        w._arm = _MaintArm()
        getattr(w, method)()
        _drain_once(w)
        assert w._arm.calls == [sdk], f"{method} 应当调用 arm.{sdk}，实际 {w._arm.calls}"


def test_go_home_refuses_when_the_arm_is_not_enabled():
    """⚠⚠ **未使能时不许闷头发 home** —— 固件会拒，而用户只会看到"点了没反应"。

    判别力：把那个预检删掉，本用例会红（`home()` 会被调用）。
    """
    w = ArmWorker()
    w._arm = _MaintArm(enabled=False)
    w.go_home()
    _drain_once(w)
    assert w._arm.calls == [], f"未使能就不该调 arm.home()，实际 {w._arm.calls}"
    assert "未使能" in w._snap.error, f"要把原因说出来，实际 {w._snap.error!r}"


def test_maintenance_failure_is_visible_not_swallowed():
    """失败要**记进快照**（界面看得见），不能只写一行日志。"""
    w = ArmWorker()
    w._arm = _MaintArm(boom="clear_faults")
    w.clear_faults()
    _drain_once(w)
    assert "清错失败" in w._snap.error and "炸了" in w._snap.error, w._snap.error


def test_a_successful_maintenance_clears_the_previous_error():
    """一次成功的手动操作 ⇒ 清掉旧错误，否则界面会一直挂着上一次的错。"""
    w = ArmWorker()
    w._arm = _MaintArm()
    with w._lock:
        w._snap.error = "上一次的旧错"
    w.reset_arm()
    _drain_once(w)
    assert w._snap.error == "", f"成功之后该清空，实际 {w._snap.error!r}"


# ────────────── 锁存态恢复：使能 → ERR[10,6] → 清错→复位→使能 ──────────────

class _Rejected(Exception):
    """假 `CommandRejectedError` —— 判锁存只看 `.cmd` / `.code` 两个字段。"""

    def __init__(self, cmd=arm_worker.ENABLE_CMD, code=arm_worker.ENABLE_LATCHED_CODE):
        super().__init__(f"enable 被固件拒绝: ERR [{cmd:02x},{code}]")
        self.cmd = cmd
        self.code = code


class _LatchedArm:
    """真机复现的形态：锁存期间 `enable` 恒拒；`reset` 之后才放行。"""

    def __init__(self, latched=True, recovery_works=True):
        self.latched = latched
        self.recovery_works = recovery_works
        self.calls = []

    def get_state(self, refresh=False):
        st = type("S", (), {"enabled": not self.latched,
                            "faulted": self.latched,
                            "joint_fault": 1 if self.latched else 0,
                            "fault_detail": "锁存"})()
        return type("M", (), {"value": st})()

    def enable(self):
        self.calls.append("enable")
        if self.latched:
            raise _Rejected()

    def clear_faults(self):
        self.calls.append("clear_faults")

    def reset(self):
        self.calls.append("reset")
        if self.recovery_works:
            self.latched = False


def test_enable_recovers_from_a_firmware_latch():
    """⚠⚠ 断整机电源之后 `enable` 恒回 `ERR[10,6]` —— 必须自动走完恢复顺序。

    判别力：把 `_enable_with_recovery` 里的三步删掉，本用例会红。
    """
    logs = []
    w = ArmWorker(on_log=logs.append)
    w._arm = _LatchedArm()
    w.enable_arm()
    _drain_once(w)
    assert w._arm.calls == ["enable", "clear_faults", "reset", "enable"], w._arm.calls
    assert any("锁存已解除" in x for x in logs), logs
    assert w._snap.error == "", "自动恢复成功就不该留错误，实际 " + repr(w._snap.error)


def test_enable_does_not_mask_other_failures():
    """非锁存错误（总线/超时）**原样报错**，不许拿 清错/复位 瞎重试。"""
    w = ArmWorker()
    arm = _MaintArm(boom="enable")
    w._arm = arm
    w.enable_arm()
    _drain_once(w)
    assert w._arm.calls == ["enable"], w._arm.calls
    assert "使能失败" in w._snap.error and "炸了" in w._snap.error, w._snap.error


def test_connect_keeps_the_link_when_enable_fails():
    """⚠⚠ 连接时的自动使能失败**不得拆连接** —— 否则界面未连接 ⇒ 复位键灰掉 ⇒
    用户被卡死（2026-11-18 真机就是靠命令行才救回来的）。

    判别力：把这个异常放出去让 `_run` 走 `_teardown`，本用例会红。
    """
    logs = []
    w = ArmWorker(on_log=logs.append)
    arm = _LatchedArm(recovery_works=False)      # 连自动恢复都救不回来
    w._arm = arm
    w._enable_at_connect(arm)                    # 不许抛
    assert any("连接保持不变" in x for x in logs), logs
    assert w._snap.error == "", "别写 error —— 它会把安全确认勾选永久摘掉"


def test_connect_enable_success_under_the_normal_path():
    """没锁存的臂：只发一次 ENABLE，日志「已使能」。"""
    logs = []
    w = ArmWorker(on_log=logs.append)
    arm = _LatchedArm(latched=False)
    w._arm = arm
    w._enable_at_connect(arm)
    assert arm.calls == ["enable"], arm.calls
    assert any("已使能" in x for x in logs), logs


def test_maintenance_on_a_missing_arm_says_so():
    """没连接时要说一声，不是静默 no-op。

    ⚠ 这里只**记日志**、**不写 `snap.error`** —— 与 `set_payload` 同一惯例：
    "没连接"是用户操作问题，不是臂的故障；写进快照会让顶栏的"故障"格亮红。
    （界面上这条路径其实到不了：`_relock` 没连接时就把四键禁掉了，这里是兜底。）
    """
    logs = []
    w = ArmWorker(on_log=logs.append)
    w._arm = None
    w.enable_arm()
    _drain_once(w)
    assert any("未连接" in x for x in logs), logs
    assert w._snap.error == "", "不是臂的故障，别污染快照里的 error"
