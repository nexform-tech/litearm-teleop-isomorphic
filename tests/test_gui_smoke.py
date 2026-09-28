"""GUI 冒烟测试（离屏，不需要显示器）。

两条最要紧的判据：
  1. **`err != 1` 才是异常** —— 用户裁决的口径。写反了（`err != 0`）健康的使能臂会七轴全红，
     本用例会红。
  2. 状态条与页面能吃下一份 `Snapshot` 而不炸。
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest                                                    # noqa: E402

pytest.importorskip("PyQt5")

from PyQt5 import QtWidgets                                      # noqa: E402

from liteteleop.arm_worker import ROLE_MASTER, ROLE_SLAVE, Snapshot   # noqa: E402
from liteteleop.gui.main_window import MainWindow                 # noqa: E402
from liteteleop.gui.widgets import BAD_BRUSH, ERR_ENABLED, OK_BRUSH  # noqa: E402
from liteteleop.settings import Settings                          # noqa: E402
from liteteleop.wire import N_JOINTS                              # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _snap(err=None, q=None, role=ROLE_MASTER):
    s = Snapshot(role=role, connected=True, firmware="Litearm1.8.0-7J",
                 mode_name="MOVE_J", enabled=True)
    s.q = list(q or [0.1] * N_JOINTS)
    s.dq = [0.0] * N_JOINTS
    s.tau = [1.0] * N_JOINTS
    s.t_mos = [30.0] * N_JOINTS
    s.t_coil = [28.0] * N_JOINTS
    s.err = list(err or [1] * N_JOINTS)
    return s


def test_window_constructs(qapp):
    w = MainWindow(Settings())
    assert w.stack.count() == 3
    assert w.lab_link.text() == "● 未连接"
    w.close()


def test_healthy_arm_shows_no_red_axis(qapp):
    """⚠ 核心判据：**健康的使能臂（七轴 err==1）不许有红轴**。

    这是把 `err != 1` 写成 `err != 0` 时唯一会红的用例。
    """
    w = MainWindow(Settings())
    w._on_state(_snap(err=[1] * N_JOINTS))
    for r in range(N_JOINTS):
        assert w.page_joints.table.item(r, 6).background().color() == OK_BRUSH, \
            f"J{r + 1} 被误判为故障 —— 判据写反了？（健康臂的 err 是 1）"
    w.close()


def test_disabled_axis_is_highlighted(qapp):
    """`err == 0`（失能）**必须**高亮 —— 否则"全绿"就没有判别力。"""
    w = MainWindow(Settings())
    w._on_state(_snap(err=[1, 0, 1, 1, 1, 1, 1]))
    assert w.page_joints.table.item(1, 6).background().color() == BAD_BRUSH
    assert w.page_joints.table.item(0, 6).background().color() == OK_BRUSH
    w.close()


def test_err_enabled_constant_is_one(qapp):
    assert ERR_ENABLED == 1, "口径：1 = 使能（用户裁决）"


def test_status_strip_master_vs_slave(qapp):
    w = MainWindow(Settings())
    w._on_state(_snap(role=ROLE_MASTER))
    assert "已发" in w.strip._labels["频率"].text()
    w._on_state(_snap(role=ROLE_SLAVE))
    assert "已收" in w.strip._labels["频率"].text()
    w.close()


def test_fault_locks_the_arm_checkbox(qapp):
    """FAULT ⇒ 自动重新锁定安全确认（§7.2）。"""
    w = MainWindow(Settings())
    w.chk_ok.setChecked(True)
    s = _snap()
    s.faulted = True
    w._on_state(s)
    assert w.chk_ok.isChecked() is False
    w.close()


def test_joint_table_tolerates_empty_snapshot(qapp):
    """刚启动、还没有任何状态帧时不许炸。"""
    w = MainWindow(Settings())
    w._on_state(Snapshot(role=ROLE_MASTER))
    assert w.page_joints.table.item(0, 1).text() == "—"
    w.close()


def test_teleop_button_emits_the_state_the_user_clicked(qapp):
    """⚠ 回归：`clicked` 在按钮状态**切换之后**才发出 ⇒ 必须直接发 `isChecked()`。

    判别力：若有人写成 `not isChecked()`，"启动"会发成"停止"——
    真机表现是"点了启动但臂不变软"，而**界面上看不出任何异常**。
    """
    w = MainWindow(Settings())
    got = []
    w.page_link.teleop_toggled.connect(got.append)
    btn = w.page_link.btn_teleop
    btn.setEnabled(True)

    btn.click()
    assert got == [True], f"第一次点击应发 True（启动），实际 {got}"
    btn.click()
    assert got == [True, False], f"第二次点击应发 False（停止），实际 {got}"
    w.close()


# ────────────────────────── 末端载荷（夹爪）──────────────────────────

def test_gripper_preset_fills_600g_and_3cm(qapp):
    w = MainWindow(Settings())
    w.page_teleop.btn_gripper.click()
    assert w.page_teleop.sp_mass.value() == 0.6
    assert [sp.value() for sp in w.page_teleop.sp_com] == [0.0, 0.0, 0.03]
    w.close()


def test_apply_payload_emits_what_is_in_the_boxes(qapp):
    w = MainWindow(Settings())
    got = []
    w.page_teleop.payload_applied.connect(lambda m, c: got.append((m, list(c))))
    w.page_teleop.btn_gripper.click()
    w.page_teleop.btn_payload.click()
    assert got == [(0.6, [0.0, 0.0, 0.03])], got
    w.close()


def test_payload_label_shows_the_readback_not_the_input(qapp):
    """⚠ 显示必须是**读回值** —— 固件静默钳制，显示输入值就是在撒谎。"""
    w = MainWindow(Settings())
    s = Snapshot(role=ROLE_MASTER)
    s.payload_mass = 0.0            # 固件把 -5 钳成了 0
    s.payload_com = [1.0, 0.0, 0.0]
    w._on_state(s)
    txt = w.page_teleop.lab_payload.text()
    assert "0.000 kg" in txt and "没设上" in txt, txt
    w.close()


# ────────────────────────── 夹爪遥操分区 ──────────────────────────

def test_grip_panel_defaults_to_disabled_and_independent(qapp):
    """⚠ 夹爪默认**不启用**（通道空），与臂是**两个独立控件**，端口也是分开的。

    判别力：
      · 若谁把默认填成 `can0`（或在显示时写回），「默认不启用」的安全默认被静默丢掉；
      · 若 `sp_gport` 忘了 `setValue`，QSpinBox 会被钳到 **1**（特权端口），
        主端 `link.Listener(1, …)` 根本绑不上 —— 本用例会红。
    """
    w = MainWindow(Settings())
    v = w.page_teleop.gripper_values()
    assert v["gcan"] == "", f"通道默认必须是空（不启用），实际 {v['gcan']!r}"
    assert w.page_teleop.chk_align.isChecked() is True
    assert w.page_teleop.btn_grip is not w.page_link.btn_teleop
    assert v["gport"] == 17448, f"夹爪端口必须独立于臂的 17447，实际 {v['gport']}"
    assert v["gport"] != 1, "⚠ 端口 1 是特权端口 —— QSpinBox 忘 setValue 就是这个症状"
    w.close()


def test_grip_button_is_clickable_from_construction(qapp):
    """⚠⚠ 死锁回归：夹爪按钮**不能**初始禁用。

    夹爪的 `GripWorker` **只能由点这个按钮创建**，而 `apply_grip` 只在
    `connected` 时才启用按钮 ⇒ 一旦初始禁用就**永远点不了**（只能重启应用）。
    判别力：在 `__init__` 里加回 `setEnabled(False)` 时本用例必红。
    """
    w = MainWindow(Settings())
    assert w.page_teleop.btn_grip.isEnabled() is True, \
        "夹爪按钮必须从构造起就可点，否则没有任何路径能创建 GripWorker"
    w.close()


def test_grip_button_emits_the_state_the_user_clicked(qapp):
    """⚠ 与臂那条同款的回归：`clicked` 在按钮状态**切换之后**才发出
    ⇒ 必须直接发 `isChecked()`，不能加 `not`。

    判别力：写成 `not isChecked()` 时"启动"会发成"停止" —— 表现为"点了没反应"。

    ⚠ 这里用**独立的 TeleopPage**（不过 MainWindow）：过了主窗口的话，
    `_toggle_grip` 会因「没填通道」把按钮弹回未勾选，第二次点击又从 False→True，
    于是测到 `[True, True]` —— 那测的是弹回行为，不是本用例要钉的信号语义。
    """
    from liteteleop.gui.pages import TeleopPage

    p = TeleopPage(Settings())
    got = []
    p.grip_toggled.connect(got.append)

    p.btn_grip.click()
    assert got == [True], f"第一次点击应发 True（启动），实际 {got}"
    p.btn_grip.click()
    assert got == [True, False], f"第二次点击应发 False（停止），实际 {got}"


def test_grip_click_without_channel_bounces_back_and_creates_nothing(qapp):
    """⚠ 没填通道 ⇒ **不建** `GripWorker`（不启用夹爪遥操的安全默认），
    并且按钮**弹回未勾选** —— 否则界面会显示"正在遥操"而实际什么都没发生。
    """
    w = MainWindow(Settings())

    w.page_teleop.btn_grip.click()
    assert w.grip is None, "没填通道就不该建 GripWorker"
    assert w.page_teleop.btn_grip.isChecked() is False, \
        "没建起来时必须弹回未勾选，不能停在「已启动」的样子"
    w.close()


def test_grip_snapshot_renders_master_slave_and_stale(qapp):
    """三种夹爪快照都要能吃下而不炸，且 stale 要**看得见**。"""
    from liteteleop.grip_worker import GripSnapshot

    w = MainWindow(Settings())

    m = GripSnapshot(role="master", connected=True, topic="litearm/v4/gripA/gripper_teleop",
                     frames_sent=10, openness=0.5, position_mm=60.0, matching=True)
    w._on_grip_state(m)
    assert "主端夹爪" in w.page_teleop.lab_grip.text()
    assert "已匹配订阅者" in w.page_teleop.lab_grip.text()

    m.matching = False
    w._on_grip_state(m)
    assert "未匹配" in w.page_teleop.lab_grip.text()

    s = GripSnapshot(role="slave", connected=True, frames_received=5, stale=False,
                     openness=0.5, position_mm=60.0, frame_age=0.01, loop_hz=50.0)
    w._on_grip_state(s)
    assert "跟随中" in w.page_teleop.lab_grip.text()

    s.stale = True
    w._on_grip_state(s)
    assert "持位" in w.page_teleop.lab_grip.text(), "watchdog 超时必须看得见"

    # ⚠ 刷新按钮**不该**回头再发一次 `grip_toggled` —— 那会让界面自激。
    #    （`apply_grip` 用的是 `setChecked()`，它只发 `toggled`，
    #     而我们接的是 `clicked`。这里就是钉住这一点。）
    got = []
    w.page_teleop.grip_toggled.connect(got.append)
    s.teleop_active = True
    w._on_grip_state(s)
    assert w.page_teleop.btn_grip.text() == "停止夹爪遥操"
    assert w.page_teleop.btn_grip.isChecked() is True
    s.teleop_active = False
    w._on_grip_state(s)
    assert w.page_teleop.btn_grip.text() == "启动夹爪遥操"
    assert got == [], f"刷新按钮不该发 toggled（自激），实际 {got}"
    w.close()


def test_grip_mismatch_and_error_are_visible(qapp):
    """主从标定不一致与夹爪报错都要在界面上看得见（不能静默）。"""
    from liteteleop.grip_worker import GripSnapshot

    w = MainWindow(Settings())
    g = GripSnapshot(role="slave", connected=True)
    g.mismatch = "主从夹爪标定可能不一致：主端 travel≈60.0 mm，本端 120.1 mm"
    w._on_grip_state(g)
    assert "不一致" in w.page_teleop.lab_grip_mismatch.text()

    g.error = "夹爪未标定"
    w._on_grip_state(g)
    assert "未标定" in w.page_teleop.lab_grip.text()

    g2 = GripSnapshot(role="slave", connected=True)
    w._on_grip_state(g2)
    assert w.page_teleop.lab_grip_mismatch.text() == "", "没告警时必须清空，不能留旧的"
    w.close()


def test_grip_sdk_unavailable_does_not_abort_the_app(qapp, monkeypatch):
    """⚠⚠ SDK 不可用时**绝不能让异常逃出 Qt 槽** —— PyQt5 会直接 abort 进程。

    本机实测：`ModuleNotFoundError` 逃出槽 ⇒ **exit 134、core dumped**。
    只在 `_ensure_grip_worker` 里兜 `SystemExit` 是不够的，因为
    ① `assert_sdk_pinned` 抛的 `SystemExit` **不是** `Exception` 的子类；
    ② 一台**没装 litegrip** 的机器上 `import litegrip` 抛 `ModuleNotFoundError`。
    后果是点一下夹爪按钮把**整个应用**带走，连正在跑的臂遥操一起
    （没有收尾 movej）—— 直接违反 spec §2 的故障隔离承诺。

    判别力：把 `except BaseException` 改回 `except SystemExit` 时，
    第一个 `assert` 会因为异常逃出来而报错。
    """
    import liteteleop.grip_worker as gwk

    w = MainWindow(Settings(gcan="can0"))
    try:
        # ① 没装 SDK
        monkeypatch.setattr(gwk, "pin_grip_sdk", lambda *a, **k: (_ for _ in ()).throw(
            ModuleNotFoundError("import of litegrip halted")))
        assert w._ensure_grip_worker() is None, "SDK 缺失应当只是停用夹爪遥操"
        assert w.grip is None

        # ② 走错仓（断言抛 SystemExit）
        monkeypatch.setattr(gwk, "pin_grip_sdk", lambda *a, **k: (_ for _ in ()).throw(
            SystemExit("⛔ litegrip 导入自别处")))
        assert w._ensure_grip_worker() is None
        assert w.grip is None
    finally:
        w.close()


def test_grip_force_and_fault_are_visible(qapp):
    """`force_n`、夹爪自身故障码、send_mit_frame 失败次数都要看得见（spec §9.3）。"""
    from liteteleop.grip_worker import GripSnapshot

    w = MainWindow(Settings())
    g = GripSnapshot(role="slave", connected=True, force_n=12.3)
    w._on_grip_state(g)
    assert "12.3" in w.page_teleop.lab_grip.text(), "力要显示"

    g.fault = "夹爪报告 error_code=11（故障：过温/过流等）—— 继续发帧持位"
    g.send_failed = 4
    w._on_grip_state(g)
    txt = w.page_teleop.lab_grip_mismatch.text()
    assert "过温" in txt, txt
    assert "4 次" in txt, txt
    w.close()


def test_can_channel_hint_does_not_crash(qapp):
    """`can_channels()` 在没有任何 can* 接口的机器上也要安全返回。"""
    from liteteleop.gui.pages import TeleopPage, can_channels

    assert isinstance(can_channels(), list)
    p = TeleopPage(Settings())
    assert isinstance(p.gripper_values()["gcan"], str)


def test_grip_rejected_frames_are_visible(qapp):
    """⚠ 丢弃非有限值帧（NaN/Inf）必须**看得见** —— 静默丢弃就是静默失败。"""
    from liteteleop.grip_worker import GripSnapshot

    w = MainWindow(Settings())
    g = GripSnapshot(role="slave", connected=True, rejected=7)
    g.mismatch = "主从标定可能不一致"
    w._on_grip_state(g)
    txt = w.page_teleop.lab_grip_mismatch.text()
    assert "7" in txt and "非有限值" in txt, txt
    assert "不一致" in txt, "两条告警要能同时显示，不能互相顶掉"
    w.close()
