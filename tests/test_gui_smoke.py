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
