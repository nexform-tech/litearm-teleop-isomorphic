"""主窗口：顶栏 + 常驻状态条 + 左侧导航 + 页面栈 + 底部日志（spec §8）。"""
from __future__ import annotations

import sys
import time

from PyQt5 import QtCore, QtWidgets

from ..arm_worker import ROLE_MASTER, ArmWorker, Snapshot
from ..settings import Settings, load_settings, save_settings
from .bridge import WorkerBridge
from .pages import JointsPage, LinkPage, TeleopPage
from .widgets import StatusStrip

__all__ = ["MainWindow", "run"]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.s = settings
        self.setWindowTitle("同构遥操 · 主/从 · zenoh 点对点")
        self.resize(1080, 720)

        self.worker: ArmWorker | None = None
        #: ⚠ **界面显示的那一份**快照 —— `_relock` 必须读它，不能另去问 worker，
        #: 否则"显示的是这帧、判定用的是另一帧"，而且没连接时判定拿不到东西。
        self._last: Snapshot | None = None
        self.bridge = WorkerBridge(self)
        self.bridge.state.connect(self._on_state)
        self.bridge.log.connect(self._log)

        # ── 顶栏 ──
        top = QtWidgets.QWidget()
        tl = QtWidgets.QHBoxLayout(top)
        tl.setContentsMargins(8, 6, 8, 6)
        self.lab_role = QtWidgets.QLabel("角色: —")
        self.lab_link = QtWidgets.QLabel("● 未连接")
        self.chk_ok = QtWidgets.QCheckBox("☑ 我已确认机械臂周围无障碍、急停可及")
        self.chk_ok.setChecked(False)
        self.chk_ok.toggled.connect(self._relock)
        self.btn_estop = QtWidgets.QPushButton("⛔ 急停")
        self.btn_estop.setStyleSheet("background:#b02020; color:white; font-weight:bold;")
        self.btn_estop.setToolTip(
            "⚠ 急停 = 全失能 ⇒ 臂**自由落体**。\n"
            "要「稳住」请用「停止遥操」（受控接管 movej）—— 那是两件事。")
        self.btn_estop.clicked.connect(self._estop)
        for w in (self.lab_role, self.lab_link, self.chk_ok):
            tl.addWidget(w)
        tl.addStretch(1)
        tl.addWidget(self.btn_estop)
        self.addToolBar(self._toolbarise(top))

        # ── 状态条 ──
        self.strip = StatusStrip()

        # ── 导航 + 页面栈 ──
        self.stack = QtWidgets.QStackedWidget()
        self.page_link = LinkPage(settings)
        self.page_joints = JointsPage()
        self.page_teleop = TeleopPage()
        for p in (self.page_link, self.page_joints, self.page_teleop):
            self.stack.addWidget(p)
        nav = QtWidgets.QListWidget()
        for name in ("链路", "关节", "遥操"):
            nav.addItem(name)
        nav.setFixedWidth(110)
        nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        nav.setCurrentRow(0)

        mid = QtWidgets.QWidget()
        ml = QtWidgets.QHBoxLayout(mid)
        ml.setContentsMargins(0, 0, 0, 0)
        ml.addWidget(nav)
        ml.addWidget(self.stack, 1)

        # ── 日志面板（常驻）──
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setFixedHeight(150)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter.addWidget(mid)
        splitter.addWidget(self.log)

        central = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(central)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.addWidget(self.strip)
        cl.addWidget(splitter, 1)
        self.setCentralWidget(central)

        # ── 接线 ──
        self.page_link.connect_clicked.connect(self._connect)
        self.page_link.teleop_toggled.connect(self._toggle_teleop)
        self.page_link.settings_changed.connect(self._save)
        self.page_teleop.payload_applied.connect(self._apply_payload)

        self._relock()
        self._log("就绪。⚠ 一个 CDC 口只允许一个进程。")

    @staticmethod
    def _toolbarise(w: QtWidgets.QWidget) -> QtWidgets.QToolBar:
        tb = QtWidgets.QToolBar()
        tb.setMovable(False)
        tb.addWidget(w)
        return tb

    # ────────────────────────── 安全闸门（§7.2）──────────────────────────
    def _relock(self, *_a) -> None:
        """勾选前运动类按钮一律锁定；断线 / 急停 / FAULT 自动重新锁定。

        ⚠ 判据取 **`self._last`（界面正在显示的那一帧）**，不另去问 worker ——
        否则就是两份真相，而且没连接时判定会拿不到东西（FAULT 就锁不住了）。
        """
        snap = self._last
        bad = bool(snap and (snap.faulted or snap.joint_fault or snap.error))
        armed = self.chk_ok.isChecked() and not bad
        self.page_link.btn_connect.setEnabled(self.worker is None)
        self.page_link.btn_teleop.setEnabled(armed and bool(snap and snap.connected))
        if bad:
            self.chk_ok.setChecked(False)

    # ────────────────────────── worker 生命周期 ──────────────────────────
    def _save(self) -> None:
        save_settings(self.s)

    def _connect(self) -> None:
        if self.worker is not None:
            self._log("已经连接（先关闭窗口再改角色）")
            return
        self.s.role = self.page_link.role()
        self._save()
        self.worker = ArmWorker(
            role=self.s.role, port=self.s.cdc_port or None, arm_id=self.s.arm_id,
            peer=self.s.peer, jport=self.s.jport,
            on_state=self.bridge.on_state, on_log=self.bridge.on_log)
        self.worker.start()
        self.lab_role.setText(f"角色: {'主臂' if self.s.role == ROLE_MASTER else '从臂'}")
        self._log(f"正在连接（角色={self.s.role}）…")

    def _toggle_teleop(self, on: bool) -> None:
        if self.worker is None:
            return
        self._log("启动遥操" if on else "停止遥操（受控接管 movej）")
        self.worker.set_teleop(on)

    def _apply_payload(self, mass, com) -> None:
        if self.worker is None:
            self._log("⚠ 未连接，载荷未设置")
            return
        self._log(f"设置末端载荷：{mass:.3f} kg，质心 {[round(v, 4) for v in com]} m …")
        self.worker.set_payload(mass, com)

    def _estop(self) -> None:
        """⛔ **不经 worker 队列**（§7.4）—— worker 可能正卡在收尾 `movej` 里。"""
        if self.worker is None:
            return
        self.chk_ok.setChecked(False)
        self.btn_estop.setEnabled(False)
        self.btn_estop.setText("急停已发出…")
        self._log("⛔ 急停已发出 —— 全失能 ⇒ 臂**自由落体**")
        self.worker.emergency_stop()
        QtCore.QTimer.singleShot(1500, self._estop_check)

    def _estop_check(self) -> None:
        out = self.worker.emergency_outcome() if self.worker else None
        if out is None:
            self._log("⚠ 急停尚无结果（最坏等 1.2 s 应答超时）")
            QtCore.QTimer.singleShot(1500, self._estop_check)
            return
        ok, err = out
        self._log("⛔ 急停已执行" if ok else f"⛔ 急停失败: {err}")
        self.btn_estop.setText("⛔ 急停")
        self.btn_estop.setEnabled(True)

    # ────────────────────────── 刷新 ──────────────────────────
    def _on_state(self, snap: Snapshot) -> None:
        # ⚠ 本槽在 **Qt 主线程**跑（信号跨线程排队），碰控件是安全的
        self._last = snap
        self.strip.update_from(snap)
        self.page_joints.apply(snap)
        self.page_teleop.apply(snap)
        self.page_link.apply(snap)
        self.lab_link.setText("● 已连接" if snap.connected else "● 未连接")
        self._relock()

    def _log(self, text: str) -> None:
        self.log.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {text}")

    def closeEvent(self, ev) -> None:
        if self.worker is not None:
            self._log("正在退出：停遥操 → 关 zenoh → 关臂 …")
            self.worker.shutdown()
        self._save()
        super().closeEvent(ev)


def run(argv=None, settings=None) -> int:
    argv = list(sys.argv if argv is None else argv)
    app = QtWidgets.QApplication(argv)
    w = MainWindow(settings or load_settings())
    w.show()
    return app.exec_()
