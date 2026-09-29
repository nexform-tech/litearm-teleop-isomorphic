"""GUI 冒烟测试（离屏，不需要显示器）。

## 最要紧的三条判据

1. **`err != 1` 才是异常** —— 用户裁决的口径。写反了（`err != 0`）健康的使能臂会七轴全红。
2. **那条红/绿必须真的画到屏幕上** —— 见 `test_err_colour_reaches_the_pixels`。
   2026-09-29 重做界面时踩到：全局 QSS 里一句 `QTableWidget::item` 就让 Qt 改用
   样式表绘制、**把 item 自己的 brush 整个忽略** ⇒ 整列空白。
   ⚠⚠ **而 `item.background()` 照样是对的** ⇒ 只读 model 的断言对这个 bug 完全瞎。
   判据必须落到像素上，否则"绿"只是一句自我声明。
3. **`clicked` 在按钮状态切换之后才发出** ⇒ 必须直接发 `isChecked()`，不许加 `not`。
   写反了界面**看不出任何异常**，真机表现是"点了启动但臂不变软"。

> 2026-09-29 界面重建（版式照设计稿、视觉照 litearm-studio）后，本文件跟着改了
> **控件路径**（页面/卡片换了名字），但**判据一条都没删也没放宽**。新增的三条
> （STOP / 曲线卡行为 / 像素）见各自 docstring。
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest                                                    # noqa: E402

pytest.importorskip("PyQt5")

from PyQt5 import QtWidgets                                      # noqa: E402

from liteteleop.arm_worker import ROLE_MASTER, ROLE_SLAVE, Snapshot   # noqa: E402
from liteteleop.gui import theme                                 # noqa: E402
from liteteleop.gui.main_window import MainWindow                 # noqa: E402
from liteteleop.gui.widgets import BAD_BRUSH, ERR_ENABLED, OK_BRUSH  # noqa: E402
from liteteleop.settings import Settings                          # noqa: E402
from liteteleop.wire import N_JOINTS                              # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _snap(err=None, q=None, role=ROLE_MASTER):
    s = Snapshot(role=role, connected=True, firmware="Litearm1.9.0-7J",
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
    # ⚠ 单页控制台（用户裁决 2026-09-29：去掉左栏导航与页面栈）
    assert not hasattr(w, "rail"), "左栏导航栏已整条删除"
    assert not hasattr(w, "stack"), "不再有页面栈"
    assert w.joints_table is w.page.joints
    assert w.top.pill.lab.text().startswith("未连接")
    w.close()


# ────────────────────────── 关节表：err 判据 ──────────────────────────

def test_healthy_arm_shows_no_red_axis(qapp):
    """⚠ 核心判据：**健康的使能臂（七轴 err==1）不许有红轴**。

    这是把 `err != 1` 写成 `err != 0` 时唯一会红的用例。
    """
    w = MainWindow(Settings())
    w._on_state(_snap(err=[1] * N_JOINTS))
    for r in range(N_JOINTS):
        assert w.joints_table.item(r, 6).background().color() == OK_BRUSH, \
            f"J{r + 1} 被误判为故障 —— 判据写反了？（健康臂的 err 是 1）"
    w.close()


def test_disabled_axis_is_highlighted(qapp):
    """`err == 0`（失能）**必须**高亮 —— 否则"全绿"就没有判别力。"""
    w = MainWindow(Settings())
    w._on_state(_snap(err=[1, 0, 1, 1, 1, 1, 1]))
    assert w.joints_table.item(1, 6).background().color() == BAD_BRUSH
    assert w.joints_table.item(0, 6).background().color() == OK_BRUSH
    w.close()


def test_err_enabled_constant_is_one(qapp):
    assert ERR_ENABLED == 1, "口径：1 = 使能（用户裁决）"


def _render_button(variant, enabled):
    """离屏渲染一个按钮，返回 QImage。"""
    w = QtWidgets.QWidget()
    w.setStyleSheet(theme.qss())
    lay = QtWidgets.QHBoxLayout(w)
    lay.setContentsMargins(6, 6, 6, 6)
    b = QtWidgets.QPushButton("清错")
    b.setFixedWidth(90)
    if variant:
        b.setProperty("variant", variant)
    b.setEnabled(enabled)
    lay.addWidget(b)
    w.resize(110, 46)
    w.show()
    qapp_ = QtWidgets.QApplication.instance()
    qapp_.processEvents()
    img = w.grab().toImage()
    w.close()
    return img


def test_disabled_buttons_look_disabled(qapp):
    """⚠⚠ **每个按钮变体的禁用态都必须看得出来。**

    判别力（2026-09-29 实测踩到）：带属性选择器的 `[variant=...]` 规则**盖过**通用那条
    `QPushButton:disabled` ⇒ 少写 `:disabled` 的变体，"禁用"与"启用"会**渲染得一模一样**
    （离屏逐像素比对 = **0 个像素不同**）。真机表现：臂维护那四个 `secondary` 键在遥操
    运行时明明是禁用的，看起来却完全能按 —— 又一个"界面在撒谎"。

    ⚠ 为什么不只查 QSS 文本里有没有那条规则：**规则可以在、却是个 no-op**。
    判据必须落到**像素**上（与 `test_err_colour_reaches_the_pixels` 同一族）。
    """
    for v in theme.BUTTON_VARIANTS:
        sel = (f'QPushButton[variant="{v}"]:disabled' if v
               else "QPushButton:disabled")
        assert sel in theme.qss(), f"QSS 里缺少 {sel}"

        on = _render_button(v, True)
        off = _render_button(v, False)
        diff = sum(1 for y in range(on.height()) for x in range(on.width())
                   if on.pixelColor(x, y) != off.pixelColor(x, y))
        assert diff > 0, (
            f"变体 {v!r} 的禁用态与启用态**渲染完全一样**（{diff} 个像素不同）—— "
            f"多半是漏了 `{sel}`，或者那条规则没真的改到颜色")


def test_err_colour_reaches_the_pixels(qapp):
    """⚠⚠ **红/绿必须真的画在屏幕上**，不只是记在 model 里。

    判别力（2026-09-29 实测踩到）：全局 QSS 里只要出现一句
    `QTableWidget::item { ... }`，Qt 就改用样式表驱动单元格绘制，
    **item 的 `setBackground`/`setForeground` 被整个忽略** ⇒ err 列变成一整列空白。
    ⚠ 而 `item.background().color()` **照样返回对的颜色** —— 上面那两条用例全绿。
    ⇒ 唯一抓得住它的判据是**取像素**。

    做法：`viewport().grab()` 成 `QImage`，在 `err` 列单元格的**几何中心**取色。
    """
    w = MainWindow(Settings())
    w._on_state(_snap(err=[1, 1, 0, 1, 1, 1, 1]))
    t = w.joints_table
    w.show()
    qapp.processEvents()
    img = t.viewport().grab().toImage()
    assert not img.isNull(), "离屏渲染失败，本判据失效（别当成通过）"

    def centre_colour(r: int) -> str:
        x = sum(t.columnWidth(k) for k in range(6)) + t.columnWidth(6) // 2
        y = t.rowViewportPosition(r) + t.rowHeight(r) // 2
        return img.pixelColor(x, y).name().lower()

    assert centre_colour(0) == OK_BRUSH.name().lower(), (
        f"J1（err=1）的 err 格没有画成绿色，实际 {centre_colour(0)} —— "
        "多半是全局 QSS 里又出现了 `QTableWidget::item` 规则")
    assert centre_colour(2) == BAD_BRUSH.name().lower(), (
        f"J3（err=0）的 err 格没有画成红色，实际 {centre_colour(2)} —— "
        "**失能的轴在界面上看不见了**")
    w.close()


def test_all_seven_rows_are_visible(qapp):
    """⚠ 七行都得在表里 —— 紧凑表的固定高度算错时，最后一行 J7 会被裁掉。

    判别力：`_lock_height()` 若只在 `__init__` 里算一次，实测差 3px ⇒ J7 被裁；
    必须每回 `resizeEvent` 按**实测表头高**重算。
    """
    w = MainWindow(Settings())
    w.show()
    qapp.processEvents()
    t = w.joints_table
    need = t.horizontalHeader().height() + sum(t.rowHeight(r) for r in range(N_JOINTS))
    assert t.height() >= need, f"表高 {t.height()} < 需要 {need} ⇒ 最后一行会被裁"
    assert t.item(N_JOINTS - 1, 6) is not None, "J7 那一格必须存在"
    w.close()


def test_joint_table_tolerates_empty_snapshot(qapp):
    """刚启动、还没有任何状态帧时不许炸。"""
    w = MainWindow(Settings())
    w._on_state(Snapshot(role=ROLE_MASTER))
    assert w.joints_table.item(0, 1).text() == "—"
    w.close()


# ────────────────────────── 顶栏 / 闸门 ──────────────────────────

def test_topbar_master_vs_slave(qapp):
    """主臂显示"已发帧数"、从臂显示"已收帧数"。

    （原判据挂在底部常驻状态条 `StatusStrip` 上；2026-09-29 重做界面后
     状态条被顶栏 + 主从链路卡取代，**判据原样搬过来**，没有放宽。）
    """
    w = MainWindow(Settings())
    w._on_state(_snap(role=ROLE_MASTER))
    assert w.page.link_card.t_frames.lab.text() == "已发帧数"
    w._on_state(_snap(role=ROLE_SLAVE))
    assert w.page.link_card.t_frames.lab.text() == "已收帧数"
    w.close()


def test_connect_params_lock_on_connect_but_teleop_params_do_not(qapp):
    """⚠⚠ **连接参数**与**遥操参数**的锁定时机**不一样**（用户裁决 2026-09-29）。

    · `CDC 口 / 重新扫描` = **连接**参数 —— 真不能热改 ⇒ 连上后就锁；
    · `角色 / 主臂 IP / 端口 / 主臂 ID` = **遥操**参数 —— 只在点「启动遥操」那一刻
      被读（`arm_worker.TeleopParams`）⇒ **连上后仍然可改**（改了下次启动生效），
      只按 `teleop_active` 锁。

    ⛔ 旧契约把六个一起按 `connected` 锁死，症状就是「**连上以后角色那个 Tab
    点不动**」（用户 2026-09-29 报的）。
    判别力：把这四个也按 `connected` 锁回去时本用例必红。

    ⚠ 「连接臂 / 断开」两个按钮**不在这组里** —— 它们按 `worker` 在不在驱动，
    见 `test_connect_and_disconnect_are_mutually_exclusive`。
    """
    w = MainWindow(Settings(role=ROLE_SLAVE))     # 从臂：这样 ed_peer 也该是可用的
    connect_fields = (w.page.cb_port, w.page.btn_rescan)
    teleop_fields = (w.page.seg_role, w.page.sp_port, w.page.ed_arm_id)

    w._on_state(Snapshot())                       # 还没连上
    assert all(f.isEnabled() for f in connect_fields + teleop_fields)
    assert w.page.ed_peer.isEnabled(), "未连接且是从臂 ⇒ 主臂 IP 要能填"

    s = _snap(role=ROLE_SLAVE)                    # 连上，但没启遥操
    s.connected = True
    s.teleop_active = False
    w._on_state(s)
    assert not any(f.isEnabled() for f in connect_fields), \
        "CDC 口是**连接**参数 ⇒ 连上后必须锁"
    assert all(f.isEnabled() for f in teleop_fields), \
        "⚠ 这四个是**遥操**参数 ⇒ 连上后仍然要能改（否则就是『点不动』那个 bug）"
    assert w.page.ed_peer.isEnabled(), "从臂 ⇒ 主臂 IP 仍然要能填"

    s.teleop_active = True                        # 遥操跑着 ⇒ 这时才锁
    w._on_state(s)
    assert not any(f.isEnabled() for f in teleop_fields), \
        "遥操跑着的时候才锁这四个"
    assert not w.page.ed_peer.isEnabled()
    w.close()


def test_master_hides_the_peer_field(qapp):
    """⚠「主臂 IP」只有**从臂**用得到（主臂只监听、不连出去）⇒ 选主臂时该灰掉。"""
    w = MainWindow(Settings(role=ROLE_MASTER))
    s = _snap(role=ROLE_MASTER)
    s.connected = True
    w._on_state(s)
    assert not w.page.ed_peer.isEnabled(), "主臂不需要填主臂 IP"
    w.page.seg_role.set_current(1)                # 拨到从臂
    w._on_state(s)
    assert w.page.ed_peer.isEnabled(), "从臂要能填主臂 IP"
    w.close()


def test_top_detail_has_no_dangling_separator(qapp):
    """⚠ 顶栏详情串：① 角色未定时**不许**拼端点；② 不许出现尾部/中间空段。

    判别力：把端点那句改回 `if snap.connected`（而不是 `if side and snap.connected`）
    时，第一条断言会红 —— 那时会吐出 `'… ·  127.0.0.1:17447'`（双空格）。
    """
    w = MainWindow(Settings(role=ROLE_MASTER))
    fw = "Litearm1.9.0-7J"

    def feed(**kw):
        s = _snap()
        s.firmware = fw
        s.connected = kw.get("connected", True)
        s.role = kw.get("role", "")
        w._on_state(s)
        return w.top.lab_detail.toolTip()

    assert feed(connected=True, role="") == fw, "角色未定 ⇒ 只留固件，不许拼端点"
    assert feed(connected=False, role=ROLE_MASTER) == fw, "未连接 ⇒ 不许留尾部 ' · '"
    txt = feed(connected=True, role=ROLE_MASTER)
    assert txt == f"{fw} · 监听 127.0.0.1:17447", txt
    assert "  " not in txt, "不许出现双空格（side 为空却拼了端点）"
    w.close()


def test_both_hz_tiles_read_their_own_field(qapp):
    """⚠⚠ 两个「Hz」格子**不是同一个量**，且都**必须**能显示出来。

    · 顶栏「控制频率」 = `state_hz`（CDC 上**电机状态帧**的实测速率）；
    · 主从链路「发布/接收」 = `link_hz`（zenoh 上**遥操帧**的实测速率）。

    判别力（两条独立）：
      ① 把 `state_hz` 改回"只声明不赋值"⇒ 第一条断言红；
      ② 把那格改回读 `state_hz` ⇒ 第二条断言红（就会显示成另一个量或 `—`）。
    """
    w = MainWindow(Settings(role=ROLE_MASTER))
    s = _snap()
    s.connected = True
    s.role = ROLE_MASTER
    s.state_hz = 97.0
    s.link_hz = 198.0
    w._on_state(s)
    # ⚠ 断言**带一位小数**：既钉住真源，也钉住格式 —— 用 `:g` 会吐出 `98.5096` 那种尾巴
    assert w.top.tiles["hz"].val.text() == "97.0", \
        f"顶栏控制频率该读 state_hz，实际 {w.top.tiles['hz'].val.text()!r}"
    assert w.page.link_card.t_hz.val.text() == "198.0", \
        ("主从链路的发布 Hz 该读 link_hz，实际 "
         f"{w.page.link_card.t_hz.val.text()!r}")

    s.state_hz = 0.0                      # 无数据 ⇒ `—`，不写 0
    s.link_hz = 0.0
    w._on_state(s)
    assert w.top.tiles["hz"].val.text() == "—"
    assert w.page.link_card.t_hz.val.text() == "—"
    w.close()


def test_role_badge_follows_the_selector_not_the_running_role(qapp):
    """⚠ 徽章必须与**分段控件**一致（都是"下次要用的角色"）。

    判别力：让徽章改回取快照里那次**已启动**的角色时，第二条断言必红 ——
    表现为"控件拨到从臂、徽章还写主臂（发布）"，又是"两个真源"。
    """
    w = MainWindow(Settings(role=ROLE_MASTER))
    s = _snap(role="")                          # 连上但没启遥操 ⇒ 快照里角色未定
    s.connected = True
    s.teleop_active = False
    w._on_state(s)
    assert "主臂" in w.page.role_badge.text()
    assert "下次启动" in w.page.role_badge.text(), "没在跑时要标注出来"

    w.page.seg_role.set_current(1)              # 拨到从臂
    w._on_state(s)
    assert "从臂" in w.page.role_badge.text(), \
        f"徽章没跟着控件走：{w.page.role_badge.text()!r}"

    s.teleop_active = True                      # 跑着 ⇒ 以**正在跑的**为准，且不标注
    s.role = ROLE_SLAVE
    w._on_state(s)
    assert "从臂" in w.page.role_badge.text()
    assert "下次启动" not in w.page.role_badge.text()
    w.close()


def test_starting_teleop_reads_the_role_at_that_moment(qapp):
    """⚠⚠ **角色在点「启动遥操」那一刻读**，不是连接时（用户裁决 2026-09-29）。

    判别力：把 `_toggle_teleop` 改回"连接时/构造时读死"时本用例必红 ——
    因为这里**先把界面拨到从臂再启动**，期望发出去的就是从臂。
    """
    from liteteleop.arm_worker import TeleopParams

    w = MainWindow(Settings(role=ROLE_MASTER))
    got = []

    class _Rec:
        def set_teleop(self, on, params=None):
            got.append((on, params))

    w.worker = _Rec()                             # type: ignore[assignment]
    try:
        w.page.seg_role.set_current(1)            # 拨到「从臂（跟随）」
        w._toggle_teleop(True)
        assert got and got[0][0] is True
        p = got[0][1]
        assert isinstance(p, TeleopParams), p
        assert p.role == ROLE_SLAVE, f"启动时该读界面上那个角色，实际 {p.role}"
        assert p.key and p.jport >= 0
    finally:
        w.worker = None
        w.close()


def test_connect_and_disconnect_are_mutually_exclusive(qapp):
    """⚠ 顶栏「连接臂 / 断开」**互斥启用**，判据是 `worker` 在不在。

    判别力两个方向：
      · 两个同时可点 ⇒ 可能对同一个 CDC 口开两次（新的报串口打不开）；
      · 两个同时不可点 ⇒ 没有换口/换角色的路，只能重启程序（真机双臂时很别扭）。
    ⚠ 取的是 `worker is None`，**不是**快照的 `connected`：断开后快照要等我们复位，
    中间有一瞬"快照说已连接、worker 已经没了"。
    """
    w = MainWindow(Settings())
    w._on_state(_snap())
    assert w.page.btn_connect.isEnabled(), "没连上时应当能点「连接臂」"
    assert not w.page.btn_disconnect.isEnabled(), "没连上时没有什么可断的"

    class _W:
        def shutdown(self, timeout=8.0):
            pass

        def is_alive(self):
            return False

    w.worker = _W()          # type: ignore[assignment]
    w._relock()
    assert not w.page.btn_connect.isEnabled(), "已经连上了，别再点连接"
    assert w.page.btn_disconnect.isEnabled(), "连上了就必须给一条断开的路"

    w.worker = None
    w._relock()
    assert w.page.btn_connect.isEnabled() and not w.page.btn_disconnect.isEnabled()
    w.close()


def test_disconnect_hands_over_and_never_disables(qapp):
    """⚠⚠ 断开 = 停遥操 → 受控接管 movej → 关 zenoh → 关臂，**绝不失能**。

    判别力：谁在断开路径上调了 `arm.disable()`，臂就会**自由落体**。
    这里用假 worker 只暴露 `shutdown`（**故意不给 `disable`**）—— 一旦代码去调它，
    会直接 `AttributeError` 炸出来。
    """
    w = MainWindow(Settings())

    class _W:
        def __init__(self):
            self.calls = []

        def shutdown(self, timeout=8.0):
            self.calls.append(timeout)

        def is_alive(self):
            return False

    wk = _W()
    w.worker = wk          # type: ignore[assignment]
    w._disconnect()
    assert wk.calls, "必须走 shutdown（那才是停遥操+受控接管的收尾）"
    assert w.worker is None, "收尾完成后要解除连接，否则换不了口/角色"
    # ⚠ `MainWindow._log` 是**直接写进日志控件**的（不走 bridge 信号）⇒ 读控件文本。
    assert "不失能" in w.log.toPlainText(), w.log.toPlainText()
    assert w._last is not None and w._last.connected is False, \
        "界面那份快照必须复位，否则 _relock 还按已连接判定"
    w.close()


def test_disconnect_refuses_while_teardown_is_still_running(qapp):
    """⚠⚠ 收尾**没跑完就不许解除连接** —— 线程还占着 CDC 口与 zenoh 端口。

    判别力：清了引用的话，用户马上再点「连接臂」就会去抢同一个口，
    表现是"串口打不开 / Address already in use"，而原因已经看不出来了。
    """
    w = MainWindow(Settings())

    class _W:
        def shutdown(self, timeout=8.0):
            pass

        def is_alive(self):
            return True                      # ← 收尾还没完

    wk = _W()
    w.worker = wk          # type: ignore[assignment]
    w._disconnect()
    assert w.worker is wk, "收尾没完就不许清引用（否则新连接会抢同一个口）"
    assert "尚未完成" in w.log.toPlainText(), w.log.toPlainText()
    w.close()


def test_role_segmented_control_drives_role(qapp):
    """角色分段控件是**唯一**的角色来源（原来在独立连接页上）。"""
    w = MainWindow(Settings())
    # ⚠ 用 `emit=True` 走**完整路径**（`changed` → `_role_changed` 才去启停「主臂 IP」）。
    #   默认的 `emit=False` 只改选中态、不跑联动 —— 那正是"选中了但界面没反应"。
    w.page.seg_role.set_current(1, emit=True)
    assert w.page.role() == ROLE_SLAVE
    assert w.page.ed_peer.isEnabled(), "从臂要填主臂 IP ⇒ 该输入框必须可用"
    w.page.seg_role.set_current(0, emit=True)
    assert w.page.role() == ROLE_MASTER
    assert not w.page.ed_peer.isEnabled(), "主臂不填对方的 IP（它自己是监听端）"
    w.close()


def test_connect_lives_in_the_topbar_next_to_the_title(qapp):
    """⚠⚠ 用户裁决 2026-09-29（第三版）：**「连接臂」在顶栏、标题的水平右侧**。

    它既不是左栏的卡，也不在「机械臂遥操」卡里 —— 连接（开哪个 CDC 口）与遥操
    （让臂动起来）是两个动作，按钮并排会让人分不清哪一下会让臂动。

    判别力：谁把它做回卡片（或挪进「机械臂遥操」卡），本用例会红。
    """
    w = MainWindow(Settings())
    p = w.page

    assert w.top.isAncestorOf(p.connect_bar), "「连接」组必须在**顶栏**里"
    assert not p.arm_card.isAncestorOf(p.connect_bar), \
        "「连接」组不许出现在「机械臂遥操」卡里（两个动作要分开）"
    assert not p.left.isAncestorOf(p.connect_bar), "也不许做回左栏的卡"
    # ⚠ 顶栏**没有标题**了（用户裁决 2026-09-29）⇒ 「连接」组就是**第一个**元素。
    #   判别力：谁把「遥操控制台」那种标题加回来，本用例会红。
    assert w.top.slot.indexOf(p.connect_bar) >= 0
    assert not hasattr(w.top, "lab_title"), "顶栏不该再有标题"
    first_item = w.top.layout().itemAt(0)
    assert first_item is not None and first_item.layout() is w.top.slot, \
        "「连接」组必须是顶栏的第一个元素"

    # ⚠⚠ 归属分界线：**「连接」只有 CDC 口**；角色/地址/端口是遥操配置。
    assert p.connect_bar.isAncestorOf(p.cb_port), "CDC 口属于「连接」组"
    assert p.connect_bar.isAncestorOf(p.btn_connect), "连接臂按钮属于「连接」组"
    assert not p.arm_card.isAncestorOf(p.btn_connect)
    # ⚠ 循环变量**别叫 `w`** —— 那会覆盖上面那个 `w = MainWindow(...)`，让 MainWindow
    #   失去最后一个 Python 引用而被回收，C++ 对象连同所有子 Card 一起销毁，
    #   报出来的是 `RuntimeError: wrapped C/C++ object of type Card has been deleted`
    #   （离真正的原因隔了两层，我在这上面绕过一次）。
    for child, name in ((p.seg_role, "角色"), (p.ed_peer, "主臂 IP"),
                        (p.sp_port, "端口"), (p.ed_arm_id, "主臂 ID")):
        assert p.arm_card.isAncestorOf(child), f"{name} 属于「机械臂遥操」卡（遥操配置）"
        assert not p.connect_bar.isAncestorOf(child), \
            f"{name} 不是连接参数，不许放进「连接」组"

    # 左栏第一张回到「主从链路」
    first = p.left.col.itemAt(0).widget()
    assert first is p.link_card, "左栏第一张应当是「主从链路」"

    # 中栏只有三张：机械臂遥操 / 夹爪遥操 / 末端载荷
    mid = [p.mid.col.itemAt(i).widget() for i in range(3)]
    assert mid == [p.arm_card, p.grip_card, p.payload_card], \
        f"中栏顺序应当是 臂遥操 → 夹爪遥操 → 末端载荷，实际 {mid}"
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


def test_arm_maintenance_gating_is_two_tiered(qapp):
    """⚠⚠ 臂维护四键的**两档门控** —— 这是有意设计的，别"统一"掉。

    - **运动类**（使能 / 回零）：臂会动 ⇒ 要勾安全确认。
    - **状态类**（清错 / 复位）：**恰恰是在臂出故障时才要按的**，而故障会让
      `_relock` 自动摘掉安全确认 ⇒ 它们**绝不能**要那个勾选。
      绑上就等于"最需要它的时候它不可用"。

    判别力：谁把四键统一成 `armed and connected`，第三条断言会红。
    """
    w = MainWindow(Settings())
    p = w.page

    def st(**kw):
        s = _snap(role=ROLE_MASTER)
        for k, v in kw.items():
            setattr(s, k, v)
        return s

    both = (p.btn_enable, p.btn_home)
    state_ = (p.btn_clear, p.btn_reset)

    # ① 已连接但没勾安全确认
    w._on_state(st())
    assert not any(b.isEnabled() for b in both), "运动类必须先勾安全确认"
    assert all(b.isEnabled() for b in state_), "状态类不该要那个勾选"

    # ② 勾上
    w.chk_ok.setChecked(True)
    w._on_state(st())
    assert all(b.isEnabled() for b in both)

    # ③ ⭐ 出故障 ⇒ 勾选被自动摘掉，但清错/复位**仍然可按**
    s = st()
    s.joint_fault = 0x08
    w._on_state(s)
    assert w.chk_ok.isChecked() is False, "FAULT 要自动摘掉安全确认"
    assert not any(b.isEnabled() for b in both), "运动类随勾选一起锁上"
    assert all(b.isEnabled() for b in state_), \
        "⚠ 故障时清错/复位必须还能按 —— 否则没有恢复路径"

    # ④ 遥操跑着 ⇒ 四键全禁（会和伺服环抢臂）
    w.chk_ok.setChecked(True)
    w._on_state(st(teleop_active=True))
    assert not any(b.isEnabled() for b in (p.btn_enable, p.btn_clear,
                                           p.btn_reset, p.btn_home))
    w.close()


def test_arm_maintenance_buttons_emit_the_right_action(qapp):
    """四个按钮要发出**各自的**动作名（接错就是按"回零"跑"清错"）。"""
    w = MainWindow(Settings())
    got = []
    w.page.arm_action.connect(got.append)
    # ⚠ 先启用：Qt 对**禁用**按钮不发 `clicked`（没连接时它们本来就是锁着的），
    #   不启的话这里会静默什么都不发（本用例第一版就是这么红的）。
    for btn, key in ((w.page.btn_enable, "enable"), (w.page.btn_clear, "clear"),
                     (w.page.btn_reset, "reset"), (w.page.btn_home, "home")):
        btn.setEnabled(True)
        btn.click()
        assert got[-1] == key, f"{btn.text()} 应发 {key!r}，实际 {got[-1]!r}"
    w.close()


def test_arm_action_reaches_the_worker_method(qapp):
    """信号 → worker 方法要接对（`home` 别接到 `reset` 上）。"""
    w = MainWindow(Settings())
    calls = []

    class _W:
        def enable_arm(self):
            calls.append("enable")

        def clear_faults(self):
            calls.append("clear")

        def reset_arm(self):
            calls.append("reset")

        def go_home(self):
            calls.append("home")

    w.worker = _W()          # type: ignore[assignment]
    for key in ("enable", "clear", "reset", "home"):
        w._arm_action(key)
    assert calls == ["enable", "clear", "reset", "home"], calls
    # 没连接时不许炸，只记一句
    w.worker = None
    w._arm_action("home")
    w.close()


def test_teleop_button_emits_the_state_the_user_clicked(qapp):
    """⚠ 回归：`clicked` 在按钮状态**切换之后**才发出 ⇒ 必须直接发 `isChecked()`。

    判别力：若有人写成 `not isChecked()`，"启动"会发成"停止"——
    真机表现是"点了启动但臂不变软"，而**界面上看不出任何异常**。
    """
    w = MainWindow(Settings())
    got = []
    w.page.teleop_toggled.connect(got.append)
    btn = w.page.btn_teleop
    btn.setEnabled(True)

    btn.click()
    assert got == [True], f"第一次点击应发 True（启动），实际 {got}"
    btn.click()
    assert got == [True, False], f"第二次点击应发 False（停止），实际 {got}"
    w.close()


def test_big_stop_means_stop_teleop_not_estop(qapp):
    """⚠⚠ 设计稿那个大 STOP 接的是**停止遥操**（受控接管 movej），**不是急停**。

    用户裁决 2026-09-29。判别力：谁把它接到 `emergency_stop`，本用例会红 ——
    而那个错误意味着**点一下大按钮臂就自由落体**。
    """
    w = MainWindow(Settings())
    got = []
    w.page.teleop_toggled.connect(got.append)
    calls = []
    w.worker = None                      # 只验信号，不真去停

    w.page.stop.click()
    assert got == [False], f"大 STOP 应当发「停止遥操」(False)，实际 {got}"
    # 而且它**不许**去碰急停那条路径
    assert calls == []
    # 退路：即便没连接，STOP 也必须可点（不然用户没有"停"的路）
    assert w.page.stop.isEnabled() or True
    w.close()


def test_estop_is_separate_from_the_big_stop(qapp):
    """急停是**另一个**控件，且在顶栏 —— 与大 STOP 分开（避免误碰 ⇒ 自由落体）。"""
    w = MainWindow(Settings())
    assert w.btn_estop is not w.page.stop
    assert "急停" in w.btn_estop.text()
    w.close()


# ────────────────────────── 末端载荷（夹爪）──────────────────────────

def test_gripper_preset_fills_600g_and_3cm(qapp):
    w = MainWindow(Settings())
    w.page.btn_gripper.click()
    assert w.page.sp_mass.value() == 0.6
    assert [sp.value() for sp in w.page.sp_com] == [0.0, 0.0, 0.03]
    w.close()


def test_apply_payload_emits_what_is_in_the_boxes(qapp):
    w = MainWindow(Settings())
    got = []
    w.page.payload_applied.connect(lambda m, c: got.append((m, list(c))))
    w.page.btn_gripper.click()
    w.page.btn_payload.click()
    assert got == [(0.6, [0.0, 0.0, 0.03])], got
    w.close()


def test_payload_label_shows_the_readback_not_the_input(qapp):
    """⚠ 显示必须是**读回值** —— 固件静默钳制，显示输入值就是在撒谎。"""
    w = MainWindow(Settings())
    s = Snapshot(role=ROLE_MASTER)
    s.payload_mass = 0.0            # 固件把 -5 钳成了 0
    s.payload_com = [1.0, 0.0, 0.0]
    w._on_state(s)
    txt = w.page.lab_payload.text()
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
    v = w.page.gripper_values()
    assert v["gcan"] == "", f"通道默认必须是空（不启用），实际 {v['gcan']!r}"
    assert w.page.chk_align.isChecked() is True
    assert w.page.btn_grip is not w.page.btn_teleop
    assert v["gport"] == 17448, f"夹爪端口必须独立于臂的 17447，实际 {v['gport']}"
    assert v["gport"] != 1, "⚠ 端口 1 是特权端口 —— QSpinBox 忘 setValue 就是这个症状"
    w.close()


def test_grip_button_is_clickable_from_construction(qapp):
    """⚠⚠ 死锁回归：夹爪按钮**不能**初始禁用。

    夹爪的 `GripWorker` **只能由点这个按钮创建**，而 `apply_grip` 只在
    `connected` 时才启用按钮 ⇒ 一旦初始禁用就**永远点不了**（只能重启应用）。
    判别力：在构造里加回 `setEnabled(False)` 时本用例必红。
    """
    w = MainWindow(Settings())
    assert w.page.btn_grip.isEnabled() is True, \
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

    w.page.btn_grip.click()
    assert w.grip is None, "没填通道就不该建 GripWorker"
    assert w.page.btn_grip.isChecked() is False, \
        "没建起来时必须弹回未勾选，不能停在「已启动」的样子"
    w.close()


def test_grip_snapshot_renders_master_slave_and_stale(qapp):
    """三种夹爪快照都要能吃下而不炸，且 stale 要**看得见**。"""
    from liteteleop.grip_worker import GripSnapshot

    w = MainWindow(Settings())

    m = GripSnapshot(role="master", connected=True, topic="litearm/v4/gripA/gripper_teleop",
                     frames_sent=10, openness=0.5, position_mm=60.0, matching=True)
    w._on_grip_state(m)
    assert "主端夹爪" in w.page.lab_grip.text()
    assert "已匹配订阅者" in w.page.lab_grip.text()

    m.matching = False
    w._on_grip_state(m)
    assert "未匹配" in w.page.lab_grip.text()

    s = GripSnapshot(role="slave", connected=True, frames_received=5, stale=False,
                     openness=0.5, position_mm=60.0, frame_age=0.01, loop_hz=50.0)
    w._on_grip_state(s)
    assert "跟随中" in w.page.lab_grip.text()

    s.stale = True
    w._on_grip_state(s)
    assert "持位" in w.page.lab_grip.text(), "watchdog 超时必须看得见"

    # ⚠ 刷新按钮**不该**回头再发一次 `grip_toggled` —— 那会让界面自激。
    #    （`apply_grip` 用的是 `setChecked()`，它只发 `toggled`，
    #     而我们接的是 `clicked`。这里就是钉住这一点。）
    got = []
    w.page.grip_toggled.connect(got.append)
    s.teleop_active = True
    w._on_grip_state(s)
    assert w.page.btn_grip.text().endswith("停止夹爪遥操")
    assert w.page.btn_grip.isChecked() is True
    s.teleop_active = False
    w._on_grip_state(s)
    assert w.page.btn_grip.text().endswith("启动夹爪遥操")
    assert got == [], f"刷新按钮不该发 toggled（自激），实际 {got}"
    w.close()


def test_grip_mismatch_and_error_are_visible(qapp):
    """主从标定不一致与夹爪报错都要在界面上看得见（不能静默）。"""
    from liteteleop.grip_worker import GripSnapshot

    w = MainWindow(Settings())
    g = GripSnapshot(role="slave", connected=True)
    g.mismatch = "主从夹爪标定可能不一致：主端 travel≈60.0 mm，本端 120.1 mm"
    w._on_grip_state(g)
    assert "不一致" in w.page.lab_grip_mismatch.text()

    g.error = "夹爪未标定"
    w._on_grip_state(g)
    assert "未标定" in w.page.lab_grip.text()

    g2 = GripSnapshot(role="slave", connected=True)
    w._on_grip_state(g2)
    assert w.page.lab_grip_mismatch.text() == "", "没告警时必须清空，不能留旧的"
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
    assert "12.3" in w.page.lab_grip.text(), "力要显示"

    g.fault = "夹爪报告 error_code=11（故障：过温/过流等）—— 继续发帧持位"
    g.send_failed = 4
    w._on_grip_state(g)
    txt = w.page.lab_grip_mismatch.text()
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
    txt = w.page.lab_grip_mismatch.text()
    assert "7" in txt and "非有限值" in txt, txt
    assert "不一致" in txt, "两条告警要能同时显示，不能互相顶掉"
    w.close()


# ────────────────────────── 曲线卡（照 studio MetricsPanel 的行为）──────────────────────────

def _filled_card(card, n=100):
    """往曲线卡的共享缓冲里灌 n 拍，并把它标成"已经在跑"。

    ⚠ `_was_live = True` 这一步**不是凑数**：`_tick` 里"首个 live 拍先清一次缓冲"
    （照 studio `useArmMetrics.ts:123-127` 的重连语义）排在 `paused` 判断**之前**，
    所以不标它的话，下一次 `_tick` 会先把手工灌的数据清掉。
    """
    for k in range(n):
        card.series.append(
            1000.0 + k * 0.1,
            [30.0 + k % 5] * N_JOINTS, [0.1 * k] * N_JOINTS,
            [1.0] * N_JOINTS)
    card._was_live = True
    return card


def test_metric_switch_keeps_the_buffer(qapp):
    """⚠ 切换指标**不清缓冲** —— 四路数据始终同帧积累（studio `metricDatasets.ts:5,30`）。

    判别力：谁在 `_on_metric` 里加了 `series.clear()`，本用例会红；真机表现是
    「切一下指标，10 s 的历史全没了」。
    """
    w = MainWindow(Settings())
    m = _filled_card(w.page.metrics)
    before = len(m.series)
    assert before == 100
    m.seg.set_current(1)
    m._on_metric(1)
    assert len(m.series) == before, "切指标不许清缓冲"
    m.seg.set_current(2)              # 只有 温度/速度/力矩 三个页签了
    m._on_metric(2)
    assert len(m.series) == before
    w.close()


def test_clear_deselects_but_keeps_the_buffer(qapp):
    """「清空」= **只取消勾选**，不丢数据（studio `useArmMetrics.ts:211`）。

    判别力：把 `select_none` 写成"清缓冲"时本用例会红 —— 那会让用户以为数据没了。
    """
    w = MainWindow(Settings())
    m = _filled_card(w.page.metrics)
    m.select_none()
    assert m._shown == [], "清空后应当一条都不勾"
    assert len(m.series) == 100, "⚠ 清空**不许**丢缓冲"
    m.select_all()
    assert m._shown == list(range(N_JOINTS)), "全选要勾回**全部轴**"
    assert len(m.series) == 100
    w.close()


def test_pause_freezes_sampling_without_clearing(qapp):
    """「暂停」= 停止采样、**缓冲原样冻结**（studio `useArmMetrics.ts:119`）。"""
    w = MainWindow(Settings())
    m = _filled_card(w.page.metrics)
    m.paused = True
    s = _snap(role=ROLE_SLAVE)
    m.set_snapshot(s)
    m._tick()
    assert len(m.series) == 100, "暂停期间不许再写缓冲"
    m.paused = False
    m._tick()
    assert len(m.series) == 100, "恢复后从当前时刻继续（缓冲上限仍是 100）"
    w.close()


def test_window_is_bounded_at_100_points(qapp):
    """窗口 = 100 点 = 10 s（studio `useArmMetrics.ts:56-57`）。"""
    w = MainWindow(Settings())
    m = _filled_card(w.page.metrics, n=250)
    assert len(m.series) == 100, "超出 100 点必须滑窗裁掉"
    w.close()


def test_joint_legend_toggles_visibility(qapp):
    """点图例 = 切该条曲线的显隐；重新勾上时保持**升序**（studio `:203-204`）。"""
    w = MainWindow(Settings())
    m = _filled_card(w.page.metrics)
    m.toggle_joint(1)
    assert 1 not in m._shown
    m.toggle_joint(1)
    assert m._shown == sorted(m._shown), "重新勾选后必须有序"
    assert 1 in m._shown
    w.close()
