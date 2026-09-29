"""控件语汇 —— 卡片 / 徽标 / 分段控件 / 指标块 / 大 STOP。

逐个对着 `litearm-studio` 的组件写，出处标在各自的 docstring 里。
尺寸按 studio 的参考视口（1rem = 16.44px）**折算成 px 写死**，见 `theme.py`。

⚠ 这一层**只管长相**，不碰任何业务状态 —— 业务在 `pages.py`。
"""
from __future__ import annotations

from typing import List, Optional, Sequence

from PyQt5 import QtCore, QtGui, QtWidgets

from .theme import C, FONT_MONO, FONT_SANS, RADIUS, mono, sans

__all__ = [
    "Card", "Badge", "StatusPill", "SegmentedControl", "MetricTile",
    "StopButton", "ScrollColumn", "hint", "separator", "field_row",
]


def _lab(text: str, role: str = "") -> QtWidgets.QLabel:
    w = QtWidgets.QLabel(text)
    if role:
        w.setProperty("role", role)
    return w


def hint(text: str, role: str = "hint", wrap: bool = True) -> QtWidgets.QLabel:
    w = _lab(text, role)
    w.setWordWrap(wrap)
    return w


def separator() -> QtWidgets.QFrame:
    f = QtWidgets.QFrame()
    f.setProperty("role", "separator")
    f.setFixedHeight(1)
    return f


# ────────────────────────── 卡片 ──────────────────────────
class Card(QtWidgets.QFrame):
    """白底圆角卡 + 卡头（标题 + 可选徽标）。

    出处 `studio/src/components/ui/card.tsx:15`：底 `--card`、圆角 `rounded-xl`(14.38px)、
    1px 描边、上下内边距 `--card-spacing`。
    ⚠ studio 的描边走 `ring-1 ring-foreground/10`（box-shadow，**不占布局**）；
    QSS 没有 ring ⇒ 这里用 `border`，观感等价于 `--line`。
    ⚠ studio 真实页面里的卡片把内边距覆盖成 **12.33px**（`PreviewPanel.tsx:96` 等），
    不是默认的 16.44 —— 这里跟真实页面走。
    ⚠ **卡头与卡体之间默认没有分隔线**，只靠间距（`card.tsx`）。
    """

    def __init__(self, title: str = "", badge: Optional[QtWidgets.QWidget] = None,
                 parent=None, padding: int = 12, spacing: int = 10):
        super().__init__(parent)
        self.setObjectName("Card")
        self.setStyleSheet(
            f"QFrame#Card {{ background: {C['card']};"
            f" border: 1px solid {C['line']};"
            f" border-radius: {RADIUS['xl']}px; }}")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(padding, padding, padding, padding)
        lay.setSpacing(spacing)

        self.header = QtWidgets.QHBoxLayout()
        self.header.setContentsMargins(0, 0, 0, 0)
        self.header.setSpacing(8)
        self.title = _lab(title, "cardTitle")
        self.header.addWidget(self.title)
        self.header.addStretch(1)
        self._badge = None
        if badge is not None:
            self.header.addWidget(badge)
        lay.addLayout(self.header)
        if not title:
            self.title.hide()

        #: 卡体 —— 调用方往这里塞内容。
        self.body = QtWidgets.QVBoxLayout()
        self.body.setContentsMargins(0, 0, 0, 0)
        self.body.setSpacing(8)
        lay.addLayout(self.body)

    def set_badge(self, badge: Optional[QtWidgets.QWidget]) -> None:
        if self._badge is not None:
            self.header.removeWidget(self._badge)
            self._badge.deleteLater()
        self._badge = badge
        if badge is not None:
            self.header.addWidget(badge)

    def add(self, w) -> None:
        """往卡体里加一个控件或子布局。"""
        if isinstance(w, QtWidgets.QWidget):
            self.body.addWidget(w)
        else:
            self.body.addLayout(w)

    def add_row(self, *widgets, spacing: int = 8, stretch_at_end: bool = True) -> QtWidgets.QHBoxLayout:
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(spacing)
        for w in widgets:
            if w is None:
                row.addStretch(1)
            elif isinstance(w, str):
                row.addWidget(_lab(w, "fieldLabel"))
            else:
                row.addWidget(w)
        if stretch_at_end:
            row.addStretch(1)
        self.body.addLayout(row)
        return row


# ────────────────────────── 徽标 ──────────────────────────
#: studio `badge.tsx:8` 的变体 → (底, 文字, 描边)。`warning` 在 studio 里**没有**
#: （`badge.tsx` 只有 default/secondary/success/destructive/outline/ghost），
#: 但设计稿有"未启动/未连接"这种中性态 ⇒ 用 outline；警告态我们照
#: `SoloConsole.tsx:107-110` 的警告横幅口径自己拼一组。
_BADGE_VARIANTS = {
    "default": (C["primary"], C["primary_foreground"], "transparent"),
    "secondary": (C["secondary"], C["secondary_foreground"], "transparent"),
    "ok": (C["ok_soft"], C["ok"], C["ok_line"]),
    "warn": (C["warn_soft"], C["warn"], C["warn_line"]),
    "danger": (C["danger_soft"], C["danger"], C["danger_line"]),
    "info": (C["info_soft"], C["info"], C["info_line"]),
    "outline": (C["line_soft"], C["ink_muted"], C["line"]),
}


class Badge(QtWidgets.QLabel):
    """胶囊徽标。出处 `studio/src/components/ui/badge.tsx:8`：

    高 `h-5`(20.55px)、圆角 `rounded-4xl` 全胶囊、`px-2`(8.22)、`py-0.5`(2.06)、
    字号 `text-xs`(12.33)、字重 500。
    ⚠ studio **没有 `warning` 变体**，这里按同一版式补了一个（用 `--warn*` 三件套）。
    """

    def __init__(self, text: str = "", variant: str = "outline", parent=None):
        super().__init__(text, parent)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.set_variant(variant)

    def set_variant(self, variant: str) -> None:
        bg, fg, line = _BADGE_VARIANTS.get(variant, _BADGE_VARIANTS["outline"])
        self._variant = variant
        self.setStyleSheet(
            f"QLabel {{ background: {bg}; color: {fg};"
            f" border: 1px solid {line}; border-radius: {RADIUS['pill']}px;"
            f" padding: 1px 8px; {sans(12.33, 500)} }}")

    def set_state(self, text: str, variant: str) -> None:
        self.setText(text)
        self.set_variant(variant)


class StatusPill(QtWidgets.QFrame):
    """顶栏的连接状态胶囊（圆点 + 文字）。

    出处 `studio/src/layout/TopBar.tsx:66-73`：`rounded-full`、`px-[0.6875rem]`(11.30)、
    `py-1`(4.1)、`gap-[0.4375rem]`(7.19)、圆点 7.19px、文字 12.84px/600。
    圆点色：已连接 `--success`、错误 `--destructive`、其余 `--muted-foreground`。
    """

    def __init__(self, text: str = "未连接", parent=None):
        super().__init__(parent)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(11, 4, 11, 4)
        lay.setSpacing(7)
        self.dot = QtWidgets.QFrame()
        self.dot.setFixedSize(7, 7)
        self.lab = QtWidgets.QLabel(text)
        self.lab.setStyleSheet(f"QLabel {{ {sans(12.84, 600)} color: {C['ink_muted']}; }}")
        lay.addWidget(self.dot)
        lay.addWidget(self.lab)
        self.set_state(text, "idle")

    def set_state(self, text: str, kind: str = "idle") -> None:
        """`kind`: `ok` 已连接 / `bad` 错误 / 其它 = 中性。"""
        color = {"ok": C["ok"], "bad": C["danger"]}.get(kind, C["muted_foreground"])
        bg = {"ok": C["ok_soft"], "bad": C["danger_soft"]}.get(kind, C["line_soft"])
        border = {"ok": C["ok_line"], "bad": C["danger_line"]}.get(kind, C["line"])
        self.lab.setText(text)
        self.lab.setStyleSheet(f"QLabel {{ {sans(12.84, 600)} color: {color}; }}")
        self.dot.setStyleSheet(
            f"QFrame {{ background: {color}; border-radius: 3.5px; }}")
        self.setStyleSheet(
            f"QFrame {{ background: {bg}; border: 1px solid {border};"
            f" border-radius: {RADIUS['pill']}px; }}")


# ────────────────────────── 分段控件 ──────────────────────────
class SegmentedControl(QtWidgets.QFrame):
    """单选分段控件。出处 `studio/src/components/SegmentedControl.tsx` + 三处调用点。

    ⚠ 组件本身**零样式**，全部由调用点内联传入（轨道/段/选中项三套）。这里采
    **中号**那档（`PreviewPanel.tsx:103-107`）：轨道底 `--line-soft`、轨道圆角 9.25px、
    轨道内边距与段间隙都是 3.08px、段圆角 7.19px、段内边距 5.14/12.33px。

    ⚠ 选中项在 studio 里**没有描边**，只有 `box-shadow: 0 1px 2px rgba(16,24,40,.1)`
    浮起。QSS 不支持 `box-shadow` ⇒ 这里用 `QGraphicsDropShadowEffect` 补上，
    而不是加一根描边（加描边会偏离设计）。
    """

    changed = QtCore.pyqtSignal(int)

    def __init__(self, items: Sequence[str], current: int = 0,
                 size: str = "md", parent=None):
        super().__init__(parent)
        self._size = size
        self._buttons: List[QtWidgets.QPushButton] = []
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(3)

        pad_v, pad_h, fsize = {
            "md": (5, 12, 12.84),
            "sm": (5, 8, 11.81),
            "lg": (8, 13, 13.87),
        }.get(size, (5, 12, 12.84))

        for i, text in enumerate(items):
            b = QtWidgets.QPushButton(text)
            b.setCheckable(True)
            b.setAutoExclusive(True)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.clicked.connect(lambda _c, idx=i: self._on_click(idx))
            b.setStyleSheet(self._item_qss(False, pad_v, pad_h, fsize))
            lay.addWidget(b)
            self._buttons.append(b)
        if self._buttons:
            self._buttons[max(0, min(current, len(self._buttons) - 1))].setChecked(True)
        self._refresh()

        self.setStyleSheet(
            f"SegmentedControl {{ background: {C['line_soft']};"
            f" border-radius: 9px; }}")

    @staticmethod
    def _item_qss(active: bool, pad_v: int, pad_h: int, fsize: float) -> str:
        if active:
            return (f"QPushButton {{ background: {C['seg_active']}; color: {C['ink']};"
                    f" border: none; border-radius: 7px;"
                    f" padding: {pad_v}px {pad_h}px; {sans(fsize, 600)} }}")
        return (f"QPushButton {{ background: transparent; color: {C['ink_subtle']};"
                f" border: none; border-radius: 7px;"
                f" padding: {pad_v}px {pad_h}px; {sans(fsize, 500)} }}"
                f"QPushButton:hover {{ color: {C['ink_strong']}; }}")

    def _on_click(self, idx: int) -> None:
        self._refresh()
        self.changed.emit(idx)

    def _refresh(self) -> None:
        pad_v, pad_h, fsize = {
            "md": (5, 12, 12.84), "sm": (5, 8, 11.81), "lg": (8, 13, 13.87),
        }.get(self._size, (5, 12, 12.84))
        for b in self._buttons:
            active = b.isChecked()
            b.setStyleSheet(self._item_qss(active, pad_v, pad_h, fsize))
            # ⚠ studio 的选中项靠阴影浮起、无描边。QSS 无 box-shadow ⇒ 用效果补。
            if active:
                eff = QtWidgets.QGraphicsDropShadowEffect(b)
                eff.setBlurRadius(4)
                eff.setOffset(0, 1)
                eff.setColor(QtGui.QColor(16, 24, 40, 26))     # rgba(16,24,40,.1)
                b.setGraphicsEffect(eff)
            else:
                b.setGraphicsEffect(None)

    def current(self) -> int:
        for i, b in enumerate(self._buttons):
            if b.isChecked():
                return i
        return 0

    def set_current(self, idx: int, emit: bool = False) -> None:
        """程序化选中。

        ⚠ `emit=False`（默认）**不发** `changed` —— 避免"程序化改一下"被当成用户操作，
        同 `pages` 里按钮那条纪律（`setChecked` 只发 `toggled`、而外面接的是 `clicked`）。
        ⚠ 但**联动逻辑挂在 `changed` 上**（如 `TeleopPage._role_changed` 会启停「主臂 IP」
        那一格）⇒ 想走完整路径就得显式 `emit=True`，否则会出现"选中了但联动没跑"。
        """
        if 0 <= idx < len(self._buttons):
            self._buttons[idx].setChecked(True)
            self._refresh()
            if emit:
                self.changed.emit(idx)


# ────────────────────────── 指标块 ──────────────────────────
class MetricTile(QtWidgets.QFrame):
    """`标签 / 数值 单位` 的小块 —— 主从链路卡里那四格。

    照 `TopBar.tsx:113-123` 的指标组版式：标签 12.33px/500 `--muted-foreground`；
    数值**等宽** 14.38px/700 `--foreground`；单位**等宽** 11.30px `--muted-foreground`。
    ⚠ 数值缺失时显示 **`—`（em dash）而不是 0** —— studio 是这么做的
    （`MetricsPanel.tsx:220`），0 会被当成真实读数。
    """

    def __init__(self, label: str, unit: str = "", parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.lab = QtWidgets.QLabel(label)
        self.lab.setStyleSheet(f"QLabel {{ {sans(12.33, 500)} color: {C['muted_foreground']}; }}")
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(3)
        self.val = QtWidgets.QLabel("—")
        self.val.setStyleSheet(f"QLabel {{ {mono(14.38, 700)} color: {C['foreground']}; }}")
        self.unit = QtWidgets.QLabel(unit)
        self.unit.setStyleSheet(f"QLabel {{ {mono(11.30)} color: {C['muted_foreground']}; }}")
        row.addWidget(self.val)
        if unit:
            row.addWidget(self.unit, 0, QtCore.Qt.AlignBottom)
        row.addStretch(1)
        lay.addWidget(self.lab)
        lay.addLayout(row)

    def set_value(self, text: str, kind: str = "none") -> None:
        """`kind`: `ok` / `bad` 会换数值颜色（照 TopBar 的 success/destructive 口径）。"""
        color = {"ok": C["ok"], "bad": C["danger"]}.get(kind, C["foreground"])
        self.val.setText(text)
        self.val.setStyleSheet(f"QLabel {{ {mono(14.38, 700)} color: {color}; }}")


# ────────────────────────── 大 STOP ──────────────────────────
class StopButton(QtWidgets.QPushButton):
    """设计稿右栏那个大 STOP。

    出处 `studio/src/components/StopButton.tsx:11-30`：高 94.52px（`h-[5.75rem]`）、
    圆角 14.38px、**线性渐变** `#f0424a → #d5262e`、阴影 `rgba(213,38,46,.30)`、
    文字 `STOP` 41.09px/**800**/字距 3.08px、白色。
    ⚠ studio 里这个按钮**没有 hover 差异**，只有 `cursor: pointer`。
    ⚠ 颜色是硬编码 hex、**不跟主题**。

    ⛔ 本仓的语义（用户裁决 2026-09-29）：**这个按钮 = 停止遥操（受控接管 movej）**，
    **不是急停**。急停会让臂**自由落体**，不该做成这么大、这么容易误碰的按钮。
    见 `pages.TeleopPage` 里的接线。
    """

    def __init__(self, text: str = "STOP", parent=None):
        super().__init__(text, parent)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        self.setMinimumHeight(95)
        self.setStyleSheet(
            f"QPushButton {{ color: #ffffff; border: none;"
            f" border-radius: {RADIUS['xl']}px;"
            f" background: qlineargradient(x1:0, y1:0, x2:0, y2:1,"
            f"   stop:0 {C['stop_top']}, stop:1 {C['stop_bottom']});"
            f" font-family: {FONT_SANS};"
            f" font-size: 41px; font-weight: 800; letter-spacing: 3px; }}"
            f"QPushButton:disabled {{ background: {C['line']}; color: {C['ink_ghost']}; }}")

    def set_enabled_look(self, on: bool) -> None:
        """studio 的 disabled：`opacity .45` + `grayscale(.6)`（`StopButton.tsx:23`）。
        Qt 用 `setEnabled(False)` + 上面那条 disabled QSS 近似。"""
        self.setEnabled(on)


# ────────────────────────── 滚动列 ──────────────────────────
class ScrollColumn(QtWidgets.QScrollArea):
    """一列可纵向滚动的内容 —— studio 三列的 `SCROLL_COLUMN`
    （`responsive.ts:18`：`flex-direction: column; min-height: 0; overflow-y: auto`）。

    ⚠ 左栏要塞「主从链路 + 曲线 + 关节表 + 连接状态」四张卡，一定超过一屏高度，
    没有这个滚动区底部会被裁掉。
    """

    def __init__(self, spacing: int = 12, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        inner = QtWidgets.QWidget()
        inner.setObjectName("AppBg")
        self.col = QtWidgets.QVBoxLayout(inner)
        self.col.setContentsMargins(0, 0, 0, 0)
        self.col.setSpacing(spacing)
        self.setWidget(inner)

    def add(self, w) -> None:
        self.col.addWidget(w)

    def add_stretch(self) -> None:
        self.col.addStretch(1)


# ────────────────────────── 表单行 ──────────────────────────
def field_row(label: str, *widgets) -> QtWidgets.QWidget:
    """`标签  控件…` 一行。studio 的表单是 label 在左、控件紧跟（不是 QFormLayout 的
    右对齐两列），所以这里用 QHBoxLayout 手工拼。"""
    w = QtWidgets.QWidget()
    row = QtWidgets.QHBoxLayout(w)
    row.setContentsMargins(0, 0, 0, 0)
    row.setSpacing(8)
    row.addWidget(_lab(label, "fieldLabel"))
    for x in widgets:
        if isinstance(x, str):
            row.addWidget(_lab(x, "fieldLabel"))
        else:
            row.addWidget(x)
    row.addStretch(1)
    return w
