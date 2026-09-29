"""设计令牌 + 全局 QSS —— 视觉语言移植自 `litearm-studio`。

## 出处

`litearm-studio` 是 React + Tailwind + shadcn 的前端，它的视觉语言落在
`src/styles/index.css` 的两组 CSS 变量上：

1. **shadcn 核心令牌**（`--background` / `--card` / `--border` / `--primary` …）。
   亮色是 `oklch(...)`，已换算成 sRGB hex 落在本模块。
2. **LiteArm 业务令牌**（`--app-bg` / `--ink*` / `--line*` / `--ok|warn|danger` 三态色）。
   这套**才是** `features/**` 里实际在用的主体（亮/暗本来都是 hex）。

## 换算基准

studio 的 `html { font-size: clamp(13px, 1.522vh, 36px) }` 在参考视口 **1920×1080**
下给出 **1rem ≈ 16.44px**；studio 源码里写的 px 值就是按这个基准调的。
本模块直接把那些值**按 px 写死**（Qt 是桌面应用，窗口尺寸我们自己定，
不需要 studio 那套随视口缩放的流体字号）。换算过的值都标了原式。

## ⚠ studio 自己就是硬编码、不跟主题的

- **大 STOP 按钮**：渐变 `#f0424a → #d5262e`、阴影 `rgba(213,38,46,.30)`
  （`StopButton.tsx:21-22`）——亮暗同色。

⛔ **不要**把它改成跟随主题：那会偏离设计稿。
⚠ studio 的**左侧导航栏**（`RailNav.tsx`：底 `#141b26` / hover `#222c3a` /
选中 `#2a3444` / 文字 `#8b97a6`）**本仓不做了** —— 用户裁决去掉整条导航，
那五个令牌已一并删除。

## ⚠ 已知的 Qt 落差（studio 有、这里做不到 1:1）

- studio 的卡片描边走 `ring-1 ring-foreground/10`（**box-shadow**，不占布局）；
  QSS 没有 ring ⇒ 只能用 `border`，并靠 `box-sizing`/内边距抵消那 1px。
- studio 的分段控件选中项靠 `box-shadow: 0 1px 2px rgba(16,24,40,.1)` 浮起、**无描边**；
  Qt 的 QSS 也不支持 `box-shadow` ⇒ 见 `cards.SegmentedControl` 用
  `QGraphicsDropShadowEffect` 补，或退化为一根极浅的描边。
"""
from __future__ import annotations

from typing import Dict

__all__ = [
    "C", "FONT_SANS", "FONT_MONO", "RADIUS", "JOINT_COLORS", "joint_color",
    "sans", "mono", "qss", "TOKENS",
]

# ────────────────────────── 颜色令牌 ──────────────────────────
#: 键名刻意与 studio 的 CSS 变量同名（去掉 `--`、`-` 换 `_`），便于逐条对账。
C: Dict[str, str] = {
    # ── 外壳与卡片（shadcn 那组，oklch → sRGB）──
    "background": "#ffffff",
    "foreground": "#0a0a0a",
    "card": "#ffffff",
    "card_foreground": "#0a0a0a",
    "primary": "#171717",            # oklch(0.205 0 0)
    "primary_foreground": "#fafafa",  # oklch(0.985 0 0)
    "secondary": "#f5f5f5",          # oklch(0.97 0 0)
    "secondary_foreground": "#171717",
    "muted": "#f5f5f5",
    "muted_foreground": "#737373",   # oklch(0.556 0 0)
    "destructive": "#e7000b",        # oklch(0.577 0.245 27.325)
    "border": "#e5e5e5",             # oklch(0.922 0 0)
    "input": "#e5e5e5",
    "ring": "#a1a1a1",               # oklch(0.708 0 0)

    # ── LiteArm 业务令牌（亮色，本来就是 hex）──
    "app_bg": "#f6f7f9",             # 应用外壳底色（卡片比它亮 ⇒ 浮起）
    "ink": "#17212f",                # 最强文本：标题、关键数值
    "ink_strong": "#3c4a5c",         # 次级标题、按钮文字
    "ink_muted": "#46596f",          # 正文次要文本
    "ink_soft": "#5d6b7d",           # 三级文本
    "ink_subtle": "#6b7787",         # 非激活态文本
    "ink_faint": "#9aa6b6",          # 占位、禁用、图表刻度
    "ink_ghost": "#b3bcc8",          # 更弱的禁用态文本
    "line": "#e4e9f0",               # 标准描边
    "line_strong": "#c3cbd6",        # 强调描边、未选中的开关
    "line_soft": "#f1f4f9",          # 极轻分隔线 / 内嵌底色
    "hover": "#f4f7fb",              # 行悬浮底色
    "seg_active": "#ffffff",         # 分段控件当前项底色（比轨道亮）
    "chip": "#17212f",               # 深色胶囊底
    "chip_fg": "#ffffff",            # 深色胶囊文字
    "chip_muted": "#46596f",         # 深色胶囊次要描边
    "info": "#2563eb",
    "info_soft": "#eaf2ff",
    "info_line": "#cddcfa",
    "ok": "#137a44",
    "ok_soft": "#eaf7ef",
    "ok_line": "#c6e9d3",
    "warn": "#8a6410",
    "warn_soft": "#fff7e2",
    "warn_line": "#f2c86b",
    "danger": "#c62b30",
    "danger_soft": "#fdecec",
    "danger_line": "#f3c9ca",

    # ── ⛔ 硬编码、不跟主题 ──
    # ⚠ 原来这里还有一组 `rail_*`（studio 左导航栏那套 `#141b26`/`#222c3a`/`#2a3444`/`#8b97a6`）。
    #   2026-09-29 用户裁决去掉左侧导航栏 ⇒ 那五个令牌**已随 RailNav 一起删除**，
    #   ⛔ 别再加回来（没有使用者的令牌就是死代码）。
    "stop_top": "#f0424a",           # StopButton 渐变上端
    "stop_bottom": "#d5262e",        # 渐变下端
    "amber": "#f5a524",              # 警告圆点（studio 多处硬编码）

    # ── 图表（studio MetricsPanel 的口径）──
    "chart_grid": "rgba(128,138,150,0.16)",
    "chart_axis": "rgba(128,138,150,0.3)",
    "chart_tick": "#9aa6b6",
}

#: 关节配色 —— 逐字抄 `studio/src/lib/colors.ts:2`。
#: ⚠ **不是轴数上限**：第 8 个轴起从头循环（`jointColor` 取模）。
JOINT_COLORS = ["#e5484d", "#3b82f6", "#12a150", "#a855f7",
                "#f5a524", "#06b6d4", "#ec4899"]


def joint_color(i: int) -> str:
    """第 `i`（0 起）个轴的颜色。照 `colors.ts:11-13` 取模循环。"""
    return JOINT_COLORS[i % len(JOINT_COLORS)]


# ────────────────────────── 字体 ──────────────────────────
#: studio 写的族链是 `system-ui → PingFang SC → HarmonyOS Sans SC → Microsoft YaHei`。
#: 本机（Linux）前三个都没有，能命中的是 **Noto Sans CJK SC** —— 把它接在链尾，
#: 这样在 mac/Windows 上仍然是原设计字体，在本机也不会掉到无衬线兜底。
FONT_SANS = ('"PingFang SC", "HarmonyOS Sans SC", "Microsoft YaHei", '
             '"Noto Sans CJK SC", "DejaVu Sans", sans-serif')

#: studio 用 `'JetBrains Mono', monospace`。本机没装 JetBrains Mono，
#: 退到 DejaVu Sans Mono / Noto Sans Mono。
FONT_MONO = '"JetBrains Mono", "DejaVu Sans Mono", "Noto Sans Mono", monospace'


def sans(size: float, weight: int = 400) -> str:
    """QSS 用的 sans 字体声明，如 `font-family: ...; font-size: 12px; font-weight: 500`。"""
    return f"font-family: {FONT_SANS}; font-size: {round(size)}px; font-weight: {weight};"


def mono(size: float, weight: int = 400) -> str:
    """QSS 用的等宽字体声明。studio 里数值/单位/端口固件串走等宽。"""
    return f"font-family: {FONT_MONO}; font-size: {round(size)}px; font-weight: {weight};"


# ────────────────────────── 圆角 ──────────────────────────
#: `--radius: 0.625rem` = 10.27px；其余是它的倍数（`index.css:39,194-200`）。
RADIUS = {
    "sm": 6,        # 0.375rem × 16.44 = 6.16
    "md": 8,        # 0.5rem   × 16.44 = 8.22
    "lg": 10,       # 0.625rem × 16.44 = 10.27
    "xl": 14,       # 0.875rem × 16.44 = 14.38   ← 卡片
    "pill": 999,    # rounded-4xl / rounded-full
}


# ────────────────────────── 全局 QSS ──────────────────────────
def qss() -> str:
    """全局样式表。**只做"通用控件"**：

    卡片 / 徽标 / 分段控件 / 大 STOP 这些有自己语汇的，由 `cards.py` 各自
    `setStyleSheet` + `setProperty("role", ...)` 负责 —— 塞进全局表只会变成一大坨
    互相打架的选择器。
    """
    return f"""
/* ══════════ 基底 ══════════ */
QWidget {{
    {sans(12.5)}
    color: {C['ink']};
    background: transparent;
}}
QMainWindow, QWidget#Shell, QWidget#AppBg {{ background: {C['app_bg']}; }}

/* ══════════ 文本层级（照 --ink* 那一串）══════════ */
QLabel {{ background: transparent; }}
QLabel[role="cardTitle"] {{ {sans(15, 600)} color: {C['foreground']}; }}
QLabel[role="sectionTitle"] {{ {sans(14, 600)} color: {C['foreground']}; }}
QLabel[role="hint"] {{ {sans(12)} color: {C['ink_muted']}; }}
QLabel[role="hintSubtle"] {{ {sans(11.5)} color: {C['ink_subtle']}; }}
QLabel[role="fieldLabel"] {{ {sans(12.5)} color: {C['ink_muted']}; }}
QLabel[role="value"] {{ {mono(12.5, 700)} color: {C['ink']}; }}
QLabel[role="monoHint"] {{ {mono(11.5)} color: {C['ink_muted']}; }}
QLabel[role="danger"] {{ color: {C['danger']}; }}
QLabel[role="warn"] {{ color: {C['warn']}; }}
QLabel[role="ok"] {{ color: {C['ok']}; }}

/* ══════════ 输入类 ══════════
   ⚠ studio 的输入框高 32.88px（h-8）、圆角 8.22px（rounded-md）、描边 --input。
   Qt 的 QLineEdit 用 padding 撑高度（QSS 的 min-height 对部分控件不生效）。 */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    {sans(12.5)}
    color: {C['ink']};
    background: {C['card']};
    border: 1px solid {C['input']};
    border-radius: {RADIUS['md']}px;
    padding: 4px 8px;
    min-height: 16px;
    selection-background-color: {C['info']};
    selection-color: #ffffff;
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border: 1px solid {C['ring']};
}}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{
    background: {C['line_soft']};
    color: {C['ink_ghost']};
}}
QLineEdit[readOnly="true"] {{ background: {C['line_soft']}; color: {C['ink_soft']}; }}
QLineEdit::placeholder {{ color: {C['ink_faint']}; }}

QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
    width: 14px; border: none; background: transparent;
}}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
    image: none; border-left: 3px solid transparent; border-right: 3px solid transparent;
    border-bottom: 4px solid {C['ink_faint']}; width: 0; height: 0;
}}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
    image: none; border-left: 3px solid transparent; border-right: 3px solid transparent;
    border-top: 4px solid {C['ink_faint']}; width: 0; height: 0;
}}

QComboBox::drop-down {{ border: none; width: 18px; }}
QComboBox::down-arrow {{
    image: none; border-left: 3px solid transparent; border-right: 3px solid transparent;
    border-top: 4px solid {C['ink_muted']}; width: 0; height: 0; margin-right: 6px;
}}
QComboBox QAbstractItemView {{
    background: {C['card']};
    border: 1px solid {C['line']};
    border-radius: {RADIUS['md']}px;
    selection-background-color: {C['hover']};
    selection-color: {C['ink']};
    outline: none;
    padding: 4px;
}}

/* ══════════ 勾选 ══════════ */
QCheckBox {{ {sans(12.5)} color: {C['ink_strong']}; spacing: 7px; }}
QCheckBox::indicator {{
    width: 15px; height: 15px;
    border: 1px solid {C['line_strong']};
    border-radius: {RADIUS['sm']}px;
    background: {C['card']};
}}
QCheckBox::indicator:hover {{ border-color: {C['ink_subtle']}; }}
QCheckBox::indicator:checked {{
    background: {C['chip']};
    border-color: {C['ink']};
    image: none;
}}
QCheckBox::indicator:checked:disabled {{ background: {C['ink_ghost']}; border-color: {C['ink_ghost']}; }}
QCheckBox:disabled {{ color: {C['ink_ghost']}; }}

/* ══════════ 按钮（照 studio button.tsx）══════════
   base：rounded-lg(10.27px) + 1px 透明描边 + 14.38px/500；active 下移 1px（QSS 做不到）。
   default 变体：底 --primary / 字 --primary-foreground。 */
QPushButton {{
    {sans(13)}
    color: {C['primary_foreground']};
    background: {C['primary']};
    border: 1px solid transparent;
    border-radius: {RADIUS['lg']}px;
    padding: 5px 11px;
    min-height: 16px;
}}
QPushButton:hover {{ background: #2e2e2e; }}
QPushButton:pressed {{ background: #000000; }}
QPushButton:disabled {{ background: {C['muted']}; color: {C['ink_ghost']}; }}

/* outline 变体：亮色 --background 底 + --border 描边 */
QPushButton[variant="outline"] {{
    background: {C['background']};
    color: {C['foreground']};
    border: 1px solid {C['border']};
}}
QPushButton[variant="outline"]:hover {{ background: {C['muted']}; }}
QPushButton[variant="outline"]:disabled {{
    background: {C['background']}; color: {C['ink_ghost']}; border-color: {C['line']};
}}

/* secondary 变体 */
QPushButton[variant="secondary"] {{
    background: {C['secondary']}; color: {C['secondary_foreground']}; border-color: transparent;
}}
QPushButton[variant="secondary"]:hover {{ background: #ececec; }}
/* ⚠⚠ `:disabled` 一条都不能省。带属性选择器的 `[variant=...]` 规则**盖过**下面那条
   通用的 `QPushButton:disabled` ⇒ 少了它，"禁用"和"启用"会**渲染得一模一样**
   （离屏逐像素比对：0 个像素不同）—— 于是一个按不动的按钮看起来完全能按。
   2026-09-29 实测踩到：臂维护那四个 `secondary` 键在遥操运行时是禁用的，
   却和可点时长得一样。⭐ 判据 = `tests/test_gui_smoke.py::test_disabled_buttons_look_disabled`。 */
QPushButton[variant="secondary"]:disabled {{
    background: {C['line_soft']}; color: {C['ink_ghost']}; border-color: transparent;
}}

/* ghost 变体 */
QPushButton[variant="ghost"] {{
    background: transparent; color: {C['ink_strong']}; border-color: transparent;
}}
QPushButton[variant="ghost"]:hover {{ background: {C['muted']}; }}
QPushButton[variant="ghost"]:disabled {{
    background: transparent; color: {C['ink_ghost']};
}}

/* 行进/选中态（studio 用 --chip 深色胶囊表示「开」） */
QPushButton[variant="toggle"] {{
    background: {C['card']}; color: {C['ink_strong']};
    border: 1px solid {C['line_strong']};
    {sans(14, 600)}
    padding: 8px 16px;
    border-radius: 11px;
}}
QPushButton[variant="toggle"]:hover {{ background: {C['hover']}; }}
QPushButton[variant="toggle"]:checked {{
    background: {C['chip']}; color: {C['chip_fg']}; border-color: {C['ink']};
}}
QPushButton[variant="toggle"]:disabled {{
    background: {C['card']}; color: {C['ink_ghost']}; border-color: {C['line']};
}}

/* primary 大按钮（卡片右下那个「启动遥操」） */
QPushButton[variant="primaryAction"] {{
    {sans(14, 600)}
    background: {C['primary']}; color: {C['primary_foreground']};
    border: 1px solid transparent; border-radius: 11px; padding: 9px 16px;
}}
QPushButton[variant="primaryAction"]:hover {{ background: #2e2e2e; }}
QPushButton[variant="primaryAction"]:disabled {{ background: {C['muted']}; color: {C['ink_ghost']}; }}

/* ══════════ 表格 ══════════
   ⚠⚠ **这里绝不能写 `QTableWidget::item { ... }`。**
   一旦全局样式表里出现 `::item` 规则，Qt 就改用"样式表驱动"的单元格绘制，
   **item 自己的 `setBackground()` / `setForeground()` 会被整个忽略**。
   后果不是样式难看，是**安全指示消失**：关节表 `err` 列那套红/绿
   （`widgets.JointTable.update_from`，`err != 1` 才红）会变成一整列空白。
   ⚠ 而且**读到 model 的断言抓不到这个**（`item.background().color()` 照样是对的）
   ⇒ 判据必须落到**像素**上，见 `tests/test_gui_smoke.py::test_err_colour_reaches_the_pixels`。
   （2026-09-29 离屏 A/B 实测：带 `::item` 时整列不画；删掉即恢复。） */
QTableWidget, QTableView {{
    {sans(12)}
    background: {C['card']};
    alternate-background-color: {C['card']};
    border: none;
    gridline-color: {C['line_soft']};
    selection-background-color: {C['hover']};
    selection-color: {C['ink']};
    outline: none;
}}
QHeaderView::section {{
    {sans(11.5, 600)}
    color: {C['ink_muted']};
    background: {C['card']};
    border: none;
    border-bottom: 1px solid {C['line_soft']};
    padding: 4px 6px;
}}
QTableCornerButton::section {{ background: {C['card']}; border: none; }}

/* ══════════ 日志 / 滚动区 ══════════ */
QPlainTextEdit, QTextEdit {{
    {mono(11.5)}
    color: {C['ink_muted']};
    background: {C['card']};
    border: none;
    selection-background-color: {C['info_soft']};
}}
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QAbstractScrollArea::corner {{ background: transparent; }}

QScrollBar:vertical {{ background: transparent; width: 9px; margin: 0; }}
QScrollBar::handle:vertical {{
    background: {C['line_strong']}; border-radius: 4px; min-height: 28px;
}}
QScrollBar::handle:vertical:hover {{ background: {C['ink_faint']}; }}
QScrollBar:horizontal {{ background: transparent; height: 9px; margin: 0; }}
QScrollBar::handle:horizontal {{
    background: {C['line_strong']}; border-radius: 4px; min-width: 28px;
}}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

/* ══════════ 分隔线 ══════════ */
QFrame[role="separator"] {{ background: {C['line_soft']}; border: none; max-height: 1px; }}

/* ══════════ 提示气泡 ══════════ */
QToolTip {{
    {sans(12)}
    color: {C['chip_fg']};
    background: {C['chip']};
    border: none;
    border-radius: {RADIUS['md']}px;
    padding: 5px 8px;
}}
"""


#: 供测试/对账用的令牌原件（勿就地修改）。
TOKENS = dict(C)

#: 本应用用到的**全部按钮变体**（`None` = 不设 `variant` 属性的默认档）。
#: ⚠ 新增变体时**必须**加进来，并且要在 QSS 里给它配一条 `:disabled` ——
#: 见上面 secondary 那段的说明与 `test_disabled_buttons_look_disabled`。
BUTTON_VARIANTS = [None, "outline", "secondary", "ghost", "toggle", "primaryAction"]
