"""界面设置的持久化（角色 / 地址 / 端口 / arm_id / 夹爪）。

⚠ **不写进本仓**（配置不是源码）—— 落在用户配置目录：
`~/.config/litearm-teleop-isomorphic/settings.json`。
读失败**不抛**（配置文件坏了不该挡住启动），回默认值并记一条日志。
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict

log = logging.getLogger("liteteleop.settings")

__all__ = ["Settings", "settings_path", "load_settings", "save_settings"]


def settings_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.join(Path.home(), ".config")
    return Path(base) / "litearm-teleop-isomorphic" / "settings.json"


@dataclass
class Settings:
    role: str = "master"                 # master | slave
    peer: str = "127.0.0.1"              # 从臂填：主臂的 IP
    jport: int = 17447                   # zenoh 监听/连接端口
    arm_id: str = "armA"                 # 决定 topic：litearm/v4/{arm_id}/teleop
    cdc_port: str = ""                   # 空 = 按 VID:PID 自动找
    confirmed: bool = False              # ☑ 安全确认（**不持久化**到 True 才是默认安全）
    # ── 夹爪遥操（与臂侧**完全独立**，spec §2 铁律第 3 条：不读臂的字段）──
    gcan: str = ""                       # 夹爪 CAN 通道；空 = 不启用夹爪遥操（默认关）
    gpeer: str = "127.0.0.1"             # 从端填：主端 IP（**独立字段**）
    gport: int = 17448                   # 夹爪 zenoh 端口（独立 session ⇒ 必须独立端口）
    grip_id: str = "gripA"               # 决定夹爪 topic
    # 从端夹爪的力矩上限（Nm）：超过就卸力，护住打印件。0 = 关闭该保护。
    # ⚠ 这是 **per-machine 的值** —— 取决于打印件有多脆，不是夹爪型号。
    grip_torque_limit_nm: float = 1.0
    extra: Dict[str, Any] = field(default_factory=dict)


def load_settings() -> Settings:
    p = settings_path()
    try:
        raw = json.loads(p.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return Settings()
    except Exception as e:                                # noqa: BLE001
        log.warning("读设置失败（用默认值）: %s", e)
        return Settings()
    known = {f for f in Settings.__dataclass_fields__}
    kwargs = {k: v for k, v in raw.items() if k in known}
    # ⚠ 安全确认**不跨会话保留** —— 每次启动都要重新勾
    kwargs["confirmed"] = False
    try:
        return Settings(**kwargs)
    except TypeError as e:
        log.warning("设置字段不合法（用默认值）: %s", e)
        return Settings()


def save_settings(s: Settings) -> None:
    p = settings_path()
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        d = asdict(s)
        d.pop("confirmed", None)                          # 不持久化
        p.write_text(json.dumps(d, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception as e:                                # noqa: BLE001
        log.warning("写设置失败: %s", e)
