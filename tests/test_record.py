"""`record.py` —— 诊断 CSV 记录器（只在 `LITEARM_RECORD_DIR` 设置时启用）。

判据：环境变量未设置时**什么都不创建**；设置后四个文件各有表头，行能按时间戳读回。
"""
from __future__ import annotations

import csv

from liteteleop import record


def test_from_env_is_off_by_default(monkeypatch):
    monkeypatch.delenv("LITEARM_RECORD_DIR", raising=False)
    assert record.from_env(pid=1234) is None


def test_from_env_creates_four_files_with_headers(tmp_path, monkeypatch):
    monkeypatch.setenv("LITEARM_RECORD_DIR", str(tmp_path))
    rec = record.from_env(pid=4321)
    try:
        assert set(rec) == {"state", "pub", "rx", "loop"}
        rec["pub"].row([0.1] * 7 + [0.2] * 7, t=1.5)
        rec["rx"].row([9.0] + [0.1] * 7 + [0.2] * 7, t=1.6)
        files = sorted(p.name for p in tmp_path.glob("*.csv"))
        assert files == ["4321-master_pub.csv", "4321-slave_loop.csv",
                         "4321-slave_rx.csv", "4321-state.csv"]
    finally:
        for r in rec.values():
            r.close()

    with open(tmp_path / "4321-master_pub.csv", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == record.PUB_HEADER
    assert len(rows) == 2
    assert float(rows[1][0]) == 1.5
    assert len(rows[1]) == 1 + 14


def _fake_state(q, *, tau=None, t_mos=30.0):
    n = len(q)
    return type("S", (), {
        "q": list(q), "dq": [0.0] * n, "tau": list(tau if tau is not None else [0.0] * n),
        "joints": [type("J", (), {"t_mos": t_mos, "t_coil": 0.0, "err": 0})() for _ in q],
        "mode_name": "MOVE_J", "enabled": True, "faulted": False,
        "joint_fault": 0, "flags": 0, "flag_names": [], "seq": 7,
    })()


def test_state_row_width_matches_header_and_tau_lands_in_its_column(tmp_path):
    """⚠ CSV 错位是**静默**的：多写/少写一列不报错，只有读回时才发现各列都错一格。

    这里直接跑 worker 的真实写法（`ArmWorker._push`），而不是只比对常量表。
    """
    from liteteleop.arm_worker import ArmWorker

    w = ArmWorker(on_state=None)
    w._rec = {"state": record.Recorder(tmp_path / "state.csv", record.STATE_HEADER)}
    w._push(_fake_state([0.5] * 7, tau=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0], t_mos=42.0))
    w._rec["state"].close()

    with open(tmp_path / "state.csv", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == record.STATE_HEADER
    assert len(rows[1]) == 1 + len(record.STATE_HEADER) - 1
    row = dict(zip(rows[0], rows[1]))
    assert [float(row[f"tau{i}"]) for i in range(7)] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert all(float(row[f"tmos{i}"]) == 42.0 for i in range(7))
    assert float(row["q0"]) == 0.5 and int(row["seq"]) == 7


def test_state_header_matches_what_the_worker_writes():
    # `_push` 写 7 q + 7 dq + 7 tau + 7 tmos + enabled + flags + joint_fault + seq
    assert record.STATE_HEADER == (
        ["t"] + [f"q{i}" for i in range(7)] + [f"dq{i}" for i in range(7)]
        + [f"tau{i}" for i in range(7)] + [f"tmos{i}" for i in range(7)]
        + ["enabled", "flags", "joint_fault", "seq"])
