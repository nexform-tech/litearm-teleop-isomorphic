"""夹爪那 4 个设置字段 —— 独立、且默认不启用（spec §9.1）。"""
from liteteleop.settings import DEFAULT_PEER, Settings


def test_gripper_defaults_are_off_and_independent():
    s = Settings()
    assert s.gcan == "", "默认必须是「不启用」"
    assert s.gport == 17448
    assert s.grip_id == "gripA"
    assert s.gpeer == DEFAULT_PEER
    assert s.grip_torque_limit_nm == 0.0, "默认关闭力矩判定（纯位置跟随）"


def test_peer_defaults_are_loopback():
    """⚠ 主从两臂、夹爪都接在**同一台电脑**上 ⇒ 默认回环。

    把固定网段的 `192.168.x.y` 写进默认值，换一个网段就整条链路收不到帧
    （zenoh 严格点对点，帧不会自动改道）。只有两个进程分跑两台机器时才填地址。
    判别力：把 `Settings.peer` / `gpeer` 的默认值改成任何非回环值时本用例红。
    """
    s = Settings()
    assert DEFAULT_PEER == "127.0.0.1"
    assert s.peer == "127.0.0.1"
    assert s.gpeer == "127.0.0.1"


def test_gripper_fields_do_not_read_the_arm_fields():
    """⚠ spec §2 铁律第 3 条：夹爪字段**不读**臂的同名字段。

    改臂的 `peer` / `arm_id` / `jport` 不得影响夹爪的。
    """
    s = Settings(peer="10.0.0.9", arm_id="armZ", jport=1234)
    assert s.gpeer == DEFAULT_PEER
    assert s.grip_id == "gripA"
    assert s.gport == 17448
    assert s.gcan == ""


def test_gripper_defaults_survive_a_settings_roundtrip(tmp_path, monkeypatch):
    """⚠ `gcan` 默认空 = 安全默认。落盘再读回**不得**变成 `can0`。

    （spec §9.1 的 ⚠：界面**不得**在显示时把 `can0` 写进 `gcan`，
    否则「默认不启用」会被静默变成「默认启用」。）
    """
    from liteteleop import settings as st

    p = tmp_path / "settings.json"
    monkeypatch.setattr(st, "settings_path", lambda: p)
    st.save_settings(st.Settings())
    assert st.load_settings().gcan == ""
