# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Tests for infra.tailscale.network_monitor.

Every modem/connectivity probe funnels through the module-level ``_run`` shell
wrapper, so the readers are exercised with ``_run`` patched to a command-
dispatching stub returning ``(exit_code, stdout)``. ``_check_modem_status``'s
merge logic is tested with the individual query methods stubbed; ``_run`` itself
is tested against a faked ``subprocess.run``.
"""

from __future__ import annotations

from types import SimpleNamespace

from infra.tailscale import network_monitor as nm
from infra.tailscale.network_monitor import (
    ModemStatus,
    NetworkMonitor,
    SignalQuality,
    _rsrp_to_percent,
    _rsrp_to_quality,
)


def make_run(rules, default=(1, "")):
    """rules: list of (needle|predicate, (code, stdout)); first match wins."""

    def run(cmd, timeout=10.0):
        joined = " ".join(cmd)
        for needle, out in rules:
            ok = needle(cmd) if callable(needle) else (needle in joined)
            if ok:
                return out
        return default

    return run


# --------------------------------------------------------------------------- #
# pure helpers
# --------------------------------------------------------------------------- #


def test_rsrp_to_quality_bands():
    assert _rsrp_to_quality(None) is SignalQuality.NO_SIGNAL
    assert _rsrp_to_quality(-70) is SignalQuality.EXCELLENT
    assert _rsrp_to_quality(-85) is SignalQuality.GOOD
    assert _rsrp_to_quality(-95) is SignalQuality.FAIR
    assert _rsrp_to_quality(-105) is SignalQuality.POOR
    assert _rsrp_to_quality(-120) is SignalQuality.NO_SIGNAL


def test_rsrp_to_percent_clamps():
    assert _rsrp_to_percent(None) == 0
    assert _rsrp_to_percent(-140) == 0
    assert _rsrp_to_percent(-44) == 100
    assert _rsrp_to_percent(-200) == 0  # below range -> clamped low


def test_modem_status_to_dict():
    d = ModemStatus(connected=True, signal_strength_dbm=-85, signal_quality=SignalQuality.GOOD).to_dict()
    assert d["connected"] is True
    assert d["signal_quality"] == "good"
    assert d["signal_strength_dbm"] == -85
    assert set(d) >= {"carrier", "interface", "imei", "apn", "nm_connection_state"}


# --------------------------------------------------------------------------- #
# _run
# --------------------------------------------------------------------------- #


def test_run_returns_code_and_stdout(monkeypatch):
    monkeypatch.setattr(nm.subprocess, "run", lambda *a, **k: SimpleNamespace(returncode=0, stdout="hi"))
    assert nm._run(["echo", "hi"]) == (0, "hi")


def test_run_missing_binary_returns_127(monkeypatch):
    def boom(*a, **k):
        raise FileNotFoundError("nmcli")

    monkeypatch.setattr(nm.subprocess, "run", boom)
    assert nm._run(["nmcli"]) == (127, "")


def test_run_generic_error_returns_1(monkeypatch):
    def boom(*a, **k):
        raise OSError("kaboom")

    monkeypatch.setattr(nm.subprocess, "run", boom)
    assert nm._run(["ping"]) == (1, "")


# --------------------------------------------------------------------------- #
# _ping_host
# --------------------------------------------------------------------------- #


def test_ping_host_unreachable(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("ping", (1, ""))]))
    assert NetworkMonitor()._ping_host("8.8.8.8") is None


def test_ping_host_parses_rtt(monkeypatch):
    out = "rtt min/avg/max/mdev = 10.5/15.2/20.1/3.2 ms"
    monkeypatch.setattr(nm, "_run", make_run([("ping", (0, out))]))
    assert NetworkMonitor()._ping_host("8.8.8.8") == 15.2


def test_ping_host_averages_time_lines(monkeypatch):
    out = "64 bytes time=10.0 ms\n64 bytes time=20.0 ms\n"
    monkeypatch.setattr(nm, "_run", make_run([("ping", (0, out))]))
    assert NetworkMonitor()._ping_host("8.8.8.8") == 15.0


def test_ping_host_no_match_returns_none(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("ping", (0, "nothing useful"))]))
    assert NetworkMonitor()._ping_host("8.8.8.8") is None


# --------------------------------------------------------------------------- #
# _guess_modem_interface
# --------------------------------------------------------------------------- #


def test_guess_interface_found(monkeypatch):
    out = "1: lo    inet 127.0.0.1/8 scope host\n3: wwan0    inet 10.50.10.2/30 scope global\n"
    monkeypatch.setattr(nm, "_run", make_run([("ip", (0, out))]))
    assert NetworkMonitor()._guess_modem_interface() == ("wwan0", "10.50.10.2")


def test_guess_interface_none_when_no_wan(monkeypatch):
    # A too-short line is skipped; the valid one is loopback, not a WAN iface.
    out = "shortline\n1: lo    inet 127.0.0.1/8 scope host\n"
    monkeypatch.setattr(nm, "_run", make_run([("ip", (0, out))]))
    assert NetworkMonitor()._guess_modem_interface() == (None, None)


def test_guess_interface_command_fails(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("ip", (1, ""))]))
    assert NetworkMonitor()._guess_modem_interface() == (None, None)


# --------------------------------------------------------------------------- #
# _query_nm_connection
# --------------------------------------------------------------------------- #

_NM_LIST = "NOMAD-LTE:gsm:wwan0:activated\nWired:ethernet:eth0:activated\n"
_NM_DETAILS = (
    "gsm.apn:internet\n"
    "IP4.ADDRESS[1]:192.0.2.10/24\n"
    "GENERAL.DEVICES:wwan0\n"
    "GENERAL.STATE:100 (activated)\n"
    "IP6.ADDRESS[1]:\n"  # blank value -> skipped
)


def test_query_nm_connection_full(monkeypatch):
    monkeypatch.setattr(
        nm,
        "_run",
        make_run([("NAME,TYPE,DEVICE,STATE", (0, _NM_LIST)), ("gsm.apn,GENERAL.STATE", (0, _NM_DETAILS))]),
    )
    info = NetworkMonitor()._query_nm_connection("NOMAD-LTE")
    assert info["name"] == "NOMAD-LTE"
    assert info["state"] == "activated"
    assert info["interface"] == "wwan0"
    assert info["ip4"] == "192.0.2.10"
    assert info["apn"] == "internet"


def test_query_nm_connection_list_fails(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("NAME,TYPE,DEVICE,STATE", (1, ""))]))
    assert NetworkMonitor()._query_nm_connection("NOMAD-LTE") is None


def test_query_nm_connection_name_absent(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("NAME,TYPE,DEVICE,STATE", (0, "Wired:ethernet:eth0:activated\n"))]))
    assert NetworkMonitor()._query_nm_connection("NOMAD-LTE") is None


def test_query_nm_connection_details_fail_keeps_basics(monkeypatch):
    monkeypatch.setattr(
        nm,
        "_run",
        make_run([("NAME,TYPE,DEVICE,STATE", (0, _NM_LIST)), ("gsm.apn,GENERAL.STATE", (1, ""))]),
    )
    info = NetworkMonitor()._query_nm_connection("NOMAD-LTE")
    assert info == {"name": "NOMAD-LTE", "state": "activated", "interface": "wwan0"}


# --------------------------------------------------------------------------- #
# _query_nm_connection_fuzzy
# --------------------------------------------------------------------------- #


def test_fuzzy_prefers_cellular_active(monkeypatch):
    listing = "MyModem:gsm:wwan0:activated\nLTE-thing:ethernet:eth1:activated\n"
    calls = {}

    def fake_query(name):
        calls["name"] = name
        return {"name": name}

    m = NetworkMonitor()
    monkeypatch.setattr(nm, "_run", make_run([("NAME,TYPE,DEVICE,STATE", (0, listing))]))
    monkeypatch.setattr(m, "_query_nm_connection", fake_query)
    assert m._query_nm_connection_fuzzy() == {"name": "MyModem"}
    assert calls["name"] == "MyModem"


def test_fuzzy_falls_back_to_named_active_then_inactive(monkeypatch):
    m = NetworkMonitor()
    monkeypatch.setattr(m, "_query_nm_connection", lambda name: {"name": name})

    # No cellular type, but an activated LTE-named ethernet (CDC-ECM case).
    monkeypatch.setattr(nm, "_run", make_run([("NAME", (0, "LTE-ECM:ethernet:eth1:activated\n"))]))
    assert m._query_nm_connection_fuzzy() == {"name": "LTE-ECM"}

    # Only an inactive LTE-named profile.
    monkeypatch.setattr(nm, "_run", make_run([("NAME", (0, "modem-backup:ethernet:eth2:\n"))]))
    assert m._query_nm_connection_fuzzy() == {"name": "modem-backup"}


def test_fuzzy_none_when_nothing_matches(monkeypatch):
    # A malformed (<4 field) line is skipped; the valid line is not LTE-related.
    listing = "malformed-line\nWired:ethernet:eth0:activated\n"
    monkeypatch.setattr(nm, "_run", make_run([("NAME", (0, listing))]))
    assert NetworkMonitor()._query_nm_connection_fuzzy() is None


def test_fuzzy_list_fails(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("NAME", (1, ""))]))
    assert NetworkMonitor()._query_nm_connection_fuzzy() is None


# --------------------------------------------------------------------------- #
# _query_modemmanager / _read_modem
# --------------------------------------------------------------------------- #

_MM_SHOW = (
    "  Status | state: 'connected'\n"
    "  Hardware | model: Quectel EM05\n"
    "           | access tech: lte\n"
    "  3GPP | operator name: Rogers\n"
    "       | imei: 359123456789012\n"
    "  System | primary port: wwan0\n"
)


def test_query_modemmanager_no_modems(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("mmcli -L", (0, "No modems were found"))]))
    assert NetworkMonitor()._query_modemmanager() is None


def test_query_modemmanager_list_fails(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("mmcli -L", (1, ""))]))
    assert NetworkMonitor()._query_modemmanager() is None


def test_query_modemmanager_reads_first_modem(monkeypatch):
    rules = [
        ("mmcli -L", (0, "/org/freedesktop/ModemManager1/Modem/0\n")),
        ("--signal-get", (0, "rsrp: -85.00 dBm\n")),
        ("-m 0", (0, _MM_SHOW)),
    ]
    monkeypatch.setattr(nm, "_run", make_run(rules))
    info = NetworkMonitor()._query_modemmanager()
    assert info["connected"] is True
    assert info["model"] == "Quectel EM05"
    assert info["technology"] == "lte"
    assert info["carrier"] == "Rogers"
    assert info["imei"] == "359123456789012"
    assert info["interface"] == "wwan0"
    assert info["rsrp_dbm"] == -85


def test_query_modemmanager_skips_empty_modem(monkeypatch):
    rules = [
        ("mmcli -L", (0, "/Modem/0\n")),
        ("-m 0", (1, "")),  # _read_modem returns None
    ]
    monkeypatch.setattr(nm, "_run", make_run(rules))
    assert NetworkMonitor()._query_modemmanager() is None


def test_read_modem_command_fails(monkeypatch):
    monkeypatch.setattr(nm, "_run", make_run([("-m 0", (1, ""))]))
    assert NetworkMonitor()._read_modem("0") is None


def test_read_modem_skips_unset_values_and_uses_signal_fallback(monkeypatch):
    show = "  Status | state: 'registered'\n  Hardware | model: --\n  System | signal quality: 50% (recent)\n"
    rules = [
        ("--signal-get", (0, "no rsrp here")),  # no rsrp -> fall back to percent
        ("--bearer", (0, "  Bearer | interface: wwan1\n")),
        ("-m 0", (0, show)),
    ]
    monkeypatch.setattr(nm, "_run", make_run(rules))
    info = NetworkMonitor()._read_modem("0")
    assert info["connected"] is False  # 'registered' != 'connected'
    assert "model" not in info  # "--" is skipped
    assert info["rsrp_dbm"] == round(50 / 100.0 * 96 - 140)  # -92
    assert info["interface"] == "wwan1"  # filled from the bearer fallback


def test_read_modem_returns_none_when_empty(monkeypatch):
    # Nothing parseable and signal-get fails -> info stays empty -> None.
    rules = [("--signal-get", (1, "")), ("--bearer", (1, "")), ("-m 0", (0, "  Modem | path: /Modem/0\n"))]
    monkeypatch.setattr(nm, "_run", make_run(rules))
    assert NetworkMonitor()._read_modem("0") is None


# --------------------------------------------------------------------------- #
# _check_modem_status (merge logic; sub-queries stubbed)
# --------------------------------------------------------------------------- #


def _monitor_with_queries(monkeypatch, nm_info=None, fuzzy=None, mm_info=None, guess=(None, None)):
    m = NetworkMonitor()
    monkeypatch.setattr(m, "_query_nm_connection", lambda cand: nm_info)
    monkeypatch.setattr(m, "_query_nm_connection_fuzzy", lambda: fuzzy)
    monkeypatch.setattr(m, "_query_modemmanager", lambda: mm_info)
    monkeypatch.setattr(m, "_guess_modem_interface", lambda: guess)
    return m


def test_check_modem_status_none_when_nothing(monkeypatch):
    assert _monitor_with_queries(monkeypatch)._check_modem_status() is None


def test_check_modem_status_merges_nm_and_mm(monkeypatch):
    nm_info = {"name": "NOMAD-LTE", "state": "activated", "interface": "wwan0", "ip4": "10.0.0.2", "apn": "net"}
    mm_info = {"connected": False, "model": "EM05", "carrier": "Rogers", "rsrp_dbm": -85}
    status = _monitor_with_queries(monkeypatch, nm_info=nm_info, mm_info=mm_info)._check_modem_status()
    assert status.connected is True  # NM activated wins over mm "disconnected"
    assert status.model == "EM05"
    assert status.signal_quality is SignalQuality.GOOD
    assert status.signal_percent == _rsrp_to_percent(-85)


def test_check_modem_status_uses_fuzzy_when_named_absent(monkeypatch):
    fuzzy = {"name": "LTE-ECM", "state": "activated", "interface": "eth1"}
    status = _monitor_with_queries(monkeypatch, nm_info=None, fuzzy=fuzzy)._check_modem_status()
    assert status.nm_connection_name == "LTE-ECM"
    assert status.connected is True


def test_check_modem_status_mm_only_sets_connected(monkeypatch):
    mm_info = {"connected": True, "model": "EM05", "interface": "wwan0"}
    status = _monitor_with_queries(monkeypatch, mm_info=mm_info)._check_modem_status()
    assert status.connected is True
    assert status.interface == "wwan0"


def test_check_modem_status_guesses_interface_when_missing(monkeypatch):
    mm_info = {"connected": True, "model": "EM05"}  # no interface/ip
    status = _monitor_with_queries(monkeypatch, mm_info=mm_info, guess=("usb0", "10.9.9.9"))._check_modem_status()
    assert status.interface == "usb0"
    assert status.ip_address == "10.9.9.9"


# --------------------------------------------------------------------------- #
# check_connectivity
# --------------------------------------------------------------------------- #


def test_check_connectivity_with_gcs(monkeypatch):
    m = NetworkMonitor(gcs_tailscale_ip="100.64.0.1")
    monkeypatch.setattr(m, "_check_modem_status", lambda: ModemStatus(connected=True))
    monkeypatch.setattr(m, "_ping_host", lambda host, **k: 12.0)
    status = m.check_connectivity()
    assert status.internet_reachable is True
    assert status.tailscale_reachable is True
    assert status.gcs_ip == "100.64.0.1"
    assert m.status is status


def test_check_connectivity_without_gcs_and_offline(monkeypatch):
    m = NetworkMonitor()
    monkeypatch.setattr(m, "_check_modem_status", lambda: None)
    monkeypatch.setattr(m, "_ping_host", lambda host, **k: None)
    status = m.check_connectivity()
    assert status.internet_reachable is False
    assert status.tailscale_reachable is False
    assert status.gcs_ip is None


# --------------------------------------------------------------------------- #
# ctor + lifecycle
# --------------------------------------------------------------------------- #


def test_ctor_connection_candidates(monkeypatch):
    monkeypatch.delenv("NOMAD_LTE_CONNECTION", raising=False)
    assert NetworkMonitor()._nm_conn_candidates == ["NOMAD-LTE", "LTE-ECM"]
    assert NetworkMonitor(nm_connection_name="Custom")._nm_conn_candidates == ["Custom"]
    monkeypatch.setenv("NOMAD_LTE_CONNECTION", "FromEnv")
    assert NetworkMonitor()._nm_conn_candidates == ["FromEnv"]


def test_status_property_default():
    assert NetworkMonitor().status.internet_reachable is False


def test_start_runs_loop_then_stop(monkeypatch):
    m = NetworkMonitor(check_interval=0.0)
    calls = {"n": 0}

    def fake_check():
        calls["n"] += 1
        m._stop_event.set()

    monkeypatch.setattr(m, "check_connectivity", fake_check)
    m.start()
    m._thread.join(timeout=2.0)
    m.stop()
    assert calls["n"] >= 1


def test_monitor_loop_swallows_errors(monkeypatch):
    m = NetworkMonitor()
    calls = {"n": 0}

    def fake_check():
        calls["n"] += 1
        m._stop_event.set()
        raise RuntimeError("probe blew up")

    monkeypatch.setattr(m, "check_connectivity", fake_check)
    m._monitor_loop()  # error caught; loop exits on the stop event
    assert calls["n"] == 1


def test_start_idempotent_when_alive():
    m = NetworkMonitor()
    sentinel = SimpleNamespace(is_alive=lambda: True)
    m._thread = sentinel
    m.start()
    assert m._thread is sentinel


def test_stop_joins_live_thread():
    m = NetworkMonitor()
    joined = []
    m._thread = SimpleNamespace(is_alive=lambda: True, join=lambda timeout: joined.append(timeout))
    m.stop()
    assert joined == [2.0]
    assert m._stop_event.is_set()


def test_stop_without_thread_is_noop():
    NetworkMonitor().stop()
