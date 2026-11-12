"""sim_stack argument handling and bookkeeping. Never starts a sim: only usage, status and down
with nothing running."""
import os
import subprocess

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim_stack")


def run(*args, state_dir):
    env = dict(os.environ, HALO_STACK_DIR=str(state_dir))
    return subprocess.run([SCRIPT, *args], capture_output=True, text=True, env=env, timeout=60)


@pytest.mark.parametrize("args", [[], ["bogus"], ["up", "--bogus"], ["up", "--gui", "extra"]])
def test_bad_arguments_print_usage_and_exit_2(args, tmp_path):
    result = run(*args, state_dir=tmp_path)
    assert result.returncode == 2
    assert "sim_stack up" in result.stdout


def test_status_with_nothing_started(tmp_path):
    result = run("status", state_dir=tmp_path)
    assert result.returncode == 0
    assert "nothing running" in result.stdout


def test_status_reports_a_dead_pid_as_stopped(tmp_path):
    (tmp_path / "ui.pid").write_text("999999")  # no such process
    result = run("status", state_dir=tmp_path)
    assert "ui: stopped" in result.stdout


def test_script_never_arms_or_moves_the_vehicle():
    text = open(SCRIPT).read()
    for word in ("sitl_vehicle arm", "takeoff", "offboard", "/dev/tty"):
        assert word not in text.replace("It never arms, takes off", "")
