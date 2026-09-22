#!/usr/bin/env python3
"""
One-shot launcher for the PX4 SITL Docker setup — works on both a
Windows/WSL2 machine and the NVIDIA DGX Spark, auto-detecting which
compose override to use.

Usage:
    python3 run.py                    # auto-detect platform, launch gazebo-classic_typhoon_h480
    python3 run.py <other_target>     # launch a different vehicle model
    python3 run.py bash               # skip launching, just get a shell
    python3 run.py --platform spark   # force a specific override
"""

import argparse
import platform
import subprocess
import sys


def detect_override():
    system = platform.system()
    machine = platform.machine().lower()

    if system == "Linux" and machine in ("aarch64", "arm64"):
        return "spark"

    if system == "Linux":
        # Distinguish WSL2 (has GPU-accelerated WSLg display) from a
        # generic Linux box, by checking the kernel version string that
        # WSL2 always includes.
        try:
            with open("/proc/version") as f:
                if "microsoft" in f.read().lower():
                    return "wsl"
        except FileNotFoundError:
            pass

    return "windows"


def run(cmd):
    print(f">> {' '.join(cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"!! Command failed with exit code {result.returncode}")
        sys.exit(result.returncode)


def docker_available():
    try:
        result = subprocess.run(
            ["docker", "info"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def running_inside_container():
    return Path("/.dockerenv").exists()


def main():
    if running_inside_container():
        print("!! You're running this INSIDE a container (likely the dev container).")
        print("!! run.py launches a container from the HOST — it can't run from")
        print("!! inside one. Open a WSL2/host terminal (not the VS Code dev")
        print("!! container terminal) and run this from there instead.")
        print("!!")
        print("!! If you're in the dev container and want to launch SITL directly")
        print("!! here, just run: fly")
        sys.exit(1)

    parser = argparse.ArgumentParser()
    parser.add_argument("target", nargs="?", default="gazebo-classic_typhoon_h480")
    parser.add_argument("--platform", choices=["windows", "spark", "wsl"], default=None)
    args = parser.parse_args()

    override = args.platform or detect_override()
    print(f">> Platform: {override}")

    print(">> Checking Docker is available...")
    if not docker_available():
        print("!! Docker doesn't seem to be running (or isn't installed).")
        sys.exit(1)

    compose_files = ["-f", "docker-compose.yml", "-f", f"docker-compose.{override}.yml"]

    print(">> Building image (skips layers that haven't changed)...")
    run(["docker", "compose"] + compose_files + ["build"])

    print(f">> Starting PX4 SITL container (target: {args.target})...")
    run(["docker", "compose"] + compose_files + ["run", "--rm", "px4-sitl", args.target])


if __name__ == "__main__":
    main()