#!/usr/bin/env python3
"""Build and run the MAVSDK movement test against PX4 SITL."""

from pathlib import Path
import shutil
import subprocess
import sys


IMAGE_NAME = "halo-movement"
BUILD_JOBS = "2"
PROJECT_DIR = Path(__file__).resolve().parent
DOCKERFILE = PROJECT_DIR / "Dockerfile.mavsdk"
PROGRAMS = {
    "movement": "/app/build/test_takeoff",
}


def run(command: list[str]) -> None:
    print(f">> {' '.join(command)}", flush=True)
    subprocess.run(command, check=True)


def main() -> int:
    program_name = sys.argv[1] if len(sys.argv) > 1 else "movement"
    if program_name not in PROGRAMS:
        valid_names = ", ".join(PROGRAMS)
        print(f"Unknown program '{program_name}'. Choose: {valid_names}")
        return 2

    if shutil.which("docker") is None:
        print("Docker was not found on PATH. Start Docker Desktop and try again.")
        return 1

    if not DOCKERFILE.exists():
        print(f"Missing Dockerfile: {DOCKERFILE}")
        return 1

    try:
        run(
            [
                "docker",
                "build",
                "-f",
                str(DOCKERFILE),
                "-t",
                IMAGE_NAME,
                "--build-arg",
                f"BUILD_JOBS={BUILD_JOBS}",
                str(PROJECT_DIR),
            ]
        )
        run(
            [
                "docker",
                "run",
                "--rm",
                "--network",
                "host",
                IMAGE_NAME,
                PROGRAMS[program_name],
            ]
        )
    except subprocess.CalledProcessError as error:
        print(f"Docker command failed with exit code {error.returncode}.")
        return error.returncode
    except KeyboardInterrupt:
        print("\nMovement test stopped.")
        return 130

    return 0


if __name__ == "__main__":
    sys.exit(main())
