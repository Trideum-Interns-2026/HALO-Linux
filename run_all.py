#!/usr/bin/env python3
"""
Build and run one of the HALO MAVSDK programs.

Works in two different environments, auto-detected:

- Inside the dev container: builds directly with cmake/make (using the
  MAVSDK SDK already installed in the dev container's image) and runs the
  resulting binary. No Docker calls at all in this mode.
- From the WSL2 host (outside any container): falls back to building a
  throwaway Docker image via Movement/Dockerfile.mavsdk and running it
  with --network host against whatever SITL instance is currently running.

Usage:
    python3 run_all.py <folder> <file.cpp>

Example:
    python3 run_all.py Movement test_takeoff.cpp
    python3 run_all.py Tracking_Geolocation takeoff_forward_back.cpp
"""

from pathlib import Path
from typing import List, Optional
import shutil
import subprocess
import sys


IMAGE_NAME = "halo-mavsdk"
REPO_DIR = Path(__file__).resolve().parent
DOCKERFILE = REPO_DIR / "Movement" / "Dockerfile.mavsdk"


def running_inside_container() -> bool:
    return Path("/.dockerenv").exists()


def run(command: List[str], cwd: Optional[Path] = None) -> None:
    print(f">> {' '.join(command)}", flush=True)
    subprocess.run(command, check=True, cwd=cwd)


def find_source(folder: Path, file_name: str) -> Optional[Path]:
    folder_path = REPO_DIR / folder
    matches = list(folder_path.rglob(file_name)) if folder_path.is_dir() else []
    if not matches:
        return None
    if len(matches) > 1:
        choices = "\n".join(f"  {p.relative_to(REPO_DIR)}" for p in matches)
        print(f"Multiple C++ files named '{file_name}' were found:\n{choices}")
        print("Use a folder that contains only one match.")
        return None
    return matches[0]


def build_and_run_in_container(source_path: Path) -> int:
    """Build directly with cmake/make — assumes MAVSDK is already
    installed in this environment (true inside the dev container)."""
    project_dir = source_path.parent
    build_dir = project_dir / "build"

    if not (project_dir / "CMakeLists.txt").exists():
        print(f"No CMakeLists.txt found in {project_dir}")
        return 1

    build_dir.mkdir(exist_ok=True)

    try:
        run(["cmake", ".."], cwd=build_dir)
        run(["make"], cwd=build_dir)
    except subprocess.CalledProcessError as error:
        print(f"Build failed with exit code {error.returncode}.")
        return error.returncode

    executable = build_dir / source_path.stem
    if not executable.exists():
        print(f"Build succeeded but expected executable not found: {executable}")
        return 1

    try:
        run([str(executable)])
    except subprocess.CalledProcessError as error:
        print(f"Program exited with code {error.returncode}.")
        return error.returncode
    except KeyboardInterrupt:
        print("\nMission stopped.")
        return 130

    return 0


def build_and_run_via_docker(source_path: Path) -> int:
    """Fallback for running from the WSL2 host, outside any container:
    build a throwaway image with MAVSDK and run it with host networking."""
    if shutil.which("docker") is None:
        print("Docker was not found on PATH. Start Docker Desktop and try again.")
        return 1

    if not DOCKERFILE.exists():
        print(f"Missing Dockerfile: {DOCKERFILE}")
        return 1

    program_path = f"/app/build/{source_path.stem}"

    try:
        run([
            "docker", "build", "-f", str(DOCKERFILE), "-t", IMAGE_NAME,
            "--build-arg", "BUILD_JOBS=2", str(REPO_DIR),
        ])
        run([
            "docker", "run", "--rm", "--network", "host", IMAGE_NAME,
            program_path,
        ])
    except subprocess.CalledProcessError as error:
        print(f"Docker command failed with exit code {error.returncode}.")
        return error.returncode
    except KeyboardInterrupt:
        print("\nMission stopped.")
        return 130

    return 0


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: python3 run_all.py <folder> <file.cpp>")
        print("Example: python3 run_all.py Movement test_takeoff.cpp")
        return 2

    folder = Path(sys.argv[1])
    file_name = Path(sys.argv[2]).name
    if Path(file_name).suffix != ".cpp":
        print(f"Not a .cpp file: {file_name}")
        return 2

    source_path = find_source(folder, file_name)
    if source_path is None:
        print(f"C++ file not found: {folder / file_name}")
        return 2

    if running_inside_container():
        print(">> Detected dev container — building directly with cmake/make.")
        return build_and_run_in_container(source_path)
    else:
        print(">> Detected host environment — building via throwaway Docker image.")
        return build_and_run_via_docker(source_path)


if __name__ == "__main__":
    sys.exit(main())