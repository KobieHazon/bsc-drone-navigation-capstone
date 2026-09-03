#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
required = [
    ROOT / "src" / "main_code.py",
    ROOT / "src" / "drone_controller.py",
    ROOT / "src" / "tello_slam_control.py",
    ROOT / "src" / "tag_manager.py",
    ROOT / "src" / "project_gui.py",
    ROOT / "docs" / "ROS_Diagram.png",
    ROOT / "docs" / "ROS_Diagram.drawio",
]
missing = [str(path.relative_to(ROOT)) for path in required if not path.exists()]
if missing:
    print("Missing required files: " + ", ".join(missing), file=sys.stderr)
    sys.exit(1)

for forbidden in ROOT.rglob("*"):
    rel = forbidden.relative_to(ROOT).as_posix()
    if any(part.startswith("._") for part in forbidden.parts):
        print(f"Apple metadata file is tracked candidate: {rel}", file=sys.stderr)
        sys.exit(1)
    if forbidden.suffix in {".pyc", ".mov", ".MOV", ".mp4", ".MP4"}:
        print(f"Forbidden generated or heavy artifact: {rel}", file=sys.stderr)
        sys.exit(1)
    if forbidden.name in {"all of us.png", "itzchak.jpeg", "marzim.png"}:
        print(f"People image should not be staged: {rel}", file=sys.stderr)
        sys.exit(1)

text_files = [p for p in ROOT.rglob("*") if p.is_file() and p.suffix.lower() in {".py", ".md", ".csv", ".txt", ".drawio", ""}]
combined = "\n".join(p.read_text(encoding="utf-8", errors="ignore") for p in text_files)
markers = [
    "".join(["208", "234", "161"]),
    "/" + "Users" + "/",
    "/" + "home" + "/" + "kobie",
    "/" + "home" + "/" + "arkadiros",
]
if any(marker in combined for marker in markers):
    print("Privacy or machine-path marker found in tracked text", file=sys.stderr)
    sys.exit(1)

required_markers = ["rospy", "AprilTagDetectionArray", "ORB_SLAM", "PointCloud2", "TelloLoadTrajectory"]
missing_markers = [marker for marker in required_markers if marker not in combined]
if missing_markers:
    print("Missing expected ROS/drone markers: " + ", ".join(missing_markers), file=sys.stderr)
    sys.exit(1)

print("Repository static checks passed.")
