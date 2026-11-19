"""Shared log locations for clients run from a checkout or installed package."""
import os
from pathlib import Path


def client_log_directory(client_type="carla"):
    explicit = os.environ.get("UB_CLIENT_LOG_DIR")
    if explicit:
        return Path(explicit).expanduser().resolve()
    root = os.environ.get("UB_CLIENT_LOG_ROOT")
    if root:
        root = Path(root).expanduser()
    else:
        repository = next((parent for parent in Path(__file__).resolve().parents
                           if (parent / "UB-MR").is_dir() and (parent / "CARLA").is_dir()), None)
        root = (repository or Path.cwd()) / "Client-Logs"
    group = "ub-mr" if client_type == "ub-mr" else "manual-control"
    return (root / group).resolve()
