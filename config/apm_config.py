"""APM Configuration Utilities

Provides a portable way to resolve the APM root directory without relying on
hardcoded absolute paths like '/home/arm1/APM'.

Resolution order:
1. Environment variable `APM_HOME` if set.
2. Walk up from this file's parent until a directory containing `_ADMIN` exists.
3. Fallback to current working directory.

Other modules should import `get_apm_root` and build paths relative to it.
"""

from __future__ import annotations
import os
from pathlib import Path

_CACHE: Path | None = None

def get_apm_root() -> Path:
    global _CACHE
    if _CACHE is not None:
        return _CACHE

    env_root = os.getenv("APM_HOME")
    if env_root:
        p = Path(env_root).expanduser().resolve()
        if p.exists():
            _CACHE = p
            return p

    # Attempt to locate by ascending parents
    current = Path(__file__).resolve().parent
    for parent in [current] + list(current.parents):
        if (parent / "_ADMIN").exists():
            _CACHE = parent
            return parent

    # Fallback
    _CACHE = Path.cwd().resolve()
    return _CACHE

def path(*parts: str) -> Path:
    """Join parts onto the resolved APM root."""
    return get_apm_root().joinpath(*parts)

def ensure_directories(*relative_dirs: str) -> None:
    for d in relative_dirs:
        p = path(d)
        p.mkdir(parents=True, exist_ok=True)

if __name__ == "__main__":
    print(f"APM root: {get_apm_root()}")