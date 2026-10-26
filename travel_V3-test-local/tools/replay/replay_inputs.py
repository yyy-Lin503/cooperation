"""Resolve portable replay manifests without rewriting archived evidence."""
from pathlib import Path, PureWindowsPath


def basename(value):
    return PureWindowsPath(str(value)).name


def resolve_input(value, manifest, data_root=None, kind=None):
    path = Path(value)
    name = basename(value)
    candidates = []
    if data_root is not None:
        root = Path(data_root).resolve()
        folders = {"bin": ("recordings/radar", "bin", ""), "diagnostics": ("diagnostics", "tracking", "tracking_diagnostics", ""),
                   "cfg": ("config/radar", "config", "")}.get(kind, ("",))
        candidates += [root / folder / name for folder in folders]
    if path.is_absolute():
        candidates.append(path)
    elif not PureWindowsPath(str(value)).is_absolute():
        candidates.append(Path(manifest).resolve().parent / path)
    if kind == "cfg":
        candidates.append(Path(__file__).resolve().parents[2] / "config/radar" / name)
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"Cannot resolve {kind or 'input'}: {value}. Set --data-root or update a copy of the manifest.")
