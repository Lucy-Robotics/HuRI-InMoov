"""Shipped code uses only the part of huri-core's public surface it needs: the
bridge is a client, so the client SDK, the event base classes and the client
config (huri-core records its public surface in tests/core/public_api.txt)."""

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
ALLOWED = {
    "bridge/huri_lucy_bridge": {
        "huri_core.core.client",
        "huri_core.core.events",
        "huri_core.core.dataclasses.config",
    },
}


def is_allowed(name: str, allowed: set) -> bool:
    # A parent package (`from huri_core.core import client`) is fine too.
    return any(
        name == a or name.startswith(a + ".") or a.startswith(name + ".")
        for a in allowed
    )


def violations(rel: str, source: str, allowed: set) -> set:
    """huri-core imports outside `allowed`, at any depth of the file."""
    names = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            names += [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.append(node.module)
            names += [f"{node.module}.{alias.name}" for alias in node.names]
    return {
        f"{rel} imports {name}"
        for name in names
        if name.split(".")[0] == "huri_core" and not is_allowed(name, allowed)
    }


def package_violations(package: str) -> set:
    found = set()
    for path in sorted((ROOT / package).rglob("*.py")):
        rel = str(path.relative_to(ROOT))
        found |= violations(rel, path.read_text(), ALLOWED[package])
    return found


@pytest.mark.parametrize("package", sorted(ALLOWED))
def test_every_checked_package_exists(package):
    # A moved or renamed package would otherwise pass with nothing to check.
    assert list((ROOT / package).rglob("*.py")), f"{package} has no Python file"


def test_the_bridge_passes_the_public_surface_check():
    found = package_violations("bridge/huri_lucy_bridge")
    assert not found, sorted(found)


@pytest.mark.parametrize(
    "file,module",
    [
        ("bridge/huri_lucy_bridge/app.py", "huri_core.core.plugins"),
        ("bridge/huri_lucy_bridge/audio.py", "huri_core.core.huri"),
        ("bridge/huri_lucy_bridge/session.py", "huri_core.core.manifest"),
    ],
)
def test_a_huri_core_internal_imported_by_shipped_code_fails_ci(file, module):
    package = "/".join(file.split("/")[:2])
    source = f"def late():\n    from {module} import something\n"
    assert violations(file, source, ALLOWED[package]) == {
        f"{file} imports {module}",
        f"{file} imports {module}.something",
    }
