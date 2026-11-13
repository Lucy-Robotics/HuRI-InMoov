"""Shipped code uses only the part of huri-core's public surface it needs: the
robot the client SDK, the module the module base classes (huri-core records
its public surface in tests/core/public_api.txt). Tests may load the module
the way the server does."""

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
ALLOWED = {
    "robot/huri_inmoov_robot": {
        "huri_core.core.client",
        "huri_core.core.events",
        "huri_core.core.dataclasses.config",
    },
    "module/huri_inmoov": {"huri_core.core.module", "huri_core.core.events"},
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


def test_the_robot_client_passes_the_public_surface_check():
    found = package_violations("robot/huri_inmoov_robot")
    assert not found, sorted(found)


def test_the_inmoov_module_reads_its_toml_without_a_huri_core_internal():
    found = package_violations("module/huri_inmoov")
    assert not found, sorted(found)


@pytest.mark.parametrize(
    "file,module",
    [
        ("robot/huri_inmoov_robot/app.py", "huri_core.core.plugins"),
        ("robot/huri_inmoov_robot/audio.py", "huri_core.core.huri"),
        ("module/huri_inmoov/retarget.py", "huri_core.core.manifest"),
    ],
)
def test_a_huri_core_internal_imported_by_shipped_code_fails_ci(file, module):
    package = "/".join(file.split("/")[:2])
    source = f"def late():\n    from {module} import something\n"
    assert violations(file, source, ALLOWED[package]) == {
        f"{file} imports {module}",
        f"{file} imports {module}.something",
    }
