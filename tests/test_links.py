"""Links to other HuRI repositories point only at repositories that exist."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# huri-core is the ecosystem front door; HuRI-landing is archived and
# Lucy-Robotics/HuRI only redirects, so neither is ever linked.
# lucy_ws and lucy_control_panel: the Lucy robot stack the bridge talks to.
KNOWN = {"huri-core", "huri-modules", "HuRI-InMoov", "lucy_ws", "lucy_control_panel"}
SKIP = {".git", ".venv", "venv", "node_modules", "build", "dist", "__pycache__"}
SUFFIXES = {".md", ".toml", ".yml", ".yaml", ".py", ".sh"}


def linked_repositories():
    for path in ROOT.rglob("*"):
        if path.suffix not in SUFFIXES or SKIP & set(path.relative_to(ROOT).parts):
            continue
        for repo in re.findall(r"github\.com/Lucy-Robotics/([\w-]+)", path.read_text()):
            yield path.relative_to(ROOT), repo


def test_only_existing_huri_repositories_are_linked():
    unknown = [(str(p), r) for p, r in linked_repositories() if r not in KNOWN]
    assert unknown == []
