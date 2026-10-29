import pytest

from huri_lucy_bridge.session import modules_url, prune_modules, without

# The server's GET /modules details of a voice + gestures pipeline.
DETAILS = {
    "vad": {"input": "huri.audio.in", "output": "huri.voice"},
    "stt": {"input": "huri.voice", "output": "huri.transcript"},
    "rag": {"input": "huri.transcript", "output": "huri.token"},
    "tts": {"input": "huri.token", "output": "huri.audio.out"},
    "mov": {"input": "huri.audio.out", "output": "huri.motion"},
    "smooth": {"input": "huri.motion", "output": "huri.smooth_motion"},
    "logger": {"input": "huri.transcript", "output": "huri.log"},
}
MODULES = {name: {"name": name} for name in DETAILS}
FULL = ["huri.audio.out", "huri.smooth_motion"]


@pytest.mark.parametrize(
    "url, expected",
    [
        ("ws://localhost:8000/session", "http://localhost:8000/modules"),
        ("wss://huri.example/api/session", "https://huri.example/api/modules"),
    ],
)
def test_modules_url(url, expected):
    assert modules_url(url) == expected


def test_what_only_fed_a_dropped_topic_goes():
    # No gestures: smooth goes, and mov, which only fed it.
    assert prune_modules(MODULES, DETAILS, FULL, ["huri.audio.out"]) == [
        "mov",
        "smooth",
    ]


def test_modules_the_yaml_already_left_unconsumed_stay():
    # `logger`'s output was never consumed: a side effect, kept.
    pruned = prune_modules(MODULES, DETAILS, FULL, ["huri.audio.out"])
    assert "logger" not in pruned


def test_a_module_still_feeding_another_stays():
    # Muting the speaker keeps tts: mov still needs its audio.
    assert prune_modules(MODULES, DETAILS, FULL, ["huri.smooth_motion"]) == []


def test_without_server_details_nothing_is_pruned():
    assert prune_modules(MODULES, None, FULL, []) == []
    # Modules the server does not describe (`module: pkg:Cls`) stay.
    modules = {**MODULES, "mov": {"module": "my_pkg:Mov"}}
    assert prune_modules(modules, DETAILS, FULL, ["huri.audio.out"]) == ["smooth"]


def test_without_rewrites_the_session():
    raw = {"user_id": "u", "send": ["huri.audio.in"], "subscribe": FULL}
    raw["modules"] = MODULES
    session = without(raw, ["huri.smooth_motion"], DETAILS)
    assert session["subscribe"] == ["huri.audio.out"]
    assert set(session["modules"]) == set(MODULES) - {"mov", "smooth"}
    assert session["send"] == ["huri.audio.in"] and raw["subscribe"] == FULL
