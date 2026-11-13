"""The HuRI session a robot can use: the YAML's session, minus what it cannot.

The YAML describes the richest session (say, with gestures); a route that
needs a capability the robot lacks is dropped, its HuRI topic unsubscribed,
and the modules that only fed it are dropped too: a robot without arms never
runs mov on the server's GPU. Which module feeds what comes from the server
(GET /modules); without it, nothing is pruned (the session still works, the
unused modules just run).
"""

import json
import logging
import urllib.request
from typing import Any, Dict, Iterable, List, Mapping, Optional, Set
from urllib.parse import urlsplit, urlunsplit

logger = logging.getLogger("huri_lucy_bridge.session")

ModuleDetails = Mapping[str, Mapping[str, Any]]  # name -> {input, output}


def modules_url(huri_url: str) -> str:
    """http(s)://host/modules of the server whose session is ws(s)://host/session."""

    parts = urlsplit(huri_url)
    scheme = {"ws": "http", "wss": "https"}.get(parts.scheme, parts.scheme)
    path = parts.path.rsplit("/", 1)[0] + "/modules"
    return urlunsplit((scheme, parts.netloc, path, "", ""))


def fetch_module_details(
    huri_url: str, timeout: float = 5.0
) -> Optional[ModuleDetails]:
    """The server's modules and their topics, or None when it cannot tell."""

    url = modules_url(huri_url)
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            details = json.load(response).get("details")
    except (OSError, ValueError, AttributeError) as e:
        logger.info("cannot read %s (%s): no module pruning", url, e)
        return None
    return details if isinstance(details, dict) else None


def prune_modules(
    modules: Mapping[str, Mapping[str, Any]],
    details: Optional[ModuleDetails],
    subscribed_before: Iterable[str],
    subscribed_after: Iterable[str],
) -> List[str]:
    """Ids of the modules whose output nobody consumes any more once the
    client subscribes to `subscribed_after` instead of `subscribed_before`.

    Only what the change starves goes: a module the YAML already left
    unconsumed (a side effect, a log) stays. Modules the server does not
    describe (`module: pkg:Cls`) stay."""

    if not details:
        return []

    def output(mod_id: str) -> Optional[str]:
        spec = details.get(str(modules[mod_id].get("name", "")))
        return spec.get("output") if spec else None

    def consumed(ids: Iterable[str], subscribed: Iterable[str]) -> Set[str]:
        topics = set(subscribed)
        for mod_id in ids:
            spec = details.get(str(modules[mod_id].get("name", "")))
            if spec and spec.get("input"):
                topics.add(spec["input"])
        return topics

    before = consumed(modules, subscribed_before)
    kept = list(modules)
    while True:
        now = consumed(kept, subscribed_after)
        starved = [
            mod_id
            for mod_id in kept
            if output(mod_id) in before and output(mod_id) not in now
        ]
        if not starved:
            return [mod_id for mod_id in modules if mod_id not in kept]
        kept = [mod_id for mod_id in kept if mod_id not in starved]


def without(
    raw: Mapping[str, Any], topics: Iterable[str], details: Optional[ModuleDetails]
) -> Dict[str, Any]:
    """`raw` (a client config) no longer subscribing to `topics`, and without
    the modules that only fed them."""

    topics = set(topics)
    subscribed = list(raw.get("subscribe") or [])
    after = [t for t in subscribed if t not in topics]
    modules = dict(raw.get("modules") or {})
    pruned = prune_modules(modules, details, subscribed, after)
    if pruned:
        logger.info("session without modules %s", ", ".join(pruned))
    return {
        **raw,
        "subscribe": after,
        "modules": {k: v for k, v in modules.items() if k not in pruned},
    }
