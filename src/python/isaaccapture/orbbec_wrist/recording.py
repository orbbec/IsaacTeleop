# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""TeleopSession recording with committed Orbbec Wrist media fragments."""

from contextlib import contextmanager
from dataclasses import replace
import logging
import importlib
from pathlib import Path
from typing import Mapping

from .merge import _fresh_paths, _partial, merge_recording

logger = logging.getLogger("isaaccapture.orbbec_wrist.recording")


def _option(arguments, name):
    values = [
        argument.split("=", 1)[1]
        for argument in arguments
        if argument.startswith(name + "=")
    ]
    return values[-1] if values else None


@contextmanager
def embedded_recording(config, *, media_fragments: Mapping[str, str | Path]):
    """Record one or more Orbbec Wrist plugins and merge their media after clean shutdown.

    Map every enabled Orbbec Wrist plugin_root_id to a fresh fragment path. The helper
    adds embedded/spool options. Catch KeyboardInterrupt inside the context.
    """
    mcap_reader = importlib.import_module("mcap.reader")
    importlib.import_module("mcap.writer")

    from isaaccapture.deviceio_session import McapRecordingConfig
    from isaaccapture.plugin_manager import PluginManager
    from isaaccapture.teleop_session_manager import SessionMode, TeleopSession

    if config.mode != SessionMode.LIVE or not isinstance(
        config.mcap_config, McapRecordingConfig
    ):
        raise ValueError(
            "Orbbec Wrist embedded_recording requires a live McapRecordingConfig"
        )
    plugins = [
        plugin
        for plugin in config.plugins
        if plugin.enabled and plugin.plugin_name == "orbbec_wrist_camera"
    ]
    roots = [plugin.plugin_root_id for plugin in plugins]
    all_roots = [plugin.plugin_root_id for plugin in config.plugins if plugin.enabled]
    if (
        not roots
        or any(not root for root in roots)
        or len(set(all_roots)) != len(all_roots)
        or set(media_fragments) != set(roots)
    ):
        raise ValueError(
            "Map each enabled Orbbec Wrist plugin_root_id exactly once; plugin roots must be unique"
        )
    final = Path(config.mcap_config.filename).absolute()
    recording = final.with_name(final.stem + ".session" + final.suffix)
    fragments = {root: Path(media_fragments[root]).absolute() for root in roots}
    paths = [final, recording, *fragments.values()]
    _fresh_paths([*paths, *(_partial(path) for path in paths)])
    if any(not path.parent.is_dir() for path in paths):
        raise FileNotFoundError(
            "Create recording and fragment directories before opening the session"
        )
    updated = {}
    expected = {}
    prefixes = set()
    serials = set()
    for plugin in plugins:
        manager = PluginManager(
            [str(path) for path in plugin.search_paths if path.is_dir()]
        )
        info = manager.get_plugin_info(plugin.plugin_name)
        arguments = list(info.args) + list(plugin.plugin_args)
        prefix = _option(arguments, "--collection-prefix")
        serial = _option(arguments, "--serial-number")
        if (
            not prefix
            or not prefix.strip("/")
            or prefix.strip("/") in prefixes
            or prefix.strip("/") == "session_observed"
            or prefix.startswith("session_observed/")
        ):
            raise ValueError(
                "Every Orbbec Wrist plugin requires a unique --collection-prefix"
            )
        prefixes.add(prefix.strip("/"))
        if (len(plugins) > 1 and not serial) or (serial and serial in serials):
            raise ValueError(
                "Multiple Orbbec Wrist plugins require distinct explicit serial numbers"
            )
        if serial:
            serials.add(serial)
        root = _option(arguments, "--plugin-root-id")
        if root is not None and root != plugin.plugin_root_id:
            raise ValueError(
                "Orbbec Wrist --plugin-root-id disagrees with PluginConfig"
            )
        if _option(arguments, "--mcap-filename") is not None:
            raise ValueError(
                "Use --mcap-media-spool instead of --mcap-filename with embedded_recording"
            )
        mode = _option(arguments, "--media-mode")
        if mode not in (None, "embedded"):
            raise ValueError(
                "Orbbec Wrist embedded_recording requires --media-mode=embedded"
            )
        spool = _option(arguments, "--mcap-media-spool")
        fragment = fragments[plugin.plugin_root_id]
        if (
            spool is not None
            and (Path(info.working_dir) / spool).resolve() != fragment.resolve()
        ):
            raise ValueError(
                "Orbbec Wrist --mcap-media-spool disagrees with media_fragments"
            )
        args = list(plugin.plugin_args)
        args.extend(["--media-mode=embedded", f"--mcap-media-spool={fragment}"])
        updated[plugin.plugin_root_id] = replace(
            plugin, plugin_args=args, required=True
        )
        expected[plugin.plugin_root_id] = (prefix, serial)
    session_config = replace(
        config,
        mcap_config=McapRecordingConfig(
            str(recording), config.mcap_config.get_tracker_names()
        ),
        plugins=[
            updated.get(plugin.plugin_root_id, plugin) if plugin.enabled else plugin
            for plugin in config.plugins
        ],
    )
    with TeleopSession(session_config) as session:
        yield session
    # A committed, clean terminal fragment is required even if Plugin.stop() clears exit status.
    for root, path in fragments.items():
        with path.open("rb") as stream:
            reader = mcap_reader.make_reader(stream, validate_crcs=True)
            identities = [
                item.metadata
                for item in reader.iter_metadata()
                if item.name == "wrist.capture"
            ]
        prefix, serial = expected[root]
        if (
            len(identities) != 1
            or identities[0].get("collection_prefix") != prefix
            or identities[0].get("plugin_root_id") != root
            or (serial and identities[0].get("serial_number") != serial)
        ):
            raise ValueError(
                f"Orbbec Wrist fragment belongs to another device/session: {path}"
            )
    merge_recording(recording, fragments.values(), final)
    logger.info("Finalized Orbbec Wrist session recording: %s", final)
