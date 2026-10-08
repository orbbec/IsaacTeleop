# SPDX-FileCopyrightText: Copyright (c) 2026 ORBBEC INC. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Finalize EGO media after the official session has released its resources."""

from __future__ import annotations

import logging
import os
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING
import subprocess

if TYPE_CHECKING:
    from collections.abc import Iterator

    from isaaccapture.teleop_session_manager import TeleopSession, TeleopSessionConfig

logger = logging.getLogger(__name__)


def _option(args: list[str], name: str) -> str | None:
    for argument in reversed(args):
        if argument.startswith(name + "="):
            return argument[len(name) + 1 :]
    return None


def _fresh_paths(paths: tuple[Path, ...]) -> None:
    if len(set(paths)) != len(paths):
        raise ValueError("Recording, media fragment, and final output must be distinct")
    for path in paths:
        if not path.parent.is_dir():
            raise ValueError(
                f"Create the output directory before recording: {path.parent}"
            )
        for candidate in (path, Path(str(path) + ".partial")):
            if candidate.exists() or candidate.is_symlink():
                raise FileExistsError(
                    f"Refusing to overwrite recording output: {candidate}"
                )


@contextmanager
def embedded_recording(
    config: TeleopSessionConfig,
    *,
    media_fragment: str | os.PathLike[str],
    merger: str | os.PathLike[str] | None = None,
) -> Iterator[TeleopSession]:
    """Record through TeleopSession, then atomically merge the EGO capture fragment.

    Configure one EGO plugin with collection-prefix, embedded media, and a matching
    mcap-media-spool. Catch KeyboardInterrupt inside the context for a clean stop.
    Failures preserve intermediate files and propagate to the caller.
    """
    from isaaccapture.deviceio_session import McapRecordingConfig
    from isaaccapture.plugin_manager import PluginManager
    from isaaccapture.teleop_session_manager import SessionMode, TeleopSession

    if config.mode != SessionMode.LIVE:
        raise ValueError("embedded_recording requires a live session")
    if not isinstance(config.mcap_config, McapRecordingConfig):
        raise ValueError("embedded_recording requires McapRecordingConfig")
    plugins = [
        plugin
        for plugin in config.plugins
        if plugin.enabled and plugin.plugin_name == "ego_camera"
    ]
    if len(plugins) != 1:
        raise ValueError(
            "embedded_recording requires exactly one enabled ego_camera plugin"
        )
    plugin = plugins[0]
    manager = PluginManager(
        [str(path) for path in plugin.search_paths if path.is_dir()]
    )
    info = manager.get_plugin_info(plugin.plugin_name)
    working_dir = Path(info.working_dir).resolve()
    arguments = list(info.args) + list(plugin.plugin_args)
    fragment = Path(media_fragment).resolve()
    spool = _option(arguments, "--mcap-media-spool")
    if not _option(arguments, "--collection-prefix"):
        raise ValueError("The EGO plugin requires --collection-prefix=PREFIX")
    if _option(arguments, "--mcap-media") != "embedded" or not spool:
        raise ValueError(
            "The EGO plugin requires embedded media and --mcap-media-spool=PATH"
        )
    if (working_dir / spool).resolve() != fragment:
        raise ValueError("media_fragment must match the EGO plugin's mcap-media-spool")
    if _option(arguments, "--mcap-filename"):
        raise ValueError(
            "The EGO plugin cannot use --mcap-filename with a live collection"
        )
    executable = (
        Path(merger).resolve() if merger is not None else working_dir / "ego_mcap_merge"
    )
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise FileNotFoundError(
            f"Build or install the EGO merge tool before capture: {executable}"
        )

    final = Path(config.mcap_config.filename).resolve()
    recording = final.with_name(final.stem + ".session" + final.suffix)
    _fresh_paths((recording, fragment, final))
    recording_config = McapRecordingConfig(
        str(recording), config.mcap_config.get_tracker_names()
    )
    session_config = replace(
        config,
        mcap_config=recording_config,
        plugins=[
            replace(item, required=True) if item is plugin else item
            for item in config.plugins
        ],
    )
    with TeleopSession(session_config) as session:
        yield session

    # Explicit Plugin.stop() clears exit status, so only a committed fragment proves completion.
    if not fragment.is_file() or Path(str(fragment) + ".partial").exists():
        raise RuntimeError(
            f"EGO capture did not commit a complete fragment: {fragment}"
        )
    subprocess.run(
        [
            str(executable),
            "--recording",
            str(recording),
            "--media",
            str(fragment),
            "--output",
            str(final),
        ],
        check=True,
    )
    if not final.is_file() or Path(str(final) + ".partial").exists():
        raise RuntimeError(f"The merge tool did not commit a complete output: {final}")
    logger.info("Finalized EGO recording: %s", final)
