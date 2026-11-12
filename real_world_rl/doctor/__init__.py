"""``rwrl doctor``: host, project-environment and device diagnostics.

``report`` inspects and repairs (the command), ``host``/``platform``/``preset``/``repair`` are its
host-side parts, ``checks`` is the layered device engine that ``probe`` runs in an isolated
interpreter, and ``deps``/``probes`` serve ``checks``. Nothing is imported here, so host-only
diagnosis keeps working when the device layer cannot be imported."""
