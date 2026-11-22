"""Published documentation links, safe to import from the device layer."""

DOCS_BASE_URL = "https://real-world-rl.readthedocs.io/zh-cn/latest/"


def docs_url(page: str, anchor: str | None = None, *, lang: str = "en") -> str:
    """Link to a page in English or Chinese (``zh-cn``), with a verbatim label."""
    prefix = "en/" if lang == "en" else ""
    fragment = f"#{anchor}" if anchor is not None else ""
    return f"{DOCS_BASE_URL}{prefix}{page}.html{fragment}"
