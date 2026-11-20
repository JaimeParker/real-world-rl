"""Sphinx configuration for the real-world-rl tutorials."""

import os
import tomllib
from pathlib import Path

metadata_path = Path(__file__).resolve().parents[1] / "pyproject.toml"
with metadata_path.open("rb") as metadata_file:
    metadata = tomllib.load(metadata_file)["project"]

project = metadata["name"]
release = metadata["version"]
root_doc = "index"
language = "zh_CN"
extensions = ["myst_parser", "sphinx_copybutton", "sphinx_design"]
templates_path = ["_templates"]
myst_heading_anchors = 3
# Publish the tracked guides and tutorials. Local design notes live under design/.
exclude_patterns = ["_build", "README.md", "design"]
nitpicky = True

html_theme = "pydata_sphinx_theme"
html_static_path = ["_static"]
html_css_files = ["css/expnote.css"]
html_title = project
# Sphinx indexes Chinese words with jieba and stems English terms in this mode.
html_search_language = "zh"
html_baseurl = os.environ.get("READTHEDOCS_CANONICAL_URL", "")
html_sidebars = {
    "**": ["expnote-sidebar-header.html", "expnote-sidebar-nav.html"],
}
html_theme_options = {
    "navbar_start": [],
    "navbar_center": [],
    "navbar_end": [],
    "navbar_persistent": [],
    "article_header_start": [],
    "secondary_sidebar_items": {"**": ["page-toc"], "index": [], "en/index": []},
    "github_url": "https://github.com/JaimeParker/real-world-rl",
    "primary_sidebar_end": [],
    "show_nav_level": 2,
    "navigation_depth": 3,
    "collapse_navigation": False,
}
html_show_sourcelink = False


def set_sidebar_context(app, pagename, templatename, context, doctree):
    """Resolve page-level language links before Sphinx renders the sidebar."""
    chinese = not pagename.startswith("en/")
    chinese_page = pagename.removeprefix("en/")
    english_page = f"en/{chinese_page}"
    pages = app.env.found_docs
    context.update(
        sidebar_home="index" if chinese else "en/index",
        sidebar_english=english_page if english_page in pages else "en/index",
        sidebar_chinese=chinese_page if chinese_page in pages else "index",
        sidebar_is_chinese=chinese,
        theme_search_bar_text="搜索文档…" if chinese else "Search docs…",
    )
    context["language"] = "zh-CN" if chinese else "en"


def setup(app):
    app.connect("html-page-context", set_sidebar_context)
