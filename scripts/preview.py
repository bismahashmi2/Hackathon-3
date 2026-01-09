#!/usr/bin/env python3
"""
HTML Preview Generator
Physical AI and Humanoid Robotics Textbook

Converts markdown content to HTML for preview.

Usage:
    python preview.py <module_path>
    python preview.py textbook/modules/01-introduction-physical-ai/
"""

import argparse
import re
import sys
from pathlib import Path
from typing import Optional

try:
    from markdown_it import MarkdownIt
    HAS_MARKDOWN_IT = True
except ImportError:
    HAS_MARKDOWN_IT = False

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
BUILD_DIR = PROJECT_ROOT / "textbook" / "build" / "preview"

# HTML template
HTML_TEMPLATE = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title}</title>
    <style>
        :root {{
            --bg-color: #ffffff;
            --text-color: #1a1a1a;
            --link-color: #0066cc;
            --code-bg: #f5f5f5;
            --border-color: #e0e0e0;
            --callout-bg: #f0f7ff;
            --warning-bg: #fff3e0;
            --definition-bg: #e8f5e9;
        }}

        @media (prefers-color-scheme: dark) {{
            :root {{
                --bg-color: #1a1a1a;
                --text-color: #e0e0e0;
                --link-color: #66b3ff;
                --code-bg: #2d2d2d;
                --border-color: #444;
                --callout-bg: #1a2a3a;
                --warning-bg: #3a2a1a;
                --definition-bg: #1a2a1a;
            }}
        }}

        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            line-height: 1.6;
            max-width: 800px;
            margin: 0 auto;
            padding: 2rem;
            background-color: var(--bg-color);
            color: var(--text-color);
        }}

        h1 {{ border-bottom: 2px solid var(--border-color); padding-bottom: 0.5rem; }}
        h2 {{ border-bottom: 1px solid var(--border-color); padding-bottom: 0.3rem; margin-top: 2rem; }}
        h3 {{ margin-top: 1.5rem; }}

        a {{ color: var(--link-color); text-decoration: none; }}
        a:hover {{ text-decoration: underline; }}

        pre {{
            background-color: var(--code-bg);
            padding: 1rem;
            border-radius: 4px;
            overflow-x: auto;
        }}

        code {{
            background-color: var(--code-bg);
            padding: 0.2rem 0.4rem;
            border-radius: 3px;
            font-size: 0.9em;
        }}

        pre code {{
            background: none;
            padding: 0;
        }}

        blockquote {{
            border-left: 4px solid var(--border-color);
            margin: 1rem 0;
            padding: 0.5rem 1rem;
            background-color: var(--code-bg);
        }}

        table {{
            border-collapse: collapse;
            width: 100%;
            margin: 1rem 0;
        }}

        th, td {{
            border: 1px solid var(--border-color);
            padding: 0.5rem;
            text-align: left;
        }}

        th {{
            background-color: var(--code-bg);
        }}

        .definition {{
            background-color: var(--definition-bg);
            padding: 1rem;
            border-radius: 4px;
            margin: 1rem 0;
            border-left: 4px solid #4caf50;
        }}

        .warning {{
            background-color: var(--warning-bg);
            padding: 1rem;
            border-radius: 4px;
            margin: 1rem 0;
            border-left: 4px solid #ff9800;
        }}

        .example {{
            background-color: var(--callout-bg);
            padding: 1rem;
            border-radius: 4px;
            margin: 1rem 0;
            border-left: 4px solid #2196f3;
        }}

        .checkpoint {{
            background-color: #e3f2fd;
            padding: 1rem;
            border-radius: 4px;
            margin: 1rem 0;
            border-left: 4px solid #1976d2;
        }}

        .ethics-callout {{
            background-color: #fce4ec;
            padding: 1rem;
            border-radius: 4px;
            margin: 1rem 0;
            border-left: 4px solid #c2185b;
        }}

        .equation {{
            text-align: center;
            margin: 1.5rem 0;
            padding: 1rem;
            background-color: var(--code-bg);
            border-radius: 4px;
        }}

        .nav {{
            position: fixed;
            top: 1rem;
            right: 1rem;
            background: var(--bg-color);
            padding: 0.5rem 1rem;
            border: 1px solid var(--border-color);
            border-radius: 4px;
        }}

        .frontmatter {{
            background-color: var(--code-bg);
            padding: 1rem;
            border-radius: 4px;
            margin-bottom: 2rem;
            font-size: 0.9em;
        }}
    </style>
</head>
<body>
    <nav class="nav">
        <a href="index.html">← Back</a>
    </nav>
    <article>
        {content}
    </article>
</body>
</html>
'''


def convert_custom_tags(content: str) -> str:
    """Convert custom markdown tags to HTML divs."""
    # Definition tags
    content = re.sub(
        r'<definition[^>]*>(.*?)</definition>',
        r'<div class="definition">\1</div>',
        content,
        flags=re.DOTALL
    )

    # Warning tags
    content = re.sub(
        r'<warning[^>]*>(.*?)</warning>',
        r'<div class="warning">⚠️ \1</div>',
        content,
        flags=re.DOTALL
    )

    # Example tags
    content = re.sub(
        r'<example[^>]*>(.*?)</example>',
        r'<div class="example">\1</div>',
        content,
        flags=re.DOTALL
    )

    # Checkpoint tags
    content = re.sub(
        r'<checkpoint[^>]*>(.*?)</checkpoint>',
        r'<div class="checkpoint">🔍 Checkpoint\n\1</div>',
        content,
        flags=re.DOTALL
    )

    # Ethics callout tags
    content = re.sub(
        r'<ethics-callout[^>]*>(.*?)</ethics-callout>',
        r'<div class="ethics-callout">⚖️ Ethics\n\1</div>',
        content,
        flags=re.DOTALL
    )

    # Equation tags
    content = re.sub(
        r'<equation[^>]*>(.*?)</equation>',
        r'<div class="equation">\1</div>',
        content,
        flags=re.DOTALL
    )

    return content


def extract_frontmatter(content: str) -> tuple[Optional[dict], str]:
    """Extract and remove YAML frontmatter from content."""
    import yaml

    match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
    if match:
        try:
            frontmatter = yaml.safe_load(match.group(1))
            content = content[match.end():]
            return frontmatter, content
        except yaml.YAMLError:
            pass
    return None, content


def render_frontmatter(fm: dict) -> str:
    """Render frontmatter as HTML."""
    if not fm:
        return ""

    html = '<div class="frontmatter"><h4>Module Information</h4><dl>'
    fields = [
        ("Title", fm.get("title")),
        ("Week", fm.get("week")),
        ("Difficulty", fm.get("difficulty")),
        ("Estimated Hours", fm.get("estimated_hours")),
        ("Status", fm.get("status")),
    ]

    for label, value in fields:
        if value:
            html += f"<dt>{label}</dt><dd>{value}</dd>"

    if "learning_objectives" in fm:
        html += "<dt>Learning Objectives</dt><dd><ul>"
        for obj in fm["learning_objectives"]:
            html += f"<li>{obj}</li>"
        html += "</ul></dd>"

    html += "</dl></div>"
    return html


def convert_markdown_to_html(markdown_content: str, title: str = "Preview") -> str:
    """Convert markdown content to HTML."""
    if not HAS_MARKDOWN_IT:
        # Fallback: basic conversion
        html_content = f"<pre>{markdown_content}</pre>"
        return HTML_TEMPLATE.format(title=title, content=html_content)

    # Extract frontmatter
    frontmatter, content = extract_frontmatter(markdown_content)

    # Pre-process custom tags
    content = convert_custom_tags(content)

    # Convert markdown to HTML
    md = MarkdownIt('commonmark', {'typographer': True})
    md.enable('table')
    html_content = md.render(content)

    # Add frontmatter display
    fm_html = render_frontmatter(frontmatter)

    # Get title from frontmatter if available
    if frontmatter and "title" in frontmatter:
        title = frontmatter["title"]

    return HTML_TEMPLATE.format(title=title, content=fm_html + html_content)


def generate_module_preview(module_path: Path, output_dir: Path) -> list[Path]:
    """Generate HTML preview for all files in a module."""
    generated = []

    # Create output directory
    module_name = module_path.name
    module_output = output_dir / module_name
    module_output.mkdir(parents=True, exist_ok=True)

    # Process theory.md
    theory_path = module_path / "theory.md"
    if theory_path.exists():
        content = theory_path.read_text(encoding='utf-8')
        html = convert_markdown_to_html(content, f"Theory - {module_name}")
        output_path = module_output / "theory.html"
        output_path.write_text(html, encoding='utf-8')
        generated.append(output_path)

    # Process labs
    labs_dir = module_path / "labs"
    if labs_dir.exists():
        for lab_file in sorted(labs_dir.glob("*.md")):
            if "_template" in lab_file.name:
                continue
            content = lab_file.read_text(encoding='utf-8')
            html = convert_markdown_to_html(content, f"Lab - {lab_file.stem}")
            output_path = module_output / f"{lab_file.stem}.html"
            output_path.write_text(html, encoding='utf-8')
            generated.append(output_path)

    # Process ethics.md
    ethics_path = module_path / "ethics.md"
    if ethics_path.exists():
        content = ethics_path.read_text(encoding='utf-8')
        html = convert_markdown_to_html(content, f"Ethics - {module_name}")
        output_path = module_output / "ethics.html"
        output_path.write_text(html, encoding='utf-8')
        generated.append(output_path)

    # Generate index page
    index_html = generate_index_page(module_name, generated)
    index_path = module_output / "index.html"
    index_path.write_text(index_html, encoding='utf-8')
    generated.insert(0, index_path)

    return generated


def generate_index_page(module_name: str, files: list[Path]) -> str:
    """Generate an index page for the module preview."""
    links = []
    for f in files:
        if f.name != "index.html":
            name = f.stem.replace("-", " ").title()
            links.append(f'<li><a href="{f.name}">{name}</a></li>')

    content = f'''
    <h1>{module_name}</h1>
    <h2>Preview Files</h2>
    <ul>
        {"".join(links)}
    </ul>
    '''

    return HTML_TEMPLATE.format(title=f"Module Preview - {module_name}", content=content)


def main():
    parser = argparse.ArgumentParser(description="Generate HTML preview for module content")
    parser.add_argument("path", type=Path, help="Module directory path")
    parser.add_argument("--output", "-o", type=Path, default=BUILD_DIR,
                       help="Output directory (default: textbook/build/preview)")
    args = parser.parse_args()

    if not args.path.exists():
        print(f"Error: Path not found: {args.path}")
        sys.exit(1)

    if not HAS_MARKDOWN_IT:
        print("Warning: markdown-it-py not installed. Install with: pip install markdown-it-py")
        print("Falling back to basic preview.")

    print(f"Generating preview for: {args.path}")

    if args.path.is_dir():
        generated = generate_module_preview(args.path, args.output)
    else:
        # Single file
        content = args.path.read_text(encoding='utf-8')
        html = convert_markdown_to_html(content, args.path.stem)
        output_path = args.output / f"{args.path.stem}.html"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(html, encoding='utf-8')
        generated = [output_path]

    print(f"\nGenerated {len(generated)} file(s):")
    for f in generated:
        print(f"  {f}")

    print(f"\nOpen preview: file://{generated[0]}")


if __name__ == "__main__":
    main()
