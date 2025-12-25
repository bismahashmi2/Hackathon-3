#!/usr/bin/env python3
"""
Code Example Extraction Utility
Physical AI and Humanoid Robotics Textbook

Extracts Python code blocks from markdown files for testing.

Usage:
    python extract-code.py <markdown_file> [--output-dir <dir>]
"""

import argparse
import re
import sys
from pathlib import Path
from typing import NamedTuple


class CodeBlock(NamedTuple):
    """Represents an extracted code block."""
    language: str
    code: str
    line_number: int
    block_id: str | None


def extract_code_blocks(content: str) -> list[CodeBlock]:
    """Extract all code blocks from markdown content."""
    blocks = []
    # Pattern matches ```language ... ``` blocks
    pattern = r'^```(\w+)\n(.*?)^```'

    for match in re.finditer(pattern, content, re.MULTILINE | re.DOTALL):
        language = match.group(1)
        code = match.group(2)
        line_number = content[:match.start()].count('\n') + 1

        # Try to find a preceding ID comment
        block_id = None
        preceding_lines = content[:match.start()].split('\n')[-3:]
        for line in preceding_lines:
            if 'id:' in line.lower():
                id_match = re.search(r'id[:\s]+["\']?(\S+)["\']?', line)
                if id_match:
                    block_id = id_match.group(1)
                    break

        blocks.append(CodeBlock(
            language=language,
            code=code.strip(),
            line_number=line_number,
            block_id=block_id
        ))

    return blocks


def extract_python_code(markdown_path: Path) -> list[CodeBlock]:
    """Extract only Python code blocks from a markdown file."""
    content = markdown_path.read_text(encoding='utf-8')
    all_blocks = extract_code_blocks(content)
    return [b for b in all_blocks if b.language == 'python']


def save_code_blocks(blocks: list[CodeBlock], output_dir: Path, source_name: str):
    """Save extracted code blocks to individual files."""
    output_dir.mkdir(parents=True, exist_ok=True)

    for i, block in enumerate(blocks):
        # Create filename from block ID or index
        if block.block_id:
            filename = f"{source_name}_{block.block_id}.py"
        else:
            filename = f"{source_name}_block_{i+1:02d}.py"

        output_path = output_dir / filename

        # Add header comment
        header = f'''#!/usr/bin/env python3
"""
Extracted from: {source_name}
Line: {block.line_number}
Block ID: {block.block_id or 'N/A'}

This code was automatically extracted for testing.
"""

'''
        output_path.write_text(header + block.code, encoding='utf-8')
        print(f"  Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Extract code blocks from markdown")
    parser.add_argument("file", type=Path, help="Markdown file to extract from")
    parser.add_argument("--output-dir", "-o", type=Path, default=Path("extracted_code"),
                       help="Output directory for extracted code")
    parser.add_argument("--python-only", "-p", action="store_true",
                       help="Extract only Python code blocks")
    parser.add_argument("--list-only", "-l", action="store_true",
                       help="List blocks without saving")
    args = parser.parse_args()

    if not args.file.exists():
        print(f"Error: File not found: {args.file}")
        sys.exit(1)

    print(f"Extracting code from: {args.file}")

    if args.python_only:
        blocks = extract_python_code(args.file)
    else:
        content = args.file.read_text(encoding='utf-8')
        blocks = extract_code_blocks(content)

    print(f"Found {len(blocks)} code blocks")

    if args.list_only:
        for i, block in enumerate(blocks):
            print(f"\n[Block {i+1}] Language: {block.language}, Line: {block.line_number}")
            print(f"ID: {block.block_id or 'N/A'}")
            print(f"Preview: {block.code[:100]}...")
    else:
        source_name = args.file.stem
        save_code_blocks(blocks, args.output_dir, source_name)
        print(f"\nExtracted {len(blocks)} blocks to {args.output_dir}/")


if __name__ == "__main__":
    main()
