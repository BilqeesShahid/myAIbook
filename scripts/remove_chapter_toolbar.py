#!/usr/bin/env python3
"""
Script to remove ChapterToolbar from all markdown files in the docs folder.
This reverts the changes made by add_chapter_toolbar_fixed.py
"""

import os
from pathlib import Path
import re

def remove_chapter_toolbar_from_file(file_path):
    """Remove ChapterToolbar import and wrapper from a single markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Store original content for comparison
    original_content = content

    # Remove the import line
    content = re.sub(r'import ChapterToolbar from .*\n+', '', content)

    # Remove the <ChapterToolbar> wrapper tags (both opening and closing)
    content = re.sub(r'<ChapterToolbar>\n?', '', content)
    content = re.sub(r'\n?</ChapterToolbar>\n?$', '', content, flags=re.MULTILINE)

    # Clean up any empty lines left behind
    content = content.strip()

    # Only write if content has changed
    if content != original_content:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    return False

def main():
    docs_path = Path("F:/speckit-hackathon/docs")
    markdown_files = docs_path.rglob("*.md")

    updated_count = 0
    for md_file in markdown_files:
        try:
            if remove_chapter_toolbar_from_file(md_file):
                print(f"Updated {md_file}")
                updated_count += 1
        except Exception as e:
            print(f"Error processing {md_file}: {e}")

    print(f"Removed ChapterToolbar from {updated_count} files")

if __name__ == "__main__":
    main()