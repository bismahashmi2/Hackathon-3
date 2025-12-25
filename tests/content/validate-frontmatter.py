#!/usr/bin/env python3
"""
Module Frontmatter Validator
Physical AI and Humanoid Robotics Textbook

Validates frontmatter in module theory.md files, checking prerequisites,
learning objectives count, and other required metadata.

Usage:
    python validate-frontmatter.py [--module XX] [--verbose]
"""

import argparse
import re
import sys
from pathlib import Path
from typing import Optional
import yaml


# Project paths
PROJECT_ROOT = Path(__file__).parent.parent.parent
MODULES_DIR = PROJECT_ROOT / "textbook" / "modules"


class FrontmatterError:
    def __init__(self, module_id: str, field: str, message: str, severity: str = "error"):
        self.module_id = module_id
        self.field = field
        self.message = message
        self.severity = severity

    def __str__(self):
        icon = "❌" if self.severity == "error" else "⚠️"
        return f"{icon} [Module {self.module_id}] {self.field}: {self.message}"


def extract_frontmatter(file_path: Path) -> Optional[dict]:
    """Extract YAML frontmatter from markdown file."""
    try:
        content = file_path.read_text(encoding='utf-8')
        match = re.match(r'^---\s*\n(.*?)\n---', content, re.DOTALL)
        if match:
            return yaml.safe_load(match.group(1))
        return None
    except Exception:
        return None


def validate_frontmatter(module_path: Path) -> list[FrontmatterError]:
    """Validate frontmatter for a module's theory.md file."""
    errors = []
    theory_path = module_path / "theory.md"
    module_id = module_path.name[:2] if module_path.name[0].isdigit() else "XX"

    if not theory_path.exists():
        return [FrontmatterError(module_id, "theory.md", "File not found")]

    frontmatter = extract_frontmatter(theory_path)
    if frontmatter is None:
        return [FrontmatterError(module_id, "frontmatter", "Missing or invalid YAML frontmatter")]

    # Required fields
    required_fields = {
        "id": str,
        "title": str,
        "slug": str,
        "week": int,
        "difficulty": str,
        "learning_objectives": list,
    }

    for field, expected_type in required_fields.items():
        if field not in frontmatter:
            errors.append(FrontmatterError(
                module_id, field, f"Required field missing"
            ))
        elif not isinstance(frontmatter[field], expected_type):
            errors.append(FrontmatterError(
                module_id, field,
                f"Expected {expected_type.__name__}, got {type(frontmatter[field]).__name__}"
            ))

    # Validate id format (should be 2-digit string)
    if "id" in frontmatter:
        fm_id = frontmatter["id"]
        if not re.match(r'^[0-9]{2}$', str(fm_id)):
            errors.append(FrontmatterError(
                module_id, "id",
                f"Must be two-digit string (e.g., '01'), got '{fm_id}'"
            ))
        elif str(fm_id) != module_id:
            errors.append(FrontmatterError(
                module_id, "id",
                f"ID '{fm_id}' doesn't match directory name module ID '{module_id}'",
                severity="warning"
            ))

    # Validate slug format (lowercase with hyphens)
    if "slug" in frontmatter:
        slug = frontmatter["slug"]
        if not re.match(r'^[a-z0-9-]+$', slug):
            errors.append(FrontmatterError(
                module_id, "slug",
                f"Must be lowercase alphanumeric with hyphens, got '{slug}'"
            ))

    # Validate week range (1-14)
    if "week" in frontmatter:
        week = frontmatter["week"]
        if isinstance(week, int):
            if week < 1 or week > 14:
                errors.append(FrontmatterError(
                    module_id, "week",
                    f"Must be between 1 and 14, got {week}"
                ))

    # Validate difficulty enum
    if "difficulty" in frontmatter:
        difficulty = frontmatter["difficulty"]
        valid_values = ["beginner", "intermediate", "advanced"]
        if difficulty not in valid_values:
            errors.append(FrontmatterError(
                module_id, "difficulty",
                f"Must be one of {valid_values}, got '{difficulty}'"
            ))

    # Validate learning objectives (3-5 items)
    if "learning_objectives" in frontmatter:
        objectives = frontmatter["learning_objectives"]
        if isinstance(objectives, list):
            count = len(objectives)
            if count < 3:
                errors.append(FrontmatterError(
                    module_id, "learning_objectives",
                    f"Minimum 3 required, found {count}"
                ))
            elif count > 5:
                errors.append(FrontmatterError(
                    module_id, "learning_objectives",
                    f"Maximum 5 allowed, found {count}",
                    severity="warning"
                ))

            # Check each objective is meaningful (min length)
            for i, obj in enumerate(objectives):
                if not isinstance(obj, str):
                    errors.append(FrontmatterError(
                        module_id, f"learning_objectives[{i}]",
                        f"Must be string, got {type(obj).__name__}"
                    ))
                elif len(obj) < 20:
                    errors.append(FrontmatterError(
                        module_id, f"learning_objectives[{i}]",
                        f"Should be at least 20 characters, got {len(obj)}",
                        severity="warning"
                    ))

    # Validate prerequisites (must reference lower module numbers)
    if "prerequisites" in frontmatter:
        prerequisites = frontmatter["prerequisites"]
        if isinstance(prerequisites, list):
            current_id = frontmatter.get("id", "99")
            for prereq in prerequisites:
                if not re.match(r'^[0-9]{2}$', str(prereq)):
                    errors.append(FrontmatterError(
                        module_id, "prerequisites",
                        f"Invalid prerequisite format: '{prereq}'"
                    ))
                elif str(prereq) >= str(current_id):
                    errors.append(FrontmatterError(
                        module_id, "prerequisites",
                        f"Prerequisite '{prereq}' should come before module '{current_id}'"
                    ))

    # Validate estimated_hours (5-20)
    if "estimated_hours" in frontmatter:
        hours = frontmatter["estimated_hours"]
        if isinstance(hours, int):
            if hours < 5 or hours > 20:
                errors.append(FrontmatterError(
                    module_id, "estimated_hours",
                    f"Should be between 5 and 20, got {hours}",
                    severity="warning"
                ))

    # Validate status enum
    if "status" in frontmatter:
        status = frontmatter["status"]
        valid_values = ["draft", "review", "published"]
        if status not in valid_values:
            errors.append(FrontmatterError(
                module_id, "status",
                f"Must be one of {valid_values}, got '{status}'"
            ))

    return errors


def validate_all_modules(verbose: bool = False) -> list[FrontmatterError]:
    """Validate frontmatter for all modules."""
    all_errors = []

    if not MODULES_DIR.exists():
        return [FrontmatterError("--", "modules/", "Modules directory not found")]

    for module_dir in sorted(MODULES_DIR.iterdir()):
        if module_dir.is_dir() and module_dir.name != "_template":
            errors = validate_frontmatter(module_dir)
            all_errors.extend(errors)

            if verbose:
                module_id = module_dir.name[:2]
                error_count = sum(1 for e in errors if e.severity == "error")
                warning_count = sum(1 for e in errors if e.severity == "warning")
                status = "✓" if error_count == 0 else f"✗ ({error_count} errors, {warning_count} warnings)"
                print(f"Module {module_id}: {status}")

    return all_errors


def main():
    parser = argparse.ArgumentParser(description="Validate module frontmatter")
    parser.add_argument("--module", "-m", type=str,
                       help="Validate specific module by ID (e.g., 01)")
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    if args.module:
        # Find and validate specific module
        module_dirs = list(MODULES_DIR.glob(f"{args.module}-*"))
        if not module_dirs:
            print(f"Module not found: {args.module}")
            sys.exit(1)
        errors = validate_frontmatter(module_dirs[0])
    else:
        errors = validate_all_modules(args.verbose)

    # Print results
    if errors:
        print("\n" + "=" * 50)
        print("FRONTMATTER VALIDATION RESULTS")
        print("=" * 50 + "\n")

        for error in errors:
            print(error)

        error_count = sum(1 for e in errors if e.severity == "error")
        warning_count = sum(1 for e in errors if e.severity == "warning")

        print(f"\nTotal: {error_count} errors, {warning_count} warnings")
        sys.exit(1 if error_count > 0 else 0)
    else:
        print("✅ All frontmatter validation passed!")


if __name__ == "__main__":
    main()
