#!/usr/bin/env python3
"""
Lab Structure Validator
Physical AI and Humanoid Robotics Textbook

Validates lab exercises to ensure proper count (3-5 per module) and format.

Usage:
    python validate-labs.py [--module XX] [--verbose]
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


class LabError:
    def __init__(self, module_id: str, lab_id: str, message: str, severity: str = "error"):
        self.module_id = module_id
        self.lab_id = lab_id
        self.message = message
        self.severity = severity

    def __str__(self):
        icon = "❌" if self.severity == "error" else "⚠️"
        return f"{icon} [Module {self.module_id}] Lab {self.lab_id}: {self.message}"


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


def validate_lab_file(lab_path: Path, module_id: str) -> list[LabError]:
    """Validate a single lab file."""
    errors = []
    lab_id = lab_path.stem

    frontmatter = extract_frontmatter(lab_path)
    if frontmatter is None:
        return [LabError(module_id, lab_id, "Missing or invalid YAML frontmatter")]

    # Required fields
    required_fields = ["id", "module_id", "title", "difficulty", "tier"]
    for field in required_fields:
        if field not in frontmatter:
            errors.append(LabError(module_id, lab_id, f"Missing required field: {field}"))

    # Validate id format (lab-XX-NN)
    if "id" in frontmatter:
        fm_id = frontmatter["id"]
        if not re.match(r'^lab-[0-9]{2}-[0-9]{2}$', str(fm_id)):
            errors.append(LabError(
                module_id, lab_id,
                f"ID must match format 'lab-XX-NN', got '{fm_id}'"
            ))

    # Validate module_id matches
    if "module_id" in frontmatter:
        fm_module = frontmatter["module_id"]
        if str(fm_module) != module_id:
            errors.append(LabError(
                module_id, lab_id,
                f"module_id '{fm_module}' doesn't match parent module '{module_id}'",
                severity="warning"
            ))

    # Validate difficulty enum
    if "difficulty" in frontmatter:
        difficulty = frontmatter["difficulty"]
        valid_values = ["guided", "intermediate", "challenge"]
        if difficulty not in valid_values:
            errors.append(LabError(
                module_id, lab_id,
                f"difficulty must be one of {valid_values}, got '{difficulty}'"
            ))

    # Validate tier enum
    if "tier" in frontmatter:
        tier = frontmatter["tier"]
        valid_values = ["simulation", "low_cost_hardware", "advanced_hardware"]
        if tier not in valid_values:
            errors.append(LabError(
                module_id, lab_id,
                f"tier must be one of {valid_values}, got '{tier}'"
            ))

    # Validate duration_minutes (30-180)
    if "duration_minutes" in frontmatter:
        duration = frontmatter["duration_minutes"]
        if isinstance(duration, int):
            if duration < 30:
                errors.append(LabError(
                    module_id, lab_id,
                    f"duration_minutes should be at least 30, got {duration}",
                    severity="warning"
                ))
            elif duration > 180:
                errors.append(LabError(
                    module_id, lab_id,
                    f"duration_minutes should be at most 180, got {duration}",
                    severity="warning"
                ))

    # Check content structure (look for required sections)
    content = lab_path.read_text(encoding='utf-8')

    required_sections = ["## Objectives", "## Instructions"]
    for section in required_sections:
        if section.lower() not in content.lower():
            errors.append(LabError(
                module_id, lab_id,
                f"Missing required section: {section}"
            ))

    # Check for at least one checkpoint
    if "<checkpoint>" not in content.lower() and "**expected" not in content.lower():
        errors.append(LabError(
            module_id, lab_id,
            "Lab should include at least one checkpoint for student verification",
            severity="warning"
        ))

    return errors


def validate_module_labs(module_path: Path) -> list[LabError]:
    """Validate all labs in a module."""
    errors = []
    module_id = module_path.name[:2] if module_path.name[0].isdigit() else "XX"
    labs_dir = module_path / "labs"

    if not labs_dir.exists():
        return [LabError(module_id, "--", "labs/ directory not found")]

    # Find all lab files (exclude template)
    lab_files = [f for f in labs_dir.glob("lab-*.md") if "_template" not in f.name]

    # Check count (3-5 labs required)
    lab_count = len(lab_files)
    if lab_count < 3:
        errors.append(LabError(
            module_id, "--",
            f"Module requires 3-5 labs, found {lab_count}"
        ))
    elif lab_count > 5:
        errors.append(LabError(
            module_id, "--",
            f"Module should have at most 5 labs, found {lab_count}",
            severity="warning"
        ))

    # Validate each lab file
    for lab_file in sorted(lab_files):
        lab_errors = validate_lab_file(lab_file, module_id)
        errors.extend(lab_errors)

    # Check for variety in difficulty levels
    difficulties = set()
    for lab_file in lab_files:
        fm = extract_frontmatter(lab_file)
        if fm and "difficulty" in fm:
            difficulties.add(fm["difficulty"])

    if lab_count >= 3 and len(difficulties) < 2:
        errors.append(LabError(
            module_id, "--",
            f"Labs should have variety in difficulty levels, all are '{list(difficulties)[0] if difficulties else 'unknown'}'",
            severity="warning"
        ))

    return errors


def validate_all_modules(verbose: bool = False) -> list[LabError]:
    """Validate labs for all modules."""
    all_errors = []

    if not MODULES_DIR.exists():
        return [LabError("--", "--", "Modules directory not found")]

    for module_dir in sorted(MODULES_DIR.iterdir()):
        if module_dir.is_dir() and module_dir.name != "_template":
            errors = validate_module_labs(module_dir)
            all_errors.extend(errors)

            if verbose:
                module_id = module_dir.name[:2]
                error_count = sum(1 for e in errors if e.severity == "error")
                status = "✓" if error_count == 0 else f"✗ ({error_count} errors)"
                print(f"Module {module_id}: {status}")

    return all_errors


def main():
    parser = argparse.ArgumentParser(description="Validate module lab structure")
    parser.add_argument("--module", "-m", type=str,
                       help="Validate specific module by ID")
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    if args.module:
        module_dirs = list(MODULES_DIR.glob(f"{args.module}-*"))
        if not module_dirs:
            print(f"Module not found: {args.module}")
            sys.exit(1)
        errors = validate_module_labs(module_dirs[0])
    else:
        errors = validate_all_modules(args.verbose)

    if errors:
        print("\n" + "=" * 50)
        print("LAB VALIDATION RESULTS")
        print("=" * 50 + "\n")

        for error in errors:
            print(error)

        error_count = sum(1 for e in errors if e.severity == "error")
        warning_count = sum(1 for e in errors if e.severity == "warning")

        print(f"\nTotal: {error_count} errors, {warning_count} warnings")
        sys.exit(1 if error_count > 0 else 0)
    else:
        print("✅ All lab validations passed!")


if __name__ == "__main__":
    main()
