#!/usr/bin/env python3
"""
Content Structure Validation Script
Physical AI and Humanoid Robotics Textbook

Validates module directory structure and content against the defined schema.

Usage:
    python validate-structure.py [--module XX] [--verbose]
"""

import argparse
import sys
from pathlib import Path
import yaml
import json
import re
from typing import Optional

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent.parent
TEXTBOOK_DIR = PROJECT_ROOT / "textbook"
MODULES_DIR = TEXTBOOK_DIR / "modules"
SCHEMA_PATH = PROJECT_ROOT / "specs" / "002-physical-ai-robotics-textbook" / "contracts" / "module-schema.yaml"

# Required files in each module
REQUIRED_FILES = [
    "theory.md",
    "ethics.md",
    "assessment.md",
]

REQUIRED_DIRS = [
    "labs",
    "simulations",
]


class ValidationError:
    def __init__(self, module_id: str, file_path: str, message: str, severity: str = "error"):
        self.module_id = module_id
        self.file_path = file_path
        self.message = message
        self.severity = severity  # error | warning

    def __str__(self):
        icon = "❌" if self.severity == "error" else "⚠️"
        return f"{icon} [{self.module_id}] {self.file_path}: {self.message}"


class ModuleValidator:
    def __init__(self, module_path: Path, verbose: bool = False):
        self.module_path = module_path
        self.module_id = module_path.name[:2] if module_path.name[0].isdigit() else "XX"
        self.verbose = verbose
        self.errors: list[ValidationError] = []

    def validate(self) -> list[ValidationError]:
        """Run all validation checks on the module."""
        self.errors = []

        # Skip template directory
        if self.module_path.name == "_template":
            return self.errors

        self._validate_directory_structure()
        self._validate_theory_frontmatter()
        self._validate_labs()
        self._validate_ethics()
        self._validate_assessment()

        return self.errors

    def _validate_directory_structure(self):
        """Check that all required files and directories exist."""
        for file_name in REQUIRED_FILES:
            file_path = self.module_path / file_name
            if not file_path.exists():
                self.errors.append(ValidationError(
                    self.module_id,
                    file_name,
                    f"Required file missing"
                ))

        for dir_name in REQUIRED_DIRS:
            dir_path = self.module_path / dir_name
            if not dir_path.exists():
                self.errors.append(ValidationError(
                    self.module_id,
                    dir_name,
                    f"Required directory missing"
                ))

    def _validate_theory_frontmatter(self):
        """Validate theory.md frontmatter against schema."""
        theory_path = self.module_path / "theory.md"
        if not theory_path.exists():
            return

        frontmatter = self._extract_frontmatter(theory_path)
        if frontmatter is None:
            self.errors.append(ValidationError(
                self.module_id,
                "theory.md",
                "Missing or invalid YAML frontmatter"
            ))
            return

        # Check required fields
        required_fields = ["id", "title", "slug", "week", "difficulty", "learning_objectives"]
        for field in required_fields:
            if field not in frontmatter:
                self.errors.append(ValidationError(
                    self.module_id,
                    "theory.md",
                    f"Missing required frontmatter field: {field}"
                ))

        # Validate learning objectives count (3-5)
        objectives = frontmatter.get("learning_objectives", [])
        if not isinstance(objectives, list):
            self.errors.append(ValidationError(
                self.module_id,
                "theory.md",
                "learning_objectives must be a list"
            ))
        elif len(objectives) < 3:
            self.errors.append(ValidationError(
                self.module_id,
                "theory.md",
                f"Minimum 3 learning objectives required, found {len(objectives)}"
            ))
        elif len(objectives) > 5:
            self.errors.append(ValidationError(
                self.module_id,
                "theory.md",
                f"Maximum 5 learning objectives allowed, found {len(objectives)}",
                severity="warning"
            ))

        # Validate difficulty enum
        valid_difficulties = ["beginner", "intermediate", "advanced"]
        difficulty = frontmatter.get("difficulty", "")
        if difficulty not in valid_difficulties:
            self.errors.append(ValidationError(
                self.module_id,
                "theory.md",
                f"Invalid difficulty '{difficulty}', must be one of: {valid_difficulties}"
            ))

        # Validate week range
        week = frontmatter.get("week", 0)
        if not isinstance(week, int) or week < 1 or week > 14:
            self.errors.append(ValidationError(
                self.module_id,
                "theory.md",
                f"Week must be an integer between 1 and 14, found {week}"
            ))

        # Validate prerequisites don't create circular dependencies
        prerequisites = frontmatter.get("prerequisites", [])
        module_id = frontmatter.get("id", "99")
        for prereq in prerequisites:
            if prereq >= module_id:
                self.errors.append(ValidationError(
                    self.module_id,
                    "theory.md",
                    f"Prerequisite '{prereq}' must come before module '{module_id}'",
                    severity="warning"
                ))

    def _validate_labs(self):
        """Validate lab files (3-5 labs per module)."""
        labs_dir = self.module_path / "labs"
        if not labs_dir.exists():
            return

        lab_files = list(labs_dir.glob("lab-*.md"))
        # Exclude template
        lab_files = [f for f in lab_files if "_template" not in f.name]

        if len(lab_files) < 3:
            self.errors.append(ValidationError(
                self.module_id,
                "labs/",
                f"Minimum 3 labs required, found {len(lab_files)}"
            ))
        elif len(lab_files) > 5:
            self.errors.append(ValidationError(
                self.module_id,
                "labs/",
                f"Maximum 5 labs allowed, found {len(lab_files)}",
                severity="warning"
            ))

        # Validate each lab frontmatter
        for lab_file in lab_files:
            self._validate_lab_file(lab_file)

    def _validate_lab_file(self, lab_path: Path):
        """Validate individual lab file."""
        frontmatter = self._extract_frontmatter(lab_path)
        if frontmatter is None:
            self.errors.append(ValidationError(
                self.module_id,
                f"labs/{lab_path.name}",
                "Missing or invalid YAML frontmatter"
            ))
            return

        # Check required fields
        required_fields = ["id", "module_id", "title", "difficulty", "tier"]
        for field in required_fields:
            if field not in frontmatter:
                self.errors.append(ValidationError(
                    self.module_id,
                    f"labs/{lab_path.name}",
                    f"Missing required frontmatter field: {field}"
                ))

        # Validate difficulty enum
        valid_difficulties = ["guided", "intermediate", "challenge"]
        difficulty = frontmatter.get("difficulty", "")
        if difficulty not in valid_difficulties:
            self.errors.append(ValidationError(
                self.module_id,
                f"labs/{lab_path.name}",
                f"Invalid difficulty '{difficulty}', must be one of: {valid_difficulties}"
            ))

        # Validate tier enum
        valid_tiers = ["simulation", "low_cost_hardware", "advanced_hardware"]
        tier = frontmatter.get("tier", "")
        if tier not in valid_tiers:
            self.errors.append(ValidationError(
                self.module_id,
                f"labs/{lab_path.name}",
                f"Invalid tier '{tier}', must be one of: {valid_tiers}"
            ))

        # Validate duration range
        duration = frontmatter.get("duration_minutes", 60)
        if not isinstance(duration, int) or duration < 30 or duration > 180:
            self.errors.append(ValidationError(
                self.module_id,
                f"labs/{lab_path.name}",
                f"Duration must be between 30 and 180 minutes, found {duration}",
                severity="warning"
            ))

    def _validate_ethics(self):
        """Validate ethics.md content."""
        ethics_path = self.module_path / "ethics.md"
        if not ethics_path.exists():
            return

        frontmatter = self._extract_frontmatter(ethics_path)
        if frontmatter is None:
            self.errors.append(ValidationError(
                self.module_id,
                "ethics.md",
                "Missing or invalid YAML frontmatter"
            ))
            return

        # Check module_id matches
        if frontmatter.get("module_id", "") != self.module_id:
            self.errors.append(ValidationError(
                self.module_id,
                "ethics.md",
                f"module_id in frontmatter doesn't match directory ({frontmatter.get('module_id', 'missing')} != {self.module_id})",
                severity="warning"
            ))

    def _validate_assessment(self):
        """Validate assessment.md content."""
        assessment_path = self.module_path / "assessment.md"
        if not assessment_path.exists():
            return

        frontmatter = self._extract_frontmatter(assessment_path)
        if frontmatter is None:
            self.errors.append(ValidationError(
                self.module_id,
                "assessment.md",
                "Missing or invalid YAML frontmatter"
            ))
            return

    def _extract_frontmatter(self, file_path: Path) -> Optional[dict]:
        """Extract YAML frontmatter from markdown file."""
        try:
            content = file_path.read_text(encoding='utf-8')
            # Match YAML frontmatter between --- markers
            match = re.match(r'^---\s*\n(.*?)\n---', content, re.DOTALL)
            if match:
                return yaml.safe_load(match.group(1))
            return None
        except Exception as e:
            if self.verbose:
                print(f"Error reading {file_path}: {e}")
            return None


def validate_all_modules(verbose: bool = False) -> list[ValidationError]:
    """Validate all modules in the textbook directory."""
    all_errors = []

    if not MODULES_DIR.exists():
        print(f"❌ Modules directory not found: {MODULES_DIR}")
        return [ValidationError("--", str(MODULES_DIR), "Modules directory not found")]

    for module_dir in sorted(MODULES_DIR.iterdir()):
        if module_dir.is_dir() and module_dir.name != "_template":
            validator = ModuleValidator(module_dir, verbose)
            errors = validator.validate()
            all_errors.extend(errors)

            if verbose:
                status = "✓" if len(errors) == 0 else f"✗ ({len(errors)} issues)"
                print(f"Module {module_dir.name}: {status}")

    return all_errors


def validate_single_module(module_id: str, verbose: bool = False) -> list[ValidationError]:
    """Validate a single module by ID."""
    # Find the module directory
    module_dirs = list(MODULES_DIR.glob(f"{module_id}-*"))
    if not module_dirs:
        return [ValidationError(module_id, "--", f"Module directory not found for ID {module_id}")]

    validator = ModuleValidator(module_dirs[0], verbose)
    return validator.validate()


def main():
    parser = argparse.ArgumentParser(description="Validate textbook module structure")
    parser.add_argument("--module", "-m", type=str, help="Validate specific module by ID (e.g., 01, 05)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--json", action="store_true", help="Output results as JSON")
    args = parser.parse_args()

    if args.module:
        errors = validate_single_module(args.module, args.verbose)
    else:
        errors = validate_all_modules(args.verbose)

    # Output results
    if args.json:
        result = {
            "total_errors": sum(1 for e in errors if e.severity == "error"),
            "total_warnings": sum(1 for e in errors if e.severity == "warning"),
            "errors": [
                {
                    "module": e.module_id,
                    "file": e.file_path,
                    "message": e.message,
                    "severity": e.severity
                }
                for e in errors
            ]
        }
        print(json.dumps(result, indent=2))
    else:
        if errors:
            print("\n" + "=" * 60)
            print("VALIDATION RESULTS")
            print("=" * 60 + "\n")

            for error in errors:
                print(error)

            error_count = sum(1 for e in errors if e.severity == "error")
            warning_count = sum(1 for e in errors if e.severity == "warning")

            print(f"\n{'=' * 60}")
            print(f"Total: {error_count} errors, {warning_count} warnings")
            print("=" * 60)
        else:
            print("✅ All validations passed!")

    # Exit with error code if there are errors (not warnings)
    error_count = sum(1 for e in errors if e.severity == "error")
    sys.exit(1 if error_count > 0 else 0)


if __name__ == "__main__":
    main()
