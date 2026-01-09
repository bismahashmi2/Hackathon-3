#!/usr/bin/env python3
"""
Schema Validation Helper
Physical AI and Humanoid Robotics Textbook

Validates YAML/JSON content against OpenAPI/JSON Schema definitions.

Usage:
    python validate-schema.py <content_file> --schema <schema_name>
"""

import argparse
import json
import sys
from pathlib import Path
import yaml

try:
    from jsonschema import validate, ValidationError, Draft7Validator
    HAS_JSONSCHEMA = True
except ImportError:
    HAS_JSONSCHEMA = False
    print("Warning: jsonschema not installed. Run: pip install jsonschema")

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent.parent
SCHEMA_PATH = PROJECT_ROOT / "specs" / "002-physical-ai-robotics-textbook" / "contracts" / "module-schema.yaml"


def load_schema(schema_name: str) -> dict:
    """Load schema definition from the OpenAPI schema file."""
    if not SCHEMA_PATH.exists():
        raise FileNotFoundError(f"Schema file not found: {SCHEMA_PATH}")

    with open(SCHEMA_PATH, 'r', encoding='utf-8') as f:
        schema_doc = yaml.safe_load(f)

    # Extract the specific schema from components/schemas
    schemas = schema_doc.get("components", {}).get("schemas", {})
    if schema_name not in schemas:
        available = list(schemas.keys())
        raise ValueError(f"Schema '{schema_name}' not found. Available: {available}")

    # Build a complete JSON Schema with definitions
    schema = schemas[schema_name].copy()
    schema["$schema"] = "http://json-schema.org/draft-07/schema#"

    # Add all schemas as definitions for $ref resolution
    schema["definitions"] = {}
    for name, definition in schemas.items():
        schema["definitions"][name] = definition

    # Convert $ref paths from OpenAPI to JSON Schema format
    schema = _convert_refs(schema)

    return schema


def _convert_refs(obj):
    """Convert OpenAPI $ref paths to JSON Schema format."""
    if isinstance(obj, dict):
        if "$ref" in obj:
            # Convert #/components/schemas/Name to #/definitions/Name
            ref = obj["$ref"]
            if ref.startswith("#/components/schemas/"):
                obj["$ref"] = ref.replace("#/components/schemas/", "#/definitions/")
        return {k: _convert_refs(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_convert_refs(item) for item in obj]
    return obj


def validate_content(content: dict, schema_name: str) -> list[str]:
    """Validate content against a schema and return list of errors."""
    if not HAS_JSONSCHEMA:
        return ["jsonschema library not installed"]

    try:
        schema = load_schema(schema_name)
    except (FileNotFoundError, ValueError) as e:
        return [str(e)]

    errors = []
    validator = Draft7Validator(schema)

    for error in validator.iter_errors(content):
        path = ".".join(str(p) for p in error.absolute_path) or "root"
        errors.append(f"{path}: {error.message}")

    return errors


def load_content(file_path: Path) -> dict:
    """Load YAML or JSON content from file."""
    content = file_path.read_text(encoding='utf-8')

    if file_path.suffix in ['.yaml', '.yml']:
        return yaml.safe_load(content)
    elif file_path.suffix == '.json':
        return json.loads(content)
    else:
        # Try YAML first (more permissive)
        try:
            return yaml.safe_load(content)
        except yaml.YAMLError:
            return json.loads(content)


def main():
    parser = argparse.ArgumentParser(description="Validate content against schema")
    parser.add_argument("file", type=Path, help="Content file to validate (YAML or JSON)")
    parser.add_argument("--schema", "-s", type=str, required=True,
                       help="Schema name from module-schema.yaml (e.g., Module, LabExercise)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--list-schemas", "-l", action="store_true",
                       help="List available schema names")
    args = parser.parse_args()

    if args.list_schemas:
        if not SCHEMA_PATH.exists():
            print(f"Schema file not found: {SCHEMA_PATH}")
            sys.exit(1)

        with open(SCHEMA_PATH, 'r', encoding='utf-8') as f:
            schema_doc = yaml.safe_load(f)

        schemas = schema_doc.get("components", {}).get("schemas", {})
        print("Available schemas:")
        for name in sorted(schemas.keys()):
            print(f"  - {name}")
        sys.exit(0)

    if not args.file.exists():
        print(f"Error: File not found: {args.file}")
        sys.exit(1)

    print(f"Validating: {args.file}")
    print(f"Against schema: {args.schema}")

    try:
        content = load_content(args.file)
    except Exception as e:
        print(f"Error loading content: {e}")
        sys.exit(1)

    errors = validate_content(content, args.schema)

    if errors:
        print(f"\n❌ Validation failed with {len(errors)} error(s):\n")
        for error in errors:
            print(f"  • {error}")
        sys.exit(1)
    else:
        print("\n✅ Validation passed!")
        sys.exit(0)


if __name__ == "__main__":
    main()
