#!/bin/bash
# Physical AI and Humanoid Robotics Textbook
# Master Validation Runner Script
#
# Runs all content validation scripts and reports results.
#
# Usage:
#   ./scripts/validate-all.sh [--verbose] [--module XX]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Project root (relative to script location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Parse arguments
VERBOSE=""
MODULE=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE="--verbose"
            shift
            ;;
        -m|--module)
            MODULE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

MODULE_ARG=""
if [ -n "$MODULE" ]; then
    MODULE_ARG="--module $MODULE"
fi

echo "=============================================="
echo " Physical AI Textbook - Content Validation"
echo "=============================================="
echo ""
echo "Project root: $PROJECT_ROOT"
echo ""

# Track overall status
PASSED=0
FAILED=0
WARNINGS=0

# Function to run a validation script
run_validation() {
    local name="$1"
    local script="$2"

    echo -n "Running $name... "

    if [ ! -f "$PROJECT_ROOT/$script" ]; then
        echo -e "${YELLOW}SKIP${NC} (script not found)"
        return 0
    fi

    # Run the script and capture output
    set +e
    output=$(python3 "$PROJECT_ROOT/$script" $VERBOSE $MODULE_ARG 2>&1)
    status=$?
    set -e

    if [ $status -eq 0 ]; then
        echo -e "${GREEN}PASS${NC}"
        ((PASSED++))
    else
        echo -e "${RED}FAIL${NC}"
        ((FAILED++))
        if [ -n "$VERBOSE" ]; then
            echo "$output" | head -50
        fi
    fi
}

# Run all validation scripts
echo "Structure Validation:"
echo "---------------------"
run_validation "Structure validator" "tests/content/validate-structure.py"
run_validation "Frontmatter validator" "tests/content/validate-frontmatter.py"
run_validation "Labs validator" "tests/content/validate-labs.py"

echo ""
echo "Code Example Tests:"
echo "-------------------"
# Check if pytest is available and code examples exist
if command -v pytest &> /dev/null; then
    if [ -d "$PROJECT_ROOT/tests/code-examples" ] && [ -n "$(ls -A $PROJECT_ROOT/tests/code-examples/*.py 2>/dev/null)" ]; then
        echo -n "Running pytest... "
        set +e
        output=$(cd "$PROJECT_ROOT" && pytest tests/code-examples/ -q 2>&1)
        status=$?
        set -e
        if [ $status -eq 0 ]; then
            echo -e "${GREEN}PASS${NC}"
            ((PASSED++))
        else
            echo -e "${RED}FAIL${NC}"
            ((FAILED++))
            if [ -n "$VERBOSE" ]; then
                echo "$output" | head -20
            fi
        fi
    else
        echo -e "Code example tests: ${YELLOW}SKIP${NC} (no test files found)"
    fi
else
    echo -e "pytest: ${YELLOW}SKIP${NC} (pytest not installed)"
fi

# Summary
echo ""
echo "=============================================="
echo " Summary"
echo "=============================================="
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✅ All validations passed!${NC}"
    exit 0
else
    echo -e "${RED}❌ Some validations failed.${NC}"
    exit 1
fi
