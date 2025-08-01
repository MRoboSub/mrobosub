# Pre-commit Hooks Setup Guide

This guide explains how to set up and use the pre-commit hooks for automated code formatting and quality checks in the mrobosub project.

## What's Included

The pre-commit configuration includes the following tools:

### Core Formatting Tools (Active)
- **Black**: Python code formatter - ensures consistent code style
- **isort**: Import sorter - organizes Python imports consistently  
- **flake8**: Python linter - catches code quality issues (with relaxed rules for existing codebase)
- **cmake-format**: CMake file formatter - formats CMake files consistently

### File Quality Checks
- **trailing-whitespace**: Removes trailing whitespace
- **end-of-file-fixer**: Ensures files end with newline
- **check-yaml/xml/json**: Validates syntax of config files
- **check-added-large-files**: Prevents accidentally committing large files
- **check-merge-conflict**: Detects merge conflict markers
- **debug-statements**: Finds leftover debug statements

## Installation

Run the setup script to install and configure pre-commit hooks:

`ash
python setup_precommit.py
`

This script will:
1. Install pre-commit if not already installed
2. Install the pre-commit hooks
3. Run an initial check on all files

## Usage

### Automatic Usage
Once installed, pre-commit will run automatically on every git commit, checking only the files you're committing.

### Manual Usage
To run pre-commit on all files manually:
`ash
pre-commit run --all-files
`

To run pre-commit on specific files:
`ash
pre-commit run --files path/to/file.py
`

To run only specific hooks:
`ash
pre-commit run black --all-files
pre-commit run flake8 --all-files
`

## Configuration Details

### Black (Python Formatter)
- Line length: 88 characters (Black's default)
- Automatically formats Python code for consistency

### isort (Import Sorter)  
- Profile: black (compatible with Black formatter)
- Automatically sorts and organizes imports

### flake8 (Python Linter)
- Line length: 120 characters (relaxed for existing codebase)
- Ignores common issues in existing ROS codebases
- Helps catch potential bugs and style issues

### cmake-format (CMake Formatter)
- Formats CMakeLists.txt files consistently
- Important for ROS package build files

## Type Checking Note

mypy (static type checker) was considered but excluded from the current configuration due to the extensive type annotation requirements in the existing codebase. Adding comprehensive type checking would require significant changes to existing code, which is beyond the scope of this formatting improvement.

Future contributors can add mypy back with more lenient settings if desired.

## Troubleshooting

### Pre-commit fails with "command not found"
Make sure pre-commit is installed:
`ash
pip install pre-commit
`

### Files are modified by hooks
This is normal! The hooks automatically fix formatting issues. Review the changes and commit them:
`ash
git add .
git commit -m "Apply auto-formatting fixes"
`

### Hook execution is slow on first run
The first run downloads and caches dependencies. Subsequent runs will be much faster.

## Benefits

1. **Consistent Code Style**: All Python code follows the same formatting standards
2. **Automatic Import Organization**: Imports are consistently sorted and grouped
3. **Early Error Detection**: Catches common issues before they reach code review
4. **Reduced Review Overhead**: Maintainers can focus on logic rather than style
5. **CMake Consistency**: Build files are consistently formatted
6. **File Quality**: Ensures clean, well-formed files

## Integration with Development Workflow

1. **Before Committing**: Pre-commit runs automatically, fixing formatting issues
2. **During Development**: Run manually to check code quality
3. **In CI/CD**: Can be integrated into continuous integration pipelines
4. **Code Reviews**: Reduces style-related review comments

This setup significantly improves code quality and developer experience while being respectful of the existing codebase structure.
