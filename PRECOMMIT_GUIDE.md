# Pre-commit Hooks for Automated Code Formatting

This document explains the pre-commit hooks setup for automated code formatting and quality checks in the mrobosub project.

## What's Included

### Code Formatting & Quality Tools

1. **Black** - Python code formatter
   - Ensures consistent code style
   - Line length: 88 characters
   - Automatically formats Python files

2. **isort** - Import statement organizer
   - Sorts and organizes Python imports
   - Configured to work with Black
   - Knows about ROS-specific packages

3. **flake8** - Python linting
   - Catches common Python errors
   - Enforces PEP 8 style guidelines
   - Configured to work with Black

4. **mypy** - Type checking
   - Uses existing `mypy.ini` configuration
   - Helps catch type-related errors
   - Maintains current project settings

### File Quality Checks

- **Trailing whitespace removal**
- **End-of-file fixer** (ensures files end with newline)
- **YAML/JSON/XML validation**
- **Large file detection** (warns about files >1MB)
- **Merge conflict detection**
- **Debug statement detection**

### ROS-Specific Tools

- **CMake formatting** - Formats CMakeLists.txt files
- **XML formatting** - Formats ROS launch files

## Setup Instructions

### Option 1: Automated Setup (Recommended)

Run the setup script:

```bash
python setup_precommit.py
```

This will:
1. Install pre-commit if needed
2. Install all hooks
3. Run initial formatting check
4. Show next steps

### Option 2: Manual Setup

1. Install pre-commit:
   ```bash
   pip install pre-commit
   ```

2. Install hooks:
   ```bash
   pre-commit install
   ```

3. Run on all files (optional):
   ```bash
   pre-commit run --all-files
   ```

## How It Works

### Automatic Execution
- Pre-commit hooks run automatically when you commit
- If any tool makes changes, the commit is blocked
- You can review changes and commit again
- Only staged files are checked

### Manual Execution
Run on all files:
```bash
pre-commit run --all-files
```

Run specific hook:
```bash
pre-commit run black
pre-commit run flake8
```

### Skip Hooks (if needed)
Skip all hooks for a commit:
```bash
git commit --no-verify -m "message"
```

Skip specific hook:
```bash
SKIP=flake8 git commit -m "message"
```

## Configuration Files

- **`.pre-commit-config.yaml`** - Main pre-commit configuration
- **`pyproject.toml`** - Tool-specific settings (Black, isort, mypy)
- **`mypy.ini`** - Existing mypy configuration (unchanged)

## Benefits

1. **Consistent Code Style** - All code follows the same formatting standards
2. **Automatic Formatting** - No need to manually format code
3. **Early Error Detection** - Catch issues before they reach CI/CD
4. **Improved Code Quality** - Standardized imports, type checking, linting
5. **Reduced Review Time** - Less time spent on style discussions
6. **ROS Integration** - Handles ROS-specific file types (launch files, CMake)

## Troubleshooting

### Pre-commit is slow on first run
- This is normal - it downloads and installs tools
- Subsequent runs are much faster

### Hook fails
1. Review the error message
2. Fix the issue manually or let the tool auto-fix
3. Stage the changes: `git add .`
4. Commit again

### Disable temporarily
If you need to commit without running hooks:
```bash
git commit --no-verify -m "your message"
```

## Customization

To modify settings:
- Edit `.pre-commit-config.yaml` for hook configuration
- Edit `pyproject.toml` for tool-specific settings
- Keep `mypy.ini` for existing mypy settings

## Support

- Pre-commit documentation: https://pre-commit.com/
- Black documentation: https://black.readthedocs.io/
- isort documentation: https://pycqa.github.io/isort/
- flake8 documentation: https://flake8.pycqa.org/
