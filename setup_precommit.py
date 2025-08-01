#!/usr/bin/env python3
"""
Setup script for installing and configuring pre-commit hooks.

This script helps set up the development environment with automated
code formatting and quality checks.
"""

import os
import subprocess
import sys
from pathlib import Path


def run_command(cmd, description=""):
    """Run a command and handle errors gracefully."""
    print(f"Running: {description or cmd}")
    try:
        result = subprocess.run(
            cmd, shell=True, check=True, capture_output=True, text=True
        )
        if result.stdout:
            print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        if e.stdout:
            print(f"stdout: {e.stdout}")
        if e.stderr:
            print(f"stderr: {e.stderr}")
        return False


def check_python_version():
    """Check if Python version is compatible."""
    if sys.version_info < (3, 8):
        print("Warning: Python 3.8+ is recommended for best compatibility.")
        return False
    return True


def install_pre_commit():
    """Install pre-commit if not already installed."""
    try:
        subprocess.run(
            [sys.executable, "-m", "pre_commit", "--version"],
            capture_output=True,
            check=True,
        )
        print("✅ pre-commit is already installed")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("📦 Installing pre-commit...")
        return run_command(
            f"{sys.executable} -m pip install pre-commit", "Installing pre-commit"
        )


def setup_pre_commit_hooks():
    """Set up pre-commit hooks."""
    if not Path(".pre-commit-config.yaml").exists():
        print("❌ .pre-commit-config.yaml not found!")
        return False

    print("🔧 Installing pre-commit hooks...")
    # Use Python module syntax to ensure it works cross-platform
    return run_command(
        f"{sys.executable} -m pre_commit install", "Installing pre-commit hooks"
    )


def run_initial_check():
    """Run pre-commit on all files to check setup."""
    print("🧪 Running initial pre-commit check on all files...")
    print("This may take a while on first run as it downloads dependencies...")

    # Run pre-commit on all files using Python module syntax
    success = run_command(
        f"{sys.executable} -m pre_commit run --all-files",
        "Running pre-commit on all files",
    )

    if not success:
        print("\n⚠️  Some files needed formatting. This is normal on first run.")
        print("The files have been automatically formatted.")
        print("Please review the changes and commit them.")
    else:
        print("✅ All checks passed!")

    return True


def main():
    """Main setup function."""
    print("🚀 Setting up pre-commit hooks for mrobosub...")
    print("=" * 50)

    # Check Python version
    if not check_python_version():
        print("Consider upgrading Python for best results.")

    # Install pre-commit
    if not install_pre_commit():
        print("❌ Failed to install pre-commit")
        return 1

    # Setup hooks
    if not setup_pre_commit_hooks():
        print("❌ Failed to set up pre-commit hooks")
        return 1

    # Run initial check
    run_initial_check()

    print("\n🎉 Setup complete!")
    print("\nNext steps:")
    print("1. Review any auto-formatted files")
    print(
        "2. Commit the changes: git add . && git commit -m 'feat: add pre-commit hooks for auto-formatting'"
    )
    print("3. Pre-commit will now run automatically on every commit")
    print("\nTo manually run pre-commit: pre-commit run --all-files")

    return 0


if __name__ == "__main__":
    sys.exit(main())
