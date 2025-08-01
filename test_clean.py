# Test file for pre-commit verification
print("This is a test file to verify pre-commit hooks work properly!")


def test_function():
    # This should be formatted by Black and checked by flake8
    x = 1
    y = 2
    return x + y


if __name__ == "__main__":
    result = test_function()
    print(f"Result: {result}")
    # Test commit to verify pre-commit hooks
