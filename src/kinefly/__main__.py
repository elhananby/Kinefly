"""CLI entry point for kinefly."""

import sys


def main() -> int:
    print(f"kinefly {__import__('kinefly').__version__}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
