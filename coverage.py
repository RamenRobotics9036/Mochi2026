#!/usr/bin/env python3
"""Run unit tests with JaCoCo code coverage and open the HTML report."""

import subprocess
import sys
import platform
import os

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
REPORT_HTML = os.path.join(PROJECT_DIR, "build", "reports", "jacoco", "test", "html", "index.html")

def main():
    gradlew = os.path.join(PROJECT_DIR, "gradlew.bat" if platform.system() == "Windows" else "gradlew")

    print("Running tests with JaCoCo coverage...")
    result = subprocess.run(
        [gradlew, "clean", "test", "jacocoTestReport", "-PjacocoEnabled"],
        cwd=PROJECT_DIR,
    )

    if result.returncode != 0:
        print("Tests or coverage report failed.", file=sys.stderr)
        sys.exit(result.returncode)

    print(f"\nCoverage report: {REPORT_HTML}")

    # Open the report in the default browser
    if platform.system() == "Darwin":
        subprocess.run(["open", REPORT_HTML])
    elif platform.system() == "Windows":
        os.startfile(REPORT_HTML)
    else:
        subprocess.run(["xdg-open", REPORT_HTML])

if __name__ == "__main__":
    main()
