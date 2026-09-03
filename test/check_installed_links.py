#!/usr/bin/env python3
"""Walk every relative link in the INSTALLED documentation and fail on a break.

Why this exists
---------------
Stage 2 added `docker/` and `examples/` to the install rules because the
installed documentation links into them: README.md, README.ja.md,
docs/README.md, docs/en/README.md and both language versions of
docs/ablation_and_matched_evaluation.md carry 12 relative links to
docker/nav2-jazzy/README.md and examples/gazebo/README.md, and before those two
rules none of the 12 resolved under share/<package>. Nothing but a comment in
CMakeLists.txt held that coupling: drop an install rule and the links go quiet.
This makes the coupling a test.

WHY NOT ctest. It needs an install tree, and there is none to walk in the
plain-CMake build: every install() rule in CMakeLists.txt sits inside
`if(ament_cmake_FOUND)`, ament_cmake_DIR is NOTFOUND in a host build, and
`cmake --install build_host --prefix ...` was measured to write zero files. In
the container ctest already runs 11 tests against a build tree, not an install
tree. So this runs from docker/nav2-jazzy/test_package.sh, after colcon has
installed, against the real prefix.

docs/reviews/ IS EXCLUDED, on purpose and not to make a number look good.
Those 72 links point at src/, test/ and a sibling repository by design - the
review records cite the code they reviewed - and no install rule can make them
resolve under share/<package>. Excluding them is a decision about what install
rules can defend, not a way of hiding a break: the count of what was skipped is
printed, and a break anywhere else is still a failure. (Measured on the
installed tree: 393 relative link targets, 72 broken, all 72 under
docs/reviews/, 0 broken outside it.)

Usage: check_installed_links.py <install-prefix> [--min-links N]
"""
import argparse
import os
import re
import sys
import urllib.parse

# `[text](target)` and `![text](target)`, with an optional "title" after the
# target. The same expression the stage 2 audit used on this tree.
#
# MARKDOWN ONLY, and that is a limit of the checker, not a property of the
# tree. A reStructuredText link, `text <target>`_, CANNOT be matched by this
# expression at all - it is not that none happens to be present. The walk
# opens .rst files, so a future .rst relative link would be read and silently
# not checked. Measured today: the only installed .rst is CHANGELOG.rst and it
# carries no relative link in either form, so the gap costs nothing yet. Add a
# second pattern here before that changes.
LINK = re.compile(r'\[[^\]]*\]\(([^)\s]+)(?:\s+"[^"]*")?\)')

# Relative to the package's share directory. See the docstring.
EXCLUDED = ("docs/reviews/",)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", help="install prefix to walk")
    parser.add_argument(
        "--min-links", type=int, default=200,
        help="fail if fewer relative links than this were found: zero links "
             "would otherwise be zero broken links, and an install rule that "
             "stopped installing the documentation would PASS this check")
    args = parser.parse_args()

    if not os.path.isdir(args.prefix):
        print(f"install prefix does not exist: {args.prefix}", file=sys.stderr)
        return 2

    checked = 0
    skipped = 0
    broken = []
    for directory, _, files in os.walk(args.prefix):
        for name in sorted(files):
            if not name.endswith((".md", ".rst")):
                continue
            path = os.path.join(directory, name)
            relative = os.path.relpath(path, args.prefix)
            excluded = any(part in relative for part in EXCLUDED)
            with open(path, encoding="utf-8", errors="replace") as handle:
                for number, line in enumerate(handle, 1):
                    for match in LINK.finditer(line):
                        target = match.group(1)
                        if target.startswith(("http://", "https://", "mailto:", "#")):
                            continue
                        if excluded:
                            skipped += 1
                            continue
                        checked += 1
                        # Strip the #fragment: it addresses a line or an
                        # anchor inside the file, not a separate path.
                        cleaned = urllib.parse.unquote(target.split("#")[0])
                        if not cleaned:
                            continue
                        resolved = os.path.normpath(os.path.join(directory, cleaned))
                        if not os.path.exists(resolved):
                            broken.append((relative, number, target))

    print(f"installed relative links checked: {checked}")
    print(f"skipped under {' '.join(EXCLUDED)}: {skipped}")
    if checked < args.min_links:
        print(f"only {checked} relative links were found, expected at least "
              f"{args.min_links}: the installed documentation is missing, so "
              f"this check would have passed vacuously", file=sys.stderr)
        return 1
    if broken:
        print(f"{len(broken)} broken relative link(s) in the installed tree:",
              file=sys.stderr)
        for path, number, target in broken:
            print(f"  {path}:{number} -> {target}", file=sys.stderr)
        print("An installed document links to something that is not installed. "
              "Either add it to the install rules in CMakeLists.txt or un-link "
              "it in the same change.", file=sys.stderr)
        return 1
    print("no broken relative links in the installed tree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
