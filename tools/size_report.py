#!/usr/bin/env python3
# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

"""Report the flash/RAM footprint of the USB Type-C stack in a Zephyr build.

Parses the GNU ld map file of a linked Zephyr image and sums the section
sizes contributed by the stack's object files (the usbc library, the
application object that instantiates the state machines, and any extra
objects selected with --match).

Merged string pools (.rodata.*.str1.*) are attributed by the linker to the
first contributing object with the size of the whole pool; this script uses
the per-object "(size before relaxing)" contribution instead, so an object
is charged only for its own strings. Sections listed under "Discarded input
sections" (removed by --gc-sections) are ignored.

Usage:
  size_report.py [BUILD_DIR_OR_MAP] [--lib NAME]... [--match REGEX]... [--json]

BUILD_DIR_OR_MAP defaults to "build"; a directory is searched for
zephyr/zephyr_final.map (falling back to zephyr/zephyr.map). --lib NAME
selects the objects of lib<NAME>.a; it can be given multiple times and
defaults to "usbc" and "app" (the application objects holding the template
instantiations).
"""

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

DEFAULT_LIBS = ["usbc", "app"]

BUCKETS = ("code", "rodata", "strings", "data", "bss")

SECTION_RE = re.compile(r"^ (\.[\w.$*-]+|COMMON)\s*$")
CONTRIB_RE = re.compile(
    r"^ (\.[\w.$*-]+|COMMON)?\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+(\S+\.(?:obj|o)\)?)$"
)
RELAX_RE = re.compile(r"^\s+(0x[0-9a-f]+) \(size before relaxing\)")


def bucket_of(section: str) -> str | None:
    if ".str" in section:
        return "strings"
    if section.startswith((".text", ".init", ".fini", ".ARM", ".glue", ".vector")):
        return "code"
    if section.startswith(".rodata"):
        return "rodata"
    if section.startswith(".data"):
        return "data"
    if section.startswith((".bss", ".noinit")) or section == "COMMON":
        return "bss"
    return None  # debug info, comments, ...


def find_map(target: Path) -> Path:
    if target.is_file():
        return target
    for name in ("zephyr/zephyr_final.map", "zephyr/zephyr.map"):
        candidate = target / name
        if candidate.is_file():
            return candidate
    sys.exit(f"error: no linker map found under {target}")


def parse_map(map_path: Path, patterns: list[re.Pattern]):
    totals: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    in_memory_map = False
    pending_section = None
    pending_contrib = None  # (obj, section, size) awaiting a possible relax line

    def commit(contrib):
        obj, section, size = contrib
        kind = bucket_of(section)
        if kind is not None:
            totals[obj][kind] += size

    with open(map_path) as f:
        for line in f:
            if not in_memory_map:
                if line.startswith("Linker script and memory map"):
                    in_memory_map = True
                continue

            relax = RELAX_RE.match(line)
            if relax and pending_contrib:
                obj, section, _ = pending_contrib
                commit((obj, section, int(relax.group(1), 16)))
                pending_contrib = None
                continue
            if pending_contrib:
                commit(pending_contrib)
                pending_contrib = None

            m = SECTION_RE.match(line)
            if m:
                pending_section = m.group(1)
                continue
            m = CONTRIB_RE.match(line)
            if m:
                section = m.group(1) or pending_section
                pending_section = None
                if section is None:
                    continue
                obj = m.group(3)
                if not any(p.search(obj) for p in patterns):
                    continue
                size = int(m.group(2), 16)
                if ".str" in section:
                    # wait one line for "(size before relaxing)"
                    pending_contrib = (obj, section, size)
                else:
                    commit((obj, section, size))
    if pending_contrib:
        commit(pending_contrib)
    return totals


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "target",
        nargs="?",
        default="build",
        help="build directory or map file (default: build)",
    )
    parser.add_argument(
        "--lib",
        action="append",
        default=[],
        metavar="NAME",
        help="measure the objects of lib<NAME>.a; repeatable "
        f"(default: {', '.join(DEFAULT_LIBS)})",
    )
    parser.add_argument(
        "--match",
        action="append",
        default=[],
        metavar="REGEX",
        help="additional object-path regex to include "
        "(e.g. 'drivers__usb_c' for the Zephyr VBUS driver); repeatable",
    )
    parser.add_argument("--json", action="store_true", help="emit JSON instead of a table")
    args = parser.parse_args()

    libs = args.lib or DEFAULT_LIBS
    lib_patterns = [rf"lib{re.escape(name)}\.a\(" for name in libs]
    patterns = [re.compile(p) for p in lib_patterns + args.match]
    map_path = find_map(Path(args.target))
    totals = parse_map(map_path, patterns)
    if not totals:
        sys.exit(f"error: no matching objects found in {map_path}")

    def flash(t):
        return t["code"] + t["rodata"] + t["strings"] + t["data"]

    def ram(t):
        return t["data"] + t["bss"]

    grand = {b: sum(t[b] for t in totals.values()) for b in BUCKETS}

    if args.json:
        out = {
            "map": str(map_path),
            "objects": {
                obj: {**{b: t[b] for b in BUCKETS}, "flash": flash(t), "ram": ram(t)}
                for obj, t in sorted(totals.items())
            },
            "total": {**grand, "flash": flash(grand), "ram": ram(grand)},
        }
        print(json.dumps(out, indent=2))
        return

    name_width = max(len(obj) for obj in totals) + 2
    header = f"{'object':{name_width}s}" + "".join(f"{b:>9s}" for b in BUCKETS) + f"{'flash':>9s}{'ram':>7s}"
    print(f"map: {map_path}")
    print(header)
    print("-" * len(header))
    for obj, t in sorted(totals.items(), key=lambda kv: -flash(kv[1])):
        print(
            f"{obj:{name_width}s}"
            + "".join(f"{t[b]:9d}" for b in BUCKETS)
            + f"{flash(t):9d}{ram(t):7d}"
        )
    print("-" * len(header))
    print(
        f"{'TOTAL':{name_width}s}"
        + "".join(f"{grand[b]:9d}" for b in BUCKETS)
        + f"{flash(grand):9d}{ram(grand):7d}"
    )


if __name__ == "__main__":
    main()
