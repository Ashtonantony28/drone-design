#!/usr/bin/env python3
"""Summarise a kicad-cli DRC JSON report: counts by severity and category."""
import collections
import json
import sys

r = json.load(open(sys.argv[1]))
by = collections.Counter((v["severity"], v["type"]) for v in r.get("violations", []))
errors = sum(n for (sev, _), n in by.items() if sev == "error")
warnings = sum(n for (sev, _), n in by.items() if sev == "warning")
print("DRC: %d errors, %d warnings, %d unconnected, %d schematic-parity" % (
    errors, warnings, len(r.get("unconnected_items", [])), len(r.get("schematic_parity", []))))
for (sev, kind), n in sorted(by.items()):
    print("  %-7s %-28s %d" % (sev, kind, n))
if len(sys.argv) > 2:
    for v in r.get("violations", []):
        if v["severity"] == "error" or sys.argv[2] == "all":
            print("   ", v["type"], "|", v["description"], "|", "; ".join("%s @(%.2f,%.2f)" % (i["description"], i["pos"]["x"] - 100, i["pos"]["y"] - 100) for i in v["items"]))
    for u in r.get("unconnected_items", []):
        print("    unconnected |", "; ".join("%s @(%.2f,%.2f)" % (i["description"], i["pos"]["x"] - 100, i["pos"]["y"] - 100) for i in u["items"]))
