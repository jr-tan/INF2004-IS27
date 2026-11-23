#!/usr/bin/env python3
"""Automated BARR-C:2018 checks over app_program/ (the rules a tool can
check; the rest are code-review rules)."""
import glob
import os
import re
import sys

root = sys.argv[1]
os.chdir(root)
files = sorted(glob.glob("**/*.[ch]", recursive=True))


def code_only(text):
    text = re.sub(r"/\*.*?\*/", lambda m: " " * len(m.group(0)), text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    text = re.sub(r'"(\\.|[^"\\])*"', '""', text)
    text = re.sub(r"'(\\.|[^'\\])*'", "''", text)
    return text


checks = {
    "1.2.a line > 80 chars": [],
    "3.5.a tab character": [],
    "3.6.a CR-LF line ending": [],
    "1.7.a/b auto/register keyword": [],
    "1.7.c goto": [],
    "5.2.b short/long keyword": [],
    "8.5.b abort/exit/setjmp/longjmp": [],
    "2.1.b backslash inside a comment": [],
    "3.3.c no end-of-file comment": [],
    "4.1.a file name not lower case": [],
    "4.2.b header without include guard": [],
    "8.4.c infinite loop not for (;;)": [],
}
for f in files:
    raw = open(f, "rb").read()
    text = raw.decode("utf-8")
    code = code_only(text)
    lines = text.split("\n")
    if b"\r\n" in raw:
        checks["3.6.a CR-LF line ending"].append(f)
    if f != f.lower():
        checks["4.1.a file name not lower case"].append(f)
    if "/*** end of file ***/" not in text:
        checks["3.3.c no end-of-file comment"].append(f)
    if f.endswith(".h") and not re.search(r"#ifndef\s+\w+\s*\n\s*#define", text):
        checks["4.2.b header without include guard"].append(f)
    for n, l in enumerate(lines, 1):
        if len(l) > 80:
            checks["1.2.a line > 80 chars"].append(f"{f}:{n}")
        if "\t" in l:
            checks["3.5.a tab character"].append(f"{f}:{n}")
    for n, l in enumerate(code.split("\n"), 1):
        if re.search(r"\b(auto|register)\b", l):
            checks["1.7.a/b auto/register keyword"].append(f"{f}:{n}")
        if re.search(r"\bgoto\b", l):
            checks["1.7.c goto"].append(f"{f}:{n}")
        if re.search(r"\b(short|long)\b", l):
            checks["5.2.b short/long keyword"].append(f"{f}:{n}: {l.strip()[:60]}")
        if re.search(r"\b(abort|exit|setjmp|longjmp)\s*\(", l):
            checks["8.5.b abort/exit/setjmp/longjmp"].append(f"{f}:{n}")
        if re.search(r"\bwhile\s*\(\s*(1|TRUE|true)\s*\)", l):
            checks["8.4.c infinite loop not for (;;)"].append(f"{f}:{n}")
    in_c = False
    for n, l in enumerate(lines, 1):
        stripped = l.strip()
        if "/*" in l and "*/" not in l.split("/*", 1)[1]:
            in_c = True
        if (in_c or "/*" in l) and stripped.endswith("\\") and "*/" not in l:
            checks["2.1.b backslash inside a comment"].append(f"{f}:{n}")
        if "*/" in l:
            in_c = False

print(f"{len(files)} files scanned")
for rule, hits in checks.items():
    print(f"  {'OK ' if not hits else 'FIX'} {rule}: {len(hits)} {hits[:6]}")
