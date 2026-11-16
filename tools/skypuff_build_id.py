'''
*******************************************************
 Skypuff: the build id of a module firmware
*******************************************************
 PlatformIO extra script, pre:. The module reports its build id, the board decides
 by it whether the release has another firmware, and a crash log names the .elf by it.

 The id comes from the source, not from the time of the build: the mLRS commit and
 the env. The same commit gives the same id on any machine and after any clean, and
 with no date in it the same firmware. Even ids are committed source; odd ones have
 changes which aren't committed, hashed in too: such a build never takes the id of a
 release and keeps its id while the changes stay the same. Changes are the diff of
 the tracked files and the files of mLRS/ not in git yet (the stray tools/ of the
 fork root don't count).

 Writes skypuff_build_id.h into the build folder of the env, see skypuff_crash.h.
*******************************************************
'''
Import("env")

import hashlib
import os
import subprocess


def git(*args):
    return subprocess.run(["git", *args], cwd=env.subst("$PROJECT_DIR"), capture_output=True, check=True).stdout


def fnv1a(data):
    h = 2166136261
    for c in data:
        h = ((h ^ c) * 16777619) & 0xFFFFFFFF
    return h


pioenv = env["PIOENV"]
try:
    commit = git("rev-parse", "HEAD").decode().strip()
    changes = git("diff", "HEAD", "--")
    for name in git("ls-files", "--others", "--exclude-standard", "-z", "--", "mLRS").split(b"\0"):
        if name:
            with open(os.path.join(env.subst("$PROJECT_DIR"), name.decode()), "rb") as f:
                changes += b"\0new " + name + b"\0" + f.read()
except (OSError, subprocess.CalledProcessError):
    commit, changes = "unknown", b"no git"

if changes:
    build_id = fnv1a(("%s:%s:" % (commit, pioenv)).encode() + hashlib.sha256(changes).digest()) | 1
    source = commit[:8] + "+changes"
else:
    build_id = fnv1a(("%s:%s" % (commit, pioenv)).encode()) & ~1
    source = commit[:8]

header_dir = env.subst("$BUILD_DIR")
os.makedirs(header_dir, exist_ok=True)
header = os.path.join(header_dir, "skypuff_build_id.h")
text = ("// Made by tools/skypuff_build_id.py, %s %s\n"
        "#define SKYPUFF_BUILD_ID 0x%08xu\n"
        "#define SKYPUFF_BUILD_ID_STR \"%08x\"\n" % (pioenv, source, build_id, build_id))
old = open(header).read() if os.path.exists(header) else None
if old != text:  # untouched, nothing builds again
    with open(header, "w") as f:
        f.write(text)
env.Append(CPPPATH=[header_dir])
print("Skypuff build id %08x (%s, %s)" % (build_id, pioenv, source))
