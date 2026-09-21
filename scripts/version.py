Import("env")

import subprocess


def git_short_sha():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unknown"


env.Append(
    CPPDEFINES=[
        ("TRACKER_GIT_SHA", env.StringifyMacro(git_short_sha())),
    ]
)
