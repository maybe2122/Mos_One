"""Convert a TensorBoard run into a *local* SwanLab log, viewable offline.

Why this exists: SwanLab 0.8.0's `swanlab convert -t tensorboard --mode local`
is broken. Its TFEvent converter (swanlab/converter/tfb/converter.py) calls
`swanlab.login(relogin=True)` unconditionally *before* `swanlab.init(mode=...)`,
so even a local conversion aborts with:

    AuthenticationError: No API key provided and no stored API key found.

We don't want to depend on the cloud (or patch site-packages, which would risk
the Isaac Sim install — see the wrapt/sentry-sdk version pin). So this script
reuses the official `TFBConverter` but neutralises the stray `login()` call by
monkeypatching `swanlab.login` to a no-op for the duration of the conversion.
Everything else (tag discovery, scalar extraction, per-file runs) is the
upstream code path.

Run with the Isaac Sim venv that has swanlab installed:

    /home/maybe/code/rl/env_isaaclab/bin/python3 tools/swanlab_convert_local.py \
        logs/rsl_rl/mos2026_2_closed_usd/2026-05-17_00-44-23 \
        --out ./swanlog_local --project mos_one-mos

Then serve the result in the browser:

    /home/maybe/code/rl/env_isaaclab/bin/swanlab watch ./swanlog_local
"""

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tb_log_dir", help="Directory containing TensorBoard events.out.tfevents.* files.")
    parser.add_argument("--out", default="./swanlog_local",
                        help="Output directory for the local SwanLab log (default: ./swanlog_local).")
    parser.add_argument("--project", default="mos_one-mos", help="SwanLab project name.")
    parser.add_argument("--types", default="scalar",
                        help="Comma-separated TB data types to convert (scalar,image,audio,text).")
    args = parser.parse_args()

    if not os.path.isdir(args.tb_log_dir):
        print(f"[convert] TensorBoard dir not found: {args.tb_log_dir}", file=sys.stderr)
        return 1

    import swanlab
    from swanlab.converter import TFBConverter

    # Force local mode and disable the broken auth path: the upstream converter
    # calls swanlab.login(relogin=True) per TFEvent file regardless of mode.
    swanlab.login = lambda *a, **k: None  # type: ignore[assignment]

    os.makedirs(args.out, exist_ok=True)
    converter = TFBConverter(
        project=args.project,
        mode="local",
        log_dir=args.out,
        types=args.types,
    )
    converter.run(convert_dir=args.tb_log_dir)

    print(f"[convert] Done. Local SwanLab log written to: {os.path.abspath(args.out)}")
    print(f"[convert] View it with:\n    swanlab watch {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
