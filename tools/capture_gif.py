#!/usr/bin/env python3
"""
Make a GIF of a Tessera simulation.

Drives the Test_capture executable (which renders a fixed grid region into
an offscreen window and dumps PPM frames), then assembles those frames into a GIF.

The capture is deterministic: every single simulation step is captured (stride
is fixed at 1 — no skipped frames), so the same parameters always produce the
same GIF. No ImGui, no mouse cursor, static camera, simulation at full speed.

By default the GIF is saved into your Windows Pictures folder
(%USERPROFILE%\\Pictures\\Tessera\\capture.gif) — pass --out to save elsewhere.

Examples
--------
  # 300 steps, 700x400, show grid region (0,0)-(350,200) -> saved to Pictures
  python tools/capture_gif.py --exe out/build/x64-release/Test_capture.exe \
      --stop 300 --res 700x400 --region 0 0 350 200 --delay 60

  # explicit output path
  python tools/capture_gif.py --exe out/build/x64-release/Test_capture.exe \
      --stop 300 --res 700x400 --region 0 0 350 200 --out docs/demo.gif

Parameters
----------
  --stop     last simulation step to capture
  --res WxH  output resolution in pixels
  --region   x0 y0 x1 y1  region to show, in grid (tile) coordinates
  --grid     field size in tiles (square)
  --delay    milliseconds between GIF frames (playback speed)
  --colors   GIF palette size (fewer = smaller file)
  --out      output GIF path (default: Pictures\\Tessera\\capture.gif)
  --exe      path to Test_capture executable
  --workdir  working directory for the exe (must contain patterns/ and Shaders/)

Note: the step between captured frames is fixed at 1 (every generation is
captured) — this isn't configurable, so the same run always produces the
same number of frames.
"""
import argparse
import os
import subprocess
import sys
import tempfile
import glob

# Step between captured frames is intentionally NOT configurable: capturing
# every generation keeps output reproducible and removes a knob that could
# silently change what a "step" means between runs.
STRIDE = 1


def parse_res(s):
    try:
        w, h = s.lower().split("x")
        return int(w), int(h)
    except Exception:
        raise argparse.ArgumentTypeError(f"--res must be WxH, e.g. 700x400 (got {s!r})")


def default_out_path():
    """Pictures\\Tessera\\capture.gif under the user's profile (Windows)."""
    home = os.environ.get("USERPROFILE") or os.path.expanduser("~")
    return os.path.join(home, "Pictures", "Tessera", "capture.gif")


def main():
    ap = argparse.ArgumentParser(description="Make a GIF of a Tessera simulation.")
    ap.add_argument("--exe", required=True, help="path to Test_capture executable")
    ap.add_argument("--out", default=None,
                    help="output GIF path (default: Pictures\\Tessera\\capture.gif)")
    ap.add_argument("--stop", type=int, default=300, help="last simulation step to capture")
    ap.add_argument("--res", type=parse_res, default=(600, 400), help="output resolution WxH")
    ap.add_argument("--region", type=int, nargs=4, metavar=("X0", "Y0", "X1", "Y1"),
                    default=[0, 0, 300, 200], help="grid region to show (tile coords)")
    ap.add_argument("--grid", type=int, default=1024, help="field size in tiles (square)")
    ap.add_argument("--delay", type=int, default=60, help="ms between GIF frames")
    ap.add_argument("--colors", type=int, default=128, help="GIF palette size (2-256)")
    ap.add_argument("--workdir", default=None,
                    help="working dir for the exe (defaults to the exe's folder)")
    args = ap.parse_args()

    try:
        from PIL import Image
    except ImportError:
        sys.exit("Pillow is required:  pip install pillow")

    exe = os.path.abspath(args.exe)
    if not os.path.isfile(exe):
        sys.exit(f"executable not found: {exe}")
    workdir = args.workdir or os.path.dirname(exe)

    resW, resH = args.res
    x0, y0, x1, y1 = args.region

    with tempfile.TemporaryDirectory(prefix="tessera_capture_") as tmp:
        # Test_capture arg order:
        #   outDir stopStep stride resW resH x0 y0 x1 y1 gridW gridH
        cmd = [exe, tmp,
               str(args.stop), str(STRIDE),
               str(resW), str(resH),
               str(x0), str(y0), str(x1), str(y1),
               str(args.grid), str(args.grid)]
        print("Running capture:", " ".join(cmd))
        res = subprocess.run(cmd, cwd=workdir, capture_output=True, text=True)
        if res.stderr:
            print(res.stderr.strip())
        if res.returncode != 0:
            sys.exit(f"capture exe failed (exit {res.returncode})")

        frame_paths = sorted(glob.glob(os.path.join(tmp, "frame_*.ppm")))
        if not frame_paths:
            sys.exit("no frames were produced")

        frames = []
        for p in frame_paths:
            img = Image.open(p).convert("RGB")
            frames.append(img.convert("P", palette=Image.ADAPTIVE,
                                      colors=max(2, min(256, args.colors))))

        out = os.path.abspath(args.out or default_out_path())
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        frames[0].save(out, save_all=True, append_images=frames[1:],
                       duration=args.delay, loop=0, optimize=True)

        size_kb = os.path.getsize(out) // 1024
        print(f"Saved {out}  ({resW}x{resH}, {len(frames)} frames, {size_kb} KB)")


if __name__ == "__main__":
    main()
