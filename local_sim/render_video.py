"""Render an MP4 of a local_sim flight: top-down course view, the actual flown trajectory drawn
live and colored by ground-truth clearance, a stats overlay, and a "MISSION COMPLETE" outro.

Not a camera-feed video -- this is a top-down replay built from the same decision-logic +
geometry data local_sim already validates (see local_sim/README.md for what that does and
doesn't cover). Needs Pillow (already present) and the system `ffmpeg` binary to encode.

    cd local_sim && python3 render_video.py ../route.json ../diagrams/course-run.mp4
"""
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim as S  # noqa: E402
from course import BOULDERS, HALF, LANDING_XY, LAUNCH_XY, WALL_BOXES  # noqa: E402

SCALE = 100.0          # px per metre
MARGIN = 60
STAGE = int(2 * HALF * SCALE)                 # the square floor, in px
W, H = STAGE + 2 * MARGIN, STAGE + 2 * MARGIN + 160   # + bottom stats bar
W, H = W + (W % 2), H + (H % 2)               # libx264 needs even dimensions
FPS = 20
SPEEDUP = 2.0          # 2x playback: a ~95s sim flight becomes a ~48s video

BG = (241, 237, 226)
SURFACE = (255, 253, 248)
WALL = (60, 52, 38)
BOULDER = (138, 106, 74)
GOAL = (31, 122, 92)
GRID = (32, 28, 20, 24)
PLAN = (207, 198, 174)
INK = (32, 28, 20)
INK_DIM = (107, 99, 84)
GOOD, WARNING, CRITICAL = (12, 163, 12), (250, 178, 25), (208, 59, 59)

FONT_DIR = Path("/System/Library/Fonts/Supplemental")


def font(name, size):
    try:
        return ImageFont.truetype(str(FONT_DIR / name), size)
    except OSError:
        return ImageFont.load_default()


F_TITLE = font("Arial Bold.ttf", 26)
F_LABEL = font("Arial.ttf", 15)
F_MONO = font("Courier New.ttf", 15)
F_BANNER = font("Arial Bold.ttf", 46)


def to_px(x, y):
    return (MARGIN + (x + HALF) * SCALE, MARGIN + (HALF - y) * SCALE)


def status_color(clr):
    if clr is None or not math.isfinite(clr):
        return INK_DIM
    if clr < 0.0:
        return CRITICAL
    if clr < 0.15:
        return WARNING
    return GOOD


def draw_base(route):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img, "RGBA")
    d.rectangle([MARGIN, MARGIN, MARGIN + STAGE, MARGIN + STAGE], fill=SURFACE)
    for i in range(0, STAGE + 1, int(0.5 * SCALE)):
        d.line([(MARGIN + i, MARGIN), (MARGIN + i, MARGIN + STAGE)], fill=GRID)
        d.line([(MARGIN, MARGIN + i), (MARGIN + STAGE, MARGIN + i)], fill=GRID)
    d.rectangle([MARGIN, MARGIN, MARGIN + STAGE, MARGIN + STAGE], outline=WALL, width=3)

    for _, xmin, xmax, ymin, ymax in WALL_BOXES:
        p0, p1 = to_px(xmin, ymax), to_px(xmax, ymin)
        d.rectangle([p0, p1], fill=WALL)
    for cx, cy, r in BOULDERS:
        p0, p1 = to_px(cx - r, cy + r), to_px(cx + r, cy - r)
        d.ellipse([p0, p1], fill=BOULDER, outline=WALL)

    lx, ly = to_px(*LANDING_XY)
    for r_m in (0.76, 0.51, 0.23):
        rp = r_m * SCALE
        d.ellipse([lx - rp, ly - rp, lx + rp, ly + rp], outline=GOAL, width=2)
    d.text((lx, ly + 90), "LANDING", font=F_LABEL, fill=INK, anchor="mm")

    plan_px = [to_px(*LAUNCH_XY)]
    for f, l in route:
        # route legs are (forward, left) relative to launch heading (course +y = forward)
        plan_px.append(to_px(LAUNCH_XY[0] - l, LAUNCH_XY[1] + f))
    d.line(plan_px, fill=PLAN, width=2)

    sx, sy = to_px(*LAUNCH_XY)
    d.rectangle([sx - 6, sy - 6, sx + 6, sy + 6], fill=INK)
    d.text((sx, sy + 20), "LAUNCH", font=F_LABEL, fill=INK, anchor="mm")

    d.text((MARGIN, 14), "Dead Reckoning · DroneHacks 2026", font=F_TITLE, fill=(122, 54, 17))
    # north arrow + scale bar
    nx, ny = MARGIN + STAGE + 20, MARGIN + 10
    d.line([(nx, ny + 26), (nx, ny)], fill=INK, width=2)
    d.polygon([(nx, ny - 6), (nx + 6, ny + 8), (nx - 6, ny + 8)], fill=INK)
    d.text((nx, ny + 40), "N", font=F_LABEL, fill=INK_DIM, anchor="mm")
    bx, by = MARGIN, MARGIN + STAGE + 20
    d.line([(bx, by), (bx + SCALE, by)], fill=INK, width=2)
    d.text((bx + SCALE / 2, by + 14), "1 m", font=F_LABEL, fill=INK_DIM, anchor="mm")
    return img


def draw_stats_bar(img, t, flight_s, phase, clr, worst_so_far, result=None):
    d = ImageDraw.Draw(img, "RGBA")
    bar_y = MARGIN + STAGE + 2
    d.rectangle([0, bar_y, W, H], fill=SURFACE)
    d.line([(0, bar_y), (W, bar_y)], fill=PLAN, width=1)
    frac = max(0.0, min(1.0, t / flight_s)) if flight_s > 0 else 0.0
    bar_x0, bar_x1, bar_y0 = MARGIN, W - MARGIN, bar_y + 20
    d.line([(bar_x0, bar_y0), (bar_x1, bar_y0)], fill=PLAN, width=4)
    d.line([(bar_x0, bar_y0), (bar_x0 + frac * (bar_x1 - bar_x0), bar_y0)], fill=(198, 89, 30), width=4)

    c = status_color(clr)
    clr_txt = f"{clr:.3f} m" if (clr is not None and math.isfinite(clr)) else "—"
    worst_txt = f"{worst_so_far:.3f} m" if math.isfinite(worst_so_far) else "—"
    line1 = f"t={t:5.1f}s / {flight_s:.1f}s   phase={phase:<8s} clearance={clr_txt:<9s} worst-so-far={worst_txt}"
    d.text((MARGIN, bar_y0 + 14), line1, font=F_MONO, fill=INK)
    d.ellipse([W - MARGIN - 160, bar_y0 + 14, W - MARGIN - 146, bar_y0 + 28], fill=c)
    d.text((W - MARGIN - 136, bar_y0 + 13), "clearance status", font=F_LABEL, fill=INK_DIM)
    if result:
        d.text((MARGIN, bar_y0 + 38), f"result: {result}", font=F_LABEL, fill=INK_DIM)
    return img


def main():
    route_path = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parents[1] / "route.json")
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parents[1] / "diagrams" / "course-run.mp4"
    route, _ = S.load_route(route_path)

    print("running mission...")
    r = S.run(route_path)
    log = r["log"]  # (t, phase, cx, cy, clr), dt=0.05
    print(f"  result={r['result']} legs={r['legs_done']}/{r['legs']} flight={r['flight_s']:.1f}s "
          f"worst_clearance={r['worst_clearance_m']:.3f}m")

    step = max(1, round(SPEEDUP))
    samples = log[::step]

    base = draw_base(route)
    trail = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    trail_draw = ImageDraw.Draw(trail)

    tmpdir = Path(tempfile.mkdtemp(prefix="course_run_frames_"))
    print(f"rendering {len(samples)} frames to {tmpdir} ...")
    worst_so_far = math.inf
    prev_px = None
    frame_i = 0
    for t, phase, cx, cy, clr in samples:
        if math.isfinite(clr):
            worst_so_far = min(worst_so_far, clr)
        px, py = to_px(cx, cy)
        if prev_px is not None:
            trail_draw.line([prev_px, (px, py)], fill=status_color(clr) + (255,), width=5)
        prev_px = (px, py)

        frame = base.copy()
        frame.paste(trail, (0, 0), trail)
        d = ImageDraw.Draw(frame, "RGBA")
        c = status_color(clr)
        d.ellipse([px - 11, py - 11, px + 11, py + 11], outline=c, width=2)
        d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=c, outline=SURFACE, width=2)
        draw_stats_bar(frame, t, r["flight_s"], phase, clr, worst_so_far)
        frame.save(tmpdir / f"frame_{frame_i:06d}.png")
        frame_i += 1

    # outro: hold the final frame with a result banner
    outro = base.copy()
    outro.paste(trail, (0, 0), trail)
    d = ImageDraw.Draw(outro, "RGBA")
    px, py = prev_px
    c = status_color(r["worst_clearance_m"])
    d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=c, outline=SURFACE, width=2)
    draw_stats_bar(outro, r["flight_s"], r["flight_s"], "done", r["worst_clearance_m"], worst_so_far, result=r["result"])
    banner = "MISSION COMPLETE" if r["result"] == "complete" else f"RESULT: {r['result']}".upper()
    bbox = d.textbbox((0, 0), banner, font=F_BANNER)
    bw, bh = bbox[2] - bbox[0], bbox[3] - bbox[1]
    bx, by = (STAGE - bw) / 2 + MARGIN, MARGIN + STAGE / 2 - bh
    d.rectangle([bx - 20, by - 14, bx + bw + 20, by + bh + 14], fill=(255, 253, 248, 235), outline=(122, 54, 17), width=2)
    d.text((bx, by), banner, font=F_BANNER, fill=(122, 54, 17))
    for _ in range(int(FPS * 2.5)):
        outro.save(tmpdir / f"frame_{frame_i:06d}.png")
        frame_i += 1

    print(f"encoding {frame_i} frames with ffmpeg...")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-y", "-framerate", str(FPS), "-i", str(tmpdir / "frame_%06d.png"),
          "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-movflags", "+faststart", str(out_path)]
    subprocess.run(cmd, check=True, capture_output=True)
    shutil.rmtree(tmpdir, ignore_errors=True)
    print(f"wrote {out_path} ({out_path.stat().st_size / 1e6:.1f} MB, {frame_i / FPS:.1f}s @ {FPS}fps, "
          f"{SPEEDUP:.0f}x speed)")


if __name__ == "__main__":
    main()
