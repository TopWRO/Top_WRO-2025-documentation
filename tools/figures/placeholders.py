"""Specification placeholders for photos that must be taken on the real car.

Each placeholder is written at the FINAL path the README already links to, so
replacing a photo means overwriting one file - no README edits needed.
The same list drives docs/photo-checklist.md.
"""
from PIL import Image, ImageDraw, ImageFont

from style import REPO

W, H = 1600, 1200   # 4:3, matches the requested photo aspect ratio
MARKER = b"WRO-DOC-PLACEHOLDER"   # JPEG comment that identifies a generated placeholder

# (path relative to repo, title, shot specification)
PHOTOS = [
    ("v-photos/front.jpg", "Vehicle - front", "Camera at chassis height, lens centred on the front ToF (F); plain light background"),
    ("v-photos/back.jpg", "Vehicle - back", "Camera at chassis height; rear ToF (B) and drive gear visible"),
    ("v-photos/left.jpg", "Vehicle - left side", "Full side profile; L1 / L2 sensors visible; wheels straight"),
    ("v-photos/right.jpg", "Vehicle - right side", "Full side profile; R1 / R2 sensors visible; wheels straight"),
    ("v-photos/top.jpg", "Vehicle - top", "Straight down from ~60 cm; whole car in frame; cables tidy"),
    ("v-photos/bottom.jpg", "Vehicle - bottom", "Straight up; colour sensor (TCS34725) and drivetrain visible"),
    ("t-photos/official.jpg", "Team - official photo", "All members + coach, team name readable, with the vehicle"),
    ("t-photos/funny.jpg", "Team - fun photo", "Required by WRO: one informal team photo"),
    ("docs/assets/photos/drivetrain.jpg", "Detail - drivetrain", "DXL ID 1 + 1:3 gear train + rear axle; show gears/differential if present"),
    ("docs/assets/photos/steering.jpg", "Detail - steering", "DXL ID 2 + linkage, wheels at full lock (±40° command)"),
    ("docs/assets/photos/sensor_hub.jpg", "Detail - Pico 2 sensor hub", "Pico 2 + BNO085 + wiring; pin labels readable"),
    ("docs/assets/photos/tof_side_pair.jpg", "Detail - side ToF pair", "R1 / R2 mounts with a ruler showing the 14.8 cm spacing"),
    ("docs/assets/photos/tof_front_back.jpg", "Detail - front / rear ToF", "F (VL53L4CX) and B mounts, height above floor visible"),
    ("docs/assets/photos/color_sensor.jpg", "Detail - floor colour sensor", "TCS34725 under the chassis with LED on; ground clearance visible"),
    ("docs/assets/photos/camera_mount.jpg", "Detail - camera mount", "Side view with ruler: mounting height and tilt angle"),
    ("docs/assets/photos/power.jpg", "Detail - power system", "Battery, switch, fuse, regulators and their wiring"),
    ("docs/assets/photos/u2d2.jpg", "Detail - U2D2 and DXL bus", "U2D2 adapter and daisy-chained DYNAMIXEL cables"),
    ("docs/assets/photos/wiring_overview.jpg", "Detail - wiring overview", "Top view with top deck removed; all cables visible"),
    ("docs/assets/photos/start_ui.jpg", "Detail - start-up UI", "Sense HAT LED showing the CW / CCW choice before start"),
    ("docs/assets/photos/on_track_open.jpg", "Action - Open Challenge", "Car on the mat mid-corner; random wall layout visible"),
    ("docs/assets/photos/on_track_obstacle.jpg", "Action - Obstacle Challenge", "Car passing a red or green pillar"),
    ("docs/assets/photos/parking.jpg", "Action - parallel parking", "Car parked inside the magenta parking lot"),
]


def _font(size, bold=False):
    candidates = (["/System/Library/Fonts/Supplemental/Arial Bold.ttf", "DejaVuSans-Bold.ttf"] if bold else
                  ["/System/Library/Fonts/Supplemental/Arial.ttf", "DejaVuSans.ttf"])
    for c in candidates:
        try:
            return ImageFont.truetype(c, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


def _wrap(draw, text, font, width):
    words, lines, cur = text.split(), [], ""
    for w in words:
        test = (cur + " " + w).strip()
        if draw.textlength(test, font=font) <= width:
            cur = test
        else:
            lines.append(cur)
            cur = w
    return lines + [cur]


def make(path, title, spec):
    img = Image.new("RGB", (W, H), "#f2f2f0")
    d = ImageDraw.Draw(img)
    # dashed frame
    for x in range(60, W - 60, 40):
        d.line([(x, 60), (x + 22, 60)], fill="#b8b7b2", width=4)
        d.line([(x, H - 60), (x + 22, H - 60)], fill="#b8b7b2", width=4)
    for y in range(60, H - 60, 40):
        d.line([(60, y), (60, y + 22)], fill="#b8b7b2", width=4)
        d.line([(W - 60, y), (W - 60, y + 22)], fill="#b8b7b2", width=4)
    # camera glyph
    cx, cy = W // 2, 380
    d.rounded_rectangle([cx - 110, cy - 70, cx + 110, cy + 80], radius=22, outline="#8a8984", width=8)
    d.ellipse([cx - 48, cy - 38, cx + 48, cy + 58], outline="#8a8984", width=8)
    d.rectangle([cx - 50, cy - 95, cx + 10, cy - 70], fill="#8a8984")
    d.text((cx, 560), "PHOTO PENDING", font=_font(54, True), fill="#e34948", anchor="mm")
    d.text((cx, 650), title, font=_font(64, True), fill="#0b0b0b", anchor="mm")
    y = 740
    for ln in _wrap(d, spec, _font(38), W - 300):
        d.text((cx, y), ln, font=_font(38), fill="#52514e", anchor="mm")
        y += 54
    d.text((cx, H - 130), f"replace: {path}  (4:3, ≥ 2000 px wide)", font=_font(30), fill="#8a8984", anchor="mm")
    out = REPO / path
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out, quality=80, comment=MARKER)


def main():
    for path, title, spec in PHOTOS:
        out = REPO / path
        # never overwrite a real photo: only (re)write files that are placeholders
        if out.exists() and MARKER not in out.read_bytes()[:4096]:
            print("  kept real photo", path)
            continue
        make(path, title, spec)
    print(f"  wrote {len(PHOTOS)} photo placeholders")


if __name__ == "__main__":
    main()
