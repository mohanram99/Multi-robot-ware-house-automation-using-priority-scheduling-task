#to change in theh world file

import re

WORLD_FILE = "warehouse.world"   # ← change to your full path if needed

with open(WORLD_FILE) as f:
    c = f.read()

def edit_model(c, name, fn):
    i = c.find(f'<model name="{name}">')
    if i == -1:
        print("NOT FOUND:", name)
        return c
    e = c.find('</model>', i) + len('</model>')
    return c[:i] + fn(c[i:e]) + c[e:]

# ════════════════════════════════════════════════════════════
# 1. ROW 1 FRAMES — shrink to 1.0m tall (top just above low board)
#    old: size 2.0x0.08x2.0 at z=1.0 (top=2.0)
#    new: size 2.0x0.08x1.0 at z=0.5 (top=1.0)
# ════════════════════════════════════════════════════════════
for col, sx in [('1','-9'),('2','-3'),('3','3'),('4','9')]:
    def fix_r1(b, sx=sx):
        b = b.replace(f'<pose>{sx} 6 1.0 0 0 0</pose>',
                      f'<pose>{sx} 6 0.5 0 0 0</pose>')
        b = b.replace('<size>2.0 0.08 2.0</size>',
                      '<size>2.0 0.08 1.0</size>')
        return b
    c = edit_model(c, f'shelf_1_{col}_frame', fix_r1)

# ════════════════════════════════════════════════════════════
# 2. ROW 2 & 3 FRAMES — shrink so frame top = high board top
#    high board: z=1.5, thickness 0.05 → top = 1.525
#    new frame: size 2.0x0.08x1.53 at z=0.765 (top = 1.53)
# ════════════════════════════════════════════════════════════
for row, sy in [('2','0'),('3','-6')]:
    for col, sx in [('1','-9'),('2','-3'),('3','3'),('4','9')]:
        def fix_fr(b, sx=sx, sy=sy):
            b = b.replace(f'<pose>{sx} {sy} 1.0 0 0 0</pose>',
                          f'<pose>{sx} {sy} 0.765 0 0 0</pose>')
            b = b.replace('<size>2.0 0.08 2.0</size>',
                          '<size>2.0 0.08 1.53</size>')
            return b
        c = edit_model(c, f'shelf_{row}_{col}_frame', fix_fr)

# ════════════════════════════════════════════════════════════
# 3. WIDEN BOARDS to 1.2m depth (row2 low, row2 high, row3 low,
#    row3 high — idempotent, skips boards already 1.2)
#    Replaces BOTH visual and collision sizes in each board.
# ════════════════════════════════════════════════════════════
board_names = []
for row in ['2','3']:
    for col in ['1','2','3','4']:
        board_names.append(f'shelf_{row}_{col}_board_low')
        board_names.append(f'shelf_{row}_{col}_board_high')

for name in board_names:
    def fix_board(b):
        return b.replace('<size>2.0 0.85 0.05</size>',
                         '<size>2.0 1.2 0.05</size>')
    c = edit_model(c, name, fix_board)

# ════════════════════════════════════════════════════════════
# 4. MOVE LOW BOXES to board edges (same pattern as row 2 high)
#    Row 2 low: y ±0.15 → ±0.45
#    Row 3 low: y -6.15 → -6.45 and -5.85 → -5.55
# ════════════════════════════════════════════════════════════
def move_box_y(c, name, old_y, new_y):
    def fix(b):
        return re.sub(
            rf'(<pose>\S+ ){re.escape(old_y)}( )',
            rf'\g<1>{new_y}\g<2>',
            b, count=1)
    return edit_model(c, name, fix)

for col in ['1','2','3','4']:
    # Row 2 low
    c = move_box_y(c, f'box_2_{col}_low_la', '-0.15', '-0.45')
    c = move_box_y(c, f'box_2_{col}_low_lb',  '0.15',  '0.45')
    c = move_box_y(c, f'box_2_{col}_low_ra', '-0.15', '-0.45')
    c = move_box_y(c, f'box_2_{col}_low_rb',  '0.15',  '0.45')
    # Row 3 low
    c = move_box_y(c, f'box_3_{col}_low_la', '-6.15', '-6.45')
    c = move_box_y(c, f'box_3_{col}_low_lb', '-5.85', '-5.55')
    c = move_box_y(c, f'box_3_{col}_low_ra', '-6.15', '-6.45')
    c = move_box_y(c, f'box_3_{col}_low_rb', '-5.85', '-5.55')

with open(WORLD_FILE, "w") as f:
    f.write(c)

print("All changes applied successfully.")
