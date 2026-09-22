#!/usr/bin/env python3
"""Генератор кронштейна для камеры Razer Kiyo Pro.

Выдаёт бинарный STL, готовый к нарезке. Деталь собрана из нескольких
замкнутых тел, которые намеренно пересекаются: слайсер объединяет их сам,
и это избавляет от полноценной CSG.

Запуск:  python hardware/make_mount.py
"""

import math
import struct
import sys
import zlib

# --------------------------------------------------------------------------
# Параметры детали, мм.
# --------------------------------------------------------------------------

TILT_DEG = 30.0      # наклон площадки вниз: под этим углом камера смотрит в зону

BASE_W = 80.0        # основание: ширина (X)
BASE_D = 60.0        # глубина (Y)
BASE_T = 6.0         # толщина

SLOT_W = 8.0         # паз под болт M5 с Т-гайкой профиля 2020
SLOT_CENTRE = 20.0   # смещение оси паза от центра
SLOT_FRONT = 6.0     # насколько паз не доходит до переднего края

WALL_W = 60.0        # задняя стойка
WALL_T = 8.0
WALL_H = 46.0

PLAT_W = 62.0        # площадка под камеру
PLAT_D = 56.0
PLAT_T = 6.0

HOLE_D = 7.0         # проход под винт 1/4" (ISO 1222); с запасом и под M6
CBORE_D = 14.0       # утопление головки винта снизу
CBORE_T = 4.0

RIB_H = 3.0          # упор, не дающий камере провернуться на одном винте
RIB_D = 4.0

GUSSET_T = 6.0       # рёбра жёсткости
GUSSET_X = 24.0
GUSSET_BITE = 0.4   # врезание ребра в площадку, чтобы шов был сплошным

SEG = 64             # сегментов на окружность

OUTPUT_STL = "hardware/kiyo_pro_mount.stl"
OUTPUT_PNG = "hardware/kiyo_pro_mount.png"


# --------------------------------------------------------------------------
# Примитивы. Треугольник - кортеж из трёх точек, обход против часовой стрелки
# при взгляде снаружи (нормаль наружу).
# --------------------------------------------------------------------------

def box(x0, x1, y0, y1, z0, z1):
    """Замкнутый параллелепипед."""
    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    quads = [
        (0, 3, 2, 1),  # низ
        (4, 5, 6, 7),  # верх
        (0, 1, 5, 4),  # -Y
        (2, 3, 7, 6),  # +Y
        (1, 2, 6, 5),  # +X
        (3, 0, 4, 7),  # -X
    ]
    tris = []
    for a, b, c, d in quads:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def _rect_boundary_point(angle, half_w, half_d):
    """Точка на границе прямоугольника по лучу из центра под данным углом."""
    cx, sy = math.cos(angle), math.sin(angle)
    tx = half_w / abs(cx) if abs(cx) > 1e-12 else float("inf")
    ty = half_d / abs(sy) if abs(sy) > 1e-12 else float("inf")
    t = min(tx, ty)
    return (cx * t, sy * t)


def plate_with_hole(w, d, z0, z1, hole_d, seg=SEG):
    """Прямоугольная плита с круглым отверстием по центру.

    Торцы триангулируются кольцом между окружностью и границей
    прямоугольника. Углы прямоугольника добавляются в набор углов явно,
    иначе они срезались бы при дискретизации.
    """
    half_w, half_d = w * 0.5, d * 0.5
    r = hole_d * 0.5

    angles = [2.0 * math.pi * i / seg for i in range(seg)]
    for corner in (
        math.atan2(half_d, half_w),
        math.atan2(half_d, -half_w),
        math.atan2(-half_d, -half_w),
        math.atan2(-half_d, half_w),
    ):
        angles.append(corner % (2.0 * math.pi))
    angles = sorted(set(round(a, 9) for a in angles))

    circle = [(r * math.cos(a), r * math.sin(a)) for a in angles]
    rect = [_rect_boundary_point(a, half_w, half_d) for a in angles]

    tris = []
    n = len(angles)
    for i in range(n):
        j = (i + 1) % n
        p0, p1 = circle[i], circle[j]
        q0, q1 = rect[i], rect[j]

        # Низ (нормаль -Z).
        tris.append(((p0[0], p0[1], z0), (q0[0], q0[1], z0), (q1[0], q1[1], z0)))
        tris.append(((p0[0], p0[1], z0), (q1[0], q1[1], z0), (p1[0], p1[1], z0)))

        # Верх (нормаль +Z).
        tris.append(((p0[0], p0[1], z1), (q1[0], q1[1], z1), (q0[0], q0[1], z1)))
        tris.append(((p0[0], p0[1], z1), (p1[0], p1[1], z1), (q1[0], q1[1], z1)))

        # Внешняя боковая стенка.
        tris.append(((q0[0], q0[1], z0), (q0[0], q0[1], z1), (q1[0], q1[1], z1)))
        tris.append(((q0[0], q0[1], z0), (q1[0], q1[1], z1), (q1[0], q1[1], z0)))

        # Стенка отверстия.
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z1), (p0[0], p0[1], z1)))
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z1)))

    return tris


def prism_yz(profile, x0, x1):
    """Призма: плоский выпуклый профиль в плоскости YZ, вытянутый вдоль X."""
    n = len(profile)
    tris = []

    for i in range(1, n - 1):
        a, b, c = profile[0], profile[i], profile[i + 1]
        tris.append(((x0, a[0], a[1]), (x0, c[0], c[1]), (x0, b[0], b[1])))
        tris.append(((x1, a[0], a[1]), (x1, b[0], b[1]), (x1, c[0], c[1])))

    for i in range(n):
        a = profile[i]
        b = profile[(i + 1) % n]
        tris.append(((x0, a[0], a[1]), (x0, b[0], b[1]), (x1, b[0], b[1])))
        tris.append(((x0, a[0], a[1]), (x1, b[0], b[1]), (x1, a[0], a[1])))

    return tris


def rotate_x(tris, deg):
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    out = []
    for tri in tris:
        out.append(tuple((x, y * ca - z * sa, y * sa + z * ca) for x, y, z in tri))
    return out


def translate(tris, dx, dy, dz):
    return [tuple((x + dx, y + dy, z + dz) for x, y, z in tri) for tri in tris]


# --------------------------------------------------------------------------
# Сборка кронштейна
# --------------------------------------------------------------------------

def build():
    tris = []

    half_w = BASE_W * 0.5
    y_back = -BASE_D * 0.5
    y_front = BASE_D * 0.5

    # --- Основание с двумя пазами, открытыми к заднему краю ---------------
    # Паз открыт с торца, поэтому болт заводится сдвигом: положение
    # кронштейна на профиле правится без полной разборки.
    s0, s1 = SLOT_CENTRE - SLOT_W * 0.5, SLOT_CENTRE + SLOT_W * 0.5
    y_slot_end = y_front - SLOT_FRONT

    for x0, x1 in ((-half_w, -s1), (-s0, s0), (s1, half_w)):
        tris += box(x0, x1, y_back, y_front, 0.0, BASE_T)
    for x0, x1 in ((-s1, -s0), (s0, s1)):
        tris += box(x0, x1, y_slot_end, y_front, 0.0, BASE_T)

    # --- Задняя стойка -----------------------------------------------------
    wall_y1 = y_back + WALL_T
    tris += box(-WALL_W * 0.5, WALL_W * 0.5, y_back, wall_y1, 0.0, WALL_H)

    # --- Площадка под камеру ----------------------------------------------
    # Строится в собственных координатах, затем наклоняется и переносится
    # так, чтобы задний край сел на верх стойки.
    plat = []
    plat += plate_with_hole(PLAT_W, PLAT_D, -PLAT_T * 0.5,
                            -PLAT_T * 0.5 + CBORE_T, CBORE_D)
    plat += plate_with_hole(PLAT_W, PLAT_D, -PLAT_T * 0.5 + CBORE_T,
                            PLAT_T * 0.5, HOLE_D)
    # Упор ставится у НИЖНЕГО (переднего) края: камера сползает по уклону
    # под собственным весом и должна упираться, а не отходить от упора.
    plat += box(-PLAT_W * 0.5, PLAT_W * 0.5,
                PLAT_D * 0.5 - RIB_D, PLAT_D * 0.5,
                PLAT_T * 0.5, PLAT_T * 0.5 + RIB_H)

    plat = rotate_x(plat, -TILT_DEG)

    a = math.radians(-TILT_DEG)
    ca, sa = math.cos(a), math.sin(a)
    dy = (wall_y1 - WALL_T * 0.5) - (-PLAT_D * 0.5) * ca
    dz = WALL_H - (-PLAT_D * 0.5) * sa
    plat = translate(plat, 0.0, dy, dz)
    tris += plat

    # Нижняя грань площадки в мировых координатах: по ней строятся рёбра,
    # иначе их кромка идёт по своей прямой и к площадке не прилегает.
    def underside(local_y):
        z = -PLAT_T * 0.5
        return (local_y * ca - z * sa + dy, local_y * sa + z * ca + dz)

    p_rear = underside(-PLAT_D * 0.5)
    p_front = underside(PLAT_D * 0.5)

    def underside_z(world_y):
        t = (world_y - p_rear[0]) / (p_front[0] - p_rear[0])
        return p_rear[1] + t * (p_front[1] - p_rear[1])

    # --- Рёбра жёсткости ---------------------------------------------------
    # Треугольник от верха стойки к переднему краю основания. Гипотенуза
    # проходит ниже площадки, поэтому в неё не упирается.
    # Кромка поднята на GUSSET_BITE выше грани площадки: тела должны
    # пересекаться, иначе слайсер получит две соприкасающиеся поверхности
    # нулевой толщины вместо шва.
    gy0 = wall_y1
    gy1 = p_front[0]
    profile = [
        (gy0, BASE_T),
        (gy1, BASE_T),
        (gy1, underside_z(gy1) + GUSSET_BITE),
        (gy0, underside_z(gy0) + GUSSET_BITE),
    ]
    for cx in (-GUSSET_X, GUSSET_X):
        tris += prism_yz(profile, cx - GUSSET_T * 0.5, cx + GUSSET_T * 0.5)

    return tris


# --------------------------------------------------------------------------
# Вывод
# --------------------------------------------------------------------------

def normal(tri):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"Razer Kiyo Pro mount - workzone vision".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for tri in tris:
            f.write(struct.pack("<3f", *normal(tri)))
            for p in tri:
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def write_png(path, width, height, pixels):
    """Минимальный PNG без внешних библиотек: RGB8, фильтр 0."""
    raw = b"".join(b"\x00" + bytes(pixels[y]) for y in range(height))

    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(
            ">I", zlib.crc32(body) & 0xFFFFFFFF)

    header = struct.pack(">2I5B", width, height, 8, 2, 0, 0, 0)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", header))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def render(tris, path, width=760, height=560, yaw_deg=-35.0, pitch_deg=24.0):
    """Изометрический предпросмотр с Z-буфером - чтобы глазами проверить деталь."""
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    def project(p):
        x, y, z = p
        xe = x * cy + y * sy
        ye = -x * sy + y * cy
        # Возвращаем (экранный X, экранный верх, глубина от камеры).
        return (xe, ye * sp + z * cp, ye * cp - z * sp)

    view = [tuple(project(p) for p in tri) for tri in tris]

    xs = [p[0] for tri in view for p in tri]
    ys = [p[1] for tri in view for p in tri]
    scale = 0.82 * min(width / (max(xs) - min(xs)), height / (max(ys) - min(ys)))
    ox = width * 0.5 - 0.5 * (max(xs) + min(xs)) * scale
    oy = height * 0.5 + 0.5 * (max(ys) + min(ys)) * scale

    pixels = [bytearray((18, 20, 26) * width) for _ in range(height)]
    depth = [[1e30] * width for _ in range(height)]
    light = (-0.40, -0.55, 0.73)

    for tri, orig in zip(view, tris):
        nx, ny, nz = normal(orig)
        shade = max(0.0, nx * light[0] + ny * light[1] + nz * light[2])
        base = 40.0 + 190.0 * shade
        colour = bytes((int(base), int(base * 0.78), int(base * 0.46)))

        pts = [(ox + p[0] * scale, oy - p[1] * scale, p[2]) for p in tri]
        min_x = max(0, int(min(p[0] for p in pts)))
        max_x = min(width - 1, int(max(p[0] for p in pts)) + 1)
        min_y = max(0, int(min(p[1] for p in pts)))
        max_y = min(height - 1, int(max(p[1] for p in pts)) + 1)

        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = pts
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if abs(area) < 1e-9:
            continue

        for py in range(min_y, max_y + 1):
            row = pixels[py]
            drow = depth[py]
            for px in range(min_x, max_x + 1):
                fx, fy = px + 0.5, py + 0.5
                w0 = ((x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy)) / area
                w1 = ((x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy)) / area
                w2 = 1.0 - w0 - w1
                if w0 < 0 or w1 < 0 or w2 < 0:
                    continue
                z = w0 * z0 + w1 * z1 + w2 * z2
                if z >= drow[px]:
                    continue
                drow[px] = z
                row[px * 3:px * 3 + 3] = colour

    write_png(path, width, height, pixels)


def main():
    tris = build()
    write_stl(OUTPUT_STL, tris)

    xs = [p[0] for tri in tris for p in tri]
    ys = [p[1] for tri in tris for p in tri]
    zs = [p[2] for tri in tris for p in tri]

    print("Треугольников: %d" % len(tris))
    print("Габарит: %.1f x %.1f x %.1f мм"
          % (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)))
    print("Наклон площадки: %.0f град" % TILT_DEG)
    print("Записано: %s" % OUTPUT_STL)

    render(tris, OUTPUT_PNG)
    print("Предпросмотр: %s" % OUTPUT_PNG)

    # Вид сбоку: на нём сразу видно, прилегает ли ребро к площадке.
    side = OUTPUT_PNG.replace(".png", "_side.png")
    render(tris, side, width=700, height=520, yaw_deg=90.0, pitch_deg=0.0)
    print("Вид сбоку:    %s" % side)
    return 0


if __name__ == "__main__":
    sys.exit(main())
