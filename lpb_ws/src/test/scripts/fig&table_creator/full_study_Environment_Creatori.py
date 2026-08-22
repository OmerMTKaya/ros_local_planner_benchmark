import cv2
import numpy as np
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

# ============================================================
# USER SETTINGS
# ============================================================

VIDEO_PATH = "mixedEnv_video.mp4"
VIDEO_PATH_ALTERNATIVES = ()

REFERENCE_IMAGE_PATH = "mixedEnv.jpg"

# Ana figürün videodan alınacağı frame.
# Bu frame, dikey engelin üst bölgede ve yatay engelin sağ bölgede
# birlikte görüldüğü temiz bir 2B görünüm sağlar.
BASE_FRAME_INDEX = 543

# Video frame'inden kare test alanını kırpmak için kullanılan pay.
# Değer, statik engeller arasındaki bir grid aralığının katı olarak uygulanır.
# 1.65 değeri 3 m × 3 m iç alanı ve çevresindeki duvarları birlikte kapsar.
ARENA_CROP_MARGIN_CELLS = 1.65

# Önceki ana görselle aynı yaklaşık çalışma boyutunu korur.
# Böylece mevcut daire, ok ve legend ölçüleri değişmeden kalır.
BASE_IMAGE_SIZE = 500

# Videodan alınan 2B tabanı, eski temiz referans görseldeki
# parlaklık ve netlik düzeyine yaklaştırır.
ENHANCE_BASE_IMAGE = False
FLOOR_BRIGHTNESS_STRENGTH = 0.85
SATURATION_GAIN = 1.08
UNSHARP_AMOUNT = 0.70
UNSHARP_SIGMA = 0.85

# Videodaki seçili/bozulmuş beyaz engeller tabandan temizlenir
# ve izler betik tarafından yeniden, temiz daireler olarak çizilir.
CLEAN_BASE_DYNAMIC_OBSTACLES = True
DYNAMIC_CLEAN_RADIUS_FACTOR = 1.65

# TurtleBot3 Burger hem odanın içinde hem de legend'de
# temiz referans JPG içindeki net robot görüntüsünden alınır.
USE_REFERENCE_ROBOT = True

# Hareket sınırları statik engellerin 1 m'lik grid geometrisinden
# hesaplanır. Bu sayede farklı videolarda beyaz engel görünümü
# bozulsa bile izlerin başlangıç ve bitişleri değişmez.
USE_GEOMETRIC_TRAIL_LIMITS = True

# Ana çıktı adı. PDF ve SVG dosyaları da bu isimden türetilir.
OUTPUT_PATH = "karma_ortam_dynamic_trail_clean.png"

# PNG, PDF ve SVG çıktıları üret.
EXPORT_PNG = True
EXPORT_PDF = True
EXPORT_SVG = True

# Çıktıyı yüksek çözünürlüklü üretmek için ölçek.
# 1: mevcut boyut
# 2: 2 kat büyük
# 4: makale/Overleaf için daha iyi
OUTPUT_SCALE = 4

# PDF/SVG içine gömülen raster görüntü için DPI değeri.
EXPORT_DPI = 300

# Video tarama aralığı.
# Küçük değer daha fazla frame tarar.
DETECTION_FRAME_STEP = 10

# Makale figürü için gölge sayıları.
# Dikey hareket eden engel daha hızlı görünüyorsa daha çok gölge veriyoruz.
VERTICAL_GHOST_COUNT = 9
HORIZONTAL_GHOST_COUNT = 6

# Beyaz silindir yarıçapı
CIRCLE_RADIUS = 18

# Başlangıç daha opak, hareket sonu daha silik
START_ALPHA = 0.92
END_ALPHA = 0.18

# Algılanan noktaları ilgili eksene kabul etme toleransı
AXIS_TOLERANCE = 55

# Eğer video takibi yetersiz kalırsa kullanılacak varsayılan hareket uzunlukları
FALLBACK_VERTICAL_LENGTH = 245
FALLBACK_HORIZONTAL_LENGTH = 245

# Beyaz silindir algılama eşikleri
WHITE_VALUE_MIN = 210
WHITE_SAT_MAX = 75

# Kahverengi statik engel algılama eşikleri
BROWN_H_MIN = 5
BROWN_H_MAX = 30
BROWN_S_MIN = 40
BROWN_V_MIN = 40


# ============================================================
# DOUBLE-HEADED ARROW SETTINGS
# ============================================================

# Okları kapatmak için bunu False yap.
DRAW_DOUBLE_HEADED_ARROWS = True

# OpenCV BGR renk formatı kullanır.
# (70, 70, 70) koyu gri verir.
ARROW_COLOR = (70, 70, 70)

# Okların saydamlığı.
# 0.0 tamamen görünmez, 1.0 tamamen opak.
ARROW_ALPHA = 0.35

# Ok çizgisi kalınlığı.
ARROW_THICKNESS = 2

# Ok başı uzunluğu.
# Daha küçük değer daha küçük ok başı üretir.
ARROW_TIP_LENGTH = 0.075


# ============================================================
# LEGEND SETTINGS
# ============================================================

# Legend'i kapatmak için bunu False yap.
DRAW_LEGEND = True

# Legend alanı ana görselin altına eklenir.
LEGEND_HEIGHT = 72
LEGEND_BACKGROUND_COLOR = (245, 245, 245)
LEGEND_BORDER_COLOR = (190, 190, 190)

# Legend icon ve yazı ayarları
LEGEND_ICON_SIZE = 28
LEGEND_ICON_TEXT_GAP = 6
LEGEND_ITEM_GAP = 10
LEGEND_FONT = cv2.FONT_HERSHEY_SIMPLEX
LEGEND_FONT_SCALE = 0.45
LEGEND_FONT_THICKNESS = 1
LEGEND_TEXT_COLOR = (35, 35, 35)

# Dynamic obstacle beyaz olduğu için açık arka planda kaybolmasın diye
# çok hafif gri bir dış çember eklenir.
DYNAMIC_ICON_OUTLINE_COLOR = (170, 170, 170)
DYNAMIC_ICON_OUTLINE_THICKNESS = 1

# Legend ikonlarının etrafındaki kırpılmış gri arka planı temizlemek için.
REMOVE_ICON_BACKGROUND = True
ICON_MASK_BLUR_SIGMA = 0.8


# ============================================================
# SCALE HELPERS
# ============================================================

def sc(value):
    """
    Görsel eleman boyutlarını OUTPUT_SCALE ile çarpar.
    """
    return max(1, int(round(value * OUTPUT_SCALE)))


def sc_float(value):
    """
    Font scale gibi kayan noktalı değerleri OUTPUT_SCALE ile çarpar.
    """
    return float(value * OUTPUT_SCALE)


def scale_point(point, source_size):
    """
    Native video crop coordinates are mapped directly to the final
    high-resolution arena. This avoids the old 690 -> 500 -> 2000
    resize chain that softened the image.
    """
    target_size = BASE_IMAGE_SIZE * OUTPUT_SCALE
    factor = target_size / float(source_size)

    return (
        float(point[0] * factor),
        float(point[1] * factor)
    )


def resize_for_render(image):
    """
    Resize the native video crop only once, directly to the final arena size.
    No intermediate downsampling and no blur/sharpen filter are applied.
    """
    target_size = BASE_IMAGE_SIZE * OUTPUT_SCALE

    if image.shape[0] == target_size and image.shape[1] == target_size:
        return image.copy()

    return cv2.resize(
        image,
        (target_size, target_size),
        interpolation=cv2.INTER_LANCZOS4
    )


# ============================================================
# BASIC HELPERS
# ============================================================

def resolve_input_path(primary_path, alternative_paths=()):
    """
    Ana dosya adı bulunamazsa aynı klasördeki alternatif adları dener.
    """
    candidates = []
    for path in [primary_path, *alternative_paths]:
        candidate = Path(path)
        candidates.append(candidate)
        if not candidate.is_absolute():
            candidates.append(SCRIPT_DIR / candidate)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    tried = ", ".join(str(p) for p in candidates)
    raise FileNotFoundError(f"Girdi dosyası bulunamadı. Denenen yollar: {tried}")


def read_video_frame(cap, frame_idx):
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_idx = min(frame_idx, total_frames - 1)

    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    success, frame = cap.read()

    if not success:
        raise RuntimeError(f"Frame okunamadı: {frame_idx}")

    return frame


def sort_grid_points(points):
    """
    2x2 düzende bulunan noktaları şu sıraya dizer:
    top-left, top-right, bottom-left, bottom-right
    """
    pts = np.array(points, dtype=np.float32)

    pts = pts[np.argsort(pts[:, 1])]
    top = pts[:2]
    bottom = pts[2:]

    top = top[np.argsort(top[:, 0])]
    bottom = bottom[np.argsort(bottom[:, 0])]

    return np.vstack([top, bottom]).astype(np.float32)


def crop_arena_from_video_frame(frame):
    """
    Videodan alınan 2B frame içindeki kare test alanını otomatik kırpar.

    Kırpma merkezi ve ölçeği, dört kahverengi statik engelin
    merkezlerinden hesaplanır. Böylece ayrı bir ana görsel dosyasına
    ihtiyaç kalmaz.
    """
    static_pts = detect_brown_static_obstacle_centers(frame)

    top_left, top_right, bottom_left, bottom_right = static_pts

    horizontal_spacing = (
        np.linalg.norm(top_right - top_left) +
        np.linalg.norm(bottom_right - bottom_left)
    ) / 2.0

    vertical_spacing = (
        np.linalg.norm(bottom_left - top_left) +
        np.linalg.norm(bottom_right - top_right)
    ) / 2.0

    grid_spacing = (horizontal_spacing + vertical_spacing) / 2.0

    center_x = float(np.mean(static_pts[:, 0]))
    center_y = float(np.mean(static_pts[:, 1]))

    half_size = ARENA_CROP_MARGIN_CELLS * grid_spacing

    h, w = frame.shape[:2]

    x1 = max(0, int(round(center_x - half_size)))
    y1 = max(0, int(round(center_y - half_size)))
    x2 = min(w, int(round(center_x + half_size)))
    y2 = min(h, int(round(center_y + half_size)))

    crop = frame[y1:y2, x1:x2].copy()

    if crop.size == 0:
        raise RuntimeError("Videodan test alanı kırpılamadı.")

    # Yuvarlama veya görüntü sınırı nedeniyle oluşabilecek küçük
    # en-boy farklarını kare olacak şekilde merkezden düzelt.
    crop_h, crop_w = crop.shape[:2]
    square_size = min(crop_h, crop_w)

    offset_x = max(0, (crop_w - square_size) // 2)
    offset_y = max(0, (crop_h - square_size) // 2)

    crop = crop[
        offset_y:offset_y + square_size,
        offset_x:offset_x + square_size
    ].copy()

    # Isaac/Gazebo görünümündeki yeşil seçim çerçevesini temizle.
    # Bu işlem yalnızca videodan gelen arka plan üzerindeki seçim
    # göstergesini kaldırır; figürün diğer içeriğini değiştirmez.
    blue, green, red = cv2.split(crop)

    green_pixels = (
        (green.astype(np.int16) > red.astype(np.int16) + 8) &
        (green.astype(np.int16) > blue.astype(np.int16) + 8) &
        (green > 60)
    )

    green_y, green_x = np.where(green_pixels)

    if len(green_x) > 0:
        corner_x = int(np.percentile(green_x, 1))
        corner_y = int(np.percentile(green_y, 1))

        selection_mask = np.zeros(crop.shape[:2], dtype=np.uint8)

        selection_mask[
            max(0, corner_y - 2):min(crop.shape[0], corner_y + 7),
            max(0, corner_x - 3):crop.shape[1]
        ] = 255

        selection_mask[
            max(0, corner_y - 3):crop.shape[0],
            max(0, corner_x - 2):min(crop.shape[1], corner_x + 7)
        ] = 255

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        yy, xx = np.indices(gray.shape)

        robot_mask = (
            (gray < 95) &
            (xx > corner_x - 20) &
            (yy > corner_y - 20)
        )

        crop_cleaned = cv2.inpaint(
            crop,
            selection_mask,
            2,
            cv2.INPAINT_TELEA
        )

        restore_robot = robot_mask & (~green_pixels)
        crop_cleaned[restore_robot] = crop[restore_robot]

        crop = crop_cleaned

    print(
        f"Videodan alınan ana figür: frame {BASE_FRAME_INDEX}, "
        f"native kırpma boyutu {crop.shape[1]}x{crop.shape[0]}"
    )

    return crop


# ============================================================
# IMAGE ENHANCEMENT AND REFERENCE-OBJECT HELPERS
# ============================================================

def neutral_floor_mask(image):
    """
    Düşük doygunluklu gri zemin/hücre bölgelerini seçer.
    Duvar, statik engel, grid çizgisi, robot ve beyaz silindirleri dışarıda bırakır.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    mask = (
        (hsv[:, :, 1] < 48) &
        (gray > 120) &
        (gray < 238)
    ).astype(np.uint8) * 255

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.GaussianBlur(mask, (0, 0), 2.2)

    return mask


def enhance_video_base(base_image, reference_image):
    """
    Video karesinden alınan 2B tabanı daha parlak ve net hale getirir.

    Yalnızca nötr gri hücrelerin parlaklığı referans görsele yaklaştırılır;
    duvarların, grid çizgilerinin ve nesnelerin rengi ezilmez.
    Ardından hafif doygunluk artışı ve unsharp-mask netleştirmesi uygulanır.
    """
    if not ENHANCE_BASE_IMAGE:
        return base_image.copy()

    output = base_image.copy()

    base_mask = neutral_floor_mask(output)
    ref_resized = cv2.resize(
        reference_image,
        (output.shape[1], output.shape[0]),
        interpolation=cv2.INTER_LANCZOS4
    )
    ref_mask = neutral_floor_mask(ref_resized)

    base_gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    ref_gray = cv2.cvtColor(ref_resized, cv2.COLOR_BGR2GRAY)

    base_values = base_gray[base_mask > 128]
    ref_values = ref_gray[ref_mask > 128]

    if base_values.size > 0 and ref_values.size > 0:
        base_level = float(np.median(base_values))
        reference_level = float(np.median(ref_values))
        brightness_delta = max(0.0, reference_level - base_level)
        brightness_delta *= FLOOR_BRIGHTNESS_STRENGTH

        brighter = np.clip(
            output.astype(np.float32) + brightness_delta,
            0,
            255
        )

        alpha = (base_mask.astype(np.float32) / 255.0)[..., None]
        output = (
            output.astype(np.float32) * (1.0 - alpha) +
            brighter * alpha
        )
        output = np.clip(output, 0, 255).astype(np.uint8)

    # Statik engellerin ve ahşap duvarların renklerini biraz daha canlı tut.
    hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * SATURATION_GAIN, 0, 255)
    output = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    # Video sıkıştırmasının oluşturduğu yumuşaklığı kontrollü biçimde azalt.
    blur = cv2.GaussianBlur(output, (0, 0), UNSHARP_SIGMA)
    output = cv2.addWeighted(
        output,
        1.0 + UNSHARP_AMOUNT,
        blur,
        -UNSHARP_AMOUNT,
        0
    )

    return np.clip(output, 0, 255).astype(np.uint8)


def estimate_motion_geometry_from_static_grid(base_image):
    """
    Dört statik engelin merkezlerinden 1 m'lik grid vektörlerini çıkarır.

    Dinamik engellerin bilinen hareket doğruları:
      - dikey:   (1.5, 2.5) <-> (1.5, 0.5)
      - yatay:   (2.5, 1.5) <-> (0.5, 1.5)

    Böylece beyaz silindirlerin video karesindeki görünümüne bağımlılık kalmaz.
    """
    pts = detect_brown_static_obstacle_centers(base_image)
    top_left, top_right, bottom_left, bottom_right = pts

    grid_x = ((top_right - top_left) + (bottom_right - bottom_left)) / 2.0
    grid_y = ((bottom_left - top_left) + (bottom_right - top_right)) / 2.0
    center = np.mean(pts, axis=0)

    vertical_start = center - grid_y
    vertical_end = center + grid_y
    horizontal_start = center + grid_x
    horizontal_end = center - grid_x

    return {
        "vertical_axis_x": float(center[0]),
        "vertical_start_y": float(vertical_start[1]),
        "vertical_end_y": float(vertical_end[1]),
        "horizontal_axis_y": float(center[1]),
        "horizontal_start_x": float(horizontal_start[0]),
        "horizontal_end_x": float(horizontal_end[0]),
        "vertical_start": tuple(vertical_start),
        "vertical_end": tuple(vertical_end),
        "horizontal_start": tuple(horizontal_start),
        "horizontal_end": tuple(horizontal_end),
    }


def clean_dynamic_obstacles_from_base(base_image, axes):
    """
    Seçilen video karesindeki iki özgün beyaz dinamik engeli temizler.

    Yalnızca beklenen dikey ve yatay hareket başlangıçlarına en yakın
    iki beyaz daire seçilir. Böylece statik engeller üzerindeki parlak
    yansımalar yanlışlıkla dinamik engel olarak silinmez.
    """
    if not CLEAN_BASE_DYNAMIC_OBSTACLES:
        return base_image.copy()

    mask = np.zeros(base_image.shape[:2], dtype=np.uint8)
    radius = max(10, int(round(CIRCLE_RADIUS * DYNAMIC_CLEAN_RADIUS_FACTOR)))

    expected_centers = [
        np.array(axes["vertical_start"], dtype=np.float32),
        np.array(axes["horizontal_start"], dtype=np.float32),
    ]

    detected_centers = [
        np.array(p, dtype=np.float32)
        for p in detect_white_circle_centers(base_image, max_count=10)
    ]

    selected_centers = []

    for expected in expected_centers:
        selected = expected

        if detected_centers:
            distances = [float(np.linalg.norm(p - expected)) for p in detected_centers]
            nearest_index = int(np.argmin(distances))
            nearest = detected_centers[nearest_index]

            # Yalnızca beklenen hareket başlangıcına yeterince yakın
            # olan daireyi videodaki gerçek dinamik engel kabul et.
            if distances[nearest_index] <= base_image.shape[0] * 0.14:
                selected = nearest
                detected_centers.pop(nearest_index)

        selected_centers.append(selected)

    # Videodaki gerçek beyaz daireleri ve geometrik başlangıç çevresini
    # temizle. Ardından tüm iz deseni doğru koordinatlarda yeniden çizilir.
    for center in selected_centers + expected_centers:
        cv2.circle(
            mask,
            (int(round(center[0])), int(round(center[1]))),
            radius,
            255,
            thickness=-1,
            lineType=cv2.LINE_AA
        )

    mask = cv2.dilate(
        mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=1
    )

    return cv2.inpaint(base_image, mask, 4, cv2.INPAINT_TELEA)


def locate_reference_robot(reference_image):
    """
    Referans görselin sağ-alt bölgesindeki TurtleBot3 Burger'i bulur.
    Dönüş değeri: (tam-boyutlu yumuşak alpha maskesi, bbox)
    """
    h, w = reference_image.shape[:2]
    x0 = int(w * 0.55)
    y0 = int(h * 0.55)
    roi = reference_image[y0:h, x0:w]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    binary = cv2.inRange(gray, 0, 112)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)

    candidates = []
    for label_idx in range(1, num_labels):
        x, y, bw, bh, area = stats[label_idx]

        if area < 25 or bw < 5 or bh < 5:
            continue

        aspect = bw / float(bh)
        if not (0.45 < aspect < 2.5):
            continue

        # Uzun grid çizgilerini değil, kompakt robot bileşenini seç.
        compactness = area / float(bw * bh)
        if compactness < 0.18:
            continue

        candidates.append((area * compactness, x, y, bw, bh))

    if not candidates:
        raise RuntimeError(
            "Referans görselde TurtleBot3 Burger bulunamadı. "
            "REFERENCE_IMAGE_PATH değerini kontrol et."
        )

    _, x, y, bw, bh = max(candidates, key=lambda item: item[0])

    pad = max(5, int(round(max(bw, bh) * 0.30)))
    gx1 = max(0, x0 + x - pad)
    gy1 = max(0, y0 + y - pad)
    gx2 = min(w, x0 + x + bw + pad)
    gy2 = min(h, y0 + y + bh + pad)

    crop = reference_image[gy1:gy2, gx1:gx2]

    # Kenar piksellerinden yerel arka plan rengini tahmin et.
    border = np.concatenate([
        crop[0, :, :],
        crop[-1, :, :],
        crop[:, 0, :],
        crop[:, -1, :],
    ], axis=0)
    background_color = np.median(border.astype(np.float32), axis=0)

    color_distance = np.linalg.norm(
        crop.astype(np.float32) - background_color[None, None, :],
        axis=2
    )

    crop_gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    alpha = np.maximum(
        np.clip((color_distance - 6.0) / 34.0, 0.0, 1.0),
        np.clip((145.0 - crop_gray.astype(np.float32)) / 65.0, 0.0, 1.0)
    )

    alpha = cv2.GaussianBlur(alpha, (0, 0), 0.65)
    alpha = np.clip(alpha, 0.0, 1.0)

    full_alpha = np.zeros((h, w), dtype=np.float32)
    full_alpha[gy1:gy2, gx1:gx2] = alpha

    return full_alpha, (gx1, gy1, gx2, gy2)


def locate_video_robot_bbox(base_image):
    """
    Locate the TurtleBot3 Burger in the lower-right part of the selected
    video frame. The search uses only compact dark pixels, so grid lines
    are excluded.
    """
    h, w = base_image.shape[:2]
    x1 = int(round(w * 0.66))
    y1 = int(round(h * 0.66))
    x2 = int(round(w * 0.94))
    y2 = int(round(h * 0.94))

    roi = base_image[y1:y2, x1:x2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    dark = cv2.inRange(gray, 0, 105)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel)

    ys, xs = np.where(dark > 0)
    if len(xs) == 0:
        raise RuntimeError("Videodaki TurtleBot3 Burger bulunamadı.")

    bx1 = x1 + int(xs.min())
    by1 = y1 + int(ys.min())
    bx2 = x1 + int(xs.max()) + 1
    by2 = y1 + int(ys.max()) + 1

    return bx1, by1, bx2, by2


def replace_robot_from_reference(base_image, reference_image):
    """
    Replace only the robot in the attached video's frame with the clean
    TurtleBot3 Burger from the reference JPG. Placement is based on the
    robot's actual location in the selected video frame, not on a
    homography. This prevents duplicate or shifted robot images.
    """
    if not USE_REFERENCE_ROBOT:
        return base_image.copy()

    result = base_image.copy()
    h, w = result.shape[:2]

    bx1, by1, bx2, by2 = locate_video_robot_bbox(result)
    robot_cx = (bx1 + bx2) / 2.0
    robot_cy = (by1 + by2) / 2.0

    # Remove the video's robot and the green selection guides.
    cleanup_mask = np.zeros((h, w), dtype=np.uint8)
    pad = max(8, int(round(max(bx2 - bx1, by2 - by1) * 0.45)))
    cv2.rectangle(
        cleanup_mask,
        (max(0, bx1 - pad), max(0, by1 - pad)),
        (min(w - 1, bx2 + pad), min(h - 1, by2 + pad)),
        255,
        thickness=-1
    )

    blue, green, red = cv2.split(result)
    green_guides = (
        (green.astype(np.int16) > red.astype(np.int16) + 12) &
        (green.astype(np.int16) > blue.astype(np.int16) + 12) &
        (green > 65) &
        (np.indices((h, w))[0] > int(h * 0.60)) &
        (np.indices((h, w))[1] > int(w * 0.55))
    )
    cleanup_mask[green_guides] = 255

    cleaned = cv2.inpaint(result, cleanup_mask, 3, cv2.INPAINT_TELEA)

    full_alpha, (rx1, ry1, rx2, ry2) = locate_reference_robot(reference_image)
    robot_crop = reference_image[ry1:ry2, rx1:rx2]
    robot_alpha = full_alpha[ry1:ry2, rx1:rx2]

    target_w = max(bx2 - bx1, int(round(w * 0.055)))
    aspect = robot_crop.shape[0] / float(robot_crop.shape[1])
    target_h = max(1, int(round(target_w * aspect)))

    robot_crop = cv2.resize(
        robot_crop,
        (target_w, target_h),
        interpolation=cv2.INTER_LANCZOS4
    )
    robot_alpha = cv2.resize(
        robot_alpha,
        (target_w, target_h),
        interpolation=cv2.INTER_LINEAR
    )
    robot_alpha = np.clip(robot_alpha, 0.0, 1.0)

    px1 = int(round(robot_cx - target_w / 2.0))
    py1 = int(round(robot_cy - target_h / 2.0))
    px2 = px1 + target_w
    py2 = py1 + target_h

    # Clip safely to image bounds.
    sx1 = max(0, -px1)
    sy1 = max(0, -py1)
    sx2 = target_w - max(0, px2 - w)
    sy2 = target_h - max(0, py2 - h)
    dx1 = max(0, px1)
    dy1 = max(0, py1)
    dx2 = dx1 + (sx2 - sx1)
    dy2 = dy1 + (sy2 - sy1)

    alpha3 = robot_alpha[sy1:sy2, sx1:sx2, None]
    cleaned[dy1:dy2, dx1:dx2] = np.clip(
        cleaned[dy1:dy2, dx1:dx2].astype(np.float32) * (1.0 - alpha3) +
        robot_crop[sy1:sy2, sx1:sx2].astype(np.float32) * alpha3,
        0,
        255
    ).astype(np.uint8)

    return cleaned


def make_reference_robot_icon(reference_image, size):
    """
    Legend için referans JPG'den temiz TurtleBot3 Burger ikonu üretir.
    """
    full_alpha, (x1, y1, x2, y2) = locate_reference_robot(reference_image)
    crop = reference_image[y1:y2, x1:x2]
    alpha = full_alpha[y1:y2, x1:x2]

    crop = cv2.resize(crop, (size, size), interpolation=cv2.INTER_LANCZOS4)
    alpha = cv2.resize(alpha, (size, size), interpolation=cv2.INTER_LINEAR)
    alpha = np.clip(alpha, 0.0, 1.0)[..., None]

    background = np.full(
        (size, size, 3),
        LEGEND_BACKGROUND_COLOR,
        dtype=np.float32
    )

    icon = (
        crop.astype(np.float32) * alpha +
        background * (1.0 - alpha)
    )

    icon = np.clip(icon, 0, 255).astype(np.uint8)

    # Küçük JPG robot kırpmasının büyütülmesinden doğan yumuşamayı azalt.
    icon_blur = cv2.GaussianBlur(icon, (0, 0), 0.75)
    icon = cv2.addWeighted(icon, 1.55, icon_blur, -0.55, 0)

    return np.clip(icon, 0, 255).astype(np.uint8)


# ============================================================
# DETECTION FUNCTIONS
# ============================================================

def detect_white_circle_centers(image, max_count=2):
    """
    Görüntüdeki beyaz dairesel dinamik engel merkezlerini bulur.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, WHITE_VALUE_MIN], dtype=np.uint8)
    upper_white = np.array([179, WHITE_SAT_MAX, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    h, w = image.shape[:2]
    image_area = h * w

    candidates = []

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area <= 0:
            continue

        perimeter = cv2.arcLength(cnt, True)

        if perimeter == 0:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)

        if bh == 0:
            continue

        aspect_ratio = bw / float(bh)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)

        # Beyaz silindirler küçük ve yaklaşık dairesel olmalı.
        if not (image_area * 0.00010 < area < image_area * 0.010):
            continue

        if not (0.55 < aspect_ratio < 1.75):
            continue

        if circularity < 0.35:
            continue

        moments = cv2.moments(cnt)

        if moments["m00"] == 0:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        candidates.append((area, cx, cy))

    candidates = sorted(candidates, key=lambda item: item[0], reverse=True)

    centers = [(cx, cy) for _, cx, cy in candidates[:max_count]]

    return centers


def detect_brown_static_obstacle_centers(image):
    """
    Dört kahverengi statik engelin merkezlerini bulur.
    Bunlar video -> ana figür koordinat dönüşümü için kullanılır.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_brown = np.array(
        [BROWN_H_MIN, BROWN_S_MIN, BROWN_V_MIN],
        dtype=np.uint8
    )
    upper_brown = np.array(
        [BROWN_H_MAX, 255, 255],
        dtype=np.uint8
    )

    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE
    )

    h, w = image.shape[:2]
    image_area = h * w

    candidates = []

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area <= 0:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)

        if bh == 0:
            continue

        aspect_ratio = bw / float(bh)

        if not (0.70 < aspect_ratio < 1.40):
            continue

        if not (100 < area < image_area * 0.02):
            continue

        moments = cv2.moments(cnt)

        if moments["m00"] == 0:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        candidates.append((area, cx, cy))

    if len(candidates) < 4:
        raise RuntimeError(
            "Dört statik engel merkezi bulunamadı. "
            "Kahverengi eşik değerlerini kontrol et."
        )

    candidates = sorted(candidates, key=lambda item: item[0])[:4]
    points = [(cx, cy) for _, cx, cy in candidates]

    return sort_grid_points(points)


def compute_video_to_base_homography(video_frame, base_image):
    """
    Video koordinatlarını ana figür koordinatlarına dönüştürür.
    Referans olarak kahverengi statik engeller kullanılır.
    """
    video_static_pts = detect_brown_static_obstacle_centers(video_frame)
    base_static_pts = detect_brown_static_obstacle_centers(base_image)

    homography, _ = cv2.findHomography(video_static_pts, base_static_pts)

    if homography is None:
        raise RuntimeError("Homografi hesaplanamadı.")

    return homography


def transform_points(points, homography):
    if len(points) == 0:
        return []

    pts = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(pts, homography)

    return [tuple(p) for p in transformed.reshape(-1, 2)]


# ============================================================
# AXIS ESTIMATION FROM BASE IMAGE
# ============================================================

def estimate_axes_from_base_image(base_image):
    """
    Hareket eksenlerini temiz ana figürdeki başlangıç beyaz silindirlerinden belirler.

    Üstteki beyaz silindir dikey hareket eden engeldir.
    Sağdaki beyaz silindir yatay hareket eden engeldir.
    """
    centers = detect_white_circle_centers(base_image, max_count=2)

    if len(centers) < 2:
        raise RuntimeError(
            "Ana figürde iki beyaz dinamik engel bulunamadı. "
            "Beyaz eşik değerlerini kontrol et."
        )

    vertical_start = min(centers, key=lambda p: p[1])
    horizontal_start = max(centers, key=lambda p: p[0])

    vertical_axis_x = vertical_start[0]
    vertical_start_y = vertical_start[1]

    horizontal_axis_y = horizontal_start[1]
    horizontal_start_x = horizontal_start[0]

    print(f"Dikey başlangıç merkezi: ({vertical_axis_x:.2f}, {vertical_start_y:.2f})")
    print(f"Yatay başlangıç merkezi: ({horizontal_start_x:.2f}, {horizontal_axis_y:.2f})")

    return {
        "vertical_axis_x": vertical_axis_x,
        "vertical_start_y": vertical_start_y,
        "horizontal_axis_y": horizontal_axis_y,
        "horizontal_start_x": horizontal_start_x,
    }


# ============================================================
# TRACK COLLECTION
# ============================================================

def assign_centers_to_tracks(base_centers, axes):
    """
    Algılanan beyaz merkezleri dikey veya yatay harekete atar.
    Daha sonra ilgili eksene kilitler.
    """
    vertical_axis_x = axes["vertical_axis_x"]
    horizontal_axis_y = axes["horizontal_axis_y"]

    vertical_candidates = []
    horizontal_candidates = []

    for x, y in base_centers:
        vertical_distance = abs(x - vertical_axis_x)
        horizontal_distance = abs(y - horizontal_axis_y)

        if vertical_distance < AXIS_TOLERANCE:
            vertical_candidates.append((vertical_distance, (vertical_axis_x, y)))

        if horizontal_distance < AXIS_TOLERANCE:
            horizontal_candidates.append((horizontal_distance, (x, horizontal_axis_y)))

    vertical_point = None
    horizontal_point = None

    if vertical_candidates:
        vertical_candidates = sorted(vertical_candidates, key=lambda item: item[0])
        vertical_point = vertical_candidates[0][1]

    if horizontal_candidates:
        horizontal_candidates = sorted(horizontal_candidates, key=lambda item: item[0])
        horizontal_point = horizontal_candidates[0][1]

    return vertical_point, horizontal_point


def collect_tracks_from_video(cap, homography, axes):
    """
    Video boyunca beyaz engel merkezlerini toplar,
    ana figür koordinatına taşır ve eksene kilitler.
    """
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    vertical_track = []
    horizontal_track = []

    for frame_idx in range(0, total_frames, DETECTION_FRAME_STEP):
        frame = read_video_frame(cap, frame_idx)

        video_centers = detect_white_circle_centers(frame, max_count=2)
        base_centers = transform_points(video_centers, homography)

        vertical_point, horizontal_point = assign_centers_to_tracks(
            base_centers,
            axes
        )

        if vertical_point is not None:
            vertical_track.append((frame_idx, vertical_point))

        if horizontal_point is not None:
            horizontal_track.append((frame_idx, horizontal_point))

    return vertical_track, horizontal_track


def summarize_track_endpoints(vertical_track, horizontal_track, axes):
    """
    İzlerin başlangıç ve bitiş noktalarını belirler.
    Tespit yetersizse güvenli varsayılan uzunluklar kullanır.
    """
    vertical_axis_x = axes["vertical_axis_x"]
    vertical_start_y = axes["vertical_start_y"]

    horizontal_axis_y = axes["horizontal_axis_y"]
    horizontal_start_x = axes["horizontal_start_x"]

    vertical_ys = [p[1][1] for p in vertical_track]
    horizontal_xs = [p[1][0] for p in horizontal_track]

    # Bilinen 3 m x 3 m grid geometrisi, video tespitlerinden daha kararlıdır.
    if USE_GEOMETRIC_TRAIL_LIMITS and "vertical_end_y" in axes:
        vertical_end_y = axes["vertical_end_y"]
    elif len(vertical_ys) >= 2:
        vertical_end_y = max(vertical_ys)
    else:
        vertical_end_y = vertical_start_y + FALLBACK_VERTICAL_LENGTH

    if USE_GEOMETRIC_TRAIL_LIMITS and "horizontal_end_x" in axes:
        horizontal_end_x = axes["horizontal_end_x"]
    elif len(horizontal_xs) >= 2:
        horizontal_end_x = min(horizontal_xs)
    else:
        horizontal_end_x = horizontal_start_x - FALLBACK_HORIZONTAL_LENGTH

    vertical_start = (vertical_axis_x, vertical_start_y)
    vertical_end = (vertical_axis_x, vertical_end_y)

    horizontal_start = (horizontal_start_x, horizontal_axis_y)
    horizontal_end = (horizontal_end_x, horizontal_axis_y)

    print(f"Dikey iz başlangıç: {vertical_start}")
    print(f"Dikey iz bitiş:     {vertical_end}")
    print(f"Yatay iz başlangıç: {horizontal_start}")
    print(f"Yatay iz bitiş:     {horizontal_end}")

    return vertical_start, vertical_end, horizontal_start, horizontal_end


# ============================================================
# DRAWING
# ============================================================

def interpolate_points(start, end, count):
    """
    Başlangıç ve bitiş arasında eşit aralıklı noktalar üretir.
    """
    xs = np.linspace(start[0], end[0], count)
    ys = np.linspace(start[1], end[1], count)

    return list(zip(xs, ys))


def draw_transparent_circle(image, center, radius, alpha, color=(255, 255, 255)):
    overlay = image.copy()

    center_int = (
        int(round(center[0])),
        int(round(center[1]))
    )

    cv2.circle(
        overlay,
        center=center_int,
        radius=sc(radius),
        color=color,
        thickness=-1,
        lineType=cv2.LINE_AA
    )

    cv2.addWeighted(
        overlay,
        alpha,
        image,
        1.0 - alpha,
        0,
        dst=image
    )


def draw_double_headed_arrow(
    image,
    start,
    end,
    color=ARROW_COLOR,
    alpha=ARROW_ALPHA,
    thickness=ARROW_THICKNESS,
    tip_length=ARROW_TIP_LENGTH
):
    """
    Başlangıç ve bitiş merkezleri arasında çift taraflı ok çizer.

    Önemli:
    Ok uçları beyaz silindirlerin dış sınırında değil,
    doğrudan başlangıç ve bitiş merkez noktalarında biter.
    """
    overlay = image.copy()

    start_int = (
        int(round(start[0])),
        int(round(start[1]))
    )

    end_int = (
        int(round(end[0])),
        int(round(end[1]))
    )

    cv2.arrowedLine(
        overlay,
        start_int,
        end_int,
        color,
        sc(thickness),
        line_type=cv2.LINE_AA,
        shift=0,
        tipLength=tip_length
    )

    cv2.arrowedLine(
        overlay,
        end_int,
        start_int,
        color,
        sc(thickness),
        line_type=cv2.LINE_AA,
        shift=0,
        tipLength=tip_length
    )

    cv2.addWeighted(
        overlay,
        alpha,
        image,
        1.0 - alpha,
        0,
        dst=image
    )


def draw_motion_arrows(
    image,
    vertical_start,
    vertical_end,
    horizontal_start,
    horizontal_end
):
    """
    Dikey ve yatay dinamik engel hareketleri için çift taraflı okları çizer.
    """
    draw_double_headed_arrow(
        image=image,
        start=vertical_start,
        end=vertical_end
    )

    draw_double_headed_arrow(
        image=image,
        start=horizontal_start,
        end=horizontal_end
    )


# ============================================================
# LEGEND HELPERS
# ============================================================

def crop_square_around_center(image, center, crop_size):
    """
    Verilen merkez etrafında kare crop alır.
    Crop görüntü sınırlarını aşarsa güvenli biçimde kırpar.
    """
    h, w = image.shape[:2]
    cx, cy = center
    half = crop_size // 2

    x1 = max(0, int(round(cx)) - half)
    y1 = max(0, int(round(cy)) - half)
    x2 = min(w, int(round(cx)) + half)
    y2 = min(h, int(round(cy)) + half)

    crop = image[y1:y2, x1:x2].copy()

    if crop.size == 0:
        return None

    return crop


def resize_icon(icon, size=None):
    """
    Legend içinde kullanılacak iconu kare boyuta getirir.
    """
    if size is None:
        size = sc(LEGEND_ICON_SIZE)

    if icon is None or icon.size == 0:
        blank = np.full((size, size, 3), LEGEND_BACKGROUND_COLOR, dtype=np.uint8)
        return blank

    return cv2.resize(icon, (size, size), interpolation=cv2.INTER_AREA)


def compose_icon_on_legend_background(icon, mask):
    """
    Icon içindeki istenmeyen gri kırpma zeminini temizler.
    Maskede beyaz olan nesne pikselleri korunur,
    diğer bölgeler legend arka plan rengiyle değiştirilir.
    """
    if not REMOVE_ICON_BACKGROUND:
        return icon

    background = np.full(
        icon.shape,
        LEGEND_BACKGROUND_COLOR,
        dtype=np.uint8
    )

    if mask is None or mask.size == 0:
        return background

    mask = mask.astype(np.uint8)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (sc(3), sc(3))
    )

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)

    mask_float = mask.astype(np.float32) / 255.0

    if ICON_MASK_BLUR_SIGMA > 0:
        mask_float = cv2.GaussianBlur(
            mask_float,
            ksize=(0, 0),
            sigmaX=ICON_MASK_BLUR_SIGMA * OUTPUT_SCALE,
            sigmaY=ICON_MASK_BLUR_SIGMA * OUTPUT_SCALE
        )

    mask_float = np.clip(mask_float, 0.0, 1.0)
    mask_float = mask_float[..., None]

    composed = (
        icon.astype(np.float32) * mask_float +
        background.astype(np.float32) * (1.0 - mask_float)
    )

    return np.clip(composed, 0, 255).astype(np.uint8)


def make_robot_icon_mask(icon):
    """
    TurtleBot3 Burger koyu renkli olduğu için düşük parlaklık maskesi kullanılır.
    """
    gray = cv2.cvtColor(icon, cv2.COLOR_BGR2GRAY)

    # Koyu robot piksellerini seç.
    mask = cv2.inRange(gray, 0, 105)

    return mask


def make_robot_icon(base_image):
    """
    Legend için TurtleBot3 Burger ikonunu temiz referans JPG'den üretir.
    base_image parametresi geriye dönük uyumluluk için korunur.
    """
    icon_size = sc(LEGEND_ICON_SIZE)

    reference_path = resolve_input_path(
        REFERENCE_IMAGE_PATH
    )
    reference_image = cv2.imread(str(reference_path))
    if reference_image is None:
        raise RuntimeError(f"Referans görsel okunamadı: {reference_path}")

    return make_reference_robot_icon(reference_image, icon_size)

def make_static_obstacle_icon(base_image):
    """
    Static obstacle iconunu legend arka planı üzerinde temiz şekilde çizer.
    Böylece ana figürden kırpılan gri arka plan legend'e taşınmaz.
    """
    icon_size = sc(LEGEND_ICON_SIZE)

    icon = np.full(
        (icon_size, icon_size, 3),
        LEGEND_BACKGROUND_COLOR,
        dtype=np.uint8
    )

    # Kahverengi statik engel için küçük, gölgeli kare.
    margin = sc(5)

    # Ana dolgu rengi, OpenCV BGR formatındadır.
    fill_color = (55, 135, 205)
    border_color = (35, 95, 150)
    shadow_color = (170, 170, 170)

    # Hafif gölge
    cv2.rectangle(
        icon,
        (margin + sc(2), margin + sc(2)),
        (icon_size - margin + sc(1), icon_size - margin + sc(1)),
        shadow_color,
        thickness=-1,
        lineType=cv2.LINE_AA
    )

    # Ana statik engel karesi
    cv2.rectangle(
        icon,
        (margin, margin),
        (icon_size - margin, icon_size - margin),
        fill_color,
        thickness=-1,
        lineType=cv2.LINE_AA
    )

    # Kenarlık
    cv2.rectangle(
        icon,
        (margin, margin),
        (icon_size - margin, icon_size - margin),
        border_color,
        thickness=sc(1),
        lineType=cv2.LINE_AA
    )

    # Hafif parlaklık etkisi
    cv2.line(
        icon,
        (margin + sc(2), margin + sc(2)),
        (icon_size - margin - sc(2), margin + sc(2)),
        (90, 170, 235),
        thickness=sc(1),
        lineType=cv2.LINE_AA
    )

    cv2.line(
        icon,
        (margin + sc(2), margin + sc(2)),
        (margin + sc(2), icon_size - margin - sc(2)),
        (90, 170, 235),
        thickness=sc(1),
        lineType=cv2.LINE_AA
    )

    return icon


def make_dynamic_obstacle_icon():
    """
    Dynamic obstacle iconunu beyaz silindir/dairesel engel olarak çizer.
    """
    icon_size = sc(LEGEND_ICON_SIZE)

    icon = np.full(
        (icon_size, icon_size, 3),
        LEGEND_BACKGROUND_COLOR,
        dtype=np.uint8
    )

    center = (icon_size // 2, icon_size // 2)
    radius = int(icon_size * 0.33)

    cv2.circle(
        icon,
        center,
        radius,
        (255, 255, 255),
        thickness=-1,
        lineType=cv2.LINE_AA
    )

    cv2.circle(
        icon,
        center,
        radius,
        DYNAMIC_ICON_OUTLINE_COLOR,
        thickness=sc(DYNAMIC_ICON_OUTLINE_THICKNESS),
        lineType=cv2.LINE_AA
    )

    return icon


def paste_icon(canvas, icon, x, y):
    """
    Iconu verilen sol-üst koordinata yapıştırır.
    """
    icon_h, icon_w = icon.shape[:2]
    canvas[y:y + icon_h, x:x + icon_w] = icon


def add_legend_below(image, base_image):
    """
    Ana görselin altına legend paneli ekler.

    Legend sırası:
    1. TurtleBot3 Burger
    2. Static obstacle
    3. Dynamic obstacle
    """
    if not DRAW_LEGEND:
        return image

    h, w = image.shape[:2]

    legend_height = sc(LEGEND_HEIGHT)
    icon_size = sc(LEGEND_ICON_SIZE)
    icon_text_gap = sc(LEGEND_ICON_TEXT_GAP)
    item_gap = sc(LEGEND_ITEM_GAP)
    font_scale = sc_float(LEGEND_FONT_SCALE)
    font_thickness = sc(LEGEND_FONT_THICKNESS)

    canvas = np.full(
        (h + legend_height, w, 3),
        LEGEND_BACKGROUND_COLOR,
        dtype=np.uint8
    )

    canvas[:h, :w] = image

    # Legend'i ana görselden ayıran ince çizgi
    cv2.line(
        canvas,
        (0, h),
        (w, h),
        LEGEND_BORDER_COLOR,
        thickness=sc(1),
        lineType=cv2.LINE_AA
    )

    items = [
        (make_robot_icon(base_image), "TurtleBot3 Burger"),
        (make_static_obstacle_icon(base_image), "Static obstacle"),
        (make_dynamic_obstacle_icon(), "Dynamic obstacle"),
    ]

    # Legend genişliği görselden taşarsa font ve boşluğu otomatik küçült.
    available_width = w - sc(16)

    while True:
        item_widths = []

        for _, label in items:
            (text_w, _), _ = cv2.getTextSize(
                label,
                LEGEND_FONT,
                font_scale,
                font_thickness
            )

            item_widths.append(
                icon_size + icon_text_gap + text_w
            )

        total_width = sum(item_widths) + item_gap * (len(items) - 1)

        if total_width <= available_width:
            break

        if font_scale <= sc_float(0.32):
            break

        font_scale *= 0.94
        item_gap = max(sc(5), int(item_gap * 0.94))
        icon_text_gap = max(sc(4), int(icon_text_gap * 0.96))

    start_x = max(sc(8), int((w - total_width) / 2))
    icon_y = h + int((legend_height - icon_size) / 2)

    x = start_x

    for (icon, label), item_width in zip(items, item_widths):
        paste_icon(canvas, icon, x, icon_y)

        (text_w, text_h), baseline = cv2.getTextSize(
            label,
            LEGEND_FONT,
            font_scale,
            font_thickness
        )

        text_x = x + icon_size + icon_text_gap
        text_y = h + int((legend_height + text_h) / 2) - sc(1)

        cv2.putText(
            canvas,
            label,
            (text_x, text_y),
            LEGEND_FONT,
            font_scale,
            LEGEND_TEXT_COLOR,
            font_thickness,
            lineType=cv2.LINE_AA
        )

        x += item_width + item_gap

    return canvas


def draw_clean_trails(
    base_image,
    vertical_start,
    vertical_end,
    horizontal_start,
    horizontal_end
):
    output = base_image.copy()

    vertical_points = interpolate_points(
        vertical_start,
        vertical_end,
        VERTICAL_GHOST_COUNT
    )

    horizontal_points = interpolate_points(
        horizontal_start,
        horizontal_end,
        HORIZONTAL_GHOST_COUNT
    )

    vertical_alphas = np.linspace(
        START_ALPHA,
        END_ALPHA,
        VERTICAL_GHOST_COUNT
    )

    horizontal_alphas = np.linspace(
        START_ALPHA,
        END_ALPHA,
        HORIZONTAL_GHOST_COUNT
    )

    drawing_items = []

    for point, alpha in zip(vertical_points, vertical_alphas):
        drawing_items.append((point, float(alpha)))

    for point, alpha in zip(horizontal_points, horizontal_alphas):
        drawing_items.append((point, float(alpha)))

    # Önce silik olanları, sonra koyu olanları çiz.
    # Böylece başlangıç konumları daha belirgin kalır.
    drawing_items = sorted(drawing_items, key=lambda item: item[1])

    for point, alpha in drawing_items:
        draw_transparent_circle(
            output,
            center=point,
            radius=CIRCLE_RADIUS,
            alpha=alpha
        )

    # ========================================================
    # OPTIONAL: DOUBLE-HEADED MOTION ARROWS
    # ========================================================
    # Controlled by DRAW_DOUBLE_HEADED_ARROWS.
    if DRAW_DOUBLE_HEADED_ARROWS:
        draw_motion_arrows(
            image=output,
            vertical_start=vertical_start,
            vertical_end=vertical_end,
            horizontal_start=horizontal_start,
            horizontal_end=horizontal_end
        )

    # ========================================================
    # OPTIONAL: LEGEND BELOW FIGURE
    # ========================================================
    # Controlled by DRAW_LEGEND.
    output = add_legend_below(
        image=output,
        base_image=base_image
    )

    return output


# ============================================================
# EXPORT HELPERS
# ============================================================

def save_outputs(image_bgr, output_path):
    """
    Aynı görseli PNG, PDF ve SVG olarak kaydeder.

    Not:
    PDF ve SVG dosyaları burada tam vektörel değildir.
    Çünkü ana görsel/video raster kaynaktır.
    Bu dosyaların içine yüksek çözünürlüklü raster görüntü gömülür.
    """
    output_path = Path(output_path)
    if not output_path.is_absolute():
        output_path = SCRIPT_DIR / output_path
    output_dir = output_path.parent
    output_stem = output_path.stem

    if str(output_dir) == ".":
        output_dir = Path(".")

    output_dir.mkdir(parents=True, exist_ok=True)

    png_path = output_dir / f"{output_stem}.png"
    pdf_path = output_dir / f"{output_stem}.pdf"
    svg_path = output_dir / f"{output_stem}.svg"

    if EXPORT_PNG:
        cv2.imwrite(str(png_path), image_bgr)
        print(f"PNG kaydedildi: {png_path}")

    if EXPORT_PDF or EXPORT_SVG:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise ImportError(
                "PDF/SVG çıktısı için matplotlib gerekiyor. "
                "Kurmak için: pip install matplotlib"
            ) from exc

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        height_px, width_px = image_rgb.shape[:2]

        figure_width_in = width_px / EXPORT_DPI
        figure_height_in = height_px / EXPORT_DPI

        def save_with_matplotlib(save_path, file_format):
            fig = plt.figure(
                figsize=(figure_width_in, figure_height_in),
                dpi=EXPORT_DPI,
                frameon=False
            )

            ax = fig.add_axes([0, 0, 1, 1])
            ax.imshow(image_rgb)
            ax.axis("off")

            fig.savefig(
                str(save_path),
                format=file_format,
                dpi=EXPORT_DPI,
                pad_inches=0,
                facecolor="white",
                edgecolor="none"
            )

            plt.close(fig)

        if EXPORT_PDF:
            save_with_matplotlib(pdf_path, "pdf")
            print(f"PDF kaydedildi: {pdf_path}")

        if EXPORT_SVG:
            save_with_matplotlib(svg_path, "svg")
            print(f"SVG kaydedildi: {svg_path}")


# ============================================================
# MAIN
# ============================================================

def create_dynamic_trail_figure():
    video_path = resolve_input_path(
        VIDEO_PATH,
        VIDEO_PATH_ALTERNATIVES
    )
    reference_path = resolve_input_path(
        REFERENCE_IMAGE_PATH
    )

    reference_image = cv2.imread(str(reference_path))

    if reference_image is None:
        raise RuntimeError(f"Referans görsel okunamadı: {reference_path}")

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Video açılamadı: {video_path}")

    # Ana figür doğrudan videodaki 2B görünümden alınır.
    base_frame = read_video_frame(cap, BASE_FRAME_INDEX)
    base_image_original = crop_arena_from_video_frame(base_frame)

    # Hücre parlaklığı ve genel netlik eski temiz görsel seviyesine yaklaştırılır.
    base_image_original = enhance_video_base(
        base_image_original,
        reference_image
    )

    # Hareket eksenleri ve uç noktaları doğrudan 3 m x 3 m ortamın
    # sabit grid geometrisinden hesaplanır. Statik engeller (1,1),
    # (2,1), (1,2) ve (2,2) noktalarında olduğundan merkez (1.5,1.5),
    # dinamik engel doğruları ise eski doğru figürdeki gibi
    # (1.5,0.5)<->(1.5,2.5) ve (0.5,1.5)<->(2.5,1.5) olur.
    axes = estimate_motion_geometry_from_static_grid(base_image_original)

    # Video karesindeki özgün/bazen bozuk beyaz silindirleri temizle.
    base_image_original = clean_dynamic_obstacles_from_base(
        base_image_original,
        axes
    )

    # Odanın içindeki TurtleBot3 Burger'i temiz referans JPG'den yerleştir.
    base_image_original = replace_robot_from_reference(
        base_image_original,
        reference_image
    )

    first_frame = read_video_frame(cap, 0)

    homography = compute_video_to_base_homography(
        video_frame=first_frame,
        base_image=base_image_original
    )

    vertical_track, horizontal_track = collect_tracks_from_video(
        cap,
        homography,
        axes
    )

    cap.release()

    print(f"Videodan toplanan dikey nokta sayısı: {len(vertical_track)}")
    print(f"Videodan toplanan yatay nokta sayısı: {len(horizontal_track)}")

    vertical_start, vertical_end, horizontal_start, horizontal_end = summarize_track_endpoints(
        vertical_track,
        horizontal_track,
        axes
    )

    # Render işlemi yüksek çözünürlüklü görsel üzerinde yapılır.
    base_image_render = resize_for_render(base_image_original)

    source_size = base_image_original.shape[1]
    vertical_start_render = scale_point(vertical_start, source_size)
    vertical_end_render = scale_point(vertical_end, source_size)
    horizontal_start_render = scale_point(horizontal_start, source_size)
    horizontal_end_render = scale_point(horizontal_end, source_size)

    output = draw_clean_trails(
        base_image_render,
        vertical_start_render,
        vertical_end_render,
        horizontal_start_render,
        horizontal_end_render
    )

    save_outputs(output, OUTPUT_PATH)

    print("\nTüm çıktılar üretildi.")


if __name__ == "__main__":
    create_dynamic_trail_figure()
