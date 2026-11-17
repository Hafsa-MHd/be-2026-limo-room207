#!/usr/bin/env python3
"""Camera -> black-asphalt mask + centerline.

When the view is half black road / half white parking, prefer the black side.
"""

import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray

_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def segment_road(
    image_bgr,
    roi_top_frac,
    black_threshold,
    morph_kernel,
    include_yellow=True,
    fill_specular=True,
    white_v_min=120,
    white_s_max=60,
):
    """Return mask, samples, usable, roi_top, black_bias.

    black_bias in [-1, 1]: negative => more black asphalt on the left,
    positive => more on the right. Used when exiting parking (split view).
    """
    height, width = image_bgr.shape[:2]
    top = int(max(0, min(height - 2, height * float(roi_top_frac))))
    band = image_bgr[top:, :]

    gray = cv2.GaussianBlur(cv2.cvtColor(band, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)

    # Strict black asphalt only (never bright parking floor).
    black = (gray < int(black_threshold)).astype(np.uint8) * 255
    white = (
        (hsv[:, :, 1] < int(white_s_max)) & (hsv[:, :, 2] > int(white_v_min))
    ).astype(np.uint8) * 255

    yellow = np.zeros_like(black)
    road = black.copy()
    if include_yellow:
        # Visible yellow paint (V high enough that hue is meaningful).
        yellow = cv2.inRange(hsv, (15, 60, 20), (40, 255, 220))
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        yellow = _keep_vertical_yellow(yellow)
        # Keep yellow only if it touches black asphalt (paint on road).
        yellow = cv2.bitwise_and(
            yellow, cv2.dilate(black, np.ones((9, 9), np.uint8), iterations=1)
        )
        road = cv2.bitwise_or(road, yellow)

    # Hard reject white / light-gray parking and walls.
    road = cv2.bitwise_and(road, cv2.bitwise_not(white))

    kernel = np.ones((max(3, int(morph_kernel) | 1),) * 2, np.uint8)
    road = cv2.morphologyEx(road, cv2.MORPH_OPEN, kernel)
    road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, kernel)

    if fill_specular:
        road = _fill_specular_holes(band, road, black_threshold=int(black_threshold))
        road = cv2.bitwise_and(road, cv2.bitwise_not(white))

    # Mid-distance bias: decide at the parking-exit split, not under the bumper.
    black_bias = _black_side_bias(black, white)
    # Yellow centerline is the strongest "take this black lane" cue.
    black_bias = _blend_yellow_bias(black_bias, yellow, width=band.shape[1])
    # Drop the wrong half so parking slab / white floor cannot win the blob.
    road = _suppress_wrong_side(road, black_bias)
    lane = _prefer_black_component(
        road, gray, yellow=yellow, white=white, black_bias=black_bias
    )
    # Keep a corridor around yellow so we follow the lane, not the white border.
    lane = _corridor_around_yellow(lane, yellow)
    samples = _boundaries(lane)
    # If split parking exit: push each sample center toward black mass.
    samples = _bias_samples_to_black(lane, samples, black_bias)
    samples = _snap_samples_to_yellow(samples, yellow, lane)

    full = np.zeros((height, width), np.uint8)
    full[top:, :] = lane
    shifted = [(y + top, cx, left, right) for y, cx, left, right in samples]
    usable = len(shifted) >= 8 and int(np.count_nonzero(lane)) > 400
    # Hard gate: bumper / near field must be on black. Distant black alone
    # must never authorize driving across a white island.
    usable = usable and _near_field_on_black(lane, black, white)
    return full, shifted, usable, top, float(black_bias)


def _near_field_on_black(lane_u8, black_u8, white_u8) -> bool:
    """True only if asphalt under / just ahead of the bumper is black.

    Uses black/white segmentation (not the selected lane blob): the lane mask
    can stop mid-image while the bumper is still on asphalt. White-island
    shortcuts fail because white dominates the near band.
    """
    h, w = lane_u8.shape
    if h < 8 or w < 8:
        return False
    near = max(4, int(h * 0.22))
    band_black = black_u8[-near:, :]
    band_white = white_u8[-near:, :]
    black_px = int(np.count_nonzero(band_black))
    white_px = int(np.count_nonzero(band_white))
    # Sitting on a white island: bright floor dominates under the bumper.
    if white_px >= 400 and black_px < max(100, int(0.50 * white_px)):
        return False
    # Require real asphalt under the bumper (blocks "distant black only").
    if black_px < 150:
        return False
    return True


def _keep_vertical_yellow(yellow_u8):
    """Keep thin elongated yellow streaks (lane paint), drop speckles."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (yellow_u8 > 0).astype(np.uint8), connectivity=8
    )
    if count <= 1:
        return yellow_u8
    out = np.zeros_like(yellow_u8)
    for index in range(1, count):
        bw = int(stats[index, cv2.CC_STAT_WIDTH])
        bh = int(stats[index, cv2.CC_STAT_HEIGHT])
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < 15 or bh < 12:
            continue
        # Average stroke thickness — diagonal paint has a wide bbox but thin stroke.
        thickness = area / max(1.0, float(bh))
        if thickness <= 26.0:
            out[labels == index] = 255
    return out


def _corridor_around_yellow(lane_u8, yellow_u8, half_frac=0.22):
    """Restrict lane mask to a band around the yellow centerline."""
    ys, xs = np.where(yellow_u8 > 0)
    if xs.size < 12:
        return lane_u8
    yx = int(np.median(xs))
    half = max(40, int(lane_u8.shape[1] * half_frac))
    x0 = max(0, yx - half)
    x1 = min(lane_u8.shape[1], yx + half)
    out = np.zeros_like(lane_u8)
    out[:, x0:x1] = lane_u8[:, x0:x1]
    if int(np.count_nonzero(out)) < 250:
        return lane_u8
    return out


def _snap_samples_to_yellow(samples, yellow_u8, lane_u8):
    """Pull each row center toward yellow paint when present on that row."""
    if not samples or int(np.count_nonzero(yellow_u8)) < 12:
        return samples
    height, width = lane_u8.shape
    out = []
    for y, cx, left, right in samples:
        if y < 0 or y >= height:
            out.append((y, cx, left, right))
            continue
        xs = np.flatnonzero(yellow_u8[y] > 0)
        if xs.size >= 2:
            yx = int(np.median(xs))
            # Blend geometric mid with yellow.
            mid = (left + right) // 2
            nx = int(0.35 * mid + 0.65 * yx)
            nx = max(left, min(right, nx))
            out.append((y, nx, left, right))
        else:
            out.append((y, cx, left, right))
    return out


def _suppress_wrong_side(road_u8, black_bias):
    """When bias is clear, clear the opposite half (avoid white parking pull)."""
    if abs(float(black_bias)) < 0.25:
        return road_u8
    out = road_u8.copy()
    h, w = out.shape
    if black_bias < 0:
        # Prefer left black lane — erase far-right third.
        out[:, int(w * 0.68) :] = 0
    else:
        out[:, : int(w * 0.32)] = 0
    return out


def _blend_yellow_bias(black_bias, yellow_u8, width) -> float:
    """If yellow lane paint is visible, pull bias toward that side hard."""
    ys, xs = np.where(yellow_u8 > 0)
    if xs.size < 15:
        return float(black_bias)
    # Focus on mid-distance yellow (ignore tiny noise near bumper).
    h = yellow_u8.shape[0]
    keep = (ys > int(h * 0.05)) & (ys < int(h * 0.70))
    xs = xs[keep]
    ys = ys[keep]
    if xs.size < 12:
        return float(black_bias)
    yx = float(np.median(xs))
    yellow_bias = (yx - 0.5 * width) / max(1.0, 0.5 * width)
    yellow_bias = float(np.clip(yellow_bias, -1.0, 1.0))
    # Strong yellow streak: trust it for the parking-exit choice.
    if xs.size >= 40:
        return yellow_bias
    return float(np.clip(0.2 * float(black_bias) + 0.8 * yellow_bias, -1.0, 1.0))


def _black_side_bias(black_u8, white_u8) -> float:
    """Compare black vs white in the mid-distance band (parking-exit decision).

    Near bumper is often all parking asphalt — ignore it for the side choice.
    """
    h, w = black_u8.shape
    mid = w // 2
    # Mid band: where the half-black / half-white split usually appears.
    y0 = int(h * 0.05)
    y1 = int(h * 0.55)
    region_b = black_u8[y0:y1, :]
    region_w = white_u8[y0:y1, :]
    bl = int(np.count_nonzero(region_b[:, :mid]))
    br = int(np.count_nonzero(region_b[:, mid:]))
    wl = int(np.count_nonzero(region_w[:, :mid]))
    wr = int(np.count_nonzero(region_w[:, mid:]))

    black_total = bl + br + 1
    white_total = wl + wr
    # Split scene: bright parking / floor on one side, asphalt on the other.
    if white_total >= 600 and black_total >= 600:
        score_l = bl - 1.2 * wl
        score_r = br - 1.2 * wr
        denom = max(1.0, abs(score_l) + abs(score_r))
        return float(np.clip((score_r - score_l) / denom, -1.0, 1.0))

    # No clear white split: mild nudge toward more black.
    return float(np.clip((br - bl) / black_total, -1.0, 1.0))


def _prefer_black_component(road_u8, gray, yellow=None, white=None, black_bias=0.0):
    """Keep the asphalt blob that looks like the driven lane (yellow + black side)."""
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        (road_u8 > 0).astype(np.uint8), connectivity=8
    )
    if count <= 1:
        return road_u8

    height, width = road_u8.shape
    if yellow is None:
        yellow = np.zeros_like(road_u8)
    if white is None:
        white = np.zeros_like(road_u8)
    best_label = 0
    best_score = -1e18
    # Strong pull to the black side when exiting parking.
    target_x = 0.5 * width + 0.45 * float(black_bias) * width
    split = abs(float(black_bias)) > 0.2

    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < 80:
            continue
        ys, xs = np.where(labels == index)
        if ys.size == 0:
            continue
        # Must reach the lower part of the image (under / near the car).
        if int(ys.max()) < int(height * 0.45):
            continue
        median_luma = float(np.median(gray[ys, xs]))
        cx = float(centroids[index][0])
        cy = float(centroids[index][1])
        y_min = float(ys.min())
        y_max = float(ys.max())
        vertical_span = y_max - y_min
        # Yellow centerline = real driving lane (not empty parking slab).
        yellow_hits = int(np.count_nonzero(yellow[ys, xs]))
        # Penalize blobs that sit against a lot of white on their outer side.
        x0 = max(0, int(xs.min()) - 4)
        x1 = min(width, int(xs.max()) + 5)
        y0 = max(0, int(ys.min()))
        y1 = min(height, int(ys.max()) + 1)
        white_touch = int(np.count_nonzero(white[y0:y1, x0:x1]))

        score = (
            0.35 * area
            - 40.0 * median_luma
            + 3.5 * vertical_span
            - 1.8 * y_min  # prefer blobs that reach farther ahead
            + 120.0 * yellow_hits
            - 0.05 * white_touch
            - (1.4 if split else 0.5) * abs(cx - target_x)
        )
        if score > best_score:
            best_score = score
            best_label = index

    if best_label == 0:
        return _component_at_bottom(road_u8)
    return np.where(labels == best_label, 255, 0).astype(np.uint8)


def _bias_samples_to_black(lane_u8, samples, black_bias):
    """Shift center points toward black asphalt when parking white is on the other side."""
    if abs(black_bias) < 0.12 or not samples:
        return samples
    height, width = lane_u8.shape
    out = []
    pull = 0.35 * float(black_bias)  # fraction of half-width
    for y, cx, left, right in samples:
        # Recompute center from black pixels only on this row, weighted to preferred side.
        xs = np.flatnonzero(lane_u8[y] > 0)
        if xs.size < 10:
            out.append((y, cx, left, right))
            continue
        weights = np.ones(xs.shape, dtype=np.float32)
        if black_bias < 0:
            weights *= (1.0 + 1.5 * (1.0 - xs / max(1.0, width - 1)))
        else:
            weights *= (1.0 + 1.5 * (xs / max(1.0, width - 1)))
        weighted_cx = int(np.average(xs, weights=weights))
        # Also nudge toward preferred side of the geometric mid.
        mid = (left + right) // 2
        nudged = int(mid + pull * 0.5 * width)
        final_cx = int(0.5 * weighted_cx + 0.5 * nudged)
        final_cx = max(left, min(right, final_cx))
        out.append((y, final_cx, left, right))
    return out


def _fill_specular_holes(
    band_bgr,
    road_u8,
    *,
    black_threshold,
    max_gray=165,
    max_sat=40,
    max_hole_frac=0.30,
):
    """Fill gray asphalt specular holes; leave white borders alone."""
    if road_u8.size == 0 or int(np.count_nonzero(road_u8)) < 80:
        return road_u8

    non = (road_u8 == 0).astype(np.uint8)
    height, width = non.shape
    flood = non.copy()
    ff_mask = np.zeros((height + 2, width + 2), np.uint8)
    for x in range(width):
        if flood[0, x] == 1:
            cv2.floodFill(flood, ff_mask, (int(x), 0), 2)
    for y in range(height):
        if flood[y, 0] == 1:
            cv2.floodFill(flood, ff_mask, (0, int(y)), 2)
        if flood[y, width - 1] == 1:
            cv2.floodFill(flood, ff_mask, (int(width - 1), int(y)), 2)

    holes = (flood == 1).astype(np.uint8)
    if int(holes.sum()) < 30:
        return road_u8

    gray = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2GRAY)
    sat = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2HSV)[:, :, 1]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(holes, connectivity=8)
    road_area = max(1, int(np.count_nonzero(road_u8)))
    out = road_u8.copy()
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area < 30 or area > max_hole_frac * road_area:
            continue
        selected = labels == index
        luma = gray[selected]
        saturation = sat[selected]
        if int(np.median(luma)) < int(black_threshold):
            continue
        if int(np.median(luma)) >= max_gray:
            continue
        if float(np.mean(luma >= max_gray)) > 0.15:
            continue
        if int(np.median(saturation)) > max_sat:
            continue
        out[selected & (gray < max_gray)] = 255
    return out


def _component_at_bottom(road_u8):
    count, labels = cv2.connectedComponents(
        (road_u8 > 0).astype(np.uint8), connectivity=8
    )
    if count <= 1:
        return road_u8
    height, width = road_u8.shape
    for y in range(height - 1, int(height * 0.35), -2):
        xs = np.flatnonzero(labels[y] > 0)
        if xs.size == 0:
            continue
        nearest = int(xs[np.argmin(np.abs(xs - width / 2.0))])
        return np.where(labels == labels[y, nearest], 255, 0).astype(np.uint8)
    return np.zeros_like(road_u8)


def _boundaries(lane_u8):
    samples = []
    height = lane_u8.shape[0]
    for y in range(height - 1, int(height * 0.15), -4):
        xs = np.flatnonzero(lane_u8[y] > 0)
        if xs.size < 10:
            continue
        left = int(xs[0])
        right = int(xs[-1])
        samples.append((y, (left + right) // 2, left, right))
    return samples


def _heading(samples):
    if len(samples) < 2:
        return 0.0
    near = samples[0]
    far = samples[min(len(samples) - 1, max(1, len(samples) // 3))]
    return float(np.arctan2(far[1] - near[1], near[0] - far[0]))


class RoadVision(Node):
    def __init__(self):
        super().__init__("road_vision")
        self.declare_parameter("image_topic", "/rgb/image_raw")
        self.declare_parameter("roi_top_frac", 0.40)
        self.declare_parameter("black_threshold", 55)
        self.declare_parameter("morph_kernel", 5)
        self.declare_parameter("include_yellow", True)
        self.declare_parameter("fill_specular", True)
        self.declare_parameter("white_v_min", 120)
        self.declare_parameter("white_s_max", 60)
        self.declare_parameter("save_path", "")
        self.bridge = CvBridge()
        self._saved = False
        topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.pub_image = self.create_publisher(Image, "/vision/image", _RELIABLE)
        self.pub_mask = self.create_publisher(Image, "/vision/mask", _RELIABLE)
        self.pub_overlay = self.create_publisher(Image, "/vision/overlay", _RELIABLE)
        self.pub_panel = self.create_publisher(Image, "/vision/panel", _RELIABLE)
        self.pub_ok = self.create_publisher(Bool, "/vision/road_ok", 10)
        self.pub_center = self.create_publisher(Float32MultiArray, "/vision/center", 10)
        self.pub_bias = self.create_publisher(Float32, "/vision/black_bias", 10)
        self.get_logger().info(
            f"road_vision on {topic}. Prefers black asphalt over white parking."
        )

    def on_image(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return

        mask, samples, usable, top, black_bias = segment_road(
            image,
            float(self.get_parameter("roi_top_frac").value),
            int(self.get_parameter("black_threshold").value),
            int(self.get_parameter("morph_kernel").value),
            include_yellow=bool(self.get_parameter("include_yellow").value),
            fill_specular=bool(self.get_parameter("fill_specular").value),
            white_v_min=int(self.get_parameter("white_v_min").value),
            white_s_max=int(self.get_parameter("white_s_max").value),
        )
        heading = _heading(samples)
        overlay = self._draw(image, mask, samples, usable, top, heading, black_bias)
        panel = self._panel(image, mask, overlay, usable, black_bias)
        stamp = msg.header
        self._publish(self.pub_image, image, "bgr8", stamp)
        self._publish(self.pub_mask, mask, "mono8", stamp)
        self._publish(self.pub_overlay, overlay, "bgr8", stamp)
        self._publish(self.pub_panel, panel, "bgr8", stamp)

        ok = Bool()
        ok.data = bool(usable)
        self.pub_ok.publish(ok)

        bias_msg = Float32()
        bias_msg.data = float(black_bias)
        self.pub_bias.publish(bias_msg)

        center = Float32MultiArray()
        if usable:
            y, cx, left, right = samples[len(samples) // 5]
            center.data = [
                float(cx),
                float(y),
                float(left),
                float(right),
                float(heading),
                float(black_bias),
            ]
        else:
            center.data = []
        self.pub_center.publish(center)
        self._save_once(panel, usable)

    def _publish(self, publisher, image, encoding, header):
        out = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        out.header = header
        publisher.publish(out)

    def _draw(self, image, mask, samples, usable, top, heading, black_bias):
        overlay = image.copy()
        color = np.zeros_like(image)
        color[:, :] = (0, 180, 0)
        road = mask > 0
        overlay[road] = (
            0.55 * overlay[road].astype(np.float32)
            + 0.45 * color[road].astype(np.float32)
        ).astype(np.uint8)
        cv2.line(overlay, (0, top), (overlay.shape[1] - 1, top), (255, 0, 255), 1)
        for y, cx, left, right in samples[::2]:
            cv2.circle(overlay, (left, y), 2, (0, 0, 255), -1)
            cv2.circle(overlay, (right, y), 2, (255, 0, 0), -1)
            cv2.circle(overlay, (cx, y), 2, (0, 255, 255), -1)

        if not usable:
            text = "NO ROAD"
        elif abs(black_bias) > 0.25:
            side = "LEFT" if black_bias < 0 else "RIGHT"
            text = f"prefer BLACK {side} bias={black_bias:+.2f}"
        else:
            text = f"center heading {heading:+.2f} rad"
        cv2.putText(overlay, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3)
        cv2.putText(
            overlay, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
        )
        return overlay

    def _panel(self, image, mask, overlay, usable, black_bias):
        panes = [image, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), overlay]
        if not usable:
            label_c = "C no road"
        elif abs(black_bias) > 0.25:
            label_c = "C prefer BLACK"
        else:
            label_c = "C center on road"
        labels = ["A camera", "B black mask", label_c]
        fitted = []
        for pane, label in zip(panes, labels):
            view = cv2.resize(pane, (480, int(pane.shape[0] * 480 / pane.shape[1])))
            cv2.putText(view, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(
                view, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
            )
            fitted.append(view)
        return np.hstack(fitted)

    def _save_once(self, panel, usable):
        path = str(self.get_parameter("save_path").value).strip()
        if self._saved or not path or not usable:
            return
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        if cv2.imwrite(path, panel):
            self._saved = True
            self.get_logger().info(f"saved {path}")


def main(args=None):
    rclpy.init(args=args)
    node = RoadVision()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
