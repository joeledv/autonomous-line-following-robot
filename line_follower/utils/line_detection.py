import cv2
import numpy as np


def preprocessing(img, roi_start, width):
    h, w = img.shape[:2]
    height = int(width * h / w)   # derived from THIS frame, not a fixed test photo
    img = cv2.resize(img, (width, height))
    img = img[ int(roi_start * height):, :, : ]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img =  cv2.GaussianBlur(img, (5, 5), 0)
    return img


def binarize(img, roi_start, width):
    img = preprocessing(img, roi_start, width)
    thresh_val = max(8, 0.5 * float(np.median(img))) # returns max value between 8 and approximately 101
    _, img = cv2.threshold(img, thresh_val, 255, cv2.THRESH_BINARY_INV)
    # pixels below thresh_val become white (255); the rest become black (0)
    return img

def morphological_transformations(img, roi_start, width, kernel):
    img = binarize(img, roi_start, width)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=1) # Opening
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel, iterations=1) # Closing

    return img

def side_cx(labels, keep, side):
    # At a fork both branches are one blob, so its centroid falls on the floor between
    # them. Row by row, only the piece of tape farthest to `side` counts; a row with a
    # single piece counts whole, so without a fork this equals the blob's centroid.
    mask = np.isin(labels, keep)

    # A piece starts at a tape pixel whose left neighbour is floor, so counting the
    # starts along each row numbers its pieces 1, 2, ... from left to right.
    starts = mask.copy()
    starts[:, 1:] &= ~mask[:, :-1]
    piece_num = np.cumsum(starts, axis=1)

    if side == 'right':
        chosen = mask & (piece_num == piece_num.max(axis=1, keepdims=True))
    else:
        chosen = mask & (piece_num == 1)
    return np.nonzero(chosen)[1].mean()

def centroid(img, roi_start, width, kernel, frac_min, frac_max, width_max, side=None):
    # frac_min/frac_max: share of the ROI a blob must cover to count as tape (below is
    # noise, above is a shadow or the background). width_max: share of the ROI width.
    # side: 'right'/'left' follows that branch at a fork; None takes the plain centroid.
    img = morphological_transformations(img, roi_start, width, kernel)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStatsWithAlgorithm(img, 8, cv2.CV_32S, cv2.CCL_WU)

    if num_labels <= 1:
        return img, None

    # Evaluate every blob (not just the single largest one) so a big noise blob
    # (e.g. a shadow or something in the background) can't hide a smaller, real
    # piece of tape from ever being considered.
    candidates = []
    for j in range(1, num_labels):  # skip 0, the background label
        area = stats[j, cv2.CC_STAT_AREA]
        blob_width = stats[j, cv2.CC_STAT_WIDTH]
        frac = area / img.size
        too_wide = blob_width > width_max * img.shape[1]

        # With a side chosen a fork is wide on purpose, and side_cx keeps only one branch.
        if frac < frac_min or frac > frac_max or (too_wide and side is None):
            continue  # doesn't look like tape: too small, too big, or too wide

        bottom_edge = stats[j, cv2.CC_STAT_TOP] + stats[j, cv2.CC_STAT_HEIGHT]
        candidates.append((bottom_edge, area, j))

    if not candidates:
        return img, None

    # Past the fork the two branches are separate blobs that both reach the bottom, so
    # every blob as low as the lowest one goes in; a shadow farther up stays out.
    if side is not None:
        lowest = max(bottom for bottom, _, _ in candidates)
        keep = [j for bottom, _, j in candidates if bottom == lowest]
        return img, side_cx(labels, keep, side)

    # The camera looks forward-and-down, so real tape should reach toward the
    # bottom of the ROI (closest to the robot). Prefer whichever valid blob
    # extends lowest; break ties with area (bigger = more confident it's tape,
    # not a speck).
    candidates.sort(key=lambda c: (c[0], c[1]), reverse=True)
    i = candidates[0][2]
    cx = centroids[i][0]
    return img, cx

def error_calculation(cx, center):
    if cx is None:
        return None
    return (cx - center) / center


def draw_debug(mask, cx, error_normalized, center):
    # Takes what the caller already computed: running centroid() again here would
    # process every frame twice and double the control loop's latency.
    vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)   # mask is 0/255, needs 3 channels for color lines
    h = vis.shape[0]
    center = int(center)

    cv2.line(vis, (center, 0), (center, h), (0, 255, 0), 1)   # green: desired center

    if cx is not None:
        cv2.line(vis, (int(cx), 0), (int(cx), h), (0, 0, 255), 2)   # red: detected centroid
        cv2.putText(vis, f"error: {error_normalized:+.3f}", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    else:
        cv2.putText(vis, "NO LINE", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    return vis
