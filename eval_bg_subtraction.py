#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
포인트 클라우드 배경차감(Background Subtraction) 전처리 성능 평가 스크립트
=======================================================================

평가 지표
---------
1) False Foreground Rate (FFR)
   - 사람이 없는 빈 장면(raw)을 전처리한 뒤에도 "전경"으로 남아있는 포인트 비율.
   - FFR = (전처리 후 남은 포인트 수) / (원본 전체 포인트 수)
   - 라벨이 있는 장면에서는 "배경(GT)인데 전경으로 예측된" 포인트 비율(=배경 FPR)로 확장.

2) Background Removal Rate (BRR)
   - 정적 배경 포인트가 전처리 후 얼마나 제거되었는가.
   - 빈 장면 기준: BRR = (제거된 포인트 수) / (원본 전체 포인트 수) = 1 - FFR
   - 라벨이 있는 장면에서는 "배경(GT) 포인트 중 실제로 제거된 비율"(=배경 Recall)로 확장.

3) Foreground Ratio / SNR 개념
   - Foreground Ratio = 전경 포인트 수 / 전체 포인트 수
   - 전처리 전후 비율을 비교하여 배경 노이즈 제거 효과를 정량화 (SNR 개선 정도의 대리 지표).

부가 지표 (라벨이 있는 경우, 즉 사람이 있는 장면에서 point-wise GT가 있을 때)
---------------------------------------------------------------------
   - Precision / Recall / F1 (foreground = 사람)
   - IoU (foreground), IoU (background)
   - Overall Accuracy (OA)
   - Over-removal Rate: 실제 사람(전경) 포인트인데 배경으로 오인되어 제거된 비율
     (과차단 지표. 응급상황 관점에서 "사람을 놓치는" 위험도)

사용 방법
---------
1) 빈 장면(사람 없음) 평가:
   python eval_bg_subtraction.py \
       --raw empty_raw.pcd --processed empty_processed.pcd --mode empty

2) 라벨이 있는 장면(사람 있음) 평가:
   python eval_bg_subtraction.py \
       --raw scene_raw.pcd --processed scene_processed.pcd \
       --labels scene_labels.npy --mode labeled

   * scene_labels.npy 는 raw 포인트와 같은 순서/개수를 가지는 1D 배열이며
     0 = 배경(background), 1 = 전경/사람(foreground) 값을 가져야 합니다.

3) 데모(합성 데이터)로 바로 테스트:
   python eval_bg_subtraction.py --demo

지원 파일 포맷: .npy, .npz(키 'points'), .csv/.txt (공백 또는 콤마 구분 XYZ...),
              .pcd (ASCII, XYZ 앞 3열만 사용). open3d가 설치되어 있으면 .ply 등도 지원.
"""

import argparse
import sys
import numpy as np
from scipy.spatial import cKDTree

try:
    import open3d as o3d  # 선택적 의존성
    _HAS_OPEN3D = True
except ImportError:
    _HAS_OPEN3D = False


# ---------------------------------------------------------------------------
# 1. 포인트 클라우드 입출력
# ---------------------------------------------------------------------------

def load_points(path: str) -> np.ndarray:
    """다양한 포맷의 포인트 클라우드 파일을 (N, 3) numpy 배열로 로드."""
    path_lower = path.lower()

    if path_lower.endswith(".npy"):
        arr = np.load(path)
    elif path_lower.endswith(".npz"):
        data = np.load(path)
        key = "points" if "points" in data else list(data.keys())[0]
        arr = data[key]
    elif path_lower.endswith((".csv", ".txt", ".xyz")):
        delim = "," if path_lower.endswith(".csv") else None
        arr = np.loadtxt(path, delimiter=delim)
    elif path_lower.endswith(".pcd"):
        arr = _load_pcd_ascii(path)
    elif _HAS_OPEN3D:
        pcd = o3d.io.read_point_cloud(path)
        arr = np.asarray(pcd.points)
    else:
        raise ValueError(
            f"지원하지 않는 파일 형식입니다: {path} "
            f"(open3d가 설치되어 있지 않아 .ply/.pcd(binary) 등은 읽을 수 없습니다. "
            f"pip install open3d --break-system-packages 로 설치하거나 "
            f".npy/.csv/.txt/.pcd(ascii) 형식을 사용하세요.)"
        )

    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(f"포인트 배열의 형태가 올바르지 않습니다: {arr.shape}")
    return arr[:, :3]


def _load_pcd_ascii(path: str) -> np.ndarray:
    """간단한 ASCII PCD 파서 (헤더의 DATA ascii 가정)."""
    with open(path, "r") as f:
        lines = f.readlines()

    data_start = None
    for i, line in enumerate(lines):
        if line.strip().upper().startswith("DATA"):
            if "ascii" not in line.lower():
                raise ValueError("binary PCD는 지원하지 않습니다. open3d를 설치해 주세요.")
            data_start = i + 1
            break
    if data_start is None:
        raise ValueError("PCD 헤더에서 DATA 필드를 찾을 수 없습니다.")

    rows = []
    for line in lines[data_start:]:
        line = line.strip()
        if not line:
            continue
        vals = [float(v) for v in line.split()]
        rows.append(vals[:3])
    return np.array(rows, dtype=np.float64)


# ---------------------------------------------------------------------------
# 2. Raw <-> Processed 매칭 (전처리가 포인트를 "삭제"만 하고 좌표를 바꾸지 않는다는 가정)
# ---------------------------------------------------------------------------

def match_kept_points(raw_points: np.ndarray, processed_points: np.ndarray,
                       tol: float = 1e-6) -> np.ndarray:
    """
    raw_points 각각에 대해 processed_points 안에 (거의) 동일한 좌표가 남아있는지 판정.

    Returns
    -------
    kept_mask : (N,) bool array
        True  -> 전처리 후에도 남아있는(=전경으로 판정된) 포인트
        False -> 전처리 과정에서 제거된(=배경으로 판정된) 포인트
    """
    if len(processed_points) == 0:
        return np.zeros(len(raw_points), dtype=bool)

    tree = cKDTree(processed_points)
    dist, _ = tree.query(raw_points, k=1)
    return dist <= tol


# ---------------------------------------------------------------------------
# 3. 빈 장면(사람 없음) 평가 — False Foreground Rate / Background Removal Rate
# ---------------------------------------------------------------------------

def evaluate_empty_scene(raw_points: np.ndarray, processed_points: np.ndarray,
                          tol: float = 1e-6) -> dict:
    """
    사람이 없는 상태로 녹화한 raw 포인트 클라우드에 대해,
    전처리 후 남은 포인트를 '오탐 전경(false foreground)'으로 간주하여 평가.
    """
    n_total = len(raw_points)
    if n_total == 0:
        raise ValueError("raw_points가 비어 있습니다.")

    kept_mask = match_kept_points(raw_points, processed_points, tol=tol)
    n_kept = int(kept_mask.sum())
    n_removed = n_total - n_kept

    ffr = n_kept / n_total                 # False Foreground Rate
    brr = n_removed / n_total              # Background Removal Rate (= 1 - FFR)

    return {
        "n_total_points": n_total,
        "n_kept_after_preprocessing": n_kept,
        "n_removed_after_preprocessing": n_removed,
        "false_foreground_rate": ffr,
        "background_removal_rate": brr,
    }


# ---------------------------------------------------------------------------
# 4. 라벨이 있는 장면(사람 있음) 평가 — Precision/Recall/IoU/OA + 과차단율
# ---------------------------------------------------------------------------

def evaluate_labeled_scene(raw_points: np.ndarray, processed_points: np.ndarray,
                            gt_labels: np.ndarray, tol: float = 1e-6) -> dict:
    """
    raw_points 에 대응하는 point-wise GT 라벨(0=배경, 1=전경/사람)이 있을 때
    confusion matrix 기반 지표를 계산.

    예측 규칙: 전처리 후에도 남아있는 포인트 -> 예측 전경(1)
              전처리 과정에서 제거된 포인트   -> 예측 배경(0)
    """
    gt_labels = np.asarray(gt_labels).astype(int)
    if len(gt_labels) != len(raw_points):
        raise ValueError(
            f"gt_labels 길이({len(gt_labels)})가 raw_points 개수({len(raw_points)})와 다릅니다."
        )

    kept_mask = match_kept_points(raw_points, processed_points, tol=tol)
    pred = kept_mask.astype(int)  # 1 = 전경 예측, 0 = 배경 예측
    gt = gt_labels

    tp = int(np.sum((pred == 1) & (gt == 1)))   # 사람인데 전경으로 남음 (정상)
    fp = int(np.sum((pred == 1) & (gt == 0)))   # 배경인데 전경으로 남음 (false foreground)
    fn = int(np.sum((pred == 0) & (gt == 1)))   # 사람인데 배경으로 제거됨 (과차단, over-removal)
    tn = int(np.sum((pred == 0) & (gt == 0)))   # 배경인데 배경으로 제거됨 (정상)

    n_total = tp + fp + fn + tn
    n_gt_bg = tn + fp
    n_gt_fg = tp + fn

    precision_fg = tp / (tp + fp) if (tp + fp) > 0 else float("nan")
    recall_fg = tp / (tp + fn) if (tp + fn) > 0 else float("nan")
    f1_fg = (2 * precision_fg * recall_fg / (precision_fg + recall_fg)
             if (precision_fg + recall_fg) > 0 else float("nan"))

    iou_fg = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else float("nan")
    iou_bg = tn / (tn + fp + fn) if (tn + fp + fn) > 0 else float("nan")

    oa = (tp + tn) / n_total if n_total > 0 else float("nan")

    # False Foreground Rate (배경 기준 FPR) : 배경인데 전경으로 남은 비율
    ffr = fp / n_gt_bg if n_gt_bg > 0 else float("nan")
    # Background Removal Rate (배경 Recall) : 배경 중 실제로 제거된 비율
    brr = tn / n_gt_bg if n_gt_bg > 0 else float("nan")
    # Over-removal Rate (= FNR for foreground) : 사람인데 잘못 제거된 비율 (응급상황 시 매우 중요)
    over_removal_rate = fn / n_gt_fg if n_gt_fg > 0 else float("nan")

    return {
        "confusion_matrix": {"TP": tp, "FP": fp, "FN": fn, "TN": tn},
        "n_gt_background": n_gt_bg,
        "n_gt_foreground": n_gt_fg,
        "false_foreground_rate": ffr,          # 배경 -> 전경 오탐율
        "background_removal_rate": brr,        # 배경 제거율 (recall)
        "over_removal_rate": over_removal_rate,  # 사람 -> 배경 오제거율 (miss)
        "precision_foreground": precision_fg,
        "recall_foreground": recall_fg,
        "f1_foreground": f1_fg,
        "iou_foreground": iou_fg,
        "iou_background": iou_bg,
        "overall_accuracy": oa,
    }


# ---------------------------------------------------------------------------
# 5. Foreground Ratio (SNR 개념) — 전처리 전후 비교
# ---------------------------------------------------------------------------

def foreground_ratio_before_after(raw_points: np.ndarray, processed_points: np.ndarray,
                                   gt_labels: np.ndarray = None,
                                   tol: float = 1e-6) -> dict:
    """
    전처리 전후의 전경 비율 변화를 계산.

    gt_labels가 주어지면 '실제' 전경 비율(라벨 기반)을,
    없으면 '추정' 전경 비율(전처리 후 남은 포인트 = 전경으로 간주)을 사용.
    """
    n_total_before = len(raw_points)
    n_total_after = len(processed_points)

    if gt_labels is not None:
        gt_labels = np.asarray(gt_labels).astype(int)
        ratio_before = float(np.mean(gt_labels == 1)) if n_total_before > 0 else float("nan")

        kept_mask = match_kept_points(raw_points, processed_points, tol=tol)
        # 전처리 후 실제로 남은 포인트 중 GT 전경 비율
        if kept_mask.sum() > 0:
            ratio_after = float(np.mean(gt_labels[kept_mask] == 1))
        else:
            ratio_after = float("nan")
    else:
        # 라벨이 없으면: "before"는 정의할 수 없으므로(전부 배경/전경 혼재 가정 불가) None 처리
        ratio_before = None
        ratio_after = 1.0 if n_total_after > 0 else float("nan")  # 남은 점을 전경으로 간주

    return {
        "n_total_before": n_total_before,
        "n_total_after": n_total_after,
        "foreground_ratio_before": ratio_before,
        "foreground_ratio_after": ratio_after,
        "improvement": (
            (ratio_after - ratio_before)
            if (ratio_before is not None and ratio_after is not None
                and not np.isnan(ratio_before) and not np.isnan(ratio_after))
            else None
        ),
    }


# ---------------------------------------------------------------------------
# 6. 결과 출력
# ---------------------------------------------------------------------------

def print_report(mode: str, result: dict, ratio_result: dict = None):
    print("=" * 60)
    print(f" 배경차감 전처리 평가 결과  (mode = {mode})")
    print("=" * 60)

    if mode == "empty":
        print(f"전체 포인트 수                : {result['n_total_points']}")
        print(f"전처리 후 남은 포인트 수       : {result['n_kept_after_preprocessing']}")
        print(f"전처리 후 제거된 포인트 수     : {result['n_removed_after_preprocessing']}")
        print(f"False Foreground Rate (FFR)   : {result['false_foreground_rate']:.4%}")
        print(f"Background Removal Rate (BRR) : {result['background_removal_rate']:.4%}")

    elif mode == "labeled":
        cm = result["confusion_matrix"]
        print(f"Confusion Matrix  TP={cm['TP']}  FP={cm['FP']}  FN={cm['FN']}  TN={cm['TN']}")
        print(f"GT 배경 포인트 수              : {result['n_gt_background']}")
        print(f"GT 전경(사람) 포인트 수        : {result['n_gt_foreground']}")
        print("-" * 60)
        print(f"False Foreground Rate (배경→전경 오탐율)   : {result['false_foreground_rate']:.4%}")
        print(f"Background Removal Rate (배경 제거율)      : {result['background_removal_rate']:.4%}")
        print(f"Over-removal Rate (사람→배경 오제거율, 과차단): {result['over_removal_rate']:.4%}")
        print("-" * 60)
        print(f"Precision (foreground)         : {result['precision_foreground']:.4f}")
        print(f"Recall (foreground)            : {result['recall_foreground']:.4f}")
        print(f"F1 (foreground)                : {result['f1_foreground']:.4f}")
        print(f"IoU (foreground)               : {result['iou_foreground']:.4f}")
        print(f"IoU (background)               : {result['iou_background']:.4f}")
        print(f"Overall Accuracy (OA)          : {result['overall_accuracy']:.4%}")

    if ratio_result is not None:
        print("-" * 60)
        print(" Foreground Ratio (SNR 개념)")
        print(f"  전처리 전 총 포인트 수  : {ratio_result['n_total_before']}")
        print(f"  전처리 후 총 포인트 수  : {ratio_result['n_total_after']}")
        if ratio_result["foreground_ratio_before"] is not None:
            print(f"  전경 비율 (전)          : {ratio_result['foreground_ratio_before']:.4%}")
        print(f"  전경 비율 (후)          : {ratio_result['foreground_ratio_after']:.4%}")
        if ratio_result["improvement"] is not None:
            print(f"  전경 비율 개선폭         : {ratio_result['improvement']:+.4%}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# 7. 데모 (합성 데이터로 동작 확인)
# ---------------------------------------------------------------------------

def run_demo():
    rng = np.random.default_rng(42)

    print("\n[DEMO 1] 빈 장면(사람 없음) 평가 — 배경차감이 배경 포인트 95%를 제거했다고 가정")
    n_bg = 5000
    raw_empty = rng.uniform(-3, 3, size=(n_bg, 3))
    n_remaining = int(n_bg * 0.05)  # 5%가 노이즈로 잘못 남음
    processed_empty = raw_empty[:n_remaining]  # 앞부분 포인트가 우연히 안 지워졌다고 가정
    result_empty = evaluate_empty_scene(raw_empty, processed_empty)
    print_report("empty", result_empty)

    print("\n[DEMO 2] 라벨이 있는 장면(사람 있음) 평가")
    n_bg2, n_fg2 = 4000, 300
    bg_points = rng.uniform(-3, 3, size=(n_bg2, 3))
    fg_points = rng.uniform(-0.5, 0.5, size=(n_fg2, 3)) + np.array([1.0, 0.0, 0.5])
    raw_scene = np.vstack([bg_points, fg_points])
    gt_labels = np.concatenate([np.zeros(n_bg2, dtype=int), np.ones(n_fg2, dtype=int)])

    # 전처리 시뮬레이션: 배경의 97% 제거(3% 잔류), 사람 포인트의 90% 유지(10% 과차단)
    bg_keep_mask = rng.random(n_bg2) < 0.03
    fg_keep_mask = rng.random(n_fg2) < 0.90
    processed_scene = np.vstack([bg_points[bg_keep_mask], fg_points[fg_keep_mask]])

    result_labeled = evaluate_labeled_scene(raw_scene, processed_scene, gt_labels)
    ratio_result = foreground_ratio_before_after(raw_scene, processed_scene, gt_labels)
    print_report("labeled", result_labeled, ratio_result)


# ---------------------------------------------------------------------------
# 8. CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="포인트 클라우드 배경차감 전처리 성능 평가 (FFR / BRR / Foreground Ratio 등)"
    )
    parser.add_argument("--raw", type=str, help="전처리 전 원본 포인트 클라우드 파일 경로")
    parser.add_argument("--processed", type=str, help="전처리 후 포인트 클라우드 파일 경로")
    parser.add_argument("--labels", type=str, default=None,
                         help="raw_points 순서에 대응하는 GT 라벨 파일 (.npy, 0=배경/1=전경). "
                              "제공 시 mode=labeled 평가도 함께 수행")
    parser.add_argument("--mode", type=str, choices=["empty", "labeled"], default=None,
                         help="empty: 빈 장면 평가 / labeled: 라벨 기반 평가 "
                              "(labels 인자를 주면 자동으로 labeled로 판단)")
    parser.add_argument("--tol", type=float, default=1e-6,
                         help="raw-processed 포인트 매칭 허용 오차 (기본 1e-6)")
    parser.add_argument("--demo", action="store_true", help="합성 데이터로 데모 실행")

    args = parser.parse_args()

    if args.demo:
        run_demo()
        return

    if not args.raw or not args.processed:
        parser.error("--raw 와 --processed 경로를 지정하거나 --demo 를 사용하세요.")

    raw_points = load_points(args.raw)
    processed_points = load_points(args.processed)

    mode = args.mode
    if args.labels is not None:
        mode = "labeled"
    elif mode is None:
        mode = "empty"

    if mode == "empty":
        result = evaluate_empty_scene(raw_points, processed_points, tol=args.tol)
        ratio_result = foreground_ratio_before_after(raw_points, processed_points, tol=args.tol)
        print_report("empty", result, ratio_result)
    else:
        if args.labels is None:
            parser.error("mode=labeled 평가를 위해서는 --labels 인자가 필요합니다.")
        gt_labels = np.load(args.labels) if args.labels.endswith(".npy") \
            else np.loadtxt(args.labels)
        result = evaluate_labeled_scene(raw_points, processed_points, gt_labels, tol=args.tol)
        ratio_result = foreground_ratio_before_after(raw_points, processed_points, gt_labels, tol=args.tol)
        print_report("labeled", result, ratio_result)


if __name__ == "__main__":
    main()
