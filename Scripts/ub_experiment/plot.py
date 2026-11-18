#!/usr/bin/env python3
"""Render client experiment metrics, including legacy UB-MR recordings."""
import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path


def load_metrics(path):
    # Retain one aggregate per second/series, rather than every LiDAR scan.
    timing = defaultdict(dict)
    traffic = defaultdict(list)
    summary = {}
    invalid_rows = 0
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            try:
                t = float(row["elapsed_s"])
                if not math.isfinite(t) or t < 0:
                    raise ValueError("invalid timestamp")
                metric, source = row["metric"], row["source"]
                if not metric or not source:
                    raise ValueError("missing metric or source")
                if metric in ("receive_to_publish", "redis_rtt", "network_rtt", "carla_rpc_rtt") or row.get("duration_ms") not in ("", None):
                    value = float(row["duration_ms"])
                    if not math.isfinite(value) or value < 0:
                        raise ValueError("invalid duration")
                    bucket = timing[(metric, source)].setdefault(int(t), [0, 0.0, value, value])
                    bucket[0] += 1
                    bucket[1] += value
                    bucket[2] = min(bucket[2], value)
                    bucket[3] = max(bucket[3], value)
                elif metric == "throughput":
                    available = row["available"] == "1"
                    rx, tx = float(row["rx_Bps"] or "nan"), float(row["tx_Bps"] or "nan")
                    if available and (not math.isfinite(rx) or not math.isfinite(tx) or min(rx, tx) < 0):
                        raise ValueError("invalid rate")
                    received, sent = int(row["rx_bytes"]), int(row["tx_bytes"])
                    traffic[source].append((t, rx if available else math.nan, tx if available else math.nan))
                    summary[source] = {"received_bytes": received, "sent_bytes": sent}
            except (KeyError, TypeError, ValueError):
                # A crash can leave an incomplete final CSV row. Keep earlier data.
                invalid_rows += 1
    return timing, traffic, summary, invalid_rows


def with_gaps(points, max_gap=2.5):
    result = []
    previous = None
    for point in sorted(points):
        if previous is not None and point[0] - previous > max_gap:
            result.append((previous + .001,) + (math.nan,) * (len(point) - 1))
        result.append(point)
        previous = point[0]
    return result


def series_label(source):
    return re.sub(r"\[([0-9a-f]{32})\]", lambda match: "[" + match[1][:8] + "]", source)


def render(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    timing, traffic, totals, invalid = load_metrics(directory / "metrics.csv")
    metadata_path = directory / "experiment.json"
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    client = metadata.get("client_type", "Client")
    summary = {"invalid_rows_skipped": invalid, "timings": {}, "traffic_last_sample_totals": totals}
    summary["experiment"] = metadata
    status = directory / "recording-status.txt"
    summary["recording_status"] = status.read_text() if status.exists() else "No clean shutdown status; recording may be incomplete."

    def finish(fig, axes, title, filename):
        fig.suptitle(title)
        for ax in axes:
            ax.set_xlabel("Elapsed experiment time (s)")
            ax.grid(alpha=.25)
            if ax.lines:
                ax.legend(fontsize=7, loc="best")
            else:
                ax.text(.5, .5, "No samples recorded", ha="center", va="center", transform=ax.transAxes)
        fig.tight_layout()
        fig.savefig(directory / (filename + ".png"), dpi=160)
        plt.close(fig)

    metrics = {kind for kind, _ in timing}
    figures = [("receive_to_publish", "Receive/input to successful local publish/control call", "receive_to_publish"),
               ("rtt", "Network/service RTT", "network_rtt")]
    figures += [(kind, kind.replace("_", " "), "timing_" + re.sub(r"[^A-Za-z0-9_-]", "_", kind))
                for kind in sorted(metrics) if kind != "receive_to_publish" and not kind.endswith("rtt")]
    for metric, title, filename in figures:
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.set_ylabel("Milliseconds")
        for (kind, source), buckets in sorted(timing.items()):
            if kind != metric and not (metric == "rtt" and kind.endswith("rtt")):
                continue
            points = with_gaps([(second + .5, b[1] / b[0], b[2], b[3]) for second, b in buckets.items()])
            x, mean, low, high = zip(*points)
            label = series_label(source) + (" / " + kind if metric == "rtt" else "")
            line, = ax.plot(x, mean, ".-", markersize=3, linewidth=1, label=label)
            ax.fill_between(x, low, high, color=line.get_color(), alpha=.15)
            count = sum(b[0] for b in buckets.values())
            summary["timings"][source + "/" + kind] = {
                "count": count, "mean_ms": sum(b[1] for b in buckets.values()) / count,
                "min_ms": min(b[2] for b in buckets.values()), "max_ms": max(b[3] for b in buckets.values()),
            }
        finish(fig, [ax], client + ": " + title + " (1 s mean; shading = min/max)", filename)

    groups = sorted({source.split(" [", 1)[0] for source in traffic})
    fig, axes = plt.subplots(max(1, len(groups)), 1, figsize=(12, 4 * max(1, len(groups))), squeeze=False)
    axes = list(axes[:, 0])
    for ax, prefix in zip(axes, groups):
        divisor = 1048576 if prefix == "ros_sensor_payload" else 1024
        unit = "MiB/s" if divisor == 1048576 else "KiB/s"
        ax.set_ylabel(prefix + " (" + unit + ")")
        description = metadata.get("traffic_scopes", {}).get(prefix)
        if description:
            ax.set_title(description, fontsize=8, wrap=True)
        for source, points in sorted(traffic.items()):
            if source.split(" [", 1)[0] != prefix:
                continue
            x, rx, tx = zip(*with_gaps(points))
            line, = ax.plot(x, [v / divisor for v in rx], ".-", markersize=3, label=series_label(source) + " RX", linewidth=1)
            ax.plot(x, [v / divisor for v in tx], ".--", markersize=3, color=line.get_color(), label=series_label(source) + " TX", linewidth=1)
    finish(fig, axes, client + ": throughput (elapsed-time rates; measurement scopes shown separately)", "network_throughput")
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Saved experiment graphs and summary to {directory}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="Experiment run directory, or parent containing run directories")
    args = parser.parse_args()
    files = sorted(args.directory.rglob("metrics.csv"))
    if not files:
        parser.error(f"No metrics.csv found in {args.directory}. Enable recording in the client before running an experiment.")
    for path in files:
        render(path.parent)


if __name__ == "__main__":
    main()
