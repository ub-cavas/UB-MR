import csv
import math
import tempfile
import unittest
from pathlib import Path

from plot_experiment_metrics import load_metrics, with_gaps


class MetricsChecks(unittest.TestCase):
    def test_raw_measurements_and_unavailable_rates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow("elapsed_s,utc,metric,source,duration_ms,bypass,connected,rx_bytes,tx_bytes,rx_Bps,tx_Bps,available".split(","))
                writer.writerow([.1, "", "receive_to_publish", 'front, "LiDAR"', 10, 0, "", "", "", "", "", 1])
                writer.writerow([.9, "", "receive_to_publish", 'front, "LiDAR"', 30, 1, "", "", "", "", "", 1])
                writer.writerow([1, "", "throughput", "redis [first]", "", "", "", 100, 200, 100, 200, 1])
                writer.writerow([2, "", "throughput", "redis [first]", "", "", "", 100, 200, 0, 0, 1])
                writer.writerow([3, "", "throughput", "redis [first]", "", "", "", 100, 200, "", "", 0])
                stream.write("4,,redis_rtt,interrupted\n")
            timing, traffic, summary, invalid = load_metrics(path)
            self.assertEqual(timing[("receive_to_publish", 'front, "LiDAR"')][0], [2, 40, 10, 30])
            self.assertEqual(traffic["redis [first]"][1], (2, 0, 0))
            self.assertTrue(math.isnan(traffic["redis [first]"][2][1]))
            self.assertEqual(summary["redis [first]"], {"received_bytes": 100, "sent_bytes": 200})
            self.assertEqual(invalid, 1)

    def test_gaps_do_not_connect_inactive_periods(self):
        points = with_gaps([(10, 7), (1, 2), (2, 3)])
        self.assertEqual(len(points), 4)
        self.assertTrue(math.isnan(points[2][1]))
        self.assertEqual(points[-1], (10, 7))


if __name__ == "__main__":
    unittest.main()
