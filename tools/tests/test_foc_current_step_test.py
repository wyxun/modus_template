import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import foc_current_step_test as step_test


class CurrentStepAnalysisTests(unittest.TestCase):
    def test_prefers_sibling_aitrace_that_exports_sample_indices(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir) / "modus_template"
            sibling_aitrace = (
                repo_root.parent / "mstudio" / "aitrace" / "aitrace.exe"
            )
            legacy_aitrace = repo_root / "tools" / "aitrace.exe"
            sibling_aitrace.parent.mkdir(parents=True)
            legacy_aitrace.parent.mkdir(parents=True)
            sibling_aitrace.touch()
            legacy_aitrace.touch()

            resolved = step_test._resolve_aitrace(repo_root)

        self.assertEqual(resolved, sibling_aitrace)

    def test_uses_repo_aitrace_when_sibling_build_is_missing(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir) / "modus_template"
            legacy_aitrace = repo_root / "tools" / "aitrace.exe"
            legacy_aitrace.parent.mkdir(parents=True)
            legacy_aitrace.touch()

            resolved = step_test._resolve_aitrace(repo_root)

        self.assertEqual(resolved, legacy_aitrace)

    def test_counts_only_complete_waveform_rows_for_capture_baseline(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "partial.csv"
            csv_path.write_text(
                "time,sample_index,IqRef,Iq\n"
                "0,1,0,0\n"
                "0.0001,2,,\n"
                "0.0002,3,0,0\n",
                encoding="utf-8",
            )

            count = step_test._csv_data_rows(csv_path)

        self.assertEqual(count, 2)

    def test_waits_for_waveform_metadata_header_before_counting(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "header.csv"
            csv_path.write_text("time,IqRef,Iq\n", encoding="utf-8")

            count = step_test._csv_data_rows(csv_path)

        self.assertEqual(count, 0)

    def test_analyzes_two_channel_csv_using_metadata_time_and_index(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "step.csv"
            lines = ["time,sample_index,IqRef,Iq"]
            timestamp = 0.0
            sample_index = 100
            for _ in range(20):
                lines.append(f"{timestamp:.4f},{sample_index},0,0")
                timestamp += 0.0001
                sample_index += 1
            lines.append(f"{timestamp:.4f},{sample_index},,")
            # A capture-buffer gap before the test window must not poison it.
            timestamp += 82.0
            sample_index += 820000
            for _ in range(600):
                lines.append(f"{timestamp:.4f},{sample_index},0,0")
                timestamp += 0.0001
                sample_index += 1
            for iq in (0.0, 0.01, 0.02, 0.04, 0.05, 0.06, 0.05):
                lines.append(
                    f"{timestamp:.4f},{sample_index},0.05,{iq}"
                )
                timestamp += 0.0001
                sample_index += 1
            for iq in (0.02, 0.0):
                lines.append(f"{timestamp:.4f},{sample_index},0,{iq}")
                timestamp += 0.0001
                sample_index += 1
            csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

            result = step_test.analyze_csv(
                csv_path, sample_rate_hz=2000, target_iq=0.05,
                smoothing_samples=1,
            )

        self.assertEqual(result.sample_count, 629)
        self.assertEqual(result.step_index, 620)
        self.assertEqual(result.end_index, 627)
        self.assertAlmostEqual(result.rise_time_ms, 0.3)
        self.assertAlmostEqual(result.actual_pulse_ms, 0.7)
        self.assertAlmostEqual(result.measured_sample_rate_hz, 10000.0,
                               places=3)
        self.assertIsNone(result.speed_start_pu)

    def test_rejects_missing_sample_inside_step_window(self):
        rows = ["time,sample_index,IqRef,Iq"]
        for index in range(20):
            sample_index = 100 + index
            if index >= 15:
                sample_index += 1
            reference = 0.05 if index >= 10 else 0.0
            rows.append(f"{index * 0.0001:.4f},{sample_index},{reference},0")

        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "step.csv"
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "sample gap"):
                step_test.analyze_csv(
                    csv_path, sample_rate_hz=10000, target_iq=0.05,
                    smoothing_samples=1,
                )

    def test_uses_capture_timestamps_for_rise_and_pulse_duration(self):
        rows = ["time,sample_index,IqRef,Iq,ElecSpe"]
        samples = [(0.0, 0.0)] * 12
        samples.extend((0.05, iq) for iq in (0.0, 0.01, 0.02, 0.04,
                                             0.05, 0.06, 0.05))
        samples.extend(((0.0, 0.02), (0.0, 0.0)))
        for index, (reference, iq) in enumerate(samples):
            timestamp = index * 0.0002
            speed = 0.1
            rows.append(
                f"{timestamp:.4f},{500 + index},{reference},{iq},{speed}"
            )

        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "step.csv"
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            result = step_test.analyze_csv(
                csv_path, sample_rate_hz=10000, target_iq=0.05,
                smoothing_samples=1,
            )

        self.assertEqual(result.step_index, 12)
        self.assertEqual(result.end_index, 19)
        self.assertAlmostEqual(result.rise_time_ms, 0.6)
        self.assertAlmostEqual(result.actual_pulse_ms, 1.4)
        self.assertAlmostEqual(result.measured_sample_rate_hz, 5000.0,
                               places=3)
        self.assertAlmostEqual(result.peak_iq, 0.06)
        self.assertAlmostEqual(result.overshoot_percent, 20.0)
        self.assertAlmostEqual(result.speed_end_pu, 0.1)

    def test_separates_target_overshoot_from_peak_above_steady_level(self):
        rows = ["time,sample_index,IqRef,Iq"]
        values = [(0.0, 0.0)] * 20
        values.extend((0.05, iq) for iq in
                      (0.0, 0.02, 0.04, 0.043, 0.04, 0.041, 0.04))
        values.append((0.0, 0.0))
        for index, (reference, iq) in enumerate(values):
            rows.append(f"{index * 0.0001:.4f},{index},{reference},{iq}")

        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "step.csv"
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
            result = step_test.analyze_csv(
                csv_path, sample_rate_hz=10000, target_iq=0.05,
                smoothing_samples=1,
            )

        self.assertEqual(result.overshoot_percent, 0.0)
        self.assertAlmostEqual(result.peak_above_steady_percent, 7.5)

    def test_rejects_unbounded_pulse_and_current(self):
        with self.assertRaises(ValueError):
            step_test.validate_test_parameters(0.05, 151)
        with self.assertRaises(ValueError):
            step_test.validate_test_parameters(0.06, 100)


if __name__ == "__main__":
    unittest.main()
