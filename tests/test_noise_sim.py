"""Run: python -m unittest discover -s tests -p test_noise_sim.py -v"""
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import numpy as np

import noise_sim as sim


def samples(word, widths=(2, 4), extra=False):
    result = []
    for bit in range(15, -1, -1):
        width = widths[(word >> bit) & 1]
        result.extend([1] * width + [0] * (5 - width + int(extra)))
    return result


def receive(sequence, word):
    receiver = sim.DshotSampler(lambda value, start, end: 0 if value == word else None)
    return [event for time, high in enumerate(sequence)
            if (event := receiver.sample(high, time * 50_000)) is not None]


class SimulationTests(unittest.TestCase):
    def test_waveform_window_first_error_and_zero_fallback(self):
        self.assertEqual(sim.waveform_window([]), (0, None))
        self.assertEqual(sim.waveform_window([dict(time_ns=10_000_000, status="accepted_correct", width_error_us=0)]),
                         (0, None))
        events = [dict(time_ns=50_000_000, status="rejected"),
                  dict(time_ns=30_000_000, status="accepted_false"),
                  dict(time_ns=20_000_000, status="expected_update_missing")]
        self.assertEqual(sim.waveform_window(events), (18_000_000, 20_000_000))
        self.assertEqual(sim.waveform_window([dict(time_ns=1_000_000, status="rejected")]), (0, 1_000_000))
        self.assertEqual(sim.waveform_window([dict(time_ns=10_000_000, status="accepted_correct", width_error_us=1)]),
                         (8_000_000, 10_000_000))

    def test_waveform_preview_alignment_and_full_filter_history(self):
        summary, events, plots, arrays = sim.simulate(sim.Settings(duration_s=.05, seed=42), True)
        start_ns, first_ns = sim.waveform_window(events)
        self.assertEqual(summary["waveform_window_start_s"], start_ns / 1e9)
        self.assertIsNotNone(first_ns)
        for method in sim.METHODS:
            p = plots[method]
            self.assertEqual(p["window_start_ns"], start_ns)
            section = slice(start_ns // 1000, int(np.searchsorted(arrays["time_ns"], start_ns + 25_000_000)))
            np.testing.assert_array_equal(p["filtered"], arrays[f"{method}_filtered_v"][section])
        # The memory-saving path must produce exactly the same previews.
        other, _, other_plots, _ = sim.simulate(sim.Settings(duration_s=.05, seed=42))
        self.assertEqual(other["waveform_window_start_s"], summary["waveform_window_start_s"])
        for method in sim.METHODS:
            np.testing.assert_array_equal(plots[method]["filtered"], other_plots[method]["filtered"])

    def test_noise_rms_and_reproducibility(self):
        a = sim.generate_noise(10000, 1.2, np.random.default_rng(42))
        b = sim.generate_noise(10000, 1.2, np.random.default_rng(42))
        np.testing.assert_array_equal(a, b)
        self.assertAlmostEqual(float(a.mean()), 0)
        self.assertAlmostEqual(float(np.sqrt(np.mean(a * a))), 1.2)
        self.assertFalse(sim.generate_noise(10, 0, np.random.default_rng()).any())

    def test_filter_bypass_and_time_constant(self):
        values = np.ones(100)
        np.testing.assert_array_equal(sim.low_pass(values, 1, 0), values)
        response = sim.low_pass(values, 1, 5)
        self.assertAlmostEqual(response[0], 1 - np.exp(-1 / 5))
        self.assertAlmostEqual(response[4], 1 - np.exp(-1))
        fine = sim.low_pass(np.ones(1000), .1, 5)
        np.testing.assert_allclose(response, fine[9::10], atol=1e-12)

    def test_schmitt_retains_state_inside_band(self):
        values = np.array([1.6, 1.75, 1.6, 1.55, 1.6, 2, 0])
        np.testing.assert_array_equal(sim.schmitt(values, 1.75, 1.55),
                                      [0, 1, 1, 0, 0, 1, 0])

    def test_setting_validation(self):
        for kwargs in ({"step_us": .15}, {"step_us": 0}, {"noise_rms_v": -1},
                       {"filter_tau_us": -1}, {"duration_s": .0001},
                       {"vih_v": 1, "vil_v": 2}, {"duration_s": float("nan")}, {"seed": -1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                sim.Settings(**kwargs).validate()
        sim.Settings(step_us=.1).validate()

    def test_zero_noise_all_methods_and_seeds(self):
        for seed in range(5):
            summary, _, _, _ = sim.simulate(sim.Settings(duration_s=.05, noise_rms_v=0, seed=seed))
            for row in summary["reports"]:
                self.assertGreater(row["expected_updates"], 0)
                self.assertEqual(row["failed_updates"], 0, (seed, row))
                self.assertEqual(row["false_command_s"], 0)
                self.assertEqual(row["overdue_update_s"], 0)
            self.assertEqual(summary["waveform_window_start_s"], 0)
            self.assertIsNone(summary["first_signal_error_s"])

    def test_coarse_baseline_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "baseline"):
            sim.simulate(sim.Settings(duration_s=.05, step_us=100, noise_rms_v=0))

    def test_exact_pwm_schedule_and_boundary_fragment(self):
        duration = 50_000_000
        times = np.arange(0, duration, 1000)
        _, expected, _ = sim.waveform("PWM333", times, duration)
        self.assertEqual(expected[1][0], sim.IDLE_NS + round(1e9 / 333))
        self.assertTrue(all(e < duration for _, e in expected))
        np.testing.assert_equal(sim.captured_pulses(np.array([0, 1, 1, 0, 1, 1]), 1000), [(1000, 3000)])

    def test_pwm_strict_acceptance_boundaries(self):
        for width, valid in ((800, False), (801, True), (1500, True), (2199, True), (2200, False)):
            binary = np.zeros(5000, dtype=bool)
            binary[100:100 + width] = True
            expected = [(100_000, 1_600_000)]
            report, _, _, _ = sim.analyze_pwm(binary, expected, expected, 1000, 10_000, 5_000_000)
            self.assertEqual(report["received_updates"], int(valid))
            self.assertEqual(report["rejected_candidates"], int(not valid))

    def test_pwm_altered_spurious_pulses_and_held_command_time(self):
        # Wrong matched pulse ends at 2.4 ms, spurious pulse at 5 ms;
        # the correct next command ends at 7.5 ms. False hold = 5.1 ms.
        binary = np.zeros(10000, dtype=bool)
        binary[1000:2400] = True
        binary[4000:5000] = True
        binary[6000:7500] = True
        expected = [(1_000_000, 2_500_000), (6_000_000, 7_500_000)]
        report, _, _, _ = sim.analyze_pwm(binary, expected, expected, 1000, 10_000, 10_000_000)
        self.assertEqual(report["width_changed_pulses"], 1)
        self.assertEqual(report["unmatched_accepted_pulses"], 1)
        self.assertEqual(report["false_command_pulses"], 2)
        self.assertAlmostEqual(report["false_command_s"], .0051)
        self.assertEqual(report["failed_updates"], 0)

    def test_pwm_tolerance_boundary_and_false_command_to_end(self):
        expected = [(1_000_000, 2_500_000)]
        for delta, incorrect in ((10, False), (11, True)):
            binary = np.zeros(5000, dtype=bool)
            binary[1000:2500 + delta] = True
            report, _, _, _ = sim.analyze_pwm(binary, expected, expected, 1000, 10_000, 5_000_000)
            self.assertEqual(report["width_changed_pulses"], 1)
            self.assertEqual(report["false_command_pulses"], int(incorrect))
            self.assertAlmostEqual(report["false_command_s"], .002489 if incorrect else 0)

    def test_missing_deadlines_integrated_as_union(self):
        intervals = sim.overdue_intervals([10, 20, 30], {0, 1}, [35], 50)
        self.assertEqual(intervals, [(10, 35), (20, 35)])
        self.assertAlmostEqual(sim.union_duration(intervals), 25e-9)
        self.assertAlmostEqual(sim.union_duration(sim.overdue_intervals([10], {0}, [], 50)), 40e-9)

    def test_average_signal_loss_uses_continuous_episodes(self):
        report = sim.base_report("test", 5, 2, 0,
                                 [(10, 35), (20, 35), (35, 40), (60, 80), (90, 90)])
        self.assertEqual(report["signal_loss_episodes"], 2)
        self.assertAlmostEqual(report["overdue_update_s"], 50e-9)
        self.assertAlmostEqual(report["average_signal_loss_s"], 25e-9)
        report = sim.base_report("test", 5, 5, 0, [])
        self.assertEqual(report["signal_loss_episodes"], 0)
        self.assertEqual(report["average_signal_loss_s"], 0)
        # A loss still active at capture end is included with its observed length.
        report = sim.base_report("test", 1, 0, 0, sim.overdue_intervals([10], {0}, [], 50))
        self.assertAlmostEqual(report["average_signal_loss_s"], 40e-9)

    def test_dshot_five_six_mixed_bits_and_back_to_back(self):
        for widths in ((1, 3), (1, 4), (2, 3), (2, 4)):
            for extra in (False, True):
                events = receive([0] * 5 + samples(sim.WORD, widths, extra) * 2, sim.WORD)
                self.assertEqual([e["status"] for e in events], ["accepted_correct"] * 2)
        mixed = []
        for bit in range(15, -1, -1):
            width = 4 if (sim.WORD >> bit) & 1 else 2
            mixed += [1] * width + [0] * (5 - width + bit % 2)
        self.assertEqual(receive([0] * 5 + mixed, sim.WORD)[0]["status"], "accepted_correct")

    def test_dshot_corruption_idle_recovery_and_invalid_patterns(self):
        events = receive([0] * 5 + samples(sim.WORD ^ 1) + [0] * 5 + samples(sim.WORD), sim.WORD)
        self.assertEqual([e["status"] for e in events], ["rejected", "accepted_correct"])
        for malformed in ([1, 0, 1], [1] * 5, [1, 1, 0, 0, 0, 0, 0]):
            events = receive([0] * 5 + malformed + [0] * 5 + samples(sim.WORD), sim.WORD)
            self.assertEqual([e["status"] for e in events], ["rejected", "accepted_correct"])
        self.assertEqual(receive(samples(sim.WORD)[:40], sim.WORD), [])

    def test_sampler_phase_quantization_and_dshot_packet_totals(self):
        settings = sim.Settings(duration_s=.015, step_us=.1, noise_rms_v=0)
        summary, _, _, arrays = sim.simulate(settings, save_waveforms=True)
        self.assertIn("DSHOT16_filtered_v", arrays)
        self.assertEqual(len(arrays["noise_v"]), 150000)
        for row in summary["reports"][2:]:
            self.assertEqual(row["received_updates"] + row["corrupted_packets"], row["expected_updates"])

    def test_rejected_dshot_has_no_false_command_and_counts_frames_once(self):
        settings = sim.Settings(duration_s=.02)
        times = np.arange(0, settings.duration_ns, settings.step_ns)
        clean, expected, tick = sim.waveform("DSHOT4", times, settings.duration_ns)
        binary = sim.schmitt(sim.low_pass(clean, 1, 5), 1.75, 1.55)
        report, _, _, _ = sim.analyze_dshot(np.zeros_like(binary), binary, expected, settings, tick, 0)
        self.assertEqual(report["corrupted_packets"], len(expected))
        self.assertEqual(report["false_command_s"], 0)
        # First clean receipt ends at 5 ms at this phase (first edge at 1.05 ms).
        self.assertAlmostEqual(report["overdue_update_s"], .015)

    def test_output_names_and_reports(self):
        summary, events, plots, arrays = sim.simulate(sim.Settings(duration_s=.025, noise_rms_v=0), True)
        with tempfile.TemporaryDirectory() as directory:
            stamp = "20261007T120000_123456Z"
            run = sim.save_outputs(Path(directory), stamp, summary, events, plots, arrays)
            self.assertEqual(list(Path(directory).iterdir()), [run])
            files = list(run.iterdir())
            self.assertEqual(len(files), 9)
            self.assertTrue(all(p.name.startswith(stamp) or p.name == "waveform_data" for p in files))
            self.assertTrue((run / (stamp + "_waveforms.html")).is_file())
            report = json.loads((run / (stamp + "_summary.json")).read_text())
            self.assertEqual(report["settings"]["seed"], 0)
            with np.load(run / (stamp + "_waveforms.npz")) as data:
                self.assertIn("PWM333_binary", data.files)


@unittest.skipUnless(sys.platform == "win32" and shutil.which("clang"), "Production-C harness needs Windows clang/lld")
class ProductionCrossCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load the existing harness without editing its user-modified source.
        spec = importlib.util.spec_from_file_location("c_sampler_tests", Path(__file__).with_name("test_dshot_slow.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls.harness = module.SlowDshotTests
        cls.harness.setUpClass()

    @classmethod
    def tearDownClass(cls):
        cls.harness.tearDownClass()

    def test_python_matches_actual_c_sampler_and_decoder(self):
        word = self.harness.packet(1000)
        fixtures = [
            [0] * 5 + samples(word) * 2,
            [0] * 5 + samples(word, (1, 3), True) * 2,
            [0] * 5 + samples(word ^ 1) + [0] * 5 + samples(word),
            [0] * 5 + [1, 0, 1] + [0] * 5 + samples(word),
            [0] * 5 + [1] * 5 + [0] * 5 + samples(word),
            [0] * 5 + [1, 1, 0, 0, 0, 0, 0] + [0] * 5 + samples(word),
        ]
        for hz, library in self.harness.libraries:
            for fixture in fixtures:
                with self.subTest(hz=hz, fixture=fixture[:12]):
                    library.reset(1)
                    self.harness.feed(library, fixture)
                    events = receive(fixture, word)
                    self.assertEqual(library.good(), sum(e["status"] == "accepted_correct" for e in events))
                    self.assertEqual(library.bad(), sum(e["status"] == "rejected" for e in events))


if __name__ == "__main__":
    unittest.main()
