"""Run: python -m unittest discover -s tests -p test_motor_noise.py -v"""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import motor_noise as motor
import noise_sim as sim


class MotorNoiseTests(unittest.TestCase):
    def profile(self):
        p = deepcopy(motor.DEFAULT_PROFILE)
        p["startup"].update(at_s=.001, peak_multiplier=1.)
        p["motor"].update(electrical_hz=0., modulation_depth=0.)
        p["ground_shift"].update(startup_v=0., steady_v=0.)
        return p

    def generate(self, profile, seed=42, duration_ns=5_000_000):
        times = np.arange(0, duration_ns, 100, dtype=np.int64)
        return motor.generate_motor_noise(times, 100, profile, seed, np.zeros(len(times)))

    def test_bundled_profiles_and_invalid_custom_fields(self):
        names = [p.stem for p in motor.PROFILE_DIR.glob("*.json")]
        self.assertEqual(len(names), 6)
        for name in names:
            profile = motor.load_profile(name)
            motor.validate_profile(profile, profile["simulation"]["step_us"])
            self.assertTrue(profile["description"])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "custom.json"
            path.write_text(json.dumps({"ground_shift": {"startup_v": -.5}}))
            p = motor.load_profile(path)
            self.assertEqual(p["ground_shift"]["startup_v"], -.5)
            self.assertEqual(p["switching"], motor.DEFAULT_PROFILE["switching"])
            path.write_text('{"switching": {"freq_typo": 200}}')
            with self.assertRaises(ValueError):
                motor.load_profile(path)
            path.write_text('[]')
            with self.assertRaises(ValueError):
                motor.load_profile(path)

    def test_startup_and_ground_exponential_decay(self):
        profile = self.profile()
        profile["startup"].update(peak_multiplier=4., decay_ms=50.)
        profile["ground_shift"].update(startup_v=.8, steady_v=.05, decay_ms=20.)
        age = np.array([-.001, 0, .05])
        np.testing.assert_allclose(motor.startup_envelope(age, profile), [0, 4, 1 + 3 / np.e])
        ground = motor.ground_displacement(np.array([-.001, 0, .02]), profile)
        np.testing.assert_allclose(ground, [0, .8, .05 + .75 / np.e])

    def test_integrates_accelerating_electrical_frequency(self):
        profile = self.profile()
        profile["motor"].update(electrical_hz=1000, spinup_s=.5)
        phase = motor.electrical_phase(np.array([0, .25, .5, 1.]), profile)
        np.testing.assert_allclose(phase / (2 * np.pi), [0, 62.5, 250, 750])
        phase_shifted = motor.electrical_phase(np.array([0, .5]), profile, .7)
        np.testing.assert_allclose(phase_shifted, [.7, .7 + 2 * np.pi * 250])
        profile["motor"]["spinup_s"] = 0
        self.assertAlmostEqual(motor.electrical_phase(np.array([.1]), profile)[0], 2 * np.pi * 100)

    def test_duty_event_spacing_and_polarity(self):
        profile = self.profile()
        profile["switching"].update(pwm_hz=10000, duty=.25)
        times, signs = motor.switching_events(1_500_000, profile, .1)
        np.testing.assert_array_equal(times[:4], [1_010_000, 1_035_000, 1_110_000, 1_135_000])
        np.testing.assert_array_equal(signs[:4], [1, -1, 1, -1])
        self.assertTrue(np.all(times >= 1_000_000))
        profile["switching"]["ring_peak_v"] = 0
        _, components, _, _ = self.generate(profile)
        self.assertGreater(float(components["switching_spikes_v"].max()), 0)
        self.assertLess(float(components["switching_spikes_v"].min()), 0)

    def test_triangular_peak_and_following_ring(self):
        profile = self.profile()
        profile["switching"].update(spike_peak_v=2., spike_width_us=1.,
                                    ring_peak_v=1., ring_hz=200000, ring_decay_us=3.)
        ages = np.array([-.1, 0, .5, 1., 2.25])
        spike, ring = motor.event_kernel(ages, profile)
        np.testing.assert_allclose(spike, [0, 0, 2, 0, 0])
        np.testing.assert_allclose(ring[:4], 0)
        self.assertAlmostEqual(ring[4], np.exp(-1.25 / 3))

    def test_disabled_components_and_signed_ground(self):
        profile = self.profile()
        profile["switching"].update(spike_peak_v=0, ring_peak_v=0, pwm_hz=0)
        combined, components, _, _ = self.generate(profile)
        self.assertFalse(combined.any())
        self.assertTrue(all(not c.any() for c in components.values()))
        profile["ground_shift"].update(startup_v=.5, steady_v=.5)
        positive, _, _, _ = self.generate(profile)
        self.assertEqual(float(positive[10000]), -.5)
        self.assertFalse(positive[:10000].any())
        profile["ground_shift"].update(startup_v=-.5, steady_v=-.5)
        negative, _, _, _ = self.generate(profile)
        np.testing.assert_array_equal(negative, -positive)

    def test_overlapping_ringing_adds_event_tails(self):
        profile = self.profile()
        profile["switching"].update(pwm_hz=10000, duty=.5, spike_peak_v=0,
                                    ring_peak_v=1., ring_decay_us=100.)
        combined, _, _, info = self.generate(profile, duration_ns=1_300_000)
        times, signs = motor.switching_events(1_300_000, profile, info["switching_phase_fraction"])
        sample_ns = int(times[2] // 100 * 100 + 1300)
        ages = (sample_ns - times) / 1000
        _, ring = motor.event_kernel(ages, profile)
        expected = float(np.sum(ring * signs))
        self.assertAlmostEqual(float(combined[sample_ns // 100]), expected, places=6)

    def test_resolution_and_invalid_numbers(self):
        profile = self.profile()
        with self.assertRaisesRegex(ValueError, "step <= 0.25"):
            motor.validate_profile(profile, 1)
        motor.validate_profile(profile, .1)
        profile["switching"].update(spike_peak_v=0, ring_hz=2_000_000)
        with self.assertRaisesRegex(ValueError, "100 ns"):
            motor.validate_profile(profile, .1)
        for field, value in (("duty", 1), ("spike_peak_v", -1), ("ring_hz", float("nan"))):
            profile = self.profile()
            profile["switching"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                motor.validate_profile(profile)

    def test_reproducibility_and_amplitudes_do_not_move_events(self):
        profile = self.profile()
        a, _, _, info = self.generate(profile)
        b, _, _, _ = self.generate(profile)
        np.testing.assert_array_equal(a, b)
        profile["switching"]["spike_peak_v"] *= 2
        profile["switching"]["ring_peak_v"] *= 2
        c, _, _, other = self.generate(profile)
        self.assertEqual(info["switching_phase_fraction"], other["switching_phase_fraction"])
        self.assertEqual(info["electrical_initial_phase_rad"], other["electrical_initial_phase_rad"])
        np.testing.assert_allclose(c, a * 2, atol=1e-6)

    def test_cli_defaults_overrides_and_effective_profile(self):
        _, _, settings, profile = sim.parse_options(["--profile", "weapon-startup", "--seed", "42"])
        self.assertEqual(settings.step_us, .1)
        self.assertEqual(settings.noise_rms_v, .1)
        _, _, settings, profile = sim.parse_options(["--profile", "weapon-startup", "--step-us", ".2",
                                                    "--noise-rms-v", ".3", "--duration-s", ".025", "--seed", "42"])
        self.assertEqual(settings.step_us, .2)
        self.assertEqual(settings.noise_rms_v, .3)
        summary, _, _, arrays = sim.simulate(settings, True, profile)
        self.assertEqual(summary["effective_profile"]["background_rms_v"], .3)
        self.assertEqual(summary["effective_profile"]["simulation"]["step_us"], .2)
        self.assertIn("switching_ringing_v", arrays)
        self.assertEqual(summary["disturbance"]["switching_event_count"], 0)
        _, _, defaults, profile = sim.parse_options(["--seed", "42"])
        self.assertIsNone(profile)
        self.assertEqual(defaults, sim.Settings(seed=42))

    def test_same_disturbance_reused_across_methods(self):
        profile = self.profile()
        settings = sim.Settings(duration_s=.025, step_us=.1, noise_rms_v=0, seed=42)
        summary, _, _, arrays = sim.simulate(settings, True, profile)
        for method in sim.METHODS:
            np.testing.assert_allclose(arrays[f"{method}_noisy_v"] - arrays[f"{method}_clean_v"],
                                       arrays["noise_v"], atol=1e-12)
        self.assertEqual(len(summary["reports"]), 5)

    def test_profile_output_and_noise_free_reception(self):
        profile = self.profile()
        profile["switching"].update(spike_peak_v=0, ring_peak_v=0)
        settings = sim.Settings(duration_s=.025, step_us=.1, noise_rms_v=0, seed=42)
        summary, events, plots, arrays = sim.simulate(settings, True, profile)
        self.assertTrue(all(r["failed_updates"] == 0 for r in summary["reports"]))
        with tempfile.TemporaryDirectory() as directory:
            run = sim.save_outputs(Path(directory), "test", summary, events, plots, arrays)
            self.assertTrue((run / "test_disturbance.png").is_file())
            saved = json.loads((run / "test_summary.json").read_text())
            self.assertIn("effective_profile", saved)
            self.assertIn("ground_shift_v", saved["disturbance"]["components"])

    def test_white_profile_preserves_existing_gaussian_results(self):
        settings = sim.Settings(duration_s=.025, seed=42)
        old, _, _, old_arrays = sim.simulate(settings, True)
        new, _, _, new_arrays = sim.simulate(settings, True, motor.load_profile("white-baseline"))
        np.testing.assert_array_equal(old_arrays["noise_v"], new_arrays["noise_v"])
        self.assertEqual(old["reports"], new["reports"])

    def test_plot_preserves_fast_modulation_extrema(self):
        profile = self.profile()
        profile["motor"].update(electrical_hz=10000, spinup_s=0., modulation_depth=.5)
        _, _, preview, _ = self.generate(profile, duration_ns=500_000_000)
        active = (preview["time_s"] > .002) & (preview["time_s"] < .49)
        np.testing.assert_allclose(preview["electrical_modulation_min"][active], 1.)
        np.testing.assert_allclose(preview["electrical_modulation_max"][active], 1.5)


if __name__ == "__main__":
    unittest.main()
