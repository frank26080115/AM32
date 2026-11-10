"""Validate lossless archives and the compact offline viewer data format."""
import base64
import gzip
import json
from pathlib import Path
import re
import tempfile
import unittest

import numpy as np

import noise_sim as sim
import waveform_html as viewer


class WaveformViewerTests(unittest.TestCase):
    def test_envelope_preserves_extrema_and_partial_last_bin(self):
        values = np.array([0, 100, 0, -50, 1, 2, 3, 4, 9])
        np.testing.assert_array_equal(viewer.envelope(values, 4, 0, 3),
                                      [[-50, 100], [1, 4], [9, 9]])
        np.testing.assert_array_equal(viewer.envelope(values, 4, 1, 2), [[1, 4], [9, 9]])

    def test_export_chunks_cover_full_duration_and_decode(self):
        summary, _, _, arrays = sim.simulate(sim.Settings(duration_s=.025, noise_rms_v=0), True)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "waveforms.html"
            manifest = viewer.export_viewer(path, summary, arrays)
            self.assertEqual(manifest["count"], len(arrays["time_ns"]))
            self.assertEqual(manifest["duration_s"], .025)
            self.assertTrue((Path(directory) / "waveform_data/plotly.min.js").is_file())
            self.assertIn("scrollZoom:true", path.read_text(encoding="utf-8"))
            self.assertIn("fixedrange:true", path.read_text(encoding="utf-8"))
            self.assertLess(manifest["voltage_range"][0], float(arrays["noise_v"].min()))
            self.assertGreater(manifest["voltage_range"][1], 3.3)
            for method in sim.METHODS:
                for factor in manifest["factors"]:
                    chunks = manifest["methods"][method][str(factor)]
                    self.assertEqual(sum(c["count"] for c in chunks),
                                     (manifest["count"] + factor - 1) // factor)
                chunk = manifest["methods"][method]["1"][0]
                encoded = re.search(r",'([^']+)',", (Path(directory) / chunk["file"]).read_text()).group(1)
                packed = gzip.decompress(base64.b64decode(encoded))
                count = chunk["count"]
                voltages = np.frombuffer(packed, dtype="<f4", count=count * 2)
                states = np.frombuffer(packed, dtype=np.uint8, offset=count * 8)
                np.testing.assert_array_equal(voltages[:count], arrays[f"{method}_noisy_v"].astype(np.float32))
                np.testing.assert_array_equal(voltages[count:], arrays[f"{method}_filtered_v"].astype(np.float32))
                np.testing.assert_array_equal(states & 1, arrays[f"{method}_clean_v"] > 0)
                np.testing.assert_array_equal((states >> 1) & 1, arrays[f"{method}_binary"])

    def test_two_runs_have_separate_folders_and_binary_archives(self):
        summary, events, plots, arrays = sim.simulate(sim.Settings(duration_s=.025, noise_rms_v=0), True)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for stamp in ("run1", "run2"):
                run = sim.save_outputs(root, stamp, summary, events, plots, arrays)
                self.assertEqual(run, root / stamp)
                with np.load(run / (stamp + "_waveforms.npz")) as archive:
                    for key, value in arrays.items():
                        np.testing.assert_array_equal(archive[key], value)
            self.assertEqual(sorted(p.name for p in root.iterdir()), ["run1", "run2"])
            with self.assertRaises(FileExistsError):
                sim.save_outputs(root, "run1", summary, events, plots, arrays)

    def test_binary_export_does_not_silently_omit_arrays(self):
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(ValueError, "Full waveforms"):
            sim.save_outputs(Path(directory), "run", {}, [], {}, {})


if __name__ == "__main__":
    unittest.main()
