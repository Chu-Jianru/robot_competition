import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bench_model import BenchData, duration_ms, load_settings, save_settings


class DataTests(unittest.TestCase):
    def test_four_channels_keep_rates_calibration_and_samples_separate(self):
        data = BenchData()
        for index, channel in enumerate("ABCD"):
            data.channels[channel].ppr = (index + 1) * 100
            data.receive(f"ENC {channel} ticks=0 invalid=0 A=0 B=0", 0)
            data.receive(f"ENC {channel} ticks={(index+1)*100} invalid=0 A=1 B=0", 1)
        for index, channel in enumerate("ABCD"):
            data.select_channel(channel)
            self.assertEqual(data.latest.channel, channel)
            self.assertEqual(data.latest.counts_s, (index+1)*100)
            self.assertEqual(data.latest.rpm, 60)
        data.receive("ZEROED B", 2)
        data.receive("ENC B ticks=0 invalid=0 A=0 B=0", 2)
        self.assertIsNone(data.channels["B"].latest.counts_s)
        self.assertEqual(data.channels["A"].latest.ticks, 100)
        data.disconnect(3)
        self.assertTrue(all(state.latest is None for state in data.channels.values()))

    def test_final_record_waits_for_its_own_channel_not_first_encoder_line(self):
        data = BenchData()
        data.receive("ENC D ticks=20 invalid=0 A=0 B=0", 0)
        data.receive("JOG D -50 percent 1000 ms; automatic stop enabled", 1)
        data.receive("STOPPED timer", 2)
        for channel in "ABC":
            data.receive(f"ENC {channel} ticks=999 invalid=4 A=0 B=0", 2.01)
        self.assertFalse(data.runs)
        data.receive("ENC D ticks=-100 invalid=1 A=0 B=0", 2.02)
        self.assertEqual((data.runs[-1].channel, data.runs[-1].ticks_delta), ("D", -120))

    def test_per_channel_settings_migrate_legacy_calibration_only_to_a(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "settings.json"
            save_settings(path, dict(duty="50", duration="1", ppr="200"))
            self.assertEqual(load_settings(path)["ppr_by_channel"], {"A": "200"})
            save_settings(path, dict(duty="50", duration="1", ppr_by_channel={"B": "400", "C": "NaN", "D": "-1"}))
            self.assertEqual(load_settings(path)["ppr_by_channel"], {"B": "400"})
    def test_duration_units_and_precision(self):
        self.assertEqual(duration_ms("2.5", "秒"), 2500)
        self.assertEqual(duration_ms("0.001", "秒"), 1)
        self.assertEqual(duration_ms("2147483647", "毫秒"), 2147483647)
        for value in ("0", "-1", "NaN", "Infinity", "0.0001", "2147484"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                duration_ms(value, "秒")

    def test_counts_rate_and_calibration(self):
        data = BenchData()
        data.receive("ENC ticks=0 invalid=0 A=0 B=0", 0)
        self.assertIsNone(data.latest.rpm)
        data.receive("ENC ticks=100 invalid=0 A=1 B=0", 1)
        self.assertEqual(data.latest.counts_s, 100)
        data.ppr = 200
        data.receive("ENC ticks=200 invalid=0 A=0 B=0", 2)
        self.assertEqual(data.latest.rpm, 30)
        data.receive("ENC ticks=190 invalid=0 A=0 B=0", 2.01)
        data.receive("ENC ticks=100 invalid=0 A=0 B=0", 3)
        self.assertEqual(data.latest.counts_s, -100)

    def test_reset_and_zero_do_not_create_rate_spikes(self):
        data = BenchData()
        data.receive("ENC ticks=10000 invalid=2 A=0 B=0", 1)
        data.clear_baseline(zero=True)
        data.receive("ENC ticks=10000 invalid=2 A=0 B=0", 1.2)
        data.receive("ENC ticks=0 invalid=0 A=0 B=0", 1.5)
        self.assertIsNone(data.latest.counts_s)
        data.receive("ENC ticks=50 invalid=0 A=0 B=0", 2)
        self.assertEqual(data.latest.counts_s, 100)
        data.disconnect(3)
        self.assertIsNone(data.latest)
        data.receive("ENC ticks=500 invalid=0 A=0 B=0", 4)
        self.assertIsNone(data.latest.counts_s)

    def test_run_waits_for_device_ack_and_final_encoder(self):
        data = BenchData()
        data.log("tx", "JOG 50 5000")
        self.assertIsNone(data.active)
        data.receive("ENC ticks=10 invalid=0 A=0 B=0", 0)
        data.receive("JOG A 50 percent 5000 ms; automatic stop enabled", 1)
        data.receive("ENC ticks=50 invalid=0 A=0 B=0", 2)
        data.receive("STOPPED timer", 6)
        self.assertFalse(data.runs)
        data.receive("ENC ticks=100 invalid=1 A=0 B=1", 6.01)
        run = data.runs[-1]
        self.assertEqual((run.ticks_delta, run.invalid_delta, run.observed_s), (90, 1, 5))
        self.assertTrue(run.final_sample)

    def test_interrupted_or_missing_final_sample_is_explicit(self):
        for interrupted in (False, True):
            data = BenchData()
            data.receive("JOG A -50 percent 5000 ms; automatic stop enabled", 1)
            if interrupted:
                data.disconnect(2)
            else:
                data.receive("STOPPED command", 2)
                data.tick(3.1)
            self.assertFalse(data.runs[-1].final_sample)
            self.assertIsNone(data.runs[-1].ticks_delta)

    def test_bounded_samples_and_export_preserve_observations(self):
        data = BenchData(limit=2)
        for index in range(3):
            data.receive(f"ENC ticks={index} invalid=0 A=0 B=0", index)
        self.assertEqual(data.dropped, 1)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "samples.csv"
            data.export_csv(path)
            with path.open(encoding="utf-8-sig", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([row["ticks"] for row in rows], ["1", "2"])
            self.assertEqual(rows[0]["rpm"], "")

    def test_settings_corruption_and_atomic_save(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "settings.json"
            path.write_text("broken", encoding="utf-8")
            self.assertEqual(load_settings(path)["duty"], "50")
            settings = dict(duty="60", duration="2.5", unit="秒", ppr="200", history=["STOP", "ENC\nJOG 50 5000"])
            save_settings(path, settings)
            loaded = load_settings(path)
            self.assertEqual(loaded["duration"], "2.5")
            self.assertEqual(loaded["history"], ["STOP"])
            self.assertFalse(path.with_suffix(".tmp").exists())


if __name__ == "__main__":
    unittest.main()
