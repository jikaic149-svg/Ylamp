"""Offline firmware-binding regression; never opens a port or probe."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import five_happy_motion as motion


class FirmwareBindingTest(unittest.TestCase):
    def test_other_firmware_rejected_before_layout_or_hardware(self):
        with patch.object(Path, "read_text", return_value=json.dumps(
                {"firmware_sha256": "0" * 64})), patch.object(
                motion, "motion_layout",
                side_effect=AssertionError("unverified plan reached I/O")):
            with self.assertRaisesRegex(ValueError, "another firmware"):
                motion.execute(Path("unused-mismatched-plan"))

    def test_preflight_reads_the_entire_declared_image_not_old_fixed_length(self):
        class StoppedBeforeHardware(Exception):
            pass
        binding=dict(firmware_sha256="digest",elf_sha256="digest",
                     control_address=0x20002ae8,control_size=4808)
        with patch.object(Path,"read_text",side_effect=[
                json.dumps(binding),json.dumps({"source_sha256":{}})]), \
                patch.object(motion,"EXPECTED","digest"), \
                patch.object(motion,"sha",return_value="digest"), \
                patch.object(Path,"stat",return_value=SimpleNamespace(st_size=179452)), \
                patch.object(motion.subprocess,"check_output",
                             return_value="20002ae8 000012c8 b c\n"), \
                patch.object(motion.subprocess,"run") as probe:
            def check(command,**_):
                dump=next(item for item in command if item.startswith("dump_image "))
                self.assertTrue(dump.endswith(" 179452"))
                raise StoppedBeforeHardware()
            probe.side_effect=check
            with self.assertRaises(StoppedBeforeHardware):
                motion.preflight(Path("unused-output"))


if __name__ == "__main__":
    unittest.main()
