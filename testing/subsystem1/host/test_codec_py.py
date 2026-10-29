"""Cross-check: the Python decoder must agree with the C encoder (vectors below
were produced by testing/subsystem1/host/test_codec.c and are asserted there)."""
import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "comms"))
from comms_codec import decode_status, decode_fault, encode_cmd, encode_status


class CrossCheck(unittest.TestCase):
    def test_status_vector_from_c(self):
        # C: {seq 5, up 312, rssi 61, mode LINE(2), faults 0x040 (LINE), drop 0}
        s = decode_status(bytes.fromhex("85 13 00 3D 01 02"))
        self.assertEqual((s["seq"], s["up_s"], s["rssi_dbm"], s["mode"], s["faults"], s["tx_drop"]),
                         (5, 312, -61, "LINE", 0x040, False))
        self.assertEqual(s["fault_names"], ["LINE"])

    def test_status_roundtrip_all_fields(self):
        for seq in (0, 15):
            for up in (0, 1, 0xFFFFF):
                for mode in range(9):
                    for faults in (0, 0xFFF, 0xA5A):
                        raw = encode_status(seq, up, 127, mode, faults, True)
                        s = decode_status(raw)
                        self.assertEqual((s["seq"], s["up_s"], s["faults"], s["tx_drop"]), (seq, up, faults, True))

    def test_unknown_rssi(self):
        self.assertIsNone(decode_status(encode_status(0, 0, 0, 0, 0))["rssi_dbm"])

    def test_bad_mode_rejected(self):
        with self.assertRaises(ValueError):
            decode_status(encode_status(0, 0, 0, 9, 0))

    def test_fault_vector_from_c(self):
        # C (S1-H12): {t 13550, raised AVOID|NO_ECHO} prints "EE 34 88 00"
        f = decode_fault(bytes.fromhex("EE 34 88 00"))
        self.assertEqual((f["t_ms"], f["raised"], f["raised_names"]), (13550, 0x088, ["NO_ECHO", "AVOID"]))

    def test_cmd_vector_from_c(self):
        self.assertEqual(encode_cmd("START", 7), bytes([0x39]))      # C S1-H14
        self.assertEqual(encode_cmd("ping", 31), bytes([(31 << 3) | 5]))

    def test_wrong_length(self):
        with self.assertRaises(ValueError):
            decode_status(b"\x00" * 5)
        with self.assertRaises(ValueError):
            decode_fault(b"\x00" * 3)


if __name__ == "__main__":
    unittest.main()
