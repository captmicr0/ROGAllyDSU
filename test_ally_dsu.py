"""Protocol and UDP integration tests; run with `python -m unittest -v`."""
import socket
import struct
import threading
import time
import unittest

from ally_dsu import (DATA_MSG, INFO_MSG, VERSION, VERSION_MSG, DSUServer,
                      Motion, DemoSensors, MAC, info_packet, motion_packet,
                      packet, parse_packet)


class ProtocolTests(unittest.TestCase):
    def test_packet_sizes_and_crc(self):
        p = packet(b'DSUS', 123, VERSION_MSG, struct.pack('<H', VERSION))
        self.assertEqual(len(p), 22)
        self.assertEqual(struct.unpack_from('<H', p, 6)[0], 6)
        self.assertEqual(parse_packet(p, expect_magic=b'DSUS')[1], VERSION_MSG)
        info = info_packet(123, 0)
        self.assertEqual(len(info), 32)
        self.assertEqual(struct.unpack_from('<H', info, 6)[0], 16)
        self.assertEqual(parse_packet(info, expect_magic=b'DSUS')[1], INFO_MSG)
        motion = motion_packet(123, 10, 123456789, Motion(1, 2, 3, 4, 5, 6))
        self.assertEqual(len(motion), 100)
        self.assertEqual(struct.unpack_from('<H', motion, 6)[0], 84)
        self.assertEqual(parse_packet(motion, expect_magic=b'DSUS')[1], DATA_MSG)
        self.assertEqual(struct.unpack_from('<I', motion, 32)[0], 10)
        self.assertEqual(struct.unpack_from('<Q', motion, 68)[0], 123456789)
        self.assertEqual(struct.unpack_from('<6f', motion, 76), (1, 2, 3, 4, 5, 6))
        self.assertEqual(motion[24:30], MAC)

    def test_invalid_packet(self):
        p = bytearray(packet(b'DSUC', 123, VERSION_MSG))
        p[-1] ^= 1
        self.assertIsNone(parse_packet(p))
        self.assertIsNone(parse_packet(p[:16]))
        good = packet(b'DSUC', 123, VERSION_MSG)
        self.assertIsNone(parse_packet(good[:-1]))
        self.assertIsNone(parse_packet(good + b'x', expect_magic=b'DSUS'))

    def test_udp_discovery_and_streaming(self):
        server = DSUServer(DemoSensors(), host='127.0.0.1', port=0,
                           send_hz=100, timeout=.5, status_interval=10)
        thread = threading.Thread(target=server.serve, daemon=True)
        thread.start()
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.bind(('127.0.0.1', 0))
        client.settimeout(1)
        try:
            dst = server.address
            client.sendto(packet(b'DSUC', 123, VERSION_MSG), dst)
            data, _ = client.recvfrom(2048)
            self.assertEqual(parse_packet(data, expect_magic=b'DSUS')[1], VERSION_MSG)
            self.assertEqual(len(data), 22)
            client.sendto(packet(b'DSUC', 123, INFO_MSG,
                                 struct.pack('<i4B', 4, 0, 1, 2, 3)), dst)
            infos = [client.recvfrom(2048)[0] for _ in range(4)]
            self.assertEqual([p[20] for p in infos], [0, 1, 2, 3])
            self.assertEqual([p[21] for p in infos], [2, 0, 0, 0])
            # MAC registration and slot registration both work.
            client.sendto(packet(b'DSUC', 123, DATA_MSG,
                                 struct.pack('<BB6s', 3, 0, MAC)), dst)
            frames = [client.recvfrom(2048)[0] for _ in range(8)]
            self.assertTrue(all(len(p) == 100 and parse_packet(p, expect_magic=b'DSUS')
                                for p in frames))
            sequences = [struct.unpack_from('<I', p, 32)[0] for p in frames]
            self.assertTrue(all(b > a for a, b in zip(sequences, sequences[1:])))
            # Ensure clients are not retained forever.
            time.sleep(.7)
            self.assertFalse(server.clients)
        finally:
            client.close()
            server.stop()
            thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
