"""Local DSU client test: send discovery and subscription, count real UDP frames."""
import argparse
import socket
import struct
import time
from ally_dsu import VERSION_MSG, INFO_MSG, DATA_MSG, packet, parse_packet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=26760)
    ap.add_argument('--seconds', type=float, default=5)
    args = ap.parse_args()
    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0))
    sock.settimeout(.1)
    client_id = 0x1234abcd
    send = lambda m, payload=b'': sock.sendto(packet(b'DSUC', client_id, m, payload), addr)
    send(VERSION_MSG)
    send(INFO_MSG, struct.pack('<i4B', 4, 0, 1, 2, 3))
    send(DATA_MSG, struct.pack('<BB6s', 1, 0, b'\x00'*6))
    start = time.monotonic()
    next_renew = start + 1
    counts = {VERSION_MSG: 0, INFO_MSG: 0, DATA_MSG: 0}
    first = None
    last = None
    while time.monotonic() - start < args.seconds:
        if time.monotonic() > next_renew:
            send(DATA_MSG, struct.pack('<BB6s', 1, 0, b'\x00'*6))
            next_renew = time.monotonic() + 1
        try:
            data, source = sock.recvfrom(2048)
        except socket.timeout:
            continue
        parsed = parse_packet(data, expect_magic=b'DSUS')
        if not parsed:
            print('Bad packet from', source)
            continue
        _, kind, payload = parsed
        counts[kind] = counts.get(kind, 0) + 1
        if kind == INFO_MSG:
            print('Controller info: slot', payload[0], 'state', payload[1],
                  'model', payload[2], 'MAC', payload[4:10].hex(':'))
        if kind == DATA_MSG:
            seq = struct.unpack_from('<I', payload, 12)[0]
            vals = struct.unpack_from('<6f', payload, 56)
            if first is None:
                first = seq
            last = (seq, vals)
    sock.close()
    print('Version responses:', counts[VERSION_MSG])
    print('Info responses:', counts[INFO_MSG])
    print('Motion frames:', counts[DATA_MSG],
          'average Hz:', round(counts[DATA_MSG] / max(args.seconds, .001), 1))
    if last:
        print('Sequence first/last:', first, last[0])
        print('Last accel xyz (g), gyro pitch/yaw/roll (dps):',
              tuple(round(v, 3) for v in last[1]))


if __name__ == '__main__':
    main()
