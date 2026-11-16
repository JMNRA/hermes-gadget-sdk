#!/usr/bin/env python3
"""Tab5 offline diagnostic's one-shot USB integrity exchange (not authentication).

No retries, seed files, logs of material, reset, or flashing. Requires an exclusive
raw POSIX tty and the explicitly selected offline firmware. --self-test never opens hardware.
A device challenge binds proof/ACK/RUNNING. A changed challenge rejects old ACKs,
but its entropy/uniqueness is NOT established: a repeated challenge permits full
replay after reset. An active peer can forge this public protocol. Fresh host
sessions reject historical replies; success is NOT certification of the device.
Drops that leave a complete valid transcript are undetectable; no lossless USB claim.
USB, OS, Python immutable/hash copies and core SHA internals are NOT proven secret
or securely erased. Mutable protocol buffers are wiped on success and failure.
"""
import argparse
import hashlib
import hmac
import os
import select
import time

DOMAIN = b"HG-TAB5-OFFLINE-V2"


def wipe(buf):
    buf[:] = b"\0" * len(buf)


def session(io, random=os.urandom, clock=time.monotonic):
    deadline = clock() + 60
    buffers = []

    def buffer(data):
        value = bytearray(data)
        buffers.append(value)
        return value

    def read(size):
        result = buffer(b"")
        while len(result) < size:
            if clock() >= deadline:
                raise TimeoutError("session deadline")
            part = io.read(size - len(result), deadline)
            if not part or len(part) > size - len(result):
                raise ValueError("truncated/oversized frame")
            result.extend(part)
        return result

    def send(data):
        if clock() >= deadline or io.write(data, deadline) != len(data):
            raise ValueError("short/late write; no retry")

    def wait_marker(marker):
        # ESP-IDF boot and peripheral logs share the USB TX channel. Discard
        # bounded preceding bytes without printing them; proof stays strict.
        window = buffer(b"")
        for _ in range(16384):
            if clock() >= deadline:
                raise TimeoutError("session deadline")
            part = io.read(1, deadline)
            if len(part) != 1:
                raise ValueError("truncated marker")
            window.extend(part)
            if len(window) > len(marker):
                del window[0]
            if window == marker:
                return
        raise ValueError("marker not found within limit")

    try:
        wait_marker(b"HGREADY3")
        challenge = read(16)
        # Fresh draws each invocation; there is no seed/session CLI override.
        sid, seed = buffer(random(16)), buffer(random(32))
        if len(sid) != 16 or len(seed) != 32:
            raise ValueError("random source length")
        material = buffer(DOMAIN)
        material.extend(sid)
        material.extend(seed)
        material.extend(challenge)
        digest = buffer(hashlib.sha256(material).digest())
        offer = buffer(b"HGSEED02")
        offer.extend(sid)
        offer.extend(seed)
        send(offer)
        expected = buffer(b"HGPROOF2")
        expected.extend(sid)
        expected.extend(digest)
        proof = read(56)
        if not hmac.compare_digest(proof, expected) or io.extra():
            raise ValueError("wrong session/proof or surplus data; ACK withheld")
        ack = buffer(b"HGACK002")
        ack.extend(sid)
        ack.extend(digest)
        send(ack)
        wait_marker(b"HGRUN002")
        running = read(48)
        if not hmac.compare_digest(running, expected[8:]):
            raise ValueError("wrong session/running proof")
        if clock() >= deadline:
            raise ValueError("late running marker")
        # Confirms this host's exchange only, not physical execution or secrecy.
    finally:
        for value in buffers:
            wipe(value)


class Tty:
    def __init__(self, fd):
        self.fd = fd

    def read(self, size, deadline):
        if not select.select([self.fd], [], [], max(0, deadline-time.monotonic()))[0]:
            raise TimeoutError("USB read deadline")
        return os.read(self.fd, size)

    def write(self, data, deadline):
        if not select.select([], [self.fd], [], max(0, deadline-time.monotonic()))[1]:
            raise TimeoutError("USB write deadline")
        return os.write(self.fd, data)

    def extra(self):
        return bool(select.select([self.fd], [], [], 0)[0])


def self_test():
    class Fake:
        def __init__(self, mode):
            self.mode = mode
            self.queue = bytearray((b"boot log\r\n" if mode == "logs" else b"") + b"HGREADY3" + bytes(16))
            self.writes = []
            self.refs = []
            self.acked = False

        def read(self, size, deadline):
            if self.mode == "drop" and self.writes:
                return b""
            n = min(size, len(self.queue), 3)  # deterministic fragmented ingress
            result = self.queue[:n]
            del self.queue[:n]
            return result

        def write(self, data, deadline):
            self.refs.append(data)
            self.writes.append(bytes(data))  # fixtures only
            if len(self.writes) == 1:
                sid = bytes(range(1,17)) if self.mode in ("fresh", "old-transcript") else bytes(range(16))
                assert data == b"HGSEED02" + sid + bytes(range(16,48))
                digest = bytes.fromhex("d43a4bc07f0220fcf0f0bf259dd579ee2342ba8d4a067696f9ffea49b2e24219")
                if self.mode == "fresh":
                    digest = hashlib.sha256(DOMAIN + data[8:] + bytes(16)).digest()
                proof_sid = bytes(range(16)) if self.mode == "old-transcript" else data[8:24]
                self.queue.extend(b"HGPROOF2" + proof_sid + digest)
                if self.mode == "interleaved-proof": self.queue[25:25] = b"log\r\n"
                if self.mode == "wrong": self.queue[-1] ^= 1
                if self.mode == "replay": self.queue[8] ^= 1
                if self.mode == "partial": del self.queue[-1:]
                if self.mode == "oversized": self.queue.extend(b"x")
                if self.mode == "short-write": return len(data)-1
            else:
                assert data[:8] == b"HGACK002"
                self.acked = True
                running = bytearray(b"HGRUN002" + data[8:])
                if self.mode == "stale-running": running[8] ^= 1
                if self.mode == "wrong-running": running[-1] ^= 1
                if self.mode == "interleaved-running": running[20:20] = b"log\r\n"
                if self.mode == "static-running": running = b"HGRUN002"
                self.queue.extend((b"display ready\r\n" if self.mode == "logs" else b"") + running)
            return len(data)

        def extra(self):
            return bool(self.queue)

    for mode in ("ok", "logs", "wrong", "replay", "partial", "oversized", "drop", "short-write", "timeout", "stale-running", "wrong-running", "interleaved-running", "static-running", "fresh", "old-transcript", "interleaved-proof"):
        io = Fake(mode)
        ticks = iter([0, 0, 61]) if mode == "timeout" else None
        try:
            sid = bytes(range(1,17)) if mode in ("fresh", "old-transcript") else bytes(range(16))
            session(io, random=lambda n: sid if n == 16 else bytes(range(16,48)),
                    clock=(lambda: next(ticks, 61)) if ticks else lambda: 0)
            assert mode in ("ok", "logs", "fresh")
        except (ValueError, TimeoutError):
            assert mode not in ("ok", "logs", "fresh")
        assert io.acked == (mode in ("ok", "logs", "fresh", "stale-running", "wrong-running", "interleaved-running", "static-running"))
        assert all(not any(ref) for ref in io.refs), "mutable buffers not wiped"
    print("16 deterministic host cases passed (including fragmented/logged and invalid RUNNING, withheld ACK and wipe)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--device", help="explicit POSIX USB tty; never auto-discovered")
    args = parser.parse_args()
    if args.self_test:
        if args.device: parser.error("self-test excludes hardware")
        self_test()
        return
    if not args.device: parser.error("--device required for the offline diagnostic")
    import fcntl
    import termios
    import tty
    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old = None
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        old = termios.tcgetattr(fd)
        tty.setraw(fd, termios.TCSANOW)  # do not flush away evidence of stale bytes
        session(Tty(fd))
        print("Fresh host session matched; running marker received. Not hardware/security proof.")
    finally:
        if old is not None: termios.tcsetattr(fd, termios.TCSANOW, old)
        os.close(fd)


if __name__ == "__main__":
    main()
