"""
Tests for the binary wire protocol between the sketch and the host.

Covers header parsing, frame decoding, checksum rejection, resynchronization
after corruption, and dropped-frame accounting. No hardware required.

Run with:  python test_stream.py
"""

from stream import StreamDecoder, encode_frame

HEADER = b"#TMAG5273 v=1 variant=x2 range_xy=266.0 range_z=266.0 addr=0x35\n"

failures = []


def check(name, condition, detail=""):
    if condition:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}   {detail}")
        failures.append(name)


def counts_to_ut(counts, range_mt=266.0):
    return counts / 32768.0 * range_mt * 1000.0


def primed_decoder():
    decoder = StreamDecoder()
    list(decoder.feed(HEADER))
    return decoder


def test_header():
    print("\nheader")
    decoder = StreamDecoder()
    check("not ready before the header arrives", not decoder.ready)

    list(decoder.feed(HEADER))
    check("ready once the header is consumed", decoder.ready)
    check("variant parsed", decoder.header.get("variant") == "x2")
    check("xy range parsed", decoder.range_xy == 266.0, f"got {decoder.range_xy}")
    check("z range parsed", decoder.range_z == 266.0, f"got {decoder.range_z}")


def test_header_split_across_reads():
    print("\nheader split across chunk boundaries")
    decoder = StreamDecoder()
    list(decoder.feed(HEADER[:20]))
    check("still waiting on a partial header", not decoder.ready)
    list(decoder.feed(HEADER[20:]))
    check("completes when the rest arrives", decoder.ready)


def test_error_line():
    print("\nsketch error line")
    decoder = StreamDecoder()
    list(decoder.feed(b"ERR: TMAG5273 not found - check wiring/address\n"))
    check("error surfaced", decoder.error_line is not None, f"got {decoder.error_line}")
    check("stays un-ready after an error", not decoder.ready)


def test_roundtrip():
    print("\nframe round-trip")
    decoder = primed_decoder()
    samples = [(100, -200, 5420), (0, 0, 0), (-32768, 32767, -1)]

    out = []
    for i, (x, y, z) in enumerate(samples):
        out.extend(decoder.feed(encode_frame(i, x, y, z)))

    check("decoded every frame", len(out) == len(samples), f"got {len(out)}")
    for (raw, got) in zip(samples, out):
        expected = (
            counts_to_ut(raw[0]), counts_to_ut(raw[1]), counts_to_ut(raw[2]),
        )
        close = all(abs(a - b) < 1e-9 for a, b in zip(expected, got))
        check(f"counts {raw} convert correctly", close, f"got {got}, want {expected}")

    check("no drops recorded", decoder.frames_dropped == 0)
    check("no checksum errors", decoder.checksum_errors == 0)


def test_byte_at_a_time():
    print("\nbyte-at-a-time delivery")
    decoder = primed_decoder()
    frame = encode_frame(0, 1234, -1234, 4321)

    out = []
    for byte in frame:
        out.extend(decoder.feed(bytes([byte])))

    check("frame emerges once its last byte lands", len(out) == 1, f"got {len(out)}")


def test_bad_checksum_rejected():
    print("\ncorruption handling")
    decoder = primed_decoder()
    frame = bytearray(encode_frame(0, 500, 600, 700))
    frame[9] ^= 0xFF  # break the checksum

    out = list(decoder.feed(bytes(frame)))
    check("corrupt frame not emitted", len(out) == 0, f"got {out}")
    check("checksum error counted", decoder.checksum_errors >= 1)


def test_resync_after_garbage():
    print("\nresynchronization")
    decoder = primed_decoder()
    good = encode_frame(7, 111, 222, 333)

    out = list(decoder.feed(b"\x00\xFF\xAA\x12garbage" + good))
    check("recovers the good frame after junk", len(out) == 1, f"got {len(out)}")
    if out:
        check(
            "recovered values are correct",
            abs(out[0][0] - counts_to_ut(111)) < 1e-9,
            f"got {out[0]}",
        )


def test_drop_accounting():
    print("\ndropped-frame accounting")
    decoder = primed_decoder()
    list(decoder.feed(encode_frame(10, 1, 2, 3)))
    list(decoder.feed(encode_frame(14, 4, 5, 6)))  # 11,12,13 went missing
    check("counts the gap", decoder.frames_dropped == 3, f"got {decoder.frames_dropped}")

    decoder2 = primed_decoder()
    list(decoder2.feed(encode_frame(254, 1, 2, 3)))
    list(decoder2.feed(encode_frame(0, 4, 5, 6)))  # wraps 254 -> 255 -> 0
    check(
        "handles sequence wrap without false drops",
        decoder2.frames_dropped == 1,
        f"got {decoder2.frames_dropped}",
    )


def test_sync_pattern_inside_payload():
    print("\nsync pattern appearing in payload")
    decoder = primed_decoder()
    # 0xAA55 as a little-endian int16 is 0x55AA = 21930; place it in the data.
    frame = encode_frame(3, 21930, 21930, 21930)
    out = list(decoder.feed(frame))
    check("payload that looks like sync still decodes", len(out) == 1, f"got {len(out)}")


def main():
    print("binary stream protocol tests")
    test_header()
    test_header_split_across_reads()
    test_error_line()
    test_roundtrip()
    test_byte_at_a_time()
    test_bad_checksum_rejected()
    test_resync_after_garbage()
    test_drop_accounting()
    test_sync_pattern_inside_payload()

    print()
    if failures:
        print(f"{len(failures)} FAILED: {', '.join(failures)}")
        raise SystemExit(1)
    print("all tests passed")


if __name__ == "__main__":
    main()
