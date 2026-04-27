#!/usr/bin/env python3
"""Generate short notification chimes as 16-bit signed mono WAV files.

Outputs are written to the directory given as argv[1]. Two clips:
    pairing.wav  — two-note ascending bell (Apple-ish "pairing mode" feel)
    success.wav  — quick three-note rising arpeggio ("ding")

Usage: python3 chime_gen.py <out-dir>
"""
import math
import struct
import sys
import wave
from pathlib import Path

RATE = 16000


def synth(notes, total_dur_s, amplitude=0.5):
    """Render a list of (start_s, dur_s, freq_hz) notes as int16 samples.
    Each note has an exponential decay envelope for a bell-ish tail."""
    n = int(total_dur_s * RATE)
    buf = [0.0] * n
    for start, dur, freq in notes:
        i0 = int(start * RATE)
        n_samp = int(dur * RATE)
        decay_tau = dur * 0.35  # decay time-constant in seconds
        for k in range(n_samp):
            if i0 + k >= n:
                break
            t = k / RATE
            env = math.exp(-t / decay_tau)
            # quick linear attack ramp on the first 5 ms to avoid clicks
            attack = min(t / 0.005, 1.0)
            sample = math.sin(2.0 * math.pi * freq * t) * env * attack
            buf[i0 + k] += sample
    # normalize to amplitude, clip, convert to int16
    peak = max((abs(x) for x in buf), default=1.0) or 1.0
    scale = (amplitude * 32767.0) / peak
    return [max(-32768, min(32767, int(round(x * scale)))) for x in buf]


def write_wav(path: Path, samples):
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(RATE)
        wf.writeframes(struct.pack("<" + "h" * len(samples), *samples))


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        sys.exit(1)
    out_dir = Path(argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)

    # Pairing chime: A4 -> C5 (440 -> 523.25 Hz), each 180 ms, slight overlap
    pairing_notes = [
        (0.00, 0.20, 440.00),
        (0.18, 0.30, 523.25),
    ]
    write_wav(out_dir / "pairing.wav", synth(pairing_notes, 0.55))

    # Success: C5 E5 G5 ascending, ~70 ms each, with sustained final
    success_notes = [
        (0.00, 0.10, 523.25),
        (0.08, 0.10, 659.25),
        (0.16, 0.30, 783.99),
    ]
    write_wav(out_dir / "success.wav", synth(success_notes, 0.50))

    print(f"wrote pairing.wav and success.wav to {out_dir}")


if __name__ == "__main__":
    main(sys.argv)
