#!/usr/bin/env python3
"""
RX idea 4 (JB60 chroma deconvolution), robot36-android port: design the regularized (Wiener-style)
inverse filter from real JB60ChromaResponseDump measurements.

Unlike mmsstv-linux's design (which had to fight a TX encoder's luma-guided chroma downsample),
this app has no JB60 encoder: the "known input" is exactly the slot values the measurement tool
asked for -- a plain half/half step, no box-averaging to replicate. What's measured for real is the
combined response of Demodulator's global 900Hz Kaiser low-pass cascaded with JB60's own per-line,
2-pass, 2nd-order ExponentialMovingAverage.

Usage:
  java ... xdsopl.robot36.JB60ChromaResponseDump both > dump.txt
  python3 jb60_chroma_deconv_design.py dump.txt
"""
import sys
import numpy as np

FS_SLOT = 1.0 / 0.00019  # ~5263 Hz -- same slot duration as mmsstv-linux (confirmed identical)
N = {"cr": 224, "cb": 176}
LOW, HIGH = 128.0, 255.0


def load_dump(path):
    d = {}
    for line in open(path):
        tag, rest = line.strip().split(":", 1)
        d[tag] = np.array([int(x) for x in rest.split()], dtype=float)
    return d


def clean_tail(arr, ntail=15):
    # the measured segment's last few slots bleed toward the *next* line's sync tone (a real,
    # non-bug transmission-time boundary effect -- see JB60ChromaResponseDump's own comments and
    # the design writeup) -- mask before differencing, matching mmsstv-linux's own precedent for a
    # structurally similar artifact.
    a = arr.copy()
    plateau = np.median(a[-(ntail + 15):-ntail])
    a[-ntail:] = plateau
    return a


def measure_H(d, channel, nfft=2048):
    n = N[channel]
    meas = clean_tail(d[f"{channel}_step"])
    known = np.full(n, LOW)
    known[n // 2:] = HIGH
    dIn = np.diff(known)
    dOut = np.diff(meas)
    X = np.fft.rfft(dIn, nfft)
    Y = np.fft.rfft(dOut, nfft)
    freqs = np.fft.rfftfreq(nfft, d=1.0 / FS_SLOT)
    H = Y / X
    return freqs, H


def fit_symmetric_fir(freqs, target, L, weight, fs=FS_SLOT):
    M = (L - 1) // 2
    w = 2 * np.pi * freqs / fs
    A = np.zeros((len(freqs), M + 1))
    A[:, 0] = 1.0
    for t in range(1, M + 1):
        A[:, t] = 2 * np.cos(w * t)
    Wsqrt = np.sqrt(weight)
    sol, _, _, _ = np.linalg.lstsq(A * Wsqrt[:, None], target * Wsqrt, rcond=None)
    # exact DC equality projection (flat, already-correct colour must pass through unchanged)
    g = A[0, :].copy()
    resid = target[0] - g @ sol
    sol = sol + g * (resid / (g @ g))
    return sol


def design_kernel(freqs, magAvg, L, K, reliable_hz, dc_boost=200.0, hf_weight=0.15):
    nyq = FS_SLOT / 2
    mask = freqs <= nyq
    f = freqs[mask]
    m = magAvg[mask]
    target = m / (m ** 2 + K)
    target[0] = 1.0  # DC pinned exactly, not the Wiener-shrunk value -- see mmsstv-linux precedent
    i_edge = np.searchsorted(f, reliable_hz)
    target[i_edge:] = target[i_edge]
    weight = np.where(f <= reliable_hz, 1.0, hf_weight)
    weight[0] = dc_boost
    taps = fit_symmetric_fir(f, target, L, weight)
    return taps


if __name__ == "__main__":
    d = load_dump(sys.argv[1])
    freqsCr, Hcr = measure_H(d, "cr")
    freqsCb, Hcb = measure_H(d, "cb")
    assert np.allclose(freqsCr, freqsCb)
    freqs = freqsCr
    magCr, magCb = np.abs(Hcr), np.abs(Hcb)
    magAvg = (magCr + magCb) / 2

    idx = [np.argmin(np.abs(freqs - f)) for f in (0, 100, 300, 600, 900, 1200, 1500, 2000, 2631)]
    print("--- measured |H(f)| (Cr, Cb, avg), phase (Cr,Cb, deg) ---")
    for i in idx:
        print(f"  f={freqs[i]:7.1f} Hz  |H| Cr={magCr[i]:.3f} Cb={magCb[i]:.3f} avg={magAvg[i]:.3f}"
              f"   phase Cr={np.degrees(np.angle(Hcr[i])):7.2f} Cb={np.degrees(np.angle(Hcb[i])):7.2f}")

    if len(sys.argv) > 2:
        L = int(sys.argv[2])
        K = float(sys.argv[3])
        reliable_hz = float(sys.argv[4]) if len(sys.argv) > 4 else 1500.0
        taps = design_kernel(freqs, magAvg, L, K, reliable_hz)
        M = len(taps) - 1
        dc = taps[0] + 2 * sum(taps[1:])
        print(f"\nL={L} K={K} reliable_hz={reliable_hz}")
        print(f"taps={np.round(taps, 6)}  DCgain={dc:.4f}")
        print("Java array literal:")
        print("{" + ", ".join(f"{t:.6f}f" for t in taps) + "}")
