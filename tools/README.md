# JB60 chroma deconvolution: design tools and derivation

Companion to `JB60.java`'s `DECONV_HALF_TAPS`/`DECONV_KERNEL` (the "JB60 Chroma Sharpening" RX
setting, off by default). This is a **from-scratch port** of mmsstv-linux's own RX idea 4 (see its
`tests/jb60_loopback/README.md`, "RX idea 4" section) -- the methodology transfers, the numbers do
not, because this app's demod chain is architecturally different from mmsstv-linux's own.

## Why the numbers don't transfer

mmsstv-linux's JB60 decoder runs the demodulated signal through one filter: a single 181-tap
raised-cosine FIR (-6dB at 600Hz). This app instead runs every mode's demodulated signal through a
single, global, fixed 97-tap Kaiser-windowed-sinc low-pass (900Hz cutoff, `Demodulator.java`) and
then, only for JB60, an additional per-line, 2-pass, 2nd-order `ExponentialMovingAverage`
(`JB60.java`, cutoff pinned to the slot-rate Nyquist, ~2631Hz). Different filter, different
response, different inverse -- mmsstv-linux's kernel constants were measured against *its* filter
and don't apply here.

## Measurement: `JB60ChromaResponseDump`

`app/src/test/java/xdsopl/robot36/JB60ChromaResponseDump.java` -- a standalone tool (not a
`@Test`), the equivalent of mmsstv-linux's `loopback --dump-slots`. Unlike that tool, this app has
no JB60 encoder to fight: the "known input" is simply whichever slot values the tool asks for, no
TX-side box-averaging or luma-guided-downsample nonlinearity to work around. What still has to be
measured for real is the RX chain's own effect, so the tool always synthesizes real,
phase-continuous FM audio and runs it through the real `Demodulator.process()` -- never through
`JB60Test.java`'s own `fillSegment()`/`makeRampLine()` helpers, which inject directly into
`scanLineBuffer` and so skip `Demodulator` entirely (the same pitfall mmsstv-linux's own harness
documents having made once, for the same reason).

Two things had to be discovered empirically, not assumed:

1. **Trailing-sync sample offsets.** `JB60.decodeScanLine()`'s `syncPulseIndex` parameter names
   where the *porch* begins, with the mode's own reference sync pulse occupying the `syncSamples`
   immediately *before* that index -- not after it, as a naive reading of "sync pulse index" might
   suggest. Confirmed against `JB60Test.fillSegment()`'s own working offset arithmetic. Getting
   this backwards (sync-pulse-then-porch instead of porch-with-a-preceding-sync) produces a
   960-sample gap of unwritten (0.0f) audio and garbage FM-discriminator output at the seam --
   worth remembering if this tool is ever extended.
2. **Cr/Cb must be measured in separate lines.** Cr and Cb are each an independent, full-640-column
   source resampled to their own slot count -- adjacent in *transmission time* (segment order
   `L,D,Cr,Cb`), not in column space. Driving both with a step in the same line makes Cr end at its
   high plateau right where Cb starts at its own low plateau: a real transmitted discontinuity that
   bleeds into Cb's *head* (measured effect: Cb's apparent DC gain came out as 0.638 instead of the
   physically-required ~1.0). Fixed by measuring Cr and Cb independently, each time holding the
   *other* channel flat at the tested channel's own baseline level, so every boundary the tested
   channel touches is either flat-into-flat or a boundary artifact that already gets masked (the
   existing tail-into-next-line-sync case, `clean_tail()` in the design script).

Usage:
```
cd app && ../gradlew :app:compileDebugUnitTestJavaWithJavac -Dorg.gradle.java.home=<jdk21>
java -cp app/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes:app/build/intermediates/javac/debugUnitTest/compileDebugUnitTestJavaWithJavac/classes \
     xdsopl.robot36.JB60ChromaResponseDump both > dump.txt
```

## Design: `jb60_chroma_deconv_design.py`

New script (nothing to port -- mmsstv-linux never committed its own design script either, only the
`--dump-slots` diagnostic; the Wiener-fit work there was done out of band). Same DSP as the
documented mmsstv-linux method:

1. Difference the measured step response (after masking the boundary-artifact tail) to estimate the
   impulse response; FFT it for `H(f)`.
2. `Hinv(f) = |H(f)| / (|H(f)|^2 + K)`, phase forced to zero, **DC pinned to an exact 1.0** (not the
   Wiener-shrunk value -- flat/already-correct colour must pass through unchanged; this was the fix
   that mattered most on the mmsstv-linux side too, both in the continuous target and via an
   equality-constrained least-squares projection on the fitted taps).
3. Target held flat past a chosen "reliable" frequency where the measurement itself goes
   noise-dominated (found empirically for this filter pair, not assumed to match mmsstv-linux's own
   crossover -- see the measured `|H(f)|` table below).
4. Fit a symmetric FIR by weighted least squares over the cosine basis at `fs≈5263 Hz` (same slot
   rate as mmsstv-linux -- identical 190µs slot, confirmed).

```
python3 tools/jb60_chroma_deconv_design.py dump.txt              # prints measured |H(f)|, phase
python3 tools/jb60_chroma_deconv_design.py dump.txt 5 0.1 1300   # L=5 taps, K=0.1, reliable_hz=1300
```

### Measured response

The combined chain's own `|H(f)|` (average of independently-measured Cr, Cb -- they agree closely,
confirming one shared kernel is appropriate, same as mmsstv-linux's own precedent):

| f (Hz) | 0 | 100 | 300 | 600 | 900 | 1200 | 1500 | 2000 | 2631 |
|---|---|---|---|---|---|---|---|---|---|
| \|H\| | 1.000 | 0.992 | 0.930 | 0.738 | 0.474 | 0.225 | 0.073 | 0.035 | 0.016 |

Phase stays under ~1.5° out to about 900-1200Hz, then grows (noise-dominated beyond that -- target
held flat past 1300Hz in the shipped design). This confirms `Demodulator`'s 900Hz Kaiser low-pass,
not JB60's own gentler EMA, is the dominant blur: the -6dB point of the *combined* response
(~800-900Hz) sits close to the Kaiser filter's own stated cutoff.

### Tap-count sweep (K=0.1, clean, `card` + a hard-edged colour image)

| L (taps) | 3 | 5 | 7 | 9 |
|---|---|---|---|---|
| card PSNR all (dB) | +0.08 | **+0.14** | +0.13 | +0.14 |
| img1 PSNR all (dB) | +0.00 | +0.01 | +0.01 | +0.01 |

(deltas vs. deconvolution off; luma PSNR unaffected at every L, confirming no L/D leak). Diminishing
returns past 5 taps, same shape as mmsstv-linux's own tap-count finding -- **5 taps shipped**.

### K sweep (5 taps), full cross-repo WAV-bridge SNR grid

All-channel PSNR, `card`, off → on (dB):

| K | clean | 40dB | 30dB | 25dB | 20dB | 15dB | 10dB |
|---|---|---|---|---|---|---|---|
| 0.05 | 20.45→20.62 | 20.05→20.23 | 19.95→20.06 | 19.73→19.68 | 19.11→18.71 | 17.67→16.74 | 15.10→13.70 |
| **0.10** | 20.45→20.59 | 20.05→20.17 | 19.95→20.04 | 19.73→19.75 | 19.11→18.96 | 17.67→17.26 | 15.10→14.43 |
| 0.20 | 20.45→20.45 | 20.05→20.05 | 19.95→19.95 | 19.73→19.73 | 19.11→19.11 | 17.67→17.67 | 15.10→15.11 |

`K=0.2` is essentially a no-op at every SNR tested. `K=0.05` gives marginally more clean-signal gain
than `K=0.1` but a much steeper noise penalty (-1.40dB at 10dB vs. -0.67dB). **`K=0.1` shipped** --
the balanced pick, same broad-optimum-then-regression shape as every filter in this family
(mmsstv-linux's own `Gmax`/K sweeps included), and coincidentally the same K magnitude as
mmsstv-linux's own idea 4 despite a completely different filter chain. `img1` (the same synthetic
hard-edged image used as a photo proxy on the mmsstv-linux side, since no photo is committed to
either repo) shows the identical shape at roughly 1/5th the magnitude (smaller baseline PSNR
headroom on that harsher test image).

**Real SNR crossover: around 20-25dB**, consistent with mmsstv-linux's own ~22-25dB finding for the
same category of filter (RX-side inverse, pays a noise-amplification tax that a TX-side boost like
idea 6 doesn't). **Below about 5dB SNR, the real `Decoder`'s own VIS/sync detection fails to lock at
all** in this test, regardless of the chroma setting -- a real property of this app's front end that
mmsstv-linux's own idealized-timing harness can't see (it doesn't model sync detection at all,
using a calibrated-delay reference instead). Shipped default: **off**, same reasoning as every other
RX-side idea in this family.

## Validation: `WavFile` + `JB60WavBridge`

`app/src/test/java/xdsopl/robot36/{WavFile,JB60WavBridge}.java` -- reads a WAV produced by
mmsstv-linux's `loopback --wav --vis [--snr N --ssb]`, decodes it through the **real `Decoder`**
(real `Demodulator` + real VIS/sync detection + `JB60.decodeScanLine()` -- not a hand-driven
`Demodulator`+`JB60` pair), and writes the result as a binary PPM. PPM, not PNG: this app's Android
Gradle Plugin unit-test compile classpath excludes `javax.imageio`/`java.awt` even for a plain
JVM-side tool in `src/test`, so a hand-rolled, compression-free format was the path of least
resistance -- Qt's `QImage` (what `loopback --compare` uses) reads PPM natively.

```
# mmsstv-linux side
loopback jb --image card --wav card.wav --vis [--snr N --ssb]

# robot36-android side
java -cp <classes> xdsopl.robot36.JB60WavBridge card.wav --deconv=off card_off.ppm
java -cp <classes> xdsopl.robot36.JB60WavBridge card.wav --deconv=on  card_on.ppm

# mmsstv-linux side again -- no new comparison code needed on the Java side
loopback --compare card_src.ppm card_off.ppm
loopback --compare card_src.ppm card_on.ppm
```

(`card_src.ppm`/`img1_src.ppm`: the true source images, converted to PPM once -- `card.png` via any
PNG→PPM conversion, `img1` via `loopback jb --image 1 --out img1` then converting `img1_src.png`.)

**Caveats, stated plainly**: this validates against mmsstv-linux's own TX output only, not a real
off-air recording or another program's encoder. There is no Java-side equivalent to mmsstv-linux's
windowed/text-region SSIM -- only whatever `loopback --compare` reports on the whole image (the
same trusted engine, just without the region-specific metrics).
