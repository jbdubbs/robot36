/*
JB60 mode

Half-time (about 61 s) PD120-class mode from mmsstv-linux, see modejb60.cpp there.
L is the mean luma of the two rows at one slot per pixel (the same density PD's own channels use); D is
the vertical detail (a companded residual against a prediction from the neighbouring L lines); Cr/Cb are
luma-guided upsampled chroma. See src/sstv/modes/modejb60.{h,cpp} in mmsstv-linux for the reference
implementation this ports; that file's header comment explains why the format is laid out this way.

Modified 2026 Jason <jason@weisb.net>
*/

package xdsopl.robot36;

public class JB60 extends BaseMode {
	private static final int WIDTH = 640;
	private static final int HEIGHT = 496;
	private static final int PAIRS = HEIGHT / 2;
	private static final int L = 0, D = 1, CR = 2, CB = 3;
	private static final int[] SEGMENT_SLOTS = {640, 144, 224, 176};
	private static final int[] SEGMENT_OFFSETS = {0, 640, 784, 1008};
	private static final int TOTAL_SLOTS = 1184;
	// vertical detail channel: level = 128 + sign(d)*D_MAX*(|d|/D_MAX)^D_GAMMA, so small differences get more of the swing
	private static final float D_MAX = 127.f;
	private static final float D_GAMMA = 0.6f;
	// receive: luma guided chroma upsampling
	private static final float GUIDE_SIGMA = 20.f;
	private static final float GUIDE_FLOOR = 0.02f;
	// RX chroma deconvolution (opt-in, off by default): regularized (Wiener-style) inverse of the
	// combined Demodulator (900Hz Kaiser low-pass) + this mode's own 2-pass EMA slot-domain
	// response, applied to already-demodulated Cr/Cb right after capture. Symmetric (zero group
	// delay), 5 taps.
	//
	// Design (see tools/README.md and tools/jb60_chroma_deconv_design.py for the full derivation, a
	// from-scratch redesign against this app's own filter chain -- not a port of mmsstv-linux's own
	// modejb60.cpp constants, which are specific to its different demod filter): H(f) was measured
	// through the real chain (JB60ChromaResponseDump, phase-continuous FM audio synthesis -> real
	// Demodulator.process() -> real decodeScanLine()), Cr and Cb each in their own line so a step in
	// one never bleeds into the other's head (this app has no encoder, so unlike mmsstv-linux there
	// was no need to fight a TX-side nonlinearity, only that one self-inflicted transmission-time
	// boundary artifact). Measurement confirms Demodulator's 900Hz Kaiser low-pass, not this mode's
	// own gentler slot-Nyquist-cutoff EMA, is the dominant blur (|H(600Hz)|~=0.74, |H(900Hz)|~=0.47,
	// falling off further beyond, matching the Kaiser filter's own stated cutoff closely). The
	// inverse is Hinv(f)=|H(f)|/(|H(f)|^2+K), K=0.1, DC pinned to an exact 1.0 (flat/already-correct
	// colour must pass through unchanged -- same fix that mattered most on the mmsstv-linux side),
	// target held flat past 1300 Hz where the measurement itself goes noise-dominated. Fit to a
	// 5-tap symmetric FIR by weighted least squares over the cosine basis.
	//
	// Tap-count sweep (K=0.1, clean, card + a hard-edged colour image): diminishing returns past 5
	// taps (same shape as mmsstv-linux's own tap-count finding) -- 5 taps shipped. K sweep at 5 taps
	// through the full cross-repo WAV-bridge SNR grid (tools/README.md has the whole table): K=0.2
	// is essentially a no-op at every SNR tested; K=0.05 gives slightly more clean-signal gain but a
	// much steeper noise penalty (-1.40dB card all-channel PSNR at 10dB SNR, vs K=0.1's -0.67dB);
	// K=0.1 is the balanced pick -- same broad-optimum-then-regression shape as every other filter
	// in this family, landing at the same K magnitude as mmsstv-linux's own idea 4 despite a
	// completely different filter chain. Real SNR crossover found around 20-25dB (small gains
	// above, growing losses below), consistent with mmsstv-linux's own ~22-25dB finding for the
	// analogous RX-side (noise-amplifying, unlike TX-side idea 6) filter. Below about 5dB SNR the
	// real Decoder's own VIS/sync detection fails to lock at all in this test, independent of this
	// setting -- a real property of this app's front end that mmsstv-linux's own idealized-timing
	// harness can't see (it doesn't model sync detection at all).
	private static final int DECONV_HALF_TAPS = 2;
	private static final float[] DECONV_KERNEL = {1.284351f, -0.033209f, -0.108966f};

	private final ExponentialMovingAverage lowPassFilter;
	private final int scanLineSamples;
	private final int beginSamples;
	private final int endSamples;
	private final double slotSamples;
	private final int[][] slotCenters;
	private final int[][] footprintBegin;
	private final int[][] footprintEnd;
	private final float[] guideLut;
	private final float[] dDecodeLut;
	private int[] curL, curD, curCr, curCb;
	private int[] prevL, prevD, prevCr, prevCb;
	private int[] prev2L;
	private final int[] rowY;
	private final int[] rowCr;
	private final int[] rowCb;
	private final float[] segBar; // scratch: per-footprint mean luma, reused by upsampleChroma
	private final float[] dExp; // scratch: decoded (expanded) D samples
	private final float[] dFull; // scratch: full width vertical detail
	private final int[] deconvScratchCr; // scratch: read-from copy for applyChromaDeconvolution(curCr, ...)
	private final int[] deconvScratchCb; // scratch: read-from copy for applyChromaDeconvolution(curCb, ...)
	private boolean chromaDeconvolutionEnabled = false;
	private int lineCounter;

	JB60(int sampleRate) {
		double syncPulseSeconds = 0.02;
		double syncPorchSeconds = 0.00208;
		double slotSeconds = 0.00019;
		double scanLineSeconds = syncPulseSeconds + syncPorchSeconds + TOTAL_SLOTS * slotSeconds;
		scanLineSamples = (int) Math.round(scanLineSeconds * sampleRate);
		beginSamples = (int) Math.round(syncPorchSeconds * sampleRate);
		endSamples = (int) Math.round((syncPorchSeconds + TOTAL_SLOTS * slotSeconds) * sampleRate);
		slotSamples = slotSeconds * sampleRate;
		slotCenters = new int[4][];
		footprintBegin = new int[4][];
		footprintEnd = new int[4][];
		for (int seg = 0; seg < 4; ++seg) {
			int n = SEGMENT_SLOTS[seg];
			slotCenters[seg] = new int[n];
			for (int i = 0; i < n; ++i)
				slotCenters[seg][i] = (int) Math.round((syncPorchSeconds + (SEGMENT_OFFSETS[seg] + i + 0.5) * slotSeconds) * sampleRate);
			footprintBegin[seg] = new int[n];
			footprintEnd[seg] = new int[n];
			double pw = (double) WIDTH / n;
			for (int j = 0; j < n; ++j) {
				int c0 = (int) Math.floor(j * pw + 1e-9);
				int c1 = Math.min((int) Math.ceil((j + 1) * pw - 1e-9) - 1, WIDTH - 1);
				footprintBegin[seg][j] = c0;
				footprintEnd[seg][j] = Math.max(c1, c0);
			}
		}
		guideLut = new float[256];
		dDecodeLut = new float[256];
		for (int i = 0; i < 256; ++i) {
			float x = i / GUIDE_SIGMA;
			guideLut[i] = (float) Math.exp(-x * x);
			float u = (i - 128) / D_MAX;
			u = Math.min(Math.max(u, -1.f), 1.f);
			float v = D_MAX * (float) Math.pow(Math.abs(u), 1.f / D_GAMMA);
			dDecodeLut[i] = u < 0 ? -v : v;
		}
		curL = new int[SEGMENT_SLOTS[L]];
		curD = new int[SEGMENT_SLOTS[D]];
		curCr = new int[SEGMENT_SLOTS[CR]];
		curCb = new int[SEGMENT_SLOTS[CB]];
		prevL = new int[SEGMENT_SLOTS[L]];
		prevD = new int[SEGMENT_SLOTS[D]];
		prevCr = new int[SEGMENT_SLOTS[CR]];
		prevCb = new int[SEGMENT_SLOTS[CB]];
		prev2L = new int[SEGMENT_SLOTS[L]];
		rowY = new int[WIDTH];
		rowCr = new int[WIDTH];
		rowCb = new int[WIDTH];
		segBar = new float[WIDTH];
		dExp = new float[SEGMENT_SLOTS[D]];
		dFull = new float[WIDTH];
		deconvScratchCr = new int[SEGMENT_SLOTS[CR]];
		deconvScratchCb = new int[SEGMENT_SLOTS[CB]];
		lowPassFilter = new ExponentialMovingAverage();
		resetState();
	}

	private static int clamp(int value) {
		return Math.min(Math.max(value, 0), 255);
	}

	private float freqToLevel(float frequency, float offset) {
		return 0.5f * (frequency - offset + 1.f);
	}

	@Override
	public String getName() {
		return "JB60";
	}

	@Override
	public int getVISCode() {
		return 102;
	}

	@Override
	public int getWidth() {
		return WIDTH;
	}

	@Override
	public int getHeight() {
		return HEIGHT;
	}

	@Override
	public int getFirstPixelSampleIndex() {
		return beginSamples;
	}

	@Override
	public int getFirstSyncPulseIndex() {
		return 0;
	}

	@Override
	public int getScanLineSamples() {
		return scanLineSamples;
	}

	@Override
	public void resetState() {
		lineCounter = 0;
		java.util.Arrays.fill(curL, 128);
		java.util.Arrays.fill(curD, 128);
		java.util.Arrays.fill(curCr, 128);
		java.util.Arrays.fill(curCb, 128);
		java.util.Arrays.fill(prevL, 128);
		java.util.Arrays.fill(prevD, 128);
		java.util.Arrays.fill(prevCr, 128);
		java.util.Arrays.fill(prevCb, 128);
		java.util.Arrays.fill(prev2L, 128);
	}

	public void setChromaDeconvolutionEnabled(boolean enabled) {
		chromaDeconvolutionEnabled = enabled;
	}

	// symmetric (zero group delay) FIR, edge-clamped taps -- see the DECONV_KERNEL comment above
	private void applyChromaDeconvolution(int[] arr, int n, int[] scratch) {
		System.arraycopy(arr, 0, scratch, 0, n); // read from a copy, not partially-overwritten neighbours
		for (int k = 0; k < n; ++k) {
			float v = DECONV_KERNEL[0] * scratch[k];
			for (int t = 1; t <= DECONV_HALF_TAPS; ++t) {
				int lo = Math.min(Math.max(k - t, 0), n - 1);
				int hi = Math.min(Math.max(k + t, 0), n - 1);
				v += DECONV_KERNEL[t] * (scratch[lo] + scratch[hi]);
			}
			arr[k] = clamp(Math.round(v));
		}
	}

	// plain linear interpolation of n samples (sample j centred at (j+0.5)*WIDTH/n) across the full width
	private float interpolate(float[] s, int n, int x) {
		double pw = (double) WIDTH / n;
		double jf = (x + 0.5) / pw - 0.5;
		int j0 = (int) Math.floor(jf);
		float t = (float) (jf - j0);
		int j1 = j0 + 1;
		j0 = Math.min(Math.max(j0, 0), n - 1);
		j1 = Math.min(Math.max(j1, 0), n - 1);
		return s[j0] * (1.f - t) + s[j1] * t;
	}

	// upsample one chroma component (n samples) to the full row, luma-guided (edges snap to the luma's)
	private void upsampleChroma(int seg, int[] y, int[] c, int[] out) {
		int n = c.length;
		double pw = (double) WIDTH / n;
		for (int j = 0; j < n; ++j) {
			int sum = 0;
			for (int x = footprintBegin[seg][j]; x <= footprintEnd[seg][j]; ++x)
				sum += y[x];
			segBar[j] = (float) sum / (footprintEnd[seg][j] - footprintBegin[seg][j] + 1);
		}
		for (int x = 0; x < WIDTH; ++x) {
			double jf = (x + 0.5) / pw - 0.5;
			int j0 = (int) Math.floor(jf);
			float t = (float) (jf - j0);
			int j1 = j0 + 1;
			j0 = Math.min(Math.max(j0, 0), n - 1);
			j1 = Math.min(Math.max(j1, 0), n - 1);
			int d0 = Math.min(Math.round(Math.abs(y[x] - segBar[j0])), 255);
			int d1 = Math.min(Math.round(Math.abs(y[x] - segBar[j1])), 255);
			float w0 = (1.f - t) * (GUIDE_FLOOR + guideLut[d0]);
			float w1 = t * (GUIDE_FLOOR + guideLut[d1]);
			out[x] = clamp(Math.round((w0 * c[j0] + w1 * c[j1]) / (w0 + w1)));
		}
	}

	// rebuild the two rows of one pair (lPrev/lNext: the L lines of the pairs before/after, or a self
	// mirror at the picture edges) and write them into pixelBuffer starting at row `at`
	private void emitPair(PixelBuffer pixelBuffer, int at, int[] lPrev, int[] l, int[] lNext, int[] d, int[] cr, int[] cb) {
		int nD = d.length;
		for (int k = 0; k < nD; ++k)
			dExp[k] = dDecodeLut[d[k]];
		for (int c = 0; c < WIDTH; ++c)
			dFull[c] = interpolate(dExp, nD, c) + (lPrev[c] - (float) lNext[c]) / 8.f;
		for (int row = 0; row < 2; ++row) {
			for (int c = 0; c < WIDTH; ++c)
				rowY[c] = clamp(Math.round(row == 0 ? l[c] + dFull[c] : l[c] - dFull[c]));
			upsampleChroma(CR, rowY, cr, rowCr);
			upsampleChroma(CB, rowY, cb, rowCb);
			int offset = (at + row) * WIDTH;
			for (int i = 0; i < WIDTH; ++i) {
				int y = rowY[i];
				int r = clamp((100 * y + 140 * rowCr[i] - 17850) / 100);
				int b = clamp((100 * y + 178 * rowCb[i] - 22695) / 100);
				int g = clamp((100 * y - 71 * rowCr[i] - 33 * rowCb[i] + 13260) / 100);
				pixelBuffer.pixels[offset + i] = 0xff000000 | (r << 16) | (g << 8) | b;
			}
		}
	}

	@Override
	public boolean decodeScanLine(PixelBuffer pixelBuffer, float[] scratchBuffer, float[] scanLineBuffer, int scopeBufferWidth, int syncPulseIndex, int scanLineSamples, float frequencyOffset) {
		if (syncPulseIndex + beginSamples < 0 || syncPulseIndex + endSamples > scanLineBuffer.length)
			return false;
		lowPassFilter.cutoff(1, 2 * slotSamples, 2);
		lowPassFilter.reset();
		for (int i = beginSamples; i < endSamples; ++i)
			scratchBuffer[i] = lowPassFilter.avg(scanLineBuffer[syncPulseIndex + i]);
		lowPassFilter.reset();
		for (int i = endSamples - 1; i >= beginSamples; --i)
			scratchBuffer[i] = freqToLevel(lowPassFilter.avg(scratchBuffer[i]), frequencyOffset);
		for (int i = 0; i < SEGMENT_SLOTS[L]; ++i)
			curL[i] = clamp(Math.round(255 * scratchBuffer[slotCenters[L][i]]));
		for (int i = 0; i < SEGMENT_SLOTS[D]; ++i)
			curD[i] = clamp(Math.round(255 * scratchBuffer[slotCenters[D][i]]));
		for (int i = 0; i < SEGMENT_SLOTS[CR]; ++i)
			curCr[i] = clamp(Math.round(255 * scratchBuffer[slotCenters[CR][i]]));
		for (int i = 0; i < SEGMENT_SLOTS[CB]; ++i)
			curCb[i] = clamp(Math.round(255 * scratchBuffer[slotCenters[CB][i]]));
		if (chromaDeconvolutionEnabled) {
			applyChromaDeconvolution(curCr, SEGMENT_SLOTS[CR], deconvScratchCr);
			applyChromaDeconvolution(curCb, SEGMENT_SLOTS[CB], deconvScratchCb);
		}

		// the picture trails the received data by one pair: pair p finishes pair p-1 (and, at the end of
		// the picture, itself as well) -- see modejb60.cpp's showLine() for the reference logic
		int pair = lineCounter;
		boolean last = pair + 1 >= PAIRS;
		int rows = 0;
		if (pair > 0) {
			emitPair(pixelBuffer, rows, pair > 1 ? prev2L : prevL, prevL, curL, prevD, prevCr, prevCb);
			rows += 2;
		}
		if (last) {
			emitPair(pixelBuffer, rows, pair > 0 ? prevL : curL, curL, curL, curD, curCr, curCb);
			rows += 2;
		}

		int[] tmp = prev2L;
		prev2L = prevL;
		prevL = curL;
		curL = tmp;
		tmp = prevD;
		prevD = curD;
		curD = tmp;
		tmp = prevCr;
		prevCr = curCr;
		curCr = tmp;
		tmp = prevCb;
		prevCb = curCb;
		curCb = tmp;

		lineCounter = (pair + 1) % PAIRS;
		pixelBuffer.width = WIDTH;
		pixelBuffer.height = rows;
		return true;
	}

	// Diagnostic accessors, not used by the app itself: the most recently RX-captured (post-demod,
	// pre-deconvolution) Cr/Cb slot arrays. decodeScanLine()'s tmp-swap moves its freshly captured
	// curCr/curCb into prevCr/prevCb before returning, so immediately after any decodeScanLine()
	// call these hold exactly the last line pair's demodulated chroma -- used by
	// JB60ChromaResponseDump/JB60WavBridge to measure the real chain's slot-domain response for the
	// chroma deconvolution filter's design (see mmsstv-linux's modejb60.h for the reference of this
	// same accessor pair).
	int[] lastCr() {
		return prevCr;
	}

	int[] lastCb() {
		return prevCb;
	}
}
