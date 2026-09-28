/*
JB60 mode

Half-time (about 61 s) PD120-class mode from mmsstv-linux, see modejb60.cpp there.
Luma is sent on a quincunx lattice and chroma is subsampled,
the receiver rebuilds the full picture with edge directed interpolation.

Modified 2026 Jason <jason@weisb.net>
*/

package xdsopl.robot36;

public class JB60 extends BaseMode {
	private static final int WIDTH = 640;
	private static final int HEIGHT = 496;
	private static final int PAIRS = HEIGHT / 2;
	private static final int Y0 = 0;
	private static final int Y1 = 1;
	private static final int CR = 2;
	private static final int CB = 3;
	private static final int[] SEGMENT_SLOTS = {WIDTH / 2, WIDTH / 2, WIDTH / 2, 224};
	private static final int[] SEGMENT_OFFSETS = {0, 320, 640, 960};
	private static final int TOTAL_SLOTS = 1184;
	// receive: luma guided chroma upsampling
	private static final float GUIDE_SIGMA = 20.f;
	private static final float GUIDE_FLOOR = 0.02f;

	private final ExponentialMovingAverage lowPassFilter;
	private final int scanLineSamples;
	private final int beginSamples;
	private final int endSamples;
	private final double slotSamples;
	private final int[][] slotCenters;
	private final int[][] chromaBegin;
	private final int[][] chromaEnd;
	private final float[] guideLut;
	private final int[][] cur;
	private final int[][] prev;
	private final int[] rowY;
	private final int[] rowCr;
	private final int[] rowCb;
	private final float[] chromaLuma;
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
		for (int seg = 0; seg < 4; ++seg) {
			slotCenters[seg] = new int[SEGMENT_SLOTS[seg]];
			for (int i = 0; i < SEGMENT_SLOTS[seg]; ++i)
				slotCenters[seg][i] = (int) Math.round((syncPorchSeconds + (SEGMENT_OFFSETS[seg] + i + 0.5) * slotSeconds) * sampleRate);
		}
		chromaBegin = new int[4][];
		chromaEnd = new int[4][];
		for (int seg = CR; seg <= CB; ++seg) {
			int n = SEGMENT_SLOTS[seg];
			chromaBegin[seg] = new int[n];
			chromaEnd[seg] = new int[n];
			double pw = (double) WIDTH / n;
			for (int j = 0; j < n; ++j) {
				int c0 = (int) Math.floor(j * pw + 1e-9);
				int c1 = Math.min((int) Math.ceil((j + 1) * pw - 1e-9) - 1, WIDTH - 1);
				chromaBegin[seg][j] = c0;
				chromaEnd[seg][j] = Math.max(c1, c0);
			}
		}
		guideLut = new float[256];
		for (int d = 0; d < 256; ++d) {
			float x = d / GUIDE_SIGMA;
			guideLut[d] = (float) Math.exp(-x * x);
		}
		cur = new int[4][];
		prev = new int[4][];
		for (int seg = 0; seg < 4; ++seg) {
			cur[seg] = new int[SEGMENT_SLOTS[seg]];
			prev[seg] = new int[SEGMENT_SLOTS[seg]];
		}
		rowY = new int[WIDTH];
		rowCr = new int[WIDTH];
		rowCb = new int[WIDTH];
		chromaLuma = new float[WIDTH];
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
		for (int seg = 0; seg < 4; ++seg) {
			java.util.Arrays.fill(cur[seg], 128);
			java.util.Arrays.fill(prev[seg], 128);
		}
	}

	// rebuild a full luma row from its own quincunx samples and the rows above and below
	private void reconstructLuma(int[] out, int[] own, int parity, int[] up, int[] down) {
		if (up == null)
			up = down;
		if (down == null)
			down = up;
		int samples = own.length;
		for (int c = 0; c < WIDTH; ++c) {
			if ((c & 1) == parity) {
				out[c] = own[c >> 1];
				continue;
			}
			int wi, ei;
			if (parity == 0) {
				wi = c >> 1;
				ei = wi + 1;
			} else {
				ei = c >> 1;
				wi = ei - 1;
			}
			if (wi < 0)
				wi = ei;
			if (ei >= samples)
				ei = wi;
			float w = own[wi];
			float e = own[ei];
			float n = up[c >> 1];
			float s = down[c >> 1];
			float dh = Math.abs(w - e) + 1.f;
			float dv = Math.abs(n - s) + 1.f;
			float wh = 1.f / (dh * dh);
			float wv = 1.f / (dv * dv);
			out[c] = clamp(Math.round((wh * (w + e) * 0.5f + wv * (n + s) * 0.5f) / (wh + wv)));
		}
	}

	// upsample chroma to full width, weights follow the luma
	private void upsampleChroma(int seg, int[] y, int[] c, int[] out) {
		int n = c.length;
		double pw = (double) WIDTH / n;
		for (int j = 0; j < n; ++j) {
			int sum = 0;
			for (int x = chromaBegin[seg][j]; x <= chromaEnd[seg][j]; ++x)
				sum += y[x];
			chromaLuma[j] = (float) sum / (chromaEnd[seg][j] - chromaBegin[seg][j] + 1);
		}
		for (int x = 0; x < WIDTH; ++x) {
			double jf = (x + 0.5) / pw - 0.5;
			int j0 = (int) Math.floor(jf);
			float t = (float) (jf - j0);
			int j1 = j0 + 1;
			j0 = Math.min(Math.max(j0, 0), n - 1);
			j1 = Math.min(Math.max(j1, 0), n - 1);
			int d0 = Math.min(Math.round(Math.abs(y[x] - chromaLuma[j0])), 255);
			int d1 = Math.min(Math.round(Math.abs(y[x] - chromaLuma[j1])), 255);
			float w0 = (1.f - t) * (GUIDE_FLOOR + guideLut[d0]);
			float w1 = t * (GUIDE_FLOOR + guideLut[d1]);
			out[x] = clamp(Math.round((w0 * c[j0] + w1 * c[j1]) / (w0 + w1)));
		}
	}

	private void emitRow(PixelBuffer pixelBuffer, int row, int[] own, int parity, int[] up, int[] down, int[] cr, int[] cb) {
		reconstructLuma(rowY, own, parity, up, down);
		upsampleChroma(CR, rowY, cr, rowCr);
		upsampleChroma(CB, rowY, cb, rowCb);
		int offset = row * WIDTH;
		for (int i = 0; i < WIDTH; ++i) {
			int y = rowY[i];
			int r = clamp((100 * y + 140 * rowCr[i] - 17850) / 100);
			int b = clamp((100 * y + 178 * rowCb[i] - 22695) / 100);
			int g = clamp((100 * y - 71 * rowCr[i] - 33 * rowCb[i] + 13260) / 100);
			pixelBuffer.pixels[offset + i] = 0xff000000 | (r << 16) | (g << 8) | b;
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
		for (int seg = 0; seg < 4; ++seg)
			for (int i = 0; i < SEGMENT_SLOTS[seg]; ++i)
				cur[seg][i] = clamp(Math.round(255 * scratchBuffer[slotCenters[seg][i]]));
		// the picture trails the received data by one row, a row needs its neighbours above and below
		int pair = lineCounter;
		boolean last = pair + 1 >= PAIRS;
		int rows = 0;
		if (pair == 0) {
			emitRow(pixelBuffer, rows++, cur[Y0], 0, null, cur[Y1], cur[CR], cur[CB]);
		} else {
			emitRow(pixelBuffer, rows++, prev[Y1], 1, prev[Y0], cur[Y0], prev[CR], prev[CB]);
			emitRow(pixelBuffer, rows++, cur[Y0], 0, prev[Y1], cur[Y1], cur[CR], cur[CB]);
		}
		if (last)
			emitRow(pixelBuffer, rows++, cur[Y1], 1, cur[Y0], null, cur[CR], cur[CB]);
		for (int seg = 0; seg < 4; ++seg) {
			int[] tmp = prev[seg];
			prev[seg] = cur[seg];
			cur[seg] = tmp;
		}
		lineCounter = (pair + 1) % PAIRS;
		pixelBuffer.width = WIDTH;
		pixelBuffer.height = rows;
		return true;
	}
}
