/*
Test for JB60 mode
*/

package xdsopl.robot36;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class JB60Test {
	private static final int sampleRate = 48000;
	private static final double porch = 0.00208;
	private static final double slot = 0.00019;
	private static final int[] segmentSlots = {640, 144, 224, 176};
	private static final int[] segmentOffsets = {0, 640, 784, 1008};
	private static final int L = 0, D = 1, CR = 2, CB = 3;
	// matches modejb60.cpp's encodeD(): kDMax=127, kDGamma=0.6
	private static final double D_MAX = 127.0;
	private static final double D_GAMMA = 0.6;

	private static int encodeD(double d) {
		double a = Math.min(Math.abs(d), D_MAX);
		double v = D_MAX * Math.pow(a / D_MAX, D_GAMMA);
		return (int) Math.min(255, Math.max(0, Math.round(128 + (d < 0 ? -v : v))));
	}

	// places one segment's worth of slots into a synthetic frequency track, `level` in 0..1 per slot index
	private void fillSegment(float[] line, int seg, double[] level) {
		for (int i = 0; i < segmentSlots[seg]; ++i) {
			int begin = (int) Math.round((porch + (segmentOffsets[seg] + i) * slot) * sampleRate);
			int end = (int) Math.round((porch + (segmentOffsets[seg] + i + 1) * slot) * sampleRate);
			for (int j = begin; j < end; ++j)
				line[j] = (float) (2 * level[i] - 1);
		}
	}

	// L is a left-to-right ramp, D is flat (zero vertical detail, i.e. both rows of every pair are the
	// ramp), chroma is flat grey -- every pair is identical, so the (Lprev-Lnext)/8 prediction is zero too
	private float[] makeRampLine(JB60 mode) {
		float[] line = new float[2 * mode.getScanLineSamples()];
		double[] lLevel = new double[segmentSlots[L]];
		for (int i = 0; i < lLevel.length; ++i)
			lLevel[i] = i / (segmentSlots[L] - 1.0);
		fillSegment(line, L, lLevel);
		double[] flat128 = new double[segmentSlots[D]];
		java.util.Arrays.fill(flat128, 128.0 / 255.0);
		fillSegment(line, D, flat128);
		double[] grey = new double[segmentSlots[CR]];
		java.util.Arrays.fill(grey, 127.5 / 255.0);
		fillSegment(line, CR, grey);
		grey = new double[segmentSlots[CB]];
		java.util.Arrays.fill(grey, 127.5 / 255.0);
		fillSegment(line, CB, grey);
		return line;
	}

	// every pair is the same flat pair (rowA==rowAVal, rowB==rowBVal everywhere), so L and the true D are
	// both constant across pairs and the prediction is zero: D directly encodes (rowAVal-rowBVal)/2
	private float[] makeFlatPairLine(JB60 mode, int rowAVal, int rowBVal) {
		float[] line = new float[2 * mode.getScanLineSamples()];
		double lVal = Math.round((rowAVal + rowBVal) / 2.0) / 255.0;
		double[] lLevel = new double[segmentSlots[L]];
		java.util.Arrays.fill(lLevel, lVal);
		fillSegment(line, L, lLevel);
		double[] dLevel = new double[segmentSlots[D]];
		java.util.Arrays.fill(dLevel, encodeD((rowAVal - rowBVal) / 2.0) / 255.0);
		fillSegment(line, D, dLevel);
		double[] grey = new double[segmentSlots[CR]];
		java.util.Arrays.fill(grey, 127.5 / 255.0);
		fillSegment(line, CR, grey);
		grey = new double[segmentSlots[CB]];
		java.util.Arrays.fill(grey, 127.5 / 255.0);
		fillSegment(line, CB, grey);
		return line;
	}

	@Test
	public void jb60_parameters() {
		JB60 mode = new JB60(sampleRate);
		assertEquals("JB60", mode.getName());
		assertEquals(102, mode.getVISCode());
		assertEquals(640, mode.getWidth());
		assertEquals(496, mode.getHeight());
		assertEquals(11858, mode.getScanLineSamples());
		assertEquals(100, mode.getFirstPixelSampleIndex());
		assertEquals(0, mode.getFirstSyncPulseIndex());
	}

	@Test
	public void jb60_row_counts() {
		JB60 mode = new JB60(sampleRate);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 4);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = makeRampLine(mode);
		int total = 0;
		for (int pair = 0; pair < 248; ++pair) {
			assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
			assertEquals(640, pixelBuffer.width);
			// pair 0: nothing finishes yet. pairs 1..246: finish the previous pair (2 rows). the last pair
			// (247) finishes pair 246 *and* mirrors itself for the bottom edge (4 rows).
			assertEquals(pair == 0 ? 0 : pair == 247 ? 4 : 2, pixelBuffer.height);
			total += pixelBuffer.height;
		}
		assertEquals(496, total);
		// wraps around to the start of the next picture: pair 0 again emits nothing yet
		assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
		assertEquals(0, pixelBuffer.height);
	}

	@Test
	public void jb60_reconstructs_luma_ramp() {
		JB60 mode = new JB60(sampleRate);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 4);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = makeRampLine(mode);
		int rows = 0;
		for (int pair = 0; pair < 3 && rows == 0; ++pair) {
			assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
			rows = pixelBuffer.height;
		}
		// L is one slot per pixel (no interpolation guesswork, unlike the old quincunx layout), so this
		// should reconstruct tightly; D is flat/zero, so both rows carry the same ramp
		for (int row = 0; row < 2; ++row) {
			for (int col = 10; col < 630; col += 20) {
				int pixel = pixelBuffer.pixels[row * 640 + col];
				int expected = (int) Math.round(col * 255 / 639.0);
				int r = (pixel >> 16) & 255;
				int g = (pixel >> 8) & 255;
				int b = pixel & 255;
				assertTrue("row " + row + " col " + col + " r=" + r + " expected " + expected, Math.abs(r - expected) <= 4);
				assertTrue("row " + row + " col " + col + " g=" + g + " expected " + expected, Math.abs(g - expected) <= 4);
				assertTrue("row " + row + " col " + col + " b=" + b + " expected " + expected, Math.abs(b - expected) <= 4);
			}
		}
	}

	@Test
	public void jb60_reconstructs_vertical_detail() {
		JB60 mode = new JB60(sampleRate);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 4);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = makeFlatPairLine(mode, 200, 100);
		int rows = 0;
		for (int pair = 0; pair < 3 && rows == 0; ++pair) {
			assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
			rows = pixelBuffer.height;
		}
		// row 0 of the pair should come back near 200, row 1 near 100 -- the vertical detail the quincunx
		// layout could not represent at all without heavy interpolation guesswork
		for (int col = 10; col < 630; col += 40) {
			int p0 = pixelBuffer.pixels[col];
			int p1 = pixelBuffer.pixels[640 + col];
			int g0 = (p0 >> 8) & 255;
			int g1 = (p1 >> 8) & 255;
			assertTrue("row 0 col " + col + " g=" + g0, Math.abs(g0 - 200) <= 6);
			assertTrue("row 1 col " + col + " g=" + g1, Math.abs(g1 - 100) <= 6);
		}
	}

	// Wiring/safety only, not efficacy evidence: confirms the chroma deconvolution toggle doesn't
	// crash, doesn't push chroma out of byte range, and -- since flat/grey chroma is a fixed point
	// of a DC-pinned filter -- barely moves the reconstructed colour at all on this synthetic
	// signal. It cannot demonstrate the filter's real benefit, since makeRampLine()/fillSegment()
	// write directly into scanLineBuffer and so skip the real Demodulator entirely (the same
	// limitation JB60ChromaResponseDump's own header comment documents) -- see tools/README.md for
	// the real evidence, gathered through the actual Demodulator+Decoder chain.
	@Test
	public void jb60_chroma_deconvolution_is_a_no_op_on_flat_chroma() {
		JB60 mode = new JB60(sampleRate);
		mode.setChromaDeconvolutionEnabled(true);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 4);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = makeRampLine(mode);
		int rows = 0;
		for (int pair = 0; pair < 3 && rows == 0; ++pair) {
			assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
			rows = pixelBuffer.height;
		}
		// same bounds as jb60_reconstructs_luma_ramp (deconvolution off): a DC-pinned filter applied
		// to already-flat chroma should reproduce the same picture, within the same tolerance
		for (int row = 0; row < 2; ++row) {
			for (int col = 10; col < 630; col += 20) {
				int pixel = pixelBuffer.pixels[row * 640 + col];
				int expected = (int) Math.round(col * 255 / 639.0);
				int r = (pixel >> 16) & 255;
				int g = (pixel >> 8) & 255;
				int b = pixel & 255;
				assertTrue("row " + row + " col " + col + " r=" + r + " expected " + expected, Math.abs(r - expected) <= 4);
				assertTrue("row " + row + " col " + col + " g=" + g + " expected " + expected, Math.abs(g - expected) <= 4);
				assertTrue("row " + row + " col " + col + " b=" + b + " expected " + expected, Math.abs(b - expected) <= 4);
			}
		}
	}
}
