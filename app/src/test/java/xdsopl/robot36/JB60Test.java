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
	private static final int[] segmentSlots = {320, 320, 320, 224};
	private static final int[] segmentOffsets = {0, 320, 640, 960};

	// flat chroma, luma ramps left to right, sampled like the transmitter does (quincunx)
	private float[] makeLine(JB60 mode) {
		float[] line = new float[2 * mode.getScanLineSamples()];
		for (int seg = 0; seg < 4; ++seg) {
			for (int i = 0; i < segmentSlots[seg]; ++i) {
				double level;
				if (seg == 0)
					level = (2 * i) / 639.0;
				else if (seg == 1)
					level = (2 * i + 1) / 639.0;
				else
					level = 127.5 / 255.0;
				int begin = (int) Math.round((porch + (segmentOffsets[seg] + i) * slot) * sampleRate);
				int end = (int) Math.round((porch + (segmentOffsets[seg] + i + 1) * slot) * sampleRate);
				for (int j = begin; j < end; ++j)
					line[j] = (float) (2 * level - 1);
			}
		}
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
		PixelBuffer pixelBuffer = new PixelBuffer(800, 3);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = makeLine(mode);
		int total = 0;
		for (int pair = 0; pair < 248; ++pair) {
			assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
			assertEquals(640, pixelBuffer.width);
			assertEquals(pair == 0 ? 1 : pair == 247 ? 3 : 2, pixelBuffer.height);
			total += pixelBuffer.height;
		}
		assertEquals(496, total);
		// wraps around to the start of the next picture
		assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
		assertEquals(1, pixelBuffer.height);
	}

	@Test
	public void jb60_reconstructs_luma_ramp() {
		JB60 mode = new JB60(sampleRate);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 3);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = makeLine(mode);
		for (int pair = 0; pair < 3; ++pair)
			assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
		// last call emitted rows 3 and 4, grey with luma from the ramp
		for (int row = 0; row < 2; ++row) {
			for (int col = 40; col < 600; col += 20) {
				int pixel = pixelBuffer.pixels[row * 640 + col];
				int expected = (int) Math.round(col * 255 / 639.0);
				int r = (pixel >> 16) & 255;
				int g = (pixel >> 8) & 255;
				int b = pixel & 255;
				assertTrue("row " + row + " col " + col + " r=" + r + " expected " + expected, Math.abs(r - expected) <= 8);
				assertTrue("row " + row + " col " + col + " g=" + g + " expected " + expected, Math.abs(g - expected) <= 8);
				assertTrue("row " + row + " col " + col + " b=" + b + " expected " + expected, Math.abs(b - expected) <= 8);
			}
		}
	}
}
