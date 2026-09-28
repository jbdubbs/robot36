/*
Test for PD modes
*/

package xdsopl.robot36;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PaulDonTest {
	private static final int sampleRate = 48000;

	@Test
	public void pd120s_parameters() {
		PaulDon mode = new PaulDon("120S", 100, 640, 496, 0.01, 0.00104, 0.0608, sampleRate);
		assertEquals("PD 120S", mode.getName());
		assertEquals(100, mode.getVISCode());
		assertEquals(640, mode.getWidth());
		assertEquals(496, mode.getHeight());
		assertEquals(12204, mode.getScanLineSamples());
		assertEquals(50, mode.getFirstPixelSampleIndex());
	}

	@Test
	public void pd120w_parameters() {
		PaulDon mode = new PaulDon("120W", 101, 768, 432, 0.02, 0.00208, 0.14592, sampleRate);
		assertEquals("PD 120W", mode.getName());
		assertEquals(101, mode.getVISCode());
		assertEquals(768, mode.getWidth());
		assertEquals(432, mode.getHeight());
		assertEquals(29076, mode.getScanLineSamples());
	}

	@Test
	public void pd120_defaults_unchanged() {
		PaulDon mode = new PaulDon("120", 95, 640, 496, 0.1216, sampleRate);
		assertEquals("PD 120", mode.getName());
		assertEquals(24407, mode.getScanLineSamples());
		assertEquals(100, mode.getFirstPixelSampleIndex());
	}

	@Test
	public void pd120w_decodes_full_width_line() {
		PaulDon mode = new PaulDon("120W", 101, 768, 432, 0.02, 0.00208, 0.14592, sampleRate);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 2);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		float[] scanLineBuffer = new float[2 * mode.getScanLineSamples()];
		assertTrue(mode.decodeScanLine(pixelBuffer, scratchBuffer, scanLineBuffer, 640, 0, mode.getScanLineSamples(), 0));
		assertEquals(768, pixelBuffer.width);
		assertEquals(2, pixelBuffer.height);
	}
}
