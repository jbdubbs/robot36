/*
JB60 chroma deconvolution filter design tool: measures the real Demodulator+JB60 chain's slot-
domain response to known Cr/Cb patterns.

Not a JUnit test -- a standalone tool (clean stdout for a Python design script to consume). Run
with, e.g.:

  java -cp app/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes:\
app/build/intermediates/javac/debugUnitTest/compileDebugUnitTestJavaWithJavac/classes \
       xdsopl.robot36.JB60ChromaResponseDump both

Unlike mmsstv-linux's --dump-slots (which has to fight a real TX encoder's luma-guided chroma
downsample), this app has no JB60 encoder at all: the "known input" is simply whichever slot
values this tool chooses to synthesize, with no intermediate nonlinearity to defeat. What still has
to be measured for real is the RX chain's effect on those slots: Demodulator's global, stateful,
900 Hz Kaiser-windowed-sinc low-pass, cascaded with JB60's own per-line, 2-pass, 2nd-order
ExponentialMovingAverage. A synthetic scanLineBuffer built by hand (the way JB60Test's own
fillSegment()/makeRampLine() helpers do) bypasses Demodulator entirely and would under-measure the
real response -- this tool always goes through real, phase-continuous FM audio and the real
Demodulator.process(), the same as a live signal would.

Modified 2026 Jason <jason@weisb.net>
*/

package xdsopl.robot36;

public class JB60ChromaResponseDump {
	private static final int sampleRate = 48000;
	// must match JB60's own private constants exactly (JB60.java)
	private static final double syncPulseSeconds = 0.02;
	private static final double syncPorchSeconds = 0.00208;
	private static final double slotSeconds = 0.00019;
	private static final int[] segmentSlots = {640, 144, 224, 176};
	private static final int[] segmentOffsets = {0, 640, 784, 1008};
	private static final int L = 0, D = 1, CR = 2, CB = 3;
	// Demodulator's own constants (Demodulator.java)
	private static final double syncPulseFrequency = 1200;
	private static final double porchFrequency = 1500; // Demodulator's syncPorchFrequency
	private static final double blackFrequency = 1500;
	private static final double whiteFrequency = 2300;
	private static final double amplitude = 0.9;

	private static double freqForLevel(double level0to1) {
		// inverts JB60.freqToLevel()/Demodulator.normalizeFrequency(): level 0..1 -> 1500..2300 Hz
		return blackFrequency + level0to1 * (whiteFrequency - blackFrequency);
	}

	// Trailing-sync layout (matches modejb60.cpp's header comment: "bp, L, D, Cr, Cb, fp, sync"):
	// the sync pulse for line k comes *before* line k's own porch, i.e. right after line (k-1)'s
	// video content ends -- JB60.decodeScanLine()'s syncPulseIndex parameter names the sample where
	// the porch begins, with beginSamples/endSamples measured forward from there, so the reference
	// sync pulse itself occupies the syncSamples immediately *before* that index, not after it.
	//
	// Phase-continuous FM synthesis of one full JB60 line (sync pulse, porch, L/D/Cr/Cb slots) at
	// the given per-segment, per-slot levels (0..1). `porchStart` is where the porch begins (the
	// syncPulseIndex value this line will be decoded with); the sync pulse is written into
	// [porchStart-syncSamples, porchStart). Returns the running phase (radians) so the next line
	// (whose own sync pulse starts exactly at this line's Cb end -- no separate front-porch gap,
	// confirmed by construction: syncSamples + endOfVideoSamples == lineSamples() exactly) can
	// continue without a discontinuity.
	private static double synthesizeLine(float[] audio, int porchStart, double phase, double[][] levels) {
		double twoPi = 2 * Math.PI;
		int syncSamples = (int) Math.round(syncPulseSeconds * sampleRate);
		for (int n = porchStart - syncSamples; n < porchStart; ++n) {
			phase += twoPi * syncPulseFrequency / sampleRate;
			audio[n] = (float) (amplitude * Math.sin(phase));
		}
		int porchSamples = (int) Math.round(syncPorchSeconds * sampleRate);
		for (int n = porchStart; n < porchStart + porchSamples; ++n) {
			phase += twoPi * porchFrequency / sampleRate;
			audio[n] = (float) (amplitude * Math.sin(phase));
		}
		// L/D/Cr/Cb: continuous-time slot boundaries (not accumulated per-slot rounding) exactly
		// mirror JB60Test.fillSegment()'s own begin/end formula, so slot centers land where JB60's
		// own slotCenters[] expects them with no cumulative drift over 640+ slots.
		for (int seg = 0; seg < 4; ++seg) {
			int n = segmentSlots[seg];
			for (int s = 0; s < n; ++s) {
				int begin = (int) Math.round((syncPorchSeconds + (segmentOffsets[seg] + s) * slotSeconds) * sampleRate);
				int end = (int) Math.round((syncPorchSeconds + (segmentOffsets[seg] + s + 1) * slotSeconds) * sampleRate);
				double freq = freqForLevel(levels[seg][s]);
				for (int n2 = porchStart + begin; n2 < porchStart + end; ++n2) {
					phase += twoPi * freq / sampleRate;
					audio[n2] = (float) (amplitude * Math.sin(phase));
				}
			}
		}
		return phase;
	}

	private static int lineSamples() {
		double seconds = syncPulseSeconds + syncPorchSeconds;
		for (int n : segmentSlots) seconds += n * slotSeconds;
		return (int) Math.round(seconds * sampleRate);
	}

	// fill every slot of a segment with the same level
	private static void fillFlat(double[] seg, double level) {
		java.util.Arrays.fill(seg, level);
	}

	// one saturated slot near the middle, everything else at baseline
	private static void fillImpulse(double[] seg, double baseline, double peak) {
		fillFlat(seg, baseline);
		seg[seg.length / 2] = peak;
	}

	// first half at low, second half at high (a slot-domain step -- no luma-guided TX nonlinearity
	// to fight here, unlike mmsstv-linux, so a plain half/half split is a clean, exact known input)
	private static void fillStep(double[] seg, double low, double high) {
		for (int i = 0; i < seg.length; ++i)
			seg[i] = i < seg.length / 2 ? low : high;
	}

	private static double[][] baseFlatLine() {
		double[][] levels = new double[4][];
		for (int seg = 0; seg < 4; ++seg) {
			levels[seg] = new double[segmentSlots[seg]];
			fillFlat(levels[seg], 128.0 / 255.0);
		}
		return levels;
	}

	// finds the porch-start index (the 50% crossing from the sync tone's deeply negative demod
	// level up to the porch's -1.0 level) in the demodulated buffer, searching from `from` --
	// mirrors mmsstv-linux's own calibrateDelay() 50%-crossing technique, and is what makes this
	// measurement robust to Demodulator's own group delay without having to compute it analytically
	// (the real Decoder locates syncPulseIndex the same self-consistent way, via its own trigger).
	private static double findPorchStart(float[] demod, int from) {
		double syncLevel = 2 * (syncPulseFrequency - 1900) / 800.0; // Demodulator.normalizeFrequency(1200)
		double porchLevel = 2 * (porchFrequency - 1900) / 800.0;    // normalizeFrequency(1500) = -1.0
		double threshold = (syncLevel + porchLevel) / 2;
		for (int n = from + 1; n < demod.length; ++n) {
			if (demod[n] >= threshold && demod[n - 1] < threshold) {
				double frac = (threshold - demod[n - 1]) / (demod[n] - demod[n - 1]);
				return (n - 1) + frac;
			}
		}
		throw new RuntimeException("porch start not found");
	}

	private static void dump(String tag, int[] arr) {
		StringBuilder sb = new StringBuilder(tag).append(':');
		for (int v : arr) sb.append(' ').append(v);
		System.out.println(sb);
	}

	// runs one two-line (lead-in + measured) audio buffer through the real Demodulator and JB60,
	// returning the measured line's post-demod, pre-deconvolution Cr/Cb slot arrays
	private static int[][] measure(double[][] levels) {
		int oneLine = lineSamples();
		int syncSamples = (int) Math.round(syncPulseSeconds * sampleRate);
		float[] audio = new float[2 * oneLine + sampleRate / 10]; // +100ms tail so decodeScanLine's endSamples window never runs off the end
		double phase = 0;
		// porchStart(line 0) = syncSamples, leaving room for its own leading sync pulse at [0,syncSamples)
		double phaseAfterLeadIn = synthesizeLine(audio, syncSamples, phase, baseFlatLine()); // lead-in: lets Demodulator's Kaiser FIR settle
		synthesizeLine(audio, syncSamples + oneLine, phaseAfterLeadIn, levels);              // the line actually measured
		Demodulator demod = new Demodulator(sampleRate);
		demod.process(audio, 0);
		// locate the *second* sync-tone-to-porch transition (the measured line's own), well past
		// the lead-in line so the Kaiser filter's ~2ms settle time is long since over
		double firstPorch = findPorchStart(audio, 0);
		double secondPorch = findPorchStart(audio, (int) firstPorch + oneLine / 2);
		int syncPulseIndex = (int) Math.round(secondPorch);
		if (System.getenv("JB60_DEBUG") != null) {
			System.err.println("oneLine=" + oneLine + " firstPorch=" + firstPorch + " secondPorch=" + secondPorch + " syncPulseIndex=" + syncPulseIndex);
			StringBuilder sb = new StringBuilder("around CR start: ");
			for (int n = syncPulseIndex + 7240; n < syncPulseIndex + 7265; ++n) sb.append(String.format("%.2f ", audio[n]));
			System.err.println(sb);
			sb = new StringBuilder("around CB start: ");
			for (int n = syncPulseIndex + 9283; n < syncPulseIndex + 9308; ++n) sb.append(String.format("%.2f ", audio[n]));
			System.err.println(sb);
			sb = new StringBuilder("around porch/L start: ");
			for (int n = syncPulseIndex - 5; n < syncPulseIndex + 20; ++n) sb.append(String.format("%.2f ", audio[n]));
			System.err.println(sb);
		}

		JB60 mode = new JB60(sampleRate);
		PixelBuffer pixelBuffer = new PixelBuffer(800, 4);
		float[] scratchBuffer = new float[(int) Math.round(1.1 * sampleRate)];
		mode.decodeScanLine(pixelBuffer, scratchBuffer, audio, 640, syncPulseIndex, mode.getScanLineSamples(), 0);
		return new int[][]{mode.lastCr().clone(), mode.lastCb().clone()};
	}

	// Cr and Cb are adjacent in transmission time (segment order L,D,CR,CB): if both were driven
	// with a step in the same line, CR ending at its high plateau would walk straight into CB
	// starting at its own low plateau -- a real transmitted discontinuity that would bleed into
	// CB's *head* (the mirror image of the already-masked tail-into-next-line-sync artifact). Since
	// this app has no encoder to fight, the simplest fix is to just not create that discontinuity:
	// measure Cr and Cb in separate lines, keeping the untested channel flat at the tested
	// channel's own low/baseline level so every segment boundary the tested channel touches is
	// either flat-into-flat (clean) or ends at its own genuine high plateau (an unavoidable, already
	// -masked tail artifact into whatever follows, same as the existing Cb->next-sync case).
	private static int[][] measureChannel(int channel, double baseline, double peak) {
		double[][] flatLine = baseFlatLine();
		double[][] impulseLine = baseFlatLine();
		fillImpulse(impulseLine[channel], baseline, peak);
		double[][] stepLine = baseFlatLine();
		fillStep(stepLine[channel], baseline, peak);
		return new int[][]{measure(flatLine)[channel - CR], measure(impulseLine)[channel - CR], measure(stepLine)[channel - CR]};
	}

	public static void main(String[] args) {
		String channel = args.length > 0 ? args[0] : "both";
		double baseline = 128.0 / 255.0, peak = 255.0 / 255.0;

		if (channel.equals("cr") || channel.equals("both")) {
			int[][] cr = measureChannel(CR, baseline, peak);
			dump("cr_flat", cr[0]);
			dump("cr_impulse", cr[1]);
			dump("cr_step", cr[2]);
		}
		if (channel.equals("cb") || channel.equals("both")) {
			int[][] cb = measureChannel(CB, baseline, peak);
			dump("cb_flat", cb[0]);
			dump("cb_impulse", cb[1]);
			dump("cb_step", cb[2]);
		}
	}
}
