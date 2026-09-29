/*
Cross-repo validation bridge for JB60 chroma deconvolution: decodes a WAV file (produced by
mmsstv-linux's tests/jb60_loopback/loopback.cpp --wav --vis) through the real Decoder (real
Demodulator + real sync/VIS detection + JB60.decodeScanLine()) and dumps the resulting image as a
binary PPM (P6), so mmsstv-linux's own `loopback --compare a.ppm b.ppm` can report PSNR/SSIM
against the source image or between an off/on pair, with no new comparison code needed on this
side. PPM, not PNG: this app's Android Gradle Plugin unit-test compile classpath excludes
javax.imageio/java.awt (Android-restricted, even for a plain JVM-side tool in src/test), so a
hand-rolled, compression-free format is the path of least resistance -- Qt's QImage (what
`loopback --compare` uses) reads PPM natively, no plugin needed (verified).

Not used by the app itself. Usage:

  java -cp <classes> xdsopl.robot36.JB60WavBridge in.wav --deconv=off out_off.ppm
  java -cp <classes> xdsopl.robot36.JB60WavBridge in.wav --deconv=on  out_on.ppm

Modified 2026 Jason <jason@weisb.net>
*/

package xdsopl.robot36;

import java.io.BufferedOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

public class JB60WavBridge {
	// binary PPM (P6): header, then row-major RGB triplets, alpha discarded (imageBuffer.pixels is
	// ARGB, 0xff000000-opaque per combineColors()-equivalent code -- see JB60.emitPair())
	private static void writePpm(String path, int[] argbPixels, int width, int height) throws IOException {
		try (OutputStream out = new BufferedOutputStream(new FileOutputStream(path))) {
			out.write(("P6\n" + width + " " + height + "\n255\n").getBytes(StandardCharsets.US_ASCII));
			byte[] row = new byte[width * 3];
			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					int p = argbPixels[y * width + x];
					row[x * 3] = (byte) ((p >> 16) & 0xff);
					row[x * 3 + 1] = (byte) ((p >> 8) & 0xff);
					row[x * 3 + 2] = (byte) (p & 0xff);
				}
				out.write(row);
			}
		}
	}


	public static void main(String[] args) throws Exception {
		if (args.length < 3) {
			System.err.println("usage: JB60WavBridge in.wav --deconv=on|off out.png");
			System.exit(1);
		}
		String wavPath = args[0];
		boolean deconv = args[1].equals("--deconv=on");
		String outPath = args[2];

		WavFile wav = WavFile.read(wavPath);

		// same buffer sizes MainActivity uses (app/src/main/java/xdsopl/robot36/MainActivity.java),
		// so Decoder/copyLines() behave exactly as they do in production
		PixelBuffer scopeBuffer = new PixelBuffer(800, 2 * 1280);
		PixelBuffer imageBuffer = new PixelBuffer(800, 616);
		imageBuffer.line = -1;

		Decoder decoder = new Decoder(scopeBuffer, imageBuffer, "raw", wav.sampleRate);
		decoder.setChromaDeconvolutionEnabled(deconv);

		int chunk = wav.sampleRate / 10;
		boolean finished = false;
		for (int i = 0; i < wav.samples.length && !finished; i += chunk) {
			int len = Math.min(chunk, wav.samples.length - i);
			float[] buf = new float[len];
			System.arraycopy(wav.samples, i, buf, 0, len);
			decoder.process(buf, 0);
			if (imageBuffer.line == imageBuffer.height)
				finished = true;
		}
		if (!finished) {
			System.err.println("warning: WAV ended before a full image was decoded (imageBuffer.line=" + imageBuffer.line + "/" + imageBuffer.height + ")");
		}

		writePpm(outPath, imageBuffer.pixels, imageBuffer.width, imageBuffer.height);
		System.err.println("wrote " + outPath + " (" + imageBuffer.width + "x" + imageBuffer.height + ", deconv=" + deconv + ")");
	}
}
