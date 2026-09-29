/*
Minimal PCM16 mono WAV reader, for JB60WavBridge -- reads WAV files produced by mmsstv-linux's
tests/jb60_loopback/loopback.cpp (--wav). Not used by the app itself.

Modified 2026 Jason <jason@weisb.net>
*/

package xdsopl.robot36;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.EOFException;
import java.io.FileInputStream;
import java.io.IOException;

public class WavFile {
	public int sampleRate;
	public float[] samples;

	private static int readLE16(DataInputStream in) throws IOException {
		int lo = in.readUnsignedByte(), hi = in.readUnsignedByte();
		return lo | (hi << 8);
	}

	private static short readLE16Signed(DataInputStream in) throws IOException {
		return (short) readLE16(in);
	}

	private static int readLE32(DataInputStream in) throws IOException {
		int b0 = in.readUnsignedByte(), b1 = in.readUnsignedByte(), b2 = in.readUnsignedByte(), b3 = in.readUnsignedByte();
		return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
	}

	private static String readTag(DataInputStream in) throws IOException {
		byte[] tag = new byte[4];
		in.readFully(tag);
		return new String(tag, "US-ASCII");
	}

	// scales the same way MainActivity's real audio-read callback does (PCM16 -> normalized float)
	private static final float scale = .000030517578125f;

	public static WavFile read(String path) throws IOException {
		WavFile w = new WavFile();
		try (DataInputStream in = new DataInputStream(new BufferedInputStream(new FileInputStream(path)))) {
			if (!readTag(in).equals("RIFF")) throw new IOException("not a RIFF file: " + path);
			readLE32(in); // RIFF chunk size, unused
			if (!readTag(in).equals("WAVE")) throw new IOException("not a WAVE file: " + path);
			boolean sawFmt = false, sawData = false;
			int channels = 1, bitsPerSample = 16;
			while (!sawData) {
				String id;
				try {
					id = readTag(in);
				} catch (EOFException e) {
					break;
				}
				int size = readLE32(in);
				if (id.equals("fmt ")) {
					readLE16(in); // format tag, assumed PCM
					channels = readLE16(in);
					w.sampleRate = readLE32(in);
					readLE32(in); // byte rate
					readLE16(in); // block align
					bitsPerSample = readLE16(in);
					int consumed = 16;
					if (size > consumed) in.skipBytes(size - consumed);
					if ((size & 1) != 0) in.skipBytes(1);
					sawFmt = true;
				} else if (id.equals("data")) {
					if (!sawFmt) throw new IOException("data chunk before fmt chunk: " + path);
					if (channels != 1 || bitsPerSample != 16)
						throw new IOException("only mono 16-bit PCM is supported, got " + channels + "ch/" + bitsPerSample + "bit: " + path);
					int n = size / 2;
					w.samples = new float[n];
					for (int i = 0; i < n; ++i)
						w.samples[i] = scale * readLE16Signed(in);
					if ((size & 1) != 0) in.skipBytes(1);
					sawData = true;
				} else {
					in.skipBytes(size + (size & 1));
				}
			}
			if (!sawData) throw new IOException("no data chunk found: " + path);
		}
		return w;
	}
}
