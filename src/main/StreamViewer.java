package main;

import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.awt.Toolkit;
import java.awt.Color;
import java.awt.Dimension;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class StreamViewer {

	public static boolean running = false;

	public static final int CARGO_CAMERA = 0;
	public static final int HATCH_CAMERA = 1;
	public int currentCamera = HATCH_CAMERA;
	public VideoCapture stream;
	public VideoCapture hatchStream, cargoStream;

	private BufferedImage image;

	int streamWidth, streamHeight;
	public static int streamPixelWidth  = 416;
	public static int streamPixelHeight = 240;

	public double[] contour_left  = new double[6];
	public double[] contour_right = new double[6];

	public double[] cargoX = {};
	public double[] cargoY = {};
	public double[] cargoR = {};

	public static void main(String[] args) {
		//System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		//System.loadLibrary("opencv_ffmpeg401_64");
		StreamViewer streamViewer = new StreamViewer();
	}

	public StreamViewer() {
		final JFrame frame = new JFrame("Camera");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setUndecorated(true);
		frame.getContentPane().setBackground(new Color(42, 42, 42));
		frame.setResizable(false);
		frame.setLocation(0, 0);
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
		frame.setSize((int)screenSize.getWidth(), (int)screenSize.getHeight() - 200);
		streamHeight = (int)screenSize.getHeight() - 300;
		streamWidth = (int)(streamHeight * 1.734);
		JLabel label = new JLabel("", JLabel.CENTER);
		frame.getContentPane().add(label);
		frame.validate();
		frame.setVisible(true);

		CrosshairPipeline crosshairPipeline = new CrosshairPipeline();

		hatchStream = new VideoCapture();
		cargoStream = new VideoCapture();
		stream = hatchStream;
		connect();

		NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
		ntinst.startClientTeam(4509);
		ntinst.setUpdateRate(0.02);

		ntinst.getTable("vision").addEntryListener("source", (table, key, entry, value, flags) -> {
			currentCamera = ((int)value.getDouble() % 2);
			if(currentCamera == HATCH_CAMERA)
				stream = hatchStream;
			else if(currentCamera == CARGO_CAMERA)
				stream = cargoStream;
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		ntinst.getTable("vision/targets").addEntryListener("contour_left", (table, key, entry, value, flags) -> {
			contour_left = value.getDoubleArray();
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		ntinst.getTable("vision/targets").addEntryListener("contour_right", (table, key, entry, value, flags) -> {
			contour_right = value.getDoubleArray();
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		ntinst.getTable("vision/cargo").addEntryListener("x", (table, key, entry, value, flags) -> {
			cargoX = value.getDoubleArray();
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		ntinst.getTable("vision/cargo").addEntryListener("y", (table, key, entry, value, flags) -> {
			cargoY = value.getDoubleArray();
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		ntinst.getTable("vision/cargo").addEntryListener("r", (table, key, entry, value, flags) -> {
			cargoR = value.getDoubleArray();
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		(new Thread(() -> {
			Mat mat = new Mat();
			BufferedImage image = null;
			Timer timer = new Timer();
			timer.start();
			while(true) {
				try {
					if(!stream.read(mat)) {
						connect();
					}
					if(streamPixelWidth == 0 || streamPixelHeight == 0) {
						streamPixelWidth = mat.width();
						streamPixelHeight = mat.height();
					}
					crosshairPipeline.process(mat);
					mat = crosshairPipeline.cvCrosshairHOutput();
					drawCargo(mat);
					drawTargets(mat);
					if(mat.width() < 1 || mat.height() < 1) continue;
					image = createAwtImage(mat);
					image = toBufferedImage(image.getScaledInstance(streamWidth, streamHeight, Image.SCALE_SMOOTH));
					label.setIcon(new ImageIcon(image));
				} catch(Exception e) {
					System.err.println(e.getMessage());
					continue;
				}
			}
		})).start();

		while(true) {
			try {
				Thread.sleep(10000);
			} catch(Exception e) {}
		}
	}

	public void connect() {
		do {
			System.out.println("trying to connect...");
			hatchStream.open("http://10.45.9.3:118" + (HATCH_CAMERA + 1) + "/stream.mjpg");
			cargoStream.open("http://10.45.9.3:118" + (CARGO_CAMERA + 1) + "/stream.mjpg");
		} while(!stream.isOpened());
		System.out.println("connected.");
	}

	// https://stackoverflow.com/questions/22284823/opencv-output-using-mat-object-in-jpanel
	public static BufferedImage createAwtImage(Mat mat) {
		int type = 0;
		if(mat.channels() == 1) {
			type = BufferedImage.TYPE_BYTE_GRAY;
		} else if(mat.channels() == 3) {
			type = BufferedImage.TYPE_3BYTE_BGR;
		} else if(mat.channels() == 4) {
			type = BufferedImage.TYPE_4BYTE_ABGR;
		} else {
			return null;
		}

		BufferedImage image = new BufferedImage(mat.width(), mat.height(), type);
		WritableRaster raster = image.getRaster();
		DataBufferByte dataBuffer = (DataBufferByte) raster.getDataBuffer();
		byte[] data = dataBuffer.getData();
		mat.get(0, 0, data);

		return image;
	}

	// https://stackoverflow.com/questions/13605248/java-converting-image-to-bufferedimage/13605411#13605411
	public static BufferedImage toBufferedImage(Image img) {
		if(img instanceof BufferedImage) {
			return (BufferedImage)img;
		}

		BufferedImage bimage = new BufferedImage(img.getWidth(null), img.getHeight(null), BufferedImage.TYPE_INT_ARGB);

		Graphics2D bGr = bimage.createGraphics();
		bGr.drawImage(img, 0, 0, null);
		bGr.dispose();

		return bimage;
	}

	public void drawTargets(Mat mat) {
		if(contour_left[0] != 0) {
			RotatedRect l = new RotatedRect(new Point(contour_left[0], contour_left[1]), new Size(contour_left[2], contour_left[3]), contour_left[4]);
			drawMinAreaRect(mat, l, new Scalar(255, 0, 0));
		}
		if(contour_right[0] != 0) {
			RotatedRect r = new RotatedRect(new Point(contour_right[0], contour_right[1]), new Size(contour_right[2], contour_right[3]), contour_right[4]);
			drawMinAreaRect(mat, r, new Scalar(0, 0, 255));
		}
		if(contour_left[0] != 0 && contour_right[0] != 0) {
			double avg = (contour_left[0] + contour_right[0]) / 2;
			Imgproc.line(mat, new Point(avg, 0), new Point(avg, streamPixelHeight), new Scalar(100, 0, 100), 1);
		}
	}

	public static void drawMinAreaRect(Mat mat, RotatedRect rect, Scalar color) {
		if(rect == null) return;
		Point[] vertices = new Point[4];
		rect.points(vertices);
		for (int j = 0; j < 4; j++) {
			Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], color, 2);
		}
	}

	public void drawCargo(Mat mat) {
		if(cargoX.length != cargoY.length || cargoY.length != cargoR.length) return;
		for(int i = 0; i < cargoX.length; i++) {
			Imgproc.circle(mat, new Point(cargoX[i], cargoY[i]), (int)cargoR[i], new Scalar(0, 100 + (i * 10), 0), 1);
		}
	}

}