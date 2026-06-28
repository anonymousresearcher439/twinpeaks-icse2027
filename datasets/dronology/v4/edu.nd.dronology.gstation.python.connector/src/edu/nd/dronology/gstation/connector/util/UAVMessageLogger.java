package edu.nd.dronology.gstation.connector.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.stream.Collectors;

import edu.nd.dronology.gstation.connector.messages.AbstractUAVMessage;



public class UAVMessageLogger {

	private static final boolean ACTIVE = true;
	private static final String LOG_FILE_PATH = "/home/uav/messagelog/messages.log";
	private static List<String> messageList = Collections.synchronizedList(new ArrayList<>());

	static {
		init();
	}
	
	public static void init() {
		if (!ACTIVE) {
			return;
		}
		Timer timer = new Timer();
		Date date = new Date();
		long t = date.getTime();

		timer.schedule(new WriterTask(), 30000, 30000);

	}

	public static synchronized void logMessage(AbstractUAVMessage<?> message) {

		String messageString = message.toString();
		messageList.add(messageString);
	}

	static class WriterTask extends TimerTask {

		@Override
		public void run() {
			try {
				List<String> messageWrite;

				synchronized (messageList) {
					messageWrite = new ArrayList<>(messageList);
					messageList.clear();
				}

				String staticString = messageWrite.stream()
						.collect(Collectors.joining(System.getProperty("line.separator")))
						+ System.getProperty("line.separator");

				try {

					Files.write(Paths.get(LOG_FILE_PATH), staticString.getBytes(), StandardOpenOption.CREATE,
							StandardOpenOption.APPEND);

				} catch (IOException e) {
					e.printStackTrace();
				}

			} catch (Throwable t) {
				t.printStackTrace();
			}
		}
	}

}