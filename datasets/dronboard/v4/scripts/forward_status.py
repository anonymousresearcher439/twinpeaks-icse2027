#!/usr/bin/env python3
"""Bridge status messages between two MQTT brokers.

Subscribes to `status_message` on a SOURCE (mTLS) broker and republishes
payloads unchanged to `update_drone` on a DEST (non‑TLS) broker.

Configuration precedence (lowest -> highest):
  hardcoded defaults < environment variables < CLI arguments.

Environment variables:
  SOURCE_MQTT_HOST, SOURCE_MQTT_PORT
  SOURCE_MQTT_CA_CERT, SOURCE_MQTT_CLIENT_CERT, SOURCE_MQTT_PRIVATE_KEY
  SOURCE_MQTT_CLIENT_ID
  DEST_MQTT_HOST, DEST_MQTT_PORT, DEST_MQTT_CLIENT_ID
  SOURCE_TOPIC (default: status_message)
  DEST_TOPIC   (default: update_drone)

Run example:
  python scripts/forward_status.py \
	--source-host a3xxxx-ats.iot.us-east-2.amazonaws.com \
	--source-ca /etc/dr/CAs.crt --source-cert /etc/dr/public.crt --source-key /etc/dr/private.key \
	--dest-host mqtt --dest-port 1883

Graceful shutdown with Ctrl+C.
"""

from __future__ import annotations

import argparse
import logging
import os
import queue
import signal
import ssl
import sys
import threading
import time
from typing import Optional

import paho.mqtt.client as mqtt


def env_int(key: str, default: int) -> int:
	try:
		return int(os.getenv(key, default))
	except ValueError:
		return default


class MQTTBridge:
	"""Bridge between source (TLS) and destination (plain) MQTT brokers."""

	def __init__(
		self,
		source_host: str,
		source_port: int,
		dest_host: str,
		dest_port: int,
		source_topic: str = "status_message",
		dest_topic: str = "update_drone",
		source_ca: Optional[str] = None,
		source_cert: Optional[str] = None,
		source_key: Optional[str] = None,
		source_client_id: Optional[str] = None,
		dest_client_id: Optional[str] = None,
		qos: int = 0,
		max_queue: int = 1000,
	) -> None:
		self.log = logging.getLogger(self.__class__.__name__)
		self.source_topic = source_topic
		self.dest_topic = dest_topic
		self.qos = qos
		self._stop = threading.Event()
		self._replay_queue: "queue.Queue[tuple[str, bytes, int]]" = queue.Queue(maxsize=max_queue)

		# Source client (TLS / mTLS)
		self.source_client = mqtt.Client(client_id=source_client_id or "status-forwarder-src")
		if source_ca and source_cert and source_key:
			self.log.info("Configuring source client for mutual TLS")
			self.source_client.tls_set(
				ca_certs=source_ca,
				certfile=source_cert,
				keyfile=source_key,
				tls_version=ssl.PROTOCOL_TLS,
			)
		elif source_ca:  # CA only / server-auth TLS
			self.log.info("Configuring source client for TLS (server-auth only)")
			self.source_client.tls_set(ca_certs=source_ca, tls_version=ssl.PROTOCOL_TLS)

		self.source_client.on_connect = self._on_source_connect
		self.source_client.on_message = self._on_source_message
		self.source_client.on_disconnect = self._on_source_disconnect

		# Destination client (plain)
		self.dest_client = mqtt.Client(client_id=dest_client_id or "status-forwarder-dest")
		self.dest_client.on_connect = self._on_dest_connect
		self.dest_client.on_disconnect = self._on_dest_disconnect

		self.source_host = source_host
		self.source_port = source_port
		self.dest_host = dest_host
		self.dest_port = dest_port

		# Start network loops
		self.source_client.loop_start()
		self.dest_client.loop_start()

	# ---- Source callbacks ----
	def _on_source_connect(self, client, _userdata, _flags, rc):
		if rc == 0:
			self.log.info(f"Source connected to {self.source_host}:{self.source_port}")
			client.subscribe(self.source_topic, qos=self.qos)
			self.log.info(f"Subscribed to source topic '{self.source_topic}'")
		else:
			self.log.error(f"Source connect failed rc={rc}")

	def _on_source_message(self, _client, _userdata, msg):
		payload = msg.payload  # raw bytes
		if not self.dest_client.is_connected():
			# Buffer temporarily (drop oldest if full)
			if self._replay_queue.full():
				try:
					_ = self._replay_queue.get_nowait()
				except queue.Empty:
					pass
			try:
				self._replay_queue.put_nowait((self.dest_topic, payload, self.qos))
			except queue.Full:
				self.log.warning("Replay queue full; dropping message")
			return
		self.dest_client.publish(self.dest_topic, payload, qos=self.qos)

	def _on_source_disconnect(self, _client, _userdata, rc):
		if rc != 0:
			self.log.warning("Unexpected source disconnect; will auto-reconnect")

	# ---- Destination callbacks ----
	def _on_dest_connect(self, _client, _userdata, rc, properties):
		if rc == 0:
			self.log.info(f"Destination connected to {self.dest_host}:{self.dest_port}")
			# Flush buffered messages
			flushed = 0
			while not self._replay_queue.empty():
				try:
					topic, payload, qos = self._replay_queue.get_nowait()
				except queue.Empty:
					break
				self.dest_client.publish(topic, payload, qos=qos)
				flushed += 1
			if flushed:
				self.log.info(f"Flushed {flushed} buffered messages to destination")
		else:
			self.log.error(f"Destination connect failed rc={rc}")

	def _on_dest_disconnect(self, _client, _userdata, rc):
		if rc != 0:
			self.log.warning("Unexpected destination disconnect; will auto-reconnect")

	# ---- Public API ----
	def start(self):
		self._connect_with_backoff()

	def _connect_with_backoff(self):
		backoff = 1
		while not self._stop.is_set():
			try:
				if not self.source_client.is_connected():
					self.source_client.connect(self.source_host, self.source_port, keepalive=60)
				if not self.dest_client.is_connected():
					self.dest_client.connect(self.dest_host, self.dest_port, keepalive=60)
				# Give network loop time to process connect; then sleep longer
				time.sleep(2)
				if self.source_client.is_connected() and self.dest_client.is_connected():
					backoff = 1  # reset after success
				else:
					raise RuntimeError("One or both clients not yet connected")
			except Exception as e:  # noqa: BLE001
				self.log.warning(f"Connect attempt failed: {e}; retry in {backoff}s")
				time.sleep(backoff)
				backoff = min(backoff * 2, 30)
			else:
				# Normal run loop (sleep in small increments for responsiveness)
				while not self._stop.is_set():
					time.sleep(0.5)
				break

	def stop(self):
		self._stop.set()
		self.source_client.loop_stop()
		self.dest_client.loop_stop()
		try:
			if self.source_client.is_connected():
				self.source_client.disconnect()
		finally:
			if self.dest_client.is_connected():
				self.dest_client.disconnect()


def parse_args() -> argparse.Namespace:
	p = argparse.ArgumentParser(description="Forward status_message to update_drone between brokers")
	p.add_argument("--source-host", default=os.getenv("SOURCE_MQTT_HOST", "localhost"))
	p.add_argument("--source-port", type=int, default=env_int("SOURCE_MQTT_PORT", 8883))
	p.add_argument("--source-ca", default=os.getenv("SOURCE_MQTT_CA_CERT"))
	p.add_argument("--source-cert", default=os.getenv("SOURCE_MQTT_CLIENT_CERT"))
	p.add_argument("--source-key", default=os.getenv("SOURCE_MQTT_PRIVATE_KEY"))
	p.add_argument("--source-client-id", default=os.getenv("SOURCE_MQTT_CLIENT_ID"))
	p.add_argument("--dest-host", default=os.getenv("DEST_MQTT_HOST", "mqtt"))
	p.add_argument("--dest-port", type=int, default=env_int("DEST_MQTT_PORT", 1883))
	p.add_argument("--dest-client-id", default=os.getenv("DEST_MQTT_CLIENT_ID"))
	p.add_argument("--source-topic", default=os.getenv("SOURCE_TOPIC", "status_message"))
	p.add_argument("--dest-topic", default=os.getenv("DEST_TOPIC", "update_drone"))
	p.add_argument("--qos", type=int, default=0, choices=[0, 1])  # QoS 2 not supported by AWS IoT
	p.add_argument("--log-level", default=os.getenv("LOG_LEVEL", "INFO"))
	return p.parse_args()


def setup_logging(level: str):
	logging.basicConfig(
		level=getattr(logging, level.upper(), logging.INFO),
		format="%(asctime)s %(levelname)s %(name)s: %(message)s",
	)


def main():
	args = parse_args()
	setup_logging(args.log_level)

	bridge = MQTTBridge(
		source_host=args.source_host,
		source_port=args.source_port,
		dest_host=args.dest_host,
		dest_port=args.dest_port,
		source_topic=args.source_topic,
		dest_topic=args.dest_topic,
		source_ca=args.source_ca,
		source_cert=args.source_cert,
		source_key=args.source_key,
		source_client_id=args.source_client_id,
		dest_client_id=args.dest_client_id,
		qos=args.qos,
	)

	stop_event = threading.Event()

	def _signal_handler(_sig, _frame):
		logging.getLogger("forward_status").info("Shutdown requested")
		stop_event.set()
		bridge.stop()

	signal.signal(signal.SIGINT, _signal_handler)
	signal.signal(signal.SIGTERM, _signal_handler)

	bridge.start()
	while not stop_event.is_set():
		time.sleep(0.2)

	return 0


if __name__ == "__main__":  # pragma: no cover
	sys.exit(main())
