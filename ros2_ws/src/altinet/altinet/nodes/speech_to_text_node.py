"""ROS 2 node that streams speech-to-text transcripts."""
from __future__ import annotations

import re
from queue import Empty, Queue
from typing import Callable, Iterable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import speech_recognition as sr
except ImportError as exc:  # pragma: no cover - dependency managed at runtime
    sr = None  # type: ignore[assignment]
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class SpeechToTextNode(Node):
    """Continuously capture audio, transcribe it, and publish each sentence."""

    def __init__(self) -> None:
        super().__init__('speech_to_text_node')

        if sr is None:
            raise RuntimeError(
                'speech_recognition is required for SpeechToTextNode to run. '
                'Install it with `pip install SpeechRecognition`.'
            ) from _IMPORT_ERROR

        self._result_queue: Queue[str] = Queue()
        self._stop_listening: Optional[Callable[[bool], None]] = None

        topic_name = str(
            self.declare_parameter('output_topic', 'speech_transcript').value
        ).strip()
        qos_depth = int(self.declare_parameter('queue_size', 10).value)
        self._publisher = self.create_publisher(String, topic_name, qos_depth)

        self._engine = (
            str(self.declare_parameter('engine', 'google').value).strip().lower()
        )
        self._language = str(
            self.declare_parameter('language_code', 'en-US').value
        ).strip()
        self._phrase_time_limit = self._coerce_optional_float(
            self.declare_parameter('phrase_time_limit', 10.0).value
        )
        self._energy_threshold = self._coerce_optional_float(
            self.declare_parameter('energy_threshold', 300.0).value
        )
        self._dynamic_energy_threshold = bool(
            self.declare_parameter('dynamic_energy_threshold', True).value
        )
        self._ambient_duration = float(
            self.declare_parameter('ambient_noise_duration', 1.0).value
        )
        self._pause_threshold = float(
            self.declare_parameter('pause_threshold', 0.8).value
        )
        self._non_speaking_duration = float(
            self.declare_parameter('non_speaking_duration', 0.5).value
        )
        self._microphone_index = int(
            self.declare_parameter('microphone_index', -1).value
        )
        self._recognizer_api_key = self.declare_parameter('api_key', '').value
        if isinstance(self._recognizer_api_key, (bytes, bytearray)):
            self._recognizer_api_key = self._recognizer_api_key.decode('utf-8')
        self._recognizer_api_key = (
            str(self._recognizer_api_key).strip() if self._recognizer_api_key else None
        )

        self._recognizer = sr.Recognizer()
        self._recognizer.dynamic_energy_threshold = self._dynamic_energy_threshold
        self._recognizer.pause_threshold = self._pause_threshold
        self._recognizer.non_speaking_duration = self._non_speaking_duration
        if self._energy_threshold is not None:
            self._recognizer.energy_threshold = self._energy_threshold

        self._microphone = sr.Microphone(
            device_index=self._microphone_index
            if self._microphone_index >= 0
            else None
        )

        self._start_background_listener()
        self._publish_timer = self.create_timer(0.1, self._publish_pending_results)

    # ------------------------------------------------------------------
    # Lifecycle helpers
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        if self._stop_listening is not None:
            try:
                self._stop_listening(wait_for_stop=False)
            except Exception:  # pragma: no cover - best effort cleanup
                self.get_logger().warning('Failed to stop background listener cleanly')
        return super().destroy_node()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _start_background_listener(self) -> None:
        with self._microphone as source:
            if self._ambient_duration and self._ambient_duration > 0:
                self.get_logger().info('Calibrating microphone for ambient noise…')
                self._recognizer.adjust_for_ambient_noise(
                    source, duration=self._ambient_duration
                )

        self.get_logger().info('Starting continuous speech recognition')
        self._stop_listening = self._recognizer.listen_in_background(
            self._microphone,
            self._handle_audio,
            phrase_time_limit=self._phrase_time_limit,
        )

    def _handle_audio(self, recognizer: sr.Recognizer, audio: sr.AudioData) -> None:
        try:
            transcript = self._transcribe_audio(recognizer, audio)
        except sr.UnknownValueError:
            self.get_logger().debug('Speech recognition could not understand audio')
            return
        except sr.RequestError as exc:
            self.get_logger().error(f'Speech recognition request failed: {exc}')
            return
        except Exception as exc:  # pragma: no cover - defensive programming
            self.get_logger().error(f'Unexpected error during transcription: {exc}')
            return

        sentences = self._split_sentences(transcript)
        for sentence in sentences:
            if sentence:
                self._result_queue.put(sentence)

    def _transcribe_audio(self, recognizer: sr.Recognizer, audio: sr.AudioData) -> str:
        if self._engine == 'google':
            return recognizer.recognize_google(
                audio, language=self._language, key=self._recognizer_api_key
            )
        if self._engine == 'google_cloud':
            if not self._recognizer_api_key:
                raise sr.RequestError(
                    'Google Cloud recognition requires the `api_key` parameter '
                    'to provide service account credentials.'
                )
            return recognizer.recognize_google_cloud(
                audio,
                language=self._language,
                credentials_json=self._recognizer_api_key,
            )
        if self._engine == 'sphinx':
            return recognizer.recognize_sphinx(audio, language=self._language)
        if self._engine == 'whisper':
            return recognizer.recognize_whisper(audio, language=self._language)
        raise ValueError(f'Unsupported speech recognition engine: {self._engine}')

    def _publish_pending_results(self) -> None:
        while True:
            try:
                sentence = self._result_queue.get_nowait()
            except Empty:
                return
            msg = String()
            msg.data = sentence
            self._publisher.publish(msg)
            self.get_logger().debug(f'Published transcript: {sentence}')

    @staticmethod
    def _split_sentences(text: str) -> Iterable[str]:
        segments = re.split(r'(?<=[.!?])\s+', text.strip())
        return [segment.strip() for segment in segments if segment.strip()]

    @staticmethod
    def _coerce_optional_float(value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        try:
            float_value = float(value)
        except (TypeError, ValueError):
            return None
        if float_value <= 0:
            return None
        return float_value


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    try:
        node = SpeechToTextNode()
    except Exception:
        rclpy.shutdown()
        raise

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
