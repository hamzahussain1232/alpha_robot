#!/usr/bin/env python3
import json
import os
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from std_msgs.msg import String


class RecognizerBackendError(RuntimeError):
    pass


class VoiceMicNode(Node):
    def __init__(self):
        super().__init__("voice_mic_node")

        self.declare_parameter("backend", "vosk")
        self.declare_parameter("offline_only", True)
        self.declare_parameter("language", "en-US")
        self.declare_parameter("phrase_time_limit", 3.5)
        self.declare_parameter("listen_timeout", 2.0)
        self.declare_parameter("ambient_adjust_sec", 1.2)
        self.declare_parameter("energy_threshold", 300)
        self.declare_parameter("dynamic_energy_threshold", True)
        self.declare_parameter("pause_threshold", 0.8)
        self.declare_parameter("non_speaking_duration", 0.5)
        self.declare_parameter("device_index", -1)
        self.declare_parameter("device_fallback_indices", [12, 10, 0, -1])
        self.declare_parameter("max_consecutive_errors", 8)
        self.declare_parameter("recalibrate_every_sec", 60.0)
        self.declare_parameter("allow_sphinx_fallback", False)
        self.declare_parameter("vosk_model_path", "")
        self.declare_parameter("vosk_sample_rate", 16000)
        self.declare_parameter("use_vosk_grammar", True)
        self.declare_parameter(
            "vosk_grammar",
            [
                "forward",
                "backward",
                "left",
                "right",
                "stop",
                "arm home",
                "arm pick",
                "arm place",
                "home",
                "pick",
                "place",
                "move",
                "go",
                "navigate",
                "drive",
                "to",
                "marker",
                "zero",
                "one",
                "two",
                "three",
                "four",
                "five",
                "six",
                "seven",
                "eight",
                "nine",
                "ten",
                "move to one",
                "move to two",
                "move to three",
                "go to one",
                "go to two",
                "go to three",
                "marker one",
                "marker two",
                "marker three",
            ],
        )
        self.declare_parameter("whisper_model", "tiny.en")
        self.declare_parameter("whisper_compute_type", "int8")
        self.declare_parameter("whisper_beam_size", 3)
        self.declare_parameter("whisper_vad_filter", True)
        self.declare_parameter("whisper_cpu_threads", 4)
        self.declare_parameter("whisper_language", "en")
        self.declare_parameter("whisper_temperature", 0.0)
        self.declare_parameter("whisper_best_of", 1)
        self.declare_parameter("whisper_condition_on_previous_text", False)
        self.declare_parameter(
            "whisper_initial_prompt",
            "robot commands: forward backward left right stop arm home arm pick arm place move to one move to two move to three marker one marker two marker three",
        )

        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.offline_only = bool(self.get_parameter("offline_only").value)
        self.language = str(self.get_parameter("language").value).strip()
        self.phrase_time_limit = float(self.get_parameter("phrase_time_limit").value)
        self.listen_timeout = float(self.get_parameter("listen_timeout").value)
        self.ambient_adjust_sec = float(self.get_parameter("ambient_adjust_sec").value)
        self.energy_threshold = int(self.get_parameter("energy_threshold").value)
        self.dynamic_energy_threshold = bool(self.get_parameter("dynamic_energy_threshold").value)
        self.pause_threshold = float(self.get_parameter("pause_threshold").value)
        self.non_speaking_duration = float(self.get_parameter("non_speaking_duration").value)
        self.device_index = int(self.get_parameter("device_index").value)
        self.device_fallback_indices = [
            int(v) for v in self.get_parameter("device_fallback_indices").value
        ]
        self.max_consecutive_errors = int(self.get_parameter("max_consecutive_errors").value)
        self.recalibrate_every_sec = float(self.get_parameter("recalibrate_every_sec").value)
        self.allow_sphinx_fallback = bool(self.get_parameter("allow_sphinx_fallback").value)
        self.vosk_model_path = str(self.get_parameter("vosk_model_path").value).strip()
        self.vosk_sample_rate = int(self.get_parameter("vosk_sample_rate").value)
        self.use_vosk_grammar = bool(self.get_parameter("use_vosk_grammar").value)
        self.vosk_grammar = [str(v).strip().lower() for v in self.get_parameter("vosk_grammar").value]
        self.whisper_model_name = str(self.get_parameter("whisper_model").value).strip()
        self.whisper_compute_type = str(self.get_parameter("whisper_compute_type").value).strip()
        self.whisper_beam_size = int(self.get_parameter("whisper_beam_size").value)
        self.whisper_vad_filter = bool(self.get_parameter("whisper_vad_filter").value)
        self.whisper_cpu_threads = int(self.get_parameter("whisper_cpu_threads").value)
        self.whisper_language = str(self.get_parameter("whisper_language").value).strip() or "en"
        self.whisper_temperature = float(self.get_parameter("whisper_temperature").value)
        self.whisper_best_of = int(self.get_parameter("whisper_best_of").value)
        self.whisper_condition_on_previous_text = bool(
            self.get_parameter("whisper_condition_on_previous_text").value
        )
        self.whisper_initial_prompt = str(self.get_parameter("whisper_initial_prompt").value).strip()

        self.text_pub = self.create_publisher(String, "/voice/text", 10)
        self.running = True
        self.active_device_index = self.device_index
        self.consecutive_errors = 0
        self.last_backend_error_log_ts = 0.0

        self.sr = None
        self.recognizer = None
        self.vosk = None
        self.vosk_model = None
        self.whisper_model = None
        self._sphinx_disabled = False
        self._sphinx_warned = False
        self._vosk_warned = False
        self._whisper_warned = False
        self._init_backend()
        self._init_vosk()
        self._init_whisper()

        if self.sr is None:
            self.get_logger().error(
                "speech_recognition is not installed. Install with: pip install SpeechRecognition pyaudio"
            )
            return

        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.thread.start()
        self.get_logger().info("Voice mic node started. Publishing text to /voice/text")

    def _init_backend(self):
        try:
            import speech_recognition as sr  # pylint: disable=import-outside-toplevel
        except Exception as exc:
            self.get_logger().error(f"Failed to import speech_recognition: {exc}")
            return

        self.sr = sr
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold
        self.recognizer.pause_threshold = self.pause_threshold
        self.recognizer.non_speaking_duration = self.non_speaking_duration

    def _resolve_vosk_model_path(self):
        candidates = []
        requested = Path(self.vosk_model_path).expanduser() if self.vosk_model_path else None

        if requested is not None:
            if requested.is_absolute():
                candidates.append(requested)
            else:
                candidates.extend([Path.cwd() / requested])
                try:
                    pkg_prefix = Path(get_package_prefix("articubot_one"))
                    workspace_root = pkg_prefix.parent.parent
                    candidates.extend(
                        [
                            workspace_root / requested,
                            workspace_root / "models" / requested.name,
                            pkg_prefix / requested,
                        ]
                    )
                except Exception:
                    pass
                home = Path.home()
                candidates.extend(
                    [
                        home / requested,
                        home / "models" / requested.name,
                        home / ".cache" / "vosk" / requested.name,
                    ]
                )
        else:
            home = Path.home()
            candidates.extend(
                [
                    home / "vosk-model-small-en-us-0.15",
                    home / "models" / "vosk-model-small-en-us-0.15",
                    home / ".cache" / "vosk" / "vosk-model-small-en-us-0.15",
                ]
            )

        candidates.append(Path("/usr/share/vosk/model"))

        for path in candidates:
            if path and path.is_dir():
                return str(path)
        return ""

    def _init_vosk(self):
        try:
            import vosk  # pylint: disable=import-outside-toplevel
        except Exception as exc:
            self.get_logger().warn(f"Vosk not available: {exc}")
            return

        model_path = self._resolve_vosk_model_path()
        if not model_path:
            self.get_logger().warn(
                "Vosk model not found. Set voice_mic_node.vosk_model_path "
                "to your offline model folder."
            )
            return

        try:
            self.vosk = vosk
            try:
                # Reduce Vosk console verbosity.
                self.vosk.SetLogLevel(-1)
            except Exception:
                pass
            self.vosk_model = self.vosk.Model(model_path)
            self.get_logger().info(f"Loaded Vosk model: {model_path}")
        except Exception as exc:
            self.get_logger().warn(f"Failed to load Vosk model from {model_path}: {exc}")
            self.vosk = None
            self.vosk_model = None

    def _init_whisper(self):
        try:
            from faster_whisper import WhisperModel  # pylint: disable=import-outside-toplevel
        except Exception as exc:
            self.get_logger().warn(f"faster-whisper not available: {exc}")
            return

        try:
            self.whisper_model = WhisperModel(
                self.whisper_model_name,
                device="cpu",
                compute_type=self.whisper_compute_type,
                cpu_threads=self.whisper_cpu_threads,
            )
            self.get_logger().info(
                f"Loaded faster-whisper model: {self.whisper_model_name} "
                f"(compute_type={self.whisper_compute_type}, cpu_threads={self.whisper_cpu_threads})"
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to load faster-whisper model '{self.whisper_model_name}': {exc}"
            )
            self.whisper_model = None

    @staticmethod
    def _audio_to_float32(audio, sample_rate):
        pcm = audio.get_raw_data(convert_rate=sample_rate, convert_width=2)
        samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        return samples

    def _recognize_vosk(self, audio):
        if self.vosk is None or self.vosk_model is None:
            raise RecognizerBackendError(
                "Vosk backend not ready (install vosk and set vosk_model_path)"
            )

        try:
            pcm = audio.get_raw_data(convert_rate=self.vosk_sample_rate, convert_width=2)
            if self.use_vosk_grammar and self.vosk_grammar:
                # Constrain decoding to command words for faster, more stable command recognition.
                grammar = json.dumps(self.vosk_grammar + ["[unk]"])
                recognizer = self.vosk.KaldiRecognizer(
                    self.vosk_model,
                    float(self.vosk_sample_rate),
                    grammar,
                )
            else:
                recognizer = self.vosk.KaldiRecognizer(self.vosk_model, float(self.vosk_sample_rate))
            recognizer.AcceptWaveform(pcm)
            result = json.loads(recognizer.FinalResult())
            text = str(result.get("text", "")).strip()
            return text
        except Exception as exc:
            raise RecognizerBackendError(f"Vosk recognition failed: {exc}") from exc

    def _recognize_whisper(self, audio):
        if self.whisper_model is None:
            raise RecognizerBackendError(
                "faster-whisper backend not ready (install faster-whisper and model)"
            )

        try:
            samples = self._audio_to_float32(audio, self.vosk_sample_rate)
            segments, _ = self.whisper_model.transcribe(
                samples,
                language=self.whisper_language,
                beam_size=self.whisper_beam_size,
                best_of=self.whisper_best_of,
                temperature=self.whisper_temperature,
                condition_on_previous_text=self.whisper_condition_on_previous_text,
                initial_prompt=self.whisper_initial_prompt if self.whisper_initial_prompt else None,
                vad_filter=self.whisper_vad_filter,
            )
            text = " ".join(seg.text.strip() for seg in segments if seg.text).strip()
            return text
        except Exception as exc:
            raise RecognizerBackendError(f"faster-whisper recognition failed: {exc}") from exc

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

    def transcribe(self, audio):
        def recognize_sphinx_safe():
            if not self.allow_sphinx_fallback or self._sphinx_disabled:
                raise RuntimeError("Sphinx fallback disabled")
            try:
                return self.recognizer.recognize_sphinx(audio)
            except Exception as exc:
                self._sphinx_disabled = True
                if not self._sphinx_warned:
                    self.get_logger().warn(
                        f"Sphinx disabled due to runtime error: {exc}. "
                        "Use backend:=vosk for offline mode."
                    )
                    self._sphinx_warned = True
                raise

        def recognize_offline_preferred():
            # Prefer faster-whisper for quality; fallback to Vosk.
            try:
                return self._recognize_whisper(audio)
            except Exception as exc_whisper:
                if not self._whisper_warned:
                    self.get_logger().warn(f"Whisper fallback warning: {exc_whisper}")
                    self._whisper_warned = True
                return self._recognize_vosk(audio)

        if self.backend in ("faster_whisper", "whisper"):
            return self._recognize_whisper(audio)

        if self.backend == "vosk":
            return self._recognize_vosk(audio)

        if self.backend == "auto":
            if self.offline_only:
                return recognize_offline_preferred()
            try:
                return self.recognizer.recognize_google(audio, language=self.language)
            except self.sr.UnknownValueError:
                # No words recognized; normal case.
                raise
            except self.sr.RequestError as exc:
                self.get_logger().warn(f"Google STT unavailable: {exc}")
                return recognize_offline_preferred()
            except Exception as exc:
                self.get_logger().warn(f"Google STT runtime error: {exc}")
                return recognize_offline_preferred()

        if self.backend == "google":
            if self.offline_only:
                raise RecognizerBackendError(
                    "offline_only=true but backend is google. Use backend:=faster_whisper"
                )
            return self.recognizer.recognize_google(audio, language=self.language)

        if self.backend == "sphinx":
            return recognize_sphinx_safe()

        self.get_logger().warn(f'Unknown backend "{self.backend}", using auto')
        try:
            return self.recognizer.recognize_google(audio, language=self.language)
        except Exception:
            return recognize_offline_preferred()

    def _candidate_indices(self):
        candidates = []
        if self.device_index >= 0:
            candidates.append(self.device_index)
        for idx in self.device_fallback_indices:
            idx = int(idx)
            if idx not in candidates:
                candidates.append(idx)
        if -1 not in candidates:
            candidates.append(-1)
        return candidates

    def _device_name(self, index):
        try:
            names = self.sr.Microphone.list_microphone_names()
            if 0 <= index < len(names):
                return names[index]
        except Exception:
            pass
        if index == -1:
            return "default"
        return f"index_{index}"

    def _open_microphone_source(self):
        """
        Try requested mic first, then fallback list.
        Returns (microphone, source) on success, (None, None) on failure.
        """
        for idx in self._candidate_indices():
            kwargs = {}
            if idx >= 0:
                kwargs["device_index"] = idx
            try:
                mic = self.sr.Microphone(**kwargs)
                source = mic.__enter__()
                self.active_device_index = idx
                self.get_logger().info(
                    f'Using microphone {idx}: "{self._device_name(idx)}"'
                )
                return mic, source
            except Exception as exc:
                self.get_logger().warn(f'Failed opening mic {idx}: {exc}')
                continue
        return None, None

    def _recalibrate(self, source):
        self.get_logger().info("Adjusting to ambient noise...")
        self.recognizer.adjust_for_ambient_noise(source, duration=self.ambient_adjust_sec)
        self.get_logger().info(
            f"Ready for voice commands (backend={self.backend}, language={self.language}, mic={self.active_device_index})"
        )

    def listen_loop(self):
        if self.sr is None:
            return

        last_recalibrate_ts = 0.0
        mic = None
        source = None

        while self.running and rclpy.ok():
            try:
                if source is None:
                    mic, source = self._open_microphone_source()
                    if source is None:
                        self.get_logger().error("Could not open any microphone, retrying in 2s")
                        time.sleep(2.0)
                        continue
                    self._recalibrate(source)
                    self.consecutive_errors = 0
                    last_recalibrate_ts = time.monotonic()

                # Periodic recalibration improves long-run stability with fan/noise changes.
                if time.monotonic() - last_recalibrate_ts > self.recalibrate_every_sec:
                    self._recalibrate(source)
                    last_recalibrate_ts = time.monotonic()

                audio = self.recognizer.listen(
                    source,
                    timeout=self.listen_timeout,
                    phrase_time_limit=self.phrase_time_limit,
                )
                text = self.transcribe(audio).strip()
                if text:
                    self.get_logger().info(f'Heard: "{text}"')
                    self.publish_text(text)
                self.consecutive_errors = 0

            except self.sr.WaitTimeoutError:
                continue
            except self.sr.UnknownValueError:
                # Heard audio but no valid words; not a device failure.
                continue
            except RecognizerBackendError as exc:
                # Backend issues are not microphone issues; don't reopen mic.
                now = time.monotonic()
                if now - self.last_backend_error_log_ts > 5.0:
                    self.get_logger().warn(str(exc))
                    self.last_backend_error_log_ts = now
                time.sleep(0.2)
                continue
            except self.sr.RequestError as exc:
                # Cloud backend hiccup; keep running and rely on fallback.
                self.get_logger().warn(f"Speech service error: {exc}")
                self.consecutive_errors += 1
            except Exception as exc:
                self.get_logger().warn(f"Microphone loop error: {exc}")
                self.consecutive_errors += 1

            if self.consecutive_errors >= self.max_consecutive_errors:
                self.get_logger().warn(
                    f"Too many mic errors ({self.consecutive_errors}), reopening microphone"
                )
                self.consecutive_errors = 0
                try:
                    if mic is not None:
                        mic.__exit__(None, None, None)
                except Exception:
                    pass
                mic = None
                source = None

    def destroy_node(self):
        self.running = False
        return super().destroy_node()


def main():
    rclpy.init()
    node = VoiceMicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
