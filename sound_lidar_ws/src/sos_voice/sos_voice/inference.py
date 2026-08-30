"""모델 로딩 + 추론만 담당하는 파일.

나중에 다른 모델(더 큰 CNN, 다른 구조 등)로 바꾸고 싶으면 이 파일만 교체하면 됩니다.
ROS2 노드(emergency_detector_node.py)는 이 파일의 EmergencyDetector 클래스만
알고 있고, 내부 구현(torch, tflite, onnx 뭐든)에는 관여하지 않습니다.

인터페이스 규약 (이것만 지키면 노드 코드는 안 바꿔도 됨):
    detector = EmergencyDetector(model_path)
    probs: dict = detector.predict(audio: np.ndarray[float32], sample_rate: int)
        -> {'emergency': float, 'normal_speech': float, 'background': float}
        (세 확률의 합은 1.0)
"""
from pathlib import Path

import numpy as np
import torch

TARGET_SR = 16000
CLIP_SAMPLES = TARGET_SR * 3  # 3초 clip 기준으로 학습됨
CLASS_NAMES = ['emergency', 'normal_speech', 'background']


class EmergencyDetector:
    def __init__(self, model_path: str, device: str = 'cpu'):
        """model_path: TorchScript로 export된 양자화 모델 (.pt).
        Colab 노트북 11절('라즈베리파이 배포용 변환')에서 만든 파일 그대로 사용.
        """
        self.device = device
        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"모델 파일을 찾을 수 없습니다: {model_path}")

        self.model = torch.jit.load(str(model_path), map_location=device)
        self.model.eval()

    def _prepare_audio(self, audio: np.ndarray, sample_rate: int) -> np.ndarray:
        """mono, TARGET_SR, 정확히 CLIP_SAMPLES 길이로 맞춤.

        주의: RMS 정규화는 의도적으로 적용하지 않습니다. 실사용 마이크(조용한 방,
        작은 목소리 등) 입력에 RMS 정규화(음량을 -25dBFS로 강제로 끌어올림)를 걸면,
        미세한 배경 잡음까지 크게 증폭되어 모델이 엉뚱하게(특히 emergency로)
        오판하는 문제가 실측으로 확인됐습니다 (학습 데이터는 사람 목소리/생활소음이
        뚜렷한 clip 위주라 정규화가 자연스러웠지만, 조용한 실사용 환경에는 안 맞았음).
        원본 음량 그대로 넣는 것이 검증 결과 더 안정적이었습니다.
        """
        audio = np.asarray(audio, dtype=np.float32)
        if audio.ndim > 1:
            audio = audio.mean(axis=1)

        if sample_rate != TARGET_SR:
            import librosa
            audio = librosa.resample(audio, orig_sr=sample_rate, target_sr=TARGET_SR)

        if len(audio) < CLIP_SAMPLES:
            reps = int(np.ceil(CLIP_SAMPLES / max(len(audio), 1)))
            audio = np.tile(audio, reps)[:CLIP_SAMPLES]
        elif len(audio) > CLIP_SAMPLES:
            audio = audio[:CLIP_SAMPLES]

        return audio

    @torch.no_grad()
    def predict(self, audio: np.ndarray, sample_rate: int = TARGET_SR) -> dict:
        """audio: 1D 또는 2D(다채널) numpy array, float32 권장(int16이어도 자동 변환).
        Returns: {'emergency': p0, 'normal_speech': p1, 'background': p2}
        """
        audio = np.asarray(audio)
        if audio.dtype == np.int16:
            audio = audio.astype(np.float32) / 32768.0

        audio = self._prepare_audio(audio, sample_rate)

        waveform = torch.from_numpy(audio.copy()).unsqueeze(0).to(self.device)
        logits = self.model(waveform)
        probs = torch.softmax(logits, dim=-1).cpu().numpy()[0]

        return {name: float(p) for name, p in zip(CLASS_NAMES, probs)}
