import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models
import subprocess

# 1. 모델 아키텍처 재정의 (노트북의 클래스 내용)
class PatchDescriptionNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = models.efficientnet_b4(weights='IMAGENET1K_V1')
        self.features = nn.Sequential(
            backbone.features[0], backbone.features[1], backbone.features[2],
            backbone.features[3], backbone.features[4]
        )
        for p in self.parameters():
            p.requires_grad = False
        dummy = torch.zeros(1, 3, 256, 256)
        self.out_channels = self.features(dummy).shape[1]

    def forward(self, x):
        return self.features(x)

class StudentNetwork(nn.Module):
    def __init__(self, in_channels: int, out_channels: int):
        super().__init__()
        mid = in_channels // 2
        self.net = nn.Sequential(
            nn.Conv2d(in_channels, mid, 3, padding=1), nn.BatchNorm2d(mid), nn.ReLU(inplace=True),
            nn.Conv2d(mid, mid, 3, padding=1), nn.BatchNorm2d(mid), nn.ReLU(inplace=True),
            nn.Conv2d(mid, out_channels, 1)
        )
    def forward(self, x):
        return self.net(x)

class AnomalyAutoencoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 32, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, 3, stride=2, padding=1), nn.ReLU(inplace=True)
        )
        self.bottleneck = nn.Sequential(
            nn.Conv2d(256, 128, 1), nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, 1)
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(256, 128, 3, stride=2, padding=1, output_padding=1), nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, 3, stride=2, padding=1, output_padding=1), nn.ReLU(inplace=True),
            nn.ConvTranspose2d(64, 32, 3, stride=2, padding=1, output_padding=1), nn.ReLU(inplace=True),
            nn.ConvTranspose2d(32, 3, 3, stride=2, padding=1, output_padding=1), nn.Sigmoid()
        )
    def forward(self, x):
        return self.decoder(self.bottleneck(self.encoder(x)))

class EfficientAD_Small(nn.Module):
    IMG_SIZE = 256
    def __init__(self):
        super().__init__()
        self.teacher = PatchDescriptionNetwork()
        ch = self.teacher.out_channels
        self.student = StudentNetwork(ch, ch)
        self.autoencoder = AnomalyAutoencoder()
        
        self.register_buffer('teacher_mean', torch.zeros(1, ch, 1, 1))
        self.register_buffer('teacher_std', torch.ones(1, ch, 1, 1))
        self.register_buffer('q_low', torch.tensor(0.0))
        self.register_buffer('q_high', torch.tensor(1.0))
        self.register_buffer('threshold', torch.tensor(0.5))

    def compute_anomaly_map(self, x):
        with torch.no_grad():
            t_feat = self.teacher(x)
            t_feat_norm = (t_feat - self.teacher_mean) / (self.teacher_std + 1e-8)
        s_feat = self.student(t_feat_norm)
        ts_map = ((t_feat_norm - s_feat) ** 2).mean(dim=1, keepdim=True)
        ae_out = self.autoencoder(x)
        ae_map = ((x - ae_out) ** 2).mean(dim=1, keepdim=True)
        ts_map_up = F.interpolate(ts_map, size=(x.shape[2], x.shape[3]), mode='bilinear', align_corners=False)
        combined = 0.6 * ts_map_up + 0.4 * ae_map
        combined = (combined - self.q_low) / (self.q_high - self.q_low + 1e-8)
        return combined.clamp(0, 1), ts_map_up, ae_map

# 2. TensorRT ONNX 추출을 위한 Wrapper 클래스
class EfficientAD_TRT_Wrapper(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model
    
    def forward(self, x):
        # ONNX 내보내기를 위해 텐서 연산만 반환
        combined, _, _ = self.model.compute_anomaly_map(x)
        return combined

def main():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # 모델 경로 지정
    pt_path = '/home/caps/fifo_ws/src/obstacle_detector/models/112_efficientad.pt' 
    onnx_path = 'efficientad.onnx'
    engine_path = 'efficientad.engine'

    print("1. 모델 및 가중치 로드 중...")
    base_model = EfficientAD_Small().to(device)
    ckpt = torch.load(pt_path, map_location=device)
    base_model.load_state_dict(ckpt['model_state'])
    base_model.eval()
    
    trt_model = EfficientAD_TRT_Wrapper(base_model).to(device)
    trt_model.eval()

    print(f"2. [{onnx_path}] 파일로 ONNX 추출 중...")
    dummy_input = torch.randn(1, 3, 256, 256, device=device)
    torch.onnx.export(
        trt_model, 
        dummy_input, 
        onnx_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
    )

    print(f"3. [{engine_path}] 젯슨 오린 나노용 FP16 엔진 빌드 중 (수 분 소요)...")
    trtexec_cmd = [
        "trtexec",
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        "--fp16" # 오린 나노 성능을 극대화하는 FP16 옵션
    ]
    
    try:
        subprocess.run(trtexec_cmd, check=True)
        print("TensorRT 엔진 파일 변환이 성공적으로 완료되었습니다!")
    except FileNotFoundError:
        print("trtexec를 실행할 수 없습니다. 젯슨 환경변수를 확인해 주세요.")
    except subprocess.CalledProcessError as e:
        print(f"변환 중 오류가 발생했습니다: {e}")

if __name__ == "__main__":
    main()