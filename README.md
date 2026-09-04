<p align="center">
  <img width="160" alt="ESP32 DW3000 UWB Board" src="https://github.com/user-attachments/assets/ebfb1df1-e6cc-487f-9297-0be2974176e3">
</p>

<h1 align="center">ESP32-DW3000 4-Anchor UWB Positioning</h1>

<p align="center">
  <strong>SS-TWR 거리 측정과 Robust WLS를 적용한 실시간 2차원 UWB 측위 시스템</strong>
</p>

<p align="center">
  <img height="46" alt="PlatformIO" src="https://github.com/user-attachments/assets/df1fb807-44d4-4362-bfb1-a6ed1568186f">
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img height="46" alt="C++" src="https://github.com/user-attachments/assets/cd2536e1-a553-436d-8a82-27c43ac2b8e6">
</p>

<p align="center">
  <sub>ESP32 · DW3000 · SS-TWR · Robust WLS · Web Serial</sub>
</p>

---

ESP32-DW3000 보드 5대로 구성한 실내 2차원 UWB 위치 측위 프로젝트입니다. 1대의 Tag가 4대의 Anchor와 SS-TWR 통신을 수행하고, 측정 거리의 이상값 제거와 Robust WLS 연산을 거쳐 최종 `(x, y)` 좌표를 출력합니다. 출력 좌표는 Web Serial 기반 시각화 화면에서 실시간 궤적으로 확인할 수 있습니다.

## 바로가기

| 구분 | 경로 | 주요 기능 |
|---|---|---|
| Tag Firmware | [main.cpp](./Projects/dw3000_Tag/src/main.cpp) | Poll 송신, 4개 응답 수집, 거리·좌표 계산, Serial 출력 |
| Anchor Firmware | [main.cpp](./Projects/dw3000_Anchor/src/main.cpp) | Poll 검증, ID별 Delayed Response, Timestamp 반환 |
| Position Visualizer | [index.html](./Projects/dw3000_Tag/visualizer/index.html) | Web Serial 수신, 현재 좌표와 최근 이동 궤적 표시 |
| Tag PlatformIO 설정 | [platformio.ini](./Projects/dw3000_Tag/platformio.ini) | ESP32 Arduino 빌드·업로드 및 115200bps Monitor 설정 |
| Anchor PlatformIO 설정 | [platformio.ini](./Projects/dw3000_Anchor/platformio.ini) | Anchor Firmware 빌드·업로드 환경 |

## 사용 장비 및 개발 환경

| 분류 | 구성 |
|---|---|
| UWB 보드 | Makerfabs ESP32 UWB DW3000 × 5 |
| 노드 구성 | Tag × 1, Anchor × 4 |
| 측위 영역 | 2m × 2m, A1(0,0), A2(2,0), A3(2,2), A4(0,2) |
| Firmware | C++, Arduino Framework, DW3000 Driver |
| Visualizer | HTML, CSS, JavaScript, Canvas, Web Serial API |
| 개발 도구 | VS Code, PlatformIO, Git, GitHub |
| 통신 환경 | ESP32↔DW3000 SPI, PC↔Tag USB Serial 115200bps |

## 전체 시스템 구성

Tag가 한 번의 Poll을 송신하면 4개의 Anchor가 ID별 응답 Slot을 사용해 순서대로 Response를 반환합니다. Tag는 한 Round에서 네 응답이 모두 검증된 경우에만 거리와 좌표를 계산합니다.

<p align="center">
  <img width="900" alt="4-Anchor UWB 시스템 구성도" src="https://github.com/user-attachments/assets/e2dde0e9-650f-4869-a188-f0095352dd39">
</p>
<p align="center"><sub>Tag 1대와 Anchor 4대로 구성한 2차원 UWB 측위 시스템</sub></p>

1. Tag가 Sequence 번호를 포함한 12Byte Poll Frame 송신
2. Anchor가 Frame 길이와 Header를 검증하고 Poll 수신 Timestamp 저장
3. A1부터 A4까지 ID별 Delayed TX Slot에서 Response 송신
4. Tag가 21Byte Response의 ID, Sequence, Timestamp 검증
5. Anchor별 거리 계산 및 7-Sample 평균 필터 적용
6. Linear Least Squares와 Robust WLS로 좌표 계산
7. 최근 좌표 5개의 평균을 `x.xxx,y.yyy` 형식으로 출력

## SS-TWR 통신 타이밍과 Frame

동시 응답 충돌을 피하기 위해 Anchor 응답 시점을 `1.0ms + (Anchor ID - 1) × 1.2ms`로 분리했습니다. Response에는 Poll Sequence, Anchor ID, Poll 수신 Timestamp와 Response 송신 Timestamp가 포함됩니다.

<p align="center">
  <img width="900" alt="SS-TWR 통신 타이밍과 Anchor 응답 Slot" src="https://github.com/user-attachments/assets/8d383a73-f5ad-48c5-a44f-79d0621d0819">
</p>
<p align="center"><sub>Poll 송신 후 Anchor ID별로 분리한 Response Slot</sub></p>

### Response Frame 구성

<p align="center">
  <img alt="DW3000 Response Frame WaveDrom" src="https://github.com/user-attachments/assets/07f5a9b3-530b-4c9d-90a1-ac1a37265fc5">
</p>
<p align="center"><sub>WaveDrom으로 작성한 21Byte Response Frame</sub></p>

| Frame | 길이 | 주요 필드 |
|---|---:|---|
| Poll | 12Byte | Common Header, Sequence, FCS |
| Response | 21Byte | Header, Anchor ID, Poll RX TS, Response TX TS, FCS |

## 거리 계산

Tag에서 측정한 Round Trip Time과 Anchor가 전달한 Reply Time으로 전파 시간 `ToF`를 계산합니다. 두 장치의 Clock 차이는 DW3000 Clock Offset 값으로 보정합니다.

$$
T_{round}=t_{response\_rx}-t_{poll\_tx}
$$

$$
T_{reply}=t_{response\_tx}-t_{poll\_rx}
$$

$$
ToF=\frac{T_{round}-T_{reply}(1-ClockOffset)}{2}\times DWT\_TIME\_UNITS
$$

$$
d_i=ToF\times c-b_i
$$

`c`는 빛의 속도, `b_i`는 Anchor별 거리 보정값입니다. 계산 결과가 유한하지 않거나 0~10m 범위를 벗어나면 해당 응답을 제외합니다.

## 2차원 좌표 계산

<p align="center">
  <img width="900" alt="거리 필터링 및 WLS 좌표 계산 흐름" src="https://github.com/user-attachments/assets/036a3294-5e62-422f-8e18-5a15ce10a290">
</p>
<p align="center"><sub>Frame 검증부터 거리 안정화, WLS 보정 및 좌표 출력까지의 처리 흐름</sub></p>

각 Anchor 위치 `(x_i,y_i)`와 측정 거리 `d_i`는 다음 원 방정식을 만족합니다.

$$
(x-x_i)^2+(y-y_i)^2=d_i^2
$$

A1을 기준으로 나머지 식을 빼서 선형화하고 Least Squares로 초기 좌표를 계산합니다.

$$
2(x_i-x_1)x+2(y_i-y_1)y=d_1^2-d_i^2+x_i^2-x_1^2+y_i^2-y_1^2
$$

이후 최근 거리값의 분산으로 Anchor별 가중치를 계산하고 Robust WLS를 최대 6회 반복합니다.

$$
w_i=\frac{1}{\sigma_i^2+0.0025}
$$

잔차가 0.5m보다 크면 Huber Weight를 적용해 순간적으로 튀는 거리값의 영향을 줄입니다. 최종 좌표는 실제 측위 영역인 0~2m로 제한하고 최근 좌표 5개의 평균을 출력합니다.

| 처리 항목 | 설정값 |
|---|---:|
| 유효 거리 | 0~10m |
| 거리 평균 | Anchor별 최근 7개 |
| Stale Reset | 500ms |
| WLS 반복 | 최대 6회 |
| Huber 기준 | 0.5m |
| 좌표 평균 | 최근 5개 |

## 통신 결과와 진단 로그

Anchor는 수신한 Poll 수와 정상 Response 수, Delayed TX 실패 횟수 및 잘못된 Frame 수를 주기적으로 출력합니다. Tag는 한 Round에서 일부 응답이 누락되면 좌표를 계산하지 않고 Anchor 수신 상태와 오류 원인을 진단 로그로 남깁니다.

<p align="center">
  <img width="865" height="399" alt="Anchor Log" src="https://github.com/user-attachments/assets/3400923e-7916-4ce7-bc0d-b086c3aef015" />

</p>
<p align="center"><sub>Anchor의 Poll 수신, Response 송신 및 Delayed TX 상태 확인</sub></p>

```text
WARN,INCOMPLETE_ROUND,MASK=0xE,RX=...,LEN=...,HDR=...,ID=...,
SEQ=...,TIME=...,DIST=...,RXERR=...,TXFAIL=...
```

<p align="center">
  <img width="807" height="331" alt="Tag_disconnect2Anchor_log" src="https://github.com/user-attachments/assets/5a82d91d-f1be-42d8-9002-6d78a3b1d869" />

</p>
<p align="center"><sub>Anchor 응답 누락과 Frame 오류 원인을 항목별 Counter로 분리</sub></p>

`MASK`의 각 Bit는 A1~A4의 수신 여부를 나타냅니다. Length, Header, ID, Sequence, Timestamp, Distance, RX Error를 별도 Counter로 분리하여 응답 누락 원인을 확인할 수 있도록 구성했습니다.

## 실시간 좌표 시각화

Visualizer는 Web Serial API로 115200bps 좌표 로그를 수신하고, 문자열에서 `(x,y)`를 추출해 2m × 2m Canvas에 표시합니다. 현재 위치는 점으로, 최근 100개 좌표는 이동 궤적으로 출력하며 표본 수, 경고 수, Anchor Mask와 마지막 갱신 시간도 함께 표시합니다.

Tag는 계산이 완료된 좌표를 다음 형식으로 출력합니다.

```text
0.142,1.327
0.134,1.299
0.106,1.280
```

### Serial 좌표 출력

<p align="center">
  <img width="840" height="490" alt="dw3000_tag_terminal_coordinates" src="https://github.com/user-attachments/assets/252bae21-5c90-4fe1-b356-675ccbab6419" />

</p>
<p align="center"><sub>Tag에서 출력한 실시간 x, y 좌표 데이터</sub></p>

### 정지 및 이동 측위 결과

#### 정지 상태 좌표 안정화

<p align="center">
  <img width="960" height="540" alt="uwb_position_05s_08s" src="https://github.com/user-attachments/assets/1d5dda73-3e99-4ef9-9ecc-362ef29efe08">
</p>

#### 이동 경로 실시간 추적

<p align="center">
  <img width="960" height="540" alt="uwb_position_14s_16s" src="https://github.com/user-attachments/assets/b1373047-7ce0-48d8-9d5b-ebd2ff2bb0ce">
</p>

정지 시험에서는 좌표가 일정 영역에 유지되는 것을 확인했고, 이동 시험에서는 Tag 이동에 따라 현재 위치와 이동 궤적이 연속적으로 갱신되는 것을 확인했습니다. 별도의 기준 측정 장비를 사용한 절대 위치 정확도 평가는 포함하지 않았습니다.

## Build and Upload

### Tag

```bash
cd Projects/dw3000_Tag
pio run
pio run --target upload
pio device monitor --baud 115200
```

### Anchor

각 보드에 업로드하기 전 [Anchor main.cpp](./Projects/dw3000_Anchor/src/main.cpp)의 `ANCHOR_ID`를 1~4로 변경합니다.

```cpp
#define ANCHOR_ID 1
```

```bash
cd Projects/dw3000_Anchor
pio run
pio run --target upload
```

### Visualizer

Chrome 또는 Edge에서 Web Serial을 사용합니다. 저장소 루트에서 로컬 서버를 실행한 뒤 `http://127.0.0.1:8765`에 접속합니다.

```bash
python -m http.server 8765 --directory Projects/dw3000_Tag/visualizer
```

Tag의 Serial Monitor를 종료한 뒤 **Connect**를 눌러 Tag의 USB Serial Port를 선택합니다. 하나의 Serial Port를 PlatformIO Monitor와 브라우저가 동시에 열 수는 없습니다.

## Repository Structure

```text
.
├── Projects/
│   ├── dw3000_Tag/
│   │   ├── src/main.cpp
│   │   ├── lib/Dw3000/
│   │   ├── visualizer/
│   │   └── platformio.ini
│   └── dw3000_Anchor/
│       ├── src/main.cpp
│       ├── lib/Dw3000/
│       └── platformio.ini
└── docs/
    ├── images/
    └── diagrams/
```

