# ESP32 DW3000 UWB Positioning

ESP32-DW3000 보드로 구성한 4-Anchor UWB 위치 측위 프로젝트입니다.  
Tag와 4개의 Anchor가 SS-TWR 방식으로 거리를 측정하고, 거리 보정과 WLS 연산을 통해 2차원 좌표를 계산합니다.

## Source Code

| 구분 | 코드 이동 | 설명 |
|---|---|---|
| Tag Firmware | [Tag main.cpp](./Projects/dw3000_Tag/src/main.cpp) | Poll Frame을 송신하고 4개 Anchor의 응답을 수집합니다. Timestamp로 Anchor별 거리를 계산한 뒤 WLS를 적용하여 최종 `(x, y)` 좌표를 출력합니다. |
| Anchor Firmware | [Anchor main.cpp](./Projects/dw3000_Anchor/src/main.cpp) | Tag의 Poll Frame을 수신하고 Anchor ID별 응답 Slot에 맞춰 Timestamp가 포함된 Response Frame을 반환합니다. |

## Project Directory

```text
Projects/
├── dw3000_Tag/
│   ├── src/main.cpp
│   ├── platformio.ini
│   ├── lib/
│   └── visualizer/
└── dw3000_Anchor/
    ├── src/main.cpp
    ├── platformio.ini
    └── lib/
