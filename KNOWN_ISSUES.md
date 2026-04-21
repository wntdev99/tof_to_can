# Known Issues

현재 검증된 사용 구성과 미해결 이슈를 정리한다.

## 작동 확인된 구성

다음 3 개 센서 조합에서 핫플러그 + CAN 송신 모두 정상 동작한다.

| 센서 | 주소 | 비고 |
|------|------|------|
| VL53L5CX | 0x52 → 0x50 | ULD FW 업로드 후 주소 이동 |
| VL53L4CD | 0x52 → 0x54 | reg[0x0001] 단일 write 로 주소 이동 |
| VL53L7CX | 0x52 → 0x56 | LPn 토글 후 주소 이동 |

## 미해결 이슈

### 1. VL53L8CX — 동작 미검증

- 드라이버 코드(`firmware/drivers/vl53l8cx/`)와 탐지 로직은 이미 들어가 있으나
  현재 사용 중인 하드웨어에서 정상 초기화가 확인되지 않았다.
- `phase2` / `poll_hotplug` 에서 탐지를 시도하지만 `!present` 상태로 유지된다.
- 코드는 그대로 유지한다 — 실패 시 재시도만 발생하고 다른 센서에 영향 없음.

### 2. TMF8828 — factory firmware 없는 bare 변종

증상:
```
[TMF8828] initial APPID read: ret=1 val=0x80    ← bootloader
[TMF8828] REMAP_RESET i2c_write ret=4           ← 명령 수락됨
[TMF8828] post-REMAP CMD_STAT=0x11              ← CMD 에코, 처리 안 됨
[TMF8828] app not ready (APPID=0x80)            ← 부팅할 앱이 flash 에 없음
```

해석:
- `REMAP_RESET (0x11)` 은 bootloader 가 수락했으나 `CMD_STAT` 이 `0x11` (CMD 에코)
  상태에서 고정됨. 성공했다면 `0x00` (READY) 으로 전이되어야 하며 이후 APPID 가
  `0x03` 으로 바뀌어야 한다.
- bootloader 는 정상 동작하지만 flash 에 측정 앱(`mainapp.bin`, ~75 KB) 이 들어
  있지 않아 remap 할 대상이 없는 상태로 추정.
- 이는 AMS 의 "bare" 변종(모듈러 비전 kit 등)에서 관찰되는 현상으로, 사용 전
  호스트에서 F/W 를 업로드해야 한다.

해결 경로 (향후):
- AMS 공식 사이트에서 `mainapp.bin` 받아 bootloader 의 `DOWNLOAD_INIT → SET_ADDR →
  W_RAM → RAMREMAP_RESET` 시퀀스로 업로드 필요.
- 현재는 탐지/초기화 시도는 유지하되 실패 시 slot 만 롤백 — 다른 센서 운용에
  영향 없음.

## 하드웨어 이슈 (software 로 해결 불가)

### 다중 STEMMA QT 동시 삽입 시 SDA 하드 스턱

여러 개의 센서를 **동시에** 꽂으면 SDA 라인이 LOW 로 latch 되어 SCL 클럭
아웃(`st_bus_recover_if_stuck`) 으로도 풀리지 않는 경우가 있다.

- 원인 추정: 커넥터 체터링/ESD 로 인한 slave 내부 bus driver latch-up.
- 대응: **한 번에 하나씩** 꽂는다. 이미 stuck 된 경우 전원 재인가가 유일한 복구책.

## 권장 사용 순서

1. 전원 인가 전 상태에서 원하는 센서를 커넥터에 미리 체결.
2. 전원 인가 후 핫플러그가 필요하다면 **한 번에 하나씩** 삽입.
3. 로그에서 `bus stuck (SDA=LOW)` → `bus recovered, SDA=HIGH` 확인. `LOW(still
   stuck)` 가 나오면 전원 재인가.
