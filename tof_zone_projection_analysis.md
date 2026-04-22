# ToF Zone Projection 왜곡 분석

VL53L5CX / L7CX 8×8 zone 센서의 포인트클라우드 투영 과정에서 발생하는
기하학적 왜곡 현상과 현재 구현의 한계를 정리한다.

---

## 1. 배경 — ToF 센서의 측정값

ToF 센서가 반환하는 `distance_mm`는 **radial distance** (빛이 실제로 왕복한 직선 거리)다.
3D 포인트클라우드를 생성하려면 이 값을 직교 좌표 (x, y, z)로 변환해야 한다.

```
센서 원점
   │
   │  z  (원하는 depth, 광축 수직 거리)
   │
   ├──────── x  (횡 거리)
    \
     \ r  (ToF 측정값 = 빗변)
      \
       P  (실제 점)
```

올바른 변환: `z = r × cos(θ)` — 단순히 `z = r`로 쓰면 외곽 존일수록 z가 과대 추정된다.

---

## 2. 구면 왜곡 (Spherical / Bowl Distortion)

`z = r`을 그대로 사용할 때 발생하는 고전적 왜곡이다.

- 중앙 존 (θ ≈ 0°): z ≈ r — 오차 없음
- 외곽 존 (θ = 21.5°): z = r, 실제 depth는 r × cos(21.5°) ≈ 0.93r
- **결과**: 평평한 바닥이 외곽으로 갈수록 멀리 찍혀 **오목한 그릇(Bowl)** 형태로 보임

---

## 3. 현재 구현 — zone_projection.py

현재 코드는 구면 좌표 → 직교 좌표 변환을 올바르게 수행한다.

```python
# build_unit_rays() — 각 zone의 단위 방향 벡터 사전 계산
x = math.sin(az) * math.cos(el)
y = math.sin(el)
z = math.cos(az) * math.cos(el)   # z = cos(az)·cos(el) = cos(θ) 보정 포함

# project_distances_mm() — 거리 × 단위 벡터 = 3D 포인트
points.append((d_m * rx, d_m * ry, d_m * rz))
```

`rz = cos(az) × cos(el)`은 `cos(θ)` (광축으로부터의 각도)와 수학적으로 동치다.
Bowl 왜곡은 이 단계에서 이론적으로 제거된다.

---

## 4. 실제 관찰된 현상 — 돔(Dome) 왜곡

Bowl이 아니라 **반대 방향** 왜곡이 관찰된다.

```
관찰: 가까운 바닥을 측정했을 때
  - 중앙 존:  depth가 더 크게 찍힘 (더 멀어 보임)
  - 좌우 외곽: depth가 더 작게 찍힘 (더 가까워 보임)

형태: 볼록한 돔(Dome) ∩
```

이는 `cos(θ)` 보정이 **과하게** 적용됐다는 의미다 — 즉, 코드에서 사용하는 θ가
실제보다 크다.

---

## 5. 돔 왜곡의 원인

### 5-1. FoV 파라미터 과대 추정 (주요 원인)

현재 설정: `fov_axis_deg: 43.0` (bridge_params.yaml)

```python
half_fov = math.radians(43.0) / 2.0   # ±21.5°
step     = 2.0 * half_fov / (grid-1)  # zone-center 간격
```

실제 VL53L5CX의 zone-center 간 유효 각도가 43°보다 작다면,
외곽 존에 적용되는 θ가 과도하게 커져 cos(θ)가 실제보다 작게 곱해진다.

```
실제 θ_outer = 18°  →  cos(18°) = 0.951
코드 θ_outer = 21.5°  →  cos(21.5°) = 0.930

z_코드 = d × 0.930  <  z_실제 = d × 0.951  →  외곽이 가깝게 찍힘
```

FoV가 클수록 돔이 깊어진다. 거리에 비례해 돔 깊이가 변하면 이 원인이 지배적이다.

### 5-2. Zone Cone 최근접점 편향 (구조적 한계)

코드 주석에 명시된 근사 오차다.

```
각 zone 은 자체 5-6° 콘이지만 zone center 방향 단일 ray 로 근사한다.
```

실제 센서는 각 zone cone 안에서 **가장 먼저 돌아오는 신호** (= 광축에 가장 가까운
내측 방향)를 측정한다. 이 방향은 zone center보다 θ가 작다.

```
zone center: θ = 21.5°,  half-cone: α ≈ 3°
실제 측정 방향: θ_inner = 21.5° - 3° = 18.5°

d_측정 = Z_floor / cos(18.5°)   (θ_inner 기준)
z_투영 = d_측정 × cos(21.5°)   (코드는 zone center 사용)
       = Z_floor × cos(21.5°)/cos(18.5°)
       ≈ 0.981 × Z_floor        → 외곽이 ~2% 가깝게 찍힘
```

이 오차는 측정 거리와 무관하게 일정하다. FoV 튜닝만으로는 완전 제거가 불가하다.

### 5-3. 구분 방법

| 측정 조건 | 돔 깊이 변화 | 지배 원인 |
|-----------|------------|---------|
| 거리 증가 시 비례해서 커짐 | O | FoV 파라미터 오차 |
| 거리 변해도 거의 일정 | — | Zone cone 편향 |

---

## 6. tof_imager_ros와 비교

참조 구현인 [adityakamath/tof_imager_ros (humble)](https://github.com/adityakamath/tof_imager_ros/tree/humble)과의 비교.

```python
# tof_imager_ros — tof_imager_publisher.py
z = e / 1000   # raw radial distance를 그대로 z로 사용
```

`z = r` 형태로 구면 보정을 전혀 수행하지 않는다. Bowl 왜곡을 그대로 가지고 있으며,
FoV도 45°로 하드코딩되어 있다.

| 항목 | tof_imager_ros | 현재 프로젝트 |
|------|---------------|------------|
| z 계산 | `z = d` (무보정) | `z = d × cos(az) × cos(el)` |
| Bowl 왜곡 | **있음** | 없음 |
| FoV | 45° 하드코딩 | 43° 파라미터화 |
| Zone cone 편향 | 있음 | 있음 (구조적 한계) |
| 왜곡 인식 | 없음 | 명시적 인식 |

현재 프로젝트는 Bowl 왜곡 단계는 해결했으나, 두 구현 모두 zone cone 편향을
체계적으로 해결하지 못한 상태다.

---

## 7. 향후 개선 경로

### 단기: FoV 실측 보정

평평한 바닥을 여러 거리(0.5 m / 1.0 m / 1.5 m)에서 측정하고,
외곽 존과 중앙 존의 depth 차이가 최소화되는 `fov_axis_deg` 값을 찾는다.

```python
# bridge_params.yaml 에서 조정
fov_axis_deg: 43.0   # 돔이 보이면 값을 낮춰 시험
```

돔 깊이가 거리에 비례하면 FoV 조정으로 대부분 해소 가능하다.

### 장기: Zone Cone 편향 보정

각 zone의 실제 측정 방향을 zone center가 아닌 내측 엣지 방향으로 보정하거나,
캘리브레이션 보드(체커보드)를 이용해 zone별 실측 ray 방향을 측정하여 LUT를 구성한다.
VL53L5CX ULD API의 `xtalk_calibration` 데이터도 이 편향 보정에 활용 가능하다.
