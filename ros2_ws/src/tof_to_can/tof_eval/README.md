# tof_eval

Gemini 336 대체 가능성 판정을 위한 ToF PointCloud 평가 노드.

단일 `sensor_msgs/PointCloud2` 토픽을 구독해 아래 4 범주 지표를 프레임/세션
단위로 CSV·JSON 에 기록합니다. TF 캘리브레이션 의존 없이 매 프레임 ground
plane 을 RANSAC 으로 추정해 working frame (forward/right/up) 을 산출하므로,
센서를 **물리적으로 동일한 위치에 장착만 하면** 서로 다른 센서 간 지표가
곧바로 비교 가능합니다.

## 지표 그룹

| Group | 지표 | 설명 |
|-------|------|------|
| A1 | valid_zone_ratio | 유효 복귀 비율 (expected_point_count 대비) |
| A2 | valid_count_cv | 프레임별 유효 점 수의 변동계수 (session summary) |
| A3 | effective_hz | 실제 발행 Hz |
| A4 | fwd_density_* | RANSAC inlier 를 working frame 2D 투영 후 거리별 slab density (points/m²) |
| B1 | ground_inlier_ratio | RANSAC inlier / 전체 |
| B2 | ground_residual_std | inlier signed distance stddev |
| B3 | plane_normal_jitter_std | 직전 프레임과의 normal 각도 jitter (deg) |
| B4 | ground_fit_fail_rate | inlier_ratio < min_inlier_ratio 프레임 비율 |
| C1 | obs_*_detection_rate | obstacle 별 감지율 |
| C2 | fp_per_minute | 등록되지 않은 영역에서 검출된 cluster 수 (분당) |
| C3 | obs_*_loc_err_mean | centroid 와 expected_xyz 거리 평균 (참고용, GT 정밀도 의존) |
| C4 | obs_*_size_ratio_*_mean | cluster AABB / 실제 cube 치수 |
| C5 | — | 여러 세션 합산 후 post-process 로 산출 |

## 파라미터 — working frame 기반

모든 좌표는 **working frame (forward, right, up)** 기준:

- `forward` = 센서 optical axis 를 ground plane 에 투영한 단위벡터 (전방)
- `up` = ground plane normal (센서 방향을 양수로)
- `right` = `up × forward`

장애물 위치는 운용자가 실측해서 `expected_xyz` 로 입력합니다.
예: "센서 앞 1 m, 오른쪽 30 cm, 바닥 위 10 cm" → `[1.0, 0.3, 0.1]`.

## 실행

```bash
# 빌드
colcon build --symlink-install --packages-select tof_eval
source install/setup.bash

# YAML 복사해서 세션별로 편집
cp src/tof_to_can/tof_eval/config/eval_params.example.yaml \\
   /tmp/my_session_001.yaml
# (편집: scene.session_id, input_topic, obstacles 등)

# 실행
ros2 launch tof_eval eval.launch.py params_file:=/tmp/my_session_001.yaml

# Ctrl-C 로 종료 → summary/meta/index 자동 기록
```

## 출력 구조

```
/tmp/tof_eval/
├── sessions_index.csv                  # 마스터 — 세션당 1 행 append
├── l5cx_matte_indoor_001/
│   ├── meta.json                       # scene + algo params snapshot
│   ├── summary.csv                     # 세션 1 행 집계 (index 행과 동일)
│   ├── frames.csv                      # per-frame wide table
│   └── obstacles.csv                   # (frame × obstacle) 기록
└── gemini_matte_indoor_001/
    └── ...
```

## Scene vocabulary (권장값)

필드는 자유 문자열이지만 집계 용이성을 위해 아래 vocabulary 권장:

- `floor_material`: `matte_vinyl` `glossy_tile` `carpet` `concrete` `rubber_mat` `wood` `epoxy`
- `floor_color`: `light_grey` `dark_grey` `black` `white` `wood_brown` `beige`
- `ambient_light`: `indoor_led` `indoor_fluorescent` `dim` `dark` `direct_sunlight` `window_backlight`
- `obstacle_surface`: `matte_cardboard` `glossy_plastic` `fabric` `metal` `mirror` `matte_painted`

## Post-process 스크립트

`scripts/` 아래 비교·집계 helper. ROS2 환경 없이도 실행 가능:

```bash
# 모든 세션 집계 + filter
python3 scripts/aggregate_index.py /tmp/tof_eval --filter floor_material=matte_vinyl

# 두 세션 diff
python3 scripts/compare_sessions.py /tmp/tof_eval/l5cx_matte_indoor_001 \\
                                    /tmp/tof_eval/gemini_matte_indoor_001

# 동일 scene 에서 센서별 대응쌍 자동 pairing
python3 scripts/pair_sensors.py /tmp/tof_eval --sensors VL53L5CX,GEMINI_336

# scene 축별 faceting
python3 scripts/facet_by_scene.py /tmp/tof_eval \\
    --group-by ambient_light --metric obs_cube_a_detection_rate
```

## Known limitations

- Ground plane RANSAC 은 바닥이 **프레임 내 가장 큰 평면** 이라는 가정이 있음.
  긴 벽이 시야를 덮으면 벽을 ground 로 오인할 수 있음.
- A1 `valid_zone_ratio` 는 `expected_point_count` 에 의존. Gemini 처럼 dense
  입력은 width×height 로 설정해야 의미있음.
- Obstacle 매칭은 single-snapshot (프레임 독립). 프레임 간 지속성 tracking 은 없음.
