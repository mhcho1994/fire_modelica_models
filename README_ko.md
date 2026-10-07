# FIRE 최소 multicopter 모델

활성 패키지는 `MULTICOPTER_INTERFACES.md` v0.7과
`261006_quadrotor-fidelity-port-contract.md`의 최초 실행 범위를 구현합니다.
**연속 P0 motor + R0 rotor + 단일 6DOF 강체 + 이상 센서 + 4점 탄성 착륙다리**, SI 단위,
world NED / chassis·body FRD가 기본입니다.

- `Vehicles.Copter.MultirotorPlant`: N개 로터의 hub wrench를 CG에서 합산합니다.
- `Vehicles.Copter.MultirotorWithSensors`: IMU, magnetometer, GNSS, barometer를 추가합니다.
- `Vehicles.Copter.Presets`: QuadX, HexaX, OctoX, CoaxialX8. Coaxial은 배치만 다르며 간섭 공력은 없습니다.
- `Physical.Mechanical.Chassis`: aggregate 또는 정적 부품 합성으로 질량·CG·full inertia를 계산합니다.
- `Adapters.FastDyn`: 연속 PWM 숫자 정규화, sensor-local→FRD 변환, NED 출력 및 FMU 경계입니다.

`demand`는 **추력 비율이 아닌 목표 회전속도 비율**입니다. 모터 lag는 한 번만 적용됩니다.
센서는 연속 이상 관측식이며 기본 bias는 0입니다. 샘플링·hold·timestamp·응답 필터는 없습니다.
4점 착륙다리는 기본 활성화되어 수평 지면에서 기체를 지지합니다.
`landingGear(enabled=false)`를 지정하면 지면을 통과하는 자유낙하 모델이 됩니다.
일반 지형·항력·전기 motor/ESC·배터리·프로토콜 모델은 활성 경로에 없습니다.

질량 record에는 `mass`, `r_C`, `R_Cj`, `inertia`만 포함합니다. 부품 `componentId`는
TOML과 manifest에 보존하며 composer가 모델 생성 전에 비어 있지 않은 ID의 중복을 검사합니다.
직접 작성하는 Modelica의 `MassProperties(...)` 및 파생 record 생성자에서는 `componentId`
수정자를 제거해야 합니다. Composer를 거치지 않는 경우 부품 중복 계상은 호출자가 확인해야 하며,
질량·회전행렬·관성의 물리적 유효성 검사는 기존 Modelica 함수에서 계속 수행합니다.

## 실행

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py
python3 tools/probe_export.py
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- \
  check configs/quad.toml --profile fastdyn
```

OpenModelica와 Modelica 4.0.0이 필요합니다. FMU probe는 Linux FMI 2.0 Co-Simulation을 검사합니다.
`Examples.QuadHover`, `HexaHover`, `OctoHover`, `CoaxialHover`는 초기 회전속도로 중력을 상쇄하는
개방 루프 예제입니다. 제어기는 포함하지 않습니다.

## 4점 착륙다리

OMEdit에서 `package.mo`를 열고 `Examples.QuadGroundCycle`을 6초간 시뮬레이션하거나,
라이브러리 루트에서 위 `omc` 명령을 실행하세요. 발끝 지상고 2 cm에서 낙하·안착한 뒤
2초에 대칭 추력을 가하고, 2.7초에 모터를 끄고 재착륙합니다.
`altitude`, `normalLoad`, `contactCount`, `acceleration_frd[3]`를 확인할 수 있습니다.
결과 CSV는 `build/landing-gear/QuadGroundCycle_res.csv`에 생성됩니다.

`vehicle.landingGear`는 `Physical.Mechanical.Chassis.LandingGear.Parameters` 레코드입니다.

| 파라미터 | 기본값 / 의미 |
|---|---|
| `enabled` | `true`; `false`이면 번역 시 접촉점 제거 |
| `position_C[4,3]` | X/Y = ±0.17 m, Z = +0.10 m; chassis C 기준 FRD 발끝 위치 |
| `groundZ` | 0 m; 수평 지면의 NED Down 좌표 |
| `stiffness`, `damping` | 발 하나당 3000 N/m, 150 N·s/m |
| `tangentialDamping`, `frictionCoefficient` | 25 N·s/m, 0.6; 수직 하중으로 제한되는 점성 마찰 |

Deprecated의 `CompliantPointLeg`, `LandingGearAssembly`, `Contact` 모델을 재사용했습니다.
기체 강체가 스프링 위의 질량을 제공하며 다리는 별도 질량·상태를 추가하지 않습니다.
실제 다리 질량은 aggregate 또는 `additionalParts`에 한 번만 포함하세요.
발끝 위치에서 `chassis.cg_C`를 빼서 CG 기준으로 보정하며, 지면의 바깥쪽 법선은 `{0,0,-1}`입니다.
접촉 시 수직력은 `max(0, k*max(0,-gap) - c*normalVelocity)`입니다.
마찰은 `mu*normalForce`로 제한되며 정지 마찰 구속은 없습니다.
접촉력과 `r × F` 모멘트를 로터·외력과 함께 한 번만 합산합니다.
접촉·이탈 이벤트는 유지하고, 속도 재설정이나 arming 조건은 사용하지 않습니다.

질량 1.5 kg, 발 하나당 강성 3000 N/m인 수평 quad의 정적 침하량은 1.226 mm,
CG 고도는 0.098774 m, FRD 가속도계 Z는 −9.80665 m/s²입니다.
기본 plant의 `p_start={0,0,-1}`은 유지하며, quad/hexa TOML은 기본 CG·장착 위치에서
발끝 지상고가 2 cm가 되도록 `p_start={0,0,-0.12}`를 명시합니다.
CG나 자세를 바꾸면 `p_tip = p_CG + R_wb*(position_C-cg_C)`로 초기 위치를 정하세요.
로터 수와 무관하게 발은 4개입니다.

Composer schema 3의 선택 항목 `[landing_gear]`에서 위 레코드 필드를 설정할 수 있습니다.

```toml
[landing_gear]
enabled = true
groundZ = 0.0
stiffness = 3000.0
damping = 150.0
tangentialDamping = 25.0
frictionCoefficient = 0.6
```

## 좌표계와 설정 이행

`R_ab`는 a에 대한 b의 자세이며 성분 변환은 `v_a = R_ab*v_b`입니다.
`q_wb={w,x,y,z}`는 같은 자세의 Hamilton quaternion입니다.
기본 기체는 기존 East heading을 보존해 `q_start={sqrt(0.5),0,0,sqrt(0.5)}`이며,
`p_start={0,0,-1}`은 지면 원점보다 1 m 위입니다. Identity quaternion은 North heading입니다.
Rotor local +z 추력축과 sensor local 축을 유지하므로 기본 장착 행렬은 `diag(1,-1,-1)`입니다.
Body FRD에 정렬된 IMU가 필요하면 `R_bImu=identity(3)`을 지정합니다.

TOML은 **schema_version=3**만 허용합니다. 기존 schema 1/2, 옛 `[ground]` 설정과 `geometry.nLegs`, sample period,
`first_order` sensor response는 거부합니다. 기존 FastDyn host TOML의 내장 physics table도
같은 이행이 필요합니다. Schema 숫자만 바꾸지 말고 위치·벡터·관성·장착·초기 자세를 함께 변환하세요.
변환식과 포트는 [architecture](docs/architecture.md)에 있습니다.

이전 fixed wing·rover·지형·sampling·필터·통신 경로 및 ENU/FLU 착륙다리 원본은
[deprecated](deprecated/README.md)에 보존합니다. 이전 클래스와 새 NED/FRD 클래스를
같은 namespace로 함께 로드하지 마세요. 이전 검증 기록은 새 모델의 검증 결과가 아닙니다.

[검증 범위](docs/validation.md), [센서 계약](Systems/Sensing/README.md),
[조합 도구](tools/fire-compose/README.md)를 참고하세요.
Rumoca RBC 연결, FMI 3 packaging, 실제 FastDyn firmware 통합은 이 변경의 검증 범위 밖입니다.
