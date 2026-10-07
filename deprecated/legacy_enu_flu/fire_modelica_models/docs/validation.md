# 검증 범위

기준 환경은 OpenModelica `1.26.7~1-g2b913cc`, Modelica Standard Library `4.0.0`, Linux이다. 물리 parameter는 검증용 예제 값이며 실측 기체 식별 결과가 아니다.

## Native 실행

`python3 tools/verify.py`는 17개의 assertion model과 10개의 예제, 합계 27개의 시뮬레이션을 실행한다. `Examples.QuadImuOnly`는 `Systems.Sensing`의 개별 IMU를 Suite 없이 기체에 직접 연결하는 경로를 검사한다. `Tests.SensorReconfiguration`은 사용자 정의 Measurement 교체 및 네 종류 센서의 1차 Response를 해석적 ramp 응답과 비교하며, 채널별 bypass·초기값·sampling/hold를 검사한다. `Examples.QuadImuResponse`는 직접 IMU 구성에서 Response를 교체한다. 결과와 compiler log는 `build/verification/report.json`, `native.log`에 저장된다.

| 검증 대상 | 검사 내용 |
|---|---|
| 강체 | ballistic freefall, 추력 평형, full off-diagonal inertia, quaternion 합성·norm, 회전 body 속도 수송항 |
| 추진계 | motor step의 해석해, demand saturation, rotor 회전 방향별 hub torque |
| 조립 | 4·6·8 rotor와 4-arm/8-rotor, replaceable rotor array, 비연속 채널 map, tilted rotor의 full mount 회전과 CG moment |
| Chassis·payload | 합성 질량·CG·전체 관성, aggregate/assembled 배타성, empty arrays, 모든 mount의 CG 갱신 |
| 지면 | inclined plane, normal force 비음수, 마찰 에너지 소산, angular tip velocity, drop/settling/takeoff, nLegs=0 |
| 센서 | freefall/support specific force, mount·lever arm·bias, 첫 sample·독립 rate·hold·timestamp, GNSS antenna 위치·속도 |
| 호환 경로 | 기존 네 LowFidelity sensor의 settled 출력, fixed wing·rover 예제, 8채널 PWM와 sensor→body FRD/NED·legacy frame 변환 |

질량 record의 중복 ID, 반사 회전행렬, indefinite inertia, 양의 정부호지만 물리적으로 불가능한 principal moments, 질량 없는 nonzero inertia의 **5개 잘못된 구성**은 지정된 진단으로 거부되어야 한다. `ChassisRejectedConfigurations`는 이 목적의 negative fixture package이며 일반 simulation model이 아니다.

이전 `FIRE_Modelica_Update` namespace로도 fixed wing·rover 예제 두 개의 방정식을 검사한다. 기존 vehicle와 LowFidelity는 namespace·센서 package 경로·PWM adapter class 이름과 상수 이름 치환 외에 운동·측정식을 변경하지 않았다. 기존 discrete buffer의 초기조건 부족 경고는 호환 모델에 남아 있고, 신규 sampled variant는 초기값을 명시한다.

Quad drop의 수치 기준은 다음과 같다.

- 질량 1.5 kg, leg 4개, leg당 1500 N/m, tip offset -0.15 m.
- 예측 정착 CG 높이: `0.15 - 1.5*9.80665/(4*1500) = 0.1475483375 m`.
- 0.5 m에서 정지 낙하 시 첫 접촉 예측: `sqrt(2*(0.5-0.15)/9.80665) ≈ 0.2671706101 s`.
- Event 전후 같은 timestamp의 속도를 비교해 contact switching이 velocity reset을 만들지 않는지 검사한다.

이 결과는 모든 강성·solver·step 조합의 수렴을 보장하지 않는다. 새로운 강성이나 더 작은 질량을 쓸 때에는 해당 설정에서 step/tolerance를 다시 검증한다.

## 구성 생성

`FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_generate_config.py -v`로 TOML schema 거부 검사와 생성 class 검사를 함께 실행한다. Quad/Hex/payload 구성과 4종 preset의 zero-leg 조립을 포함한다. `--check`의 성공은 compiler의 방정식 검사이며 생성된 설정의 안정 비행이나 물리 calibration을 의미하지 않는다.

## Export와 외부 통합

FMU build·load·실행은 [별도 기록](export_validation.md)을 따른다. Native DASSL 검증 결과를 FMU나 Rumoca의 실행 결과로 간주하지 않는다.

로컬 Rumoca 소스 revision은 `43b00084c65b5e17922b590ba4682807e4c49525`이며, 이번 작업 환경에는 실행 가능한 CLI/Python package 또는 기존 build artifact가 없었다. Rust/Cargo 1.95.0으로 `CARGO_TARGET_DIR=/tmp/fire_rumoca_target cargo run --offline --locked -p rumoca --bin rumoca -- --help`를 시도했지만, cached `diffsol v0.10.3`을 읽기 전용 `~/.cargo/registry/src`에 풀 수 없어 compilation 전에 종료됐다 (`os error 30`). 이는 환경 제약이며 FIRE 모델이나 Rumoca frontend의 실패를 의미하지 않는다.

Rumoca 소스와 Cargo.lock은 변경하지 않았다. 새 라이브러리의 Rumoca compile/FMU 성공을 주장하지 않으며, 이 경로의 `sample/when`, 접촉 event, export runtime 검증은 남아 있다.

FastDyn backend와 실제 firmware driver 실행도 이번 검증 범위가 아니다. 추가한 adapter는 channel·frame·단위·시간 계약을 명시한 값 경계이며, 실제 host revision의 IO와 연결하는 통합 검증이 별도로 필요하다.

## 2026-09-23 Record 패키지 재배치 검증

`Data`를 제거하고 `MassProperties`를 `Physical.Mechanical`, `AirframeGeometry`를 `Vehicles.Copter.Geometry`, preset을 `Vehicles.Copter.Presets`로 이동한 뒤 다음 검증을 다시 실행했다.

- Native 시뮬레이션 27개, 잘못된 질량 구성 거부 5개, 기존 namespace 검사 2개 통과.
- `FIRE_TEST_OMC=1`로 구성 생성기 시험 19개 통과. TOML 예제와 네 preset의 zero-leg 구성이 새 namespace에서 `checkModel`을 통과했다.
- OpenModelica FMI 2.0 Co-Simulation FMU를 새로 빌드하고 8채널 입력, 센서 sampling/hold, 착지·이륙 실행 검사 통과.
- 이동한 record와 preset의 필드·기본값이 namespace/class 이름 변경을 제외하고 동일하며, 변경한 package의 `package.order`가 실제 파일과 일치함을 확인했다.

기존 Quad/Hex/payload 생성 파일과 manifest도 새 namespace로 갱신했다. 물리식과 TOML schema는 변경하지 않았으며 Rumoca 및 FastDyn host 실행 검증은 포함하지 않는다.
