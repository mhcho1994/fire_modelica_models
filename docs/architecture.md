# 초기 구현의 경계와 확장 방법

2026-09-22 설계안의 초기 멀티로터 범위를 구현했다. 설계안에 나열된 전체 Electrical/Logical 연구 기능은 후속 범위로 남긴다. 원래 설계 제안 문서는 당시 검토 기록이며, 현재 구현과 실행 결과는 이 저장소의 문서와 검증 도구를 기준으로 한다.

## 상태와 하중

`RigidBody6DOF`의 상태는 `p_w`, `v_b`, `q_wb`, `omega_b`이다. Full inertia tensor를 사용하며 비대칭 관성, 양의 정부호 조건, 초기 quaternion을 검사한다. Quaternion은 초기 정규화 후 연속 norm 보정을 사용하고 회전행렬 평가 시에도 정규화한다. Euler angle은 표시 용도다.

기체는 non-gravity body force와 CG moment를 합산하고, 강체가 중력을 한 번 적용한다. `der(v_b)`와 world inertial acceleration을 구분한다. 예를 들어 freefall의 CG specific force는 0이고 수평 지지 상태에서는 body +z 방향으로 g이다.

Rotor local +z가 양의 추력 축이고 `spinSign`은 그 축에 대한 오른손 법칙이다. `R_br`은 rotor→body 회전이다. Hub force의 `r×F`와 hub torque는 vehicle에서 한 번 결합한다. Local inflow에는 CG 속도, `omega×r`, 바람이 들어간다. 초기 quadratic blade는 이 inflow 포트를 사용하지 않는다.

`kT`는 `N/(rad/s)^2`, `kQ`는 `N.m/(rad/s)^2`의 차원 계수다. 현재 계수는 정해진 공기 밀도에서 보정한 값으로 취급하며 plant의 `density` 입력은 body drag에만 적용된다. 무차원 Ct/Cq를 그대로 넣으면 안 된다. 밀도·유속 의존 공력을 추가할 때에는 rotor/blade 계약을 함께 확장한다.

## 부품 교체

`Systems.Propulsion.PartialRotorUnit`은 demand, local inflow·각속도와 3축 hub wrench의 공통 경계다. 초기 `MultirotorPlant`는 경험적 속도 profile이므로 `PartialSpeedDrivenRotor`로 제한한 `RotorUnit` 교체 지점을 사용한다. `Tests.RotorReplacement`는 실제 replaceable/redeclare array 조합을 검증한다.

전기 구동은 별도 `ElectricRotor` 조립을 추가해야 한다. 초기 first-order lag에는 blade 부하에 따른 속도 변화, shaft inertia reaction, gyro torque, battery sag, 전력 수지가 없다. 상세 motor를 넣으며 기존 반작용 torque를 다시 합산하지 않도록 해야 한다. Flapping·rotor 간 유동 간섭도 아직 계산하지 않는다.

Chassis는 motion state를 갖지 않고 질량 특성만 합성한다. 각 part의 inertia는 part CG와 local inertia axes 기준이다. `R_Cj`로 회전한 뒤 평행축 정리를 적용한다. Fixed payload는 자체 상태와 추가 중력을 가지지 않는다. Moving payload는 이 rigid mass budget에 중복 포함하지 않는 별도 dynamics가 필요하다.

`componentId`는 optional String이다. 비어 있지 않은 ID의 중복을 검사하며, ID가 없는 part가 물리적으로 중복인지 모델이 추론하지는 않는다. Aggregate와 assembled budget은 서로 배타적이다. 비물리적인 inertia, 반사/비직교 mount, 질량 없이 관성만 있는 part도 거부한다.

## 접촉과 환경

`CompliantPointLeg`는 고정된 tip 위치에서 spring/damper 힘을 계산한다. Normal force는 `max(0,k*penetration+c*penetrationRate)`이며 양의 gap에서 0이다. Tangential damping은 마찰 한계로 제한하고 normal load가 없으면 0이다. `ContactMode`의 실제 relation event를 유지한다. 높은 강성에서는 solver 오차·step에 따른 접촉 시각 민감도를 고려해야 한다.

`Worlds.Terrain.Plane`은 기울어진 평면과 일정한 병진 속도를 지원한다. 움직이는 평면에는 `surfacePoint_w`, `surfaceNormal_w`, `surfaceVelocity_w` 세 출력을 모두 연결한다. 위치만 고정한 채 velocity만 입력하면 움직이는 평면 geometry를 의미하지 않는다. `nLegs=0`도 지원한다.

`Worlds.Environment`는 기존 일정 중력·바람·대기·자기장 모델이다. Multirotor의 environment 입력에 해당 출력을 modifier로 연결할 수 있다. Barometer의 ambient pressure와 geometric altitude는 서로 독립적인 low-fidelity 값이며, GNSS도 geodetic 모델이 아니다.

## 입출력과 Logical 범위

신규 센서 측정과 body truth를 각각 `SensorMeasurements`, `VehicleTruth`로 구분한다. 개별 센서 구현은 `Systems.Sensing.{IMU,Magnetometer,GNSS,Barometer}`에 있다. 각 센서가 측정식·bias·sampling과 자신의 상태를 담당하고, `SensorSuite`는 mount parameter·입력·출력을 연결하는 선택적 조립체다. 통신 adapter가 추가 sampling 지연을 만들지 않는다. PWM 숫자 입력을 선택할 때에는 `PwmDemand`가 command acquisition을 맡는다. 최초 sample은 time-zero event의 right limit에서 유효하다.

`MultirotorWithSensors`와 기본 값/FMU adapter는 네 센서를 포함하는 Suite를 사용한다. 센서 단독 연구나 다중 센서 구성에서는 기체·실험 모델이 개별 센서를 직접 배치할 수 있다. `Examples.QuadImuOnly`는 Suite 없이 plant의 CG 운동과 중력을 IMU에 전달하고, 센서 mount를 합성 CG 기준으로 변환하는 실행 예제다. 센서 자체에는 Suite 의존성이 없다.

센서의 모델링 범위(ADC·register·FIFO 등), 요소별 fidelity(이상식·응답 모델·물리 회로 등), 외부 interface(값·transaction·논리 신호·전기 pin)는 독립적인 선택이다. `Device`는 장치의 의미이며 fidelity 단계가 아니다. 검증한 조합은 각 sensor family에서 profile로 제공하고, 같은 외부 interface를 유지하는 범위에서 내부 구현을 교체한다. 현재 사용자 진입점은 family별 `Sensor.mo`다. 내부 Measurement와 Response를 공통 계약으로 교체하고, 기본 즉시 응답과 채널별 1차 응답을 제공한다. 후속 device/register/EMI placeholder는 만들지 않는다. 개별 물리 부품은 `Physical`, 공통 디지털 동작은 `Logical`, 완성된 센서는 `Systems.Sensing`에 둔다.

FastDyn 값 adapter의 IMU/magnetometer 출력은 sensor mount 회전을 되돌린 뒤 body FRD로 변환한다. GNSS position·velocity는 world ENU→NED이고, altitude/climbRate는 Up positive이다. 자기장 T, gyro rad/s, acceleration m/s²를 그대로 출력한다. Native sensor output record 자체는 각 sensor local axes를 유지한다. Legacy mixed bus는 명시적인 `SensorBusAdapter`로만 구성한다.

신규 `SensorBusAdapter`의 관성·자기 측정값은 body FLU이다. 사용할 sensor의 `R_bImu`·`R_bMag`를 adapter에도 전달해야 하며 기본값은 body와 정렬된 sensor다. 기존 `LowFidelity` 센서의 자체 legacy bus 동작은 변경하지 않는다.

`Logical.Communications`로 이전한 PWM/UART/SPI의 config·bus·partial Link를 보존했다. 아직 payload 전달, baud timing, queue, edge encode/decode를 실행하지 않는다. Physical Electrical 선로, ADC, register/FIFO, host driver, task execution, EMI/vulnerability 주입은 지원 capability로 선언하지 않는다. 실제 firmware driver와 대체 driver를 동시에 실행하지 않는다는 책임 경계는 유지한다.

대표 조립체는 `Measurement → Response → bias → sampling/hold`를 연결한다. Measurement의 가족별 계약과 기본 관측식은 `BaseClasses`에 두고, 경험적 응답은 `Systems.Sensing.ResponseModels`에 둔다. 향후 실제 MEMS·AFE 등의 재사용 가능한 물리 부품과 이 helper를 구분한다. `FirstOrder.tau`가 0인 채널은 즉시 전달하고 양수인 채널은 연속 1차 응답을 계산한다. 초기값은 입력과 같으며, 순수 통신 지연을 의미하지 않는다. 교체 component는 기존 SI 단위·좌표계·채널 순서를 보존해야 한다. 구체적인 설정은 [센서 설명](../Systems/Sensing/README.md)을 따른다.

사용자는 내부 parameter를 설정하거나 `redeclare`로 구현을 바꾼다. 구조 선택은 컴파일 전이며 실행 중 ODR/range 설정 변경을 지원한다는 뜻은 아니다. 실제 변경은 향후 장치 state와 register 동작으로 구현한다. 단순 parameter 차이마다 파일을 추가하지 않고, 검증한 누적 조합이 필요할 때 preset을 제공한다. Register/SPI 등 외부 interface가 달라지면 공통 내부 모델을 사용하는 별도 외부 조립체를 허용한다.

## Migration

| 이전 | 정식 경로 / 호환 방식 |
|---|---|
| `FIRE_Modelica_Update` | `FIRE_Modelica`; 별도 compat package를 로드하면 이전 이름도 사용 가능 |
| `Communications` | `Logical.Communications`; root에 alias |
| `Worlds.Envioronment` | `Worlds.Environment`; 오탈자 alias |
| `Actuators.PwmActuator`, `Adapters.Legacy.PwmActuator` | `Adapters.Legacy.PwmCommandAdapter`; 기존 정규화·sample 식 유지, 이전 이름 별칭 없음 |
| `Actuators.RotaryServo`, `Actuators.RotaryServoActuator` | `Systems.Actuation.RotaryServo`; 기존 구현 유지, 두 이전 이름 별칭 제거 |
| 최상위 `Sensors` | `Systems.Sensing`으로 통합; 이전 최상위 별칭 없이 모든 내부 참조를 변경 |
| `IMU.MountedSampled`, 다른 family의 `Sampled` | 각 family의 `Sensor`; 기본 측정·시간 동작 보존, 이전 이름 별칭 없음 |
| 기존 센서 `LowFidelity` | `Systems.Sensing`의 동일한 family로 이전; 기존 fixed wing·rover에서 동작 유지 |

기존 fixed wing·rover의 운동식이나 sensor mount convention을 새 multirotor convention으로 재해석하지 않았다. 이번 센서 이관에서는 기체의 참조 경로를 변경하고 기존 LowFidelity의 측정·시간 동작을 유지했다. LowFidelity 삭제와 기존 bus 통합은 해당 기체의 변환을 검증한 뒤 별도로 진행한다. 최상위 `Actuators` 별칭은 제거하고, FixedWing·Rover는 `Adapters.Legacy.PwmCommandAdapter`를 직접 참조한다. PWM sampling·정규화식과 기존 `pwmActuator` instance 이름은 유지한다. `deprecated/` 자료도 유지한다. Servo는 기존의 angle saturation + first-order lag이며 별도의 rate limiter가 아니다.

`Systems.Actuation`은 servo 등 actuator 조립을 담당하고, motor·blade 조립은 `Systems.Propulsion`에 둔다. PWM 수치 정규화는 `Adapters`의 책임이다. `PwmCommandAdapter`라는 이름 변경은 PWM edge decoding이나 ESC 물리를 추가한 것이 아니다. 현재 `RotaryServo`의 내부 응답 교체·상세 전기 driver는 후속 확장이고, 이번 변경에서는 기존 각도 제한과 1차 응답을 보존했다.

## 구성과 재현

`tools/generate_config.py`는 TOML을 검증하고 기존 Modelica class의 numeric modifier를 생성한다. 운동방정식은 생성하지 않는다. Geometry·part class·채널 수 변경은 재컴파일 대상이다. Manifest는 생성 시점의 Modelica source digest와 compiler version을 담는다. 수치 parameter의 runtime 수정 가능 여부는 내보낸 FMU metadata에서 별도로 확인해야 한다.

출력 디렉터리는 저장소의 `build/` 내부 또는 저장소 외부만 허용한다. 기존 Modelica 소스와 생성기 소유 표시가 없는 파일은 덮어쓰지 않는다. 공통 arm record의 CG는 armMount의 절반에 두며, 실제 부품 형상에서 질량·관성을 추정하는 기능은 없다. 구성 도구보다 일반적인 geometry는 `Data.AirframeGeometry`를 직접 지정할 수 있다.

`tools/verify.py`는 native 물리 assertions와 조립·legacy 경로를 실행한다. `tools/probe_export.py`는 별도의 FMU build/load/실행 검증이다. Native 성공, FMU 생성 성공, 실제 FMU 실행 성공, FastDyn 통합 성공을 서로 구분한다.
