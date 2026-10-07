# Deprecated implementations

활성 패키지는 continuous P0/R0 multicopter, world NED / body FRD와 선택 가능한 4점 탄성 접촉을 제공합니다.
`Chassis/LandingGear`와 `Contact`는 이 archive의 원본에서 재사용하되, FRD 장착 위치와 NED 지면 법선으로
현재 기체에 연결했습니다. 아래 원본 파일은 그대로 보존하며 일반 Terrain과 BodyDrag는 복구하지 않았습니다.

`legacy_enu_flu/fire_modelica_models/`는 최소 구현 적용 전의 **완전한 패키지**입니다.
기존 tracked source 255개를 원본 그대로 이동했으며, 활성 경로에 남은 클래스는
검증된 새 복사본입니다. 같은 namespace의 옛 의존성을 섞지 않고 보관하기 위해
공통 클래스도 함께 보존했습니다. `migration-manifest.json`에 원본 커밋과 파일별
SHA-256, 적용된 활성 파일 목록이 있습니다.

보존한 선택 기능:

- `Vehicles/FixedWing`, `Vehicles/Rover`, 관련 시나리오·명령 mapper·legacy 센서
- 접촉·착륙다리·지형·BodyDrag, 관련 검증과 예제
- sensor sampling/hold/timestamp, Measurement/Response arrays 및 FirstOrder response
- PWM sampling, Logical communications, legacy bus/adapters, servo 및 compatibility namespace
- schema 1/2 Python/Rust 도구, 이전 설정·문서·검증 스크립트

이 archive는 활성 `package.order`에 없고 새 composer staging/source hash에서도 제외됩니다.
새 패키지와 archive를 같은 Modelica 세션에 동시에 로드하지 마세요.
독립 세션에서 archive의 package.mo를 로드하면 이전 ENU/FLU 구현을 참조할 수 있습니다.
이전 config를 새 패키지에 사용하려면 schema 3 및 물리 좌표 변환이 필요합니다.
이전 검증 기록을 새 패키지의 통과 기록으로 해석하지 마세요.

기존 MFQuadrotor.mo, MFRover.mo, MATLAB 파일과 README_bak.md는 그대로 남겼습니다.
이전 deprecated README 원문은 [previous_deprecated_README.md](legacy_enu_flu/previous_deprecated_README.md)에 보존했습니다.
