# Product Catalog

KooCADCAM이 지원하는 제품군과 그 파라메트릭 표현을 정의한다.
현재 1차 목표는 **시계(Watch)** 이며, 스마트폰이 2차, 이후 태블릿·이어버드 케이스 등이 예정된다.
각 제품에 대한 사이즈 엔벨로프, 소재, 피처 인벤토리, 제조 제약을 기술한다.

---

## Watch (1차 타겟)

### 사이즈 엔벨로프

| 항목 | 범위 |
|------|------|
| 케이스 직경 | 36 mm – 46 mm |
| 케이스 두께 | 8 mm – 14 mm |
| 러그-투-러그 길이 | 44 mm – 54 mm |
| 베젤 폭 | 1.5 mm – 4 mm |

### 소재 가정

- 기본: 알루미늄 단조 (Al 6061-T6 또는 Al 7075-T6)
- 옵션: 스테인리스 316L (DFM 규칙 별도 적용)
- 표면 처리: 브러시드, 폴리싱, PVD 코팅 (KooCADCAM 외 공정)

### 피처 인벤토리

| 피처 | 설명 |
|------|------|
| Bezel (베젤) | 상단 링, 회전형/고정형, 딸각홈 포함 가능 |
| Crown (용두) | 우측 3시 방향 기본, 직경 4–8 mm, 나사식 또는 푸시풀 |
| Pushers (푸셔) | 크로노그래프용 좌·우 버튼, 2–4개 |
| Case Middle | 메인 바디, 밴드 러그 일체형 |
| Lug (러그) | 스프링바 홀 포함, 폭 18–24 mm |
| Crystal Seat | 글라스 안착 홈, 가스켓 그루브 |
| Caseback Seat | 나사식/스냅식 백 커버 시트 |
| Spring Bar Hole | 러그 관통홀, 지름 1.5–2.0 mm |
| Pump Hole | 용두 파이프 관통, 防水 가스켓 홈 |

### 제조 제약 (Watch)

- 최소 내경 코너 R ≥ 0.3 mm (엔드밀 한계)
- 베젤 상단 에지 챔퍼 ≥ 0.1 mm (버 방지)
- 러그 스프링바 홀 ± 0.05 mm 위치 공차 (조립 간섭)
- 크라운 파이프 나사 M0.9–M1.2 범위 (탭 가용 범위)
- 자세한 DFM 규칙 목록: [[engine/dfm-rules]]

---

## Smartphone (2차 타겟)

### 사이즈 엔벨로프

| 항목 | 범위 |
|------|------|
| 길이 | 140 mm – 165 mm |
| 폭 | 68 mm – 80 mm |
| 두께 (프론트 프레임) | 6 mm – 10 mm |

### 소재 가정

- 기본: 알루미늄 압출 + 단조 하이브리드 (Apple-class)
- 피니시: 아노다이징, 마이크로 샌드블라스트

### 피처 인벤토리

| 피처 | 설명 |
|------|------|
| Display Pocket | 디스플레이 안착 포켓, 라운드 코너 R1.5–4 mm |
| Camera Holes | 후면 카메라 배열 홀, 원형/타원형 |
| Side Buttons | 볼륨, 전원, 무음 스위치 컷아웃 |
| USB-C Port | 하단 중앙, 커넥터 형상 |
| Speaker Grille | 홀 배열 패턴, 피치 0.8–1.2 mm |
| SIM Tray Slot | 측면 슬롯 |
| MagSafe-class Ring | (옵션) 후면 자석 링 채널 |

### 제조 제약 (Smartphone)

- 디스플레이 포켓 깊이 공차 ± 0.02 mm
- 카메라 홀 진원도 ≤ 0.01 mm
- 스피커 그릴 홀 피치 균일도 ± 0.05 mm
- 세부 규칙: [[engine/feature-phone]]

---

## Future Product Lines

| 제품 | 예상 추가 시기 | 주요 피처 추가 |
|------|---------------|----------------|
| Tablet (태블릿) | M7+ | 대형 디스플레이 포켓, 키보드 커넥터 레일 |
| Earbuds Case | M7+ | 힌지 메커니즘, 충전 핀 배열 |
| Smartwatch (플라스틱 혼합) | 미정 | 복합 소재 인터페이스 |

---

## Common Domain Model

아래 구조체/클래스는 **설계 예정 API** 이며, 현재 `src/` 디렉토리는 비어 있다.
실제 구현 파일이 생성되면 아래 reserved path 링크가 활성화된다.

```cpp
// Reserved path — 아직 미구현
// [[src/engine/ProductCatalog.hpp]]

enum class ProductFamily {
    Watch,
    Smartphone,
    Tablet,
    EarbudsCase
};

enum class MaterialGrade {
    Al6061_T6,
    Al7075_T6,
    SS316L
};

enum class PartRole {
    MetalFront,    // 전면 케이스 (KooCADCAM 1차 대상)
    MetalBack,     // 후면 커버
    Buttons,       // 볼륨·전원 버튼 낱개
    Bracket,       // 내부 브라켓
    Crown,         // 시계 용두
    Bezel          // 시계 베젤 링
};

struct Variant {
    ProductFamily   family;
    std::string     modelId;      // 예: "Watch_46mm_SS316L"
    MaterialGrade   material;
    PartRole        role;
    double          envelopeX_mm; // bounding box X
    double          envelopeY_mm;
    double          envelopeZ_mm;
};

// 파라메트릭 빌드 진입점 (Reserved)
// [[src/engine/WatchFrontModel.hpp#buildBezel]]
// TopoDS_Shape buildBezel(const WatchSpec& spec);
```

구현 로드맵: [[scope/milestones-and-krs]]
파라메트릭 엔진 상세: [[engine/parametric-templates]]

---

## Why Watch First?

1. **형상 복잡도 대비 피처 수 적합**: 시계 케이스는 원통/토러스 기반 형상으로 OCCT B-rep 파이프라인 검증에 이상적이다. 스마트폰 대비 피처 수가 적어 MVP 범위를 통제할 수 있다.
2. **엄격한 치수 공차**: 방수 가스켓·크라운 나사 등 시계 특유의 초정밀 공차(±0.02 mm 급)가 DFM 규칙 엔진 [[engine/dfm-rules]] 의 실질적 테스트베드가 된다.
3. **금속 케이스 시장**: 럭셔리·스포츠 시계 시장에서 알루미늄/스테인리스 금속 케이스 비중이 높으며, 설계 자동화 수요가 명확하다.
4. **소형 = 렌더링 부하 감소**: 4-패널 뷰 [[architecture/multi-view]] 에서 복수 모델을 동시 표시할 때 스마트폰 대비 폴리곤·면 수가 적어 초기 성능 목표 달성이 용이하다.
5. **피처 재사용 가능성**: 베젤·버튼·홀 등 시계 피처 구현체는 스마트폰의 카메라홀·버튼 컷아웃 구현 시 재사용된다.

관련 엔진 노드: [[engine/feature-watch]], [[engine/feature-phone]]
페르소나별 활용: [[scope/personas-and-jobs]]
마일스톤 연계: [[scope/milestones-and-krs#M1 — Watch 파라메트릭 + 단일뷰 GUI]]
