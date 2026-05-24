# Data Assets

KooCADCAM 프로젝트에서 사용하는 외부 데이터 자산의 카탈로그 및 관리 정책을 정의한다.
모든 자산은 `data/` 디렉토리 트리 아래에 저장된다.

> **RESERVED PATH**: `data/` 는 현재 비어 있는 예약 경로다. 아직 실제 파일이 없으며,
> 아래 규약에 따라 단계적으로 채워진다. `lat.md/` 하위가 **아님** — 프로젝트 루트에 위치.

---

## 디렉토리 레이아웃

```
data/
├── manifest.json              ← SHA-256 해시 레지스트리 (CI 검증 기준)
├── watch/
│   ├── reference_step/        ← 기존 워치 케이스 레퍼런스 STEP
│   ├── peripheral/            ← 디스플레이/배터리 등 주변부품 바운즈
│   └── scan/                  ← RE 입력용 STL 스캔 데이터
├── phone/
│   ├── reference_step/
│   ├── peripheral/
│   └── scan/
└── common/
    └── tooling/               ← 공용 공구 인벤토리 스펙 (JSON)
```

---

## 파일 명명 규약

형식: `<product>_<variant>_<rev>_<role>.<ext>`

| 토큰 | 허용 값 (예시) | 설명 |
|---|---|---|
| `product` | `watch44`, `watch46`, `phone61`, `phone67` | 제품 + 크기 코드 |
| `variant` | `alu`, `ti`, `ss` | 소재 (알루미늄 / 티타늄 / 스테인리스) |
| `rev` | `r01`, `r02`, `r03` | 리비전 번호 (3자리 제로패딩) |
| `role` | `metalfront`, `metalback`, `display_bounds`, `cam_bounds`, `battery_bounds` | 데이터 역할 |
| `ext` | `.step`, `.stl`, `.ply`, `.json` | 파일 포맷 |

**예시**

```
watch44_alu_r03_metalfront.step
phone67_ti_r01_display_bounds.json
watch46_ss_r02_scan.ply
```

---

## 자산 유형별 카탈로그

### 레퍼런스 STEP 파일

기존 워치/스마트폰 메탈 케이스 STEP 모델. 통합 테스트 베이스라인 및
리버스 엔지니어링 입력으로 사용된다([[process/test-strategy]]).

| 제품 | 소재 | 역할 | 경로 |
|---|---|---|---|
| 워치 44mm | 알루미늄 | metalfront | `data/watch/reference_step/watch44_alu_r01_metalfront.step` |
| 워치 46mm | 알루미늄 | metalfront | `data/watch/reference_step/watch46_alu_r01_metalfront.step` |
| 스마트폰 6.1" | 알루미늄 | metalfront | `data/phone/reference_step/phone61_alu_r01_metalfront.step` |
| 스마트폰 6.7" | 알루미늄 | metalfront | `data/phone/reference_step/phone67_alu_r01_metalfront.step` |

> **개인정보/경쟁사 주의**: 레퍼런스 STEP이 경쟁사 파생 모델인 경우 출처를
> 이 표에 기록한다 (공개 CAD / NDA 클리어 여부). 현재는 플레이스홀더.

### STL/PLY 스캔 데이터

리버스 엔지니어링 파이프라인([[engine/reverse-route]])의 입력으로 사용되는 3D 스캔 파일.
실제 부품 스캔 또는 포토그래메트리 결과물이다.

### 주변 부품 바운즈 (Peripheral Bounds)

디스플레이 모듈, 카메라 모듈, 배터리의 **외형 바운딩 박스 + 장착 위치**만 포함.
내부 구조/회로 정보는 포함하지 않는다. JSON 포맷.

```json
{
  "product": "watch44",
  "component": "display",
  "revision": "r01",
  "bounds_mm": { "x": 36.2, "y": 42.8, "z": 1.5 },
  "origin_mm": { "x": 4.1, "y": 3.0, "z": 0.0 }
}
```

---

## SHA-256 매니페스트

`data/manifest.json`에 모든 자산의 SHA-256을 기록한다.
CI에서 실제 파일 해시와 매니페스트를 비교하여 **불일치 시 빌드 실패**.

```json
{
  "version": 1,
  "assets": [
    {
      "path": "watch/reference_step/watch44_alu_r01_metalfront.step",
      "sha256": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
      "source": "public_cad_archive",
      "nda_cleared": true
    }
  ]
}
```

매니페스트 업데이트 절차:
1. 파일을 `data/` 에 추가
2. `tools/update_manifest.py` 실행 → `manifest.json` 자동 갱신
3. 두 파일을 동시에 커밋

---

## 라이선스 매트릭스

| 컴포넌트 | 라이선스 | 링크 방식 | 소스 공개 의무 | 비고 |
|---|---|---|---|---|
| **OCCT 8.0** | LGPL 2.1 | 동적 링크 (강제) | OCCT 수정분만 공개 | 프로젝트 전 스택 핵심 |
| **Qt 6.x** | LGPL 3 / 상용 | 동적 링크 | Qt 수정분만 공개 | 오픈소스 에디션 사용 시 |
| **CGAL** | GPL 2+ / LGPL 3 | **주의** | GPL 컴포넌트 사용 시 전체 공개 | Shape Detection(GPL) → 동적 링크 샌드박스 필수 |
| **OpenCAMLib** | LGPL 2.1+ | 동적 링크 | OpenCAMLib 수정분만 공개 | CAM 2단계 |
| **libIGL** | MPL 2.0 / GPL(일부 데모) | 헤더 온리 (MPL 파트) | MPL: 수정 파일만 공개 | GPL 데모 파일 제외할 것 |
| **PCL** | BSD 3-Clause | 동적 링크 | 없음 | RE 파이프라인 |
| **spdlog** | MIT | 헤더 온리 | 없음 | 로깅 |
| **GoogleTest** | BSD 3-Clause | 테스트 빌드만 | 없음 | vcpkg `gtest` 포트 |
| **tl::expected** | CC0 1.0 | 헤더 온리 | 없음 | 에러 모델 |

### LGPL 실무 운용 요약

```
OCCT, Qt, OpenCAMLib:
  ✅ 동적 링크(.dll/.so) → 자사 코드 비공개 가능
  ⚠️ 정적 링크           → 해당 컴포넌트의 LGPL 조건이 전체에 전파
  ⚠️ OCCT/Qt 소스 수정   → 수정된 부분만 공개 의무

CGAL Shape Detection (GPL):
  ✅ 독립 프로세스 또는 동적 로드(.dll)로 격리 → GPL 전파 차단
  ❌ 정적 링크 또는 동일 번역 단위(TU) 포함   → 프로젝트 전체 GPL 적용
```

---

## 라이선스 컴플라이언스 CI 게이트

`vcpkg install` 트리를 스캔하여 허용 목록 외 컴포넌트 등장 시 빌드 실패.

```cmake
# CMakeLists.txt - 커스텀 타겟 예시
add_custom_target(license_check
    COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/check_licenses.py
            ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}
            ${CMAKE_SOURCE_DIR}/tools/license_allowlist.json
    COMMENT "Scanning vcpkg tree for license compliance"
)
add_dependencies(KooCADCAM license_check)
```

허용 목록(`tools/license_allowlist.json`)에 새 컴포넌트를 추가하려면
라이선스 리뷰 후 PR 설명에 `[license-ack]` 태그를 붙여야 한다.

---

## 버전 관리 정책

- 소규모 자산(< 5MB): Git LFS 사용
- 대용량 스캔 데이터(> 50MB): 팀 내부 파일 서버 + `manifest.json` 해시로 추적
- 모든 자산은 `manifest.json`에 등록 없이 `data/` 에 직접 커밋 금지

---

## 크로스-링크 요약

- [[engine/reverse-route]] — RE 파이프라인, STL/PLY 스캔 소비
- [[process/test-strategy]] — 황금 STEP 기반 통합 테스트, SHA-256 활용
- [[architecture/build-and-deps]] — vcpkg, CMake 빌드 및 라이선스 스캔 도구
