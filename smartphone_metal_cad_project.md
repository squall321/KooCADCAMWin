# 스마트폰 프론트 메탈 CAD 설계자동화 프로젝트
> 기술 스택 검토 및 아키텍처 설계 문서  
> 기준 버전: OpenCASCADE Technology 8.0.0

---

## 목차
1. [기술 스택 결정](#1-기술-스택-결정)
2. [OpenCASCADE 8.0.0 주요 변경사항](#2-opencascade-800-주요-변경사항)
3. [OCCT vs CadQuery 속도/라이선스 비교](#3-occt-vs-cadquery-속도라이선스-비교)
4. [GUI 프레임워크: Qt 선택 근거](#4-gui-프레임워크-qt-선택-근거)
5. [CAM 연동 검토](#5-cam-연동-검토)
6. [설계자동화 아키텍처](#6-설계자동화-아키텍처)
7. [리버스 엔지니어링 파이프라인](#7-리버스-엔지니어링-파이프라인)
8. [다음 단계: OCCT 8.0.0 빌드](#8-다음-단계-occt-800-빌드)

---

## 1. 기술 스택 결정

### 확정된 스택

| 항목 | 선택 | 이유 |
|---|---|---|
| CAD 커널 | **OCCT 8.0.0** | 오픈소스 최강 B-rep 커널, 직접 제어 가능 |
| GUI 프레임워크 | **Qt 6.x** | OCCT 공식 연동, 크로스플랫폼, 팀 경험 보유 |
| 개발 언어 | **C++17** | OCCT 8.0 기준 언어, 최고 성능 |
| 지원 플랫폼 | **Windows + Linux** | Qt + OCCT 조합으로 동일 코드베이스 |
| CAM 커널 | **불필요** (설계자동화 단계) | 형상 생성/검증은 OCCT로 충분 |

### 선택하지 않은 것들

| 후보 | 제외 이유 |
|---|---|
| CadQuery | Python 래퍼 → C++ 스택과 혼용 불가, 실시간 GUI 연동 불리 |
| ImGui | CAD급 복잡한 UI 구성 한계 |
| wxWidgets | 생태계/문서 Qt 대비 빈약 |
| Electron/웹 | C++/OCCT 연동 구조적 불편 |
| ModuleWorks | 설계자동화 단계에서 과잉, 고비용 |

---

## 2. OpenCASCADE 8.0.0 주요 변경사항

> 7.9.0 대비 500개 이상 변경. 수년 만의 최대 메이저 업데이트.  
> 5개 RC + 3개 Beta 빌드를 거쳐 최종 릴리즈.

### 2.1 C++17 필수화

```
- 최소 C++ 표준: C++17 (C++14 이하 컴파일러 지원 종료)
- 코드 전반 사용: if constexpr, std::optional, std::variant,
  std::string_view, std::shared_mutex, structured bindings, fold expressions
```

### 2.2 소스 트리 재구성

```
변경 전: 기존 OCCT 디렉토리 구조
변경 후: src/Module/Toolkit/Package/File
         리소스: 최상위 /resource 디렉토리로 이동
```

### 2.3 예외 처리 변경 (Breaking Change)

```cpp
// Before (7.x)
Standard_Failure::Raise("error");   // Raise() 사용
Standard_Failure::Instance();       // Instance() 사용

// After (8.0)
throw Standard_Failure("error");    // 표준 throw 사용
// Standard_Failure는 이제 std::exception 파생
// what(), ExceptionType() 노출
```

### 2.4 수학 함수 표준화 (Migration 필요)

```cpp
// Before (deprecated)          // After
ACos(x)                    →    std::acos(x)
Sqrt(x)                    →    std::sqrt(x)
Sin(x) / Cos(x)            →    std::sin(x) / std::cos(x)
Abs(x)                     →    std::abs(x)
Standard_UNUSED            →    [[maybe_unused]]
```

### 2.5 지오메트리 평가 아키텍처 개편

```
변경 전: 가상 D0/D1/D2/D3 메서드 (virtual dispatch)
변경 후: EvalD* API (POD result structs)
         std::variant dispatch → 가상화 제거
         EvalRep descriptor system → 기하학적 정체성/평가 전략 분리
```

### 2.6 성능 최적화

| 항목 | 내용 |
|---|---|
| 행렬 곱셈 | i-j-k → i-k-j 루프 순서 변경 (캐시 효율) |
| 벡터 노름 | 4-way 루프 언롤링 + pairwise partial sum |
| 원자 참조 카운팅 | 명시적 memory ordering 적용 |
| 다항식 솔버 | MathPoly_Laguerre 추가 (Laguerre + deflation) |

### 2.7 NCollection_Array1/2::Assign 변경 주의

```
⚠️  Silent Breaking Change
- 코드는 컴파일되지만 런타임 동작이 달라짐
- 기존에 destination 배열이 자신의 범위를 유지하는 것에 의존하던 코드 영향
- 마이그레이션 가이드 필수 확인:
  https://dev.opencascade.org/doc/overview/html/occt__upgrade.html
```

### 2.8 신규 기능

- **TKHelix 툴킷**: 기하학적 헬릭스 곡선 어댑터, B-스플라인 근사, TCL 인터페이스
- **occ:: 네임스페이스** 도입
- **VCPKG 포트** 추가 (`opencascade`, TclTk, GTest 지원)

---

## 3. OCCT vs CadQuery 속도/라이선스 비교

### 3.1 구조적 관계

```
CadQuery
    └─ Python API
        └─ OCP (OCCT Python 바인딩)
            └─ OpenCASCADE Technology (C++ 커널)  ← 실제 연산
```

**형상 연산 커널은 동일 → 이론상 연산 속도 같음**

### 3.2 실제 속도 차이가 나는 지점

| 항목 | OCCT 직접 (C++) | CadQuery (Python) |
|---|---|---|
| 형상 연산 (Boolean, Fillet 등) | 기준 | 거의 동일 |
| 바인딩 오버헤드 | 없음 | 미미 (~수 ms) |
| 루프 안에서 반복 연산 | 빠름 | Python 루프면 느려짐 |
| 메모리 제어 | 직접 가능 | GC 의존 |
| 렌더링 파이프라인 통합 | 직접 최적화 | 별도 처리 필요 |

### 3.3 라이선스 비교

| 항목 | OCCT | CadQuery |
|---|---|---|
| 라이선스 | **LGPL 2.1** | **Apache 2.0** |
| 상용 사내 툴 | ✅ 가능 | ✅ 가능 |
| 소스 비공개 | ✅ (동적 링크 시) | ✅ |
| OCCT 수정 시 | 수정본 공개 의무 | — |
| 특허 조항 | 없음 | Apache 2.0 특허 허여 포함 |

### 3.4 LGPL 2.1 실무 운용 방침

```
✅ 동적 링크 (DLL/SO) → 자사 코드 비공개 가능  (권장)
⚠️ 정적 링크          → 전체 바이너리 LGPL 적용
⚠️ OCCT 소스 수정     → 수정된 OCCT 부분만 공개 의무

→ 사내 툴 + 동적 링크 유지 시 라이선스 이슈 없음
```

---

## 4. GUI 프레임워크: Qt 선택 근거

### 4.1 OCCT와의 궁합

```
- OCCT 공식 샘플/문서가 Qt 기반으로 작성
- FreeCAD, Salome 등 주요 오픈소스 CAD가 Qt + OCCT 스택
- AIS_InteractiveContext, V3d_View → QOpenGLWidget 임베드 패턴 정립
```

### 4.2 Qt 버전 선택

| | Qt 5.15 LTS | Qt 6.x |
|---|---|---|
| C++17 지원 | 일부 | 완전 |
| OCCT 8.0 연동 | 가능 | **권장** |
| 장기 지원 | 종료 임박 | 진행 중 |

> **→ OCCT 8.0 + Qt 6.x 조합 권장** (둘 다 C++17 기반)

### 4.3 주요 활용 Qt 컴포넌트

| 기능 | Qt 컴포넌트 |
|---|---|
| 3D 뷰포트 | `QOpenGLWidget` + OCCT V3d_View |
| 형상 트리 | `QTreeView` + 커스텀 모델 |
| 프로퍼티 패널 | `QDockWidget` |
| 파라미터 입력 | `QFormLayout` + 각종 위젯 |
| Undo/Redo | `QUndoStack` |
| 이벤트 연동 | Qt 이벤트 루프 → OCCT 마우스/키보드 인터랙션 |

### 4.4 전체 UI 구조

```
┌─────────────────────────────────┐
│         Qt Main Window          │
│  ┌──────────┐  ┌─────────────┐  │
│  │  Shape   │  │ QOpenGL     │  │
│  │  Tree    │  │ Widget      │  │
│  │(QTree    │  │ (OCCT V3d   │  │
│  │  View)   │  │  Viewer)    │  │
│  └──────────┘  └─────────────┘  │
│  ┌───────────────────────────┐  │
│  │   Property / Param Panel  │  │
│  │   (QDockWidget)           │  │
│  └───────────────────────────┘  │
└─────────────────────────────────┘
         │
    OCCT Kernel
  (BRep, BOP, AIS...)
```

---

## 5. CAM 연동 검토

### 5.1 스마트폰 프론트 메탈 공정

```
알루미늄 압출/단조 소재
    ├─ 1. CNC 황삭 (3축)
    ├─ 2. CNC 정삭 (3~5축)  ← 카메라홀, 버튼 포켓
    ├─ 3. 드릴링 / 탭핑
    ├─ 4. 언더컷 가공         ← 5축 또는 인덱싱 필수
    ├─ 5. 아노다이징 전 연마
    └─ 6. 검사
```

> **5축 가공 필수** — 측면 포트 홀, 버튼 캐비티, 모서리 라운드

### 5.2 CAM 옵션 비교

| 옵션 | 적합 가공 | 라이선스 | 비용 |
|---|---|---|---|
| OCCT BRepCAM | 2.5D 이하 | LGPL | 무료 |
| OCCT + Clipper2 | 2.5D 포켓/윤곽 | LGPL + BSL-1.0 | 무료 |
| OpenCAMLib | 3축 자유곡면 | LGPL | 무료 |
| **ModuleWorks** | **3~5축 고품질** | 상용 | 💰💰💰 |

### 5.3 결론: 설계자동화 단계에서는 CAM 커널 불필요

```
현재 목표: 설계자동화 (형상 생성 + DFM 검증)
           ↓
OCCT만으로 충분

향후 CAM 연동 시:
  - 2.5D: OCCT + Clipper2
  - 5축 고품질: ModuleWorks (비용 협의)
  - 또는 기존 상용 CAM 툴과 STEP 연동
```

---

## 6. 설계자동화 아키텍처

### 6.1 자동화 대상 분류

```
A. 파라메트릭 형상 생성    ← 입력값 → 3D 모델 자동생성   (핵심)
B. 설계 규칙 검증 (DFM)   ← 가공 가능 여부 자동 체크     (핵심)
C. 공정 연계 자동화        ← 모델 → CAM 파라미터 자동설정 (2단계)
D. 도면/BOM 자동출력       ← 모델 → 2D 도면 자동생성     (3단계)
```

### 6.2 스마트폰 메탈 파라미터 분류

| 고정 요소 (템플릿) | 변동 요소 (입력 파라미터) |
|---|---|
| 전체 외곽 형상 패턴 | 기종별 크기 (6.1" / 6.7" 등) |
| 카메라홀 위치/패턴 | 카메라 구성 (1~4개) |
| 버튼 포켓 구조 | 버튼 위치/개수 |
| 스피커/마이크 홀 패턴 | 커넥터 타입 |
| 모서리 R값 범위 | 두께 |
| 아노다이징 기준선 | 색상/표면처리 스펙 |

### 6.3 Feature 기반 모델링 구조

```cpp
class FrontMetalModel {
    // 1단계: 기본 블록
    void buildBase(double width, double height, double thickness);

    // 2단계: 외곽 라운드
    void applyCornerRadius(double r);

    // 3단계: 내부 포켓 (디스플레이 안착부)
    void buildDisplayPocket(DisplaySpec spec);

    // 4단계: 카메라 홀
    void addCameraHoles(std::vector<CameraSpec> cameras);

    // 5단계: 버튼 포켓
    void addButtonFeatures(std::vector<ButtonSpec> buttons);

    // 6단계: 포트 홀
    void addPortFeatures(PortSpec port);

    // 7단계: DFM 검증
    DFMReport validateDesign();
};
```

### 6.4 DFM 자동 검증 항목

| 검증 항목 | 기준 예시 | OCCT API |
|---|---|---|
| 최소 살두께 | ≥ 0.4mm | `BRepExtrema` |
| 최소 홀 직경 | ≥ φ0.8 | Feature 파라미터 |
| 홀 간 최소 거리 | ≥ 1.5mm | `BRepExtrema` |
| 모서리 R 최솟값 | ≥ R0.2 | `BRepCheck` |
| 공구 접근 각도 | 기계 스펙 기준 | 법선 벡터 분석 |
| 언더컷 감지 | 자동 플래그 | 슬라이싱 분석 |

### 6.5 전체 시스템 아키텍처

```
┌──────────────────────────────────────────┐
│              Qt GUI                       │
│                                           │
│  ┌─────────────────┐  ┌───────────────┐  │
│  │  파라미터 입력   │  │  3D 프리뷰    │  │
│  │                 │  │  (OCCT AIS)   │  │
│  │  • 기종 선택     │  │               │  │
│  │  • 카메라 구성   │  │  실시간 업데이트│ │
│  │  • 버튼 위치     │  │               │  │
│  │  • 치수 입력     │  │               │  │
│  └────────┬────────┘  └───────────────┘  │
└───────────┼──────────────────────────────┘
            │
┌───────────▼──────────────────────────────┐
│         설계 엔진 (C++)                   │
│                                           │
│  ┌─────────────┐    ┌─────────────────┐  │
│  │  파라메트릭  │    │   DFM 검증      │  │
│  │  모델 생성   │    │                 │  │
│  │  (Feature   │    │  • 최소 살두께   │  │
│  │   기반)     │    │  • 공구 접근성   │  │
│  └──────┬──────┘    │  • R값 범위     │  │
│         │           │  • 홀 간격      │  │
│         │           └─────────────────┘  │
└─────────┼────────────────────────────────┘
          │
┌─────────▼──────────────┐
│      OCCT 8.0 커널      │
│                         │
│  BRepBuilderAPI  형상생성│
│  BRepFilletAPI   필렛   │
│  BRepAlgoAPI     Boolean│
│  BRepCheck       검증   │
│  BRepExtrema     측정   │
└─────────┬───────────────┘
          │
    ┌─────┴──────┐
    │    출력     │
    ├─ STEP/IGES  │  → CAM 툴 전달
    ├─ DXF        │  → 도면
    └─ BOM JSON   │  → ERP 연동
```

### 6.6 도입 로드맵

```
1단계 (약 3개월)
  └─ 파라메트릭 형상 생성 + 3D 프리뷰
      → 기종별 STEP 자동 출력

2단계 (약 2개월)
  └─ DFM 검증 룰 추가
      → 설계 오류 사전 차단

3단계 (약 2개월)
  └─ STEP → CAM 툴 자동 전달
      → 공정 파라미터 템플릿 연동

4단계 (이후)
  └─ ERP/PLM 연동, BOM 자동생성
```

---

## 7. 리버스 엔지니어링 파이프라인

기존 스마트폰 STEP 모델에서 파라미터를 역산해 새 기종 설계를 자동 생성하는 파이프라인.

### 7.1 전체 흐름

```
기존 스마트폰 모델 (STEP/IGES)
         │
         ▼
  ① 형상 인식 & 분해 (Feature Recognition)
         │
         ▼
  ② 파라미터 추출 (치수, 위치, 규칙 역산)
         │
         ▼
  ③ 설계 규칙 역산 (비율/관계식 일반화)
         │
         ▼
  ④ 신규 파라미터 입력
         │
         ▼
  ⑤ 새 스마트폰 형상 생성
```

### 7.2 Feature Recognition 전략

스마트폰 전용 규칙 기반 인식 (도메인 특화 접근):

```
인식 규칙 예시:
  원통면 + 직경 3~15mm + 상면 관통      → 카메라홀 후보
  직사각형 포켓 + 측면 위치 + 깊이<1mm  → 버튼 포켓 후보
  대형 직사각형 포켓 + 중앙 위치         → 디스플레이 안착부
  관통홀 + 하단 위치 + 타원형            → 스피커홀
  관통홀 + 측면 + 특정 형상             → 포트 홀
```

### 7.3 파라미터 추출 (OCCT API 활용)

```cpp
struct ExtractedParams {
    // 외곽 형상
    double overall_width;       // BRep 바운딩박스
    double overall_height;
    double overall_thickness;
    double corner_radius;       // BRepAdaptor_Surface 분석

    // 카메라
    struct CameraHole {
        gp_Pnt  center;         // BRepGProp 무게중심
        double  diameter;       // BRepAdaptor_Curve
        double  depth;          // BRepExtrema
    };
    std::vector<CameraHole> cameras;

    // 버튼 포켓
    struct ButtonPocket {
        gp_Pnt  position;
        double  width, height, depth;
        Side    side;           // 좌/우/상/하 판별
    };
    std::vector<ButtonPocket> buttons;
};
```

### 7.4 설계 규칙 역산 (비율화)

```
추출된 치수                   →   역산된 규칙 (일반화)
─────────────────                 ──────────────────────────
카메라홀 중심 X: 18.3mm      →   본체 폭 대비 비율: 0.231
버튼 포켓 Y: 42.1mm          →   상단에서 전체 높이의 26%
필렛 R: 2.5mm                →   모서리 R = 두께 × 1.04
살두께: 0.45mm               →   최소 살두께 = 두께 × 0.188
```

→ 비율/관계식으로 일반화하면 다른 크기 기종에도 자동 적용 가능

### 7.5 단계별 난이도

| 단계 | 난이도 | 비고 |
|---|---|---|
| STEP 파싱 & 시각화 | ⭐⭐ | OCCT가 잘 지원 |
| 기하학적 측정 (치수 추출) | ⭐⭐⭐ | OCCT API로 가능 |
| Feature 자동 인식 | ⭐⭐⭐⭐ | 규칙 기반은 가능, 완전 자동은 어려움 |
| 설계 규칙 역산 | ⭐⭐⭐⭐ | 도메인 지식 + 통계 필요 |
| 신규 형상 생성 | ⭐⭐⭐ | 파라메트릭 엔진 구현 |
| **완전 자동화** | ⭐⭐⭐⭐⭐ | 현실적 목표는 **반자동** |

### 7.6 현실적 자동화 목표

```
완전 수동 (현재)
  설계자가 모든 치수 입력 → 수일 소요
        │
        ▼
반자동 (1차 목표)
  STEP 불러오기
  → 자동 인식 ~80% + 수동 보정 ~20%
  → 파라미터 자동 추출
  → 신규 형상 생성
  → 수시간으로 단축
        │
        ▼
고도 자동화 (ML 보강 후)
  인식률 95%+, 규칙 자동 학습
```

### 7.7 ML 보조 인식 (향후)

```
Point Cloud / B-rep 입력
        │
   Graph Neural Network
   (BRepNet, UV-Net 등)
        │
   Feature 분류 결과
```
> 학습 데이터: 레이블링된 스마트폰 STEP 파일 필요

---

## 8. 다음 단계: OCCT 8.0.0 빌드

### 8.1 사전 요구사항

| 항목 | 요구사항 |
|---|---|
| C++ 컴파일러 | MSVC 2019+, GCC 9+, Clang 10+ (C++17 지원) |
| CMake | 3.14 이상 |
| Qt | 6.x (Qt 5.15도 가능) |
| OpenGL | 3.2+ |
| 선택 의존성 | FreeType, FreeImage, TBB, VTK |

### 8.2 소스 취득

```bash
git clone https://github.com/Open-Cascade-SAS/OCCT.git
cd OCCT
git checkout V8_0_0
```

### 8.3 빌드 (CMake)

```bash
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MODULE_ApplicationFramework=ON \
  -DBUILD_MODULE_Draw=OFF \
  -DBUILD_MODULE_Visualization=ON \
  -D3RDPARTY_QT_DIR=/path/to/Qt6 \
  -DCMAKE_INSTALL_PREFIX=/opt/occt800

cmake --build . --parallel $(nproc)
cmake --install .
```

### 8.4 마이그레이션 주의사항 (7.x → 8.0)

```
1. C++17 모드 활성화 필수
   - CMakeLists.txt: set(CMAKE_CXX_STANDARD 17)

2. Standard_Failure 예외 처리 변경
   - Raise() → throw
   - Instance() 제거

3. 수학 함수 치환
   - 마이그레이션 스크립트 제공:
     adm/scripts/migration_800/replace_typedefs.py

4. NCollection_Array1/2::Assign 동작 변경
   - 런타임 테스트 필수

5. Standard_UNUSED → [[maybe_unused]]
   - 수동 마이그레이션 필요 (위치 규칙 엄격화)

6. 소스 디렉토리 구조 변경
   - src/Module/Toolkit/Package/File 구조
   - include 경로 업데이트 필요
```

### 8.5 Qt + OCCT 3D 뷰어 기본 연동

```cpp
// QOpenGLWidget 상속하여 OCCT 뷰어 연동
class OcctViewer : public QOpenGLWidget {
    Handle(AIS_InteractiveContext) myContext;
    Handle(V3d_View)               myView;

protected:
    void initializeGL() override {
        // OCCT OpenGL 컨텍스트 초기화
        Handle(OpenGl_GraphicDriver) driver =
            new OpenGl_GraphicDriver(/* display */);
        // V3d_Viewer, V3d_View 생성
    }

    void paintGL() override {
        myView->Redraw();
    }

    void resizeGL(int w, int h) override {
        myView->MustBeResized();
    }

    void mousePressEvent(QMouseEvent* e) override {
        // AIS_InteractiveContext 마우스 이벤트 전달
    }
};
```

### 8.6 참고 링크

| 자료 | URL |
|---|---|
| OCCT GitHub | https://github.com/Open-Cascade-SAS/OCCT |
| 공식 문서 | https://dev.opencascade.org |
| 마이그레이션 가이드 | https://dev.opencascade.org/doc/overview/html/occt__upgrade.html |
| 릴리즈 노트 | https://github.com/Open-Cascade-SAS/OCCT/releases/tag/V8_0_0 |
| Qt 공식 | https://www.qt.io |

---

## 부록: 핵심 OCCT API 요약

| 목적 | 주요 API |
|---|---|
| 형상 생성 | `BRepBuilderAPI_MakeBox`, `BRepPrimAPI_MakeCylinder` |
| 필렛/챔퍼 | `BRepFilletAPI_MakeFillet`, `BRepFilletAPI_MakeChamfer` |
| Boolean 연산 | `BRepAlgoAPI_Fuse`, `BRepAlgoAPI_Cut`, `BRepAlgoAPI_Common` |
| 형상 검증 | `BRepCheck_Analyzer` |
| 거리/치수 측정 | `BRepExtrema_DistShapeShape` |
| 면 분석 | `BRepAdaptor_Surface`, `BRepAdaptor_Curve` |
| 무게중심 | `BRepGProp`, `GProp_GProps` |
| STEP 입출력 | `STEPControl_Reader`, `STEPControl_Writer` |
| DXF 출력 | `IFSelect_ReturnStatus` + DataExchange |
| 시각화 | `AIS_InteractiveContext`, `AIS_Shape`, `V3d_View` |
| 선택/하이라이트 | `AIS_InteractiveContext::Select`, `HilightWithColor` |

---

*문서 작성일: 2026-05-25*  
*기준 버전: OCCT 8.0.0, Qt 6.x, C++17*
