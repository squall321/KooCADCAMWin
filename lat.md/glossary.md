# Glossary

본 프로젝트에서 자주 나오는 약어/용어. 다른 노드는 용어 등장 시 `[[glossary#용어]]` 로 인용한다.

## CAD 커널 / 형상

### B-rep
Boundary representation. 솔리드를 표면+모서리+꼭짓점의 위상 그래프로 표현. OCCT의 1급 시민. STL/메시는 아님.

### TopoDS_Shape
OCCT의 형상 핸들. compound/solid/shell/face/wire/edge/vertex 의 공통 인터페이스.

### Sewing
서로 일치하지 않는 면 사이의 공유 에지를 봉합해 닫힌 셸을 만드는 과정. 대표 API: `BRepBuilderAPI_Sewing`.

### ShapeFix / ShapeHealing
유효하지 않은 형상(자가 교차, 허용오차 초과, 끊어진 와이어 등)을 자동으로 보수하는 OCCT 패키지군.

### NURBS
Non-Uniform Rational B-Spline. 자유곡면 표현의 표준. OCCT의 B-spline은 `Geom_BSplineCurve` / `Geom_BSplineSurface`.

### EvalRep
OCCT 8.0에서 도입된 지오메트리 평가 디스크립터. 기존 virtual `D0/D1/...` 디스패치를 POD 결과 구조체 + `std::variant` 로 대체.

## 가시화 / 인터랙션

### AIS
Application Interactive Services. OCCT의 시각화 레이어. `AIS_Shape` 를 `AIS_InteractiveContext` 에 등록해서 화면에 띄운다.

### V3d_Viewer / V3d_View
하나의 3D 장면을 다루는 그릇과 그 안의 카메라/뷰포트. **본 프로젝트 패턴: 1 Viewer + N V3d_View** ([[architecture/multi-view]]).

### OpenGl_GraphicDriver
OCCT의 OpenGL 백엔드. Qt 통합 시 `buffersNoSwap=true` 로 swap 책임을 Qt에게 넘긴다.

### OCAF
OCCT Application Framework. `TDocStd_Document` 로 트랜잭션/Undo/XLink 를 제공. 본 프로젝트는 [[architecture/document-model]] 결정에 따라 도입 시점을 미룬다.

### XLink
OCAF 문서 간 참조 어트리뷰트. 레퍼런스 모델 변경 시 파생 모델로 변경 전파.

## 도메인 (메탈 프론트 CAD)

### Front Metal
워치·폰의 외측 메탈 케이스. 디스플레이/베젤이 안착되는 측. 본 프로젝트의 1급 객체 ([[engine/parametric-templates]]).

### Bezel
워치 다이얼 주위의 띠. 폰의 화면 테두리도 베젤이라 부르지만 본 프로젝트에서는 워치 베젤을 우선 지칭.

### Crown
워치 측면의 시계 조작 노브. 캐비티/축 동심도가 까다로움 ([[engine/dfm-rules#DFM-008]]).

### Pocket
재료를 파낸 공간. 디스플레이 안착부, 버튼 안착부 등.

### Anodizing
알루미늄 표면처리. 마스킹 영역과 모서리 R 정책에 영향.

## DFM (Design for Manufacturing)

### DFM
가공·조립 가능성을 설계 단계에서 점검. 본 프로젝트는 22+ 룰을 자동 검증 ([[engine/dfm-rules]]).

### 살두께 (Wall thickness)
인접 면 사이 최소 두께. 워치는 0.35mm, 폰은 0.4mm 기준 ([[engine/dfm-rules#DFM-001]]).

### 언더컷 (Undercut)
드래그 방향(통상 Z+)에서 가공 도구가 닿지 못하는 형상. 3축 가공의 적.

### 5축 가공
공작물 또는 공구가 다축 회전. 측면 포트 홀, 깊은 캐비티 등에 필수. 본 프로젝트의 [[engine/cam-3axis-verify]] 는 **3축 한정 1차 검증**만 다룬다.

## 리버스 엔지니어링

### RANSAC
Random Sample Consensus. 노이즈 섞인 점군에서 강건하게 프리미티브(평면/원통/구/토러스)를 적합. [[engine/reverse-route]] 의 핵심.

### CGAL Shape Detection
RANSAC + 영역 성장 기반 형상 분할 라이브러리. 본 프로젝트의 STL→STEP 파이프라인에서 사용 검토.

### Detessellation
삼각 메시를 분석 가능한 분석면(평면/원통/NURBS)으로 역변환. 자동화율은 60–75%.

## 빌드 / 도구

### vcpkg
Microsoft의 C/C++ 패키지 매니저. OCCT 8.0 공식 포트 추가됨 (Master Spec §2.8).

### LGPL 2.1
OCCT 라이선스. 동적 링크 시 사내 코드 비공개 가능. 정적 링크 시 바이너리 전체 LGPL 적용.

### lat.md
[1st1/lat.md](https://github.com/1st1/lat.md). 본 프로젝트 문서의 메타 포맷. `[[wikilink]]` + `// @lat: [[id]]` 컨벤션. CLI 미도입.

---

> 추가 용어가 등장하면 자유롭게 이 문서에 섹션을 추가. 알파벳 정렬 강제는 없음; 카테고리 묶음 유지.
