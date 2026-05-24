# KooCADCAM — Knowledge Graph Index

> **Project**: 메탈 프론트 CAD 설계자동화 솔루션
> **Stack**: OCCT 8.0.0 + Qt 6 + C++17 (Windows + Linux 멀티플랫폼; Windows = VS 2022 IDE, Linux = CMake CLI)
> **1차 타겟**: 워치 → 스마트폰 → (확장: 태블릿/이어버드 등)
> **Master Spec (원본 보존)**: [smartphone_metal_cad_project.md](../smartphone_metal_cad_project.md)

이 디렉토리는 [1st1/lat.md](https://github.com/1st1/lat.md) 컨벤션을 따르는 지식 그래프다. Node.js CLI는 미도입; 포맷만 준수해 향후 `npx lat.md check` 통과 가능하도록 작성됨.

---

## 🗺️ Lanes

### Scope (무엇을, 누구를 위해, 언제까지)
- [[scope/product-catalog]] — 워치·폰 등 제품 카탈로그와 공통 도메인 모델
- [[scope/personas-and-jobs]] — 설계자/생산기술 사용자 시나리오
- [[scope/milestones-and-krs]] — M1~M7 정량 KR

### Architecture (어떻게 구성되어 있나)
- [[architecture/overview]] — 시스템 블록도 (UI / Engine / OCCT)
- [[architecture/multi-view]] — 1 GraphicDriver + N V3d_View 패턴
- [[architecture/multi-document]] — AIS context per document
- [[architecture/document-model]] — 자체 Doc vs OCAF 결정
- [[architecture/build-and-deps]] — CMake/vcpkg/MSVC2022/Qt6/OCCT8

### Engine (실제로 만드는 형상과 로직)
- [[engine/parametric-templates]] — Watch/Phone 공통 Feature 베이스
- [[engine/feature-watch]] — `WatchFrontModel` API + JSON Schema (1차 타겟)
- [[engine/feature-phone]] — `PhoneFrontModel` (Master Spec §6.3 이식)
- [[engine/morphing-route]] — 외곽면 모핑 (한계 + 데모로 강등)
- [[engine/reverse-route]] — STL → STEP 복원 (CGAL Shape Detection + OCCT)
- [[engine/dfm-rules]] — DFM 룰 카탈로그 (22+ 항목)
- [[engine/cam-3axis-verify]] — OCL toolpath + OCCT `BRepExtrema` 간섭검사

### Process (어떻게 일하나)
- [[process/test-strategy]] — 단위/통합/회귀 + 골든 모델
- [[process/data-assets]] — 외부 STEP/STL/부품 카탈로그·라이선스
- [[process/coding-standards]] — C++17 + OCCT 8.0 + clang-format
- [[process/occt8-migration-cookbook]] — 7.x → 8.0 함정

---

## 📚 Cross-cuts
- [[glossary]] — 용어집 (B-rep, AIS, DFM, EvalRep, OCAF, V3d_View …)

---

## 🚦 Reading Order (신규 합류자용)
1. [[scope/milestones-and-krs]] 로 “지금 어디”를 본다
2. [[architecture/overview]] 로 큰 그림
3. 자신의 작업 레인의 노드들로 진입
4. 모르는 용어는 [[glossary]]
5. 원본의 의도가 궁금하면 [Master Spec](../smartphone_metal_cad_project.md) — 단, **lat.md/ 가 살아있는 단일 사실원천**이며 원본은 기록 보관용이다 (2026-05-25 스냅샷)

---

## 🔗 코드 ↔ 문서 연결 규약
- 향후 `src/` 트리에 코드가 들어가면, 각 파일에 `// @lat: [[engine/feature-watch#Build Sequence]]` 같은 어노테이션을 단다
- 위키링크 안에서 코드 심볼 인용은 `[[src/engine/WatchFrontModel.hpp#buildBezel]]` 형식
- 현재는 `src/` 가 비어 있으므로 모든 코드 인용은 **예약된 경로** 표기
