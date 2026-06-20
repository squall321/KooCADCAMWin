#pragma once
// @lat: [[architecture/overview#src/gui/]]

#include <QObject>

class QMenuBar;
namespace koocadcam::gui {

class MainWindow;

class AppMenus : public QObject
{
    Q_OBJECT
public:
    explicit AppMenus(MainWindow* parent);
    void install(QMenuBar* menuBar);

private slots:
    void onNewSampleCylinder();
    void onSaveStep();
    void onExportGCode();
    void onOpenStep();
    void onFitAll();
    void onExit();
    void onNewWatchRound();
    void onOpenWatchSpec();
    void onSaveWatchSpec();
    void onNewPhoneRect();
    void onRenderFullDemoWatch();

    // ── Layer 3/4/5 GUI ──────────────────────────────────────────────────
    void onShowProcessPlanEditor();
    void onRecognizeCurrentShape();
    void onExecutePlanFromEditor();
    void onNlToEditOpStub();

private:
    MainWindow* m_window;
};

}  // namespace koocadcam::gui
