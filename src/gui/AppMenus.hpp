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
    void onOpenStep();
    void onFitAll();
    void onExit();
    void onNewWatchRound();
    void onOpenWatchSpec();
    void onSaveWatchSpec();
    void onNewPhoneRect();
    void onRenderFullDemoWatch();

private:
    MainWindow* m_window;
};

}  // namespace koocadcam::gui
