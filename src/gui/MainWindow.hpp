#pragma once
// @lat: [[architecture/overview#src/gui/]]

#include <QMainWindow>

namespace koocadcam::gui {

class OcctViewWidget;
class AppMenus;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    OcctViewWidget* viewWidget() const { return m_view; }

private:
    OcctViewWidget* m_view{nullptr};
    AppMenus*       m_menus{nullptr};
};

}  // namespace koocadcam::gui
