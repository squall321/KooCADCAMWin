#pragma once
// @lat: [[architecture/overview#src/gui/]]

#include <QMainWindow>

namespace koocadcam::gui {

class OcctViewWidget;
class AppMenus;
class ParameterPanel;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    OcctViewWidget* viewWidget() const { return m_view; }
    [[nodiscard]] ParameterPanel* parameterPanel() const { return m_paramPanel; }

private slots:
    void onSpecChanged();

private:
    OcctViewWidget* m_view{nullptr};
    AppMenus*       m_menus{nullptr};
    ParameterPanel* m_paramPanel{nullptr};
};

}  // namespace koocadcam::gui
