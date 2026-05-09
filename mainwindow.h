#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QPointF>
#include <QVector>
#include <QWidget>

// ================= 用于图形可视化的自定义控件类 =================
class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = nullptr);
    void setPoints(const QVector<QPointF>& points);             // 设置边界点
    void setTrajectory(const QVector<QPointF>& trajectory);     // 设置轨迹点并重绘

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QVector<QPointF> m_points;          // 边界
    QVector<QPointF> m_trajectory;      // 轨迹
};
// =====================================================================

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onSelectBoundaryClicked();
    void onGenerateClicked();
    void onSaveClicked();

private:
    QPushButton *btnSelectBoundary;
    QPushButton *btnGenerate;
    QPushButton *btnSave;

    MapWidget *mapWidgetBoundary;      // 左侧绘图控件
    MapWidget *mapWidgetTrajectory;    // 右侧绘图控件 (原先的QTextEdit已替换)

    QLabel *lblBoundary;
    QLabel *lblTrajectory;

    QVector<QPointF> m_boundaryPoints;
    QVector<QPointF> m_trajectoryPoints; // 存储生成的轨迹点，用于保存

    void setupUI();
    bool isConvexPolygon(const QVector<QPointF>& points);
    QVector<QPointF> generateTrajectory(const QVector<QPointF>& boundary);
};

#endif // MAINWINDOW_H
