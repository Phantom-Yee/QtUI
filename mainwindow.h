#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QPointF>
#include <QVector>
#include <QWidget>
#include <QTextEdit>
#include <QMenu>
#include <QAction>

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
    void onHelpTriggered();
    void onAboutTriggered();

private:
    QMenu *menuFile;
    QMenu *menuHelp;
    QAction *actionOpenBoundary;
    QAction *actionGenerate;
    QAction *actionSave;
    QAction *actionExit;
    QAction *actionHelp;
    QAction *actionAbout;

    QTextEdit *txtBoundary;
    QTextEdit *txtTrajectory;

    QLabel *lblBoundary;
    QLabel *lblTrajectory;

    QVector<QPointF> m_boundaryPoints;
    QVector<QPointF> m_trajectoryPoints;

    void setupUI();
    bool isConvexPolygon(const QVector<QPointF>& points);
    QVector<QPointF> generateTrajectory(const QVector<QPointF>& boundary);
};

#endif // MAINWINDOW_H
