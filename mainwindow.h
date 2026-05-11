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
#include <QString>

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

    QString m_boundaryFilePath;
    QString m_outputFilePath;

    void setupUI();
    bool loadPointsFromFile(const QString& filePath, QVector<QPointF>& points);
};

#endif // MAINWINDOW_H
