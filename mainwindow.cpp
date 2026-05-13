#include "mainwindow.h"
#include "pathplanning.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QFile>
#include <QStringList>
#include <QMenuBar>
#include <QDir>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setupUI();
}

MainWindow::~MainWindow() {}

void MainWindow::setupUI()
{
    this->setWindowTitle("轨迹生成系统");
    this->resize(900, 550);

    QWidget *centralWidget = new QWidget(this);
    this->setCentralWidget(centralWidget);

    menuFile = menuBar()->addMenu("文件(&Y)");
    menuHelp = menuBar()->addMenu("帮助(&Z)");

    actionOpenBoundary = new QAction("打开边界点(&X)", this);
    actionGenerate     = new QAction("生成轨迹(&Y)", this);
    actionSave         = new QAction("保存(&S)", this);
    actionExit         = new QAction("关闭(&Z)", this);

    actionHelp  = new QAction("帮助", this);
    actionAbout = new QAction("关于", this);

    menuFile->addAction(actionOpenBoundary);
    menuFile->addAction(actionGenerate);
    menuFile->addAction(actionSave);
    menuFile->addSeparator();
    menuFile->addAction(actionExit);

    menuHelp->addAction(actionHelp);
    menuHelp->addAction(actionAbout);

    connect(actionOpenBoundary, &QAction::triggered, this, &MainWindow::onSelectBoundaryClicked);
    connect(actionGenerate,     &QAction::triggered, this, &MainWindow::onGenerateClicked);
    connect(actionSave,         &QAction::triggered, this, &MainWindow::onSaveClicked);
    connect(actionExit,         &QAction::triggered, this, &QWidget::close);
    connect(actionHelp,         &QAction::triggered, this, &MainWindow::onHelpTriggered);
    connect(actionAbout,        &QAction::triggered, this, &MainWindow::onAboutTriggered);

    lblBoundary = new QLabel("边界点", this);
    lblBoundary->setStyleSheet("color: red; border: 1px solid #333; padding: 2px;");
    lblBoundary->setAlignment(Qt::AlignCenter);

    lblTrajectory = new QLabel("轨迹点", this);
    lblTrajectory->setStyleSheet("color: red; border: 1px solid #333; padding: 2px;");
    lblTrajectory->setAlignment(Qt::AlignCenter);

    txtBoundary = new QTextEdit(this);
    txtTrajectory = new QTextEdit(this);
    txtBoundary->setReadOnly(true);
    txtTrajectory->setReadOnly(true);

    QGridLayout *mainLayout = new QGridLayout(centralWidget);
    mainLayout->addWidget(lblBoundary, 0, 0);
    mainLayout->addWidget(lblTrajectory, 0, 1);
    mainLayout->addWidget(txtBoundary, 1, 0);
    mainLayout->addWidget(txtTrajectory, 1, 1);

    mainLayout->setColumnStretch(0, 1);
    mainLayout->setColumnStretch(1, 1);
}

bool MainWindow::loadPointsFromFile(const QString& filePath, QVector<QPointF>& points)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }

    QTextStream in(&file);
    QString content = in.readAll();
    file.close();

    points.clear();
    QStringList lines = content.split('\n', QString::SkipEmptyParts);
    for (const QString& line : lines) {
        QStringList parts = line.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
        if (parts.size() >= 2) {
            bool okX, okY;
            double x = parts[0].toDouble(&okX);
            double y = parts[1].toDouble(&okY);
            if (okX && okY) {
                points.append(QPointF(x, y));
            }
        }
    }
    return points.size() >= 3;
}

void MainWindow::onSelectBoundaryClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "选择边界坐标文件", "", "Text Files (*.txt);;All Files (*)");
    if (fileName.isEmpty()) return;

    m_boundaryFilePath = fileName;

    if (!loadPointsFromFile(fileName, m_boundaryPoints)) {
        QMessageBox::warning(this, "错误", "无法读取边界文件！");
        return;
    }

    QStringList boundaryLines;
    for (const auto& p : m_boundaryPoints) {
        boundaryLines << QString::number(p.x(), 'f', 7) + ", " + QString::number(p.y(), 'f', 7);
    }
    txtBoundary->setPlainText(boundaryLines.join("\n"));
    txtTrajectory->clear();
    m_trajectoryPoints.clear();
}

void MainWindow::onGenerateClicked()
{
    txtTrajectory->clear();
    m_trajectoryPoints.clear();

    if (m_boundaryFilePath.isEmpty()) {
        QMessageBox::warning(this, "失败", "请先选择边界文件！");
        return;
    }

    QString outputPath = QDir::tempPath() + "/trajectory_output.txt";
    m_outputFilePath = outputPath;

    double robotWidth = 2.0;
    double optimizeDist = 0.6;
    double stepSize = 1.0;

    int ret = generate_path_from_file(
        m_boundaryFilePath.toUtf8().constData(),
        outputPath.toUtf8().constData(),
        robotWidth,
        optimizeDist,
        stepSize
    );

    if (ret != 0) {
        QMessageBox::warning(this, "失败", "路径规划失败，返回码: " + QString::number(ret));
        return;
    }

    if (!loadPointsFromFile(outputPath, m_trajectoryPoints)) {
        QMessageBox::warning(this, "失败", "无法读取轨迹输出文件！");
        return;
    }

    QStringList trajLines;
    for (const auto& p : m_trajectoryPoints) {
        trajLines << QString::number(p.x(), 'f', 7) + ", " + QString::number(p.y(), 'f', 7);
    }
    txtTrajectory->setPlainText(trajLines.join("\n"));
}

void MainWindow::onSaveClicked()
{
    if (m_trajectoryPoints.isEmpty()) {
        QMessageBox::information(this, "提示", "没有生成轨迹点可以保存！");
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this, "保存轨迹坐标", "trajectory.txt", "Text Files (*.txt)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法保存文件！");
        return;
    }

    QTextStream out(&file);
    for (const QPointF& pt : m_trajectoryPoints) {
        out << QString::number(pt.x(), 'f', 7) << ", " << QString::number(pt.y(), 'f', 7) << "\n";
    }

    file.close();
    QMessageBox::information(this, "成功", "轨迹已成功保存为txt文件！");
}

void MainWindow::onHelpTriggered()
{
    // 预留接口，不做任何处理
}

void MainWindow::onAboutTriggered()
{
    QMessageBox::information(this, "关于", "覆盖式路径规划与生成桌面版，Version1.0");
}
