#include "mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QFile>
#include <QStringList>
#include <cmath>
#include <algorithm>
#include <limits>
#include <QDebug>
#include <QMenuBar>

// ===== 固定起点（来自 gnss_gridmap.launch 的 start_pose）=====
static const QPointF kStartPoseXY(0.0, 0.0);
// yaw = 150° -> 转成弧度
static const double kStartYawRad = 150.0 * M_PI / 180.0;

QPointF latlonToXY(double lon, double lat, double lon0, double lat0)
{
    // 简单局部投影（米）
    const double R = 6378137.0; // 地球半径
    double dLat = (lat - lat0) * M_PI / 180.0;
    double dLon = (lon - lon0) * M_PI / 180.0;
    double x = dLon * R * std::cos(lat0 * M_PI / 180.0);
    double y = dLat * R;
    return QPointF(x, y);
}


// ================== AgriPlanner (基于 version2) ==================
namespace AgriPlanner {
    double distance(const QPointF& a, const QPointF& b) { return std::hypot(a.x() - b.x(), a.y() - b.y()); }
    double cross(const QPointF& a, const QPointF& b) { return a.x() * b.y() - a.y() * b.x(); }
    QPointF normalize(const QPointF& v) {
        double len = std::hypot(v.x(), v.y());
        return len == 0 ? QPointF(0, 0) : QPointF(v.x() / len, v.y() / len);
    }

    // 点到线段最近点
    QPointF closestPointOnSegment(const QPointF& p, const QPointF& a, const QPointF& b)
    {
        QPointF ab = b - a;
        double t = ((p.x()-a.x())*ab.x() + (p.y()-a.y())*ab.y()) /
                   (ab.x()*ab.x() + ab.y()*ab.y());
        t = std::max(0.0, std::min(1.0, t));
        return QPointF(a.x() + t*ab.x(), a.y() + t*ab.y());
    }

    // 点到多边形最近点 + 所在边索引
    QPointF closestPointOnPolygon(const QVector<QPointF>& poly, const QPointF& p, int& edgeIdx)
    {
        double bestD = std::numeric_limits<double>::max();
        QPointF best;
        edgeIdx = 0;
        for (int i=0;i<poly.size();++i){
            QPointF a = poly[i];
            QPointF b = poly[(i+1)%poly.size()];
            QPointF c = closestPointOnSegment(p, a, b);
            double d = distance(p, c);
            if (d < bestD){
                bestD = d;
                best = c;
                edgeIdx = i;
            }
        }
        return best;
    }

    // ===== 去重拼接 =====
    void appendNoRepeat(QVector<QPointF>& dst, const QVector<QPointF>& src, double eps = 1e-6) {
        for (const auto& p : src) {
            if (dst.isEmpty()) { dst.append(p); continue; }
            if (distance(dst.last(), p) > eps) dst.append(p);
        }
    }

    // ===== 多边形面积 =====
    double polygonArea(const QVector<QPointF>& polygon) {
        if (polygon.size() < 3) return 0.0;
        double area = 0.0;
        for (int i = 0; i < polygon.size(); i++) {
            int j = (i + 1) % polygon.size();
            area += polygon[i].x() * polygon[j].y() - polygon[j].x() * polygon[i].y();
        }
        return std::abs(area) / 2.0;
    }

    // ===== 线交点 =====
    QPointF lineIntersection(const QPointF& p1, const QPointF& p2,
                             const QPointF& q1, const QPointF& q2, bool& intersects) {
        QPointF r = p2 - p1;
        QPointF s = q2 - q1;
        double crossRS = cross(r, s);
        if (std::abs(crossRS) < 1e-10) { intersects = false; return QPointF(0, 0); }
        QPointF qp = q1 - p1;
        double t = cross(qp, s) / crossRS;
        intersects = true;
        return p1 + r * t;
    }

    // ===== 多边形内缩 (version2) =====
    QVector<QPointF> offsetPolygon(const QVector<QPointF>& polygon, double distance_val) {
        int n = polygon.size();
        if (n < 3) return polygon;

        double signedArea = 0.0;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            signedArea += polygon[i].x() * polygon[j].y() - polygon[j].x() * polygon[i].y();
        }
        signedArea *= 0.5;
        double actualDistance = (signedArea < 0) ? -distance_val : distance_val;

        QVector<QPointF> result;
        result.reserve(n);

        for (int i = 0; i < n; i++) {
            QPointF prev = polygon[(i - 1 + n) % n];
            QPointF curr = polygon[i];
            QPointF next = polygon[(i + 1) % n];

            QPointF v1 = curr - prev;
            QPointF v2 = next - curr;

            if (std::hypot(v1.x(), v1.y()) < 1e-4) v1 = QPointF(1e-4, 0);
            if (std::hypot(v2.x(), v2.y()) < 1e-4) v2 = QPointF(1e-4, 0);

            v1 = normalize(v1);
            v2 = normalize(v2);

            QPointF normal1(-v1.y(), v1.x());
            QPointF normal2(-v2.y(), v2.x());

            QPointF offset1 = curr + normal1 * actualDistance;
            QPointF offset2 = curr + normal2 * actualDistance;

            bool intersects;
            QPointF intersection = lineIntersection(offset1, offset1 + v1, offset2, offset2 + v2, intersects);

            if (intersects) {
                if (distance(intersection, curr) > std::abs(distance_val) * 2.5) {
                    result.push_back((offset1 + offset2) / 2.0);
                } else {
                    result.push_back(intersection);
                }
            } else {
                result.push_back((offset1 + offset2) / 2.0);
            }
        }
        return result;
    }

    // ===== 生成等高线（version2）=====
    QVector<QVector<QPointF>> generateContours(const QVector<QPointF>& polygon, double w, double min_layer_area_ratio) {
        QVector<QVector<QPointF>> contours;
        QVector<QPointF> currentPolygon = polygon;
        contours.push_back(currentPolygon);

        int maxLayers = 200;
        for (int layer = 0; layer < maxLayers; layer++) {
            QVector<QPointF> nextPolygon = offsetPolygon(currentPolygon, w);
            double currentArea = polygonArea(currentPolygon);
            double nextArea = polygonArea(nextPolygon);

            if (nextPolygon.size() < 3 ||
                nextArea < w * w ||
                nextArea / currentArea < min_layer_area_ratio ||
                std::abs(nextArea - currentArea) < 1e-4)
                break;

            contours.push_back(nextPolygon);
            currentPolygon = nextPolygon;
        }
        return contours;
    }

    // ===== 寻找相邻层切入点 =====
    std::pair<int,double> findNearestPoint(const QVector<QPointF>& contour, const QPointF& fromPoint) {
        int nearestIndex = 0;
        double minDistance = std::numeric_limits<double>::max();
        for (int i = 0; i < contour.size(); i++) {
            double dx = fromPoint.x() - contour[i].x();
            double dy = fromPoint.y() - contour[i].y();
            double dist = dx*dx + dy*dy;
            if (dist < minDistance) { minDistance = dist; nearestIndex = i; }
        }
        return {nearestIndex, std::sqrt(minDistance)};
    }

    bool getExtensionIntersection(const QPointF& rayOrigin, const QPointF& rayDir,
                                  const QVector<QPointF>& polygon, QPointF& intersection, int& edgeIndex) {
        int n = polygon.size();
        double minDist = std::numeric_limits<double>::max();
        bool found = false;

        for (int i = 0; i < n; i++) {
            QPointF p1 = polygon[i];
            QPointF p2 = polygon[(i + 1) % n];

            QPointF v2 = p2 - p1;
            double cross_D_edge = cross(rayDir, v2);
            if (std::abs(cross_D_edge) < 1e-10) continue;

            double t = cross(p1 - rayOrigin, v2) / cross_D_edge;
            double u = cross(p1 - rayOrigin, rayDir) / cross_D_edge;

            if (t > 1e-5 && u >= -1e-5 && u <= 1.0 + 1e-5) {
                if (t < minDist) {
                    minDist = t;
                    intersection = rayOrigin + rayDir * t;
                    edgeIndex = i;
                    found = true;
                }
            }
        }
        return found;
    }

    QVector<QPointF> generateContourPath(const QVector<QPointF>& contour, int startIndex,
                                         bool stopAtCutPoint, const QPointF& cutPoint, int cutEdgeIndex) {
        int n = contour.size();
        if (n < 3) return contour;
        QVector<QPointF> path;

        for (int i = 0; i < n; i++) {
            int currIdx = (startIndex + i) % n;
            path.push_back(contour[currIdx]);

            if (stopAtCutPoint && currIdx == cutEdgeIndex) {
                if (distance(cutPoint, contour[currIdx]) > 1e-5)
                    path.push_back(cutPoint);
                break;
            }
        }
        return path;
    }

    // ===== 生成连接路径（线性插值）=====
    QVector<QPointF> generateConnectionPath(const QPointF& fromPoint,
                                            const QPointF& toPoint,
                                            double stepSize)
    {
        QVector<QPointF> path;
        double dx = toPoint.x() - fromPoint.x();
        double dy = toPoint.y() - fromPoint.y();
        double dist = std::hypot(dx, dy);

        if (dist < stepSize) {
            path.push_back(toPoint);
        } else {
            int numSteps = static_cast<int>(std::ceil(dist / stepSize));
            for (int i = 1; i <= numSteps; i++) {
                double t = static_cast<double>(i) / numSteps;
                path.push_back(QPointF(fromPoint.x() + t * dx,
                                       fromPoint.y() + t * dy));
            }
        }
        return path;
    }

    // ===== 旋转外接矩形（version2）=====
    struct BoundingRect {
        QVector<QPointF> corners;
        double angle;
        double width;
        double height;
    };

    BoundingRect computeMinBoundingRect(const QVector<QPointF>& polygon) {
        BoundingRect result;
        if (polygon.size() < 2) return result;

        double angle = std::atan2(polygon[1].y() - polygon[0].y(),
                                  polygon[1].x() - polygon[0].x());
        result.angle = angle;

        double cosA = std::cos(-angle), sinA = std::sin(-angle);
        auto rotPt = [&](const QPointF& p) {
            return QPointF(p.x() * cosA - p.y() * sinA,
                           p.x() * sinA + p.y() * cosA);
        };

        double minX = std::numeric_limits<double>::max(), maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max(), maxY = std::numeric_limits<double>::lowest();
        for (const auto& p : polygon) {
            QPointF r = rotPt(p);
            minX = std::min(minX, r.x()); maxX = std::max(maxX, r.x());
            minY = std::min(minY, r.y()); maxY = std::max(maxY, r.y());
        }

        result.width = maxX - minX;
        result.height = maxY - minY;

        QVector<QPointF> cornersRot = {
            QPointF(minX, minY),
            QPointF(maxX, minY),
            QPointF(maxX, maxY),
            QPointF(minX, maxY)
        };

        double cosInv = std::cos(angle), sinInv = std::sin(angle);
        for (const auto& c : cornersRot) {
            result.corners.push_back(QPointF(
                c.x() * cosInv - c.y() * sinInv,
                c.x() * sinInv + c.y() * cosInv
            ));
        }
        return result;
    }

    // ===== S 弯路径 =====
    // ===== 将扫描线裁剪到多边形内部 =====
    std::vector<std::pair<double,double>> clipScanlineToPolygon(const QVector<QPointF>& rotatedPoly, double y) {
        std::vector<double> xs;
        int n = rotatedPoly.size();
        for (int i = 0; i < n; i++) {
            QPointF a = rotatedPoly[i];
            QPointF b = rotatedPoly[(i + 1) % n];
            if ((a.y() <= y && b.y() > y) || (b.y() <= y && a.y() > y)) {
                if (std::abs(b.y() - a.y()) > 1e-9) {
                    double x = a.x() + (y - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
                    xs.push_back(x);
                }
            } else if (std::abs(a.y() - y) < 1e-6) {
                xs.push_back(a.x());
            }
        }

        std::sort(xs.begin(), xs.end());
        xs.erase(std::unique(xs.begin(), xs.end(), [](double a, double b) {
            return std::abs(a - b) < 1e-5;
        }), xs.end());

        std::vector<std::pair<double,double>> segments;
        for (size_t k = 0; k + 1 < xs.size(); k += 2)
            segments.emplace_back(xs[k], xs[k+1]);

        return segments;
    }


    QVector<QPointF> generateSBendPath(const BoundingRect& rect,
                                       const QVector<QPointF>& polygon,
                                       double w,
                                       const QPointF& startPos)
    {
        if (rect.corners.size() < 4 || w <= 0 || polygon.size() < 3) return {};

        double angle  = rect.angle;
        double cosA   = std::cos(-angle), sinA = std::sin(-angle);
        double cosInv = std::cos(angle),  sinInv = std::sin(angle);

        auto toRot = [&](const QPointF& p) {
            return QPointF(p.x() * cosA - p.y() * sinA,
                           p.x() * sinA + p.y() * cosA);
        };
        auto toWorld = [&](const QPointF& p) {
            return QPointF(p.x() * cosInv - p.y() * sinInv,
                           p.x() * sinInv + p.y() * cosInv);
        };

        QVector<QPointF> rotatedPoly;
        rotatedPoly.reserve(polygon.size());
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();
        for (const auto& p : polygon) {
            QPointF r = toRot(p);
            rotatedPoly.push_back(r);
            minY = std::min(minY, r.y());
            maxY = std::max(maxY, r.y());
        }

        QPointF startRot = toRot(startPos);
        double y_start = (std::abs(startRot.y() - minY) <= std::abs(startRot.y() - maxY)) ? minY : maxY;
        double y_end   = (y_start == minY) ? maxY : minY;
        double y_step  = (y_end > y_start) ? w : -w;

        QVector<QVector<QPointF>> rows;
        int max_rows = static_cast<int>(std::ceil(std::abs(y_end - y_start) / w)) + 2;

        for (int i = 0; i < max_rows; i++) {
            double y = y_start + i * y_step;
            if (y_step > 0 && y > y_end + 1e-3) break;
            if (y_step < 0 && y < y_end - 1e-3) break;

            auto segments = clipScanlineToPolygon(rotatedPoly, y);
            for (const auto& seg : segments) {
                rows.push_back({ QPointF(seg.first, y), QPointF(seg.second, y) });
            }
        }

        if (rows.empty()) return {};
        QVector<QPointF> path;
        QPointF curPos = startRot;

        for (auto& row : rows) {
            double distL = distance(curPos, row[0]);
            double distR = distance(curPos, row[1]);
            if (distL > distR) std::reverse(row.begin(), row.end());

            for (const auto& p : row) path.push_back(toWorld(p));
            curPos = row.back();
        }
        return path;
    }

    // ===== 优化去重 =====
    QVector<QPointF> optimizePath(const QVector<QPointF>& path, double minDistance) {
        if (path.size() < 2) return path;
        QVector<QPointF> optimized;
        optimized.push_back(path[0]);
        for (int i = 1; i < path.size(); i++) {
            if (distance(optimized.back(), path[i]) >= minDistance) {
                optimized.push_back(path[i]);
            }
        }
        return optimized;
    }

    // ===== 主算法：version2 contour spiral + S-bend =====
    QVector<QPointF> generateContourSpiralPath(const QVector<QPointF>& polygon, double w,
                                               double min_optimization_distance = 0.3,
                                               double connection_step_factor = 0.5,
                                               double min_layer_area_ratio = 0.1)
    {
        // 轻微内缩，避免第一圈贴边
        QVector<QPointF> basePolygon = offsetPolygon(polygon, w * 0.2);
        if (basePolygon.size() < 3) basePolygon = polygon;

        QVector<QVector<QPointF>> contours = generateContours(basePolygon, w, min_layer_area_ratio);
        if (contours.isEmpty()) return {};

        // 使用 connection_step_factor 防止 unused
        double connection_step = w * connection_step_factor;
        if (connection_step < w * 0.2) connection_step = w * 0.2;

        QVector<QPointF> fullPath;
        int currentIdealStartIdx = 0;
        int currentActualStartIdx = currentIdealStartIdx;

        for (int layer = 0; layer < contours.size(); layer++) {
            const auto& currContour = contours[layer];
            bool isLastLayer = (layer == contours.size() - 1);

            if (isLastLayer) {
                double lastArea = polygonArea(currContour);
                double originalArea = polygonArea(contours[0]);
                const double narrow_area_threshold = 0.5;

                bool isNarrow = (originalArea > 1e-6 &&
                                 (lastArea / originalArea < narrow_area_threshold ||
                                  currContour.size() < 4));

                if (isNarrow) {
                    BoundingRect rect = computeMinBoundingRect(currContour);
                    QPointF sBendStart = fullPath.isEmpty() ? currContour[currentActualStartIdx] : fullPath.back();

                    // 注意：新版调用有 polygon 参数
                    QVector<QPointF> sBendPath = generateSBendPath(rect, currContour, w, sBendStart);
                    appendNoRepeat(fullPath, sBendPath, w * 0.3);
                } else {
                    QVector<QPointF> layerPath =
                        generateContourPath(currContour, currentActualStartIdx, false, QPointF(), -1);
                    appendNoRepeat(fullPath, layerPath, w * 0.3);
                }
                break;
            }

            const auto& nextContour = contours[layer + 1];
            auto nearestNext = findNearestPoint(nextContour, currContour[currentIdealStartIdx]);
            int nextIdealStartIdx = nearestNext.first;
            int nextSecondIdx = (nextIdealStartIdx + 1) % nextContour.size();

            QPointF pNextStart = nextContour[nextIdealStartIdx];
            QPointF pNextSecond = nextContour[nextSecondIdx];

            QPointF cutPoint;
            int cutEdgeIndex = -1;

            // 用“当前路径末端”或者当前层起点，去找最短切入点
            QPointF refPoint = fullPath.isEmpty() ? currContour[currentActualStartIdx] : fullPath.back();
            cutPoint = closestPointOnPolygon(currContour, refPoint, cutEdgeIndex);
            bool foundCut = true;
            if (!foundCut) {
                QVector<QPointF> layerPath = generateContourPath(currContour, currentActualStartIdx, false, QPointF(), -1);
                appendNoRepeat(fullPath, layerPath, w * 0.3);

                // 改成“连接到下一层”而不是直接 break
                QVector<QPointF> connect = generateConnectionPath(layerPath.back(), pNextSecond, w * connection_step_factor);
                appendNoRepeat(fullPath, connect, w * 0.3);

                // 继续下一层
                currentIdealStartIdx = nextIdealStartIdx;
                currentActualStartIdx = nextSecondIdx;
                continue;
            }

            QVector<QPointF> layerPath =
                generateContourPath(currContour, currentActualStartIdx, true, cutPoint, cutEdgeIndex);
            appendNoRepeat(fullPath, layerPath, w * 0.3);

            currentIdealStartIdx = nextIdealStartIdx;
            currentActualStartIdx = nextSecondIdx;
        }

        return optimizePath(fullPath, w * min_optimization_distance);
    }
}

// ================== MainWindow 主窗口逻辑的实现 ==================
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

void MainWindow::onSelectBoundaryClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "选择边界坐标文件", "", "Text Files (*.txt);;All Files (*)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开文件！");
        return;
    }

    QTextStream in(&file);
    QString content = in.readAll();
    file.close();

    m_boundaryPoints.clear();
    QStringList lines = content.split('\n', QString::SkipEmptyParts);
    for (const QString& line : lines) {
        QStringList parts = line.split(QRegExp("[,\\s]+"), QString::SkipEmptyParts);
        if (parts.size() >= 2) {
            bool okX, okY;
            double x = parts[0].toDouble(&okX);
            double y = parts[1].toDouble(&okY);
            if (okX && okY) {
                m_boundaryPoints.append(QPointF(x, y));
            }
        }
    }

    QStringList boundaryLines;
    for (const auto& p : m_boundaryPoints) {
        boundaryLines << QString::number(p.x(), 'f', 7) + ", " + QString::number(p.y(), 'f', 7);
    }
    txtBoundary->setPlainText(boundaryLines.join("\n"));
    txtTrajectory->clear();
    m_trajectoryPoints.clear();
}

bool MainWindow::isConvexPolygon(const QVector<QPointF>& points)
{
    int n = points.size();
    if (n < 3) return false;

    bool isPositive = false;
    bool isNegative = false;

    for (int i = 0; i < n; i++) {
        QPointF p0 = points[i];
        QPointF p1 = points[(i + 1) % n];
        QPointF p2 = points[(i + 2) % n];

        double dx1 = p1.x() - p0.x();
        double dy1 = p1.y() - p0.y();
        double dx2 = p2.x() - p1.x();
        double dy2 = p2.y() - p1.y();

        double crossProduct = dx1 * dy2 - dy1 * dx2;

        if (crossProduct > 0) isPositive = true;
        if (crossProduct < 0) isNegative = true;

        if (isPositive && isNegative) return false;
    }
    return true;
}

QVector<QPointF> MainWindow::generateTrajectory(const QVector<QPointF>& metricPolygon)
{
    if (metricPolygon.size() < 3) return {};

    double minX=metricPolygon[0].x(), maxX=minX;
    double minY=metricPolygon[0].y(), maxY=minY;
    for (auto &p: metricPolygon){
        minX = std::min(minX, p.x()); maxX = std::max(maxX, p.x());
        minY = std::min(minY, p.y()); maxY = std::max(maxY, p.y());
    }
    double max_dim = std::max(maxX - minX, maxY - minY);

    // 关键：w 自适应（小地块更小）
    auto clampd = [](double v, double lo, double hi){
        return std::max(lo, std::min(v, hi));
    };
    double w = clampd(max_dim / 20.0, 0.5, 2.0);

    const double min_optimization_factor = 0.3;
    const double connection_step_factor = 0.5;
    const double min_layer_area_ratio   = 0.3;

    qDebug() << "bounds(m)=" << (maxX-minX) << (maxY-minY);
    qDebug() << "w =" << w;

    auto contours = AgriPlanner::generateContours(metricPolygon, w, min_layer_area_ratio);
    qDebug() << "contour layers =" << contours.size();

    return AgriPlanner::generateContourSpiralPath(
        metricPolygon, w, min_optimization_factor, connection_step_factor, min_layer_area_ratio);
}

void MainWindow::onGenerateClicked()
{
    txtTrajectory->clear();
    m_trajectoryPoints.clear();

    if (m_boundaryPoints.size() < 3) {
        QMessageBox::warning(this, "失败", "边界点数量少于3，无法生成！");
        return;
    }

    if (!isConvexPolygon(m_boundaryPoints)) {
        QMessageBox::warning(this, "失败", "输入的边界构成了凹多边形，无法生成！");
        return;
    }

    // ===== 1) 经纬度 -> 米坐标 =====
    double lon0 = m_boundaryPoints[0].x();
    double lat0 = m_boundaryPoints[0].y();

    QVector<QPointF> metricBoundary;
    metricBoundary.reserve(m_boundaryPoints.size());
    for (const auto& p : m_boundaryPoints) {
        metricBoundary.push_back(latlonToXY(p.x(), p.y(), lon0, lat0));
    }

    // ===== 2) 用米坐标生成轨迹 =====
    m_trajectoryPoints = generateTrajectory(metricBoundary);

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
