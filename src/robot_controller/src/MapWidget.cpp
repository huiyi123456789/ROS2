#include "MapWidget.h"
#include <QPainter>
#include <cmath>

MapWidget::MapWidget(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent), node_(node)
{
    setMinimumSize(300, 300);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_, node_);

    mapSub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10),
        std::bind(&MapWidget::updateMap, this, std::placeholders::_1));

    setScanTopic("/scan");
}

void MapWidget::setScanTopic(const QString &topic)
{
    scanSub_.reset();
    scanTopic_ = topic;
    scanPointsLaser_.clear();
    scanPointsMap_.clear();

    if (topic.isEmpty())
        return;

    scanSub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        topic.toStdString(), rclcpp::QoS(10),
        std::bind(&MapWidget::updateScan, this, std::placeholders::_1));
}

void MapWidget::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    int width = msg->info.width;
    int height = msg->info.height;

    if (width <= 0 || height <= 0)
        return;

    lastMap_ = *msg;
    mapFrameId_ = QString::fromStdString(msg->header.frame_id);

    mapImage_ = QImage(width, height, QImage::Format_ARGB32);
    mapImage_.fill(Qt::gray);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            int8_t val = msg->data[idx];
            QColor color;
            if (val == -1)
                color = QColor(180, 180, 180);
            else if (val == 100)
                color = Qt::black;
            else
                color = QColor(255 - val * 2, 255 - val * 2, 255 - val * 2);
            mapImage_.setPixelColor(x, height - y - 1, color);
        }
    }

    update();
}

void MapWidget::updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scanMaxRange_ = static_cast<double>(msg->range_max);

    scanPointsLaser_.clear();
    scanPointsLaser_.reserve(msg->ranges.size());

    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
        double r = msg->ranges[i];
        if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
            scanPointsLaser_.append(QPointF(r * std::cos(angle), r * std::sin(angle)));
        }
    }

    scanPointsMap_.clear();

    if (!mapFrameId_.isEmpty() && lastMap_.info.width > 0) {
        rclcpp::Time stamp = msg->header.stamp;
        QString scanFrameId = QString::fromStdString(msg->header.frame_id);

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tfBuffer_->lookupTransform(
                mapFrameId_.toStdString(),
                scanFrameId.toStdString(),
                stamp,
                rclcpp::Duration::from_seconds(0.1));
        } catch (tf2::TransformException &ex) {
            update();
            return;
        }

        double tx = tf.transform.translation.x;
        double ty = tf.transform.translation.y;

        double qx = tf.transform.rotation.x;
        double qy = tf.transform.rotation.y;
        double qz = tf.transform.rotation.z;
        double qw = tf.transform.rotation.w;
        double siny = 2.0 * (qw * qz + qx * qy);
        double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny, cosy);
        double cosYaw = std::cos(yaw);
        double sinYaw = std::sin(yaw);

        scanPointsMap_.reserve(scanPointsLaser_.size());
        for (const auto &pt : scanPointsLaser_) {
            double mx = cosYaw * pt.x() - sinYaw * pt.y() + tx;
            double my = sinYaw * pt.x() + cosYaw * pt.y() + ty;
            scanPointsMap_.append(QPointF(mx, my));
        }
    }

    update();
}

void MapWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);

    if (!mapImage_.isNull()) {
        renderWithMap(painter);
    } else {
        renderStandalone(painter);
    }
}

void MapWidget::renderStandalone(QPainter &painter)
{
    painter.fillRect(rect(), QColor(20, 20, 20));

    double range = std::max(scanMaxRange_, 3.5);
    double margin = 10.0;
    double cx = width() / 2.0;
    double cy = height() / 2.0;
    double scale = (std::min(width(), height()) - 2.0 * margin) / (range * 2.0);

    auto toPixel = [&](double wx, double wy) -> QPointF {
        return QPointF(cx - wy * scale, cy - wx * scale);
    };

    painter.setPen(QPen(QColor(60, 60, 60), 1, Qt::DotLine));
    for (double r = 1.0; r <= range + 0.5; r += 1.0) {
        QPointF center = toPixel(0, 0);
        double pr = r * scale;
        painter.drawEllipse(center, pr, pr);
    }

    painter.setPen(QPen(QColor(40, 40, 40), 1));
    double axisLen = range * scale;
    painter.drawLine(QPointF(cx - axisLen, cy), QPointF(cx + axisLen, cy));
    painter.drawLine(QPointF(cx, cy - axisLen), QPointF(cx, cy + axisLen));

    painter.setPen(QPen(QColor(80, 80, 80), 1));
    int steps = std::max(3, static_cast<int>(range));
    for (int i = 1; i <= steps; ++i) {
        QPointF pt = toPixel(static_cast<double>(i), 0.0);
        painter.drawText(QRectF(pt.x() - 20, pt.y() - 6, 40, 20), Qt::AlignCenter,
                         QString("%1m").arg(i));
    }

    if (!scanPointsLaser_.isEmpty()) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 255, 0, 180));
        for (const auto &pt : scanPointsLaser_) {
            QPointF pixel = toPixel(pt.x(), pt.y());
            painter.drawEllipse(pixel, 2, 2);
        }

        QPointF origin = toPixel(0, 0);
        painter.setBrush(QColor(255, 80, 80));
        painter.drawEllipse(origin, 5, 5);

        double dirLen = 0.4;
        QPointF dirPt = toPixel(dirLen, 0.0);
        painter.setPen(QPen(QColor(255, 80, 80), 3));
        painter.drawLine(origin, dirPt);
    }

    painter.setPen(Qt::white);
    QString info = QString("Scan: %1 pts | Range: %2m | Topic: %3")
                       .arg(scanPointsLaser_.size())
                       .arg(scanMaxRange_, 0, 'f', 1)
                       .arg(scanTopic_);
    painter.drawText(10, 20, info);
}

void MapWidget::renderWithMap(QPainter &painter)
{
    painter.fillRect(rect(), QColor(50, 50, 50));

    QImage scaled = mapImage_.scaled(size(), Qt::KeepAspectRatio, Qt::FastTransformation);
    int imgX = (width() - scaled.width()) / 2;
    int imgY = (height() - scaled.height()) / 2;
    painter.drawImage(imgX, imgY, scaled);

    double originX = lastMap_.info.origin.position.x;
    double originY = lastMap_.info.origin.position.y;
    double resolution = lastMap_.info.resolution;
    int gridH = lastMap_.info.height;
    double scale = static_cast<double>(scaled.width()) / lastMap_.info.width;

    if (!scanPointsMap_.isEmpty()) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 255, 0, 180));
        for (const auto &pt : scanPointsMap_) {
            double px = (pt.x() - originX) / resolution;
            double py = (gridH - 1) - (pt.y() - originY) / resolution;
            int wx = static_cast<int>(imgX + px * scale);
            int wy = static_cast<int>(imgY + py * scale);
            if (wx >= imgX && wx < imgX + scaled.width() &&
                wy >= imgY && wy < imgY + scaled.height()) {
                painter.drawEllipse(QPointF(wx, wy), 2, 2);
            }
        }
    }

    painter.setPen(Qt::white);
    QString info = QString("Map: %1x%2 | Scan: %3 pts | Topic: %4")
                       .arg(mapImage_.width())
                       .arg(mapImage_.height())
                       .arg(scanPointsMap_.size())
                       .arg(scanTopic_);
    painter.drawText(10, 20, info);
}
