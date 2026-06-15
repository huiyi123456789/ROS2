#include "MapWidget.h"
#include <QPainter>

MapWidget::MapWidget(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(300, 300);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    mapSub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10),
        std::bind(&MapWidget::updateMap, this, std::placeholders::_1));
}

void MapWidget::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    int width = msg->info.width;
    int height = msg->info.height;

    if (width <= 0 || height <= 0)
        return;

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

void MapWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), QColor(50, 50, 50));

    if (!mapImage_.isNull()) {
        QImage scaled = mapImage_.scaled(size(), Qt::KeepAspectRatio, Qt::FastTransformation);
        int x = (width() - scaled.width()) / 2;
        int y = (height() - scaled.height()) / 2;
        painter.drawImage(x, y, scaled);

        painter.setPen(Qt::white);
        painter.drawText(10, 20, QString("Map: %1x%2").arg(mapImage_.width()).arg(mapImage_.height()));
    } else {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "Waiting for /map...");
    }
}
