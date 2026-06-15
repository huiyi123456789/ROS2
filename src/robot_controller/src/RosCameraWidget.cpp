#include "RosCameraWidget.h"
#include <QPainter>
#include <cstring>

RosCameraWidget::RosCameraWidget(const QString &defaultTopic, QWidget *parent)
    : QWidget(parent), topic_(defaultTopic)
{
    setMinimumSize(320, 200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void RosCameraWidget::attach(std::shared_ptr<rclcpp::Node> node, const QString &topic)
{
    sub_.reset();
    topic_ = topic;

    if (topic.isEmpty() || !node) return;

    sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        topic.toStdString(),
        rclcpp::SensorDataQoS(),
        std::bind(&RosCameraWidget::imageCallback, this, std::placeholders::_1));
}

void RosCameraWidget::setTopic(const QString &topic)
{
    topic_ = topic;
}

void RosCameraWidget::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    int w = static_cast<int>(msg->width);
    int h = static_cast<int>(msg->height);
    if (w <= 0 || h <= 0) return;
    if (msg->data.size() < w * h * 3) return;

    const std::string &enc = msg->encoding;
    QImage::Format fmt = QImage::Format_RGB888;
    bool needRgbSwap = false;

    if (enc == "rgb8") {
        fmt = QImage::Format_RGB888;
    } else if (enc == "bgr8") {
        fmt = QImage::Format_RGB888;
        needRgbSwap = true;
    } else if (enc == "rgba8") {
        fmt = QImage::Format_RGBA8888;
    } else if (enc == "bgra8") {
        fmt = QImage::Format_RGBA8888;
        needRgbSwap = true;
    } else {
        return;
    }

    QImage img(w, h, fmt);
    if (msg->step == 0 || msg->step == static_cast<unsigned int>(w * (enc.find("a") != std::string::npos ? 4 : 3))) {
        std::memcpy(img.bits(), msg->data.data(), msg->data.size());
    } else {
        int channels = (fmt == QImage::Format_RGBA8888) ? 4 : 3;
        for (int row = 0; row < h; ++row) {
            std::memcpy(img.scanLine(row),
                        msg->data.data() + row * msg->step,
                        w * channels);
        }
    }

    if (needRgbSwap) {
        img = img.rgbSwapped();
    }

    frame_ = img.mirrored(false, true);

    update();
}

void RosCameraWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), QColor(30, 30, 30));

    if (!frame_.isNull()) {
        QImage scaled = frame_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        int x = (width() - scaled.width()) / 2;
        int y = (height() - scaled.height()) / 2;
        painter.drawImage(x, y, scaled);
    } else {
        painter.setPen(QColor(150, 150, 150));
        painter.drawText(rect(), Qt::AlignCenter,
                         QString("Waiting for %1...").arg(topic_));
    }
}
