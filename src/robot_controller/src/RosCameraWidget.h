#ifndef ROSCAMERAWIDGET_H
#define ROSCAMERAWIDGET_H

#include <QWidget>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class RosCameraWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RosCameraWidget(const QString &defaultTopic,
                             QWidget *parent = nullptr);

    void attach(std::shared_ptr<rclcpp::Node> node, const QString &topic);
    void setTopic(const QString &topic);
    QString topic() const { return topic_; }

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    QString topic_;
    QImage frame_;
};

#endif
