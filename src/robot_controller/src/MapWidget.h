#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QImage>
#include <QVector>
#include <QPointF>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);

public slots:
    void setScanTopic(const QString &topic);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void renderStandalone(QPainter &painter);
    void renderWithMap(QPainter &painter);

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    QString scanTopic_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    QImage mapImage_;
    nav_msgs::msg::OccupancyGrid lastMap_;
    QString mapFrameId_;

    QVector<QPointF> scanPointsLaser_;
    QVector<QPointF> scanPointsMap_;
    double scanMaxRange_ = 3.5;
};

#endif
