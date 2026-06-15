#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
    QImage mapImage_;
};

#endif
