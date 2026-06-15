#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "MainWindow.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    auto node = window.getNode();
    auto *timer = new QTimer(&window);
    QObject::connect(timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    timer->start(10);

    int result = app.exec();

    rclcpp::shutdown();
    return result;
}
