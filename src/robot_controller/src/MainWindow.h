#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTabWidget>
#include <QStatusBar>
#include <QKeyEvent>
#include <QSet>
#include <rclcpp/rclcpp.hpp>
#include "VideoWidget.h"
#include "RosCameraWidget.h"
#include "MapWidget.h"
#include "RobotController.h"
#include "RobotControlWidget.h"
#include "SpawnWidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    std::shared_ptr<rclcpp::Node> getNode() { return node_; }

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

private:
    void setupUI();
    void setupStatusBar();
    void handleKeyEvent(QKeyEvent *event, bool pressed);
    void updateCameraTopics();

    std::shared_ptr<rclcpp::Node> node_;

    VideoWidget *frontRtspWidget;
    VideoWidget *backRtspWidget;
    RosCameraWidget *frontCamWidget;
    RosCameraWidget *backCamWidget;
    MapWidget *mapWidget;
    RobotController *controller;
    RobotControlWidget *controlWidget;
    SpawnWidget *spawnWidget;
    QTabWidget *rightTabs;
    QSet<int> pressedKeys_;
};

#endif
