#ifndef SPAWNWIDGET_H
#define SPAWNWIDGET_H

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QLabel>
#include <QComboBox>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/get_model_list.hpp>

class SpawnWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SpawnWidget(std::shared_ptr<rclcpp::Node> node,
                         const QString &robotSdfPath,
                         QWidget *parent = nullptr);

signals:
    void statusMessage(const QString &msg);

public slots:
    void refreshModelList();

private slots:
    void spawnRobot();
    void deleteSelectedRobot();
    void addSensor2DLidar();
    void addSensor3DLidar();
    void addSensorCamera();
    void addSensorDepthCamera();
    void addSensorIMU();
    void deleteSensor();
    void runScenario(int id);

private:
    void setupUI();
    void callSpawnService(const QString &name, const QString &sdfXml,
                          double x = 0, double y = 0, double z = 0.1);
    void callDeleteService(const QString &name);
    void fetchModelList();

    QString makeLidar2DSdf(const QString &name, double x, double y, double z);
    QString makeLidar3DSdf(const QString &name, double x, double y, double z);
    QString makeCameraSdf(const QString &name, double x, double y, double z);
    QString makeDepthCameraSdf(const QString &name, double x, double y, double z);
    QString makeImuSdf(const QString &name, double x, double y, double z);

    std::shared_ptr<rclcpp::Node> node_;
    QString robotSdfPath_;
    QString robotSdfContent_;

    QLineEdit *robotNameEdit_;
    QLineEdit *robotXEdit_;
    QLineEdit *robotYEdit_;
    QLineEdit *robotZEdit_;
    QPushButton *btnSpawn_;
    QPushButton *btnDelete_;

    QLineEdit *sensorNameEdit_;
    QLineEdit *sensorXEdit_;
    QLineEdit *sensorYEdit_;
    QLineEdit *sensorZEdit_;

    QComboBox *modelListCombo_;
    QPushButton *btnRefreshModels_;
    QTextEdit *logOutput_;

    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawnClient_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr deleteClient_;
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr modelListClient_;
};

#endif
