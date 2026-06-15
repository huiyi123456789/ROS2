#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RobotController : public QObject
{
    Q_OBJECT
public:
    explicit RobotController(std::shared_ptr<rclcpp::Node> node);
    ~RobotController();

    void setNamespace(const QString &ns);
    QString ns() const { return namespace_; }

    void setLinearSpeed(double val);
    double linearSpeed() const { return linearSpeed_; }

    void setAngularSpeed(double val);
    double angularSpeed() const { return angularSpeed_; }

    void setCmdVelTopic(const QString &topic);
    QString cmdVelTopic() const;

    bool isStopped() const { return stopped_; }

signals:
    void namespaceChanged(const QString &ns);
    void velocityChanged(double linear, double angular);

public slots:
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void stop();

private slots:
    void publishCurrent();

private:
    void publishVelocity(double linear, double angular);
    void rebuildPublisher();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
    QTimer *publishTimer_;
    QString namespace_;
    QString cmdVelExplicitTopic_;
    double linearSpeed_ = 0.5;
    double angularSpeed_ = 0.8;
    double cmdLinear_ = 0.0;
    double cmdAngular_ = 0.0;
    bool stopped_ = true;
};

#endif
