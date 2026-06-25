#include "RobotController.h"

RobotController::RobotController(std::shared_ptr<rclcpp::Node> node)
    : node_(node), stopped_(true)
{
    namespace_ = "robot_01";
    rebuildPublisher();

    publishTimer_ = new QTimer(this);
    publishTimer_->setInterval(50);
    connect(publishTimer_, &QTimer::timeout, this, &RobotController::publishCurrent);
    publishTimer_->start();

    safetyTimer_ = new QTimer(this);
    safetyTimer_->setSingleShot(true);
    safetyTimer_->setInterval(5000);
    connect(safetyTimer_, &QTimer::timeout, this, [this]() {
        if (!stopped_) {
            stop();
        }
    });
}

RobotController::~RobotController() = default;

void RobotController::setNamespace(const QString &ns)
{
    if (namespace_ != ns) {
        namespace_ = ns;
        rebuildPublisher();
        emit namespaceChanged(ns);
    }
}

void RobotController::setLinearSpeed(double val)
{
    linearSpeed_ = std::max(0.0, std::min(val, 10.0));
}

void RobotController::setAngularSpeed(double val)
{
    angularSpeed_ = std::max(0.0, std::min(val, 10.0));
}

void RobotController::setCmdVelTopic(const QString &topic)
{
    cmdVelExplicitTopic_ = topic;
    rebuildPublisher();
}

QString RobotController::cmdVelTopic() const
{
    return QString::fromStdString(cmdPub_ ? cmdPub_->get_topic_name() : "");
}

void RobotController::rebuildPublisher()
{
    std::string topic;
    if (!cmdVelExplicitTopic_.isEmpty()) {
        topic = cmdVelExplicitTopic_.toStdString();
    } else if (!namespace_.isEmpty()) {
        topic = "/" + namespace_.toStdString() + "/cmd_vel";
    } else {
        topic = "/cmd_vel";
    }
    cmdPub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
}

void RobotController::publishVelocity(double linear, double angular)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmdPub_->publish(msg);
    emit velocityChanged(linear, angular);
}

void RobotController::publishCurrent()
{
    if (stopped_) {
        publishVelocity(0.0, 0.0);
    } else {
        publishVelocity(cmdLinear_, cmdAngular_);
    }
}

void RobotController::moveForward()
{
    stopped_ = false;
    cmdLinear_ = linearSpeed_;
    cmdAngular_ = 0.0;
    safetyTimer_->start();
}

void RobotController::moveBackward()
{
    stopped_ = false;
    cmdLinear_ = -linearSpeed_;
    cmdAngular_ = 0.0;
    safetyTimer_->start();
}

void RobotController::turnLeft()
{
    stopped_ = false;
    cmdLinear_ = 0.0;
    cmdAngular_ = angularSpeed_;
    safetyTimer_->start();
}

void RobotController::turnRight()
{
    stopped_ = false;
    cmdLinear_ = 0.0;
    cmdAngular_ = -angularSpeed_;
    safetyTimer_->start();
}

void RobotController::stop()
{
    stopped_ = true;
    cmdLinear_ = 0.0;
    cmdAngular_ = 0.0;
    safetyTimer_->stop();
}
