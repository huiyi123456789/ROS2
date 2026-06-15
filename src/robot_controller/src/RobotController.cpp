#include "RobotController.h"

RobotController::RobotController(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    namespace_ = "robot_01";
    rebuildPublisher();
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

void RobotController::moveForward()
{
    publishVelocity(linearSpeed_, 0.0);
}

void RobotController::moveBackward()
{
    publishVelocity(-linearSpeed_, 0.0);
}

void RobotController::turnLeft()
{
    publishVelocity(0.0, angularSpeed_);
}

void RobotController::turnRight()
{
    publishVelocity(0.0, -angularSpeed_);
}

void RobotController::stop()
{
    publishVelocity(0.0, 0.0);
}
