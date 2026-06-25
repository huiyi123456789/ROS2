#ifndef ROBOTCONTROLWIDGET_H
#define ROBOTCONTROLWIDGET_H

#include <QWidget>
#include <QComboBox>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QEvent>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_model_list.hpp>

class RobotController;

class RobotControlWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RobotControlWidget(std::shared_ptr<rclcpp::Node> node,
                                RobotController *controller,
                                QWidget *parent = nullptr);

    void setKeyboardEnabled(bool enabled);
    bool isKeyboardEnabled() const;

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

signals:
    void statusMessage(const QString &msg);

public slots:
    void refreshRobotList();
    void onRobotSelected(int index);

private:
    void setupUI();
    void updateSpeedLabels();
    void fetchModelList();

    std::shared_ptr<rclcpp::Node> node_;
    RobotController *controller_;

    QComboBox *robotCombo_;
    QSlider *linearSlider_;
    QSlider *angularSlider_;
    QDoubleSpinBox *linearSpin_;
    QDoubleSpinBox *angularSpin_;
    QLabel *linearLabel_;
    QLabel *angularLabel_;
    QLabel *topicLabel_;
    QLabel *velocityLabel_;
    QPushButton *btnForward_;
    QPushButton *btnBackward_;
    QPushButton *btnLeft_;
    QPushButton *btnRight_;
    QPushButton *btnStop_;
    QPushButton *btnRefresh_;
    QPushButton *btnKeyboardToggle_;

    bool keyboardEnabled_ = false;

    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr modelListClient_;
};

#endif
