#include "RobotControlWidget.h"
#include "RobotController.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>
#include <cmath>
#include <set>

static const std::set<std::string> STATIC_EXCLUDE = {
    "ground_plane", "floor",
    "wall_north", "wall_south", "wall_east_lower", "wall_east_above_door",
    "wall_east_upper", "wall_west",
    "window_1", "window_1_frame", "window_2", "window_2_frame",
    "window_3", "window_3_frame",
    "door_frame", "door_panel",
    "blackboard", "blackboard_frame",
    "wall_clock", "wall_clock_hand_hour", "wall_clock_hand_minute",
    "platform", "podium", "podium_legs",
    "trash_bin",
    "sun", "ambient_fill", "ceiling_light_center",
};

static bool isStaticModel(const std::string &name)
{
    if (STATIC_EXCLUDE.count(name)) return true;
    for (int r = 2; r <= 4; ++r) {
        for (int c = 1; c <= 6; ++c) {
            char buf[32];
            snprintf(buf, sizeof(buf), "desk_%d_%d", r, c);
            if (name == buf) return true;
            snprintf(buf, sizeof(buf), "chair_%d_%d", r, c);
            if (name == buf) return true;
        }
    }
    return false;
}

RobotControlWidget::RobotControlWidget(std::shared_ptr<rclcpp::Node> node,
                                       RobotController *controller,
                                       QWidget *parent)
    : QWidget(parent), node_(node), controller_(controller)
{
    modelListClient_ = node_->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
    setupUI();

    connect(controller_, &RobotController::velocityChanged,
            this, [this](double linear, double angular) {
        velocityLabel_->setText(QString("速度: %1 m/s  |  角速度: %2 rad/s")
                                .arg(linear, 0, 'f', 2)
                                .arg(angular, 0, 'f', 2));
    });

    QTimer::singleShot(500, this, &RobotControlWidget::refreshRobotList);
}

void RobotControlWidget::setupUI()
{
    auto *mainLayout = new QVBoxLayout(this);

    auto *selectGroup = new QGroupBox("机器人选择");
    auto *selectLayout = new QHBoxLayout(selectGroup);
    robotCombo_ = new QComboBox();
    robotCombo_->setMinimumWidth(150);
    robotCombo_->setEditable(true);
    robotCombo_->setInsertPolicy(QComboBox::NoInsert);
    robotCombo_->lineEdit()->setPlaceholderText("robot_01");
    btnRefresh_ = new QPushButton("刷新");
    btnRefresh_->setMaximumWidth(60);
    selectLayout->addWidget(new QLabel("目标:"));
    selectLayout->addWidget(robotCombo_, 1);
    selectLayout->addWidget(btnRefresh_);
    mainLayout->addWidget(selectGroup);

    auto *speedGroup = new QGroupBox("速度控制");
    auto *speedLayout = new QGridLayout(speedGroup);

    linearLabel_ = new QLabel("线速度 (m/s):");
    linearSlider_ = new QSlider(Qt::Horizontal);
    linearSlider_->setRange(0, 200);
    linearSlider_->setValue(100);
    linearSlider_->setTickInterval(20);
    linearSpin_ = new QDoubleSpinBox();
    linearSpin_->setRange(0.0, 2.0);
    linearSpin_->setValue(0.5);
    linearSpin_->setSingleStep(0.05);
    linearSpin_->setDecimals(2);

    angularLabel_ = new QLabel("角速度 (rad/s):");
    angularSlider_ = new QSlider(Qt::Horizontal);
    angularSlider_->setRange(0, 200);
    angularSlider_->setValue(80);
    angularSlider_->setTickInterval(20);
    angularSpin_ = new QDoubleSpinBox();
    angularSpin_->setRange(0.0, 3.0);
    angularSpin_->setValue(0.8);
    angularSpin_->setSingleStep(0.1);
    angularSpin_->setDecimals(2);

    auto syncSlider = [](QSlider *s, QDoubleSpinBox *sb, double maxVal) {
        QObject::connect(s, &QSlider::valueChanged, [s, sb, maxVal](int v) {
            double val = v / 100.0 * maxVal;
            if (std::abs(sb->value() - val) > 0.005)
                sb->setValue(val);
        });
        QObject::connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                         [s, sb, maxVal](double v) {
            int sv = static_cast<int>(v / maxVal * 100);
            if (s->value() != sv)
                s->setValue(sv);
        });
    };
    syncSlider(linearSlider_, linearSpin_, 2.0);
    syncSlider(angularSlider_, angularSpin_, 3.0);

    speedLayout->addWidget(linearLabel_, 0, 0);
    speedLayout->addWidget(linearSlider_, 0, 1);
    speedLayout->addWidget(linearSpin_, 0, 2);
    speedLayout->addWidget(angularLabel_, 1, 0);
    speedLayout->addWidget(angularSlider_, 1, 1);
    speedLayout->addWidget(angularSpin_, 1, 2);
    mainLayout->addWidget(speedGroup);

    auto *dirGroup = new QGroupBox("方向控制");
    auto *dirLayout = new QGridLayout(dirGroup);

    btnForward_ = new QPushButton("▲  前进");
    btnBackward_ = new QPushButton("▼  后退");
    btnLeft_ = new QPushButton("◀  左转");
    btnRight_ = new QPushButton("▶  右转");
    btnStop_ = new QPushButton("■  急停");
    btnStop_->setStyleSheet(
        "QPushButton { background-color: #e74c3c; color: white; font-weight: bold;"
        " font-size: 14px; padding: 8px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #c0392b; }"
    );

    QString btnStyle = "QPushButton { padding: 10px 20px; font-size: 13px; }";
    btnForward_->setStyleSheet(btnStyle);
    btnBackward_->setStyleSheet(btnStyle);
    btnLeft_->setStyleSheet(btnStyle);
    btnRight_->setStyleSheet(btnStyle);

    dirLayout->addWidget(btnForward_, 0, 1);
    dirLayout->addWidget(btnLeft_, 1, 0);
    dirLayout->addWidget(btnStop_, 1, 1);
    dirLayout->addWidget(btnRight_, 1, 2);
    dirLayout->addWidget(btnBackward_, 2, 1);
    mainLayout->addWidget(dirGroup);

    auto *infoGroup = new QGroupBox("状态");
    auto *infoLayout = new QVBoxLayout(infoGroup);
    topicLabel_ = new QLabel("话题: /robot_01/cmd_vel");
    topicLabel_->setStyleSheet("color: #3498db; font-family: monospace;");
    velocityLabel_ = new QLabel("速度: 0.00 m/s  |  角速度: 0.00 rad/s");
    velocityLabel_->setStyleSheet("color: #27ae60; font-weight: bold;");
    infoLayout->addWidget(topicLabel_);
    infoLayout->addWidget(velocityLabel_);

    btnKeyboardToggle_ = new QPushButton("启用键盘控制 (WASD / 方向键)");
    btnKeyboardToggle_->setCheckable(true);
    btnKeyboardToggle_->setStyleSheet(
        "QPushButton { padding: 6px; }"
        "QPushButton:checked { background-color: #2ecc71; color: white; font-weight: bold; }"
    );
    infoLayout->addWidget(btnKeyboardToggle_);
    mainLayout->addWidget(infoGroup);

    mainLayout->addStretch();

    connect(btnForward_, &QPushButton::pressed, controller_, &RobotController::moveForward);
    connect(btnForward_, &QPushButton::released, controller_, &RobotController::stop);
    connect(btnBackward_, &QPushButton::pressed, controller_, &RobotController::moveBackward);
    connect(btnBackward_, &QPushButton::released, controller_, &RobotController::stop);
    connect(btnLeft_, &QPushButton::pressed, controller_, &RobotController::turnLeft);
    connect(btnLeft_, &QPushButton::released, controller_, &RobotController::stop);
    connect(btnRight_, &QPushButton::pressed, controller_, &RobotController::turnRight);
    connect(btnRight_, &QPushButton::released, controller_, &RobotController::stop);
    connect(btnStop_, &QPushButton::pressed, this, [this]() {
        controller_->stop();
        if (keyboardEnabled_) {
            setKeyboardEnabled(false);
        }
    });

    connect(linearSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            controller_, &RobotController::setLinearSpeed);
    connect(angularSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            controller_, &RobotController::setAngularSpeed);

    connect(btnRefresh_, &QPushButton::clicked, this, &RobotControlWidget::refreshRobotList);
    connect(robotCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &RobotControlWidget::onRobotSelected);
    connect(robotCombo_->lineEdit(), &QLineEdit::editingFinished, this, [this]() {
        controller_->setNamespace(robotCombo_->currentText());
        topicLabel_->setText("话题: " + controller_->cmdVelTopic());
    });

    connect(btnKeyboardToggle_, &QPushButton::toggled, this, [this](bool checked) {
        keyboardEnabled_ = checked;
        btnKeyboardToggle_->setText(checked
            ? "键盘控制: 开启 (WASD / 方向键)"
            : "启用键盘控制 (WASD / 方向键)");
        emit statusMessage(checked ? "键盘控制已启用" : "键盘控制已禁用");
    });
}

void RobotControlWidget::refreshRobotList()
{
    fetchModelList();
}

void RobotControlWidget::onRobotSelected(int index)
{
    if (index < 0) return;
    QString ns = robotCombo_->itemText(index);
    controller_->setNamespace(ns);
    topicLabel_->setText("话题: " + controller_->cmdVelTopic());
}

void RobotControlWidget::fetchModelList()
{
    if (!modelListClient_->wait_for_service(std::chrono::seconds(1))) {
        emit statusMessage("get_model_list 服务不可用");
        return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
    auto result = modelListClient_->async_send_request(request);
    using FutureT = std::shared_future<gazebo_msgs::srv::GetModelList::Response::SharedPtr>;
    auto future = std::make_shared<FutureT>(std::move(result.future));

    auto timer = new QTimer(this);
    timer->setInterval(50);
    connect(timer, &QTimer::timeout, this, [this, future, timer]() {
        if (future->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            timer->stop();
            timer->deleteLater();
            auto response = future->get();
            QString current = robotCombo_->currentText();
            robotCombo_->blockSignals(true);
            robotCombo_->clear();
            int dynamicCount = 0;
            for (const auto &name : response->model_names) {
                if (isStaticModel(name)) continue;
                robotCombo_->addItem(QString::fromStdString(name));
                ++dynamicCount;
            }
            robotCombo_->blockSignals(false);

            int selectIdx = -1;
            if (!current.isEmpty()) {
                selectIdx = robotCombo_->findText(current);
            }
            if (selectIdx < 0) {
                selectIdx = robotCombo_->findText("robot_01");
            }
            if (selectIdx < 0 && robotCombo_->count() > 0) {
                for (int i = 0; i < robotCombo_->count(); ++i) {
                    QString name = robotCombo_->itemText(i);
                    if (name.startsWith("robot_")) {
                        selectIdx = i;
                        break;
                    }
                }
            }
            if (selectIdx < 0 && robotCombo_->count() > 0) {
                selectIdx = 0;
            }
            if (selectIdx >= 0) {
                robotCombo_->setCurrentIndex(selectIdx);
            }

            emit statusMessage(QString("已加载 %1 个动态模型").arg(dynamicCount));
        }
    });
    timer->start();
}

bool RobotControlWidget::isKeyboardEnabled() const
{
    return keyboardEnabled_;
}

void RobotControlWidget::setKeyboardEnabled(bool enabled)
{
    btnKeyboardToggle_->setChecked(enabled);
}
