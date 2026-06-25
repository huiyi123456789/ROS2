#include "MainWindow.h"
#include <QSplitter>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    node_ = std::make_shared<rclcpp::Node>("qt_control_panel");
    setupUI();
    setupStatusBar();
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUI()
{
    setWindowTitle("机器人控制面板 - 仿真管理");
    resize(1400, 800);

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *mainLayout = new QHBoxLayout(central);
    mainLayout->setContentsMargins(4, 4, 4, 4);

    auto *videoGroup = new QGroupBox("摄像头画面");
    auto *videoLayout = new QVBoxLayout(videoGroup);

    frontCamWidget = new RosCameraWidget("/robot_01/camera/image_raw", this);
    backCamWidget = new RosCameraWidget("/robot_01/rear_camera/image_raw", this);
    frontCamWidget->setMinimumSize(320, 200);
    backCamWidget->setMinimumSize(320, 200);

    frontCamWidget->attach(node_, frontCamWidget->topic());
    backCamWidget->attach(node_, backCamWidget->topic());

    videoLayout->addWidget(frontCamWidget);
    videoLayout->addWidget(backCamWidget);

    auto *mapGroup = new QGroupBox("地图视图");
    auto *mapGroupLayout = new QVBoxLayout(mapGroup);
    mapWidget = new MapWidget(node_, this);
    mapWidget->setMinimumSize(320, 250);
    mapGroupLayout->addWidget(mapWidget);

    auto *leftLayout = new QVBoxLayout();
    leftLayout->addWidget(videoGroup, 1);
    leftLayout->addWidget(mapGroup, 1);

    controller = new RobotController(node_);

    controlWidget = new RobotControlWidget(node_, controller, this);

    QString robotSdfPath = QString::fromStdString(
        std::getenv("HOME")) + "/ROS2/install/robot_proj/share/robot_proj/models/turtlebot3_waffle/model.sdf";
    spawnWidget = new SpawnWidget(node_, robotSdfPath, this);

    rightTabs = new QTabWidget();
    rightTabs->addTab(controlWidget, "控制");
    rightTabs->addTab(spawnWidget, "生成 & 传感器");
    rightTabs->setMinimumWidth(380);

    connect(controller, &RobotController::namespaceChanged, this, [this](const QString &ns) {
        updateCameraTopics();
        if (ns.isEmpty()) {
            mapWidget->setScanTopic("/scan");
        } else {
            mapWidget->setScanTopic("/" + ns + "/scan");
        }
    });

    connect(controlWidget, &RobotControlWidget::statusMessage,
            this, [this](const QString &msg) {
        statusBar()->showMessage(msg, 3000);
    });
    connect(spawnWidget, &SpawnWidget::statusMessage,
            this, [this](const QString &msg) {
        statusBar()->showMessage(msg, 3000);
    });

    mainLayout->addLayout(leftLayout, 3);
    mainLayout->addWidget(rightTabs, 1);

    QTimer::singleShot(1000, controlWidget, &RobotControlWidget::refreshRobotList);
    QTimer::singleShot(1500, spawnWidget, &SpawnWidget::refreshModelList);
}

void MainWindow::updateCameraTopics()
{
    QString ns = controller->ns();
    if (!ns.isEmpty()) {
        frontCamWidget->attach(node_, "/" + ns + "/camera/image_raw");
        backCamWidget->attach(node_, "/" + ns + "/rear_camera/image_raw");
    }
}

void MainWindow::setupStatusBar()
{
    statusBar()->showMessage("就绪。使用[控制]标签操控机器人，使用[生成&传感器]标签管理实体。");
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    handleKeyEvent(event, true);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    handleKeyEvent(event, false);
}

void MainWindow::handleKeyEvent(QKeyEvent *event, bool pressed)
{
    if (!controlWidget->isKeyboardEnabled()) {
        if (pressed)
            QMainWindow::keyPressEvent(event);
        else
            QMainWindow::keyReleaseEvent(event);
        return;
    }

    if (event->isAutoRepeat())
        return;

    int key = event->key();
    bool isMovementKey = (key == Qt::Key_W || key == Qt::Key_Up ||
                          key == Qt::Key_S || key == Qt::Key_Down ||
                          key == Qt::Key_A || key == Qt::Key_Left ||
                          key == Qt::Key_D || key == Qt::Key_Right);

    if (!isMovementKey) {
        if (key == Qt::Key_Space || key == Qt::Key_X) {
            if (pressed) {
                controller->stop();
                pressedKeys_.clear();
            }
            return;
        }
        if (pressed)
            QMainWindow::keyPressEvent(event);
        else
            QMainWindow::keyReleaseEvent(event);
        return;
    }

    if (pressed) {
        pressedKeys_.insert(key);
        switch (key) {
        case Qt::Key_W: case Qt::Key_Up:
            controller->moveForward();
            break;
        case Qt::Key_S: case Qt::Key_Down:
            controller->moveBackward();
            break;
        case Qt::Key_A: case Qt::Key_Left:
            controller->turnLeft();
            break;
        case Qt::Key_D: case Qt::Key_Right:
            controller->turnRight();
            break;
        }
    } else {
        pressedKeys_.remove(key);
        if (pressedKeys_.isEmpty()) {
            controller->stop();
        } else {
            int lastKey = *pressedKeys_.begin();
            switch (lastKey) {
            case Qt::Key_W: case Qt::Key_Up:
                controller->moveForward();
                break;
            case Qt::Key_S: case Qt::Key_Down:
                controller->moveBackward();
                break;
            case Qt::Key_A: case Qt::Key_Left:
                controller->turnLeft();
                break;
            case Qt::Key_D: case Qt::Key_Right:
                controller->turnRight();
                break;
            }
        }
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (controlWidget->isKeyboardEnabled()) {
        controlWidget->setKeyboardEnabled(false);
    }
    controller->stop();
    QMainWindow::closeEvent(event);
}
