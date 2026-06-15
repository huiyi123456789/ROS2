#include "SpawnWidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <cmath>
#include <set>

static const std::set<std::string> STATIC_MODELS = {
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
    if (STATIC_MODELS.count(name)) return true;
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

SpawnWidget::SpawnWidget(std::shared_ptr<rclcpp::Node> node,
                         const QString &robotSdfPath,
                         QWidget *parent)
    : QWidget(parent), node_(node), robotSdfPath_(robotSdfPath)
{
    spawnClient_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    deleteClient_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    modelListClient_ = node_->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");

    QFile f(robotSdfPath_);
    if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&f);
        robotSdfContent_ = in.readAll();
        f.close();
    }

    setupUI();
    QTimer::singleShot(800, this, &SpawnWidget::refreshModelList);
}

void SpawnWidget::setupUI()
{
    auto *mainLayout = new QVBoxLayout(this);

    auto *spawnGroup = new QGroupBox("机器人生成 / 删除");
    auto *spawnLayout = new QGridLayout(spawnGroup);

    spawnLayout->addWidget(new QLabel("名称:"), 0, 0);
    robotNameEdit_ = new QLineEdit("robot_02");
    spawnLayout->addWidget(robotNameEdit_, 0, 1, 1, 3);

    spawnLayout->addWidget(new QLabel("X:"), 1, 0);
    robotXEdit_ = new QLineEdit("0.0");
    spawnLayout->addWidget(robotXEdit_, 1, 1);
    spawnLayout->addWidget(new QLabel("Y:"), 1, 2);
    robotYEdit_ = new QLineEdit("0.0");
    spawnLayout->addWidget(robotYEdit_, 1, 3);
    spawnLayout->addWidget(new QLabel("Z:"), 2, 0);
    robotZEdit_ = new QLineEdit("0.05");
    spawnLayout->addWidget(robotZEdit_, 2, 1);

    btnSpawn_ = new QPushButton("生成机器人");
    btnSpawn_->setStyleSheet("QPushButton { background-color: #2ecc71; color: white; font-weight: bold; padding: 6px; }");
    btnDelete_ = new QPushButton("删除机器人");
    btnDelete_->setStyleSheet("QPushButton { background-color: #e74c3c; color: white; font-weight: bold; padding: 6px; }");

    spawnLayout->addWidget(btnSpawn_, 2, 2);
    spawnLayout->addWidget(btnDelete_, 2, 3);
    mainLayout->addWidget(spawnGroup);

    auto *sensorGroup = new QGroupBox("传感器管理");
    auto *sensorLayout = new QGridLayout(sensorGroup);

    sensorLayout->addWidget(new QLabel("名称:"), 0, 0);
    sensorNameEdit_ = new QLineEdit("sensor_01");
    sensorLayout->addWidget(sensorNameEdit_, 0, 1, 1, 3);

    sensorLayout->addWidget(new QLabel("X:"), 1, 0);
    sensorXEdit_ = new QLineEdit("0.0");
    sensorLayout->addWidget(sensorXEdit_, 1, 1);
    sensorLayout->addWidget(new QLabel("Y:"), 1, 2);
    sensorYEdit_ = new QLineEdit("0.0");
    sensorLayout->addWidget(sensorYEdit_, 1, 3);

    sensorLayout->addWidget(new QLabel("Z:"), 2, 0);
    sensorZEdit_ = new QLineEdit("0.15");
    sensorLayout->addWidget(sensorZEdit_, 2, 1);

    auto *btnLidar2D = new QPushButton("+ 2D 激光雷达");
    auto *btnLidar3D = new QPushButton("+ 3D 激光雷达");
    auto *btnCamera = new QPushButton("+ RGB 相机");
    auto *btnDepth = new QPushButton("+ 深度相机");
    auto *btnIMU = new QPushButton("+ IMU");
    auto *btnDelSensor = new QPushButton("- 删除传感器");
    btnDelSensor->setStyleSheet("QPushButton { color: #e74c3c; }");

    sensorLayout->addWidget(btnLidar2D, 3, 0);
    sensorLayout->addWidget(btnLidar3D, 3, 1);
    sensorLayout->addWidget(btnCamera, 3, 2);
    sensorLayout->addWidget(btnDepth, 3, 3);
    sensorLayout->addWidget(btnIMU, 4, 0);
    sensorLayout->addWidget(btnDelSensor, 4, 1, 1, 3);
    mainLayout->addWidget(sensorGroup);

    auto *scenarioGroup = new QGroupBox("快速场景");
    auto *scenarioLayout = new QHBoxLayout(scenarioGroup);
    auto *btnS1 = new QPushButton("1: 教室导航");
    auto *btnS2 = new QPushButton("2: 多机器人");
    auto *btnS3 = new QPushButton("3: 传感器套件");
    auto *btnS4 = new QPushButton("4: 清理场景");
    scenarioLayout->addWidget(btnS1);
    scenarioLayout->addWidget(btnS2);
    scenarioLayout->addWidget(btnS3);
    scenarioLayout->addWidget(btnS4);
    mainLayout->addWidget(scenarioGroup);

    auto *modelGroup = new QGroupBox("当前模型");
    auto *modelLayout = new QHBoxLayout(modelGroup);
    modelListCombo_ = new QComboBox();
    modelListCombo_->setMinimumWidth(150);
    modelListCombo_->setEditable(false);
    btnRefreshModels_ = new QPushButton("刷新");
    btnRefreshModels_->setMaximumWidth(60);
    modelLayout->addWidget(modelListCombo_, 1);
    modelLayout->addWidget(btnRefreshModels_);
    mainLayout->addWidget(modelGroup);

    auto *logGroup = new QGroupBox("日志");
    auto *logLayout = new QVBoxLayout(logGroup);
    logOutput_ = new QTextEdit();
    logOutput_->setReadOnly(true);
    logOutput_->setMaximumHeight(120);
    logOutput_->setStyleSheet("QTextEdit { background-color: #2c3e50; color: #ecf0f1; font-family: monospace; font-size: 11px; }");
    logLayout->addWidget(logOutput_);
    mainLayout->addWidget(logGroup);

    connect(btnSpawn_, &QPushButton::clicked, this, &SpawnWidget::spawnRobot);
    connect(btnDelete_, &QPushButton::clicked, this, &SpawnWidget::deleteSelectedRobot);
    connect(btnLidar2D, &QPushButton::clicked, this, &SpawnWidget::addSensor2DLidar);
    connect(btnLidar3D, &QPushButton::clicked, this, &SpawnWidget::addSensor3DLidar);
    connect(btnCamera, &QPushButton::clicked, this, &SpawnWidget::addSensorCamera);
    connect(btnDepth, &QPushButton::clicked, this, &SpawnWidget::addSensorDepthCamera);
    connect(btnIMU, &QPushButton::clicked, this, &SpawnWidget::addSensorIMU);
    connect(btnDelSensor, &QPushButton::clicked, this, &SpawnWidget::deleteSensor);
    connect(btnRefreshModels_, &QPushButton::clicked, this, &SpawnWidget::refreshModelList);

    connect(btnS1, &QPushButton::clicked, this, [this]() { runScenario(1); });
    connect(btnS2, &QPushButton::clicked, this, [this]() { runScenario(2); });
    connect(btnS3, &QPushButton::clicked, this, [this]() { runScenario(3); });
    connect(btnS4, &QPushButton::clicked, this, [this]() { runScenario(4); });
}

void SpawnWidget::callSpawnService(const QString &name, const QString &sdfXml,
                                   double x, double y, double z)
{
    if (!spawnClient_->wait_for_service(std::chrono::seconds(1))) {
        logOutput_->append("[错误] /spawn_entity 服务不可用");
        return;
    }

    auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    req->name = name.toStdString();
    req->xml = sdfXml.toStdString();
    req->robot_namespace = name.toStdString();
    req->initial_pose.position.x = x;
    req->initial_pose.position.y = y;
    req->initial_pose.position.z = z;
    req->initial_pose.orientation.w = 1.0;

    auto result = spawnClient_->async_send_request(req);
    using FutureT = std::shared_future<gazebo_msgs::srv::SpawnEntity::Response::SharedPtr>;
    auto future = std::make_shared<FutureT>(std::move(result.future));
    auto timer = new QTimer(this);
    timer->setInterval(50);
    connect(timer, &QTimer::timeout, this, [this, name, future, timer]() {
        if (future->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            timer->stop();
            timer->deleteLater();
            auto res = future->get();
            if (res->success) {
                logOutput_->append(QString("[成功] 已生成: %1").arg(name));
                refreshModelList();
            } else {
                logOutput_->append(QString("[失败] %1: %2").arg(name, QString::fromStdString(res->status_message)));
            }
        }
    });
    timer->start();
}

void SpawnWidget::callDeleteService(const QString &name)
{
    if (!deleteClient_->wait_for_service(std::chrono::seconds(1))) {
        logOutput_->append("[错误] /delete_entity 服务不可用");
        return;
    }

    auto req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    req->name = name.toStdString();

    auto result = deleteClient_->async_send_request(req);
    using FutureT = std::shared_future<gazebo_msgs::srv::DeleteEntity::Response::SharedPtr>;
    auto future = std::make_shared<FutureT>(std::move(result.future));
    auto timer = new QTimer(this);
    timer->setInterval(50);
    connect(timer, &QTimer::timeout, this, [this, name, future, timer]() {
        if (future->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            timer->stop();
            timer->deleteLater();
            auto res = future->get();
            if (res->success) {
                logOutput_->append(QString("[成功] 已删除: %1").arg(name));
                refreshModelList();
            } else {
                logOutput_->append(QString("[失败] 删除 %1: %2").arg(name, QString::fromStdString(res->status_message)));
            }
        }
    });
    timer->start();
}

void SpawnWidget::spawnRobot()
{
    if (robotSdfContent_.isEmpty()) {
        logOutput_->append("[错误] 机器人 SDF 文件未加载");
        return;
    }
    QString name = robotNameEdit_->text().trimmed();
    if (name.isEmpty()) name = "robot_02";
    double x = robotXEdit_->text().toDouble();
    double y = robotYEdit_->text().toDouble();
    double z = robotZEdit_->text().toDouble();
    callSpawnService(name, robotSdfContent_, x, y, z);
}

void SpawnWidget::deleteSelectedRobot()
{
    QString name = robotNameEdit_->text().trimmed();
    if (name.isEmpty()) {
        if (modelListCombo_->currentIndex() >= 0)
            name = modelListCombo_->currentText();
    }
    if (name.isEmpty()) {
        logOutput_->append("[错误] 未指定机器人名称");
        return;
    }
    callDeleteService(name);
}

void SpawnWidget::addSensor2DLidar()
{
    QString name = sensorNameEdit_->text().trimmed();
    if (name.isEmpty()) name = "sensor_lidar2d";
    double x = sensorXEdit_->text().toDouble();
    double y = sensorYEdit_->text().toDouble();
    double z = sensorZEdit_->text().toDouble();
    callSpawnService(name, makeLidar2DSdf(name, x, y, z), x, y, z);
}

void SpawnWidget::addSensor3DLidar()
{
    QString name = sensorNameEdit_->text().trimmed();
    if (name.isEmpty()) name = "sensor_lidar3d";
    double x = sensorXEdit_->text().toDouble();
    double y = sensorYEdit_->text().toDouble();
    double z = sensorZEdit_->text().toDouble();
    callSpawnService(name, makeLidar3DSdf(name, x, y, z), x, y, z);
}

void SpawnWidget::addSensorCamera()
{
    QString name = sensorNameEdit_->text().trimmed();
    if (name.isEmpty()) name = "sensor_camera";
    double x = sensorXEdit_->text().toDouble();
    double y = sensorYEdit_->text().toDouble();
    double z = sensorZEdit_->text().toDouble();
    callSpawnService(name, makeCameraSdf(name, x, y, z), x, y, z);
}

void SpawnWidget::addSensorDepthCamera()
{
    QString name = sensorNameEdit_->text().trimmed();
    if (name.isEmpty()) name = "sensor_depth";
    double x = sensorXEdit_->text().toDouble();
    double y = sensorYEdit_->text().toDouble();
    double z = sensorZEdit_->text().toDouble();
    callSpawnService(name, makeDepthCameraSdf(name, x, y, z), x, y, z);
}

void SpawnWidget::addSensorIMU()
{
    QString name = sensorNameEdit_->text().trimmed();
    if (name.isEmpty()) name = "sensor_imu";
    double x = sensorXEdit_->text().toDouble();
    double y = sensorYEdit_->text().toDouble();
    double z = sensorZEdit_->text().toDouble();
    callSpawnService(name, makeImuSdf(name, x, y, z), x, y, z);
}

void SpawnWidget::deleteSensor()
{
    QString name = sensorNameEdit_->text().trimmed();
    if (name.isEmpty()) {
        logOutput_->append("[错误] 未指定传感器名称");
        return;
    }
    callDeleteService(name);
}

void SpawnWidget::runScenario(int id)
{
    if (id == 1) {
        callSpawnService("nav_robot", robotSdfContent_, -2.0, 0.0, 0.05);
    } else if (id == 2) {
        callSpawnService("robot_leader", robotSdfContent_, -2.0, 0.0, 0.05);
        QTimer::singleShot(600, this, [this]() {
            callSpawnService("robot_left", robotSdfContent_, -3.0, -1.0, 0.05);
        });
        QTimer::singleShot(1200, this, [this]() {
            callSpawnService("robot_right", robotSdfContent_, -3.0, 1.0, 0.05);
        });
    } else if (id == 3) {
        callSpawnService("sensor_bot", robotSdfContent_, -1.0, 0.0, 0.05);
        QTimer::singleShot(1000, this, [this]() {
            callSpawnService("extra_lidar", makeLidar2DSdf("extra_lidar", -1.0, 0.0, 0.25),
                             -1.0, 0.0, 0.25);
            callSpawnService("extra_lidar3d", makeLidar3DSdf("extra_lidar3d", -1.0, 0.5, 0.25),
                             -1.0, 0.5, 0.25);
            callSpawnService("extra_camera", makeCameraSdf("extra_camera", -1.0, -0.5, 0.25),
                             -1.0, -0.5, 0.25);
            callSpawnService("extra_depth", makeDepthCameraSdf("extra_depth", -1.0, 0.8, 0.25),
                             -1.0, 0.8, 0.25);
        });
    } else if (id == 4) {
        fetchModelList();
        QTimer::singleShot(500, this, [this]() {
            for (int i = 0; i < modelListCombo_->count(); ++i) {
                QString name = modelListCombo_->itemText(i);
                if (name.startsWith("robot_") || name.startsWith("sensor_") ||
                    name.startsWith("extra_") || name.startsWith("nav_") ||
                    name.startsWith("demo_") || name.startsWith("sensor_bot")) {
                    callDeleteService(name);
                }
            }
        });
    }
}

void SpawnWidget::refreshModelList()
{
    fetchModelList();
}

void SpawnWidget::fetchModelList()
{
    if (!modelListClient_->wait_for_service(std::chrono::seconds(1))) return;
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
            auto res = future->get();
            modelListCombo_->clear();
            for (const auto &name : res->model_names) {
                if (isStaticModel(name)) continue;
                modelListCombo_->addItem(QString::fromStdString(name));
            }
        }
    });
    timer->start();
}

QString SpawnWidget::makeLidar2DSdf(const QString &name, double x, double y, double z)
{
    return QString(
        "<?xml version=\"1.0\" ?>"
        "<sdf version=\"1.6\">"
        "  <model name=\"%1\">"
        "    <pose>%2 %3 %4 0 0 0</pose>"
        "    <link name=\"%1_link\">"
        "      <visual name=\"visual\">"
        "        <geometry><cylinder><radius>0.04</radius><length>0.03</length></cylinder></geometry>"
        "        <material><ambient>1 0 0 1</ambient></material>"
        "      </visual>"
        "      <sensor name=\"lidar\" type=\"ray\">"
        "        <always_on>1</always_on><visualize>true</visualize><update_rate>10</update_rate>"
        "        <ray>"
        "          <scan><horizontal><samples>360</samples><resolution>1</resolution>"
        "            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>"
        "          <range><min>0.12</min><max>10.0</max><resolution>0.01</resolution></range>"
        "          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>"
        "        </ray>"
        "        <plugin name=\"lidar_plugin\" filename=\"libgazebo_ros_ray_sensor.so\">"
        "          <ros><argument>~/out:=scan</argument></ros>"
        "          <output_type>sensor_msgs/LaserScan</output_type>"
        "          <frame_name>%1_link</frame_name>"
        "        </plugin>"
        "      </sensor>"
        "    </link>"
        "  </model>"
        "</sdf>"
    ).arg(name).arg(x).arg(y).arg(z);
}

QString SpawnWidget::makeLidar3DSdf(const QString &name, double x, double y, double z)
{
    return QString(
        "<?xml version=\"1.0\" ?>"
        "<sdf version=\"1.6\">"
        "  <model name=\"%1\">"
        "    <pose>%2 %3 %4 0 0 0</pose>"
        "    <link name=\"%1_link\">"
        "      <visual name=\"visual\">"
        "        <geometry><cylinder><radius>0.1</radius><length>0.07</length></cylinder></geometry>"
        "        <material><ambient>0.8 0.8 0.8 1</ambient></material>"
        "      </visual>"
        "      <sensor name=\"lidar3d\" type=\"ray\">"
        "        <always_on>1</always_on><update_rate>10</update_rate>"
        "        <ray>"
        "          <scan>"
        "            <horizontal><samples>1800</samples><resolution>1</resolution>"
        "              <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>"
        "            <vertical><samples>16</samples><resolution>1</resolution>"
        "              <min_angle>-0.261799</min_angle><max_angle>0.261799</max_angle></vertical>"
        "          </scan>"
        "          <range><min>0.1</min><max>100.0</max><resolution>0.02</resolution></range>"
        "        </ray>"
        "        <plugin name=\"velodyne_plugin\" filename=\"libgazebo_ros_ray_sensor.so\">"
        "          <ros></ros>"
        "          <output_type>sensor_msgs/PointCloud2</output_type>"
        "          <frame_name>%1_link</frame_name>"
        "        </plugin>"
        "      </sensor>"
        "    </link>"
        "  </model>"
        "</sdf>"
    ).arg(name).arg(x).arg(y).arg(z);
}

QString SpawnWidget::makeCameraSdf(const QString &name, double x, double y, double z)
{
    return QString(
        "<?xml version=\"1.0\" ?>"
        "<sdf version=\"1.6\">"
        "  <model name=\"%1\">"
        "    <pose>%2 %3 %4 0 0 0</pose>"
        "    <link name=\"%1_link\">"
        "      <visual name=\"visual\">"
        "        <geometry><box><size>0.02 0.04 0.02</size></box></geometry>"
        "        <material><ambient>0 1 0 1</ambient></material>"
        "      </visual>"
        "      <sensor name=\"camera\" type=\"camera\">"
        "        <always_on>1</always_on><update_rate>30</update_rate><visualize>true</visualize>"
        "        <camera>"
        "          <horizontal_fov>1.047</horizontal_fov>"
        "          <image><width>640</width><height>480</height><format>R8G8B8</format></image>"
        "          <clip><near>0.1</near><far>100</far></clip>"
        "          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>"
        "        </camera>"
        "        <plugin name=\"camera_plugin\" filename=\"libgazebo_ros_camera.so\">"
        "          <ros></ros><frame_name>%1_link</frame_name>"
        "        </plugin>"
        "      </sensor>"
        "    </link>"
        "  </model>"
        "</sdf>"
    ).arg(name).arg(x).arg(y).arg(z);
}

QString SpawnWidget::makeDepthCameraSdf(const QString &name, double x, double y, double z)
{
    return QString(
        "<?xml version=\"1.0\" ?>"
        "<sdf version=\"1.6\">"
        "  <model name=\"%1\">"
        "    <pose>%2 %3 %4 0 0 0</pose>"
        "    <link name=\"%1_link\">"
        "      <visual name=\"visual\">"
        "        <geometry><box><size>0.02 0.05 0.03</size></box></geometry>"
        "        <material><ambient>0 0.6 1 1</ambient></material>"
        "      </visual>"
        "      <sensor name=\"depth_camera\" type=\"depth_camera\">"
        "        <always_on>1</always_on><update_rate>30</update_rate>"
        "        <camera>"
        "          <horizontal_fov>1.047</horizontal_fov>"
        "          <image><width>640</width><height>480</height></image>"
        "          <clip><near>0.1</near><far>30</far></clip>"
        "        </camera>"
        "        <plugin name=\"depth_plugin\" filename=\"libgazebo_ros_depth_camera.so\">"
        "          <ros></ros><frame_name>%1_link</frame_name>"
        "        </plugin>"
        "      </sensor>"
        "    </link>"
        "  </model>"
        "</sdf>"
    ).arg(name).arg(x).arg(y).arg(z);
}

QString SpawnWidget::makeImuSdf(const QString &name, double x, double y, double z)
{
    return QString(
        "<?xml version=\"1.0\" ?>"
        "<sdf version=\"1.6\">"
        "  <model name=\"%1\">"
        "    <pose>%2 %3 %4 0 0 0</pose>"
        "    <link name=\"%1_link\">"
        "      <sensor name=\"imu\" type=\"imu\">"
        "        <always_on>1</always_on><update_rate>200</update_rate>"
        "        <imu>"
        "          <angular_velocity>"
        "            <x><noise type=\"gaussian\"><mean>0</mean><stddev>0.00017</stddev></noise></x>"
        "            <y><noise type=\"gaussian\"><mean>0</mean><stddev>0.00017</stddev></noise></y>"
        "            <z><noise type=\"gaussian\"><mean>0</mean><stddev>0.00017</stddev></noise></z>"
        "          </angular_velocity>"
        "          <linear_acceleration>"
        "            <x><noise type=\"gaussian\"><mean>0</mean><stddev>0.001</stddev></noise></x>"
        "            <y><noise type=\"gaussian\"><mean>0</mean><stddev>0.001</stddev></noise></y>"
        "            <z><noise type=\"gaussian\"><mean>0</mean><stddev>0.001</stddev></noise></z>"
        "          </linear_acceleration>"
        "        </imu>"
        "        <plugin name=\"imu_plugin\" filename=\"libgazebo_ros_imu_sensor.so\">"
        "          <ros></ros><frame_name>%1_link</frame_name>"
        "        </plugin>"
        "      </sensor>"
        "    </link>"
        "  </model>"
        "</sdf>"
    ).arg(name).arg(x).arg(y).arg(z);
}
