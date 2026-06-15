#include "VideoWidget.h"
#include <QVBoxLayout>
#include <QDebug>

VideoWidget::VideoWidget(const QString &rtspUrl, QWidget *parent)
    : QWidget(parent)
{
    player = new QMediaPlayer(this);
    videoOutput = new QVideoWidget(this);
    player->setVideoOutput(videoOutput);

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(videoOutput);

    player->setSource(QUrl(rtspUrl));
    player->play();

    connect(player, &QMediaPlayer::errorOccurred, [this](QMediaPlayer::Error error) {
        qDebug() << "Video player error:" << error << player->errorString();
    });
}

VideoWidget::~VideoWidget()
{
    player->stop();
}

void VideoWidget::setRtspUrl(const QString &rtspUrl)
{
    player->stop();
    player->setSource(QUrl(rtspUrl));
    player->play();
}
