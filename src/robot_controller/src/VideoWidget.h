#ifndef VIDEOWIDGET_H
#define VIDEOWIDGET_H

#include <QWidget>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QUrl>

class VideoWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VideoWidget(const QString &rtspUrl, QWidget *parent = nullptr);
    ~VideoWidget();

    void setRtspUrl(const QString &rtspUrl);

private:
    QMediaPlayer *player;
    QVideoWidget *videoOutput;
};

#endif
