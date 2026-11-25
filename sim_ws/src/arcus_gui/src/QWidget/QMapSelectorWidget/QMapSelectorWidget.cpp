#include "QMapSelectorWidget.hpp"


QMapSelectorWidget::QMapSelectorWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _timer(new QTimer(this)),
    _node(node_)
{
   //ui.setupUi(this);

    this->initSubscriber();
    this->setupUI();

    //connect(_timer,QLineEdit(timeout()), this,)
}

//namespace{
//    const QString defaultMapsPath = QDir::
//}

void QMapSelectorWidget::initSubscriber(void)
{
    if(_node)
    {
        _load_map_client = _node->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    } 
    else
    {
        assert(false && "Error: GUI node is null");
    }                                                                
    }

void QMapSelectorWidget::setupUI(void)
{

    _ui.setupUi(this);
    _ui.label_preview->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    _ui.label_preview->setAlignment(Qt::AlignCenter);
    _ui.lineEdit_path->setText(DEFAULT_MAPS_PATH);

    connect(_ui.newMap_Button, &QPushButton::clicked, this, &QMapSelectorWidget::onNewMapClicked);
    connect(_ui.pushButton_browse, &QPushButton::clicked, this, &QMapSelectorWidget::browse);
    connect(_ui.comboBox_maps, &QComboBox::currentTextChanged,this, &QMapSelectorWidget::onComboTextChanged);


    _timer->setInterval(REFRESH_DELAY_MS);
    connect(_timer, &QTimer::timeout, this, &QMapSelectorWidget::refreshMaps);
    _timer->start();

    refreshMaps(); 
}

void QMapSelectorWidget::browse(void)
{
    const QString start_dir = _ui.lineEdit_path->text().isEmpty() ? QDir::homePath(): _ui.lineEdit_path->text();
    const QString folder = QFileDialog::getExistingDirectory(this, tr("Choisir le dossier des maps"), start_dir);

    if (!folder.isEmpty()) _ui.lineEdit_path->setText(folder);
}

QStringList QMapSelectorWidget::listMaps(void) const
{
    const QDir dir(_ui.lineEdit_path->text().trimmed());
    if (!dir.exists())
        return {};

    return dir.entryList(MAP_FILTERS, QDir::Files, QDir::Name | QDir::IgnoreCase);
}

void QMapSelectorWidget::refreshMaps(void)
{

    const QStringList new_items = listMaps();

    QStringList current_items;
    for (int i = 0; i < _ui.comboBox_maps->count(); ++i)
        current_items << _ui.comboBox_maps->itemText(i);

    
    if (new_items == current_items)
        return;

    const QString previous = _ui.comboBox_maps->currentText();

    _ui.comboBox_maps->blockSignals(true);
    _ui.comboBox_maps->clear();
    _ui.comboBox_maps->addItems(new_items);

    const int index = _ui.comboBox_maps->findText(previous);
    if (index >= 0)
        _ui.comboBox_maps->setCurrentIndex(index);
    _ui.comboBox_maps->blockSignals(false);

    if (_ui.comboBox_maps->currentText() != previous)
        onComboTextChanged(_ui.comboBox_maps->currentText());
}

void QMapSelectorWidget::onComboTextChanged(const QString& name_)
{

    if (name_.isEmpty())
    {
        _preview_pixmap = QPixmap();
        _ui.label_preview->clear();
        return;
    }

    const QString path = QDir(_ui.lineEdit_path->text().trimmed()).filePath(name_);
    updatePreview(path);         
    emit mapChanged(path);
}

QString QMapSelectorWidget::currentMapPath(void) const
{
    const QString name = _ui.comboBox_maps->currentText();
    if (name.isEmpty())
        return {};
    return QDir(_ui.lineEdit_path->text().trimmed()).filePath(name);
}

void QMapSelectorWidget::updatePreview(const QString& path_)
{

    QImageReader reader(path_);
    const QImage img = reader.read();
    if (img.isNull())
    {
        _ui.label_preview->setText(tr("Aperçu indisponible"));
        return;
    }

    _preview_pixmap = QPixmap::fromImage(img);
    _ui.label_preview->setPixmap(_preview_pixmap.scaled(_ui.label_preview->size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
}

void QMapSelectorWidget::rescalePreview(void)
{
    if (_preview_pixmap.isNull())
        return;

    _ui.label_preview->setPixmap(
        _preview_pixmap.scaled(_ui.label_preview->size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
}

void QMapSelectorWidget::resizeEvent(QResizeEvent* event_)
{
    QWidget::resizeEvent(event_);
    rescalePreview();   
}

void QMapSelectorWidget::onNewMapClicked(void)
{
    using LoadMap = nav2_msgs::srv::LoadMap;

    const QString pgm_path = currentMapPath();
    if (pgm_path.isEmpty())
    {
        emit loadMapFinished(false, tr("Aucune map sélectionnée"));
        return;
    }


    const QFileInfo info(pgm_path);
    const QString yaml_path = info.absolutePath() + "/" + info.completeBaseName() + ".yaml";
    if (!QFileInfo::exists(yaml_path))
    {
        emit loadMapFinished(false, tr("Fichier .yaml introuvable : ") + yaml_path);
        return;
    }

    if (!_load_map_client->service_is_ready())
    {
        emit loadMapFinished(false, tr("Service load_map indisponible"));
        return;
    }

    auto request = std::make_shared<LoadMap::Request>();
    request->map_url = yaml_path.toStdString();

    _load_map_client->async_send_request(
        request,
        [this, yaml_path](rclcpp::Client<LoadMap>::SharedFuture future_)
        {
            const bool ok = (future_.get()->result == LoadMap::Response::RESULT_SUCCESS);
            emit loadMapFinished(ok, ok ? tr("Map chargée : ") + yaml_path: tr("Échec du chargement : ") + yaml_path);
        });
}