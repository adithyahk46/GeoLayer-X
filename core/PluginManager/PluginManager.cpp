#include "PluginManager.h"

#include "PluginInstaller.h"

#include <QBrush>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPluginLoader>
#include <QPushButton>
#include <QSettings>
#include <QSplitter>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <algorithm>

PluginManager* PluginManager::_instance = nullptr;

namespace
{
QString toStringOrDefault(const QJsonObject& object, const QString& key, const QString& fallback = QString())
{
	if (!object.contains(key))
	{
		return fallback;
	}
	return object.value(key).toString(fallback);
}

QStringList toStringList(const QJsonObject& object, const QString& key)
{
	QStringList output;
	const QJsonArray values = object.value(key).toArray();
	for (const QJsonValue& value : values)
	{
		output.append(value.toString());
	}
	return output;
}

PluginManager::FilterCategory filterFromRow(int row)
{
	switch (row)
	{
	case 1:
		return PluginManager::FilterCategory::Installed;
	case 2:
		return PluginManager::FilterCategory::NotInstalled;
	case 3:
		return PluginManager::FilterCategory::Upgradeable;
	case 4:
		return PluginManager::FilterCategory::Invalid;
	default:
		return PluginManager::FilterCategory::All;
	}
}
}

class PluginManagerDialog final : public QDialog
{
public:
	explicit PluginManagerDialog(PluginManager* manager, QWidget* parent = nullptr)
		: QDialog(parent)
		, _manager(manager)
	{
		setWindowTitle(QStringLiteral("Plugins | GeoLayer X"));
		resize(1180, 760);

		auto* rootLayout = new QVBoxLayout(this);
		rootLayout->setContentsMargins(8, 8, 8, 8);

		auto* splitter = new QSplitter(Qt::Horizontal, this);
		rootLayout->addWidget(splitter, 1);

		auto* leftPane = new QWidget(splitter);
		auto* leftLayout = new QVBoxLayout(leftPane);
		leftLayout->setContentsMargins(0, 0, 0, 0);
		_categoryList = new QListWidget(leftPane);
		_categoryList->addItems(
			{QStringLiteral("All"),
			 QStringLiteral("Installed"),
			 QStringLiteral("Not installed"),
			 QStringLiteral("Upgradeable"),
			 QStringLiteral("Invalid")});
		_categoryList->setCurrentRow(0);
		leftLayout->addWidget(_categoryList);

		auto* centerPane = new QWidget(splitter);
		auto* centerLayout = new QVBoxLayout(centerPane);
		centerLayout->setContentsMargins(0, 0, 0, 0);

		_searchEdit = new QLineEdit(centerPane);
		_searchEdit->setPlaceholderText(QStringLiteral("Search plugins..."));
		centerLayout->addWidget(_searchEdit);

		_pluginTree = new QTreeWidget(centerPane);
		_pluginTree->setRootIsDecorated(false);
		_pluginTree->setAlternatingRowColors(true);
		_pluginTree->setUniformRowHeights(true);
		_pluginTree->setColumnCount(3);
		_pluginTree->setHeaderLabels({QStringLiteral("Plugin"), QStringLiteral("Version"), QStringLiteral("Status")});
		_pluginTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
		_pluginTree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
		_pluginTree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
		centerLayout->addWidget(_pluginTree, 1);

		auto* rightPane = new QWidget(splitter);
		auto* rightLayout = new QVBoxLayout(rightPane);
		rightLayout->setContentsMargins(0, 0, 0, 0);

		auto* detailsLabel = new QLabel(QStringLiteral("Plugin Details"), rightPane);
		detailsLabel->setStyleSheet(QStringLiteral("font-weight: 600;"));
		rightLayout->addWidget(detailsLabel);

		_detailsBrowser = new QTextBrowser(rightPane);
		rightLayout->addWidget(_detailsBrowser, 1);

		splitter->setStretchFactor(0, 0);
		splitter->setStretchFactor(1, 2);
		splitter->setStretchFactor(2, 2);
		splitter->setSizes({180, 520, 480});

		auto* buttons = new QDialogButtonBox(this);
		_installZipButton = buttons->addButton(QStringLiteral("Install from ZIP"), QDialogButtonBox::ActionRole);
		_reloadButton = buttons->addButton(QStringLiteral("Reload"), QDialogButtonBox::ActionRole);
		buttons->addButton(QDialogButtonBox::Close);
		rootLayout->addWidget(buttons);

		connect(_categoryList, &QListWidget::currentRowChanged, this, &PluginManagerDialog::refreshPluginList);
		connect(_searchEdit, &QLineEdit::textChanged, this, &PluginManagerDialog::refreshPluginList);
		connect(_pluginTree, &QTreeWidget::currentItemChanged, this, &PluginManagerDialog::updateDetails);
		connect(_pluginTree, &QTreeWidget::itemChanged, this, &PluginManagerDialog::onItemChanged);
		connect(_installZipButton, &QPushButton::clicked, this, &PluginManagerDialog::installFromZip);
		connect(_reloadButton, &QPushButton::clicked, this, &PluginManagerDialog::reloadPlugins);
		connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::close);
		connect(_manager, &PluginManager::pluginsChanged, this, &PluginManagerDialog::refreshPluginList);

		refreshPluginList();
	}

private:
	void refreshPluginList()
	{
		_suppressItemEvents = true;

		_pluginTree->clear();
		const PluginManager::FilterCategory filter = filterFromRow(_categoryList->currentRow());
		const QList<PluginManager::PluginInfo> entries = _manager->plugins(filter, _searchEdit->text());

		for (const PluginManager::PluginInfo& entry : entries)
		{
			auto* item = new QTreeWidgetItem(_pluginTree);
			item->setText(0, entry.name.isEmpty() ? entry.id : entry.name);
			item->setText(1, entry.version);

			QString status = entry.valid ? QStringLiteral("Available") : QStringLiteral("Invalid");
			if (entry.loaded)
			{
				status = QStringLiteral("Loaded");
			}
			else if (entry.installed)
			{
				status = QStringLiteral("Installed");
			}
			if (entry.hasUpdate)
			{
				status += QStringLiteral(" | Update");
			}

			item->setText(2, status);
			item->setData(0, Qt::UserRole, entry.id);
			item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
			item->setCheckState(0, entry.enabled ? Qt::Checked : Qt::Unchecked);

			if (!entry.valid)
			{
				item->setForeground(0, QBrush(Qt::red));
			}
		}

		_suppressItemEvents = false;
		if (_pluginTree->topLevelItemCount() > 0)
		{
			_pluginTree->setCurrentItem(_pluginTree->topLevelItem(0));
		}
		else
		{
			_detailsBrowser->setHtml(QStringLiteral("<h3>No plugins found</h3><p>Adjust search or filter settings.</p>"));
		}
	}

	void updateDetails(QTreeWidgetItem* current, QTreeWidgetItem*)
	{
		if (!current)
		{
			_detailsBrowser->clear();
			return;
		}

		const QString pluginId = current->data(0, Qt::UserRole).toString();
		const PluginManager::PluginInfo entry = _manager->plugin(pluginId);

		const QString html = QStringLiteral(
								 "<h2>%1</h2>"
								 "<p><b>ID:</b> %2<br/>"
								 "<b>Version:</b> %3<br/>"
								 "<b>Category:</b> %4<br/>"
								 "<b>Vendor:</b> %5<br/>"
								 "<b>Status:</b> %6</p>"
								 "<p>%7</p>"
								 "<p><b>Dependencies:</b> %8</p>"
								 "%9")
								 .arg(
									 entry.name.isEmpty() ? entry.id : entry.name,
									 entry.id,
									 entry.version.isEmpty() ? QStringLiteral("unknown") : entry.version,
									 entry.category.isEmpty() ? QStringLiteral("general") : entry.category,
									 entry.vendor.isEmpty() ? QStringLiteral("unknown") : entry.vendor,
									 entry.valid ? (entry.loaded ? QStringLiteral("Loaded") : QStringLiteral("Installed"))
												 : QStringLiteral("Invalid"),
									 entry.description,
									 entry.dependencies.isEmpty() ? QStringLiteral("none") : entry.dependencies.join(QStringLiteral(", ")),
									 entry.error.isEmpty() ? QString() : QStringLiteral("<p style='color:#c0392b;'><b>Error:</b> %1</p>").arg(entry.error));

		_detailsBrowser->setHtml(html);
	}

	void onItemChanged(QTreeWidgetItem* item, int column)
	{
		if (_suppressItemEvents || column != 0 || !item)
		{
			return;
		}

		const QString pluginId = item->data(0, Qt::UserRole).toString();
		const bool enable = item->checkState(0) == Qt::Checked;

		QString errorMessage;
		if (!_manager->setEnabled(pluginId, enable, &errorMessage))
		{
			QMessageBox::warning(this, QStringLiteral("Plugin Manager"), errorMessage);

			_suppressItemEvents = true;
			item->setCheckState(0, enable ? Qt::Unchecked : Qt::Checked);
			_suppressItemEvents = false;
			return;
		}

		refreshPluginList();
	}

	void installFromZip()
	{
		const QString packagePath = QFileDialog::getOpenFileName(
			this,
			QStringLiteral("Install Plugin Package"),
			QString(),
			QStringLiteral("Plugin packages (*.zip);;All files (*.*)"));

		if (packagePath.isEmpty())
		{
			return;
		}

		QString errorMessage;
		if (!_manager->installFromZip(packagePath, &errorMessage))
		{
			QMessageBox::warning(this, QStringLiteral("Plugin Installation"), errorMessage);
			return;
		}

		QMessageBox::information(this, QStringLiteral("Plugin Installation"), QStringLiteral("Plugin package installed."));
	}

	void reloadPlugins()
	{
		_manager->scanPlugins();
	}

private:
	PluginManager* _manager = nullptr;
	QListWidget* _categoryList = nullptr;
	QLineEdit* _searchEdit = nullptr;
	QTreeWidget* _pluginTree = nullptr;
	QTextBrowser* _detailsBrowser = nullptr;
	QPushButton* _installZipButton = nullptr;
	QPushButton* _reloadButton = nullptr;
	bool _suppressItemEvents = false;
};

PluginManager* PluginManager::instance()
{
	if (!_instance)
	{
		_instance = new PluginManager();
	}
	return _instance;
}

PluginManager::PluginManager(QObject* parent)
	: QObject(parent)
{
	_pluginSearchPaths = {
		defaultPluginDir(),
		QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("plugins")),
		QDir::cleanPath(QCoreApplication::applicationDirPath() + QStringLiteral("/../plugins"))};

	loadEnabledState();
	scanPlugins();
}

PluginManager::~PluginManager()
{
	clearRuntimeLoaders();
}

void PluginManager::setHostContext(const PluginHostContext& context)
{
	_context = context;
}

PluginHostContext PluginManager::hostContext() const
{
	return _context;
}

void PluginManager::setPluginSearchPaths(const QStringList& paths)
{
	_pluginSearchPaths = paths;
	scanPlugins();
}

QStringList PluginManager::pluginSearchPaths() const
{
	return _pluginSearchPaths;
}

void PluginManager::addPluginSearchPath(const QString& path)
{
	if (path.trimmed().isEmpty())
	{
		return;
	}
	if (!_pluginSearchPaths.contains(path))
	{
		_pluginSearchPaths.append(path);
	}
}

void PluginManager::scanPlugins()
{
	clearRuntimeLoaders();
	clearDiscoveredPlugins();

	discoverFromCatalog();
	discoverFromFilesystem();

	const QStringList knownIds = _plugins.keys();
	const QSet<QString> ids = QSet<QString>(knownIds.begin(), knownIds.end());
	for (const QString& enabledId : std::as_const(_enabledIds))
	{
		if (ids.contains(enabledId))
		{
			QString errorMessage;
			if (!loadPlugin(enabledId, &errorMessage))
			{
				emit pluginError(enabledId, errorMessage);
			}
		}
	}

	emit pluginsChanged();
}

QList<PluginManager::PluginInfo> PluginManager::plugins(FilterCategory filter, const QString& searchText) const
{
	QList<PluginInfo> result;
	const QString query = searchText.trimmed();

	for (auto it = _plugins.constBegin(); it != _plugins.constEnd(); ++it)
	{
		const PluginInfo& info = it->info;

		bool matchesFilter = false;
		switch (filter)
		{
		case FilterCategory::All:
			matchesFilter = true;
			break;
		case FilterCategory::Installed:
			matchesFilter = info.installed;
			break;
		case FilterCategory::NotInstalled:
			matchesFilter = !info.installed && info.available;
			break;
		case FilterCategory::Upgradeable:
			matchesFilter = info.hasUpdate;
			break;
		case FilterCategory::Invalid:
			matchesFilter = !info.valid;
			break;
		}

		if (!matchesFilter)
		{
			continue;
		}

		if (!query.isEmpty())
		{
			const QString haystack = QStringLiteral("%1 %2 %3 %4 %5")
										.arg(info.id, info.name, info.description, info.category, info.vendor)
										.toLower();
			if (!haystack.contains(query.toLower()))
			{
				continue;
			}
		}

		result.append(info);
	}

	std::sort(result.begin(), result.end(), [](const PluginInfo& lhs, const PluginInfo& rhs) {
		return QString::localeAwareCompare(lhs.name, rhs.name) < 0;
	});
	return result;
}

PluginManager::PluginInfo PluginManager::plugin(const QString& pluginId) const
{
	const QString key = normalizePluginId(pluginId);
	const auto it = _plugins.find(key);
	if (it == _plugins.end())
	{
		return {};
	}
	return it->info;
}

bool PluginManager::setEnabled(const QString& pluginId, bool enabled, QString* errorMessage)
{
	const QString key = normalizePluginId(pluginId);
	auto it = _plugins.find(key);
	if (it == _plugins.end())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Plugin not found: %1").arg(pluginId);
		}
		return false;
	}

	bool success = true;
	if (enabled)
	{
		success = loadPlugin(key, errorMessage);
		if (success)
		{
			_enabledIds.insert(key);
		}
	}
	else
	{
		success = unloadPlugin(key, errorMessage);
		_enabledIds.remove(key);
	}

	if (success)
	{
		it->info.enabled = enabled;
		saveEnabledState();
		emit pluginsChanged();
	}

	return success;
}

bool PluginManager::loadPlugin(const QString& pluginId, QString* errorMessage)
{
	const QString key = normalizePluginId(pluginId);
	auto it = _plugins.find(key);
	if (it == _plugins.end())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Plugin not found: %1").arg(pluginId);
		}
		return false;
	}

	RuntimePlugin& plugin = it.value();
	if (plugin.info.loaded)
	{
		return true;
	}

	if (!plugin.info.valid)
	{
		if (errorMessage)
		{
			*errorMessage = plugin.info.error.isEmpty() ? QStringLiteral("Plugin metadata is invalid.") : plugin.info.error;
		}
		return false;
	}

	if (plugin.info.filePath.isEmpty() || !QFileInfo::exists(plugin.info.filePath))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Plugin binary not found for %1").arg(pluginId);
		}
		return false;
	}

	auto* loader = new QPluginLoader(plugin.info.filePath, this);
	QObject* object = loader->instance();
	if (!object)
	{
		const QString loaderError = loader->errorString();
		delete loader;
		if (errorMessage)
		{
			*errorMessage = loaderError;
		}
		plugin.info.error = loaderError;
		plugin.info.loaded = false;
		emit pluginError(pluginId, loaderError);
		return false;
	}

	PluginInterface* iface = qobject_cast<PluginInterface*>(object);
	if (!iface)
	{
		const QString message = QStringLiteral("Plugin does not implement PluginInterface: %1").arg(plugin.info.filePath);
		loader->unload();
		delete loader;
		if (errorMessage)
		{
			*errorMessage = message;
		}
		plugin.info.error = message;
		plugin.info.loaded = false;
		emit pluginError(pluginId, message);
		return false;
	}

	QString lifecycleError;
	if (!iface->load(_context, &lifecycleError) || !iface->activate(&lifecycleError))
	{
		loader->unload();
		delete loader;

		if (errorMessage)
		{
			*errorMessage = lifecycleError.isEmpty() ? QStringLiteral("Plugin activation failed") : lifecycleError;
		}
		plugin.info.error = lifecycleError;
		plugin.info.loaded = false;
		emit pluginError(pluginId, lifecycleError);
		return false;
	}

	plugin.loader = loader;
	plugin.iface = iface;
	plugin.info.loaded = true;
	plugin.info.enabled = true;
	plugin.info.error.clear();

	emit pluginStateChanged(key, true, true);
	return true;
}

bool PluginManager::unloadPlugin(const QString& pluginId, QString* errorMessage)
{
	const QString key = normalizePluginId(pluginId);
	auto it = _plugins.find(key);
	if (it == _plugins.end())
	{
		return true;
	}

	RuntimePlugin& plugin = it.value();
	if (!plugin.loader)
	{
		plugin.info.loaded = false;
		plugin.info.enabled = false;
		emit pluginStateChanged(key, false, false);
		return true;
	}

	if (plugin.iface)
	{
		QString reason;
		if (!plugin.iface->canUnload(&reason))
		{
			if (errorMessage)
			{
				*errorMessage = reason.isEmpty() ? QStringLiteral("Plugin refuses to unload.") : reason;
			}
			return false;
		}

		QString lifecycleError;
		plugin.iface->deactivate(&lifecycleError);
		plugin.iface->unload(&lifecycleError);
	}

	if (!plugin.loader->unload())
	{
		if (errorMessage)
		{
			*errorMessage = plugin.loader->errorString();
		}
		return false;
	}

	delete plugin.loader;
	plugin.loader = nullptr;
	plugin.iface = nullptr;
	plugin.info.loaded = false;
	plugin.info.enabled = false;

	emit pluginStateChanged(key, false, false);
	return true;
}

bool PluginManager::executePluginCommand(
	const QString& pluginId,
	const QString& commandId,
	const QVariantMap& arguments,
	QVariantMap* result,
	QString* errorMessage)
{
	const QString key = normalizePluginId(pluginId);
	auto it = _plugins.find(key);
	if (it == _plugins.end())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Plugin not found: %1").arg(pluginId);
		}
		return false;
	}

	RuntimePlugin& plugin = it.value();
	if (!plugin.info.loaded)
	{
		if (!loadPlugin(key, errorMessage))
		{
			return false;
		}
	}

	if (!plugin.iface)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Plugin interface unavailable for %1").arg(pluginId);
		}
		return false;
	}

	return plugin.iface->executeCommand(commandId, arguments, result, errorMessage);
}

bool PluginManager::installFromZip(const QString& zipPath, QString* errorMessage)
{
	if (!PluginInstaller::installPackage(zipPath, defaultPluginDir(), errorMessage))
	{
		return false;
	}

	scanPlugins();
	return true;
}

void PluginManager::showManagerDialog(QWidget* parent)
{
	if (!_dialog)
	{
		_dialog = new PluginManagerDialog(this, parent);
		_dialog->setAttribute(Qt::WA_DeleteOnClose, true);
		connect(_dialog, &QObject::destroyed, this, [this]() {
			_dialog = nullptr;
		});
	}

	_dialog->show();
	_dialog->raise();
	_dialog->activateWindow();
}

void PluginManager::clearRuntimeLoaders()
{
	for (auto it = _plugins.begin(); it != _plugins.end(); ++it)
	{
		RuntimePlugin& plugin = it.value();
		if (!plugin.loader)
		{
			continue;
		}

		if (plugin.iface)
		{
			QString lifecycleError;
			plugin.iface->deactivate(&lifecycleError);
			plugin.iface->unload(&lifecycleError);
		}

		plugin.loader->unload();
		delete plugin.loader;

		plugin.loader = nullptr;
		plugin.iface = nullptr;
		plugin.info.loaded = false;
	}
}

void PluginManager::clearDiscoveredPlugins()
{
	_plugins.clear();
}

void PluginManager::discoverFromFilesystem()
{
	QStringList searchPaths = _pluginSearchPaths;
	searchPaths.removeDuplicates();

	QStringList filePatterns;
#ifdef Q_OS_WIN
	filePatterns << QStringLiteral("*.dll");
#elif defined(Q_OS_MAC)
	filePatterns << QStringLiteral("*.dylib") << QStringLiteral("*.so");
#else
	filePatterns << QStringLiteral("*.so");
#endif

	for (const QString& pluginPath : searchPaths)
	{
		const QDir dir(pluginPath);
		if (!dir.exists())
		{
			continue;
		}

		const QFileInfoList files = dir.entryInfoList(filePatterns, QDir::Files | QDir::Readable, QDir::Name);
		for (const QFileInfo& fileInfo : files)
		{
			QPluginLoader loader(fileInfo.absoluteFilePath());
			const QJsonObject meta = loader.metaData();
			const QJsonObject pluginMeta = meta.value(QStringLiteral("MetaData")).toObject();
			const QString iid = meta.value(QStringLiteral("IID")).toString();

			PluginInfo info;
			info.filePath = fileInfo.absoluteFilePath();
			info.installed = true;
			info.available = true;
			info.valid = (iid == QLatin1String(GEOLAYER_PLUGIN_INTERFACE_IID));

			info.id = normalizePluginId(toStringOrDefault(pluginMeta, QStringLiteral("id"), fileInfo.completeBaseName()));
			info.name = toStringOrDefault(pluginMeta, QStringLiteral("name"), fileInfo.completeBaseName());
			info.version = toStringOrDefault(pluginMeta, QStringLiteral("version"), QStringLiteral("0.0.0"));
			info.description = toStringOrDefault(pluginMeta, QStringLiteral("description"));
			info.category = toStringOrDefault(pluginMeta, QStringLiteral("category"), QStringLiteral("general"));
			info.vendor = toStringOrDefault(pluginMeta, QStringLiteral("vendor"));
			info.tags = toStringList(pluginMeta, QStringLiteral("tags"));
			info.dependencies = toStringList(pluginMeta, QStringLiteral("dependencies"));
			info.enabled = _enabledIds.contains(info.id);

			if (!info.valid)
			{
				info.error = QStringLiteral("Invalid plugin IID: %1").arg(iid);
			}

			if (_plugins.contains(info.id))
			{
				continue;
			}

			RuntimePlugin runtime;
			runtime.info = info;
			_plugins.insert(info.id, runtime);
		}
	}
}

void PluginManager::discoverFromCatalog()
{
	QStringList searchPaths = _pluginSearchPaths;
	searchPaths.append(defaultPluginDir());
	searchPaths.removeDuplicates();

	for (const QString& path : searchPaths)
	{
		const QString catalogPath = QDir(path).filePath(QStringLiteral("catalog.json"));
		QFile catalogFile(catalogPath);
		if (!catalogFile.exists() || !catalogFile.open(QIODevice::ReadOnly))
		{
			continue;
		}

		const QJsonDocument doc = QJsonDocument::fromJson(catalogFile.readAll());
		const QJsonArray pluginsArray = doc.object().value(QStringLiteral("plugins")).toArray();

		for (const QJsonValue& value : pluginsArray)
		{
			const QJsonObject object = value.toObject();
			const QString id = normalizePluginId(toStringOrDefault(object, QStringLiteral("id")));
			if (id.isEmpty())
			{
				continue;
			}

			RuntimePlugin runtime;
			if (_plugins.contains(id))
			{
				runtime = _plugins.value(id);
			}

			runtime.info.id = id;
			runtime.info.name = toStringOrDefault(object, QStringLiteral("name"), runtime.info.name);
			runtime.info.version = toStringOrDefault(object, QStringLiteral("version"), runtime.info.version);
			runtime.info.description = toStringOrDefault(object, QStringLiteral("description"), runtime.info.description);
			runtime.info.category = toStringOrDefault(object, QStringLiteral("category"), runtime.info.category);
			runtime.info.vendor = toStringOrDefault(object, QStringLiteral("vendor"), runtime.info.vendor);
			runtime.info.tags = toStringList(object, QStringLiteral("tags"));
			runtime.info.dependencies = toStringList(object, QStringLiteral("dependencies"));
			runtime.info.available = true;
			runtime.info.hasUpdate = object.value(QStringLiteral("hasUpdate")).toBool(false);
			runtime.info.enabled = _enabledIds.contains(id);

			_plugins.insert(id, runtime);
		}
	}
}

void PluginManager::saveEnabledState()
{
	QSettings settings;
	QStringList ids = QStringList(_enabledIds.begin(), _enabledIds.end());
	ids.sort(Qt::CaseInsensitive);
	settings.setValue(QStringLiteral("PluginManager/enabledPlugins"), ids);
}

void PluginManager::loadEnabledState()
{
	const QSettings settings;
	const QStringList ids = settings.value(QStringLiteral("PluginManager/enabledPlugins")).toStringList();
	_enabledIds = QSet<QString>(ids.begin(), ids.end());
}

QString PluginManager::normalizePluginId(const QString& value) const
{
	QString id = value.trimmed().toLower();
	id.replace(' ', '.');
	return id;
}

QString PluginManager::defaultPluginDir() const
{
	return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("plugins"));
}

