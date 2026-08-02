TEMPLATE = subdirs
CONFIG += ordered

SUBDIRS += \
    core \
    app \
    app_launcher \
    plugins

app.file = app/app.pro
app_launcher.file = app/app_launcher.pro
