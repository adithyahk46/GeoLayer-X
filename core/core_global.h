#ifndef CORE_GLOBAL_H
#define CORE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(GEOLAYER_CORE_LIBRARY)
#  define GEOLAYERCORE_EXPORT Q_DECL_EXPORT
#else
#  define GEOLAYERCORE_EXPORT Q_DECL_IMPORT
#endif

#endif // CORE_GLOBAL_H
