
#ifndef OSGEARTHCORE_EXPORT_H
#define OSGEARTHCORE_EXPORT_H

#ifdef OSGEARTHCORE_STATIC_DEFINE
#  define OSGEARTHCORE_EXPORT
#  define OSGEARTHCORE_NO_EXPORT
#else
#  ifndef OSGEARTHCORE_EXPORT
#    ifdef core_EXPORTS
        /* We are building this library */
#      define OSGEARTHCORE_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define OSGEARTHCORE_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef OSGEARTHCORE_NO_EXPORT
#    define OSGEARTHCORE_NO_EXPORT 
#  endif
#endif

#ifndef OSGEARTHCORE_DEPRECATED
#  define OSGEARTHCORE_DEPRECATED __declspec(deprecated)
#endif

#ifndef OSGEARTHCORE_DEPRECATED_EXPORT
#  define OSGEARTHCORE_DEPRECATED_EXPORT OSGEARTHCORE_EXPORT OSGEARTHCORE_DEPRECATED
#endif

#ifndef OSGEARTHCORE_DEPRECATED_NO_EXPORT
#  define OSGEARTHCORE_DEPRECATED_NO_EXPORT OSGEARTHCORE_NO_EXPORT OSGEARTHCORE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef OSGEARTHCORE_NO_DEPRECATED
#    define OSGEARTHCORE_NO_DEPRECATED
#  endif
#endif

#endif /* OSGEARTHCORE_EXPORT_H */
