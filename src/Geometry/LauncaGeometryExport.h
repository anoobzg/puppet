#ifndef LAUNCA_GEOMETRY_EXPORT
#define LAUNCA_GEOMETRY_EXPORT

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif

#define LAUNCA_GEOMETRY_API DLL_API
#endif // LAUNCA_BASE_EXPORT