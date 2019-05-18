#ifndef OSG_BUILDER_EXPORT
#define OSG_BUILDER_EXPORT

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif

#define OSG_BUILDER_API DLL_API
#endif // OSG_BUILDER_EXPORT