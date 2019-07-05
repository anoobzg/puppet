#include <Windows.h>
#include "runslam.h"

int main(int argc, char* argv[])
{
	HMODULE dll = ::LoadLibrary("EsslamDLL.dll");
	CreateSlamFunc create = (CreateSlamFunc)::GetProcAddress(dll, "CreateSlam");

	esslam::IESSlam* slam = NULL;
	if (create) slam = create();

	if (!slam) return EXIT_FAILURE;

	int exit_code = run_slam(*slam, argc, argv);
	DestroySlamFunc destroy = (DestroySlamFunc)::GetProcAddress(dll, "DestroySlam");
	if (destroy) destroy(slam);

	::FreeLibrary(dll);
	return exit_code;
}