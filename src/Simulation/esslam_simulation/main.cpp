#include <Windows.h>
#include "runslam.h"

//1 type, 2 config, 3 append
int main(int argc, char* argv[])
{
	if (argc < 2) return EXIT_FAILURE;

	HMODULE dll = ::LoadLibrary("EsslamDLL.dll");
	if (!dll) return EXIT_FAILURE;

	CreateSlamFunc create = (CreateSlamFunc)::GetProcAddress(dll, "CreateSlam");
	DestroySlamFunc destroy = (DestroySlamFunc)::GetProcAddress(dll, "DestroySlam");
	if (!create || !destroy)
	{
		::FreeLibrary(dll);
		return EXIT_FAILURE;
	}

	esslam::IESSlam* slam = NULL;
	int exit_code = EXIT_SUCCESS;
	if (!strcmp(argv[1], "T"))
	{
		slam = create(argv[1]);
		if(slam) exit_code = run_t_slam(*slam, argc, argv);
	}
	else
	{
		slam = create(NULL);
		if(slam) exit_code = run_slam(*slam, argc, argv);
	}
	
	destroy(slam);
	::FreeLibrary(dll);
	return exit_code;
}