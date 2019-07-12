#pragma
#include "slam_data.h"

namespace esslam
{
	enum FASDebugType
	{
		NOTHING,
		OUTPUT_TIME,
		SAVE_INTERMEDIATE
	};

	enum FASCalibType
	{
		FORWARD_PROPAGATION,
		BACK_PROPAGATION
	};

	struct FASParameters
	{
		std::string calib_diretory;
		int width;
		int height;
		bool cal_phase;
		int move_num;
	};

	class IESStereoFAS
	{
	public:
		virtual ~IESStereoFAS() {}

		virtual bool Prepare(const FASParameters& parameters) = 0;
		virtual bool Compute(unsigned char* lefts[5]/*in*/, unsigned char* rights[5]/*in*/,
			int buffer_size/*in*/, BuildModelData* data/*out*/) = 0;
		virtual void SetCalibType(FASCalibType type = BACK_PROPAGATION) = 0;
		virtual void SetModulationThreshold(float modulationTd = 3.0f) = 0;
		virtual void SetPhaseThreshold(float phaseTd = 4.0f * 2.0f * 3.1415926f / 20.0f) = 0;
		virtual void SetZRange(float zmin = 400.0f, float zmax = 700.0f) = 0;
	};
}

typedef esslam::IESStereoFAS* (*CreateStereoFASFunc)();
typedef void(*DestroyStereoFASFunc)(esslam::IESStereoFAS* fas);